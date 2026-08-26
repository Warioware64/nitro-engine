#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
#
# Copyright (c) 2026 Warioware64
#
# AnimMat (.neaanimmat) binary format reader/writer, plus a Python mirror of the
# C evaluator. The C runtime in NEAAnimMat.c reads exactly the layout this
# module writes, and evaluates exactly the way `evaluate_track` below does --
# keep the two in lockstep.

"""animmat_format.py -- read/write .neaanimmat files, and evaluate them.

Two file versions exist and both load:

    VERSION 1 -- one material, tracks straight after the header. Every track is
    keyframed. This is what the Blender exporter and the old gen_animmat.py
    wrote, and it still loads; it parses into a single unnamed target.

    VERSION 2 -- tracks grouped under material targets addressed by name, so one
    file can drive every material in a model. Each track also says how its
    values are stored, which is where most of the runtime cost went.

Binary layout, version 2 (little-endian):

    HEADER (16 bytes)
        magic         u32   'AMKF' (0x464B4D41)
        version       u32   2
        num_targets   u16
        num_frames    u16
        reserved      u8[4]

    TARGET HEADER (40 bytes each, num_targets of them)
        name          char[32]  null-padded material name; empty means
                                "the only target", matched to anything
        num_tracks    u16
        pad           u16
        track_offset  u32       byte offset of this target's track headers

    TRACK HEADER (12 bytes each)
        type          u8    NEA_AnimMatTrackType
        interp        u8    NEA_AnimMatInterp (KEYS storage only)
        storage       u8    NEA_AnimMatStorage
        reserved      u8
        count         u16   KEYS: keyframes. BAKED: frames. CONST: 0
        pad           u16
        data          u32   KEYS/BAKED: byte offset. CONST: the value itself

    KEYFRAME (8 bytes each)
        frame         u16
        pad           u16
        value         u32

    BAKED VALUE (2 bytes each)
        value         s16   1.10.5 for the four texture translate/scale
                            tracks, plain 16 bit otherwise

The in-memory form is plain dicts, so it serialises to JSON without help:

    {
      "num_frames": 60,
      "targets": [
        {"name": "water", "tracks": [
            {"type": 8, "storage": 2, "values": [...]},
            {"type": 0, "storage": 0, "interp": 1, "keys": [[0, 31], [30, 8]]},
            {"type": 2, "storage": 1, "value": 128},
        ]}
      ]
    }
"""

import struct

MAGIC = 0x464B4D41  # 'AMKF'
VERSION_1 = 1
VERSION_2 = 2

MAX_TRACKS = 16
MAX_KEYFRAMES = 64
MAX_TARGETS = 16
NAME_LEN = 32

HEADER_SIZE = 16
TARGET_HEADER_SIZE = 40
TRACK_HEADER_SIZE = 12
KEYFRAME_SIZE = 8

# ---------------------------------------------------------------------------
# Track types (must match NEA_AnimMatTrackType)
# ---------------------------------------------------------------------------

ALPHA             = 0
LIGHTS            = 1
CULLING           = 2
COLOR             = 3
DIFFUSE_AMBIENT   = 4
SPECULAR_EMISSION = 5
MATERIAL_SWAP     = 6
POLYID            = 7
TEX_SCROLL_X      = 8
TEX_SCROLL_Y      = 9
TEX_ROTATE        = 10
TEX_SCALE_X       = 11
TEX_SCALE_Y       = 12
TEXPAL_SWAP       = 13

TRACK_NAMES = {
    ALPHA:             "Alpha",
    LIGHTS:            "Lights",
    CULLING:           "Culling",
    COLOR:             "Vertex color",
    DIFFUSE_AMBIENT:   "Diffuse/ambient",
    SPECULAR_EMISSION: "Specular/emission",
    MATERIAL_SWAP:     "Material swap",
    POLYID:            "Polygon ID",
    TEX_SCROLL_X:      "Tex scroll X",
    TEX_SCROLL_Y:      "Tex scroll Y",
    TEX_ROTATE:        "Tex rotate",
    TEX_SCALE_X:       "Tex scale X",
    TEX_SCALE_Y:       "Tex scale Y",
    TEXPAL_SWAP:       "Texture/palette swap",
}

# The four tracks that are f32 at runtime but bake as 1.10.5.
FIXED_TRACKS = (TEX_SCROLL_X, TEX_SCROLL_Y, TEX_SCALE_X, TEX_SCALE_Y)

# Packed dual-colour tracks need 32 bits, so they cannot be baked into s16.
NO_BAKE_TRACKS = (DIFFUSE_AMBIENT, SPECULAR_EMISSION)

# Tracks the hardware can only step, never interpolate.
STEP_ONLY_TRACKS = (LIGHTS, CULLING, MATERIAL_SWAP, TEXPAL_SWAP)

# ---------------------------------------------------------------------------
# Interpolation and storage (must match NEA_AnimMatInterp / NEA_AnimMatStorage)
# ---------------------------------------------------------------------------

INTERP_STEP   = 0
INTERP_LINEAR = 1

STORE_KEYS  = 0
STORE_CONST = 1
STORE_BAKED = 2

STORAGE_NAMES = {STORE_KEYS: "keys", STORE_CONST: "const", STORE_BAKED: "baked"}


class AnimMatFormatError(Exception):
    pass


# ---------------------------------------------------------------------------
# Fixed-point helpers
# ---------------------------------------------------------------------------

def f32(value):
    """float -> 1.19.12 fixed point, the runtime's native format."""
    return int(round(value * 4096.0)) & 0xFFFFFFFF


def from_f32(value):
    """1.19.12 fixed point -> float, sign-extending first."""
    v = value & 0xFFFFFFFF
    if v & 0x80000000:
        v -= 0x100000000
    return v / 4096.0


def to_1_10_5(value_f32):
    """1.19.12 -> 1.10.5, the encoding retail DS material animations use.

    Raises if the value will not fit, because silently wrapping a UV offset
    would be far harder to notice than a failed export.
    """
    v = value_f32 & 0xFFFFFFFF
    if v & 0x80000000:
        v -= 0x100000000
    # Truncating toward zero matches the runtime's `<< 7` widening exactly.
    out = v >> 7
    if out < -32768 or out > 32767:
        raise AnimMatFormatError(
            f"value {from_f32(value_f32)} is out of range for a baked "
            f"track (1.10.5 covers about -1024..1024)")
    return out


def mulf32(a, b):
    """The runtime's mulf32: a 32-bit fixed-point multiply, >> 12."""
    r = (a * b) >> 12
    # The C version works on int32, so wrap the same way.
    r &= 0xFFFFFFFF
    if r & 0x80000000:
        r -= 0x100000000
    return r


def _s32(v):
    v &= 0xFFFFFFFF
    return v - 0x100000000 if v & 0x80000000 else v


# ---------------------------------------------------------------------------
# Evaluation -- mirrors ne_animmat_eval_track() in NEAAnimMat.c
# ---------------------------------------------------------------------------

def _rgb15_lerp(c0, c1, frac):
    r0, g0, b0 = c0 & 0x1F, (c0 >> 5) & 0x1F, (c0 >> 10) & 0x1F
    r1, g1, b1 = c1 & 0x1F, (c1 >> 5) & 0x1F, (c1 >> 10) & 0x1F

    r = r0 + mulf32(r1 - r0, frac)
    g = g0 + mulf32(g1 - g0, frac)
    b = b0 + mulf32(b1 - b0, frac)

    r = max(0, min(31, r))
    g = max(0, min(31, g))
    b = max(0, min(31, b))
    return r | (g << 5) | (b << 10)


def _packed_color_lerp(a, b, frac):
    lo = _rgb15_lerp(a & 0x7FFF, b & 0x7FFF, frac)
    hi = _rgb15_lerp((a >> 16) & 0x7FFF, (b >> 16) & 0x7FFF, frac)
    lo |= a & 0x8000
    hi |= (a & 0x80000000) >> 16
    return (lo | (hi << 16)) & 0xFFFFFFFF


def _widen_baked(track_type, value):
    """s16 -> the runtime representation. Mirrors ne_animmat_widen_baked()."""
    if track_type in FIXED_TRACKS:
        return (value << 7) & 0xFFFFFFFF
    return value & 0xFFFF


def evaluate_track(track, frame_f32):
    """Evaluate one track at a frame given in 1.19.12, as the C runtime does."""
    storage = track.get("storage", STORE_KEYS)

    if storage == STORE_CONST:
        return track.get("value", 0) & 0xFFFFFFFF

    if storage == STORE_BAKED:
        values = track.get("values", [])
        if not values:
            return 0
        i = frame_f32 >> 12
        i = max(0, min(len(values) - 1, i))
        return _widen_baked(track["type"], values[i])

    keys = track.get("keys", [])
    if not keys:
        return 0
    if len(keys) == 1:
        return keys[0][1] & 0xFFFFFFFF

    frame_int = frame_f32 >> 12
    if frame_int <= keys[0][0]:
        return keys[0][1] & 0xFFFFFFFF
    if frame_int >= keys[-1][0]:
        return keys[-1][1] & 0xFFFFFFFF

    lo, hi = 0, len(keys) - 1
    while lo < hi - 1:
        mid = (lo + hi) >> 1
        if keys[mid][0] <= frame_int:
            lo = mid
        else:
            hi = mid

    if track.get("interp", INTERP_STEP) == INTERP_STEP:
        return keys[lo][1] & 0xFFFFFFFF

    span = keys[hi][0] - keys[lo][0]
    if span <= 0:
        return keys[lo][1] & 0xFFFFFFFF

    # divf32(frame - lo_frame, span), clamped to [0, 1]
    frac = ((frame_f32 - (keys[lo][0] << 12)) << 12) // (span << 12)
    frac = max(0, min(4096, frac))

    a = keys[lo][1] & 0xFFFFFFFF
    b = keys[hi][1] & 0xFFFFFFFF
    t = track["type"]

    if t == ALPHA:
        va, vb = a & 0x1F, b & 0x1F
        return max(0, min(31, va + mulf32(vb - va, frac)))
    if t == POLYID:
        va, vb = a & 0x3F, b & 0x3F
        return max(0, min(63, va + mulf32(vb - va, frac)))
    if t == COLOR:
        return _rgb15_lerp(a, b, frac)
    if t in (DIFFUSE_AMBIENT, SPECULAR_EMISSION):
        return _packed_color_lerp(a, b, frac)
    if t in FIXED_TRACKS:
        sa, sb = _s32(a), _s32(b)
        return (sa + mulf32(sb - sa, frac)) & 0xFFFFFFFF
    if t == TEX_ROTATE:
        va, vb = a & 0x1FF, b & 0x1FF
        return (va + mulf32(vb - va, frac)) & 0x1FF

    # Step for lights, culling, material swap, texture/palette swap.
    return a


def evaluate_target(target, frame):
    """Evaluate every track of a target at an integer frame.

    Returns {track_type: value}, which is all the preview needs.
    """
    return {t["type"]: evaluate_track(t, frame << 12)
            for t in target.get("tracks", [])}


# ---------------------------------------------------------------------------
# Storage selection
# ---------------------------------------------------------------------------

def track_byte_size(track, num_frames):
    """Bytes this track occupies in the file, header included."""
    storage = track.get("storage", STORE_KEYS)
    if storage == STORE_CONST:
        return TRACK_HEADER_SIZE
    if storage == STORE_BAKED:
        return TRACK_HEADER_SIZE + 2 * len(track.get("values", []))
    return TRACK_HEADER_SIZE + KEYFRAME_SIZE * len(track.get("keys", []))


def choose_storage(track, num_frames):
    """Pick the cheapest storage mode that can represent this track.

    Constant beats everything. After that it is a straight size comparison
    between the baked array and the keyframes, except that a track the runtime
    cannot bake has no choice.
    """
    t = track["type"]

    if track.get("storage") == STORE_CONST:
        return STORE_CONST

    keys = track.get("keys")
    if keys is not None:
        if len(keys) <= 1:
            return STORE_CONST
        if len(set(k[1] for k in keys)) == 1:
            return STORE_CONST
        if t in NO_BAKE_TRACKS:
            return STORE_KEYS
        # Two bytes a frame against eight a keyframe.
        return STORE_BAKED if 2 * num_frames < KEYFRAME_SIZE * len(keys) \
            else STORE_KEYS

    return track.get("storage", STORE_KEYS)


def bake_track(track, num_frames):
    """Turn a keyframed track into a baked one by sampling every frame."""
    t = track["type"]
    if t in NO_BAKE_TRACKS:
        raise AnimMatFormatError(
            f"{TRACK_NAMES[t]} needs 32 bits per value and cannot be baked")

    values = []
    for f in range(num_frames):
        v = evaluate_track(track, f << 12)
        values.append(to_1_10_5(v) if t in FIXED_TRACKS else _s16(v))

    return {"type": t, "storage": STORE_BAKED, "values": values}


def _s16(v):
    v &= 0xFFFF
    return v - 0x10000 if v & 0x8000 else v


def optimize(anim):
    """Rewrite every track into its cheapest storage mode.

    Returns a new animation; the input is left alone so an editor can show the
    before and after sizes.
    """
    num_frames = anim["num_frames"]
    out = {"num_frames": num_frames, "targets": []}

    for target in anim["targets"]:
        new_tracks = []
        for track in target.get("tracks", []):
            mode = choose_storage(track, num_frames)

            if mode == STORE_CONST:
                value = (track["value"] if track.get("storage") == STORE_CONST
                         else evaluate_track(track, 0))
                new_tracks.append({"type": track["type"],
                                   "storage": STORE_CONST,
                                   "value": value})
            elif mode == STORE_BAKED and track.get("storage") != STORE_BAKED:
                new_tracks.append(bake_track(track, num_frames))
            else:
                new_tracks.append(dict(track))

        out["targets"].append({"name": target.get("name", ""),
                               "tracks": new_tracks})

    return out


# ---------------------------------------------------------------------------
# Reading
# ---------------------------------------------------------------------------

def _read_name(blob, offset):
    raw = blob[offset:offset + NAME_LEN]
    return raw.split(b"\x00", 1)[0].decode("ascii", "replace")


def loads(blob):
    """Parse a .neaanimmat file. Both versions come back in the same shape."""
    if len(blob) < HEADER_SIZE:
        raise AnimMatFormatError("file is too short to hold a header")

    magic, version, count, num_frames = struct.unpack_from("<IIHH", blob, 0)

    if magic != MAGIC:
        raise AnimMatFormatError(f"bad magic 0x{magic:08X}, expected 0x{MAGIC:08X}")

    if version == VERSION_1:
        return _loads_v1(blob, count, num_frames)
    if version == VERSION_2:
        return _loads_v2(blob, count, num_frames)

    raise AnimMatFormatError(f"unsupported version {version}")


def _loads_v1(blob, num_tracks, num_frames):
    if not 1 <= num_tracks <= MAX_TRACKS:
        raise AnimMatFormatError(f"bad track count {num_tracks}")

    tracks = []
    for i in range(num_tracks):
        off = HEADER_SIZE + i * TRACK_HEADER_SIZE
        t_type, interp, n_keys, key_off, _ = struct.unpack_from("<BBHII", blob, off)
        tracks.append({
            "type": t_type,
            "interp": interp,
            "storage": STORE_KEYS,
            "keys": _read_keys(blob, key_off, n_keys),
        })

    return {"num_frames": num_frames,
            "targets": [{"name": "", "tracks": tracks}]}


def _loads_v2(blob, num_targets, num_frames):
    if not 1 <= num_targets <= MAX_TARGETS:
        raise AnimMatFormatError(f"bad target count {num_targets}")

    targets = []
    for t in range(num_targets):
        off = HEADER_SIZE + t * TARGET_HEADER_SIZE
        name = _read_name(blob, off)
        num_tracks, _pad, track_off = struct.unpack_from("<HHI", blob, off + NAME_LEN)

        if num_tracks > MAX_TRACKS:
            raise AnimMatFormatError(
                f"target '{name}' has {num_tracks} tracks, max is {MAX_TRACKS}")

        tracks = []
        for i in range(num_tracks):
            hoff = track_off + i * TRACK_HEADER_SIZE
            t_type, interp, storage, _res, count, _pad2, data = \
                struct.unpack_from("<BBBBHHI", blob, hoff)

            if storage == STORE_CONST:
                tracks.append({"type": t_type, "storage": STORE_CONST,
                               "value": data})
            elif storage == STORE_BAKED:
                values = list(struct.unpack_from(f"<{count}h", blob, data)) \
                    if count else []
                tracks.append({"type": t_type, "storage": STORE_BAKED,
                               "values": values})
            else:
                tracks.append({"type": t_type, "interp": interp,
                               "storage": STORE_KEYS,
                               "keys": _read_keys(blob, data, count)})

        targets.append({"name": name, "tracks": tracks})

    return {"num_frames": num_frames, "targets": targets}


def _read_keys(blob, offset, count):
    keys = []
    for k in range(count):
        frame, _pad, value = struct.unpack_from("<HHI", blob,
                                                offset + k * KEYFRAME_SIZE)
        keys.append([frame, value])
    return keys


def load(path):
    with open(path, "rb") as f:
        return loads(f.read())


# ---------------------------------------------------------------------------
# Writing
# ---------------------------------------------------------------------------

def dumps(anim):
    """Serialise an animation as a version 2 file."""
    targets = anim["targets"]
    num_frames = anim["num_frames"]

    if not 1 <= len(targets) <= MAX_TARGETS:
        raise AnimMatFormatError(
            f"{len(targets)} targets, must be 1 to {MAX_TARGETS}")

    for tgt in targets:
        n = len(tgt.get("tracks", []))
        if n > MAX_TRACKS:
            raise AnimMatFormatError(
                f"target '{tgt.get('name','')}' has {n} tracks, "
                f"max is {MAX_TRACKS}")

    # Layout: header, target headers, all track headers, then the value blobs.
    track_hdr_base = HEADER_SIZE + len(targets) * TARGET_HEADER_SIZE
    total_tracks = sum(len(t.get("tracks", [])) for t in targets)
    data_base = track_hdr_base + total_tracks * TRACK_HEADER_SIZE

    header = struct.pack("<IIHH4x", MAGIC, VERSION_2, len(targets), num_frames)

    target_hdrs = bytearray()
    track_hdrs = bytearray()
    data = bytearray()

    track_off = track_hdr_base
    for tgt in targets:
        tracks = tgt.get("tracks", [])
        name = tgt.get("name", "").encode("ascii", "replace")[:NAME_LEN - 1]
        target_hdrs += name.ljust(NAME_LEN, b"\x00")
        target_hdrs += struct.pack("<HHI", len(tracks), 0, track_off)
        track_off += len(tracks) * TRACK_HEADER_SIZE

        for track in tracks:
            storage = track.get("storage", STORE_KEYS)
            t_type = track["type"]
            interp = track.get("interp", INTERP_STEP)

            if t_type in STEP_ONLY_TRACKS:
                interp = INTERP_STEP

            if storage == STORE_CONST:
                count, payload = 0, track.get("value", 0) & 0xFFFFFFFF
            elif storage == STORE_BAKED:
                if t_type in NO_BAKE_TRACKS:
                    raise AnimMatFormatError(
                        f"{TRACK_NAMES[t_type]} cannot be baked")
                values = track.get("values", [])
                count, payload = len(values), data_base + len(data)
                data += struct.pack(f"<{len(values)}h", *values)
            else:
                keys = track.get("keys", [])
                if len(keys) > MAX_KEYFRAMES:
                    raise AnimMatFormatError(
                        f"{TRACK_NAMES[t_type]} has {len(keys)} keyframes, "
                        f"max is {MAX_KEYFRAMES}")
                keys = sorted(keys, key=lambda k: k[0])
                count, payload = len(keys), data_base + len(data)
                for frame, value in keys:
                    data += struct.pack("<HHI", frame, 0, value & 0xFFFFFFFF)

            track_hdrs += struct.pack("<BBBBHHI", t_type, interp, storage, 0,
                                      count, 0, payload)

            # Keyframe arrays are u32-aligned by construction; baked arrays are
            # only u16-aligned, so pad before whatever follows.
            if len(data) & 3:
                data += b"\x00" * (4 - (len(data) & 3))

    return bytes(header + target_hdrs + track_hdrs + data)


def dumps_v1(anim):
    """Serialise as a version 1 file.

    Version 1 holds one material's keyframed tracks: no target names, and no
    storage field, so every track has to be keyframed. Offered because opening a
    version 1 file and saving it would otherwise silently upgrade it, which is a
    problem if the runtime on the other end is older.
    """
    targets = anim["targets"]
    if len(targets) != 1:
        raise AnimMatFormatError(
            f"version 1 holds one target, this animation has {len(targets)}")

    tracks = targets[0].get("tracks", [])
    if not 1 <= len(tracks) <= MAX_TRACKS:
        raise AnimMatFormatError(
            f"{len(tracks)} tracks, version 1 allows 1 to {MAX_TRACKS}")

    for t in tracks:
        if t.get("storage", STORE_KEYS) != STORE_KEYS:
            raise AnimMatFormatError(
                f"{TRACK_NAMES[t['type']]} is stored as "
                f"{STORAGE_NAMES[t.get('storage', STORE_KEYS)]}; version 1 has "
                "no storage field and only understands keyframes")

    data_base = HEADER_SIZE + len(tracks) * TRACK_HEADER_SIZE

    header = struct.pack("<IIHH4x", MAGIC, VERSION_1, len(tracks),
                         anim["num_frames"])

    track_hdrs = bytearray()
    data = bytearray()

    for track in tracks:
        keys = sorted(track.get("keys", []), key=lambda k: k[0])
        if len(keys) > MAX_KEYFRAMES:
            raise AnimMatFormatError(
                f"{TRACK_NAMES[track['type']]} has {len(keys)} keyframes, "
                f"max is {MAX_KEYFRAMES}")

        interp = track.get("interp", INTERP_STEP)
        if track["type"] in STEP_ONLY_TRACKS:
            interp = INTERP_STEP

        track_hdrs += struct.pack("<BBHII", track["type"], interp, len(keys),
                                  data_base + len(data), 0)

        for frame, value in keys:
            data += struct.pack("<HHI", frame, 0, value & 0xFFFFFFFF)

    return bytes(header + track_hdrs + data)


def dump(anim, path):
    with open(path, "wb") as f:
        f.write(dumps(anim))


# ---------------------------------------------------------------------------
# Convenience constructors
# ---------------------------------------------------------------------------

def new_animation(num_frames=60):
    return {"num_frames": num_frames,
            "targets": [{"name": "", "tracks": []}]}


def new_track(track_type, storage=STORE_KEYS):
    if storage == STORE_CONST:
        return {"type": track_type, "storage": STORE_CONST, "value": 0}
    if storage == STORE_BAKED:
        return {"type": track_type, "storage": STORE_BAKED, "values": [0]}
    interp = (INTERP_STEP if track_type in STEP_ONLY_TRACKS else INTERP_LINEAR)
    return {"type": track_type, "interp": interp, "storage": STORE_KEYS,
            "keys": [[0, 0]]}
