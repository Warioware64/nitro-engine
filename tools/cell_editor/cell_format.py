#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
#
# Copyright (c) 2026 Warioware64
#
# NEACell (.neacell) binary format reader/writer, plus a Python mirror of the C
# evaluator. The C runtime in NEACell.c reads exactly the layout this module
# writes, and evaluates exactly the way `evaluate_sequence` below does -- keep
# the two in lockstep. tests/cell_eval checks that claim frame for frame.

"""cell_format.py -- read/write .neacell files, and evaluate them.

A .neacell is a cell bank plus its animations, in the shape retail DS games
used (NCER cells + NANR sequences) with three things they did not have:
per-part keyframe tracks, a parent hierarchy, and per-part colour/alpha for the
3D backends.

It carries **no pixels**. An atlas names a NEA_Material the runtime resolves
with NEA_MaterialFindByName(), the way .neaanimmat and .npe already do, and the
hardware backend streams tiles out of a companion .ncgfx blob that stays
resident in main RAM. See tools/readme.rst.

Binary layout (little-endian, every record 4-byte aligned):

    HEADER (16 bytes)
        magic         u32   'NCEL' (0x4C45434E)
        version       u16   1
        flags         u16   bit0 OAM_READY, bit1 HAS_HIERARCHY,
                            bit2 HAS_TRACKS, bit3 HAS_MULTI
        num_sections  u16
        reserved      u16
        file_size     u32

    SECTION DIRECTORY (8 bytes each, num_sections of them)
        tag           u32   four ASCII bytes in file order, e.g. b'ATLS'
        offset        u32   absolute byte offset of the section

    Every section starts with a u32 count, then its records back to back.

    ATLS  atlas (48 bytes each)
        name          char[32]  NEA_MaterialFindByName() key, NUL padded
        width         u16       pixels, power of two
        height        u16
        tex_format    u8        NEA_TextureFormat (PAL16=3, PAL256=4, A1RGB5=7)
        obj_color     u8        NEA_OBJColorMode, OAM backend only
        flags         u8        bit0 COLOR0_TRANSPARENT
        pad           u8[9]

    CELS  cell (20 bytes each)
        first_part    u16       index into PART
        num_parts     u16
        min_x, min_y  s16       bounding box, cell space, Y down
        max_x, max_y  s16
        radius        u16       bounding circle, retail NCER cellAttr & 0x3F << 2
        flags         u16       bit0 BBOX_VALID
        reserved      u32

    PART  part (40 bytes each) -- drawn in array order, so the LAST part is on
          top, matching how the 3D backends submit quads. Note this is the
          reverse of retail NCER, where OBJ 0 is topmost; the importer flips it.
        src_x, src_y  u16       texel origin in the atlas
        src_w, src_h  u16       texel size
        off_x, off_y  s16       top-left in cell space
        pivot_x       s16       rotation/scale centre, cell space
        pivot_y       s16
        parent        s16       parent part index, -1 = root
        color         u16       RGB15 tint, 0x7FFF = untinted (3D only)
        atlas         u8        index into ATLS
        pal_slot      u8        16-colour OBJ palette slot (OAM only)
        obj_size      u8        NEA_OBJSize, 0xFF = not OAM-representable
        obj_color     u8        NEA_OBJColorMode (OAM only)
        priority      u8        0-3
        alpha         u8        0-31 (3D only)
        poly_id_off   u8        added to the instance poly ID (3D only)
        flags         u8        bit0 HFLIP, bit1 VFLIP, bit2 DOUBLE_SIZE,
                                bit3 HIDDEN, bit4 NO_OAM, bit5 NO_3D
        gfx_offset    u32       byte offset into the .ncgfx blob (OAM only)
        gfx_size      u32       bytes to transfer (OAM only)
        reserved      u32

    SEQS  sequence (32 bytes each)
        name          char[16]
        kind          u8        0 CELL, 1 RIG, 2 MULTI
        mode          u8        1 forward, 2 forward-loop,
                                3 ping-pong once, 4 ping-pong loop
                                (retail NANR_SEQ_MODE_* values and semantics)
        interp        u8        0 step, 1 linear (CELL/MULTI frame SRT)
        flags         u8
        first_frame   u16       index into FRMS (CELL and MULTI)
        num_frames    u16       frame records (CELL and MULTI)
        first_track   u16       index into TRKS (RIG)
        num_tracks    u16       (RIG)
        total_ticks   u16       cached: sum of durations, or the RIG length
        cell          u16       the rig cell (RIG)

    FRMS  frame (20 bytes each)
        target        u16       cell index (CELL) or multi-cell index (MULTI)
        duration      u16       1/60 s ticks
        sx, sy        s32       fx32, 4096 = 1.0
        rot           u16       65536 = 360 degrees
        pad           u16
        px, py        s16

    TRKS  track (12 bytes each, the same shape as an AnimMat track header)
        channel       u8        see CH_* below
        interp        u8        0 step, 1 linear
        storage       u8        0 KEYS, 1 CONST, 2 BAKED
        flags         u8
        part          u16       part index within the rig cell
        count         u16       KEYS: keyframes. BAKED: ticks. CONST: 0
        data          u32       KEYS/BAKED: byte offset into KEYS.
                                CONST: the value itself

    KEYS  the pool the tracks point into
        keyframe      u16 tick, u16 pad, s32 value   (8 bytes)
        baked value   s16                            (2 bytes)

    MCEL  multi-cell (20 bytes each)
        name          char[16]
        first_node    u16       index into NODE
        num_nodes     u16

    NODE  multi-cell node (8 bytes each), drawn in array order
        seq           u16       the sequence this node plays, on its own clock
        x, y          s16       node offset, added to the node's translation
        priority      u8
        flags         u8        bit0 START_PAUSED

    BUDG  hardware budget (32 bytes, one record) -- what the OAM backend must
          allocate up front, computed by the exporter because the runtime
          cannot afford to discover its own worst case at load time
        max_objs      u16[12]   peak simultaneous OBJs per NEA_OBJSize
        max_transfer  u32       largest single tile copy, bytes
        max_affine    u16       peak simultaneous rotated/scaled parts
        pad           u16

The in-memory form is plain dicts, so it serialises to JSON without help.
"""

import math
import struct

MAGIC = 0x4C45434E  # b'NCEL'
VERSION = 1

HEADER_SIZE = 16
SECTION_ENTRY_SIZE = 8

ATLAS_SIZE = 48
CELL_SIZE = 20
PART_SIZE = 40
SEQ_SIZE = 32
FRAME_SIZE = 20
TRACK_SIZE = 12
KEY_SIZE = 8
MCELL_SIZE = 20
NODE_SIZE = 8
BUDGET_SIZE = 32

ATLAS_NAME_LEN = 32
SEQ_NAME_LEN = 16

# Header flags
F_OAM_READY = 1 << 0
F_HAS_HIERARCHY = 1 << 1
F_HAS_TRACKS = 1 << 2
F_HAS_MULTI = 1 << 3

# Part flags
P_HFLIP = 1 << 0
P_VFLIP = 1 << 1
P_DOUBLE_SIZE = 1 << 2
P_HIDDEN = 1 << 3
P_NO_OAM = 1 << 4
P_NO_3D = 1 << 5

# Cell flags
C_BBOX_VALID = 1 << 0

# Node flags
N_START_PAUSED = 1 << 0

# Anchors: the point of a cell that its position refers to, and that the
# instance transform rotates and scales about. Runtime state, not stored in the
# file, but mirrored here because NEACell.c applies it and a preview that
# ignores it would disagree with the hardware.
ANCHOR_CENTER = 0
ANCHOR_BOTTOM = 1
ANCHOR_TOPLEFT = 2

# Sequence kinds
KIND_CELL = 0
KIND_RIG = 1
KIND_MULTI = 2

# Playback modes. The values and the semantics are retail NANR's, including the
# fact that "backward" actually means ping-pong: mode 3 plays forward, reverses
# at the end and stops at frame 0; mode 4 does the same but never stops.
MODE_FORWARD = 1
MODE_FORWARD_LOOP = 2
MODE_PINGPONG = 3
MODE_PINGPONG_LOOP = 4

# Interpolation
INTERP_STEP = 0
INTERP_LINEAR = 1

# Track storage, the same three modes NEAAnimMat uses and for the same reasons.
STORE_KEYS = 0
STORE_CONST = 1
STORE_BAKED = 2

# Track channels
CH_TX = 0
CH_TY = 1
CH_ROT = 2
CH_SX = 3
CH_SY = 4
CH_ALPHA = 5
CH_COLOR = 6
CH_VISIBLE = 7
CH_PARTSWAP = 8
CH_COUNT = 9

CHANNEL_NAMES = {
    CH_TX: "Translate X",
    CH_TY: "Translate Y",
    CH_ROT: "Rotation",
    CH_SX: "Scale X",
    CH_SY: "Scale Y",
    CH_ALPHA: "Alpha",
    CH_COLOR: "Color",
    CH_VISIBLE: "Visible",
    CH_PARTSWAP: "Part swap",
}

# Channels the hardware OBJ path cannot express at all. Kept in the format
# because the 3D backends can, and documented so nobody is surprised.
CHANNELS_3D_ONLY = (CH_ALPHA, CH_COLOR)

# NEA_OBJSize, from NEAHw2D.h. The index is the enum value.
OBJ_SIZE_DIMS = [
    (8, 8), (16, 16), (32, 32), (64, 64),
    (16, 8), (32, 8), (32, 16), (64, 32),
    (8, 16), (8, 32), (16, 32), (32, 64),
]
OBJ_SIZE_COUNT = len(OBJ_SIZE_DIMS)
OBJ_SIZE_BY_DIMS = {dims: i for i, dims in enumerate(OBJ_SIZE_DIMS)}
OBJ_SIZE_NONE = 0xFF

OBJ_COLOR_16 = 0
OBJ_COLOR_256 = 1

# NEA_TextureFormat, from NEAPolygon.h.
TEX_A3PAL32 = 1
TEX_PAL4 = 2
TEX_PAL16 = 3
TEX_PAL256 = 4
TEX_A5PAL8 = 6
TEX_A1RGB5 = 7

FX32_ONE = 1 << 12   # f32, the engine's 1.19.12
ROT_FULL = 1 << 16   # a full turn, retail NANR's unit


# ---------------------------------------------------------------------------
# Fixed point
#
# Everything the evaluator touches is integer arithmetic, because the whole
# point of the cross-check test is that Python and the ARM9 produce the *same*
# numbers. A float multiply here would agree to about six digits and disagree
# in the low bits of every f32, which is exactly the drift the test exists to
# catch.
# ---------------------------------------------------------------------------

def f32(value):
    """Float -> 1.19.12 fixed point."""
    return int(math.floor(value * FX32_ONE + 0.5)) if value >= 0 \
        else -int(math.floor(-value * FX32_ONE + 0.5))


def from_f32(value):
    """1.19.12 fixed point -> float. For display only."""
    return value / FX32_ONE


def mulf32(a, b):
    """The engine's mulf32: a 64-bit product shifted back down by 12.

    C shifts a signed int64 right, which rounds toward negative infinity.
    Python's >> does the same on ints, so this needs no correction.
    """
    return _s32((a * b) >> 12)


def divf32(a, b):
    if b == 0:
        return 0
    return _s32((a << 12) // b)


def _s32(v):
    v &= 0xFFFFFFFF
    return v - 0x100000000 if v & 0x80000000 else v


def _s16(v):
    v &= 0xFFFF
    return v - 0x10000 if v & 0x8000 else v


# ---------------------------------------------------------------------------
# Trigonometry
#
# NEACell does NOT use libnds sinLerp(). Its lookup table lives inside libnds.a
# and is not something this module can mirror without guessing, and a preview
# that guesses is worse than no preview. So the table is ours: 512 entries,
# 4096 = 1.0, linearly interpolated over the low 7 bits of the 16-bit angle.
# emit_sin_table_c() writes the identical table into the C runtime, and
# tests/cell_eval compares them entry for entry.
# ---------------------------------------------------------------------------

SIN_TABLE_SIZE = 512
SIN_SHIFT = 7  # 16-bit angle -> 512-entry index


def _build_sin_table():
    table = []
    for i in range(SIN_TABLE_SIZE):
        v = 4096.0 * math.sin(2.0 * math.pi * i / SIN_TABLE_SIZE)
        # Round half away from zero, so the C generator and this agree on the
        # exact halves that fall out of a 512-point sine.
        table.append(int(math.floor(v + 0.5)) if v >= 0
                     else -int(math.floor(-v + 0.5)))
    return table


SIN_TABLE = _build_sin_table()


def sin_fx(rot):
    """Sine of a 16-bit angle (65536 = 360 degrees) as 4.12 fixed point."""
    rot &= 0xFFFF
    idx = rot >> SIN_SHIFT
    frac = rot & ((1 << SIN_SHIFT) - 1)
    a = SIN_TABLE[idx]
    b = SIN_TABLE[(idx + 1) & (SIN_TABLE_SIZE - 1)]
    return a + (((b - a) * frac) >> SIN_SHIFT)


def cos_fx(rot):
    return sin_fx((rot & 0xFFFF) + (ROT_FULL // 4))


def emit_sin_table_c():
    """The generated C table, so the runtime and this module cannot drift."""
    lines = [
        "// Generated by tools/cell_editor/cell_format.py "
        "-- do not edit by hand.",
        "//",
        "// 512-entry sine, 4096 = 1.0, indexed by the top 9 bits of a 16-bit",
        "// angle. tests/cell_eval compares this against the Python table.",
        "",
        "static const int16_t ne_cell_sin_table[512] = {",
    ]
    for i in range(0, SIN_TABLE_SIZE, 8):
        row = ", ".join("%6d" % v for v in SIN_TABLE[i:i + 8])
        lines.append("    %s," % row)
    lines.append("};")
    return "\n".join(lines) + "\n"


# ---------------------------------------------------------------------------
# Constructors for the in-memory form
# ---------------------------------------------------------------------------

def new_bank():
    return {
        "version": VERSION,
        "atlases": [],
        "cells": [],
        "sequences": [],
        "multicells": [],
        "budget": None,
    }


def new_atlas(name="", width=0, height=0, tex_format=TEX_PAL16,
              obj_color=OBJ_COLOR_16, flags=0):
    return {
        "name": name, "width": width, "height": height,
        "tex_format": tex_format, "obj_color": obj_color, "flags": flags,
    }


def new_cell(parts=None):
    return {
        "parts": list(parts) if parts else [],
        "min_x": 0, "min_y": 0, "max_x": 0, "max_y": 0,
        "radius": 0, "flags": 0,
    }


def new_part(src_x=0, src_y=0, src_w=8, src_h=8, off_x=0, off_y=0, **kw):
    part = {
        "src_x": src_x, "src_y": src_y, "src_w": src_w, "src_h": src_h,
        "off_x": off_x, "off_y": off_y,
        "pivot_x": off_x + src_w // 2, "pivot_y": off_y + src_h // 2,
        "parent": -1, "color": 0x7FFF,
        "atlas": 0, "pal_slot": 0,
        "obj_size": OBJ_SIZE_BY_DIMS.get((src_w, src_h), OBJ_SIZE_NONE),
        "obj_color": OBJ_COLOR_16,
        "priority": 0, "alpha": 31, "poly_id_off": 0, "flags": 0,
        "gfx_offset": 0xFFFFFFFF, "gfx_size": 0,
    }
    part.update(kw)
    if part["obj_size"] == OBJ_SIZE_NONE:
        part["flags"] |= P_NO_OAM
    return part


def new_sequence(name="", kind=KIND_CELL, mode=MODE_FORWARD_LOOP,
                 interp=INTERP_STEP):
    return {
        "name": name, "kind": kind, "mode": mode, "interp": interp,
        "flags": 0, "frames": [], "tracks": [], "cell": 0,
        "num_frames": 0,  # RIG only: length in ticks
    }


def new_frame(target=0, duration=1, sx=FX32_ONE, sy=FX32_ONE,
              rot=0, px=0, py=0):
    return {"target": target, "duration": duration, "sx": sx, "sy": sy,
            "rot": rot, "px": px, "py": py}


def new_track(part=0, channel=CH_TX, storage=STORE_KEYS,
              interp=INTERP_LINEAR, **kw):
    track = {"part": part, "channel": channel, "storage": storage,
             "interp": interp, "flags": 0}
    track.update(kw)
    return track


def new_multicell(name="", nodes=None):
    return {"name": name, "nodes": list(nodes) if nodes else []}


def new_node(seq=0, x=0, y=0, priority=0, flags=0):
    return {"seq": seq, "x": x, "y": y, "priority": priority, "flags": flags}


# ---------------------------------------------------------------------------
# Derived values
# ---------------------------------------------------------------------------

def sequence_ticks(seq):
    """Total length of a sequence in 1/60 s ticks."""
    if seq["kind"] == KIND_RIG:
        return max(1, int(seq.get("num_frames", 0)))
    total = sum(max(1, f["duration"]) for f in seq["frames"])
    return max(1, total)


def cell_bounds(bank, cell):
    """Bounding box over a cell's parts, in cell space."""
    if not cell["parts"]:
        return (0, 0, 0, 0)
    min_x = min(p["off_x"] for p in cell["parts"])
    min_y = min(p["off_y"] for p in cell["parts"])
    max_x = max(p["off_x"] + p["src_w"] for p in cell["parts"])
    max_y = max(p["off_y"] + p["src_h"] for p in cell["parts"])
    return (min_x, min_y, max_x, max_y)


def compute_bounds(bank):
    """Fill in every cell's bounding box and radius. Idempotent."""
    for cell in bank["cells"]:
        min_x, min_y, max_x, max_y = cell_bounds(bank, cell)
        cell["min_x"], cell["min_y"] = min_x, min_y
        cell["max_x"], cell["max_y"] = max_x, max_y
        cx, cy = (min_x + max_x) / 2.0, (min_y + max_y) / 2.0
        r = 0.0
        for x in (min_x, max_x):
            for y in (min_y, max_y):
                r = max(r, math.hypot(x - cx, y - cy))
        cell["radius"] = min(0xFFFF, int(math.ceil(r)))
        cell["flags"] |= C_BBOX_VALID


def compute_budget(bank):
    """What the OAM backend has to allocate up front.

    Walks every cell a sequence can reach, because the runtime binds once and
    then never allocates again -- if this understates the peak, parts vanish
    mid-animation. Multi-cell nodes are summed, since their cells are on screen
    at the same time.
    """
    def cell_cost(cell_index):
        objs = [0] * OBJ_SIZE_COUNT
        affine = 0
        transfer = 0
        if not (0 <= cell_index < len(bank["cells"])):
            return objs, affine, transfer
        for part in bank["cells"][cell_index]["parts"]:
            if part["flags"] & (P_NO_OAM | P_HIDDEN):
                continue
            size = part["obj_size"]
            if size == OBJ_SIZE_NONE or size >= OBJ_SIZE_COUNT:
                continue
            objs[size] += 1
            transfer = max(transfer, part["gfx_size"])
        return objs, affine, transfer

    def seq_cost(seq_index, seen):
        """Peak cost of one sequence. `seen` breaks multi-cell recursion."""
        objs = [0] * OBJ_SIZE_COUNT
        affine = 0
        transfer = 0
        if seq_index in seen or not (0 <= seq_index < len(bank["sequences"])):
            return objs, affine, transfer
        seen = seen | {seq_index}
        seq = bank["sequences"][seq_index]

        if seq["kind"] == KIND_RIG:
            cells = [seq["cell"]]
        elif seq["kind"] == KIND_MULTI:
            cells = []
        else:
            cells = [f["target"] for f in seq["frames"]]

        for ci in cells:
            c_objs, c_aff, c_tr = cell_cost(ci)
            objs = [max(a, b) for a, b in zip(objs, c_objs)]
            affine = max(affine, c_aff)
            transfer = max(transfer, c_tr)

        # A rig or an interpolated sequence can rotate any part, so every part
        # is a potential affine consumer. A stepped CELL sequence only needs
        # one when a frame actually carries a non-identity SRT.
        if seq["kind"] == KIND_RIG:
            rot_parts = {t["part"] for t in seq["tracks"]
                         if t["channel"] in (CH_ROT, CH_SX, CH_SY)}
            affine = max(affine, len(rot_parts))
        else:
            for frame in seq["frames"]:
                if frame["rot"] != 0 or frame["sx"] != FX32_ONE \
                        or frame["sy"] != FX32_ONE:
                    affine = max(affine, sum(cell_cost(frame["target"])[0]))
                    break

        if seq["kind"] == KIND_MULTI:
            for frame in seq["frames"]:
                mi = frame["target"]
                if not (0 <= mi < len(bank["multicells"])):
                    continue
                m_objs = [0] * OBJ_SIZE_COUNT
                m_aff = 0
                for node in bank["multicells"][mi]["nodes"]:
                    n_objs, n_aff, n_tr = seq_cost(node["seq"], seen)
                    m_objs = [a + b for a, b in zip(m_objs, n_objs)]
                    m_aff += n_aff
                    transfer = max(transfer, n_tr)
                objs = [max(a, b) for a, b in zip(objs, m_objs)]
                affine = max(affine, m_aff)

        return objs, affine, transfer

    max_objs = [0] * OBJ_SIZE_COUNT
    max_affine = 0
    max_transfer = 0
    for i in range(len(bank["sequences"])):
        objs, affine, transfer = seq_cost(i, frozenset())
        max_objs = [max(a, b) for a, b in zip(max_objs, objs)]
        max_affine = max(max_affine, affine)
        max_transfer = max(max_transfer, transfer)

    # Every multi-cell, not only the ones a MULTI sequence names.
    # NEA_CellAnimPlayMulti() can seat any of them directly, and a bank that
    # never wraps its compositions in a sequence would otherwise get a budget
    # sized for one node -- which the hardware backend would discover halfway
    # through an animation, by running out of sprites.
    for mcell in bank["multicells"]:
        m_objs = [0] * OBJ_SIZE_COUNT
        m_affine = 0
        for node in mcell["nodes"]:
            n_objs, n_aff, n_tr = seq_cost(node["seq"], frozenset())
            m_objs = [a + b for a, b in zip(m_objs, n_objs)]
            m_affine += n_aff
            max_transfer = max(max_transfer, n_tr)
        max_objs = [max(a, b) for a, b in zip(max_objs, m_objs)]
        max_affine = max(max_affine, m_affine)

    bank["budget"] = {
        "max_objs": max_objs,
        "max_transfer": max_transfer,
        "max_affine": max_affine,
    }
    return bank["budget"]


def bank_flags(bank):
    flags = 0
    oam_ready = True
    for cell in bank["cells"]:
        for part in cell["parts"]:
            if part["flags"] & P_NO_OAM or part["obj_size"] == OBJ_SIZE_NONE:
                oam_ready = False
            if part["parent"] >= 0:
                flags |= F_HAS_HIERARCHY
    if oam_ready and bank["cells"]:
        flags |= F_OAM_READY
    if any(s["tracks"] for s in bank["sequences"]):
        flags |= F_HAS_TRACKS
    if bank["multicells"]:
        flags |= F_HAS_MULTI
    return flags


# ---------------------------------------------------------------------------
# Writer
# ---------------------------------------------------------------------------

def _name_bytes(name, length):
    raw = name.encode("utf-8")[:length - 1]
    return raw + b"\0" * (length - len(raw))


def _pack_keys(bank):
    """Build the KEYS pool and stamp every track's `data` field.

    KEYS tracks store 8-byte keyframes; BAKED tracks store one s16 per tick;
    CONST tracks store the value in `data` itself and touch the pool not at
    all. Same three shapes as NEAAnimMat, for the same reason: a track that
    never changes should not cost an array, and a track that changes every
    tick should not cost a binary search.
    """
    pool = bytearray()
    for seq in bank["sequences"]:
        for track in seq["tracks"]:
            storage = track["storage"]
            if storage == STORE_CONST:
                track["_data"] = _s32(track["value"]) & 0xFFFFFFFF
                track["_count"] = 0
            elif storage == STORE_BAKED:
                values = track["values"]
                while len(pool) % 4:
                    pool.append(0)
                track["_data"] = len(pool)
                track["_count"] = len(values)
                for v in values:
                    pool += struct.pack("<h", _s16(v))
            else:
                keys = track["keys"]
                while len(pool) % 4:
                    pool.append(0)
                track["_data"] = len(pool)
                track["_count"] = len(keys)
                for key in keys:
                    tick, value = key[0], key[-1]
                    pool += struct.pack("<HHi", tick & 0xFFFF, 0, _s32(value))
    while len(pool) % 4:
        pool.append(0)
    return bytes(pool)


def _unpack_keys(bank):
    """Drop the scratch fields _pack_keys left behind."""
    for seq in bank["sequences"]:
        for track in seq["tracks"]:
            track.pop("_data", None)
            track.pop("_count", None)


def dumps(bank):
    """Serialise an in-memory bank to .neacell bytes."""
    compute_bounds(bank)
    if bank.get("budget") is None:
        compute_budget(bank)

    key_pool = _pack_keys(bank)

    # Flatten. The file stores one pool of each record type and indexes into
    # it, so the runtime can walk a cell's parts without chasing pointers.
    parts, cells = [], []
    for cell in bank["cells"]:
        cells.append((len(parts), len(cell["parts"]), cell))
        parts.extend(cell["parts"])

    frames, tracks, seqs = [], [], []
    for seq in bank["sequences"]:
        seqs.append((len(frames), len(seq["frames"]),
                     len(tracks), len(seq["tracks"]), seq))
        frames.extend(seq["frames"])
        tracks.extend(seq["tracks"])

    nodes, mcells = [], []
    for mcell in bank["multicells"]:
        mcells.append((len(nodes), len(mcell["nodes"]), mcell))
        nodes.extend(mcell["nodes"])

    sections = []

    def section(tag, count, payload):
        if count == 0 and tag not in (b"ATLS", b"CELS", b"PART", b"SEQS"):
            return
        sections.append((tag, struct.pack("<I", count) + payload))

    # ATLS
    buf = bytearray()
    for a in bank["atlases"]:
        buf += struct.pack("<32sHHBBB9x",
                           _name_bytes(a["name"], ATLAS_NAME_LEN),
                           a["width"], a["height"], a["tex_format"],
                           a["obj_color"], a["flags"])
    section(b"ATLS", len(bank["atlases"]), bytes(buf))

    # CELS
    buf = bytearray()
    for first, count, c in cells:
        buf += struct.pack("<HHhhhhHH4x", first, count,
                           c["min_x"], c["min_y"], c["max_x"], c["max_y"],
                           c["radius"], c["flags"])
    section(b"CELS", len(cells), bytes(buf))

    # PART
    buf = bytearray()
    for p in parts:
        buf += struct.pack("<HHHHhhhhhHBBBBBBBBII4x",
                           p["src_x"], p["src_y"], p["src_w"], p["src_h"],
                           p["off_x"], p["off_y"],
                           p["pivot_x"], p["pivot_y"], p["parent"],
                           p["color"],
                           p["atlas"], p["pal_slot"],
                           p["obj_size"], p["obj_color"],
                           p["priority"], p["alpha"],
                           p["poly_id_off"], p["flags"],
                           p["gfx_offset"] & 0xFFFFFFFF, p["gfx_size"])
    section(b"PART", len(parts), bytes(buf))

    # SEQS
    buf = bytearray()
    for first_f, n_f, first_t, n_t, s in seqs:
        buf += struct.pack("<16sBBBBHHHHHH",
                           _name_bytes(s["name"], SEQ_NAME_LEN),
                           s["kind"], s["mode"], s["interp"], s["flags"],
                           first_f, n_f, first_t, n_t,
                           sequence_ticks(s), s["cell"])
    section(b"SEQS", len(seqs), bytes(buf))

    # FRMS
    buf = bytearray()
    for f in frames:
        buf += struct.pack("<HHiiHHhh", f["target"], f["duration"],
                           _s32(f["sx"]), _s32(f["sy"]),
                           f["rot"] & 0xFFFF, 0, f["px"], f["py"])
    section(b"FRMS", len(frames), bytes(buf))

    # TRKS
    buf = bytearray()
    for t in tracks:
        buf += struct.pack("<BBBBHHI", t["channel"], t["interp"],
                           t["storage"], t["flags"], t["part"],
                           t["_count"], t["_data"])
    section(b"TRKS", len(tracks), bytes(buf))

    # KEYS: the count word is the pool size in bytes, not a record count.
    section(b"KEYS", len(key_pool), key_pool)

    # MCEL / NODE
    buf = bytearray()
    for first, count, m in mcells:
        buf += struct.pack("<16sHH", _name_bytes(m["name"], SEQ_NAME_LEN),
                           first, count)
    section(b"MCEL", len(mcells), bytes(buf))

    buf = bytearray()
    for n in nodes:
        buf += struct.pack("<HhhBB", n["seq"], n["x"], n["y"],
                           n["priority"], n["flags"])
    section(b"NODE", len(nodes), bytes(buf))

    # BUDG
    b = bank["budget"]
    objs = list(b["max_objs"]) + [0] * (OBJ_SIZE_COUNT - len(b["max_objs"]))
    buf = struct.pack("<12H", *objs[:OBJ_SIZE_COUNT])
    buf += struct.pack("<IHH", b["max_transfer"], b["max_affine"], 0)
    section(b"BUDG", 1, buf)

    # Lay out: header, directory, then the sections in the order built.
    dir_size = SECTION_ENTRY_SIZE * len(sections)
    offset = HEADER_SIZE + dir_size
    directory = bytearray()
    body = bytearray()
    for tag, payload in sections:
        directory += tag + struct.pack("<I", offset)
        body += payload
        offset += len(payload)
        assert len(payload) % 4 == 0, "section %r is not 4-aligned" % tag

    file_size = HEADER_SIZE + dir_size + len(body)
    header = struct.pack("<IHHHHI", MAGIC, VERSION, bank_flags(bank),
                         len(sections), 0, file_size)
    _unpack_keys(bank)
    return bytes(header + directory + body)


def dump(bank, path):
    with open(path, "wb") as fp:
        fp.write(dumps(bank))


# ---------------------------------------------------------------------------
# Reader
# ---------------------------------------------------------------------------

class CellFormatError(Exception):
    pass


def _read_name(raw):
    return raw.split(b"\0", 1)[0].decode("utf-8", "replace")


def loads(data):
    """Parse .neacell bytes into the in-memory form."""
    if len(data) < HEADER_SIZE:
        raise CellFormatError("file is shorter than a header")
    magic, version, flags, num_sections, _res, file_size = \
        struct.unpack_from("<IHHHHI", data, 0)
    if magic != MAGIC:
        raise CellFormatError("bad magic 0x%08X, expected 0x%08X"
                              % (magic, MAGIC))
    if version != VERSION:
        raise CellFormatError("unsupported version %d" % version)
    if file_size > len(data):
        raise CellFormatError("header claims %d bytes, file has %d"
                              % (file_size, len(data)))

    sections = {}
    for i in range(num_sections):
        off = HEADER_SIZE + i * SECTION_ENTRY_SIZE
        tag = data[off:off + 4]
        (payload_off,) = struct.unpack_from("<I", data, off + 4)
        if payload_off + 4 > len(data):
            raise CellFormatError("section %r points past the end" % tag)
        (count,) = struct.unpack_from("<I", data, payload_off)
        sections[tag] = (payload_off + 4, count)

    def records(tag, size):
        if tag not in sections:
            return []
        base, count = sections[tag]
        return [base + i * size for i in range(count)]

    bank = new_bank()
    bank["flags"] = flags

    for off in records(b"ATLS", ATLAS_SIZE):
        name, w, h, fmt, ocol, aflags = struct.unpack_from(
            "<32sHHBBB9x", data, off)
        bank["atlases"].append(new_atlas(_read_name(name), w, h, fmt,
                                         ocol, aflags))

    all_parts = []
    for off in records(b"PART", PART_SIZE):
        (sx, sy, sw, sh, ox, oy, pvx, pvy, parent, color,
         atlas, pal_slot, obj_size, obj_color,
         priority, alpha, poly_id_off, pflags,
         gfx_offset, gfx_size) = struct.unpack_from(
            "<HHHHhhhhhHBBBBBBBBII4x", data, off)
        all_parts.append({
            "src_x": sx, "src_y": sy, "src_w": sw, "src_h": sh,
            "off_x": ox, "off_y": oy, "pivot_x": pvx, "pivot_y": pvy,
            "parent": parent, "color": color, "atlas": atlas,
            "pal_slot": pal_slot, "obj_size": obj_size,
            "obj_color": obj_color, "priority": priority, "alpha": alpha,
            "poly_id_off": poly_id_off, "flags": pflags,
            "gfx_offset": gfx_offset, "gfx_size": gfx_size,
        })

    for off in records(b"CELS", CELL_SIZE):
        (first, count, min_x, min_y, max_x, max_y, radius, cflags) = \
            struct.unpack_from("<HHhhhhHH4x", data, off)
        cell = new_cell(all_parts[first:first + count])
        cell.update({"min_x": min_x, "min_y": min_y, "max_x": max_x,
                     "max_y": max_y, "radius": radius, "flags": cflags})
        bank["cells"].append(cell)

    all_frames = []
    for off in records(b"FRMS", FRAME_SIZE):
        target, duration, sx, sy, rot, _pad, px, py = \
            struct.unpack_from("<HHiiHHhh", data, off)
        all_frames.append(new_frame(target, duration, sx, sy, rot, px, py))

    key_base, _key_size = sections.get(b"KEYS", (0, 0))

    all_tracks = []
    for off in records(b"TRKS", TRACK_SIZE):
        channel, interp, storage, tflags, part, count, dat = \
            struct.unpack_from("<BBBBHHI", data, off)
        track = {"channel": channel, "interp": interp, "storage": storage,
                 "flags": tflags, "part": part}
        if storage == STORE_CONST:
            track["value"] = _s32(dat)
        elif storage == STORE_BAKED:
            track["values"] = [
                struct.unpack_from("<h", data, key_base + dat + i * 2)[0]
                for i in range(count)]
        else:
            keys = []
            for i in range(count):
                tick, _p, value = struct.unpack_from(
                    "<HHi", data, key_base + dat + i * KEY_SIZE)
                keys.append([tick, value])
            track["keys"] = keys
        all_tracks.append(track)

    for off in records(b"SEQS", SEQ_SIZE):
        (name, kind, mode, interp, sflags, first_f, n_f,
         first_t, n_t, total_ticks, cell) = struct.unpack_from(
            "<16sBBBBHHHHHH", data, off)
        seq = new_sequence(_read_name(name), kind, mode, interp)
        seq["flags"] = sflags
        seq["frames"] = all_frames[first_f:first_f + n_f]
        seq["tracks"] = all_tracks[first_t:first_t + n_t]
        seq["cell"] = cell
        seq["num_frames"] = total_ticks if kind == KIND_RIG else n_f
        bank["sequences"].append(seq)

    all_nodes = []
    for off in records(b"NODE", NODE_SIZE):
        seq, x, y, priority, nflags = struct.unpack_from("<HhhBB", data, off)
        all_nodes.append(new_node(seq, x, y, priority, nflags))

    for off in records(b"MCEL", MCELL_SIZE):
        name, first, count = struct.unpack_from("<16sHH", data, off)
        bank["multicells"].append(
            new_multicell(_read_name(name), all_nodes[first:first + count]))

    if b"BUDG" in sections:
        base, _count = sections[b"BUDG"]
        objs = list(struct.unpack_from("<12H", data, base))
        transfer, affine, _pad = struct.unpack_from("<IHH", data, base + 24)
        bank["budget"] = {"max_objs": objs, "max_transfer": transfer,
                          "max_affine": affine}

    return bank


def load(path):
    with open(path, "rb") as fp:
        return loads(fp.read())


# ---------------------------------------------------------------------------
# Evaluation -- mirrors ne_cell_eval_track() / NEA_CellEvaluate() in NEACell.c
#
# Everything below is integer arithmetic on 1.19.12 fixed point, because
# tests/cell_eval compares these numbers to the ARM9's byte for byte.
# ---------------------------------------------------------------------------

# Channels stored as f32 at runtime but baked as 1.10.5, widened with << 7.
FIXED_CHANNELS = (CH_TX, CH_TY, CH_SX, CH_SY)


def to_1_10_5(value_f32):
    """1.19.12 -> 1.10.5. Raises rather than silently wrapping."""
    v = _s32(value_f32)
    out = v >> 7  # truncation toward -inf, matching the runtime's << 7
    if out < -32768 or out > 32767:
        raise CellFormatError(
            "value %g is out of range for a baked track "
            "(1.10.5 covers about -1024..1024)" % from_f32(value_f32))
    return out


def _widen_baked(channel, value):
    if channel in FIXED_CHANNELS:
        return _s32(value << 7)
    if channel == CH_ROT:
        return value & 0xFFFF
    return value & 0xFFFF


def _rgb15_lerp(c0, c1, frac):
    r0, g0, b0 = c0 & 0x1F, (c0 >> 5) & 0x1F, (c0 >> 10) & 0x1F
    r1, g1, b1 = c1 & 0x1F, (c1 >> 5) & 0x1F, (c1 >> 10) & 0x1F
    r = max(0, min(31, r0 + mulf32(r1 - r0, frac)))
    g = max(0, min(31, g0 + mulf32(g1 - g0, frac)))
    b = max(0, min(31, b0 + mulf32(b1 - b0, frac)))
    return r | (g << 5) | (b << 10)


def _rot_lerp(a, b, frac):
    """Shortest-arc interpolation of two 16-bit angles.

    Without this a track going from 350 to 10 degrees spins the long way
    round, which is never what an animator drew.
    """
    a &= 0xFFFF
    b &= 0xFFFF
    delta = (b - a) & 0xFFFF
    if delta > 0x8000:
        delta -= 0x10000
    return (a + mulf32(delta, frac)) & 0xFFFF


def evaluate_track(track, tick_f32):
    """One track at a tick given in 1.19.12."""
    storage = track.get("storage", STORE_KEYS)
    channel = track["channel"]

    if storage == STORE_CONST:
        return track.get("value", 0)

    if storage == STORE_BAKED:
        values = track.get("values", [])
        if not values:
            return 0
        i = max(0, min(len(values) - 1, tick_f32 >> 12))
        return _widen_baked(channel, values[i])

    keys = track.get("keys", [])
    if not keys:
        return 0
    if len(keys) == 1:
        return keys[0][-1]

    tick_int = tick_f32 >> 12
    if tick_int <= keys[0][0]:
        return keys[0][-1]
    if tick_int >= keys[-1][0]:
        return keys[-1][-1]

    lo, hi = 0, len(keys) - 1
    while lo < hi - 1:
        mid = (lo + hi) >> 1
        if keys[mid][0] <= tick_int:
            lo = mid
        else:
            hi = mid

    # These three carry no meaningful midpoint, so they always step.
    if track.get("interp", INTERP_STEP) == INTERP_STEP \
            or channel in (CH_VISIBLE, CH_PARTSWAP):
        return keys[lo][-1]

    span = keys[hi][0] - keys[lo][0]
    if span <= 0:
        return keys[lo][-1]

    frac = ((tick_f32 - (keys[lo][0] << 12)) << 12) // (span << 12)
    frac = max(0, min(FX32_ONE, frac))

    a, b = keys[lo][-1], keys[hi][-1]
    if channel == CH_ROT:
        return _rot_lerp(a, b, frac)
    if channel == CH_COLOR:
        return _rgb15_lerp(a & 0x7FFF, b & 0x7FFF, frac)
    if channel == CH_ALPHA:
        va, vb = a & 0x1F, b & 0x1F
        return max(0, min(31, va + mulf32(vb - va, frac)))
    sa, sb = _s32(a), _s32(b)
    return sa + mulf32(sb - sa, frac)


# --- affine helpers. An "xform" is (m0, m1, m2, m3, tx, ty), the map
# --- p -> [[m0 m1],[m2 m3]] * p + (tx, ty). All entries are 1.19.12.

IDENTITY = (FX32_ONE, 0, 0, FX32_ONE, 0, 0)


def xform_apply(x, px, py):
    return (mulf32(x[0], px) + mulf32(x[1], py) + x[4],
            mulf32(x[2], px) + mulf32(x[3], py) + x[5])


def xform_compose(outer, inner):
    """The map that applies `inner` first, then `outer`."""
    m0 = mulf32(outer[0], inner[0]) + mulf32(outer[1], inner[2])
    m1 = mulf32(outer[0], inner[1]) + mulf32(outer[1], inner[3])
    m2 = mulf32(outer[2], inner[0]) + mulf32(outer[3], inner[2])
    m3 = mulf32(outer[2], inner[1]) + mulf32(outer[3], inner[3])
    tx, ty = xform_apply(outer, inner[4], inner[5])
    return (m0, m1, m2, m3, tx, ty)


def srt_xform(rot, sx, sy, cx, cy, tx, ty):
    """Scale, then rotate, then translate, about the centre (cx, cy).

    This is NitroPaint's AnmCalcTransformMatrix laid out as an affine map, so
    an imported NANR frame lands exactly where the retail runtime put it.
    """
    if rot & 0xFFFF:
        s = sin_fx(rot)
        c = cos_fx(rot)
        m0 = mulf32(sx, c)
        m1 = -mulf32(sy, s)
        m2 = mulf32(sx, s)
        m3 = mulf32(sy, c)
    else:
        m0, m1, m2, m3 = sx, 0, 0, sy
    # t = centre - M*centre + translation
    mx = mulf32(m0, cx) + mulf32(m1, cy)
    my = mulf32(m2, cx) + mulf32(m3, cy)
    return (m0, m1, m2, m3, cx - mx + tx, cy - my + ty)


def frame_prefix(seq):
    """Start tick of every frame, plus the total. Built once at load in C."""
    prefix = [0]
    for f in seq["frames"]:
        prefix.append(prefix[-1] + max(1, f["duration"]))
    return prefix


def resolve_position(seq, tick_f32):
    """Map an ever-increasing play head onto a position inside the sequence.

    Stateless on purpose. Retail drives ping-pong with a direction flag that
    the player mutates, which makes seeking to an arbitrary tick impossible
    and makes the C and Python sides easy to drift apart. Mirroring the play
    head instead gives the same visible behaviour -- including a frame playing
    for its full duration in both directions -- and lets both sides answer
    "what does tick N look like?" without having simulated ticks 0..N-1.

    Returns (position, finished).
    """
    total = sequence_ticks(seq) << 12
    mode = seq["mode"]

    if tick_f32 < 0:
        tick_f32 = 0

    if mode == MODE_FORWARD:
        if tick_f32 >= total:
            return (total - 1, True)
        return (tick_f32, False)

    if mode == MODE_FORWARD_LOOP:
        return (tick_f32 % total, False)

    period = total * 2
    if mode == MODE_PINGPONG:
        if tick_f32 >= period:
            return (0, True)
        u = tick_f32
    else:  # MODE_PINGPONG_LOOP
        u = tick_f32 % period

    if u < total:
        return (u, False)
    return (period - 1 - u, False)


def frame_at(seq, pos_f32, prefix=None):
    """Which frame covers this position, and how far into it we are.

    Returns (frame_index, frac) where frac is 1.19.12 in [0, 1).
    """
    if not seq["frames"]:
        return (0, 0)
    if prefix is None:
        prefix = frame_prefix(seq)

    tick = pos_f32 >> 12
    lo, hi = 0, len(seq["frames"]) - 1
    while lo < hi:
        mid = (lo + hi + 1) >> 1
        if prefix[mid] <= tick:
            lo = mid
        else:
            hi = mid - 1

    duration = max(1, seq["frames"][lo]["duration"])
    frac = divf32(pos_f32 - (prefix[lo] << 12), duration << 12)
    return (lo, max(0, min(FX32_ONE - 1, frac)))


def _frame_srt(seq, index, frac):
    """The frame's scale/rotate/translate, interpolated if the sequence says so.

    Imported NANR sequences are STEP, so they read straight through and play
    exactly as retail. LINEAR is the new part.
    """
    frames = seq["frames"]
    f = frames[index]
    if seq["interp"] != INTERP_LINEAR or frac == 0 or len(frames) < 2:
        return (f["rot"], f["sx"], f["sy"], f["px"], f["py"])

    nxt = index + 1
    if nxt >= len(frames):
        if seq["mode"] in (MODE_FORWARD_LOOP, MODE_PINGPONG_LOOP):
            nxt = 0
        else:
            return (f["rot"], f["sx"], f["sy"], f["px"], f["py"])
    g = frames[nxt]
    return (
        _rot_lerp(f["rot"], g["rot"], frac),
        f["sx"] + mulf32(g["sx"] - f["sx"], frac),
        f["sy"] + mulf32(g["sy"] - f["sy"], frac),
        f["px"] + (((g["px"] - f["px"]) * frac) >> 12),
        f["py"] + (((g["py"] - f["py"]) * frac) >> 12),
    )


def anchor_point(bank, cell_index, anchor=ANCHOR_CENTER):
    """The cell's anchor, in f32 cell-space pixels. Mirrors
    __NEA_CellAnchorPoint()."""
    if not (0 <= cell_index < len(bank["cells"])):
        return (0, 0)
    cell = bank["cells"][cell_index]
    if anchor == ANCHOR_TOPLEFT:
        return (0, 0)
    if anchor == ANCHOR_BOTTOM:
        return ((cell["min_x"] + cell["max_x"]) << 11, cell["max_y"] << 12)
    return ((cell["min_x"] + cell["max_x"]) << 11,
            (cell["min_y"] + cell["max_y"]) << 11)


def instance_xform(bank, cell_index, instance):
    """The caller's own rotation and scale, about the cell's anchor.

    This is runtime state rather than anything the file stores, but it is
    mirrored here because NEACell.c applies it: a preview that left it out
    would quietly disagree with the DS the moment anyone spun a sprite.

    `instance` is {"rot": 0-511, "sx": f32, "sy": f32, "anchor": ANCHOR_*}.
    """
    if not instance:
        return IDENTITY
    rot = instance.get("rot", 0)
    sx = instance.get("sx", FX32_ONE)
    sy = instance.get("sy", FX32_ONE)
    if rot == 0 and sx == FX32_ONE and sy == FX32_ONE:
        return IDENTITY
    ax, ay = anchor_point(bank, cell_index,
                          instance.get("anchor", ANCHOR_CENTER))
    return srt_xform((rot & 511) << 7, sx, sy, ax, ay, 0, 0)


def _part_xform(part, values):
    """A part's own cell-space transform, about its pivot."""
    rot = values.get(CH_ROT, 0) & 0xFFFF
    sx = values.get(CH_SX, FX32_ONE)
    sy = values.get(CH_SY, FX32_ONE)
    tx = values.get(CH_TX, 0)
    ty = values.get(CH_TY, 0)
    if rot == 0 and sx == FX32_ONE and sy == FX32_ONE and tx == 0 and ty == 0:
        return IDENTITY
    return srt_xform(rot, sx, sy,
                     part["pivot_x"] << 12, part["pivot_y"] << 12, tx, ty)


def _pose_cell(bank, cell_index, base, tracks_by_part, out):
    """Resolve one cell's parts under an outer transform.

    Parts are walked in array order, which the exporter guarantees is
    parents-before-children, so a child can read its parent's finished
    transform out of `resolved` without a second pass.
    """
    if not (0 <= cell_index < len(bank["cells"])):
        return
    cell = bank["cells"][cell_index]
    resolved = {}
    # Where this cell's parts start in the flat pool the file stores, so a
    # caller can name a part the same way the C runtime does.
    first = sum(len(c["parts"]) for c in bank["cells"][:cell_index])

    for i, part in enumerate(cell["parts"]):
        values = {}
        for track in tracks_by_part.get(i, ()):
            values[track["channel"]] = track["_value"]

        local = _part_xform(part, values)
        parent = part["parent"]
        if 0 <= parent < i and parent in resolved:
            local = xform_compose(resolved[parent], local)
        resolved[i] = local

        world = xform_compose(base, local)
        tx, ty = xform_apply(world, part["off_x"] << 12, part["off_y"] << 12)

        visible = not (part["flags"] & P_HIDDEN)
        if CH_VISIBLE in values:
            visible = bool(values[CH_VISIBLE])

        src = i
        if CH_PARTSWAP in values:
            swap = values[CH_PARTSWAP]
            if 0 <= swap < len(cell["parts"]):
                src = swap

        out.append({
            "cell": cell_index,
            "part": i,
            "src": src,
            "gpart": first + i,
            "gsrc": first + src,
            "m": [world[0], world[1], world[2], world[3]],
            "tx": tx,
            "ty": ty,
            "color": values.get(CH_COLOR, part["color"]) & 0x7FFF,
            "alpha": max(0, min(31, values.get(CH_ALPHA, part["alpha"]))),
            "priority": min(3, part["priority"]),
            "visible": 1 if visible else 0,
        })


def evaluate_sequence(bank, seq_index, tick_f32, node_ticks=None,
                      instance=None, _depth=0):
    """Resolve a sequence at a tick into a list of posed parts.

    Each entry is {cell, part, src, m[4], tx, ty, color, alpha, priority,
    visible}, where `m` and `tx`/`ty` map part-local pixels to cell space:
    a corner at (u, v) inside the part lands at m*(u,v) + (tx,ty).

    `src` is the part whose pixels to draw, which differs from `part` only
    when a PARTSWAP track is in play. `node_ticks` supplies one play head per
    multi-cell node; without it every node runs off the same clock, which is
    only right for a preview scrubbing to an absolute time. `instance` is the
    caller's own rotation and scale -- see instance_xform().

    Returns [] for an out-of-range sequence rather than raising: a bank that
    is mid-edit in the GUI should show nothing, not a traceback.
    """
    if not (0 <= seq_index < len(bank["sequences"])) or _depth > 4:
        return []
    seq = bank["sequences"][seq_index]
    out = []

    if seq["kind"] == KIND_RIG:
        pos, _finished = resolve_position(seq, tick_f32)
        tracks_by_part = {}
        for track in seq["tracks"]:
            track = dict(track)
            track["_value"] = evaluate_track(track, pos)
            tracks_by_part.setdefault(track["part"], []).append(track)
        base = instance_xform(bank, seq["cell"], instance)
        _pose_cell(bank, seq["cell"], base, tracks_by_part, out)
        return out

    if not seq["frames"]:
        return out

    pos, _finished = resolve_position(seq, tick_f32)
    index, frac = frame_at(seq, pos)
    rot, sx, sy, px, py = _frame_srt(seq, index, frac)
    frame = seq["frames"][index]

    if seq["kind"] == KIND_MULTI:
        mi = frame["target"]
        if not (0 <= mi < len(bank["multicells"])):
            return out
        nodes = bank["multicells"][mi]["nodes"]
        for n, node in enumerate(nodes):
            sub_tick = tick_f32
            if node_ticks is not None and n < len(node_ticks):
                sub_tick = node_ticks[n]
            sub = evaluate_sequence(bank, node["seq"], sub_tick,
                                    _depth=_depth + 1)
            # The node offset and the MULTI frame's own translation are
            # applied when the pose is drawn, not baked into it. The C runtime
            # keeps them on the instance for exactly the same reason: a node
            # is composed at draw time, so its pose has to stay comparable to
            # the pose that sequence produces on its own.
            for entry in sub:
                entry["node"] = n
                entry["node_x"] = node["x"] + px
                entry["node_y"] = node["y"] + py
                entry["node_priority"] = node["priority"]
            out.extend(sub)
        return out

    # KIND_CELL: one whole-cell SRT about the cell origin. That one is
    # retail's; the instance transform on top of it pivots on the anchor.
    base = srt_xform(rot, sx, sy, 0, 0, px << 12, py << 12)
    base = xform_compose(instance_xform(bank, frame["target"], instance), base)
    _pose_cell(bank, frame["target"], base, {}, out)
    return out


# ---------------------------------------------------------------------------
# Storage selection and validation
# ---------------------------------------------------------------------------

def track_byte_size(track, num_ticks):
    if track.get("storage", STORE_KEYS) == STORE_CONST:
        return TRACK_SIZE
    if track["storage"] == STORE_BAKED:
        return TRACK_SIZE + 2 * len(track.get("values", []))
    return TRACK_SIZE + KEY_SIZE * len(track.get("keys", []))


def choose_storage(track, num_ticks):
    """Pick the cheapest storage that represents this track exactly.

    Constant wins outright; otherwise it is a straight size comparison between
    one s16 per tick and eight bytes per keyframe. Channels that cannot bake
    into an s16 without loss stay as keys.
    """
    keys = track.get("keys")
    if not keys:
        return track
    if len({k[-1] for k in keys}) == 1:
        return {"part": track["part"], "channel": track["channel"],
                "storage": STORE_CONST, "interp": track.get("interp", 0),
                "flags": track.get("flags", 0), "value": keys[0][-1]}

    if track["channel"] == CH_COLOR:
        return track  # RGB15 fits an s16 only by luck of the sign bit
    if 2 * num_ticks >= KEY_SIZE * len(keys):
        return track

    try:
        values = [_bake_value(track, t) for t in range(num_ticks)]
    except CellFormatError:
        return track
    return {"part": track["part"], "channel": track["channel"],
            "storage": STORE_BAKED, "interp": track.get("interp", 0),
            "flags": track.get("flags", 0), "values": values}


def _bake_value(track, tick):
    v = evaluate_track(track, tick << 12)
    if track["channel"] in FIXED_CHANNELS:
        return to_1_10_5(v)
    return _s16(v)


def optimize(bank):
    """Re-pick every track's storage mode. Safe to run repeatedly."""
    for seq in bank["sequences"]:
        ticks = sequence_ticks(seq)
        seq["tracks"] = [choose_storage(t, ticks) for t in seq["tracks"]]
    return bank


def validate_oam(bank):
    """What would stop this bank playing on the hardware OBJ backend.

    Returns a list of human-readable strings, empty when the bank is clean.
    The editor shows these live, because the moment an author drags a part to
    an odd size they have left the OAM backend and should find out then, not
    at build time.
    """
    problems = []

    for ci, cell in enumerate(bank["cells"]):
        for pi, part in enumerate(cell["parts"]):
            where = "cell %d part %d" % (ci, pi)
            if part["flags"] & P_NO_3D:
                continue
            dims = (part["src_w"], part["src_h"])
            if dims not in OBJ_SIZE_BY_DIMS:
                near = min(OBJ_SIZE_DIMS,
                           key=lambda d: (d[0] - dims[0]) ** 2
                           + (d[1] - dims[1]) ** 2)
                problems.append(
                    "%s: %dx%d is not an OAM size class (nearest %dx%d)"
                    % (where, dims[0], dims[1], near[0], near[1]))
            elif part["obj_size"] != OBJ_SIZE_BY_DIMS[dims]:
                problems.append(
                    "%s: obj_size %d disagrees with its %dx%d source rect"
                    % (where, part["obj_size"], dims[0], dims[1]))
            if part["src_x"] % 8 or part["src_y"] % 8:
                problems.append(
                    "%s: source origin (%d, %d) is not 8-aligned"
                    % (where, part["src_x"], part["src_y"]))
            if part["gfx_offset"] == 0xFFFFFFFF and \
                    not (part["flags"] & P_NO_OAM):
                problems.append(
                    "%s: no .ncgfx offset, so there is nothing to transfer"
                    % where)
            if part["pal_slot"] > 15:
                problems.append("%s: palette slot %d, hardware has 16"
                                % (where, part["pal_slot"]))

    budget = bank.get("budget") or compute_budget(bank)
    total = sum(budget["max_objs"])
    if total > 128:
        problems.append(
            "peak %d OBJs across all size classes, one engine has 128" % total)
    if budget["max_affine"] > 32:
        problems.append(
            "peak %d transformed parts, one engine has 32 affine matrices "
            "(the surplus falls back to flip-only)" % budget["max_affine"])

    for si, seq in enumerate(bank["sequences"]):
        for track in seq["tracks"]:
            if track["channel"] in CHANNELS_3D_ONLY:
                problems.append(
                    "sequence %d: a %s track has no hardware OBJ equivalent"
                    % (si, CHANNEL_NAMES[track["channel"]]))
                break

    return problems


__all__ = [
    "MAGIC", "VERSION", "CellFormatError",
    "loads", "dumps", "load", "dump",
    "new_bank", "new_atlas", "new_cell", "new_part", "new_sequence",
    "new_frame", "new_track", "new_multicell", "new_node",
    "evaluate_track", "evaluate_sequence", "resolve_position", "frame_at",
    "anchor_point", "instance_xform",
    "ANCHOR_CENTER", "ANCHOR_BOTTOM", "ANCHOR_TOPLEFT",
    "sequence_ticks", "frame_prefix", "compute_bounds", "compute_budget",
    "optimize", "choose_storage", "validate_oam",
    "f32", "from_f32", "mulf32", "divf32", "sin_fx", "cos_fx",
    "srt_xform", "xform_compose", "xform_apply",
    "emit_sin_table_c", "SIN_TABLE",
]
