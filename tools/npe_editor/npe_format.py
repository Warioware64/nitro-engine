#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
#
# Copyright (c) 2026 Warioware64
#
# NPE (Nitro Particle Entity) binary format reader/writer. The C runtime in
# NEAParticle.c reads exactly the layout this module writes -- keep them in
# lockstep.

"""npe_format.py -- read/write NPE binary files.

Binary layout (little-endian; mirrors NEAParticle.c's `NEA_ParticleEmitterLoad`):

    HEADER (16 bytes)
        magic       u32   'NPE1' (0x3145504E)
        version     u32   1
        flags       u32   bit0 additive, bit1 continuous, bit2 axis-aligned,
                          bit4 spritesheet, bit5 stretch
        reserved    u32

    EMITTER (80 bytes)
        speed_min       s32  f32
        speed_max       s32  f32
        drag            s32  f32
        pos_min[3]      s32  f32
        pos_max[3]      s32  f32
        gravity[3]      s32  f32
        max_particles   u16
        emit_rate       u16  particles/sec, 8.8 fixed
        burst_count     u16
        cone_spread     u16  0..511
        life_min        u16  frames
        life_max        u16  frames
        base_size       u16  8.8 fixed
        base_rotation   u16  0..511
        ang_vel_min     s16
        ang_vel_max     s16
        initial_dir[3]  s16  1.15 fixed normalized
        sheet_cols      u8
        sheet_rows      u8
        sheet_fps       u16
        pad             u8[2]

    MATERIAL REF (52 bytes)
        mat_name        char[32]  null-padded; looked up at runtime via
                                  NEA_MaterialFindByName()
        pad             u8[20]

    COLOR KEYS
        num_keys        u16
        num_keys * { u16 t (0..1000), u8 r, u8 g, u8 b, u8 a }  (6 B each)

    SIZE KEYS
        num_keys        u16
        num_keys * { u16 t (0..1000), u16 size_8_8 }            (4 B each)

A typical effect is ~150 bytes.
"""

import struct

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

NPE_MAGIC   = 0x3145504E  # 'N','P','E','1' little-endian
NPE_VERSION = 1
NPE_MAX_KEYS = 16

FLAG_ADDITIVE     = 1 << 0
FLAG_CONTINUOUS   = 1 << 1
FLAG_AXIS_ALIGNED = 1 << 2
FLAG_SPRITESHEET  = 1 << 4
FLAG_STRETCH      = 1 << 5

# ---------------------------------------------------------------------------
# Helpers: f32 fixed-point conversion. The runtime uses 1.19.12 fixed-point
# (12 fractional bits) for "f32" values.
# ---------------------------------------------------------------------------

def f32(x):
    """Convert a Python float to libnds f32 (s32 with 12 fractional bits)."""
    v = int(round(x * 4096))
    if v < -(1 << 31): v = -(1 << 31)
    if v > ( 1 << 31) - 1: v = (1 << 31) - 1
    return v

def from_f32(v):
    """Inverse of f32()."""
    return v / 4096.0

def fx8_8(x):
    """Convert a Python float to 8.8 fixed (u16 / s16)."""
    v = int(round(x * 256))
    if v < 0: v = 0
    if v > 0xFFFF: v = 0xFFFF
    return v

def from_fx8_8(v):
    return v / 256.0

def fx1_15(x):
    """Convert a [-1, 1] float to 1.15 fixed (s16)."""
    v = int(round(x * 32768))
    if v < -32768: v = -32768
    if v >  32767: v =  32767
    return v

def from_fx1_15(v):
    return v / 32768.0

# ---------------------------------------------------------------------------
# Default effect (the one the editor opens with).
# ---------------------------------------------------------------------------

def default_effect():
    return {
        "flags": {
            "additive": False,
            "continuous": True,
            "axis_aligned": False,
            "spritesheet": False,
            "stretch": False,
        },
        "max_particles": 64,
        "emit_rate":     30.0,    # particles per second
        "burst_count":   16,
        "cone_spread":   60,      # 0..511
        "life_min_frames": 30,
        "life_max_frames": 90,
        "speed_min":     0.05,
        "speed_max":     0.15,
        "drag":          0.02,
        "pos_min":       [-0.1, 0.0, -0.1],
        "pos_max":       [ 0.1, 0.0,  0.1],
        "initial_dir":   [ 0.0, 1.0,  0.0],
        "gravity":       [ 0.0, 0.0,  0.0],
        "base_size":     0.5,
        "base_rotation": 0,
        "ang_vel_min":   0,
        "ang_vel_max":   0,
        "sheet_cols":    1,
        "sheet_rows":    1,
        "sheet_fps":     0,
        "mat_name":      "",
        "color_keys": [
            (0,    255, 255, 255, 255),
            (1000, 255, 255, 255,   0),
        ],
        "size_keys": [
            (0,    1.0),
            (1000, 0.0),
        ],
    }

# ---------------------------------------------------------------------------
# Encode (dict -> bytes)
# ---------------------------------------------------------------------------

def _flags_to_u32(eff):
    f = eff.get("flags", {})
    v = 0
    if f.get("additive"):     v |= FLAG_ADDITIVE
    if f.get("continuous"):   v |= FLAG_CONTINUOUS
    if f.get("axis_aligned"): v |= FLAG_AXIS_ALIGNED
    if f.get("spritesheet"):  v |= FLAG_SPRITESHEET
    if f.get("stretch"):      v |= FLAG_STRETCH
    return v

def encode(effect):
    """Serialize an effect dict to NPE bytes."""
    out = bytearray()

    # Header (16 B)
    out += struct.pack("<IIII",
                       NPE_MAGIC, NPE_VERSION,
                       _flags_to_u32(effect),
                       0)  # reserved

    # Emitter (80 B)
    emitter = struct.pack(
        "<iii"          # speed_min, speed_max, drag (s32 f32)
        "iii"           # pos_min[3]
        "iii"           # pos_max[3]
        "iii"           # gravity[3]
        "HHHH"          # max_particles, emit_rate, burst_count, cone_spread
        "HHHH"          # life_min, life_max, base_size, base_rotation
        "hh"            # ang_vel_min, ang_vel_max
        "hhh"           # initial_dir[3] (1.15)
        "BB"            # sheet_cols, sheet_rows
        "H"             # sheet_fps
        "2x",           # pad
        f32(effect["speed_min"]),
        f32(effect["speed_max"]),
        f32(effect["drag"]),
        f32(effect["pos_min"][0]), f32(effect["pos_min"][1]), f32(effect["pos_min"][2]),
        f32(effect["pos_max"][0]), f32(effect["pos_max"][1]), f32(effect["pos_max"][2]),
        f32(effect["gravity"][0]), f32(effect["gravity"][1]), f32(effect["gravity"][2]),
        int(effect["max_particles"]) & 0xFFFF,
        fx8_8(effect["emit_rate"]),
        int(effect["burst_count"]) & 0xFFFF,
        int(effect["cone_spread"]) & 0xFFFF,
        int(effect["life_min_frames"]) & 0xFFFF,
        int(effect["life_max_frames"]) & 0xFFFF,
        fx8_8(effect["base_size"]),
        int(effect["base_rotation"]) & 0xFFFF,
        int(effect["ang_vel_min"]),
        int(effect["ang_vel_max"]),
        fx1_15(effect["initial_dir"][0]),
        fx1_15(effect["initial_dir"][1]),
        fx1_15(effect["initial_dir"][2]),
        int(effect["sheet_cols"]) & 0xFF,
        int(effect["sheet_rows"]) & 0xFF,
        int(effect["sheet_fps"]) & 0xFFFF,
    )
    assert len(emitter) == 80, f"emitter block must be 80 bytes, got {len(emitter)}"
    out += emitter

    # Material ref (52 B: 32 name + 20 pad)
    name = effect.get("mat_name", "")
    name_b = name.encode("ascii", errors="replace")[:31]
    out += name_b + b"\x00" * (32 - len(name_b)) + b"\x00" * 20

    # Color keys
    cks = effect["color_keys"][:NPE_MAX_KEYS]
    out += struct.pack("<H", len(cks))
    for (t, r, g, b, a) in cks:
        out += struct.pack("<HBBBB", int(t) & 0xFFFF,
                           int(r) & 0xFF, int(g) & 0xFF,
                           int(b) & 0xFF, int(a) & 0xFF)

    # Size keys
    sks = effect["size_keys"][:NPE_MAX_KEYS]
    out += struct.pack("<H", len(sks))
    for (t, sz) in sks:
        out += struct.pack("<HH", int(t) & 0xFFFF, fx8_8(sz))

    return bytes(out)

# ---------------------------------------------------------------------------
# Decode (bytes -> dict)
# ---------------------------------------------------------------------------

def decode(data):
    """Parse NPE bytes back into an effect dict."""
    if len(data) < 16 + 80 + 52 + 2 + 2:
        raise ValueError("NPE data too short")

    magic, version, flags, _reserved = struct.unpack_from("<IIII", data, 0)
    if magic != NPE_MAGIC:
        raise ValueError(f"Bad magic 0x{magic:08X}")
    if version != NPE_VERSION:
        raise ValueError(f"Unsupported version {version}")

    p = 16
    (speed_min, speed_max, drag,
     pmin0, pmin1, pmin2,
     pmax0, pmax1, pmax2,
     g0, g1, g2,
     max_particles, emit_rate, burst_count, cone_spread,
     life_min, life_max, base_size, base_rotation,
     ang_vel_min, ang_vel_max,
     dir0, dir1, dir2,
     sheet_cols, sheet_rows, sheet_fps) = struct.unpack_from(
        "<iiiiiiiiiiiiHHHHHHHHhhhhhBBH2x", data, p)
    p += 80

    mat_name = data[p:p+32].split(b"\x00", 1)[0].decode("ascii", errors="replace")
    p += 52

    nck = struct.unpack_from("<H", data, p)[0]; p += 2
    color_keys = []
    for _ in range(nck):
        t, r, g, b, a = struct.unpack_from("<HBBBB", data, p)
        color_keys.append((t, r, g, b, a))
        p += 6

    nsk = struct.unpack_from("<H", data, p)[0]; p += 2
    size_keys = []
    for _ in range(nsk):
        t, sz = struct.unpack_from("<HH", data, p)
        size_keys.append((t, from_fx8_8(sz)))
        p += 4

    return {
        "flags": {
            "additive":     bool(flags & FLAG_ADDITIVE),
            "continuous":   bool(flags & FLAG_CONTINUOUS),
            "axis_aligned": bool(flags & FLAG_AXIS_ALIGNED),
            "spritesheet":  bool(flags & FLAG_SPRITESHEET),
            "stretch":      bool(flags & FLAG_STRETCH),
        },
        "max_particles":   max_particles,
        "emit_rate":       from_fx8_8(emit_rate),
        "burst_count":     burst_count,
        "cone_spread":     cone_spread,
        "life_min_frames": life_min,
        "life_max_frames": life_max,
        "speed_min":       from_f32(speed_min),
        "speed_max":       from_f32(speed_max),
        "drag":            from_f32(drag),
        "pos_min":         [from_f32(pmin0), from_f32(pmin1), from_f32(pmin2)],
        "pos_max":         [from_f32(pmax0), from_f32(pmax1), from_f32(pmax2)],
        "initial_dir":     [from_fx1_15(dir0), from_fx1_15(dir1), from_fx1_15(dir2)],
        "gravity":         [from_f32(g0), from_f32(g1), from_f32(g2)],
        "base_size":       from_fx8_8(base_size),
        "base_rotation":   base_rotation,
        "ang_vel_min":     ang_vel_min,
        "ang_vel_max":     ang_vel_max,
        "sheet_cols":      sheet_cols,
        "sheet_rows":      sheet_rows,
        "sheet_fps":       sheet_fps,
        "mat_name":        mat_name,
        "color_keys":      color_keys,
        "size_keys":       [(t, sz) for (t, sz) in size_keys],
    }

# ---------------------------------------------------------------------------
# Self-test: roundtrip the default effect.
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    eff = default_effect()
    data = encode(eff)
    print(f"encoded {len(data)} bytes")
    back = decode(data)
    assert back["max_particles"]   == eff["max_particles"]
    assert back["burst_count"]     == eff["burst_count"]
    assert back["cone_spread"]     == eff["cone_spread"]
    assert back["life_min_frames"] == eff["life_min_frames"]
    assert back["mat_name"]        == eff["mat_name"]
    assert len(back["color_keys"]) == len(eff["color_keys"])
    assert len(back["size_keys"])  == len(eff["size_keys"])
    print("OK: round-trip preserves fields")
