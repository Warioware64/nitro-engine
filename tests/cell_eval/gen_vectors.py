#!/usr/bin/env python3
# SPDX-License-Identifier: CC0-1.0
#
# SPDX-FileContributor: Warioware64, 2026
#
# Generates the cross-check vectors for the NEACell evaluator.
#
# The editor's live preview is only honest if cell_format.py evaluates a bank
# exactly the way NEACell.c does. Nothing enforces that on its own, and a drift
# between the two would show up as a preview that quietly disagrees with the
# hardware -- the worst kind of bug for an authoring tool.
#
# So this writes two files from one source of truth: the .neacell binary, and a
# C table of what the Python evaluator says every tick of it resolves to. The
# test ROM loads the binary, evaluates it with the real runtime, and compares.
# If the two implementations ever diverge, the ROM says which tick, which
# sequence and which part.

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "tools", "cell_editor"))

import cell_format as C  # noqa: E402

NUM_TICKS = 48

# The instance transform is runtime state rather than anything the file stores,
# so it used to go unchecked -- and a rotation about the wrong pivot reached an
# example before anyone noticed. Every sequence is now evaluated a second time
# under each of these.
INSTANCES = [
    None,
    {"rot": 96, "sx": C.FX32_ONE, "sy": C.FX32_ONE,
     "anchor": C.ANCHOR_CENTER},
    {"rot": 400, "sx": C.f32(1.75), "sy": C.f32(0.5),
     "anchor": C.ANCHOR_BOTTOM},
    {"rot": -40, "sx": -C.FX32_ONE, "sy": C.FX32_ONE,
     "anchor": C.ANCHOR_TOPLEFT},
]


def build_bank():
    """A bank chosen so that nothing can drift quietly.

    Every sequence kind, every playback mode, every storage mode, both
    interpolation modes, a three-deep parent chain, a negative scale composed
    with a rotation (where a sign error in the 2x2 hides), a rotation past a
    full turn, non-zero pivots with non-uniform scale, and a multi-cell whose
    two nodes have coprime lengths so a phase bug cannot stay hidden behind a
    coincidence.
    """
    bank = C.new_bank()
    bank["atlases"].append(C.new_atlas("test", 128, 128))

    # Cell 0: a three-deep chain with pivots that are not part centres.
    root = C.new_part(0, 0, 32, 32, -16, -16, gfx_offset=0, gfx_size=512)
    root["pivot_x"], root["pivot_y"] = 0, 0
    mid = C.new_part(32, 0, 16, 16, 16, -8, parent=0,
                     gfx_offset=512, gfx_size=128)
    mid["pivot_x"], mid["pivot_y"] = 16, 0
    tip = C.new_part(48, 0, 8, 8, 32, -4, parent=1,
                     gfx_offset=640, gfx_size=32, priority=2)
    tip["pivot_x"], tip["pivot_y"] = 32, 0
    bank["cells"].append(C.new_cell([root, mid, tip]))

    # Cell 1: two parts, no hierarchy, one of them tinted and translucent.
    a = C.new_part(0, 32, 64, 32, -32, 0, gfx_offset=672, gfx_size=1024)
    b = C.new_part(64, 32, 8, 16, 8, -16, gfx_offset=1696, gfx_size=64,
                   color=0x1F, alpha=17, priority=1)
    bank["cells"].append(C.new_cell([a, b]))

    # Cell 2: a single part, flipped.
    only = C.new_part(0, 64, 16, 32, -8, -32, gfx_offset=1760, gfx_size=256,
                      flags=C.P_HFLIP)
    bank["cells"].append(C.new_cell([only]))

    # Frames shared by the CELL sequences. Durations are deliberately not 1,
    # which is what pins the tick-to-frame mapping.
    def frames():
        return [
            C.new_frame(0, 1),
            C.new_frame(1, 5, rot=8192, px=12, py=-7),
            # A negative X scale composed with a rotation: the case where a
            # sign error in the 2x2 goes unnoticed on symmetric artwork.
            C.new_frame(2, 3, rot=45000, sx=-C.FX32_ONE, sy=C.f32(1.5),
                        px=-20, py=30),
            C.new_frame(0, 7, rot=60000, sx=C.f32(0.5), sy=C.f32(0.5)),
        ]

    for name, mode, interp in (
        ("step", C.MODE_FORWARD_LOOP, C.INTERP_STEP),
        ("lerp", C.MODE_FORWARD_LOOP, C.INTERP_LINEAR),
        ("once", C.MODE_FORWARD, C.INTERP_STEP),
        ("pingpong", C.MODE_PINGPONG, C.INTERP_STEP),
        ("ppLoop", C.MODE_PINGPONG_LOOP, C.INTERP_LINEAR),
    ):
        seq = C.new_sequence(name, mode=mode, interp=interp)
        seq["frames"] = frames()
        bank["sequences"].append(seq)

    # A rig: keyframe tracks on the three-deep chain, one of every storage mode.
    rig = C.new_sequence("rig", kind=C.KIND_RIG, mode=C.MODE_FORWARD_LOOP)
    rig["cell"] = 0
    rig["num_frames"] = 40
    rig["tracks"] = [
        # More than a full turn, so the wrap is exercised.
        C.new_track(0, C.CH_ROT, keys=[[0, 0], [39, 70000]]),
        # Non-uniform scale on a part whose pivot is not its centre.
        C.new_track(0, C.CH_SY, keys=[[0, C.FX32_ONE], [20, C.f32(1.75)],
                                      [39, C.f32(0.4)]]),
        # Baked: one int16 per tick, widened by << 7 on read.
        C.new_track(1, C.CH_TX, storage=C.STORE_BAKED,
                    values=[(i * 137) % 900 - 450 for i in range(40)]),
        # Negative scale again, this time animated.
        C.new_track(1, C.CH_SX, keys=[[0, C.FX32_ONE], [39, -C.FX32_ONE]]),
        # Constant: no array, no search.
        C.new_track(2, C.CH_ALPHA, storage=C.STORE_CONST, value=13),
        # Stepped even though the track says linear, because a half-visible
        # part is not a thing.
        C.new_track(2, C.CH_VISIBLE, keys=[[0, 1], [15, 0], [30, 1]]),
        # A wrapping angle that goes the short way round the other side of 0.
        C.new_track(2, C.CH_ROT, keys=[[0, 64000], [39, 2000]]),
    ]
    bank["sequences"].append(rig)

    # A multi-cell whose nodes are 16 and 40 ticks long -- coprime, so the two
    # clocks only realign every 80 ticks and a phase bug shows up inside the
    # window this test covers.
    bank["multicells"].append(C.new_multicell("pair", [
        C.new_node(seq=0, x=10, y=-4, priority=1),
        C.new_node(seq=5, x=-6, y=12),
    ]))
    multi = C.new_sequence("multi", kind=C.KIND_MULTI,
                           mode=C.MODE_FORWARD_LOOP)
    multi["frames"] = [C.new_frame(0, 9, px=3, py=5)]
    bank["sequences"].append(multi)

    return bank


def poses_for(bank, seq_index, tick, instance=None):
    """What the sequence resolves to at an integer tick.

    Multi-cell nodes run their own clocks, and the runtime advances every node
    once per update from the moment it was seated, so a node's play head is
    the same tick as the parent's. Mirror that here.
    """
    return C.evaluate_sequence(bank, seq_index, tick << 12,
                               node_ticks=None, instance=instance)


def emit_vectors(bank, path):
    seqs = bank["sequences"]

    counts = []
    poses = []
    for inst in INSTANCES:
        for si in range(len(seqs)):
            row = []
            for tick in range(NUM_TICKS):
                entries = poses_for(bank, si, tick, inst)
                row.append(len(entries))
                poses.extend(entries)
            counts.append(row)

    lines = [
        "// Generated by gen_vectors.py. Do not edit.",
        "//",
        "// Expected output of the NEACell evaluator, produced by the Python",
        "// implementation in tools/cell_editor/cell_format.py. The test ROM",
        "// checks the C runtime against these values tick by tick.",
        "",
        "#ifndef CELL_VECTORS_H__",
        "#define CELL_VECTORS_H__",
        "",
        "#include <stdint.h>",
        "",
        "#define VEC_NUM_SEQS      %d" % len(seqs),
        "#define VEC_NUM_TICKS     %d" % NUM_TICKS,
        "#define VEC_NUM_POSES     %d" % len(poses),
        "#define VEC_NUM_INSTANCES %d" % len(INSTANCES),
        "",
        "typedef struct {",
        "    int32_t m0, m1, m2, m3;",
        "    int32_t tx, ty;",
        "    uint16_t color;",
        "    uint16_t part, src;",
        "    uint8_t alpha, priority, visible;",
        "} vec_pose_t;",
        "",
        "static const char *const vec_seq_names[VEC_NUM_SEQS] = {",
    ]
    for seq in seqs:
        lines.append('    "%s",' % seq["name"])
    lines.append("};")
    lines.append("")

    lines.append("typedef struct {")
    lines.append("    int rot;")
    lines.append("    int32_t sx, sy;")
    lines.append("    int anchor;")
    lines.append("} vec_instance_t;")
    lines.append("")
    lines.append("static const vec_instance_t "
                 "vec_instances[VEC_NUM_INSTANCES] = {")
    for inst in INSTANCES:
        if inst is None:
            lines.append("    { 0, 4096, 4096, 0 },")
        else:
            lines.append("    { %d, %d, %d, %d },"
                         % (inst["rot"], inst["sx"], inst["sy"],
                            inst["anchor"]))
    lines.append("};")
    lines.append("")

    lines.append("static const uint8_t "
                 "vec_counts[VEC_NUM_INSTANCES * VEC_NUM_SEQS]"
                 "[VEC_NUM_TICKS] = {")
    for row in counts:
        lines.append("    { %s }," % ", ".join(str(v) for v in row))
    lines.append("};")
    lines.append("")

    lines.append("static const vec_pose_t vec_poses[VEC_NUM_POSES] = {")
    for e in poses:
        lines.append(
            "    { %d, %d, %d, %d, %d, %d, 0x%04X, %d, %d, %d, %d, %d },"
            % (e["m"][0], e["m"][1], e["m"][2], e["m"][3],
               e["tx"], e["ty"], e["color"], e["gpart"], e["gsrc"],
               e["alpha"], e["priority"], e["visible"]))
    lines.append("};")
    lines.append("")

    # The sine table is the one place the two implementations could drift
    # without any track or frame looking wrong, so it gets checked directly.
    lines.append("#define VEC_SIN_SIZE %d" % C.SIN_TABLE_SIZE)
    lines.append("static const int16_t vec_sin[VEC_SIN_SIZE] = {")
    for i in range(0, C.SIN_TABLE_SIZE, 8):
        lines.append("    %s," % ", ".join("%6d" % v
                                           for v in C.SIN_TABLE[i:i + 8]))
    lines.append("};")
    lines.append("")
    lines.append("#endif // CELL_VECTORS_H__")

    with open(path, "w") as fp:
        fp.write("\n".join(lines) + "\n")

    return len(poses)


def main():
    bank = build_bank()

    data_dir = os.path.join(HERE, "data")
    src_dir = os.path.join(HERE, "source")
    os.makedirs(data_dir, exist_ok=True)
    os.makedirs(src_dir, exist_ok=True)

    binary = os.path.join(data_dir, "vectors.bin")
    C.dump(bank, binary)

    num_poses = emit_vectors(bank, os.path.join(src_dir, "vectors.h"))

    # The runtime's table is generated from the same source, so regenerate it
    # here too rather than letting it rot.
    sin_path = os.path.join(HERE, "..", "..", "source", "NEACellSin.h")
    with open(sin_path, "w") as fp:
        fp.write(C.emit_sin_table_c())

    problems = C.validate_oam(bank)

    print("wrote %s (%d bytes)" % (binary, os.path.getsize(binary)))
    print("wrote source/vectors.h: %d instances x %d sequences x %d ticks, "
          "%d poses" % (len(INSTANCES), len(bank["sequences"]), NUM_TICKS,
                        num_poses))
    print("regenerated source/NEACellSin.h")
    if problems:
        print("\nOAM notes (expected -- this bank is deliberately awkward):")
        for p in problems:
            print("  " + p)


if __name__ == "__main__":
    main()
