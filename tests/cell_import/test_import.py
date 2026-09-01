#!/usr/bin/env python3
# SPDX-License-Identifier: CC0-1.0
#
# SPDX-FileContributor: Warioware64, 2026
#
# Host test for the retail importer.
#
# There is no retail asset in this repository to import, and there should not
# be one, so this synthesises NCLR/NCGR/NCER/NANR files in the layouts
# NitroPaint documents, runs them through cell_import.py, and checks that what
# comes out the far end is what went in: the OBJ rectangles, the draw order,
# the frame durations, and the scale/rotate/translate composition, which is
# the part with a sign error waiting in it.
#
#     python3 tests/cell_import/test_import.py

import math
import os
import struct
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "tools", "cell_editor"))

import cell_format as C      # noqa: E402
import cell_import as I      # noqa: E402

failures = []


def check(name, got, want):
    if got != want:
        failures.append("%s: got %r, expected %r" % (name, got, want))


def nns_file(magic, blocks):
    """An NNS G2D container: 16-byte header, then blocks with reversed tags."""
    body = b""
    for tag, payload in blocks:
        body += tag[::-1] + struct.pack("<I", len(payload) + 8) + payload
    total = 16 + len(body)
    header = magic[::-1] + struct.pack("<HBBIHH", 0xFFFE, 0, 1, total, 16,
                                       len(blocks))
    return header + body


def make_nclr():
    # 32 colours: slot 0 is a red ramp, slot 1 a blue one, so a palette mix-up
    # in the importer is visible rather than subtle.
    colors = []
    for i in range(16):
        colors.append((i * 2) | (0 << 5) | (0 << 10))
    for i in range(16):
        colors.append(0 | (0 << 5) | ((i * 2) << 10))
    data = struct.pack("<IIII", 3, 0, len(colors) * 2, 0x10)
    data += struct.pack("<%dH" % len(colors), *colors)
    return nns_file(b"NCLR", [(b"PLTT", data)])


def make_ncgr(num_tiles=16):
    # Tile n is filled with index (n % 15) + 1, so which tile an OBJ picked up
    # can be read straight off the imported pixels.
    raw = bytearray()
    for t in range(num_tiles):
        v = (t % 15) + 1
        raw += bytes([v | (v << 4)]) * 32
    data = struct.pack("<HHIIIII", 0xFFFF, 0xFFFF, 3, 0x000010, 0,
                       len(raw), 0x18)
    data += bytes(raw)
    return nns_file(b"NCGR", [(b"CHAR", data)])


def make_ncer():
    # Two cells. The first has two OBJ so the draw-order flip is testable; the
    # second is one flipped 16x16 with a non-zero palette.
    cells = [
        [  # (char, x, y, shape, size, palette, hflip, vflip, priority)
            (0, -16, -8, 0, 1, 0, 0, 0, 0),   # 16x16 at (-16, -8)
            (4, 8, 0, 2, 1, 0, 0, 0, 1),      # 8x32
        ],
        [
            (8, 0, -16, 0, 1, 1, 1, 0, 2),    # 16x16, h-flipped, palette 1
        ],
    ]

    oam_blob = b""
    cell_recs = b""
    for objs in cells:
        cell_recs += struct.pack("<HHI", len(objs), 0, len(oam_blob))
        for (char, x, y, shape, size, pal, hf, vf, prio) in objs:
            attr0 = (y & 0xFF) | (shape << 14)
            attr1 = (x & 0x1FF) | (hf << 12) | (vf << 13) | (size << 14)
            attr2 = (char & 0x3FF) | (prio << 10) | (pal << 12)
            oam_blob += struct.pack("<HHH", attr0, attr1, attr2)

    data = struct.pack("<HHIIIII", len(cells), 0, 0x18, 0, 0, 0, 0)
    data += cell_recs + oam_blob
    return nns_file(b"NCER", [(b"CEBK", data)])


def make_nanr():
    # One INDEX_SRT sequence: two frames with different durations, the second
    # rotated 90 degrees with a negative X scale -- the case a sign error in
    # the 2x2 hides in.
    frames = [
        (0, 3, 0, 4096, 4096, 0, 0),
        (1, 7, 16384, -4096, 6144, 12, -5),
    ]

    elements = b""
    frame_recs = b""
    for (index, duration, rot, sx, sy, px, py) in frames:
        frame_recs += struct.pack("<IHH", len(elements), duration, 0xBEEF)
        elements += struct.pack("<HHiihh", index, rot, sx, sy, px, py)

    seq_rec = struct.pack("<HHIII", len(frames), 0,
                          I.SEQ_TYPE_INDEX_SRT | (1 << 16),
                          C.MODE_FORWARD_LOOP, 0)

    seq_off = 0x18
    frame_off = seq_off + 16
    anim_off = frame_off + len(frame_recs)

    data = struct.pack("<HHIIIII", 1, len(frames), seq_off, frame_off,
                       anim_off, 0, 0)
    data += seq_rec + frame_recs + elements
    return nns_file(b"NANR", [(b"ABNK", data)])


def main():
    ncer = I.read_ncer(make_ncer())
    ncgr = I.read_ncgr(make_ncgr())
    nclr = I.read_nclr(make_nclr())
    nanr = I.read_nanr(make_nanr())

    check("nclr bits", nclr["bits"], 4)
    check("nclr colours", len(nclr["colors"]), 32)
    check("ncgr tiles", len(ncgr["tiles"]), 16)
    check("ncer cells", len(ncer["cells"]), 2)
    check("cell 0 objs", len(ncer["cells"][0]["objs"]), 2)

    obj = ncer["cells"][0]["objs"][0]
    check("obj 0 size", (obj["w"], obj["h"]), (16, 16))
    check("obj 0 pos", (obj["x"], obj["y"]), (-16, -8))

    tall = ncer["cells"][0]["objs"][1]
    check("obj 1 size", (tall["w"], tall["h"]), (8, 32))

    flipped = ncer["cells"][1]["objs"][0]
    check("flip decoded", flipped["hflip"], 1)
    check("palette decoded", flipped["palette"], 1)

    check("nanr sequences", len(nanr["sequences"]), 1)
    seq = nanr["sequences"][0]
    check("frame durations", [f["duration"] for f in seq["frames"]], [3, 7])
    check("frame 1 srt", (seq["frames"][1]["rot"], seq["frames"][1]["sx"],
                          seq["frames"][1]["sy"]), (16384, -4096, 6144))

    bank, plane, palette, gfx, size = I.build_bank(ncer, nanr, ncgr, nclr,
                                                   "test")

    check("bank cells", len(bank["cells"]), 2)
    check("bank sequences", len(bank["sequences"]), 1)
    check("bank seq ticks", C.sequence_ticks(bank["sequences"][0]), 10)

    # Retail's OBJ 0 is topmost; ours draws in array order, so the import has
    # to reverse. The 8x32 was OBJ 1 in the file and must come out first.
    parts = bank["cells"][0]["parts"]
    check("draw order flipped", (parts[0]["src_w"], parts[0]["src_h"]),
          (8, 32))
    check("second part", (parts[1]["src_w"], parts[1]["src_h"]), (16, 16))
    check("part 1 offset", (parts[1]["off_x"], parts[1]["off_y"]), (-16, -8))
    check("obj size class", parts[1]["obj_size"],
          C.OBJ_SIZE_BY_DIMS[(16, 16)])
    check("flip preserved", bool(bank["cells"][1]["parts"][0]["flags"]
                                 & C.P_HFLIP), True)

    # Pixels: the atlas rect for the first cell's 16x16 should be tile 0's
    # fill value, which is 1.
    px, py = parts[1]["src_x"], parts[1]["src_y"]
    check("atlas pixel", plane[py * size + px], 1)

    # The whole point of keeping retail's fixed point: frame 1 of the imported
    # sequence has to land where NitroPaint's AnmCalcTransformMatrix says it
    # does. Compare against a float evaluation of the same formula.
    pose = C.evaluate_sequence(bank, 0, 5 << 12)  # inside frame 1
    part = bank["cells"][1]["parts"][0]

    sx, sy = -4096 / 4096.0, 6144 / 4096.0
    rot = (16384 / 65536.0) * 2 * math.pi
    m00 = sx * math.cos(rot)
    m01 = -sy * math.sin(rot)
    m10 = sx * math.sin(rot)
    m11 = sy * math.cos(rot)
    want_x = m00 * part["off_x"] + m01 * part["off_y"] + 12
    want_y = m10 * part["off_x"] + m11 * part["off_y"] - 5

    got_x = C.from_f32(pose[0]["tx"])
    got_y = C.from_f32(pose[0]["ty"])

    # One 4096th is the fixed-point step; a quarter pixel is generous room for
    # the table's angle quantisation and still catches a sign error.
    if abs(got_x - want_x) > 0.25 or abs(got_y - want_y) > 0.25:
        failures.append("SRT: got (%.3f, %.3f), NitroPaint's formula says "
                        "(%.3f, %.3f)" % (got_x, got_y, want_x, want_y))

    # And the file has to survive a round trip.
    raw = C.dumps(bank)
    again = C.loads(raw)
    check("round trip", C.dumps(again), raw)

    if failures:
        print("FAIL (%d)" % len(failures))
        for f in failures:
            print("  " + f)
        return 1

    print("PASS: %d cells, %d sequences, %dx%d atlas, %d bytes of OBJ tiles"
          % (len(bank["cells"]), len(bank["sequences"]), size, size, len(gfx)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
