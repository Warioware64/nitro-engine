#!/usr/bin/env python3
# SPDX-License-Identifier: CC0-1.0
#
# SPDX-FileContributor: Warioware64, 2026
#
# Draws the composition example's pieces and builds the multi-cell bank.
#
# cell_pack.py handles the common case -- a uniform spritesheet becomes one
# cell per tile and one sequence stepping through them. A multi-cell is the
# other shape: several pieces, each with its own sequence running on its own
# clock, composed into one entity. There is no grid that expresses that, so
# this builds the bank with cell_format directly, the way a game's own asset
# script would.

import math
import os
import sys

from PIL import Image, ImageDraw

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.join(HERE, "..", "..", "..", "..", "tools", "cell_editor")
sys.path.insert(0, os.path.abspath(TOOLS))

import cell_format as C   # noqa: E402
import cell_pack          # noqa: E402

# The pieces below all sit in the top 32 rows, so the atlas is 128x32 rather
# than a square 128x128. The DS sizes the two axes of a texture
# independently; rounding both up to the larger one would cost four times the
# texture VRAM for nothing.
ATLAS_W = 128
ATLAS_H = 32

BODY = (86, 132, 196, 255)
BODY_DARK = (52, 84, 140, 255)
SKIN = (236, 196, 152, 255)
SKIN_DARK = (188, 148, 108, 255)
CLOTH = (208, 72, 88, 255)
POLE = (120, 108, 96, 255)


def draw_pieces():
    """Four pieces, each drawn once. The motion comes from the sequences."""
    img = Image.new("RGBA", (ATLAS_W, ATLAS_H), (0, 0, 0, 0))
    d = ImageDraw.Draw(img)

    # body: 32x32 at (0, 0)
    d.rounded_rectangle([8, 4, 24, 30], radius=5, fill=BODY)
    d.rectangle([8, 22, 24, 30], fill=BODY_DARK)
    d.rectangle([10, 30, 14, 31], fill=BODY_DARK)
    d.rectangle([18, 30, 22, 31], fill=BODY_DARK)

    # head: 16x16 at (32, 0)
    d.ellipse([34, 1, 46, 14], fill=SKIN)
    d.ellipse([34, 1, 46, 6], fill=SKIN_DARK)
    d.rectangle([37, 7, 38, 8], fill=(30, 30, 40, 255))
    d.rectangle([42, 7, 43, 8], fill=(30, 30, 40, 255))

    # arm: 8x16 at (48, 0). Drawn hanging from its top edge, because that is
    # where the cell origin goes and therefore what it swings about.
    d.rounded_rectangle([50, 0, 53, 12], radius=2, fill=BODY)
    d.ellipse([49, 11, 54, 15], fill=SKIN)

    # flag: 16x16 at (56, 0). Anchored at its top-left for the same reason.
    d.rectangle([56, 0, 57, 15], fill=POLE)
    d.polygon([(58, 1), (70, 4), (58, 8)], fill=CLOTH)
    d.polygon([(58, 8), (66, 10), (58, 12)], fill=(168, 48, 64, 255))

    return img


PIECES = {
    # name: (src rect, part offset from the cell origin)
    "body": ((0, 0, 32, 32), (-16, -32)),
    "head": ((32, 0, 16, 16), (-8, -16)),
    "arm":  ((48, 0, 8, 16), (-4, 0)),
    "flag": ((56, 0, 16, 16), (0, 0)),
}


def build_bank(indices, name="hero"):
    bank = C.new_bank()
    bank["atlases"].append(C.new_atlas(name, ATLAS_W, ATLAS_H, C.TEX_PAL16,
                                       C.OBJ_COLOR_16, flags=1))

    gfx = bytearray()
    cell_of = {}

    for piece, ((sx, sy, w, h), (ox, oy)) in PIECES.items():
        part = C.new_part(sx, sy, w, h, ox, oy)
        part["gfx_offset"] = len(gfx)
        tiles = cell_pack.pack_obj_tiles(indices, ATLAS_W, sx, sy, w, h, 16)
        part["gfx_size"] = len(tiles)
        gfx += tiles
        cell_of[piece] = len(bank["cells"])
        bank["cells"].append(C.new_cell([part]))

    def sequence(nm, cell, frames):
        """Motion is per-frame translate and rotate, interpolated.

        Rotation in a frame is about the *cell origin*, which is retail's rule
        and the reason each piece above is drawn hanging from the point it
        should pivot on: the arm swings from its shoulder because its origin
        is its shoulder.
        """
        seq = C.new_sequence(nm, mode=C.MODE_FORWARD_LOOP,
                             interp=C.INTERP_LINEAR)
        for duration, rot_deg, px, py in frames:
            seq["frames"].append(C.new_frame(
                cell, duration,
                rot=int(round(rot_deg / 360.0 * 65536)) & 0xFFFF,
                px=px, py=py))
        bank["sequences"].append(seq)
        return len(bank["sequences"]) - 1

    # Lengths are deliberately coprime -- 24, 20, 15 and 28 ticks -- so the
    # four clocks only realign every 14 seconds. Anything that resampled the
    # nodes onto one shared frame counter would be obvious within a second.
    body = sequence("body", cell_of["body"], [
        (12, 0, 0, 0), (12, 0, 0, -2)])
    head = sequence("head", cell_of["head"], [
        (7, -6, 0, 0), (6, 0, 0, -2), (7, 6, 0, 0)])
    arm = sequence("arm", cell_of["arm"], [
        (5, -28, 0, 0), (5, 10, 0, 0), (5, 22, 0, 0)])
    flag = sequence("flag", cell_of["flag"], [
        (7, 0, 0, 0), (7, 9, 0, -1), (7, 0, 0, 0), (7, -9, 0, 1)])

    # Nodes are drawn in array order, so the flag goes behind the body and the
    # head in front of it.
    bank["multicells"].append(C.new_multicell("hero", [
        C.new_node(seq=flag, x=-15, y=-46),
        C.new_node(seq=body, x=0, y=0),
        C.new_node(seq=arm, x=11, y=-26),
        C.new_node(seq=head, x=0, y=-30),
    ]))

    # A second multi-cell with the same nodes but no flag, so the example can
    # show that swapping the composition keeps every surviving node's phase.
    bank["multicells"].append(C.new_multicell("plain", [
        C.new_node(seq=body, x=0, y=0),
        C.new_node(seq=arm, x=11, y=-26),
        C.new_node(seq=head, x=0, y=-30),
    ]))

    C.compute_bounds(bank)
    C.compute_budget(bank)
    return bank, bytes(gfx)


def main():
    img = draw_pieces()
    img.save(os.path.join(HERE, "hero.png"))

    indices, palette = cell_pack.quantise(img, 16)
    texture = cell_pack.pack_texture(indices, ATLAS_W, ATLAS_H, 16)
    bank, gfx = build_bank(indices)

    pal = bytearray()
    for c in palette:
        pal += bytes((c & 0xFF, (c >> 8) & 0xFF))

    out = os.path.join(HERE, "..", "data")
    C.dump(bank, os.path.join(out, "hero_cells.bin"))
    open(os.path.join(out, "hero_atlas.bin"), "wb").write(texture)
    open(os.path.join(out, "hero_pal.bin"), "wb").write(bytes(pal))
    open(os.path.join(out, "hero_gfx.bin"), "wb").write(gfx)

    print("hero.png, %d cells, %d sequences, %d multi-cells, %d bytes of tiles"
          % (len(bank["cells"]), len(bank["sequences"]),
             len(bank["multicells"]), len(gfx)))
    for p in C.validate_oam(bank):
        print("  " + p)


if __name__ == "__main__":
    main()
