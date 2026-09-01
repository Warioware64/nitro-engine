#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
#
# Copyright (c) 2026 Warioware64
#
# Builds a .neacell and its companion artwork from a PNG spritesheet.
#
# The two backends want the same pixels in two different orders: the 3D path
# needs a linear power-of-two texture, and the hardware OBJ path needs 8x8
# tiles in 1D mapping order, cut per part into separately addressable blocks.
# So both come out of here, from one quantisation, which is what stops the two
# renderers disagreeing about what a colour index means.

"""cell_pack.py -- PNG spritesheet -> .neacell + atlas + .ncgfx + .ncpal

    python3 cell_pack.py sheet.png --grid 8x4 --fps 12 --out walk

writes walk.neacell, walk_atlas.bin, walk_pal.bin, walk.ncgfx and walk.ncpal.

The atlas and its palette are ordinary NDS texture data: load them with
NEA_MaterialTexLoad() / NEA_PaletteLoad() under the name the .neacell expects.
The .ncgfx must stay resident in main RAM -- it is what the hardware backend
transfers into OBJ VRAM as the animation plays.
"""

import argparse
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "img2ds"))

import cell_format as C  # noqa: E402

try:
    from PIL import Image
except ImportError:
    print("cell_pack needs Pillow: pip install Pillow", file=sys.stderr)
    raise


# ---------------------------------------------------------------------------
# Slicing
# ---------------------------------------------------------------------------

def slice_grid(img, cols, rows):
    w, h = img.size
    fw, fh = w // cols, h // rows
    out = []
    for r in range(rows):
        for c in range(cols):
            out.append(img.crop((c * fw, r * fh, (c + 1) * fw, (r + 1) * fh)))
    return out


def trim_and_fit(tile):
    """Trim a piece to its alpha bounding box, then grow it to an OAM size.

    Growing rather than scaling is deliberate: a sprite the hardware can draw
    has to be one of twelve exact sizes, and padding with transparency is the
    only way to get there without touching a single authored pixel.

    Returns (image, offset_x, offset_y) where the offset says where the trimmed
    content sat inside the original piece.
    """
    bbox = tile.getbbox()
    if bbox is None:
        return None, 0, 0

    trimmed = tile.crop(bbox)
    tw, th = trimmed.size

    fit = None
    for w, h in C.OBJ_SIZE_DIMS:
        if w >= tw and h >= th:
            if fit is None or w * h < fit[0] * fit[1]:
                fit = (w, h)

    if fit is None:
        # Bigger than 64x64 in some direction. cell_pack does not split yet, so
        # say so plainly rather than emitting something the hardware silently
        # will not draw.
        return trimmed, bbox[0], bbox[1]

    padded = Image.new("RGBA", fit, (0, 0, 0, 0))
    padded.paste(trimmed, (0, 0))
    return padded, bbox[0], bbox[1]


# ---------------------------------------------------------------------------
# Packing
# ---------------------------------------------------------------------------

TEX_SIZES = (8, 16, 32, 64, 128, 256, 512, 1024)


def _shelf_pack(pieces, width, height):
    """Try to shelf-pack into width x height. Returns placements or None."""
    placements = []
    x = y = shelf_h = 0
    for img in pieces:
        w, h = img.size
        w8 = (w + 7) & ~7
        h8 = (h + 7) & ~7
        if w8 > width:
            return None
        if x + w8 > width:
            x = 0
            y += shelf_h
            shelf_h = 0
        if y + h8 > height:
            return None
        placements.append((x, y))
        x += w8
        shelf_h = max(shelf_h, h8)
    return placements


def pack_atlas(pieces, max_size=256):
    """Shelf-pack into the smallest power-of-two atlas that fits.

    Width and height are chosen independently, because the DS sizes the two
    axes of a texture independently and a square atlas rounds both up. Eight
    32x32 frames need 128x64; forcing a square would make it 128x128 and cost
    twice the texture VRAM for nothing, which is the sort of waste that is
    invisible until something else runs out.

    Candidates are tried smallest-area first, and squarer shapes win ties.
    Origins stay 8-aligned throughout, because an OBJ's graphics have to start
    on a tile boundary and the same rectangle feeds both backends.
    """
    candidates = [(w, h) for w in TEX_SIZES for h in TEX_SIZES
                  if w <= max_size and h <= max_size]
    candidates.sort(key=lambda wh: (wh[0] * wh[1], abs(wh[0] - wh[1])))

    for width, height in candidates:
        placements = _shelf_pack(pieces, width, height)
        if placements is not None:
            return width, height, placements

    raise SystemExit("cell_pack: the pieces do not fit a %dx%d atlas; "
                     "split the sheet or raise --atlas-max" % (max_size,
                                                               max_size))


# ---------------------------------------------------------------------------
# Quantisation
# ---------------------------------------------------------------------------

def quantise(atlas, colors):
    """RGBA atlas -> (indices, RGB15 palette), index 0 transparent.

    Index 0 is reserved for transparency on both backends -- the 3D texture is
    loaded with COLOR0_TRANSPARENT and the hardware OBJ treats index 0 the same
    way -- so the usable palette is one entry shorter than it looks.
    """
    rgba = atlas.convert("RGBA")
    # tobytes() rather than getdata(): the latter is deprecated in new Pillow
    # and the replacement does not exist in old Pillow, and this module has to
    # work with whatever the user already has.
    raw = rgba.tobytes()

    table = []
    index_of = {}
    indices = []
    lost = 0
    for i in range(0, len(raw), 4):
        r, g, b, a = raw[i], raw[i + 1], raw[i + 2], raw[i + 3]
        if a < 128:
            indices.append(0)
            continue
        key = (r >> 3, g >> 3, b >> 3)
        if key not in index_of:
            if len(table) + 1 >= colors:
                # Nearest existing colour. Quietly losing a shade beats
                # failing an export over one stray anti-aliased pixel, but say
                # so once so it is not a surprise.
                best = min(table, key=lambda c: (c[0] - key[0]) ** 2
                           + (c[1] - key[1]) ** 2 + (c[2] - key[2]) ** 2)
                index_of[key] = index_of[best]
                lost += 1
            else:
                table.append(key)
                index_of[key] = len(table)  # 1-based; 0 is transparent
        indices.append(index_of[key])

    if lost:
        print("  note: %d colours did not fit %d entries and were snapped to "
              "the nearest kept shade" % (lost, colors))

    # Colour 0 is magenta so it stands out in a VRAM viewer, the same choice
    # img2ds makes.
    palette = [(31 << 0) | (0 << 5) | (31 << 10)]
    palette += [r | (g << 5) | (b << 10) for r, g, b in table]
    while len(palette) < colors:
        palette.append(0)

    return indices, palette


def pack_texture(indices, width, height, colors):
    """Indices -> the linear texture bytes the 3D engine expects."""
    out = bytearray()
    if colors <= 16:
        for i in range(0, len(indices), 2):
            out.append((indices[i] & 0xF) | ((indices[i + 1] & 0xF) << 4))
    else:
        for i in indices:
            out.append(i & 0xFF)
    return bytes(out)


def pack_obj_tiles(indices, atlas_w, x, y, w, h, colors):
    """One part's pixels as 8x8 characters in 1D mapping order.

    This is the layout OBJ VRAM wants and the linear texture above is not, so
    the same indices get walked a second time in tile order.
    """
    out = bytearray()
    for ty in range(0, h, 8):
        for tx in range(0, w, 8):
            for row in range(8):
                base = (y + ty + row) * atlas_w + (x + tx)
                if colors <= 16:
                    for col in range(0, 8, 2):
                        lo = indices[base + col] & 0xF
                        hi = indices[base + col + 1] & 0xF
                        out.append(lo | (hi << 4))
                else:
                    for col in range(8):
                        out.append(indices[base + col] & 0xFF)
    return bytes(out)


# ---------------------------------------------------------------------------
# Building a bank
# ---------------------------------------------------------------------------

def build(png, cols, rows, name, fps=12, colors=16, atlas_max=256,
          mode=C.MODE_FORWARD_LOOP, anchor_bottom=False):
    """One cell per grid tile, and one sequence stepping through them."""
    img = Image.open(png).convert("RGBA")
    tiles = slice_grid(img, cols, rows)

    pieces = []
    offsets = []
    tile_w, tile_h = img.size[0] // cols, img.size[1] // rows
    for tile in tiles:
        fitted, ox, oy = trim_and_fit(tile)
        pieces.append(fitted)
        offsets.append((ox, oy))

    live = [(i, p) for i, p in enumerate(pieces) if p is not None]
    if not live:
        raise SystemExit("cell_pack: every tile is fully transparent")

    aw, ah, placements = pack_atlas([p for _i, p in live], atlas_max)

    atlas = Image.new("RGBA", (aw, ah), (0, 0, 0, 0))
    rects = {}
    for (i, piece), (px, py) in zip(live, placements):
        atlas.paste(piece, (px, py))
        rects[i] = (px, py, piece.size[0], piece.size[1])

    indices, palette = quantise(atlas, colors)
    texture = pack_texture(indices, aw, ah, colors)

    bank = C.new_bank()
    bank["atlases"].append(C.new_atlas(
        name[:31], aw, ah,
        C.TEX_PAL16 if colors <= 16 else C.TEX_PAL256,
        C.OBJ_COLOR_16 if colors <= 16 else C.OBJ_COLOR_256,
        flags=1))

    gfx = bytearray()
    seq = C.new_sequence(name[:15], mode=mode)
    duration = max(1, round(60 / max(1, fps)))

    for i in range(len(pieces)):
        if i not in rects:
            # A blank frame still has to exist, or the timing changes.
            bank["cells"].append(C.new_cell([]))
            seq["frames"].append(C.new_frame(len(bank["cells"]) - 1, duration))
            continue

        px, py, pw, ph = rects[i]
        ox, oy = offsets[i]

        # Cells are centred on the tile, so a sequence whose frames trim to
        # different sizes does not wander.
        off_x = ox - tile_w // 2
        off_y = oy - tile_h if anchor_bottom else oy - tile_h // 2

        part = C.new_part(px, py, pw, ph, off_x, off_y)
        part["obj_color"] = C.OBJ_COLOR_16 if colors <= 16 else C.OBJ_COLOR_256
        part["gfx_offset"] = len(gfx)

        tiles_bytes = pack_obj_tiles(indices, aw, px, py, pw, ph, colors)
        part["gfx_size"] = len(tiles_bytes)
        gfx += tiles_bytes

        if part["obj_size"] == C.OBJ_SIZE_NONE:
            part["flags"] |= C.P_NO_OAM
            print("  note: frame %d is %dx%d, which no OBJ size class holds; "
                  "the hardware backend will skip it" % (i, pw, ph))

        bank["cells"].append(C.new_cell([part]))
        seq["frames"].append(C.new_frame(len(bank["cells"]) - 1, duration))

    bank["sequences"].append(seq)
    C.compute_bounds(bank)
    C.compute_budget(bank)

    pal_bytes = bytearray()
    for c in palette:
        pal_bytes += bytes((c & 0xFF, (c >> 8) & 0xFF))

    return bank, texture, bytes(pal_bytes), bytes(gfx), (aw, ah)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("png")
    ap.add_argument("--grid", required=True,
                    help="sheet layout, COLSxROWS, e.g. 8x4")
    ap.add_argument("--out", required=True, help="output basename")
    ap.add_argument("--name", help="atlas/material name (default: --out)")
    ap.add_argument("--fps", type=int, default=12)
    ap.add_argument("--colors", type=int, default=16, choices=(16, 256))
    ap.add_argument("--atlas-max", type=int, default=256)
    ap.add_argument("--bank-ext", default=".neacell",
                    help="extension for the bank itself. Use .bin when the "
                         "bank is shipped in an example's data/ directory, "
                         "because the build only runs bin2c over *.bin")
    ap.add_argument("--anchor-bottom", action="store_true",
                    help="put the cell origin at the bottom of the tile, so a "
                         "billboard's feet meet the ground")
    args = ap.parse_args()

    cols, rows = (int(v) for v in args.grid.lower().split("x"))
    name = args.name or os.path.basename(args.out)

    bank, texture, palette, gfx, (aw, ah) = build(
        args.png, cols, rows, name, fps=args.fps, colors=args.colors,
        atlas_max=args.atlas_max, anchor_bottom=args.anchor_bottom)

    C.dump(bank, args.out + args.bank_ext)
    open(args.out + "_atlas.bin", "wb").write(texture)
    open(args.out + "_pal.bin", "wb").write(palette)
    open(args.out + ".ncgfx", "wb").write(gfx)
    open(args.out + ".ncpal", "wb").write(palette)

    print("%s%-11s %d cells, %d sequences"
          % (args.out, args.bank_ext, len(bank["cells"]),
             len(bank["sequences"])))
    print("%s_atlas.bin  %dx%d, %d colours, %d bytes"
          % (args.out, aw, ah, args.colors, len(texture)))
    print("%s.ncgfx      %d bytes of OBJ tiles" % (args.out, len(gfx)))
    print("budget: %s objs, %d affine, %d byte transfer"
          % (bank["budget"]["max_objs"], bank["budget"]["max_affine"],
             bank["budget"]["max_transfer"]))

    for p in C.validate_oam(bank):
        print("  " + p)


if __name__ == "__main__":
    main()
