#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
#
# Copyright (c) 2026 Warioware64
#
# Reads retail NCER / NANR / NCGR / NCLR files and builds a .neacell from them.
#
# The layouts here were read off NitroPaint's implementation, vendored at
# helpSrc/NitroPaint-main/. Where retail semantics and ours differ the retail
# one wins, so that an imported animation plays exactly as it did in the game
# it came from: sequences import as STEP, the whole-cell SRT composes scale
# then rotate then translate about the origin, and durations stay in 1/60 s
# ticks.

"""cell_import.py -- retail NCER/NANR/NCGR/NCLR -> .neacell

    python3 cell_import.py --ncer chars.NCER --nanr chars.NANR \\
        --ncgr chars.NCGR --nclr chars.NCLR --out chars

Writes chars.neacell, chars_atlas.bin, chars_pal.bin, chars.ncgfx and
chars.ncpal, the same set cell_pack.py produces.

Not handled, and reported rather than silently dropped: NMCR multi-cell banks,
NCER VRAM-transfer animation, UCAT extended attributes, and the third-party
NCER variants (Hudson, Ghost Trick, Bomberman Land Touch 2).
"""

import argparse
import os
import struct
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import cell_format as C  # noqa: E402


class ImportError_(Exception):
    pass


# ---------------------------------------------------------------------------
# Compression. Retail graphics and palettes are compressed often enough that an
# importer without this is not much use.
# ---------------------------------------------------------------------------

def decompress(data):
    """Undo an LZ77, LZ11, RLE or Huffman wrapper. Passes plain data through."""
    if len(data) < 4:
        return data

    kind = data[0]
    size = data[1] | (data[2] << 8) | (data[3] << 16)
    if size == 0 and len(data) >= 8:
        size = struct.unpack_from("<I", data, 4)[0]
        body = 8
    else:
        body = 4

    # A plausibility check, the same one NitroPaint's CxIsCompressed* makes:
    # a real header decompresses to more than it occupies and not absurdly more.
    if size == 0 or size > len(data) * 200:
        return data

    try:
        if kind == 0x10:
            return _lz77(data, body, size)
        if kind == 0x11:
            return _lz11(data, body, size)
        if kind == 0x30:
            return _rle(data, body, size)
        if kind in (0x24, 0x28):
            return _huffman(data, body, size, 4 if kind == 0x24 else 8)
    except (IndexError, ValueError):
        # A false positive on the type byte is far more likely than a truncated
        # retail file, so fall back to treating it as raw.
        return data

    return data


def _lz77(data, at, size):
    out = bytearray()
    while len(out) < size:
        flags = data[at]
        at += 1
        for bit in range(8):
            if len(out) >= size:
                break
            if flags & (0x80 >> bit):
                b0, b1 = data[at], data[at + 1]
                at += 2
                length = (b0 >> 4) + 3
                disp = (((b0 & 0xF) << 8) | b1) + 1
                for _ in range(length):
                    out.append(out[-disp])
            else:
                out.append(data[at])
                at += 1
    return bytes(out[:size])


def _lz11(data, at, size):
    out = bytearray()
    while len(out) < size:
        flags = data[at]
        at += 1
        for bit in range(8):
            if len(out) >= size:
                break
            if not (flags & (0x80 >> bit)):
                out.append(data[at])
                at += 1
                continue

            b0 = data[at]
            indicator = b0 >> 4
            if indicator == 0:
                length = (b0 << 4) + (data[at + 1] >> 4) + 0x11
                disp = (((data[at + 1] & 0xF) << 8) | data[at + 2]) + 1
                at += 3
            elif indicator == 1:
                length = (((b0 & 0xF) << 12) | (data[at + 1] << 4)
                          | (data[at + 2] >> 4)) + 0x111
                disp = (((data[at + 2] & 0xF) << 8) | data[at + 3]) + 1
                at += 4
            else:
                length = indicator + 1
                disp = (((b0 & 0xF) << 8) | data[at + 1]) + 1
                at += 2

            for _ in range(length):
                out.append(out[-disp])
    return bytes(out[:size])


def _rle(data, at, size):
    out = bytearray()
    while len(out) < size:
        flag = data[at]
        at += 1
        if flag & 0x80:
            run = (flag & 0x7F) + 3
            out += bytes([data[at]]) * run
            at += 1
        else:
            run = (flag & 0x7F) + 1
            out += data[at:at + run]
            at += run
    return bytes(out[:size])


def _huffman(data, at, size, bits):
    tree_size = (data[at] + 1) * 2
    tree = data[at + 1:at + tree_size]
    at += tree_size

    out = bytearray()
    nibbles = []
    pos = 0
    mask = 0
    window = 0
    node = 0
    root_is_data = False

    while len(out) < size:
        if mask == 0:
            window = struct.unpack_from("<I", data, at)[0]
            at += 4
            mask = 0x80000000
        bit = 1 if (window & mask) else 0
        mask >>= 1

        offset = ((tree[node] & 0x3F) + 1) * 2
        next_node = (node & ~1) + offset + bit
        is_data = tree[node] & (0x80 >> bit)
        node = next_node

        if is_data:
            nibbles.append(tree[node])
            node = 0
            if bits == 8:
                out.append(nibbles.pop())
            elif len(nibbles) == 2:
                out.append(nibbles[0] | (nibbles[1] << 4))
                nibbles = []

    del pos, root_is_data
    return bytes(out[:size])


# ---------------------------------------------------------------------------
# The NNS G2D container
# ---------------------------------------------------------------------------

def nns_blocks(data):
    """Every block in a G2D file, as {tag: (payload_offset, payload_size)}.

    Signatures are byte-reversed on disk -- an NCER opens with 'RECN' and its
    cell block with 'KBEC' -- so everything here compares reversed and reports
    the readable name.
    """
    if len(data) < 16:
        raise ImportError_("file is shorter than an NNS header")

    bom = struct.unpack_from("<H", data, 4)[0]
    if bom not in (0xFFFE, 0xFEFF, 0x0000):
        raise ImportError_("byte order mark is 0x%04X, not an NNS file" % bom)

    header_size, num_blocks = struct.unpack_from("<HH", data, 12)
    if header_size < 16:
        raise ImportError_("header size %d is too small" % header_size)

    # The "old G2D" variant writes block sizes that exclude the 8-byte block
    # header. NitroPaint detects it by the file adding up exactly, and so does
    # this: guessing wrong walks the file off a cliff.
    old = (header_size + num_blocks * 8) == len(data)

    blocks = {}
    at = header_size
    for _ in range(num_blocks):
        if at + 8 > len(data):
            break
        tag = data[at:at + 4][::-1]
        (size,) = struct.unpack_from("<I", data, at + 4)
        payload = at + 8
        payload_size = size if old else size - 8
        if payload_size < 0 or payload + payload_size > len(data):
            payload_size = len(data) - payload
        blocks[tag] = (payload, payload_size)
        at = payload + payload_size
    return blocks


def read_file(path):
    with open(path, "rb") as fp:
        return decompress(fp.read())


# ---------------------------------------------------------------------------
# NCLR -- palettes
# ---------------------------------------------------------------------------

def read_nclr(data):
    blocks = nns_blocks(data)
    if b"PLTT" not in blocks:
        raise ImportError_("no PLTT block: this is not an NCLR")

    at, _size = blocks[b"PLTT"]
    depth_code, _ext, data_size, data_off = struct.unpack_from("<IIII", data, at)
    bits = 1 << (depth_code - 1)

    base = at + data_off
    count = data_size // 2
    colors = list(struct.unpack_from("<%dH" % count, data, base))

    # PCMP maps stored slots onto logical palette numbers, so that a file can
    # omit the 16-colour sub-palettes nothing uses.
    slots = None
    if b"PCMP" in blocks:
        pat, _ = blocks[b"PCMP"]
        n, _pad, table_off = struct.unpack_from("<HHI", data, pat)
        slots = list(struct.unpack_from("<%dH" % n, data, pat + table_off))

    return {"bits": bits, "colors": colors, "slots": slots}


# ---------------------------------------------------------------------------
# NCGR -- character graphics
# ---------------------------------------------------------------------------

def read_ncgr(data):
    blocks = nns_blocks(data)
    if b"CHAR" not in blocks:
        raise ImportError_("no CHAR block: this is not an NCGR")

    at, _size = blocks[b"CHAR"]
    tiles_y, tiles_x = struct.unpack_from("<HH", data, at)
    depth_code, mapping, kind, tile_size, gfx_off = \
        struct.unpack_from("<IIIII", data, at + 4)
    bits = 1 << (depth_code - 1)

    # gfx_off is relative to the start of the CHAR payload.
    base = at + gfx_off
    raw = data[base:base + tile_size]

    per_tile = 32 if bits == 4 else 64
    num_tiles = len(raw) // per_tile

    # 0xFFFF in the height field means 1D mapped, in which case the declared
    # dimensions are meaningless and the tile count is the truth.
    if tiles_y == 0xFFFF or tiles_x == 0 or tiles_x * tiles_y != num_tiles:
        tiles_x = 32 if num_tiles >= 32 else max(1, num_tiles)
        tiles_y = (num_tiles + tiles_x - 1) // tiles_x

    # Expand to one byte per pixel. Every consumer below wants indices, and
    # 4bpp packing is the sort of thing that is worth doing exactly once.
    tiles = []
    for t in range(num_tiles):
        off = t * per_tile
        px = bytearray(64)
        if bits == 4:
            for i in range(32):
                b = raw[off + i]
                px[i * 2] = b & 0xF          # low nibble first
                px[i * 2 + 1] = (b >> 4) & 0xF
        else:
            px[:] = raw[off:off + 64]
        tiles.append(bytes(px))

    return {"bits": bits, "mapping": mapping, "bitmap": kind == 1,
            "tiles_x": tiles_x, "tiles_y": tiles_y, "tiles": tiles}


# ---------------------------------------------------------------------------
# NCER -- cells
# ---------------------------------------------------------------------------

OBJ_DIMS = [
    [(8, 8), (16, 16), (32, 32), (64, 64)],
    [(16, 8), (32, 8), (32, 16), (64, 32)],
    [(8, 16), (8, 32), (16, 32), (32, 64)],
    [(8, 8), (8, 8), (8, 8), (8, 8)],  # shape 3 is prohibited; hardware gives 8x8
]


def _sext(v, bits):
    half = 1 << (bits - 1)
    return v - (1 << bits) if v >= half else v


def decode_oam(attr0, attr1, attr2):
    shape = attr0 >> 14
    size = attr1 >> 14
    w, h = OBJ_DIMS[shape][size]
    return {
        "y": _sext(attr0 & 0xFF, 8),
        "rotscale": (attr0 >> 8) & 1,
        "double_size": (attr0 >> 9) & 1 if (attr0 >> 8) & 1 else 0,
        "disable": (attr0 >> 9) & 1 if not ((attr0 >> 8) & 1) else 0,
        "mode": (attr0 >> 10) & 3,
        "mosaic": (attr0 >> 12) & 1,
        "bits": 8 if (attr0 >> 13) & 1 else 4,
        "x": _sext(attr1 & 0x1FF, 9),
        "matrix": (attr1 >> 9) & 0x1F if (attr0 >> 8) & 1 else 0,
        "hflip": (attr1 >> 12) & 1 if not ((attr0 >> 8) & 1) else 0,
        "vflip": (attr1 >> 13) & 1 if not ((attr0 >> 8) & 1) else 0,
        "char": attr2 & 0x3FF,
        "priority": (attr2 >> 10) & 3,
        "palette": (attr2 >> 12) & 0xF,
        "w": w, "h": h,
    }


def read_ncer(data):
    blocks = nns_blocks(data)
    if b"CEBK" not in blocks:
        raise ImportError_("no CEBK block: this is not a standard NCER "
                           "(Hudson, Ghost Trick and BLDT variants are not "
                           "supported)")

    at, _size = blocks[b"CEBK"]
    num_cells, bank_attribs = struct.unpack_from("<HH", data, at)
    cell_off, mapping_code = struct.unpack_from("<II", data, at + 4)

    per_cell = 16 if (bank_attribs & 1) else 8
    cell_base = at + cell_off
    oam_base = cell_base + num_cells * per_cell

    cells = []
    for i in range(num_cells):
        rec = cell_base + i * per_cell
        num_oam, cell_attr = struct.unpack_from("<HH", data, rec)
        (oam_off,) = struct.unpack_from("<I", data, rec + 4)

        bbox = None
        if bank_attribs & 1:
            # Note the order: max before min, which is what the format says
            # and what reading it the obvious way gets wrong.
            max_x, max_y, min_x, min_y = struct.unpack_from("<hhhh", data,
                                                            rec + 8)
            bbox = (min_x, min_y, max_x, max_y)

        objs = []
        for j in range(num_oam):
            a0, a1, a2 = struct.unpack_from("<HHH", data,
                                            oam_base + oam_off + j * 6)
            obj = decode_oam(a0, a1, a2)
            if not obj["disable"]:
                objs.append(obj)

        cells.append({"objs": objs, "attr": cell_attr, "bbox": bbox})

    mapping_values = [0x000010, 0x100010, 0x200010, 0x300010, 0x000000]
    mapping = mapping_values[mapping_code] if mapping_code < 5 else 0x000010

    return {"cells": cells, "mapping": mapping,
            "has_bbox": bool(bank_attribs & 1)}


# ---------------------------------------------------------------------------
# NANR -- animations
# ---------------------------------------------------------------------------

SEQ_TYPE_INDEX = 0
SEQ_TYPE_INDEX_SRT = 1
SEQ_TYPE_INDEX_T = 2


def read_nanr(data):
    blocks = nns_blocks(data)
    if b"ABNK" not in blocks:
        raise ImportError_("no ABNK block: this is not a standard NANR")

    at, _size = blocks[b"ABNK"]
    num_seqs, _total_frames = struct.unpack_from("<HH", data, at)
    seq_off, frame_off, anim_off = struct.unpack_from("<III", data, at + 4)

    seq_base = at + seq_off
    frame_base = at + frame_off
    anim_base = at + anim_off

    sequences = []
    for i in range(num_seqs):
        rec = seq_base + i * 16
        n_frames, start_index = struct.unpack_from("<HH", data, rec)
        kind, mode, frames_ptr = struct.unpack_from("<III", data, rec + 4)

        element = kind & 0xFFFF
        target = (kind >> 16) & 0xFFFF  # 1 = cell, 2 = multi-cell

        frames = []
        for f in range(n_frames):
            frec = frame_base + frames_ptr + f * 8
            elem_ptr, duration, _pad = struct.unpack_from("<IHH", data, frec)
            e = anim_base + elem_ptr

            # Every element type is widened to SRT here, exactly as
            # AnmGetAnimFrame does, so nothing downstream has to care which
            # of the three encodings the file used.
            if element == SEQ_TYPE_INDEX_SRT:
                index, rot, sx, sy, px, py = struct.unpack_from(
                    "<HHiihh", data, e)
            elif element == SEQ_TYPE_INDEX_T:
                index, _p, px, py = struct.unpack_from("<HHhh", data, e)
                rot, sx, sy = 0, 4096, 4096
            else:
                (index,) = struct.unpack_from("<H", data, e)
                rot, sx, sy, px, py = 0, 4096, 4096, 0, 0

            frames.append({"index": index, "duration": max(1, duration),
                           "rot": rot, "sx": sx, "sy": sy,
                           "px": px, "py": py})

        sequences.append({"frames": frames, "mode": mode if 1 <= mode <= 4
                          else C.MODE_FORWARD_LOOP,
                          "target": target, "start": start_index})

    return {"sequences": sequences}


# ---------------------------------------------------------------------------
# Building the bank
# ---------------------------------------------------------------------------

def obj_pixels(ncgr, obj, mapping):
    """One OBJ's pixels, as a w*h array of palette indices.

    Character names are scaled by the mapping mode's boundary, and the tiles of
    an OBJ are laid out differently under 1D and 2D mapping -- the two things
    that make a cell bank unreadable if either is guessed.
    """
    boundary = 1 << (((mapping >> 20) & 7) + 5)
    start = boundary * obj["char"] // (ncgr["bits"] * 8)

    tw, th = obj["w"] // 8, obj["h"] // 8
    out = bytearray(obj["w"] * obj["h"])

    for ty in range(th):
        for tx in range(tw):
            if mapping == 0x000000:  # 2D
                idx = ((start % ncgr["tiles_x"]) + tx) \
                    + ncgr["tiles_x"] * ((start // ncgr["tiles_x"]) + ty)
            else:                    # 1D
                idx = start + tx + ty * tw

            tile = ncgr["tiles"][idx] if 0 <= idx < len(ncgr["tiles"]) \
                else bytes(64)
            for row in range(8):
                dst = (ty * 8 + row) * obj["w"] + tx * 8
                out[dst:dst + 8] = tile[row * 8:row * 8 + 8]

    return bytes(out)


def build_bank(ncer, nanr, ncgr, nclr, name, atlas_max=512):
    """Retail banks -> one .neacell plus the artwork both backends need."""
    # Cut every OBJ's pixels out once, deduplicated: retail banks reuse the
    # same sprite heavily, and packing each copy would blow the atlas.
    pieces = {}
    order = []
    for cell in ncer["cells"]:
        for obj in cell["objs"]:
            key = (obj["char"], obj["w"], obj["h"], obj["bits"],
                   obj["palette"])
            if key in pieces:
                continue
            px = obj_pixels(ncgr, obj, ncer["mapping"])
            pieces[key] = {"px": px, "w": obj["w"], "h": obj["h"],
                           "bits": obj["bits"], "palette": obj["palette"]}
            order.append(key)

    if not order:
        raise ImportError_("the cell bank has no drawable OBJ")

    # Shelf-pack into one atlas, 8-aligned so the same rect serves the OBJ path.
    size = None
    for candidate in (32, 64, 128, 256, 512, 1024):
        if candidate > atlas_max:
            break
        x = y = shelf = 0
        ok = True
        placed = {}
        for key in order:
            p = pieces[key]
            if x + p["w"] > candidate:
                x = 0
                y += shelf
                shelf = 0
            if y + p["h"] > candidate:
                ok = False
                break
            placed[key] = (x, y)
            x += p["w"]
            shelf = max(shelf, p["h"])
        if ok:
            size = candidate
            break

    if size is None:
        raise ImportError_("the bank's sprites do not fit a %dx%d atlas"
                           % (atlas_max, atlas_max))

    # One 8bpp index plane for the whole atlas. 4bpp OBJs keep their 16-colour
    # sub-palette by having it folded into a single 256-entry table, which is
    # the only way one 3D texture can serve a bank that uses several palettes.
    plane = bytearray(size * size)
    is_4bpp = all(pieces[k]["bits"] == 4 for k in order)

    for key in order:
        p = pieces[key]
        px, py = placed[key]
        base = 0 if p["bits"] == 8 else p["palette"] * 16
        for row in range(p["h"]):
            for col in range(p["w"]):
                v = p["px"][row * p["w"] + col]
                plane[(py + row) * size + px + col] = v if v == 0 \
                    else min(255, base + v)

    palette = list(nclr["colors"])[:256]
    while len(palette) < 256:
        palette.append(0)

    bank = C.new_bank()
    bank["atlases"].append(C.new_atlas(
        name[:31], size, size, C.TEX_PAL256, C.OBJ_COLOR_16 if is_4bpp
        else C.OBJ_COLOR_256, flags=1))

    gfx = bytearray()
    gfx_at = {}

    for ci, cell in enumerate(ncer["cells"]):
        parts = []
        # Retail draws OBJ back to front from the last index, so OBJ 0 is
        # topmost. Ours draws in array order, so reverse: "later is nearer"
        # then holds for all three backends.
        for obj in reversed(cell["objs"]):
            key = (obj["char"], obj["w"], obj["h"], obj["bits"],
                   obj["palette"])
            px, py = placed[key]

            part = C.new_part(px, py, obj["w"], obj["h"],
                              obj["x"], obj["y"])
            part["priority"] = obj["priority"]
            part["pal_slot"] = obj["palette"]
            part["obj_color"] = (C.OBJ_COLOR_256 if obj["bits"] == 8
                                 else C.OBJ_COLOR_16)
            if obj["hflip"]:
                part["flags"] |= C.P_HFLIP
            if obj["vflip"]:
                part["flags"] |= C.P_VFLIP
            if obj["double_size"]:
                part["flags"] |= C.P_DOUBLE_SIZE
            # NCER has no parenting. Leaving every part a root reproduces
            # retail exactly; the hierarchy is purely something an author can
            # add afterwards.
            part["parent"] = -1

            if key not in gfx_at:
                gfx_at[key] = len(gfx)
                gfx += pack_obj_tiles_from_pixels(pieces[key])
            part["gfx_offset"] = gfx_at[key]
            part["gfx_size"] = obj_byte_size(obj["w"], obj["h"], obj["bits"])

            parts.append(part)

        c = C.new_cell(parts)
        if cell["bbox"]:
            c["min_x"], c["min_y"], c["max_x"], c["max_y"] = cell["bbox"]
            c["flags"] |= C.C_BBOX_VALID
        bank["cells"].append(c)

    if nanr:
        for si, seq in enumerate(nanr["sequences"]):
            if seq["target"] == 2:
                print("  note: sequence %d targets a multi-cell bank (NMCR), "
                      "which is not imported; skipped" % si)
                continue
            s = C.new_sequence("seq%d" % si, mode=seq["mode"],
                               interp=C.INTERP_STEP)
            for f in seq["frames"]:
                s["frames"].append(C.new_frame(
                    f["index"], f["duration"], f["sx"], f["sy"],
                    f["rot"], f["px"], f["py"]))
            bank["sequences"].append(s)
    else:
        # No NANR: give every cell a frame so the bank is at least playable.
        s = C.new_sequence("all", mode=C.MODE_FORWARD_LOOP)
        for i in range(len(bank["cells"])):
            s["frames"].append(C.new_frame(i, 4))
        bank["sequences"].append(s)

    C.compute_bounds(bank)
    C.compute_budget(bank)

    pal_bytes = bytearray()
    for c in palette:
        pal_bytes += bytes((c & 0xFF, (c >> 8) & 0xFF))

    return bank, bytes(plane), bytes(pal_bytes), bytes(gfx), size


def obj_byte_size(w, h, bits):
    return (w * h * bits) // 8


def pack_obj_tiles_from_pixels(piece):
    """A piece's own pixels as 8x8 characters, in its own bit depth."""
    out = bytearray()
    w, h, bits = piece["w"], piece["h"], piece["bits"]
    px = piece["px"]
    for ty in range(0, h, 8):
        for tx in range(0, w, 8):
            for row in range(8):
                base = (ty + row) * w + tx
                if bits == 4:
                    for col in range(0, 8, 2):
                        out.append((px[base + col] & 0xF)
                                   | ((px[base + col + 1] & 0xF) << 4))
                else:
                    out += px[base:base + 8]
    return bytes(out)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--ncer", required=True)
    ap.add_argument("--nanr")
    ap.add_argument("--ncgr", required=True)
    ap.add_argument("--nclr", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--name")
    ap.add_argument("--bank-ext", default=".neacell")
    ap.add_argument("--atlas-max", type=int, default=512)
    args = ap.parse_args()

    name = args.name or os.path.basename(args.out)

    ncer = read_ncer(read_file(args.ncer))
    ncgr = read_ncgr(read_file(args.ncgr))
    nclr = read_nclr(read_file(args.nclr))
    nanr = read_nanr(read_file(args.nanr)) if args.nanr else None

    bank, plane, palette, gfx, size = build_bank(
        ncer, nanr, ncgr, nclr, name, atlas_max=args.atlas_max)

    C.dump(bank, args.out + args.bank_ext)
    open(args.out + "_atlas.bin", "wb").write(plane)
    open(args.out + "_pal.bin", "wb").write(palette)
    open(args.out + ".ncgfx", "wb").write(gfx)
    open(args.out + ".ncpal", "wb").write(palette)

    print("%d cells, %d sequences, %dx%d atlas, %d bytes of OBJ tiles"
          % (len(bank["cells"]), len(bank["sequences"]), size, size, len(gfx)))
    for p in C.validate_oam(bank):
        print("  " + p)


if __name__ == "__main__":
    main()
