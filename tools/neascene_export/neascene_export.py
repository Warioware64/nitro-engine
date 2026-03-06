#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
#
# Copyright (c) 2024-2026 Warioware64
#
# Converts a JSON scene description into a binary .neascene file.
# The JSON is produced by the Blender addon during scene export.
#
# Usage:
#     python3 neascene_export.py --input scene.json --output level.neascene

"""neascene_export.py -- JSON to binary .neascene converter.

Binary format (all values little-endian):

FILE HEADER (16 bytes)
    magic:              uint32  0x4E53434E ("NSCN")
    version:            uint32  1
    num_nodes:          uint16
    num_assets:         uint16
    num_mat_refs:       uint16
    active_camera_idx:  uint16  (0xFFFF = none)

ASSET TABLE (64 bytes each)
    path:       char[48]  (null-padded)
    asset_type: uint8     (0=static, 1=dsm, 2=dsa, 3=colmesh, 4=boncol)
    padding:    uint8[15]

MATERIAL REF TABLE (80 bytes each)
    name:       char[32]  (null-padded)
    tex_path:   char[48]  (null-padded)

NODE TABLE (128 bytes each)
    name:       char[24]
    type:       uint8     (0=empty, 1=mesh, 2=camera, 3=trigger)
    parent_idx: uint8     (0xFF = root)
    num_tags:   uint8
    flags:      uint8     (bit 0 = visible)
    position:   int32[3]  (f32 fixed-point)
    rotation:   int16[3]  (0-511)
    padding:    int16
    scale:      int32[3]  (f32 fixed-point)
    type_data:  20 bytes
    tags:       48 bytes  (3 * 16)
"""

import argparse
import json
import struct
import sys

NSCN_MAGIC = 0x4E53434E
NSCN_VERSION = 1

HEADER_SIZE = 16
ASSET_SIZE = 64
MATREF_SIZE = 80
NODE_SIZE = 128

NODE_NAME_LEN = 24
TAG_LEN = 16
MAX_TAGS = 6

TYPE_EMPTY = 0
TYPE_MESH = 1
TYPE_CAMERA = 2
TYPE_TRIGGER = 3


def float_to_f32(val):
    """Convert a float to NDS 20.12 fixed-point (signed int32).

    Uses the same pattern as display_list.py float_to_v16.
    """
    res = int(val * (1 << 12))
    if res < -0x80000000:
        raise OverflowError(f"{val} too small for f32: {res:#010x}")
    if res > 0x7FFFFFFF:
        raise OverflowError(f"{val} too big for f32: {res:#010x}")
    if res < 0:
        res = 0x100000000 + res
    return res


def pack_string(s, length):
    """Pack a string into a fixed-length null-padded bytes object."""
    encoded = s.encode('utf-8')[:length - 1]
    return encoded + b'\x00' * (length - len(encoded))


def pack_asset(asset):
    """Pack a single asset entry (64 bytes)."""
    data = pack_string(asset.get('path', ''), 48)
    data += struct.pack('<B', asset.get('type', 0))
    data += b'\x00' * 15  # padding
    assert len(data) == ASSET_SIZE
    return data


def pack_matref(mat):
    """Pack a single material reference (80 bytes)."""
    data = pack_string(mat.get('name', ''), 32)
    data += pack_string(mat.get('tex_path', ''), 48)
    assert len(data) == MATREF_SIZE
    return data


def pack_node(node):
    """Pack a single node entry (128 bytes)."""
    buf = bytearray(NODE_SIZE)

    # Name (24 bytes at offset 0)
    name = pack_string(node.get('name', ''), NODE_NAME_LEN)
    buf[0:NODE_NAME_LEN] = name

    # Type, parent, num_tags, flags (offset 24-27)
    type_str = node.get('type', 'empty')
    type_map = {'empty': TYPE_EMPTY, 'mesh': TYPE_MESH,
                'camera': TYPE_CAMERA, 'trigger': TYPE_TRIGGER}
    buf[24] = type_map.get(type_str, TYPE_EMPTY)

    parent_idx = node.get('parent_idx', 0xFF)
    buf[25] = parent_idx & 0xFF

    tags = node.get('tags', [])[:MAX_TAGS]
    buf[26] = len(tags)

    flags = 0
    if node.get('visible', True):
        flags |= 1
    buf[27] = flags

    # Position (f32 x3 at offset 28)
    pos = node.get('position', [0.0, 0.0, 0.0])
    struct.pack_into('<III', buf, 28,
                     float_to_f32(pos[0]),
                     float_to_f32(pos[1]),
                     float_to_f32(pos[2]))

    # Rotation (int16 x3 at offset 40, + 2 bytes padding)
    rot = node.get('rotation', [0, 0, 0])
    struct.pack_into('<hhhh', buf, 40,
                     rot[0] & 0x1FF, rot[1] & 0x1FF, rot[2] & 0x1FF, 0)

    # Scale (f32 x3 at offset 48)
    scl = node.get('scale', [1.0, 1.0, 1.0])
    struct.pack_into('<III', buf, 48,
                     float_to_f32(scl[0]),
                     float_to_f32(scl[1]),
                     float_to_f32(scl[2]))

    # Type-specific data at offset 60
    if type_str == 'mesh':
        mesh = node.get('mesh', {})
        struct.pack_into('<HHB', buf, 60,
                         mesh.get('asset_index', 0),
                         mesh.get('material_index', 0xFFFF),
                         1 if mesh.get('is_animated', False) else 0)
    elif type_str == 'camera':
        cam = node.get('camera', {})
        to = cam.get('to', [0.0, 0.0, -1.0])  # default: look along -Z
        up = cam.get('up', [0.0, 1.0, 0.0])    # default: Y-up
        struct.pack_into('<IIIIII', buf, 60,
                         float_to_f32(to[0]), float_to_f32(to[1]),
                         float_to_f32(to[2]),
                         float_to_f32(up[0]), float_to_f32(up[1]),
                         float_to_f32(up[2]))
    elif type_str == 'trigger':
        trig = node.get('trigger', {})
        shape_type = {'sphere': 1, 'aabb': 2}.get(
            trig.get('shape', 'sphere'), 1)
        buf[60] = shape_type
        buf[61] = trig.get('script_id', 0)
        if shape_type == 1:  # sphere
            struct.pack_into('<I', buf, 64,
                             float_to_f32(trig.get('radius', 1.0)))
        elif shape_type == 2:  # aabb
            struct.pack_into('<III', buf, 64,
                             float_to_f32(trig.get('half_x', 1.0)),
                             float_to_f32(trig.get('half_y', 1.0)),
                             float_to_f32(trig.get('half_z', 1.0)))

    # Tags at offset 80 (3 * 16 = 48 bytes)
    for t in range(min(len(tags), MAX_TAGS)):
        tag_bytes = pack_string(tags[t], TAG_LEN)
        buf[80 + t * TAG_LEN:80 + (t + 1) * TAG_LEN] = tag_bytes

    return bytes(buf)


def convert(input_path, output_path):
    """Read JSON and write binary .neascene."""
    with open(input_path, 'r') as f:
        scene = json.load(f)

    nodes = scene.get('nodes', [])
    assets = scene.get('assets', [])
    mat_refs = scene.get('materials', [])
    active_camera_idx = scene.get('active_camera_idx', 0xFFFF)

    if len(nodes) == 0:
        print("ERROR: No nodes in scene")
        sys.exit(1)

    # Pack header
    header = struct.pack('<IIHHHH',
                         NSCN_MAGIC,
                         NSCN_VERSION,
                         len(nodes),
                         len(assets),
                         len(mat_refs),
                         active_camera_idx)
    assert len(header) == HEADER_SIZE

    with open(output_path, 'wb') as f:
        f.write(header)

        for asset in assets:
            f.write(pack_asset(asset))

        for mat in mat_refs:
            f.write(pack_matref(mat))

        for node in nodes:
            f.write(pack_node(node))

    total = HEADER_SIZE + len(assets) * ASSET_SIZE + \
        len(mat_refs) * MATREF_SIZE + len(nodes) * NODE_SIZE
    print(f"  Scene: {len(nodes)} nodes, {len(assets)} assets, "
          f"{len(mat_refs)} materials -> {output_path} ({total} bytes)")


def main():
    parser = argparse.ArgumentParser(
        description="Convert JSON scene description to binary .neascene")
    parser.add_argument("--input", "-i", required=True,
                        help="Input JSON file")
    parser.add_argument("--output", "-o", required=True,
                        help="Output .neascene binary file")
    args = parser.parse_args()

    convert(args.input, args.output)
    print("Done!")


if __name__ == '__main__':
    main()
