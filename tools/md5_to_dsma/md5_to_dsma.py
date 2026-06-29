#!/usr/bin/env python3

# SPDX-License-Identifier: MIT
#
# Copyright (c) 2022 Antonio Niño Díaz <antonio_nd@outlook.com>

import os
import struct

from collections import namedtuple, defaultdict
from math import sqrt

from display_list import DisplayList, float_to_f32, float_to_n10

class MD5FormatError(Exception):
    pass

VALID_TEXTURE_SIZES = [8, 16, 32, 64, 128, 256, 512, 1024]

def is_valid_texture_size(size):
    return size in VALID_TEXTURE_SIZES

def get_image_dimensions(path):
    """Read width and height from a PNG or JPEG file without external deps."""
    with open(path, 'rb') as f:
        header = f.read(32)

        # PNG: signature (8 bytes) then IHDR chunk
        if header[:8] == b'\x89PNG\r\n\x1a\n':
            w = struct.unpack('>I', header[16:20])[0]
            h = struct.unpack('>I', header[20:24])[0]
            return w, h

        # JPEG: scan for SOF0/SOF2 marker
        if header[:2] == b'\xff\xd8':
            f.seek(0)
            data = f.read()
            i = 2
            while i < len(data) - 9:
                if data[i] != 0xFF:
                    i += 1
                    continue
                marker = data[i + 1]
                if marker in (0xC0, 0xC2):
                    h = struct.unpack('>H', data[i+5:i+7])[0]
                    w = struct.unpack('>H', data[i+7:i+9])[0]
                    return w, h
                if marker == 0xD9:
                    break
                if marker in (0xD0, 0xD1, 0xD2, 0xD3, 0xD4, 0xD5,
                               0xD6, 0xD7, 0xD8, 0x01):
                    i += 2
                else:
                    seg_len = struct.unpack('>H', data[i+2:i+4])[0]
                    i += 2 + seg_len

    raise ValueError(f"Cannot read dimensions from {path}")


DLMM_MAGIC = 0x4D4D4C44  # "DLMM" in little-endian
DLMM_VERSION = 1
DLMM_SUBMESH_HEADER_SIZE = 56  # bytes per submesh header

# NSMW (NitroSkin MultiWeight) - two-weight smooth skinning format
NSMW_MAGIC = 0x574D534E  # "NSMW" in little-endian
NSMW_VERSION = 1
NSMW_HEADER_SIZE = 24     # magic, version, num_nodes, num_joints,
                          # num_submeshes, flags
NSMW_NODE_SIZE = 12       # num_weights/joint0/joint1/pad + 2 * weight (f32)
NSMW_INVBIND_SIZE = 48    # 12 int32 (4x3 matrix)
NSMW_MAX_NODES = 30       # matrix-stack budget (must match NEA_MAX_SKIN_NODES)

def save_dlmm(output_file, submeshes):
    """Write multi-material model to .dlmm binary format.

    Args:
        output_file: output file path
        submeshes: list of dicts with keys:
            'name': material name (str, max 31 chars)
            'dl': finalized DisplayList
            'diffuse_ambient': u32
            'specular_emission': u32
            'color': u16 RGB15
            'alpha': u16 (0-31)
            'has_texture': bool
    """
    num = len(submeshes)
    header_size = 12 + DLMM_SUBMESH_HEADER_SIZE * num

    dl_binaries = []
    for sub in submeshes:
        dl_binaries.append(sub['dl'].get_binary())

    dl_offsets = []
    offset = header_size
    for dl_bin in dl_binaries:
        dl_offsets.append(offset)
        offset += len(dl_bin)

    with open(output_file, 'wb') as f:
        f.write(struct.pack('<III', DLMM_MAGIC, DLMM_VERSION, num))

        for i, sub in enumerate(submeshes):
            flags = 0
            if sub['has_texture']:
                flags |= 1

            name_bytes = sub['name'].encode('ascii', errors='replace')[:31]
            name_bytes = name_bytes + b'\x00' * (32 - len(name_bytes))

            f.write(struct.pack('<IIIII',
                dl_offsets[i],
                len(dl_binaries[i]),
                sub['diffuse_ambient'],
                sub['specular_emission'],
                sub['color']))
            f.write(struct.pack('<HH', sub['alpha'], flags))
            f.write(name_bytes)

        for dl_bin in dl_binaries:
            f.write(dl_bin)


def assert_num_args(cmd, real, expected, tokens):
    if real != expected:
        raise MD5FormatError(f"Unexpected nargs for '{cmd}' ({real} != {expected}): {tokens}")

class Quaternion():
    def __init__(self, w, x, y, z):
        self.w = w
        self.x = x
        self.y = y
        self.z = z

    def to_v3(self):
        return Vector(self.x, self.y, self.z)

    def complement(self):
        return Quaternion(self.w, -self.x, -self.y, -self.z)

    def normalize(self):
        mag = sqrt((self.w ** 2) + (self.x ** 2) + (self.y ** 2) + (self.z ** 2))
        return Quaternion(self.w / mag, self.x /mag, self.y / mag, self.z / mag)

    def mul(self, other):
        w = (self.w * other.w) - (self.x * other.x) - (self.y * other.y) - (self.z * other.z)
        x = (self.x * other.w) + (self.w * other.x) + (self.y * other.z) - (self.z * other.y)
        y = (self.y * other.w) + (self.w * other.y) + (self.z * other.x) - (self.x * other.z)
        z = (self.z * other.w) + (self.w * other.z) + (self.x * other.y) - (self.y * other.x)
        return Quaternion(w, x, y, z)

def quaternion_fill_incomplete_w(v):
    """
    This expands an incomplete quaternion, not a regular vector. This is
    needed if a quaternion is stored as the components x, y and z and it is
    expected that the code will fill the value of w.
    """
    t = 1.0 - (v[0] * v[0]) - (v[1] * v[1]) - (v[2] * v[2])
    if t < 0:
        w = 0
    else:
        w = -sqrt(t)
    return Quaternion(w, v[0], v[1], v[2])

class Vector():
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def to_q(self):
        return Quaternion(0, self.x, self.y, self.z)

    def length(self):
        return sqrt((self.x ** 2) + (self.y ** 2) + (self.z ** 2))

    def normalize(self):
        mag = self.length()
        return Vector(self.x / mag, self.y / mag, self.z / mag)

    def add(self, other):
        return Vector(self.x + other.x, self.y + other.y, self.z + other.z)

    def sub(self, other):
        return Vector(self.x - other.x, self.y - other.y, self.z - other.z)

    def cross(self, other):
        x = (self.y * other.z) - (other.y * self.z)
        y = (self.z * other.x) - (other.z * self.x)
        z = (self.x * other.y) - (other.x * self.y)
        return Vector(x, y, z)

    def mul_m4x3(self, m):
        x = (self.x * m[0][0]) + (self.y * m[0][1]) + (self.z * m[0][2]) + (m[0][3] * 1)
        y = (self.x * m[1][0]) + (self.y * m[1][1]) + (self.z * m[1][2]) + (m[1][3] * 1)
        z = (self.x * m[2][0]) + (self.y * m[2][1]) + (self.z * m[2][2]) + (m[2][3] * 1)
        return Vector(x, y, z)

def joint_info_to_m4x3(q, trans):
    """
    This generates a 4x3 matrix that represents a rotation and a translation.
    q is a Quaternion with a orientation, trans is a Vector with a translation.
    """
    wx = 2 * q.w * q.x
    wy = 2 * q.w * q.y
    wz = 2 * q.w * q.z
    x2 = 2 * q.x * q.x
    xy = 2 * q.x * q.y
    xz = 2 * q.x * q.z
    y2 = 2 * q.y * q.y
    yz = 2 * q.y * q.z
    z2 = 2 * q.z * q.z

    return [[1 - y2 - z2,     xy - wz,     xz + wy, trans.x],
            [    xy + wz, 1 - x2 - z2,     yz - wx, trans.y],
            [    xz - wy,     yz + wx, 1 - x2 - y2, trans.z]]

def invert_affine_m4x3(m):
    """Inverts an affine 4x3 matrix in row-major 3x4 form (m[row][col], columns
    0-2 = rotation/scale, column 3 = translation). Returns (inv3x3, trans3)."""
    r = [[m[0][0], m[0][1], m[0][2]],
         [m[1][0], m[1][1], m[1][2]],
         [m[2][0], m[2][1], m[2][2]]]
    t = [m[0][3], m[1][3], m[2][3]]

    det = (r[0][0] * (r[1][1] * r[2][2] - r[1][2] * r[2][1])
         - r[0][1] * (r[1][0] * r[2][2] - r[1][2] * r[2][0])
         + r[0][2] * (r[1][0] * r[2][1] - r[1][1] * r[2][0]))

    if abs(det) < 1e-12:
        raise MD5FormatError("Singular bind matrix; cannot invert for NSMW")

    inv_det = 1.0 / det

    inv = [
        [(r[1][1] * r[2][2] - r[1][2] * r[2][1]) * inv_det,
         (r[0][2] * r[2][1] - r[0][1] * r[2][2]) * inv_det,
         (r[0][1] * r[1][2] - r[0][2] * r[1][1]) * inv_det],
        [(r[1][2] * r[2][0] - r[1][0] * r[2][2]) * inv_det,
         (r[0][0] * r[2][2] - r[0][2] * r[2][0]) * inv_det,
         (r[0][2] * r[1][0] - r[0][0] * r[1][2]) * inv_det],
        [(r[1][0] * r[2][1] - r[1][1] * r[2][0]) * inv_det,
         (r[0][1] * r[2][0] - r[0][0] * r[2][1]) * inv_det,
         (r[0][0] * r[1][1] - r[0][1] * r[1][0]) * inv_det],
    ]

    tinv = [-(inv[0][0] * t[0] + inv[0][1] * t[1] + inv[0][2] * t[2]),
            -(inv[1][0] * t[0] + inv[1][1] * t[1] + inv[1][2] * t[2]),
            -(inv[2][0] * t[0] + inv[2][1] * t[1] + inv[2][2] * t[2])]

    return inv, tinv

def invbind_to_f32_list(m):
    """Inverts an affine 4x3 rest matrix and serializes it as 12 f32 values in
    the column-major-3x3 + translation order expected by MATRIX_MULT4x3."""
    inv, tinv = invert_affine_m4x3(m)
    vals = [inv[0][0], inv[1][0], inv[2][0],   # column 0
            inv[0][1], inv[1][1], inv[2][1],   # column 1
            inv[0][2], inv[1][2], inv[2][2],   # column 2
            tinv[0], tinv[1], tinv[2]]         # translation
    return [float_to_f32(v) for v in vals]

def parse_md5mesh(input_file, nsmw=False):
    # When nsmw is True, vertices may have up to two weights with arbitrary
    # biases (the NSMW converter selects/normalizes them later). Otherwise the
    # DSMA path requires exactly one weight with a bias of 1.0 per vertex.
    Joint = namedtuple("Joint", "name parent pos orient scale")
    Vert = namedtuple("Vert", "st startWeight countWeight")
    Weight = namedtuple("Weight", "joint bias pos")
    Mesh = namedtuple("Mesh", "shader numverts verts numtris tris numweights weights")

    joints = []
    meshes = []

    with open(input_file, 'r') as md5mesh_file:

        numJoints = None
        numMeshes = None

        # This can have three values:
        # - "root": Parsing commands in the md5mesh outside of any group
        # - "joints": Inside a "joints" node.
        # - "mesh": Inside a "mesh" node.
        mode = "root"

        # Temporary variables used to store mesh information before packing it
        shader = ""
        numverts = None
        verts = None
        numtris = None
        tris = None
        numweights = None
        weights = None

        for line in md5mesh_file:
            # Remove comments
            line = line.split('//')[0]

            # Parse line
            tokens = line.split()

            if len(tokens) == 0:  # Empty line
                continue

            cmd = tokens[0]
            tokens = tokens[1:]
            nargs = len(tokens)

            if mode == "root":
                if cmd == 'MD5Version':
                    assert_num_args('MD5Version', nargs, 1, tokens)
                    version = int(tokens[0])
                    if version != 10:
                        raise MD5FormatError(f"Invalid 'MD5Version': {version} != 10")

                elif cmd == 'commandline':
                    # Ignore this
                    pass

                elif cmd == 'numJoints':
                    assert_num_args('numJoints', nargs, 1, tokens)
                    numJoints = int(tokens[0])
                    if numJoints == 0:
                        raise MD5FormatError(f"'numJoints' is 0")

                elif cmd == 'numMeshes':
                    assert_num_args('numMeshes', nargs, 1, tokens)
                    numMeshes = int(tokens[0])
                    if numMeshes == 0:
                        raise MD5FormatError(f"'numMeshes' is 0")

                elif cmd == 'joints':
                    assert_num_args('joints', nargs, 1, tokens)
                    if tokens[0] != '{':
                        raise MD5FormatError(f"Unexpected token for 'joints': {tokens}")
                    if numJoints is None:
                        raise MD5FormatError("'joints' command before 'numJoints'")
                    mode = "joints"

                elif cmd == 'mesh':
                    assert_num_args('mesh', nargs, 1, tokens)
                    if tokens[0] != '{':
                        raise MD5FormatError(f"Unexpected token for 'mesh': {tokens}")
                    if numMeshes is None:
                        raise MD5FormatError("'mesh' command before 'numMeshes'")
                    mode = "mesh"

                else:
                    print(f"Ignored unsupported command: {cmd} {tokens}")

            elif mode == "joints":
                if cmd == '}':
                    if nargs > 0:
                        raise MD5FormatError(f"Unexpected tokens after 'joints {{}}': {tokens}")
                    mode = "root"
                else:
                    _, name, line = line.split('"')
                    tokens = line.strip().split(" ")
                    nargs = len(tokens)

                    assert_num_args('joint entry', nargs, 11, tokens)

                    parent = int(tokens[0])

                    if tokens[1] != '(':
                        raise MD5FormatError(f"Unexpected token 1 for joint': {tokens}")
                    pos = Vector(float(tokens[2]), float(tokens[3]), float(tokens[4]))
                    if tokens[5] != ')':
                        raise MD5FormatError(f"Unexpected token 5 for joint': {tokens}")

                    if tokens[6] != '(':
                        raise MD5FormatError(f"Unexpected token 6 for joint': {tokens}")
                    orient = (float(tokens[7]), float(tokens[8]), float(tokens[9]))
                    q_orient = quaternion_fill_incomplete_w(orient)
                    if tokens[10] != ')':
                        raise MD5FormatError(f"Unexpected token 10 for joint': {tokens}")

                    joints.append(Joint(name, parent, pos, q_orient, Vector(1.0, 1.0, 1.0)))

            elif mode == "mesh":
                if cmd == '}':
                    if nargs != 0:
                        raise MD5FormatError(f"Unexpected tokens after 'mesh {{}}': {tokens}")
                    mode = "root"

                    meshes.append(Mesh(shader, numverts, verts, numtris, tris, numweights, weights))

                    shader = ""
                    numverts = None
                    verts = None
                    numtris = None
                    tris = None
                    numweights = None
                    weights = None

                elif cmd == 'shader':
                    _, shader_path, _ = line.split('"')
                    shader = shader_path

                elif cmd == 'numverts':
                    assert_num_args('numverts', nargs, 1, tokens)
                    numverts = int(tokens[0])
                    verts = [None] * numverts

                elif cmd == 'vert':
                    assert_num_args('vert', nargs, 7, tokens)
                    if numverts is None:
                        raise MD5FormatError("'vert' command before 'numverts'")

                    index = int(tokens[0])

                    if tokens[1] != '(':
                        raise MD5FormatError(f"Unexpected token 1 for vert': {tokens}")
                    st = (float(tokens[2]), float(tokens[3]))
                    if tokens[4] != ')':
                        raise MD5FormatError(f"Unexpected token 4 for vert': {tokens}")

                    startWeight = int(tokens[5])
                    countWeight = int(tokens[6])

                    if not nsmw and countWeight != 1:
                        raise MD5FormatError(
                            f"Vertex with {countWeight} weights detected, but this tool "
                            "only supports vertices with one weight. Ensure that all your "
                            "vertices are assigned exactly one weight with a bias of 1.0. "
                            "Use --format nsmw to allow up to two weights per vertex."
                        )

                    verts[index] = Vert(st, startWeight, countWeight)

                elif cmd == 'numtris':
                    assert_num_args('numtris', nargs, 1, tokens)
                    numtris = int(tokens[0])
                    tris = [None] * numtris

                elif cmd == 'tri':
                    assert_num_args('tri', nargs, 4, tokens)
                    if numtris is None:
                        raise MD5FormatError("'tri' command before 'numtris'")

                    index = int(tokens[0])
                    # Reverse order so that they face the right direction
                    vertIndices = (int(tokens[3]), int(tokens[2]), int(tokens[1]))

                    tris[index] = vertIndices

                elif cmd == 'numweights':
                    assert_num_args('numweights', nargs, 1, tokens)
                    numweights = int(tokens[0])
                    weights = [None] * numweights

                elif cmd == 'weight':
                    assert_num_args('weight', nargs, 8, tokens)
                    if numverts is None:
                        raise MD5FormatError("'weight' command before 'numweights'")

                    index = int(tokens[0])
                    jointIndex = int(tokens[1])
                    bias = float(tokens[2])

                    if not nsmw and bias != 1.0:
                        raise MD5FormatError(
                            f"Weight with bias {bias} detected, but this tool only"
                            "supports weights with bias equal to 1.0. Ensure that all"
                            "your vertices are assigned exactly one weight with a"
                            "bias of 1.0. Use --format nsmw to allow blended weights."
                        )

                    if tokens[3] != '(':
                        raise MD5FormatError(f"Unexpected token 3 for weight': {tokens}")
                    pos = Vector(float(tokens[4]), float(tokens[5]), float(tokens[6]))
                    if tokens[7] != ')':
                        raise MD5FormatError(f"Unexpected token 7 for weight': {tokens}")

                    weights[index] = Weight(jointIndex, bias, pos)

                else:
                    print(f"Ignored unsupported command: {cmd} {tokens}")

        if mode != "root":
            raise MD5FormatError("Unexpected end of file (expected '}')")

    realJoints = len(joints)
    if numJoints != realJoints:
        raise MD5FormatError(f"Incorrect number of joints: {numJoints} != {realJoints}")

    realMeshes = len(meshes)
    if numJoints != realJoints:
        raise MD5FormatError(f"Incorrect number of joints: {numJoints} != {realJoints}")

    return (joints, meshes)

def parse_md5anim(input_file, old_md5=False):
    Joint = namedtuple("Joint", "name parent pos orient scale")

    joints = []
    frames = []

    with open(input_file, 'r') as md5anim_file:

        numFrames = None
        numJoints = None

        baseframe = []
        hierarchy = []

        # This can have three values:
        # - "root": Parsing commands in the md5mesh outside of any group
        # - "hierarchy": Inside a "hierarchy" node.
        # - "bounds": Inside a "bounds" node.
        # - "baseframe": Inside a "baseframe" node.
        # - "frame": Inside a "frame" node.
        mode = "root"

        frame_index = None

        for line in md5anim_file:
            # Remove comments
            line = line.split('//')[0]

            # Parse line
            tokens = line.split()

            if len(tokens) == 0:  # Empty line
                continue

            cmd = tokens[0]
            tokens = tokens[1:]
            nargs = len(tokens)

            if mode == "root":
                if cmd == 'MD5Version':
                    assert_num_args('MD5Version', nargs, 1, tokens)
                    version = int(tokens[0])
                    if version != 10:
                        raise MD5FormatError(f"Invalid 'MD5Version': {version} != 10")

                elif cmd == 'commandline':
                    # Ignore this
                    pass

                elif cmd == 'numFrames':
                    assert_num_args('numFrames', nargs, 1, tokens)
                    numFrames = int(tokens[0])
                    if numFrames == 0:
                        raise MD5FormatError(f"'numFrames' is 0")
                    frames = [None] * numFrames

                elif cmd == 'numJoints':
                    assert_num_args('numJoints', nargs, 1, tokens)
                    numJoints = int(tokens[0])
                    if numJoints == 0:
                        raise MD5FormatError(f"'numJoints' is 0")

                elif cmd == 'frameRate':
                    # Ignore this
                    pass

                elif cmd == 'numAnimatedComponents':
                    # Ignore this
                    pass

                elif cmd == 'hierarchy':
                    assert_num_args('hierarchy', nargs, 1, tokens)
                    if tokens[0] != '{':
                        raise MD5FormatError(f"Unexpected token for 'hierarchy': {tokens}")
                    mode = "hierarchy"

                elif cmd == 'bounds':
                    assert_num_args('bounds', nargs, 1, tokens)
                    if tokens[0] != '{':
                        raise MD5FormatError(f"Unexpected token for 'bounds': {tokens}")
                    mode = "bounds"

                elif cmd == 'baseframe':
                    assert_num_args('baseframe', nargs, 1, tokens)
                    if tokens[0] != '{':
                        raise MD5FormatError(f"Unexpected token for 'baseframe': {tokens}")
                    mode = "baseframe"

                elif cmd == 'frame':
                    assert_num_args('frame', nargs, 2, tokens)
                    frame_index = int(tokens[0])

                    if tokens[1] != '{':
                        raise MD5FormatError(f"Unexpected token for 'frame': {tokens}")
                    if numFrames is None:
                        raise MD5FormatError("'frame' command before 'numFrames'")
                    mode = "frame"
                    joints = []

                else:
                    print(f"Ignored unsupported command: {cmd} {tokens}")

            elif mode == "hierarchy":
                if cmd == '}':
                    if nargs > 0:
                        raise MD5FormatError(f"Unexpected tokens after 'hierarchy {{}}': {tokens}")
                    mode = "root"
                else:
                    _, name, line = line.split('"')
                    tokens = line.strip().split(" ")
                    nargs = len(tokens)

                    assert_num_args('hierarchy entry', nargs, 3, tokens)

                    parent_index = int(tokens[0])
                    flags = int(tokens[1])
                    if flags != 63 and flags != 511:
                        raise MD5FormatError(f"Unexpected flags in hierarchy: {flags}")
                    frame_data_index = int(tokens[2])

                    hierarchy.append(parent_index)

            elif mode == "bounds":
                if cmd == '}':
                    if nargs > 0:
                        raise MD5FormatError(f"Unexpected tokens after 'bounds {{}}': {tokens}")
                    mode = "root"
                else:
                    # Ignore everything else
                    pass

            elif mode == "baseframe":
                if cmd == '}':
                    if nargs > 0:
                        raise MD5FormatError(f"Unexpected tokens after 'baseframe {{}}': {tokens}")
                    mode = "root"
                else:
                    values = line.strip().split()

                    # Auto-detect old (10 tokens) vs new (15 tokens) format
                    has_scale = len(values) == 15

                    if has_scale:
                        assert_num_args('baseframe joint', len(values), 15, values)
                    else:
                        assert_num_args('baseframe joint', len(values), 10, values)

                    if values[0] != '(':
                        raise MD5FormatError(f"Unexpected token 0 for baseframe': {values}")
                    pos = Vector(float(values[1]), float(values[2]), float(values[3]))
                    if values[4] != ')':
                        raise MD5FormatError(f"Unexpected token 4 for baseframe': {values}")

                    if values[5] != '(':
                        raise MD5FormatError(f"Unexpected token 5 for baseframe': {values}")
                    orient = (float(values[6]), float(values[7]), float(values[8]))
                    q_orient = quaternion_fill_incomplete_w(orient)
                    if values[9] != ')':
                        raise MD5FormatError(f"Unexpected token 9 for baseframe': {values}")

                    if has_scale:
                        if values[10] != '(':
                            raise MD5FormatError(f"Unexpected token 10 for baseframe': {values}")
                        scale = Vector(float(values[11]), float(values[12]), float(values[13]))
                        if values[14] != ')':
                            raise MD5FormatError(f"Unexpected token 14 for baseframe': {values}")
                    else:
                        scale = Vector(1.0, 1.0, 1.0)

                    baseframe.append(Joint("", -1, pos, q_orient, scale))

            elif mode == "frame":
                if cmd == '}':
                    if nargs > 0:
                        raise MD5FormatError(f"Unexpected tokens after 'frame {{}}': {tokens}")
                    mode = "root"

                    # Now that the frame has been read, process the real
                    # positions and orientations of the bones before storing
                    # them.

                    transformed_joints = []

                    for joint, parent_index in zip(joints, hierarchy):
                        if parent_index == -1:
                            # Root bone
                            transformed_joints.append(joint)
                        else:
                            parent_pos = transformed_joints[parent_index].pos
                            parent_orient = transformed_joints[parent_index].orient
                            parent_scale = transformed_joints[parent_index].scale

                            this_pos = joint.pos
                            this_orient = joint.orient
                            this_scale = joint.scale

                            # Apply parent scale to child position
                            scaled_pos = Vector(
                                this_pos.x * parent_scale.x,
                                this_pos.y * parent_scale.y,
                                this_pos.z * parent_scale.z)

                            q = parent_orient
                            qt = q.complement()
                            q_pos_delta = q.mul(scaled_pos.to_q()).mul(qt)
                            pos_delta = q_pos_delta.to_v3()

                            pos = parent_pos.add(pos_delta)
                            orient = parent_orient.mul(this_orient).normalize()

                            # Combine scales
                            scale = Vector(
                                parent_scale.x * this_scale.x,
                                parent_scale.y * this_scale.y,
                                parent_scale.z * this_scale.z)

                            transformed_joints.append(Joint("", -1, pos, orient, scale))

                    frames[frame_index] = transformed_joints
                else:
                    values = line.strip().split()

                    # Auto-detect old (6 values) vs new (9 values) format
                    has_scale = len(values) == 9

                    if has_scale:
                        assert_num_args('frame joint', len(values), 9, values)
                    else:
                        assert_num_args('frame joint', len(values), 6, values)

                    pos = Vector(float(values[0]), float(values[1]), float(values[2]))

                    orient = (float(values[3]), float(values[4]), float(values[5]))
                    q_orient = quaternion_fill_incomplete_w(orient)

                    if has_scale:
                        scale = Vector(float(values[6]), float(values[7]), float(values[8]))
                    else:
                        scale = Vector(1.0, 1.0, 1.0)

                    joints.append(Joint("", -1, pos, q_orient, scale))

        if mode != "root":
            raise MD5FormatError("Unexpected end of file (expected '}')")

    realJoints = len(joints)
    if numJoints != realJoints:
        raise MD5FormatError(f"Incorrect number of joints: {numJoints} != {realJoints}")

    realFrames = len(frames)
    if numFrames != realFrames:
        raise MD5FormatError(f"Incorrect number of frames: {numFrames} != {realFrames}")

    return frames

def save_animation(frames, output_file, blender_fix):

    num_frames = len(frames)
    num_bones = len(frames[0])

    # Auto-detect whether any joint uses non-unit scale
    needs_scale = False
    for joints in frames:
        for joint in joints:
            s = joint.scale
            if abs(s.x - 1.0) > 1e-6 or abs(s.y - 1.0) > 1e-6 or abs(s.z - 1.0) > 1e-6:
                needs_scale = True
                break
        if needs_scale:
            break

    version = 2 if needs_scale else 1
    u32_array = [version, num_frames, num_bones]

    for joints in frames:
        if num_bones != len(joints):
            raise MD5FormatError("Different number of bones across frames")

        for joint in joints:
            this_pos = joint.pos
            this_orient = joint.orient
            this_scale = joint.scale

            if blender_fix:
                # It is needed to rotate all bones because all bones have
                # absolute transformations. Rotate orientation and position by
                # -90 degrees on the X axis.
                q_rot = Quaternion(0.7071068, -0.7071068, 0, 0)
                this_orient = q_rot.mul(this_orient)
                this_pos = Vector(this_pos.x, this_pos.z, -this_pos.y)
                this_scale = Vector(this_scale.x, this_scale.z, this_scale.y)

            pos = [float_to_f32(this_pos.x), float_to_f32(this_pos.y),
                   float_to_f32(this_pos.z)]
            orient = [float_to_f32(this_orient.w), float_to_f32(this_orient.x),
                      float_to_f32(this_orient.y), float_to_f32(this_orient.z)]

            u32_array.extend(pos)
            u32_array.extend(orient)

            if needs_scale:
                scale = [float_to_f32(this_scale.x), float_to_f32(this_scale.y),
                         float_to_f32(this_scale.z)]
                u32_array.extend(scale)

    with open(output_file, "wb") as f:
        for u32 in u32_array:
            b = [u32 & 0xFF, \
                (u32 >> 8) & 0xFF, \
                (u32 >> 16) & 0xFF, \
                (u32 >> 24) & 0xFF]
            f.write(bytearray(b))

# ---------------------------------------------------------------------------
# Triangle strip helpers
# ---------------------------------------------------------------------------

def _tri_directed_edge_third(tri, p, q):
    """If triangle (a,b,c) CCW has directed edge p->q, return third vertex."""
    a, b, c = tri
    if a == p and b == q: return c
    if b == p and c == q: return a
    if c == p and a == q: return b
    return None

def stripify_triangles(resolved_tris):
    """Greedy triangle stripification.
    resolved_tris: list of (vk0, vk1, vk2) in CCW order.
    Returns (strips, singles) where:
      strips = list of (strip_vertex_keys, strip_face_indices)
      singles = list of face indices not in any strip
    """
    if not resolved_tris:
        return [], []

    edge_to_faces = defaultdict(list)
    for fi, tri in enumerate(resolved_tris):
        for i in range(3):
            edge = frozenset([tri[i], tri[(i + 1) % 3]])
            edge_to_faces[edge].append(fi)

    used = set()
    strips = []
    singles = []

    for start_fi in range(len(resolved_tris)):
        if start_fi in used:
            continue

        best_strip = None
        best_faces = None

        face = resolved_tris[start_fi]
        for rot in range(3):
            a = face[rot]
            b = face[(rot + 1) % 3]
            c = face[(rot + 2) % 3]

            strip = [a, b, c]
            faces = [start_fi]
            local_used = set(used)
            local_used.add(start_fi)

            while True:
                n = len(faces)
                p, q = strip[-2], strip[-1]
                edge_key = frozenset([p, q])

                found = False
                for cfi in edge_to_faces[edge_key]:
                    if cfi in local_used:
                        continue
                    tri = resolved_tris[cfi]
                    if n % 2 == 0:
                        new_v = _tri_directed_edge_third(tri, p, q)
                    else:
                        new_v = _tri_directed_edge_third(tri, q, p)

                    if new_v is not None:
                        strip.append(new_v)
                        faces.append(cfi)
                        local_used.add(cfi)
                        found = True
                        break

                if not found:
                    break

            if best_faces is None or len(faces) > len(best_faces):
                best_strip = strip
                best_faces = faces

        for fi in best_faces:
            used.add(fi)

        if len(best_faces) >= 2:
            strips.append((best_strip, best_faces))
        else:
            singles.append(start_fi)

    return strips, singles

# ---------------------------------------------------------------------------

def convert_md5mesh(model_file, name, output_folder, texture_size,
                    draw_normal_polygons, extension_mesh, extension_anim,
                    blender_fix, export_base_pose, no_strip=False,
                    multi_material=False):

    print(f"Converting model: {model_file}")

    # Parse md5mesh file
    joints, meshes = parse_md5mesh(model_file)

    print(f"Loaded {len(joints)} joint(s) and {len(meshes)} mesh(es).")

    if not multi_material and len(meshes) > 1:
        print("WARNING: More than one mesh found. All meshes will share the same "
              "texture. If you want them to have different textures, use "
              "--multi-material to output a DLMM file with per-mesh materials.")

    if export_base_pose:
        print("Converting base pose...")

        save_animation([joints],
                       os.path.join(output_folder, f"{name}{extension_anim}"),
                       blender_fix)

    print("Converting meshes...")

    model_dir = os.path.dirname(os.path.abspath(model_file))
    base_matrix = 30 - len(joints) + 1

    # In multi-material mode, each mesh gets its own DisplayList
    # In single mode, all meshes share one DisplayList
    if not multi_material:
        dl = DisplayList()

    last_joint_index = None
    dlmm_submeshes = []  # used only in multi-material mode

    for mesh_index, mesh in enumerate(meshes):
        if multi_material:
            dl = DisplayList()
            last_joint_index = None

            # Determine texture size from shader image
            mesh_tex_size = list(texture_size) if texture_size else [64, 64]
            if mesh.shader:
                shader_path = os.path.join(model_dir, mesh.shader)
                if os.path.isfile(shader_path):
                    try:
                        w, h = get_image_dimensions(shader_path)
                        if is_valid_texture_size(w) and is_valid_texture_size(h):
                            mesh_tex_size = [w, h]
                            print(f"  Shader '{mesh.shader}': detected {w}x{h}")
                        else:
                            print(f"  WARNING: Shader image {mesh.shader} has non-power-of-2 "
                                  f"size {w}x{h}, using fallback {mesh_tex_size}")
                    except ValueError as e:
                        print(f"  WARNING: Cannot read shader image: {e}")
                else:
                    print(f"  WARNING: Shader image not found: {shader_path}")
        else:
            mesh_tex_size = texture_size
        print(f"  Vertices: {mesh.numverts}")
        print(f"  Tris:     {mesh.numtris}")
        print(f"  Weights:  {mesh.numweights}")

        print("  Generating per-triangle normals...")

        tri_normal = []
        for tri in mesh.tris:
            verts = [mesh.verts[i] for i in tri]
            weights = [mesh.weights[v.startWeight] for v in verts]

            vtx = []
            for vert, weight in zip(verts, weights):
                joint = joints[weight.joint]
                m = joint_info_to_m4x3(joint.orient, joint.pos)
                final = weight.pos.mul_m4x3(m)
                vtx.append(final)

            a = vtx[0].sub(vtx[1])
            b = vtx[1].sub(vtx[2])

            n = a.cross(b)

            if n.length() > 0:
                n = n.normalize()
                tri_normal.append(n)
            else:
                tri_normal.append(Vector(0, 0, 0))

        # Pre-compute per-vertex data for all triangles
        # Each entry: (texcoord, joint_index, normal_joint_space, pos, final_or_None)
        all_tri_verts = []  # list of list of per-vertex tuples (one list per tri)

        for tri, norm in zip(mesh.tris, tri_normal):
            verts = [mesh.verts[i] for i in tri]
            weights = [mesh.weights[v.startWeight] for v in verts]

            tri_vdata = []
            for vert, weight in zip(verts, weights):
                st = vert.st
                u = st[0] * mesh_tex_size[0]
                v = st[1] * mesh_tex_size[1]

                joint_index = weight.joint
                joint = joints[joint_index]

                q = joint.orient
                qt = q.complement()
                n = qt.mul(norm.to_q()).mul(q).to_v3()
                if n.length() > 0:
                    n = n.normalize()

                final = None
                if draw_normal_polygons:
                    q2 = joint.orient
                    qt2 = q2.complement()
                    vq = weight.pos.to_q()
                    delta = q2.mul(vq).mul(qt2).to_v3()
                    final = joint.pos.add(delta)

                tri_vdata.append({
                    'u': u, 'v': v,
                    'joint_index': joint_index,
                    'nx': n.x, 'ny': n.y, 'nz': n.z,
                    'px': weight.pos.x, 'py': weight.pos.y, 'pz': weight.pos.z,
                    'final': final,
                })
            all_tri_verts.append(tri_vdata)

        # Build vertex keys for stripification.
        # A vertex key = (mesh_vert_index, quantized_normal) so that
        # only coplanar triangles at the same vertex can share strip edges.
        resolved_tris = []
        for ti, tri in enumerate(mesh.tris):
            vdata = all_tri_verts[ti]
            vkeys = []
            for vi in range(3):
                d = vdata[vi]
                vk = (tri[vi],
                      float_to_n10(d['nx']),
                      float_to_n10(d['ny']),
                      float_to_n10(d['nz']))
                vkeys.append(vk)
            resolved_tris.append(tuple(vkeys))

        # Stripify (skip when drawing debug normals or when disabled)
        if draw_normal_polygons or no_strip:
            tri_strips, tri_singles = [], list(range(len(resolved_tris)))
        else:
            tri_strips, tri_singles = stripify_triangles(resolved_tris)

        # Statistics
        separate_vtx = len(resolved_tris) * 3
        strip_vtx = (sum(len(s) for s, _ in tri_strips)
                     + len(tri_singles) * 3)
        stripped_count = len(resolved_tris) - len(tri_singles)
        print(f"  Triangle strips: {len(tri_strips)} ({stripped_count} faces stripped, "
              f"{len(tri_singles)} separate)")
        print(f"  GPU vertices:    {separate_vtx} -> {strip_vtx} "
              f"(saved {separate_vtx - strip_vtx})")

        # Helper: build lookup from vertex key -> (tri_index, vert_index_in_tri)
        vk_to_src = {}
        for ti, tri in enumerate(mesh.tris):
            vdata = all_tri_verts[ti]
            for vi in range(3):
                d = vdata[vi]
                vk = (tri[vi],
                      float_to_n10(d['nx']),
                      float_to_n10(d['ny']),
                      float_to_n10(d['nz']))
                vk_to_src[vk] = (ti, vi)

        def emit_md5_vertex(vk):
            nonlocal last_joint_index
            ti, vi = vk_to_src[vk]
            d = all_tri_verts[ti][vi]

            dl.texcoord(d['u'], d['v'])

            joint_index = d['joint_index']
            if draw_normal_polygons or joint_index != last_joint_index:
                dl.mtx_restore(base_matrix + joint_index)
                last_joint_index = joint_index

            dl.normal(d['nx'], d['ny'], d['nz'])
            dl.vtx(d['px'], d['py'], d['pz'])

        print("  Generating display list...")

        # Emit triangle strips
        for strip_verts, strip_faces in tri_strips:
            dl.begin_vtxs("triangle_strip")
            for vk in strip_verts:
                emit_md5_vertex(vk)
            dl.end_vtxs()

        # Emit separate triangles
        if tri_singles:
            dl.begin_vtxs("triangles")
            for fi in tri_singles:
                for vk in resolved_tris[fi]:
                    emit_md5_vertex(vk)

                if draw_normal_polygons:
                    vdata = all_tri_verts[fi]
                    norm = tri_normal[fi]
                    finals = [vdata[vi]['final'] for vi in range(3)]

                    dl.mtx_restore(1)
                    last_joint_index = None

                    vert_avg = Vector(
                        (finals[0].x + finals[1].x + finals[2].x) / 3,
                        (finals[0].y + finals[1].y + finals[2].y) / 3,
                        (finals[0].z + finals[1].z + finals[2].z) / 3
                    )

                    vert_avg_end = vert_avg.add(norm)

                    dl.texcoord(0, 0)

                    dl.color(1, 0, 0)
                    dl.vtx(vert_avg.x + 0.1, vert_avg.y, vert_avg.z)
                    dl.vtx(vert_avg.x, vert_avg.y, vert_avg.z)
                    dl.color(0, 1, 0)
                    dl.vtx(vert_avg_end.x, vert_avg_end.y, vert_avg_end.z)

                    dl.color(1, 0, 0)
                    dl.vtx(vert_avg.x, vert_avg.y, vert_avg.z)
                    dl.vtx(vert_avg.x, vert_avg.y + 0.1, vert_avg.z)
                    dl.color(0, 1, 0)
                    dl.vtx(vert_avg_end.x, vert_avg_end.y, vert_avg_end.z)

                    dl.color(1, 0, 0)
                    dl.vtx(vert_avg.x, vert_avg.y, vert_avg.z)
                    dl.vtx(vert_avg.x, vert_avg.y, vert_avg.z + 0.1)
                    dl.color(0, 1, 0)
                    dl.vtx(vert_avg_end.x, vert_avg_end.y, vert_avg_end.z)

            dl.end_vtxs()

        if multi_material:
            dl.finalize()
            # Derive material name from shader path (basename without extension)
            if mesh.shader:
                mat_name = os.path.splitext(os.path.basename(mesh.shader))[0]
            else:
                mat_name = f"mesh_{mesh_index}"

            dlmm_submeshes.append({
                'name': mat_name,
                'dl': dl,
                'diffuse_ambient': 0,
                'specular_emission': 0,
                'color': 0x7FFF,
                'alpha': 31,
                'has_texture': True,
            })
            print(f"  Material name: '{mat_name}'")

    if multi_material:
        output_path = os.path.join(output_folder, f"{name}{extension_mesh}")
        save_dlmm(output_path, dlmm_submeshes)
        print(f"Saved DLMM with {len(dlmm_submeshes)} submesh(es) to {output_path}")
    else:
        dl.finalize()
        dl.save_to_file(os.path.join(output_folder, f"{name}{extension_mesh}"))


# ---------------------------------------------------------------------------
# NSMW (two-weight smooth skinning) support
# ---------------------------------------------------------------------------

def save_nsmw(output_file, num_joints, node_list, invbind_list, submeshes):
    """Write an NSMW model to binary.

    Args:
        output_file: output path
        num_joints: number of joints in the skeleton (must match the DSA)
        node_list: list of (num_weights, joint0, joint1, weight0_f32, weight1_f32)
        invbind_list: list (length num_joints) of 12-int32 inverse-bind matrices
        submeshes: list of dicts (same fields as save_dlmm submeshes)
    """
    num_nodes = len(node_list)
    num_sub = len(submeshes)

    dl_binaries = [sub['dl'].get_binary() for sub in submeshes]

    dl_start = (NSMW_HEADER_SIZE
                + num_nodes * NSMW_NODE_SIZE
                + num_joints * NSMW_INVBIND_SIZE
                + num_sub * DLMM_SUBMESH_HEADER_SIZE)

    dl_offsets = []
    offset = dl_start
    for b in dl_binaries:
        dl_offsets.append(offset)
        offset += len(b)

    with open(output_file, 'wb') as f:
        f.write(struct.pack('<IIIIII', NSMW_MAGIC, NSMW_VERSION, num_nodes,
                            num_joints, num_sub, 0))

        # Node table
        for (nw, j0, j1, w0, w1) in node_list:
            f.write(struct.pack('<BBBBII', nw, j0, j1, 0, w0, w1))

        # Inverse-bind table
        for inv in invbind_list:
            for v in inv:
                f.write(struct.pack('<I', v))

        # Submesh table (same layout as DLMM)
        for i, sub in enumerate(submeshes):
            flags = 1 if sub['has_texture'] else 0
            name_bytes = sub['name'].encode('ascii', errors='replace')[:31]
            name_bytes = name_bytes + b'\x00' * (32 - len(name_bytes))
            f.write(struct.pack('<IIIII',
                dl_offsets[i], len(dl_binaries[i]),
                sub['diffuse_ambient'], sub['specular_emission'], sub['color']))
            f.write(struct.pack('<HH', sub['alpha'], flags))
            f.write(name_bytes)

        # Display lists
        for b in dl_binaries:
            f.write(b)


def convert_md5mesh_nsmw(model_file, name, output_folder, texture_size,
                         extension_mesh, extension_anim, blender_fix,
                         export_base_pose, no_strip=False):

    print(f"Converting model (NSMW): {model_file}")

    joints, meshes = parse_md5mesh(model_file, nsmw=True)

    print(f"Loaded {len(joints)} joint(s) and {len(meshes)} mesh(es).")

    if export_base_pose:
        print("Converting base pose...")
        save_animation([joints],
                       os.path.join(output_folder, f"{name}{extension_anim}"),
                       blender_fix)

    print("Converting meshes...")

    model_dir = os.path.dirname(os.path.abspath(model_file))

    # Rest (bind) matrix of every joint in original (un-fixed) model space. The
    # blender-fix rotation is applied only to the animation (DSA), composed on
    # the left of the animated joint matrices, so it must NOT be applied here.
    joint_rest = [joint_info_to_m4x3(j.orient, j.pos) for j in joints]

    # Global node table shared by all submeshes.
    node_map = {}      # node key -> node index
    node_list = []     # list of (num_weights, j0, j1, w0_f32, w1_f32)
    over_weight_warned = [False]

    def resolve_node(mesh, vert):
        # Returns (node_index, selected) where 'selected' is a list of
        # (joint, weight_float, weight_pos) for the chosen (1 or 2) weights.
        ws = [mesh.weights[vert.startWeight + k] for k in range(vert.countWeight)]
        if len(ws) == 0:
            raise MD5FormatError("Vertex with no weights")

        if len(ws) > 2 and not over_weight_warned[0]:
            print("  WARNING: vertices with more than 2 weights found; keeping "
                  "the two largest and renormalizing")
            over_weight_warned[0] = True

        ws = sorted(ws, key=lambda w: -w.bias)[:2]

        total = sum(w.bias for w in ws)
        if total <= 0.0:
            total = 1.0

        sel = [(w.joint, w.bias / total, w.pos) for w in ws]

        # Merge two weights that target the same joint
        if len(sel) == 2 and sel[0][0] == sel[1][0]:
            sel = [(sel[0][0], 1.0, sel[0][2])]

        if len(sel) == 1:
            j = sel[0][0]
            sel = [(j, 1.0, sel[0][2])]
            key = (1, j, j, float_to_f32(1.0), 0)
        else:
            (j0, w0, p0), (j1, w1, p1) = sel
            # Canonicalize so the smaller joint index is first (stable dedup)
            if j1 < j0:
                (j0, w0, p0), (j1, w1, p1) = (j1, w1, p1), (j0, w0, p0)
            sel = [(j0, w0, p0), (j1, w1, p1)]
            key = (2, j0, j1, float_to_f32(w0), float_to_f32(w1))

        idx = node_map.get(key)
        if idx is None:
            idx = len(node_list)
            node_map[key] = idx
            node_list.append(key)
        return idx, sel

    def compute_vbind(sel):
        vx = vy = vz = 0.0
        for (j, w, pos) in sel:
            p = pos.mul_m4x3(joint_rest[j])
            vx += w * p.x
            vy += w * p.y
            vz += w * p.z
        return Vector(vx, vy, vz)

    mesh_data = []  # per-mesh data needed to emit display lists later

    for mesh_index, mesh in enumerate(meshes):
        # Texture size (auto-detected from the shader image, like multi-material)
        mesh_tex_size = list(texture_size) if texture_size else [64, 64]
        if mesh.shader:
            shader_path = os.path.join(model_dir, mesh.shader)
            if os.path.isfile(shader_path):
                try:
                    w, h = get_image_dimensions(shader_path)
                    if is_valid_texture_size(w) and is_valid_texture_size(h):
                        mesh_tex_size = [w, h]
                        print(f"  Shader '{mesh.shader}': detected {w}x{h}")
                    else:
                        print(f"  WARNING: Shader image {mesh.shader} has non-power-of-2 "
                              f"size {w}x{h}, using fallback {mesh_tex_size}")
                except ValueError as e:
                    print(f"  WARNING: Cannot read shader image: {e}")
            else:
                print(f"  WARNING: Shader image not found: {shader_path}")

        print(f"  Mesh {mesh_index}: {mesh.numverts} verts, {mesh.numtris} tris, "
              f"{mesh.numweights} weights")

        # Per md5 vertex: node index and model-space bind position
        vert_node = [None] * mesh.numverts
        vert_vbind = [None] * mesh.numverts
        for vi, vert in enumerate(mesh.verts):
            node_idx, sel = resolve_node(mesh, vert)
            vert_node[vi] = node_idx
            vert_vbind[vi] = compute_vbind(sel)

        # Per-triangle model-space normals (from the bind positions)
        tri_normal = []
        for tri in mesh.tris:
            p = [vert_vbind[i] for i in tri]
            n = p[0].sub(p[1]).cross(p[1].sub(p[2]))
            tri_normal.append(n.normalize() if n.length() > 0 else Vector(0, 0, 0))

        # Per-(triangle, vertex) GPU data
        all_tri_verts = []
        for ti, tri in enumerate(mesh.tris):
            norm = tri_normal[ti]
            tri_vdata = []
            for vi in range(3):
                mv = tri[vi]
                st = mesh.verts[mv].st
                vb = vert_vbind[mv]
                tri_vdata.append({
                    'u': st[0] * mesh_tex_size[0],
                    'v': st[1] * mesh_tex_size[1],
                    'node_index': vert_node[mv],
                    'nx': norm.x, 'ny': norm.y, 'nz': norm.z,
                    'px': vb.x, 'py': vb.y, 'pz': vb.z,
                })
            all_tri_verts.append(tri_vdata)

        # Build keys for stripification (vertex index + quantized normal) and a
        # reverse lookup from key to source (triangle, vertex).
        resolved_tris = []
        vk_to_src = {}
        for ti, tri in enumerate(mesh.tris):
            vdata = all_tri_verts[ti]
            vkeys = []
            for vi in range(3):
                d = vdata[vi]
                vk = (tri[vi], float_to_n10(d['nx']),
                      float_to_n10(d['ny']), float_to_n10(d['nz']))
                vkeys.append(vk)
                vk_to_src[vk] = (ti, vi)
            resolved_tris.append(tuple(vkeys))

        if no_strip:
            tri_strips, tri_singles = [], list(range(len(resolved_tris)))
        else:
            tri_strips, tri_singles = stripify_triangles(resolved_tris)

        if mesh.shader:
            mat_name = os.path.splitext(os.path.basename(mesh.shader))[0]
        else:
            mat_name = f"mesh_{mesh_index}"

        mesh_data.append({
            'tri_strips': tri_strips,
            'tri_singles': tri_singles,
            'resolved_tris': resolved_tris,
            'all_tri_verts': all_tri_verts,
            'vk_to_src': vk_to_src,
            'mat_name': mat_name,
        })

    num_nodes = len(node_list)
    print(f"  Nodes (matrix-palette slots): {num_nodes}")
    if num_nodes == 0:
        raise MD5FormatError("NSMW model has no nodes")
    if num_nodes > NSMW_MAX_NODES:
        raise MD5FormatError(
            f"NSMW model needs {num_nodes} nodes but the matrix stack only has "
            f"room for {NSMW_MAX_NODES}. Reduce the number of distinct two-bone "
            "weight combinations (e.g. split the mesh or simplify the rig).")

    base_matrix = 30 - num_nodes + 1

    # Inverse-bind matrices for every joint (serialized in MATRIX_MULT4x3 order)
    invbind_list = [invbind_to_f32_list(joint_rest[j]) for j in range(len(joints))]

    # Emit one display list per submesh now that the node base index is known
    submeshes = []
    for md in mesh_data:
        dl = DisplayList()
        all_tri_verts = md['all_tri_verts']
        vk_to_src = md['vk_to_src']
        resolved_tris = md['resolved_tris']
        last_node_index = [None]

        def emit_vertex(vk):
            ti, vi = vk_to_src[vk]
            d = all_tri_verts[ti][vi]
            dl.texcoord(d['u'], d['v'])
            node_index = d['node_index']
            if node_index != last_node_index[0]:
                dl.mtx_restore(base_matrix + node_index)
                last_node_index[0] = node_index
            dl.normal(d['nx'], d['ny'], d['nz'])
            dl.vtx(d['px'], d['py'], d['pz'])

        for strip_verts, strip_faces in md['tri_strips']:
            dl.begin_vtxs("triangle_strip")
            for vk in strip_verts:
                emit_vertex(vk)
            dl.end_vtxs()

        if md['tri_singles']:
            dl.begin_vtxs("triangles")
            for fi in md['tri_singles']:
                for vk in resolved_tris[fi]:
                    emit_vertex(vk)
            dl.end_vtxs()

        dl.finalize()
        submeshes.append({
            'name': md['mat_name'],
            'dl': dl,
            'diffuse_ambient': 0,
            'specular_emission': 0,
            'color': 0x7FFF,
            'alpha': 31,
            'has_texture': True,
        })

    output_path = os.path.join(output_folder, f"{name}{extension_mesh}")
    save_nsmw(output_path, len(joints), node_list, invbind_list, submeshes)
    print(f"Saved NSMW with {num_nodes} node(s) and {len(submeshes)} submesh(es) "
          f"to {output_path}")


# ---------------------------------------------------------------------------
# Bone collision (.md5collimesh / .boncol) support
# ---------------------------------------------------------------------------

BNCL_MAGIC = 0x4C434E42   # "BNCL" little-endian
BNCL_VERSION = 1

# Type codes matching NEA_BONCOL_TYPE_* in NEABoneCollision.h
BONCOL_TYPE_NONE    = 0
BONCOL_TYPE_SPHERE  = 1
BONCOL_TYPE_CAPSULE = 2
BONCOL_TYPE_AABB    = 3

def parse_md5collimesh(path):
    """Parse a .md5collimesh text file.

    Returns a list of dicts, one per bone:
        {'name': str, 'type': str, 'radius': float, 'half_height': float,
         'half_extents': (float,float,float), 'offset': (float,float,float)}
    """
    bones = []

    with open(path, 'r') as f:
        mode = "root"
        current = None

        for line in f:
            line = line.split('//')[0].strip()
            if not line:
                continue

            tokens = line.split()
            cmd = tokens[0]

            if mode == "root":
                if cmd == "MD5CollisionVersion":
                    version = int(tokens[1])
                    if version != 1:
                        raise MD5FormatError(
                            f"Unsupported MD5CollisionVersion: {version}")

                elif cmd == "numBones":
                    # informational only, we count from parsed data
                    pass

                elif cmd == "bone":
                    # bone "name" {
                    _, name, rest = line.split('"')
                    if '{' not in rest:
                        raise MD5FormatError(
                            f"Expected '{{' after bone name: {line}")
                    current = {
                        'name': name,
                        'type': 'none',
                        'radius': 0.0,
                        'half_height': 0.0,
                        'half_extents': (0.0, 0.0, 0.0),
                        'offset': (0.0, 0.0, 0.0),
                    }
                    mode = "bone"

            elif mode == "bone":
                if cmd == '}':
                    bones.append(current)
                    current = None
                    mode = "root"

                elif cmd == "type":
                    current['type'] = tokens[1]

                elif cmd == "radius":
                    current['radius'] = float(tokens[1])

                elif cmd == "half_height":
                    current['half_height'] = float(tokens[1])

                elif cmd == "half_extents":
                    current['half_extents'] = (
                        float(tokens[1]), float(tokens[2]), float(tokens[3]))

                elif cmd == "offset":
                    current['offset'] = (
                        float(tokens[1]), float(tokens[2]), float(tokens[3]))

    return bones


def save_bone_collision(joints, collision_bones, output_path, blender_fix):
    """Write a .boncol binary file.

    Maps bone names from collision_bones to joint indices in the MD5 model.
    Only bones present in both the collision data and the joint list are written.
    """
    # Build name->index map from joints
    joint_map = {}
    for i, joint in enumerate(joints):
        joint_map[joint.name] = i

    # Resolve collision bones to joint indices
    resolved = []
    for cb in collision_bones:
        name = cb['name']
        if name not in joint_map:
            print(f"  WARNING: Bone '{name}' in collision data not found in "
                  f"model joints, skipping")
            continue
        resolved.append((joint_map[name], cb))

    # Sort by joint index for predictable output
    resolved.sort(key=lambda x: x[0])

    num_bones = len(resolved)
    if num_bones == 0:
        print("  WARNING: No collision bones matched model joints")
        return

    with open(output_path, 'wb') as f:
        # Header: magic, version, num_bones, reserved
        f.write(struct.pack('<IIII', BNCL_MAGIC, BNCL_VERSION, num_bones, 0))

        for joint_idx, cb in resolved:
            col_type = cb['type']
            offset = cb['offset']

            # NOTE: Do NOT apply blender_fix to bone-local offsets.
            # The bone's quaternion in the DSA file already includes the
            # blender_fix rotation (q_rot * orient). At runtime,
            # ne_quat_rotate_vec(bone_orient, offset) transforms the offset
            # from bone-local to model space. Applying blender_fix here
            # would double-convert the offset and produce wrong positions.

            # Determine type code and params
            if col_type == 'sphere':
                type_code = BONCOL_TYPE_SPHERE
                p1 = cb['radius']
                p2 = 0.0
                p3 = 0.0
            elif col_type == 'capsule':
                type_code = BONCOL_TYPE_CAPSULE
                p1 = cb['radius']
                p2 = cb['half_height']
                p3 = 0.0
            elif col_type == 'aabb':
                type_code = BONCOL_TYPE_AABB
                p1 = cb['half_extents'][0]
                p2 = cb['half_extents'][1]
                p3 = cb['half_extents'][2]
                if blender_fix:
                    p1, p2, p3 = p1, p3, p2
            else:
                type_code = BONCOL_TYPE_NONE
                p1 = p2 = p3 = 0.0

            # Per-bone entry: 32 bytes
            # type(u8) + bone_idx(u8) + pad(2) = 4 bytes
            # param1/2/3 (f32) = 12 bytes
            # offset_xyz (f32) = 12 bytes
            # reserved = 4 bytes
            f.write(struct.pack('<BBBB',
                type_code, joint_idx & 0xFF, 0, 0))
            f.write(struct.pack('<III',
                float_to_f32(p1), float_to_f32(p2), float_to_f32(p3)))
            f.write(struct.pack('<III',
                float_to_f32(offset[0]),
                float_to_f32(offset[1]),
                float_to_f32(offset[2])))
            f.write(struct.pack('<I', 0))  # reserved

    print(f"  Bone collision: {num_bones} bones -> {output_path}")


def convert_md5anim(name, output_folder, anim_file, skip_frames, extension_anim,
                    blender_fix, old_md5=False):

    print(f"Converting animation: {anim_file}")

    frames = parse_md5anim(anim_file, old_md5=old_md5)

    # Create name of animation based on file name
    file_basename = os.path.basename(anim_file).replace(".md5anim", "")
    anim_name = file_basename.replace(".", "_").lower()

    frames = frames[::skip_frames+1]
    save_animation(frames, os.path.join(output_folder,
                   f"{name}_{anim_name}{extension_anim}"), blender_fix)


if __name__ == "__main__":

    import argparse
    import sys
    import traceback

    print("md5_to_dsma v0.1.1")
    print("Copyright (c) 2022-2024 Antonio Niño Díaz <antonio_nd@outlook.com>")
    print("All rights reserved")
    print("")

    parser = argparse.ArgumentParser(
            description='Converts md5mesh and md5anim files into DSM and DSA files.')

    # Required arguments
    parser.add_argument("--name", required=True,
                        help="model name to be used in output files")
    parser.add_argument("--output", required=True,
                        help="output folder")

    # Optional arguments
    parser.add_argument("--model", required=False, type=str, default=None,
                        help="input md5mesh file")
    parser.add_argument("--texture", required=False, type=int, default=[],
                        nargs="+", action="extend",
                        help="texture width and height (e.g. '--texture 32 64')")
    parser.add_argument("--anims", required=False, type=str, default=[],
                        nargs="+", action="extend",
                        help="list of md5anim files to convert")
    parser.add_argument("--bin", required=False,
                        action='store_true',
                        help="add '.bin' to the name of the output files")
    parser.add_argument("--blender-fix", required=False,
                        action='store_true',
                        help="rotate model -90 degrees on X axis to match Blender's orientation")
    parser.add_argument("--export-base-pose", required=False,
                        action='store_true',
                        help="export base pose of a md5mesh as a DSA file")
    parser.add_argument("--skip-frames", required=False,
                        default=0, type=int,
                        help="number of frames to skip in an animation (0 = export all, 1 = export half, 2 = export 33%%, etc)")
    parser.add_argument("--draw-normal-polygons", required=False,
                        action='store_true',
                        help="draw polygons with the shape of normals for debugging")
    parser.add_argument("--no-strip", required=False,
                        action='store_true',
                        help="disable strip generation (original behavior)")
    parser.add_argument("--multi-material", required=False,
                        action='store_true',
                        help="output DLMM format with per-mesh materials "
                             "(texture sizes auto-detected from shader images)")
    parser.add_argument("--old-md5", required=False,
                        action='store_true',
                        help="parse old MD5 format without per-bone scale "
                             "(6 values per joint instead of 9)")
    parser.add_argument("--collision", required=False, type=str, default=None,
                        help="path to .md5collimesh file for per-bone collision "
                             "data (generates .boncol binary)")
    parser.add_argument("--format", required=False, choices=["dsma", "nsmw"],
                        default="dsma",
                        help="skinning format: 'dsma' (1 weight, rigid) or "
                             "'nsmw' (up to 2 weights, smooth)")
    parser.add_argument("--nsmw", required=False, action='store_true',
                        help="shortcut for --format nsmw")

    args = parser.parse_args()

    fmt = "nsmw" if args.nsmw else args.format

    if args.model is not None:
        if args.multi_material or fmt == "nsmw":
            # In multi-material/NSMW mode, --texture is optional (auto-detected
            # from the shader images)
            if len(args.texture) not in (0, 2):
                print("Please, provide exactly 0 or 2 values to the --texture argument "
                      "in multi-material/NSMW mode")
                sys.exit(1)
        else:
            if len(args.texture) != 2:
                print("Please, provide exactly 2 values to the --texture argument")
                sys.exit(1)

        if len(args.texture) == 2:
            if not is_valid_texture_size(args.texture[0]):
                print(f"Invalid texture width. Valid values: {VALID_TEXTURE_SIZES}")
                sys.exit(1)

            if not is_valid_texture_size(args.texture[1]):
                print(f"Invalid texture height. Valid values: {VALID_TEXTURE_SIZES}")
                sys.exit(1)

    # Create output directory if it doesn't exist
    os.makedirs(args.output, exist_ok=True)

    # Add '.bin' to the name of the files if requested
    if fmt == "nsmw":
        extension_mesh = "_nsmw.bin" if args.bin else ".nsmw"
    else:
        extension_mesh = "_dsm.bin" if args.bin else ".dsm"
    extension_anim = "_dsa.bin" if args.bin else ".dsa"

    try:
        if args.model is not None:
            if fmt == "nsmw":
                convert_md5mesh_nsmw(args.model, args.name, args.output,
                                     args.texture, extension_mesh,
                                     extension_anim, args.blender_fix,
                                     args.export_base_pose, args.no_strip)
            else:
                convert_md5mesh(args.model, args.name, args.output, args.texture,
                                args.draw_normal_polygons, extension_mesh,
                                extension_anim, args.blender_fix,
                                args.export_base_pose, args.no_strip,
                                args.multi_material)

        for anim_file in args.anims:
            convert_md5anim(args.name, args.output, anim_file, args.skip_frames,
                            extension_anim, args.blender_fix, args.old_md5)

        if args.collision is not None:
            if args.model is None:
                print("ERROR: --collision requires --model to map bone names "
                      "to joint indices")
                sys.exit(1)

            print(f"Processing bone collision: {args.collision}")
            collision_bones = parse_md5collimesh(args.collision)
            print(f"  Parsed {len(collision_bones)} bone collision entries")

            # Re-parse joints from model for name mapping
            joints, _ = parse_md5mesh(args.model, nsmw=(fmt == "nsmw"))

            extension_boncol = "_boncol.bin" if args.bin else ".boncol"
            boncol_path = os.path.join(args.output,
                                       f"{args.name}{extension_boncol}")
            save_bone_collision(joints, collision_bones, boncol_path,
                                args.blender_fix)

    except BaseException as e:
        print("ERROR: " + str(e))
        traceback.print_exc()
        sys.exit(1)
    except MD5FormatError as e:
        print("ERROR: Invalid MD5 file: " + str(e))
        traceback.print_exc()
        sys.exit(1)

    print("Done!")

    sys.exit(0)
