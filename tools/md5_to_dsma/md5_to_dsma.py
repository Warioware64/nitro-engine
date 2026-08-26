#!/usr/bin/env python3

# SPDX-License-Identifier: MIT
#
# Copyright (c) 2022 Antonio Niño Díaz <antonio_nd@outlook.com>

import os
import struct

from collections import namedtuple, defaultdict
from math import sqrt

from display_list import DisplayList, float_to_f32, float_to_n10
from smooth_normals import build_smooth_normals

class MD5FormatError(Exception):
    pass

VALID_TEXTURE_SIZES = [8, 16, 32, 64, 128, 256, 512, 1024]

def is_valid_texture_width(size):
    """The S axis must be an exact NDS texture size."""
    return size in VALID_TEXTURE_SIZES

def is_valid_texture_height(size):
    """The T axis may be trimmed (ptexconv -tt).

    Nitro Engine Advanced stores only the real rows and tells the GPU the next
    power of two, so any height up to 1024 works. UVs are still scaled by the
    real height, which is what the trimmed data holds.
    """
    return 1 <= size <= 1024

def next_texture_pow2(size):
    """The height the GPU is told for a (possibly trimmed) texture."""
    for valid in VALID_TEXTURE_SIZES:
        if size <= valid:
            return valid
    return None

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

def smooth_normal_table(mesh_positions, angle):
    """Smoothed per-corner normals for every mesh of a model, in one pass.

    `mesh_positions` is [[(p0, p1, p2), ...], ...] -- one triangle list per
    mesh, each corner an (x, y, z) tuple in model space. Smoothing runs across
    every mesh at once, because an MD5 model's meshes are its material groups
    and a surface that runs across one should not pick up a lighting seam
    there.

    Returns {(mesh_index, tri_index, corner): (nx, ny, nz)}.
    """
    flat = []
    where = []
    for mi, tris in enumerate(mesh_positions):
        for ti, tri in enumerate(tris):
            flat.append(tri)
            where.append((mi, ti))

    if not flat:
        return {}

    out = {}
    for (mi, ti), corners in zip(where, build_smooth_normals(flat, angle)):
        for vi, n in enumerate(corners):
            out[(mi, ti, vi)] = n

    return out


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
                    multi_material=False, envmap_uv=False,
                    smooth_normals=None):

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

    # Smoothed normals are computed for the whole model at once, before any
    # mesh is processed, because MD5 meshes are the model's material groups and
    # a surface running across one should not gain a lighting seam there.
    smoothed = {}
    if smooth_normals is not None:
        mesh_positions = []
        for mesh in meshes:
            tris = []
            for tri in mesh.tris:
                posed = []
                for vi in tri:
                    weight = mesh.weights[mesh.verts[vi].startWeight]
                    joint = joints[weight.joint]
                    m = joint_info_to_m4x3(joint.orient, joint.pos)
                    p = weight.pos.mul_m4x3(m)
                    posed.append((p.x, p.y, p.z))
                tris.append(tuple(posed))
            mesh_positions.append(tris)

        smoothed = smooth_normal_table(mesh_positions, smooth_normals)
        print(f"  Smooth normals: {len(smoothed)} corner normal(s) at "
              f"{smooth_normals:g} degrees")

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
                        if not is_valid_texture_width(w):
                            raise ValueError(
                                f"Shader image {mesh.shader} is {w}x{h}: the "
                                f"width must be one of {VALID_TEXTURE_SIZES}")
                        if not is_valid_texture_height(h):
                            raise ValueError(
                                f"Shader image {mesh.shader} is {w}x{h}: the "
                                f"height must be between 1 and 1024")
                        mesh_tex_size = [w, h]
                        padded = next_texture_pow2(h)
                        if padded == h:
                            print(f"  Shader '{mesh.shader}': detected {w}x{h}")
                        else:
                            print(f"  Shader '{mesh.shader}': detected {w}x{h} "
                                  f"(trimmed T axis, GPU sees {w}x{padded})")
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

        for ti, (tri, norm) in enumerate(zip(mesh.tris, tri_normal)):
            verts = [mesh.verts[i] for i in tri]
            weights = [mesh.weights[v.startWeight] for v in verts]

            tri_vdata = []
            for corner, (vert, weight) in enumerate(zip(verts, weights)):
                # Smoothing replaces the face normal per corner. Everything
                # after this is unchanged: the rotation into joint space was
                # already per corner, it just used to be handed the same value
                # three times.
                sm = smoothed.get((mesh_index, ti, corner))
                norm = Vector(*sm) if sm else tri_normal[ti]
                st = vert.st
                if envmap_uv:
                    # Sphere mapping (NEA_TEXGEN_NORMAL) adds the generated
                    # coordinate to this one, so it has to be the centre of the
                    # texture for every vertex. The mesh's own UVs are dropped.
                    u = mesh_tex_size[0] / 2.0
                    v = mesh_tex_size[1] / 2.0
                else:
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

# Fitting a smoothly weighted mesh into the matrix-stack budget
# -------------------------------------------------------------
#
# NSMW spends one matrix-stack slot per distinct (joint pair, weight)
# combination, and the hardware stack has 31 levels. A rigidly weighted mesh
# needs one slot per bone and never comes close. A smoothly weighted one keys on
# the weight as well as the bones, so a 12-bone tentacle whose blend varies
# per vertex wants 194 slots for 11 actual bone pairs.
#
# Three stages bring that down, each doing less damage than the next, and each
# reporting what it cost so a silent loss of quality is impossible:
#
#   1. quantize -- round the blend to a fixed number of steps. Off by default:
#      measured against clustering alone on the tentacle test asset it makes the
#      result *worse* (0.37 worst-case blend error against 0.23), because it
#      throws away the vertex counts clustering uses to decide what matters.
#      It survives only as a speed knob, since clustering is O(n^2) and a mesh
#      with thousands of distinct weights would otherwise take minutes.
#   2. cluster  -- merge the closest remaining pair of nodes, over and over,
#      until the budget is met. Adaptive: a bone pair whose weights are spread
#      out keeps more of them than one whose weights are nearly the same, and
#      merges are weighted by how many vertices they affect.
#   3. split    -- partition the triangles into groups that each fit, and draw
#      them in several passes. The only stage that always succeeds, and the only
#      one that costs anything at run time.
#
# Clustering cannot go below one node per bone pair, so a mesh with more bone
# pairs than slots needs stage 3 no matter how coarse the weights get.


def _node_weight(node):
    """Blend factor of a node, as a float. Single-weight nodes sit at 1.0."""
    return 1.0 if node[0] == 1 else node[3]


def _quantize_nodes(nodes, usage, buckets):
    """Round every blend to 1/buckets and merge whatever collides.

    A blend that rounds all the way to 0 or 1 stops being a blend: the node
    becomes a plain single-weight one, which is both cheaper and exactly what
    the artist's weight was approaching.
    """
    remap = [0] * len(nodes)
    out = []
    seen = {}
    max_err = 0.0

    for i, node in enumerate(nodes):
        nw, j0, j1, w0, w1 = node

        if nw == 1:
            key = (1, j0, j0)
        else:
            q = round(w0 * buckets) / buckets
            max_err = max(max_err, abs(q - w0))

            if q >= 1.0:
                key = (1, j0, j0)
            elif q <= 0.0:
                key = (1, j1, j1)
            else:
                key = (2, j0, j1, q)

        idx = seen.get(key)
        if idx is None:
            idx = len(out)
            seen[key] = idx
            if key[0] == 1:
                out.append((1, key[1], key[1], 1.0, 0.0))
            else:
                out.append((2, key[1], key[2], key[3], 1.0 - key[3]))
        remap[i] = idx

    new_usage = [0] * len(out)
    for i, n in enumerate(usage):
        new_usage[remap[i]] += n

    return out, new_usage, remap, max_err


def _cluster_nodes(nodes, usage, max_nodes):
    """Merge the closest two nodes of a bone pair until the budget is met.

    The merged blend is the vertex-count-weighted mean of the two, so a node
    covering three vertices does not drag one covering three hundred.
    """
    # Live node state, indexed the same way as `nodes`. Merging rewrites the
    # survivor and marks the other dead; the remap is resolved at the end.
    alive = list(range(len(nodes)))
    parent = list(range(len(nodes)))
    weight = [_node_weight(n) for n in nodes]
    count = list(usage)
    max_err = 0.0

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    # Only two-weight nodes of the same bone pair can be merged; a single-weight
    # node is already the cheapest thing a vertex can reference.
    groups = {}
    for i, n in enumerate(nodes):
        if n[0] == 2:
            groups.setdefault((n[1], n[2]), []).append(i)

    live = len(nodes)

    while live > max_nodes:
        best = None
        for key, members in groups.items():
            members = [m for m in members if find(m) == m]
            groups[key] = members
            if len(members) < 2:
                continue
            members.sort(key=lambda m: weight[m])
            for a, b in zip(members, members[1:]):
                gap = weight[b] - weight[a]
                if best is None or gap < best[0]:
                    best = (gap, a, b)

        if best is None:
            # Every bone pair is down to a single node. Only splitting the mesh
            # can help from here.
            break

        _, a, b = best
        total = count[a] + count[b] or 1
        merged = (weight[a] * count[a] + weight[b] * count[b]) / total

        max_err = max(max_err, abs(merged - weight[a]), abs(merged - weight[b]))

        weight[a] = merged
        count[a] = total
        parent[b] = a
        live -= 1

    # Compact the survivors into a fresh table.
    remap = [0] * len(nodes)
    out = []
    index_of = {}
    for i, n in enumerate(nodes):
        root = find(i)
        if root not in index_of:
            index_of[root] = len(out)
            if n[0] == 1 and root == i:
                out.append((1, n[1], n[1], 1.0, 0.0))
            else:
                r = nodes[root]
                w = weight[root]
                out.append((2, r[1], r[2], w, 1.0 - w))
        remap[i] = index_of[root]

    new_usage = [0] * len(out)
    for i, n in enumerate(usage):
        new_usage[remap[i]] += n

    return out, new_usage, remap, max_err


def reduce_nodes(nodes, usage, max_nodes, buckets):
    """Bring a node table within budget. Returns (nodes, usage, remap, report)."""
    report = []
    original = list(nodes)
    original_usage = list(usage)
    remap = list(range(len(nodes)))
    start = len(nodes)

    def compose(outer):
        return [outer[r] for r in remap]

    if buckets > 0 and len(nodes) > max_nodes:
        nodes, usage, r, _ = _quantize_nodes(nodes, usage, buckets)
        remap = compose(r)
        report.append(f"quantize to 1/{buckets}: {start} -> {len(nodes)} nodes")

    if len(nodes) > max_nodes:
        before = len(nodes)
        nodes, usage, r, _ = _cluster_nodes(nodes, usage, max_nodes)
        remap = compose(r)
        report.append(f"cluster: {before} -> {len(nodes)} nodes")

    # Measure the damage once, against the weights the artist authored, rather
    # than accumulating it a merge at a time. Each merge moves a centroid, so
    # summing the steps would overstate what any single vertex actually lost.
    max_drift = 0.0
    total_drift = 0.0
    total_verts = 0
    for i, node in enumerate(original):
        final = nodes[remap[i]]
        drift = abs(_node_weight(node) - _node_weight(final))
        # A blend that collapsed onto one of its two joints moved all the way
        # to that joint: to j0 the blend went to 1, to j1 it went to 0.
        if node[0] == 2 and final[0] == 1:
            drift = (1.0 - node[3]) if final[1] == node[1] else node[3]
        max_drift = max(max_drift, drift)
        total_drift += drift * original_usage[i]
        total_verts += original_usage[i]

    if total_verts:
        report.append(f"blend error: {max_drift:.3f} worst, "
                      f"{total_drift / total_verts:.3f} average per vertex")

    return nodes, usage, remap, report


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
                         export_base_pose, no_strip=False, envmap_uv=False,
                         max_nodes=NSMW_MAX_NODES, weight_buckets=0,
                         smooth_normals=None):

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

    # Global node table shared by all submeshes. Weights are kept as floats
    # here and only converted to fixed point when the file is written, because
    # the reduction passes below need to measure distances between them.
    node_map = {}      # node key -> node index
    node_list = []     # list of (num_weights, j0, j1, w0, w1) with float weights
    node_usage = []    # how many vertices reference each node
    over_weight_warned = [False]

    def select_weights(mesh, vert):
        # The pure half of resolve_node: which one or two weights a vertex ends
        # up using, with no node-table side effects. Split out so the normal
        # smoothing pre-pass can compute bind positions without inflating the
        # node usage counts that the budget reduction later reads.
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
            key = (1, j, j, 1.0, 0.0)
        else:
            (j0, w0, p0), (j1, w1, p1) = sel
            # Canonicalize so the smaller joint index is first (stable dedup)
            if j1 < j0:
                (j0, w0, p0), (j1, w1, p1) = (j1, w1, p1), (j0, w0, p0)
            sel = [(j0, w0, p0), (j1, w1, p1)]
            # Dedupe at the resolution the hardware actually stores, so two
            # weights the GPU cannot tell apart do not take two slots.
            q0 = round(w0 * 4096) / 4096.0
            key = (2, j0, j1, q0, 1.0 - q0)

        return key, sel

    def resolve_node(mesh, vert):
        # Returns (node_index, selected), registering the node on the way.
        key, sel = select_weights(mesh, vert)

        idx = node_map.get(key)
        if idx is None:
            idx = len(node_list)
            node_map[key] = idx
            node_list.append(key)
            node_usage.append(0)
        node_usage[idx] += 1
        return idx, sel

    def compute_vbind(sel):
        vx = vy = vz = 0.0
        for (j, w, pos) in sel:
            p = pos.mul_m4x3(joint_rest[j])
            vx += w * p.x
            vy += w * p.y
            vz += w * p.z
        return Vector(vx, vy, vz)

    # Smoothed normals for the whole model, before any mesh is processed. The
    # bind positions come from select_weights(), the side-effect-free half of
    # resolve_node(), so this pre-pass does not inflate the node usage counts
    # that the budget reduction reads later.
    smoothed = {}
    if smooth_normals is not None:
        mesh_positions = []
        for mesh in meshes:
            vbind = [compute_vbind(select_weights(mesh, v)[1])
                     for v in mesh.verts]
            tris = [tuple((vbind[i].x, vbind[i].y, vbind[i].z) for i in tri)
                    for tri in mesh.tris]
            mesh_positions.append(tris)

        smoothed = smooth_normal_table(mesh_positions, smooth_normals)
        print(f"  Smooth normals: {len(smoothed)} corner normal(s) at "
              f"{smooth_normals:g} degrees")

    mesh_data = []  # per-mesh data needed to emit display lists later

    for mesh_index, mesh in enumerate(meshes):
        # Texture size (auto-detected from the shader image, like multi-material)
        mesh_tex_size = list(texture_size) if texture_size else [64, 64]
        if mesh.shader:
            shader_path = os.path.join(model_dir, mesh.shader)
            if os.path.isfile(shader_path):
                try:
                    w, h = get_image_dimensions(shader_path)
                    if not is_valid_texture_width(w):
                        raise ValueError(
                            f"Shader image {mesh.shader} is {w}x{h}: the width "
                            f"must be one of {VALID_TEXTURE_SIZES}")
                    if not is_valid_texture_height(h):
                        raise ValueError(
                            f"Shader image {mesh.shader} is {w}x{h}: the height "
                            f"must be between 1 and 1024")
                    mesh_tex_size = [w, h]
                    padded = next_texture_pow2(h)
                    if padded == h:
                        print(f"  Shader '{mesh.shader}': detected {w}x{h}")
                    else:
                        print(f"  Shader '{mesh.shader}': detected {w}x{h} "
                              f"(trimmed T axis, GPU sees {w}x{padded})")
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

                # Smoothing gives each corner its own normal; without it all
                # three share the triangle's.
                sm = smoothed.get((mesh_index, ti, vi))
                if sm:
                    norm = Vector(*sm)
                else:
                    norm = tri_normal[ti]

                # See the note in convert_md5mesh(): sphere mapping wants every
                # texture coordinate at the centre of the texture.
                tri_vdata.append({
                    'u': (mesh_tex_size[0] / 2.0) if envmap_uv
                         else st[0] * mesh_tex_size[0],
                    'v': (mesh_tex_size[1] / 2.0) if envmap_uv
                         else st[1] * mesh_tex_size[1],
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

    if len(node_list) == 0:
        raise MD5FormatError("NSMW model has no nodes")

    print(f"  Nodes wanted (matrix-palette slots): {len(node_list)}")

    if len(node_list) > max_nodes:
        node_list, node_usage, remap, report = reduce_nodes(
            node_list, node_usage, max_nodes, weight_buckets)

        for line in report:
            print(f"    {line}")

        # The node indices baked into the per-vertex data were assigned before
        # the reduction, so point them at the survivors.
        for md in mesh_data:
            for tri_vdata in md['all_tri_verts']:
                for d in tri_vdata:
                    d['node_index'] = remap[d['node_index']]

    num_nodes = len(node_list)
    print(f"  Nodes used: {num_nodes} (budget {max_nodes})")

    if num_nodes > max_nodes:
        pairs = len(set((n[1], n[2]) for n in node_list))
        raise MD5FormatError(
            f"NSMW model still needs {num_nodes} nodes after quantizing and "
            f"clustering, and the budget is {max_nodes}. It uses {pairs} "
            "distinct bone pairs, and clustering cannot go below one node per "
            "pair, so the mesh has to be split across several draws. Splitting "
            "is not implemented yet; for now, reduce the number of bone pairs "
            "the mesh actually blends between.")

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
    # The node table carried float weights so the reduction passes could
    # measure distances between them; the file format wants fixed point.
    node_list_fixed = [(n[0], n[1], n[2], float_to_f32(n[3]), float_to_f32(n[4]))
                       for n in node_list]

    save_nsmw(output_path, len(joints), node_list_fixed, invbind_list, submeshes)
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
                        'axis': None,
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

                elif cmd == "axis":
                    # Optional, and new with .b3col. A capsule should lie along
                    # its bone rather than along Y, and a shape's orientation is
                    # the one thing .boncol has no room for; this is where it
                    # comes from. Absent, the shape keeps .boncol's convention
                    # of being Y-aligned in bone-local space.
                    current['axis'] = (
                        float(tokens[1]), float(tokens[2]), float(tokens[3]))

    return bones


def resolve_collision_bones(joints, collision_bones):
    """Map bone names to joint indices, dropping any the model does not have.

    Shared by the .boncol and .b3col writers so that the two files always
    describe the same bone set in the same order.

    :return: a list of (joint_index, bone) sorted by joint index.
    """
    joint_map = {}
    for i, joint in enumerate(joints):
        joint_map[joint.name] = i

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
    return resolved


def save_bone_collision(joints, collision_bones, output_path, blender_fix):
    """Write a .boncol binary file.

    This is the format the *older* NEABoneCollision module reads. Box3D wants a
    .b3col instead -- see save_bone_collision_b3.
    """
    resolved = resolve_collision_bones(joints, collision_bones)

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


# ---------------------------------------------------------------------------
# Box3D bone collision (.b3col) support
# ---------------------------------------------------------------------------

B3CL_MAGIC = 0x4C433342   # "B3CL" little-endian
B3CL_VERSION = 1

# Type codes matching NEA_B3COL_TYPE_* in NEAPhysics3D.h.
B3COL_TYPE_NONE    = 0
B3COL_TYPE_SPHERE  = 1
B3COL_TYPE_CAPSULE = 2
B3COL_TYPE_BOX     = 3

# b3n, the port's scale for unit vectors and quaternions. A quaternion
# component is in [-1, 1], so storing one as f32 would spend 19 of 32 bits on an
# integer part that is always 0 and leave 12 fractional bits -- about a third of
# a degree of resolution on a hitbox. Q30 is what b3Quat holds, so the loader
# reads these straight in with no conversion and no loss.
B3_N_SHIFT = 30


def float_to_b3n(val):
    """Convert a quaternion or unit-vector component to b3n (Q30)."""
    res = int(val * (1 << B3_N_SHIFT))
    if res < -0x80000000 or res > 0x7FFFFFFF:
        raise MD5FormatError(
            f"{val} is out of range for a quaternion component")
    if res < 0:
        res = 0x100000000 + res
    return res


def quat_from_y_to_axis(axis):
    """The shortest-arc rotation taking +Y onto `axis`, as (x, y, z, w).

    A capsule in Box3D runs between two hemisphere centres, and this is what
    puts those centres along the bone instead of along Y. The degenerate cases
    matter: an axis already pointing along +Y needs no rotation, and one
    pointing along -Y has no shortest arc at all -- every half-turn about an
    axis in the XZ plane works, so one is picked.
    """
    length = sqrt(axis[0] ** 2 + axis[1] ** 2 + axis[2] ** 2)
    if length < 1e-9:
        return (0.0, 0.0, 0.0, 1.0)

    x, y, z = axis[0] / length, axis[1] / length, axis[2] / length

    # dot((0,1,0), axis) is just y.
    if y > 1.0 - 1e-9:
        return (0.0, 0.0, 0.0, 1.0)

    if y < -1.0 + 1e-9:
        # Antiparallel: a half turn about any perpendicular axis. +X will do.
        return (1.0, 0.0, 0.0, 0.0)

    # cross((0,1,0), axis) = (z, 0, -x), and the half-angle form keeps this to
    # one square root.
    cx, cy, cz = z, 0.0, -x
    w = 1.0 + y
    norm = sqrt(cx * cx + cy * cy + cz * cz + w * w)

    return (cx / norm, cy / norm, cz / norm, w / norm)


def save_bone_collision_b3(joints, collision_bones, output_path, blender_fix):
    """Write a .b3col binary file: per-bone Box3D shapes.

    The Box3D counterpart of .boncol. Same source data, same bone set, but the
    shapes are described the way b3Sphere, b3Capsule and b3MakeTransformedBoxHull
    want them, and each carries an orientation .boncol has no room for.

    The intended use is **one kinematic body per bone**, its transform driven
    each frame from the animated skeleton -- a single body cannot deform, so a
    hitbox set is bodies, not shapes on one body. NEA_Phys3DBodyAddBoneShapeI
    attaches one entry.

    Everything except the quaternion is f32 (Q19.12), which is exactly b3f, so
    nothing is requantized at load. There is no uint64_t in the layout, so unlike
    a .b3mesh a .b3col is fine 4-byte aligned and may go through bin2c into a
    project's data/ directory.
    """
    resolved = resolve_collision_bones(joints, collision_bones)

    num_bones = len(resolved)
    if num_bones == 0:
        print("  WARNING: No collision bones matched model joints")
        return

    counts = {'sphere': 0, 'capsule': 0, 'box': 0, 'none': 0}

    with open(output_path, 'wb') as f:
        # Header: magic, version, num_bones, reserved
        f.write(struct.pack('<IIII', B3CL_MAGIC, B3CL_VERSION, num_bones, 0))

        for joint_idx, cb in resolved:
            col_type = cb['type']
            offset = cb['offset']

            # NOTE: Do NOT apply blender_fix to bone-local offsets. The bone's
            # quaternion in the DSA file already includes the blender_fix
            # rotation, and the runtime transforms the offset from bone-local to
            # model space with it. Applying it here would double-convert. This
            # is .boncol's rule, restated because it is easy to lose.

            if col_type == 'sphere':
                type_code = B3COL_TYPE_SPHERE
                p1, p2, p3 = cb['radius'], 0.0, 0.0
            elif col_type == 'capsule':
                type_code = B3COL_TYPE_CAPSULE
                p1, p2, p3 = cb['radius'], cb['half_height'], 0.0
            elif col_type == 'aabb':
                # An 'aabb' in .md5collimesh is a box; Box3D can orient it, so
                # it stops being axis-aligned the moment `axis` is given.
                type_code = B3COL_TYPE_BOX
                p1, p2, p3 = cb['half_extents']
                if blender_fix:
                    p1, p2, p3 = p1, p3, p2
            else:
                type_code = B3COL_TYPE_NONE
                p1 = p2 = p3 = 0.0

            counts[{B3COL_TYPE_SPHERE: 'sphere', B3COL_TYPE_CAPSULE: 'capsule',
                    B3COL_TYPE_BOX: 'box', B3COL_TYPE_NONE: 'none'}[type_code]] += 1

            axis = cb.get('axis')
            rot = (0.0, 0.0, 0.0, 1.0) if axis is None else quat_from_y_to_axis(axis)

            # Per-bone entry: 48 bytes
            #   type(u8) + joint(u8) + pad(2)   =  4
            #   param1/2/3 (f32)                = 12
            #   offset xyz (f32)                = 12
            #   rotation xyzw (b3n, Q30)        = 16
            #   reserved                        =  4
            f.write(struct.pack('<BBBB', type_code, joint_idx & 0xFF, 0, 0))
            f.write(struct.pack('<III',
                float_to_f32(p1), float_to_f32(p2), float_to_f32(p3)))
            f.write(struct.pack('<III',
                float_to_f32(offset[0]),
                float_to_f32(offset[1]),
                float_to_f32(offset[2])))
            f.write(struct.pack('<IIII',
                float_to_b3n(rot[0]), float_to_b3n(rot[1]),
                float_to_b3n(rot[2]), float_to_b3n(rot[3])))
            f.write(struct.pack('<I', 0))  # reserved

    summary = ", ".join(f"{n} {kind}" for kind, n in counts.items() if n > 0)
    print(f"  Box3D bone collision: {num_bones} bones ({summary}) -> {output_path}")


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
                        help="texture width and height (e.g. '--texture 32 64'). The width must be a power of two; the height may be any value up to 1024, which is how a T-axis trimmed texture (ptexconv -tt) is declared")
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
    parser.add_argument("--max-nodes", required=False, type=int,
                        default=NSMW_MAX_NODES,
                        help=f"NSMW matrix-palette budget (default "
                             f"{NSMW_MAX_NODES}, the size of the hardware "
                             "matrix stack). Lower it to see the reduction "
                             "passes work harder.")
    parser.add_argument("--weight-buckets", required=False, type=int, default=0,
                        help="NSMW: round blend weights to this many steps "
                             "before clustering. Off by default -- it makes the "
                             "result measurably worse, and exists only to keep "
                             "the clustering pass tractable on meshes with "
                             "thousands of distinct weights.")
    parser.add_argument("--smooth-normals", required=False, type=float,
                        nargs='?', const=60.0, default=None,
                        help="average vertex normals across adjacent triangles "
                             "whose face angle is below this threshold in "
                             "degrees (default 60 if flag given with no "
                             "value); omit the flag entirely to keep flat "
                             "per-triangle normals")
    parser.add_argument("--envmap-uv", required=False,
                        action='store_true',
                        help="replace all texture coordinates with the centre "
                             "of the texture, as required by sphere-map "
                             "environment mapping (NEA_TEXGEN_NORMAL)")
    parser.add_argument("--old-md5", required=False,
                        action='store_true',
                        help="parse old MD5 format without per-bone scale "
                             "(6 values per joint instead of 9)")
    parser.add_argument("--collision", required=False, type=str, default=None,
                        help="path to .md5collimesh file for per-bone collision "
                             "data (generates .boncol binary, for the "
                             "NEABoneCollision module)")
    parser.add_argument("--collision-b3", required=False,
                        action='store_true',
                        help="also write a .b3col of per-bone Box3D shapes, for "
                             "NEA_Phys3DBodyAddBoneShape(). Needs --collision")
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
            if not is_valid_texture_width(args.texture[0]):
                print(f"Invalid texture width. Valid values: {VALID_TEXTURE_SIZES}")
                sys.exit(1)

            if not is_valid_texture_height(args.texture[1]):
                print("Invalid texture height. It must be between 1 and 1024 "
                      "(a height that isn't a power of two is a T-axis trimmed "
                      "texture, see ptexconv -tt)")
                sys.exit(1)

    if args.collision_b3 and args.collision is None:
        print("--collision-b3 needs --collision <file.md5collimesh>, which is "
              "where the shapes come from")
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
                                     args.export_base_pose, args.no_strip,
                                     args.envmap_uv, args.max_nodes,
                                     args.weight_buckets, args.smooth_normals)
            else:
                convert_md5mesh(args.model, args.name, args.output, args.texture,
                                args.draw_normal_polygons, extension_mesh,
                                extension_anim, args.blender_fix,
                                args.export_base_pose, args.no_strip,
                                args.multi_material, args.envmap_uv,
                                args.smooth_normals)

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

            # The Box3D shapes are an addition, not a replacement: .boncol
            # feeds the older NEABoneCollision module and a project can want
            # either or both.
            if args.collision_b3:
                extension_b3col = "_b3col.bin" if args.bin else ".b3col"
                b3col_path = os.path.join(args.output,
                                          f"{args.name}{extension_b3col}")
                save_bone_collision_b3(joints, collision_bones, b3col_path,
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
