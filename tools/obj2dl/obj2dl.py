#!/usr/bin/env python3

# SPDX-License-Identifier: MIT
#
# Copyright (c) 2022-2024 Antonio Niño Díaz <antonio_nd@outlook.com>

import os
import struct

from display_list import DisplayList
from mtl_parser import parse_mtl, float_to_rgb15, pack_diffuse_ambient, pack_specular_emission
from collections import defaultdict

class OBJFormatError(Exception):
    pass

VALID_TEXTURE_SIZES = [8, 16, 32, 64, 128, 256, 512, 1024]

def is_valid_texture_size(size):
    return size in VALID_TEXTURE_SIZES

def get_image_dimensions(path):
    """Read width and height from a PNG or JPEG file without external deps."""
    with open(path, 'rb') as f:
        header = f.read(32)

        # PNG: signature (8 bytes) then IHDR chunk (length(4) + 'IHDR'(4) + w(4) + h(4))
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
                if marker in (0xC0, 0xC2):  # SOF0 or SOF2
                    h = struct.unpack('>H', data[i+5:i+7])[0]
                    w = struct.unpack('>H', data[i+7:i+9])[0]
                    return w, h
                if marker == 0xD9:  # EOI
                    break
                if marker in (0xD0, 0xD1, 0xD2, 0xD3, 0xD4, 0xD5,
                               0xD6, 0xD7, 0xD8, 0x01):
                    i += 2
                else:
                    seg_len = struct.unpack('>H', data[i+2:i+4])[0]
                    i += 2 + seg_len

    raise ValueError(f"Cannot read dimensions from {path}")

def parse_face_vertex(vertex_str, use_vertex_color):
    """Parse 'v/vt/vn' string, return (vertex_idx, texcoord_idx, normal_idx) 0-based."""
    tokens = vertex_str.split('/')
    vertex_index = None
    texcoord_index = None
    normal_index = None

    if len(tokens) == 1:
        vertex_index = int(tokens[0])
    elif len(tokens) == 2:
        vertex_index = int(tokens[0])
        texcoord_index = int(tokens[1])
    elif len(tokens) == 3:
        vertex_index = int(tokens[0])
        if len(tokens[1]) > 0:
            texcoord_index = int(tokens[1])
        if not use_vertex_color:
            normal_index = int(tokens[2])
    else:
        raise OBJFormatError(f"Invalid face vertex {vertex_str}")

    if vertex_index < 0:
        raise OBJFormatError("Unsupported negative indices")
    vertex_index -= 1

    if texcoord_index is not None:
        if texcoord_index < 0:
            raise OBJFormatError("Unsupported negative indices")
        texcoord_index -= 1

    if normal_index is not None:
        if normal_index < 0:
            raise OBJFormatError("Unsupported negative indices")
        normal_index -= 1

    return (vertex_index, texcoord_index, normal_index)

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
      strips = list of vertex-key lists (each is a triangle strip sequence)
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
        # Try all 3 CCW rotations as forward direction
        for rot in range(3):
            a = face[rot]
            b = face[(rot + 1) % 3]
            c = face[(rot + 2) % 3]

            strip = [a, b, c]
            faces = [start_fi]
            local_used = set(used)
            local_used.add(start_fi)

            # Extend forward
            while True:
                n = len(faces)  # index of next triangle in strip
                p, q = strip[-2], strip[-1]
                edge_key = frozenset([p, q])

                found = False
                for cfi in edge_to_faces[edge_key]:
                    if cfi in local_used:
                        continue
                    tri = resolved_tris[cfi]
                    # Even position: need directed edge p->q in candidate
                    # Odd position: need directed edge q->p in candidate
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
            strips.append(best_strip)
        else:
            singles.append(start_fi)

    return strips, singles

# ---------------------------------------------------------------------------
# Quad strip helpers
# ---------------------------------------------------------------------------

def stripify_quads(resolved_quads):
    """Greedy quad stripification.
    resolved_quads: list of (vk0, vk1, vk2, vk3) in CCW order.

    NDS quad strip layout:
      Strip sequence: s0, s1, s2, s3, s4, s5, ...
      Quad i uses: (s[2i], s[2i+1], s[2i+3], s[2i+2]) in CCW
      So for OBJ quad (a,b,c,d) CCW -> strip emits a, b, d, c
      Exit directed edge (right side) = c->d in CCW
      Next quad must have entry directed edge d->c (reversed)

    Returns (strips, singles).
    """
    if not resolved_quads:
        return [], []

    edge_to_faces = defaultdict(list)
    for fi, quad in enumerate(resolved_quads):
        for i in range(4):
            edge = frozenset([quad[i], quad[(i + 1) % 4]])
            edge_to_faces[edge].append(fi)

    used = set()
    strips = []
    singles = []

    for start_fi in range(len(resolved_quads)):
        if start_fi in used:
            continue

        best_strip = None
        best_faces = None
        quad = resolved_quads[start_fi]

        # Try all 4 rotations (determines which edge pair is entry/exit)
        for rot in range(4):
            v = [quad[(rot + j) % 4] for j in range(4)]
            # CCW: v[0], v[1], v[2], v[3]
            # Strip order: v[0], v[1], v[3], v[2]
            strip = [v[0], v[1], v[3], v[2]]
            faces = [start_fi]
            local_used = set(used)
            local_used.add(start_fi)

            # Exit directed edge in CCW: v[2]->v[3]
            exit_p, exit_q = v[2], v[3]

            while True:
                edge_key = frozenset([exit_p, exit_q])
                found = False
                for cfi in edge_to_faces[edge_key]:
                    if cfi in local_used:
                        continue
                    candidate = resolved_quads[cfi]
                    # Need directed edge exit_q->exit_p in candidate CCW
                    for ci in range(4):
                        if candidate[ci] == exit_q and candidate[(ci + 1) % 4] == exit_p:
                            cv = [candidate[(ci + j) % 4] for j in range(4)]
                            # cv CCW: cv[0]=exit_q, cv[1]=exit_p, cv[2], cv[3]
                            # Strip adds: cv[3], cv[2]
                            strip.extend([cv[3], cv[2]])
                            faces.append(cfi)
                            local_used.add(cfi)
                            exit_p, exit_q = cv[2], cv[3]
                            found = True
                            break
                    if found:
                        break
                if not found:
                    break

            if best_faces is None or len(faces) > len(best_faces):
                best_strip = strip
                best_faces = faces

        for fi in best_faces:
            used.add(fi)

        if len(best_faces) >= 2:
            strips.append(best_strip)
        else:
            singles.append(start_fi)

    return strips, singles

# ---------------------------------------------------------------------------
# Vertex emission
# ---------------------------------------------------------------------------

def emit_vertex(dl, vk, vertices, texcoords, normals, texture_size,
                model_scale, model_translation, use_vertex_color):
    """Emit all display-list commands for one vertex key."""
    vertex_idx, texcoord_idx, normal_idx = vk

    if texcoord_idx is not None:
        u, v = texcoords[texcoord_idx]
        # OBJ (0,0) = bottom-left, NDS (0,0) = top-left
        v = 1.0 - v
        u *= texture_size[0]
        v *= texture_size[1]
        dl.texcoord(u, v)

    if normal_idx is not None:
        n = normals[normal_idx]
        dl.normal(n[0], n[1], n[2])

    vtx = []
    for i in range(3):
        val = vertices[vertex_idx][i]
        val += model_translation[i]
        val *= model_scale
        vtx.append(val)

    if use_vertex_color:
        rgb = [vertices[vertex_idx][i] for i in range(3, 6)]
        dl.color(*rgb)

    dl.vtx(vtx[0], vtx[1], vtx[2])

# ---------------------------------------------------------------------------
# Display list generation for a set of faces
# ---------------------------------------------------------------------------

def generate_display_list(resolved_tris, resolved_quads, vertices, texcoords,
                          normals, texture_size, model_scale, model_translation,
                          use_vertex_color, no_strip):
    """Generate a display list for a group of faces.

    Returns a finalized DisplayList instance.
    """
    if no_strip:
        tri_strips, tri_singles = [], list(range(len(resolved_tris)))
        quad_strips, quad_singles = [], list(range(len(resolved_quads)))
    else:
        tri_strips, tri_singles = stripify_triangles(resolved_tris)
        quad_strips, quad_singles = stripify_quads(resolved_quads)

    # Statistics
    separate_vtx = len(resolved_tris) * 3 + len(resolved_quads) * 4
    strip_vtx = (sum(len(s) for s in tri_strips) + len(tri_singles) * 3
               + sum(len(s) for s in quad_strips) + len(quad_singles) * 4)

    tri_stripped = len(resolved_tris) - len(tri_singles)
    quad_stripped = len(resolved_quads) - len(quad_singles)

    print(f"  Triangle strips: {len(tri_strips)} ({tri_stripped} faces stripped, "
          f"{len(tri_singles)} separate)")
    print(f"  Quad strips:     {len(quad_strips)} ({quad_stripped} faces stripped, "
          f"{len(quad_singles)} separate)")
    print(f"  GPU vertices:    {separate_vtx} -> {strip_vtx} "
          f"(saved {separate_vtx - strip_vtx})")

    dl = DisplayList()

    # Emit triangle strips
    for strip_verts in tri_strips:
        dl.begin_vtxs("triangle_strip")
        for vk in strip_verts:
            emit_vertex(dl, vk, vertices, texcoords, normals, texture_size,
                        model_scale, model_translation, use_vertex_color)
        dl.end_vtxs()

    # Emit separate triangles
    if tri_singles:
        dl.begin_vtxs("triangles")
        for fi in tri_singles:
            for vk in resolved_tris[fi]:
                emit_vertex(dl, vk, vertices, texcoords, normals, texture_size,
                            model_scale, model_translation, use_vertex_color)
        dl.end_vtxs()

    # Emit quad strips
    for strip_verts in quad_strips:
        dl.begin_vtxs("quad_strip")
        for vk in strip_verts:
            emit_vertex(dl, vk, vertices, texcoords, normals, texture_size,
                        model_scale, model_translation, use_vertex_color)
        dl.end_vtxs()

    # Emit separate quads
    if quad_singles:
        dl.begin_vtxs("quads")
        for fi in quad_singles:
            for vk in resolved_quads[fi]:
                emit_vertex(dl, vk, vertices, texcoords, normals, texture_size,
                            model_scale, model_translation, use_vertex_color)
        dl.end_vtxs()

    dl.finalize()
    return dl

# ---------------------------------------------------------------------------
# DLMM binary format writer
# ---------------------------------------------------------------------------

DLMM_MAGIC = 0x4D4D4C44  # "DLMM" in little-endian
DLMM_VERSION = 1
DLMM_SUBMESH_HEADER_SIZE = 56  # bytes per submesh header

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

    # Get binary data for each display list
    dl_binaries = []
    for sub in submeshes:
        dl_binaries.append(sub['dl'].get_binary())

    # Calculate offsets for display list data
    dl_offsets = []
    offset = header_size
    for dl_bin in dl_binaries:
        dl_offsets.append(offset)
        offset += len(dl_bin)

    with open(output_file, 'wb') as f:
        # File header
        f.write(struct.pack('<III', DLMM_MAGIC, DLMM_VERSION, num))

        # Submesh headers
        for i, sub in enumerate(submeshes):
            flags = 0
            if sub['has_texture']:
                flags |= 1

            # Pack name into 32 bytes (null-terminated, zero-padded)
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

        # Display list data
        for dl_bin in dl_binaries:
            f.write(dl_bin)

# ---------------------------------------------------------------------------
# OBJ parsing
# ---------------------------------------------------------------------------

def parse_obj(input_file, use_vertex_color):
    """Parse an OBJ file and return geometry + material groupings.

    Returns:
        vertices, texcoords, normals: lists of parsed geometry
        material_faces: dict of material_name -> list of face token lists
        mtl_file: path to .mtl file (or None)
    """
    vertices = []
    texcoords = []
    normals = []
    material_faces = defaultdict(list)
    current_material = "__default__"
    mtl_file = None

    with open(input_file, 'r') as obj_file:
        for line in obj_file:
            line = line.split('#')[0]
            tokens = line.split()
            if len(tokens) < 2:
                continue

            cmd = tokens[0]
            tokens = tokens[1:]

            if cmd == 'mtllib':
                mtl_file = os.path.join(os.path.dirname(input_file), tokens[0])

            elif cmd == 'usemtl':
                current_material = ' '.join(tokens)

            elif cmd == 'v':
                if len(tokens) == 3:
                    if use_vertex_color:
                        raise OBJFormatError(f"Found vertex with no color info: {tokens}")
                    v = [float(tokens[i]) for i in range(3)]
                elif len(tokens) == 6:
                    v = [float(tokens[i]) for i in range(6)]
                else:
                    raise OBJFormatError(f"Unsupported vertex command: {tokens}")
                vertices.append(v)

            elif cmd == 'vt':
                v = (float(tokens[0]), float(tokens[1]))
                texcoords.append(v)

            elif cmd == 'vn':
                v = (float(tokens[0]), float(tokens[1]), float(tokens[2]))
                normals.append(v)

            elif cmd == 'f':
                material_faces[current_material].append(tokens)

            elif cmd == 'l':
                raise OBJFormatError(f"Unsupported polyline command: {tokens}")

            else:
                pass  # Silently ignore other commands

    return vertices, texcoords, normals, material_faces, mtl_file

# ---------------------------------------------------------------------------
# Main conversion
# ---------------------------------------------------------------------------

def convert_obj(input_file, output_file, texture_size,
                model_scale, model_translation, use_vertex_color,
                no_strip=False, multi_material=False):

    vertices, texcoords, normals, material_faces, mtl_file = \
        parse_obj(input_file, use_vertex_color)

    print(f"Vertices:  {len(vertices)}")
    print(f"Texcoords: {len(texcoords)}")
    print(f"Normals:   {len(normals)}")
    print(f"Materials: {len(material_faces)}")
    print("")

    # Load MTL if available
    materials = {}
    if mtl_file and os.path.isfile(mtl_file):
        materials = parse_mtl(mtl_file)
        print(f"Loaded {len(materials)} materials from {os.path.basename(mtl_file)}")
        for name in materials:
            mat = materials[name]
            tex_info = f" [tex: {mat['map_Kd']}]" if mat['map_Kd'] else ""
            print(f"  - {name}: Kd={mat['Kd']}, d={mat['d']}{tex_info}")
        print("")
    elif mtl_file:
        print(f"Warning: MTL file not found: {mtl_file}")
        print("")

    # Determine if we should use multi-material mode
    use_multi = multi_material and len(material_faces) > 1

    if not use_multi:
        # ---- Legacy single-material path ----
        all_faces = []
        for face_list in material_faces.values():
            all_faces.extend(face_list)

        print(f"Faces:     {len(all_faces)}")
        print("")

        resolved_tris = []
        resolved_quads = []

        for face in all_faces:
            n = len(face)
            if n != 3 and n != 4:
                raise OBJFormatError(
                    f"Unsupported polygons with {n} faces. "
                    "Please, split the polygons in your model to triangles."
                )
            vkeys = tuple(parse_face_vertex(v, use_vertex_color) for v in face)
            if n == 3:
                resolved_tris.append(vkeys)
            else:
                resolved_quads.append(vkeys)

        print("Single-material mode:")
        dl = generate_display_list(resolved_tris, resolved_quads, vertices,
                                    texcoords, normals, texture_size,
                                    model_scale, model_translation,
                                    use_vertex_color, no_strip)
        dl.save_to_file(output_file)
    else:
        # ---- Multi-material path ----
        # Resolve base directory for texture image lookups
        mtl_dir = os.path.dirname(mtl_file) if mtl_file else os.path.dirname(input_file)

        submeshes = []
        total_faces = 0

        for mat_name, face_list in material_faces.items():
            print(f"Material '{mat_name}': {len(face_list)} faces")
            total_faces += len(face_list)

            resolved_tris = []
            resolved_quads = []
            for face in face_list:
                n = len(face)
                if n != 3 and n != 4:
                    raise OBJFormatError(
                        f"Unsupported polygons with {n} faces in material '{mat_name}'. "
                        "Please, split the polygons in your model to triangles."
                    )
                vkeys = tuple(parse_face_vertex(v, use_vertex_color) for v in face)
                if n == 3:
                    resolved_tris.append(vkeys)
                else:
                    resolved_quads.append(vkeys)

            # Determine per-material texture size from map_Kd image
            mat_props = materials.get(mat_name, {})
            mat_tex_size = list(texture_size)  # fallback to CLI --texture
            map_kd = mat_props.get('map_Kd')
            if map_kd:
                tex_path = os.path.join(mtl_dir, map_kd)
                if os.path.isfile(tex_path):
                    w, h = get_image_dimensions(tex_path)
                    if is_valid_texture_size(w) and is_valid_texture_size(h):
                        mat_tex_size = [w, h]
                        print(f"  Texture: {map_kd} ({w}x{h})")
                    else:
                        print(f"  Warning: {map_kd} has non-NDS size {w}x{h}, "
                              f"using fallback {mat_tex_size}")
                else:
                    print(f"  Warning: texture not found: {tex_path}")

            dl = generate_display_list(resolved_tris, resolved_quads, vertices,
                                        texcoords, normals, mat_tex_size,
                                        model_scale, model_translation,
                                        use_vertex_color, no_strip)

            kd = mat_props.get('Kd', (1.0, 1.0, 1.0))
            ka = mat_props.get('Ka', (0.0, 0.0, 0.0))
            ks = mat_props.get('Ks', (0.0, 0.0, 0.0))
            ke = mat_props.get('Ke', (0.0, 0.0, 0.0))
            alpha = mat_props.get('d', 1.0)
            has_tex = mat_props.get('map_Kd') is not None

            submeshes.append({
                'name': mat_name,
                'dl': dl,
                'diffuse_ambient': pack_diffuse_ambient(kd, ka),
                'specular_emission': pack_specular_emission(ks, ke),
                'color': float_to_rgb15(*kd),
                'alpha': max(0, min(31, int(alpha * 31))),
                'has_texture': has_tex,
            })
            print("")

        print(f"Total faces: {total_faces}")
        print(f"Submeshes:   {len(submeshes)}")
        print(f"Output:      {output_file} (DLMM format)")
        save_dlmm(output_file, submeshes)

if __name__ == "__main__":

    import argparse
    import sys
    import traceback

    print("obj2dl v0.3.0")
    print("Copyright (c) 2022-2024 Antonio Niño Díaz <antonio_nd@outlook.com>")
    print("Multi-material support by Nitro Engine Advanced Contributors")
    print("")

    parser = argparse.ArgumentParser(
            description='Convert Wavefront OBJ files into NDS display lists.')

    # Required arguments
    parser.add_argument("--input", required=True,
                        help="input file")
    parser.add_argument("--output", required=True,
                        help="output file")

    # Optional arguments
    parser.add_argument("--texture", default=None, type=int,
                        nargs="+", action="extend",
                        help="texture width and height (e.g. '--texture 32 64')")
    parser.add_argument("--translation", default=[0, 0, 0], type=float,
                        nargs="+", action="extend",
                        help="translate model by this value")
    parser.add_argument("--scale", default=1.0, type=float,
                        help="scale model by this value (after the translation)")
    parser.add_argument("--use-vertex-color", required=False,
                        action='store_true',
                        help="use vertex colors instead of normals")
    parser.add_argument("--no-strip", required=False,
                        action='store_true',
                        help="disable strip generation (original behavior)")
    parser.add_argument("--multi-material", required=False,
                        action='store_true',
                        help="enable multi-material output (DLMM format)")

    args = parser.parse_args()

    # Texture size: required for single-material, optional for multi-material
    texture_size = [0, 0]
    if args.texture is not None:
        if len(args.texture) != 2:
            print("Please, provide exactly 2 values to the --texture argument")
            sys.exit(1)
        if not is_valid_texture_size(args.texture[0]):
            print(f"Invalid texture width. Valid values: {VALID_TEXTURE_SIZES}")
            sys.exit(1)
        if not is_valid_texture_size(args.texture[1]):
            print(f"Invalid texture height. Valid values: {VALID_TEXTURE_SIZES}")
            sys.exit(1)
        texture_size = args.texture
    elif not args.multi_material:
        print("--texture is required for single-material mode")
        sys.exit(1)

    if len(args.translation) != 3:
        print("Please, provide exactly 3 values to the --translation argument")
        sys.exit(1)

    try:
        convert_obj(args.input, args.output, texture_size,
                    args.scale, args.translation, args.use_vertex_color,
                    args.no_strip, args.multi_material)
    except BaseException as e:
        print("ERROR: " + str(e))
        traceback.print_exc()
        sys.exit(1)
    except OBJFormatError as e:
        print("ERROR: Invalid OBJ file: " + str(e))
        traceback.print_exc()
        sys.exit(1)

    print("Done!")

    sys.exit(0)
