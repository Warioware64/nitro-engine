# SPDX-License-Identifier: MIT
#
# Copyright (c) 2024 Nitro Engine Advanced Contributors

"""Parser for Wavefront .mtl material library files.

Extracts material properties and converts them to NDS-compatible formats.
"""

import os


def float_to_rgb15(r, g, b):
    """Convert float RGB (0-1) to NDS RGB15 (5-5-5) format."""
    ri = max(0, min(31, int(r * 31)))
    gi = max(0, min(31, int(g * 31)))
    bi = max(0, min(31, int(b * 31)))
    return ri | (gi << 5) | (bi << 10)


def pack_diffuse_ambient(kd, ka, vtxcolor=False):
    """Pack diffuse + ambient into u32 matching NDS GFX_DIFFUSE_AMBIENT format.

    Args:
        kd: (r, g, b) diffuse color, floats 0-1
        ka: (r, g, b) ambient color, floats 0-1
        vtxcolor: if True, set the 'vertex color as diffuse' bit
    """
    diffuse = float_to_rgb15(*kd)
    ambient = float_to_rgb15(*ka)
    val = diffuse | (ambient << 16)
    if vtxcolor:
        val |= (1 << 15)
    return val


def pack_specular_emission(ks, ke, use_shininess=False):
    """Pack specular + emission into u32 matching NDS GFX_SPECULAR_EMISSION format.

    Args:
        ks: (r, g, b) specular color, floats 0-1
        ke: (r, g, b) emission color, floats 0-1
        use_shininess: if True, set the shininess table enable bit
    """
    specular = float_to_rgb15(*ks)
    emission = float_to_rgb15(*ke)
    val = specular | (emission << 16)
    if use_shininess:
        val |= (1 << 15)
    return val


def parse_mtl(mtl_path):
    """Parse a Wavefront .mtl file.

    Args:
        mtl_path: path to the .mtl file

    Returns:
        dict of material_name -> dict with keys:
            'Kd': (r, g, b) diffuse color (floats 0-1)
            'Ka': (r, g, b) ambient color
            'Ks': (r, g, b) specular color
            'Ke': (r, g, b) emission color
            'd':  float dissolve/alpha (1.0 = opaque)
            'map_Kd': str texture filename or None
    """
    materials = {}
    current = None

    with open(mtl_path, 'r') as f:
        for line in f:
            line = line.split('#')[0].strip()
            if not line:
                continue

            tokens = line.split()
            cmd = tokens[0]

            if cmd == 'newmtl':
                name = ' '.join(tokens[1:])
                current = {
                    'Kd': (1.0, 1.0, 1.0),
                    'Ka': (0.0, 0.0, 0.0),
                    'Ks': (0.0, 0.0, 0.0),
                    'Ke': (0.0, 0.0, 0.0),
                    'd': 1.0,
                    'map_Kd': None,
                }
                materials[name] = current

            elif current is None:
                continue

            elif cmd == 'Kd' and len(tokens) >= 4:
                current['Kd'] = (float(tokens[1]), float(tokens[2]), float(tokens[3]))

            elif cmd == 'Ka' and len(tokens) >= 4:
                current['Ka'] = (float(tokens[1]), float(tokens[2]), float(tokens[3]))

            elif cmd == 'Ks' and len(tokens) >= 4:
                current['Ks'] = (float(tokens[1]), float(tokens[2]), float(tokens[3]))

            elif cmd == 'Ke' and len(tokens) >= 4:
                current['Ke'] = (float(tokens[1]), float(tokens[2]), float(tokens[3]))

            elif cmd == 'd' and len(tokens) >= 2:
                current['d'] = float(tokens[1])

            elif cmd == 'Tr' and len(tokens) >= 2:
                # Tr is the inverse of d
                current['d'] = 1.0 - float(tokens[1])

            elif cmd == 'map_Kd' and len(tokens) >= 2:
                # Store the texture filename (basename only)
                tex_path = ' '.join(tokens[1:])
                current['map_Kd'] = os.path.basename(tex_path)

    return materials
