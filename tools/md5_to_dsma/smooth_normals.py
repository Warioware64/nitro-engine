#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
#
# Copyright (c) 2026 Warioware64
#
# Crease-angle vertex normal smoothing, shared by the two mesh exporters.

"""smooth_normals.py -- average face normals across shallow edges.

A triangle's own normal is constant across its surface, which is what makes a
flat-shaded mesh look faceted. Smoothing replaces each *corner's* normal with
the average of the faces meeting at that vertex -- but only the ones that meet
it shallowly enough to be part of the same surface. An edge sharper than the
threshold stays hard, which is what keeps a cube looking like a cube.

Three details are worth being deliberate about, because each is a quiet way to
get this wrong:

    Adjacency is by position, not by index. An OBJ exported per-face has a
    separate vertex entry for every corner, so two triangles meeting along an
    edge share no indices at all. Keying on the index would find no neighbours
    and silently produce flat output on exactly the files most likely to be
    fed in.

    The averaging is area-weighted, and that comes free: the cross product's
    magnitude is twice the triangle's area, so summing the *un-normalised* face
    normals weights each contribution by how much surface it represents. A fan
    of slivers then cannot outvote the one large face a vertex mostly belongs
    to.

    The angle test compares *normalised* normals while the sum uses the
    un-normalised ones. Using the un-normalised normals for the test would make
    the threshold depend on triangle size.

This module duplicates into both tool directories, the way display_list.py
already does, so each tool stays runnable from its own directory.
"""

import math

# Positions are snapped to this grid before being compared, so that corners that
# are meant to be the same vertex are treated as one. Fine enough not to weld
# anything an artist meant to keep apart, coarse enough to absorb the rounding
# an exporter leaves in ASCII coordinates.
WELD_EPSILON = 1e-5


def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def _length(v):
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])


def _normalize(v):
    n = _length(v)
    if n <= 1e-12:
        return (0.0, 0.0, 0.0)
    return (v[0] / n, v[1] / n, v[2] / n)


def face_normal(p0, p1, p2):
    """Un-normalised face normal. Its magnitude is twice the triangle's area."""
    return _cross(_sub(p1, p0), _sub(p2, p0))


def _key(p):
    return (round(p[0] / WELD_EPSILON),
            round(p[1] / WELD_EPSILON),
            round(p[2] / WELD_EPSILON))


def build_smooth_normals(triangles, angle_degrees):
    """Smooth per-corner normals for a list of triangles.

    Args:
        triangles: [(p0, p1, p2), ...] where each p is an (x, y, z) sequence.
                   Whole model at once -- a surface that runs across a material
                   boundary should not pick up a lighting seam there.
        angle_degrees: faces meeting at less than this angle are averaged
                       together. 0 keeps everything flat, 180 smooths
                       everything.

    Returns:
        [[n0, n1, n2], ...] -- one unit normal per corner, in the same order.
        A degenerate triangle gets three zero normals.
    """
    cos_limit = math.cos(math.radians(max(0.0, min(180.0, angle_degrees))))

    # Un-normalised (area-weighted) and normalised (for the angle test) face
    # normals, computed once.
    raw = []
    unit = []
    for p0, p1, p2 in triangles:
        n = face_normal(p0, p1, p2)
        raw.append(n)
        unit.append(_normalize(n))

    # Which triangles touch each welded position.
    incident = {}
    for ti, (p0, p1, p2) in enumerate(triangles):
        for corner, p in enumerate((p0, p1, p2)):
            incident.setdefault(_key(p), []).append(ti)

    out = []
    for ti, (p0, p1, p2) in enumerate(triangles):
        mine = unit[ti]

        if mine == (0.0, 0.0, 0.0):
            # Degenerate: it has no direction to contribute or to average
            # towards, and letting it join in would drag its neighbours.
            out.append([(0.0, 0.0, 0.0)] * 3)
            continue

        corners = []
        for p in (p0, p1, p2):
            sx = sy = sz = 0.0
            for other in incident[_key(p)]:
                o = unit[other]
                if o == (0.0, 0.0, 0.0):
                    continue
                dot = mine[0] * o[0] + mine[1] * o[1] + mine[2] * o[2]
                if dot >= cos_limit:
                    sx += raw[other][0]
                    sy += raw[other][1]
                    sz += raw[other][2]

            n = _normalize((sx, sy, sz))
            # Everything cancelled out -- two coincident faces pointing
            # opposite ways, say. The face's own normal is the honest answer.
            corners.append(n if n != (0.0, 0.0, 0.0) else mine)

        out.append(corners)

    return out
