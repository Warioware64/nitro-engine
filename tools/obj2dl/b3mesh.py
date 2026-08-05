#!/usr/bin/env python3
# SPDX-License-Identifier: CC0-1.0
#
# SPDX-FileContributor: Warioware64, 2026

"""Bake a triangle soup into a Box3D ``.b3mesh`` blob.

The port has no run-time mesh builder: a bounding volume hierarchy on a 67 MHz
ARM9 is work that belongs in the asset pipeline, so a level is baked offline and
the device only reads the result. This is that offline step.

It runs on a PC, so it computes in double and emits Q12 -- which is what makes
the whole off-device mesh design work: the tree, the welding and the edge
classification are decided at full precision and only the *result* is quantized.

Four things here are decisions rather than transcription:

1. **Everything derived from a vertex is derived from the quantized vertex.**
   Two derived things decide what is *in* the blob rather than only what its
   numbers are -- the degeneracy test (a triangle with real area in double can
   collapse to nothing at Q12) and the edge flags (a "flat" or "concave" bit
   derived from exact geometry can contradict the quantized triangle it labels,
   and the narrow phase trusts those bits to decide which contacts to discard;
   that is the ghost-collision failure mode, arriving from the baker).

2. **Spread the residual rather than pinning it to one vertex.** Here that is
   the welding: near-duplicate vertices are merged *after* quantization, so two
   vertices that land on the same lattice point become one rather than two
   coincident ones.

3. **Node bounds round outward.** ``b3AABB_Contains`` is an exact comparison
   with no tolerance, so a node bound rounded inward both fails the port's own
   consistency check and, worse, silently culls a triangle the node really does
   contain. Since the bounds here are min/max over already-quantized vertices
   they are exact, and this rule costs nothing -- but it is the rule, and
   computing bounds from the pre-quantization doubles would break it.

4. **The tree is split at the median** rather than by upstream's surface area
   heuristic. For a level of a few hundred triangles the traversal difference is
   not measurable on an ARM9, and it removes a cost function that would
   otherwise have to be justified.

# On being the second baker

``tests/box3d_host/mesh_bake.c`` bakes the same blob in C, and it is the one
every ``run_pair`` mesh case and every ``test_world`` mode already exercises.
This module is a **second implementation of a binary contract**, which is only
safe because something compares the two: ``tests/box3d_host/test_bake_diff.py``
bakes a corpus with both and asserts the bytes are identical.

So every routine below is a line-for-line port of its C original, and where a
faster formulation exists it is used only when it provably gives the same
answer -- see ``_weld`` and ``_identify_edges``, the two places the C is O(n^2).
Anything here that "looks like it could be tidier" is very likely load-bearing.

# Quantization rounds toward zero

``mesh_bake.c``'s header says vertices "round to nearest, not toward zero", but
``quantize()`` there is ``b3fFromDouble``, which is ``(int32_t)( x * 4096.0 )``
-- a C cast, truncating. The comment describes an intent the code does not
implement. This module copies the code, not the comment: changing it would alter
every blob ever baked and is a decision for ``source/box3d``, not for a tool.
``obj2dl.py``'s ``float_to_f32`` already truncates the same way.
"""

import math
import os
import struct

# ---------------------------------------------------------------------------
# The contract: constants and sizes, all restated from the port's headers
# ---------------------------------------------------------------------------

# include/box3d/types.h. Spells "NEAMESH" followed by a layout revision, and is
# deliberately not upstream's value: a float mesh loaded as a fixed-point one
# would be silently wrong rather than obviously wrong.
B3_MESH_VERSION = 0x4E45414D45534801

# include/box3d/b3fixed.h. Q19.12, which is also libnds' f32.
B3_F_ONE = 1 << 12

# source/box3d/mesh.h. The device walks the tree with a fixed stack and no
# recursion, so a tree it could not walk is rejected here rather than
# overflowing there.
B3_MESH_STACK_SIZE = 32

# source/box3d/mesh.h. An internal node stores its split axis in the same two
# bits, and an axis is 0, 1 or 2 -- so 3 is the one value left over.
B3_LEAF_NODE = 3

# include/box3d/constants.h, b3fFromInt(2000). The port's own statement of where
# Q12 stops having useful resolution for a world coordinate.
B3_HUGE = 2000.0

# include/box3d/types.h, b3MeshEdgeFlags. Edge N runs from vertex N to N+1.
CONCAVE_BITS = (0x01, 0x02, 0x04)
INVERSE_CONCAVE_BITS = (0x10, 0x20, 0x40)

# Device-mode sizes. b3f is a bare int32_t there; under the host harness's
# shadow-value builds b3Vec3 is much wider, which is why mesh_bake.c computes
# its offsets from sizeof() and why these numbers are only correct for the
# layout the ARM9 reads -- which is the only one a file on disk can carry.
SIZEOF_MESH_DATA = 88
SIZEOF_MESH_NODE = 32
SIZEOF_VEC3 = 12
SIZEOF_MESH_TRIANGLE = 12

# mesh_bake.c. At most four triangles in a leaf.
PD_TRIANGLES_PER_LEAF = 4

# The smallest cross-product length a triangle may have and still be kept.
#
# Upstream compares |cross|^2 against (0.01 * B3_LINEAR_SLOP^2)^2, a threshold
# from a world where the smallest representable number is 1e-38. It cannot be
# converted, only re-derived. The question a fixed-point port actually has to
# answer is "can this triangle produce a usable normal?", and the answer is
# bounded by the lattice: the smallest non-degenerate triangle on a Q12 grid has
# two edges one quantum apart, so |cross| is one quantum squared. This asks for
# four times that, so a triangle has to be clearly rather than marginally
# non-degenerate to survive.
PD_MIN_CROSS_LENGTH = 4.0 / (4096.0 * 4096.0)

# The .colmesh format, written by generate_colmesh() in obj2dl.py.
COLM_MAGIC = 0x4D4C4F43   # "COLM"
COLM_VERSION = 1
COLM_HEADER_BYTES = 16     # magic, version, triangleCount, pad
COLM_BOUNDS_BYTES = 24     # 6 x f32
COLM_TRIANGLE_BYTES = 48   # 9 x f32 vertex + 3 x f32 face normal


class B3MeshError(Exception):
    """A mesh that cannot be baked, with the reason an author can act on.

    ``pdBakeMesh`` returns one number -- zero -- for every refusal. Saying which
    one it was is the main thing this module offers over the C, because every
    refusal has a different fix.
    """


# ---------------------------------------------------------------------------
# Quantization
# ---------------------------------------------------------------------------

def _quantize(v):
    """One coordinate as a b3f, truncating toward zero.

    ``int()`` on a float truncates toward zero, which is what the C cast in
    ``b3fFromDouble`` does. The range check has no counterpart there -- an
    out-of-range cast is undefined behaviour in C and in practice wraps -- and
    it is here because a tool that silently wraps a coordinate is a tool that
    ships a level with a corner in the wrong place.
    """
    q = int(v * float(B3_F_ONE))
    if q < -0x80000000 or q > 0x7FFFFFFF:
        raise B3MeshError(
            f"coordinate {v} does not fit in Q19.12 (raw {q}). "
            f"Scale the mesh down; 1 unit should be about 1 metre.")
    return q


def _quantize_saturating(v):
    """As ``_quantize``, but clamped to the b3f range instead of refusing.

    Used for ``surfaceArea`` and nothing else. It is the one field in the blob
    that can legitimately exceed Q12 -- a coarse level of 100 large triangles
    reaches 1e8 length-squared while every *coordinate* stays inside B3_HUGE --
    and it is also the one field nothing reads. b3MeshData::surfaceArea's own
    comment says upstream computes it, stores it and then never looks at it
    either; it is kept because dropping a field from the middle of a blob layout
    buys 4 bytes and costs the diff against upstream.

    Refusing a whole level over an unread diagnostic would be the wrong trade.
    Note that the C is **undefined** here rather than saturating -- the cast in
    b3fFromDouble simply overflows -- so this is the one place b3mesh.py and
    mesh_bake.c can disagree, and it is a disagreement in Python's favour.
    test_bake_diff.py stays inside the range so the comparison never depends on
    it.
    """
    raw = v * float(B3_F_ONE)
    if raw > 0x7FFFFFFF:
        return 0x7FFFFFFF
    if raw < -0x80000000:
        return -0x80000000
    return int(raw)


# ---------------------------------------------------------------------------
# Small vector helpers, all in double over quantized coordinates
# ---------------------------------------------------------------------------

def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _len(a):
    return math.sqrt(_dot(a, a))


# ---------------------------------------------------------------------------
# Welding
# ---------------------------------------------------------------------------

def _weld(vertices):
    """Merge vertices that land on the same Q12 lattice point.

    Not a tolerance weld -- upstream's spatial hash and its ``weldTolerance``
    are a modelling convenience, and a level authored for a DS does not need
    one. What this does need to do is collapse vertices that quantization made
    identical, because two coincident vertices give an edge no shared-edge
    partner and the ghost filter then has nothing to work with.

    ``weldQuantized`` scans the vertices stored so far and takes the **first**
    with an identical raw triple, which is exactly what a dict recording
    first-seen gives -- in O(n) instead of O(n^2). The results are identical
    because both answer the same question in the same order.

    :return: ``(quantized, doubles, remap)``. ``doubles`` is the stored value
        read back, never the caller's coordinate: every derived quantity below
        reads it, which is decision 1.
    """
    seen = {}
    quantized = []
    doubles = []
    remap = [0] * len(vertices)

    for i, v in enumerate(vertices):
        q = (_quantize(v[0]), _quantize(v[1]), _quantize(v[2]))

        index = seen.get(q)
        if index is None:
            index = len(quantized)
            seen[q] = index
            quantized.append(q)
            doubles.append((q[0] / 4096.0, q[1] / 4096.0, q[2] / 4096.0))

        remap[i] = index

    return quantized, doubles, remap


def _is_degenerate(v1, v2, v3):
    return _len(_cross(_sub(v2, v1), _sub(v3, v1))) < PD_MIN_CROSS_LENGTH


# ---------------------------------------------------------------------------
# The BVH
# ---------------------------------------------------------------------------
#
# A primitive is a tuple (lower, upper, center, triangle). The list is
# partitioned in place, exactly as s_primitives is, because the leaf offsets
# refer to positions in it.

_P_LOWER, _P_UPPER, _P_CENTER, _P_TRIANGLE = range(4)


def _primitive_bounds(prims, first, count):
    lower = list(prims[first][_P_LOWER])
    upper = list(prims[first][_P_UPPER])

    for i in range(first + 1, first + count):
        p = prims[i]
        for axis in range(3):
            if p[_P_LOWER][axis] < lower[axis]:
                lower[axis] = p[_P_LOWER][axis]
            if p[_P_UPPER][axis] > upper[axis]:
                upper[axis] = p[_P_UPPER][axis]

    return lower, upper


def _split_median(prims, first, count):
    """Partition [first, first+count) about the median of the widest axis.

    :return: the size of the left half. Never 0 and never ``count``.
    """
    lower = list(prims[first][_P_CENTER])
    upper = list(prims[first][_P_CENTER])

    for i in range(first + 1, first + count):
        c = prims[i][_P_CENTER]
        for axis in range(3):
            if c[axis] < lower[axis]:
                lower[axis] = c[axis]
            if c[axis] > upper[axis]:
                upper[axis] = c[axis]

    axis = 0
    widest = upper[0] - lower[0]
    if upper[1] - lower[1] > widest:
        axis = 1
        widest = upper[1] - lower[1]
    if upper[2] - lower[2] > widest:
        axis = 2

    pivot = 0.5 * (lower[axis] + upper[axis])

    # Hoare partition, as upstream's median split does.
    i1 = first
    i2 = first + count
    while i1 < i2:
        while i1 < i2 and prims[i1][_P_CENTER][axis] < pivot:
            i1 += 1

        while i1 < i2 and prims[i2 - 1][_P_CENTER][axis] >= pivot:
            i2 -= 1

        if i1 < i2:
            prims[i1], prims[i2 - 1] = prims[i2 - 1], prims[i1]
            i1 += 1
            i2 -= 1

    left_count = i1 - first

    # Every centroid on one side of the pivot -- coincident centroids, or a
    # degenerate spread. Halve it so the recursion still terminates.
    if left_count == 0 or left_count == count:
        left_count = count // 2

    return left_count


class _Node:
    """One b3MeshNode under construction.

    A class rather than a tuple because the builder reserves a node's slot
    before it knows the node's contents -- pre-order emission is what lets the
    left child be implicit, and that requires taking the index first.
    """

    __slots__ = ("lower", "upper", "data", "triangle_offset")

    def __init__(self):
        self.lower = (0, 0, 0)
        self.upper = (0, 0, 0)
        self.data = 0
        self.triangle_offset = 0


def _store_bounds(node, lower, upper):
    # Decision 3. The values are min/max over vertices that were already
    # quantized, so _quantize here is exact and rounds nothing -- but going
    # through the doubles at all is what would break it, so the invariant is
    # stated rather than assumed.
    node.lower = (_quantize(lower[0]), _quantize(lower[1]), _quantize(lower[2]))
    node.upper = (_quantize(upper[0]), _quantize(upper[1]), _quantize(upper[2]))


def _build_recursive(nodes, prims, first, count, depth, state, max_nodes):
    """:return: the node index, or -1 if the node budget or height bound was hit."""
    if len(nodes) >= max_nodes:
        return -1

    # The device walks this with a fixed stack and no recursion.
    if depth >= B3_MESH_STACK_SIZE:
        return -1

    if depth + 1 > state["height"]:
        state["height"] = depth + 1

    index = len(nodes)
    nodes.append(_Node())

    lower, upper = _primitive_bounds(prims, first, count)

    if count <= PD_TRIANGLES_PER_LEAF:
        node = nodes[index]
        _store_bounds(node, lower, upper)
        node.data = B3_LEAF_NODE | (count << 2)
        node.triangle_offset = first
        return index

    left_count = _split_median(prims, first, count)

    left_index = _build_recursive(nodes, prims, first, left_count,
                                  depth + 1, state, max_nodes)
    if left_index < 0:
        return -1

    right_index = _build_recursive(nodes, prims, first + left_count,
                                   count - left_count, depth + 1, state, max_nodes)
    if right_index < 0:
        return -1

    # Pre-order emission is what lets the left child be implicit.
    if left_index != index + 1 or right_index <= index + 1:
        return -1

    node = nodes[index]
    _store_bounds(node, lower, upper)

    # Split axis 0 in the low two bits, right-child offset above it. The offset
    # is relative to this node, which is what keeps it inside 30 bits.
    node.data = 0 | ((right_index - index) << 2)

    # Zero for an internal node, so the blob's bytes are deterministic and its
    # hash is a function of the geometry alone.
    node.triangle_offset = 0

    return index


def _sort_triangles_depth_first(nodes, prims, triangles):
    """Reorder triangles into depth-first leaf order, rewriting each leaf offset.

    This is the invariant ``b3QueryMesh``'s ascending-index guarantee rests on,
    and nothing downstream asserts it -- the mesh narrow phase just quietly
    stops matching its per-triangle cache. So it is done here and checked here.
    """
    sorted_triangles = []

    stack = [0]
    while stack:
        index = stack.pop()
        node = nodes[index]

        if (node.data & 3) != B3_LEAF_NODE:
            if len(stack) + 2 > B3_MESH_STACK_SIZE:
                return None

            # Right pushed first so left pops first: the same order the
            # device's descent takes.
            stack.append(index + (node.data >> 2))
            stack.append(index + 1)
            continue

        triangle_count = node.data >> 2
        triangle_offset = node.triangle_offset

        for i in range(triangle_count):
            sorted_triangles.append(triangles[prims[triangle_offset + i][_P_TRIANGLE]])

        node.triangle_offset = len(sorted_triangles) - triangle_count

    if len(sorted_triangles) != len(triangles):
        return None

    return sorted_triangles


# ---------------------------------------------------------------------------
# Shared-edge classification
# ---------------------------------------------------------------------------

def _identify_edges(triangles, doubles):
    """Set the concave / inverse-concave bits per triangle.

    These let the narrow phase tell an interior edge from a silhouette, and are
    what stops a body catching on the seams of a flat floor.

    Run on the quantized vertices (decision 1), in double. This is the single
    biggest reason baking off-device is worth doing: ``signedVolume`` is an
    unnormalized scalar triple product, so it is cubic in the coordinates and
    only its sign is ever used -- exactly the shape of expression that is
    awkward in Q12 and free here.

    The C pairs each half-edge with the **first** later one sharing its vertex
    pair and then stops, but still visits every half-edge -- so three coincident
    half-edges produce the pairs (e0, e1) and (e1, e2), not (e0, e1), (e0, e2)
    and (e1, e2). Grouping by vertex pair in ascending edge order and pairing
    consecutive members reproduces that exactly, in O(n) rather than O(n^2).
    Flags are OR-accumulated, so the order the pairs are visited in does not
    matter.
    """
    count = len(triangles)
    flags = [0] * count
    normals = [None] * count

    # Global edge index is 3 * triangle + edge, ascending, as in the C.
    groups = {}

    for i, tri in enumerate(triangles):
        v1 = doubles[tri[0]]
        v2 = doubles[tri[1]]
        v3 = doubles[tri[2]]

        n = _cross(_sub(v2, v1), _sub(v3, v1))
        length = _len(n)
        if length > 0.0:
            normals[i] = (n[0] / length, n[1] / length, n[2] / length)
        else:
            normals[i] = (0.0, 1.0, 0.0)

        for e in range(3):
            a = tri[e]
            b = tri[(e + 1) % 3]
            key = (a, b) if a < b else (b, a)
            groups.setdefault(key, []).append((i, e))

    cos5_deg = 0.9962

    for group in groups.values():
        for p in range(len(group) - 1):
            a_tri, a_edge = group[p]
            b_tri, b_edge = group[p + 1]

            # The opposite vertex of the neighbouring triangle: edge 0 is
            # v1->v2 so v3 is opposite, edge 1 is v2->v3 so v1 is, edge 2 is
            # v3->v1 so v2 is.
            opposite = triangles[b_tri][(b_edge + 2) % 3]

            self_tri = triangles[a_tri]
            v1 = doubles[self_tri[0]]
            v2 = doubles[self_tri[1]]
            v3 = doubles[self_tri[2]]
            p_vertex = doubles[opposite]

            # signedVolume = dot( cross( v2-v1, v3-v1 ), p-v1 ). Negative when
            # the neighbour lies below this triangle's plane.
            n = _cross(_sub(v2, v1), _sub(v3, v1))
            signed_volume = _dot(n, _sub(p_vertex, v1))

            cos_angle = _dot(normals[a_tri], normals[b_tri])

            if signed_volume > 0.0 or cos_angle > cos5_deg:
                flags[a_tri] |= CONCAVE_BITS[a_edge]
                flags[b_tri] |= CONCAVE_BITS[b_edge]

            if signed_volume < 0.0 or cos_angle > cos5_deg:
                flags[a_tri] |= INVERSE_CONCAVE_BITS[a_edge]
                flags[b_tri] |= INVERSE_CONCAVE_BITS[b_edge]

    return flags


# ---------------------------------------------------------------------------
# Layout
# ---------------------------------------------------------------------------

def _align_to(cursor, alignment):
    """Round up to a multiple of `alignment`.

    Ceil division, as ``alignTo`` is -- so a 12-byte "alignment" means a
    multiple of 12, which is not a power of two. b3Vec3 is 12 bytes and the
    layout genuinely uses that.
    """
    return ((cursor + alignment - 1) // alignment) * alignment


def _mesh_layout(node_count, vertex_count, triangle_count):
    cursor = SIZEOF_MESH_DATA

    cursor = _align_to(cursor, SIZEOF_MESH_NODE)
    node_offset = cursor
    cursor += node_count * SIZEOF_MESH_NODE

    cursor = _align_to(cursor, SIZEOF_VEC3)
    vertex_offset = cursor
    cursor += vertex_count * SIZEOF_VEC3

    cursor = _align_to(cursor, SIZEOF_MESH_TRIANGLE)
    triangle_offset = cursor
    cursor += triangle_count * SIZEOF_MESH_TRIANGLE

    material_offset = cursor
    cursor += triangle_count

    flags_offset = cursor
    cursor += triangle_count

    # Round the total up so an array of meshes stays aligned.
    byte_count = (cursor + 7) & ~7

    return {
        "nodeOffset": node_offset,
        "vertexOffset": vertex_offset,
        "triangleOffset": triangle_offset,
        "materialOffset": material_offset,
        "flagsOffset": flags_offset,
        "byteCount": byte_count,
    }


# ---------------------------------------------------------------------------
# The bake
# ---------------------------------------------------------------------------

def bake(vertices, indices, scale=1.0):
    """Bake a triangle soup into a ``.b3mesh`` blob.

    :param vertices: sequence of (x, y, z) in doubles.
    :param indices: flat sequence of 3 vertex indices per triangle.
    :param scale: every coordinate is divided by this on the way in. A level
        authored at render scale is the wrong size for a solver whose
        tolerances are absolute; 1 unit should be about 1 metre.
    :return: ``(blob, info)`` -- the bytes, and what went into them.
    :raises B3MeshError: with the reason, for anything that cannot be baked.
    """
    if scale <= 0.0:
        raise B3MeshError(f"scale must be positive, got {scale}")

    triangle_count_in = len(indices) // 3

    if len(vertices) < 3 or triangle_count_in < 1:
        raise B3MeshError(
            f"a mesh needs at least 3 vertices and 1 triangle, "
            f"got {len(vertices)} and {triangle_count_in}")

    if scale != 1.0:
        vertices = [(v[0] / scale, v[1] / scale, v[2] / scale) for v in vertices]

    # Quantize and weld first, so that everything after this reads stored
    # coordinates only.
    quantized, doubles, remap = _weld(vertices)

    if len(quantized) < 3:
        raise B3MeshError(
            f"{len(vertices)} vertices collapsed to {len(quantized)} on the Q12 "
            f"lattice, which is not a surface. The mesh is smaller than a "
            f"quantum ({1.0 / 4096.0} units) across.")

    # Q12 range. B3_HUGE is the port's own statement of where a world
    # coordinate stops having useful resolution.
    for d in doubles:
        for axis in range(3):
            if abs(d[axis]) > B3_HUGE:
                raise B3MeshError(
                    f"a coordinate reaches {d[axis]:.1f} units, past the world "
                    f"bound of {B3_HUGE:.0f}. Solver tolerances are absolute, "
                    f"so a level this large loses precision where it is "
                    f"furthest from the origin. Rescale it.")

    # Keep the triangles that still have area on the lattice.
    triangles = []
    degenerate_count = 0
    surface_area = 0.0

    for i in range(triangle_count_in):
        i1 = remap[indices[3 * i + 0]]
        i2 = remap[indices[3 * i + 1]]
        i3 = remap[indices[3 * i + 2]]

        if i1 == i2 or i1 == i3 or i2 == i3:
            degenerate_count += 1
            continue

        v1 = doubles[i1]
        v2 = doubles[i2]
        v3 = doubles[i3]

        if _is_degenerate(v1, v2, v3):
            degenerate_count += 1
            continue

        surface_area += 0.5 * _len(_cross(_sub(v2, v1), _sub(v3, v1)))

        triangles.append((i1, i2, i3))

    if not triangles:
        raise B3MeshError(
            f"every one of the {triangle_count_in} triangles was degenerate "
            f"once quantized. The mesh is too small or too thin for Q12.")

    # Primitives, whose bounds are min/max over the stored vertices and are
    # therefore exact on the lattice.
    prims = []
    for i, tri in enumerate(triangles):
        v1 = doubles[tri[0]]
        v2 = doubles[tri[1]]
        v3 = doubles[tri[2]]

        lower = []
        upper = []
        center = []
        for axis in range(3):
            lo = min(v1[axis], v2[axis], v3[axis])
            hi = max(v1[axis], v2[axis], v3[axis])
            lower.append(lo)
            upper.append(hi)
            center.append(0.5 * (lo + hi))

        prims.append((tuple(lower), tuple(upper), tuple(center), i))

    # An upper bound on nodes, matching PD_BAKE_MAX_NODES. With at most four
    # triangles per leaf the real count is nowhere near it; it exists so a
    # pathological split cannot run away.
    max_nodes = 2 * len(triangles)

    nodes = []
    state = {"height": 0}
    if _build_recursive(nodes, prims, 0, len(triangles), 0, state, max_nodes) != 0:
        raise B3MeshError(
            f"the bounding volume hierarchy could not be built from "
            f"{len(triangles)} triangles: it went deeper than "
            f"{B3_MESH_STACK_SIZE}, which is as far as the device can walk. "
            f"Split the level, or make its triangles more evenly sized.")

    sorted_triangles = _sort_triangles_depth_first(nodes, prims, triangles)
    if sorted_triangles is None:
        raise B3MeshError("the depth-first triangle sort did not visit every leaf")

    triangles = sorted_triangles
    flags = _identify_edges(triangles, doubles)

    layout = _mesh_layout(len(nodes), len(quantized), len(triangles))

    # Zero everything, padding included: the hash is over raw bytes.
    blob = bytearray(layout["byteCount"])

    # Bounds from the stored vertices, so they contain what is in the blob
    # rather than what was asked for.
    lower_bound = [quantized[0][axis] for axis in range(3)]
    upper_bound = [quantized[0][axis] for axis in range(3)]
    for q in quantized[1:]:
        for axis in range(3):
            if q[axis] < lower_bound[axis]:
                lower_bound[axis] = q[axis]
            if q[axis] > upper_bound[axis]:
                upper_bound[axis] = q[axis]

    # version, byteCount, hash, bounds.lower, bounds.upper, surfaceArea,
    # treeHeight, degenerateCount -- the first 52 bytes of b3MeshData. Spelled
    # by concatenation because a miscounted repeat in one long format string is
    # the kind of mistake that produces a plausible-looking wrong blob.
    struct.pack_into(
        "<" + "Q" "i" "I" + "3i" + "3i" + "i" "i" "i", blob, 0,
        B3_MESH_VERSION,
        layout["byteCount"],
        0,                                  # hash, filled in below
        lower_bound[0], lower_bound[1], lower_bound[2],
        upper_bound[0], upper_bound[1], upper_bound[2],
        _quantize_saturating(surface_area),
        state["height"],
        degenerate_count)

    struct.pack_into(
        "<9i", blob, 52,
        layout["nodeOffset"], len(nodes),
        layout["vertexOffset"], len(quantized),
        layout["triangleOffset"], len(triangles),
        layout["materialOffset"], 1,
        layout["flagsOffset"])

    cursor = layout["nodeOffset"]
    for node in nodes:
        struct.pack_into("<3iI3iI", blob, cursor,
                         node.lower[0], node.lower[1], node.lower[2],
                         node.data,
                         node.upper[0], node.upper[1], node.upper[2],
                         node.triangle_offset)
        cursor += SIZEOF_MESH_NODE

    cursor = layout["vertexOffset"]
    for q in quantized:
        struct.pack_into("<3i", blob, cursor, q[0], q[1], q[2])
        cursor += SIZEOF_VEC3

    cursor = layout["triangleOffset"]
    for tri in triangles:
        struct.pack_into("<3i", blob, cursor, tri[0], tri[1], tri[2])
        cursor += SIZEOF_MESH_TRIANGLE

    # The material indices are left zero -- one material, index 0 for every
    # triangle -- which the zero-fill above already did.

    cursor = layout["flagsOffset"]
    for f in flags:
        blob[cursor] = f
        cursor += 1

    # djb2 over the whole blob with the hash field zero, which it already is.
    # Nothing on device verifies this -- see the note at b3MeshData::hash -- but
    # a .b3mesh is a file, and a file can go stale.
    h = 5381
    for b in blob:
        h = (h * 33 + b) & 0xFFFFFFFF
    h = h if h != 0 else 1
    struct.pack_into("<I", blob, 12, h)

    blob = bytes(blob)

    # The port's own validator has the last word, ported below. A mesh whose
    # tree no longer contains its triangles after quantization is rejected here
    # rather than losing contacts at run time.
    problem = validate(blob)
    if problem is not None:
        raise B3MeshError(f"the baked blob did not validate: {problem}")

    info = {
        "inputTriangles": triangle_count_in,
        "inputVertices": len(vertices),
        "triangleCount": len(triangles),
        "vertexCount": len(quantized),
        "degenerateCount": degenerate_count,
        "nodeCount": len(nodes),
        "treeHeight": state["height"],
        "surfaceArea": surface_area,
        "byteCount": layout["byteCount"],
        "hash": h,
        "lower": tuple(b / 4096.0 for b in lower_bound),
        "upper": tuple(b / 4096.0 for b in upper_bound),
        "scale": scale,
    }

    return blob, info


# ---------------------------------------------------------------------------
# Validation -- the port's b3IsValidMesh, ported
# ---------------------------------------------------------------------------

def validate(blob):
    """Check a blob the way ``b3IsValidMesh`` does.

    Version, offsets, and that every node's bounds contain what the node points
    at. That last property is the one the traversal rests on: a node whose
    bounds are too small does not corrupt anything -- it silently stops
    reporting triangles that are really there, which is a bug that presents as a
    body falling through a floor an hour later.

    :return: ``None`` if the blob is valid, else a string saying what is wrong.
    """
    if len(blob) < SIZEOF_MESH_DATA:
        return "shorter than the header"

    (version, byte_count, _hash) = struct.unpack_from("<QiI", blob, 0)
    if version != B3_MESH_VERSION:
        return f"version is {version:#x}, expected {B3_MESH_VERSION:#x}"

    (surface_area, tree_height, degenerate_count) = struct.unpack_from("<3i", blob, 40)
    (node_offset, node_count, vertex_offset, vertex_count,
     triangle_offset, triangle_count, material_offset, material_count,
     flags_offset) = struct.unpack_from("<9i", blob, 52)

    if byte_count < SIZEOF_MESH_DATA:
        return f"byteCount {byte_count} is shorter than the header"

    if byte_count != len(blob):
        return f"byteCount is {byte_count}, blob is {len(blob)} bytes"

    if node_count <= 0 or vertex_count < 3 or triangle_count <= 0:
        return (f"empty: {node_count} nodes, {vertex_count} vertices, "
                f"{triangle_count} triangles")

    if tree_height <= 0 or tree_height > B3_MESH_STACK_SIZE:
        return f"tree height {tree_height} is outside 1..{B3_MESH_STACK_SIZE}"

    # Every array the traversal reads has to be present. The flags and the
    # material indices are optional -- a mesh with no edge classification
    # collides, it just ghosts on internal edges.
    if node_offset == 0 or vertex_offset == 0 or triangle_offset == 0:
        return "a required array is absent"

    if (node_offset < SIZEOF_MESH_DATA or vertex_offset < SIZEOF_MESH_DATA
            or triangle_offset < SIZEOF_MESH_DATA):
        return "an array overlaps the header"

    if (node_offset + node_count * SIZEOF_MESH_NODE > byte_count
            or vertex_offset + vertex_count * SIZEOF_VEC3 > byte_count
            or triangle_offset + triangle_count * SIZEOF_MESH_TRIANGLE > byte_count):
        return "an array runs past the end of the blob"

    if flags_offset != 0 and flags_offset + triangle_count > byte_count:
        return "the flags array runs past the end of the blob"

    if material_offset != 0 and material_offset + triangle_count > byte_count:
        return "the material array runs past the end of the blob"

    def node_at(index):
        base = node_offset + index * SIZEOF_MESH_NODE
        lx, ly, lz, data, ux, uy, uz, tri = struct.unpack_from("<3iI3iI", blob, base)
        return (lx, ly, lz), (ux, uy, uz), data, tri

    def vertex_at(index):
        return struct.unpack_from("<3i", blob, vertex_offset + index * SIZEOF_VEC3)

    def contains(outer_lower, outer_upper, inner_lower, inner_upper):
        return all(outer_lower[a] <= inner_lower[a] and inner_upper[a] <= outer_upper[a]
                   for a in range(3))

    stack = [0]
    guard = 0

    while stack:
        # A blob that arrived as bytes may describe a cycle. Every node is meant
        # to be visited once, so more visits than there are nodes means the tree
        # is not one.
        guard += 1
        if guard > node_count:
            return "the node graph is not a tree"

        index = stack.pop()
        if index < 0 or index >= node_count:
            return f"node index {index} is out of range"

        lower, upper, data, triangle_start = node_at(index)

        if (data & 3) != B3_LEAF_NODE:
            if len(stack) + 2 > B3_MESH_STACK_SIZE:
                # Taller than the traversal stack, so b3QueryMesh could not walk
                # it safely even though it is otherwise well formed.
                return "the tree is taller than the traversal stack"

            child1 = index + 1
            child2 = index + (data >> 2)

            if child2 <= child1 or child2 >= node_count:
                return f"node {index} has a bad right-child offset"

            for child in (child1, child2):
                c_lower, c_upper, _, _ = node_at(child)
                if not contains(lower, upper, c_lower, c_upper):
                    return f"node {index} does not contain child {child}"

            stack.append(child2)
            stack.append(child1)
            continue

        leaf_count = data >> 2

        if leaf_count <= 0 or triangle_start < 0 or triangle_start + leaf_count > triangle_count:
            return f"leaf {index} names triangles outside the array"

        for i in range(leaf_count):
            base = triangle_offset + (triangle_start + i) * SIZEOF_MESH_TRIANGLE
            i1, i2, i3 = struct.unpack_from("<3i", blob, base)

            if not all(0 <= v < vertex_count for v in (i1, i2, i3)):
                return f"triangle {triangle_start + i} indexes a vertex that does not exist"

            v1 = vertex_at(i1)
            v2 = vertex_at(i2)
            v3 = vertex_at(i3)

            t_lower = tuple(min(v1[a], v2[a], v3[a]) for a in range(3))
            t_upper = tuple(max(v1[a], v2[a], v3[a]) for a in range(3))

            if not contains(lower, upper, t_lower, t_upper):
                return (f"leaf {index} does not contain triangle "
                        f"{triangle_start + i}")

    return None


# ---------------------------------------------------------------------------
# .colmesh input
# ---------------------------------------------------------------------------

def bake_colmesh(path, scale=1.0):
    """Bake a ``.colmesh`` written by ``obj2dl --collision``.

    A .colmesh is a non-indexed triangle soup -- 9 f32 vertex coordinates plus a
    face normal per triangle, no shared vertex array. The baker keys shared
    edges off vertex *indices*, so without indices every edge would be a free
    rim and the ghost filter would have nothing to work with. The trick is that
    the baker already welds on the Q12 lattice, so the right thing to do is hand
    it 3*triangleCount vertices with sequential indices and let the weld
    collapse them -- one weld rather than two that can disagree.

    This is the line-for-line counterpart of ``loadColMesh`` in
    ``tests/box3d_host/bake_ref.c``, and ``test_bake_diff.py`` depends on the
    two staying that way.
    """
    with open(path, "rb") as f:
        data = f.read()

    if len(data) < COLM_HEADER_BYTES + COLM_BOUNDS_BYTES:
        raise B3MeshError(f"{path}: too short to be a .colmesh")

    magic, version, triangle_count, _pad = struct.unpack_from("<IIII", data, 0)

    if magic != COLM_MAGIC:
        raise B3MeshError(
            f"{path}: not a .colmesh (magic {magic:#010x}, expected {COLM_MAGIC:#010x})")

    if version != COLM_VERSION:
        raise B3MeshError(
            f"{path}: .colmesh version {version}, this reads {COLM_VERSION}")

    expected = COLM_HEADER_BYTES + COLM_BOUNDS_BYTES + triangle_count * COLM_TRIANGLE_BYTES
    if len(data) != expected:
        raise B3MeshError(
            f"{path}: claims {triangle_count} triangles ({expected} bytes) "
            f"but the file is {len(data)}")

    if triangle_count < 1:
        raise B3MeshError(f"{path}: no triangles")

    vertices = []
    indices = []

    base = COLM_HEADER_BYTES + COLM_BOUNDS_BYTES
    for i in range(triangle_count):
        for corner in range(3):
            # The face normal at the end of each triangle record is skipped: the
            # port derives normals from the stored vertices, and a normal read
            # from the file could disagree with the quantized triangle it
            # labels.
            offset = base + i * COLM_TRIANGLE_BYTES + corner * 12
            x, y, z = struct.unpack_from("<3i", data, offset)
            vertices.append((x / 4096.0, y / 4096.0, z / 4096.0))
            indices.append(len(vertices) - 1)

    return bake(vertices, indices, scale)


# ---------------------------------------------------------------------------
# Reporting
# ---------------------------------------------------------------------------

def report(info, prefix="  "):
    """Print everything worth knowing about a baked level.

    Read back out of the bake rather than assembled from the inputs: these are
    the numbers the device will act on. The two an author can act on -- the
    world bound and the weld -- are checked rather than merely printed.
    """
    lines = []

    dropped = info["degenerateCount"]
    line = f"{prefix}triangles      {info['inputTriangles']} in"
    if dropped > 0:
        line += f", {dropped} dropped as degenerate"
    line += f", {info['triangleCount']} kept"
    lines.append(line)

    corners = 3 * info["inputTriangles"]
    line = f"{prefix}vertices       {info['vertexCount']} welded from {corners} corners"
    if info["vertexCount"] == corners:
        line += "  <-- NOTHING WELDED"
    lines.append(line)

    lines.append(f"{prefix}bvh            {info['nodeCount']} nodes, "
                 f"height {info['treeHeight']} of {B3_MESH_STACK_SIZE}")

    lo = info["lower"]
    hi = info["upper"]
    lines.append(f"{prefix}bounds         ({lo[0]:.3f} {lo[1]:.3f} {lo[2]:.3f}) .. "
                 f"({hi[0]:.3f} {hi[1]:.3f} {hi[2]:.3f})")

    extent = f"{hi[0] - lo[0]:.3f} x {hi[1] - lo[1]:.3f} x {hi[2] - lo[2]:.3f} units"
    if info["scale"] != 1.0:
        extent += f" (at scale {info['scale']:g})"
    lines.append(f"{prefix}extent         {extent}")

    lines.append(f"{prefix}surface area   {info['surfaceArea']:.3f}")
    lines.append(f"{prefix}size           {info['byteCount']} bytes, "
                 f"hash {info['hash']:08x}")

    # A mesh can legitimately reach far from the origin if the game never puts a
    # body at its far corner, so this warns and continues.
    worst = max(abs(v) for v in lo + hi)
    if worst > B3_HUGE:
        lines.append("")
        lines.append(f"{prefix}WARNING: a coordinate reaches {worst:.1f} units, past the "
                     f"practical world bound of {B3_HUGE:.1f}.")
        lines.append(f"{prefix}         Solver tolerances are absolute, so a level this "
                     f"large loses precision where")
        lines.append(f"{prefix}         it is furthest from the origin. Rescale it.")

    if info["vertexCount"] == corners and info["inputTriangles"] > 1:
        lines.append("")
        lines.append(f"{prefix}WARNING: no two corners merged, so every edge in this mesh "
                     f"is a free rim and the")
        lines.append(f"{prefix}         ghost filter cannot suppress internal-edge "
                     f"collisions. A body sliding")
        lines.append(f"{prefix}         across it will catch on the seams. Check that the "
                     f"source mesh shares")
        lines.append(f"{prefix}         its vertices.")

    print("\n".join(lines))


# ---------------------------------------------------------------------------
# C array output
# ---------------------------------------------------------------------------

def _symbol_from_name(name):
    """Turn a path into an identifier: basename, non-identifier chars to '_'."""
    base = os.path.basename(name)

    out = "".join(c if (c.isascii() and (c.isalnum() or c == "_")) else "_"
                  for c in base)

    if not out:
        out = "_"

    # A leading digit would not be an identifier.
    if out[0].isdigit():
        out = "_" + out

    return out


def emit_c(name, blob):
    """Write ``NAME_b3mesh.c`` and ``NAME_b3mesh.h``.

    The naming follows BlocksDS's bin2c -- ``cube.bin`` becomes ``cube_bin`` and
    ``cube_bin_size`` -- so a .c generated here drops into a project's
    ``source/`` directory and reads like anything else in its build.

    What it does *not* follow is bin2c's ``aligned(4)``, and that is the whole
    reason this exists. ``b3MeshData`` opens with a ``uint64_t``, so the
    compiler may read it with LDRD, which faults on a 4-aligned address on
    ARMv5TE. A .b3mesh dropped in a project's ``data/`` directory is therefore a
    fault waiting to happen; this array is ``aligned(8)``.

    :param name: output path prefix. The symbol comes from its basename.
    :return: the two paths written.
    """
    symbol = _symbol_from_name(name)
    size = len(blob)

    header_path = f"{name}_b3mesh.h"
    with open(header_path, "w") as f:
        f.write("// Generated by tools/obj2dl/obj2dl.py --collision-b3-c. Do not edit.\n\n")
        f.write("#pragma once\n\n")
        f.write("#include <stdint.h>\n\n")
        f.write(f"#define {symbol}_b3mesh_size ({size})\n")
        f.write(f"extern const uint8_t {symbol}_b3mesh[{size}];\n")

    source_path = f"{name}_b3mesh.c"
    with open(source_path, "w") as f:
        f.write("// Generated by tools/obj2dl/obj2dl.py --collision-b3-c. Do not edit.\n")
        f.write("//\n")
        f.write("// aligned(8), not bin2c's aligned(4): b3MeshData opens with a uint64_t and\n")
        f.write("// the ARM9 reads one with LDRD, which requires an 8-byte-aligned address.\n\n")
        f.write("#include <stdint.h>\n\n")
        f.write(f"const uint8_t {symbol}_b3mesh[{size}] __attribute__((aligned(8))) =\n{{\n")

        for i in range(0, size, 12):
            row = blob[i:i + 12]
            f.write("    " + " ".join(f"0x{b:02X}," for b in row) + "\n")

        f.write("};\n")

    return header_path, source_path


# ---------------------------------------------------------------------------
# Command line -- for baking a .colmesh that is already on disk
# ---------------------------------------------------------------------------
#
# The normal path is `obj2dl.py --collision-b3`, which goes straight from the
# OBJ. This exists for a .colmesh produced earlier, and for test_bake_diff.py.

def main():
    import argparse
    import sys

    parser = argparse.ArgumentParser(
        description="Bake a .colmesh into a Box3D .b3mesh blob.")
    parser.add_argument("input", help="the .colmesh to read")
    parser.add_argument("--output", "-o", required=True, help="the .b3mesh to write")
    parser.add_argument("--scale", "-s", type=float, default=1.0,
                        help="divide every coordinate by this (default 1). "
                             "1 unit should be about 1 metre")
    parser.add_argument("--c", dest="c_name", default=None, metavar="NAME",
                        help="also write NAME_b3mesh.c / .h, an 8-byte-aligned "
                             "C array to compile into a ROM")

    args = parser.parse_args()

    try:
        blob, info = bake_colmesh(args.input, args.scale)
    except B3MeshError as e:
        print(f"ERROR: {args.input}: {e}", file=sys.stderr)
        return 1

    print(f"{args.input}")
    report(info)

    with open(args.output, "wb") as f:
        f.write(blob)
    print(f"  wrote          {args.output}")

    if args.c_name is not None:
        for path in emit_c(args.c_name, blob):
            print(f"  wrote          {path}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
