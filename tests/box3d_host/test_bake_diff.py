#!/usr/bin/env python3
# SPDX-License-Identifier: CC0-1.0
#
# SPDX-FileContributor: Warioware64, 2026

"""Bake a corpus with both bakers and assert the bytes are identical.

``tools/obj2dl/b3mesh.py`` is a second implementation of the ``.b3mesh`` binary
contract. The first is ``mesh_bake.c``, which every ``run_pair`` mesh case and
every ``test_world`` mode already exercises, and which ``bake_ref`` puts behind
a command line for exactly this test.

A second implementation is only safe if something compares the two, and
comparing *behaviour* is not enough here: the failure mode this guards against
is not a crash but a subtly wrong edge flag, which presents months later as a
body catching on the seams of a flat floor. So the comparison is **byte for
byte** over the whole blob, which pins the layout, the tree, the weld, the
degeneracy test, the edge classification and the hash all at once.

Both bakers are fed the same ``.colmesh``, so the comparison isolates the bake
from the OBJ parsing on either side.

Run it with::

    make -f Makefile.host bake-diff
"""

import os
import struct
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, "..", ".."))
ASSETS = os.path.join(ROOT, "examples", "assets")
OBJ2DL = os.path.join(ROOT, "tools", "obj2dl", "obj2dl.py")
BAKE_REF = os.path.join(HERE, "bake_ref")

sys.path.insert(0, os.path.join(ROOT, "tools", "obj2dl"))
import b3mesh  # noqa: E402


# ---------------------------------------------------------------------------
# Writing a .colmesh directly, for the cases no OBJ in the tree produces
# ---------------------------------------------------------------------------

def make_colmesh(triangles):
    """Build a .colmesh image from a list of 3-tuples of (x, y, z) floats.

    The same layout ``generate_colmesh`` writes. The face normals are filled
    with zeros: neither baker reads them, and a test that supplied plausible
    ones would be asserting that fact rather than testing it.
    """
    out = bytearray()
    out += struct.pack("<IIII", b3mesh.COLM_MAGIC, b3mesh.COLM_VERSION,
                       len(triangles), 0)

    def f32(value):
        return struct.pack("<i", int(value * 4096.0))

    lo = [float("inf")] * 3
    hi = [float("-inf")] * 3
    for tri in triangles:
        for v in tri:
            for axis in range(3):
                lo[axis] = min(lo[axis], v[axis])
                hi[axis] = max(hi[axis], v[axis])

    for axis in range(3):
        out += f32(lo[axis])
    for axis in range(3):
        out += f32(hi[axis])

    for tri in triangles:
        for v in tri:
            for axis in range(3):
                out += f32(v[axis])
        out += f32(0.0) * 3

    return bytes(out)


# ---------------------------------------------------------------------------
# The corpus
# ---------------------------------------------------------------------------
#
# Chosen so that every branch of the bake is reached by something. A case that
# only ever exercises the happy path would pass while the interesting code was
# wrong.

def grid_floor(cells, size, y=0.0):
    """A subdivided plane. Interior edges, shared vertices, coincident centroids."""
    triangles = []
    step = size / cells
    origin = -0.5 * size

    for i in range(cells):
        for k in range(cells):
            x0 = origin + i * step
            x1 = x0 + step
            z0 = origin + k * step
            z1 = z0 + step

            triangles.append(((x0, y, z0), (x0, y, z1), (x1, y, z1)))
            triangles.append(((x0, y, z0), (x1, y, z1), (x1, y, z0)))

    return triangles


def corpus():
    """Yield (name, colmesh_bytes, scale) for every case."""
    # Real geometry, through the real OBJ path.
    #
    # The obj2dl scale is not the bake scale: a display list stores positions as
    # v16, which tops out at 8 units, so the teapot has to be shrunk before
    # obj2dl will emit one at all. The bake scale is the second number, and the
    # 8.0 case is there because --collision-b3-scale divides on the way into the
    # bake and that division must land the same way in both bakers.
    for obj, obj_scale, bake_scale in (("cube.obj", 1.0, 1.0),
                                       ("sphere.obj", 1.0, 1.0),
                                       ("teapot.obj", 0.3, 1.0),
                                       ("teapot.obj", 0.3, 8.0),
                                       ("teapot.obj", 0.3, 0.3),
                                       # The example's own numbers: the display
                                       # list is halved to fit v16 and the bake
                                       # halves back, so collision is at true
                                       # scale. Exactly the case where the two
                                       # scales differ and must not be confused.
                                       ("level.obj", 0.5, 0.5)):
        path = os.path.join(ASSETS, obj)
        if not os.path.exists(path):
            print(f"  (skipping {obj}, not in the tree)")
            continue
        name = f"{obj} obj={obj_scale:g} bake={bake_scale:g}"
        yield name, ("obj", path, obj_scale), bake_scale

    # A flat floor: the ghost filter's subject, and the case where every
    # interior edge is flagged flat because the two faces are coplanar.
    yield "grid floor 6x6", ("colmesh", make_colmesh(grid_floor(6, 6.0))), 1.0

    # Enough triangles to build a tree several levels deep, with the leaf
    # count not a multiple of four.
    yield "grid floor 13x13", ("colmesh", make_colmesh(grid_floor(13, 13.0))), 1.0

    # Two identical triangles: the weld collapses the second onto the first and
    # the index-collision test drops it.
    tri = ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0))
    yield "coincident triangles", ("colmesh", make_colmesh([tri, tri, tri])), 1.0

    # A sliver thinner than one Q12 quantum: real area in double, none on the
    # lattice, so PD_MIN_CROSS_LENGTH has to drop it -- and something else has
    # to survive, or the bake refuses for a different reason.
    quantum = 1.0 / 4096.0
    sliver = ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.5, 0.0, 0.1 * quantum))
    yield "sliver below a quantum", ("colmesh", make_colmesh(
        grid_floor(2, 2.0, y=1.0) + [sliver])), 1.0

    # Three triangles on one edge. Only the first partner counts, and the C
    # pairs (e0,e1) and (e1,e2) rather than all three combinations -- this is
    # the case that catches a "tidier" rewrite of the edge loop.
    a = (0.0, 0.0, 0.0)
    b = (1.0, 0.0, 0.0)
    yield "non-manifold edge", ("colmesh", make_colmesh([
        (a, b, (0.5, 0.0, 1.0)),
        (a, b, (0.5, 1.0, -0.5)),
        (a, b, (0.5, -1.0, -0.5)),
        # A second component so the tree has something to split.
        ((4.0, 0.0, 0.0), (5.0, 0.0, 0.0), (4.0, 0.0, 1.0)),
        ((4.0, 0.0, 0.0), (5.0, 0.0, 1.0), (5.0, 0.0, 0.0)),
    ])), 1.0

    # Every centroid on one side of the pivot, forcing splitMedian's count / 2
    # fallback: coincident bounding boxes with distinct vertices.
    stacked = []
    for i in range(9):
        y = i * quantum
        stacked.append(((0.0, y, 0.0), (1.0, y, 0.0), (0.0, y + quantum, 1.0)))
    yield "coincident centroids", ("colmesh", make_colmesh(stacked)), 1.0

    # A concave crease and a convex one, so both the signedVolume branches are
    # taken rather than only the coplanar shortcut.
    yield "concave and convex creases", ("colmesh", make_colmesh([
        ((-1.0, 1.0, -1.0), (-1.0, 1.0, 1.0), (0.0, 0.0, 1.0)),
        ((-1.0, 1.0, -1.0), (0.0, 0.0, 1.0), (0.0, 0.0, -1.0)),
        ((0.0, 0.0, -1.0), (0.0, 0.0, 1.0), (1.0, 1.0, 1.0)),
        ((0.0, 0.0, -1.0), (1.0, 1.0, 1.0), (1.0, 1.0, -1.0)),
    ])), 1.0

    # Negative coordinates only, so the truncate-toward-zero rounding is
    # exercised on the side where a round-to-nearest baker would differ.
    yield "negative octant", ("colmesh", make_colmesh(
        [tuple((v[0] - 3.3, v[1] - 3.3, v[2] - 3.3) for v in t)
         for t in grid_floor(4, 4.0)])), 1.0

    # Coordinates that do not land on the lattice, so the weld has real work.
    yield "off-lattice grid", ("colmesh", make_colmesh(
        [tuple((v[0] * 1.0 / 3.0, v[1], v[2] * 1.0 / 7.0) for v in t)
         for t in grid_floor(5, 5.0)])), 1.0


# ---------------------------------------------------------------------------
# Comparison
# ---------------------------------------------------------------------------

def section_of(blob, offset):
    """Name the part of the blob an offset lands in, for a readable failure."""
    if offset < b3mesh.SIZEOF_MESH_DATA:
        fields = [
            (0, 8, "version"), (8, 12, "byteCount"), (12, 16, "hash"),
            (16, 40, "bounds"), (40, 44, "surfaceArea"), (44, 48, "treeHeight"),
            (48, 52, "degenerateCount"), (52, 56, "nodeOffset"),
            (56, 60, "nodeCount"), (60, 64, "vertexOffset"),
            (64, 68, "vertexCount"), (68, 72, "triangleOffset"),
            (72, 76, "triangleCount"), (76, 80, "materialOffset"),
            (80, 84, "materialCount"), (84, 88, "flagsOffset"),
        ]
        for start, end, name in fields:
            if start <= offset < end:
                return f"header.{name}"
        return "header"

    (node_offset, node_count, vertex_offset, vertex_count,
     triangle_offset, triangle_count, material_offset, _material_count,
     flags_offset) = struct.unpack_from("<9i", blob, 52)

    if node_offset <= offset < node_offset + node_count * b3mesh.SIZEOF_MESH_NODE:
        index = (offset - node_offset) // b3mesh.SIZEOF_MESH_NODE
        field = (offset - node_offset) % b3mesh.SIZEOF_MESH_NODE
        part = ("lowerBound" if field < 12 else
                "data" if field < 16 else
                "upperBound" if field < 28 else "triangleOffset")
        return f"node[{index}].{part}"

    if vertex_offset <= offset < vertex_offset + vertex_count * b3mesh.SIZEOF_VEC3:
        return f"vertex[{(offset - vertex_offset) // b3mesh.SIZEOF_VEC3}]"

    if triangle_offset <= offset < triangle_offset + triangle_count * b3mesh.SIZEOF_MESH_TRIANGLE:
        return f"triangle[{(offset - triangle_offset) // b3mesh.SIZEOF_MESH_TRIANGLE}]"

    if material_offset <= offset < material_offset + triangle_count:
        return f"material[{offset - material_offset}]"

    if flags_offset <= offset < flags_offset + triangle_count:
        return f"flags[{offset - flags_offset}]  <-- the edge classification"

    return "trailing padding"


def summarise(blob):
    (node_count, vertex_count, triangle_count) = (
        struct.unpack_from("<i", blob, 56)[0],
        struct.unpack_from("<i", blob, 64)[0],
        struct.unpack_from("<i", blob, 72)[0])
    height = struct.unpack_from("<i", blob, 44)[0]
    return (f"{len(blob):6d} B  {triangle_count:5d} tri  {vertex_count:5d} vtx  "
            f"{node_count:4d} nodes  height {height:2d}")


def compare(name, ref, mine):
    if ref == mine:
        print(f"  ok    {name:32s}  {summarise(mine)}")
        return True

    print(f"  FAIL  {name}")

    if len(ref) != len(mine):
        print(f"        bake_ref wrote {len(ref)} bytes, b3mesh.py wrote {len(mine)}")

    for i in range(min(len(ref), len(mine))):
        if ref[i] != mine[i]:
            print(f"        first difference at byte {i} ({section_of(ref, i)}): "
                  f"bake_ref 0x{ref[i]:02X}, b3mesh.py 0x{mine[i]:02X}")
            lo = max(0, i - 8)
            hi = min(len(ref), i + 8)
            print(f"        bake_ref   {ref[lo:hi].hex(' ')}")
            print(f"        b3mesh.py  {mine[lo:hi].hex(' ')}")
            break

    differing = sum(1 for x, y in zip(ref, mine) if x != y)
    print(f"        {differing} of {min(len(ref), len(mine))} bytes differ")
    return False


def main():
    if not os.path.exists(BAKE_REF):
        print("bake_ref is not built. Run: make -f Makefile.host bake_ref",
              file=sys.stderr)
        return 1

    print("Baking each case with tests/box3d_host/bake_ref (C) and "
          "tools/obj2dl/b3mesh.py, and comparing every byte.\n")

    failures = 0
    total = 0

    with tempfile.TemporaryDirectory() as tmp:
        for name, source, scale in corpus():
            kind = source[0]

            colmesh_path = os.path.join(tmp, "case.colmesh")

            if kind == "obj":
                # The real path: obj2dl parses the OBJ and writes the .colmesh
                # both bakers then read.
                #
                # --collision-b3 is asked for at the same time so that the blob
                # an author actually gets is compared too, not only the one
                # reached through bake_colmesh. Those are different code paths
                # into the same bake -- one goes through a file, one does not --
                # and generate_b3mesh quantizes its coordinates first precisely
                # so the two cannot drift apart. This is what checks that.
                result = subprocess.run(
                    [sys.executable, OBJ2DL,
                     "--input", source[1],
                     "--output", os.path.join(tmp, "case.bin"),
                     "--texture", "32", "32",
                     "--scale", repr(source[2]),
                     "--collision",
                     "--collision-b3",
                     "--collision-b3-scale", repr(scale)],
                    capture_output=True, text=True)
                if result.returncode != 0:
                    print(f"  FAIL  {name}: obj2dl failed\n{result.stdout}{result.stderr}")
                    failures += 1
                    total += 1
                    continue
            else:
                with open(colmesh_path, "wb") as f:
                    f.write(source[1])

            b3mesh_path = os.path.join(tmp, "case.b3mesh")

            result = subprocess.run(
                [BAKE_REF, colmesh_path, b3mesh_path, repr(scale)],
                capture_output=True, text=True)
            if result.returncode != 0:
                print(f"  FAIL  {name}: bake_ref failed\n{result.stderr}")
                failures += 1
                total += 1
                continue

            with open(b3mesh_path, "rb") as f:
                ref = f.read()

            try:
                mine, _info = b3mesh.bake_colmesh(colmesh_path, scale)
            except b3mesh.B3MeshError as e:
                print(f"  FAIL  {name}: b3mesh.py refused a mesh bake_ref accepted: {e}")
                failures += 1
                total += 1
                continue

            total += 1
            if not compare(name, ref, mine):
                failures += 1

            # And the blob obj2dl itself wrote, where there was one.
            direct_path = os.path.join(tmp, "case.b3mesh")
            if kind == "obj" and os.path.exists(direct_path):
                with open(direct_path, "rb") as f:
                    direct = f.read()

                total += 1
                if not compare(f"{name} via --collision-b3", ref, direct):
                    failures += 1

    print()
    if failures:
        print(f"{failures} of {total} cases differ.")
        print("\nA difference means the two bakers disagree about the blob the device")
        print("reads. Fix b3mesh.py to match mesh_bake.c -- the C is the reference,")
        print("because it is the one run_pair and test_world already exercise.")
        return 1

    print(f"{total} cases, every byte identical.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
