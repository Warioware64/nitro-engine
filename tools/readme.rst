Nitro Engine Advanced Tools
==================

The following tools are used to export models created on the PC to the NDS:

- **obj2dl**

  Converts a Wavefront OBJ file into a NDS display list.

  It also produces collision geometry, in either of two unrelated formats:

  ``--collision``
    a ``.colmesh`` for the older ``NEACollision`` module (``NEA_ColMesh``).

  ``--collision-b3``
    a ``.b3mesh`` for the Box3D physics engine
    (``NEA_Phys3DBodyAddMesh``). The bounding volume hierarchy, the vertex
    weld and the shared-edge classification are all baked here, because
    there is no run-time mesh builder on the DS — building a BVH on a
    67 MHz ARM9 is work for the asset pipeline.

  Both can be asked for at once. The Box3D one takes two more options::

      python3 obj2dl.py --input level.obj --output level.bin --texture 128 128 \
          --collision-b3 --collision-b3-scale 2 --collision-b3-c level

  ``--collision-b3-scale`` divides every collision coordinate. Set it. The
  solver's tolerances are absolute and want 1 unit to be about 1 metre, which
  is usually *not* the scale a display list is authored at — positions there
  are ``v16`` and top out at 8 units, so a level is normally shrunk to render
  and this is what undoes that for the physics.

  ``--collision-b3-c NAME`` also writes ``NAME_b3mesh.c`` / ``.h``, an array to
  compile into the ROM. Use it: the array is 8-byte aligned, which the mesh
  format requires and which ``bin2c`` does not give a file dropped in ``data/``
  — ``b3MeshData`` opens with a ``uint64_t`` and the ARM9 reads one with
  ``LDRD``, which faults on a 4-aligned address.

- **md5_to_dsma**

  Converts MD5 models with skeletal animation (md5mesh and md5anim files) into a
  format that allows them to be displayed on the NDS efficiently.

  https://codeberg.org/SkyLyrac/dsma-library

  Per-bone collision comes from a ``.md5collimesh`` and, as with obj2dl, in two
  unrelated formats:

  ``--collision <file.md5collimesh>``
    a ``.boncol`` for the older ``NEABoneCollision`` module.

  ``--collision-b3``
    additionally a ``.b3col`` of per-bone Box3D shapes, for
    ``NEA_Phys3DBodyAddBoneShape()``. Needs ``--collision``, which is where
    the shapes come from.

  A ``.b3col`` entry carries an orientation that a ``.boncol`` has no room for,
  so a bone capsule can lie along its bone rather than along Y. Give the bone an
  ``axis x y z`` in the ``.md5collimesh`` to use it; without one the shape stays
  Y-aligned and existing files behave exactly as before.

  The intended use is **one kinematic body per bone**, its transform driven each
  frame from the animated skeleton — a single rigid body cannot deform, so a
  hitbox set is bodies, not several shapes on one body.

- **img2ds**

  Converts images in several formats to NDS textures and palettes. It is
  recommended to use PNG files with transparency.

  This tool has been deprecated. You should only use it for the depth bitmap
  (DEPTHBMP), as this conversion isn't supported by any other tool.
