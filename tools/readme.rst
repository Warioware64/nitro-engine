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

- **cell_pack** and **cell_import**

  The two ways to get a ``.neacell`` cell bank, in ``tools/cell_editor/``.

  ``cell_pack.py`` slices a PNG spritesheet, trims each frame to its alpha
  bounding box, grows it to one of the twelve hardware OBJ sizes, packs the
  result into a power-of-two atlas and writes the bank plus every piece of
  artwork the runtime needs::

      python3 tools/cell_editor/cell_pack.py sheet.png --grid 8x4 \
          --out walk --fps 12 --anchor-bottom

  ``cell_import.py`` does the same job starting from retail NCER/NANR/NCGR/NCLR
  files, decompressing the LZ, RLE or Huffman wrapper they usually carry::

      python3 tools/cell_editor/cell_import.py --ncer c.NCER --nanr c.NANR \
          --ncgr c.NCGR --nclr c.NCLR --out chars

  The atlas is packed to the smallest power-of-two rectangle that fits, sizing
  width and height independently -- the DS does, and rounding both up to a
  square costs texture VRAM for nothing.

  Both write five files: the bank, a linear atlas texture and its palette for
  the 3D backends, and a ``.ncgfx`` tile blob plus ``.ncpal`` for the hardware
  OBJ backend. The same pixels in two orders, because the two renderers want
  them differently and quantising twice would let them disagree about what a
  colour index means. The ``.ncgfx`` has to stay resident in main RAM: it is
  what ``NEA_CellAnimBindOAM()`` streams each frame's tiles out of.

  ``--bank-ext .bin`` writes the bank with a ``.bin`` extension, which is what
  an example needs, because the build only runs ``bin2c`` over ``*.bin``.

  Neither builds a **multi-cell** -- an entity composed of several sequences
  running on their own clocks. There is no grid or retail file that expresses
  one, so it comes from the editor or from a script calling ``cell_format``
  directly; ``examples/2d_system/cell_composition/graphics/make_assets.py`` is
  a worked example of the latter.

- **img2ds**

  Converts images in several formats to NDS textures and palettes. It is
  recommended to use PNG files with transparency.

  This tool has been deprecated. You should only use it for the depth bitmap
  (DEPTHBMP), as this conversion isn't supported by any other tool.

Editors
=======

Two of the tools are interactive rather than command-line. Both need Python's
``tkinter`` and `Pillow <https://pypi.org/project/Pillow/>`_::

    pip install Pillow

Pillow is what lets their previews rotate, scale, tint and alpha-composite real
artwork, none of which Tk can do on its own.

- **npe_editor**

  Edits a particle effect and saves it as the ``.npe`` binary ``NEAParticle``
  loads. The preview runs the same simulation the C runtime does, so what it
  shows is what the DS will show::

      python3 tools/npe_editor/npe_editor.py effects/fire.npe

- **animmat_editor**

  Edits a material animation and saves it as the ``.neaanimmat`` binary
  ``NEAAnimMat`` loads, in either format version. Tracks that interpolate get a
  curve with draggable keyframes; tracks the hardware can only step through get
  a strip of labelled segments, because a light mask or a culling mode plotted
  on a numeric axis is not readable::

      python3 tools/animmat_editor/animmat_editor.py panels.neaanimmat

  Its evaluator is checked frame-for-frame against the C runtime by
  ``tests/animmat_eval``, which is what makes the preview trustworthy.

- **cell_editor**

  Edits a ``.neacell`` cell bank: composes cells by dragging parts, builds
  sequences on a timeline whose frame widths are their durations, and animates
  parts with keyframe tracks over a parent hierarchy::

      python3 tools/cell_editor/cell_editor.py hero.neacell

  Its preview has a backend switch. **3D quads** shows full fidelity;
  **hardware OBJ** drops everything the OBJ hardware could not draw and lists
  what it dropped, so "will this still work as sprites?" is answered while
  authoring rather than at build time; **billboard** foreshortens the plane
  with a yaw slider, which is the one thing about that backend easy to get
  wrong without noticing. All three run
  ``cell_format.evaluate_sequence()``, which ``tests/cell_eval`` checks
  tick-for-tick against ``NEACell.c``.

  It can also import retail NCER/NANR through **File → Import retail**, though
  that path builds the bank only -- run ``cell_import.py`` to get the artwork
  too.

None of these three formats records the artwork it animates -- each names a
material that the runtime resolves later -- so the editors take the image
separately, through **File → Import image**, and remember it in a
``<file>.editor.json`` sidecar. The binary itself is never touched by that, and
losing the sidecar costs the preview and nothing else.
