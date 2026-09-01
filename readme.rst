######################
Nitro Engine Advanced
######################

Introduction
============

Nitro Engine Advanced (NEA) is a 3D game engine for the Nintendo DS, forked from
`Nitro Engine <https://codeberg.org/SkyLyrac/nitro-engine>`_. It provides a
large set of functions designed to simplify the process of making a 3D game.
It isn't standalone — it needs libnds to work.

You can use Nitro Engine Advanced with `BlocksDS <https://blocksds.skylyrac.net>`_.

Features:

- Support for **static models**, converted from OBJ files with ``obj2dl``.
- Support for **multi-material static models** using the DLMM format, where
  each submesh can have its own texture and material properties.
- Support for **animated models** with skeletal animation (MD5 format),
  converted with ``md5_to_dsma``. Supports animation blending for smooth
  transitions. Multi-material animated models are also supported.
- **Animated material system**: keyframe-driven material animation with 13
  track types (alpha, lights, culling, vertex color, diffuse/ambient,
  specular/emission, material swap, polygon ID, texture scroll X/Y, texture
  rotate, texture scale X/Y). Exported from Blender via F-Curves, loaded at
  runtime as ``.neaanimmat`` binaries. Supports STEP and LINEAR interpolation.
- **Two-pass rendering** to double the polygon budget by splitting the screen
  into two halves, each rendered in a separate hardware frame (effective 30 FPS).
  Three modes available: FIFO, framebuffer, and HBL DMA.
- **Depth buffer mode selection**: switch between Z-buffering (linear, default)
  and W-buffering (reciprocal, better depth precision for perspective).
- **Texture matrix manipulation**: translate, rotate, and scale textures at
  runtime on materials using ``NEA_TEXGEN_TEXCOORD``. Materials with
  ``NEA_TEXGEN_OFF`` (default) are unaffected.
- Support for all texture formats (even compressed textures, thanks to
  `ptexconv <https://github.com/Garhoogin/ptexconv>`_).
- Dual 3D (render 3D to both screens, but at 30 FPS instead of 60 FPS).
- Functions to render 2D images accelerated by 3D hardware.
- Text system based on `libDSF <https://codeberg.org/SkyLyrac/libdsf>`_, which
  is based on `BMFont <https://www.angelcode.com/products/bmfont/>`_.
- Basic GUI elements like buttons and scrollbars.
- **Physics engine** with multiple collision shapes (AABB, sphere, capsule,
  triangle mesh), collision responses (bounce, stop, slide, sensor), mass-based
  impulse, collision groups with bitmask filtering, and collision callbacks.
- **Triangle mesh collision (ColMesh)**: generate collision meshes from OBJ files
  with ``obj2dl --collision``. Supports sphere-vs-mesh, AABB-vs-mesh, and
  capsule-vs-mesh detection with 64-bit precision arithmetic.
- **Per-bone collision** for animated models: attach collision primitives
  (sphere, capsule, AABB) to skeleton bones. Shapes follow the animation in
  real time for accurate hit detection on animated characters.
- **3D rigid body physics (Box3D)**: a fixed-point port of Erin Catto's
  `Box3D <https://github.com/erincatto/box3d>`_, with no floating point anywhere
  in the simulation. Spheres, capsules, convex hulls and baked triangle-mesh
  levels; persistent contacts with warm starting, islands and sleeping; nine
  joint types (distance, revolute, spherical, weld, motor, prismatic, parallel,
  wheel and a non-solving filter joint) with springs, motors and limits;
  continuous collision for fast-moving bodies; sensors; world ray casts, shape
  casts and overlap queries; and a **kinematic character mover** that walks a
  capsule up stairs and slopes without fighting the solver. Ships as a separate
  archive (``-lNEA_box3d``) so a project that does not want it pays nothing.
  See ``examples/physics/box3d_*``.
- **Sound system** (optional, powered by `Maxmod <https://maxmod.devkitpro.org/>`_):
  spatial 3D sound with distance attenuation and stereo panning, background music
  with volume/tempo/pitch control, non-spatial SFX, and WAV streaming. Spatial
  sources can be attached to models so audio follows objects in the scene. Enabled
  by building with ``NEA_MAXMOD=1`` — when disabled, the module compiles to
  nothing and has no dependencies.
- **Hardware 2D pipeline** (``NEAHw2D``): use the NDS 2D hardware alongside
  the 3D engine. Supports tiled backgrounds (4bpp/8bpp), bitmap backgrounds
  (8bpp indexed and 16bpp direct color), and hardware OBJ sprites with
  animation frames, flip, priority, palette slots, and affine transformations.
  VRAM banks assigned to 2D are automatically excluded from 3D texture
  allocation. Includes ``NEA_Hw2DTextRender()`` for rendering libDSF rich text
  onto 16bpp bitmap backgrounds. OAM updates are integrated into
  ``NEA_WaitForVBL()`` via the ``NEA_UPDATE_HW2D`` flag. See the examples
  under ``examples/hw2d/`` for tiled backgrounds, bitmap backgrounds, sprites,
  and text rendering on bitmap BGs.
- **Configurable texture palette VRAM**: ``NEA_SetTexPaletteBank()`` lets you
  choose which VRAM banks (E, F, G) back 3D texture palettes, freeing bank E
  for 2D backgrounds when needed.
- **Asynchronous asset loading**: load files in the background using BlocksDS
  cooperative threads, so the main loop keeps running — audio keeps streaming
  and the scene keeps rendering — while a file is read from the filesystem.
  This avoids the audio gaps and frame stutter caused by blocking loads. A
  worker thread reads the file in chunks, then a finalize step (such as a
  texture VRAM upload) runs during the vertical blank. ``NEA_FATLoadDataAsync()``
  reads any file, with async variants for textures (``NEA_MaterialTexLoadFATAsync()``,
  ``NEA_MaterialTex4x4LoadFATAsync()``, ``NEA_MaterialTexLoadGRFAsync()``) and
  models (``NEA_ModelLoadStaticMeshFATAsync()``, ``NEA_ModelLoadDSMFATAsync()``,
  ``NEA_ModelLoadMultiMeshFATAsync()``). Loads are advanced with the
  ``NEA_UPDATE_ASSETS`` flag of ``NEA_WaitForVBL()`` or by calling
  ``NEA_AsyncProcess()`` directly. See the ``examples/loading/async_loading``
  example. Note: for the loading to truly overlap on DS hardware, the
  filesystem driver must run on the ARM7 (DSi SD and cartridge NitroFS already
  do; on a DS flashcard call ``dldiSetMode(DLDI_MODE_ARM7)``).
- **Particle system (NEAParticle / NPE)**: SPL-style particle effects in a
  compact binary format (``.npe`` -- "Nitro Particle Entity"). Each emitter
  owns a fixed particle pool and runs a simple physics model (initial
  position inside a box, direction inside a cone with random speed, constant
  gravity, per-frame drag) with key-framed RGBA color and size over each
  particle's life. Particles draw as billboarded textured quads by default,
  with a per-effect axis-aligned mode for waterfalls / flame planes, plus
  sprite-sheet animation, per-particle angular velocity, and attachment to
  a ``NEA_Model`` (the emitter follows the model's position). Pump emitters
  with the ``NEA_UPDATE_PARTICLES`` flag of ``NEA_WaitForVBL()``. Design
  effects visually with ``tools/npe_editor/npe_editor.py`` -- a standalone
  tkinter editor with a live 2D preview that runs the same simulation logic
  as the runtime. See the ``examples/effects/particles`` example for fire /
  smoke / explosion presets.
- **Archive containers (NEANPAC / NPAC)**: packs a directory tree into one
  file, the way NARC does in retail games, and mounts it as a drive:
  ``NEA_NpacMount("levels", "nitro:/levels.npac")`` makes ``levels:/robot.bin``
  a path like any other. It registers with libnds as a ``device_io``
  filesystem, so ``fopen()``, ``opendir()``, ``stat()`` and every existing
  ``NEA_*LoadFAT()`` call reach it without changing -- there are no new loaders.
  Members are compressed individually by the packer and inflated transparently
  on ``open()``, so ``stat()`` reports the decompressed size. Several archives
  can be mounted at once under names you choose. Build one with
  ``tools/npac/npac.py``. See ``examples/loading/npac_archive`` and
  ``examples/loading/npac_multi_archive``.

For features not covered by NEA (e.g. advanced 2D tilemaps, scrolling engines),
you can also use libnds directly, or a library like `NFlib
<https://github.com/knightfox75/nds_nflib>`_. There is an example of how
to integrate Nitro Engine Advanced and NFlib in the same project `here
<./examples/templates/using_nflib>`_.

Setup
=====

Building from source
--------------------

1. Clone this repository and run:

   .. code:: bash

       make -f Makefile.blocksds install -j`nproc`

   This builds the library in both debug and release modes and installs it.

   To enable sound support (Maxmod integration), add ``NEA_MAXMOD=1``:

   .. code:: bash

       make -f Makefile.blocksds install NEA_MAXMOD=1 -j`nproc`

   Your project must also link ``-lmm9``, add ``-DNEA_MAXMOD`` to its defines,
   and use ``arm7_maxmod.elf`` as the ARM7 binary. See the examples under
   ``examples/sound/`` for reference Makefiles.

2. If you want to check that everything is working as expected, open one of the
   folders of the examples and run:

   .. code:: bash

       make

   That should build an ``.nds`` file that you can run on an emulator or real
   hardware.

Note: The build system of the examples in this repository is make. The makefiles
aren't very flexible, and they don't support converting 3D models, or saving
graphics or models to the filesystem (you can only inject them as data to the
ARM9, which isn't acceptable for big games).

You can also try `ArchitectDS <https://codeberg.org/blocksds/architectds>`_.
This build system written in Python supports converting every format that Nitro
Engine Advanced supports, and it lets you save everything in NitroFS so that your
game can grow as much as you want.

Usage notes
-----------

Note that some features of the 3D hardware aren't emulated by most emulators, so
you may need to use an actual NDS to test some things. **melonDS** seems to
emulate all features correctly. **DeSmuME** doesn't emulate the polygon/vertices
count registers, so the touch test feature of Nitro Engine Advanced doesn't work.

Normally you should link your programs with ``-lNEA``, which is the release
version. If you want to use the debug features, link with ``-lNEA_debug`` and
add ``-DNEA_DEBUG`` to the ``CFLAGS`` and ``CPPFLAGS`` in your Makefile. Make
sure to clean and rebuild your project after doing the changes mentioned in this
step. Check the **error_handling** example to see how to use the debug mode.

3D rigid body physics (Box3D)
-----------------------------

A fixed-point port of Erin Catto's `Box3D <https://github.com/erincatto/box3d>`_.
There is **no floating point anywhere in the simulation** — every length,
velocity, mass and impulse is a fixed-point integer in a format chosen for what
it holds, so the engine runs on an ARM9 that has no FPU without ever trapping
into a software float routine.

It ships in a **separate archive**, because it is roughly four times the code
size of the rest of the engine (about 340 KB of ARM9 ``.text`` against 90 KB).
``libNEA.a`` is Nitro Engine Advanced *without* it, so a project that does not
want physics pays nothing for it.

To use it, name it in your Makefile **before** including ``Makefile.example``
(the template sets ``LIBS`` with ``?=`` and yields to a value already set)::

    LIBS := -lNEA_box3d -lNEA -lnds9 -lc

and include ``<NEAPhysics3D.h>``. The debug build is ``-lNEA_box3d_debug``.
``NEAPhysics3D.h`` wraps the common path in NEA's conventions — f32 arguments,
``NEA_Model`` binding, a pool allocator that reports any allocation made after
the world starts running. The full Box3D API is in ``<box3d/box3d.h>``, the two
share all state, and they can be mixed freely.

What is there
~~~~~~~~~~~~~

- **Shapes**: spheres, capsules, convex hulls and baked triangle meshes, in any
  combination.
- **Joints**, nine of them: distance, revolute (hinge), spherical (ball),
  weld, motor, prismatic (slider), parallel (upright-keeper), wheel, and a
  filter joint that solves nothing and only stops two bodies colliding. Each
  with the springs, motors and limits its type allows.
- **Contacts** that persist across steps with warm starting, gathered into
  islands that fall asleep when they settle.
- **Continuous collision**, so a bullet does not tunnel through a wall. Set
  ``b3Body_SetBullet()`` on the fast body.
- **Sensors**: shapes that report what overlaps them and resolve nothing. Set
  ``b3ShapeDef::isSensor`` and read ``b3World_GetSensorEvents()``.
- **World queries**: ``NEA_Phys3DRayCast()`` and ``NEA_Phys3DOverlapBox()``, or
  ``b3World_CastRay``, ``b3World_CastShape``, ``b3World_OverlapShape`` and
  friends for the full versions.
- **A character mover**: ``NEA_Phys3DMover``, a kinematic capsule that is *not*
  a rigid body. A player made of a dynamic body has to fight the solver for
  every step, slope and ledge and loses — it slides down ramps, tips over and
  picks up spin. This one decides where it goes and puts it there, with
  friction, acceleration, jumping, a ground probe that climbs stairs with no
  step-up hack, and crates it can shove. See **physics/box3d_character**.

The time step is fixed at 60 Hz so that the solver's coefficients fold into
constants instead of costing a hardware divide per body per sub-step;
``NEA_Phys3DWorldStep()`` takes a sub-step count, not a duration. If your game
runs at 30 fps, step twice.

Levels are baked offline
~~~~~~~~~~~~~~~~~~~~~~~~

A level is a triangle mesh baked offline: run ``obj2dl --collision-b3`` to get
the ``.b3mesh`` that ``NEA_Phys3DBodyAddMesh()`` takes. There is no run-time
mesh builder, and none for convex hulls either — building a bounding volume
hierarchy or running quickhull on a 67 MHz ARM9 is work for the asset pipeline.
An animated model can carry per-bone hitbox shapes the same way, with
``md5_to_dsma --collision-b3``.

Sizing a world
~~~~~~~~~~~~~~

``NEA_Phys3DWorldDef::box3d.capacity`` is **not a hint**. Every pool is reserved
from it before the first step, and the world seals itself on that step: anything
allocated afterwards is counted by ``NEA_Phys3DWorldGetLateAllocCount()`` and,
under ``NEA_DEBUG``, asserts. Build your scene, print
``NEA_Phys3DWorldGetMemoryUsage()`` once, and size ``poolBytes`` from that plus
headroom. The one field that is not just a count is
``capacity.meshContactCount`` — a mesh contact costs several times what a convex
one does, and leaving it at zero in a scene with a mesh level is the sizing
mistake that shows up as a mid-frame allocation rather than a slightly small
pool.

The examples all print these counters on the bottom screen, which is the
quickest way to see what a scene of your own costs.

Examples
~~~~~~~~

Under ``examples/physics/``. Each one is a single feature with a readout, and
each ``main.c`` opens with a note on what to watch and why:

======================= ======================================================
Example                 Shows
======================= ======================================================
``box3d_basic``         Bodies, shapes, contacts and sleeping
``box3d_level``         A baked triangle mesh level
``box3d_character``     The character mover: stairs, walls and crates
``box3d_pick``          Ray casts and box overlap queries
``box3d_bullet``        Continuous collision — a fast sphere at a wall
``box3d_sensor``        A trigger volume over a level
``box3d_rope``          Distance joints
``box3d_hinge``         A revolute joint
``box3d_slider``        A prismatic joint
``box3d_motor``         Weld and motor joints
``box3d_ragdoll``       Spherical joints
``box3d_car``           Wheel joints — a car on terrain
======================= ======================================================

Choosing what goes in ITCM
~~~~~~~~~~~~~~~~~~~~~~~~~~

The ARM9's instruction cache is 8 KB and the solver does not fit in it. The
sub-step loop calls each joint's solve routine twice per sub-step per joint —
32 times a frame for four joints at the default four sub-steps — so a solve
function larger than the cache re-streams its entire body from main RAM on
every call. ITCM, the ARM9's 32 KB of zero-wait-state instruction memory,
removes that fetch. On a DSi, moving just the shared math helpers there took
**examples/physics/box3d_car from ~240% CPU to ~150–160%**.

Two things are always in ITCM: about 3 KB of shared math (``b3RotateVector``,
``b3MulQuat`` and the rest of ``source/box3d/b3hot.c``) and, by default, the
contact solver. Everything else is opt-in, because ITCM is one budget shared
with your whole game and only you know which joints your scenes use — the
library cannot guess, and ``b3SolveJoint``'s type switch means every joint
implementation is linked into any ROM that uses joints at all.

============================== ========= ==========
Group                              Bytes    Default
============================== ========= ==========
``NEA_BOX3D_ITCM_CONTACTS``        7,252   **on**
``NEA_BOX3D_ITCM_WHEEL``          14,104   off
``NEA_BOX3D_ITCM_PRISMATIC``      10,180   off
``NEA_BOX3D_ITCM_MESH``            8,496   off
``NEA_BOX3D_ITCM_SPHERICAL``       7,456   off
``NEA_BOX3D_ITCM_REVOLUTE``        7,376   off
``NEA_BOX3D_ITCM_CORE``            7,264   off
``NEA_BOX3D_ITCM_DISTANCE``        6,516   off
``NEA_BOX3D_ITCM_MOTOR``           5,976   off
``NEA_BOX3D_ITCM_WELD``            4,600   off
``NEA_BOX3D_ITCM_PARALLEL``        2,640   off
============================== ========= ==========

Each joint group is that joint's solve plus warm-start routine. Roughly
12,400 bytes are free on top of the defaults, so you will not fit everything —
that is the point of choosing. Print the live figures with::

    make -f Makefile.blocksds itcm-report

Set a group to ``1`` to add it, or to ``0`` to drop one that is on by default::

    make -f Makefile.blocksds install -j`nproc` \
        NEA_BOX3D_ITCM_CONTACTS=0 NEA_BOX3D_ITCM_WHEEL=1

That is exactly what a vehicle game wants, and it has to turn contacts off:
wheel is 14,104 bytes against the 12,416 that remain with contacts on. A
ragdoll instead wants ``NEA_BOX3D_ITCM_SPHERICAL=1``, which fits alongside the
default. Mixed-joint scenes benefit most — a spherical solver (6,244 B) and a
revolute solver (6,108 B) interleaved in one pass have a 12 KB working set
against an 8 KB cache and evict each other on every joint.

Overshooting the budget is a link error when you build your ROM, not a silent
failure::

    section .itcm will not fit in region itcm

``NEA_BOX3D_NO_ITCM=1`` is the master switch and keeps the whole library out of
ITCM, including the shared math.

This is independent of the two older physics modules (``NEAPhysics.h`` and
``NEACollision.h``), which are unchanged and still in ``libNEA.a``.

Tools
=====

Nitro Engine Advanced includes the following conversion tools under ``tools/``:

- **obj2dl**: Converts Wavefront OBJ files into NDS display lists (``.bin``).
  Supports ``--texture``, ``--scale``, ``--collision`` (a ``.colmesh`` for the
  ``NEACollision`` module) and ``--collision-b3`` (a baked ``.b3mesh`` for the
  Box3D physics engine).
- **md5_to_dsma**: Converts MD5 mesh and animation files into NDS-compatible
  formats. Supports ``--multi-material`` for DLMM output with per-submesh
  materials, and ``--collision <md5collimesh>`` to generate per-bone collision
  data (``.boncol``) from a collision mesh exported by the Blender addon.
  ``--collision-b3`` writes the same bones as Box3D shapes (``.b3col``).
- **img2ds**: Converts images to NDS textures and palettes (deprecated, except
  for DEPTHBMP conversion).
- **npac**: Packs a directory tree into an NPAC archive container, which
  ``NEA_NpacMount()`` mounts as a drive at run time.

  .. code:: bash

      python3 tools/npac/npac.py create --input assets/ --output levels.npac

  ``extract`` and ``list`` go the other way. Compression is per file and is
  done by the Wonderful Toolchain encoders (``wf-nnpack-lzss``,
  ``wf-nnpack-huffman``, ``wf-nnpack-rle``); ``--compress auto``, the default,
  tries each and stores the file when none of them wins. Huffman is left out of
  ``auto`` unless ``--allow-huffman`` is given, because melonDS's default HLE
  BIOS decodes it to the wrong bytes without reporting an error -- hardware and
  a real BIOS dump are both fine.

Blender Addon
=============

The ``blender_addon/`` directory contains **io_scene_md5.py**, a comprehensive
addon for **Blender 5.0** and above. It provides:

- MD5 mesh/animation import and export
- **Animated material editor** (Material Properties panel): keyframe NDS material
  properties (alpha, colors, material swap, texture scroll/rotate/scale) using
  Blender's F-Curve system. Animated properties are previewed in the viewport
  during playback. Export to ``.neaanimmat`` binary with one click.
- **Per-bone collision editor** for animated models.
- Integrated conversion tool execution (obj2dl, md5_to_dsma, ptexconv).

Animated model pipeline:

1. **Model and rig** your character in Blender with an armature.
2. **Assign materials** — each material slot with a unique shader image becomes
   a separate mesh block in the MD5 export. Multi-material models are fully
   supported.
3. **Animate** using Blender's action system (the addon supports Blender 5.0's
   action slots and bone collections).
4. **Export** via *File > Export > MD5 Mesh/Anim (.md5mesh/.md5anim)*.
5. **Convert** the exported MD5 files with ``md5_to_dsma``:

   .. code:: bash

       python3 tools/md5_to_dsma/md5_to_dsma.py \
           --mesh model.md5mesh \
           --anim walk.md5anim \
           --output data/model \
           --multi-material

6. **Load** in your game code:

   .. code:: c

       // For single-material animated models:
       NEA_ModelLoadDSMA(model, mesh_bin, anim_bin);

       // For multi-material animated models (DLMM):
       NEA_ModelLoadMultiMesh(model, dlmm_bin);
       NEA_ModelSetSubMeshMaterialByName(model, "skin", skin_material);
       NEA_ModelSetSubMeshMaterialByName(model, "armor", armor_material);

Installing the Blender addon
----------------------------

1. Open Blender 5.0+.
2. Go to *Edit > Preferences > Add-ons > Install from Disk*.
3. Select ``blender_addon/io_scene_md5.py``.
4. Enable the addon in the list.

Screenshots
===========

Screenshots of some of the examples included with Nitro Engine Advanced:

.. |animated_model| image:: screenshots/animated_model.png
.. |box_tower| image:: screenshots/box_tower.png
.. |fog| image:: screenshots/fog.png
.. |specular_material| image:: screenshots/specular_material.png
.. |screen_effects| image:: screenshots/screen_effects.png
.. |shadow_volume| image:: screenshots/shadow_volume.png
.. |sprites| image:: screenshots/sprites.png
.. |text| image:: screenshots/text.png

+------------------+-------------------+
| Animated model   | Box tower physics |
+------------------+-------------------+
| |animated_model| | |box_tower|       |
+------------------+-------------------+

+------------------+---------------------+
| Hardware fog     | Specular material   |
+------------------+---------------------+
| |fog|            | |specular_material| |
+------------------+---------------------+

+------------------+-------------------+
| Text             | Shadow volume     |
+------------------+-------------------+
| |text|           | |shadow_volume|   |
+------------------+-------------------+

+------------------+-------------------+
| Screen effects   | 2D sprites        |
+------------------+-------------------+
| |screen_effects| | |sprites|         |
+------------------+-------------------+

Contact
=======

This fork is hosted on `GitHub <https://github.com/Warioware64/nitro-engine>`__.

The original Nitro Engine is hosted on `Codeberg <https://codeberg.org/SkyLyrac/nitro-engine>`__.

License
=======

The code of this repository is under the MIT license. The examples are under the
CC0-1.0 license.

The full text of the licenses can be found under the ``licenses`` folder.

Thanks to
=========

- **BlocksDS**: https://blocksds.skylyrac.net/
- **SkyLyrac**: Original Nitro Engine author
- **devkitPro**: https://devkitpro.org/
- **DLDI**: https://www.chishm.com/DLDI/
- **DeSmuME**: http://desmume.org/
- **melonDS**: https://melonds.kuribo64.net/
- **no$gba**: https://problemkaputt.de/gba.htm
- **gbatek**: https://problemkaputt.de/gbatek.htm
- **gbadev forums**: https://forum.gbadev.org/
