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
- Basic physics system: Axis-aligned bounding boxes (AABB) only.

Nitro Engine Advanced doesn't support any of the 2D hardware of the DS. In order
to use the 2D hardware you can use libnds directly, or you can use a library like
`NFlib <https://github.com/knightfox75/nds_nflib>`_. There is an example of how
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

Tools
=====

Nitro Engine Advanced includes the following conversion tools under ``tools/``:

- **obj2dl**: Converts Wavefront OBJ files into NDS display lists (``.bin``).
  Supports ``--texture`` and ``--scale`` options.
- **md5_to_dsma**: Converts MD5 mesh and animation files into NDS-compatible
  formats. Supports ``--multi-material`` for DLMM output with per-submesh
  materials.
- **img2ds**: Converts images to NDS textures and palettes (deprecated, except
  for DEPTHBMP conversion).

Blender Addon (MD5 Export)
==========================

The ``blender_addon/`` directory contains **io_scene_md5.py**, an MD5
import/export addon for **Blender 5.0** and above.

This addon is essential for the animated model pipeline:

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
