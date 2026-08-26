Changelog
=========

Unreleased
----------

**The scene system and the ARM7 rigid body system were removed.** Both were
designed badly enough to be unusable in practice, and neither was load-bearing:
nothing inside the engine ever called into the scene system, and the rigid body
system reached it through a single weak reference in ``NEA_WaitForVBL()``.

- Gone from the library: ``NEAScene.c/h`` (the ``.neascene`` node hierarchy with
  tags and trigger zones) and ``NEARigidBody.c/h`` with ``NEA_RB_IPC.h`` (OBB
  bodies solved on the ARM7 over FIFO). ``NEAMain.h`` no longer includes
  ``NEAScene.h``.
- ``NEA_UPDATE_RIGIDBODY`` is gone from ``NEA_UpdateFlags``. Its ``BIT(6)`` is
  left as a **gap** on purpose, so every other update flag keeps the value it
  had -- code passing the surviving flags is unaffected.
- **The whole ``arm7/`` subproject went with it.** It existed only to carry the
  ARM7 solver, so ``arm7_nea.elf`` and ``arm7_nea_maxmod.elf`` are no longer
  built or installed, and ``make install`` no longer creates
  ``$(BLOCKSDSEXT)/nitro-engine-advanced/arm7/``. A project that set
  ``ARM7ELF := .../arm7/arm7_nea.elf`` must go back to the stock
  ``$(BLOCKSDS)/sys/default_arm7/arm7.elf``, which already supports Maxmod --
  that is what every other example was using all along.
- The authoring side went too: ``tools/neascene_export/`` and the ``.neascene``
  half of the Blender addon (node panel, trigger overlay, scene export operator
  and binary writer -- about 580 lines). The MD5, animated material, per-bone
  collision and texture VRAM parts of the addon are untouched.
- Examples removed: the four under ``examples/loading/scene_*`` and
  ``examples/physics/rigid_body``.

Box3D (``NEAPhysics3D``) is a separate ARM9 engine and is **not** affected; it
remains the answer for rotating bodies, stacking and contact events.

**Both mesh exporters gained ``--smooth-normals``.** Neither could produce smooth
shading from geometry, and ``md5_to_dsma.py`` could not produce it at all: both
its skinning paths computed one normal per triangle and handed it to all three
corners, so every MD5 model NEA has ever exported was faceted. The ``tentacle``
test asset is a ten-sided cylinder that had exactly **ten distinct normals across
its 780 triangles** for that reason.

``obj2dl.py`` passed through whatever ``vn`` entries the OBJ carried, and when a
file had none it emitted no ``NORMAL`` command at all -- leaving the GPU to light
the mesh with whatever normal the previous draw had left behind, silently.

- The flag takes an optional crease angle, defaulting to 60 degrees, and averages
  the faces meeting at a vertex that are shallower than that. Sharper edges stay
  hard: ``cube.obj`` at 60 comes out byte-identical to flat, and only rounds over
  at 120.
- **Omitting the flag changes nothing.** Every exporter output in the tree is
  byte-identical without it.
- Smoothing runs over the **whole model** before it is split by material, so a
  surface that continues across a material boundary does not gain a lighting
  seam there.
- Adjacency is by **position**, not by vertex index. An OBJ exported per-face has
  a separate vertex entry for every corner, so index-based adjacency would find
  no neighbours and silently produce flat output on exactly those files.
- Averaging is area-weighted, which the cross product gives for free: its
  magnitude is twice the triangle's area, so a fan of slivers cannot outvote the
  one large face a vertex mostly belongs to. The angle test uses normalised
  normals so the threshold does not depend on triangle size.
- The Blender addon exposes it for both pipelines, with the crease angle greyed
  out until the checkbox is on.
- ``examples/assets/tentacle`` is now exported smooth, and its example says the
  banding is the texture rather than the shading.

Smoothing changes the display list in two directions at once, and which wins
depends on the mesh. Corners that used to differ only by which triangle they
belonged to now share a strip vertex key, so fewer vertices are emitted -- the
sphere goes from 180 to 132. But ``DisplayList.normal()`` only skips a ``NORMAL``
command when it repeats the previous one, and a flat mesh repeats it for a whole
strip while a smooth one does not, so each corner starts costing its own. The
sphere ends up smaller (2356 to 2312 bytes) and the robot larger (18820 to
20620), because most of the robot's edges are sharper than 60 degrees and get no
merging to pay for the extra normals.

**npe_editor's curves are graphs now, matching the animmat editor.** Colour and
size over a particle's life are keyframed curves exactly like a material
animation track, but they were edited through a listbox and a chain of modal
prompts -- moving one colour key meant answering three dialogs in a row, and the
only picture of the result was a read-only 24 pixel strip. They use the same
timeline the animmat editor does, down to the palette and the wording:

- **Size over life** is a draggable curve. Click a key to select it, drag to
  move it, double-click empty space to add one, right-click to delete.
- **Colour over life** puts the colour itself behind the graph as a band, faded
  by its own alpha the way the DS composites it, and plots **alpha as the
  curve** -- because a colour is not a height but alpha is, and alpha is the
  channel that decides whether a particle is visible at all. Each key is a
  swatch you can drag in time and in alpha, with a picker for its RGB.
- Both carry the numeric ``t`` / value / **Set** row underneath, so a key can be
  placed exactly rather than by eye.
- The curve is sampled through the same functions the simulator and the runtime
  use, so the graph cannot disagree with the preview beside it.
- Dropping a key onto a time that already has one is refused, on drag and on
  typed edits alike. Two keys at one time leave a zero-length span that the
  samplers skip, so one of them would silently stop mattering -- the same defect
  that was fixed in the animmat timeline.

That removed the keyframe listboxes, their add/edit/delete buttons and the
prompt dialogs behind them, about 130 lines.

**Particle pipeline fixes.** An audit of ``NEAParticle.c`` against the format
module and the editor's simulator turned up six defects.

- **Fixed: an emitter attached to a deleted model read freed memory.**
  ``NEA_ParticleEmitterAttachToModel()`` keeps a raw pointer and
  ``ne_update_emitter()`` reads the model's position every frame, but nothing
  cleared it when the model went away. ``NEA_ModelDelete()`` now calls
  ``__NEA_ParticleDetachModel()``, the same way it already cancelled
  asynchronous loads that would write into the model.
- **Fixed: overlapping particles could not blend with each other.** Every
  particle was submitted with ``POLY_ID(0)``, and the hardware refuses to blend
  a translucent polygon over a translucent pixel carrying the same ID -- so in a
  dense effect the first quad drawn at a pixel won and the rest were discarded.
  Every particle in the ``fire`` preset is translucent for its whole life, so
  none of them layered. An emitter now spreads its particles over eight IDs, the
  same trick ``NEA_RichTextRender3D()`` uses for overlapping glyphs, moveable
  with ``NEA_ParticleEmitterSetPolyID()``. Measured on the particles example:
  **1.19x luminance and 1.18x coverage**.
- **Fixed: a truncated ``.npe`` read past the end of its buffer.** The parser
  walks the file using counts stored inside it and took a bare pointer, so there
  was nothing to compare them against. The file loaders now pass the size they
  already had -- ``__NEA_FATLoadDataSize()`` for the synchronous path, and the
  asynchronous one was discarding a size it was being handed --  and
  ``NEA_ParticleEmitterLoadSize()`` exposes the bounded parse. The unbounded
  ``NEA_ParticleEmitterLoad()`` stays for ROM-linked data and now says so.
  ``tests/particle_load`` feeds it prefixes cut at each stage of the parse.
- **Fixed: the sprite-sheet flag was ignored.** The runtime played a sheet
  whenever ``cols * rows > 1``, regardless of ``FLAG_SPRITESHEET``, so an effect
  with the flag off but a leftover grid animated on hardware and not in the
  editor. The flag decides now.
- **Implemented: velocity stretch.** ``FLAG_STRETCH`` was parsed and ignored. A
  stretched particle's quad is lengthened along its direction of travel and
  tapers as it slows.
- **Implemented, as far as the hardware allows: additive.** ``FLAG_ADDITIVE``
  was also parsed and ignored, and it cannot be honoured literally -- DISP3DCNT
  offers alpha blending on or off and the DS 3D engine has no additive mode at
  all. The runtime now weights each particle's alpha by its brightness instead,
  so a dark particle contributes almost nothing where adding zero would, and a
  bright one dominates. Together with the polygon IDs above that is close enough
  for sparks and flames. The editor preview does exactly the same arithmetic
  rather than compositing a true add it could never reproduce.

**npe_editor: the simulator had drifted from the runtime.** Its docstring calls
it a copy of ``NEAParticle.c``'s per-frame logic; three things had come loose.

- **Cone spread was half the width the DS produces.** The runtime picks an angle
  in 0..``cone_spread`` NEA units and feeds ``sinLerp(theta << 6)`` against a
  32768-unit circle, so one unit is pi/256 radians; the editor was using
  pi/511. An effect authored at ``cone_spread`` 128 was a 45 degree cone in the
  editor and a 90 degree one on hardware.
- The random-direction sphere used a half turn for the polar angle where the
  runtime uses a whole one.
- A zero particle life became one frame in the editor and sixty on hardware.

**Both editors can see the artwork now.** ``npe_editor`` drew particles as flat
coloured ovals and ``animmat_editor`` drew a tinted rectangle with a wireframe
grid, because neither format records the texture it animates -- both name a
material the runtime resolves later. **File -> Import image** now points either
editor at the real artwork, remembered in a ``<file>.editor.json`` sidecar. The
binary is never touched by that, and losing the sidecar costs the preview and
nothing else.

- The particle preview **composites** instead of drawing shapes, which is what
  makes the two flags that matter real rather than approximated: additive
  blending becomes a genuine add, and a sprite sheet plays its cells. Rotation
  and per-particle size come along with it. Transformed sprites are cached per
  (cell, rotation, size) bucket, since rotating every particle every frame is
  not something Python does at 30 fps.
- The material preview textures its quad and applies the real UV transform, with
  an all-targets mode -- driving several materials at once being the point of a
  version 2 animation -- and a switch for whether the tint comes from the vertex
  colour or from emission, since a vertex-colour track does nothing on a mesh
  with normals.
- Both editors now need `Pillow <https://pypi.org/project/Pillow/>`_. Their
  previews cannot rotate, scale, tint or alpha-composite without it, and
  ``tools/img2ds`` already depended on it.

**Fixed: a texture matrix translation is in sixteenths of a texel.** The
hardware multiplies that row of the matrix by a constant 1/16 (GBATEK, texture
coordinate transformation mode 1), so ``NEA_TextureMatrixTranslate(64.0)`` moves
a texture by four texels, not sixty-four. Nothing said so, and both material
animation examples were consequently scrolling a sixteenth of what their
comments claimed. Confirmed on hardware: with a 64 texel texture, a translation
of 1024 is pixel-identical to zero, and 512 is half a wrap.
``NEA_TextureMatrixTranslateI()`` now documents the unit, and
``NEA_TextureMatrixTranslateTexels()`` says it in texels for you. The preview
finding this is exactly what a preview is for.

**animmat_editor cleanup.** It was written in one pass and showed it.

- **Discrete tracks are no longer plotted as numbers.** Lights, culling,
  material swap and texture/palette swap never interpolate, so a line on a
  numeric axis said nothing -- a bitmask, or a staircase between 0, 64, 128 and
  192. They get a strip of labelled segments instead, and a value editor that
  speaks their language: a dropdown for culling, four checkboxes for lights,
  spinboxes with an "unchanged" state for the swap pair.
- **Losing work is harder.** There was no dirty flag and no unsaved-changes
  guard at all, where ``npe_editor`` has had one since it was written. New, Open
  and closing the window now ask.
- Dragging a key onto another key's frame silently produced two keys on one
  frame, which leaves a zero-length span the runtime skips -- so one of them
  quietly stopped doing anything. Refused now, on drag and on typed edits alike.
- Changing the length used to freeze a baked track's last value across the new
  frames; it resamples, and keyframes scale with it.
- The value boxes kept the previous track's contents after switching tracks, so
  **Set** wrote a stale value into the new one.
- Two targets could share a name. ``NEA_AnimMatFindTarget()`` returns the first
  match, so the second never bound and never said so.
- The colour picker handled only the vertex-colour track and explained itself
  with an error box after the click. It is enabled by track type now, and the
  packed diffuse/ambient and specular/emission tracks get a swatch per half --
  they could not be authored at all before, which is why the version 2 example
  had to write its emission values by hand in Python.
- The interpolation control stayed active for constant and baked tracks, where
  it does nothing.
- **Save As version 1** exists, with the checks that make it safe. Opening a
  version 1 file and saving silently rewrote it as version 2.
- The vertical axis on the fixed-point tracks was labelled with values run
  through the fixed-point conversion twice, so it was neither correct nor in
  order.
- The size report re-serialised the whole animation twice on every keystroke,
  once through ``optimize()``; it is debounced. A drag re-evaluated every frame
  of the track on each mouse motion just to find the plot range; that is
  computed once per gesture.

**NSMW can now fit a smoothly weighted mesh.** NSMW spends one matrix-stack
slot -- one "node" -- per distinct combination of bone pair and blend weight,
and the hardware stack has 31 levels. ``resolve_node()`` keyed on the exact
weight, at 1/4096 resolution, so two vertices weighted 0.500 and 0.501 to the
same bone pair took two of the 30 slots. Any smoothly weighted character blew
the budget and the exporter raised an error telling the user to "simplify the
rig", with no tool to do it.

This had never been exercised: every skinned asset in the tree is rigidly
weighted, ``Robot.md5mesh`` included, so the two-weight path NSMW exists for had
no test case at all.

- **A test asset that actually has the property.**
  ``examples/assets/tentacle/`` is a chain of 12 bones in a cylinder with the
  blend varying per vertex, generated by a committed script. It wants **194
  slots** for its 11 bone pairs.
- **Clustering.** ``md5_to_dsma.py`` merges the closest two nodes of a bone pair
  until the budget is met, weighting each merge by how many vertices it affects
  so a node covering three vertices cannot drag one covering three hundred. The
  tentacle comes down to 30 slots for a worst-case blend error of 0.230 and an
  average of 0.096 per vertex, and the tool prints both.
- **``--max-nodes``** sets the budget and **``--weight-buckets``** pre-rounds the
  weights. Quantizing is **off by default** because measurement says it makes
  the result worse, not better: on the tentacle, clustering alone gives 0.230
  worst-case error, and quantizing to 1/16 first gives 0.368. Rounding throws
  away the vertex counts clustering uses to decide what matters. It survives
  only as a speed knob, since clustering is O(n^2).
- The rest pose is untouched. Vertex bind positions still use the weights the
  artist authored, and a node matrix is the identity at rest whatever its
  weights are, so clustering changes only how the mesh deforms.

``examples/loading/nsmw_node_budget`` loads the same tentacle at three budgets
(30, 16 and 11 slots) side by side, on a banded texture that makes a skinning
error show up as a kink in the stripes.

Clustering cannot go below one node per bone pair, so a mesh that blends between
more than 30 pairs still cannot be exported. Getting past that means splitting
the mesh into groups drawn in separate passes, which needs a per-submesh node
set in the NSMW file and a per-group prepare at run time, not just an exporter
change. The error message says so explicitly rather than pointing at the rig.

**Material animation, rebuilt around what retail DS games did.** The
reverse-engineered SRT0 and PAT0 subfile formats (documented in
``helpSrc/nsbmd_docs.txt``) target a material *by name* and store dense channels
as flat per-frame arrays. NEAAnimMat now does both, which fixes the two things
it could not do.

- **One animation can drive a whole model.** An ``NEA_AnimMatInstance`` used to
  be one draw call's worth of GPU state, and ``NEA_ModelDraw()`` walks every
  submesh internally, so a multi-material model could not be animated per
  material at all. A version 2 file groups its tracks under named targets, and
  ``NEA_ModelSetAnimMat()`` matches those names against the material names the
  submeshes carry from their DLMM file. The matching happens once, at bind time,
  so nothing compares strings while drawing.
- **Version 1 files still load.** They parse into a single unnamed target, which
  is what a one-material version 2 file looks like, so the old "apply then draw"
  idiom is untouched.
- **Storage modes.** Every track used to cost a binary search, a division and an
  interpolation every frame. ``NEA_AMSTORE_CONST`` stores an unchanging value in
  the track header and costs nothing at all; ``NEA_AMSTORE_BAKED`` stores one
  16 bit value per frame and costs a single indexed load, with the four texture
  translate/scale tracks encoded as 1.10.5 exactly as retail does.
  ``NEA_AMSTORE_KEYS`` is the old path, kept for sparse tracks.
- **``NEA_AMTRACK_TEXPAL_SWAP``**, PAT0's trick: swap the texture and the palette
  out of small tables instead of exchanging a whole ``NEA_Material``. This is
  what flipbooks, blinking eyes and scrolling water want.
  ``NEA_MaterialTexUse()`` binds an image without disturbing colours,
  properties or palette.
- **Fixed: one flag governed three colour registers.** ``has_color_props`` was
  set by any of the vertex-colour, diffuse/ambient or specular/emission tracks,
  and the apply path then wrote all three registers from it. A target animating
  only its colour therefore also zeroed the material's diffuse and specular.
  Latent in version 1, where an animation tended to drive all of them or none;
  unavoidable in version 2, where partial track sets across several targets are
  the normal case. Each register now has its own flag and is written only if a
  track drives it.
- **The apply path stopped resetting the texture matrix every frame.** It called
  ``NEA_TextureMatrixIdentity()`` unconditionally and then built the transform
  through three helpers that each switch matrix mode and switch back -- up to
  eight mode switches for one transform, plus an identity load for instances
  that never touch the texture matrix. It is now one mode switch, and the reset
  only happens when a previous apply actually left a transform loaded.

**Fixed: keyframed alpha, polygon ID, angle and colour tracks never actually
interpolated.** ``mulf32()`` already brings its product down by 12 bits, and the
integer lerps shifted the result down by 12 again, dividing every step by a
further 4096. A linear alpha ramp from 31 to 3 across seventeen frames moved
from 31 to 30 and then snapped. Every keyframed ``ALPHA``, ``POLYID``,
``TEX_ROTATE``, ``COLOR``, ``DIFFUSE_AMBIENT`` and ``SPECULAR_EMISSION`` track
has been behaving as a step. The texture translate/scale tracks were correct and
are unchanged. **Animations tuned around the old behaviour will now move.**

**New: an editor.** ``tools/animmat_editor/`` follows the ``npe_editor``
pattern: ``animmat_format.py`` reads and writes both file versions and can
rewrite a track into its cheapest storage mode, and ``animmat_editor.py`` is a
tkinter GUI with a keyframe timeline and a live preview.

``examples/loading/animated_material_v2`` is the demonstration: four named
materials on one model, three of them driven from a single 484-byte file and the
fourth left alone because no target names it. Between them they use all three
storage modes -- a baked UV scroll, keyframed alpha and emission, a stepped
palette swap and a constant. It also shows why a vertex-colour track does
nothing on a mesh with normals: any NORMAL command re-runs the lighting
equation and overwrites the vertex colour, so lit meshes animate emission
instead.

The preview is only worth having if it agrees with the hardware, so
``tests/animmat_eval`` enforces that: ``gen_vectors.py`` writes an animation and
the values the Python evaluator says each frame should produce, and the test ROM
checks the real runtime against them. It covers all three storage modes, both
interpolation modes and every non-linear lerp. That harness is what surfaced the
double-shift bug above.

**The idle parts of the GPU.** The DS rendering engine has a set of per-frame
fixed-function units that cost nothing once configured. NEA reached most of the
registers already, but through a door narrow enough that the interesting uses
were out of reach. This opens them up.

- **Toon table ramps.** ``NEA_SetupToonShadingTables()`` wrote a hardcoded
  two-band step into a table that holds 32 arbitrary colors. The table is indexed
  by how lit a surface is, which makes it a gradient map and not just a
  cel-shading switch: shading can shift hue as it darkens instead of only losing
  brightness. ``NEA_ToonTableBands()`` builds an N-band cel ramp,
  ``NEA_ToonTableGradient()`` a smooth two-color one,
  ``NEA_ToonTableGradientStops()`` a multi-stop ramp, and ``NEA_ToonTableSet()``
  takes a raw table. All of them are 32 halfwords pushed once, cheap enough to
  rebuild every frame.
- **Environment mapping.** ``NEA_TEXGEN_NORMAL`` was declared but unreachable,
  because nothing ever loaded a texture matrix that would make it produce a
  reflection. ``NEA_ModelSetEnvMap()`` makes ``NEA_ModelDraw()`` load one at the
  only moment it can be right, after the model's transform and before its
  geometry, so the reflection follows the object and not just the camera.
  ``NEA_TextureMatrixEnvMap()`` is the manual form and
  ``NEA_MaterialSetTexGen()`` switches a material's texgen mode at runtime. The
  mesh needs its texture coordinates at the center of the texture: ``obj2dl.py``
  and ``md5_to_dsma.py`` (both the DSMA and NSMW paths) gained ``--envmap-uv``
  for that, which as a side effect makes the display list smaller, since every
  coordinate is then identical and the packer emits one ``TEXCOORD`` command
  instead of one per vertex. On animated models the reflection is anchored to
  the model rather than to each bone, because there is only one texture matrix
  and the display list restores a different bone matrix as it goes; see
  ``NEA_ModelSetEnvMap()`` for what that looks like.
- **Rear plane depth.** ``NEA_ClearDepthSet()`` and ``NEA_ClearDepthGet()`` expose
  a register that was written once at init and never again. It decides how far
  away "nothing" is, and it is what makes the clear bitmap's per-pixel depth
  channel usable: a background with its own depth *occludes* geometry rather than
  merely sitting behind it.
- **Polygon IDs on models.** Edge marking outlines a pixel only where its
  neighbour has a different polygon ID, and ``NEA_ModelDraw()`` never touched the
  polygon format at all, so per-object outlining meant hand-written
  ``NEA_PolyFormat()`` calls. ``NEA_ModelSetPolyID()`` overrides just the ID,
  leaving alpha, lights and culling as the last ``NEA_PolyFormat()`` left them,
  and puts it back afterwards. ``NEA_PolyFormatGet()`` exposes the shadow copy of
  the write-only register this needs. ``NEA_OutliningSetColorAll()`` covers the
  common case of one outline color, and the docs now explain the ID grouping,
  the rear-plane comparison and the antialiasing conflict.
- **Alpha test.** ``NEA_AlphaTestEnable()`` and ``NEA_AlphaTestDisable()`` reach a
  register that was zeroed at init and never touched. Cutout textures can now be
  drawn as opaque polygons: correct depth, no manual sorting, and no polygon IDs
  spent on translucency.
- **Shininess ramps.** ``NEA_ShininessTableSet()`` uploads a custom 128-entry
  table, and ``NEA_SHININESS_STEPPED`` and ``NEA_SHININESS_THRESHOLD`` join the
  four power curves with a banded and a hard-clipped highlight.
- **1-dot polygon depth.** ``NEA_OneDotDepthSet()`` resolves a long-standing TODO.
  The register bypasses the geometry FIFO, so the function drains it first.

New examples under ``examples/effects``: ``toon_ramps``, ``env_mapping``,
``rear_plane_depth``, ``edge_marking`` and ``alpha_test``. ``specular_material``
gained the two new shininess ramps.

**Three enum fixes in NEAPolygon.h.** All three were copy-paste defects that made
the affected values silently do nothing:

- ``NEA_DEPTH_TEST_EQUAL`` was ``(0 << 14)``, the same as
  ``NEA_DEPTH_TEST_LESS``. **This changes behaviour**: code that passed it was
  getting a less-than test and will now get the equal test it asked for. That is
  the point, since depth-equal is what decals and the matching depth tricks need.
- ``NEA_RENDER_ONEA_DOT_POLYS`` was ``(0 << 13)``, the same as
  ``NEA_HIDE_ONEA_DOT_POLYS``. It is now ``(1 << 13)``.
- ``NEA_LIGHT_013``, ``NEA_LIGHT_023`` and ``NEA_LIGHT_123`` all expanded to
  lights 0, 1 and 2. They now name the lights they claim to.

**NEAThread: a background task system.** ``NEA_TaskSubmit()`` runs a unit of
work on a pooled worker thread and calls an optional completion callback on the
main thread during the vertical blank, which is the only place a task's results
may safely reach VRAM. Tasks can report progress, be cancelled, and are torn
down with the engine.

Be clear about what this is for: the threads underneath are BlocksDS cooperative
threads on the ARM9, so there is one CPU, no preemption, and **nothing runs in
parallel**. A task takes as long as the same work run inline. What it buys is
that the game keeps drawing, keeps streaming audio and keeps reading the pad
while the work happens, and that the player can cancel it.

- ``NEA_ThreadSystemReset(workers, stack_size)`` reserves every worker stack up
  front and never grows, so the cost is fixed and visible at startup rather than
  appearing gradually during play. After it succeeds, submitting can only fail
  because too many tasks are live.
- Pass ``NEA_UPDATE_TASKS`` to ``NEA_WaitForVBL()`` to run completion callbacks.
- A **per-frame budget** stops the workers once they have used their slice of
  the frame, so background work costs loading time instead of frame rate.
  ``NEA_ThreadSetFrameBudget(0)`` removes the limit for loading screens.
- Debug builds **name a task that runs several frames without yielding**, which
  turns the classic cooperative-threading mistake from a mysterious frame-rate
  drop into a message identifying the culprit. They also track **stack
  high-water marks** (``NEA_ThreadStackPeak()``) so stack sizes can be chosen
  from measurement.

The asynchronous asset loader is unchanged and still has its own workers; the
two systems coexist.

**Asynchronous asset loading everywhere it was missing.** The three modules
that still blocked the main loop for the whole duration of a filesystem read
now have ``*Async`` loaders alongside their synchronous ones, built on the same
worker-thread + vertical-blank-finalize machinery as the texture and model
loaders:

- **NEAHw2D**: ``NEA_Hw2DBGLoadTiles/Map/Bitmap/GRFFATAsync()``,
  ``NEA_Hw2DOBJLoadGfx/Palette/GRFFATAsync()``,
  ``NEA_Hw2DOBJAssetLoadGfx/Palette/GRFFATAsync()``,
  ``NEA_Hw2DTextCtxMetadataLoadFATAsync()`` and
  ``NEA_Hw2DTextCtxBitmapLoadGRFAsync()``. GRF decompression runs on the worker
  thread too, so only the VRAM copy lands on the main thread. Deleting a
  background, sprite, asset or text context — or calling
  ``NEA_Hw2DSystemEnd()`` — aborts any load aimed at it.
- **NEACollision**: ``NEA_ColMeshLoadFATAsync()``.
- **NEABoneCollision**: ``NEA_BoneCollisionLoadFATAsync()``.
- **NEARichText**: ``NEA_RichTextMetadataLoadFATAsync()``.

NSMW node-skin data was already covered by ``NEA_ModelLoadNSMWFATAsync()``.

**Asynchronous file writing**: ``NEA_FATWriteDataAsync()`` writes a buffer from
the worker thread, so a game can save without freezing the main loop. The write
is atomic: the data goes to ``<path>.temp`` and is only renamed into place once
all of it has reached the filesystem, so a failed or cancelled save leaves the
previous file intact rather than truncated. The caller chooses whether the
engine copies the buffer, takes ownership of it, or writes straight out of it
(``NEA_ASYNC_WRITE_COPY`` / ``_TAKE`` / ``_BORROW``).

The synchronous ``*FAT`` loaders in these modules now share the engine's file
reader instead of each hand-rolling ``fopen``/``fread``, which also means they
check for the read errors they used to ignore.

**Fixed: oversized 2D assets corrupted a neighbouring background.**
``NEA_Hw2DBGLoadTiles()``, ``NEA_Hw2DBGLoadMap()`` and
``NEA_Hw2DBGLoadBitmap()`` copied the whole source into VRAM without checking it
against the background's own allocation. Because the allocator hands out
contiguous blocks, an asset larger than the background ran straight on into the
next layer's tiles or map, which showed up as garbage in an unrelated
background rather than as an error where the mistake was made. The copies are
now clamped to the VRAM the background owns (reported by the new ``gfx_size``
and ``map_size`` fields of ``NEA_Hw2DBG``) and the debug build prints the sizes
involved. ``NEA_Hw2DBGLoadPalette()`` gained the same bound that
``NEA_Hw2DOBJLoadPalette()`` already had, so a 4bpp palette padded out to 256
entries can no longer run past the end of the palette region when loaded into a
non-zero slot.

Version 3.0.0 (2026-08-06)
---------------------------

The headline is a complete 3D rigid body physics engine. Everything else in
this release is engine plumbing that landed alongside it.

**3D rigid body physics (Box3D)** — a fixed-point port of Erin Catto's
`Box3D <https://github.com/erincatto/box3d>`_, with no floating point anywhere
in the simulation. It ships as its own archive, ``libNEA_box3d.a``, so a
project that does not want physics pays nothing for it; ``libNEA.a`` is Nitro
Engine Advanced without it.

- **Shapes**: spheres, capsules, convex hulls and baked triangle meshes, in any
  combination. Hulls and meshes are baked offline by ``obj2dl --collision-b3``
  and ``md5_to_dsma --collision-b3``; there is no run-time builder, because
  running quickhull or building a BVH on a 67 MHz ARM9 is work for the asset
  pipeline.
- **Contacts** that persist across steps with warm starting, gathered into
  islands that fall asleep when they settle.
- **Nine joint types**: distance, revolute, spherical, weld, motor, prismatic,
  parallel and wheel, plus a filter joint that solves nothing and only stops
  two bodies colliding. Each with the springs, motors and limits its type
  allows.
- **Continuous collision**, so a fast body does not tunnel through a wall.
- **Sensors**: shapes that report what overlaps them and resolve nothing,
  including bodies that cross one entirely within a single step.
- **World queries**: ray casts, shape casts and overlap queries against every
  shape type, meshes included.
- **A character mover** (``NEA_Phys3DMover``): a kinematic capsule that is not
  a rigid body, so a player stops fighting the solver for every step, slope and
  ledge. Friction, acceleration, jumping, a ground probe that climbs stairs
  without a step-up hack, and dynamic bodies it can shove.
- **A pool allocator that makes sizing visible**: the world reserves every pool
  up front and seals itself on the first step, so any allocation after that is
  counted and, in the debug build, asserts.
- **Selectable ITCM residency**: the ARM9's 8 KB instruction cache does not hold
  the solver, so the hot routines can be moved into ITCM per group. Print the
  live budget with ``make -f Makefile.blocksds itcm-report``.
- Twelve examples under ``examples/physics/box3d_*``, one per feature, each
  printing its own cost on the bottom screen.

**NSMW (NitroSkin MultiWeight) skinning**: an animated mesh format supporting
two bone weights per vertex, against DSMA's rigid single-weight skinning. The
DS GPU has no hardware skinning, so NSMW uses the same matrix-palette scheme
the retail NSBMD format does.

**Display list improvements**: display lists can now be edited at runtime, and
on a DSi they can be sent with NDMA in GFX FIFO mode — an engine independent of
the legacy DS DMA, so it is safe alongside dual 3D and two-pass rendering. It
is opt-in via ``NEA_DisplayListEnableNDMA()`` and never selected automatically.

**Dirty texture system**: materials can keep their texture in a RAM buffer,
be modified in place, marked with ``NEA_MaterialTexSetDirty()`` and pushed to
VRAM with ``NEA_MaterialTexVramUpdate()``. Palettes gained the same treatment.

**Hardware 2D (NEAHw2D)**: substantial rework, plus corrected text rendering on
bitmap backgrounds and a fix for a 2D OBJ initialization issue.

Version 2.0.0 (2026-03-06)
---------------------------

- **Animated Material System**: keyframe-driven material animation with 13
  track types (alpha, lights, culling, colors, material swap, polygon ID,
  texture scroll/rotate/scale). Exported from Blender via F-Curves, loaded
  at runtime as ``.neaanimmat`` binaries. Integrates with the scene system
  (per-node ``animmat`` pointer applied automatically during scene draw).

- **Scene System**: binary ``.neascene`` format with node hierarchy
  (mesh, camera, trigger, empty), asset tables, material references, and
  parent-children transforms. Supports tag queries, trigger zones with
  enter/exit/tick callbacks, and recursive scene update/draw.

- **Automatic NitroFS scene loading**: ``NEA_SceneLoadFAT()`` now auto-loads
  meshes from the asset table and GRF textures from the material reference
  table. Materials are created, loaded, and bound to mesh nodes automatically.

- **Scene export tool** (``tools/neascene_export/``): converts JSON scene
  descriptions to the binary ``.neascene`` format.

- **Collision system**: multiple collision shapes (AABB, sphere, capsule),
  collision testing between arbitrary shape pairs, and scene-integrated
  trigger zones.

- **Sound module**: spatial 3D sound with distance attenuation and stereo
  panning (powered by Maxmod, enabled with ``NEA_MAXMOD=1``).

- **Multiple texture support**: static and animated meshes support per-submesh
  materials via DLMM format. ``NEA_MaterialClone()`` shares VRAM for
  identical textures with different properties.

- **Texture matrix manipulation**: translate, rotate, and scale textures at
  runtime via ``NEA_TextureMatrix*`` functions. Identity reset prevents
  state leaking between draw calls.

- **Depth buffer mode selection**: switch between Z-buffering and W-buffering.

- **Blender addon (v2.0.0)**: Animated material properties moved to Material
  Properties panel. Visual preview via frame change handler (material swap,
  texture scroll, alpha). Scene node properties, per-bone collision editing,
  trigger zone overlays, and integrated conversion tool execution.


- All public API prefixes changed from ``NE_`` to ``NEA_``.

Version 0.15.5 (2026-01-23)
---------------------------

- Ensure final position is returned on dry run with cursor. @Jonko
- Add x and y axis independent sprite scaling. @BanceDev
- Optimize physics update and camera use code. @Kuratius
- Fix off-by-one error that was causing junk to be copied into rendered
  materials. @Jonko
- Rendering font with translucent textures (A5PAL8 and A3PAL32) has been
  modified. Now the new pixels overwrite the previous pixels regardless of the
  value being overwritten. @Jonko
- The height of texture buffers is now trimmed instead of being expanded to a
  valid DS texture size. @Jonko

Version 0.15.4 (2025-07-16)
---------------------------

- Allow the number of rich text fonts to be specified dynamically. @Jonko
- Optionally return cursor position from dry run & allow indenting rich text.
  @Jonko

Version 0.15.3 (2025-03-26)
---------------------------

- The devkitARM makefiles have been removed as they only work with old versions
  of devkitARM, which aren't supported by its maintainers. The code and examples
  of Nitro Engine Advanced will probably need changes to work with current devkitARM.
- The GRF files used in an example have been updated, they were built before the
  format was changed.
- The build scripts for assets have been modified to use the value of the
  environment variables ``BLOCKSDS`` and ``BLOCKSDSEXT`` if they are found.
- Build library with debug symbols to help debug applications that use it.

Version 0.15.2 (2025-01-15)
---------------------------

- Fix `ar` binary used to build the library.
- Update documentation.
- Fix build of examples in devkitARM.
- Improve setup instructions.

Version 0.15.1 (2024-12-23)
---------------------------

- Install licenses with the rest of the library.
- Clarify some comments in examples.

Version 0.15.0 (2024-12-01)
---------------------------

- Allow users to specify the transformation matrix of a model manually.
- Fix a devkitARM Makefile in an example. @W3SLAV

Version 0.14.0 (2024-09-10)
---------------------------

- Use ``stdint.h`` types instead of libnds types.
- Update BlocksDS makefiles.
- Update libdsf to v0.1.3.
- Fix some memory leaks/use-after-free bugs in the RichText module.

Version 0.13.0 (2024-06-08)
---------------------------

- Define BlocksDS as the main toolchain to use with Nitro Engine Advanced.
- Simplify build and installation instructions of the library.
- Update ``md5_2_dsma`` to correctly export the base animation of models.
- Stop using ``NEA_RGB5`` in the examples, this format is discouraged.
- Optimize copy of ``NEA_RGB5`` textures to VRAM.

Version 0.12.0 (2024-03-30)
---------------------------

- Deprecate ``NEA_ShadingEnable()``. The name was misleading. All examples that
  use it have stopped calling it.
- For sprites, add a way to specify texture coordinates inside the material to
  use a small part of the texture instead of the whole texture:
  ``NEA_SpriteSetMaterialCanvas()``. This is now used in the sprites example.
- Stop using global variables in most examples. Instead, the rendering functions
  get values through the arguments of ``NEA_ProcessArg()`` and
  ``NEA_ProcessDualArg()``.
- Don't expect palette objects when loading GRF files if the file doesn't
  contain a palette.
- Allow loading BMFont fonts from RAM, not just nitroFS. Add an example of
  using rich text fonts from RAM.
- Added a function to reset the rich text font system. Also add a function to
  return the size that a specific text string would have if drawn.
- Add shadow volume example.
- Build some functions that do lots of multiplications as ARM to increase
  performance.
- Fix compilation with devkitARM.
- Fix linker invocation for C++ with BlocksDS.
- Update libDSF to version 0.1.2, with some speed improvements.
- Relicensed libDSF under "Zlib OR MIT" to simplify licensing with Nitro Engine Advanced.
- Some minor documentation improvements.

Version 0.11.0 (2024-03-02)
---------------------------

- Added a rich text system. This allows the user to draw non-monospaced text,
  and it's based on `BMFont <https://www.angelcode.com/products/bmfont/>`_. An
  example has been added to show how to use it to render text. It can render
  text by using one quad per character, or by rendering to a texture which is
  then drawn as a single quad.
- Fix "properties" typo. This involves renaming some functions, but compatibilty
  definitions have been added.
- Update calls to GRF libnds functions after their prototypes have changed.
- Cleanup setup of dual 3D modes that involve setting up 2D sprites to be used
  as a framebuffer.
- Now the size of a sprite is set to the size of the material it is assigned. It
  can be modified later as usual.

Version 0.10.0 (2024-01-28)
---------------------------

- Create variants of ``NEA_Process()`` and ``NEA_ProcessDual()`` that can pass an
  argument to the screen draw callbacks.

- Add function to load textures in GRF format (BlocksDS only).

- Add functions to load compressed textures (Texel 4x4 format).

- Add examples to load assets from the filesystem, both in binary and GRF
  function.

- Cleanup code style: Use stdint types, turn some functions ``static inline``...

- Remove unneeded call to ``DC_FlushAll()``.

- ``obj2dl``: Improve error message. Fix vertical coordinates being flipped.

- ``img2ds``: Deprecate it except for generating ``DEPTHBMP`` binaries.

- The makefile for BlocksDS now installs the tools to ``/opt/blockds/external``,
  not only the library and headers.

- Migrate all examples to using ``grit`` instead of ``img2ds``.

Version 0.9.1 (2023-11-12)
--------------------------

- Update Makefiles to build on BlocksDS.

- Fix functions used to load assets from FAT.

Version 0.9.0 (2023-10-19)
--------------------------

- Introduce two new dual 3D modes. They are resilient and they will always show
  the same output on both screens even if the framerate drops. This isn't the
  case with the previous dual 3D mode.

- Fix 2D projection used to display 3D sprites. The Y coordinate didn't work
  correctly for numbers close to 192. This means that an ugly hack to apply an
  offset to the texture coordinates of 2D polygons is no longer needed.

- Fix initialization of the library. Sometimes, depending on the loader, the
  game would start in a different time in the screen rendering cycle. This would
  swap the images of the screens until the framerate dropped when loading
  assets, for example.

- The code that switches between screens in dual 3D mode has been more reliable.
  Nitro Engine Advanced now swaps screens after they are actually drawn, not in the
  vertical blanking interrupt handler, when it switched every frame even if no
  new frame had been drawn by the game.

- Switch a lot of assert() in the library into permanent runtime checks. Several
  functions now return error codes instead of not returning any value.

- Use safe DMA copy functions if the libnds of the toolchain provides them (they
  are only available in BlocksDS at the moment).

- The library now supports sending display lists to the GPU in different ways to
  work around a hardware bug in the ARM9 DMA when it is set to GFX FIFO mode.

- Fix debug build of the library.

- Fix build of the NFlib template with devkitPro libraries.

- Update examples and add some more, particularly about comparisons between dual
  3D modes.

Version 0.8.2 (2023-04-20)
--------------------------

- Decouple mesh objects from model objects. This simplifies cloning models.
  Previously it was needed to preserve the original object as long as you wanted
  to use the clones. Now, it can be deleted and Nitro Engine Advanced won't free the mesh
  until all clones have been deleted.

- Support vertex color commands in ``obj2dl``. This can't be used at the same
  time as normals.

- Improve examples. A script has been added to convert all assets used by the
  examples. Also, the NFlib example has been updated to work with upstream
  NFlib.

- Support BlocksDS.

- A few minor fixes.

Version 0.8.1 (2022-11-10)
--------------------------

Models and materials:

- Improve support of specular properties of materials and add an example of how
  to use it for metalic objects.

- Fix material cloning:

  - Copy material properties apart from just the texture.

  - Assign palettes to materials instead of textures, so that a single texture
    can have multiple textures. You can load a texture to a material, clone the
    material, and assign a different palette to the cloned material.

- Support loading compressed textures and add an example of how to load them.
  Note that ``img2ds`` doesn't support this format yet. Until that support is
  added, compressed texture support should be considered experimental.

- Add example of how to use NFlib at the same time as Nitro Engine Advanced. NFlib is a
  library that has support for 2D graphics, which complements the 3D hardware
  support of Nitro Engine Advanced.

Other:

- Rename a few functions for consistency. The old names have been kept for
  compatibility, but they will be removed.

- Added some enumerations to help remember the names to be used as function
  arguments.

- The general-purpose allocator has been improved a lot to support compressed
  textures. This is needed due to the special way to load them to VRAM.
  Extensive tests for the allocator have also been added.

- Many internal changes to simplify the code and remove dependencies on libnds
  functions.

Version 0.8.0 (2022-10-21)
--------------------------

Models and materials:

- Add support for MD5 animated models (thanks to `dsma library
  <https://codeberg.org/SkyLyrac/dsma-library>`__): Introduce tool
  ``md5_to_dsma`` to convert them to a format that Nitro Engine Advanced can use.

- Add support for OBJ static models: Introduce tool obj2dl to convert them to a
  format that Nitro Engine Advanced can use.

- Introduce tool ``img2ds`` to convert images in many popular formats (PNG, JPG,
  BMP, etc) to DS textures (PNG is still recommended over other formats, as it
  supports alpha better than other formats).

- Drop support for MD2 models (static or animated).

- Remove NDS Model Exporter, Nitro Texture Converter, md2_to_bin and md2_to_nea.
  The animation system has been refactored (but NEA files don't work anymore, so
  you need to update your code anyway).

General:

- Huge cleanup of code style of the codebase.

- Cleanup of all examples. Add the original assets and textures used in all
  examples to the repository, along scripts to convert them to the formats used
  by Nitro Engine Advanced.

- Implement a better way to have debug and release builds of the library.

Notes:

- You can still use textures converted with Nitro Texture Converter or NDS Model
  Exporter, and you can still use any model exported with NDS Model Exporter or
  ``md2_to_bin``. However, support for NEA files has been removed (it had awful
  performance, and it was just a bad way to do things), so any file converted by
  ``md2_to_nea`` won't work anymore.

- The reason to replace most tools is that several people had issues building
  them. All the new tools are written in Python, so they don't need to be
  compiled.

Version 0.7.0 (2019-6-14)
-------------------------

- Pushed to GitHub.

- Major cleanup of code.

- Clarify license.

- Reworked tools to build under Linux and Windows.

Version 0.6.1 (2011-9-1)
------------------------

- Fixed identation in all code. Now it isn't a pain to read it (not as much as
  before, :P). Also, a few warnings fixed (related to libnds new versions).

Version 0.6.0 (2009-6-30)
-------------------------

- The functions used to modify textures and palettes now return a pointer to the
  data so that you can modify them easily.

- Each material can have different properties (amient, diffuse...). You can set
  the default ones, the properties each new material will have, and then you
  can set each material's properties individually.

- New texture and palette allocation system, it is faster and better.
  Defragmenting functions don't work now, but I'll fix them for the next
  version.

- Added a debug system. You can compile Nitro Engine Advanced in "debug mode" and it will
  send error messages to the function you want. Once you have finished debugging
  or whatever, just recompile Nitro Engine Advanced without debug mode.

- Window system renamed to Sprite system. You can set a rotation and a scale for
  each one.

- The most important thing... The animation system has been improved, and now
  animated models are drawn using linear interpolation (you can disable it,
  anyway).

- As a result, I've modified the converters, so you'll have to convert yout
  animated models again.

Version 0.5.1 (2009-1-28)
-------------------------

- Minor bugfixes.

Version 0.5.0 (2009-1-5)
------------------------

- Text system and camera system optimized. New functions for the camera system.

- ``NEA_TextPrintBox()`` and ``NEA_TextPrintBoxFree()`` slightly changed. They can
  limit the text drawn to a number of characters set by the coder.

- Some functions made internal. Don't use them unless you know what you are
  doing.

- Fixed (?) at least the 2D projection.

- HBL effects fixed.

- Touch test functions.

- ``NEA_UPDATE_INPUT`` removed.

- It now supports any BMP size, and BMP with 4 bits of depth.

- Arrays made pointers, so there is more memory free when you are not using
  Nitro Engine Advanced. You can also configure the number of objects of each systems you
  are going to use.

- ``NEA_TextPalette`` replaced by ``NEA_Palette``.

- You can clone materials to use the same texture with different colors. This
  doesn't have the problems of cloning models.

- Added functions to remove all palettes and textures.

- Fixed ``NEA_End()``.

- NE can free all memory used by it, and the coder can tell NE how much memory
  to use.

- Texture drawing system improved a bit.

- ``NEA_PolyFormat()`` simplified.

- Some bugfixes, code reorganized, define lists converted into enums.

- Clear bitmap supported, this is used to display an bitmap as rear plane. Each
  pixel can have different depth. This needs 2 VRAM banks to work.

- Solved some problems with 2D system and culling.

- Nomad ``NDS_Texture_Converter`` is no longer included, if you want it, look for it
  in Google.

- Added Nitro Texture Converter, made by me. Open source, and it exports various
  levels of alpha in the textures that can handle it. It does only accept PNG
  files.

- NE now accepts any texture size. ``NEA_SIZE_XXX`` defines removed as they are
  not needed now.

- Added a couple of examples.

Version 0.4.2 (2008-12-14)
--------------------------

- Fixed 2D system (textures were displaying wrong on 2D quads) and text system
  (paletted textures sometimes were drawn without palette).

- Modified ``MD2_2_NEA``, ``MD2_2_BIN`` and ``bin2nea`` to work in linux. Thanks
  to nintendork32.

- Added a couple of examples.

Version 0.4.1 (2008-12-12)
--------------------------

- Lots of bugfixes. Specially, UV coordinates swapping fixed.

- Added a function to draw on RGBA textures ^_^.

- Fixed ``MD2_2_NEA`` and ``MD2_2_BIN``. You'll have to convert again your
  models.

- Updated to work with latest libnds. There is a define in case you want to use
  an older version.

Version 0.4.0 (2008-10-15)
--------------------------

- Added ``MD2_2_NEA`` (converts an MD2 model into a NEA file that can used by
  Nitro Engine Advanced) and ``MD2_2_BIN`` (Converts the first frame of an MD2 model
  into a display list). Display lists created by them are really optimized.

- Updated ``DisplayList_Fixer``. Now it can remove normal commands too.

- Added a text system. It can use fonts of any size. ^^

- Added some simple API functions (buttons, check boxes, radio buttons and slide
  bars).

- Fixed 2D projection.

- Removed some internal unused functions to save space, and made 'inline' some
  of the rest.

- Functions that used float parameters modified so they use integers now. You
  can still use some wrappers if you want to use floats. This will let the
  compiler try to optimize the code.

- Animated and static models are now the same. You can move, rotate, etc them
  with the same functions.

- Now, you can 'clone' models so you can save a lot of RAM if they are repeated.

- Renamed lots of model functions. Take a look at new examples or documentation.

- ``NEA_Color`` struct removed (I don't even know why I created it...).

- Examples updated to work with last version and added examples of clonning
  models, API and text system.

- libnds' console is not inited with Nitro Engine Advanced. You will have to init it
  yourself with ``NEA_InitConsole()`` or libnds' functions.

Version 0.3.0 (2008-9-16)
-------------------------

- Support for animated models (NEA format) and a program to make a new NEA file
  from many models (in bin format).

- 2D over 3D system. You can draw easily quads (with or without texture) as if
  they were drawn using 2D.

- Basic physics engine (gravity, friction and collitions). It does only support
  bounding boxes for now.

- Added a function to delete all models, animated or not.

- Window system, very simple. I will make some API functions in next versions.

- Nitro Engine Advanced compiled as a library to include it easier in projects and save
  space.

- Examples folder organized a bit and added some new examples.

- Nitro Engine Advanced is now licensed under the BSD license.

Version 0.2.0 (2008-8-31)
-------------------------

- Added effects like fog and shading, functions to load BMP files and convert
  them in textures and more examples.

Version 0.1 (2008-8-24)
-----------------------

- Includes 2 examples, documentation, tools to export models from the PC, the
  license and full source.
