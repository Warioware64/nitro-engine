// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Warioware64
//
// This file is part of Nitro Engine Advanced

#ifndef NEA_PARTICLE_H__
#define NEA_PARTICLE_H__

#include <nds.h>

/// @file   NEAParticle.h
/// @brief  Particle system (NPE: Nitro Particle Entity).

/// @defgroup particle_system Particle system
///
/// Runtime that updates and draws particle emitters from compact binary
/// `.npe` files. Designed in the spirit of Nintendo's SPL: an emitter spawns
/// short-lived particles that follow a simple physics model (initial
/// position/velocity inside a box+cone, plus constant gravity and a per-frame
/// drag), and their color and size animate from RGBA/size keyframes sampled
/// over each particle's life.
///
/// Particles are drawn as textured quads, billboarded against the active
/// camera by default. The per-emitter flags in the NPE file change that:
///
/// - **axis-aligned** drops billboarding and uses world X/Y for the quad
///   basis, which suits waterfalls or vertical flame planes.
/// - **stretch** lengthens the quad along the direction of travel, so a fast
///   particle reads as a streak and tapers as it slows.
/// - **sprite sheet** takes each frame from a cell of a `cols` x `rows` grid,
///   advancing with the particle's age at `sheet_fps`.
/// - **additive** is an approximation, because the DS 3D engine has no
///   additive blend -- DISP3DCNT offers alpha blending on or off and nothing
///   else. The runtime weights each particle's alpha by its brightness
///   instead, so dark particles contribute almost nothing and bright ones
///   dominate, which is what additive looks like for sparks and flames.
///
/// An emitter spreads its particles over eight polygon IDs so that overlapping
/// ones actually blend; see NEA_ParticleEmitterSetPolyID() for why that is
/// needed and when to move the range.
///
/// One `.npe` file = one emitter (e.g. "fire", "smoke", "explosion"). Author
/// `.npe` files with `tools/npe_editor/`.
///
/// Usage:
///
/// @code
/// NEA_ParticleSystemReset(0); // 0 = default cap
/// NEA_ParticleSystemSetCamera(my_camera); // needed for billboarding
///
/// NEA_ParticleEmitter *fire = NEA_ParticleEmitterCreate();
/// NEA_ParticleEmitterLoadFAT(fire, "fx/fire.npe");
/// // The NPE file references a GRF texture path; if you'd rather bind your
/// // own loaded material, call NEA_ParticleEmitterSetMaterial() afterwards.
/// NEA_ParticleEmitterSetPosition(fire, 0, 0, 0);
/// NEA_ParticleEmitterPlay(fire);
///
/// while (1)
/// {
///     NEA_WaitForVBL(NEA_UPDATE_PARTICLES);
///     NEA_Process(DrawScene);
/// }
///
/// // Inside DrawScene, after the camera is applied:
/// // NEA_ParticleEmitterDraw(fire);
/// @endcode
///
/// @{

#define NEA_DEFAULT_PARTICLE_EMITTERS 16  ///< Default max emitters.
#define NEA_PARTICLE_DEFAULT_POOL     128 ///< Particle pool size when the NPE asks for 0.

/// First of the eight polygon IDs an emitter cycles through by default.
///
/// Not zero: scene geometry usually draws with ID 0, and a particle sharing an
/// ID with what is behind it cannot blend over it. Not near 63 either, which is
/// the rear plane, or 61 and 62, which the GUI uses.
#define NEA_PARTICLE_DEFAULT_POLY_ID  8

/// Opaque handle representing one emitter.
typedef struct NEA_ParticleEmitter NEA_ParticleEmitter;

// NEAMain.h includes NEAParticle.h after NEATexture.h / NEAModel.h /
// NEACamera.h / NEAFAT.h, so the typedefs (NEA_Material, NEA_Camera,
// NEA_Model, NEA_AsyncFile) are visible here without forward declarations.

/// Resets the particle system and sets the maximum number of emitters.
///
/// Called automatically by NEA_Init3D() and friends with a sensible default,
/// so you only need to call it explicitly to override the cap.
///
/// @param max_emitters Max number of emitters. If `< 1`, uses
///                     #NEA_DEFAULT_PARTICLE_EMITTERS.
/// @return 0 on success, -1 on out-of-memory.
int NEA_ParticleSystemReset(int max_emitters);

/// Stops every emitter and frees all particle-system memory.
void NEA_ParticleSystemEnd(void);

/// Sets the camera used to compute billboard orientation.
///
/// Required for any emitter without the axis-aligned flag. Pass the same
/// camera you apply with NEA_CameraUse() during your draw function.
///
/// @param cam Camera to read `from`/`to`/`up` from.
void NEA_ParticleSystemSetCamera(NEA_Camera *cam);

/// Creates an empty emitter.
///
/// @return Handle, or NULL if the system has no free slot.
NEA_ParticleEmitter *NEA_ParticleEmitterCreate(void);

/// Deletes an emitter and frees its particle pool.
///
/// The bound material is **not** deleted (it may be shared); delete it
/// yourself with NEA_MaterialDelete() if appropriate.
///
/// @param emitter Emitter handle.
void NEA_ParticleEmitterDelete(NEA_ParticleEmitter *emitter);

/// Loads NPE data from RAM into an emitter.
///
/// Parses the header, copies the parameter block, allocates the particle
/// pool, and resets emission state. If the NPE file carries a non-empty
/// material name, the loader looks it up via NEA_MaterialFindByName() and
/// binds it as the emitter's material -- so it is enough to set the name on
/// the material (NEA_MaterialSetName()) when you load its texture, and the
/// emitter will pick it up automatically. NEA_ParticleEmitterSetMaterial()
/// can still be used to override afterwards.
///
/// **This overload cannot bounds-check.** The parser walks the file using
/// counts stored inside it, and with no length there is nothing to compare
/// them against, so a truncated image reads past the end of the buffer. That is
/// fine for data linked into the ROM, which is as trustworthy as the code
/// beside it. For anything read from a file, use
/// NEA_ParticleEmitterLoadSize() or NEA_ParticleEmitterLoadFAT(), which know
/// how many bytes they actually have.
///
/// @param emitter Emitter handle.
/// @param data Pointer to the NPE file bytes in RAM.
/// @return 1 on success, 0 on error.
int NEA_ParticleEmitterLoad(NEA_ParticleEmitter *emitter, const void *data);

/// Loads NPE data from the filesystem (synchronous).
///
/// @param emitter Emitter handle.
/// @param path Path to the `.npe` file.
/// @return 1 on success, 0 on error.
int NEA_ParticleEmitterLoadFAT(NEA_ParticleEmitter *emitter, const char *path);

/// Asynchronously loads an NPE file into an emitter.
///
/// Mirrors the other async loaders (textures, models). The finalize step
/// runs on the main thread inside NEA_AsyncProcess() once the data is in
/// RAM, so it is safe to call NEA_ParticleEmitterPlay() only after the
/// returned handle reaches `NEA_ASYNC_DONE`.
///
/// @param emitter Emitter handle.
/// @param path Path to the `.npe` file.
/// @return Async handle, or NULL on error.
NEA_AsyncFile *NEA_ParticleEmitterLoadFATAsync(
                                NEA_ParticleEmitter *emitter, const char *path);

/// Binds a material to an emitter for drawing.
///
/// Replaces any material previously set. The material must outlive the
/// emitter (or be replaced before being deleted).
///
/// @param emitter Emitter handle.
/// @param material Material to use. NULL disables texturing.
void NEA_ParticleEmitterSetMaterial(NEA_ParticleEmitter *emitter,
                                    NEA_Material *material);

/// Sets the emitter origin in world space (f32 fixed point).
///
/// Particle initial positions are sampled inside an axis-aligned box around
/// this origin, with the box extents taken from the NPE file.
///
/// @param emitter Emitter handle.
/// @param x World X (f32).
/// @param y World Y (f32).
/// @param z World Z (f32).
void NEA_ParticleEmitterSetPosition(NEA_ParticleEmitter *emitter,
                                    int x, int y, int z);

/// Attaches the emitter to a model so it follows its position each frame.
///
/// Pass NULL to detach. While attached, NEA_ParticleEmitterSetPosition() is
/// ignored.
///
/// @param emitter Emitter handle.
/// @param model Model to follow, or NULL to detach.
void NEA_ParticleEmitterAttachToModel(NEA_ParticleEmitter *emitter,
                                      NEA_Model *model);

/// Starts (or resumes) continuous emission for the emitter.
///
/// For burst-mode NPE files this only re-enables update; spawn new bursts
/// with NEA_ParticleEmitterEmitBurst().
///
/// @param emitter Emitter handle.
void NEA_ParticleEmitterPlay(NEA_ParticleEmitter *emitter);

/// Pauses the emitter without clearing live particles.
///
/// @param emitter Emitter handle.
void NEA_ParticleEmitterPause(NEA_ParticleEmitter *emitter);

/// Stops the emitter and clears all live particles.
///
/// @param emitter Emitter handle.
void NEA_ParticleEmitterStop(NEA_ParticleEmitter *emitter);

/// Spawns up to `count` particles right now.
///
/// Works whether the emitter is playing or paused. Particles exceeding the
/// emitter's pool capacity are silently dropped.
///
/// @param emitter Emitter handle.
/// @param count Particles to spawn.
void NEA_ParticleEmitterEmitBurst(NEA_ParticleEmitter *emitter, int count);

/// Advances every emitter by one frame.
///
/// Called automatically when #NEA_UPDATE_PARTICLES is passed to
/// NEA_WaitForVBL(). Performs continuous-rate spawning, particle ageing,
/// position/velocity integration with gravity and drag, and key-framed
/// sampling of color and size over each particle's life.
void NEA_ParticleUpdateAll(void);

/// Draws every live particle of one emitter.
///
/// Call this inside your draw function (the callback you pass to
/// NEA_Process()/NEA_ProcessArg()), after the camera is applied. The
/// modelview matrix at call time is preserved.
///
/// @param emitter Emitter handle.
void NEA_ParticleEmitterDraw(NEA_ParticleEmitter *emitter);

/// Returns the number of live particles in an emitter.
///
/// @param emitter Emitter handle.
/// @return Number of alive particles (0..pool size).
int NEA_ParticleEmitterAliveCount(const NEA_ParticleEmitter *emitter);

/// Clears any emitter's attachment to a model that is about to be deleted.
///
/// Internal. NEA_ModelDelete() calls this before the model's memory goes away,
/// the same way it cancels asynchronous loads that would write into it. An
/// emitter keeps a raw pointer to the model it follows and reads its position
/// every frame, so without this the next update would read freed memory.
///
/// @param model Model being deleted.
void __NEA_ParticleDetachModel(NEA_Model *model);

/// Loads emitter parameters from an NPE image of a known size.
///
/// The same as NEA_ParticleEmitterLoad(), but bounded: the parser walks the
/// file using counts stored inside it, so it needs the length to refuse a
/// truncated or corrupt one instead of reading past the end. Use this for
/// anything that did not come linked into the ROM.
///
/// @param emitter Emitter to fill in.
/// @param data Pointer to the NPE image.
/// @param size Bytes actually available at that pointer.
/// @return 1 on success, 0 on error.
int NEA_ParticleEmitterLoadSize(NEA_ParticleEmitter *emitter, const void *data,
                                size_t size);

/// Sets the first of the eight polygon IDs this emitter cycles through.
///
/// Particles are translucent, and the hardware refuses to blend a translucent
/// polygon over a translucent pixel carrying the *same* polygon ID. An emitter
/// therefore spreads its particles across eight consecutive IDs so that
/// overlapping ones layer instead of the first one winning outright.
///
/// The reason to change the base is collision with the rest of the scene: a
/// particle whose ID matches the geometry behind it will not blend over that
/// geometry. Give an emitter its own eight if something in the scene is drawn
/// with IDs in the default range.
///
/// Note that eight is a mitigation rather than a guarantee. Particles that are
/// eight pool slots apart share an ID again, so a very dense effect can still
/// have occasional pairs that refuse to blend.
///
/// @param emitter Emitter.
/// @param base First polygon ID of the eight (0 - 56).
void NEA_ParticleEmitterSetPolyID(NEA_ParticleEmitter *emitter, int base);

/// @}

#endif // NEA_PARTICLE_H__
