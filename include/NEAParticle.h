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
/// camera by default. The per-emitter axis-aligned flag in the NPE file
/// disables billboarding and uses world X/Y for the quad basis instead,
/// which suits effects like waterfalls or vertical flame planes.
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

/// @}

#endif // NEA_PARTICLE_H__
