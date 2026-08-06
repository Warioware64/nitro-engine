// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Warioware64
//
// This file is part of Nitro Engine Advanced

#ifndef NEA_PHYSICS3D_H__
#define NEA_PHYSICS3D_H__

/// @file   NEAPhysics3D.h
/// @brief  3D rigid body physics: convex shapes, meshes, contacts, sleeping.
///
/// A fixed-point port of Erin Catto's Box3D, wrapped in Nitro Engine Advanced's
/// conventions. Spheres, capsules and convex hulls against each other and
/// against baked triangle meshes; persistent contacts with warm starting;
/// islands that fall asleep when they settle.
///
/// @section link Linking
///
/// Box3D lives in its **own archive**, because it is roughly three times the
/// size of the rest of the engine. `libNEA.a` is Nitro Engine Advanced without
/// it. A project that wants physics asks for it by name, in its Makefile,
/// before including Makefile.example:
///
///     LIBS := -lNEA_box3d -lNEA -lnds9 -lc
///
/// A project that does not name it links exactly as much Box3D as before: none.
///
/// @section relation Relation to the other physics modules
///
/// NEA has three older physics layers and this replaces none of them:
///
///   - [NEAPhysics.h](NEAPhysics.h) -- velocity/impulse objects, AABB and
///     sphere only, no rotation. Cheap and entirely adequate for a platformer.
///   - [NEARigidBody.h](NEARigidBody.h) -- OBB bodies solved on the ARM7 over
///     FIFO. Rotation, but one shape type and no convex hulls.
///   - [NEACollision.h](NEACollision.h) -- shapes and narrow phase with no
///     solver, for queries and triggers.
///
/// Use this module when you need convex hulls, stacking that holds, or contact
/// events. It runs on the ARM9 and costs ARM9 time; the other three are still
/// the right answer for simpler scenes.
///
/// @section rawapi Reaching Box3D directly
///
/// This header wraps the common path. It deliberately does not wrap all ~140
/// public Box3D functions -- contact events, custom filtering, pre-solve
/// callbacks, surface materials, mass data and the shape-level collision
/// routines are reached by including <box3d/box3d.h> and passing the ids that
/// the functions below return. `NEA_Phys3DBody` exposes its `b3BodyId` for
/// exactly this. The two styles share all state and can be mixed freely.
///
/// @section timestep The time step is fixed
///
/// dt is a compile-time constant (B3_NEA_STEP_HZ, 60 Hz) so the solver's
/// inverse-dt and soft-constraint coefficients fold into literals instead of
/// costing a hardware divide per body per sub-step. `NEA_Phys3DWorldStepI` takes
/// a sub-step count, not a duration. If your game runs at 30 fps, step twice.
///
/// **B3_NEA_STEP_HZ is a display rate, not a clock rate**, and it is 60 on
/// every DS and DSi. It is worth saying because the question comes up: it is
/// the reciprocal of dt, so raising it *shortens* the simulated time step
/// rather than making anything faster. Setting it to a CPU frequency would
/// advance the world by nanoseconds per step.
///
/// The knob that spends CPU on quality is `subStepCount`, and it is runtime.
///
/// @section dsi On DSi
///
/// There is no DSi profile, and the reason is measured rather than assumed.
/// A DSi clocks its ARM9 at 133 MHz against the DS's 67, which looks like an
/// obvious win to spend on sub-steps -- but `isDSiMode()` reports the
/// *console*, not the clock: it returns true for a DS-only ROM booted on a
/// DSi, which still runs at 67 MHz. Gating a heavier default on it would slow
/// down exactly the case it was meant to speed up.
///
/// Measured with this example under melonDS 1.1, peak step cost at 8 awake
/// bodies was 7094 us as a DS-only ROM, 7094 us with unit code 2, and 7125 us
/// as a TWL ROM with DSi access flags -- identical within noise, with
/// SCFG_CLK's ARM9 TWL bit already set in all three. So pick `subStepCount`
/// from your own frame budget, not from the console.

#include "NEAModel.h"

#include "box3d/box3d.h"

#ifdef __cplusplus
extern "C" {
#endif

/// @defgroup physics3d 3D rigid body physics (Box3D)
///
/// Fixed-point Box3D: spheres, capsules, convex hulls and baked triangle
/// meshes, persistent contacts, islands and sleeping. Requires linking
/// `-lNEA_box3d`.
///
/// @{

// =========================================================================
// Types
// =========================================================================

/// A body handle, and the binding between a physics body and a model.
///
/// Allocated by NEA_Phys3DBodyCreate from a pool sized at world creation, and
/// installed as the Box3D body's user data -- which is how NEA_Phys3DSyncModels
/// gets from a move event back to your model without searching.
///
/// `id` is the raw Box3D handle. Pass it to anything in <box3d/box3d.h>.
typedef struct NEA_Phys3DBody {
    b3BodyId id;         ///< The Box3D body. Valid for the raw API.
    NEA_Model *model;    ///< Model to drive, or NULL. See NEA_Phys3DBodySetModel().
    void *userData;      ///< Yours. Untouched by this module.
} NEA_Phys3DBody;

/// World creation parameters.
///
/// Initialize with NEA_Phys3DDefaultWorldDef() and adjust, the same way Box3D's
/// own definitions work. Never zero-initialize one: `box3d.internalValue` is a
/// validity check that b3CreateWorld enforces.
///
/// @section sizing Sizing
///
/// `box3d.capacity` is not a hint. Every pool -- bodies, shapes, contacts,
/// islands, the broad-phase trees and the per-step solver scratch -- is
/// reserved from it before the first step, and the allocator installed by
/// NEA_Phys3DWorldCreate reports any allocation that happens afterwards.
/// Over-estimate: the cost of a spare slot is bytes, the cost of a missing one
/// is an allocation in the middle of a frame.
///
/// One field is not just a count: `box3d.capacity.meshContactCount` says how
/// many of `contactCount` touch a triangle mesh. A mesh contact costs several
/// times what a convex one does, because it carries one manifold per contact
/// *normal* rather than one in total, so leaving this at zero in a scene with a
/// mesh level is the one sizing mistake that shows up as a mid-frame
/// allocation rather than as a slightly small pool.
typedef struct NEA_Phys3DWorldDef {
    /// The Box3D world definition: gravity, sleeping, contact tuning and the
    /// capacities. Set `box3d.capacity` to the size of your scene.
    b3WorldDef box3d;

    /// Bytes reserved for the whole world, in one block taken at creation.
    ///
    /// Everything Box3D allocates comes from here. Start from the default,
    /// print NEA_Phys3DWorldGetMemoryUsage() once your scene is built, and set
    /// this to that number plus headroom.
    ///
    /// **A world with a triangle mesh needs about 24 KB more than the default
    /// 64 KB**, before any of its bodies. A mesh contact clips up to 32
    /// triangles against a shape inside the step, and that scratch comes from
    /// the per-step arena, which b3CreateWorld grows from 4 KB to 23,605 bytes
    /// as soon as `box3d.capacity.meshContactCount` is non-zero.
    int32_t poolBytes;

    /// Box hull slots to reserve for NEA_Phys3DBodyAddBox().
    ///
    /// A convex hull's data must outlive every shape referencing it, so the
    /// hull a box shape is built from has to live somewhere permanent. This
    /// module keeps a pool of them. 0 means "one per shape in
    /// `box3d.capacity`". Costs 440 bytes each, and only boxes use them --
    /// spheres and capsules do not.
    int maxBoxHulls;

    /// Bodies to reserve NEA_Phys3DBody records for. 0 means the body count in
    /// `box3d.capacity`.
    int maxBodies;

    /// Solver sub-steps per step. 0 selects B3_NEA_DEFAULT_SUBSTEPS (4).
    ///
    /// **This is the knob that trades CPU for simulation quality**, and the
    /// only one — the time step itself is fixed (see the note below). More
    /// sub-steps means stiffer stacks and less penetration at a near-linear
    /// cost. 2 is a reasonable floor for a CPU-bound scene; above 8 the
    /// returns are small.
    ///
    /// `NEA_Phys3DWorldStep()` uses this; `NEA_Phys3DWorldStepI(n)` overrides
    /// it for one step.
    int subStepCount;
} NEA_Phys3DWorldDef;

// =========================================================================
// World
// =========================================================================

/// A world definition with the defaults filled in.
///
/// Gravity is (0, -9.8, 0), sleeping is on, and the capacities are small.
/// Set `box3d.capacity` before creating anything real.
NEA_Phys3DWorldDef NEA_Phys3DDefaultWorldDef(void);

/// Create the physics world.
///
/// There is one world. B3_MAX_WORLDS is 1 on this platform: a second call
/// without an intervening NEA_Phys3DWorldEnd() fails.
///
/// Takes `poolBytes` from the heap in a single block and installs it as
/// Box3D's allocator, so that everything the simulation owns is contiguous and
/// nothing fragments the heap your models and textures live in.
///
/// @param def World definition, from NEA_Phys3DDefaultWorldDef().
/// @return 0 on success, -1 on failure (no memory, or a world already exists).
int NEA_Phys3DWorldInit(const NEA_Phys3DWorldDef *def);

/// Destroy the world, every body in it, and release the pool.
void NEA_Phys3DWorldEnd(void);

/// Is there a world?
bool NEA_Phys3DWorldExists(void);

/// The Box3D world handle, for the raw API in <box3d/box3d.h>.
b3WorldId NEA_Phys3DWorldGetId(void);

/// Advance the simulation by one 1/60 s step.
///
/// @param subStepCount Solver sub-steps. 4 is the recommended default and what
///                     NEA_Phys3DWorldStep() uses; 2 trades accuracy for time
///                     in a CPU-bound scene. Clamped to at least 1.
///
/// Call this yourself, or set NEA_UPDATE_PHYS3D in NEA_WaitForVBL() and let the
/// engine do it -- not both.
void NEA_Phys3DWorldStepI(int subStepCount);

/// Advance the simulation by one step, using the world's sub-step count.
///
/// The count comes from NEA_Phys3DWorldDef::subStepCount, or from the last
/// NEA_Phys3DSetSubStepCount() call.
void NEA_Phys3DWorldStep(void);

/// Set gravity (f32).
void NEA_Phys3DWorldSetGravityI(int32_t x, int32_t y, int32_t z);

/// Set gravity (float).
#define NEA_Phys3DWorldSetGravity(x, y, z) \
    NEA_Phys3DWorldSetGravityI(floattof32(x), floattof32(y), floattof32(z))

/// Copy every moved body's transform into its bound model.
///
/// Walks the move events the last step published, which is one entry per awake
/// body that actually moved -- so a settled scene costs nothing. Call it after
/// stepping and before drawing. NEA_UPDATE_PHYS3D does this for you.
void NEA_Phys3DSyncModels(void);

// =========================================================================
// Memory
// =========================================================================

/// Bytes of the pool handed out so far (high-water mark).
///
/// The number to size `poolBytes` from. Print it once after building your
/// scene and once after a few hundred steps; if the second is larger, something
/// is allocating during simulation and NEA_Phys3DWorldGetLateAllocCount() will
/// say how often.
int32_t NEA_Phys3DWorldGetMemoryUsage(void);

/// Bytes the pool was created with.
int32_t NEA_Phys3DWorldGetMemoryCapacity(void);

/// Bytes served from the heap because the pool ran out.
///
/// Non-zero means `poolBytes` is too small. The simulation is still correct --
/// this module never refuses an allocation, because Box3D's callers write
/// through the pointer without checking -- but the contiguity guarantee is
/// gone.
int32_t NEA_Phys3DWorldGetOverflowBytes(void);

/// Allocations made after the world was sealed.
///
/// The world seals itself on the first NEA_Phys3DWorldStepI(). Non-zero means
/// the simulation allocated after it started running, which is the thing this
/// module exists to make visible: a capacity in `box3d.capacity` is too small.
/// Under NEA_DEBUG each one also fires an assert naming the size.
int NEA_Phys3DWorldGetLateAllocCount(void);

/// Peak bytes of per-step solver scratch used so far.
///
/// Compare against NEA_Phys3DWorldGetStackCapacity(): if it is close, a busier
/// frame will exceed the stack and allocate.
int32_t NEA_Phys3DWorldGetStackUsage(void);

/// Bytes of per-step solver scratch reserved, sized from `box3d.capacity`.
int32_t NEA_Phys3DWorldGetStackCapacity(void);

/// Mesh contact manifolds discarded **in the last step** because a shape
/// touched the level along more distinct normals than the port preallocates.
///
/// Not a running total: the collide pass zeroes it every step, so read it after
/// NEA_Phys3DUpdate() and keep your own peak if you want one.
///
/// A non-zero reading means contacts were thrown away, and the ones kept are
/// the deepest -- so the visible symptom is a body sinking into a crease rather
/// than anything obviously wrong. A level that trips this wants coarser
/// geometry, or fewer triangles meeting at the places bodies land.
///
/// Zero in steady state is the thing to check when bringing a level up.
int NEA_Phys3DWorldGetMeshManifoldDropCount(void);

// =========================================================================
// Bodies
// =========================================================================

/// Create a body.
///
/// The body has no shape and therefore no mass yet. Add one with
/// NEA_Phys3DBodyAddBox() / AddSphere() / AddCapsule(); each updates the mass.
/// A static or kinematic body can also carry a level, with
/// NEA_Phys3DBodyAddMesh().
///
/// @param type b3_staticBody, b3_kinematicBody or b3_dynamicBody.
/// @param x (x, y, z) Initial position (f32).
/// @param y (x, y, z) Initial position (f32).
/// @param z (x, y, z) Initial position (f32).
/// @return The body, or NULL if the body pool is full.
NEA_Phys3DBody *NEA_Phys3DBodyCreateI(b3BodyType type,
                                      int32_t x, int32_t y, int32_t z);

/// Create a body (float position).
#define NEA_Phys3DBodyCreate(type, x, y, z) \
    NEA_Phys3DBodyCreateI(type, floattof32(x), floattof32(y), floattof32(z))

/// Destroy a body and every shape on it. Its model is left alone.
void NEA_Phys3DBodyDelete(NEA_Phys3DBody *body);

/// Attach a box shape, centred on the body origin.
///
/// The hull comes from the world's box hull pool -- see
/// NEA_Phys3DWorldDef::maxBoxHulls.
///
/// @param body The body.
/// @param hx (hx, hy, hz) Half-extents (f32).
/// @param hy (hx, hy, hz) Half-extents (f32).
/// @param hz (hx, hy, hz) Half-extents (f32).
/// @param density Mass per cubic unit (f32). Ignored for static bodies.
/// @return 0 on success, -1 if the hull pool is full.
int NEA_Phys3DBodyAddBoxI(NEA_Phys3DBody *body,
                          int32_t hx, int32_t hy, int32_t hz, int32_t density);

/// Attach a box shape (float).
#define NEA_Phys3DBodyAddBox(body, hx, hy, hz, density) \
    NEA_Phys3DBodyAddBoxI(body, floattof32(hx), floattof32(hy), \
                          floattof32(hz), floattof32(density))

/// Attach a sphere shape centred on the body origin.
///
/// @param body The body.
/// @param radius Radius (f32).
/// @param density Mass per cubic unit (f32).
/// @return 0 on success.
int NEA_Phys3DBodyAddSphereI(NEA_Phys3DBody *body, int32_t radius,
                             int32_t density);

/// Attach a sphere shape (float).
#define NEA_Phys3DBodyAddSphere(body, radius, density) \
    NEA_Phys3DBodyAddSphereI(body, floattof32(radius), floattof32(density))

/// Attach a capsule shape along the Y axis, centred on the body origin.
///
/// @param body The body.
/// @param radius Radius (f32).
/// @param halfHeight Half the distance between the two cap centres (f32).
/// @param density Mass per cubic unit (f32).
/// @return 0 on success.
int NEA_Phys3DBodyAddCapsuleI(NEA_Phys3DBody *body, int32_t radius,
                              int32_t halfHeight, int32_t density);

/// Attach a capsule shape (float).
#define NEA_Phys3DBodyAddCapsule(body, radius, halfHeight, density) \
    NEA_Phys3DBodyAddCapsuleI(body, floattof32(radius), \
                              floattof32(halfHeight), floattof32(density))

/// Attach a baked triangle mesh -- a level -- to a static or kinematic body.
///
/// @section blob The blob
///
/// `blob` is a `.b3mesh` baked by `obj2dl.py --collision-b3`. It is **not
/// copied**: the shape keeps this pointer, exactly as a hull shape does, so it
/// must outlive every shape referencing it and must not move. One blob can back
/// several shapes at different scales.
///
/// It must be **8-byte aligned**. `b3MeshData` opens with a `uint64_t` and the
/// ARM9 reads one with LDRD, which faults on a 4-byte-aligned address. Two ways
/// to get that right, and one that looks right and is not:
///
///   - `obj2dl.py --collision-b3-c level` writes `level_b3mesh.c` / `.h`
///     declaring the array `__attribute__((aligned(8)))`. Drop the `.c` in your
///     `source/` directory. **This is the ROM-embedded path.**
///   - `NEA_FATLoadData("level.b3mesh")` for NitroFS. `malloc` returns 8-byte
///     aligned blocks, so a loaded blob is fine -- keep the pointer and free it
///     after NEA_Phys3DWorldEnd(), not before.
///   - **Not** a `.b3mesh` dropped in `data/`: BlocksDS's bin2c emits
///     `aligned(4)`, which is one byte of luck away from a data abort.
///
/// This function refuses a misaligned or invalid blob rather than only
/// asserting on it, because a stale asset is exactly the input that reaches a
/// release ROM.
///
/// @section mass Mass
///
/// A mesh shape has **zero mass and zero inertia** -- a triangle soup has no
/// volume to integrate -- so it belongs on a static or kinematic body, and this
/// refuses a dynamic one. That is upstream Box3D's behaviour, not a limitation
/// of the port.
///
/// @section sizing Sizing
///
/// Set `box3d.capacity.meshContactCount` before creating the world: it is how
/// many shapes may touch the level at once, and a mesh contact carries one
/// manifold per contact *normal* where a convex contact carries one in total.
/// Leaving it at zero and then adding a mesh is the one sizing mistake that
/// shows up as an allocation in the middle of a frame, so this warns rather
/// than letting it happen quietly.
///
/// @section colmesh Where is NEA_Phys3DBodyAddColMesh?
///
/// There is deliberately no function taking a `.colmesh` directly. Bridging one
/// means building a bounding volume hierarchy, and that is work for the asset
/// pipeline rather than for a 67 MHz ARM9 at load time. `obj2dl.py` bakes the
/// hierarchy offline and reports what it produced:
///
///     python3 tools/obj2dl/obj2dl.py --input level.obj --output level.bin
///         --texture 128 128 --collision-b3 --collision-b3-c level
///
/// `--collision` is a different thing and still means what it always did: a
/// `.colmesh` for the older NEACollision module. The two can be asked for
/// together.
///
/// `--collision-b3-scale` is worth setting. The solver's tolerances are
/// absolute and want 1 unit to be about 1 metre, which is usually not the scale
/// a display list is authored at -- positions there are v16 and top out at 8
/// units.
///
/// @param body   A static or kinematic body.
/// @param blob   The `.b3mesh`, 8-byte aligned, outliving the shape.
/// @param sx (sx, sy, sz) Per-axis scale (f32). Pass f32 1.0 for none. A
///               negative component reflects the mesh and flips triangle
///               winding; each is floored at B3_MIN_SCALE.
/// @param sy (sx, sy, sz) Per-axis scale (f32).
/// @param sz (sx, sy, sz) Per-axis scale (f32).
/// @return 0 on success, -1 if the body is dynamic or the blob is NULL,
///         misaligned or not a valid mesh.
int NEA_Phys3DBodyAddMeshI(NEA_Phys3DBody *body, const void *blob,
                           int32_t sx, int32_t sy, int32_t sz);

/// Attach a baked triangle mesh (float scale).
#define NEA_Phys3DBodyAddMesh(body, blob, sx, sy, sz) \
    NEA_Phys3DBodyAddMeshI(body, blob, floattof32(sx), floattof32(sy), \
                           floattof32(sz))

// =========================================================================
// Bone shapes -- hitboxes for an animated model
// =========================================================================
//
// A `.b3col` is a table of per-bone shapes written by
// `md5_to_dsma.py --collision --collision-b3` from a `.md5collimesh`. It is the
// Box3D counterpart of `.boncol`, which feeds the older NEABoneCollision
// module; the two are independent and a project can use either or both.
//
// **The intended shape of the thing.** A mesh shape is static-only and a single
// rigid body cannot deform, so a hitbox set is not several shapes on one body
// -- it is **one kinematic body per bone**, each carrying one shape, with its
// transform driven each frame from the animated skeleton. That costs one body
// per entry, which is why the table is per-bone and why the joint index is
// stored: it is what you look the bone's current transform up by.
//
// Unlike a `.b3mesh` there is no `uint64_t` in the layout, so a `.b3col` is
// fine 4-byte aligned and may go through `bin2c` into a project's `data/`
// directory.

/// Shape types in the binary `.b3col` format.
typedef enum {
    NEA_B3COL_TYPE_NONE    = 0, ///< The bone has no shape; skip it.
    NEA_B3COL_TYPE_SPHERE  = 1, ///< param1 is the radius.
    NEA_B3COL_TYPE_CAPSULE = 2, ///< param1 radius, param2 half height.
    NEA_B3COL_TYPE_BOX     = 3, ///< param1/2/3 are the half extents.
} NEA_Phys3DBoneShapeType;

/// How many entries a `.b3col` has, or 0 if it is not one.
int NEA_Phys3DBoneShapeCount(const void *b3col);

/// The joint index entry `index` describes, or -1 if there is no such entry.
///
/// This is the index into the model's DSA skeleton, which is what you drive the
/// body's transform from.
int NEA_Phys3DBoneShapeJoint(const void *b3col, int index);

/// The shape type of entry `index`, or NEA_B3COL_TYPE_NONE if there is none.
NEA_Phys3DBoneShapeType NEA_Phys3DBoneShapeGetType(const void *b3col, int index);

/// Attach entry `index` of a `.b3col` to a body, at its bone-local offset.
///
/// The offset and orientation in the table are **bone-local** and are baked
/// into the shape here, so the body's own transform is the bone's transform
/// with nothing added. A capsule lies along whatever axis the `.md5collimesh`
/// gave it, which is +Y unless the file names one.
///
/// A `NEA_B3COL_TYPE_BOX` entry takes a slot from the world's box hull pool,
/// exactly as NEA_Phys3DBodyAddBoxI does and for the same reason -- `b3Shape`
/// keeps the caller's hull pointer rather than copying it. Count the bone boxes
/// into `NEA_Phys3DWorldDef::maxBoxHulls`.
///
/// @param body    The body to attach to, normally one kinematic body per bone.
/// @param b3col   The `.b3col` table.
/// @param index   Entry to attach, in [0, NEA_Phys3DBoneShapeCount).
/// @param density Mass per cubic unit (f32). Ignored for a non-dynamic body.
/// @return 0 on success, -1 if the table, the index or the shape is unusable.
///         A NEA_B3COL_TYPE_NONE entry is not an error and attaches nothing,
///         returning 0.
int NEA_Phys3DBodyAddBoneShapeI(NEA_Phys3DBody *body, const void *b3col,
                                int index, int32_t density);

/// Attach a bone shape (float density).
#define NEA_Phys3DBodyAddBoneShape(body, b3col, index, density) \
    NEA_Phys3DBodyAddBoneShapeI(body, b3col, index, floattof32(density))

/// Bind a model to a body. NEA_Phys3DSyncModels() then drives its transform.
///
/// Pass NULL to unbind. The model is not owned and is not freed with the body.
void NEA_Phys3DBodySetModel(NEA_Phys3DBody *body, NEA_Model *model);

/// Set position and clear rotation (f32).
void NEA_Phys3DBodySetPositionI(NEA_Phys3DBody *body,
                                int32_t x, int32_t y, int32_t z);

/// Set position and clear rotation (float).
#define NEA_Phys3DBodySetPosition(body, x, y, z) \
    NEA_Phys3DBodySetPositionI(body, floattof32(x), floattof32(y), floattof32(z))

/// Read the body position (f32) into x/y/z. Any pointer may be NULL.
void NEA_Phys3DBodyGetPositionI(const NEA_Phys3DBody *body,
                                int32_t *x, int32_t *y, int32_t *z);

/// Set linear velocity in units per second (f32).
void NEA_Phys3DBodySetVelocityI(NEA_Phys3DBody *body,
                                int32_t x, int32_t y, int32_t z);

/// Set linear velocity (float).
#define NEA_Phys3DBodySetVelocity(body, x, y, z) \
    NEA_Phys3DBodySetVelocityI(body, floattof32(x), floattof32(y), floattof32(z))

/// Read linear velocity (f32) into x/y/z. Any pointer may be NULL.
void NEA_Phys3DBodyGetVelocityI(const NEA_Phys3DBody *body,
                               int32_t *x, int32_t *y, int32_t *z);

/// Apply a force at the centre of mass, in units of mass*length/s^2 (f32).
///
/// A force accumulates over the step; for an instantaneous change use
/// NEA_Phys3DBodyApplyImpulseI(). Wakes the body.
void NEA_Phys3DBodyApplyForceI(NEA_Phys3DBody *body,
                               int32_t x, int32_t y, int32_t z);

/// Apply a force at the centre of mass (float).
#define NEA_Phys3DBodyApplyForce(body, x, y, z) \
    NEA_Phys3DBodyApplyForceI(body, floattof32(x), floattof32(y), floattof32(z))

/// Apply an impulse at the centre of mass, in units of mass*length/s (f32).
///
/// Changes the velocity immediately by impulse/mass. Wakes the body.
void NEA_Phys3DBodyApplyImpulseI(NEA_Phys3DBody *body,
                                 int32_t x, int32_t y, int32_t z);

/// Apply an impulse at the centre of mass (float).
#define NEA_Phys3DBodyApplyImpulse(body, x, y, z) \
    NEA_Phys3DBodyApplyImpulseI(body, floattof32(x), floattof32(y), floattof32(z))

/// Apply a torque about the body's centre of mass (f32). Wakes the body.
void NEA_Phys3DBodyApplyTorqueI(NEA_Phys3DBody *body,
                                int32_t x, int32_t y, int32_t z);

/// Apply a torque (float).
#define NEA_Phys3DBodyApplyTorque(body, x, y, z) \
    NEA_Phys3DBodyApplyTorqueI(body, floattof32(x), floattof32(y), floattof32(z))

/// Set friction and restitution on every shape currently on the body (f32).
///
/// Both are dimensionless and normally in [0, 1]. Shapes added afterwards get
/// the Box3D defaults, so call this last.
void NEA_Phys3DBodySetMaterialI(NEA_Phys3DBody *body, int32_t friction,
                                int32_t restitution);

/// Set friction and restitution (float).
#define NEA_Phys3DBodySetMaterial(body, friction, restitution) \
    NEA_Phys3DBodySetMaterialI(body, floattof32(friction), \
                               floattof32(restitution))

/// Is the body awake? A sleeping body costs almost nothing per step.
bool NEA_Phys3DBodyIsAwake(const NEA_Phys3DBody *body);

/// Wake a body, or force it to sleep.
void NEA_Phys3DBodySetAwake(NEA_Phys3DBody *body, bool awake);

/// How many bodies are awake. The number that predicts your step time.
int NEA_Phys3DWorldGetAwakeBodyCount(void);

// =========================================================================
// Queries
// =========================================================================
//
// Two of Box3D's five world queries, wrapped in this module's conventions:
// f32 arguments, an `I` suffix and a float macro, and the world implied rather
// than passed. These are the two a game reaches for -- "what am I pointing at"
// and "what is near me".
//
// The other three (b3World_CastRay with a callback, b3World_OverlapShape,
// b3World_CastShape) are deliberately *not* wrapped. They take a b3-typed
// callback whose fixed-point arguments a wrapper cannot usefully convert
// without also inventing a second callback type, and a project that needs them
// is already including box3d.h for b3CreateWheelJoint and friends -- every
// physics example here does. Wrapping them would add surface without removing
// a dependency.

/// What a ray hit. Distances and points are f32, as everything else here.
typedef struct
{
    /// Did the ray hit anything?
    bool hit;

    /// The world point of the hit (f32).
    int32_t x, y, z;

    /// The unit surface normal there (f32).
    int32_t nx, ny, nz;

    /// How far along the ray, as an f32 fraction in [0, 1]. Multiply by the
    /// ray's own length for a distance.
    int32_t fraction;

    /// The user material id at the hit point, from the shape's material.
    uint64_t userMaterialId;

    /// Which triangle, for a mesh shape. -1 for every other shape type.
    int triangleIndex;

    /// BVH nodes and leaves the query touched. Diagnostic: on a DS this is
    /// what tells you whether a per-frame ray is affordable.
    int nodeVisits, leafVisits;
} NEA_Phys3DRayHit;

/// Cast a ray from `origin` along `direction` and report the nearest hit.
///
/// `direction` is **not** normalized: its length is the length of the ray, and
/// `out->fraction` is relative to it. Firing (0, -100, 0) from the camera and
/// reading back a fraction of 0.03 means the ground is 3 units down.
///
/// Costs one BVH descent per body type. Safe to call every frame; check
/// `out->nodeVisits` if you want to know what it actually cost.
///
/// @return true if something was hit, which is also `out->hit`.
bool NEA_Phys3DRayCastI(int32_t ox, int32_t oy, int32_t oz,
                        int32_t dx, int32_t dy, int32_t dz,
                        NEA_Phys3DRayHit *out);

/// Cast a ray (float).
#define NEA_Phys3DRayCast(ox, oy, oz, dx, dy, dz, out) \
    NEA_Phys3DRayCastI(floattof32(ox), floattof32(oy), floattof32(oz), \
                       floattof32(dx), floattof32(dy), floattof32(dz), out)

/// Called once per body found by NEA_Phys3DOverlapBoxI.
///
/// Return false to stop the query early.
typedef bool (*NEA_Phys3DOverlapFn)(NEA_Phys3DBody *body, void *context);

/// Visit every body with a shape whose bounds overlap the given box.
///
/// Bounds, not exact shapes -- this is the broad-phase answer, so it can report
/// a body whose shape does not quite reach into the box. That is the right
/// trade for the things a box query is for (what is near the player, what is in
/// the blast radius); use `b3World_OverlapShape` directly when it is not.
///
/// A body with several shapes in the box is reported once per shape.
///
/// @return the number of times the callback was invoked.
int NEA_Phys3DOverlapBoxI(int32_t lx, int32_t ly, int32_t lz,
                          int32_t ux, int32_t uy, int32_t uz,
                          NEA_Phys3DOverlapFn fn, void *context);

/// Overlap a box (float).
#define NEA_Phys3DOverlapBox(lx, ly, lz, ux, uy, uz, fn, context) \
    NEA_Phys3DOverlapBoxI(floattof32(lx), floattof32(ly), floattof32(lz), \
                          floattof32(ux), floattof32(uy), floattof32(uz), \
                          fn, context)

// =========================================================================
// Character mover
// =========================================================================
//
// A kinematic character controller: a capsule that is **not** a rigid body,
// moved by depenetration and shape casting rather than by the solver.
//
// A player made of a dynamic body has to fight the solver for every step,
// slope and ledge, and loses -- it slides down ramps, tips over, bounces off
// crates and picks up spin, and no amount of friction and damping fixes it,
// because the solver is doing exactly what it is supposed to. This does none
// of that: it decides where the capsule goes and then puts it there.
//
// It touches the world in three directions. It is **blocked** by every shape
// its filter accepts. It **pushes** dynamic bodies it walks into. It is pushed
// by nothing -- being pushed is your job, through
// NEA_Phys3DMoverSetVelocityI().
//
// A mover is a value you own, not a world resource: there is no pool and
// nothing to destroy. Put one in your player struct.
//
// This is Erin Catto's own mover sample (samples/mover.cpp) in this module's
// conventions, and it is the one piece of Stage 4 that has no counterpart in
// raw Box3D -- b3World_CollideMover and b3World_CastMover are queries, and the
// loop that turns them into a character is the caller's. Here it is written
// out. Everything below is f32, as everything else in this header.

/// Shapes one mover may be told to pass through.
#define NEA_PHYS3D_MOVER_MAX_IGNORE 4

/// Tuning for a mover. Get one from NEA_Phys3DDefaultMoverDef() and adjust.
typedef struct NEA_Phys3DMoverDef {
    /// Capsule radius (f32). Default 0.3.
    int32_t radius;

    /// Half the distance between the two cap centres (f32). Default 0.5, so
    /// the capsule is 1.6 units tall overall.
    int32_t halfHeight;

    /// Ground speed cap (f32, units/s). Default 6.
    int32_t maxSpeed;

    /// Below this, horizontal velocity is zeroed outright (f32). Default 0.01.
    /// Without it a mover creeps forever on the last bit of a stick deflection.
    int32_t minSpeed;

    /// Friction floor (f32). Friction removes `max(speed, stopSpeed) *
    /// friction * dt` per step, so this is what stops a slow mover promptly
    /// instead of asymptotically. Default 1.
    int32_t stopSpeed;

    /// Acceleration (f32, 1/s). Default 30.
    int32_t accelerate;

    /// Ground friction (f32, 1/s). Default 4.
    int32_t friction;

    /// Downward acceleration (f32, units/s^2). Default 15 -- higher than
    /// gravity, because a character that falls at 9.8 feels floaty.
    int32_t gravity;

    /// Upward velocity NEA_Phys3DMoverJump() applies (f32). Default 5.
    int32_t jumpSpeed;

    /// Run the ground probe. Default true.
    ///
    /// The probe is one downward ray per frame driving a soft spring, and it is
    /// what rides stairs and slopes with no step-up hack and no walkable-slope
    /// threshold. It also keeps the capsule *off* the floor, which matters more
    /// here than upstream: without it the floor contributes a plane every
    /// frame, the solver pushes out by B3_LINEAR_SLOP, and in Q12 that is a
    /// real 0.0049 units of upward motion injected 60 times a second.
    ///
    /// Turn it off for a flying or swimming character, where the ray is waste.
    bool pogo;

    /// Derive the post-move velocity by clipping against the collision planes
    /// rather than from the distance actually travelled. Default true.
    ///
    /// True is almost always right: it stops the mover inheriting the
    /// depenetration push as velocity and launching off whatever it was
    /// resting against.
    bool clipVelocity;

    /// Query filter for the depenetration pass, the sweep and the ground probe.
    /// Defaults to b3DefaultQueryFilter()'s bits.
    uint64_t categoryBits, maskBits;

    /// Vertical offset from the capsule centre to the bound model's origin
    /// (f32). Default -(halfHeight + radius), which puts a model whose origin
    /// is between its feet on the ground.
    int32_t modelOffsetY;
} NEA_Phys3DMoverDef;

/// A character mover.
///
/// The fields above the line are yours: read them every frame, and write them
/// when you mean it. Everything below is internal and will move.
typedef struct NEA_Phys3DMover {
    /// Capsule centre in world space (f32). Read every frame to place a camera.
    int32_t x, y, z;

    /// Velocity (f32). Read it, and write it to launch or knock back --
    /// an explosion is `v += impulse`, not a force.
    int32_t vx, vy, vz;

    /// Was the ground probe in contact after the last step?
    bool onGround;

    /// Facing, in brad (32768 per circle). Set by NEA_Phys3DMoverStepYawI()
    /// and used to orient the bound model.
    int16_t yaw;

    /// Planes the last step depenetrated against. Diagnostic.
    int planeCount;

    /// Gauss-Seidel iterations summed over the slide loop. Diagnostic.
    int solverIterations;

    /// Slide-loop iterations actually run, 1 to 5. Diagnostic, and worth
    /// putting on screen: pinned at 5 means the mover never reached its target,
    /// which is what a jammed character looks like from the outside.
    int castIterations;

    /// Model to drive, or NULL. See NEA_Phys3DMoverSetModel().
    NEA_Model *model;

    /// Yours. Untouched by this module.
    void *userData;

    // --- internal ---
    NEA_Phys3DMoverDef def;
    int32_t pogoVelocity;
    b3CollisionPlane planes[B3_NEA_MAX_MOVER_PLANES];
    b3ShapeId planeShapes[B3_NEA_MAX_MOVER_PLANES];
    b3Vec3 planePoints[B3_NEA_MAX_MOVER_PLANES];
    b3ShapeId ignore[NEA_PHYS3D_MOVER_MAX_IGNORE];
    int ignoreCount;
} NEA_Phys3DMover;

/// Default tuning. Never zero-initialize a NEA_Phys3DMoverDef -- a zero radius
/// is not a mover.
NEA_Phys3DMoverDef NEA_Phys3DDefaultMoverDef(void);

/// Set a mover up at a world position. Zeroes its velocity and its state.
void NEA_Phys3DMoverInitI(NEA_Phys3DMover *mover, const NEA_Phys3DMoverDef *def,
                          int32_t x, int32_t y, int32_t z);

/// Set a mover up (float).
#define NEA_Phys3DMoverInit(mover, def, x, y, z) \
    NEA_Phys3DMoverInitI(mover, def, floattof32(x), floattof32(y), floattof32(z))

/// Advance a mover by one step, steering by a heading.
///
/// `yaw` is a brad angle; forward and right come from the libnds sine table, so
/// no basis is built, nothing is normalized and nothing is divided. The mover
/// keeps the yaw and orients its bound model with it.
///
/// `forwardThrottle` and `strafeThrottle` are f32 in [-1, 1] -- what a d-pad or
/// a stick gives.
///
/// Call **after** NEA_Phys3DUpdate(): the mover queries the world, and it wants
/// the world the step just produced.
void NEA_Phys3DMoverStepYawI(NEA_Phys3DMover *mover, int yaw,
                             int32_t forwardThrottle, int32_t strafeThrottle);

/// Advance a mover, steering by a heading (float throttles).
#define NEA_Phys3DMoverStepYaw(mover, yaw, fwd, strafe) \
    NEA_Phys3DMoverStepYawI(mover, yaw, floattof32(fwd), floattof32(strafe))

/// Advance a mover along an explicit basis. What NEA_Phys3DMoverStepYawI()
/// calls.
///
/// `forward` and `right` are world directions (f32); neither has to be unit
/// length or perpendicular to the other, and the vertical component of each is
/// ignored -- a camera-relative basis can be passed straight in.
void NEA_Phys3DMoverStepI(NEA_Phys3DMover *mover,
                          int32_t fx, int32_t fy, int32_t fz,
                          int32_t rx, int32_t ry, int32_t rz,
                          int32_t forwardThrottle, int32_t strafeThrottle);

/// Jump, if the mover is on the ground.
/// @return false if it was not, in which case nothing happened.
bool NEA_Phys3DMoverJump(NEA_Phys3DMover *mover);

/// Move a mover without sweeping. Clears onGround and the ground spring, so
/// the next step re-probes rather than assuming the old contact.
void NEA_Phys3DMoverSetPositionI(NEA_Phys3DMover *mover, int32_t x, int32_t y, int32_t z);

/// Move a mover without sweeping (float).
#define NEA_Phys3DMoverSetPosition(mover, x, y, z) \
    NEA_Phys3DMoverSetPositionI(mover, floattof32(x), floattof32(y), floattof32(z))

/// Set a mover's velocity outright.
void NEA_Phys3DMoverSetVelocityI(NEA_Phys3DMover *mover, int32_t x, int32_t y, int32_t z);

/// Set a mover's velocity (float).
#define NEA_Phys3DMoverSetVelocity(mover, x, y, z) \
    NEA_Phys3DMoverSetVelocityI(mover, floattof32(x), floattof32(y), floattof32(z))

/// Bind a model to a mover, or NULL to unbind.
///
/// The mover writes the model's matrix itself, at the end of each step.
/// NEA_Phys3DSyncModels() cannot do it: that walks the step's move events, and
/// a mover has no body and produces none.
void NEA_Phys3DMoverSetModel(NEA_Phys3DMover *mover, NEA_Model *model);

/// Make a mover pass through one shape -- a platform it is riding, a door being
/// opened, its own trigger volume.
///
/// Up to NEA_PHYS3D_MOVER_MAX_IGNORE of them; further calls are ignored.
void NEA_Phys3DMoverIgnoreShape(NEA_Phys3DMover *mover, b3ShapeId shapeId);

/// Forget every ignored shape.
void NEA_Phys3DMoverClearIgnored(NEA_Phys3DMover *mover);

/// Shapes whose plane batch filled B3_NEA_MAX_MOVER_PLANES **in the last
/// b3World_CollideMover call**.
///
/// Not a running total, and not cleared by the mover: b3Collide zeroes it at
/// the top of the next step, so read it right after stepping your movers.
///
/// A non-zero reading means a shape had more collision planes than the port
/// preallocates and the rest were never computed. Unlike a dropped contact,
/// which is a body settling slowly into a crease, a dropped mover plane is a
/// surface the character is never pushed away from -- so the symptom is walking
/// through a wall, at the one moment the geometry was busiest. Zero in steady
/// state is the thing to check when bringing a level up.
int NEA_Phys3DWorldGetMoverPlaneDropCount(void);

/// Calls to b3Body_SetTransform over the life of the world.
///
/// **Cumulative**, unlike every other counter here, because a teleport is a
/// rare event and the question is whether it happened at all.
///
/// It is here because a teleport is not free and nothing else says so: the
/// body's shapes re-enter the broad phase, re-pair on the next step, form
/// contacts and re-form their island, and in a world sized from a settled scene
/// that is where a mid-step allocation comes from. A rising
/// NEA_Phys3DWorldGetLateAllocCount() next to a non-zero reading here is
/// explained rather than a leak -- the memory is reused. Without this the two
/// look identical.
int NEA_Phys3DWorldGetTeleportCount(void);

// =========================================================================
// Engine integration
// =========================================================================

/// Step the world and sync every bound model, in one call.
///
/// What NEA_WaitForVBL() runs for NEA_UPDATE_PHYS3D. libNEA.a reaches it
/// through a **weak reference** -- the same mechanism NEA_RigidBodySync and
/// NEA_SoundUpdateAll use -- so the flag does nothing at all unless a project
/// links libNEA_box3d.a and creates a world. The core library never acquires an
/// undefined Box3D symbol, which is what keeps the two archives independent.
///
/// Uses the sub-step count from NEA_Phys3DSetSubStepCount(), or the last count
/// passed to NEA_Phys3DWorldStepI().
void NEA_Phys3DUpdate(void);

/// Set the sub-step count NEA_UPDATE_PHYS3D uses. Clamped to at least 1.
void NEA_Phys3DSetSubStepCount(int subStepCount);

/// @}

#ifdef __cplusplus
}
#endif

#endif // NEA_PHYSICS3D_H__
