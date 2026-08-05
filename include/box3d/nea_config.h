// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Erin Catto        (original Box3D)
// Copyright (c) 2026 Warioware64       (Nitro Engine Advanced fixed-point port)
//
// This file is part of Nitro Engine Advanced

#ifndef BOX3D_NEA_CONFIG_H__
#define BOX3D_NEA_CONFIG_H__

/// @file   nea_config.h
/// @brief  Compile-time configuration of the Box3D fixed-point port.
///
/// This header is force-included ahead of Box3D's own config.h through
/// BOX3D_USER_CONFIG. It shrinks the desktop-oriented defaults down to
/// something a 67 MHz ARM946E-S with 4 MB of RAM can actually host, and
/// selects the fixed-point math layer.
///
/// See source/box3d/b3fixed.h for the number formats themselves.

// =========================================================================
// Port identification
// =========================================================================

/// Set when Box3D is built as part of Nitro Engine Advanced. Ported source
/// files test this to select fixed-point behaviour over the upstream float
/// path, so that a diff against upstream Box3D stays readable.
#define B3_NEA_PORT 1

/// The whole point of the port: no floats anywhere in the simulation.
#define B3_FIXED_POINT 1

// =========================================================================
// Hard memory limits
// =========================================================================

// Upstream declares `b3World b3_worlds[B3_MAX_WORLDS]` as a static array with
// B3_MAX_WORLDS == 128. sizeof(b3World) is several kilobytes, so the default
// would statically exceed the DS's entire 4 MB of main RAM before a single
// body exists. One world is all a DS game needs.
#define B3_MAX_WORLDS 1

// The constraint graph exists to let worker threads solve disjoint constraint
// sets in parallel. There is one core available here, so every constraint goes
// into the single overflow colour and the per-colour body bitsets vanish.
// B3_OVERFLOW_INDEX is (B3_GRAPH_COLOR_COUNT - 1) == 0.
#define B3_GRAPH_COLOR_COUNT 1

// Single-threaded, so a single worker context.
#define B3_MAX_WORKERS 1

// Convex hull limits. Upstream allows 128 vertices/faces/edges per hull, which
// costs a lot of stack in the SAT routines and is far more detail than a DS
// game will ever author. 32 comfortably covers boxes, prisms and low-poly
// collision proxies.
#define B3_MAX_HULL_VERTICES 32
#define B3_MAX_HULL_FACES 32
#define B3_MAX_HULL_EDGES 32

// Follows B3_MAX_HULL_VERTICES upstream; restated so the two stay in step.
#define B3_MAX_SHAPE_CAST_POINTS B3_MAX_HULL_VERTICES

// Shape id hashing. Upstream reserves 22 bits per shape (4M shapes) to make
// b3ShapePairKey trivially collision-free. A DS world holds hundreds of
// shapes, so 12 bits (4096) is generous and leaves 40 bits of child index.
#define B3_SHAPE_POWER 12

// =========================================================================
// Triangle mesh limits
// =========================================================================
//
// Three bounds on what one shape-versus-mesh contact may cost. Upstream has a
// bound on the first two and none at all on the third; all three are load
// bearing here, because each one sizes a pool that b3CreateWorld reserves
// before the first step.

/// Manifolds one mesh contact may carry, and therefore the block-allocator
/// size class a mesh contact allocates from.
///
/// This is **not** the triangle count. b3ComputeMeshManifolds collides the
/// overlapping triangles one at a time, then clusters the results by normal
/// similarity -- a manifold joins an existing cluster when its normal and its
/// triangle normal are both within cos 0.996 (about 5 degrees) of it -- and
/// allocates one manifold per *cluster*. A box lying flat on a mesh floor is
/// one cluster no matter how many triangles it spans; a shape wedged into a
/// corner is three. Eight leaves real room above that.
///
/// The cost is linear in this number twice over: the mesh size class is
/// `B3_NEA_MAX_MESH_MANIFOLDS * sizeof( b3Manifold )` per mesh contact (268
/// bytes each), and b3SolverStackDemand reserves a b3ManifoldConstraint for
/// every manifold that can exist at once.
///
/// @section drop What happens at the cap
///
/// Upstream has no bound here, so there is no upstream rule to follow. When
/// the clusters exceed this count the port keeps the ones with the **most
/// negative minimum separation** -- deepest first. A dropped shallow cluster
/// costs at worst a speculative contact that never touched; a dropped deep one
/// is a body sinking into the level. Under B3_ENABLE_ASSERT this asserts, and
/// the world counts it either way, so a scene that needs a larger cap says so
/// rather than losing contacts quietly.
#define B3_NEA_MAX_MESH_MANIFOLDS 8

/// Triangles one mesh contact may take from the BVH in a step.
///
/// Upstream's B3_MAX_MESH_CONTACT_TRIANGLES is 256. The query stops when the
/// buffer is full, so this bounds work as well as memory: every triangle past
/// the cap is a triangle-versus-shape collide that does not run.
#define B3_NEA_MAX_MESH_CONTACT_TRIANGLES 32

/// Clip points one triangle may contribute before reduction to a manifold.
///
/// Upstream's B3_MAX_POINTS_PER_TRIANGLE is 32. The product of this and the
/// triangle cap is the arena's point buffer for a single contact, which is why
/// upstream's pair would want 256 * 32 * sizeof( b3LocalManifoldPoint ) = 192
/// KB of scratch on a machine with 4 MB. This pair wants 6 KB.
#define B3_NEA_MAX_POINTS_PER_TRIANGLE 8

// =========================================================================
// Disabled subsystems
// =========================================================================

// No SSE2, no NEON. ARMv5TE has no SIMD unit at all, and the scalar b3FloatW
// fallback would just be four fixed-point ops in a trench coat.
#define BOX3D_DISABLE_SIMD 1

/// Recording, replay and world snapshots are development tooling that write
/// large buffers. Excluded from the port.
#define B3_NEA_NO_RECORDING 1

/// Height field and compound shapes are outside the ported feature set.
#define B3_NEA_NO_HEIGHTFIELD 1
#define B3_NEA_NO_COMPOUND 1

/// Body/shape/joint debug names. They allocate strings per entity.
#define B3_NEA_NO_NAMES 1

// =========================================================================
// Fixed timestep
// =========================================================================

/// The simulation runs at the DS refresh rate. dt is a compile-time constant
/// so that inverse-dt, dt*dt and the solver's soft-constraint coefficients
/// fold into constants instead of costing a hardware divide every substep.
///
/// b3World_Step() therefore takes only a substep count.
#define B3_NEA_STEP_HZ 60

/// Default substep count. 4 is Box3D's own recommendation; 2 is a reasonable
/// trade if a scene is CPU-bound.
#define B3_NEA_DEFAULT_SUBSTEPS 4

/// Gyroscopic torque, off.
///
/// Upstream defaults this to 1. Each iteration is a 3x3 matrix inverse plus a
/// 3x3 linear solve -- two hardware divides and about forty multiplies -- **per
/// body per sub-step**. At four sub-steps and 60 Hz that is the single most
/// expensive per-body operation in the engine, and what it buys is accuracy for
/// bodies spinning about a non-principal axis: a tumbling plank looks right, a
/// crate is unaffected.
///
/// The code is ported and correct (see b3IntegrateVelocitiesTask on why the
/// equation's homogeneity in I makes it representable at all). Raise this to 1
/// if a scene has long skinny bodies whose tumbling matters and the frame
/// budget allows it.
#define B3_GYROSCOPIC_ITERATIONS 0

// =========================================================================
// World scale
// =========================================================================

/// Length units per meter, frozen at compile time.
///
/// Upstream makes this a runtime float behind b3GetLengthUnitsPerMeter(), and
/// then calls that function from inside macros like B3_LINEAR_SLOP -- which
/// expand at 21 sites, several of them in hot loops. Here it is a constant so
/// every derived tolerance becomes a literal.
///
/// Positions are Q19.12, so the usable world is roughly +/-2000 units before
/// the linear slop stops being visually insignificant. Author at 1 unit = 1 m.
#define B3_NEA_LENGTH_UNITS_PER_METER 1

// =========================================================================
// ITCM groups
// =========================================================================
//
// Which parts of the solver are placed in ITCM, the ARM9's 32 KB
// zero-wait-state instruction memory. Set from the command line as
// `make -f Makefile.blocksds NEA_BOX3D_ITCM_WHEEL=1`; the makefile turns each
// into a -D, which is why every switch below is #ifndef-guarded.
//
// Why this is a choice and not a default:
//
// The ARM9's instruction cache is 8 KB and the solver does not fit in it. The
// sub-step loop calls b3Solve*Joint twice per joint per sub-step -- 32 times a
// frame for four joints at the default four sub-steps -- so a solve function
// larger than the cache re-streams its whole body from main RAM on every call.
// Moving one to ITCM removes that fetch entirely. Measured: placing the shared
// leaf math (b3hot.c) in ITCM took examples/physics/box3d_car from ~240% CPU
// to ~150-160% on a DSi.
//
// But ITCM is one budget shared with the whole game, and b3SolveJoint's type
// switch references every joint implementation, so archive-member linking pulls
// all of them into any ROM that uses joints at all. Without these switches a
// ragdoll would spend 14 KB of ITCM on a wheel solver it never calls. Only the
// game knows which joints its scenes actually use.
//
// The budget, and what is already spent:
//
//     32,768  total ITCM
//    -10,112  libnds interrupt/cothread code, NEA skinning
//     -2,988  b3hot.c shared leaf math (always on, see base.h)
//     -7,252  B3_ITCM_CONTACTS, on by default
//     -------
//     12,416  left for joint groups
//
// `make -f Makefile.blocksds itcm-report` prints the live costs from the built
// archive. Overshooting is a link error at ROM build time -- "section .itcm
// will not fit in region itcm" -- not a silent failure.
//
// B3_NO_ITCM (NEA_BOX3D_NO_ITCM=1) is the master switch and overrides all of
// these, for a game that wants its ITCM to itself.

/// Contact solve + warm start. 7,252 bytes.
///
/// The one group on by default: every scene has contacts, so these bytes are
/// never wasted. Turn it off to afford a large joint group -- box3d_car has to,
/// because CONTACTS plus WHEEL is 21,356 against 19,668 free.
#ifndef B3_ITCM_CONTACTS
#define B3_ITCM_CONTACTS 1
#endif

/// Wheel joint solve + warm start. 14,104 bytes.
///
/// The most valuable group by a wide margin, and the reason the mechanism
/// exists. b3SolveWheelJoint is 11,312 bytes -- the only function in the engine
/// that exceeds the instruction cache on its own -- and accounts for 77% of the
/// engine's per-frame instruction fetch in a four-wheeled vehicle scene.
#ifndef B3_ITCM_WHEEL
#define B3_ITCM_WHEEL 0
#endif

/// Prismatic joint solve + warm start. 10,180 bytes.
#ifndef B3_ITCM_PRISMATIC
#define B3_ITCM_PRISMATIC 0
#endif

/// Spherical joint solve + warm start. 7,456 bytes.
#ifndef B3_ITCM_SPHERICAL
#define B3_ITCM_SPHERICAL 0
#endif

/// Revolute joint solve + warm start. 7,376 bytes.
#ifndef B3_ITCM_REVOLUTE
#define B3_ITCM_REVOLUTE 0
#endif

/// Distance joint solve + warm start. 6,516 bytes.
#ifndef B3_ITCM_DISTANCE
#define B3_ITCM_DISTANCE 0
#endif

/// Motor joint solve + warm start. 5,976 bytes.
#ifndef B3_ITCM_MOTOR
#define B3_ITCM_MOTOR 0
#endif

/// Weld joint solve + warm start. 4,600 bytes.
#ifndef B3_ITCM_WELD
#define B3_ITCM_WELD 0
#endif

/// Parallel joint solve + warm start. 2,640 bytes.
#ifndef B3_ITCM_PARALLEL
#define B3_ITCM_PARALLEL 0
#endif

/// b3ComputeMeshManifolds, the triangle-mesh narrow phase. 8,496 bytes.
///
/// Runs once per mesh contact per step rather than per sub-step, so its fetch
/// volume is far below the joint groups'. Worth it only for a scene whose cost
/// is dominated by shapes resting on a mesh level.
#ifndef B3_ITCM_MESH
#define B3_ITCM_MESH 0
#endif

/// b3Solve, the step driver. 7,200 bytes.
///
/// Called once per step, but its sub-step loop -- which has the velocity and
/// position integrators inlined into it -- is re-entered after every joint and
/// contact pass, each of which is large enough to have evicted it. Modest,
/// universal, and the group to try when no single joint type dominates.
#ifndef B3_ITCM_CORE
#define B3_ITCM_CORE 0
#endif

#endif // BOX3D_NEA_CONFIG_H__
