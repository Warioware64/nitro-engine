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

/// Shapes one sensor may report as overlapping it at once.
///
/// Upstream creates the two overlap arrays at 16 and lets them grow; the port
/// reserves them once when the sensor shape is created and never grows them,
/// because growing an array during a step is the allocation the pool refuses.
/// 16 is upstream's starting size kept as the bound.
///
/// @section drop What happens at the cap
///
/// The visitor is dropped and b3World::sensorOverlapDropCount counts it. That
/// is a begin event that never fires -- and, a step later, an end event that
/// never fires either, since a set that was never entered cannot be left. Same
/// rule and same reason as B3_NEA_MAX_MESH_MANIFOLDS: a bound the port invents
/// is a bound the port has to be able to see bind.
#define B3_NEA_MAX_SENSOR_VISITORS 16

/// Sensors one fast body may sweep through in a single step.
///
/// Upstream's B2_MAX_CONTINUOUS_SENSOR_HITS, unchanged at 8. This is the array
/// the continuous pass fills for a body moving fast enough to cross a trigger
/// volume entirely within one step -- without it, the end-of-step overlap
/// query is looking at a body that is already past the sensor and the trigger
/// never fires. It is per fast body, not per world, and lives on the stack in
/// b3ContinuousContext.
#define B3_NEA_MAX_CONTINUOUS_SENSOR_HITS 8

/// Collision planes one shape may hand a character mover in one call.
///
/// Two arrays share this bound: the buffer b3World_CollideMover puts on the
/// stack inside its tree callback, and whatever the caller collects into.
/// Upstream uses 64 for the first and 8 for the second (samples/mover.h), so 8
/// is the number that was ever reachable -- the other 56 are computed by GJK,
/// copied, and dropped by the collector on the way past.
///
/// 64 is also the wrong number for this machine. sizeof(b3PlaneResult) is 28
/// bytes in device mode, so 64 of them is 1792 bytes of stack, claimed inside a
/// broad-phase tree callback that is already standing on b3DynamicTree_Query's
/// node stack and a GJK frame, on an ARM9 with 16 KB of DTCM. Eight is 224.
///
/// Eight also fits the geometry. Only the mesh backend returns more than one
/// plane, and it returns one per *triangle*: a capsule in the corner of a
/// triangulated room touches two floor triangles and two per wall, which is
/// six.
///
/// @section drop What happens at the cap
///
/// The traversal stops -- b3CollideMoverAndMesh returns false from its
/// b3QueryMesh callback and the remaining triangles are never tested -- and
/// b3World::moverPlaneDropCount counts the batch that saturated.
///
/// What it costs is a **surface the mover is never pushed away from**, because
/// b3SolvePlanes only pushes against planes it was handed. That is worse than
/// the other two caps in this file and worth saying plainly: a dropped contact
/// is a body settling slowly into a crease, and a dropped mover plane is a
/// player walking through a wall, at the one moment the geometry was busiest.
/// Same rule all the same: a bound the port invents is a bound the port has to
/// be able to see bind.
///
/// The counter counts saturated *batches*, not dropped planes. Counting planes
/// exactly would mean finishing the traversal past the cap -- a GJK call per
/// remaining triangle, which is the cost the cap exists to avoid. A saturated
/// batch is an upper bound on the drops and a lower bound of one, which is all
/// "see it bind" needs.
#define B3_NEA_MAX_MOVER_PLANES 8

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
// Profiling
// =========================================================================
//
// Set from the command line as `make -f Makefile.blocksds NEA_BOX3D_PROFILE=1`,
// which is why each is #ifndef-guarded like the ITCM switches below.
//
// Three levels, because they do not cost the same. Level 1 is cheap enough to
// leave on; the other two change the thing they are measuring and are for
// answering one question and then turning off again.

/// Level 1: the step broken into phases. Off by default.
///
/// Ten probes per step -- broad phase, narrow phase, solve, sensors, and the
/// eight stages inside b3Solve -- each two 16-bit I/O reads plus a subtract and
/// a store. Order tens of cycles against a DSi frame of roughly 2.2 million, so
/// this is measurable only as noise.
///
/// This is the switch that also publishes the counters b3World already keeps
/// (recycled contacts, parked bodies, the TOI iteration counts, the mesh
/// manifold drops). Those are free either way; they simply had no way out of
/// the library before.
///
/// See include/box3d/b3profile.h for what the numbers mean and why the port
/// counts timer ticks where upstream counted milliseconds.
///
/// @section timer What it takes over
///
/// Timers 0 and 1, for the duration of the step -- the same cascaded pair
/// libnds's cpuStartTiming()/cpuEndTiming() drives. They cannot be shared: a
/// caller that brackets b3World_Step with its own cpuStartTiming will have its
/// count reset by the first probe. Drop the bracket and read
/// b3Profile::totalTicks, which is measuring the same interval.
#ifndef B3_NEA_PROFILE
#define B3_NEA_PROFILE 0
#endif

/// Level 2: the sub-step loop broken into its eight stages. Requires level 1.
///
/// 32 probes per step at the default four sub-steps -- but only eight *sites*,
/// because they are inside the loop rather than unrolled with it. Measured on
/// b3Solve, which is the function that carries them:
///
///     7,264 B   no profiling
///     7,488 B   level 1   (+224)
///     7,600 B   level 2   (+336, so +112 over level 1)
///
/// That is 4.6% growth on one function, which is smaller than it first looks
/// worth worrying about: the sub-step loop is one copy of the code, not four.
///
/// It is still not free. b3Solve is re-entered after every joint and contact
/// pass, each large enough to have evicted it from an 8 KiB instruction cache,
/// so bytes added here are bytes re-fetched many times per frame. Two rules
/// follow:
///
///   - Read stages against each other rather than against the frame budget.
///     The proportions are sound; the absolute microseconds are inflated.
///   - Never choose an ITCM group from a level-2 build. An ITCM group exists
///     to change instruction fetch, and this build has already perturbed
///     exactly that. Measure ITCM changes with level 1, or with profiling off
///     and NEA_GetCPUPercent().
#ifndef B3_NEA_PROFILE_SUBSTEP
#define B3_NEA_PROFILE_SUBSTEP 0
#endif

/// Level 3: the narrow phase split into mesh and convex, plus a triangle count.
/// Requires level 1.
///
/// Two probes per contact rather than per step, so the overhead scales with the
/// contact count instead of being constant -- but it is bracketing
/// b3ComputeMeshManifolds, which is 8,496 bytes of work, so the ratio stays
/// tiny.
///
/// The triangle count is the reason to turn this on rather than infer from
/// level 1. Mesh narrow-phase cost is roughly linear in triangles tested, and
/// B3_NEA_MAX_MESH_CONTACT_TRIANGLES caps that at 32 per contact -- so a scene
/// sitting at the cap is both spending the most it can and dropping geometry,
/// and only this counter distinguishes that from a scene that happens to touch
/// 32 triangles honestly.
#ifndef B3_NEA_PROFILE_NARROW
#define B3_NEA_PROFILE_NARROW 0
#endif

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

// -------------------------------------------------------------------------
// Why solve and warm start are separate groups
// -------------------------------------------------------------------------
//
// They were one group per joint type until the budget arithmetic was actually
// done. For a four-wheeled vehicle:
//
//     19,668  free after baseline + shared math
//      7,252  CONTACTS  (b3SolveContacts 4,924 + b3WarmStartContacts 2,328)
//     14,104  WHEEL     (b3SolveWheelJoint 11,312 + b3WarmStartWheelJoint 2,792)
//     ------
//     21,356  over by 1,688
//
// So box3d_car shipped with WHEEL **off** -- excluding the one function the
// comment below calls 77% of the engine's per-frame instruction fetch, to keep
// a warm start that did not need to be there.
//
// The two halves have completely different fetch volumes:
//
//     b3SolveWheelJoint       11,312 B, called 2x per joint per sub-step
//                             (32 calls/frame at 4 wheels, 4 sub-steps)
//     b3WarmStartWheelJoint    2,792 B, called 1x per joint per sub-step
//                             (16 calls/frame)
//
// The solve is four times the size and runs twice as often, and at 11,312 bytes
// it does not fit in the ARM9's 8 KiB instruction cache even alone -- so every
// one of those 32 calls re-streams it from main RAM. The warm start fits with
// room to spare and stays resident between calls, so main RAM costs it a miss
// on first use and nothing after.
//
// Splitting the group spends ITCM where the fetch is:
//
//      7,252  CONTACTS
//     11,312  WHEEL_SOLVE
//     ------
//     18,564  fits, 1,104 to spare
//
// The old names still work and set both halves -- see the aliases at the end of
// this section -- so an existing `NEA_BOX3D_ITCM_WHEEL=1` build line keeps its
// meaning.

/// Contact solve. 4,924 bytes.
///
/// On by default with its warm-start half: every scene has contacts, so these
/// bytes are never wasted.
#ifndef B3_ITCM_CONTACTS_SOLVE
#define B3_ITCM_CONTACTS_SOLVE 1
#endif

/// Contact warm start. 2,328 bytes.
#ifndef B3_ITCM_CONTACTS_WARM
#define B3_ITCM_CONTACTS_WARM 1
#endif

/// Wheel joint solve. 11,312 bytes.
///
/// The most valuable single placement in the engine, and the reason the
/// mechanism exists. The only function in the port that exceeds the instruction
/// cache on its own, and it accounts for 77% of the engine's per-frame
/// instruction fetch in a four-wheeled vehicle scene.
///
/// This is the one to turn on for a vehicle game: `NEA_BOX3D_ITCM_WHEEL_SOLVE=1`
/// fits alongside the default CONTACTS pair, where the old whole-group
/// `NEA_BOX3D_ITCM_WHEEL=1` did not.
#ifndef B3_ITCM_WHEEL_SOLVE
#define B3_ITCM_WHEEL_SOLVE 0
#endif

/// Wheel joint warm start. 2,792 bytes.
#ifndef B3_ITCM_WHEEL_WARM
#define B3_ITCM_WHEEL_WARM 0
#endif

/// Prismatic joint solve. 8,316 bytes.
#ifndef B3_ITCM_PRISMATIC_SOLVE
#define B3_ITCM_PRISMATIC_SOLVE 0
#endif

/// Prismatic joint warm start. 1,864 bytes.
#ifndef B3_ITCM_PRISMATIC_WARM
#define B3_ITCM_PRISMATIC_WARM 0
#endif

/// Spherical joint solve. 6,244 bytes.
#ifndef B3_ITCM_SPHERICAL_SOLVE
#define B3_ITCM_SPHERICAL_SOLVE 0
#endif

/// Spherical joint warm start. 1,212 bytes.
#ifndef B3_ITCM_SPHERICAL_WARM
#define B3_ITCM_SPHERICAL_WARM 0
#endif

/// Revolute joint solve. 6,108 bytes.
#ifndef B3_ITCM_REVOLUTE_SOLVE
#define B3_ITCM_REVOLUTE_SOLVE 0
#endif

/// Revolute joint warm start. 1,268 bytes.
#ifndef B3_ITCM_REVOLUTE_WARM
#define B3_ITCM_REVOLUTE_WARM 0
#endif

/// Distance joint solve. 5,672 bytes.
#ifndef B3_ITCM_DISTANCE_SOLVE
#define B3_ITCM_DISTANCE_SOLVE 0
#endif

/// Distance joint warm start. 844 bytes.
#ifndef B3_ITCM_DISTANCE_WARM
#define B3_ITCM_DISTANCE_WARM 0
#endif

/// Motor joint solve. 4,900 bytes.
#ifndef B3_ITCM_MOTOR_SOLVE
#define B3_ITCM_MOTOR_SOLVE 0
#endif

/// Motor joint warm start. 1,076 bytes.
#ifndef B3_ITCM_MOTOR_WARM
#define B3_ITCM_MOTOR_WARM 0
#endif

/// Weld joint solve. 3,588 bytes.
#ifndef B3_ITCM_WELD_SOLVE
#define B3_ITCM_WELD_SOLVE 0
#endif

/// Weld joint warm start. 1,012 bytes.
#ifndef B3_ITCM_WELD_WARM
#define B3_ITCM_WELD_WARM 0
#endif

/// Parallel joint solve. 2,052 bytes.
#ifndef B3_ITCM_PARALLEL_SOLVE
#define B3_ITCM_PARALLEL_SOLVE 0
#endif

/// Parallel joint warm start. 588 bytes.
#ifndef B3_ITCM_PARALLEL_WARM
#define B3_ITCM_PARALLEL_WARM 0
#endif

// -------------------------------------------------------------------------
// Whole-group aliases, kept for compatibility
// -------------------------------------------------------------------------
//
// `-DB3_ITCM_WHEEL=1` still means "both halves of the wheel joint". Defined
// after the halves so that setting a half directly is not overridden, and
// tested with #if defined so that only a group the caller actually named
// participates.
//
// Makefile.blocksds expands NEA_BOX3D_ITCM_<GROUP> into one of these, so a
// build line written against the old group list keeps working unchanged.

#if defined( B3_ITCM_WHEEL ) && B3_ITCM_WHEEL
#undef B3_ITCM_WHEEL_SOLVE
#undef B3_ITCM_WHEEL_WARM
#define B3_ITCM_WHEEL_SOLVE 1
#define B3_ITCM_WHEEL_WARM 1
#endif

#if defined( B3_ITCM_PRISMATIC ) && B3_ITCM_PRISMATIC
#undef B3_ITCM_PRISMATIC_SOLVE
#undef B3_ITCM_PRISMATIC_WARM
#define B3_ITCM_PRISMATIC_SOLVE 1
#define B3_ITCM_PRISMATIC_WARM 1
#endif

#if defined( B3_ITCM_SPHERICAL ) && B3_ITCM_SPHERICAL
#undef B3_ITCM_SPHERICAL_SOLVE
#undef B3_ITCM_SPHERICAL_WARM
#define B3_ITCM_SPHERICAL_SOLVE 1
#define B3_ITCM_SPHERICAL_WARM 1
#endif

#if defined( B3_ITCM_REVOLUTE ) && B3_ITCM_REVOLUTE
#undef B3_ITCM_REVOLUTE_SOLVE
#undef B3_ITCM_REVOLUTE_WARM
#define B3_ITCM_REVOLUTE_SOLVE 1
#define B3_ITCM_REVOLUTE_WARM 1
#endif

#if defined( B3_ITCM_DISTANCE ) && B3_ITCM_DISTANCE
#undef B3_ITCM_DISTANCE_SOLVE
#undef B3_ITCM_DISTANCE_WARM
#define B3_ITCM_DISTANCE_SOLVE 1
#define B3_ITCM_DISTANCE_WARM 1
#endif

#if defined( B3_ITCM_MOTOR ) && B3_ITCM_MOTOR
#undef B3_ITCM_MOTOR_SOLVE
#undef B3_ITCM_MOTOR_WARM
#define B3_ITCM_MOTOR_SOLVE 1
#define B3_ITCM_MOTOR_WARM 1
#endif

#if defined( B3_ITCM_WELD ) && B3_ITCM_WELD
#undef B3_ITCM_WELD_SOLVE
#undef B3_ITCM_WELD_WARM
#define B3_ITCM_WELD_SOLVE 1
#define B3_ITCM_WELD_WARM 1
#endif

#if defined( B3_ITCM_PARALLEL ) && B3_ITCM_PARALLEL
#undef B3_ITCM_PARALLEL_SOLVE
#undef B3_ITCM_PARALLEL_WARM
#define B3_ITCM_PARALLEL_SOLVE 1
#define B3_ITCM_PARALLEL_WARM 1
#endif

// CONTACTS is the one whose alias must also be able to turn things **off**,
// because both halves default to on -- that is how a scene affords a large
// joint group. `-DB3_ITCM_CONTACTS=0` therefore clears both rather than being
// ignored the way a 0 is for the default-off groups above.
#if defined( B3_ITCM_CONTACTS )
#undef B3_ITCM_CONTACTS_SOLVE
#undef B3_ITCM_CONTACTS_WARM
#define B3_ITCM_CONTACTS_SOLVE B3_ITCM_CONTACTS
#define B3_ITCM_CONTACTS_WARM B3_ITCM_CONTACTS
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
