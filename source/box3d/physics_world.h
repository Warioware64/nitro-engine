// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Erin Catto        (original Box3D)
// Copyright (c) 2026 Warioware64       (Nitro Engine Advanced fixed-point port)
//
// This file is part of Nitro Engine Advanced

#pragma once

/// @file   physics_world.h
/// @brief  The world: every pool, every array, every tuning value.
///
/// @section absent What is not in b3World
///
/// Upstream's b3World is 130 fields. Most of what is missing here was decided
/// before this phase and is only listed so the diff is explicable:
///
///   - **Worker/task contexts.** One core, so upstream's `b3Array(b3TaskContext)`
///     of per-thread arenas and bitsets folds into the world's single copy.
///     `enqueueTaskFcn`, `finishTaskFcn`, `scheduler`, `activeTaskCount` and
///     `workerCount` go with them.
///   - **The hull database.** Hulls are baked on the host and live in ROM, so
///     a shape keeps the caller's pointer. See shape.c.
///   - **Recording** and **world snapshots** (B3_NEA_NO_RECORDING), **name
///     caches** (B3_NEA_NO_NAMES), and the four **debug draw bitsets** with
///     their shape callbacks. Sensors arrived in Phase 7 Stage 3 and are here;
///     their per-worker b3SensorTaskContext folded into sensorEventBitSet by
///     the first rule above.
///   - **Upstream's `b3Profile`.** Millisecond timers on a machine without a
///     millisecond clock: a 16.6 ms frame quantized to milliseconds has
///     sixteen distinguishable values. Replaced rather than dropped -- the
///     `profile` field below counts BUS_CLOCK ticks instead, which resolve the
///     same frame into 558,000. See include/box3d/b3profile.h. The counters
///     upstream kept alongside it were always here; they just had no accessor
///     until the profile gave them one.
///   - **`inv_dt`.** The time step is a compile-time constant here
///     (B3_NEA_STEP_HZ), which is the whole reason `b3World_Step` takes only a
///     substep count. `inv_h` *is* kept, since it depends on that count -- see
///     the field, which Phase 6 Stage 2 added for the reaction queries.
///
/// @section manifolds Manifold allocation
///
/// Upstream keeps an array of block allocators, one per manifold count, behind
/// a mutex. The mutex is gone -- one thread -- but the allocators stay: a
/// contact's manifold array is sized by its manifold count, and a general heap
/// allocation per contact per step is exactly what the port must not do.

#include "arena_allocator.h"
#include "block_allocator.h"
#include "broad_phase.h"
#include "constraint_graph.h"
#include "container.h"
#include "core.h"
#include "id_pool.h"
#include "sensor.h"

#include "box3d/box3d.h"
#include "box3d/id.h"
#include "box3d/types.h"

#include <string.h>

typedef struct b3Body b3Body;
typedef struct b3Contact b3Contact;
typedef struct b3Island b3Island;
typedef struct b3Joint b3Joint;
typedef struct b3Shape b3Shape;
typedef struct b3SolverSet b3SolverSet;

b3DeclareArray( b3BlockAllocator );
b3DeclareArray( b3Body );
b3DeclareArray( b3SolverSet );
b3DeclareArray( b3Joint );
b3DeclareArray( b3Contact );
b3DeclareArray( b3Island );
b3DeclareArray( b3Shape );
b3DeclareArray( b3BodyMoveEvent );
b3DeclareArray( b3ContactBeginTouchEvent );
b3DeclareArray( b3ContactEndTouchEvent );
b3DeclareArray( b3ContactHitEvent );
b3DeclareArray( b3JointEvent );
b3DeclareArray( b3SensorBeginTouchEvent );
b3DeclareArray( b3SensorEndTouchEvent );

/// The three fixed solver sets, and where the sleeping ones start.
enum b3SetType
{
	b3_staticSet = 0,
	b3_disabledSet = 1,
	b3_awakeSet = 2,
	b3_firstSleepingSet = 3,
};

typedef struct b3World
{
	b3Stack stack;

	/// Per-call scratch for the narrow phase. Upstream gives each worker its
	/// own; there is one worker here, so there is one arena.
	///
	/// b3Arena is passed by value precisely so the bump pointer restores on
	/// return -- b3ComputeConvexManifold takes a 32-entry b3LocalManifoldPoint
	/// buffer from it on every contact and never frees it explicitly. Its
	/// capacity is a starting guess, not a bound: b3ArenaSync grows it to last
	/// step's peak, which is why b3Collide calls that once per pass.
	b3Arena arena;

	b3BroadPhase broadPhase;
	b3ConstraintGraph constraintGraph;

	/// One block allocator per manifold count. Upstream guards these with a
	/// mutex for the parallel narrow phase; there is one thread here.
	b3Array( b3BlockAllocator ) manifoldAllocators;

	/// One fixed-size triangle-cache array per mesh contact.
	///
	/// Reserved from b3Capacity::meshContactCount and taken at contact
	/// creation, for the same reason the manifold classes are pre-created:
	/// upstream grows a b3Array inside the narrow phase, and this port must not
	/// reach the allocator during a step. A world with no mesh contacts
	/// declared reserves nothing here.
	b3BlockAllocator triangleCacheAllocator;

	/// Allocates and recycles body ids. An id is a stable handle for the user
	/// but a cache miss for the engine, which is why body *data* lives in
	/// solver sets and this array is only the sparse map into them.
	b3IdPool bodyIdPool;
	b3Array( b3Body ) bodies;

	b3IdPool solverSetIdPool;
	b3Array( b3SolverSet ) solverSets;

	b3IdPool jointIdPool;
	b3Array( b3Joint ) joints;

	b3IdPool contactIdPool;
	b3Array( b3Contact ) contacts;

	b3IdPool islandIdPool;
	b3Array( b3Island ) islands;

	b3IdPool shapeIdPool;
	b3Array( b3Shape ) shapes;

	/// The sensors, dense and unordered -- b3Shape::sensorIndex is the map into
	/// it, and b3DestroySensor keeps that map correct by swapping the last
	/// sensor into the hole. There is no id pool, because nothing outside the
	/// shape refers to a sensor by index.
	b3Array( b3Sensor ) sensors;

	// -------------------------------------------------------------------
	// Per-step scratch, one worker
	// -------------------------------------------------------------------

	/// Aligns with the constraint graph's contact blocks; a set bit means a
	/// contact changed touching state this step.
	b3BitSet contactStateBitSet;

	/// Aligns with the contact id capacity; a set bit means a hit event.
	b3BitSet hitEventBitSet;

	/// Set true when any bit was set in hitEventBitSet this step, so the event
	/// pass can skip the whole bitset in the common case.
	bool hasHitEvents;

	/// Aligns with the joint id capacity; a set bit means the joint's reaction
	/// crossed one of its thresholds this step.
	///
	/// A bitset rather than appending to the event array directly, for the reason
	/// the hit events use one: the solve runs per constraint-graph colour and a
	/// joint can be visited more than once, so recording "this joint tripped" and
	/// draining afterwards reports each joint at most once and keeps the event
	/// array in a stable order.
	b3BitSet jointStateBitSet;

	/// The hasHitEvents counterpart, and the same optimization.
	bool hasJointEvents;

	/// Aligns with world->sensors; a set bit means that sensor's overlap set
	/// differs from last step's and owes begin or end events.
	///
	/// Upstream keeps one of these per worker in a b3SensorTaskContext and
	/// unions them before publishing. One worker, one bitset. It is still a
	/// bitset rather than a flag on b3Sensor because the publish pass walks it
	/// by trailing-zero count, which skips whole 64-sensor words of unchanged
	/// sensors -- the common case, since a settled trigger volume changes
	/// nothing from step to step.
	b3BitSet sensorEventBitSet;

	/// How many contacts the last collide pass answered from the recycling
	/// fast path instead of re-running the narrow phase.
	///
	/// Upstream keeps this per worker among the b3Counters this port dropped.
	/// It is kept because it is the only externally visible difference between
	/// recycling working and recycling never firing -- when it works, the
	/// manifold is deliberately the same either way, so a test that cannot see
	/// this number cannot tell the two apart.
	int recycledContactCount;

	/// Dynamic bodies parked this step for leaving maximumWorldExtent.
	/// Diagnostic, and reset with the other per-step counters in b3Collide.
	int parkedBodyCount;

	/// Fast bodies the continuous pass pulled back to a time of impact this
	/// step, and the worst iteration counts b3TimeOfImpact reported while doing
	/// it.
	///
	/// The iteration counts are the reason b3TOIOutput carries them at all.
	/// Upstream sets its caps -- 25 outer, 50 root -- from float experience,
	/// and whether they are right for a Q12 separation function paired with a
	/// Q30 bracket is a question only measurement answers. Because the port is
	/// deterministic integer code, what the host tests read here is exactly
	/// what the hardware runs.
	int toiEventCount;
	int toiDistanceIterations;
	int toiPushBackIterations;
	int toiRootIterations;

	/// Normal clusters the last collide pass threw away because a mesh contact
	/// produced more than B3_NEA_MAX_MESH_MANIFOLDS of them.
	///
	/// The cap is the port's rule and has no upstream counterpart, so it is
	/// counted rather than assumed never to bite: a dropped cluster is a
	/// contact that silently is not solved, and nothing else in the world
	/// looks any different. See B3_NEA_MAX_MESH_MANIFOLDS for the retention
	/// rule.
	int meshManifoldDropCount;

	/// Visitors the last sensor pass threw away because a sensor already held
	/// B3_NEA_MAX_SENSOR_VISITORS of them.
	///
	/// Same rule and same reason as meshManifoldDropCount above: the cap is the
	/// port's and has no upstream counterpart, and what it costs when it binds
	/// is invisible from anywhere else -- a begin event that never fires, and
	/// then an end event that never fires either, because a set that was never
	/// entered cannot be left.
	int sensorOverlapDropCount;

	/// Shapes whose plane batch filled B3_NEA_MAX_MOVER_PLANES during the last
	/// b3World_CollideMover call.
	///
	/// Batches, not planes -- counting planes exactly means finishing a mesh
	/// traversal past the cap, which is a GJK call per remaining triangle and
	/// exactly the cost the cap exists to avoid. See B3_NEA_MAX_MOVER_PLANES.
	///
	/// Unlike the two above, this one is not produced by the step: a mover runs
	/// between steps, and b3Collide clears this at the top of the next one. So
	/// read it straight after the mover, not at the end of the frame.
	int moverPlaneDropCount;

	/// Calls to b3Body_SetTransform over the life of the world.
	///
	/// The odd one out here: **cumulative, never reset**. A teleport is a rare
	/// event -- a respawn, a level warp, a camera target snapping -- and the
	/// question worth asking is "did this happen at all", which a per-step
	/// reading answers with 1 for the single frame that did it and 0 for the
	/// hundred either side.
	///
	/// It is here because a teleport is not free and nothing else says so. The
	/// body's shapes re-enter the move buffer, re-pair on the next step, form
	/// contacts and re-form their island; in a world whose capacity was sized
	/// from the settled scene, that is where a mid-step allocation comes from.
	/// A rising lateAllocCount with no visible cause, next to a non-zero
	/// reading here, is the explanation -- and it is not a leak, because the
	/// memory is reused. Without this counter the two are indistinguishable.
	int teleportCount;

	/// Bodies whose shapes have enlarged AABBs. A bitset rather than a flag
	/// walk, because a world can hold thousands of static shapes that never
	/// enlarge.
	b3BitSet enlargedSimBitSet;

	/// Islands that stayed awake this step.
	b3BitSet awakeIslandBitSet;

	/// The island wanting to sleep with the longest-settled body, and its
	/// sleep time. Chosen per step as the single split candidate.
	b3t splitSleepTime;
	int splitIslandId;

	// -------------------------------------------------------------------
	// Events
	// -------------------------------------------------------------------

	b3Array( b3BodyMoveEvent ) bodyMoveEvents;
	b3Array( b3ContactBeginTouchEvent ) contactBeginEvents;

	/// Double buffered so the user need not flush events: a contact destroyed
	/// between steps still reports its end touch.
	b3Array( b3ContactEndTouchEvent ) contactEndEvents[2];
	int endEventArrayIndex;

	b3Array( b3ContactHitEvent ) contactHitEvents;

	b3Array( b3JointEvent ) jointEvents;

	b3Array( b3SensorBeginTouchEvent ) sensorBeginEvents;

	/// Double buffered for the reason the contact end events are, and sharing
	/// their endEventArrayIndex: a sensor or a visitor destroyed *between*
	/// steps writes its end touch into the buffer the next step will publish.
	b3Array( b3SensorEndTouchEvent ) sensorEndEvents[2];

	// -------------------------------------------------------------------
	// Tuning
	// -------------------------------------------------------------------

	b3Vec3 gravity;
	b3f hitEventThreshold;
	b3f restitutionThreshold;
	b3f maxLinearSpeed;
	b3f maxWorldExtent;
	b3f contactSpeed;
	b3f contactHertz;
	b3f contactDampingRatio;
	b3f contactRecycleDistance;

	b3FrictionCallback* frictionCallback;
	b3RestitutionCallback* restitutionCallback;

	b3PreSolveFcn* preSolveFcn;
	void* preSolveContext;

	b3CustomFilterFcn* customFilterFcn;
	void* customFilterContext;

	void* userData;

	/// Advanced every time step.
	uint64_t stepIndex;

	/// The inverse sub-step of the most recent step, at Q12.
	///
	/// The one piece of step timing the world has to remember. `inv_dt` does
	/// not need storing -- the time step is compile-time (B3_NEA_STEP_HZ) --
	/// but `inv_h` depends on the sub-step count, which is an argument to
	/// b3World_Step and may differ from one step to the next.
	///
	/// It exists for the reaction queries. They turn an accumulated impulse
	/// back into the force it represents, and they are called *between* steps,
	/// when the b3StepContext that carried inv_h is gone. Zero before the
	/// first step, which makes every reaction query report zero -- correct,
	/// since no constraint has been solved yet.
	b3f inv_h;

	/// The capacities the world was created with, for reporting.
	b3Capacity maxCapacity;

	/// The step's clock, live only between b3ProfileStart and b3ProfileEnd
	/// inside b3World_Step.
	///
	/// On the world rather than passed down as a parameter because the probes
	/// are three call levels apart -- b3World_Step, b3Solve, and the mesh/convex
	/// branch inside b3UpdateContact -- and every one of those already has the
	/// world in hand. Threading a parameter through b3Collide, b3CollideTask
	/// and b3UpdateContact would change three hot signatures to carry something
	/// that is nothing at all in a non-profiling build.
	///
	/// Four bytes, and the world is already hot in the data cache wherever a
	/// probe fires.
	b3ProfileTimer profileTimer;

	/// Where the last step went, in timer ticks. Published by
	/// b3World_GetProfile; see include/box3d/b3profile.h.
	///
	/// The field is here unconditionally even though the probes that fill it
	/// are compiled out unless B3_NEA_PROFILE is set. Roughly 120 bytes on a
	/// struct already several kilobytes wide, in exchange for an accessor that
	/// always links and a game that keeps compiling when the switch goes off.
	///
	/// The counter half of it is filled either way -- those numbers were
	/// already being maintained by b3Collide and b3Solve and cost nothing to
	/// copy out.
	b3Profile profile;

	uint16_t generation;
	uint16_t worldId;

	bool enableSleep;
	bool enableWarmStarting;
	bool enableContinuous;
	bool enableSpeculative;
	bool inUse;

	/// A world write is in progress. Debugging aid, not a real mutex.
	bool locked;
} b3World;

b3World* b3GetUnlockedWorldFromId( b3WorldId id );
b3World* b3GetWorldFromId( b3WorldId id );

b3World* b3GetUnlockedWorld( int index );
b3World* b3GetWorld( int index );

void b3ValidateConnectivity( b3World* world );
void b3ValidateSolverSets( b3World* world );
void b3ValidateContacts( b3World* world );

/// The narrow phase: update every awake contact's manifold, then apply the
/// resulting begin/end-touch transitions to the islands and constraint graph.
///
/// Upstream takes a `b3StepContext*` and is static, because its only caller is
/// b3World_Step. There is no step context until Phase 3C -- the only field of
/// it this reads is the world -- so it takes the world directly and is visible
/// here, which is also what lets the host tests drive a collide pass without a
/// solver. 3C's b3World_Step calls this same function.
void b3Collide( b3World* world );

/// Allocate a zeroed array of `capacity` manifolds from the block allocator
/// for that size class.
///
/// `capacity`, not the number of manifolds that will carry points. A convex
/// contact asks for 1 and uses 1. A mesh contact asks for
/// B3_NEA_MAX_MESH_MANIFOLDS once and then varies `b3Contact::manifoldCount`
/// from step to step without coming back here -- see b3Contact::manifoldCapacity
/// for why that matters. Whatever is passed in must be passed to
/// b3FreeManifolds, or the memory goes back to the wrong size class.
///
/// @section cost What this used to cost
///
/// This function used to create the allocator it needed on first use:
///
///     b3CreateBlockAllocator( ( i + 1 ) * sizeof( b3Manifold ), 2 * B3_BLOCK_SIZE )
///
/// with a comment claiming it was never a general heap allocation. Creating an
/// allocator *is* one, and at upstream's block size it was a large one: 268
/// bytes per manifold, 256 per block, two blocks -- **137 KB, taken the first
/// time two bodies touched**, in the middle of a frame, on a machine with 4 MB.
/// A host test now counts allocations across 240 steps and holds it to zero.
///
/// So b3CreateWorld pre-creates every size class from `b3Capacity`, and this is
/// now only the lookup. There is deliberately no fallback: an index the world
/// did not reserve is a sizing bug, and creating an allocator to paper over it
/// is exactly the mid-step allocation this asserts against.
///
/// @section growth When this still allocates
///
/// One case, and it means the world was sized wrong rather than that the design
/// failed: more live manifolds of one class than that class reserved.
/// b3AllocateElement then adds a block. Under NEA_Phys3D that is reported by
/// NEA_Phys3DWorldGetLateAllocCount().
static inline b3Manifold* b3AllocateManifolds( b3World* world, int capacity )
{
	if ( capacity == 0 )
	{
		return NULL;
	}

	int index = capacity - 1;
	B3_ASSERT( 0 <= index && index < world->manifoldAllocators.count );

	b3BlockAllocator* allocator = b3Array_Get( world->manifoldAllocators, index );
	b3Manifold* manifolds = (b3Manifold*)b3AllocateElement( allocator );
	memset( manifolds, 0, capacity * sizeof( b3Manifold ) );
	return manifolds;
}

/// Return manifolds to their size class. `capacity` must be what
/// b3AllocateManifolds was given, which is b3Contact::manifoldCapacity and not
/// b3Contact::manifoldCount.
static inline void b3FreeManifolds( b3World* world, b3Manifold* manifolds, int capacity )
{
	if ( capacity == 0 )
	{
		return;
	}

	int index = capacity - 1;
	B3_ASSERT( 0 <= index && index < world->manifoldAllocators.count );

	b3BlockAllocator* allocator = b3Array_Get( world->manifoldAllocators, index );
	b3FreeElement( allocator, manifolds );
}

// The public API -- b3CreateBody, b3Body_*, b3CreateWorld, b3World_*,
// b3Create*Shape, b3Shape_*, b3Contact_GetData -- is declared in
// include/box3d/box3d.h, which is the header a game installs and includes.
// This file keeps only what the port's own translation units need.
