// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Erin Catto        (original Box3D)
// Copyright (c) 2026 Warioware64       (Nitro Engine Advanced fixed-point port)
//
// This file is part of Nitro Engine Advanced

/// @file   physics_world.c
/// @brief  World creation, teardown, tuning, the narrow phase, and the three
///         validators.
///
/// @section scope What is here
///
/// The half of upstream's physics_world.c that does not integrate. Absent,
/// each with a phase: `b3World_Draw` and every debug-draw
/// callback, the ray/shape/mover casts and overlap queries (Phase 7 with the
/// mover), `b3World_Explode`, recording, and `b3World_DumpMemoryStats`.
///
/// @section validators Why the validators are the point of this file
///
/// Phase 3A simulated nothing, so its correctness was entirely structural: do
/// the solver sets, islands, id pools and linked lists agree with each other
/// after every mutation? Upstream ships three functions that answer exactly
/// that, and they are ported in full and compiled under B3_ENABLE_VALIDATION.
/// Every create, destroy, transfer and re-type calls one -- and so does
/// b3Collide, which is 3B's only oracle for the same reason.

#include "physics_world.h"

#include "aabb.h"
#include "body.h"
#include "constraint_graph.h"
#include "contact.h"
#include "ctz.h"
#include "island.h"
#include "joint.h"
#include "mesh_contact.h"
#include "platform.h"
#include "shape.h"
#include "solver.h"
#include "solver_set.h"

#include "box3d/constants.h"

#include <string.h>

/// The world pool. B3_MAX_WORLDS is 1 here; upstream's default of 128 would
/// statically exceed the DS's entire 4 MB before a body existed.
b3World b3_worlds[B3_MAX_WORLDS];

b3World* b3GetUnlockedWorldFromId( b3WorldId id )
{
	B3_ASSERT( 1 <= id.index1 && id.index1 <= B3_MAX_WORLDS );
	b3World* world = b3_worlds + ( id.index1 - 1 );
	B3_ASSERT( id.index1 == world->worldId + 1 );
	B3_ASSERT( id.generation == world->generation );

	// A world reached through an id should never be mid-write.
	if ( world->locked )
	{
		B3_ASSERT( false );
		return NULL;
	}
	return world;
}

b3World* b3GetWorldFromId( b3WorldId id )
{
	B3_ASSERT( 1 <= id.index1 && id.index1 <= B3_MAX_WORLDS );
	b3World* world = b3_worlds + ( id.index1 - 1 );
	B3_ASSERT( id.index1 == world->worldId + 1 );
	B3_ASSERT( id.generation == world->generation );
	return world;
}

b3World* b3GetWorld( int index )
{
	B3_ASSERT( 0 <= index && index < B3_MAX_WORLDS );
	b3World* world = b3_worlds + index;
	B3_ASSERT( world->worldId == index );
	return world;
}

b3World* b3GetUnlockedWorld( int index )
{
	B3_ASSERT( 0 <= index && index < B3_MAX_WORLDS );
	b3World* world = b3_worlds + index;
	B3_ASSERT( world->worldId == index );
	if ( world->locked )
	{
		B3_ASSERT( false );
		return NULL;
	}

	return world;
}

// =========================================================================
// Material mixing
// =========================================================================

static b3c b3DefaultFrictionCallback( b3c frictionA, uint64_t materialA, b3c frictionB, uint64_t materialB )
{
	B3_UNUSED( materialA, materialB );

	// sqrt(a*b). Both operands are Q30 coefficients in [0,1], so the product
	// is too, and the root of a value below 1 is larger than the value -- no
	// range problem in either direction. The product is kept wide going into
	// the root for the same reason b3Length does it.
	int64_t product = (int64_t)b3Raw( frictionA ) * b3Raw( frictionB );
	if ( product <= 0 )
	{
		return b3c_zero;
	}

	// product is Q60. b3HwSqrt64 of a Q60 value gives Q30 directly.
	return b3Makeb3cRef( (int32_t)b3HwSqrt64( (uint64_t)product ),
						 B3_REF( sqrt( b3RefC( frictionA ) * b3RefC( frictionB ) ) ) );
}

static b3c b3DefaultRestitutionCallback( b3c restitutionA, uint64_t materialA, b3c restitutionB, uint64_t materialB )
{
	B3_UNUSED( materialA, materialB );
	return b3MaxC( restitutionA, restitutionB );
}

// =========================================================================
// Create and destroy
// =========================================================================

b3WorldId b3CreateWorld( const b3WorldDef* def )
{
	B3_CHECK_DEF( def );

	B3_ASSERT( b3Raw( B3_LINEAR_SLOP ) <= b3Raw( B3_MESH_REST_OFFSET ) );
	B3_ASSERT( b3Raw( B3_MESH_REST_OFFSET ) < b3Raw( B3_SPECULATIVE_DISTANCE ) );

	int worldId = B3_NULL_INDEX;
	for ( int i = 0; i < B3_MAX_WORLDS; ++i )
	{
		if ( b3_worlds[i].inUse == false )
		{
			worldId = i;
			break;
		}
	}

	if ( worldId == B3_NULL_INDEX )
	{
		b3Log( "B3_MAX_WORLDS of %d exceeded", B3_MAX_WORLDS );
		B3_ASSERT( worldId != B3_NULL_INDEX );
		return b3_nullWorldId;
	}

	b3InitializeContactRegisters();

	b3World* world = b3_worlds + worldId;
	uint16_t generation = world->generation;

	memset( world, 0, sizeof( b3World ) );

	world->worldId = (uint16_t)worldId;
	world->generation = generation;
	world->inUse = true;

	// The four numbers every pool below is sized from.
	//
	// A floor of 16 keeps a caller who left b3Capacity zeroed from getting a
	// world that reallocates on its first frame; above that these are taken at
	// face value, because the port treats capacity as the size of the world
	// rather than as upstream's hint.
	int bodyCapacity = b3MaxInt( 16, def->capacity.staticBodyCount + def->capacity.dynamicBodyCount );
	int dynamicBodyCapacity = b3MaxInt( 16, def->capacity.dynamicBodyCount );
	int shapeCapacity = b3MaxInt( 16, def->capacity.staticShapeCount + def->capacity.dynamicShapeCount );
	int contactCapacity = b3MaxInt( 16, def->capacity.contactCount );

	// No floor, for meshContactCount's reason rather than contactCount's: most
	// worlds have no joints at all, and a joint's sim is reserved *four times
	// over* -- the graph colour plus the three fixed sets -- because the sim
	// moves between them as its bodies sleep, wake, disable and change type.
	// A floor of 16 would have charged every world four arrays it never uses.
	//
	// Zero is safe rather than merely cheap: joints are created while the
	// scene is being built, so a caller who forgets this field grows the
	// arrays then, which counts as build memory. Only a joint created during a
	// step would be a late allocation, and nothing creates one.
	int jointCapacity = b3MaxInt( 0, def->capacity.jointCount );

	// No floor on this one: zero is the common case and it is meaningful --
	// a world with no mesh shapes must pay nothing for the mesh pools.
	int meshContactCapacity = b3MaxInt( 0, def->capacity.meshContactCount );

	// Peak per-step scratch, from the capacities rather than from a round
	// number and hope.
	//
	// b3Stack is LIFO and the stages are sequential -- the pair update frees its
	// entries before b3Solve takes any -- so the peak is the larger stage, not
	// their sum. Each stage owns the definition of its own demand so that adding
	// an allocation and forgetting to account for it is a one-file mistake.
	//
	// This matters more here than upstream. b3GrowStack calls b3Alloc, and
	// NEA_Phys3D installs a pool allocator that refuses to allocate once the
	// world exists: an under-sized stack is not a few slow frames, it is an
	// assert. Sizing it correctly here is what buys the guarantee.
	//
	// A convex contact produces exactly one manifold (contact.c asserts it), so
	// for a world without meshes the manifold count *is* the contact count. A
	// mesh contact carries up to B3_NEA_MAX_MESH_MANIFOLDS -- one per normal
	// cluster -- so each one declared adds cap-1 manifolds above the one it
	// already contributes as a contact.
	{
		int collideBytes = b3BroadPhaseStackDemand( shapeCapacity );

		int solveContactCapacity = contactCapacity + meshContactCapacity;
		int solveManifoldCapacity = solveContactCapacity + meshContactCapacity * ( B3_NEA_MAX_MESH_MANIFOLDS - 1 );
		int solveBytes = b3SolverStackDemand( solveContactCapacity, solveManifoldCapacity );

		world->stack = b3CreateStack( b3MaxInt( 2048, b3MaxInt( collideBytes, solveBytes ) ) );
	}

	// The narrow phase takes one 32-entry b3LocalManifoldPoint buffer per
	// contact it updates, and the buffer is released by the bump pointer
	// restoring on return rather than explicitly -- so the live demand is one
	// buffer, not one per contact. Sized generously anyway because b3ArenaSync
	// only ever grows this, and starting too small costs a heap allocation on
	// the first frames (Phase 1 finding 3).
	//
	// A mesh contact is the larger customer by far: it bumps a point buffer of
	// B3_NEA_MAX_MESH_CONTACT_TRIANGLES * B3_NEA_MAX_POINTS_PER_TRIANGLE plus
	// several per-triangle arrays, all of it live at once. That demand is
	// defined next to the b3Bump calls it accounts for, the same way
	// b3SolverStackDemand and b3BroadPhaseStackDemand are.
	{
		int arenaBytes = 4096;

		if ( meshContactCapacity > 0 )
		{
			arenaBytes = b3MaxInt( arenaBytes, b3MeshContactArenaDemand() );
		}

		world->arena = b3CreateArena( arenaBytes );
	}

	// One block allocator per manifold size class, all of them created here.
	//
	// b3AllocateManifolds indexes this array by `capacity - 1` and does not
	// create anything: it runs inside the step, and creating an allocator is a
	// heap allocation. So every index a contact can ask for must exist now.
	//
	// Only two classes are ever asked for. Convex contacts take one manifold
	// each, and mesh contacts take B3_NEA_MAX_MESH_MANIFOLDS each and keep them
	// (see b3Contact::manifoldCapacity). The classes in between exist so the
	// lookup is a plain index rather than a mapping, and cost nothing: an
	// allocator created for zero elements reserves no blocks.
	b3Array_Reserve( world->manifoldAllocators, B3_NEA_MAX_MESH_MANIFOLDS );
	for ( int i = 0; i < B3_NEA_MAX_MESH_MANIFOLDS; ++i )
	{
		int elementSize = ( i + 1 ) * (int)sizeof( b3Manifold );

		int initialCount = 0;
		if ( i == 0 )
		{
			initialCount = contactCapacity;
		}
		else if ( i == B3_NEA_MAX_MESH_MANIFOLDS - 1 )
		{
			initialCount = meshContactCapacity;
		}

		b3BlockAllocator manifoldAllocator = b3CreateBlockAllocator( elementSize, initialCount );
		b3Array_Push( world->manifoldAllocators, manifoldAllocator );
	}

	// The triangle caches, on exactly the same terms and for the same reason.
	// One element is one mesh contact's whole array; the narrow phase never
	// resizes it, so this allocator is touched only when a mesh contact is
	// created or destroyed.
	world->triangleCacheAllocator =
		b3CreateBlockAllocator( B3_NEA_MAX_MESH_CONTACT_TRIANGLES * (int)sizeof( b3TriangleCache ), meshContactCapacity );

	b3CreateBroadPhase( &world->broadPhase, &def->capacity );
	b3CreateGraph( &world->constraintGraph, contactCapacity, jointCapacity );

	// Every pool is reserved from b3Capacity here. Upstream treats these as a
	// hint; the port treats them as the size of the world, because a mid-step
	// allocation is what Phase 4's pool allocator will assert on.
	world->bodyIdPool = b3CreateIdPool();

	b3Array_Reserve( world->bodies, bodyCapacity );
	b3Array_Reserve( world->solverSets, 8 );

	// The three fixed sets, in the order b3SetType names them. The asserts are
	// what pins that order: everything downstream compares set indices against
	// b3_staticSet / b3_disabledSet / b3_awakeSet by value.
	world->solverSetIdPool = b3CreateIdPool();
	b3SolverSet set = { 0 };

	set.setIndex = b3AllocId( &world->solverSetIdPool );
	b3Array_Push( world->solverSets, set );
	b3Array_Reserve( world->solverSets.data[b3_staticSet].bodySims, b3MaxInt( 16, def->capacity.staticBodyCount ) );
	b3Array_Reserve( world->solverSets.data[b3_staticSet].jointSims, jointCapacity );
	B3_ASSERT( world->solverSets.data[b3_staticSet].setIndex == b3_staticSet );

	set.setIndex = b3AllocId( &world->solverSetIdPool );
	b3Array_Push( world->solverSets, set );
	b3Array_Reserve( world->solverSets.data[b3_disabledSet].jointSims, jointCapacity );
	B3_ASSERT( world->solverSets.data[b3_disabledSet].setIndex == b3_disabledSet );

	set.setIndex = b3AllocId( &world->solverSetIdPool );
	b3Array_Push( world->solverSets, set );
	b3Array_Reserve( world->solverSets.data[b3_awakeSet].bodySims, dynamicBodyCapacity );
	b3Array_Reserve( world->solverSets.data[b3_awakeSet].bodyStates, dynamicBodyCapacity );
	b3Array_Reserve( world->solverSets.data[b3_awakeSet].contactIndices, contactCapacity );
	b3Array_Reserve( world->solverSets.data[b3_awakeSet].jointSims, jointCapacity );
	B3_ASSERT( world->solverSets.data[b3_awakeSet].setIndex == b3_awakeSet );

	world->shapeIdPool = b3CreateIdPool();

	b3Array_Reserve( world->shapes, shapeCapacity );

	world->contactIdPool = b3CreateIdPool();
	b3Array_Reserve( world->contacts, contactCapacity );

	world->jointIdPool = b3CreateIdPool();
	b3Array_Reserve( world->joints, jointCapacity );

	world->islandIdPool = b3CreateIdPool();
	b3Array_Reserve( world->islands, dynamicBodyCapacity );

	// The event arrays, sized from the scene rather than from 4.
	//
	// Every one of these is pushed to *during* the step, so a reserve that is
	// too small is a b3Array grow -- a heap allocation -- in the middle of a
	// frame. Four entries meant that happened on the first frame anything
	// touched. Each is reserved from whatever bounds it:
	//
	//   move events    one per awake body that moved
	//   begin/end      bounded by the contact count
	//   hit events     a subset of begin events
	//   joint events   at most one per joint, so the joint capacity
	b3Array_Reserve( world->bodyMoveEvents, dynamicBodyCapacity );
	b3Array_Reserve( world->contactBeginEvents, contactCapacity );
	b3Array_Reserve( world->contactEndEvents[0], contactCapacity );
	b3Array_Reserve( world->contactEndEvents[1], contactCapacity );
	b3Array_Reserve( world->contactHitEvents, contactCapacity );
	b3Array_Reserve( world->jointEvents, jointCapacity );
	world->endEventArrayIndex = 0;

	// The per-step scratch that upstream keeps per worker.
	//
	// b3SetBitCountAndClear grows these to the live count at the top of every
	// step, so the initial size is the only thing standing between the first
	// busy frame and a b3Alloc. Indexed by contact id, shape id and island id
	// respectively, so that is what they are sized from -- not 1024 and 256,
	// which were both too small for a contact-heavy scene and too large for a
	// quiet one.
	world->contactStateBitSet = b3CreateBitSet( (uint32_t)contactCapacity );
	world->hitEventBitSet = b3CreateBitSet( (uint32_t)contactCapacity );
	world->hasHitEvents = false;
	world->jointStateBitSet = b3CreateBitSet( (uint32_t)jointCapacity );
	world->hasJointEvents = false;
	world->enlargedSimBitSet = b3CreateBitSet( (uint32_t)shapeCapacity );
	world->awakeIslandBitSet = b3CreateBitSet( (uint32_t)dynamicBodyCapacity );

	world->stepIndex = 0;
	world->splitIslandId = B3_NULL_INDEX;
	world->splitSleepTime = b3t_zero;

	world->gravity = def->gravity;
	world->hitEventThreshold = def->hitEventThreshold;
	world->restitutionThreshold = def->restitutionThreshold;
	world->maxLinearSpeed = def->maximumLinearSpeed;
	world->maxWorldExtent = def->maximumWorldExtent;
	world->contactSpeed = def->contactSpeed;
	world->contactHertz = def->contactHertz;
	world->contactDampingRatio = def->contactDampingRatio;
	world->contactRecycleDistance = B3_CONTACT_RECYCLE_DISTANCE;

	world->frictionCallback = def->frictionCallback != NULL ? def->frictionCallback : b3DefaultFrictionCallback;
	world->restitutionCallback = def->restitutionCallback != NULL ? def->restitutionCallback : b3DefaultRestitutionCallback;

	world->enableSleep = def->enableSleep;
	world->locked = false;
	world->enableWarmStarting = true;
	world->enableContinuous = def->enableContinuous;
	world->enableSpeculative = true;
	world->userData = def->userData;
	world->maxCapacity = def->capacity;

	// One is added to worldId so that zero can represent a null b3WorldId.
	return ( b3WorldId ){ (uint16_t)( worldId + 1 ), world->generation };
}

void b3DestroyWorld( b3WorldId worldId )
{
	b3World* world = b3GetUnlockedWorldFromId( worldId );
	if ( world == NULL )
	{
		return;
	}

	world->locked = true;

	b3DestroyBitSet( &world->contactStateBitSet );
	b3DestroyBitSet( &world->hitEventBitSet );
	b3DestroyBitSet( &world->jointStateBitSet );
	b3DestroyBitSet( &world->enlargedSimBitSet );
	b3DestroyBitSet( &world->awakeIslandBitSet );

	b3Array_Destroy( world->bodyMoveEvents );
	b3Array_Destroy( world->contactBeginEvents );
	b3Array_Destroy( world->contactEndEvents[0] );
	b3Array_Destroy( world->contactEndEvents[1] );
	b3Array_Destroy( world->contactHitEvents );
	b3Array_Destroy( world->jointEvents );

	b3Array_Destroy( world->bodies );

	int shapeCapacity = world->shapes.count;
	b3Shape* shapes = world->shapes.data;
	for ( int i = 0; i < shapeCapacity; ++i )
	{
		b3Shape* shape = shapes + i;
		if ( shape->id != B3_NULL_INDEX )
		{
			b3DestroyShapeAllocations( world, shape );
		}
	}

	// Upstream also frees each mesh contact's triangle cache here, and asserts
	// the hull database emptied. Neither exists in the port.

	b3Array_Destroy( world->shapes );
	b3Array_Destroy( world->contacts );
	b3Array_Destroy( world->joints );

	for ( int i = 0; i < world->islands.count; ++i )
	{
		b3Array_Destroy( world->islands.data[i].bodies );
		b3Array_Destroy( world->islands.data[i].contacts );
		b3Array_Destroy( world->islands.data[i].joints );
	}
	b3Array_Destroy( world->islands );

	// Every slot, live or retired.
	//
	// b3DestroySolverSet retires a slot without freeing its arrays, so that a
	// sleep/wake cycle reuses them instead of allocating -- which means a
	// retired slot (setIndex == B3_NULL_INDEX) still owns memory. Skipping
	// those here, as the live-only loop used to, would leak every set the world
	// ever put to sleep.
	int setCapacity = world->solverSets.count;
	for ( int i = 0; i < setCapacity; ++i )
	{
		b3FreeSolverSetArrays( world->solverSets.data + i );
	}

	b3Array_Destroy( world->solverSets );

	b3DestroyGraph( &world->constraintGraph );
	b3DestroyBroadPhase( &world->broadPhase );

	b3DestroyIdPool( &world->bodyIdPool );
	b3DestroyIdPool( &world->shapeIdPool );
	b3DestroyIdPool( &world->contactIdPool );
	b3DestroyIdPool( &world->jointIdPool );
	b3DestroyIdPool( &world->islandIdPool );
	b3DestroyIdPool( &world->solverSetIdPool );

	for ( int i = 0; i < world->manifoldAllocators.count; ++i )
	{
		b3DestroyBlockAllocator( world->manifoldAllocators.data + i );
	}
	b3Array_Destroy( world->manifoldAllocators );

	b3DestroyBlockAllocator( &world->triangleCacheAllocator );

	b3DestroyArena( &world->arena );
	b3DestroyStack( &world->stack );

	// Wipe the world, advancing the generation so every outstanding id becomes
	// detectably stale.
	uint16_t generation = world->generation;
	memset( world, 0, sizeof( b3World ) );
	world->generation = generation + 1;
}

// =========================================================================
// Handle validation
// =========================================================================

bool b3World_IsValid( b3WorldId id )
{
	if ( id.index1 < 1 || B3_MAX_WORLDS < id.index1 )
	{
		return false;
	}

	b3World* world = b3_worlds + ( id.index1 - 1 );

	if ( world->worldId != id.index1 - 1 )
	{
		// Not allocated.
		return false;
	}

	return id.generation == world->generation;
}

bool b3Body_IsValid( b3BodyId id )
{
	if ( B3_MAX_WORLDS <= id.world0 )
	{
		return false;
	}

	b3World* world = b3_worlds + id.world0;
	if ( world->worldId != id.world0 )
	{
		return false;
	}

	if ( id.index1 < 1 || world->bodies.count < id.index1 )
	{
		return false;
	}

	b3Body* body = world->bodies.data + ( id.index1 - 1 );
	if ( body->setIndex == B3_NULL_INDEX )
	{
		// Freed.
		return false;
	}

	B3_ASSERT( body->localIndex != B3_NULL_INDEX );

	if ( body->generation != id.generation )
	{
		// Orphaned: the slot was recycled.
		return false;
	}

	return true;
}

bool b3Shape_IsValid( b3ShapeId id )
{
	if ( B3_MAX_WORLDS <= id.world0 )
	{
		return false;
	}

	b3World* world = b3_worlds + id.world0;
	if ( world->worldId != id.world0 )
	{
		return false;
	}

	int shapeId = id.index1 - 1;
	if ( shapeId < 0 || world->shapes.count <= shapeId )
	{
		return false;
	}

	b3Shape* shape = world->shapes.data + shapeId;
	if ( shape->id == B3_NULL_INDEX )
	{
		return false;
	}

	B3_ASSERT( shape->id == shapeId );

	return id.generation == shape->generation;
}

bool b3Joint_IsValid( b3JointId id )
{
	if ( B3_MAX_WORLDS <= id.world0 )
	{
		return false;
	}

	b3World* world = b3_worlds + id.world0;
	if ( world->worldId != id.world0 )
	{
		return false;
	}

	int jointId = id.index1 - 1;
	if ( jointId < 0 || world->joints.count <= jointId )
	{
		return false;
	}

	b3Joint* joint = world->joints.data + jointId;
	if ( joint->jointId == B3_NULL_INDEX )
	{
		return false;
	}

	B3_ASSERT( joint->jointId == jointId );

	return id.generation == joint->generation;
}

bool b3Contact_IsValid( b3ContactId id )
{
	if ( B3_MAX_WORLDS <= id.world0 )
	{
		return false;
	}

	b3World* world = b3_worlds + id.world0;
	if ( world->worldId != id.world0 )
	{
		return false;
	}

	int contactId = id.index1 - 1;
	if ( contactId < 0 || world->contacts.count <= contactId )
	{
		return false;
	}

	b3Contact* contact = world->contacts.data + contactId;
	if ( contact->contactId == B3_NULL_INDEX )
	{
		return false;
	}

	B3_ASSERT( contact->contactId == contactId );

	return id.generation == contact->generation;
}

// =========================================================================
// Tuning
// =========================================================================

void b3World_SetGravity( b3WorldId worldId, b3Vec3 gravity )
{
	b3World* world = b3GetWorldFromId( worldId );
	world->gravity = gravity;
}

b3Vec3 b3World_GetGravity( b3WorldId worldId )
{
	b3World* world = b3GetWorldFromId( worldId );
	return world->gravity;
}

void b3World_EnableSleeping( b3WorldId worldId, bool flag )
{
	b3World* world = b3GetUnlockedWorldFromId( worldId );
	if ( world == NULL )
	{
		return;
	}

	if ( flag == world->enableSleep )
	{
		return;
	}

	world->enableSleep = flag;

	if ( flag == false )
	{
		// Disabling sleep has to wake everything that is already asleep.
		int setCount = world->solverSets.count;
		for ( int i = b3_firstSleepingSet; i < setCount; ++i )
		{
			b3SolverSet* set = b3Array_Get( world->solverSets, i );
			if ( set->bodySims.count > 0 )
			{
				b3WakeSolverSet( world, i );
			}
		}
	}
}

bool b3World_IsSleepingEnabled( b3WorldId worldId )
{
	b3World* world = b3GetWorldFromId( worldId );
	return world->enableSleep;
}

void b3World_EnableWarmStarting( b3WorldId worldId, bool flag )
{
	b3World* world = b3GetUnlockedWorldFromId( worldId );
	if ( world == NULL )
	{
		return;
	}

	world->enableWarmStarting = flag;
}

bool b3World_IsWarmStartingEnabled( b3WorldId worldId )
{
	b3World* world = b3GetWorldFromId( worldId );
	return world->enableWarmStarting;
}

void b3World_EnableContinuous( b3WorldId worldId, bool flag )
{
	b3World* world = b3GetUnlockedWorldFromId( worldId );
	if ( world == NULL )
	{
		return;
	}

	world->enableContinuous = flag;
}

bool b3World_IsContinuousEnabled( b3WorldId worldId )
{
	b3World* world = b3GetWorldFromId( worldId );
	return world->enableContinuous;
}

void b3World_EnableSpeculative( b3WorldId worldId, bool flag )
{
	b3World* world = b3GetUnlockedWorldFromId( worldId );
	if ( world == NULL )
	{
		return;
	}

	world->enableSpeculative = flag;
}

void b3World_SetRestitutionThreshold( b3WorldId worldId, b3f value )
{
	b3World* world = b3GetUnlockedWorldFromId( worldId );
	if ( world == NULL )
	{
		return;
	}

	world->restitutionThreshold = b3MaxF( b3f_zero, value );
}

b3f b3World_GetRestitutionThreshold( b3WorldId worldId )
{
	b3World* world = b3GetWorldFromId( worldId );
	return world->restitutionThreshold;
}

void b3World_SetHitEventThreshold( b3WorldId worldId, b3f value )
{
	b3World* world = b3GetUnlockedWorldFromId( worldId );
	if ( world == NULL )
	{
		return;
	}

	world->hitEventThreshold = b3MaxF( b3f_zero, value );
}

b3f b3World_GetHitEventThreshold( b3WorldId worldId )
{
	b3World* world = b3GetWorldFromId( worldId );
	return world->hitEventThreshold;
}

void b3World_SetContactTuning( b3WorldId worldId, b3f hertz, b3f dampingRatio, b3f contactSpeed )
{
	b3World* world = b3GetUnlockedWorldFromId( worldId );
	if ( world == NULL )
	{
		return;
	}

	world->contactHertz = b3MaxF( b3f_zero, hertz );
	world->contactDampingRatio = b3MaxF( b3f_zero, dampingRatio );
	world->contactSpeed = b3MaxF( b3f_zero, contactSpeed );
}

void b3World_SetContactRecycleDistance( b3WorldId worldId, b3f recycleDistance )
{
	b3World* world = b3GetUnlockedWorldFromId( worldId );
	if ( world == NULL )
	{
		return;
	}

	world->contactRecycleDistance = b3MaxF( b3f_zero, recycleDistance );
}

b3f b3World_GetContactRecycleDistance( b3WorldId worldId )
{
	b3World* world = b3GetWorldFromId( worldId );
	return world->contactRecycleDistance;
}

void b3World_SetMaximumLinearSpeed( b3WorldId worldId, b3f maximumLinearSpeed )
{
	b3World* world = b3GetUnlockedWorldFromId( worldId );
	if ( world == NULL )
	{
		return;
	}

	B3_ASSERT( b3Raw( maximumLinearSpeed ) > 0 );
	world->maxLinearSpeed = maximumLinearSpeed;
}

b3f b3World_GetMaximumLinearSpeed( b3WorldId worldId )
{
	b3World* world = b3GetWorldFromId( worldId );
	return world->maxLinearSpeed;
}

void b3World_SetFrictionCallback( b3WorldId worldId, b3FrictionCallback* callback )
{
	b3World* world = b3GetUnlockedWorldFromId( worldId );
	if ( world == NULL )
	{
		return;
	}

	world->frictionCallback = callback != NULL ? callback : b3DefaultFrictionCallback;
}

void b3World_SetRestitutionCallback( b3WorldId worldId, b3RestitutionCallback* callback )
{
	b3World* world = b3GetUnlockedWorldFromId( worldId );
	if ( world == NULL )
	{
		return;
	}

	world->restitutionCallback = callback != NULL ? callback : b3DefaultRestitutionCallback;
}

void b3World_SetCustomFilterCallback( b3WorldId worldId, b3CustomFilterFcn* fcn, void* context )
{
	b3World* world = b3GetWorldFromId( worldId );
	world->customFilterFcn = fcn;
	world->customFilterContext = context;
}

void b3World_SetPreSolveCallback( b3WorldId worldId, b3PreSolveFcn* fcn, void* context )
{
	b3World* world = b3GetWorldFromId( worldId );
	world->preSolveFcn = fcn;
	world->preSolveContext = context;
}

void b3World_SetUserData( b3WorldId worldId, void* userData )
{
	b3World* world = b3GetWorldFromId( worldId );
	world->userData = userData;
}

void* b3World_GetUserData( b3WorldId worldId )
{
	b3World* world = b3GetWorldFromId( worldId );
	return world->userData;
}

int b3World_GetAwakeBodyCount( b3WorldId worldId )
{
	b3World* world = b3GetWorldFromId( worldId );
	b3SolverSet* awakeSet = b3Array_Get( world->solverSets, b3_awakeSet );
	return awakeSet->bodySims.count;
}

b3Capacity b3World_GetMaxCapacity( b3WorldId worldId )
{
	b3World* world = b3GetWorldFromId( worldId );
	return world->maxCapacity;
}

b3AABB b3World_GetBounds( b3WorldId worldId )
{
	b3World* world = b3GetWorldFromId( worldId );

	b3AABB bounds = b3DynamicTree_GetRootBounds( world->broadPhase.trees + b3_staticBody );
	for ( int i = 1; i < b3_bodyTypeCount; ++i )
	{
		bounds = b3AABB_Union( bounds, b3DynamicTree_GetRootBounds( world->broadPhase.trees + i ) );
	}

	return bounds;
}

void b3World_RebuildStaticTree( b3WorldId worldId )
{
	b3World* world = b3GetUnlockedWorldFromId( worldId );
	if ( world == NULL )
	{
		return;
	}

	b3DynamicTree* staticTree = world->broadPhase.trees + b3_staticBody;
	b3DynamicTree_Rebuild( staticTree, true );
}

// =========================================================================
// The step
// =========================================================================

void b3World_Step( b3WorldId worldId, int subStepCount )
{
	b3World* world = b3GetUnlockedWorldFromId( worldId );
	if ( world == NULL )
	{
		return;
	}

	world->locked = true;

	// Cleared up front so a caller who reads events after an early return sees
	// an empty set rather than the previous step's.
	b3Array_Clear( world->bodyMoveEvents );
	b3Array_Clear( world->contactBeginEvents );
	b3Array_Clear( world->contactHitEvents );
	b3Array_Clear( world->jointEvents );

	// The capacities the world actually reached, for reporting. Upstream also
	// uses these to size the next world; the port treats b3Capacity as the size
	// of the world rather than a hint, so this is purely diagnostic.
	{
		b3Capacity* c = &world->maxCapacity;
		c->staticShapeCount = b3MaxInt( c->staticShapeCount, world->broadPhase.trees[b3_staticBody].proxyCount );
		c->dynamicShapeCount = b3MaxInt( c->dynamicShapeCount, world->broadPhase.trees[b3_dynamicBody].proxyCount );

		int staticBodyCount = world->solverSets.data[b3_staticSet].bodySims.count;
		c->staticBodyCount = b3MaxInt( c->staticBodyCount, staticBodyCount );

		int totalBodyCount = b3GetIdCount( &world->bodyIdPool );
		c->dynamicBodyCount = b3MaxInt( c->dynamicBodyCount, totalBodyCount - staticBodyCount );

		int totalContactCount = b3GetIdCount( &world->contactIdPool );
		c->contactCount = b3MaxInt( c->contactCount, totalContactCount );
	}

	// Find new overlapping pairs and create their contacts.
	b3UpdateBroadPhasePairs( world );

	b3StepContext context = { 0 };
	context.world = world;
	context.graph = &world->constraintGraph;

	// dt is a compile-time constant here, which is the whole reason this
	// function takes only a sub-step count -- see nea_config.h.
	context.subStepCount = b3MaxInt( 1, subStepCount );
	context.dt = B3_NEA_DT;
	context.inv_dt = B3_NEA_INV_DT;
	// Not b3tFromFrac: its body is `((int64_t)num << SHIFT) / den`, and with a
	// runtime denominator that promotes to a 64/64 divide, which is
	// __aeabi_ldivmod on ARM -- a libgcc helper this build is checked not to
	// link. Every other b3*FromFrac call site passes compile-time constants and
	// folds. b3HwDiv64 is the 64-by-32 form, and on device it is the DS's
	// hardware divider rather than a call at all.
	context.h = b3Makeb3t( b3HwDiv64( (int64_t)1 << B3_T_SHIFT, B3_NEA_STEP_HZ * context.subStepCount ) );
	context.inv_h = b3MulFF( b3fFromInt( context.subStepCount ), context.inv_dt );

	// Kept on the world so a reaction query can turn an accumulated impulse
	// back into a force after the step context is gone. See b3World::inv_h.
	world->inv_h = context.inv_h;

	// Upstream reduces the contact hertz for large time steps, capping it at an
	// eighth of the sub-step rate. dt is fixed here, so the cap is a constant
	// comparison rather than a per-step one -- but it is kept, because a caller
	// can still set a contactHertz above it through b3World_SetContactTuning.
	b3f contactHertz = b3MinF( world->contactHertz, b3MulFF( b3fFromFrac( 1, 8 ), context.inv_h ) );
	context.contactSoftness = b3MakeSoft( contactHertz, world->contactDampingRatio, context.h );

	// A dynamic body against a static one is solved twice as stiff, because
	// only one side can move.
	context.staticSoftness =
		b3MakeSoft( b3AddF( contactHertz, contactHertz ), b3MulFF( b3fFromFrac( 1, 2 ), world->contactDampingRatio ),
					context.h );

	context.restitutionThreshold = world->restitutionThreshold;
	context.maxLinearVelocity = world->maxLinearSpeed;
	context.enableWarmStarting = world->enableWarmStarting;

	// Narrow phase.
	b3Collide( world );

	// Integrate, solve, integrate, finalize.
	b3Solve( world, &context );

	// Every stack allocation made this step must have been freed.
	B3_ASSERT( b3GetStackAllocation( &world->stack ) == 0 );
	b3GrowStack( &world->stack );

	// Publish this step's end-touch events and hand the other buffer to the
	// next step. The double buffering is what lets a contact destroyed
	// *between* steps still report its end touch: the destroy writes into the
	// buffer the next step will publish.
	world->endEventArrayIndex = 1 - world->endEventArrayIndex;
	b3Array_Clear( world->contactEndEvents[world->endEventArrayIndex] );

	world->locked = false;
}

b3BodyEvents b3World_GetBodyEvents( b3WorldId worldId )
{
	b3World* world = b3GetWorldFromId( worldId );
	B3_ASSERT( world->locked == false );
	if ( world->locked )
	{
		return ( b3BodyEvents ){ 0 };
	}

	b3BodyEvents events = {
		.moveEvents = world->bodyMoveEvents.data,
		.moveCount = world->bodyMoveEvents.count,
	};
	return events;
}

b3ContactEvents b3World_GetContactEvents( b3WorldId worldId )
{
	b3World* world = b3GetWorldFromId( worldId );
	B3_ASSERT( world->locked == false );
	if ( world->locked )
	{
		return ( b3ContactEvents ){ 0 };
	}

	// The end events are double buffered, so a contact destroyed *between*
	// steps still reports its end touch: this step's array is the one the
	// previous step was not writing.
	int endEventArrayIndex = 1 - world->endEventArrayIndex;

	b3ContactEvents events = {
		.beginEvents = world->contactBeginEvents.data,
		.endEvents = world->contactEndEvents[endEventArrayIndex].data,
		.hitEvents = world->contactHitEvents.data,
		.beginCount = world->contactBeginEvents.count,
		.endCount = world->contactEndEvents[endEventArrayIndex].count,
		.hitCount = world->contactHitEvents.count,
	};
	return events;
}

b3JointEvents b3World_GetJointEvents( b3WorldId worldId )
{
	b3World* world = b3GetWorldFromId( worldId );
	B3_ASSERT( world->locked == false );
	if ( world->locked )
	{
		return ( b3JointEvents ){ 0 };
	}

	// Not double buffered, unlike the end-touch events. A joint event describes
	// a load measured during a step, so there is nothing for a joint destroyed
	// between steps to report -- the analogous case simply does not arise.
	b3JointEvents events = {
		.jointEvents = world->jointEvents.data,
		.jointCount = world->jointEvents.count,
	};
	return events;
}

// =========================================================================
// The narrow phase
// =========================================================================

static inline void b3PrefetchContact( const b3Contact* contact )
{
	// A b3Contact is a couple of hundred bytes and is reached through an
	// index, so every one of these is a random miss. b3Prefetch maps to a real
	// PLD on ARMv5TE (Phase 1), so this is worth keeping -- but the distance
	// of 4 below was tuned against a desktop cache and is a guess here.
	const char* p = (const char*)contact;
	b3Prefetch( p );
	b3Prefetch( p + 64 );
	b3Prefetch( p + 128 );
	b3Prefetch( p + 192 );
}

/// Update every awake contact's manifold and record which ones changed
/// touching state.
///
/// Upstream runs this over a sub-range per worker under b3ParallelFor. One
/// worker here, so it is called once over the whole array; the index
/// parameters are kept so the diff reads against upstream.
static void b3CollideTask( int startIndex, int endIndex, b3World* world, const int* contactIndices )
{
	b3Contact* contacts = world->contacts.data;
	b3Shape* shapes = world->shapes.data;
	b3Body* bodies = world->bodies.data;
	b3BodySim* awakeSims = world->solverSets.data[b3_awakeSet].bodySims.data;
	b3BodySim* staticSims = world->solverSets.data[b3_staticSet].bodySims.data;

	B3_ASSERT( startIndex < endIndex );

	b3f recycleDistance = world->contactRecycleDistance;
	b3f recycleDistanceNonTouching = b3MinF( recycleDistance, B3_SPECULATIVE_DISTANCE );

	const int contactPrefetchDistance = 4;
	int prefetchEnd = endIndex - contactPrefetchDistance;

	for ( int i = startIndex; i < endIndex; ++i )
	{
		if ( i < prefetchEnd )
		{
			b3PrefetchContact( contacts + contactIndices[i + contactPrefetchDistance] );
		}

		int contactIndex = contactIndices[i];
		B3_ASSERT( contactIndex < world->contacts.count );

		b3Contact* contact = contacts + contactIndex;
		B3_ASSERT( contact->contactId == contactIndex );

		b3Shape* shapeA = shapes + contact->shapeIdA;
		b3Shape* shapeB = shapes + contact->shapeIdB;

		// Do the proxies still overlap?
		if ( b3AABB_Overlaps( shapeA->fatAABB, shapeB->fatAABB ) == false )
		{
			// This contact will be destroyed by the state pass below.
			contact->flags |= b3_simDisjoint;
			contact->flags &= ~b3_simTouchingFlag;
			b3SetBit( &world->contactStateBitSet, contactIndex );
			continue;
		}

		b3Body* bodyA = bodies + shapeA->bodyId;
		b3Body* bodyB = bodies + shapeB->bodyId;
		bool isStaticA = bodyA->type == b3_staticBody;
		bool isStaticB = bodyB->type == b3_staticBody;
		bool wasTouching = ( contact->flags & b3_simTouchingFlag ) != 0;

		// A body behind a *touching* awake contact is always either awake or
		// static, so the set lookup can be skipped. A non-touching one can sit
		// against a sleeping body, and cannot.
		b3BodySim* bodySimA;
		b3BodySim* bodySimB;
		if ( wasTouching )
		{
			B3_ASSERT( bodyA->setIndex == b3_awakeSet || bodyA->setIndex == b3_staticSet );
			B3_ASSERT( bodyB->setIndex == b3_awakeSet || bodyB->setIndex == b3_staticSet );
			bodySimA = ( isStaticA ? staticSims : awakeSims ) + bodyA->localIndex;
			bodySimB = ( isStaticB ? staticSims : awakeSims ) + bodyB->localIndex;
		}
		else
		{
			{
				b3SolverSet* set = b3Array_Get( world->solverSets, bodyA->setIndex );
				bodySimA = b3Array_Get( set->bodySims, bodyA->localIndex );
			}
			{
				b3SolverSet* set = b3Array_Get( world->solverSets, bodyB->setIndex );
				bodySimB = b3Array_Get( set->bodySims, bodyB->localIndex );
			}
		}

		b3WorldTransform transformA = bodySimA->transform;
		b3WorldTransform transformB = bodySimB->transform;

		// Used by the contact solver. When an awake body starts touching a
		// sleeping one these are momentarily wrong, and are fixed when the
		// contact is linked into the constraint graph.
		contact->bodySimIndexA = isStaticA ? B3_NULL_INDEX : bodyA->localIndex;
		contact->bodySimIndexB = isStaticB ? B3_NULL_INDEX : bodyB->localIndex;

		b3f recycleTolerance = wasTouching ? recycleDistance : recycleDistanceNonTouching;

		// Set by last step's continuous pass (solver.c). Only the mesh narrow
		// phase reads it, to refuse to replay a cached edge axis: a hull moving
		// fast can rotate around a triangle edge and tunnel through it.
		bool isFast = ( bodySimA->flags & b3_isFast ) != 0 || ( bodySimB->flags & b3_isFast ) != 0;
		bool isMeshContact = ( contact->flags & b3_simMeshContact ) != 0;

		// -----------------------------------------------------------------
		// Contact recycling
		// -----------------------------------------------------------------
		//
		// If neither body has moved or turned enough for the manifold to be
		// meaningfully wrong, keep the anchors and update only the separation.
		// This skips the whole narrow phase, which for a hull pair is the most
		// expensive thing the engine does per step -- and a resting stack is
		// exactly the case where it is entirely redundant.
		//
		// A *fast* mesh contact is excluded: recycling keeps last step's
		// anchors, and a body moving fast enough for the continuous pass to
		// flag it can cross a triangle in a step. Losing this is a tunnelling
		// bug, not a performance one.
		//
		// Three thresholds, at three different scales. See the plan's section
		// 2: getting any of them wrong is silent, because a contact that
		// wrongly recycles still produces a plausible manifold.
		if ( ( isFast == false || isMeshContact == false ) && b3Raw( recycleDistance ) > 0 &&
			 ( contact->flags & b3_relativeTransformValid ) && ( contact->flags & b3_contactRecycleFlag ) )
		{
			// b3DotQuat is Q30; squaring with b3MulNN stays Q30, which is the
			// scale B3_CONTACT_RECYCLE_ANGULAR_DISTANCE is frozen at.
			b3n angleA = b3DotQuat( transformA.q, contact->cachedRotationA );
			b3n angleB = b3DotQuat( transformB.q, contact->cachedRotationB );
			b3c angularDistance = b3MinC( b3NToC( b3MulNN( angleA, angleA ) ), b3NToC( b3MulNN( angleB, angleB ) ) );

			b3Transform xf = b3InvMulWorldTransforms( transformA, transformB );
			b3Transform xfc = contact->cachedRelativePose;

			// Both sides are Q12 lengths, so both squares are Q24. Compared
			// raw in int64 rather than through b3f, which saturates past 512
			// units of separation.
			int64_t distSquared = b3LengthSquaredWide( b3Sub( xf.p, xfc.p ) );
			int64_t toleranceSquared = (int64_t)b3Raw( recycleTolerance ) * b3Raw( recycleTolerance );

			if ( b3Raw( angularDistance ) > b3Raw( B3_CONTACT_RECYCLE_ANGULAR_DISTANCE ) && distSquared < toleranceSquared )
			{
				b3f distance = b3SqrtWide( distSquared );
				b3f slack = b3SubF( recycleTolerance, distance );

				// Conservative advancement: a point at `maxExtent` from the
				// centre sweeps at most 2*|qr.v| x maxExtent under the
				// relative rotation, because 2*|sin(theta/2)| ~= theta.
				//
				// qr = inv(qA0) * qA * ... -- see upstream's derivation; when A
				// is static it reduces to body B's own local rotation delta.
				b3Quat qr = b3InvMulQuat( xfc.q, xf.q );
				b3Vec3 maxExtentA = isStaticA ? b3Vec3_zeroFn() : bodySimA->maxExtent;
				b3Vec3 maxExtentB = isStaticB ? b3Vec3_zeroFn() : bodySimB->maxExtent;
				b3Vec3 maxExtent = b3Max( maxExtentA, maxExtentB );

				b3Vec3 arc = b3ModifiedCrossNF( b3AbsDir( qr.v ), maxExtent );

				// 4 * |arc|^2 < slack^2, both at Q24. The factor of four is
				// the square of upstream's 2*, and is applied to the int64 sum
				// rather than to a b3f, which would overflow for a body more
				// than a few hundred units across.
				int64_t arcSq = 4 * b3LengthSquaredWide( arc );
				int64_t slackSq = (int64_t)b3Raw( slack ) * b3Raw( slack );

				if ( arcSq < slackSq )
				{
					b3Quat dqA = b3MulQuat( transformA.q, b3Conjugate( contact->cachedRotationA ) );
					b3Quat dqB = b3MulQuat( transformB.q, b3Conjugate( contact->cachedRotationB ) );
					b3Matrix3 matrixA = b3MakeMatrixFromQuat( dqA );
					b3Matrix3 matrixB = b3MakeMatrixFromQuat( dqB );

					// Differencing the two centres first keeps the round-off
					// down: both are world positions, and their difference is
					// small where they themselves need not be.
					b3Vec3 dc = b3SubPos( bodySimB->center, bodySimA->center );

					int manifoldCount = contact->manifoldCount;
					for ( int manifoldIndex = 0; manifoldIndex < manifoldCount; ++manifoldIndex )
					{
						b3Manifold* manifold = contact->manifolds + manifoldIndex;
						b3Vec3 normal = manifold->normal;

						int pointCount = manifold->pointCount;
						for ( int pointIndex = 0; pointIndex < pointCount; ++pointIndex )
						{
							// Keep the anchors, move the separation -- the
							// same trick sub-stepping uses, and what stops the
							// contact jittering.
							//
							// The increment is applied to baseSeparation, not
							// to the previous separation, so it does not
							// compound: a rounding error here is bounded by
							// one quantum for as long as the contact stays
							// recycled, rather than accumulating per step.
							// That is why b3Dot's round-to-nearest matters
							// (Phase 1 finding 2) and why this reads from
							// baseSeparation rather than separation.
							b3ManifoldPoint* mp = manifold->points + pointIndex;
							b3Vec3 rA = b3MulMV( matrixA, mp->anchorA );
							b3Vec3 rB = b3MulMV( matrixB, mp->anchorB );
							b3Vec3 dp = b3Add( dc, b3Sub( rB, rA ) );
							mp->separation = b3AddF( mp->baseSeparation, b3Dot( dp, normal ) );
							mp->persisted = true;
						}
					}

					world->recycledContactCount += 1;

					// Recycled: this also skips re-mixing the materials, which
					// is what upstream's comment warns about.
					continue;
				}
			}
		}

		// Cache the pose this manifold was built at, for the test above.
		contact->cachedRotationA = transformA.q;
		contact->cachedRotationB = transformB.q;
		contact->cachedRelativePose = b3InvMulWorldTransforms( transformA, transformB );
		contact->flags |= b3_relativeTransformValid;

		bool touching = b3UpdateContact( world, contact, shapeA, bodySimA->localCenter, transformA, shapeB,
										 bodySimB->localCenter, transformB, isFast, world->arena );

		// The constraint graph caches each contact's manifold count in its
		// b3ContactSpec, written once when the contact entered the colour. That
		// is correct forever for a convex contact, which has one manifold for
		// as long as it is touching. A mesh contact's count is its cluster
		// count and changes as the shape slides across the level, so the cached
		// copy the solver reads has to be refreshed while it stays touching.
		//
		// Not on the start-touching edge: b3AddContactToGraph has just written
		// the count. Not on the stop-touching edge either: the contact is about
		// to leave the colour.
		if ( touching == true && wasTouching == true && ( contact->flags & b3_simMeshContact ) != 0 )
		{
			B3_ASSERT( contact->colorIndex == B3_OVERFLOW_INDEX );
			b3GraphColor* color = world->constraintGraph.colors + contact->colorIndex;
			b3ContactSpec* spec = b3Array_Get( color->contacts, contact->localIndex );
			spec->manifoldCount = (uint16_t)contact->manifoldCount;
		}

		// State changes that affect island connectivity, and therefore events.
		if ( touching == true && wasTouching == false )
		{
			contact->flags |= b3_simStartedTouching;
			b3SetBit( &world->contactStateBitSet, contactIndex );
		}
		else if ( touching == false && wasTouching == true )
		{
			contact->flags |= b3_simStoppedTouching;
			b3SetBit( &world->contactStateBitSet, contactIndex );
		}

		// Snapshot the separations the recycling path will offset from.
		for ( int manifoldIndex = 0; manifoldIndex < contact->manifoldCount; ++manifoldIndex )
		{
			b3Manifold* manifold = contact->manifolds + manifoldIndex;
			for ( int pointIndex = 0; pointIndex < manifold->pointCount; ++pointIndex )
			{
				b3ManifoldPoint* mp = manifold->points + pointIndex;
				mp->baseSeparation = mp->separation;
			}
		}
	}
}

static void b3AddNonTouchingContact( b3World* world, b3Contact* contact )
{
	B3_ASSERT( contact->setIndex == b3_awakeSet );
	b3SolverSet* set = b3Array_Get( world->solverSets, b3_awakeSet );
	contact->colorIndex = B3_NULL_INDEX;
	contact->localIndex = set->contactIndices.count;
	contact->bodySimIndexA = B3_NULL_INDEX;
	contact->bodySimIndexB = B3_NULL_INDEX;
	b3Array_Push( set->contactIndices, contact->contactId );
}

static void b3RemoveNonTouchingContact( b3World* world, int setIndex, int localIndex )
{
	b3SolverSet* set = b3Array_Get( world->solverSets, setIndex );
	int movedIndex = b3Array_RemoveSwap( set->contactIndices, localIndex );
	if ( movedIndex != B3_NULL_INDEX )
	{
		int movedContactIndex = set->contactIndices.data[localIndex];
		b3Contact* movedContact = b3Array_Get( world->contacts, movedContactIndex );
		B3_ASSERT( movedContact->setIndex == setIndex );
		B3_ASSERT( movedContact->colorIndex == B3_NULL_INDEX );
		B3_ASSERT( movedContact->localIndex == movedIndex );
		movedContact->localIndex = localIndex;
	}
}

void b3Collide( b3World* world )
{
	world->recycledContactCount = 0;
	world->meshManifoldDropCount = 0;
	world->parkedBodyCount = 0;

	// Phase 3B had this pass clear the begin/hit events and swap the end-event
	// buffers, because there was no step to own them. b3World_Step exists as of
	// 3C-i and does both, where upstream has them.

	// Gather every awake contact -- touching ones from the constraint graph,
	// non-touching ones from the awake set -- into one flat array, so the
	// update loop is a single pass over indices.
	int touchingCount = 0;

	b3GraphColor* graphColors = world->constraintGraph.colors;
	for ( int i = 0; i < B3_GRAPH_COLOR_COUNT; ++i )
	{
		// Upstream also counts `convexContacts`, the wide solver's array. One
		// solver path here, so there is one array (3A).
		touchingCount += graphColors[i].contacts.count;
	}

	b3SolverSet* awakeSet = b3Array_Get( world->solverSets, b3_awakeSet );
	int nonTouchingCount = awakeSet->contactIndices.count;

	int contactCount = touchingCount + nonTouchingCount;

	if ( contactCount == 0 )
	{
		return;
	}

	int* contactIndices = (int*)b3StackAlloc( &world->stack, contactCount * (int)sizeof( int ), "contact indices" );

	int contactIndex = 0;
	for ( int i = 0; i < B3_GRAPH_COLOR_COUNT; ++i )
	{
		b3GraphColor* color = graphColors + i;
		int count = color->contacts.count;
		for ( int j = 0; j < count; ++j )
		{
			contactIndices[contactIndex] = color->contacts.data[j].contactId;
			contactIndex += 1;
		}
	}

	B3_ASSERT( contactIndex == touchingCount );

	if ( nonTouchingCount > 0 )
	{
		memcpy( contactIndices + touchingCount, awakeSet->contactIndices.data, nonTouchingCount * sizeof( int ) );
	}

	// The bit set is keyed on contact *ids*, not on positions in the array
	// above, because a contact moves between the touching and non-touching
	// arrays as a result of this very pass.
	int contactIdCapacity = b3GetIdCapacity( &world->contactIdPool );
	b3SetBitCountAndClear( &world->contactStateBitSet, (uint32_t)contactIdCapacity );

	b3CollideTask( 0, contactCount, world, contactIndices );

	b3StackFree( &world->stack, contactIndices );
	contactIndices = NULL;

	// Release this step's arena overflow blocks and grow the backing block if
	// demand exceeded it. The narrow phase is the only consumer and is done.
	b3ArenaSync( &world->arena );

	// Upstream unions the per-worker bit sets here and sums the SAT counters.
	// One worker, and the counters went with b3Profile in 3A.
	b3BitSet* bitSet = &world->contactStateBitSet;

	int endEventArrayIndex = world->endEventArrayIndex;
	const b3Shape* shapes = world->shapes.data;
	uint16_t worldId = world->worldId;

	// Apply the state changes serially, walking the set bits.
	for ( uint32_t k = 0; k < bitSet->blockCount; ++k )
	{
		uint64_t bits = bitSet->bits[k];
		while ( bits != 0 )
		{
			uint32_t ctz = b3CTZ64( bits );
			int contactId = (int)( 64 * k + ctz );

			b3Contact* contact = b3Array_Get( world->contacts, contactId );
			B3_ASSERT( contact->setIndex == b3_awakeSet );

			const b3Shape* shapeA = shapes + contact->shapeIdA;
			const b3Shape* shapeB = shapes + contact->shapeIdB;
			b3ShapeId shapeIdA = { shapeA->id + 1, worldId, shapeA->generation };
			b3ShapeId shapeIdB = { shapeB->id + 1, worldId, shapeB->generation };
			b3ContactId contactFullId = {
				.index1 = contactId + 1,
				.world0 = worldId,
				.padding = 0,
				.generation = contact->generation,
			};
			uint32_t flags = contact->flags;

			if ( flags & b3_simDisjoint )
			{
				// The fat AABBs stopped overlapping.
				b3DestroyContact( world, contact, false );
				contact = NULL;
			}
			else if ( flags & b3_simStartedTouching )
			{
				B3_ASSERT( contact->islandId == B3_NULL_INDEX );

				if ( flags & b3_contactEnableContactEvents )
				{
					b3ContactBeginTouchEvent event = { shapeIdA, shapeIdB, contactFullId };
					b3Array_Push( world->contactBeginEvents, event );
				}

				B3_ASSERT( contact->manifoldCount > 0 );
				B3_ASSERT( contact->setIndex == b3_awakeSet );

				// Link first: this wakes the colliding bodies and puts their
				// sims where the graph expects to find them.
				contact->flags &= ~b3_simStartedTouching;
				contact->flags |= b3_contactTouchingFlag;
				b3LinkContact( world, contact );

				B3_ASSERT( contact->colorIndex == B3_NULL_INDEX );

				int oldLocalIndex = contact->localIndex;

				b3AddContactToGraph( world, contact );
				b3RemoveNonTouchingContact( world, b3_awakeSet, oldLocalIndex );
			}
			else if ( flags & b3_simStoppedTouching )
			{
				contact->flags &= ~b3_simStoppedTouching;
				contact->flags &= ~b3_contactTouchingFlag;

				if ( contact->flags & b3_contactEnableContactEvents )
				{
					b3ContactEndTouchEvent event = { shapeIdA, shapeIdB, contactFullId };
					b3Array_Push( world->contactEndEvents[endEventArrayIndex], event );
				}

				B3_ASSERT( contact->manifoldCount == 0 );

				// Cached because b3UnlinkContact clears them.
				int colorIndex = contact->colorIndex;
				int localIndex = contact->localIndex;

				b3UnlinkContact( world, contact );
				int bodyIdA = contact->edges[0].bodyId;
				int bodyIdB = contact->edges[1].bodyId;

				b3AddNonTouchingContact( world, contact );

				b3RemoveContactFromGraph( world, bodyIdA, bodyIdB, colorIndex, localIndex );
				contact = NULL;
			}

			// Clear the lowest set bit.
			bits = bits & ( bits - 1 );
		}
	}

	b3ValidateSolverSets( world );
	b3ValidateContacts( world );
}

// =========================================================================
// Validation
// =========================================================================

#if B3_ENABLE_VALIDATION

void b3ValidateConnectivity( b3World* world )
{
	b3Body* bodies = world->bodies.data;
	int bodyCapacity = world->bodies.count;

	for ( int bodyIndex = 0; bodyIndex < bodyCapacity; ++bodyIndex )
	{
		b3Body* body = bodies + bodyIndex;
		if ( body->id == B3_NULL_INDEX )
		{
			b3ValidateFreeId( &world->bodyIdPool, bodyIndex );
			continue;
		}

		B3_ASSERT( bodyIndex == body->id );

		int bodyIslandId = body->islandId;
		int bodySetIndex = body->setIndex;

		int contactKey = body->headContactKey;
		while ( contactKey != B3_NULL_INDEX )
		{
			int contactId = contactKey >> 1;
			int edgeIndex = contactKey & 1;

			b3Contact* contact = b3Array_Get( world->contacts, contactId );

			bool touching = ( contact->flags & b3_contactTouchingFlag ) != 0;
			if ( touching )
			{
				if ( bodySetIndex != b3_staticSet )
				{
					int contactIslandId = contact->islandId;
					B3_ASSERT( contactIslandId == bodyIslandId );
				}
			}
			else
			{
				B3_ASSERT( contact->islandId == B3_NULL_INDEX );
			}

			contactKey = contact->edges[edgeIndex].nextKey;
		}

		int jointKey = body->headJointKey;
		while ( jointKey != B3_NULL_INDEX )
		{
			int jointId = jointKey >> 1;
			int edgeIndex = jointKey & 1;

			b3Joint* joint = b3Array_Get( world->joints, jointId );

			int otherEdgeIndex = edgeIndex ^ 1;

			b3Body* otherBody = b3Array_Get( world->bodies, joint->edges[otherEdgeIndex].bodyId );

			if ( bodySetIndex == b3_disabledSet || otherBody->setIndex == b3_disabledSet )
			{
				B3_ASSERT( joint->islandId == B3_NULL_INDEX );
			}
			else if ( bodySetIndex == b3_staticSet )
			{
				// Deliberate nesting: a static-static joint has no island, but
				// a static-dynamic one belongs to the dynamic body's island,
				// which this branch says nothing about.
				if ( otherBody->setIndex == b3_staticSet )
				{
					B3_ASSERT( joint->islandId == B3_NULL_INDEX );
				}
			}
			else if ( body->type != b3_dynamicBody && otherBody->type != b3_dynamicBody )
			{
				B3_ASSERT( joint->islandId == B3_NULL_INDEX );
			}
			else
			{
				int jointIslandId = joint->islandId;
				B3_ASSERT( jointIslandId == bodyIslandId );
			}

			jointKey = joint->edges[edgeIndex].nextKey;
		}
	}
}

// Validates solver sets, but not island connectivity.
void b3ValidateSolverSets( b3World* world )
{
	B3_ASSERT( b3GetIdCapacity( &world->bodyIdPool ) == world->bodies.count );
	B3_ASSERT( b3GetIdCapacity( &world->contactIdPool ) == world->contacts.count );
	B3_ASSERT( b3GetIdCapacity( &world->jointIdPool ) == world->joints.count );
	B3_ASSERT( b3GetIdCapacity( &world->islandIdPool ) == world->islands.count );
	B3_ASSERT( b3GetIdCapacity( &world->solverSetIdPool ) == world->solverSets.count );

	int activeSetCount = 0;
	int totalBodyCount = 0;
	int totalJointCount = 0;
	int totalContactCount = 0;
	int totalIslandCount = 0;

	int setCount = world->solverSets.count;
	for ( int setIndex = 0; setIndex < setCount; ++setIndex )
	{
		b3SolverSet* set = world->solverSets.data + setIndex;
		if ( set->setIndex != B3_NULL_INDEX )
		{
			activeSetCount += 1;

			if ( setIndex == b3_staticSet )
			{
				B3_ASSERT( set->contactIndices.count == 0 );
				B3_ASSERT( set->islandSims.count == 0 );
				B3_ASSERT( set->bodyStates.count == 0 );
			}
			else if ( setIndex == b3_disabledSet )
			{
				B3_ASSERT( set->islandSims.count == 0 );
				B3_ASSERT( set->bodyStates.count == 0 );
			}
			else if ( setIndex == b3_awakeSet )
			{
				B3_ASSERT( set->bodySims.count == set->bodyStates.count );
				B3_ASSERT( set->jointSims.count == 0 );
			}
			else
			{
				B3_ASSERT( set->bodyStates.count == 0 );
			}

			// Bodies
			{
				b3Body* bodies = world->bodies.data;
				B3_ASSERT( set->bodySims.count >= 0 );
				totalBodyCount += set->bodySims.count;
				for ( int i = 0; i < set->bodySims.count; ++i )
				{
					b3BodySim* bodySim = set->bodySims.data + i;

					int bodyId = bodySim->bodyId;
					B3_ASSERT( 0 <= bodyId && bodyId < world->bodies.count );
					b3Body* body = bodies + bodyId;
					B3_ASSERT( body->setIndex == setIndex );
					B3_ASSERT( body->localIndex == i );

					uint32_t syncedFlags = body->flags & ~b3_bodyTransientFlags;
					B3_ASSERT( ( bodySim->flags & syncedFlags ) == syncedFlags );

					b3BodyState* bodyState = b3GetBodyState( world, body );
					if ( bodyState != NULL )
					{
						B3_ASSERT( ( bodyState->flags & syncedFlags ) == syncedFlags );
					}

					if ( body->type == b3_dynamicBody )
					{
						B3_ASSERT( body->flags & b3_dynamicFlag );
					}

					if ( setIndex == b3_disabledSet )
					{
						B3_ASSERT( body->headContactKey == B3_NULL_INDEX );
					}

					// Shapes
					int prevShapeId = B3_NULL_INDEX;
					int shapeId = body->headShapeId;
					while ( shapeId != B3_NULL_INDEX )
					{
						b3Shape* shape = b3Array_Get( world->shapes, shapeId );
						B3_ASSERT( shape->id == shapeId );
						B3_ASSERT( shape->prevShapeId == prevShapeId );

						if ( setIndex == b3_disabledSet )
						{
							B3_ASSERT( shape->proxyKey == B3_NULL_INDEX );
						}
						else if ( setIndex == b3_staticSet )
						{
							B3_ASSERT( B3_PROXY_TYPE( shape->proxyKey ) == b3_staticBody );
						}
						else
						{
							b3BodyType proxyType = B3_PROXY_TYPE( shape->proxyKey );
							B3_ASSERT( proxyType == b3_kinematicBody || proxyType == b3_dynamicBody );
						}

						prevShapeId = shapeId;
						shapeId = shape->nextShapeId;
					}

					// Contacts
					int contactKey = body->headContactKey;
					while ( contactKey != B3_NULL_INDEX )
					{
						int contactId = contactKey >> 1;
						int edgeIndex = contactKey & 1;

						b3Contact* contact = b3Array_Get( world->contacts, contactId );
						B3_ASSERT( contact->setIndex != b3_staticSet );
						B3_ASSERT( contact->edges[0].bodyId == bodyId || contact->edges[1].bodyId == bodyId );

						contactKey = contact->edges[edgeIndex].nextKey;
					}

					// Joints
					int jointKey = body->headJointKey;
					while ( jointKey != B3_NULL_INDEX )
					{
						int jointId = jointKey >> 1;
						int edgeIndex = jointKey & 1;

						b3Joint* joint = b3Array_Get( world->joints, jointId );

						int otherEdgeIndex = edgeIndex ^ 1;

						b3Body* otherBody = b3Array_Get( world->bodies, joint->edges[otherEdgeIndex].bodyId );

						if ( setIndex == b3_disabledSet || otherBody->setIndex == b3_disabledSet )
						{
							B3_ASSERT( joint->setIndex == b3_disabledSet );
						}
						else if ( setIndex == b3_staticSet && otherBody->setIndex == b3_staticSet )
						{
							B3_ASSERT( joint->setIndex == b3_staticSet );
						}
						else if ( body->type != b3_dynamicBody && otherBody->type != b3_dynamicBody )
						{
							B3_ASSERT( joint->setIndex == b3_staticSet );
						}
						else if ( setIndex == b3_awakeSet )
						{
							B3_ASSERT( joint->setIndex == b3_awakeSet );
						}
						else if ( setIndex >= b3_firstSleepingSet )
						{
							B3_ASSERT( joint->setIndex == setIndex );
						}

						b3JointSim* jointSim = b3GetJointSim( world, joint );
						B3_ASSERT( jointSim->jointId == jointId );
						B3_ASSERT( jointSim->bodyIdA == joint->edges[0].bodyId );
						B3_ASSERT( jointSim->bodyIdB == joint->edges[1].bodyId );

						jointKey = joint->edges[edgeIndex].nextKey;
					}
				}
			}

			// Contacts held by the set itself: non-touching in the awake set,
			// all of them in a sleeping set.
			{
				B3_ASSERT( set->contactIndices.count >= 0 );
				totalContactCount += set->contactIndices.count;
				for ( int i = 0; i < set->contactIndices.count; ++i )
				{
					int contactIndex = set->contactIndices.data[i];
					b3Contact* contact = b3Array_Get( world->contacts, contactIndex );
					if ( setIndex == b3_awakeSet )
					{
						// Non-touching, or touching and not yet transferred.
						B3_ASSERT( contact->manifoldCount == 0 || ( contact->flags & b3_simStartedTouching ) != 0 );
					}
					B3_ASSERT( contact->setIndex == setIndex );
					B3_ASSERT( contact->colorIndex == B3_NULL_INDEX );
					B3_ASSERT( contact->localIndex == i );
				}
			}

			// Joints
			{
				B3_ASSERT( set->jointSims.count >= 0 );
				totalJointCount += set->jointSims.count;
				for ( int i = 0; i < set->jointSims.count; ++i )
				{
					b3JointSim* jointSim = set->jointSims.data + i;
					b3Joint* joint = b3Array_Get( world->joints, jointSim->jointId );
					B3_ASSERT( joint->setIndex == setIndex );
					B3_ASSERT( joint->colorIndex == B3_NULL_INDEX );
					B3_ASSERT( joint->localIndex == i );
				}
			}

			// Islands
			{
				B3_ASSERT( set->islandSims.count >= 0 );
				totalIslandCount += set->islandSims.count;
				for ( int i = 0; i < set->islandSims.count; ++i )
				{
					b3IslandSim* islandSim = set->islandSims.data + i;
					b3Island* island = b3Array_Get( world->islands, islandSim->islandId );
					B3_ASSERT( island->setIndex == setIndex );
					B3_ASSERT( island->localIndex == i );
				}
			}
		}
		else
		{
			B3_ASSERT( set->bodySims.count == 0 );
			B3_ASSERT( set->contactIndices.count == 0 );
			B3_ASSERT( set->jointSims.count == 0 );
			B3_ASSERT( set->islandSims.count == 0 );
			B3_ASSERT( set->bodyStates.count == 0 );
		}
	}

	int setIdCount = b3GetIdCount( &world->solverSetIdPool );
	B3_ASSERT( activeSetCount == setIdCount );

	int bodyIdCount = b3GetIdCount( &world->bodyIdPool );
	B3_ASSERT( totalBodyCount == bodyIdCount );

	int islandIdCount = b3GetIdCount( &world->islandIdPool );
	B3_ASSERT( totalIslandCount == islandIdCount );

	// The constraint graph. Upstream also checks the per-colour body bitset
	// population against the constraints in the colour; there is no bitset at
	// one colour, so that half of the check has nothing to compare.
	for ( int colorIndex = 0; colorIndex < B3_GRAPH_COLOR_COUNT; ++colorIndex )
	{
		b3GraphColor* color = world->constraintGraph.colors + colorIndex;

		totalContactCount += color->contacts.count;
		for ( int i = 0; i < color->contacts.count; ++i )
		{
			int contactId = color->contacts.data[i].contactId;
			b3Contact* contact = b3Array_Get( world->contacts, contactId );

			// Touching, or awaiting transfer back to non-touching.
			B3_ASSERT( contact->manifoldCount > 0 ||
					   ( contact->flags & ( b3_simStoppedTouching | b3_simDisjoint ) ) != 0 );
			B3_ASSERT( contact->setIndex == b3_awakeSet );
			B3_ASSERT( contact->colorIndex == colorIndex );
			B3_ASSERT( contact->localIndex == i );
		}

		B3_ASSERT( color->jointSims.count >= 0 );
		totalJointCount += color->jointSims.count;
		for ( int i = 0; i < color->jointSims.count; ++i )
		{
			b3JointSim* jointSim = color->jointSims.data + i;
			b3Joint* joint = b3Array_Get( world->joints, jointSim->jointId );
			B3_ASSERT( joint->setIndex == b3_awakeSet );
			B3_ASSERT( joint->colorIndex == colorIndex );
			B3_ASSERT( joint->localIndex == i );
		}
	}

	int contactIdCount = b3GetIdCount( &world->contactIdPool );
	B3_ASSERT( totalContactCount == contactIdCount );
	B3_ASSERT( totalContactCount == (int)world->broadPhase.pairSet.count );

	int jointIdCount = b3GetIdCount( &world->jointIdPool );
	B3_ASSERT( totalJointCount == jointIdCount );
}

// Validate contact touching status.
void b3ValidateContacts( b3World* world )
{
	b3ConstraintGraph* graph = &world->constraintGraph;
	int contactCount = world->contacts.count;
	B3_ASSERT( contactCount == b3GetIdCapacity( &world->contactIdPool ) );
	int allocatedContactCount = 0;

	for ( int contactIndex = 0; contactIndex < contactCount; ++contactIndex )
	{
		b3Contact* contact = b3Array_Get( world->contacts, contactIndex );
		if ( contact->contactId == B3_NULL_INDEX )
		{
			continue;
		}

		B3_ASSERT( contact->contactId == contactIndex );

		allocatedContactCount += 1;

		bool touching = ( contact->flags & b3_contactTouchingFlag ) != 0;

		int setId = contact->setIndex;
		b3SolverSet* set = b3Array_Get( world->solverSets, setId );

		if ( setId == b3_awakeSet )
		{
			if ( touching )
			{
				B3_ASSERT( 0 <= contact->colorIndex && contact->colorIndex < B3_GRAPH_COLOR_COUNT );

				b3Shape* shapeA = b3Array_Get( world->shapes, contact->shapeIdA );
				b3Shape* shapeB = b3Array_Get( world->shapes, contact->shapeIdB );

				b3Body* bodyA = b3Array_Get( world->bodies, shapeA->bodyId );
				b3Body* bodyB = b3Array_Get( world->bodies, shapeB->bodyId );

				if ( bodyA->type == b3_staticBody )
				{
					B3_ASSERT( contact->bodySimIndexA == B3_NULL_INDEX );
				}
				else
				{
					B3_ASSERT( contact->bodySimIndexA == bodyA->localIndex );
				}

				if ( bodyB->type == b3_staticBody )
				{
					B3_ASSERT( contact->bodySimIndexB == B3_NULL_INDEX );
				}
				else
				{
					B3_ASSERT( contact->bodySimIndexB == bodyB->localIndex );
				}

				b3GraphColor* color = graph->colors + contact->colorIndex;
				int contactId = b3Array_Get( color->contacts, contact->localIndex )->contactId;
				B3_ASSERT( contactId == contactIndex );
			}
			else
			{
				B3_ASSERT( contact->colorIndex == B3_NULL_INDEX );

				// A non-touching contact carries no points. It carries no
				// *allocation* either -- unless it is a mesh contact, which
				// holds its manifold block from creation to destruction so that
				// a body sliding on and off the level never reaches the
				// allocator mid-step. So the count is the invariant; the
				// pointer only is for convex pairs.
				B3_ASSERT( contact->manifoldCount == 0 );
				B3_ASSERT( ( contact->flags & b3_simMeshContact ) != 0 || contact->manifolds == NULL );

				int* index = b3Array_Get( set->contactIndices, contact->localIndex );
				B3_ASSERT( *index == contactIndex );
			}
		}
		else if ( setId >= b3_firstSleepingSet )
		{
			// A sleeping set holds only touching contacts.
			B3_ASSERT( touching == true );
			B3_ASSERT( contact->manifolds != NULL );
			B3_ASSERT( contact->manifoldCount > 0 );
			int* index = b3Array_Get( set->contactIndices, contact->localIndex );
			B3_ASSERT( *index == contactIndex );
		}
		else
		{
			// Sleeping non-touching contacts belong to the disabled set.
			B3_ASSERT( touching == false && setId == b3_disabledSet );
			B3_ASSERT( contact->manifoldCount == 0 );
			B3_ASSERT( ( contact->flags & b3_simMeshContact ) != 0 || contact->manifolds == NULL );
			int* index = b3Array_Get( set->contactIndices, contact->localIndex );
			B3_ASSERT( *index == contactIndex );
		}
	}

	int contactIdCount = b3GetIdCount( &world->contactIdPool );
	B3_ASSERT( allocatedContactCount == contactIdCount );
}

#else

void b3ValidateConnectivity( b3World* world )
{
	B3_UNUSED( world );
}

void b3ValidateSolverSets( b3World* world )
{
	B3_UNUSED( world );
}

void b3ValidateContacts( b3World* world )
{
	B3_UNUSED( world );
}

#endif
