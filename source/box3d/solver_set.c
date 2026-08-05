// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Erin Catto        (original Box3D)
// Copyright (c) 2026 Warioware64       (Nitro Engine Advanced fixed-point port)
//
// This file is part of Nitro Engine Advanced

/// @file   solver_set.c
/// @brief  Moving bodies, contacts, joints and islands between solver sets.
///
/// Almost all of this is integer bookkeeping and transliterates exactly. The
/// one place it does not is the single graph colour: with B3_GRAPH_COLOR_COUNT
/// at 1 every constraint is an overflow constraint, so the per-colour body
/// bitsets that keep two constraints on the same body out of one colour have
/// no work to do and are gone, and with them the `colorIndex != B3_OVERFLOW_INDEX`
/// branches in b3TrySleepIsland. The convex/mesh split in the same function
/// goes for the same reason: upstream routes wide-solved convex contacts
/// through a separate array, and the port has no wide path.

#include "solver_set.h"

#include "body.h"
#include "constraint_graph.h"
#include "contact.h"
#include "core.h"
#include "id_pool.h"
#include "island.h"
#include "joint.h"
#include "physics_world.h"

#include <string.h>

void b3FreeSolverSetArrays( b3SolverSet* set )
{
	b3Array_Destroy( set->bodySims );
	b3Array_Destroy( set->bodyStates );
	b3Array_Destroy( set->contactIndices );
	b3Array_Destroy( set->jointSims );
	b3Array_Destroy( set->islandSims );
}

void b3DestroySolverSet( b3World* world, int setIndex )
{
	// Retire the slot but **keep its arrays**.
	//
	// Upstream frees them here. That is right for a desktop and wrong for this
	// port, because the slot is going straight back to solverSetIdPool and the
	// next island to fall asleep will claim it and immediately re-reserve the
	// same five arrays at roughly the same sizes. A five-box stack going to
	// sleep and waking again cost 4.3 KB of allocate-and-free per cycle,
	// forever -- inside b3World_Step, which is where NEA_Phys3D's pool
	// allocator refuses to allocate.
	//
	// Keeping the capacity makes a sleep/wake cycle free after the first one of
	// each size. The memory is not lost: the arrays belong to a fixed slot in
	// world->solverSets, they are bounded by the largest island the scene has
	// ever had, and b3DestroyWorld releases every slot's arrays whether the
	// slot is live or retired.
	b3SolverSet* set = b3Array_Get( world->solverSets, setIndex );

	b3Array_Clear( set->bodySims );
	b3Array_Clear( set->bodyStates );
	b3Array_Clear( set->contactIndices );
	b3Array_Clear( set->jointSims );
	b3Array_Clear( set->islandSims );

	b3FreeId( &world->solverSetIdPool, setIndex );
	set->setIndex = B3_NULL_INDEX;
}

// Wake a solver set. Does not merge islands.
//
// Contacts can be in three places:
//   1. non-touching contacts in the disabled set
//   2. non-touching contacts already in the awake set
//   3. touching contacts in the sleeping set
// This handles 1 and 3; 2 needs nothing.
void b3WakeSolverSet( b3World* world, int setIndex )
{
	B3_ASSERT( setIndex >= b3_firstSleepingSet );
	b3SolverSet* set = b3Array_Get( world->solverSets, setIndex );
	b3SolverSet* awakeSet = b3Array_Get( world->solverSets, b3_awakeSet );
	b3SolverSet* disabledSet = b3Array_Get( world->solverSets, b3_disabledSet );

	b3Body* bodies = world->bodies.data;

	int bodyCount = set->bodySims.count;
	for ( int i = 0; i < bodyCount; ++i )
	{
		b3BodySim* simSrc = set->bodySims.data + i;

		b3Body* body = bodies + simSrc->bodyId;
		B3_ASSERT( body->setIndex == setIndex );
		body->setIndex = b3_awakeSet;
		body->localIndex = awakeSet->bodySims.count;

		// Reset the sleep timer.
		body->sleepTime = b3t_zero;

		b3BodySim* simDst = b3Array_Emplace( awakeSet->bodySims );
		memcpy( simDst, simSrc, sizeof( b3BodySim ) );

		b3BodyState* state = b3Array_Emplace( awakeSet->bodyStates );
		*state = b3_identityBodyState;
		state->flags = body->flags;

		// Move non-touching contacts from the disabled set to the awake set.
		int contactKey = body->headContactKey;
		while ( contactKey != B3_NULL_INDEX )
		{
			int edgeIndex = contactKey & 1;
			int contactId = contactKey >> 1;

			b3Contact* contact = b3Array_Get( world->contacts, contactId );
			contactKey = contact->edges[edgeIndex].nextKey;

			if ( contact->setIndex != b3_disabledSet )
			{
				B3_ASSERT( contact->setIndex == b3_awakeSet || contact->setIndex == setIndex );
				continue;
			}

			int localIndex = contact->localIndex;

			B3_ASSERT( 0 <= localIndex && localIndex < disabledSet->contactIndices.count );
			B3_ASSERT( disabledSet->contactIndices.data[localIndex] == contactId );
			B3_ASSERT( ( contact->flags & b3_contactTouchingFlag ) == 0 && contact->manifoldCount == 0 );

			contact->setIndex = b3_awakeSet;
			contact->localIndex = awakeSet->contactIndices.count;
			b3Array_Push( awakeSet->contactIndices, contactId );

			int movedLocalIndex = b3Array_RemoveSwap( disabledSet->contactIndices, localIndex );
			if ( movedLocalIndex != B3_NULL_INDEX )
			{
				int movedContactIndex = disabledSet->contactIndices.data[localIndex];
				b3Contact* movedContact = b3Array_Get( world->contacts, movedContactIndex );
				B3_ASSERT( movedContact->localIndex == movedLocalIndex );
				movedContact->localIndex = localIndex;
			}
		}
	}

	// Transfer touching contacts from the sleeping set to the constraint graph.
	{
		int contactCount = set->contactIndices.count;
		for ( int i = 0; i < contactCount; ++i )
		{
			int contactIndex = set->contactIndices.data[i];
			b3Contact* contact = b3Array_Get( world->contacts, contactIndex );
			B3_ASSERT( contact->flags & b3_contactTouchingFlag );
			B3_ASSERT( contact->flags & b3_simTouchingFlag );
			B3_ASSERT( contact->setIndex == setIndex );
			b3AddContactToGraph( world, contact );
			contact->setIndex = b3_awakeSet;
		}
	}

	// Transfer joints from the sleeping set to the awake set.
	{
		int jointCount = set->jointSims.count;
		for ( int i = 0; i < jointCount; ++i )
		{
			b3JointSim* jointSim = set->jointSims.data + i;
			b3Joint* joint = b3Array_Get( world->joints, jointSim->jointId );
			B3_ASSERT( joint->setIndex == setIndex );
			b3AddJointToGraph( world, jointSim, joint );
			joint->setIndex = b3_awakeSet;
		}
	}

	// Transfer islands. A sleeping set usually has one, but joints created
	// between sleeping islands move them into the same set.
	{
		int islandCount = set->islandSims.count;
		for ( int i = 0; i < islandCount; ++i )
		{
			b3IslandSim* islandSrc = set->islandSims.data + i;
			b3Island* island = b3Array_Get( world->islands, islandSrc->islandId );
			island->setIndex = b3_awakeSet;
			island->localIndex = awakeSet->islandSims.count;
			b3IslandSim* islandDst = b3Array_Emplace( awakeSet->islandSims );
			memcpy( islandDst, islandSrc, sizeof( b3IslandSim ) );
		}
	}

	b3DestroySolverSet( world, setIndex );
}

bool b3TrySleepIsland( b3World* world, int islandId )
{
	b3Island* island = b3Array_Get( world->islands, islandId );
	B3_ASSERT( island->setIndex == b3_awakeSet );

	// An island with a pending split and more than one body cannot sleep.
	//
	// **Returning false rather than void is load-bearing.** The caller in
	// b3Solve walks the awake island array and steps its index back after each
	// call, because an island that sleeps is swap-removed and a new one lands
	// in the slot just visited. When this early-out fires, nothing is removed --
	// so a void return left that caller stepping back onto the same island,
	// with its awake bit still clear, calling this function on it forever.
	// That was the physics examples' second softlock, and it needed only a
	// scene where contacts are made and broken often enough to keep
	// constraintRemoveCount above zero.
	if ( island->constraintRemoveCount > 0 && island->bodies.count > 1 )
	{
		return false;
	}

	int sleepSetId = b3AllocId( &world->solverSetIdPool );
	if ( sleepSetId == world->solverSets.count )
	{
		b3SolverSet set = { 0 };
		set.setIndex = B3_NULL_INDEX;
		b3Array_Push( world->solverSets, set );
	}

	b3SolverSet* sleepSet = b3Array_Get( world->solverSets, sleepSetId );

	// Deliberately *not* `*sleepSet = (b3SolverSet){ 0 }`. A retired slot keeps
	// its array capacity (see b3DestroySolverSet), and zeroing the struct would
	// throw away the pointers and leak them. The arrays are already empty --
	// retiring cleared their counts -- so there is nothing else to reset.
	B3_ASSERT( sleepSet->bodySims.count == 0 && sleepSet->bodyStates.count == 0 );
	B3_ASSERT( sleepSet->contactIndices.count == 0 && sleepSet->jointSims.count == 0 );
	B3_ASSERT( sleepSet->islandSims.count == 0 );

	// Grab the awake set *after* creating the sleep set: the solver set array
	// may have been resized by the push above.
	b3SolverSet* awakeSet = b3Array_Get( world->solverSets, b3_awakeSet );
	b3SolverSet* disabledSet = b3Array_Get( world->solverSets, b3_disabledSet );
	B3_ASSERT( 0 <= island->localIndex && island->localIndex < awakeSet->islandSims.count );

	sleepSet->setIndex = sleepSetId;
	b3Array_Reserve( sleepSet->bodySims, island->bodies.count );
	b3Array_Reserve( sleepSet->contactIndices, island->contacts.count );
	b3Array_Reserve( sleepSet->jointSims, island->joints.count );

	// Move awake bodies to the sleeping set. This shuffles the awake set.
	{
		for ( int i = 0; i < island->bodies.count; ++i )
		{
			int bodyId = island->bodies.data[i];
			b3Body* body = b3Array_Get( world->bodies, bodyId );
			B3_ASSERT( body->setIndex == b3_awakeSet );
			B3_ASSERT( body->islandId == islandId );
			B3_ASSERT( body->islandIndex == i );

			// Mark the move event so the application learns the body slept. A
			// body can be forced asleep before it has ever moved.
			if ( body->bodyMoveIndex != B3_NULL_INDEX )
			{
				b3BodyMoveEvent* moveEvent = b3Array_Get( world->bodyMoveEvents, body->bodyMoveIndex );
				B3_ASSERT( moveEvent->bodyId.index1 - 1 == bodyId );
				B3_ASSERT( moveEvent->bodyId.generation == body->generation );
				moveEvent->fellAsleep = true;
				body->bodyMoveIndex = B3_NULL_INDEX;
			}

			int awakeBodyIndex = body->localIndex;
			b3BodySim* awakeSim = b3Array_Get( awakeSet->bodySims, awakeBodyIndex );

			int sleepBodyIndex = sleepSet->bodySims.count;
			b3BodySim* sleepBodySim = b3Array_Emplace( sleepSet->bodySims );
			memcpy( sleepBodySim, awakeSim, sizeof( b3BodySim ) );

			int movedIndex = b3Array_RemoveSwap( awakeSet->bodySims, awakeBodyIndex );
			if ( movedIndex != B3_NULL_INDEX )
			{
				b3BodySim* movedSim = awakeSet->bodySims.data + awakeBodyIndex;
				int movedId = movedSim->bodyId;
				b3Body* movedBody = b3Array_Get( world->bodies, movedId );
				B3_ASSERT( movedBody->localIndex == movedIndex );
				movedBody->localIndex = awakeBodyIndex;

				// The moved body's contacts cache its sim index, and the swap
				// above just changed it.
				//
				// Those caches are rebuilt by the collide pass before the
				// solver reads them, so leaving them stale is invisible in a
				// release build -- but b3ValidateContacts states the invariant
				// unconditionally, and a stale index that *is* read is a body
				// solved as if it were a different body. Repairing it here
				// costs one walk of the moved body's contact list, and only
				// when an island sleeps.
				//
				// Only reachable with more than one awake island: with a single
				// island every awake body sleeps together and there is nothing
				// left holding a stale index. That is why the whole host suite
				// missed it until a scene with walls produced two.
				int movedContactKey = movedBody->headContactKey;
				while ( movedContactKey != B3_NULL_INDEX )
				{
					int movedContactId = movedContactKey >> 1;
					int movedEdge = movedContactKey & 1;
					b3Contact* movedContact = b3Array_Get( world->contacts, movedContactId );

					if ( movedEdge == 0 )
					{
						if ( movedContact->bodySimIndexA != B3_NULL_INDEX )
						{
							movedContact->bodySimIndexA = awakeBodyIndex;
						}
					}
					else
					{
						if ( movedContact->bodySimIndexB != B3_NULL_INDEX )
						{
							movedContact->bodySimIndexB = awakeBodyIndex;
						}
					}

					movedContactKey = movedContact->edges[movedEdge].nextKey;
				}
			}

			// The velocity state is simply dropped; a sleeping body has none.
			b3Array_RemoveSwap( awakeSet->bodyStates, awakeBodyIndex );

			body->setIndex = sleepSetId;
			body->localIndex = sleepBodyIndex;

			// Move non-touching contacts to the disabled set. They can exist
			// between two sleeping islands, where neither owns them.
			int contactKey = body->headContactKey;
			while ( contactKey != B3_NULL_INDEX )
			{
				int contactId = contactKey >> 1;
				int edgeIndex = contactKey & 1;

				b3Contact* contact = b3Array_Get( world->contacts, contactId );

				B3_ASSERT( contact->setIndex == b3_awakeSet || contact->setIndex == b3_disabledSet );
				contactKey = contact->edges[edgeIndex].nextKey;

				if ( contact->setIndex == b3_disabledSet )
				{
					// Already moved by another body in this island.
					continue;
				}

				if ( contact->colorIndex != B3_NULL_INDEX )
				{
					// Touching, so it moves with the island below.
					B3_ASSERT( ( contact->flags & b3_contactTouchingFlag ) != 0 );
					continue;
				}

				// The other body may still be awake. If it later sleeps it
				// becomes responsible for moving this contact.
				int otherEdgeIndex = edgeIndex ^ 1;
				int otherBodyId = contact->edges[otherEdgeIndex].bodyId;
				b3Body* otherBody = b3Array_Get( world->bodies, otherBodyId );
				if ( otherBody->setIndex == b3_awakeSet )
				{
					continue;
				}

				int localIndex = contact->localIndex;
				B3_ASSERT( awakeSet->contactIndices.data[localIndex] == contactId );
				B3_ASSERT( contact->manifoldCount == 0 );
				B3_ASSERT( ( contact->flags & b3_contactTouchingFlag ) == 0 );

				contact->setIndex = b3_disabledSet;

				// Mandatory for b3ValidateSolverSets to work.
				contact->localIndex = disabledSet->contactIndices.count;
				b3Array_Push( disabledSet->contactIndices, contact->contactId );

				int movedLocalIndex = b3Array_RemoveSwap( awakeSet->contactIndices, localIndex );
				if ( movedLocalIndex != B3_NULL_INDEX )
				{
					int movedContactIndex = awakeSet->contactIndices.data[localIndex];
					b3Contact* movedContact = b3Array_Get( world->contacts, movedContactIndex );
					B3_ASSERT( movedContact->localIndex == movedLocalIndex );
					movedContact->localIndex = localIndex;
				}
			}
		}
	}

	// Move touching contacts out of the constraint graph into the sleeping set.
	{
		for ( int i = 0; i < island->contacts.count; ++i )
		{
			int contactId = island->contacts.data[i].contactId;
			b3Contact* contact = b3Array_Get( world->contacts, contactId );
			B3_ASSERT( contact->setIndex == b3_awakeSet );
			B3_ASSERT( contact->islandId == islandId );
			B3_ASSERT( contact->islandIndex == i );

			// One colour, so this is always the overflow colour and the
			// per-colour body bitset upstream clears here does not exist.
			B3_ASSERT( contact->colorIndex == B3_OVERFLOW_INDEX );
			b3GraphColor* color = world->constraintGraph.colors + B3_OVERFLOW_INDEX;

			int sleepContactIndex = sleepSet->contactIndices.count;
			b3Array_Push( sleepSet->contactIndices, contactId );

			int localIndex = contact->localIndex;
			int movedLocalIndex = b3Array_RemoveSwap( color->contacts, localIndex );
			if ( movedLocalIndex != B3_NULL_INDEX )
			{
				int movedContactId = color->contacts.data[localIndex].contactId;
				b3Contact* movedContact = b3Array_Get( world->contacts, movedContactId );
				B3_ASSERT( movedContact->localIndex == movedLocalIndex );
				movedContact->localIndex = localIndex;
			}

			contact->setIndex = sleepSetId;
			contact->colorIndex = B3_NULL_INDEX;
			contact->localIndex = sleepContactIndex;
		}
	}

	// Move joints. Empty until Phase 6.
	{
		for ( int i = 0; i < island->joints.count; ++i )
		{
			int jointId = island->joints.data[i].jointId;
			b3Joint* joint = b3Array_Get( world->joints, jointId );
			B3_ASSERT( joint->setIndex == b3_awakeSet );
			B3_ASSERT( joint->islandId == islandId );
			B3_ASSERT( joint->islandIndex == i );
			int colorIndex = joint->colorIndex;
			int localIndex = joint->localIndex;

			B3_ASSERT( 0 <= colorIndex && colorIndex < B3_GRAPH_COLOR_COUNT );

			b3GraphColor* color = world->constraintGraph.colors + colorIndex;

			b3JointSim* awakeJointSim = b3Array_Get( color->jointSims, localIndex );

			int sleepJointIndex = sleepSet->jointSims.count;
			b3JointSim* sleepJointSim = b3Array_Emplace( sleepSet->jointSims );
			memcpy( sleepJointSim, awakeJointSim, sizeof( b3JointSim ) );

			int movedIndex = b3Array_RemoveSwap( color->jointSims, localIndex );
			if ( movedIndex != B3_NULL_INDEX )
			{
				b3JointSim* movedJointSim = color->jointSims.data + localIndex;
				int movedId = movedJointSim->jointId;
				b3Joint* movedJoint = b3Array_Get( world->joints, movedId );
				B3_ASSERT( movedJoint->localIndex == movedIndex );
				movedJoint->localIndex = localIndex;
			}

			joint->setIndex = sleepSetId;
			joint->colorIndex = B3_NULL_INDEX;
			joint->localIndex = sleepJointIndex;
		}
	}

	// Move the island itself.
	{
		B3_ASSERT( island->setIndex == b3_awakeSet );

		int islandIndex = island->localIndex;
		b3IslandSim* sleepIsland = b3Array_Emplace( sleepSet->islandSims );
		sleepIsland->islandId = islandId;

		int movedIslandIndex = b3Array_RemoveSwap( awakeSet->islandSims, islandIndex );
		if ( movedIslandIndex != B3_NULL_INDEX )
		{
			b3IslandSim* movedIslandSim = awakeSet->islandSims.data + islandIndex;
			int movedIslandId = movedIslandSim->islandId;
			b3Island* movedIsland = b3Array_Get( world->islands, movedIslandId );
			B3_ASSERT( movedIsland->localIndex == movedIslandIndex );
			movedIsland->localIndex = islandIndex;
		}

		island->setIndex = sleepSetId;
		island->localIndex = 0;
	}

	if ( world->splitIslandId == islandId )
	{
		world->splitIslandId = B3_NULL_INDEX;
	}

	b3ValidateSolverSets( world );
	return true;
}

// Called when joints are created between sets. Two sleeping sets can stay
// asleep; otherwise one is woken. Islands merge when the set wakes.
void b3MergeSolverSets( b3World* world, int setId1, int setId2 )
{
	B3_ASSERT( setId1 >= b3_firstSleepingSet );
	B3_ASSERT( setId2 >= b3_firstSleepingSet );
	b3SolverSet* set1 = b3Array_Get( world->solverSets, setId1 );
	b3SolverSet* set2 = b3Array_Get( world->solverSets, setId2 );

	// Move the fewest bodies.
	if ( set1->bodySims.count < set2->bodySims.count )
	{
		b3SolverSet* tempSet = set1;
		set1 = set2;
		set2 = tempSet;

		int tempId = setId1;
		setId1 = setId2;
		setId2 = tempId;
	}

	{
		b3Body* bodies = world->bodies.data;
		int bodyCount = set2->bodySims.count;
		for ( int i = 0; i < bodyCount; ++i )
		{
			b3BodySim* simSrc = set2->bodySims.data + i;

			b3Body* body = bodies + simSrc->bodyId;
			B3_ASSERT( body->setIndex == setId2 );
			body->setIndex = setId1;
			body->localIndex = set1->bodySims.count;

			b3BodySim* simDst = b3Array_Emplace( set1->bodySims );
			memcpy( simDst, simSrc, sizeof( b3BodySim ) );
		}
	}

	{
		int contactCount = set2->contactIndices.count;
		for ( int i = 0; i < contactCount; ++i )
		{
			int contactIndex = set2->contactIndices.data[i];
			b3Contact* contact = b3Array_Get( world->contacts, contactIndex );
			B3_ASSERT( contact->setIndex == setId2 );
			contact->setIndex = setId1;
			contact->localIndex = set1->contactIndices.count;
			b3Array_Push( set1->contactIndices, contactIndex );
		}
	}

	{
		int jointCount = set2->jointSims.count;
		for ( int i = 0; i < jointCount; ++i )
		{
			b3JointSim* jointSrc = set2->jointSims.data + i;

			b3Joint* joint = b3Array_Get( world->joints, jointSrc->jointId );
			B3_ASSERT( joint->setIndex == setId2 );
			joint->setIndex = setId1;
			joint->localIndex = set1->jointSims.count;

			b3JointSim* jointDst = b3Array_Emplace( set1->jointSims );
			memcpy( jointDst, jointSrc, sizeof( b3JointSim ) );
		}
	}

	{
		int islandCount = set2->islandSims.count;
		for ( int i = 0; i < islandCount; ++i )
		{
			b3IslandSim* islandSrc = set2->islandSims.data + i;
			int islandId = islandSrc->islandId;

			b3Island* island = b3Array_Get( world->islands, islandId );
			island->setIndex = setId1;
			island->localIndex = set1->islandSims.count;

			b3IslandSim* islandDst = b3Array_Emplace( set1->islandSims );
			memcpy( islandDst, islandSrc, sizeof( b3IslandSim ) );
		}
	}

	b3DestroySolverSet( world, setId2 );

	b3ValidateSolverSets( world );
}

void b3TransferBody( b3World* world, b3SolverSet* targetSet, b3SolverSet* sourceSet, b3Body* body )
{
	if ( targetSet == sourceSet )
	{
		return;
	}

	int sourceIndex = body->localIndex;
	b3BodySim* sourceSim = b3Array_Get( sourceSet->bodySims, sourceIndex );

	int targetIndex = targetSet->bodySims.count;
	b3BodySim* targetSim = b3Array_Emplace( targetSet->bodySims );
	memcpy( targetSim, sourceSim, sizeof( b3BodySim ) );

	// Clear the transient flags.
	targetSim->flags &= ~( b3_isFast | b3_isSpeedCapped | b3_hadTimeOfImpact );

	int movedIndex = b3Array_RemoveSwap( sourceSet->bodySims, sourceIndex );
	if ( movedIndex != B3_NULL_INDEX )
	{
		b3BodySim* movedSim = sourceSet->bodySims.data + sourceIndex;
		int movedId = movedSim->bodyId;
		b3Body* movedBody = b3Array_Get( world->bodies, movedId );
		B3_ASSERT( movedBody->localIndex == movedIndex );
		movedBody->localIndex = sourceIndex;
	}

	if ( sourceSet->setIndex == b3_awakeSet )
	{
		b3Array_RemoveSwap( sourceSet->bodyStates, sourceIndex );
	}
	else if ( targetSet->setIndex == b3_awakeSet )
	{
		b3BodyState* state = b3Array_Emplace( targetSet->bodyStates );
		*state = b3_identityBodyState;
		state->flags = body->flags;
	}

	body->setIndex = targetSet->setIndex;
	body->localIndex = targetIndex;
}

void b3TransferJoint( b3World* world, b3SolverSet* targetSet, b3SolverSet* sourceSet, b3Joint* joint )
{
	if ( targetSet == sourceSet )
	{
		return;
	}

	int localIndex = joint->localIndex;
	int colorIndex = joint->colorIndex;

	// Find the source.
	b3JointSim* sourceSim;
	if ( sourceSet->setIndex == b3_awakeSet )
	{
		B3_ASSERT( 0 <= colorIndex && colorIndex < B3_GRAPH_COLOR_COUNT );
		b3GraphColor* color = world->constraintGraph.colors + colorIndex;

		sourceSim = b3Array_Get( color->jointSims, localIndex );
	}
	else
	{
		B3_ASSERT( colorIndex == B3_NULL_INDEX );
		sourceSim = b3Array_Get( sourceSet->jointSims, localIndex );
	}

	// Create the target and copy.
	if ( targetSet->setIndex == b3_awakeSet )
	{
		b3AddJointToGraph( world, sourceSim, joint );
		joint->setIndex = b3_awakeSet;
	}
	else
	{
		joint->setIndex = targetSet->setIndex;
		joint->localIndex = targetSet->jointSims.count;
		joint->colorIndex = B3_NULL_INDEX;

		b3JointSim* targetSim = b3Array_Emplace( targetSet->jointSims );
		memcpy( targetSim, sourceSim, sizeof( b3JointSim ) );
	}

	// Destroy the source.
	if ( sourceSet->setIndex == b3_awakeSet )
	{
		b3RemoveJointFromGraph( world, joint->edges[0].bodyId, joint->edges[1].bodyId, colorIndex, localIndex );
	}
	else
	{
		int movedIndex = b3Array_RemoveSwap( sourceSet->jointSims, localIndex );
		if ( movedIndex != B3_NULL_INDEX )
		{
			b3JointSim* movedJointSim = sourceSet->jointSims.data + localIndex;
			int movedId = movedJointSim->jointId;
			b3Joint* movedJoint = b3Array_Get( world->joints, movedId );
			movedJoint->localIndex = localIndex;
		}
	}
}
