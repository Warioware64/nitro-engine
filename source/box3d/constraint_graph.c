// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Erin Catto        (original Box3D)
// Copyright (c) 2026 Warioware64       (Nitro Engine Advanced fixed-point port)
//
// This file is part of Nitro Engine Advanced

/// @file   constraint_graph.c
/// @brief  Adding and removing constraints from the single graph colour.
///
/// See constraint_graph.h for why there is only one. What is left after that
/// collapse is short enough to read in one go: upstream's three colour-search
/// loops, `b3AssignJointColor` and every `b3GetBit`/`b3SetBitGrow`/`b3ClearBit`
/// call disappear, because the answer is always B3_OVERFLOW_INDEX.
///
/// Also gone: `b3GetGraphColor` and the 24-entry `b3_graphColors` table, which
/// exist to tint the debug renderer.

#include "constraint_graph.h"

#include "body.h"
#include "contact.h"
#include "core.h"
#include "joint.h"
#include "physics_world.h"

#include <string.h>

void b3CreateGraph( b3ConstraintGraph* graph, int contactCapacity, int jointCapacity )
{
	*graph = ( b3ConstraintGraph ){ 0 };

	// Upstream sizes one body bitset per colour from its argument and lets the
	// contact arrays grow from zero. There are no bitsets here -- one colour --
	// so the parameters are repurposed for the things that actually allocate.
	//
	// Every touching contact is pushed into the overflow colour by
	// b3AddContactToGraph, which runs inside b3Collide. Growing there is a heap
	// allocation in the middle of a step, and NEA_Phys3D's pool allocator seals
	// once the world is running.
	b3Array_Reserve( graph->colors[B3_OVERFLOW_INDEX].contacts, contactCapacity );

	// Joints reach the colour through b3CreateJointInGraph, which a caller
	// invokes outside a step -- but also through b3TransferJoint, which runs
	// inside one whenever an island wakes.
	b3Array_Reserve( graph->colors[B3_OVERFLOW_INDEX].jointSims, jointCapacity );
}

void b3DestroyGraph( b3ConstraintGraph* graph )
{
	for ( int i = 0; i < B3_GRAPH_COLOR_COUNT; ++i )
	{
		b3GraphColor* color = graph->colors + i;

		b3Array_Destroy( color->contacts );
		b3Array_Destroy( color->jointSims );
	}
}

// Contacts are always created non-touching. They are cloned into the
// constraint graph once the narrow phase finds them touching.
void b3AddContactToGraph( b3World* world, b3Contact* contact )
{
	B3_ASSERT( contact->manifoldCount > 0 );
	B3_ASSERT( contact->flags & b3_contactTouchingFlag );

	b3ConstraintGraph* graph = &world->constraintGraph;

	// One colour, so no search: upstream's three loops over the colour body
	// sets all end at B3_OVERFLOW_INDEX here.
	const int colorIndex = B3_OVERFLOW_INDEX;

	int bodyIdA = contact->edges[0].bodyId;
	int bodyIdB = contact->edges[1].bodyId;
	b3Body* bodyA = b3Array_Get( world->bodies, bodyIdA );
	b3Body* bodyB = b3Array_Get( world->bodies, bodyIdB );

	B3_ASSERT( bodyA->type == b3_dynamicBody || bodyB->type == b3_dynamicBody );

	b3GraphColor* color = graph->colors + colorIndex;
	contact->colorIndex = colorIndex;
	contact->localIndex = color->contacts.count;

	// A static body has no sim in the awake set, so the solver reads it as
	// "infinite mass, zero velocity" from a null index rather than from data.
	contact->bodySimIndexA = bodyA->type == b3_staticBody ? B3_NULL_INDEX : bodyA->localIndex;
	contact->bodySimIndexB = bodyB->type == b3_staticBody ? B3_NULL_INDEX : bodyB->localIndex;

	B3_ASSERT( contact->manifoldCount < UINT16_MAX );
	b3ContactSpec spec = {
		.contactId = contact->contactId,
		.manifoldStart = 0,
		.manifoldCount = (uint16_t)contact->manifoldCount,
	};
	b3Array_Push( color->contacts, spec );
}

void b3RemoveContactFromGraph( b3World* world, int bodyIdA, int bodyIdB, int colorIndex, int localIndex )
{
	// The body ids are upstream's, for clearing the colour's body set.
	B3_UNUSED( bodyIdA, bodyIdB );

	b3ConstraintGraph* graph = &world->constraintGraph;

	B3_ASSERT( colorIndex == B3_OVERFLOW_INDEX );
	b3GraphColor* color = graph->colors + colorIndex;

	int movedIndex = b3Array_RemoveSwap( color->contacts, localIndex );
	if ( movedIndex != B3_NULL_INDEX )
	{
		int movedContactId = color->contacts.data[localIndex].contactId;
		b3Contact* movedContact = b3Array_Get( world->contacts, movedContactId );
		B3_ASSERT( movedContact->setIndex == b3_awakeSet );
		B3_ASSERT( movedContact->colorIndex == colorIndex );
		B3_ASSERT( movedContact->localIndex == movedIndex );
		movedContact->localIndex = localIndex;
	}
}

b3JointSim* b3CreateJointInGraph( b3World* world, b3Joint* joint )
{
	b3ConstraintGraph* graph = &world->constraintGraph;

#if defined( NEA_DEBUG ) || defined( B3_ENABLE_ASSERT )
	b3Body* bodyA = b3Array_Get( world->bodies, joint->edges[0].bodyId );
	b3Body* bodyB = b3Array_Get( world->bodies, joint->edges[1].bodyId );
	B3_ASSERT( bodyA->type == b3_dynamicBody || bodyB->type == b3_dynamicBody );
#endif

	// b3AssignJointColor collapses to this at one colour.
	const int colorIndex = B3_OVERFLOW_INDEX;

	b3JointSim* jointSim = b3Array_Emplace( graph->colors[colorIndex].jointSims );
	memset( jointSim, 0, sizeof( b3JointSim ) );

	joint->colorIndex = colorIndex;
	joint->localIndex = graph->colors[colorIndex].jointSims.count - 1;
	return jointSim;
}

void b3AddJointToGraph( b3World* world, b3JointSim* jointSim, b3Joint* joint )
{
	b3JointSim* jointDst = b3CreateJointInGraph( world, joint );
	memcpy( jointDst, jointSim, sizeof( b3JointSim ) );
}

void b3RemoveJointFromGraph( b3World* world, int bodyIdA, int bodyIdB, int colorIndex, int localIndex )
{
	B3_UNUSED( bodyIdA, bodyIdB );

	b3ConstraintGraph* graph = &world->constraintGraph;

	B3_ASSERT( colorIndex == B3_OVERFLOW_INDEX );
	b3GraphColor* color = graph->colors + colorIndex;

	int movedIndex = b3Array_RemoveSwap( color->jointSims, localIndex );
	if ( movedIndex != B3_NULL_INDEX )
	{
		b3JointSim* movedJointSim = color->jointSims.data + localIndex;
		int movedId = movedJointSim->jointId;
		b3Joint* movedJoint = b3Array_Get( world->joints, movedId );
		B3_ASSERT( movedJoint->setIndex == b3_awakeSet );
		B3_ASSERT( movedJoint->colorIndex == colorIndex );
		B3_ASSERT( movedJoint->localIndex == movedIndex );
		movedJoint->localIndex = localIndex;
	}
}
