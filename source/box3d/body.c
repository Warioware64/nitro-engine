// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Erin Catto        (original Box3D)
// Copyright (c) 2026 Warioware64       (Nitro Engine Advanced fixed-point port)
//
// This file is part of Nitro Engine Advanced

/// @file   body.c
/// @brief  Body lifetime, mass properties, and the accessors a game needs.
///
/// See body.h for the field scales and for what is not ported.

#include "body.h"

#include "constraint_graph.h"
#include "contact.h"
#include "core.h"
#include "id_pool.h"
#include "island.h"
#include "joint.h"
#include "physics_world.h"
#include "shape.h"
#include "solver_set.h"

#include <stddef.h>

b3Body* b3GetBodyFullId( b3World* world, b3BodyId bodyId )
{
	B3_ASSERT( b3Body_IsValid( bodyId ) );

	// The id index starts at one so that zero can represent null.
	return b3Array_Get( world->bodies, bodyId.index1 - 1 );
}

b3WorldTransform b3GetBodyTransformQuick( b3World* world, b3Body* body )
{
	b3SolverSet* set = b3Array_Get( world->solverSets, body->setIndex );
	b3BodySim* bodySim = b3Array_Get( set->bodySims, body->localIndex );
	return bodySim->transform;
}

b3WorldTransform b3GetBodyTransform( b3World* world, int bodyId )
{
	b3Body* body = b3Array_Get( world->bodies, bodyId );
	return b3GetBodyTransformQuick( world, body );
}

b3BodyId b3MakeBodyId( b3World* world, int bodyId )
{
	b3Body* body = b3Array_Get( world->bodies, bodyId );
	return ( b3BodyId ){ bodyId + 1, world->worldId, body->generation };
}

b3BodySim* b3GetBodySim( b3World* world, b3Body* body )
{
	b3SolverSet* set = b3Array_Get( world->solverSets, body->setIndex );
	b3BodySim* bodySim = b3Array_Get( set->bodySims, body->localIndex );
	return bodySim;
}

b3BodyState* b3GetBodyState( b3World* world, b3Body* body )
{
	if ( body->setIndex == b3_awakeSet )
	{
		b3SolverSet* set = b3Array_Get( world->solverSets, b3_awakeSet );
		return b3Array_Get( set->bodyStates, body->localIndex );
	}

	return NULL;
}

void b3SyncBodyFlags( b3World* world, b3Body* body )
{
	// Transient flags are per step and may legitimately differ between the
	// three copies, so they are never propagated.
	uint32_t flags = body->flags & ~b3_bodyTransientFlags;

	b3BodySim* bodySim = b3GetBodySim( world, body );
	bodySim->flags = flags;

	b3BodyState* bodyState = b3GetBodyState( world, body );
	if ( bodyState != NULL )
	{
		bodyState->flags = flags;
	}
}

static void b3CreateIslandForBody( b3World* world, int setIndex, b3Body* body )
{
	B3_ASSERT( body->islandId == B3_NULL_INDEX );
	B3_ASSERT( setIndex != b3_disabledSet );

	b3Island* island = b3CreateIsland( world, setIndex );
	b3Array_Push( island->bodies, body->id );
	body->islandId = island->islandId;
	body->islandIndex = 0;

	b3ValidateIsland( world, island->islandId );
}

static void b3RemoveBodyFromIsland( b3World* world, b3Body* body )
{
	if ( body->islandId == B3_NULL_INDEX )
	{
		B3_ASSERT( body->islandIndex == B3_NULL_INDEX );
		return;
	}

	int islandId = body->islandId;
	b3Island* island = b3Array_Get( world->islands, islandId );
	{
		int localIndex = body->islandIndex;
		int movedBodyId = island->bodies.data[island->bodies.count - 1];
		island->bodies.data[localIndex] = movedBodyId;
		world->bodies.data[movedBodyId].islandIndex = localIndex;
		island->bodies.count -= 1;
	}

	if ( island->bodies.count == 0 )
	{
		B3_ASSERT( island->contacts.count == 0 );
		B3_ASSERT( island->joints.count == 0 );

		b3DestroyIsland( world, island->islandId );
	}
	else
	{
		b3ValidateIsland( world, islandId );
	}

	body->islandId = B3_NULL_INDEX;
	body->islandIndex = B3_NULL_INDEX;
}

static void b3DestroyBodyContacts( b3World* world, b3Body* body, bool wakeBodies )
{
	int edgeKey = body->headContactKey;
	while ( edgeKey != B3_NULL_INDEX )
	{
		int contactId = edgeKey >> 1;
		int edgeIndex = edgeKey & 1;

		b3Contact* contact = b3Array_Get( world->contacts, contactId );
		edgeKey = contact->edges[edgeIndex].nextKey;
		b3DestroyContact( world, contact, wakeBodies );
	}

	b3ValidateSolverSets( world );
}

b3BodyId b3CreateBody( b3WorldId worldId, const b3BodyDef* def )
{
	B3_CHECK_DEF( def );
	B3_ASSERT( b3IsValidQuat( def->rotation ) );
	B3_ASSERT( b3Raw( def->linearDamping ) >= 0 );
	B3_ASSERT( b3Raw( def->angularDamping ) >= 0 );
	B3_ASSERT( b3Raw( def->sleepThreshold ) >= 0 );

	b3World* world = b3GetUnlockedWorldFromId( worldId );

	if ( world == NULL )
	{
		return b3_nullBodyId;
	}

	world->locked = true;

	bool isAwake = ( def->isAwake || def->enableSleep == false ) && def->isEnabled;

	// Determine the solver set.
	int setId;
	if ( def->isEnabled == false )
	{
		// Any body type can be disabled.
		setId = b3_disabledSet;
	}
	else if ( def->type == b3_staticBody )
	{
		setId = b3_staticSet;
	}
	else if ( isAwake == true )
	{
		setId = b3_awakeSet;
	}
	else
	{
		// A new set for a sleeping body in its own island.
		setId = b3AllocId( &world->solverSetIdPool );
		if ( setId == world->solverSets.count )
		{
			b3Array_Push( world->solverSets, ( b3SolverSet ){ 0 } );
		}
		else
		{
			B3_ASSERT( world->solverSets.data[setId].setIndex == B3_NULL_INDEX );
		}

		world->solverSets.data[setId].setIndex = setId;
	}

	B3_ASSERT( 0 <= setId && setId < world->solverSets.count );

	int bodyId = b3AllocId( &world->bodyIdPool );

	uint32_t lockFlags = 0;
	lockFlags |= def->motionLocks.linearX ? b3_lockLinearX : 0;
	lockFlags |= def->motionLocks.linearY ? b3_lockLinearY : 0;
	lockFlags |= def->motionLocks.linearZ ? b3_lockLinearZ : 0;
	lockFlags |= def->motionLocks.angularX ? b3_lockAngularX : 0;
	lockFlags |= def->motionLocks.angularY ? b3_lockAngularY : 0;
	lockFlags |= def->motionLocks.angularZ ? b3_lockAngularZ : 0;

	b3SolverSet* set = b3Array_Get( world->solverSets, setId );
	b3BodySim* bodySim = b3Array_Emplace( set->bodySims );
	*bodySim = ( b3BodySim ){ 0 };
	bodySim->transform.p = def->position;
	bodySim->transform.q = def->rotation;
	bodySim->center = def->position;
	bodySim->rotation0 = bodySim->transform.q;
	bodySim->center0 = bodySim->center;
	bodySim->localCenter = b3Vec3_zero;
	bodySim->force = b3Vec3_zero;
	bodySim->torque = b3Vec3_zero;
	bodySim->invMass = b3iw_zero;
	bodySim->invInertiaLocal = b3MatW_zero;
	bodySim->invInertiaWorld = b3MatW_zero;
	bodySim->minExtent = B3_HUGE;
	bodySim->maxExtent = b3Vec3_zero;
	bodySim->linearDamping = def->linearDamping;
	bodySim->angularDamping = def->angularDamping;
	bodySim->gravityScale = def->gravityScale;
	bodySim->bodyId = bodyId;
	bodySim->flags = lockFlags;
	bodySim->flags |= def->isBullet ? b3_isBullet : 0;
	bodySim->flags |= def->allowFastRotation ? b3_allowFastRotation : 0;
	bodySim->flags |= def->type == b3_dynamicBody ? b3_dynamicFlag : 0;
	bodySim->flags |= def->enableSleep ? b3_enableSleep : 0;
	bodySim->flags |= def->enableContactRecycling ? b3_bodyEnableContactRecycling : 0;

	if ( setId == b3_awakeSet )
	{
		b3BodyState* bodyState = b3Array_Emplace( set->bodyStates );

		*bodyState = b3_identityBodyState;
		bodyState->linearVelocity = def->linearVelocity;
		bodyState->angularVelocity = def->angularVelocity;
		bodyState->flags = bodySim->flags;
	}

	if ( bodyId == world->bodies.count )
	{
		b3Array_Push( world->bodies, ( b3Body ){ 0 } );
	}
	else
	{
		B3_ASSERT( world->bodies.data[bodyId].id == B3_NULL_INDEX );
	}

	b3Body* body = b3Array_Get( world->bodies, bodyId );
	body->userData = def->userData;
	body->setIndex = setId;
	body->localIndex = set->bodySims.count - 1;
	body->generation += 1;
	body->headShapeId = B3_NULL_INDEX;
	body->shapeCount = 0;
	body->headContactKey = B3_NULL_INDEX;
	body->contactCount = 0;
	body->headJointKey = B3_NULL_INDEX;
	body->jointCount = 0;
	body->islandId = B3_NULL_INDEX;
	body->islandIndex = B3_NULL_INDEX;
	body->bodyMoveIndex = B3_NULL_INDEX;
	body->id = bodyId;
	body->sleepThreshold = def->sleepThreshold;
	body->sleepTime = b3t_zero;
	body->sleepVelocity = b3f_zero;
	body->mass = b3f_zero;
	body->inertia = b3Mat3_zero;
	body->type = def->type;
	body->flags = bodySim->flags;

	// Enabled dynamic and kinematic bodies need an island.
	if ( setId >= b3_awakeSet )
	{
		b3CreateIslandForBody( world, setId, body );
	}

	b3ValidateSolverSets( world );

	b3BodyId id = { bodyId + 1, world->worldId, body->generation };

	world->locked = false;

	return id;
}

bool b3IsBodyAwake( b3World* world, b3Body* body )
{
	B3_UNUSED( world );
	return body->setIndex == b3_awakeSet;
}

bool b3WakeBody( b3World* world, b3Body* body )
{
	if ( body->setIndex >= b3_firstSleepingSet )
	{
		b3WakeSolverSet( world, body->setIndex );
		b3ValidateSolverSets( world );
		return true;
	}

	return false;
}

bool b3WakeBodyWithLock( b3World* world, b3Body* body )
{
	B3_ASSERT( world->locked == false );
	world->locked = true;
	bool woke = b3WakeBody( world, body );
	world->locked = false;
	return woke;
}

void b3DestroyBody( b3BodyId bodyId )
{
	b3World* world = b3GetUnlockedWorld( bodyId.world0 );
	if ( world == NULL )
	{
		return;
	}

	world->locked = true;

	b3Body* body = b3GetBodyFullId( world, bodyId );

	// Wake bodies attached to this one, even if this body is static.
	bool wakeBodies = true;

	// Destroy the attached joints. Always empty until Phase 6.
	int edgeKey = body->headJointKey;
	while ( edgeKey != B3_NULL_INDEX )
	{
		int jointId = edgeKey >> 1;
		int edgeIndex = edgeKey & 1;

		b3Joint* joint = b3Array_Get( world->joints, jointId );
		edgeKey = joint->edges[edgeIndex].nextKey;

		// Careful: this modifies the list being traversed.
		b3DestroyJointInternal( world, joint, wakeBodies );
	}

	b3DestroyBodyContacts( world, body, wakeBodies );

	// Destroy the attached shapes and their broad-phase proxies.
	int shapeId = body->headShapeId;
	while ( shapeId != B3_NULL_INDEX )
	{
		b3Shape* shape = b3Array_Get( world->shapes, shapeId );

		b3DestroyShapeProxy( shape, &world->broadPhase );
		b3DestroyShapeAllocations( world, shape );

		b3FreeId( &world->shapeIdPool, shapeId );
		shape->id = B3_NULL_INDEX;

		shapeId = shape->nextShapeId;
	}

	b3RemoveBodyFromIsland( world, body );

	// Remove the body sim from the solver set that owns it.
	b3SolverSet* set = b3Array_Get( world->solverSets, body->setIndex );
	int movedIndex = b3Array_RemoveSwap( set->bodySims, body->localIndex );
	if ( movedIndex != B3_NULL_INDEX )
	{
		b3BodySim* movedSim = set->bodySims.data + body->localIndex;
		int movedId = movedSim->bodyId;
		b3Body* movedBody = b3Array_Get( world->bodies, movedId );
		B3_ASSERT( movedBody->localIndex == movedIndex );
		movedBody->localIndex = body->localIndex;
	}

	// Remove the body state from the awake set.
	if ( body->setIndex == b3_awakeSet )
	{
		int result = b3Array_RemoveSwap( set->bodyStates, body->localIndex );
		B3_UNUSED( result );
		B3_ASSERT( result == movedIndex );
	}
	else if ( set->setIndex >= b3_firstSleepingSet && set->bodySims.count == 0 )
	{
		// The sleeping set is now an orphan.
		b3DestroySolverSet( world, set->setIndex );
	}

	// Free the body and its id, preserving the generation.
	b3FreeId( &world->bodyIdPool, body->id );

	body->setIndex = B3_NULL_INDEX;
	body->localIndex = B3_NULL_INDEX;
	body->id = B3_NULL_INDEX;

	b3ValidateSolverSets( world );

	world->locked = false;
}

// =========================================================================
// Mass properties
// =========================================================================

void b3UpdateBodyMassData( b3World* world, b3Body* body )
{
	b3BodySim* bodySim = b3GetBodySim( world, body );

	// Mass is no longer dirty.
	body->flags &= ~b3_dirtyMass;
	b3SyncBodyFlags( world, body );

	body->mass = b3f_zero;
	body->inertia = b3Mat3_zero;

	bodySim->invMass = b3iw_zero;
	bodySim->invInertiaLocal = b3MatW_zero;
	bodySim->invInertiaWorld = b3MatW_zero;
	bodySim->localCenter = b3Vec3_zero;
	bodySim->minExtent = B3_HUGE;
	bodySim->maxExtent = b3Vec3_zero;

	if ( body->headShapeId == B3_NULL_INDEX )
	{
		return;
	}

	// Static and kinematic bodies have zero mass.
	if ( body->type != b3_dynamicBody )
	{
		bodySim->center = bodySim->transform.p;
		bodySim->center0 = bodySim->center;

		// Kinematic bodies still need extents, or sleeping misbehaves.
		if ( body->type == b3_kinematicBody )
		{
			int shapeId = body->headShapeId;
			while ( shapeId != B3_NULL_INDEX )
			{
				const b3Shape* s = b3Array_Get( world->shapes, shapeId );

				b3ShapeExtent extent = b3ComputeShapeExtent( s, b3Vec3_zero );
				bodySim->minExtent = b3MinF( bodySim->minExtent, extent.minExtent );
				bodySim->maxExtent = b3Max( bodySim->maxExtent, extent.maxExtent );

				shapeId = s->nextShapeId;
			}
		}

		return;
	}

	int shapeCount = body->shapeCount;
	b3MassData* masses = b3StackAlloc( &world->stack, shapeCount * (int)sizeof( b3MassData ), "mass data" );

	// Accumulate mass over all shapes.
	b3Vec3 localCenter = b3Vec3_zero;
	int shapeId = body->headShapeId;
	int shapeIndex = 0;
	while ( shapeId != B3_NULL_INDEX )
	{
		const b3Shape* s = b3Array_Get( world->shapes, shapeId );
		shapeId = s->nextShapeId;

		if ( b3Raw( s->density ) == 0 )
		{
			masses[shapeIndex] = ( b3MassData ){ 0 };
			shapeIndex += 1;
			continue;
		}

		b3MassData massData = b3ComputeShapeMass( s );
		body->mass = b3AddF( body->mass, massData.mass );
		localCenter = b3MulAdd( localCenter, massData.mass, massData.center );

		masses[shapeIndex] = massData;
		shapeIndex += 1;
	}

	// Compute the centre of mass.
	if ( b3Raw( body->mass ) > 0 )
	{
		bodySim->invMass = b3RcpF( body->mass );
		localCenter = b3MulWV( bodySim->invMass, localCenter );
	}

	// Second pass: accumulate rotational inertia about the centre of mass.
	//
	// This is where the port departs from upstream. Every b3MassData::inertia
	// is per unit mass, so the per-shape tensors cannot simply be summed --
	// they have to be mass weighted. The weighted sum is the *absolute*
	// inertia, the quantity that overflows Q12, so it is accumulated wide at
	// Q24 in int64 and divided by the total mass at the end to land back on a
	// per-unit-mass tensor. That is the same rule b3ComputeCapsuleMass follows
	// when it composes a cylinder, a sphere and a Steiner term.
	int64_t wide[6] = { 0, 0, 0, 0, 0, 0 };
	for ( shapeIndex = 0; shapeIndex < shapeCount; ++shapeIndex )
	{
		b3MassData massData = masses[shapeIndex];
		if ( b3Raw( massData.mass ) == 0 )
		{
			continue;
		}

		// Shift to the centre of mass. Safe because it can only increase.
		b3Vec3 offset = b3Sub( localCenter, massData.center );
		b3Matrix3 unit = b3AddMM( massData.inertia, b3SteinerUnit( offset ) );

		int64_t m = (int64_t)b3Raw( massData.mass );
		wide[0] += m * b3Raw( unit.cx.x );
		wide[1] += m * b3Raw( unit.cy.x );
		wide[2] += m * b3Raw( unit.cz.x );
		wide[3] += m * b3Raw( unit.cy.y );
		wide[4] += m * b3Raw( unit.cz.y );
		wide[5] += m * b3Raw( unit.cz.z );
	}

	b3StackFree( &world->stack, masses );
	masses = NULL;

	if ( b3Raw( body->mass ) > 0 )
	{
		// Q24 wide divided by a Q12 mass gives a Q12 per-unit-mass entry.
		int32_t massRaw = b3Raw( body->mass );
		int32_t u[6];
		for ( int i = 0; i < 6; ++i )
		{
			u[i] = (int32_t)b3HwDiv64( wide[i], massRaw );
		}

		b3f xx = b3Makeb3fRef( u[0], B3_REF( (double)u[0] / (double)B3_F_ONE ) );
		b3f xy = b3Makeb3fRef( u[1], B3_REF( (double)u[1] / (double)B3_F_ONE ) );
		b3f xz = b3Makeb3fRef( u[2], B3_REF( (double)u[2] / (double)B3_F_ONE ) );
		b3f yy = b3Makeb3fRef( u[3], B3_REF( (double)u[3] / (double)B3_F_ONE ) );
		b3f yz = b3Makeb3fRef( u[4], B3_REF( (double)u[4] / (double)B3_F_ONE ) );
		b3f zz = b3Makeb3fRef( u[5], B3_REF( (double)u[5] / (double)B3_F_ONE ) );

		body->inertia = b3MakeMatrix3( b3MakeVec3( xx, xy, xz ), b3MakeVec3( xy, yy, yz ), b3MakeVec3( xz, yz, zz ) );

		// b3InvertInertia forms mass * unitInertia internally and normalizes
		// it, so the tensor that would have overflowed is never stored.
		bodySim->invInertiaLocal = b3InvertInertia( body->inertia, body->mass );
		bodySim->invInertiaWorld = b3RotateInertiaW( bodySim->transform.q, bodySim->invInertiaLocal );
	}

	// Move the centre of mass.
	b3Pos oldCenter = bodySim->center;
	bodySim->localCenter = localCenter;
	bodySim->center = b3TransformPoint( bodySim->transform, bodySim->localCenter );
	bodySim->center0 = bodySim->center;

	// Update the centre of mass velocity.
	b3BodyState* state = b3GetBodyState( world, body );
	if ( state != NULL )
	{
		b3Vec3 deltaLinear = b3Cross( state->angularVelocity, b3Sub( bodySim->center, oldCenter ) );
		state->linearVelocity = b3Add( state->linearVelocity, deltaLinear );
	}

	// Compute body extents relative to the centre of mass.
	shapeId = body->headShapeId;
	while ( shapeId != B3_NULL_INDEX )
	{
		b3Shape* s = b3Array_Get( world->shapes, shapeId );

		b3ShapeExtent extent = b3ComputeShapeExtent( s, localCenter );
		bodySim->minExtent = b3MinF( bodySim->minExtent, extent.minExtent );
		bodySim->maxExtent = b3Max( bodySim->maxExtent, extent.maxExtent );

		shapeId = s->nextShapeId;
	}

	// Apply fixed rotation.
	if ( ( bodySim->flags & b3_fixedRotation ) == b3_fixedRotation )
	{
		body->inertia = b3Mat3_zero;
		bodySim->invInertiaLocal = b3MatW_zero;
		bodySim->invInertiaWorld = b3MatW_zero;
	}
}

// =========================================================================
// Transform and velocity
// =========================================================================

b3Pos b3Body_GetPosition( b3BodyId bodyId )
{
	b3World* world = b3GetWorld( bodyId.world0 );
	b3Body* body = b3GetBodyFullId( world, bodyId );
	b3WorldTransform transform = b3GetBodyTransformQuick( world, body );
	return transform.p;
}

b3Quat b3Body_GetRotation( b3BodyId bodyId )
{
	b3World* world = b3GetWorld( bodyId.world0 );
	b3Body* body = b3GetBodyFullId( world, bodyId );
	b3WorldTransform transform = b3GetBodyTransformQuick( world, body );
	return transform.q;
}

b3WorldTransform b3Body_GetTransform( b3BodyId bodyId )
{
	b3World* world = b3GetWorld( bodyId.world0 );
	b3Body* body = b3GetBodyFullId( world, bodyId );
	return b3GetBodyTransformQuick( world, body );
}

b3Vec3 b3Body_GetLocalPoint( b3BodyId bodyId, b3Pos worldPoint )
{
	b3World* world = b3GetWorld( bodyId.world0 );
	b3Body* body = b3GetBodyFullId( world, bodyId );
	b3WorldTransform transform = b3GetBodyTransformQuick( world, body );
	return b3InvTransformPoint( transform, worldPoint );
}

b3Pos b3Body_GetWorldPoint( b3BodyId bodyId, b3Vec3 localPoint )
{
	b3World* world = b3GetWorld( bodyId.world0 );
	b3Body* body = b3GetBodyFullId( world, bodyId );
	b3WorldTransform transform = b3GetBodyTransformQuick( world, body );
	return b3TransformPoint( transform, localPoint );
}

b3Vec3 b3Body_GetLocalVector( b3BodyId bodyId, b3Vec3 worldVector )
{
	b3World* world = b3GetWorld( bodyId.world0 );
	b3Body* body = b3GetBodyFullId( world, bodyId );
	b3WorldTransform transform = b3GetBodyTransformQuick( world, body );
	return b3InvRotateVector( transform.q, worldVector );
}

b3Vec3 b3Body_GetWorldVector( b3BodyId bodyId, b3Vec3 localVector )
{
	b3World* world = b3GetWorld( bodyId.world0 );
	b3Body* body = b3GetBodyFullId( world, bodyId );
	b3WorldTransform transform = b3GetBodyTransformQuick( world, body );
	return b3RotateVector( transform.q, localVector );
}

void b3Body_SetTransform( b3BodyId bodyId, b3Pos position, b3Quat rotation )
{
	B3_ASSERT( b3IsValidQuat( rotation ) );
	b3World* world = b3GetWorld( bodyId.world0 );
	B3_ASSERT( world->locked == false );

	b3Body* body = b3GetBodyFullId( world, bodyId );
	b3BodySim* bodySim = b3GetBodySim( world, body );

	bodySim->transform.p = position;
	bodySim->transform.q = rotation;
	bodySim->center = b3TransformPoint( bodySim->transform, bodySim->localCenter );

	bodySim->invInertiaWorld = b3RotateInertiaW( bodySim->transform.q, bodySim->invInertiaLocal );

	bodySim->rotation0 = bodySim->transform.q;
	bodySim->center0 = bodySim->center;

	b3BroadPhase* broadPhase = &world->broadPhase;

	b3WorldTransform transform = bodySim->transform;
	const b3f speculativeDistance = B3_SPECULATIVE_DISTANCE;

	int shapeId = body->headShapeId;
	while ( shapeId != B3_NULL_INDEX )
	{
		b3Shape* shape = b3Array_Get( world->shapes, shapeId );
		b3AABB aabb = b3ComputeFatShapeAABB( shape, transform, speculativeDistance );
		shape->aabb = aabb;

		if ( b3AABB_Contains( shape->fatAABB, aabb ) == false )
		{
			b3f margin = shape->aabbMargin;
			b3AABB fatAABB;
			fatAABB.lowerBound.x = b3SubF( aabb.lowerBound.x, margin );
			fatAABB.lowerBound.y = b3SubF( aabb.lowerBound.y, margin );
			fatAABB.lowerBound.z = b3SubF( aabb.lowerBound.z, margin );
			fatAABB.upperBound.x = b3AddF( aabb.upperBound.x, margin );
			fatAABB.upperBound.y = b3AddF( aabb.upperBound.y, margin );
			fatAABB.upperBound.z = b3AddF( aabb.upperBound.z, margin );
			shape->fatAABB = fatAABB;

			// The body could be disabled, in which case it has no proxy.
			if ( shape->proxyKey != B3_NULL_INDEX )
			{
				b3BroadPhase_MoveProxy( broadPhase, shape->proxyKey, fatAABB );
			}
		}

		shapeId = shape->nextShapeId;
	}
}

b3Vec3 b3Body_GetLinearVelocity( b3BodyId bodyId )
{
	b3World* world = b3GetWorld( bodyId.world0 );
	b3Body* body = b3GetBodyFullId( world, bodyId );
	b3BodyState* state = b3GetBodyState( world, body );
	if ( state != NULL )
	{
		return state->linearVelocity;
	}
	return b3Vec3_zero;
}

b3Vec3 b3Body_GetAngularVelocity( b3BodyId bodyId )
{
	b3World* world = b3GetWorld( bodyId.world0 );
	b3Body* body = b3GetBodyFullId( world, bodyId );
	b3BodyState* state = b3GetBodyState( world, body );
	if ( state != NULL )
	{
		return state->angularVelocity;
	}
	return b3Vec3_zero;
}

void b3Body_SetLinearVelocity( b3BodyId bodyId, b3Vec3 linearVelocity )
{
	b3World* world = b3GetWorld( bodyId.world0 );
	b3Body* body = b3GetBodyFullId( world, bodyId );

	if ( body->type == b3_staticBody )
	{
		return;
	}

	if ( b3LengthSquaredWide( linearVelocity ) > 0 )
	{
		b3WakeBodyWithLock( world, body );
	}

	b3BodyState* state = b3GetBodyState( world, body );
	if ( state == NULL )
	{
		return;
	}

	state->linearVelocity = linearVelocity;
}

void b3Body_SetAngularVelocity( b3BodyId bodyId, b3Vec3 angularVelocity )
{
	b3World* world = b3GetWorld( bodyId.world0 );
	b3Body* body = b3GetBodyFullId( world, bodyId );

	if ( body->type == b3_staticBody )
	{
		return;
	}

	// Apply the locks first, so a fully locked axis does not wake the body.
	b3Vec3 w;
	w.x = ( body->flags & b3_lockAngularX ) ? b3f_zero : angularVelocity.x;
	w.y = ( body->flags & b3_lockAngularY ) ? b3f_zero : angularVelocity.y;
	w.z = ( body->flags & b3_lockAngularZ ) ? b3f_zero : angularVelocity.z;

	if ( b3LengthSquaredWide( w ) != 0 )
	{
		b3WakeBodyWithLock( world, body );
	}

	b3BodyState* state = b3GetBodyState( world, body );
	if ( state == NULL )
	{
		return;
	}

	state->angularVelocity = w;
}

b3Vec3 b3Body_GetLocalPointVelocity( b3BodyId bodyId, b3Vec3 localPoint )
{
	b3World* world = b3GetWorld( bodyId.world0 );
	b3Body* body = b3GetBodyFullId( world, bodyId );
	b3BodyState* state = b3GetBodyState( world, body );
	if ( state == NULL )
	{
		return b3Vec3_zero;
	}

	b3SolverSet* set = b3Array_Get( world->solverSets, body->setIndex );
	b3BodySim* bodySim = b3Array_Get( set->bodySims, body->localIndex );

	b3Vec3 r = b3RotateVector( bodySim->transform.q, b3Sub( localPoint, bodySim->localCenter ) );
	return b3Add( state->linearVelocity, b3Cross( state->angularVelocity, r ) );
}

b3Vec3 b3Body_GetWorldPointVelocity( b3BodyId bodyId, b3Pos worldPoint )
{
	b3World* world = b3GetWorld( bodyId.world0 );
	b3Body* body = b3GetBodyFullId( world, bodyId );
	b3BodyState* state = b3GetBodyState( world, body );
	if ( state == NULL )
	{
		return b3Vec3_zero;
	}

	b3SolverSet* set = b3Array_Get( world->solverSets, body->setIndex );
	b3BodySim* bodySim = b3Array_Get( set->bodySims, body->localIndex );

	b3Vec3 r = b3Sub( worldPoint, bodySim->center );
	return b3Add( state->linearVelocity, b3Cross( state->angularVelocity, r ) );
}

// =========================================================================
// Forces and impulses
// =========================================================================

void b3Body_ApplyForce( b3BodyId bodyId, b3Vec3 force, b3Pos point, bool wake )
{
	b3World* world = b3GetWorld( bodyId.world0 );
	b3Body* body = b3GetBodyFullId( world, bodyId );

	if ( wake && body->setIndex >= b3_firstSleepingSet )
	{
		b3WakeBodyWithLock( world, body );
	}

	if ( body->setIndex == b3_awakeSet )
	{
		b3BodySim* bodySim = b3GetBodySim( world, body );
		bodySim->force = b3Add( bodySim->force, force );
		bodySim->torque = b3Add( bodySim->torque, b3Cross( b3Sub( point, bodySim->center ), force ) );
	}
}

void b3Body_ApplyForceToCenter( b3BodyId bodyId, b3Vec3 force, bool wake )
{
	b3World* world = b3GetWorld( bodyId.world0 );
	b3Body* body = b3GetBodyFullId( world, bodyId );

	if ( wake && body->setIndex >= b3_firstSleepingSet )
	{
		b3WakeBodyWithLock( world, body );
	}

	if ( body->setIndex == b3_awakeSet )
	{
		b3BodySim* bodySim = b3GetBodySim( world, body );
		bodySim->force = b3Add( bodySim->force, force );
	}
}

void b3Body_ApplyTorque( b3BodyId bodyId, b3Vec3 torque, bool wake )
{
	b3World* world = b3GetWorld( bodyId.world0 );
	b3Body* body = b3GetBodyFullId( world, bodyId );

	if ( wake && body->setIndex >= b3_firstSleepingSet )
	{
		b3WakeBodyWithLock( world, body );
	}

	if ( body->setIndex == b3_awakeSet )
	{
		b3BodySim* bodySim = b3GetBodySim( world, body );
		bodySim->torque = b3Add( bodySim->torque, torque );
	}
}

/// Clamp a linear velocity to the world's maximum speed.
///
/// The comparison is wide: maxLinearSpeed defaults to 400, whose square is
/// 160000 -- comfortably inside Q12 as a value, but 2.7e12 as a raw product,
/// which is why b3LengthSquaredWide exists.
static void b3ClampLinearSpeed( b3World* world, b3BodyState* state )
{
	b3f maxLinearSpeed = world->maxLinearSpeed;
	int64_t maxSqr = (int64_t)b3Raw( maxLinearSpeed ) * b3Raw( maxLinearSpeed );

	if ( b3LengthSquaredWide( state->linearVelocity ) > maxSqr )
	{
		state->linearVelocity = b3MulSV( maxLinearSpeed, b3Normalize( state->linearVelocity ) );
	}
}

void b3Body_ApplyLinearImpulse( b3BodyId bodyId, b3Vec3 impulse, b3Pos point, bool wake )
{
	b3World* world = b3GetWorld( bodyId.world0 );
	b3Body* body = b3GetBodyFullId( world, bodyId );

	if ( wake && body->setIndex >= b3_firstSleepingSet )
	{
		b3WakeBodyWithLock( world, body );
	}

	if ( body->setIndex == b3_awakeSet )
	{
		int localIndex = body->localIndex;
		b3SolverSet* set = b3Array_Get( world->solverSets, b3_awakeSet );
		b3BodyState* state = b3Array_Get( set->bodyStates, localIndex );
		b3BodySim* bodySim = b3Array_Get( set->bodySims, localIndex );

		state->linearVelocity = b3Add( state->linearVelocity, b3MulWV( bodySim->invMass, impulse ) );
		b3ClampLinearSpeed( world, state );

		b3Vec3 delta = b3MulMWV( bodySim->invInertiaWorld, b3Cross( b3Sub( point, bodySim->center ), impulse ) );
		state->angularVelocity = b3Add( state->angularVelocity, delta );
	}
}

void b3Body_ApplyLinearImpulseToCenter( b3BodyId bodyId, b3Vec3 impulse, bool wake )
{
	b3World* world = b3GetWorld( bodyId.world0 );
	b3Body* body = b3GetBodyFullId( world, bodyId );

	if ( wake && body->setIndex >= b3_firstSleepingSet )
	{
		b3WakeBodyWithLock( world, body );
	}

	if ( body->setIndex == b3_awakeSet )
	{
		int localIndex = body->localIndex;
		b3SolverSet* set = b3Array_Get( world->solverSets, b3_awakeSet );
		b3BodyState* state = b3Array_Get( set->bodyStates, localIndex );
		b3BodySim* bodySim = b3Array_Get( set->bodySims, localIndex );

		state->linearVelocity = b3Add( state->linearVelocity, b3MulWV( bodySim->invMass, impulse ) );
		b3ClampLinearSpeed( world, state );
	}
}

void b3Body_ApplyAngularImpulse( b3BodyId bodyId, b3Vec3 impulse, bool wake )
{
	B3_ASSERT( b3Body_IsValid( bodyId ) );

	b3World* world = b3GetWorld( bodyId.world0 );

	int id = bodyId.index1 - 1;
	b3Body* body = b3Array_Get( world->bodies, id );
	B3_ASSERT( body->generation == bodyId.generation );

	if ( wake && body->setIndex >= b3_firstSleepingSet )
	{
		// This does not invalidate the body pointer.
		b3WakeBodyWithLock( world, body );
	}

	if ( body->setIndex == b3_awakeSet )
	{
		int localIndex = body->localIndex;
		b3SolverSet* set = b3Array_Get( world->solverSets, b3_awakeSet );
		b3BodyState* state = b3Array_Get( set->bodyStates, localIndex );
		b3BodySim* bodySim = b3Array_Get( set->bodySims, localIndex );

		b3Vec3 localImpulse = b3InvRotateVector( bodySim->transform.q, impulse );
		b3Vec3 localAngularVelocityDelta = b3MulMWV( bodySim->invInertiaLocal, localImpulse );
		state->angularVelocity =
			b3Add( state->angularVelocity, b3RotateVector( bodySim->transform.q, localAngularVelocityDelta ) );
	}
}

// =========================================================================
// Body type
// =========================================================================

b3BodyType b3Body_GetType( b3BodyId bodyId )
{
	b3World* world = b3GetWorld( bodyId.world0 );
	b3Body* body = b3GetBodyFullId( world, bodyId );
	return body->type;
}

// This follows the same steps as destroying and recreating the body, its
// shapes and its joints. Contacts are not preserved: the broad-phase pairs
// change, so they are destroyed and rebuilt.
void b3Body_SetType( b3BodyId bodyId, b3BodyType type )
{
	b3World* world = b3GetUnlockedWorld( bodyId.world0 );
	if ( world == NULL )
	{
		return;
	}

	world->locked = true;
	b3Body* body = b3GetBodyFullId( world, bodyId );

	b3BodyType originalType = body->type;
	if ( originalType == type )
	{
		world->locked = false;
		return;
	}

	// Upstream rejects the change here for bodies carrying compound or height
	// field shapes, which must stay static. Neither type exists in the port.

	// Stage 1: disabled bodies do not change solver sets or islands.
	if ( body->setIndex == b3_disabledSet )
	{
		body->type = type;

		if ( type == b3_dynamicBody )
		{
			body->flags |= b3_dynamicFlag;
		}
		else
		{
			body->flags &= ~b3_dynamicFlag;
		}

		b3SyncBodyFlags( world, body );

		// The body type affects the mass properties.
		b3UpdateBodyMassData( world, body );
		world->locked = false;
		return;
	}

	// Stage 2: destroy all contacts, without waking bodies.
	bool wakeBodies = false;
	b3DestroyBodyContacts( world, body, wakeBodies );

	// Stage 3: wake this body. Does nothing for a static body; otherwise it
	// wakes every body in the same sleeping set.
	b3WakeBody( world, body );

	// Stage 4: move joints to temporary storage. Empty until Phase 6.
	b3SolverSet* staticSet = b3Array_Get( world->solverSets, b3_staticSet );

	int jointKey = body->headJointKey;
	while ( jointKey != B3_NULL_INDEX )
	{
		int jointId = jointKey >> 1;
		int edgeIndex = jointKey & 1;

		b3Joint* joint = b3Array_Get( world->joints, jointId );
		jointKey = joint->edges[edgeIndex].nextKey;

		// The joint may be disabled by the other body.
		if ( joint->setIndex == b3_disabledSet )
		{
			continue;
		}

		// b3WakeBody above does not wake bodies attached to a static body, and
		// the body may have no joints at all, so wake both ends explicitly.
		b3Body* bodyA = b3Array_Get( world->bodies, joint->edges[0].bodyId );
		b3Body* bodyB = b3Array_Get( world->bodies, joint->edges[1].bodyId );
		b3WakeBody( world, bodyA );
		b3WakeBody( world, bodyB );

		b3UnlinkJoint( world, joint );

		// Every joint has to pass through the static set so they are added to
		// the constraint graph below with consistent colours.
		b3SolverSet* jointSourceSet = b3Array_Get( world->solverSets, joint->setIndex );
		b3TransferJoint( world, staticSet, jointSourceSet, joint );
	}

	// Stage 5: change the type and transfer the body.
	body->type = type;

	if ( type == b3_dynamicBody )
	{
		body->flags |= b3_dynamicFlag;
	}
	else
	{
		body->flags &= ~b3_dynamicFlag;
	}

	b3SolverSet* awakeSet = b3Array_Get( world->solverSets, b3_awakeSet );
	b3SolverSet* sourceSet = b3Array_Get( world->solverSets, body->setIndex );
	b3SolverSet* targetSet = type == b3_staticBody ? staticSet : awakeSet;

	b3TransferBody( world, targetSet, sourceSet, body );

	// Stage 6: update island participation.
	if ( originalType == b3_staticBody )
	{
		b3CreateIslandForBody( world, b3_awakeSet, body );
	}
	else if ( type == b3_staticBody )
	{
		b3RemoveBodyFromIsland( world, body );
	}

	// Stage 7: transfer joints to the target set.
	jointKey = body->headJointKey;
	while ( jointKey != B3_NULL_INDEX )
	{
		int jointId = jointKey >> 1;
		int edgeIndex = jointKey & 1;

		b3Joint* joint = b3Array_Get( world->joints, jointId );

		jointKey = joint->edges[edgeIndex].nextKey;

		if ( joint->setIndex == b3_disabledSet )
		{
			continue;
		}

		B3_ASSERT( joint->setIndex == b3_staticSet );

		b3Body* bodyA = b3Array_Get( world->bodies, joint->edges[0].bodyId );
		b3Body* bodyB = b3Array_Get( world->bodies, joint->edges[1].bodyId );
		B3_ASSERT( bodyA->setIndex == b3_staticSet || bodyA->setIndex == b3_awakeSet );
		B3_ASSERT( bodyB->setIndex == b3_staticSet || bodyB->setIndex == b3_awakeSet );

		if ( bodyA->type == b3_dynamicBody || bodyB->type == b3_dynamicBody )
		{
			b3TransferJoint( world, awakeSet, staticSet, joint );
		}
	}

	// Stage 8: recreate shape proxies in the correct tree.
	b3WorldTransform transform = b3GetBodyTransformQuick( world, body );
	int shapeId = body->headShapeId;
	while ( shapeId != B3_NULL_INDEX )
	{
		b3Shape* shape = b3Array_Get( world->shapes, shapeId );

		shapeId = shape->nextShapeId;
		b3DestroyShapeProxy( shape, &world->broadPhase );
		bool forcePairCreation = true;
		b3CreateShapeProxy( shape, &world->broadPhase, type, transform, forcePairCreation );
	}

	// Relink all joints.
	jointKey = body->headJointKey;
	while ( jointKey != B3_NULL_INDEX )
	{
		int jointId = jointKey >> 1;
		int edgeIndex = jointKey & 1;

		b3Joint* joint = b3Array_Get( world->joints, jointId );
		jointKey = joint->edges[edgeIndex].nextKey;

		int otherEdgeIndex = edgeIndex ^ 1;
		int otherBodyId = joint->edges[otherEdgeIndex].bodyId;
		b3Body* otherBody = b3Array_Get( world->bodies, otherBodyId );

		if ( otherBody->setIndex == b3_disabledSet )
		{
			continue;
		}

		if ( body->type != b3_dynamicBody && otherBody->type != b3_dynamicBody )
		{
			continue;
		}

		b3LinkJoint( world, joint );
	}

	b3SyncBodyFlags( world, body );

	// The body type affects the mass.
	b3UpdateBodyMassData( world, body );

	b3ValidateSolverSets( world );
	b3ValidateIsland( world, body->islandId );

	world->locked = false;
}

// =========================================================================
// Mass accessors
// =========================================================================

b3f b3Body_GetMass( b3BodyId bodyId )
{
	b3World* world = b3GetWorld( bodyId.world0 );
	b3Body* body = b3GetBodyFullId( world, bodyId );
	return body->mass;
}

b3Matrix3 b3Body_GetLocalRotationalInertia( b3BodyId bodyId )
{
	b3World* world = b3GetWorld( bodyId.world0 );
	b3Body* body = b3GetBodyFullId( world, bodyId );
	return body->inertia;
}

b3iw b3Body_GetInverseMass( b3BodyId bodyId )
{
	b3World* world = b3GetWorld( bodyId.world0 );
	b3Body* body = b3GetBodyFullId( world, bodyId );
	b3BodySim* bodySim = b3GetBodySim( world, body );
	return bodySim->invMass;
}

b3MatrixW b3Body_GetWorldInverseRotationalInertia( b3BodyId bodyId )
{
	b3World* world = b3GetWorld( bodyId.world0 );
	b3Body* body = b3GetBodyFullId( world, bodyId );
	b3BodySim* bodySim = b3GetBodySim( world, body );
	return bodySim->invInertiaWorld;
}

b3Vec3 b3Body_GetLocalCenter( b3BodyId bodyId )
{
	b3World* world = b3GetWorld( bodyId.world0 );
	b3Body* body = b3GetBodyFullId( world, bodyId );
	b3BodySim* bodySim = b3GetBodySim( world, body );
	return bodySim->localCenter;
}

b3Pos b3Body_GetWorldCenter( b3BodyId bodyId )
{
	b3World* world = b3GetWorld( bodyId.world0 );
	b3Body* body = b3GetBodyFullId( world, bodyId );
	b3BodySim* bodySim = b3GetBodySim( world, body );
	return bodySim->center;
}

void b3Body_SetMassData( b3BodyId bodyId, b3MassData massData )
{
	B3_ASSERT( b3Raw( massData.mass ) >= 0 );

	b3World* world = b3GetUnlockedWorld( bodyId.world0 );
	if ( world == NULL )
	{
		return;
	}

	b3Body* body = b3GetBodyFullId( world, bodyId );
	b3BodySim* bodySim = b3GetBodySim( world, body );

	// Mass is no longer dirty.
	body->flags &= ~b3_dirtyMass;
	b3SyncBodyFlags( world, body );

	// massData.inertia is per unit mass, matching b3MassData's convention
	// throughout the port. A value from b3Body_GetMassData round trips.
	body->mass = massData.mass;
	body->inertia = massData.inertia;
	bodySim->localCenter = massData.center;

	b3Pos oldCenter = bodySim->center;
	b3Pos center = b3TransformPoint( bodySim->transform, massData.center );
	bodySim->center = center;
	bodySim->center0 = center;
	bodySim->invMass = b3Raw( body->mass ) > 0 ? b3RcpF( body->mass ) : b3iw_zero;

	// Update the centre of mass velocity.
	b3BodyState* state = b3GetBodyState( world, body );
	if ( state != NULL )
	{
		b3Vec3 deltaLinear = b3Cross( state->angularVelocity, b3Sub( bodySim->center, oldCenter ) );
		state->linearVelocity = b3Add( state->linearVelocity, deltaLinear );
	}

	// b3InvertInertia returns zero for a singular or non-positive-definite
	// tensor, so upstream's explicit determinant branch is inside it.
	bodySim->invInertiaLocal = b3InvertInertia( body->inertia, body->mass );
	bodySim->invInertiaWorld = b3RotateInertiaW( bodySim->transform.q, bodySim->invInertiaLocal );

	// Apply fixed rotation.
	if ( ( bodySim->flags & b3_fixedRotation ) == b3_fixedRotation )
	{
		body->inertia = b3Mat3_zero;
		bodySim->invInertiaLocal = b3MatW_zero;
		bodySim->invInertiaWorld = b3MatW_zero;
	}

	// Update extents using the supplied mass centre.
	bodySim->minExtent = B3_HUGE;
	bodySim->maxExtent = b3Vec3_zero;
	int shapeId = body->headShapeId;
	while ( shapeId != B3_NULL_INDEX )
	{
		const b3Shape* s = b3Array_Get( world->shapes, shapeId );
		b3ShapeExtent extent = b3ComputeShapeExtent( s, massData.center );
		bodySim->minExtent = b3MinF( bodySim->minExtent, extent.minExtent );
		bodySim->maxExtent = b3Max( bodySim->maxExtent, extent.maxExtent );
		shapeId = s->nextShapeId;
	}
}

b3MassData b3Body_GetMassData( b3BodyId bodyId )
{
	b3World* world = b3GetWorld( bodyId.world0 );
	b3Body* body = b3GetBodyFullId( world, bodyId );
	b3BodySim* bodySim = b3GetBodySim( world, body );
	b3MassData massData = { body->mass, bodySim->localCenter, body->inertia };
	return massData;
}

void b3Body_ApplyMassFromShapes( b3BodyId bodyId )
{
	b3World* world = b3GetUnlockedWorld( bodyId.world0 );
	if ( world == NULL )
	{
		return;
	}

	b3Body* body = b3GetBodyFullId( world, bodyId );
	b3UpdateBodyMassData( world, body );
}

// =========================================================================
// Damping, gravity and sleep
// =========================================================================

void b3Body_SetLinearDamping( b3BodyId bodyId, b3f linearDamping )
{
	B3_ASSERT( b3Raw( linearDamping ) >= 0 );

	b3World* world = b3GetUnlockedWorld( bodyId.world0 );
	if ( world == NULL )
	{
		return;
	}

	b3Body* body = b3GetBodyFullId( world, bodyId );
	b3BodySim* bodySim = b3GetBodySim( world, body );
	bodySim->linearDamping = linearDamping;
}

b3f b3Body_GetLinearDamping( b3BodyId bodyId )
{
	b3World* world = b3GetWorld( bodyId.world0 );
	b3Body* body = b3GetBodyFullId( world, bodyId );
	b3BodySim* bodySim = b3GetBodySim( world, body );
	return bodySim->linearDamping;
}

void b3Body_SetAngularDamping( b3BodyId bodyId, b3f angularDamping )
{
	B3_ASSERT( b3Raw( angularDamping ) >= 0 );

	b3World* world = b3GetUnlockedWorld( bodyId.world0 );
	if ( world == NULL )
	{
		return;
	}

	b3Body* body = b3GetBodyFullId( world, bodyId );
	b3BodySim* bodySim = b3GetBodySim( world, body );
	bodySim->angularDamping = angularDamping;
}

b3f b3Body_GetAngularDamping( b3BodyId bodyId )
{
	b3World* world = b3GetWorld( bodyId.world0 );
	b3Body* body = b3GetBodyFullId( world, bodyId );
	b3BodySim* bodySim = b3GetBodySim( world, body );
	return bodySim->angularDamping;
}

void b3Body_SetGravityScale( b3BodyId bodyId, b3f gravityScale )
{
	b3World* world = b3GetUnlockedWorld( bodyId.world0 );
	if ( world == NULL )
	{
		return;
	}

	b3Body* body = b3GetBodyFullId( world, bodyId );
	b3BodySim* bodySim = b3GetBodySim( world, body );
	bodySim->gravityScale = gravityScale;
}

b3f b3Body_GetGravityScale( b3BodyId bodyId )
{
	b3World* world = b3GetWorld( bodyId.world0 );
	b3Body* body = b3GetBodyFullId( world, bodyId );
	b3BodySim* bodySim = b3GetBodySim( world, body );
	return bodySim->gravityScale;
}

bool b3Body_IsAwake( b3BodyId bodyId )
{
	b3World* world = b3GetWorld( bodyId.world0 );
	b3Body* body = b3GetBodyFullId( world, bodyId );
	return body->setIndex == b3_awakeSet;
}

void b3Body_SetAwake( b3BodyId bodyId, bool awake )
{
	b3World* world = b3GetUnlockedWorld( bodyId.world0 );
	if ( world == NULL )
	{
		return;
	}

	world->locked = true;

	b3Body* body = b3GetBodyFullId( world, bodyId );

	if ( awake && body->setIndex >= b3_firstSleepingSet )
	{
		b3WakeBody( world, body );
	}
	else if ( awake == false && body->setIndex == b3_awakeSet )
	{
		b3Island* island = b3Array_Get( world->islands, body->islandId );
		if ( island->constraintRemoveCount > 0 )
		{
			// The island has to be split before it can sleep. This is expensive.
			b3SplitIsland( world, body->islandId );
		}

		b3TrySleepIsland( world, body->islandId );
	}

	world->locked = false;
}

bool b3Body_IsEnabled( b3BodyId bodyId )
{
	b3World* world = b3GetWorld( bodyId.world0 );
	b3Body* body = b3GetBodyFullId( world, bodyId );
	return body->setIndex != b3_disabledSet;
}

bool b3Body_IsSleepEnabled( b3BodyId bodyId )
{
	b3World* world = b3GetWorld( bodyId.world0 );
	b3Body* body = b3GetBodyFullId( world, bodyId );
	return ( body->flags & b3_enableSleep ) == b3_enableSleep;
}

void b3Body_SetSleepThreshold( b3BodyId bodyId, b3f sleepThreshold )
{
	b3World* world = b3GetWorld( bodyId.world0 );
	b3Body* body = b3GetBodyFullId( world, bodyId );
	body->sleepThreshold = sleepThreshold;
}

b3f b3Body_GetSleepThreshold( b3BodyId bodyId )
{
	b3World* world = b3GetWorld( bodyId.world0 );
	b3Body* body = b3GetBodyFullId( world, bodyId );
	return body->sleepThreshold;
}

void b3Body_EnableSleep( b3BodyId bodyId, bool enableSleep )
{
	b3World* world = b3GetUnlockedWorld( bodyId.world0 );
	if ( world == NULL )
	{
		return;
	}

	b3Body* body = b3GetBodyFullId( world, bodyId );

	bool flag = ( body->flags & b3_enableSleep ) == b3_enableSleep;
	if ( enableSleep == flag )
	{
		return;
	}

	world->locked = true;

	body->flags = enableSleep ? body->flags | b3_enableSleep : body->flags & ~b3_enableSleep;
	b3SyncBodyFlags( world, body );

	if ( enableSleep == false )
	{
		b3WakeBody( world, body );
	}

	world->locked = false;
}

// =========================================================================
// Enable and disable
// =========================================================================

// Disabling a body is a lot of detailed bookkeeping, but it is a valuable
// feature. The hard part is that joints may connect to bodies that stay
// enabled.
void b3Body_Disable( b3BodyId bodyId )
{
	b3World* world = b3GetUnlockedWorld( bodyId.world0 );
	if ( world == NULL )
	{
		return;
	}

	world->locked = true;

	b3Body* body = b3GetBodyFullId( world, bodyId );
	if ( body->setIndex == b3_disabledSet )
	{
		world->locked = false;
		return;
	}

	// Destroy contacts and wake the bodies touching this one, so nothing is
	// left floating. Necessary even for a static body.
	bool wakeBodies = true;
	b3DestroyBodyContacts( world, body, wakeBodies );

	b3SolverSet* set = b3Array_Get( world->solverSets, body->setIndex );
	b3SolverSet* disabledSet = b3Array_Get( world->solverSets, b3_disabledSet );

	// Unlink joints and transfer them to the disabled set.
	int jointKey = body->headJointKey;
	while ( jointKey != B3_NULL_INDEX )
	{
		int jointId = jointKey >> 1;
		int edgeIndex = jointKey & 1;

		b3Joint* joint = b3Array_Get( world->joints, jointId );
		jointKey = joint->edges[edgeIndex].nextKey;

		// The joint may already be disabled by the other body.
		if ( joint->setIndex == b3_disabledSet )
		{
			continue;
		}

		B3_ASSERT( joint->setIndex == set->setIndex || set->setIndex == b3_staticSet );

		b3UnlinkJoint( world, joint );

		b3SolverSet* jointSet = b3Array_Get( world->solverSets, joint->setIndex );
		b3TransferJoint( world, disabledSet, jointSet, joint );
	}

	// Remove shapes from the broad phase.
	int shapeId = body->headShapeId;
	while ( shapeId != B3_NULL_INDEX )
	{
		b3Shape* shape = b3Array_Get( world->shapes, shapeId );
		shapeId = shape->nextShapeId;
		b3DestroyShapeProxy( shape, &world->broadPhase );
	}

	// Disabled bodies are not in an island. An emptied island is destroyed.
	b3RemoveBodyFromIsland( world, body );

	b3TransferBody( world, disabledSet, set, body );

	b3ValidateConnectivity( world );
	b3ValidateSolverSets( world );

	world->locked = false;
}

void b3Body_Enable( b3BodyId bodyId )
{
	b3World* world = b3GetUnlockedWorld( bodyId.world0 );
	if ( world == NULL )
	{
		return;
	}

	b3Body* body = b3GetBodyFullId( world, bodyId );
	if ( body->setIndex != b3_disabledSet )
	{
		return;
	}

	b3SolverSet* disabledSet = b3Array_Get( world->solverSets, b3_disabledSet );
	int setId = body->type == b3_staticBody ? b3_staticSet : b3_awakeSet;
	b3SolverSet* targetSet = b3Array_Get( world->solverSets, setId );

	b3TransferBody( world, targetSet, disabledSet, body );

	b3WorldTransform transform = b3GetBodyTransformQuick( world, body );

	// Add shapes back to the broad phase.
	b3BodyType proxyType = body->type;
	bool forcePairCreation = true;
	int shapeId = body->headShapeId;
	while ( shapeId != B3_NULL_INDEX )
	{
		b3Shape* shape = b3Array_Get( world->shapes, shapeId );
		shapeId = shape->nextShapeId;

		b3CreateShapeProxy( shape, &world->broadPhase, proxyType, transform, forcePairCreation );
	}

	if ( setId != b3_staticSet )
	{
		b3CreateIslandForBody( world, setId, body );
	}

	// Transfer joints. If the other body is still disabled, leave the joint.
	int jointKey = body->headJointKey;
	while ( jointKey != B3_NULL_INDEX )
	{
		int jointId = jointKey >> 1;
		int edgeIndex = jointKey & 1;

		b3Joint* joint = b3Array_Get( world->joints, jointId );
		B3_ASSERT( joint->setIndex == b3_disabledSet );
		B3_ASSERT( joint->islandId == B3_NULL_INDEX );

		jointKey = joint->edges[edgeIndex].nextKey;

		b3Body* bodyA = b3Array_Get( world->bodies, joint->edges[0].bodyId );
		b3Body* bodyB = b3Array_Get( world->bodies, joint->edges[1].bodyId );

		if ( bodyA->setIndex == b3_disabledSet || bodyB->setIndex == b3_disabledSet )
		{
			continue;
		}

		int jointSetId;
		if ( bodyA->setIndex == b3_staticSet && bodyB->setIndex == b3_staticSet )
		{
			jointSetId = b3_staticSet;
		}
		else if ( bodyA->setIndex == b3_staticSet )
		{
			jointSetId = bodyB->setIndex;
		}
		else
		{
			jointSetId = bodyA->setIndex;
		}

		b3SolverSet* jointSet = b3Array_Get( world->solverSets, jointSetId );
		b3TransferJoint( world, jointSet, disabledSet, joint );

		// Now the joint is in the right set it can be linked into the island.
		if ( jointSetId != b3_staticSet )
		{
			b3LinkJoint( world, joint );
		}
	}

	b3ValidateSolverSets( world );
}

// =========================================================================
// Flags
// =========================================================================

void b3Body_SetMotionLocks( b3BodyId bodyId, b3MotionLocks locks )
{
	b3World* world = b3GetUnlockedWorld( bodyId.world0 );
	if ( world == NULL )
	{
		return;
	}

	uint32_t newLocks = 0;
	newLocks |= locks.linearX ? b3_lockLinearX : 0;
	newLocks |= locks.linearY ? b3_lockLinearY : 0;
	newLocks |= locks.linearZ ? b3_lockLinearZ : 0;
	newLocks |= locks.angularX ? b3_lockAngularX : 0;
	newLocks |= locks.angularY ? b3_lockAngularY : 0;
	newLocks |= locks.angularZ ? b3_lockAngularZ : 0;

	b3Body* body = b3GetBodyFullId( world, bodyId );
	if ( ( body->flags & b3_allLocks ) == newLocks )
	{
		return;
	}

	bool fixedRotation1 = ( body->flags & b3_fixedRotation ) == b3_fixedRotation;
	bool fixedRotation2 = ( newLocks & b3_fixedRotation ) == b3_fixedRotation;

	body->flags &= ~b3_allLocks;
	body->flags |= newLocks;

	b3SyncBodyFlags( world, body );

	b3BodyState* state = b3GetBodyState( world, body );

	if ( state != NULL )
	{
		if ( locks.linearX )
		{
			state->linearVelocity.x = b3f_zero;
		}

		if ( locks.linearY )
		{
			state->linearVelocity.y = b3f_zero;
		}

		if ( locks.linearZ )
		{
			state->linearVelocity.z = b3f_zero;
		}

		if ( locks.angularX )
		{
			state->angularVelocity.x = b3f_zero;
		}

		if ( locks.angularY )
		{
			state->angularVelocity.y = b3f_zero;
		}

		if ( locks.angularZ )
		{
			state->angularVelocity.z = b3f_zero;
		}
	}

	if ( fixedRotation1 != fixedRotation2 )
	{
		b3UpdateBodyMassData( world, body );
	}
}

b3MotionLocks b3Body_GetMotionLocks( b3BodyId bodyId )
{
	b3World* world = b3GetWorld( bodyId.world0 );
	b3Body* body = b3GetBodyFullId( world, bodyId );

	b3MotionLocks locks;
	locks.linearX = ( body->flags & b3_lockLinearX ) != 0;
	locks.linearY = ( body->flags & b3_lockLinearY ) != 0;
	locks.linearZ = ( body->flags & b3_lockLinearZ ) != 0;
	locks.angularX = ( body->flags & b3_lockAngularX ) != 0;
	locks.angularY = ( body->flags & b3_lockAngularY ) != 0;
	locks.angularZ = ( body->flags & b3_lockAngularZ ) != 0;
	return locks;
}

void b3Body_SetBullet( b3BodyId bodyId, bool flag )
{
	b3World* world = b3GetUnlockedWorld( bodyId.world0 );
	if ( world == NULL )
	{
		return;
	}

	uint32_t newFlag = flag ? b3_isBullet : 0;

	b3Body* body = b3GetBodyFullId( world, bodyId );
	if ( ( body->flags & b3_isBullet ) == newFlag )
	{
		return;
	}

	body->flags &= ~b3_isBullet;
	body->flags |= newFlag;

	b3SyncBodyFlags( world, body );
}

bool b3Body_IsBullet( b3BodyId bodyId )
{
	b3World* world = b3GetWorld( bodyId.world0 );
	b3Body* body = b3GetBodyFullId( world, bodyId );
	return ( body->flags & b3_isBullet ) != 0;
}

void b3Body_AllowFastRotation( b3BodyId bodyId, bool flag )
{
	b3World* world = b3GetUnlockedWorld( bodyId.world0 );
	if ( world == NULL )
	{
		return;
	}

	uint32_t newFlag = flag ? b3_allowFastRotation : 0;

	b3Body* body = b3GetBodyFullId( world, bodyId );
	if ( ( body->flags & b3_allowFastRotation ) == newFlag )
	{
		return;
	}

	body->flags &= ~b3_allowFastRotation;
	body->flags |= newFlag;

	b3SyncBodyFlags( world, body );
}

bool b3Body_IsFastRotationAllowed( b3BodyId bodyId )
{
	b3World* world = b3GetWorld( bodyId.world0 );
	b3Body* body = b3GetBodyFullId( world, bodyId );
	return ( body->flags & b3_allowFastRotation ) != 0;
}

void b3Body_EnableContactRecycling( b3BodyId bodyId, bool flag )
{
	b3World* world = b3GetUnlockedWorld( bodyId.world0 );
	if ( world == NULL )
	{
		return;
	}

	uint32_t newFlag = flag ? b3_bodyEnableContactRecycling : 0;

	b3Body* body = b3GetBodyFullId( world, bodyId );
	if ( ( body->flags & b3_bodyEnableContactRecycling ) == newFlag )
	{
		return;
	}

	body->flags &= ~b3_bodyEnableContactRecycling;
	body->flags |= newFlag;

	b3SyncBodyFlags( world, body );
}

bool b3Body_IsContactRecyclingEnabled( b3BodyId bodyId )
{
	b3World* world = b3GetWorld( bodyId.world0 );
	b3Body* body = b3GetBodyFullId( world, bodyId );
	return ( body->flags & b3_bodyEnableContactRecycling ) != 0;
}

void b3Body_EnableHitEvents( b3BodyId bodyId, bool flag )
{
	b3World* world = b3GetWorld( bodyId.world0 );
	b3Body* body = b3GetBodyFullId( world, bodyId );
	int shapeId = body->headShapeId;
	while ( shapeId != B3_NULL_INDEX )
	{
		b3Shape* shape = b3Array_Get( world->shapes, shapeId );
		shape->flags = flag ? shape->flags | b3_enableHitEvents : shape->flags & ~b3_enableHitEvents;
		shapeId = shape->nextShapeId;
	}
}

// =========================================================================
// Queries
// =========================================================================

b3WorldId b3Body_GetWorld( b3BodyId bodyId )
{
	b3World* world = b3GetWorld( bodyId.world0 );
	return ( b3WorldId ){ (uint16_t)( bodyId.world0 + 1 ), world->generation };
}

void b3Body_SetUserData( b3BodyId bodyId, void* userData )
{
	b3World* world = b3GetWorld( bodyId.world0 );
	b3Body* body = b3GetBodyFullId( world, bodyId );
	body->userData = userData;
}

void* b3Body_GetUserData( b3BodyId bodyId )
{
	b3World* world = b3GetWorld( bodyId.world0 );
	b3Body* body = b3GetBodyFullId( world, bodyId );
	return body->userData;
}

int b3Body_GetShapeCount( b3BodyId bodyId )
{
	b3World* world = b3GetWorld( bodyId.world0 );
	b3Body* body = b3GetBodyFullId( world, bodyId );
	return body->shapeCount;
}

int b3Body_GetShapes( b3BodyId bodyId, b3ShapeId* shapeArray, int capacity )
{
	b3World* world = b3GetWorld( bodyId.world0 );
	b3Body* body = b3GetBodyFullId( world, bodyId );
	int shapeId = body->headShapeId;
	int shapeCount = 0;
	while ( shapeId != B3_NULL_INDEX && shapeCount < capacity )
	{
		b3Shape* shape = b3Array_Get( world->shapes, shapeId );
		b3ShapeId id = { shape->id + 1, bodyId.world0, shape->generation };
		shapeArray[shapeCount] = id;
		shapeCount += 1;

		shapeId = shape->nextShapeId;
	}

	return shapeCount;
}

int b3Body_GetJointCount( b3BodyId bodyId )
{
	b3World* world = b3GetWorld( bodyId.world0 );
	b3Body* body = b3GetBodyFullId( world, bodyId );
	return body->jointCount;
}

int b3Body_GetJoints( b3BodyId bodyId, b3JointId* jointArray, int capacity )
{
	b3World* world = b3GetWorld( bodyId.world0 );
	b3Body* body = b3GetBodyFullId( world, bodyId );
	int jointKey = body->headJointKey;

	int jointCount = 0;
	while ( jointKey != B3_NULL_INDEX && jointCount < capacity )
	{
		int jointId = jointKey >> 1;
		int edgeIndex = jointKey & 1;

		b3Joint* joint = b3Array_Get( world->joints, jointId );

		b3JointId id = { jointId + 1, bodyId.world0, joint->generation };
		jointArray[jointCount] = id;
		jointCount += 1;

		jointKey = joint->edges[edgeIndex].nextKey;
	}

	return jointCount;
}

b3AABB b3Body_ComputeAABB( b3BodyId bodyId )
{
	b3World* world = b3GetWorld( bodyId.world0 );
	if ( world == NULL )
	{
		return ( b3AABB ){ 0 };
	}

	b3Body* body = b3GetBodyFullId( world, bodyId );
	if ( body->headShapeId == B3_NULL_INDEX )
	{
		b3WorldTransform transform = b3GetBodyTransform( world, body->id );
		b3AABB aabb = { transform.p, transform.p };
		return aabb;
	}

	b3Shape* shape = b3Array_Get( world->shapes, body->headShapeId );
	b3AABB aabb = shape->aabb;
	while ( shape->nextShapeId != B3_NULL_INDEX )
	{
		shape = b3Array_Get( world->shapes, shape->nextShapeId );
		aabb = b3AABB_Union( aabb, shape->aabb );
	}

	return aabb;
}

bool b3ShouldBodiesCollide( b3World* world, b3Body* bodyA, b3Body* bodyB )
{
	if ( bodyA->type != b3_dynamicBody && bodyB->type != b3_dynamicBody )
	{
		return false;
	}

	int jointKey;
	int otherBodyId;
	if ( bodyA->jointCount < bodyB->jointCount )
	{
		jointKey = bodyA->headJointKey;
		otherBodyId = bodyB->id;
	}
	else
	{
		jointKey = bodyB->headJointKey;
		otherBodyId = bodyA->id;
	}

	while ( jointKey != B3_NULL_INDEX )
	{
		int jointId = jointKey >> 1;
		int edgeIndex = jointKey & 1;
		int otherEdgeIndex = edgeIndex ^ 1;

		b3Joint* joint = b3Array_Get( world->joints, jointId );
		if ( joint->collideConnected == false && joint->edges[otherEdgeIndex].bodyId == otherBodyId )
		{
			return false;
		}

		jointKey = joint->edges[edgeIndex].nextKey;
	}

	return true;
}
