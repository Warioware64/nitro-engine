// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Erin Catto        (original Box3D)
// Copyright (c) 2026 Warioware64       (Nitro Engine Advanced fixed-point port)
//
// This file is part of Nitro Engine Advanced

/// @file   contact.c
/// @brief  The contact lifetime functions Phase 3A needs. The rest is 3B.
///
/// @section why Why these two, now
///
/// Phase 3A creates no contacts, but it *destroys* them: b3DestroyShape,
/// b3Body_Disable, b3Body_SetType and b3DestroyBody all walk a body's contact
/// list and tear down what they find. So b3DestroyContact has to exist even
/// though `world->contacts` is always empty here, exactly as
/// b3DestroyJointInternal does.
///
/// Both functions are transliterated rather than stubbed, and neither is
/// narrow-phase work: b3DestroyContact is edge-list surgery, island unlinking
/// and solver-set bookkeeping, and the register table is a static array of
/// which shape pairs collide at all. Phase 3B would have written both
/// unchanged.
///
/// Phase 3B adds: b3CreateContact, b3UpdateContact, b3ComputeConvexManifold
/// and the narrow-phase dispatch that reads the register table below.

#include "contact.h"

#include "body.h"
#include "constraint_graph.h"
#include "core.h"
#include "id_pool.h"
#include "island.h"
#include "manifold.h"
#include "mesh_contact.h"
#include "physics_world.h"
#include "shape.h"
#include "solver_set.h"
#include "table.h"

/// Which shape pairs the narrow phase can handle, and in which order.
///
/// `primary` records which way round the pair was registered, so a
/// capsule-versus-sphere query and a sphere-versus-capsule query both reach
/// b3CollideCapsuleAndSphere with the arguments the right way round.
struct b3ContactRegister
{
	bool supported;
	bool primary;
};

static struct b3ContactRegister s_registers[b3_shapeTypeCount][b3_shapeTypeCount];
static bool s_initialized = false;

static b3Contact* b3GetContactFullId( b3World* world, b3ContactId contactId )
{
	int id = contactId.index1 - 1;
	b3Contact* contact = b3Array_Get( world->contacts, id );
	B3_ASSERT( contact->contactId == id && contact->generation == contactId.generation );
	return contact;
}

b3ContactData b3Contact_GetData( b3ContactId contactId )
{
	b3World* world = b3GetWorld( contactId.world0 );
	b3Contact* contact = b3GetContactFullId( world, contactId );

	const b3Shape* shapeA = b3Array_Get( world->shapes, contact->shapeIdA );
	const b3Shape* shapeB = b3Array_Get( world->shapes, contact->shapeIdB );

	b3ContactData data = { 0 };
	data.contactId = contactId;
	data.shapeIdA = ( b3ShapeId ){
		.index1 = shapeA->id + 1,
		.world0 = contactId.world0,
		.generation = shapeA->generation,
	};
	data.shapeIdB = ( b3ShapeId ){
		.index1 = shapeB->id + 1,
		.world0 = contactId.world0,
		.generation = shapeB->generation,
	};

	if ( contact->manifoldCount > 0 )
	{
		data.manifolds = contact->manifolds;
		data.manifoldCount = contact->manifoldCount;
	}
	else
	{
		data.manifolds = NULL;
		data.manifoldCount = 0;
	}

	return data;
}

static void b3AddType( b3ShapeType type1, b3ShapeType type2 )
{
	B3_ASSERT( 0 <= type1 && type1 < b3_shapeTypeCount );
	B3_ASSERT( 0 <= type2 && type2 < b3_shapeTypeCount );

	s_registers[type1][type2].supported = true;
	s_registers[type1][type2].primary = true;

	if ( type1 != type2 )
	{
		s_registers[type2][type1].supported = true;
		s_registers[type2][type1].primary = false;
	}
}

void b3InitializeContactRegisters( void )
{
	if ( s_initialized == false )
	{
		// The six convex pairs, then the three mesh rows. Upstream also
		// registers the compound and height field rows; neither is on any
		// phase.
		b3AddType( b3_sphereShape, b3_sphereShape );
		b3AddType( b3_capsuleShape, b3_sphereShape );
		b3AddType( b3_capsuleShape, b3_capsuleShape );
		b3AddType( b3_hullShape, b3_sphereShape );
		b3AddType( b3_hullShape, b3_capsuleShape );
		b3AddType( b3_hullShape, b3_hullShape );

		// Mesh first in every pair, which makes it shape A: b3AddType marks the
		// argument order primary and the reverse secondary, and b3CreateContact
		// re-enters with the arguments swapped for a secondary hit. So the mesh
		// narrow phase never has to handle a flipped pair, and there is
		// deliberately no mesh-versus-mesh row -- two triangle soups have no
		// narrow phase, here or upstream.
		b3AddType( b3_meshShape, b3_sphereShape );
		b3AddType( b3_meshShape, b3_capsuleShape );
		b3AddType( b3_meshShape, b3_hullShape );
		s_initialized = true;
	}
}

void b3CreateContact( b3World* world, b3Shape* shapeA, b3Shape* shapeB, int childIndex )
{
	b3ShapeType typeA = shapeA->type;
	b3ShapeType typeB = shapeB->type;

	B3_ASSERT( 0 <= typeA && typeA < b3_shapeTypeCount );
	B3_ASSERT( 0 <= typeB && typeB < b3_shapeTypeCount );

	if ( s_registers[typeA][typeB].supported == false )
	{
		// This pair has no narrow phase at all.
		return;
	}

	if ( s_registers[typeA][typeB].primary == false )
	{
		// Registered the other way round, so retry with the arguments swapped.
		// This is what guarantees the narrow phase always sees, say, hull as A
		// and sphere as B.
		b3CreateContact( world, shapeB, shapeA, childIndex );
		return;
	}

	b3Body* bodyA = b3Array_Get( world->bodies, shapeA->bodyId );
	b3Body* bodyB = b3Array_Get( world->bodies, shapeB->bodyId );

	B3_ASSERT( bodyA->setIndex != b3_disabledSet && bodyB->setIndex != b3_disabledSet );
	B3_ASSERT( bodyA->setIndex != b3_staticSet || bodyB->setIndex != b3_staticSet );

	int setIndex;
	if ( bodyA->setIndex == b3_awakeSet || bodyB->setIndex == b3_awakeSet )
	{
		setIndex = b3_awakeSet;
	}
	else
	{
		// Both bodies are asleep. A non-touching contact between sleeping
		// bodies is parked in the disabled set: it is not simulated, and if it
		// is later found to be touching, linking the islands moves it out.
		//
		// Reachable when a shape moves slightly and then falls asleep.
		setIndex = b3_disabledSet;
	}

	b3SolverSet* set = b3Array_Get( world->solverSets, setIndex );

	int contactId = b3AllocId( &world->contactIdPool );
	if ( contactId == world->contacts.count )
	{
		b3Contact emptyContact = { 0 };
		b3Array_Push( world->contacts, emptyContact );
	}

	int shapeIdA = shapeA->id;
	int shapeIdB = shapeB->id;

	b3Contact* contact = b3Array_Get( world->contacts, contactId );
	int generation = contact->generation;
	*contact = ( b3Contact ){ 0 };
	contact->contactId = contactId;
	contact->generation = generation + 1;
	contact->setIndex = setIndex;
	contact->colorIndex = B3_NULL_INDEX;
	contact->localIndex = set->contactIndices.count;
	contact->islandId = B3_NULL_INDEX;
	contact->islandIndex = B3_NULL_INDEX;
	contact->shapeIdA = shapeIdA;
	contact->shapeIdB = shapeIdB;
	contact->childIndex = childIndex;

	// Both bodies must opt in for the contact to be recyclable.
	if ( ( bodyA->flags & b3_bodyEnableContactRecycling ) != 0 && ( bodyB->flags & b3_bodyEnableContactRecycling ) != 0 )
	{
		contact->flags |= b3_contactRecycleFlag;
	}

	// Upstream also sets this for height field and mesh-child compound shapes.
	// B3_NEA_NO_HEIGHTFIELD / B3_NEA_NO_COMPOUND.
	//
	// The whole mesh half of the contact is taken here, before the world is
	// stepping: the triangle cache array and the manifold block both. Neither
	// is ever resized afterwards, so a body sliding across a level costs the
	// allocator nothing -- which is the guarantee
	// test_no_allocation_while_stepping exists to hold. The manifolds are taken
	// at the cap and kept even while the contact is not touching, so
	// manifoldCount can move 0..cap without an allocator round trip.
	if ( shapeA->type == b3_meshShape )
	{
		contact->flags |= b3_simMeshContact;

		contact->meshContact.triangleCache = (b3TriangleCache*)b3AllocateElement( &world->triangleCacheAllocator );
		contact->meshContact.triangleCount = 0;

		// Zeroed bounds contain nothing, so the first refresh always queries.
		contact->meshContact.queryBounds = ( b3AABB ){ b3Vec3_zero, b3Vec3_zero };

		contact->manifolds = b3AllocateManifolds( world, B3_NEA_MAX_MESH_MANIFOLDS );
		contact->manifoldCount = 0;
		contact->manifoldCapacity = B3_NEA_MAX_MESH_MANIFOLDS;
	}

	B3_ASSERT( shapeB->type == b3_sphereShape || shapeB->type == b3_capsuleShape || shapeB->type == b3_hullShape );

	if ( bodyA->type == b3_staticBody || bodyB->type == b3_staticBody )
	{
		contact->flags |= b3_contactStaticFlag;
	}

	// Unreachable rather than merely untrue: b3PairQueryCallback drops any pair
	// with a sensor in it before a contact is ever asked for, which is what
	// keeps the rest of this file, the contact graph and the solver free of
	// sensor cases.
	B3_ASSERT( shapeA->sensorIndex == B3_NULL_INDEX && shapeB->sensorIndex == B3_NULL_INDEX );

	if ( ( shapeA->flags & b3_enableContactEvents ) || ( shapeB->flags & b3_enableContactEvents ) )
	{
		contact->flags |= b3_contactEnableContactEvents;
	}

	// Note the && where the events above are ||: speculative points change the
	// contact's geometry, so both shapes must want them.
	if ( ( shapeA->flags & b3_enableSpeculative ) && ( shapeB->flags & b3_enableSpeculative ) )
	{
		contact->flags |= b3_enableSpeculativePoints;
	}

	if ( ( shapeA->flags & b3_enablePreSolveEvents ) || ( shapeB->flags & b3_enablePreSolveEvents ) )
	{
		contact->flags |= b3_simEnablePreSolveEvents;
	}

	// Splice into body A's contact list.
	{
		contact->edges[0].bodyId = shapeA->bodyId;
		contact->edges[0].prevKey = B3_NULL_INDEX;
		contact->edges[0].nextKey = bodyA->headContactKey;

		int keyA = ( contactId << 1 ) | 0;
		int headContactKey = bodyA->headContactKey;
		if ( headContactKey != B3_NULL_INDEX )
		{
			b3Contact* headContact = b3Array_Get( world->contacts, headContactKey >> 1 );
			headContact->edges[headContactKey & 1].prevKey = keyA;
		}
		bodyA->headContactKey = keyA;
		bodyA->contactCount += 1;
	}

	// Splice into body B's contact list.
	{
		contact->edges[1].bodyId = shapeB->bodyId;
		contact->edges[1].prevKey = B3_NULL_INDEX;
		contact->edges[1].nextKey = bodyB->headContactKey;

		int keyB = ( contactId << 1 ) | 1;
		int headContactKey = bodyB->headContactKey;
		if ( headContactKey != B3_NULL_INDEX )
		{
			b3Contact* headContact = b3Array_Get( world->contacts, headContactKey >> 1 );
			headContact->edges[headContactKey & 1].prevKey = keyB;
		}
		bodyB->headContactKey = keyB;
		bodyB->contactCount += 1;
	}

	// Register the pair so the broad phase does not report it again.
	b3ShapeKey pairKey = b3ShapePairKey( shapeIdA, shapeIdB, childIndex );
	b3AddKey( &world->broadPhase.pairSet, pairKey );

	// Contacts start non-touching. Finding them touching is what links the
	// islands and moves them into the constraint graph -- see b3Collide.
	b3Array_Push( set->contactIndices, contactId );

	// Rolling resistance is fixed for the life of the contact when neither
	// material has any, so it is computed once here and only recomputed in
	// b3UpdateConvexContact if a material actually asks for it.
	//
	// A hull contributes no radius here, where b3UpdateConvexContact gives it
	// 0.25 * innerRadius. That asymmetry is upstream's, and it does not matter
	// while the coefficient is zero -- which is the only case this value
	// survives to be used in.
	b3f radiusA = b3f_zero;
	if ( typeA == b3_sphereShape )
	{
		radiusA = shapeA->sphere.radius;
	}
	else if ( typeA == b3_capsuleShape )
	{
		radiusA = shapeA->capsule.radius;
	}

	b3f radiusB = b3f_zero;
	if ( typeB == b3_sphereShape )
	{
		radiusB = shapeB->sphere.radius;
	}
	else if ( typeB == b3_capsuleShape )
	{
		radiusB = shapeB->capsule.radius;
	}

	b3f maxRadius = b3MaxF( radiusA, radiusB );

	// Q30 coefficient times a Q12 length, landing at Q12: a length, per the
	// note on the field.
	b3c resistance = b3MaxC( b3GetShapeMaterials( shapeA )[0].rollingResistance, b3GetShapeMaterials( shapeB )[0].rollingResistance );
	contact->rollingResistance = b3MulFC( maxRadius, resistance );
}

void b3DestroyContact( b3World* world, b3Contact* contact, bool wakeBodies )
{
	// Remove the pair from the broad phase's set, so the pair can be found
	// again if the shapes overlap in a later step.
	b3ShapeKey pairKey = b3ShapePairKey( contact->shapeIdA, contact->shapeIdB, contact->childIndex );
	b3RemoveKey( &world->broadPhase.pairSet, pairKey );

	b3FreeManifolds( world, contact->manifolds, contact->manifoldCapacity );
	contact->manifolds = NULL;
	contact->manifoldCount = 0;
	contact->manifoldCapacity = 0;

	b3ContactEdge* edgeA = contact->edges + 0;
	b3ContactEdge* edgeB = contact->edges + 1;

	int bodyIdA = edgeA->bodyId;
	int bodyIdB = edgeB->bodyId;
	b3Body* bodyA = b3Array_Get( world->bodies, bodyIdA );
	b3Body* bodyB = b3Array_Get( world->bodies, bodyIdB );

	uint32_t flags = contact->flags;
	bool touching = ( flags & b3_contactTouchingFlag ) != 0;

	// A contact destroyed while touching still owes the application an end
	// touch event -- that is what the double-buffered end event arrays are for.
	if ( touching && ( flags & b3_contactEnableContactEvents ) != 0 )
	{
		uint16_t worldId = world->worldId;
		const b3Shape* shapeA = b3Array_Get( world->shapes, contact->shapeIdA );
		const b3Shape* shapeB = b3Array_Get( world->shapes, contact->shapeIdB );
		b3ShapeId shapeIdA = { shapeA->id + 1, worldId, shapeA->generation };
		b3ShapeId shapeIdB = { shapeB->id + 1, worldId, shapeB->generation };

		b3ContactId contactId = {
			.index1 = contact->contactId + 1,
			.world0 = world->worldId,
			.padding = 0,
			.generation = contact->generation,
		};

		b3ContactEndTouchEvent event = {
			.shapeIdA = shapeIdA,
			.shapeIdB = shapeIdB,
			.contactId = contactId,
		};

		b3Array_Push( world->contactEndEvents[world->endEventArrayIndex], event );
	}

	// Remove from body A's contact list.
	if ( edgeA->prevKey != B3_NULL_INDEX )
	{
		b3Contact* prevContact = b3Array_Get( world->contacts, edgeA->prevKey >> 1 );
		b3ContactEdge* prevEdge = prevContact->edges + ( edgeA->prevKey & 1 );
		prevEdge->nextKey = edgeA->nextKey;
	}

	if ( edgeA->nextKey != B3_NULL_INDEX )
	{
		b3Contact* nextContact = b3Array_Get( world->contacts, edgeA->nextKey >> 1 );
		b3ContactEdge* nextEdge = nextContact->edges + ( edgeA->nextKey & 1 );
		nextEdge->prevKey = edgeA->prevKey;
	}

	int contactId = contact->contactId;

	int edgeKeyA = ( contactId << 1 ) | 0;
	if ( bodyA->headContactKey == edgeKeyA )
	{
		bodyA->headContactKey = edgeA->nextKey;
	}

	bodyA->contactCount -= 1;

	// Remove from body B's contact list.
	if ( edgeB->prevKey != B3_NULL_INDEX )
	{
		b3Contact* prevContact = b3Array_Get( world->contacts, edgeB->prevKey >> 1 );
		b3ContactEdge* prevEdge = prevContact->edges + ( edgeB->prevKey & 1 );
		prevEdge->nextKey = edgeB->nextKey;
	}

	if ( edgeB->nextKey != B3_NULL_INDEX )
	{
		b3Contact* nextContact = b3Array_Get( world->contacts, edgeB->nextKey >> 1 );
		b3ContactEdge* nextEdge = nextContact->edges + ( edgeB->nextKey & 1 );
		nextEdge->prevKey = edgeB->prevKey;
	}

	int edgeKeyB = ( contactId << 1 ) | 1;
	if ( bodyB->headContactKey == edgeKeyB )
	{
		bodyB->headContactKey = edgeB->nextKey;
	}

	bodyB->contactCount -= 1;

	// The mesh half goes back to the allocator it came from. The flag is what
	// says the union holds a triangle cache pointer rather than a b3SATCache,
	// so it has to be tested and not merely assumed non-NULL.
	if ( ( contact->flags & b3_simMeshContact ) != 0 )
	{
		b3FreeElement( &world->triangleCacheAllocator, contact->meshContact.triangleCache );
		contact->meshContact.triangleCache = NULL;
		contact->meshContact.triangleCount = 0;
	}

	if ( contact->islandId != B3_NULL_INDEX )
	{
		b3UnlinkContact( world, contact );
	}

	// Remove the contact from whichever array owns it.
	if ( contact->colorIndex != B3_NULL_INDEX )
	{
		// An active constraint, so it lives in the constraint graph.
		B3_ASSERT( contact->setIndex == b3_awakeSet );
		b3RemoveContactFromGraph( world, bodyIdA, bodyIdB, contact->colorIndex, contact->localIndex );
	}
	else
	{
		// Non-touching, or sleeping: it lives in a solver set.
		B3_ASSERT( contact->setIndex != b3_awakeSet || ( contact->flags & b3_contactTouchingFlag ) == 0 );
		b3SolverSet* set = b3Array_Get( world->solverSets, contact->setIndex );

		int localIndex = contact->localIndex;
		int movedIndex = b3Array_RemoveSwap( set->contactIndices, localIndex );
		if ( movedIndex != B3_NULL_INDEX )
		{
			int movedContactIndex = set->contactIndices.data[localIndex];
			b3Contact* movedContact = b3Array_Get( world->contacts, movedContactIndex );
			movedContact->localIndex = localIndex;
		}
	}

	// Free the contact and its id, preserving the generation.
	contact->contactId = B3_NULL_INDEX;
	contact->setIndex = B3_NULL_INDEX;
	contact->colorIndex = B3_NULL_INDEX;
	contact->localIndex = B3_NULL_INDEX;
	b3FreeId( &world->contactIdPool, contactId );

	if ( wakeBodies && touching )
	{
		b3WakeBody( world, bodyA );
		b3WakeBody( world, bodyB );
	}
}

// =========================================================================
// The narrow phase
// =========================================================================

/// Build the manifold for one convex pair and carry the warm-start impulses
/// across from last step's points.
///
/// @return true when the shapes produced at least one point.
static bool b3ComputeConvexManifold( b3World* world, b3Contact* contact, const b3Shape* shapeA, b3WorldTransform xfA,
									 const b3Shape* shapeB, b3WorldTransform xfB, b3Arena arena )
{
	b3ShapeType typeA = shapeA->type;
	b3ShapeType typeB = shapeB->type;

	b3ContactCache* cache = &contact->cache;

	// The arena is taken by value, so this buffer is released by the bump
	// pointer restoring when this function returns.
	int pointCapacity = B3_MAX_CLIP_POINTS;
	b3LocalManifoldPoint* pointBuffer = (b3LocalManifoldPoint*)b3Bump( &arena, pointCapacity * (int)sizeof( b3LocalManifoldPoint ) );

	b3LocalManifold geomManifold = { 0 };
	geomManifold.points = pointBuffer;

	b3Transform transformBtoA = b3InvMulWorldTransforms( xfA, xfB );

	if ( typeA == b3_sphereShape )
	{
		B3_ASSERT( typeB == b3_sphereShape );
		b3CollideSpheres( &geomManifold, pointCapacity, &shapeA->sphere, &shapeB->sphere, transformBtoA );
	}
	else if ( typeA == b3_capsuleShape )
	{
		if ( typeB == b3_sphereShape )
		{
			b3CollideCapsuleAndSphere( &geomManifold, pointCapacity, &shapeA->capsule, &shapeB->sphere, transformBtoA );
		}
		else
		{
			B3_ASSERT( typeB == b3_capsuleShape );
			b3CollideCapsules( &geomManifold, pointCapacity, &shapeA->capsule, &shapeB->capsule, transformBtoA );
		}
	}
	else
	{
		B3_ASSERT( typeA == b3_hullShape );

		if ( typeB == b3_sphereShape )
		{
			b3CollideHullAndSphere( &geomManifold, pointCapacity, shapeA->hull, &shapeB->sphere, transformBtoA,
									&cache->simplexCache );
		}
		else if ( typeB == b3_capsuleShape )
		{
			b3CollideHullAndCapsule( &geomManifold, pointCapacity, shapeA->hull, &shapeB->capsule, transformBtoA,
									 &cache->simplexCache );
		}
		else
		{
			B3_ASSERT( typeB == b3_hullShape );
			b3CollideHulls( &geomManifold, pointCapacity, shapeA->hull, shapeB->hull, transformBtoA, &cache->satCache );

			// Upstream accumulates satCallCount / satCacheHitCount into the
			// worker's task context here. Those are b3Counters, dropped in 3A.
		}
	}

	if ( geomManifold.pointCount == 0 )
	{
		if ( contact->manifoldCount > 0 )
		{
			b3FreeManifolds( world, contact->manifolds, contact->manifoldCapacity );
			contact->manifolds = NULL;
			contact->manifoldCount = 0;
			contact->manifoldCapacity = 0;
		}

		return false;
	}

	b3ManifoldPoint oldPoints[B3_MAX_MANIFOLD_POINTS];
	int oldCount = 0;

	if ( contact->manifoldCount == 0 )
	{
		contact->manifolds = b3AllocateManifolds( world, 1 );
		contact->manifoldCount = 1;
		contact->manifoldCapacity = 1;
	}
	else
	{
		oldCount = contact->manifolds[0].pointCount;
		memcpy( oldPoints, contact->manifolds[0].points, oldCount * sizeof( b3ManifoldPoint ) );
	}

	b3Manifold* manifold = contact->manifolds;
	manifold->pointCount = geomManifold.pointCount;

	// Upstream rotates the normal through b3MakeMatrixFromQuat, which the port
	// cannot follow. That matrix is Q12 and is not exactly orthonormal, so a
	// unit normal comes back a raw unit or two off -- ~5e-4 relative, the same
	// error that made b3BuildFaceBContact renormalize in Phase 2B and forced
	// b3RotateInertiaW to Q30 in 3A. The contact normal is the axis the
	// solver's effective mass is formed about, so it is rotated from the
	// quaternion directly, which never narrows the orientation.
	manifold->normal = b3RotateVector( xfA.q, geomManifold.normal );

	// The anchors keep the matrix: they are positions, not directions, and are
	// rebuilt from geometry every step rather than fed back into themselves,
	// so their rounding does not accumulate (Phase 1).
	b3Matrix3 matrixA = b3MakeMatrixFromQuat( xfA.q );

	for ( int i = 0; i < geomManifold.pointCount; ++i )
	{
		const b3LocalManifoldPoint* source = geomManifold.points + i;
		b3ManifoldPoint* target = manifold->points + i;

		// Contact points are computed in frame A.
		target->anchorA = b3MulMV( matrixA, source->point );
		target->anchorB = b3Add( target->anchorA, b3SubPos( xfA.p, xfB.p ) );
		target->separation = source->separation;
		target->featureId = b3MakeFeatureId( source->pair );
		target->triangleIndex = B3_NULL_INDEX;
		target->normalVelocity = b3f_zero;
	}

	// Carry the warm-start impulse across from any point with a matching
	// feature id. This is the consumer that makes b3FlipPair's owner
	// complement load bearing (Phase 2B): a reference-face flip between steps
	// must not renumber a contact point, or the impulse is thrown away and a
	// resting stack jitters.
	for ( int i = 0; i < geomManifold.pointCount; ++i )
	{
		b3ManifoldPoint* pt2 = manifold->points + i;
		pt2->totalNormalImpulse = b3imp_zero;
		pt2->persisted = false;

		for ( int j = 0; j < oldCount; ++j )
		{
			b3ManifoldPoint* pt1 = oldPoints + j;

			if ( pt2->featureId == pt1->featureId )
			{
				pt2->normalImpulse = pt1->normalImpulse;
				pt2->persisted = true;

				// Claimed, so a second new point cannot match the same old one.
				pt1->featureId = UINT32_MAX;

				break;
			}
		}

		if ( pt2->persisted == false )
		{
			pt2->normalImpulse = b3imp_zero;
		}
	}

	return true;
}

static bool b3UpdateConvexContact( b3World* world, b3Contact* contact, b3Shape* shapeA, b3WorldTransform xfA, b3Shape* shapeB,
								   b3WorldTransform xfB, bool flip, b3Arena arena )
{
	bool touching = b3ComputeConvexManifold( world, contact, shapeA, xfA, shapeB, xfB, arena );

	if ( touching == false )
	{
		B3_ASSERT( contact->manifolds == NULL && contact->manifoldCount == 0 );
		return false;
	}

	// The convex invariant, and the reason the mesh cap does not reach here:
	// one manifold, allocated from size class 1, for as long as the pair is
	// touching. b3ComputeMeshManifolds is where a count above 1 comes from.
	B3_ASSERT( contact->manifoldCount == 1 && contact->manifoldCapacity == 1 );

	if ( flip )
	{
		// The feature ids are deliberately not flipped: they only have to
		// match across steps, and flipping is consistent.
		b3Manifold* manifold = contact->manifolds + 0;
		manifold->normal = b3Neg( manifold->normal );
		int pointCount = manifold->pointCount;
		for ( int i = 0; i < pointCount; ++i )
		{
			b3ManifoldPoint* mp = manifold->points + i;
			B3_SWAP( mp->anchorA, mp->anchorB );
		}
	}

	const b3SurfaceMaterial* materialA = b3GetShapeMaterials( shapeA );
	const b3SurfaceMaterial* materialB = b3GetShapeMaterials( shapeB );

	// Re-mixed every step so a material changed on the shape takes effect.
	contact->friction =
		world->frictionCallback( materialA->friction, materialA->userMaterialId, materialB->friction, materialB->userMaterialId );
	contact->restitution = world->restitutionCallback( materialA->restitution, materialA->userMaterialId, materialB->restitution,
													   materialB->userMaterialId );

	if ( b3Raw( materialA->rollingResistance ) > 0 || b3Raw( materialB->rollingResistance ) > 0 )
	{
		b3ShapeType typeA = shapeA->type;
		b3ShapeType typeB = shapeB->type;

		b3f radiusA = b3f_zero;
		if ( typeA == b3_sphereShape )
		{
			radiusA = shapeA->sphere.radius;
		}
		else if ( typeA == b3_capsuleShape )
		{
			radiusA = shapeA->capsule.radius;
		}
		else if ( typeA == b3_hullShape )
		{
			// 0.25 * innerRadius. A shift, not a multiply.
			radiusA = b3Makeb3f( b3Raw( shapeA->hull->innerRadius ) >> 2 );
		}

		b3f radiusB = b3f_zero;
		if ( typeB == b3_sphereShape )
		{
			radiusB = shapeB->sphere.radius;
		}
		else if ( typeB == b3_capsuleShape )
		{
			radiusB = shapeB->capsule.radius;
		}
		else if ( typeB == b3_hullShape )
		{
			radiusB = b3Makeb3f( b3Raw( shapeB->hull->innerRadius ) >> 2 );
		}

		b3f maxRadius = b3MaxF( radiusA, radiusB );
		b3c resistance = b3MaxC( materialA->rollingResistance, materialB->rollingResistance );
		contact->rollingResistance = b3MulFC( maxRadius, resistance );
	}
	else
	{
		contact->rollingResistance = b3f_zero;
	}

	b3Vec3 tangentVelocityA = b3RotateVector( xfA.q, materialA->tangentVelocity );
	b3Vec3 tangentVelocityB = b3RotateVector( xfB.q, materialB->tangentVelocity );
	contact->tangentVelocity = b3Sub( tangentVelocityA, tangentVelocityB );

	if ( world->preSolveFcn && ( contact->flags & b3_simEnablePreSolveEvents ) != 0 )
	{
		b3ShapeId shapeIdA = { shapeA->id + 1, world->worldId, shapeA->generation };
		b3ShapeId shapeIdB = { shapeB->id + 1, world->worldId, shapeB->generation };

		b3Pos point = b3OffsetPos( xfA.p, contact->manifolds[0].points[0].anchorA );
		b3Vec3 normal = contact->manifolds[0].normal;
		touching = world->preSolveFcn( shapeIdA, shapeIdB, point, normal, world->preSolveContext );
		if ( touching == false )
		{
			// The callback vetoed the contact.
			b3FreeManifolds( world, contact->manifolds, contact->manifoldCapacity );
			contact->manifolds = NULL;
			contact->manifoldCount = 0;
			contact->manifoldCapacity = 0;
			return false;
		}
	}

	if ( ( shapeA->flags & b3_enableHitEvents ) || ( shapeB->flags & b3_enableHitEvents ) )
	{
		contact->flags |= b3_simEnableHitEvent;
	}
	else
	{
		contact->flags &= ~b3_simEnableHitEvent;
	}

	return true;
}

bool b3UpdateContact( b3World* world, b3Contact* contact, b3Shape* shapeA, b3Vec3 localCenterA, b3WorldTransform xfA,
					  b3Shape* shapeB, b3Vec3 localCenterB, b3WorldTransform xfB, bool isFast, b3Arena arena )
{
	// Upstream branches four ways here: compound, mesh, height field, and
	// convex-vs-convex. Compound is B3_NEA_NO_COMPOUND and the height field is
	// B3_NEA_NO_HEIGHTFIELD, so two remain.
	B3_ASSERT( shapeA->type != b3_compoundShape && shapeB->type != b3_compoundShape );

	bool touching;

	// Level 3 opens here rather than at the top of the function: what it is
	// separating is the two branches below, and the dozen lines above them are
	// shared by both. Charging those to whichever branch happened to run would
	// blur exactly the line being drawn.
	B3_PROFILE_NARROW( &world->profileTimer, &world->profile.narrowPhaseOtherTicks );

	if ( shapeA->type == b3_meshShape )
	{
		// The registers put the mesh first, so there is no flipped mesh pair to
		// handle and `materialMap` -- upstream's compound-child remapping --
		// is not a parameter at all.
		touching = b3ComputeMeshManifolds( world, contact, shapeA, xfA, shapeB, xfB, isFast, arena );
		B3_PROFILE_NARROW( &world->profileTimer, &world->profile.meshManifoldTicks );

		// Unlike the convex path, which sets this once after the manifold is
		// built, the mesh path owns its own hit-event flag. Transliterated.
		if ( touching && ( ( shapeA->flags & b3_enableHitEvents ) || ( shapeB->flags & b3_enableHitEvents ) ) )
		{
			contact->flags |= b3_simEnableHitEvent;
		}
		else
		{
			contact->flags &= ~b3_simEnableHitEvent;
		}

		B3_ASSERT( ( touching == true && contact->manifoldCount > 0 ) || ( touching == false && contact->manifoldCount == 0 ) );
	}
	else
	{
		bool flip = false;
		touching = b3UpdateConvexContact( world, contact, shapeA, xfA, shapeB, xfB, flip, arena );
		B3_PROFILE_NARROW( &world->profileTimer, &world->profile.narrowPhaseOtherTicks );
	}

	// The re-anchoring below runs for both branches and is charged to the
	// convex field, which is a small deliberate inaccuracy: splitting it would
	// need a third field for work that is a handful of subtractions per
	// manifold point and belongs to neither branch.
	if ( touching )
	{
		b3Vec3 centerA = b3RotateVector( xfA.q, localCenterA );
		b3Vec3 centerB = b3RotateVector( xfB.q, localCenterB );

		// Re-anchor from the body origin to the centre of mass, which is the
		// frame the solver works in.
		for ( int i = 0; i < contact->manifoldCount; ++i )
		{
			b3Manifold* manifold = contact->manifolds + i;
			for ( int j = 0; j < manifold->pointCount; ++j )
			{
				b3ManifoldPoint* mp = manifold->points + j;
				mp->anchorA = b3Sub( mp->anchorA, centerA );
				mp->anchorB = b3Sub( mp->anchorB, centerB );
			}
		}

		contact->flags |= b3_simTouchingFlag;
	}
	else
	{
		contact->flags &= ~b3_simTouchingFlag;
	}

	return touching;
}
