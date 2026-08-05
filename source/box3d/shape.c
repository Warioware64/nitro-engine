// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Erin Catto        (original Box3D)
// Copyright (c) 2026 Warioware64       (Nitro Engine Advanced fixed-point port)
//
// This file is part of Nitro Engine Advanced

/// @file   shape.c
/// @brief  Shape lifetime, geometry dispatch and the broad-phase proxy.
///
/// See shape.h for what is absent and why. The dispatch switches keep every
/// b3ShapeType label with a comment on the unported ones rather than a
/// `default:` that swallows them, so adding mesh support in Phase 5 is a
/// compiler-guided exercise rather than a search.

#include "shape.h"

#include "body.h"
#include "broad_phase.h"
#include "contact.h"
#include "core.h"
#include "physics_world.h"

#include "box3d/collision.h"
#include "box3d/constants.h"

#include <string.h>

static b3Shape* b3GetShape( b3World* world, b3ShapeId shapeId )
{
	int id = shapeId.index1 - 1;
	b3Shape* shape = b3Array_Get( world->shapes, id );
	B3_ASSERT( shape->id == id && shape->generation == shapeId.generation );
	return shape;
}

/// How far to inflate a moving proxy's AABB beyond the shape.
///
/// A fraction of the shape's own size, capped. The fraction keeps a small
/// shape from carrying a margin larger than itself, and the cap keeps a large
/// one from flooding the tree with overlap.
static b3f b3ComputeShapeMargin( b3Shape* shape )
{
	b3f margin = b3f_zero;

	switch ( shape->type )
	{
		case b3_sphereShape:
		{
			margin = shape->sphere.radius;
		}
		break;

		case b3_capsuleShape:
		{
			b3f halfLength = b3MulFC( b3Distance( shape->capsule.center2, shape->capsule.center1 ), b3cFromFrac( 1, 2 ) );
			margin = b3AddF( halfLength, shape->capsule.radius );
		}
		break;

		case b3_hullShape:
		{
			// The squared distances stay wide. A hull vertex 80 units from the
			// centroid -- the practical ceiling on hull size, see b3HullData --
			// squares to 6400, which Q12 holds, but the sum of three such
			// components does not have to be assumed to fit when b3SqrtWide
			// takes the wide value directly.
			const b3HullData* hull = shape->hull;
			const b3Vec3* points = b3GetHullPoints( hull );
			int64_t maxExtentSqr = 0;
			int count = hull->vertexCount;
			for ( int i = 0; i < count; ++i )
			{
				int64_t distSqr = b3LengthSquaredWide( b3Sub( points[i], hull->center ) );
				if ( distSqr > maxExtentSqr )
				{
					maxExtentSqr = distSqr;
				}
			}
			margin = b3SqrtWide( maxExtentSqr );
		}
		break;

		case b3_meshShape:
		case b3_heightShape:
		case b3_compoundShape:
		{
			// Static-only shapes: the broad phase uses the speculative distance
			// for static proxies, so the per-shape margin is never consumed.
			// Return the cap so any incidental use is generous.
			return B3_MAX_AABB_MARGIN;
		}

		default:
			B3_ASSERT( false );
			return B3_MAX_AABB_MARGIN;
	}

	return b3MinF( B3_MAX_AABB_MARGIN, b3MulFC( margin, B3_AABB_MARGIN_FRACTION ) );
}

static void b3UpdateShapeAABBs( b3Shape* shape, b3WorldTransform transform, b3BodyType proxyType )
{
	// Compute a bounding box with a speculative margin.
	const b3f speculativeDistance = B3_SPECULATIVE_DISTANCE;
	const b3f aabbMargin = shape->aabbMargin;

	b3AABB aabb = b3ComputeFatShapeAABB( shape, transform, speculativeDistance );
	shape->aabb = aabb;

	// Smaller margin for static bodies. It cannot be zero because of the
	// time-of-impact tolerance.
	b3f margin = proxyType == b3_staticBody ? speculativeDistance : aabbMargin;
	b3AABB fatAABB;
	fatAABB.lowerBound.x = b3SubF( aabb.lowerBound.x, margin );
	fatAABB.lowerBound.y = b3SubF( aabb.lowerBound.y, margin );
	fatAABB.lowerBound.z = b3SubF( aabb.lowerBound.z, margin );
	fatAABB.upperBound.x = b3AddF( aabb.upperBound.x, margin );
	fatAABB.upperBound.y = b3AddF( aabb.upperBound.y, margin );
	fatAABB.upperBound.z = b3AddF( aabb.upperBound.z, margin );
	shape->fatAABB = fatAABB;
}

static b3Shape* b3CreateShapeInternal( b3World* world, b3Body* body, b3WorldTransform bodyTransform, const b3ShapeDef* def,
									   const void* geometry, b3ShapeType shapeType )
{
	int shapeId = b3AllocId( &world->shapeIdPool );

	if ( shapeId == world->shapes.count )
	{
		b3Array_Push( world->shapes, ( b3Shape ){ 0 } );
	}
	else
	{
		B3_ASSERT( world->shapes.data[shapeId].id == B3_NULL_INDEX );
	}

	b3Shape* shape = b3Array_Get( world->shapes, shapeId );

	switch ( shapeType )
	{
		case b3_capsuleShape:
			shape->capsule = *(const b3Capsule*)geometry;
			break;

		case b3_sphereShape:
			shape->sphere = *(const b3Sphere*)geometry;
			break;

		case b3_hullShape:
			// Upstream copies the hull into a reference-counted world database
			// keyed by content, and b3CreateTransformedHullShape bakes a fresh
			// one. Both exist so a hull built at run time has an owner. The
			// port has no run-time hull builder -- hulls are baked on the host
			// and live in ROM -- so the shape keeps the caller's pointer and
			// the whole database goes with them. The data must outlive the
			// shape; nothing here can check that.
			B3_ASSERT( geometry != NULL );
			shape->hull = (const b3HullData*)geometry;
			break;

		case b3_meshShape:
			// Same contract as the hull above, and for a stronger reason: there
			// is no device mesh builder at all, so the blob is always something
			// the caller baked offline. b3CreateMeshShape has already run
			// b3SafeScale over the scale.
			B3_ASSERT( geometry != NULL );
			shape->mesh = *(const b3Mesh*)geometry;
			break;

		default:
			// b3_heightShape and b3_compoundShape are not on any phase.
			// b3CreateShape rejects them before this point.
			B3_ASSERT( false );
			break;
	}

	shape->id = shapeId;
	shape->bodyId = body->id;
	shape->type = shapeType;
	shape->density = def->density;
	shape->filter = def->filter;
	shape->userData = def->userData;
	shape->flags = 0;
	shape->flags |= def->enableSensorEvents ? b3_enableSensorEvents : 0;
	shape->flags |= def->enableContactEvents ? b3_enableContactEvents : 0;
	shape->flags |= def->enableCustomFiltering ? b3_enableCustomFiltering : 0;
	shape->flags |= def->enableHitEvents ? b3_enableHitEvents : 0;
	shape->flags |= def->enablePreSolveEvents ? b3_enablePreSolveEvents : 0;
	shape->flags |= def->enableSpeculativeContact ? b3_enableSpeculative : 0;
	shape->proxyKey = B3_NULL_INDEX;
	shape->localCentroid = b3GetShapeCentroid( shape );
	shape->aabbMargin = b3ComputeShapeMargin( shape );
	shape->aabb = ( b3AABB ){ b3Vec3_zero, b3Vec3_zero };
	shape->fatAABB = ( b3AABB ){ b3Vec3_zero, b3Vec3_zero };
	shape->generation += 1;

	// One material, inline, no allocation. The multi-material branch upstream
	// serves meshes and compounds and arrives with Phase 5; b3GetShapeMaterials
	// already presents this one as a single-element array so that code will not
	// have to change here.
	shape->material = ( def->materialCount == 1 && def->materials != NULL ) ? def->materials[0] : def->baseMaterial;
	shape->materialCount = 1;
	shape->materials = NULL;

	// Sensors are Phase 7.
	shape->sensorIndex = B3_NULL_INDEX;

	if ( body->setIndex != b3_disabledSet )
	{
		b3BodyType proxyType = body->type;
		bool forcePairCreation = def->invokeContactCreation;
		b3CreateShapeProxy( shape, &world->broadPhase, proxyType, bodyTransform, forcePairCreation );
	}

	// Add to the body's doubly linked shape list.
	if ( body->headShapeId != B3_NULL_INDEX )
	{
		b3Shape* headShape = b3Array_Get( world->shapes, body->headShapeId );
		headShape->prevShapeId = shapeId;
	}

	shape->prevShapeId = B3_NULL_INDEX;
	shape->nextShapeId = body->headShapeId;
	body->headShapeId = shapeId;
	body->shapeCount += 1;

	b3ValidateSolverSets( world );

	return shape;
}

static b3ShapeId b3CreateShape( b3BodyId bodyId, const b3ShapeDef* def, const void* geometry, b3ShapeType shapeType )
{
	B3_CHECK_DEF( def );
	B3_ASSERT( b3Raw( def->density ) >= 0 );
	B3_ASSERT( b3Raw( def->baseMaterial.friction ) >= 0 );
	B3_ASSERT( b3Raw( def->baseMaterial.restitution ) >= 0 );

	b3World* world = b3GetUnlockedWorld( bodyId.world0 );
	if ( world == NULL )
	{
		return b3_nullShapeId;
	}

	if ( world->shapes.count == B3_MAX_SHAPES && world->shapeIdPool.freeArray.count == 0 )
	{
		B3_ASSERT( false );
		return b3_nullShapeId;
	}

	b3Body* body = b3GetBodyFullId( world, bodyId );

	world->locked = true;

	b3WorldTransform bodyTransform = b3GetBodyTransformQuick( world, body );

	b3Shape* shape = b3CreateShapeInternal( world, body, bodyTransform, def, geometry, shapeType );

	if ( shape == NULL )
	{
		world->locked = false;
		return b3_nullShapeId;
	}

	if ( def->updateBodyMass == true )
	{
		b3UpdateBodyMassData( world, body );
	}
	else if ( ( body->flags & b3_dirtyMass ) == 0 )
	{
		body->flags |= b3_dirtyMass;
		b3SyncBodyFlags( world, body );
	}

	b3ValidateSolverSets( world );

	b3ShapeId id = { shape->id + 1, bodyId.world0, shape->generation };

	world->locked = false;

	return id;
}

b3ShapeId b3CreateSphereShape( b3BodyId bodyId, const b3ShapeDef* def, const b3Sphere* sphere )
{
	return b3CreateShape( bodyId, def, sphere, b3_sphereShape );
}

b3ShapeId b3CreateCapsuleShape( b3BodyId bodyId, const b3ShapeDef* def, const b3Capsule* capsule )
{
	// A capsule shorter than the linear slop is a sphere, and the narrow phase
	// is much happier being told so than deriving a degenerate segment axis.
	// The comparison is wide because a squared slop is 400 raw units at Q12 and
	// the squared length it is compared against is a genuine length squared.
	int64_t lengthSqr = b3LengthSquaredWide( b3Sub( capsule->center1, capsule->center2 ) );
	int64_t slopSqr = (int64_t)b3Raw( B3_LINEAR_SLOP ) * b3Raw( B3_LINEAR_SLOP );

	if ( lengthSqr <= slopSqr )
	{
		b3Sphere sphere = { b3Lerp( capsule->center1, capsule->center2, b3cFromFrac( 1, 2 ) ), capsule->radius };
		return b3CreateShape( bodyId, def, &sphere, b3_sphereShape );
	}

	return b3CreateShape( bodyId, def, capsule, b3_capsuleShape );
}

b3ShapeId b3CreateHullShape( b3BodyId bodyId, const b3ShapeDef* def, const b3HullData* hull )
{
	B3_ASSERT( b3IsValidHull( hull ) );
	return b3CreateShape( bodyId, def, hull, b3_hullShape );
}

b3ShapeId b3CreateMeshShape( b3BodyId bodyId, const b3ShapeDef* def, const b3MeshData* mesh, b3Vec3 scale )
{
	// A baked blob is untrusted input -- it can come from NitroFS, and it can
	// be stale -- so the structural walk runs here rather than behind
	// B3_ENABLE_VALIDATION, the same rule hull.c:121 already states.
	B3_ASSERT( b3IsValidMesh( mesh ) );

	// A mesh shape has zero mass and zero inertia (b3ComputeShapeMass), so a
	// dynamic body carrying one and nothing else has no mass to solve with.
	// That is upstream's behaviour; assert it here where a caller can see it
	// rather than let the body silently fall through the world.
	B3_ASSERT( b3Body_GetType( bodyId ) != b3_dynamicBody );

	b3Mesh geometry = { mesh, b3SafeScale( scale ) };
	return b3CreateShape( bodyId, def, &geometry, b3_meshShape );
}

static void b3DestroyShapeInternal( b3World* world, b3Shape* shape, b3Body* body, bool wakeBodies )
{
	int shapeId = shape->id;

	// Remove the shape from the body's doubly linked list.
	if ( shape->prevShapeId != B3_NULL_INDEX )
	{
		b3Shape* prevShape = b3Array_Get( world->shapes, shape->prevShapeId );
		prevShape->nextShapeId = shape->nextShapeId;
	}

	if ( shape->nextShapeId != B3_NULL_INDEX )
	{
		b3Shape* nextShape = b3Array_Get( world->shapes, shape->nextShapeId );
		nextShape->prevShapeId = shape->prevShapeId;
	}

	if ( shapeId == body->headShapeId )
	{
		body->headShapeId = shape->nextShapeId;
	}

	body->shapeCount -= 1;

	// Remove from the broad phase.
	b3DestroyShapeProxy( shape, &world->broadPhase );

	// Destroy any contacts associated with the shape.
	int contactKey = body->headContactKey;
	while ( contactKey != B3_NULL_INDEX )
	{
		int contactId = contactKey >> 1;
		int edgeIndex = contactKey & 1;

		b3Contact* contact = b3Array_Get( world->contacts, contactId );
		contactKey = contact->edges[edgeIndex].nextKey;

		if ( contact->shapeIdA == shapeId || contact->shapeIdB == shapeId )
		{
			b3DestroyContact( world, contact, wakeBodies );
		}
	}

	// The sensor teardown upstream does here arrives with Phase 7.

	b3DestroyShapeAllocations( world, shape );

	// Return the shape to the free list.
	b3FreeId( &world->shapeIdPool, shapeId );
	shape->id = B3_NULL_INDEX;

	b3ValidateSolverSets( world );
}

void b3DestroyShape( b3ShapeId shapeId, bool updateBodyMass )
{
	b3World* world = b3GetUnlockedWorld( shapeId.world0 );
	if ( world == NULL )
	{
		return;
	}

	world->locked = true;

	b3Shape* shape = b3GetShape( world, shapeId );

	// Bodies must be woken because this might be a static shape that sleeping
	// bodies are resting on.
	bool wakeBodies = true;

	b3Body* body = b3Array_Get( world->bodies, shape->bodyId );
	b3DestroyShapeInternal( world, shape, body, wakeBodies );

	if ( updateBodyMass == true )
	{
		b3UpdateBodyMassData( world, body );
	}

	world->locked = false;
}

b3AABB b3ComputeShapeAABB( const b3Shape* shape, b3Transform transform )
{
	switch ( shape->type )
	{
		case b3_capsuleShape:
			return b3ComputeCapsuleAABB( &shape->capsule, transform );

		case b3_hullShape:
			return b3ComputeHullAABB( shape->hull, transform );

		case b3_sphereShape:
			return b3ComputeSphereAABB( &shape->sphere, transform );

		case b3_meshShape:
			// Reads the blob's baked local bounds and transforms them; it does
			// not walk the tree.
			return b3ComputeMeshAABB( shape->mesh.data, transform, shape->mesh.scale );

		default:
		{
			// height field and compound: not ported.
			B3_ASSERT( false );
			b3AABB empty = { transform.p, transform.p };
			return empty;
		}
	}
}

b3AABB b3ComputeFatShapeAABB( const b3Shape* shape, b3WorldTransform transform, b3f extra )
{
	b3Vec3 r = { extra, extra, extra };
	b3AABB aabb = b3ComputeShapeAABB( shape, transform );
	aabb.lowerBound = b3Sub( aabb.lowerBound, r );
	aabb.upperBound = b3Add( aabb.upperBound, r );
	return aabb;
}

b3Vec3 b3GetShapeCentroid( const b3Shape* shape )
{
	switch ( shape->type )
	{
		case b3_capsuleShape:
			return b3Lerp( shape->capsule.center1, shape->capsule.center2, b3cFromFrac( 1, 2 ) );

		case b3_sphereShape:
			return shape->sphere.center;

		case b3_hullShape:
			return shape->hull->center;

		case b3_meshShape:
			// The centre of the baked local bounds, scaled. Upstream returns
			// the origin for a mesh; this is strictly better for the shape
			// margin and cannot matter anywhere else, because a mesh has no
			// mass and its centroid never reaches the inertia integration.
			return b3Mul( b3Lerp( shape->mesh.data->bounds.lowerBound, shape->mesh.data->bounds.upperBound, b3cFromFrac( 1, 2 ) ),
						  shape->mesh.scale );

		default:
			return b3Vec3_zero;
	}
}

b3MassData b3ComputeShapeMass( const b3Shape* shape )
{
	switch ( shape->type )
	{
		case b3_capsuleShape:
			return b3ComputeCapsuleMass( &shape->capsule, shape->density );

		case b3_hullShape:
			return b3ComputeHullMass( shape->hull, shape->density );

		case b3_sphereShape:
			return b3ComputeSphereMass( &shape->sphere, shape->density );

		default:
			// A mesh contributes no mass and no inertia. That is upstream's
			// behaviour (shape.c:714-730 falls to the same default), not a port
			// limitation: a triangle soup has no volume to integrate. It is
			// what makes a mesh shape belong on a static or kinematic body, and
			// b3CreateMeshShape says so where a caller will read it.
			return ( b3MassData ){ 0 };
	}
}

b3ShapeExtent b3ComputeShapeExtent( const b3Shape* shape, b3Vec3 localCenter )
{
	b3ShapeExtent extent = { 0 };

	switch ( shape->type )
	{
		case b3_capsuleShape:
		{
			// Transliterated, including the missing absolute value on the two
			// end centres. Upstream's sphere and capsule cases measure the
			// signed offset from the centre of mass where the hull case takes
			// the magnitude, so a shape entirely on the negative side of the
			// centre reports a smaller maxExtent than it should. It is a real
			// upstream inconsistency, but maxExtent only feeds the sleeping
			// threshold and the speed cap, and "fixing" it here would put the
			// port permanently out of agreement with the reference for every
			// off-centre shape. Left as found, and recorded rather than
			// silently corrected.
			b3f radius = shape->capsule.radius;
			extent.minExtent = radius;
			b3Vec3 c1 = b3Sub( shape->capsule.center1, localCenter );
			b3Vec3 c2 = b3Sub( shape->capsule.center2, localCenter );
			b3Vec3 r = { radius, radius, radius };
			extent.maxExtent = b3Add( b3Max( c1, c2 ), r );
		}
		break;

		case b3_sphereShape:
		{
			// Same note as the capsule case above; here localCenter is
			// additionally subtracted twice, which upstream does too.
			b3f radius = shape->sphere.radius;
			extent.minExtent = radius;
			b3Vec3 r = { radius, radius, radius };
			b3Vec3 p = b3Add( b3Sub( shape->sphere.center, localCenter ), r );
			extent.maxExtent = b3Abs( b3Sub( p, localCenter ) );
		}
		break;

		case b3_hullShape:
			extent = b3ComputeHullExtent( shape->hull, localCenter );
			break;

		case b3_meshShape:
		{
			// A zeroed extent, deliberately. minExtent feeds the speed cap and
			// maxExtent the sleeping threshold, and both are read from the
			// *body's* extent -- which a mesh, being static or kinematic, never
			// contributes to, because b3UpdateBodyMassData skips a body with no
			// mass. Left zero rather than derived from the bounds so that a
			// kinematic mesh does not quietly acquire a speed cap the size of a
			// level.
		}
		break;

		default:
			break;
	}

	return extent;
}

uint64_t b3GetShapeUserMaterialId( const b3Shape* shape, int childIndex, int triangleIndex )
{
	// Every shape in this port has exactly one material, meshes included, so
	// neither index selects anything. They stay in the signature because the
	// contact code passes them through unconditionally, and because a
	// multi-material mesh is a blob feature the baker already emits
	// (b3GetMeshMaterialIndices) that only the API would have to grow to use.
	B3_UNUSED( childIndex, triangleIndex );
	return b3GetShapeMaterials( shape )[0].userMaterialId;
}

b3CastOutput b3RayCastShape( const b3Shape* shape, b3Transform transform, const b3RayCastInput* input )
{
	b3RayCastInput localInput = *input;
	localInput.origin = b3InvTransformPoint( transform, input->origin );
	localInput.translation = b3InvRotateVector( transform.q, input->translation );

	b3CastOutput output = { 0 };
	switch ( shape->type )
	{
		case b3_capsuleShape:
			output = b3RayCastCapsule( &shape->capsule, &localInput );
			break;
		case b3_sphereShape:
			output = b3RayCastSphere( &shape->sphere, &localInput );
			break;
		case b3_hullShape:
			output = b3RayCastHull( shape->hull, &localInput );
			break;
		default:
			return output;
	}

	output.point = b3TransformPoint( transform, output.point );
	output.normal = b3RotateVector( transform.q, output.normal );
	return output;
}

b3CastOutput b3ShapeCastShape( const b3Shape* shape, b3Transform transform, const b3ShapeCastInput* input )
{
	b3ShapeCastInput localInput = *input;
	b3Vec3 localPoints[B3_MAX_SHAPE_CAST_POINTS];

	localInput.proxy.count = b3MinInt( input->proxy.count, B3_MAX_SHAPE_CAST_POINTS );
	for ( int i = 0; i < localInput.proxy.count; ++i )
	{
		localPoints[i] = b3InvTransformPoint( transform, input->proxy.points[i] );
	}

	localInput.proxy.points = localPoints;
	localInput.translation = b3InvRotateVector( transform.q, input->translation );

	b3CastOutput output = { 0 };
	switch ( shape->type )
	{
		case b3_capsuleShape:
			output = b3ShapeCastCapsule( &shape->capsule, &localInput );
			break;
		case b3_hullShape:
			output = b3ShapeCastHull( shape->hull, &localInput );
			break;
		case b3_sphereShape:
			output = b3ShapeCastSphere( &shape->sphere, &localInput );
			break;
		default:
			return output;
	}

	output.point = b3TransformPoint( transform, output.point );
	output.normal = b3RotateVector( transform.q, output.normal );
	return output;
}

bool b3OverlapShape( const b3Shape* shape, b3Transform transform, const b3ShapeProxy* proxy )
{
	switch ( shape->type )
	{
		case b3_capsuleShape:
			return b3OverlapCapsule( &shape->capsule, transform, proxy );

		case b3_hullShape:
			return b3OverlapHull( shape->hull, transform, proxy );

		case b3_sphereShape:
			return b3OverlapSphere( &shape->sphere, transform, proxy );

		default:
			B3_ASSERT( false );
			return false;
	}
}

void b3CreateShapeProxy( b3Shape* shape, b3BroadPhase* bp, b3BodyType type, b3WorldTransform transform, bool forcePairCreation )
{
	B3_ASSERT( shape->proxyKey == B3_NULL_INDEX );

	b3UpdateShapeAABBs( shape, transform, type );

	shape->proxyKey =
		b3BroadPhase_CreateProxy( bp, type, shape->fatAABB, shape->filter.categoryBits, shape->id, forcePairCreation );
	B3_ASSERT( B3_PROXY_TYPE( shape->proxyKey ) < b3_bodyTypeCount );
}

void b3DestroyShapeProxy( b3Shape* shape, b3BroadPhase* bp )
{
	if ( shape->proxyKey != B3_NULL_INDEX )
	{
		b3BroadPhase_DestroyProxy( bp, shape->proxyKey );
		shape->proxyKey = B3_NULL_INDEX;
	}
}

void b3DestroyShapeAllocations( b3World* world, b3Shape* shape )
{
	B3_UNUSED( world );

	// Upstream also releases the hull database reference and the user debug
	// shape here. The port has neither.
	if ( shape->materials != NULL )
	{
		B3_ASSERT( shape->materialCount > 0 );
		b3Free( shape->materials, shape->materialCount * (int)sizeof( b3SurfaceMaterial ) );
		shape->materials = NULL;
		shape->materialCount = 0;
	}
}

b3ShapeProxy b3MakeShapeProxy( const b3Shape* shape )
{
	switch ( shape->type )
	{
		case b3_capsuleShape:
			return ( b3ShapeProxy ){ &shape->capsule.center1, 2, shape->capsule.radius };

		case b3_sphereShape:
			return ( b3ShapeProxy ){ &shape->sphere.center, 1, shape->sphere.radius };

		case b3_hullShape:
		{
			const b3HullData* hull = shape->hull;
			const b3Vec3* points = b3GetHullPoints( hull );
			return ( b3ShapeProxy ){ points, hull->vertexCount, b3f_zero };
		}

		default:
		{
			B3_ASSERT( false );
			return ( b3ShapeProxy ){ 0 };
		}
	}
}

b3ShapeProxy b3MakeLocalProxy( const b3ShapeProxy* proxy, b3Transform transform, b3Vec3* buffer )
{
	b3Transform invTransform = b3InvertTransform( transform );

	int count = b3MinInt( proxy->count, B3_MAX_SHAPE_CAST_POINTS );
	for ( int i = 0; i < count; ++i )
	{
		// Upstream builds a rotation matrix once and multiplies. Rotating by
		// the quaternion directly is both cheaper here (counts are 1, 2 or a
		// hull vertex count) and more accurate, since b3MakeMatrixFromQuat
		// narrows the Q30 orientation to Q12.
		buffer[i] = b3Add( b3RotateVector( invTransform.q, proxy->points[i] ), invTransform.p );
	}

	return ( b3ShapeProxy ){
		.points = buffer,
		.count = count,
		.radius = proxy->radius,
	};
}

b3AABB b3ComputeProxyAABB( const b3ShapeProxy* proxy )
{
	const b3Vec3* points = proxy->points;
	b3AABB aabb = {
		.lowerBound = points[0],
		.upperBound = points[0],
	};

	for ( int i = 1; i < proxy->count; ++i )
	{
		aabb.lowerBound = b3Min( aabb.lowerBound, points[i] );
		aabb.upperBound = b3Max( aabb.upperBound, points[i] );
	}

	b3Vec3 r = { proxy->radius, proxy->radius, proxy->radius };
	aabb.lowerBound = b3Sub( aabb.lowerBound, r );
	aabb.upperBound = b3Add( aabb.upperBound, r );

	return aabb;
}

// =========================================================================
// Public API
// =========================================================================

b3BodyId b3Shape_GetBody( b3ShapeId shapeId )
{
	b3World* world = b3GetWorld( shapeId.world0 );
	b3Shape* shape = b3GetShape( world, shapeId );
	return b3MakeBodyId( world, shape->bodyId );
}

b3WorldId b3Shape_GetWorld( b3ShapeId shapeId )
{
	b3World* world = b3GetWorld( shapeId.world0 );
	return ( b3WorldId ){ shapeId.world0 + 1, world->generation };
}

void b3Shape_SetUserData( b3ShapeId shapeId, void* userData )
{
	b3World* world = b3GetWorld( shapeId.world0 );
	b3Shape* shape = b3GetShape( world, shapeId );
	shape->userData = userData;
}

void* b3Shape_GetUserData( b3ShapeId shapeId )
{
	b3World* world = b3GetWorld( shapeId.world0 );
	b3Shape* shape = b3GetShape( world, shapeId );
	return shape->userData;
}

bool b3Shape_IsSensor( b3ShapeId shapeId )
{
	b3World* world = b3GetWorld( shapeId.world0 );
	b3Shape* shape = b3GetShape( world, shapeId );
	return shape->sensorIndex != B3_NULL_INDEX;
}

void b3Shape_SetDensity( b3ShapeId shapeId, b3f density, bool updateBodyMass )
{
	B3_ASSERT( b3Raw( density ) >= 0 );

	b3World* world = b3GetUnlockedWorld( shapeId.world0 );
	if ( world == NULL )
	{
		return;
	}

	b3Shape* shape = b3GetShape( world, shapeId );
	if ( b3Raw( density ) == b3Raw( shape->density ) )
	{
		// Early out: recomputing mass walks every shape on the body.
		return;
	}

	shape->density = density;

	if ( updateBodyMass == true )
	{
		b3Body* body = b3Array_Get( world->bodies, shape->bodyId );
		b3UpdateBodyMassData( world, body );
	}
}

b3f b3Shape_GetDensity( b3ShapeId shapeId )
{
	b3World* world = b3GetWorld( shapeId.world0 );
	b3Shape* shape = b3GetShape( world, shapeId );
	return shape->density;
}

void b3Shape_SetFriction( b3ShapeId shapeId, b3c friction )
{
	B3_ASSERT( b3Raw( friction ) >= 0 );
	b3World* world = b3GetWorld( shapeId.world0 );
	b3Shape* shape = b3GetShape( world, shapeId );
	b3GetShapeMaterials( shape )[0].friction = friction;
}

b3c b3Shape_GetFriction( b3ShapeId shapeId )
{
	b3World* world = b3GetWorld( shapeId.world0 );
	b3Shape* shape = b3GetShape( world, shapeId );
	return b3GetShapeMaterials( shape )[0].friction;
}

void b3Shape_SetRestitution( b3ShapeId shapeId, b3c restitution )
{
	B3_ASSERT( b3Raw( restitution ) >= 0 );
	b3World* world = b3GetWorld( shapeId.world0 );
	b3Shape* shape = b3GetShape( world, shapeId );
	b3GetShapeMaterials( shape )[0].restitution = restitution;
}

b3c b3Shape_GetRestitution( b3ShapeId shapeId )
{
	b3World* world = b3GetWorld( shapeId.world0 );
	b3Shape* shape = b3GetShape( world, shapeId );
	return b3GetShapeMaterials( shape )[0].restitution;
}

void b3Shape_SetSurfaceMaterial( b3ShapeId shapeId, b3SurfaceMaterial surfaceMaterial )
{
	b3World* world = b3GetWorld( shapeId.world0 );
	b3Shape* shape = b3GetShape( world, shapeId );
	b3GetShapeMaterials( shape )[0] = surfaceMaterial;
}

b3SurfaceMaterial b3Shape_GetSurfaceMaterial( b3ShapeId shapeId )
{
	b3World* world = b3GetWorld( shapeId.world0 );
	b3Shape* shape = b3GetShape( world, shapeId );
	return b3GetShapeMaterials( shape )[0];
}

b3Filter b3Shape_GetFilter( b3ShapeId shapeId )
{
	b3World* world = b3GetWorld( shapeId.world0 );
	b3Shape* shape = b3GetShape( world, shapeId );
	return shape->filter;
}

/// Rebuild a shape's contacts and proxy after something that invalidates them.
static void b3ResetProxy( b3World* world, b3Shape* shape, bool wakeBodies, bool destroyProxy )
{
	b3Body* body = b3Array_Get( world->bodies, shape->bodyId );

	int shapeId = shape->id;

	// Destroy all contacts associated with this shape.
	int contactKey = body->headContactKey;
	while ( contactKey != B3_NULL_INDEX )
	{
		int contactId = contactKey >> 1;
		int edgeIndex = contactKey & 1;

		b3Contact* contact = b3Array_Get( world->contacts, contactId );
		contactKey = contact->edges[edgeIndex].nextKey;

		if ( contact->shapeIdA == shapeId || contact->shapeIdB == shapeId )
		{
			b3DestroyContact( world, contact, wakeBodies );
		}
	}

	b3WorldTransform transform = b3GetBodyTransformQuick( world, body );
	if ( shape->proxyKey != B3_NULL_INDEX )
	{
		b3BodyType proxyType = B3_PROXY_TYPE( shape->proxyKey );
		b3UpdateShapeAABBs( shape, transform, proxyType );

		if ( destroyProxy )
		{
			b3BroadPhase_DestroyProxy( &world->broadPhase, shape->proxyKey );

			bool forcePairCreation = true;
			shape->proxyKey = b3BroadPhase_CreateProxy( &world->broadPhase, proxyType, shape->fatAABB,
														shape->filter.categoryBits, shapeId, forcePairCreation );
		}
		else
		{
			b3BroadPhase_MoveProxy( &world->broadPhase, shape->proxyKey, shape->fatAABB );
		}
	}
	else
	{
		b3BodyType proxyType = body->type;
		b3UpdateShapeAABBs( shape, transform, proxyType );
	}

	b3ValidateSolverSets( world );
}

void b3Shape_SetFilter( b3ShapeId shapeId, b3Filter filter, bool invokeContacts )
{
	b3World* world = b3GetUnlockedWorld( shapeId.world0 );
	if ( world == NULL )
	{
		return;
	}

	b3Shape* shape = b3GetShape( world, shapeId );
	if ( filter.maskBits == shape->filter.maskBits && filter.categoryBits == shape->filter.categoryBits &&
		 filter.groupIndex == shape->filter.groupIndex )
	{
		return;
	}

	// The category comparison has to happen before the assignment: it decides
	// whether the proxy must be rebuilt rather than moved, because the tree
	// sorts on category bits.
	bool destroyProxy = filter.categoryBits != shape->filter.categoryBits;

	shape->filter = filter;

	if ( invokeContacts )
	{
		world->locked = true;

		// Bodies must be woken because a filter change can destroy contacts.
		bool wakeBodies = true;
		b3ResetProxy( world, shape, wakeBodies, destroyProxy );
		world->locked = false;
	}
}

void b3Shape_EnableContactEvents( b3ShapeId shapeId, bool flag )
{
	b3World* world = b3GetUnlockedWorld( shapeId.world0 );
	if ( world == NULL )
	{
		return;
	}

	b3Shape* shape = b3GetShape( world, shapeId );
	if ( flag )
	{
		shape->flags |= b3_enableContactEvents;
	}
	else
	{
		shape->flags &= ~b3_enableContactEvents;
	}
}

bool b3Shape_AreContactEventsEnabled( b3ShapeId shapeId )
{
	b3World* world = b3GetWorld( shapeId.world0 );
	b3Shape* shape = b3GetShape( world, shapeId );
	return ( shape->flags & b3_enableContactEvents ) != 0;
}

void b3Shape_EnableHitEvents( b3ShapeId shapeId, bool flag )
{
	b3World* world = b3GetUnlockedWorld( shapeId.world0 );
	if ( world == NULL )
	{
		return;
	}

	b3Shape* shape = b3GetShape( world, shapeId );
	if ( flag )
	{
		shape->flags |= b3_enableHitEvents;
	}
	else
	{
		shape->flags &= ~b3_enableHitEvents;
	}
}

bool b3Shape_AreHitEventsEnabled( b3ShapeId shapeId )
{
	b3World* world = b3GetWorld( shapeId.world0 );
	b3Shape* shape = b3GetShape( world, shapeId );
	return ( shape->flags & b3_enableHitEvents ) != 0;
}

void b3Shape_EnablePreSolveEvents( b3ShapeId shapeId, bool flag )
{
	b3World* world = b3GetUnlockedWorld( shapeId.world0 );
	if ( world == NULL )
	{
		return;
	}

	b3Shape* shape = b3GetShape( world, shapeId );
	if ( flag )
	{
		shape->flags |= b3_enablePreSolveEvents;
	}
	else
	{
		shape->flags &= ~b3_enablePreSolveEvents;
	}
}

bool b3Shape_ArePreSolveEventsEnabled( b3ShapeId shapeId )
{
	b3World* world = b3GetWorld( shapeId.world0 );
	b3Shape* shape = b3GetShape( world, shapeId );
	return ( shape->flags & b3_enablePreSolveEvents ) != 0;
}

b3ShapeType b3Shape_GetType( b3ShapeId shapeId )
{
	b3World* world = b3GetWorld( shapeId.world0 );
	b3Shape* shape = b3GetShape( world, shapeId );
	return shape->type;
}

b3Sphere b3Shape_GetSphere( b3ShapeId shapeId )
{
	b3World* world = b3GetWorld( shapeId.world0 );
	b3Shape* shape = b3GetShape( world, shapeId );
	B3_ASSERT( shape->type == b3_sphereShape );
	return shape->sphere;
}

b3Capsule b3Shape_GetCapsule( b3ShapeId shapeId )
{
	b3World* world = b3GetWorld( shapeId.world0 );
	b3Shape* shape = b3GetShape( world, shapeId );
	B3_ASSERT( shape->type == b3_capsuleShape );
	return shape->capsule;
}

const b3HullData* b3Shape_GetHull( b3ShapeId shapeId )
{
	b3World* world = b3GetWorld( shapeId.world0 );
	b3Shape* shape = b3GetShape( world, shapeId );
	B3_ASSERT( shape->type == b3_hullShape );
	return shape->hull;
}

/// Replace a shape's geometry in place.
///
/// The common tail of the three setters below: the geometry has already been
/// written, so everything derived from it is stale and every contact the shape
/// had was computed against the old geometry.
static void b3ShapeGeometryChanged( b3World* world, b3Shape* shape )
{
	shape->localCentroid = b3GetShapeCentroid( shape );
	shape->aabbMargin = b3ComputeShapeMargin( shape );

	bool wakeBodies = true;
	bool destroyProxy = false;
	b3ResetProxy( world, shape, wakeBodies, destroyProxy );

	b3Body* body = b3Array_Get( world->bodies, shape->bodyId );
	b3UpdateBodyMassData( world, body );
}

void b3Shape_SetSphere( b3ShapeId shapeId, const b3Sphere* sphere )
{
	b3World* world = b3GetUnlockedWorld( shapeId.world0 );
	if ( world == NULL )
	{
		return;
	}

	b3Shape* shape = b3GetShape( world, shapeId );
	b3DestroyShapeAllocations( world, shape );

	shape->sphere = *sphere;
	shape->type = b3_sphereShape;
	b3ShapeGeometryChanged( world, shape );
}

void b3Shape_SetCapsule( b3ShapeId shapeId, const b3Capsule* capsule )
{
	b3World* world = b3GetUnlockedWorld( shapeId.world0 );
	if ( world == NULL )
	{
		return;
	}

	b3Shape* shape = b3GetShape( world, shapeId );
	b3DestroyShapeAllocations( world, shape );

	shape->capsule = *capsule;
	shape->type = b3_capsuleShape;
	b3ShapeGeometryChanged( world, shape );
}

void b3Shape_SetHull( b3ShapeId shapeId, const b3HullData* hull )
{
	B3_ASSERT( b3IsValidHull( hull ) );

	b3World* world = b3GetUnlockedWorld( shapeId.world0 );
	if ( world == NULL )
	{
		return;
	}

	b3Shape* shape = b3GetShape( world, shapeId );
	b3DestroyShapeAllocations( world, shape );

	shape->hull = hull;
	shape->type = b3_hullShape;
	b3ShapeGeometryChanged( world, shape );
}

b3AABB b3Shape_GetAABB( b3ShapeId shapeId )
{
	b3World* world = b3GetWorld( shapeId.world0 );
	if ( world == NULL )
	{
		return ( b3AABB ){ 0 };
	}

	b3Shape* shape = b3GetShape( world, shapeId );
	return shape->aabb;
}

b3MassData b3Shape_ComputeMassData( b3ShapeId shapeId )
{
	b3World* world = b3GetWorld( shapeId.world0 );
	if ( world == NULL )
	{
		return ( b3MassData ){ 0 };
	}

	b3Shape* shape = b3GetShape( world, shapeId );
	return b3ComputeShapeMass( shape );
}
