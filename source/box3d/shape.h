// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Erin Catto        (original Box3D)
// Copyright (c) 2026 Warioware64       (Nitro Engine Advanced fixed-point port)
//
// This file is part of Nitro Engine Advanced

#pragma once

/// @file   shape.h
/// @brief  A collider attached to a body: geometry, material, filter, proxy.
///
/// @section scope Four of six geometry types
///
/// Upstream's shape.c dispatches over six. Height field and compound are not
/// on any phase and their cases are `default:` asserts. The mesh arrived in
/// Phase 5 as collidable, and Phase 7's query layer made it castable: it now
/// has a case in b3RayCastShape, b3ShapeCastShape and b3OverlapShape.
/// b3MakeShapeProxy still rejects it, and must -- a proxy is a convex point
/// cloud and a mesh is not convex, which is why the mesh queries are their own
/// traversals rather than a proxy handed to the shared code. b3ShapeType keeps
/// all six enumerators, because the contact register table in Phase 3B is
/// indexed by the pair and its shape must not shift between phases.
///
/// Phase 7 Stage 2 added b3ShapeTimeOfImpact, which dispatches over the same
/// types again -- and for the mesh, drives b3QueryMesh with a three-point proxy
/// per triangle rather than reaching for b3MakeShapeProxy.
///
/// Phase 7 Stage 4 added b3CollideMover, the fifth dispatch over those types
/// and the only one that can return more than one result per shape.
///
/// Also absent: b3Shape_ApplyWind and b3GetShapeProjectedArea (they exist for
/// b3World_Explode and wind, neither of which the port carries), and
/// b3Shape_SetName/GetName (B3_NEA_NO_NAMES). The sensor half arrived in
/// Phase 7 Stage 3: creation and teardown are hooked here, the pass itself is
/// sensor.c.

#include "box3d/box3d.h"
#include "box3d/collision.h"
#include "box3d/id.h"
#include "box3d/types.h"

#include <stdbool.h>

typedef struct b3BroadPhase b3BroadPhase;
typedef struct b3World b3World;

typedef enum b3ShapeFlags
{
	b3_enableSensorEvents = 0x01,
	b3_enableContactEvents = 0x02,
	b3_enableCustomFiltering = 0x04,
	b3_enableHitEvents = 0x08,
	b3_enablePreSolveEvents = 0x10,
	b3_enlargedAABB = 0x20,
	b3_enableSpeculative = 0x40,
} b3ShapeFlags;

typedef struct b3Shape
{
	int id;
	int bodyId;
	int prevShapeId;
	int nextShapeId;

	/// Index into b3World::sensors, or B3_NULL_INDEX for an ordinary shape.
	///
	/// This is the only thing that makes a shape a sensor, and every "is this a
	/// sensor" test in the library is a compare of this against B3_NULL_INDEX.
	/// It is not stable: b3DestroySensor fills the hole by swapping the last
	/// sensor down and rewriting that shape's copy of this field.
	int sensorIndex;

	int proxyKey;
	b3ShapeType type;
	b3f density;

	/// Half the fat-AABB inflation for a moving proxy, derived from the shape's
	/// own size by B3_AABB_MARGIN_FRACTION and capped at B3_MAX_AABB_MARGIN.
	b3f aabbMargin;

	b3AABB aabb;
	b3AABB fatAABB;
	b3Vec3 localCentroid;

	int materialCount;
	b3SurfaceMaterial material;

	/// Non-NULL only for a multi-material mesh (Phase 5). A single-material
	/// shape presents the inline `material` as a one-element array through
	/// b3GetShapeMaterials, so both cases read the same way.
	b3SurfaceMaterial* materials;

	b3Filter filter;
	void* userData;

	uint16_t generation;

	/// b3ShapeFlags
	uint8_t flags;

	union
	{
		b3Capsule capsule;
		b3Sphere sphere;

		/// Not owned. Hulls are baked on the host and live in ROM, so the shape
		/// keeps the caller's pointer and upstream's reference-counted world
		/// hull database is gone. The data must outlive every shape using it.
		const b3HullData* hull;

		/// Not owned either, and for the same reason: there is no device mesh
		/// builder at all, so a b3MeshData is always a baked blob the caller
		/// supplies -- from ROM, from NitroFS, or from a static in the test
		/// suite. It must outlive every shape referencing it. The scale lives
		/// here rather than in the blob so one blob can back several shapes.
		b3Mesh mesh;
	};

} b3Shape;

/// Reach a shape's materials the same way whether it has one or many. Do not
/// cache the pointer: the shapes array can move.
static inline b3SurfaceMaterial* b3GetShapeMaterials( const b3Shape* shape )
{
	return shape->materials != NULL ? shape->materials : (b3SurfaceMaterial*)&shape->material;
}

void b3CreateShapeProxy( b3Shape* shape, b3BroadPhase* bp, b3BodyType type, b3WorldTransform transform, bool forcePairCreation );
void b3DestroyShapeProxy( b3Shape* shape, b3BroadPhase* bp );

void b3DestroyShapeAllocations( b3World* world, b3Shape* shape );

b3MassData b3ComputeShapeMass( const b3Shape* shape );
b3ShapeExtent b3ComputeShapeExtent( const b3Shape* shape, b3Vec3 localCenter );

b3AABB b3ComputeShapeAABB( const b3Shape* shape, b3Transform transform );

/// Conservative world AABB for a shape, inflated by `extra` on every axis.
b3AABB b3ComputeFatShapeAABB( const b3Shape* shape, b3WorldTransform transform, b3f extra );
b3Vec3 b3GetShapeCentroid( const b3Shape* shape );
uint64_t b3GetShapeUserMaterialId( const b3Shape* shape, int childIndex, int triangleIndex );

/// The bounds of a shape over a whole sweep, up to `time`.
///
/// Asserts on a mesh: a mesh never sweeps, it is what gets swept against.
b3AABB b3ComputeSweptShapeAABB( const b3Shape* shape, const b3Sweep* sweep, b3c time );

/// The first time in [0, maxFraction] at which two swept shapes touch.
///
/// Shape A may be a mesh, in which case the sweep runs against each triangle
/// the swept bounds reach; shape B may not, and the continuous path never asks
/// it to. b3GetShapeArea and b3GetShapeProjectedArea exist upstream for wind
/// and b3World_Explode, neither of which the port carries.
b3TOIOutput b3ShapeTimeOfImpact( const b3Shape* shapeA, const b3Shape* shapeB, const b3Sweep* sweepA, const b3Sweep* sweepB,
								 b3c maxFraction );

b3ShapeProxy b3MakeShapeProxy( const b3Shape* shape );

// b3MakeLocalProxy and b3ComputeProxyAABB moved to distance.c in Phase 7 and
// are declared in collision.h -- neither touches a b3Shape, and keeping them
// here made the mesh query layer reach into shape.c for them. See the note
// above their definitions.

b3CastOutput b3RayCastShape( const b3Shape* shape, b3Transform transform, const b3RayCastInput* input );
b3CastOutput b3ShapeCastShape( const b3Shape* shape, b3Transform transform, const b3ShapeCastInput* input );
bool b3OverlapShape( const b3Shape* shape, b3Transform transform, const b3ShapeProxy* proxy );

/// Collision planes between a character mover and one shape, in world space.
///
/// Dispatches over the same four types as the three above and post-transforms
/// the result -- rotating each normal and transforming each point, but
/// **leaving each offset alone**, because a mover plane's offset is a
/// penetration depth rather than a dot( normal, point ). See types.h.
///
/// Only the mesh case can return more than one plane, so `planeCapacity` binds
/// there and nowhere else.
int b3CollideMover( b3PlaneResult* planes, int planeCapacity, const b3Shape* shape, b3Transform transform,
					const b3Capsule* mover );

// The public API -- b3CreateBody, b3Body_*, b3CreateWorld, b3World_*,
// b3Create*Shape, b3Shape_*, b3Contact_GetData -- is declared in
// include/box3d/box3d.h, which is the header a game installs and includes.
// This file keeps only what the port's own translation units need.

/// Filtering test shared by the broad phase and the contact update.
static inline bool b3ShouldShapesCollide( b3Filter filterA, b3Filter filterB )
{
	if ( filterA.groupIndex == filterB.groupIndex && filterA.groupIndex != 0 )
	{
		return filterA.groupIndex > 0;
	}

	return ( filterA.maskBits & filterB.categoryBits ) != 0 && ( filterA.categoryBits & filterB.maskBits ) != 0;
}

/// The same test between a shape and a *query*, for Phase 7's world queries.
///
/// Not b3ShouldShapesCollide with a synthetic b3Filter: a b3QueryFilter has no
/// groupIndex, and the group override is the one rule that would then apply by
/// accident. A query asks about category and mask only.
static inline bool b3ShouldQueryCollide( const b3Filter* shapeFilter, const b3QueryFilter* queryFilter )
{
	return ( shapeFilter->categoryBits & queryFilter->maskBits ) != 0 &&
		   ( shapeFilter->maskBits & queryFilter->categoryBits ) != 0;
}
