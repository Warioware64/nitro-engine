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
/// on any phase and their cases are `default:` asserts; the mesh arrived in
/// Phase 5 and is collidable but not yet castable against -- b3RayCastShape,
/// b3ShapeCastShape, b3OverlapShape and b3MakeShapeProxy still reject it,
/// because the mesh query layer is Phase 7. b3ShapeType keeps all six
/// enumerators, because the contact register table in Phase 3B is indexed by
/// the pair and its shape must not shift between phases.
///
/// Also absent: b3Shape_ApplyWind and b3GetShapeProjectedArea (they exist for
/// b3World_Explode and wind, neither of which the port carries),
/// b3ShapeTimeOfImpact and b3CollideMover (Phase 7), the sensor half (Phase 7),
/// and b3Shape_SetName/GetName (B3_NEA_NO_NAMES).

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

	/// Always B3_NULL_INDEX until sensors arrive in Phase 7. The field stays so
	/// that b3DestroyShapeInternal keeps its shape.
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

// b3ComputeSweptShapeAABB needs b3GetSweepTransform and is called only from
// the continuous path in solver.c, so it arrives with Phase 7. b3GetShapeArea
// and b3GetShapeProjectedArea exist upstream for wind and b3World_Explode,
// neither of which the port carries.

b3ShapeProxy b3MakeShapeProxy( const b3Shape* shape );
b3ShapeProxy b3MakeLocalProxy( const b3ShapeProxy* proxy, b3Transform transform, b3Vec3* buffer );
b3AABB b3ComputeProxyAABB( const b3ShapeProxy* proxy );

b3CastOutput b3RayCastShape( const b3Shape* shape, b3Transform transform, const b3RayCastInput* input );
b3CastOutput b3ShapeCastShape( const b3Shape* shape, b3Transform transform, const b3ShapeCastInput* input );
bool b3OverlapShape( const b3Shape* shape, b3Transform transform, const b3ShapeProxy* proxy );

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
