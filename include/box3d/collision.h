// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Erin Catto        (original Box3D)
// Copyright (c) 2026 Warioware64       (Nitro Engine Advanced fixed-point port)
//
// This file is part of Nitro Engine Advanced

#pragma once

/// @file   collision.h
/// @brief  Collision function declarations.
///
/// The types live in types.h; this is the function surface over them. Like
/// types.h it covers the collision tier only -- the mesh entry points arrive
/// with Phase 5.

#include "base.h"
#include "math_fixed.h"
#include "types.h"

#include <stddef.h>

// =========================================================================
// Sphere
// =========================================================================

/// Compute mass properties. The returned inertia is **per unit mass** --
/// see the note on b3MassData in types.h.
B3_API b3MassData b3ComputeSphereMass( const b3Sphere* shape, b3f density );

B3_API b3AABB b3ComputeSphereAABB( const b3Sphere* shape, b3Transform transform );
B3_API b3AABB b3ComputeSweptSphereAABB( const b3Sphere* shape, b3Transform xf1, b3Transform xf2 );

B3_API bool b3OverlapSphere( const b3Sphere* shape, b3Transform shapeTransform, const b3ShapeProxy* proxy );

B3_API b3CastOutput b3RayCastSphere( const b3Sphere* shape, const b3RayCastInput* input );
B3_API b3CastOutput b3ShapeCastSphere( const b3Sphere* shape, const b3ShapeCastInput* input );

/// Ray cast a sphere treated as a hollow shell, so a ray starting inside
/// reports the far wall instead of missing.
///
/// Unlike upstream, the reported fraction is a fraction of the ray, matching
/// b3RayCastSphere and every other cast. See the implementation note.
B3_API b3CastOutput b3RayCastHollowSphere( const b3Sphere* shape, const b3RayCastInput* input );

// =========================================================================
// Capsule
// =========================================================================

/// Compute mass properties. The returned inertia is **per unit mass**.
///
/// Composing the cylinder and the two hemispheres requires mass-weighting
/// rather than simple addition, because per-unit-mass inertias are not
/// additive. See the implementation for why that is the cheaper trade.
B3_API b3MassData b3ComputeCapsuleMass( const b3Capsule* shape, b3f density );

B3_API b3AABB b3ComputeCapsuleAABB( const b3Capsule* shape, b3Transform transform );
B3_API b3AABB b3ComputeSweptCapsuleAABB( const b3Capsule* shape, b3Transform xf1, b3Transform xf2 );

B3_API bool b3OverlapCapsule( const b3Capsule* shape, b3Transform shapeTransform, const b3ShapeProxy* proxy );

/// Ray cast a capsule.
///
/// Degenerate and near-parallel configurations are resolved with thresholds
/// derived from Q12 resolution rather than translated from float epsilons --
/// see B3_CAPSULE_RAY_MIN_DET in capsule.c for the error analysis.
B3_API b3CastOutput b3RayCastCapsule( const b3Capsule* shape, const b3RayCastInput* input );
B3_API b3CastOutput b3ShapeCastCapsule( const b3Capsule* shape, const b3ShapeCastInput* input );

// =========================================================================
// Convex hull
// =========================================================================
//
// Implemented in hull.c, which holds the query half only -- hulls are built
// on the host and baked, so nothing here allocates. The accessors resolve
// the byte offsets in b3HullData; a zero offset means the array is absent.

B3_INLINE const b3HullVertex* b3GetHullVertices( const b3HullData* hull )
{
	if ( hull->vertexOffset == 0 )
	{
		return NULL;
	}

	return (const b3HullVertex*)( (intptr_t)hull + hull->vertexOffset );
}

B3_INLINE const b3Vec3* b3GetHullPoints( const b3HullData* hull )
{
	if ( hull->pointOffset == 0 )
	{
		return NULL;
	}

	return (const b3Vec3*)( (intptr_t)hull + hull->pointOffset );
}

B3_INLINE const b3HullHalfEdge* b3GetHullEdges( const b3HullData* hull )
{
	if ( hull->edgeOffset == 0 )
	{
		return NULL;
	}

	return (const b3HullHalfEdge*)( (intptr_t)hull + hull->edgeOffset );
}

B3_INLINE const b3Plane* b3GetHullPlanes( const b3HullData* hull )
{
	if ( hull->planeOffset == 0 )
	{
		return NULL;
	}

	return (const b3Plane*)( (intptr_t)hull + hull->planeOffset );
}

B3_INLINE const b3HullFace* b3GetHullFaces( const b3HullData* hull )
{
	if ( hull->faceOffset == 0 )
	{
		return NULL;
	}

	return (const b3HullFace*)( (intptr_t)hull + hull->faceOffset );
}

/// Index of the hull point furthest along the given direction.
B3_API int b3FindHullSupportVertex( const b3HullData* hull, b3Vec3 direction );

/// Index of the hull face whose normal is closest to the given direction.
B3_API int b3FindHullSupportFace( const b3HullData* hull, b3Vec3 direction );

/// Check the half-edge structure, the face planes and convexity.
///
/// Worth running on any hull that came from outside the program: a baked
/// blob is untrusted input, and a hull that is not convex makes the SAT
/// silently wrong rather than loudly wrong.
B3_API bool b3IsValidHull( const b3HullData* hull );

/// Make a box hull. The topology is constant, so this needs no hull builder
/// and no allocation -- do not call b3DestroyHull on the result.
B3_API b3BoxHull b3MakeBoxHull( b3f hx, b3f hy, b3f hz );

/// Make a cube hull.
B3_API b3BoxHull b3MakeCubeHull( b3f halfWidth );

/// Make a box hull centred on an offset.
B3_API b3BoxHull b3MakeOffsetBoxHull( b3f hx, b3f hy, b3f hz, b3Vec3 offset );

/// Make a box hull placed by a transform.
B3_API b3BoxHull b3MakeTransformedBoxHull( b3f hx, b3f hy, b3f hz, b3Transform transform );

/// Make a right prism with a regular `sides`-gon cross-section, centred on
/// the origin with its axis along Y.
///
/// `radius` is the circumradius -- the distance from the axis to a vertex,
/// not to a face. Like the box hulls this needs no builder and no
/// allocation; unlike them it can fail, returning a zeroed hull for a side
/// count outside [3, B3_MAX_PRISM_SIDES] or a degenerate size. Check with
/// b3IsValidHull if the arguments are not known good.
B3_API b3PrismHull b3MakePrismHull( b3f radius, b3f halfHeight, int sides );

/// Compute mass properties. The returned inertia is **per unit mass**, and
/// since the hull already stores it that way this is a copy, not a divide.
B3_API b3MassData b3ComputeHullMass( const b3HullData* shape, b3f density );

/// Distance from `origin` to the nearest and farthest points of the hull.
///
/// minExtent is the baked inner radius, so it measures from the hull's own
/// centroid rather than from `origin`; that is upstream's definition and the
/// solver reads it as "the thinnest the shape gets".
B3_API b3ShapeExtent b3ComputeHullExtent( const b3HullData* hull, b3Vec3 origin );

B3_API b3AABB b3ComputeHullAABB( const b3HullData* shape, b3Transform transform );
B3_API b3AABB b3ComputeSweptHullAABB( const b3HullData* shape, b3Transform xf1, b3Transform xf2 );

B3_API bool b3OverlapHull( const b3HullData* shape, b3Transform shapeTransform, const b3ShapeProxy* proxy );

/// Ray cast a hull by clipping the ray against every face plane.
B3_API b3CastOutput b3RayCastHull( const b3HullData* shape, const b3RayCastInput* input );
B3_API b3CastOutput b3ShapeCastHull( const b3HullData* shape, const b3ShapeCastInput* input );

// =========================================================================
// Triangle mesh
// =========================================================================
//
// Implemented in mesh.c, which is the read half only -- meshes are built on
// the host and baked, so nothing here allocates. The accessors resolve the
// byte offsets in b3MeshData; a zero offset means the array is absent.
//
// b3RayCastMesh, b3ShapeCastMesh, b3OverlapMesh and b3CollideMoverAndMesh are
// Phase 7, with the rest of the query layer.

B3_INLINE const b3MeshNode* b3GetMeshNodes( const b3MeshData* mesh )
{
	if ( mesh->nodeOffset == 0 )
	{
		return NULL;
	}

	return (const b3MeshNode*)( (intptr_t)mesh + mesh->nodeOffset );
}

B3_INLINE const b3Vec3* b3GetMeshVertices( const b3MeshData* mesh )
{
	if ( mesh->vertexOffset == 0 )
	{
		return NULL;
	}

	return (const b3Vec3*)( (intptr_t)mesh + mesh->vertexOffset );
}

B3_INLINE const b3MeshTriangle* b3GetMeshTriangles( const b3MeshData* mesh )
{
	if ( mesh->triangleOffset == 0 )
	{
		return NULL;
	}

	return (const b3MeshTriangle*)( (intptr_t)mesh + mesh->triangleOffset );
}

B3_INLINE const uint8_t* b3GetMeshMaterialIndices( const b3MeshData* mesh )
{
	if ( mesh->materialOffset == 0 )
	{
		return NULL;
	}

	return (const uint8_t*)( (intptr_t)mesh + mesh->materialOffset );
}

/// Per-triangle b3MeshEdgeFlags. One byte each, so the count is the triangle
/// count.
B3_INLINE const uint8_t* b3GetMeshFlags( const b3MeshData* mesh )
{
	if ( mesh->flagsOffset == 0 )
	{
		return NULL;
	}

	return (const uint8_t*)( (intptr_t)mesh + mesh->flagsOffset );
}

/// Check the version, the offsets, and that every node's bounds contain what
/// the node points at.
///
/// Compiled in unconditionally, unlike upstream's, for the reason b3IsValidHull
/// gives: a baked blob is untrusted input, and a BVH whose bounds do not
/// contain their triangles silently loses contacts rather than failing.
B3_API bool b3IsValidMesh( const b3MeshData* mesh );

/// Clamp a mesh scale away from zero, preserving sign.
B3_API b3Vec3 b3SafeScale( b3Vec3 scale );

/// The mesh's world AABB. Reads the baked bounds; no traversal.
B3_API b3AABB b3ComputeMeshAABB( const b3MeshData* shape, b3Transform transform, b3Vec3 scale );

// b3GetMeshTriangle returns b3Triangle, which is internal to the collision
// tier, so it is declared in source/box3d/mesh.h alongside it -- as upstream
// does.

/// Called once per triangle whose bounds overlap the query box. Return false
/// to stop the query.
typedef bool b3MeshQueryFcn( b3Vec3 a, b3Vec3 b, b3Vec3 c, int triangleIndex, void* context );

/// Visit every triangle overlapping `bounds`, in **ascending triangle index**
/// order with no repeats.
///
/// That ordering is not incidental: the baker stores triangles in depth-first
/// leaf order and this descends left-child-first, so the two compose. The mesh
/// narrow phase matches its per-triangle cache against the result with a linear
/// merge join, which is only correct because of it.
B3_API void b3QueryMesh( const b3Mesh* mesh, b3AABB bounds, b3MeshQueryFcn* fcn, void* context );

// =========================================================================
// Triangle manifolds
// =========================================================================
//
// Implemented in triangle_manifold.c. The mesh narrow phase transforms the
// other shape into the mesh's frame before calling, so unlike the hull
// entry points none of these takes a transform.
//
// The normal always points from the triangle to the other shape, and a
// triangle is one-sided: a contact from behind its plane is culled, because
// a mesh triangle's back face is the inside of the level.

/// The point of triangle abc closest to q, and which feature it lies on.
B3_API b3TrianglePoint b3ClosestPointOnTriangle( b3Vec3 a, b3Vec3 b, b3Vec3 c, b3Vec3 q );

/// Collide a triangle and a hull. Normal points from triangle to hull.
///
/// `enableSpeculative` widens the contact window to B3_SPECULATIVE_DISTANCE;
/// the mesh narrow phase passes it from the contact's `isFast` flag.
B3_API void b3CollideTriangleAndHull( b3LocalManifold* manifold, int capacity, b3Vec3 v1, b3Vec3 v2, b3Vec3 v3,
									  int triangleFlags, const b3HullData* hullB, b3SATCache* cache,
									  bool enableSpeculative );

/// Collide a triangle and a capsule. Normal points from triangle to capsule.
///
/// GJK first, falling back to a separating axis test only on deep overlap.
/// The cache is the caller's and is warm-started through b3ShapeDistance; a
/// separated pair deliberately leaves it alone.
B3_API void b3CollideTriangleAndCapsule( b3LocalManifold* manifold, int capacity, const b3Vec3* triangleA,
										 const b3Capsule* capsuleB, b3SimplexCache* cache );

/// Collide a triangle and a sphere. At most one point.
///
/// The only writer of b3LocalManifold::squaredDistance, which the mesh narrow
/// phase orders its sphere candidates by.
B3_API void b3CollideTriangleAndSphere( b3LocalManifold* manifold, int capacity, const b3Vec3* triangleA,
										const b3Sphere* sphereB );

// =========================================================================
// Support points
// =========================================================================
//
// Implemented in distance.c. Both shift the first point to the origin before
// projecting, which keeps the comparison meaningful for point clouds far from
// the world origin.

/// Index of the point in the proxy furthest along the given axis.
B3_API int b3GetProxySupport( const b3ShapeProxy* proxy, b3Vec3 axis );

/// Index of the point in the array furthest along the given axis.
B3_API int b3GetPointSupport( const b3Vec3* points, int count, b3Vec3 axis );

// =========================================================================
// Contact manifolds
// =========================================================================
//
// Implemented in convex_manifold.c.
//
// The hull/sphere and hull/capsule entry points take a b3SimplexCache because
// they run GJK first and warm-start it from the previous step. b3CollideHulls
// runs no GJK, so it caches a separating *feature* instead.

B3_API void b3CollideSpheres( b3LocalManifold* manifold, int capacity, const b3Sphere* sphereA, const b3Sphere* sphereB,
							  b3Transform transformBtoA );

B3_API void b3CollideCapsuleAndSphere( b3LocalManifold* manifold, int capacity, const b3Capsule* capsuleA,
									   const b3Sphere* sphereB, b3Transform transformBtoA );

B3_API void b3CollideCapsules( b3LocalManifold* manifold, int capacity, const b3Capsule* capsuleA, const b3Capsule* capsuleB,
							   b3Transform transformBtoA );

B3_API void b3CollideHullAndSphere( b3LocalManifold* manifold, int capacity, const b3HullData* hullA,
									const b3Sphere* sphereB, b3Transform transformBtoA, b3SimplexCache* cache );

B3_API void b3CollideHullAndCapsule( b3LocalManifold* manifold, int capacity, const b3HullData* hullA,
									 const b3Capsule* capsuleB, b3Transform transformBtoA, b3SimplexCache* cache );

/// Contact manifold between two convex hulls, in frame A.
///
/// The cache remembers which separating feature won last time, so a pair that
/// has not moved much can re-test one axis instead of all of them.
/// b3_invalidAxis is zero, so a zero-initialized cache means "nothing
/// remembered" and the first call takes the full path. `cache->hit` reports
/// whether the shortcut was taken.
///
/// @param capacity must be at least 4. A hull-hull manifold is reduced to four
/// points and there is no meaningful three-point answer, so a smaller capacity
/// produces no points at all rather than a truncated manifold.
B3_API void b3CollideHulls( b3LocalManifold* manifold, int capacity, const b3HullData* hullA, const b3HullData* hullB,
							b3Transform transformBtoA, b3SATCache* cache );

// =========================================================================
// Dynamic tree
// =========================================================================
//
// Implemented in dynamic_tree.c. Upstream declares the tree here too.
//
// Serialization (b3DynamicTree_Save / _Load) is absent: it is built on stdio.

/// Create a tree sized for the given number of proxies. The pool grows on
/// demand, so this is a hint rather than a limit.
B3_API b3DynamicTree b3DynamicTree_Create( int proxyCapacity );

/// Destroy a tree and release its memory.
B3_API void b3DynamicTree_Destroy( b3DynamicTree* tree );

/// Create a proxy as a leaf node.
///
/// @return the proxy id, or B3_NULL_INDEX if the node pool could not grow. A
/// null return follows the port's refusal policy; upstream cannot fail here
/// because it allocates unconditionally.
B3_API int b3DynamicTree_CreateProxy( b3DynamicTree* tree, b3AABB aabb, uint64_t categoryBits, uint64_t userData );

/// Destroy a proxy. This asserts if the id is invalid.
B3_API void b3DynamicTree_DestroyProxy( b3DynamicTree* tree, int proxyId );

/// Move a proxy to a new AABB by removing and reinserting into the tree.
B3_API void b3DynamicTree_MoveProxy( b3DynamicTree* tree, int proxyId, b3AABB aabb );

/// Enlarge a proxy and enlarge ancestors as necessary.
B3_API void b3DynamicTree_EnlargeProxy( b3DynamicTree* tree, int proxyId, b3AABB aabb );

/// Modify the category bits on a proxy. This is an expensive operation.
B3_API void b3DynamicTree_SetCategoryBits( b3DynamicTree* tree, int proxyId, uint64_t categoryBits );

/// Get the category bits on a proxy.
B3_API uint64_t b3DynamicTree_GetCategoryBits( b3DynamicTree* tree, int proxyId );

/// Query an AABB for overlapping proxies. The callback is called for each
/// proxy that overlaps the supplied AABB.
/// @return performance data
B3_API b3TreeStats b3DynamicTree_Query( const b3DynamicTree* tree, b3AABB aabb, uint64_t maskBits, bool requireAllBits,
										b3TreeQueryCallbackFcn* callback, void* context );

/// Query for the closest proxy to a point.
///
/// @param minDistanceSqr the initial and final minimum **squared** distance.
/// A small initial value restricts the search; a large one makes this scale
/// linearly with the proxy count. Squared distances are Q24 in int64, not
/// b3f -- see b3TreeQueryClosestCallbackFcn in types.h.
/// @return performance data
B3_API b3TreeStats b3DynamicTree_QueryClosest( const b3DynamicTree* tree, b3Vec3 point, uint64_t maskBits, bool requireAllBits,
											   b3TreeQueryClosestCallbackFcn* callback, void* context,
											   int64_t* minDistanceSqr );

/// Ray cast against the proxies in the tree. The callback performs the exact
/// cast and any filtering, and returns the new clip fraction.
/// @return performance data
B3_API b3TreeStats b3DynamicTree_RayCast( const b3DynamicTree* tree, const b3RayCastInput* input, uint64_t maskBits,
										  bool requireAllBits, b3TreeRayCastCallbackFcn* callback, void* context );

/// Sweep an AABB through the tree.
B3_API b3TreeStats b3DynamicTree_BoxCast( const b3DynamicTree* tree, const b3BoxCastInput* input, uint64_t maskBits,
										  bool requireAllBits, b3TreeBoxCastCallbackFcn* callback, void* context );

/// Get the height of the binary tree.
B3_API int b3DynamicTree_GetHeight( const b3DynamicTree* tree );

/// Ratio of the sum of the internal node areas to the root area, a tree
/// quality metric. Dimensionless, so an ordinary b3f.
B3_API b3f b3DynamicTree_GetAreaRatio( const b3DynamicTree* tree );

/// Get the bounding box that contains the entire tree.
B3_API b3AABB b3DynamicTree_GetRootBounds( const b3DynamicTree* tree );

/// Get the number of proxies created.
B3_API int b3DynamicTree_GetProxyCount( const b3DynamicTree* tree );

/// Get the user data on a proxy.
B3_INLINE uint64_t b3DynamicTree_GetUserData( const b3DynamicTree* tree, int proxyId )
{
	return tree->nodes[proxyId].userData;
}

/// Get the AABB of a proxy.
B3_INLINE b3AABB b3DynamicTree_GetAABB( const b3DynamicTree* tree, int proxyId )
{
	return tree->nodes[proxyId].aabb;
}

/// Rebuild the tree bottom up. @return the number of leaves rebuilt.
B3_API int b3DynamicTree_Rebuild( b3DynamicTree* tree, bool fullBuild );

/// Get the number of bytes used by this tree.
B3_API int b3DynamicTree_GetByteCount( const b3DynamicTree* tree );

/// Validate the tree structure. Compiled out unless B3_ENABLE_VALIDATION.
B3_API void b3DynamicTree_Validate( const b3DynamicTree* tree );

/// Validate that no node is marked enlarged. Compiled out likewise.
B3_API void b3DynamicTree_ValidateNoEnlarged( const b3DynamicTree* tree );
