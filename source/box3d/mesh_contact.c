// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Erin Catto        (original Box3D)
// Copyright (c) 2026 Warioware64       (Nitro Engine Advanced fixed-point port)
//
// This file is part of Nitro Engine Advanced

/// @file   mesh_contact.c
/// @brief  The shape-versus-triangle-mesh narrow phase.
///
/// One function does the work -- b3ComputeMeshManifolds -- and it is five
/// stages: refresh the per-triangle cache from a BVH query, collide each
/// overlapping triangle, filter the ghost contacts an interior edge would
/// otherwise produce, cluster what survives by normal, and match the clusters
/// against last step's manifolds so the warm-start impulses follow.
///
/// @section caps Where this departs from upstream
///
/// Upstream's caps are 256 triangles and 32 points per triangle, which puts a
/// 196 KB point buffer on the arena for one contact. This port's are 32 and 8
/// (nea_config.h), and it additionally caps the *output* at
/// B3_NEA_MAX_MESH_MANIFOLDS clusters -- a bound upstream does not have,
/// because upstream allocates per cluster inside the step and this port refuses
/// to. The retention rule at that cap is the port's own and is documented at
/// the constant.
///
/// Everything else here is a transliteration, including two upstream defects
/// that are deliberately preserved; both are marked where they occur.

#include "mesh_contact.h"

#include "contact.h"
#include "manifold.h"
#include "mesh.h"
#include "physics_world.h"
#include "shape.h"

#include "box3d/collision.h"
#include "box3d/constants.h"
#include "box3d/nea_config.h"
#include "box3d/types.h"

#include <string.h>

/// Upstream's debug switch for forcing every triangle contact through the
/// accepted path, bypassing the ghost filter. Kept, and kept off, because the
/// filter is the hardest thing in this file to be sure of and turning it off is
/// how a suspected ghost collision is attributed.
#define B3_FORCE_GHOST_COLLISIONS 0

// =========================================================================
// The ghost filter's two tables
// =========================================================================
//
// A contact against an edge or a vertex shared by two triangles must be
// claimed by exactly one of them, or a body sliding across a flat floor
// catches on the seams. These record which edges and vertices the accepted
// manifolds have already claimed.
//
// Both counts stay at upstream's 64 rather than dropping to the port's
// 32-triangle cap. A patch of 32 triangles has up to 96 edges, so 64 is not a
// bound either way -- and matching upstream's number keeps run_pair comparing
// two libraries that overflow at the same point, which a smaller table would
// turn into a configuration difference reported as a port bug.

#define B3_MAX_EDGE_COUNT 64
#define B3_MAX_VERTEX_COUNT 64

typedef struct b3FoundEdges
{
	uint64_t keys[B3_MAX_EDGE_COUNT];
	int count;
} b3FoundEdges;

/// @return true if this edge had not been claimed yet, so the caller may keep
/// its contact. On overflow it returns true, which upstream notes will lead to
/// a potential ghost collision -- deliberately, because dropping a contact the
/// filter cannot reason about is the worse failure.
static inline bool b3AddEdge( b3FoundEdges* edges, int vertex1, int vertex2 )
{
	uint64_t i1 = (uint64_t)b3MinInt( vertex1, vertex2 );
	uint64_t i2 = (uint64_t)b3MaxInt( vertex1, vertex2 );
	uint64_t key = i1 << 32 | i2;

	int count = edges->count;
	for ( int i = 0; i < count; ++i )
	{
		if ( edges->keys[i] == key )
		{
			return false;
		}
	}

	if ( count == B3_MAX_EDGE_COUNT )
	{
		// This will lead to a potential ghost collision
		return true;
	}

	edges->keys[count] = key;
	edges->count += 1;

	return true;
}

static inline bool b3FindEdge( const b3FoundEdges* edges, int vertex1, int vertex2 )
{
	uint64_t i1 = (uint64_t)b3MinInt( vertex1, vertex2 );
	uint64_t i2 = (uint64_t)b3MaxInt( vertex1, vertex2 );
	uint64_t key = i1 << 32 | i2;

	int count = edges->count;
	for ( int i = 0; i < count; ++i )
	{
		if ( edges->keys[i] == key )
		{
			return true;
		}
	}

	return false;
}

typedef struct b3FoundVertices
{
	int keys[B3_MAX_VERTEX_COUNT];
	int count;
} b3FoundVertices;

static inline bool b3AddVertex( b3FoundVertices* vertices, int vertex )
{
	int count = vertices->count;
	for ( int i = 0; i < count; ++i )
	{
		if ( vertices->keys[i] == vertex )
		{
			return false;
		}
	}

	if ( count == B3_MAX_VERTEX_COUNT )
	{
		// This will lead to a potential ghost collision
		return true;
	}

	vertices->keys[count] = vertex;
	vertices->count += 1;

	return true;
}

/// Claim all three edges and all three vertices of an accepted triangle.
///
/// Upstream spells these six calls out three times over, once per accepted
/// path, discarding every return value. Folded here because the discard is what
/// distinguishes it from the tentative loop below, which reads the returns.
static inline void b3ClaimTriangle( b3FoundEdges* edges, b3FoundVertices* vertices, int i1, int i2, int i3 )
{
	(void)b3AddEdge( edges, i1, i2 );
	(void)b3AddEdge( edges, i2, i3 );
	(void)b3AddEdge( edges, i3, i1 );
	(void)b3AddVertex( vertices, i1 );
	(void)b3AddVertex( vertices, i2 );
	(void)b3AddVertex( vertices, i3 );
}

// =========================================================================
// The triangle cache refresh
// =========================================================================

#if B3_ENABLE_VALIDATION
/// The invariant the merge join below is a merge join *because of*.
///
/// b3QueryMesh emits ascending, duplicate-free triangle indices, which follows
/// from the baker storing triangles in depth-first leaf order and the descent
/// taking the left child first. Nothing else asserts it at run time, and if it
/// breaks the only symptom is that the warm cache silently stops matching.
static bool b3IsSorted( const int* array, int count )
{
	for ( int i = 0; i < count - 1; ++i )
	{
		if ( array[i] >= array[i + 1] )
		{
			return false;
		}
	}

	return true;
}
#endif

typedef struct b3TriangleQueryContext
{
	int* indices;
	int capacity;
	int count;
} b3TriangleQueryContext;

static bool b3CollectTriangleIndicesCallback( b3Vec3 a, b3Vec3 b, b3Vec3 c, int triangleIndex, void* context )
{
	B3_UNUSED( a, b, c );

	b3TriangleQueryContext* triangleContext = (b3TriangleQueryContext*)context;
	if ( triangleContext->count == triangleContext->capacity )
	{
		return false;
	}

	triangleContext->indices[triangleContext->count] = triangleIndex;
	triangleContext->count += 1;
	return triangleContext->count < triangleContext->capacity;
}

static int b3QueryMeshTriangles( int* indices, int capacity, const b3Mesh* mesh, b3AABB bounds )
{
	b3TriangleQueryContext context = {
		.indices = indices,
		.capacity = capacity,
		.count = 0,
	};

	b3QueryMesh( mesh, bounds, b3CollectTriangleIndicesCallback, &context );
	return context.count;
}

/// Re-query the mesh if shape B has left the box the last query covered, and
/// carry each triangle's warm cache across.
///
/// `arena` is by value: both scratch buffers are released when this returns,
/// so the peak the builder below sees is its own, not the sum. Upstream keeps
/// them on the C stack, which at its 256-triangle cap is 6 KB.
static void b3RefreshCache( b3Contact* contact, const b3Shape* shapeA, b3WorldTransform xfA, const b3AABB* bounds,
							b3Arena arena )
{
	B3_ASSERT( shapeA->type == b3_meshShape );

	b3MeshContact* meshContact = &contact->meshContact;

	// If the dynamic body didn't move out of the cached query bounds we are done!
	if ( b3AABB_Contains( meshContact->queryBounds, *bounds ) )
	{
		return;
	}

	// Enlarge to the query bounds to absorb small movement
	b3f radius = b3AddF( B3_MAX_AABB_MARGIN, B3_SPECULATIVE_DISTANCE );
	b3Vec3 extension = { radius, radius, radius };
	meshContact->queryBounds.lowerBound = b3Sub( bounds->lowerBound, extension );
	meshContact->queryBounds.upperBound = b3Add( bounds->upperBound, extension );

	int triangleCapacity = B3_NEA_MAX_MESH_CONTACT_TRIANGLES;
	int* triangleIndices = (int*)b3Bump( &arena, triangleCapacity * (int)sizeof( int ) );

	// Bounds are in world space; the query wants them in the mesh's frame.
	// Upstream demotes the mesh transform to float first, because its world
	// transform carries a wider position type. b3WorldTransform *is*
	// b3Transform here (math_fixed.h), so there is nothing to demote.
	b3AABB localBounds = b3AABB_Transform( b3InvertTransform( xfA ), meshContact->queryBounds );

	int triangleCount = b3QueryMeshTriangles( triangleIndices, triangleCapacity, &shapeA->mesh, localBounds );

	if ( triangleCount == triangleCapacity )
	{
		// Upstream logs this once per process. Every triangle past the cap is a
		// collide that does not run, so it is a contact quality problem rather
		// than a memory one -- and on a DS the answer is a coarser level or a
		// larger B3_NEA_MAX_MESH_CONTACT_TRIANGLES, both of which are decisions
		// for whoever reads the log.
		static bool s_once = false;
		if ( s_once == false )
		{
			b3Log( "WARNING: complex mesh contact, triangle buffer capacity of %d reached\n", triangleCapacity );
			s_once = true;
		}
	}

	// Triangle indices must be sorted to match caches.
	//
	// Upstream spells this B3_VALIDATE, which this port compiles out always --
	// that tier is float-tolerance checks with no fixed-point meaning
	// (base.h:142). This is an exact integer check, so it belongs in the
	// B3_ENABLE_VALIDATION tier instead, beside the structural walks.
#if B3_ENABLE_VALIDATION
	B3_ASSERT( b3IsSorted( triangleIndices, triangleCount ) );
#endif

	// Build this step's caches by merge-joining the new index list against the
	// old one. Both are ascending, so one pass over each suffices.
	b3ContactCache* contactCache = (b3ContactCache*)b3Bump( &arena, triangleCapacity * (int)sizeof( b3ContactCache ) );

	b3TriangleCache* triangleCache = meshContact->triangleCache;
	int oldCount = meshContact->triangleCount;

	int index2 = 0;
	for ( int index1 = 0; index1 < triangleCount; ++index1 )
	{
		contactCache[index1] = ( b3ContactCache ){ 0 };

		while ( index2 < oldCount && triangleCache[index2].triangleIndex < triangleIndices[index1] )
		{
			index2 += 1;
		}

		if ( index2 < oldCount && triangleCache[index2].triangleIndex == triangleIndices[index1] )
		{
			contactCache[index1] = triangleCache[index2].cache;
		}
	}

	// Save the new cache. The array is fixed capacity and was taken when the
	// contact was created, so this is a write, never a resize.
	meshContact->triangleCount = triangleCount;
	for ( int i = 0; i < triangleCount; ++i )
	{
		B3_ASSERT( 0 <= triangleIndices[i] && triangleIndices[i] < shapeA->mesh.data->triangleCount );
		triangleCache[i] = ( b3TriangleCache ){ triangleIndices[i], contactCache[i] };
	}
}

// =========================================================================
// Cluster reduction
// =========================================================================

/// One accepted manifold awaiting the ghost filter's verdict.
typedef struct b3TentativeTriangle
{
	/// **Written only by the sphere path.** b3CollideTriangleAndSphere is the
	/// only b3Collide function that fills b3LocalManifold::squaredDistance, so
	/// for a capsule or a hull this field carries whatever the manifold buffer
	/// held. That is an upstream defect and it is harmless there for one
	/// reason only -- the sort below is sphere-only -- so the port keeps the
	/// sort sphere-only too and zero-initialises this rather than "fixing" it
	/// by sorting unconditionally, which would sort on garbage.
	int64_t squaredDistance;
	int index;
} b3TentativeTriangle;

/// Returns true if (score, separation) should replace (bestScore, bestSeparation).
///
/// Both scores are areas or squared lengths at Q24 and both separations are raw
/// Q12 lengths; the caller keeps them wide, so nothing here narrows.
static inline bool b3IsBetterCullCandidate( int64_t score, int64_t separation, int64_t bestScore, int64_t bestSeparation,
											int64_t scoreTol, int64_t separationTol )
{
	if ( score > bestScore + scoreTol )
	{
		return true;
	}
	if ( score < bestScore - scoreTol )
	{
		return false;
	}

	// Break the tie using separation
	return separation < bestSeparation - separationTol;
}

/// A cluster point projected into the cluster's own plane.
///
/// Deliberately not a general b3Vec2. b3CullPoints is the only 2D code in the
/// port, and a public 2D vector type would invite a b3Cross2 that narrows to
/// Q12 -- where the pairwise cross products of a contact patch a few
/// hundredths of a unit across are exactly zero.
typedef struct b3Point2D
{
	b3f x, y;
	b3f separation;
	int originalIndex;
} b3Point2D;

/// Squared distance between two projected points, at Q24.
static inline int64_t b3PointDistanceSquared2( b3Point2D a, b3Point2D b )
{
	int64_t dx = (int64_t)b3Raw( a.x ) - b3Raw( b.x );
	int64_t dy = (int64_t)b3Raw( a.y ) - b3Raw( b.y );
	return dx * dx + dy * dy;
}

/// Twice the signed area of (0, u, v), at Q24. The 2D analogue of
/// b3SignedAreaWide, and wide for the same reason.
static inline int64_t b3Cross2Wide( int64_t ux, int64_t uy, int64_t vx, int64_t vy )
{
	return ux * vy - uy * vx;
}

/// Reduce a set of coplanar points to at most four, keeping the ones that hold
/// the contact patch open: the two furthest apart, then the one adding the most
/// triangular area, then the one adding the most area outside that triangle.
///
/// @note Rearranges `points`; the survivors are written to the front.
static int b3CullPoints( b3Point2D* points, int count )
{
	if ( count <= 1 )
	{
		return count;
	}

	// 0.25 * B3_LINEAR_SLOP is 5 raw, so this is 25 at Q24 -- a hundredth of a
	// Q12 quantum squared. It is a coincidence guard, not a tolerance: the
	// `bestScore < tolSqr` branch below is reached only when every point in the
	// cluster is the same point.
	const int64_t tolSqr = ( (int64_t)b3Raw( B3_LINEAR_SLOP ) * b3Raw( B3_LINEAR_SLOP ) ) / 16;
	const int64_t separationTol = b3Raw( B3_LINEAR_SLOP );

	b3Point2D finalPoints[4];
	int count1 = count;

	// Step 1: the two points with the largest distance, ties broken by deepest
	// combined separation.
	//
	// The separation *sum* is wide: two b3f are being added before anything
	// compares them, and B3_F_MAX as the running-best sentinel then has to
	// survive `bestSeparation - separationTol` without wrapping. Wide makes
	// both non-questions.
	int64_t bestScore = 0;
	int64_t bestSeparation = b3Raw( B3_F_MAX );
	int bestIndex1 = B3_NULL_INDEX;
	int bestIndex2 = B3_NULL_INDEX;

	for ( int i = 0; i < count1; ++i )
	{
		for ( int j = i + 1; j < count1; ++j )
		{
			int64_t score = b3PointDistanceSquared2( points[i], points[j] );
			int64_t separation = (int64_t)b3Raw( points[i].separation ) + b3Raw( points[j].separation );

			if ( b3IsBetterCullCandidate( score, separation, bestScore, bestSeparation, tolSqr, separationTol ) )
			{
				bestIndex1 = i;
				bestIndex2 = j;
				bestScore = score;
				bestSeparation = separation;
			}
		}
	}

	if ( bestScore < tolSqr )
	{
		// Choose deepest point
		int deepestIndex = 0;
		for ( int i = 1; i < count1; ++i )
		{
			if ( b3Raw( points[i].separation ) < b3Raw( points[deepestIndex].separation ) )
			{
				deepestIndex = i;
			}
		}

		if ( deepestIndex != 0 )
		{
			points[0] = points[deepestIndex];
		}
		return 1;
	}

	B3_ASSERT( bestIndex1 != B3_NULL_INDEX && bestIndex2 != B3_NULL_INDEX );

	finalPoints[0] = points[bestIndex1];
	finalPoints[1] = points[bestIndex2];

	// Cull
	points[bestIndex2] = points[count1 - 1];
	points[bestIndex1] = points[count1 - 2];
	count1 -= 2;

	if ( count1 == 0 )
	{
		points[0] = finalPoints[0];
		points[1] = finalPoints[1];
		return 2;
	}

	// First and second anchor points.
	int64_t ax = b3Raw( finalPoints[0].x );
	int64_t ay = b3Raw( finalPoints[0].y );
	int64_t bx = b3Raw( finalPoints[1].x );
	int64_t by = b3Raw( finalPoints[1].y );
	int64_t bax = bx - ax;
	int64_t bay = by - ay;

	// Step 2: the point with the maximum triangular area, ties broken by
	// deepest separation.
	bestScore = 0;
	bestSeparation = b3Raw( B3_F_MAX );
	int bestIndex = B3_NULL_INDEX;
	int64_t bestSignedArea = 0;

	for ( int i = 0; i < count1; ++i )
	{
		int64_t signedArea = b3Cross2Wide( bax, bay, (int64_t)b3Raw( points[i].x ) - ax, (int64_t)b3Raw( points[i].y ) - ay );
		int64_t score = signedArea < 0 ? -signedArea : signedArea;

		if ( b3IsBetterCullCandidate( score, b3Raw( points[i].separation ), bestScore, bestSeparation, tolSqr,
									  separationTol ) )
		{
			bestSignedArea = signedArea;
			bestScore = score;
			bestSeparation = b3Raw( points[i].separation );
			bestIndex = i;
		}
	}

	if ( bestIndex == B3_NULL_INDEX )
	{
		// All points collinear
		points[0] = finalPoints[0];
		points[1] = finalPoints[1];
		return 2;
	}

	finalPoints[2] = points[bestIndex];

	if ( count1 == 1 )
	{
		points[0] = finalPoints[0];
		points[1] = finalPoints[1];
		points[2] = finalPoints[2];
		return 3;
	}

	// Cull
	points[bestIndex] = points[count1 - 1];
	count1 -= 1;

	// Step 3: the point that adds the most area outside the current triangle.
	int64_t cx = b3Raw( finalPoints[2].x );
	int64_t cy = b3Raw( finalPoints[2].y );

	// Ensure CCW ordering
	if ( bestSignedArea < 0 )
	{
		int64_t tx = bx, ty = by;
		bx = cx;
		by = cy;
		cx = tx;
		cy = ty;
		bax = bx - ax;
		bay = by - ay;
	}

	int64_t cbx = cx - bx;
	int64_t cby = cy - by;
	int64_t acx = ax - cx;
	int64_t acy = ay - cy;

	bestScore = 0;
	bestSeparation = b3Raw( B3_F_MAX );
	bestIndex = B3_NULL_INDEX;

	for ( int i = 0; i < count1; ++i )
	{
		int64_t px = b3Raw( points[i].x );
		int64_t py = b3Raw( points[i].y );

		int64_t u1 = b3Cross2Wide( px - ax, py - ay, bax, bay );
		int64_t u2 = b3Cross2Wide( px - bx, py - by, cbx, cby );
		int64_t u3 = b3Cross2Wide( px - cx, py - cy, acx, acy );

		int64_t score = u1 > u2 ? u1 : u2;
		score = score > u3 ? score : u3;

		// Use the area tolerance for collinear points and hysteresis
		if ( b3IsBetterCullCandidate( score, b3Raw( points[i].separation ), bestScore, bestSeparation, tolSqr,
									  separationTol ) )
		{
			bestScore = score;
			bestSeparation = b3Raw( points[i].separation );
			bestIndex = i;
		}
	}

	if ( bestIndex == B3_NULL_INDEX )
	{
		// No additional area
		points[0] = finalPoints[0];
		points[1] = finalPoints[1];
		points[2] = finalPoints[2];
		return 3;
	}

	finalPoints[3] = points[bestIndex];

	points[0] = finalPoints[0];
	points[1] = finalPoints[1];
	points[2] = finalPoints[2];
	points[3] = finalPoints[3];
	return 4;
}

/// Project a cluster's points into the plane of its triangle normal and cull
/// them to at most B3_MAX_MANIFOLD_POINTS.
static int b3ReduceCluster( b3LocalManifoldPoint* points, int count1, b3Vec3 normal, b3Arena arena )
{
	if ( count1 <= 1 )
	{
		return count1;
	}

	b3Point2D* pts = (b3Point2D*)b3Bump( &arena, count1 * (int)sizeof( b3Point2D ) );

	// b3ArbitraryPerp, not upstream's b3Perp, and not because b3Perp is wrong
	// here: it crosses against the *least* aligned axis, so for a unit normal
	// its result is at least 0.816 long and never degenerates. The reason is
	// that it is a cross product followed by a Q12 normalize, where
	// b3ArbitraryPerp is a linear combination -- one rounding instead of two on
	// the basis this projection is measured in, and every point in the cluster
	// carries that error. b3ReduceManifoldPoints made the same substitution.
	b3Vec3 u = b3ArbitraryPerp( normal );

	// b3Cross is safe here where it is not for triangle edges: both operands
	// are unit length, so the result is unit length, not an area that
	// quantizes away.
	b3Vec3 v = b3Cross( normal, u );

	b3Vec3 origin = points[0].point;

	for ( int i = 0; i < count1; ++i )
	{
		b3Vec3 d = b3Sub( points[i].point, origin );
		pts[i].x = b3Dot( d, u );
		pts[i].y = b3Dot( d, v );
		pts[i].separation = points[i].separation;
		pts[i].originalIndex = i;
	}

	int count2 = b3CullPoints( pts, count1 );
	B3_ASSERT( count2 <= B3_MAX_MANIFOLD_POINTS );

	b3LocalManifoldPoint finalPoints[B3_MAX_MANIFOLD_POINTS];
	for ( int i = 0; i < count2; ++i )
	{
		int index = pts[i].originalIndex;
		B3_ASSERT( 0 <= index && index < count1 );
		finalPoints[i] = points[index];
	}

	memcpy( points, finalPoints, count2 * sizeof( b3LocalManifoldPoint ) );
	return count2;
}

typedef struct b3Cluster
{
	b3Vec3 manifoldNormal;
	b3Vec3 triangleNormal;
	b3LocalManifoldPoint* points;
	int pointCapacity;
	int pointCount;

	/// The deepest point in the cluster once it is populated. Read only by the
	/// cap below, which keeps the deepest clusters when there are too many.
	b3f minSeparation;
} b3Cluster;

// Two cosines between unit vectors, at Q24 -- the scale b3DotWide produces from
// two Q12 unit vectors, with no narrowing anywhere.
//
// Their margin from 1.0 is where the precision goes. A Q12 unit vector's
// components are quantized at 1/4096, so a dot of two of them carries roughly
// 2e-4 of noise against a 4e-3 margin: about sixteen times the noise, not a
// hundred. Two triangles the reference clusters together this port will
// occasionally split, and the other way round. That is a counted disagreement
// in run_pair, not a bug to tune away -- tightening either number would trade
// it for a worse one, since a cluster that should have merged costs a manifold
// out of the cap.
#define B3_CLUSTER_THRESHOLD ( (int64_t)16710084 )		 // 0.996
#define B3_NORMAL_MATCH_TOLERANCE ( (int64_t)16693351 )	 // 0.995

// =========================================================================
// The narrow phase
// =========================================================================

/// @note ITCM group B3_ITCM_MESH -- see nea_config.h.
bool B3_ITCM_IF( B3_ITCM_MESH, b3ComputeMeshManifolds )( b3World* world, b3Contact* contact, const b3Shape* shapeA,
														 b3WorldTransform xfA, const b3Shape* shapeB, b3WorldTransform xfB,
														 bool isFast, b3Arena arena )
{
	B3_ASSERT( shapeA->type == b3_meshShape );
	B3_ASSERT( contact->manifolds != NULL && contact->manifoldCapacity == B3_NEA_MAX_MESH_MANIFOLDS );

	b3RefreshCache( contact, shapeA, xfA, &shapeB->aabb, arena );

	b3MeshContact* meshContact = &contact->meshContact;
	int triangleCount = meshContact->triangleCount;

	// The work unit meshContactTicks is spent on. Accumulated across every mesh
	// contact in the step, so the two together give a cost per triangle that
	// is comparable between scenes -- which the tick count alone is not.
	//
	// Counted before the early return, so a contact whose cache refresh found
	// nothing contributes its honest zero.
#if defined( B3_NEA_PROFILE_NARROW ) && B3_NEA_PROFILE_NARROW
	world->profile.meshTriangleCount += (uint32_t)triangleCount;
#endif

	if ( triangleCount == 0 )
	{
		contact->manifoldCount = 0;
		return false;
	}

	b3LocalManifold** acceptedManifolds = (b3LocalManifold**)b3Bump( &arena, triangleCount * (int)sizeof( b3LocalManifold* ) );
	int acceptedManifoldCount = 0;
	b3LocalManifold** tentativeManifolds = (b3LocalManifold**)b3Bump( &arena, triangleCount * (int)sizeof( b3LocalManifold* ) );
	int tentativeManifoldCount = 0;
	b3TentativeTriangle* tentativeTriangles =
		(b3TentativeTriangle*)b3Bump( &arena, triangleCount * (int)sizeof( b3TentativeTriangle ) );
	int tentativeTriangleCount = 0;

	b3FoundEdges foundEdges;
	b3FoundVertices foundVertices;
	foundEdges.count = 0;
	foundVertices.count = 0;

	// This transform converts from mesh frame into the shapeB frame
	b3Transform transformAtoB = b3InvMulWorldTransforms( xfB, xfA );
	b3Matrix3 relativeMatrix = b3MakeMatrixFromQuat( transformAtoB.q );

	// This should push apart shapes after a time of impact event. Upstream
	// calls it a rest offset, after PhysX and Unreal: it costs a small visual
	// gap and buys noticeably better hull-versus-mesh contact.
	b3f restOffset = B3_MESH_REST_OFFSET;
	bool enableSpeculative = ( contact->flags & b3_enableSpeculativePoints ) != 0;

	int pointBufferCapacity = B3_NEA_MAX_POINTS_PER_TRIANGLE * triangleCount;
	b3LocalManifoldPoint* pointBuffer =
		(b3LocalManifoldPoint*)b3Bump( &arena, pointBufferCapacity * (int)sizeof( b3LocalManifoldPoint ) );
	int totalPointCount = 0;

	b3LocalManifold* manifoldBuffer = (b3LocalManifold*)b3Bump( &arena, triangleCount * (int)sizeof( b3LocalManifold ) );
	int manifoldCount = 0;

	b3TriangleCache* triangleCaches = meshContact->triangleCache;

	const b3HullData* hullB = shapeB->type == b3_hullShape ? shapeB->hull : NULL;

	for ( int index = 0; index < triangleCount && totalPointCount + 3 < pointBufferCapacity; ++index )
	{
		int triangleIndex = triangleCaches[index].triangleIndex;

		b3Triangle triangle = b3GetMeshTriangle( &shapeA->mesh, triangleIndex );

		// Transform triangle into the shape frame. Positions through the Q12
		// matrix, which is the established rule -- they are rebuilt from
		// geometry every step rather than fed back into themselves.
		b3Vec3 vertices[3];
		vertices[0] = b3Add( b3MulMV( relativeMatrix, triangle.vertices[0] ), transformAtoB.p );
		vertices[1] = b3Add( b3MulMV( relativeMatrix, triangle.vertices[1] ), transformAtoB.p );
		vertices[2] = b3Add( b3MulMV( relativeMatrix, triangle.vertices[2] ), transformAtoB.p );

		b3ContactCache* cache = &triangleCaches[index].cache;
		int pointCapacity = pointBufferCapacity - totalPointCount;
		b3LocalManifold* manifold = manifoldBuffer + manifoldCount;
		*manifold = ( b3LocalManifold ){ 0 };
		manifold->points = pointBuffer + totalPointCount;
		manifold->triangleFlags = triangle.flags;
		manifold->feature = b3_featureNone;

		switch ( shapeB->type )
		{
			case b3_capsuleShape:
				b3CollideTriangleAndCapsule( manifold, pointCapacity, vertices, &shapeB->capsule, &cache->simplexCache );
				break;

			case b3_hullShape:
				// Cached edge contact is dangerous at high speed because the
				// hull can rotate around the edge and tunnel through the
				// triangle.
				if ( isFast && cache->satCache.type == b3_edgePairAxis )
				{
					cache->satCache = ( b3SATCache ){ 0 };
				}

				b3CollideTriangleAndHull( manifold, pointCapacity, vertices[0], vertices[1], vertices[2], triangle.flags, hullB,
										  &cache->satCache, enableSpeculative );

				// Upstream accumulates satCallCount / satCacheHitCount into the
				// worker's task context here. Those are b3Counters, dropped in
				// Phase 3A.
				break;

			case b3_sphereShape:
				b3CollideTriangleAndSphere( manifold, pointCapacity, vertices, &shapeB->sphere );
				break;

			default:
				B3_ASSERT( false );
				contact->manifoldCount = 0;
				return false;
		}

		int manifoldPointCount = manifold->pointCount;

		if ( manifoldPointCount == 0 )
		{
			continue;
		}

		B3_ASSERT( manifold->feature != b3_featureNone );

		manifoldCount += 1;
		totalPointCount += manifoldPointCount;
		manifold->triangleIndex = triangleIndex;

		// Through b3MakePlaneFromPoints, whose normal comes from
		// b3CrossDirection: the cross of two 0.01-unit triangle edges is
		// exactly zero at Q12, and every triangle in a level mesh has short
		// edges.
		manifold->triangleNormal = b3MakePlaneFromPoints( vertices[0], vertices[1], vertices[2] ).normal;

		manifold->i1 = triangle.i1;
		manifold->i2 = triangle.i2;
		manifold->i3 = triangle.i3;

		if ( manifold->feature == b3_featureTriangleFace || B3_FORCE_GHOST_COLLISIONS )
		{
			// The triangle's own face answered, so this contact cannot be a
			// ghost. Claim the features and accept.
			b3ClaimTriangle( &foundEdges, &foundVertices, triangle.i1, triangle.i2, triangle.i3 );
			acceptedManifolds[acceptedManifoldCount++] = manifold;
		}
		else if ( manifold->feature == b3_featureHullFace )
		{
			// 0.5 at Q24: the hull face and the triangle face are within 60
			// degrees, so the hull is resting on the triangle rather than
			// catching its edge.
			int64_t cosNormalAngle = b3DotWide( manifold->triangleNormal, manifold->normal );
			if ( cosNormalAngle > ( (int64_t)1 << 23 ) )
			{
				b3ClaimTriangle( &foundEdges, &foundVertices, triangle.i1, triangle.i2, triangle.i3 );
				acceptedManifolds[acceptedManifoldCount++] = manifold;
			}
			else
			{
				b3f minSeparation = manifold->points[0].separation;
				for ( int i = 1; i < manifoldPointCount; ++i )
				{
					minSeparation = b3MinF( minSeparation, manifold->points[i].separation );
				}

				if ( b3Raw( minSeparation ) < -2 * b3Raw( B3_LINEAR_SLOP ) )
				{
					// Deep overlap: accept regardless of the angle, because a
					// dropped contact here is a body sinking into the level.
					b3ClaimTriangle( &foundEdges, &foundVertices, triangle.i1, triangle.i2, triangle.i3 );
					acceptedManifolds[acceptedManifoldCount++] = manifold;
				}
				else
				{
					b3TentativeTriangle tentativeTriangle = { manifold->squaredDistance, tentativeManifoldCount };
					tentativeTriangles[tentativeTriangleCount++] = tentativeTriangle;
					tentativeManifolds[tentativeManifoldCount++] = manifold;
				}
			}
		}
		else
		{
			// An edge or a vertex answered, which is exactly the case a
			// neighbouring triangle may already have claimed.
			b3TentativeTriangle tentativeTriangle = { manifold->squaredDistance, tentativeManifoldCount };
			tentativeTriangles[tentativeTriangleCount++] = tentativeTriangle;
			tentativeManifolds[tentativeManifoldCount++] = manifold;
		}
	}

	B3_ASSERT( acceptedManifoldCount <= triangleCount );
	B3_ASSERT( tentativeManifoldCount <= triangleCount );
	B3_ASSERT( tentativeTriangleCount <= triangleCount );

	if ( shapeB->type == b3_sphereShape )
	{
		// Sort triangles so the closest are processed first, and therefore get
		// first claim on a shared edge or vertex.
		//
		// Upstream uses its QSORT macro. An insertion sort over at most
		// B3_NEA_MAX_MESH_CONTACT_TRIANGLES entries is both faster here and
		// *stable*, which quicksort is not: ties then keep ascending triangle
		// order rather than an order that depends on the pivot. Fixed point
		// produces more exact ties than float does, so this is the difference
		// between a deterministic claim order and one that is merely
		// reproducible on one library.
		for ( int i = 1; i < tentativeTriangleCount; ++i )
		{
			b3TentativeTriangle key = tentativeTriangles[i];
			int j = i - 1;
			while ( j >= 0 && tentativeTriangles[j].squaredDistance > key.squaredDistance )
			{
				tentativeTriangles[j + 1] = tentativeTriangles[j];
				j -= 1;
			}
			tentativeTriangles[j + 1] = key;
		}

		// Add tentative manifolds in sorted order, skipping the ones whose
		// feature a nearer triangle has already claimed.
		for ( int i = 0; i < tentativeTriangleCount; ++i )
		{
			b3LocalManifold* m = tentativeManifolds[tentativeTriangles[i].index];

			bool addedEdge1 = b3AddEdge( &foundEdges, m->i1, m->i2 );
			bool addedEdge2 = b3AddEdge( &foundEdges, m->i2, m->i3 );
			bool addedEdge3 = b3AddEdge( &foundEdges, m->i3, m->i1 );
			bool addedVertex1 = b3AddVertex( &foundVertices, m->i1 );
			bool addedVertex2 = b3AddVertex( &foundVertices, m->i2 );
			bool addedVertex3 = b3AddVertex( &foundVertices, m->i3 );

			bool shouldCollide = false;
			switch ( m->feature )
			{
				case b3_featureEdge1:
					shouldCollide = addedEdge1;
					break;

				case b3_featureEdge2:
					shouldCollide = addedEdge2;
					break;

				case b3_featureEdge3:
					shouldCollide = addedEdge3;
					break;

				case b3_featureVertex1:
					shouldCollide = addedVertex1;
					break;

				case b3_featureVertex2:
					shouldCollide = addedVertex2;
					break;

				case b3_featureVertex3:
					shouldCollide = addedVertex3;
					break;

				default:
					// b3_featureNone and b3_featureTriangleFace never reach the
					// tentative list; b3_featureHullFace does not either,
					// because the sphere path cannot produce it.
					B3_ASSERT( false );
					break;
			}

			if ( shouldCollide )
			{
				acceptedManifolds[acceptedManifoldCount++] = m;
			}
		}
	}
	else
	{
		// A hull can tunnel if the time of impact lands on a concave edge --
		// a flat box sliding down a ramp onto a flat bottom is the case. So
		// only *flat* edges are ignored, and only when a neighbour claimed
		// them; a concave edge keeps its contact.
		for ( int i = 0; i < tentativeManifoldCount; ++i )
		{
			b3LocalManifold* m = tentativeManifolds[i];
			int triangleFlags = m->triangleFlags;

			if ( ( triangleFlags & b3_allFlatEdges ) == b3_allFlatEdges )
			{
				continue;
			}

			if ( ( triangleFlags & b3_flatEdge1 ) == b3_flatEdge1 && b3FindEdge( &foundEdges, m->i1, m->i2 ) )
			{
				continue;
			}

			if ( ( triangleFlags & b3_flatEdge2 ) == b3_flatEdge2 && b3FindEdge( &foundEdges, m->i2, m->i3 ) )
			{
				continue;
			}

			if ( ( triangleFlags & b3_flatEdge3 ) == b3_flatEdge3 && b3FindEdge( &foundEdges, m->i3, m->i1 ) )
			{
				continue;
			}

			acceptedManifolds[acceptedManifoldCount++] = m;
		}
	}

	B3_ASSERT( acceptedManifoldCount <= triangleCount );

	if ( acceptedManifoldCount == 0 )
	{
		// Upstream frees the manifolds here. This port keeps them: they were
		// taken at contact creation and are returned at contact destruction,
		// so a body sliding on and off the level costs the allocator nothing.
		contact->manifoldCount = 0;
		return false;
	}

	b3Cluster* clusters = (b3Cluster*)b3Bump( &arena, acceptedManifoldCount * (int)sizeof( b3Cluster ) );
	int* clusterMemberships = (int*)b3Bump( &arena, acceptedManifoldCount * (int)sizeof( int ) );

	int clusterCount = 0;
	int clusterPointCount = 0;
	for ( int i = 0; i < acceptedManifoldCount; ++i )
	{
		clusterMemberships[i] = B3_NULL_INDEX;

		const b3LocalManifold* manifold = acceptedManifolds[i];
		clusterPointCount += manifold->pointCount;

		// Cluster on both the contact normal and the triangle normal. The
		// first cluster within tolerance wins, because the tolerance is tight.
		// Upstream's two todos -- consider requiring the triangles to share an
		// edge, consider looking for the best cluster rather than the first --
		// stand, and its #if 0'd edge-connectivity experiment is dropped rather
		// than carried, since b3TrianglesShareEdge has no other caller.
		b3Vec3 manifoldNormal = manifold->normal;
		b3Vec3 triangleNormal = manifold->triangleNormal;
		int clusterIndex = B3_NULL_INDEX;

		for ( int j = 0; j < clusterCount; ++j )
		{
			int64_t cosManifoldAngle = b3DotWide( clusters[j].manifoldNormal, manifoldNormal );
			int64_t cosTriangleAngle = b3DotWide( clusters[j].triangleNormal, triangleNormal );
			if ( cosManifoldAngle <= B3_CLUSTER_THRESHOLD || cosTriangleAngle <= B3_CLUSTER_THRESHOLD )
			{
				continue;
			}

			clusterIndex = j;
			break;
		}

		if ( clusterIndex != B3_NULL_INDEX )
		{
			clusterMemberships[i] = clusterIndex;
			clusters[clusterIndex].pointCapacity += manifold->pointCount;
		}
		else
		{
			clusters[clusterCount].manifoldNormal = manifoldNormal;
			clusters[clusterCount].triangleNormal = triangleNormal;
			clusters[clusterCount].pointCapacity = manifold->pointCount;
			clusterMemberships[i] = clusterCount;
			clusterCount += 1;
		}
	}

	if ( clusterPointCount == 0 )
	{
		// Unreachable -- every accepted manifold has at least one point -- and
		// upstream returns here without zeroing anything, which would trip its
		// caller's assert. Correct by construction under fixed capacity.
		contact->manifoldCount = 0;
		return false;
	}

	// Setup clusters
	b3LocalManifoldPoint* clusterPoints =
		(b3LocalManifoldPoint*)b3Bump( &arena, clusterPointCount * (int)sizeof( b3LocalManifoldPoint ) );
	int pointOffset = 0;

	for ( int i = 0; i < clusterCount; ++i )
	{
		b3Cluster* cluster = clusters + i;
		cluster->points = clusterPoints + pointOffset;
		cluster->pointCount = 0;
		pointOffset += cluster->pointCapacity;
	}

	// Populate clusters
	for ( int i = 0; i < acceptedManifoldCount; ++i )
	{
		int clusterIndex = clusterMemberships[i];
		B3_ASSERT( 0 <= clusterIndex && clusterIndex < clusterCount );

		b3LocalManifold* am = acceptedManifolds[i];
		b3Cluster* cm = clusters + clusterIndex;
		for ( int j = 0; j < am->pointCount; ++j )
		{
			B3_ASSERT( cm->pointCount < cm->pointCapacity );
			b3LocalManifoldPoint* ap = am->points + j;
			b3LocalManifoldPoint* cp = cm->points + cm->pointCount;

			cp->triangleIndex = am->triangleIndex;
			cp->point = ap->point;
			cp->separation = ap->separation;
			cp->pair = ap->pair;
			cm->pointCount += 1;
		}
	}

	// Simplify clusters, and record how deep each one reaches. The depth is
	// what the cap below selects on, and it is not known until now.
	for ( int i = 0; i < clusterCount; ++i )
	{
		b3Cluster* cm = clusters + i;
		B3_ASSERT( cm->pointCount == cm->pointCapacity );
		cm->pointCount = b3ReduceCluster( cm->points, cm->pointCount, cm->triangleNormal, arena );

		b3f minSeparation = cm->points[0].separation;
		for ( int j = 1; j < cm->pointCount; ++j )
		{
			minSeparation = b3MinF( minSeparation, cm->points[j].separation );
		}
		cm->minSeparation = minSeparation;
	}

	// The cap, which is the port's rule and has no upstream counterpart:
	// upstream allocates one manifold per cluster inside the step, and this
	// port allocates B3_NEA_MAX_MESH_MANIFOLDS once at contact creation. Over
	// the cap, keep the deepest clusters -- a dropped shallow one costs at
	// worst a speculative contact that never touched, a dropped deep one is a
	// body sinking into the level. Selection sort, because it stops after the
	// first `cap` elements and the array is at most 32 long.
	if ( clusterCount > B3_NEA_MAX_MESH_MANIFOLDS )
	{
		world->meshManifoldDropCount += clusterCount - B3_NEA_MAX_MESH_MANIFOLDS;

		// Loud in a debug build, counted in a release one. A scene that trips
		// this wants a larger cap or a coarser level, and either way should not
		// find out by watching bodies sink.
		B3_ASSERT( false );

		for ( int i = 0; i < B3_NEA_MAX_MESH_MANIFOLDS; ++i )
		{
			int deepest = i;
			for ( int j = i + 1; j < clusterCount; ++j )
			{
				if ( b3Raw( clusters[j].minSeparation ) < b3Raw( clusters[deepest].minSeparation ) )
				{
					deepest = j;
				}
			}

			if ( deepest != i )
			{
				b3Cluster tmp = clusters[i];
				clusters[i] = clusters[deepest];
				clusters[deepest] = tmp;
			}
		}

		clusterCount = B3_NEA_MAX_MESH_MANIFOLDS;
	}

	// Make a temporary copy of previous manifolds
	int oldManifoldCount = contact->manifoldCount;
	b3Manifold* oldManifolds = NULL;
	if ( oldManifoldCount > 0 )
	{
		oldManifolds = (b3Manifold*)b3Bump( &arena, oldManifoldCount * (int)sizeof( b3Manifold ) );
		memcpy( oldManifolds, contact->manifolds, oldManifoldCount * sizeof( b3Manifold ) );
	}

	// Upstream reallocates whenever the cluster count changes. Here the block
	// is already the right size, so only the entries about to be filled are
	// zeroed -- the copy above is what carries the warm-start state across.
	memset( contact->manifolds, 0, clusterCount * sizeof( b3Manifold ) );
	contact->manifoldCount = clusterCount;

	bool* consumed = NULL;
	if ( oldManifoldCount > 0 )
	{
		consumed = (bool*)b3Bump( &arena, oldManifoldCount * (int)sizeof( bool ) );
		memset( consumed, 0, oldManifoldCount * sizeof( bool ) );
	}

	b3Matrix3 matrixB = b3MakeMatrixFromQuat( xfB.q );
	b3Vec3 offsetA = b3SubPos( xfB.p, xfA.p );

	for ( int i = 0; i < clusterCount; ++i )
	{
		b3Cluster* cm = clusters + i;
		int pointCount = cm->pointCount;
		B3_ASSERT( 0 < pointCount && pointCount <= B3_MAX_MANIFOLD_POINTS );

		b3Manifold* manifold = contact->manifolds + i;
		manifold->pointCount = pointCount;

		// From the quaternion, not through matrixB. b3MakeMatrixFromQuat is
		// Q12 and not exactly orthonormal, and the contact normal is the axis
		// the solver forms its effective mass about -- the same argument
		// b3ComputeConvexManifold makes for its normal.
		b3Vec3 clusterNormal = b3RotateVector( xfB.q, cm->manifoldNormal );
		manifold->normal = clusterNormal;

		int64_t bestDot = B3_NORMAL_MATCH_TOLERANCE;
		int bestIndex = B3_NULL_INDEX;

		for ( int j = 0; j < oldManifoldCount; ++j )
		{
			if ( consumed[j] )
			{
				continue;
			}

			int64_t dot = b3DotWide( oldManifolds[j].normal, clusterNormal );
			if ( dot > bestDot )
			{
				bestIndex = j;
				bestDot = dot;
			}
		}

		b3Manifold* matchedManifold = NULL;
		if ( bestIndex != B3_NULL_INDEX )
		{
			matchedManifold = oldManifolds + bestIndex;
			manifold->frictionImpulse = matchedManifold->frictionImpulse;
			manifold->rollingImpulse = matchedManifold->rollingImpulse;
			manifold->twistImpulse = matchedManifold->twistImpulse;
			consumed[bestIndex] = true;
		}

		for ( int j = 0; j < pointCount; ++j )
		{
			const b3LocalManifoldPoint* source = cm->points + j;
			b3ManifoldPoint* target = manifold->points + j;

			// Contact points are computed in frame B, where the convex path
			// computes them in frame A. b3UpdateContact re-anchors both to the
			// centres of mass afterwards.
			target->anchorB = b3MulMV( matrixB, source->point );
			target->anchorA = b3Add( target->anchorB, offsetA );
			target->separation = b3SubF( source->separation, restOffset );
			target->featureId = b3MakeFeatureId( source->pair );
			target->triangleIndex = source->triangleIndex;
			target->normalVelocity = b3f_zero;

			// Preserve the normal impulse if this point existed last step. The
			// triangle index is part of the identity: the same feature id on a
			// different triangle is a different contact point.
			if ( matchedManifold != NULL )
			{
				int oldPointCount = matchedManifold->pointCount;
				for ( int k = 0; k < oldPointCount; ++k )
				{
					b3ManifoldPoint* oldPt = matchedManifold->points + k;

					if ( target->featureId == oldPt->featureId && target->triangleIndex == oldPt->triangleIndex )
					{
						target->normalImpulse = oldPt->normalImpulse;
						target->persisted = true;

						// Claimed, so a second new point cannot match it.
						oldPt->triangleIndex = B3_NULL_INDEX;
						break;
					}
				}
			}
		}
	}

	// Materials.
	//
	// Upstream averages a per-triangle friction and restitution over every
	// contact point when the mesh has more than one material. Every shape in
	// this port has exactly one (shape.c), so that average is the average of N
	// copies of one number and is *identically* the single-material branch
	// below -- not an approximation of it. The blob does carry a per-triangle
	// material index array, which the baker writes and nothing reads; giving it
	// a consumer is an API change, not a narrow-phase one.
	const b3SurfaceMaterial* materialsA = b3GetShapeMaterials( shapeA );
	const b3SurfaceMaterial* materialB = b3GetShapeMaterials( shapeB );

	contact->friction = world->frictionCallback( materialsA[0].friction, materialsA[0].userMaterialId, materialB->friction,
												 materialB->userMaterialId );
	contact->restitution = world->restitutionCallback( materialsA[0].restitution, materialsA[0].userMaterialId,
													   materialB->restitution, materialB->userMaterialId );

	b3Vec3 tangentVelocityA = b3RotateVector( xfA.q, materialsA[0].tangentVelocity );

	// Only shape B contributes a radius: a triangle soup has none. Note this
	// takes the hull's full innerRadius where the convex path takes a quarter
	// of it -- transliterated, because the two are upstream's own numbers.
	b3f radiusB = b3f_zero;
	if ( shapeB->type == b3_sphereShape )
	{
		radiusB = shapeB->sphere.radius;
	}
	else if ( shapeB->type == b3_capsuleShape )
	{
		radiusB = shapeB->capsule.radius;
	}
	else if ( shapeB->type == b3_hullShape )
	{
		radiusB = shapeB->hull->innerRadius;
	}

	contact->rollingResistance = b3MulFC( radiusB, materialB->rollingResistance );

	b3Vec3 tangentVelocityB = b3RotateVector( xfB.q, materialB->tangentVelocity );
	contact->tangentVelocity = b3Sub( tangentVelocityA, tangentVelocityB );
	return true;
}

int b3MeshContactArenaDemand( void )
{
	const int triangles = B3_NEA_MAX_MESH_CONTACT_TRIANGLES;
	const int points = B3_NEA_MAX_POINTS_PER_TRIANGLE * B3_NEA_MAX_MESH_CONTACT_TRIANGLES;
	const int manifolds = B3_NEA_MAX_MESH_MANIFOLDS;

	// b3Bump aligns each allocation, so every entry is rounded up. Counted
	// rather than estimated, because an arena that overflows costs a heap
	// allocation mid-step -- which under NEA_Phys3D's sealed pool is an assert,
	// not a slow frame.
	const int slack = B3_ARENA_ALIGNMENT - 1;

	// b3RefreshCache, released before the builder starts.
	int refresh = ( triangles * (int)sizeof( int ) + slack ) + ( triangles * (int)sizeof( b3ContactCache ) + slack );

	// b3ComputeMeshManifolds, all live at once.
	int builder = 2 * ( triangles * (int)sizeof( b3LocalManifold* ) + slack );
	builder += triangles * (int)sizeof( b3TentativeTriangle ) + slack;
	builder += points * (int)sizeof( b3LocalManifoldPoint ) + slack;	// pointBuffer
	builder += triangles * (int)sizeof( b3LocalManifold ) + slack;		// manifoldBuffer
	builder += triangles * (int)sizeof( b3Cluster ) + slack;
	builder += triangles * (int)sizeof( int ) + slack;					// clusterMemberships
	builder += points * (int)sizeof( b3LocalManifoldPoint ) + slack;	// clusterPoints
	builder += points * (int)sizeof( b3Point2D ) + slack;				// b3ReduceCluster, nested
	builder += manifolds * (int)sizeof( b3Manifold ) + slack;			// oldManifolds
	builder += manifolds * (int)sizeof( bool ) + slack;					// consumed

	return refresh > builder ? refresh : builder;
}
