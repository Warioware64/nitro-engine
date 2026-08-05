// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Erin Catto        (original Box3D)
// Copyright (c) 2026 Warioware64       (Nitro Engine Advanced fixed-point port)
//
// This file is part of Nitro Engine Advanced

/// @file   triangle_manifold.c
/// @brief  Contact manifolds between a triangle and a convex shape.
///
/// The three entry points the mesh narrow phase dispatches to --
/// b3CollideTriangleAndSphere, b3CollideTriangleAndCapsule and
/// b3CollideTriangleAndHull -- plus the closest-point routine the first of
/// them needs.
///
/// @section reuse What this file does not contain
///
/// Nearly every fixed-point decision here was already made for the convex
/// manifolds and is reused rather than re-derived. A triangle is a 3-vertex,
/// single-face hull, so the shapes line up almost one to one:
///
///   b3NormalOrUp             the degenerate-direction answer
///   b3MidpointContact        a contact point between two surfaces
///   b3SeparationFromWide     the one place a Q24 separation narrows
///   b3SignedAreaWide         a wide signed area
///   b3ReduceManifoldPoints   the four-point reducer and its 0.95 bias
///   b3ClipSegment            segment against a plane
///   b3ClipPolygon            polygon against a plane
///   b3FindIncidentFace       the most-opposed face of a hull
///
/// All eight live in manifold.{c,h}. The first six were static in
/// convex_manifold.c until this file needed them; the move is the only change
/// they saw.
///
/// @section tolerances The constants that are not transliteration
///
/// Upstream's `squaredTolerance = 0.005f * 0.005f` (four sites) is the same
/// near-parallel-edge reject the hull SAT already makes, and reuses the same
/// derivation: B3_PARALLEL_EDGE_TOL squared, 26843 at Q30. Note the 0.005
/// there is a dimensionless *sine*, not the 0.005 metres of B3_LINEAR_SLOP --
/// they are numerically equal and must not share a constant.
///
/// Upstream's `100.0f * FLT_EPSILON` shallow/deep split and its
/// `1000.0f * FLT_MIN` denormal guards both sit far below anything Q12 can
/// represent; the port's answer to both is "the distance is exactly zero",
/// which is what b3NormalOrUp encodes and what convex_manifold.c already
/// records at its two GJK sites.

#include "box3d/collision.h"
#include "box3d/constants.h"
#include "box3d/math_fixed.h"
#include "box3d/types.h"

#include "core.h"
#include "manifold.h"
#include "mesh.h"

#include <string.h>

// =========================================================================
// Closest point on a triangle
// =========================================================================

/// The point of triangle abc closest to q, and which feature it lies on.
///
/// Ericson's barycentric-region routine, unchanged in structure. What changes
/// is the arithmetic: the region tests are signs of `d1*d4 - d3*d2` and its
/// two siblings, which are products of dot products and therefore **degree
/// four** in the coordinates. Upstream reads them as floats and never
/// notices; here they have to be carried wide or they wrap.
///
/// @section range Why the shift is computed rather than fixed
///
/// The hull SAT solves the same overflow by shifting its Q24 products down to
/// Q12 before multiplying, and that pre-shift is correct *there* because its
/// operands are large -- a hull edge dot reaches 2e11. A triangle's are not.
/// A level triangle with 0.05-unit edges gives dots in the tens of thousands
/// at Q24, and shifting those down twelve bits leaves one significant digit:
/// the first version of this function did exactly that and produced normals
/// tilted by 3 to 30 degrees on small triangles, against a reference that had
/// them exactly vertical.
///
/// So the shift comes from the operands. All six dots are scaled down by one
/// common amount, chosen so the largest fits in 2^30 and a product of two
/// therefore fits int64. A common shift is what makes it safe: it changes
/// every product by the same factor, so the *signs* the region tests read and
/// the *ratios* the interior case divides are both exact, and for a small
/// triangle the shift is zero and nothing is lost at all.
///
/// This is also why all six dots are computed before the first region test
/// rather than staged the way upstream stages them -- the shift has to see
/// all of them. Six dots against upstream's two-to-six is a few multiplies on
/// a path that already ran GJK's worth of work to get here.
///
/// The four divisions are ratios whose denominators provably dominate their
/// numerators, so they go through b3DivWideToC and land as Q30 coefficients
/// in [0, 1] with no narrowing on the way.
b3TrianglePoint b3ClosestPointOnTriangle( b3Vec3 a, b3Vec3 b, b3Vec3 c, b3Vec3 q )
{
	b3Vec3 ab = b3Sub( b, a );
	b3Vec3 ac = b3Sub( c, a );
	b3Vec3 aq = b3Sub( q, a );
	b3Vec3 bq = b3Sub( q, b );
	b3Vec3 cq = b3Sub( q, c );

	int64_t d1 = b3DotWide( ab, aq );
	int64_t d2 = b3DotWide( ac, aq );
	int64_t d3 = b3DotWide( ab, bq );
	int64_t d4 = b3DotWide( ac, bq );
	int64_t d5 = b3DotWide( ab, cq );
	int64_t d6 = b3DotWide( ac, cq );

	// Vertex region of A.
	if ( d1 <= 0 && d2 <= 0 )
	{
		return ( b3TrianglePoint ){ a, b3_featureVertex1 };
	}

	// Vertex region of B.
	if ( d3 > 0 && d4 <= d3 )
	{
		return ( b3TrianglePoint ){ b, b3_featureVertex2 };
	}

	// Vertex region of C.
	if ( d6 >= 0 && d5 <= d6 )
	{
		return ( b3TrianglePoint ){ c, b3_featureVertex3 };
	}

	// One shift for all six, so signs and ratios both survive it.
	int64_t maxAbs = 0;
	{
		const int64_t all[6] = { d1, d2, d3, d4, d5, d6 };
		for ( int i = 0; i < 6; ++i )
		{
			int64_t m = all[i] < 0 ? -all[i] : all[i];
			maxAbs = m > maxAbs ? m : maxAbs;
		}
	}

	int shift = 0;
	if ( maxAbs > ( (int64_t)1 << 30 ) )
	{
		// 64 - 30 - clz is the number of bits above 2^30, which is exactly
		// what has to come off.
		shift = 34 - b3Clz64( (uint64_t)maxAbs );
	}

	int64_t n1 = d1 >> shift;
	int64_t n2 = d2 >> shift;
	int64_t n3 = d3 >> shift;
	int64_t n4 = d4 >> shift;
	int64_t n5 = d5 >> shift;
	int64_t n6 = d6 >> shift;

	// Edge region AB.
	int64_t vc = n1 * n4 - n3 * n2;
	if ( vc <= 0 && d1 >= 0 && d3 <= 0 )
	{
		b3c t = b3DivWideToC( d1, d1 - d3 );
		return ( b3TrianglePoint ){ b3Add( a, b3MulCV( t, ab ) ), b3_featureEdge1 };
	}

	// Edge region AC.
	int64_t vb = n5 * n2 - n1 * n6;
	if ( vb <= 0 && d2 >= 0 && d6 <= 0 )
	{
		b3c t = b3DivWideToC( d2, d2 - d6 );
		return ( b3TrianglePoint ){ b3Add( a, b3MulCV( t, ac ) ), b3_featureEdge3 };
	}

	// Edge region BC.
	int64_t va = n3 * n6 - n5 * n4;
	if ( va <= 0 && d4 >= d3 && d5 >= d6 )
	{
		b3Vec3 bc = b3Sub( c, b );
		b3c t = b3DivWideToC( d4 - d3, ( d4 - d3 ) + ( d5 - d6 ) );
		return ( b3TrianglePoint ){ b3Add( b, b3MulCV( t, bc ) ), b3_featureEdge2 };
	}

	// Inside the face. va, vb and vc share a shift, so the ratios are exact.
	int64_t denom = va + vb + vc;
	if ( denom == 0 )
	{
		// A degenerate triangle: every barycentric weight is zero and there is
		// no interior to be inside of. Upstream divides anyway and gets an
		// infinity; the honest answer here is the first vertex.
		return ( b3TrianglePoint ){ a, b3_featureVertex1 };
	}

	b3c t1 = b3DivWideToC( vb, denom );
	b3c t2 = b3DivWideToC( vc, denom );

	b3Vec3 p = b3Add( a, b3MulCV( t1, ab ) );
	p = b3Add( p, b3MulCV( t2, ac ) );
	return ( b3TrianglePoint ){ p, b3_featureTriangleFace };
}

// =========================================================================
// Simplex feature
// =========================================================================

/// Which triangle feature a GJK simplex settled on, by vertex bitmask.
///
/// Index 0 is unreachable: b3ShapeDistance never returns an empty simplex.
static const b3TriangleFeature s_triangleFeatures[8] = {
	b3_featureNone,			// 000
	b3_featureVertex1,		// 001
	b3_featureVertex2,		// 010
	b3_featureEdge1,		// 011  v1-v2
	b3_featureVertex3,		// 100
	b3_featureEdge3,		// 101  v3-v1
	b3_featureEdge2,		// 110  v2-v3
	b3_featureTriangleFace, // 111
};

/// Read the triangle feature out of a simplex cache.
///
/// Pure integer, and the only coupling between this file and b3SimplexCache:
/// the cache's indexA[] are vertex indices into the 3-point triangle proxy,
/// so a bitmask over them names an edge, a vertex or the face.
static b3TriangleFeature b3GetTriangleFeature( const b3SimplexCache* cache )
{
	int count = (int)cache->count;
	B3_ASSERT( 0 < count && count < 4 );

	int mask = 0;
	for ( int i = 0; i < count; ++i )
	{
		B3_ASSERT( cache->indexA[i] < 3 );
		mask |= 1 << cache->indexA[i];
	}

	B3_ASSERT( 0 < mask && mask < 8 );
	return s_triangleFeatures[mask];
}

// =========================================================================
// Triangle versus sphere
// =========================================================================

void b3CollideTriangleAndSphere( b3LocalManifold* manifold, int capacity, const b3Vec3* triangleA, const b3Sphere* sphereB )
{
	manifold->pointCount = 0;

	if ( capacity == 0 )
	{
		return;
	}

	b3Vec3 center = sphereB->center;
	b3Vec3 v1 = triangleA[0];
	b3Vec3 v2 = triangleA[1];
	b3Vec3 v3 = triangleA[2];

	b3Plane plane = b3MakePlaneFromPoints( v1, v2, v3 );

	// Cull the back side. A mesh triangle is one-sided by construction: a
	// contact from underneath is a body that has already tunnelled, and
	// answering it would push the body further through.
	if ( b3Raw( b3PlaneSeparation( plane, center ) ) < 0 )
	{
		return;
	}

	b3f radius = sphereB->radius;

	b3TrianglePoint closest = b3ClosestPointOnTriangle( v1, v2, v3, center );

	// Reject squared against squared, so the square root below is only ever
	// reached for a pair that is actually close. maxDistance squared does not
	// fit b3f -- 82 + a radius, squared, is past Q12 for any real radius --
	// so the comparison is wide.
	int64_t squaredDistance = b3LengthSquaredWide( b3Sub( closest.point, center ) );
	int64_t maxDistance = (int64_t)b3Raw( radius ) + b3Raw( B3_SPECULATIVE_DISTANCE );
	if ( squaredDistance > maxDistance * maxDistance )
	{
		return;
	}

	b3f distance = b3SqrtWide( squaredDistance );

	// Normal points from the triangle to the sphere. When the centre lies
	// exactly on the triangle there is no direction to take from the offset,
	// and the face normal is the answer -- upstream's 1000*FLT_MIN guard,
	// which in fixed point is a test against zero.
	b3Vec3 normal;
	if ( squaredDistance == 0 )
	{
		normal = plane.normal;
	}
	else
	{
		normal = b3NormalOrUp( b3Sub( center, closest.point ), squaredDistance );
	}

	manifold->normal = normal;
	manifold->pointCount = 1;
	manifold->feature = closest.feature;

	// Wide, and stays wide: this is what the mesh narrow phase sorts its
	// sphere-versus-mesh candidates on, and narrowing it to Q12 would collapse
	// the ordering for triangles closer than a quantum apart.
	manifold->squaredDistance = squaredDistance;

	b3LocalManifoldPoint* mp = manifold->points + 0;
	mp->point = b3MidpointContact( closest.point, b3f_zero, center, radius, normal );
	mp->separation = b3SubF( distance, radius );
	mp->pair = b3FeaturePair_single;
	mp->triangleIndex = B3_NULL_INDEX;
}

// =========================================================================
// Triangle versus capsule
// =========================================================================

/// tol^2 = 0.005^2 = 2.5e-5, which is 26843 at Q30.
///
/// The same near-parallel-edge reject the hull SAT makes, and the same
/// literal -- B3_PARALLEL_EDGE_TOL squared. Note the 0.005 here is a
/// dimensionless *sine*, not the 0.005 metres of B3_LINEAR_SLOP: the two are
/// numerically equal and mean different things, and this one must not scale
/// with the length unit.
#define B3_TRIANGLE_PARALLEL_TOL_SQ ( (int64_t)26843 )

/// Are two edge directions within about 0.29 degrees of parallel?
///
/// `a` and `b` are the edge projected onto two orthonormal directions, so
/// `a^2 + b^2` is the squared sine of the angle times the squared edge
/// length. Both sides go wide, and the squared length comes down six bits
/// first: 7.7e14 for the longest Q12 vector times 26843 is 2.1e19, past
/// int64, which is the same pre-shift b3ComputeSeparatingAxis records.
static bool b3IsNearlyParallel( b3f a, b3f b, b3Vec3 edge )
{
	int64_t ra = b3Raw( a );
	int64_t rb = b3Raw( b );
	int64_t lhs = ra * ra + rb * rb;

	int64_t limit = ( ( b3LengthSquaredWide( edge ) >> 6 ) * B3_TRIANGLE_PARALLEL_TOL_SQ ) >> ( B3_C_SHIFT - 6 );
	return lhs < limit;
}

/// The outward side normal of triangle edge v1->v2.
///
/// b3CrossDirection, not b3Cross: the edges of a level triangle are short and
/// a narrow cross product of two short vectors underflows to nothing. The
/// result is then normalized because its magnitude is read -- b3IsNearlyParallel
/// compares a projection onto it against an absolute tolerance.
static b3Vec3 b3TriangleSideNormal( b3Vec3 edge, b3Vec3 faceNormal )
{
	return b3Normalize( b3CrossDirection( edge, faceNormal ) );
}

/// Clip a segment against the three side planes of a triangle.
///
/// The side planes are deliberately **not** normalized. b3PlaneSeparation
/// against them is read only through a sign and through the ratio
/// d1 / (d1 - d2), and both are invariant under scaling the plane's normal --
/// the same argument b3BuildFaceAContact makes, and it saves three square
/// roots per call.
///
/// @return false if the segment ever fails to leave exactly two points, which
/// means it does not cross the triangle's interior at all.
static bool b3ClipSegmentToTriangleFace( b3ClipVertex segment[2], const b3Vec3* points, b3Plane plane )
{
	b3Vec3 vertex1 = points[2];

	for ( int i = 0; i < 3; ++i )
	{
		b3Vec3 vertex2 = points[i];
		b3Vec3 binormal = b3CrossDirection( b3Sub( vertex2, vertex1 ), plane.normal );
		b3Plane clipPlane = b3MakePlaneFromNormalAndPoint( binormal, vertex1 );

		if ( b3ClipSegment( segment, clipPlane ) != 2 )
		{
			return false;
		}

		vertex1 = vertex2;
	}

	return true;
}

/// The triangle's own plane, scored against the deeper capsule centre.
static b3SeparatingAxis b3QueryTriangleFaceAndCapsule( b3Plane plane, const b3Capsule* capsule )
{
	b3f separation1 = b3PlaneSeparation( plane, capsule->center1 );
	b3f separation2 = b3PlaneSeparation( plane, capsule->center2 );

	b3SeparatingAxis axis;
	axis.normal = plane.normal;
	axis.type = b3_faceAxisA;
	axis.indexA = 0;

	if ( b3Raw( separation1 ) < b3Raw( separation2 ) )
	{
		axis.separation = separation1;
		axis.indexB = 0;
	}
	else
	{
		axis.separation = separation2;
		axis.indexB = 1;
	}

	return axis;
}

/// The best of the three triangle-edge versus capsule-edge axes.
///
/// Each triangle edge is treated as a zero-area face carrying the outward
/// side normal, which is what gives an edge-edge axis a sense of "outward"
/// that a bare cross product does not have.
static b3SeparatingAxis b3QueryTriangleAndCapsuleEdges( const b3Vec3* vertices, b3Plane plane, const b3Capsule* capsule )
{
	b3Vec3 p1 = capsule->center1;
	b3Vec3 capsuleEdge = b3Sub( capsule->center2, p1 );

	b3SeparatingAxis result;
	result.normal = b3Vec3_zeroFn();
	result.separation = B3_F_MIN;
	result.indexA = B3_NULL_INDEX;
	result.indexB = B3_NULL_INDEX;
	result.type = b3_edgePairAxis;

	int edgeIndex = 2;
	b3Vec3 v1 = vertices[2];

	for ( int index = 0; index < 3; ++index )
	{
		b3Vec3 v2 = vertices[index];
		b3Vec3 sideNormal = b3TriangleSideNormal( b3Sub( v2, v1 ), plane.normal );

		b3f a = b3Dot( capsuleEdge, plane.normal );
		b3f b = b3Dot( capsuleEdge, sideNormal );

		// Parallel to this edge: the face contact already covers it, and the
		// axis derived below would be decided by quantization.
		if ( b3IsNearlyParallel( a, b, capsuleEdge ) )
		{
			v1 = v2;
			edgeIndex = index;
			continue;
		}

		// The same construction b3QueryEdgeDirections uses: the axis is the
		// blend of the two "face" normals that is perpendicular to the capsule
		// edge. t is a ratio whose denominator dominates, by the sign test.
		b3Vec3 axis;
		if ( (int64_t)b3Raw( a ) * b3Raw( b ) <= 0 )
		{
			b3c t = b3DivFFToC( b, b3SubF( b, a ) );
			axis = b3Lerp( sideNormal, plane.normal, t );
		}
		else
		{
			b3c t = b3DivFFToC( b, b3AddF( a, b ) );
			axis = b3Lerp( sideNormal, b3Neg( plane.normal ), t );
		}

		// A lerp of two unit vectors is zero only if they are exactly
		// opposite, which the parallel reject above has already excluded.
		axis = b3Normalize( axis );

		b3f separation = b3SeparationFromWide( b3DotWide( axis, b3Sub( p1, v1 ) ) );

		if ( b3Raw( separation ) > b3Raw( result.separation ) )
		{
			// No early exit on a separating axis: the best one is wanted even
			// when it separates, because the caller compares it against the
			// face axis and the capsule radius is not in it yet.
			result.normal = axis;
			result.separation = separation;
			result.indexA = edgeIndex;
			result.indexB = 0;
		}

		v1 = v2;
		edgeIndex = index;
	}

	return result;
}

/// Two points, from the capsule's segment clipped to the triangle's interior.
static void b3BuildTriangleAndCapsuleFaceContact( b3LocalManifold* manifold, const b3Vec3* triangle, b3Plane plane,
												  const b3Capsule* capsule )
{
	B3_ASSERT( manifold->pointCount == 0 );

	b3ClipVertex segment[2];
	segment[0].position = capsule->center1;
	segment[0].separation = b3f_zero;
	segment[0].pair = b3MakeFeaturePair( b3_featureShapeA, 0, b3_featureShapeA, 0 );
	segment[1].position = capsule->center2;
	segment[1].separation = b3f_zero;
	segment[1].pair = b3MakeFeaturePair( b3_featureShapeA, 1, b3_featureShapeA, 1 );

	if ( b3ClipSegmentToTriangleFace( segment, triangle, plane ) == false )
	{
		return;
	}

	b3f radius = capsule->radius;
	b3f distance1 = b3PlaneSeparation( plane, segment[0].position );
	b3f distance2 = b3PlaneSeparation( plane, segment[1].position );

	b3f reach = b3AddF( B3_SPECULATIVE_DISTANCE, radius );
	if ( b3Raw( distance1 ) > b3Raw( reach ) && b3Raw( distance2 ) > b3Raw( reach ) )
	{
		return;
	}

	// Half way between the capsule's surface and the triangle plane. Halved
	// with b3MulFC rather than a shift: an arithmetic shift of a negative
	// separation rounds toward minus infinity, and these are usually negative.
	b3c half = b3cFromFrac( 1, 2 );
	b3Vec3 point1 = b3MulSub( segment[0].position, b3MulFC( b3AddF( distance1, radius ), half ), plane.normal );
	b3Vec3 point2 = b3MulSub( segment[1].position, b3MulFC( b3AddF( distance2, radius ), half ), plane.normal );

	manifold->normal = plane.normal;
	manifold->feature = b3_featureTriangleFace;
	manifold->pointCount = 2;

	b3LocalManifoldPoint* pt = manifold->points + 0;
	pt->point = point1;
	pt->separation = b3SubF( distance1, radius );
	pt->pair = segment[0].pair;
	pt->triangleIndex = B3_NULL_INDEX;

	pt = manifold->points + 1;
	pt->point = point2;
	pt->separation = b3SubF( distance2, radius );
	pt->pair = segment[1].pair;
	pt->triangleIndex = B3_NULL_INDEX;
}

/// One point, where the capsule segment crosses a triangle edge.
static void b3BuildTriangleAndCapsuleEdgeContact( b3LocalManifold* manifold, const b3Vec3* triangle, b3Plane plane,
												  const b3Capsule* capsule, b3SeparatingAxis query )
{
	B3_ASSERT( 0 <= query.indexA && query.indexA < 3 );

	b3Vec3 p1 = capsule->center1;
	b3Vec3 capsuleEdge = b3Sub( capsule->center2, p1 );

	b3Vec3 v1 = triangle[query.indexA];
	b3Vec3 v2 = triangle[( query.indexA + 1 ) % 3];
	b3Vec3 triangleEdge = b3Sub( v2, v1 );

	b3Vec3 sideNormal = b3TriangleSideNormal( triangleEdge, plane.normal );

	b3f a = b3Dot( capsuleEdge, plane.normal );
	b3f b = b3Dot( capsuleEdge, sideNormal );

	if ( b3IsNearlyParallel( a, b, capsuleEdge ) )
	{
		return;
	}

	b3Vec3 normal = query.normal;
	b3SegmentDistanceResult result = b3LineDistance( v1, triangleEdge, p1, capsuleEdge );

	if ( b3IsWithinSegments( &result ) == false )
	{
		// The closest points ran off the ends of one of the segments, so this
		// is a vertex contact the face path owns rather than an edge one.
		return;
	}

	b3Vec3 point = b3Lerp( b3MulSub( result.point1, capsule->radius, normal ), result.point2, b3cFromFrac( 1, 2 ) );
	b3f separation = b3SeparationFromWide( b3DotWide( normal, b3Sub( p1, v1 ) ) );

	manifold->normal = normal;
	manifold->pointCount = 1;

	static const b3TriangleFeature edgeFeatures[3] = { b3_featureEdge1, b3_featureEdge2, b3_featureEdge3 };
	manifold->feature = edgeFeatures[query.indexA];

	b3LocalManifoldPoint* pt = manifold->points + 0;
	pt->point = point;
	pt->separation = b3SubF( separation, capsule->radius );
	pt->pair = b3MakeFeaturePair( b3_featureShapeA, query.indexA, b3_featureShapeB, query.indexB );
	pt->triangleIndex = B3_NULL_INDEX;
}

void b3CollideTriangleAndCapsule( b3LocalManifold* manifold, int capacity, const b3Vec3* triangleA,
								  const b3Capsule* capsuleB, b3SimplexCache* cache )
{
	manifold->pointCount = 0;

	if ( capacity < 2 )
	{
		return;
	}

	b3Vec3 v1 = triangleA[0];
	b3Vec3 v2 = triangleA[1];
	b3Vec3 v3 = triangleA[2];

	b3Plane plane = b3MakePlaneFromPoints( v1, v2, v3 );
	b3Vec3 capsuleCenter = b3Lerp( capsuleB->center1, capsuleB->center2, b3cFromFrac( 1, 2 ) );

	// Cull the back side, as the sphere path does.
	if ( b3Raw( b3PlaneSeparation( plane, capsuleCenter ) ) < 0 )
	{
		return;
	}

	b3DistanceInput distanceInput;
	distanceInput.proxyA = ( b3ShapeProxy ){ triangleA, 3, b3f_zero };
	distanceInput.proxyB = ( b3ShapeProxy ){ &capsuleB->center1, 2, b3f_zero };
	distanceInput.transform = b3Transform_identity;
	distanceInput.useRadii = false;

	b3DistanceOutput distanceOutput = b3ShapeDistance( &distanceInput, cache, NULL, 0 );

	b3f radius = capsuleB->radius;
	if ( b3Raw( distanceOutput.distance ) > b3Raw( b3AddF( radius, B3_SPECULATIVE_DISTANCE ) ) )
	{
		// Separated, and the cache is deliberately left alone so the next step
		// warm starts from the same simplex.
		return;
	}

	// Shallow versus deep. Upstream splits at 100 * FLT_EPSILON, which is a
	// twentieth of a Q12 quantum; the fixed-point statement of the same
	// question is whether the witness points are distinct at all, which is
	// what convex_manifold.c's two GJK sites already record.
	if ( b3Raw( distanceOutput.distance ) > 0 )
	{
		b3Vec3 delta = b3Normalize( b3Sub( distanceOutput.pointB, distanceOutput.pointA ) );

		// Two points if the closest-point direction is near enough the face
		// normal that the capsule is lying on the triangle rather than
		// touching it at one end. Both operands are unit, so the dot is a
		// cosine; taken wide at Q24 so the threshold means what it says.
		const int64_t kToleranceWide = 3355443; // 0.2 at Q24
		int64_t alignment = b3DotWide( plane.normal, delta );
		if ( ( alignment < 0 ? -alignment : alignment ) > kToleranceWide )
		{
			b3ClipVertex segment[2];
			segment[0].position = capsuleB->center1;
			segment[0].separation = b3f_zero;
			segment[0].pair = b3MakeFeaturePair( b3_featureShapeA, 0, b3_featureShapeA, 0 );
			segment[1].position = capsuleB->center2;
			segment[1].separation = b3f_zero;
			segment[1].pair = b3MakeFeaturePair( b3_featureShapeA, 1, b3_featureShapeA, 1 );

			if ( b3ClipSegmentToTriangleFace( segment, triangleA, plane ) )
			{
				b3f distance1 = b3PlaneSeparation( plane, segment[0].position );
				b3f distance2 = b3PlaneSeparation( plane, segment[1].position );

				b3c half = b3cFromFrac( 1, 2 );
				b3Vec3 point1 =
					b3MulSub( segment[0].position, b3MulFC( b3AddF( radius, distance1 ), half ), plane.normal );
				b3Vec3 point2 =
					b3MulSub( segment[1].position, b3MulFC( b3AddF( radius, distance2 ), half ), plane.normal );

				manifold->normal = plane.normal;
				manifold->feature = b3_featureTriangleFace;
				manifold->pointCount = 2;

				b3LocalManifoldPoint* mp = manifold->points + 0;
				mp->point = point1;
				mp->separation = b3SubF( distance1, radius );
				mp->pair = segment[0].pair;
				mp->triangleIndex = B3_NULL_INDEX;

				mp = manifold->points + 1;
				mp->point = point2;
				mp->separation = b3SubF( distance2, radius );
				mp->pair = segment[1].pair;
				mp->triangleIndex = B3_NULL_INDEX;
				return;
			}
		}

		// One point, from the closest points.
		manifold->normal = delta;
		manifold->pointCount = 1;
		manifold->feature = b3GetTriangleFeature( cache );

		b3LocalManifoldPoint* mp = manifold->points + 0;
		mp->point = b3MidpointContact( distanceOutput.pointA, b3f_zero, distanceOutput.pointB, radius, delta );
		mp->separation = b3SubF( distanceOutput.distance, radius );
		mp->pair = b3FeaturePair_single;
		mp->triangleIndex = B3_NULL_INDEX;
		return;
	}

	// Deep penetration: GJK has no direction to give, so fall back to SAT.
	b3SeparatingAxis faceQuery = b3QueryTriangleFaceAndCapsule( plane, capsuleB );
	if ( b3Raw( faceQuery.separation ) > b3Raw( radius ) )
	{
		return;
	}

	b3SeparatingAxis edgeQuery = b3QueryTriangleAndCapsuleEdges( triangleA, plane, capsuleB );
	if ( b3Raw( edgeQuery.separation ) > b3Raw( radius ) )
	{
		return;
	}

	b3f faceSeparation = b3SubF( faceQuery.separation, radius );
	b3BuildTriangleAndCapsuleFaceContact( manifold, triangleA, plane, capsuleB );
	B3_ASSERT( manifold->pointCount == 0 || manifold->pointCount == 2 );

	if ( manifold->pointCount == 2 )
	{
		// The clip knows something the axis query does not, so this becomes
		// the separation the edge axis is judged against.
		faceSeparation = b3MinF( manifold->points[0].separation, manifold->points[1].separation );
	}

	// The face contact can come out empty if it does not realize the axis of
	// least penetration. Take the edge contact then, or when the edge axis is
	// clearly better than what the clip produced.
	if ( edgeQuery.indexA == B3_NULL_INDEX )
	{
		return;
	}

	b3f edgeSeparation = b3SubF( edgeQuery.separation, radius );
	if ( manifold->pointCount == 0 || b3Raw( edgeSeparation ) > b3Raw( b3AddF( faceSeparation, B3_LINEAR_SLOP ) ) )
	{
		manifold->pointCount = 0;
		b3BuildTriangleAndCapsuleEdgeContact( manifold, triangleA, plane, capsuleB, edgeQuery );
	}
}

// =========================================================================
// Triangle versus hull
// =========================================================================

/// The triangle, packed once and passed to every helper below.
typedef struct
{
	b3Vec3 v1, v2, v3;
	b3Vec3 e1, e2, e3;
	b3Plane plane;
	int flags;
} b3TriangleData;

/// Index of the triangle vertex furthest along a direction.
static inline int b3GetTriangleSupport( const b3Vec3* points, b3Vec3 direction )
{
	int index = 0;
	int64_t distance = b3DotWide( points[0], direction );

	int64_t d = b3DotWide( points[1], direction );
	if ( d > distance )
	{
		distance = d;
		index = 1;
	}

	d = b3DotWide( points[2], direction );
	if ( d > distance )
	{
		return 2;
	}

	return index;
}

/// Axis A: the triangle's own plane, against the hull's support vertex.
static b3SeparatingAxis b3QueryTriangleFace( const b3TriangleData* triangle, const b3HullData* hull )
{
	const b3Vec3* hullPoints = b3GetHullPoints( hull );
	b3Plane plane = triangle->plane;

	int vertexIndex = b3FindHullSupportVertex( hull, b3Neg( plane.normal ) );

	b3SeparatingAxis axis;
	axis.normal = plane.normal;
	axis.separation = b3PlaneSeparation( plane, hullPoints[vertexIndex] );
	axis.indexA = 0;
	axis.indexB = vertexIndex;
	axis.type = b3_faceAxisA;
	return axis;
}

/// Axis B: every hull face, against the triangle's support vertex.
static b3SeparatingAxis b3QueryHullFace( const b3TriangleData* triangle, const b3HullData* hull )
{
	const b3Plane* hullPlanes = b3GetHullPlanes( hull );
	b3Vec3 trianglePoints[3] = { triangle->v1, triangle->v2, triangle->v3 };

	b3Vec3 maxNormal = b3Vec3_zeroFn();
	b3f maxSeparation = B3_F_MIN;
	int maxFaceIndex = B3_NULL_INDEX;
	int maxVertexIndex = B3_NULL_INDEX;

	for ( int faceIndex = 0; faceIndex < hull->faceCount; ++faceIndex )
	{
		b3Plane plane = hullPlanes[faceIndex];

		int vertexIndex = b3GetTriangleSupport( trianglePoints, b3Neg( plane.normal ) );
		b3f separation = b3PlaneSeparation( plane, trianglePoints[vertexIndex] );

		if ( b3Raw( separation ) > b3Raw( maxSeparation ) )
		{
			maxNormal = plane.normal;
			maxSeparation = separation;
			maxFaceIndex = faceIndex;
			maxVertexIndex = vertexIndex;
		}
	}

	b3SeparatingAxis axis;
	axis.normal = b3Neg( maxNormal ); // points from triangle to hull
	axis.separation = maxSeparation;
	axis.indexA = maxVertexIndex;
	axis.indexB = maxFaceIndex;
	axis.type = b3_faceAxisB;
	return axis;
}

/// The edge-pair axis for one hull edge against one triangle edge, or false
/// if the pair is not a Minkowski face or is too near parallel to trust.
///
/// Shared by the full query and the cache replay, which upstream writes out
/// twice; the two copies had already drifted apart there (one tests `>=`
/// where the other tests `<`), so they are one function here and the
/// difference is a parameter.
static bool b3TriangleHullEdgeAxis( b3Vec3* axisOut, b3f* separationOut, b3Vec3 triPoint, b3Vec3 triEdge,
									b3Vec3 triNormal, b3Vec3 hullPoint, b3Vec3 hullEdge, b3Vec3 hullNormal1,
									b3Vec3 hullNormal2 )
{
	b3f cab = b3Dot( hullNormal1, triEdge );
	b3f dab = b3Dot( hullNormal2, triEdge );
	b3f bcd = b3Dot( triNormal, hullEdge );

	// The Gauss / Minkowski face test, as sign products. Taken as signs rather
	// than as products because the products are squared lengths and would
	// overflow for no reason -- only the sign was ever wanted.
	int64_t cabRaw = b3Raw( cab );
	int64_t dabRaw = b3Raw( dab );
	int64_t bcdRaw = b3Raw( bcd );

	if ( cabRaw * dabRaw >= 0 || cabRaw * bcdRaw <= 0 )
	{
		return false;
	}

	// Near-parallel edges give a separation decided by quantization rather
	// than by geometry. Same reject, same literal, as the hull SAT.
	int64_t maxSq = cabRaw * cabRaw;
	int64_t otherSq = dabRaw * dabRaw;
	maxSq = otherSq > maxSq ? otherSq : maxSq;

	int64_t limit = ( ( b3LengthSquaredWide( triEdge ) >> 6 ) * B3_TRIANGLE_PARALLEL_TOL_SQ ) >> ( B3_C_SHIFT - 6 );
	if ( maxSq < limit )
	{
		return false;
	}

	// dot(hullNormal1 + t * (hullNormal2 - hullNormal1), triEdge) = 0.
	// The denominator dominates by the sign test above, so t is in [0, 1].
	b3c t = b3DivFFToC( cab, b3SubF( cab, dab ) );
	b3Vec3 axis = b3Lerp( hullNormal1, hullNormal2, t );

	if ( b3LengthSquaredWide( axis ) == 0 )
	{
		return false;
	}

	axis = b3Normalize( axis );

	*axisOut = axis;
	*separationOut = b3SeparationFromWide( b3DotWide( axis, b3Sub( triPoint, hullPoint ) ) );
	return true;
}

/// The best of the hull-edge versus triangle-edge axes.
static b3SeparatingAxis b3QueryTriangleAndHullEdges( const b3TriangleData* triangle, const b3HullData* hull )
{
	b3SeparatingAxis result;
	result.normal = b3Vec3_zeroFn();
	result.separation = B3_F_MIN;
	result.indexA = B3_NULL_INDEX;
	result.indexB = B3_NULL_INDEX;
	result.type = b3_edgePairAxis;

	b3Vec3 trianglePoints[3] = { triangle->v1, triangle->v2, triangle->v3 };
	b3Vec3 triangleEdges[3] = { triangle->e1, triangle->e2, triangle->e3 };
	b3Vec3 triNormal = triangle->plane.normal;

	// Upstream computes the triangle's concave-edge flags here and then casts
	// them to void -- the culling that would have used them is commented out
	// at its two sites. Kept as a comment rather than deleted, because the
	// live mechanism is the mesh narrow phase's ghost filter and a reader
	// needs to see that upstream tried it here first.
	B3_UNUSED( triangle->flags );

	const b3HullHalfEdge* hullEdges = b3GetHullEdges( hull );
	const b3Vec3* hullPoints = b3GetHullPoints( hull );
	const b3Plane* hullPlanes = b3GetHullPlanes( hull );

	for ( int i = 0; i < hull->edgeCount; i += 2 )
	{
		const b3HullHalfEdge* edge = hullEdges + i;
		const b3HullHalfEdge* twin = hullEdges + i + 1;
		B3_ASSERT( edge->twin == i + 1 && twin->twin == i );

		b3Vec3 hullPoint = hullPoints[edge->origin];
		b3Vec3 hullEdge = b3Sub( hullPoints[twin->origin], hullPoint );
		b3Vec3 hullNormal1 = hullPlanes[edge->face].normal;
		b3Vec3 hullNormal2 = hullPlanes[twin->face].normal;

		for ( int j = 0; j < 3; ++j )
		{
			b3Vec3 axis;
			b3f separation;

			if ( b3TriangleHullEdgeAxis( &axis, &separation, trianglePoints[j], triangleEdges[j], triNormal, hullPoint,
										 hullEdge, hullNormal1, hullNormal2 ) == false )
			{
				continue;
			}

			if ( b3Raw( separation ) > b3Raw( result.separation ) )
			{
				// No early exit even on a separating axis: the best one is
				// wanted for the cache.
				result.normal = b3Neg( axis ); // points from triangle to hull
				result.separation = separation;
				result.indexA = j;
				result.indexB = i;
			}
		}
	}

	return result;
}

/// Contact points with the hull face as reference and the triangle clipped
/// against it.
///
/// @return the clipped separation, which the caller weighs against the edge
/// axis. Upstream returns FLT_MAX from the sibling below to mean "nothing";
/// this one always returns a real separation.
static b3f b3CollideHullFace( b3LocalManifold* manifold, int pointCapacity, const b3TriangleData* triangle,
							  const b3HullData* hull, b3SeparatingAxis query, b3SATCache* cache, bool enableSpeculative )
{
	B3_ASSERT( query.type == b3_faceAxisB );
	B3_ASSERT( 0 <= query.indexA && query.indexA < 3 );
	B3_ASSERT( 0 <= query.indexB && query.indexB < hull->faceCount );

	manifold->pointCount = 0;

	const b3HullFace* hullFaces = b3GetHullFaces( hull );
	const b3HullHalfEdge* hullEdges = b3GetHullEdges( hull );
	const b3Plane* hullPlanes = b3GetHullPlanes( hull );
	const b3Vec3* hullPoints = b3GetHullPoints( hull );

	b3Plane refPlane = hullPlanes[query.indexB];

	b3ClipVertex buffer1[B3_MAX_CLIP_POINTS];
	b3ClipVertex buffer2[B3_MAX_CLIP_POINTS];

	// The triangle is the incident face here, so its vertices carry shape-B
	// feature owners and the whole pair is flipped on the way out.
	buffer1[0].position = triangle->v1;
	buffer1[0].separation = b3PlaneSeparation( refPlane, triangle->v1 );
	buffer1[0].pair = b3MakeFeaturePair( b3_featureShapeB, 2, b3_featureShapeB, 0 );
	buffer1[1].position = triangle->v2;
	buffer1[1].separation = b3PlaneSeparation( refPlane, triangle->v2 );
	buffer1[1].pair = b3MakeFeaturePair( b3_featureShapeB, 0, b3_featureShapeB, 1 );
	buffer1[2].position = triangle->v3;
	buffer1[2].separation = b3PlaneSeparation( refPlane, triangle->v3 );
	buffer1[2].pair = b3MakeFeaturePair( b3_featureShapeB, 1, b3_featureShapeB, 2 );
	int pointCount = 3;

	b3ClipVertex* input = buffer1;
	b3ClipVertex* output = buffer2;

	const b3HullFace* face = hullFaces + query.indexB;
	int edgeIndex = face->edge;
	int guard = 0;

	do
	{
		const b3HullHalfEdge* edge = hullEdges + edgeIndex;
		int nextEdgeIndex = edge->next;
		const b3HullHalfEdge* next = hullEdges + nextEdgeIndex;

		b3Vec3 vertex1 = hullPoints[edge->origin];
		b3Vec3 vertex2 = hullPoints[next->origin];

		// The side plane is deliberately unnormalized -- b3ClipPolygon reads
		// it through a sign and a ratio, both invariant under scaling it, and
		// the separations it records come from refPlane, which is unit as
		// baked. Same argument, and same saving, as b3BuildFaceAContact.
		b3Vec3 binormal = b3CrossDirection( b3Sub( vertex2, vertex1 ), refPlane.normal );
		b3Plane clipPlane = b3MakePlaneFromNormalAndPoint( binormal, vertex1 );

		pointCount = b3ClipPolygon( output, input, pointCount, clipPlane, edgeIndex, refPlane );

		// b3ClipPolygon returns B3_NULL_INDEX on overflow, which upstream has
		// no equivalent of; `< 3` catches both that and a genuine clip-away.
		if ( pointCount < 3 )
		{
			*cache = ( b3SATCache ){ 0 };
			return query.separation;
		}

		b3ClipVertex* swap = output;
		output = input;
		input = swap;

		edgeIndex = nextEdgeIndex;

		// See b3ClipSegmentToHullFace in convex_manifold.c. The hull here is
		// the shape's, not the level's, so it comes from the same baker the
		// other two walks trust -- the guard is cheap insurance against a
		// malformed one, not a suspicion about this call site.
		if ( ++guard > hull->edgeCount )
		{
			break;
		}
	}
	while ( edgeIndex != face->edge );

	pointCount = b3MinInt( pointCount, pointCapacity );

	b3f minSeparation = input[0].separation;
	int finalPointCount = 0;

	for ( int i = 0; i < pointCount; ++i )
	{
		b3ClipVertex* clipPoint = input + i;
		minSeparation = b3MinF( minSeparation, clipPoint->separation );

		if ( enableSpeculative == false && b3Raw( clipPoint->separation ) > 0 )
		{
			continue;
		}

		b3LocalManifoldPoint* pt = manifold->points + finalPointCount;

		// Moved onto the hull face, which culls better. Note the sibling
		// deliberately does not do this -- see there.
		pt->point = b3MulSub( clipPoint->position, clipPoint->separation, refPlane.normal );
		pt->separation = clipPoint->separation;
		pt->pair = b3FlipPair( clipPoint->pair );
		pt->triangleIndex = B3_NULL_INDEX;

		finalPointCount += 1;
	}

	b3f speculativeDistance = enableSpeculative ? B3_SPECULATIVE_DISTANCE : b3f_zero;

	// Strict here, non-strict in the sibling. That asymmetry is upstream's and
	// is preserved rather than tidied.
	if ( b3Raw( minSeparation ) > b3Raw( speculativeDistance ) )
	{
		manifold->pointCount = 0;
		*cache = ( b3SATCache ){ 0 };
		return minSeparation;
	}

	manifold->pointCount = finalPointCount;
	manifold->normal = b3Neg( refPlane.normal );
	manifold->feature = b3_featureHullFace;

	cache->separation = minSeparation;
	cache->type = (uint8_t)b3_faceAxisB;
	cache->indexA = (uint8_t)query.indexA;
	cache->indexB = (uint8_t)query.indexB;
	return minSeparation;
}

/// Contact points with the triangle as reference and a hull face clipped
/// against it.
///
/// @return the clipped separation, or B3_F_MAX when the incident face clipped
/// away entirely. See the note in b3CollideTriangleAndHull on why that
/// sentinel never reaches an addition.
static b3f b3CollideTriangleFace( b3LocalManifold* manifold, int pointCapacity, const b3TriangleData* triangle,
								  const b3HullData* hull, b3SeparatingAxis query, b3SATCache* cache,
								  bool enableSpeculative )
{
	B3_ASSERT( query.type == b3_faceAxisA );
	B3_ASSERT( query.indexA == 0 );
	B3_ASSERT( 0 <= query.indexB && query.indexB < hull->vertexCount );
	B3_ASSERT( manifold->pointCount == 0 );

	const b3HullFace* hullFaces = b3GetHullFaces( hull );
	const b3HullHalfEdge* hullEdges = b3GetHullEdges( hull );
	const b3Vec3* hullPoints = b3GetHullPoints( hull );

	b3Plane refPlane = triangle->plane;
	int incFace = b3FindIncidentFace( hull, refPlane.normal, query.indexB );

	b3ClipVertex buffer1[2 * B3_MAX_CLIP_POINTS];
	b3ClipVertex buffer2[2 * B3_MAX_CLIP_POINTS];

	int pointCount = 0;
	const b3HullFace* face = hullFaces + incFace;
	int hullEdgeIndex = face->edge;

	do
	{
		const b3HullHalfEdge* edge = hullEdges + hullEdgeIndex;
		int nextEdgeIndex = edge->next;
		const b3HullHalfEdge* next = hullEdges + nextEdgeIndex;

		b3Vec3 hullPoint = hullPoints[next->origin];
		buffer1[pointCount].position = hullPoint;
		buffer1[pointCount].separation = b3PlaneSeparation( refPlane, hullPoint );
		buffer1[pointCount].pair = b3MakeFeaturePair( b3_featureShapeB, hullEdgeIndex, b3_featureShapeB, nextEdgeIndex );

		pointCount += 1;
		hullEdgeIndex = nextEdgeIndex;
	}
	while ( hullEdgeIndex != face->edge && pointCount < 2 * B3_MAX_CLIP_POINTS );

	if ( pointCount < 3 )
	{
		*cache = ( b3SATCache ){ 0 };
		return B3_F_MAX;
	}

	b3ClipVertex* input = buffer1;
	b3ClipVertex* output = buffer2;

	b3Vec3 trianglePoints[3] = { triangle->v1, triangle->v2, triangle->v3 };
	b3Vec3 triangleEdges[3] = { triangle->e1, triangle->e2, triangle->e3 };

	for ( int i = 0; i < 3 && pointCount > 0; ++i )
	{
		// Unnormalized, as above.
		b3Vec3 sideNormal = b3CrossDirection( triangleEdges[i], refPlane.normal );
		b3Plane clipPlane = b3MakePlaneFromNormalAndPoint( sideNormal, trianglePoints[i] );

		pointCount = b3ClipPolygon( output, input, pointCount, clipPlane, i, refPlane );

		// The port's b3ClipPolygon can return B3_NULL_INDEX where upstream
		// cannot. Upstream's `== 0` test below would let -1 through, and it
		// only recovers by accident because minSeparation was seeded with
		// FLT_MAX. Caught here instead.
		if ( pointCount <= 0 )
		{
			*cache = ( b3SATCache ){ 0 };
			return B3_F_MAX;
		}

		b3ClipVertex* swap = output;
		output = input;
		input = swap;
	}

	pointCount = b3MinInt( pointCount, pointCapacity );

	b3f minSeparation = input[0].separation;
	int finalPointCount = 0;

	for ( int i = 0; i < pointCount; ++i )
	{
		b3ClipVertex* clipPoint = input + i;
		minSeparation = b3MinF( minSeparation, clipPoint->separation );

		if ( enableSpeculative == false && b3Raw( clipPoint->separation ) > 0 )
		{
			continue;
		}

		b3LocalManifoldPoint* pt = manifold->points + finalPointCount;

		// Deliberately *not* moved onto the reference face, unlike the
		// sibling. Upstream has that line commented out here, and the two
		// behaving differently is the upstream behaviour.
		pt->point = clipPoint->position;
		pt->separation = clipPoint->separation;
		pt->pair = clipPoint->pair;
		pt->triangleIndex = B3_NULL_INDEX;

		finalPointCount += 1;
	}

	b3f speculativeDistance = enableSpeculative ? B3_SPECULATIVE_DISTANCE : b3f_zero;

	// Non-strict here, strict in the sibling. Upstream's asymmetry, kept.
	if ( b3Raw( minSeparation ) >= b3Raw( speculativeDistance ) )
	{
		*cache = ( b3SATCache ){ 0 };
		return minSeparation;
	}

	manifold->pointCount = finalPointCount;
	manifold->normal = refPlane.normal;
	manifold->feature = b3_featureTriangleFace;

	cache->separation = minSeparation;
	cache->type = (uint8_t)b3_faceAxisA;
	cache->indexA = (uint8_t)query.indexA;
	cache->indexB = (uint8_t)query.indexB;
	return minSeparation;
}

/// One point, where a triangle edge crosses a hull edge.
static void b3CollideTriangleAndHullEdges( b3LocalManifold* manifold, int capacity, b3Vec3 trianglePoint,
										   b3Vec3 triangleEdge, const b3HullData* hull, b3SeparatingAxis query,
										   b3SATCache* cache )
{
	B3_ASSERT( query.type == b3_edgePairAxis );
	B3_ASSERT( 0 <= query.indexA && query.indexA < 3 );
	B3_ASSERT( 0 <= query.indexB && query.indexB < hull->edgeCount );

	const b3HullHalfEdge* edgesB = b3GetHullEdges( hull );
	const b3Vec3* pointsB = b3GetHullPoints( hull );

	const b3HullHalfEdge* edgeB = edgesB + query.indexB;
	const b3HullHalfEdge* twinB = edgesB + edgeB->twin;
	b3Vec3 pB = pointsB[edgeB->origin];
	b3Vec3 eB = b3Sub( pointsB[twinB->origin], pB );

	b3SegmentDistanceResult result = b3LineDistance( trianglePoint, triangleEdge, pB, eB );

	if ( capacity == 0 || b3IsWithinSegments( &result ) == false )
	{
		// Caching can slide the closest points off the ends of the segments.
		B3_ASSERT( manifold->pointCount == 0 );
		*cache = ( b3SATCache ){ 0 };
		return;
	}

	b3f separation = b3SeparationFromWide( b3DotWide( query.normal, b3Sub( pB, trianglePoint ) ) );

	manifold->normal = query.normal;
	manifold->pointCount = 1;

	static const b3TriangleFeature edgeFeatures[3] = { b3_featureEdge1, b3_featureEdge2, b3_featureEdge3 };
	manifold->feature = edgeFeatures[query.indexA];

	b3LocalManifoldPoint* pt = manifold->points + 0;
	pt->point = b3MulCV( b3cFromFrac( 1, 2 ), b3Add( result.point1, result.point2 ) );
	pt->separation = separation;
	pt->pair = b3MakeFeaturePair( b3_featureShapeA, query.indexA, b3_featureShapeB, query.indexB );
	pt->triangleIndex = B3_NULL_INDEX;

	cache->separation = separation;
	cache->type = (uint8_t)b3_edgePairAxis;
	cache->indexA = (uint8_t)query.indexA;
	cache->indexB = (uint8_t)query.indexB;
}

/// Collide a triangle and a hull.
///
/// Five stages, and the middle one is the reason this function is long: a
/// backside cull with hysteresis, then a **cache replay** that tries to
/// rebuild last step's contact without running the separating axis test at
/// all, then the full three-axis query, then the face-or-edge decision, then
/// a GJK fallback for the cases speculative distance leaves SAT with nothing.
///
/// @section sentinel The B3_F_MAX return, and why it never overflows
///
/// b3CollideTriangleFace returns B3_F_MAX to mean "the incident face clipped
/// away". Two comparisons below add B3_LINEAR_SLOP to a clipped separation,
/// and B3_F_MAX + slop would in principle wrap. It cannot here, for two
/// independent reasons, and both are worth stating because a reordering would
/// break them:
///
///   - Both comparisons are guarded by `manifold->pointCount` on the *left*
///     of an `&&`, and a B3_F_MAX return always leaves the count at zero. C's
///     short circuit means the addition is never evaluated.
///   - B3_F_MAX is INT32_MAX/2, so even if it were evaluated the sum stays
///     inside int32.
void b3CollideTriangleAndHull( b3LocalManifold* manifold, int capacity, b3Vec3 v1, b3Vec3 v2, b3Vec3 v3,
							   int triangleFlags, const b3HullData* hullB, b3SATCache* cache, bool enableSpeculative )
{
	manifold->pointCount = 0;

	if ( capacity < 4 )
	{
		return;
	}

	const b3Vec3* hullPoints = b3GetHullPoints( hullB );
	const b3Plane* hullPlanes = b3GetHullPlanes( hullB );
	const b3HullHalfEdge* edges = b3GetHullEdges( hullB );

	b3Plane trianglePlane = b3MakePlaneFromPoints( v1, v2, v3 );

	// Back-side cull, with hysteresis so a hull skimming the plane does not
	// flip between contact and none every step.
	b3f offset = b3PlaneSeparation( trianglePlane, hullB->center );

	if ( cache->type == b3_backsideAxis &&
		 b3Raw( b3AbsF( b3SubF( cache->separation, offset ) ) ) < b3Raw( B3_LINEAR_SLOP ) )
	{
		return;
	}

	if ( b3Raw( offset ) < -b3Raw( B3_LINEAR_SLOP ) )
	{
		cache->type = (uint8_t)b3_backsideAxis;
		cache->separation = offset;
		return;
	}

	b3Vec3 trianglePoints[3] = { v1, v2, v3 };
	b3Vec3 triangleEdges[3] = { b3Sub( v2, v1 ), b3Sub( v3, v2 ), b3Sub( v1, v3 ) };

	b3TriangleData triangle;
	triangle.v1 = v1;
	triangle.v2 = v2;
	triangle.v3 = v3;
	triangle.e1 = triangleEdges[0];
	triangle.e2 = triangleEdges[1];
	triangle.e3 = triangleEdges[2];
	triangle.plane = trianglePlane;
	triangle.flags = triangleFlags;

	b3f speculativeDistance = enableSpeculative ? B3_SPECULATIVE_DISTANCE : b3f_zero;

	cache->hit = 1;

	// --- cache replay -----------------------------------------------------
	//
	// Every branch either returns on a hit or falls out of the switch into the
	// full test, so this is pure optimization: deleting it would leave the
	// function correct. Each one passes a *copy* of the cache to the builder,
	// because the builders write their cache argument unconditionally and a
	// rejected replay must leave the real one untouched.
	switch ( cache->type )
	{
		case b3_faceAxisA:
		{
			if ( cache->indexB >= hullB->vertexCount )
			{
				break;
			}

			int vertexIndex = b3FindHullSupportVertex( hullB, b3Neg( trianglePlane.normal ) );
			b3f separation = b3PlaneSeparation( trianglePlane, hullPoints[vertexIndex] );

			if ( b3Raw( separation ) > b3Raw( speculativeDistance ) )
			{
				return;
			}

			b3SeparatingAxis faceQuery;
			faceQuery.normal = trianglePlane.normal;
			faceQuery.separation = separation;
			faceQuery.indexA = 0;
			faceQuery.indexB = vertexIndex;
			faceQuery.type = b3_faceAxisA;

			b3SATCache localCache = *cache;
			b3f clipped = b3CollideTriangleFace( manifold, capacity, &triangle, hullB, faceQuery, &localCache,
												 enableSpeculative );

			if ( manifold->pointCount > 0 &&
				 b3Raw( b3AbsF( b3SubF( cache->separation, clipped ) ) ) < b3Raw( B3_LINEAR_SLOP ) )
			{
				return;
			}

			manifold->pointCount = 0;
			*cache = ( b3SATCache ){ 0 };
		}
		break;

		case b3_faceAxisB:
		{
			if ( cache->indexB >= hullB->faceCount )
			{
				break;
			}

			b3Plane plane = hullPlanes[cache->indexB];
			int vertexIndex = b3GetTriangleSupport( trianglePoints, b3Neg( plane.normal ) );
			b3f separation = b3PlaneSeparation( plane, trianglePoints[vertexIndex] );

			if ( b3Raw( separation ) > b3Raw( speculativeDistance ) )
			{
				return;
			}

			// A deep overlap can leave a cache that no longer describes the
			// configuration, so it is not replayed.
			bool isDeep = b3Raw( separation ) < -2 * b3Raw( B3_LINEAR_SLOP );

			if ( isDeep == false )
			{
				b3SeparatingAxis faceQuery;
				faceQuery.normal = b3Neg( plane.normal );
				faceQuery.separation = separation;
				faceQuery.indexA = vertexIndex;
				faceQuery.indexB = cache->indexB;
				faceQuery.type = b3_faceAxisB;

				b3SATCache localCache = *cache;
				b3f clipped =
					b3CollideHullFace( manifold, capacity, &triangle, hullB, faceQuery, &localCache, enableSpeculative );

				if ( manifold->pointCount > 0 &&
					 b3Raw( b3AbsF( b3SubF( cache->separation, clipped ) ) ) < b3Raw( B3_LINEAR_SLOP ) )
				{
					return;
				}
			}

			manifold->pointCount = 0;
			*cache = ( b3SATCache ){ 0 };
		}
		break;

		case b3_edgePairAxis:
		{
			if ( cache->indexA >= 3 || cache->indexB + 1 >= hullB->edgeCount || ( cache->indexB & 1 ) != 0 )
			{
				break;
			}

			int indexA = cache->indexA;
			int indexB = cache->indexB;

			const b3HullHalfEdge* edge2 = edges + indexB;
			const b3HullHalfEdge* twin2 = edges + indexB + 1;

			b3Vec3 hullPoint = hullPoints[edge2->origin];
			b3Vec3 hullEdge = b3Sub( hullPoints[twin2->origin], hullPoint );

			b3Vec3 axis;
			b3f separation;

			if ( b3TriangleHullEdgeAxis( &axis, &separation, trianglePoints[indexA], triangleEdges[indexA],
										 trianglePlane.normal, hullPoint, hullEdge, hullPlanes[edge2->face].normal,
										 hullPlanes[twin2->face].normal ) )
			{
				if ( b3Raw( separation ) > b3Raw( speculativeDistance ) )
				{
					return;
				}

				if ( b3Raw( b3AbsF( b3SubF( cache->separation, separation ) ) ) < b3Raw( B3_LINEAR_SLOP ) )
				{
					b3SeparatingAxis edgeQuery;
					edgeQuery.normal = b3Neg( axis );
					edgeQuery.separation = separation;
					edgeQuery.indexA = indexA;
					edgeQuery.indexB = indexB;
					edgeQuery.type = b3_edgePairAxis;

					b3SATCache localCache = *cache;
					b3CollideTriangleAndHullEdges( manifold, capacity, trianglePoints[indexA], triangleEdges[indexA],
												   hullB, edgeQuery, &localCache );

					if ( manifold->pointCount > 0 )
					{
						return;
					}
				}
			}

			manifold->pointCount = 0;
			*cache = ( b3SATCache ){ 0 };
		}
		break;

		default:
			// Upstream asserts on anything unexpected. A b3SATCache is
			// persistent state that may not mean what it claims, so an unknown
			// type simply costs the full test.
			break;
	}

	// --- the full test ----------------------------------------------------

	cache->hit = 0;
	manifold->pointCount = 0;

	b3SeparatingAxis faceQueryA = b3QueryTriangleFace( &triangle, hullB );
	if ( b3Raw( faceQueryA.separation ) > b3Raw( speculativeDistance ) )
	{
		cache->separation = faceQueryA.separation;
		cache->type = (uint8_t)b3_faceAxisA;
		cache->indexA = (uint8_t)faceQueryA.indexA;
		cache->indexB = (uint8_t)faceQueryA.indexB;
		return;
	}

	b3SeparatingAxis faceQueryB = b3QueryHullFace( &triangle, hullB );
	if ( b3Raw( faceQueryB.separation ) > b3Raw( speculativeDistance ) )
	{
		cache->separation = faceQueryB.separation;
		cache->type = (uint8_t)b3_faceAxisB;
		cache->indexA = (uint8_t)faceQueryB.indexA;
		cache->indexB = (uint8_t)faceQueryB.indexB;
		return;
	}

	b3SeparatingAxis edgeQuery = b3QueryTriangleAndHullEdges( &triangle, hullB );
	if ( b3Raw( edgeQuery.separation ) > b3Raw( speculativeDistance ) )
	{
		cache->separation = edgeQuery.separation;
		cache->type = (uint8_t)b3_edgePairAxis;
		cache->indexA = (uint8_t)edgeQuery.indexA;
		cache->indexB = (uint8_t)edgeQuery.indexB;
		return;
	}

	// A hull face pointing back into the triangle is not a reference face
	// worth having -- clipping against it produces contacts that push a body
	// the wrong way. The tolerance is what stops a wavy mesh ghosting.
	const int64_t kPushingDownWide = -4194304; // -0.25 at Q24
	bool pushingDown = b3DotWide( faceQueryB.normal, trianglePlane.normal ) < kPushingDownWide;

	b3f clipSeparation;
	if ( b3Raw( faceQueryB.separation ) >= b3Raw( faceQueryA.separation ) && pushingDown == false )
	{
		clipSeparation = b3CollideHullFace( manifold, capacity, &triangle, hullB, faceQueryB, cache, enableSpeculative );
	}
	else
	{
		clipSeparation =
			b3CollideTriangleFace( manifold, capacity, &triangle, hullB, faceQueryA, cache, enableSpeculative );
	}

	if ( edgeQuery.indexA != B3_NULL_INDEX )
	{
		// When the axes are aligned the edge separation can be garbage, and a
		// face axis with positive separation may have produced no points.
		//
		// Note the pointCount tests sit on the left of both `&&`s: that is
		// what keeps clipSeparation's B3_F_MAX out of the addition below.
		b3f maxFaceSeparation = b3MaxF( faceQueryA.separation, faceQueryB.separation );

		if ( ( manifold->pointCount == 0 && b3Raw( edgeQuery.separation ) > b3Raw( maxFaceSeparation ) ) ||
			 ( manifold->pointCount == 1 &&
			   b3Raw( edgeQuery.separation ) > b3Raw( b3AddF( clipSeparation, B3_LINEAR_SLOP ) ) ) )
		{
			B3_ASSERT( 0 <= edgeQuery.indexA && edgeQuery.indexA < 3 );
			manifold->pointCount = 0;
			b3CollideTriangleAndHullEdges( manifold, capacity, trianglePoints[edgeQuery.indexA],
										   triangleEdges[edgeQuery.indexA], hullB, edgeQuery, cache );
		}
	}

	// Speculative distance means SAT can legitimately produce no points at
	// all. GJK is the fallback, and it is what prevents tunnelling in the
	// rare cases where it happens.
	if ( manifold->pointCount == 0 )
	{
		b3Vec3 triangleProxy[3] = { v1, v2, v3 };

		b3DistanceInput input;
		input.proxyA = ( b3ShapeProxy ){ triangleProxy, 3, b3f_zero };
		input.proxyB = ( b3ShapeProxy ){ hullPoints, hullB->vertexCount, b3f_zero };
		input.transform = b3Transform_identity;
		input.useRadii = false;

		b3SimplexCache simplexCache = { 0 };
		b3DistanceOutput output = b3ShapeDistance( &input, &simplexCache, NULL, 0 );

		if ( b3Raw( output.distance ) > 0 )
		{
			manifold->pointCount = 1;
			manifold->feature = b3GetTriangleFeature( &simplexCache );
			manifold->normal = output.normal;

			b3LocalManifoldPoint* pt = manifold->points + 0;
			pt->point = output.pointB;
			pt->separation = output.distance;

			// Not an accurate feature pair, but there is no better one and
			// warm starting a single speculative point matters little.
			pt->pair = b3FeaturePair_single;
			pt->triangleIndex = B3_NULL_INDEX;
		}

		// There is no way to cache this scenario.
		*cache = ( b3SATCache ){ 0 };
	}
}
