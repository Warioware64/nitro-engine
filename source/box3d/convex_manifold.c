// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Erin Catto        (original Box3D)
// Copyright (c) 2026 Warioware64       (Nitro Engine Advanced fixed-point port)
//
// Dirk Gregorius contributed portions of this code
//
// This file is part of Nitro Engine Advanced

/// @file   convex_manifold.c
/// @brief  Contact manifold generation for convex shapes.
///
/// Complete: the primitive collide functions -- sphere/sphere,
/// capsule/sphere, capsule/capsule -- plus hull/sphere, hull/capsule, and
/// hull/hull with the separating axis test it runs on.
///
/// Two departures from upstream are structural rather than incidental, and
/// are recorded here because a diff against upstream will not explain them:
///
///   - simd.h is not ported. Upstream's b3CollideHulls is written against a
///     4-wide b3FloatW over structure-of-arrays hull data, with the support
///     function recovering its argmax index from float mantissa bits.
///     ARMv5TE has no SIMD unit, upstream already ships a scalar edge phase
///     behind B3_SIMD_NONE, and the index trick would cost 5 of Q12's 12
///     fraction bits. The port goes scalar throughout and the SoA arrays are
///     gone from b3HullData.
///
///   - The second b3CollideHulls in upstream's file, behind
///     `#if B3_SIMD_COLLIDE_HULLS == 1`'s #else, is not ported either. It is
///     marked "this code has gone stale, will be deleted soon" upstream and
///     never compiles, since the macro is defined to 1 immediately above it.
///
/// @section midpoint The midpoint construction
///
/// Every function here places the contact at the midpoint between the two
/// surfaces rather than on either one:
///
///     point = 0.5 * ( (pA + rA*n) + (pB - rB*n) )
///
/// That is upstream's choice, and it happens to suit fixed point: the two
/// surface points are each within a radius of their centre, so the midpoint
/// is bounded by the geometry rather than by how far the pair sits from the
/// world origin.

#include "box3d/collision.h"
#include "box3d/constants.h"
#include "box3d/math_fixed.h"
#include "box3d/types.h"

#include "core.h"
#include "manifold.h"

// =========================================================================
// Sphere vs sphere
// =========================================================================

void b3CollideSpheres( b3LocalManifold* manifold, int capacity, const b3Sphere* sphereA, const b3Sphere* sphereB,
					   b3Transform transformBtoA )
{
	manifold->pointCount = 0;

	if ( capacity < 1 )
	{
		return;
	}

	b3Vec3 center1 = sphereA->center;
	b3Vec3 center2 = b3TransformPoint( transformBtoA, sphereB->center );

	b3f totalRadius = b3AddF( sphereA->radius, sphereB->radius );
	b3Vec3 offset = b3Sub( center2, center1 );

	// Both sides squared, both wide. Comparing squared lengths avoids a
	// square root on the rejection path, which is the common case.
	int64_t distanceSq = b3LengthSquaredWide( offset );
	int64_t totalRadiusSq = (int64_t)b3Raw( totalRadius ) * b3Raw( totalRadius );

	if ( distanceSq > totalRadiusSq )
	{
		// Separated.
		return;
	}

	b3Vec3 normal = b3NormalOrUp( offset, distanceSq );
	b3f distance = b3SqrtWide( distanceSq );

	manifold->normal = normal;
	manifold->pointCount = 1;

	b3LocalManifoldPoint* pt = manifold->points + 0;
	pt->point = b3MidpointContact( center1, sphereA->radius, center2, sphereB->radius, normal );
	pt->separation = b3SubF( distance, totalRadius );
	pt->pair = b3FeaturePair_single;
	pt->triangleIndex = B3_NULL_INDEX;
}

// =========================================================================
// Capsule vs sphere
// =========================================================================

void b3CollideCapsuleAndSphere( b3LocalManifold* manifold, int capacity, const b3Capsule* capsuleA, const b3Sphere* sphereB,
								b3Transform transformBtoA )
{
	manifold->pointCount = 0;

	if ( capacity < 1 )
	{
		return;
	}

	b3Vec3 center = b3TransformPoint( transformBtoA, sphereB->center );
	b3Vec3 center1 = capsuleA->center1;
	b3Vec3 center2 = capsuleA->center2;

	b3f totalRadius = b3AddF( sphereB->radius, capsuleA->radius );

	b3Vec3 closestPoint = b3PointToSegmentDistance( center1, center2, center );
	b3Vec3 offset = b3Sub( center, closestPoint );

	int64_t distanceSq = b3LengthSquaredWide( offset );
	int64_t totalRadiusSq = (int64_t)b3Raw( totalRadius ) * b3Raw( totalRadius );

	if ( distanceSq > totalRadiusSq )
	{
		return;
	}

	b3Vec3 normal = b3NormalOrUp( offset, distanceSq );
	b3f distance = b3SqrtWide( distanceSq );

	manifold->normal = normal;
	manifold->pointCount = 1;

	// The sphere is B here, so the roles reverse relative to b3CollideSpheres:
	// the sphere's surface is behind the normal, the capsule's ahead of it.
	b3LocalManifoldPoint* pt = manifold->points + 0;
	pt->point = b3MidpointContact( closestPoint, capsuleA->radius, center, sphereB->radius, normal );
	pt->separation = b3SubF( distance, totalRadius );
	pt->pair = b3FeaturePair_single;
	pt->triangleIndex = B3_NULL_INDEX;
}

// =========================================================================
// Capsule vs capsule
// =========================================================================

void b3CollideCapsules( b3LocalManifold* manifold, int capacity, const b3Capsule* capsuleA, const b3Capsule* capsuleB,
						b3Transform transformBtoA )
{
	manifold->pointCount = 0;

	if ( capacity < 2 )
	{
		return;
	}

	b3Vec3 centerA1 = capsuleA->center1;
	b3Vec3 centerA2 = capsuleA->center2;
	b3Vec3 centerB1 = b3TransformPoint( transformBtoA, capsuleB->center1 );
	b3Vec3 centerB2 = b3TransformPoint( transformBtoA, capsuleB->center2 );

	b3f radius = b3AddF( capsuleA->radius, capsuleB->radius );
	b3f maxDistance = b3AddF( radius, B3_SPECULATIVE_DISTANCE );

	b3SegmentDistanceResult result = b3SegmentDistance( centerA1, centerA2, centerB1, centerB2 );
	b3Vec3 offset = b3Sub( result.point2, result.point1 );
	int64_t distanceSquared = b3LengthSquaredWide( offset );

	int64_t maxDistanceSq = (int64_t)b3Raw( maxDistance ) * b3Raw( maxDistance );

	// Upstream's lower guard is `distanceSquared < minDistance*minDistance`
	// with minDistance = 0.01 * B3_LINEAR_SLOP. In Q12 the slop is 20 raw, so
	// a hundredth of it is 0.2 -- it rounds to zero and the test could never
	// fire. The guard exists to avoid dividing by a vanishing distance, and
	// the fixed-point statement of that is simply a squared distance of zero.
	if ( distanceSquared > maxDistanceSq || distanceSquared == 0 )
	{
		return;
	}

	b3f lengthA;
	b3Vec3 edgeA = b3GetLengthAndNormalize( &lengthA, b3Sub( centerA2, centerA1 ) );
	if ( b3Raw( lengthA ) < b3Raw( B3_MIN_CAPSULE_LENGTH ) )
	{
		return;
	}

	b3f lengthB;
	b3Vec3 edgeB = b3GetLengthAndNormalize( &lengthB, b3Sub( centerB2, centerB1 ) );
	if ( b3Raw( lengthB ) < b3Raw( B3_MIN_CAPSULE_LENGTH ) )
	{
		return;
	}

	// |eA x eB| is sin(angle between the edges), so a small cross product
	// means the capsules are nearly parallel and deserve two contact points
	// rather than one. The tolerance is 0.05, compared squared: 0.0025, which
	// at Q24 is 41943 -- comfortably above quantization, so the test stays
	// meaningful.
	const int64_t alphaTolSqr = 41943;
	b3Vec3 axis = b3Cross( edgeA, edgeB );

	if ( b3LengthSquaredWide( axis ) < alphaTolSqr )
	{
		// Clip segment B against the side planes of segment A.
		b3Plane planesA[2];
		planesA[0].normal = b3Neg( edgeA );
		planesA[0].offset = b3NegF( b3Dot( edgeA, capsuleA->center1 ) );
		planesA[1].normal = edgeA;
		planesA[1].offset = b3Dot( edgeA, capsuleA->center2 );

		b3ClipVertex verticesB[2];
		verticesB[0].position = centerB1;
		verticesB[0].separation = b3f_zero;
		verticesB[0].pair = b3MakeFeaturePair( b3_featureShapeA, 0, b3_featureShapeA, 0 );
		verticesB[1].position = centerB2;
		verticesB[1].separation = b3f_zero;
		verticesB[1].pair = b3MakeFeaturePair( b3_featureShapeA, 1, b3_featureShapeA, 1 );

		int pointCount = b3ClipSegment( verticesB, planesA[0] );
		if ( pointCount == 2 )
		{
			pointCount = b3ClipSegment( verticesB, planesA[1] );
		}

		if ( pointCount == 2 )
		{
			b3Vec3 closestPoint1 = b3PointToSegmentDistance( centerA1, centerA2, verticesB[0].position );
			b3Vec3 closestPoint2 = b3PointToSegmentDistance( centerA1, centerA2, verticesB[1].position );

			b3Vec3 delta1 = b3Sub( verticesB[0].position, closestPoint1 );
			b3Vec3 delta2 = b3Sub( verticesB[1].position, closestPoint2 );

			int64_t distSq1 = b3LengthSquaredWide( delta1 );
			int64_t distSq2 = b3LengthSquaredWide( delta2 );

			b3f distance1 = b3SqrtWide( distSq1 );
			b3f distance2 = b3SqrtWide( distSq2 );

			if ( b3Raw( distance1 ) <= b3Raw( radius ) && b3Raw( distance2 ) <= b3Raw( radius ) )
			{
				// Same vanishing-distance guard as above, for the same
				// reason: these two feed a normalization.
				if ( distSq1 == 0 || distSq2 == 0 )
				{
					return;
				}

				b3Vec3 normal1 = b3MulWV( b3RsqrtWide( distSq1 ), delta1 );
				b3Vec3 normal2 = b3MulWV( b3RsqrtWide( distSq2 ), delta2 );
				b3Vec3 normal = b3Normalize( b3Add( normal1, normal2 ) );

				b3f radiusA = capsuleA->radius;
				b3f radiusB = capsuleB->radius;

				// Each point is built with its own normal on the A side but
				// the shared normal on the B side, matching upstream.
				b3Vec3 point1 = b3MulCV(
					b3cFromFrac( 1, 2 ),
					b3MulSub( b3Add( b3MulAdd( verticesB[0].position, radiusA, normal1 ), closestPoint1 ), radiusB, normal ) );
				b3Vec3 point2 = b3MulCV(
					b3cFromFrac( 1, 2 ),
					b3MulSub( b3Add( b3MulAdd( verticesB[1].position, radiusA, normal2 ), closestPoint2 ), radiusB, normal ) );

				manifold->normal = normal;
				manifold->pointCount = 2;

				b3LocalManifoldPoint* pt1 = manifold->points + 0;
				pt1->point = point1;
				pt1->separation = b3SubF( distance1, radius );
				pt1->pair = verticesB[0].pair;
				pt1->triangleIndex = B3_NULL_INDEX;

				b3LocalManifoldPoint* pt2 = manifold->points + 1;
				pt2->point = point2;
				pt2->separation = b3SubF( distance2, radius );
				pt2->pair = verticesB[1].pair;
				pt2->triangleIndex = B3_NULL_INDEX;

				return;
			}
		}
	}

	// Single contact at the closest approach.
	b3f distance;
	b3Vec3 normal = b3GetLengthAndNormalize( &distance, offset );

	manifold->normal = normal;
	manifold->pointCount = 1;

	b3LocalManifoldPoint* pt = manifold->points + 0;
	pt->point = b3MidpointContact( result.point1, capsuleA->radius, result.point2, capsuleB->radius, normal );
	pt->separation = b3SubF( distance, radius );
	pt->pair = b3FeaturePair_single;
	pt->triangleIndex = B3_NULL_INDEX;
}

// =========================================================================
// Hull vs sphere
// =========================================================================

void b3CollideHullAndSphere( b3LocalManifold* manifold, int capacity, const b3HullData* hullA, const b3Sphere* sphereB,
							 b3Transform transformBtoA, b3SimplexCache* cache )
{
	manifold->pointCount = 0;

	if ( capacity == 0 )
	{
		return;
	}

	b3Vec3 center = b3TransformPoint( transformBtoA, sphereB->center );

	// Work in shapeA coordinates. The hull is a bare point cloud to GJK; its
	// faces only matter on the deep path below.
	b3DistanceInput distanceInput;
	distanceInput.proxyA = ( b3ShapeProxy ){ b3GetHullPoints( hullA ), hullA->vertexCount, b3f_zero };
	distanceInput.proxyB = ( b3ShapeProxy ){ &center, 1, b3f_zero };
	distanceInput.transform = b3Transform_identity;
	distanceInput.useRadii = false;

	b3DistanceOutput distanceOutput = b3ShapeDistance( &distanceInput, cache, NULL, 0 );

	b3f radius = sphereB->radius;

	if ( b3Raw( distanceOutput.distance ) > b3Raw( b3AddF( radius, B3_SPECULATIVE_DISTANCE ) ) )
	{
		// Separated.
		*cache = ( b3SimplexCache ){ 0 };
		return;
	}

	b3Vec3 normal;
	b3f separation;

	// Upstream splits shallow from deep at `distance > 100 * FLT_EPSILON`,
	// which is 1.2e-5 -- a twentieth of a Q12 quantum. What it is really
	// asking is whether the two witness points are far enough apart to define
	// a direction, and in fixed point that question has one answer: a distance
	// of zero. The same test b3NormalOrUp makes.
	if ( b3Raw( distanceOutput.distance ) > 0 )
	{
		// Shallow: the witness points give the normal directly.
		normal = b3Normalize( b3Sub( distanceOutput.pointB, distanceOutput.pointA ) );
		separation = distanceOutput.distance;
	}
	else
	{
		// Deep: the centre is inside the hull, so take the face it is least
		// far behind and push out along that.
		const b3Plane* planes = b3GetHullPlanes( hullA );

		int bestIndex = 0;
		b3f bestDistance = b3PlaneSeparation( planes[0], center );

		for ( int index = 1; index < hullA->faceCount; ++index )
		{
			b3f distance = b3PlaneSeparation( planes[index], center );
			if ( b3Raw( distance ) > b3Raw( bestDistance ) )
			{
				bestIndex = index;
				bestDistance = distance;
			}
		}

		normal = planes[bestIndex].normal;
		separation = bestDistance;
	}

	// cA is the sphere centre projected onto the hull surface along the
	// normal; cB is the deepest point of the sphere. Their midpoint is the
	// contact, which is b3MidpointContact with a zero radius on the A side --
	// spelt out here because the projection distance is not the separation
	// when the centre is inside.
	b3f depth = b3Dot( b3Sub( center, distanceOutput.pointA ), normal );
	b3Vec3 cA = b3MulSub( center, depth, normal );
	b3Vec3 cB = b3MulSub( center, radius, normal );

	manifold->normal = normal;
	manifold->pointCount = 1;

	b3LocalManifoldPoint* pt = manifold->points + 0;
	pt->point = b3MulCV( b3cFromFrac( 1, 2 ), b3Add( cA, cB ) );
	pt->separation = b3SubF( separation, radius );
	pt->pair = b3FeaturePair_single;
	pt->triangleIndex = B3_NULL_INDEX;
}

// =========================================================================
// Hull vs capsule
// =========================================================================

/// Clip a segment against the side planes of one hull face.
///
/// Each side plane is built from a face edge and the face normal. Upstream
/// normalizes the edge tangent first; the port does not, and that is a gain
/// rather than a compromise: b3ClipSegment consumes the plane only through
/// sign tests and the ratio d1 / (d1 - d2), and both are invariant under
/// scaling the plane. Skipping the normalization avoids rounding a unit
/// vector into Q12 and saves a square root per face edge.
///
/// The cost is range. An unnormalized binormal has magnitude |edge|, so the
/// separations it produces are distances scaled by that, and b3Dot narrows to
/// Q12 -- which caps hull edge length times point magnitude at ~500 units
/// squared. That is far outside the documented world scale, and the
/// alternative would have cost precision on every hull instead.
static int b3ClipSegmentToHullFace( b3ClipVertex segment[2], const b3HullData* hull, int refFace )
{
	const b3HullFace* faces = b3GetHullFaces( hull );
	const b3Plane* planes = b3GetHullPlanes( hull );
	const b3HullHalfEdge* edges = b3GetHullEdges( hull );
	const b3Vec3* points = b3GetHullPoints( hull );

	b3Plane refPlane = planes[refFace];
	const b3HullFace* face = faces + refFace;

	int edgeIndex = face->edge;
	int guard = 0;

	do
	{
		const b3HullHalfEdge* edge = edges + edgeIndex;
		int nextEdgeIndex = edge->next;
		const b3HullHalfEdge* next = edges + nextEdgeIndex;

		b3Vec3 vertex1 = points[edge->origin];
		b3Vec3 vertex2 = points[next->origin];
		b3Vec3 tangent = b3Sub( vertex2, vertex1 );
		b3Vec3 binormal = b3Cross( tangent, refPlane.normal );

		int pointCount = b3ClipSegment( segment, b3MakePlaneFromNormalAndPoint( binormal, vertex1 ) );
		if ( pointCount < 2 )
		{
			return 0;
		}

		edgeIndex = nextEdgeIndex;

		// This ring closes back on face->edge only if the hull's `next` chain
		// is well formed. A well formed hull always closes it, so the guard
		// never fires -- but the failure it prevents is a frozen console with
		// no diagnostic, and the cost is one increment against a loop body
		// that already does a cross product and a segment clip. Same pattern
		// as b3FindMinEdge in manifold.c.
		if ( ++guard > hull->edgeCount )
		{
			break;
		}
	}
	while ( edgeIndex != face->edge );

	return 2;
}

/// The hull face the capsule segment is least far behind.
static b3SeparatingAxis b3QueryFaceDirectionHullAndCapsule( const b3HullData* hull, const b3Capsule* capsule,
															b3Transform capsuleTransform )
{
	const b3Plane* planes = b3GetHullPlanes( hull );

	b3Vec3 capsulePoints[2] = {
		b3TransformPoint( capsuleTransform, capsule->center1 ),
		b3TransformPoint( capsuleTransform, capsule->center2 ),
	};

	B3_ASSERT( hull->faceCount > 0 );

	// Upstream seeds with -FLT_MAX. Face zero serves, and costs nothing.
	int maxFaceIndex = 0;
	int maxVertexIndex = b3GetPointSupport( capsulePoints, 2, b3Neg( planes[0].normal ) );
	b3f maxFaceSeparation = b3PlaneSeparation( planes[0], capsulePoints[maxVertexIndex] );

	for ( int faceIndex = 1; faceIndex < hull->faceCount; ++faceIndex )
	{
		b3Plane plane = planes[faceIndex];

		int vertexIndex = b3GetPointSupport( capsulePoints, 2, b3Neg( plane.normal ) );
		b3f separation = b3PlaneSeparation( plane, capsulePoints[vertexIndex] );
		if ( b3Raw( separation ) > b3Raw( maxFaceSeparation ) )
		{
			maxVertexIndex = vertexIndex;
			maxFaceIndex = faceIndex;
			maxFaceSeparation = separation;
		}
	}

	b3SeparatingAxis axis;
	axis.normal = planes[maxFaceIndex].normal;
	axis.separation = maxFaceSeparation;
	axis.indexA = maxFaceIndex;
	axis.indexB = maxVertexIndex;
	axis.type = b3_faceAxisA;
	return axis;
}

/// The best edge-edge axis between a hull edge and the capsule segment.
static b3SeparatingAxis b3QueryEdgeDirectionHullAndCapsule( const b3HullData* hull, const b3Capsule* capsule,
															b3Transform capsuleTransform )
{
	b3SeparatingAxis best;
	best.normal = b3Vec3_zero;
	best.separation = b3f_zero;
	best.indexA = B3_NULL_INDEX;
	best.indexB = B3_NULL_INDEX;
	best.type = b3_edgePairAxis;

	// Work in the hull's frame.
	b3Vec3 pA = b3TransformPoint( capsuleTransform, capsule->center1 );
	b3Vec3 qA = b3TransformPoint( capsuleTransform, capsule->center2 );
	b3Vec3 eA = b3Sub( qA, pA );

	const b3HullHalfEdge* edges = b3GetHullEdges( hull );
	const b3Vec3* points = b3GetHullPoints( hull );
	const b3Plane* planes = b3GetHullPlanes( hull );

	int64_t edgeLengthSq = b3LengthSquaredWide( eA );

	for ( int index = 0; index < hull->edgeCount; index += 2 )
	{
		const b3HullHalfEdge* edge = edges + index;
		const b3HullHalfEdge* twin = edges + index + 1;
		B3_ASSERT( edge->twin == index + 1 && twin->twin == index );

		b3Vec3 qB = points[twin->origin];
		b3Vec3 uB = planes[edge->face].normal;
		b3Vec3 vB = planes[twin->face].normal;

		// An isolated edge -- which is what a capsule is -- traces a circle
		// through the origin on the Gauss map, so overlap with the arc from
		// uB to vB reduces to the two dot products having opposite signs.
		b3f cba = b3Dot( uB, eA );
		b3f dba = b3Dot( vB, eA );

		// Upstream writes `cba * dba < 0`. The signs are the whole content of
		// that test, and taking them directly avoids a product that would need
		// widening to be safe.
		bool opposite = ( b3Raw( cba ) < 0 ) != ( b3Raw( dba ) < 0 ) && b3Raw( cba ) != 0 && b3Raw( dba ) != 0;
		if ( opposite == false )
		{
			continue;
		}

		// Reject near-parallel edges, where the separation would be decided by
		// quantization. Upstream compares max(cba², dba²) against
		// tol² * |eA|²; both sides are squared lengths, so both go wide, and
		// the Q30 tolerance is squared into Q30 once outside the loop.
		int64_t cbaSq = (int64_t)b3Raw( cba ) * b3Raw( cba );
		int64_t dbaSq = (int64_t)b3Raw( dba ) * b3Raw( dba );
		int64_t maxSq = cbaSq > dbaSq ? cbaSq : dbaSq;

		// tol² = 0.005² = 2.5e-5, which is 26843 at Q30.
		const int64_t parallelTolSq = 26843;
		if ( maxSq < ( ( edgeLengthSq * parallelTolSq ) >> B3_C_SHIFT ) )
		{
			continue;
		}

		// The arc from uB to vB crosses the plane of the capsule's circle at
		// t = cba / (cba - dba). The signs differ, so the denominator has the
		// larger magnitude and the quotient lands in [0, 1] -- the division is
		// safe by construction, as it is in b3ClipSegment.
		b3c t = b3DivFFToC( cba, b3SubF( cba, dba ) );
		b3Vec3 axis = b3Lerp( uB, vB, t );

		if ( b3LengthSquaredWide( axis ) == 0 )
		{
			// Opposed face normals that cancel. Upstream asserts this away;
			// here it is a real possibility once the lerp is quantized.
			continue;
		}

		axis = b3Normalize( axis );

		// The axis lies between two of B's face normals, so it already points
		// from B to A and needs no orienting. It is perpendicular to both
		// edges, so any point on each serves to measure the gap.
		b3f separation = b3Dot( axis, b3Sub( qA, qB ) );

		if ( best.indexB == B3_NULL_INDEX || b3Raw( separation ) > b3Raw( best.separation ) )
		{
			// No early exit on a separating axis: the best one is wanted for
			// the cache, and the radius is accounted for later.
			best.normal = axis;
			best.separation = separation;
			best.indexA = 0;
			best.indexB = index;
		}
	}

	return best;
}

/// Two contact points from the capsule segment clipped to a hull face.
static bool b3BuildHullFaceAndCapsuleContact( b3LocalManifold* manifold, const b3HullData* hullA, const b3Capsule* capsuleB,
											  b3Transform transformBtoA, b3SeparatingAxis query )
{
	const b3Plane* planes = b3GetHullPlanes( hullA );

	int refFace = query.indexA;
	b3Plane refPlane = planes[refFace];

	b3ClipVertex segmentB[2];
	segmentB[0].position = b3TransformPoint( transformBtoA, capsuleB->center1 );
	segmentB[0].separation = b3f_zero;
	segmentB[0].pair = b3MakeFeaturePair( b3_featureShapeA, 0, b3_featureShapeA, 0 );
	segmentB[1].position = b3TransformPoint( transformBtoA, capsuleB->center2 );
	segmentB[1].separation = b3f_zero;
	segmentB[1].pair = b3MakeFeaturePair( b3_featureShapeA, 1, b3_featureShapeA, 1 );

	int pointCount = b3ClipSegmentToHullFace( segmentB, hullA, refFace );
	if ( pointCount < 2 )
	{
		return false;
	}

	b3f distance1 = b3PlaneSeparation( refPlane, segmentB[0].position );
	b3f distance2 = b3PlaneSeparation( refPlane, segmentB[1].position );

	if ( b3Raw( distance1 ) <= b3Raw( B3_SPECULATIVE_DISTANCE ) || b3Raw( distance2 ) <= b3Raw( B3_SPECULATIVE_DISTANCE ) )
	{
		b3Vec3 normal = refPlane.normal;
		b3Vec3 point1 = b3MulSub( segmentB[0].position, b3MulFC( b3AddF( distance1, capsuleB->radius ), b3cFromFrac( 1, 2 ) ), normal );
		b3Vec3 point2 = b3MulSub( segmentB[1].position, b3MulFC( b3AddF( distance2, capsuleB->radius ), b3cFromFrac( 1, 2 ) ), normal );

		manifold->normal = normal;
		manifold->pointCount = 2;

		b3LocalManifoldPoint* pt1 = manifold->points + 0;
		pt1->point = point1;
		pt1->separation = b3SubF( distance1, capsuleB->radius );
		pt1->pair = segmentB[0].pair;
		pt1->triangleIndex = B3_NULL_INDEX;

		b3LocalManifoldPoint* pt2 = manifold->points + 1;
		pt2->point = point2;
		pt2->separation = b3SubF( distance2, capsuleB->radius );
		pt2->pair = segmentB[1].pair;
		pt2->triangleIndex = B3_NULL_INDEX;

		return true;
	}

	return false;
}

/// One contact point where a hull edge crosses the capsule segment.
static bool b3BuildHullAndCapsuleEdgeContact( b3LocalManifold* manifold, int capacity, const b3HullData* hullA,
											  const b3Capsule* capsuleB, b3Transform transformBtoA, b3SeparatingAxis query )
{
	if ( capacity < 1 )
	{
		return false;
	}

	b3Vec3 pc = b3TransformPoint( transformBtoA, capsuleB->center1 );
	b3Vec3 qc = b3TransformPoint( transformBtoA, capsuleB->center2 );
	b3Vec3 ec = b3Sub( qc, pc );

	const b3HullHalfEdge* edges = b3GetHullEdges( hullA );
	const b3Vec3* points = b3GetHullPoints( hullA );

	const b3HullHalfEdge* edge2 = edges + query.indexB;
	const b3HullHalfEdge* twin2 = edges + edge2->twin;
	b3Vec3 ph = points[edge2->origin];
	b3Vec3 qh = points[twin2->origin];
	b3Vec3 eh = b3Sub( qh, ph );

	b3Vec3 normal = query.normal;

	b3SegmentDistanceResult result = b3LineDistance( ph, eh, pc, ec );

	if ( b3IsWithinSegments( &result ) == false )
	{
		// The closest approach is past an end of one of the segments, so this
		// is not really an edge-edge feature.
		return false;
	}

	b3Vec3 point =
		b3MulCV( b3cFromFrac( 1, 2 ), b3Add( b3MulSub( result.point1, capsuleB->radius, normal ), result.point2 ) );
	b3f separation = b3Dot( normal, b3Sub( result.point2, result.point1 ) );

	manifold->normal = normal;
	manifold->pointCount = 1;

	b3LocalManifoldPoint* pt = manifold->points + 0;
	pt->point = point;
	pt->separation = b3SubF( separation, capsuleB->radius );
	pt->pair = b3MakeFeaturePair( b3_featureShapeA, query.indexA, b3_featureShapeB, query.indexB );
	pt->triangleIndex = B3_NULL_INDEX;
	return true;
}

void b3CollideHullAndCapsule( b3LocalManifold* manifold, int capacity, const b3HullData* hullA, const b3Capsule* capsuleB,
							  b3Transform transformBtoA, b3SimplexCache* cache )
{
	manifold->pointCount = 0;

	if ( capacity < 2 )
	{
		return;
	}

	// Work in shapeA coordinates.
	b3DistanceInput distanceInput;
	distanceInput.proxyA = ( b3ShapeProxy ){ b3GetHullPoints( hullA ), hullA->vertexCount, b3f_zero };
	distanceInput.proxyB = ( b3ShapeProxy ){ &capsuleB->center1, 2, b3f_zero };
	distanceInput.transform = transformBtoA;
	distanceInput.useRadii = false;

	b3DistanceOutput distanceOutput = b3ShapeDistance( &distanceInput, cache, NULL, 0 );

	b3f radius = capsuleB->radius;

	if ( b3Raw( distanceOutput.distance ) > b3Raw( b3AddF( radius, B3_SPECULATIVE_DISTANCE ) ) )
	{
		// Separated.
		*cache = ( b3SimplexCache ){ 0 };
		return;
	}

	// Same shallow/deep split as b3CollideHullAndSphere: a distance of zero is
	// the fixed-point statement of upstream's 100 * FLT_EPSILON.
	if ( b3Raw( distanceOutput.distance ) > 0 )
	{
		const b3Plane* planes = b3GetHullPlanes( hullA );

		// Shallow penetration.
		b3Vec3 delta = distanceOutput.normal;
		int refFace = b3FindHullSupportFace( hullA, delta );
		b3Plane refPlane = planes[refFace];

		// Two points if the closest-point direction is nearly the face normal.
		//
		// Upstream compares the dot against 0.998. Two Q12 unit vectors dot to
		// Q12, where 1 - 0.998 is eight quanta and the test would be decided by
		// rounding; taking the dot wide puts the same threshold at Q24, where
		// it is 16744 quanta and means what it says.
		const int64_t kToleranceWide = 16743661; // 0.998 at Q24
		int64_t alignment = b3DotWide( refPlane.normal, delta );
		if ( ( alignment < 0 ? -alignment : alignment ) > kToleranceWide )
		{
			b3ClipVertex verticesB[2];
			verticesB[0].position = b3TransformPoint( transformBtoA, capsuleB->center1 );
			verticesB[0].separation = b3f_zero;
			verticesB[0].pair = b3MakeFeaturePair( b3_featureShapeA, 0, b3_featureShapeA, 0 );
			verticesB[1].position = b3TransformPoint( transformBtoA, capsuleB->center2 );
			verticesB[1].separation = b3f_zero;
			verticesB[1].pair = b3MakeFeaturePair( b3_featureShapeA, 1, b3_featureShapeA, 1 );

			int pointCount = b3ClipSegmentToHullFace( verticesB, hullA, refFace );

			if ( pointCount == 2 )
			{
				b3f distance1 = b3PlaneSeparation( refPlane, verticesB[0].position );
				b3f distance2 = b3PlaneSeparation( refPlane, verticesB[1].position );
				b3f maxDistance = b3AddF( radius, B3_SPECULATIVE_DISTANCE );

				if ( b3Raw( distance1 ) <= b3Raw( maxDistance ) || b3Raw( distance2 ) <= b3Raw( maxDistance ) )
				{
					b3Vec3 normal = refPlane.normal;
					b3Vec3 point1 = b3MulSub( verticesB[0].position, b3MulFC( b3AddF( radius, distance1 ), b3cFromFrac( 1, 2 ) ), normal );
					b3Vec3 point2 = b3MulSub( verticesB[1].position, b3MulFC( b3AddF( radius, distance2 ), b3cFromFrac( 1, 2 ) ), normal );

					manifold->normal = normal;
					manifold->pointCount = 2;

					b3LocalManifoldPoint* pt1 = manifold->points + 0;
					pt1->point = point1;
					pt1->separation = b3SubF( distance1, radius );
					pt1->pair = verticesB[0].pair;
					pt1->triangleIndex = B3_NULL_INDEX;

					b3LocalManifoldPoint* pt2 = manifold->points + 1;
					pt2->point = point2;
					pt2->separation = b3SubF( distance2, radius );
					pt2->pair = verticesB[1].pair;
					pt2->triangleIndex = B3_NULL_INDEX;

					return;
				}
			}
		}

		// Otherwise one point, from the closest points.
		manifold->normal = delta;
		manifold->pointCount = 1;

		b3LocalManifoldPoint* pt = manifold->points + 0;
		pt->point = b3MulCV( b3cFromFrac( 1, 2 ),
							 b3Add( b3MulSub( distanceOutput.pointA, radius, delta ), distanceOutput.pointB ) );
		pt->separation = b3SubF( distanceOutput.distance, radius );
		pt->pair = b3FeaturePair_single;
		pt->triangleIndex = B3_NULL_INDEX;
		return;
	}

	// Deep penetration: GJK has nothing to say, so fall back to SAT over the
	// hull's faces and its edges against the capsule segment.
	b3SeparatingAxis faceQuery = b3QueryFaceDirectionHullAndCapsule( hullA, capsuleB, transformBtoA );
	if ( b3Raw( faceQuery.separation ) > b3Raw( radius ) )
	{
		return;
	}

	b3SeparatingAxis edgeQuery = b3QueryEdgeDirectionHullAndCapsule( hullA, capsuleB, transformBtoA );
	if ( edgeQuery.indexB != B3_NULL_INDEX && b3Raw( edgeQuery.separation ) > b3Raw( radius ) )
	{
		return;
	}

	b3f faceSeparation = b3SubF( faceQuery.separation, radius );
	b3BuildHullFaceAndCapsuleContact( manifold, hullA, capsuleB, transformBtoA, faceQuery );

	if ( manifold->pointCount == 2 )
	{
		// The clipped points give a better separation than the face query did.
		faceSeparation = b3MinF( manifold->points[0].separation, manifold->points[1].separation );
	}

	if ( edgeQuery.indexB == B3_NULL_INDEX )
	{
		// Every hull edge was parallel to the capsule; there is no edge axis.
		return;
	}

	// A face contact can come out empty when it does not realize the axis of
	// minimum penetration. Take the edge contact when that happens, or when
	// the edge axis is meaningfully better.
	b3f edgeSeparation = b3SubF( edgeQuery.separation, radius );
	if ( manifold->pointCount == 0 || b3Raw( edgeSeparation ) > b3Raw( b3AddF( faceSeparation, B3_LINEAR_SLOP ) ) )
	{
		b3BuildHullAndCapsuleEdgeContact( manifold, capacity, hullA, capsuleB, transformBtoA, edgeQuery );
	}
}

// =========================================================================
// Hull vs hull: the separating axis test
// =========================================================================
//
// Upstream's b3ComputeSeparatingAxis is a SIMD design after Cairn Overturf's
// article, and two of its structural choices exist only to serve four lanes:
//
//   - Twenty-five stack arrays of B3_MAX_HULL_* + 4 floats, 16368 bytes, so
//     that every per-edge quantity is a contiguous stream. Every one of them
//     is a gather from data the hull already holds. Scalar code reads the
//     half-edge tables directly, and the only arrays left are B's face
//     normals and B's vertices in A's frame: 768 bytes on a machine whose
//     whole stack is measured in kilobytes.
//
//   - b3GetSupportWide, which embeds a vertex index in the low mantissa bits
//     of a float so a horizontal minimum returns the argmin and its index
//     together. That needs the compared value to be positive, which is what
//     the 1.0625f bias and the hull AABB prologue are for, and it is why
//     upstream asserts B3_MAX_HULL_VERTICES == 128 (seven index bits). Scalar,
//     an argmax is just an argmax: b3GetPointSupport already is one, ties
//     already fall to the lower index, and the bias, the assert and
//     B3_HULL_BIT_COUNT all go together.
//
// What is kept exactly is the loop order -- B's edges outside, A's inside --
// because ties are resolved by which pair is seen first, and the harness
// compares against upstream case for case.

/// Both hulls' face axes and all of their edge-pair axes.
b3AxisQuery b3ComputeSeparatingAxis( const b3HullData* hullA, const b3HullData* hullB, b3Transform transformBtoA,
									 bool earlyReturn )
{
	b3AxisQuery res;
	res.faceA = ( b3SeparatingAxis ){ b3Vec3_zero, b3f_zero, B3_NULL_INDEX, B3_NULL_INDEX, b3_faceAxisA };
	res.faceB = ( b3SeparatingAxis ){ b3Vec3_zero, b3f_zero, B3_NULL_INDEX, B3_NULL_INDEX, b3_faceAxisB };
	res.edge = ( b3SeparatingAxis ){ b3Vec3_zero, b3f_zero, B3_NULL_INDEX, B3_NULL_INDEX, b3_edgePairAxis };
	res.separatedFeature = b3_invalidAxis;

	// Every separation below is held at Q24 and narrowed once, on the way out.
	// The margin between two face axes can be a single Q12 quantum, and
	// narrowing inside the loop would let rounding pick the reference face.
	const int64_t speculative = (int64_t)b3Raw( B3_SPECULATIVE_DISTANCE ) << B3_F_SHIFT;

	int64_t bestFaceA = 0;
	int64_t bestFaceB = 0;
	int64_t bestEdge = 0;

	const b3Plane* planesA = b3GetHullPlanes( hullA );
	const b3Plane* planesB = b3GetHullPlanes( hullB );
	const b3Vec3* pointsA = b3GetHullPoints( hullA );
	const b3Vec3* pointsB = b3GetHullPoints( hullB );

	// --- A's face planes against B's vertices -----------------------------

	for ( int i = 0; i < hullA->faceCount; ++i )
	{
		b3Plane plane = planesA[i];

		// The support is taken in B's *local* frame against a rotated
		// direction, which is not merely cheaper than rotating B's points: the
		// points are exactly the values that were baked, so the argmax is
		// decided by the hull's own geometry and one rotated direction rather
		// than by up to 32 independently rounded rotations.
		b3Vec3 direction = b3Neg( b3InvRotateVector( transformBtoA.q, plane.normal ) );

		int vertexIndex = b3GetPointSupport( pointsB, hullB->vertexCount, direction );

		// separation = dot(n, p) - offset - dot(-invR*n, vB)
		//
		// Ranges: a Q12 unit normal is at most 4096 raw per component and
		// transformBtoA.p is bounded by B3_HUGE, so the dot reaches 1.0e11 and
		// the offset term 1.3e9. Both are far inside int64.
		int64_t planeSeparation =
			b3DotWide( plane.normal, transformBtoA.p ) - ( (int64_t)b3Raw( plane.offset ) << B3_F_SHIFT );
		int64_t separation = planeSeparation - b3DotWide( direction, pointsB[vertexIndex] );

		if ( res.faceA.indexA == B3_NULL_INDEX || separation > bestFaceA )
		{
			bestFaceA = separation;
			res.faceA.normal = plane.normal;
			res.faceA.indexA = i;
			res.faceA.indexB = vertexIndex;

			if ( separation > speculative && earlyReturn )
			{
				res.faceA.separation = b3Makeb3fRef( (int32_t)B3_SHIFT_ROUND( separation, B3_F_SHIFT ),
													 B3_REF( (double)separation / 16777216.0 ) );
				res.separatedFeature = b3_faceAxisA;
				return res;
			}
		}
	}

	res.faceA.separation =
		b3Makeb3fRef( (int32_t)B3_SHIFT_ROUND( bestFaceA, B3_F_SHIFT ), B3_REF( (double)bestFaceA / 16777216.0 ) );

	// --- B's face planes against A's vertices -----------------------------

	for ( int i = 0; i < hullB->faceCount; ++i )
	{
		b3Plane plane = planesB[i];

		// This one *is* the axis: it is in A's frame and points from A to B.
		//
		// It is a rotated Q12 unit, so it is a raw unit or two off unit
		// length, and it is deliberately not renormalized. Nothing downstream
		// reads it -- b3BuildFaceBContact flips the problem round and uses the
		// swapped hull's own plane normal, which is exactly unit as baked.
		b3Vec3 direction = b3Neg( b3RotateVector( transformBtoA.q, plane.normal ) );

		int vertexIndex = b3GetPointSupport( pointsA, hullA->vertexCount, direction );

		int64_t separation = b3DotWide( direction, b3Sub( transformBtoA.p, pointsA[vertexIndex] ) ) -
							 ( (int64_t)b3Raw( plane.offset ) << B3_F_SHIFT );

		if ( res.faceB.indexB == B3_NULL_INDEX || separation > bestFaceB )
		{
			bestFaceB = separation;
			res.faceB.normal = direction;
			res.faceB.indexA = vertexIndex;
			res.faceB.indexB = i;

			if ( separation > speculative && earlyReturn )
			{
				res.faceB.separation = b3Makeb3fRef( (int32_t)B3_SHIFT_ROUND( separation, B3_F_SHIFT ),
													 B3_REF( (double)separation / 16777216.0 ) );
				res.separatedFeature = b3_faceAxisB;
				return res;
			}
		}
	}

	res.faceB.separation =
		b3Makeb3fRef( (int32_t)B3_SHIFT_ROUND( bestFaceB, B3_F_SHIFT ), B3_REF( (double)bestFaceB / 16777216.0 ) );

	// --- edge pairs -------------------------------------------------------

	// The only scratch this function keeps: B's face normals rotated into A's
	// frame and negated, and B's vertices transformed into A's frame and
	// negated. 768 bytes in device mode, against upstream's 16368. Built after
	// the face phases so an early return skips the work entirely, which in a
	// broad-phase-filtered world is the common case.
	b3Vec3 bFN[B3_MAX_HULL_FACES];
	b3Vec3 bW[B3_MAX_HULL_VERTICES];

	B3_ASSERT( hullB->faceCount <= B3_MAX_HULL_FACES && hullB->vertexCount <= B3_MAX_HULL_VERTICES );

	for ( int i = 0; i < hullB->faceCount; ++i )
	{
		bFN[i] = b3Neg( b3RotateVector( transformBtoA.q, planesB[i].normal ) );
	}

	for ( int i = 0; i < hullB->vertexCount; ++i )
	{
		bW[i] = b3Neg( b3TransformPoint( transformBtoA, pointsB[i] ) );
	}

	const b3HullHalfEdge* edgesA = b3GetHullEdges( hullA );
	const b3HullHalfEdge* edgesB = b3GetHullEdges( hullB );

	// tol^2 = 0.005^2 = 2.5e-5, which is 26843 at Q30. This is
	// B3_PARALLEL_EDGE_TOL squared -- 5368709^2 >> 30 -- and the same literal
	// b3QueryEdgeDirectionHullAndCapsule uses.
	const int64_t parallelTolSq = 26843;

	for ( int j = 0; j < hullB->edgeCount; j += 2 )
	{
		const b3HullHalfEdge* edgeB = edgesB + j;
		const b3HullHalfEdge* twinB = edgesB + j + 1;
		B3_ASSERT( edgeB->twin == j + 1 );

		b3Vec3 c = bFN[edgeB->face];
		b3Vec3 d = bFN[twinB->face];
		b3Vec3 bv0 = bW[edgeB->origin];
		b3Vec3 dc = b3Sub( bW[twinB->origin], bv0 );

		for ( int i = 0; i < hullA->edgeCount; i += 2 )
		{
			const b3HullHalfEdge* edgeA = edgesA + i;
			const b3HullHalfEdge* twinA = edgesA + i + 1;

			b3Vec3 av0 = pointsA[edgeA->origin];
			b3Vec3 dir = b3Sub( pointsA[twinA->origin], av0 );

			int64_t cba = b3DotWide( c, dir );
			int64_t dba = b3DotWide( d, dir );

			// Upstream writes `cba * dba >= EPS` with EPS = -0.0001f, asking
			// two things at once: do the signs differ, and is the product big
			// enough not to be noise. The port answers them separately,
			// because the product cannot be formed -- cba is a length at Q24
			// and reaches 2e11 for a long edge, so cba*dba reaches 4e22, four
			// times past int64, and it would wrap to an arbitrary sign. The
			// obvious escape, __int128, lowers to __multi3, a libgcc helper
			// this build must not link.
			//
			// The sign half is exact and free. The noise half is what the
			// parallel-edge test below already does, and does better: EPS is
			// an absolute threshold on a quantity carrying units of length, so
			// it means different things for a 0.1-unit edge and a 10-unit one,
			// whereas tol^2 * |dir|^2 does not.
			//
			// What this widens is the band where a product falls in
			// (-1e-4, 0): there one of cba, dba is essentially zero, so the
			// crossing parameter sits at an end and the axis that comes out is
			// one of B's own face normals -- an axis the face-B phase has
			// already scored. Accepting it costs nothing.
			if ( ( cba < 0 ) == ( dba < 0 ) || cba == 0 || dba == 0 )
			{
				continue;
			}

			b3Vec3 n0 = planesA[edgeA->face].normal;
			b3Vec3 n1 = planesA[twinA->face].normal;

			int64_t adc = b3DotWide( n0, dc );
			int64_t bdc = b3DotWide( n1, dc );

			if ( ( adc < 0 ) == ( bdc < 0 ) || adc == 0 || bdc == 0 )
			{
				continue;
			}

			// The third arc test. cba and bdc are both known non-zero here.
			if ( ( cba < 0 ) == ( bdc < 0 ) )
			{
				continue;
			}

			// Reject near-parallel edges, where quantization rather than
			// geometry would decide the separation. Both sides are squared
			// lengths, so both belong at Q24 -- but cba at Q24 reaches 2e11
			// and squaring that overflows, so the magnitudes come down to Q12
			// first, as int64 rather than b3f so a long edge cannot clamp.
			int64_t cbaN = cba >> B3_F_SHIFT;
			int64_t dbaN = dba >> B3_F_SHIFT;
			int64_t maxSq = cbaN * cbaN;
			int64_t otherSq = dbaN * dbaN;
			if ( otherSq > maxSq )
			{
				maxSq = otherSq;
			}

			// tol^2 * |dir|^2, at Q24. |dir|^2 reaches 7.7e14 for the longest
			// vector Q12 holds, and 7.7e14 * 26843 is 2.1e19 -- past int64.
			// Shifting six bits out of the squared length first buys 64x of
			// headroom and costs six bits of a quantity being compared against
			// a 2.5e-5 fraction of itself.
			int64_t limit = ( ( b3LengthSquaredWide( dir ) >> 6 ) * parallelTolSq ) >> ( B3_C_SHIFT - 6 );
			if ( maxSq <= limit )
			{
				continue;
			}

			// The Gauss arc from c to d crosses the plane of A's edge at
			// t = -cba / (dba - cba). The signs strictly differ, so the
			// denominator dominates and t lands in (0, 1). Both operands are
			// int64 at Q24, which is what b3DivWideToC is for -- writing the
			// division out would link __divdi3.
			b3c t = b3DivWideToC( -cba, dba - cba );

			// axis = c + t*(d - c), kept wide.
			//
			// b3Lerp would narrow this to Q12, and that is the same mistake
			// b3Cross makes on a short-edge normal: when c and d are nearly
			// opposed -- a near-degenerate dihedral -- the lerp cancels to a
			// handful of raw units and b3Normalize then returns a direction
			// made entirely of rounding. Q12 vector times Q30 coefficient is
			// Q42, worst case 4096 * 2^30 * 2 = 8.8e12.
			int64_t one = (int64_t)B3_C_ONE;
			int64_t tw = (int64_t)b3Raw( t );
			int64_t nx = (int64_t)b3Raw( c.x ) * ( one - tw ) + (int64_t)b3Raw( d.x ) * tw;
			int64_t ny = (int64_t)b3Raw( c.y ) * ( one - tw ) + (int64_t)b3Raw( d.y ) * tw;
			int64_t nz = (int64_t)b3Raw( c.z ) * ( one - tw ) + (int64_t)b3Raw( d.z ) * tw;

			b3Vec3 axis = b3DirectionFromWide( nx, ny, nz );
			if ( b3LengthSquaredWide( axis ) == 0 )
			{
				// Exactly opposed face normals. Upstream asserts this away;
				// here it is a real outcome once the dihedral is quantized,
				// and the honest answer is that this pair defines no axis.
				continue;
			}

			axis = b3Normalize( axis );

			// bW holds B's vertices negated, so av0 + bv0 is pA - pB. The axis
			// is a blend of B's negated face normals, so it already points
			// from A to B and needs no orienting.
			int64_t separation = -b3DotWide( axis, b3Add( av0, bv0 ) );

			if ( res.edge.indexA == B3_NULL_INDEX || separation > bestEdge )
			{
				bestEdge = separation;
				res.edge.normal = axis;
				res.edge.indexA = i;
				res.edge.indexB = j;

				if ( separation > speculative && earlyReturn )
				{
					res.edge.separation = b3Makeb3fRef( (int32_t)B3_SHIFT_ROUND( separation, B3_F_SHIFT ),
														B3_REF( (double)separation / 16777216.0 ) );
					res.separatedFeature = b3_edgePairAxis;
					return res;
				}
			}
		}
	}

	res.edge.separation =
		b3Makeb3fRef( (int32_t)B3_SHIFT_ROUND( bestEdge, B3_F_SHIFT ), B3_REF( (double)bestEdge / 16777216.0 ) );

	return res;
}

// =========================================================================
// Hull vs hull: the contact builders
// =========================================================================

/// The incident face as a polygon of clip vertices, in the reference frame.
static int b3BuildPolygon( b3ClipVertex* out, b3Transform transform, const b3HullData* hull, int incFace,
						   b3Plane refPlane )
{
	const b3HullFace* faces = b3GetHullFaces( hull );
	const b3HullHalfEdge* edges = b3GetHullEdges( hull );
	const b3Vec3* points = b3GetHullPoints( hull );

	const b3HullFace* face = faces + incFace;
	int baseEdgeIndex = face->edge;
	B3_ASSERT( edges[baseEdgeIndex].face == incFace );

	int edgeIndex = baseEdgeIndex;
	int outCount = 0;

	do
	{
		const b3HullHalfEdge* edge = edges + edgeIndex;
		int nextEdgeIndex = edge->next;
		const b3HullHalfEdge* next = edges + nextEdgeIndex;

		if ( outCount >= B3_MAX_CLIP_POINTS )
		{
			// Upstream stops here and returns what it has. That leaves an
			// *unclosed* polygon: the last vertex's outgoing edge no longer
			// matches the first vertex's incoming one, so the feature-pair
			// chain is broken and every id built from it is a fresh contact
			// the solver has never seen. Failing is the smaller harm.
			return B3_NULL_INDEX;
		}

		b3ClipVertex vertex;

		// b3TransformPoint rather than upstream's precomputed b3Matrix3 and
		// b3MulMV. b3MakeMatrixFromQuat narrows a Q30 quaternion into Q12
		// columns before it ever touches the vector, throwing away eighteen
		// bits of orientation, and every other transform in the port goes
		// through b3TransformPoint. The matrix form is cheaper above two
		// points -- b3RotateVector is fifteen multiplies against nine plus a
		// sixteen-multiply matrix build -- so it is worth revisiting if this
		// shows up in a device profile, but not on a guess.
		vertex.position = b3TransformPoint( transform, points[next->origin] );
		vertex.separation = b3PlaneSeparation( refPlane, vertex.position );
		vertex.pair = b3MakeFeaturePair( b3_featureShapeB, edgeIndex, b3_featureShapeB, nextEdgeIndex );

		out[outCount] = vertex;
		outCount += 1;

		edgeIndex = nextEdgeIndex;
	}
	while ( edgeIndex != baseEdgeIndex );

	B3_ASSERT( b3ValidatePolygon( out, outCount ) );

	return outCount;
}

/// Contact points from B's incident face clipped to A's reference face.
static bool b3BuildFaceAContact( b3LocalManifold* manifold, int capacity, const b3HullData* hullA,
								 const b3HullData* hullB, b3Transform transformBtoA, b3SeparatingAxis query,
								 b3SATCache* cache )
{
	B3_ASSERT( query.type == b3_faceAxisA );
	B3_ASSERT( 0 <= query.indexA && query.indexA < hullA->faceCount );
	B3_ASSERT( 0 <= query.indexB && query.indexB < hullB->vertexCount );

	const b3HullFace* facesA = b3GetHullFaces( hullA );
	const b3HullHalfEdge* edgesA = b3GetHullEdges( hullA );
	const b3Plane* planesA = b3GetHullPlanes( hullA );
	const b3Vec3* pointsA = b3GetHullPoints( hullA );

	int refFace = query.indexA;
	b3Plane refPlane = planesA[refFace];

	b3Vec3 refNormalInB = b3InvRotateVector( transformBtoA.q, refPlane.normal );
	int incFace = b3FindIncidentFace( hullB, refNormalInB, query.indexB );

	b3ClipVertex buffer1[B3_MAX_CLIP_POINTS];
	b3ClipVertex buffer2[B3_MAX_CLIP_POINTS];

	int pointCount = b3BuildPolygon( buffer1, transformBtoA, hullB, incFace, refPlane );
	if ( pointCount == B3_NULL_INDEX || pointCount < 3 )
	{
		*cache = ( b3SATCache ){ 0 };
		return false;
	}

	b3ClipVertex* input = buffer1;
	b3ClipVertex* output = buffer2;

	const b3HullFace* face = facesA + refFace;
	int baseEdgeIndex = face->edge;
	int edgeIndex = baseEdgeIndex;
	int guard = 0;

	do
	{
		const b3HullHalfEdge* edge = edgesA + edgeIndex;
		int nextEdgeIndex = edge->next;
		const b3HullHalfEdge* next = edgesA + nextEdgeIndex;

		b3Vec3 vertex1 = pointsA[edge->origin];
		b3Vec3 vertex2 = pointsA[next->origin];
		b3Vec3 tangent = b3Sub( vertex2, vertex1 );

		// The side plane is deliberately not normalized: b3ClipPolygon reads
		// it only through sign tests and the ratio d1 / (d1 - d2), both
		// invariant under scaling it, while the separations it records come
		// from refPlane, which is a hull face plane and is unit by
		// construction. That saves a square root per reference edge and avoids
		// rounding a unit vector into Q12.
		//
		// Unlike b3ClipSegmentToHullFace, which crosses narrow, this goes
		// through b3CrossDirection. It fixes both ends of the range problem
		// recorded there: a short face edge no longer crosses to zero, and a
		// long one no longer scales the plane offset out of Q12, since the
		// binormal comes back at a fixed magnitude near one. (The older path
		// still carries its ~500 units squared limit; retrofitting it was left
		// for its own change.)
		b3Vec3 binormal = b3CrossDirection( tangent, refPlane.normal );
		b3Plane clipPlane = b3MakePlaneFromNormalAndPoint( binormal, vertex1 );

		pointCount = b3ClipPolygon( output, input, pointCount, clipPlane, edgeIndex, refPlane );

		b3ClipVertex* swap = output;
		output = input;
		input = swap;

		if ( pointCount == B3_NULL_INDEX || pointCount < 3 )
		{
			*cache = ( b3SATCache ){ 0 };
			return false;
		}

		edgeIndex = nextEdgeIndex;

		// See b3ClipSegmentToHullFace: the ring closes only if hullA's `next`
		// chain is well formed, and a hull that arrives malformed would
		// otherwise spin here forever with nothing to name the loop.
		if ( ++guard > hullA->edgeCount )
		{
			break;
		}
	}
	while ( edgeIndex != baseEdgeIndex );

	b3LocalManifoldPoint points[B3_MAX_CLIP_POINTS];

	// Seeded from the first point rather than from B3_F_MAX: the clip loop
	// above already guarantees at least three survive, so there is a real
	// value to start from.
	b3f minSeparation = input[0].separation;

	manifold->normal = refPlane.normal;

	for ( int index = 0; index < pointCount; ++index )
	{
		const b3ClipVertex* clipPoint = input + index;
		b3LocalManifoldPoint* pt = points + index;

		// Half way between the two surfaces, so the point does not move when
		// the reference face swaps from A to B. Halved with b3MulFC rather
		// than a shift: an arithmetic shift of a negative separation rounds
		// toward minus infinity, and separations here are usually negative.
		b3Vec3 point =
			b3MulSub( clipPoint->position, b3MulFC( clipPoint->separation, b3cFromFrac( 1, 2 ) ), refPlane.normal );

		pt->point = point;
		pt->separation = clipPoint->separation;
		pt->pair = clipPoint->pair;
		pt->triangleIndex = B3_NULL_INDEX;

		minSeparation = b3MinF( minSeparation, clipPoint->separation );
	}

	if ( b3Raw( minSeparation ) >= b3Raw( B3_SPECULATIVE_DISTANCE ) )
	{
		*cache = ( b3SATCache ){ 0 };
		return false;
	}

	b3ReduceManifoldPoints( manifold, capacity, points, pointCount );

	cache->separation = minSeparation;
	cache->type = (uint8_t)b3_faceAxisA;
	cache->indexA = (uint8_t)query.indexA;
	cache->indexB = (uint8_t)query.indexB;

	return true;
}

/// The same, with B supplying the reference face: solve it in B's frame and
/// bring the answer back.
static bool b3BuildFaceBContact( b3LocalManifold* manifold, int capacity, const b3HullData* hullA,
								 const b3HullData* hullB, b3Transform transformBtoA, b3SeparatingAxis query,
								 b3SATCache* cache )
{
	B3_ASSERT( query.type == b3_faceAxisB );

	b3Transform transformAtoB = b3InvertTransform( transformBtoA );

	b3SeparatingAxis flippedQuery;
	flippedQuery.normal = b3Neg( query.normal );
	flippedQuery.separation = query.separation;
	flippedQuery.indexA = query.indexB;
	flippedQuery.indexB = query.indexA;
	flippedQuery.type = b3_faceAxisA;

	bool touching = b3BuildFaceAContact( manifold, capacity, hullB, hullA, transformAtoB, flippedQuery, cache );
	if ( touching == false )
	{
		*cache = ( b3SATCache ){ 0 };
		return false;
	}

	// Renormalizing after the rotation is not in upstream and is not optional
	// here. manifold->normal at this point is a hull face plane normal, which
	// is exactly unit as baked; b3RotateVector returns Q12, so the rotated
	// copy is a raw unit or two off -- about 5e-4 relative, which the
	// constraint solver would carry straight into the effective mass.
	manifold->normal = b3Normalize( b3Neg( b3RotateVector( transformBtoA.q, manifold->normal ) ) );

	for ( int index = 0; index < manifold->pointCount; ++index )
	{
		b3LocalManifoldPoint* pt = manifold->points + index;
		pt->point = b3TransformPoint( transformBtoA, pt->point );
		pt->pair = b3FlipPair( pt->pair );
	}

	// The nested call wrote the swapped indices; this puts them back in A/B
	// order. cache->separation is deliberately left as the nested call set it.
	cache->type = (uint8_t)b3_faceAxisB;
	cache->indexA = (uint8_t)query.indexA;
	cache->indexB = (uint8_t)query.indexB;

	return true;
}

/// One contact point where two hull edges cross. Always one point, so this
/// takes no capacity.
static bool b3BuildEdgeContact( b3LocalManifold* manifold, const b3HullData* hullA, const b3HullData* hullB,
								b3Transform transformBtoA, b3SeparatingAxis query, b3SATCache* cache )
{
	B3_ASSERT( query.type == b3_edgePairAxis );
	B3_ASSERT( 0 <= query.indexA && query.indexA < hullA->edgeCount );
	B3_ASSERT( 0 <= query.indexB && query.indexB < hullB->edgeCount );

	const b3HullHalfEdge* edgesA = b3GetHullEdges( hullA );
	const b3Vec3* pointsA = b3GetHullPoints( hullA );
	const b3HullHalfEdge* edgesB = b3GetHullEdges( hullB );
	const b3Vec3* pointsB = b3GetHullPoints( hullB );

	const b3HullHalfEdge* edgeA = edgesA + query.indexA;
	const b3HullHalfEdge* twinA = edgesA + edgeA->twin;
	b3Vec3 pA = pointsA[edgeA->origin];
	b3Vec3 eA = b3Sub( pointsA[twinA->origin], pA );

	const b3HullHalfEdge* edgeB = edgesB + query.indexB;
	const b3HullHalfEdge* twinB = edgesB + edgeB->twin;
	b3Vec3 pB = b3TransformPoint( transformBtoA, pointsB[edgeB->origin] );
	b3Vec3 qB = b3TransformPoint( transformBtoA, pointsB[twinB->origin] );
	b3Vec3 eB = b3Sub( qB, pB );

	b3Vec3 normal = query.normal;
	b3SegmentDistanceResult result = b3LineDistance( pA, eA, pB, eB );

	if ( b3IsWithinSegments( &result ) == false )
	{
		// Caching can slide the closest points off the ends of the segments.
		*cache = ( b3SATCache ){ 0 };
		return false;
	}

	// Wide, then narrowed once: the gap between two crossing edges is short by
	// construction -- that is what an edge contact is -- so the low bits are
	// the whole of the answer.
	b3f separation = b3SeparationFromWide( b3DotWide( normal, b3Sub( result.point2, result.point1 ) ) );
	b3Vec3 point = b3MulCV( b3cFromFrac( 1, 2 ), b3Add( result.point1, result.point2 ) );

	manifold->normal = normal;
	manifold->pointCount = 1;

	b3LocalManifoldPoint* pt = manifold->points + 0;
	pt->point = point;
	pt->separation = separation;
	pt->pair = b3MakeFeaturePair( b3_featureShapeA, query.indexA, b3_featureShapeB, query.indexB );
	pt->triangleIndex = B3_NULL_INDEX;

	cache->separation = separation;
	cache->type = (uint8_t)b3_edgePairAxis;
	cache->indexA = (uint8_t)query.indexA;
	cache->indexB = (uint8_t)query.indexB;

	return true;
}

// =========================================================================
// Hull vs hull
// =========================================================================

/// Re-run one cached edge pair and, if it still crosses, rebuild its axis.
///
/// This is the cache path's copy of the edge phase, and it is deliberately
/// *not* shared with the one inside b3ComputeSeparatingAxis. The two use
/// opposite sign conventions: there C, D and DC are all negated, so all three
/// Gauss tests read "product negative"; here the normals are un-negated and
/// upstream's tests are `cba*dba < 0 && adc*bdc < 0 && cba*bdc > 0` -- the
/// third one the other way round. Folding them together would mean carrying a
/// sign flag through six comparisons to save a dozen lines.
///
/// @return false if the pair no longer crosses, is too near parallel, or
/// yields no axis -- all of which mean "fall through to the full test".
static bool b3RebuildCachedEdgeAxis( b3Vec3* axisOut, const b3HullData* hullA, const b3HullData* hullB,
									 b3Transform transformBtoA, int indexA, int indexB )
{
	const b3HullHalfEdge* edgesA = b3GetHullEdges( hullA );
	const b3Plane* planesA = b3GetHullPlanes( hullA );
	const b3Vec3* pointsA = b3GetHullPoints( hullA );

	const b3HullHalfEdge* edgesB = b3GetHullEdges( hullB );
	const b3Plane* planesB = b3GetHullPlanes( hullB );
	const b3Vec3* pointsB = b3GetHullPoints( hullB );

	const b3HullHalfEdge* edge1 = edgesA + indexA;
	const b3HullHalfEdge* twin1 = edgesA + indexA + 1;
	B3_ASSERT( edge1->twin == indexA + 1 && twin1->twin == indexA );

	b3Vec3 pA = pointsA[edge1->origin];
	b3Vec3 qA = pointsA[twin1->origin];
	b3Vec3 eA = b3Sub( qA, pA );

	b3Vec3 uA = planesA[edge1->face].normal;
	b3Vec3 vA = planesA[twin1->face].normal;

	const b3HullHalfEdge* edge2 = edgesB + indexB;
	const b3HullHalfEdge* twin2 = edgesB + indexB + 1;
	B3_ASSERT( edge2->twin == indexB + 1 && twin2->twin == indexB );

	b3Vec3 pB = b3TransformPoint( transformBtoA, pointsB[edge2->origin] );
	b3Vec3 qB = b3TransformPoint( transformBtoA, pointsB[twin2->origin] );
	b3Vec3 eB = b3Sub( qB, pB );

	b3Vec3 uB = b3RotateVector( transformBtoA.q, planesB[edge2->face].normal );
	b3Vec3 vB = b3RotateVector( transformBtoA.q, planesB[twin2->face].normal );

	int64_t cba = b3DotWide( uB, eA );
	int64_t dba = b3DotWide( vB, eA );
	int64_t adc = -b3DotWide( uA, eB );
	int64_t bdc = -b3DotWide( vA, eB );

	// Sign tests rather than products, for the reason set out in
	// b3ComputeSeparatingAxis: at Q24 these reach 2e11 and their products
	// 4e22, past int64, and __int128 would link __multi3.
	if ( cba == 0 || dba == 0 || adc == 0 || bdc == 0 )
	{
		return false;
	}

	if ( ( cba < 0 ) == ( dba < 0 ) || ( adc < 0 ) == ( bdc < 0 ) || ( cba < 0 ) != ( bdc < 0 ) )
	{
		return false;
	}

	// Same pre-shift as the SAT: tol^2 * |eA|^2 is a fourth power of length
	// and overflows without it. Note the comparison runs the other way here --
	// upstream accepts on >= where the SAT rejects on <=.
	const int64_t parallelTolSq = 26843;

	int64_t cbaN = cba >> B3_F_SHIFT;
	int64_t dbaN = dba >> B3_F_SHIFT;
	int64_t maxSq = cbaN * cbaN;
	int64_t otherSq = dbaN * dbaN;
	if ( otherSq > maxSq )
	{
		maxSq = otherSq;
	}

	int64_t limit = ( ( b3LengthSquaredWide( eA ) >> 6 ) * parallelTolSq ) >> ( B3_C_SHIFT - 6 );
	if ( maxSq < limit )
	{
		return false;
	}

	// t = cba / (cba - dba). Algebraically the SAT's -cba / (dba - cba), and
	// bounded in (0, 1) by the same argument: the signs differ, so the
	// denominator dominates.
	b3c t = b3DivWideToC( cba, cba - dba );

	// Wide lerp for the reason given in the SAT: a Q12 lerp of two nearly
	// opposed normals cancels to a handful of raw units.
	int64_t one = (int64_t)B3_C_ONE;
	int64_t tw = (int64_t)b3Raw( t );
	int64_t nx = (int64_t)b3Raw( uB.x ) * ( one - tw ) + (int64_t)b3Raw( vB.x ) * tw;
	int64_t ny = (int64_t)b3Raw( uB.y ) * ( one - tw ) + (int64_t)b3Raw( vB.y ) * tw;
	int64_t nz = (int64_t)b3Raw( uB.z ) * ( one - tw ) + (int64_t)b3Raw( vB.z ) * tw;

	b3Vec3 axis = b3DirectionFromWide( nx, ny, nz );
	if ( b3LengthSquaredWide( axis ) == 0 )
	{
		return false;
	}

	*axisOut = b3Normalize( axis );
	return true;
}

void b3CollideHulls( b3LocalManifold* manifold, int capacity, const b3HullData* hullA, const b3HullData* hullB,
					 b3Transform transformBtoA, b3SATCache* cache )
{
	manifold->pointCount = 0;

	if ( capacity < 4 )
	{
		return;
	}

	const b3Plane* planesA = b3GetHullPlanes( hullA );
	const b3Plane* planesB = b3GetHullPlanes( hullB );
	const b3Vec3* pointsA = b3GetHullPoints( hullA );
	const b3Vec3* pointsB = b3GetHullPoints( hullB );

	cache->hit = 0;

	// Every branch below either returns on a cache hit or falls out of the
	// switch into the full separating axis test, so the cache is pure
	// optimization: deleting the whole switch would leave this correct.
	switch ( cache->type )
	{
		case b3_faceAxisA:
		{
			// Upstream asserts the cached index. The port checks it: a
			// b3SATCache is persistent state in the contact table, and a shape
			// swap leaves indices that would read planes past the end of the
			// blob.
			if ( cache->indexA >= hullA->faceCount || cache->indexB >= hullB->vertexCount )
			{
				break;
			}

			b3Plane plane = planesA[cache->indexA];
			b3Vec3 searchDirectionInB = b3Neg( b3InvRotateVector( transformBtoA.q, plane.normal ) );

			int vertexIndex = b3FindHullSupportVertex( hullB, searchDirectionInB );
			b3Vec3 support = b3TransformPoint( transformBtoA, pointsB[vertexIndex] );
			b3f separation = b3PlaneSeparation( plane, support );

			if ( b3Raw( separation ) >= b3Raw( B3_SPECULATIVE_DISTANCE ) )
			{
				// Still separated along the remembered face. No contacts, and
				// no need to look at any other axis.
				cache->hit = 1;
				return;
			}

			b3SeparatingAxis faceQuery;
			faceQuery.normal = plane.normal;
			faceQuery.separation = b3f_zero;
			faceQuery.indexA = cache->indexA;
			faceQuery.indexB = vertexIndex;
			faceQuery.type = b3_faceAxisA;

			b3SATCache localCache = { 0 };
			bool touching = b3BuildFaceAContact( manifold, capacity, hullA, hullB, transformBtoA, faceQuery, &localCache );

			if ( touching && b3Raw( b3AbsF( b3SubF( cache->separation, localCache.separation ) ) ) <
								 b3Raw( B3_LINEAR_SLOP ) )
			{
				cache->hit = 1;
				return;
			}
		}
		break;

		case b3_faceAxisB:
		{
			if ( cache->indexA >= hullA->vertexCount || cache->indexB >= hullB->faceCount )
			{
				break;
			}

			b3Plane plane = planesB[cache->indexB];
			b3Vec3 searchDirectionInA = b3Neg( b3RotateVector( transformBtoA.q, plane.normal ) );

			int vertexIndex = b3FindHullSupportVertex( hullA, searchDirectionInA );
			b3Vec3 support = b3InvTransformPoint( transformBtoA, pointsA[vertexIndex] );
			b3f separation = b3PlaneSeparation( plane, support );

			if ( b3Raw( separation ) >= b3Raw( B3_SPECULATIVE_DISTANCE ) )
			{
				cache->hit = 1;
				return;
			}

			b3SeparatingAxis faceQuery;
			faceQuery.normal = b3Neg( plane.normal );
			faceQuery.separation = b3f_zero;
			faceQuery.indexA = vertexIndex;
			faceQuery.indexB = cache->indexB;
			faceQuery.type = b3_faceAxisB;

			b3SATCache localCache = { 0 };
			bool touching = b3BuildFaceBContact( manifold, capacity, hullA, hullB, transformBtoA, faceQuery, &localCache );

			if ( touching && b3Raw( b3AbsF( b3SubF( cache->separation, localCache.separation ) ) ) <
								 b3Raw( B3_LINEAR_SLOP ) )
			{
				cache->hit = 1;
				return;
			}
		}
		break;

		case b3_edgePairAxis:
		{
			// Edge indices are half-edge indices and must be the even one of a
			// twin pair, which is what the rebuild asserts on.
			if ( cache->indexA + 1 >= hullA->edgeCount || cache->indexB + 1 >= hullB->edgeCount ||
				 ( cache->indexA & 1 ) != 0 || ( cache->indexB & 1 ) != 0 )
			{
				break;
			}

			b3Vec3 axis;
			if ( b3RebuildCachedEdgeAxis( &axis, hullA, hullB, transformBtoA, cache->indexA, cache->indexB ) == false )
			{
				break;
			}

			const b3HullHalfEdge* twinA = b3GetHullEdges( hullA ) + cache->indexA + 1;
			const b3HullHalfEdge* twinB = b3GetHullEdges( hullB ) + cache->indexB + 1;
			b3Vec3 qA = pointsA[twinA->origin];
			b3Vec3 qB = b3TransformPoint( transformBtoA, pointsB[twinB->origin] );

			b3f separation = b3SeparationFromWide( b3DotWide( axis, b3Sub( qA, qB ) ) );

			if ( b3Raw( separation ) > b3Raw( B3_SPECULATIVE_DISTANCE ) )
			{
				cache->hit = 1;
				return;
			}

			b3SeparatingAxis edgeQuery;
			edgeQuery.normal = b3Neg( axis );
			edgeQuery.separation = b3f_zero;
			edgeQuery.indexA = cache->indexA;
			edgeQuery.indexB = cache->indexB;
			edgeQuery.type = b3_edgePairAxis;

			b3SATCache localCache = { 0 };
			bool touching = b3BuildEdgeContact( manifold, hullA, hullB, transformBtoA, edgeQuery, &localCache );

			// Upstream notes this tolerance can dominate some benchmarks.
			if ( touching && b3Raw( b3AbsF( b3SubF( cache->separation, localCache.separation ) ) ) <
								 b3Raw( B3_LINEAR_SLOP ) )
			{
				cache->hit = 1;
				return;
			}
		}
		break;

		// The three b3_manual* types exist for testing: they score every axis
		// and then force one specific builder, so a test can check a builder
		// without also depending on which axis would have won.
		case b3_manualFaceAxisA:
		{
			b3AxisQuery axisQuery = b3ComputeSeparatingAxis( hullA, hullB, transformBtoA, false );
			b3BuildFaceAContact( manifold, capacity, hullA, hullB, transformBtoA, axisQuery.faceA, cache );
			return;
		}

		case b3_manualFaceAxisB:
		{
			b3AxisQuery axisQuery = b3ComputeSeparatingAxis( hullA, hullB, transformBtoA, false );
			b3BuildFaceBContact( manifold, capacity, hullA, hullB, transformBtoA, axisQuery.faceB, cache );
			return;
		}

		case b3_manualEdgePairAxis:
		{
			b3AxisQuery axisQuery = b3ComputeSeparatingAxis( hullA, hullB, transformBtoA, false );
			if ( axisQuery.edge.indexA != B3_NULL_INDEX )
			{
				b3BuildEdgeContact( manifold, hullA, hullB, transformBtoA, axisQuery.edge, cache );
			}
			return;
		}

		case b3_invalidAxis:
		default:
			// Upstream asserts on anything unexpected. Here the cache is bytes
			// that may not mean what they claim, so an unknown type simply
			// costs the full test.
			break;
	}

	// --- the full test ----------------------------------------------------

	manifold->pointCount = 0;
	*cache = ( b3SATCache ){ 0 };

	b3AxisQuery axisQuery = b3ComputeSeparatingAxis( hullA, hullB, transformBtoA, true );

	if ( axisQuery.separatedFeature != b3_invalidAxis )
	{
		// Separated by more than the speculative distance. Remember the axis
		// and produce nothing: speculation lives entirely inside that window,
		// so there is no such thing as a contact from out here.
		cache->type = (uint8_t)axisQuery.separatedFeature;

		const b3SeparatingAxis* axis = &axisQuery.edge;
		if ( axisQuery.separatedFeature == b3_faceAxisA )
		{
			axis = &axisQuery.faceA;
		}
		else if ( axisQuery.separatedFeature == b3_faceAxisB )
		{
			axis = &axisQuery.faceB;
		}

		cache->separation = axis->separation;
		cache->indexA = (uint8_t)axis->indexA;
		cache->indexB = (uint8_t)axis->indexB;
		return;
	}

	// A bare comparison, with no bias and no tie tolerance -- upstream's, and
	// the source of the one disagreement with it that is expected rather than
	// wrong: when two faces are nearly parallel their separations can differ
	// by less than a Q12 quantum, and then rounding picks the reference face.
	// The resulting manifolds are near enough identical; what must not differ
	// is the feature ids, which b3FlipPair is what guarantees.
	if ( b3Raw( axisQuery.faceA.separation ) > b3Raw( axisQuery.faceB.separation ) )
	{
		b3BuildFaceAContact( manifold, capacity, hullA, hullB, transformBtoA, axisQuery.faceA, cache );
	}
	else
	{
		b3BuildFaceBContact( manifold, capacity, hullA, hullB, transformBtoA, axisQuery.faceB, cache );
	}

	b3SeparatingAxis edgeQuery = axisQuery.edge;

	if ( edgeQuery.indexA == B3_NULL_INDEX )
	{
		// Every edge pair was parallel or failed the Gauss test.
		return;
	}

	// The face contact can come out empty if it does not realize the axis of
	// least penetration. Build the edge contact when it does, or when the edge
	// axis is better than what the clip actually produced.
	if ( manifold->pointCount == 0 ||
		 b3Raw( edgeQuery.separation ) > b3Raw( b3AddF( cache->separation, B3_LINEAR_SLOP ) ) )
	{
		b3LocalManifold edgeManifold = { 0 };
		b3LocalManifoldPoint edgePoint = { 0 };
		edgeManifold.points = &edgePoint;

		b3SATCache edgeCache = { 0 };
		b3BuildEdgeContact( &edgeManifold, hullA, hullB, transformBtoA, edgeQuery, &edgeCache );

		// Speculation can produce vertex-vertex contacts the axis test misses,
		// leaving the edge builder with nothing; then the face contact's
		// points are the better answer and are kept.
		if ( edgeManifold.pointCount == 1 )
		{
			// Copy out, preserving the caller's point buffer -- edgeManifold's
			// points pointer aims at a local that is about to go out of scope,
			// and assigning the struct wholesale would carry it with it.
			b3LocalManifoldPoint* points = manifold->points;
			*manifold = edgeManifold;
			manifold->points = points;
			manifold->points[0] = edgePoint;
			*cache = edgeCache;
		}
	}
}
