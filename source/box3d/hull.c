// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Erin Catto        (original Box3D)
// Copyright (c) 2026 Warioware64       (Nitro Engine Advanced fixed-point port)
//
// Dirk Gregorius contributed portions of this code
//
// This file is part of Nitro Engine Advanced

/// @file   hull.c
/// @brief  Convex hull queries and the analytic box hull.
///
/// @section scope What is here, and what is deliberately not
///
/// Upstream's hull.c is 2946 lines, and about 1400 of them are the quickhull
/// builder: conflict lists, horizon walks, face merging, and the degeneracy
/// handling that all of it needs. **None of that is ported.** Hulls are built
/// on the host at full float precision and baked into the flat blob that
/// b3HullData already is -- byte offsets rather than pointers, so a baked hull
/// loads with no fixup and can sit in ROM. The device gets the query half,
/// which is this file, plus box hulls, which need no builder at all.
///
/// Absent, and where each one went:
///
///   b3CreateHull, b3CreateCylinder, b3CreateCone, b3CreateRock,
///   b3CloneHull, b3CloneAndTransformHull, b3DestroyHull   -- host baking
///   b3HashHullData, b3CompareHullData, b3HullMap*         -- dropped: Phase
///                                                            3A keeps the
///                                                            caller's hull
///                                                            pointer instead
///                                                            of a
///                                                            reference-counted
///                                                            world database
///   b3ComputeHullProjectedArea                            -- dropped with
///                                                            b3World_Explode
///                                                            and wind
///   b3CollideMoverAndHull                                 -- Phase 7 (mover)
///   b3UpdateHullBounds, b3UpdateHullBulkProperties        -- host baking; the
///                                                            device never
///                                                            constructs a
///                                                            hull, so it
///                                                            never recomputes
///                                                            these
///
/// @section soa No structure-of-arrays
///
/// Upstream keeps a second copy of the vertices and face normals split into
/// x/y/z streams, for a 4-wide support function that recovers its argmax index
/// from float mantissa bits. ARMv5TE has no SIMD, and that index trick would
/// spend 5 of Q12's 12 fraction bits. The port takes support points with a
/// scalar loop instead, which is what b3GetPointSupport in distance.c already
/// is, and every hull is ~380 bytes smaller for it.

#include "box3d/collision.h"
#include "box3d/constants.h"
#include "box3d/math_fixed.h"
#include "box3d/types.h"

#include "core.h"

#include <stddef.h>
#include <string.h>

// =========================================================================
// Support points
// =========================================================================

int b3FindHullSupportVertex( const b3HullData* hull, b3Vec3 direction )
{
	// Identical to the proxy support search, including its tie-breaking (the
	// first of equal projections wins) and its trick of shifting the first
	// point to the origin so the comparison stays meaningful for a hull far
	// from it.
	return b3GetPointSupport( b3GetHullPoints( hull ), hull->vertexCount, direction );
}

int b3FindHullSupportFace( const b3HullData* hull, b3Vec3 direction )
{
	int faceCount = hull->faceCount;
	const b3Plane* planes = b3GetHullPlanes( hull );

	B3_ASSERT( faceCount > 0 );

	// Upstream seeds the search with -FLT_MAX. There is no such value here,
	// and none is needed: face zero is a candidate like any other.
	int bestIndex = 0;
	int64_t bestDot = b3DotWide( planes[0].normal, direction );

	for ( int index = 1; index < faceCount; ++index )
	{
		int64_t dot = b3DotWide( planes[index].normal, direction );
		if ( dot > bestDot )
		{
			bestDot = dot;
			bestIndex = index;
		}
	}

	return bestIndex;
}

// =========================================================================
// Validation
// =========================================================================

/// How far a point may sit outside a face plane before the hull is called
/// non-convex.
///
/// A hull baked into Q12 is not exactly the hull that was built: every point
/// moves by up to half a quantum per component, and the face planes are fitted
/// to the moved points. So a genuinely convex hull arrives here with vertices
/// a quantum or two proud of their neighbours' planes, and a test against zero
/// would reject every hull that has ever been baked.
///
/// The slop is the right threshold rather than a tuned one: the collision code
/// already treats anything within B3_LINEAR_SLOP as touching, so a convexity
/// violation smaller than that cannot move a contact by more than the amount
/// the solver already absorbs. Anything larger is a real defect in the blob.
#define B3_HULL_CONVEX_TOL B3_LINEAR_SLOP

bool b3IsValidHull( const b3HullData* hull )
{
	// Unlike upstream this is compiled in unconditionally rather than sitting
	// behind B3_ENABLE_VALIDATION. It is not a per-frame check: a baked hull
	// is untrusted input that arrived as bytes, and a hull that is not convex
	// makes the separating axis test quietly wrong rather than loudly wrong.
	if ( hull == NULL )
	{
		return false;
	}

	if ( hull->version != B3_HULL_VERSION )
	{
		return false;
	}

	int v = hull->vertexCount;
	int e = hull->edgeCount / 2;
	int f = hull->faceCount;

	if ( v <= 0 || f <= 0 || ( hull->edgeCount & 1 ) != 0 )
	{
		return false;
	}

	if ( v > B3_MAX_HULL_VERTICES || f > B3_MAX_HULL_FACES || e > B3_MAX_HULL_EDGES )
	{
		return false;
	}

	// Euler's formula for a closed convex polyhedron.
	if ( v - e + f != 2 )
	{
		return false;
	}

	const b3HullVertex* vertices = b3GetHullVertices( hull );
	const b3HullHalfEdge* edges = b3GetHullEdges( hull );
	const b3HullFace* faces = b3GetHullFaces( hull );
	const b3Plane* planes = b3GetHullPlanes( hull );
	const b3Vec3* points = b3GetHullPoints( hull );

	if ( vertices == NULL || edges == NULL || faces == NULL || planes == NULL || points == NULL )
	{
		return false;
	}

	// Every vertex names a half-edge leaving it.
	for ( int index = 0; index < v; ++index )
	{
		if ( vertices[index].edge >= hull->edgeCount )
		{
			return false;
		}

		if ( edges[vertices[index].edge].origin != index )
		{
			return false;
		}
	}

	// Half-edges are stored in twin pairs.
	for ( int index = 0; index < hull->edgeCount; index += 2 )
	{
		if ( edges[index + 0].twin != index + 1 || edges[index + 1].twin != index + 0 )
		{
			return false;
		}
	}

	for ( int faceIndex = 0; faceIndex < f; ++faceIndex )
	{
		int baseEdgeIndex = faces[faceIndex].edge;
		if ( baseEdgeIndex >= hull->edgeCount )
		{
			return false;
		}

		// The centroid must be behind every face plane.
		if ( b3Raw( b3PlaneSeparation( planes[faceIndex], hull->center ) ) >= 0 )
		{
			return false;
		}

		int edgeIndex = baseEdgeIndex;
		int guard = 0;
		do
		{
			const b3HullHalfEdge* edge = edges + edgeIndex;
			if ( edge->next >= hull->edgeCount || edge->twin >= hull->edgeCount || edge->origin >= v )
			{
				return false;
			}

			const b3HullHalfEdge* next = edges + edge->next;
			const b3HullHalfEdge* twin = edges + edge->twin;

			if ( edge->face != faceIndex )
			{
				return false;
			}

			if ( twin->twin != edgeIndex )
			{
				return false;
			}

			// Walking forward around the face must reach the same vertex as
			// stepping across the twin.
			if ( next->origin != twin->origin )
			{
				return false;
			}

			edgeIndex = edge->next;

			// Upstream's loop trusts the structure to close. This one is
			// reading a blob that may not, and an unterminated do-while on
			// device is a hang rather than a failed check.
			if ( ++guard > hull->edgeCount )
			{
				return false;
			}
		}
		while ( edgeIndex != baseEdgeIndex );
	}

	// Convexity. Not an upstream check -- upstream's hulls come from its own
	// builder and are convex by construction. These arrive quantized, and
	// quantization is exactly what can break it.
	for ( int faceIndex = 0; faceIndex < f; ++faceIndex )
	{
		b3Plane plane = planes[faceIndex];

		for ( int pointIndex = 0; pointIndex < v; ++pointIndex )
		{
			if ( b3Raw( b3PlaneSeparation( plane, points[pointIndex] ) ) > b3Raw( B3_HULL_CONVEX_TOL ) )
			{
				return false;
			}
		}
	}

	if ( b3Raw( hull->volume ) <= 0 || b3Raw( hull->surfaceArea ) <= 0 || b3Raw( hull->innerRadius ) <= 0 )
	{
		return false;
	}

	return true;
}

// =========================================================================
// Mass and bounds
// =========================================================================

b3MassData b3ComputeHullMass( const b3HullData* shape, b3f density )
{
	b3MassData out;
	out.mass = b3MulFF( density, shape->volume );
	out.center = shape->center;

	// The stored tensor is already per unit mass, so unlike upstream there is
	// nothing to scale here: density cancels. That is the whole reason for the
	// convention -- the absolute tensor grows as length⁵ and would overflow
	// Q12 for any hull worth colliding.
	out.inertia = shape->centralInertia;
	return out;
}

b3ShapeExtent b3ComputeHullExtent( const b3HullData* hull, b3Vec3 origin )
{
	const b3Vec3* points = b3GetHullPoints( hull );

	b3ShapeExtent extent;
	extent.minExtent = hull->innerRadius;
	extent.maxExtent = b3Vec3_zero;
	for ( int index = 0; index < hull->vertexCount; ++index )
	{
		b3Vec3 point = points[index];
		extent.maxExtent = b3Max( extent.maxExtent, b3Abs( b3Sub( point, origin ) ) );
	}

	return extent;
}

b3AABB b3ComputeHullAABB( const b3HullData* shape, b3Transform transform )
{
	return b3AABB_Transform( transform, shape->aabb );
}

b3AABB b3ComputeSweptHullAABB( const b3HullData* shape, b3Transform xf1, b3Transform xf2 )
{
	b3AABB aabb1 = b3AABB_Transform( xf1, shape->aabb );
	b3AABB aabb2 = b3AABB_Transform( xf2, shape->aabb );
	return b3AABB_Union( aabb1, aabb2 );
}

// =========================================================================
// Queries
// =========================================================================

bool b3OverlapHull( const b3HullData* shape, b3Transform shapeTransform, const b3ShapeProxy* proxy )
{
	b3DistanceInput input;
	input.proxyA = ( b3ShapeProxy ){ b3GetHullPoints( shape ), shape->vertexCount, b3f_zero };
	input.proxyB = *proxy;
	input.transform = b3InvMulTransforms( shapeTransform, b3Transform_identity );
	input.useRadii = true;

	b3SimplexCache cache = { 0 };
	b3DistanceOutput output = b3ShapeDistance( &input, &cache, NULL, 0 );
	return b3Raw( output.distance ) < b3Raw( B3_OVERLAP_SLOP );
}

/// Where a face plane crossing falls relative to the useful part of the ray.
typedef enum b3FaceCrossing
{
	b3_crossingBelow = -1, ///< Fraction below -maxFraction.
	b3_crossingInside = 0, ///< Fraction within [-maxFraction, maxFraction].
	b3_crossingAbove = 1,  ///< Fraction above maxFraction.
} b3FaceCrossing;

/// Clamped plane crossing, the same guard aabb.c uses on the ray slabs.
///
/// Upstream divides unconditionally and lets float hand back an enormous
/// quotient for a face the ray is nearly parallel to, which the later interval
/// clipping discards. Fixed point cannot: the hardware divider returns 32 bits
/// and an oversized quotient is undefined, not merely large. An epsilon on the
/// denominator would not do either, since whether the quotient fits depends on
/// the numerator equally.
///
/// So the test is made against what actually matters -- the interval never
/// leaves [0, maxFraction], so any |fraction| beyond it is indistinguishable
/// from the plane being parallel, and *which side* it is beyond decides the
/// outcome without the division ever being attempted.
static b3FaceCrossing b3FaceCross( b3f num, b3f den, b3c maxFraction, b3c* fraction )
{
	int64_t n = b3Raw( num );
	int64_t d = b3Raw( den );

	B3_ASSERT( d != 0 );

	int64_t absN = n < 0 ? -n : n;
	int64_t absD = d < 0 ? -d : d;

	// |num / den| > maxFraction  <=>  |num| * 2^30 > |den| * maxFractionRaw.
	if ( ( absN << B3_C_SHIFT ) > absD * (int64_t)b3Raw( maxFraction ) )
	{
		return ( n < 0 ) == ( d < 0 ) ? b3_crossingAbove : b3_crossingBelow;
	}

	// The quotient is now known to be within [-maxFraction, maxFraction], so
	// it lands in Q30 and the divider is safe.
	*fraction = b3DivFFToC( num, den );
	return b3_crossingInside;
}

b3CastOutput b3RayCastHull( const b3HullData* shape, const b3RayCastInput* input )
{
	b3CastOutput output = { 0 };

	b3c lower = b3c_zero;
	b3c upper = input->maxFraction;
	int bestFace = B3_NULL_INDEX;

	const b3Plane* planes = b3GetHullPlanes( shape );

	for ( int faceIndex = 0; faceIndex < shape->faceCount; ++faceIndex )
	{
		b3Plane plane = planes[faceIndex];

		b3f distance = b3SubF( plane.offset, b3Dot( plane.normal, input->origin ) );
		b3f denominator = b3Dot( plane.normal, input->translation );

		if ( b3Raw( denominator ) == 0 )
		{
			// Parallel to this face. The ray is either inside its half-space
			// for its whole length or outside it for the whole length.
			if ( b3Raw( distance ) < 0 )
			{
				return output;
			}

			continue;
		}

		b3c fraction = b3c_zero;
		b3FaceCrossing crossing = b3FaceCross( distance, denominator, input->maxFraction, &fraction );

		if ( b3Raw( denominator ) < 0 )
		{
			// Entering the half-space: this raises the near end.
			if ( crossing == b3_crossingAbove )
			{
				// Entry beyond maxFraction, so the interval is empty.
				return output;
			}

			if ( crossing == b3_crossingInside && b3Raw( fraction ) > b3Raw( lower ) )
			{
				bestFace = faceIndex;
				lower = fraction;
			}
		}
		else
		{
			// Leaving the half-space: this lowers the far end.
			if ( crossing == b3_crossingBelow )
			{
				// Exit before the ray starts, so the interval is empty.
				return output;
			}

			if ( crossing == b3_crossingInside && b3Raw( fraction ) < b3Raw( upper ) )
			{
				upper = fraction;
			}
		}

		if ( b3Raw( upper ) < b3Raw( lower ) )
		{
			return output;
		}
	}

	if ( bestFace != B3_NULL_INDEX )
	{
		output.point = b3MulAdd( input->origin, b3CToF( lower ), input->translation );
		output.normal = planes[bestFace].normal;
		output.fraction = lower;
		output.hit = true;
	}
	else
	{
		// The ray never crossed into the hull from outside, which means it
		// started inside it. Upstream reports a hit at the origin with no
		// normal and no fraction; keep that, since callers use `hit` plus a
		// zero fraction to detect exactly this case.
		output.point = input->origin;
		output.hit = true;
	}

	return output;
}

b3CastOutput b3ShapeCastHull( const b3HullData* shape, const b3ShapeCastInput* input )
{
	b3ShapeCastPairInput pairInput;
	pairInput.proxyA = ( b3ShapeProxy ){ b3GetHullPoints( shape ), shape->vertexCount, b3f_zero };
	pairInput.proxyB = input->proxy;
	pairInput.transform = b3Transform_identity;
	pairInput.translationB = input->translation;
	pairInput.maxFraction = input->maxFraction;
	pairInput.canEncroach = input->canEncroach;

	return b3ShapeCast( &pairInput );
}

// =========================================================================
// Box hulls
// =========================================================================
//
// The one hull the device builds for itself. A box's topology is fixed, so
// the half-edge structure is a constant and only the points, planes and bulk
// properties depend on the arguments -- no builder, no allocation, and the
// result is a value the caller owns.
//
// The topology table below is upstream's, transcribed unchanged: vertex 0 is
// (+h, +h, +h) and the winding is what makes face planes come out pointing
// away from the centre.

/// Constant template: vertex, edge and face topology, plus the offsets that
/// make the embedded header address the arrays that follow it.
static const b3BoxHull s_boxHull = {
	.base =
		{
			.version = B3_HULL_VERSION,
			.byteCount = sizeof( b3BoxHull ),
			.hash = 0,
			.vertexCount = 8,
			.edgeCount = 24,
			.faceCount = 6,
			.vertexOffset = offsetof( b3BoxHull, boxVertices ),
			.pointOffset = offsetof( b3BoxHull, boxPoints ),
			.edgeOffset = offsetof( b3BoxHull, boxEdges ),
			.planeOffset = offsetof( b3BoxHull, boxPlanes ),
			.faceOffset = offsetof( b3BoxHull, boxFaces ),
		},
	.boxVertices =
		{
			[0] = { .edge = 8 },
			[1] = { .edge = 1 },
			[2] = { .edge = 0 },
			[3] = { .edge = 9 },
			[4] = { .edge = 13 },
			[5] = { .edge = 3 },
			[6] = { .edge = 5 },
			[7] = { .edge = 11 },
		},
	.boxEdges =
		{
			[0] = { 2, 1, 2, 0 },	 [1] = { 17, 0, 1, 5 },	  [2] = { 4, 3, 1, 0 },	   [3] = { 20, 2, 5, 3 },
			[4] = { 6, 5, 5, 0 },	 [5] = { 23, 4, 6, 4 },	  [6] = { 0, 7, 6, 0 },	   [7] = { 18, 6, 2, 2 },
			[8] = { 10, 9, 0, 1 },	 [9] = { 21, 8, 3, 5 },	  [10] = { 12, 11, 3, 1 }, [11] = { 16, 10, 7, 2 },
			[12] = { 14, 13, 7, 1 }, [13] = { 19, 12, 4, 4 }, [14] = { 8, 15, 4, 1 },  [15] = { 22, 14, 0, 3 },
			[16] = { 7, 17, 3, 2 },	 [17] = { 9, 16, 2, 5 },  [18] = { 11, 19, 6, 2 }, [19] = { 5, 18, 7, 4 },
			[20] = { 15, 21, 1, 3 }, [21] = { 1, 20, 0, 5 },  [22] = { 3, 23, 4, 3 },  [23] = { 13, 22, 5, 4 },
		},
	.boxFaces =
		{
			[0] = { .edge = 0 },
			[1] = { .edge = 8 },
			[2] = { .edge = 16 },
			[3] = { .edge = 20 },
			[4] = { .edge = 19 },
			[5] = { .edge = 21 },
		},
};

b3BoxHull b3MakeTransformedBoxHull( b3f hx, b3f hy, b3f hz, b3Transform transform )
{
	B3_ASSERT( b3IsValidTransform( transform ) );

	b3BoxHull boxHull = s_boxHull;

	// Upstream's floor of 0.2 * B3_LINEAR_SLOP, which is 4 raw here. A box
	// thinner than this has an inner radius that rounds to nothing and cannot
	// carry a plane through its centre.
	b3f minH = b3MulFC( B3_LINEAR_SLOP, b3cFromFrac( 1, 5 ) );
	b3Vec3 h = b3Max( b3MakeVec3( minH, minH, minH ), b3MakeVec3( hx, hy, hz ) );

	boxHull.base.aabb = b3AABB_Transform( transform, b3MakeAABB( b3Neg( h ), h ) );

	b3f eight = b3fFromInt( 8 );
	boxHull.base.surfaceArea =
		b3MulFF( eight, b3AddF( b3AddF( b3MulFF( h.x, h.y ), b3MulFF( h.x, h.z ) ), b3MulFF( h.y, h.z ) ) );

	// Volume reaches the Q12 ceiling at a half-extent of about 40 units, which
	// is the practical size limit on a hull and well outside the documented
	// world scale. Each intermediate here stays below it too.
	boxHull.base.volume = b3MulFF( eight, b3MulFF( b3MulFF( h.x, h.y ), h.z ) );
	boxHull.base.innerRadius = b3MinF( h.x, b3MinF( h.y, h.z ) );
	boxHull.base.center = transform.p;

	// b3BoxUnitInertia is already per unit mass, which is the convention
	// b3HullData stores, so upstream's volume factor simply drops out.
	boxHull.base.centralInertia = b3RotateInertia( transform.q, b3BoxUnitInertia( b3Neg( h ), h ) );

	// The content hash stays zero. Upstream hashes the whole struct here so
	// its hull database can de-duplicate; that database is Phase 3, the baker
	// is what fills the field for a baked hull, and hashing 440 bytes on every
	// box creation would be paid for nothing.
	boxHull.base.hash = 0;

	b3Vec3 lower = b3Neg( h );
	b3Vec3 upper = h;

	boxHull.boxPlanes[0] = b3TransformPlane( transform, b3MakePlaneFromNormalAndPoint( b3Neg( b3Vec3_axisX ), lower ) );
	boxHull.boxPlanes[1] = b3TransformPlane( transform, b3MakePlaneFromNormalAndPoint( b3Vec3_axisX, upper ) );
	boxHull.boxPlanes[2] = b3TransformPlane( transform, b3MakePlaneFromNormalAndPoint( b3Neg( b3Vec3_axisY ), lower ) );
	boxHull.boxPlanes[3] = b3TransformPlane( transform, b3MakePlaneFromNormalAndPoint( b3Vec3_axisY, upper ) );
	boxHull.boxPlanes[4] = b3TransformPlane( transform, b3MakePlaneFromNormalAndPoint( b3Neg( b3Vec3_axisZ ), lower ) );
	boxHull.boxPlanes[5] = b3TransformPlane( transform, b3MakePlaneFromNormalAndPoint( b3Vec3_axisZ, upper ) );

	boxHull.boxPoints[0] = b3TransformPoint( transform, b3MakeVec3( h.x, h.y, h.z ) );
	boxHull.boxPoints[1] = b3TransformPoint( transform, b3MakeVec3( b3NegF( h.x ), h.y, h.z ) );
	boxHull.boxPoints[2] = b3TransformPoint( transform, b3MakeVec3( b3NegF( h.x ), b3NegF( h.y ), h.z ) );
	boxHull.boxPoints[3] = b3TransformPoint( transform, b3MakeVec3( h.x, b3NegF( h.y ), h.z ) );
	boxHull.boxPoints[4] = b3TransformPoint( transform, b3MakeVec3( h.x, h.y, b3NegF( h.z ) ) );
	boxHull.boxPoints[5] = b3TransformPoint( transform, b3MakeVec3( b3NegF( h.x ), h.y, b3NegF( h.z ) ) );
	boxHull.boxPoints[6] = b3TransformPoint( transform, b3MakeVec3( b3NegF( h.x ), b3NegF( h.y ), b3NegF( h.z ) ) );
	boxHull.boxPoints[7] = b3TransformPoint( transform, b3MakeVec3( h.x, b3NegF( h.y ), b3NegF( h.z ) ) );

	return boxHull;
}

b3BoxHull b3MakeBoxHull( b3f hx, b3f hy, b3f hz )
{
	return b3MakeTransformedBoxHull( hx, hy, hz, b3Transform_identity );
}

b3BoxHull b3MakeCubeHull( b3f halfWidth )
{
	return b3MakeBoxHull( halfWidth, halfWidth, halfWidth );
}

b3BoxHull b3MakeOffsetBoxHull( b3f hx, b3f hy, b3f hz, b3Vec3 offset )
{
	b3Transform transform = { offset, b3Quat_identity };
	return b3MakeTransformedBoxHull( hx, hy, hz, transform );
}

// =========================================================================
// Prism hulls
// =========================================================================
//
// The second hull the device builds for itself. A right prism with a regular
// polygon cross-section has topology that is a function of the side count
// alone, so like the box it needs no builder -- but unlike the box its
// vertices come out of a sine table and are therefore quantized, so the face
// planes are fitted to the quantized points rather than derived analytically.
// That is the same decision the host baker makes, and for the same reason: a
// point rounded one way and its plane rounded another leaves the vertex in
// front of the plane it is supposed to lie on, and every separation measured
// against that face is then wrong by the discrepancy.
//
// Topology, with top(i) = i and bot(i) = sides + i, ip = (i + 1) % sides:
//
//   face i        the quad top(i) -> top(ip) -> bot(ip) -> bot(i)
//   face sides    the top cap, wound in *decreasing* angle so its normal is +Y
//   face sides+1  the bottom cap, wound in increasing angle, normal -Y
//
//   half-edge 6i+0  top(i)  -> top(ip)   face i
//   half-edge 6i+1  top(ip) -> top(i)    face sides       (twin of 6i+0)
//   half-edge 6i+2  bot(ip) -> bot(i)    face i
//   half-edge 6i+3  bot(i)  -> bot(ip)   face sides+1     (twin of 6i+2)
//   half-edge 6i+4  bot(i)  -> top(i)    face i
//   half-edge 6i+5  top(i)  -> bot(i)    face (i-1+sides) % sides   (twin of 6i+4)

b3PrismHull b3MakePrismHull( b3f radius, b3f halfHeight, int sides )
{
	b3PrismHull hull;
	memset( &hull, 0, sizeof( hull ) );

	if ( sides < 3 || sides > B3_MAX_PRISM_SIDES )
	{
		return hull;
	}

	// The same floor b3MakeTransformedBoxHull applies: thinner than this and
	// the inner radius rounds to nothing, so no plane passes through the
	// centre and b3IsValidHull would reject the result anyway.
	b3f minH = b3MulFC( B3_LINEAR_SLOP, b3cFromFrac( 1, 5 ) );
	b3f r = b3Raw( radius ) > b3Raw( minH ) ? radius : minH;
	b3f h = b3Raw( halfHeight ) > b3Raw( minH ) ? halfHeight : minH;

	int vertexCount = 2 * sides;
	int edgeCount = 6 * sides;
	int faceCount = sides + 2;

	hull.base.version = B3_HULL_VERSION;
	hull.base.byteCount = (int)sizeof( b3PrismHull );
	hull.base.hash = 0;
	hull.base.vertexCount = vertexCount;
	hull.base.edgeCount = edgeCount;
	hull.base.faceCount = faceCount;
	hull.base.vertexOffset = (int)offsetof( b3PrismHull, prismVertices );
	hull.base.pointOffset = (int)offsetof( b3PrismHull, prismPoints );
	hull.base.edgeOffset = (int)offsetof( b3PrismHull, prismEdges );
	hull.base.planeOffset = (int)offsetof( b3PrismHull, prismPlanes );
	hull.base.faceOffset = (int)offsetof( b3PrismHull, prismFaces );

	// Points. The angle is formed as i * 32768 / sides rather than by
	// accumulating a step, so the rounding does not walk around the ring.
	//
	// Through b3HwDiv64, not the `/` operator: `sides` is a runtime value, and
	// a plain integer division by one links __aeabi_idiv -- which works, but
	// this build is checked with `nm -u` precisely to keep libgcc helpers out
	// of the ROM. b3HwDiv64 is the DS divider on device and a plain divide on
	// the host, which is why it exists.
	for ( int i = 0; i < sides; ++i )
	{
		b3a angle = (b3a)b3HwDiv64( (int64_t)i * 32768, sides );
		b3f x = b3MulFC( r, b3CosA( angle ) );
		b3f z = b3MulFC( r, b3SinA( angle ) );

		hull.prismPoints[i] = b3MakeVec3( x, h, z );
		hull.prismPoints[sides + i] = b3MakeVec3( x, b3NegF( h ), z );
	}

	// Topology.
	for ( int i = 0; i < sides; ++i )
	{
		// Wrapping by comparison rather than by `%`, for the same reason the
		// angle avoids `/`: a modulo by a runtime value is a libgcc call.
		int ip = i + 1 == sides ? 0 : i + 1;
		int im = i == 0 ? sides - 1 : i - 1;

		hull.prismVertices[i].edge = (uint8_t)( 6 * i + 0 );
		hull.prismVertices[sides + i].edge = (uint8_t)( 6 * i + 4 );

		// top(i) -> top(ip), on side face i. Next is the vertical half-edge
		// leaving top(ip), which belongs to face (ip-1) % sides == i.
		hull.prismEdges[6 * i + 0] = ( b3HullHalfEdge ){ (uint8_t)( 6 * ip + 5 ), (uint8_t)( 6 * i + 1 ), (uint8_t)i,
														(uint8_t)i };

		// top(ip) -> top(i), on the top cap, which winds backwards.
		hull.prismEdges[6 * i + 1] = ( b3HullHalfEdge ){ (uint8_t)( 6 * im + 1 ), (uint8_t)( 6 * i + 0 ),
														(uint8_t)ip, (uint8_t)sides };

		// bot(ip) -> bot(i), on side face i.
		hull.prismEdges[6 * i + 2] = ( b3HullHalfEdge ){ (uint8_t)( 6 * i + 4 ), (uint8_t)( 6 * i + 3 ),
														(uint8_t)( sides + ip ), (uint8_t)i };

		// bot(i) -> bot(ip), on the bottom cap.
		hull.prismEdges[6 * i + 3] = ( b3HullHalfEdge ){ (uint8_t)( 6 * ip + 3 ), (uint8_t)( 6 * i + 2 ),
														(uint8_t)( sides + i ), (uint8_t)( sides + 1 ) };

		// bot(i) -> top(i), closing side face i.
		hull.prismEdges[6 * i + 4] = ( b3HullHalfEdge ){ (uint8_t)( 6 * i + 0 ), (uint8_t)( 6 * i + 5 ),
														(uint8_t)( sides + i ), (uint8_t)i };

		// top(i) -> bot(i), on side face i-1.
		hull.prismEdges[6 * i + 5] = ( b3HullHalfEdge ){ (uint8_t)( 6 * im + 2 ), (uint8_t)( 6 * i + 4 ), (uint8_t)i,
														(uint8_t)im };

		hull.prismFaces[i].edge = (uint8_t)( 6 * i + 0 );
	}

	hull.prismFaces[sides].edge = (uint8_t)( 6 * ( sides - 1 ) + 1 );
	hull.prismFaces[sides + 1].edge = 3;

	// Side planes, fitted to the quantized points: the normal from the face's
	// own edges, the offset averaged over its four vertices so the residual is
	// spread rather than pinned to whichever vertex was picked.
	for ( int i = 0; i < sides; ++i )
	{
		int ip = i + 1 == sides ? 0 : i + 1;
		b3Vec3 t0 = hull.prismPoints[i];
		b3Vec3 t1 = hull.prismPoints[ip];
		b3Vec3 b0 = hull.prismPoints[sides + i];
		b3Vec3 b1 = hull.prismPoints[sides + ip];

		// b3CrossDirection rather than b3Cross: the two edges span a face that
		// may be narrow for a many-sided prism, and a narrow cross narrows to
		// nothing in Q12.
		b3Vec3 normal = b3Normalize( b3CrossDirection( b3Sub( t1, t0 ), b3Sub( b1, t1 ) ) );

		int64_t sum = b3DotWide( normal, t0 ) + b3DotWide( normal, t1 ) + b3DotWide( normal, b0 ) +
					  b3DotWide( normal, b1 );

		hull.prismPlanes[i].normal = normal;
		hull.prismPlanes[i].offset = b3Makeb3fRef( (int32_t)B3_SHIFT_ROUND( sum / 4, B3_F_SHIFT ),
												   B3_REF( (double)( sum / 4 ) / 16777216.0 ) );
	}

	// The caps are exact: every top point has y = +h and every bottom -h.
	hull.prismPlanes[sides] = b3MakePlaneFromNormalAndPoint( b3Vec3_axisY, b3MakeVec3( b3f_zero, h, b3f_zero ) );
	hull.prismPlanes[sides + 1] =
		b3MakePlaneFromNormalAndPoint( b3Neg( b3Vec3_axisY ), b3MakeVec3( b3f_zero, b3NegF( h ), b3f_zero ) );

	// Bulk properties, in closed form for the ideal polygon. The quantized
	// points differ from it by half a raw unit, which is far below what a mass
	// property is used for.
	//
	//   A   = (s/2) r^2 sin(2*pi/s)
	//   J/A = (r^2/6) (1 + 2 cos^2(pi/s))     polar second moment per area
	//
	// so the inertia per unit mass, which is what b3HullData stores, is
	// J/A about the prism axis and J/(2A) + h^2/3 about the other two.
	b3a halfAngle = (b3a)b3HwDiv64( 16384, sides );
	b3c cosHalf = b3CosA( halfAngle );
	b3c sinHalf = b3SinA( halfAngle );
	b3c sinFull = b3SinA( (b3a)b3HwDiv64( 32768, sides ) );

	b3f r2 = b3MulFF( r, r );
	b3f area = b3MulFF( b3MulFC( r2, sinFull ), b3fFromFrac( sides, 2 ) );
	b3f perimeter = b3MulFF( b3MulFC( r, sinHalf ), b3fFromInt( 2 * sides ) );

	hull.base.volume = b3MulFF( area, b3MulFF( b3fFromInt( 2 ), h ) );
	hull.base.surfaceArea =
		b3AddF( b3MulFF( b3fFromInt( 2 ), area ), b3MulFF( perimeter, b3MulFF( b3fFromInt( 2 ), h ) ) );

	// The largest sphere that fits: the apothem across, the half height along.
	b3f apothem = b3MulFC( r, cosHalf );
	hull.base.innerRadius = b3MinF( apothem, h );

	hull.base.center = b3Vec3_zero;

	// shape = r^2 (1 + 2 cos^2(pi/s)). The bracket runs up to 3, which does not
	// fit a Q30 coefficient -- b3c tops out just under 2 -- so the doubling is
	// done after the multiply, in Q12, where it is a length squared and has
	// room.
	b3f shape = b3AddF( r2, b3MulFF( b3fFromInt( 2 ), b3MulFC( r2, b3MulCC( cosHalf, cosHalf ) ) ) );
	b3f iyy = b3MulFC( shape, b3cFromFrac( 1, 6 ) );
	b3f ixx = b3AddF( b3MulFC( shape, b3cFromFrac( 1, 12 ) ), b3MulFC( b3MulFF( h, h ), b3cFromFrac( 1, 3 ) ) );
	hull.base.centralInertia = b3MakeDiagonalMatrix( ixx, iyy, ixx );

	// The AABB is scanned from the quantized points, not from r, so it really
	// does contain them.
	b3AABB aabb = b3MakeAABB( hull.prismPoints[0], hull.prismPoints[0] );
	for ( int i = 1; i < vertexCount; ++i )
	{
		aabb = b3AABB_AddPoint( aabb, hull.prismPoints[i] );
	}
	hull.base.aabb = aabb;

	return hull;
}
