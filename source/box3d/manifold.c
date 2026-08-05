// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Erin Catto        (original Box3D)
// Copyright (c) 2026 Warioware64       (Nitro Engine Advanced fixed-point port)
//
// Dirk Gregorius contributed portions of this code
//
// This file is part of Nitro Engine Advanced

/// @file   manifold.c
/// @brief  Clipping, incident face selection, and the shared manifold helpers.
///
/// The helpers every manifold generator shares, kept apart from
/// convex_manifold.c the way upstream keeps them. That was written when the
/// hull-versus-hull path was the only caller and the triangle path was a
/// promise; the triangle path is now the second caller, and six helpers moved
/// here from convex_manifold.c to meet it -- see "Shared helpers" below.
///
/// @section validation What this file refuses that upstream asserts
///
/// Both functions here read topology that arrived as bytes. Upstream's hulls
/// come from its own builder and its own asserts, so a malformed one is a
/// programming error; a baked blob is untrusted input, and on a handheld a
/// do-while that never closes is a hang rather than a failed check. So the
/// face walk carries a guard counter, and b3ClipPolygon reports an output
/// that would not fit rather than writing past the buffer -- upstream has no
/// output bound at all.

#include "box3d/collision.h"
#include "box3d/constants.h"
#include "box3d/math_fixed.h"
#include "box3d/types.h"

#include "core.h"
#include "manifold.h"

int b3FindIncidentFace( const b3HullData* hull, b3Vec3 refNormal, int vertexIndex )
{
	B3_ASSERT( 0 <= vertexIndex && vertexIndex < hull->vertexCount );

	const b3HullVertex* vertices = b3GetHullVertices( hull );
	const b3HullHalfEdge* edges = b3GetHullEdges( hull );
	const b3Plane* planes = b3GetHullPlanes( hull );
	const b3Vec3* points = b3GetHullPoints( hull );

	const b3HullVertex* vertex = vertices + vertexIndex;
	int baseEdgeIndex = vertex->edge;

	b3Vec3 edgeOrigin = points[vertexIndex];

	int minEdgeIndex = B3_NULL_INDEX;
	int64_t minProjection = 0;

	int edgeIndex = baseEdgeIndex;
	int guard = 0;
	do
	{
		const b3HullHalfEdge* edge = edges + edgeIndex;
		const b3HullHalfEdge* twin = edges + edge->twin;
		B3_ASSERT( edge->origin == vertexIndex );

		b3Vec3 offset = b3Sub( points[twin->origin], edgeOrigin );

		// Upstream normalizes before taking the projection, and that cannot be
		// skipped: the comparison is *between different edges*, so their
		// lengths do not cancel the way they would in a single ratio. The
		// tempting alternative -- cross-multiplying dot_i^2 * |e_j|^2 against
		// dot_j^2 * |e_i|^2 to avoid the square roots -- is a fourth power of
		// length, and it overflows int64 for edges much past a unit.
		int64_t lengthSq = b3LengthSquaredWide( offset );
		if ( lengthSq != 0 )
		{
			b3Vec3 axis = b3MulWV( b3RsqrtWide( lengthSq ), offset );

			// Kept at Q24. The winning margin between two edges that are both
			// nearly perpendicular to the reference normal is small, and
			// narrowing to Q12 first would let rounding decide it.
			int64_t projection = b3DotWide( axis, refNormal );
			if ( projection < 0 )
			{
				projection = -projection;
			}

			if ( minEdgeIndex == B3_NULL_INDEX || projection < minProjection )
			{
				minEdgeIndex = edgeIndex;
				minProjection = projection;
			}
		}

		edgeIndex = twin->next;

		if ( ++guard > hull->edgeCount )
		{
			break;
		}
	}
	while ( edgeIndex != baseEdgeIndex );

	if ( minEdgeIndex == B3_NULL_INDEX )
	{
		// Every edge leaving this vertex quantized to zero length, which a
		// validated hull cannot produce. Answering with the vertex's own face
		// keeps the caller on a real face rather than on index -1.
		return edges[baseEdgeIndex].face;
	}

	const b3HullHalfEdge* minEdge = edges + minEdgeIndex;
	int faceIndex1 = minEdge->face;
	int faceIndex2 = edges[minEdge->twin].face;

	// Wide again, and note the tie goes to face 2, exactly as upstream's `<`
	// does.
	return b3DotWide( planes[faceIndex1].normal, refNormal ) < b3DotWide( planes[faceIndex2].normal, refNormal )
			   ? faceIndex1
			   : faceIndex2;
}

#if B3_ENABLE_VALIDATION
bool b3ValidatePolygon( const b3ClipVertex* polygon, int count )
{
	// An empty polygon is valid: rebuilding a manifold from the cache can clip
	// every point away.
	if ( count == 0 )
	{
		return true;
	}

	b3ClipVertex vertex1 = polygon[count - 1];
	for ( int index = 0; index < count; ++index )
	{
		b3ClipVertex vertex2 = polygon[index];

		if ( vertex1.pair.owner2 != vertex2.pair.owner1 || vertex1.pair.index2 != vertex2.pair.index1 )
		{
			return false;
		}

		vertex1 = vertex2;
	}

	return true;
}
#endif

int b3ClipPolygon( b3ClipVertex* out, const b3ClipVertex* polygon, int count, b3Plane clipPlane, int edge,
				   b3Plane refPlane )
{
	B3_ASSERT( count >= 3 );

	b3ClipVertex vertex1 = polygon[count - 1];
	b3f distance1 = b3PlaneSeparation( clipPlane, vertex1.position );
	int outCount = 0;

	for ( int index = 0; index < count; ++index )
	{
		b3ClipVertex vertex2 = polygon[index];
		b3f distance2 = b3PlaneSeparation( clipPlane, vertex2.position );

		// The clip plane's normal need not be unit: only the signs of these
		// distances and the ratio below are read, and both are invariant under
		// scaling the plane. Separations that leave here are measured against
		// refPlane, which is a hull face plane and is normalized.
		bool behind1 = b3Raw( distance1 ) <= 0;
		bool behind2 = b3Raw( distance2 ) <= 0;

		if ( behind1 && behind2 )
		{
			if ( outCount >= B3_MAX_CLIP_POINTS )
			{
				return B3_NULL_INDEX;
			}

			out[outCount] = vertex2;
			outCount += 1;
		}
		else if ( behind1 || behind2 )
		{
			// The signs strictly differ, so |d1 - d2| = |d1| + |d2| > |d1| and
			// the quotient lands in [0, 1]. b3DivFFToC exists for exactly this
			// shape -- a ratio whose denominator provably dominates -- and the
			// same proof is written out in b3ClipSegment.
			b3c fraction = b3DivFFToC( distance1, b3SubF( distance1, distance2 ) );

			// Interpolate as a weighted sum of the endpoints rather than as an
			// endpoint plus a scaled difference. Upstream's b3MulAdd form is
			// the same algebra, but this one keeps every intermediate inside
			// the segment's own coordinate range, which matters when the two
			// endpoints are close together and far from the origin.
			b3Vec3 position =
				b3Add( b3MulCV( b3SubC( b3c_one, fraction ), vertex1.position ), b3MulCV( fraction, vertex2.position ) );

			b3ClipVertex vertex;
			vertex.position = position;
			vertex.separation = b3PlaneSeparation( refPlane, position );

			if ( behind1 )
			{
				// Leaving the region: the outgoing edge becomes this clip edge.
				vertex.pair = vertex2.pair;
				vertex.pair.owner2 = (uint8_t)b3_featureShapeA;
				vertex.pair.index2 = (uint8_t)edge;

				if ( outCount >= B3_MAX_CLIP_POINTS )
				{
					return B3_NULL_INDEX;
				}

				out[outCount] = vertex;
				outCount += 1;
			}
			else
			{
				// Entering the region: the incoming edge becomes this clip
				// edge, and vertex2 survives behind it.
				vertex.pair = vertex1.pair;
				vertex.pair.owner1 = (uint8_t)b3_featureShapeA;
				vertex.pair.index1 = (uint8_t)edge;

				if ( outCount + 1 >= B3_MAX_CLIP_POINTS )
				{
					return B3_NULL_INDEX;
				}

				out[outCount] = vertex;
				outCount += 1;
				out[outCount] = vertex2;
				outCount += 1;
			}
		}

		vertex1 = vertex2;
		distance1 = distance2;
	}

	B3_ASSERT( b3ValidatePolygon( out, outCount ) );

	return outCount;
}

// =========================================================================
// Shared helpers
// =========================================================================
//
// These six were static in convex_manifold.c until the triangle path needed
// them. Nothing about them changed in the move except the `static`: they are
// the fixed-point decisions the hull manifolds already rest on -- the
// degenerate-direction answer, the one place a wide separation is narrowed,
// the wide signed area, the reducer and its bias -- and a triangle collider
// that re-derived any of them would be re-deriving something already tested
// by run_pair's hull cases.

/// Unit normal from an offset whose squared length is already known.
///
/// Upstream writes `if (distance*distance > 1000*FLT_MIN) normal = offset/distance`,
/// which asks whether the offset is long enough to normalize without dividing
/// by a denormal. Here the offset is degenerate exactly when its squared
/// length quantizes to zero -- there is no denormal range -- and the fallback
/// is upstream's arbitrary +Y.
b3Vec3 b3NormalOrUp( b3Vec3 offset, int64_t distanceSq )
{
	if ( distanceSq == 0 )
	{
		return b3Vec3_axisYFn();
	}

	return b3MulWV( b3RsqrtWide( distanceSq ), offset );
}

/// Contact point midway between the two surfaces.
b3Vec3 b3MidpointContact( b3Vec3 pA, b3f rA, b3Vec3 pB, b3f rB, b3Vec3 normal )
{
	b3Vec3 surfaceA = b3MulAdd( pA, rA, normal );
	b3Vec3 surfaceB = b3MulSub( pB, rB, normal );
	return b3MulCV( b3cFromFrac( 1, 2 ), b3Add( surfaceA, surfaceB ) );
}

/// Narrow a separation held wide at Q24 down to the b3f the manifold stores.
///
/// The separating axis test keeps every candidate separation at Q24 until it
/// leaves, because the margin between two face axes can be a single Q12
/// quantum and narrowing inside the loop would let rounding choose the
/// reference face. This is the one place the narrowing happens.
b3f b3SeparationFromWide( int64_t wide )
{
	return b3Makeb3fRef( (int32_t)B3_SHIFT_ROUND( wide, B3_F_SHIFT ), B3_REF( (double)wide / 16777216.0 ) );
}

/// Twice the signed area of the triangle spanned by u and v, measured about
/// `normal`, at Q24.
///
/// The narrow spelling -- b3Dot( normal, b3Cross( u, v ) ) -- is wrong here
/// for the reason recorded in distance.c: a cross product is an area, so it
/// shrinks quadratically, and b3Cross narrows the result back to Q12. Two
/// operands a hundredth of a unit long cross to exactly zero there. Keeping
/// the cross at Q24 and folding the Q12 normal in by hand gives Q36, shifted
/// once to land on the Q24 that squared lengths use.
///
/// Range: with points bounded by a hull extent of ~80 units the cross
/// components reach 2.1e11 and the result 2.6e15. At the B3_HUGE limit it
/// would be 6e18 -- still inside int64, but with only 1.5x of margin, so
/// hulls that large are outside what this is checked for.
int64_t b3SignedAreaWide( b3Vec3 normal, b3Vec3 u, b3Vec3 v )
{
	int64_t c[3];
	b3CrossWide( c, u, v );

	int64_t dot = (int64_t)b3Raw( normal.x ) * c[0] + (int64_t)b3Raw( normal.y ) * c[1] +
				  (int64_t)b3Raw( normal.z ) * c[2];

	return dot >> B3_F_SHIFT;
}
// =========================================================================
// Manifold reduction
// =========================================================================

/// Cut a manifold down to at most four points, keeping the ones that hold the
/// contact patch open.
///
/// Note this modifies the input array: each accepted point is removed by
/// swapping the last one over it, which is upstream's trick and is why
/// `count` is by value.
///
/// @section bias The 0.95 bias, and why it is a rational here
///
/// Upstream multiplies every candidate score by 0.95f before comparing it
/// against the running best, creating a pecking order so two candidates of
/// nearly equal merit do not swap places from one step to the next -- contact
/// point identity is what warm starting is built on, and flicker in it is
/// visible as jitter.
///
/// The scale of "score" changes between the four steps: a length at Q12 in
/// step 1, an area at Q24 in steps 2 through 4. A Q30 coefficient would
/// overflow the area comparisons -- 6.7e13 * 1.02e9 is 6.9e22, past int64 --
/// so the bias is applied as the exact rational 95/100 by cross-multiplying,
/// which keeps both sides at their native scale. Ranges: 8.2e8 in step 1,
/// 6.7e15 in steps 2 to 4.
///
/// That is marginally *more* exact than upstream, where 0.95f is really
/// 0.94999998807907104; the two differ only on an exact tie.
///
/// @section area Why the triangle areas run wide
///
/// Steps 3 and 4 score candidates by dot(normal, cross(ba, p - a)), where
/// both operands are differences between manifold points on a *single face*
/// and can be a few quanta long. b3Cross narrows to Q12, and the cross of two
/// 0.01-unit vectors is exactly zero there -- which is the configuration this
/// step exists to rank. So the cross stays wide and the dot with it is taken
/// by hand at Q36, shifted once to Q24 to meet tolSqr.
void b3ReduceManifoldPoints( b3LocalManifold* manifold, int capacity, b3LocalManifoldPoint* points, int count )
{
	if ( capacity < 4 )
	{
		return;
	}

	if ( count <= 4 )
	{
		for ( int index = 0; index < count; ++index )
		{
			manifold->points[index] = points[index];
		}

		manifold->pointCount = count;
		return;
	}

	b3Vec3 normal = manifold->normal;

	// B3_SPECULATIVE_DISTANCE squared, at Q24 -- the scale b3LengthSquaredWide
	// produces. 82 raw squared is 6724, which is four digits clear of
	// quantization, so unlike some Q12 thresholds this one means what it says.
	const int64_t tolSqr = (int64_t)b3Raw( B3_SPECULATIVE_DISTANCE ) * b3Raw( B3_SPECULATIVE_DISTANCE );

	const int64_t biasNum = 95;
	const int64_t biasDen = 100;

	// Step 1: the deepest point that is actually touching.
	int bestIndex = B3_NULL_INDEX;
	int64_t bestScore = 0;

	b3Vec3 searchDirection = b3ArbitraryPerp( normal );
	for ( int index = 0; index < count; ++index )
	{
		const b3LocalManifoldPoint* pt = points + index;

		if ( b3Raw( pt->separation ) > b3Raw( B3_SPECULATIVE_DISTANCE ) )
		{
			continue;
		}

		// A Q12 length. Note the bias *raises* a negative score rather than
		// lowering it -- asymmetric, and almost certainly not what upstream's
		// comment intends, but it decides which point wins and so is
		// transliterated rather than corrected.
		int64_t score = -(int64_t)b3Raw( pt->separation ) + ( b3DotWide( searchDirection, pt->point ) >> B3_F_SHIFT );
		if ( bestIndex == B3_NULL_INDEX || score * biasNum > bestScore * biasDen )
		{
			bestIndex = index;
			bestScore = score;
		}
	}

	if ( bestIndex == B3_NULL_INDEX )
	{
		manifold->pointCount = 0;
		return;
	}

	manifold->points[0] = points[bestIndex];
	manifold->pointCount = 1;

	points[bestIndex] = points[count - 1];
	count -= 1;

	b3Vec3 a = manifold->points[0].point;

	// Step 2: the point furthest from it in the contact plane, with depth
	// counted as a tiebreaker.
	bestScore = 0;
	bestIndex = B3_NULL_INDEX;

	for ( int index = 0; index < count; ++index )
	{
		b3Vec3 p = points[index].point;
		b3Vec3 d = b3Sub( p, a );
		b3Vec3 v = b3MulSub( d, b3Dot( d, normal ), normal );

		// Both terms land at Q24 with no shifting: a squared length from
		// b3LengthSquaredWide, and a raw separation squared. That is the one
		// place in this function where the scales line up for free.
		int64_t distanceSquared = b3LengthSquaredWide( v );
		int64_t separation = b3Raw( points[index].separation ) < 0 ? -(int64_t)b3Raw( points[index].separation ) : 0;
		int64_t score = distanceSquared + 4 * separation * separation;

		if ( score * biasNum > bestScore * biasDen )
		{
			bestScore = score;
			bestIndex = index;
		}
	}

	if ( bestScore < tolSqr || bestIndex == B3_NULL_INDEX )
	{
		return;
	}

	manifold->points[1] = points[bestIndex];
	manifold->pointCount = 2;

	points[bestIndex] = points[count - 1];
	count -= 1;

	b3Vec3 b = manifold->points[1].point;

	// Step 3: the point making the largest triangle with the first two.
	bestScore = tolSqr;
	bestIndex = B3_NULL_INDEX;
	int64_t bestSignedArea = 0;
	b3Vec3 ba = b3Sub( b, a );

	for ( int index = 0; index < count; ++index )
	{
		b3Vec3 p = points[index].point;
		int64_t signedArea = b3SignedAreaWide( normal, ba, b3Sub( p, a ) );
		int64_t score = signedArea < 0 ? -signedArea : signedArea;

		if ( score * biasNum >= bestScore * biasDen )
		{
			bestScore = score;
			bestIndex = index;
			bestSignedArea = signedArea;
		}
	}

	if ( bestIndex == B3_NULL_INDEX )
	{
		return;
	}

	manifold->points[2] = points[bestIndex];
	manifold->pointCount = 3;
	points[bestIndex] = points[count - 1];
	count -= 1;

	b3Vec3 c = manifold->points[2].point;

	// Step 4: the point adding the most area outside that triangle.
	bestScore = tolSqr;
	bestIndex = B3_NULL_INDEX;
	bool negate = bestSignedArea < 0;

	for ( int index = 0; index < count; ++index )
	{
		b3Vec3 p = points[index].point;
		int64_t u1 = b3SignedAreaWide( normal, b3Sub( p, a ), ba );
		int64_t u2 = b3SignedAreaWide( normal, b3Sub( p, b ), b3Sub( c, b ) );
		int64_t u3 = b3SignedAreaWide( normal, b3Sub( p, c ), b3Sub( a, c ) );

		// Upstream multiplies by a +/-1 sign; negating is the same thing
		// without a multiply.
		if ( negate )
		{
			u1 = -u1;
			u2 = -u2;
			u3 = -u3;
		}

		int64_t score = u1 > u2 ? u1 : u2;
		score = score > u3 ? score : u3;

		if ( score * biasNum > bestScore * biasDen )
		{
			bestScore = score;
			bestIndex = index;
		}
	}

	if ( bestIndex != B3_NULL_INDEX )
	{
		manifold->points[manifold->pointCount] = points[bestIndex];
		manifold->pointCount += 1;
	}
}

/// Clip a segment against a plane, keeping the part behind it.
int b3ClipSegment( b3ClipVertex segment[2], b3Plane plane )
{
	int vertexCount = 0;
	b3ClipVertex vertex1 = segment[0];
	b3ClipVertex vertex2 = segment[1];

	b3f distance1 = b3PlaneSeparation( plane, vertex1.position );
	b3f distance2 = b3PlaneSeparation( plane, vertex2.position );

	if ( b3Raw( distance1 ) <= 0 )
	{
		segment[vertexCount++] = vertex1;
	}
	if ( b3Raw( distance2 ) <= 0 )
	{
		segment[vertexCount++] = vertex2;
	}

	// Straddling the plane: interpolate the crossing point. The sign test is
	// done on the product, which is exact once widened.
	if ( (int64_t)b3Raw( distance1 ) * b3Raw( distance2 ) < 0 )
	{
		// t = d1 / (d1 - d2), in [0, 1] because the two have opposite signs,
		// so the denominator has the larger magnitude.
		b3c t = b3DivFFToC( distance1, b3SubF( distance1, distance2 ) );

		segment[vertexCount].position =
			b3Add( b3MulCV( b3SubC( b3c_one, t ), vertex1.position ), b3MulCV( t, vertex2.position ) );
		segment[vertexCount].pair = b3Raw( distance1 ) > 0 ? vertex1.pair : vertex2.pair;
		vertexCount++;
	}

	return vertexCount;
}
