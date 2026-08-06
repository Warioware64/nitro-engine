// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Erin Catto        (original Box3D)
// Copyright (c) 2026 Warioware64       (Nitro Engine Advanced fixed-point port)
//
// Dirk Gregorius contributed portions of this code
//
// This file is part of Nitro Engine Advanced

/// @file   distance.c
/// @brief  GJK closest points, shape casting and time of impact.
///
/// Three layers, each built on the one above it: the simplex solvers and
/// b3ShapeDistance (Phase 2A), b3ShapeCast's conservative advancement (Phase
/// 2A), and b3TimeOfImpact's separation function and root finder (Phase 7,
/// Stage 2). The two proxy helpers at the end joined in Phase 7 Stage 1.
///
/// @section deferred Deferred division
///
/// The reason GJK ports cleanly is that upstream never divides while it is
/// deciding *which* Voronoi region the origin lies in. Each barycentric
/// routine returns its numerators and a divisor separately, and the region
/// tests are sign comparisons on the numerators alone. Only once the region
/// is known does it divide, and only for the vertices that survived.
///
/// That is exactly the structure fixed point wants. The numerators and the
/// divisor stay in int64 at their natural scale, the sign tests are exact,
/// and the single division per surviving vertex goes through b3DivWideToC,
/// which normalizes the pair before dividing.
///
/// @section scale What scale the intermediates live at
///
/// These quantities are powers of length, and they get large fast:
///
///   edge      dot(ab, ab)                        length²   Q24 wide
///   triangle  dot(cross(ab,ac), cross(ab,ac))    length⁴   Q24 wide
///   tetra     scalar triple product              length³   Q24 wide
///
/// The triangle case is the tight one. With Q12 cross products and a wide
/// dot, the divisor reaches 3.2e18 for operands 500 units long and overflows
/// int64 near 600. Simplex vertices are Minkowski differences, so they are
/// bounded by the two shapes' extents plus their separation -- far below that
/// for any shape inside the documented world scale, but the limit is real.

#include "box3d/collision.h"
#include "box3d/constants.h"
#include "box3d/math_fixed.h"
#include "box3d/types.h"

#include "core.h"

#define B3_MAX_SIMPLEX_VERTICES 4
#define B3_MAX_GJK_ITERATIONS 32

// =========================================================================
// Support functions
// =========================================================================

int b3GetProxySupport( const b3ShapeProxy* proxy, b3Vec3 axis )
{
	int count = proxy->count;
	const b3Vec3* points = proxy->points;

	B3_ASSERT( count > 0 );
	B3_ASSERT( points != NULL );

	// Upstream shifts the first vertex to the origin "for improved
	// precision", because the proxy carries no transform and its points may
	// be far from the world origin. That argument is stronger here than in
	// float: the projections are dot products, so distant points would square
	// their coordinates before any comparison, and the difference between two
	// nearby candidates would be lost in the high bits of two large numbers.
	b3Vec3 origin = points[0];
	int maxIndex = 0;

	// Held wide. The projections are only ever compared against each other,
	// so there is no reason to narrow them, and narrowing is where two
	// candidates that differ slightly would become equal.
	int64_t maxProjection = 0;

	for ( int index = 1; index < count; ++index )
	{
		int64_t projection = b3DotWide( axis, b3Sub( points[index], origin ) );
		if ( projection > maxProjection )
		{
			maxIndex = index;
			maxProjection = projection;
		}
	}

	return maxIndex;
}

int b3GetPointSupport( const b3Vec3* points, int count, b3Vec3 axis )
{
	B3_ASSERT( count > 0 );
	B3_ASSERT( points != NULL );

	b3Vec3 origin = points[0];
	int maxIndex = 0;
	int64_t maxProjection = 0;

	for ( int index = 1; index < count; ++index )
	{
		int64_t projection = b3DotWide( axis, b3Sub( points[index], origin ) );
		if ( projection > maxProjection )
		{
			maxIndex = index;
			maxProjection = projection;
		}
	}

	return maxIndex;
}

// =========================================================================
// Barycentric coordinates
// =========================================================================
//
// Each of these fills an array whose last element is the divisor. Callers
// compare the earlier elements against zero to classify the region, and
// divide only once the region is settled. Everything is int64 at Q24.

/// Scalar triple product, held wide.
///
/// dot(a, cross(b, c)) is a volume -- a third power of length -- which
/// overflows Q12 at around 80 units. The cross stays at Q12 and the dot
/// accumulates wide, giving Q24.
static int64_t b3ScalarTripleProductWide( b3Vec3 a, b3Vec3 b, b3Vec3 c )
{
	return b3DotWide( a, b3Cross( b, c ) );
}

static void b3BarycentricCoordsEdge( int64_t out[3], b3Vec3 a, b3Vec3 b )
{
	b3Vec3 ab = b3Sub( b, a );

	// Last element is divisor
	int64_t divisor = b3DotWide( ab, ab );

	out[0] = b3DotWide( b, ab );
	out[1] = -b3DotWide( a, ab );
	out[2] = divisor;
}

static void b3BarycentricCoordsTri( int64_t out[4], b3Vec3 a, b3Vec3 b, b3Vec3 c )
{
	b3Vec3 ab = b3Sub( b, a );
	b3Vec3 ac = b3Sub( c, a );

	b3Vec3 bXC = b3Cross( b, c );
	b3Vec3 cXA = b3Cross( c, a );
	b3Vec3 aXB = b3Cross( a, b );

	b3Vec3 abXAc = b3Cross( ab, ac );

	// Last element is divisor. This is the length⁴ term -- see the file
	// header for where it runs out.
	int64_t divisor = b3DotWide( abXAc, abXAc );

	out[0] = b3DotWide( bXC, abXAc );
	out[1] = b3DotWide( cXA, abXAc );
	out[2] = b3DotWide( aXB, abXAc );
	out[3] = divisor;
}

static void b3BarycentricCoordsTet( int64_t out[5], b3Vec3 a, b3Vec3 b, b3Vec3 c, b3Vec3 d )
{
	b3Vec3 ab = b3Sub( b, a );
	b3Vec3 ac = b3Sub( c, a );
	b3Vec3 ad = b3Sub( d, a );

	// Last element is divisor, forced positive.
	int64_t divisor = b3ScalarTripleProductWide( ab, ac, ad );

	int64_t sign = divisor < 0 ? -1 : 1;
	out[0] = sign * b3ScalarTripleProductWide( b, c, d );
	out[1] = sign * b3ScalarTripleProductWide( a, d, c );
	out[2] = sign * b3ScalarTripleProductWide( a, b, d );
	out[3] = sign * b3ScalarTripleProductWide( a, c, b );
	out[4] = sign * divisor;
}

// =========================================================================
// Simplex metric and cache
// =========================================================================

/// A scale-invariant measure of the simplex, used only to decide whether a
/// cached simplex still describes the same configuration.
///
/// Upstream returns a length, an area or a volume depending on the vertex
/// count -- three different dimensions from one function. That is fine
/// because the value is only ever compared against another metric from the
/// same query, but it means there is no single correct scale for it.
///
/// Everything is normalized to Q24 here so that a comparison between metrics
/// of different vertex counts is at least not comparing values that differ by
/// a factor of 4096, even though such a comparison is meaningless anyway.
static int64_t b3GetMetric( const b3Simplex* simplex )
{
	int count = simplex->count;
	B3_ASSERT( 1 <= count && count <= 4 );

	const b3SimplexVertex* vertices = simplex->vertices;

	switch ( count )
	{
		case 1:
			return 0;

		case 2:
		{
			// Length, Q12 raw, promoted to Q24.
			b3f d = b3Distance( vertices[0].w, vertices[1].w );
			return (int64_t)b3Raw( d ) << B3_F_SHIFT;
		}

		case 3:
		{
			// Twice the triangle area is the cross product magnitude. The
			// cross is Q12 and its magnitude is an area, so promote to Q24
			// and halve.
			b3Vec3 a = vertices[0].w;
			b3Vec3 b = vertices[1].w;
			b3Vec3 c = vertices[2].w;
			b3f cross = b3Length( b3Cross( b3Sub( b, a ), b3Sub( c, a ) ) );
			return ( (int64_t)b3Raw( cross ) << B3_F_SHIFT ) / 2;
		}

		case 4:
		{
			// The triple product is already Q24.
			//
			// Upstream divides by 6, which is a tetrahedron's volume. This
			// shifts by 3 instead -- the same quantity scaled by 3/4 -- because
			// an int64 divide by 6 is a call to __aeabi_ldivmod, and it was the
			// only one left in either archive.
			//
			// That is sound only because of what this number is for, which the
			// comment above spells out: it is compared against another metric
			// from the same query and nothing else. Within a count-4 pair both
			// sides carry the same factor and it cancels. Across counts the
			// comparison is between a volume and an area and is meaningless
			// whatever the factor is. And the `metric2 <= 0` degeneracy test is
			// a sign test, which a positive scale cannot change.
			//
			// Measured, not argued: run_pair holds at 7886 / 0 / 30 either way.
			b3Vec3 a = vertices[0].w;
			b3Vec3 b = vertices[1].w;
			b3Vec3 c = vertices[2].w;
			b3Vec3 d = vertices[3].w;
			return b3ScalarTripleProductWide( b3Sub( b, a ), b3Sub( c, a ), b3Sub( d, a ) ) >> 3;
		}

		default:
			B3_ASSERT( !"Should never get here!" );
			break;
	}

	return 0;
}

static void b3WriteCache( b3SimplexCache* cache, const b3Simplex* simplex )
{
	int count = simplex->count;
	cache->metric = b3GetMetric( simplex );
	cache->count = (uint16_t)count;
	for ( int index = 0; index < count; ++index )
	{
		cache->indexA[index] = (uint8_t)simplex->vertices[index].indexA;
		cache->indexB[index] = (uint8_t)simplex->vertices[index].indexB;
	}
}

// =========================================================================
// Simplex solvers
// =========================================================================
//
// Not static: the host tests drive them directly, because the region
// classification is the part most worth testing in isolation -- a wrong sign
// here returns a plausible closest feature rather than failing.

bool b3SolveSimplex2( b3Simplex* simplex )
{
	b3SimplexVertex* vs = simplex->vertices;
	B3_ASSERT( simplex->count == 2 );

	b3Vec3 a = vs[0].w;
	b3Vec3 b = vs[1].w;
	b3Vec3 ab = b3Sub( b, a );

	// Last element is divisor
	int64_t divisor = b3DotWide( ab, ab );

	int64_t u = b3DotWide( b, ab );
	int64_t v = -b3DotWide( a, ab );

	// V( A )
	if ( v <= 0 )
	{
		simplex->count = 1;
		vs[0].a = b3c_one;
		return true;
	}

	// V( B )
	if ( u <= 0 )
	{
		simplex->count = 1;
		vs[0] = vs[1];
		vs[0].a = b3c_one;
		return true;
	}

	// Edge region. A non-positive divisor means the two vertices coincide to
	// within what the representation can distinguish; upstream tests the same
	// thing against float zero.
	if ( divisor <= 0 )
	{
		return false;
	}

	// VR( AB ). Upstream computes a reciprocal once and multiplies twice;
	// here two divisions are cheaper than a reciprocal, because forming the
	// reciprocal of a wide value would need the same normalization work as
	// the division itself.
	vs[0].a = b3DivWideToC( u, divisor );
	vs[1].a = b3DivWideToC( v, divisor );

	return true;
}

// Reduce the simplex to two vertices and normalize their weights.
//
// Six of b3SolveSimplex4's edge regions and three of b3SolveSimplex3's are
// this same block verbatim in upstream. Factoring it out leaves the region
// *conditions* -- the part that carries the actual geometry -- as the only
// thing that varies between them, and saves the repeated divisor check being
// written nine times.
static bool b3Reduce2( b3Simplex* simplex, b3SimplexVertex v0, b3SimplexVertex v1, const int64_t w[3] )
{
	simplex->count = 2;
	simplex->vertices[0] = v0;
	simplex->vertices[1] = v1;

	if ( w[2] <= 0 )
	{
		return false;
	}

	simplex->vertices[0].a = b3DivWideToC( w[0], w[2] );
	simplex->vertices[1].a = b3DivWideToC( w[1], w[2] );
	return true;
}

/// Reduce the simplex to three vertices and normalize their weights.
static bool b3Reduce3( b3Simplex* simplex, b3SimplexVertex v0, b3SimplexVertex v1, b3SimplexVertex v2, const int64_t w[4] )
{
	simplex->count = 3;
	simplex->vertices[0] = v0;
	simplex->vertices[1] = v1;
	simplex->vertices[2] = v2;

	if ( w[3] <= 0 )
	{
		return false;
	}

	simplex->vertices[0].a = b3DivWideToC( w[0], w[3] );
	simplex->vertices[1].a = b3DivWideToC( w[1], w[3] );
	simplex->vertices[2].a = b3DivWideToC( w[2], w[3] );
	return true;
}

/// Reduce the simplex to a single vertex.
static bool b3Reduce1( b3Simplex* simplex, b3SimplexVertex v0 )
{
	simplex->count = 1;
	simplex->vertices[0] = v0;
	simplex->vertices[0].a = b3c_one;
	return true;
}

bool b3SolveSimplex3( b3Simplex* simplex )
{
	b3SimplexVertex* vs = simplex->vertices;
	B3_ASSERT( simplex->count == 3 );

	// Copy out first: the reductions below overwrite vs while still reading
	// the original vertices.
	b3SimplexVertex v1 = vs[0];
	b3SimplexVertex v2 = vs[1];
	b3SimplexVertex v3 = vs[2];

	// Vertex regions
	int64_t wAB[3], wBC[3], wCA[3];
	b3BarycentricCoordsEdge( wAB, v1.w, v2.w );
	b3BarycentricCoordsEdge( wBC, v2.w, v3.w );
	b3BarycentricCoordsEdge( wCA, v3.w, v1.w );

	if ( wAB[1] <= 0 && wCA[0] <= 0 )
	{
		return b3Reduce1( simplex, v1 ); // VR( A )
	}

	if ( wBC[1] <= 0 && wAB[0] <= 0 )
	{
		return b3Reduce1( simplex, v2 ); // VR( B )
	}

	if ( wCA[1] <= 0 && wBC[0] <= 0 )
	{
		return b3Reduce1( simplex, v3 ); // VR( C )
	}

	// Edge regions
	int64_t wABC[4];
	b3BarycentricCoordsTri( wABC, v1.w, v2.w, v3.w );

	if ( wABC[2] <= 0 && wAB[0] > 0 && wAB[1] > 0 )
	{
		return b3Reduce2( simplex, v1, v2, wAB ); // VR( AB )
	}

	if ( wABC[0] <= 0 && wBC[0] > 0 && wBC[1] > 0 )
	{
		return b3Reduce2( simplex, v2, v3, wBC ); // VR( BC )
	}

	if ( wABC[1] <= 0 && wCA[0] > 0 && wCA[1] > 0 )
	{
		return b3Reduce2( simplex, v3, v1, wCA ); // VR( CA )
	}

	// Face region. A non-positive divisor means the triangle is degenerate --
	// collinear vertices, so the cross product vanished. The caller retries
	// with a reduced simplex rather than dividing by it.
	if ( wABC[3] <= 0 )
	{
		return false;
	}

	// VR( ABC )
	vs[0].a = b3DivWideToC( wABC[0], wABC[3] );
	vs[1].a = b3DivWideToC( wABC[1], wABC[3] );
	vs[2].a = b3DivWideToC( wABC[2], wABC[3] );

	return true;
}

bool b3SolveSimplex4( b3Simplex* simplex )
{
	b3SimplexVertex* vs = simplex->vertices;
	B3_ASSERT( simplex->count == 4 );

	// Copy out first: the reductions overwrite vs while still reading these.
	b3SimplexVertex vA = vs[0];
	b3SimplexVertex vB = vs[1];
	b3SimplexVertex vC = vs[2];
	b3SimplexVertex vD = vs[3];

	// Vertex regions. Six edges, each giving two numerators and a divisor.
	int64_t wAB[3], wAC[3], wAD[3], wBC[3], wCD[3], wDB[3];
	b3BarycentricCoordsEdge( wAB, vA.w, vB.w );
	b3BarycentricCoordsEdge( wAC, vA.w, vC.w );
	b3BarycentricCoordsEdge( wAD, vA.w, vD.w );
	b3BarycentricCoordsEdge( wBC, vB.w, vC.w );
	b3BarycentricCoordsEdge( wCD, vC.w, vD.w );
	b3BarycentricCoordsEdge( wDB, vD.w, vB.w );

	if ( wAB[1] <= 0 && wAC[1] <= 0 && wAD[1] <= 0 )
	{
		return b3Reduce1( simplex, vA ); // VR( A )
	}

	if ( wAB[0] <= 0 && wDB[0] <= 0 && wBC[1] <= 0 )
	{
		return b3Reduce1( simplex, vB ); // VR( B )
	}

	if ( wAC[0] <= 0 && wBC[0] <= 0 && wCD[1] <= 0 )
	{
		return b3Reduce1( simplex, vC ); // VR( C )
	}

	if ( wAD[0] <= 0 && wCD[0] <= 0 && wDB[1] <= 0 )
	{
		return b3Reduce1( simplex, vD ); // VR( D )
	}

	// Edge regions. The four face triangles decide which side of each face
	// the origin lies on; an edge owns the origin when both adjacent faces
	// point away and the edge's own numerators are positive.
	int64_t wACB[4], wABD[4], wADC[4], wBCD[4];
	b3BarycentricCoordsTri( wACB, vA.w, vC.w, vB.w );
	b3BarycentricCoordsTri( wABD, vA.w, vB.w, vD.w );
	b3BarycentricCoordsTri( wADC, vA.w, vD.w, vC.w );
	b3BarycentricCoordsTri( wBCD, vB.w, vC.w, vD.w );

	if ( wABD[2] <= 0 && wACB[1] <= 0 && wAB[0] > 0 && wAB[1] > 0 )
	{
		return b3Reduce2( simplex, vA, vB, wAB ); // VR( AB )
	}

	if ( wACB[2] <= 0 && wADC[1] <= 0 && wAC[0] > 0 && wAC[1] > 0 )
	{
		return b3Reduce2( simplex, vA, vC, wAC ); // VR( AC )
	}

	if ( wADC[2] <= 0 && wABD[1] <= 0 && wAD[0] > 0 && wAD[1] > 0 )
	{
		return b3Reduce2( simplex, vA, vD, wAD ); // VR( AD )
	}

	if ( wACB[0] <= 0 && wBCD[2] <= 0 && wBC[0] > 0 && wBC[1] > 0 )
	{
		return b3Reduce2( simplex, vB, vC, wBC ); // VR( BC )
	}

	if ( wADC[0] <= 0 && wBCD[0] <= 0 && wCD[0] > 0 && wCD[1] > 0 )
	{
		return b3Reduce2( simplex, vC, vD, wCD ); // VR( CD )
	}

	if ( wABD[0] <= 0 && wBCD[1] <= 0 && wDB[0] > 0 && wDB[1] > 0 )
	{
		return b3Reduce2( simplex, vD, vB, wDB ); // VR( DB )
	}

	// Face regions. The tetrahedron's four numerators are signed volumes;
	// a negative one means the origin is outside that face.
	int64_t wABCD[5];
	b3BarycentricCoordsTet( wABCD, vA.w, vB.w, vC.w, vD.w );

	if ( wABCD[3] < 0 && wACB[0] > 0 && wACB[1] > 0 && wACB[2] > 0 )
	{
		return b3Reduce3( simplex, vA, vC, vB, wACB ); // VR( ACB )
	}

	if ( wABCD[2] < 0 && wABD[0] > 0 && wABD[1] > 0 && wABD[2] > 0 )
	{
		return b3Reduce3( simplex, vA, vB, vD, wABD ); // VR( ABD )
	}

	if ( wABCD[1] < 0 && wADC[0] > 0 && wADC[1] > 0 && wADC[2] > 0 )
	{
		return b3Reduce3( simplex, vA, vD, vC, wADC ); // VR( ADC )
	}

	if ( wABCD[0] < 0 && wBCD[0] > 0 && wBCD[1] > 0 && wBCD[2] > 0 )
	{
		return b3Reduce3( simplex, vB, vC, vD, wBCD ); // VR( BCD )
	}

	// Inside the tetrahedron: the origin is enclosed, so the shapes overlap.
	//
	// A non-positive divisor here means the tetrahedron is degenerate -- the
	// four points are coplanar, so the signed volume vanished. That is the
	// case fixed point is most exposed to, because the volume is a third
	// power of length and a nearly-flat tetrahedron quantizes to exactly
	// zero. Returning false makes the caller fall back to a lower-dimensional
	// simplex, which is the correct response either way.
	if ( wABCD[4] <= 0 )
	{
		return false;
	}

	vs[0].a = b3DivWideToC( wABCD[0], wABCD[4] );
	vs[1].a = b3DivWideToC( wABCD[1], wABCD[4] );
	vs[2].a = b3DivWideToC( wABCD[2], wABCD[4] );
	vs[3].a = b3DivWideToC( wABCD[3], wABCD[4] );

	return true;
}

// =========================================================================
// Witness points
// =========================================================================

/// Blend two points by their barycentric weights.
static b3Vec3 b3Blend2( b3c a1, b3Vec3 w1, b3c a2, b3Vec3 w2 )
{
	return b3Add( b3MulCV( a1, w1 ), b3MulCV( a2, w2 ) );
}

/// Blend three points by their barycentric weights.
static b3Vec3 b3Blend3( b3c a1, b3Vec3 w1, b3c a2, b3Vec3 w2, b3c a3, b3Vec3 w3 )
{
	return b3Add( b3Add( b3MulCV( a1, w1 ), b3MulCV( a2, w2 ) ), b3MulCV( a3, w3 ) );
}

static void b3ComputeWitnessPoints( const b3Simplex* simplex, b3Vec3* vertexA, b3Vec3* vertexB )
{
	const b3SimplexVertex* vs = simplex->vertices;
	int count = simplex->count;
	B3_ASSERT( 1 <= count && count <= 4 );

	switch ( count )
	{
		case 1:
			*vertexA = vs[0].wA;
			*vertexB = vs[0].wB;
			break;

		case 2:
			*vertexA = b3Blend2( vs[0].a, vs[0].wA, vs[1].a, vs[1].wA );
			*vertexB = b3Blend2( vs[0].a, vs[0].wB, vs[1].a, vs[1].wB );
			break;

		case 3:
			*vertexA = b3Blend3( vs[0].a, vs[0].wA, vs[1].a, vs[1].wA, vs[2].a, vs[2].wA );
			*vertexB = b3Blend3( vs[0].a, vs[0].wB, vs[1].a, vs[1].wB, vs[2].a, vs[2].wB );
			break;

		case 4:
		{
			// Force identical points and *zero* distance: a full simplex
			// means the origin is inside the Minkowski difference, so the
			// shapes overlap and there is no separation to report.
			b3Vec3 sum = b3Add( b3Blend2( vs[0].a, vs[0].wA, vs[1].a, vs[1].wA ),
								b3Blend2( vs[2].a, vs[2].wA, vs[3].a, vs[3].wA ) );
			*vertexA = sum;
			*vertexB = sum;
		}
		break;

		default:
			B3_ASSERT( !"Should never get here!" );
			break;
	}
}

// =========================================================================
// GJK
// =========================================================================

b3DistanceOutput b3ShapeDistance( const b3DistanceInput* input, b3SimplexCache* cache, b3Simplex* simplexes, int simplexCapacity )
{
	// The query runs in frame A using the relative pose of B in A. That is
	// upstream's choice for precision, and it matters more here: working in a
	// local frame keeps the Minkowski points small, which is what stops the
	// length⁴ triangle divisor from running out of int64.
	b3Transform xf = input->transform;

	b3Matrix3 m = b3MakeMatrixFromQuat( xf.q );
	b3Matrix3 mt = b3Transpose( m );

	const b3ShapeProxy* proxyA = &input->proxyA;
	const b3ShapeProxy* proxyB = &input->proxyB;

	B3_ASSERT( cache->count <= B3_MAX_SIMPLEX_VERTICES );

	b3Simplex simplex = { 0 };
	b3SimplexVertex* vs = simplex.vertices;

	// Rebuild the simplex from the cache.
	simplex.count = cache->count;
	for ( int i = 0; i < cache->count; ++i )
	{
		int index1 = cache->indexA[i];
		int index2 = cache->indexB[i];

		B3_ASSERT( 0 <= index1 && index1 < proxyA->count );
		B3_ASSERT( 0 <= index2 && index2 < proxyB->count );

		b3Vec3 vertex1 = proxyA->points[index1];
		b3Vec3 vertex2 = b3Add( b3MulMV( m, proxyB->points[index2] ), xf.p );

		vs[i].indexA = index1;
		vs[i].indexB = index2;
		vs[i].wA = vertex1;
		vs[i].wB = vertex2;
		vs[i].w = b3Sub( vertex2, vertex1 );
		vs[i].a = b3c_zero;
	}

	// Flush the cache if the simplex has changed shape substantially.
	//
	// Upstream's third test is `metric2 < FLT_EPSILON`, which asks whether
	// the new simplex has collapsed to nothing. The metrics here are int64,
	// so the equivalent is simply zero -- there is no denormal range to worry
	// about, and a metric that quantized to zero is exactly the degenerate
	// case the test is looking for.
	if ( simplex.count > 0 )
	{
		int64_t metric1 = cache->metric;
		int64_t metric2 = b3GetMetric( &simplex );

		if ( 2 * metric1 < metric2 || metric2 < metric1 / 2 || metric2 <= 0 )
		{
			simplex.count = 0;
		}
	}

	if ( simplex.count == 0 )
	{
		b3Vec3 vertex1 = proxyA->points[0];
		b3Vec3 vertex2 = b3Add( b3MulMV( m, proxyB->points[0] ), xf.p );

		simplex.count = 1;
		simplex.vertices[0].indexA = 0;
		simplex.vertices[0].indexB = 0;
		simplex.vertices[0].wA = vertex1;
		simplex.vertices[0].wB = vertex2;
		simplex.vertices[0].w = b3Sub( vertex2, vertex1 );
		simplex.vertices[0].a = b3c_zero;
	}

	b3Simplex backup = { 0 };

	int simplexIndex = 0;
	if ( simplexes != NULL && simplexIndex < simplexCapacity )
	{
		simplexes[simplexIndex] = simplex;
		simplexIndex += 1;
	}

	b3DistanceOutput distanceOutput = { 0 };

	// The squared distance to the origin, held wide.
	//
	// This is the value the progression test compares against itself each
	// iteration, and the iterations converge -- so late in the loop two
	// successive values differ by very little. Narrowing to Q12 would make
	// them compare equal while GJK was still making progress, and the loop
	// would exit early with a simplex that had not converged.
	int64_t distanceSq = INT64_MAX;

	b3Vec3 normal = b3Vec3_zero;

	int iteration = 0;
	for ( ; iteration < B3_MAX_GJK_ITERATIONS; ++iteration )
	{
		bool solved = false;
		switch ( simplex.count )
		{
			case 1:
				simplex.vertices[0].a = b3c_one;
				solved = true;
				break;

			case 2:
				solved = b3SolveSimplex2( &simplex );
				break;

			case 3:
				solved = b3SolveSimplex3( &simplex );
				break;

			case 4:
				solved = b3SolveSimplex4( &simplex );
				break;

			default:
				B3_ASSERT( !"Should never get here!" );
				break;
		}

		if ( solved == false )
		{
			// A degenerate simplex. Fall back to the last good one.
			if ( backup.count == 0 )
			{
				break;
			}
			simplex = backup;
			break;
		}

		if ( simplexes != NULL && simplexIndex < simplexCapacity )
		{
			simplexes[simplexIndex] = simplex;
			simplexIndex += 1;
			distanceOutput.iterations = iteration;
			distanceOutput.simplexCount = simplexIndex;
		}

		if ( simplex.count == B3_MAX_SIMPLEX_VERTICES )
		{
			// The origin is enclosed: the shapes overlap.
			b3Vec3 localPointA, localPointB;
			b3ComputeWitnessPoints( &simplex, &localPointA, &localPointB );
			distanceOutput.pointA = localPointA;
			distanceOutput.pointB = localPointB;
			return distanceOutput;
		}

		int64_t oldDistanceSq = distanceSq;

		b3Vec3 closestPoint = b3Vec3_zero;

		switch ( simplex.count )
		{
			case 1:
				closestPoint = vs[0].w;
				break;

			case 2:
				closestPoint = b3Blend2( vs[0].a, vs[0].w, vs[1].a, vs[1].w );
				break;

			case 3:
				closestPoint = b3Blend3( vs[0].a, vs[0].w, vs[1].a, vs[1].w, vs[2].a, vs[2].w );
				break;

			case 4:
				closestPoint = b3Add( b3Blend2( vs[0].a, vs[0].w, vs[1].a, vs[1].w ),
									  b3Blend2( vs[2].a, vs[2].w, vs[3].a, vs[3].w ) );
				break;

			default:
				B3_ASSERT( !"Should never get here!" );
				break;
		}

		distanceSq = b3LengthSquaredWide( closestPoint );

		if ( distanceSq >= oldDistanceSq )
		{
			// No progress. Fall back to the last good simplex.
			if ( backup.count == 0 )
			{
				break;
			}
			simplex = backup;
			break;
		}

		// Build the next search direction.
		//
		// Only the *direction* of this is ever read -- it picks the support
		// points, and at the end it becomes the contact normal, which is
		// normalized. Its magnitude carries no information, and in Q12 it is
		// actively harmful: a cross product is an area, so the direction for
		// a thin simplex underflows to a handful of raw units or to nothing.
		// b3CrossDirection rescales each result to about a unit long, which
		// is exact in the only property being used.
		//
		// This is not a refinement. Without it, a query point sitting under a
		// large flat hull face makes the support triangle thin enough that
		// b3Cross returns a vector too short to normalize, the final
		// b3IsNormalized test below fails, and GJK reports a **spurious
		// overlap** -- a positive distance turning into zero. It was found by
		// the run_pair cone cases and would have looked like a solver bug
		// three phases later.
		b3Vec3 searchDirection = b3Vec3_zero;

		switch ( simplex.count )
		{
			case 1:
				// v = -A
				searchDirection = b3DirectionFromWide( -(int64_t)b3Raw( vs[0].w.x ), -(int64_t)b3Raw( vs[0].w.y ),
													   -(int64_t)b3Raw( vs[0].w.z ) );
				break;

			case 2:
			{
				// v = (AB x AO) x AB
				b3Vec3 a = vs[0].w;
				b3Vec3 b = vs[1].w;
				b3Vec3 ab = b3Sub( b, a );

				// Rescaled between the two crosses as well as after, so the
				// second one is taken on a unit-length operand rather than on
				// an area that has already lost its low bits.
				b3Vec3 n = b3CrossDirection( ab, b3Neg( a ) );
				searchDirection = b3CrossDirection( n, ab );
			}
			break;

			case 3:
			{
				// v = AB x AC, oriented away from the origin.
				b3Vec3 a = vs[0].w;
				b3Vec3 b = vs[1].w;
				b3Vec3 c = vs[2].w;

				int64_t n[3];
				b3CrossWide( n, b3Sub( b, a ), b3Sub( c, a ) );

				// Sign test on the wide value, before any rescaling, so the
				// orientation is decided exactly.
				int64_t side = n[0] * (int64_t)b3Raw( a.x ) + n[1] * (int64_t)b3Raw( a.y ) + n[2] * (int64_t)b3Raw( a.z );

				searchDirection = side < 0 ? b3DirectionFromWide( n[0], n[1], n[2] )
										   : b3DirectionFromWide( -n[0], -n[1], -n[2] );
			}
			break;

			default:
				B3_ASSERT( !"Should never get here!" );
				break;
		}

		// Upstream compares the squared length against 1000*FLT_MIN, which is
		// a statement about float denormals. Here the direction has genuinely
		// vanished only when b3DirectionFromWide was handed an exact zero --
		// the rescale above means a short direction is no longer indistinguishable
		// from an absent one -- and that means the origin lies on the current
		// segment or triangle, so the shapes overlap.
		if ( b3LengthSquaredWide( searchDirection ) == 0 )
		{
			b3Vec3 localPointA, localPointB;
			b3ComputeWitnessPoints( &simplex, &localPointA, &localPointB );
			distanceOutput.pointA = localPointA;
			distanceOutput.pointB = localPointB;
			return distanceOutput;
		}

		normal = b3Neg( searchDirection );

		// New support points, one per shape, in frame A.
		int indexA = b3GetProxySupport( &input->proxyA, b3Neg( searchDirection ) );
		b3Vec3 supportA = input->proxyA.points[indexA];

		b3Vec3 searchDirectionB = b3MulMV( mt, searchDirection );
		int indexB = b3GetProxySupport( &input->proxyB, searchDirectionB );
		b3Vec3 supportB = b3Add( b3MulMV( m, input->proxyB.points[indexB] ), xf.p );

		backup = simplex;

		// Duplicate support points are the main termination criterion, and
		// the comparison is on *indices* rather than positions -- which is
		// why it survives fixed point unchanged. A position-based test would
		// have needed a tolerance.
		bool duplicate = false;
		for ( int i = 0; i < simplex.count; ++i )
		{
			if ( vs[i].indexA == indexA && vs[i].indexB == indexB )
			{
				duplicate = true;
				break;
			}
		}

		if ( duplicate )
		{
			break;
		}

		vs[simplex.count].indexA = indexA;
		vs[simplex.count].indexB = indexB;
		vs[simplex.count].wA = supportA;
		vs[simplex.count].wB = supportB;
		vs[simplex.count].w = b3Sub( supportB, supportA );
		simplex.count += 1;
	}

	normal = b3Normalize( normal );
	if ( b3IsNormalized( normal ) == false )
	{
		// The normal collapsed, so treat this as an overlap.
		return distanceOutput;
	}

	b3Vec3 localPointA, localPointB;
	b3ComputeWitnessPoints( &simplex, &localPointA, &localPointB );
	b3WriteCache( cache, &simplex );

	distanceOutput.pointA = localPointA;
	distanceOutput.pointB = localPointB;
	distanceOutput.distance = b3Distance( localPointA, localPointB );
	distanceOutput.normal = normal;
	distanceOutput.iterations = iteration;
	distanceOutput.simplexCount = simplexIndex;

	if ( input->useRadii )
	{
		b3f rA = input->proxyA.radius;
		b3f rB = input->proxyB.radius;
		distanceOutput.distance = b3MaxF( b3f_zero, b3SubF( b3SubF( distanceOutput.distance, rA ), rB ) );

		// Keep the closest points on the perimeter even when overlapped, so
		// they move smoothly rather than jumping when contact is made.
		distanceOutput.pointA = b3Add( distanceOutput.pointA, b3MulSV( rA, normal ) );
		distanceOutput.pointB = b3Sub( distanceOutput.pointB, b3MulSV( rB, normal ) );
	}

	return distanceOutput;
}

// =========================================================================
// Shape cast
// =========================================================================

b3CastOutput b3ShapeCast( const b3ShapeCastPairInput* input )
{
	b3f linearSlop = B3_LINEAR_SLOP;
	b3f totalRadius = b3AddF( input->proxyA.radius, input->proxyB.radius );
	b3f target = b3MaxF( linearSlop, b3SubF( totalRadius, linearSlop ) );

	// A quarter of the linear slop. In Q12 the slop is 20 raw, so the
	// tolerance is 5 -- small but still several quanta, which is what matters:
	// a tolerance below one quantum would make the convergence test compare
	// two values that can never differ.
	b3f tolerance = b3Makeb3f( b3Raw( linearSlop ) / 4 );

	B3_ASSERT( b3Raw( target ) > b3Raw( tolerance ) );

	b3SimplexCache cache = { 0 };

	// The fraction along the sweep, accumulated across iterations.
	b3c alpha = b3c_zero;

	b3DistanceInput distanceInput = { 0 };
	distanceInput.proxyA = input->proxyA;
	distanceInput.proxyB = input->proxyB;
	distanceInput.useRadii = false;

	// The whole cast runs in frame A, advancing B's relative pose each
	// iteration. That keeps the geometry near the local origin instead of
	// re-relativizing world poses -- the same reason b3ShapeDistance works in
	// frame A, and worth more in fixed point than in float.
	distanceInput.transform = input->transform;

	b3Vec3 delta2 = input->translationB;
	b3CastOutput output = { 0 };
	output.triangleIndex = B3_NULL_INDEX;

	const int maxIterations = 20;

	for ( int iteration = 0; iteration < maxIterations; ++iteration )
	{
		output.iterations += 1;

		b3DistanceOutput distanceOutput = b3ShapeDistance( &distanceInput, &cache, NULL, 0 );

		if ( b3Raw( distanceOutput.distance ) < b3Raw( b3AddF( target, tolerance ) ) )
		{
			if ( iteration == 0 )
			{
				if ( input->canEncroach && b3Raw( distanceOutput.distance ) > 2 * b3Raw( linearSlop ) )
				{
					target = b3SubF( distanceOutput.distance, linearSlop );
				}
				else
				{
					// Already touching at the start of the sweep.
					output.hit = true;

					b3Vec3 c1 = b3MulAdd( distanceOutput.pointA, input->proxyA.radius, distanceOutput.normal );
					b3Vec3 c2 = b3MulSub( distanceOutput.pointB, input->proxyB.radius, distanceOutput.normal );
					output.point = b3Lerp( c1, c2, b3cFromFrac( 1, 2 ) );
					return output;
				}
			}
			else
			{
				// Upstream logs the whole input here when the normal is not
				// normalized, to diagnose extreme data. That diagnostic is
				// dropped: it prints a dozen floats, which this build has no
				// formatter for, and b3Log compiles to nothing in release
				// anyway. The bail-out it guards is kept.
				if ( b3Raw( distanceOutput.distance ) > 0 && b3IsNormalized( distanceOutput.normal ) == false )
				{
					b3Log( "shape cast: degenerate normal, likely extreme input" );
					return output;
				}

				output.fraction = alpha;
				output.point = b3MulAdd( distanceOutput.pointA, input->proxyA.radius, distanceOutput.normal );
				output.normal = distanceOutput.normal;
				output.hit = true;
				return output;
			}
		}

		// Are the shapes approaching? The normal is unit length, so this is a
		// length and its sign is all that is read.
		b3f denominator = b3Dot( delta2, distanceOutput.normal );
		if ( b3Raw( denominator ) >= 0 )
		{
			// Separating: a miss.
			return output;
		}

		// Advance the sweep.
		//
		// Both operands are lengths so the quotient is dimensionless, but it
		// is *not* bounded: a nearly-tangential sweep gives a tiny denominator
		// and a huge step. Upstream relies on float producing a large value
		// that the maxFraction test then rejects. b3DivWideToC saturates
		// instead of wrapping, which preserves that behaviour -- the
		// saturated value is still far above maxFraction, so the same branch
		// is taken. b3DivFFToC would have wrapped here.
		b3c step = b3DivWideToC( (int64_t)b3Raw( b3SubF( target, distanceOutput.distance ) ), (int64_t)b3Raw( denominator ) );
		alpha = b3AddC( alpha, step );

		if ( b3Raw( alpha ) >= b3Raw( input->maxFraction ) )
		{
			// Swept past the allowed fraction without touching.
			return output;
		}

		distanceInput.transform.p = b3Add( input->transform.p, b3MulCV( alpha, delta2 ) );
	}

	// Ran out of iterations.
	return output;
}

// =========================================================================
// Proxy helpers
// =========================================================================
//
// These two used to live in shape.c, and moved here in Phase 7. Neither has
// anything to do with a b3Shape: both take a b3ShapeProxy, which is a public
// point cloud with a radius, and they sit naturally beside b3ShapeDistance and
// b3ShapeCast, the two functions that consume one.
//
// The move was forced by a link rather than chosen on taste, and the link was
// right. tests/box3d_host builds `bake_ref` from a deliberately minimal set of
// objects -- the mesh baker needs b3IsValidMesh and therefore mesh.c, and
// nothing else. When Phase 7 gave mesh.c its query half, calling these two
// through shape.c would have dragged the body list, the broad phase and the
// contact graph into a command-line mesh baker. Here, they cost distance.c,
// which needs nothing outside that set.

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
// Time of impact
// =========================================================================
//
// The first time two moving convex shapes touch, found by root finding on a
// separating axis. Three nested loops, and each one has a different job:
//
//   outer   pick a separating axis from a GJK distance query at t1, and
//           advance t1 until the shapes are within `target` of each other
//   middle  resolve the deepest point along that axis, one witness pair at a
//           time ("push back")
//   inner   the actual root find on f(t) = separation(t) - target, alternating
//           false position with bisection
//
// The outer loop is what makes this robust: the axis is only valid near t1, so
// each advance re-derives it rather than trusting the old one.
//
// @section toiscale What lives at which scale
//
// Time fractions are b3c (Q30). That is not for precision -- it is because
// bisection halves, and halving a Q30 value is a shift with no rounding at
// all, so the bracket is exact for as long as it is meaningful.
//
// Separations are b3f (Q12), like every other length. The pairing of the two
// is the one genuinely new fixed-point problem in this file, and it is handled
// by the bracket floor in b3TimeOfImpact: Q30 can keep subdividing time long
// after Q12 has stopped being able to tell two separations apart, and a root
// finder that does not know that will spend its whole iteration budget
// comparing a value against itself.

/// The pose a sweep has reached at `time`.
///
/// The rotation is an nlerp rather than a slerp, as upstream: over one time
/// step the two quaternions are close, so the constant-speed property a slerp
/// buys is not worth an acos and a pair of sines.
///
/// The equal-rotation fast path is the port's, and it is not a micro
/// optimization. b3NLerp ends in b3NormalizeQuat, which is a reciprocal square
/// root, and the mesh path in b3ShapeTimeOfImpact sweeps against a *static*
/// mesh -- so sweepA has q1 == q2 and this is called with it once per
/// separation evaluation, per triangle, per iteration. Skipping it there costs
/// one comparison of four Q30 words.
b3Transform b3GetSweepTransform( const b3Sweep* sweep, b3c time )
{
	B3_ASSERT( 0 <= b3Raw( time ) && b3Raw( time ) <= B3_C_ONE );

	b3Transform transform;

	if ( b3Raw( sweep->q1.v.x ) == b3Raw( sweep->q2.v.x ) && b3Raw( sweep->q1.v.y ) == b3Raw( sweep->q2.v.y ) &&
		 b3Raw( sweep->q1.v.z ) == b3Raw( sweep->q2.v.z ) && b3Raw( sweep->q1.s ) == b3Raw( sweep->q2.s ) )
	{
		transform.q = sweep->q1;
	}
	else
	{
		transform.q = b3NLerp( sweep->q1, sweep->q2, time );
	}

	transform.p = b3Sub( b3Lerp( sweep->c1, sweep->c2, time ), b3RotateVector( transform.q, sweep->localCenter ) );
	return transform;
}

/// The pose at the end of the sweep, without interpolating to get there.
static inline b3Transform b3GetFinalSweepTransform( const b3Sweep* sweep )
{
	b3Transform transform;
	transform.q = sweep->q2;
	transform.p = b3Sub( sweep->c2, b3RotateVector( transform.q, sweep->localCenter ) );
	return transform;
}

/// How many distinct vertices the cached simplex names on one shape.
///
/// The cache stores `count` indices, but a simplex can name the same vertex
/// twice -- an edge that degenerated to a point, a triangle standing on an
/// edge. The separation function needs to know which geometric feature it
/// really has before it can pick an axis, and this is that test.
static int b3UniqueCount( int vertexCount, int vertices[3] )
{
	B3_ASSERT( 1 <= vertexCount && vertexCount <= 3 );

	switch ( vertexCount )
	{
		case 1:
			return 1;

		case 2:
			return vertices[0] != vertices[1] ? 2 : 1;

		case 3:
			if ( vertices[0] != vertices[1] && vertices[0] != vertices[2] && vertices[1] != vertices[2] )
			{
				// All different
				return 3;
			}

			if ( vertices[0] == vertices[1] && vertices[0] == vertices[2] && vertices[1] == vertices[2] )
			{
				// All equal
				return 1;
			}

			return 2;

		default:
			B3_ASSERT( !"Should never get here!" );
			return 0;
	}
}

/// Would the edge-edge cross product flip sign before the sweep ends?
///
/// An edge-edge separating axis is cross(edgeA, edgeB), and that reverses when
/// the two edges rotate through being parallel. If it does, the axis stops
/// describing the same side of the contact partway through the sweep and the
/// root finder is chasing a function that changes sign under it. Cheaper to
/// ask the question once, at the end pose, and fall back to a fixed world axis
/// than to detect the failure afterwards.
static inline bool b3CheckFastEdges( b3Transform xfA, b3Vec3 localEdgeA, b3Transform xfB, b3Vec3 localEdgeB, b3Vec3 axis0 )
{
	// The local witness axes, so a flipped one stays flipped.
	b3Vec3 edgeA = b3RotateVector( xfA.q, localEdgeA );
	b3Vec3 edgeB = b3RotateVector( xfB.q, localEdgeB );

	// Direction only: the magnitude of this cross is an area and two nearly
	// parallel unit edges make it vanish in Q12. Only the sign of the dot is
	// read, and rescaling cannot change a sign.
	b3Vec3 axis = b3CrossDirection( edgeA, edgeB );
	return b3Raw( b3Dot( axis, axis0 ) ) < 0;
}

typedef enum b3SeparationType
{
	b3_separationUnknown = 0,
	b3_separationVertices,
	b3_separationEdges,
	b3_separationFaceA,
	b3_separationFaceB,
} b3SeparationType;

typedef struct b3SeparationFunction
{
	const b3ShapeProxy* proxyA;
	const b3ShapeProxy* proxyB;
	b3Sweep sweepA;
	b3Sweep sweepB;

	// Which body these belong to depends on the type -- both can be on A.
	b3Vec3 witness1;
	b3Vec3 witness2;

	b3SeparationType type;
} b3SeparationFunction;

/// The smallest edge-edge cross product worth normalizing, squared.
///
/// Upstream rejects |cross(eA, eB)| < 0.05 for unit edges, which is sin(alpha)
/// between two directions and so a near-parallel test. The port takes the same
/// threshold on a *wide* cross, where both sides are exact integers: two Q12
/// unit vectors cross to Q24, so 0.05 is (B3_F_ONE / 20) * B3_F_ONE = 835,584
/// and the test never rounds.
///
/// Upstream uses a second, ten times smaller threshold (0.005) in the
/// three-point branch. **The port uses this one in both places.** At
/// sin(alpha) = 0.005 the two normalized Q12 edges cross to components of
/// about 20 raw, and b3Normalize has already spent a quantum or two of each
/// input -- so the axis that survives is some 10-20% wrong in *direction*, and
/// the root finder would then be converging carefully onto a function built on
/// it. The fallback is the fixed world axis, which is always valid and merely
/// converges more slowly. Q12 cannot honour the tighter threshold, so it does
/// not claim to.
#define B3_EDGE_TOLERANCE_WIDE ( (int64_t)( B3_F_ONE / 20 ) * B3_F_ONE )
#define B3_EDGE_TOLERANCE_SQ ( B3_EDGE_TOLERANCE_WIDE * B3_EDGE_TOLERANCE_WIDE )

/// The squared length of a wide cross product, at Q48.
///
/// Each component is bounded by B3_F_ONE * B3_F_ONE for unit inputs, so the
/// sum of three squares reaches 8.4e14 -- four orders below the int64 ceiling.
static inline int64_t b3WideLengthSquared( const int64_t c[3] )
{
	return c[0] * c[0] + c[1] * c[1] + c[2] * c[2];
}

static b3SeparationFunction b3MakeSeparationFunction( const b3SimplexCache cache, const b3ShapeProxy* proxyA,
													  const b3Sweep* sweepA, const b3ShapeProxy* proxyB, const b3Sweep* sweepB,
													  b3Vec3 worldNormal, b3c t1 )
{
	B3_ASSERT( 1 <= cache.count && cache.count <= 3 );

	b3SeparationFunction fcn = { 0 };
	fcn.proxyA = proxyA;
	fcn.proxyB = proxyB;
	fcn.sweepA = *sweepA;
	fcn.sweepB = *sweepB;
	fcn.type = b3_separationUnknown;

	int indexA[3] = { cache.indexA[0], cache.indexA[1], cache.indexA[2] };
	int indexB[3] = { cache.indexB[0], cache.indexB[1], cache.indexB[2] };

	int uniqueCountA = b3UniqueCount( cache.count, indexA );
	int uniqueCountB = b3UniqueCount( cache.count, indexB );

	b3Transform xfA1 = b3GetSweepTransform( sweepA, t1 );
	b3Transform xfB1 = b3GetSweepTransform( sweepB, t1 );

	b3Quat qA = xfA1.q;
	b3Quat qB = xfB1.q;

	// Difference the origins rather than the transformed points, so the terms
	// that are about to be dotted together stay small.
	b3Vec3 deltaP = b3Sub( xfB1.p, xfA1.p );

	switch ( cache.count )
	{
		case 1:
		{
			// The witness is the world direction itself.
			fcn.type = b3_separationVertices;
			fcn.witness1 = worldNormal;
		}
		break;

		case 2:
		{
			if ( uniqueCountA == 2 && uniqueCountB == 2 )
			{
				// Edge versus edge.
				b3Vec3 vA1 = proxyA->points[indexA[0]];
				b3Vec3 localEdgeA = b3Normalize( b3Sub( proxyA->points[indexA[1]], vA1 ) );
				b3Vec3 edgeA = b3RotateVector( qA, localEdgeA );

				b3Vec3 vB1 = proxyB->points[indexB[0]];
				b3Vec3 localEdgeB = b3Normalize( b3Sub( proxyB->points[indexB[1]], vB1 ) );
				b3Vec3 edgeB = b3RotateVector( qB, localEdgeB );

				int64_t wide[3];
				b3CrossWide( wide, edgeA, edgeB );

				if ( b3WideLengthSquared( wide ) < B3_EDGE_TOLERANCE_SQ )
				{
					// Too near parallel to normalize: use a world axis.
					fcn.type = b3_separationVertices;
					fcn.witness1 = worldNormal;
				}
				else
				{
					b3Vec3 axis = b3DirectionFromWide( wide[0], wide[1], wide[2] );

					b3Vec3 delta = b3Add( b3Sub( b3RotateVector( qB, vB1 ), b3RotateVector( qA, vA1 ) ), deltaP );
					if ( b3Raw( b3Dot( delta, axis ) ) < 0 )
					{
						// Point the axis from A to B.
						axis = b3Neg( axis );
						localEdgeB = b3Neg( localEdgeB );
					}

					b3Transform xfA2 = b3GetFinalSweepTransform( sweepA );
					b3Transform xfB2 = b3GetFinalSweepTransform( sweepB );
					if ( b3CheckFastEdges( xfA2, localEdgeA, xfB2, localEdgeB, axis ) )
					{
						// The cross product flips before the sweep ends. Freeze
						// the axis as it is now.
						fcn.type = b3_separationVertices;
						fcn.witness1 = b3Normalize( axis );
					}
					else
					{
						// Safe, and it converges faster than a fixed axis.
						fcn.type = b3_separationEdges;
						fcn.witness1 = localEdgeA;
						fcn.witness2 = localEdgeB;
					}
				}
			}
			else
			{
				// Vertex versus edge: no local feature to track, use the world
				// axis.
				fcn.type = b3_separationVertices;
				fcn.witness1 = worldNormal;
			}
		}
		break;

		case 3:
		{
			if ( uniqueCountA == 3 )
			{
				b3Vec3 vA1 = proxyA->points[indexA[0]];
				b3Vec3 vA2 = proxyA->points[indexA[1]];
				b3Vec3 vA3 = proxyA->points[indexA[2]];
				b3Vec3 localAxisA = b3Normalize( b3CrossDirection( b3Sub( vA2, vA1 ), b3Sub( vA3, vA1 ) ) );
				b3Vec3 axisA = b3RotateVector( qA, localAxisA );

				// The face centroid. Divide by three once, on the sum.
				b3Vec3 sumA = b3Add( b3Add( vA1, vA2 ), vA3 );
				b3Vec3 localPointA = b3MulCV( b3cFromFrac( 1, 3 ), sumA );

				b3Vec3 localPointB = proxyB->points[indexB[0]];
				b3Vec3 delta = b3Add( b3Sub( b3RotateVector( qB, localPointB ), b3RotateVector( qA, localPointA ) ), deltaP );

				if ( b3Raw( b3Dot( delta, axisA ) ) < 0 )
				{
					// Point the axis from A to B.
					localAxisA = b3Neg( localAxisA );
				}

				fcn.type = b3_separationFaceA;
				fcn.witness1 = localAxisA;
				fcn.witness2 = localPointA;
			}
			else if ( uniqueCountB == 3 )
			{
				b3Vec3 vB1 = proxyB->points[indexB[0]];
				b3Vec3 vB2 = proxyB->points[indexB[1]];
				b3Vec3 vB3 = proxyB->points[indexB[2]];
				b3Vec3 localAxisB = b3Normalize( b3CrossDirection( b3Sub( vB2, vB1 ), b3Sub( vB3, vB1 ) ) );
				b3Vec3 axisB = b3RotateVector( qB, localAxisB );

				b3Vec3 localPointA = proxyA->points[indexA[0]];
				b3Vec3 sumB = b3Add( b3Add( vB1, vB2 ), vB3 );
				b3Vec3 localPointB = b3MulCV( b3cFromFrac( 1, 3 ), sumB );

				b3Vec3 delta = b3Sub( b3Sub( b3RotateVector( qA, localPointA ), b3RotateVector( qB, localPointB ) ), deltaP );

				if ( b3Raw( b3Dot( delta, axisB ) ) < 0 )
				{
					// Point the axis from B to A.
					localAxisB = b3Neg( localAxisB );
				}

				fcn.type = b3_separationFaceB;
				fcn.witness1 = localAxisB;
				fcn.witness2 = localPointB;
			}
			else
			{
				B3_ASSERT( uniqueCountA == 2 && uniqueCountB == 2 );

				if ( indexA[0] == indexA[1] )
				{
					// Make the first two indices the distinct pair.
					indexA[1] = indexA[2];
					B3_ASSERT( indexA[0] != indexA[1] );
				}

				b3Vec3 vA1 = proxyA->points[indexA[0]];
				b3Vec3 localEdgeA = b3Normalize( b3Sub( proxyA->points[indexA[1]], vA1 ) );
				b3Vec3 edgeA = b3RotateVector( qA, localEdgeA );

				if ( indexB[0] == indexB[1] )
				{
					indexB[1] = indexB[2];
					B3_ASSERT( indexB[0] != indexB[1] );
				}

				b3Vec3 vB1 = proxyB->points[indexB[0]];
				b3Vec3 localEdgeB = b3Normalize( b3Sub( proxyB->points[indexB[1]], vB1 ) );
				b3Vec3 edgeB = b3RotateVector( qB, localEdgeB );

				int64_t wide[3];
				b3CrossWide( wide, edgeA, edgeB );

				// See B3_EDGE_TOLERANCE_SQ: upstream's second threshold here is
				// ten times tighter and Q12 cannot honour it.
				if ( b3WideLengthSquared( wide ) < B3_EDGE_TOLERANCE_SQ )
				{
					fcn.type = b3_separationVertices;
					fcn.witness1 = worldNormal;
				}
				else
				{
					b3Vec3 axis = b3DirectionFromWide( wide[0], wide[1], wide[2] );

					b3Vec3 delta = b3Add( b3Sub( b3RotateVector( qB, vB1 ), b3RotateVector( qA, vA1 ) ), deltaP );
					if ( b3Raw( b3Dot( delta, axis ) ) < 0 )
					{
						axis = b3Neg( axis );
						localEdgeB = b3Neg( localEdgeB );
					}

					b3Transform xfA2 = b3GetFinalSweepTransform( sweepA );
					b3Transform xfB2 = b3GetFinalSweepTransform( sweepB );
					if ( b3CheckFastEdges( xfA2, localEdgeA, xfB2, localEdgeB, axis ) )
					{
						fcn.type = b3_separationVertices;
						fcn.witness1 = b3Normalize( axis );
					}
					else
					{
						fcn.type = b3_separationEdges;
						fcn.witness1 = localEdgeA;
						fcn.witness2 = localEdgeB;
					}
				}
			}
		}
		break;

		default:
			B3_ASSERT( !"Should never get here!" );
			break;
	}

	return fcn;
}

/// The deepest separation along the current axis at time `t`, and the witness
/// pair that achieves it.
static b3f b3FindMinSeparation( b3SeparationFunction* fcn, int* indexA, int* indexB, b3c t )
{
	b3Transform xfA = b3GetSweepTransform( &fcn->sweepA, t );
	b3Transform xfB = b3GetSweepTransform( &fcn->sweepB, t );

	switch ( fcn->type )
	{
		case b3_separationVertices:
		{
			b3Vec3 axis = fcn->witness1;

			b3Vec3 localAxisA = b3InvRotateVector( xfA.q, axis );
			b3Vec3 localAxisB = b3InvRotateVector( xfB.q, b3Neg( axis ) );

			*indexA = b3GetPointSupport( fcn->proxyA->points, fcn->proxyA->count, localAxisA );
			*indexB = b3GetPointSupport( fcn->proxyB->points, fcn->proxyB->count, localAxisB );

			b3Vec3 deltaP = b3Sub( xfB.p, xfA.p );
			b3Vec3 localPointA = fcn->proxyA->points[*indexA];
			b3Vec3 localPointB = fcn->proxyB->points[*indexB];
			b3Vec3 delta = b3Add( b3Sub( b3RotateVector( xfB.q, localPointB ), b3RotateVector( xfA.q, localPointA ) ), deltaP );
			return b3Dot( delta, axis );
		}

		case b3_separationEdges:
		{
			b3Vec3 edgeA = b3RotateVector( xfA.q, fcn->witness1 );
			b3Vec3 edgeB = b3RotateVector( xfB.q, fcn->witness2 );

			// b3MakeSeparationFunction only chose this type after the wide
			// cross cleared B3_EDGE_TOLERANCE_SQ, so the direction exists.
			b3Vec3 axis = b3Normalize( b3CrossDirection( edgeA, edgeB ) );
			B3_ASSERT( b3Raw( axis.x ) != 0 || b3Raw( axis.y ) != 0 || b3Raw( axis.z ) != 0 );

			b3Vec3 axisA = b3InvRotateVector( xfA.q, axis );
			*indexA = b3GetPointSupport( fcn->proxyA->points, fcn->proxyA->count, axisA );

			b3Vec3 axisB = b3InvRotateVector( xfB.q, axis );
			*indexB = b3GetPointSupport( fcn->proxyB->points, fcn->proxyB->count, b3Neg( axisB ) );

			b3Vec3 deltaP = b3Sub( xfB.p, xfA.p );
			b3Vec3 localPointA = fcn->proxyA->points[*indexA];
			b3Vec3 localPointB = fcn->proxyB->points[*indexB];
			b3Vec3 delta = b3Add( b3Sub( b3RotateVector( xfB.q, localPointB ), b3RotateVector( xfA.q, localPointA ) ), deltaP );

			return b3Dot( delta, axis );
		}

		case b3_separationFaceA:
		{
			b3Vec3 normal = b3RotateVector( xfA.q, fcn->witness1 );
			*indexA = -1;
			b3Vec3 pointA = b3TransformPoint( xfA, fcn->witness2 );

			b3Vec3 axisB = b3InvRotateVector( xfB.q, normal );
			*indexB = b3GetPointSupport( fcn->proxyB->points, fcn->proxyB->count, b3Neg( axisB ) );
			b3Vec3 pointB = b3TransformPoint( xfB, fcn->proxyB->points[*indexB] );

			return b3Dot( b3Sub( pointB, pointA ), normal );
		}

		case b3_separationFaceB:
		{
			b3Vec3 normal = b3RotateVector( xfB.q, fcn->witness1 );

			b3Vec3 axisA = b3InvRotateVector( xfA.q, normal );
			*indexA = b3GetPointSupport( fcn->proxyA->points, fcn->proxyA->count, b3Neg( axisA ) );
			b3Vec3 pointA = b3TransformPoint( xfA, fcn->proxyA->points[*indexA] );

			*indexB = -1;
			b3Vec3 pointB = b3TransformPoint( xfB, fcn->witness2 );

			return b3Dot( b3Sub( pointA, pointB ), normal );
		}

		default:
			B3_ASSERT( !"Should never get here!" );
			break;
	}

	return b3f_zero;
}

/// The separation of one *fixed* witness pair at time `beta`.
///
/// This is the function the root finder actually brackets. b3FindMinSeparation
/// re-picks the witnesses and so is not continuous in t; this one holds them
/// still, which is what makes a root meaningful.
static b3f b3EvaluateSeparation( b3SeparationFunction* fcn, int index1, int index2, b3c beta )
{
	b3Transform transform1 = b3GetSweepTransform( &fcn->sweepA, beta );
	b3Transform transform2 = b3GetSweepTransform( &fcn->sweepB, beta );

	switch ( fcn->type )
	{
		case b3_separationVertices:
		{
			b3Vec3 axis = fcn->witness1;

			b3Vec3 point1 = b3TransformPoint( transform1, fcn->proxyA->points[index1] );
			b3Vec3 point2 = b3TransformPoint( transform2, fcn->proxyB->points[index2] );

			return b3Dot( b3Sub( point2, point1 ), axis );
		}

		case b3_separationEdges:
		{
			b3Vec3 edge1 = b3RotateVector( transform1.q, fcn->witness1 );
			b3Vec3 edge2 = b3RotateVector( transform2.q, fcn->witness2 );
			b3Vec3 axis = b3Normalize( b3CrossDirection( edge1, edge2 ) );

			b3Vec3 point1 = b3TransformPoint( transform1, fcn->proxyA->points[index1] );
			b3Vec3 point2 = b3TransformPoint( transform2, fcn->proxyB->points[index2] );

			return b3Dot( b3Sub( point2, point1 ), axis );
		}

		case b3_separationFaceA:
		{
			b3Vec3 axis = b3RotateVector( transform1.q, fcn->witness1 );

			b3Vec3 point1 = b3TransformPoint( transform1, fcn->witness2 );
			b3Vec3 point2 = b3TransformPoint( transform2, fcn->proxyB->points[index2] );

			return b3Dot( b3Sub( point2, point1 ), axis );
		}

		case b3_separationFaceB:
		{
			b3Vec3 axis = b3RotateVector( transform2.q, fcn->witness1 );

			b3Vec3 point1 = b3TransformPoint( transform1, fcn->proxyA->points[index1] );
			b3Vec3 point2 = b3TransformPoint( transform2, fcn->witness2 );

			return b3Dot( b3Sub( point1, point2 ), axis );
		}

		default:
			B3_ASSERT( !"Should never get here!" );
			break;
	}

	return b3f_zero;
}

/// Freeze an edge-edge axis at the pose it has at `beta`.
///
/// The escape hatch for the one case the root finder cannot converge on: an
/// edge-edge axis that keeps moving under it. Evaluating the axis once and
/// holding it turns the function into the vertex form, which is monotone
/// enough to bracket.
static void b3ForceFixedAxis( b3SeparationFunction* fcn, b3c beta )
{
	B3_ASSERT( fcn->type == b3_separationEdges );

	b3Transform transform1 = b3GetSweepTransform( &fcn->sweepA, beta );
	b3Transform transform2 = b3GetSweepTransform( &fcn->sweepB, beta );

	b3Vec3 edge1 = b3RotateVector( transform1.q, fcn->witness1 );
	b3Vec3 edge2 = b3RotateVector( transform2.q, fcn->witness2 );
	b3Vec3 axis = b3Normalize( b3CrossDirection( edge1, edge2 ) );

	fcn->type = b3_separationVertices;
	fcn->witness1 = axis;
	fcn->witness2 = b3Vec3_zero;
}

/// The smallest sweep fraction that still moves a witness point by a Q12
/// quantum.
///
/// This is the port's, and it is what makes the root finder terminate for a
/// stated reason instead of by exhausting its iteration count. Bisection on a
/// Q30 bracket can halve about thirty times before the two endpoints are the
/// same number, but the separations being compared are Q12: long before that,
/// every t in the bracket gives the *same* separation, and each further
/// iteration re-evaluates an unchanged function and re-takes the same branch.
///
/// The bound is the motion of the furthest point of either proxy across the
/// whole sweep, measured as a Chebyshev norm because this is a floor and not a
/// measurement -- underestimating motion only permits a few more iterations,
/// while overestimating it would stop the search early.
///
///   translation   the centre displacement, per component
///   rotation      extent * |dq|, since rotating a quaternion by dq moves a
///                 point at radius `extent` by at most about 2*|dq|*extent
///
/// Returns one raw Q30 unit -- effectively no floor, leaving the iteration cap
/// in charge -- when nothing moves at all.
static b3c b3SweepMotionFloor( const b3ShapeProxy* proxy, const b3Sweep* sweep )
{
	int32_t travel = 0;

	b3Vec3 delta = b3Sub( sweep->c2, sweep->c1 );
	travel = b3MaxInt( travel, b3Raw( b3AbsF( delta.x ) ) );
	travel = b3MaxInt( travel, b3Raw( b3AbsF( delta.y ) ) );
	travel = b3MaxInt( travel, b3Raw( b3AbsF( delta.z ) ) );

	// How far from the centre of rotation the geometry reaches.
	int32_t extent = b3Raw( proxy->radius );
	for ( int i = 0; i < proxy->count; ++i )
	{
		b3Vec3 p = b3Sub( proxy->points[i], sweep->localCenter );
		extent = b3MaxInt( extent, b3Raw( b3AbsF( p.x ) ) + b3Raw( proxy->radius ) );
		extent = b3MaxInt( extent, b3Raw( b3AbsF( p.y ) ) + b3Raw( proxy->radius ) );
		extent = b3MaxInt( extent, b3Raw( b3AbsF( p.z ) ) + b3Raw( proxy->radius ) );
	}

	// |dq| at Q30, taken per component like the translation above. The factor
	// of two that turns a quaternion difference into an arc length is folded
	// in by not halving anywhere.
	int32_t dq = 0;
	dq = b3MaxInt( dq, b3Raw( b3AbsN( b3SubN( sweep->q2.v.x, sweep->q1.v.x ) ) ) );
	dq = b3MaxInt( dq, b3Raw( b3AbsN( b3SubN( sweep->q2.v.y, sweep->q1.v.y ) ) ) );
	dq = b3MaxInt( dq, b3Raw( b3AbsN( b3SubN( sweep->q2.v.z, sweep->q1.v.z ) ) ) );

	// extent (Q12) * dq (Q30) -> Q12, exactly, in int64.
	int64_t arc = ( (int64_t)extent * dq ) >> B3_N_SHIFT;
	int64_t motion = (int64_t)travel + arc;

	if ( motion <= 0 )
	{
		return b3Makeb3c( 1 );
	}

	// One raw Q12 quantum as a fraction of that motion.
	b3c floorStep = b3DivWideToC( 1, motion );
	return b3Raw( floorStep ) > 0 ? floorStep : b3Makeb3c( 1 );
}

b3TOIOutput b3TimeOfImpact( const b3TOIInput* input )
{
	b3TOIOutput output = { 0 };

	// Invalid on purpose, so the exit asserts can tell that a branch set them.
	output.state = b3_toiStateUnknown;
	output.fraction = b3Makeb3c( -1 );

	b3Sweep sweepA = input->sweepA;
	b3Sweep sweepB = input->sweepB;

	// Shift to the origin. Upstream does this for float round-off; in Q12 it
	// buys range as well, and that is the binding constraint here. Afterwards
	// every coordinate the separation function multiplies is an extent plus a
	// sweep length rather than a world position, and those products are
	// quadratic -- the same reason b3ShapeCastMesh works in vertex1-relative
	// coordinates.
	b3Vec3 origin = sweepA.c1;
	sweepA.c1 = b3Vec3_zero;
	sweepA.c2 = b3Sub( sweepA.c2, origin );
	sweepB.c1 = b3Sub( sweepB.c1, origin );
	sweepB.c2 = b3Sub( sweepB.c2, origin );

	b3ShapeProxy proxyA = input->proxyA;
	b3ShapeProxy proxyB = input->proxyB;

	int maxPushBackIterations = proxyA.count + proxyB.count;
	b3c tMax = input->maxFraction;

	// Identical to b3ShapeCast above, and for the same reasons: the target is
	// the touching distance pulled in by a slop, and a quarter of the slop is
	// five raw quanta -- small, but never sub-quantum, which is the property
	// that matters.
	b3f linearSlop = B3_LINEAR_SLOP;
	b3f totalRadius = b3AddF( proxyA.radius, proxyB.radius );
	b3f target = b3MaxF( linearSlop, b3SubF( totalRadius, linearSlop ) );
	b3f tolerance = b3Makeb3f( b3Raw( linearSlop ) / 4 );
	B3_ASSERT( b3Raw( target ) > b3Raw( tolerance ) );

	// The bracket floor. Taken over both sweeps, since either shape moving is
	// enough to change the separation.
	b3c minStep = b3MinC( b3SweepMotionFloor( &proxyA, &sweepA ), b3SweepMotionFloor( &proxyB, &sweepB ) );

	b3c t1 = b3c_zero;
	const int maxIterations = 25;
	int distanceIterations = 0;

	b3SimplexCache cache = { 0 };
	b3DistanceInput distanceInput = { 0 };
	distanceInput.proxyA = proxyA;
	distanceInput.proxyB = proxyB;
	distanceInput.useRadii = false;

	// The outer loop derives a new separating axis each time it advances t1,
	// and stops when it stops making progress.
	for ( ;; )
	{
		b3Transform xfA = b3GetSweepTransform( &sweepA, t1 );
		b3Transform xfB = b3GetSweepTransform( &sweepB, t1 );
		distanceInput.transform = b3InvMulTransforms( xfA, xfB );
		b3DistanceOutput distanceOutput = b3ShapeDistance( &distanceInput, &cache, NULL, 0 );
		output.distance = distanceOutput.distance;

		// The distance query runs in frame A, so its witness data comes back
		// in frame A and goes to the shifted world through xfA.
		b3Vec3 worldNormal = b3RotateVector( xfA.q, distanceOutput.normal );
		b3Vec3 worldPointA = b3TransformPoint( xfA, distanceOutput.pointA );
		b3Vec3 worldPointB = b3TransformPoint( xfA, distanceOutput.pointB );

		output.distanceIterations += 1;
		distanceIterations += 1;

		if ( b3Raw( distanceOutput.distance ) <= 0 )
		{
			// Already overlapped. Continuous collision has nothing to say.
			output.state = b3_toiStateOverlapped;
			output.fraction = b3c_zero;
			break;
		}

		if ( b3Raw( distanceOutput.distance ) <= b3Raw( b3AddF( target, tolerance ) ) )
		{
			output.state = b3_toiStateHit;

			b3Vec3 pA = b3MulAdd( worldPointA, proxyA.radius, worldNormal );
			b3Vec3 pB = b3MulSub( worldPointB, proxyB.radius, worldNormal );
			output.point = b3Add( b3Lerp( pA, pB, b3cFromFrac( 1, 2 ) ), origin );
			output.normal = worldNormal;
			output.fraction = t1;
			break;
		}

		if ( distanceIterations == maxIterations )
		{
			// Progress too slow -- a capsule rotating around a triangle vertex
			// is the usual cause. t1 is the last time known separated, so
			// stopping there is conservative rather than wrong.
			output.state = b3_toiStateFailed;
			output.fraction = t1;

			b3Vec3 pA = b3MulAdd( worldPointA, proxyA.radius, worldNormal );
			b3Vec3 pB = b3MulSub( worldPointB, proxyB.radius, worldNormal );
			output.point = b3Add( b3Lerp( pA, pB, b3cFromFrac( 1, 2 ) ), origin );
			output.normal = worldNormal;
			break;
		}

		b3SeparationFunction function =
			b3MakeSeparationFunction( cache, &proxyA, &sweepA, &proxyB, &sweepB, worldNormal, t1 );

		// Resolve the deepest point, one witness pair at a time.
		bool done = false;
		b3c t2 = tMax;
		int pushBackIterations = 0;
		for ( ;; )
		{
			// Seeded with the sentinel the face cases use for "this side is
			// the plane, not a point", so the unreachable default in
			// b3FindMinSeparation cannot hand an index to a proxy lookup.
			int indexA = B3_NULL_INDEX;
			int indexB = B3_NULL_INDEX;
			b3f s2 = b3FindMinSeparation( &function, &indexA, &indexB, t2 );

			if ( b3Raw( b3SubF( s2, target ) ) > b3Raw( tolerance ) )
			{
				// Still apart at the end of the sweep.
				output.state = b3_toiStateSeparated;
				output.fraction = input->maxFraction;
				done = true;
				break;
			}

			if ( b3Raw( s2 ) >= b3Raw( b3SubF( target, tolerance ) ) )
			{
				// Close enough at t2: advance and re-derive the axis.
				t1 = t2;
				break;
			}

			b3f s1 = b3EvaluateSeparation( &function, indexA, indexB, t1 );

			if ( b3Raw( s1 ) < b3Raw( b3SubF( target, tolerance ) ) )
			{
				// The bracket is already violated at t1. Only reachable if the
				// root finder below ran out of iterations on a previous pass.
				output.state = b3_toiStateFailed;
				output.fraction = t1;
				done = true;
				break;
			}

			if ( b3Raw( s1 ) <= b3Raw( b3AddF( target, tolerance ) ) )
			{
				// t1 is the time of impact, and it may well be zero.
				output.state = b3_toiStateHit;
				output.fraction = t1;
				done = true;
				break;
			}

			// Root of f(t) = separation(t) - target, on [t1, t2].
			int rootIterationCount = 0;
			const int maxRootIterations = 50;
			b3c a1 = t1;
			b3c a2 = t2;
			for ( ;; )
			{
				b3c t;
				if ( rootIterationCount & 1 )
				{
					// False position, for convergence. The quotient is a ratio
					// of two Q12 lengths, so it is dimensionless and lands in
					// Q30 -- and b3DivWideToC's saturation is unreachable here
					// rather than merely unlikely: the bracket invariant is
					// s1 > target > s2, so numerator and denominator are both
					// negative and |num| <= |den|, putting the quotient in
					// [0, 1] by construction.
					b3c fraction = b3DivWideToC( b3Raw( b3SubF( target, s1 ) ), b3Raw( b3SubF( s2, s1 ) ) );
					t = b3AddC( a1, b3MulCC( b3SubC( a2, a1 ), fraction ) );
				}
				else
				{
					// Bisection, for guaranteed progress. Exact in Q30: a
					// halving is a shift, and it rounds nothing.
					t = b3Makeb3c( ( b3Raw( a1 ) + b3Raw( a2 ) ) >> 1 );
				}

				output.rootIterations += 1;
				rootIterationCount += 1;

				b3f s = b3EvaluateSeparation( &function, indexA, indexB, t );

				if ( b3Raw( b3AbsF( b3SubF( s, target ) ) ) <= b3Raw( tolerance ) )
				{
					// t2 carries a tentative t1 back out.
					t2 = t;
					break;
				}

				// Keep bracketing the root.
				if ( b3Raw( s ) > b3Raw( target ) )
				{
					a1 = t;
					s1 = s;
				}
				else
				{
					a2 = t;
					s2 = s;
				}

				if ( b3Raw( b3SubC( a2, a1 ) ) <= b3Raw( minStep ) )
				{
					// The bracket is below the point where the Q12 separation
					// can tell its two ends apart, so there is nothing left to
					// find. Take a2 -- the end that is *not* known separated --
					// which stops the body at or before the true impact.
					t2 = a2;
					break;
				}

				if ( rootIterationCount == maxRootIterations )
				{
					break;
				}
			}

			// One failing edge case gets a second chance on a frozen axis.
			if ( rootIterationCount == maxRootIterations - 1 && function.type == b3_separationEdges )
			{
				rootIterationCount = 0;
				t2 = input->maxFraction;
				b3ForceFixedAxis( &function, t1 );
				B3_ASSERT( function.type != b3_separationEdges );
			}

			output.pushBackIterations += 1;
			pushBackIterations += 1;

			if ( pushBackIterations == maxPushBackIterations )
			{
				break;
			}
		}

		if ( done )
		{
			b3Vec3 pA = b3MulAdd( worldPointA, proxyA.radius, worldNormal );
			b3Vec3 pB = b3MulSub( worldPointB, proxyB.radius, worldNormal );
			output.point = b3Add( b3Lerp( pA, pB, b3cFromFrac( 1, 2 ) ), origin );
			output.normal = worldNormal;
			break;
		}
	}

	// Every exit above sets both.
	B3_ASSERT( output.state != b3_toiStateUnknown );
	B3_ASSERT( b3Raw( output.fraction ) >= 0 );

	return output;
}
