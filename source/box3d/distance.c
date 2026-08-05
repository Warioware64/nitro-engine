// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Erin Catto        (original Box3D)
// Copyright (c) 2026 Warioware64       (Nitro Engine Advanced fixed-point port)
//
// Dirk Gregorius contributed portions of this code
//
// This file is part of Nitro Engine Advanced

/// @file   distance.c
/// @brief  GJK closest points and shape casting.
///
/// NOTE: this file is being converted in two passes. Present so far: the
/// support functions, barycentric coordinates, the simplex metric, and the
/// 2- and 3-vertex simplex solvers. Still to come in Phase 2A:
/// b3SolveSimplex4, b3ComputeWitnessPoints, b3ShapeDistance and b3ShapeCast.
/// b3TimeOfImpact and its separation function belong to Phase 7 (continuous
/// collision) and stay in upstream until then.
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
			b3Vec3 a = vertices[0].w;
			b3Vec3 b = vertices[1].w;
			b3Vec3 c = vertices[2].w;
			b3Vec3 d = vertices[3].w;
			return b3ScalarTripleProductWide( b3Sub( b, a ), b3Sub( c, a ), b3Sub( d, a ) ) / 6;
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
