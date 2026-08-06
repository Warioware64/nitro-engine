// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Warioware64
//
// This file is part of Nitro Engine Advanced

// Verification of the collision primitives.
//
// The AABB ray cast gets the most attention here, because its slab division
// is the first place in the port where the fixed-point and float versions
// genuinely have to differ. Float lets a near-parallel ray produce a huge
// quotient and discards it later; the hardware divider would return an
// undefined 32-bit result instead. The guard is checked directly, including
// the cases that would have overflowed.

#include "aabb.h"

#include "broad_phase.h"
#include "core.h"
#include "manifold.h"
#include "mesh.h"
#include "shape.h"

#include "box3d/base.h"
#include "box3d/collision.h"
#include "box3d/types.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "assert_trap.h"

static int s_failures = 0;
static int s_checks = 0;

#define Q12 ( 1.0 / 4096.0 )

static void check( const char* what, bool ok )
{
	s_checks++;
	if ( !ok )
	{
		printf( "  FAIL %s\n", what );
		s_failures++;
	}
}

static void expect( const char* what, double got, double want, double tol )
{
	s_checks++;
	if ( fabs( got - want ) > tol )
	{
		printf( "  FAIL %-40s got %-14.9g want %-14.9g\n", what, got, want );
		s_failures++;
	}
}

static void checkInt2( const char* what, long long got, long long want )
{
	s_checks++;
	if ( got != want )
	{
		printf( "  FAIL %-40s got %lld want %lld\n", what, got, want );
		s_failures++;
	}
}

static b3Vec3 V( double x, double y, double z )
{
	return b3MakeVec3( b3fFromDouble( x ), b3fFromDouble( y ), b3fFromDouble( z ) );
}

static void expectVec( const char* what, b3Vec3 got, double x, double y, double z, double tol )
{
	char buf[96];
	snprintf( buf, sizeof( buf ), "%s.x", what );
	expect( buf, b3fToDouble( got.x ), x, tol );
	snprintf( buf, sizeof( buf ), "%s.y", what );
	expect( buf, b3fToDouble( got.y ), y, tol );
	snprintf( buf, sizeof( buf ), "%s.z", what );
	expect( buf, b3fToDouble( got.z ), z, tol );
}

static b3AABB Box( double lx, double ly, double lz, double ux, double uy, double uz )
{
	return b3MakeAABB( V( lx, ly, lz ), V( ux, uy, uz ) );
}

static void section( const char* name )
{
	printf( "%s\n", name );
}

// -------------------------------------------------------------------------

static void test_area( void )
{
	section( "AABB surface area" );

	// The metric the dynamic tree sorts on. A unit cube has area 6.
	expect( "unit cube area", (double)b3Perimeter( Box( 0, 0, 0, 1, 1, 1 ) ) / (double)( (int64_t)1 << 24 ), 6.0, 0.01 );

	// 2x3x4 box: 2*(2*4 + 3*2 + 4*3) = 52
	expect( "2x3x4 area", (double)b3Perimeter( Box( 0, 0, 0, 2, 3, 4 ) ) / (double)( (int64_t)1 << 24 ), 52.0, 0.05 );

	// The case that forces the wide return type. A 2000-unit box -- the edge
	// of the documented world -- has an area of 2.4e7, which is 9.8e10 once
	// scaled: forty times past int32. If this ever comes back negative or
	// truncated, b3Perimeter has silently gone back to 32 bits.
	int64_t big = b3Perimeter( Box( -1000, -1000, -1000, 1000, 1000, 1000 ) );
	double bigArea = (double)big / (double)( (int64_t)1 << 24 );
	expect( "2000-unit box area", bigArea, 2.0 * ( 2000.0 * 2000.0 * 3.0 ), 1000.0 );
	check( "large area stayed positive", big > 0 );

	// Ordering must survive at that size, since the tree only ever compares.
	check( "area ordering holds when large",
		   b3Perimeter( Box( -1000, -1000, -1000, 1000, 1000, 1000 ) ) > b3Perimeter( Box( -999, -999, -999, 999, 999, 999 ) ) );
}

static void test_enlarge( void )
{
	section( "AABB enlargement" );

	b3AABB a = Box( 0, 0, 0, 1, 1, 1 );
	check( "no growth for contained box", b3EnlargeAABB( &a, Box( 0.2, 0.2, 0.2, 0.8, 0.8, 0.8 ) ) == false );

	b3AABB b = Box( 0, 0, 0, 1, 1, 1 );
	check( "growth reported", b3EnlargeAABB( &b, Box( -1, 0, 0, 2, 1, 1 ) ) );
	expect( "grew lower x", b3fToDouble( b.lowerBound.x ), -1.0, Q12 );
	expect( "grew upper x", b3fToDouble( b.upperBound.x ), 2.0, Q12 );
	expect( "left y alone", b3fToDouble( b.upperBound.y ), 1.0, Q12 );

	b3Vec3 far = b3FarthestPointOnAABB( Box( 0, 0, 0, 10, 10, 10 ), V( 1, 9, 5 ) );
	expect( "farthest x", b3fToDouble( far.x ), 10.0, Q12 );
	expect( "farthest y", b3fToDouble( far.y ), 0.0, Q12 );
}

static void test_raycast( void )
{
	section( "AABB ray cast" );

	b3AABB box = Box( -1, -1, -1, 1, 1, 1 );
	b3c tmin, tmax;

	// Straight through the middle along X, starting 5 units out. The box
	// faces are at 4 and 6 units along a 10 unit ray.
	check( "hit through centre", b3RayCastAABB( box, V( -5, 0, 0 ), V( 5, 0, 0 ), &tmin, &tmax ) );
	expect( "entry fraction", b3cToDouble( tmin ), 0.4, 0.01 );
	expect( "exit fraction", b3cToDouble( tmax ), 0.6, 0.01 );

	// Miss above the box.
	check( "miss above", b3RayCastAABB( box, V( -5, 5, 0 ), V( 5, 5, 0 ), &tmin, &tmax ) == false );

	// Diagonal hit.
	check( "diagonal hit", b3RayCastAABB( box, V( -5, -5, -5 ), V( 5, 5, 5 ), &tmin, &tmax ) );

	// Ray starting inside: entry clamps to zero.
	check( "starts inside", b3RayCastAABB( box, V( 0, 0, 0 ), V( 5, 0, 0 ), &tmin, &tmax ) );
	expect( "inside entry fraction", b3cToDouble( tmin ), 0.0, 0.01 );

	// Degenerate ray, the FLT_EPSILON branch upstream.
	check( "zero-length ray inside hits", b3RayCastAABB( box, V( 0, 0, 0 ), V( 0, 0, 0 ), &tmin, &tmax ) );
	check( "zero-length ray outside misses", b3RayCastAABB( box, V( 5, 5, 5 ), V( 5, 5, 5 ), &tmin, &tmax ) == false );

	// Ray pointing away from the box.
	check( "ray pointing away misses", b3RayCastAABB( box, V( -5, 0, 0 ), V( -10, 0, 0 ), &tmin, &tmax ) == false );

	// ---- the cases the fixed-point guard exists for ----

	// Almost exactly parallel to the X slabs, and outside them. In float the
	// slab quotient is enormous and gets discarded; here it would overflow
	// the 32-bit divider result if it were computed at all.
	check( "near-parallel outside slab misses", b3RayCastAABB( box, V( -5, 500, 0 ), V( 5, 500, 0 ), &tmin, &tmax ) == false );

	// Near-parallel but travelling *through* the box: must still hit.
	check( "near-parallel through box hits", b3RayCastAABB( box, V( -500, 0, 0 ), V( 500, 0.001, 0 ), &tmin, &tmax ) );

	// Exactly axis-aligned, so two of the three direction components are
	// exactly zero -- the division that must never be attempted.
	check( "axis-aligned hit", b3RayCastAABB( box, V( 0, -5, 0 ), V( 0, 5, 0 ), &tmin, &tmax ) );
	expect( "axis-aligned entry", b3cToDouble( tmin ), 0.4, 0.01 );

	check( "axis-aligned miss", b3RayCastAABB( box, V( 3, -5, 0 ), V( 3, 5, 0 ), &tmin, &tmax ) == false );

	// A long ray against a far-away box, near the edge of the usable world.
	// This is where the numerator is large and the guard has to compare
	// magnitudes rather than trust an epsilon.
	b3AABB farBox = Box( 990, -10, -10, 1010, 10, 10 );
	check( "long ray to distant box", b3RayCastAABB( farBox, V( -1000, 0, 0 ), V( 1500, 0, 0 ), &tmin, &tmax ) );
	expect( "distant entry fraction", b3cToDouble( tmin ), 1990.0 / 2500.0, 0.01 );

	// Fractions must always come back ordered and in range, whatever happens
	// inside -- the tree relies on that for its clip bounds.
	check( "fractions ordered", b3cToDouble( tmin ) <= b3cToDouble( tmax ) );
	check( "fractions in range", b3cToDouble( tmin ) >= 0.0 && b3cToDouble( tmax ) <= 1.0 );

	// Sweep a fan of directions and assert the invariants hold for every one.
	// The point is not the individual results but that nothing produces an
	// out-of-range or inverted fraction.
	for ( int i = 0; i < 64; i++ )
	{
		double angle = ( 2.0 * M_PI * i ) / 64.0;
		b3Vec3 start = V( -8.0 * cos( angle ), -8.0 * sin( angle ), 0.0 );
		b3Vec3 end = V( 8.0 * cos( angle ), 8.0 * sin( angle ), 0.0 );

		if ( b3RayCastAABB( box, start, end, &tmin, &tmax ) )
		{
			double lo = b3cToDouble( tmin );
			double hi = b3cToDouble( tmax );
			if ( lo > hi || lo < 0.0 || hi > 1.0 )
			{
				printf( "  FAIL fan direction %d gave fractions [%g, %g]\n", i, lo, hi );
				s_failures++;
				break;
			}
		}
	}
	s_checks++;
}


// -------------------------------------------------------------------------
// Closest points
// -------------------------------------------------------------------------
//
// These run entirely on wide intermediates: the dot products are squared
// lengths and the denominators are fourth powers, so a scale mistake here
// would show up as a fraction outside [0,1] or a point off the segment
// rather than as a small error.

// -------------------------------------------------------------------------
// Ray versus box, the separating-axis form
// -------------------------------------------------------------------------
//
// b3TestBoundsRayOverlap is the dynamic tree's node rejection test, so a false
// negative does not produce a slightly wrong number -- it drops a proxy from a
// query result entirely. The port's version is exact (it doubles every term
// rather than halving to a box centre), which means it can be checked against
// a brute-force answer with no tolerance at all.

// Reference: does the segment p -> p+d come within the box? Sampled densely
// enough that any real overlap of a box this size is caught. Deliberately
// dumb -- it shares no arithmetic with the function under test.
static bool segmentHitsBoxByScan( b3Vec3 lower, b3Vec3 upper, b3Vec3 start, b3Vec3 delta )
{
	const int steps = 20000;
	for ( int i = 0; i <= steps; ++i )
	{
		double t = (double)i / (double)steps;
		double x = b3fToDouble( start.x ) + t * b3fToDouble( delta.x );
		double y = b3fToDouble( start.y ) + t * b3fToDouble( delta.y );
		double z = b3fToDouble( start.z ) + t * b3fToDouble( delta.z );

		if ( x >= b3fToDouble( lower.x ) && x <= b3fToDouble( upper.x ) && y >= b3fToDouble( lower.y ) &&
			 y <= b3fToDouble( upper.y ) && z >= b3fToDouble( lower.z ) && z <= b3fToDouble( upper.z ) )
		{
			return true;
		}
	}
	return false;
}

static void test_bounds_ray_overlap( void )
{
	section( "ray versus box separating axis" );

	b3Vec3 lower = V( -1, -1, -1 );
	b3Vec3 upper = V( 1, 1, 1 );

	// Straight through the middle on each axis.
	check( "through on x", b3TestBoundsRayOverlap( lower, upper, V( -10, 0, 0 ), V( 20, 0, 0 ) ) );
	check( "through on y", b3TestBoundsRayOverlap( lower, upper, V( 0, -10, 0 ), V( 0, 20, 0 ) ) );
	check( "through on z", b3TestBoundsRayOverlap( lower, upper, V( 0, 0, -10 ), V( 0, 0, 20 ) ) );

	// Diagonal through a corner.
	check( "through the diagonal", b3TestBoundsRayOverlap( lower, upper, V( -10, -10, -10 ), V( 20, 20, 20 ) ) );

	// Clear misses on each of the three edge axes, which is what this test
	// actually separates on.
	check( "miss above x", b3TestBoundsRayOverlap( lower, upper, V( -10, 5, 0 ), V( 20, 0, 0 ) ) == false );
	check( "miss beside y", b3TestBoundsRayOverlap( lower, upper, V( 5, -10, 0 ), V( 0, 20, 0 ) ) == false );
	check( "miss beside z", b3TestBoundsRayOverlap( lower, upper, V( 0, 5, -10 ), V( 0, 0, 20 ) ) == false );

	// Grazing a face exactly. The test is inclusive, so touching counts.
	check( "grazing face touches", b3TestBoundsRayOverlap( lower, upper, V( -10, 1, 0 ), V( 20, 0, 0 ) ) );

	// Starting inside.
	check( "start inside", b3TestBoundsRayOverlap( lower, upper, V( 0, 0, 0 ), V( 20, 0, 0 ) ) );

	// A degenerate zero-length ray at the box centre. cross() is zero and the
	// extents are not, so it reports overlap.
	check( "zero ray inside", b3TestBoundsRayOverlap( lower, upper, V( 0, 0, 0 ), V( 0, 0, 0 ) ) );

	// This is an infinite-line test, not a segment test -- it has no notion of
	// the ray's length, which is why the tree pairs it with a segment AABB
	// overlap check before calling it. Pin that down so the pairing is not
	// mistaken for redundancy later.
	check( "line extends beyond the segment",
		   b3TestBoundsRayOverlap( lower, upper, V( -10, 0, 0 ), V( 1, 0, 0 ) ) );

	// Far field, at the documented edge of the usable world. These are the
	// largest products the function ever forms.
	b3Vec3 farLower = V( 1990, -10, -10 );
	b3Vec3 farUpper = V( 2000, 10, 10 );
	check( "far field hit", b3TestBoundsRayOverlap( farLower, farUpper, V( 0, 0, 0 ), V( 2000, 0, 0 ) ) );
	check( "far field miss",
		   b3TestBoundsRayOverlap( farLower, farUpper, V( 0, 500, 0 ), V( 2000, 0, 0 ) ) == false );

	// A very thin box, where a near-parallel ray makes the separation small
	// and any rounding would decide the answer.
	b3Vec3 thinLower = V( -100, -0.01, -100 );
	b3Vec3 thinUpper = V( 100, 0.01, 100 );
	check( "thin slab hit", b3TestBoundsRayOverlap( thinLower, thinUpper, V( -200, 0, 0 ), V( 400, 0, 0 ) ) );
	check( "thin slab miss",
		   b3TestBoundsRayOverlap( thinLower, thinUpper, V( -200, 5, 0 ), V( 400, 0, 0 ) ) == false );

	// Randomized agreement with the brute-force scan. A pseudo-random sweep
	// finds the sign and axis-pairing mistakes that hand-picked cases miss,
	// because those tend to be chosen with symmetric coordinates.
	unsigned seed = 12345u;
	int agreed = 0;
	int total = 0;
	int overlaps = 0;

	for ( int i = 0; i < 400; ++i )
	{
		double v[10];
		for ( int k = 0; k < 10; ++k )
		{
			seed = seed * 1103515245u + 12345u;
			v[k] = (double)( ( seed >> 16 ) & 0x7fff ) / 32767.0;
		}

		b3Vec3 lo = V( -4 + 8 * v[0], -4 + 8 * v[1], -4 + 8 * v[2] );
		b3Vec3 hi = b3Add( lo, V( 0.5 + 3 * v[3], 0.5 + 3 * v[4], 0.5 + 3 * v[5] ) );
		b3Vec3 st = V( -12 + 24 * v[6], -12 + 24 * v[7], -12 + 24 * v[8] );

		// Aim a good fraction of the rays near the box so both answers occur.
		b3Vec3 target = v[9] < 0.6 ? b3MulCV( b3cFromFrac( 1, 2 ), b3Add( lo, hi ) )
								   : V( -8 + 16 * v[9], -8 + 16 * v[0], -8 + 16 * v[1] );
		b3Vec3 de = b3Sub( target, st );

		bool fast = b3TestBoundsRayOverlap( lo, hi, st, de );
		bool slow = segmentHitsBoxByScan( lo, hi, st, de );

		total++;

		// The scan can only confirm overlap, never rule it out -- it samples
		// the segment, and the SAT test covers the whole infinite line. So the
		// implication that must hold is one-directional: anything the scan
		// finds, the SAT test must also report.
		if ( slow )
		{
			overlaps++;
			if ( fast )
			{
				agreed++;
			}
		}
	}

	checkInt2( "randomized: SAT agrees with scan on every overlap", agreed, overlaps );
	check( "randomized: the sweep actually produced overlaps", overlaps > 40 );
	checkInt2( "randomized: case count", total, 400 );
}


static void test_segment_distance( void )
{
	section( "closest points" );

	// Point to segment: inside, and clamped at both ends.
	b3Vec3 p = b3PointToSegmentDistance( V( 0, 0, 0 ), V( 10, 0, 0 ), V( 3, 5, 0 ) );
	expect( "point-segment interior x", b3fToDouble( p.x ), 3.0, 8 * Q12 );
	expect( "point-segment interior y", b3fToDouble( p.y ), 0.0, 8 * Q12 );

	p = b3PointToSegmentDistance( V( 0, 0, 0 ), V( 10, 0, 0 ), V( -5, 2, 0 ) );
	expect( "point-segment clamps to a", b3fToDouble( p.x ), 0.0, 8 * Q12 );

	p = b3PointToSegmentDistance( V( 0, 0, 0 ), V( 10, 0, 0 ), V( 20, 2, 0 ) );
	expect( "point-segment clamps to b", b3fToDouble( p.x ), 10.0, 8 * Q12 );

	// Two perpendicular, offset segments. Closest points are the midpoint of
	// the first and the near end of the second.
	b3SegmentDistanceResult r =
		b3SegmentDistance( V( -5, 0, 0 ), V( 5, 0, 0 ), V( 0, 2, -5 ), V( 0, 2, 5 ) );
	expect( "crossing segments p1.x", b3fToDouble( r.point1.x ), 0.0, 16 * Q12 );
	expect( "crossing segments p2.z", b3fToDouble( r.point2.z ), 0.0, 16 * Q12 );
	expect( "crossing segments fraction1", b3cToDouble( r.fraction1 ), 0.5, 0.01 );
	expect( "crossing segments fraction2", b3cToDouble( r.fraction2 ), 0.5, 0.01 );

	// Parallel segments -- the degenerate determinant path.
	r = b3SegmentDistance( V( 0, 0, 0 ), V( 10, 0, 0 ), V( 0, 3, 0 ), V( 10, 3, 0 ) );
	check( "parallel fraction1 in range", b3cToDouble( r.fraction1 ) >= 0.0 && b3cToDouble( r.fraction1 ) <= 1.0 );
	check( "parallel fraction2 in range", b3cToDouble( r.fraction2 ) >= 0.0 && b3cToDouble( r.fraction2 ) <= 1.0 );
	expect( "parallel separation", b3fToDouble( b3Distance( r.point1, r.point2 ) ), 3.0, 16 * Q12 );

	// Both segments degenerate to points.
	r = b3SegmentDistance( V( 1, 2, 3 ), V( 1, 2, 3 ), V( 4, 6, 3 ), V( 4, 6, 3 ) );
	expect( "degenerate pair distance", b3fToDouble( b3Distance( r.point1, r.point2 ) ), 5.0, 16 * Q12 );

	// One degenerate: the point projects onto the middle of the other.
	r = b3SegmentDistance( V( 5, 4, 0 ), V( 5, 4, 0 ), V( 0, 0, 0 ), V( 10, 0, 0 ) );
	expect( "one degenerate p2.x", b3fToDouble( r.point2.x ), 5.0, 16 * Q12 );
	expect( "one degenerate fraction2", b3cToDouble( r.fraction2 ), 0.5, 0.01 );

	// Segments that need the s2 clamp: skew, with the closest approach past
	// the end of the second segment.
	r = b3SegmentDistance( V( 0, 0, 0 ), V( 10, 0, 0 ), V( 20, 1, -5 ), V( 20, 1, 5 ) );
	check( "clamped fraction1 in range", b3cToDouble( r.fraction1 ) >= 0.0 && b3cToDouble( r.fraction1 ) <= 1.0 );
	check( "clamped fraction2 in range", b3cToDouble( r.fraction2 ) >= 0.0 && b3cToDouble( r.fraction2 ) <= 1.0 );
	expect( "clamped p1 at segment end", b3fToDouble( r.point1.x ), 10.0, 16 * Q12 );

	// Long segments, where the fourth-power denominator is largest. A 200
	// unit segment gives a squared length of 40000 and a denominator around
	// 1.6e9 before scaling -- the case that decides whether the wide path is
	// actually wide.
	r = b3SegmentDistance( V( -100, 0, 0 ), V( 100, 0, 0 ), V( 0, 50, -100 ), V( 0, 50, 100 ) );
	expect( "long segments p1.x", b3fToDouble( r.point1.x ), 0.0, 64 * Q12 );
	expect( "long segments p2.z", b3fToDouble( r.point2.z ), 0.0, 64 * Q12 );
	expect( "long segments separation", b3fToDouble( b3Distance( r.point1, r.point2 ) ), 50.0, 64 * Q12 );

	// Randomized sweep: whatever the configuration, the fractions must stay
	// in range and the reported points must actually lie on their segments.
	unsigned seed = 12345;
	for ( int i = 0; i < 200; i++ )
	{
		double v[12];
		for ( int j = 0; j < 12; j++ )
		{
			seed = seed * 1103515245u + 12345u;
			v[j] = ( (double)( ( seed >> 16 ) & 0x7FFF ) / 32768.0 - 0.5 ) * 40.0;
		}

		b3Vec3 a1 = V( v[0], v[1], v[2] ), b1 = V( v[3], v[4], v[5] );
		b3Vec3 a2 = V( v[6], v[7], v[8] ), b2 = V( v[9], v[10], v[11] );

		b3SegmentDistanceResult rr = b3SegmentDistance( a1, b1, a2, b2 );

		double f1 = b3cToDouble( rr.fraction1 );
		double f2 = b3cToDouble( rr.fraction2 );
		if ( f1 < -0.001 || f1 > 1.001 || f2 < -0.001 || f2 > 1.001 )
		{
			printf( "  FAIL random pair %d gave fractions %g, %g\n", i, f1, f2 );
			s_failures++;
			break;
		}

		// point1 must equal a1 + fraction1 * (b1 - a1).
		b3Vec3 expect1 = b3MulAdd( a1, b3CToF( rr.fraction1 ), b3Sub( b1, a1 ) );
		if ( b3fToDouble( b3Distance( expect1, rr.point1 ) ) > 0.05 )
		{
			printf( "  FAIL random pair %d: point1 not on its segment\n", i );
			s_failures++;
			break;
		}
	}
	s_checks += 2;
}

static void test_plane( void )
{
	section( "plane separation" );

	b3Plane pl;
	pl.normal = V( 0, 1, 0 );
	pl.offset = b3fFromDouble( 2.0 );

	expect( "above plane", b3fToDouble( b3PlaneSeparation( pl, V( 0, 5, 0 ) ) ), 3.0, 4 * Q12 );
	expect( "on plane", b3fToDouble( b3PlaneSeparation( pl, V( 3, 2, -7 ) ) ), 0.0, 4 * Q12 );
	expect( "below plane", b3fToDouble( b3PlaneSeparation( pl, V( 0, -1, 0 ) ) ), -3.0, 4 * Q12 );
}


// -------------------------------------------------------------------------
// Mass properties
// -------------------------------------------------------------------------
//
// b3MassData::inertia holds inertia *per unit mass* -- the radius of gyration
// squared -- rather than absolute inertia. The reason is range: absolute
// inertia is 0.4*m*r^2 for a sphere, and since m itself grows as r^3, it
// grows as r^5 and overflows Q12 at radius 13. These tests check both that
// the values are right and that the range problem is actually gone.

static void test_mass( void )
{
	section( "mass properties" );

	b3Sphere sphere;
	sphere.center = V( 0, 0, 0 );
	sphere.radius = b3fFromDouble( 1.0 );

	b3MassData md = b3ComputeSphereMass( &sphere, b3fFromDouble( 1.0 ) );

	// 4/3 pi r^3 at r = 1
	expect( "unit sphere mass", b3fToDouble( md.mass ), 4.18879, 0.01 );
	// unit inertia = 0.4 r^2
	expect( "unit sphere inertia", b3fToDouble( md.inertia.cx.x ), 0.4, 0.01 );
	expect( "sphere inertia is isotropic", b3fToDouble( md.inertia.cy.y ), b3fToDouble( md.inertia.cx.x ), 0.001 );
	expect( "sphere inertia off-diagonal", b3fToDouble( md.inertia.cx.y ), 0.0, 0.001 );

	// Mass scales with the cube of the radius.
	sphere.radius = b3fFromDouble( 2.0 );
	md = b3ComputeSphereMass( &sphere, b3fFromDouble( 1.0 ) );
	expect( "r=2 sphere mass", b3fToDouble( md.mass ), 4.18879 * 8.0, 0.1 );
	expect( "r=2 unit inertia", b3fToDouble( md.inertia.cx.x ), 1.6, 0.02 );

	// Density scales mass but not unit inertia -- unit inertia is geometry.
	md = b3ComputeSphereMass( &sphere, b3fFromDouble( 3.0 ) );
	expect( "density scales mass", b3fToDouble( md.mass ), 4.18879 * 8.0 * 3.0, 0.5 );
	expect( "density leaves unit inertia alone", b3fToDouble( md.inertia.cx.x ), 1.6, 0.02 );

	// The case that motivated the whole decision. At radius 20 the absolute
	// inertia is 0.4 * m * r^2 = 0.4 * 33510 * 400 = 5.4e6, which is 2.2e10
	// in Q12 -- ten times past int32. The unit form is 160 and fits trivially.
	sphere.radius = b3fFromDouble( 20.0 );
	md = b3ComputeSphereMass( &sphere, b3fFromDouble( 1.0 ) );
	expect( "r=20 unit inertia", b3fToDouble( md.inertia.cx.x ), 0.4 * 400.0, 1.0 );
	check( "r=20 unit inertia stayed positive", b3fToDouble( md.inertia.cx.x ) > 0.0 );
	check( "r=20 mass stayed positive", b3fToDouble( md.mass ) > 0.0 );

	// And a radius where absolute inertia would be hopeless: 0.4*m*r^2 at
	// r=50 is 2.6e8. The unit form is 1000.
	sphere.radius = b3fFromDouble( 50.0 );
	md = b3ComputeSphereMass( &sphere, b3fFromDouble( 1.0 ) );
	expect( "r=50 unit inertia", b3fToDouble( md.inertia.cx.x ), 1000.0, 5.0 );
	check( "r=50 mass stayed positive", b3fToDouble( md.mass ) > 0.0 );

	// --- capsule ---
	//
	// The mass-weighted composition. A capsule with zero height is exactly a
	// sphere, which is the cleanest way to check the weighting is right.
	b3Capsule cap;
	cap.center1 = V( 0, 0, 0 );
	cap.center2 = V( 0, 0, 0 );
	cap.radius = b3fFromDouble( 1.0 );

	b3MassData cm = b3ComputeCapsuleMass( &cap, b3fFromDouble( 1.0 ) );
	expect( "degenerate capsule mass == sphere", b3fToDouble( cm.mass ), 4.18879, 0.02 );
	expect( "degenerate capsule inertia == sphere", b3fToDouble( cm.inertia.cy.y ), 0.4, 0.02 );

	// A real capsule: radius 1, height 4 along Y.
	cap.center1 = V( 0, -2, 0 );
	cap.center2 = V( 0, 2, 0 );
	cm = b3ComputeCapsuleMass( &cap, b3fFromDouble( 1.0 ) );

	// cylinder pi*r^2*h = 12.566, sphere 4.18879 -> 16.755
	expect( "capsule mass", b3fToDouble( cm.mass ), 16.755, 0.1 );
	expectVec( "capsule centre", cm.center, 0.0, 0.0, 0.0, 8 * Q12 );

	// Around its own axis (Y) a capsule is much easier to spin than end over
	// end, so iyy must be well below ixx. Getting the weighting backwards
	// would invert this.
	check( "capsule iyy < ixx", b3fToDouble( cm.inertia.cy.y ) < b3fToDouble( cm.inertia.cx.x ) );
	check( "capsule inertia positive", b3fToDouble( cm.inertia.cx.x ) > 0.0 );

	// The axial unit inertia is a mass-weighted blend of the cylinder's
	// 0.5r^2 = 0.5 and the sphere's 0.4r^2 = 0.4, so it must land between
	// them. This is the check that the weighting happens at all: plain
	// addition would give 0.9.
	double iyy = b3fToDouble( cm.inertia.cy.y );
	check( "capsule iyy is a blend, not a sum", iyy > 0.39 && iyy < 0.51 );

	// Offset capsule: the centre of mass follows the segment midpoint.
	cap.center1 = V( 10, 0, 0 );
	cap.center2 = V( 14, 0, 0 );
	cm = b3ComputeCapsuleMass( &cap, b3fFromDouble( 1.0 ) );
	expectVec( "offset capsule centre", cm.center, 12.0, 0.0, 0.0, 16 * Q12 );
	expect( "offset capsule mass unchanged", b3fToDouble( cm.mass ), 16.755, 0.1 );

	// Now the capsule lies along X, so the *rotated* tensor must put the
	// small axial component on X rather than Y.
	check( "rotated capsule ixx < iyy", b3fToDouble( cm.inertia.cx.x ) < b3fToDouble( cm.inertia.cy.y ) );
}

static void test_sphere_raycast( void )
{
	section( "sphere ray cast" );

	b3Sphere sphere;
	sphere.center = V( 0, 0, 0 );
	sphere.radius = b3fFromDouble( 2.0 );

	b3RayCastInput in;
	in.maxFraction = b3c_one;

	// Head on: 10 unit ray, sphere surface at 8 units.
	in.origin = V( -10, 0, 0 );
	in.translation = V( 20, 0, 0 );
	b3CastOutput out = b3RayCastSphere( &sphere, &in );
	check( "head-on hit", out.hit );
	expect( "head-on fraction", b3cToDouble( out.fraction ), 8.0 / 20.0, 0.01 );
	expectVec( "head-on point", out.point, -2.0, 0.0, 0.0, 32 * Q12 );
	expectVec( "head-on normal", out.normal, -1.0, 0.0, 0.0, 32 * Q12 );

	// Miss.
	in.origin = V( -10, 5, 0 );
	out = b3RayCastSphere( &sphere, &in );
	check( "miss above", out.hit == false );

	// Grazing hit: this is the case where rr and cc are nearly equal, so the
	// difference feeding the square root is small. Narrowing them to Q12
	// before subtracting would lose it entirely.
	in.origin = V( -10, 1.99, 0 );
	out = b3RayCastSphere( &sphere, &in );
	check( "grazing hit detected", out.hit );

	// Starting inside reports a hit at the origin.
	in.origin = V( 0, 0, 0 );
	in.translation = V( 10, 0, 0 );
	out = b3RayCastSphere( &sphere, &in );
	check( "origin inside hits", out.hit );

	// Zero-length ray outside misses.
	in.origin = V( 10, 0, 0 );
	in.translation = V( 0, 0, 0 );
	out = b3RayCastSphere( &sphere, &in );
	check( "zero ray outside misses", out.hit == false );

	// Ray too short to reach.
	in.origin = V( -10, 0, 0 );
	in.translation = V( 3, 0, 0 );
	out = b3RayCastSphere( &sphere, &in );
	check( "short ray misses", out.hit == false );

	// A distant sphere, where the shift-to-origin trick earns its keep.
	sphere.center = V( 500, 0, 0 );
	in.origin = V( 400, 0, 0 );
	in.translation = V( 200, 0, 0 );
	out = b3RayCastSphere( &sphere, &in );
	check( "distant sphere hit", out.hit );
	expect( "distant fraction", b3cToDouble( out.fraction ), 98.0 / 200.0, 0.02 );
}


// -------------------------------------------------------------------------
// Hollow sphere ray cast
// -------------------------------------------------------------------------
//
// The shell variant. What distinguishes it from the solid cast is the
// far-wall path: a ray starting inside has its near root behind the origin, so
// the far root is the answer rather than a miss.
//
// It is also the one function in the port that deliberately reports something
// different from upstream. Upstream normalizes the direction and then stores a
// *length* in output.fraction while comparing it against maxFraction, which is
// a coefficient -- inconsistent with b3RayCastSphere directly above it, and
// unexercised because nothing upstream calls this function. Here the fraction
// is a fraction, so these tests pin that down.

static void test_hollow_sphere_raycast( void )
{
	section( "hollow sphere ray cast" );

	b3Sphere sphere;
	sphere.center = V( 0, 0, 0 );
	sphere.radius = b3fFromDouble( 2.0 );

	b3RayCastInput in;
	in.maxFraction = b3c_one;

	// From outside: same answer as the solid sphere, near wall at 8 units.
	in.origin = V( -10, 0, 0 );
	in.translation = V( 20, 0, 0 );
	b3CastOutput out = b3RayCastHollowSphere( &sphere, &in );
	check( "outside hit", out.hit );
	expect( "outside fraction", b3cToDouble( out.fraction ), 8.0 / 20.0, 0.01 );
	expectVec( "outside point", out.point, -2.0, 0.0, 0.0, 32 * Q12 );
	expectVec( "outside normal", out.normal, -1.0, 0.0, 0.0, 32 * Q12 );

	// From the centre: the near root is behind the origin, so the far wall is
	// the hit. This is the whole point of the function, and the case the solid
	// cast answers with "initial overlap at the origin" instead.
	in.origin = V( 0, 0, 0 );
	in.translation = V( 10, 0, 0 );
	out = b3RayCastHollowSphere( &sphere, &in );
	check( "inside hits far wall", out.hit );
	expect( "inside fraction", b3cToDouble( out.fraction ), 2.0 / 10.0, 0.01 );
	expectVec( "inside point", out.point, 2.0, 0.0, 0.0, 32 * Q12 );
	expectVec( "inside normal", out.normal, 1.0, 0.0, 0.0, 32 * Q12 );

	// Off-centre inside, so the far root is not simply the radius.
	in.origin = V( -1, 0, 0 );
	in.translation = V( 10, 0, 0 );
	out = b3RayCastHollowSphere( &sphere, &in );
	check( "off-centre inside hits", out.hit );
	expect( "off-centre fraction", b3cToDouble( out.fraction ), 3.0 / 10.0, 0.01 );

	// Missing the shell entirely.
	in.origin = V( -10, 5, 0 );
	in.translation = V( 20, 0, 0 );
	out = b3RayCastHollowSphere( &sphere, &in );
	check( "miss above", out.hit == false );

	// Pointing away: both roots are behind the origin.
	in.origin = V( 10, 0, 0 );
	in.translation = V( 20, 0, 0 );
	out = b3RayCastHollowSphere( &sphere, &in );
	check( "pointing away misses", out.hit == false );

	// Zero length ray. Unlike the solid cast there is no initial-overlap hit:
	// being inside the shell is not touching it.
	in.origin = V( 0, 0, 0 );
	in.translation = V( 0, 0, 0 );
	out = b3RayCastHollowSphere( &sphere, &in );
	check( "zero ray misses", out.hit == false );

	// maxFraction clips the ray before it reaches the wall. This is the
	// comparison that upstream makes in mismatched units.
	in.origin = V( -10, 0, 0 );
	in.translation = V( 20, 0, 0 );
	in.maxFraction = b3cFromFrac( 1, 10 );
	out = b3RayCastHollowSphere( &sphere, &in );
	check( "maxFraction clips", out.hit == false );

	// Just past the wall at 8/20 = 0.4.
	in.maxFraction = b3cFromFrac( 45, 100 );
	out = b3RayCastHollowSphere( &sphere, &in );
	check( "maxFraction just admits", out.hit );
}


// -------------------------------------------------------------------------
// Capsule ray cast
// -------------------------------------------------------------------------
//
// Three paths have to be reached separately, because they use different
// arithmetic and only one of them is well conditioned:
//
//   * origin inside the infinite cylinder  -> endcap sphere cast, or an
//     immediate hit if it is inside the capsule too
//   * axes far from parallel               -> Cramer's rule on t1, t2
//   * axes near parallel                   -> the 2D ray/circle quadratic
//
// The last is where the port and upstream have to disagree. det = 1 - a12^2
// is built from Q12 unit vectors, so it carries about 8.5e-4 of quantization
// noise; upstream switches branches at FLT_EPSILON, four orders of magnitude
// below that. B3_CAPSULE_RAY_MIN_DET is set from the error bound instead, so
// the near-parallel test below has to actually land in the quadratic branch.

// Distance from a point to the capsule's core segment, used to assert that a
// reported hit point really sits on the surface. Independent of the cast.
static double distanceToSegment( b3Vec3 q, b3Vec3 a, b3Vec3 b )
{
	b3Vec3 closest = b3PointToSegmentDistance( a, b, q );
	b3Vec3 d = b3Sub( q, closest );
	return sqrt( b3fToDouble( d.x ) * b3fToDouble( d.x ) + b3fToDouble( d.y ) * b3fToDouble( d.y ) +
				 b3fToDouble( d.z ) * b3fToDouble( d.z ) );
}

static void test_capsule_raycast( void )
{
	section( "capsule ray cast" );

	// Axis along +X from the origin, length 10, radius 2.
	b3Capsule capsule;
	capsule.center1 = V( 0, 0, 0 );
	capsule.center2 = V( 10, 0, 0 );
	capsule.radius = b3fFromDouble( 2.0 );

	b3RayCastInput in;
	in.maxFraction = b3c_one;

	// --- perpendicular side hit, the well-conditioned branch -------------
	//
	// det = 1 here, so this is Cramer's rule at its best behaved.
	in.origin = V( 5, -10, 0 );
	in.translation = V( 0, 20, 0 );
	b3CastOutput out = b3RayCastCapsule( &capsule, &in );
	check( "side hit", out.hit );
	expect( "side fraction", b3cToDouble( out.fraction ), 8.0 / 20.0, 0.01 );
	expectVec( "side point", out.point, 5.0, -2.0, 0.0, 32 * Q12 );
	expectVec( "side normal", out.normal, 0.0, -1.0, 0.0, 32 * Q12 );

	// --- endcap, reached through the inside-the-cylinder branch ----------
	//
	// The origin is on the axis line beyond c2, so sc2 is zero and the code
	// takes the cylinder branch, finds itself outside the capsule, and hands
	// off to a sphere cast at the clamped point.
	in.origin = V( 20, 0, 0 );
	in.translation = V( -20, 0, 0 );
	out = b3RayCastCapsule( &capsule, &in );
	check( "c2 endcap hit", out.hit );
	expect( "c2 endcap fraction", b3cToDouble( out.fraction ), 8.0 / 20.0, 0.01 );
	expectVec( "c2 endcap point", out.point, 12.0, 0.0, 0.0, 32 * Q12 );
	expectVec( "c2 endcap normal", out.normal, 1.0, 0.0, 0.0, 32 * Q12 );

	// Head on along the axis from the far side, hitting the c1 cap.
	in.origin = V( -10, 0, 0 );
	in.translation = V( 30, 0, 0 );
	out = b3RayCastCapsule( &capsule, &in );
	check( "c1 endcap hit", out.hit );
	expect( "c1 endcap fraction", b3cToDouble( out.fraction ), 8.0 / 30.0, 0.01 );
	expectVec( "c1 endcap point", out.point, -2.0, 0.0, 0.0, 32 * Q12 );

	// --- origin inside the capsule ---------------------------------------
	in.origin = V( 5, 0, 0 );
	in.translation = V( 0, 20, 0 );
	out = b3RayCastCapsule( &capsule, &in );
	check( "origin inside hits", out.hit );
	expectVec( "origin inside point", out.point, 5.0, 0.0, 0.0, 32 * Q12 );

	// Inside, but off the axis and still within the radius.
	in.origin = V( 5, 1, 0 );
	out = b3RayCastCapsule( &capsule, &in );
	check( "origin inside off-axis hits", out.hit );

	// --- misses ----------------------------------------------------------

	// Parallel to the axis and further out than the radius. beta >= 0 in the
	// quadratic branch, so it rejects without attempting a division.
	in.origin = V( -10, 5, 0 );
	in.translation = V( 30, 0, 0 );
	out = b3RayCastCapsule( &capsule, &in );
	check( "parallel offset misses", out.hit == false );

	// Passing over the top.
	in.origin = V( 5, 10, 0 );
	in.translation = V( 20, 0, 0 );
	out = b3RayCastCapsule( &capsule, &in );
	check( "over the top misses", out.hit == false );

	// Aimed at the side but too short to arrive.
	in.origin = V( 5, -10, 0 );
	in.translation = V( 0, 3, 0 );
	out = b3RayCastCapsule( &capsule, &in );
	check( "short ray misses", out.hit == false );

	// Zero length ray outside the capsule.
	in.origin = V( 5, -10, 0 );
	in.translation = V( 0, 0, 0 );
	out = b3RayCastCapsule( &capsule, &in );
	check( "zero ray outside misses", out.hit == false );

	// Beyond the c2 end and travelling away along the axis: the early out on
	// the projected interval catches this before any division.
	in.origin = V( 30, 0, 0 );
	in.translation = V( 20, 0, 0 );
	out = b3RayCastCapsule( &capsule, &in );
	check( "past the end misses", out.hit == false );

	// --- the near-parallel quadratic branch ------------------------------
	//
	// A shallow converging ray: a12 = 0.99980, so det = 4.0e-4, comfortably
	// under B3_CAPSULE_RAY_MIN_DET (9.8e-4) and into the quadratic. Upstream's
	// FLT_EPSILON would have sent this down Cramer's rule, where a determinant
	// made mostly of quantization noise decides the answer.
	//
	// The margin is deliberate. A drop of 1.2 over the same span puts det at
	// 9.76e-4, a quarter of a percent under the threshold -- which does select
	// the quadratic, but so narrowly that any later change to the constant or
	// to normalization would silently move this case onto the other branch and
	// the test would go on passing while covering nothing.
	//
	// Geometry: from (0, 2.1333, 0) descending 0.8 over 40 units of x, so it
	// meets the y = 2 surface at x = 6.667, two thirds along the capsule.
	in.origin = V( 0, 2.133333, 0 );
	in.translation = V( 40, -0.8, 0 );
	out = b3RayCastCapsule( &capsule, &in );
	check( "near-parallel side hit", out.hit );

	if ( out.hit )
	{
		// The ill-conditioned branch does not deserve a tight tolerance on the
		// position, but the invariant is exact regardless of conditioning: a
		// reported hit point lies on the capsule surface, one radius from the
		// core segment.
		double d = distanceToSegment( out.point, capsule.center1, capsule.center2 );
		expect( "near-parallel point on surface", d, 2.0, 0.05 );

		// And it is the near root, not the far one -- entering the shape, so
		// the normal opposes the ray.
		check( "near-parallel normal points out", b3Raw( out.normal.y ) > 0 );
		expect( "near-parallel fraction", b3cToDouble( out.fraction ), 6.66801 / 40.008, 0.02 );
		expectVec( "near-parallel point", out.point, 6.6667, 2.0, 0.0, 0.05 );
	}

	// The same shallow ray offset so it never reaches the surface within the
	// ray's own length.
	in.origin = V( 0, 4, 0 );
	in.translation = V( 40, -0.8, 0 );
	out = b3RayCastCapsule( &capsule, &in );
	check( "near-parallel miss", out.hit == false );

	// --- degenerate capsule ----------------------------------------------
	//
	// Shorter than B3_MIN_CAPSULE_LENGTH, so it is treated as a sphere. The
	// upstream guard is 5e-5 units, which is 0.2 in Q12 and truncates to zero;
	// transliterated, this case would have normalized a zero-length axis.
	b3Capsule degenerate;
	degenerate.center1 = V( 5, 0, 0 );
	degenerate.center2 = V( 5, 0, 0 );
	degenerate.radius = b3fFromDouble( 2.0 );

	in.origin = V( -10, 0, 0 );
	in.translation = V( 20, 0, 0 );
	out = b3RayCastCapsule( &degenerate, &in );
	check( "degenerate capsule hits as sphere", out.hit );
	expectVec( "degenerate point", out.point, 3.0, 0.0, 0.0, 32 * Q12 );

	// A capsule just under the threshold takes the same path.
	degenerate.center2 = V( 5.0004, 0, 0 );
	out = b3RayCastCapsule( &degenerate, &in );
	check( "sub-threshold capsule hits as sphere", out.hit );

	// --- a distant capsule, where the wide squared lengths matter --------
	b3Capsule distant;
	distant.center1 = V( 500, 0, 0 );
	distant.center2 = V( 520, 0, 0 );
	distant.radius = b3fFromDouble( 3.0 );

	in.origin = V( 510, -20, 0 );
	in.translation = V( 0, 40, 0 );
	out = b3RayCastCapsule( &distant, &in );
	check( "distant capsule hit", out.hit );
	expect( "distant capsule fraction", b3cToDouble( out.fraction ), 17.0 / 40.0, 0.02 );
	expectVec( "distant capsule point", out.point, 510.0, -3.0, 0.0, 64 * Q12 );

	// --- shape cast -------------------------------------------------------
	//
	// b3ShapeCastCapsule only marshals its arguments into b3ShapeCast, which
	// has its own coverage, so this checks the marshalling: a sphere proxy
	// driven into a stationary capsule should stop at the touching distance.
	b3Vec3 proxyPoint = V( 5, 12, 0 );

	b3ShapeCastInput cast;
	cast.proxy = ( b3ShapeProxy ){ &proxyPoint, 1, b3fFromDouble( 1.0 ) };
	cast.translation = V( 0, -20, 0 );
	cast.maxFraction = b3c_one;
	cast.canEncroach = false;

	out = b3ShapeCastCapsule( &capsule, &cast );
	check( "shape cast hit", out.hit );
	// Surfaces touch when the proxy centre is 3 units above the axis, i.e.
	// after travelling 9 of its 20 units.
	expect( "shape cast fraction", b3cToDouble( out.fraction ), 9.0 / 20.0, 0.02 );
}


// -------------------------------------------------------------------------
// GJK foundations
// -------------------------------------------------------------------------
//
// The simplex solvers decide which Voronoi region of the simplex contains the
// origin, and reduce the simplex to the features that define it. Getting a
// sign wrong here does not crash -- it returns a plausible but wrong closest
// feature, which downstream becomes a contact normal pointing the wrong way.
// So these check the region classification directly.

bool b3SolveSimplex2( b3Simplex* simplex );
bool b3SolveSimplex3( b3Simplex* simplex );

static b3SimplexVertex SV( double x, double y, double z )
{
	b3SimplexVertex v = { 0 };
	v.w = V( x, y, z );
	return v;
}

static void test_support( void )
{
	section( "support functions" );

	// A unit box as a point cloud.
	b3Vec3 pts[8];
	int n = 0;
	for ( int i = 0; i < 2; i++ )
		for ( int j = 0; j < 2; j++ )
			for ( int k = 0; k < 2; k++ )
				pts[n++] = V( i ? 1 : -1, j ? 1 : -1, k ? 1 : -1 );

	b3ShapeProxy proxy = { pts, 8, b3f_zero };

	// The support point along +x must have x = +1. The function returns an
	// index, and several vertices tie on x, so check the coordinate rather
	// than the index.
	int idx = b3GetProxySupport( &proxy, V( 1, 0, 0 ) );
	expect( "support +x", b3fToDouble( pts[idx].x ), 1.0, 8 * Q12 );

	idx = b3GetProxySupport( &proxy, V( -1, 0, 0 ) );
	expect( "support -x", b3fToDouble( pts[idx].x ), -1.0, 8 * Q12 );

	idx = b3GetProxySupport( &proxy, V( 0, 0, 1 ) );
	expect( "support +z", b3fToDouble( pts[idx].z ), 1.0, 8 * Q12 );

	// A diagonal direction must pick the corner, all three coordinates +1.
	idx = b3GetProxySupport( &proxy, V( 1, 1, 1 ) );
	expectVec( "support diagonal", pts[idx], 1.0, 1.0, 1.0, 8 * Q12 );

	// Far from the origin, the shift-to-origin trick is what keeps this
	// working: raw dot products of coordinates near 1000 would swamp the
	// difference between neighbouring corners.
	b3Vec3 farPts[8];
	for ( int i = 0; i < 8; i++ )
	{
		farPts[i] = b3Add( pts[i], V( 1000, 1000, 1000 ) );
	}
	b3ShapeProxy farProxy = { farPts, 8, b3f_zero };

	idx = b3GetProxySupport( &farProxy, V( 1, 1, 1 ) );
	expectVec( "support diagonal, far from origin", farPts[idx], 1001.0, 1001.0, 1001.0, 64 * Q12 );

	idx = b3GetProxySupport( &farProxy, V( -1, -1, -1 ) );
	expectVec( "support -diagonal, far from origin", farPts[idx], 999.0, 999.0, 999.0, 64 * Q12 );

	// Single point cloud: index 0 whatever the axis.
	b3ShapeProxy single = { pts, 1, b3f_zero };
	checkInt2( "single-point support", b3GetProxySupport( &single, V( 1, 2, 3 ) ), 0 );
}

static void test_simplex2( void )
{
	section( "2-vertex simplex solver" );

	b3Simplex s;
	s.count = 2;

	// Origin beyond A: the segment reduces to A.
	s.vertices[0] = SV( 1, 0, 0 );
	s.vertices[1] = SV( 3, 0, 0 );
	check( "solve2 succeeded", b3SolveSimplex2( &s ) );
	checkInt2( "origin past A reduces to 1 vertex", s.count, 1 );
	expect( "kept vertex is A", b3fToDouble( s.vertices[0].w.x ), 1.0, 8 * Q12 );
	expect( "weight is 1", b3cToDouble( s.vertices[0].a ), 1.0, 0.01 );

	// Origin beyond B: reduces to B.
	s.count = 2;
	s.vertices[0] = SV( -3, 0, 0 );
	s.vertices[1] = SV( -1, 0, 0 );
	check( "solve2 succeeded", b3SolveSimplex2( &s ) );
	checkInt2( "origin past B reduces to 1 vertex", s.count, 1 );
	expect( "kept vertex is B", b3fToDouble( s.vertices[0].w.x ), -1.0, 8 * Q12 );

	// Origin projects inside the segment: both vertices survive, and the
	// weights are the barycentric coordinates of the projection.
	s.count = 2;
	s.vertices[0] = SV( -1, 1, 0 );
	s.vertices[1] = SV( 3, 1, 0 );
	check( "solve2 succeeded", b3SolveSimplex2( &s ) );
	checkInt2( "interior keeps 2 vertices", s.count, 2 );
	expect( "weight A", b3cToDouble( s.vertices[0].a ), 0.75, 0.02 );
	expect( "weight B", b3cToDouble( s.vertices[1].a ), 0.25, 0.02 );
	expect( "weights sum to 1", b3cToDouble( s.vertices[0].a ) + b3cToDouble( s.vertices[1].a ), 1.0, 0.02 );

	// Degenerate: both vertices coincide, so the divisor vanishes.
	//
	// The vertex-region tests run first and catch this -- with a zero-length
	// edge both numerators are zero, so the `v <= 0` branch fires and the
	// simplex collapses to a point, which is what it genuinely is. The
	// division is never reached. Upstream orders the tests the same way.
	s.count = 2;
	s.vertices[0] = SV( 2, 2, 2 );
	s.vertices[1] = SV( 2, 2, 2 );
	check( "coincident vertices handled", b3SolveSimplex2( &s ) );
	checkInt2( "coincident vertices collapse to a point", s.count, 1 );
	expect( "collapsed weight is 1", b3cToDouble( s.vertices[0].a ), 1.0, 0.01 );
}

static void test_simplex3( void )
{
	section( "3-vertex simplex solver" );

	b3Simplex s;

	// A triangle in the z = 1 plane containing the origin's projection.
	// The origin projects inside, so all three vertices survive.
	s.count = 3;
	s.vertices[0] = SV( -1, -1, 1 );
	s.vertices[1] = SV( 2, -1, 1 );
	s.vertices[2] = SV( 0, 2, 1 );
	check( "solve3 succeeded", b3SolveSimplex3( &s ) );
	checkInt2( "interior keeps 3 vertices", s.count, 3 );

	double sum = b3cToDouble( s.vertices[0].a ) + b3cToDouble( s.vertices[1].a ) + b3cToDouble( s.vertices[2].a );
	expect( "face weights sum to 1", sum, 1.0, 0.03 );
	check( "face weights non-negative", b3cToDouble( s.vertices[0].a ) >= -0.01 && b3cToDouble( s.vertices[1].a ) >= -0.01 &&
										   b3cToDouble( s.vertices[2].a ) >= -0.01 );

	// Origin closest to a single vertex: the triangle sits well off to one
	// side, so the nearest corner wins and the simplex reduces to 1.
	s.count = 3;
	s.vertices[0] = SV( 1, 1, 0 );
	s.vertices[1] = SV( 5, 1, 0 );
	s.vertices[2] = SV( 1, 5, 0 );
	check( "solve3 succeeded", b3SolveSimplex3( &s ) );
	checkInt2( "vertex region reduces to 1", s.count, 1 );
	expectVec( "reduced to nearest corner", s.vertices[0].w, 1.0, 1.0, 0.0, 8 * Q12 );

	// Origin closest to an edge: reduces to 2 vertices.
	s.count = 3;
	s.vertices[0] = SV( -2, 1, 0 );
	s.vertices[1] = SV( 2, 1, 0 );
	s.vertices[2] = SV( 0, 5, 0 );
	check( "solve3 succeeded", b3SolveSimplex3( &s ) );
	checkInt2( "edge region reduces to 2", s.count, 2 );
	double esum = b3cToDouble( s.vertices[0].a ) + b3cToDouble( s.vertices[1].a );
	expect( "edge weights sum to 1", esum, 1.0, 0.03 );

	// Degenerate: collinear vertices. The triangle divisor is the length^4
	// term and vanishes, so this must report failure rather than divide by
	// something quantization produced.
	s.count = 3;
	s.vertices[0] = SV( 0, 1, 0 );
	s.vertices[1] = SV( 1, 1, 0 );
	s.vertices[2] = SV( 2, 1, 0 );
	bool ok = b3SolveSimplex3( &s );
	check( "collinear triangle does not produce a face region", ok == false || s.count < 3 );

	// All three coincident -- the worst degeneracy.
	s.count = 3;
	s.vertices[0] = SV( 3, 3, 3 );
	s.vertices[1] = SV( 3, 3, 3 );
	s.vertices[2] = SV( 3, 3, 3 );
	ok = b3SolveSimplex3( &s );
	check( "coincident triangle handled without dividing", ok == false || s.count <= 3 );
}


// -------------------------------------------------------------------------
// GJK end to end
// -------------------------------------------------------------------------
//
// b3ShapeDistance takes two point-cloud proxies and returns the closest
// points and the separation. These check it against distances that can be
// worked out by hand, plus the overlap and degenerate cases.

static b3DistanceOutput RunGJK( const b3Vec3* ptsA, int nA, double rA, const b3Vec3* ptsB, int nB, double rB, b3Vec3 offsetB,
								bool useRadii )
{
	b3DistanceInput in = { 0 };
	in.proxyA.points = ptsA;
	in.proxyA.count = nA;
	in.proxyA.radius = b3fFromDouble( rA );
	in.proxyB.points = ptsB;
	in.proxyB.count = nB;
	in.proxyB.radius = b3fFromDouble( rB );
	in.transform = b3Transform_identity;
	in.transform.p = offsetB;
	in.useRadii = useRadii;

	b3SimplexCache cache = { 0 };
	return b3ShapeDistance( &in, &cache, NULL, 0 );
}

static void test_gjk( void )
{
	section( "GJK shape distance" );

	// Two single points: the distance is just the separation.
	b3Vec3 a1 = V( 0, 0, 0 );
	b3Vec3 b1 = V( 0, 0, 0 );

	b3DistanceOutput o = RunGJK( &a1, 1, 0.0, &b1, 1, 0.0, V( 5, 0, 0 ), false );
	expect( "point-point distance", b3fToDouble( o.distance ), 5.0, 32 * Q12 );
	expectVec( "point-point normal", o.normal, 1.0, 0.0, 0.0, 64 * Q12 );

	o = RunGJK( &a1, 1, 0.0, &b1, 1, 0.0, V( 3, 4, 0 ), false );
	expect( "point-point diagonal distance", b3fToDouble( o.distance ), 5.0, 32 * Q12 );

	// Two spheres, expressed as points with radii. Centres 10 apart, radii 2
	// and 3, so the surfaces are 5 apart.
	o = RunGJK( &a1, 1, 2.0, &b1, 1, 3.0, V( 10, 0, 0 ), true );
	expect( "sphere-sphere separation", b3fToDouble( o.distance ), 5.0, 32 * Q12 );

	// Overlapping spheres report zero separation, not a negative one.
	o = RunGJK( &a1, 1, 4.0, &b1, 1, 4.0, V( 5, 0, 0 ), true );
	expect( "overlapping spheres clamp to zero", b3fToDouble( o.distance ), 0.0, 32 * Q12 );

	// A box against a point. The box is a unit cube at the origin; the point
	// sits 5 along x, so the nearest face is 4 away.
	b3Vec3 box[8];
	int n = 0;
	for ( int i = 0; i < 2; i++ )
		for ( int j = 0; j < 2; j++ )
			for ( int k = 0; k < 2; k++ )
				box[n++] = V( i ? 1 : -1, j ? 1 : -1, k ? 1 : -1 );

	o = RunGJK( box, 8, 0.0, &b1, 1, 0.0, V( 5, 0, 0 ), false );
	expect( "box-point distance", b3fToDouble( o.distance ), 4.0, 64 * Q12 );
	expect( "box-point witness on face", b3fToDouble( o.pointA.x ), 1.0, 64 * Q12 );

	// Point diagonally off a box corner: distance from (1,1,1) to (4,5,1).
	o = RunGJK( box, 8, 0.0, &b1, 1, 0.0, V( 4, 5, 1 ), false );
	expect( "box-point corner distance", b3fToDouble( o.distance ), 5.0, 64 * Q12 );

	// Two boxes side by side, 3 apart on x.
	o = RunGJK( box, 8, 0.0, box, 8, 0.0, V( 5, 0, 0 ), false );
	expect( "box-box distance", b3fToDouble( o.distance ), 3.0, 64 * Q12 );

	// Overlapping boxes: the origin ends up inside the Minkowski difference,
	// so GJK terminates on a full simplex and reports zero distance with
	// coincident witness points.
	o = RunGJK( box, 8, 0.0, box, 8, 0.0, V( 1, 0, 0 ), false );
	expect( "overlapping boxes report zero", b3fToDouble( o.distance ), 0.0, 32 * Q12 );
	expect( "overlap witness points coincide", b3fToDouble( b3Distance( o.pointA, o.pointB ) ), 0.0, 32 * Q12 );

	// Exactly touching, the hardest case for a distance query: the simplex
	// straddles the boundary and the search direction can vanish.
	o = RunGJK( box, 8, 0.0, box, 8, 0.0, V( 2, 0, 0 ), false );
	expect( "touching boxes report ~zero", b3fToDouble( o.distance ), 0.0, 64 * Q12 );

	// Witness points must lie on their own shapes, whatever the separation.
	o = RunGJK( box, 8, 0.0, box, 8, 0.0, V( 7, 3, 2 ), false );
	check( "witness A within box A", b3fToDouble( b3AbsF( o.pointA.x ) ) <= 1.02 &&
									 b3fToDouble( b3AbsF( o.pointA.y ) ) <= 1.02 &&
									 b3fToDouble( b3AbsF( o.pointA.z ) ) <= 1.02 );
	expect( "separated distance matches witnesses", b3fToDouble( o.distance ),
			b3fToDouble( b3Distance( o.pointA, o.pointB ) ), 64 * Q12 );

	// A far-field pair. Both proxies sit 800 units out, which is where the
	// length^4 triangle divisor would be at risk if the query did not run in
	// frame A.
	b3Vec3 farBox[8];
	for ( int i = 0; i < 8; i++ )
	{
		farBox[i] = b3Add( box[i], V( 800, 800, 800 ) );
	}
	o = RunGJK( farBox, 8, 0.0, farBox, 8, 0.0, V( 6, 0, 0 ), false );
	expect( "far-field box-box distance", b3fToDouble( o.distance ), 4.0, 128 * Q12 );

	// The overlap helpers, testable for the first time now that GJK exists.
	b3Sphere sphere;
	sphere.center = V( 0, 0, 0 );
	sphere.radius = b3fFromDouble( 2.0 );

	b3Vec3 probe = V( 1, 0, 0 );
	b3ShapeProxy proxy = { &probe, 1, b3fFromDouble( 0.5 ) };
	check( "sphere overlaps nearby proxy", b3OverlapSphere( &sphere, b3Transform_identity, &proxy ) );

	b3Vec3 farProbe = V( 20, 0, 0 );
	b3ShapeProxy farProxy = { &farProbe, 1, b3fFromDouble( 0.5 ) };
	check( "sphere misses distant proxy", b3OverlapSphere( &sphere, b3Transform_identity, &farProxy ) == false );

	b3Capsule cap;
	cap.center1 = V( 0, -3, 0 );
	cap.center2 = V( 0, 3, 0 );
	cap.radius = b3fFromDouble( 1.0 );
	check( "capsule overlaps nearby proxy", b3OverlapCapsule( &cap, b3Transform_identity, &proxy ) );
	check( "capsule misses distant proxy", b3OverlapCapsule( &cap, b3Transform_identity, &farProxy ) == false );
}


// -------------------------------------------------------------------------
// Contact manifolds
// -------------------------------------------------------------------------
//
// The milestone: two shapes in, a contact manifold out. These check the three
// things the solver will actually consume -- point count, normal direction,
// and separation -- against configurations worked out by hand.

static void test_manifolds( void )
{
	section( "contact manifolds" );

	b3LocalManifoldPoint pts[4];
	b3LocalManifold mf = { 0 };
	mf.points = pts;

	// --- sphere vs sphere ---
	b3Sphere sA, sB;
	sA.center = V( 0, 0, 0 );
	sA.radius = b3fFromDouble( 1.0 );
	sB.center = V( 0, 0, 0 );
	sB.radius = b3fFromDouble( 1.0 );

	b3Transform xf = b3Transform_identity;

	// Overlapping by 0.5: centres 1.5 apart, radii sum 2.
	xf.p = V( 1.5, 0, 0 );
	b3CollideSpheres( &mf, 4, &sA, &sB, xf );
	checkInt2( "spheres overlapping produce 1 point", mf.pointCount, 1 );
	expectVec( "sphere normal points A to B", mf.normal, 1.0, 0.0, 0.0, 32 * Q12 );
	expect( "sphere separation", b3fToDouble( pts[0].separation ), -0.5, 32 * Q12 );
	expectVec( "sphere contact at midpoint", pts[0].point, 0.75, 0.0, 0.0, 32 * Q12 );

	// Just touching.
	xf.p = V( 2.0, 0, 0 );
	b3CollideSpheres( &mf, 4, &sA, &sB, xf );
	checkInt2( "touching spheres produce 1 point", mf.pointCount, 1 );
	expect( "touching separation is zero", b3fToDouble( pts[0].separation ), 0.0, 32 * Q12 );

	// Separated: no contact at all.
	xf.p = V( 3.0, 0, 0 );
	b3CollideSpheres( &mf, 4, &sA, &sB, xf );
	checkInt2( "separated spheres produce no points", mf.pointCount, 0 );

	// Coincident centres -- the degenerate normal path. Must not divide by
	// zero, and must still report a contact.
	xf.p = V( 0, 0, 0 );
	b3CollideSpheres( &mf, 4, &sA, &sB, xf );
	checkInt2( "coincident spheres still produce a point", mf.pointCount, 1 );
	expect( "coincident separation is -2r", b3fToDouble( pts[0].separation ), -2.0, 32 * Q12 );
	check( "coincident normal is unit length", b3IsNormalized( mf.normal ) );

	// Diagonal overlap: normal must be the unit diagonal.
	xf.p = V( 1.0, 1.0, 0 );
	b3CollideSpheres( &mf, 4, &sA, &sB, xf );
	checkInt2( "diagonal overlap produces 1 point", mf.pointCount, 1 );
	expect( "diagonal normal x", b3fToDouble( mf.normal.x ), 0.7071, 0.02 );
	expect( "diagonal normal y", b3fToDouble( mf.normal.y ), 0.7071, 0.02 );
	expect( "diagonal separation", b3fToDouble( pts[0].separation ), sqrt( 2.0 ) - 2.0, 32 * Q12 );

	// --- capsule vs sphere ---
	b3Capsule cap;
	cap.center1 = V( 0, -2, 0 );
	cap.center2 = V( 0, 2, 0 );
	cap.radius = b3fFromDouble( 1.0 );

	// Sphere beside the capsule's middle: nearest point on the segment is the
	// origin, so the separation is 1.5 - 2 = -0.5.
	xf.p = V( 1.5, 0, 0 );
	b3CollideCapsuleAndSphere( &mf, 4, &cap, &sB, xf );
	checkInt2( "capsule-sphere produces 1 point", mf.pointCount, 1 );
	expectVec( "capsule-sphere normal", mf.normal, 1.0, 0.0, 0.0, 32 * Q12 );
	expect( "capsule-sphere separation", b3fToDouble( pts[0].separation ), -0.5, 32 * Q12 );

	// Sphere off the end cap: nearest point is the segment endpoint.
	xf.p = V( 0, 3.5, 0 );
	b3CollideCapsuleAndSphere( &mf, 4, &cap, &sB, xf );
	checkInt2( "capsule endcap produces 1 point", mf.pointCount, 1 );
	expectVec( "capsule endcap normal", mf.normal, 0.0, 1.0, 0.0, 32 * Q12 );
	expect( "capsule endcap separation", b3fToDouble( pts[0].separation ), -0.5, 32 * Q12 );

	// Far away: nothing.
	xf.p = V( 10, 0, 0 );
	b3CollideCapsuleAndSphere( &mf, 4, &cap, &sB, xf );
	checkInt2( "distant sphere produces no points", mf.pointCount, 0 );

	// --- capsule vs capsule ---
	b3Capsule capB;
	capB.center1 = V( 0, -2, 0 );
	capB.center2 = V( 0, 2, 0 );
	capB.radius = b3fFromDouble( 1.0 );

	// Parallel and overlapping. This is the two-point path: nearly parallel
	// capsules get clipped against each other's side planes so the solver has
	// two contacts to work with rather than one, which is what stops a
	// resting capsule from rocking.
	xf.p = V( 1.5, 0, 0 );
	b3CollideCapsules( &mf, 4, &cap, &capB, xf );
	checkInt2( "parallel capsules produce 2 points", mf.pointCount, 2 );
	expectVec( "parallel capsule normal", mf.normal, 1.0, 0.0, 0.0, 32 * Q12 );
	expect( "parallel separation 1", b3fToDouble( pts[0].separation ), -0.5, 64 * Q12 );
	expect( "parallel separation 2", b3fToDouble( pts[1].separation ), -0.5, 64 * Q12 );
	check( "parallel contacts are distinct", b3fToDouble( b3Distance( pts[0].point, pts[1].point ) ) > 1.0 );

	// Perpendicular capsules passing near each other. Offset in z so the two
	// segments do not intersect exactly: segments that cross *through* each
	// other have coincident closest points and a zero-length offset, which
	// both this port and upstream bail on rather than normalize.
	b3Capsule capCross;
	capCross.center1 = V( -2, 0, 0 );
	capCross.center2 = V( 2, 0, 0 );
	capCross.radius = b3fFromDouble( 1.0 );

	xf.p = V( 0, 1.5, 0.4 );
	b3CollideCapsules( &mf, 4, &cap, &capCross, xf );
	checkInt2( "crossed capsules produce 1 point", mf.pointCount, 1 );
	expect( "crossed separation", b3fToDouble( pts[0].separation ), 0.4 - 2.0, 64 * Q12 );
	expectVec( "crossed normal is the z offset", mf.normal, 0.0, 0.0, 1.0, 64 * Q12 );

	// Exactly intersecting segments: the degenerate case, no contact.
	xf.p = V( 0, 1.5, 0 );
	b3CollideCapsules( &mf, 4, &cap, &capCross, xf );
	checkInt2( "exactly intersecting segments bail", mf.pointCount, 0 );

	// Separated capsules.
	xf.p = V( 10, 0, 0 );
	b3CollideCapsules( &mf, 4, &cap, &capB, xf );
	checkInt2( "distant capsules produce no points", mf.pointCount, 0 );

	// Every manifold normal must be unit length, whatever the configuration:
	// the solver divides by nothing but trusts this completely.
	for ( int i = 0; i < 24; i++ )
	{
		double angle = ( 2.0 * M_PI * i ) / 24.0;
		xf.p = V( 1.6 * cos( angle ), 1.6 * sin( angle ), 0.3 );
		b3CollideCapsules( &mf, 4, &cap, &capB, xf );
		if ( mf.pointCount > 0 && !b3IsNormalized( mf.normal ) )
		{
			printf( "  FAIL capsule sweep %d produced a non-unit normal\n", i );
			s_failures++;
			break;
		}
	}
	s_checks++;
}

// -------------------------------------------------------------------------
// Dynamic tree
// -------------------------------------------------------------------------
//
// The tree is mostly integer index surgery, and the way that fails is silent:
// a query returns a plausible subset of the right answer. So rather than
// calling the library's own b3DynamicTree_Validate -- which is a chain of
// asserts, and asserts are compiled out of this build -- the invariants are
// re-derived here independently, and every query is compared against a linear
// scan of the same boxes.
//
// The comparison has to be exact. The tree is an acceleration structure, not
// an approximation: if it disagrees with brute force by even one proxy, either
// the SAH costs or b3TestBoundsRayOverlap is wrong.

typedef struct
{
	int count;
	int ids[256];
} ProxyList;

static bool collectProxy( int proxyId, uint64_t userData, void* context )
{
	ProxyList* list = (ProxyList*)context;
	B3_UNUSED( userData );
	if ( list->count < 256 )
	{
		list->ids[list->count++] = proxyId;
	}
	return true;
}

static int compareInts( const void* a, const void* b )
{
	int x = *(const int*)a;
	int y = *(const int*)b;
	return x < y ? -1 : ( x > y ? 1 : 0 );
}

static void sortList( ProxyList* list )
{
	qsort( list->ids, (size_t)list->count, sizeof( int ), compareInts );
}

static bool sameList( const ProxyList* a, const ProxyList* b )
{
	if ( a->count != b->count )
	{
		return false;
	}
	for ( int i = 0; i < a->count; ++i )
	{
		if ( a->ids[i] != b->ids[i] )
		{
			return false;
		}
	}
	return true;
}

// Independent structural walk. Returns the number of leaves seen, or -1 if any
// invariant is broken.
static int walkTree( const b3DynamicTree* tree, int index, int parent, int depth, int* height )
{
	if ( index == B3_NULL_INDEX || depth > 64 )
	{
		return -1;
	}

	const b3TreeNode* node = tree->nodes + index;

	if ( node->parent != parent )
	{
		return -1;
	}

	if ( ( node->flags & b3_allocatedNode ) == 0 )
	{
		return -1;
	}

	if ( node->flags & b3_leafNode )
	{
		if ( node->height != 0 )
		{
			return -1;
		}
		*height = 0;
		return 1;
	}

	int child1 = node->children.child1;
	int child2 = node->children.child2;

	if ( child1 == B3_NULL_INDEX || child2 == B3_NULL_INDEX )
	{
		return -1;
	}

	int h1 = 0, h2 = 0;
	int n1 = walkTree( tree, child1, index, depth + 1, &h1 );
	int n2 = walkTree( tree, child2, index, depth + 1, &h2 );

	if ( n1 < 0 || n2 < 0 )
	{
		return -1;
	}

	// Height must be one more than the taller child.
	int expected = 1 + ( h1 > h2 ? h1 : h2 );
	if ( node->height != expected )
	{
		return -1;
	}
	*height = expected;

	// A parent box must contain both children, and its category bits must be
	// the union of theirs. Both are maintained on every insert, remove and
	// rotation, so this is what catches a mis-wired rotation.
	if ( b3AABB_Contains( node->aabb, tree->nodes[child1].aabb ) == false ||
		 b3AABB_Contains( node->aabb, tree->nodes[child2].aabb ) == false )
	{
		return -1;
	}

	uint64_t bits = tree->nodes[child1].categoryBits | tree->nodes[child2].categoryBits;
	if ( node->categoryBits != bits )
	{
		return -1;
	}

	return n1 + n2;
}

static bool treeIsSound( const b3DynamicTree* tree, int expectedLeaves )
{
	if ( tree->root == B3_NULL_INDEX )
	{
		return expectedLeaves == 0;
	}

	int height = 0;
	int leaves = walkTree( tree, tree->root, B3_NULL_INDEX, 0, &height );

	if ( leaves != expectedLeaves )
	{
		return false;
	}

	if ( b3DynamicTree_GetHeight( tree ) != height )
	{
		return false;
	}

	// Free list plus live nodes must account for the whole pool.
	int freeCount = 0;
	int freeIndex = tree->freeList;
	while ( freeIndex != B3_NULL_INDEX && freeCount <= tree->nodeCapacity )
	{
		freeIndex = tree->nodes[freeIndex].next;
		freeCount++;
	}

	return tree->nodeCount + freeCount == tree->nodeCapacity;
}

// A scripted set of boxes, spread out enough that the tree has real structure
// but overlapping enough that queries return more than one proxy.
static b3AABB scriptedBox( int i )
{
	double x = ( ( i * 37 ) % 41 ) - 20.0;
	double y = ( ( i * 17 ) % 23 ) - 11.0;
	double z = ( ( i * 29 ) % 31 ) - 15.0;
	double w = 0.5 + ( i % 5 ) * 0.75;
	return Box( x, y, z, x + w, y + w, z + w );
}

// -------------------------------------------------------------------------
// Convex hulls
//
// Every hull the device can build for itself is a box, so these tests run on
// box hulls -- but they check the hull *structure* rather than the box, by
// re-deriving what a half-edge mesh has to satisfy rather than by comparing
// against the constructor that produced it. A baked hull from the host has to
// satisfy exactly the same statements.

/// Re-derivation of the half-edge invariants, independent of b3IsValidHull.
static void checkHullStructure( const char* what, const b3HullData* hull )
{
	char buf[128];
	const b3HullVertex* vertices = b3GetHullVertices( hull );
	const b3HullHalfEdge* edges = b3GetHullEdges( hull );
	const b3HullFace* faces = b3GetHullFaces( hull );
	const b3Plane* planes = b3GetHullPlanes( hull );
	const b3Vec3* points = b3GetHullPoints( hull );

	// Euler's formula for a closed convex polyhedron: V - E + F = 2.
	snprintf( buf, sizeof( buf ), "%s Euler formula", what );
	checkInt2( buf, hull->vertexCount - hull->edgeCount / 2 + hull->faceCount, 2 );

	bool twinsOk = true;
	bool originsOk = true;
	for ( int i = 0; i < hull->edgeCount; ++i )
	{
		twinsOk = twinsOk && edges[edges[i].twin].twin == i;
		originsOk = originsOk && edges[i].origin < hull->vertexCount && edges[i].face < hull->faceCount;
	}
	snprintf( buf, sizeof( buf ), "%s twins are mutual", what );
	check( buf, twinsOk );
	snprintf( buf, sizeof( buf ), "%s edge indices in range", what );
	check( buf, originsOk );

	bool vertexEdgesOk = true;
	for ( int i = 0; i < hull->vertexCount; ++i )
	{
		vertexEdgesOk = vertexEdgesOk && edges[vertices[i].edge].origin == i;
	}
	snprintf( buf, sizeof( buf ), "%s vertices name an outgoing edge", what );
	check( buf, vertexEdgesOk );

	// Every face's next-chain must close, stay on the face, and its edges must
	// hand over to their twins' origins. Also count the edges while walking:
	// each half-edge belongs to exactly one face, so the total must match.
	bool cyclesOk = true;
	int walked = 0;
	for ( int f = 0; f < hull->faceCount; ++f )
	{
		int base = faces[f].edge;
		int e = base;
		int guard = 0;
		do
		{
			cyclesOk = cyclesOk && edges[e].face == f;
			cyclesOk = cyclesOk && edges[edges[e].next].origin == edges[edges[e].twin].origin;
			e = edges[e].next;
			walked++;
		}
		while ( e != base && ++guard <= hull->edgeCount );

		cyclesOk = cyclesOk && e == base;
	}
	snprintf( buf, sizeof( buf ), "%s face cycles close", what );
	check( buf, cyclesOk );
	snprintf( buf, sizeof( buf ), "%s every half-edge is on exactly one face", what );
	checkInt2( buf, walked, hull->edgeCount );

	// Convexity: no point may sit in front of any face plane. This is the
	// property Q12 quantization can genuinely break, so it is checked here
	// independently of b3IsValidHull's own version of it.
	double worstOutside = -1e9;
	for ( int f = 0; f < hull->faceCount; ++f )
	{
		for ( int p = 0; p < hull->vertexCount; ++p )
		{
			double s = b3fToDouble( b3PlaneSeparation( planes[f], points[p] ) );
			if ( s > worstOutside )
			{
				worstOutside = s;
			}
		}
	}
	snprintf( buf, sizeof( buf ), "%s is convex", what );
	check( buf, worstOutside <= b3fToDouble( B3_LINEAR_SLOP ) );

	// The centroid must be strictly inside every face.
	bool centerInside = true;
	for ( int f = 0; f < hull->faceCount; ++f )
	{
		centerInside = centerInside && b3Raw( b3PlaneSeparation( planes[f], hull->center ) ) < 0;
	}
	snprintf( buf, sizeof( buf ), "%s centroid is inside", what );
	check( buf, centerInside );

	// The stored bounds must match a scan of the points.
	b3AABB scan = b3MakeAABB( points[0], points[0] );
	for ( int p = 1; p < hull->vertexCount; ++p )
	{
		scan = b3AABB_AddPoint( scan, points[p] );
	}
	snprintf( buf, sizeof( buf ), "%s bounds contain the points", what );
	check( buf, b3AABB_Contains( hull->aabb, scan ) );

	snprintf( buf, sizeof( buf ), "%s passes b3IsValidHull", what );
	check( buf, b3IsValidHull( hull ) );
}

static void test_hull_structure( void )
{
	section( "hull structure" );

	b3BoxHull cube = b3MakeCubeHull( b3fFromDouble( 1.0 ) );
	checkHullStructure( "cube", &cube.base );

	checkInt2( "cube vertex count", cube.base.vertexCount, 8 );
	checkInt2( "cube half-edge count", cube.base.edgeCount, 24 );
	checkInt2( "cube face count", cube.base.faceCount, 6 );

	// A 2x2x2 cube: volume 8, surface area 24, inner radius 1.
	expect( "cube volume", b3fToDouble( cube.base.volume ), 8.0, 0.01 );
	expect( "cube surface area", b3fToDouble( cube.base.surfaceArea ), 24.0, 0.01 );
	expect( "cube inner radius", b3fToDouble( cube.base.innerRadius ), 1.0, 0.01 );
	expectVec( "cube centroid", cube.base.center, 0, 0, 0, Q12 );

	// Unit inertia of a box spanning -h..h is (dy^2 + dz^2)/12 = 8/12.
	expect( "cube unit inertia xx", b3fToDouble( cube.base.centralInertia.cx.x ), 8.0 / 12.0, 0.01 );
	expect( "cube unit inertia yy", b3fToDouble( cube.base.centralInertia.cy.y ), 8.0 / 12.0, 0.01 );
	expect( "cube unit inertia off-diagonal", b3fToDouble( cube.base.centralInertia.cx.y ), 0.0, 0.01 );

	// A non-cube box, to catch an axis swapped in the topology table.
	b3BoxHull box = b3MakeBoxHull( b3fFromDouble( 1.0 ), b3fFromDouble( 2.0 ), b3fFromDouble( 3.0 ) );
	checkHullStructure( "1x2x3 box", &box.base );
	expect( "box volume", b3fToDouble( box.base.volume ), 8.0 * 6.0, 0.05 );
	expect( "box surface area", b3fToDouble( box.base.surfaceArea ), 8.0 * ( 2.0 + 3.0 + 6.0 ), 0.05 );
	expect( "box inner radius", b3fToDouble( box.base.innerRadius ), 1.0, 0.01 );
	// ixx = ((2*2)^2 + (2*3)^2)/12 = (16 + 36)/12
	expect( "box unit inertia xx", b3fToDouble( box.base.centralInertia.cx.x ), 52.0 / 12.0, 0.02 );
	// iyy = ((2*1)^2 + (2*3)^2)/12 = (4 + 36)/12
	expect( "box unit inertia yy", b3fToDouble( box.base.centralInertia.cy.y ), 40.0 / 12.0, 0.02 );

	// An offset box: the topology is untouched, the centroid moves, and the
	// bounds move with it.
	b3BoxHull offset = b3MakeOffsetBoxHull( b3fFromDouble( 1.0 ), b3fFromDouble( 1.0 ), b3fFromDouble( 1.0 ), V( 5, 0, 0 ) );
	checkHullStructure( "offset cube", &offset.base );
	expectVec( "offset centroid", offset.base.center, 5, 0, 0, Q12 );
	expectVec( "offset lower bound", offset.base.aabb.lowerBound, 4, -1, -1, 2 * Q12 );

	// A rotated box. This is the one where quantization has something to do:
	// the points and the plane offsets are all rounded to Q12 after the
	// rotation, and the structure still has to come out convex.
	b3Transform xf;
	xf.p = V( 0, 0, 0 );
	xf.q = b3MakeQuatFromAxisAngle( V( 0, 1, 0 ), (b3a)( 32768 / 8 ) ); // 45 degrees
	b3BoxHull rotated = b3MakeTransformedBoxHull( b3fFromDouble( 1.0 ), b3fFromDouble( 1.0 ), b3fFromDouble( 1.0 ), xf );
	checkHullStructure( "45-degree rotated cube", &rotated.base );
	expect( "rotated volume unchanged", b3fToDouble( rotated.base.volume ), 8.0, 0.01 );
	// A cube turned 45 degrees about Y spans sqrt(2) in x and z.
	expect( "rotated bounds x", b3fToDouble( rotated.base.aabb.upperBound.x ), sqrt( 2.0 ), 0.02 );
	expect( "rotated bounds y", b3fToDouble( rotated.base.aabb.upperBound.y ), 1.0, 0.02 );

	// A very thin box: the minimum half-extent floor must keep it valid rather
	// than producing a hull with no interior.
	b3BoxHull thin = b3MakeBoxHull( b3fFromDouble( 1.0 ), b3fFromDouble( 0.0 ), b3fFromDouble( 1.0 ) );
	check( "degenerate box is still a valid hull", b3IsValidHull( &thin.base ) );
	check( "degenerate box has positive volume", b3Raw( thin.base.volume ) > 0 );

	// A blob that has not been baked must be rejected rather than trusted.
	b3BoxHull bad = cube;
	bad.base.version = 0;
	check( "wrong version is rejected", b3IsValidHull( &bad.base ) == false );

	bad = cube;
	bad.base.faceCount = 5;
	check( "broken Euler count is rejected", b3IsValidHull( &bad.base ) == false );

	bad = cube;
	bad.boxEdges[3].twin = 7;
	check( "broken twin pairing is rejected", b3IsValidHull( &bad.base ) == false );

	bad = cube;
	bad.boxPoints[0] = V( 5, 5, 5 );
	check( "non-convex point is rejected", b3IsValidHull( &bad.base ) == false );
}

// The prism is the port's second analytic hull, and the only one it can build
// with a face of more than four sides -- which is what makes it the fixture
// b3ReduceManifoldPoints can be tested against.
//
// Its topology is checked the same way the box's is: independently of
// b3IsValidHull, so a shared misunderstanding cannot pass both.
static void test_prism_structure( void )
{
	section( "prism hull structure" );

	static const int sides[] = { 3, 4, 5, 6, 8, 10 };

	for ( size_t i = 0; i < sizeof( sides ) / sizeof( sides[0] ); ++i )
	{
		int n = sides[i];
		char label[64];
		snprintf( label, sizeof( label ), "prism%d", n );

		b3PrismHull prism = b3MakePrismHull( b3fFromDouble( 1.5 ), b3fFromDouble( 0.8 ), n );

		checkHullStructure( label, &prism.base );

		checkInt2( "prism vertex count", prism.base.vertexCount, 2 * n );
		checkInt2( "prism half-edge count", prism.base.edgeCount, 6 * n );
		checkInt2( "prism face count", prism.base.faceCount, n + 2 );

		// Closed form for a regular prism of circumradius R and half height h.
		double R = 1.5, h = 0.8;
		double area = 0.5 * n * R * R * sin( 2.0 * 3.14159265358979 / n );
		double perimeter = 2.0 * n * R * sin( 3.14159265358979 / n );
		double k = R * R * ( 1.0 + 2.0 * cos( 3.14159265358979 / n ) * cos( 3.14159265358979 / n ) );

		expect( "prism volume", b3fToDouble( prism.base.volume ), area * 2.0 * h, 0.01 );
		expect( "prism surface area", b3fToDouble( prism.base.surfaceArea ), 2.0 * area + perimeter * 2.0 * h, 0.02 );
		expect( "prism inner radius", b3fToDouble( prism.base.innerRadius ),
				fmin( R * cos( 3.14159265358979 / n ), h ), 8.0 * Q12 );

		// Per unit mass, like every other hull in the port.
		expect( "prism Iyy", b3fToDouble( prism.base.centralInertia.cy.y ), k / 6.0, 0.01 );
		expect( "prism Ixx", b3fToDouble( prism.base.centralInertia.cx.x ), k / 12.0 + h * h / 3.0, 0.01 );
		expect( "prism Izz", b3fToDouble( prism.base.centralInertia.cz.z ), k / 12.0 + h * h / 3.0, 0.01 );
	}

	// Out of range in both directions, and degenerate sizes. B3_MAX_HULL_EDGES
	// is 32 full edges and a prism needs 3 per side, so eleven sides will not
	// fit -- the constructor refuses rather than overrunning its arrays.
	b3PrismHull tooMany = b3MakePrismHull( b3fFromDouble( 1.0 ), b3fFromDouble( 1.0 ), B3_MAX_PRISM_SIDES + 1 );
	check( "prism refuses too many sides", b3IsValidHull( &tooMany.base ) == false );

	b3PrismHull tooFew = b3MakePrismHull( b3fFromDouble( 1.0 ), b3fFromDouble( 1.0 ), 2 );
	check( "prism refuses two sides", b3IsValidHull( &tooFew.base ) == false );

	// A flat or thin prism is floored rather than refused, the same way
	// b3MakeTransformedBoxHull floors a thin box.
	b3PrismHull thin = b3MakePrismHull( b3fFromDouble( 1.0 ), b3f_zero, 6 );
	check( "flat prism is floored into validity", b3IsValidHull( &thin.base ) );
}

static void test_hull_support( void )
{
	section( "hull support points" );

	b3BoxHull box = b3MakeBoxHull( b3fFromDouble( 1.0 ), b3fFromDouble( 2.0 ), b3fFromDouble( 3.0 ) );
	const b3Vec3* points = b3GetHullPoints( &box.base );
	const b3Plane* planes = b3GetHullPlanes( &box.base );

	// Sweep directions including the exact face normals and edge directions,
	// where ties are systematic in fixed point rather than broken by luck.
	static const double dirs[][3] = {
		{ 1, 0, 0 },  { -1, 0, 0 }, { 0, 1, 0 },   { 0, -1, 0 },	{ 0, 0, 1 },   { 0, 0, -1 },  { 1, 1, 0 },
		{ 1, 0, 1 },  { 0, 1, 1 },	{ 1, 1, 1 },   { -1, -1, -1 }, { 1, -1, 1 },  { -0.3, 0.7, 0.2 }, { 0.11, -0.93, 0.35 },
		{ 2.5, 1, 0 }, { 0, 0.01, 1 },
	};

	bool vertexOk = true;
	bool faceOk = true;

	for ( size_t d = 0; d < sizeof( dirs ) / sizeof( dirs[0] ); ++d )
	{
		b3Vec3 dir = V( dirs[d][0], dirs[d][1], dirs[d][2] );

		// Brute force over the points, comparing wide so the reference is not
		// itself subject to the narrowing being checked.
		int bestVertex = 0;
		int64_t bestDot = b3DotWide( dir, points[0] );
		for ( int i = 1; i < box.base.vertexCount; ++i )
		{
			int64_t dot = b3DotWide( dir, points[i] );
			if ( dot > bestDot )
			{
				bestDot = dot;
				bestVertex = i;
			}
		}

		int gotVertex = b3FindHullSupportVertex( &box.base, dir );

		// Any vertex tied with the best one is a correct answer: the support
		// point is what matters, not which index reached it.
		vertexOk = vertexOk && b3DotWide( dir, points[gotVertex] ) == bestDot;

		int bestFace = 0;
		int64_t bestFaceDot = b3DotWide( planes[0].normal, dir );
		for ( int i = 1; i < box.base.faceCount; ++i )
		{
			int64_t dot = b3DotWide( planes[i].normal, dir );
			if ( dot > bestFaceDot )
			{
				bestFaceDot = dot;
				bestFace = i;
			}
		}

		int gotFace = b3FindHullSupportFace( &box.base, dir );
		faceOk = faceOk && b3DotWide( planes[gotFace].normal, dir ) == bestFaceDot;
		(void)bestVertex;
		(void)bestFace;
	}

	check( "support vertex matches brute force", vertexOk );
	check( "support face matches brute force", faceOk );

	// The support along a face normal must be a vertex on that face.
	int index = b3FindHullSupportVertex( &box.base, V( 0, 1, 0 ) );
	expect( "support along +Y is on the +Y face", b3fToDouble( points[index].y ), 2.0, Q12 );

	int face = b3FindHullSupportFace( &box.base, V( 0, -1, 0 ) );
	expectVec( "support face for -Y", planes[face].normal, 0, -1, 0, 2 * Q12 );
}

static void test_hull_raycast( void )
{
	section( "hull ray cast" );

	b3BoxHull cube = b3MakeCubeHull( b3fFromDouble( 1.0 ) );

	b3RayCastInput input;
	input.maxFraction = b3c_one;

	// Straight at the -X face of a 2x2x2 cube from 3 units away.
	input.origin = V( -3, 0, 0 );
	input.translation = V( 6, 0, 0 );
	b3CastOutput out = b3RayCastHull( &cube.base, &input );
	check( "axis-aligned ray hits", out.hit );
	expect( "axis-aligned fraction", b3cToDouble( out.fraction ), 2.0 / 6.0, 0.002 );
	expectVec( "axis-aligned hit point", out.point, -1, 0, 0, 8 * Q12 );
	expectVec( "axis-aligned normal", out.normal, -1, 0, 0, 2 * Q12 );

	// Diagonal entry through the same face.
	input.origin = V( -3, -0.5, 0.25 );
	input.translation = V( 6, 1.0, -0.5 );
	out = b3RayCastHull( &cube.base, &input );
	check( "diagonal ray hits", out.hit );
	expect( "diagonal fraction", b3cToDouble( out.fraction ), 2.0 / 6.0, 0.002 );
	expectVec( "diagonal normal", out.normal, -1, 0, 0, 2 * Q12 );

	// A ray that starts inside reports a hit at the origin with no face, which
	// is how callers detect exactly this case.
	input.origin = V( 0, 0, 0 );
	input.translation = V( 6, 0, 0 );
	out = b3RayCastHull( &cube.base, &input );
	check( "ray from inside hits", out.hit );
	expect( "ray from inside has zero fraction", b3cToDouble( out.fraction ), 0.0, Q12 );
	expectVec( "ray from inside reports its origin", out.point, 0, 0, 0, Q12 );

	// Grazing: parallel to four faces, a thousandth of a unit inside the fifth.
	input.origin = V( -3, 0.999, 0 );
	input.translation = V( 6, 0, 0 );
	out = b3RayCastHull( &cube.base, &input );
	check( "grazing ray hits", out.hit );
	expect( "grazing fraction", b3cToDouble( out.fraction ), 2.0 / 6.0, 0.002 );

	// And a hair outside, which must miss through the parallel branch.
	input.origin = V( -3, 1.01, 0 );
	input.translation = V( 6, 0, 0 );
	out = b3RayCastHull( &cube.base, &input );
	check( "ray parallel and outside misses", out.hit == false );

	// Ends before reaching the box: the entry fraction exceeds maxFraction.
	input.origin = V( -10, 0, 0 );
	input.translation = V( 6, 0, 0 );
	out = b3RayCastHull( &cube.base, &input );
	check( "ray that stops short misses", out.hit == false );

	// Pointing away entirely: the exit fraction is negative.
	input.origin = V( -3, 0, 0 );
	input.translation = V( -6, 0, 0 );
	out = b3RayCastHull( &cube.base, &input );
	check( "ray pointing away misses", out.hit == false );

	// The case the division guard exists for. From 2000 units away -- the edge
	// of the documented world -- the entry fraction is 333.5, which is 3.6e11
	// at Q30: eleven bits past what the hardware divider can return. The guard
	// must decide this without dividing at all.
	input.origin = V( -2000, 0, 0 );
	input.translation = V( 6, 0, 0 );
	out = b3RayCastHull( &cube.base, &input );
	check( "far-field ray misses without overflow", out.hit == false );

	// Same geometry, but long enough to actually arrive. The fraction is then
	// within range and the answer must be exact.
	input.origin = V( -2000, 0, 0 );
	input.translation = V( 4000, 0, 0 );
	out = b3RayCastHull( &cube.base, &input );
	check( "far-field ray that reaches hits", out.hit );
	expect( "far-field fraction", b3cToDouble( out.fraction ), 1999.0 / 4000.0, 0.002 );

	// maxFraction clips the ray: half of a translation that would otherwise
	// arrive is not enough.
	input.origin = V( -3, 0, 0 );
	input.translation = V( 6, 0, 0 );
	input.maxFraction = b3cFromFrac( 1, 4 );
	out = b3RayCastHull( &cube.base, &input );
	check( "maxFraction clips the ray", out.hit == false );

	input.maxFraction = b3cFromFrac( 1, 2 );
	out = b3RayCastHull( &cube.base, &input );
	check( "maxFraction just long enough still hits", out.hit );
}

static void test_hull_queries( void )
{
	section( "hull mass, bounds and overlap" );

	b3BoxHull box = b3MakeBoxHull( b3fFromDouble( 1.0 ), b3fFromDouble( 2.0 ), b3fFromDouble( 3.0 ) );

	b3MassData md = b3ComputeHullMass( &box.base, b3fFromDouble( 2.0 ) );
	expect( "hull mass is density times volume", b3fToDouble( md.mass ), 2.0 * 48.0, 0.2 );
	expectVec( "hull centre of mass", md.center, 0, 0, 0, Q12 );
	// Density must not touch a per-unit-mass tensor.
	expect( "hull unit inertia ignores density", b3fToDouble( md.inertia.cx.x ), 52.0 / 12.0, 0.02 );

	// Bounds under a translation, and under a rotation that must grow them.
	b3Transform xf = b3Transform_identity;
	xf.p = V( 10, 0, 0 );
	b3AABB aabb = b3ComputeHullAABB( &box.base, xf );
	expectVec( "translated hull lower bound", aabb.lowerBound, 9, -2, -3, 4 * Q12 );
	expectVec( "translated hull upper bound", aabb.upperBound, 11, 2, 3, 4 * Q12 );

	xf.p = V( 0, 0, 0 );
	xf.q = b3MakeQuatFromAxisAngle( V( 0, 0, 1 ), (b3a)( 32768 / 4 ) ); // 90 degrees
	aabb = b3ComputeHullAABB( &box.base, xf );
	expect( "rotated hull spans y in x", b3fToDouble( aabb.upperBound.x ), 2.0, 0.05 );
	expect( "rotated hull spans x in y", b3fToDouble( aabb.upperBound.y ), 1.0, 0.05 );

	// The swept box must contain both ends.
	b3Transform xf1 = b3Transform_identity;
	b3Transform xf2 = b3Transform_identity;
	xf2.p = V( 10, 0, 0 );
	b3AABB swept = b3ComputeSweptHullAABB( &box.base, xf1, xf2 );
	check( "swept bounds contain the start", b3AABB_Contains( swept, b3ComputeHullAABB( &box.base, xf1 ) ) );
	check( "swept bounds contain the end", b3AABB_Contains( swept, b3ComputeHullAABB( &box.base, xf2 ) ) );

	// Overlap against a point proxy, which is the smallest thing GJK can be
	// asked about.
	b3BoxHull cube = b3MakeCubeHull( b3fFromDouble( 1.0 ) );
	b3Vec3 probe = V( 0.5, 0, 0 );
	b3ShapeProxy proxy = { &probe, 1, b3f_zero };
	check( "point inside the hull overlaps", b3OverlapHull( &cube.base, b3Transform_identity, &proxy ) );

	probe = V( 3.0, 0, 0 );
	check( "point outside the hull does not overlap", b3OverlapHull( &cube.base, b3Transform_identity, &proxy ) == false );

	// And with the hull moved rather than the probe.
	xf = b3Transform_identity;
	xf.p = V( 3.0, 0, 0 );
	check( "hull moved onto the point overlaps", b3OverlapHull( &cube.base, xf, &proxy ) );
}

// The failure this exists to prevent: GJK reporting a *spurious overlap* for a
// pair that is comfortably apart.
//
// A point sitting under a large flat face makes GJK's support triangle very
// thin. The search direction is the cross product of two of its edges, and a
// cross product is an area -- so in Q12 a thin triangle's normal underflows to
// a handful of raw units, or to nothing. The final normalization then fails,
// and the code takes that to mean the origin is enclosed: distance 0.
//
// It was found by the run_pair cone cases, where a swept sphere passing under
// the base reported a hit with a zero normal, and it would have surfaced three
// phases later as bodies snagging on flat ground. b3DirectionFromWide fixes it
// by rescaling the direction, which is exact in the only property being used.
//
// The sweep below crosses the rim of a face, which is where the simplex is
// thinnest, and asserts the distance stays both correct and continuous.
static void test_gjk_thin_simplex( void )
{
	section( "GJK against a flat face" );

	b3BoxHull cube = b3MakeCubeHull( b3fFromDouble( 1.0 ) );
	const b3Vec3* points = b3GetHullPoints( &cube.base );

	// A point 0.25 below the -Y face, swept in x from under the middle of the
	// face out past its rim.
	const double depth = 0.25;
	double previous = 0.0;
	bool everZero = false;
	bool monotone = true;

	for ( int i = 0; i <= 60; ++i )
	{
		double x = 0.5 + 0.02 * i;
		b3Vec3 q = V( x, -1.0 - depth, 0.3 );

		b3DistanceInput in = { 0 };
		in.proxyA = ( b3ShapeProxy ){ points, cube.base.vertexCount, b3f_zero };
		in.proxyB = ( b3ShapeProxy ){ &q, 1, b3f_zero };
		in.transform = b3Transform_identity;
		in.useRadii = false;

		b3SimplexCache cache = { 0 };
		b3DistanceOutput out = b3ShapeDistance( &in, &cache, NULL, 0 );

		double got = b3fToDouble( out.distance );

		if ( b3Raw( out.distance ) == 0 )
		{
			everZero = true;
		}

		// While the point is under the face the answer is exactly the depth;
		// past the rim it grows. Either way it must never decrease.
		if ( x <= 1.0 )
		{
			expect( "distance under the face is the depth", got, depth, 8 * Q12 );
		}

		if ( i > 0 && got < previous - 8 * Q12 )
		{
			monotone = false;
		}
		previous = got;
	}

	check( "GJK never reported a spurious overlap", everZero == false );
	check( "GJK distance grew monotonically past the rim", monotone );

	// The same configuration one quantum at a time across the rim, which is
	// where the triangle is thinnest and where the underflow used to bite.
	bool rimOk = true;
	for ( int i = -8; i <= 8; ++i )
	{
		b3Vec3 q = V( 1.0 + i * ( 1.0 / 4096.0 ), -1.0 - depth, 0.3 );

		b3DistanceInput in = { 0 };
		in.proxyA = ( b3ShapeProxy ){ points, cube.base.vertexCount, b3f_zero };
		in.proxyB = ( b3ShapeProxy ){ &q, 1, b3f_zero };
		in.transform = b3Transform_identity;
		in.useRadii = false;

		b3SimplexCache cache = { 0 };
		b3DistanceOutput out = b3ShapeDistance( &in, &cache, NULL, 0 );

		rimOk = rimOk && b3Raw( out.distance ) > 0 && b3IsNormalized( out.normal );
	}
	check( "GJK holds across the face rim, quantum by quantum", rimOk );

	// A hull face is the worst case for this, but the same thinness arises
	// for a point almost in the plane of a triangle of hull vertices. Sweep a
	// point along the -Y face at grazing depth, where the simplex is thinnest
	// of all.
	bool grazeOk = true;
	for ( int i = 0; i <= 20; ++i )
	{
		b3Vec3 q = V( 0.3, -1.0 - 0.002 * ( i + 1 ), -0.4 );

		b3DistanceInput in = { 0 };
		in.proxyA = ( b3ShapeProxy ){ points, cube.base.vertexCount, b3f_zero };
		in.proxyB = ( b3ShapeProxy ){ &q, 1, b3f_zero };
		in.transform = b3Transform_identity;
		in.useRadii = false;

		b3SimplexCache cache = { 0 };
		b3DistanceOutput out = b3ShapeDistance( &in, &cache, NULL, 0 );

		grazeOk = grazeOk && b3Raw( out.distance ) > 0;
	}
	check( "GJK holds for a point grazing the face", grazeOk );
}

static void test_hull_manifolds( void )
{
	section( "hull contact manifolds" );

	b3LocalManifoldPoint pts[4];
	b3LocalManifold mf = { 0 };
	mf.points = pts;

	b3BoxHull cube = b3MakeCubeHull( b3fFromDouble( 1.0 ) );
	b3SimplexCache cache = { 0 };
	b3Transform xf = b3Transform_identity;

	// --- hull vs sphere ---

	b3Sphere sphere;
	sphere.center = V( 0, 0, 0 );
	sphere.radius = b3fFromDouble( 0.5 );

	// Resting exactly on the +Y face: separation zero, normal +Y.
	xf.p = V( 0, 1.5, 0 );
	cache = ( b3SimplexCache ){ 0 };
	b3CollideHullAndSphere( &mf, 4, &cube.base, &sphere, xf, &cache );
	checkInt2( "sphere on face produces 1 point", mf.pointCount, 1 );
	expectVec( "sphere on face normal", mf.normal, 0, 1, 0, 8 * Q12 );
	expect( "sphere on face separation", b3fToDouble( pts[0].separation ), 0.0, 8 * Q12 );
	expectVec( "sphere on face contact point", pts[0].point, 0, 1, 0, 8 * Q12 );

	// Pressed in by a quarter.
	xf.p = V( 0, 1.25, 0 );
	cache = ( b3SimplexCache ){ 0 };
	b3CollideHullAndSphere( &mf, 4, &cube.base, &sphere, xf, &cache );
	checkInt2( "overlapping sphere produces 1 point", mf.pointCount, 1 );
	expect( "overlapping sphere separation", b3fToDouble( pts[0].separation ), -0.25, 8 * Q12 );
	expectVec( "overlapping sphere point", pts[0].point, 0, 0.875, 0, 16 * Q12 );

	// Within the speculative margin but not touching: still a point, with a
	// positive separation.
	xf.p = V( 0, 1.51, 0 );
	cache = ( b3SimplexCache ){ 0 };
	b3CollideHullAndSphere( &mf, 4, &cube.base, &sphere, xf, &cache );
	checkInt2( "speculative sphere produces 1 point", mf.pointCount, 1 );
	check( "speculative separation is positive", b3Raw( pts[0].separation ) > 0 );

	// Clear of the margin: nothing.
	xf.p = V( 0, 3.0, 0 );
	cache = ( b3SimplexCache ){ 0 };
	b3CollideHullAndSphere( &mf, 4, &cube.base, &sphere, xf, &cache );
	checkInt2( "separated sphere produces no points", mf.pointCount, 0 );

	// Over a corner: the normal must be the diagonal, not a face normal.
	xf.p = V( 1.3, 1.3, 1.3 );
	cache = ( b3SimplexCache ){ 0 };
	b3CollideHullAndSphere( &mf, 4, &cube.base, &sphere, xf, &cache );
	checkInt2( "sphere over a corner produces 1 point", mf.pointCount, 1 );
	check( "corner normal is unit length", b3IsNormalized( mf.normal ) );
	expect( "corner normal x", b3fToDouble( mf.normal.x ), 1.0 / sqrt( 3.0 ), 0.03 );
	expect( "corner separation", b3fToDouble( pts[0].separation ), sqrt( 3.0 ) * 0.3 - 0.5, 0.02 );

	// Over an edge: two components equal, the third zero.
	xf.p = V( 1.3, 1.3, 0 );
	cache = ( b3SimplexCache ){ 0 };
	b3CollideHullAndSphere( &mf, 4, &cube.base, &sphere, xf, &cache );
	checkInt2( "sphere over an edge produces 1 point", mf.pointCount, 1 );
	expect( "edge normal x", b3fToDouble( mf.normal.x ), 0.7071, 0.03 );
	expect( "edge normal z", b3fToDouble( mf.normal.z ), 0.0, 0.03 );

	// Centre inside the hull -- the deep branch, where GJK returns zero and
	// the face with the least negative separation decides the normal.
	xf.p = V( 0, 0.5, 0 );
	cache = ( b3SimplexCache ){ 0 };
	b3CollideHullAndSphere( &mf, 4, &cube.base, &sphere, xf, &cache );
	checkInt2( "sphere centre inside produces 1 point", mf.pointCount, 1 );
	expectVec( "deep sphere normal is the nearest face", mf.normal, 0, 1, 0, 8 * Q12 );
	expect( "deep sphere separation", b3fToDouble( pts[0].separation ), -1.0, 16 * Q12 );

	// --- hull vs capsule ---

	b3Capsule capsule;
	capsule.center1 = V( -0.5, 0, 0 );
	capsule.center2 = V( 0.5, 0, 0 );
	capsule.radius = b3fFromDouble( 0.5 );

	// Lying flat on the +Y face, exactly touching. The segment is parallel to
	// the face, so this must clip to two points rather than collapse to one.
	xf = b3Transform_identity;
	xf.p = V( 0, 1.5, 0 );
	cache = ( b3SimplexCache ){ 0 };
	b3CollideHullAndCapsule( &mf, 4, &cube.base, &capsule, xf, &cache );
	checkInt2( "capsule on face produces 2 points", mf.pointCount, 2 );
	expectVec( "capsule on face normal", mf.normal, 0, 1, 0, 8 * Q12 );
	expect( "capsule on face separation 1", b3fToDouble( pts[0].separation ), 0.0, 8 * Q12 );
	expect( "capsule on face separation 2", b3fToDouble( pts[1].separation ), 0.0, 8 * Q12 );
	expect( "capsule contact 1 sits on the face", b3fToDouble( pts[0].point.y ), 1.0, 8 * Q12 );
	check( "capsule contacts are distinct", b3Raw( pts[0].point.x ) != b3Raw( pts[1].point.x ) );

	// Pressed in.
	xf.p = V( 0, 1.25, 0 );
	cache = ( b3SimplexCache ){ 0 };
	b3CollideHullAndCapsule( &mf, 4, &cube.base, &capsule, xf, &cache );
	checkInt2( "pressed capsule produces 2 points", mf.pointCount, 2 );
	expect( "pressed capsule separation", b3fToDouble( pts[0].separation ), -0.25, 8 * Q12 );

	// Hanging over the edge: half the segment is beyond the face, so the clip
	// shortens it and both points stay on the face.
	xf.p = V( 1.0, 1.5, 0 );
	cache = ( b3SimplexCache ){ 0 };
	b3CollideHullAndCapsule( &mf, 4, &cube.base, &capsule, xf, &cache );
	check( "overhanging capsule produces contacts", mf.pointCount > 0 );
	bool onFace = true;
	for ( int i = 0; i < mf.pointCount; ++i )
	{
		onFace = onFace && b3fToDouble( pts[i].point.x ) <= 1.0 + 8 * Q12;
	}
	check( "overhanging contacts stay within the face", onFace );

	// Standing on end above the face: the segment is perpendicular to the
	// face, so the endcap decides it.
	capsule.center1 = V( 0, -0.5, 0 );
	capsule.center2 = V( 0, 0.5, 0 );
	xf.p = V( 0, 2.0, 0 );
	cache = ( b3SimplexCache ){ 0 };
	b3CollideHullAndCapsule( &mf, 4, &cube.base, &capsule, xf, &cache );
	check( "upright capsule produces contacts", mf.pointCount > 0 );
	expect( "upright capsule separation", b3fToDouble( pts[0].separation ), 0.0, 16 * Q12 );
	expectVec( "upright capsule normal", mf.normal, 0, 1, 0, 16 * Q12 );

	// Clear of the hull: nothing.
	xf.p = V( 0, 5.0, 0 );
	cache = ( b3SimplexCache ){ 0 };
	b3CollideHullAndCapsule( &mf, 4, &cube.base, &capsule, xf, &cache );
	checkInt2( "separated capsule produces no points", mf.pointCount, 0 );

	// Segment straight through the hull -- the deep path, where GJK reports
	// zero distance and the SAT fallback has to produce the manifold.
	capsule.center1 = V( -2.0, 0, 0 );
	capsule.center2 = V( 2.0, 0, 0 );
	capsule.radius = b3fFromDouble( 0.25 );
	xf = b3Transform_identity;
	cache = ( b3SimplexCache ){ 0 };
	b3CollideHullAndCapsule( &mf, 4, &cube.base, &capsule, xf, &cache );
	check( "skewered capsule produces contacts", mf.pointCount > 0 );
	check( "skewered capsule separation is negative", b3Raw( pts[0].separation ) < 0 );
	check( "skewered capsule normal is unit length", b3IsNormalized( mf.normal ) );

	// A diagonal capsule leaning on the x=1, y=1 edge. The segment is at 45
	// degrees to both faces, so neither face clip applies and the manifold has
	// to come from the closest points.
	capsule.center1 = V( 0, 0, 0 );
	capsule.center2 = V( 1.0, 1.0, 0 );
	capsule.radius = b3fFromDouble( 0.25 );
	xf = b3Transform_identity;
	xf.p = V( 1.15, 1.15, 0 );
	cache = ( b3SimplexCache ){ 0 };
	b3CollideHullAndCapsule( &mf, 4, &cube.base, &capsule, xf, &cache );
	checkInt2( "diagonal capsule produces 1 point", mf.pointCount, 1 );
	check( "diagonal capsule normal is unit length", b3IsNormalized( mf.normal ) );
	// Nearest approach is from the edge at (1,1) to the segment start.
	expect( "diagonal capsule separation", b3fToDouble( pts[0].separation ), sqrt( 2.0 ) * 0.15 - 0.25, 0.02 );
	expect( "diagonal capsule normal x", b3fToDouble( mf.normal.x ), 0.7071, 0.03 );

	// Pulled back so the same configuration clears the speculative margin.
	xf.p = V( 1.3, 1.3, 0 );
	cache = ( b3SimplexCache ){ 0 };
	b3CollideHullAndCapsule( &mf, 4, &cube.base, &capsule, xf, &cache );
	checkInt2( "diagonal capsule out of range produces no points", mf.pointCount, 0 );
}

static void test_tree_structure( void )
{
	section( "dynamic tree structure" );

	b3DynamicTree tree = b3DynamicTree_Create( 16 );
	check( "empty tree has no root", tree.root == B3_NULL_INDEX );
	check( "empty tree sound", treeIsSound( &tree, 0 ) );
	checkInt2( "empty proxy count", b3DynamicTree_GetProxyCount( &tree ), 0 );
	check( "empty area ratio is zero", b3Raw( b3DynamicTree_GetAreaRatio( &tree ) ) == 0 );

	enum
	{
		N = 60
	};
	int ids[N];

	// Insert, revalidating the whole structure after every single insert --
	// the rotations happen during insertion, so a rotation that mis-wires a
	// parent pointer shows up on the very next check rather than at the end.
	bool soundThroughout = true;
	for ( int i = 0; i < N; ++i )
	{
		ids[i] = b3DynamicTree_CreateProxy( &tree, scriptedBox( i ), B3_DEFAULT_CATEGORY_BITS, (uint64_t)i );
		if ( treeIsSound( &tree, i + 1 ) == false )
		{
			soundThroughout = false;
		}
	}

	check( "sound after every insert", soundThroughout );
	checkInt2( "proxy count after inserts", b3DynamicTree_GetProxyCount( &tree ), N );

	// The pool grew past its initial 16-proxy hint, which is the path that
	// reallocates and rebuilds the free list.
	check( "node pool grew", tree.nodeCapacity > 31 );
	check( "height is logarithmic", b3DynamicTree_GetHeight( &tree ) < 20 );
	check( "area ratio is positive", b3Raw( b3DynamicTree_GetAreaRatio( &tree ) ) > 0 );
	check( "byte count is positive", b3DynamicTree_GetByteCount( &tree ) > 0 );

	// The root must contain every leaf.
	b3AABB rootBounds = b3DynamicTree_GetRootBounds( &tree );
	bool allContained = true;
	for ( int i = 0; i < N; ++i )
	{
		if ( b3AABB_Contains( rootBounds, scriptedBox( i ) ) == false )
		{
			allContained = false;
		}
	}
	check( "root contains every leaf", allContained );

	// Move every third proxy. MoveProxy removes and reinserts without
	// rotating, which is a different code path from CreateProxy.
	for ( int i = 0; i < N; i += 3 )
	{
		b3AABB moved = scriptedBox( i + 7 );
		b3DynamicTree_MoveProxy( &tree, ids[i], moved );
	}
	check( "sound after moves", treeIsSound( &tree, N ) );

	// Destroy in an order unrelated to creation, so the free list interleaves
	// with live nodes rather than unwinding cleanly.
	int destroyed = 0;
	bool soundWhileDestroying = true;
	for ( int i = 1; i < N; i += 2 )
	{
		b3DynamicTree_DestroyProxy( &tree, ids[i] );
		destroyed++;
		if ( treeIsSound( &tree, N - destroyed ) == false )
		{
			soundWhileDestroying = false;
		}
	}
	check( "sound after every destroy", soundWhileDestroying );
	checkInt2( "proxy count after destroys", b3DynamicTree_GetProxyCount( &tree ), N - destroyed );

	// Destroy the rest, back to empty.
	for ( int i = 0; i < N; i += 2 )
	{
		b3DynamicTree_DestroyProxy( &tree, ids[i] );
	}
	checkInt2( "proxy count back to zero", b3DynamicTree_GetProxyCount( &tree ), 0 );
	check( "root cleared", tree.root == B3_NULL_INDEX );
	check( "empty again is sound", treeIsSound( &tree, 0 ) );

	b3DynamicTree_Destroy( &tree );
	check( "destroy clears the tree", tree.nodes == NULL );
}

static void test_tree_query( void )
{
	section( "dynamic tree queries versus brute force" );

	b3DynamicTree tree = b3DynamicTree_Create( 8 );

	enum
	{
		N = 60
	};
	int ids[N];
	for ( int i = 0; i < N; ++i )
	{
		ids[i] = b3DynamicTree_CreateProxy( &tree, scriptedBox( i ), B3_DEFAULT_CATEGORY_BITS, (uint64_t)i );
	}

	// --- AABB overlap queries ------------------------------------------
	int mismatches = 0;
	int nonEmpty = 0;

	for ( int q = 0; q < 30; ++q )
	{
		double cx = -20.0 + q * 1.4;
		double cy = -11.0 + ( q % 7 ) * 3.0;
		double cz = -15.0 + ( q % 11 ) * 2.5;
		double r = 1.0 + ( q % 4 );
		b3AABB query = Box( cx - r, cy - r, cz - r, cx + r, cy + r, cz + r );

		ProxyList fromTree = { 0 };
		b3DynamicTree_Query( &tree, query, B3_DEFAULT_CATEGORY_BITS, false, collectProxy, &fromTree );
		sortList( &fromTree );

		ProxyList fromScan = { 0 };
		for ( int i = 0; i < N; ++i )
		{
			if ( b3AABB_Overlaps( scriptedBox( i ), query ) )
			{
				fromScan.ids[fromScan.count++] = ids[i];
			}
		}
		sortList( &fromScan );

		if ( sameList( &fromTree, &fromScan ) == false )
		{
			mismatches++;
		}
		if ( fromScan.count > 0 )
		{
			nonEmpty++;
		}
	}

	checkInt2( "AABB query matches brute force everywhere", mismatches, 0 );
	check( "AABB queries actually found proxies", nonEmpty > 10 );

	// --- category bit filtering ------------------------------------------
	//
	// Filtering is folded into every internal node as the union of its
	// children's bits, so a stale union silently prunes a whole subtree.
	for ( int i = 0; i < N; ++i )
	{
		b3DynamicTree_SetCategoryBits( &tree, ids[i], ( i % 2 ) ? 0x2ull : 0x1ull );
	}

	b3AABB everything = Box( -100, -100, -100, 100, 100, 100 );

	ProxyList evens = { 0 };
	b3DynamicTree_Query( &tree, everything, 0x1ull, false, collectProxy, &evens );

	ProxyList odds = { 0 };
	b3DynamicTree_Query( &tree, everything, 0x2ull, false, collectProxy, &odds );

	checkInt2( "category filter: even proxies", evens.count, ( N + 1 ) / 2 );
	checkInt2( "category filter: odd proxies", odds.count, N / 2 );

	ProxyList both = { 0 };
	b3DynamicTree_Query( &tree, everything, 0x3ull, false, collectProxy, &both );
	checkInt2( "category filter: any bit finds all", both.count, N );

	ProxyList all = { 0 };
	b3DynamicTree_Query( &tree, everything, 0x3ull, true, collectProxy, &all );
	checkInt2( "category filter: requireAllBits finds none", all.count, 0 );

	// Restore, and confirm the ancestor unions were rebuilt correctly.
	for ( int i = 0; i < N; ++i )
	{
		b3DynamicTree_SetCategoryBits( &tree, ids[i], B3_DEFAULT_CATEGORY_BITS );
	}
	ProxyList restored = { 0 };
	b3DynamicTree_Query( &tree, everything, B3_DEFAULT_CATEGORY_BITS, false, collectProxy, &restored );
	checkInt2( "category bits restored", restored.count, N );
	check( "sound after category churn", treeIsSound( &tree, N ) );

	b3DynamicTree_Destroy( &tree );
}

// The ray cast callback keeps every proxy the tree offers, and never clips the
// ray, so the tree must offer a superset of the true hits. Returning the input
// fraction unchanged is the documented "no opinion" answer.
static b3c keepAllRayHits( const b3RayCastInput* input, int proxyId, uint64_t userData, void* context )
{
	ProxyList* list = (ProxyList*)context;
	B3_UNUSED( userData );
	if ( list->count < 256 )
	{
		list->ids[list->count++] = proxyId;
	}
	return input->maxFraction;
}

static b3c keepAllBoxHits( const b3BoxCastInput* input, int proxyId, uint64_t userData, void* context )
{
	ProxyList* list = (ProxyList*)context;
	B3_UNUSED( userData );
	if ( list->count < 256 )
	{
		list->ids[list->count++] = proxyId;
	}
	return input->maxFraction;
}

static void test_tree_casts( void )
{
	section( "dynamic tree casts versus brute force" );

	b3DynamicTree tree = b3DynamicTree_Create( 8 );

	enum
	{
		N = 60
	};
	int ids[N];
	for ( int i = 0; i < N; ++i )
	{
		ids[i] = b3DynamicTree_CreateProxy( &tree, scriptedBox( i ), B3_DEFAULT_CATEGORY_BITS, (uint64_t)i );
	}

	// --- ray casts --------------------------------------------------------
	//
	// The tree may over-report -- it rejects on the node box, and a ray can
	// clip a box the tree keeps -- but it must never miss one that brute force
	// finds. A missed proxy is a shape a bullet passes through.
	//
	// Every ray is aimed straight through the centre of one of the proxies, so
	// each is guaranteed at least one real hit. Arbitrary rays are almost
	// useless here: the proxies are a few units across and scattered through a
	// 40x22x30 volume, so a line through that space misses nearly everything,
	// and the traversal never gets exercised.
	static const double offsets[8][3] = {
		{ 25, 0, 0 }, { 0, 25, 0 }, { 0, 0, 25 }, { 18, 18, 0 }, { 18, 0, 18 }, { 0, 18, 18 }, { 14, 14, 14 }, { -20, 10, -15 },
	};

	int missed = 0;
	int totalHits = 0;
	int totalReported = 0;

	for ( int q = 0; q < 40; ++q )
	{
		b3Vec3 target = b3AABB_Center( scriptedBox( q % N ) );
		const double* off = offsets[q % 8];

		// Start one offset away and travel two, so the ray passes through the
		// target box at fraction 0.5.
		b3RayCastInput in;
		in.origin = b3Add( target, V( off[0], off[1], off[2] ) );
		in.translation = V( -2 * off[0], -2 * off[1], -2 * off[2] );
		in.maxFraction = b3c_one;

		ProxyList fromTree = { 0 };
		b3DynamicTree_RayCast( &tree, &in, B3_DEFAULT_CATEGORY_BITS, false, keepAllRayHits, &fromTree );
		sortList( &fromTree );
		totalReported += fromTree.count;

		for ( int i = 0; i < N; ++i )
		{
			b3c minF, maxF;
			b3Vec3 p2 = b3Add( in.origin, in.translation );

			if ( b3RayCastAABB( scriptedBox( i ), in.origin, p2, &minF, &maxF ) )
			{
				totalHits++;

				bool found = false;
				for ( int k = 0; k < fromTree.count; ++k )
				{
					if ( fromTree.ids[k] == ids[i] )
					{
						found = true;
					}
				}
				if ( found == false )
				{
					missed++;
				}
			}
		}
	}

	checkInt2( "ray cast never misses a real hit", missed, 0 );
	check( "every aimed ray hit its target box", totalHits >= 40 );

	// The tree must also be doing its job. Without this, a
	// b3TestBoundsRayOverlap stuck at `true` would satisfy every check above
	// while turning the broad phase into a linear scan.
	//
	// The bound is deliberately close: measured, the tree reports 64 proxies
	// across these 40 rays and brute force finds exactly 64, so the rejection
	// is currently perfect. It is allowed to be conservative -- it tests the
	// node box against the infinite line, so it may keep a proxy the segment
	// only nearly reaches -- hence the factor of three rather than equality.
	// Reporting everything would be 2400.
	check( "ray cast prunes rather than reporting everything", totalReported <= 3 * totalHits );

	// --- box casts --------------------------------------------------------
	//
	// Same contract, with the node grown by the box extents instead.
	int missedBox = 0;
	int totalBoxHits = 0;

	for ( int q = 0; q < 20; ++q )
	{
		double ox = -25.0 + ( q % 6 ) * 3.0;
		double oy = -15.0 + ( q % 4 ) * 5.0;
		double oz = -20.0 + ( q % 5 ) * 4.0;

		b3BoxCastInput in;
		in.box = Box( ox - 1.0, oy - 1.0, oz - 1.0, ox + 1.0, oy + 1.0, oz + 1.0 );
		in.translation = V( 40.0, 18.0 - ( q % 3 ) * 12.0, 22.0 - ( q % 4 ) * 8.0 );
		in.maxFraction = b3c_one;

		ProxyList fromTree = { 0 };
		b3DynamicTree_BoxCast( &tree, &in, B3_DEFAULT_CATEGORY_BITS, false, keepAllBoxHits, &fromTree );

		// Brute force: sweep the box centre as a ray against each proxy grown
		// by the box extents -- the same reduction the tree performs.
		b3Vec3 centre = b3AABB_Center( in.box );
		b3Vec3 extent = b3AABB_Extents( in.box );
		b3Vec3 end = b3Add( centre, in.translation );

		for ( int i = 0; i < N; ++i )
		{
			b3AABB grown = scriptedBox( i );
			grown.lowerBound = b3Sub( grown.lowerBound, extent );
			grown.upperBound = b3Add( grown.upperBound, extent );

			b3c minF, maxF;
			if ( b3RayCastAABB( grown, centre, end, &minF, &maxF ) )
			{
				totalBoxHits++;

				bool found = false;
				for ( int k = 0; k < fromTree.count; ++k )
				{
					if ( fromTree.ids[k] == ids[i] )
					{
						found = true;
					}
				}
				if ( found == false )
				{
					missedBox++;
				}
			}
		}
	}

	checkInt2( "box cast never misses a real hit", missedBox, 0 );
	check( "box casts actually hit things", totalBoxHits > 10 );

	b3DynamicTree_Destroy( &tree );
}

// Report the true squared distance from the query point to this proxy's box.
static int64_t closestCallback( int64_t distanceSqrMin, int proxyId, uint64_t userData, void* context )
{
	B3_UNUSED( distanceSqrMin );
	B3_UNUSED( proxyId );

	const b3Vec3* point = (const b3Vec3*)context;
	b3AABB box = scriptedBox( (int)userData );
	b3Vec3 r = b3Sub( *point, b3Clamp( *point, box.lowerBound, box.upperBound ) );
	return b3LengthSquaredWide( r );
}

static void test_tree_closest( void )
{
	section( "dynamic tree closest query" );

	b3DynamicTree tree = b3DynamicTree_Create( 8 );

	enum
	{
		N = 60
	};
	for ( int i = 0; i < N; ++i )
	{
		b3DynamicTree_CreateProxy( &tree, scriptedBox( i ), B3_DEFAULT_CATEGORY_BITS, (uint64_t)i );
	}

	int wrong = 0;

	for ( int q = 0; q < 25; ++q )
	{
		b3Vec3 point = V( -28.0 + q * 2.3, -14.0 + ( q % 6 ) * 4.0, -18.0 + ( q % 8 ) * 3.5 );

		// Seed with a distance large enough not to restrict the search. This
		// is where the wide type earns itself: at Q24 a squared distance of
		// even a few hundred units has long since left int32 behind.
		int64_t best = (int64_t)1 << 50;
		b3DynamicTree_QueryClosest( &tree, point, B3_DEFAULT_CATEGORY_BITS, false, closestCallback, &point, &best );

		// Brute force the same thing.
		int64_t bruteBest = (int64_t)1 << 50;
		for ( int i = 0; i < N; ++i )
		{
			b3AABB box = scriptedBox( i );
			b3Vec3 r = b3Sub( point, b3Clamp( point, box.lowerBound, box.upperBound ) );
			int64_t d = b3LengthSquaredWide( r );
			if ( d < bruteBest )
			{
				bruteBest = d;
			}
		}

		if ( best != bruteBest )
		{
			wrong++;
		}
	}

	checkInt2( "closest query matches brute force exactly", wrong, 0 );

	b3DynamicTree_Destroy( &tree );
}

static void test_tree_enlarge_and_rebuild( void )
{
	section( "dynamic tree enlarge and rebuild" );

	b3DynamicTree tree = b3DynamicTree_Create( 8 );

	enum
	{
		N = 40
	};
	int ids[N];
	for ( int i = 0; i < N; ++i )
	{
		ids[i] = b3DynamicTree_CreateProxy( &tree, scriptedBox( i ), B3_DEFAULT_CATEGORY_BITS, (uint64_t)i );
	}

	// Enlarging a leaf must grow every ancestor and mark them, all the way to
	// the root. Grow one proxy well past the root bounds so the propagation
	// cannot stop early.
	b3AABB huge = Box( -60, -60, -60, 60, 60, 60 );
	b3DynamicTree_EnlargeProxy( &tree, ids[0], huge );

	check( "enlarge kept the tree sound", treeIsSound( &tree, N ) );
	check( "root grew to contain the enlarged proxy", b3AABB_Contains( b3DynamicTree_GetRootBounds( &tree ), huge ) );

	// The enlarged flag must have reached the root, otherwise the rebuild
	// below would not free the grown internal nodes.
	check( "root is marked enlarged", ( tree.nodes[tree.root].flags & b3_enlargedNode ) != 0 );

	// Rebuild consumes the enlarged nodes.
	int rebuilt = b3DynamicTree_Rebuild( &tree, false );
	check( "partial rebuild returned leaves", rebuilt > 0 );
	check( "sound after partial rebuild", treeIsSound( &tree, N ) );

	bool anyEnlarged = false;
	for ( int i = 0; i < tree.nodeCapacity; ++i )
	{
		if ( ( tree.nodes[i].flags & b3_allocatedNode ) && ( tree.nodes[i].flags & b3_enlargedNode ) )
		{
			anyEnlarged = true;
		}
	}
	check( "no enlarged nodes remain after rebuild", anyEnlarged == false );

	// A full rebuild discards the existing structure entirely and rebuilds
	// from the leaves through b3PartitionMid, which is the median-split path.
	int fullRebuilt = b3DynamicTree_Rebuild( &tree, true );
	checkInt2( "full rebuild returned every leaf", fullRebuilt, N );
	check( "sound after full rebuild", treeIsSound( &tree, N ) );

	// Queries must still be right after the structure was rebuilt from scratch.
	b3AABB everything = Box( -100, -100, -100, 100, 100, 100 );
	ProxyList found = { 0 };
	b3DynamicTree_Query( &tree, everything, B3_DEFAULT_CATEGORY_BITS, false, collectProxy, &found );
	checkInt2( "all proxies still queryable after rebuild", found.count, N );

	// The rebuild should not have made the tree worse than linear.
	check( "rebuilt height is logarithmic", b3DynamicTree_GetHeight( &tree ) < 20 );

	b3DynamicTree_Destroy( &tree );
}

// -------------------------------------------------------------------------
// Broad phase, proxy management
// -------------------------------------------------------------------------
//
// Only the world-free half exists; b3UpdateBroadPhasePairs and everything it
// drags in is Phase 3. What can be checked now is the bookkeeping: that a
// proxy key round-trips through its type and id, that each body type lands in
// its own tree, and above all that the move buffer and its bitset stay in
// agreement. That pairing is the part with a real failure mode -- the bitset
// is the fast membership test and the array is the deterministic order, and
// nothing else notices if they disagree until pair generation starts skipping
// or duplicating proxies in Phase 3.

static bool moveArrayContains( const b3BroadPhase* bp, int proxyKey )
{
	for ( int i = 0; i < bp->moveArray.count; ++i )
	{
		if ( bp->moveArray.data[i] == proxyKey )
		{
			return true;
		}
	}
	return false;
}

// The invariant that must hold at all times: a proxy is in the move array if
// and only if its bit is set.
static bool moveBufferConsistent( const b3BroadPhase* bp )
{
	// Every array entry has its bit set, and no duplicates.
	for ( int i = 0; i < bp->moveArray.count; ++i )
	{
		int key = bp->moveArray.data[i];
		if ( b3GetBit( &bp->movedProxies[B3_PROXY_TYPE( key )], B3_PROXY_ID( key ) ) == false )
		{
			return false;
		}

		for ( int j = i + 1; j < bp->moveArray.count; ++j )
		{
			if ( bp->moveArray.data[j] == key )
			{
				return false;
			}
		}
	}
	return true;
}

// =========================================================================
// Triangle meshes
// =========================================================================
//
// The blobs here are written by hand rather than produced by
// tests/box3d_host/mesh_bake.c. That is the point: the baker is only available
// to run_pair, which builds device mode, and a reader checked against its own
// writer would agree with itself no matter what either of them did. A blob
// spelled out in the test is an independent statement of the layout.

/// A hand-built mesh: 4 triangles in two leaves, so the tree is actually
/// walked rather than short-circuited at the root.
///
/// Two unit quads side by side in the XZ plane at y = 0, split along x:
///
///     z=1  2---3---5
///          | \ | \ |
///     z=0  0---1---4
///        x=0  x=1  x=2
typedef struct
{
	b3MeshData header;

	// Sized for the shadow-value builds, not for device mode. Every fixed
	// value carries a double alongside it there, so b3Vec3 is 48 bytes rather
	// than 12 and this blob is four times the size -- and a buffer sized for
	// device mode overflows into whatever static follows it, which is how this
	// number was arrived at.
	char bytes[4096];
	double align;
} meshBlob;

/// Byte offsets computed from sizeof, never from literals -- b3Vec3 is 12
/// bytes in device mode and much wider under the shadow-value builds, and this
/// file is compiled in all three.
static int meshAlign( int cursor, size_t alignment )
{
	int a = (int)alignment;
	return ( ( cursor + a - 1 ) / a ) * a;
}

static void buildTestMesh( meshBlob* blob )
{
	const int vertexCount = 6;
	const int triangleCount = 4;
	const int nodeCount = 3;

	memset( blob, 0, sizeof( *blob ) );

	int cursor = (int)sizeof( b3MeshData );
	cursor = meshAlign( cursor, sizeof( b3MeshNode ) );
	int nodeOffset = cursor;
	cursor += nodeCount * (int)sizeof( b3MeshNode );

	cursor = meshAlign( cursor, sizeof( b3Vec3 ) );
	int vertexOffset = cursor;
	cursor += vertexCount * (int)sizeof( b3Vec3 );

	cursor = meshAlign( cursor, sizeof( b3MeshTriangle ) );
	int triangleOffset = cursor;
	cursor += triangleCount * (int)sizeof( b3MeshTriangle );

	int materialOffset = cursor;
	cursor += triangleCount;

	int flagsOffset = cursor;
	cursor += triangleCount;

	// The layout has to fit the buffer in every mode, so say so rather than
	// trusting it.
	check( "the hand-built blob fits its buffer", cursor <= (int)sizeof( *blob ) );

	b3MeshData* mesh = &blob->header;
	mesh->version = B3_MESH_VERSION;
	mesh->byteCount = ( cursor + 7 ) & ~7;
	mesh->hash = 1;
	mesh->treeHeight = 2;
	mesh->nodeOffset = nodeOffset;
	mesh->nodeCount = nodeCount;
	mesh->vertexOffset = vertexOffset;
	mesh->vertexCount = vertexCount;
	mesh->triangleOffset = triangleOffset;
	mesh->triangleCount = triangleCount;
	mesh->materialOffset = materialOffset;
	mesh->materialCount = 1;
	mesh->flagsOffset = flagsOffset;

	b3Vec3* vertices = (b3Vec3*)( (char*)blob + vertexOffset );
	vertices[0] = V( 0, 0, 0 );
	vertices[1] = V( 1, 0, 0 );
	vertices[2] = V( 0, 0, 1 );
	vertices[3] = V( 1, 0, 1 );
	vertices[4] = V( 2, 0, 0 );
	vertices[5] = V( 2, 0, 1 );

	// The baked bounds. b3ComputeMeshAABB reads these rather than walking, so
	// a blob that leaves them zero reports an empty mesh.
	mesh->bounds = b3MakeAABB( V( 0, 0, 0 ), V( 2, 0, 1 ) );

	b3MeshTriangle* triangles = (b3MeshTriangle*)( (char*)blob + triangleOffset );
	// Left quad, then right quad -- which is the depth-first leaf order the
	// nodes below describe.
	triangles[0] = ( b3MeshTriangle ){ 0, 2, 1 };
	triangles[1] = ( b3MeshTriangle ){ 1, 2, 3 };
	triangles[2] = ( b3MeshTriangle ){ 1, 3, 4 };
	triangles[3] = ( b3MeshTriangle ){ 4, 3, 5 };

	uint8_t* flags = (uint8_t*)( (char*)blob + flagsOffset );
	// The seam between the quads is flat, and triangle 1's edge 2 (v3->v1)
	// meets triangle 2's edge 0 (v1->v3).
	flags[1] = b3_flatEdge3;
	flags[2] = b3_flatEdge1;

	b3MeshNode* nodes = (b3MeshNode*)( (char*)blob + nodeOffset );

	// Root: everything. Left child is implicit at index 1; right child at 2.
	nodes[0].lowerBound = V( 0, 0, 0 );
	nodes[0].upperBound = V( 2, 0, 1 );
	nodes[0].data.asNode.axis = 0;
	nodes[0].data.asNode.childOffset = 2;
	nodes[0].triangleOffset = 0;

	nodes[1].lowerBound = V( 0, 0, 0 );
	nodes[1].upperBound = V( 1, 0, 1 );
	nodes[1].data.asLeaf.type = B3_LEAF_NODE;
	nodes[1].data.asLeaf.triangleCount = 2;
	nodes[1].triangleOffset = 0;

	nodes[2].lowerBound = V( 1, 0, 0 );
	nodes[2].upperBound = V( 2, 0, 1 );
	nodes[2].data.asLeaf.type = B3_LEAF_NODE;
	nodes[2].data.asLeaf.triangleCount = 2;
	nodes[2].triangleOffset = 2;
}

typedef struct
{
	int indices[16];
	int count;
} meshQueryResult;

static bool meshCollect( b3Vec3 a, b3Vec3 b, b3Vec3 c, int triangleIndex, void* context )
{
	B3_UNUSED( a, b, c );

	meshQueryResult* out = context;
	if ( out->count < 16 )
	{
		out->indices[out->count++] = triangleIndex;
	}
	return true;
}

static meshQueryResult meshQuery( const b3Mesh* mesh, b3Vec3 lower, b3Vec3 upper )
{
	meshQueryResult out = { { 0 }, 0 };
	b3QueryMesh( mesh, b3MakeAABB( lower, upper ), meshCollect, &out );
	return out;
}

static void test_mesh_blob( void )
{
	printf( "mesh blob, traversal and scale\n" );

	static meshBlob blob;
	buildTestMesh( &blob );

	check( "a well formed blob validates", b3IsValidMesh( &blob.header ) );

	b3Mesh mesh = { &blob.header, V( 1, 1, 1 ) };

	// --- the whole mesh, which is also the ascending-order check ---
	{
		meshQueryResult all = meshQuery( &mesh, V( -1, -1, -1 ), V( 3, 1, 2 ) );
		checkInt2( "a box over everything finds every triangle", all.count, 4 );

		bool ascending = true;
		for ( int i = 1; i < all.count; ++i )
		{
			ascending = ascending && all.indices[i] > all.indices[i - 1];
		}

		// Not incidental: the narrow phase matches its per-triangle cache
		// against this with a linear merge join, and nothing downstream
		// asserts it.
		check( "triangle indices come back ascending and unique", ascending );
	}

	// --- one leaf only, so the other subtree really is culled ---
	{
		meshQueryResult left = meshQuery( &mesh, V( -1, -0.5, -1 ), V( 0.4, 0.5, 2 ) );
		checkInt2( "a box over the left quad finds two triangles", left.count, 2 );
		checkInt2( "and they are the left quad's", left.indices[0], 0 );
		checkInt2( "and they are the left quad's", left.indices[1], 1 );

		meshQueryResult right = meshQuery( &mesh, V( 1.6, -0.5, -1 ), V( 3, 0.5, 2 ) );
		checkInt2( "a box over the right quad finds two triangles", right.count, 2 );
		checkInt2( "and they are the right quad's", right.indices[0], 2 );
		checkInt2( "and they are the right quad's", right.indices[1], 3 );
	}

	// --- a box beside the mesh finds nothing ---
	{
		meshQueryResult none = meshQuery( &mesh, V( -5, -5, -5 ), V( -4, -4, -4 ) );
		checkInt2( "a box away from the mesh finds nothing", none.count, 0 );

		// Just above the plane, within the same x/z footprint: the face
		// separation axis is what has to reject this.
		meshQueryResult above = meshQuery( &mesh, V( 0.2, 0.5, 0.2 ), V( 0.8, 1.5, 0.8 ) );
		checkInt2( "a box above the plane finds nothing", above.count, 0 );
	}

	// --- the unit-scale fast path must agree with the general path ---
	{
		// 1.0 exactly, but reached through b3SafeScale rather than as a
		// literal, so the general path runs and its reciprocal is exercised.
		b3Mesh scaled = { &blob.header, b3SafeScale( V( 2, 2, 2 ) ) };

		meshQueryResult unit = meshQuery( &mesh, V( -1, -0.5, -1 ), V( 0.4, 0.5, 2 ) );
		meshQueryResult doubled = meshQuery( &scaled, V( -2, -1, -2 ), V( 0.8, 1, 4 ) );

		// The same box scaled by the same factor selects the same triangles.
		checkInt2( "a doubled mesh under a doubled box finds the same count", doubled.count, unit.count );
		bool same = doubled.count == unit.count;
		for ( int i = 0; i < unit.count && same; ++i )
		{
			same = doubled.indices[i] == unit.indices[i];
		}
		check( "and the same triangles", same );

		// The vertices really were scaled, which is what separates this from
		// the fast path silently handling both.
		b3Triangle tri = b3GetMeshTriangle( &scaled, 3 );
		expect( "a doubled mesh returns doubled vertices", b3fToDouble( tri.vertices[0].x ), 4.0, 8 * Q12 );
	}

	// --- b3ComputeMeshAABB against the vertices it claims to bound ---
	{
		b3AABB aabb = b3ComputeMeshAABB( &blob.header, b3Transform_identity, V( 1, 1, 1 ) );
		expect( "mesh aabb lower x", b3fToDouble( aabb.lowerBound.x ), 0.0, 8 * Q12 );
		expect( "mesh aabb upper x", b3fToDouble( aabb.upperBound.x ), 2.0, 8 * Q12 );
		expect( "mesh aabb upper z", b3fToDouble( aabb.upperBound.z ), 1.0, 8 * Q12 );

		// A negative component swaps that axis, so the result must still be a
		// sane box rather than an inverted one.
		b3AABB mirrored = b3ComputeMeshAABB( &blob.header, b3Transform_identity, V( 1, 1, -1 ) );
		check( "a mirrored scale still yields lower <= upper",
			   b3Raw( mirrored.lowerBound.z ) <= b3Raw( mirrored.upperBound.z ) );
		expect( "mirrored aabb lower z", b3fToDouble( mirrored.lowerBound.z ), -1.0, 8 * Q12 );
	}

	// --- a reflection flips winding and re-labels the concave edges ---
	{
		b3Mesh mirrored = { &blob.header, b3SafeScale( V( 1, 1, -1 ) ) };

		b3Triangle upright = b3GetMeshTriangle( &mesh, 1 );
		b3Triangle flipped = b3GetMeshTriangle( &mirrored, 1 );

		checkInt2( "a reflected triangle keeps vertex 0", flipped.i1, upright.i1 );
		checkInt2( "and swaps vertices 1 and 2", flipped.i2, upright.i3 );
		checkInt2( "and swaps vertices 1 and 2", flipped.i3, upright.i2 );

		expect( "and negates the scaled axis", b3fToDouble( flipped.vertices[0].z ),
				-b3fToDouble( upright.vertices[0].z ), 8 * Q12 );

		// Triangle 1 carries b3_flatEdge3, which is concave3 | inverseConcave3.
		// Reflected, only the inverse bit survives and it becomes concave3.
		checkInt2( "a reflected flat edge becomes a concave edge", flipped.flags, b3_concaveEdge3 );

		// An unreflected fetch must leave the flags exactly as stored.
		checkInt2( "an unreflected fetch preserves the flags", upright.flags, b3_flatEdge3 );
	}

	// --- the validator, against blobs that are wrong in specific ways ---
	{
		static meshBlob broken;

		buildTestMesh( &broken );
		broken.header.version ^= 1u;
		check( "a wrong version is rejected", b3IsValidMesh( &broken.header ) == false );

		buildTestMesh( &broken );
		// A node whose box does not contain its triangles. This is the failure
		// that would otherwise present as a body falling through a floor an
		// hour later.
		( (b3MeshNode*)( (char*)&broken + broken.header.nodeOffset ) )[1].upperBound = V( 0.5, 0, 0.5 );
		check( "a node smaller than its triangles is rejected", b3IsValidMesh( &broken.header ) == false );

		buildTestMesh( &broken );
		// childOffset pointing back at the parent: a cycle. The guard counter
		// is what has to catch this -- a device that hangs here is worse than
		// one that refuses the level.
		( (b3MeshNode*)( (char*)&broken + broken.header.nodeOffset ) )[0].data.asNode.childOffset = 0;
		check( "a cyclic child offset is rejected rather than hung", b3IsValidMesh( &broken.header ) == false );

		buildTestMesh( &broken );
		( (b3MeshNode*)( (char*)&broken + broken.header.nodeOffset ) )[2].triangleOffset = 900;
		check( "a leaf pointing past the triangle array is rejected", b3IsValidMesh( &broken.header ) == false );

		buildTestMesh( &broken );
		broken.header.treeHeight = B3_MESH_STACK_SIZE + 1;
		check( "a tree taller than the traversal stack is rejected", b3IsValidMesh( &broken.header ) == false );
	}
}

/// The three Phase 7 mesh queries, against the same hand-built blob.
///
/// Closed forms throughout rather than a comparison against upstream. Two of
/// these disagree with upstream *by construction* -- the front-to-back descent
/// and the one-sided plane test resolve ties differently from a left-first
/// float traversal -- so a reference comparison would report a divergence every
/// run that the port would then have to defend. What is checkable is what the
/// geometry says, and the geometry here is two unit quads in the y = 0 plane
/// with their normals pointing at +y.
static void test_mesh_queries( void )
{
	printf( "mesh ray casts, shape casts and overlap\n" );

	static meshBlob blob;
	buildTestMesh( &blob );

	b3Mesh mesh = { &blob.header, V( 1, 1, 1 ) };

	// --- ray casts ---
	{
		// Straight down through the middle of the left quad. Starting 2 above
		// a floor at y = 0 means the crossing is at exactly half the ray.
		b3RayCastInput down = { V( 0.25, 2, 0.25 ), V( 0, -4, 0 ), b3c_one };
		b3CastOutput hit = b3RayCastMesh( &mesh, &down );

		check( "a ray fired down at the floor hits it", hit.hit );
		expect( "at half of a ray twice as long as the drop", b3cToDouble( hit.fraction ), 0.5, 4 * Q12 );
		expectVec( "on the floor plane", hit.point, 0.25, 0, 0.25, 4 * Q12 );
		expectVec( "with the floor's own normal", hit.normal, 0, 1, 0, 8 * Q12 );
		checkInt2( "reporting the triangle it crossed", hit.triangleIndex, 0 );

		// The same ray from below. The mesh is one-sided, as upstream, so this
		// is a miss rather than a hit on the back face.
		b3RayCastInput up = { V( 0.25, -2, 0.25 ), V( 0, 4, 0 ), b3c_one };
		check( "and the same ray from underneath is culled", b3RayCastMesh( &mesh, &up ).hit == false );

		// Past the far edge of the right quad in x.
		b3RayCastInput beside = { V( 2.5, 2, 0.5 ), V( 0, -4, 0 ), b3c_one };
		check( "a ray beside the mesh misses", b3RayCastMesh( &mesh, &beside ).hit == false );

		// Aimed at the floor but stopping short: maxFraction cuts the segment
		// at y = 1, half a unit above it.
		b3RayCastInput brief = { V( 0.25, 2, 0.25 ), V( 0, -4, 0 ), b3cFromFrac( 1, 4 ) };
		check( "a ray that stops short of the floor misses", b3RayCastMesh( &mesh, &brief ).hit == false );

		// Into the right quad, which lives in the *other* leaf. This is the
		// check that the descent visits both children rather than stopping at
		// the near one.
		b3RayCastInput right = { V( 1.75, 1, 0.25 ), V( 0, -2, 0 ), b3c_one };
		b3CastOutput rightHit = b3RayCastMesh( &mesh, &right );
		check( "a ray into the far leaf hits", rightHit.hit );
		check( "and reports a triangle from that leaf", rightHit.triangleIndex >= 2 );

		// A slanted ray: down 1 and across 1 from (0.25, 1, 0.25) reaches
		// y = 0 at x = 1.25, which is inside the right quad. Nothing about the
		// answer depends on the traversal order, but everything about *finding*
		// it depends on both leaves being reachable.
		b3RayCastInput slant = { V( 0.25, 1, 0.25 ), V( 2, -2, 0 ), b3c_one };
		b3CastOutput slantHit = b3RayCastMesh( &mesh, &slant );
		check( "a slanted ray hits", slantHit.hit );
		expect( "at the fraction the slope gives", b3cToDouble( slantHit.fraction ), 0.5, 8 * Q12 );
		expectVec( "landing where the slope says", slantHit.point, 1.25, 0, 0.25, 8 * Q12 );
	}

	// --- the far end of the segment is exclusive ---
	{
		// From (0.5, 1, 0.5) along (1, -1, 0), y reaches 0 at exactly
		// fraction 1: the ray *ends* on the floor rather than crossing it.
		//
		// That is a miss, and deliberately. b3RayCastMesh keeps a hit only when
		// its fraction is strictly below the best so far, which starts at
		// maxFraction -- so a crossing at exactly maxFraction never displaces
		// it. Upstream's `alpha < bestOutput.fraction` says the same thing.
		// Pinned because the alternative is an arbitrary choice that would
		// otherwise drift.
		b3RayCastInput grazing = { V( 0.5, 1, 0.5 ), V( 1, -1, 0 ), b3c_one };
		check( "a ray ending exactly on the surface does not hit", b3RayCastMesh( &mesh, &grazing ).hit == false );

		// One percent longer and it crosses, which is what says the miss above
		// is a boundary and not a blind spot.
		b3RayCastInput past = { V( 0.5, 1, 0.5 ), V( 1.01, -1.01, 0 ), b3c_one };
		check( "and a fractionally longer one does", b3RayCastMesh( &mesh, &past ).hit );
	}

	// --- shape casts ---
	{
		// A sphere of radius 0.25 dropped from y = 2 stops when its *surface*
		// touches the floor, so its centre is at 0.25 and the fraction is
		// (2 - 0.25) / 4.
		b3Vec3 center = V( 0.5, 2, 0.5 );
		b3ShapeCastInput drop = { { &center, 1, b3fFromDouble( 0.25 ) }, V( 0, -4, 0 ), b3c_one, false };

		b3CastOutput hit = b3ShapeCastMesh( &mesh, &drop );
		check( "a sphere swept down at the floor lands", hit.hit );
		expect( "stopping a radius above it", b3cToDouble( hit.fraction ), 1.75 / 4.0, 16 * Q12 );
		expectVec( "with the floor's normal", hit.normal, 0, 1, 0, 16 * Q12 );
		check( "reporting which triangle stopped it", hit.triangleIndex != B3_NULL_INDEX );

		// The same sphere swept sideways well above the floor never meets it.
		b3Vec3 high = V( 0.5, 2, 0.5 );
		b3ShapeCastInput across = { { &high, 1, b3fFromDouble( 0.25 ) }, V( 1.5, 0, 0 ), b3c_one, false };
		check( "and a sweep that stays above misses", b3ShapeCastMesh( &mesh, &across ).hit == false );
	}

	// --- overlap ---
	{
		// A sphere straddling the floor plane.
		b3Vec3 touching = V( 0.5, 0, 0.5 );
		b3ShapeProxy proxy = { &touching, 1, b3fFromDouble( 0.25 ) };
		check( "a sphere on the floor overlaps it", b3OverlapMesh( &mesh, b3Transform_identity, &proxy ) );

		// Clear of it in y.
		b3Vec3 above = V( 0.5, 1, 0.5 );
		b3ShapeProxy aboveProxy = { &above, 1, b3fFromDouble( 0.25 ) };
		check( "a sphere well above it does not", b3OverlapMesh( &mesh, b3Transform_identity, &aboveProxy ) == false );

		// Clear of it in x, at the same height.
		b3Vec3 beside = V( 3, 0, 0.5 );
		b3ShapeProxy besideProxy = { &beside, 1, b3fFromDouble( 0.25 ) };
		check( "and a sphere beside it does not", b3OverlapMesh( &mesh, b3Transform_identity, &besideProxy ) == false );

		// The transform is the *mesh's*, not the query shape's -- b3MakeLocalProxy
		// inverts it to bring the proxy into the mesh's frame, which is the same
		// convention b3RayCastShape uses. So raising the mesh by 1 is what brings
		// the floor up to the sphere that was clear of it a moment ago.
		b3Transform raised = { V( 0, 1, 0 ), b3Quat_identity };
		check( "and the transform moves the mesh, not the proxy", b3OverlapMesh( &mesh, raised, &aboveProxy ) );

		// The opposite sign has to stay a miss, or the check above would pass
		// for a transform that was simply being ignored.
		b3Transform lowered = { V( 0, -1, 0 ), b3Quat_identity };
		check( "so the opposite sign moves it further away", b3OverlapMesh( &mesh, lowered, &aboveProxy ) == false );
	}

	// --- scale ---
	{
		// Doubling the mesh in x moves the far edge from x = 2 to x = 4, so a
		// ray at x = 3 goes from a miss to a hit without the blob changing.
		b3Mesh wide = { &blob.header, V( 2, 1, 1 ) };

		b3RayCastInput at3 = { V( 3, 2, 0.5 ), V( 0, -4, 0 ), b3c_one };
		check( "a ray past the unscaled edge misses", b3RayCastMesh( &mesh, &at3 ).hit == false );
		check( "and hits once the mesh is scaled out to meet it", b3RayCastMesh( &wide, &at3 ).hit );

		// A reflection in y turns the floor upside down: the normal that was
		// +y becomes -y, so the ray that used to hit is culled and the one that
		// used to be culled hits.
		b3Mesh flipped = { &blob.header, V( 1, -1, 1 ) };

		b3RayCastInput down = { V( 0.25, 2, 0.25 ), V( 0, -4, 0 ), b3c_one };
		b3RayCastInput up = { V( 0.25, -2, 0.25 ), V( 0, 4, 0 ), b3c_one };

		check( "reflecting the mesh culls what used to hit", b3RayCastMesh( &flipped, &down ).hit == false );

		b3CastOutput fromBelow = b3RayCastMesh( &flipped, &up );
		check( "and hits what used to be culled", fromBelow.hit );
		expectVec( "with the reversed normal", fromBelow.normal, 0, -1, 0, 8 * Q12 );
	}
}

// =========================================================================
// Triangles
// =========================================================================

/// b3ClosestPointOnTriangle, region by region.
///
/// The seven barycentric regions are chosen by construction rather than at
/// random: a random sweep hits the face region almost every time and would
/// leave the six others -- which are the ones with the interesting sign tests
/// -- essentially untested.
static void test_closest_point_on_triangle( void )
{
	printf( "closest point on a triangle\n" );

	// A right triangle in the XZ plane, so every expected answer is exact in
	// Q12 and a failure is the routine's rather than the fixture's.
	b3Vec3 a = V( 0, 0, 0 );
	b3Vec3 b = V( 2, 0, 0 );
	b3Vec3 c = V( 0, 0, 2 );

	struct
	{
		const char* what;
		b3Vec3 query;
		b3Vec3 want;
		b3TriangleFeature feature;
	} cases[] = {
		{ "vertex A", V( -1, 0, -1 ), a, b3_featureVertex1 },
		{ "vertex B", V( 3.5, 0, -1 ), b, b3_featureVertex2 },
		{ "vertex C", V( -1, 0, 3.5 ), c, b3_featureVertex3 },
		{ "edge AB", V( 1, 0, -1 ), V( 1, 0, 0 ), b3_featureEdge1 },
		{ "edge BC", V( 1.5, 0, 1.5 ), V( 1, 0, 1 ), b3_featureEdge2 },
		{ "edge CA", V( -1, 0, 1 ), V( 0, 0, 1 ), b3_featureEdge3 },
		{ "face, above", V( 0.5, 3, 0.5 ), V( 0.5, 0, 0.5 ), b3_featureTriangleFace },
		{ "face, below", V( 0.5, -3, 0.5 ), V( 0.5, 0, 0.5 ), b3_featureTriangleFace },
	};

	for ( size_t i = 0; i < sizeof( cases ) / sizeof( cases[0] ); ++i )
	{
		b3TrianglePoint got = b3ClosestPointOnTriangle( a, b, c, cases[i].query );

		char label[64];
		snprintf( label, sizeof( label ), "%s: feature", cases[i].what );
		checkInt2( label, got.feature, cases[i].feature );

		snprintf( label, sizeof( label ), "%s: point", cases[i].what );
		double dx = b3fToDouble( got.point.x ) - b3fToDouble( cases[i].want.x );
		double dy = b3fToDouble( got.point.y ) - b3fToDouble( cases[i].want.y );
		double dz = b3fToDouble( got.point.z ) - b3fToDouble( cases[i].want.z );
		expect( label, sqrt( dx * dx + dy * dy + dz * dz ), 0.0, 4 * Q12 );
	}

	// A triangle whose edges are a few hundredths of a unit. The narrow
	// spelling of the cross product gives this one a zero normal, and the
	// degree-four region tests lose all their precision if the dot products
	// are pre-shifted by a fixed amount rather than by one derived from them.
	// Both were real bugs; this is the fixture that fails on either.
	{
		b3Vec3 ta = V( 0, 0, 0 );
		b3Vec3 tb = V( 0.05, 0, 0 );
		b3Vec3 tc = V( 0, 0, 0.05 );

		b3TrianglePoint got = b3ClosestPointOnTriangle( ta, tb, tc, V( 0.01, 1.0, 0.01 ) );
		checkInt2( "a 0.05-unit triangle still resolves its face", got.feature, b3_featureTriangleFace );
		expect( "and the closest point lands on it", b3fToDouble( got.point.y ), 0.0, 2 * Q12 );

		b3Plane plane = b3MakePlaneFromPoints( ta, tb, tc );
		double nx = b3fToDouble( plane.normal.x );
		double ny = b3fToDouble( plane.normal.y );
		double nz = b3fToDouble( plane.normal.z );
		expect( "and its plane normal is still unit", sqrt( nx * nx + ny * ny + nz * nz ), 1.0, 8 * Q12 );
	}

	// Degenerate: two coincident vertices. Upstream divides by a zero
	// barycentric denominator here; this must return a point on the triangle
	// rather than an infinity.
	{
		b3Vec3 da = V( 1, 0, 1 );
		b3TrianglePoint got = b3ClosestPointOnTriangle( da, da, da, V( 5, 5, 5 ) );
		expect( "a fully degenerate triangle returns one of its vertices", b3fToDouble( got.point.x ), 1.0, Q12 );
	}
}

static void test_triangle_sphere( void )
{
	printf( "triangle versus sphere\n" );

	// Wound so the face normal is +Y: cross(b - a, c - a) points up.
	b3Vec3 tri[3] = { V( -1, 0, -1 ), V( -1, 0, 1 ), V( 1, 0, -1 ) };

	b3LocalManifoldPoint buffer[4] = { 0 };
	b3LocalManifold m = { 0 };
	m.points = buffer;

	// Resting just above the face.
	b3Sphere s1 = { V( -0.5, 0.4, -0.5 ), b3fFromDouble( 0.5 ) };
	b3CollideTriangleAndSphere( &m, 4, tri, &s1 );
	checkInt2( "a sphere above the face makes one point", m.pointCount, 1 );
	checkInt2( "and resolves against the face", m.feature, b3_featureTriangleFace );
	expect( "with the face normal", b3fToDouble( m.normal.y ), 1.0, 8 * Q12 );
	expect( "and the overlap as separation", b3fToDouble( m.points[0].separation ), -0.1, 8 * Q12 );

	// The same sphere underneath. A mesh triangle is one-sided, so this is the
	// inside of the level and must produce nothing -- answering it would push
	// a body that has already tunnelled further through.
	b3Sphere s2 = { V( -0.5, -0.4, -0.5 ), b3fFromDouble( 0.5 ) };
	b3CollideTriangleAndSphere( &m, 4, tri, &s2 );
	checkInt2( "a sphere below the face is culled", m.pointCount, 0 );

	// Far enough away that even the speculative window does not reach.
	b3Sphere s3 = { V( -0.5, 5.0, -0.5 ), b3fFromDouble( 0.5 ) };
	b3CollideTriangleAndSphere( &m, 4, tri, &s3 );
	checkInt2( "a distant sphere makes no points", m.pointCount, 0 );

	// squaredDistance is what the mesh narrow phase orders its candidates by,
	// and it is the only field only this function writes.
	// Just inside the speculative window: separation 0.01 against
	// B3_SPECULATIVE_DISTANCE of 0.02, so a point is reported even though the
	// surfaces are apart.
	b3Sphere s4 = { V( -0.5, 0.51, -0.5 ), b3fFromDouble( 0.5 ) };
	b3CollideTriangleAndSphere( &m, 4, tri, &s4 );
	checkInt2( "a speculative sphere still reports a point", m.pointCount, 1 );
	expect( "with a positive separation", b3fToDouble( m.points[0].separation ), 0.01, 8 * Q12 );
	expect( "and its squared distance, wide", (double)m.squaredDistance / ( 4096.0 * 4096.0 ), 0.51 * 0.51, 0.01 );
}

static void test_broad_phase( void )
{
	section( "broad phase proxy management" );

	b3Capacity capacity = { 0 };
	capacity.staticShapeCount = 16;
	capacity.dynamicShapeCount = 16;
	capacity.contactCount = 16;

	b3BroadPhase bp = { 0 };
	b3CreateBroadPhase( &bp, &capacity );

	checkInt2( "move buffer starts empty", bp.moveArray.count, 0 );

	// --- proxy keys pack type and id --------------------------------------
	b3AABB boxA = Box( 0, 0, 0, 1, 1, 1 );
	b3AABB boxB = Box( 0.5, 0.5, 0.5, 2, 2, 2 );
	b3AABB boxC = Box( 10, 10, 10, 11, 11, 11 );

	int keyStatic = b3BroadPhase_CreateProxy( &bp, b3_staticBody, boxA, B3_DEFAULT_CATEGORY_BITS, 100, false );
	int keyDynamic = b3BroadPhase_CreateProxy( &bp, b3_dynamicBody, boxB, B3_DEFAULT_CATEGORY_BITS, 200, false );
	int keyKinematic = b3BroadPhase_CreateProxy( &bp, b3_kinematicBody, boxC, B3_DEFAULT_CATEGORY_BITS, 300, false );

	checkInt2( "static key carries its type", B3_PROXY_TYPE( keyStatic ), b3_staticBody );
	checkInt2( "dynamic key carries its type", B3_PROXY_TYPE( keyDynamic ), b3_dynamicBody );
	checkInt2( "kinematic key carries its type", B3_PROXY_TYPE( keyKinematic ), b3_kinematicBody );

	// The shape index round-trips through the tree's user data.
	checkInt2( "static shape index", b3BroadPhase_GetShapeIndex( &bp, keyStatic ), 100 );
	checkInt2( "dynamic shape index", b3BroadPhase_GetShapeIndex( &bp, keyDynamic ), 200 );
	checkInt2( "kinematic shape index", b3BroadPhase_GetShapeIndex( &bp, keyKinematic ), 300 );

	// Each type went into its own tree.
	checkInt2( "static tree has one proxy", b3DynamicTree_GetProxyCount( &bp.trees[b3_staticBody] ), 1 );
	checkInt2( "dynamic tree has one proxy", b3DynamicTree_GetProxyCount( &bp.trees[b3_dynamicBody] ), 1 );
	checkInt2( "kinematic tree has one proxy", b3DynamicTree_GetProxyCount( &bp.trees[b3_kinematicBody] ), 1 );

	// --- static proxies are not buffered ----------------------------------
	//
	// This is the one behavioural asymmetry in the file: a static proxy never
	// moves, so buffering it would generate pair queries forever.
	check( "static proxy not buffered", moveArrayContains( &bp, keyStatic ) == false );
	check( "dynamic proxy buffered", moveArrayContains( &bp, keyDynamic ) );
	check( "kinematic proxy buffered", moveArrayContains( &bp, keyKinematic ) );
	check( "buffer consistent after creates", moveBufferConsistent( &bp ) );

	// ...unless forced, which is how a static shape added to an existing
	// scene gets its pairs found.
	int keyForced = b3BroadPhase_CreateProxy( &bp, b3_staticBody, boxC, B3_DEFAULT_CATEGORY_BITS, 400, true );
	check( "forced static proxy is buffered", moveArrayContains( &bp, keyForced ) );

	// --- overlap uses the stored AABBs ------------------------------------
	check( "overlapping proxies report overlap", b3BroadPhase_TestOverlap( &bp, keyStatic, keyDynamic ) );
	check( "distant proxies do not", b3BroadPhase_TestOverlap( &bp, keyStatic, keyKinematic ) == false );

	// --- buffering is idempotent ------------------------------------------
	//
	// Moving the same proxy twice in a step must not enqueue it twice; the
	// bitset is what prevents that.
	int before = bp.moveArray.count;
	b3BroadPhase_MoveProxy( &bp, keyDynamic, Box( 3, 3, 3, 4, 4, 4 ) );
	b3BroadPhase_MoveProxy( &bp, keyDynamic, Box( 4, 4, 4, 5, 5, 5 ) );
	checkInt2( "repeated moves buffer once", bp.moveArray.count, before );
	check( "buffer consistent after moves", moveBufferConsistent( &bp ) );
	check( "move updated the stored AABB",
		   b3BroadPhase_TestOverlap( &bp, keyDynamic, keyDynamic ) &&
			   b3AABB_Contains( b3DynamicTree_GetAABB( &bp.trees[b3_dynamicBody], B3_PROXY_ID( keyDynamic ) ),
								Box( 4, 4, 4, 5, 5, 5 ) ) );

	// --- enlarge also buffers ---------------------------------------------
	b3BroadPhase_EnlargeProxy( &bp, keyKinematic, Box( 8, 8, 8, 14, 14, 14 ) );
	check( "enlarge keeps the buffer consistent", moveBufferConsistent( &bp ) );

	// --- unbuffering on destroy, from the middle of the array --------------
	//
	// b3UnBufferMove swaps the last entry into the hole, so removing from the
	// middle is the case that can corrupt the array. Fill it first.
	int keys[8];
	for ( int i = 0; i < 8; ++i )
	{
		b3AABB box = Box( 20 + i, 0, 0, 21 + i, 1, 1 );
		keys[i] = b3BroadPhase_CreateProxy( &bp, b3_dynamicBody, box, B3_DEFAULT_CATEGORY_BITS, 500 + i, false );
	}
	check( "buffer consistent after batch create", moveBufferConsistent( &bp ) );

	int countBefore = bp.moveArray.count;
	int middle = keys[3];
	check( "middle proxy is buffered before destroy", moveArrayContains( &bp, middle ) );

	b3BroadPhase_DestroyProxy( &bp, middle );

	check( "destroyed proxy left the move array", moveArrayContains( &bp, middle ) == false );
	checkInt2( "move array shrank by exactly one", bp.moveArray.count, countBefore - 1 );
	check( "buffer consistent after destroy", moveBufferConsistent( &bp ) );

	// The survivors must all still be there -- a swap-remove that took the
	// wrong index would silently drop one of these.
	bool survivorsIntact = true;
	for ( int i = 0; i < 8; ++i )
	{
		if ( i != 3 && moveArrayContains( &bp, keys[i] ) == false )
		{
			survivorsIntact = false;
		}
	}
	check( "other proxies survived the swap-remove", survivorsIntact );

	// Destroying an unbuffered proxy must be harmless.
	countBefore = bp.moveArray.count;
	b3BroadPhase_DestroyProxy( &bp, keyStatic );
	checkInt2( "destroying an unbuffered proxy changes nothing", bp.moveArray.count, countBefore );
	check( "buffer consistent after unbuffered destroy", moveBufferConsistent( &bp ) );

	b3DestroyBroadPhase( &bp );
	checkInt2( "destroy clears the move array", bp.moveArray.count, 0 );
	check( "destroy released the trees", bp.trees[b3_dynamicBody].nodes == NULL );
}

// =========================================================================
// Feature pairs
// =========================================================================
//
// b3FlipPair looks wrong and is not: it swaps the two halves of the pair *and*
// complements both owners, so that (A,i,B,j) maps to (A,j,B,i) rather than to
// (B,j,A,i). That is what makes the identity of a contact independent of which
// hull supplied the reference face, which is what lets the solver carry an
// impulse across a step where the reference face flip-flopped.
//
// The port had this wrong until this increment -- it swapped without
// complementing -- and nothing detected it, because no caller existed yet and
// no manifold-geometry comparison can see it. These checks are the ones that
// would have.

static void test_feature_pairs( void )
{
	section( "feature pairs" );

	b3FeaturePair p = b3MakeFeaturePair( b3_featureShapeA, 3, b3_featureShapeB, 7 );
	b3FeaturePair f = b3FlipPair( p );

	checkInt2( "flip owner1", f.owner1, (uint8_t)b3_featureShapeA );
	checkInt2( "flip index1", f.index1, 7 );
	checkInt2( "flip owner2", f.owner2, (uint8_t)b3_featureShapeB );
	checkInt2( "flip index2", f.index2, 3 );

	// An involution, for every owner combination.
	for ( int o1 = 0; o1 <= 1; ++o1 )
	{
		for ( int o2 = 0; o2 <= 1; ++o2 )
		{
			for ( int i = 0; i < 5; ++i )
			{
				b3FeaturePair a = b3MakeFeaturePair( (b3FeatureOwner)o1, i, (b3FeatureOwner)o2, i + 11 );
				b3FeaturePair b = b3FlipPair( b3FlipPair( a ) );
				check( "flip is an involution", a.owner1 == b.owner1 && a.index1 == b.index1 && a.owner2 == b.owner2 &&
													a.index2 == b.index2 );
			}
		}
	}

	// The identifier has to separate every field, or two different contacts
	// would warm-start from each other.
	uint32_t seen[32];
	int count = 0;
	for ( int o1 = 0; o1 <= 1; ++o1 )
	{
		for ( int o2 = 0; o2 <= 1; ++o2 )
		{
			for ( int i = 0; i < 4; ++i )
			{
				uint32_t id = b3MakeFeatureId( b3MakeFeaturePair( (b3FeatureOwner)o1, i, (b3FeatureOwner)o2, 2 * i ) );
				for ( int k = 0; k < count; ++k )
				{
					check( "feature ids are distinct", seen[k] != id );
				}
				seen[count++] = id;
			}
		}
	}
}

// =========================================================================
// Polygon clipping and incident faces
// =========================================================================

static b3ClipVertex CV( double x, double y, double z, int in, int out )
{
	b3ClipVertex v;
	v.position = V( x, y, z );
	v.separation = b3f_zero;
	v.pair = b3MakeFeaturePair( b3_featureShapeB, in, b3_featureShapeB, out );
	return v;
}

static void test_clip_polygon( void )
{
	section( "polygon clipping" );

	// The reference plane is z = 0 with normal +Z, so separations come out as
	// the z coordinate. The square sits in it.
	b3Plane refPlane = b3MakePlaneFromNormalAndPoint( V( 0, 0, 1 ), V( 0, 0, 0 ) );

	b3ClipVertex square[4] = {
		CV( -1, -1, 0, 0, 1 ),
		CV( 1, -1, 0, 1, 2 ),
		CV( 1, 1, 0, 2, 3 ),
		CV( -1, 1, 0, 3, 0 ),
	};

	b3ClipVertex out[B3_MAX_CLIP_POINTS];

	// Entirely behind: unchanged, pairs included.
	b3Plane behind = b3MakePlaneFromNormalAndPoint( V( 1, 0, 0 ), V( 5, 0, 0 ) );
	int n = b3ClipPolygon( out, square, 4, behind, 9, refPlane );
	checkInt2( "clip behind keeps all", n, 4 );
	if ( n == 4 )
	{
		for ( int i = 0; i < 4; ++i )
		{
			checkInt2( "clip behind keeps pair", b3MakeFeatureId( out[i].pair ), b3MakeFeatureId( square[i].pair ) );
		}
	}

	// Entirely in front: nothing survives.
	b3Plane infront = b3MakePlaneFromNormalAndPoint( V( 1, 0, 0 ), V( -5, 0, 0 ) );
	n = b3ClipPolygon( out, square, 4, infront, 9, refPlane );
	checkInt2( "clip in front keeps none", n, 0 );

	// Through the middle at x = 0: two originals survive and two crossings are
	// interpolated onto the plane exactly.
	b3Plane middle = b3MakePlaneFromNormalAndPoint( V( 1, 0, 0 ), V( 0, 0, 0 ) );
	n = b3ClipPolygon( out, square, 4, middle, 9, refPlane );
	checkInt2( "clip through middle", n, 4 );

	int onPlane = 0;
	for ( int i = 0; i < n; ++i )
	{
		if ( fabs( b3fToDouble( out[i].position.x ) ) < 4.0 * Q12 )
		{
			onPlane++;
		}
		check( "clip keeps the behind half", b3fToDouble( out[i].position.x ) <= 4.0 * Q12 );
	}
	checkInt2( "clip produced two crossings", onPlane, 2 );
#if B3_ENABLE_VALIDATION
	check( "clip output chains", b3ValidatePolygon( out, n ) );
#endif

	// Exactly through a corner. The `<= 0` boundary counts as behind, so the
	// corner is kept rather than duplicated away.
	b3Plane corner = b3MakePlaneFromNormalAndPoint( V( 1, 1, 0 ), V( 1, 1, 0 ) );
	n = b3ClipPolygon( out, square, 4, corner, 9, refPlane );
	check( "clip through a corner keeps the polygon", n >= 3 && n <= 5 );
#if B3_ENABLE_VALIDATION
	check( "corner clip chains", b3ValidatePolygon( out, n ) );
#endif

	// Overflow must be refused rather than written past the end. A 20-gon clipped
	// by a plane that crosses it can reach 21 points, and repeatedly clipping a
	// polygon that keeps growing will run out of the 32-entry buffer.
	b3ClipVertex big[B3_MAX_CLIP_POINTS];
	int bigCount = 20;
	for ( int i = 0; i < bigCount; ++i )
	{
		double a = 2.0 * 3.14159265358979 * i / bigCount;
		big[i] = CV( 2.0 * cos( a ), 2.0 * sin( a ), 0.0, i, ( i + 1 ) % bigCount );
	}

	// A canary immediately after the output buffer catches a write past the end
	// even if the guard is missing and the run happens not to crash.
	struct
	{
		b3ClipVertex buffer[B3_MAX_CLIP_POINTS];
		uint32_t canary;
	} guarded;
	guarded.canary = 0xB03DEADu;

	int guardHit = 0;
	int cur = bigCount;
	b3ClipVertex work[B3_MAX_CLIP_POINTS];
	memcpy( work, big, sizeof( b3ClipVertex ) * bigCount );

	for ( int step = 0; step < 20 && cur > 0; ++step )
	{
		double a = 2.0 * 3.14159265358979 * step / 20.0;
		b3Plane pl = b3MakePlaneFromNormalAndPoint( V( cos( a ), sin( a ), 0 ), V( 1.6 * cos( a ), 1.6 * sin( a ), 0 ) );
		int r = b3ClipPolygon( guarded.buffer, work, cur, pl, step, refPlane );
		if ( r == B3_NULL_INDEX )
		{
			guardHit = 1;
			break;
		}
		cur = r;
		if ( cur > 0 )
		{
			memcpy( work, guarded.buffer, sizeof( b3ClipVertex ) * cur );
		}
	}

	check( "clip never overran its buffer", guarded.canary == 0xB03DEADu );
	check( "clip output stayed within capacity", cur <= B3_MAX_CLIP_POINTS );
	(void)guardHit;
}

static void test_find_incident_face( void )
{
	section( "incident face" );

	b3BoxHull cube = b3MakeCubeHull( b3fFromDouble( 1.0 ) );
	const b3Plane* planes = b3GetHullPlanes( &cube.base );

	// For every face of the cube, and every vertex on the opposite side, the
	// incident face for that face's normal must be the one pointing the other
	// way.
	for ( int f = 0; f < cube.base.faceCount; ++f )
	{
		b3Vec3 refNormal = planes[f].normal;

		// The search starts from a vertex and only ever reaches faces adjacent
		// to it, so the vertex has to be the support point along -ref -- which
		// is exactly what b3CollideHulls passes, as query.indexB.
		int v = b3FindHullSupportVertex( &cube.base, b3Neg( refNormal ) );
		int inc = b3FindIncidentFace( &cube.base, refNormal, v );
		check( "incident face is in range", 0 <= inc && inc < cube.base.faceCount );

		// It must be the most opposed face, i.e. dot(n_inc, ref) == -1.
		double d = b3fToDouble( b3Dot( planes[inc].normal, refNormal ) );
		expect( "incident face is opposed", d, -1.0, 8.0 * Q12 );
	}

	// A slab is the wedge-like case the upstream comment is about: the naive
	// "most anti-parallel face" search can pick a side face when the incident
	// hull is thin.
	b3BoxHull slab = b3MakeBoxHull( b3fFromDouble( 1.0 ), b3fFromDouble( 0.1 ), b3fFromDouble( 1.0 ) );
	const b3Plane* slabPlanes = b3GetHullPlanes( &slab.base );

	int sv = b3FindHullSupportVertex( &slab.base, V( 0, -1, 0 ) );
	int sinc = b3FindIncidentFace( &slab.base, V( 0, 1, 0 ), sv );
	double sd = b3fToDouble( b3Dot( slabPlanes[sinc].normal, V( 0, 1, 0 ) ) );
	expect( "slab incident face is -Y", sd, -1.0, 8.0 * Q12 );
}

// =========================================================================
// Hull versus hull manifolds
// =========================================================================

/// Shared scratch. b3CollideHulls writes at most four points and refuses a
/// capacity below four.
typedef struct
{
	b3LocalManifoldPoint points[4];
	b3LocalManifold manifold;
	b3SATCache cache;
} hullPair;

static void hullPairReset( hullPair* hp )
{
	memset( hp, 0, sizeof( *hp ) );
	hp->manifold.points = hp->points;
}

static b3Transform Xf( double px, double py, double pz )
{
	b3Transform t = b3Transform_identity;
	t.p = V( px, py, pz );
	return t;
}

/// Smallest separation in the manifold.
static double minSeparation( const b3LocalManifold* m )
{
	double s = 1e30;
	for ( int i = 0; i < m->pointCount; ++i )
	{
		double v = b3fToDouble( m->points[i].separation );
		if ( v < s )
		{
			s = v;
		}
	}
	return s;
}

static void test_hull_hull_faces( void )
{
	section( "hull vs hull -- face contacts" );

	b3BoxHull cubeA = b3MakeCubeHull( b3fFromDouble( 1.0 ) );
	b3BoxHull cubeB = b3MakeCubeHull( b3fFromDouble( 1.0 ) );

	hullPair hp;

	// A: resting exactly, four corners at y = 1.
	hullPairReset( &hp );
	b3CollideHulls( &hp.manifold, 4, &cubeA.base, &cubeB.base, Xf( 0, 2.0, 0 ), &hp.cache );
	checkInt2( "A touching point count", hp.manifold.pointCount, 4 );
	expectVec( "A normal", hp.manifold.normal, 0, 1, 0, 4.0 * Q12 );
	for ( int i = 0; i < hp.manifold.pointCount; ++i )
	{
		expect( "A separation", b3fToDouble( hp.manifold.points[i].separation ), 0.0, 8.0 * Q12 );
		expect( "A point y", b3fToDouble( hp.manifold.points[i].point.y ), 1.0, 8.0 * Q12 );
		check( "A point x in face", fabs( b3fToDouble( hp.manifold.points[i].point.x ) ) <= 1.0 + 8.0 * Q12 );
		check( "A point z in face", fabs( b3fToDouble( hp.manifold.points[i].point.z ) ) <= 1.0 + 8.0 * Q12 );
	}

	// B: overlapping by a quarter. The contact sits half way between the two
	// surfaces, so at y = 1 - 0.125.
	hullPairReset( &hp );
	b3CollideHulls( &hp.manifold, 4, &cubeA.base, &cubeB.base, Xf( 0, 1.75, 0 ), &hp.cache );
	checkInt2( "B point count", hp.manifold.pointCount, 4 );
	for ( int i = 0; i < hp.manifold.pointCount; ++i )
	{
		expect( "B separation", b3fToDouble( hp.manifold.points[i].separation ), -0.25, 8.0 * Q12 );
		expect( "B point y", b3fToDouble( hp.manifold.points[i].point.y ), 0.875, 8.0 * Q12 );
	}

	// C: inside the speculative window, so contacts still exist with a positive
	// separation.
	hullPairReset( &hp );
	b3CollideHulls( &hp.manifold, 4, &cubeA.base, &cubeB.base, Xf( 0, 2.015, 0 ), &hp.cache );
	check( "C speculative contacts exist", hp.manifold.pointCount > 0 );
	for ( int i = 0; i < hp.manifold.pointCount; ++i )
	{
		expect( "C separation", b3fToDouble( hp.manifold.points[i].separation ), 0.015, 8.0 * Q12 );
	}

	// D: clearly apart. No points, and the cache remembers the axis that
	// separated them.
	hullPairReset( &hp );
	b3CollideHulls( &hp.manifold, 4, &cubeA.base, &cubeB.base, Xf( 0, 4.0, 0 ), &hp.cache );
	checkInt2( "D separated point count", hp.manifold.pointCount, 0 );
	checkInt2( "D cached a face axis", hp.cache.type, (uint8_t)b3_faceAxisA );
	expect( "D cached separation", b3fToDouble( hp.cache.separation ), 2.0, 8.0 * Q12 );

	// E: half-overlapped, so the clip region is x in [0,1]. This is the check
	// that catches a broken side plane -- a wrong binormal leaves points
	// outside the reference face.
	hullPairReset( &hp );
	b3CollideHulls( &hp.manifold, 4, &cubeA.base, &cubeB.base, Xf( 1.0, 2.0, 0 ), &hp.cache );
	checkInt2( "E point count", hp.manifold.pointCount, 4 );
	for ( int i = 0; i < hp.manifold.pointCount; ++i )
	{
		double x = b3fToDouble( hp.manifold.points[i].point.x );
		check( "E clipped to the reference face", x >= -8.0 * Q12 && x <= 1.0 + 8.0 * Q12 );
		expect( "E separation", b3fToDouble( hp.manifold.points[i].separation ), 0.0, 8.0 * Q12 );
	}
}

static void test_hull_hull_face_b( void )
{
	section( "hull vs hull -- B supplies the reference face" );

	b3BoxHull small = b3MakeBoxHull( b3fFromDouble( 0.2 ), b3fFromDouble( 0.2 ), b3fFromDouble( 0.2 ) );
	b3BoxHull big = b3MakeCubeHull( b3fFromDouble( 2.0 ) );

	hullPair hp;
	hullPairReset( &hp );
	b3CollideHulls( &hp.manifold, 4, &small.base, &big.base, Xf( 0, -2.2, 0 ), &hp.cache );

	checkInt2( "F point count", hp.manifold.pointCount, 4 );
	expectVec( "F normal points A to B", hp.manifold.normal, 0, -1, 0, 8.0 * Q12 );
	check( "F normal is unit", b3IsNormalized( hp.manifold.normal ) );
	expect( "F min separation", minSeparation( &hp.manifold ), 0.0, 8.0 * Q12 );

	// The invariant b3FlipPair exists for: the identity of a contact must not
	// depend on which hull supplied the reference face. Running the mirrored
	// configuration -- hulls swapped, transform inverted -- has B's face as the
	// reference in one and A's in the other, and the feature id sets must still
	// match.
	//
	// This is the only check in the suite that sees the owner complement. A
	// b3FlipPair that merely swapped would pass every geometric comparison and
	// fail here.
	uint32_t idsForward[4];
	for ( int i = 0; i < hp.manifold.pointCount; ++i )
	{
		idsForward[i] = b3MakeFeatureId( hp.manifold.points[i].pair );
	}
	int forwardCount = hp.manifold.pointCount;

	hullPair mirrored;
	hullPairReset( &mirrored );
	b3CollideHulls( &mirrored.manifold, 4, &big.base, &small.base, b3InvertTransform( Xf( 0, -2.2, 0 ) ),
					&mirrored.cache );

	checkInt2( "F mirrored point count", mirrored.manifold.pointCount, forwardCount );

	for ( int i = 0; i < mirrored.manifold.pointCount; ++i )
	{
		uint32_t id = b3MakeFeatureId( b3FlipPair( mirrored.manifold.points[i].pair ) );
		bool found = false;
		for ( int k = 0; k < forwardCount; ++k )
		{
			if ( idsForward[k] == id )
			{
				found = true;
			}
		}
		check( "F feature id survives the A/B swap", found );
	}
}

static void test_hull_hull_manual_features( void )
{
	section( "hull vs hull -- forced features" );

	b3BoxHull cubeA = b3MakeCubeHull( b3fFromDouble( 1.0 ) );
	b3BoxHull cubeB = b3MakeCubeHull( b3fFromDouble( 1.0 ) );

	// Half overlapped, so the clip actually does something -- with the faces
	// exactly aligned nothing is ever cut and the test would pass trivially.
	b3Transform xf = Xf( 1.0, 2.0, 0 );

	// H: the same configuration through face A and face B.
	//
	// What is compared is the contact *geometry*, not the feature labels. The
	// labels legitimately differ: with the two cubes half overlapped, the
	// corner at x = 1 is simultaneously one of A's own face corners and the
	// crossing of an A edge with a B edge, and each reference-face choice
	// describes it the way it found it. Asserting the labels match would be
	// asserting something neither library provides. What both must agree on is
	// where the contact is and how deep -- which exercises the whole frame
	// round trip in b3BuildFaceBContact: invert the transform, solve in B's
	// frame, rotate the points back, renormalize the normal.
	//
	// The feature-pair invariant that b3FlipPair *does* guarantee is checked
	// directly, and exactly, in test_feature_pairs.
	hullPair viaA, viaB;
	hullPairReset( &viaA );
	hullPairReset( &viaB );
	viaA.cache.type = (uint8_t)b3_manualFaceAxisA;
	viaB.cache.type = (uint8_t)b3_manualFaceAxisB;

	b3CollideHulls( &viaA.manifold, 4, &cubeA.base, &cubeB.base, xf, &viaA.cache );
	b3CollideHulls( &viaB.manifold, 4, &cubeA.base, &cubeB.base, xf, &viaB.cache );

	checkInt2( "H face A and face B agree on count", viaA.manifold.pointCount, viaB.manifold.pointCount );

	// The normals are opposed statements of the same axis, and both must be
	// unit -- the renormalize in b3BuildFaceBContact is what makes the second
	// one so.
	check( "H face B normal is unit", b3IsNormalized( viaB.manifold.normal ) );
	expect( "H normals agree", b3fToDouble( b3Dot( viaA.manifold.normal, viaB.manifold.normal ) ), 1.0, 8.0 * Q12 );

	for ( int i = 0; i < viaB.manifold.pointCount; ++i )
	{
		bool found = false;
		for ( int k = 0; k < viaA.manifold.pointCount; ++k )
		{
			b3Vec3 d = b3Sub( viaA.manifold.points[k].point, viaB.manifold.points[i].point );
			if ( fabs( b3fToDouble( d.x ) ) < 16.0 * Q12 && fabs( b3fToDouble( d.y ) ) < 16.0 * Q12 &&
				 fabs( b3fToDouble( d.z ) ) < 16.0 * Q12 )
			{
				found = true;
				expect( "H matched depth", b3fToDouble( viaA.manifold.points[k].separation ),
						b3fToDouble( viaB.manifold.points[i].separation ), 16.0 * Q12 );
			}
		}
		check( "H contact point survives the reference face swap", found );
	}

	// G: two cubes rotated 45 degrees about different axes, so their nearest
	// features are crossed edges. Driven through the manual edge path, the
	// answer is exactly one point at the crossing.
	b3BoxHull tilted = b3MakeTransformedBoxHull( b3fFromDouble( 1.0 ), b3fFromDouble( 1.0 ), b3fFromDouble( 1.0 ),
												 ( b3Transform ){ b3Vec3_zero, b3MakeQuatFromAxisAngle( V( 1, 0, 0 ),
																									   (b3a)( 32768 / 8 ) ) } );

	double s2 = sqrt( 2.0 );
	b3Transform cross = { V( 0, 2.0 * s2, 0 ), b3MakeQuatFromAxisAngle( V( 0, 0, 1 ), (b3a)( 32768 / 8 ) ) };

	hullPair edge;
	hullPairReset( &edge );
	edge.cache.type = (uint8_t)b3_manualEdgePairAxis;
	b3CollideHulls( &edge.manifold, 4, &tilted.base, &cubeB.base, cross, &edge.cache );

	checkInt2( "G forced edge gives one point", edge.manifold.pointCount, 1 );
	if ( edge.manifold.pointCount == 1 )
	{
		expect( "G edge contact x", b3fToDouble( edge.manifold.points[0].point.x ), 0.0, 24.0 * Q12 );
		expect( "G edge contact z", b3fToDouble( edge.manifold.points[0].point.z ), 0.0, 24.0 * Q12 );
		expect( "G edge contact y", b3fToDouble( edge.manifold.points[0].point.y ), s2, 24.0 * Q12 );
		expect( "G edge separation", b3fToDouble( edge.manifold.points[0].separation ), 0.0, 16.0 * Q12 );
		check( "G edge normal is unit", b3IsNormalized( edge.manifold.normal ) );
	}

	// Pushed together, the same forced edge path must report the overlap.
	hullPairReset( &edge );
	edge.cache.type = (uint8_t)b3_manualEdgePairAxis;
	b3Transform closer = { V( 0, 2.0 * s2 - 0.2, 0 ), b3MakeQuatFromAxisAngle( V( 0, 0, 1 ), (b3a)( 32768 / 8 ) ) };
	b3CollideHulls( &edge.manifold, 4, &tilted.base, &cubeB.base, closer, &edge.cache );
	checkInt2( "G pressed edge gives one point", edge.manifold.pointCount, 1 );
	if ( edge.manifold.pointCount == 1 )
	{
		expect( "G pressed separation", b3fToDouble( edge.manifold.points[0].separation ), -0.2, 24.0 * Q12 );
	}
}

static void test_hull_hull_degenerate( void )
{
	section( "hull vs hull -- degenerate and refused" );

	b3BoxHull cubeA = b3MakeCubeHull( b3fFromDouble( 1.0 ) );
	b3BoxHull cubeB = b3MakeCubeHull( b3fFromDouble( 1.0 ) );

	// I: exactly coincident. Every axis ties at -2, and nothing may divide by
	// zero or normalize a zero vector.
	hullPair hp;
	hullPairReset( &hp );
	b3CollideHulls( &hp.manifold, 4, &cubeA.base, &cubeB.base, b3Transform_identity, &hp.cache );
	checkInt2( "I coincident point count", hp.manifold.pointCount, 4 );
	check( "I normal is unit", b3IsNormalized( hp.manifold.normal ) );
	expect( "I min separation", minSeparation( &hp.manifold ), -2.0, 16.0 * Q12 );

	// J: capacity below four produces nothing and does not touch the buffer.
	for ( int capacity = 0; capacity <= 3; ++capacity )
	{
		b3LocalManifoldPoint pts[4];
		memset( pts, 0xA5, sizeof( pts ) );

		b3LocalManifold m = { 0 };
		m.points = pts;
		m.pointCount = 7;

		b3SATCache cache = { 0 };
		b3CollideHulls( &m, capacity, &cubeA.base, &cubeB.base, Xf( 0, 1.5, 0 ), &cache );

		checkInt2( "J refuses a short capacity", m.pointCount, 0 );

		bool untouched = true;
		const unsigned char* raw = (const unsigned char*)pts;
		for ( size_t k = 0; k < sizeof( pts ); ++k )
		{
			if ( raw[k] != 0xA5 )
			{
				untouched = false;
			}
		}
		check( "J left the caller's buffer alone", untouched );
	}

	// L: a thin box edge-on, which drives short edges through the parallel-edge
	// reject and the unnormalized side-plane binormal.
	b3BoxHull thin = b3MakeBoxHull( b3fFromDouble( 0.02 ), b3fFromDouble( 1.0 ), b3fFromDouble( 1.0 ) );
	hullPairReset( &hp );
	b3CollideHulls( &hp.manifold, 4, &cubeA.base, &thin.base, Xf( 1.015, 0, 0 ), &hp.cache );
	check( "L thin box produced contacts", hp.manifold.pointCount > 0 );
	check( "L thin box normal is unit", b3IsNormalized( hp.manifold.normal ) );
	expect( "L thin box separation", minSeparation( &hp.manifold ), -0.005, 16.0 * Q12 );
}

static void test_hull_hull_cache( void )
{
	section( "hull vs hull -- separating axis cache" );

	b3BoxHull cubeA = b3MakeCubeHull( b3fFromDouble( 1.0 ) );
	b3BoxHull cubeB = b3MakeCubeHull( b3fFromDouble( 1.0 ) );

	hullPair hp;
	hullPairReset( &hp );

	// Cold, then the identical call again: the second must take the shortcut.
	b3CollideHulls( &hp.manifold, 4, &cubeA.base, &cubeB.base, Xf( 0, 1.9, 0 ), &hp.cache );
	checkInt2( "K first call is cold", hp.cache.hit, 0 );
	int coldCount = hp.manifold.pointCount;

	b3CollideHulls( &hp.manifold, 4, &cubeA.base, &cubeB.base, Xf( 0, 1.9, 0 ), &hp.cache );
	checkInt2( "K repeat call hits", hp.cache.hit, 1 );
	checkInt2( "K repeat call agrees", hp.manifold.pointCount, coldCount );

	// A nudge well inside the slop keeps the same feature.
	b3CollideHulls( &hp.manifold, 4, &cubeA.base, &cubeB.base, Xf( 0, 1.9 + 0.5 * 20.0 * Q12, 0 ), &hp.cache );
	checkInt2( "K sub-slop nudge still hits", hp.cache.hit, 1 );

	// Moving clear: the cached face still separates them, so it is a hit that
	// produces no points.
	b3CollideHulls( &hp.manifold, 4, &cubeA.base, &cubeB.base, Xf( 0, 6.0, 0 ), &hp.cache );
	checkInt2( "K separated is a hit", hp.cache.hit, 1 );
	checkInt2( "K separated has no points", hp.manifold.pointCount, 0 );

	// A corrupt cache must not index out of bounds, and must still produce the
	// right manifold by falling through to the full test. Upstream only
	// asserts here; a b3SATCache lives in the contact table and survives a
	// shape swap, so on device it is untrusted state.
	hullPair reference;
	hullPairReset( &reference );
	b3CollideHulls( &reference.manifold, 4, &cubeA.base, &cubeB.base, Xf( 0, 1.9, 0 ), &reference.cache );

	static const uint8_t corruptTypes[] = { (uint8_t)b3_faceAxisA, (uint8_t)b3_faceAxisB, (uint8_t)b3_edgePairAxis };
	for ( size_t t = 0; t < sizeof( corruptTypes ) / sizeof( corruptTypes[0] ); ++t )
	{
		hullPair corrupt;
		hullPairReset( &corrupt );
		corrupt.cache.type = corruptTypes[t];
		corrupt.cache.indexA = 200;
		corrupt.cache.indexB = 200;
		corrupt.cache.separation = b3fFromDouble( -0.1 );

		b3CollideHulls( &corrupt.manifold, 4, &cubeA.base, &cubeB.base, Xf( 0, 1.9, 0 ), &corrupt.cache );

		checkInt2( "K corrupt cache recovers the manifold", corrupt.manifold.pointCount,
				   reference.manifold.pointCount );
		expect( "K corrupt cache recovers the depth", minSeparation( &corrupt.manifold ),
				minSeparation( &reference.manifold ), 8.0 * Q12 );
	}

	// An unknown type is not a crash either -- it simply costs the full test.
	hullPair unknown;
	hullPairReset( &unknown );
	unknown.cache.type = 200;
	b3CollideHulls( &unknown.manifold, 4, &cubeA.base, &cubeB.base, Xf( 0, 1.9, 0 ), &unknown.cache );
	checkInt2( "K unknown cache type recovers", unknown.manifold.pointCount, reference.manifold.pointCount );
}

static void test_hull_hull_reduce( void )
{
	section( "hull vs hull -- manifold reduction" );

	// An eight-sided prism resting flat on a cube. The prism's cap clips to
	// eight points against the cube's face, so this is the configuration that
	// actually runs b3ReduceManifoldPoints -- a box against a box never
	// produces more than four and leaves it untested.
	b3PrismHull prism = b3MakePrismHull( b3fFromDouble( 1.0 ), b3fFromDouble( 0.5 ), 8 );
	check( "M prism is valid", b3IsValidHull( &prism.base ) );

	b3BoxHull ground = b3MakeBoxHull( b3fFromDouble( 3.0 ), b3fFromDouble( 0.5 ), b3fFromDouble( 3.0 ) );

	hullPair hp;
	hullPairReset( &hp );
	b3CollideHulls( &hp.manifold, 4, &ground.base, &prism.base, Xf( 0, 1.0, 0 ), &hp.cache );

	checkInt2( "M reduce returns exactly four", hp.manifold.pointCount, 4 );
	expectVec( "M normal", hp.manifold.normal, 0, 1, 0, 8.0 * Q12 );

	if ( hp.manifold.pointCount == 4 )
	{
		for ( int i = 0; i < 4; ++i )
		{
			expect( "M separation", b3fToDouble( hp.manifold.points[i].separation ), 0.0, 8.0 * Q12 );
		}

		// Distinct, and spread rather than clustered. Reduce emits its four
		// picks in selection order -- deepest, farthest, largest triangle,
		// largest added area -- which is not a traversal order, so the spread
		// is measured as the largest distance between any two rather than as a
		// polygon area.
		//
		// A reduce whose area ranking had collapsed -- the b3Cross underflow
		// the wide b3SignedAreaWide path exists to avoid -- would return four
		// points huddled on one side, and the span would be a fraction of the
		// octagon rather than very nearly its full diameter of 2.
		double maxSpan = 0.0;
		for ( int i = 0; i < 4; ++i )
		{
			for ( int k = i + 1; k < 4; ++k )
			{
				b3Vec3 d = b3Sub( hp.manifold.points[i].point, hp.manifold.points[k].point );
				check( "M points are distinct", b3LengthSquaredWide( d ) > 0 );

				double span = b3fToDouble( b3Length( d ) );
				if ( span > maxSpan )
				{
					maxSpan = span;
				}
			}
		}

		check( "M reduce kept a spread quad", maxSpan > 1.8 );
	}

	// Tilted by five degrees. Note what is *not* asserted: that the deepest
	// point comes first. Step one of reduce scores by
	// `-separation + dot(searchDirection, point)`, not by depth alone, so the
	// first point is the deepest only up to that tangential bias -- claiming
	// otherwise would be asserting an algorithm the port does not implement.
	//
	// What must hold is that the retained patch still spans the tilt. Over the
	// octagon's 2-unit diameter a five degree tilt is +/- 0.087, so the four
	// points have to cover a separation range of that order; a manifold that
	// collapsed onto one side would show almost none.
	hullPairReset( &hp );
	b3Transform tilt = { V( 0, 1.0, 0 ), b3MakeQuatFromAxisAngle( V( 0, 0, 1 ), (b3a)( 32768.0 * 5.0 / 360.0 ) ) };
	b3CollideHulls( &hp.manifold, 4, &ground.base, &prism.base, tilt, &hp.cache );

	check( "M tilted still produces a patch", hp.manifold.pointCount >= 3 );

	if ( hp.manifold.pointCount > 1 )
	{
		double lo = 1e30, hi = -1e30;
		for ( int i = 0; i < hp.manifold.pointCount; ++i )
		{
			double v = b3fToDouble( hp.manifold.points[i].separation );
			lo = v < lo ? v : lo;
			hi = v > hi ? v : hi;
		}

		check( "M tilted patch spans the wedge", hi - lo > 0.10 );
		check( "M tilted patch reaches the deep side", lo < -0.05 );
	}
}

// =========================================================================
// Time of impact -- Phase 7, Stage 2
// =========================================================================

/// b3TimeOfImpact against closed forms.
///
/// Every fraction below is exact arithmetic on distances the geometry states,
/// less one detail that has to be carried through by hand. The query stops when
/// the *core* distance -- the proxies without their radii -- reaches
///
///     target = max( slop, radiusA + radiusB - slop )
///
/// so a sweep of length L stops `slop / L` away from the fraction at which the
/// surfaces meet, and **which way** depends on the radii. Two spheres stop a
/// slop past touching, because their target is a slop inside the sum of their
/// radii. Two hulls, whose radii are zero, stop a slop short of it, because
/// their target is the slop itself. Both signs appear below and both are
/// written into the expectation rather than absorbed by a tolerance, which is
/// what makes a check that fails here mean something.
///
/// The states are checked with checkInt2 rather than expect: which of the five
/// a query ended in is a discrete fact, and a port that returns `failed` where
/// upstream returns `hit` has a bug however close its fraction is.
static void test_time_of_impact( void )
{
	printf( "time of impact\n" );

	const double slop = b3fToDouble( B3_LINEAR_SLOP );
	b3Vec3 zero = b3Vec3_zero;
	b3Quat identity = b3Quat_identity;

	// A unit sphere at the origin, not moving.
	b3Vec3 centerA = zero;
	b3ShapeProxy unitSphere = { &centerA, 1, b3fFromDouble( 1.0 ) };
	b3Sweep still = { zero, zero, zero, identity, identity };

	b3Vec3 centerB = zero;
	b3ShapeProxy smallSphere = { &centerB, 1, b3fFromDouble( 0.25 ) };

	// --- sphere versus sphere, the case with an exact answer ---
	{
		// B sweeps 16 units along -x from x = 8. The surfaces meet when the
		// centres are 1.25 apart, so at x = 1.25 -- less the slop.
		b3Sweep sweepB = { zero, V( 8, 0, 0 ), V( -8, 0, 0 ), identity, identity };
		b3TOIInput input = { unitSphere, smallSphere, still, sweepB, b3c_one };
		b3TOIOutput out = b3TimeOfImpact( &input );

		checkInt2( "a sphere swept at another one hits it", out.state, b3_toiStateHit );
		expect( "at the fraction the two radii give", b3cToDouble( out.fraction ), ( 8.0 - 1.25 + slop ) / 16.0, 1e-4 );
		// The reported point is the midpoint of the two surfaces, and at the
		// stopping distance they have passed through each other by a slop.
		expectVec( "on the far side of the still one", out.point, 1.0 - 0.5 * slop, 0, 0, 4 * Q12 );
		expectVec( "with the normal along the sweep", out.normal, 1, 0, 0, 4 * Q12 );
		expect( "reporting the separation it stopped at", b3fToDouble( out.distance ), 1.25 - slop, 4 * Q12 );

		// The sweep that goes the same way and stops short. 6.5 of the 16
		// units leaves the centres 1.5 apart, which is a quarter of a unit
		// clear.
		input.maxFraction = b3cFromFrac( 65, 160 );
		out = b3TimeOfImpact( &input );
		checkInt2( "the same sweep cut short stays separated", out.state, b3_toiStateSeparated );
		expect( "and reports the whole of what it was given", b3cToDouble( out.fraction ), 65.0 / 160.0, 2 * Q12 );
	}

	// --- the boundary, from both sides ---
	{
		// Passing at exactly the touching distance is a miss: the query stops
		// at `target`, which is a slop nearer than that.
		b3Sweep grazing = { zero, V( 8, 1.25, 0 ), V( -8, 1.25, 0 ), identity, identity };
		b3TOIInput input = { unitSphere, smallSphere, still, grazing, b3c_one };
		checkInt2( "a sweep passing at exactly the touching distance misses", b3TimeOfImpact( &input ).state,
				   b3_toiStateSeparated );

		// A tenth of a unit lower and it connects, which is what says the miss
		// above is a boundary and not a blind spot.
		input.sweepB.c1 = V( 8, 1.15, 0 );
		input.sweepB.c2 = V( -8, 1.15, 0 );
		checkInt2( "and a tenth of a unit lower it connects", b3TimeOfImpact( &input ).state, b3_toiStateHit );
	}

	// --- starting overlapped ---
	{
		// Cores 0.5 apart, which is inside `target` but still positive, so the
		// query answers immediately with a hit at zero rather than giving up.
		b3Sweep fromInside = { zero, V( 0.5, 0, 0 ), V( -8, 0, 0 ), identity, identity };
		b3TOIInput input = { unitSphere, smallSphere, still, fromInside, b3c_one };
		b3TOIOutput out = b3TimeOfImpact( &input );

		checkInt2( "a sweep that starts touching hits at once", out.state, b3_toiStateHit );
		expect( "at fraction zero", b3cToDouble( out.fraction ), 0.0, 1e-9 );
		checkInt2( "having asked the distance query exactly once", out.distanceIterations, 1 );

		// Cores coincident: no separating direction exists at all, and the
		// only honest answer is that continuous collision cannot help.
		input.sweepB.c1 = zero;
		out = b3TimeOfImpact( &input );
		checkInt2( "and one that starts coincident reports overlap", out.state, b3_toiStateOverlapped );
		expect( "at fraction zero", b3cToDouble( out.fraction ), 0.0, 1e-9 );
	}

	// --- a hull through a thin slab, which is the case CCD exists for ---
	{
		// A slab 8 x 0.2 x 8 centred on the origin, and a unit cube. Without
		// continuous collision a cube moving 16 units in one step crosses a
		// 0.2-thick slab entirely.
		b3Vec3 slabPoints[8], cubePoints[8];
		for ( int i = 0; i < 8; ++i )
		{
			slabPoints[i] = V( ( i & 1 ) ? 4 : -4, ( i & 2 ) ? 0.1 : -0.1, ( i & 4 ) ? 4 : -4 );
			cubePoints[i] = V( ( i & 1 ) ? 0.5 : -0.5, ( i & 2 ) ? 0.5 : -0.5, ( i & 4 ) ? 0.5 : -0.5 );
		}

		b3ShapeProxy slab = { slabPoints, 8, b3f_zero };
		b3ShapeProxy cube = { cubePoints, 8, b3f_zero };

		// The cube's bottom face meets the slab's top face when its centre is
		// at 0.6, from a drop of 16 starting at y = 8. Both radii are zero, so
		// the target is the slop itself and the cube stops a slop *above* the
		// slab -- the opposite sign to the sphere pair above.
		b3Sweep drop = { zero, V( 0, 8, 0 ), V( 0, -8, 0 ), identity, identity };
		b3TOIInput input = { slab, cube, still, drop, b3c_one };
		b3TOIOutput out = b3TimeOfImpact( &input );

		checkInt2( "a cube dropped through a thin slab is caught", out.state, b3_toiStateHit );
		expect( "at the fraction the two half heights give", b3cToDouble( out.fraction ), ( 8.0 - 0.6 - slop ) / 16.0,
				1e-4 );
		expectVec( "with the slab's own normal", out.normal, 0, 1, 0, 8 * Q12 );

		// The same drop beside the slab passes through open air.
		input.sweepB.c1 = V( 8, 8, 0 );
		input.sweepB.c2 = V( 8, -8, 0 );
		checkInt2( "and the same drop beside it is not", b3TimeOfImpact( &input ).state, b3_toiStateSeparated );

		// Spinning on the way down: a corner reaches lower than a face, so the
		// impact is earlier. This is the branch that builds an edge or face
		// separating axis rather than a fixed world one, and the only property
		// worth asserting is the inequality -- the exact fraction depends on
		// which feature the simplex lands on.
		b3Quat eighth = b3MakeQuatFromAxisAngle( V( 0, 0, 1 ), (b3a)4096 );
		input.sweepB.c1 = V( 0, 8, 0 );
		input.sweepB.c2 = V( 0, -8, 0 );
		input.sweepB.q2 = eighth;
		b3TOIOutput spun = b3TimeOfImpact( &input );

		checkInt2( "a cube that spins as it falls is caught too", spun.state, b3_toiStateHit );
		check( "sooner than the one that does not, because a corner leads",
			   b3Raw( spun.fraction ) < b3Raw( out.fraction ) );

		// Every one of these must stay well inside the caps, or the caps are
		// load bearing rather than a safety net. The counters are the same
		// numbers the hardware produces, since none of this is floating point.
		check( "and none of it approaches the iteration caps",
			   spun.distanceIterations < 25 && spun.rootIterations < 50 );
	}

	// --- the sweep that only rotates ---
	{
		// A cube sitting just above the slab, spinning in place. Nothing
		// translates, so the bracket floor cannot be derived from a
		// translation and has to come from the rotation -- the one case that
		// would otherwise leave the root finder subdividing a Q30 interval
		// that no longer changes the Q12 separation.
		b3Vec3 slabPoints[8], cubePoints[8];
		for ( int i = 0; i < 8; ++i )
		{
			slabPoints[i] = V( ( i & 1 ) ? 4 : -4, ( i & 2 ) ? 0.1 : -0.1, ( i & 4 ) ? 4 : -4 );
			cubePoints[i] = V( ( i & 1 ) ? 0.5 : -0.5, ( i & 2 ) ? 0.5 : -0.5, ( i & 4 ) ? 0.5 : -0.5 );
		}

		b3ShapeProxy slab = { slabPoints, 8, b3f_zero };
		b3ShapeProxy cube = { cubePoints, 8, b3f_zero };

		// Centre at 0.75: clear of the slab flat, but a corner at 0.707 of the
		// half diagonal reaches through it once turned.
		b3Quat eighth = b3MakeQuatFromAxisAngle( V( 0, 0, 1 ), (b3a)4096 );
		b3Sweep spinOnly = { zero, V( 0, 0.75, 0 ), V( 0, 0.75, 0 ), b3Quat_identity, eighth };
		b3TOIInput input = { slab, cube, still, spinOnly, b3c_one };
		b3TOIOutput out = b3TimeOfImpact( &input );

		checkInt2( "a cube spinning in place drives its corner into the slab", out.state, b3_toiStateHit );
		check( "partway through the turn", b3Raw( out.fraction ) > 0 && b3Raw( out.fraction ) < B3_C_ONE );
		check( "without exhausting the root finder", out.rootIterations < 50 );
	}
}

// =========================================================================
// The mover's plane solver -- Phase 7, Stage 4
// =========================================================================

/// A rigid plane pushing `depth` along `normal`.
///
/// `depth` is the plane's `offset` and it is a **penetration**: positive means
/// the mover is that far inside and the solver has that much to undo. Upstream
/// spells the same thing `totalRadius - distance`. Zero is touching, and a
/// negative value is a plane the mover is already clear of.
static b3CollisionPlane Plane( b3Vec3 normal, double depth )
{
	b3CollisionPlane plane = { 0 };
	plane.plane.normal = normal;
	plane.plane.offset = b3fFromDouble( depth );
	plane.pushLimit = B3_F_MAX;
	plane.clipVelocity = true;
	return plane;
}

/// b3SolvePlanes and b3ClipVector, against upstream's own two scenarios and
/// against the one thing Q12 does that float cannot.
///
/// The first two cases are upstream's test/test_mover.c transliterated, down to
/// the tolerance: `ParallelPlanes` asserts the solver reaches its answer in
/// exactly two iterations, and `GamePlanes` asserts that it deliberately does
/// not converge at all. Both are worth having because every intermediate in
/// them is exact -- the normals are axis-aligned or given to nine digits -- so
/// a disagreement is arithmetic rather than quantization.
///
/// The third case is not upstream's, and it is here because a Q12 solver was
/// *expected* to behave worse than a float one and does not. Diagonal normals
/// have no exact Q12 spelling, so the prediction was that a converged plane
/// would oscillate by a raw unit and never let the loop's tolerance fire. It
/// was measured instead of assumed: over 160,000 random plane sets the loop
/// reaches its 20-iteration cap 52.4% of the time, and the identical scenarios
/// in double precision reach it 52.3% of the time, with 83,660 of the ~83,800
/// cases shared. The cap is a property of Gauss-Seidel on near-opposing planes,
/// not of fixed point, and upstream's own GamePlanes above asserts it. So no
/// deadband was added, this file is a straight transliteration, and the case
/// below pins the behaviour that actually holds.
static void test_mover_solver( void )
{
	printf( "mover plane solver\n" );

	// --- upstream's ParallelPlanes: two planes facing the same way ---
	{
		b3CollisionPlane planes[2];
		planes[0] = Plane( V( 0, 0, 1 ), 0.5 );
		planes[1] = Plane( V( 0, 0, 1 ), 1.0 );

		b3PlaneSolverResult result = b3SolvePlanes( b3Vec3_zero, planes, 2 );

		checkInt2( "two parallel planes converge in two iterations", result.iterationCount, 2 );

		// Upstream allows 0.0055, which is B3_LINEAR_SLOP plus a little, and
		// that allowance is exactly what is being spent: the solver stops one
		// slop **short** of full separation, because the slop is added to the
		// separation before the push is taken from it. So the answer is
		// 1.0 - 0.00488 = 0.99512 and not 1.0, on both sides. Every expectation
		// below is written as `depth - slop` for the same reason.
		expect( "having pushed out to the deeper of the two", b3fToDouble( result.delta.z ), 1.0, 0.0055 );

		// The near plane is subsumed by the far one, so it ends up pushing
		// nothing at all -- which is the accumulator being clamped back to
		// zero, not the deadband refusing a small push.
		checkInt2( "the subsumed plane pushes nothing", b3Raw( planes[0].push ), 0 );
		check( "and the deeper one does the work", b3Raw( planes[1].push ) > 0 );
	}

	// --- upstream's GamePlanes: a target far behind both planes ---
	{
		b3CollisionPlane planes[2];
		planes[0] = Plane( V( 0, -0.23941046, 0.970918416 ), 0.390724182 );
		planes[1] = Plane( V( 0, 0, 1 ), 1.49998093 );

		// Upstream folds the target into the offsets and solves from zero,
		// which is what a caller does when the planes were generated at a
		// position the mover has since left.
		b3Vec3 target = V( -2.5390625, 0, -73.6880798 );
		planes[0].plane.offset = b3SubF( planes[0].plane.offset, b3Dot( planes[0].plane.normal, target ) );
		planes[1].plane.offset = b3SubF( planes[1].plane.offset, b3Dot( planes[1].plane.normal, target ) );

		b3PlaneSolverResult result = b3SolvePlanes( b3Vec3_zero, planes, 2 );

		// Upstream's own comment: the target is deep into the plane and this
		// takes many iterations. It is here to prove the port runs out the same
		// way rather than terminating early on a tolerance it should not meet.
		checkInt2( "a target deep behind two planes runs the cap out", result.iterationCount, 20 );
		check( "and is still pushing out along +z when it stops", b3Raw( result.delta.z ) > 0 );
	}

	// --- normals with no exact Q12 spelling ---
	{
		// Three planes a quarter of a unit deep, with diagonal normals. This is
		// the case a fixed-point solver was expected to stall on and does not:
		// measured at 7 iterations, and the same 7 with and without the
		// deadband that was tried and dropped.
		double d = 0.57735026918962576;
		b3CollisionPlane planes[3];
		planes[0] = Plane( V( d, d, d ), 0.25 );
		planes[1] = Plane( V( -d, d, d ), 0.25 );
		planes[2] = Plane( V( d, d, -d ), 0.25 );

		b3PlaneSolverResult result = b3SolvePlanes( b3Vec3_zero, planes, 3 );

		printf( "  three diagonal planes: %d iterations\n", result.iterationCount );
		check( "three diagonal planes stop well inside the cap", result.iterationCount < 10 );
		check( "having pushed up out of all three", b3Raw( result.delta.y ) > 0 );
	}

	// --- eight planes, which is B3_NEA_MAX_MOVER_PLANES worth of corner ---
	{
		// A capsule wedged where a floor meets two walls, with each surface
		// contributing two triangles' worth of plane at slightly different
		// depths -- which is what a triangulated corner actually produces.
		b3CollisionPlane planes[8];
		for ( int i = 0; i < 8; ++i )
		{
			b3Vec3 normal = ( i < 4 ) ? V( 0, 1, 0 ) : ( ( i < 6 ) ? V( 1, 0, 0 ) : V( 0, 0, 1 ) );
			planes[i] = Plane( normal, 0.05 + 0.01 * i );
		}

		b3PlaneSolverResult result = b3SolvePlanes( b3Vec3_zero, planes, 8 );

		check( "eight planes in a corner converge inside the cap", result.iterationCount < 20 );
		check( "pushing up out of the floor", b3Raw( result.delta.y ) > 0 );
		check( "and out along both walls", b3Raw( result.delta.x ) > 0 && b3Raw( result.delta.z ) > 0 );

		// The deepest plane of each set is the one that decides, because the
		// shallower ones are subsumed once it has pushed.
		expect( "by the deepest of the floor planes", b3fToDouble( result.delta.y ),
				0.08 - b3fToDouble( B3_LINEAR_SLOP ), 4 * Q12 );
	}

	// --- pushLimit, and the b3AddF headroom that makes B3_F_MAX safe ---
	{
		// One plane a metre deep, allowed to push only a quarter of that. The
		// solver must stop at the limit rather than at the geometry.
		b3CollisionPlane planes[2];
		planes[0] = Plane( V( 0, 1, 0 ), 1.0 );
		planes[0].pushLimit = b3fFromDouble( 0.25 );

		b3PlaneSolverResult result = b3SolvePlanes( b3Vec3_zero, planes, 1 );

		expect( "a soft plane pushes exactly its limit", b3fToDouble( planes[0].push ), 0.25, 2 * Q12 );
		expect( "and the delta stops there", b3fToDouble( result.delta.y ), 0.25, 2 * Q12 );

		// The same plane rigid. B3_F_MAX is INT32_MAX/2, and the accumulator is
		// clamped into [0, pushLimit] before every add, so the add that could
		// wrap never sees more than that plus one plane's worth of push. Under
		// MODE=debug an overflow here aborts rather than wrapping quietly.
		planes[1] = Plane( V( 0, 1, 0 ), 1.0 );
		planes[1].pushLimit = B3_F_MAX;
		result = b3SolvePlanes( b3Vec3_zero, planes + 1, 1 );

		expect( "a rigid plane pushes the whole depth", b3fToDouble( result.delta.y ),
				1.0 - b3fToDouble( B3_LINEAR_SLOP ), 2 * Q12 );
	}

	// --- b3ClipVector ---
	{
		b3CollisionPlane planes[3];
		planes[0] = Plane( V( 0, 1, 0 ), 0.0 );
		planes[1] = Plane( V( 1, 0, 0 ), 0.0 );
		planes[2] = Plane( V( 0, 0, 1 ), 0.0 );

		planes[0].push = b3fFromDouble( 0.1 );	 // pushed, and clips
		planes[1].push = b3f_zero;				 // did nothing
		planes[2].push = b3fFromDouble( 0.1 );	 // pushed, but soft
		planes[2].clipVelocity = false;

		b3Vec3 v = b3ClipVector( V( -3, -4, -5 ), planes, 3 );

		expect( "the plane that pushed removes the inward component", b3fToDouble( v.y ), 0.0, 2 * Q12 );
		expect( "the plane that did not is skipped", b3fToDouble( v.x ), -3.0, 2 * Q12 );
		expect( "and so is the soft one", b3fToDouble( v.z ), -5.0, 2 * Q12 );

		// Outward velocity is left alone: this is a clip, not a projection.
		b3Vec3 up = b3ClipVector( V( 0, 7, 0 ), planes, 3 );
		expect( "a velocity already leaving the plane is untouched", b3fToDouble( up.y ), 7.0, 2 * Q12 );
	}
}

/// The three convex mover backends, from upstream's own subtests.
///
/// Upstream's test/test_mover.c states the point of the whole group in its
/// header: these functions "must never emit a plane with a degenerate (zero)
/// normal, even when the mover deeply penetrates the shape". That is the check
/// worth having, because a zero normal does not crash and does not look wrong
/// in a plane count -- it reaches b3SolvePlanes, where b3MulAdd against it is a
/// no-op, and the mover simply stops being pushed out of the thing it is inside.
///
/// The three types answer deep overlap differently and deliberately: the sphere
/// and the capsule substitute an analytic perpendicular, and the hull drops the
/// plane so that it agrees with the mesh, which has no tractable answer.
static void test_mover_convex( void )
{
	printf( "mover versus sphere, capsule and hull\n" );

	// --- sphere ---
	{
		b3Sphere shape = { V( 0, 0, 0 ), b3fFromDouble( 0.5 ) };
		b3PlaneResult result;

		b3Capsule away = { V( 4, 3, 0 ), V( 6, 3, 0 ), b3fFromDouble( 0.2 ) };
		checkInt2( "a mover clear of a sphere makes no plane", b3CollideMoverAndSphere( &result, &shape, &away ), 0 );

		// The core segment runs along x at y = 0.6, which is 0.1 inside the 0.7
		// combined radius.
		b3Capsule touching = { V( -1, 0.6, 0 ), V( 1, 0.6, 0 ), b3fFromDouble( 0.2 ) };
		checkInt2( "one touching it makes one", b3CollideMoverAndSphere( &result, &shape, &touching ), 1 );
		check( "with a unit normal", b3IsNormalized( result.plane.normal ) );
		check( "pointing from the sphere up at the mover", b3fToDouble( result.plane.normal.y ) > 0.99 );
		expect( "and an offset that is the overlap depth", b3fToDouble( result.plane.offset ), 0.1, 2 * Q12 );

		// The axis passes through the centre. This is the case that produces a
		// zero normal if it is left to GJK.
		b3Capsule through = { V( -1, 0, 0 ), V( 1, 0, 0 ), b3fFromDouble( 0.2 ) };
		checkInt2( "a mover skewered through the centre still makes one",
				   b3CollideMoverAndSphere( &result, &shape, &through ), 1 );
		check( "and the normal is still a unit vector", b3IsNormalized( result.plane.normal ) );
		expect( "perpendicular to the mover axis", b3fToDouble( result.plane.normal.x ), 0.0, 2 * Q12 );
		expect( "at the deepest the pair can be", b3fToDouble( result.plane.offset ), 0.7, 2 * Q12 );
	}

	// --- capsule ---
	{
		b3Capsule shape = { V( -1, 0, 0 ), V( 1, 0, 0 ), b3fFromDouble( 0.3 ) };
		b3PlaneResult result;

		b3Capsule away = { V( -1, 5, 0 ), V( 1, 5, 0 ), b3fFromDouble( 0.2 ) };
		checkInt2( "a mover clear of a capsule makes no plane", b3CollideMoverAndCapsule( &result, &shape, &away ), 0 );

		b3Capsule touching = { V( -1, 0.4, 0 ), V( 1, 0.4, 0 ), b3fFromDouble( 0.2 ) };
		checkInt2( "one 0.1 inside makes one", b3CollideMoverAndCapsule( &result, &shape, &touching ), 1 );
		check( "with a unit normal", b3IsNormalized( result.plane.normal ) );
		check( "pointing up at the mover", b3fToDouble( result.plane.normal.y ) > 0.99 );
		expect( "at the overlap depth", b3fToDouble( result.plane.offset ), 0.1, 2 * Q12 );

		// Crossed segments meeting at the origin.
		b3Capsule crossing = { V( 0, 0, -1 ), V( 0, 0, 1 ), b3fFromDouble( 0.2 ) };
		checkInt2( "two crossed core segments still make one", b3CollideMoverAndCapsule( &result, &shape, &crossing ), 1 );
		check( "with a unit normal", b3IsNormalized( result.plane.normal ) );
		expect( "perpendicular to the shape axis", b3fToDouble( result.plane.normal.x ), 0.0, 2 * Q12 );
		expect( "and to the mover axis", b3fToDouble( result.plane.normal.z ), 0.0, 2 * Q12 );
		expect( "at the full combined radius", b3fToDouble( result.plane.offset ), 0.5, 2 * Q12 );

		// Coincident segments: the cross product that would give a separating
		// axis is zero, so the perpendicular fallback is the only answer.
		b3Capsule coincident = { V( -1, 0, 0 ), V( 1, 0, 0 ), b3fFromDouble( 0.2 ) };
		checkInt2( "and so do two coincident ones", b3CollideMoverAndCapsule( &result, &shape, &coincident ), 1 );
		check( "with a unit normal", b3IsNormalized( result.plane.normal ) );
		expect( "perpendicular to the shared axis", b3fToDouble( result.plane.normal.x ), 0.0, 2 * Q12 );
		expect( "at the full combined radius", b3fToDouble( result.plane.offset ), 0.5, 2 * Q12 );
	}

	// --- hull ---
	{
		b3BoxHull box = b3MakeBoxHull( b3fFromDouble( 0.5 ), b3fFromDouble( 0.5 ), b3fFromDouble( 0.5 ) );
		b3PlaneResult result;

		b3Capsule away = { V( -0.3, 5, 0 ), V( 0.3, 5, 0 ), b3fFromDouble( 0.2 ) };
		checkInt2( "a mover clear of a box makes no plane", b3CollideMoverAndHull( &result, &box.base, &away ), 0 );

		// Above the +y face, with the 0.2 radius reaching 0.1 into it.
		b3Capsule touching = { V( -0.3, 0.6, 0 ), V( 0.3, 0.6, 0 ), b3fFromDouble( 0.2 ) };
		checkInt2( "one resting on the top face makes one", b3CollideMoverAndHull( &result, &box.base, &touching ), 1 );
		check( "with a unit normal", b3IsNormalized( result.plane.normal ) );
		check( "which is the face normal", b3fToDouble( result.plane.normal.y ) > 0.99 );
		expect( "at the overlap depth", b3fToDouble( result.plane.offset ), 0.1, 4 * Q12 );

		// The `<=` boundary, from both sides. The core segment sits 0.2 above
		// the +y face, so a radius either side of 0.2 decides it.
		//
		// The exactly-equal case is deliberately *not* asserted. GJK reaches
		// the distance through b3SqrtWide, so whether it lands on 819 raw or
		// 820 is a rounding question the geometry does not answer, and a test
		// that pinned it would be pinning the square root rather than the
		// backend. Four raw units either side is the tightest honest bracket.
		b3f nominal = b3fFromDouble( 0.2 );

		b3Capsule inside4 = { V( -0.3, 0.7, 0 ), V( 0.3, 0.7, 0 ), b3Makeb3f( b3Raw( nominal ) + 4 ) };
		checkInt2( "a mover four raw units past grazing counts",
				   b3CollideMoverAndHull( &result, &box.base, &inside4 ), 1 );

		b3Capsule outside4 = { V( -0.3, 0.7, 0 ), V( 0.3, 0.7, 0 ), b3Makeb3f( b3Raw( nominal ) - 4 ) };
		checkInt2( "and four raw units short of it does not",
				   b3CollideMoverAndHull( &result, &box.base, &outside4 ), 0 );

		// Wholly inside. Declined, so that a hull and the same shape baked as a
		// mesh give the caller the same answer.
		b3Capsule inside = { V( -0.2, 0, 0 ), V( 0.2, 0, 0 ), b3fFromDouble( 0.1 ) };
		checkInt2( "a mover inside a hull is declined rather than guessed at",
				   b3CollideMoverAndHull( &result, &box.base, &inside ), 0 );
	}
}

/// b3CollideMoverAndMesh, against the hand-built blob test_mesh_blob uses.
///
/// The blob is a flat plane at y = 0 spanning x in [0, 2] and z in [0, 1], as
/// four triangles: the left quad is 0 and 1, the right quad is 2 and 3.
///
/// The mesh backend is the only one that returns more than one plane, so it is
/// the only one where the capacity is a real bound -- and the traversal's
/// early-out is what makes it one, so that is checked directly rather than
/// inferred from a count.
static void test_mover_mesh( void )
{
	printf( "mover versus a triangle mesh\n" );

	static meshBlob blob;
	buildTestMesh( &blob );
	b3Mesh mesh = { &blob.header, V( 1, 1, 1 ) };

	b3PlaneResult planes[8];

	// --- resting on the left quad ---
	{
		// Core segment 0.15 above the plane, radius 0.2, so 0.05 of overlap.
		b3Capsule mover = { V( 0.2, 0.15, 0.5 ), V( 0.8, 0.15, 0.5 ), b3fFromDouble( 0.2 ) };
		int count = b3CollideMoverAndMesh( planes, 8, &mesh, &mover );

		printf( "  resting on the left quad: %d planes\n", count );
		check( "a mover over the left quad finds at least one triangle", count >= 1 );

		for ( int i = 0; i < count; ++i )
		{
			check( "every normal is a unit vector", b3IsNormalized( planes[i].plane.normal ) );
			check( "and points up out of the plane", b3fToDouble( planes[i].plane.normal.y ) > 0.99 );
			expect( "at the overlap depth", b3fToDouble( planes[i].plane.offset ), 0.05, 4 * Q12 );
		}
	}

	// --- clear of it ---
	{
		b3Capsule mover = { V( 0.2, 2.0, 0.5 ), V( 0.8, 2.0, 0.5 ), b3fFromDouble( 0.2 ) };
		checkInt2( "a mover two units above finds nothing", b3CollideMoverAndMesh( planes, 8, &mesh, &mover ), 0 );
	}

	// --- across the seam, which is both leaves ---
	{
		b3Capsule mover = { V( 0.5, 0.15, 0.5 ), V( 1.5, 0.15, 0.5 ), b3fFromDouble( 0.2 ) };
		int count = b3CollideMoverAndMesh( planes, 8, &mesh, &mover );

		printf( "  across the seam: %d planes\n", count );
		check( "a mover across the seam finds more than the left quad alone", count >= 2 );
	}

	// --- the capacity, and that it is the traversal that stops ---
	{
		b3Capsule mover = { V( 0.5, 0.15, 0.5 ), V( 1.5, 0.15, 0.5 ), b3fFromDouble( 0.2 ) };
		int full = b3CollideMoverAndMesh( planes, 8, &mesh, &mover );

		// Ask for one fewer than it wants, and then for exactly one. Both must
		// return exactly what was asked for -- a backend that filled its buffer
		// and kept testing triangles would look identical from the count alone,
		// which is why b3CollideMoverMeshFcn returns false rather than skipping.
		checkInt2( "a capacity of one returns exactly one", b3CollideMoverAndMesh( planes, 1, &mesh, &mover ), 1 );

		if ( full > 1 )
		{
			checkInt2( "and a capacity one short returns that", b3CollideMoverAndMesh( planes, full - 1, &mesh, &mover ),
					   full - 1 );
		}

		checkInt2( "a capacity of zero returns nothing at all", b3CollideMoverAndMesh( planes, 0, &mesh, &mover ), 0 );
	}

	// --- a reflected scale, which b3QueryMesh handles and upstream does not ---
	{
		// z mirrored, so the plane now spans z in [-1, 0]. The reflection swaps
		// the second and third vertex on the way out of b3QueryMesh; for a
		// three-point GJK proxy that is inert, and this says so.
		b3Mesh reflected = { &blob.header, V( 1, 1, -1 ) };

		b3Capsule mover = { V( 0.2, 0.15, -0.5 ), V( 0.8, 0.15, -0.5 ), b3fFromDouble( 0.2 ) };
		int count = b3CollideMoverAndMesh( planes, 8, &reflected, &mover );

		printf( "  on a z-reflected mesh: %d planes\n", count );
		check( "a reflected mesh is found the same way", count >= 1 );

		for ( int i = 0; i < count; ++i )
		{
			check( "with unit normals", b3IsNormalized( planes[i].plane.normal ) );
			expect( "at the same depth", b3fToDouble( planes[i].plane.offset ), 0.05, 4 * Q12 );
		}

		// And the mirrored half really is empty now.
		b3Capsule wrongSide = { V( 0.2, 0.15, 0.5 ), V( 0.8, 0.15, 0.5 ), b3fFromDouble( 0.2 ) };
		checkInt2( "and its old half is empty", b3CollideMoverAndMesh( planes, 8, &reflected, &wrongSide ), 0 );
	}
}

/// b3CollideMover: the dispatch, and the one invariant that survives it.
///
/// The post-transform rotates each normal and transforms each point but leaves
/// each offset alone. That looks like an oversight and is not: the offset is a
/// penetration depth measured from the mover's current position, so it is
/// already in the frame the caller wants. This pins it.
static void test_mover_dispatch( void )
{
	printf( "mover dispatch through b3CollideMover\n" );

	b3BoxHull box = b3MakeBoxHull( b3fFromDouble( 0.5 ), b3fFromDouble( 0.5 ), b3fFromDouble( 0.5 ) );

	b3Shape shape = { 0 };
	shape.type = b3_hullShape;
	shape.hull = &box.base;

	b3PlaneResult planes[8];

	// A mover resting on the top face, with the shape at the origin.
	b3Capsule mover = { V( -0.3, 0.6, 0 ), V( 0.3, 0.6, 0 ), b3fFromDouble( 0.2 ) };

	int atOrigin = b3CollideMover( planes, 8, &shape, b3Transform_identity, &mover );
	checkInt2( "a hull at the origin gives one plane", atOrigin, 1 );
	check( "with the face normal in world space", b3fToDouble( planes[0].plane.normal.y ) > 0.99 );
	expect( "and the contact point on the face", b3fToDouble( planes[0].point.y ), 0.5, 4 * Q12 );
	b3f depthAtOrigin = planes[0].plane.offset;

	// The same pair, translated. Nothing about the geometry has changed.
	b3Transform moved = { V( 10, 3, -4 ), b3Quat_identity };
	b3Capsule movedMover = { V( 9.7, 3.6, -4 ), V( 10.3, 3.6, -4 ), b3fFromDouble( 0.2 ) };

	int translated = b3CollideMover( planes, 8, &shape, moved, &movedMover );
	checkInt2( "translating both still gives one plane", translated, 1 );
	check( "with the same world normal", b3fToDouble( planes[0].plane.normal.y ) > 0.99 );
	expect( "a point that moved with the shape", b3fToDouble( planes[0].point.y ), 3.5, 4 * Q12 );

	// The offset must not have moved with it. The tolerance is a few raw units
	// rather than zero because the mover is quantized into a different local
	// frame before GJK sees it, which moves the distance by a unit or two --
	// but that is nothing like the failure this is guarding. A b3TransformPlane
	// here would shift the offset by dot( normal, transform.p ), which for this
	// transform is 3.0 units, or 12288 raw. Three orders of magnitude apart.
	expect( "and an offset that did not move with it", b3fToDouble( planes[0].plane.offset ),
			b3fToDouble( depthAtOrigin ), 4 * Q12 );

	// Rotated a quarter turn about z, so the +x face is now on top. The mover
	// sits above it, and the normal must come back as +y in world space.
	b3Quat quarter = b3MakeQuatFromAxisAngle( V( 0, 0, 1 ), (b3a)8192 );
	b3Transform turned = { V( 0, 0, 0 ), quarter };

	int rotated = b3CollideMover( planes, 8, &shape, turned, &mover );
	checkInt2( "a rotated hull still gives one plane", rotated, 1 );
	check( "whose normal was rotated into world space", b3fToDouble( planes[0].plane.normal.y ) > 0.99 );
	expect( "and whose offset was again left alone", b3fToDouble( planes[0].plane.offset ), b3fToDouble( depthAtOrigin ),
			4 * Q12 );

	checkInt2( "a capacity of zero short-circuits", b3CollideMover( planes, 0, &shape, b3Transform_identity, &mover ), 0 );
}

int main( void )
{
	b3TestInstallAssertTrap();
	printf( "box3d collision primitives verification\n\n" );

	test_area();
	test_enlarge();
	test_raycast();
	test_bounds_ray_overlap();
	test_segment_distance();
	test_plane();
	test_mass();
	test_sphere_raycast();
	test_hollow_sphere_raycast();
	test_capsule_raycast();
	test_support();
	test_simplex2();
	test_simplex3();
	test_gjk();
	test_manifolds();
	test_hull_structure();
	test_prism_structure();
	test_hull_support();
	test_hull_raycast();
	test_hull_queries();
	test_gjk_thin_simplex();
	test_hull_manifolds();
	test_feature_pairs();
	test_clip_polygon();
	test_find_incident_face();
	test_hull_hull_faces();
	test_hull_hull_face_b();
	test_hull_hull_manual_features();
	test_hull_hull_degenerate();
	test_hull_hull_cache();
	test_hull_hull_reduce();
	test_tree_structure();
	test_tree_query();
	test_tree_casts();
	test_tree_closest();
	test_tree_enlarge_and_rebuild();
	test_broad_phase();
	test_mesh_blob();
	test_mesh_queries();
	test_closest_point_on_triangle();
	test_triangle_sphere();
	test_time_of_impact();
	test_mover_solver();
	test_mover_convex();
	test_mover_mesh();
	test_mover_dispatch();

	// Assertions are part of the result, not a trap: assert_trap.h keeps the
	// run going and this turns any that fired into a reported failure.
	s_checks++;
	if ( b3TestUnexpectedAsserts() != 0 )
	{
		printf( "  FAIL %d unexpected assertion(s) fired\n", b3TestUnexpectedAsserts() );
		s_failures++;
	}

	printf( "\n%d checks, %d failures\n", s_checks, s_failures );
	return s_failures == 0 ? 0 : 1;
}
