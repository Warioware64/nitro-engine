// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Warioware64
//
// This file is part of Nitro Engine Advanced

// Verification of the fixed-point scalar layer against exact double
// arithmetic. Every operation in b3fixed.h is exercised over its intended
// range and the result compared to the value a float build would produce.
//
// This is the bottom of the tower: if these tolerances do not hold, nothing
// built on top of them can. Run it before and after any change to b3fixed.h.
//
//   make -f Makefile.host test_fixed && ./test_fixed

#include "box3d/b3fixed.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "assert_trap.h"

static int s_failures = 0;
static int s_checks = 0;

// Compares a fixed result against the exact value, allowing an error of
// `quanta` times the destination's resolution. Anything larger means the
// operation is not merely quantizing.
static void expect( const char* what, double got, double want, double quantum, double quanta )
{
	s_checks++;
	double tol = quantum * quanta;
	double diff = fabs( got - want );

	// Scale the tolerance with magnitude: a Q12 product of two large numbers
	// legitimately carries error proportional to the operands.
	double mag = fabs( want );
	if ( mag > 1.0 )
	{
		tol *= mag;
	}

	if ( diff > tol )
	{
		printf( "  FAIL %-28s got %-16.9g want %-16.9g diff %.4g (tol %.4g)\n", what, got, want, diff, tol );
		s_failures++;
	}
}

static void section( const char* name )
{
	printf( "%s\n", name );
}

// -------------------------------------------------------------------------

static void test_construction( void )
{
	section( "construction and round-trip" );

	expect( "fFromInt(5)", b3fToDouble( b3fFromInt( 5 ) ), 5.0, 1.0 / 4096.0, 1.0 );
	expect( "fFromInt(-1234)", b3fToDouble( b3fFromInt( -1234 ) ), -1234.0, 1.0 / 4096.0, 1.0 );

	// The reason b3t exists at all: 1/240 is 17.06 in Q12 (a 0.4% error) but
	// is carried to eight significant figures in Q24.
	double h = b3tToDouble( b3tFromFrac( 1, 240 ) );
	expect( "tFromFrac(1,240)", h, 1.0 / 240.0, 1.0 / 16777216.0, 2.0 );

	double dt = b3tToDouble( b3tFromFrac( 1, 60 ) );
	expect( "tFromFrac(1,60)", dt, 1.0 / 60.0, 1.0 / 16777216.0, 2.0 );

	// Demonstrate the failure the scale avoids, so a future change that
	// collapses b3t into b3f trips this test rather than the solver.
	double hAtQ12 = (double)( ( 1 << 12 ) / 240 ) / 4096.0;
	if ( fabs( hAtQ12 - 1.0 / 240.0 ) < 1e-5 )
	{
		printf( "  FAIL Q12 substep unexpectedly accurate -- premise of b3t is wrong\n" );
		s_failures++;
	}
	s_checks++;

	expect( "cFromFrac(1,3)", b3cToDouble( b3cFromFrac( 1, 3 ) ), 1.0 / 3.0, 1.0 / 1073741824.0, 2.0 );
	expect( "nFromFrac(-1,2)", b3nToDouble( b3nFromFrac( -1, 2 ) ), -0.5, 1.0 / 1073741824.0, 2.0 );
}

static void test_arithmetic( void )
{
	section( "same-scale arithmetic" );

	b3f a = b3fFromDouble( 12.5 );
	b3f b = b3fFromDouble( -3.25 );

	expect( "addF", b3fToDouble( b3AddF( a, b ) ), 9.25, 1.0 / 4096.0, 1.0 );
	expect( "subF", b3fToDouble( b3SubF( a, b ) ), 15.75, 1.0 / 4096.0, 1.0 );
	expect( "negF", b3fToDouble( b3NegF( a ) ), -12.5, 1.0 / 4096.0, 1.0 );
	expect( "absF", b3fToDouble( b3AbsF( b ) ), 3.25, 1.0 / 4096.0, 1.0 );
	expect( "minF", b3fToDouble( b3MinF( a, b ) ), -3.25, 1.0 / 4096.0, 1.0 );
	expect( "maxF", b3fToDouble( b3MaxF( a, b ) ), 12.5, 1.0 / 4096.0, 1.0 );

	b3f lo = b3fFromDouble( 0.0 );
	b3f hi = b3fFromDouble( 10.0 );
	expect( "clampF above", b3fToDouble( b3ClampF( a, lo, hi ) ), 10.0, 1.0 / 4096.0, 1.0 );
	expect( "clampF below", b3fToDouble( b3ClampF( b, lo, hi ) ), 0.0, 1.0 / 4096.0, 1.0 );

	s_checks++;
	if ( !b3LtF( b, a ) || !b3GtF( a, b ) || !b3EqzF( b3f_zero ) )
	{
		printf( "  FAIL comparison predicates\n" );
		s_failures++;
	}
}

static void test_multiply( void )
{
	section( "cross-scale multiply" );

	expect( "mulFF", b3fToDouble( b3MulFF( b3fFromDouble( 3.5 ), b3fFromDouble( -2.25 ) ) ), -7.875, 1.0 / 4096.0, 4.0 );
	expect( "mulFN", b3fToDouble( b3MulFN( b3fFromDouble( 40.0 ), b3nFromDouble( 0.5 ) ) ), 20.0, 1.0 / 4096.0, 4.0 );
	expect( "mulNN", b3nToDouble( b3MulNN( b3nFromDouble( 0.6 ), b3nFromDouble( 0.8 ) ) ), 0.48, 1.0 / 1073741824.0, 4.0 );

	// invMass of a 1000 kg body is 0.001 -- four quanta in Q12, sixteen
	// thousand in Q24. This is the case that decides the b3iw scale.
	b3iw w = b3RcpF( b3fFromDouble( 1000.0 ) );
	expect( "rcpF(1000)", b3iwToDouble( w ), 0.001, 1.0 / 16777216.0, 64.0 );

	// force * invMass -> acceleration
	expect( "mulFW", b3fToDouble( b3MulFW( b3fFromDouble( 500.0 ), w ) ), 0.5, 1.0 / 4096.0, 4.0 );

	// velocity * substep -> position delta, the integration inner loop
	b3t h = b3tFromFrac( 1, 240 );
	expect( "mulFT", b3fToDouble( b3MulFT( b3fFromDouble( 24.0 ), h ) ), 0.1, 1.0 / 4096.0, 4.0 );

	expect( "mulFC", b3fToDouble( b3MulFC( b3fFromDouble( 8.0 ), b3cFromFrac( 1, 4 ) ) ), 2.0, 1.0 / 4096.0, 4.0 );
	expect( "mulCC", b3cToDouble( b3MulCC( b3cFromFrac( 1, 2 ), b3cFromFrac( 1, 4 ) ) ), 0.125, 1.0 / 1073741824.0, 4.0 );
	expect( "mulWW", b3iwToDouble( b3MulWW( b3iwFromDouble( 0.25 ), b3iwFromDouble( 0.5 ) ) ), 0.125, 1.0 / 16777216.0, 4.0 );
	expect( "mulWN", b3iwToDouble( b3MulWN( b3iwFromDouble( 0.04 ), b3nFromDouble( -0.5 ) ) ), -0.02, 1.0 / 16777216.0, 4.0 );

	// accumulated impulse through an inverse mass -> velocity change
	b3imp j = b3FToImp( b3fFromDouble( 12.0 ) );
	expect( "mulImpW", b3fToDouble( b3MulImpW( j, b3iwFromDouble( 0.25 ) ) ), 3.0, 1.0 / 4096.0, 4.0 );
	expect( "mulImpC", b3impToDouble( b3MulImpC( j, b3cFromFrac( 1, 3 ) ) ), 4.0, 1.0 / 65536.0, 4.0 );
	expect( "mulImpN", b3fToDouble( b3MulImpN( j, b3nFromDouble( 0.5 ) ) ), 6.0, 1.0 / 4096.0, 4.0 );

	// The two directions a joint converts between a force bound and an impulse
	// bound. A motor's maxMotorForce is a force; the accumulator it clamps is
	// an impulse; and a reaction query turns that accumulator back into the
	// force it reports. Round-tripping through both is the check that matters
	// -- either alone could be off by a shift and still look plausible.
	expect( "mulFTToImp", b3impToDouble( b3MulFTToImp( b3fFromDouble( 240.0 ), h ) ), 1.0, 1.0 / 65536.0, 4.0 );
	expect( "mulImpFToF", b3fToDouble( b3MulImpFToF( j, b3fFromInt( 240 ) ) ), 2880.0, 1.0 / 4096.0, 4.0 );

	{
		// force -> impulse -> force, at the default substep. inv_h is the Q12
		// reciprocal of h, per B3_NEA_INV_DT's reasoning.
		b3f force = b3fFromDouble( 37.5 );
		b3imp impulse = b3MulFTToImp( force, h );
		expect( "force impulse round trip", b3fToDouble( b3MulImpFToF( impulse, b3fFromInt( 240 ) ) ), 37.5,
				1.0 / 4096.0, 64.0 );
	}
}

static void test_conversions( void )
{
	section( "scale conversions" );

	expect( "FToN round trip", b3fToDouble( b3NToF( b3FToN( b3fFromDouble( 0.75 ) ) ) ), 0.75, 1.0 / 4096.0, 1.0 );
	expect( "FToW round trip", b3fToDouble( b3WToF( b3FToW( b3fFromDouble( 3.5 ) ) ) ), 3.5, 1.0 / 4096.0, 1.0 );
	expect( "FToImp round trip", b3fToDouble( b3ImpToF( b3FToImp( b3fFromDouble( -9.25 ) ) ) ), -9.25, 1.0 / 4096.0, 1.0 );

	// Saturating conversion must clamp rather than wrap. A wrap here would
	// turn an out-of-range coefficient into one of the opposite sign, which
	// in a solver means energy injection rather than damping.
	expect( "FToCSat high", b3cToDouble( b3FToCSat( b3fFromDouble( 5.0 ) ) ), 2.0, 1.0, 1.0 );
	expect( "FToCSat low", b3cToDouble( b3FToCSat( b3fFromDouble( -5.0 ) ) ), -2.0, 1.0, 1.0 );
	expect( "FToCSat in range", b3cToDouble( b3FToCSat( b3fFromDouble( 0.5 ) ) ), 0.5, 1.0 / 1073741824.0, 4.0 );
}

static void test_divide( void )
{
	section( "hardware divide" );

	expect( "divFF", b3fToDouble( b3DivFF( b3fFromDouble( 7.0 ), b3fFromDouble( 2.0 ) ) ), 3.5, 1.0 / 4096.0, 4.0 );
	expect( "divFF negative", b3fToDouble( b3DivFF( b3fFromDouble( -9.0 ), b3fFromDouble( 4.0 ) ) ), -2.25, 1.0 / 4096.0, 4.0 );

	// Division by zero must not trap or produce a wild value. Box3D relies on
	// float's 1/0 -> inf in places; fixed point has no such value, so the
	// convention is zero and callers guard explicitly.
	expect( "divFF by zero", b3fToDouble( b3DivFF( b3fFromDouble( 1.0 ), b3f_zero ) ), 0.0, 1.0 / 4096.0, 1.0 );
	expect( "rcpF(0)", b3iwToDouble( b3RcpF( b3f_zero ) ), 0.0, 1.0 / 16777216.0, 1.0 );

	expect( "divFFToC", b3cToDouble( b3DivFFToC( b3fFromDouble( 1.0 ), b3fFromDouble( 3.0 ) ) ), 1.0 / 3.0, 1.0 / 1073741824.0,
			8.0 );
	expect( "divFFToW", b3iwToDouble( b3DivFFToW( b3fFromDouble( 1.0 ), b3fFromDouble( 64.0 ) ) ), 0.015625, 1.0 / 16777216.0,
			4.0 );

	// The async form must agree with the synchronous one.
	b3DivFFStart( b3fFromDouble( 10.0 ), b3fFromDouble( 4.0 ) );
	expect( "divFF async", b3fToDouble( b3DivFFCollect() ), 2.5, 1.0 / 4096.0, 4.0 );

	// A mass below the Q12 quantum rounds to zero, which by Box3D's
	// convention means infinite mass. Documented behaviour, not a failure --
	// but assert it so a future change to b3RcpF cannot alter it silently.
	s_checks++;
	if ( b3iwToDouble( b3RcpF( b3fFromDouble( 0.0001 ) ) ) != 0.0 )
	{
		printf( "  FAIL sub-quantum mass should read as infinite (invMass 0)\n" );
		s_failures++;
	}

	// Just above the quantum, the inverse must clamp rather than overflow the
	// 32-bit divider result. This is the case that decides B3_MIN_MASS_RAW.
	b3iw light = b3RcpF( b3fFromDouble( 0.001 ) );
	s_checks++;
	if ( b3iwToDouble( light ) <= 0.0 || b3iwToDouble( light ) > 127.99 )
	{
		printf( "  FAIL rcpF of a very light mass: %g (must be in (0, 128))\n", b3iwToDouble( light ) );
		s_failures++;
	}

	// The lightest mass with an exact inverse, at the clamp boundary.
	b3iw atLimit = b3RcpF( b3Makeb3f( B3_MIN_MASS_RAW ) );
	expect( "rcpF at min mass", b3iwToDouble( atLimit ), 4096.0 / (double)B3_MIN_MASS_RAW, 1.0 / 16777216.0, 64.0 );

	// Zero over anything is zero. This looks too obvious to test, and it was
	// wrong for four phases.
	//
	// b3DivWideToF measures its numerator with b3Clz64, which called
	// __builtin_clz( 0 ) for a zero numerator -- undefined, and the optimizer
	// took different views at different levels. At -O1 it produced a bit length
	// large enough to enter the saturation branch, so 0/x returned B3_F_MAX.
	//
	// Nothing reached it until Phase 6 Stage 4. b3InvertMatrixW divides every
	// cofactor by the determinant, and an exactly diagonal matrix has six
	// cofactors that are exactly zero -- so an isotropic inverse inertia, which
	// is what a sphere has, inverted to a matrix with six saturated
	// off-diagonals. A contact's lever arms and a hinge's axes never land a
	// cofactor exactly on zero, which is why four phases of tests missed it.
	{
		const int64_t denominators[5] = { 1, 4096, 73728000, -4096, INT32_MAX };
		for ( int i = 0; i < 5; ++i )
		{
			char buf[64];
			snprintf( buf, sizeof( buf ), "zero over %lld divides to zero", (long long)denominators[i] );
			expect( buf, b3fToDouble( b3DivWideToF( 0, denominators[i] ) ), 0.0, 0.0, 0.0 );
		}

		// And the guard itself, at the boundary between its three branches.
		expect( "clz64 of zero is 64", (double)b3Clz64( 0 ), 64.0, 1.0, 0.0 );
		expect( "clz64 of one is 63", (double)b3Clz64( 1 ), 63.0, 1.0, 0.0 );
		expect( "clz64 of 2^32 is 31", (double)b3Clz64( (uint64_t)1 << 32 ), 31.0, 1.0, 0.0 );
	}
}

static void test_sqrt( void )
{
	section( "square root" );

	expect( "sqrtF(4)", b3fToDouble( b3SqrtF( b3fFromDouble( 4.0 ) ) ), 2.0, 1.0 / 4096.0, 4.0 );
	expect( "sqrtF(2)", b3fToDouble( b3SqrtF( b3fFromDouble( 2.0 ) ) ), sqrt( 2.0 ), 1.0 / 4096.0, 4.0 );
	expect( "sqrtF(0)", b3fToDouble( b3SqrtF( b3f_zero ) ), 0.0, 1.0 / 4096.0, 1.0 );
	expect( "sqrtF negative", b3fToDouble( b3SqrtF( b3fFromDouble( -1.0 ) ) ), 0.0, 1.0 / 4096.0, 1.0 );

	// The wide form is the one the vector length routines use. Build the
	// squared length the way they do -- accumulate Q12 products into an int64
	// at Q24 without narrowing -- and check a vector large enough that the
	// naive (dot << 12) path would have overflowed.
	double comps[3] = { 400.0, 300.0, 1200.0 };
	int64_t sq = 0;
	for ( int i = 0; i < 3; i++ )
	{
		int64_t c = (int64_t)b3Raw( b3fFromDouble( comps[i] ) );
		sq += c * c;
	}
	double wantLen = sqrt( comps[0] * comps[0] + comps[1] * comps[1] + comps[2] * comps[2] );
	expect( "sqrtWide large vector", b3fToDouble( b3SqrtWide( sq ) ), wantLen, 1.0 / 4096.0, 8.0 );

	// And the reciprocal form used for normalization. It returns b3iw, not
	// b3c: a reciprocal length exceeds 2 for any vector shorter than half a
	// unit, which Q30 cannot represent at all.
	expect( "rsqrtWide", b3iwToDouble( b3RsqrtWide( sq ) ), 1.0 / wantLen, 1.0 / 16777216.0, 64.0 );

	// Normalizing with rsqrtWide must produce a unit vector.
	b3iw inv = b3RsqrtWide( sq );
	double n2 = 0.0;
	for ( int i = 0; i < 3; i++ )
	{
		double comp = b3fToDouble( b3MulFW( b3fFromDouble( comps[i] ), inv ) );
		n2 += comp * comp;
	}
	expect( "normalized length", sqrt( n2 ), 1.0, 1.0 / 4096.0, 8.0 );

	// Short vectors are the case the b3c version got wrong. 1/0.25 is 4,
	// which does not fit Q30 but sits comfortably in Q24.
	int64_t shortSq = 0;
	double shortComps[3] = { 0.15, 0.2, 0.0 }; // length 0.25
	for ( int i = 0; i < 3; i++ )
	{
		int64_t c = (int64_t)b3Raw( b3fFromDouble( shortComps[i] ) );
		shortSq += c * c;
	}
	// The tolerance here is loose on purpose. (0.15, 0.2) quantizes to a
	// length of 0.24993 rather than 0.25, so the reciprocal is 4.0011 before
	// the operation does anything -- the input error dominates. What this
	// checks is that the result is near 4 at all, which the old b3c version
	// could not manage because 4 does not fit Q30.
	expect( "rsqrtWide of a short vector", b3iwToDouble( b3RsqrtWide( shortSq ) ), 1.0 / 0.25, 0.01, 1.0 );
}

static void test_trig( void )
{
	section( "trigonometry" );

	// sinLerp/cosLerp are 4.12, so the achievable accuracy is one Q12 quantum
	// regardless of the Q30 container.
	expect( "sin(0)", b3cToDouble( b3SinA( 0 ) ), 0.0, 1.0 / 4096.0, 2.0 );
	expect( "sin(pi/2)", b3cToDouble( b3SinA( B3_BRAD_HALF_PI ) ), 1.0, 1.0 / 4096.0, 2.0 );
	expect( "cos(0)", b3cToDouble( b3CosA( 0 ) ), 1.0, 1.0 / 4096.0, 2.0 );
	expect( "cos(pi)", b3cToDouble( b3CosA( B3_BRAD_PI ) ), -1.0, 1.0 / 4096.0, 2.0 );

	// Sweep the circle and check the Pythagorean identity, which catches a
	// sign or quadrant error anywhere in the table mapping.
	for ( int i = 0; i < 32; i++ )
	{
		b3a angle = (b3a)( i * ( B3_BRAD_CIRCLE / 32 ) - B3_BRAD_PI );
		double s = b3cToDouble( b3SinA( angle ) );
		double c = b3cToDouble( b3CosA( angle ) );
		expect( "sin^2+cos^2", s * s + c * c, 1.0, 1.0 / 4096.0, 8.0 );
	}

	expect( "acos(1)", (double)b3AcosC( b3c_one ), 0.0, 1.0, 8.0 );
	expect( "acos(0)", (double)b3AcosC( b3c_zero ), (double)B3_BRAD_HALF_PI, 1.0, 32.0 );

	// atan2 across all four quadrants. Tolerance is in brads; the circle is
	// 32768 brads, so 64 brads is about 0.7 degrees.
	struct
	{
		double y, x, deg;
	} cases[] = {
		{ 0.0, 1.0, 0.0 },    { 1.0, 1.0, 45.0 },    { 1.0, 0.0, 90.0 },    { 1.0, -1.0, 135.0 },
		{ 0.0, -1.0, 180.0 }, { -1.0, -1.0, -135.0 }, { -1.0, 0.0, -90.0 }, { -1.0, 1.0, -45.0 },
		{ 3.0, 4.0, 36.8699 }, { -12.0, 5.0, -67.3801 },
	};

	for ( size_t i = 0; i < sizeof( cases ) / sizeof( cases[0] ); i++ )
	{
		b3a got = b3Atan2F( b3fFromDouble( cases[i].y ), b3fFromDouble( cases[i].x ) );
		double wantBrad = cases[i].deg * ( 32768.0 / 360.0 );

		// 180 and -180 are the same angle; fold the wrap before comparing.
		double diff = (double)got - wantBrad;
		while ( diff > 16384.0 )
		{
			diff -= 32768.0;
		}
		while ( diff < -16384.0 )
		{
			diff += 32768.0;
		}

		char label[64];
		snprintf( label, sizeof( label ), "atan2(%g,%g)", cases[i].y, cases[i].x );
		expect( label, diff, 0.0, 1.0, 64.0 );
	}

	expect( "atan2(0,0)", (double)b3Atan2F( b3f_zero, b3f_zero ), 0.0, 1.0, 1.0 );
}

// The integration loop in miniature: drop a body under gravity for one second
// of substeps and compare against the closed form. This is the end-to-end
// check that the chosen scales survive being composed.
static void test_integration_drift( void )
{
	section( "integration drift over 240 substeps" );

	b3t h = b3tFromFrac( 1, 240 );
	b3f gravity = b3fFromDouble( -9.8 );

	b3f pos = b3f_zero;
	b3f vel = b3f_zero;

	for ( int i = 0; i < 240; i++ )
	{
		vel = b3AddF( vel, b3MulFT( gravity, h ) );
		pos = b3AddF( pos, b3MulFT( vel, h ) );
	}

	// Semi-implicit Euler lands at v = g*t and p = g*t*(t+h)/2, not the
	// continuous g*t^2/2. Compare against what the float build would produce
	// with the identical scheme, so this measures fixed-point drift only.
	double dt = 1.0 / 240.0;
	double refVel = 0.0;
	double refPos = 0.0;
	for ( int i = 0; i < 240; i++ )
	{
		refVel += -9.8 * dt;
		refPos += refVel * dt;
	}

	expect( "velocity after 1s", b3fToDouble( vel ), refVel, 1.0 / 4096.0, 64.0 );
	expect( "position after 1s", b3fToDouble( pos ), refPos, 1.0 / 4096.0, 64.0 );

	printf( "  velocity %.6f (ref %.6f)   position %.6f (ref %.6f)\n", b3fToDouble( vel ), refVel, b3fToDouble( pos ), refPos );
}

int main( void )
{
	b3TestInstallAssertTrap();

#if defined( B3_FIXED_DEBUG )
	b3FixedResetStats();
	printf( "b3fixed verification  [B3_FIXED_DEBUG: shadow values active]\n\n" );
#elif defined( B3_FIXED_STRICT )
	printf( "b3fixed verification  [B3_FIXED_STRICT: distinct scale types]\n\n" );
#else
	printf( "b3fixed verification  [device mode: bare int32_t]\n\n" );
#endif

	test_construction();
	test_arithmetic();
	test_multiply();
	test_conversions();
	test_divide();
	test_sqrt();
	test_trig();
	test_integration_drift();

#if defined( B3_FIXED_DEBUG )
	printf( "\n" );
	b3FixedReport( "test_fixed" );
#endif

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
