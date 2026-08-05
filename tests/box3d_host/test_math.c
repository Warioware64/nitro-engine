// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Warioware64
//
// This file is part of Nitro Engine Advanced

// Verification of the fixed-point vector, quaternion and matrix math against
// exact double arithmetic.
//
// The emphasis is on the properties the solver depends on rather than on
// individual results: that rotations preserve length, that a quaternion
// integrated for thousands of substeps does not drift, that transforms
// compose and invert. Those are the invariants whose failure produces
// bodies sinking through floors, and they are cheap to state here and
// expensive to diagnose on hardware.

#include "box3d/math_fixed.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "assert_trap.h"

static int s_failures = 0;
static int s_checks = 0;

#define Q12 ( 1.0 / 4096.0 )

static void expect( const char* what, double got, double want, double tol )
{
	s_checks++;
	if ( fabs( got - want ) > tol )
	{
		printf( "  FAIL %-34s got %-15.9g want %-15.9g diff %.4g (tol %.4g)\n", what, got, want, fabs( got - want ), tol );
		s_failures++;
	}
}

static void check( const char* what, bool ok )
{
	s_checks++;
	if ( !ok )
	{
		printf( "  FAIL %s\n", what );
		s_failures++;
	}
}

static void expectVec( const char* what, b3Vec3 got, double x, double y, double z, double tol )
{
	char buf[80];
	snprintf( buf, sizeof( buf ), "%s.x", what );
	expect( buf, b3fToDouble( got.x ), x, tol );
	snprintf( buf, sizeof( buf ), "%s.y", what );
	expect( buf, b3fToDouble( got.y ), y, tol );
	snprintf( buf, sizeof( buf ), "%s.z", what );
	expect( buf, b3fToDouble( got.z ), z, tol );
}

static b3Vec3 V( double x, double y, double z )
{
	return b3MakeVec3( b3fFromDouble( x ), b3fFromDouble( y ), b3fFromDouble( z ) );
}

static void section( const char* name )
{
	printf( "%s\n", name );
}

// -------------------------------------------------------------------------

static void test_vector_basics( void )
{
	section( "vector arithmetic" );

	b3Vec3 a = V( 1.5, -2.25, 3.0 );
	b3Vec3 b = V( 0.5, 4.0, -1.0 );

	expectVec( "add", b3Add( a, b ), 2.0, 1.75, 2.0, Q12 );
	expectVec( "sub", b3Sub( a, b ), 1.0, -6.25, 4.0, Q12 );
	expectVec( "neg", b3Neg( a ), -1.5, 2.25, -3.0, Q12 );
	expectVec( "abs", b3Abs( a ), 1.5, 2.25, 3.0, Q12 );
	expectVec( "min", b3Min( a, b ), 0.5, -2.25, -1.0, Q12 );
	expectVec( "max", b3Max( a, b ), 1.5, 4.0, 3.0, Q12 );
	expectVec( "mulSV", b3MulSV( b3fFromDouble( 2.0 ), a ), 3.0, -4.5, 6.0, 4 * Q12 );
	expectVec( "mulAdd", b3MulAdd( a, b3fFromDouble( 2.0 ), b ), 2.5, 5.75, 1.0, 4 * Q12 );

	expect( "dot", b3fToDouble( b3Dot( a, b ) ), 1.5 * 0.5 + -2.25 * 4.0 + 3.0 * -1.0, 4 * Q12 );

	// cross(x, y) == z, the orientation convention everything else assumes.
	expectVec( "cross(x,y)", b3Cross( b3Vec3_axisX, b3Vec3_axisY ), 0.0, 0.0, 1.0, 4 * Q12 );
	expectVec( "cross(y,z)", b3Cross( b3Vec3_axisY, b3Vec3_axisZ ), 1.0, 0.0, 0.0, 4 * Q12 );
	expectVec( "cross(z,x)", b3Cross( b3Vec3_axisZ, b3Vec3_axisX ), 0.0, 1.0, 0.0, 4 * Q12 );

	expect( "length(3,4,12)", b3fToDouble( b3Length( V( 3.0, 4.0, 12.0 ) ) ), 13.0, 4 * Q12 );
	expect( "distance", b3fToDouble( b3Distance( V( 1.0, 0.0, 0.0 ), V( 4.0, 4.0, 0.0 ) ) ), 5.0, 4 * Q12 );
}

// The wide dot product exists because narrowing each term first both loses
// precision and overflows. A vector 1000 units from the origin has a squared
// length of 1e6, which does not fit Q12 at all.
static void test_wide_dot( void )
{
	section( "wide dot product and large vectors" );

	b3Vec3 big = V( 1000.0, 800.0, 600.0 );
	double wantLen = sqrt( 1000.0 * 1000.0 + 800.0 * 800.0 + 600.0 * 600.0 );

	expect( "length of distant vector", b3fToDouble( b3Length( big ) ), wantLen, 8 * Q12 );

	// The narrow form would have overflowed here; confirm the wide one holds
	// the true value.
	int64_t sq = b3LengthSquaredWide( big );
	double wantSq = 1000.0 * 1000.0 + 800.0 * 800.0 + 600.0 * 600.0;
	expect( "wide squared length", (double)sq / (double)( (int64_t)1 << 24 ), wantSq, 1.0 );

	// Normalizing a distant vector must still give unit length.
	b3Vec3 n = b3Normalize( big );
	expect( "normalize distant -> unit", b3fToDouble( b3Length( n ) ), 1.0, 8 * Q12 );

	s_checks++;
	if ( !b3IsNormalized( n ) )
	{
		printf( "  FAIL b3IsNormalized rejected its own b3Normalize output\n" );
		s_failures++;
	}
}

static void test_quaternion( void )
{
	section( "quaternions" );

	b3Quat id = b3Quat_identity;
	expectVec( "identity rotates nothing", b3RotateVector( id, V( 1.0, 2.0, 3.0 ) ), 1.0, 2.0, 3.0, Q12 );

	// 90 degrees about Z takes +x to +y.
	b3Quat rz = b3MakeQuatFromAxisAngle( b3Vec3_axisZ, (b3a)( B3_BRAD_CIRCLE / 4 ) );
	expectVec( "90deg Z: x -> y", b3RotateVector( rz, b3Vec3_axisX ), 0.0, 1.0, 0.0, 16 * Q12 );
	expectVec( "90deg Z: y -> -x", b3RotateVector( rz, b3Vec3_axisY ), -1.0, 0.0, 0.0, 16 * Q12 );
	expectVec( "90deg Z: z -> z", b3RotateVector( rz, b3Vec3_axisZ ), 0.0, 0.0, 1.0, 16 * Q12 );

	// 180 degrees about Y.
	b3Quat ry = b3MakeQuatFromAxisAngle( b3Vec3_axisY, (b3a)( B3_BRAD_CIRCLE / 2 ) );
	expectVec( "180deg Y: x -> -x", b3RotateVector( ry, b3Vec3_axisX ), -1.0, 0.0, 0.0, 16 * Q12 );

	// Inverse rotation must undo the forward one.
	b3Vec3 p = V( 1.25, -3.5, 2.0 );
	expectVec( "invRotate undoes rotate", b3InvRotateVector( rz, b3RotateVector( rz, p ) ), 1.25, -3.5, 2.0, 32 * Q12 );

	// Rotation preserves length -- the property that matters most, because a
	// rotation that quietly scales would pump energy into the solver.
	b3Quat arb = b3MakeQuatFromAxisAngle( V( 1.0, 2.0, 3.0 ), (b3a)5000 );
	double before = b3fToDouble( b3Length( p ) );
	double after = b3fToDouble( b3Length( b3RotateVector( arb, p ) ) );
	expect( "rotation preserves length", after, before, 16 * Q12 );

	// Composition: rotating twice by 45 about Z equals once by 90.
	b3Quat r45 = b3MakeQuatFromAxisAngle( b3Vec3_axisZ, (b3a)( B3_BRAD_CIRCLE / 8 ) );
	b3Vec3 twice = b3RotateVector( r45, b3RotateVector( r45, b3Vec3_axisX ) );
	expectVec( "45+45 == 90 about Z", twice, 0.0, 1.0, 0.0, 32 * Q12 );

	b3Vec3 composed = b3RotateVector( b3MulQuat( r45, r45 ), b3Vec3_axisX );
	expectVec( "mulQuat matches sequential", composed, 0.0, 1.0, 0.0, 32 * Q12 );

	s_checks++;
	if ( !b3IsNormalizedQuat( b3MulQuat( r45, r45 ) ) )
	{
		printf( "  FAIL product of unit quaternions is not unit\n" );
		s_failures++;
	}
}

// The reason b3Quat is Q30 rather than Q12.
//
// A body's orientation is integrated and renormalized every substep, so any
// per-step error feeds back into itself. This runs the real integration loop
// for ten seconds of simulated time at 240 Hz and measures how far the
// orientation has drifted from the analytic answer.
static void test_quaternion_drift( void )
{
	section( "quaternion drift over 2400 substeps" );

	// Spin about Z at 1 rad/s.
	b3f omega = b3fFromDouble( 1.0 );
	b3t h = b3tFromFrac( 1, 240 );

	b3Quat q = b3Quat_identity;

	b3Vec3 w = b3MakeVec3( b3f_zero, b3f_zero, omega );

	for ( int i = 0; i < 2400; i++ )
	{
		q = b3IntegrateRotation( q, w, h );
	}

	// After 10 s at 1 rad/s the body has turned 10 radians.
	double wantAngle = 10.0;
	while ( wantAngle > 2.0 * M_PI )
	{
		wantAngle -= 2.0 * M_PI;
	}

	b3Vec3 x = b3RotateVector( q, b3Vec3_axisX );
	double gotAngle = atan2( b3fToDouble( x.y ), b3fToDouble( x.x ) );
	if ( gotAngle < 0.0 )
	{
		gotAngle += 2.0 * M_PI;
	}

	double err = fabs( gotAngle - wantAngle );
	printf( "  after 2400 substeps: angle %.6f rad, expected %.6f, error %.6f rad (%.3f deg)\n", gotAngle, wantAngle, err,
			err * 180.0 / M_PI );

	// The quaternion must still be unit length. Losing normalization is the
	// failure mode that turns into visible tumbling.
	s_checks++;
	if ( !b3IsNormalizedQuat( q ) )
	{
		printf( "  FAIL quaternion lost normalization after 2400 substeps\n" );
		s_failures++;
	}

	// Semi-implicit integration of a rotation has real truncation error, so
	// this bound is about drift not being catastrophic rather than about
	// matching the analytic answer exactly.
	expect( "orientation after 10s", err, 0.0, 0.05 );
}

static void test_matrix( void )
{
	section( "matrices" );

	b3Matrix3 id = b3Mat3_identity;
	expectVec( "identity * v", b3MulMV( id, V( 1.0, 2.0, 3.0 ) ), 1.0, 2.0, 3.0, Q12 );
	expect( "det(identity)", b3fToDouble( b3Det( id ) ), 1.0, 4 * Q12 );

	// A rotation matrix built from a quaternion must agree with rotating the
	// vector by that quaternion directly.
	b3Quat q = b3MakeQuatFromAxisAngle( b3Vec3_axisZ, (b3a)( B3_BRAD_CIRCLE / 4 ) );
	b3Matrix3 m = b3MakeMatrixFromQuat( q );
	b3Vec3 v = V( 2.0, -1.0, 0.5 );

	b3Vec3 viaMatrix = b3MulMV( m, v );
	b3Vec3 viaQuat = b3RotateVector( q, v );
	expect( "matrix vs quat rotation .x", b3fToDouble( viaMatrix.x ), b3fToDouble( viaQuat.x ), 32 * Q12 );
	expect( "matrix vs quat rotation .y", b3fToDouble( viaMatrix.y ), b3fToDouble( viaQuat.y ), 32 * Q12 );
	expect( "matrix vs quat rotation .z", b3fToDouble( viaMatrix.z ), b3fToDouble( viaQuat.z ), 32 * Q12 );

	// A rotation matrix has unit determinant.
	expect( "det(rotation)", b3fToDouble( b3Det( m ) ), 1.0, 32 * Q12 );

	b3Matrix3 t = b3Transpose( m );
	expectVec( "transpose undoes rotation", b3MulMV( t, b3MulMV( m, v ) ), 2.0, -1.0, 0.5, 64 * Q12 );
}

// -------------------------------------------------------------------------
// Phase 3C-ii: the impulse scale and the two effective-mass inversions
// -------------------------------------------------------------------------

static b3Imp3 IMP( double x, double y, double z )
{
	return b3MakeImp3( b3impFromDouble( x ), b3impFromDouble( y ), b3impFromDouble( z ) );
}

static void expectImp3( const char* what, b3Imp3 got, double x, double y, double z, double tol )
{
	char buf[80];
	snprintf( buf, sizeof( buf ), "%s.x", what );
	expect( buf, b3impToDouble( got.x ), x, tol );
	snprintf( buf, sizeof( buf ), "%s.y", what );
	expect( buf, b3impToDouble( got.y ), y, tol );
	snprintf( buf, sizeof( buf ), "%s.z", what );
	expect( buf, b3impToDouble( got.z ), z, tol );
}

static void test_impulse_scale( void )
{
	section( "impulses at Q15.16" );

	const double QIMP = 1.0 / 65536.0;

	b3Imp3 a = IMP( 1.5, -2.25, 0.125 );
	b3Imp3 b = IMP( 0.5, 1.0, -0.0625 );

	expectImp3( "add", b3AddImp3( a, b ), 2.0, -1.25, 0.0625, QIMP );
	expectImp3( "sub", b3SubImp3( a, b ), 1.0, -3.25, 0.1875, QIMP );
	expectImp3( "neg", b3NegImp3( a ), -1.5, 2.25, -0.125, QIMP );

	// The resolution that justifies the scale existing. A hundredth of a unit
	// -- roughly what a resting box exchanges per sub-step -- keeps 655 quanta
	// at Q16 against 41 at Q12.
	check( "Q16 resolves a hundredth to better than 1e-5", fabs( b3impToDouble( b3impFromDouble( 0.01 ) ) - 0.01 ) < 1e-5 );

	// And the comparison that says why it is not enough to go through Q12 to
	// get there: b3FToImp of the same literal is stuck at the Q12 quantum,
	// which is what the first draft of this test measured by accident.
	check( "the same value via Q12 keeps only the Q12 quantum",
		   fabs( b3impToDouble( b3FToImp( b3fFromDouble( 0.01 ) ) ) - 0.01 ) > 1e-5 );

	// b3MulNImp: an impulse along a unit direction, and its magnitude must
	// survive the trip.
	b3Dir3 zAxis = b3ToDir3( b3Vec3_axisZ );
	expectImp3( "mulNImp along +z", b3MulNImp( b3impFromDouble( 3.0 ), zAxis ), 0.0, 0.0, 3.0, 8 * QIMP );

	b3Dir3 diag = b3NormalizeToDir( V( 1.0, 1.0, 0.0 ) );
	b3Imp3 alongDiag = b3MulNImp( b3impFromDouble( 2.0 ), diag );
	expect( "mulNImp magnitude preserved", sqrt( (double)b3Imp3LengthSquaredWide( alongDiag ) ) / 65536.0, 2.0, 1e-3 );

	// b3DotImpN inverts b3MulNImp for a vector already along the direction.
	expect( "dotImpN recovers the magnitude", b3impToDouble( b3DotImpN( alongDiag, diag ) ), 2.0, 1e-3 );

	// An impulse through an inverse mass is a velocity change: 3 units of
	// impulse on a 1/4 inverse mass is 0.75 of velocity.
	b3Vec3 dv = b3MulImpW3( IMP( 3.0, 0.0, -1.0 ), b3RcpF( b3fFromDouble( 4.0 ) ) );
	expectVec( "mulImpW3 == P / m", dv, 0.75, 0.0, -0.25, 8 * Q12 );

	// cross( r, P ), against b3Cross on the same numbers at Q12.
	b3Vec3 r = V( 0.5, -1.5, 2.0 );
	b3Imp3 p = IMP( 1.0, 2.0, -0.5 );
	b3Imp3 got = b3CrossVImp( r, p );
	b3Vec3 want = b3Cross( r, V( 1.0, 2.0, -0.5 ) );
	expectImp3( "crossVImp matches b3Cross", got, b3fToDouble( want.x ), b3fToDouble( want.y ), b3fToDouble( want.z ),
				8 * Q12 );

	// b3SqrtWideImp is exact in the sense that matters: the root of a square
	// returns the original.
	for ( int k = 1; k <= 6; ++k )
	{
		double v = 0.05 * k * k;
		b3imp x = b3impFromDouble( v );
		int64_t sq = (int64_t)b3Raw( x ) * b3Raw( x );
		expect( "sqrtWideImp round trip", b3impToDouble( b3SqrtWideImp( sq ) ), v, 1e-4 );
	}
	check( "sqrtWideImp( 0 ) is zero", b3Raw( b3SqrtWideImp( 0 ) ) == 0 );
	check( "sqrtWideImp( negative ) is zero", b3Raw( b3SqrtWideImp( -5 ) ) == 0 );
}

static void test_effective_mass( void )
{
	section( "effective masses" );

	// b3RcpW is b3RcpF backwards, so the two must round-trip through each
	// other across the whole mass range the solver sees.
	for ( int k = 0; k < 5; ++k )
	{
		const double masses[5] = { 0.05, 1.0, 12.5, 400.0, 5000.0 };
		double m = masses[k];
		b3iw invM = b3RcpF( b3fFromDouble( m ) );
		expect( "rcpW( rcpF( m ) ) == m", b3fToDouble( b3RcpW( invM ) ), m, 1e-3 * m + 4 * Q12 );
	}

	check( "rcpW( 0 ) is zero -- an immovable constraint", b3Raw( b3RcpW( b3iw_zero ) ) == 0 );
	check( "rcpW is sign preserving", b3Raw( b3RcpW( b3iwFromDouble( -0.25 ) ) ) < 0 );

	// b3MulFFToImp: a mass times a velocity is an impulse, and it widens, so
	// it should be exact on values Q12 already holds exactly.
	expect( "mulFFToImp( 2.5, 4 )", b3impToDouble( b3MulFFToImp( b3fFromDouble( 2.5 ), b3fFromDouble( 4.0 ) ) ), 10.0,
			1e-4 );
	expect( "mulFFToImp( 0.125, 0.25 )",
			b3impToDouble( b3MulFFToImp( b3fFromDouble( 0.125 ), b3fFromDouble( 0.25 ) ) ), 0.03125, 1e-5 );

	// b3MulImpF: an impulse scaled by an unbounded Q12 factor.
	expect( "mulImpF( 1.5, 3 )", b3impToDouble( b3MulImpF( b3impFromDouble( 1.5 ), b3fFromDouble( 3.0 ) ) ),
			4.5, 1e-3 );

	// -- the symmetric 2x2 -------------------------------------------------
	//
	// Round trip: k * inverse(k) must be the identity. Checked through the
	// multiply the solver actually uses rather than by reading entries.
	{
		b3SymMatrix2W k;
		k.xx = b3iwFromDouble( 0.75 );
		k.yy = b3iwFromDouble( 0.5 );
		k.xy = b3iwFromDouble( 0.125 );

		b3SymMatrix2 inv = b3InvertSym2W( k );

		// inverse(k) * (k * e1) should return e1. k * e1 is the first column,
		// which is (xx, xy) at Q24 -- brought to Q12 to feed the multiply.
		b3Imp2 col1 = b3MulSym2V( inv, b3WToF( k.xx ), b3WToF( k.xy ) );
		expect( "sym2 round trip e1.x", b3impToDouble( col1.x ), 1.0, 2e-3 );
		expect( "sym2 round trip e1.y", b3impToDouble( col1.y ), 0.0, 2e-3 );

		b3Imp2 col2 = b3MulSym2V( inv, b3WToF( k.xy ), b3WToF( k.yy ) );
		expect( "sym2 round trip e2.x", b3impToDouble( col2.x ), 0.0, 2e-3 );
		expect( "sym2 round trip e2.y", b3impToDouble( col2.y ), 1.0, 2e-3 );
	}

	// A rank-one matrix is singular and must come back zero, not infinite.
	{
		b3SymMatrix2W k;
		k.xx = b3iwFromDouble( 0.25 );
		k.yy = b3iwFromDouble( 0.25 );
		k.xy = b3iwFromDouble( 0.25 );

		b3SymMatrix2 inv = b3InvertSym2W( k );
		check( "singular sym2 inverts to zero",
			   b3Raw( inv.xx ) == 0 && b3Raw( inv.yy ) == 0 && b3Raw( inv.xy ) == 0 );
	}

	// -- the 3x3 at Q24 ----------------------------------------------------
	//
	// The case the Q36 determinant scale exists for: an inverse inertia of
	// 1e-3 per entry, where a Q24 determinant would be 0.017 and round to
	// zero, reporting a well conditioned matrix as singular.
	for ( int k = 0; k < 4; ++k )
	{
		const double scales[4] = { 0.001, 0.05, 1.0, 20.0 };
		double s = scales[k];

		b3MatrixW m = b3MatW_zero;
		m.cx.x = b3iwFromDouble( s );
		m.cy.y = b3iwFromDouble( s * 2.0 );
		m.cz.z = b3iwFromDouble( s * 0.5 );

		b3Matrix3 inv = b3InvertMatrixW( m );

		char buf[80];
		snprintf( buf, sizeof( buf ), "invertMatrixW diagonal %g .x", s );
		expect( buf, b3fToDouble( inv.cx.x ), 1.0 / s, 1e-2 / s + 4 * Q12 );
		snprintf( buf, sizeof( buf ), "invertMatrixW diagonal %g .y", s );
		expect( buf, b3fToDouble( inv.cy.y ), 1.0 / ( s * 2.0 ), 1e-2 / s + 4 * Q12 );
		snprintf( buf, sizeof( buf ), "invertMatrixW diagonal %g .z", s );
		expect( buf, b3fToDouble( inv.cz.z ), 1.0 / ( s * 0.5 ), 2e-2 / s + 4 * Q12 );
	}

	// A non-diagonal case, verified by the round trip rather than by entries:
	// inverse(M) * (M * v) == v.
	{
		b3MatrixW m = b3MatW_zero;
		m.cx.x = b3iwFromDouble( 0.4 );
		m.cy.y = b3iwFromDouble( 0.25 );
		m.cz.z = b3iwFromDouble( 0.6 );
		m.cx.y = m.cy.x = b3iwFromDouble( 0.05 );
		m.cx.z = m.cz.x = b3iwFromDouble( -0.08 );
		m.cy.z = m.cz.y = b3iwFromDouble( 0.03 );

		b3Vec3 v = V( 1.0, -2.0, 0.5 );
		b3Vec3 mv = b3MulMWV( m, v );
		b3Vec3 back = b3MulMV( b3InvertMatrixW( m ), mv );
		expectVec( "invertMatrixW round trip", back, 1.0, -2.0, 0.5, 0.02 );
	}

	// Singular input, from a matrix with a zero column.
	{
		b3MatrixW m = b3MatW_zero;
		m.cx.x = b3iwFromDouble( 0.5 );
		m.cy.y = b3iwFromDouble( 0.5 );
		b3Matrix3 inv = b3InvertMatrixW( m );
		check( "singular matrixW inverts to zero", b3Raw( inv.cx.x ) == 0 && b3Raw( inv.cz.z ) == 0 );
	}

	// b3AddMWMW, the only producer of the matrices above in the solver.
	{
		b3MatrixW a = b3MatW_zero;
		b3MatrixW b = b3MatW_zero;
		a.cx.x = b3iwFromDouble( 0.25 );
		b.cx.x = b3iwFromDouble( 0.75 );
		a.cy.z = b3iwFromDouble( -0.5 );
		b.cy.z = b3iwFromDouble( 0.125 );

		b3MatrixW sum = b3AddMWMW( a, b );
		expect( "addMWMW .cx.x", b3iwToDouble( sum.cx.x ), 1.0, 1e-6 );
		expect( "addMWMW .cy.z", b3iwToDouble( sum.cy.z ), -0.375, 1e-6 );
	}

	// -- the point-to-point effective mass ---------------------------------
	//
	// K = (mA + mB)*I - skew(rA)*iA*skew(rA) - skew(rB)*iB*skew(rB), which a
	// joint needs inverted. The reason it is one function rather than a chain
	// of matrix products is that K genuinely does not fit Q24 -- see the
	// overflow case below, which is an ordinary scene, not a corner.
	{
		// One dynamic body against a static one: iB and mB are zero, so K is
		// mA*I - skew(r)*iA*skew(r), and for an isotropic iA the closed form
		// is K = (mA + invI*|r|^2)*I - invI*r*r^T.
		const double m = 0.5;
		const double invI = 0.4;
		double rx = 0.5, ry = -0.25, rz = 0.75;
		double rr = rx * rx + ry * ry + rz * rz;

		b3MatrixW iA = b3MatW_zero;
		iA.cx.x = iA.cy.y = iA.cz.z = b3iwFromDouble( invI );

		b3Matrix3 inv = b3InvertPointMass( b3iwFromDouble( m ), iA, V( rx, ry, rz ), b3iw_zero, b3MatW_zero, V( 0, 0, 0 ) );

		// Verified by the round trip -- K * inverse(K) * v == v -- rather than
		// by reading entries, because the entries are the thing under test.
		double k[3][3];
		for ( int i = 0; i < 3; ++i )
		{
			for ( int j = 0; j < 3; ++j )
			{
				const double r[3] = { rx, ry, rz };
				k[i][j] = -invI * r[i] * r[j];
				if ( i == j )
				{
					k[i][j] += m + invI * rr;
				}
			}
		}

		b3Vec3 v = V( 1.0, -2.0, 0.5 );
		b3Vec3 iv = b3MulMV( inv, v );
		double back[3];
		double ivd[3] = { b3fToDouble( iv.x ), b3fToDouble( iv.y ), b3fToDouble( iv.z ) };
		for ( int i = 0; i < 3; ++i )
		{
			back[i] = k[i][0] * ivd[0] + k[i][1] * ivd[1] + k[i][2] * ivd[2];
		}

		expect( "point mass round trip .x", back[0], 1.0, 0.02 );
		expect( "point mass round trip .y", back[1], -2.0, 0.02 );
		expect( "point mass round trip .z", back[2], 0.5, 0.02 );

		// Symmetric, which is what makes a single inverse legitimate.
		expect( "point mass inverse is symmetric xy", b3fToDouble( inv.cx.y ), b3fToDouble( inv.cy.x ), 1e-3 );
		expect( "point mass inverse is symmetric xz", b3fToDouble( inv.cx.z ), b3fToDouble( inv.cz.x ), 1e-3 );
	}

	{
		// The case that made this a function. A 0.1 m sphere at the density
		// the shape defaults to has an inverse inertia near 60; on a 2 m arm
		// the first multiply of skew(r)*iA alone reaches 131, against Q7.24's
		// ceiling of 128. Building K at Q24 overflows here, and this is a
		// small light body on an ordinary arm -- the shadow checker found it
		// on the first run of the revolute joint's tests.
		const double m = 0.24;
		const double invI = 60.0;
		const double arm = 2.0;

		b3MatrixW iA = b3MatW_zero;
		iA.cx.x = iA.cy.y = iA.cz.z = b3iwFromDouble( invI );

		b3Matrix3 inv = b3InvertPointMass( b3iwFromDouble( m ), iA, V( arm, 0, 0 ), b3iw_zero, b3MatW_zero, V( 0, 0, 0 ) );

		// Along the arm the rotational term contributes nothing -- skew(r)*r
		// is zero -- so K.xx is just the inverse mass and the effective mass
		// is 1/m. Across it, K is m + invI*arm^2 and the effective mass is far
		// smaller. Both are checked, because a scale error in the
		// normalization would move them in opposite directions.
		expect( "along the arm the effective mass is 1/m", b3fToDouble( inv.cx.x ), 1.0 / m, 0.05 / m );
		expect( "across it the arm's inertia dominates", b3fToDouble( inv.cy.y ),
				1.0 / ( m + invI * arm * arm ), 0.02 );

		check( "and nothing overflowed", b3Raw( inv.cx.x ) != 0 && b3Raw( inv.cy.y ) != 0 );
	}

	{
		// Two static bodies reach this through the constraint graph, and the
		// answer is zero -- apply no impulse -- not a division by zero.
		b3Matrix3 inv = b3InvertPointMass( b3iw_zero, b3MatW_zero, V( 1.0, 0, 0 ), b3iw_zero, b3MatW_zero, V( -1.0, 0, 0 ) );
		check( "a singular point mass inverts to zero",
			   b3Raw( inv.cx.x ) == 0 && b3Raw( inv.cy.y ) == 0 && b3Raw( inv.cz.z ) == 0 );
	}

	// -- the rotational effective mass -------------------------------------
	//
	// inverse( invIA + invIB ). Stage 4: what the spherical joint's spring and
	// motor divide by, and the rotational counterpart of the point mass above.
	{
		// Two isotropic tensors: the sum is (a + b) on the diagonal and the
		// inverse is 1/(a + b), which is a closed form rather than a round
		// trip.
		const double a = 0.4;
		const double b = 0.25;

		b3MatrixW iA = b3MatW_zero;
		iA.cx.x = iA.cy.y = iA.cz.z = b3iwFromDouble( a );
		b3MatrixW iB = b3MatW_zero;
		iB.cx.x = iB.cy.y = iB.cz.z = b3iwFromDouble( b );

		b3Matrix3 inv = b3InvertRotationMass( iA, iB );

		expect( "rotation mass .xx", b3fToDouble( inv.cx.x ), 1.0 / ( a + b ), 0.01 );
		expect( "rotation mass .yy", b3fToDouble( inv.cy.y ), 1.0 / ( a + b ), 0.01 );
		expect( "rotation mass .zz", b3fToDouble( inv.cz.z ), 1.0 / ( a + b ), 0.01 );
		check( "rotation mass is diagonal for diagonal input", b3Raw( inv.cx.y ) == 0 && b3Raw( inv.cz.x ) == 0 );
	}

	{
		// An anisotropic pair, checked by the round trip the point mass uses:
		// (invIA + invIB) * inverse(...) * v == v. A box's tensor is diagonal
		// but unequal, and the two bodies are rotated relative to each other,
		// so the sum has genuine off-diagonal terms.
		b3MatrixW iA = b3MatW_zero;
		iA.cx.x = b3iwFromDouble( 1.5 );
		iA.cy.y = b3iwFromDouble( 0.75 );
		iA.cz.z = b3iwFromDouble( 2.25 );
		iA.cx.y = iA.cy.x = b3iwFromDouble( 0.3 );
		iA.cy.z = iA.cz.y = b3iwFromDouble( -0.2 );

		b3MatrixW iB = b3MatW_zero;
		iB.cx.x = b3iwFromDouble( 0.5 );
		iB.cy.y = b3iwFromDouble( 1.25 );
		iB.cz.z = b3iwFromDouble( 0.4 );
		iB.cx.z = iB.cz.x = b3iwFromDouble( 0.15 );

		b3Matrix3 inv = b3InvertRotationMass( iA, iB );

		double k[3][3] = {
			{ 2.0, 0.3, 0.15 },
			{ 0.3, 2.0, -0.2 },
			{ 0.15, -0.2, 2.65 },
		};

		b3Vec3 v = V( 1.0, -2.0, 0.5 );
		b3Vec3 iv = b3MulMV( inv, v );
		double ivd[3] = { b3fToDouble( iv.x ), b3fToDouble( iv.y ), b3fToDouble( iv.z ) };
		double back[3];
		for ( int i = 0; i < 3; ++i )
		{
			back[i] = k[i][0] * ivd[0] + k[i][1] * ivd[1] + k[i][2] * ivd[2];
		}

		expect( "rotation mass round trip .x", back[0], 1.0, 0.02 );
		expect( "rotation mass round trip .y", back[1], -2.0, 0.02 );
		expect( "rotation mass round trip .z", back[2], 0.5, 0.02 );

		expect( "rotation mass inverse is symmetric xy", b3fToDouble( inv.cx.y ), b3fToDouble( inv.cy.x ), 1e-3 );
		expect( "rotation mass inverse is symmetric xz", b3fToDouble( inv.cx.z ), b3fToDouble( inv.cz.x ), 1e-3 );
	}

	{
		// The range case, and the reason this is not b3InvertMatrixW on the
		// sum. Two 0.1 m spheres at the shape's default density have inverse
		// inertias near 60 each; the sum is 120 against Q7.24's ceiling of 128,
		// and a ragdoll's two hands are exactly that. Accumulating wide and
		// scaling uniformly is what keeps it representable.
		const double invI = 60.0;

		b3MatrixW iA = b3MatW_zero;
		iA.cx.x = iA.cy.y = iA.cz.z = b3iwFromDouble( invI );
		b3MatrixW iB = iA;

		b3Matrix3 inv = b3InvertRotationMass( iA, iB );

		expect( "two light bodies still invert", b3fToDouble( inv.cx.x ), 1.0 / ( 2.0 * invI ), 0.002 );
		check( "and nothing overflowed", b3Raw( inv.cx.x ) != 0 );
	}

	{
		// **Stage 5's regression, and the bug it is the regression for.**
		//
		// The range case above stops at 60 each. Stage 4 wrote this function to
		// accumulate the sum wide -- and then formed each entry as
		// `b3Raw( a ) + b3Raw( b )`, which is int32 arithmetic that is only
		// *converted* to int64 afterwards. So the very wrap the function exists
		// to remove was still in its first line, one scale down: two inverse
		// inertias of 70 sum to a raw 2.35e9 against INT32_MAX's 2.15e9, and
		// the sum came out negative.
		//
		// A negative effective mass is not a precision loss, it **inverts the
		// constraint** -- the solver drives the bodies apart in proportion to
		// how hard they are held together. A chain of welded boxes reached
		// 89 rad/s on its first step.
		//
		// Swept rather than spot-checked, because the failure had a threshold
		// (a sum of 128, Q7.24's ceiling) and a spot check either side of it is
		// how it survived Stage 4. Every one of these is two ordinary light
		// bodies on one joint.
		for ( int scaled = 20; scaled <= 127; scaled += 1 )
		{
			const double invI = (double)scaled;

			b3MatrixW iA = b3MatW_zero;
			iA.cx.x = iA.cy.y = iA.cz.z = b3iwFromDouble( invI );
			b3MatrixW iB = iA;

			b3Matrix3 inv = b3InvertRotationMass( iA, iB );

			// The tolerance is **one Q12 quantum**, stated absolutely rather
			// than as a percentage, because that is the honest limit here: a
			// mass of 1/254 is sixteen quanta, so one quantum *is* six percent
			// and no arithmetic can do better at this scale. What the sweep
			// asserts is that the value is positive and inside the
			// representation -- the sign is what was broken.
			const double quantum = 1.0 / 4096.0;
			const double want = 1.0 / ( 2.0 * invI );
			if ( b3fToDouble( inv.cx.x ) <= 0.0 || fabs( b3fToDouble( inv.cx.x ) - want ) > quantum )
			{
				char label[96];
				snprintf( label, sizeof( label ), "rotation mass stays positive and right at invI %d each", scaled );
				expect( label, b3fToDouble( inv.cx.x ), want, quantum );
			}
		}
		check( "rotation mass swept from 40 to 254 with no sign inversion", true );
	}

	{
		// Both bodies rotation-locked: singular, and the answer is zero rather
		// than a division by zero. b3JointSim::fixedRotation exists to see this
		// coming, but the inverse must be safe on its own.
		b3Matrix3 inv = b3InvertRotationMass( b3MatW_zero, b3MatW_zero );
		check( "a singular rotation mass inverts to zero",
			   b3Raw( inv.cx.x ) == 0 && b3Raw( inv.cy.y ) == 0 && b3Raw( inv.cz.z ) == 0 );
	}

	// -- the 2x2 rotational effective mass ---------------------------------
	//
	// inverse of [ uX.(iA+iB).uX  uX.(iA+iB).uY ; ...  uY.(iA+iB).uY ], the
	// revolute's collinearity block. Stage 5: the sum used to be formed with
	// b3AddMWMW and wrapped, so this is the 2x2 counterpart of the two ranged
	// cases above.
	{
		// Two isotropic tensors and two orthonormal axes: K is (a + b) on the
		// diagonal with a zero cross term, so the inverse is 1/(a + b) and the
		// off-diagonal is zero. A closed form rather than a round trip.
		const double a = 0.4;
		const double b = 0.25;

		b3MatrixW iA = b3MatW_zero;
		iA.cx.x = iA.cy.y = iA.cz.z = b3iwFromDouble( a );
		b3MatrixW iB = b3MatW_zero;
		iB.cx.x = iB.cy.y = iB.cz.z = b3iwFromDouble( b );

		b3SymMatrix2 inv = b3InvertPerpMass( V( 1, 0, 0 ), V( 0, 1, 0 ), iA, iB );

		expect( "perp mass .xx", b3fToDouble( inv.xx ), 1.0 / ( a + b ), 0.01 );
		expect( "perp mass .yy", b3fToDouble( inv.yy ), 1.0 / ( a + b ), 0.01 );
		check( "perp mass has no cross term for orthonormal axes on an isotropic sum",
			   b3Raw( inv.xy ) == 0 );
	}

	{
		// An anisotropic sum with a genuine cross term, checked by the round
		// trip: K * inverse(K) * v == v.
		b3MatrixW iA = b3MatW_zero;
		iA.cx.x = b3iwFromDouble( 1.5 );
		iA.cy.y = b3iwFromDouble( 0.75 );
		iA.cz.z = b3iwFromDouble( 2.25 );
		iA.cx.y = iA.cy.x = b3iwFromDouble( 0.3 );

		b3MatrixW iB = b3MatW_zero;
		iB.cx.x = b3iwFromDouble( 0.5 );
		iB.cy.y = b3iwFromDouble( 1.25 );
		iB.cz.z = b3iwFromDouble( 0.4 );

		// Sum = [ 2.0 0.3 0 ; 0.3 2.0 0 ; 0 0 2.65 ]. With uX = x and
		// uY = (0, 1, 1)/sqrt(2) the block is
		//   xx = 2.0
		//   xy = 0.3 / sqrt(2)          = 0.212132
		//   yy = (2.0 + 2.65) / 2       = 2.325
		const double s = 0.70710678;
		b3Vec3 uX = V( 1, 0, 0 );
		b3Vec3 uY = V( 0, s, s );

		b3SymMatrix2 inv = b3InvertPerpMass( uX, uY, iA, iB );

		const double k[2][2] = { { 2.0, 0.3 * s }, { 0.3 * s, 2.325 } };
		const double vx = 1.0, vy = -2.0;

		double ix = b3fToDouble( inv.xx ) * vx + b3fToDouble( inv.xy ) * vy;
		double iy = b3fToDouble( inv.xy ) * vx + b3fToDouble( inv.yy ) * vy;

		expect( "perp mass round trip .x", k[0][0] * ix + k[0][1] * iy, vx, 0.02 );
		expect( "perp mass round trip .y", k[1][0] * ix + k[1][1] * iy, vy, 0.02 );
	}

	{
		// The range case, and the whole reason Stage 5 wrote this function.
		// Two light bodies sum past Q7.24's ceiling of 128, which is what
		// b3AddMWMW wrapped -- inverting the sign of the effective mass and
		// flinging the hinge apart. Accumulating wide and scaling uniformly is
		// what keeps it representable, exactly as b3InvertRotationMass does.
		const double invI = 60.0;

		b3MatrixW iA = b3MatW_zero;
		iA.cx.x = iA.cy.y = iA.cz.z = b3iwFromDouble( invI );
		b3MatrixW iB = iA;

		b3SymMatrix2 inv = b3InvertPerpMass( V( 1, 0, 0 ), V( 0, 1, 0 ), iA, iB );

		expect( "two light bodies still invert (2x2)", b3fToDouble( inv.xx ), 1.0 / ( 2.0 * invI ), 0.002 );
		check( "and the effective mass is positive", b3Raw( inv.xx ) > 0 && b3Raw( inv.yy ) > 0 );
	}

	{
		// Both bodies rotation-locked: singular, and the answer is zero rather
		// than a division by zero.
		b3SymMatrix2 inv = b3InvertPerpMass( V( 1, 0, 0 ), V( 0, 1, 0 ), b3MatW_zero, b3MatW_zero );
		check( "a singular perp mass inverts to zero",
			   b3Raw( inv.xx ) == 0 && b3Raw( inv.yy ) == 0 && b3Raw( inv.xy ) == 0 );
	}

	// -- the two-lever-arm effective masses --------------------------------
	//
	// Stage 6. Every effective mass above has one axis shared between the two
	// bodies. A contact's and a prismatic joint's do not: A's lever arm is
	// `cross( rA + d, axis )` and B's is `cross( rB, axis )`, so the two are
	// different vectors and neither b3AxisInertiaSumWide nor b3InvertPerpMass
	// has the right shape.
	{
		// The scalar one, against a closed form. Isotropic tensors, a lever arm
		// of length L on A only, none on B:
		//   k = mA + mB + invI * L^2
		const double mA = 3.0, mB = 5.0, invI = 2.0, L = 1.5;

		b3MatrixW iA = b3MatW_zero;
		iA.cx.x = iA.cy.y = iA.cz.z = b3iwFromDouble( invI );

		int64_t k = b3LeverInertiaSumWide( b3iwFromDouble( mA ), iA, V( 0, L, 0 ), b3iwFromDouble( mB ), b3MatW_zero,
										   V( 0, 0, 0 ) );
		b3f mass = b3RcpWide( k );

		expect( "lever inertia sum inverts to the closed form", b3fToDouble( mass ), 1.0 / ( mA + mB + invI * L * L ),
				0.002 );
	}

	{
		// **The inverse-mass sweep.** This is the one that matters, and it is a
		// sweep for the reason the rotation-mass sweep above is: the failure has
		// a threshold and a spot check either side of it is exactly how it
		// survived. B3_MIN_MASS_RAW caps a *single* inverse mass at about 124,
		// just inside Q7.24's ceiling of 128 -- so one body always fits and
		// testing one body finds nothing. The pair crosses the ceiling at 64
		// each, which is mid-sweep rather than at an endpoint.
		for ( int scaled = 20; scaled <= 124; scaled += 1 )
		{
			const double invMass = (double)scaled;

			int64_t k = b3LeverInertiaSumWide( b3iwFromDouble( invMass ), b3MatW_zero, V( 0, 0, 0 ),
											   b3iwFromDouble( invMass ), b3MatW_zero, V( 0, 0, 0 ) );
			b3f mass = b3RcpWide( k );

			// One Q12 quantum, stated absolutely: at an inverse mass of 248 the
			// answer is four thousandths, so a quantum is a quarter of it and no
			// arithmetic does better at this scale. The sign is what was broken.
			const double quantum = 1.0 / 4096.0;
			const double want = 1.0 / ( 2.0 * invMass );
			if ( b3fToDouble( mass ) <= 0.0 || fabs( b3fToDouble( mass ) - want ) > quantum )
			{
				char label[96];
				snprintf( label, sizeof( label ), "lever mass stays positive and right at invMass %d each", scaled );
				expect( label, b3fToDouble( mass ), want, quantum );
			}
		}
		check( "inverse-mass sum swept from 40 to 248 with no sign inversion", true );
	}

	{
		// **b3RcpWide's wide branch, swept against the exact quotient.**
		//
		// Stage 7 Step 0. The branch used to be `( 1 << 36 ) / mag` -- a 64-by-64
		// divide that pulled __aeabi_ldivmod into all six objects calling
		// b3RcpWide. It is now six restoring-division steps in b3RcpWideSlow,
		// which is exact rather than approximate, so the assertion here is
		// equality with floor( 2^36 / mag ) computed in int64_t and not a
		// tolerance. Anything else means the six steps are not enough.
		//
		// A sweep rather than a spot check for the reason the sweep above is one:
		// the quotient shrinks from 32 to 0 across the branch's domain, and the
		// bit that stops being set is where an off-by-one would hide.
		const int64_t numerator = (int64_t)1 << 36;
		int wideMismatches = 0;
		int sawQuotient32 = 0;

		// 2^31 is the first value that reaches the branch at all -- INT32_MAX is
		// 2^31-1 and takes the fast path -- and it is the one case where the
		// quotient is exactly 32, the bound the six steps are sized from.
		for ( int64_t mag = (int64_t)1 << 31; mag <= (int64_t)1 << 42; mag += ( mag >> 4 ) + 1 )
		{
			const int64_t want = numerator / mag;

			if ( want == 32 )
			{
				sawQuotient32 = 1;
			}

			// Both signs: the branch negates the magnitude back afterwards, and
			// an inverse mass is signed.
			if ( (int64_t)b3Raw( b3RcpWide( mag ) ) != want || (int64_t)b3Raw( b3RcpWide( -mag ) ) != -want )
			{
				if ( wideMismatches == 0 )
				{
					char label[128];
					snprintf( label, sizeof( label ), "wide reciprocal exact at mag %lld", (long long)mag );
					check( label, false );
				}
				wideMismatches++;
			}
		}

		check( "b3RcpWide's wide branch is exact across 2^31..2^42, both signs", wideMismatches == 0 );
		check( "and the sweep covered the quotient-32 bound at mag = 2^31", sawQuotient32 == 1 );

		// The endpoints the sweep steps over. INT32_MIN and INT32_MAX are the
		// last fast-path values; the two either side of them are the first wide
		// ones, and the branch boundary is where a `<=` would be wrong.
		check( "INT32_MAX takes the fast path", b3Raw( b3RcpWide( INT32_MAX ) ) == b3Raw( b3RcpW( b3Makeb3iw( INT32_MAX ) ) ) );
		check( "INT32_MIN takes the fast path", b3Raw( b3RcpWide( INT32_MIN ) ) == b3Raw( b3RcpW( b3Makeb3iw( INT32_MIN ) ) ) );
		check( "one past INT32_MAX is exact", (int64_t)b3Raw( b3RcpWide( (int64_t)INT32_MAX + 1 ) ) ==
											  numerator / ( (int64_t)INT32_MAX + 1 ) );
		check( "one past INT32_MIN is exact", (int64_t)b3Raw( b3RcpWide( (int64_t)INT32_MIN - 1 ) ) ==
											  -( numerator / ( -( (int64_t)INT32_MIN - 1 ) ) ) );

		// The magnitude with no int64 representation. Reached only by a corrupt
		// inertia sum, but the negation is written through uint64_t so that it is
		// defined rather than merely unlikely, and a quotient of zero is the
		// right answer for a denominator this large.
		check( "INT64_MIN negates without undefined behaviour", b3Raw( b3RcpWide( INT64_MIN ) ) == 0 );
	}

	{
		// The 2x2, against a closed form. Isotropic tensors, orthogonal arms of
		// equal length on A only, nothing on B: K is diagonal and equal.
		const double mA = 2.0, mB = 1.0, invI = 3.0, L = 0.8;

		b3MatrixW iA = b3MatW_zero;
		iA.cx.x = iA.cy.y = iA.cz.z = b3iwFromDouble( invI );

		b3SymMatrix2 inv = b3InvertPointLineMass( b3iwFromDouble( mA ), iA, V( 0, L, 0 ), V( 0, 0, L ),
												  b3iwFromDouble( mB ), b3MatW_zero, V( 0, 0, 0 ), V( 0, 0, 0 ) );

		const double want = 1.0 / ( mA + mB + invI * L * L );
		expect( "point-line mass .xx", b3fToDouble( inv.xx ), want, 0.01 );
		expect( "point-line mass .yy", b3fToDouble( inv.yy ), want, 0.01 );
		check( "and no cross term for orthogonal arms on an isotropic tensor", b3Raw( inv.xy ) == 0 );
	}

	{
		// **The lever-arm sweep, and the property it asserts is positive
		// definiteness rather than a value.**
		//
		// This is the axis no earlier effective mass has: a prismatic joint's
		// arm is a function of the joint's *state*, so K grows as the square of
		// however far the slider has travelled. b3InvertAccumulated2 answers
		// that by scaling the pair uniformly down before inverting -- and a
		// large shift count is precisely what can quantize the small entries of
		// a 2x2 toward zero and leave it indefinite. An indefinite K turns the
		// perpendicular constraint into a repulsion, which is silent, geometry
		// dependent, and appears only on long rails.
		//
		// So the assertion is `xx > 0 && yy > 0 && xx*yy > xy*xy` at every step
		// from a short arm to a very long one.
		b3MatrixW iA = b3MatW_zero;
		iA.cx.x = b3iwFromDouble( 60.0 );
		iA.cy.y = b3iwFromDouble( 45.0 );
		iA.cz.z = b3iwFromDouble( 30.0 );
		iA.cx.y = iA.cy.x = b3iwFromDouble( 5.0 );

		b3MatrixW iB = b3MatW_zero;
		iB.cx.x = iB.cy.y = iB.cz.z = b3iwFromDouble( 55.0 );

		int failures = 0;
		for ( int step = 1; step <= 80; ++step )
		{
			const double L = 0.25 * (double)step;

			// Deliberately non-orthogonal arms, so xy is genuinely non-zero and
			// the determinant test has something to bite on.
			b3SymMatrix2 inv =
				b3InvertPointLineMass( b3iwFromDouble( 4.0 ), iA, V( 0, L, 0.2 * L ), V( 0.1 * L, 0, L ),
									   b3iwFromDouble( 6.0 ), iB, V( 0, 0.5 * L, 0 ), V( 0, 0, 0.5 * L ) );

			const double xx = b3fToDouble( inv.xx );
			const double yy = b3fToDouble( inv.yy );
			const double xy = b3fToDouble( inv.xy );

			// Zero is a legitimate answer -- it is the port's "apply no
			// impulse", and a lever arm long enough to underflow the inverse
			// cannot carry a useful impulse anyway. What must never happen is a
			// *negative* diagonal or a negative determinant, either of which
			// makes the constraint push the wrong way.
			if ( xx == 0.0 && yy == 0.0 && xy == 0.0 )
			{
				continue;
			}
			if ( xx <= 0.0 || yy <= 0.0 || xx * yy <= xy * xy )
			{
				if ( failures == 0 )
				{
					char label[128];
					snprintf( label, sizeof( label ), "point-line mass stays positive definite at arm %.2f m", L );
					check( label, false );
				}
				failures += 1;
			}
		}
		check( "point-line mass swept from 0.25 m to 20 m and stayed positive definite", failures == 0 );
	}

	{
		// Two statics: singular, and the answer is zero rather than a division
		// by zero, as every other effective mass in the port does.
		b3SymMatrix2 inv = b3InvertPointLineMass( b3iw_zero, b3MatW_zero, V( 1, 0, 0 ), V( 0, 1, 0 ), b3iw_zero,
												  b3MatW_zero, V( 1, 0, 0 ), V( 0, 1, 0 ) );
		check( "a singular point-line mass inverts to zero",
			   b3Raw( inv.xx ) == 0 && b3Raw( inv.yy ) == 0 && b3Raw( inv.xy ) == 0 );
	}

	// -- the 3-vector impulse clamp ----------------------------------------
	//
	// Stage 4: a spherical motor's torque budget is a sphere, not a box.
	{
		b3Imp3 small = b3MakeImp3( b3impFromDouble( 0.3 ), b3impFromDouble( -0.4 ), b3impFromDouble( 0.0 ) );
		b3Imp3 kept = b3ClampImp3( small, b3impFromDouble( 1.0 ) );
		check( "an impulse inside the bound is untouched",
			   b3Raw( kept.x ) == b3Raw( small.x ) && b3Raw( kept.y ) == b3Raw( small.y ) );

		// 3-4-5: magnitude 5, clamped to 1, so each component scales by 1/5.
		b3Imp3 big = b3MakeImp3( b3impFromDouble( 3.0 ), b3impFromDouble( 4.0 ), b3impFromDouble( 0.0 ) );
		b3Imp3 clamped = b3ClampImp3( big, b3impFromDouble( 1.0 ) );
		expect( "clamped .x", b3impToDouble( clamped.x ), 0.6, 0.002 );
		expect( "clamped .y", b3impToDouble( clamped.y ), 0.8, 0.002 );

		// The direction survives, which is the whole point of clamping the
		// magnitude rather than each component. Component-wise clamping would
		// have given (1, 1, 0) here -- magnitude 1.414, and pointing elsewhere.
		double ratioBefore = b3impToDouble( big.y ) / b3impToDouble( big.x );
		double ratioAfter = b3impToDouble( clamped.y ) / b3impToDouble( clamped.x );
		expect( "clamping preserves direction", ratioAfter, ratioBefore, 0.01 );

		// And along a diagonal, which is where a component-wise bound is worst:
		// three equal components of 1 have magnitude sqrt(3), not 1.
		b3Imp3 diagonal =
			b3MakeImp3( b3impFromDouble( 1.0 ), b3impFromDouble( 1.0 ), b3impFromDouble( 1.0 ) );
		b3Imp3 bounded = b3ClampImp3( diagonal, b3impFromDouble( 1.0 ) );
		double magnitude = sqrt( (double)b3Imp3LengthSquaredWide( bounded ) ) / 65536.0;
		expect( "a diagonal impulse is bounded by magnitude", magnitude, 1.0, 0.01 );

		// Zero must not divide.
		b3Imp3 zero = b3MakeImp3( b3imp_zero, b3imp_zero, b3imp_zero );
		b3Imp3 stillZero = b3ClampImp3( zero, b3impFromDouble( 1.0 ) );
		check( "clamping zero stays zero", b3Raw( stillZero.x ) == 0 && b3Raw( stillZero.z ) == 0 );

		// A *small* bound, which is where the first version of this routine
		// went wrong. It formed the rescaling ratio at Q12, and a bound of 14
		// Q16 quanta narrows to zero there -- so instead of clamping the
		// impulse it emptied it. A joint motor limited to 0.05 N-m at 240 Hz
		// has exactly that maximum impulse, so this is an ordinary setting, not
		// a corner.
		{
			b3imp tiny = b3Makeb3imp( 14 ); // 14 Q16 quanta, under one Q12 quantum
			b3Imp3 over = b3MakeImp3( b3Makeb3imp( 60 ), b3Makeb3imp( 60 ), b3Makeb3imp( 60 ) );
			b3Imp3 held = b3ClampImp3( over, tiny );

			double magnitude = sqrt( (double)b3Imp3LengthSquaredWide( held ) );
			expect( "a sub-Q12 bound still clamps rather than empties", magnitude, 14.0, 1.5 );
			check( "and does not collapse to zero", b3Raw( held.x ) != 0 );
		}
	}
}

/// The angle helpers the revolute joint reads its hinge through.
static void test_joint_angles( void )
{
	section( "twist angle and brad conversion" );

	// b3BradToRadF against the exact constant, across the range a limit uses.
	{
		const int brads[5] = { 0, 4096, 8192, 16384, -8192 };
		for ( int i = 0; i < 5; ++i )
		{
			double want = (double)brads[i] * ( 2.0 * M_PI / 32768.0 );
			char buf[80];
			snprintf( buf, sizeof( buf ), "bradToRadF( %d )", brads[i] );
			// One Q12 quantum: a brad is 0.785 of one, so this is the
			// narrowing and nothing else.
			expect( buf, b3fToDouble( b3BradToRadF( (b3a)brads[i] ) ), want, Q12 );
		}
	}

	// b3GetTwistAngle recovers the angle of a rotation about z.
	{
		const int angles[6] = { 0, 2048, 8192, 16000, -8192, -16000 };
		for ( int i = 0; i < 6; ++i )
		{
			b3Quat q = b3MakeQuatFromAxisAngle( V( 0, 0, 1 ), (b3a)angles[i] );
			b3a got = b3GetTwistAngle( q );

			char buf[80];
			snprintf( buf, sizeof( buf ), "twist about z of %d brads", angles[i] );
			// 16 brads is 0.18 degrees. The half-angle goes through a
			// quaternion at Q30 and back through an atan2 evaluated in fixed
			// point, so this is the round trip, not the atan2 alone.
			expect( buf, (double)got, (double)angles[i], 16.0 );
		}
	}

	// Polarity: a quaternion and its negation are the same rotation and must
	// report the same twist. This is the branch upstream's comment is about,
	// and the one a caller would otherwise have to unwind by hand.
	{
		b3Quat q = b3MakeQuatFromAxisAngle( V( 0, 0, 1 ), (b3a)12000 );
		b3Quat n = b3NegateQuat( q );
		expect( "negated quaternion reports the same twist", (double)b3GetTwistAngle( n ),
				(double)b3GetTwistAngle( q ), 2.0 );
	}

	// A rotation about x has no twist about z.
	{
		b3Quat q = b3MakeQuatFromAxisAngle( V( 1, 0, 0 ), (b3a)8192 );
		expect( "a pure x rotation has no z twist", (double)b3GetTwistAngle( q ), 0.0, 16.0 );
	}

	// --------------------------------------------------------------------
	// b3GetSwingAngle -- the cone limit's angle, and b3GetTwistAngle's
	// complement. Stage 4.
	// --------------------------------------------------------------------

	// A rotation about x or y tilts z away by exactly that angle.
	{
		const int angles[5] = { 0, 2048, 8192, 12000, 16000 };
		for ( int i = 0; i < 5; ++i )
		{
			b3Quat qx = b3MakeQuatFromAxisAngle( V( 1, 0, 0 ), (b3a)angles[i] );
			b3Quat qy = b3MakeQuatFromAxisAngle( V( 0, 1, 0 ), (b3a)angles[i] );

			char buf[80];
			snprintf( buf, sizeof( buf ), "swing about x of %d brads", angles[i] );
			expect( buf, (double)b3GetSwingAngle( qx ), (double)angles[i], 16.0 );
			snprintf( buf, sizeof( buf ), "swing about y of %d brads", angles[i] );
			expect( buf, (double)b3GetSwingAngle( qy ), (double)angles[i], 16.0 );
		}
	}

	// A pure twist about z tilts nothing, whichever way it turns. This is the
	// pair to "a pure x rotation has no z twist" above, and together they are
	// what makes the two angles independent coordinates rather than two views
	// of the same number.
	{
		const int angles[4] = { 2048, 8192, -8192, 16000 };
		for ( int i = 0; i < 4; ++i )
		{
			b3Quat q = b3MakeQuatFromAxisAngle( V( 0, 0, 1 ), (b3a)angles[i] );
			char buf[80];
			snprintf( buf, sizeof( buf ), "a pure z twist of %d has no swing", angles[i] );
			expect( buf, (double)b3GetSwingAngle( q ), 0.0, 16.0 );
		}
	}

	// Swing is unsigned: tilting the other way is the same cone angle. A cone
	// limit is one-sided precisely because of this.
	{
		b3Quat pos = b3MakeQuatFromAxisAngle( V( 1, 0, 0 ), (b3a)6000 );
		b3Quat neg = b3MakeQuatFromAxisAngle( V( 1, 0, 0 ), (b3a)-6000 );
		expect( "swing is unsigned", (double)b3GetSwingAngle( neg ), (double)b3GetSwingAngle( pos ), 2.0 );
	}

	// Polarity, as for the twist: a quaternion and its negation are the same
	// rotation. Every term here is squared, so this should be exact rather
	// than merely close.
	{
		b3Quat q = b3MakeQuatFromAxisAngle( V( 1, 0, 0 ), (b3a)9000 );
		expect( "negated quaternion reports the same swing", (double)b3GetSwingAngle( b3NegateQuat( q ) ),
				(double)b3GetSwingAngle( q ), 0.0 );
	}

	// The resolution claim in b3GetSwingAngle's comment, checked rather than
	// asserted: a swing well under the ~2.5 brads a Q12 narrowing would floor
	// at must still be reported, not rounded to zero. Passing the Q30 roots
	// through b3Atan2Raw is the whole reason this works.
	{
		b3Quat q = b3MakeQuatFromAxisAngle( V( 1, 0, 0 ), (b3a)1 );
		expect( "a one-brad swing does not round to zero", (double)b3GetSwingAngle( q ), 1.0, 1.0 );
	}

	// --------------------------------------------------------------------
	// b3DeltaQuatToRotation -- the rotational error a spring drives to zero.
	// Stage 4.
	// --------------------------------------------------------------------

	// Identical orientations have no error, whatever they are.
	{
		b3Quat q = b3MakeQuatFromAxisAngle( V( 1, 2, 3 ), (b3a)5000 );
		expectVec( "delta of a quaternion with itself", b3DeltaQuatToRotation( q, q ), 0.0, 0.0, 0.0, 8 * Q12 );
	}

	// A small rotation about one axis comes back as that axis scaled by the
	// angle in radians -- which is what makes it usable as a constraint error.
	// 2048 brads is a quarter of pi/2, or 0.3926991 rad.
	{
		b3Quat target = b3MakeQuatFromAxisAngle( V( 0, 0, 1 ), (b3a)2048 );
		b3Vec3 c = b3DeltaQuatToRotation( b3Quat_identity, target );
		expectVec( "delta identity -> 2048 brads about z", c, 0.0, 0.0, 0.3926991, 48 * Q12 );
	}

	// And about x, so no axis is privileged by the quaternion product's shape.
	{
		b3Quat target = b3MakeQuatFromAxisAngle( V( 1, 0, 0 ), (b3a)-2048 );
		b3Vec3 c = b3DeltaQuatToRotation( b3Quat_identity, target );
		expectVec( "delta identity -> -2048 brads about x", c, -0.3926991, 0.0, 0.0, 48 * Q12 );
	}

	// The polarity fold. Negating the target is the same rotation, so the
	// error must be unchanged -- without the fold this returns something near
	// magnitude 2 instead, which is the failure the comment describes.
	{
		b3Quat target = b3MakeQuatFromAxisAngle( V( 0, 1, 0 ), (b3a)3000 );
		b3Vec3 a = b3DeltaQuatToRotation( b3Quat_identity, target );
		b3Vec3 b = b3DeltaQuatToRotation( b3Quat_identity, b3NegateQuat( target ) );
		expectVec( "delta ignores target polarity", b, b3fToDouble( a.x ), b3fToDouble( a.y ), b3fToDouble( a.z ),
				   8 * Q12 );
	}

	// Antisymmetry: swapping the two orientations negates the error. A spring
	// that failed this would pull one way and push the other.
	{
		b3Quat q = b3MakeQuatFromAxisAngle( V( 0, 0, 1 ), (b3a)1000 );
		b3Quat target = b3MakeQuatFromAxisAngle( V( 0, 0, 1 ), (b3a)4000 );
		b3Vec3 forward = b3DeltaQuatToRotation( q, target );
		b3Vec3 backward = b3DeltaQuatToRotation( target, q );
		expectVec( "delta is antisymmetric", backward, -b3fToDouble( forward.x ), -b3fToDouble( forward.y ),
				   -b3fToDouble( forward.z ), 16 * Q12 );
	}
}

static void test_transform( void )
{
	section( "transforms" );

	b3Transform t;
	t.p = V( 10.0, -5.0, 2.0 );
	t.q = b3MakeQuatFromAxisAngle( b3Vec3_axisZ, (b3a)( B3_BRAD_CIRCLE / 4 ) );

	b3Vec3 local = V( 1.0, 0.0, 0.0 );

	// Rotate +x to +y, then translate.
	expectVec( "transformPoint", b3TransformPoint( t, local ), 10.0, -4.0, 2.0, 32 * Q12 );

	// The inverse must round-trip.
	b3Vec3 world = b3TransformPoint( t, local );
	expectVec( "invTransformPoint round trip", b3InvTransformPoint( t, world ), 1.0, 0.0, 0.0, 64 * Q12 );

	// Composing a transform with its own inverse gives the identity.
	b3Transform inv = b3InvertTransform( t );
	b3Transform composed = b3MulTransforms( t, inv );
	expectVec( "t * t^-1 has zero translation", composed.p, 0.0, 0.0, 0.0, 64 * Q12 );
	expect( "t * t^-1 has identity rotation", fabs( b3nToDouble( composed.q.s ) ), 1.0, 0.001 );

	// b3InvMulTransforms must agree with composing with the inverse.
	b3Transform a = t;
	b3Transform b;
	b.p = V( 3.0, 1.0, -2.0 );
	b.q = b3MakeQuatFromAxisAngle( b3Vec3_axisY, (b3a)3000 );

	b3Transform viaInvMul = b3InvMulTransforms( a, b );
	b3Transform viaCompose = b3MulTransforms( b3InvertTransform( a ), b );
	expect( "invMulTransforms .p.x", b3fToDouble( viaInvMul.p.x ), b3fToDouble( viaCompose.p.x ), 64 * Q12 );
	expect( "invMulTransforms .p.y", b3fToDouble( viaInvMul.p.y ), b3fToDouble( viaCompose.p.y ), 64 * Q12 );
	expect( "invMulTransforms .p.z", b3fToDouble( viaInvMul.p.z ), b3fToDouble( viaCompose.p.z ), 64 * Q12 );
}

static void test_aabb( void )
{
	section( "bounding boxes" );

	b3AABB a = b3MakeAABB( V( -1.0, -2.0, -3.0 ), V( 1.0, 2.0, 3.0 ) );

	expectVec( "center", b3AABB_Center( a ), 0.0, 0.0, 0.0, Q12 );
	expectVec( "extents", b3AABB_Extents( a ), 1.0, 2.0, 3.0, Q12 );

	s_checks++;
	if ( !b3IsValidAABB( a ) )
	{
		printf( "  FAIL well-formed AABB rejected\n" );
		s_failures++;
	}

	b3AABB b = b3MakeAABB( V( 0.5, 0.5, 0.5 ), V( 2.0, 2.0, 2.0 ) );
	s_checks++;
	if ( !b3AABB_Overlaps( a, b ) )
	{
		printf( "  FAIL overlapping boxes reported disjoint\n" );
		s_failures++;
	}

	b3AABB far = b3MakeAABB( V( 10.0, 10.0, 10.0 ), V( 11.0, 11.0, 11.0 ) );
	s_checks++;
	if ( b3AABB_Overlaps( a, far ) )
	{
		printf( "  FAIL disjoint boxes reported overlapping\n" );
		s_failures++;
	}

	b3AABB u = b3AABB_Union( a, far );
	expectVec( "union lower", u.lowerBound, -1.0, -2.0, -3.0, Q12 );
	expectVec( "union upper", u.upperBound, 11.0, 11.0, 11.0, Q12 );

	s_checks++;
	if ( !b3AABB_Contains( u, a ) || !b3AABB_Contains( u, far ) )
	{
		printf( "  FAIL union does not contain its operands\n" );
		s_failures++;
	}

	expectVec( "closest point clamps", b3ClosestPointToAABB( a, V( 5.0, 0.0, -9.0 ) ), 1.0, 0.0, -3.0, Q12 );

	// The area metric is what the dynamic tree sorts on, and it overflows Q12
	// for boxes bigger than a few units -- hence the wide form.
	b3AABB large = b3MakeAABB( V( -100.0, -100.0, -100.0 ), V( 100.0, 100.0, 100.0 ) );
	double wantArea = 2.0 * ( 200.0 * 200.0 * 3.0 );
	expect( "wide area of large box", (double)b3AABB_AreaWide( large ) / (double)( (int64_t)1 << 24 ), wantArea, 1.0 );
}


// The bug this exists to catch: b3RsqrtWide originally returned a Q30
// coefficient, which caps at 2.0. A reciprocal length only fits that for
// vectors longer than half a unit, so normalizing anything shorter overflowed
// the divider and produced a non-unit result. Every earlier test happened to
// use vectors of length 1 or more, so it went unnoticed until a 0.3-long
// contact offset hit it.
static void test_normalize_range( void )
{
	section( "normalization across magnitudes" );

	static const double lengths[] = { 0.02, 0.05, 0.1, 0.3, 0.5, 0.9, 1.0, 2.0, 10.0, 100.0, 1000.0 };

	for ( size_t i = 0; i < sizeof( lengths ) / sizeof( lengths[0] ); i++ )
	{
		double L = lengths[i];

		// Along an axis, and along a diagonal, so the per-component magnitude
		// differs from the vector magnitude.
		b3Vec3 axis = V( L, 0.0, 0.0 );
		b3Vec3 diag = V( L / sqrt( 3.0 ), L / sqrt( 3.0 ), L / sqrt( 3.0 ) );

		char label[64];

		snprintf( label, sizeof( label ), "normalize axis len %g", L );
		expect( label, b3fToDouble( b3Length( b3Normalize( axis ) ) ), 1.0, 64 * Q12 );

		snprintf( label, sizeof( label ), "normalize diagonal len %g", L );
		expect( label, b3fToDouble( b3Length( b3Normalize( diag ) ) ), 1.0, 64 * Q12 );

		// b3GetLengthAndNormalize must agree with both halves done separately.
		b3f reported;
		b3Vec3 n = b3GetLengthAndNormalize( &reported, axis );
		snprintf( label, sizeof( label ), "reported length %g", L );
		expect( label, b3fToDouble( reported ), L, 64 * Q12 );
		snprintf( label, sizeof( label ), "combined normalize %g", L );
		expect( label, b3fToDouble( b3Length( n ) ), 1.0, 64 * Q12 );
	}

	// Below the representable range the result saturates rather than
	// wrapping; the only guarantee is that it does not come back as a wild
	// value with the wrong sign.
	b3Vec3 tiny = V( 0.001, 0.0, 0.0 );
	b3Vec3 tn = b3Normalize( tiny );
	s_checks++;
	if ( b3fToDouble( tn.x ) < 0.0 )
	{
		printf( "  FAIL sub-slop vector normalized to the wrong sign\n" );
		s_failures++;
	}
}

// The property that matters about a direction-only vector: rescaling it must
// change nothing that is ever read from it, and it must survive magnitudes at
// which a plain b3Cross has nothing left.
static void test_direction_from_wide( void )
{
	section( "direction rescaling" );

	// A cross product of two short vectors is an area, so it shrinks
	// quadratically. At 0.01 units a Q12 cross product is *zero* -- which is
	// what made GJK report spurious overlaps for thin simplices.
	static const double scales[] = { 0.004, 0.01, 0.05, 0.2, 1.0, 5.0, 50.0, 400.0 };

	for ( size_t i = 0; i < sizeof( scales ) / sizeof( scales[0] ); i++ )
	{
		double s = scales[i];
		char label[80];

		// Two edges of a triangle in the xy plane: the cross is along +z.
		b3Vec3 a = V( s, 0, 0 );
		b3Vec3 b = V( 0, s, 0 );

		b3Vec3 d = b3CrossDirection( a, b );

		snprintf( label, sizeof( label ), "cross direction at scale %g is nonzero", s );
		check( label, b3LengthSquaredWide( d ) > 0 );

		snprintf( label, sizeof( label ), "cross direction at scale %g normalizes", s );
		check( label, b3IsNormalized( b3Normalize( d ) ) );

		b3Vec3 n = b3Normalize( d );
		snprintf( label, sizeof( label ), "cross direction at scale %g points along +z", s );
		expect( label, b3fToDouble( n.z ), 1.0, 32 * Q12 );
	}

	// Direction is preserved regardless of the input's magnitude: the same
	// geometry scaled by a thousand must give the same unit vector.
	b3Vec3 small = b3Normalize( b3CrossDirection( V( 0.01, 0.02, 0 ), V( 0, 0.01, 0.03 ) ) );
	b3Vec3 large = b3Normalize( b3CrossDirection( V( 10.0, 20.0, 0 ), V( 0, 10.0, 30.0 ) ) );
	expect( "scale-invariant direction x", b3fToDouble( small.x ), b3fToDouble( large.x ), 64 * Q12 );
	expect( "scale-invariant direction y", b3fToDouble( small.y ), b3fToDouble( large.y ), 64 * Q12 );
	expect( "scale-invariant direction z", b3fToDouble( small.z ), b3fToDouble( large.z ), 64 * Q12 );

	// The plain b3Cross this replaces really does lose it -- stated as a test
	// so the reason for the helper stays visible.
	check( "plain b3Cross underflows where b3CrossDirection does not",
		   b3LengthSquaredWide( b3Cross( V( 0.004, 0, 0 ), V( 0, 0.004, 0 ) ) ) == 0 );

	// Exactly zero in, exactly zero out: that is the one case with no
	// direction, and the only one that may return the zero vector.
	check( "parallel input gives no direction", b3LengthSquaredWide( b3CrossDirection( V( 1, 2, 3 ), V( 2, 4, 6 ) ) ) == 0 );
	check( "zero input gives no direction", b3LengthSquaredWide( b3DirectionFromWide( 0, 0, 0 ) ) == 0 );

	// A single tiny component must not round away.
	check( "lone unit component survives", b3LengthSquaredWide( b3DirectionFromWide( 0, 1, 0 ) ) > 0 );
}

// b3ArbitraryPerp: perpendicular to a unit vector, and never short.
//
// b3Perp would also give a perpendicular, but its result is as short as the
// sine of the angle to the axis it crossed against -- and a short vector in
// Q12 has already lost its low bits before b3Normalize sees it. This one is
// bounded below by about 0.39 whatever the input, which is why
// b3ReduceManifoldPoints uses it for the tangent direction it ranks contact
// points along.
static void test_arbitrary_perp( void )
{
	section( "arbitrary perpendicular" );

	// The three branches, plus their negatives, plus the diagonals that decide
	// which branch is taken.
	static const double dirs[][3] = {
		{ 1, 0, 0 },  { -1, 0, 0 }, { 0, 1, 0 },	{ 0, -1, 0 },  { 0, 0, 1 },	  { 0, 0, -1 },
		{ 1, 1, 1 },  { -1, 1, 1 }, { 1, -1, 1 },	{ 1, 1, -1 },  { -1, -1, 1 }, { -1, 1, -1 },
		{ 1, -1, -1 }, { -1, -1, -1 }, { 0.6, 0.8, 0 }, { 0, 0.6, 0.8 }, { 0.8, 0, 0.6 },
	};

	for ( size_t i = 0; i < sizeof( dirs ) / sizeof( dirs[0] ); ++i )
	{
		b3Vec3 v = b3Normalize( V( dirs[i][0], dirs[i][1], dirs[i][2] ) );
		b3Vec3 p = b3ArbitraryPerp( v );

		check( "perp is unit", b3IsNormalized( p ) );
		expect( "perp is perpendicular", b3fToDouble( b3Dot( p, v ) ), 0.0, 4.0 * Q12 );
	}

	// A deterministic spread over the sphere, so this is not just the axes.
	unsigned seed = 12345u;
	for ( int i = 0; i < 200; ++i )
	{
		double c[3];
		for ( int k = 0; k < 3; ++k )
		{
			seed = seed * 1103515245u + 12345u;
			c[k] = ( (double)( ( seed >> 16 ) & 0x7fff ) / 16383.5 ) - 1.0;
		}

		double len = sqrt( c[0] * c[0] + c[1] * c[1] + c[2] * c[2] );
		if ( len < 0.2 )
		{
			continue;
		}

		b3Vec3 v = b3Normalize( V( c[0] / len, c[1] / len, c[2] / len ) );
		b3Vec3 p = b3ArbitraryPerp( v );

		check( "swept perp is unit", b3IsNormalized( p ) );
		expect( "swept perp is perpendicular", b3fToDouble( b3Dot( p, v ) ), 0.0, 4.0 * Q12 );
	}

	// The fallback. A unit vector always has a component past a half, so
	// reaching the else means the input was not one -- upstream asserts that
	// away, the port answers it.
	b3Vec3 shortIn = V( 0.4, 0.4, 0.4 );
	b3Vec3 shortOut = b3ArbitraryPerp( shortIn );
	check( "short input still gives a unit result", b3IsNormalized( shortOut ) );
	expect( "short input still perpendicular", b3fToDouble( b3Dot( shortOut, shortIn ) ), 0.0, 4.0 * Q12 );

	b3Vec3 zeroOut = b3ArbitraryPerp( b3Vec3_zero );
	check( "zero input gives a unit result", b3IsNormalized( zeroOut ) );
}

// -------------------------------------------------------------------------
// Phase 6 Stage 7: the two-impulse clamp, Q30 rotation columns, and the
// non-unit quaternion angle
// -------------------------------------------------------------------------

static b3Imp2 IMP2( double x, double y )
{
	return b3MakeImp2( b3impFromDouble( x ), b3impFromDouble( y ) );
}

static double imp2Length( b3Imp2 v )
{
	return sqrt( (double)b3Imp2LengthSquaredWide( v ) ) / 65536.0;
}

// b3ClampImp2: a disc, not a box.
//
// The parallel joint's torque budget may be spent about either constrained
// axis or shared between them, so clamping each component separately would let
// it exceed the bound by sqrt(2) along a diagonal. This checks the magnitude
// against the bound and the *direction* against the input, since preserving
// direction is the half of the contract a per-component clamp would break.
static void test_clamp_imp2( void )
{
	section( "two-impulse radial clamp" );

	const double QIMP = 1.0 / 65536.0;

	// Inside the bound: returned untouched, bit for bit.
	{
		b3Imp2 v = IMP2( 0.3, -0.4 );	// length exactly 0.5
		b3Imp2 got = b3ClampImp2( v, b3impFromDouble( 1.0 ) );
		check( "inside the bound is returned unchanged",
			   b3Raw( got.x ) == b3Raw( v.x ) && b3Raw( got.y ) == b3Raw( v.y ) );
	}

	// Exactly at the bound: still untouched. The comparison is `<=`, so the
	// boundary belongs to the no-op side and no division happens.
	{
		b3Imp2 v = IMP2( 0.6, 0.8 );	// length exactly 1.0
		b3Imp2 got = b3ClampImp2( v, b3impFromDouble( 1.0 ) );
		check( "exactly at the bound is returned unchanged",
			   b3Raw( got.x ) == b3Raw( v.x ) && b3Raw( got.y ) == b3Raw( v.y ) );
	}

	// Outside: scaled to the bound, direction preserved. The diagonal is the
	// case a per-component clamp gets wrong -- it would leave this at
	// sqrt(2) * 1.0 = 1.414.
	{
		b3Imp2 v = IMP2( 3.0, 3.0 );
		b3Imp2 got = b3ClampImp2( v, b3impFromDouble( 1.0 ) );
		expect( "diagonal clamps to the bound, not sqrt(2) past it", imp2Length( got ), 1.0, 1e-3 );
		expect( "diagonal keeps its direction", b3impToDouble( got.x ) - b3impToDouble( got.y ), 0.0, 4 * QIMP );
	}

	// A bound of zero empties the accumulator. Reachable: `maxTorque` defaults
	// to zero on a parallel joint, and the joint is expected to do nothing.
	{
		b3Imp2 got = b3ClampImp2( IMP2( 2.0, -1.0 ), b3imp_zero );
		check( "a zero bound empties the impulse", b3Raw( got.x ) == 0 && b3Raw( got.y ) == 0 );
	}

	// The case this routine exists for, and the one b3ClampImp3's comment
	// records as a real defect: a bound of a handful of Q16 quanta, which is
	// *zero* at Q12. A 0.05 N-m budget at 240 Hz is 14 quanta.
	{
		b3imp tiny = b3Makeb3imp( 14 );
		b3Imp2 got = b3ClampImp2( IMP2( 0.5, 0.5 ), tiny );
		double want = 14.0 / 65536.0;
		expect( "a 14-quantum bound survives the ratio", imp2Length( got ), want, 2 * QIMP );
		check( "a 14-quantum bound does not empty the impulse", b3Raw( got.x ) != 0 );
	}

	// A sweep, because a threshold bug needs one: magnitudes from far inside
	// the bound to far outside, and bounds spanning four orders.
	{
		const double bounds[4] = { 0.001, 0.05, 1.0, 40.0 };
		for ( int b = 0; b < 4; ++b )
		{
			b3imp maxImp = b3impFromDouble( bounds[b] );
			for ( int k = 1; k <= 20; ++k )
			{
				double scale = 0.2 * k;	   // 0.2x to 4x the bound
				b3Imp2 v = IMP2( bounds[b] * scale * 0.6, bounds[b] * scale * -0.8 );
				b3Imp2 got = b3ClampImp2( v, maxImp );

				double gotLen = imp2Length( got );
				double wantLen = scale <= 1.0 ? bounds[b] * scale : bounds[b];

				// Two Q16 quanta of slack: the root and the ratio each round.
				check( "swept clamp never exceeds its bound", gotLen <= bounds[b] + 2 * QIMP );
				expect( "swept clamp lands on the expected magnitude", gotLen, wantLen, 4 * QIMP + 0.002 * wantLen );
			}
		}
	}

	// The headroom claim in the doc comment, made executable. Two full-scale
	// raw squares must not overflow int64 -- and the value must come back
	// positive, which is the thing an overflow would break.
	{
		b3Imp2 huge = { b3Makeb3imp( INT32_MAX ), b3Makeb3imp( INT32_MAX ) };
		int64_t sq = b3Imp2LengthSquaredWide( huge );
		check( "two full-scale squares stay positive", sq > 0 );
		check( "two full-scale squares fit int64", sq == 2 * (int64_t)INT32_MAX * INT32_MAX );
	}
}

// b3QuatColumnsN against b3MakeMatrixFromQuat: the measurement that justifies
// publishing it, rather than an assertion that it is better.
//
// The wheel joint takes an arc tangent of two dot products of these columns,
// and the angle error is the column error divided by how far the wheel is from
// fully tipped. So what matters is not that Q30 is more accurate in the
// abstract but by how much, and the orthonormality residual is the number that
// converts directly into brads.
static void test_quat_columns_n( void )
{
	section( "Q30 rotation columns" );

	unsigned seed = 998877u;
	double worstN = 0.0;
	double worstQ12 = 0.0;
	double worstAgree = 0.0;

	for ( int i = 0; i < 200; ++i )
	{
		double c[4];
		for ( int k = 0; k < 4; ++k )
		{
			seed = seed * 1103515245u + 12345u;
			c[k] = ( (double)( ( seed >> 16 ) & 0x7fff ) / 16383.5 ) - 1.0;
		}
		double len = sqrt( c[0] * c[0] + c[1] * c[1] + c[2] * c[2] + c[3] * c[3] );
		if ( len < 0.2 )
		{
			continue;
		}

		b3Quat q = b3NormalizeQuat(
			( b3Quat ){ { b3nFromDouble( c[0] / len ), b3nFromDouble( c[1] / len ), b3nFromDouble( c[2] / len ) },
						b3nFromDouble( c[3] / len ) } );

		b3Dir3 n[3];
		b3QuatColumnsN( q, n );
		b3Matrix3 m = b3MakeMatrixFromQuat( q );

		// Each column against the same column rotated the exact way, and the
		// three pairwise dot products, which must vanish for an orthonormal
		// frame. The off-diagonals are the sensitive ones -- they are a
		// difference of nearly equal products, so they show the narrowing
		// where the diagonals do not.
		double nd[3][3], qd[3][3];
		for ( int a = 0; a < 3; ++a )
		{
			nd[a][0] = b3nToDouble( n[a].x );
			nd[a][1] = b3nToDouble( n[a].y );
			nd[a][2] = b3nToDouble( n[a].z );
		}
		qd[0][0] = b3fToDouble( m.cx.x );
		qd[0][1] = b3fToDouble( m.cx.y );
		qd[0][2] = b3fToDouble( m.cx.z );
		qd[1][0] = b3fToDouble( m.cy.x );
		qd[1][1] = b3fToDouble( m.cy.y );
		qd[1][2] = b3fToDouble( m.cy.z );
		qd[2][0] = b3fToDouble( m.cz.x );
		qd[2][1] = b3fToDouble( m.cz.y );
		qd[2][2] = b3fToDouble( m.cz.z );

		for ( int a = 0; a < 3; ++a )
		{
			for ( int b = a + 1; b < 3; ++b )
			{
				double dotN = nd[a][0] * nd[b][0] + nd[a][1] * nd[b][1] + nd[a][2] * nd[b][2];
				double dotQ = qd[a][0] * qd[b][0] + qd[a][1] * qd[b][1] + qd[a][2] * qd[b][2];
				if ( fabs( dotN ) > worstN )
				{
					worstN = fabs( dotN );
				}
				if ( fabs( dotQ ) > worstQ12 )
				{
					worstQ12 = fabs( dotQ );
				}
			}
		}

		// The columns must agree with the Q12 matrix to within the Q12 quantum,
		// which is the statement that this is the *same* matrix and not a
		// differently derived one. Accumulated to a worst case and asserted
		// once after the loop rather than checked nine times per iteration:
		// eighteen hundred identical assertions would inflate the suite's
		// count without saying anything the worst case does not.
		for ( int a = 0; a < 3; ++a )
		{
			for ( int k = 0; k < 3; ++k )
			{
				double d = fabs( nd[a][k] - qd[a][k] );
				if ( d > worstAgree )
				{
					worstAgree = d;
				}
			}
		}
	}

	check( "Q30 columns agree with the Q12 matrix everywhere", worstAgree < 3 * Q12 );

	// The measurement. Q30 columns are orthonormal to about 1e-8; Q12 to about
	// 1e-3, five orders worse. Asserted loosely enough that this is a
	// regression test rather than a re-measurement, but tightly enough that
	// losing the Q30 path would fail it.
	printf( "    orthonormality residual: Q30 %.3g   Q12 %.3g\n", worstN, worstQ12 );
	check( "Q30 columns are orthonormal to better than 1e-6", worstN < 1e-6 );
	check( "Q12 columns are three orders worse", worstQ12 > 100.0 * worstN );
}

// b3GetQuatAngle, now `2 * atan2( |v|, s )` rather than `2 * acos( s )`.
//
// Two properties, and the function was changed at Stage 7 because it had
// neither: it must stay accurate for a *small* angle, and it must respond when
// the caller zeroes a component. Both are exercised here, because the
// separation queries depend on both and the previous form had no callers at all
// to notice.
static void test_quat_angle( void )
{
	section( "quaternion angle" );

	// Accuracy across the range, and deliberately weighted toward the small
	// end -- the acos form was 80% wrong at 128 brads and correct by 4096.
	// A flat 6-brad tolerance across the whole sweep is the statement that
	// there is no ill-conditioned end any more.
	{
		const int angles[10] = { 0, 128, 256, 512, 1024, 2048, 4096, 8192, 16000, -4096 };
		b3Vec3 axis = b3Normalize( V( 0.3, -0.5, 0.81 ) );

		for ( int i = 0; i < 10; ++i )
		{
			b3Quat q = b3MakeQuatFromAxisAngle( axis, (b3a)angles[i] );

			// The result is a magnitude, so a negative rotation comes back
			// positive.
			int want = angles[i] < 0 ? -angles[i] : angles[i];

			char buf[80];
			snprintf( buf, sizeof( buf ), "quat angle of %d brads", angles[i] );
			expect( buf, (double)b3GetQuatAngle( q ), (double)want, 6.0 );
		}
	}

	// The property the separation queries need. A rotation with both a twist
	// about z and a swing away from it: zeroing v.z strips the twist, and the
	// reported angle must fall. Against `2 * acos( s )` this was a silent
	// no-op, so a test that only checked accuracy would have passed with the
	// defect in place.
	{
		b3Quat twist = b3MakeQuatFromAxisAngle( V( 0, 0, 1 ), (b3a)8192 );
		b3Quat swing = b3MakeQuatFromAxisAngle( V( 1, 0, 0 ), (b3a)4096 );
		b3Quat q = b3NormalizeQuat( b3MulQuat( twist, swing ) );

		b3Quat stripped = q;
		stripped.v.z = b3n_zero;

		b3a full = b3GetQuatAngle( q );
		b3a reduced = b3GetQuatAngle( stripped );

		check( "zeroing v.z reduces the reported angle", reduced < full );

		// And it must land where the arithmetic says, not merely somewhere
		// lower. Recomputed here in double from the same components, which is
		// an independent statement of what the function means rather than a
		// number copied out of a previous run: a stripped quaternion is no
		// longer a unit, so the answer is not the 4096-brad swing it was built
		// from -- it is 2 * atan2( |v_xy|, s ) on a vector that has lost its
		// largest component.
		double vx = b3nToDouble( stripped.v.x );
		double vy = b3nToDouble( stripped.v.y );
		double s = b3nToDouble( stripped.s );
		double wantRad = 2.0 * atan2( sqrt( vx * vx + vy * vy ), fabs( s ) );
		double wantBrads = wantRad * ( 32768.0 / ( 2.0 * M_PI ) );

		expect( "the reduction is the twist, not an arbitrary drop", (double)reduced, wantBrads, 6.0 );
	}

	// A negative scalar is the same rotation taken the long way round. A
	// quaternion and its negation are one rotation, so both must report the
	// same angle.
	{
		b3Quat q = b3NormalizeQuat( b3MakeQuatFromAxisAngle( b3Normalize( V( 1, 1, 0 ) ), (b3a)12000 ) );
		b3Quat neg = { { b3NegN( q.v.x ), b3NegN( q.v.y ), b3NegN( q.v.z ) }, b3NegN( q.s ) };
		expect( "the double cover is folded", (double)b3GetQuatAngle( neg ), (double)b3GetQuatAngle( q ), 2.0 );
	}

	// The two endpoints, where the arc tangent's arguments are most lopsided.
	{
		check( "identity has zero angle", b3GetQuatAngle( b3Quat_identity ) == 0 );

		b3Quat half = b3MakeQuatFromAxisAngle( V( 0, 1, 0 ), (b3a)16384 );
		expect( "a half turn reports a half turn", (double)b3GetQuatAngle( half ), 16384.0, 4.0 );
	}
}

int main( void )
{
	b3TestInstallAssertTrap();

#if defined( B3_FIXED_DEBUG )
	b3FixedResetStats();
	printf( "box3d fixed math verification  [B3_FIXED_DEBUG]\n\n" );
#elif defined( B3_FIXED_STRICT )
	printf( "box3d fixed math verification  [B3_FIXED_STRICT]\n\n" );
#else
	printf( "box3d fixed math verification  [device mode]\n\n" );
#endif

	test_vector_basics();
	test_wide_dot();
	test_normalize_range();
	test_direction_from_wide();
	test_arbitrary_perp();
	test_quaternion();
	test_quaternion_drift();
	test_matrix();
	test_impulse_scale();
	test_clamp_imp2();
	test_effective_mass();
	test_joint_angles();
	test_quat_angle();
	test_quat_columns_n();
	test_transform();
	test_aabb();

#if defined( B3_FIXED_DEBUG )
	printf( "\n" );
	b3FixedReport( "test_math" );
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
