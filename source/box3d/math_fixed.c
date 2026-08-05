// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Erin Catto        (original Box3D)
// Copyright (c) 2026 Warioware64       (Nitro Engine Advanced fixed-point port)
//
// This file is part of Nitro Engine Advanced

/// @file   math_fixed.c
/// @brief  Out-of-line math: closest points between segments and lines.
///
/// The counterpart to math_fixed.h, holding the parts of upstream's
/// math_functions.c that are too large to inline.
///
/// @section wide Why these run wide
///
/// Every quantity here is a dot product of edge vectors -- that is, a squared
/// length. `denom = a*e - b*b` multiplies two of those together, making it a
/// *fourth* power of length, and the fractions are then that quantity divided
/// by another of the same size. In float this is unremarkable. In Q12 a
/// squared length overflows for edges past ~512 units, and the fourth power
/// overflows almost immediately.
///
/// So the dot products are taken with b3DotWide, which accumulates in int64
/// and never narrows, and the fractions come out through b3DivWideToC, which
/// normalizes numerator and denominator together before dividing. Nothing is
/// narrowed to Q12 until the final point positions, which are genuine
/// lengths and belong there.

#include "box3d/math_fixed.h"

#include "core.h"

// b3DotWide gives Q24. Multiplying two of those would give Q48, which
// overflows int64 for large inputs, so the squared lengths are brought back
// to Q12 before they are multiplied together. That keeps the products at Q24
// and bounds the usable edge length at about 512 units -- far beyond any
// capsule or segment a DS scene will contain, and the same bound the rest of
// the port already documents.
static inline int64_t b3DotNarrow( b3Vec3 a, b3Vec3 b )
{
	return (int64_t)b3Raw( b3Dot( a, b ) );
}

b3Vec3 b3DirectionFromWide( int64_t x, int64_t y, int64_t z )
{
	uint64_t ax = (uint64_t)( x < 0 ? -x : x );
	uint64_t ay = (uint64_t)( y < 0 ? -y : y );
	uint64_t az = (uint64_t)( z < 0 ? -z : z );

	uint64_t largest = ax > ay ? ax : ay;
	largest = largest > az ? largest : az;

	if ( largest == 0 )
	{
		return b3Vec3_zero;
	}

	// Put the largest component in [2^12, 2^13) raw, i.e. between one and two
	// units. Below that, normalization runs out of resolution -- b3RsqrtWide
	// tops out at 128, so a vector shorter than 1/128 of a unit cannot be
	// normalized at all. Above it, the squared length starts eating into the
	// int64 headroom for no benefit.
	int bits = 64 - b3Clz64( largest );
	int shift = bits - 13;

	if ( shift > 0 )
	{
		int64_t half = (int64_t)1 << ( shift - 1 );
		x = ( x + half ) >> shift;
		y = ( y + half ) >> shift;
		z = ( z + half ) >> shift;
	}
	else if ( shift < 0 )
	{
		x <<= -shift;
		y <<= -shift;
		z <<= -shift;
	}

	// The rescale can round every component of a very lopsided vector to
	// zero -- x = 1, y = z = 0 shifts to nothing only if shift > 0, which
	// cannot happen once the largest component is the one being targeted, so
	// this is exact. The assert states the invariant rather than guarding it.
	B3_ASSERT( x != 0 || y != 0 || z != 0 );

	return b3MakeVec3( b3Makeb3f( (int32_t)x ), b3Makeb3f( (int32_t)y ), b3Makeb3f( (int32_t)z ) );
}

b3SegmentDistanceResult b3LineDistance( b3Vec3 p1, b3Vec3 d1, b3Vec3 p2, b3Vec3 d2 )
{
	b3SegmentDistanceResult result;

	// Solve A*x = b
	int64_t a11 = b3DotNarrow( d1, d1 );
	int64_t a12 = -b3DotNarrow( d1, d2 );
	int64_t a21 = b3DotNarrow( d2, d1 );
	int64_t a22 = -b3DotNarrow( d2, d2 );

	b3Vec3 w = b3Sub( p1, p2 );
	int64_t b1 = -b3DotNarrow( d1, w );
	int64_t b2 = -b3DotNarrow( d2, w );

	// Q12 * Q12 -> Q24, held wide.
	int64_t det = a11 * a22 - a12 * a21;

	// Upstream tests det*det against 1000*FLT_MIN, which is a statement about
	// float denormals. The meaningful fixed-point equivalent is simply
	// whether the determinant has any significance left at all: below a
	// couple of quanta at Q24 the lines are parallel to within what the
	// representation can distinguish, and the division would be noise.
	if ( det > -4 && det < 4 )
	{
		// Lines are parallel. Project p2 onto line 1.
		b3c s1 = b3DivWideToC( b3DotNarrow( b3Sub( p2, p1 ), d1 ), a11 );

		result.point1 = b3MulAdd( p1, b3CToF( s1 ), d1 );
		result.fraction1 = s1;
		result.point2 = p2;
		result.fraction2 = b3c_zero;

		return result;
	}

	b3c s1 = b3DivWideToC( a22 * b1 - a12 * b2, det );
	b3c s2 = b3DivWideToC( a11 * b2 - a21 * b1, det );

	result.point1 = b3MulAdd( p1, b3CToF( s1 ), d1 );
	result.fraction1 = s1;
	result.point2 = b3MulAdd( p2, b3CToF( s2 ), d2 );
	result.fraction2 = s2;
	return result;
}

b3SegmentDistanceResult b3SegmentDistance( b3Vec3 p1, b3Vec3 q1, b3Vec3 p2, b3Vec3 q2 )
{
	b3SegmentDistanceResult result;

	b3Vec3 d1 = b3Sub( q1, p1 );
	b3Vec3 d2 = b3Sub( q2, p2 );
	b3Vec3 r = b3Sub( p1, p2 );

	int64_t a = b3DotNarrow( d1, d1 );
	int64_t b = b3DotNarrow( d1, d2 );
	int64_t c = b3DotNarrow( d1, r );
	int64_t e = b3DotNarrow( d2, d2 );
	int64_t f = b3DotNarrow( d2, r );

	// Degeneracy: upstream compares squared lengths against 100*FLT_EPSILON.
	// Here a segment is degenerate exactly when its squared length rounds to
	// zero in Q12 -- shorter than about 1/64 of a unit, which is well below
	// B3_LINEAR_SLOP and so is genuinely a point as far as the solver cares.
	bool degenerate1 = a == 0;
	bool degenerate2 = e == 0;

	if ( degenerate1 && degenerate2 )
	{
		result.point1 = p1;
		result.fraction1 = b3c_zero;
		result.point2 = p2;
		result.fraction2 = b3c_zero;
		return result;
	}

	if ( degenerate1 )
	{
		b3c s2 = b3ClampC( b3DivWideToC( f, e ), b3c_zero, b3c_one );

		result.point1 = p1;
		result.fraction1 = b3c_zero;
		result.point2 = b3MulAdd( p2, b3CToF( s2 ), d2 );
		result.fraction2 = s2;
		return result;
	}

	if ( degenerate2 )
	{
		b3c s1 = b3ClampC( b3DivWideToC( -c, a ), b3c_zero, b3c_one );

		result.point1 = b3MulAdd( p1, b3CToF( s1 ), d1 );
		result.fraction1 = s1;
		result.point2 = p2;
		result.fraction2 = b3c_zero;
		return result;
	}

	// Non-degenerate. denom is a fourth power of length, at Q24.
	int64_t denom = a * e - b * b;

	b3c s1 = denom > 4 ? b3ClampC( b3DivWideToC( b * f - c * e, denom ), b3c_zero, b3c_one ) : b3c_zero;

	// s2 = (b * s1 + f) / e. b*s1 needs s1 back at Q24 to match b's scale,
	// so the multiply is done wide rather than through b3MulCC.
	int64_t bs1 = ( b * (int64_t)b3Raw( s1 ) ) >> B3_C_SHIFT;
	b3c s2 = b3DivWideToC( bs1 + f, e );

	// Clamp s2 and recompute s1 if it left the segment.
	if ( b3Raw( s2 ) < 0 )
	{
		s1 = b3ClampC( b3DivWideToC( -c, a ), b3c_zero, b3c_one );
		s2 = b3c_zero;
	}
	else if ( b3Raw( s2 ) > B3_C_ONE )
	{
		s1 = b3ClampC( b3DivWideToC( b - c, a ), b3c_zero, b3c_one );
		s2 = b3c_one;
	}

	result.point1 = b3MulAdd( p1, b3CToF( s1 ), d1 );
	result.fraction1 = s1;
	result.point2 = b3MulAdd( p2, b3CToF( s2 ), d2 );
	result.fraction2 = s2;

	return result;
}

b3Quat b3ComputeQuatBetweenUnitVectors( b3Vec3 v1, b3Vec3 v2 )
{
	B3_ASSERT( b3IsNormalized( v1 ) );
	B3_ASSERT( b3IsNormalized( v2 ) );

	b3Quat out;

	// The half-way vector. When v1 and v2 are anti-parallel this cancels to
	// zero and the rotation axis is undefined -- upstream detects that with
	// a tolerance of 100*FLT_EPSILON on its squared length. Here the
	// meaningful test is whether the midpoint survives Q12 at all: below a
	// few quanta it carries no direction information, and normalizing it
	// would amplify quantization noise into an arbitrary axis.
	b3Vec3 m = b3Lerp( v1, v2, b3cFromFrac( 1, 2 ) );

	if ( b3LengthSquaredWide( m ) > 16 )
	{
		b3Vec3 axis = b3Cross( v1, m );
		out.v = b3ToDir3( axis );
		out.s = b3FToN( b3Dot( v1, m ) );
	}
	else
	{
		// Anti-parallel: any perpendicular axis is a valid half turn. Pick
		// the one that avoids cancelling.
		if ( b3Raw( b3AbsF( v1.x ) ) > B3_F_ONE / 2 )
		{
			out.v.x = b3FToN( v1.y );
			out.v.y = b3FToN( b3NegF( v1.x ) );
			out.v.z = b3n_zero;
		}
		else
		{
			out.v.x = b3n_zero;
			out.v.y = b3FToN( v1.z );
			out.v.z = b3FToN( b3NegF( v1.y ) );
		}

		out.s = b3n_zero;
	}

	// Upstream's note applies with more force here: normalizing at the end
	// is what makes the construction accurate, and it is also what absorbs
	// the Q12 quantization of the inputs. The result is Q30.
	return b3NormalizeQuat( out );
}

b3Vec3 b3PointToSegmentDistance( b3Vec3 a, b3Vec3 b, b3Vec3 q )
{
	b3Vec3 ab = b3Sub( b, a );
	b3Vec3 aq = b3Sub( q, a );

	int64_t alpha = b3DotNarrow( ab, aq );

	if ( alpha <= 0 )
	{
		// q projects outside [a, b] on the side of a.
		return a;
	}

	int64_t denominator = b3DotNarrow( ab, ab );
	if ( alpha > denominator )
	{
		// q projects outside [a, b] on the side of b.
		return b;
	}

	// q projects inside the interval. Both operands are squared lengths, and
	// the quotient is a fraction in [0, 1] by the two tests above.
	return b3MulAdd( a, b3CToF( b3DivWideToC( alpha, denominator ) ), ab );
}

// =========================================================================
// Composite operations, out of line
// =========================================================================
//
// These are the large ones. b3MulMM expands to three b3MulMV, each nine
// multiplies; b3MakeMatrixFromQuat is three b3RotateVector. Inlined into
// every caller they cost more than they save -- b3ComputeCapsuleMass alone
// was 7.7 KB, most of it one similarity transform expanded twice.
//
// The genuinely hot per-frame operations -- b3Dot, b3Cross, b3RotateVector,
// b3MulMV, b3Normalize -- stay inline in the header. The line is drawn at
// what runs per body per substep versus what runs when a shape is created or
// a transform is rebuilt.

b3Quat b3NormalizeQuat( b3Quat q )
{
	int64_t sq = b3QuatLengthSquaredWide( q );
	if ( sq == 0 )
	{
		return b3Quat_identityFn();
	}

	// sq is Q60. Its square root is Q30.
	int32_t len = (int32_t)b3HwSqrt64( (uint64_t)sq );
	if ( len == 0 )
	{
		return b3Quat_identityFn();
	}

	b3Quat r;
	r.v.x = b3Makeb3n( b3HwDiv64( (int64_t)b3Raw( q.v.x ) << B3_N_SHIFT, len ) );
	r.v.y = b3Makeb3n( b3HwDiv64( (int64_t)b3Raw( q.v.y ) << B3_N_SHIFT, len ) );
	r.v.z = b3Makeb3n( b3HwDiv64( (int64_t)b3Raw( q.v.z ) << B3_N_SHIFT, len ) );
	r.s = b3Makeb3n( b3HwDiv64( (int64_t)b3Raw( q.s ) << B3_N_SHIFT, len ) );
	return r;
}

b3Dir3 b3NormalizeToDir( b3Vec3 v )
{
	int64_t sq = b3LengthSquaredWide( v );
	b3Dir3 d = { b3n_zero, b3n_zero, b3n_zero };
	if ( sq == 0 )
	{
		return d;
	}

	// len is Q12; each component divided by it lands in [-1, 1], so compute
	// the quotient directly at Q30 instead of going through a Q12 normalize
	// and widening -- that would cap the result at 12 bits of precision.
	int32_t len = (int32_t)b3HwSqrt64( (uint64_t)sq );
	if ( len == 0 )
	{
		return d;
	}

	d.x = b3Makeb3n( b3HwDiv64( (int64_t)b3Raw( v.x ) << B3_N_SHIFT, len ) );
	d.y = b3Makeb3n( b3HwDiv64( (int64_t)b3Raw( v.y ) << B3_N_SHIFT, len ) );
	d.z = b3Makeb3n( b3HwDiv64( (int64_t)b3Raw( v.z ) << B3_N_SHIFT, len ) );
	return d;
}

b3Matrix3 b3MulMM( b3Matrix3 a, b3Matrix3 b )
{
	return b3MakeMatrix3( b3MulMV( a, b.cx ), b3MulMV( a, b.cy ), b3MulMV( a, b.cz ) );
}

// Cramer's rule, kept wide throughout.
//
// Upstream forms `1/det` once as a float and multiplies by it three times.
// That is not available here: det is Q36 and its reciprocal would be a Q-minus
// something with no useful representation. Each component is instead a single
// wide divide of a Q36 cofactor product by the Q36 determinant, which lands the
// dimensionless ratio at Q12 in one step -- three divides rather than one, and
// exact where forming the reciprocal first would not be.
//
// The singularity test is upstream's, retargeted: it compares against zero
// rather than a float epsilon, because Q36 has a smallest representable value
// and "denormal" is not a thing here. b3DivWideToF saturates rather than wraps
// for a determinant so small the quotient leaves Q12, which is the same policy
// b3DivWideToC has.

b3Vec3 b3Solve3( b3Matrix3 m, b3Vec3 a )
{
	int64_t det = b3DetWide( m );
	if ( det == 0 )
	{
		return b3Vec3_zeroFn();
	}

	int64_t sx[3], sy[3], sz[3];
	b3CrossWide( sx, m.cy, m.cz );
	b3CrossWide( sy, m.cz, m.cx );
	b3CrossWide( sz, m.cx, m.cy );

	// Each cofactor is Q24 and `a` is Q12, so each dot is Q36 -- the same scale
	// as the determinant, which is what makes the quotient dimensionless.
	int64_t nx = ( sx[0] * b3Raw( a.x ) + sx[1] * b3Raw( a.y ) + sx[2] * b3Raw( a.z ) ) >> B3_F_SHIFT;
	int64_t ny = ( sy[0] * b3Raw( a.x ) + sy[1] * b3Raw( a.y ) + sy[2] * b3Raw( a.z ) ) >> B3_F_SHIFT;
	int64_t nz = ( sz[0] * b3Raw( a.x ) + sz[1] * b3Raw( a.y ) + sz[2] * b3Raw( a.z ) ) >> B3_F_SHIFT;

	return b3MakeVec3( b3DivWideToF( nx, det ), b3DivWideToF( ny, det ), b3DivWideToF( nz, det ) );
}

b3Matrix3 b3InvertMatrix( b3Matrix3 m )
{
	int64_t det = b3DetWide( m );
	if ( det == 0 )
	{
		return b3Mat3_zeroFn();
	}

	int64_t sx[3], sy[3], sz[3];
	b3CrossWide( sx, m.cy, m.cz );
	b3CrossWide( sy, m.cz, m.cx );
	b3CrossWide( sz, m.cx, m.cy );

	// Cofactors and determinant are both Q24, so the divide needs no
	// correction: b3DivWideToF shifts the numerator up by twelve, and
	// (e^2 * 2^24 << 12) / (e^3 * 2^24) is (1/e) * 2^12, which is the Q12
	// inverse entry. See b3DetWide on why the two scales are matched.
	b3Matrix3 out;
	out.cx = b3MakeVec3( b3DivWideToF( sx[0], det ), b3DivWideToF( sx[1], det ), b3DivWideToF( sx[2], det ) );
	out.cy = b3MakeVec3( b3DivWideToF( sy[0], det ), b3DivWideToF( sy[1], det ), b3DivWideToF( sy[2], det ) );
	out.cz = b3MakeVec3( b3DivWideToF( sz[0], det ), b3DivWideToF( sz[1], det ), b3DivWideToF( sz[2], det ) );

	return b3Transpose( out );
}

b3Matrix3 b3MakeMatrixFromQuat( b3Quat q )
{
	return b3MakeMatrix3( b3RotateVector( q, b3Vec3_axisXFn() ), b3RotateVector( q, b3Vec3_axisYFn() ),
						  b3RotateVector( q, b3Vec3_axisZFn() ) );
}

// =========================================================================
// The contact solver's two effective-mass inversions
// =========================================================================
//
// Both follow b3InvertMatrix's structure -- adjugate over determinant, both
// kept wide, both at a *matched* scale so that b3DivWideToF's built-in shift of
// twelve lands the quotient at Q12 with no correction. What differs is which
// scale the two are matched at, and that is chosen by range rather than by
// convenience.

/// Cofactor columns of a Q24 matrix, at Q36.
///
/// The raw products are Q48; shifting twelve leaves Q36, which is the scale
/// the small end of the range needs. b3InvertMatrixW then shifts all three
/// columns down together if the large end needs it.
static void b3CrossWideW( int64_t out[3], b3Vec3W a, b3Vec3W b )
{
	out[0] = ( (int64_t)b3Raw( a.y ) * b3Raw( b.z ) - (int64_t)b3Raw( a.z ) * b3Raw( b.y ) ) >> 12;
	out[1] = ( (int64_t)b3Raw( a.z ) * b3Raw( b.x ) - (int64_t)b3Raw( a.x ) * b3Raw( b.z ) ) >> 12;
	out[2] = ( (int64_t)b3Raw( a.x ) * b3Raw( b.y ) - (int64_t)b3Raw( a.y ) * b3Raw( b.x ) ) >> 12;
}

static uint64_t b3AbsWide( int64_t v )
{
	return (uint64_t)( v < 0 ? -v : v );
}

b3Matrix3 b3InvertMatrixW( b3MatrixW m )
{
	int64_t sx[3], sy[3], sz[3];
	b3CrossWideW( sx, m.cy, m.cz );
	b3CrossWideW( sy, m.cz, m.cx );
	b3CrossWideW( sz, m.cx, m.cy );

	// No fixed cofactor scale spans this range, and that is worth stating
	// because the first draft assumed one did.
	//
	// The determinant itself is fine anywhere: entries from 1e-3 to the Q7.24
	// ceiling of 128 put e^3 across 15 orders, which is 51 bits inside an
	// int64's 63. What does not fit is the *intermediate* -- the cofactor
	// times a column, before the shift that brings the product back down. At
	// Q36 cofactors that product overflows for entries above about 2; at Q24 it
	// survives to 100 but the determinant of a 1e-3 matrix underflows to zero,
	// which reports a perfectly well conditioned tensor as singular and
	// silently disables rolling resistance on every heavy body.
	//
	// So the scale is chosen per matrix. Shifting all three cofactor columns by
	// the same amount leaves their ratios -- and therefore the inverse --
	// untouched, and the determinant inherits the shift because it is built
	// from them. The target is the largest cofactor sitting at exactly 31 bits,
	// which caps the intermediate at 2^62 since a Q24 column is itself bounded
	// by 2^31.
	//
	// The shift goes **both ways**, and the upward half is not an optimisation.
	// A 1e-3 tensor leaves its largest cofactor at 17 bits, and the determinant
	// -- a further three orders down -- then truncates from 68.7 to 68, a 1%
	// error on the inverse. Normalising up by the unused 14 bits makes the same
	// case exact to four decimal places.
	uint64_t largest = 0;
	for ( int i = 0; i < 3; ++i )
	{
		uint64_t a = b3AbsWide( sx[i] );
		uint64_t b = b3AbsWide( sy[i] );
		uint64_t c = b3AbsWide( sz[i] );
		largest = a > largest ? a : largest;
		largest = b > largest ? b : largest;
		largest = c > largest ? c : largest;
	}

	if ( largest == 0 )
	{
		return b3Mat3_zeroFn();
	}

	int shift = ( 64 - b3Clz64( largest ) ) - 31;

	if ( shift > 0 )
	{
		for ( int i = 0; i < 3; ++i )
		{
			sx[i] >>= shift;
			sy[i] >>= shift;
			sz[i] >>= shift;
		}
	}
	else if ( shift < 0 )
	{
		for ( int i = 0; i < 3; ++i )
		{
			sx[i] <<= -shift;
			sy[i] <<= -shift;
			sz[i] <<= -shift;
		}
	}

	// The cofactors are now Q(36 - shift). Against a Q24 column that is
	// Q(60 - shift), so twenty-four brings the determinant back to the
	// cofactors' own scale -- which is what makes b3DivWideToF's built-in shift
	// of twelve land the quotient at Q12 with no correction factor. Matching
	// the two is the same trick b3InvertMatrix uses; only the scale they are
	// matched *at* is decided here rather than fixed.
	int64_t det = ( sx[0] * b3Raw( m.cx.x ) + sx[1] * b3Raw( m.cx.y ) + sx[2] * b3Raw( m.cx.z ) ) >> B3_W_SHIFT;

	if ( det == 0 )
	{
		return b3Mat3_zeroFn();
	}

	b3Matrix3 out;
	out.cx = b3MakeVec3( b3DivWideToF( sx[0], det ), b3DivWideToF( sx[1], det ), b3DivWideToF( sx[2], det ) );
	out.cy = b3MakeVec3( b3DivWideToF( sy[0], det ), b3DivWideToF( sy[1], det ), b3DivWideToF( sy[2], det ) );
	out.cz = b3MakeVec3( b3DivWideToF( sz[0], det ), b3DivWideToF( sz[1], det ), b3DivWideToF( sz[2], det ) );

	return b3Transpose( out );
}

b3SymMatrix2 b3InvertSym2W( b3SymMatrix2W k )
{
	// A 2x2 determinant is only *quadratic* in the entries, so it needs none of
	// the headroom the 3x3 above does: Q24 entries give a Q48 product, and
	// shifting 24 leaves the determinant at Q24 -- the same scale as the
	// adjugate, which is just the entries themselves rearranged. Matched, so
	// b3DivWideToF lands at Q12.
	int64_t det = ( (int64_t)b3Raw( k.xx ) * b3Raw( k.yy ) - (int64_t)b3Raw( k.xy ) * b3Raw( k.xy ) ) >> B3_W_SHIFT;

	if ( det == 0 )
	{
		// Rank deficient: the two tangent directions see arms that make the
		// contact unable to resist one of them. No friction impulse rather
		// than an unbounded one.
		b3SymMatrix2 zero = { b3f_zero, b3f_zero, b3f_zero };
		return zero;
	}

	b3SymMatrix2 out;
	out.xx = b3DivWideToF( (int64_t)b3Raw( k.yy ), det );
	out.yy = b3DivWideToF( (int64_t)b3Raw( k.xx ), det );
	out.xy = b3DivWideToF( -(int64_t)b3Raw( k.xy ), det );
	return out;
}

// =========================================================================
// Inverse inertia
// =========================================================================

/// Q24 matrix times a Q24 column, staying at Q24.
static b3Vec3W b3MulMWVW( b3MatrixW m, b3Vec3W v )
{
	return b3MakeVec3W( b3AddW( b3AddW( b3MulWW( m.cx.x, v.x ), b3MulWW( m.cy.x, v.y ) ), b3MulWW( m.cz.x, v.z ) ),
						b3AddW( b3AddW( b3MulWW( m.cx.y, v.x ), b3MulWW( m.cy.y, v.y ) ), b3MulWW( m.cz.y, v.z ) ),
						b3AddW( b3AddW( b3MulWW( m.cx.z, v.x ), b3MulWW( m.cy.z, v.y ) ), b3MulWW( m.cz.z, v.z ) ) );
}

b3MatrixW b3MulMWMW( b3MatrixW a, b3MatrixW b )
{
	return b3MakeMatrixW( b3MulMWVW( a, b.cx ), b3MulMWVW( a, b.cy ), b3MulMWVW( a, b.cz ) );
}

/// Accumulate `skew(r) * I * skew(r)^T` into a wide Q24 3x3, in int64.
///
/// Note the transpose: upstream writes `-skew(r) * I * skew(r)`, and
/// `skew(r)^T == -skew(r)`, so the two are the same expression with the sign
/// already folded in -- and in this form the result is visibly symmetric
/// positive semi-definite, which is what makes it safe to hand to a symmetric
/// inverse.
///
/// Everything stays int64 because the whole point of the caller is that the
/// entries do not fit b3iw.
static void b3AccumulateSkewInertia( int64_t k[3][3], b3MatrixW invI, b3Vec3 r )
{
	// skew(r) as plain Q12 integers.
	const int64_t rx = b3Raw( r.x );
	const int64_t ry = b3Raw( r.y );
	const int64_t rz = b3Raw( r.z );

	const int64_t s[3][3] = {
		{ 0, rz, -ry },
		{ -rz, 0, rx },
		{ ry, -rx, 0 },
	};

	const int64_t iw[3][3] = {
		{ b3Raw( invI.cx.x ), b3Raw( invI.cy.x ), b3Raw( invI.cz.x ) },
		{ b3Raw( invI.cx.y ), b3Raw( invI.cy.y ), b3Raw( invI.cz.y ) },
		{ b3Raw( invI.cx.z ), b3Raw( invI.cy.z ), b3Raw( invI.cz.z ) },
	};

	// m = I * skew(r)^T, at Q24 * Q12 -> Q36, kept wide.
	int64_t m[3][3];
	for ( int i = 0; i < 3; ++i )
	{
		for ( int j = 0; j < 3; ++j )
		{
			// skew^T[a][j] == skew[j][a]
			m[i][j] = iw[i][0] * s[j][0] + iw[i][1] * s[j][1] + iw[i][2] * s[j][2];
		}
	}

	// k += skew(r) * m, at Q12 * Q36 -> Q48, shifted back to Q24.
	for ( int i = 0; i < 3; ++i )
	{
		for ( int j = 0; j < 3; ++j )
		{
			int64_t sum = s[i][0] * m[0][j] + s[i][1] * m[1][j] + s[i][2] * m[2][j];
			k[i][j] += sum >> ( B3_F_SHIFT + B3_F_SHIFT );
		}
	}
}

/// Invert a symmetric Q24 matrix accumulated wide, without requiring its
/// entries to fit a b3iw on the way in.
///
/// Shared by every effective mass a joint builds by accumulation, which is why
/// it is factored out rather than repeated: b3InvertPointMass, whose entries
/// grow as the square of a lever arm, and b3InvertRotationMass, whose entries
/// are a sum of two inverse inertias. Both overflow Q7.24 for perfectly
/// ordinary scenes, and both are fixed the same way.
static b3Matrix3 b3InvertAccumulated( int64_t k[3][3] )
{
	// Scale the whole matrix down until every entry fits a b3iw, counting the
	// shifts so they can be undone on the inverse. Uniform, so it commutes
	// with inversion exactly -- unlike clamping an entry, which would break
	// the symmetry the inverse relies on.
	int64_t largest = 0;
	for ( int i = 0; i < 3; ++i )
	{
		for ( int j = 0; j < 3; ++j )
		{
			int64_t magnitude = k[i][j] < 0 ? -k[i][j] : k[i][j];
			if ( magnitude > largest )
			{
				largest = magnitude;
			}
		}
	}

	int shift = 0;
	while ( largest > INT32_MAX / 2 && shift < 32 )
	{
		largest >>= 1;
		shift += 1;
	}

	b3MatrixW scaled;
	scaled.cx = b3MakeVec3W( b3Makeb3iw( (int32_t)( k[0][0] >> shift ) ), b3Makeb3iw( (int32_t)( k[1][0] >> shift ) ),
							 b3Makeb3iw( (int32_t)( k[2][0] >> shift ) ) );
	scaled.cy = b3MakeVec3W( b3Makeb3iw( (int32_t)( k[0][1] >> shift ) ), b3Makeb3iw( (int32_t)( k[1][1] >> shift ) ),
							 b3Makeb3iw( (int32_t)( k[2][1] >> shift ) ) );
	scaled.cz = b3MakeVec3W( b3Makeb3iw( (int32_t)( k[0][2] >> shift ) ), b3Makeb3iw( (int32_t)( k[1][2] >> shift ) ),
							 b3Makeb3iw( (int32_t)( k[2][2] >> shift ) ) );

	b3Matrix3 inv = b3InvertMatrixW( scaled );

	if ( shift == 0 )
	{
		return inv;
	}

	// inverse( K >> s ) == inverse( K ) << s, so undoing the scale means
	// shifting the inverse back down by the same count.
	inv.cx = b3MakeVec3( b3Makeb3f( b3Raw( inv.cx.x ) >> shift ), b3Makeb3f( b3Raw( inv.cx.y ) >> shift ),
						 b3Makeb3f( b3Raw( inv.cx.z ) >> shift ) );
	inv.cy = b3MakeVec3( b3Makeb3f( b3Raw( inv.cy.x ) >> shift ), b3Makeb3f( b3Raw( inv.cy.y ) >> shift ),
						 b3Makeb3f( b3Raw( inv.cy.z ) >> shift ) );
	inv.cz = b3MakeVec3( b3Makeb3f( b3Raw( inv.cz.x ) >> shift ), b3Makeb3f( b3Raw( inv.cz.y ) >> shift ),
						 b3Makeb3f( b3Raw( inv.cz.z ) >> shift ) );
	return inv;
}

b3Matrix3 b3InvertPointMass( b3iw mA, b3MatrixW iA, b3Vec3 rA, b3iw mB, b3MatrixW iB, b3Vec3 rB )
{
	int64_t k[3][3] = { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } };

	b3AccumulateSkewInertia( k, iA, rA );
	b3AccumulateSkewInertia( k, iB, rB );

	// The two inverse masses are summed here, in the int64 the rest of K is
	// already accumulated in, rather than by the caller.
	//
	// Stage 5, and the same defect as the inertia sum one type over: Q7.24
	// tops out at 128, a 0.1 m sphere at density 1 weighs 4.2 g and has an
	// inverse mass of 239 -- so a single light body is already past the
	// ceiling, and b3AddW wrapped their sum. Two pebbles on a hinge is not a
	// corner case, and in a device build it wrapped silently.
	const int64_t diagonal = (int64_t)b3Raw( mA ) + (int64_t)b3Raw( mB );
	k[0][0] += diagonal;
	k[1][1] += diagonal;
	k[2][2] += diagonal;

	return b3InvertAccumulated( k );
}

b3Matrix3 b3InvertRotationMass( b3MatrixW iA, b3MatrixW iB )
{
	// No lever arms and no mass term: the rotational effective mass a joint's
	// spring and motor push through is simply the inverse of the two bodies'
	// summed inverse inertias.
	//
	// It cannot be b3InvertMatrixW on the sum directly, for the range reason
	// rather than the algebra: an inverse inertia is *large* for a small light
	// body -- a 0.1 m sphere at the default density is near 60 -- so two of
	// them sum to about 120 against Q7.24's ceiling of 128. That is not a
	// corner case, it is a ragdoll's hand. Accumulating wide and letting
	// b3InvertAccumulated scale the whole matrix costs one pass over nine
	// entries and removes the ceiling entirely.
	//
	// **Each addend is widened before the addition, not after.** b3Raw returns
	// an int32, so `b3Raw( a ) + b3Raw( b )` is evaluated in int -- and only
	// then converted to the int64 it is being stored into. Stage 4 wrote it
	// that way and it carried the very wrap this function exists to remove:
	// two inverse inertias of 70 sum to 140, whose Q7.24 raw is 2.35e9 against
	// INT32_MAX's 2.15e9, so the sum came out **negative** and the effective
	// mass with it. A negative effective mass does not merely lose precision,
	// it inverts the constraint -- it drives the two bodies apart in proportion
	// to how hard they are held together, which is an explosion rather than a
	// drift. Measured: a chain of welded 0.6 m boxes reached 89 rad/s on its
	// first step and 1,100 by its third.
	//
	// The threshold is a sum of 128, which is Q7.24's ceiling and reachable by
	// two ordinary light bodies, so this fired for the spherical joint too --
	// it is Stage 4's own routine, and Stage 5's weld is only what happened to
	// drive it hard enough to see.
	int64_t k[3][3] = {
		{ (int64_t)b3Raw( iA.cx.x ) + b3Raw( iB.cx.x ), (int64_t)b3Raw( iA.cy.x ) + b3Raw( iB.cy.x ),
		  (int64_t)b3Raw( iA.cz.x ) + b3Raw( iB.cz.x ) },
		{ (int64_t)b3Raw( iA.cx.y ) + b3Raw( iB.cx.y ), (int64_t)b3Raw( iA.cy.y ) + b3Raw( iB.cy.y ),
		  (int64_t)b3Raw( iA.cz.y ) + b3Raw( iB.cz.y ) },
		{ (int64_t)b3Raw( iA.cx.z ) + b3Raw( iB.cx.z ), (int64_t)b3Raw( iA.cy.z ) + b3Raw( iB.cy.z ),
		  (int64_t)b3Raw( iA.cz.z ) + b3Raw( iB.cz.z ) },
	};

	return b3InvertAccumulated( k );
}

/// b3InvertAccumulated, on a symmetric 2x2 given as `{ xx, yy, xy }`.
///
/// The 3x3 version cannot be reused: this one carries a b3SymMatrix2W through
/// b3InvertSym2W, whose determinant is quadratic rather than cubic and therefore
/// needs no Q36 step at all. But the *scaling* argument is identical, and Stage
/// 6 gave it a second caller -- so it is factored out rather than written twice.
/// A shift search that exists in two places is one that gets fixed in one.
static b3SymMatrix2 b3InvertAccumulated2( const int64_t k[3] )
{
	int64_t largest = 0;
	for ( int i = 0; i < 3; ++i )
	{
		int64_t magnitude = k[i] < 0 ? -k[i] : k[i];
		if ( magnitude > largest )
		{
			largest = magnitude;
		}
	}

	int shift = 0;
	while ( largest > INT32_MAX / 2 && shift < 32 )
	{
		largest >>= 1;
		shift += 1;
	}

	b3SymMatrix2W scaled;
	scaled.xx = b3Makeb3iw( (int32_t)( k[0] >> shift ) );
	scaled.yy = b3Makeb3iw( (int32_t)( k[1] >> shift ) );
	scaled.xy = b3Makeb3iw( (int32_t)( k[2] >> shift ) );

	b3SymMatrix2 inv = b3InvertSym2W( scaled );

	if ( shift == 0 )
	{
		return inv;
	}

	// inverse( K >> s ) == inverse( K ) << s, so the scale is undone by shifting
	// the inverse back down. Uniform, so it commutes with inversion exactly.
	inv.xx = b3Makeb3f( b3Raw( inv.xx ) >> shift );
	inv.yy = b3Makeb3f( b3Raw( inv.yy ) >> shift );
	inv.xy = b3Makeb3f( b3Raw( inv.xy ) >> shift );

	// The shift back is where a *representable* inverse can stop being a valid
	// one, and the failure is not a rounding error.
	//
	// A large K has a small inverse, and shifting right truncates toward zero.
	// The diagonal reaches zero first, because it is the larger divisor -- so
	// there is a band where `xx` and `yy` have underflowed to nothing while `xy`
	// still holds one Q12 quantum. That leaves
	//
	//     inverse( K ) = [  0  -q ]     with eigenvalues +/-q and det = -q^2
	//                    [ -q   0 ]
	//
	// which is **indefinite**: applied to a velocity error it returns an impulse
	// along the wrong axis with the wrong sign, so the constraint pushes apart
	// what it exists to hold together. Silent, and it depends on geometry rather
	// than on any input a caller would think to check -- measured on a lever arm
	// of 9 m with two inverse inertias near 60, which is a long rail and not an
	// absurd one.
	//
	// A matrix this small cannot carry an impulse worth applying in any case, so
	// the answer is the port's usual one for an effective mass it cannot
	// represent: zero, meaning apply none. Never a matrix that inverts the sign.
	if ( b3Raw( inv.xx ) <= 0 || b3Raw( inv.yy ) <= 0 ||
		 (int64_t)b3Raw( inv.xx ) * b3Raw( inv.yy ) <= (int64_t)b3Raw( inv.xy ) * b3Raw( inv.xy ) )
	{
		b3SymMatrix2 zero = { b3f_zero, b3f_zero, b3f_zero };
		return zero;
	}

	return inv;
}

b3SymMatrix2 b3InvertPerpMass( b3Vec3 uX, b3Vec3 uY, b3MatrixW iA, b3MatrixW iB )
{
	const int64_t k[3] = {
		b3AxisInertiaCrossWide( uX, uX, iA, iB ),
		b3AxisInertiaCrossWide( uY, uY, iA, iB ),
		b3AxisInertiaCrossWide( uX, uY, iA, iB ),
	};

	return b3InvertAccumulated2( k );
}

b3SymMatrix2 b3InvertPointLineMass( b3iw mA, b3MatrixW iA, b3Vec3 sAy, b3Vec3 sAz, b3iw mB, b3MatrixW iB, b3Vec3 sBy,
									b3Vec3 sBz )
{
	// The diagonal carries the inverse-mass sum and the off-diagonal does not:
	// the linear part of K is `(mA + mB) * I` in the two-dimensional constraint
	// space, whose off-diagonal is zero by construction.
	//
	// The sum is formed here, in the int64 the rest of K accumulates in, and
	// never by the caller -- b3InvertPointMass's rule for the same quantity, and
	// for the same reason: B3_MIN_MASS_RAW caps a single inverse mass at ~124
	// against Q7.24's ceiling of 128, so any two bodies below about 16 g sum
	// past it and b3AddW wraps the result negative.
	const int64_t diagonal = (int64_t)b3Raw( mA ) + (int64_t)b3Raw( mB );

	const int64_t k[3] = {
		diagonal + b3LeverInertiaCrossWide( iA, sAy, sAy, iB, sBy, sBy ),
		diagonal + b3LeverInertiaCrossWide( iA, sAz, sAz, iB, sBz, sBz ),
		b3LeverInertiaCrossWide( iA, sAy, sAz, iB, sBy, sBz ),
	};

	return b3InvertAccumulated2( k );
}

/// Rotation matrix columns from a quaternion, at Q30. Declared in math_fixed.h,
/// where the reasoning and the two callers are written up.
///
/// b3MakeMatrixFromQuat returns Q12, because that is what b3Matrix3 is. That
/// is fine for a transform handed to the renderer and wrong here, and the
/// shadow checker says so in one line: a Q12 rotation matrix is not exactly
/// orthonormal, so R * I^-1 * Rᵀ for an *isotropic* tensor -- where every
/// off-diagonal term must cancel to exactly zero -- comes out at 1e-3 instead.
/// That is half a permille of the diagonal, harmless in itself, but it is
/// 16000 quanta at Q24 and it would pin the checker's worst-error statistic
/// for every later phase.
///
/// A rotation matrix entry is bounded by 1, which is precisely the range b3n
/// exists for. Every term below is a product of two Q30 quaternion components,
/// so nothing is narrowed anywhere.
/// `1 - 2*(a + b)` at Q30, with the doubling done wide.
///
/// The three diagonal entries of a rotation matrix have this shape, and writing
/// it as nested b3AddN -- which is how this function was first written -- forms
/// `2*(a + b)` as a b3n. **That intermediate reaches exactly 2.0 at a half
/// turn**, and 2.0 at Q30 is 2^31, one past what an int32 holds, so the debug
/// shadow checker reported an OVERFLOW and the release build wrapped the sign.
/// The *result* is always in [-1, 1] for a unit quaternion and was never the
/// problem; only the way it was reached.
///
/// Found by test_joint_angular_separation_wheel, which tips a wheel through a
/// quarter turn against a joint frame already rotated a quarter turn -- so the
/// composed rotation passes exactly through a half turn about z, where
/// `yy + zz` is exactly one. The wheel joint's own degenerate sweep missed it
/// because it sweeps tip angles rather than the composed frame.
///
/// Accumulating wide and narrowing once is what the rest of the file does with
/// dot products for the same reason.
static b3n b3OneMinusTwiceN( b3n a, b3n b )
{
	int64_t wide = (int64_t)B3_N_ONE - 2 * ( (int64_t)b3Raw( a ) + (int64_t)b3Raw( b ) );
	return b3Makeb3nRef( (int32_t)wide, B3_REF( 1.0 - 2.0 * ( b3RefN( a ) + b3RefN( b ) ) ) );
}

void b3QuatColumnsN( b3Quat q, b3Dir3 out[3] )
{
	const b3n x = q.v.x;
	const b3n y = q.v.y;
	const b3n z = q.v.z;
	const b3n s = q.s;

	const b3n xx = b3MulNN( x, x );
	const b3n yy = b3MulNN( y, y );
	const b3n zz = b3MulNN( z, z );
	const b3n xy = b3MulNN( x, y );
	const b3n xz = b3MulNN( x, z );
	const b3n yz = b3MulNN( y, z );
	const b3n sx = b3MulNN( s, x );
	const b3n sy = b3MulNN( s, y );
	const b3n sz = b3MulNN( s, z );

	// The off-diagonal entries stay as nested b3AddN: each is 2*(p +/- q) where
	// |p|, |q| <= 1/2 for a unit quaternion, so neither the intermediate nor the
	// result can leave Q30. Only the three diagonals need b3OneMinusTwiceN.
	out[0] = b3MakeDir3( b3OneMinusTwiceN( yy, zz ), b3AddN( b3AddN( xy, sz ), b3AddN( xy, sz ) ),
						 b3SubN( b3AddN( xz, xz ), b3AddN( sy, sy ) ) );

	out[1] = b3MakeDir3( b3SubN( b3AddN( xy, xy ), b3AddN( sz, sz ) ), b3OneMinusTwiceN( xx, zz ),
						 b3AddN( b3AddN( yz, sx ), b3AddN( yz, sx ) ) );

	out[2] = b3MakeDir3( b3AddN( b3AddN( xz, sy ), b3AddN( xz, sy ) ), b3SubN( b3AddN( yz, yz ), b3AddN( sx, sx ) ),
						 b3OneMinusTwiceN( xx, yy ) );
}

b3MatrixW b3RotateInertiaW( b3Quat q, b3MatrixW localInverse )
{
	b3Dir3 r[3];
	b3QuatColumnsN( q, r );

	// b3MulWN is Q24 * Q30 -> Q24, and its comment upstream in b3fixed.h says
	// what it is for: "inverse inertia rotated by a unit-vector component".
	// This is that use.

	// t = I^-1 * Rᵀ. Column j of Rᵀ is row j of R, which is component j of
	// each of the three columns.
	b3Vec3W t[3];
	const b3Vec3W icols[3] = { localInverse.cx, localInverse.cy, localInverse.cz };

	for ( int j = 0; j < 3; ++j )
	{
		const b3n c0 = j == 0 ? r[0].x : ( j == 1 ? r[0].y : r[0].z );
		const b3n c1 = j == 0 ? r[1].x : ( j == 1 ? r[1].y : r[1].z );
		const b3n c2 = j == 0 ? r[2].x : ( j == 1 ? r[2].y : r[2].z );

		t[j] = b3MakeVec3W(
			b3AddW( b3AddW( b3MulWN( icols[0].x, c0 ), b3MulWN( icols[1].x, c1 ) ), b3MulWN( icols[2].x, c2 ) ),
			b3AddW( b3AddW( b3MulWN( icols[0].y, c0 ), b3MulWN( icols[1].y, c1 ) ), b3MulWN( icols[2].y, c2 ) ),
			b3AddW( b3AddW( b3MulWN( icols[0].z, c0 ), b3MulWN( icols[1].z, c1 ) ), b3MulWN( icols[2].z, c2 ) ) );
	}

	// out = R * t
	b3MatrixW out;
	b3Vec3W* ocols[3] = { &out.cx, &out.cy, &out.cz };

	for ( int j = 0; j < 3; ++j )
	{
		b3Vec3W c = t[j];
		*ocols[j] = b3MakeVec3W(
			b3AddW( b3AddW( b3MulWN( c.x, r[0].x ), b3MulWN( c.y, r[1].x ) ), b3MulWN( c.z, r[2].x ) ),
			b3AddW( b3AddW( b3MulWN( c.x, r[0].y ), b3MulWN( c.y, r[1].y ) ), b3MulWN( c.z, r[2].y ) ),
			b3AddW( b3AddW( b3MulWN( c.x, r[0].z ), b3MulWN( c.y, r[1].z ) ), b3MulWN( c.z, r[2].z ) ) );
	}

	return out;
}

/// Number of significant bits in |v|.
static int b3Bits64( int64_t v )
{
	uint64_t a = (uint64_t)( v < 0 ? -v : v );
	return a == 0 ? 0 : 64 - b3Clz64( a );
}

/// Compute (num << shift) / den, using only the hardware divider.
///
/// Two constraints shape this. b3HwDiv64 takes a 64-bit numerator and a
/// *32-bit* denominator and returns 32 bits, because that is what the DS
/// divider registers are; and a bare `/` on two int64s lowers to
/// __aeabi_uldivmod, which this build is checked not to link. So the
/// denominator is first brought within 31 bits by shifting it down, with the
/// same amount taken out of `shift` so the ratio is preserved -- shifting both
/// sides of a fraction discards only bits the result could not have held --
/// and the rest of the shift is applied to the numerator.
///
/// The caller is expected to have chosen `shift` so the quotient fits 32 bits.
/// The saturating guard below is a safety net rather than the mechanism: it
/// compares against `den << 31` before dividing, because on hardware an
/// over-range quotient is silently truncated rather than detectable after.
static int32_t b3ShiftedDiv( int64_t num, int64_t den, int shift )
{
	if ( den == 0 )
	{
		return 0;
	}

	bool negative = ( num < 0 ) != ( den < 0 );

	int denBits = b3Bits64( den );
	if ( denBits > 31 )
	{
		int k = denBits - 31;
		den = ( den + ( (int64_t)1 << ( k - 1 ) ) ) >> k;
		shift -= k;

		if ( den == 0 )
		{
			return negative ? INT32_MIN : INT32_MAX;
		}
	}

	// Apply the remaining shift to the numerator, as far as int64 allows.
	while ( shift > 0 )
	{
		int room = 62 - b3Bits64( num );
		if ( room <= 0 )
		{
			break;
		}

		int step = shift < room ? shift : room;
		num <<= step;
		shift -= step;
	}

	if ( shift < 0 )
	{
		num = ( num + ( (int64_t)1 << ( -shift - 1 ) ) ) >> ( -shift );
		shift = 0;
	}

	if ( shift > 0 )
	{
		// The numerator filled before the shift was spent, so the quotient is
		// far out of range.
		return negative ? INT32_MIN : INT32_MAX;
	}

	// Would the quotient overflow 32 bits? Check before dividing.
	{
		int64_t absNum = num < 0 ? -num : num;
		int64_t absDen = den < 0 ? -den : den;
		if ( absDen <= ( absNum >> 31 ) )
		{
			return negative ? INT32_MIN : INT32_MAX;
		}
	}

	return b3HwDiv64( num, (int32_t)den );
}

b3MatrixW b3InvertInertia( b3Matrix3 unitInertia, b3f mass )
{
	// The tensor is symmetric, so six raws describe it. Reading the upper
	// triangle rather than averaging with the lower one is deliberate: the
	// callers build it from b3SteinerUnit and the b3*UnitInertia helpers,
	// which are symmetric by construction, and an asymmetric input is a bug
	// worth tripping an assert over rather than quietly smoothing away.
	int32_t a = b3Raw( unitInertia.cx.x );
	int32_t b = b3Raw( unitInertia.cy.x );
	int32_t c = b3Raw( unitInertia.cz.x );
	int32_t d = b3Raw( unitInertia.cy.y );
	int32_t e = b3Raw( unitInertia.cz.y );
	int32_t f = b3Raw( unitInertia.cz.z );

	B3_ASSERT( b == b3Raw( unitInertia.cx.y ) );
	B3_ASSERT( c == b3Raw( unitInertia.cx.z ) );
	B3_ASSERT( e == b3Raw( unitInertia.cy.z ) );

	const int64_t massRaw = (int64_t)b3Raw( mass );
	if ( massRaw <= 0 )
	{
		return b3MatW_zeroFn();
	}

	// Form the absolute tensor I = mass * U, wide, and leave it at Q24.
	//
	// This is the quantity b3MassData deliberately never stores, because at
	// Q12 it overflows for anything larger than a toy. Here it exists only as
	// an int64 that the next step normalizes and then consumes, so its range
	// never has to fit a register: mass and U raws are each under 2^31, so the
	// product is under 2^62.
	//
	// Not narrowing that product back to Q12 is the point. Inertia is small
	// for small bodies, not just large for large ones: a 5 cm sphere at the
	// density of water has I = 5.1e-4, which is *two raw units* at Q12. The
	// inversion would then be working from one bit of input and the answer
	// would be quantization, not physics. At Q24 the same tensor is 8584 raw
	// units and the result is good to a part in 8000.
	int64_t entries[6];
	{
		const int32_t u[6] = { a, b, c, d, e, f };
		for ( int i = 0; i < 6; ++i )
		{
			entries[i] = massRaw * (int64_t)u[i];
		}
	}

	// Normalize so the largest entry has at most 20 significant bits.
	//
	// The determinant is a triple product of entries, so it needs 3p + 2 bits
	// for entries of p bits, and int64 gives 62. p = 20 is the largest value
	// that fits, and it leaves the cofactors -- double products, 41 bits --
	// with plenty of room. The shift is undone exactly at the end, because
	// scaling a matrix by 2^-s scales its inverse by 2^s.
	int64_t maxEntry = 0;
	for ( int i = 0; i < 6; ++i )
	{
		int64_t m = entries[i] < 0 ? -entries[i] : entries[i];
		if ( m > maxEntry )
		{
			maxEntry = m;
		}
	}

	if ( maxEntry == 0 )
	{
		return b3MatW_zeroFn();
	}

	int s = 0;
	{
		int bits = b3Bits64( maxEntry );
		if ( bits > 20 )
		{
			s = bits - 20;
		}
	}

	const int64_t half = s > 0 ? ( (int64_t)1 << ( s - 1 ) ) : 0;
	const int64_t A = ( entries[0] + half ) >> s;
	const int64_t B = ( entries[1] + half ) >> s;
	const int64_t C = ( entries[2] + half ) >> s;
	const int64_t D = ( entries[3] + half ) >> s;
	const int64_t E = ( entries[4] + half ) >> s;
	const int64_t F = ( entries[5] + half ) >> s;

	// Adjugate of a symmetric matrix, itself symmetric.
	const int64_t c11 = D * F - E * E;
	const int64_t c12 = C * E - B * F;
	const int64_t c13 = B * E - C * D;
	const int64_t c22 = A * F - C * C;
	const int64_t c23 = B * C - A * E;
	const int64_t c33 = A * D - B * B;

	const int64_t det = A * c11 + B * c12 + C * c13;

	// A physical inertia tensor is positive definite, so a non-positive
	// determinant means the input was degenerate: a zero-density body, a
	// fixed-rotation body, or a shape whose extents collapsed in Q12. Zero is
	// the right answer for all three -- it is what "cannot rotate" means to
	// the solver.
	if ( det <= 0 )
	{
		return b3MatW_zeroFn();
	}

	// I^-1 at Q24 is cofactor * 2^(48-s) / det. Derivation: with the shifted
	// entries read as Q24, the cofactors carry Q48 and the determinant Q72, so
	// the raw ratio is the real inverse times 2^-24; the normalization adds
	// 2^-s; and landing at Q24 adds 2^24.
	const int64_t cof[6] = { c11, c12, c13, c22, c23, c33 };
	int64_t maxCof = 0;
	for ( int i = 0; i < 6; ++i )
	{
		int64_t m = cof[i] < 0 ? -cof[i] : cof[i];
		if ( m > maxCof )
		{
			maxCof = m;
		}
	}

	// The uniform clamp, applied *before* the division rather than after.
	//
	// Q7.24 tops out at 128 and a small enough body genuinely wants more --
	// a 5 cm sphere at the density of water asks for about 1900. Taking the
	// excess out of the shift scales every entry by the same power of two,
	// which is a uniform scaling of the forward tensor, so symmetry and
	// positive definiteness both survive. Entry-wise saturation after the fact
	// would preserve neither, and an indefinite effective mass is how a solver
	// explodes. The body simply behaves as though it had more rotational
	// inertia than asked for.
	//
	// Doing it here rather than afterwards is also what keeps the quotient
	// inside the 32 bits the hardware divider returns: bits(cof/det * 2^shift)
	// is at most bits(cof) + shift - bits(det) + 1.
	const int baseShift = 48 - s;
	int estimatedBits = b3Bits64( maxCof ) + baseShift - b3Bits64( det ) + 1;
	int excess = estimatedBits > 31 ? estimatedBits - 31 : 0;
	int shift = baseShift - excess;

	int32_t inv[6];
	for ( int i = 0; i < 6; ++i )
	{
		inv[i] = b3ShiftedDiv( cof[i], det, shift );
	}

	// The estimate above is an upper bound and runs one or two bits loose, so
	// a tensor that needed one halving can come back scaled by four. That is
	// visible: it is the difference between a body that resists spin a little
	// more than asked and one that resists it four times as much. Measure the
	// headroom the first pass actually left and give back what was not needed.
	//
	// The cost is six more divides on a path that runs once per mass change,
	// and it only runs at all for a body small enough to have clamped.
	if ( excess > 0 )
	{
		int32_t peak = 0;
		for ( int i = 0; i < 6; ++i )
		{
			int32_t m = inv[i] < 0 ? -inv[i] : inv[i];
			if ( m > peak )
			{
				peak = m;
			}
		}

		int spare = 31 - b3Bits64( peak );
		if ( spare > 0 )
		{
			int give = spare < excess ? spare : excess;
			excess -= give;
			shift += give;

			for ( int i = 0; i < 6; ++i )
			{
				inv[i] = b3ShiftedDiv( cof[i], det, shift );
			}
		}
	}

	const b3iw ixx = b3Makeb3iwRef( inv[0], B3_REF( (double)inv[0] / (double)B3_W_ONE ) );
	const b3iw ixy = b3Makeb3iwRef( inv[1], B3_REF( (double)inv[1] / (double)B3_W_ONE ) );
	const b3iw ixz = b3Makeb3iwRef( inv[2], B3_REF( (double)inv[2] / (double)B3_W_ONE ) );
	const b3iw iyy = b3Makeb3iwRef( inv[3], B3_REF( (double)inv[3] / (double)B3_W_ONE ) );
	const b3iw iyz = b3Makeb3iwRef( inv[4], B3_REF( (double)inv[4] / (double)B3_W_ONE ) );
	const b3iw izz = b3Makeb3iwRef( inv[5], B3_REF( (double)inv[5] / (double)B3_W_ONE ) );

	return b3MakeMatrixW( b3MakeVec3W( ixx, ixy, ixz ), b3MakeVec3W( ixy, iyy, iyz ), b3MakeVec3W( ixz, iyz, izz ) );
}

b3Quat b3NLerp( b3Quat a, b3Quat b, b3c t )
{
	// Take the shorter arc.
	if ( b3Raw( b3DotQuat( a, b ) ) < 0 )
	{
		b = b3NegateQuat( b );
	}

	b3c invT = b3SubC( b3c_one, t );
	b3Quat r;
	r.v.x = b3AddN( b3CToN( b3MulCC( b3NToC( a.v.x ), invT ) ), b3CToN( b3MulCC( b3NToC( b.v.x ), t ) ) );
	r.v.y = b3AddN( b3CToN( b3MulCC( b3NToC( a.v.y ), invT ) ), b3CToN( b3MulCC( b3NToC( b.v.y ), t ) ) );
	r.v.z = b3AddN( b3CToN( b3MulCC( b3NToC( a.v.z ), invT ) ), b3CToN( b3MulCC( b3NToC( b.v.z ), t ) ) );
	r.s = b3AddN( b3CToN( b3MulCC( b3NToC( a.s ), invT ) ), b3CToN( b3MulCC( b3NToC( b.s ), t ) ) );
	return b3NormalizeQuat( r );
}

b3Quat b3IntegrateRotation( b3Quat q, b3Vec3 omega, b3t h )
{
	// Half the rotation increment, per axis, at Q30.
	b3n hx = b3HalfN( b3MulFTToN( omega.x, h ) );
	b3n hy = b3HalfN( b3MulFTToN( omega.y, h ) );
	b3n hz = b3HalfN( b3MulFTToN( omega.z, h ) );

	// dq = (hx, hy, hz, 0) * q, expanded so the zero scalar part costs
	// nothing.
	b3Quat dq;
	dq.v.x = b3AddN( b3MulNN( hx, q.s ), b3SubN( b3MulNN( hy, q.v.z ), b3MulNN( hz, q.v.y ) ) );
	dq.v.y = b3AddN( b3MulNN( hy, q.s ), b3SubN( b3MulNN( hz, q.v.x ), b3MulNN( hx, q.v.z ) ) );
	dq.v.z = b3AddN( b3MulNN( hz, q.s ), b3SubN( b3MulNN( hx, q.v.y ), b3MulNN( hy, q.v.x ) ) );
	dq.s = b3NegN( b3AddN( b3AddN( b3MulNN( hx, q.v.x ), b3MulNN( hy, q.v.y ) ), b3MulNN( hz, q.v.z ) ) );

	b3Quat r;
	r.v.x = b3AddN( q.v.x, dq.v.x );
	r.v.y = b3AddN( q.v.y, dq.v.y );
	r.v.z = b3AddN( q.v.z, dq.v.z );
	r.s = b3AddN( q.s, dq.s );

	return b3NormalizeQuat( r );
}

b3Vec3 b3ArbitraryPerp( b3Vec3 v )
{
	// Upstream's coefficients, at Q30. Any pair with a != -b gives a vector
	// perpendicular to v; this particular pair is what keeps |p| between 0.39
	// and 0.79 for every unit input, which is what matters here -- the result
	// is normalized immediately, and b3RsqrtWide runs out of resolution below
	// about 1/128 of a unit.
	//
	//   0.67 * 2^30 =  719407022
	//  -0.42 * 2^30 = -450971566
	const int64_t kA = 719407022;
	const int64_t kB = -450971566;

	// A unit vector always has a component of at least 1/sqrt(3) = 0.577, so
	// exactly one of these three branches is reachable for a valid input.
	const int32_t kHalf = B3_F_ONE / 2;

	int64_t x = (int64_t)b3Raw( v.x );
	int64_t y = (int64_t)b3Raw( v.y );
	int64_t z = (int64_t)b3Raw( v.z );

	int64_t px, py, pz;

	if ( x < -kHalf || kHalf < x )
	{
		// dot([a*y + b*z, -a*x, -b*x], v) cancels term by term.
		px = kA * y + kB * z;
		py = -kA * x;
		pz = -kB * x;
	}
	else if ( y < -kHalf || kHalf < y )
	{
		px = kA * y;
		py = -kA * x + kB * z;
		pz = -kB * y;
	}
	else if ( z < -kHalf || kHalf < z )
	{
		px = kA * z;
		py = kB * z;
		pz = -kA * x - kB * y;
	}
	else
	{
		// Not a unit vector. Upstream asserts this away; the port answers it,
		// because a Q12 "unit" vector that quantized badly must not fall
		// through into a branch whose derivation assumed |v.z| > 0.5.
		//
		// b3Perp normalizes a cross product, so for an input that is itself
		// zero it hands back a zero vector -- and every caller here is about to
		// use the result as a direction. Answering +X keeps the contract that
		// the return value is always unit length.
		b3Vec3 fallback = b3Perp( v );
		if ( b3LengthSquaredWide( fallback ) == 0 )
		{
			return b3Vec3_axisXFn();
		}

		return fallback;
	}

	// The components are at Q42 -- Q12 vector times Q30 coefficient. Narrowing
	// them to Q12 first would leave a vector of about 2000 raw units carrying
	// a direction that is about to be normalized, for no reason:
	// b3DirectionFromWide is a count-leading-zeros and three shifts, and
	// scaling a vector changes neither its direction nor its normalization.
	//
	// Overflow: |kA| < 2^30 and a unit vector's raw components are at most
	// 4096, so the worst term is 2 * 2^30 * 4096 = 8.8e12. Even at B3_HUGE it
	// is 1.8e16, well inside int64.
	b3Vec3 p = b3DirectionFromWide( px, py, pz );
	if ( b3LengthSquaredWide( p ) == 0 )
	{
		return b3Vec3_axisXFn();
	}

	return b3Normalize( p );
}
