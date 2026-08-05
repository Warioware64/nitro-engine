// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Erin Catto        (original Box3D)
// Copyright (c) 2026 Warioware64       (Nitro Engine Advanced fixed-point port)
//
// This file is part of Nitro Engine Advanced

#include "box3d/b3fixed.h"

// =========================================================================
// Arc tangent
// =========================================================================

// libnds ships sin/cos/asin/acos lookup tables but no atan2, so this is the
// one trig function that still has to be evaluated rather than looked up.
//
// The shape is upstream's minimax polynomial for atan on [0, 1], carried over
// because it is accurate to about 1e-5 in float and needs only multiplies
// after the initial ratio. Evaluating it in Q30 keeps roughly 1e-6, well
// under the brad resolution the result is quantized to anyway.
//
// Joint angle readout is the only caller, so the single divide here is not on
// a hot path.
//
// Note what the body below never does: it never uses the magnitude of either
// argument, only their ratio (as min/max, taken at Q30) and their signs. That
// is why the entry point takes raw int32 and b3Atan2F is a thin Q12 wrapper --
// a caller holding Q30 roots can pass them unnarrowed and keep the bits. See
// b3Atan2Raw's declaration.

// Coefficients of the minimax fit, in Q30.
#define B3_ATAN_C0 26670445  // 0.024840285
#define B3_ATAN_C1 200557900 // 0.18681418
#define B3_ATAN_C2 -101043000 // -0.094097948
#define B3_ATAN_C3 -356654600 // -0.33213072

b3a b3Atan2Raw( int32_t ry, int32_t rx )
{
	// Upstream adds this check to match atan2f and avoid a NaN; here it
	// avoids a divide by zero.
	if ( rx == 0 && ry == 0 )
	{
		return 0;
	}

	int32_t ax = rx < 0 ? -rx : rx;
	int32_t ay = ry < 0 ? -ry : ry;
	int32_t mx = ay > ax ? ay : ax;
	int32_t mn = ay > ax ? ax : ay;

	// a = mn / mx, in [0, 1]. Q30 so the polynomial has room to work.
	int32_t a = b3HwDiv64( (int64_t)mn << B3_C_SHIFT, mx );

	// r = ((c0*q + c1)*s + (c2*q + c3))*c + a  with s = a^2, c = a^3, q = s^2
	int32_t s = (int32_t)( ( (int64_t)a * a ) >> B3_C_SHIFT );
	int32_t c = (int32_t)( ( (int64_t)s * a ) >> B3_C_SHIFT );
	int32_t q = (int32_t)( ( (int64_t)s * s ) >> B3_C_SHIFT );

	int32_t r = (int32_t)( ( (int64_t)B3_ATAN_C0 * q ) >> B3_C_SHIFT ) + B3_ATAN_C1;
	int32_t t = (int32_t)( ( (int64_t)B3_ATAN_C2 * q ) >> B3_C_SHIFT ) + B3_ATAN_C3;
	r = (int32_t)( ( (int64_t)r * s ) >> B3_C_SHIFT ) + t;
	r = (int32_t)( ( (int64_t)r * c ) >> B3_C_SHIFT ) + a;

	// r is now an angle in radians, Q30. Convert to brads: the circle is
	// 32768 brads, so brad = rad * 32768 / (2*pi) = rad * 5215.19.
	// 5215.189175 in Q12 is 21361555 >> 12; fold the Q30 shift in directly.
	// brad = (r * 32768) / (2*pi * 2^30)
	//      = r * 5215.189 / 2^30
	const int64_t radToBrad = 21361559; // 5215.189175 * 2^12

	// Quarter turn, used to fold the |y| > |x| case.
	const int32_t quarter = B3_BRAD_CIRCLE / 4;

	int32_t brad = (int32_t)( ( ( (int64_t)r * radToBrad ) >> B3_C_SHIFT ) >> B3_F_SHIFT );

	if ( ay > ax )
	{
		brad = quarter - brad;
	}

	if ( rx < 0 )
	{
		brad = ( B3_BRAD_CIRCLE / 2 ) - brad;
	}

	if ( ry < 0 )
	{
		brad = -brad;
	}

	return (b3a)brad;
}

// =========================================================================
// Wide division
// =========================================================================
//
// Out of line rather than inline in the header. It is called a handful of
// times per narrow-phase query, never in an inner loop, and the CLZ
// normalization plus saturation branch is large enough that duplicating it
// into every translation unit cost 6.4 KB across the port -- more than the
// entire support layer.

b3c b3DivWideToC( int64_t num, int64_t den )
{
	if ( den == 0 )
	{
		return b3c_zero;
	}

	uint64_t an = (uint64_t)( num < 0 ? -num : num );
	uint64_t ad = (uint64_t)( den < 0 ? -den : den );

	// |quotient| >= 2 does not fit Q30. Saturate with the correct sign.
	if ( an >= ( ad << 1 ) )
	{
		bool negative = ( num < 0 ) != ( den < 0 );
		return b3Makeb3cRef( negative ? INT32_MIN : INT32_MAX, B3_REF( negative ? -2.0 : 2.0 ) );
	}

	// The divider takes a 64-bit numerator and a 32-bit denominator, and
	// returns 32 bits. So the denominator must fit 31 bits (plus sign), and
	// the pre-shifted numerator must fit 63.
	int shift = 0;

	int denBits = 64 - b3Clz64( ad );
	if ( denBits > 31 )
	{
		shift = denBits - 31;
	}

	// num << B3_C_SHIFT must not overflow either.
	int numBits = 64 - b3Clz64( an );
	int numRoom = numBits + B3_C_SHIFT - 63;
	if ( numRoom > shift )
	{
		shift = numRoom;
	}

	if ( shift > 0 )
	{
		num >>= shift;
		den >>= shift;

		// Shifting can only reach zero here when the denominator was
		// vanishingly small relative to the numerator, which means the
		// quotient was not bounded after all.
		if ( den == 0 )
		{
			return b3c_zero;
		}
	}

	return b3Makeb3cRef( b3HwDiv64( num << B3_C_SHIFT, (int32_t)den ),
						 B3_REF( den != 0 ? (double)num / (double)den : 0.0 ) );
}

b3f b3DivWideToF( int64_t num, int64_t den )
{
	if ( den == 0 )
	{
		return b3f_zero;
	}

	uint64_t an = (uint64_t)( num < 0 ? -num : num );
	uint64_t ad = (uint64_t)( den < 0 ? -den : den );

	bool negative = ( num < 0 ) != ( den < 0 );

	int numBits = 64 - b3Clz64( an );
	int denBits = 64 - b3Clz64( ad );

	// Saturate rather than wrap. B3_F_MAX is INT32_MAX/2 -- half, so that an
	// accidental add on the sentinel does not wrap to a large negative -- which
	// at Q12 is a quotient of 2^18.
	//
	// The bit-length difference bounds the quotient to within one bit, so it is
	// used only to decide whether the exact test is worth doing. Inside the
	// branch denBits <= 45, so `ad << 18` cannot overflow.
	if ( numBits - denBits >= 18 && an >= ( ad << 18 ) )
	{
		return negative ? B3_F_MIN : B3_F_MAX;
	}

	// The divider takes a 64-bit numerator and a 32-bit denominator and returns
	// 32 bits, so the denominator must fit 31 bits plus sign and the numerator
	// must still fit 63 after being shifted up by B3_F_SHIFT.
	int shift = 0;

	if ( denBits > 31 )
	{
		shift = denBits - 31;
	}

	int numRoom = numBits + B3_F_SHIFT - 63;
	if ( numRoom > shift )
	{
		shift = numRoom;
	}

	if ( shift > 0 )
	{
		num >>= shift;
		den >>= shift;

		if ( den == 0 )
		{
			return negative ? B3_F_MIN : B3_F_MAX;
		}
	}

	return b3Makeb3fRef( b3HwDiv64( num << B3_F_SHIFT, (int32_t)den ),
						 B3_REF( den != 0 ? (double)num / (double)den : 0.0 ) );
}

int32_t b3RcpWideSlow( uint64_t mag )
{
	// Exactly floor( 2^36 / mag ), by restoring division, for mag > INT32_MAX.
	//
	// Six iterations suffice and that is a property of the caller, not a
	// tolerance: b3RcpWide only reaches here when the operand leaves int32, so
	// mag >= 2^31, and the numerator is 2^36 -- giving a quotient of at most
	// 2^36 / 2^31 = 32. Bit 5 is therefore the highest that can ever be set,
	// and mag == 2^31 exactly is the case that sets it.
	//
	// The test is written `rem >> bit >= mag` rather than `rem >= mag << bit`
	// because mag can be most of the way to 2^63 and the shifted form would
	// overflow. The two are equivalent for non-negative rem and positive mag,
	// since mag is an integer: floor( rem / 2^bit ) >= mag iff rem >= mag*2^bit.
	// Inside the branch the shift is then known to be safe, because mag << bit
	// is bounded by rem, which never exceeds 2^36.
	uint64_t rem = (uint64_t)1 << ( B3_F_SHIFT + B3_W_SHIFT );
	int32_t q = 0;

	for ( int bit = 5; bit >= 0; --bit )
	{
		if ( ( rem >> bit ) >= mag )
		{
			rem -= mag << bit;
			q |= 1 << bit;
		}
	}

	return q;
}

// =========================================================================
// Host divide staging
// =========================================================================

#ifndef __NDS__
// The asynchronous divide exists so the DS can overlap the divider's ~36
// cycle latency with unrelated work. There is nothing to overlap on the host,
// so the value is simply held between the start and collect calls.
b3f b3_hostPendingDiv;
#endif

// =========================================================================
// Shadow-value verification
// =========================================================================

#if defined( B3_FIXED_DEBUG )

// Only the verification path reports and aborts; a device build pulls in
// neither of these.
#include <stdio.h>
#include <stdlib.h>

b3FixedStats b3_fixedStats;

void b3FixedResetStats( void )
{
	b3_fixedStats.maxRelError = 0.0;
	b3_fixedStats.worstOp = "none";
	b3_fixedStats.worstFile = "none";
	b3_fixedStats.worstLine = 0;
	b3_fixedStats.overflowCount = 0;
	b3_fixedStats.scaleErrorCount = 0;
	b3_fixedStats.opCount = 0;
}

void b3FixedCheck( int64_t wide, int32_t narrow, double ref, int shift, const char* op, const char* file, int line )
{
	b3_fixedStats.opCount++;

	// Overflow is categorically different from imprecision: it means a value
	// left the range its scale can represent, and every result downstream is
	// meaningless. Report it loudly and immediately.
	if ( wide != (int64_t)narrow )
	{
		b3_fixedStats.overflowCount++;
		fprintf( stderr, "b3fixed OVERFLOW in %s at %s:%d -- wide %lld does not fit int32 (ref %.9g)\n", op, file, line,
				 (long long)wide, ref );
		abort();
	}

	double actual = (double)narrow / (double)( (int64_t)1 << shift );
	double diff = actual - ref;
	if ( diff < 0.0 )
	{
		diff = -diff;
	}

	// Compare relative to the larger of the two magnitudes, with an absolute
	// floor of one quantum so that values near zero -- where the relative
	// error is unbounded but harmless -- do not dominate the statistics.
	double mag = ref < 0.0 ? -ref : ref;
	double quantum = 1.0 / (double)( (int64_t)1 << shift );
	double denom = mag > quantum ? mag : quantum;
	double rel = diff / denom;

	if ( rel > b3_fixedStats.maxRelError )
	{
		b3_fixedStats.maxRelError = rel;
		b3_fixedStats.worstOp = op;
		b3_fixedStats.worstFile = file;
		b3_fixedStats.worstLine = line;
	}

	// Distinguishing a scale bug from ordinary quantization needs both tests.
	//
	// Relative error alone is not enough: a result a few quanta from zero has
	// a huge relative error that is entirely harmless, and flagging those
	// buries the real findings in noise.
	//
	// Absolute error alone is not enough either: a large value can be badly
	// wrong in absolute terms while still being correct to its scale.
	//
	// A genuine scale mismatch is wrong by a factor of 2^18 or so, which
	// clears both bars by orders of magnitude.
	if ( rel > B3_FIXED_SCALE_TOL && diff > 8.0 * quantum )
	{
		b3_fixedStats.scaleErrorCount++;
		fprintf( stderr, "b3fixed SCALE ERROR in %s at %s:%d -- got %.9g, expected %.9g (rel %.4g, %.1f quanta)\n", op, file,
				 line, actual, ref, rel, diff / quantum );
	}
}

void b3FixedReport( const char* label )
{
	printf( "[b3fixed] %s: %lld ops, max rel error %.4g at %s (%s:%d)", label, b3_fixedStats.opCount,
			b3_fixedStats.maxRelError, b3_fixedStats.worstOp, b3_fixedStats.worstFile, b3_fixedStats.worstLine );

	if ( b3_fixedStats.overflowCount > 0 || b3_fixedStats.scaleErrorCount > 0 )
	{
		printf( "  [%d overflows, %d scale errors]", b3_fixedStats.overflowCount, b3_fixedStats.scaleErrorCount );
	}

	printf( "\n" );
}

#endif // B3_FIXED_DEBUG
