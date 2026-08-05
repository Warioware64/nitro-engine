// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Erin Catto        (original Box3D)
// Copyright (c) 2026 Warioware64       (Nitro Engine Advanced fixed-point port)
//
// This file is part of Nitro Engine Advanced

#ifndef B3_FIXED_H__
#define B3_FIXED_H__

/// @file   b3fixed.h
/// @brief  Fixed-point scalar layer for the Box3D port.
///
/// Upstream Box3D is float throughout. The DS has no FPU, so every scalar in
/// the simulation becomes an int32_t in one of several fixed-point formats.
/// A single uniform format does not work: the same Q12 that suits a world
/// position is hopeless for an inverse inertia tensor, and a substep of
/// 1/240 s rounds to 17 in Q12 -- a 0.4% error compounded every substep.
///
/// So each quantity class gets its own scale:
///
///   type   format    holds
///   ----   ------    -----------------------------------------------
///   b3f    Q19.12    length, position, velocity, mass, inertia
///   b3n    Q1.30     unit vectors, normals, quaternion components
///   b3iw   Q7.24     inverse mass, inverse inertia
///   b3imp  Q15.16    impulses and warm-started accumulators
///   b3t    Q7.24     time (dt and the substep h)
///   b3c    Q1.30     dimensionless coefficients in [-1, 1]
///   b3a    brad      angles, 32768 per circle (libnds convention)
///
/// b3f is deliberately Q19.12 so that it *is* libnds f32: positions feed
/// NEA_ModelSetMatrix and the hardware divide/sqrt registers with no shifting.
///
/// Every cross-scale multiply goes through an int64 intermediate. That is
/// cheap here -- ARM9E's SMULL is a single 32x32->64 instruction -- and it is
/// what makes the mixed-scale scheme affordable.
///
/// @section strict Catching scale bugs
///
/// Mixing scales is silent: pass a Q12 where Q30 is expected and you get a
/// number 262144x too small, with no diagnostic. Two build modes exist to
/// stop that:
///
///   B3_FIXED_STRICT  each scale becomes a distinct struct type, so the
///                    compiler rejects cross-scale assignment outright.
///
///   B3_FIXED_DEBUG   implies STRICT, and additionally carries the exact
///                    float value alongside every fixed value. Each operation
///                    recomputes in float and compares. Overflow and gross
///                    divergence (a scale bug) abort; ordinary quantization
///                    is accumulated into a report. This is how the host
///                    harness localizes a precision failure to one operation
///                    instead of one frame.
///
/// Device builds define neither, so every type is a bare int32_t and every
/// operation is a shift -- no wrapper, no overhead.

#include <stdint.h>
#include <stdbool.h>

#ifdef __NDS__
#include <nds.h>
#else
#include <math.h>
#endif

#if defined( B3_FIXED_DEBUG ) && !defined( B3_FIXED_STRICT )
#define B3_FIXED_STRICT 1
#endif

// =========================================================================
// Scales
// =========================================================================

#define B3_F_SHIFT 12   ///< length, velocity, mass
#define B3_N_SHIFT 30   ///< unit vectors, quaternions
#define B3_W_SHIFT 24   ///< inverse mass, inverse inertia
#define B3_IMP_SHIFT 16 ///< impulses
#define B3_T_SHIFT 24   ///< time
#define B3_C_SHIFT 30   ///< dimensionless coefficients

#define B3_F_ONE ( (int32_t)1 << B3_F_SHIFT )
#define B3_N_ONE ( (int32_t)1 << B3_N_SHIFT )
#define B3_W_ONE ( (int32_t)1 << B3_W_SHIFT )
#define B3_IMP_ONE ( (int32_t)1 << B3_IMP_SHIFT )
#define B3_T_ONE ( (int32_t)1 << B3_T_SHIFT )
#define B3_C_ONE ( (int32_t)1 << B3_C_SHIFT )

/// Angles use the libnds convention: 32768 brad per full circle. This is what
/// sinLerp/cosLerp index, so no conversion is needed at the LUT.
#define B3_BRAD_CIRCLE 32768
#define B3_BRAD_PI 16384
#define B3_BRAD_HALF_PI 8192

// =========================================================================
// Type definitions
// =========================================================================

#if defined( B3_FIXED_STRICT )

#if defined( B3_FIXED_DEBUG )
#define B3_FIX_TYPE( NAME )                                                                                                      \
	typedef struct NAME                                                                                                          \
	{                                                                                                                            \
		int32_t v;                                                                                                               \
		double ref;                                                                                                              \
	} NAME
#else
#define B3_FIX_TYPE( NAME )                                                                                                      \
	typedef struct NAME                                                                                                          \
	{                                                                                                                            \
		int32_t v;                                                                                                               \
	} NAME
#endif

B3_FIX_TYPE( b3f );
B3_FIX_TYPE( b3n );
B3_FIX_TYPE( b3iw );
B3_FIX_TYPE( b3imp );
B3_FIX_TYPE( b3t );
B3_FIX_TYPE( b3c );

#undef B3_FIX_TYPE

/// Extract the raw integer of a fixed value. Use only at hardware and
/// serialization boundaries -- inside the solver, go through the operations.
#define b3Raw( x ) ( ( x ).v )

#else // !B3_FIXED_STRICT

typedef int32_t b3f;
typedef int32_t b3n;
typedef int32_t b3iw;
typedef int32_t b3imp;
typedef int32_t b3t;
typedef int32_t b3c;

#define b3Raw( x ) ( x )

#endif // B3_FIXED_STRICT

/// Angles are a plain int16 in both modes; there is only one angle scale, so
/// there is nothing to confuse it with.
typedef int16_t b3a;

// =========================================================================
// Debug instrumentation
// =========================================================================

#if defined( B3_FIXED_DEBUG )

/// Accumulated statistics for a host validation run.
typedef struct b3FixedStats
{
	/// Largest relative error between a fixed result and its exact float
	/// counterpart, across every operation since the last reset.
	double maxRelError;

	/// Operation and source location that produced maxRelError.
	const char* worstOp;
	const char* worstFile;
	int worstLine;

	/// Number of results that saturated the int32 range. Any non-zero value
	/// here is a bug, not a precision limit.
	int overflowCount;

	/// Number of results whose relative error exceeded B3_FIXED_SCALE_TOL.
	/// Almost always a mismatched scale rather than quantization.
	int scaleErrorCount;

	long long opCount;
} b3FixedStats;

extern b3FixedStats b3_fixedStats;

/// A relative error above this is treated as a scale bug rather than
/// quantization. The coarsest scale in use is Q12, whose relative error on a
/// value of order 1 is about 2.4e-4; 1% leaves two orders of magnitude of
/// headroom, so anything tripping it is structurally wrong.
#define B3_FIXED_SCALE_TOL 0.01

void b3FixedReport( const char* label );
void b3FixedResetStats( void );

/// Records one operation. Aborts on overflow or on a divergence large enough
/// to indicate a scale mismatch; otherwise folds the error into the stats.
void b3FixedCheck( int64_t wide, int32_t narrow, double ref, int shift, const char* op, const char* file, int line );

#define B3_FIX_CHECK( wide, narrow, ref, shift, op ) b3FixedCheck( wide, narrow, ref, shift, op, __FILE__, __LINE__ )

/// Guards a shadow-value expression. In a device build the expression is not
/// merely discarded, it is never written -- which matters because several of
/// them call sqrt(). Evaluating those would pull soft-float into the ROM to
/// compute a number nothing reads.
#define B3_REF( expr ) ( expr )

#else

#define B3_FIX_CHECK( wide, narrow, ref, shift, op ) ( (void)0 )
#define B3_REF( expr ) 0.0

#endif // B3_FIXED_DEBUG

// =========================================================================
// Constructors
// =========================================================================
//
// b3MakeX() builds a fixed value from an already-scaled raw integer.
// b3XFromInt()/b3XFromFrac() build one from a real quantity.
//
// In debug mode the constructors seed the shadow value, which is why they
// exist at all rather than being plain casts.

#if defined( B3_FIXED_STRICT )

#if defined( B3_FIXED_DEBUG )
#define B3_FIX_MAKE( TYPE, SHIFT )                                                                                               \
	static inline TYPE b3Make##TYPE( int32_t raw )                                                                               \
	{                                                                                                                            \
		TYPE r;                                                                                                                  \
		r.v = raw;                                                                                                               \
		r.ref = (double)raw / (double)( (int64_t)1 << ( SHIFT ) );                                                               \
		return r;                                                                                                                \
	}                                                                                                                            \
	static inline TYPE b3Make##TYPE##Ref( int32_t raw, double ref )                                                              \
	{                                                                                                                            \
		TYPE r;                                                                                                                  \
		r.v = raw;                                                                                                               \
		r.ref = ref;                                                                                                             \
		return r;                                                                                                                \
	}
#else
#define B3_FIX_MAKE( TYPE, SHIFT )                                                                                               \
	static inline TYPE b3Make##TYPE( int32_t raw )                                                                               \
	{                                                                                                                            \
		TYPE r;                                                                                                                  \
		r.v = raw;                                                                                                               \
		return r;                                                                                                                \
	}                                                                                                                            \
	static inline TYPE b3Make##TYPE##Ref( int32_t raw, double ref )                                                              \
	{                                                                                                                            \
		(void)ref;                                                                                                               \
		TYPE r;                                                                                                                  \
		r.v = raw;                                                                                                               \
		return r;                                                                                                                \
	}
#endif

#else // !B3_FIXED_STRICT

#define B3_FIX_MAKE( TYPE, SHIFT )                                                                                               \
	static inline TYPE b3Make##TYPE( int32_t raw )                                                                               \
	{                                                                                                                            \
		return raw;                                                                                                              \
	}                                                                                                                            \
	static inline TYPE b3Make##TYPE##Ref( int32_t raw, double ref )                                                              \
	{                                                                                                                            \
		(void)ref;                                                                                                               \
		return raw;                                                                                                              \
	}

#endif

B3_FIX_MAKE( b3f, B3_F_SHIFT )
B3_FIX_MAKE( b3n, B3_N_SHIFT )
B3_FIX_MAKE( b3iw, B3_W_SHIFT )
B3_FIX_MAKE( b3imp, B3_IMP_SHIFT )
B3_FIX_MAKE( b3t, B3_T_SHIFT )
B3_FIX_MAKE( b3c, B3_C_SHIFT )

#undef B3_FIX_MAKE

/// Zero of each scale. Needed because in strict mode a fixed value is a
/// struct, so `= 0` does not compile.
#define b3f_zero b3Makeb3f( 0 )
#define b3n_zero b3Makeb3n( 0 )
#define b3iw_zero b3Makeb3iw( 0 )
#define b3imp_zero b3Makeb3imp( 0 )
#define b3t_zero b3Makeb3t( 0 )
#define b3c_zero b3Makeb3c( 0 )

#define b3f_one b3Makeb3f( B3_F_ONE )
#define b3n_one b3Makeb3n( B3_N_ONE )
#define b3c_one b3Makeb3c( B3_C_ONE )

/// Largest representable b3f, used where upstream writes FLT_MAX to seed a
/// minimum search. Deliberately not INT32_MAX: leaving headroom means an
/// accidental add on the sentinel does not wrap to a large negative.
#define B3_F_MAX b3Makeb3f( INT32_MAX / 2 )
#define B3_F_MIN b3Makeb3f( -( INT32_MAX / 2 ) )

/// Construct from an integer count of units.
static inline b3f b3fFromInt( int32_t n )
{
	return b3Makeb3fRef( n << B3_F_SHIFT, B3_REF( (double)n ) );
}

/// Construct from a rational num/den. This is the constant-friendly path:
/// b3fFromFrac(1, 60) is exact where a float literal would not be.
static inline b3f b3fFromFrac( int32_t num, int32_t den )
{
	return b3Makeb3fRef( (int32_t)( ( (int64_t)num << B3_F_SHIFT ) / den ), B3_REF( (double)num / (double)den ) );
}

static inline b3t b3tFromFrac( int32_t num, int32_t den )
{
	return b3Makeb3tRef( (int32_t)( ( (int64_t)num << B3_T_SHIFT ) / den ), B3_REF( (double)num / (double)den ) );
}

static inline b3c b3cFromFrac( int32_t num, int32_t den )
{
	return b3Makeb3cRef( (int32_t)( ( (int64_t)num << B3_C_SHIFT ) / den ), B3_REF( (double)num / (double)den ) );
}

static inline b3n b3nFromFrac( int32_t num, int32_t den )
{
	return b3Makeb3nRef( (int32_t)( ( (int64_t)num << B3_N_SHIFT ) / den ), B3_REF( (double)num / (double)den ) );
}

/// Round toward zero to a whole number of units.
static inline int32_t b3fToInt( b3f a )
{
	return b3Raw( a ) >> B3_F_SHIFT;
}

// -------------------------------------------------------------------------
// Host-only float bridges
// -------------------------------------------------------------------------
// Only for the host harness, test scenarios and the NEA float-convenience
// macros. Never used inside the solver -- a device build has no FPU and these
// would silently pull in soft-float.

static inline b3f b3fFromDouble( double x )
{
	return b3Makeb3fRef( (int32_t)( x * (double)B3_F_ONE ), x );
}

static inline double b3fToDouble( b3f a )
{
	return (double)b3Raw( a ) / (double)B3_F_ONE;
}

static inline b3n b3nFromDouble( double x )
{
	return b3Makeb3nRef( (int32_t)( x * (double)B3_N_ONE ), x );
}

static inline double b3nToDouble( b3n a )
{
	return (double)b3Raw( a ) / (double)B3_N_ONE;
}

static inline b3iw b3iwFromDouble( double x )
{
	return b3Makeb3iwRef( (int32_t)( x * (double)B3_W_ONE ), x );
}

static inline double b3iwToDouble( b3iw a )
{
	return (double)b3Raw( a ) / (double)B3_W_ONE;
}

/// Completes the set. Its absence was not deliberate -- nothing needed to
/// build an impulse from a literal until Phase 3C-ii's tests did, and going
/// through b3FToImp instead quietly costs the four bits that distinguish Q16
/// from Q12, which is the entire reason the scale exists.
static inline b3imp b3impFromDouble( double x )
{
	return b3Makeb3impRef( (int32_t)( x * (double)B3_IMP_ONE ), x );
}

static inline double b3impToDouble( b3imp a )
{
	return (double)b3Raw( a ) / (double)B3_IMP_ONE;
}

static inline double b3tToDouble( b3t a )
{
	return (double)b3Raw( a ) / (double)B3_T_ONE;
}

static inline double b3cToDouble( b3c a )
{
	return (double)b3Raw( a ) / (double)B3_C_ONE;
}

// =========================================================================
// Same-scale arithmetic
// =========================================================================
//
// Addition and subtraction never change scale, so these are generated per
// type. Only the debug build does anything beyond the integer op.

#if defined( B3_FIXED_DEBUG )
#define B3_FIX_REF2( a, b, expr ) ( ( a ).ref expr( b ).ref )
#define B3_FIX_ARITH_BODY( TYPE, SHIFT, OP, NAME, a, b )                                                                         \
	int64_t wide = (int64_t)( a ).v OP( b ).v;                                                                                   \
	double ref = ( a ).ref OP( b ).ref;                                                                                          \
	B3_FIX_CHECK( wide, (int32_t)wide, ref, SHIFT, NAME );                                                                       \
	return b3Make##TYPE##Ref( (int32_t)wide, ref );
#else
#define B3_FIX_ARITH_BODY( TYPE, SHIFT, OP, NAME, a, b ) return b3Make##TYPE( b3Raw( a ) OP b3Raw( b ) );
#endif

#define B3_FIX_ARITH( TYPE, SHIFT, SUFFIX )                                                                                      \
	static inline TYPE b3Add##SUFFIX( TYPE a, TYPE b )                                                                           \
	{                                                                                                                            \
		B3_FIX_ARITH_BODY( TYPE, SHIFT, +, "add" #SUFFIX, a, b )                                                                 \
	}                                                                                                                            \
	static inline TYPE b3Sub##SUFFIX( TYPE a, TYPE b )                                                                           \
	{                                                                                                                            \
		B3_FIX_ARITH_BODY( TYPE, SHIFT, -, "sub" #SUFFIX, a, b )                                                                 \
	}                                                                                                                            \
	static inline TYPE b3Neg##SUFFIX( TYPE a )                                                                                   \
	{                                                                                                                            \
		return b3Make##TYPE##Ref( -b3Raw( a ), B3_REF( -b3Ref##SUFFIX( a ) ) );                                                            \
	}                                                                                                                            \
	static inline TYPE b3Abs##SUFFIX( TYPE a )                                                                                   \
	{                                                                                                                            \
		return b3Raw( a ) < 0 ? b3Neg##SUFFIX( a ) : a;                                                                          \
	}                                                                                                                            \
	static inline TYPE b3Min##SUFFIX( TYPE a, TYPE b )                                                                           \
	{                                                                                                                            \
		return b3Raw( a ) < b3Raw( b ) ? a : b;                                                                                  \
	}                                                                                                                            \
	static inline TYPE b3Max##SUFFIX( TYPE a, TYPE b )                                                                           \
	{                                                                                                                            \
		return b3Raw( a ) > b3Raw( b ) ? a : b;                                                                                  \
	}                                                                                                                            \
	static inline TYPE b3Clamp##SUFFIX( TYPE a, TYPE lo, TYPE hi )                                                               \
	{                                                                                                                            \
		return b3Raw( a ) < b3Raw( lo ) ? lo : ( b3Raw( hi ) < b3Raw( a ) ? hi : a );                                            \
	}                                                                                                                            \
	static inline bool b3Lt##SUFFIX( TYPE a, TYPE b )                                                                            \
	{                                                                                                                            \
		return b3Raw( a ) < b3Raw( b );                                                                                          \
	}                                                                                                                            \
	static inline bool b3Le##SUFFIX( TYPE a, TYPE b )                                                                            \
	{                                                                                                                            \
		return b3Raw( a ) <= b3Raw( b );                                                                                         \
	}                                                                                                                            \
	static inline bool b3Gt##SUFFIX( TYPE a, TYPE b )                                                                            \
	{                                                                                                                            \
		return b3Raw( a ) > b3Raw( b );                                                                                          \
	}                                                                                                                            \
	static inline bool b3Eqz##SUFFIX( TYPE a )                                                                                   \
	{                                                                                                                            \
		return b3Raw( a ) == 0;                                                                                                  \
	}

// Shadow-value accessor. Folds to a no-op outside the debug build so that
// b3Neg's ref argument costs nothing.
#if defined( B3_FIXED_DEBUG )
#define B3_FIX_REFACC( TYPE, SUFFIX )                                                                                            \
	static inline double b3Ref##SUFFIX( TYPE a )                                                                                 \
	{                                                                                                                            \
		return a.ref;                                                                                                            \
	}
#else
#define B3_FIX_REFACC( TYPE, SUFFIX )                                                                                            \
	static inline double b3Ref##SUFFIX( TYPE a )                                                                                 \
	{                                                                                                                            \
		(void)a;                                                                                                                 \
		return 0.0;                                                                                                              \
	}
#endif

B3_FIX_REFACC( b3f, F )
B3_FIX_REFACC( b3n, N )
B3_FIX_REFACC( b3iw, W )
B3_FIX_REFACC( b3imp, Imp )
B3_FIX_REFACC( b3t, T )
B3_FIX_REFACC( b3c, C )

B3_FIX_ARITH( b3f, B3_F_SHIFT, F )
B3_FIX_ARITH( b3n, B3_N_SHIFT, N )
B3_FIX_ARITH( b3iw, B3_W_SHIFT, W )
B3_FIX_ARITH( b3imp, B3_IMP_SHIFT, Imp )
B3_FIX_ARITH( b3t, B3_T_SHIFT, T )
B3_FIX_ARITH( b3c, B3_C_SHIFT, C )

#undef B3_FIX_ARITH
#undef B3_FIX_ARITH_BODY
#undef B3_FIX_REFACC

// =========================================================================
// Cross-scale multiply
// =========================================================================
//
// Naming is b3Mul<A><B> where the letters are the operand scales, and the
// result scale is stated in the comment. The set below is deliberately
// closed: only combinations the solver actually needs exist, so a
// nonsensical product simply fails to compile.
//
// Every one is a 32x32->64 multiply (SMULL, ~2 cycles) plus a 64-bit shift.

/// Rounds a 64-bit product to nearest when narrowing, rather than letting the
/// arithmetic shift floor it.
///
/// This matters more than it looks. A plain `>>` rounds toward negative
/// infinity, so a repeated operation loses up to one quantum *every time, in
/// the same direction*. Integrating gravity for one second at 240 Hz that way
/// accumulates a 0.45% velocity error -- and it keeps growing, because the
/// bias never cancels. Adding half a quantum first makes the error zero-mean,
/// so it cancels instead of compounding.
///
/// The cost is one add before a shift that was happening anyway.
#define B3_MUL_ROUND( wide, shift ) ( ( ( wide ) + ( (int64_t)1 << ( ( shift ) - 1 ) ) ) >> ( shift ) )

/// Same rounding for a narrowing scale conversion.
#define B3_SHIFT_ROUND( v, shift ) ( (int32_t)( ( (int64_t)( v ) + ( (int64_t)1 << ( ( shift ) - 1 ) ) ) >> ( shift ) ) )

#if defined( B3_FIXED_DEBUG )
#define B3_FIX_MUL_BODY( RTYPE, RSHIFT, SHIFT, NAME, a, b, aref, bref )                                                          \
	int64_t wide = (int64_t)( a ).v * (int64_t)( b ).v;                                                                          \
	int64_t scaled = B3_MUL_ROUND( wide, SHIFT );                                                                                \
	double ref = ( aref ) * ( bref );                                                                                            \
	B3_FIX_CHECK( scaled, (int32_t)scaled, ref, RSHIFT, NAME );                                                                  \
	return b3Make##RTYPE##Ref( (int32_t)scaled, ref );
#else
#define B3_FIX_MUL_BODY( RTYPE, RSHIFT, SHIFT, NAME, a, b, aref, bref )                                                          \
	return b3Make##RTYPE( (int32_t)B3_MUL_ROUND( (int64_t)b3Raw( a ) * (int64_t)b3Raw( b ), SHIFT ) );
#endif

/// Q12 * Q12 -> Q12. Length by length, velocity by scalar.
static inline b3f b3MulFF( b3f a, b3f b )
{
	B3_FIX_MUL_BODY( b3f, B3_F_SHIFT, B3_F_SHIFT, "mulFF", a, b, b3RefF( a ), b3RefF( b ) )
}

/// Q12 * Q30 -> Q12. Length projected onto a unit direction.
static inline b3f b3MulFN( b3f a, b3n b )
{
	B3_FIX_MUL_BODY( b3f, B3_F_SHIFT, B3_N_SHIFT, "mulFN", a, b, b3RefF( a ), b3RefN( b ) )
}

/// Q30 * Q30 -> Q30. Component of a unit-vector dot product, quaternion
/// products. Stays in Q30 so quaternion multiplication does not shed bits.
static inline b3n b3MulNN( b3n a, b3n b )
{
	B3_FIX_MUL_BODY( b3n, B3_N_SHIFT, B3_N_SHIFT, "mulNN", a, b, b3RefN( a ), b3RefN( b ) )
}

/// Q12 * Q24 -> Q12. Impulse magnitude scaled by inverse mass gives a
/// velocity; force by inverse inertia gives an angular acceleration.
static inline b3f b3MulFW( b3f a, b3iw b )
{
	B3_FIX_MUL_BODY( b3f, B3_F_SHIFT, B3_W_SHIFT, "mulFW", a, b, b3RefF( a ), b3RefW( b ) )
}

/// Q12 * Q24 -> Q12. Velocity by a timestep gives a position delta. This is
/// the operation that makes b3t worth having: at Q12 the substep would
/// quantize to 17/4096 instead of 1/240.
static inline b3f b3MulFT( b3f a, b3t b )
{
	B3_FIX_MUL_BODY( b3f, B3_F_SHIFT, B3_T_SHIFT, "mulFT", a, b, b3RefF( a ), b3RefT( b ) )
}

/// Q12 * Q30 -> Q12. Length scaled by a coefficient (friction, restitution,
/// solver softness).
static inline b3f b3MulFC( b3f a, b3c b )
{
	B3_FIX_MUL_BODY( b3f, B3_F_SHIFT, B3_C_SHIFT, "mulFC", a, b, b3RefF( a ), b3RefC( b ) )
}

/// Q30 * Q30 -> Q30. Coefficient by coefficient.
static inline b3c b3MulCC( b3c a, b3c b )
{
	B3_FIX_MUL_BODY( b3c, B3_C_SHIFT, B3_C_SHIFT, "mulCC", a, b, b3RefC( a ), b3RefC( b ) )
}

/// Q24 * Q24 -> Q24. Composing timesteps.
static inline b3t b3MulTT( b3t a, b3t b )
{
	B3_FIX_MUL_BODY( b3t, B3_T_SHIFT, B3_T_SHIFT, "mulTT", a, b, b3RefT( a ), b3RefT( b ) )
}

/// Q24 * Q30 -> Q24. Inverse inertia rotated by a unit-vector component.
static inline b3iw b3MulWN( b3iw a, b3n b )
{
	B3_FIX_MUL_BODY( b3iw, B3_W_SHIFT, B3_N_SHIFT, "mulWN", a, b, b3RefW( a ), b3RefN( b ) )
}

/// Q24 * Q24 -> Q24. Composing inverse inertias (the I^-1 similarity
/// transform does this twice).
static inline b3iw b3MulWW( b3iw a, b3iw b )
{
	B3_FIX_MUL_BODY( b3iw, B3_W_SHIFT, B3_W_SHIFT, "mulWW", a, b, b3RefW( a ), b3RefW( b ) )
}

/// Q24 * Q12 -> Q24. An inverse inertia scaled by a rotation matrix entry.
///
/// Distinct from b3MulWN, which takes the rotation as a Q30 direction
/// component. b3MakeMatrixFromQuat returns Q12, so the world-space inverse
/// inertia -- R * I^-1 * R^T -- is built from this one instead. Keeping the
/// result at Q24 rather than narrowing to Q12 is the whole point: the tensor
/// is the solver's divisor, and a Q12 copy of a 0.001 entry has four bits.
static inline b3iw b3MulWF( b3iw a, b3f b )
{
	B3_FIX_MUL_BODY( b3iw, B3_W_SHIFT, B3_F_SHIFT, "mulWF", a, b, b3RefW( a ), b3RefF( b ) )
}

/// Q12 * Q24 -> Q30. Angular velocity times a substep, as a rotation
/// increment. Shift is 12+24-30 = 6.
///
/// This primitive exists because the obvious spelling is badly wrong. A body
/// spinning at 1 rad/s advances 1/240 rad per substep, and the half-angle a
/// quaternion update needs is half of that -- about 0.00208 rad. In Q12 that
/// is *eight quanta*, and halving it truncates to four: a 6% error, applied
/// every substep, always in the same direction. Integrated for ten seconds
/// the body ends up 36 degrees behind where it belongs.
///
/// At Q30 the same increment is 2.2 million quanta and the error disappears.
/// So an angular increment goes from Q12 velocity straight to a Q30 rotation
/// and is never represented at Q12 in between.
static inline b3n b3MulFTToN( b3f a, b3t b )
{
	B3_FIX_MUL_BODY( b3n, B3_N_SHIFT, ( B3_F_SHIFT + B3_T_SHIFT - B3_N_SHIFT ), "mulFTToN", a, b, b3RefF( a ), b3RefT( b ) )
}

/// Q12 * Q24 -> Q24. Velocity times a substep, as a *position* increment.
/// Shift is 12+24-24 = 12.
///
/// The counterpart of b3MulFTToN, for the same reason and with the same
/// failure mode. b3MulFT gives the increment at Q12, and a Q12 position
/// increment is far coarser than it looks: a body moving at 1.5 m/s advances
/// 25.6 quanta per substep, and *no* rounding of 25.6 is exact. Round to
/// nearest takes 26 every single substep -- the error is a constant +0.4, not
/// a zero-mean one, because the operand does not change between substeps.
///
/// That is the case B3_MUL_ROUND cannot help with. Its comment is about a
/// truncation bias, which rounding fixes; this is a *representation* bias, and
/// the only fix is not to represent the increment at Q12. Measured against
/// float Box3D the position ran 1.56% long at 1.5 m/s, and it gets worse as
/// the body slows: 5.5% at 0.5 m/s, 17% at 0.1 m/s, which is the speed range a
/// settling stack lives in.
///
/// At Q24 the same increment is 105 million quanta and the residual rounding
/// is 3e-8 m per substep. b3BodyState::deltaPosition is carried at this scale
/// and narrowed once per step, with the remainder carried forward, so the
/// position integral has no drift at all rather than merely a slower one.
static inline b3iw b3MulFTToW( b3f a, b3t b )
{
	B3_FIX_MUL_BODY( b3iw, B3_W_SHIFT, ( B3_F_SHIFT + B3_T_SHIFT - B3_W_SHIFT ), "mulFTToW", a, b, b3RefF( a ),
					 b3RefT( b ) )
}

/// Halve a Q30 value. Exact at this scale, unlike halving a small Q12 value.
static inline b3n b3HalfN( b3n a )
{
	return b3Makeb3nRef( b3Raw( a ) >> 1, B3_REF( b3RefN( a ) * 0.5 ) );
}

/// Q16 * Q24 -> Q12. Accumulated impulse applied through an inverse mass,
/// yielding the velocity change. Shift is 16+24-12 = 28.
static inline b3f b3MulImpW( b3imp a, b3iw b )
{
	B3_FIX_MUL_BODY( b3f, B3_F_SHIFT, ( B3_IMP_SHIFT + B3_W_SHIFT - B3_F_SHIFT ), "mulImpW", a, b, b3RefImp( a ), b3RefW( b ) )
}

/// Q16 * Q30 -> Q16. Impulse scaled by a coefficient, e.g. the friction cone
/// clamp or a warm-start scale factor.
static inline b3imp b3MulImpC( b3imp a, b3c b )
{
	B3_FIX_MUL_BODY( b3imp, B3_IMP_SHIFT, B3_C_SHIFT, "mulImpC", a, b, b3RefImp( a ), b3RefC( b ) )
}

/// Q16 * Q30 -> Q12. Impulse projected onto a unit direction, in length
/// units. Shift is 16+30-12 = 34.
static inline b3f b3MulImpN( b3imp a, b3n b )
{
	B3_FIX_MUL_BODY( b3f, B3_F_SHIFT, ( B3_IMP_SHIFT + B3_N_SHIFT - B3_F_SHIFT ), "mulImpN", a, b, b3RefImp( a ), b3RefN( b ) )
}

/// Q12 * Q12 -> Q16. A mass times a velocity, giving an impulse. Shift is
/// 12+12-16 = 8, so this *widens* rather than narrowing.
///
/// The contact solver's innermost line:
///
///     deltaImpulse = -normalMass * ( massScale * vn + velocityBias )
///
/// Both operands are lengths-per-time and masses at Q12, and the product is
/// the accumulator that gets warm-started, so it lands at the impulse scale
/// rather than staying at Q12. Widening means no rounding at all here -- the
/// eight bits come from the product being wider than either operand, not from
/// discarding any.
static inline b3imp b3MulFFToImp( b3f a, b3f b )
{
	B3_FIX_MUL_BODY( b3imp, B3_IMP_SHIFT, ( B3_F_SHIFT + B3_F_SHIFT - B3_IMP_SHIFT ), "mulFFToImp", a, b, b3RefF( a ),
					 b3RefF( b ) )
}

/// Q16 * Q12 -> Q16. An impulse scaled by a length or a plain Q12 factor:
/// `leverArm * normalImpulse` for the twist limit, and the friction cone's
/// `maxImpulse / |accumulated|` rescale.
///
/// Distinct from b3MulImpC, which takes a coefficient already known to be in
/// [-1,1] at Q30. A lever arm is not bounded by one.
static inline b3imp b3MulImpF( b3imp a, b3f b )
{
	B3_FIX_MUL_BODY( b3imp, B3_IMP_SHIFT, B3_F_SHIFT, "mulImpF", a, b, b3RefImp( a ), b3RefF( b ) )
}

/// Q12 * Q24 -> Q16. A force times a substep, giving the impulse it may
/// deliver in that substep. Shift is 12+24-16 = 20.
///
/// This is how a joint turns a *force* bound into an impulse bound -- a
/// motor's `h * maxMotorForce`, a spring's `lowerSpringForce * h`. Those
/// clamps have to be compared against a warm-started accumulator, which lives
/// at Q16, so the conversion belongs in the multiply rather than after it.
///
/// b3MulFT is the wrong operation despite the same operand scales: it returns
/// Q12, and narrowing a force bound to Q12 before comparing it against a Q16
/// accumulator throws away the four bits that distinguish adjacent clamp
/// values on a light body.
static inline b3imp b3MulFTToImp( b3f a, b3t b )
{
	B3_FIX_MUL_BODY( b3imp, B3_IMP_SHIFT, ( B3_F_SHIFT + B3_T_SHIFT - B3_IMP_SHIFT ), "mulFTToImp", a, b, b3RefF( a ),
					 b3RefT( b ) )
}

/// Q16 * Q12 -> Q12. An accumulated impulse divided by the substep -- spelled
/// as a multiply by inv_h, which is the Q12 form the solver carries. Shift is
/// 16+12-12 = 16.
///
/// The inverse of b3MulFTToImp, and what every joint reaction query is:
/// `b3GetJointReaction` reports the force a constraint applied, and the only
/// record of it is the impulse the solver accumulated.
///
/// Distinct from b3MulImpF, which has the same operand scales but returns an
/// impulse. That one scales an impulse by a dimensionless-or-length factor;
/// this one divides by time and changes what the number *is*.
static inline b3f b3MulImpFToF( b3imp a, b3f b )
{
	B3_FIX_MUL_BODY( b3f, B3_F_SHIFT, ( B3_IMP_SHIFT + B3_F_SHIFT - B3_F_SHIFT ), "mulImpFToF", a, b, b3RefImp( a ),
					 b3RefF( b ) )
}

#undef B3_FIX_MUL_BODY

// =========================================================================
// Scale conversions
// =========================================================================
//
// Widening (Q12 -> Q30) is exact but can overflow; the value must already be
// in range for the destination. Narrowing truncates toward negative infinity.

/// Q12 -> Q30. Valid only for |a| < 2; used on values already known to be a
/// direction component or a normalized coefficient.
static inline b3n b3FToN( b3f a )
{
	return b3Makeb3nRef( b3Raw( a ) << ( B3_N_SHIFT - B3_F_SHIFT ), b3RefF( a ) );
}

/// Q30 -> Q12.
static inline b3f b3NToF( b3n a )
{
	return b3Makeb3fRef( B3_SHIFT_ROUND( b3Raw( a ), B3_N_SHIFT - B3_F_SHIFT ), b3RefN( a ) );
}

/// Q12 -> Q24. Valid only for |a| < 128.
static inline b3iw b3FToW( b3f a )
{
	return b3Makeb3iwRef( b3Raw( a ) << ( B3_W_SHIFT - B3_F_SHIFT ), b3RefF( a ) );
}

/// Q24 -> Q12.
static inline b3f b3WToF( b3iw a )
{
	return b3Makeb3fRef( B3_SHIFT_ROUND( b3Raw( a ), B3_W_SHIFT - B3_F_SHIFT ), b3RefW( a ) );
}

/// Q12 -> Q16. Valid only for |a| < 32768.
static inline b3imp b3FToImp( b3f a )
{
	return b3Makeb3impRef( b3Raw( a ) << ( B3_IMP_SHIFT - B3_F_SHIFT ), b3RefF( a ) );
}

/// Q16 -> Q12.
static inline b3f b3ImpToF( b3imp a )
{
	return b3Makeb3fRef( B3_SHIFT_ROUND( b3Raw( a ), B3_IMP_SHIFT - B3_F_SHIFT ), b3RefImp( a ) );
}

/// Q30 -> Q30, reinterpreting a direction component as a coefficient. Both
/// are Q30, so this is free; it exists to keep the intent visible in strict
/// mode rather than to change bits.
static inline b3c b3NToC( b3n a )
{
	return b3Makeb3cRef( b3Raw( a ), b3RefN( a ) );
}

static inline b3n b3CToN( b3c a )
{
	return b3Makeb3nRef( b3Raw( a ), b3RefC( a ) );
}

/// Q24 -> Q24, time reinterpreted as an inverse-weight scale.
static inline b3t b3WToT( b3iw a )
{
	return b3Makeb3tRef( b3Raw( a ), b3RefW( a ) );
}

/// Q24 -> Q12. A timestep as a plain scalar, for the 1/h factors.
static inline b3f b3TToF( b3t a )
{
	return b3Makeb3fRef( B3_SHIFT_ROUND( b3Raw( a ), B3_T_SHIFT - B3_F_SHIFT ), b3RefT( a ) );
}

/// Q30 -> Q12.
static inline b3f b3CToF( b3c a )
{
	return b3Makeb3fRef( B3_SHIFT_ROUND( b3Raw( a ), B3_C_SHIFT - B3_F_SHIFT ), b3RefC( a ) );
}

/// Q12 -> Q30, saturating. Unlike b3FToN this is safe for any input: values
/// outside [-2, 2] clamp instead of wrapping. Used where a computed ratio is
/// expected to be a coefficient but is not guaranteed to be in range.
static inline b3c b3FToCSat( b3f a )
{
	int32_t raw = b3Raw( a );
	if ( raw >= ( B3_F_ONE << 1 ) )
	{
		return b3Makeb3cRef( INT32_MAX, b3RefF( a ) );
	}
	if ( raw <= -( B3_F_ONE << 1 ) )
	{
		return b3Makeb3cRef( INT32_MIN, b3RefF( a ) );
	}
	return b3Makeb3cRef( raw << ( B3_C_SHIFT - B3_F_SHIFT ), b3RefF( a ) );
}

// =========================================================================
// Hardware divide
// =========================================================================
//
// The DS exposes a divider through REG_DIV_NUMER / REG_DIV_DENOM with a
// 64/32 mode. It is not instantaneous -- roughly 36 cycles -- so the async
// form is provided too: start the divide, do unrelated work, collect. The
// solver's inner loops are the place to use that.
//
// On the host these fall back to plain integer division, which produces
// identical results (the hardware truncates toward zero the same way).

#ifdef __NDS__

static inline int32_t b3HwDiv64( int64_t num, int32_t den )
{
	return div64( num, den );
}

static inline uint32_t b3HwSqrt64( uint64_t a )
{
	return sqrt64( a );
}

#else

static inline int32_t b3HwDiv64( int64_t num, int32_t den )
{
	return (int32_t)( num / den );
}

static inline uint32_t b3HwSqrt64( uint64_t a )
{
	// Integer Newton iteration. Matches the hardware's truncating result.
	if ( a == 0 )
	{
		return 0;
	}
	uint64_t x = (uint64_t)sqrt( (double)a );
	// Correct the last bit, which the double round-trip can get wrong for
	// operands above 2^52.
	while ( x > 0 && x * x > a )
	{
		x--;
	}
	while ( ( x + 1 ) * ( x + 1 ) <= a )
	{
		x++;
	}
	return (uint32_t)x;
}

#endif

/// Q12 / Q12 -> Q12.
static inline b3f b3DivFF( b3f a, b3f b )
{
	if ( b3Raw( b ) == 0 )
	{
		return b3f_zero;
	}
	int32_t r = b3HwDiv64( (int64_t)b3Raw( a ) << B3_F_SHIFT, b3Raw( b ) );
	return b3Makeb3fRef( r, B3_REF( b3RefF( a ) / b3RefF( b ) ) );
}

/// Q12 / Q12 -> Q30. The quotient must be a coefficient in [-2, 2]; used for
/// ratios that are known to be bounded, such as a barycentric weight.
static inline b3c b3DivFFToC( b3f a, b3f b )
{
	if ( b3Raw( b ) == 0 )
	{
		return b3c_zero;
	}
	int32_t r = b3HwDiv64( (int64_t)b3Raw( a ) << B3_C_SHIFT, b3Raw( b ) );
	return b3Makeb3cRef( r, B3_REF( b3RefF( a ) / b3RefF( b ) ) );
}

/// Smallest mass with a representable inverse.
///
/// The quotient is 2^36 / mass_raw, and the DS divider returns only 32 bits,
/// so mass_raw must be at least 2^36 / 2^31 = 32 for the result to fit. 33 is
/// the first safe value: 2^36 / 33 = 2083697220, just inside INT32_MAX.
///
/// In real terms the lightest body the solver can represent is 33/4096, about
/// 8 grams at 1 unit = 1 kg, with a maximum inverse mass of ~124.
#define B3_MIN_MASS_RAW 33

/// The magnitude of 2^36 / mag for a denominator too wide for the divider.
/// Used by b3RcpWide below; see the commentary there.
int32_t b3RcpWideSlow( uint64_t mag );

/// Shared by b3RcpF and b3RcpW, which are the same divide with the Q12 and Q24
/// operands swapped and had drifted into two copies of it.
///
/// @param raw
///     Non-zero. The caller has already mapped zero to zero.
/// @return
///     The signed quotient, clamped as B3_MIN_MASS_RAW describes.
///
/// The INT32_MIN arm is the fix for a defect the Stage 7 Step 0 sweep found. It
/// used to be `raw < 0 ? -raw : raw`, where -INT32_MIN has no int32
/// representation and wraps back to INT32_MIN -- which is negative, so it then
/// tested `< B3_MIN_MASS_RAW` and clamped to 33. b3RcpW( INT32_MIN ) therefore
/// returned -2083697220 where the answer is -32: a constraint with an almost
/// vanishing inverse mass reported as maximally stiff, off by a factor of 65
/// million and in the saturating direction. That is the same shape as the
/// spherical joint's measured disaster, which also saturated instead of
/// reporting failure.
///
/// Not reachable from the solver today -- an inverse mass of exactly -128 at Q24
/// means nothing physical, and effective masses are positive -- so this is a
/// latent boundary defect, recorded as one. It was found because b3RcpWide's new
/// wide branch made INT32_MIN the fast path's last value and the sweep asserted
/// across the seam.
///
/// Clamping the magnitude to INT32_MAX is **exact here, not approximate**, which
/// is what keeps the fix free: 2^36 / 2^31 is 32, and 2^36 / (2^31 - 1) is
/// 32.0000000149, which floors to 32 as well. The one value that cannot be
/// negated lands on the quotient it would have had anyway. Routing it through
/// b3RcpWideSlow instead was the first attempt and cost ~900 bytes of .text,
/// because b3RcpW is inlined at a great many call sites and each one carried the
/// wide branch.
static inline int32_t b3RcpRaw( int32_t raw )
{
	int32_t mag = raw < 0 ? ( raw == INT32_MIN ? INT32_MAX : -raw ) : raw;
	if ( mag < B3_MIN_MASS_RAW )
	{
		mag = B3_MIN_MASS_RAW;
	}

	int32_t r = b3HwDiv64( (int64_t)1 << ( B3_F_SHIFT + B3_W_SHIFT ), mag );
	return raw < 0 ? -r : r;
}

/// 1 / Q12 -> Q24. The mass-to-inverse-mass conversion.
///
/// Zero maps to zero, matching Box3D's convention that an inverse mass of
/// zero means infinite mass -- a static body. Note that a mass small enough
/// to round to zero in Q12 (below 1/4096) therefore becomes static rather
/// than very light. Masses are clamped to B3_MIN_MASS_RAW above that, so the
/// quotient can never overflow.
static inline b3iw b3RcpF( b3f a )
{
	int32_t raw = b3Raw( a );
	if ( raw == 0 )
	{
		return b3iw_zero;
	}
	return b3Makeb3iwRef( b3RcpRaw( raw ), B3_REF( 1.0 / b3RefF( a ) ) );
}

/// Q12 / Q12 -> Q24. A ratio that needs inverse-weight precision, such as an
/// effective mass reciprocal.
static inline b3iw b3DivFFToW( b3f a, b3f b )
{
	if ( b3Raw( b ) == 0 )
	{
		return b3iw_zero;
	}
	int32_t r = b3HwDiv64( (int64_t)b3Raw( a ) << B3_W_SHIFT, b3Raw( b ) );
	return b3Makeb3iwRef( r, B3_REF( b3RefF( a ) / b3RefF( b ) ) );
}

/// 1 / Q24 -> Q12. The inverse-mass-to-mass conversion -- b3RcpF backwards.
///
/// Every effective mass in the contact solver is built this way. `kNormal` is
/// a sum of inverse masses and inverse-inertia quadratic forms, so it lives at
/// Q24; the constraint needs its reciprocal, which is a mass and belongs at
/// Q12. b3RcpF cannot be reused: it takes the Q12 operand and returns the Q24
/// one, which is the opposite pair.
///
/// The numerator is the same 2^36, because F_SHIFT + W_SHIFT is symmetric, and
/// so is the clamp -- the DS divider returns 32 bits, so the divisor must be at
/// least 33 for the quotient to fit. Here that bounds the *effective mass* at
/// 2^36/33 / 4096 = 508714 rather than bounding the body mass, so an
/// unreachably stiff constraint saturates instead of wrapping.
///
/// Zero maps to zero: an infinite effective mass is a constraint between two
/// bodies that cannot move, and the solver reads a zero normalMass as "apply
/// no impulse", which is the right answer for it.
static inline b3f b3RcpW( b3iw a )
{
	int32_t raw = b3Raw( a );
	if ( raw == 0 )
	{
		return b3f_zero;
	}
	return b3Makeb3fRef( b3RcpRaw( raw ), B3_REF( 1.0 / b3RefW( a ) ) );
}

/// Reciprocal of a Q24 value held in an int64, giving a Q12 result.
///
/// The wide sibling of b3RcpW, for effective masses that do not fit b3iw.
///
/// A single body's inverse inertia is capped at Q7.24's ceiling of 128 by
/// b3InvertInertia, which scales rather than wraps. But a *joint* divides by
/// the sum of two of them, and two light bodies sum past 128 -- a ragdoll's
/// head and torso reach 128.197 -- so the sum has to be carried wide even
/// though neither term needs to be.
///
/// Anything b3iw can hold is handed straight to b3RcpW, so a value in range
/// gives bit-identical results to the code this replaced. Only the case that
/// used to wrap takes the wide divide.
///
/// The magnitude of the wide quotient, floor( 2^36 / mag ), for mag > INT32_MAX.
///
/// Out of line, and not a divide. It used to be `( 1 << 36 ) / mag` written
/// inline, which is a 64-by-64 divide the ARM has no instruction for and the DS
/// divider has no mode for -- b3HwDiv64 takes a 32-bit denominator and this
/// branch's denominator is what exceeded it. That pulled __aeabi_ldivmod into
/// all six objects that call b3RcpWide.
///
/// Its comment justified the cost as "once per prepare", and the wheel joint
/// made that stale: b3WheelJoint calls b3RcpWide up to four times per *solve*.
/// The replacement exploits the fact that this branch's quotient is at most 32
/// -- see b3RcpWideSlow's own comment -- so six restoring-division steps compute
/// it exactly, with no library call and no divider access. Exactly, so the
/// change moves no result: normalising mag into int32 and shifting the numerator
/// to match would have been shorter and would have cost up to one raw unit out
/// of a quotient under 32.
///
/// Defined in b3hot.c, alongside b3RcpWideSlow's other caller-side neighbours,
/// and for the same reason the slow path moved out of line: three objects were
/// carrying their own copy.
b3f b3RcpWide( int64_t a );

/// Count leading zeros of a 64-bit value. Zero counts as 64.
///
/// Inline rather than __builtin_clzll, which lowers to a __clzdi2 call on
/// ARMv5TE for the same reason __builtin_ctzll does. The 32-bit builtin maps
/// straight onto the CLZ instruction.
///
/// The zero case is **not** defensive padding. `__builtin_clz( 0 )` is
/// undefined, and without the guard this returned whatever the optimizer felt
/// like: at -O0 and -O2 it happened to give 64, and at -O1 it gave a value
/// small enough to send b3DivWideToF into its saturation branch, so a zero
/// numerator divided to B3_F_MAX instead of to zero.
///
/// That is reachable from ordinary code. b3InvertMatrixW divides every cofactor
/// by the determinant, and an exactly diagonal matrix has six cofactors that are
/// exactly zero -- so its inverse came back with six saturated off-diagonal
/// entries. Nothing hit it before Phase 6 Stage 4 because a contact's lever arms
/// and a hinge's axes never leave a cofactor exactly zero; a ball joint on a
/// *sphere* has a perfectly isotropic inverse inertia, and it does.
static inline int b3Clz64( uint64_t v )
{
	uint32_t hi = (uint32_t)( v >> 32 );
	if ( hi != 0 )
	{
		return __builtin_clz( hi );
	}

	uint32_t lo = (uint32_t)v;
	return lo != 0 ? 32 + __builtin_clz( lo ) : 64;
}

/// Divide one wide value by another, producing a Q30 coefficient.
///
/// This is the shape that keeps appearing once geometry gets involved: a
/// ratio of two quantities that are each far too large for Q12, whose
/// *quotient* is a well-behaved fraction. Segment-segment closest points
/// divide by `a*e - b*b`, a fourth power of length; GJK divides barycentric
/// numerators by a simplex determinant. Neither operand fits 32 bits, and
/// neither needs to -- only the ratio is ever used.
///
/// The operands are normalized down together until the hardware divider can
/// take them, which is exact in the ratio: shifting both sides of a fraction
/// by the same amount does not change it, it only discards low bits that the
/// Q30 result could not have represented anyway.
///
/// The result **saturates** rather than wrapping when the quotient leaves the
/// Q30 range. That is not defensive padding -- it is required by how the
/// callers are written. They divide first and clamp afterwards, so the
/// pre-clamp quotient is routinely out of range: two skew segments whose
/// closest approach lies beyond the end of one of them produce a barycentric
/// fraction of 2 or more, and Q30 cannot hold 2.0 at all (it is exactly
/// 2^31). Wrapping there turns "past the far end" into "at the near end",
/// which is the wrong end of the segment.
b3c b3DivWideToC( int64_t num, int64_t den );

/// Same, landing at Q12 instead of Q30.
///
/// The Q30 form exists for quotients that are ratios and therefore near one.
/// This one is for quotients that are *lengths* -- specifically the cofactor
/// over determinant of a 3x3 solve, where numerator and denominator are both
/// triple products at Q36 and the answer is an ordinary Q12 quantity with no
/// reason to be small. It saturates to B3_F_MAX/B3_F_MIN rather than to two.
b3f b3DivWideToF( int64_t num, int64_t den );

// -------------------------------------------------------------------------
// Asynchronous divide
// -------------------------------------------------------------------------
// Start a divide, run independent work, then collect. Only worth it where
// there genuinely is unrelated work to overlap -- otherwise b3DivFF is the
// same thing with less ceremony.

#ifdef __NDS__

static inline void b3DivFFStart( b3f a, b3f b )
{
	divf32_asynch( b3Raw( a ), b3Raw( b ) );
}

static inline b3f b3DivFFCollect( void )
{
	return b3Makeb3f( divf32_result() );
}

#else

// The host has no divider to overlap with, so the value is simply stashed.
extern b3f b3_hostPendingDiv;

static inline void b3DivFFStart( b3f a, b3f b )
{
	b3_hostPendingDiv = b3DivFF( a, b );
}

static inline b3f b3DivFFCollect( void )
{
	return b3_hostPendingDiv;
}

#endif

// =========================================================================
// Square root
// =========================================================================

/// sqrt of a Q12 value, in Q12.
///
/// Routed through the 64-bit hardware path so that the intermediate a<<12
/// cannot overflow -- which is exactly the failure NEA_Vec3LengthSq in
/// NEACollision.h has today for large vectors.
static inline b3f b3SqrtF( b3f a )
{
	if ( b3Raw( a ) <= 0 )
	{
		return b3f_zero;
	}
	uint32_t r = b3HwSqrt64( (uint64_t)(uint32_t)b3Raw( a ) << B3_F_SHIFT );
	return b3Makeb3fRef( (int32_t)r, B3_REF( sqrt( b3RefF( a ) ) ) );
}

/// sqrt of a squared length held in an int64 at Q24, giving a Q12 length.
///
/// This is the form the vector length routines want: dot(v,v) accumulated
/// wide, never narrowed, so a vector of any representable magnitude has an
/// exact squared length going into the root.
///
/// Defined in b3hot.c.
b3f b3SqrtWide( int64_t sq );

/// sqrt of a squared *impulse* held in an int64 at Q32, giving a Q16 impulse.
///
/// The sibling of b3SqrtWide, and exact for the same reason: squaring a Q16
/// value gives Q32, and the integer square root of the raw Q32 product is
/// itself the raw Q16 root -- 2^32 halves to 2^16 under the root, so no shift
/// is involved and no bits are lost.
///
/// Both friction cones need it, to rescale an accumulated impulse that has
/// left the cone. The *comparison* that decides whether to rescale is done on
/// raw int64 squares and never comes here, so this runs only on the clamping
/// path.
static inline b3imp b3SqrtWideImp( int64_t sq )
{
	if ( sq <= 0 )
	{
		return b3imp_zero;
	}
	return b3Makeb3impRef( (int32_t)b3HwSqrt64( (uint64_t)sq ),
						   B3_REF( sqrt( (double)sq / (double)( (int64_t)1 << ( 2 * B3_IMP_SHIFT ) ) ) ) );
}

/// Reciprocal square root of a wide Q24 squared length, as a Q24 inverse
/// weight.
///
/// The return type is b3iw, not b3c, and that is not cosmetic. A reciprocal
/// length is only bounded by 2 for vectors longer than half a unit -- the
/// reciprocal of a 0.3-long offset is 3.33, which Q30 cannot hold at all.
/// Returning b3c capped normalization at vectors of half a unit and silently
/// produced garbage below that, because the quotient overflowed the 32-bit
/// divider result. Q24 raises the ceiling to 128, so vectors down to about
/// 1/128 of a unit normalize correctly -- comfortably below B3_LINEAR_SLOP,
/// where a vector is no longer meaningfully a direction.
///
/// Shorter than that, the result saturates rather than wrapping.
static inline b3iw b3RsqrtWide( int64_t sq )
{
	if ( sq <= 0 )
	{
		return b3iw_zero;
	}

	int32_t len = (int32_t)b3HwSqrt64( (uint64_t)sq );
	if ( len == 0 )
	{
		return b3iw_zero;
	}

	// 1/len at Q24 is 2^(12+24) / len_raw. Saturate when len is small enough
	// that the quotient would not fit.
	const int64_t numerator = (int64_t)1 << ( B3_F_SHIFT + B3_W_SHIFT );
	if ( (int64_t)len * INT32_MAX < numerator )
	{
		return b3Makeb3iw( INT32_MAX );
	}

	return b3Makeb3iwRef( b3HwDiv64( numerator, len ),
						  B3_REF( 1.0 / sqrt( (double)sq / (double)( (int64_t)1 << 24 ) ) ) );
}

// =========================================================================
// Trigonometry
// =========================================================================
//
// Upstream computes sine and cosine with a Bhaskara rational approximation
// and atan2 with a minimax polynomial. Both are division-heavy, which is the
// wrong trade on a machine with a 36-cycle divider and a spare LUT.
//
// sinLerp/cosLerp return 4.12, so a directly constructed quaternion carries
// only 12 bits of trig precision. That is fine: trig appears when *building*
// a rotation, and the result is normalized immediately. Quaternion
// integration -- the path that runs every substep and where drift would
// accumulate -- uses no trig at all and stays at Q30 throughout.

/// Sine of an angle in brads, as a Q30 coefficient.
static inline b3c b3SinA( b3a angle )
{
#ifdef __NDS__
	int32_t s = sinLerp( angle );
#else
	int32_t s = (int32_t)( sin( (double)angle * ( 2.0 * 3.14159265358979323846 / 32768.0 ) ) * 4096.0 );
#endif
	return b3Makeb3cRef( s << ( B3_C_SHIFT - B3_F_SHIFT ), B3_REF( (double)s / 4096.0 ) );
}

/// Cosine of an angle in brads, as a Q30 coefficient.
static inline b3c b3CosA( b3a angle )
{
#ifdef __NDS__
	int32_t c = cosLerp( angle );
#else
	int32_t c = (int32_t)( cos( (double)angle * ( 2.0 * 3.14159265358979323846 / 32768.0 ) ) * 4096.0 );
#endif
	return b3Makeb3cRef( c << ( B3_C_SHIFT - B3_F_SHIFT ), B3_REF( (double)c / 4096.0 ) );
}

/// Arc cosine of a Q30 coefficient in [-1, 1], in brads.
static inline b3a b3AcosC( b3c a )
{
	int32_t q12 = b3Raw( a ) >> ( B3_C_SHIFT - B3_F_SHIFT );
	if ( q12 > B3_F_ONE )
	{
		q12 = B3_F_ONE;
	}
	if ( q12 < -B3_F_ONE )
	{
		q12 = -B3_F_ONE;
	}
#ifdef __NDS__
	return acosLerp( (int16_t)q12 );
#else
	return (int16_t)( acos( (double)q12 / 4096.0 ) * ( 32768.0 / ( 2.0 * 3.14159265358979323846 ) ) );
#endif
}

/// Arc sine of a Q30 coefficient in [-1, 1], in brads.
static inline b3a b3AsinC( b3c a )
{
	int32_t q12 = b3Raw( a ) >> ( B3_C_SHIFT - B3_F_SHIFT );
	if ( q12 > B3_F_ONE )
	{
		q12 = B3_F_ONE;
	}
	if ( q12 < -B3_F_ONE )
	{
		q12 = -B3_F_ONE;
	}
#ifdef __NDS__
	return asinLerp( (int16_t)q12 );
#else
	return (int16_t)( asin( (double)q12 / 4096.0 ) * ( 32768.0 / ( 2.0 * 3.14159265358979323846 ) ) );
#endif
}

/// Two-argument arc tangent of two raw values at any **common** scale, in
/// brads.
///
/// An arc tangent depends only on the ratio of its arguments, and this one is
/// written that way: the polynomial is evaluated on `min/max` at Q30 and every
/// other decision is a sign test. So the scale of the inputs cancels, and any
/// scale may be passed as long as *both* arguments share it and neither
/// overflows an int32.
///
/// That is worth having as its own entry point rather than leaving implicit in
/// b3Atan2F, because a caller with more bits than Q12 should not have to throw
/// them away to ask this question. b3GetSwingAngle is exactly that caller: its
/// two arguments are Q30 square roots, and narrowing them to Q12 first would
/// put a floor of about 2.5 brads under the smallest swing it can report --
/// coarser than the b3a it returns.
b3a b3Atan2Raw( int32_t y, int32_t x );

/// Two-argument arc tangent of Q12 lengths, in brads.
///
/// libnds has no atan2, so this keeps upstream's minimax polynomial shape but
/// evaluates it in fixed point. Joint angle readout is the only caller, so
/// the remaining divide is not on any hot path.
static inline b3a b3Atan2F( b3f y, b3f x )
{
	return b3Atan2Raw( b3Raw( y ), b3Raw( x ) );
}

/// Brads to radians, as a Q12 length-scale value.
///
/// The port measures angles in brads everywhere a caller sees one -- it is
/// what b3MakeQuatFromAxisAngle takes and what b3Atan2F returns -- but a joint
/// *constraint* cannot work in them. `bias = biasRate * c` has to come out as
/// an angular velocity in rad/s, so `c` has to be in radians, and the two
/// scales differ by 2*pi/32768.
///
/// One brad is 0.785 Q12 quanta, so this narrows slightly: a converted angle
/// carries a little under one quantum of error. That is why the revolute
/// joint applies it to the *difference* `angle - lowerAngle` rather than to
/// each angle separately -- converting both and subtracting would double the
/// error and make it depend on where the hinge happens to sit rather than on
/// how far it is from its limit.
///
/// The constant: one brad at Q12 is 2*pi*4096/32768 = pi/4 = 0.7853982.
/// 25736/32768 is 0.7854004, which is 2.8e-6 high -- three orders below the
/// quantum the result is rounded to, so the multiply contributes nothing next
/// to the narrowing above.
static inline b3f b3BradToRadF( b3a brads )
{
	// (brads * pi/4) at Q12, as a 32768ths multiply so it is one SMULL and a
	// shift rather than a divide.
	return b3Makeb3fRef( (int32_t)( ( (int64_t)brads * 25736 ) >> 15 ),
						 B3_REF( (double)brads * ( 2.0 * 3.14159265358979323846 / 32768.0 ) ) );
}

#endif // B3_FIXED_H__
