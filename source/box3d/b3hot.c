// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Erin Catto        (original Box3D)
// Copyright (c) 2026 Warioware64       (Nitro Engine Advanced fixed-point port)
//
// This file is part of Nitro Engine Advanced

/// @file   b3hot.c
/// @brief  The shared leaf math, defined once and placed in ITCM.
///
/// @section why Why these live here rather than in a header
///
/// Everything in this file was a `B3_INLINE` (= `static inline`) definition in
/// a header, and every one of them is called from most of the engine. At the
/// `-Os` the archive is built with -- see the flag comment in
/// Makefile.blocksds -- GCC judges them too large to inline and emits a
/// private out-of-line copy in each translation unit that uses one. That is
/// the worst of both worlds:
///
///   b3RotateVector         436 B x 20 copies = 8,284 B wasted
///   b3MulQuat              444 B x  9        = 3,552 B
///   b3MulMWImp             288 B x  9        = 2,304 B
///   b3Cross                184 B x 10        = 1,656 B
///   b3CollinearityPerpAxes 608 B x  3        = 1,216 B
///   ... and the rest below
///
/// The wasted ROM is the small half of it. The ARM9's instruction cache is
/// 8 KB, and the sub-step loop walks contact_solver.c, solver.c and every
/// joint file in turn -- so twenty private copies of b3RotateVector means each
/// of those files evicts the others' copy of *the same function*. One shared
/// definition is one cache footprint instead of twenty.
///
/// @section itcm Why ITCM
///
/// Once there is a single copy, it is small enough to put somewhere better
/// than main RAM. B3_ITCM (base.h) places these in the ARM9's 32 KB
/// zero-wait-state instruction memory, about 2.9 KB in total, which takes them
/// out of contention for the instruction cache altogether rather than merely
/// reducing their share of it. NEA_BOX3D_NO_ITCM=1 opts out for a game that
/// needs the ITCM for its own code.
///
/// @section membership What belongs in this file
///
/// The bar is: **GCC already refused to inline it.** Every function here had
/// three or more out-of-line copies in the archive, which is the compiler
/// stating that the body costs more than the call. Trivial helpers -- b3Add,
/// b3Sub, b3MakeVec3, the raw shift and round macros -- must stay inline in
/// the headers, because a call would cost more than the one or two
/// instructions they expand to. Adding something here that GCC *would* have
/// inlined is a regression, so check the duplicate count before moving a
/// function in:
///
///     arm-none-eabi-nm -S --size-sort -t d lib/libNEA_box3d.a
///       | awk '$3=="t"{print $2, $4}' | sort -k2 | uniq -c -f1 | sort -rn
///
/// (joined onto one line; the continuation is omitted so this stays a comment)
///
/// b3CollinearityPerpAxes is the one member that is deduplicated but *not*
/// placed in ITCM: it runs once per joint per prepare, not once per sub-step,
/// so 608 B of ITCM would buy a poor hit rate. It is here for the single
/// definition alone.

#include "joint.h"
#include "solver.h"

#include "box3d/b3fixed.h"
#include "box3d/math_fixed.h"

// =========================================================================
// Accumulate wide, round once
// =========================================================================
//
// Every function below computes a sum or difference of products that all share
// one shift. Written the obvious way -- `b3AddF( b3MulFF( .. ), b3MulFF( .. ) )`
// -- each product is separately rounded and narrowed to 32 bits before the add,
// so a three-term expression rounds three times and a cross product twice.
//
// Accumulating the whole expression in one int64 and rounding once at the end
// is better on both counts, which is unusual enough to be worth stating
// plainly:
//
//   **Fewer instructions.** ARMv5TE has SMLAL -- a 64-bit multiply-accumulate
//   in one instruction. So `a*b + c*d + e*f` is SMULL followed by two SMLALs,
//   and the rounding add and the 64-to-32 narrowing shift happen once instead
//   of three times. Written term by term it is three SMULLs, three rounding
//   adds, three narrowing shift pairs and two 32-bit adds.
//
//   **More accurate.** The intermediate roundings are gone. B3_MUL_ROUND's
//   comment in b3fixed.h explains why round-to-nearest matters at all; not
//   rounding until the end is strictly better than rounding well three times.
//
// b3DotWide and b3CrossWide in math_fixed.h already do this -- b3DotWide's own
// comment says "three SMLALs and one shift" -- and these four functions are
// simply the rest of the shared leaf math brought into line with them.
//
// The rule for reading the code: the shift passed to B3_SHIFT_ROUND is the one
// the corresponding b3Mul* would have used, and it is the same for every term
// by construction. Where the terms do *not* share a shift, the term-by-term
// form is still correct and must stay.
//
// @section results This changes results in the last bit
//
// It has to -- removing an intermediate rounding is a change in the number
// produced, always toward the exactly-rounded answer. The host suite in
// tests/box3d_host is what says whether that is safe.
//
// @section notdone Two functions this was NOT applied to, and why
//
// b3MulMV and b3RotateVector are the obvious next candidates -- both are pure
// sums of same-shift products, both are called from everywhere, and the
// rewrite saves 120 and 76 bytes respectively on top of what is here. Neither
// is applied, because each one **on its own** turns test_world's "a box slides
// down a mesh ramp" case into a hard abort: an impulse overflows int32 in
// b3MulFFToImp, with the double shadow already NaN.
//
// The scene is at least partly to blame. Replacing the rewrite with a
// deliberate one-quantum nudge to a single component of the *old* b3MulMV --
// no restructuring, just `+1` at Q12 -- also breaks that case, at 4 failures
// and 2 unexpected assertions. So the ramp is sitting on a stability cliff
// where a sub-quantum change in any direction tips it.
//
// That is not a licence to call these two safe. The nudge degrades the test;
// the rewrite *aborts* it, which is a strictly worse failure and is not
// explained by fragility alone. Something in the mesh-ramp friction path
// depends on the current rounding in a way nothing here has yet identified,
// and until it is identified these two stay as they are.
//
// Both directions are worth having: the ramp scene's sensitivity is a defect
// in its own right, and finding it would unblock 196 bytes and a measurable
// share of the per-frame instruction fetch.
//
// The four applied below each pass the whole suite individually and together.

// =========================================================================
// Vectors
// =========================================================================

b3Vec3 B3_ITCM( b3Cross )( b3Vec3 a, b3Vec3 b )
{
	// b3CrossWide is the same three components at Q24, which is exactly this
	// accumulator -- so the narrowing is all that is left to do here.
	int64_t w[3];
	b3CrossWide( w, a, b );

	return b3MakeVec3(
		b3Makeb3fRef( B3_SHIFT_ROUND( w[0], B3_F_SHIFT ),
					  B3_REF( b3RefF( a.y ) * b3RefF( b.z ) - b3RefF( a.z ) * b3RefF( b.y ) ) ),
		b3Makeb3fRef( B3_SHIFT_ROUND( w[1], B3_F_SHIFT ),
					  B3_REF( b3RefF( a.z ) * b3RefF( b.x ) - b3RefF( a.x ) * b3RefF( b.z ) ) ),
		b3Makeb3fRef( B3_SHIFT_ROUND( w[2], B3_F_SHIFT ),
					  B3_REF( b3RefF( a.x ) * b3RefF( b.y ) - b3RefF( a.y ) * b3RefF( b.x ) ) ) );
}

b3Vec3 B3_ITCM( b3Normalize )( b3Vec3 v )
{
	int64_t sq = b3LengthSquaredWide( v );
	if ( sq == 0 )
	{
		return b3Vec3_zeroFn();
	}
	return b3MulWV( b3RsqrtWide( sq ), v );
}

// Joined its sibling above only after the first pass: with b3Normalize out of
// line, this was left as the largest remaining duplicate in the archive at
// three copies of 292 B, which is the same bar every other member here met.
b3Vec3 B3_ITCM( b3GetLengthAndNormalize )( b3f* length, b3Vec3 v )
{
	int64_t sq = b3LengthSquaredWide( v );
	if ( sq == 0 )
	{
		*length = b3f_zero;
		return b3Vec3_zeroFn();
	}
	*length = b3SqrtWide( sq );
	return b3MulWV( b3RsqrtWide( sq ), v );
}

// =========================================================================
// Quaternions
// =========================================================================

b3Quat B3_ITCM( b3MulQuat )( b3Quat a, b3Quat b )
{
	b3Quat r;
	r.v.x = b3AddN( b3AddN( b3MulNN( a.s, b.v.x ), b3MulNN( a.v.x, b.s ) ),
					b3SubN( b3MulNN( a.v.y, b.v.z ), b3MulNN( a.v.z, b.v.y ) ) );
	r.v.y = b3AddN( b3AddN( b3MulNN( a.s, b.v.y ), b3MulNN( a.v.y, b.s ) ),
					b3SubN( b3MulNN( a.v.z, b.v.x ), b3MulNN( a.v.x, b.v.z ) ) );
	r.v.z = b3AddN( b3AddN( b3MulNN( a.s, b.v.z ), b3MulNN( a.v.z, b.s ) ),
					b3SubN( b3MulNN( a.v.x, b.v.y ), b3MulNN( a.v.y, b.v.x ) ) );
	r.s = b3SubN( b3MulNN( a.s, b.s ),
				  b3AddN( b3AddN( b3MulNN( a.v.x, b.v.x ), b3MulNN( a.v.y, b.v.y ) ), b3MulNN( a.v.z, b.v.z ) ) );
	return r;
}

b3Vec3 B3_ITCM( b3RotateVector )( b3Quat q, b3Vec3 v )
{
	// t = 2 * (u x v), in length units. u is Q30, v is Q12.
	b3Vec3 t = b3MakeVec3( b3SubF( b3MulFN( v.z, q.v.y ), b3MulFN( v.y, q.v.z ) ),
						   b3SubF( b3MulFN( v.x, q.v.z ), b3MulFN( v.z, q.v.x ) ),
						   b3SubF( b3MulFN( v.y, q.v.x ), b3MulFN( v.x, q.v.y ) ) );
	t = b3MakeVec3( b3AddF( t.x, t.x ), b3AddF( t.y, t.y ), b3AddF( t.z, t.z ) );

	// v + s*t + (u x t)
	b3Vec3 uxt = b3MakeVec3( b3SubF( b3MulFN( t.z, q.v.y ), b3MulFN( t.y, q.v.z ) ),
							 b3SubF( b3MulFN( t.x, q.v.z ), b3MulFN( t.z, q.v.x ) ),
							 b3SubF( b3MulFN( t.y, q.v.x ), b3MulFN( t.x, q.v.y ) ) );

	return b3MakeVec3( b3AddF( b3AddF( v.x, b3MulFN( t.x, q.s ) ), uxt.x ), b3AddF( b3AddF( v.y, b3MulFN( t.y, q.s ) ), uxt.y ),
					   b3AddF( b3AddF( v.z, b3MulFN( t.z, q.s ) ), uxt.z ) );
}

// =========================================================================
// Transforms
// =========================================================================

b3Vec3 B3_ITCM( b3TransformPoint )( b3Transform t, b3Vec3 p )
{
	return b3Add( b3RotateVector( t.q, p ), t.p );
}

// =========================================================================
// Matrices
// =========================================================================

b3Vec3 B3_ITCM( b3MulMV )( b3Matrix3 m, b3Vec3 v )
{
	return b3MakeVec3( b3AddF( b3AddF( b3MulFF( m.cx.x, v.x ), b3MulFF( m.cy.x, v.y ) ), b3MulFF( m.cz.x, v.z ) ),
					   b3AddF( b3AddF( b3MulFF( m.cx.y, v.x ), b3MulFF( m.cy.y, v.y ) ), b3MulFF( m.cz.y, v.z ) ),
					   b3AddF( b3AddF( b3MulFF( m.cx.z, v.x ), b3MulFF( m.cy.z, v.y ) ), b3MulFF( m.cz.z, v.z ) ) );
}

// The same row, with the matrix at Q24 and the vector at Q12 -- b3MulFW, so the
// shift is B3_W_SHIFT. Operand order is (vector, matrix) to match b3MulFW's.
#define B3_ROW_FW( c0, c1, c2, v )                                                                                               \
	( (int64_t)b3Raw( ( v ).x ) * b3Raw( c0 ) + (int64_t)b3Raw( ( v ).y ) * b3Raw( c1 ) +                                        \
	  (int64_t)b3Raw( ( v ).z ) * b3Raw( c2 ) )

#define B3_ROW_FW_REF( c0, c1, c2, v )                                                                                           \
	B3_REF( b3RefF( ( v ).x ) * b3RefW( c0 ) + b3RefF( ( v ).y ) * b3RefW( c1 ) + b3RefF( ( v ).z ) * b3RefW( c2 ) )

b3Vec3 B3_ITCM( b3MulMWV )( b3MatrixW m, b3Vec3 v )
{
	return b3MakeVec3(
		b3Makeb3fRef( B3_SHIFT_ROUND( B3_ROW_FW( m.cx.x, m.cy.x, m.cz.x, v ), B3_W_SHIFT ),
					  B3_ROW_FW_REF( m.cx.x, m.cy.x, m.cz.x, v ) ),
		b3Makeb3fRef( B3_SHIFT_ROUND( B3_ROW_FW( m.cx.y, m.cy.y, m.cz.y, v ), B3_W_SHIFT ),
					  B3_ROW_FW_REF( m.cx.y, m.cy.y, m.cz.y, v ) ),
		b3Makeb3fRef( B3_SHIFT_ROUND( B3_ROW_FW( m.cx.z, m.cy.z, m.cz.z, v ), B3_W_SHIFT ),
					  B3_ROW_FW_REF( m.cx.z, m.cy.z, m.cz.z, v ) ) );
}

// =========================================================================
// Impulses
// =========================================================================

// An impulse through an inverse-inertia row: b3MulImpW, Q16 * Q24 -> Q12, so
// the shift is 16 + 24 - 12 = 28.
#define B3_IMP_W_SHIFT ( B3_IMP_SHIFT + B3_W_SHIFT - B3_F_SHIFT )

#define B3_ROW_IMPW( c0, c1, c2, p )                                                                                             \
	( (int64_t)b3Raw( ( p ).x ) * b3Raw( c0 ) + (int64_t)b3Raw( ( p ).y ) * b3Raw( c1 ) +                                        \
	  (int64_t)b3Raw( ( p ).z ) * b3Raw( c2 ) )

#define B3_ROW_IMPW_REF( c0, c1, c2, p )                                                                                         \
	B3_REF( b3RefImp( ( p ).x ) * b3RefW( c0 ) + b3RefImp( ( p ).y ) * b3RefW( c1 ) + b3RefImp( ( p ).z ) * b3RefW( c2 ) )

b3Vec3 B3_ITCM( b3MulMWImp )( b3MatrixW m, b3Imp3 p )
{
	return b3MakeVec3(
		b3Makeb3fRef( B3_SHIFT_ROUND( B3_ROW_IMPW( m.cx.x, m.cy.x, m.cz.x, p ), B3_IMP_W_SHIFT ),
					  B3_ROW_IMPW_REF( m.cx.x, m.cy.x, m.cz.x, p ) ),
		b3Makeb3fRef( B3_SHIFT_ROUND( B3_ROW_IMPW( m.cx.y, m.cy.y, m.cz.y, p ), B3_IMP_W_SHIFT ),
					  B3_ROW_IMPW_REF( m.cx.y, m.cy.y, m.cz.y, p ) ),
		b3Makeb3fRef( B3_SHIFT_ROUND( B3_ROW_IMPW( m.cx.z, m.cy.z, m.cz.z, p ), B3_IMP_W_SHIFT ),
					  B3_ROW_IMPW_REF( m.cx.z, m.cy.z, m.cz.z, p ) ) );
}

b3Imp3 B3_ITCM( b3CrossVImp )( b3Vec3 r, b3Imp3 p )
{
	// b3MulImpF is Q16 * Q12 -> Q16, so the shift is B3_F_SHIFT and the two
	// terms of each component share it.
#define B3_CROSS_IMP( pa, ra, pb, rb ) ( (int64_t)b3Raw( pa ) * b3Raw( ra ) - (int64_t)b3Raw( pb ) * b3Raw( rb ) )

	return b3MakeImp3(
		b3Makeb3impRef( B3_SHIFT_ROUND( B3_CROSS_IMP( p.z, r.y, p.y, r.z ), B3_F_SHIFT ),
						B3_REF( b3RefImp( p.z ) * b3RefF( r.y ) - b3RefImp( p.y ) * b3RefF( r.z ) ) ),
		b3Makeb3impRef( B3_SHIFT_ROUND( B3_CROSS_IMP( p.x, r.z, p.z, r.x ), B3_F_SHIFT ),
						B3_REF( b3RefImp( p.x ) * b3RefF( r.z ) - b3RefImp( p.z ) * b3RefF( r.x ) ) ),
		b3Makeb3impRef( B3_SHIFT_ROUND( B3_CROSS_IMP( p.y, r.x, p.x, r.y ), B3_F_SHIFT ),
						B3_REF( b3RefImp( p.y ) * b3RefF( r.x ) - b3RefImp( p.x ) * b3RefF( r.y ) ) ) );

#undef B3_CROSS_IMP
}

// =========================================================================
// Wide scalars
// =========================================================================

b3f B3_ITCM( b3RcpWide )( int64_t a )
{
	if ( a >= INT32_MIN && a <= INT32_MAX )
	{
		return b3RcpW( b3Makeb3iw( (int32_t)a ) );
	}

	// Negated through uint64_t so that INT64_MIN, whose magnitude has no int64
	// representation, is still handled rather than being undefined.
	uint64_t mag = a < 0 ? -(uint64_t)a : (uint64_t)a;

	int32_t r = b3RcpWideSlow( mag );
	return b3Makeb3fRef( a < 0 ? -r : r, B3_REF( 1.0 / ( (double)a / (double)( (int64_t)1 << B3_W_SHIFT ) ) ) );
}

b3f B3_ITCM( b3SqrtWide )( int64_t sq )
{
	if ( sq <= 0 )
	{
		return b3f_zero;
	}
	return b3Makeb3fRef( (int32_t)b3HwSqrt64( (uint64_t)sq ), B3_REF( sqrt( (double)sq / (double)( (int64_t)1 << 24 ) ) ) );
}

// =========================================================================
// Solver support
// =========================================================================

b3Softness B3_ITCM( b3MakeSoft )( b3f hertz, b3f zeta, b3t h )
{
	if ( b3Raw( hertz ) == 0 )
	{
		b3Softness soft = { b3f_zero, b3c_zero, b3c_zero };
		return soft;
	}

	// omega = 2*pi*hertz          [1/time]  -- Q12, per B3_NEA_INV_DT
	// a1    = 2*zeta + h*omega    [1]       -- but zeta defaults to 10, so a1
	//                                          reaches ~21 and does NOT fit
	//                                          Q1.30. Q12 it is.
	// a2    = h*omega*a1          [1]       -- ~16 at the defaults. Same.
	// a3    = 1/(1 + a2)          [1]       -- in (0,1], so Q30 at last.
	b3f omega = b3MulFF( B3_TWO_PI, hertz );
	b3f hOmega = b3MulFT( omega, h );
	b3f a1 = b3AddF( b3AddF( zeta, zeta ), hOmega );
	b3f a2 = b3MulFF( hOmega, a1 );

	b3Softness soft;
	soft.impulseScale = b3DivFFToC( b3f_one, b3AddF( b3f_one, a2 ) );
	soft.massScale = b3SubC( b3c_one, soft.impulseScale );
	soft.biasRate = b3DivFF( omega, a1 );

	return soft;
}

// Deduplicated but deliberately not in ITCM -- see the file comment.
void b3CollinearityPerpAxes( b3Quat frameQ, b3Quat relQ, b3Vec3* outX, b3Vec3* outY )
{
	b3f s = b3NToF( relQ.s );
	b3Vec3 v = b3FromDir3( relQ.v );

	b3Vec3 x = b3Add( b3MulSV( s, b3Vec3_axisX ), b3Cross( v, b3Vec3_axisX ) );
	b3Vec3 y = b3Add( b3MulSV( s, b3Vec3_axisY ), b3Cross( v, b3Vec3_axisY ) );

	*outX = b3MulSV( b3CToF( B3_JOINT_HALF ), b3RotateVector( frameQ, x ) );
	*outY = b3MulSV( b3CToF( B3_JOINT_HALF ), b3RotateVector( frameQ, y ) );
}
