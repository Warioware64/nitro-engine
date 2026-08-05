// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Erin Catto        (original Box3D)
// Copyright (c) 2026 Warioware64       (Nitro Engine Advanced fixed-point port)
//
// This file is part of Nitro Engine Advanced

#ifndef B3_MATH_FIXED_H__
#define B3_MATH_FIXED_H__

/// @file   math_fixed.h
/// @brief  Fixed-point vector, quaternion and matrix math for the Box3D port.
///
/// This replaces upstream's math_functions.h. Function names are kept
/// identical so that the call sites in the ported .c files read the same as
/// upstream and stay diffable against it.
///
/// @section split Why there are two vector types
///
/// Upstream uses b3Vec3 for three things with incompatible precision needs:
/// world positions, unit normals, and the vector part of a quaternion. In
/// float that is free. In fixed point it is not, so the port splits them:
///
///   b3Vec3   three b3f (Q12)   positions, velocities, impulses, normals
///   b3Dir3   three b3n (Q30)   the quaternion vector part, unit axes
///
/// The split follows where error *accumulates*, which is the only thing that
/// matters:
///
///   - A contact normal is recomputed from geometry every frame and thrown
///     away. Q12 gives it about 0.014 degrees of angular error, and nothing
///     compounds. b3Vec3 is fine.
///
///   - A body's orientation is integrated every substep and fed back into
///     itself. At Q12 the renormalization residue would compound into visible
///     tumbling within seconds. That is why b3Quat is Q30 throughout, and why
///     b3RotateVector works from the quaternion directly rather than building
///     an intermediate rotation matrix that would narrow it to Q12.
///
/// b3Matrix3 stays Q12, because its main job is holding inertia tensors,
/// whose entries are not bounded by 1 and so cannot live in Q30.

#include "b3fixed.h"

// Upstream gets these from base.h. Defining them here when absent keeps this
// header usable on its own, which is what lets the math tests build without
// dragging in the rest of the library.
#ifndef B3_INLINE
#define B3_INLINE static inline
#endif

#ifndef B3_API
#define B3_API
#endif

// =========================================================================
// Types
// =========================================================================

/// A 3D vector in length units (Q12). Positions, velocities, impulses
/// expressed as lengths, and contact normals.
typedef struct b3Vec3
{
	b3f x, y, z;
} b3Vec3;

/// A unit direction in Q30. Used where a direction is integrated or
/// renormalized rather than recomputed -- principally the vector part of a
/// quaternion.
typedef struct b3Dir3
{
	b3n x, y, z;
} b3Dir3;

/// An impulse vector in Q15.16.
///
/// The third vector scale, and it exists for the same reason b3Dir3 does: an
/// impulse accumulator is *warm started*, meaning it is written back into
/// itself every sub-step and again every step, so a coarse representation
/// compounds instead of averaging out. Contact impulses are also small -- a
/// resting box exchanges hundredths of a unit per sub-step -- and Q12 leaves
/// such a value only a handful of bits.
///
/// b3imp has been the port's impulse scale since Phase 1; this is the vector
/// form the contact solver needs. Q15.16 reaches 32768 with a resolution of
/// 1.5e-5, against Q12's 524288 and 2.4e-4.
typedef struct b3Imp3
{
	b3imp x, y, z;
} b3Imp3;

/// A quaternion. Q30 throughout: this is the value the solver integrates
/// every substep, so it is the one place drift would compound.
typedef struct b3Quat
{
	b3Dir3 v;
	b3n s;
} b3Quat;

/// A 2D vector of impulses, for the two tangent components of central
/// friction. There is no general 2D vector type here and no reason for one --
/// the tangent plane is the only place the solver leaves three dimensions.
typedef struct b3Imp2
{
	b3imp x, y;
} b3Imp2;

/// A symmetric 2x2 in Q12, the inverse of a tangent-plane effective mass.
///
/// Only the three distinct entries are stored. Central friction is the sole
/// consumer, and it only ever forms `m * v`, so a full four-entry matrix would
/// be a fourth of the storage and none of the generality.
typedef struct b3SymMatrix2
{
	b3f xx, yy, xy;
} b3SymMatrix2;

/// The same shape at Q24, as the *un*-inverted effective mass is built: its
/// entries are sums of inverse masses and inverse-inertia quadratic forms, so
/// they carry the inverse-weight scale exactly as b3MatrixW's do.
typedef struct b3SymMatrix2W
{
	b3iw xx, yy, xy;
} b3SymMatrix2W;

/// A rigid transform.
typedef struct b3Transform
{
	b3Vec3 p;
	b3Quat q;
} b3Transform;

/// A 3x3 column-major matrix in Q12. Holds inertia tensors, whose entries are
/// unbounded, so it cannot use the Q30 direction scale.
typedef struct b3Matrix3
{
	b3Vec3 cx, cy, cz;
} b3Matrix3;

/// A vector of inverse weights (Q24).
typedef struct b3Vec3W
{
	b3iw x, y, z;
} b3Vec3W;

/// A 3x3 column-major matrix in Q24. This is the *inverse* inertia tensor, and
/// it needs its own scale for the same reason b3iw exists at all.
///
/// A forward inertia entry is a mass times a length squared and is unbounded
/// above, which is what b3Matrix3's Q12 is for. Its inverse runs the other
/// way: a 1000-unit body has entries near 0.001, where Q12 leaves four bits
/// and Q24 leaves fourteen thousand. The tensor is the solver's divisor -- it
/// converts an angular impulse into an angular velocity every substep -- so
/// those bits are the ones that decide whether a stack settles or creeps.
///
/// Q7.24 tops out at 128, so a body whose rotational inertia falls below
/// 1/128 has its inverse scaled down uniformly rather than wrapped. See
/// b3InvertInertia.
typedef struct b3MatrixW
{
	b3Vec3W cx, cy, cz;
} b3MatrixW;

/// Axis aligned bounding box.
typedef struct b3AABB
{
	b3Vec3 lowerBound;
	b3Vec3 upperBound;
} b3AABB;

/// A plane. separation = dot(normal, point) - offset
typedef struct b3Plane
{
	b3Vec3 normal;
	b3f offset;
} b3Plane;

/// In single precision mode upstream aliases these; the port has no large
/// world mode, so they alias unconditionally.
typedef b3Vec3 b3Pos;
typedef b3Transform b3WorldTransform;

// =========================================================================
// Constructors and constants
// =========================================================================

B3_INLINE b3Vec3 b3MakeVec3( b3f x, b3f y, b3f z )
{
	b3Vec3 v = { x, y, z };
	return v;
}

/// Build a vector from whole units. The constant-friendly path.
B3_INLINE b3Vec3 b3Vec3FromInts( int32_t x, int32_t y, int32_t z )
{
	b3Vec3 v = { b3fFromInt( x ), b3fFromInt( y ), b3fFromInt( z ) };
	return v;
}

B3_INLINE b3Dir3 b3MakeDir3( b3n x, b3n y, b3n z )
{
	b3Dir3 d = { x, y, z };
	return d;
}

B3_INLINE b3Vec3 b3Vec3_zeroFn( void )
{
	b3Vec3 v = { b3f_zero, b3f_zero, b3f_zero };
	return v;
}

B3_INLINE b3Quat b3Quat_identityFn( void )
{
	b3Quat q;
	q.v.x = b3n_zero;
	q.v.y = b3n_zero;
	q.v.z = b3n_zero;
	q.s = b3n_one;
	return q;
}

B3_INLINE b3Transform b3Transform_identityFn( void )
{
	b3Transform t;
	t.p = b3Vec3_zeroFn();
	t.q = b3Quat_identityFn();
	return t;
}

// Upstream declares these as `static const` globals. In strict mode the
// scales are structs, so brace-initializing them portably is awkward; these
// macros keep every use site reading the same as upstream.
#define b3Vec3_zero b3Vec3_zeroFn()
#define b3Quat_identity b3Quat_identityFn()
#define b3Transform_identity b3Transform_identityFn()
#define b3Pos_zero b3Vec3_zeroFn()
#define b3WorldTransform_identity b3Transform_identityFn()

B3_INLINE b3Vec3 b3Vec3_axisXFn( void )
{
	return b3MakeVec3( b3f_one, b3f_zero, b3f_zero );
}

B3_INLINE b3Vec3 b3Vec3_axisYFn( void )
{
	return b3MakeVec3( b3f_zero, b3f_one, b3f_zero );
}

B3_INLINE b3Vec3 b3Vec3_axisZFn( void )
{
	return b3MakeVec3( b3f_zero, b3f_zero, b3f_one );
}

#define b3Vec3_axisX b3Vec3_axisXFn()
#define b3Vec3_axisY b3Vec3_axisYFn()
#define b3Vec3_axisZ b3Vec3_axisZFn()

// =========================================================================
// Integer helpers
// =========================================================================

B3_INLINE int b3MinInt( int a, int b )
{
	return a < b ? a : b;
}

B3_INLINE int b3MaxInt( int a, int b )
{
	return a > b ? a : b;
}

B3_INLINE int b3ClampInt( int a, int lower, int upper )
{
	return a < lower ? lower : ( upper < a ? upper : a );
}

// =========================================================================
// Scalar helpers
// =========================================================================
//
// Upstream names these ...Float. The names are kept so call sites match, even
// though nothing here is a float any more.

B3_INLINE b3f b3AbsFloat( b3f a )
{
	return b3AbsF( a );
}

B3_INLINE b3f b3MinFloat( b3f a, b3f b )
{
	return b3MinF( a, b );
}

B3_INLINE b3f b3MaxFloat( b3f a, b3f b )
{
	return b3MaxF( a, b );
}

B3_INLINE b3f b3ClampFloat( b3f a, b3f lower, b3f upper )
{
	return b3ClampF( a, lower, upper );
}

/// Upstream guards against NaN and infinity here. Fixed point has neither, so
/// the meaningful check is that a value is inside the range where Q12 still
/// has useful resolution -- which is also what B3_HUGE tests for.
B3_INLINE bool b3IsValidFloat( b3f a )
{
	return b3Raw( a ) > -( INT32_MAX / 2 ) && b3Raw( a ) < ( INT32_MAX / 2 );
}

B3_INLINE b3f b3Sign( b3f a )
{
	return b3Raw( a ) < 0 ? b3NegF( b3f_one ) : b3f_one;
}

/// Linear interpolation, t in [0, 1] as a Q30 coefficient.
B3_INLINE b3f b3LerpFloat( b3f a, b3f b, b3c t )
{
	return b3AddF( a, b3MulFC( b3SubF( b, a ), t ) );
}

// =========================================================================
// Vector arithmetic
// =========================================================================

B3_INLINE b3Vec3 b3Add( b3Vec3 a, b3Vec3 b )
{
	return b3MakeVec3( b3AddF( a.x, b.x ), b3AddF( a.y, b.y ), b3AddF( a.z, b.z ) );
}

B3_INLINE b3Vec3 b3Sub( b3Vec3 a, b3Vec3 b )
{
	return b3MakeVec3( b3SubF( a.x, b.x ), b3SubF( a.y, b.y ), b3SubF( a.z, b.z ) );
}

B3_INLINE b3Vec3 b3Neg( b3Vec3 a )
{
	return b3MakeVec3( b3NegF( a.x ), b3NegF( a.y ), b3NegF( a.z ) );
}

B3_INLINE b3Vec3 b3Abs( b3Vec3 a )
{
	return b3MakeVec3( b3AbsF( a.x ), b3AbsF( a.y ), b3AbsF( a.z ) );
}

B3_INLINE b3Vec3 b3Min( b3Vec3 a, b3Vec3 b )
{
	return b3MakeVec3( b3MinF( a.x, b.x ), b3MinF( a.y, b.y ), b3MinF( a.z, b.z ) );
}

B3_INLINE b3Vec3 b3Max( b3Vec3 a, b3Vec3 b )
{
	return b3MakeVec3( b3MaxF( a.x, b.x ), b3MaxF( a.y, b.y ), b3MaxF( a.z, b.z ) );
}

B3_INLINE b3Vec3 b3Clamp( b3Vec3 v, b3Vec3 a, b3Vec3 b )
{
	return b3MakeVec3( b3ClampF( v.x, a.x, b.x ), b3ClampF( v.y, a.y, b.y ), b3ClampF( v.z, a.z, b.z ) );
}

/// Scale a vector by a length-scaled scalar.
B3_INLINE b3Vec3 b3MulSV( b3f s, b3Vec3 v )
{
	return b3MakeVec3( b3MulFF( s, v.x ), b3MulFF( s, v.y ), b3MulFF( s, v.z ) );
}

/// Scale a vector by a dimensionless coefficient. Preferred over b3MulSV when
/// the scalar is genuinely dimensionless, because Q30 keeps far more of it.
B3_INLINE b3Vec3 b3MulCV( b3c s, b3Vec3 v )
{
	return b3MakeVec3( b3MulFC( v.x, s ), b3MulFC( v.y, s ), b3MulFC( v.z, s ) );
}

/// Scale a vector by an inverse weight (Q24). This is what normalization uses,
/// because a reciprocal length does not fit the Q30 coefficient scale -- see
/// b3RsqrtWide.
B3_INLINE b3Vec3 b3MulWV( b3iw s, b3Vec3 v )
{
	return b3MakeVec3( b3MulFW( v.x, s ), b3MulFW( v.y, s ), b3MulFW( v.z, s ) );
}

/// Scale a vector by a time, keeping the time at Q24.
///
/// Never narrow a sub-step to Q12 first. h at 240 Hz is 69905 raw at Q24 and
/// **17** at Q12 -- a 0.39% loss, and it is a loss in one direction, so a
/// velocity increment built that way runs 0.67 quanta low every sub-step and
/// 160 quanta low after one second of falling. That is Phase 1 finding 2
/// exactly, and it is why b3MulFT exists; this is its vector form.
B3_INLINE b3Vec3 b3MulVT( b3Vec3 v, b3t h )
{
	return b3MakeVec3( b3MulFT( v.x, h ), b3MulFT( v.y, h ), b3MulFT( v.z, h ) );
}

/// a + s * b
B3_INLINE b3Vec3 b3MulAdd( b3Vec3 a, b3f s, b3Vec3 b )
{
	return b3MakeVec3( b3AddF( a.x, b3MulFF( s, b.x ) ), b3AddF( a.y, b3MulFF( s, b.y ) ), b3AddF( a.z, b3MulFF( s, b.z ) ) );
}

/// a - s * b
B3_INLINE b3Vec3 b3MulSub( b3Vec3 a, b3f s, b3Vec3 b )
{
	return b3MakeVec3( b3SubF( a.x, b3MulFF( s, b.x ) ), b3SubF( a.y, b3MulFF( s, b.y ) ), b3SubF( a.z, b3MulFF( s, b.z ) ) );
}

/// Componentwise product.
B3_INLINE b3Vec3 b3Mul( b3Vec3 a, b3Vec3 b )
{
	return b3MakeVec3( b3MulFF( a.x, b.x ), b3MulFF( a.y, b.y ), b3MulFF( a.z, b.z ) );
}

B3_INLINE b3Vec3 b3Lerp( b3Vec3 a, b3Vec3 b, b3c t )
{
	return b3MakeVec3( b3LerpFloat( a.x, b.x, t ), b3LerpFloat( a.y, b.y, t ), b3LerpFloat( a.z, b.z, t ) );
}

// =========================================================================
// Dot, cross and length
// =========================================================================
//
// The dot product is the one place a naive port loses badly. Narrowing each
// term to Q12 before summing throws away the low bits of all three and can
// overflow on large vectors -- which is what NEA_Vec3LengthSq in
// NEACollision.h does today. Accumulating wide and narrowing once is both
// more accurate and cheaper: three SMLALs and one shift.

/// Dot product accumulated in an int64 at Q24, before narrowing. Callers that
/// need the extra range (squared lengths, GJK) should use this directly.
B3_INLINE int64_t b3DotWide( b3Vec3 a, b3Vec3 b )
{
	return (int64_t)b3Raw( a.x ) * b3Raw( b.x ) + (int64_t)b3Raw( a.y ) * b3Raw( b.y ) + (int64_t)b3Raw( a.z ) * b3Raw( b.z );
}

B3_INLINE b3f b3Dot( b3Vec3 a, b3Vec3 b )
{
	return b3Makeb3fRef( B3_SHIFT_ROUND( b3DotWide( a, b ), B3_F_SHIFT ),
						 B3_REF( b3RefF( a.x ) * b3RefF( b.x ) + b3RefF( a.y ) * b3RefF( b.y ) + b3RefF( a.z ) * b3RefF( b.z ) ) );
}

/// Defined in b3hot.c, which explains why this and its neighbours are not
/// inline.
B3_API b3Vec3 b3Cross( b3Vec3 a, b3Vec3 b );

/// Cross product kept wide: three int64 components at Q24.
///
/// b3Cross narrows each component back to Q12, and for a cross product that
/// is a real hazard rather than a rounding detail. The magnitude of a cross
/// product is an **area**, so two short edges produce a result two orders
/// smaller than either of them: the cross of two 0.05-unit vectors is 0.0025,
/// which is ten raw units in Q12, and of two 0.01-unit vectors it is *zero*.
/// A caller that only wants the direction has then lost it entirely.
///
/// Pair this with b3DirectionFromWide when the direction is what matters.
B3_INLINE void b3CrossWide( int64_t out[3], b3Vec3 a, b3Vec3 b )
{
	out[0] = (int64_t)b3Raw( a.y ) * b3Raw( b.z ) - (int64_t)b3Raw( a.z ) * b3Raw( b.y );
	out[1] = (int64_t)b3Raw( a.z ) * b3Raw( b.x ) - (int64_t)b3Raw( a.x ) * b3Raw( b.z );
	out[2] = (int64_t)b3Raw( a.x ) * b3Raw( b.y ) - (int64_t)b3Raw( a.y ) * b3Raw( b.x );
}

/// A Q12 vector along (x, y, z), rescaled so it is about a unit long.
///
/// For a quantity whose direction is all that is ever read -- a search
/// direction, a face normal before normalization, a separating axis -- the
/// magnitude is not information, and carrying it costs range at both ends. A
/// cross product of short vectors underflows to nothing; one of long vectors
/// overflows Q12. Rescaling to a fixed magnitude removes both, and it is
/// exact in the only thing being asked for, since scaling a vector by a
/// positive constant changes neither its direction nor the outcome of any
/// support query or normalization made from it.
///
/// Returns the zero vector only when the input is exactly zero, which is the
/// one case where there genuinely is no direction.
///
/// The scale is arbitrary. It is not a length and must not be read as one.
B3_API b3Vec3 b3DirectionFromWide( int64_t x, int64_t y, int64_t z );

/// Cross product reduced straight to a unit-ish Q12 direction.
B3_INLINE b3Vec3 b3CrossDirection( b3Vec3 a, b3Vec3 b )
{
	int64_t c[3];
	b3CrossWide( c, a, b );
	return b3DirectionFromWide( c[0], c[1], c[2] );
}

/// Upstream's b3ModifiedCross: a cross product with every minus turned into a
/// plus, so each component is a *sum* of magnitudes rather than a difference.
///
/// It is not a cross product and has no geometric meaning on its own. Both
/// callers want the same thing -- an upper bound on how far a point at
/// `extent` from the centre sweeps under a small rotation. cross(theta, r)
/// bounds that arc, and taking every term positive bounds it without needing
/// to know the signs.
///
/// Two overloads, because the two callers hold the rotation at different
/// scales, and upstream's single float signature hides the difference:
///
///   - b3ModifiedCrossNF -- the contact-recycling bound in b3CollideTask,
///     where `a` is the vector part of a relative quaternion and is therefore
///     a b3Dir3 at Q30.
///   - b3ModifiedCrossFF -- the sleep velocity in b3FinalizeBodiesTask, where
///     `a` is a local angular velocity (or a delta-rotation vector already
///     brought back through b3InvRotateVector) and is a b3Vec3 at Q12.
///
/// Either way `b` is a body extent, a length at Q12, and the result is a
/// length -- which is what both callers compare against.
B3_INLINE b3Vec3 b3ModifiedCrossNF( b3Dir3 a, b3Vec3 b )
{
	return b3MakeVec3( b3AddF( b3MulFN( b.z, a.y ), b3MulFN( b.y, a.z ) ),
					   b3AddF( b3MulFN( b.x, a.z ), b3MulFN( b.z, a.x ) ),
					   b3AddF( b3MulFN( b.y, a.x ), b3MulFN( b.x, a.y ) ) );
}

/// @copydoc b3ModifiedCrossNF
B3_INLINE b3Vec3 b3ModifiedCrossFF( b3Vec3 a, b3Vec3 b )
{
	return b3MakeVec3( b3AddF( b3MulFF( a.y, b.z ), b3MulFF( a.z, b.y ) ),
					   b3AddF( b3MulFF( a.z, b.x ), b3MulFF( a.x, b.z ) ),
					   b3AddF( b3MulFF( a.x, b.y ), b3MulFF( a.y, b.x ) ) );
}

/// Componentwise absolute value of a Q30 direction.
B3_INLINE b3Dir3 b3AbsDir( b3Dir3 a )
{
	return b3MakeDir3( b3AbsN( a.x ), b3AbsN( a.y ), b3AbsN( a.z ) );
}

B3_INLINE b3f b3LengthSquared( b3Vec3 v )
{
	return b3Dot( v, v );
}

/// Exact squared length, never narrowed. Comparisons against a squared
/// threshold should use this rather than b3LengthSquared, which can overflow
/// Q12 for vectors longer than about 512 units.
B3_INLINE int64_t b3LengthSquaredWide( b3Vec3 v )
{
	return b3DotWide( v, v );
}

B3_INLINE b3f b3Length( b3Vec3 v )
{
	return b3SqrtWide( b3LengthSquaredWide( v ) );
}

B3_INLINE b3f b3DistanceSquared( b3Vec3 a, b3Vec3 b )
{
	return b3LengthSquared( b3Sub( a, b ) );
}

B3_INLINE b3f b3Distance( b3Vec3 a, b3Vec3 b )
{
	return b3Length( b3Sub( a, b ) );
}

/// Normalize, returning the zero vector for a degenerate input.
///
/// One reciprocal square root and three multiplies, rather than a root
/// followed by three divides -- the divider is the expensive unit on this
/// machine, so trading two of the three away is worth it.
B3_API b3Vec3 b3Normalize( b3Vec3 v );

/// Normalize and report the original length in one pass, so callers that need
/// both do not pay for two square roots.
B3_API b3Vec3 b3GetLengthAndNormalize( b3f* length, b3Vec3 v );

/// Normalize to a Q30 direction. Used where the result feeds a quaternion,
/// which is the path where precision has to survive being integrated.
B3_API b3Dir3 b3NormalizeToDir( b3Vec3 v );

/// Widen a Q12 vector to a Q30 direction without renormalizing. Valid only
/// when the vector is already unit length.
B3_INLINE b3Dir3 b3ToDir3( b3Vec3 v )
{
	b3Dir3 d = { b3FToN( v.x ), b3FToN( v.y ), b3FToN( v.z ) };
	return d;
}

/// A length projected onto a Q30 unit direction. The b3Dot the contact solver
/// wants, since its frame is Q30 and everything it measures is Q12.
B3_INLINE b3f b3DotDirF( b3Dir3 d, b3Vec3 v )
{
	return b3AddF( b3AddF( b3MulFN( v.x, d.x ), b3MulFN( v.y, d.y ) ), b3MulFN( v.z, d.z ) );
}

/// cross( v, d ) with the right operand a Q30 direction. Q12 in, Q12 out.
///
/// The lever-arm-against-frame product: `cross( r, normal )` and its two
/// tangent counterparts, which are what the effective masses are built from.
B3_INLINE b3Vec3 b3CrossDirRight( b3Vec3 v, b3Dir3 d )
{
	return b3MakeVec3( b3SubF( b3MulFN( v.y, d.z ), b3MulFN( v.z, d.y ) ),
					   b3SubF( b3MulFN( v.z, d.x ), b3MulFN( v.x, d.z ) ),
					   b3SubF( b3MulFN( v.x, d.y ), b3MulFN( v.y, d.x ) ) );
}

/// Dot product of two Q12 vectors, kept at **Q24** rather than narrowed back.
///
/// b3Dot narrows, which is right when both operands are lengths and so is the
/// answer. It is wrong for the solver's effective masses: `dot( rn, I*rn )` is
/// a length times an angular-acceleration-per-impulse, and the product is an
/// *inverse mass*, which belongs at Q24 alongside b3BodySim::invMass rather
/// than at Q12 with four bits left of a 0.001 entry.
///
/// No shift is involved -- the raw product of two Q12 values already is the
/// Q24 result -- so this is the natural scale and b3Dot is the lossy one.
B3_INLINE b3iw b3DotVWide( b3Vec3 a, b3Vec3 b )
{
	int64_t sum = (int64_t)b3Raw( a.x ) * b3Raw( b.x ) + (int64_t)b3Raw( a.y ) * b3Raw( b.y ) +
				  (int64_t)b3Raw( a.z ) * b3Raw( b.z );
	return b3Makeb3iw( (int32_t)sum );
}

B3_INLINE b3Vec3 b3FromDir3( b3Dir3 d )
{
	return b3MakeVec3( b3NToF( d.x ), b3NToF( d.y ), b3NToF( d.z ) );
}

/// True if the vector is unit length to within a few Q12 quanta.
B3_INLINE bool b3IsNormalized( b3Vec3 v )
{
	int64_t sq = b3LengthSquaredWide( v );
	int64_t one = (int64_t)1 << ( 2 * B3_F_SHIFT );
	int64_t tol = one / 256;
	int64_t diff = sq - one;
	return ( diff < 0 ? -diff : diff ) < tol;
}

/// A vector perpendicular to v. Picks the axis least aligned with v so the
/// cross product never degenerates.
B3_INLINE b3Vec3 b3Perp( b3Vec3 v )
{
	if ( b3Raw( b3AbsF( v.x ) ) <= b3Raw( b3AbsF( v.y ) ) && b3Raw( b3AbsF( v.x ) ) <= b3Raw( b3AbsF( v.z ) ) )
	{
		return b3Normalize( b3Cross( v, b3Vec3_axisXFn() ) );
	}
	if ( b3Raw( b3AbsF( v.y ) ) <= b3Raw( b3AbsF( v.z ) ) )
	{
		return b3Normalize( b3Cross( v, b3Vec3_axisYFn() ) );
	}
	return b3Normalize( b3Cross( v, b3Vec3_axisZFn() ) );
}

/// A vector perpendicular to a *unit* vector, never short.
///
/// b3Perp crosses against an axis, so its result is as short as the sine of
/// the angle between them -- fine when the caller only wants a direction,
/// poor when the result is about to be normalized in Q12, where a short
/// vector has already lost its low bits. This construction keeps the result
/// between about 0.39 and 0.79 units whatever the input, at the cost of
/// requiring the input to be unit length.
///
/// Falls back to b3Perp when no component exceeds a half, which for a genuine
/// unit vector cannot happen -- one component is always at least 1/sqrt(3).
B3_API b3Vec3 b3ArbitraryPerp( b3Vec3 v );

B3_INLINE bool b3IsValidVec3( b3Vec3 v )
{
	return b3IsValidFloat( v.x ) && b3IsValidFloat( v.y ) && b3IsValidFloat( v.z );
}

// =========================================================================
// Quaternions
// =========================================================================
//
// Everything here stays in Q30. This is the value the solver feeds back into
// itself every substep, so it is the one place where shedding bits compounds
// instead of washing out.

B3_INLINE b3Quat b3MakeQuat( b3n x, b3n y, b3n z, b3n s )
{
	b3Quat q;
	q.v.x = x;
	q.v.y = y;
	q.v.z = z;
	q.s = s;
	return q;
}

B3_INLINE b3Quat b3NegateQuat( b3Quat q )
{
	return b3MakeQuat( b3NegN( q.v.x ), b3NegN( q.v.y ), b3NegN( q.v.z ), b3NegN( q.s ) );
}

B3_INLINE b3Quat b3Conjugate( b3Quat q )
{
	return b3MakeQuat( b3NegN( q.v.x ), b3NegN( q.v.y ), b3NegN( q.v.z ), q.s );
}

B3_INLINE b3n b3DotQuat( b3Quat a, b3Quat b )
{
	return b3AddN( b3AddN( b3MulNN( a.v.x, b.v.x ), b3MulNN( a.v.y, b.v.y ) ),
				   b3AddN( b3MulNN( a.v.z, b.v.z ), b3MulNN( a.s, b.s ) ) );
}

/// Exact squared magnitude of a quaternion, at Q60 in an int64.
///
/// Kept wide because normalization divides by its root, and a Q30 narrowing
/// here would put a floor on how accurately a quaternion can be renormalized
/// -- which is exactly the drift this scale exists to prevent.
B3_INLINE int64_t b3QuatLengthSquaredWide( b3Quat q )
{
	return (int64_t)b3Raw( q.v.x ) * b3Raw( q.v.x ) + (int64_t)b3Raw( q.v.y ) * b3Raw( q.v.y ) +
		   (int64_t)b3Raw( q.v.z ) * b3Raw( q.v.z ) + (int64_t)b3Raw( q.s ) * b3Raw( q.s );
}

/// Renormalize a quaternion. Called every substep after integration.
///
/// The magnitude is computed at Q60, rooted to Q30, then each component is
/// divided by it at Q30. That keeps the whole operation at the quaternion's
/// own scale; going through Q12 anywhere would cap renormalization accuracy
/// at 12 bits and let orientation drift accumulate.
B3_API b3Quat b3NormalizeQuat( b3Quat q );

B3_INLINE bool b3IsNormalizedQuat( b3Quat q )
{
	int64_t sq = b3QuatLengthSquaredWide( q );
	int64_t one = (int64_t)1 << ( 2 * B3_N_SHIFT );
	int64_t tol = one / 1024;
	int64_t diff = sq - one;
	return ( diff < 0 ? -diff : diff ) < tol;
}

/// Hamilton product.
B3_API b3Quat b3MulQuat( b3Quat a, b3Quat b );

/// conjugate(a) * b
B3_INLINE b3Quat b3InvMulQuat( b3Quat a, b3Quat b )
{
	return b3MulQuat( b3Conjugate( a ), b );
}

/// Rotate a vector by a quaternion.
///
/// Uses v' = v + 2*s*(u x v) + 2*(u x (u x v)) with u the vector part,
/// rather than building a rotation matrix first. The matrix route would
/// narrow the Q30 quaternion to Q12 before touching the vector and throw away
/// 18 bits of the orientation for no reason.
B3_API b3Vec3 b3RotateVector( b3Quat q, b3Vec3 v );

/// Rotate a vector by the inverse of a quaternion.
B3_INLINE b3Vec3 b3InvRotateVector( b3Quat q, b3Vec3 v )
{
	return b3RotateVector( b3Conjugate( q ), v );
}

/// Build a quaternion from a unit axis and an angle in brads.
///
/// The half-angle sine and cosine come from the libnds LUT, which is 4.12, so
/// the result carries about 12 bits of trig accuracy. That is acceptable here
/// and only here: this runs when a rotation is *authored*, and the
/// normalization below absorbs the residue. Nothing in the substep loop calls
/// it, so the coarse LUT never gets a chance to accumulate.
B3_INLINE b3Quat b3MakeQuatFromAxisAngle( b3Vec3 axis, b3a angle )
{
	b3a half = (b3a)( angle >> 1 );
	b3c s = b3SinA( half );
	b3c c = b3CosA( half );

	b3Dir3 u = b3NormalizeToDir( axis );

	b3Quat q;
	q.v.x = b3CToN( b3Makeb3c( (int32_t)( ( (int64_t)b3Raw( u.x ) * b3Raw( s ) ) >> B3_C_SHIFT ) ) );
	q.v.y = b3CToN( b3Makeb3c( (int32_t)( ( (int64_t)b3Raw( u.y ) * b3Raw( s ) ) >> B3_C_SHIFT ) ) );
	q.v.z = b3CToN( b3Makeb3c( (int32_t)( ( (int64_t)b3Raw( u.z ) * b3Raw( s ) ) >> B3_C_SHIFT ) ) );
	q.s = b3CToN( c );

	return b3NormalizeQuat( q );
}

/// Advance an orientation by an angular velocity over one substep.
///
/// This is the rigid body rotation update, q += 0.5 * w * q * h followed by
/// renormalization, and it is the single hottest use of quaternion math in
/// the engine -- it runs for every awake body, every substep.
///
/// It is also the operation with the least margin for error, which is why it
/// is a function rather than something each caller open-codes. The half-angle
/// increment is tiny (about 0.002 rad at 1 rad/s and 240 Hz) and it is fed
/// back into the orientation, so any truncation compounds instead of
/// cancelling. Routing it through b3MulFTToN keeps the increment at Q30 from
/// beginning to end; the same computation via Q12 loses 6% per substep and
/// puts a body 36 degrees off after ten seconds.
///
/// @param q Current orientation, unit length.
/// @param omega Angular velocity in radians per second (Q12).
/// @param h Substep duration (Q24).
B3_API b3Quat b3IntegrateRotation( b3Quat q, b3Vec3 omega, b3t h );

/// Total rotation angle of a quaternion, in brads. `2 * atan2( |v|, s )`.
///
/// @section acos Why this is not `2 * acos( s )`
///
/// It was, through Stage 6, and it had **no callers** -- Stage 7's separation
/// queries would have been the first, at exactly the angles where the acos form
/// is worst. Two independent reasons to take the arc tangent instead, and
/// either alone would be enough:
///
/// **1. The acos form cannot resolve a small angle.** b3AcosC narrows its Q30
/// argument to Q12 before the lookup, and near identity the arc cosine's slope
/// is unbounded: with `s = cos(theta/2)`,
///
///     dtheta / ds = 2 / sin( theta / 2 )
///
/// so one Q12 quantum of `s` becomes `2 * (1/4096) / sin(theta/2)` of angle.
/// At a 128-brad rotation that is 208 brads of error -- larger than the angle
/// being measured. Measured: the acos form returns **230 for a true 128**, an
/// 80% error, against 126 for this one. The arc tangent has no such term
/// because its two arguments shrink together; measured error is flat at +-2
/// brads from 128 brads all the way to a half turn.
///
/// **2. It must not assume a unit quaternion.** The separation queries strip a
/// degree of freedom by zeroing a component -- `relQ.v.z = 0` -- and then take
/// the angle of what is left. That is meaningful here, because `|v|` shortens.
/// Against `2 * acos( s )` it is a **silent no-op**: the scalar part is
/// untouched, so the query would return the full rotation angle while looking
/// like it had removed a component. Upstream is the arc tangent form for this
/// reason, and transliterating its callers onto an acos would be a bug that
/// compiles, runs, and returns a plausible number.
///
/// @section scale Why the root is not narrowed
///
/// The same argument b3GetSwingAngle makes: squaring Q30 components gives Q60,
/// which int64 holds exactly, and the integer square root of a Q60 value is
/// itself Q30. Passing both arguments to b3Atan2Raw at Q30 keeps thirty bits
/// where narrowing to Q12 would put a 2.5-brad floor under the smallest angle
/// that could be reported -- coarser than the b3a being returned.
B3_INLINE b3a b3GetQuatAngle( b3Quat q )
{
	int64_t vSquared = (int64_t)b3Raw( q.v.x ) * b3Raw( q.v.x ) + (int64_t)b3Raw( q.v.y ) * b3Raw( q.v.y ) +
					   (int64_t)b3Raw( q.v.z ) * b3Raw( q.v.z );

	int32_t y = (int32_t)b3HwSqrt64( (uint64_t)vSquared );
	int32_t x = b3Raw( q.s );

	// The scalar part carries a sign and the vector length does not, so a
	// negative scalar puts the result in the second quadrant -- which is right:
	// it is the same rotation taken the long way round, and doubling it gives an
	// angle past a half turn. Fold the double cover the way b3GetTwistAngle
	// does, by negating, so the result stays in [0, half a turn].
	if ( x < 0 )
	{
		x = -x;
	}

	// Both arguments are non-negative, so b3Atan2Raw returns [0, quarter turn]
	// and the doubled result cannot leave [0, half a turn]. No clamp is needed,
	// unlike b3GetTwistAngle where the half turn is reachable from both sides.
	return (b3a)( 2 * (int32_t)b3Atan2Raw( y, x ) );
}

/// Rotation angle about the quaternion's own z axis, in brads.
///
/// The hinge angle. Unlike b3GetQuatAngle, which reports how far a rotation
/// turns about whatever axis it uses, this reports only the *twist* component
/// -- which for a revolute joint's relative rotation is the one degree of
/// freedom the joint leaves open, and the quantity its limits and motor act on.
///
/// Negating both arguments when the scalar part is negative is upstream's
/// trick and worth keeping: a quaternion and its negation are the same
/// rotation, so this folds the double cover into a result that stays in
/// [-pi, pi] without the caller having to unwind anything.
B3_INLINE b3a b3GetTwistAngle( b3Quat q )
{
	b3f z = b3NToF( q.v.z );
	b3f s = b3NToF( q.s );

	b3a half = b3Raw( q.s ) < 0 ? b3Atan2F( b3NegF( z ), b3NegF( s ) ) : b3Atan2F( z, s );

	// Doubled in int32 before narrowing: 2 * half can reach exactly +-32768,
	// which is one past what a b3a holds, and it is the half-turn case rather
	// than an unreachable corner.
	int32_t twist = 2 * (int32_t)half;
	if ( twist > 32767 )
	{
		twist = 32767;
	}
	if ( twist < -32768 )
	{
		twist = -32768;
	}
	return (b3a)twist;
}

/// Swing angle of a quaternion, in brads: how far its z axis has tilted away
/// from the reference z axis, ignoring any twist about it.
///
/// The complement of b3GetTwistAngle, and the quantity a spherical joint's cone
/// limit acts on. Always non-negative and never more than half a turn, so
/// unlike the twist there is no polarity to fold -- every term is squared.
///
/// @section scale Why the roots are not narrowed
///
/// The two arguments are sqrt(x^2 + y^2) and sqrt(z^2 + s^2). Squaring Q30
/// components gives Q60, which an int64 holds exactly for any unit quaternion,
/// and the integer square root of a Q60 value is itself Q30 -- so both roots
/// arrive with 30 bits and no rounding.
///
/// Narrowing them to Q12 to reach b3Atan2F would throw that away where it
/// matters most. A nearly aligned joint has a tiny numerator and a denominator
/// near one, and Q12's quantum of 1/4096 puts a floor of roughly 2.5 brads
/// under the smallest swing that could be reported -- coarser than the b3a this
/// returns, and the cone limit would sit blind inside that band. b3Atan2Raw
/// takes both roots at Q30 instead; an arc tangent depends only on the ratio,
/// so passing a common scale other than Q12 is exactly as correct.
B3_INLINE b3a b3GetSwingAngle( b3Quat q )
{
	int64_t xySquared = (int64_t)b3Raw( q.v.x ) * b3Raw( q.v.x ) + (int64_t)b3Raw( q.v.y ) * b3Raw( q.v.y );
	int64_t zsSquared = (int64_t)b3Raw( q.v.z ) * b3Raw( q.v.z ) + (int64_t)b3Raw( q.s ) * b3Raw( q.s );

	int32_t y = (int32_t)b3HwSqrt64( (uint64_t)xySquared );
	int32_t x = (int32_t)b3HwSqrt64( (uint64_t)zsSquared );

	// Both arguments are non-negative, so b3Atan2Raw returns [0, quarter turn]
	// and the doubled result cannot leave [0, half a turn]. No clamp is needed
	// here, unlike in b3GetTwistAngle where the half-turn case is reachable
	// from both sides.
	return (b3a)( 2 * (int32_t)b3Atan2Raw( y, x ) );
}

/// The rotation vector that carries `q` onto `target`: 2 * (target - q) * q^-1.
///
/// A small-angle rotation error, in radians, expressed as an axis scaled by the
/// angle -- which is what a rotational constraint's positional error C has to
/// be for `bias = biasRate * C` to come out as an angular velocity. Every joint
/// with a spring toward a target *orientation* needs it: the spherical joint
/// here, and the weld, motor and prismatic joints in the stages after.
///
/// Two details carry the fixed point:
///
///   - The polarity fold is upstream's and is not optional. A quaternion and
///     its negation are the same rotation but differ as four numbers, so
///     without it the difference below can come out near 2 in magnitude for two
///     orientations that are in fact identical. After the fold, dot(q, target)
///     is non-negative, which bounds |target - q| at sqrt(2) -- comfortably
///     inside b3n's range of 2, so the subtraction cannot saturate.
///   - The doubling happens **after** the narrowing to Q12, not before. The
///     product's components are bounded by sqrt(2) at Q30, but twice that is
///     2.83 and Q30 tops out at 2. At Q12 there is room to spare.
///
/// `diff` is not a unit quaternion, which b3MulQuat does not care about -- it
/// is plain Q30 arithmetic with no normalization assumed.
B3_INLINE b3Vec3 b3DeltaQuatToRotation( b3Quat q, b3Quat target )
{
	b3Quat s = q;
	if ( b3Raw( b3DotQuat( q, target ) ) < 0 )
	{
		s = b3NegateQuat( q );
	}

	b3Quat diff;
	diff.v.x = b3SubN( target.v.x, s.v.x );
	diff.v.y = b3SubN( target.v.y, s.v.y );
	diff.v.z = b3SubN( target.v.z, s.v.z );
	diff.s = b3SubN( target.s, s.s );

	b3Quat product = b3MulQuat( diff, b3Conjugate( s ) );

	b3f x = b3NToF( product.v.x );
	b3f y = b3NToF( product.v.y );
	b3f z = b3NToF( product.v.z );
	return b3MakeVec3( b3AddF( x, x ), b3AddF( y, y ), b3AddF( z, z ) );
}

B3_INLINE bool b3IsValidQuat( b3Quat q )
{
	return b3IsNormalizedQuat( q );
}

/// Normalized linear interpolation between two quaternions.
B3_API b3Quat b3NLerp( b3Quat a, b3Quat b, b3c t );

// =========================================================================
// Matrices
// =========================================================================

B3_INLINE b3Matrix3 b3MakeMatrix3( b3Vec3 cx, b3Vec3 cy, b3Vec3 cz )
{
	b3Matrix3 m = { cx, cy, cz };
	return m;
}

B3_INLINE b3Matrix3 b3Mat3_identityFn( void )
{
	return b3MakeMatrix3( b3Vec3_axisXFn(), b3Vec3_axisYFn(), b3Vec3_axisZFn() );
}

B3_INLINE b3Matrix3 b3Mat3_zeroFn( void )
{
	return b3MakeMatrix3( b3Vec3_zeroFn(), b3Vec3_zeroFn(), b3Vec3_zeroFn() );
}

#define b3Mat3_identity b3Mat3_identityFn()
#define b3Mat3_zero b3Mat3_zeroFn()

/// Matrix times vector.
B3_API b3Vec3 b3MulMV( b3Matrix3 m, b3Vec3 v );

B3_API b3Matrix3 b3MulMM( b3Matrix3 a, b3Matrix3 b );

B3_INLINE b3Matrix3 b3AddMM( b3Matrix3 a, b3Matrix3 b )
{
	return b3MakeMatrix3( b3Add( a.cx, b.cx ), b3Add( a.cy, b.cy ), b3Add( a.cz, b.cz ) );
}

B3_INLINE b3Matrix3 b3SubMM( b3Matrix3 a, b3Matrix3 b )
{
	return b3MakeMatrix3( b3Sub( a.cx, b.cx ), b3Sub( a.cy, b.cy ), b3Sub( a.cz, b.cz ) );
}

B3_INLINE b3Matrix3 b3MulSM( b3f s, b3Matrix3 m )
{
	return b3MakeMatrix3( b3MulSV( s, m.cx ), b3MulSV( s, m.cy ), b3MulSV( s, m.cz ) );
}

B3_INLINE b3Matrix3 b3NegateMat3( b3Matrix3 m )
{
	return b3MakeMatrix3( b3Neg( m.cx ), b3Neg( m.cy ), b3Neg( m.cz ) );
}

B3_INLINE b3Matrix3 b3Transpose( b3Matrix3 m )
{
	return b3MakeMatrix3( b3MakeVec3( m.cx.x, m.cy.x, m.cz.x ), b3MakeVec3( m.cx.y, m.cy.y, m.cz.y ),
						  b3MakeVec3( m.cx.z, m.cy.z, m.cz.z ) );
}

B3_INLINE b3Matrix3 b3AbsMatrix3( b3Matrix3 m )
{
	return b3MakeMatrix3( b3Abs( m.cx ), b3Abs( m.cy ), b3Abs( m.cz ) );
}

B3_INLINE b3f b3Det( b3Matrix3 m )
{
	return b3Dot( m.cx, b3Cross( m.cy, m.cz ) );
}

/// Determinant kept wide: **Q24** in an int64.
///
/// A determinant is a *triple* product, so it is three orders of magnitude
/// away from its entries in a way b3Det's Q12 result cannot express. b3Cross
/// alone already narrows once (see its comment); doing that twice and then
/// dividing by the result is how a 3x3 solve turns into noise. Every consumer
/// of a determinant here works from this.
///
/// Q24 rather than the Q36 the triple product naturally lands at: the cofactor
/// arrays it is divided into are Q24 as well (b3CrossWide's output scale), and
/// keeping both at the same scale is what makes `b3DivWideToF( cofactor, det )`
/// come out at Q12 with no correction factor. Matching the two is the whole
/// trick; mismatching them is an error of exactly 4096 and looks like a
/// completely broken inverse.
B3_INLINE int64_t b3DetWide( b3Matrix3 m )
{
	int64_t c[3];
	b3CrossWide( c, m.cy, m.cz );
	return ( c[0] * b3Raw( m.cx.x ) + c[1] * b3Raw( m.cx.y ) + c[2] * b3Raw( m.cx.z ) ) >> B3_F_SHIFT;
}

/// Solve `m * x = a` for x, by Cramer's rule. Returns zero if m is singular.
///
/// @warning Both this and b3InvertMatrix want a matrix whose entries are of
/// order one. That is not a fixed-point quibble -- a determinant is cubic in
/// the entries, so a matrix scaled by k has a determinant scaled by k^3, and
/// Q36 runs out three times as fast as Q12 does.
///
/// The one caller, the gyroscopic solve in b3IntegrateVelocitiesTask, satisfies
/// this for a reason worth stating: its equation
/// `I*(w2 - w1) + h * w2 x (I*w2) = 0` is homogeneous of degree one in I, so I
/// may be supplied at any convenient scale and the answer is unchanged. The
/// port supplies it per unit mass. That also means 3A's uniform Q7.24 clamp on
/// the inverse inertia cancels exactly rather than biasing the result.
B3_API b3Vec3 b3Solve3( b3Matrix3 m, b3Vec3 a );

/// Matrix inverse. Returns the zero matrix if m is singular.
/// @warning See b3Solve3 on the scale of the entries.
B3_API b3Matrix3 b3InvertMatrix( b3Matrix3 m );

/// Rotation matrix from a quaternion.
///
/// The result is Q12, so this narrows the orientation. That is fine for the
/// uses it has -- feeding a transform to the renderer, or an inertia
/// similarity transform -- but it is why b3RotateVector does not go through
/// here.
B3_API b3Matrix3 b3MakeMatrixFromQuat( b3Quat q );

/// The three columns of a rotation matrix, at Q30.
///
/// b3MakeMatrixFromQuat above, without the narrowing -- and the difference is
/// not cosmetic. A rotation matrix entry is bounded by one, which is precisely
/// the range b3n exists for, and every term here is a product of two Q30
/// quaternion components, so nothing is narrowed anywhere. At Q12 the columns
/// are not exactly orthonormal: for an isotropic tensor, where every
/// off-diagonal must cancel to exactly zero, the residual is about 1e-3.
///
/// Two callers, for two different reasons. b3RotateInertiaW needs it because
/// that 1e-3 is 16000 quanta at Q24 and would pin the debug checker's
/// worst-error statistic. wheel_joint.c needs it because it takes an arc
/// tangent of two dot products of these columns, and the angle's error is the
/// columns' error divided by how far the wheel is from fully tipped: at Q12
/// that is 2.6 brads upright and 333 brads (3.7 degrees) near the degenerate
/// end, while at Q30 it stays under one brad throughout.
///
/// Lived as a `static` in math_fixed.c through Phase 3; published at Stage 7
/// when the wheel joint became its second caller.
B3_API void b3QuatColumnsN( b3Quat q, b3Dir3 out[3] );

B3_INLINE bool b3IsValidMatrix3( b3Matrix3 m )
{
	return b3IsValidVec3( m.cx ) && b3IsValidVec3( m.cy ) && b3IsValidVec3( m.cz );
}

B3_INLINE b3Matrix3 b3MakeDiagonalMatrix( b3f a, b3f b, b3f c )
{
	return b3MakeMatrix3( b3MakeVec3( a, b3f_zero, b3f_zero ), b3MakeVec3( b3f_zero, b, b3f_zero ),
						  b3MakeVec3( b3f_zero, b3f_zero, c ) );
}

B3_INLINE b3Matrix3 b3Skew( b3Vec3 v )
{
	return b3MakeMatrix3( b3MakeVec3( b3f_zero, v.z, b3NegF( v.y ) ), b3MakeVec3( b3NegF( v.z ), b3f_zero, v.x ),
						  b3MakeVec3( v.y, b3NegF( v.x ), b3f_zero ) );
}

// =========================================================================
// Inertia
// =========================================================================
//
// These return inertia **per unit mass** -- the radius of gyration squared,
// in length² -- rather than absolute inertia. b3MassData carries the same
// convention; see the note on its `inertia` field.
//
// The reason is range. A sphere's absolute inertia is 0.4·m·r², and since m
// itself grows as r³, inertia grows as r⁵: in Q12 a solid sphere overflows at
// radius 13 with Box3D's default density of 1. Dividing the mass out removes
// the growth completely -- the same sphere stores 0.4·r², which is 67 rather
// than 622,000 -- so nothing that fits in the world can overflow.
//
// It also makes these simpler than upstream rather than more complex. Every
// one had the form `mass × (something in length²)`, so the mass parameter
// simply disappears and what is left is pure geometry.
//
// The one place that pays is composing a shape from parts: per-unit-mass
// inertias cannot be added, they have to be mass-weighted. See
// b3ComputeCapsuleMass, which does that in an int64 accumulator.

/// Unit inertia of a solid sphere: 0.4 r².
B3_INLINE b3Matrix3 b3SphereUnitInertia( b3f radius )
{
	b3f i = b3MulFC( b3MulFF( radius, radius ), b3cFromFrac( 2, 5 ) );
	return b3MakeDiagonalMatrix( i, i, i );
}

/// Unit inertia of a solid cylinder aligned with the Y axis.
/// ixx = izz = (3r² + h²)/12, iyy = r²/2.
B3_INLINE b3Matrix3 b3CylinderUnitInertia( b3f radius, b3f height )
{
	b3f rr = b3MulFF( radius, radius );
	b3f hh = b3MulFF( height, height );

	b3f ixx = b3MulFC( b3AddF( b3MulFF( rr, b3fFromInt( 3 ) ), hh ), b3cFromFrac( 1, 12 ) );
	b3f iyy = b3MulFC( rr, b3cFromFrac( 1, 2 ) );

	return b3MakeDiagonalMatrix( ixx, iyy, ixx );
}

/// Unit inertia of a solid box spanning min..max.
B3_INLINE b3Matrix3 b3BoxUnitInertia( b3Vec3 min, b3Vec3 max )
{
	b3Vec3 d = b3Sub( max, min );
	b3f xx = b3MulFF( d.x, d.x );
	b3f yy = b3MulFF( d.y, d.y );
	b3f zz = b3MulFF( d.z, d.z );

	b3c twelfth = b3cFromFrac( 1, 12 );
	return b3MakeDiagonalMatrix( b3MulFC( b3AddF( yy, zz ), twelfth ), b3MulFC( b3AddF( xx, zz ), twelfth ),
								 b3MulFC( b3AddF( xx, yy ), twelfth ) );
}

/// Parallel axis theorem, per unit mass, with the mass factored out:
///
///     I_offset = |d|² * Identity - d (x) d
///
/// The outer product is what makes this **not** a diagonal matrix. An earlier
/// version of this function returned only the diagonal, and every test through
/// Phase 2 passed anyway: the sole caller was b3ComputeCapsuleMass, which
/// shifts along the capsule's own axis in a canonical frame where that axis is
/// a coordinate axis -- so two of the three components are zero and all three
/// off-diagonal terms vanish. Phase 3A's b3UpdateBodyMassData shifts by an
/// arbitrary offset from a body's centre of mass, where they do not.
///
/// https://en.wikipedia.org/wiki/Parallel_axis_theorem
B3_INLINE b3Matrix3 b3SteinerUnit( b3Vec3 origin )
{
	b3f xx = b3MulFF( origin.x, origin.x );
	b3f yy = b3MulFF( origin.y, origin.y );
	b3f zz = b3MulFF( origin.z, origin.z );

	b3f ixy = b3NegF( b3MulFF( origin.x, origin.y ) );
	b3f ixz = b3NegF( b3MulFF( origin.x, origin.z ) );
	b3f iyz = b3NegF( b3MulFF( origin.y, origin.z ) );

	return b3MakeMatrix3( b3MakeVec3( b3AddF( yy, zz ), ixy, ixz ), b3MakeVec3( ixy, b3AddF( xx, zz ), iyz ),
						  b3MakeVec3( ixz, iyz, b3AddF( xx, yy ) ) );
}

/// Rotate an inertia tensor: R * I * Rᵀ.
///
/// Scale-neutral, so it applies equally to the per-unit-mass tensors above and
/// to an absolute one. The similarity transform costs precision either way --
/// b3MakeMatrixFromQuat narrows the Q30 orientation to Q12 -- but an inertia
/// tensor is only ever consumed as a mass, not integrated, so the error does
/// not compound.
B3_INLINE b3Matrix3 b3RotateInertia( b3Quat q, b3Matrix3 centralInertia )
{
	b3Matrix3 rotation = b3MakeMatrixFromQuat( q );
	return b3MulMM( rotation, b3MulMM( centralInertia, b3Transpose( rotation ) ) );
}

// -------------------------------------------------------------------------
// Inverse inertia, at Q24
// -------------------------------------------------------------------------

B3_INLINE b3Vec3W b3MakeVec3W( b3iw x, b3iw y, b3iw z )
{
	b3Vec3W v = { x, y, z };
	return v;
}

B3_INLINE b3MatrixW b3MakeMatrixW( b3Vec3W cx, b3Vec3W cy, b3Vec3W cz )
{
	b3MatrixW m = { cx, cy, cz };
	return m;
}

// -------------------------------------------------------------------------
// Q24 as a *length* scale
// -------------------------------------------------------------------------
//
// Everything above uses Q24 for inverse mass and inverse inertia, which is
// what the scale was introduced for. The position accumulator below borrows
// it for a bounded length, on the same footing that b3t borrows it for time:
// Q7.24 is a scale, and a quantity qualifies for it by its range rather than
// by its units. A per-step position delta is bounded by maxLinearSpeed * dt,
// which at the port's 400 m/s cap and 60 Hz is 6.7 -- well inside 128.

B3_INLINE b3Vec3W b3Vec3W_zeroFn( void )
{
	return b3MakeVec3W( b3iw_zero, b3iw_zero, b3iw_zero );
}

#define b3Vec3W_zero b3Vec3W_zeroFn()

B3_INLINE b3Vec3W b3AddW3( b3Vec3W a, b3Vec3W b )
{
	return b3MakeVec3W( b3AddW( a.x, b.x ), b3AddW( a.y, b.y ), b3AddW( a.z, b.z ) );
}

B3_INLINE b3Vec3W b3SubW3( b3Vec3W a, b3Vec3W b )
{
	return b3MakeVec3W( b3SubW( a.x, b.x ), b3SubW( a.y, b.y ), b3SubW( a.z, b.z ) );
}

/// Q24 -> Q12, rounding each component to nearest. The narrowing a Q24
/// position accumulator goes through when it is applied to a Q12 position.
B3_INLINE b3Vec3 b3W3ToVec3( b3Vec3W v )
{
	return b3MakeVec3( b3WToF( v.x ), b3WToF( v.y ), b3WToF( v.z ) );
}

/// Q12 -> Q24, exact. Valid for |component| < 128.
B3_INLINE b3Vec3W b3Vec3ToW3( b3Vec3 v )
{
	return b3MakeVec3W( b3FToW( v.x ), b3FToW( v.y ), b3FToW( v.z ) );
}

/// Scale a velocity by a sub-step, keeping the *result* at Q24 as well.
///
/// b3MulVT solves half the problem -- it stops the sub-step itself being
/// narrowed to Q12. This solves the other half: at Q12 the position increment
/// is only tens of quanta, and rounding it is a constant bias rather than a
/// zero-mean one, because the velocity does not change between sub-steps. See
/// b3MulFTToW, which carries the derivation and the measured figures.
B3_INLINE b3Vec3W b3MulVTToW( b3Vec3 v, b3t h )
{
	return b3MakeVec3W( b3MulFTToW( v.x, h ), b3MulFTToW( v.y, h ), b3MulFTToW( v.z, h ) );
}

// -------------------------------------------------------------------------
// Impulses, at Q15.16
// -------------------------------------------------------------------------
//
// The contact solver's currency. Every one of these is a thin wrapper over a
// b3fixed primitive that already existed for the scalar case; the scale
// reasoning lives there and is not repeated per component.
//
// The recurring shape in all five solver stages is
//
//     P  = deltaImpulse * normal            b3MulNImp
//     v += invMass * P                      b3MulImpW3
//     w += invInertia * cross( r, P )       b3CrossVImp then b3MulMWImp
//
// -- an impulse built from a direction, then spent through an inverse mass and
// an inverse inertia to give velocity changes at Q12.

B3_INLINE b3Imp3 b3MakeImp3( b3imp x, b3imp y, b3imp z )
{
	b3Imp3 v = { x, y, z };
	return v;
}

B3_INLINE b3Imp3 b3Imp3_zeroFn( void )
{
	return b3MakeImp3( b3imp_zero, b3imp_zero, b3imp_zero );
}

#define b3Imp3_zero b3Imp3_zeroFn()

B3_INLINE b3Imp3 b3AddImp3( b3Imp3 a, b3Imp3 b )
{
	return b3MakeImp3( b3AddImp( a.x, b.x ), b3AddImp( a.y, b.y ), b3AddImp( a.z, b.z ) );
}

B3_INLINE b3Imp3 b3SubImp3( b3Imp3 a, b3Imp3 b )
{
	return b3MakeImp3( b3SubImp( a.x, b.x ), b3SubImp( a.y, b.y ), b3SubImp( a.z, b.z ) );
}

B3_INLINE b3Imp3 b3NegImp3( b3Imp3 a )
{
	return b3MakeImp3( b3NegImp( a.x ), b3NegImp( a.y ), b3NegImp( a.z ) );
}

/// An impulse magnitude along a unit direction. Q16 x Q30 -> Q16.
///
/// The direction is a b3Dir3 rather than a b3Vec3 because the contact frame is
/// converted to Q30 once in the prepare pass: this product runs several times
/// per point per sub-step, and a Q12 direction would put its 0.014-degree
/// error inside the loop rather than outside it.
B3_INLINE b3Imp3 b3MulNImp( b3imp s, b3Dir3 d )
{
	return b3MakeImp3( b3MulImpC( s, b3NToC( d.x ) ), b3MulImpC( s, b3NToC( d.y ) ), b3MulImpC( s, b3NToC( d.z ) ) );
}

/// An impulse scaled by a dimensionless coefficient -- the warm-start scale
/// and the friction cone's rescale.
B3_INLINE b3Imp3 b3MulCImp3( b3c s, b3Imp3 a )
{
	return b3MakeImp3( b3MulImpC( a.x, s ), b3MulImpC( a.y, s ), b3MulImpC( a.z, s ) );
}

/// An impulse spent through an inverse mass, giving a velocity change.
/// Q16 x Q24 -> Q12.
B3_INLINE b3Vec3 b3MulImpW3( b3Imp3 p, b3iw m )
{
	return b3MakeVec3( b3MulImpW( p.x, m ), b3MulImpW( p.y, m ), b3MulImpW( p.z, m ) );
}

/// An angular impulse spent through an inverse inertia tensor, giving an
/// angular velocity change. Q24 x Q16 -> Q12.
///
/// The b3MulMWV of the impulse world: same matrix, same output scale, an
/// operand one scale over.
B3_API b3Vec3 b3MulMWImp( b3MatrixW m, b3Imp3 p );

/// cross( r, P ) -- a lever arm at Q12 against an impulse at Q16, giving the
/// angular impulse about the centre of mass at Q16.
B3_API b3Imp3 b3CrossVImp( b3Vec3 r, b3Imp3 p );

/// An impulse vector projected onto a unit direction. Q16 x Q30 -> Q16.
///
/// Used once per manifold in the prepare pass, to split a stored friction
/// impulse across the two tangents of a frame that may have been rebuilt since
/// it was written.
B3_INLINE b3imp b3DotImpN( b3Imp3 a, b3Dir3 d )
{
	return b3AddImp( b3AddImp( b3MulImpC( a.x, b3NToC( d.x ) ), b3MulImpC( a.y, b3NToC( d.y ) ) ),
					 b3MulImpC( a.z, b3NToC( d.z ) ) );
}

/// A Q12 inertia times a Q12 angular velocity, landing directly at the impulse
/// scale. Q12 x Q12 -> Q16.
///
/// The rolling-resistance term, `rollingMass * (wB - wA)`. Going through
/// b3MulMV and widening the result would narrow each product to Q12 first and
/// then pad it with zeros -- the four bits that distinguish the scales would be
/// discarded and immediately faked. b3MulFFToImp widens as it multiplies, so
/// they are real.
B3_INLINE b3Imp3 b3MulMVToImp( b3Matrix3 m, b3Vec3 v )
{
	return b3MakeImp3(
		b3AddImp( b3AddImp( b3MulFFToImp( m.cx.x, v.x ), b3MulFFToImp( m.cy.x, v.y ) ), b3MulFFToImp( m.cz.x, v.z ) ),
		b3AddImp( b3AddImp( b3MulFFToImp( m.cx.y, v.x ), b3MulFFToImp( m.cy.y, v.y ) ), b3MulFFToImp( m.cz.y, v.z ) ),
		b3AddImp( b3AddImp( b3MulFFToImp( m.cx.z, v.x ), b3MulFFToImp( m.cy.z, v.y ) ), b3MulFFToImp( m.cz.z, v.z ) ) );
}

/// `m * v` at the impulse scale, uniformly scaled down if it does not fit.
///
/// The same product b3MulMVToImp forms, for the one caller that cannot let it
/// saturate: a **bounded** drive whose bound is being exceeded.
///
/// b3MulMVToImp saturates each component independently, which is right for a
/// rigid constraint -- a saturating impulse there means the scene is already
/// lost -- and wrong for motor_joint.c. A bounded branch is *expected* to ask
/// for more than it may spend, every step, whenever the load exceeds the
/// budget; that is what a bound is for. And the position bias grows without
/// limit while the body it failed to hold falls away, so the raw ask does reach
/// the ceiling in an ordinary scene. Saturating each component separately at
/// that point rotates the vector -- three components clipped by different
/// amounts do not point where they did -- and b3ClampImp3 then dutifully
/// bounds the *wrong direction*, so an under-powered drive pushes askew rather
/// than simply weakly.
///
/// Scaling the whole vector down instead preserves the direction exactly, and
/// costs nothing real: the caller is about to clamp the accumulator to a
/// magnitude far below the ceiling anyway, so the discarded bits could not have
/// survived. This is b3InvertAccumulated's trade, on a vector.
///
/// Each product is rounded and narrowed to Q16 exactly as b3MulFFToImp rounds
/// it, and only then summed -- so a result that *does* fit is bit-identical to
/// b3MulMVToImp's and no baseline moves. Narrowing before the sum also keeps
/// the accumulation itself inside the int64.
B3_INLINE b3Imp3 b3MulMVToImpSat( b3Matrix3 m, b3Vec3 v )
{
#define B3_MV_TERM( a, b ) B3_MUL_ROUND( (int64_t)b3Raw( a ) * (int64_t)b3Raw( b ), B3_F_SHIFT + B3_F_SHIFT - B3_IMP_SHIFT )

	int64_t c[3];
	c[0] = B3_MV_TERM( m.cx.x, v.x ) + B3_MV_TERM( m.cy.x, v.y ) + B3_MV_TERM( m.cz.x, v.z );
	c[1] = B3_MV_TERM( m.cx.y, v.x ) + B3_MV_TERM( m.cy.y, v.y ) + B3_MV_TERM( m.cz.y, v.z );
	c[2] = B3_MV_TERM( m.cx.z, v.x ) + B3_MV_TERM( m.cy.z, v.y ) + B3_MV_TERM( m.cz.z, v.z );

#undef B3_MV_TERM

	int64_t largest = 0;
	for ( int i = 0; i < 3; ++i )
	{
		int64_t magnitude = c[i] < 0 ? -c[i] : c[i];
		if ( magnitude > largest )
		{
			largest = magnitude;
		}
	}

	// INT32_MAX / 2, so the caller's `old + lambda` has a bit of room too.
	int down = 0;
	while ( largest > INT32_MAX / 2 && down < 32 )
	{
		largest >>= 1;
		down += 1;
	}

	return b3MakeImp3( b3Makeb3imp( (int32_t)( c[0] >> down ) ), b3Makeb3imp( (int32_t)( c[1] >> down ) ),
					   b3Makeb3imp( (int32_t)( c[2] >> down ) ) );
}

/// Scale a plain Q12 vector by an impulse. Q16 x Q12 -> Q16.
///
/// The counterpart of b3MulNImp for an axis that is *not* a unit direction.
/// The revolute joint's two perpendicular axes are half-length by
/// construction, so they cannot be b3Dir3 without lying about what they are,
/// and the angular impulse is assembled from them component by component.
B3_INLINE b3Imp3 b3MulImpV( b3imp s, b3Vec3 v )
{
	return b3MakeImp3( b3MulImpF( s, v.x ), b3MulImpF( s, v.y ), b3MulImpF( s, v.z ) );
}

/// `x * t1 + y * t2` -- an impulse rebuilt from its two tangent components.
B3_INLINE b3Imp3 b3BlendImp2( b3imp x, b3Dir3 t1, b3imp y, b3Dir3 t2 )
{
	return b3AddImp3( b3MulNImp( x, t1 ), b3MulNImp( y, t2 ) );
}

/// Squared magnitude of an impulse vector, kept wide at Q32.
///
/// Both friction cones compare this against `maxImpulse^2` and neither may go
/// through a 32-bit type to do it -- the same treatment 3B's `distSquared`
/// needed. Only the rescale that follows a failed comparison takes a root, via
/// b3SqrtWideImp.
B3_INLINE int64_t b3Imp3LengthSquaredWide( b3Imp3 a )
{
	return (int64_t)b3Raw( a.x ) * b3Raw( a.x ) + (int64_t)b3Raw( a.y ) * b3Raw( a.y ) +
		   (int64_t)b3Raw( a.z ) * b3Raw( a.z );
}

/// Clamp an impulse vector to a maximum *magnitude*, leaving its direction
/// alone.
///
/// A 3-vector accumulator's bound is a sphere, not a box: the spherical joint's
/// motor may spend its whole torque budget about one axis or spread it over
/// three, and clamping each component separately would let it exceed the bound
/// by sqrt(3) along a diagonal.
///
/// The comparison is done on raw int64 squares and never through a 32-bit type,
/// so the square root runs only on the clamping path -- the same shape
/// contact_solver.c uses for both friction cones. Those two call sites open-code
/// it rather than calling here: they sit on the hot path with measured
/// baselines, and Stage 4 is not the change that should move them.
B3_INLINE b3Imp3 b3ClampImp3( b3Imp3 v, b3imp maxMagnitude )
{
	int64_t magnitudeSquared = b3Imp3LengthSquaredWide( v );
	int64_t maxSquared = (int64_t)b3Raw( maxMagnitude ) * b3Raw( maxMagnitude );

	if ( magnitudeSquared <= maxSquared )
	{
		return v;
	}

	b3imp magnitude = b3SqrtWideImp( magnitudeSquared );
	if ( b3Raw( magnitude ) == 0 )
	{
		return v;
	}

	// The ratio is formed at the *impulse* scale, not narrowed to Q12 first.
	// A ratio has no scale of its own, so dividing the two raw Q16 values is
	// exactly as correct and keeps four bits that Q12 would drop -- and those
	// four bits are the whole bound when the bound is small. A joint motor
	// limited to 0.05 N-m at 240 Hz has a maximum impulse of 14 Q16 quanta,
	// which narrows to *zero* at Q12: the first version of this routine scaled
	// every such impulse to nothing and reported a torque 18% over its bound,
	// because the accumulator was being emptied and refilled rather than
	// clamped.
	//
	// contact_solver.c's two friction cones narrow to Q12 and are correct to,
	// for the reason this is not: a contact impulse carrying a stack is
	// thousands of quanta, where four bits cost nothing.
	b3c scale = b3DivWideToC( (int64_t)b3Raw( maxMagnitude ), (int64_t)b3Raw( magnitude ) );
	return b3MulCImp3( scale, v );
}

B3_INLINE b3Imp2 b3MakeImp2( b3imp x, b3imp y )
{
	b3Imp2 v = { x, y };
	return v;
}

B3_INLINE b3Imp2 b3Imp2_zeroFn( void )
{
	return b3MakeImp2( b3imp_zero, b3imp_zero );
}

#define b3Imp2_zero b3Imp2_zeroFn()

B3_INLINE b3Imp2 b3AddImp2( b3Imp2 a, b3Imp2 b )
{
	return b3MakeImp2( b3AddImp( a.x, b.x ), b3AddImp( a.y, b.y ) );
}

B3_INLINE b3Imp2 b3SubImp2( b3Imp2 a, b3Imp2 b )
{
	return b3MakeImp2( b3SubImp( a.x, b.x ), b3SubImp( a.y, b.y ) );
}

/// A two-impulse scaled by a dimensionless coefficient. b3MulCImp3 one
/// dimension down.
B3_INLINE b3Imp2 b3MulCImp2( b3c s, b3Imp2 a )
{
	return b3MakeImp2( b3MulImpC( a.x, s ), b3MulImpC( a.y, s ) );
}

/// Squared magnitude of a two-impulse, kept wide at Q32.
///
/// Unlike b3Imp3LengthSquaredWide above, this one is safe at *full-scale raw*
/// and provably so: `2 * (2^31 - 1)^2` is 9.2233720283e18 against an INT64_MAX
/// of 9.2233720369e18, so it fits with 8.6e9 to spare no matter what the
/// operands are. Three squares do not -- `3 * (2^31 - 1)^2` is 1.38e19 -- so
/// the 3-vector form carries a latent overflow this one does not. It needs a
/// component near 32768 N-s to reach, which is why it has never been hit, but a
/// reader should not infer the sibling's safety from this one's.
B3_INLINE int64_t b3Imp2LengthSquaredWide( b3Imp2 a )
{
	return (int64_t)b3Raw( a.x ) * b3Raw( a.x ) + (int64_t)b3Raw( a.y ) * b3Raw( a.y );
}

/// Clamp a two-impulse to a maximum *magnitude*, leaving its direction alone.
///
/// b3ClampImp3's bound is a sphere; this one is a disc, and the reasoning is
/// identical: the parallel joint's torque budget may be spent about either
/// constrained axis or shared between them, and clamping each component
/// separately would let it exceed the bound by sqrt(2) along a diagonal.
///
/// The ratio is formed on raw Q16 for the reason b3ClampImp3 spells out at
/// length, and it matters *more* here than anywhere it has been argued before:
/// a parallel joint is defined by its torque bound, so the bound is not an
/// exceptional case but the joint's normal operating point. At 240 Hz a
/// 0.05 N-m limit is 14 Q16 quanta and zero Q12 quanta.
///
/// The ratio also cannot saturate. The branch is only entered when
/// `maxSquared < magnitudeSquared`, so `0 <= max < |v|` strictly and the
/// quotient lies in [0, 1) -- inside Q30 with a factor of two to spare, which
/// makes b3DivWideToC's saturation path unreachable from here.
B3_INLINE b3Imp2 b3ClampImp2( b3Imp2 v, b3imp maxMagnitude )
{
	int64_t magnitudeSquared = b3Imp2LengthSquaredWide( v );
	int64_t maxSquared = (int64_t)b3Raw( maxMagnitude ) * b3Raw( maxMagnitude );

	if ( magnitudeSquared <= maxSquared )
	{
		return v;
	}

	b3imp magnitude = b3SqrtWideImp( magnitudeSquared );
	if ( b3Raw( magnitude ) == 0 )
	{
		return v;
	}

	b3c scale = b3DivWideToC( (int64_t)b3Raw( maxMagnitude ), (int64_t)b3Raw( magnitude ) );
	return b3MulCImp2( scale, v );
}

// -------------------------------------------------------------------------
// The two effective-mass inversions
// -------------------------------------------------------------------------

/// Sum of two Q24 matrices. **Nothing in the library calls this, deliberately.**
///
/// It was written for `invIA + invIB` and every one of those call sites has
/// since been removed: Q7.24 tops out at 128, b3InvertInertia caps a *single*
/// body's inverse inertia there, and so a sum of two ordinary bodies' wraps.
/// Stage 5 closed the two sites in contact_solver.c and Stage 6 the rest. The
/// effective-mass helpers below take the two matrices unsummed and accumulate
/// in int64 for exactly this reason.
///
/// It survives because test_math.c exercises the wrap deliberately, and because
/// a joint author reaching for it should find this comment rather than an
/// absence. Do not add a caller.
B3_INLINE b3MatrixW b3AddMWMW( b3MatrixW a, b3MatrixW b )
{
	return b3MakeMatrixW( b3AddW3( a.cx, b.cx ), b3AddW3( a.cy, b.cy ), b3AddW3( a.cz, b.cz ) );
}

/// Invert a Q24 inverse-inertia sum, giving a Q12 inertia. Zero if singular.
///
/// `rollingMass = inverse( invIA + invIB )` is the one caller. b3InvertMatrix
/// cannot be reused by narrowing the sum to Q12 first: that is precisely the
/// loss b3MulWF's comment describes, where a 0.001 entry keeps four bits, and
/// the result here is a *divisor* in the rolling-resistance solve.
///
/// The cofactor and determinant scales are matched at Q36 rather than
/// b3InvertMatrix's Q24, because a determinant is cubic in entries that are
/// themselves small: at an inverse inertia of 1e-3 a Q24 determinant is 0.017
/// and rounds to zero, which would report a perfectly well conditioned matrix
/// as singular and silently disable rolling resistance on every heavy body.
/// Q36 puts that case at 69 and still leaves the Q7.24 ceiling of 128 inside
/// int64 once b3DivWideToF has shifted the numerator up by twelve.
B3_API b3Matrix3 b3InvertMatrixW( b3MatrixW m );

/// Invert a symmetric Q24 2x2, giving the Q12 tangent mass. Zero if singular.
///
/// Central friction's effective mass. Singular is reachable and is not an
/// error: two tangents through a contact whose arms are collinear give a
/// rank-one matrix, and the right answer there is to apply no friction impulse
/// rather than an infinite one.
B3_API b3SymMatrix2 b3InvertSym2W( b3SymMatrix2W k );

/// `m * v` for the symmetric tangent mass. Q12 x Q12 -> Q16.
B3_INLINE b3Imp2 b3MulSym2V( b3SymMatrix2 m, b3f x, b3f y )
{
	b3Imp2 r = { b3MulFFToImp( m.xx, x ), b3MulFFToImp( m.yy, y ) };
	r.x = b3AddImp( r.x, b3MulFFToImp( m.xy, y ) );
	r.y = b3AddImp( r.y, b3MulFFToImp( m.xy, x ) );
	return r;
}

/// `m * v` again, but saturating instead of wrapping. b3MulMVToImpSat one
/// dimension down, and it exists for the same reason.
///
/// @section why What overflows, and why the clamp cannot catch it
///
/// A bounded drive computes an unbounded ask and then clamps it. That is fine
/// in float, where the intermediate is merely large, and it is not fine here:
/// the product must survive Q15.16 *before* the clamp can bound it, and a
/// wrapped intermediate arrives at the clamp with the wrong magnitude and
/// possibly the wrong sign -- so the clamp dutifully bounds a torque that now
/// points the wrong way, and the constraint drives what it exists to hold.
///
/// The parallel joint reaches this in ordinary use, which is what makes it
/// worth a routine rather than a comment. Its Jacobian rows have length
/// `0.5 * sqrt(1 - (v.e)^2)`, which **vanishes at a half turn**, so a body
/// tumbling past upright drives the 2x2's determinant toward zero and its
/// inverse toward infinity. Measured on a box spun at 40 rad/s: an ask of
/// 301,342 N-s against Q15.16's ceiling of 32,768 -- a factor of nine past the
/// scale, found by the debug shadow checker, and silent in device and strict
/// modes because there the wrap is just an int32 wrapping.
///
/// Scaling both components down together preserves the *direction* of the ask,
/// which is all the caller needs: the clamp that follows replaces the magnitude
/// anyway. b3MulSym2V is left alone rather than made to saturate, because its
/// other caller is the contact solver's central friction, whose asks are bounded
/// by the geometry and whose numbers are a measured baseline.
B3_INLINE b3Imp2 b3MulSym2VSat( b3SymMatrix2 m, b3f x, b3f y )
{
#define B3_SYM2_TERM( a, b ) B3_MUL_ROUND( (int64_t)b3Raw( a ) * (int64_t)b3Raw( b ), B3_F_SHIFT + B3_F_SHIFT - B3_IMP_SHIFT )

	int64_t c[2];
	c[0] = B3_SYM2_TERM( m.xx, x ) + B3_SYM2_TERM( m.xy, y );
	c[1] = B3_SYM2_TERM( m.xy, x ) + B3_SYM2_TERM( m.yy, y );

#undef B3_SYM2_TERM

	int64_t largest = 0;
	for ( int i = 0; i < 2; ++i )
	{
		int64_t magnitude = c[i] < 0 ? -c[i] : c[i];
		if ( magnitude > largest )
		{
			largest = magnitude;
		}
	}

	// INT32_MAX / 2, so the caller's `old + lambda` has a bit of room too --
	// b3MulMVToImpSat's bound, for b3MulMVToImpSat's reason.
	int down = 0;
	while ( largest > INT32_MAX / 2 && down < 32 )
	{
		largest >>= 1;
		down += 1;
	}

	return b3MakeImp2( b3Makeb3imp( (int32_t)( c[0] >> down ) ), b3Makeb3imp( (int32_t)( c[1] >> down ) ) );
}

B3_INLINE b3MatrixW b3MatW_zeroFn( void )
{
	b3Vec3W z = b3MakeVec3W( b3iw_zero, b3iw_zero, b3iw_zero );
	return b3MakeMatrixW( z, z, z );
}

#define b3MatW_zero b3MatW_zeroFn()

/// Inverse-inertia matrix times a vector. Q24 * Q12 -> Q12.
///
/// The solver's use: an angular impulse in, an angular velocity change out.
B3_API b3Vec3 b3MulMWV( b3MatrixW m, b3Vec3 v );

/// `axis . (iA + iB) . axis`, at Q24 in an int64: a joint's scalar effective
/// mass before it is reciprocated.
///
/// This exists because forming `iA + iB` first does not work, and the failure
/// is silent. b3InvertInertia caps a *single* body's inverse inertia at Q7.24's
/// ceiling of 128 by scaling rather than wrapping, so each matrix is always in
/// range -- but their **sum** need not be, and b3AddMWMW wraps it. A ragdoll's
/// head and torso sum to 128.197, which wrapped to -128 and inverted the
/// constraint: the figure was flung apart rather than held together, on the
/// first frame it was upright.
///
/// Two light bodies jointed together is an ordinary scene, not a corner, so the
/// sum is never formed. Each matrix is applied to the axis separately at Q12 --
/// where 256 is nothing next to the 524287 the scale holds -- and only the
/// final quadratic form is accumulated wide, exactly as b3DotVWide would but
/// without narrowing the result to b3iw on the way out.
///
/// Pair with b3RcpWide, which takes the int64 this returns.
B3_INLINE int64_t b3AxisInertiaSumWide( b3Vec3 axis, b3MatrixW iA, b3MatrixW iB )
{
	b3Vec3 t = b3Add( b3MulMWV( iA, axis ), b3MulMWV( iB, axis ) );
	return (int64_t)b3Raw( axis.x ) * b3Raw( t.x ) + (int64_t)b3Raw( axis.y ) * b3Raw( t.y ) +
		   (int64_t)b3Raw( axis.z ) * b3Raw( t.z );
}

/// `u . (iA + iB) . v`, at Q24 in an int64: the off-diagonal companion to
/// b3AxisInertiaSumWide, for a joint whose effective mass is a matrix rather
/// than a scalar.
///
/// Same argument, same reason, and `u == v` gives exactly what
/// b3AxisInertiaSumWide gives. Both exist because the diagonal case is the
/// common one and reads better with one axis named once.
///
/// Pair with b3InvertPerpMass, which builds a symmetric 2x2 from three of
/// these.
B3_INLINE int64_t b3AxisInertiaCrossWide( b3Vec3 u, b3Vec3 v, b3MatrixW iA, b3MatrixW iB )
{
	b3Vec3 t = b3Add( b3MulMWV( iA, v ), b3MulMWV( iB, v ) );
	return (int64_t)b3Raw( u.x ) * b3Raw( t.x ) + (int64_t)b3Raw( u.y ) * b3Raw( t.y ) +
		   (int64_t)b3Raw( u.z ) * b3Raw( t.z );
}

/// `sA . iA . tA + sB . iB . tB`, at Q24 in an int64.
///
/// The two-lever-arm counterpart of b3AxisInertiaCrossWide. That one computes
/// `u . (iA + iB) . v` -- one direction, both matrices -- which is the shape a
/// constraint gets when **both bodies push through the same axis**: a hinge's
/// twist, a ball joint's cone. Plenty of constraints are not that shape. A
/// contact's are `cross( rA, n )` and `cross( rB, n )`, two different lever
/// arms; a prismatic joint's are `cross( rA + d, axis )` and
/// `cross( rB, axis )`, where `d` is the slide translation, so the arm is a
/// function of the joint's own **state** and not merely of its geometry.
///
/// Each matrix is applied to its own arm at Q12 through b3MulMWV -- where a
/// 128-entry tensor against a 20 m arm reaches 2560 against the 524287 Q12
/// holds -- and only the final quadratic form is accumulated wide. The sum
/// `iA + iB` is never formed, for b3AxisInertiaSumWide's reason.
///
/// No mass term: callers that need one add it to the diagonal, because the
/// off-diagonal of `(mA + mB) * I` is zero and folding it in here would be
/// wrong for exactly the entry that matters.
B3_INLINE int64_t b3LeverInertiaCrossWide( b3MatrixW iA, b3Vec3 sA, b3Vec3 tA, b3MatrixW iB, b3Vec3 sB, b3Vec3 tB )
{
	b3Vec3 a = b3MulMWV( iA, tA );
	b3Vec3 b = b3MulMWV( iB, tB );
	return (int64_t)b3Raw( sA.x ) * b3Raw( a.x ) + (int64_t)b3Raw( sA.y ) * b3Raw( a.y ) +
		   (int64_t)b3Raw( sA.z ) * b3Raw( a.z ) + (int64_t)b3Raw( sB.x ) * b3Raw( b.x ) +
		   (int64_t)b3Raw( sB.y ) * b3Raw( b.y ) + (int64_t)b3Raw( sB.z ) * b3Raw( b.z );
}

/// `mA + mB + sA . iA . sA + sB . iB . sB`, at Q24 in an int64: the scalar
/// effective mass of a constraint whose two bodies have different lever arms,
/// before it is reciprocated.
///
/// **Both inverse masses are summed here**, in the int64 the rest of the
/// accumulation lives in, and never by the caller. That is the whole reason the
/// signature takes two masses rather than one sum. B3_MIN_MASS_RAW caps a
/// *single* inverse mass at about 124, just inside Q7.24's ceiling of 128 -- so
/// one body always fits and no amount of testing one body finds anything. Two
/// bodies below roughly 16 g each sum past the ceiling, and b3AddW is a plain
/// int32 add in a device build, so the sum comes back **negative** and the
/// effective mass with it. A negative effective mass does not lose precision,
/// it inverts the constraint.
///
/// Pair with b3RcpWide, which takes the int64 this returns.
B3_INLINE int64_t b3LeverInertiaSumWide( b3iw mA, b3MatrixW iA, b3Vec3 sA, b3iw mB, b3MatrixW iB, b3Vec3 sB )
{
	return (int64_t)b3Raw( mA ) + (int64_t)b3Raw( mB ) + b3LeverInertiaCrossWide( iA, sA, sA, iB, sB, sB );
}

/// Q24 matrix product, used only by the similarity transform below.
B3_API b3MatrixW b3MulMWMW( b3MatrixW a, b3MatrixW b );

/// The point-to-point effective mass of a joint, inverted.
///
/// `K = (mA + mB) * I - skew(rA) * iA * skew(rA) - skew(rB) * iB * skew(rB)`,
/// returned already inverted at Q12 -- the form a solve wants, since K itself
/// is only ever a divisor.
///
/// **This cannot be built at Q24, which is why it is a function.** Every entry
/// of K is an inverse mass, so Q24 looks like the natural scale and is what
/// the contact solver's `kNormal` uses. But a contact's lever arm lies inside
/// the shape, while a *joint's* is however long the designer made it, and the
/// product grows as its square: a 0.1 m sphere has an inverse inertia near 60,
/// and on a 2 m arm the first multiply alone reaches 131 against Q7.24's
/// ceiling of 128. That overflow is not a corner case -- it is a small light
/// body on an ordinary arm, and the shadow checker found it on the first run
/// of the revolute joint's tests.
///
/// So the entries are accumulated in int64 and the whole matrix is scaled down
/// by a power of two before inversion, exactly the uniform-scaling trade
/// b3InvertInertia makes and for the same reason: scaling K uniformly scales
/// its inverse uniformly, so the shift is undone exactly on the way out.
/// Better conditioned, too -- a large K has a small inverse, and inverting the
/// scaled-down matrix puts that inverse where Q12 can hold it.
///
/// Returns the zero matrix if K is singular, meaning "apply no impulse", as
/// every other effective mass in the port does.
/// The two inverse masses are passed separately and summed inside, at int64.
/// Stage 5: `b3AddW( mA, mB )` at the call site was the inertia sum's defect
/// one type over -- Q7.24 tops out at 128 and a 4.2 g pebble already has an
/// inverse mass of 239, so a *single* light body is past the ceiling and the
/// sum wrapped to a large negative.
B3_API b3Matrix3 b3InvertPointMass( b3iw mA, b3MatrixW iA, b3Vec3 rA, b3iw mB, b3MatrixW iB, b3Vec3 rB );

/// The rotational effective mass of a joint: inverse( invIA + invIB ), Q24 in
/// and Q12 out.
///
/// What a constraint acting purely on rotation divides by, the way
/// b3InvertPointMass is what a constraint acting on position divides by. The
/// spherical joint's spring and motor both push through it; the weld and motor
/// joints will too.
///
/// b3InvertMatrixW cannot be used on the sum directly, and the reason is range
/// rather than algebra. An inverse inertia is *large* for a small light body --
/// near 60 for a 0.1 m sphere at the default density -- so two of them sum to
/// around 120 against Q7.24's ceiling of 128. A ragdoll's hand reaches that,
/// which makes it a scene rather than a corner case, so the entries are
/// accumulated wide and the whole matrix uniformly scaled before inversion:
/// the same trade b3InvertPointMass makes, sharing the same implementation.
///
/// Returns the zero matrix when the sum is singular -- two bodies with no
/// rotational freedom between them -- which means "apply no impulse", and is
/// what b3JointSim::fixedRotation exists to anticipate.
B3_API b3Matrix3 b3InvertRotationMass( b3MatrixW iA, b3MatrixW iB );

/// The 2x2 rotational effective mass of a joint constrained along two axes,
/// inverted. Q24 in, Q12 out.
///
/// `K = [ uX.(iA+iB).uX  uX.(iA+iB).uY ; ...  uY.(iA+iB).uY ]`, the revolute's
/// collinearity block. b3InvertSym2W is what inverts it and is not enough on
/// its own, for b3InvertRotationMass's range reason exactly: an entry is a sum
/// of two inverse inertias, and two light bodies sum past Q7.24's ceiling of
/// 128 before the entry is ever formed. So the three entries are accumulated in
/// int64 through b3AxisInertiaCrossWide, the pair is scaled uniformly down
/// until each fits a b3iw, and the Q12 inverse is shifted back by the same
/// count -- b3InvertAccumulated's trade, applied to a 2x2.
///
/// Returns zero when K is singular, meaning "apply no impulse", as every other
/// effective mass in the port does.
B3_API b3SymMatrix2 b3InvertPerpMass( b3Vec3 uX, b3Vec3 uY, b3MatrixW iA, b3MatrixW iB );

/// The point-to-line effective mass of a constraint, inverted. Q24 in, Q12 out.
///
/// ```
/// K = [ mA+mB + sAy.iA.sAy + sBy.iB.sBy      sAy.iA.sAz + sBy.iB.sBz ]
///     [        (symmetric)                mA+mB + sAz.iA.sAz + sBz.iB.sBz ]
/// ```
///
/// A prismatic joint's perpendicular block and a contact's central-friction
/// tangent block are both this. Named against b3InvertPointMass -- the 3x3 that
/// locks a point outright -- rather than against b3InvertPerpMass, because what
/// it is is a point held **on a line**: two directions removed, the third free.
///
/// b3InvertPerpMass cannot serve. It has no mass term at all, and it shares one
/// pair of axes between the two bodies; here each body has its own lever arms,
/// which is what b3LeverInertiaCrossWide exists for.
///
/// **The wide accumulation is mandatory rather than prudent here, and both
/// terms are why.** With an inverse inertia of 60 on a 2 m arm the inertia term
/// alone reaches 240 -- raw 4.0e9 against INT32_MAX's 2.15e9 -- and two bodies
/// under 16 g put the mass term past the ceiling on its own. Unlike
/// b3InvertPerpMass, where only the *sum* of two in-range inertias overflowed, a
/// single term here does.
///
/// Returns zero when K is singular, meaning "apply no impulse", as every other
/// effective mass in the port does.
B3_API b3SymMatrix2 b3InvertPointLineMass( b3iw mA, b3MatrixW iA, b3Vec3 sAy, b3Vec3 sAz, b3iw mB, b3MatrixW iB,
										   b3Vec3 sBy, b3Vec3 sBz );

/// Rotate an inverse inertia tensor into world space: R * I^-1 * Rᵀ.
///
/// The b3Matrix3 form above cannot be reused because the result must stay at
/// Q24; narrowing to Q12 in the middle would throw away exactly the bits the
/// tensor exists to carry.
B3_API b3MatrixW b3RotateInertiaW( b3Quat q, b3MatrixW localInverse );

/// Invert the body inertia tensor, given per unit mass, producing the local
/// inverse inertia at Q24.
///
/// @param unitInertia  symmetric positive-definite tensor per unit mass (Q12),
///                     as b3MassData carries it
/// @param mass         total body mass (Q12)
///
/// Why the inputs are split this way. Absolute inertia is mass times length
/// squared and overflows Q12 for anything larger than a toy -- which is the
/// reason b3MassData stores the per-unit-mass form in the first place. This
/// function does form mass * U, but only as an int64 that is normalized in the
/// same breath, so the dangerous quantity never has to fit a register.
///
/// Range. The result is genuinely unbounded below in body size: a 5 cm sphere
/// at the density of water has an inverse inertia near 1900, well past Q24's
/// ceiling of 128. Rather than saturate entry by entry -- which would break
/// the symmetry and positive-definiteness the solver relies on, and can make
/// an effective mass indefinite -- the whole tensor is scaled down uniformly
/// when any entry would overflow. Uniform scaling of I^-1 is exactly a uniform
/// scaling of I, so the body simply behaves as though it had more rotational
/// inertia than requested. That is the same trade, in the same direction, as
/// B3_MIN_MASS_RAW makes for linear mass.
///
/// Returns the zero matrix for a singular or non-positive-definite tensor,
/// which is also what a fixed-rotation body wants.
B3_API b3MatrixW b3InvertInertia( b3Matrix3 unitInertia, b3f mass );

// =========================================================================
// Transforms
// =========================================================================

B3_API b3Vec3 b3TransformPoint( b3Transform t, b3Vec3 p );

B3_INLINE b3Vec3 b3InvTransformPoint( b3Transform t, b3Vec3 p )
{
	return b3InvRotateVector( t.q, b3Sub( p, t.p ) );
}

B3_INLINE b3Transform b3MulTransforms( b3Transform a, b3Transform b )
{
	b3Transform r;
	r.q = b3MulQuat( a.q, b.q );
	r.p = b3Add( b3RotateVector( a.q, b.p ), a.p );
	return r;
}

B3_INLINE b3Transform b3InvMulTransforms( b3Transform a, b3Transform b )
{
	b3Transform r;
	r.q = b3InvMulQuat( a.q, b.q );
	r.p = b3InvRotateVector( a.q, b3Sub( b.p, a.p ) );
	return r;
}

B3_INLINE b3Transform b3InvertTransform( b3Transform t )
{
	b3Transform r;
	r.q = b3Conjugate( t.q );
	r.p = b3Neg( b3InvRotateVector( t.q, t.p ) );
	return r;
}

B3_INLINE bool b3IsValidTransform( b3Transform t )
{
	return b3IsValidVec3( t.p ) && b3IsValidQuat( t.q );
}

// Large world mode does not exist in the port, so these are the identity.
#define b3ToPos( v ) ( v )
#define b3ToVec3( p ) ( p )
#define b3SubPos( a, b ) b3Sub( a, b )
#define b3OffsetPos( p, v ) b3Add( p, v )
#define b3TransformWorldPoint( t, p ) b3TransformPoint( t, p )
#define b3InvTransformWorldPoint( t, p ) b3InvTransformPoint( t, p )
#define b3MulWorldTransforms( a, b ) b3MulTransforms( a, b )
#define b3InvMulWorldTransforms( a, b ) b3InvMulTransforms( a, b )
#define b3IsValidPosition( p ) b3IsValidVec3( p )
#define b3IsValidWorldTransform( t ) b3IsValidTransform( t )

// =========================================================================
// Bounding boxes
// =========================================================================

B3_INLINE b3AABB b3MakeAABB( b3Vec3 lower, b3Vec3 upper )
{
	b3AABB a = { lower, upper };
	return a;
}

/// Midpoint of a box.
///
/// The sum stays in int32, and that is safe **because of b3IsValidFloat**: a
/// valid b3f has |raw| < INT32_MAX/2, so the sum of two of them cannot overflow.
/// That invariant is what lets this be two instructions instead of a widening
/// pair, and it is the reason b3IsValidFloat's bound is where it is rather than
/// at the format's own limit.
///
/// It is also an invariant nothing enforced until b3WorldDef::maximumWorldExtent
/// existed: a body with nothing under it fell forever, left the valid range
/// after about eleven minutes, and in a release build -- where every
/// B3_ASSERT and therefore every b3IsValid* check compiles away -- carried on
/// with values this arithmetic is not correct for. See the note on that field.
B3_INLINE b3Vec3 b3AABB_Center( b3AABB a )
{
	// Halving by shift rather than a multiply: exact, and one instruction.
	return b3MakeVec3( b3Makeb3f( ( b3Raw( a.lowerBound.x ) + b3Raw( a.upperBound.x ) ) >> 1 ),
					   b3Makeb3f( ( b3Raw( a.lowerBound.y ) + b3Raw( a.upperBound.y ) ) >> 1 ),
					   b3Makeb3f( ( b3Raw( a.lowerBound.z ) + b3Raw( a.upperBound.z ) ) >> 1 ) );
}

B3_INLINE b3Vec3 b3AABB_Extents( b3AABB a )
{
	return b3MakeVec3( b3Makeb3f( ( b3Raw( a.upperBound.x ) - b3Raw( a.lowerBound.x ) ) >> 1 ),
					   b3Makeb3f( ( b3Raw( a.upperBound.y ) - b3Raw( a.lowerBound.y ) ) >> 1 ),
					   b3Makeb3f( ( b3Raw( a.upperBound.z ) - b3Raw( a.lowerBound.z ) ) >> 1 ) );
}

B3_INLINE b3AABB b3AABB_Union( b3AABB a, b3AABB b )
{
	return b3MakeAABB( b3Min( a.lowerBound, b.lowerBound ), b3Max( a.upperBound, b.upperBound ) );
}

B3_INLINE bool b3AABB_Contains( b3AABB a, b3AABB b )
{
	return b3Raw( a.lowerBound.x ) <= b3Raw( b.lowerBound.x ) && b3Raw( a.lowerBound.y ) <= b3Raw( b.lowerBound.y ) &&
		   b3Raw( a.lowerBound.z ) <= b3Raw( b.lowerBound.z ) && b3Raw( b.upperBound.x ) <= b3Raw( a.upperBound.x ) &&
		   b3Raw( b.upperBound.y ) <= b3Raw( a.upperBound.y ) && b3Raw( b.upperBound.z ) <= b3Raw( a.upperBound.z );
}

B3_INLINE bool b3AABB_Overlaps( b3AABB a, b3AABB b )
{
	return !( b3Raw( b.lowerBound.x ) > b3Raw( a.upperBound.x ) || b3Raw( b.lowerBound.y ) > b3Raw( a.upperBound.y ) ||
			  b3Raw( b.lowerBound.z ) > b3Raw( a.upperBound.z ) || b3Raw( a.lowerBound.x ) > b3Raw( b.upperBound.x ) ||
			  b3Raw( a.lowerBound.y ) > b3Raw( b.upperBound.y ) || b3Raw( a.lowerBound.z ) > b3Raw( b.upperBound.z ) );
}

/// Surface area, used as the dynamic tree's cost metric.
///
/// Returned wide because the product of three Q12 extents overflows Q12 for
/// boxes larger than a few units, and the tree compares these values against
/// each other rather than against a length.
B3_INLINE int64_t b3AABB_AreaWide( b3AABB a )
{
	int64_t wx = b3Raw( a.upperBound.x ) - b3Raw( a.lowerBound.x );
	int64_t wy = b3Raw( a.upperBound.y ) - b3Raw( a.lowerBound.y );
	int64_t wz = b3Raw( a.upperBound.z ) - b3Raw( a.lowerBound.z );
	return 2 * ( wx * wy + wy * wz + wz * wx );
}

B3_INLINE b3AABB b3AABB_Inflate( b3AABB a, b3f r )
{
	return b3MakeAABB( b3MakeVec3( b3SubF( a.lowerBound.x, r ), b3SubF( a.lowerBound.y, r ), b3SubF( a.lowerBound.z, r ) ),
					   b3MakeVec3( b3AddF( a.upperBound.x, r ), b3AddF( a.upperBound.y, r ), b3AddF( a.upperBound.z, r ) ) );
}

B3_INLINE b3AABB b3OffsetAABB( b3AABB a, b3Vec3 v )
{
	return b3MakeAABB( b3Add( a.lowerBound, v ), b3Add( a.upperBound, v ) );
}

B3_INLINE b3AABB b3AABB_AddPoint( b3AABB a, b3Vec3 point )
{
	return b3MakeAABB( b3Min( a.lowerBound, point ), b3Max( a.upperBound, point ) );
}

/// Transform an axis-aligned box. The result is the box of the rotated box,
/// which is larger than the box of the rotated contents.
B3_INLINE b3AABB b3AABB_Transform( b3Transform transform, b3AABB a )
{
	b3Vec3 center = b3TransformPoint( transform, b3AABB_Center( a ) );
	b3Matrix3 m = b3MakeMatrixFromQuat( transform.q );
	b3Vec3 extent = b3MulMV( b3AbsMatrix3( m ), b3AABB_Extents( a ) );
	return b3MakeAABB( b3Sub( center, extent ), b3Add( center, extent ) );
}

B3_INLINE bool b3IsValidAABB( b3AABB a )
{
	return b3IsValidVec3( a.lowerBound ) && b3IsValidVec3( a.upperBound ) &&
		   b3Raw( a.lowerBound.x ) <= b3Raw( a.upperBound.x ) && b3Raw( a.lowerBound.y ) <= b3Raw( a.upperBound.y ) &&
		   b3Raw( a.lowerBound.z ) <= b3Raw( a.upperBound.z );
}

B3_INLINE b3Vec3 b3ClosestPointToAABB( b3AABB a, b3Vec3 p )
{
	return b3Clamp( p, a.lowerBound, a.upperBound );
}

// =========================================================================
// Planes
// =========================================================================

/// Signed distance from a point to a plane. Negative is behind.
B3_INLINE b3f b3PlaneSeparation( b3Plane plane, b3Vec3 point )
{
	return b3SubF( b3Dot( plane.normal, point ), plane.offset );
}

/// Plane through a point with the given normal.
///
/// The normal is not required to be unit length. Where it is not, the plane
/// still classifies points correctly and b3PlaneSeparation still returns a
/// value with the right sign -- it is a distance scaled by |normal|, which is
/// all a clipper needs. See b3ClipSegmentToHullFace.
B3_INLINE b3Plane b3MakePlaneFromNormalAndPoint( b3Vec3 normal, b3Vec3 point )
{
	b3Plane plane = { normal, b3Dot( normal, point ) };
	return plane;
}

/// The plane through three points, with a unit normal.
///
/// The cross product goes through b3CrossDirection, and that is load bearing
/// rather than stylistic. b3Cross narrows each component back to Q12, and a
/// cross product is an **area**: the cross of two 0.05-unit edges is ten raw
/// units, and of two 0.01-unit edges it is exactly zero. Level meshes are
/// made of small triangles, so the narrow spelling would hand back a zero
/// normal for a perfectly good face. See the note on b3CrossWide.
B3_INLINE b3Plane b3MakePlaneFromPoints( b3Vec3 point1, b3Vec3 point2, b3Vec3 point3 )
{
	b3Vec3 normal = b3Normalize( b3CrossDirection( b3Sub( point2, point1 ), b3Sub( point3, point1 ) ) );
	b3Plane plane = { normal, b3Dot( normal, point1 ) };
	return plane;
}

/// normal2 = q * normal1, offset2 = offset1 + dot(normal2, p)
B3_INLINE b3Plane b3TransformPlane( b3Transform transform, b3Plane plane )
{
	b3Vec3 normal = b3RotateVector( transform.q, plane.normal );
	b3Plane out = { normal, b3AddF( plane.offset, b3Dot( normal, transform.p ) ) };
	return out;
}

/// Negative if p is below the triangle v1-v2-v3.
B3_INLINE b3f b3SignedVolume( b3Vec3 v1, b3Vec3 v2, b3Vec3 v3, b3Vec3 p )
{
	return b3Dot( b3Cross( b3Sub( v2, v1 ), b3Sub( v3, v1 ) ), b3Sub( p, v1 ) );
}

// =========================================================================
// Closest points
// =========================================================================
//
// Implemented in math_fixed.c. These run on wide intermediates throughout --
// see the note at the top of that file for why -- and return their positions
// along each segment as Q30 fractions rather than lengths, which is both what
// the callers want and the only scale a ratio of fourth powers lands in
// cleanly.

/// The closest points between two segments or infinite lines.
typedef struct b3SegmentDistanceResult
{
	b3Vec3 point1;
	b3c fraction1;
	b3Vec3 point2;
	b3c fraction2;
} b3SegmentDistanceResult;

/// True when both closest points lie strictly within their segments rather
/// than at an endpoint, i.e. the result describes an edge-edge feature.
B3_INLINE bool b3IsWithinSegments( const b3SegmentDistanceResult* result )
{
	return b3Raw( result->fraction1 ) >= 0 && b3Raw( result->fraction1 ) <= B3_C_ONE &&
		   b3Raw( result->fraction2 ) >= 0 && b3Raw( result->fraction2 ) <= B3_C_ONE;
}

/// Compute the closest point on the segment a-b to the target q.
B3_API b3Vec3 b3PointToSegmentDistance( b3Vec3 a, b3Vec3 b, b3Vec3 q );

/// Compute the closest points on two infinite lines.
B3_API b3SegmentDistanceResult b3LineDistance( b3Vec3 p1, b3Vec3 d1, b3Vec3 p2, b3Vec3 d2 );

/// Compute the closest points on two line segments.
B3_API b3SegmentDistanceResult b3SegmentDistance( b3Vec3 p1, b3Vec3 q1, b3Vec3 p2, b3Vec3 q2 );

/// The shortest rotation taking one unit vector to another.
B3_API b3Quat b3ComputeQuatBetweenUnitVectors( b3Vec3 v1, b3Vec3 v2 );

#endif // B3_MATH_FIXED_H__
