// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Erin Catto        (original Box3D)
// Copyright (c) 2026 Warioware64       (Nitro Engine Advanced fixed-point port)
//
// This file is part of Nitro Engine Advanced

/// @file   wheel_joint.c
/// @brief  The wheel joint: a suspension, a spin axis and a steering axis.
///
/// @section shape What a wheel joint actually is
///
/// The largest joint in the library, and the only one that is two joints at
/// once. Seven constraint blocks, solved in upstream's order exactly:
///
///   1. the **spin motor**, a scalar drive about the wheel's own axis;
///   2. the **suspension spring**, a scalar along the travel axis, applied even
///      during relax because it is a real spring;
///   3. the **steering spring**, a scalar about the derived steering axis;
///   4. the **two steering limits**, one-sided and speculative;
///   5. the **two suspension limits**, likewise;
///   6. the **collinearity** constraint holding the spin axis square to the
///      suspension -- a scalar when steering is enabled, a 2x2 when it is not;
///   7. the **point-to-line** 2x2 that holds the anchor on the travel axis.
///
/// Up to nine coupled impulses per sub-step, the most of any joint here. They
/// couple through the shared velocity state, which is why the order is kept.
///
/// @section axes Which axis is which, since it is easy to get backwards
///
///   - the **suspension travels along frame A's x** (`matrixA.cx`), as a
///     prismatic slides along frame A's x. A vertical suspension therefore
///     needs frame A rotated, exactly as a vertical slider does.
///   - the **wheel spins about frame B's z** (`matrixB.cz`), not A's. That is
///     what lets steering carry the spin axis with it.
///   - **steering is a twist about A's x**, measured by `cs = dot(B.cz, A.cz)`
///     and `ss = -dot(B.cz, A.cy)`.
///
/// Upstream's comment at its line 472 says "Rotation axis is the z-axis of body
/// A" while the code one line below uses `matrixB.cz`. The code is what is
/// ported; the comment is recorded here rather than carried, the same way the
/// prismatic's stale z/x header comment was.
///
/// @section scales What is new here, and what is not
///
/// Nearly every convention is Stage 2's, unchanged: the effective mass
/// accumulated wide and reciprocated once, `massScale` folded at Q30 into each
/// term, the clamp landing on the accumulator with the delta recomputed after,
/// one-sided limits through `b3MaxImp` with a speculative band. What this file
/// adds:
///
///   - **The steering axis, which is unbounded** -- the one genuinely new
///     precision problem of the stage. See @section steering.
///   - **Q30 frame columns** for the two dot products the steering angle is
///     built from, via b3QuatColumnsN. See @section angle.
///   - **No cached effective masses at all**, for the reason joint.h's
///     `@section nomass` sets out: upstream's cached suspension mass is built
///     from a different Jacobian than the one applied.
///   - **Two 2x2 blocks that need different multiplies.** The collinearity
///     block's rows vanish, so it takes the saturating b3MulSym2VSat; the
///     point-to-line block's arms are bounded by the geometry, so it takes
///     plain b3MulSym2V as the prismatic's does. See @section twotwo.
///
/// @section steering The steering axis, and why `den` is never formed
///
/// Upstream computes `den = 1/(cs^2 + ss^2)` and scales a cross product by it.
/// Writing `den0 = cs^2 + ss^2` and working the geometry through:
///
///     den0 = 1 - dot( A.cx, B.cz )^2        B.cz projected into A's y-z plane
///     |steeringAxis| = 1 / sqrt( den0 )
///
/// so upstream's axis is **unbounded**, growing as the wheel tips toward its
/// own suspension axis. In exact arithmetic that is benign -- the effective
/// mass collapses as `1/|axis|^2` and the applied impulse *shrinks*, so
/// steering fades out gracefully. In fixed point it is not: `den` cannot be a
/// b3f, which saturates at 128 and so is reached at `den0 = 1/128`, a wheel
/// tipped only 5.05 degrees from fully tipped. A saturated `den` scaling a
/// vector gives the wrong length and, once the components clip differently, an
/// arbitrary direction.
///
/// So `den` is never formed. The port follows the spherical joint's
/// normalize-and-carry-a-scale trade: `den0` is accumulated wide at Q60 and
/// never narrowed, the axis is normalized, and `sqrt(den0)` is carried
/// separately as a b3c coefficient.
///
/// The scale is needed because the velocity constraint is scale-invariant but
/// the **position bias is not**. With the unit Jacobian the effective
/// constraint function is `C_hat = C * sqrt(den0)`, so the scale *multiplies*
/// the spring error and both limit errors. It is at most one, so nothing can
/// grow -- which is the whole point of normalizing rather than dividing.
///
/// One consequence worth stating: with a unit Jacobian the clamp
/// `b3MulFTToImp( maxSteeringTorque, h )` bounds the **actual applied torque**.
/// Upstream's bound is on the coefficient along its long axis, so upstream's
/// effective torque bound *loosens* as the wheel tips. The port's bound means
/// what its name says. A deliberate divergence, recorded.
///
/// @section degenerate The one quantity that gates two blocks
///
/// The steering-on collinearity axis is `u = cross( B.cz, A.cx )`, whose
/// squared length is `den0` **exactly** -- the same number. So `den0` is
/// computed once per solve and gates both blocks.
///
/// The collinearity block keeps upstream's *unnormalized* `u`, unlike the
/// steering axis, and the asymmetry is not an oversight. Normalizing it would
/// make the bias worse rather than better: its Jacobian is *shorter* than unit,
/// so the effective constraint function would be `C / |u|`, which diverges
/// where the steering axis's `C * sqrt(den0)` shrinks. Both forms diverge as
/// `den0` goes to zero, and the honest answer for both is the same -- apply
/// nothing, because the wheel's spin axis has aligned with the axis it is meant
/// to be square to, and no angular velocity changes that error to first order.
///
/// The floor is `den0 < 1/64`, i.e. the wheel within 7.2 degrees of fully
/// tipped. Below it both blocks apply nothing. Without it, `b3RcpWide` on a
/// vanishing `k` runs into `B3_MIN_MASS_RAW`, which caps the reciprocal at a
/// mass of about 508,000 and **saturates rather than reporting failure** --
/// the spherical joint's measured disaster (spherical_joint.c:399-404) with a
/// bigger number. A divergence from upstream, with that precedent as its
/// defence.
///
/// @section angle Why the steering angle is built from Q30 columns
///
/// `steeringAngle = atan2( ss, cs )`, and `(cs, ss)` sit on a circle of radius
/// `sqrt(den0)` -- so a perturbation of size e in the frame axes gives an angle
/// error of roughly `5215 * e / sqrt(den0)` brads.
///
/// With Q12 axes e is about 5e-4, which is 2.6 brads with the wheel upright but
/// **333 brads -- 3.7 degrees -- near the degenerate end**. Built from Q30
/// columns through b3QuatColumnsN, e drops to about 1e-9 and the error stays
/// **under one brad** everywhere the guard admits. That is what makes the floor
/// above a statement about the *constraint* rather than about the angle
/// readout, and it is why b3QuatColumnsN was published at Stage 7.
///
/// The Q12 axes are still what the impulses are applied along, as in every
/// other joint. Only `cs`, `ss`, `den0` and the steering axis are built wide.
///
/// Contrast b3GetTwistAngle, which takes an arc tangent of a quaternion's z and
/// scalar: for a twist-only rotation those satisfy `z^2 + s^2 = 1`, so its
/// arguments lie on a *unit* circle by construction and its error is flat. It
/// folds the double cover, which this does not need; this needs a magnitude
/// guard, which it does not have. Complements, not substitutes.
///
/// @section twotwo Which 2x2 saturates, and why only one
///
/// The collinearity block, when steering is disabled, is the parallel joint's
/// block character for character -- b3CollinearityPerpAxes into
/// b3InvertPerpMass. Those rows have length `0.5 * sqrt(1 - (v.e)^2)`, which
/// **vanishes at a half turn**, and Stage 7 measured the resulting ask at
/// 301,342 N-s against Q15.16's ceiling of 32,768. So it takes b3MulSym2VSat,
/// which scales both components down together and preserves the direction the
/// clamp keeps.
///
/// The point-to-line block does not: its arms are `cross( d + rA, perp )` and
/// `cross( rB, perp )`, bounded by the geometry rather than vanishing, which is
/// why the prismatic still uses plain b3MulSym2V for the identical block.
///
/// @section upstream Four upstream defects in the reaction readouts
///
/// All four are in queries rather than in the solve, so none of them affects
/// the simulation -- and `run_pair` compares the reaction force by *magnitude*
/// and does not compare torque at all, so it is blind to three of them by
/// construction. test_world.c is what settles them.
///
///   1. `b3GetWheelJointForce` adds `lowerSuspensionLimit` -- **a metre** --
///      where `lowerSuspensionImpulse` is meant, and gets the upper term's
///      sign wrong too. Warm start uses `spring + lower - upper`; the readout
///      writes `... + upper + spring`. Invisible at the default lower limit of
///      zero, and it fabricates about 72 N at a lower limit of -0.3.
///   2. `b3GetWheelJointTorque` ignores the steering and collinearity impulses
///      entirely.
///   3. The same function reports the spin impulse along `matrixA.cz` when the
///      solve applied it along `matrixB.cz`. Indistinguishable only while the
///      wheel points straight ahead.
///   4. `b3GetWheelJointForce` **cyclically permutes** the force: the solve
///      applies the axial impulse along `cx` and the two perpendicular ones
///      along `cy` and `cz`, and the readout assembles `(perp.x, perp.y,
///      axial)`. The double rotation it performs is *correct*, unlike the
///      prismatic's torque bug -- the permutation is the defect.
///
/// The port fixes all four, and builds both readouts directly from the axes the
/// solve used rather than from a joint-space intermediate, so a permutation is
/// structurally impossible.
///
/// @section absent What is not here
///
/// b3DrawWheelJoint, every B3_REC hook and the `#if 0` dump block, as in every
/// other joint file. Also **`enableSteeringMotor`**, which upstream declares in
/// its sim struct and never reads or writes anywhere -- noticed rather than
/// lost.
///
/// b3WheelJoint_GetSuspensionTranslation is a port *addition*: upstream has no
/// equivalent, and a suspension whose travel cannot be read is hard to tune or
/// to instrument. It is the prismatic's GetTranslation on this joint's axis.

#include "joint.h"

#include "body.h"
#include "core.h"
#include "physics_world.h"
#include "solver.h"
#include "solver_set.h"

#include "box3d/box3d.h"

/// A quarter, as a Q30 coefficient. Both speculative bands are built with it.
#define B3_WHEEL_QUARTER b3cFromFrac( 1, 4 )

/// The floor on `den0 = 1 - dot(A.cx, B.cz)^2`, at Q60.
///
/// One sixty-fourth, so the wheel is within 7.2 degrees of fully tipped. Below
/// it the steering block and the steering-on collinearity block both apply
/// nothing -- see @section degenerate.
#define B3_WHEEL_MIN_DEN0 ( (int64_t)1 << 54 )

// =========================================================================
// Q30 helpers, file-local
// =========================================================================
//
// The steering geometry is the only place in the port that needs a dot and a
// cross of two Q30 directions. b3DotQuat is the shape both follow.

static b3n b3DotDir3N( b3Dir3 a, b3Dir3 b )
{
	return b3AddN( b3AddN( b3MulNN( a.x, b.x ), b3MulNN( a.y, b.y ) ), b3MulNN( a.z, b.z ) );
}

static b3Dir3 b3CrossDir3N( b3Dir3 a, b3Dir3 b )
{
	return b3MakeDir3( b3SubN( b3MulNN( a.y, b.z ), b3MulNN( a.z, b.y ) ),
					   b3SubN( b3MulNN( a.z, b.x ), b3MulNN( a.x, b.z ) ),
					   b3SubN( b3MulNN( a.x, b.y ), b3MulNN( a.y, b.x ) ) );
}

/// The steering geometry, computed once and used by three blocks.
///
/// Everything here is derived from the two joint frame rotations, so it is the
/// same whether the caller is the solve or a query outside the step.
typedef struct b3WheelSteering
{
	/// The normalized steering axis. Zero when degenerate.
	b3Vec3 axis;

	/// `sqrt(den0)`, the factor the position bias must carry because the axis
	/// was normalized. Zero when degenerate. At most one.
	b3c scale;

	/// `cross( B.cz, A.cx )` at Q12 -- upstream's unnormalized collinearity
	/// axis, kept unnormalized for the reason @section degenerate gives.
	b3Vec3 collinearityAxis;

	/// The steering angle in brads, from the Q30 dots.
	b3a angle;

	/// False when `den0` is under the floor: both steering and the steering-on
	/// collinearity block must then apply nothing.
	bool valid;
} b3WheelSteering;

/// Build the steering geometry from the two world frame rotations.
static b3WheelSteering b3WheelSteeringFrame( b3Quat quatA, b3Quat quatB, b3Vec3 axisAx )
{
	b3WheelSteering out;
	out.axis = b3Vec3_zero;
	out.scale = b3c_zero;
	out.collinearityAxis = b3Vec3_zero;
	out.angle = 0;
	out.valid = false;

	// Q30 columns, not the Q12 axes the impulses use. @section angle explains
	// the 333-brad difference this makes near the degenerate end.
	b3Dir3 colsA[3], colsB[3];
	b3QuatColumnsN( quatA, colsA );
	b3QuatColumnsN( quatB, colsB );

	b3Dir3 ay = colsA[1];
	b3Dir3 az = colsA[2];
	b3Dir3 bz = colsB[2];

	b3n cs = b3DotDir3N( bz, az );
	b3n ss = b3NegN( b3DotDir3N( bz, ay ) );

	// den0 at Q60, wide and never narrowed. Two Q30 squares, so at most 2^61 --
	// comfortably inside int64.
	int64_t csRaw = (int64_t)b3Raw( cs );
	int64_t ssRaw = (int64_t)b3Raw( ss );
	int64_t den0 = csRaw * csRaw + ssRaw * ssRaw;

	// The collinearity axis is upstream's, at Q12, and is built whether or not
	// the steering block will run -- the steering-off branch does not use it,
	// but the steering-on one does and shares this guard.
	out.collinearityAxis = b3Cross( b3FromDir3( bz ), axisAx );

	if ( den0 < B3_WHEEL_MIN_DEN0 )
	{
		return out;
	}

	// The angle from the wide dots. b3Atan2Raw takes raw values at a common
	// scale and depends only on their ratio, which is why the Q30 pair goes in
	// directly rather than through b3Atan2F's Q12.
	out.angle = b3Atan2Raw( b3Raw( ss ), b3Raw( cs ) );

	// w = -cs * A.cy - ss * A.cz, then axis = cross( B.cz, w ), all at Q30.
	// The result has magnitude sqrt(den0) <= 1, so nothing here can overflow.
	b3Dir3 w = b3MakeDir3( b3SubN( b3NegN( b3MulNN( cs, ay.x ) ), b3MulNN( ss, az.x ) ),
						   b3SubN( b3NegN( b3MulNN( cs, ay.y ) ), b3MulNN( ss, az.y ) ),
						   b3SubN( b3NegN( b3MulNN( cs, ay.z ) ), b3MulNN( ss, az.z ) ) );

	b3Dir3 axisRaw = b3CrossDir3N( bz, w );

	// Conditioned wide and then normalized, rather than narrowed to Q12 first.
	// The vector is as short as sqrt(den0), so at the floor a Q12 copy would
	// have five significant bits left; b3DirectionFromWide rescales the largest
	// component into [1, 2) units first, which is exactly what it exists for.
	b3Vec3 conditioned =
		b3DirectionFromWide( (int64_t)b3Raw( axisRaw.x ), (int64_t)b3Raw( axisRaw.y ), (int64_t)b3Raw( axisRaw.z ) );
	b3Vec3 unitAxis = b3Normalize( conditioned );

	if ( b3IsNormalized( unitAxis ) == false )
	{
		return out;
	}

	// sqrt of a Q60 value is Q30, which is a b3c exactly -- and it is at most
	// one, so it can never saturate.
	out.axis = unitAxis;
	out.scale = b3Makeb3c( (int32_t)b3HwSqrt64( (uint64_t)den0 ) );
	out.valid = true;
	return out;
}

// =========================================================================
// Accessors
// =========================================================================

void b3WheelJoint_EnableSuspensionSpring( b3JointId jointId, bool enableSpring )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_wheelJoint );
	if ( enableSpring != base->wheelJoint.enableSuspensionSpring )
	{
		base->wheelJoint.enableSuspensionSpring = enableSpring;
		base->wheelJoint.suspensionSpringImpulse = b3imp_zero;
	}
}

bool b3WheelJoint_IsSuspensionSpringEnabled( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_wheelJoint );
	return base->wheelJoint.enableSuspensionSpring;
}

void b3WheelJoint_SetSuspensionHertz( b3JointId jointId, b3f hertz )
{
	B3_ASSERT( b3IsValidFloat( hertz ) && b3Raw( hertz ) >= 0 );

	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_wheelJoint );
	base->wheelJoint.suspensionHertz = hertz;
}

b3f b3WheelJoint_GetSuspensionHertz( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_wheelJoint );
	return base->wheelJoint.suspensionHertz;
}

void b3WheelJoint_SetSuspensionDampingRatio( b3JointId jointId, b3f dampingRatio )
{
	B3_ASSERT( b3IsValidFloat( dampingRatio ) && b3Raw( dampingRatio ) >= 0 );

	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_wheelJoint );
	base->wheelJoint.suspensionDampingRatio = dampingRatio;
}

b3f b3WheelJoint_GetSuspensionDampingRatio( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_wheelJoint );
	return base->wheelJoint.suspensionDampingRatio;
}

void b3WheelJoint_EnableSuspensionLimit( b3JointId jointId, bool enableLimit )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_wheelJoint );
	if ( enableLimit != base->wheelJoint.enableSuspensionLimit )
	{
		base->wheelJoint.enableSuspensionLimit = enableLimit;
		base->wheelJoint.lowerSuspensionImpulse = b3imp_zero;
		base->wheelJoint.upperSuspensionImpulse = b3imp_zero;
	}
}

bool b3WheelJoint_IsSuspensionLimitEnabled( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_wheelJoint );
	return base->wheelJoint.enableSuspensionLimit;
}

void b3WheelJoint_SetSuspensionLimits( b3JointId jointId, b3f lower, b3f upper )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_wheelJoint );
	b3WheelJoint* joint = &base->wheelJoint;

	// Sorted rather than asserted, as every other range in the port is.
	joint->lowerSuspensionLimit = b3Raw( lower ) < b3Raw( upper ) ? lower : upper;
	joint->upperSuspensionLimit = b3Raw( lower ) < b3Raw( upper ) ? upper : lower;

	joint->lowerSuspensionImpulse = b3imp_zero;
	joint->upperSuspensionImpulse = b3imp_zero;
}

b3f b3WheelJoint_GetLowerSuspensionLimit( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_wheelJoint );
	return base->wheelJoint.lowerSuspensionLimit;
}

b3f b3WheelJoint_GetUpperSuspensionLimit( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_wheelJoint );
	return base->wheelJoint.upperSuspensionLimit;
}

void b3WheelJoint_EnableSpinMotor( b3JointId jointId, bool enableMotor )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_wheelJoint );
	if ( enableMotor != base->wheelJoint.enableSpinMotor )
	{
		base->wheelJoint.enableSpinMotor = enableMotor;
		base->wheelJoint.spinImpulse = b3imp_zero;
	}
}

bool b3WheelJoint_IsSpinMotorEnabled( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_wheelJoint );
	return base->wheelJoint.enableSpinMotor;
}

void b3WheelJoint_SetSpinMotorSpeed( b3JointId jointId, b3f speed )
{
	B3_ASSERT( b3IsValidFloat( speed ) );

	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_wheelJoint );
	base->wheelJoint.spinSpeed = speed;
}

b3f b3WheelJoint_GetSpinMotorSpeed( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_wheelJoint );
	return base->wheelJoint.spinSpeed;
}

void b3WheelJoint_SetMaxSpinTorque( b3JointId jointId, b3f torque )
{
	B3_ASSERT( b3IsValidFloat( torque ) );

	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_wheelJoint );
	base->wheelJoint.maxSpinTorque = b3MaxF( b3f_zero, torque );
}

b3f b3WheelJoint_GetMaxSpinTorque( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_wheelJoint );
	return base->wheelJoint.maxSpinTorque;
}

void b3WheelJoint_EnableSteering( b3JointId jointId, bool enableSteering )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_wheelJoint );
	if ( enableSteering != base->wheelJoint.enableSteering )
	{
		base->wheelJoint.enableSteering = enableSteering;
		base->wheelJoint.steeringSpringImpulse = b3imp_zero;
		base->wheelJoint.lowerSteeringImpulse = b3imp_zero;
		base->wheelJoint.upperSteeringImpulse = b3imp_zero;

		// The collinearity block changes shape with this flag -- a scalar when
		// steering is on, a 2x2 when it is off -- so its accumulator means a
		// different thing on either side and must not carry across.
		base->wheelJoint.angularImpulse = b3Imp2_zero;
	}
}

bool b3WheelJoint_IsSteeringEnabled( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_wheelJoint );
	return base->wheelJoint.enableSteering;
}

void b3WheelJoint_SetSteeringHertz( b3JointId jointId, b3f hertz )
{
	B3_ASSERT( b3IsValidFloat( hertz ) && b3Raw( hertz ) >= 0 );

	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_wheelJoint );
	base->wheelJoint.steeringHertz = hertz;
}

b3f b3WheelJoint_GetSteeringHertz( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_wheelJoint );
	return base->wheelJoint.steeringHertz;
}

void b3WheelJoint_SetSteeringDampingRatio( b3JointId jointId, b3f dampingRatio )
{
	B3_ASSERT( b3IsValidFloat( dampingRatio ) && b3Raw( dampingRatio ) >= 0 );

	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_wheelJoint );
	base->wheelJoint.steeringDampingRatio = dampingRatio;
}

b3f b3WheelJoint_GetSteeringDampingRatio( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_wheelJoint );
	return base->wheelJoint.steeringDampingRatio;
}

void b3WheelJoint_SetMaxSteeringTorque( b3JointId jointId, b3f torque )
{
	B3_ASSERT( b3IsValidFloat( torque ) );

	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_wheelJoint );
	base->wheelJoint.maxSteeringTorque = b3MaxF( b3f_zero, torque );
}

b3f b3WheelJoint_GetMaxSteeringTorque( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_wheelJoint );
	return base->wheelJoint.maxSteeringTorque;
}

void b3WheelJoint_SetTargetSteeringAngle( b3JointId jointId, b3a angle )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_wheelJoint );
	base->wheelJoint.targetSteeringAngle = angle;
}

b3a b3WheelJoint_GetTargetSteeringAngle( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_wheelJoint );
	return base->wheelJoint.targetSteeringAngle;
}

void b3WheelJoint_EnableSteeringLimit( b3JointId jointId, bool enableLimit )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_wheelJoint );
	if ( enableLimit != base->wheelJoint.enableSteeringLimit )
	{
		base->wheelJoint.enableSteeringLimit = enableLimit;
		base->wheelJoint.lowerSteeringImpulse = b3imp_zero;
		base->wheelJoint.upperSteeringImpulse = b3imp_zero;
	}
}

bool b3WheelJoint_IsSteeringLimitEnabled( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_wheelJoint );
	return base->wheelJoint.enableSteeringLimit;
}

void b3WheelJoint_SetSteeringLimits( b3JointId jointId, b3a lower, b3a upper )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_wheelJoint );
	b3WheelJoint* joint = &base->wheelJoint;

	// Sorted, and both accumulators cleared. Upstream asserts the order and
	// clears only in EnableSteeringLimit -- an asymmetry the prismatic's
	// SetLimits already resolved this way one stage ago.
	joint->lowerSteeringLimit = lower < upper ? lower : upper;
	joint->upperSteeringLimit = lower < upper ? upper : lower;

	joint->lowerSteeringImpulse = b3imp_zero;
	joint->upperSteeringImpulse = b3imp_zero;
}

b3a b3WheelJoint_GetLowerSteeringLimit( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_wheelJoint );
	return base->wheelJoint.lowerSteeringLimit;
}

b3a b3WheelJoint_GetUpperSteeringLimit( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_wheelJoint );
	return base->wheelJoint.upperSteeringLimit;
}

// =========================================================================
// Derived queries
// =========================================================================

/// The joint's live world frame, for the queries that run outside a step.
///
/// The prismatic's b3PrismaticFrame with both frame rotations kept, because
/// this joint's spin and steering axes live in frame B.
static void b3WheelFrame( b3World* world, b3JointSim* base, b3Quat* outQA, b3Quat* outQB, b3Vec3* outD, b3Vec3* outRA,
						  b3Vec3* outRB )
{
	b3WorldTransform transformA = b3GetBodyTransform( world, base->bodyIdA );
	b3WorldTransform transformB = b3GetBodyTransform( world, base->bodyIdB );

	b3Vec3 rA = b3RotateVector( transformA.q, base->localFrameA.p );
	b3Vec3 rB = b3RotateVector( transformB.q, base->localFrameB.p );

	*outQA = b3MulQuat( transformA.q, base->localFrameA.q );
	*outQB = b3MulQuat( transformB.q, base->localFrameB.q );
	*outD = b3Add( b3Sub( transformB.p, transformA.p ), b3Sub( rB, rA ) );
	*outRA = rA;
	*outRB = rB;
}

b3f b3WheelJoint_GetSuspensionTranslation( b3JointId jointId )
{
	b3World* world = b3GetUnlockedWorld( jointId.world0 );
	if ( world == NULL )
	{
		return b3f_zero;
	}

	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_wheelJoint );

	b3Quat qA, qB;
	b3Vec3 d, rA, rB;
	b3WheelFrame( world, base, &qA, &qB, &d, &rA, &rB );

	return b3Dot( d, b3RotateVector( qA, b3Vec3_axisX ) );
}

b3f b3WheelJoint_GetSpinSpeed( b3JointId jointId )
{
	b3World* world = b3GetUnlockedWorld( jointId.world0 );
	if ( world == NULL )
	{
		return b3f_zero;
	}

	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_wheelJoint );

	b3Quat qA, qB;
	b3Vec3 d, rA, rB;
	b3WheelFrame( world, base, &qA, &qB, &d, &rA, &rB );

	// A sleeping or static body has no b3BodyState, and zero is its velocity.
	b3Body* bodyA = b3Array_Get( world->bodies, base->bodyIdA );
	b3Body* bodyB = b3Array_Get( world->bodies, base->bodyIdB );
	b3BodyState* stateA = b3GetBodyState( world, bodyA );
	b3BodyState* stateB = b3GetBodyState( world, bodyB );

	b3Vec3 wA = stateA ? stateA->angularVelocity : b3Vec3_zero;
	b3Vec3 wB = stateB ? stateB->angularVelocity : b3Vec3_zero;

	// About frame B's z, which is where the wheel actually spins.
	return b3Dot( b3Sub( wB, wA ), b3RotateVector( qB, b3Vec3_axisZ ) );
}

b3a b3WheelJoint_GetSteeringAngle( b3JointId jointId )
{
	b3World* world = b3GetUnlockedWorld( jointId.world0 );
	if ( world == NULL )
	{
		return 0;
	}

	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_wheelJoint );

	b3Quat qA, qB;
	b3Vec3 d, rA, rB;
	b3WheelFrame( world, base, &qA, &qB, &d, &rA, &rB );

	// The same geometry the solve uses, so a caller reading this and a caller
	// setting a limit are talking about one number.
	b3WheelSteering steering = b3WheelSteeringFrame( qA, qB, b3RotateVector( qA, b3Vec3_axisX ) );
	return steering.angle;
}

b3f b3WheelJoint_GetSpinTorque( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_wheelJoint );
	return b3MulImpFToF( base->wheelJoint.spinImpulse, world->inv_h );
}

b3f b3WheelJoint_GetSteeringTorque( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_wheelJoint );
	b3WheelJoint* joint = &base->wheelJoint;

	// The **spring alone**, as upstream reports it, and the limits deliberately
	// excluded rather than overlooked.
	//
	// Combining them was tried and reverted, because the combined number is
	// worse: a wheel held against a steering stop is in equilibrium, so the
	// spring pushing into the limit and the limit pushing back very nearly
	// cancel. Measured on such a scene, the sum came to 0.0037 N-m while the
	// spring was spending its whole 500 N-m budget -- a reading that says
	// "nothing is happening" about a joint under maximum load.
	//
	// A caller who wants the *net* torque about every axis already has
	// b3Joint_GetConstraintTorque, which does include the limits (see
	// b3GetWheelJointTorque). This accessor answers the narrower question its
	// name implies: how hard the steering spring is pulling.
	return b3MulImpFToF( joint->steeringSpringImpulse, world->inv_h );
}

/// How far the wheel's rotation has left the configuration it constrains, in
/// brads. The wheel case of b3Joint_GetAngularSeparation.
///
/// **A port addition.** Upstream's angular-separation switch reaches the wheel
/// and gives up: `// todo`, then `B3_ASSERT( false )` and zero
/// (`helpSrc/box3d-main/src/joint.c:1413`). A public query that asserts in debug
/// and lies in release is not shippable, so the port answers it, in the shape
/// upstream's *spherical* case already uses -- an error term plus the excess
/// over whichever limits are enabled.
///
/// Two terms, and they are the two rotational things the joint actually holds:
///
///   - **collinearity.** The spin axis B.cz is constrained to stay in A's y-z
///     plane, so its component along A.cx is the whole of the error and the arc
///     sine of that component is it as an angle. Read from the Q30 columns for
///     the reason @section angle gives about the Q12 ones.
///   - **the steering limits**, when enabled, as the excess past whichever stop
///     is exceeded -- the same form the suspension limits take on the linear
///     side.
///
/// The suspension spring is deliberately absent: it is a *linear* degree of
/// freedom and belongs to b3Joint_GetLinearSeparation. So is the steering
/// spring, which is a target rather than a constraint -- a wheel steered away
/// from its target is doing its job, not separating.
///
/// Accumulated in int32_t and saturated at a half turn, because two terms of up
/// to a half turn each overflow the b3a this returns, and a separation past a
/// half turn carries no more information than one at it.
b3a b3GetWheelJointAngularSeparation( b3World* world, b3JointSim* base )
{
	b3WheelJoint* joint = &base->wheelJoint;

	b3Quat qA, qB;
	b3Vec3 d, rA, rB;
	b3WheelFrame( world, base, &qA, &qB, &d, &rA, &rB );

	b3Dir3 colsA[3], colsB[3];
	b3QuatColumnsN( qA, colsA );
	b3QuatColumnsN( qB, colsB );

	// dot( A.cx, B.cz ) is the sine of the out-of-plane angle: both are unit, so
	// the dot is in [-1, 1] and b3AsinC cannot be handed an out-of-range Q30.
	b3a outOfPlane = b3AsinC( b3NToC( b3DotDir3N( colsA[0], colsB[2] ) ) );
	int32_t sum = outOfPlane < 0 ? -(int32_t)outOfPlane : (int32_t)outOfPlane;

	if ( joint->enableSteeringLimit )
	{
		b3WheelSteering steering = b3WheelSteeringFrame( qA, qB, b3RotateVector( qA, b3Vec3_axisX ) );

		// A degenerate steering frame reports no steering excess rather than a
		// fabricated one: with the axis dead there is no meaningful angle to
		// compare against the stops. The collinearity term above stands on its
		// own and is exactly what is large in that configuration.
		if ( steering.valid )
		{
			if ( steering.angle < joint->lowerSteeringLimit )
			{
				sum += (int32_t)joint->lowerSteeringLimit - (int32_t)steering.angle;
			}

			if ( steering.angle > joint->upperSteeringLimit )
			{
				sum += (int32_t)steering.angle - (int32_t)joint->upperSteeringLimit;
			}
		}
	}

	const int32_t halfTurn = B3_BRAD_CIRCLE / 2;
	return (b3a)( sum > halfTurn ? halfTurn : sum );
}

// =========================================================================
// Reaction readouts
// =========================================================================

/// The force the joint is applying, in world space.
///
/// **Built directly from the axes the solve used.** Upstream assembles a
/// joint-space vector and rotates it, and gets the component order wrong doing
/// it -- the solve applies the axial impulse along `cx` and the two
/// perpendicular ones along `cy` and `cz`, while the readout writes
/// `(perp.x, perp.y, axial)`. Constructing the world vector directly makes that
/// permutation impossible to write. See @section upstream, defect 4.
///
/// The axial term is `spring + lower - upper`, which is what warm start
/// replays. Upstream writes `... + upper + spring` **and** uses
/// `lowerSuspensionLimit` -- a length -- in place of `lowerSuspensionImpulse`.
/// See @section upstream, defect 1.
b3Vec3 b3GetWheelJointForce( b3World* world, b3JointSim* base )
{
	b3WheelJoint* joint = &base->wheelJoint;

	b3Quat qA, qB;
	b3Vec3 d, rA, rB;
	b3WheelFrame( world, base, &qA, &qB, &d, &rA, &rB );

	b3Vec3 axisX = b3RotateVector( qA, b3Vec3_axisX );
	b3Vec3 axisY = b3RotateVector( qA, b3Vec3_axisY );
	b3Vec3 axisZ = b3RotateVector( qA, b3Vec3_axisZ );

	b3imp axial = b3AddImp( joint->suspensionSpringImpulse,
							b3SubImp( joint->lowerSuspensionImpulse, joint->upperSuspensionImpulse ) );

	b3Imp3 p = b3AddImp3( b3MulImpV( axial, axisX ),
						  b3AddImp3( b3MulImpV( joint->linearImpulse.x, axisY ),
									 b3MulImpV( joint->linearImpulse.y, axisZ ) ) );

	b3f inv_h = world->inv_h;
	return b3MakeVec3( b3MulImpFToF( p.x, inv_h ), b3MulImpFToF( p.y, inv_h ), b3MulImpFToF( p.z, inv_h ) );
}

/// The torque the joint is applying, in world space.
///
/// Every angular impulse the solve applied, along the axis it applied it on.
/// Upstream reports the spin impulse alone, along the wrong body's z. See
/// @section upstream, defects 2 and 3.
///
/// The suspension and point-to-line terms are **excluded**, and that is not an
/// omission: their arms are `cross( d + rA, axis )` for body A and
/// `cross( rB, axis )` for body B, which differ, so what they apply is not a
/// single torque the joint can be said to exert. The revolute's and prismatic's
/// readouts exclude the analogous terms for the same reason.
b3Vec3 b3GetWheelJointTorque( b3World* world, b3JointSim* base )
{
	b3WheelJoint* joint = &base->wheelJoint;

	b3Quat qA, qB;
	b3Vec3 d, rA, rB;
	b3WheelFrame( world, base, &qA, &qB, &d, &rA, &rB );

	b3Vec3 axisAx = b3RotateVector( qA, b3Vec3_axisX );
	b3Vec3 spinAxis = b3RotateVector( qB, b3Vec3_axisZ );

	// The spin impulse goes along frame **B's** z, which is where it was
	// applied -- upstream reports it along A's z, indistinguishable only while
	// the wheel points straight ahead.
	b3Imp3 t = b3MulImpV( joint->spinImpulse, spinAxis );

	if ( joint->enableSteering )
	{
		b3WheelSteering steering = b3WheelSteeringFrame( qA, qB, axisAx );

		b3imp steeringImpulse = b3AddImp( joint->steeringSpringImpulse,
										  b3SubImp( joint->lowerSteeringImpulse, joint->upperSteeringImpulse ) );

		t = b3AddImp3( t, b3MulImpV( steeringImpulse, steering.axis ) );
		t = b3AddImp3( t, b3MulImpV( joint->angularImpulse.x, steering.collinearityAxis ) );
	}
	else
	{
		b3Quat relQ = b3InvMulQuat( qA, qB );
		b3Vec3 perpAxisX, perpAxisY;
		b3CollinearityPerpAxes( qA, relQ, &perpAxisX, &perpAxisY );

		t = b3AddImp3( t, b3MulImpV( joint->angularImpulse.x, perpAxisX ) );
		t = b3AddImp3( t, b3MulImpV( joint->angularImpulse.y, perpAxisY ) );
	}

	b3f inv_h = world->inv_h;
	return b3MakeVec3( b3MulImpFToF( t.x, inv_h ), b3MulImpFToF( t.y, inv_h ), b3MulImpFToF( t.z, inv_h ) );
}

// =========================================================================
// Solver
// =========================================================================

void b3PrepareWheelJoint( b3JointSim* base, b3StepContext* context )
{
	B3_ASSERT( base->type == b3_wheelJoint );

	b3World* world = context->world;

	b3Body* bodyA = b3Array_Get( world->bodies, base->bodyIdA );
	b3Body* bodyB = b3Array_Get( world->bodies, base->bodyIdB );

	B3_ASSERT( bodyA->setIndex == b3_awakeSet || bodyB->setIndex == b3_awakeSet );

	b3SolverSet* setA = b3Array_Get( world->solverSets, bodyA->setIndex );
	b3SolverSet* setB = b3Array_Get( world->solverSets, bodyB->setIndex );

	int localIndexA = bodyA->localIndex;
	int localIndexB = bodyB->localIndex;

	b3BodySim* bodySimA = b3Array_Get( setA->bodySims, localIndexA );
	b3BodySim* bodySimB = b3Array_Get( setB->bodySims, localIndexB );

	base->invMassA = bodySimA->invMass;
	base->invMassB = bodySimB->invMass;
	base->invIA = bodySimA->invInertiaWorld;
	base->invIB = bodySimB->invInertiaWorld;

	// The sum `invIA + invIB` is never formed anywhere in this file, for the
	// reason revolute_joint.c's prepare sets out at length.
	base->fixedRotation = b3FixedRotationFromMass( b3InvertRotationMass( base->invIA, base->invIB ) );

	b3WheelJoint* joint = &base->wheelJoint;

	joint->indexA = bodyA->setIndex == b3_awakeSet ? localIndexA : B3_NULL_INDEX;
	joint->indexB = bodyB->setIndex == b3_awakeSet ? localIndexB : B3_NULL_INDEX;

	// Joint frames in world space, position relative to the centre of mass --
	// one rotation of (localFrame.p - localCenter) rather than a transform and
	// a subtraction, which is where the bits go far from the origin.
	joint->frameA.q = b3MulQuat( bodySimA->transform.q, base->localFrameA.q );
	joint->frameA.p = b3RotateVector( bodySimA->transform.q, b3Sub( base->localFrameA.p, bodySimA->localCenter ) );
	joint->frameB.q = b3MulQuat( bodySimB->transform.q, base->localFrameB.q );
	joint->frameB.p = b3RotateVector( bodySimB->transform.q, b3Sub( base->localFrameB.p, bodySimB->localCenter ) );

	joint->deltaCenter = b3Sub( bodySimB->center, bodySimA->center );

	joint->suspensionSoftness = b3MakeSoft( joint->suspensionHertz, joint->suspensionDampingRatio, context->h );
	joint->steeringSoftness = b3MakeSoft( joint->steeringHertz, joint->steeringDampingRatio, context->h );

	// **No effective masses are computed here**, and no axes either. See
	// joint.h's `@section nomass` and `@section noaxes`: upstream caches three
	// masses, one of which is built from a different Jacobian than the one
	// applied, and every axis this joint uses is rebuilt by warm start anyway.

	if ( context->enableWarmStarting == false )
	{
		joint->linearImpulse = b3Imp2_zero;
		joint->angularImpulse = b3Imp2_zero;
		joint->spinImpulse = b3imp_zero;
		joint->suspensionSpringImpulse = b3imp_zero;
		joint->lowerSuspensionImpulse = b3imp_zero;
		joint->upperSuspensionImpulse = b3imp_zero;
		joint->steeringSpringImpulse = b3imp_zero;
		joint->lowerSteeringImpulse = b3imp_zero;
		joint->upperSteeringImpulse = b3imp_zero;
	}
}

/// @note In ITCM, with b3SolveWheelJoint below. See the note on that function.
///
/// 2,792 B, called once per wheel per sub-step -- sixteen times a frame for a
/// four-wheeled vehicle at the default sub-step count, which is 43 KB of
/// instruction fetch. It fits the instruction cache on its own, so it is the
/// cheaper half of the pair; it is here because it is called from the same
/// loop and would otherwise be what evicts its sibling.
void B3_ITCM_IF( B3_ITCM_WHEEL, b3WarmStartWheelJoint )( b3JointSim* base, b3StepContext* context )
{
	B3_ASSERT( base->type == b3_wheelJoint );

	b3iw mA = base->invMassA;
	b3iw mB = base->invMassB;
	b3MatrixW iA = base->invIA;
	b3MatrixW iB = base->invIB;

	b3BodyState dummyState = b3_identityBodyState;

	b3WheelJoint* joint = &base->wheelJoint;
	b3BodyState* stateA = joint->indexA == B3_NULL_INDEX ? &dummyState : context->states + joint->indexA;
	b3BodyState* stateB = joint->indexB == B3_NULL_INDEX ? &dummyState : context->states + joint->indexB;

	b3Vec3 rA = b3RotateVector( stateA->deltaRotation, joint->frameA.p );
	b3Vec3 rB = b3RotateVector( stateB->deltaRotation, joint->frameB.p );

	// deltaPosition is Q24, so the difference is taken there and narrowed once.
	b3Vec3 dp = b3W3ToVec3( b3SubW3( stateB->deltaPosition, stateA->deltaPosition ) );
	b3Vec3 d = b3Add( b3Add( dp, joint->deltaCenter ), b3Sub( rB, rA ) );

	b3Quat quatA = b3MulQuat( stateA->deltaRotation, joint->frameA.q );
	b3Quat quatB = b3MulQuat( stateB->deltaRotation, joint->frameB.q );
	if ( b3Raw( b3DotQuat( quatA, quatB ) ) < 0 )
	{
		quatB = b3NegateQuat( quatB );
	}

	// The axes at Q12, from b3RotateVector rather than a rotation matrix --
	// b3MakeMatrixFromQuat is not exactly orthonormal at Q12, and these are
	// what impulses are applied along.
	b3Vec3 axisAx = b3RotateVector( quatA, b3Vec3_axisX );
	b3Vec3 axisAy = b3RotateVector( quatA, b3Vec3_axisY );
	b3Vec3 axisAz = b3RotateVector( quatA, b3Vec3_axisZ );
	b3Vec3 spinAxis = b3RotateVector( quatB, b3Vec3_axisZ );

	b3Vec3 rAd = b3Add( d, rA );
	b3Vec3 sAx = b3Cross( rAd, axisAx );
	b3Vec3 sBx = b3Cross( rB, axisAx );
	b3Vec3 sAy = b3Cross( rAd, axisAy );
	b3Vec3 sBy = b3Cross( rB, axisAy );
	b3Vec3 sAz = b3Cross( rAd, axisAz );
	b3Vec3 sBz = b3Cross( rB, axisAz );

	b3imp suspensionImpulse = b3AddImp( joint->suspensionSpringImpulse,
										b3SubImp( joint->lowerSuspensionImpulse, joint->upperSuspensionImpulse ) );

	b3imp py = joint->linearImpulse.x;
	b3imp pz = joint->linearImpulse.y;

	b3Imp3 P = b3AddImp3( b3MulImpV( suspensionImpulse, axisAx ),
						  b3AddImp3( b3MulImpV( py, axisAy ), b3MulImpV( pz, axisAz ) ) );

	b3Imp3 LA = b3AddImp3( b3MulImpV( suspensionImpulse, sAx ),
						   b3AddImp3( b3MulImpV( py, sAy ), b3MulImpV( pz, sAz ) ) );
	b3Imp3 LB = b3AddImp3( b3MulImpV( suspensionImpulse, sBx ),
						   b3AddImp3( b3MulImpV( py, sBy ), b3MulImpV( pz, sBz ) ) );

	// The purely angular half, applied identically to both bodies.
	b3Imp3 angular = b3MulImpV( joint->spinImpulse, spinAxis );

	if ( joint->enableSteering )
	{
		b3WheelSteering steering = b3WheelSteeringFrame( quatA, quatB, axisAx );

		b3imp steeringImpulse = b3AddImp( joint->steeringSpringImpulse,
										  b3SubImp( joint->lowerSteeringImpulse, joint->upperSteeringImpulse ) );

		angular = b3AddImp3( angular, b3MulImpV( steeringImpulse, steering.axis ) );
		angular = b3AddImp3( angular, b3MulImpV( joint->angularImpulse.x, steering.collinearityAxis ) );
	}
	else
	{
		b3Quat relQ = b3InvMulQuat( quatA, quatB );
		b3Vec3 perpAxisX, perpAxisY;
		b3CollinearityPerpAxes( quatA, relQ, &perpAxisX, &perpAxisY );

		angular = b3AddImp3( angular, b3MulImpV( joint->angularImpulse.x, perpAxisX ) );
		angular = b3AddImp3( angular, b3MulImpV( joint->angularImpulse.y, perpAxisY ) );
	}

	b3Vec3 vA = b3Sub( stateA->linearVelocity, b3MulImpW3( P, mA ) );
	b3Vec3 wA = b3Sub( stateA->angularVelocity, b3MulMWImp( iA, b3AddImp3( LA, angular ) ) );

	b3Vec3 vB = b3Add( stateB->linearVelocity, b3MulImpW3( P, mB ) );
	b3Vec3 wB = b3Add( stateB->angularVelocity, b3MulMWImp( iB, b3AddImp3( LB, angular ) ) );

	if ( stateA->flags & b3_dynamicFlag )
	{
		stateA->linearVelocity = vA;
		stateA->angularVelocity = wA;
	}

	if ( stateB->flags & b3_dynamicFlag )
	{
		stateB->linearVelocity = vB;
		stateB->angularVelocity = wB;
	}
}

/// @note In ITCM, and the reason the tier exists.
///
/// At 11,312 B this is the only function in the engine that does not fit the
/// ARM9's 8 KiB instruction cache, so every call streams the whole body from
/// main RAM rather than re-using what the previous call left behind. The
/// sub-step loop in solver.c calls it twice per sub-step per joint -- biased
/// then relaxed -- which for four wheels at four sub-steps is thirty-two calls
/// and roughly 354 KB of instruction fetch per frame, 77% of the engine's
/// total and on the order of a quarter of the frame budget spent on fetch
/// stalls alone.
///
/// Splitting it was tried on paper first and rejected: the five optional
/// blocks below are gated on `enable*` flags, but a vehicle that steers and
/// has suspension turns all five on, so the split moves no bytes. ITCM does,
/// because it removes the fetch rather than reducing it.
///
/// This costs 11 KB of a 32 KB budget shared with the whole ROM, which is why
/// it is not unconditional -- see NEA_BOX3D_NO_ITCM in Makefile.blocksds.
void B3_ITCM_IF( B3_ITCM_WHEEL, b3SolveWheelJoint )( b3JointSim* base, b3StepContext* context, bool useBias )
{
	B3_ASSERT( base->type == b3_wheelJoint );

	b3iw mA = base->invMassA;
	b3iw mB = base->invMassB;
	b3MatrixW iA = base->invIA;
	b3MatrixW iB = base->invIB;

	b3BodyState dummyState = b3_identityBodyState;

	b3WheelJoint* joint = &base->wheelJoint;
	b3BodyState* stateA = joint->indexA == B3_NULL_INDEX ? &dummyState : context->states + joint->indexA;
	b3BodyState* stateB = joint->indexB == B3_NULL_INDEX ? &dummyState : context->states + joint->indexB;

	b3Vec3 vA = stateA->linearVelocity;
	b3Vec3 wA = stateA->angularVelocity;
	b3Vec3 vB = stateB->linearVelocity;
	b3Vec3 wB = stateB->angularVelocity;

	bool fixedRotation = base->fixedRotation;

	b3Vec3 rA = b3RotateVector( stateA->deltaRotation, joint->frameA.p );
	b3Vec3 rB = b3RotateVector( stateB->deltaRotation, joint->frameB.p );

	b3Quat quatA = b3MulQuat( stateA->deltaRotation, joint->frameA.q );
	b3Quat quatB = b3MulQuat( stateB->deltaRotation, joint->frameB.q );
	if ( b3Raw( b3DotQuat( quatA, quatB ) ) < 0 )
	{
		quatB = b3NegateQuat( quatB );
	}

	b3Quat relQ = b3InvMulQuat( quatA, quatB );

	b3Vec3 axisAx = b3RotateVector( quatA, b3Vec3_axisX );
	b3Vec3 axisAy = b3RotateVector( quatA, b3Vec3_axisY );
	b3Vec3 axisAz = b3RotateVector( quatA, b3Vec3_axisZ );
	b3Vec3 spinAxis = b3RotateVector( quatB, b3Vec3_axisZ );

	// deltaPosition is Q24, so the difference is taken there and narrowed once
	// -- a difference of two nearly equal numbers, where rounding each side
	// first would round twice.
	b3Vec3 dp = b3W3ToVec3( b3SubW3( stateB->deltaPosition, stateA->deltaPosition ) );
	b3Vec3 d = b3Add( b3Add( dp, joint->deltaCenter ), b3Sub( rB, rA ) );

	b3Vec3 rAd = b3Add( d, rA );
	b3Vec3 sAx = b3Cross( rAd, axisAx );
	b3Vec3 sBx = b3Cross( rB, axisAx );
	b3Vec3 sAy = b3Cross( rAd, axisAy );
	b3Vec3 sBy = b3Cross( rB, axisAy );
	b3Vec3 sAz = b3Cross( rAd, axisAz );
	b3Vec3 sBz = b3Cross( rB, axisAz );

	b3f translation = b3Dot( axisAx, d );

	// The steering geometry, once. It gates three blocks -- the spring, both
	// steering limits, and the steering-on collinearity branch.
	b3WheelSteering steering = b3WheelSteeringFrame( quatA, quatB, axisAx );

	// The suspension's effective mass, rebuilt here rather than cached. Its
	// arms are sAx and sBx, formed above for the Jacobian, so this costs one
	// wide accumulation and one reciprocal. joint.h's @section nomass explains
	// why upstream's cached copy is not merely stale but wrong.
	b3f suspensionMass = b3RcpWide( b3LeverInertiaSumWide( mA, iA, sAx, mB, iB, sBx ) );

	// -----------------------------------------------------------------
	// Spin motor: the scalar drive about the wheel's own axis
	// -----------------------------------------------------------------
	if ( joint->enableSpinMotor && fixedRotation == false )
	{
		b3f spinMass = b3RcpWide( b3AxisInertiaSumWide( spinAxis, iA, iB ) );

		b3f cdot = b3SubF( b3Dot( b3Sub( wB, wA ), spinAxis ), joint->spinSpeed );

		b3imp oldImpulse = joint->spinImpulse;
		b3imp deltaImpulse = b3NegImp( b3MulFFToImp( spinMass, cdot ) );

		b3imp maxImpulse = b3MulFTToImp( joint->maxSpinTorque, context->h );
		joint->spinImpulse = b3ClampImp( b3AddImp( oldImpulse, deltaImpulse ), b3NegImp( maxImpulse ), maxImpulse );
		deltaImpulse = b3SubImp( joint->spinImpulse, oldImpulse );

		b3Imp3 L = b3MulImpV( deltaImpulse, spinAxis );
		wA = b3Sub( wA, b3MulMWImp( iA, L ) );
		wB = b3Add( wB, b3MulMWImp( iB, L ) );
	}

	// -----------------------------------------------------------------
	// Suspension spring: a real spring, so applied even during relax
	// -----------------------------------------------------------------
	//
	// No `useBias` guard, deliberately. Upstream's comment says it and the
	// prismatic's spring does the same: a spring's bias is its own restoring
	// force, not a position correction to be relaxed away.
	if ( joint->enableSuspensionSpring )
	{
		b3Softness ss = joint->suspensionSoftness;
		b3f bias = b3MulFF( b3MulFC( ss.biasRate, ss.massScale ), translation );

		b3f cdot = b3AddF( b3Dot( axisAx, b3Sub( vB, vA ) ), b3SubF( b3Dot( sBx, wB ), b3Dot( sAx, wA ) ) );
		b3f driving = b3AddF( b3MulFC( cdot, ss.massScale ), bias );

		b3imp deltaImpulse = b3SubImp( b3NegImp( b3MulFFToImp( suspensionMass, driving ) ),
									   b3MulImpC( joint->suspensionSpringImpulse, ss.impulseScale ) );
		joint->suspensionSpringImpulse = b3AddImp( joint->suspensionSpringImpulse, deltaImpulse );

		b3Imp3 P = b3MulImpV( deltaImpulse, axisAx );
		vA = b3Sub( vA, b3MulImpW3( P, mA ) );
		wA = b3Sub( wA, b3MulMWImp( iA, b3MulImpV( deltaImpulse, sAx ) ) );
		vB = b3Add( vB, b3MulImpW3( P, mB ) );
		wB = b3Add( wB, b3MulMWImp( iB, b3MulImpV( deltaImpulse, sBx ) ) );
	}

	// -----------------------------------------------------------------
	// Steering: a spring and two one-sided limits about the derived axis
	// -----------------------------------------------------------------
	//
	// `steering.valid` is the degeneracy guard of @section degenerate: below
	// the floor the axis is undefined and the whole block applies nothing,
	// rather than asking b3RcpWide for a mass it would saturate on.
	if ( joint->enableSteering && fixedRotation == false && steering.valid )
	{
		b3Vec3 axis = steering.axis;

		// Unit Jacobian, so this mass cannot blow up however far the wheel has
		// tipped -- which is the whole reason the axis was normalized.
		b3f steeringMass = b3RcpWide( b3AxisInertiaSumWide( axis, iA, iB ) );

		// The spring.
		{
			b3Softness cs = joint->steeringSoftness;

			// The angle difference, converted once. b3BradToRadF on the
			// *difference* rather than on each angle, as every other joint's
			// limit does -- then scaled by sqrt(den0), because the Jacobian was
			// normalized and the position bias is not scale-invariant.
			b3f c = b3MulFC( b3BradToRadF( (b3a)( steering.angle - joint->targetSteeringAngle ) ), steering.scale );
			b3f bias = b3MulFF( b3MulFC( cs.biasRate, cs.massScale ), c );

			b3f cdot = b3Dot( axis, b3Sub( wB, wA ) );
			b3f driving = b3AddF( b3MulFC( cdot, cs.massScale ), bias );

			b3imp oldImpulse = joint->steeringSpringImpulse;
			b3imp deltaImpulse = b3SubImp( b3NegImp( b3MulFFToImp( steeringMass, driving ) ),
										   b3MulImpC( oldImpulse, cs.impulseScale ) );

			b3imp maxImpulse = b3MulFTToImp( joint->maxSteeringTorque, context->h );
			joint->steeringSpringImpulse =
				b3ClampImp( b3AddImp( oldImpulse, deltaImpulse ), b3NegImp( maxImpulse ), maxImpulse );
			deltaImpulse = b3SubImp( joint->steeringSpringImpulse, oldImpulse );

			b3Imp3 L = b3MulImpV( deltaImpulse, axis );
			wA = b3Sub( wA, b3MulMWImp( iA, L ) );
			wB = b3Add( wB, b3MulMWImp( iB, L ) );
		}

		if ( joint->enableSteeringLimit )
		{
			// A quarter of the range, exactly as the prismatic's limits use --
			// an exact Q30 shift, and the reason the steering limits must not
			// default to a full turn.
			b3f speculative = b3MulFC(
				b3BradToRadF( (b3a)( joint->upperSteeringLimit - joint->lowerSteeringLimit ) ), B3_WHEEL_QUARTER );

			// Lower limit.
			{
				b3f c = b3MulFC( b3BradToRadF( (b3a)( steering.angle - joint->lowerSteeringLimit ) ), steering.scale );

				if ( b3Raw( c ) < b3Raw( speculative ) )
				{
					b3f bias = b3f_zero;
					b3c massScale = b3c_one;
					b3c impulseScale = b3c_zero;
					if ( b3Raw( c ) > 0 )
					{
						bias = b3MulFF( c, context->inv_h );
					}
					else if ( useBias )
					{
						b3Softness cs = base->constraintSoftness;
						bias = b3MulFF( b3MulFC( cs.biasRate, cs.massScale ), c );
						massScale = cs.massScale;
						impulseScale = cs.impulseScale;
					}

					b3f cdot = b3Dot( axis, b3Sub( wB, wA ) );
					b3f driving = b3AddF( b3MulFC( cdot, massScale ), bias );

					b3imp oldImpulse = joint->lowerSteeringImpulse;
					b3imp deltaImpulse = b3SubImp( b3NegImp( b3MulFFToImp( steeringMass, driving ) ),
												   b3MulImpC( oldImpulse, impulseScale ) );

					joint->lowerSteeringImpulse = b3MaxImp( b3AddImp( oldImpulse, deltaImpulse ), b3imp_zero );
					deltaImpulse = b3SubImp( joint->lowerSteeringImpulse, oldImpulse );

					b3Imp3 L = b3MulImpV( deltaImpulse, axis );
					wA = b3Sub( wA, b3MulMWImp( iA, L ) );
					wB = b3Add( wB, b3MulMWImp( iB, L ) );
				}
				else
				{
					joint->lowerSteeringImpulse = b3imp_zero;
				}
			}

			// Upper limit. The same constraint the other way round: the error,
			// the relative velocity and the applied impulse all flip sign, which
			// keeps `c` positive when satisfied and the impulse positive when
			// the limit is active.
			{
				b3f c = b3MulFC( b3BradToRadF( (b3a)( joint->upperSteeringLimit - steering.angle ) ), steering.scale );

				if ( b3Raw( c ) < b3Raw( speculative ) )
				{
					b3f bias = b3f_zero;
					b3c massScale = b3c_one;
					b3c impulseScale = b3c_zero;
					if ( b3Raw( c ) > 0 )
					{
						bias = b3MulFF( c, context->inv_h );
					}
					else if ( useBias )
					{
						b3Softness cs = base->constraintSoftness;
						bias = b3MulFF( b3MulFC( cs.biasRate, cs.massScale ), c );
						massScale = cs.massScale;
						impulseScale = cs.impulseScale;
					}

					b3f cdot = b3Dot( axis, b3Sub( wA, wB ) );
					b3f driving = b3AddF( b3MulFC( cdot, massScale ), bias );

					b3imp oldImpulse = joint->upperSteeringImpulse;
					b3imp deltaImpulse = b3SubImp( b3NegImp( b3MulFFToImp( steeringMass, driving ) ),
												   b3MulImpC( oldImpulse, impulseScale ) );

					joint->upperSteeringImpulse = b3MaxImp( b3AddImp( oldImpulse, deltaImpulse ), b3imp_zero );
					deltaImpulse = b3SubImp( joint->upperSteeringImpulse, oldImpulse );

					b3Imp3 L = b3MulImpV( deltaImpulse, axis );
					wA = b3Add( wA, b3MulMWImp( iA, L ) );
					wB = b3Sub( wB, b3MulMWImp( iB, L ) );
				}
				else
				{
					joint->upperSteeringImpulse = b3imp_zero;
				}
			}
		}
	}

	// -----------------------------------------------------------------
	// Suspension limits: the travel stops
	// -----------------------------------------------------------------
	if ( joint->enableSuspensionLimit )
	{
		b3f speculative =
			b3MulFC( b3SubF( joint->upperSuspensionLimit, joint->lowerSuspensionLimit ), B3_WHEEL_QUARTER );

		// Lower limit.
		{
			b3f c = b3SubF( translation, joint->lowerSuspensionLimit );

			if ( b3Raw( c ) < b3Raw( speculative ) )
			{
				b3f bias = b3f_zero;
				b3c massScale = b3c_one;
				b3c impulseScale = b3c_zero;
				if ( b3Raw( c ) > 0 )
				{
					bias = b3MulFF( c, context->inv_h );
				}
				else if ( useBias )
				{
					b3Softness cs = base->constraintSoftness;
					bias = b3MulFF( b3MulFC( cs.biasRate, cs.massScale ), c );
					massScale = cs.massScale;
					impulseScale = cs.impulseScale;
				}

				b3f cdot = b3AddF( b3Dot( axisAx, b3Sub( vB, vA ) ), b3SubF( b3Dot( sBx, wB ), b3Dot( sAx, wA ) ) );
				b3f driving = b3AddF( b3MulFC( cdot, massScale ), bias );

				b3imp oldImpulse = joint->lowerSuspensionImpulse;
				b3imp deltaImpulse = b3SubImp( b3NegImp( b3MulFFToImp( suspensionMass, driving ) ),
											   b3MulImpC( oldImpulse, impulseScale ) );

				joint->lowerSuspensionImpulse = b3MaxImp( b3AddImp( oldImpulse, deltaImpulse ), b3imp_zero );
				deltaImpulse = b3SubImp( joint->lowerSuspensionImpulse, oldImpulse );

				b3Imp3 P = b3MulImpV( deltaImpulse, axisAx );
				vA = b3Sub( vA, b3MulImpW3( P, mA ) );
				wA = b3Sub( wA, b3MulMWImp( iA, b3MulImpV( deltaImpulse, sAx ) ) );
				vB = b3Add( vB, b3MulImpW3( P, mB ) );
				wB = b3Add( wB, b3MulMWImp( iB, b3MulImpV( deltaImpulse, sBx ) ) );
			}
			else
			{
				joint->lowerSuspensionImpulse = b3imp_zero;
			}
		}

		// Upper limit, signs flipped throughout.
		{
			b3f c = b3SubF( joint->upperSuspensionLimit, translation );

			if ( b3Raw( c ) < b3Raw( speculative ) )
			{
				b3f bias = b3f_zero;
				b3c massScale = b3c_one;
				b3c impulseScale = b3c_zero;
				if ( b3Raw( c ) > 0 )
				{
					bias = b3MulFF( c, context->inv_h );
				}
				else if ( useBias )
				{
					b3Softness cs = base->constraintSoftness;
					bias = b3MulFF( b3MulFC( cs.biasRate, cs.massScale ), c );
					massScale = cs.massScale;
					impulseScale = cs.impulseScale;
				}

				b3f cdot = b3AddF( b3Dot( axisAx, b3Sub( vA, vB ) ), b3SubF( b3Dot( sAx, wA ), b3Dot( sBx, wB ) ) );
				b3f driving = b3AddF( b3MulFC( cdot, massScale ), bias );

				b3imp oldImpulse = joint->upperSuspensionImpulse;
				b3imp deltaImpulse = b3SubImp( b3NegImp( b3MulFFToImp( suspensionMass, driving ) ),
											   b3MulImpC( oldImpulse, impulseScale ) );

				joint->upperSuspensionImpulse = b3MaxImp( b3AddImp( oldImpulse, deltaImpulse ), b3imp_zero );
				deltaImpulse = b3SubImp( joint->upperSuspensionImpulse, oldImpulse );

				b3Imp3 P = b3MulImpV( deltaImpulse, axisAx );
				vA = b3Add( vA, b3MulImpW3( P, mA ) );
				wA = b3Add( wA, b3MulMWImp( iA, b3MulImpV( deltaImpulse, sAx ) ) );
				vB = b3Sub( vB, b3MulImpW3( P, mB ) );
				wB = b3Sub( wB, b3MulMWImp( iB, b3MulImpV( deltaImpulse, sBx ) ) );
			}
			else
			{
				joint->upperSuspensionImpulse = b3imp_zero;
			}
		}
	}

	// -----------------------------------------------------------------
	// Collinearity: the spin axis stays square to the suspension
	// -----------------------------------------------------------------
	if ( fixedRotation == false )
	{
		if ( joint->enableSteering )
		{
			// One degree of freedom, because steering owns the other. The axis
			// is upstream's unnormalized `cross( B.cz, A.cx )` -- see
			// @section degenerate for why this one is *not* normalized while
			// the steering axis is -- and `steering.valid` is the same guard.
			if ( steering.valid )
			{
				b3Vec3 u = steering.collinearityAxis;

				b3f bias = b3f_zero;
				b3c massScale = b3c_one;
				b3c impulseScale = b3c_zero;
				if ( useBias )
				{
					b3Softness cs = base->constraintSoftness;
					b3f c = b3Dot( axisAx, spinAxis );
					bias = b3MulFF( b3MulFC( cs.biasRate, cs.massScale ), c );
					massScale = cs.massScale;
					impulseScale = cs.impulseScale;
				}

				b3f perpMass = b3RcpWide( b3AxisInertiaSumWide( u, iA, iB ) );

				b3f cdot = b3Dot( b3Sub( wB, wA ), u );
				b3f driving = b3AddF( b3MulFC( cdot, massScale ), bias );

				b3imp oldImpulse = joint->angularImpulse.x;
				b3imp deltaImpulse = b3SubImp( b3NegImp( b3MulFFToImp( perpMass, driving ) ),
											   b3MulImpC( oldImpulse, impulseScale ) );
				joint->angularImpulse.x = b3AddImp( oldImpulse, deltaImpulse );

				b3Imp3 L = b3MulImpV( deltaImpulse, u );
				wA = b3Sub( wA, b3MulMWImp( iA, L ) );
				wB = b3Add( wB, b3MulMWImp( iB, L ) );
			}
		}
		else
		{
			// Two degrees, and this is the parallel joint's block exactly --
			// b3CollinearityPerpAxes into b3InvertPerpMass, with the saturating
			// multiply because these rows vanish at a half turn. See
			// @section twotwo.
			b3f biasX = b3f_zero;
			b3f biasY = b3f_zero;
			b3c massScale = b3c_one;
			b3c impulseScale = b3c_zero;

			if ( useBias )
			{
				b3Softness cs = base->constraintSoftness;
				b3f rate = b3MulFC( cs.biasRate, cs.massScale );
				biasX = b3MulFF( rate, b3NToF( relQ.v.x ) );
				biasY = b3MulFF( rate, b3NToF( relQ.v.y ) );
				massScale = cs.massScale;
				impulseScale = cs.impulseScale;
			}

			b3Vec3 perpAxisX, perpAxisY;
			b3CollinearityPerpAxes( quatA, relQ, &perpAxisX, &perpAxisY );

			b3SymMatrix2 invK = b3InvertPerpMass( perpAxisX, perpAxisY, iA, iB );

			b3Vec3 wRel = b3Sub( wB, wA );
			b3f cdotX = b3AddF( b3MulFC( b3Dot( wRel, perpAxisX ), massScale ), biasX );
			b3f cdotY = b3AddF( b3MulFC( b3Dot( wRel, perpAxisY ), massScale ), biasY );

			b3Imp2 sol = b3MulSym2VSat( invK, cdotX, cdotY );

			b3Imp2 oldImpulse = joint->angularImpulse;

			b3Imp2 deltaImpulse;
			deltaImpulse.x = b3SubImp( b3NegImp( sol.x ), b3MulImpC( oldImpulse.x, impulseScale ) );
			deltaImpulse.y = b3SubImp( b3NegImp( sol.y ), b3MulImpC( oldImpulse.y, impulseScale ) );

			joint->angularImpulse = b3AddImp2( oldImpulse, deltaImpulse );

			b3Imp3 L =
				b3AddImp3( b3MulImpV( deltaImpulse.x, perpAxisX ), b3MulImpV( deltaImpulse.y, perpAxisY ) );
			wA = b3Sub( wA, b3MulMWImp( iA, L ) );
			wB = b3Add( wB, b3MulMWImp( iB, L ) );
		}
	}

	// -----------------------------------------------------------------
	// Point to line: the 2x2 that holds the anchor on the travel axis
	// -----------------------------------------------------------------
	//
	// The prismatic's block verbatim, including the plain (non-saturating)
	// b3MulSym2V: these arms are bounded by the geometry rather than vanishing.
	{
		b3f biasY = b3f_zero;
		b3f biasZ = b3f_zero;
		b3c massScale = b3c_one;
		b3c impulseScale = b3c_zero;
		if ( useBias )
		{
			b3Softness cs = base->constraintSoftness;
			b3f rate = b3MulFC( cs.biasRate, cs.massScale );
			biasY = b3MulFF( rate, b3Dot( axisAy, d ) );
			biasZ = b3MulFF( rate, b3Dot( axisAz, d ) );
			massScale = cs.massScale;
			impulseScale = cs.impulseScale;
		}

		b3SymMatrix2 invK = b3InvertPointLineMass( mA, iA, sAy, sAz, mB, iB, sBy, sBz );

		b3Vec3 vRel = b3Sub( b3Sub( b3Add( vB, b3Cross( wB, rB ) ), vA ), b3Cross( wA, rAd ) );
		b3f cdotY = b3AddF( b3MulFC( b3Dot( vRel, axisAy ), massScale ), biasY );
		b3f cdotZ = b3AddF( b3MulFC( b3Dot( vRel, axisAz ), massScale ), biasZ );

		b3Imp2 sol = b3MulSym2V( invK, cdotY, cdotZ );

		b3Imp2 oldImpulse = joint->linearImpulse;

		b3Imp2 deltaImpulse;
		deltaImpulse.x = b3SubImp( b3NegImp( sol.x ), b3MulImpC( oldImpulse.x, impulseScale ) );
		deltaImpulse.y = b3SubImp( b3NegImp( sol.y ), b3MulImpC( oldImpulse.y, impulseScale ) );

		joint->linearImpulse = b3AddImp2( oldImpulse, deltaImpulse );

		b3Imp3 P = b3AddImp3( b3MulImpV( deltaImpulse.x, axisAy ), b3MulImpV( deltaImpulse.y, axisAz ) );
		b3Imp3 LA = b3AddImp3( b3MulImpV( deltaImpulse.x, sAy ), b3MulImpV( deltaImpulse.y, sAz ) );
		b3Imp3 LB = b3AddImp3( b3MulImpV( deltaImpulse.x, sBy ), b3MulImpV( deltaImpulse.y, sBz ) );

		vA = b3Sub( vA, b3MulImpW3( P, mA ) );
		wA = b3Sub( wA, b3MulMWImp( iA, LA ) );
		vB = b3Add( vB, b3MulImpW3( P, mB ) );
		wB = b3Add( wB, b3MulMWImp( iB, LB ) );
	}

	if ( stateA->flags & b3_dynamicFlag )
	{
		stateA->linearVelocity = vA;
		stateA->angularVelocity = wA;
	}

	if ( stateB->flags & b3_dynamicFlag )
	{
		stateB->linearVelocity = vB;
		stateB->angularVelocity = wB;
	}
}
