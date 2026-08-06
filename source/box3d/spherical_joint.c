// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Erin Catto        (original Box3D)
// Copyright (c) 2026 Warioware64       (Nitro Engine Advanced fixed-point port)
//
// This file is part of Nitro Engine Advanced

/// @file   spherical_joint.c
/// @brief  The spherical joint: a ball joint, and the first 3-vector spring.
///
/// @section shape What a ball joint actually is
///
/// The revolute's point-to-point constraint, unchanged, plus four rotational
/// constraints that *bound* rather than lock:
///
///   1. the **3x3 point-to-point** constraint on the two anchors, lifted from
///      revolute_joint.c and identical to it;
///   2. a **3-vector spring** pulling frame B's orientation toward
///      `targetRotation`, through the rotational effective mass;
///   3. a **3-vector motor** driving the relative angular velocity, bounded by
///      the magnitude of its torque;
///   4. a **cone limit**, one-sided, on how far frame B's z axis has tilted
///      away from frame A's;
///   5. a **twist limit**, two one-sided constraints, on the rotation about
///      that axis.
///
/// Constraints 2 and 3 are what make this stage different from Stage 3. Every
/// spring and motor before it was a scalar on one degree of freedom; these act
/// on all three at once, which is why they need b3InvertRotationMass and why
/// the motor's bound is a sphere rather than an interval.
///
/// @section jacobian The twist Jacobian, and the departure from upstream
///
/// This is the one place the port does not transliterate, and the reason is
/// that upstream's form is unbounded.
///
/// Upstream builds the twist Jacobian as `coneAxis + tan(theta/2) * perpAxis`,
/// where theta is the swing angle ([spherical_joint.c:350-357](helpSrc/box3d-main/src/spherical_joint.c#L350-L357),
/// carrying a `// todo verify this Jacobian` beside it). That tangent diverges
/// as the swing approaches half a turn: at 179 degrees the Jacobian's magnitude
/// is 114, and the effective mass it feeds -- `J . invI . J` -- grows as its
/// *square*. Q7.24 tops out at 128, so the mass overflows long before the
/// Jacobian does, and no cap on the tangent fixes that without also being a
/// cap on which scenes are allowed.
///
/// So the port never forms the tangent. Writing `J = s * u` with `u` a unit
/// vector and `s = |J|`:
///
///     u = cos(theta/2) * coneAxis + sin(theta/2) * perpAxis
///     s = 1 / cos(theta/2)
///
/// -- the same direction, by construction, and both factors bounded, with `u`
/// costing one sine and one cosine of an angle the cone limit already computed.
///
/// A velocity constraint is invariant under scaling its Jacobian: scaling `J`
/// by `1/s` scales Cdot by `1/s` and the effective mass by `s^2`, and the
/// impulse *vector* that comes out is unchanged. The position bias is not
/// scaled by that argument, because it is an angle rather than a velocity --
/// so it is scaled explicitly, by `cos(theta/2)`, which is the reciprocal of
/// the `s` that was divided out. Both branches of each limit are linear in the
/// error, so scaling the error once before them covers both.
///
/// The result is exactly upstream's constraint with nothing left unbounded, and
/// the accumulator now holds the impulse along `u` rather than along `J`.
///
/// @section scales What is new here, and what is not
///
/// Stage 2's conventions and Stage 3's are unchanged and not re-derived: the
/// effective mass accumulated at Q24 and reciprocated once, `massScale` folded
/// into each term at Q30, the clamp landing on the accumulator with the delta
/// recomputed from it, angles converted on the *difference* against a limit.
///
/// @section absent What is not here
///
/// b3DrawSphericalJoint and every B3_REC hook, as in every other joint file.

#include "joint.h"

#include "body.h"
#include "core.h"
#include "physics_world.h"
#include "solver.h"
#include "solver_set.h"

#include "box3d/box3d.h"

// =========================================================================
// Accessors
// =========================================================================

void b3SphericalJoint_EnableSpring( b3JointId jointId, bool enableSpring )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_sphericalJoint );
	if ( enableSpring != base->sphericalJoint.enableSpring )
	{
		base->sphericalJoint.enableSpring = enableSpring;
		base->sphericalJoint.springImpulse = b3Imp3_zero;
	}
}

bool b3SphericalJoint_IsSpringEnabled( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_sphericalJoint );
	return base->sphericalJoint.enableSpring;
}

void b3SphericalJoint_SetSpringHertz( b3JointId jointId, b3f hertz )
{
	B3_ASSERT( b3IsValidFloat( hertz ) && b3Raw( hertz ) >= 0 );

	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_sphericalJoint );
	base->sphericalJoint.hertz = hertz;
}

b3f b3SphericalJoint_GetSpringHertz( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_sphericalJoint );
	return base->sphericalJoint.hertz;
}

void b3SphericalJoint_SetSpringDampingRatio( b3JointId jointId, b3f dampingRatio )
{
	B3_ASSERT( b3IsValidFloat( dampingRatio ) && b3Raw( dampingRatio ) >= 0 );

	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_sphericalJoint );
	base->sphericalJoint.dampingRatio = dampingRatio;
}

b3f b3SphericalJoint_GetSpringDampingRatio( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_sphericalJoint );
	return base->sphericalJoint.dampingRatio;
}

void b3SphericalJoint_SetTargetRotation( b3JointId jointId, b3Quat targetRotation )
{
	B3_ASSERT( b3IsValidQuat( targetRotation ) );

	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_sphericalJoint );
	base->sphericalJoint.targetRotation = targetRotation;
}

b3Quat b3SphericalJoint_GetTargetRotation( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_sphericalJoint );
	return base->sphericalJoint.targetRotation;
}

void b3SphericalJoint_EnableConeLimit( b3JointId jointId, bool enableLimit )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_sphericalJoint );
	if ( enableLimit != base->sphericalJoint.enableConeLimit )
	{
		base->sphericalJoint.enableConeLimit = enableLimit;
		base->sphericalJoint.swingImpulse = b3imp_zero;
	}
}

bool b3SphericalJoint_IsConeLimitEnabled( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_sphericalJoint );
	return base->sphericalJoint.enableConeLimit;
}

void b3SphericalJoint_SetConeLimit( b3JointId jointId, b3a angle )
{
	B3_ASSERT( angle >= 0 && angle <= B3_BRAD_PI );

	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_sphericalJoint );
	base->sphericalJoint.coneAngle = angle;
	base->sphericalJoint.swingImpulse = b3imp_zero;
}

b3a b3SphericalJoint_GetConeLimit( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_sphericalJoint );
	return base->sphericalJoint.coneAngle;
}

void b3SphericalJoint_EnableTwistLimit( b3JointId jointId, bool enableLimit )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_sphericalJoint );
	if ( enableLimit != base->sphericalJoint.enableTwistLimit )
	{
		base->sphericalJoint.enableTwistLimit = enableLimit;
		base->sphericalJoint.lowerTwistImpulse = b3imp_zero;
		base->sphericalJoint.upperTwistImpulse = b3imp_zero;
	}
}

bool b3SphericalJoint_IsTwistLimitEnabled( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_sphericalJoint );
	return base->sphericalJoint.enableTwistLimit;
}

void b3SphericalJoint_SetTwistLimits( b3JointId jointId, b3a lower, b3a upper )
{
	B3_ASSERT( lower <= upper );

	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_sphericalJoint );
	base->sphericalJoint.lowerTwistAngle = lower;
	base->sphericalJoint.upperTwistAngle = upper;
	base->sphericalJoint.lowerTwistImpulse = b3imp_zero;
	base->sphericalJoint.upperTwistImpulse = b3imp_zero;
}

b3a b3SphericalJoint_GetLowerTwistLimit( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_sphericalJoint );
	return base->sphericalJoint.lowerTwistAngle;
}

b3a b3SphericalJoint_GetUpperTwistLimit( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_sphericalJoint );
	return base->sphericalJoint.upperTwistAngle;
}

void b3SphericalJoint_EnableMotor( b3JointId jointId, bool enableMotor )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_sphericalJoint );
	if ( enableMotor != base->sphericalJoint.enableMotor )
	{
		base->sphericalJoint.enableMotor = enableMotor;
		base->sphericalJoint.motorImpulse = b3Imp3_zero;
	}
}

bool b3SphericalJoint_IsMotorEnabled( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_sphericalJoint );
	return base->sphericalJoint.enableMotor;
}

void b3SphericalJoint_SetMotorVelocity( b3JointId jointId, b3Vec3 motorVelocity )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_sphericalJoint );
	base->sphericalJoint.motorVelocity = motorVelocity;
}

b3Vec3 b3SphericalJoint_GetMotorVelocity( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_sphericalJoint );
	return base->sphericalJoint.motorVelocity;
}

void b3SphericalJoint_SetMaxMotorTorque( b3JointId jointId, b3f torque )
{
	B3_ASSERT( b3IsValidFloat( torque ) && b3Raw( torque ) >= 0 );

	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_sphericalJoint );
	base->sphericalJoint.maxMotorTorque = torque;
}

b3f b3SphericalJoint_GetMaxMotorTorque( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_sphericalJoint );
	return base->sphericalJoint.maxMotorTorque;
}

b3Vec3 b3SphericalJoint_GetMotorTorque( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_sphericalJoint );

	b3Imp3 p = base->sphericalJoint.motorImpulse;
	b3f inv_h = world->inv_h;
	return b3MakeVec3( b3MulImpFToF( p.x, inv_h ), b3MulImpFToF( p.y, inv_h ), b3MulImpFToF( p.z, inv_h ) );
}

/// The relative rotation of frame B against frame A, as the joint sees it.
///
/// Both angle readouts below need it, and so does the solve. Kept in one place
/// because getting the operand order wrong here inverts every angle the joint
/// reports, silently.
static b3Quat b3SphericalRelativeRotation( b3World* world, b3JointSim* base )
{
	b3WorldTransform transformA = b3GetBodyTransform( world, base->bodyIdA );
	b3WorldTransform transformB = b3GetBodyTransform( world, base->bodyIdB );

	b3Quat qA = b3MulQuat( transformA.q, base->localFrameA.q );
	b3Quat qB = b3MulQuat( transformB.q, base->localFrameB.q );

	return b3InvMulQuat( qA, qB );
}

b3a b3SphericalJoint_GetConeAngle( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	if ( world == NULL )
	{
		return 0;
	}

	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_sphericalJoint );
	return b3GetSwingAngle( b3SphericalRelativeRotation( world, base ) );
}

b3a b3SphericalJoint_GetTwistAngle( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	if ( world == NULL )
	{
		return 0;
	}

	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_sphericalJoint );
	return b3GetTwistAngle( b3SphericalRelativeRotation( world, base ) );
}

/// The force the point-to-point constraint is applying, in world space.
b3Vec3 b3GetSphericalJointForce( b3World* world, b3JointSim* base )
{
	b3Imp3 p = base->sphericalJoint.linearImpulse;
	b3f inv_h = world->inv_h;
	return b3MakeVec3( b3MulImpFToF( p.x, inv_h ), b3MulImpFToF( p.y, inv_h ), b3MulImpFToF( p.z, inv_h ) );
}

/// The torque the joint is applying, in world space.
///
/// Four contributions: the spring and motor, which are already 3-vectors; the
/// twist accumulators along the cached twist axis; and the cone accumulator
/// along the swing axis.
///
/// Two departures from upstream's version, both about signs and both settled by
/// the closed-form reaction test rather than by reading:
///
///   - upstream recomputes `swingAxis` and `twistAxis` from the two quaternions
///     here while the solve and warm start use the *cached* axes. The port uses
///     the cached ones throughout, so the torque reported is the torque
///     actually applied rather than a second estimate of it.
///   - upstream adds the swing term with a **+**
///     ([spherical_joint.c:276](helpSrc/box3d-main/src/spherical_joint.c#L276))
///     where both its own warm start and its own solve apply that impulse to
///     body B with a **-**. The port matches the solve. Reporting a torque the
///     solver did not apply is the same class of bug Stage 3 found in
///     b3GetRevoluteJointTorque, in the same debug-only readout.
b3Vec3 b3GetSphericalJointTorque( b3World* world, b3JointSim* base )
{
	b3SphericalJoint* joint = &base->sphericalJoint;

	b3Imp3 angular = b3AddImp3( joint->springImpulse, joint->motorImpulse );

	b3imp twistImpulse = b3SubImp( joint->lowerTwistImpulse, joint->upperTwistImpulse );
	angular = b3AddImp3( angular, b3MulImpV( twistImpulse, joint->twistAxis ) );
	angular = b3SubImp3( angular, b3MulImpV( joint->swingImpulse, joint->swingAxis ) );

	b3f inv_h = world->inv_h;
	return b3MakeVec3( b3MulImpFToF( angular.x, inv_h ), b3MulImpFToF( angular.y, inv_h ),
					   b3MulImpFToF( angular.z, inv_h ) );
}

// =========================================================================
// Solver
// =========================================================================

/// The cone axis, the swing axis and the unit twist axis, from the two frames.
///
/// `coneAxis` is frame A's z and `twistDir` is frame B's z; the swing axis is
/// perpendicular to both, and is what the cone limit pushes along. The twist
/// axis is the unit form of upstream's Jacobian -- see the file header for why
/// it is built from a half-angle sine and cosine rather than from a tangent.
///
/// `*outTwistScale` is `cos(theta/2)`, the factor the twist limit's position
/// error is scaled by to undo the Jacobian's normalization.
///
/// @section degenerate The parallel case, which is the common one
///
/// When the two z axes are nearly parallel -- which is a limb at rest, not a
/// corner -- their cross product is nearly zero and has no direction to
/// normalize. b3Normalize only guards an *exactly* zero input; below about
/// 1/128 of a unit b3RsqrtWide saturates instead, so what comes back is a
/// short vector rather than a unit one.
///
/// That is not harmless, and assuming it was cost this stage its device
/// example. The cone limit divides by `swingAxis . invI . swingAxis`, so a
/// swing axis a fortieth of unit length inflates the effective mass by
/// sixteen hundred: the measured value was 11,915 against a true one near 7,
/// and the resulting impulse tore a ragdoll apart on the first frame it was
/// upright.
///
/// So a degenerate axis is reported as *zero* here and the caller gives it no
/// effective mass, which is the port's convention everywhere else and is also
/// the physically right answer -- a cone limit has nothing to resist when
/// there is no swing to resist.
static void b3SphericalAxes( b3Quat frameQA, b3Quat frameQB, b3a swingAngle, b3Vec3* outSwingAxis, b3Vec3* outTwistAxis,
							 b3c* outTwistScale )
{
	b3Vec3 coneAxis = b3RotateVector( frameQA, b3Vec3_axisZ );
	b3Vec3 twistDir = b3RotateVector( frameQB, b3Vec3_axisZ );

	b3Vec3 swingAxis = b3Normalize( b3Cross( coneAxis, twistDir ) );
	if ( b3IsNormalized( swingAxis ) == false )
	{
		swingAxis = b3Vec3_zero;
	}

	b3Vec3 perpAxis = b3Cross( swingAxis, coneAxis );

	// Half the swing, in brads. The division is exact -- brads are integers and
	// this is a shift -- which is the other reason to work in them here.
	b3a half = (b3a)( swingAngle / 2 );
	b3c cosHalf = b3CosA( half );
	b3c sinHalf = b3SinA( half );

	*outSwingAxis = swingAxis;
	*outTwistAxis = b3Add( b3MulCV( cosHalf, coneAxis ), b3MulCV( sinHalf, perpAxis ) );
	*outTwistScale = cosHalf;
}

void b3PrepareSphericalJoint( b3JointSim* base, b3StepContext* context )
{
	B3_ASSERT( base->type == b3_sphericalJoint );

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

	b3SphericalJoint* joint = &base->sphericalJoint;

	joint->indexA = bodyA->setIndex == b3_awakeSet ? localIndexA : B3_NULL_INDEX;
	joint->indexB = bodyB->setIndex == b3_awakeSet ? localIndexB : B3_NULL_INDEX;

	// Joint frames in world space, with the position relative to the centre of
	// mass. One rotation of (localFrame.p - localCenter) rather than a transform
	// followed by a subtraction, which is upstream's round-off note: the two
	// centres can be far from the origin and the difference of two large numbers
	// is where the bits go.
	joint->frameA.q = b3MulQuat( bodySimA->transform.q, base->localFrameA.q );
	joint->frameA.p = b3RotateVector( bodySimA->transform.q, b3Sub( base->localFrameA.p, bodySimA->localCenter ) );
	joint->frameB.q = b3MulQuat( bodySimB->transform.q, base->localFrameB.q );
	joint->frameB.p = b3RotateVector( bodySimB->transform.q, b3Sub( base->localFrameB.p, bodySimB->localCenter ) );

	joint->deltaCenter = b3Sub( bodySimB->center, bodySimA->center );

	// The rotational effective mass, and the singularity guard that comes free
	// with it.
	joint->rotationMass = b3InvertRotationMass( base->invIA, base->invIB );
	base->fixedRotation = b3FixedRotationFromMass( joint->rotationMass );

	b3Quat relQ = b3InvMulQuat( joint->frameA.q, joint->frameB.q );
	b3a swingAngle = b3GetSwingAngle( relQ );

	b3Vec3 swingAxis, twistAxis;
	b3c twistScale;
	b3SphericalAxes( joint->frameA.q, joint->frameB.q, swingAngle, &swingAxis, &twistAxis, &twistScale );

	joint->swingAxis = swingAxis;
	joint->twistAxis = twistAxis;
	joint->twistScale = twistScale;

	// Both scalar masses follow the contact solver's shape, which is also the
	// revolute's: a Q24 inverse inertia accumulated through b3DotVWide and
	// reciprocated once by b3RcpW into a Q12 mass. Both axes are unit length --
	// the twist one by construction -- so neither product can leave Q24 the way
	// upstream's tangent-scaled Jacobian would.
	//
	// Except when an axis has degenerated, which b3SphericalAxes reports by
	// handing back a zero vector. A mass is a *reciprocal*, so a short axis
	// does not merely weaken the constraint, it inflates it -- and the guard
	// has to be on the axis rather than on the mass, because by the time the
	// reciprocal has been taken the damage is indistinguishable from a
	// legitimately light body. Zero mass means no impulse, which is what every
	// other singular effective mass in the port answers.
	joint->swingMass =
		b3IsNormalized( swingAxis ) ? b3RcpWide( b3AxisInertiaSumWide( swingAxis, base->invIA, base->invIB ) )
									: b3f_zero;
	joint->twistMass =
		b3IsNormalized( twistAxis ) ? b3RcpWide( b3AxisInertiaSumWide( twistAxis, base->invIA, base->invIB ) )
									: b3f_zero;

	joint->springSoftness = b3MakeSoft( joint->hertz, joint->dampingRatio, context->h );

	if ( context->enableWarmStarting == false )
	{
		joint->linearImpulse = b3Imp3_zero;
		joint->springImpulse = b3Imp3_zero;
		joint->motorImpulse = b3Imp3_zero;
		joint->swingImpulse = b3imp_zero;
		joint->lowerTwistImpulse = b3imp_zero;
		joint->upperTwistImpulse = b3imp_zero;
	}
}

/// @note ITCM group B3_ITCM_SPHERICAL -- see nea_config.h.
void B3_ITCM_IF( B3_ITCM_SPHERICAL_WARM, b3WarmStartSphericalJoint )( b3JointSim* base, b3StepContext* context )
{
	B3_ASSERT( base->type == b3_sphericalJoint );

	b3iw mA = base->invMassA;
	b3iw mB = base->invMassB;
	b3MatrixW iA = base->invIA;
	b3MatrixW iB = base->invIB;

	b3BodyState dummyState = b3_identityBodyState;

	b3SphericalJoint* joint = &base->sphericalJoint;
	b3BodyState* stateA = joint->indexA == B3_NULL_INDEX ? &dummyState : context->states + joint->indexA;
	b3BodyState* stateB = joint->indexB == B3_NULL_INDEX ? &dummyState : context->states + joint->indexB;

	b3Vec3 rA = b3RotateVector( stateA->deltaRotation, joint->frameA.p );
	b3Vec3 rB = b3RotateVector( stateB->deltaRotation, joint->frameB.p );

	// The four rotational accumulators, resolved onto the axes they were
	// accumulated along. The signs match the solve exactly -- see
	// b3GetSphericalJointTorque on why that is worth stating.
	b3Imp3 angular = b3AddImp3( joint->springImpulse, joint->motorImpulse );

	b3imp twistImpulse = b3SubImp( joint->lowerTwistImpulse, joint->upperTwistImpulse );
	angular = b3AddImp3( angular, b3MulImpV( twistImpulse, joint->twistAxis ) );
	angular = b3SubImp3( angular, b3MulImpV( joint->swingImpulse, joint->swingAxis ) );

	b3Imp3 P = joint->linearImpulse;

	b3Vec3 vA = b3Sub( stateA->linearVelocity, b3MulImpW3( P, mA ) );
	b3Vec3 wA = b3Sub( stateA->angularVelocity, b3MulMWImp( iA, b3AddImp3( b3CrossVImp( rA, P ), angular ) ) );

	b3Vec3 vB = b3Add( stateB->linearVelocity, b3MulImpW3( P, mB ) );
	b3Vec3 wB = b3Add( stateB->angularVelocity, b3MulMWImp( iB, b3AddImp3( b3CrossVImp( rB, P ), angular ) ) );

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

/// @note ITCM group B3_ITCM_SPHERICAL -- see nea_config.h.
void B3_ITCM_IF( B3_ITCM_SPHERICAL_SOLVE, b3SolveSphericalJoint )( b3JointSim* base, b3StepContext* context, bool useBias )
{
	B3_ASSERT( base->type == b3_sphericalJoint );

	b3iw mA = base->invMassA;
	b3iw mB = base->invMassB;
	b3MatrixW iA = base->invIA;
	b3MatrixW iB = base->invIB;

	b3BodyState dummyState = b3_identityBodyState;

	b3SphericalJoint* joint = &base->sphericalJoint;
	b3BodyState* stateA = joint->indexA == B3_NULL_INDEX ? &dummyState : context->states + joint->indexA;
	b3BodyState* stateB = joint->indexB == B3_NULL_INDEX ? &dummyState : context->states + joint->indexB;

	b3Vec3 vA = stateA->linearVelocity;
	b3Vec3 wA = stateA->angularVelocity;
	b3Vec3 vB = stateB->linearVelocity;
	b3Vec3 wB = stateB->angularVelocity;

	bool fixedRotation = base->fixedRotation;
	b3t h = context->h;

	b3Quat quatA = b3MulQuat( stateA->deltaRotation, joint->frameA.q );
	b3Quat quatB = b3MulQuat( stateB->deltaRotation, joint->frameB.q );

	// A quaternion and its negation are the same rotation, and picking the
	// nearer of the two keeps the twist angle in [-pi, pi] instead of jumping a
	// full turn when the joint crosses the half-way point.
	if ( b3Raw( b3DotQuat( quatA, quatB ) ) < 0 )
	{
		quatB = b3NegateQuat( quatB );
	}

	b3Quat relQ = b3InvMulQuat( quatA, quatB );

	// The cached axes, as upstream uses them: the accumulators below were
	// warm-started along these, so recomputing them mid-solve would apply a
	// stored impulse along a direction it was never accumulated in. Upstream
	// carries a `todo does an updated axis help?` beside both.
	b3Vec3 swingAxis = joint->swingAxis;
	b3Vec3 twistAxis = joint->twistAxis;
	b3c twistScale = joint->twistScale;

	// -----------------------------------------------------------------
	// Rotational spring, toward targetRotation
	// -----------------------------------------------------------------
	if ( joint->enableSpring && fixedRotation == false )
	{
		// The orientation error as a rotation vector in radians, rotated into
		// world space. Negated because C is measured from B to A while the
		// impulse is applied to B.
		b3Vec3 deltaRotation = b3DeltaQuatToRotation( relQ, joint->targetRotation );
		b3Vec3 c = b3Neg( b3RotateVector( quatA, deltaRotation ) );

		b3Softness soft = joint->springSoftness;
		b3Vec3 bias = b3MulSV( b3MulFC( soft.biasRate, soft.massScale ), c );
		b3Vec3 cdot = b3Sub( wB, wA );
		b3Vec3 driving = b3Add( b3MulSV( b3CToF( soft.massScale ), cdot ), bias );

		b3Imp3 sol = b3MulMVToImp( joint->rotationMass, driving );
		b3Imp3 impulse = b3SubImp3( b3NegImp3( sol ), b3MulCImp3( soft.impulseScale, joint->springImpulse ) );
		joint->springImpulse = b3AddImp3( joint->springImpulse, impulse );

		wA = b3Sub( wA, b3MulMWImp( iA, impulse ) );
		wB = b3Add( wB, b3MulMWImp( iB, impulse ) );
	}

	// -----------------------------------------------------------------
	// Rotational motor
	// -----------------------------------------------------------------
	if ( joint->enableMotor && fixedRotation == false )
	{
		b3Vec3 cdot = b3Sub( b3Sub( wB, wA ), joint->motorVelocity );

		b3Imp3 oldImpulse = joint->motorImpulse;
		b3Imp3 lambda = b3NegImp3( b3MulMVToImp( joint->rotationMass, cdot ) );

		// A torque bound becomes an impulse bound over one sub-step, as in the
		// revolute -- but the bound is on the *magnitude*, so it is a sphere
		// rather than an interval and b3ClampImp3 is what applies it. Clamping
		// each component separately would let a diagonal torque exceed the
		// budget by sqrt(3).
		//
		// The clamp lands on the accumulator and the delta is recomputed from
		// it, which is Stage 2's rule and matters more here than anywhere: a
		// 3-vector clamp that scaled the delta instead would rotate the stored
		// impulse and quietly break warm starting.
		b3imp maxImpulse = b3MulFTToImp( joint->maxMotorTorque, h );
		joint->motorImpulse = b3ClampImp3( b3AddImp3( oldImpulse, lambda ), maxImpulse );
		lambda = b3SubImp3( joint->motorImpulse, oldImpulse );

		wA = b3Sub( wA, b3MulMWImp( iA, lambda ) );
		wB = b3Add( wB, b3MulMWImp( iB, lambda ) );
	}

	// -----------------------------------------------------------------
	// Twist limit
	// -----------------------------------------------------------------
	if ( joint->enableTwistLimit && fixedRotation == false )
	{
		b3a twistAngle = b3GetTwistAngle( relQ );
		b3f twistMass = joint->twistMass;

		// Lower limit.
		{
			// Scaled by cos(theta/2) because the Jacobian was normalized -- see
			// the file header. Both branches below are linear in c, so scaling
			// it once here covers the speculative and the softened path alike,
			// and the sign is preserved because the scale is non-negative.
			b3f c = b3MulFC( b3BradToRadF( (b3a)( twistAngle - joint->lowerTwistAngle ) ), twistScale );

			b3f bias = b3f_zero;
			b3c massScale = b3c_one;
			b3c impulseScale = b3c_zero;
			if ( b3Raw( c ) > 0 )
			{
				// Speculative: still inside the range, so the constraint may
				// only stop it closing faster than one sub-step of slack.
				bias = b3MulFF( c, context->inv_h );
			}
			else if ( useBias )
			{
				b3Softness cs = base->constraintSoftness;
				bias = b3MulFF( b3MulFC( cs.biasRate, cs.massScale ), c );
				massScale = cs.massScale;
				impulseScale = cs.impulseScale;
			}

			b3f cdot = b3Dot( b3Sub( wB, wA ), twistAxis );
			b3f driving = b3AddF( b3MulFC( cdot, massScale ), bias );

			b3imp oldImpulse = joint->lowerTwistImpulse;
			b3imp deltaImpulse =
				b3SubImp( b3NegImp( b3MulFFToImp( twistMass, driving ) ), b3MulImpC( oldImpulse, impulseScale ) );

			joint->lowerTwistImpulse = b3MaxImp( b3AddImp( oldImpulse, deltaImpulse ), b3imp_zero );
			deltaImpulse = b3SubImp( joint->lowerTwistImpulse, oldImpulse );

			wA = b3Sub( wA, b3MulMWImp( iA, b3MulImpV( deltaImpulse, twistAxis ) ) );
			wB = b3Add( wB, b3MulMWImp( iB, b3MulImpV( deltaImpulse, twistAxis ) ) );
		}

		// Upper limit. The same constraint the other way round, which is why the
		// relative velocity is formed A-minus-B and the impulse is applied with
		// its sign flipped.
		{
			b3f c = b3MulFC( b3BradToRadF( (b3a)( joint->upperTwistAngle - twistAngle ) ), twistScale );

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

			b3f cdot = b3Dot( b3Sub( wA, wB ), twistAxis );
			b3f driving = b3AddF( b3MulFC( cdot, massScale ), bias );

			b3imp oldImpulse = joint->upperTwistImpulse;
			b3imp deltaImpulse =
				b3SubImp( b3NegImp( b3MulFFToImp( twistMass, driving ) ), b3MulImpC( oldImpulse, impulseScale ) );

			joint->upperTwistImpulse = b3MaxImp( b3AddImp( oldImpulse, deltaImpulse ), b3imp_zero );
			deltaImpulse = b3SubImp( joint->upperTwistImpulse, oldImpulse );

			wA = b3Add( wA, b3MulMWImp( iA, b3MulImpV( deltaImpulse, twistAxis ) ) );
			wB = b3Sub( wB, b3MulMWImp( iB, b3MulImpV( deltaImpulse, twistAxis ) ) );
		}
	}

	// -----------------------------------------------------------------
	// Cone limit
	// -----------------------------------------------------------------
	if ( joint->enableConeLimit && fixedRotation == false )
	{
		b3a swingAngle = b3GetSwingAngle( relQ );

		// The swing is unsigned and the cone is one-sided, so there is one
		// branch here where the twist has two. No scaling: the swing axis is a
		// unit vector as it stands.
		b3f c = b3BradToRadF( (b3a)( joint->coneAngle - swingAngle ) );

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

		// Sign flipped on Cdot, and on the applied impulse below: a growing
		// swing must be opposed.
		b3f cdot = b3Dot( b3Sub( wA, wB ), swingAxis );
		b3f driving = b3AddF( b3MulFC( cdot, massScale ), bias );

		b3imp oldImpulse = joint->swingImpulse;
		b3imp deltaImpulse =
			b3SubImp( b3NegImp( b3MulFFToImp( joint->swingMass, driving ) ), b3MulImpC( oldImpulse, impulseScale ) );

		joint->swingImpulse = b3MaxImp( b3AddImp( oldImpulse, deltaImpulse ), b3imp_zero );
		deltaImpulse = b3SubImp( joint->swingImpulse, oldImpulse );

		wA = b3Add( wA, b3MulMWImp( iA, b3MulImpV( deltaImpulse, swingAxis ) ) );
		wB = b3Sub( wB, b3MulMWImp( iB, b3MulImpV( deltaImpulse, swingAxis ) ) );
	}

	// -----------------------------------------------------------------
	// Point to point: the 3x3 that locks the two anchors together
	// -----------------------------------------------------------------
	//
	// Identical to revolute_joint.c's, deliberately -- a ball joint and a hinge
	// constrain position the same way and differ only in what they leave free.
	{
		b3Vec3 rA = b3RotateVector( stateA->deltaRotation, joint->frameA.p );
		b3Vec3 rB = b3RotateVector( stateB->deltaRotation, joint->frameB.p );

		b3Vec3 cdot = b3Sub( b3Sub( b3Add( vB, b3Cross( wB, rB ) ), vA ), b3Cross( wA, rA ) );

		b3Vec3 bias = b3Vec3_zero;
		b3c massScale = b3c_one;
		b3c impulseScale = b3c_zero;
		if ( useBias )
		{
			// deltaPosition is Q24, so the difference is taken there and
			// narrowed once -- this is a difference of two nearly equal numbers
			// and rounding each side first rounds twice.
			b3Vec3 dp = b3W3ToVec3( b3SubW3( stateB->deltaPosition, stateA->deltaPosition ) );
			b3Vec3 separation = b3Add( b3Add( dp, b3Sub( rB, rA ) ), joint->deltaCenter );

			b3Softness cs = base->constraintSoftness;
			bias = b3MulSV( b3MulFC( cs.biasRate, cs.massScale ), separation );
			massScale = cs.massScale;
			impulseScale = cs.impulseScale;
		}

		b3Matrix3 invK = b3InvertPointMass( mA, iA, rA, mB, iB, rB );

		b3Vec3 driving = b3Add( b3MulSV( b3CToF( massScale ), cdot ), bias );
		b3Imp3 sol = b3MulMVToImp( invK, driving );

		b3Imp3 impulse = b3SubImp3( b3NegImp3( sol ), b3MulCImp3( impulseScale, joint->linearImpulse ) );
		joint->linearImpulse = b3AddImp3( joint->linearImpulse, impulse );

		vA = b3Sub( vA, b3MulImpW3( impulse, mA ) );
		wA = b3Sub( wA, b3MulMWImp( iA, b3CrossVImp( rA, impulse ) ) );
		vB = b3Add( vB, b3MulImpW3( impulse, mB ) );
		wB = b3Add( wB, b3MulMWImp( iB, b3CrossVImp( rB, impulse ) ) );
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
