// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Erin Catto        (original Box3D)
// Copyright (c) 2026 Warioware64       (Nitro Engine Advanced fixed-point port)
//
// This file is part of Nitro Engine Advanced

/// @file   revolute_joint.c
/// @brief  The revolute joint: a hinge, and the first constrained rotation.
///
/// @section shape What a hinge actually is
///
/// Three constraints solved in sequence, not one:
///
///   1. a **3x3 point-to-point** constraint locking the two anchors together,
///      which removes all three linear degrees of freedom;
///   2. a **2x2 collinearity** constraint holding the two hinge axes parallel,
///      which removes two of the three rotational degrees;
///   3. a **scalar axial** constraint on what is left -- the hinge angle --
///      carrying the spring, the motor and the angle limit.
///
/// They couple through the shared velocity state, which is why the order
/// matters and why upstream's order is kept exactly. Unlike the distance
/// joint's four near-identical branches, these are structurally different, and
/// a sign error in the second reads as a slightly loose hinge rather than as a
/// failure -- which is what the closed-form pendulum period test exists for.
///
/// @section scales What is new here, and what is not
///
/// Almost every scale convention is Stage 2's, unchanged: the effective mass
/// accumulated at Q24 and reciprocated once, `massScale` folded into each term
/// at Q30, the clamp landing on the accumulator with the delta recomputed. The
/// three things this file adds:
///
///   - **Two matrix effective masses.** Both are symmetric, and both are built
///     by a routine that never forms `invIA + invIB`: b3InvertPointMass for
///     the 3x3 and b3InvertPerpMass for the 2x2. Upstream's b3Solve3 and
///     b3Solve2 are Gaussian solves with no port counterpart and none is
///     needed. Stage 3 wrote this file against `b3AddMWMW( iA, iB )`, which
///     wraps for two light bodies; Stage 5 closed that -- see prepare.
///   - **Angles.** The public API is brads; the constraint is radians. The
///     conversion happens on the *difference* against a limit, once, so its
///     sub-quantum error does not depend on where the hinge sits.
///   - **The singular case.** `fixedRotation` is upstream's
///     `b3Det( invIA + invIB ) < 1000 * FLT_MIN`. The port asks
///     b3InvertRotationMass instead, which already reports a singular matrix by
///     returning zero -- one tested code path rather than a second threshold.
///     b3FixedRotationFromMass in joint.h is that test, shared by four joints.
///
/// @section absent What is not here
///
/// b3DrawRevoluteJoint and every B3_REC hook, as in every other joint file.

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

void b3RevoluteJoint_EnableSpring( b3JointId jointId, bool enableSpring )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_revoluteJoint );
	if ( enableSpring != base->revoluteJoint.enableSpring )
	{
		base->revoluteJoint.enableSpring = enableSpring;
		base->revoluteJoint.springImpulse = b3imp_zero;
	}
}

bool b3RevoluteJoint_IsSpringEnabled( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_revoluteJoint );
	return base->revoluteJoint.enableSpring;
}

void b3RevoluteJoint_SetSpringHertz( b3JointId jointId, b3f hertz )
{
	B3_ASSERT( b3IsValidFloat( hertz ) && b3Raw( hertz ) >= 0 );

	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_revoluteJoint );
	base->revoluteJoint.hertz = hertz;
}

b3f b3RevoluteJoint_GetSpringHertz( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_revoluteJoint );
	return base->revoluteJoint.hertz;
}

void b3RevoluteJoint_SetSpringDampingRatio( b3JointId jointId, b3f dampingRatio )
{
	B3_ASSERT( b3IsValidFloat( dampingRatio ) && b3Raw( dampingRatio ) >= 0 );

	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_revoluteJoint );
	base->revoluteJoint.dampingRatio = dampingRatio;
}

b3f b3RevoluteJoint_GetSpringDampingRatio( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_revoluteJoint );
	return base->revoluteJoint.dampingRatio;
}

void b3RevoluteJoint_SetTargetAngle( b3JointId jointId, b3a angle )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_revoluteJoint );
	base->revoluteJoint.targetAngle = angle;
}

b3a b3RevoluteJoint_GetTargetAngle( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_revoluteJoint );
	return base->revoluteJoint.targetAngle;
}

void b3RevoluteJoint_EnableLimit( b3JointId jointId, bool enableLimit )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_revoluteJoint );
	if ( enableLimit != base->revoluteJoint.enableLimit )
	{
		base->revoluteJoint.enableLimit = enableLimit;
		base->revoluteJoint.lowerImpulse = b3imp_zero;
		base->revoluteJoint.upperImpulse = b3imp_zero;
	}
}

bool b3RevoluteJoint_IsLimitEnabled( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_revoluteJoint );
	return base->revoluteJoint.enableLimit;
}

void b3RevoluteJoint_SetLimits( b3JointId jointId, b3a lower, b3a upper )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_revoluteJoint );
	b3RevoluteJoint* joint = &base->revoluteJoint;

	// Sorted rather than asserted, as the distance joint's length range is: a
	// caller who passes them the wrong way round gets the range they meant.
	joint->lowerAngle = lower < upper ? lower : upper;
	joint->upperAngle = lower < upper ? upper : lower;

	joint->lowerImpulse = b3imp_zero;
	joint->upperImpulse = b3imp_zero;
}

b3a b3RevoluteJoint_GetLowerLimit( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_revoluteJoint );
	return base->revoluteJoint.lowerAngle;
}

b3a b3RevoluteJoint_GetUpperLimit( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_revoluteJoint );
	return base->revoluteJoint.upperAngle;
}

void b3RevoluteJoint_EnableMotor( b3JointId jointId, bool enableMotor )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_revoluteJoint );
	if ( enableMotor != base->revoluteJoint.enableMotor )
	{
		base->revoluteJoint.enableMotor = enableMotor;
		base->revoluteJoint.motorImpulse = b3imp_zero;
	}
}

bool b3RevoluteJoint_IsMotorEnabled( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_revoluteJoint );
	return base->revoluteJoint.enableMotor;
}

void b3RevoluteJoint_SetMotorSpeed( b3JointId jointId, b3f motorSpeed )
{
	B3_ASSERT( b3IsValidFloat( motorSpeed ) );

	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_revoluteJoint );
	base->revoluteJoint.motorSpeed = motorSpeed;
}

b3f b3RevoluteJoint_GetMotorSpeed( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_revoluteJoint );
	return base->revoluteJoint.motorSpeed;
}

void b3RevoluteJoint_SetMaxMotorTorque( b3JointId jointId, b3f torque )
{
	B3_ASSERT( b3IsValidFloat( torque ) && b3Raw( torque ) >= 0 );

	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_revoluteJoint );
	base->revoluteJoint.maxMotorTorque = torque;
}

b3f b3RevoluteJoint_GetMaxMotorTorque( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_revoluteJoint );
	return base->revoluteJoint.maxMotorTorque;
}

b3f b3RevoluteJoint_GetMotorTorque( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_revoluteJoint );
	return b3MulImpFToF( base->revoluteJoint.motorImpulse, world->inv_h );
}

b3a b3RevoluteJoint_GetAngle( b3JointId jointId )
{
	b3World* world = b3GetUnlockedWorld( jointId.world0 );
	if ( world == NULL )
	{
		return 0;
	}

	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_revoluteJoint );

	b3WorldTransform transformA = b3GetBodyTransform( world, base->bodyIdA );
	b3WorldTransform transformB = b3GetBodyTransform( world, base->bodyIdB );

	// The two joint frames in world space, then the rotation of B relative to
	// A -- whose twist about z is the hinge angle by definition.
	b3Quat qA = b3MulQuat( transformA.q, base->localFrameA.q );
	b3Quat qB = b3MulQuat( transformB.q, base->localFrameB.q );

	b3Quat relQ = b3InvMulQuat( qA, qB );
	return (b3a)( b3GetTwistAngle( relQ ) - base->revoluteJoint.targetAngle );
}

/// The force the point-to-point constraint is applying, in world space.
b3Vec3 b3GetRevoluteJointForce( b3World* world, b3JointSim* base )
{
	b3Imp3 p = base->revoluteJoint.linearImpulse;
	b3f inv_h = world->inv_h;
	return b3MakeVec3( b3MulImpFToF( p.x, inv_h ), b3MulImpFToF( p.y, inv_h ), b3MulImpFToF( p.z, inv_h ) );
}

/// The torque the joint is applying, in world space.
///
/// Two contributions: the collinearity constraint along the two perpendicular
/// axes, and the axial accumulators along the hinge axis. Upstream adds the
/// axial term twice -- once through `rotationAxisZ` and once through a
/// separately recomputed `axis` -- which is a bug in a debug-only readout; the
/// port adds it once.
b3Vec3 b3GetRevoluteJointTorque( b3World* world, b3JointSim* base )
{
	b3RevoluteJoint* joint = &base->revoluteJoint;

	b3imp axialImpulse = b3SubImp( b3AddImp( b3AddImp( joint->springImpulse, joint->motorImpulse ), joint->lowerImpulse ),
								   joint->upperImpulse );

	b3Imp3 angular = b3AddImp3( b3MulImpV( joint->perpImpulse.x, joint->perpAxisX ),
								b3MulImpV( joint->perpImpulse.y, joint->perpAxisY ) );
	angular = b3AddImp3( angular, b3MulImpV( axialImpulse, joint->rotationAxisZ ) );

	b3f inv_h = world->inv_h;
	return b3MakeVec3( b3MulImpFToF( angular.x, inv_h ), b3MulImpFToF( angular.y, inv_h ),
					   b3MulImpFToF( angular.z, inv_h ) );
}

// =========================================================================
// Solver
// =========================================================================

// The two collinearity axes come from b3CollinearityPerpAxes in joint.h. They
// lived here as a `static` through Stages 3 to 6; Stage 7 gave them callers in
// the parallel and wheel joints, so they moved rather than being copied.

void b3PrepareRevoluteJoint( b3JointSim* base, b3StepContext* context )
{
	B3_ASSERT( base->type == b3_revoluteJoint );

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

	// The sum `invIA + invIB` is never formed, here or in the 2x2 block below.
	//
	// b3InvertInertia caps a *single* body's inverse inertia at Q7.24's ceiling
	// of 128 by scaling rather than wrapping, so neither matrix can overflow --
	// but their sum can, and b3AddMWMW wrapped it to a large negative, which
	// inverted the sign of the effective mass. Two bodies light enough to reach
	// it is an ordinary scene, not a corner: Stage 4's ragdoll hit 128.197 with
	// a 0.18 kg head against a 0.59 kg torso and was flung apart on the first
	// frame it was upright.
	//
	// So every quantity that used to read through the sum now goes through a
	// helper that applies each matrix separately at Q12 -- where 256 is nothing
	// against the 524287 the scale holds -- and accumulates only the final
	// quadratic form wide. b3RcpWide and b3InvertPerpMass delegate to the
	// narrow routines for anything already in range, so results that were
	// correct before are bit-identical and only the wrapping case moved.
	base->fixedRotation = b3FixedRotationFromMass( b3InvertRotationMass( base->invIA, base->invIB ) );

	b3RevoluteJoint* joint = &base->revoluteJoint;

	joint->indexA = bodyA->setIndex == b3_awakeSet ? localIndexA : B3_NULL_INDEX;
	joint->indexB = bodyB->setIndex == b3_awakeSet ? localIndexB : B3_NULL_INDEX;

	// Joint frames in world space, with the position relative to the centre of
	// mass. Written as one rotation of (localFrame.p - localCenter) rather
	// than as a transform followed by a subtraction, which is upstream's
	// round-off note: the two centres can be far from the origin and the
	// difference of two large numbers is where the bits go.
	joint->frameA.q = b3MulQuat( bodySimA->transform.q, base->localFrameA.q );
	joint->frameA.p = b3RotateVector( bodySimA->transform.q, b3Sub( base->localFrameA.p, bodySimA->localCenter ) );
	joint->frameB.q = b3MulQuat( bodySimB->transform.q, base->localFrameB.q );
	joint->frameB.p = b3RotateVector( bodySimB->transform.q, b3Sub( base->localFrameB.p, bodySimB->localCenter ) );

	joint->deltaCenter = b3Sub( bodySimB->center, bodySimA->center );

	{
		// The hinge axis is frame A's z. The axial effective mass follows the
		// contact solver's shape exactly: a Q24 inverse-inertia accumulated
		// through b3DotVWide, reciprocated once by b3RcpW into a Q12 mass.
		//
		// The axis is a plain Q12 vector rather than a Q30 b3Dir3, matching
		// upstream and matching the contact solver's lever arms. It is used as
		// the impulse direction and as the mass direction, so the two Q12
		// roundings are the same rounding and largely cancel.
		b3Vec3 axis = b3RotateVector( joint->frameA.q, b3Vec3_axisZ );
		joint->axialMass = b3RcpWide( b3AxisInertiaSumWide( axis, base->invIA, base->invIB ) );
		joint->rotationAxisZ = axis;
	}

	b3Quat relQ = b3InvMulQuat( joint->frameA.q, joint->frameB.q );
	b3CollinearityPerpAxes( joint->frameA.q, relQ, &joint->perpAxisX, &joint->perpAxisY );

	joint->springSoftness = b3MakeSoft( joint->hertz, joint->dampingRatio, context->h );

	if ( context->enableWarmStarting == false )
	{
		joint->linearImpulse = b3Imp3_zero;
		joint->perpImpulse.x = b3imp_zero;
		joint->perpImpulse.y = b3imp_zero;
		joint->springImpulse = b3imp_zero;
		joint->motorImpulse = b3imp_zero;
		joint->lowerImpulse = b3imp_zero;
		joint->upperImpulse = b3imp_zero;
	}
}

/// @note ITCM group B3_ITCM_REVOLUTE -- see nea_config.h.
void B3_ITCM_IF( B3_ITCM_REVOLUTE, b3WarmStartRevoluteJoint )( b3JointSim* base, b3StepContext* context )
{
	B3_ASSERT( base->type == b3_revoluteJoint );

	b3iw mA = base->invMassA;
	b3iw mB = base->invMassB;
	b3MatrixW iA = base->invIA;
	b3MatrixW iB = base->invIB;

	b3BodyState dummyState = b3_identityBodyState;

	b3RevoluteJoint* joint = &base->revoluteJoint;
	b3BodyState* stateA = joint->indexA == B3_NULL_INDEX ? &dummyState : context->states + joint->indexA;
	b3BodyState* stateB = joint->indexB == B3_NULL_INDEX ? &dummyState : context->states + joint->indexB;

	b3Vec3 rA = b3RotateVector( stateA->deltaRotation, joint->frameA.p );
	b3Vec3 rB = b3RotateVector( stateB->deltaRotation, joint->frameB.p );

	b3imp axialImpulse = b3SubImp( b3AddImp( b3AddImp( joint->springImpulse, joint->motorImpulse ), joint->lowerImpulse ),
								   joint->upperImpulse );

	b3Imp3 angular = b3AddImp3( b3MulImpV( joint->perpImpulse.x, joint->perpAxisX ),
								b3MulImpV( joint->perpImpulse.y, joint->perpAxisY ) );
	angular = b3AddImp3( angular, b3MulImpV( axialImpulse, joint->rotationAxisZ ) );

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

/// @note ITCM group B3_ITCM_REVOLUTE -- see nea_config.h.
void B3_ITCM_IF( B3_ITCM_REVOLUTE, b3SolveRevoluteJoint )( b3JointSim* base, b3StepContext* context, bool useBias )
{
	B3_ASSERT( base->type == b3_revoluteJoint );

	b3iw mA = base->invMassA;
	b3iw mB = base->invMassB;
	b3MatrixW iA = base->invIA;
	b3MatrixW iB = base->invIB;

	b3BodyState dummyState = b3_identityBodyState;

	b3RevoluteJoint* joint = &base->revoluteJoint;
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
	// nearer of the two is what keeps the hinge angle in [-pi, pi] instead of
	// jumping a full turn when the joint crosses the half-way point.
	if ( b3Raw( b3DotQuat( quatA, quatB ) ) < 0 )
	{
		quatB = b3NegateQuat( quatB );
	}

	b3Quat relQ = b3InvMulQuat( quatA, quatB );

	b3f axialMass = joint->axialMass;
	b3Vec3 axis = joint->rotationAxisZ;

	// -----------------------------------------------------------------
	// Axial spring
	// -----------------------------------------------------------------
	if ( joint->enableSpring && fixedRotation == false )
	{
		// The angle error, in radians. Both operands are brads and the
		// difference is taken there -- converting each separately would round
		// twice and make the error depend on the hinge's absolute position
		// rather than on how far it is from its target.
		b3a angle = b3GetTwistAngle( relQ );
		b3f c = b3BradToRadF( (b3a)( angle - joint->targetAngle ) );

		b3Softness soft = joint->springSoftness;
		b3f bias = b3MulFF( b3MulFC( soft.biasRate, soft.massScale ), c );
		b3f cdot = b3Dot( b3Sub( wB, wA ), axis );
		b3f driving = b3AddF( b3MulFC( cdot, soft.massScale ), bias );

		b3imp deltaImpulse = b3SubImp( b3NegImp( b3MulFFToImp( axialMass, driving ) ),
									   b3MulImpC( joint->springImpulse, soft.impulseScale ) );
		joint->springImpulse = b3AddImp( joint->springImpulse, deltaImpulse );

		wA = b3Sub( wA, b3MulMWImp( iA, b3MulImpV( deltaImpulse, axis ) ) );
		wB = b3Add( wB, b3MulMWImp( iB, b3MulImpV( deltaImpulse, axis ) ) );
	}

	// -----------------------------------------------------------------
	// Axial motor
	// -----------------------------------------------------------------
	if ( joint->enableMotor && fixedRotation == false )
	{
		b3f cdot = b3SubF( b3Dot( b3Sub( wB, wA ), axis ), joint->motorSpeed );

		b3imp oldImpulse = joint->motorImpulse;
		b3imp deltaImpulse = b3NegImp( b3MulFFToImp( axialMass, cdot ) );

		// A torque bound becomes an impulse bound over one sub-step, which is
		// what b3MulFTToImp spells -- the same conversion the distance joint's
		// motor makes against a force.
		b3imp maxImpulse = b3MulFTToImp( joint->maxMotorTorque, h );
		joint->motorImpulse = b3ClampImp( b3AddImp( oldImpulse, deltaImpulse ), b3NegImp( maxImpulse ), maxImpulse );
		deltaImpulse = b3SubImp( joint->motorImpulse, oldImpulse );

		wA = b3Sub( wA, b3MulMWImp( iA, b3MulImpV( deltaImpulse, axis ) ) );
		wB = b3Add( wB, b3MulMWImp( iB, b3MulImpV( deltaImpulse, axis ) ) );
	}

	// -----------------------------------------------------------------
	// Axial limit
	// -----------------------------------------------------------------
	if ( joint->enableLimit && fixedRotation == false )
	{
		b3a angle = (b3a)( b3GetTwistAngle( relQ ) - joint->targetAngle );

		// Lower limit.
		{
			b3f c = b3BradToRadF( (b3a)( angle - joint->lowerAngle ) );

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

			b3f cdot = b3Dot( b3Sub( wB, wA ), axis );
			b3f driving = b3AddF( b3MulFC( cdot, massScale ), bias );

			b3imp oldImpulse = joint->lowerImpulse;
			b3imp deltaImpulse =
				b3SubImp( b3NegImp( b3MulFFToImp( axialMass, driving ) ), b3MulImpC( oldImpulse, impulseScale ) );

			joint->lowerImpulse = b3MaxImp( b3AddImp( oldImpulse, deltaImpulse ), b3imp_zero );
			deltaImpulse = b3SubImp( joint->lowerImpulse, oldImpulse );

			wA = b3Sub( wA, b3MulMWImp( iA, b3MulImpV( deltaImpulse, axis ) ) );
			wB = b3Add( wB, b3MulMWImp( iB, b3MulImpV( deltaImpulse, axis ) ) );
		}

		// Upper limit. The same constraint the other way round, which is why
		// the relative velocity is formed B-minus-A reversed and the impulse
		// is applied with its sign flipped.
		{
			b3f c = b3BradToRadF( (b3a)( joint->upperAngle - angle ) );

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

			b3f cdot = b3Dot( b3Sub( wA, wB ), axis );
			b3f driving = b3AddF( b3MulFC( cdot, massScale ), bias );

			b3imp oldImpulse = joint->upperImpulse;
			b3imp deltaImpulse =
				b3SubImp( b3NegImp( b3MulFFToImp( axialMass, driving ) ), b3MulImpC( oldImpulse, impulseScale ) );

			joint->upperImpulse = b3MaxImp( b3AddImp( oldImpulse, deltaImpulse ), b3imp_zero );
			deltaImpulse = b3SubImp( joint->upperImpulse, oldImpulse );

			wA = b3Add( wA, b3MulMWImp( iA, b3MulImpV( deltaImpulse, axis ) ) );
			wB = b3Sub( wB, b3MulMWImp( iB, b3MulImpV( deltaImpulse, axis ) ) );
		}
	}

	// -----------------------------------------------------------------
	// Collinearity: the 2x2 that holds the two hinge axes parallel
	// -----------------------------------------------------------------
	if ( fixedRotation == false )
	{
		b3f biasX = b3f_zero;
		b3f biasY = b3f_zero;
		b3c massScale = b3c_one;
		b3c impulseScale = b3c_zero;

		if ( useBias )
		{
			// The constraint is on the x and y components of the relative
			// rotation's vector part -- they are zero exactly when the two z
			// axes are collinear.
			b3Softness cs = base->constraintSoftness;
			b3f rate = b3MulFC( cs.biasRate, cs.massScale );
			biasX = b3MulFF( rate, b3NToF( relQ.v.x ) );
			biasY = b3MulFF( rate, b3NToF( relQ.v.y ) );
			massScale = cs.massScale;
			impulseScale = cs.impulseScale;
		}

		b3Vec3 perpAxisX, perpAxisY;
		b3CollinearityPerpAxes( quatA, relQ, &perpAxisX, &perpAxisY );
		joint->perpAxisX = perpAxisX;
		joint->perpAxisY = perpAxisY;

		// The symmetric 2x2 effective mass, accumulated at Q24 like every
		// other one in the port and inverted by the routine central friction
		// already uses -- reached through b3InvertPerpMass, which never forms
		// `iA + iB`, for the reason prepare gives above. Singular comes back
		// zero, which means no impulse: the right answer when the two axes
		// give the joint nothing to push against.
		b3SymMatrix2 invK = b3InvertPerpMass( perpAxisX, perpAxisY, iA, iB );

		b3Vec3 wRel = b3Sub( wB, wA );
		b3f cdotX = b3AddF( b3MulFC( b3Dot( wRel, perpAxisX ), massScale ), biasX );
		b3f cdotY = b3AddF( b3MulFC( b3Dot( wRel, perpAxisY ), massScale ), biasY );

		b3Imp2 sol = b3MulSym2V( invK, cdotX, cdotY );

		b3Imp2 deltaImpulse;
		deltaImpulse.x = b3SubImp( b3NegImp( sol.x ), b3MulImpC( joint->perpImpulse.x, impulseScale ) );
		deltaImpulse.y = b3SubImp( b3NegImp( sol.y ), b3MulImpC( joint->perpImpulse.y, impulseScale ) );

		joint->perpImpulse.x = b3AddImp( joint->perpImpulse.x, deltaImpulse.x );
		joint->perpImpulse.y = b3AddImp( joint->perpImpulse.y, deltaImpulse.y );

		b3Imp3 angular =
			b3AddImp3( b3MulImpV( deltaImpulse.x, perpAxisX ), b3MulImpV( deltaImpulse.y, perpAxisY ) );

		wA = b3Sub( wA, b3MulMWImp( iA, angular ) );
		wB = b3Add( wB, b3MulMWImp( iB, angular ) );
	}

	// -----------------------------------------------------------------
	// Point to point: the 3x3 that locks the two anchors together
	// -----------------------------------------------------------------
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
			// narrowed once -- the same treatment the contact solver gives it,
			// and for the same reason: this is a difference of two nearly
			// equal numbers and rounding each side first rounds twice.
			b3Vec3 dp = b3W3ToVec3( b3SubW3( stateB->deltaPosition, stateA->deltaPosition ) );
			b3Vec3 separation = b3Add( b3Add( dp, b3Sub( rB, rA ) ), joint->deltaCenter );

			b3Softness cs = base->constraintSoftness;
			bias = b3MulSV( b3MulFC( cs.biasRate, cs.massScale ), separation );
			massScale = cs.massScale;
			impulseScale = cs.impulseScale;
		}

		// K = (mA + mB) * I - (skew(rA) * iA * skew(rA) + skew(rB) * iB * skew(rB))
		//
		// Built and inverted in one call, because K does not fit Q24 the way
		// the contact solver's kNormal does -- a joint's lever arm is as long
		// as the designer made it, and the product grows as its square. See
		// b3InvertPointMass, which accumulates wide and normalizes.
		//
		// Upstream reaches the same answer with a Gaussian b3Solve3; the port
		// inverts, which is symmetric-safe here and reuses a routine that
		// already carries its determinant at Q36.
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
