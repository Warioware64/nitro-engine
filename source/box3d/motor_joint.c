// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Erin Catto        (original Box3D)
// Copyright (c) 2026 Warioware64       (Nitro Engine Advanced fixed-point port)
//
// This file is part of Nitro Engine Advanced

/// @file   motor_joint.c
/// @brief  The motor joint: a bounded 6-DOF drive, and what a moving platform
///         actually wants.
///
/// @section shape What a motor joint actually is
///
/// It constrains nothing. Every other joint in the port *removes* degrees of
/// freedom; this one removes none and instead pushes the two bodies toward a
/// target, with a hard ceiling on how much force and torque it may spend doing
/// so. That ceiling is the point. A platform moved with b3Body_SetTransform
/// teleports through whatever is in the way; a platform driven by a motor joint
/// goes where it is told and still loses to a wall, and the crate riding on it
/// is pushed rather than passed through.
///
/// Four independent branches, solved in this order:
///
///   1. **angular spring** -- toward the target orientation, bounded by
///      `maxSpringTorque`;
///   2. **angular velocity** -- toward `angularVelocity`, bounded by
///      `maxVelocityTorque`;
///   3. **linear spring** -- toward the target position, bounded by
///      `maxSpringForce`;
///   4. **linear velocity** -- toward `linearVelocity`, bounded by
///      `maxVelocityForce`.
///
/// Structurally that is weld_joint.c's two constraints, each split into a
/// spring half and a velocity half, with a bound on each. Every bound is on the
/// **magnitude** of a 3-vector, so each is a sphere rather than an interval and
/// b3ClampImp3 applies it -- the spherical motor's rule, and for its reason:
/// clamping each component separately would let a diagonal drive exceed its
/// budget by sqrt(3). The clamp lands on the accumulator and the delta is
/// recomputed from it, never the other way round; a 3-vector clamp applied to
/// the delta would rotate the stored impulse and quietly break warm starting.
///
/// A branch with a zero bound is skipped entirely rather than solved and then
/// clamped to nothing, which is upstream's structure and is also how a caller
/// disables one.
///
/// @section hoist The one departure from upstream, and it is a saving
///
/// Upstream builds the linear effective mass `K` and solves it **twice** --
/// once in the linear spring branch ([motor_joint.c:364-373](helpSrc/box3d-main/src/motor_joint.c#L364-L373))
/// and again in the linear velocity branch ([motor_joint.c:399-408](helpSrc/box3d-main/src/motor_joint.c#L399-L408)).
/// The two are built from identical inputs: the same `mA`/`mB`, the same
/// `iA`/`iB`, and the same `rA`/`rB`, which upstream computes once above both
/// and does not touch in between. The second build is bit-identical to the
/// first.
///
/// So the port inverts once, above both branches, and reuses the result. This
/// is not an approximation and it changes no answer -- it removes a duplicate
/// 3x3 inversion, which is the most expensive single operation in the file, per
/// sub-step, per motor joint.
///
/// @section scales What is new here, and what is not
///
/// Almost nothing, as in weld_joint.c: `b3InvertRotationMass` for the two
/// angular branches, `b3InvertPointMass` for the two linear ones, `b3ClampImp3`
/// for the four bounds, `b3DeltaQuatToRotation` for the orientation error. All
/// four arrived in Stages 3 and 4.
///
/// The one addition is **b3MulMVToImpSat**, and this file is the only caller.
/// Every other joint may use b3MulMVToImp, which saturates each component
/// independently -- for a rigid constraint an impulse that reaches the ceiling
/// means the scene is already lost. A *bounded* drive is different in kind: it
/// is expected to ask for more than it may spend, every step, whenever the load
/// exceeds the budget. And when a spring cannot hold what it was pointed at,
/// the body falls away and the position error -- and the bias built from it --
/// grows without limit, so the raw ask genuinely reaches Q16's ceiling in an
/// ordinary scene rather than a contrived one.
///
/// At that point saturating each component separately **rotates the vector**,
/// because three components clip by different amounts, and b3ClampImp3 then
/// bounds the wrong direction: an under-powered drive pushes askew rather than
/// simply weakly. b3MulMVToImpSat scales the whole vector down instead, which
/// preserves the direction and discards only bits the clamp was about to throw
/// away anyway. A result that fits is bit-identical to b3MulMVToImp's.
///
/// One number worth carrying from Stage 4: a bound below roughly **0.004 N·m at
/// 240 Hz** lands under half a Q16 quantum and clamps the motor to nothing. It
/// is a real limit of the impulse scale, not a bug, and it is the reason a
/// motor with a plausible-looking tiny bound appears dead.
///
/// @section absent What is not here
///
/// b3DrawMotorJoint and every B3_REC hook, as in every other joint file.

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

void b3MotorJoint_SetLinearVelocity( b3JointId jointId, b3Vec3 velocity )
{
	B3_ASSERT( b3IsValidVec3( velocity ) );

	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_motorJoint );
	base->motorJoint.linearVelocity = velocity;
}

b3Vec3 b3MotorJoint_GetLinearVelocity( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_motorJoint );
	return base->motorJoint.linearVelocity;
}

void b3MotorJoint_SetAngularVelocity( b3JointId jointId, b3Vec3 velocity )
{
	B3_ASSERT( b3IsValidVec3( velocity ) );

	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_motorJoint );
	base->motorJoint.angularVelocity = velocity;
}

b3Vec3 b3MotorJoint_GetAngularVelocity( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_motorJoint );
	return base->motorJoint.angularVelocity;
}

void b3MotorJoint_SetMaxVelocityForce( b3JointId jointId, b3f maxForce )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_motorJoint );
	base->motorJoint.maxVelocityForce = b3MaxF( b3f_zero, maxForce );
}

b3f b3MotorJoint_GetMaxVelocityForce( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_motorJoint );
	return base->motorJoint.maxVelocityForce;
}

void b3MotorJoint_SetMaxVelocityTorque( b3JointId jointId, b3f maxTorque )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_motorJoint );
	base->motorJoint.maxVelocityTorque = b3MaxF( b3f_zero, maxTorque );
}

b3f b3MotorJoint_GetMaxVelocityTorque( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_motorJoint );
	return base->motorJoint.maxVelocityTorque;
}

void b3MotorJoint_SetMaxSpringForce( b3JointId jointId, b3f maxForce )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_motorJoint );
	base->motorJoint.maxSpringForce = b3MaxF( b3f_zero, maxForce );
}

b3f b3MotorJoint_GetMaxSpringForce( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_motorJoint );
	return base->motorJoint.maxSpringForce;
}

void b3MotorJoint_SetMaxSpringTorque( b3JointId jointId, b3f maxTorque )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_motorJoint );
	base->motorJoint.maxSpringTorque = b3MaxF( b3f_zero, maxTorque );
}

b3f b3MotorJoint_GetMaxSpringTorque( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_motorJoint );
	return base->motorJoint.maxSpringTorque;
}

void b3MotorJoint_SetLinearHertz( b3JointId jointId, b3f hertz )
{
	B3_ASSERT( b3IsValidFloat( hertz ) && b3Raw( hertz ) >= 0 );

	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_motorJoint );
	base->motorJoint.linearHertz = hertz;
}

b3f b3MotorJoint_GetLinearHertz( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_motorJoint );
	return base->motorJoint.linearHertz;
}

void b3MotorJoint_SetLinearDampingRatio( b3JointId jointId, b3f dampingRatio )
{
	B3_ASSERT( b3IsValidFloat( dampingRatio ) && b3Raw( dampingRatio ) >= 0 );

	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_motorJoint );
	base->motorJoint.linearDampingRatio = dampingRatio;
}

b3f b3MotorJoint_GetLinearDampingRatio( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_motorJoint );
	return base->motorJoint.linearDampingRatio;
}

void b3MotorJoint_SetAngularHertz( b3JointId jointId, b3f hertz )
{
	B3_ASSERT( b3IsValidFloat( hertz ) && b3Raw( hertz ) >= 0 );

	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_motorJoint );
	base->motorJoint.angularHertz = hertz;
}

b3f b3MotorJoint_GetAngularHertz( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_motorJoint );
	return base->motorJoint.angularHertz;
}

void b3MotorJoint_SetAngularDampingRatio( b3JointId jointId, b3f dampingRatio )
{
	B3_ASSERT( b3IsValidFloat( dampingRatio ) && b3Raw( dampingRatio ) >= 0 );

	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_motorJoint );
	base->motorJoint.angularDampingRatio = dampingRatio;
}

b3f b3MotorJoint_GetAngularDampingRatio( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_motorJoint );
	return base->motorJoint.angularDampingRatio;
}

/// The force the joint is applying, in world space: both linear branches.
b3Vec3 b3GetMotorJointForce( b3World* world, b3JointSim* base )
{
	b3MotorJoint* joint = &base->motorJoint;
	b3Imp3 p = b3AddImp3( joint->linearVelocityImpulse, joint->linearSpringImpulse );
	b3f inv_h = world->inv_h;
	return b3MakeVec3( b3MulImpFToF( p.x, inv_h ), b3MulImpFToF( p.y, inv_h ), b3MulImpFToF( p.z, inv_h ) );
}

/// The torque the joint is applying, in world space: both angular branches.
b3Vec3 b3GetMotorJointTorque( b3World* world, b3JointSim* base )
{
	b3MotorJoint* joint = &base->motorJoint;
	b3Imp3 t = b3AddImp3( joint->angularVelocityImpulse, joint->angularSpringImpulse );
	b3f inv_h = world->inv_h;
	return b3MakeVec3( b3MulImpFToF( t.x, inv_h ), b3MulImpFToF( t.y, inv_h ), b3MulImpFToF( t.z, inv_h ) );
}

// =========================================================================
// Solver
// =========================================================================

void b3PrepareMotorJoint( b3JointSim* base, b3StepContext* context )
{
	B3_ASSERT( base->type == b3_motorJoint );

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

	b3MotorJoint* joint = &base->motorJoint;

	joint->indexA = bodyA->setIndex == b3_awakeSet ? localIndexA : B3_NULL_INDEX;
	joint->indexB = bodyB->setIndex == b3_awakeSet ? localIndexB : B3_NULL_INDEX;

	// Joint frames in world space, with the position relative to the centre of
	// mass -- as in every other joint, and for the same round-off reason.
	joint->frameA.q = b3MulQuat( bodySimA->transform.q, base->localFrameA.q );
	joint->frameA.p = b3RotateVector( bodySimA->transform.q, b3Sub( base->localFrameA.p, bodySimA->localCenter ) );
	joint->frameB.q = b3MulQuat( bodySimB->transform.q, base->localFrameB.q );
	joint->frameB.p = b3RotateVector( bodySimB->transform.q, b3Sub( base->localFrameB.p, bodySimB->localCenter ) );

	// The initial centre delta. Incremental position updates are relative to it.
	joint->deltaCenter = b3Sub( bodySimB->center, bodySimA->center );

	joint->angularMass = b3InvertRotationMass( base->invIA, base->invIB );
	base->fixedRotation = b3FixedRotationFromMass( joint->angularMass );

	// Unconditional, unlike the weld's: this joint has no rigid mode to fall
	// back to, so a zero hertz means "no spring" and b3MakeSoft's all-zero
	// return is exactly right. The branches are gated on the bound instead.
	joint->linearSpring = b3MakeSoft( joint->linearHertz, joint->linearDampingRatio, context->h );
	joint->angularSpring = b3MakeSoft( joint->angularHertz, joint->angularDampingRatio, context->h );

	if ( context->enableWarmStarting == false )
	{
		joint->linearVelocityImpulse = b3Imp3_zero;
		joint->angularVelocityImpulse = b3Imp3_zero;
		joint->linearSpringImpulse = b3Imp3_zero;
		joint->angularSpringImpulse = b3Imp3_zero;
	}
}

/// @note ITCM group B3_ITCM_MOTOR -- see nea_config.h.
void B3_ITCM_IF( B3_ITCM_MOTOR, b3WarmStartMotorJoint )( b3JointSim* base, b3StepContext* context )
{
	B3_ASSERT( base->type == b3_motorJoint );

	b3iw mA = base->invMassA;
	b3iw mB = base->invMassB;
	b3MatrixW iA = base->invIA;
	b3MatrixW iB = base->invIB;

	b3BodyState dummyState = b3_identityBodyState;

	b3MotorJoint* joint = &base->motorJoint;
	b3BodyState* stateA = joint->indexA == B3_NULL_INDEX ? &dummyState : context->states + joint->indexA;
	b3BodyState* stateB = joint->indexB == B3_NULL_INDEX ? &dummyState : context->states + joint->indexB;

	b3Vec3 rA = b3RotateVector( stateA->deltaRotation, joint->frameA.p );
	b3Vec3 rB = b3RotateVector( stateB->deltaRotation, joint->frameB.p );

	// The two halves of each pair are applied along the same directions, so
	// they are summed before being applied rather than applied twice.
	b3Imp3 P = b3AddImp3( joint->linearVelocityImpulse, joint->linearSpringImpulse );
	b3Imp3 angular = b3AddImp3( joint->angularVelocityImpulse, joint->angularSpringImpulse );

	b3Vec3 vA = b3Sub( stateA->linearVelocity, b3MulImpW3( P, mA ) );
	b3Vec3 wA = b3Sub( stateA->angularVelocity, b3MulMWImp( iA, b3AddImp3( b3CrossVImp( rA, P ), angular ) ) );

	b3Vec3 vB = b3Add( stateB->linearVelocity, b3MulImpW3( P, mB ) );
	b3Vec3 wB = b3Add( stateB->angularVelocity, b3MulMWImp( iB, b3AddImp3( b3CrossVImp( rB, P ), angular ) ) );

	// Upstream writes both states unconditionally here while every other joint
	// guards on b3_dynamicFlag ([motor_joint.c:263-266](helpSrc/box3d-main/src/motor_joint.c#L263-L266)).
	// The port guards, matching its own solve below and the other four joints:
	// writing a velocity onto a static body's dummy state is harmless only
	// because the dummy is a local, and relying on that is relying on an
	// accident.
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

/// @note ITCM group B3_ITCM_MOTOR -- see nea_config.h.
void B3_ITCM_IF( B3_ITCM_MOTOR, b3SolveMotorJoint )( b3JointSim* base, b3StepContext* context, bool useBias )
{
	B3_ASSERT( base->type == b3_motorJoint );

	// Every branch here carries its own bias, from its own spring, on every
	// iteration -- there is no rigid constraint to relax on the unbiased pass.
	B3_UNUSED( useBias );

	b3iw mA = base->invMassA;
	b3iw mB = base->invMassB;
	b3MatrixW iA = base->invIA;
	b3MatrixW iB = base->invIB;
	b3t h = context->h;

	b3BodyState dummyState = b3_identityBodyState;

	b3MotorJoint* joint = &base->motorJoint;
	b3BodyState* stateA = joint->indexA == B3_NULL_INDEX ? &dummyState : context->states + joint->indexA;
	b3BodyState* stateB = joint->indexB == B3_NULL_INDEX ? &dummyState : context->states + joint->indexB;

	b3Vec3 vA = stateA->linearVelocity;
	b3Vec3 wA = stateA->angularVelocity;
	b3Vec3 vB = stateB->linearVelocity;
	b3Vec3 wB = stateB->angularVelocity;

	// -----------------------------------------------------------------
	// Angular spring, toward the target orientation
	// -----------------------------------------------------------------
	if ( base->fixedRotation == false && b3Raw( joint->maxSpringTorque ) > 0 && b3Raw( joint->angularHertz ) > 0 )
	{
		b3Quat quatA = b3MulQuat( stateA->deltaRotation, joint->frameA.q );
		b3Quat quatB = b3MulQuat( stateB->deltaRotation, joint->frameB.q );

		if ( b3Raw( b3DotQuat( quatA, quatB ) ) < 0 )
		{
			quatB = b3NegateQuat( quatB );
		}

		b3Quat relQ = b3InvMulQuat( quatA, quatB );

		b3Vec3 deltaRotation = b3DeltaQuatToRotation( relQ, b3Quat_identity );
		b3Vec3 c = b3Neg( b3RotateVector( quatA, deltaRotation ) );

		b3Softness soft = joint->angularSpring;
		b3Vec3 bias = b3MulSV( b3MulFC( soft.biasRate, soft.massScale ), c );
		b3Vec3 cdot = b3Sub( wB, wA );
		b3Vec3 driving = b3Add( b3MulSV( b3CToF( soft.massScale ), cdot ), bias );

		b3Imp3 oldImpulse = joint->angularSpringImpulse;
		b3Imp3 sol = b3MulMVToImpSat( joint->angularMass, driving );
		b3Imp3 lambda = b3SubImp3( b3NegImp3( sol ), b3MulCImp3( soft.impulseScale, oldImpulse ) );

		b3imp maxImpulse = b3MulFTToImp( joint->maxSpringTorque, h );
		joint->angularSpringImpulse = b3ClampImp3( b3AddImp3( oldImpulse, lambda ), maxImpulse );
		lambda = b3SubImp3( joint->angularSpringImpulse, oldImpulse );

		wA = b3Sub( wA, b3MulMWImp( iA, lambda ) );
		wB = b3Add( wB, b3MulMWImp( iB, lambda ) );
	}

	// -----------------------------------------------------------------
	// Angular velocity drive
	// -----------------------------------------------------------------
	if ( base->fixedRotation == false && b3Raw( joint->maxVelocityTorque ) > 0 )
	{
		b3Vec3 cdot = b3Sub( b3Sub( wB, wA ), joint->angularVelocity );

		b3Imp3 oldImpulse = joint->angularVelocityImpulse;
		b3Imp3 lambda = b3NegImp3( b3MulMVToImpSat( joint->angularMass, cdot ) );

		b3imp maxImpulse = b3MulFTToImp( joint->maxVelocityTorque, h );
		joint->angularVelocityImpulse = b3ClampImp3( b3AddImp3( oldImpulse, lambda ), maxImpulse );
		lambda = b3SubImp3( joint->angularVelocityImpulse, oldImpulse );

		wA = b3Sub( wA, b3MulMWImp( iA, lambda ) );
		wB = b3Add( wB, b3MulMWImp( iB, lambda ) );
	}

	// The lever arms, and the linear effective mass built from them. Both
	// linear branches below share these unchanged -- see the file header on why
	// upstream's second build is redundant and what dropping it saves.
	b3Vec3 rA = b3RotateVector( stateA->deltaRotation, joint->frameA.p );
	b3Vec3 rB = b3RotateVector( stateB->deltaRotation, joint->frameB.p );

	const bool linearSpringActive = b3Raw( joint->maxSpringForce ) > 0 && b3Raw( joint->linearHertz ) > 0;
	const bool linearVelocityActive = b3Raw( joint->maxVelocityForce ) > 0;

	b3Matrix3 invK = b3Mat3_zero;
	if ( linearSpringActive || linearVelocityActive )
	{
		invK = b3InvertPointMass( mA, iA, rA, mB, iB, rB );
	}

	// -----------------------------------------------------------------
	// Linear spring, toward the target position
	// -----------------------------------------------------------------
	if ( linearSpringActive )
	{
		b3Vec3 dp = b3W3ToVec3( b3SubW3( stateB->deltaPosition, stateA->deltaPosition ) );
		b3Vec3 c = b3Add( b3Add( dp, b3Sub( rB, rA ) ), joint->deltaCenter );

		b3Softness soft = joint->linearSpring;
		b3Vec3 bias = b3MulSV( b3MulFC( soft.biasRate, soft.massScale ), c );
		b3Vec3 cdot = b3Sub( b3Sub( b3Add( vB, b3Cross( wB, rB ) ), vA ), b3Cross( wA, rA ) );
		b3Vec3 driving = b3Add( b3MulSV( b3CToF( soft.massScale ), cdot ), bias );

		b3Imp3 oldImpulse = joint->linearSpringImpulse;
		b3Imp3 sol = b3MulMVToImpSat( invK, driving );
		b3Imp3 lambda = b3SubImp3( b3NegImp3( sol ), b3MulCImp3( soft.impulseScale, oldImpulse ) );

		b3imp maxImpulse = b3MulFTToImp( joint->maxSpringForce, h );
		joint->linearSpringImpulse = b3ClampImp3( b3AddImp3( oldImpulse, lambda ), maxImpulse );
		lambda = b3SubImp3( joint->linearSpringImpulse, oldImpulse );

		vA = b3Sub( vA, b3MulImpW3( lambda, mA ) );
		wA = b3Sub( wA, b3MulMWImp( iA, b3CrossVImp( rA, lambda ) ) );
		vB = b3Add( vB, b3MulImpW3( lambda, mB ) );
		wB = b3Add( wB, b3MulMWImp( iB, b3CrossVImp( rB, lambda ) ) );
	}

	// -----------------------------------------------------------------
	// Linear velocity drive
	// -----------------------------------------------------------------
	if ( linearVelocityActive )
	{
		b3Vec3 cdot = b3Sub( b3Sub( b3Add( vB, b3Cross( wB, rB ) ), vA ), b3Cross( wA, rA ) );
		cdot = b3Sub( cdot, joint->linearVelocity );

		b3Imp3 oldImpulse = joint->linearVelocityImpulse;
		b3Imp3 lambda = b3NegImp3( b3MulMVToImpSat( invK, cdot ) );

		b3imp maxImpulse = b3MulFTToImp( joint->maxVelocityForce, h );
		joint->linearVelocityImpulse = b3ClampImp3( b3AddImp3( oldImpulse, lambda ), maxImpulse );
		lambda = b3SubImp3( joint->linearVelocityImpulse, oldImpulse );

		vA = b3Sub( vA, b3MulImpW3( lambda, mA ) );
		wA = b3Sub( wA, b3MulMWImp( iA, b3CrossVImp( rA, lambda ) ) );
		vB = b3Add( vB, b3MulImpW3( lambda, mB ) );
		wB = b3Add( wB, b3MulMWImp( iB, b3CrossVImp( rB, lambda ) ) );
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
