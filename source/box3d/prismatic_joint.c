// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Erin Catto        (original Box3D)
// Copyright (c) 2026 Warioware64       (Nitro Engine Advanced fixed-point port)
//
// This file is part of Nitro Engine Advanced

/// @file   prismatic_joint.c
/// @brief  The prismatic joint: a slider, and the first state-dependent mass.
///
/// @section shape What a slider actually is
///
/// Five constraints solved in sequence, leaving one translational degree free:
///
///   1. a **scalar axial** spring along the slide axis;
///   2. a **scalar axial** motor, bounded by a force;
///   3. two **one-sided axial limits**, lower then upper, each with a
///      speculative band;
///   4. a **3-vector orientation lock** holding the two frames aligned -- the
///      weld joint's block with an identity target, because a slider does not
///      turn;
///   5. a **2x2 point-to-line** constraint holding the anchor on the rail,
///      which removes the two linear degrees the slide does not use.
///
/// They couple through the shared velocity state, so upstream's order is kept
/// exactly. Reversing two does not diverge -- it converges more slowly and
/// reads as a slightly loose slider, which no single-frame test catches.
///
/// **The slide axis is frame A's x**, not the revolute's z. Upstream's choice,
/// and its own header comment still says z from an older convention while the
/// code uses x; the code is what is ported.
///
/// @section scales What is new here, and what is not
///
/// Nearly every convention is Stage 2's, unchanged: the effective mass
/// accumulated wide and reciprocated once, `massScale` folded into each term at
/// Q30, the clamp landing on the accumulator with the delta recomputed. Three
/// things this file adds or removes:
///
///   - **The effective masses depend on the joint's state.** Every joint before
///     this one builds its mass in prepare from geometry fixed there. A
///     prismatic's lever arms are `cross( rA + d, axis )` and
///     `cross( rB, axis )`, and `d` is the slide translation -- so both masses
///     are rebuilt in the *solve*, which is upstream's "must be fresh to avoid
///     divergence when the joint is stressed". b3LeverInertiaSumWide gives the
///     scalar one and b3InvertPointLineMass the 2x2; neither existed before
///     Stage 6, because no earlier constraint had two different lever arms.
///   - **No angle unit.** The revolute's public API is brads and its constraint
///     is radians, so b3BradToRadF converts the difference against a limit. A
///     translation limit is a length in the same Q12 as every position in the
///     port, so there is nothing to convert and no counterpart to look for.
///   - **No cached axialMass**, for the first reason. See b3PrismaticJoint.
///
/// @section world Why there is no `double` here, and what replaces it
///
/// Upstream differences the two body centres in `double` (prismatic_joint.c:252)
/// so that GetSpeed stays exact far from the origin. Float needs that: two
/// positions near 10,000 m have a relative quantum of about a millimetre each,
/// and differencing them cancels the shared magnitude away and keeps the noise.
///
/// Fixed point has the opposite failure mode, so it needs the opposite remedy.
/// b3Pos is b3Vec3 at Q12, whose quantum is an **absolute** 1/4096 everywhere --
/// there is no exponent to lose, so the difference of two positions is exact
/// wherever they sit, and `deltaCenter`, the translation and the speed are as
/// accurate at the edge of the world as at its centre. Nothing widens.
///
/// What is bounded instead is the world's absolute extent. Q12 tops out at
/// 524,288 and B3_HUGE (constants.h) documents 2000 units as the practical
/// size. A rail placed anywhere inside that behaves identically; a scene that
/// wants to be at 10,000 m must move its origin rather than widen the type.
/// That is the port's trade for not needing upstream's double: uniform
/// precision, finite range.
///
/// @section upstream Two upstream bugs, both fixed here
///
/// Both are in the debug reaction readouts, and both are invisible to run_pair
/// **by construction** -- the harness compares the reaction force by magnitude,
/// and a permutation and a spurious rotation each preserve magnitude. Only a
/// closed form on the *direction* can see them, which is what
/// test_prismatic_joint_reaction_direction in test_world.c exists for.
///
///   1. **The force permuted its components.** Prepare sets the axes from
///      matrixA's columns in the order (x, y, z) = (jointAxis, perpY, perpZ)
///      and warm start assembles the impulse in that order, so in frame A's
///      basis it is `(axial, perp.x, perp.y)`. Upstream's b3GetPrismaticJointForce
///      writes `(perp.x, perp.y, axial)` -- a leftover of the older z-axis
///      convention its header still describes. The port writes the order the
///      solver actually uses.
///   2. **The torque was rotated twice.** `angularImpulse` is accumulated in
///      world space -- `cdot` is `wB - wA`, `rotationMass` is world-space, and
///      it is applied straight to the angular velocities -- but upstream then
///      rotates it by `localFrameA.q` and again by `transformA.q`.
///      b3GetWeldJointTorque returns the identical quantity unrotated and says
///      why. The port does the same.
///
/// @section absent What is not here
///
/// b3DrawPrismaticJoint and every B3_REC hook, as in every other joint file.

#include "joint.h"

#include "body.h"
#include "core.h"
#include "physics_world.h"
#include "solver.h"
#include "solver_set.h"

#include "box3d/box3d.h"

/// A quarter, as a Q30 coefficient. The limits' speculative band is this much
/// of the range, and a quarter is an exact shift rather than a rounded literal.
#define B3_QUARTER b3cFromFrac( 1, 4 )

// =========================================================================
// Accessors
// =========================================================================

void b3PrismaticJoint_EnableSpring( b3JointId jointId, bool enableSpring )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_prismaticJoint );
	if ( enableSpring != base->prismaticJoint.enableSpring )
	{
		base->prismaticJoint.enableSpring = enableSpring;
		base->prismaticJoint.springImpulse = b3imp_zero;
	}
}

bool b3PrismaticJoint_IsSpringEnabled( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_prismaticJoint );
	return base->prismaticJoint.enableSpring;
}

void b3PrismaticJoint_SetSpringHertz( b3JointId jointId, b3f hertz )
{
	B3_ASSERT( b3IsValidFloat( hertz ) && b3Raw( hertz ) >= 0 );

	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_prismaticJoint );
	base->prismaticJoint.hertz = hertz;
}

b3f b3PrismaticJoint_GetSpringHertz( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_prismaticJoint );
	return base->prismaticJoint.hertz;
}

void b3PrismaticJoint_SetSpringDampingRatio( b3JointId jointId, b3f dampingRatio )
{
	B3_ASSERT( b3IsValidFloat( dampingRatio ) && b3Raw( dampingRatio ) >= 0 );

	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_prismaticJoint );
	base->prismaticJoint.dampingRatio = dampingRatio;
}

b3f b3PrismaticJoint_GetSpringDampingRatio( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_prismaticJoint );
	return base->prismaticJoint.dampingRatio;
}

void b3PrismaticJoint_SetTargetTranslation( b3JointId jointId, b3f targetTranslation )
{
	B3_ASSERT( b3IsValidFloat( targetTranslation ) );

	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_prismaticJoint );
	base->prismaticJoint.targetTranslation = targetTranslation;
}

b3f b3PrismaticJoint_GetTargetTranslation( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_prismaticJoint );
	return base->prismaticJoint.targetTranslation;
}

void b3PrismaticJoint_EnableLimit( b3JointId jointId, bool enableLimit )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_prismaticJoint );
	if ( enableLimit != base->prismaticJoint.enableLimit )
	{
		base->prismaticJoint.enableLimit = enableLimit;
		base->prismaticJoint.lowerImpulse = b3imp_zero;
		base->prismaticJoint.upperImpulse = b3imp_zero;
	}
}

bool b3PrismaticJoint_IsLimitEnabled( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_prismaticJoint );
	return base->prismaticJoint.enableLimit;
}

void b3PrismaticJoint_SetLimits( b3JointId jointId, b3f lower, b3f upper )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_prismaticJoint );
	b3PrismaticJoint* joint = &base->prismaticJoint;

	// Sorted rather than asserted, as the revolute's angles and the distance
	// joint's length range are: a caller who passes them the wrong way round
	// gets the range they meant.
	joint->lowerTranslation = b3Raw( lower ) < b3Raw( upper ) ? lower : upper;
	joint->upperTranslation = b3Raw( lower ) < b3Raw( upper ) ? upper : lower;

	joint->lowerImpulse = b3imp_zero;
	joint->upperImpulse = b3imp_zero;
}

b3f b3PrismaticJoint_GetLowerLimit( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_prismaticJoint );
	return base->prismaticJoint.lowerTranslation;
}

b3f b3PrismaticJoint_GetUpperLimit( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_prismaticJoint );
	return base->prismaticJoint.upperTranslation;
}

void b3PrismaticJoint_EnableMotor( b3JointId jointId, bool enableMotor )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_prismaticJoint );
	if ( enableMotor != base->prismaticJoint.enableMotor )
	{
		base->prismaticJoint.enableMotor = enableMotor;
		base->prismaticJoint.motorImpulse = b3imp_zero;
	}
}

bool b3PrismaticJoint_IsMotorEnabled( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_prismaticJoint );
	return base->prismaticJoint.enableMotor;
}

void b3PrismaticJoint_SetMotorSpeed( b3JointId jointId, b3f motorSpeed )
{
	B3_ASSERT( b3IsValidFloat( motorSpeed ) );

	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_prismaticJoint );
	base->prismaticJoint.motorSpeed = motorSpeed;
}

b3f b3PrismaticJoint_GetMotorSpeed( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_prismaticJoint );
	return base->prismaticJoint.motorSpeed;
}

void b3PrismaticJoint_SetMaxMotorForce( b3JointId jointId, b3f force )
{
	B3_ASSERT( b3IsValidFloat( force ) );

	// Clamped rather than asserted, matching the motor joint's four bounds and
	// this joint's own creator. The revolute asserts instead, and the
	// difference is not arbitrary: a *bound* has one obvious right answer for a
	// negative input, which is no budget, and the motor branch reads the value
	// to decide how much it may spend. An unclamped negative would hand the
	// drive a negative allowance rather than none.
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_prismaticJoint );
	base->prismaticJoint.maxMotorForce = b3MaxF( b3f_zero, force );
}

b3f b3PrismaticJoint_GetMaxMotorForce( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_prismaticJoint );
	return base->prismaticJoint.maxMotorForce;
}

b3f b3PrismaticJoint_GetMotorForce( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_prismaticJoint );
	return b3MulImpFToF( base->prismaticJoint.motorImpulse, world->inv_h );
}

/// The slide axis in world space, and the anchor separation along it.
///
/// Shared by GetTranslation and GetSpeed, which differ only in what they do
/// with the two results.
static void b3PrismaticFrame( b3World* world, b3JointSim* base, b3Vec3* outAxis, b3Vec3* outD, b3Vec3* outRA,
							  b3Vec3* outRB )
{
	b3WorldTransform transformA = b3GetBodyTransform( world, base->bodyIdA );
	b3WorldTransform transformB = b3GetBodyTransform( world, base->bodyIdB );

	b3Quat qA = b3MulQuat( transformA.q, base->localFrameA.q );

	b3Vec3 rA = b3RotateVector( transformA.q, base->localFrameA.p );
	b3Vec3 rB = b3RotateVector( transformB.q, base->localFrameB.p );

	*outAxis = b3RotateVector( qA, b3Vec3_axisX );
	*outD = b3Add( b3Sub( transformB.p, transformA.p ), b3Sub( rB, rA ) );
	*outRA = rA;
	*outRB = rB;
}

b3f b3PrismaticJoint_GetTranslation( b3JointId jointId )
{
	b3World* world = b3GetUnlockedWorld( jointId.world0 );
	if ( world == NULL )
	{
		return b3f_zero;
	}

	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_prismaticJoint );

	b3Vec3 axis, d, rA, rB;
	b3PrismaticFrame( world, base, &axis, &d, &rA, &rB );
	return b3Dot( d, axis );
}

b3f b3PrismaticJoint_GetSpeed( b3JointId jointId )
{
	b3World* world = b3GetUnlockedWorld( jointId.world0 );
	if ( world == NULL )
	{
		return b3f_zero;
	}

	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_prismaticJoint );

	b3Vec3 axis, d, rA, rB;
	b3PrismaticFrame( world, base, &axis, &d, &rA, &rB );

	// A sleeping or static body has no b3BodyState, and zero is its velocity.
	b3Body* bodyA = b3Array_Get( world->bodies, base->bodyIdA );
	b3Body* bodyB = b3Array_Get( world->bodies, base->bodyIdB );
	b3BodyState* stateA = b3GetBodyState( world, bodyA );
	b3BodyState* stateB = b3GetBodyState( world, bodyB );

	b3Vec3 vA = stateA ? stateA->linearVelocity : b3Vec3_zero;
	b3Vec3 vB = stateB ? stateB->linearVelocity : b3Vec3_zero;
	b3Vec3 wA = stateA ? stateA->angularVelocity : b3Vec3_zero;
	b3Vec3 wB = stateB ? stateB->angularVelocity : b3Vec3_zero;

	b3Vec3 vRel = b3Sub( b3Add( vB, b3Cross( wB, rB ) ), b3Add( vA, b3Cross( wA, rA ) ) );

	// The axis is fixed in body A, so it moves when A turns -- hence the first
	// term, which is what upstream's comment means by "account for its
	// rotation". Without it a slider on a rotating base reads the wrong speed.
	return b3AddF( b3Dot( d, b3Cross( wA, axis ) ), b3Dot( axis, vRel ) );
}

/// The force the joint is applying, in world space.
///
/// **The component order is the solver's, not upstream's.** In frame A's basis
/// the axes are (x, y, z) = (jointAxis, perpAxisY, perpAxisZ) -- that is what
/// prepare stores and what warm start assembles the impulse along -- so the
/// axial accumulators belong in x and the two perpendicular ones in y and z.
/// Upstream writes (perp.x, perp.y, axial), a leftover of the older convention
/// its own file header still describes. See @section upstream.
b3Vec3 b3GetPrismaticJointForce( b3World* world, b3JointSim* base )
{
	b3PrismaticJoint* joint = &base->prismaticJoint;

	b3imp axial = b3AddImp( b3AddImp( joint->motorImpulse, joint->springImpulse ),
							b3SubImp( joint->lowerImpulse, joint->upperImpulse ) );

	// Straight into world space along the axes the solve used, rather than
	// building a joint-space vector and rotating it twice. Same answer, and it
	// cannot pick up a permutation on the way.
	b3Imp3 p = b3AddImp3( b3MulImpV( axial, joint->jointAxis ),
						  b3AddImp3( b3MulImpV( joint->perpImpulse.x, joint->perpAxisY ),
									 b3MulImpV( joint->perpImpulse.y, joint->perpAxisZ ) ) );

	b3f inv_h = world->inv_h;
	return b3MakeVec3( b3MulImpFToF( p.x, inv_h ), b3MulImpFToF( p.y, inv_h ), b3MulImpFToF( p.z, inv_h ) );
}

/// The torque the orientation lock is applying, in world space.
///
/// Returned **unrotated**, as b3GetWeldJointTorque is and for the same reason:
/// `angularImpulse` is accumulated in world space already. Upstream rotates it
/// by localFrameA.q and again by transformA.q, which is a bug in a debug-only
/// readout. See @section upstream.
b3Vec3 b3GetPrismaticJointTorque( b3World* world, b3JointSim* base )
{
	b3Imp3 t = base->prismaticJoint.angularImpulse;
	b3f inv_h = world->inv_h;
	return b3MakeVec3( b3MulImpFToF( t.x, inv_h ), b3MulImpFToF( t.y, inv_h ), b3MulImpFToF( t.z, inv_h ) );
}

// =========================================================================
// Solver
// =========================================================================

void b3PreparePrismaticJoint( b3JointSim* base, b3StepContext* context )
{
	B3_ASSERT( base->type == b3_prismaticJoint );

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

	b3PrismaticJoint* joint = &base->prismaticJoint;

	// The sum `invIA + invIB` is never formed; b3InvertRotationMass accumulates
	// it wide and scales the whole matrix before inverting. Sixth caller of
	// b3FixedRotationFromMass, which reads a singular rotation mass as "these
	// two bodies have no rotational freedom between them".
	joint->rotationMass = b3InvertRotationMass( base->invIA, base->invIB );
	base->fixedRotation = b3FixedRotationFromMass( joint->rotationMass );

	joint->indexA = bodyA->setIndex == b3_awakeSet ? localIndexA : B3_NULL_INDEX;
	joint->indexB = bodyB->setIndex == b3_awakeSet ? localIndexB : B3_NULL_INDEX;

	// Joint frames in world space, with the position relative to the centre of
	// mass -- one rotation of (localFrame.p - localCenter) rather than a
	// transform followed by a subtraction, which is where the bits go.
	joint->frameA.q = b3MulQuat( bodySimA->transform.q, base->localFrameA.q );
	joint->frameA.p = b3RotateVector( bodySimA->transform.q, b3Sub( base->localFrameA.p, bodySimA->localCenter ) );
	joint->frameB.q = b3MulQuat( bodySimB->transform.q, base->localFrameB.q );
	joint->frameB.p = b3RotateVector( bodySimB->transform.q, b3Sub( base->localFrameB.p, bodySimB->localCenter ) );

	joint->deltaCenter = b3Sub( bodySimB->center, bodySimA->center );

	// The three axes, rotated one at a time rather than taken as the columns of
	// b3MakeMatrixFromQuat. That routine returns Q12, and a Q12 rotation matrix
	// is not exactly orthonormal -- math_fixed.c says so where it explains why
	// b3RotateInertiaW cannot use it. Rotating each basis vector through the
	// quaternion keeps all three unit to the quaternion's own precision.
	//
	// Q12 b3Vec3 rather than Q30 b3Dir3, for revolute_joint.c's reason: each is
	// used as both the mass direction and the impulse direction, so the two Q12
	// roundings are the same rounding and largely cancel.
	joint->jointAxis = b3RotateVector( joint->frameA.q, b3Vec3_axisX );
	joint->perpAxisY = b3RotateVector( joint->frameA.q, b3Vec3_axisY );
	joint->perpAxisZ = b3RotateVector( joint->frameA.q, b3Vec3_axisZ );

	joint->springSoftness = b3MakeSoft( joint->hertz, joint->dampingRatio, context->h );

	if ( context->enableWarmStarting == false )
	{
		joint->perpImpulse.x = b3imp_zero;
		joint->perpImpulse.y = b3imp_zero;
		joint->angularImpulse = b3Imp3_zero;
		joint->springImpulse = b3imp_zero;
		joint->motorImpulse = b3imp_zero;
		joint->lowerImpulse = b3imp_zero;
		joint->upperImpulse = b3imp_zero;
	}
}

/// @note ITCM group B3_ITCM_PRISMATIC -- see nea_config.h.
void B3_ITCM_IF( B3_ITCM_PRISMATIC_WARM, b3WarmStartPrismaticJoint )( b3JointSim* base, b3StepContext* context )
{
	B3_ASSERT( base->type == b3_prismaticJoint );

	b3iw mA = base->invMassA;
	b3iw mB = base->invMassB;
	b3MatrixW iA = base->invIA;
	b3MatrixW iB = base->invIB;

	b3BodyState dummyState = b3_identityBodyState;

	b3PrismaticJoint* joint = &base->prismaticJoint;
	b3BodyState* stateA = joint->indexA == B3_NULL_INDEX ? &dummyState : context->states + joint->indexA;
	b3BodyState* stateB = joint->indexB == B3_NULL_INDEX ? &dummyState : context->states + joint->indexB;

	b3Vec3 rA = b3RotateVector( stateA->deltaRotation, joint->frameA.p );
	b3Vec3 rB = b3RotateVector( stateB->deltaRotation, joint->frameB.p );

	// deltaPosition is Q24, so the difference is taken there and narrowed once.
	b3Vec3 dp = b3W3ToVec3( b3SubW3( stateB->deltaPosition, stateA->deltaPosition ) );
	b3Vec3 d = b3Add( b3Add( dp, joint->deltaCenter ), b3Sub( rB, rA ) );

	b3Vec3 axis = b3RotateVector( stateA->deltaRotation, joint->jointAxis );
	b3Vec3 perpY = b3RotateVector( stateA->deltaRotation, joint->perpAxisY );
	b3Vec3 perpZ = b3RotateVector( stateA->deltaRotation, joint->perpAxisZ );

	// A's lever arms are measured from `rA + d` and B's from `rB`. That
	// asymmetry is the joint: the anchor slides along the rail, so how far it
	// has gone is part of A's arm.
	b3Vec3 rAd = b3Add( rA, d );
	b3Vec3 sAx = b3Cross( rAd, axis );
	b3Vec3 sBx = b3Cross( rB, axis );
	b3Vec3 sAy = b3Cross( rAd, perpY );
	b3Vec3 sBy = b3Cross( rB, perpY );
	b3Vec3 sAz = b3Cross( rAd, perpZ );
	b3Vec3 sBz = b3Cross( rB, perpZ );

	b3imp axial = b3AddImp( b3AddImp( joint->springImpulse, joint->motorImpulse ),
							b3SubImp( joint->lowerImpulse, joint->upperImpulse ) );
	b3Imp2 perp = joint->perpImpulse;

	b3Imp3 P = b3AddImp3( b3MulImpV( axial, axis ),
						  b3AddImp3( b3MulImpV( perp.x, perpY ), b3MulImpV( perp.y, perpZ ) ) );

	b3Imp3 LA = b3AddImp3( b3AddImp3( b3MulImpV( axial, sAx ), b3MulImpV( perp.x, sAy ) ),
						   b3AddImp3( b3MulImpV( perp.y, sAz ), joint->angularImpulse ) );
	b3Imp3 LB = b3AddImp3( b3AddImp3( b3MulImpV( axial, sBx ), b3MulImpV( perp.x, sBy ) ),
						   b3AddImp3( b3MulImpV( perp.y, sBz ), joint->angularImpulse ) );

	b3Vec3 vA = b3Sub( stateA->linearVelocity, b3MulImpW3( P, mA ) );
	b3Vec3 wA = b3Sub( stateA->angularVelocity, b3MulMWImp( iA, LA ) );
	b3Vec3 vB = b3Add( stateB->linearVelocity, b3MulImpW3( P, mB ) );
	b3Vec3 wB = b3Add( stateB->angularVelocity, b3MulMWImp( iB, LB ) );

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

/// @note ITCM group B3_ITCM_PRISMATIC -- see nea_config.h.
void B3_ITCM_IF( B3_ITCM_PRISMATIC_SOLVE, b3SolvePrismaticJoint )( b3JointSim* base, b3StepContext* context, bool useBias )
{
	B3_ASSERT( base->type == b3_prismaticJoint );

	b3iw mA = base->invMassA;
	b3iw mB = base->invMassB;
	b3MatrixW iA = base->invIA;
	b3MatrixW iB = base->invIB;

	b3BodyState dummyState = b3_identityBodyState;

	b3PrismaticJoint* joint = &base->prismaticJoint;
	b3BodyState* stateA = joint->indexA == B3_NULL_INDEX ? &dummyState : context->states + joint->indexA;
	b3BodyState* stateB = joint->indexB == B3_NULL_INDEX ? &dummyState : context->states + joint->indexB;

	b3Vec3 vA = stateA->linearVelocity;
	b3Vec3 wA = stateA->angularVelocity;
	b3Vec3 vB = stateB->linearVelocity;
	b3Vec3 wB = stateB->angularVelocity;

	bool fixedRotation = base->fixedRotation;
	b3t h = context->h;

	b3Vec3 rA = b3RotateVector( stateA->deltaRotation, joint->frameA.p );
	b3Vec3 rB = b3RotateVector( stateB->deltaRotation, joint->frameB.p );

	b3Vec3 dp = b3W3ToVec3( b3SubW3( stateB->deltaPosition, stateA->deltaPosition ) );
	b3Vec3 d = b3Add( b3Add( dp, joint->deltaCenter ), b3Sub( rB, rA ) );
	b3Vec3 rAd = b3Add( rA, d );

	b3Vec3 axis = b3RotateVector( stateA->deltaRotation, joint->jointAxis );
	b3Vec3 sAx = b3Cross( rAd, axis );
	b3Vec3 sBx = b3Cross( rB, axis );

	b3f translation = b3Dot( d, axis );

	// **Rebuilt here rather than cached in prepare**, which is upstream's
	// "the axial effective mass must be fresh to avoid divergence when the joint
	// is stressed". The lever arms above depend on `d`, so this mass is a
	// function of where the slider currently sits -- unlike every joint before
	// it, whose mass prepare could compute once.
	b3f axialMass = b3RcpWide( b3LeverInertiaSumWide( mA, iA, sAx, mB, iB, sBx ) );

	// -----------------------------------------------------------------
	// Axial spring
	// -----------------------------------------------------------------
	if ( joint->enableSpring && fixedRotation == false )
	{
		// A length against a length. No unit conversion, unlike the revolute's
		// b3BradToRadF -- see @section scales.
		b3f c = b3SubF( translation, joint->targetTranslation );

		b3Softness soft = joint->springSoftness;
		b3f bias = b3MulFF( b3MulFC( soft.biasRate, soft.massScale ), c );

		b3Vec3 vRel = b3Sub( b3Sub( b3Add( vB, b3Cross( wB, rB ) ), vA ), b3Cross( wA, rAd ) );
		b3f cdot = b3Dot( vRel, axis );
		b3f driving = b3AddF( b3MulFC( cdot, soft.massScale ), bias );

		b3imp deltaImpulse = b3SubImp( b3NegImp( b3MulFFToImp( axialMass, driving ) ),
									   b3MulImpC( joint->springImpulse, soft.impulseScale ) );
		joint->springImpulse = b3AddImp( joint->springImpulse, deltaImpulse );

		b3Imp3 P = b3MulImpV( deltaImpulse, axis );
		vA = b3Sub( vA, b3MulImpW3( P, mA ) );
		wA = b3Sub( wA, b3MulMWImp( iA, b3MulImpV( deltaImpulse, sAx ) ) );
		vB = b3Add( vB, b3MulImpW3( P, mB ) );
		wB = b3Add( wB, b3MulMWImp( iB, b3MulImpV( deltaImpulse, sBx ) ) );
	}

	// -----------------------------------------------------------------
	// Axial motor
	// -----------------------------------------------------------------
	if ( joint->enableMotor && fixedRotation == false )
	{
		b3Vec3 vRel = b3Sub( b3Sub( b3Add( vB, b3Cross( wB, rB ) ), vA ), b3Cross( wA, rAd ) );
		b3f cdot = b3SubF( b3Dot( vRel, axis ), joint->motorSpeed );

		b3imp oldImpulse = joint->motorImpulse;
		b3imp deltaImpulse = b3NegImp( b3MulFFToImp( axialMass, cdot ) );

		// A force bound becomes an impulse bound over one sub-step, which is
		// what b3MulFTToImp spells -- the same conversion the revolute makes
		// against a torque.
		b3imp maxImpulse = b3MulFTToImp( joint->maxMotorForce, h );
		joint->motorImpulse = b3ClampImp( b3AddImp( oldImpulse, deltaImpulse ), b3NegImp( maxImpulse ), maxImpulse );
		deltaImpulse = b3SubImp( joint->motorImpulse, oldImpulse );

		b3Imp3 P = b3MulImpV( deltaImpulse, axis );
		vA = b3Sub( vA, b3MulImpW3( P, mA ) );
		wA = b3Sub( wA, b3MulMWImp( iA, b3MulImpV( deltaImpulse, sAx ) ) );
		vB = b3Add( vB, b3MulImpW3( P, mB ) );
		wB = b3Add( wB, b3MulMWImp( iB, b3MulImpV( deltaImpulse, sBx ) ) );
	}

	// -----------------------------------------------------------------
	// Axial limits
	// -----------------------------------------------------------------
	if ( joint->enableLimit && fixedRotation == false )
	{
		// A quarter of the range, which is an exact Q30 shift. This is why the
		// limits must not default to +/-B3_HUGE: a 4000-unit range would put
		// this band at 1000 m and the speculative bias past what Q12 holds.
		b3f speculative = b3MulFC( b3SubF( joint->upperTranslation, joint->lowerTranslation ), B3_QUARTER );

		// Lower limit.
		{
			b3f c = b3SubF( translation, joint->lowerTranslation );

			if ( b3Raw( c ) < b3Raw( speculative ) )
			{
				b3f bias = b3f_zero;
				b3c massScale = b3c_one;
				b3c impulseScale = b3c_zero;
				if ( b3Raw( c ) > 0 )
				{
					// Still inside the range, so the constraint may only stop it
					// closing faster than one sub-step of slack.
					bias = b3MulFF( c, context->inv_h );
				}
				else if ( useBias )
				{
					b3Softness cs = base->constraintSoftness;
					bias = b3MulFF( b3MulFC( cs.biasRate, cs.massScale ), c );
					massScale = cs.massScale;
					impulseScale = cs.impulseScale;
				}

				b3Vec3 vRel = b3Sub( b3Sub( b3Add( vB, b3Cross( wB, rB ) ), vA ), b3Cross( wA, rAd ) );
				b3f cdot = b3Dot( vRel, axis );
				b3f driving = b3AddF( b3MulFC( cdot, massScale ), bias );

				b3imp oldImpulse = joint->lowerImpulse;
				b3imp deltaImpulse =
					b3SubImp( b3NegImp( b3MulFFToImp( axialMass, driving ) ), b3MulImpC( oldImpulse, impulseScale ) );

				joint->lowerImpulse = b3MaxImp( b3AddImp( oldImpulse, deltaImpulse ), b3imp_zero );
				deltaImpulse = b3SubImp( joint->lowerImpulse, oldImpulse );

				b3Imp3 P = b3MulImpV( deltaImpulse, axis );
				vA = b3Sub( vA, b3MulImpW3( P, mA ) );
				wA = b3Sub( wA, b3MulMWImp( iA, b3MulImpV( deltaImpulse, sAx ) ) );
				vB = b3Add( vB, b3MulImpW3( P, mB ) );
				wB = b3Add( wB, b3MulMWImp( iB, b3MulImpV( deltaImpulse, sBx ) ) );
			}
			else
			{
				joint->lowerImpulse = b3imp_zero;
			}
		}

		// Upper limit. The same constraint the other way round, which is why the
		// relative velocity is formed reversed and the impulse applied with its
		// sign flipped.
		{
			b3f c = b3SubF( joint->upperTranslation, translation );

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

				b3Vec3 vRel = b3Sub( b3Sub( b3Add( vB, b3Cross( wB, rB ) ), vA ), b3Cross( wA, rAd ) );
				b3f cdot = b3NegF( b3Dot( vRel, axis ) );
				b3f driving = b3AddF( b3MulFC( cdot, massScale ), bias );

				b3imp oldImpulse = joint->upperImpulse;
				b3imp deltaImpulse =
					b3SubImp( b3NegImp( b3MulFFToImp( axialMass, driving ) ), b3MulImpC( oldImpulse, impulseScale ) );

				joint->upperImpulse = b3MaxImp( b3AddImp( oldImpulse, deltaImpulse ), b3imp_zero );
				deltaImpulse = b3SubImp( joint->upperImpulse, oldImpulse );

				b3Imp3 P = b3MulImpV( deltaImpulse, axis );
				vA = b3Add( vA, b3MulImpW3( P, mA ) );
				wA = b3Add( wA, b3MulMWImp( iA, b3MulImpV( deltaImpulse, sAx ) ) );
				vB = b3Sub( vB, b3MulImpW3( P, mB ) );
				wB = b3Sub( wB, b3MulMWImp( iB, b3MulImpV( deltaImpulse, sBx ) ) );
			}
			else
			{
				joint->upperImpulse = b3imp_zero;
			}
		}
	}

	// -----------------------------------------------------------------
	// Orientation lock: the 3-vector that keeps the two frames aligned
	// -----------------------------------------------------------------
	//
	// The weld joint's angular block with an identity target and no spring: a
	// slider does not turn, so there is no `|| hertz > 0` here and the softness
	// is always the joint's own.
	//
	// Note there is **no `b3DotQuat < 0` guard**, unlike the revolute's and the
	// weld's. Upstream's prismatic does not have one, and adding a guard
	// upstream lacks would be a divergence run_pair reports and the port would
	// then have to defend. The asymmetry is upstream's, recorded rather than
	// silently resolved.
	if ( fixedRotation == false )
	{
		b3Vec3 bias = b3Vec3_zero;
		b3c massScale = b3c_one;
		b3c impulseScale = b3c_zero;

		if ( useBias )
		{
			b3Quat quatA = b3MulQuat( stateA->deltaRotation, joint->frameA.q );
			b3Quat quatB = b3MulQuat( stateB->deltaRotation, joint->frameB.q );

			b3Quat relQ = b3InvMulQuat( quatA, quatB );
			b3Vec3 deltaRotation = b3DeltaQuatToRotation( relQ, b3Quat_identity );
			b3Vec3 c = b3Neg( b3RotateVector( quatA, deltaRotation ) );

			b3Softness cs = base->constraintSoftness;
			bias = b3MulSV( b3MulFC( cs.biasRate, cs.massScale ), c );
			massScale = cs.massScale;
			impulseScale = cs.impulseScale;
		}

		b3Vec3 cdot = b3Sub( wB, wA );
		b3Vec3 driving = b3Add( b3MulSV( b3CToF( massScale ), cdot ), bias );

		b3Imp3 sol = b3MulMVToImp( joint->rotationMass, driving );
		b3Imp3 impulse = b3SubImp3( b3NegImp3( sol ), b3MulCImp3( impulseScale, joint->angularImpulse ) );
		joint->angularImpulse = b3AddImp3( joint->angularImpulse, impulse );

		wA = b3Sub( wA, b3MulMWImp( iA, impulse ) );
		wB = b3Add( wB, b3MulMWImp( iB, impulse ) );
	}

	// -----------------------------------------------------------------
	// Point to line: the 2x2 that holds the anchor on the rail
	// -----------------------------------------------------------------
	{
		b3Vec3 perpY = b3RotateVector( stateA->deltaRotation, joint->perpAxisY );
		b3Vec3 perpZ = b3RotateVector( stateA->deltaRotation, joint->perpAxisZ );

		b3f biasY = b3f_zero;
		b3f biasZ = b3f_zero;
		b3c massScale = b3c_one;
		b3c impulseScale = b3c_zero;
		if ( useBias )
		{
			b3Softness cs = base->constraintSoftness;
			b3f rate = b3MulFC( cs.biasRate, cs.massScale );
			biasY = b3MulFF( rate, b3Dot( perpY, d ) );
			biasZ = b3MulFF( rate, b3Dot( perpZ, d ) );
			massScale = cs.massScale;
			impulseScale = cs.impulseScale;
		}

		b3Vec3 sAy = b3Cross( rAd, perpY );
		b3Vec3 sBy = b3Cross( rB, perpY );
		b3Vec3 sAz = b3Cross( rAd, perpZ );
		b3Vec3 sBz = b3Cross( rB, perpZ );

		// K = (mA + mB) * I - the two skew-inertia terms, with A's arms and B's
		// differing. Rebuilt every solve for the same reason axialMass is.
		// Upstream reaches the same answer with a Gaussian b3Solve2; the port
		// inverts, which is symmetric-safe and reuses the routine that carries
		// the uniform-scaling trade. Singular comes back zero -- no impulse.
		b3SymMatrix2 invK = b3InvertPointLineMass( mA, iA, sAy, sAz, mB, iB, sBy, sBz );

		b3Vec3 vRel = b3Sub( b3Sub( b3Add( vB, b3Cross( wB, rB ) ), vA ), b3Cross( wA, rAd ) );
		b3f cdotY = b3AddF( b3MulFC( b3Dot( vRel, perpY ), massScale ), biasY );
		b3f cdotZ = b3AddF( b3MulFC( b3Dot( vRel, perpZ ), massScale ), biasZ );

		b3Imp2 sol = b3MulSym2V( invK, cdotY, cdotZ );

		b3Imp2 deltaImpulse;
		deltaImpulse.x = b3SubImp( b3NegImp( sol.x ), b3MulImpC( joint->perpImpulse.x, impulseScale ) );
		deltaImpulse.y = b3SubImp( b3NegImp( sol.y ), b3MulImpC( joint->perpImpulse.y, impulseScale ) );

		joint->perpImpulse.x = b3AddImp( joint->perpImpulse.x, deltaImpulse.x );
		joint->perpImpulse.y = b3AddImp( joint->perpImpulse.y, deltaImpulse.y );

		b3Imp3 P = b3AddImp3( b3MulImpV( deltaImpulse.x, perpY ), b3MulImpV( deltaImpulse.y, perpZ ) );
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
