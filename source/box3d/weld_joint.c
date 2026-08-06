// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Erin Catto        (original Box3D)
// Copyright (c) 2026 Warioware64       (Nitro Engine Advanced fixed-point port)
//
// This file is part of Nitro Engine Advanced

/// @file   weld_joint.c
/// @brief  The weld joint: two bodies made one, or joined by a 6-DOF spring.
///
/// @section shape What a weld actually is
///
/// Two constraints, solved in that order:
///
///   1. a **3-vector angular** constraint holding the two joint frames'
///      *orientations* together, which removes all three rotational degrees;
///   2. the **3x3 point-to-point** constraint locking the two anchors, which
///      removes all three linear degrees.
///
/// Six degrees removed is every degree there is, so a rigid weld is two bodies
/// behaving as one. It is not the same thing as *being* one body -- the
/// constraint is solved rather than assumed, so it can be broken at runtime and
/// the two halves fall apart, which is the whole reason a physics engine offers
/// it instead of telling you to merge the shapes.
///
/// Both constraints optionally **soften into springs**. At zero hertz each uses
/// `base->constraintSoftness` and is rigid; at a non-zero hertz it becomes a
/// spring of that frequency and damping, and the weld turns into a suspension.
/// The two halves are independent -- a rigid position with a springy
/// orientation is a legal and useful thing to ask for.
///
/// @section order Why the angular constraint goes first
///
/// Upstream's order, kept. The two couple through the shared velocity state,
/// and the angular one has no lever arms while the linear one does: solving
/// rotation first means the linear constraint sees the anchors where the
/// orientation constraint just put them, rather than correcting a position
/// error that the next rotational impulse is about to re-create. Reversing it
/// does not diverge, it just converges more slowly, which reads as a slightly
/// soft weld and is exactly the kind of thing no single-frame test catches.
///
/// @section scales What is new here, and what is not
///
/// Nothing, and that is worth stating: this is the first joint since Stage 2
/// that introduces no new fixed-point primitive. Both effective masses already
/// existed --
///
///   - `b3InvertRotationMass( iA, iB )` for the angular constraint, written in
///     Stage 4 for the ball joint's spring;
///   - `b3InvertPointMass( mA, iA, rA, mB, iB, rB )` for the linear one,
///     written in Stage 3 for the hinge's anchor lock.
///
/// Neither forms `invIA + invIB` or `mA + mB` in a b3iw, which is what Stage 5
/// spent its first increment on -- see revolute_joint.c's prepare.
///
/// Upstream reaches the linear answer with a Gaussian `b3Solve3`; the port
/// inverts, which is symmetric-safe here and reuses a routine that already
/// carries its determinant wide.
///
/// Every other convention is Stage 2's and Stage 4's unchanged and not
/// re-derived: `massScale` folded into each term at Q30 *before* the effective
/// mass rather than after, the softness bias multiplied by `massScale` once,
/// `deltaPosition` differenced at Q24 and narrowed once.
///
/// @section absent What is not here
///
/// b3DrawWeldJoint and every B3_REC hook, as in every other joint file.

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

void b3WeldJoint_SetLinearHertz( b3JointId jointId, b3f hertz )
{
	B3_ASSERT( b3IsValidFloat( hertz ) && b3Raw( hertz ) >= 0 );

	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_weldJoint );
	base->weldJoint.linearHertz = hertz;
}

b3f b3WeldJoint_GetLinearHertz( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_weldJoint );
	return base->weldJoint.linearHertz;
}

void b3WeldJoint_SetLinearDampingRatio( b3JointId jointId, b3f dampingRatio )
{
	B3_ASSERT( b3IsValidFloat( dampingRatio ) && b3Raw( dampingRatio ) >= 0 );

	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_weldJoint );
	base->weldJoint.linearDampingRatio = dampingRatio;
}

b3f b3WeldJoint_GetLinearDampingRatio( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_weldJoint );
	return base->weldJoint.linearDampingRatio;
}

void b3WeldJoint_SetAngularHertz( b3JointId jointId, b3f hertz )
{
	B3_ASSERT( b3IsValidFloat( hertz ) && b3Raw( hertz ) >= 0 );

	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_weldJoint );
	base->weldJoint.angularHertz = hertz;
}

b3f b3WeldJoint_GetAngularHertz( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_weldJoint );
	return base->weldJoint.angularHertz;
}

void b3WeldJoint_SetAngularDampingRatio( b3JointId jointId, b3f dampingRatio )
{
	B3_ASSERT( b3IsValidFloat( dampingRatio ) && b3Raw( dampingRatio ) >= 0 );

	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_weldJoint );
	base->weldJoint.angularDampingRatio = dampingRatio;
}

b3f b3WeldJoint_GetAngularDampingRatio( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_weldJoint );
	return base->weldJoint.angularDampingRatio;
}

/// The force the point-to-point constraint is applying, in world space.
b3Vec3 b3GetWeldJointForce( b3World* world, b3JointSim* base )
{
	b3Imp3 p = base->weldJoint.linearImpulse;
	b3f inv_h = world->inv_h;
	return b3MakeVec3( b3MulImpFToF( p.x, inv_h ), b3MulImpFToF( p.y, inv_h ), b3MulImpFToF( p.z, inv_h ) );
}

/// The torque the orientation constraint is applying, in world space.
///
/// One accumulator and no axes to resolve onto, so unlike the revolute's and
/// the spherical's this one has no sign question in it: the stored impulse is
/// already the world-space angular impulse the solve applied.
b3Vec3 b3GetWeldJointTorque( b3World* world, b3JointSim* base )
{
	b3Imp3 t = base->weldJoint.angularImpulse;
	b3f inv_h = world->inv_h;
	return b3MakeVec3( b3MulImpFToF( t.x, inv_h ), b3MulImpFToF( t.y, inv_h ), b3MulImpFToF( t.z, inv_h ) );
}

// =========================================================================
// Solver
// =========================================================================

void b3PrepareWeldJoint( b3JointSim* base, b3StepContext* context )
{
	B3_ASSERT( base->type == b3_weldJoint );

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

	b3WeldJoint* joint = &base->weldJoint;

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
	// with it. Never `b3AddMWMW( invIA, invIB )`: see revolute_joint.c's
	// prepare for what that wrapped and what it cost.
	joint->angularMass = b3InvertRotationMass( base->invIA, base->invIB );
	base->fixedRotation = b3FixedRotationFromMass( joint->angularMass );

	// Zero hertz means "not a spring": use the joint's own constraint softness
	// and be rigid. b3MakeSoft returns all-zero at zero hertz, so the branch is
	// about *which* softness to use rather than about guarding a division.
	joint->linearSpring = b3Raw( joint->linearHertz ) == 0
							  ? base->constraintSoftness
							  : b3MakeSoft( joint->linearHertz, joint->linearDampingRatio, context->h );

	joint->angularSpring = b3Raw( joint->angularHertz ) == 0
							   ? base->constraintSoftness
							   : b3MakeSoft( joint->angularHertz, joint->angularDampingRatio, context->h );

	if ( context->enableWarmStarting == false )
	{
		joint->linearImpulse = b3Imp3_zero;
		joint->angularImpulse = b3Imp3_zero;
	}
}

/// @note ITCM group B3_ITCM_WELD -- see nea_config.h.
void B3_ITCM_IF( B3_ITCM_WELD_WARM, b3WarmStartWeldJoint )( b3JointSim* base, b3StepContext* context )
{
	B3_ASSERT( base->type == b3_weldJoint );

	b3iw mA = base->invMassA;
	b3iw mB = base->invMassB;
	b3MatrixW iA = base->invIA;
	b3MatrixW iB = base->invIB;

	b3BodyState dummyState = b3_identityBodyState;

	b3WeldJoint* joint = &base->weldJoint;
	b3BodyState* stateA = joint->indexA == B3_NULL_INDEX ? &dummyState : context->states + joint->indexA;
	b3BodyState* stateB = joint->indexB == B3_NULL_INDEX ? &dummyState : context->states + joint->indexB;

	b3Vec3 rA = b3RotateVector( stateA->deltaRotation, joint->frameA.p );
	b3Vec3 rB = b3RotateVector( stateB->deltaRotation, joint->frameB.p );

	b3Imp3 P = joint->linearImpulse;
	b3Imp3 angular = joint->angularImpulse;

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

/// @note ITCM group B3_ITCM_WELD -- see nea_config.h.
void B3_ITCM_IF( B3_ITCM_WELD_SOLVE, b3SolveWeldJoint )( b3JointSim* base, b3StepContext* context, bool useBias )
{
	B3_ASSERT( base->type == b3_weldJoint );

	b3iw mA = base->invMassA;
	b3iw mB = base->invMassB;
	b3MatrixW iA = base->invIA;
	b3MatrixW iB = base->invIB;

	b3BodyState dummyState = b3_identityBodyState;

	b3WeldJoint* joint = &base->weldJoint;
	b3BodyState* stateA = joint->indexA == B3_NULL_INDEX ? &dummyState : context->states + joint->indexA;
	b3BodyState* stateB = joint->indexB == B3_NULL_INDEX ? &dummyState : context->states + joint->indexB;

	b3Vec3 vA = stateA->linearVelocity;
	b3Vec3 wA = stateA->angularVelocity;
	b3Vec3 vB = stateB->linearVelocity;
	b3Vec3 wB = stateB->angularVelocity;

	// -----------------------------------------------------------------
	// Angular: hold the two frames' orientations together
	// -----------------------------------------------------------------
	if ( base->fixedRotation == false )
	{
		b3Quat quatA = b3MulQuat( stateA->deltaRotation, joint->frameA.q );
		b3Quat quatB = b3MulQuat( stateB->deltaRotation, joint->frameB.q );

		// A quaternion and its negation are the same rotation, and picking the
		// nearer of the two keeps the error angle in [-pi, pi] instead of
		// jumping a full turn when the weld crosses the half-way point.
		if ( b3Raw( b3DotQuat( quatA, quatB ) ) < 0 )
		{
			quatB = b3NegateQuat( quatB );
		}

		b3Quat relQ = b3InvMulQuat( quatA, quatB );

		b3Vec3 bias = b3Vec3_zero;
		b3c massScale = b3c_one;
		b3c impulseScale = b3c_zero;

		// A spring applies its bias every iteration; a rigid constraint applies
		// it only on the biased pass. Hence the `||` rather than a plain
		// `useBias` -- without it a springy weld would relax on the second pass
		// and read as half the stiffness asked for.
		if ( useBias || b3Raw( joint->angularHertz ) > 0 )
		{
			// The orientation error as a rotation vector in radians, rotated
			// into world space. The target is the identity: a weld holds the
			// two frames as they were built, which is what the joint frames
			// already encode. Negated because C is measured from B to A while
			// the impulse is applied to B.
			b3Vec3 deltaRotation = b3DeltaQuatToRotation( relQ, b3Quat_identity );
			b3Vec3 c = b3Neg( b3RotateVector( quatA, deltaRotation ) );

			b3Softness soft = joint->angularSpring;
			bias = b3MulSV( b3MulFC( soft.biasRate, soft.massScale ), c );
			massScale = soft.massScale;
			impulseScale = soft.impulseScale;
		}

		b3Vec3 cdot = b3Sub( wB, wA );
		b3Vec3 driving = b3Add( b3MulSV( b3CToF( massScale ), cdot ), bias );

		b3Imp3 sol = b3MulMVToImp( joint->angularMass, driving );
		b3Imp3 impulse = b3SubImp3( b3NegImp3( sol ), b3MulCImp3( impulseScale, joint->angularImpulse ) );
		joint->angularImpulse = b3AddImp3( joint->angularImpulse, impulse );

		wA = b3Sub( wA, b3MulMWImp( iA, impulse ) );
		wB = b3Add( wB, b3MulMWImp( iB, impulse ) );
	}

	// -----------------------------------------------------------------
	// Linear: the 3x3 that locks the two anchors together
	// -----------------------------------------------------------------
	//
	// The revolute's and the spherical's point-to-point constraint, with the
	// softness coming from `linearSpring` instead of `constraintSoftness` so
	// that a non-zero `linearHertz` turns it into a spring.
	{
		b3Vec3 rA = b3RotateVector( stateA->deltaRotation, joint->frameA.p );
		b3Vec3 rB = b3RotateVector( stateB->deltaRotation, joint->frameB.p );

		b3Vec3 cdot = b3Sub( b3Sub( b3Add( vB, b3Cross( wB, rB ) ), vA ), b3Cross( wA, rA ) );

		b3Vec3 bias = b3Vec3_zero;
		b3c massScale = b3c_one;
		b3c impulseScale = b3c_zero;
		if ( useBias || b3Raw( joint->linearHertz ) > 0 )
		{
			// deltaPosition is Q24, so the difference is taken there and
			// narrowed once -- this is a difference of two nearly equal numbers
			// and rounding each side first rounds twice.
			b3Vec3 dp = b3W3ToVec3( b3SubW3( stateB->deltaPosition, stateA->deltaPosition ) );
			b3Vec3 separation = b3Add( b3Add( dp, b3Sub( rB, rA ) ), joint->deltaCenter );

			b3Softness soft = joint->linearSpring;
			bias = b3MulSV( b3MulFC( soft.biasRate, soft.massScale ), separation );
			massScale = soft.massScale;
			impulseScale = soft.impulseScale;
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
