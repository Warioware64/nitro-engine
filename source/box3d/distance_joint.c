// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Erin Catto        (original Box3D)
// Copyright (c) 2026 Warioware64       (Nitro Engine Advanced fixed-point port)
//
// This file is part of Nitro Engine Advanced

/// @file   distance_joint.c
/// @brief  The distance joint: one linear DOF, rigid or sprung.
///
/// @section why Why this joint came first
///
/// It is the smallest constraint in Box3D that still exercises every pattern
/// the other seven joints reuse -- a softness, a spring with a force range, a
/// two-sided limit with a speculative branch, a motor with a force bound, and
/// four warm-started impulse accumulators -- and its effective mass is a
/// *scalar*. Every later joint reuses those patterns over a 2x2 or 3x3 matrix
/// inverse, so any scale error made here would be inherited by 3,800 more
/// lines before anything caught it. Landing it alone, against closed-form
/// tests, is what stops that.
///
/// @section model The scale model
///
/// The contact solver is the reference for all of it, and this file follows it
/// deliberately rather than re-deriving:
///
///   - the effective mass accumulates at Q24 (every term is an inverse mass)
///     and is reciprocated once into Q12 by b3RcpW, exactly as
///     b3ManifoldConstraintPoint::normalMass is;
///   - the axis is a Q30 b3Dir3, not a Q12 b3Vec3, so b3DotDirF and b3MulNImp
///     keep their precision;
///   - massScale multiplies each *term* at Q30 rather than the finished Q12
///     product;
///   - the clamp lands on the accumulator, and the delta is recomputed from
///     it, which is what keeps a one-sided constraint one-sided under
///     quantization as well as under sign.
///
/// @section absent What is not here
///
/// b3DrawDistanceJoint, with the rest of the debug renderer, and every B3_REC
/// recording hook. The spring's rotation target and the joint-event thresholds
/// belong to joints that constrain a rotation.

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
//
// Each resolves the sim through b3GetJointSimCheckType, whose assert is the
// only thing standing between a caller passing a revolute joint id here and a
// union read at the wrong offset.

void b3DistanceJoint_SetLength( b3JointId jointId, b3f length )
{
	B3_ASSERT( b3IsValidFloat( length ) );

	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_distanceJoint );
	b3DistanceJoint* joint = &base->distanceJoint;

	joint->length = b3ClampF( length, B3_LINEAR_SLOP, B3_HUGE );

	// The accumulators describe a constraint that no longer exists. Keeping
	// them would warm-start the new rest length with the old one's impulse.
	joint->impulse = b3imp_zero;
	joint->lowerImpulse = b3imp_zero;
	joint->upperImpulse = b3imp_zero;
}

b3f b3DistanceJoint_GetLength( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_distanceJoint );
	return base->distanceJoint.length;
}

b3f b3DistanceJoint_GetCurrentLength( b3JointId jointId )
{
	b3World* world = b3GetUnlockedWorld( jointId.world0 );
	if ( world == NULL )
	{
		return b3f_zero;
	}

	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_distanceJoint );

	b3WorldTransform transformA = b3GetBodyTransform( world, base->bodyIdA );
	b3WorldTransform transformB = b3GetBodyTransform( world, base->bodyIdB );

	b3Vec3 pA = b3TransformPoint( transformA, base->localFrameA.p );
	b3Vec3 pB = b3TransformPoint( transformB, base->localFrameB.p );
	return b3Length( b3Sub( pB, pA ) );
}

void b3DistanceJoint_EnableSpring( b3JointId jointId, bool enableSpring )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_distanceJoint );
	base->distanceJoint.enableSpring = enableSpring;
}

bool b3DistanceJoint_IsSpringEnabled( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_distanceJoint );
	return base->distanceJoint.enableSpring;
}

void b3DistanceJoint_SetSpringHertz( b3JointId jointId, b3f hertz )
{
	B3_ASSERT( b3IsValidFloat( hertz ) && b3Raw( hertz ) >= 0 );

	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_distanceJoint );
	base->distanceJoint.hertz = hertz;
}

b3f b3DistanceJoint_GetSpringHertz( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_distanceJoint );
	return base->distanceJoint.hertz;
}

void b3DistanceJoint_SetSpringDampingRatio( b3JointId jointId, b3f dampingRatio )
{
	B3_ASSERT( b3IsValidFloat( dampingRatio ) && b3Raw( dampingRatio ) >= 0 );

	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_distanceJoint );
	base->distanceJoint.dampingRatio = dampingRatio;
}

b3f b3DistanceJoint_GetSpringDampingRatio( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_distanceJoint );
	return base->distanceJoint.dampingRatio;
}

void b3DistanceJoint_SetSpringForceRange( b3JointId jointId, b3f lowerForce, b3f upperForce )
{
	B3_ASSERT( b3IsValidFloat( lowerForce ) && b3IsValidFloat( upperForce ) );
	B3_ASSERT( b3LeF( lowerForce, upperForce ) );

	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_distanceJoint );
	base->distanceJoint.lowerSpringForce = lowerForce;
	base->distanceJoint.upperSpringForce = upperForce;
}

void b3DistanceJoint_GetSpringForceRange( b3JointId jointId, b3f* lowerForce, b3f* upperForce )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_distanceJoint );
	*lowerForce = base->distanceJoint.lowerSpringForce;
	*upperForce = base->distanceJoint.upperSpringForce;
}

void b3DistanceJoint_EnableLimit( b3JointId jointId, bool enableLimit )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_distanceJoint );
	base->distanceJoint.enableLimit = enableLimit;
}

bool b3DistanceJoint_IsLimitEnabled( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_distanceJoint );
	return base->distanceJoint.enableLimit;
}

void b3DistanceJoint_SetLengthRange( b3JointId jointId, b3f minLength, b3f maxLength )
{
	B3_ASSERT( b3IsValidFloat( minLength ) && b3IsValidFloat( maxLength ) );

	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_distanceJoint );
	b3DistanceJoint* joint = &base->distanceJoint;

	minLength = b3ClampF( minLength, B3_LINEAR_SLOP, B3_HUGE );
	maxLength = b3ClampF( maxLength, B3_LINEAR_SLOP, B3_HUGE );

	// Sorted rather than asserted, which is upstream's choice: a caller that
	// passes them the wrong way round gets the range it meant.
	joint->minLength = b3MinF( minLength, maxLength );
	joint->maxLength = b3MaxF( minLength, maxLength );

	joint->impulse = b3imp_zero;
	joint->lowerImpulse = b3imp_zero;
	joint->upperImpulse = b3imp_zero;
}

b3f b3DistanceJoint_GetMinLength( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_distanceJoint );
	return base->distanceJoint.minLength;
}

b3f b3DistanceJoint_GetMaxLength( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_distanceJoint );
	return base->distanceJoint.maxLength;
}

void b3DistanceJoint_EnableMotor( b3JointId jointId, bool enableMotor )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_distanceJoint );
	if ( enableMotor != base->distanceJoint.enableMotor )
	{
		base->distanceJoint.enableMotor = enableMotor;
		base->distanceJoint.motorImpulse = b3imp_zero;
	}
}

bool b3DistanceJoint_IsMotorEnabled( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_distanceJoint );
	return base->distanceJoint.enableMotor;
}

void b3DistanceJoint_SetMotorSpeed( b3JointId jointId, b3f motorSpeed )
{
	B3_ASSERT( b3IsValidFloat( motorSpeed ) );

	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_distanceJoint );
	base->distanceJoint.motorSpeed = motorSpeed;
}

b3f b3DistanceJoint_GetMotorSpeed( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_distanceJoint );
	return base->distanceJoint.motorSpeed;
}

void b3DistanceJoint_SetMaxMotorForce( b3JointId jointId, b3f force )
{
	B3_ASSERT( b3IsValidFloat( force ) && b3Raw( force ) >= 0 );

	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_distanceJoint );
	base->distanceJoint.maxMotorForce = force;
}

b3f b3DistanceJoint_GetMaxMotorForce( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_distanceJoint );
	return base->distanceJoint.maxMotorForce;
}

b3f b3DistanceJoint_GetMotorForce( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_distanceJoint );
	return b3MulImpFToF( base->distanceJoint.motorImpulse, world->inv_h );
}

/// The axial force the joint applied over the last sub-step, in world space.
///
/// All four accumulators contribute, with the upper limit's negated because it
/// pushes the other way along the axis. Upstream multiplies the sum by inv_h
/// and then by the axis; the port does the axis first, at Q30, and converts
/// each component from an impulse to a force afterwards -- one narrowing
/// instead of two, and the Q30 direction is not thrown away before it is used.
b3Vec3 b3GetDistanceJointForce( b3World* world, b3JointSim* base )
{
	b3DistanceJoint* joint = &base->distanceJoint;

	b3WorldTransform transformA = b3GetBodyTransform( world, base->bodyIdA );
	b3WorldTransform transformB = b3GetBodyTransform( world, base->bodyIdB );

	b3Vec3 pA = b3TransformPoint( transformA, base->localFrameA.p );
	b3Vec3 pB = b3TransformPoint( transformB, base->localFrameB.p );
	b3Dir3 axis = b3NormalizeToDir( b3Sub( pB, pA ) );

	b3imp axialImpulse = b3AddImp( b3SubImp( b3AddImp( joint->impulse, joint->lowerImpulse ), joint->upperImpulse ),
								   joint->motorImpulse );

	b3Imp3 P = b3MulNImp( axialImpulse, axis );
	b3f inv_h = world->inv_h;
	return b3MakeVec3( b3MulImpFToF( P.x, inv_h ), b3MulImpFToF( P.y, inv_h ), b3MulImpFToF( P.z, inv_h ) );
}

// =========================================================================
// Solver
// =========================================================================
//
// 1-D constrained system
// m (v2 - v1) = lambda
// v2 + (beta/h) * x1 + gamma * lambda = 0, gamma has units of inverse mass.
// x2 = x1 + h * v2
//
// C = norm(p2 - p1) - L
// u = (p2 - p1) / norm(p2 - p1)
// Cdot = dot(u, v2 + cross(w2, r2) - v1 - cross(w1, r1))
// J = [-u -cross(r1, u) u cross(r2, u)]
// K = J * invM * JT
//   = invMass1 + invI1 * cross(r1, u)^2 + invMass2 + invI2 * cross(r2, u)^2

void b3PrepareDistanceJoint( b3JointSim* base, b3StepContext* context )
{
	B3_ASSERT( base->type == b3_distanceJoint );

	// Chase each body id to the solver set the body actually lives in. A joint
	// in the graph may have one sleeping or static end.
	int idA = base->bodyIdA;
	int idB = base->bodyIdB;

	b3World* world = context->world;
	b3Body* bodyA = b3Array_Get( world->bodies, idA );
	b3Body* bodyB = b3Array_Get( world->bodies, idB );

	B3_ASSERT( bodyA->setIndex == b3_awakeSet || bodyB->setIndex == b3_awakeSet );

	b3SolverSet* setA = b3Array_Get( world->solverSets, bodyA->setIndex );
	b3SolverSet* setB = b3Array_Get( world->solverSets, bodyB->setIndex );

	int localIndexA = bodyA->localIndex;
	int localIndexB = bodyB->localIndex;

	b3BodySim* bodySimA = b3Array_Get( setA->bodySims, localIndexA );
	b3BodySim* bodySimB = b3Array_Get( setB->bodySims, localIndexB );

	b3iw mA = bodySimA->invMass;
	b3MatrixW iA = bodySimA->invInertiaWorld;
	b3iw mB = bodySimB->invMass;
	b3MatrixW iB = bodySimB->invInertiaWorld;

	base->invMassA = mA;
	base->invMassB = mB;
	base->invIA = iA;
	base->invIB = iB;

	b3DistanceJoint* joint = &base->distanceJoint;

	// B3_NULL_INDEX means "not in the solver's state array", i.e. static or
	// sleeping, and the solve reads a dummy identity state for it instead.
	joint->indexA = bodyA->setIndex == b3_awakeSet ? localIndexA : B3_NULL_INDEX;
	joint->indexB = bodyB->setIndex == b3_awakeSet ? localIndexB : B3_NULL_INDEX;

	// Anchors in world space, relative to each centre of mass -- the frame the
	// solver works in, and the reason the local frames are stored against the
	// body origin instead.
	joint->anchorA = b3RotateVector( bodySimA->transform.q, b3Sub( base->localFrameA.p, bodySimA->localCenter ) );
	joint->anchorB = b3RotateVector( bodySimB->transform.q, b3Sub( base->localFrameB.p, bodySimB->localCenter ) );
	joint->deltaCenter = b3Sub( bodySimB->center, bodySimA->center );

	b3Vec3 rA = joint->anchorA;
	b3Vec3 rB = joint->anchorB;
	b3Vec3 separation = b3Add( b3Sub( rB, rA ), joint->deltaCenter );
	b3Dir3 axis = b3NormalizeToDir( separation );

	// k = mA + mB + dot( crA, iA*crA ) + dot( crB, iB*crB ). Every term is an
	// inverse mass, so the sum stays at Q24 and its reciprocal -- an effective
	// mass -- comes back at Q12. Identical in shape to kNormal in
	// contact_solver.c, and b3RcpWide reads a zero k as "apply no impulse",
	// which is the two-static-bodies case reaching here through the graph.
	//
	// Stage 6: this used to be three b3AddW into a b3iw, and both halves of that
	// could wrap. `b3AddW( mA, mB )` is the one that mattered -- B3_MIN_MASS_RAW
	// caps a single inverse mass at ~124 against Q7.24's ceiling of 128, so one
	// body always fits and two bodies under about 16 g never do. The sum came
	// back negative and inverted the constraint. b3LeverInertiaSumWide sums both
	// masses and both quadratic forms in one int64, and b3RcpWide delegates to
	// b3RcpW inside b3iw's range, so every result that was correct before is
	// bit-identical.
	b3Vec3 crA = b3CrossDirRight( rA, axis );
	b3Vec3 crB = b3CrossDirRight( rB, axis );
	joint->axialMass = b3RcpWide( b3LeverInertiaSumWide( mA, iA, crA, mB, iB, crB ) );

	joint->distanceSoftness = b3MakeSoft( joint->hertz, joint->dampingRatio, context->h );

	if ( context->enableWarmStarting == false )
	{
		joint->impulse = b3imp_zero;
		joint->lowerImpulse = b3imp_zero;
		joint->upperImpulse = b3imp_zero;
		joint->motorImpulse = b3imp_zero;
	}
}

/// The current axis and anchors, recomputed from the within-step deltas.
///
/// Warm start and solve both need exactly this, and getting the deltaPosition
/// difference wrong is the one mistake that would not show up as a type error:
/// it is a b3Vec3W at Q24, and it must be differenced *there* and narrowed
/// once. Narrowing each side first rounds twice on a difference of two nearly
/// equal numbers, which for a joint at rest is the whole signal.
static b3Vec3 b3DistanceJointSeparation( const b3DistanceJoint* joint, const b3BodyState* stateA,
										 const b3BodyState* stateB, b3Vec3* rAOut, b3Vec3* rBOut )
{
	b3Vec3 rA = b3RotateVector( stateA->deltaRotation, joint->anchorA );
	b3Vec3 rB = b3RotateVector( stateB->deltaRotation, joint->anchorB );

	b3Vec3 dp = b3W3ToVec3( b3SubW3( stateB->deltaPosition, stateA->deltaPosition ) );
	b3Vec3 ds = b3Add( dp, b3Sub( rB, rA ) );

	*rAOut = rA;
	*rBOut = rB;
	return b3Add( joint->deltaCenter, ds );
}

/// @note ITCM group B3_ITCM_DISTANCE -- see nea_config.h.
void B3_ITCM_IF( B3_ITCM_DISTANCE, b3WarmStartDistanceJoint )( b3JointSim* base, b3StepContext* context )
{
	B3_ASSERT( base->type == b3_distanceJoint );

	b3iw mA = base->invMassA;
	b3iw mB = base->invMassB;
	b3MatrixW iA = base->invIA;
	b3MatrixW iB = base->invIB;

	b3BodyState dummyState = b3_identityBodyState;

	b3DistanceJoint* joint = &base->distanceJoint;
	b3BodyState* stateA = joint->indexA == B3_NULL_INDEX ? &dummyState : context->states + joint->indexA;
	b3BodyState* stateB = joint->indexB == B3_NULL_INDEX ? &dummyState : context->states + joint->indexB;

	b3Vec3 rA, rB;
	b3Vec3 separation = b3DistanceJointSeparation( joint, stateA, stateB, &rA, &rB );
	b3Dir3 axis = b3NormalizeToDir( separation );

	b3imp axialImpulse = b3AddImp( b3SubImp( b3AddImp( joint->impulse, joint->lowerImpulse ), joint->upperImpulse ),
								   joint->motorImpulse );
	b3Imp3 P = b3MulNImp( axialImpulse, axis );

	if ( stateA->flags & b3_dynamicFlag )
	{
		stateA->linearVelocity = b3Sub( stateA->linearVelocity, b3MulImpW3( P, mA ) );
		stateA->angularVelocity = b3Sub( stateA->angularVelocity, b3MulMWImp( iA, b3CrossVImp( rA, P ) ) );
	}

	if ( stateB->flags & b3_dynamicFlag )
	{
		stateB->linearVelocity = b3Add( stateB->linearVelocity, b3MulImpW3( P, mB ) );
		stateB->angularVelocity = b3Add( stateB->angularVelocity, b3MulMWImp( iB, b3CrossVImp( rB, P ) ) );
	}
}

/// @note ITCM group B3_ITCM_DISTANCE -- see nea_config.h.
void B3_ITCM_IF( B3_ITCM_DISTANCE, b3SolveDistanceJoint )( b3JointSim* base, b3StepContext* context, bool useBias )
{
	B3_ASSERT( base->type == b3_distanceJoint );

	b3iw mA = base->invMassA;
	b3iw mB = base->invMassB;
	b3MatrixW iA = base->invIA;
	b3MatrixW iB = base->invIB;

	b3BodyState dummyState = b3_identityBodyState;

	b3DistanceJoint* joint = &base->distanceJoint;
	b3BodyState* stateA = joint->indexA == B3_NULL_INDEX ? &dummyState : context->states + joint->indexA;
	b3BodyState* stateB = joint->indexB == B3_NULL_INDEX ? &dummyState : context->states + joint->indexB;

	b3Vec3 vA = stateA->linearVelocity;
	b3Vec3 wA = stateA->angularVelocity;
	b3Vec3 vB = stateB->linearVelocity;
	b3Vec3 wB = stateB->angularVelocity;

	b3Vec3 rA, rB;
	b3Vec3 separation = b3DistanceJointSeparation( joint, stateA, stateB, &rA, &rB );

	b3f length = b3Length( separation );
	b3Dir3 axis = b3NormalizeToDir( separation );

	b3f axialMass = joint->axialMass;
	b3t h = context->h;

	// The joint is soft only when the spring is on *and* it has room to move:
	// a limit whose two ends coincide is a rigid constraint by another name,
	// and upstream routes it to the rigid branch rather than solving a spring
	// against an immovable range.
	bool soft = joint->enableSpring && ( b3LtF( joint->minLength, joint->maxLength ) || joint->enableLimit == false );

	if ( soft )
	{
		if ( b3Raw( joint->hertz ) > 0 )
		{
			b3Vec3 vr = b3Add( b3Sub( vB, vA ), b3Sub( b3Cross( wB, rB ), b3Cross( wA, rA ) ) );
			b3f Cdot = b3DotDirF( axis, vr );
			b3f C = b3SubF( length, joint->length );

			// massScale multiplies both terms, folded in at Q30 against a Q12
			// operand rather than scaling the finished product -- the form
			// contact_solver.c uses, and the reason its bias is pre-scaled.
			b3Softness soft2 = joint->distanceSoftness;
			b3f bias = b3MulFF( b3MulFC( soft2.biasRate, soft2.massScale ), C );
			b3f driving = b3AddF( b3MulFC( Cdot, soft2.massScale ), bias );

			b3imp oldImpulse = joint->impulse;
			b3imp deltaImpulse =
				b3SubImp( b3NegImp( b3MulFFToImp( axialMass, driving ) ), b3MulImpC( oldImpulse, soft2.impulseScale ) );

			// The force range is a force; the accumulator is an impulse. One
			// sub-step's worth of that force is the bound, which is what
			// b3MulFTToImp spells.
			joint->impulse = b3ClampImp( b3AddImp( oldImpulse, deltaImpulse ),
										 b3MulFTToImp( joint->lowerSpringForce, h ),
										 b3MulFTToImp( joint->upperSpringForce, h ) );
			deltaImpulse = b3SubImp( joint->impulse, oldImpulse );

			b3Imp3 P = b3MulNImp( deltaImpulse, axis );
			vA = b3Sub( vA, b3MulImpW3( P, mA ) );
			wA = b3Sub( wA, b3MulMWImp( iA, b3CrossVImp( rA, P ) ) );
			vB = b3Add( vB, b3MulImpW3( P, mB ) );
			wB = b3Add( wB, b3MulMWImp( iB, b3CrossVImp( rB, P ) ) );
		}

		if ( joint->enableLimit )
		{
			// Lower limit: the separation may not fall below minLength.
			{
				b3Vec3 vr = b3Add( b3Sub( vB, vA ), b3Sub( b3Cross( wB, rB ), b3Cross( wA, rA ) ) );
				b3f Cdot = b3DotDirF( axis, vr );
				b3f C = b3SubF( length, joint->minLength );

				b3f bias = b3f_zero;
				b3c massScale = b3c_one;
				b3c impulseScale = b3c_zero;
				if ( b3Raw( C ) > 0 )
				{
					// Speculative: still inside the range, so the constraint
					// may only stop it closing faster than one sub-step of
					// slack allows.
					bias = b3MulFF( C, context->inv_h );
				}
				else if ( useBias )
				{
					b3Softness cs = base->constraintSoftness;
					bias = b3MulFF( b3MulFC( cs.biasRate, cs.massScale ), C );
					massScale = cs.massScale;
					impulseScale = cs.impulseScale;
				}

				b3f driving = b3AddF( b3MulFC( Cdot, massScale ), bias );
				b3imp deltaImpulse = b3SubImp( b3NegImp( b3MulFFToImp( axialMass, driving ) ),
											   b3MulImpC( joint->lowerImpulse, impulseScale ) );

				b3imp newImpulse = b3MaxImp( b3AddImp( joint->lowerImpulse, deltaImpulse ), b3imp_zero );
				deltaImpulse = b3SubImp( newImpulse, joint->lowerImpulse );
				joint->lowerImpulse = newImpulse;

				b3Imp3 P = b3MulNImp( deltaImpulse, axis );
				vA = b3Sub( vA, b3MulImpW3( P, mA ) );
				wA = b3Sub( wA, b3MulMWImp( iA, b3CrossVImp( rA, P ) ) );
				vB = b3Add( vB, b3MulImpW3( P, mB ) );
				wB = b3Add( wB, b3MulMWImp( iB, b3CrossVImp( rB, P ) ) );
			}

			// Upper limit: the separation may not exceed maxLength. Same
			// constraint with the axis reversed, which is why the relative
			// velocity is formed A-minus-B here and the impulse is applied
			// negated.
			{
				b3Vec3 vr = b3Add( b3Sub( vA, vB ), b3Sub( b3Cross( wA, rA ), b3Cross( wB, rB ) ) );
				b3f Cdot = b3DotDirF( axis, vr );
				b3f C = b3SubF( joint->maxLength, length );

				b3f bias = b3f_zero;
				b3c massScale = b3c_one;
				b3c impulseScale = b3c_zero;
				if ( b3Raw( C ) > 0 )
				{
					bias = b3MulFF( C, context->inv_h );
				}
				else if ( useBias )
				{
					b3Softness cs = base->constraintSoftness;
					bias = b3MulFF( b3MulFC( cs.biasRate, cs.massScale ), C );
					massScale = cs.massScale;
					impulseScale = cs.impulseScale;
				}

				b3f driving = b3AddF( b3MulFC( Cdot, massScale ), bias );
				b3imp deltaImpulse = b3SubImp( b3NegImp( b3MulFFToImp( axialMass, driving ) ),
											   b3MulImpC( joint->upperImpulse, impulseScale ) );

				b3imp newImpulse = b3MaxImp( b3AddImp( joint->upperImpulse, deltaImpulse ), b3imp_zero );
				deltaImpulse = b3SubImp( newImpulse, joint->upperImpulse );
				joint->upperImpulse = newImpulse;

				b3Imp3 P = b3MulNImp( b3NegImp( deltaImpulse ), axis );
				vA = b3Sub( vA, b3MulImpW3( P, mA ) );
				wA = b3Sub( wA, b3MulMWImp( iA, b3CrossVImp( rA, P ) ) );
				vB = b3Add( vB, b3MulImpW3( P, mB ) );
				wB = b3Add( wB, b3MulMWImp( iB, b3CrossVImp( rB, P ) ) );
			}
		}

		if ( joint->enableMotor )
		{
			b3Vec3 vr = b3Add( b3Sub( vB, vA ), b3Sub( b3Cross( wB, rB ), b3Cross( wA, rA ) ) );
			b3f Cdot = b3DotDirF( axis, vr );

			b3imp oldImpulse = joint->motorImpulse;
			b3imp deltaImpulse = b3MulFFToImp( axialMass, b3SubF( joint->motorSpeed, Cdot ) );

			b3imp maxImpulse = b3MulFTToImp( joint->maxMotorForce, h );
			joint->motorImpulse =
				b3ClampImp( b3AddImp( oldImpulse, deltaImpulse ), b3NegImp( maxImpulse ), maxImpulse );
			deltaImpulse = b3SubImp( joint->motorImpulse, oldImpulse );

			b3Imp3 P = b3MulNImp( deltaImpulse, axis );
			vA = b3Sub( vA, b3MulImpW3( P, mA ) );
			wA = b3Sub( wA, b3MulMWImp( iA, b3CrossVImp( rA, P ) ) );
			vB = b3Add( vB, b3MulImpW3( P, mB ) );
			wB = b3Add( wB, b3MulMWImp( iB, b3CrossVImp( rB, P ) ) );
		}
	}
	else
	{
		// Rigid: hold the rest length exactly, with no clamp on the
		// accumulator because the constraint is two-sided.
		b3Vec3 vr = b3Add( b3Sub( vB, vA ), b3Sub( b3Cross( wB, rB ), b3Cross( wA, rA ) ) );
		b3f Cdot = b3DotDirF( axis, vr );
		b3f C = b3SubF( length, joint->length );

		b3f bias = b3f_zero;
		b3c massScale = b3c_one;
		b3c impulseScale = b3c_zero;
		if ( useBias )
		{
			b3Softness cs = base->constraintSoftness;
			bias = b3MulFF( b3MulFC( cs.biasRate, cs.massScale ), C );
			massScale = cs.massScale;
			impulseScale = cs.impulseScale;
		}

		b3f driving = b3AddF( b3MulFC( Cdot, massScale ), bias );
		b3imp deltaImpulse =
			b3SubImp( b3NegImp( b3MulFFToImp( axialMass, driving ) ), b3MulImpC( joint->impulse, impulseScale ) );
		joint->impulse = b3AddImp( joint->impulse, deltaImpulse );

		b3Imp3 P = b3MulNImp( deltaImpulse, axis );
		vA = b3Sub( vA, b3MulImpW3( P, mA ) );
		wA = b3Sub( wA, b3MulMWImp( iA, b3CrossVImp( rA, P ) ) );
		vB = b3Add( vB, b3MulImpW3( P, mB ) );
		wB = b3Add( wB, b3MulMWImp( iB, b3CrossVImp( rB, P ) ) );
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
