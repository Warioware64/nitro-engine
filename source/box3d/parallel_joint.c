// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Erin Catto        (original Box3D)
// Copyright (c) 2026 Warioware64       (Nitro Engine Advanced fixed-point port)
//
// This file is part of Nitro Engine Advanced

/// @file   parallel_joint.c
/// @brief  The parallel joint: an upright-keeper, and the first bounded 2x2.
///
/// @section shape What a parallel joint actually is
///
/// One constraint, and it is the smallest solved joint in the port:
///
///   1. a **2x2 collinearity** constraint holding body B's z axis parallel to
///      body A's z, leaving the twist about z entirely free.
///
/// There is no linear constraint at all -- the two bodies may be anywhere
/// relative to one another, and only their orientations are coupled. That is
/// what a caller wants for "keep this upright": a mast on a vehicle, a turret,
/// a camera boom. The twist stays free because an upright-keeper that also
/// fixed the heading would be a weld joint.
///
/// The constraint is **always a spring**, never rigid, and it is bounded by
/// `maxTorque`. Those two together are the joint's whole character: it is a
/// soft suggestion with a budget, so a strong enough disturbance wins and the
/// body tips, which is the readable behaviour the bound exists for.
///
/// @section scales What is new here, and what is not
///
/// Nearly every convention is Stage 2's, unchanged: the effective mass
/// accumulated wide and reciprocated once, `massScale` folded at Q30 into each
/// term, the clamp landing on the accumulator with the delta recomputed after.
/// What this file adds, and what it deliberately does not:
///
///   - **The bound is a disc, not a box.** `perpImpulse` is a two-vector and
///     its budget may be spent about either constrained axis or shared between
///     them, so b3ClampImp2 scales the pair toward the bound rather than
///     clamping each component. A per-component clamp would let the joint
///     exceed its stated torque by sqrt(2) along a diagonal. This is the first
///     joint to need the 2-vector form; the spherical joint's motor has used
///     the 3-vector b3ClampImp3 since Stage 4 for the same reason.
///   - **The ratio inside that clamp is formed at Q16.** It matters more here
///     than anywhere it has been argued before, because a parallel joint is
///     *defined* by its torque bound -- the bound is the normal operating
///     point, not an exceptional case. At 240 Hz a 0.05 N-m budget is fourteen
///     Q16 quanta and zero Q12 quanta, so narrowing the ratio would empty the
///     accumulator every sub-step and report a torque over the bound.
///   - **`maxTorque` bounds the impulse coefficient, not the torque**, and the
///     factor is two. The clamp lands on `perpImpulse`, which is then applied
///     along a Jacobian row of length `0.5 * sqrt(1 - (v.e)^2)` -- so a
///     saturated joint reports a peak torque of about half its stated budget,
///     and less as it tips. That is upstream's arrangement, kept rather than
///     rescaled: the factor is a bounded constant, it errs *conservatively*,
///     and correcting it would put the port out of step with upstream on a
///     number a caller may have tuned against. Measured at 0.2486 for a budget
///     of 0.5. b3ParallelJointDef::maxTorque carries the warning where a
///     caller will meet it, and test_world.c pins the ratio.
///   - **No `useBias`.** Upstream's solve does not take the parameter at all,
///     because there is no rigid constraint to relax on the unbiased pass --
///     the spring carries its own bias on every iteration. The port's three
///     dispatchers pass one to every type, so this file takes it and ignores
///     it, exactly as the motor joint does.
///   - **No new effective-mass math.** The 2x2 is b3InvertPerpMass on the two
///     axes from b3CollinearityPerpAxes, which is character-for-character the
///     revolute's collinearity block. Those two moved into joint.h at Stage 7
///     when this file became their third caller.
///   - **No b3MakeMatrixFromQuat anywhere**, so its Q12 non-orthonormality is
///     not in play here. Said plainly so a reader does not go looking.
///
/// @section silent The heavy pair this joint cannot hold
///
/// b3CollinearityPerpAxes' rows are half-length by construction, so the 2x2's
/// entries carry a factor of a quarter and its determinant a factor of a
/// sixteenth. b3InvertPerpMass returns zero -- "apply no impulse" -- once the
/// determinant underflows Q24, which happens when the pair's combined inverse
/// inertia about the constrained axes falls below about **1e-3**, i.e. a
/// rotational inertia above roughly 1000 kg m^2.
///
/// That is inherited from the revolute rather than introduced here, and zero is
/// the safe direction to fail in, but it is a real limit and it is silent: a
/// sufficiently heavy pair is simply not held upright. test_world.c pins it so
/// the number cannot drift without someone noticing.
///
/// @section cost What fixedRotation costs, since it is not free
///
/// This joint needs no rotation mass for anything else, so
/// `b3FixedRotationFromMass( b3InvertRotationMass( iA, iB ) )` is a full 3x3
/// wide inversion performed to produce a bool. It is kept anyway: it is the
/// tested path with six existing callers, it runs once per step per joint
/// rather than per sub-step, and the alternative is a second threshold in units
/// the port has no equivalent for -- upstream's `b3Det( iA + iB ) < 1000 *
/// FLT_MIN`, which would also have to form the sum this port never forms.
///
/// @section upstream One upstream wart, not reproduced
///
/// b3GetParallelJointTorque is a *getter* that writes `joint->perpAxisX` and
/// `perpAxisY` as a side effect while recomputing them. Nothing depends on the
/// write -- the values are rebuilt from scratch at the top of the next prepare
/// and solve -- but a query that mutates the simulation is a hazard for no
/// benefit, and it means calling the accessor twice is not the same as calling
/// it once. The port computes them into locals.
///
/// @section absent What is not here
///
/// b3DrawParallelJoint and every B3_REC hook, as in every other joint file.

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

void b3ParallelJoint_SetSpringHertz( b3JointId jointId, b3f hertz )
{
	B3_ASSERT( b3IsValidFloat( hertz ) && b3Raw( hertz ) >= 0 );

	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_parallelJoint );
	base->parallelJoint.hertz = hertz;
}

b3f b3ParallelJoint_GetSpringHertz( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_parallelJoint );
	return base->parallelJoint.hertz;
}

void b3ParallelJoint_SetSpringDampingRatio( b3JointId jointId, b3f dampingRatio )
{
	B3_ASSERT( b3IsValidFloat( dampingRatio ) && b3Raw( dampingRatio ) >= 0 );

	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_parallelJoint );
	base->parallelJoint.dampingRatio = dampingRatio;
}

b3f b3ParallelJoint_GetSpringDampingRatio( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_parallelJoint );
	return base->parallelJoint.dampingRatio;
}

void b3ParallelJoint_SetMaxTorque( b3JointId jointId, b3f maxTorque )
{
	B3_ASSERT( b3IsValidFloat( maxTorque ) );

	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_parallelJoint );

	// Clamped rather than asserted, as every other joint's force and torque
	// budget is: a negative bound is a caller mistake with an obvious
	// intention, and zero already means "apply nothing".
	base->parallelJoint.maxTorque = b3Raw( maxTorque ) > 0 ? maxTorque : b3f_zero;
}

b3f b3ParallelJoint_GetMaxTorque( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( world, jointId, b3_parallelJoint );
	return base->parallelJoint.maxTorque;
}

// =========================================================================
// Solver
// =========================================================================

b3Vec3 b3GetParallelJointTorque( b3World* world, b3JointSim* base )
{
	b3ParallelJoint* joint = &base->parallelJoint;

	// Recomputed into locals rather than written back through the struct --
	// see @section upstream. The axes are a pure function of the two cached
	// frame rotations, so this is the same answer upstream produces.
	b3Quat relQ = b3InvMulQuat( joint->quatA, joint->quatB );

	b3Vec3 perpAxisX, perpAxisY;
	b3CollinearityPerpAxes( joint->quatA, relQ, &perpAxisX, &perpAxisY );

	b3Imp3 angular =
		b3AddImp3( b3MulImpV( joint->perpImpulse.x, perpAxisX ), b3MulImpV( joint->perpImpulse.y, perpAxisY ) );

	b3f inv_h = world->inv_h;
	return b3MakeVec3( b3MulImpFToF( angular.x, inv_h ), b3MulImpFToF( angular.y, inv_h ),
					   b3MulImpFToF( angular.z, inv_h ) );
}

void b3PrepareParallelJoint( b3JointSim* base, b3StepContext* context )
{
	B3_ASSERT( base->type == b3_parallelJoint );

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

	// The 3x3 whose only purpose is this bool -- see @section cost. The sum
	// `invIA + invIB` is never formed here or below, for the reason
	// revolute_joint.c's prepare sets out at length.
	base->fixedRotation = b3FixedRotationFromMass( b3InvertRotationMass( base->invIA, base->invIB ) );

	b3ParallelJoint* joint = &base->parallelJoint;

	joint->indexA = bodyA->setIndex == b3_awakeSet ? localIndexA : B3_NULL_INDEX;
	joint->indexB = bodyB->setIndex == b3_awakeSet ? localIndexB : B3_NULL_INDEX;

	// Only the rotations. Every other joint also builds `frame.p` relative to
	// the centre of mass and a `deltaCenter`; this one has no linear constraint
	// to use them for, which is why b3ParallelJoint is the smallest member of
	// the union.
	joint->quatA = b3MulQuat( bodySimA->transform.q, base->localFrameA.q );
	joint->quatB = b3MulQuat( bodySimB->transform.q, base->localFrameB.q );

	// Needed by warm start, which runs before the solve rebuilds them against
	// the sub-step rotation. The duplication is upstream's and is deliberate:
	// the warm-start axes must be the ones the stored impulse was accumulated
	// along.
	b3Quat relQ = b3InvMulQuat( joint->quatA, joint->quatB );
	b3CollinearityPerpAxes( joint->quatA, relQ, &joint->perpAxisX, &joint->perpAxisY );

	joint->softness = b3MakeSoft( joint->hertz, joint->dampingRatio, context->h );

	if ( context->enableWarmStarting == false )
	{
		joint->perpImpulse = b3Imp2_zero;
	}
}

/// @note ITCM group B3_ITCM_PARALLEL -- see nea_config.h.
void B3_ITCM_IF( B3_ITCM_PARALLEL_WARM, b3WarmStartParallelJoint )( b3JointSim* base, b3StepContext* context )
{
	B3_ASSERT( base->type == b3_parallelJoint );

	b3MatrixW iA = base->invIA;
	b3MatrixW iB = base->invIB;

	b3BodyState dummyState = b3_identityBodyState;

	b3ParallelJoint* joint = &base->parallelJoint;
	b3BodyState* stateA = joint->indexA == B3_NULL_INDEX ? &dummyState : context->states + joint->indexA;
	b3BodyState* stateB = joint->indexB == B3_NULL_INDEX ? &dummyState : context->states + joint->indexB;

	// Angular only. There is no linear impulse to replay, so unlike every other
	// joint's warm start this one never touches linearVelocity.
	b3Imp3 angular =
		b3AddImp3( b3MulImpV( joint->perpImpulse.x, joint->perpAxisX ), b3MulImpV( joint->perpImpulse.y, joint->perpAxisY ) );

	b3Vec3 wA = b3Sub( stateA->angularVelocity, b3MulMWImp( iA, angular ) );
	b3Vec3 wB = b3Add( stateB->angularVelocity, b3MulMWImp( iB, angular ) );

	if ( stateA->flags & b3_dynamicFlag )
	{
		stateA->angularVelocity = wA;
	}

	if ( stateB->flags & b3_dynamicFlag )
	{
		stateB->angularVelocity = wB;
	}
}

/// @note ITCM group B3_ITCM_PARALLEL -- see nea_config.h.
void B3_ITCM_IF( B3_ITCM_PARALLEL_SOLVE, b3SolveParallelJoint )( b3JointSim* base, b3StepContext* context, bool useBias )
{
	B3_ASSERT( base->type == b3_parallelJoint );

	// The constraint is a spring and nothing else, so it carries its own bias
	// on every iteration and there is no rigid part to relax on the unbiased
	// pass. Upstream's solve does not take the parameter at all; the port's
	// dispatchers pass one to every type, so it is taken and dropped here --
	// the motor joint's arrangement, and for the same reason.
	B3_UNUSED( useBias );

	b3MatrixW iA = base->invIA;
	b3MatrixW iB = base->invIB;

	b3BodyState dummyState = b3_identityBodyState;

	b3ParallelJoint* joint = &base->parallelJoint;
	b3BodyState* stateA = joint->indexA == B3_NULL_INDEX ? &dummyState : context->states + joint->indexA;
	b3BodyState* stateB = joint->indexB == B3_NULL_INDEX ? &dummyState : context->states + joint->indexB;

	b3Vec3 wA = stateA->angularVelocity;
	b3Vec3 wB = stateB->angularVelocity;

	b3Quat quatA = b3MulQuat( stateA->deltaRotation, joint->quatA );
	b3Quat quatB = b3MulQuat( stateB->deltaRotation, joint->quatB );

	// A quaternion and its negation are the same rotation, and taking the
	// nearer of the two keeps the relative rotation inside [-pi, pi] so the
	// constraint error does not jump a full turn when the joint passes through
	// a half turn. The revolute and weld both do this; upstream's prismatic
	// does not, which Stage 6 recorded as an upstream asymmetry rather than
	// resolving. Upstream's parallel joint *does*, so it is here.
	if ( b3Raw( b3DotQuat( quatA, quatB ) ) < 0 )
	{
		quatB = b3NegateQuat( quatB );
	}

	b3Quat relQ = b3InvMulQuat( quatA, quatB );

	// -----------------------------------------------------------------
	// Collinearity: the bounded 2x2 that keeps the two z axes parallel
	// -----------------------------------------------------------------
	//
	// The `maxTorque > 0` guard is upstream's and is load-bearing rather than
	// an optimisation: a default-constructed parallel joint has a zero budget
	// and must do nothing at all. Said again in the def's doc comment, because
	// it is the one joint whose defaults leave it inert.
	if ( base->fixedRotation == false && b3Raw( joint->maxTorque ) > 0 )
	{
		// The constraint is on the x and y components of the relative
		// rotation's vector part -- they are zero exactly when the two z axes
		// are collinear. The z component is the twist, and leaving it out of
		// the constraint is what leaves the twist free.
		b3Softness cs = joint->softness;
		b3f rate = b3MulFC( cs.biasRate, cs.massScale );
		b3f biasX = b3MulFF( rate, b3NToF( relQ.v.x ) );
		b3f biasY = b3MulFF( rate, b3NToF( relQ.v.y ) );

		b3Vec3 perpAxisX, perpAxisY;
		b3CollinearityPerpAxes( quatA, relQ, &perpAxisX, &perpAxisY );
		joint->perpAxisX = perpAxisX;
		joint->perpAxisY = perpAxisY;

		// Rebuilt every solve because the axes are functions of relQ, which
		// changes within the step. Upstream recomputes it too, so unlike the
		// prismatic's this is not a departure -- it is simply what the joint
		// is. b3InvertPerpMass never forms `iA + iB` and returns zero for a
		// singular or indefinite result, which means "apply no impulse".
		b3SymMatrix2 invK = b3InvertPerpMass( perpAxisX, perpAxisY, iA, iB );

		b3Vec3 wRel = b3Sub( wB, wA );
		b3f cdotX = b3AddF( b3MulFC( b3Dot( wRel, perpAxisX ), cs.massScale ), biasX );
		b3f cdotY = b3AddF( b3MulFC( b3Dot( wRel, perpAxisY ), cs.massScale ), biasY );

		// Saturating, not wrapping, and this is the one place in the joint that
		// genuinely needs it. b3CollinearityPerpAxes' rows vanish at a half
		// turn, so a tumbling body drives invK toward infinity and the ask past
		// Q15.16 -- measured at 301,342 N-s on a box spun at 40 rad/s, nine
		// times the scale's ceiling. A wrapped ask reaches b3ClampImp2 with the
		// wrong sign and the joint then drives the tumble instead of arresting
		// it. Saturating preserves the direction, which is all that survives the
		// clamp anyway. See b3MulSym2VSat.
		b3Imp2 sol = b3MulSym2VSat( invK, cdotX, cdotY );

		b3Imp2 oldImpulse = joint->perpImpulse;

		b3Imp2 deltaImpulse;
		deltaImpulse.x = b3SubImp( b3NegImp( sol.x ), b3MulImpC( oldImpulse.x, cs.impulseScale ) );
		deltaImpulse.y = b3SubImp( b3NegImp( sol.y ), b3MulImpC( oldImpulse.y, cs.impulseScale ) );

		// The clamp lands on the accumulator and the delta is recomputed from
		// it, which is the port's universal shape: clamping the delta instead
		// would let the accumulator walk past the bound one step at a time.
		joint->perpImpulse = b3AddImp2( oldImpulse, deltaImpulse );
		joint->perpImpulse = b3ClampImp2( joint->perpImpulse, b3MulFTToImp( joint->maxTorque, context->h ) );
		deltaImpulse = b3SubImp2( joint->perpImpulse, oldImpulse );

		b3Imp3 angular =
			b3AddImp3( b3MulImpV( deltaImpulse.x, perpAxisX ), b3MulImpV( deltaImpulse.y, perpAxisY ) );

		wA = b3Sub( wA, b3MulMWImp( iA, angular ) );
		wB = b3Add( wB, b3MulMWImp( iB, angular ) );
	}

	if ( stateA->flags & b3_dynamicFlag )
	{
		stateA->angularVelocity = wA;
	}

	if ( stateB->flags & b3_dynamicFlag )
	{
		stateB->angularVelocity = wB;
	}
}
