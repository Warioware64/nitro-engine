// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Erin Catto        (original Box3D)
// Copyright (c) 2026 Warioware64       (Nitro Engine Advanced fixed-point port)
//
// This file is part of Nitro Engine Advanced

#include "contact_solver.h"

#include "body.h"
#include "constraint_graph.h"
#include "contact.h"
#include "core.h"
#include "physics_world.h"
#include "solver_set.h"

// Contact separation for sub-stepping, which is what baseSeparation exists to
// make cheap:
//
//   s(t) = s0 + dot( cB(t) + rB(t) - cA(t) - rA(t), n )
//        = s0 + dot( cB0 - cA0, n ) + dot( dp + rot(dqB,rB0) - rot(dqA,rA0), n )
//        = baseSeparation      + dot( dp + rot(dqB,rB0) - rot(dqA,rA0), n )
//
// with the normal held constant across the step. Only the second term moves,
// so prepare computes the first once.

/// 1 / B3_SPECULATIVE_DISTANCE, for the friction-centre weight ramp.
///
/// Computed from the same literal rather than written as 50. The constant is
/// b3Makeb3f( 82 ) and 4096/82 is 49.95, so a hand-written 50 would put the
/// ramp 0.1% away from the distance the narrow phase actually uses -- small,
/// but a discrepancy with no reason to exist.
#define B3_INV_SPECULATIVE_DISTANCE b3DivFF( b3f_one, B3_SPECULATIVE_DISTANCE )

// =========================================================================
// Prepare
// =========================================================================

void b3PrepareContacts( int startIndex, int endIndex, b3StepContext* context )
{
	b3World* world = context->world;
	b3BodySim* bodySims = context->sims;
	b3BodyState* bodyStates = context->states;

	b3GraphColor* color = world->constraintGraph.colors + B3_OVERFLOW_INDEX;
	b3ContactSpec* specs = color->contacts.data;
	b3ManifoldConstraint* manifoldBase = color->manifoldConstraints;
	b3ContactConstraint* base = color->contactConstraints;

	b3c warmStartScale = context->enableWarmStarting ? b3c_one : b3c_zero;
	b3f invTau = B3_INV_SPECULATIVE_DISTANCE;

	// Upstream walks colour spans to find each contact; there is one colour, so
	// the flat index is the local index.
	for ( int index = startIndex; index < endIndex; ++index )
	{
		b3ContactConstraint* contactConstraint = base + index;

		int contactId = specs[index].contactId;
		b3Contact* contact = b3Array_Get( world->contacts, contactId );
		B3_ASSERT( contact->contactId == contactId );

		int indexA = contact->bodySimIndexA;
		int indexB = contact->bodySimIndexB;

		b3iw mA = b3iw_zero;
		b3MatrixW iA = b3MatW_zero;
		b3Vec3 vA = b3Vec3_zero;
		b3Vec3 wA = b3Vec3_zero;

		if ( indexA != B3_NULL_INDEX )
		{
			b3BodySim* simA = bodySims + indexA;
			mA = simA->invMass;
			iA = simA->invInertiaWorld;

			b3BodyState* stateA = bodyStates + indexA;
			vA = stateA->linearVelocity;
			wA = stateA->angularVelocity;
		}

		b3iw mB = b3iw_zero;
		b3MatrixW iB = b3MatW_zero;
		b3Vec3 vB = b3Vec3_zero;
		b3Vec3 wB = b3Vec3_zero;

		if ( indexB != B3_NULL_INDEX )
		{
			b3BodySim* simB = bodySims + indexB;
			mB = simB->invMass;
			iB = simB->invInertiaWorld;

			b3BodyState* stateB = bodyStates + indexB;
			vB = stateB->linearVelocity;
			wB = stateB->angularVelocity;
		}

		int manifoldCount = contact->manifoldCount;
		contactConstraint->contact = contact;
		contactConstraint->manifoldCount = manifoldCount;
		contactConstraint->indexA = indexA;
		contactConstraint->indexB = indexB;
		contactConstraint->invIA = iA;
		contactConstraint->invMassA = mA;
		contactConstraint->invIB = iB;
		contactConstraint->invMassB = mB;
		// Never `b3InvertMatrixW( b3AddMWMW( iA, iB ) )`. b3InvertInertia caps a
		// *single* body's inverse inertia at Q7.24's ceiling of 128 by scaling
		// rather than wrapping, so neither matrix can overflow -- but their sum
		// can, and b3AddMWMW wraps it to a large negative, which inverts the sign
		// of the effective mass. Two bodies light enough to reach it is an
		// ordinary scene: Stage 5 measured a 0.18 kg head against a 0.59 kg torso
		// at 128.197.
		//
		// b3InvertRotationMass is that inverse with the sum never formed --
		// accumulated wide and uniformly scaled before inversion -- and it
		// delegates to the narrow routine for anything already in range, so every
		// result that was correct before is bit-identical and only the wrapping
		// case moved.
		contactConstraint->rollingMass = b3InvertRotationMass( iA, iB );
		contactConstraint->softness =
			( contact->flags & b3_contactStaticFlag ) != 0 ? context->staticSoftness : context->contactSoftness;
		contactConstraint->friction = contact->friction;
		contactConstraint->restitution = contact->restitution;
		contactConstraint->rollingResistance = contact->rollingResistance;

		b3ManifoldConstraint* manifoldConstraints = manifoldBase + specs[index].manifoldStart;
		contactConstraint->constraints = manifoldConstraints;

		for ( int manifoldIndex = 0; manifoldIndex < manifoldCount; ++manifoldIndex )
		{
			b3Manifold* manifold = contact->manifolds + manifoldIndex;
			b3ManifoldConstraint* constraint = manifoldConstraints + manifoldIndex;
			int pointCount = manifold->pointCount;

			// The frame moves to Q30 here and stays there for the rest of the
			// step. b3ArbitraryPerp rather than b3Perp: b3Perp crosses against
			// an axis, so its result is as short as the sine of the angle
			// between them, and a short Q12 vector has already lost its low
			// bits before b3NormalizeToDir sees it. The tangent mass is formed
			// from these, so a mis-scaled tangent is a mis-scaled friction
			// cone.
			b3Vec3 normalF = manifold->normal;
			b3Vec3 tangent1F = b3ArbitraryPerp( normalF );
			b3Vec3 tangent2F = b3Cross( tangent1F, normalF );

			b3Dir3 normal = b3NormalizeToDir( normalF );
			b3Dir3 tangent1 = b3NormalizeToDir( tangent1F );
			b3Dir3 tangent2 = b3NormalizeToDir( tangent2F );

			constraint->pointCount = pointCount;
			constraint->normal = normal;
			constraint->tangent1 = tangent1;
			constraint->tangent2 = tangent2;

			constraint->tangentVelocity1 = b3DotDirF( tangent1, contact->tangentVelocity );
			constraint->tangentVelocity2 = b3DotDirF( tangent2, contact->tangentVelocity );

			// -----------------------------------------------------------
			// Per point: anchors, base separation, effective mass
			// -----------------------------------------------------------
			//
			// The friction centre is a weighted mean of the anchors, and
			// upstream forms it as `sum(w * r) * (1 / sum(w))`. That
			// reciprocal cannot be materialised here. B3_MIN_FRICTION_WEIGHT
			// is one Q30 quantum -- 9.3e-10, the faithful translation of
			// upstream's float epsilon -- and its reciprocal is 1.07e9, which
			// no scale in this port holds.
			//
			// So the sums are accumulated wide and divided once per component,
			// which needs no bound on the denominator beyond nonzero and is
			// the DS's hardware divider on device.
			int64_t centerAx = 0, centerAy = 0, centerAz = 0;
			int64_t centerBx = 0, centerBy = 0, centerBz = 0;
			int64_t totalFrictionWeight = 0;

			for ( int pointIndex = 0; pointIndex < pointCount; ++pointIndex )
			{
				b3ManifoldConstraintPoint* cp = constraint->points + pointIndex;
				b3ManifoldPoint* mp = manifold->points + pointIndex;

				cp->rA = mp->anchorA;
				cp->rB = mp->anchorB;

				b3f s = mp->separation;
				cp->baseSeparation = b3SubF( s, b3DotDirF( normal, b3Sub( cp->rB, cp->rA ) ) );
				cp->normalImpulse = b3MulImpC( mp->normalImpulse, warmStartScale );
				cp->totalNormalImpulse = b3imp_zero;

				b3Vec3 rA = cp->rA;
				b3Vec3 rB = cp->rB;

				// kNormal = mA + mB + dot( rnA, iA*rnA ) + dot( rnB, iB*rnB ).
				// Every term is an inverse mass, so the sum is Q24 and its
				// reciprocal -- an effective mass -- is Q12.
				//
				// Stage 6: `b3AddW( mA, mB )` wrapped here, and this is the site
				// where it hurt most. B3_MIN_MASS_RAW caps a *single* inverse
				// mass at ~124 against Q7.24's ceiling of 128, so one body always
				// fits -- and two dynamic bodies under about 16 g each never do.
				// A negative normal mass inverts the contact: measured on a pair
				// of 7 g spheres, the two were driven to x = 3862 in 600 steps
				// instead of resting. Two static bodies cannot reach it, which is
				// why nothing in the suite had caught it.
				//
				// b3LeverInertiaSumWide sums both masses and both quadratic forms
				// in one int64, and b3RcpWide delegates to b3RcpW inside b3iw's
				// range, so every correct result is bit-identical.
				//
				// Zero is still reachable: a contact whose bodies are both static
				// reaches here through the graph, and b3RcpWide reads a zero
				// effective mass as "apply no impulse", which is right.
				b3Vec3 rnA = b3CrossDirRight( rA, normal );
				b3Vec3 rnB = b3CrossDirRight( rB, normal );
				cp->normalMass = b3RcpWide( b3LeverInertiaSumWide( mA, iA, rnA, mB, iB, rnB ) );

				b3Vec3 vrA = b3Add( vA, b3Cross( wA, rA ) );
				b3Vec3 vrB = b3Add( vB, b3Cross( wB, rB ) );
				cp->relativeVelocity = b3DotDirF( normal, b3Sub( vrB, vrA ) );

				// C0 friction-centre decay. A point further than twice the
				// speculative distance only matters for CCD and must not pull
				// the friction centre towards itself; closer points come and
				// go, so the weight ramps rather than switching.
				//
				// `2 - s/tau` exceeds 1 whenever the point is penetrating, so
				// the intermediate is b3f and the clamp to one is doing real
				// work -- computing it at b3c would saturate before the clamp.
				b3f weight = b3SubF( b3AddF( b3f_one, b3f_one ), b3MulFF( s, invTau ) );
				weight = b3ClampF( weight, b3CToF( B3_MIN_FRICTION_WEIGHT ), b3f_one );

				int64_t w = b3Raw( weight );
				centerAx += w * b3Raw( rA.x );
				centerAy += w * b3Raw( rA.y );
				centerAz += w * b3Raw( rA.z );
				centerBx += w * b3Raw( rB.x );
				centerBy += w * b3Raw( rB.y );
				centerBz += w * b3Raw( rB.z );
				totalFrictionWeight += w;
			}

			// The weights are clamped at one Q30 quantum each and pointCount is
			// at least one, so this cannot be zero -- but the divide is guarded
			// anyway, because a manifold with no points would otherwise trap.
			b3Vec3 centerA = b3Vec3_zero;
			b3Vec3 centerB = b3Vec3_zero;
			if ( totalFrictionWeight != 0 )
			{
				centerA = b3MakeVec3( b3Makeb3f( b3HwDiv64( centerAx, (int32_t)totalFrictionWeight ) ),
									  b3Makeb3f( b3HwDiv64( centerAy, (int32_t)totalFrictionWeight ) ),
									  b3Makeb3f( b3HwDiv64( centerAz, (int32_t)totalFrictionWeight ) ) );
				centerB = b3MakeVec3( b3Makeb3f( b3HwDiv64( centerBx, (int32_t)totalFrictionWeight ) ),
									  b3Makeb3f( b3HwDiv64( centerBy, (int32_t)totalFrictionWeight ) ),
									  b3Makeb3f( b3HwDiv64( centerBz, (int32_t)totalFrictionWeight ) ) );
			}

			constraint->centerA = centerA;
			constraint->centerB = centerB;

			for ( int pointIndex = 0; pointIndex < pointCount; ++pointIndex )
			{
				b3ManifoldConstraintPoint* cp = constraint->points + pointIndex;
				cp->leverArm = b3Distance( cp->rA, centerA );
			}

			// -----------------------------------------------------------
			// Central friction: the 2x2 tangent mass and the twist mass
			// -----------------------------------------------------------

			b3Vec3 rtA1 = b3CrossDirRight( centerA, tangent1 );
			b3Vec3 rtA2 = b3CrossDirRight( centerA, tangent2 );
			b3Vec3 rtB1 = b3CrossDirRight( centerB, tangent1 );
			b3Vec3 rtB2 = b3CrossDirRight( centerB, tangent2 );

			{
				// The same two-lever-arm 2x2 that a prismatic joint's
				// perpendicular block is, so it goes through the same routine.
				//
				// Stage 6: this carried `b3AddW( mA, mB )` on both diagonals --
				// kNormal's wrap again, in the friction block.
				// b3InvertPointLineMass forms that sum in the int64 it accumulates
				// K in, and reduces to b3InvertSym2W on the identical entries
				// whenever they already fit, so a correct tangent mass is
				// bit-identical.
				constraint->tangentMass = b3InvertPointLineMass( mA, iA, rtA1, rtA2, mB, iB, rtB1, rtB2 );

				// The stored friction impulse is a world vector, and the frame
				// it was written in may have been rebuilt since. Projecting it
				// onto the current tangents is what carries it across.
				constraint->frictionImpulse.x =
					b3MulImpC( b3DotImpN( manifold->frictionImpulse, tangent1 ), warmStartScale );
				constraint->frictionImpulse.y =
					b3MulImpC( b3DotImpN( manifold->frictionImpulse, tangent2 ), warmStartScale );
			}

			{
				// `normal . (iA + iB) . normal`, with the sum never formed -- the
				// scalar counterpart of rollingMass above, and the same argument:
				// each matrix is applied to the axis separately at Q12, where 256
				// is nothing against the 524287 the scale holds, and only the
				// quadratic form accumulates wide. b3RcpWide delegates to b3RcpW
				// inside b3iw's range, so the correct cases are bit-identical.
				b3Vec3 normalV = b3FromDir3( normal );
				constraint->twistMass = b3RcpWide( b3AxisInertiaSumWide( normalV, iA, iB ) );
				constraint->twistImpulse = b3MulImpC( manifold->twistImpulse, warmStartScale );
			}

			constraint->rollingImpulse = b3MulCImp3( warmStartScale, manifold->rollingImpulse );
		}
	}
}

// =========================================================================
// Warm start
// =========================================================================

/// @note ITCM group B3_ITCM_CONTACTS -- see nea_config.h.
void B3_ITCM_IF( B3_ITCM_CONTACTS, b3WarmStartContacts )( int startIndex, int endIndex, b3StepContext* context )
{
	b3World* world = context->world;
	b3GraphColor* color = world->constraintGraph.colors + B3_OVERFLOW_INDEX;
	b3ContactConstraint* constraints = color->contactConstraints;
	b3BodyState* states = context->states;

	// A static body has no solver state, so it borrows one that never moves and
	// is never written back.
	b3BodyState dummyState = b3_identityBodyState;

	for ( int constraintIndex = startIndex; constraintIndex < endIndex; ++constraintIndex )
	{
		const b3ContactConstraint* contactConstraint = constraints + constraintIndex;
		int indexA = contactConstraint->indexA;
		int indexB = contactConstraint->indexB;

		b3BodyState* stateA = indexA == B3_NULL_INDEX ? &dummyState : states + indexA;
		b3BodyState* stateB = indexB == B3_NULL_INDEX ? &dummyState : states + indexB;

		b3Vec3 vA = stateA->linearVelocity;
		b3Vec3 wA = stateA->angularVelocity;
		b3Vec3 vB = stateB->linearVelocity;
		b3Vec3 wB = stateB->angularVelocity;

		b3iw mA = contactConstraint->invMassA;
		b3MatrixW iA = contactConstraint->invIA;
		b3iw mB = contactConstraint->invMassB;
		b3MatrixW iB = contactConstraint->invIB;

		int manifoldCount = contactConstraint->manifoldCount;
		for ( int manifoldIndex = 0; manifoldIndex < manifoldCount; ++manifoldIndex )
		{
			b3ManifoldConstraint* constraint = contactConstraint->constraints + manifoldIndex;

			b3Dir3 normal = constraint->normal;
			int pointCount = constraint->pointCount;

			for ( int j = 0; j < pointCount; ++j )
			{
				const b3ManifoldConstraintPoint* cp = constraint->points + j;

				b3Vec3 rA = cp->rA;
				b3Vec3 rB = cp->rB;

				b3Imp3 impulse = b3MulNImp( cp->normalImpulse, normal );
				wA = b3Sub( wA, b3MulMWImp( iA, b3CrossVImp( rA, impulse ) ) );
				vA = b3Sub( vA, b3MulImpW3( impulse, mA ) );
				wB = b3Add( wB, b3MulMWImp( iB, b3CrossVImp( rB, impulse ) ) );
				vB = b3Add( vB, b3MulImpW3( impulse, mB ) );
			}

			// Central friction, acting at the friction centres.
			{
				b3Vec3 rA = constraint->centerA;
				b3Vec3 rB = constraint->centerB;
				b3Imp3 impulse = b3BlendImp2( constraint->frictionImpulse.x, constraint->tangent1,
											  constraint->frictionImpulse.y, constraint->tangent2 );

				wA = b3Sub( wA, b3MulMWImp( iA, b3CrossVImp( rA, impulse ) ) );
				vA = b3Sub( vA, b3MulImpW3( impulse, mA ) );
				wB = b3Add( wB, b3MulMWImp( iB, b3CrossVImp( rB, impulse ) ) );
				vB = b3Add( vB, b3MulImpW3( impulse, mB ) );
			}

			// Twist and rolling are pure couples -- no linear term.
			{
				b3Imp3 impulse = b3MulNImp( constraint->twistImpulse, normal );
				wA = b3Sub( wA, b3MulMWImp( iA, impulse ) );
				wB = b3Add( wB, b3MulMWImp( iB, impulse ) );
			}

			{
				b3Imp3 impulse = constraint->rollingImpulse;
				wA = b3Sub( wA, b3MulMWImp( iA, impulse ) );
				wB = b3Add( wB, b3MulMWImp( iB, impulse ) );
			}
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
}

// =========================================================================
// Solve
// =========================================================================
//
// Upstream merged the normal and friction loops here, which it notes is much
// more stable for a Jenga stack, and the merge is kept: friction is bounded by
// the normal impulse computed in the same pass, so splitting them means
// friction always trails the normal force by an iteration.

/// @note ITCM group B3_ITCM_CONTACTS -- see nea_config.h.
void B3_ITCM_IF( B3_ITCM_CONTACTS, b3SolveContacts )( int startIndex, int endIndex, b3StepContext* context, bool useBias )
{
	b3World* world = context->world;
	b3GraphColor* color = world->constraintGraph.colors + B3_OVERFLOW_INDEX;
	b3ContactConstraint* contactConstraints = color->contactConstraints;
	b3BodyState* states = context->states;

	b3BodyState dummyState = b3_identityBodyState;

	b3f inv_h = context->inv_h;
	b3f contactSpeed = world->contactSpeed;

	for ( int i = startIndex; i < endIndex; ++i )
	{
		b3ContactConstraint* contactConstraint = contactConstraints + i;
		int manifoldCount = contactConstraint->manifoldCount;

		int indexA = contactConstraint->indexA;
		int indexB = contactConstraint->indexB;

		b3iw mA = contactConstraint->invMassA;
		b3MatrixW iA = contactConstraint->invIA;
		b3iw mB = contactConstraint->invMassB;
		b3MatrixW iB = contactConstraint->invIB;

		b3BodyState* stateA = indexA == B3_NULL_INDEX ? &dummyState : states + indexA;
		b3Vec3 vA = stateA->linearVelocity;
		b3Vec3 wA = stateA->angularVelocity;
		b3Quat dqA = stateA->deltaRotation;

		b3BodyState* stateB = indexB == B3_NULL_INDEX ? &dummyState : states + indexB;
		b3Vec3 vB = stateB->linearVelocity;
		b3Vec3 wB = stateB->angularVelocity;
		b3Quat dqB = stateB->deltaRotation;

		// deltaPosition is Q24 since the position integrator was fixed, so the
		// difference is taken at Q24 and narrowed **once**. Narrowing each side
		// first would round twice on a quantity that is a difference of two
		// nearly equal numbers -- and for a resting stack the two are equal to
		// the quantum, which is exactly when the second rounding would matter.
		b3Vec3 dp = b3W3ToVec3( b3SubW3( stateB->deltaPosition, stateA->deltaPosition ) );

		b3Softness softness = contactConstraint->softness;
		b3c friction = contactConstraint->friction;
		b3f rollingResistance = contactConstraint->rollingResistance;

		for ( int j = 0; j < manifoldCount; ++j )
		{
			b3ManifoldConstraint* constraint = contactConstraint->constraints + j;

			int pointCount = constraint->pointCount;
			b3Dir3 normal = constraint->normal;

			b3imp totalNormalImpulse = b3imp_zero;
			b3imp totalTwistLimit = b3imp_zero;

			for ( int pointIndex = 0; pointIndex < pointCount; ++pointIndex )
			{
				b3ManifoldConstraintPoint* cp = constraint->points + pointIndex;

				b3Vec3 rA = cp->rA;
				b3Vec3 rB = cp->rB;

				// Current separation, with the normal held constant over the
				// step. Round-off here grows with the distance from the centre
				// of mass, which is upstream's caveat and the port's too.
				b3Vec3 ds = b3Add( dp, b3Sub( b3RotateVector( dqB, rB ), b3RotateVector( dqA, rA ) ) );
				b3f s = b3AddF( b3DotDirF( normal, ds ), cp->baseSeparation );

				b3f velocityBias = b3f_zero;
				b3c massScale = b3c_one;
				b3c impulseScale = b3c_zero;

				if ( b3Raw( s ) > 0 )
				{
					// Speculative: the point is still separated, so the
					// constraint may only stop it closing faster than it can
					// close in one sub-step.
					velocityBias = b3MulFF( s, inv_h );
				}
				else if ( useBias )
				{
					velocityBias = b3MaxF( b3MulFF( b3MulFC( softness.biasRate, softness.massScale ), s ),
										   b3NegF( contactSpeed ) );
					massScale = softness.massScale;
					impulseScale = softness.impulseScale;
				}

				b3Vec3 vrA = b3Add( vA, b3Cross( wA, rA ) );
				b3Vec3 vrB = b3Add( vB, b3Cross( wB, rB ) );
				b3f vn = b3DotDirF( normal, b3Sub( vrB, vrA ) );

				// mass * velocity -> impulse, which is what b3MulFFToImp is
				// for, and the impulseScale term is already an impulse.
				b3imp deltaImpulse = b3SubImp( b3NegImp( b3MulFFToImp( cp->normalMass, b3AddF( b3MulFC( vn, massScale ),
																							   velocityBias ) ) ),
											   b3MulImpC( cp->normalImpulse, impulseScale ) );

				// Clamping the *accumulated* impulse rather than the increment
				// is what keeps the constraint one-sided under quantization as
				// well as under sign, so the recomputation below is not
				// redundant.
				b3imp newImpulse = b3MaxImp( b3AddImp( cp->normalImpulse, deltaImpulse ), b3imp_zero );
				deltaImpulse = b3SubImp( newImpulse, cp->normalImpulse );
				cp->normalImpulse = newImpulse;
				cp->totalNormalImpulse = b3AddImp( cp->totalNormalImpulse, newImpulse );

				totalNormalImpulse = b3AddImp( totalNormalImpulse, newImpulse );
				totalTwistLimit = b3AddImp( totalTwistLimit, b3MulImpF( cp->normalImpulse, cp->leverArm ) );

				b3Imp3 P = b3MulNImp( deltaImpulse, normal );
				vA = b3Sub( vA, b3MulImpW3( P, mA ) );
				wA = b3Sub( wA, b3MulMWImp( iA, b3CrossVImp( rA, P ) ) );
				vB = b3Add( vB, b3MulImpW3( P, mB ) );
				wB = b3Add( wB, b3MulMWImp( iB, b3CrossVImp( rB, P ) ) );
			}

			// No friction while the bias is on: a friction impulse bounded by a
			// normal impulse that includes position correction would resist
			// sliding in proportion to how deeply the bodies overlap.
			if ( useBias )
			{
				continue;
			}

			// -----------------------------------------------------------
			// Central twist friction
			// -----------------------------------------------------------
			{
				b3f twistSpeed = b3DotDirF( normal, b3Sub( wB, wA ) );
				b3imp maxImpulse = b3MulImpC( totalTwistLimit, friction );
				b3imp deltaImpulse = b3NegImp( b3MulFFToImp( constraint->twistMass, twistSpeed ) );
				b3imp oldImpulse = constraint->twistImpulse;

				constraint->twistImpulse =
					b3ClampImp( b3AddImp( oldImpulse, deltaImpulse ), b3NegImp( maxImpulse ), maxImpulse );
				deltaImpulse = b3SubImp( constraint->twistImpulse, oldImpulse );

				b3Imp3 P = b3MulNImp( deltaImpulse, normal );
				wA = b3Sub( wA, b3MulMWImp( iA, P ) );
				wB = b3Add( wB, b3MulMWImp( iB, P ) );
			}

			// -----------------------------------------------------------
			// Rolling resistance
			// -----------------------------------------------------------
			//
			// Off by default: the material coefficient is zero unless a caller
			// sets it, and this is the most expensive term in the solver.
			if ( b3Raw( rollingResistance ) > 0 )
			{
				b3Vec3 dw = b3Sub( wB, wA );
				b3Imp3 rollingDelta = b3NegImp3( b3MulMVToImp( contactConstraint->rollingMass, dw ) );
				b3Imp3 oldImpulse = constraint->rollingImpulse;
				constraint->rollingImpulse = b3AddImp3( oldImpulse, rollingDelta );

				// rollingResistance is a length, so this product is an angular
				// impulse -- which is what the accumulator holds. See
				// b3Contact::rollingResistance on why it is not a coefficient.
				b3imp maxImpulse = b3MulImpF( totalNormalImpulse, rollingResistance );

				// Compared as raw squares, never through a 32-bit type.
				// Upstream adds FLT_EPSILON to break the tie; there is no such
				// constant here and the comparison is simply strict.
				int64_t magSqr = b3Imp3LengthSquaredWide( constraint->rollingImpulse );
				int64_t maxSqr = (int64_t)b3Raw( maxImpulse ) * b3Raw( maxImpulse );
				if ( magSqr > maxSqr )
				{
					b3imp mag = b3SqrtWideImp( magSqr );
					if ( b3Raw( mag ) != 0 )
					{
						b3c scale = b3DivFFToC( b3ImpToF( maxImpulse ), b3ImpToF( mag ) );
						constraint->rollingImpulse = b3MulCImp3( scale, constraint->rollingImpulse );
					}
				}

				b3Imp3 P = b3SubImp3( constraint->rollingImpulse, oldImpulse );
				wA = b3Sub( wA, b3MulMWImp( iA, P ) );
				wB = b3Add( wB, b3MulMWImp( iB, P ) );
			}

			// -----------------------------------------------------------
			// Central friction
			// -----------------------------------------------------------
			{
				b3Dir3 tangent1 = constraint->tangent1;
				b3Dir3 tangent2 = constraint->tangent2;

				b3Vec3 rA = constraint->centerA;
				b3Vec3 rB = constraint->centerB;

				b3Vec3 vrA = b3Add( vA, b3Cross( wA, rA ) );
				b3Vec3 vrB = b3Add( vB, b3Cross( wB, rB ) );
				b3Vec3 vr = b3Sub( vrB, vrA );

				b3f vt1 = b3SubF( b3DotDirF( tangent1, vr ), constraint->tangentVelocity1 );
				b3f vt2 = b3SubF( b3DotDirF( tangent2, vr ), constraint->tangentVelocity2 );

				b3Imp2 tm = b3MulSym2V( constraint->tangentMass, vt1, vt2 );

				b3Imp2 newImpulse = { b3SubImp( constraint->frictionImpulse.x, tm.x ),
									  b3SubImp( constraint->frictionImpulse.y, tm.y ) };

				b3imp maxImpulse = b3MulImpC( totalNormalImpulse, friction );

				int64_t lengthSquared =
					(int64_t)b3Raw( newImpulse.x ) * b3Raw( newImpulse.x ) + (int64_t)b3Raw( newImpulse.y ) * b3Raw( newImpulse.y );
				int64_t maxSqr = (int64_t)b3Raw( maxImpulse ) * b3Raw( maxImpulse );
				if ( lengthSquared > maxSqr )
				{
					b3imp mag = b3SqrtWideImp( lengthSquared );
					if ( b3Raw( mag ) != 0 )
					{
						b3c scale = b3DivFFToC( b3ImpToF( maxImpulse ), b3ImpToF( mag ) );
						newImpulse.x = b3MulImpC( newImpulse.x, scale );
						newImpulse.y = b3MulImpC( newImpulse.y, scale );
					}
				}

				b3Imp2 deltaImpulse = { b3SubImp( newImpulse.x, constraint->frictionImpulse.x ),
										b3SubImp( newImpulse.y, constraint->frictionImpulse.y ) };
				constraint->frictionImpulse = newImpulse;

				b3Imp3 P = b3BlendImp2( deltaImpulse.x, tangent1, deltaImpulse.y, tangent2 );
				vA = b3Sub( vA, b3MulImpW3( P, mA ) );
				wA = b3Sub( wA, b3MulMWImp( iA, b3CrossVImp( rA, P ) ) );
				vB = b3Add( vB, b3MulImpW3( P, mB ) );
				wB = b3Add( wB, b3MulMWImp( iB, b3CrossVImp( rB, P ) ) );
			}
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
}

// =========================================================================
// Restitution
// =========================================================================

void b3ApplyRestitution( int startIndex, int endIndex, b3StepContext* context )
{
	b3World* world = context->world;
	b3GraphColor* color = world->constraintGraph.colors + B3_OVERFLOW_INDEX;
	b3ContactConstraint* constraints = color->contactConstraints;
	b3BodyState* states = context->states;

	b3BodyState dummyState = b3_identityBodyState;
	b3f threshold = world->restitutionThreshold;

	for ( int constraintIndex = startIndex; constraintIndex < endIndex; ++constraintIndex )
	{
		const b3ContactConstraint* contactConstraint = constraints + constraintIndex;
		b3c restitution = contactConstraint->restitution;
		if ( b3Raw( restitution ) == 0 )
		{
			continue;
		}

		int indexA = contactConstraint->indexA;
		int indexB = contactConstraint->indexB;

		b3BodyState* stateA = indexA == B3_NULL_INDEX ? &dummyState : states + indexA;
		b3BodyState* stateB = indexB == B3_NULL_INDEX ? &dummyState : states + indexB;

		b3Vec3 vA = stateA->linearVelocity;
		b3Vec3 wA = stateA->angularVelocity;
		b3Vec3 vB = stateB->linearVelocity;
		b3Vec3 wB = stateB->angularVelocity;

		b3iw mA = contactConstraint->invMassA;
		b3MatrixW iA = contactConstraint->invIA;
		b3iw mB = contactConstraint->invMassB;
		b3MatrixW iB = contactConstraint->invIB;

		int manifoldCount = contactConstraint->manifoldCount;
		for ( int manifoldIndex = 0; manifoldIndex < manifoldCount; ++manifoldIndex )
		{
			b3ManifoldConstraint* cm = contactConstraint->constraints + manifoldIndex;

			b3Dir3 normal = cm->normal;
			int pointCount = cm->pointCount;
			B3_ASSERT( 0 < pointCount && pointCount <= B3_MAX_MANIFOLD_POINTS );

			for ( int pointIndex = 0; pointIndex < pointCount; ++pointIndex )
			{
				b3ManifoldConstraintPoint* cp = cm->points + pointIndex;

				// Two ways a point earns no restitution: it was not approaching
				// fast enough to bounce, or it never actually collided. The
				// second test is the total rather than the current impulse,
				// because a point can collide early in the step and separate
				// again before the last sub-step.
				if ( b3Raw( cp->relativeVelocity ) > -b3Raw( threshold ) || b3Raw( cp->totalNormalImpulse ) == 0 )
				{
					continue;
				}

				b3Vec3 rA = cp->rA;
				b3Vec3 rB = cp->rB;

				b3Vec3 vrA = b3Add( vA, b3Cross( wA, rA ) );
				b3Vec3 vrB = b3Add( vB, b3Cross( wB, rB ) );
				b3f vn = b3DotDirF( normal, b3Sub( vrB, vrA ) );

				b3imp impulse =
					b3NegImp( b3MulFFToImp( cp->normalMass, b3AddF( vn, b3MulFC( cp->relativeVelocity, restitution ) ) ) );

				b3imp newImpulse = b3MaxImp( b3AddImp( cp->normalImpulse, impulse ), b3imp_zero );
				impulse = b3SubImp( newImpulse, cp->normalImpulse );
				cp->normalImpulse = newImpulse;
				cp->totalNormalImpulse = b3AddImp( cp->totalNormalImpulse, impulse );

				b3Imp3 P = b3MulNImp( impulse, normal );
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
	}
}

// =========================================================================
// Store impulses
// =========================================================================
//
// Writes the accumulators back onto the manifold, which is what makes the next
// step's warm start possible, and flags the contacts that produced a hit event.

void b3StoreImpulses( int startIndex, int endIndex, b3StepContext* context )
{
	b3World* world = context->world;
	b3GraphColor* color = world->constraintGraph.colors + B3_OVERFLOW_INDEX;
	b3ContactConstraint* base = color->contactConstraints;
	b3ContactSpec* specs = color->contacts.data;

	b3BitSet* hitEventBitSet = &world->hitEventBitSet;
	bool hasHitEvents = world->hasHitEvents;
	b3f negHitThreshold = b3NegF( world->hitEventThreshold );

	for ( int index = startIndex; index < endIndex; ++index )
	{
		b3ContactConstraint* contactConstraint = base + index;

		b3Contact* contact = contactConstraint->contact;
		B3_ASSERT( contact != NULL );

		// Catches a prepare/store index mismatch, which is what upstream's span
		// validation existed to prevent. Cheap, and the failure it guards
		// against would be a contact warm-starting from another contact's
		// impulses.
		B3_ASSERT( contact->contactId == specs[index].contactId );
		B3_UNUSED( specs );

		int manifoldCount = contactConstraint->manifoldCount;
		B3_ASSERT( manifoldCount == contact->manifoldCount );

		bool checkHitEvents = ( contact->flags & b3_simEnableHitEvent ) != 0;
		bool flagged = false;

		for ( int manifoldIndex = 0; manifoldIndex < manifoldCount; ++manifoldIndex )
		{
			b3Manifold* manifold = contact->manifolds + manifoldIndex;
			b3ManifoldConstraint* constraint = contactConstraint->constraints + manifoldIndex;

			manifold->twistImpulse = constraint->twistImpulse;
			manifold->frictionImpulse = b3BlendImp2( constraint->frictionImpulse.x, constraint->tangent1,
													 constraint->frictionImpulse.y, constraint->tangent2 );
			manifold->rollingImpulse = constraint->rollingImpulse;

			int count = constraint->pointCount;
			B3_ASSERT( count == manifold->pointCount );

			for ( int pointIndex = 0; pointIndex < count; ++pointIndex )
			{
				b3ManifoldConstraintPoint* cp = constraint->points + pointIndex;
				b3ManifoldPoint* mp = manifold->points + pointIndex;

				mp->normalImpulse = cp->normalImpulse;
				mp->totalNormalImpulse = cp->totalNormalImpulse;
				mp->normalVelocity = cp->relativeVelocity;

				if ( checkHitEvents && flagged == false && b3Raw( mp->normalVelocity ) < b3Raw( negHitThreshold ) &&
					 b3Raw( mp->totalNormalImpulse ) > 0 )
				{
					b3SetBit( hitEventBitSet, (uint32_t)contact->contactId );
					hasHitEvents = true;
					flagged = true;
				}
			}
		}
	}

	world->hasHitEvents = hasHitEvents;
}
