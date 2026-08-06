// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Erin Catto        (original Box3D)
// Copyright (c) 2026 Warioware64       (Nitro Engine Advanced fixed-point port)
//
// This file is part of Nitro Engine Advanced

/// @file   solver.c
/// @brief  The step: integrate, solve, integrate, finalize.
///
/// @section shape What this file is, once the threading is gone
///
/// Upstream's solver.c is 2331 lines, and roughly 900 of them are a
/// work-stealing scheduler: block descriptors, per-block atomic sync indices,
/// stage tables, worker start offsets, and a `b3SolverTask` that orchestrates
/// all of it. On one core every one of those becomes a `for` loop.
///
/// A second collapse is larger and less obvious. Upstream's per-colour loops
/// are written `for ( i < B3_GRAPH_COLOR_COUNT - 1 )`, and B3_GRAPH_COLOR_COUNT
/// is 1 here -- so they iterate **zero** times, `activeColorCount` is always
/// zero, and every constraint lives in the overflow colour. About 500 of
/// b3Solve's 550 setup lines exist only to describe per-colour blocks that
/// cannot exist. 3A found the same thing in constraint_graph.c.
///
/// @section scope Phase 3C-i solves nothing
///
/// The constraint stages are present and empty. Contacts are created, updated
/// and validated by 3B, and have no effect -- two bodies that meet will pass
/// through each other. That is deliberate: sub-stepping, damping, the speed
/// caps, the sleep trigger and the move-event bookkeeping are each checkable
/// against a closed form, and checking them before any impulse exists is the
/// point of the split. Phase 3C-ii fills the stages in.
///
/// `b3SolveContinuous` and `b3ContinuousQueryCallback` arrived in Phase 7 Stage
/// 2. `b3BulletBodyTask` did not: it exists upstream to hand a stack-allocated
/// array of bullet indices to a worker pool, and on one core the second pass is
/// a plain loop that re-tests the flag. Stage 3 added the sensor half of the
/// continuous pass, which upstream also routes through a per-worker array and
/// which for the same reason goes straight onto the sensor here.

#include "solver.h"

#include "body.h"
#include "broad_phase.h"
#include "constraint_graph.h"
#include "contact.h"
#include "contact_solver.h"
#include "core.h"
#include "ctz.h"
#include "island.h"
#include "joint.h"
#include "physics_world.h"
#include "sensor.h"
#include "shape.h"
#include "solver_set.h"

#include "box3d/constants.h"

#include <string.h>

// =========================================================================
// Integration
// =========================================================================

static void b3IntegrateVelocitiesTask( int startIndex, int endIndex, b3StepContext* context )
{
	b3BodyState* states = context->states;
	b3BodySim* sims = context->sims;

	b3Vec3 gravity = context->world->gravity;
	b3t h = context->h;

	for ( int i = startIndex; i < endIndex; ++i )
	{
		b3BodySim* sim = sims + i;
		b3BodyState* state = states + i;

		b3Vec3 v = state->linearVelocity;
		b3Vec3 w = state->angularVelocity;

		// Damping is a Pade approximant to exp(-c*h), not the exponential
		// itself:
		//   dv/dt + c*v = 0  =>  v(t+h) = v(t) * exp(-c*h) ~= v(t) / (1 + c*h)
		// Both factors are dimensionless and in (0,1], so Q30 -- which is also
		// what makes them exact at c = 0, the overwhelmingly common case.
		b3c linearDamping = b3DivFFToC( b3f_one, b3AddF( b3f_one, b3MulFT( sim->linearDamping, h ) ) );
		b3c angularDamping = b3DivFFToC( b3f_one, b3AddF( b3f_one, b3MulFT( sim->angularDamping, h ) ) );

		// Kinematic bodies have zero inverse mass and must not fall.
		b3f gravityScale = b3Raw( sim->invMass ) > 0 ? sim->gravityScale : b3f_zero;

		// Acceleration first, then the sub-step. Forming (h * invMass) first
		// would be a Q24 x Q24 product of two small numbers -- 1/240 times
		// 1/1000 is 4.2e-6, which is seventy raw units at Q24 and throws away
		// four bits before the force is even applied.
		b3Vec3 linearAccel = b3MulWV( sim->invMass, sim->force );
		b3Vec3 gravityAccel = b3MulSV( gravityScale, gravity );
		b3Vec3 dv = b3MulVT( b3Add( linearAccel, gravityAccel ), h );

		v = b3Add( dv, b3MulCV( linearDamping, v ) );

		b3Vec3 angularAccel = b3MulMWV( sim->invInertiaWorld, sim->torque );
		b3Vec3 dw = b3MulVT( angularAccel, h );

		w = b3Add( dw, b3MulCV( angularDamping, w ) );

#if B3_GYROSCOPIC_ITERATIONS > 0
		// Gyroscopic torque, by Newton-Raphson on
		//     I*(w2 - w1) + h * cross(w2, I*w2) = 0
		// solved in local coordinates where the Jacobian is tractable. This is
		// what keeps a long skinny body tumbling correctly rather than spinning
		// about a false axis.
		//
		// Two things make it portable, and neither is obvious:
		//
		// 1. The equation is **homogeneous of degree one in I**. Scaling I by
		//    any positive constant scales every term equally and leaves w2
		//    unchanged. So the port may use the inertia at whatever scale it
		//    can actually represent -- and 3A's uniform Q7.24 clamp on the
		//    inverse inertia, which would otherwise bias the answer, cancels
		//    exactly instead.
		// 2. That is also what keeps b3InvertMatrix and b3Solve3 in range: a
		//    determinant is cubic in the entries, so it is only the fact that
		//    these entries are of order one that makes Q24 enough.
		//
		// It is nevertheless off by default -- B3_GYROSCOPIC_ITERATIONS is 0 in
		// nea_config.h. One iteration is a 3x3 inverse plus a 3x3 solve, two
		// divides and about forty multiplies, per body per sub-step. At four
		// sub-steps and 60 Hz that is the most expensive per-body operation in
		// the engine, and it buys accuracy only for bodies spinning about a
		// non-principal axis. A game that wants it can raise the count.
		{
			b3Quat q = b3MulQuat( state->deltaRotation, sim->transform.q );

			// Q24 inverse inertia narrowed to Q12 for the inverse. Legitimate
			// only because of the homogeneity above: this is the inertia up to
			// an unknown positive factor, which is all the solve needs.
			const b3MatrixW* iw = &sim->invInertiaLocal;
			b3Matrix3 invInertia =
				b3MakeMatrix3( b3MakeVec3( b3WToF( iw->cx.x ), b3WToF( iw->cx.y ), b3WToF( iw->cx.z ) ),
							   b3MakeVec3( b3WToF( iw->cy.x ), b3WToF( iw->cy.y ), b3WToF( iw->cy.z ) ),
							   b3MakeVec3( b3WToF( iw->cz.x ), b3WToF( iw->cz.y ), b3WToF( iw->cz.z ) ) );
			b3Matrix3 inertiaLocal = b3InvertMatrix( invInertia );

			b3Vec3 omega1 = b3InvRotateVector( q, w );
			b3Vec3 omega2 = omega1;

			// Symmetric, so six unique entries.
			b3f i00 = inertiaLocal.cx.x, i01 = inertiaLocal.cy.x, i02 = inertiaLocal.cz.x;
			b3f i11 = inertiaLocal.cy.y, i12 = inertiaLocal.cz.y, i22 = inertiaLocal.cz.z;

			// Narrowing h to Q12 here would cost the same 0.39% b3MulVT's
			// comment describes, so every h in the iteration is a b3MulFT.
			#define GYRO_MUL_H( x ) b3MulFT( ( x ), h )

			for ( int gyroIteration = 0; gyroIteration < B3_GYROSCOPIC_ITERATIONS; ++gyroIteration )
			{
				b3f w1 = omega2.x, w2 = omega2.y, w3 = omega2.z;

				// Iw = I * omega2, shared by the residual and the Jacobian.
				b3f Iw1 = b3AddF( b3AddF( b3MulFF( i00, w1 ), b3MulFF( i01, w2 ) ), b3MulFF( i02, w3 ) );
				b3f Iw2 = b3AddF( b3AddF( b3MulFF( i01, w1 ), b3MulFF( i11, w2 ) ), b3MulFF( i12, w3 ) );
				b3f Iw3 = b3AddF( b3AddF( b3MulFF( i02, w1 ), b3MulFF( i12, w2 ) ), b3MulFF( i22, w3 ) );

				b3Vec3 dwv = b3Sub( omega2, omega1 );

				b3Vec3 b = b3MakeVec3(
					b3AddF( b3AddF( b3AddF( b3MulFF( i00, dwv.x ), b3MulFF( i01, dwv.y ) ), b3MulFF( i02, dwv.z ) ),
							GYRO_MUL_H( b3SubF( b3MulFF( w2, Iw3 ), b3MulFF( w3, Iw2 ) ) ) ),
					b3AddF( b3AddF( b3AddF( b3MulFF( i01, dwv.x ), b3MulFF( i11, dwv.y ) ), b3MulFF( i12, dwv.z ) ),
							GYRO_MUL_H( b3SubF( b3MulFF( w3, Iw1 ), b3MulFF( w1, Iw3 ) ) ) ),
					b3AddF( b3AddF( b3AddF( b3MulFF( i02, dwv.x ), b3MulFF( i12, dwv.y ) ), b3MulFF( i22, dwv.z ) ),
							GYRO_MUL_H( b3SubF( b3MulFF( w1, Iw2 ), b3MulFF( w2, Iw1 ) ) ) ) );

				// J = I + h * (skew(omega2) * I - skew(I * omega2)). Upstream's
				// derivation; the doubled inertia terms fold into Iw.
				b3Matrix3 J = b3MakeMatrix3(
					b3MakeVec3(
						b3AddF( i00, GYRO_MUL_H( b3SubF( b3MulFF( w2, i02 ), b3MulFF( w3, i01 ) ) ) ),
						b3AddF( i01, GYRO_MUL_H( b3SubF( b3SubF( b3MulFF( w3, i00 ), b3MulFF( w1, i02 ) ), Iw3 ) ) ),
						b3AddF( i02, GYRO_MUL_H( b3AddF( b3SubF( b3MulFF( w1, i01 ), b3MulFF( w2, i00 ) ), Iw2 ) ) ) ),
					b3MakeVec3(
						b3AddF( i01, GYRO_MUL_H( b3AddF( b3SubF( b3MulFF( w2, i12 ), b3MulFF( w3, i11 ) ), Iw3 ) ) ),
						b3AddF( i11, GYRO_MUL_H( b3SubF( b3MulFF( w3, i01 ), b3MulFF( w1, i12 ) ) ) ),
						b3AddF( i12, GYRO_MUL_H( b3SubF( b3SubF( b3MulFF( w1, i11 ), b3MulFF( w2, i01 ) ), Iw1 ) ) ) ),
					b3MakeVec3(
						b3AddF( i02, GYRO_MUL_H( b3SubF( b3SubF( b3MulFF( w2, i22 ), b3MulFF( w3, i12 ) ), Iw2 ) ) ),
						b3AddF( i12, GYRO_MUL_H( b3AddF( b3SubF( b3MulFF( w3, i02 ), b3MulFF( w1, i22 ) ), Iw1 ) ) ),
						b3AddF( i22, GYRO_MUL_H( b3SubF( b3MulFF( w1, i12 ), b3MulFF( w2, i02 ) ) ) ) ) );

				omega2 = b3Sub( omega2, b3Solve3( J, b ) );
			}

			w = b3RotateVector( q, omega2 );
		}
#endif

		state->linearVelocity = v;
		state->angularVelocity = w;
	}
}

static void b3IntegratePositionsTask( int startIndex, int endIndex, b3StepContext* context )
{
	b3BodyState* states = context->states;
	b3t h = context->h;

	b3f maxLinearSpeed = context->maxLinearVelocity;

	// B3_MAX_ROTATION is a b3a -- brads, 32768 to a *full circle* -- so this is
	// a unit conversion, not a multiply. The division is by the whole circle,
	// giving revolutions, and 2*pi then turns revolutions into radians:
	// 4096/32768 = 0.125 rev = pi/4 rad. At 60 Hz maxAngularSpeed is 47 rad/s.
	//
	// Dividing by half the circle instead reads as "brads per half turn" and
	// silently doubles the cap, which lets a body rotate pi/2 in a step --
	// exactly what B3_MAX_ROTATION's comment warns breaks continuous collision.
	b3f maxAngularSpeed = b3MulFF( b3fFromFrac( B3_MAX_ROTATION, 32768 ), b3MulFF( B3_TWO_PI, context->inv_dt ) );

	// Both squares are compared wide. 47.1^2 is 2219, which at Q24 is 3.7e10
	// and overflows int32 by four bits -- so a Q12 threshold here would wrap
	// and the cap would fire on almost every body. Same treatment 3B's
	// recycling thresholds needed.
	int64_t maxLinearSpeedSq = (int64_t)b3Raw( maxLinearSpeed ) * b3Raw( maxLinearSpeed );
	int64_t maxAngularSpeedSq = (int64_t)b3Raw( maxAngularSpeed ) * b3Raw( maxAngularSpeed );

	for ( int i = startIndex; i < endIndex; ++i )
	{
		b3BodyState* state = states + i;

		b3Vec3 v = state->linearVelocity;
		b3Vec3 w = state->angularVelocity;

		// Motion locks read as a constraint applied last.
		uint32_t flags = state->flags;
		v.x = ( flags & b3_lockLinearX ) ? b3f_zero : v.x;
		v.y = ( flags & b3_lockLinearY ) ? b3f_zero : v.y;
		v.z = ( flags & b3_lockLinearZ ) ? b3f_zero : v.z;
		w.x = ( flags & b3_lockAngularX ) ? b3f_zero : w.x;
		w.y = ( flags & b3_lockAngularY ) ? b3f_zero : w.y;
		w.z = ( flags & b3_lockAngularZ ) ? b3f_zero : w.z;

		if ( b3LengthSquaredWide( v ) > maxLinearSpeedSq )
		{
			b3f ratio = b3DivFF( maxLinearSpeed, b3Length( v ) );
			v = b3MulSV( ratio, v );
			state->flags |= b3_isSpeedCapped;
		}

		if ( b3LengthSquaredWide( w ) > maxAngularSpeedSq && ( flags & b3_allowFastRotation ) == 0 )
		{
			b3f ratio = b3DivFF( maxAngularSpeed, b3Length( w ) );
			w = b3MulSV( ratio, w );
			state->flags |= b3_isSpeedCapped;
		}

		state->linearVelocity = v;
		state->angularVelocity = w;
		// Q24, not Q12. The increment is tens of quanta at Q12 and the velocity
		// does not change between the sub-steps of a step, so rounding it lands
		// the same way every time -- a constant bias that round-to-nearest
		// cannot cancel. b3BodyState::deltaPosition carries the derivation.
		state->deltaPosition = b3AddW3( state->deltaPosition, b3MulVTToW( v, h ) );

		// b3IntegrateRotation keeps the half-angle increment at Q30 throughout.
		// Phase 1 finding 3: at Q12 a body at 1 rad/s advances eight quanta per
		// sub-step and halving truncates to four, which put a body 35.8 degrees
		// out after ten seconds.
		state->deltaRotation = b3IntegrateRotation( state->deltaRotation, w, h );
	}
}

// =========================================================================
// Joints
// =========================================================================
//
// The plumbing landed in 3A and these loops ran over an always-empty array
// until Phase 6 Stage 1 -- kept, rather than stripped and re-added, because
// removing them would have churned every call site twice.
//
// Upstream reaches the same loops through b3PrepareJoints_Overflow and its two
// siblings, which exist to distinguish the overflow colour from the 23 solved
// in parallel. The port has one colour, so those wrappers would each be a loop
// with nothing to choose between and the bodies live here directly.

static void b3PrepareJointsTask( int startIndex, int endIndex, b3StepContext* context )
{
	b3JointSim* joints = context->graph->colors[B3_OVERFLOW_INDEX].jointSims.data;
	B3_ASSERT( startIndex == endIndex || joints != NULL );

	for ( int i = startIndex; i < endIndex; ++i )
	{
		b3PrepareJoint( joints + i, context );
	}
}

static void b3WarmStartJointsTask( int startIndex, int endIndex, b3StepContext* context )
{
	b3JointSim* joints = context->graph->colors[B3_OVERFLOW_INDEX].jointSims.data;
	B3_ASSERT( startIndex == endIndex || joints != NULL );

	for ( int i = startIndex; i < endIndex; ++i )
	{
		b3WarmStartJoint( joints + i, context );
	}
}

static void b3SolveJointsTask( int startIndex, int endIndex, b3StepContext* context, bool useBias )
{
	b3JointSim* joints = context->graph->colors[B3_OVERFLOW_INDEX].jointSims.data;
	B3_ASSERT( startIndex == endIndex || joints != NULL );

	b3World* world = context->world;
	b3BitSet* jointStateBitSet = &world->jointStateBitSet;

	for ( int i = startIndex; i < endIndex; ++i )
	{
		b3JointSim* joint = joints + i;
		b3SolveJoint( joint, context, useBias );

		// The joint-event threshold test. Only on the biased pass, so a joint is
		// measured once per sub-step rather than twice, and only while it still
		// has an unset bit -- the reaction queries are not free and a joint that
		// has already tripped cannot trip harder.
		if ( useBias == false )
		{
			continue;
		}

		if ( b3Raw( joint->forceThreshold ) >= b3Raw( B3_NO_BOUND ) &&
			 b3Raw( joint->torqueThreshold ) >= b3Raw( B3_NO_BOUND ) )
		{
			continue;
		}

		if ( b3GetBit( jointStateBitSet, (uint32_t)joint->jointId ) )
		{
			continue;
		}

		b3f force, torque;
		b3GetJointReactionScalars( world, joint, &force, &torque );

		// `>=`, so a threshold of zero reports every awake joint rather than
		// none -- upstream's behaviour, and a useful way to watch everything.
		if ( b3Raw( force ) >= b3Raw( joint->forceThreshold ) || b3Raw( torque ) >= b3Raw( joint->torqueThreshold ) )
		{
			b3SetBit( jointStateBitSet, (uint32_t)joint->jointId );
			world->hasJointEvents = true;
		}
	}
}

// =========================================================================
// Continuous collision
// =========================================================================
//
// A body that moves more than half its own smallest extent in one step can end
// the step on the far side of something thin, having never produced a contact.
// This pass sweeps such a body from where it started to where it ended, and if
// it meets anything on the way, puts it back at the moment of contact so the
// next step's narrow phase sees a normal overlap.
//
// It runs after b3FinalizeBodiesTask has written the end pose, and before the
// enlarged AABBs reach the broad phase -- so a body that gets pulled back has
// its proxy built from the pose it was pulled back to.
//
// Two passes, for the reason upstream has two: ordinary fast bodies sweep
// against static geometry only, which no other body's motion can affect, so
// they are handled inline as they are finalized. Bullets also sweep against
// kinematic and dynamic bodies, and must therefore wait until every ordinary
// fast body has been resolved, or they would collide with poses that are about
// to be corrected.

typedef struct b3ContinuousContext
{
	b3World* world;
	b3BodySim* fastBodySim;
	const b3Shape* fastShape;
	b3Sweep sweep;

	/// Where the sweep was re-centred, so world space can be recovered.
	b3Vec3 base;

	/// The earliest impact found so far, and the sweep limit for the rest.
	b3c fraction;

	/// Sensors this sweep passed through, and where along it.
	///
	/// A sensor hit is not an impact -- nothing stops -- so it cannot narrow
	/// `fraction`, and it cannot be acted on when it is found either: a solid
	/// hit discovered later may turn out to be earlier, and a body that never
	/// reached the sensor did not trip it. So they are collected here and
	/// filtered against the final fraction once the sweep is done.
	b3SensorHit sensorHits[B3_NEA_MAX_CONTINUOUS_SENSOR_HITS];
	b3c sensorFractions[B3_NEA_MAX_CONTINUOUS_SENSOR_HITS];
	int sensorCount;
} b3ContinuousContext;

/// The sweep a body traced this step, relative to `base`.
///
/// Upstream calls this b3MakeRelativeSweep and takes a b3Pos origin, which in
/// this port is the same type as b3Vec3. It is kept anyway, unlike the query
/// layer's `origin` argument which Stage 1b dropped as a no-op: here it is not
/// one. Re-centring on center0 leaves the separation function multiplying
/// coordinates that are the size of a body and a step of motion, rather than
/// the size of the world -- and those products are quadratic.
static b3Sweep b3MakeRelativeSweep( const b3BodySim* sim, b3Vec3 base )
{
	return ( b3Sweep ){
		.localCenter = sim->localCenter,
		.c1 = b3Sub( sim->center0, base ),
		.c2 = b3Sub( sim->center, base ),
		.q1 = sim->rotation0,
		.q2 = sim->transform.q,
	};
}

/// Implements b3TreeQueryCallbackFcn.
static bool b3ContinuousQueryCallback( int proxyId, uint64_t userData, void* context )
{
	B3_UNUSED( proxyId );

	int shapeId = (int)userData;
	b3ContinuousContext* continuousContext = context;

	const b3Shape* fastShape = continuousContext->fastShape;
	b3BodySim* fastBodySim = continuousContext->fastBodySim;

	// Skip the shape itself.
	if ( shapeId == fastShape->id )
	{
		return true;
	}

	b3World* world = continuousContext->world;
	b3Shape* shape = b3Array_Get( world->shapes, shapeId );

	// Skip the rest of the same body.
	if ( shape->bodyId == fastShape->bodyId )
	{
		return true;
	}

	// A sensor is swept against, but only when both ends want sensor events --
	// the same pair of flags the ordinary sensor pass tests, so a shape cannot
	// be invisible to a trigger at rest and visible to it at speed.
	bool isSensor = shape->sensorIndex != B3_NULL_INDEX;
	if ( isSensor && ( ( shape->flags & b3_enableSensorEvents ) == 0 || ( fastShape->flags & b3_enableSensorEvents ) == 0 ) )
	{
		return true;
	}

	if ( b3ShouldShapesCollide( fastShape->filter, shape->filter ) == false )
	{
		return true;
	}

	b3Body* body = b3Array_Get( world->bodies, shape->bodyId );
	b3BodySim* bodySim = b3GetBodySim( world, body );
	B3_ASSERT( body->type == b3_staticBody || ( fastBodySim->flags & b3_isBullet ) );

	// Bullets do not stop each other. Two of them would each be sweeping
	// against the other's uncorrected pose, and the answer would depend on
	// which was resolved first.
	if ( bodySim->flags & b3_isBullet )
	{
		return true;
	}

	b3Body* fastBody = b3Array_Get( world->bodies, fastBodySim->bodyId );
	if ( b3ShouldBodiesCollide( world, fastBody, body ) == false )
	{
		return true;
	}

	if ( ( shape->flags & b3_enableCustomFiltering ) != 0 || ( fastShape->flags & b3_enableCustomFiltering ) != 0 )
	{
		b3CustomFilterFcn* customFilterFcn = world->customFilterFcn;
		if ( customFilterFcn != NULL )
		{
			b3ShapeId idA = { shape->id + 1, world->worldId, shape->generation };
			b3ShapeId idB = { fastShape->id + 1, world->worldId, fastShape->generation };
			if ( customFilterFcn( idA, idB, world->customFilterContext ) == false )
			{
				return true;
			}
		}
	}

	// The struck body's own sweep. For a static body this is a fixed pose, and
	// b3GetSweepTransform's equal-rotation fast path picks that up.
	b3Sweep sweepA = b3MakeRelativeSweep( bodySim, continuousContext->base );

	b3TOIOutput output =
		b3ShapeTimeOfImpact( shape, fastShape, &sweepA, &continuousContext->sweep, continuousContext->fraction );

	world->toiDistanceIterations = b3MaxInt( world->toiDistanceIterations, output.distanceIterations );
	world->toiPushBackIterations = b3MaxInt( world->toiPushBackIterations, output.pushBackIterations );
	world->toiRootIterations = b3MaxInt( world->toiRootIterations, output.rootIterations );

	if ( isSensor )
	{
		// `<=` where the solid branch below uses `<`: a trigger touched at the
		// exact moment of a solid impact was still touched. Nothing narrows the
		// sweep here, because passing through a sensor does not stop anything.
		if ( b3Raw( output.fraction ) <= b3Raw( continuousContext->fraction ) &&
			 continuousContext->sensorCount < B3_NEA_MAX_CONTINUOUS_SENSOR_HITS )
		{
			int index = continuousContext->sensorCount;
			continuousContext->sensorHits[index] = ( b3SensorHit ){ shape->id, fastShape->id };
			continuousContext->sensorFractions[index] = output.fraction;
			continuousContext->sensorCount += 1;
		}

		return true;
	}

	// A fraction of zero means the shapes were already touching, which the
	// narrow phase handles perfectly well; only a hit strictly inside the
	// sweep is this pass's business.
	if ( 0 < b3Raw( output.fraction ) && b3Raw( output.fraction ) < b3Raw( continuousContext->fraction ) )
	{
		bool didHit = true;

		if ( ( shape->flags & b3_enablePreSolveEvents ) || ( fastShape->flags & b3_enablePreSolveEvents ) )
		{
			b3PreSolveFcn* preSolveFcn = world->preSolveFcn;
			if ( preSolveFcn != NULL )
			{
				b3ShapeId shapeIdA = { shape->id + 1, world->worldId, shape->generation };
				b3ShapeId shapeIdB = { fastShape->id + 1, world->worldId, fastShape->generation };
				b3Vec3 point = b3Add( continuousContext->base, output.point );
				didHit = preSolveFcn( shapeIdA, shapeIdB, point, output.normal, world->preSolveContext );
			}
		}

		if ( didHit )
		{
			fastBodySim->flags |= b3_hadTimeOfImpact;
			continuousContext->fraction = output.fraction;
		}
	}

	// Continue the query.
	return true;
}

/// Sweep one fast body and, if it hits something, put it back at the impact.
static void b3SolveContinuous( b3World* world, int bodySimIndex, b3StepContext* context )
{
	b3SolverSet* awakeSet = b3Array_Get( world->solverSets, b3_awakeSet );
	b3BodySim* fastBodySim = b3Array_Get( awakeSet->bodySims, bodySimIndex );
	B3_ASSERT( fastBodySim->flags & b3_isFast );

	b3Vec3 base = fastBodySim->center0;
	b3Sweep sweep = b3MakeRelativeSweep( fastBodySim, base );

	b3Transform xf2;
	xf2.q = sweep.q2;
	xf2.p = b3Sub( sweep.c2, b3RotateVector( sweep.q2, sweep.localCenter ) );

	b3DynamicTree* staticTree = world->broadPhase.trees + b3_staticBody;
	b3DynamicTree* kinematicTree = world->broadPhase.trees + b3_kinematicBody;
	b3DynamicTree* dynamicTree = world->broadPhase.trees + b3_dynamicBody;
	b3Body* fastBody = b3Array_Get( world->bodies, fastBodySim->bodyId );

	b3ContinuousContext continuousContext = { 0 };
	continuousContext.world = world;
	continuousContext.sweep = sweep;
	continuousContext.base = base;
	continuousContext.fastBodySim = fastBodySim;
	continuousContext.fraction = b3c_one;

	bool isBullet = ( fastBodySim->flags & b3_isBullet ) != 0;

	int shapeId = fastBody->headShapeId;
	while ( shapeId != B3_NULL_INDEX )
	{
		b3Shape* fastShape = b3Array_Get( world->shapes, shapeId );
		shapeId = fastShape->nextShapeId;

		continuousContext.fastShape = fastShape;

		b3AABB box1 = fastShape->aabb;

		// xf2 is relative to the base, so the box comes back to world space.
		b3AABB box2 = b3ComputeShapeAABB( fastShape, xf2 );
		box2.lowerBound = b3Add( box2.lowerBound, base );
		box2.upperBound = b3Add( box2.upperBound, base );

		// Kept, so the no-impact path below does not compute it twice.
		fastShape->aabb = box2;

		// A mesh is a triangle soup on a body that should not be moving fast
		// in the first place, and b3ShapeTimeOfImpact cannot sweep one.
		if ( fastShape->type == b3_meshShape )
		{
			continue;
		}

		// A sensor on a fast body sweeps against nothing. It has no collision
		// response to preserve, and the sensor pass will query it from its end
		// pose in a moment -- upstream's rule, and the same asymmetry the
		// callback above has: a sensor is something to be swept *against*, not
		// something that sweeps.
		if ( fastShape->sensorIndex != B3_NULL_INDEX )
		{
			continue;
		}

		b3AABB sweptBox = b3AABB_Union( box1, box2 );
		b3DynamicTree_Query( staticTree, sweptBox, B3_DEFAULT_MASK_BITS, false, b3ContinuousQueryCallback, &continuousContext );

		if ( isBullet )
		{
			b3DynamicTree_Query( kinematicTree, sweptBox, B3_DEFAULT_MASK_BITS, false, b3ContinuousQueryCallback, &continuousContext );
			b3DynamicTree_Query( dynamicTree, sweptBox, B3_DEFAULT_MASK_BITS, false, b3ContinuousQueryCallback, &continuousContext );
		}
	}

	// Deposit the sensor hits, now that the final fraction is known.
	//
	// The filter is upstream's: a sensor found beyond where the body was
	// stopped was not actually reached, so it did not happen. `<` rather than
	// `<=` because a sensor exactly at the impact was recorded by a `<=` in the
	// callback, and one of the two comparisons has to be strict or a trigger
	// sitting on a wall fires on every body that hits the wall.
	//
	// Upstream pushes these to a per-worker b3TaskContext and drains every
	// worker's array back in b3Solve, because its workers fill them
	// concurrently and the event order must not depend on which thread won.
	// There is one core here, so the hits go straight onto the sensor -- no
	// second pass, no array on the solver stack, and b3SolverStackDemand
	// unchanged. Same argument as the bullet pass below.
	for ( int i = 0; i < continuousContext.sensorCount; ++i )
	{
		if ( b3Raw( continuousContext.sensorFractions[i] ) >= b3Raw( continuousContext.fraction ) )
		{
			continue;
		}

		b3SensorHit hit = continuousContext.sensorHits[i];
		b3Shape* sensorShape = b3Array_Get( world->shapes, hit.sensorId );
		b3Shape* visitor = b3Array_Get( world->shapes, hit.visitorId );

		b3Sensor* sensor = b3Array_Get( world->sensors, sensorShape->sensorIndex );

		// Bounded like every other sensor array, and dropped the same way: the
		// sensor pass folds these into overlaps2, so a hit that does not fit is
		// a begin event that does not fire.
		if ( sensor->hits.count == sensor->hits.capacity )
		{
			world->sensorOverlapDropCount += 1;
			continue;
		}

		b3Visitor shapeRef = { hit.visitorId, visitor->generation };
		b3Array_Push( sensor->hits, shapeRef );
	}

	if ( b3Raw( continuousContext.fraction ) < B3_C_ONE )
	{
		world->toiEventCount += 1;

		// Put the body back where it first touched, and make that the start of
		// the next step's sweep as well -- it is a pose the body actually
		// occupied, so nothing was skipped over.
		b3Quat q = b3NLerp( sweep.q1, sweep.q2, continuousContext.fraction );
		b3Vec3 c = b3Lerp( sweep.c1, sweep.c2, continuousContext.fraction );
		b3Vec3 origin = b3Sub( c, b3RotateVector( q, sweep.localCenter ) );

		b3WorldTransform transform = { b3Add( base, origin ), q };
		b3Pos center = b3Add( base, c );
		fastBodySim->transform = transform;
		fastBodySim->center = center;
		fastBodySim->rotation0 = q;
		fastBodySim->center0 = center;

		// The move event was written from the un-swept pose in finalize.
		b3BodyMoveEvent* event = b3Array_Get( world->bodyMoveEvents, bodySimIndex );
		event->transform = transform;

		// A body can be fast and still barely move, so the AABBs are rebuilt
		// at the impact pose and only grown if they actually left the fat one.
		shapeId = fastBody->headShapeId;
		while ( shapeId != B3_NULL_INDEX )
		{
			b3Shape* shape = b3Array_Get( world->shapes, shapeId );

			b3AABB aabb = b3ComputeFatShapeAABB( shape, transform, B3_SPECULATIVE_DISTANCE );
			shape->aabb = aabb;

			if ( b3AABB_Contains( shape->fatAABB, aabb ) == false )
			{
				b3f margin = shape->aabbMargin;
				b3Vec3 aabbMargin = b3MakeVec3( margin, margin, margin );
				shape->fatAABB.lowerBound = b3Sub( aabb.lowerBound, aabbMargin );
				shape->fatAABB.upperBound = b3Add( aabb.upperBound, aabbMargin );
				shape->flags |= b3_enlargedAABB;
			}

			shapeId = shape->nextShapeId;
		}
	}
	else
	{
		// Nothing in the way: the body keeps the pose it reached, and that pose
		// starts the next sweep.
		fastBodySim->rotation0 = fastBodySim->transform.q;
		fastBodySim->center0 = fastBodySim->center;

		shapeId = fastBody->headShapeId;
		while ( shapeId != B3_NULL_INDEX )
		{
			b3Shape* shape = b3Array_Get( world->shapes, shapeId );

			// shape->aabb was written by the loop above, except for a mesh,
			// which that loop skipped before storing it -- and a mesh on a fast
			// dynamic body has no mass anyway.
			if ( b3AABB_Contains( shape->fatAABB, shape->aabb ) == false )
			{
				b3f margin = shape->aabbMargin;
				b3Vec3 aabbMargin = b3MakeVec3( margin, margin, margin );
				shape->fatAABB.lowerBound = b3Sub( shape->aabb.lowerBound, aabbMargin );
				shape->fatAABB.upperBound = b3Add( shape->aabb.upperBound, aabbMargin );
				shape->flags |= b3_enlargedAABB;
			}

			shapeId = shape->nextShapeId;
		}
	}

	B3_UNUSED( context );
}

// =========================================================================
// Finalize
// =========================================================================

/// Advance every awake body's transform from the deltas the sub-steps
/// accumulated, then decide what that motion means: is the body sleepy, is it
/// moving fast enough to need continuous collision, and did any of its shapes
/// outgrow their fat AABB.
static void b3FinalizeBodiesTask( int startIndex, int endIndex, b3StepContext* context )
{
	b3World* world = context->world;
	b3Body* bodies = world->bodies.data;
	b3BodySim* sims = context->sims;
	b3BodyState* states = context->states;

	bool enableSleep = world->enableSleep;
	bool enableContinuous = world->enableContinuous;
	b3t timeStep = context->dt;
	b3f invTimeStep = context->inv_dt;
	uint16_t worldId = world->worldId;

	b3BodyMoveEvent* moveEvents = world->bodyMoveEvents.data;

	b3BitSet* enlargedSimBitSet = &world->enlargedSimBitSet;
	b3BitSet* awakeIslandBitSet = &world->awakeIslandBitSet;

	for ( int simIndex = startIndex; simIndex < endIndex; ++simIndex )
	{
		b3BodyState* state = states + simIndex;
		b3BodySim* sim = sims + simIndex;

		b3Vec3 v = state->linearVelocity;
		b3Vec3 w = state->angularVelocity;
		b3Vec3 localOmega = b3InvRotateVector( sim->transform.q, w );
		b3Vec3 localDeltaRotation = b3InvRotateVector( sim->transform.q, b3FromDir3( state->deltaRotation.v ) );

		B3_ASSERT( b3IsValidVec3( v ) );
		B3_ASSERT( b3IsValidVec3( w ) );

		// The Q24 accumulator is narrowed to the Q12 position exactly once,
		// here. Whatever did not fit stays in deltaPosition and is applied by
		// a later step -- see the field's comment. Zeroing it instead would
		// discard up to half a quantum per step, always with the same sign for
		// a body in steady motion, which is 7 mm/s of drift at 60 Hz.
		b3Vec3 appliedPosition = b3W3ToVec3( state->deltaPosition );
		b3Vec3W positionRemainder = b3SubW3( state->deltaPosition, b3Vec3ToW3( appliedPosition ) );

		sim->center = b3OffsetPos( sim->center, appliedPosition );
		sim->transform.q = b3NormalizeQuat( b3MulQuat( state->deltaRotation, sim->transform.q ) );

		// The velocity of the farthest point on the body, so that a body which
		// is only spinning still counts as moving.
		b3Vec3 velocityArc = b3ModifiedCrossFF( b3Abs( localOmega ), sim->maxExtent );
		b3f maxVelocity = b3AddF( b3Length( v ), b3Length( velocityArc ) );

		// Sleep must observe position correction as well as true velocity: a
		// body being pushed out of penetration is not at rest even if its
		// velocity says so.
		//   q = [sin(theta/2) * v, cos(theta/2)], so for small angles
		//   |theta| ~= 2 * |sin(theta/2) * v|.
		b3Vec3 rotationArc = b3ModifiedCrossFF( b3Abs( localDeltaRotation ), sim->maxExtent );
		b3f rotationArcLength = b3Length( rotationArc );
		b3f maxDeltaPosition = b3AddF( b3Length( appliedPosition ), b3AddF( rotationArcLength, rotationArcLength ) );

		// inv_dt times a length is a velocity, so this is Q12 x Q12 -> Q12 with
		// inv_dt carried at the length scale (see B3_NEA_INV_DT). Position
		// correction counts for half, because it matters less than true motion.
		b3f positionSleepVelocity = b3MulFF( b3MulFF( b3fFromFrac( 1, 2 ), invTimeStep ), maxDeltaPosition );
		b3f sleepVelocity = b3MaxF( maxVelocity, positionSleepVelocity );

		state->deltaPosition = positionRemainder;
		state->deltaRotation = b3Quat_identityFn();

		sim->transform.p = b3OffsetPos( sim->center, b3Neg( b3RotateVector( sim->transform.q, sim->localCenter ) ) );

		b3Body* body = bodies + sim->bodyId;
		body->bodyMoveIndex = simIndex;
		body->sleepVelocity = sleepVelocity;

		moveEvents[simIndex].userData = body->userData;
		moveEvents[simIndex].transform = sim->transform;
		moveEvents[simIndex].bodyId = ( b3BodyId ){ sim->bodyId + 1, worldId, body->generation };
		moveEvents[simIndex].fellAsleep = false;

		sim->force = b3Vec3_zeroFn();
		sim->torque = b3Vec3_zeroFn();

		// If this fires the caller deferred mass computation and never called
		// b3Body_ApplyMassFromShapes or b3Body_SetMassData.
		B3_ASSERT( ( body->flags & b3_dirtyMass ) == 0 );

		body->flags &= ~b3_bodyTransientFlags;
		body->flags |= ( sim->flags & ( b3_isSpeedCapped | b3_hadTimeOfImpact ) );
		body->flags |= ( state->flags & ( b3_isSpeedCapped | b3_hadTimeOfImpact ) );
		sim->flags &= ~b3_bodyTransientFlags;
		state->flags &= ~b3_bodyTransientFlags;

		// A body that has left the world is parked where it is.
		//
		// Fixed point makes this necessary where float does not. A dynamic body
		// with nothing under it falls at maxLinearSpeed forever, and in Q19.12
		// "forever" runs into the format: b3AABB_Center's sum overflows at
		// 2^18 units and the position wraps at 2^19. Either one puts a proxy
		// with a nonsense centre into the broad phase, and in a release build
		// -- where B3_ASSERT is compiled out -- nothing notices until a
		// traversal reads through a corrupted index. That was the physics
		// examples' softlock: push a box off the floor, wait about eleven
		// minutes, watch the ROM freeze.
		//
		// Parking rather than destroying, clamping or teleporting: the body
		// keeps its position, so a game can find it and decide. It is checked
		// against the *center*, which is what the broad phase actually uses.
		//
		// Zero disables the check and restores the old behaviour.
		if ( body->type == b3_dynamicBody && b3Raw( world->maxWorldExtent ) > 0 )
		{
			int32_t extent = b3Raw( world->maxWorldExtent );
			b3Pos c = sim->center;

			if ( b3Raw( b3AbsF( c.x ) ) > extent || b3Raw( b3AbsF( c.y ) ) > extent ||
				 b3Raw( b3AbsF( c.z ) ) > extent )
			{
				// Stop it dead and mark it sleepy enough that the island check
				// below parks it this step rather than after B3_TIME_TO_SLEEP
				// more seconds of falling.
				state->linearVelocity = b3Vec3_zeroFn();
				state->angularVelocity = b3Vec3_zeroFn();
				v = state->linearVelocity;
				w = state->angularVelocity;
				sleepVelocity = b3f_zero;

				body->sleepVelocity = sleepVelocity;
				body->sleepTime = B3_TIME_TO_SLEEP;
				world->parkedBodyCount += 1;
			}
		}

		if ( enableSleep == false || ( body->flags & b3_enableSleep ) == 0 ||
			 b3Raw( sleepVelocity ) > b3Raw( body->sleepThreshold ) )
		{
			body->sleepTime = b3t_zero;

			b3f maxMotion = b3MaxF( maxDeltaPosition, b3MulFT( maxVelocity, timeStep ) );
			b3f safeMotion = b3Makeb3f( b3Raw( sim->minExtent ) >> 1 );

			if ( body->type == b3_dynamicBody && enableContinuous && b3Raw( maxMotion ) > b3Raw( safeMotion ) )
			{
				sim->flags |= b3_isFast;

				// center0 and rotation0 are deliberately *not* stamped here.
				// They are the start of the sweep, and overwriting them with
				// the pose the body has already reached would leave the
				// continuous pass a sweep of zero length -- which is what this
				// branch did for every phase before Stage 2, when there was no
				// sweep to feed.
				if ( ( sim->flags & b3_isBullet ) == 0 )
				{
					b3SolveContinuous( world, simIndex, context );
				}

				// A bullet is left for the second pass in b3Solve, so that it
				// sweeps against where the other fast bodies *stopped* rather
				// than where they were heading.
			}
			else
			{
				// Safe to advance.
				sim->center0 = sim->center;
				sim->rotation0 = sim->transform.q;
			}
		}
		else
		{
			// Safe to advance, and falling asleep.
			sim->center0 = sim->center;
			sim->rotation0 = sim->transform.q;
			body->sleepTime = b3AddT( body->sleepTime, timeStep );
		}

		// Q30 similarity transform, not the Q12 matrix form: b3RotateInertiaW
		// was built that way in 3A precisely because a Q12 rotation matrix is
		// not exactly orthonormal and the off-diagonal terms of an isotropic
		// tensor must cancel to zero.
		sim->invInertiaWorld = b3RotateInertiaW( sim->transform.q, sim->invInertiaLocal );

		// One awake body keeps its whole island awake.
		b3Island* island = b3Array_Get( world->islands, body->islandId );
		if ( b3Raw( body->sleepTime ) < b3Raw( B3_TIME_TO_SLEEP ) )
		{
			b3SetBit( awakeIslandBitSet, island->localIndex );
		}
		else if ( island->constraintRemoveCount > 0 )
		{
			// Wants to sleep, but the island needs splitting first. Ties are
			// broken by island id so the choice is deterministic.
			if ( b3Raw( body->sleepTime ) > b3Raw( world->splitSleepTime ) ||
				 ( b3Raw( body->sleepTime ) == b3Raw( world->splitSleepTime ) && body->islandId > world->splitIslandId ) )
			{
				world->splitIslandId = body->islandId;
				world->splitSleepTime = body->sleepTime;
			}
		}

		// Rebuild the shape AABBs.
		b3WorldTransform transform = sim->transform;
		bool isFast = ( sim->flags & b3_isFast ) != 0;
		int shapeId = body->headShapeId;
		while ( shapeId != B3_NULL_INDEX )
		{
			b3Shape* shape = b3Array_Get( world->shapes, shapeId );

			if ( isFast )
			{
				// The AABB belongs to the continuous pass: b3SolveContinuous
				// wrote it at the impact pose above, or a bullet's second pass
				// will write it shortly. Either way it must not be recomputed
				// here from the un-swept transform.
				//
				// Flagged enlarged regardless of whether it grew, and through a
				// bit set rather than directly, to keep the move array in
				// deterministic order.
				b3SetBit( enlargedSimBitSet, simIndex );
			}
			else
			{
				b3AABB aabb = b3ComputeFatShapeAABB( shape, transform, B3_SPECULATIVE_DISTANCE );
				shape->aabb = aabb;

				if ( b3AABB_Contains( shape->fatAABB, aabb ) == false )
				{
					b3f margin = shape->aabbMargin;
					b3Vec3 aabbMargin = b3MakeVec3( margin, margin, margin );
					shape->fatAABB.lowerBound = b3Sub( aabb.lowerBound, aabbMargin );
					shape->fatAABB.upperBound = b3Add( aabb.upperBound, aabbMargin );
					shape->flags |= b3_enlargedAABB;

					b3SetBit( enlargedSimBitSet, simIndex );
				}
			}

			shapeId = shape->nextShapeId;
		}
	}
}

// =========================================================================
// b3Solve
// =========================================================================

int b3SolverStackDemand( int contactCount, int manifoldCount )
{
	// Three live entries at the peak, each rounded up to B3_ALIGNMENT by
	// b3StackAlloc: the contact index list b3World_Step takes before calling in
	// here, then the two constraint arrays below.
	int bytes = 0;

	if ( contactCount > 0 )
	{
		int indices = contactCount * (int)sizeof( int );
		bytes += ( ( indices - 1 ) | ( B3_ALIGNMENT - 1 ) ) + 1;

		int contacts = contactCount * (int)sizeof( b3ContactConstraint );
		bytes += ( ( contacts - 1 ) | ( B3_ALIGNMENT - 1 ) ) + 1;
	}

	if ( manifoldCount > 0 )
	{
		int manifolds = manifoldCount * (int)sizeof( b3ManifoldConstraint );
		bytes += ( ( manifolds - 1 ) | ( B3_ALIGNMENT - 1 ) ) + 1;
	}

	return bytes;
}

/// @note ITCM group B3_ITCM_CORE -- see nea_config.h.
void B3_ITCM_IF( B3_ITCM_CORE, b3Solve )( b3World* world, b3StepContext* context )
{
	// Only steps that advance the simulation are counted.
	world->stepIndex += 1;

	b3SolverSet* awakeSet = b3Array_Get( world->solverSets, b3_awakeSet );
	int awakeBodyCount = awakeSet->bodySims.count;
	world->profile.awakeBodyCount = awakeBodyCount;
	if ( awakeBodyCount == 0 )
	{
		b3ValidateNoEnlarged( &world->broadPhase );

		// Marked even though it is nothing, so the phases still tile the step.
		// Without this the whole of a fully-asleep step lands on sensorTicks,
		// which is the one reading that would make sleeping look expensive at
		// exactly the moment it is doing its job.
		B3_PROFILE_MARK( &world->profileTimer, &world->profile.solveTicks );
		return;
	}

	context->sims = awakeSet->bodySims.data;
	context->states = awakeSet->bodyStates.data;
	context->graph = &world->constraintGraph;

	// One move event per awake body, filled by the finalize pass.
	b3Array_Resize( world->bodyMoveEvents, awakeBodyCount );

	// Per-step scratch. Upstream sizes these per worker and unions them
	// afterwards; there is one of each here.
	b3SetBitCountAndClear( &world->enlargedSimBitSet, (uint32_t)awakeBodyCount );
	b3SetBitCountAndClear( &world->awakeIslandBitSet, (uint32_t)world->islands.count );
	world->splitIslandId = B3_NULL_INDEX;
	world->splitSleepTime = b3t_zero;

	b3GraphColor* color = world->constraintGraph.colors + B3_OVERFLOW_INDEX;
	int contactCount = color->contacts.count;
	int jointCount = color->jointSims.count;

	// Upstream computes manifold starts across every active colour here. There
	// is one colour, so this is the whole of it.
	int manifoldCount = 0;
	for ( int i = 0; i < contactCount; ++i )
	{
		color->contacts.data[i].manifoldStart = manifoldCount;
		manifoldCount += color->contacts.data[i].manifoldCount;
	}

	// The per-step constraint arrays. From the stack allocator rather than the
	// heap: they live for exactly one step, they are the largest transient
	// allocation the engine makes, and b3World_Step asserts the stack is empty
	// again at the end -- so a missed free here is caught immediately rather
	// than leaking a step at a time.
	//
	// b3SolverStackDemand below must account for both, because b3CreateWorld
	// sizes the stack from it and a shortfall would send b3GrowStack to the
	// heap mid-step.
	color->contactConstraintCount = contactCount;
	color->manifoldConstraintCount = manifoldCount;
	color->contactConstraints =
		(b3ContactConstraint*)b3StackAlloc( &world->stack, contactCount * (int)sizeof( b3ContactConstraint ), "contacts" );
	color->manifoldConstraints = (b3ManifoldConstraint*)b3StackAlloc(
		&world->stack, manifoldCount * (int)sizeof( b3ManifoldConstraint ), "manifold constraints" );

	context->contactConstraints = color->contactConstraints;
	context->manifoldConstraints = color->manifoldConstraints;

	// Hit events are collected by b3StoreImpulses into a bit set indexed by
	// contact id, then turned into events once at the end of the step. One
	// worker, so upstream's per-worker union is gone.
	b3SetBitCountAndClear( &world->hitEventBitSet, (uint32_t)world->contacts.count );
	world->hasHitEvents = false;

	// Same shape for joint events, sized by the joint array rather than the
	// contact array because the bit index is a joint id.
	b3SetBitCountAndClear( &world->jointStateBitSet, (uint32_t)world->joints.count );
	world->hasJointEvents = false;

	// The sub-step loop. Upstream drives this through a stage table so that
	// workers can synchronise between stages; serially it is just the loop the
	// stage table describes.
	// Prepare runs **once**, before the loop -- not per sub-step.
	//
	// 3C-i placed b3PrepareJointsTask inside the loop, which was invisible
	// while the joint stages were stubs. It is not invisible for contacts:
	// prepare captures `relativeVelocity`, the approach speed restitution is
	// computed from, and re-running it each sub-step captures the velocity
	// *after* the normal solve has already stopped the body. A ball dropped
	// with restitution 0.5 then never bounced at all, because by the last
	// sub-step it was no longer approaching. Prepare also seeds the impulse
	// accumulators from the manifold, so re-running it discarded each
	// sub-step's accumulation.
	b3PrepareJointsTask( 0, jointCount, context );
	b3PrepareContacts( 0, contactCount, context );
	B3_PROFILE_MARK( &world->profileTimer, &world->profile.prepareTicks );

	// The stage counts the solver actually worked on, for the profile. Free --
	// all three are already in hand.
	world->profile.contactCount = contactCount;
	world->profile.manifoldCount = manifoldCount;
	world->profile.jointCount = jointCount;

	for ( int i = 0; i < context->subStepCount; ++i )
	{
		// The eight B3_PROFILE_SUBSTEP marks below accumulate rather than
		// assign, so each field ends the step holding that stage's total over
		// every sub-step. They are gated on B3_NEA_PROFILE_SUBSTEP separately
		// from level 1 because they grow this loop -- see the switch's comment
		// in nea_config.h for why that makes their absolute values suspect and
		// their ratios still useful.
		b3IntegrateVelocitiesTask( 0, awakeBodyCount, context );
		B3_PROFILE_SUBSTEP( &world->profileTimer, &world->profile.integrateVelocitiesTicks );

		b3WarmStartJointsTask( 0, jointCount, context );
		B3_PROFILE_SUBSTEP( &world->profileTimer, &world->profile.warmStartJointsTicks );

		b3WarmStartContacts( 0, contactCount, context );
		B3_PROFILE_SUBSTEP( &world->profileTimer, &world->profile.warmStartContactsTicks );

		b3SolveJointsTask( 0, jointCount, context, true );
		B3_PROFILE_SUBSTEP( &world->profileTimer, &world->profile.solveJointsTicks );

		b3SolveContacts( 0, contactCount, context, true );
		B3_PROFILE_SUBSTEP( &world->profileTimer, &world->profile.solveContactsTicks );

		b3IntegratePositionsTask( 0, awakeBodyCount, context );
		B3_PROFILE_SUBSTEP( &world->profileTimer, &world->profile.integratePositionsTicks );

		b3SolveJointsTask( 0, jointCount, context, false );
		B3_PROFILE_SUBSTEP( &world->profileTimer, &world->profile.relaxJointsTicks );

		// The relax pass. Without bias, so it removes the velocity the bias
		// injected rather than adding more -- and it is where friction is
		// applied, since a friction impulse computed against a biased normal
		// impulse would be scaled by the position correction.
		b3SolveContacts( 0, contactCount, context, false );
		B3_PROFILE_SUBSTEP( &world->profileTimer, &world->profile.relaxContactsTicks );
	}

	// Level 1's view of the same loop, and the one to trust. With level 2 off
	// this is a single mark across the whole loop and the only probe inside
	// b3Solve's hot region, so it does not disturb what it measures.
	//
	// With level 2 *on* the eight marks above have already advanced the
	// timer's reference point to here, so this charges zero rather than
	// double-counting -- which is why it is a mark and not an elapsed-since.
	// Sum the eight instead in that build.
	B3_PROFILE_MARK( &world->profileTimer, &world->profile.subStepTicks );

	b3ApplyRestitution( 0, contactCount, context );
	b3StoreImpulses( 0, contactCount, context );
	B3_PROFILE_MARK( &world->profileTimer, &world->profile.restitutionTicks );

	// -----------------------------------------------------------------
	// Hit events
	// -----------------------------------------------------------------
	//
	// The last piece of the event API 3B plumbed and left unfed: the array is
	// created, cleared each step and published by b3World_GetContactEvents,
	// and until now nothing ever pushed to it.
	//
	// b3StoreImpulses flags a *contact* whose approach speed cleared the
	// threshold; this turns each flagged contact into one event carrying its
	// fastest point. Upstream unions a bit set per worker first, which is one
	// loop here.
	if ( world->hasHitEvents )
	{
		B3_ASSERT( world->contactHitEvents.count == 0 );

		b3f threshold = world->hitEventThreshold;
		b3Contact* contactArray = world->contacts.data;
		uint16_t worldIdShort = world->worldId;

		b3BitSet* hitEventBitSet = &world->hitEventBitSet;
		for ( uint32_t k = 0; k < hitEventBitSet->blockCount; ++k )
		{
			uint64_t word = hitEventBitSet->bits[k];
			while ( word != 0 )
			{
				uint32_t ctz = b3CTZ64( word );
				int contactId = (int)( 64 * k + ctz );

				b3Contact* contact = contactArray + contactId;
				B3_ASSERT( contact->setIndex == b3_awakeSet && contact->colorIndex != B3_NULL_INDEX );

				b3Shape* shapeA = b3Array_Get( world->shapes, contact->shapeIdA );
				b3Shape* shapeB = b3Array_Get( world->shapes, contact->shapeIdB );
				b3Body* bodyA = b3Array_Get( world->bodies, shapeA->bodyId );
				b3Body* bodyB = b3Array_Get( world->bodies, shapeB->bodyId );
				b3BodySim* simA = b3GetBodySim( world, bodyA );
				b3BodySim* simB = b3GetBodySim( world, bodyB );
				b3Pos midCenter = b3Lerp( simA->center, simB->center, b3cFromFrac( 1, 2 ) );

				b3ContactHitEvent event = { 0 };
				event.approachSpeed = threshold;

				bool found = false;
				int triangleIndex = 0;
				int manifoldCount = contact->manifoldCount;
				for ( int i = 0; i < manifoldCount; ++i )
				{
					b3Manifold* manifold = contact->manifolds + i;
					int pointCount = manifold->pointCount;
					for ( int p = 0; p < pointCount; ++p )
					{
						b3ManifoldPoint* mp = manifold->points + p;
						b3f approachSpeed = b3NegF( mp->normalVelocity );

						// The total impulse test is what excludes a speculative
						// point that was approaching fast but never touched.
						if ( b3Raw( approachSpeed ) > b3Raw( event.approachSpeed ) &&
							 b3Raw( mp->totalNormalImpulse ) > 0 )
						{
							event.approachSpeed = approachSpeed;
							event.point =
								b3OffsetPos( midCenter, b3Lerp( mp->anchorA, mp->anchorB, b3cFromFrac( 1, 2 ) ) );
							event.normal = manifold->normal;
							triangleIndex = mp->triangleIndex;
							found = true;
						}
					}
				}

				if ( found )
				{
					event.shapeIdA = ( b3ShapeId ){ shapeA->id + 1, worldIdShort, shapeA->generation };
					event.shapeIdB = ( b3ShapeId ){ shapeB->id + 1, worldIdShort, shapeB->generation };
					event.contactId = ( b3ContactId ){
						.index1 = contact->contactId + 1,
						.world0 = worldIdShort,
						.padding = 0,
						.generation = contact->generation,
					};

					// shapeB is never a compound (asserted in b3CreateContact),
					// so its child index is irrelevant; shapeA carries it.
					event.userMaterialIdA = b3GetShapeUserMaterialId( shapeA, contact->childIndex, triangleIndex );
					event.userMaterialIdB = b3GetShapeUserMaterialId( shapeB, 0, triangleIndex );

					b3Array_Push( world->contactHitEvents, event );
				}

				word = word & ( word - 1 );
			}
		}
	}

	// -----------------------------------------------------------------
	// Joint events
	// -----------------------------------------------------------------
	//
	// The same drain as above, over joint ids rather than contact ids.
	// b3SolveJointsTask flagged each joint whose reaction crossed a threshold;
	// this reads the reaction back once per flagged joint and pushes one event.
	//
	// Reading it again here rather than stashing it in the solve is deliberate:
	// the flag can be set on any sub-step, and the number worth reporting is the
	// one that stands at the end of the step -- which is what
	// b3Joint_GetConstraintForce would report to a caller looking at the same
	// joint after b3World_Step returns.
	if ( world->hasJointEvents )
	{
		B3_ASSERT( world->jointEvents.count == 0 );

		b3Joint* jointArray = world->joints.data;

		b3BitSet* jointStateBitSet = &world->jointStateBitSet;
		for ( uint32_t k = 0; k < jointStateBitSet->blockCount; ++k )
		{
			uint64_t word = jointStateBitSet->bits[k];
			while ( word != 0 )
			{
				uint32_t ctz = b3CTZ64( word );
				int jointId = (int)( 64 * k + ctz );

				b3Joint* joint = jointArray + jointId;
				b3JointSim* base = b3GetJointSim( world, joint );

				b3f force, torque;
				b3GetJointReactionScalars( world, base, &force, &torque );

				b3JointEvent event = { 0 };
				event.jointId = ( b3JointId ){
					.index1 = jointId + 1,
					.world0 = world->worldId,
					.generation = joint->generation,
				};
				event.userData = joint->userData;
				event.force = force;
				event.torque = torque;
				event.forceExceeded = b3Raw( force ) >= b3Raw( base->forceThreshold );
				event.torqueExceeded = b3Raw( torque ) >= b3Raw( base->torqueThreshold );

				b3Array_Push( world->jointEvents, event );

				word = word & ( word - 1 );
			}
		}
	}

	// Both event drains above are conditional on hasHitEvents/hasJointEvents,
	// so this reads zero in the common step where nothing crossed a threshold.
	// That is the useful answer, not a missing one: it says the drains are not
	// what the step is spending.
	B3_PROFILE_MARK( &world->profileTimer, &world->profile.eventTicks );

	b3FinalizeBodiesTask( 0, awakeBodyCount, context );
	B3_PROFILE_MARK( &world->profileTimer, &world->profile.finalizeTicks );

	// Reverse order, because the stack allocator is a stack. Done before the
	// sleeping pass below, which can move body sims between solver sets and
	// invalidate everything the constraints point at.
	b3StackFree( &world->stack, color->manifoldConstraints );
	b3StackFree( &world->stack, color->contactConstraints );
	color->manifoldConstraints = NULL;
	color->contactConstraints = NULL;
	color->manifoldConstraintCount = 0;
	color->contactConstraintCount = 0;
	context->manifoldConstraints = NULL;
	context->contactConstraints = NULL;

	// -----------------------------------------------------------------
	// Bullets
	// -----------------------------------------------------------------
	//
	// The fast bodies that also sweep against kinematic and dynamic geometry,
	// held back until every ordinary fast body has been pulled to its impact.
	//
	// Upstream collects these into a stack-allocated array during finalize,
	// because its workers fill it concurrently and the order must not depend on
	// which thread got there first. There is nothing to collect here: b3_isFast
	// is a transient that finalize sets after clearing, so re-testing the flag
	// finds exactly the same bodies in exactly the same order, with no
	// allocation and nothing added to b3SolverStackDemand.
	//
	// Must stay ahead of the AABB pass below and the sleeping pass after it --
	// the first reads the proxies a bullet may still move, and the second
	// invalidates these very indices by moving sims between solver sets.
	{
		b3BodySim* sims = context->sims;
		const uint32_t bulletFlags = b3_isFast | b3_isBullet;

		for ( int simIndex = 0; simIndex < awakeBodyCount; ++simIndex )
		{
			if ( ( sims[simIndex].flags & bulletFlags ) == bulletFlags )
			{
				b3SolveContinuous( world, simIndex, context );
			}
		}

		// Pairs with toiEventCount and the three TOI iteration counters. A
		// large reading here beside a zero event count means the sweeps are
		// running and finding nothing, which is a different problem from the
		// sweeps being expensive when they hit.
		B3_PROFILE_MARK( &world->profileTimer, &world->profile.continuousTicks );
	}

	// -----------------------------------------------------------------
	// Apply the AABB growth to the broad phase
	// -----------------------------------------------------------------
	//
	// Walking a bit set of *bodies* rather than shapes, because a world can
	// hold far more shapes than bodies and the move array has to come out in
	// deterministic order.
	{
		b3BroadPhase* broadPhase = &world->broadPhase;
		b3BitSet* bitSet = &world->enlargedSimBitSet;
		b3BodySim* sims = context->sims;
		b3Body* bodies = world->bodies.data;

		for ( uint32_t k = 0; k < bitSet->blockCount; ++k )
		{
			uint64_t bits = bitSet->bits[k];
			while ( bits != 0 )
			{
				uint32_t ctz = b3CTZ64( bits );
				int simIndex = (int)( 64 * k + ctz );

				b3BodySim* sim = sims + simIndex;
				b3Body* body = bodies + sim->bodyId;

				int shapeId = body->headShapeId;
				while ( shapeId != B3_NULL_INDEX )
				{
					b3Shape* shape = b3Array_Get( world->shapes, shapeId );

					// The body being flagged does not mean every one of its
					// shapes grew -- a multi-shape body may have enlarged only
					// one, and a fast body is flagged regardless.
					if ( shape->flags & b3_enlargedAABB )
					{
						b3BroadPhase_EnlargeProxy( broadPhase, shape->proxyKey, shape->fatAABB );
						shape->flags &= ~b3_enlargedAABB;
					}

					shapeId = shape->nextShapeId;
				}

				bits = bits & ( bits - 1 );
			}
		}

		b3ValidateBroadPhase( &world->broadPhase );

		B3_PROFILE_MARK( &world->profileTimer, &world->profile.enlargeTicks );
	}

	// -----------------------------------------------------------------
	// Island splitting and sleeping
	// -----------------------------------------------------------------
	//
	// Last, and it has to be: putting an island to sleep moves body sims
	// between solver sets, which invalidates every index the enlarged bit set
	// above is expressed in.
	if ( world->enableSleep )
	{
		if ( world->splitIslandId != B3_NULL_INDEX )
		{
			b3SplitIsland( world, world->splitIslandId );
		}

		// Islands whose bit is clear had no awake body this step.
		b3BitSet* awakeIslandBitSet = &world->awakeIslandBitSet;
		for ( int islandIndex = 0; islandIndex < world->solverSets.data[b3_awakeSet].islandSims.count; ++islandIndex )
		{
			if ( b3GetBit( awakeIslandBitSet, (uint32_t)islandIndex ) == true )
			{
				continue;
			}

			int islandId = world->solverSets.data[b3_awakeSet].islandSims.data[islandIndex].islandId;

			// Only step back when the island actually slept. A sleeping island
			// is swap-removed, so an unvisited one lands in the slot just
			// looked at and the index has to be revisited -- but an island
			// with a pending split *declines*, and stepping back onto one of
			// those is an infinite loop: same index, awake bit still clear,
			// same refusal, forever. It hung the ROM in about forty seconds in
			// any scene that makes and breaks contacts often enough.
			if ( b3TrySleepIsland( world, islandId ) )
			{
				islandIndex -= 1;
			}
		}
	}

	b3ValidateSolverSets( world );

	// Outside the enableSleep guard, so the field is written on every step
	// rather than keeping a stale value from the last step that had sleeping
	// on. A world with sleeping disabled reads zero here, which is the truth.
	B3_PROFILE_MARK( &world->profileTimer, &world->profile.sleepTicks );

	// The eight sub-phases summed. Done here rather than by b3World_Step
	// because this is the only scope that knows the sub-phases exist -- and
	// summing them rather than bracketing b3Solve from outside means a phase
	// added later without a probe shows up as a gap against totalTicks instead
	// of silently inflating the solve.
	{
		const b3Profile* p = &world->profile;
		world->profile.solveTicks = p->prepareTicks + p->subStepTicks + p->restitutionTicks + p->eventTicks +
									p->finalizeTicks + p->continuousTicks + p->enlargeTicks + p->sleepTicks;
	}
}
