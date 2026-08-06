// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Erin Catto        (original Box3D)
// Copyright (c) 2026 Warioware64       (Nitro Engine Advanced fixed-point port)
//
// This file is part of Nitro Engine Advanced

#pragma once

/// @file   body.h
/// @brief  Rigid bodies, split three ways for locality.
///
/// @section split The three structs
///
/// b3Body is the sparse record a b3BodyId resolves to: lists, counts, indices,
/// and the handful of scalars nothing in the solver's inner loop reads.
/// b3BodySim is the dense per-body simulation data, contiguous within a solver
/// set. b3BodyState is velocity, and exists only for awake bodies -- static
/// bodies have none, which is what stops the solver writing to shared data.
///
/// @section scales Scales
///
/// The mass fields are where this file departs most from upstream:
///
///   b3Body::mass          b3f     total mass
///   b3Body::inertia       b3Matrix3, **per unit mass** -- see below
///   b3BodySim::invMass    b3iw    Q24, because 1/1000 has four bits at Q12
///   b3BodySim::invInertia b3MatrixW, Q24, for the same reason
///   b3Body::sleepTime     b3t     Q24, because it accumulates h every step
///   b3Body::sleepThreshold, sleepVelocity   b3f, both speeds
///
/// b3Body::inertia holds the tensor **divided by the mass** -- the radius of
/// gyration squared. That is the convention b3MassData established in Phase 1
/// and the reason nothing here overflows: absolute inertia grows as length to
/// the fifth and caps a solid sphere at radius 12 in Q12. b3Body_GetMassData
/// and b3Body_SetMassData both speak this convention, so a value round trips.
///
/// @section absent What is not here
///
/// b3Body_CastRay, b3Body_CastShape, b3Body_OverlapShape, b3Body_CollideMover
/// and b3Body_GetClosestPoint are still absent, and by now deliberately: every
/// one is a thin wrapper over machinery that has existed since Phase 7 Stage 1,
/// and none has ever had a caller. Four stages have declined them on that
/// ground -- adding an entry point with no caller is how `forceThreshold` sat
/// dead from Stage 1 to Stage 5 of Phase 6. b3World_CollideMover covers what a
/// mover actually needs. b3Body_SetTargetTransform is a kinematic-target helper that
/// needs a quaternion log; NEA_Phys3D can add it if a game wants it.
/// b3Body_SetName/GetName follow B3_NEA_NO_NAMES. b3Body_ApplyWind went with
/// the rest of the wind code.

#include "shape.h"

#include "box3d/box3d.h"
#include "box3d/constants.h"
#include "box3d/id.h"
#include "box3d/math_fixed.h"
#include "box3d/types.h"

typedef struct b3World b3World;

enum b3BodyFlags
{
	// This body has fixed translation along the x-axis
	b3_lockLinearX = 0x00000001,

	// This body has fixed translation along the y-axis
	b3_lockLinearY = 0x00000002,

	// This body has fixed translation along the z-axis
	b3_lockLinearZ = 0x00000004,

	// This body has fixed rotation around the x-axis
	b3_lockAngularX = 0x00000008,

	// This body has fixed rotation around the y-axis
	b3_lockAngularY = 0x00000010,

	// This body has fixed rotation around the z-axis
	b3_lockAngularZ = 0x00000020,

	// This body is moving fast enough that the continuous path cares (Phase 7)
	b3_isFast = 0x00000040,

	// This dynamic body does a final CCD pass against all body types, but not
	// other bullets (Phase 7)
	b3_isBullet = 0x00000080,

	// This body was speed capped in the current time step
	b3_isSpeedCapped = 0x00000100,

	// This body had a time of impact event in the current time step
	b3_hadTimeOfImpact = 0x00000200,

	// This body has no limit on angular velocity
	b3_allowFastRotation = 0x00000400,

	// This body needs its AABB increased
	b3_enlargeBounds = 0x00000800,

	// This body is dynamic so the solver should write to it. Upstream keeps
	// this to avoid cache line sharing between workers writing kinematic
	// bodies; here it is simply the cheapest dynamic test the solver has.
	b3_dynamicFlag = 0x00001000,

	b3_enableSleep = 0x00002000,

	b3_bodyEnableContactRecycling = 0x00004000,

	// The user deferred mass computation via the updateBodyMass shape option
	// and mass data still hasn't been set.
	b3_dirtyMass = 0x00008000,

	// All lock flags
	b3_allLocks = b3_lockLinearX | b3_lockLinearY | b3_lockLinearZ | b3_lockAngularX | b3_lockAngularY | b3_lockAngularZ,

	// With all of these set the body has fixed rotation
	b3_fixedRotation = b3_lockAngularX | b3_lockAngularY | b3_lockAngularZ,

	// Transient per time step. These may differ between b3Body, b3BodySim and
	// b3BodyState, so b3SyncBodyFlags never propagates them.
	b3_bodyTransientFlags = b3_isFast | b3_isSpeedCapped | b3_hadTimeOfImpact,
};

/// Body organizational details that are not used in the solver.
typedef struct b3Body
{
	void* userData;

	/// Index of the solver set holding this body's sim. May be B3_NULL_INDEX.
	int setIndex;

	/// Sim and state index within that set. May be B3_NULL_INDEX.
	int localIndex;

	/// [31 : contactId | 1 : edgeIndex]
	int headContactKey;
	int contactCount;

	int headShapeId;
	int shapeCount;

	/// [31 : jointId | 1 : edgeIndex]
	int headJointKey;
	int jointCount;

	/// All enabled dynamic and kinematic bodies are in an island.
	int islandId;

	/// Index into the island's bodies array, for O(1) swap removal.
	int islandIndex;

	b3f sleepThreshold;

	/// Accumulates the substep h, so it is b3t and not b3f. At 60 Hz a Q12
	/// accumulator would gain 68 raw units a step and drift toward the
	/// threshold in one direction only -- the truncation bias that cost 0.45%
	/// of a second's gravity integration in Phase 1.
	b3t sleepTime;

	b3f sleepVelocity;
	b3f mass;

	/// Local inertia **per unit mass**. See the file comment.
	b3Matrix3 inertia;

	/// Used to adjust the fellAsleep flag in the body move array.
	int bodyMoveIndex;

	int id;

	/// b3BodyFlags
	uint32_t flags;

	b3BodyType type;

	/// Monotonically advanced when a body is allocated in this slot, so a stale
	/// b3BodyId can be detected.
	uint16_t generation;
} b3Body;

/// Velocity, and the deltas the solver accumulates over a step.
///
/// Only awake dynamic and kinematic bodies have one. Upstream's long note about
/// SIMD scatter-gather does not apply here, but the reason for delta position
/// and delta rotation does: they keep the substep integration relative, which
/// is what stops a body far from the origin losing its low bits every substep.
typedef struct b3BodyState
{
	b3Vec3 linearVelocity;
	b3Vec3 angularVelocity;

	/// Position delta over the step, at **Q24**, holding whatever the sub-steps
	/// have integrated and the finalize pass has not yet applied.
	///
	/// Upstream is float and calls this the delta over the step, existing to
	/// reduce round-off far from the origin. It does that here too, but it also
	/// carries a second job the port needs and float does not.
	///
	/// A Q12 position increment is far coarser than it looks -- 25.6 quanta per
	/// sub-step at 1.5 m/s, 1.7 at 0.1 m/s -- and because the velocity is
	/// constant across the sub-steps of a step, rounding it lands the *same*
	/// way every time. That is a representation bias, not a truncation bias, so
	/// round-to-nearest does not cancel it: measured against float Box3D the
	/// port ran 1.56% long on distance at 1.5 m/s and 17% at 0.1 m/s.
	///
	/// So the accumulation happens at Q24 (b3MulVTToW), and the finalize pass
	/// narrows to Q12 once, applies that, and **leaves the remainder here**
	/// rather than zeroing the field. The sub-quantum part of the integral is
	/// therefore never discarded, only deferred, and the position has no drift
	/// at all rather than a slower one.
	///
	/// The solver reads this as the within-step motion, where the carried-in
	/// remainder is one Q12 quantum at worst -- three orders below the
	/// separations it is differencing.
	b3Vec3W deltaPosition;

	/// Rotation delta over the step. A delta because the solver cannot read the
	/// full rotation of a static body and must use identity for one.
	b3Quat deltaRotation;

	/// b3BodyFlags -- the locking and dynamic bits matter here.
	uint32_t flags;
} b3BodyState;

/// The identity body state: zero velocity, zero position delta, and an
/// *identity* delta rotation rather than a zero one.
///
/// A function rather than upstream's static const, following the same idiom as
/// b3Vec3_zero and b3Quat_identity: under B3_FIXED_DEBUG each scalar is a
/// struct carrying a shadow double, which a flat brace initializer would fill
/// in the wrong order.
B3_INLINE b3BodyState b3IdentityBodyStateFn( void )
{
	b3BodyState s;
	s.linearVelocity = b3Vec3_zeroFn();
	s.angularVelocity = b3Vec3_zeroFn();
	s.deltaPosition = b3Vec3W_zeroFn();
	s.deltaRotation = b3Quat_identityFn();
	s.flags = 0;
	return s;
}

#define b3_identityBodyState b3IdentityBodyStateFn()

/// Dense per-body simulation data: transform, mass and integration parameters.
typedef struct b3BodySim
{
	/// Transform of the body origin.
	b3WorldTransform transform;

	/// Centre of mass in world space.
	b3Pos center;

	/// Previous rotation and centre of mass, for the time of impact (Phase 7).
	b3Quat rotation0;
	b3Pos center0;

	/// Centre of mass relative to the body origin.
	b3Vec3 localCenter;

	b3Vec3 force;
	b3Vec3 torque;

	b3iw invMass;

	/// Inverse rotational inertia about the centre of mass. The world tensor
	/// must be refreshed whenever the body rotation changes.
	b3MatrixW invInertiaLocal;
	b3MatrixW invInertiaWorld;

	b3f minExtent;
	b3Vec3 maxExtent;
	b3f linearDamping;
	b3f angularDamping;
	b3f gravityScale;

	/// Index of the b3Body record.
	int bodyId;

	/// b3BodyFlags
	uint32_t flags;
} b3BodySim;

// -------------------------------------------------------------------------
// Internal
// -------------------------------------------------------------------------

/// Get a validated body from a world using an id.
b3Body* b3GetBodyFullId( b3World* world, b3BodyId bodyId );

b3WorldTransform b3GetBodyTransformQuick( b3World* world, b3Body* body );
b3WorldTransform b3GetBodyTransform( b3World* world, int bodyId );

/// Build a b3BodyId from a raw index.
b3BodyId b3MakeBodyId( b3World* world, int bodyId );

bool b3ShouldBodiesCollide( b3World* world, b3Body* bodyA, b3Body* bodyB );
bool b3IsBodyAwake( b3World* world, b3Body* body );

b3BodySim* b3GetBodySim( b3World* world, b3Body* body );
b3BodyState* b3GetBodyState( b3World* world, b3Body* body );

/// @warning Can invalidate body, state, joint and contact pointers.
bool b3WakeBody( b3World* world, b3Body* body );
bool b3WakeBodyWithLock( b3World* world, b3Body* body );

void b3UpdateBodyMassData( b3World* world, b3Body* body );
void b3SyncBodyFlags( b3World* world, b3Body* body );

// The public API -- b3CreateBody, b3Body_*, b3CreateWorld, b3World_*,
// b3Create*Shape, b3Shape_*, b3Contact_GetData -- is declared in
// include/box3d/box3d.h, which is the header a game installs and includes.
// This file keeps only what the port's own translation units need.
