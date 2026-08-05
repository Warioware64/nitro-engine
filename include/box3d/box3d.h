// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Erin Catto        (original Box3D)
// Copyright (c) 2026 Warioware64       (Nitro Engine Advanced fixed-point port)
//
// This file is part of Nitro Engine Advanced

#ifndef BOX3D_BOX3D_H__
#define BOX3D_BOX3D_H__

/// @file   box3d.h
/// @brief  The public Box3D API. Include this and nothing else.
///
/// @section why Why this file exists
///
/// Box3D ships as its own archive here -- `libNEA_box3d.a` -- and only
/// `include/` is installed. The declarations below used to live next to the
/// structs they operate on, in `source/box3d/physics_world.h`, `body.h`,
/// `shape.h` and `contact.h`, which are internal headers a game never sees.
/// This file gathers them, the way upstream Box3D's own `box3d.h` does, so
/// that the split of public surface from private layout matches the split of
/// what is installed from what is not.
///
/// `b3World`, `b3Body`, `b3BodySim`, `b3BodyState`, `b3Shape` and `b3Contact`
/// stay opaque. Everything a public signature mentions is a value type from
/// `types.h`, `math_fixed.h` or `collision.h`, all of which this header pulls
/// in.
///
/// @section nea Reaching this from Nitro Engine Advanced
///
/// [NEAPhysics3D.h](../NEAPhysics3D.h) wraps the common path in NEA's own
/// conventions -- f32 arguments, `NEA_Model` binding, a pool allocator that
/// forbids mid-step allocation. It deliberately does not wrap all of the
/// below. Contact events, custom filtering, pre-solve callbacks, surface
/// materials and the mass-data accessors are reached by including this header
/// directly and passing the ids `NEAPhysics3D.h` hands out. Both are supported
/// and can be mixed freely; there is no hidden state between them.
///
/// A project links Box3D by naming it explicitly:
///
///     LIBS := -lNEA_box3d -lNEA -lnds9 -lc
///
/// @section absent What is not here
///
/// The port's phase boundaries, not upstream's API:
///
///   - **Joints are complete** as of Phase 6 Stage 7. All nine entry points are
///     here: `b3CreateFilterJoint`, which solves nothing and only stops two
///     bodies colliding; `b3CreateDistanceJoint`, the first with constraint
///     math; `b3CreateRevoluteJoint`, the first that constrains a rotation;
///     `b3CreateSphericalJoint`, a ball joint, the first whose spring and motor
///     act on all three rotational degrees at once; `b3CreateWeldJoint` and
///     `b3CreateMotorJoint`; `b3CreatePrismaticJoint`, a slider, the first
///     whose effective mass depends on its own state rather than only on its
///     geometry; `b3CreateParallelJoint`, an upright-keeper, the smallest
///     solved joint and the only one constraining orientation alone; and
///     `b3CreateWheelJoint`, the largest, a prismatic and a revolute in one.
///     `b3Joint_GetConstraintForce` and `b3Joint_GetConstraintTorque` are here.
///     The linear and angular *separation* queries land with the wheel joint:
///     upstream writes each as one switch over every type, and the ninth type
///     existing is what made writing them once cheaper than growing them a case
///     at a time.
///   - **World queries, ray casts and the mover** (Phase 7):
///     `b3World_CastRay`, `b3World_OverlapAABB`, `b3Body_CastShape`,
///     `b3World_CollideMover` and friends.
///   - **Sensors** (Phase 7).
///   - **Triangle meshes** are here, as `b3CreateMeshShape` -- collidable
///     against spheres, capsules and hulls. The *queries* against them
///     (`b3RayCastMesh`, `b3ShapeCastMesh`, `b3OverlapMesh`) are Phase 7 with
///     the rest of the query layer, and there is no run-time mesh builder:
///     blobs are baked offline.
///   - **Names, profiles and counters** -- see `B3_NEA_NO_NAMES` and the
///     "What is not in b3World" note in `source/box3d/physics_world.h`.
///
/// The shape-level collision functions (`b3CollideSpheres`, `b3ShapeDistance`,
/// `b3MakeBoxHull`, ...) are in `collision.h`, which this header includes.

#include "box3d/base.h"
#include "box3d/collision.h"
#include "box3d/constants.h"
#include "box3d/id.h"
#include "box3d/math_fixed.h"
#include "box3d/types.h"

// =========================================================================
// World
// =========================================================================

B3_API b3WorldId b3CreateWorld( const b3WorldDef* def );
B3_API void b3DestroyWorld( b3WorldId worldId );

B3_API bool b3World_IsValid( b3WorldId id );
B3_API bool b3Body_IsValid( b3BodyId id );
B3_API bool b3Shape_IsValid( b3ShapeId id );
B3_API bool b3Joint_IsValid( b3JointId id );
B3_API bool b3Contact_IsValid( b3ContactId id );

B3_API void b3World_SetGravity( b3WorldId worldId, b3Vec3 gravity );
B3_API b3Vec3 b3World_GetGravity( b3WorldId worldId );
B3_API void b3World_EnableSleeping( b3WorldId worldId, bool flag );
B3_API bool b3World_IsSleepingEnabled( b3WorldId worldId );
B3_API void b3World_EnableWarmStarting( b3WorldId worldId, bool flag );
B3_API bool b3World_IsWarmStartingEnabled( b3WorldId worldId );
B3_API void b3World_EnableContinuous( b3WorldId worldId, bool flag );
B3_API bool b3World_IsContinuousEnabled( b3WorldId worldId );
B3_API void b3World_EnableSpeculative( b3WorldId worldId, bool flag );
B3_API void b3World_SetRestitutionThreshold( b3WorldId worldId, b3f value );
B3_API b3f b3World_GetRestitutionThreshold( b3WorldId worldId );
B3_API void b3World_SetHitEventThreshold( b3WorldId worldId, b3f value );
B3_API b3f b3World_GetHitEventThreshold( b3WorldId worldId );
B3_API void b3World_SetContactTuning( b3WorldId worldId, b3f hertz, b3f dampingRatio, b3f contactSpeed );
B3_API void b3World_SetContactRecycleDistance( b3WorldId worldId, b3f recycleDistance );
B3_API b3f b3World_GetContactRecycleDistance( b3WorldId worldId );
B3_API void b3World_SetMaximumLinearSpeed( b3WorldId worldId, b3f maximumLinearSpeed );
B3_API b3f b3World_GetMaximumLinearSpeed( b3WorldId worldId );
B3_API void b3World_SetFrictionCallback( b3WorldId worldId, b3FrictionCallback* callback );
B3_API void b3World_SetRestitutionCallback( b3WorldId worldId, b3RestitutionCallback* callback );
B3_API void b3World_SetCustomFilterCallback( b3WorldId worldId, b3CustomFilterFcn* fcn, void* context );
B3_API void b3World_SetPreSolveCallback( b3WorldId worldId, b3PreSolveFcn* fcn, void* context );
B3_API void b3World_SetUserData( b3WorldId worldId, void* userData );
B3_API void* b3World_GetUserData( b3WorldId worldId );
B3_API int b3World_GetAwakeBodyCount( b3WorldId worldId );
B3_API b3Capacity b3World_GetMaxCapacity( b3WorldId worldId );
B3_API b3AABB b3World_GetBounds( b3WorldId worldId );
B3_API void b3World_RebuildStaticTree( b3WorldId worldId );

/// The contact events accumulated by the last collide pass.
///
/// Valid until the next one.
B3_API b3ContactEvents b3World_GetContactEvents( b3WorldId worldId );

/// The joint events from the last step: one per joint whose reaction crossed a
/// threshold set on it with b3Joint_SetForceThreshold or
/// b3Joint_SetTorqueThreshold.
///
/// Empty unless a threshold has been set -- both default to B3_NO_BOUND. A joint
/// appears at most once per step no matter how many sub-steps it tripped on, and
/// the reported magnitudes are the ones standing at the end of the step, so they
/// match what b3Joint_GetConstraintForce reports for the same joint afterwards.
///
/// Valid until the next step.
B3_API b3JointEvents b3World_GetJointEvents( b3WorldId worldId );

/// The body move events from the last step: one per awake body, carrying the
/// transform the body ended the step with and whether it fell asleep.
B3_API b3BodyEvents b3World_GetBodyEvents( b3WorldId worldId );

/// Advance the simulation by one time step.
///
/// There is no `timeStep` parameter. dt is fixed at compile time by
/// B3_NEA_STEP_HZ so that 1/h and the soft-constraint coefficients fold into
/// constants rather than costing a hardware divide every sub-step -- see
/// nea_config.h. `subStepCount` is clamped to at least one.
B3_API void b3World_Step( b3WorldId worldId, int subStepCount );

// =========================================================================
// Body
// =========================================================================

B3_API b3BodyId b3CreateBody( b3WorldId worldId, const b3BodyDef* def );
B3_API void b3DestroyBody( b3BodyId bodyId );

B3_API b3WorldId b3Body_GetWorld( b3BodyId bodyId );
B3_API void b3Body_SetUserData( b3BodyId bodyId, void* userData );
B3_API void* b3Body_GetUserData( b3BodyId bodyId );

B3_API b3Pos b3Body_GetPosition( b3BodyId bodyId );
B3_API b3Quat b3Body_GetRotation( b3BodyId bodyId );
B3_API b3WorldTransform b3Body_GetTransform( b3BodyId bodyId );
B3_API void b3Body_SetTransform( b3BodyId bodyId, b3Pos position, b3Quat rotation );

B3_API b3Vec3 b3Body_GetLocalPoint( b3BodyId bodyId, b3Pos worldPoint );
B3_API b3Pos b3Body_GetWorldPoint( b3BodyId bodyId, b3Vec3 localPoint );
B3_API b3Vec3 b3Body_GetLocalVector( b3BodyId bodyId, b3Vec3 worldVector );
B3_API b3Vec3 b3Body_GetWorldVector( b3BodyId bodyId, b3Vec3 localVector );

B3_API b3Vec3 b3Body_GetLinearVelocity( b3BodyId bodyId );
B3_API b3Vec3 b3Body_GetAngularVelocity( b3BodyId bodyId );
B3_API void b3Body_SetLinearVelocity( b3BodyId bodyId, b3Vec3 linearVelocity );
B3_API void b3Body_SetAngularVelocity( b3BodyId bodyId, b3Vec3 angularVelocity );
B3_API b3Vec3 b3Body_GetLocalPointVelocity( b3BodyId bodyId, b3Vec3 localPoint );
B3_API b3Vec3 b3Body_GetWorldPointVelocity( b3BodyId bodyId, b3Pos worldPoint );

B3_API void b3Body_ApplyForce( b3BodyId bodyId, b3Vec3 force, b3Pos point, bool wake );
B3_API void b3Body_ApplyForceToCenter( b3BodyId bodyId, b3Vec3 force, bool wake );
B3_API void b3Body_ApplyTorque( b3BodyId bodyId, b3Vec3 torque, bool wake );
B3_API void b3Body_ApplyLinearImpulse( b3BodyId bodyId, b3Vec3 impulse, b3Pos point, bool wake );
B3_API void b3Body_ApplyLinearImpulseToCenter( b3BodyId bodyId, b3Vec3 impulse, bool wake );
B3_API void b3Body_ApplyAngularImpulse( b3BodyId bodyId, b3Vec3 impulse, bool wake );

B3_API b3BodyType b3Body_GetType( b3BodyId bodyId );
B3_API void b3Body_SetType( b3BodyId bodyId, b3BodyType type );

B3_API b3f b3Body_GetMass( b3BodyId bodyId );
B3_API b3Matrix3 b3Body_GetLocalRotationalInertia( b3BodyId bodyId );
B3_API b3iw b3Body_GetInverseMass( b3BodyId bodyId );
B3_API b3MatrixW b3Body_GetWorldInverseRotationalInertia( b3BodyId bodyId );
B3_API b3Vec3 b3Body_GetLocalCenter( b3BodyId bodyId );
B3_API b3Pos b3Body_GetWorldCenter( b3BodyId bodyId );
B3_API void b3Body_SetMassData( b3BodyId bodyId, b3MassData massData );
B3_API b3MassData b3Body_GetMassData( b3BodyId bodyId );
B3_API void b3Body_ApplyMassFromShapes( b3BodyId bodyId );

B3_API void b3Body_SetLinearDamping( b3BodyId bodyId, b3f linearDamping );
B3_API b3f b3Body_GetLinearDamping( b3BodyId bodyId );
B3_API void b3Body_SetAngularDamping( b3BodyId bodyId, b3f angularDamping );
B3_API b3f b3Body_GetAngularDamping( b3BodyId bodyId );
B3_API void b3Body_SetGravityScale( b3BodyId bodyId, b3f gravityScale );
B3_API b3f b3Body_GetGravityScale( b3BodyId bodyId );

B3_API bool b3Body_IsAwake( b3BodyId bodyId );
B3_API void b3Body_SetAwake( b3BodyId bodyId, bool awake );
B3_API bool b3Body_IsEnabled( b3BodyId bodyId );
B3_API void b3Body_Disable( b3BodyId bodyId );
B3_API void b3Body_Enable( b3BodyId bodyId );
B3_API bool b3Body_IsSleepEnabled( b3BodyId bodyId );
B3_API void b3Body_EnableSleep( b3BodyId bodyId, bool enableSleep );
B3_API void b3Body_SetSleepThreshold( b3BodyId bodyId, b3f sleepThreshold );
B3_API b3f b3Body_GetSleepThreshold( b3BodyId bodyId );

B3_API void b3Body_SetMotionLocks( b3BodyId bodyId, b3MotionLocks locks );
B3_API b3MotionLocks b3Body_GetMotionLocks( b3BodyId bodyId );
B3_API void b3Body_SetBullet( b3BodyId bodyId, bool flag );
B3_API bool b3Body_IsBullet( b3BodyId bodyId );
B3_API void b3Body_AllowFastRotation( b3BodyId bodyId, bool flag );
B3_API bool b3Body_IsFastRotationAllowed( b3BodyId bodyId );
B3_API void b3Body_EnableContactRecycling( b3BodyId bodyId, bool flag );
B3_API bool b3Body_IsContactRecyclingEnabled( b3BodyId bodyId );
B3_API void b3Body_EnableHitEvents( b3BodyId bodyId, bool flag );

B3_API int b3Body_GetShapeCount( b3BodyId bodyId );
B3_API int b3Body_GetShapes( b3BodyId bodyId, b3ShapeId* shapeArray, int capacity );
B3_API int b3Body_GetJointCount( b3BodyId bodyId );
B3_API int b3Body_GetJoints( b3BodyId bodyId, b3JointId* jointArray, int capacity );
B3_API b3AABB b3Body_ComputeAABB( b3BodyId bodyId );

// =========================================================================
// Shape
// =========================================================================

B3_API b3ShapeId b3CreateSphereShape( b3BodyId bodyId, const b3ShapeDef* def, const b3Sphere* sphere );
B3_API b3ShapeId b3CreateCapsuleShape( b3BodyId bodyId, const b3ShapeDef* def, const b3Capsule* capsule );
B3_API b3ShapeId b3CreateHullShape( b3BodyId bodyId, const b3ShapeDef* def, const b3HullData* hull );

/// Attach a baked triangle mesh to a body.
///
/// `mesh` is **not copied**: there is no device mesh builder, so the blob is
/// always something baked offline, and it must outlive every shape referencing
/// it. `scale` is applied per axis on the way out of the blob, so one blob can
/// back several shapes; a negative component reflects the mesh and flips
/// triangle winding, and every component is floored at `B3_MIN_SCALE`.
///
/// A mesh shape has **zero mass and zero inertia** -- a triangle soup has no
/// volume to integrate -- so it belongs on a static or kinematic body. That is
/// upstream's behaviour, not a port limitation.
///
/// There is deliberately no function taking a `.colmesh` here. Bridging one is
/// an offline conversion to a `.b3mesh`, because building a BVH at load time on
/// a 67 MHz ARM9 is work that belongs in the asset pipeline -- and `.colmesh`
/// is itself already an offline artefact of `tools/obj2dl`.
B3_API b3ShapeId b3CreateMeshShape( b3BodyId bodyId, const b3ShapeDef* def, const b3MeshData* mesh, b3Vec3 scale );
B3_API void b3DestroyShape( b3ShapeId shapeId, bool updateBodyMass );

B3_API b3BodyId b3Shape_GetBody( b3ShapeId shapeId );
B3_API b3WorldId b3Shape_GetWorld( b3ShapeId shapeId );
B3_API void b3Shape_SetUserData( b3ShapeId shapeId, void* userData );
B3_API void* b3Shape_GetUserData( b3ShapeId shapeId );
B3_API bool b3Shape_IsSensor( b3ShapeId shapeId );

B3_API void b3Shape_SetDensity( b3ShapeId shapeId, b3f density, bool updateBodyMass );
B3_API b3f b3Shape_GetDensity( b3ShapeId shapeId );
B3_API void b3Shape_SetFriction( b3ShapeId shapeId, b3c friction );
B3_API b3c b3Shape_GetFriction( b3ShapeId shapeId );
B3_API void b3Shape_SetRestitution( b3ShapeId shapeId, b3c restitution );
B3_API b3c b3Shape_GetRestitution( b3ShapeId shapeId );
B3_API void b3Shape_SetSurfaceMaterial( b3ShapeId shapeId, b3SurfaceMaterial surfaceMaterial );
B3_API b3SurfaceMaterial b3Shape_GetSurfaceMaterial( b3ShapeId shapeId );

B3_API b3Filter b3Shape_GetFilter( b3ShapeId shapeId );
B3_API void b3Shape_SetFilter( b3ShapeId shapeId, b3Filter filter, bool invokeContacts );

B3_API void b3Shape_EnableContactEvents( b3ShapeId shapeId, bool flag );
B3_API bool b3Shape_AreContactEventsEnabled( b3ShapeId shapeId );
B3_API void b3Shape_EnableHitEvents( b3ShapeId shapeId, bool flag );
B3_API bool b3Shape_AreHitEventsEnabled( b3ShapeId shapeId );
B3_API void b3Shape_EnablePreSolveEvents( b3ShapeId shapeId, bool flag );
B3_API bool b3Shape_ArePreSolveEventsEnabled( b3ShapeId shapeId );

B3_API b3ShapeType b3Shape_GetType( b3ShapeId shapeId );
B3_API b3Sphere b3Shape_GetSphere( b3ShapeId shapeId );
B3_API b3Capsule b3Shape_GetCapsule( b3ShapeId shapeId );
B3_API const b3HullData* b3Shape_GetHull( b3ShapeId shapeId );
B3_API void b3Shape_SetSphere( b3ShapeId shapeId, const b3Sphere* sphere );
B3_API void b3Shape_SetCapsule( b3ShapeId shapeId, const b3Capsule* capsule );
B3_API void b3Shape_SetHull( b3ShapeId shapeId, const b3HullData* hull );

B3_API b3AABB b3Shape_GetAABB( b3ShapeId shapeId );
B3_API b3MassData b3Shape_ComputeMassData( b3ShapeId shapeId );

// =========================================================================
// Joint
// =========================================================================

/// Create a filter joint: the two bodies stop colliding with each other and
/// nothing else changes. No constraint is solved, so it costs one entry in the
/// joint arrays and nothing per step.
B3_API b3JointId b3CreateFilterJoint( b3WorldId worldId, const b3FilterJointDef* def );

B3_API void b3DestroyJoint( b3JointId jointId, bool wakeAttached );

B3_API b3JointType b3Joint_GetType( b3JointId jointId );
B3_API b3BodyId b3Joint_GetBodyA( b3JointId jointId );
B3_API b3BodyId b3Joint_GetBodyB( b3JointId jointId );
B3_API b3WorldId b3Joint_GetWorld( b3JointId jointId );

B3_API void b3Joint_SetLocalFrameA( b3JointId jointId, b3Transform localFrame );
B3_API b3Transform b3Joint_GetLocalFrameA( b3JointId jointId );
B3_API void b3Joint_SetLocalFrameB( b3JointId jointId, b3Transform localFrame );
B3_API b3Transform b3Joint_GetLocalFrameB( b3JointId jointId );

B3_API void b3Joint_SetCollideConnected( b3JointId jointId, bool shouldCollide );
B3_API bool b3Joint_GetCollideConnected( b3JointId jointId );

B3_API void b3Joint_SetUserData( b3JointId jointId, void* userData );
B3_API void* b3Joint_GetUserData( b3JointId jointId );

B3_API void b3Joint_SetConstraintTuning( b3JointId jointId, b3f hertz, b3f dampingRatio );
B3_API void b3Joint_GetConstraintTuning( b3JointId jointId, b3f* hertz, b3f* dampingRatio );
B3_API void b3Joint_SetForceThreshold( b3JointId jointId, b3f threshold );
B3_API b3f b3Joint_GetForceThreshold( b3JointId jointId );
B3_API void b3Joint_SetTorqueThreshold( b3JointId jointId, b3f threshold );
B3_API b3f b3Joint_GetTorqueThreshold( b3JointId jointId );

B3_API void b3Joint_WakeBodies( b3JointId jointId );

/// The force this joint is currently applying, in world space.
///
/// Zero for a filter joint, which applies none. Derived from the impulse the
/// solver accumulated, so it is meaningful only after a step -- and it is the
/// *constraint* force, not including anything the bodies' contacts did.
///
B3_API b3Vec3 b3Joint_GetConstraintForce( b3JointId jointId );

/// The torque this joint is currently applying, in world space.
///
/// Zero for a joint that constrains no rotation, which for now means the
/// filter and distance joints -- a real answer rather than an absent one.
/// Meaningful only after a step, for the same reason the force is.
B3_API b3Vec3 b3Joint_GetConstraintTorque( b3JointId jointId );

/// How far this joint has been pulled from the position it constrains, as a
/// length in metres. Zero when it is holding.
///
/// Unlike the reaction queries this reads the body transforms rather than the
/// accumulated impulses, so it is a geometric measurement and is meaningful at
/// any time, not only after a step. It is what to watch to decide whether a
/// joint is being asked to hold more than it can.
///
/// Zero is the true answer for the types that constrain no position: the motor,
/// filter and parallel joints, a distance joint that is a spring without limits,
/// and a weld joint with a linear spring. Those are all separations by design.
///
/// **The slider and wheel answers differ from Box3D's**, and the port's are
/// right. Upstream measures the off-rail offset along a single arbitrary
/// perpendicular where the constraint removes two, so it under-reports and
/// returns exactly zero for an offset perpendicular to its chosen one. The port
/// computes the true perpendicular distance. See b3PointLineSeparation.
B3_API b3f b3Joint_GetLinearSeparation( b3JointId jointId );

/// The same question about orientation, in **brads** -- the unit every other
/// angle in this API uses, where Box3D returns radians.
///
/// Zero for the types that constrain no rotation, and for a weld joint with an
/// angular spring.
///
/// A spherical joint sums its cone and twist excesses, and the wheel joint sums
/// its collinearity error with its steering-limit excess; both saturate at a
/// half turn rather than wrapping. Box3D asserts rather than answering for the
/// wheel joint -- the port answers.
B3_API b3a b3Joint_GetAngularSeparation( b3JointId jointId );

// -------------------------------------------------------------------------
// Distance joint
// -------------------------------------------------------------------------

/// Create a distance joint: two points held a set distance apart, rigidly or
/// as a spring, with an optional length range and an optional axial motor.
///
/// @see b3DistanceJointDef
B3_API b3JointId b3CreateDistanceJoint( b3WorldId worldId, const b3DistanceJointDef* def );

B3_API void b3DistanceJoint_SetLength( b3JointId jointId, b3f length );
B3_API b3f b3DistanceJoint_GetLength( b3JointId jointId );

/// The distance between the two anchor points right now, as opposed to the
/// rest length the joint is trying to reach.
B3_API b3f b3DistanceJoint_GetCurrentLength( b3JointId jointId );

B3_API void b3DistanceJoint_EnableSpring( b3JointId jointId, bool enableSpring );
B3_API bool b3DistanceJoint_IsSpringEnabled( b3JointId jointId );
B3_API void b3DistanceJoint_SetSpringHertz( b3JointId jointId, b3f hertz );
B3_API b3f b3DistanceJoint_GetSpringHertz( b3JointId jointId );
B3_API void b3DistanceJoint_SetSpringDampingRatio( b3JointId jointId, b3f dampingRatio );
B3_API b3f b3DistanceJoint_GetSpringDampingRatio( b3JointId jointId );
B3_API void b3DistanceJoint_SetSpringForceRange( b3JointId jointId, b3f lowerForce, b3f upperForce );
B3_API void b3DistanceJoint_GetSpringForceRange( b3JointId jointId, b3f* lowerForce, b3f* upperForce );

B3_API void b3DistanceJoint_EnableLimit( b3JointId jointId, bool enableLimit );
B3_API bool b3DistanceJoint_IsLimitEnabled( b3JointId jointId );
B3_API void b3DistanceJoint_SetLengthRange( b3JointId jointId, b3f minLength, b3f maxLength );
B3_API b3f b3DistanceJoint_GetMinLength( b3JointId jointId );
B3_API b3f b3DistanceJoint_GetMaxLength( b3JointId jointId );

B3_API void b3DistanceJoint_EnableMotor( b3JointId jointId, bool enableMotor );
B3_API bool b3DistanceJoint_IsMotorEnabled( b3JointId jointId );
B3_API void b3DistanceJoint_SetMotorSpeed( b3JointId jointId, b3f motorSpeed );
B3_API b3f b3DistanceJoint_GetMotorSpeed( b3JointId jointId );
B3_API void b3DistanceJoint_SetMaxMotorForce( b3JointId jointId, b3f force );
B3_API b3f b3DistanceJoint_GetMaxMotorForce( b3JointId jointId );
B3_API b3f b3DistanceJoint_GetMotorForce( b3JointId jointId );

// -------------------------------------------------------------------------
// Revolute joint
// -------------------------------------------------------------------------

/// Create a revolute joint: a hinge about the **z axis of the joint frames**.
///
/// To hinge about some other direction, rotate `base.localFrameA.q` and
/// `localFrameB.q` so their z axes point along it. There is no axis parameter,
/// which is upstream's design and the port's.
///
/// @see b3RevoluteJointDef
B3_API b3JointId b3CreateRevoluteJoint( b3WorldId worldId, const b3RevoluteJointDef* def );

/// The current hinge angle, in brads, measured from the reference state.
B3_API b3a b3RevoluteJoint_GetAngle( b3JointId jointId );

B3_API void b3RevoluteJoint_EnableSpring( b3JointId jointId, bool enableSpring );
B3_API bool b3RevoluteJoint_IsSpringEnabled( b3JointId jointId );
B3_API void b3RevoluteJoint_SetSpringHertz( b3JointId jointId, b3f hertz );
B3_API b3f b3RevoluteJoint_GetSpringHertz( b3JointId jointId );
B3_API void b3RevoluteJoint_SetSpringDampingRatio( b3JointId jointId, b3f dampingRatio );
B3_API b3f b3RevoluteJoint_GetSpringDampingRatio( b3JointId jointId );
B3_API void b3RevoluteJoint_SetTargetAngle( b3JointId jointId, b3a angle );
B3_API b3a b3RevoluteJoint_GetTargetAngle( b3JointId jointId );

B3_API void b3RevoluteJoint_EnableLimit( b3JointId jointId, bool enableLimit );
B3_API bool b3RevoluteJoint_IsLimitEnabled( b3JointId jointId );
B3_API void b3RevoluteJoint_SetLimits( b3JointId jointId, b3a lower, b3a upper );
B3_API b3a b3RevoluteJoint_GetLowerLimit( b3JointId jointId );
B3_API b3a b3RevoluteJoint_GetUpperLimit( b3JointId jointId );

B3_API void b3RevoluteJoint_EnableMotor( b3JointId jointId, bool enableMotor );
B3_API bool b3RevoluteJoint_IsMotorEnabled( b3JointId jointId );

/// Motor speed in radians per second -- an angular velocity, unlike the
/// limits, which are brads.
B3_API void b3RevoluteJoint_SetMotorSpeed( b3JointId jointId, b3f motorSpeed );
B3_API b3f b3RevoluteJoint_GetMotorSpeed( b3JointId jointId );
B3_API void b3RevoluteJoint_SetMaxMotorTorque( b3JointId jointId, b3f torque );
B3_API b3f b3RevoluteJoint_GetMaxMotorTorque( b3JointId jointId );
B3_API b3f b3RevoluteJoint_GetMotorTorque( b3JointId jointId );

// -------------------------------------------------------------------------
// Spherical joint
// -------------------------------------------------------------------------

/// Create a spherical joint: a ball joint about the joint frame **origins**.
///
/// All three rotational degrees stay free. The optional cone limit bounds how
/// far frame B's z axis may tilt away from frame A's; the optional twist limit
/// bounds the rotation about that axis. As with the revolute, the axis is
/// frame z and there is no axis parameter -- rotate `base.localFrameA.q` and
/// `localFrameB.q` to point it elsewhere.
///
/// @see b3SphericalJointDef
B3_API b3JointId b3CreateSphericalJoint( b3WorldId worldId, const b3SphericalJointDef* def );

/// The current cone angle in brads: how far frame B's z axis has tilted away
/// from frame A's. Always non-negative, and never more than half a turn.
B3_API b3a b3SphericalJoint_GetConeAngle( b3JointId jointId );

/// The current twist angle in brads, about the cone axis.
B3_API b3a b3SphericalJoint_GetTwistAngle( b3JointId jointId );

B3_API void b3SphericalJoint_EnableSpring( b3JointId jointId, bool enableSpring );
B3_API bool b3SphericalJoint_IsSpringEnabled( b3JointId jointId );
B3_API void b3SphericalJoint_SetSpringHertz( b3JointId jointId, b3f hertz );
B3_API b3f b3SphericalJoint_GetSpringHertz( b3JointId jointId );
B3_API void b3SphericalJoint_SetSpringDampingRatio( b3JointId jointId, b3f dampingRatio );
B3_API b3f b3SphericalJoint_GetSpringDampingRatio( b3JointId jointId );
B3_API void b3SphericalJoint_SetTargetRotation( b3JointId jointId, b3Quat targetRotation );
B3_API b3Quat b3SphericalJoint_GetTargetRotation( b3JointId jointId );

B3_API void b3SphericalJoint_EnableConeLimit( b3JointId jointId, bool enableLimit );
B3_API bool b3SphericalJoint_IsConeLimitEnabled( b3JointId jointId );
B3_API void b3SphericalJoint_SetConeLimit( b3JointId jointId, b3a angle );
B3_API b3a b3SphericalJoint_GetConeLimit( b3JointId jointId );

B3_API void b3SphericalJoint_EnableTwistLimit( b3JointId jointId, bool enableLimit );
B3_API bool b3SphericalJoint_IsTwistLimitEnabled( b3JointId jointId );
B3_API void b3SphericalJoint_SetTwistLimits( b3JointId jointId, b3a lower, b3a upper );
B3_API b3a b3SphericalJoint_GetLowerTwistLimit( b3JointId jointId );
B3_API b3a b3SphericalJoint_GetUpperTwistLimit( b3JointId jointId );

B3_API void b3SphericalJoint_EnableMotor( b3JointId jointId, bool enableMotor );
B3_API bool b3SphericalJoint_IsMotorEnabled( b3JointId jointId );

/// Motor angular velocity in radians per second. Unlike the revolute's, this is
/// a **3-vector**: a ball joint's motor drives all three rotational degrees.
B3_API void b3SphericalJoint_SetMotorVelocity( b3JointId jointId, b3Vec3 motorVelocity );
B3_API b3Vec3 b3SphericalJoint_GetMotorVelocity( b3JointId jointId );

/// Bounds the **magnitude** of the motor's 3-vector torque, not each axis.
B3_API void b3SphericalJoint_SetMaxMotorTorque( b3JointId jointId, b3f torque );
B3_API b3f b3SphericalJoint_GetMaxMotorTorque( b3JointId jointId );
B3_API b3Vec3 b3SphericalJoint_GetMotorTorque( b3JointId jointId );

/// Create a weld joint: two bodies held as one.
///
/// Locks the joint frame **origins** and their **orientations** together,
/// removing all six degrees of freedom. Give `linearHertz` or `angularHertz` a
/// non-zero value to soften that half into a spring; leave both at zero, which
/// is the default, for a rigid weld.
///
/// @see b3WeldJointDef
B3_API b3JointId b3CreateWeldJoint( b3WorldId worldId, const b3WeldJointDef* def );

B3_API void b3WeldJoint_SetLinearHertz( b3JointId jointId, b3f hertz );
B3_API b3f b3WeldJoint_GetLinearHertz( b3JointId jointId );
B3_API void b3WeldJoint_SetLinearDampingRatio( b3JointId jointId, b3f dampingRatio );
B3_API b3f b3WeldJoint_GetLinearDampingRatio( b3JointId jointId );

B3_API void b3WeldJoint_SetAngularHertz( b3JointId jointId, b3f hertz );
B3_API b3f b3WeldJoint_GetAngularHertz( b3JointId jointId );
B3_API void b3WeldJoint_SetAngularDampingRatio( b3JointId jointId, b3f dampingRatio );
B3_API b3f b3WeldJoint_GetAngularDampingRatio( b3JointId jointId );

/// Create a motor joint: a bounded drive rather than a constraint.
///
/// Removes no degrees of freedom. It pushes body B toward body A's joint frame
/// and toward a target relative velocity, spending at most the force and torque
/// its four bounds allow -- so it can always be overpowered by the scene. That
/// is what a moving platform wants: it goes where it is told and still loses to
/// a wall, where b3Body_SetTransform would teleport through one.
///
/// **All four bounds default to zero and a zero bound disables its branch**, so
/// a default motor joint does nothing until a drive is given a budget.
///
/// @see b3MotorJointDef
B3_API b3JointId b3CreateMotorJoint( b3WorldId worldId, const b3MotorJointDef* def );

/// Target linear velocity of B relative to A, in metres per second.
B3_API void b3MotorJoint_SetLinearVelocity( b3JointId jointId, b3Vec3 velocity );
B3_API b3Vec3 b3MotorJoint_GetLinearVelocity( b3JointId jointId );

/// Target angular velocity of B relative to A, in **radians per second** -- a
/// velocity, so it is not in brads.
B3_API void b3MotorJoint_SetAngularVelocity( b3JointId jointId, b3Vec3 velocity );
B3_API b3Vec3 b3MotorJoint_GetAngularVelocity( b3JointId jointId );

/// Each bound is on the **magnitude** of a 3-vector, not on each axis, and a
/// zero bound disables its drive. Negative values are clamped to zero.
B3_API void b3MotorJoint_SetMaxVelocityForce( b3JointId jointId, b3f maxForce );
B3_API b3f b3MotorJoint_GetMaxVelocityForce( b3JointId jointId );
B3_API void b3MotorJoint_SetMaxVelocityTorque( b3JointId jointId, b3f maxTorque );
B3_API b3f b3MotorJoint_GetMaxVelocityTorque( b3JointId jointId );
B3_API void b3MotorJoint_SetMaxSpringForce( b3JointId jointId, b3f maxForce );
B3_API b3f b3MotorJoint_GetMaxSpringForce( b3JointId jointId );
B3_API void b3MotorJoint_SetMaxSpringTorque( b3JointId jointId, b3f maxTorque );
B3_API b3f b3MotorJoint_GetMaxSpringTorque( b3JointId jointId );

/// The two springs run only when their hertz **and** their bound are non-zero.
B3_API void b3MotorJoint_SetLinearHertz( b3JointId jointId, b3f hertz );
B3_API b3f b3MotorJoint_GetLinearHertz( b3JointId jointId );
B3_API void b3MotorJoint_SetLinearDampingRatio( b3JointId jointId, b3f dampingRatio );
B3_API b3f b3MotorJoint_GetLinearDampingRatio( b3JointId jointId );
B3_API void b3MotorJoint_SetAngularHertz( b3JointId jointId, b3f hertz );
B3_API b3f b3MotorJoint_GetAngularHertz( b3JointId jointId );
B3_API void b3MotorJoint_SetAngularDampingRatio( b3JointId jointId, b3f dampingRatio );
B3_API b3f b3MotorJoint_GetAngularDampingRatio( b3JointId jointId );

// -------------------------------------------------------------------------
// Prismatic joint
// -------------------------------------------------------------------------

/// Create a prismatic joint: a slider along the **x axis of the joint frames**.
///
/// One translational degree of freedom; everything else is locked, including
/// all three rotations. To slide along some other direction, rotate
/// `base.localFrameA.q` and `localFrameB.q` so their x axes point along it.
/// There is no axis parameter, which is upstream's design and the port's — and
/// note the axis is **x** here where the revolute's hinge is z.
///
/// Every quantity is a length in metres or a speed in metres per second, at the
/// same Q12 as every other position in the port. So unlike the revolute there
/// is no brad-versus-radian split to keep straight.
///
/// The translation therefore has the same absolute resolution (1/4096 of a
/// unit) wherever the rail sits — a slider 1500 units from the origin reads
/// exactly as accurately as one at the origin, and needs no special handling.
/// What is bounded is the world's extent, not the precision; keep it inside
/// `B3_HUGE` and see `prismatic_joint.c`'s header for the full trade against
/// upstream, which needs a `double` here and the port does not.
///
/// @see b3PrismaticJointDef
B3_API b3JointId b3CreatePrismaticJoint( b3WorldId worldId, const b3PrismaticJointDef* def );

/// The current translation along the slide axis, in metres, measured from the
/// reference state.
B3_API b3f b3PrismaticJoint_GetTranslation( b3JointId jointId );

/// The current speed along the slide axis, in metres per second. Accounts for
/// body A's rotation, since the axis is fixed in A.
B3_API b3f b3PrismaticJoint_GetSpeed( b3JointId jointId );

B3_API void b3PrismaticJoint_EnableSpring( b3JointId jointId, bool enableSpring );
B3_API bool b3PrismaticJoint_IsSpringEnabled( b3JointId jointId );
B3_API void b3PrismaticJoint_SetSpringHertz( b3JointId jointId, b3f hertz );
B3_API b3f b3PrismaticJoint_GetSpringHertz( b3JointId jointId );
B3_API void b3PrismaticJoint_SetSpringDampingRatio( b3JointId jointId, b3f dampingRatio );
B3_API b3f b3PrismaticJoint_GetSpringDampingRatio( b3JointId jointId );

/// The translation the spring pulls toward, in metres.
B3_API void b3PrismaticJoint_SetTargetTranslation( b3JointId jointId, b3f targetTranslation );
B3_API b3f b3PrismaticJoint_GetTargetTranslation( b3JointId jointId );

B3_API void b3PrismaticJoint_EnableLimit( b3JointId jointId, bool enableLimit );
B3_API bool b3PrismaticJoint_IsLimitEnabled( b3JointId jointId );

/// Limits in metres. Passed either way round: they are sorted, not asserted.
///
/// **Both default to zero**, so enabling the limit without setting a range is a
/// slider locked shut. See b3PrismaticJointDef::lowerTranslation for why the
/// default is not the distance joint's B3_HUGE.
B3_API void b3PrismaticJoint_SetLimits( b3JointId jointId, b3f lower, b3f upper );
B3_API b3f b3PrismaticJoint_GetLowerLimit( b3JointId jointId );
B3_API b3f b3PrismaticJoint_GetUpperLimit( b3JointId jointId );

B3_API void b3PrismaticJoint_EnableMotor( b3JointId jointId, bool enableMotor );
B3_API bool b3PrismaticJoint_IsMotorEnabled( b3JointId jointId );

/// Motor speed in metres per second, and its force budget in newtons. Negative
/// budgets are clamped to zero.
B3_API void b3PrismaticJoint_SetMotorSpeed( b3JointId jointId, b3f motorSpeed );
B3_API b3f b3PrismaticJoint_GetMotorSpeed( b3JointId jointId );
B3_API void b3PrismaticJoint_SetMaxMotorForce( b3JointId jointId, b3f force );
B3_API b3f b3PrismaticJoint_GetMaxMotorForce( b3JointId jointId );
B3_API b3f b3PrismaticJoint_GetMotorForce( b3JointId jointId );

// ---- Parallel joint ----

/// Create a parallel joint: an upright-keeper.
///
/// Holds body B's z axis aligned with body A's z axis using a spring, and
/// leaves the twist about z free. It constrains **orientation only** -- the two
/// bodies may sit anywhere relative to one another -- which is what separates it
/// from a weld joint and what makes it the right choice for a mast that should
/// stay vertical while its platform pitches, or a turret that should stay level.
///
/// The constraint is always soft and always bounded by `maxTorque`. That is the
/// design rather than a shortcoming: a large enough disturbance wins and the
/// body tips, and the spring then rights it.
///
/// **A default def does nothing**, because `maxTorque` defaults to zero. See
/// b3ParallelJointDef::maxTorque for why the port cannot use upstream's
/// FLT_MAX.
///
/// @see b3ParallelJointDef
B3_API b3JointId b3CreateParallelJoint( b3WorldId worldId, const b3ParallelJointDef* def );

/// Spring stiffness in cycles per second, and its damping ratio.
///
/// Unlike every other joint's spring there is no enable flag: a parallel joint
/// is a spring and nothing else, so a zero frequency is how it is turned off.
B3_API void b3ParallelJoint_SetSpringHertz( b3JointId jointId, b3f hertz );
B3_API b3f b3ParallelJoint_GetSpringHertz( b3JointId jointId );
B3_API void b3ParallelJoint_SetSpringDampingRatio( b3JointId jointId, b3f dampingRatio );
B3_API b3f b3ParallelJoint_GetSpringDampingRatio( b3JointId jointId );

/// The torque budget in newton-metres. Negative budgets are clamped to zero,
/// and zero means the joint applies nothing at all.
B3_API void b3ParallelJoint_SetMaxTorque( b3JointId jointId, b3f maxTorque );
B3_API b3f b3ParallelJoint_GetMaxTorque( b3JointId jointId );

// ---- Wheel joint ----

/// Create a wheel joint: a suspension, a spin axis and a steering axis in one.
///
/// The largest joint in the library and the only one that is two joints at
/// once — a prismatic carrying the suspension and a revolute carrying the
/// wheel's spin, coupled by a steering twist.
///
/// **The axis convention decides how you build the frames.** The suspension
/// travels along frame A's **x**, so a vertical suspension needs frame A
/// rotated to put x where the travel should go — exactly as
/// `b3CreatePrismaticJoint` does. The wheel spins about frame **B's z**, not
/// A's, so steering carries the spin axis with it.
///
/// A default def gives a sprung suspension with no travel limit, a rigid
/// steering lock and both motors at zero budget. See b3WheelJointDef.
///
/// @see b3WheelJointDef
B3_API b3JointId b3CreateWheelJoint( b3WorldId worldId, const b3WheelJointDef* def );

/// The suspension spring: enable, stiffness in cycles per second, damping ratio.
B3_API void b3WheelJoint_EnableSuspensionSpring( b3JointId jointId, bool enableSpring );
B3_API bool b3WheelJoint_IsSuspensionSpringEnabled( b3JointId jointId );
B3_API void b3WheelJoint_SetSuspensionHertz( b3JointId jointId, b3f hertz );
B3_API b3f b3WheelJoint_GetSuspensionHertz( b3JointId jointId );
B3_API void b3WheelJoint_SetSuspensionDampingRatio( b3JointId jointId, b3f dampingRatio );
B3_API b3f b3WheelJoint_GetSuspensionDampingRatio( b3JointId jointId );

/// The suspension travel limit, in **metres** — a length, not an angle.
///
/// `SetSuspensionLimits` sorts its arguments and zeroes both limit impulses, so
/// a caller who passes them the wrong way round gets the range they meant.
B3_API void b3WheelJoint_EnableSuspensionLimit( b3JointId jointId, bool enableLimit );
B3_API bool b3WheelJoint_IsSuspensionLimitEnabled( b3JointId jointId );
B3_API void b3WheelJoint_SetSuspensionLimits( b3JointId jointId, b3f lower, b3f upper );
B3_API b3f b3WheelJoint_GetLowerSuspensionLimit( b3JointId jointId );
B3_API b3f b3WheelJoint_GetUpperSuspensionLimit( b3JointId jointId );

/// The spin motor that drives the wheel. Speed is in radians per second — a
/// velocity, so it is not in brads. Negative torque budgets clamp to zero.
B3_API void b3WheelJoint_EnableSpinMotor( b3JointId jointId, bool enableMotor );
B3_API bool b3WheelJoint_IsSpinMotorEnabled( b3JointId jointId );
B3_API void b3WheelJoint_SetSpinMotorSpeed( b3JointId jointId, b3f speed );
B3_API b3f b3WheelJoint_GetSpinMotorSpeed( b3JointId jointId );
B3_API void b3WheelJoint_SetMaxSpinTorque( b3JointId jointId, b3f torque );
B3_API b3f b3WheelJoint_GetMaxSpinTorque( b3JointId jointId );

/// Steering. With it disabled the wheel is held pointing forward by a rigid
/// collinearity constraint; with it enabled the wheel steers about frame A's x
/// under a spring, a torque budget and an optional angle limit.
B3_API void b3WheelJoint_EnableSteering( b3JointId jointId, bool enableSteering );
B3_API bool b3WheelJoint_IsSteeringEnabled( b3JointId jointId );
B3_API void b3WheelJoint_SetSteeringHertz( b3JointId jointId, b3f hertz );
B3_API b3f b3WheelJoint_GetSteeringHertz( b3JointId jointId );
B3_API void b3WheelJoint_SetSteeringDampingRatio( b3JointId jointId, b3f dampingRatio );
B3_API b3f b3WheelJoint_GetSteeringDampingRatio( b3JointId jointId );
B3_API void b3WheelJoint_SetMaxSteeringTorque( b3JointId jointId, b3f torque );
B3_API b3f b3WheelJoint_GetMaxSteeringTorque( b3JointId jointId );

/// The steering angles, all in **brads** — 32768 to a full turn, so
/// B3_BRAD_HALF_PI is a right angle. Upstream uses radians.
B3_API void b3WheelJoint_SetTargetSteeringAngle( b3JointId jointId, b3a angle );
B3_API b3a b3WheelJoint_GetTargetSteeringAngle( b3JointId jointId );
B3_API void b3WheelJoint_EnableSteeringLimit( b3JointId jointId, bool enableLimit );
B3_API bool b3WheelJoint_IsSteeringLimitEnabled( b3JointId jointId );
B3_API void b3WheelJoint_SetSteeringLimits( b3JointId jointId, b3a lower, b3a upper );
B3_API b3a b3WheelJoint_GetLowerSteeringLimit( b3JointId jointId );
B3_API b3a b3WheelJoint_GetUpperSteeringLimit( b3JointId jointId );

/// Derived readouts, all valid outside a step.
///
/// The suspension travel in metres, the wheel's spin rate in radians per
/// second, the steering angle in brads, and the two torques the joint is
/// applying about its spin and steering axes.
B3_API b3f b3WheelJoint_GetSuspensionTranslation( b3JointId jointId );
B3_API b3f b3WheelJoint_GetSpinSpeed( b3JointId jointId );
B3_API b3a b3WheelJoint_GetSteeringAngle( b3JointId jointId );
B3_API b3f b3WheelJoint_GetSpinTorque( b3JointId jointId );
B3_API b3f b3WheelJoint_GetSteeringTorque( b3JointId jointId );

// =========================================================================
// Contact
// =========================================================================

/// Read a contact's shape ids and manifolds through its public id.
B3_API b3ContactData b3Contact_GetData( b3ContactId contactId );

#endif // BOX3D_BOX3D_H__
