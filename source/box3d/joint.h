// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Erin Catto        (original Box3D)
// Copyright (c) 2026 Warioware64       (Nitro Engine Advanced fixed-point port)
//
// This file is part of Nitro Engine Advanced

#pragma once

/// @file   joint.h
/// @brief  Joint records and the generic joint plumbing.
///
/// @section why Why this existed before there were joints
///
/// `b3Body` carries a joint list, solver sets carry joint sims, islands count
/// joints, and b3Body_SetType and b3Body_Disable both walk the joint list --
/// so the *plumbing* was threaded through every file Phase 3A touched.
/// Stripping it out would have meant editing all of them again here, and would
/// have left the port's structure looking nothing like upstream's in between.
/// Through Phases 3A-5 `world->joints` was always empty and every loop over it
/// was a no-op: deliberately the only unexercised code in the port.
///
/// @section stage1 What Phase 6 Stage 1 added
///
/// The generic layer that turns that plumbing on: b3CreateJoint, b3DestroyJoint
/// and the b3Joint_* accessors in joint.c, the three type dispatchers, and
/// b3_filterJoint -- the one joint type with no constraint to solve, so the
/// plumbing gets exercised end to end before any fixed-point constraint math
/// exists to be blamed for a failure.
///
/// @section stage2 What Phase 6 Stage 2 added
///
/// The distance joint, and with it the two things Stage 1 had no way to land:
/// the b3JointSim per-type union, and the reaction *force* query -- both of
/// which need a joint that actually solves something.
///
/// @section stage3 What Phase 6 Stage 3 added
///
/// The revolute joint -- a hinge, and the first joint that constrains a
/// *rotation*. Three constraints in one: a 3x3 point-to-point lock on the
/// anchors, a 2x2 collinearity lock on the axes, and the scalar axial degree
/// the spring, motor and angle limit act on. Both matrix effective masses are
/// solved with inverses Phase 3C already wrote for the contact solver
/// (b3InvertMatrixW, b3InvertSym2W), so this stage adds constraint math
/// rather than linear algebra.
///
/// With it, the reaction *torque* query -- which Stages 1 and 2 both deferred
/// for the same reason, that nothing yet constrained a rotation to report on.
///
/// @section stage4 What Phase 6 Stage 4 added
///
/// The spherical joint -- a ball joint, and the first whose spring and motor
/// are *3-vectors* rather than scalars: a spring toward a target orientation
/// and a motor driving a relative angular velocity both act on all three
/// rotational degrees at once. Its point-to-point constraint is the revolute's
/// unchanged; what is new is the pair of one-sided rotational bounds -- a cone
/// limit on how far the axes may tilt apart and a twist limit about them --
/// and, with the twist, the port's one genuinely unbounded Jacobian.
///
/// Still absent: the five other solved joint types, the linear and angular
/// separation queries (upstream writes each as one switch over all nine types,
/// and growing that a case at a time is worse than writing it once when the
/// set is complete), and joint events.

#include "solver.h"

#include "box3d/id.h"
#include "box3d/math_fixed.h"
#include "box3d/types.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct b3World b3World;

// b3JointType lived here through Phases 3A-5, while it was an implementation
// detail of the plumbing. It is public API the moment b3Joint_GetType exists,
// so Stage 1 moved it to include/box3d/types.h unchanged, order and all.

/// One end of a joint in a body's doubly linked joint list. Each joint has two.
typedef struct b3JointEdge
{
	int bodyId;
	int prevKey;
	int nextKey;
} b3JointEdge;

/// Maps a b3JointId to a joint sim living in a solver set or a graph colour.
typedef struct b3Joint
{
	void* userData;

	/// Index of the solver set holding the sim. B3_NULL_INDEX when free.
	int setIndex;

	/// Index into the constraint graph colour array. B3_NULL_INDEX for a
	/// sleeping or disabled joint, and when free.
	int colorIndex;

	/// Joint index within its set or graph colour. B3_NULL_INDEX when free.
	int localIndex;

	b3JointEdge edges[2];

	int jointId;
	int islandId;

	/// Index into the island's joints array, for O(1) swap removal.
	int islandIndex;

	b3JointType type;

	/// Monotonically advanced when a joint is allocated in this slot, so a
	/// stale b3JointId can be detected.
	uint16_t generation;

	bool collideConnected;
} b3Joint;

/// The distance joint's per-type state.
///
/// Upstream's is fourteen floats and three vectors; the port's differ only in
/// scale, and the scales are the stage's whole point:
///
///   - lengths (`length`, `minLength`, `maxLength`) are b3f, like every other
///     position in the port.
///   - `hertz` and `dampingRatio` are b3f, not b3c, for the same reason
///     b3JointSim::constraintDampingRatio is: b3MakeSoft takes b3f, and a
///     damping ratio is routinely above one.
///   - forces (`lowerSpringForce`, `upperSpringForce`, `maxMotorForce`) and
///     `motorSpeed` are b3f.
///   - the four accumulators are b3imp. They are warm-started, so they must
///     live at the impulse scale rather than being re-derived each step.
///   - `axialMass` is b3f: it is b3RcpW of a Q24 inverse-mass sum, exactly as
///     b3ManifoldConstraintPoint::normalMass is.
typedef struct b3DistanceJoint
{
	b3f length;
	b3f hertz;
	b3f dampingRatio;
	b3f lowerSpringForce;
	b3f upperSpringForce;
	b3f minLength;
	b3f maxLength;

	b3f maxMotorForce;
	b3f motorSpeed;

	b3imp impulse;
	b3imp lowerImpulse;
	b3imp upperImpulse;
	b3imp motorImpulse;

	int indexA;
	int indexB;
	b3Vec3 anchorA;
	b3Vec3 anchorB;
	b3Vec3 deltaCenter;
	b3Softness distanceSoftness;
	b3f axialMass;

	bool enableSpring;
	bool enableLimit;
	bool enableMotor;
} b3DistanceJoint;

/// The revolute joint's per-type state.
///
/// A hinge is three constraints rather than one, and the field groups below
/// follow that split:
///
///   - `linearImpulse` is the 3x3 point-to-point constraint that locks the two
///     anchors together, killing all linear freedom;
///   - `perpImpulse` is the 2x2 collinearity constraint that holds the two
///     hinge axes parallel, killing two of the three rotational degrees;
///   - the four scalar accumulators carry the spring, motor and angle limit on
///     the one degree of freedom that is left.
///
/// Scales follow the distance joint's, with three worth stating:
///
///   - the three angles are b3a brads, the port's angle type everywhere a
///     caller sees one. The constraint needs radians, so b3BradToRadF converts
///     the *difference* against a limit -- never each angle separately.
///   - `motorSpeed` is a b3f, because it is an angular *velocity* in rad/s and
///     shares its units and scale with b3BodyState::angularVelocity.
///   - `perpImpulse` is a b3Imp2, which is both the port's two-impulse type
///     and exactly what b3MulSym2V returns.
typedef struct b3RevoluteJoint
{
	b3Imp3 linearImpulse;
	b3Imp2 perpImpulse;
	b3imp springImpulse;
	b3imp motorImpulse;
	b3imp lowerImpulse;
	b3imp upperImpulse;

	b3f hertz;
	b3f dampingRatio;
	b3f maxMotorTorque;
	b3f motorSpeed;

	b3a targetAngle;
	b3a lowerAngle;
	b3a upperAngle;

	int indexA;
	int indexB;

	/// Joint frames in world space, relative to each centre of mass.
	b3Transform frameA;
	b3Transform frameB;

	/// The hinge axis, and the two perpendiculars the collinearity constraint
	/// acts along. All three are recomputed in the solve; they are cached here
	/// because warm start runs before it and needs them.
	b3Vec3 rotationAxisZ;
	b3Vec3 perpAxisX;
	b3Vec3 perpAxisY;

	b3Vec3 deltaCenter;
	b3f axialMass;
	b3Softness springSoftness;

	bool enableSpring;
	bool enableMotor;
	bool enableLimit;
} b3RevoluteJoint;

/// The spherical joint's per-type state.
///
/// A ball joint. Where the revolute leaves one rotational degree free, this
/// leaves all three, and constrains them by *bounding* rather than by locking:
///
///   - `linearImpulse` is the same 3x3 point-to-point constraint the revolute
///     uses, unchanged -- it locks the two anchors and kills all linear
///     freedom;
///   - `springImpulse` and `motorImpulse` are 3-vectors, because a spring
///     toward a target *orientation* and a motor driving a relative *angular
///     velocity* both act on all three rotational degrees at once. This is the
///     first joint whose spring and motor are not scalars;
///   - `swingImpulse` bounds how far frame B's z may tilt away from frame A's
///     (the cone limit), and the two twist accumulators bound the rotation
///     about that axis. All three are one-sided, like the revolute's limits.
///
/// Scales follow the revolute's, with three worth stating:
///
///   - `swingAxis` and `twistJacobian` are Q12 b3Vec3, not Q30 b3Dir3, for the
///     reason revolute_joint.c gives for `rotationAxisZ`: each is used as both
///     the mass direction in prepare and the impulse direction in the solve, so
///     the two Q12 roundings are the same rounding and largely cancel.
///     `twistJacobian` could not be a b3Dir3 in any case -- it is not a unit
///     vector, see below.
///   - `motorVelocity` is a b3Vec3 of rad/s, sharing units and scale with
///     b3BodyState::angularVelocity. It is a velocity, not an angle, so it is
///     not b3a -- the same call Stage 3 made for the revolute's `motorSpeed`.
///   - the three limit angles are b3a brads, and b3BradToRadF converts the
///     *difference* against a limit, never each angle separately.
///
/// @section jacobian Why twistJacobian is not a unit vector
///
/// The twist constraint acts along `coneAxis + tan(theta/2) * perpAxis`, where
/// theta is the swing angle. That tangent grows without bound as the swing
/// approaches 180 degrees, which upstream absorbs in float's exponent and Q12
/// cannot. b3SphericalTwistJacobian caps it; see the comment there for what the
/// cap costs and why a cap is the right answer rather than a larger scale.
typedef struct b3SphericalJoint
{
	b3Imp3 linearImpulse;
	b3Imp3 springImpulse;
	b3Imp3 motorImpulse;
	b3imp lowerTwistImpulse;
	b3imp upperTwistImpulse;
	b3imp swingImpulse;

	b3f hertz;
	b3f dampingRatio;
	b3f maxMotorTorque;
	b3Vec3 motorVelocity;

	b3a lowerTwistAngle;
	b3a upperTwistAngle;
	b3a coneAngle;
	b3Quat targetRotation;

	int indexA;
	int indexB;

	/// Joint frames in world space, relative to each centre of mass.
	b3Transform frameA;
	b3Transform frameB;

	b3Vec3 deltaCenter;

	/// The cone limit's axis and the twist limit's axis. Both are recomputed
	/// from the frames in prepare and cached here because warm start runs
	/// before the solve and needs them, exactly as the revolute's axes are.
	///
	/// Both are **unit** vectors. Upstream's twist Jacobian is not -- it is
	/// `coneAxis + tan(theta/2) * perpAxis`, which diverges as the swing
	/// approaches half a turn. See spherical_joint.c's header for why the port
	/// normalizes it and what `twistScale` then has to carry.
	b3Vec3 swingAxis;
	b3Vec3 twistAxis;

	/// `cos(theta/2)`, the factor the twist limit's position error is scaled by
	/// to undo the normalization of `twistAxis`. A velocity constraint does not
	/// care how its Jacobian is scaled; a position bias does.
	b3c twistScale;

	/// The inverse of (invIA + invIB): the effective mass the 3-vector spring
	/// and motor both push through. Zero when base->fixedRotation.
	b3Matrix3 rotationMass;
	b3f swingMass;
	b3f twistMass;
	b3Softness springSoftness;

	bool enableSpring;
	bool enableMotor;
	bool enableConeLimit;
	bool enableTwistLimit;
} b3SphericalJoint;

/// The weld joint's per-type state.
///
/// A rigid lock: the revolute's point-to-point constraint on the two anchors,
/// plus a 3-vector angular constraint holding the two frames' *orientations*
/// together. Where the spherical joint leaves all three rotational degrees free
/// and bounds them, this one removes them.
///
/// Two accumulators and no limits, which makes it the smallest joint since the
/// distance joint despite constraining the most:
///
///   - `linearImpulse` is the 3x3 point-to-point constraint, identical to the
///     revolute's and the spherical's;
///   - `angularImpulse` is the 3-vector orientation constraint, which is the
///     spherical spring's shape with an identity target and no bound.
///
/// Both branches optionally *soften* into springs. At zero hertz each falls
/// back to `base->constraintSoftness` and the joint is rigid; at a non-zero
/// hertz it becomes a spring of that frequency, which is upstream's `if/else`
/// in prepare and needs no port change -- b3MakeSoft already returns all-zero
/// at zero hertz.
///
/// Scales follow the spherical's, with one worth stating: the four hertz and
/// damping fields are b3f rather than b3c for the reason b3DistanceJoint's are
/// -- b3MakeSoft takes b3f, and a damping ratio is routinely above one.
typedef struct b3WeldJoint
{
	b3Imp3 linearImpulse;
	b3Imp3 angularImpulse;

	b3f linearHertz;
	b3f linearDampingRatio;
	b3f angularHertz;
	b3f angularDampingRatio;

	int indexA;
	int indexB;

	/// Joint frames in world space, relative to each centre of mass.
	b3Transform frameA;
	b3Transform frameB;

	b3Vec3 deltaCenter;

	/// The inverse of (invIA + invIB), which the angular constraint pushes
	/// through. Zero when base->fixedRotation.
	b3Matrix3 angularMass;

	/// Recomputed every prepare: `constraintSoftness` when the matching hertz
	/// is zero, a spring of that frequency otherwise.
	b3Softness linearSpring;
	b3Softness angularSpring;
} b3WeldJoint;

/// The motor joint's per-type state.
///
/// A soft 6-DOF drive with no position lock at all. Where the weld joint holds
/// two frames together, this one *pushes* them toward a target -- and every
/// branch is bounded by the magnitude of the force or torque it may use, so it
/// can always be overpowered by the scene rather than winning against it. That
/// is what makes it the joint a moving platform wants: it goes where it is
/// told, and it still loses to a wall.
///
/// Four independent branches, which is the weld's two split into a spring half
/// and a velocity half:
///
///   - `linearSpringImpulse` / `angularSpringImpulse` drive toward the target
///     *pose*, bounded by `maxSpringForce` / `maxSpringTorque`;
///   - `linearVelocityImpulse` / `angularVelocityImpulse` drive toward the
///     target *velocity*, bounded by `maxVelocityForce` / `maxVelocityTorque`.
///
/// All four bounds are on the **magnitude** of a 3-vector, so each is a sphere
/// rather than an interval and b3ClampImp3 is what applies it -- the spherical
/// motor's rule, and for the same reason: clamping each component separately
/// would let a diagonal drive exceed its budget by sqrt(3).
///
/// `linearVelocity` and `angularVelocity` are b3Vec3 of m/s and rad/s. They are
/// velocities rather than positions or angles, so they share their units and
/// scale with b3BodyState's -- the call Stage 3 made for the revolute's
/// `motorSpeed` and Stage 4 for the spherical's `motorVelocity`.
typedef struct b3MotorJoint
{
	b3Imp3 linearVelocityImpulse;
	b3Imp3 angularVelocityImpulse;
	b3Imp3 linearSpringImpulse;
	b3Imp3 angularSpringImpulse;

	b3Vec3 linearVelocity;
	b3Vec3 angularVelocity;

	b3f maxVelocityForce;
	b3f maxVelocityTorque;
	b3f maxSpringForce;
	b3f maxSpringTorque;

	b3f linearHertz;
	b3f linearDampingRatio;
	b3f angularHertz;
	b3f angularDampingRatio;

	int indexA;
	int indexB;

	/// Joint frames in world space, relative to each centre of mass.
	b3Transform frameA;
	b3Transform frameB;

	b3Vec3 deltaCenter;

	/// The inverse of (invIA + invIB). Both angular branches push through it.
	/// Zero when base->fixedRotation.
	b3Matrix3 angularMass;

	b3Softness linearSpring;
	b3Softness angularSpring;
} b3MotorJoint;

/// The prismatic joint's per-type state.
///
/// A slider. Where the revolute leaves one *rotational* degree free, this
/// leaves one *translational* one, and the split of the constraints follows:
///
///   - `perpImpulse` is the 2x2 point-to-line constraint holding the anchor on
///     the rail, which kills two of the three linear degrees;
///   - `angularImpulse` is the 3-vector orientation lock, the weld's block with
///     an identity target -- a slider does not turn;
///   - the four scalar accumulators carry the spring, motor and both
///     translation limits on the one degree that is left.
///
/// Scales, with three worth stating:
///
///   - **the three translations are b3f metres, not b3a brads.** This is the
///     revolute with its unit problem deleted: a translation limit is a length
///     in the same Q12 as every position in the port, so b3BradToRadF has no
///     counterpart here and no conversion happens at the constraint. Worth
///     saying plainly so a reader does not go looking for one.
///   - `motorSpeed` is a b3f in metres per second, sharing units and scale with
///     b3BodyState::linearVelocity, exactly as the revolute's shares them with
///     the angular velocity.
///   - `perpImpulse` is a b3Imp2, which is both the port's two-impulse type and
///     what b3MulSym2V returns.
///
/// @section nomass Why there is no cached axialMass
///
/// Every joint before this one computes its effective mass in prepare and holds
/// it for the step, because the geometry it depends on is fixed there. A
/// prismatic's is not: its lever arms are `cross( rA + d, axis )` and
/// `cross( rB, axis )`, and `d` is the slide translation, so the mass is a
/// function of where the slider currently *is*. Upstream recomputes it every
/// solve -- "must be fresh to avoid divergence when the joint is stressed" --
/// and the port keeps that. Not caching it makes the invariant unforgeable
/// rather than merely documented, and it is why b3LeverInertiaSumWide and
/// b3InvertPointLineMass sit on the hot path instead of in prepare.
typedef struct b3PrismaticJoint
{
	b3Imp2 perpImpulse;
	b3Imp3 angularImpulse;
	b3imp springImpulse;
	b3imp motorImpulse;
	b3imp lowerImpulse;
	b3imp upperImpulse;

	b3f hertz;
	b3f dampingRatio;
	b3f maxMotorForce;
	b3f motorSpeed;

	b3f targetTranslation;
	b3f lowerTranslation;
	b3f upperTranslation;

	int indexA;
	int indexB;

	/// Joint frames in world space, relative to each centre of mass.
	b3Transform frameA;
	b3Transform frameB;

	/// The slide axis and the two perpendiculars, all three fixed in body A.
	/// Recomputed against the sub-step rotation in the solve; cached here
	/// because warm start runs before it and needs them, as the revolute's are.
	b3Vec3 jointAxis;
	b3Vec3 perpAxisY;
	b3Vec3 perpAxisZ;

	b3Vec3 deltaCenter;

	/// The inverse of (invIA + invIB), which the orientation lock pushes
	/// through. Zero when base->fixedRotation.
	b3Matrix3 rotationMass;

	b3Softness springSoftness;

	bool enableSpring;
	bool enableMotor;
	bool enableLimit;
} b3PrismaticJoint;

/// A parallel joint's simulation data: the smallest solved joint in the port.
///
/// One 2x2 angular constraint holding B's z parallel to A's z, always soft and
/// always bounded. There is **no linear constraint at all**, which is why this
/// struct has no b3Transform frames, no anchor offsets and no `deltaCenter` --
/// only the two frame rotations. That absence is most of why it is 96 bytes
/// against the ball joint's 260.
///
/// `perpImpulse` is a b3Imp2 because that is both what b3MulSym2V returns and
/// what b3ClampImp2 bounds. The clamp is a *disc*: see parallel_joint.c's
/// header for why a per-component bound would be wrong here.
///
/// The two cached axes are not a redundant copy of what the solve computes.
/// Warm start runs first and must replay the stored impulse along the axes it
/// was accumulated on, which is the same duplication the revolute documents.
typedef struct b3ParallelJoint
{
	b3Imp2 perpImpulse;

	b3f hertz;
	b3f dampingRatio;
	b3f maxTorque;

	int indexA;
	int indexB;

	/// Joint frame rotations in world space. No positions: nothing here needs
	/// an anchor.
	b3Quat quatA;
	b3Quat quatB;

	/// The two collinearity axes, from b3CollinearityPerpAxes. Cached for warm
	/// start; rebuilt in the solve against the sub-step rotation.
	b3Vec3 perpAxisX;
	b3Vec3 perpAxisY;

	/// The joint's own spring. Never base->constraintSoftness -- a parallel
	/// joint has no rigid mode, the constraint *is* the spring.
	b3Softness softness;
} b3ParallelJoint;

/// A wheel joint's simulation data: a prismatic and a revolute in one.
///
/// @section nomass Why there are no cached effective masses
///
/// Upstream caches three -- `spinMass`, `suspensionMass`, `steeringMass` --
/// and the port keeps none of them. This is not the prismatic's staleness
/// argument repeated; it is a stronger one, because upstream's cached
/// `suspensionMass` is built from a **different Jacobian than the one
/// applied**. Prepare forms its arms as `cross( rA, axis )` while the solve
/// applies its impulses along `cross( d + rA, axis )`, omitting the suspension
/// travel from body A's arm entirely. The prismatic's cached version at least
/// used the right arms and only went stale; this one is wrong from the first
/// sub-step, and upstream's own `// todo use fresh effective masses` sits four
/// lines above it.
///
/// `steeringMass` is worse still to cache, because the solve recomputes the
/// steering *axis* anyway -- caching the mass while rebuilding the axis it was
/// derived from is strictly worse than rebuilding both. `spinMass` is the one
/// genuinely stable case, and caching one of three is a worse invariant than
/// caching none.
///
/// The recompute is nearly free: `sAx` and `sBx` are already formed for the
/// Jacobian, so it costs one b3LeverInertiaSumWide and one b3RcpWide and
/// *removes* two prepare-time cross products.
///
/// **Size did not decide this.** Caching all three would put the struct at
/// about 208 bytes, still inside the ball joint's 260 and still free. The
/// recompute was chosen for correctness.
///
/// @section noaxes Why there are no cached axes either
///
/// The revolute and prismatic cache theirs because warm start runs before the
/// solve and needs the axes the stored impulse was accumulated along. The
/// wheel's warm start rebuilds every axis itself from `frameA`/`frameB` and the
/// sub-step rotation, so there is nothing to cache; the reaction readouts do
/// the same from the body transforms, as b3WheelJoint_GetSteeringAngle already
/// must. Those two absences are most of why the largest joint in the library is
/// smaller than the ball joint.
typedef struct b3WheelJoint
{
	/// Point-to-line, in frame A's (y, z).
	b3Imp2 linearImpulse;

	/// Collinearity. Only `.x` is used when steering is enabled, where the
	/// block collapses from a 2x2 to a single axis.
	b3Imp2 angularImpulse;

	b3imp spinImpulse;
	b3imp suspensionSpringImpulse;
	b3imp lowerSuspensionImpulse;
	b3imp upperSuspensionImpulse;
	b3imp steeringSpringImpulse;
	b3imp lowerSteeringImpulse;
	b3imp upperSteeringImpulse;

	b3f maxSpinTorque;

	/// Radians per second, sharing units and scale with
	/// b3BodyState::angularVelocity -- a velocity, not an angle, so not a b3a.
	b3f spinSpeed;

	/// Suspension travel limits, in metres. The linear half of the joint.
	b3f lowerSuspensionLimit;
	b3f upperSuspensionLimit;

	b3f suspensionHertz;
	b3f suspensionDampingRatio;

	b3f maxSteeringTorque;
	b3f steeringHertz;
	b3f steeringDampingRatio;

	/// The three steering angles, in brads. The only b3a fields in the joint --
	/// everything else is a length, a speed, a frequency or a torque.
	b3a lowerSteeringLimit;
	b3a upperSteeringLimit;
	b3a targetSteeringAngle;

	int indexA;
	int indexB;

	/// Joint frames in world space, relative to each centre of mass.
	b3Transform frameA;
	b3Transform frameB;

	b3Vec3 deltaCenter;

	b3Softness suspensionSoftness;
	b3Softness steeringSoftness;

	bool enableSpinMotor;
	bool enableSuspensionSpring;
	bool enableSuspensionLimit;
	bool enableSteering;
	bool enableSteeringLimit;
} b3WheelJoint;

/// The base half of a joint's simulation data, followed by the per-type union.
///
/// Stage 1 deferred the union because b3_filterJoint has no per-type state and
/// an empty union is not valid C. Stage 2 added it with the joint that paid
/// for it, and each stage since has widened it:
///
///   Stage 1  184 bytes   no union
///   Stage 2  300 bytes   + b3DistanceJoint  (116)
///   Stage 3  376 bytes   + b3RevoluteJoint  (192)
///   Stage 4  444 bytes   + b3SphericalJoint (260, and still the widest member)
///   Stage 5  444 bytes   + b3WeldJoint (176) and b3MotorJoint (240) -- **free**
///   Stage 6  444 bytes   + b3PrismaticJoint (228) -- **free**
///   Stage 7  444 bytes   + b3ParallelJoint (96) and b3WheelJoint (196) -- **free**
///
/// Stage 5 was the first stage to add joints and cost nothing, and Stages 6 and
/// 7 did it again: every type since the ball joint has fitted inside the room it
/// already claimed, so the union has not widened and no solver set's reservation
/// has moved. Worth recording precisely because the split exists to attribute
/// the cost -- three consecutive stages adding joints for zero bytes is evidence
/// the union is sized by its widest member and not by the count, which is the
/// assumption Stage 1's capacity work rests on.
///
/// A joint sim migrates between four homes -- the awake set, the static set,
/// the disabled set and the graph colour -- so Stage 1's capacity work reserves
/// every one of those increments four times over. That is why these stages are
/// split: the cost lands attributed to the joint that caused it rather than
/// buried in a phase that added no math.
///
/// The revolute joint is larger than the distance joint mostly because of its
/// two b3Transform frames and its four cached axes. The parallel joint is the
/// smallest member of all, below even the distance joint, because it has no
/// linear constraint and so needs no frames at all.
typedef struct b3JointSim
{
	int jointId;

	int bodyIdA;
	int bodyIdB;

	b3JointType type;

	/// Joint frames local to the body origin.
	b3Transform localFrameA;
	b3Transform localFrameB;

	b3iw invMassA, invMassB;
	b3MatrixW invIA, invIB;

	b3f constraintHertz;

	/// Not b3c, despite being dimensionless: the joint default is 2 and the
	/// world default is 10, both outside b3c's [0,1]. Matches b3JointDef.
	b3f constraintDampingRatio;

	/// Recomputed every prepare from the two above, clamped against inv_h.
	b3Softness constraintSoftness;

	b3f forceThreshold;
	b3f torqueThreshold;

	bool fixedRotation;

	union
	{
		b3DistanceJoint distanceJoint;
		b3RevoluteJoint revoluteJoint;
		b3SphericalJoint sphericalJoint;
		b3WeldJoint weldJoint;
		b3MotorJoint motorJoint;
		b3PrismaticJoint prismaticJoint;
		b3ParallelJoint parallelJoint;
		b3WheelJoint wheelJoint;
	};
} b3JointSim;

// The size ledger above, made checkable.
//
// Device layout only. Under B3_FIXED_DEBUG every scalar is a struct carrying a
// shadow double, so a b3f is 16 bytes rather than 4 and every number here
// changes -- the same mode-dependent layout Makefile.host stamps its object
// directory against. The ledger describes what the ARM9 compiles, which is
// what the solver sets actually reserve, so the checks are scoped to it.
// Strict mode is included: it gives each scale a distinct struct type but no
// shadow, so the layout is unchanged.
//
// It was a hand-maintained comment through six stages, and at Stage 7 it was
// found to be *pre-claiming* a wheel-joint size written before the struct
// existed. These turn each claim into a compile error if it drifts. The union
// is reserved four times over -- the awake, static and disabled sets and the
// graph colour -- so a member quietly becoming the widest is a memory change
// nothing else would report.
#ifndef B3_FIXED_DEBUG
_Static_assert( sizeof( b3SphericalJoint ) == 260, "the ball joint is the widest union member; the ledger says 260" );
_Static_assert( sizeof( b3WheelJoint ) == 196, "wheel joint size changed; update the ledger in this file" );
_Static_assert( sizeof( b3ParallelJoint ) == 96, "parallel joint size changed; update the ledger in this file" );
_Static_assert( sizeof( b3WheelJoint ) <= sizeof( b3SphericalJoint ),
				"the wheel joint no longer fits inside the ball joint -- b3JointSim has grown" );
_Static_assert( sizeof( b3JointSim ) == 444, "b3JointSim changed size; every solver set reservation moves with it" );
#endif

typedef struct b3StepContext b3StepContext;

/// True when the two bodies between them have no rotational freedom at all.
///
/// Upstream tests `b3Det( invIA + invIB ) < 1000 * FLT_MIN`, a threshold in
/// units the port has no equivalent for. b3InvertRotationMass already answers
/// the same question -- it returns the zero matrix when its determinant
/// vanishes, which *is* the condition -- and every joint with a rotational
/// constraint computes that inverse anyway, so the guard costs nothing beyond
/// the test.
///
/// Lived as a `static` in revolute_joint.c and again in spherical_joint.c
/// through Stages 3 and 4. Stage 5 gave it a fourth and fifth caller in the
/// weld and motor joints, which is the point at which one copy per file stops
/// being right.
B3_INLINE bool b3FixedRotationFromMass( b3Matrix3 rotationMass )
{
	return b3Raw( rotationMass.cx.x ) == 0 && b3Raw( rotationMass.cy.y ) == 0 && b3Raw( rotationMass.cz.z ) == 0;
}

/// Half, as a Q30 coefficient. The collinearity axes are built with it.
#define B3_JOINT_HALF b3cFromFrac( 1, 2 )

/// The two axes a collinearity constraint acts along.
///
/// Upstream's expression, and the reason it looks arbitrary: the constraint is
/// on the *vector* part of the relative rotation, and these are the rows of the
/// Jacobian that maps an angular velocity onto its first two components. The
/// half is the quaternion half-angle.
///
/// Every caller computes them twice -- in prepare for warm start, and again in
/// the solve against the sub-step rotation. That duplication is upstream's and
/// is deliberate: the warm-start axes must be the ones the stored impulse was
/// accumulated along.
///
/// The rows are **half-length by construction**, and exactly so:
///
///     |perpAxis| = 0.5 * sqrt( 1 - (v . e)^2 )
///
/// since `s*e` and `v x e` are orthogonal for a unit quaternion. That is 0.5 at
/// identity and falls to zero only at a half turn about the axis in question.
/// At Q12 a half is raw 2048 -- eleven significant bits -- so the axes
/// themselves are well conditioned. What the half costs is the *determinant* of
/// the 2x2 built from them: `k` is quadratic in the row length, so the quarter
/// enters twice, and b3InvertPerpMass returns zero once
/// `i * sqrt(1 - rho^2) < 9.8e-4` -- a pair whose combined inverse inertia
/// about the constrained axes is under about 1e-3, i.e. a rotational inertia
/// above roughly 1000 kg m^2. Zero means "apply no impulse", which is the safe
/// direction, but it does mean a sufficiently heavy pair is not held. Without
/// the half the threshold would sit at 2.4e-4, so the half moves it up 4x.
///
/// Lived as a `static` in revolute_joint.c through Stages 3 to 6. Stage 7 gave
/// it callers in the parallel joint and in the wheel's steering-off
/// collinearity branch, which is the point at which one copy per file stops
/// being right -- the same rule b3FixedRotationFromMass was hoisted under.
/// Stage 8 took the last step for the same reason: `B3_INLINE` left one
/// out-of-line copy per caller anyway once the archive moved to `-Os`, so the
/// definition now lives in b3hot.c and there is exactly one.
void b3CollinearityPerpAxes( b3Quat frameQ, b3Quat relQ, b3Vec3* outX, b3Vec3* outY );

b3Joint* b3GetJointFullId( b3World* world, b3JointId jointId );
b3JointSim* b3GetJointSim( b3World* world, b3Joint* joint );
b3JointSim* b3GetJointSimCheckType( b3World* world, b3JointId jointId, b3JointType type );
void b3DestroyJointInternal( b3World* world, b3Joint* joint, bool wakeBodies );

/// The three per-joint solver stages, dispatched on b3JointSim::type.
///
/// solver.c calls these from its own task functions rather than through
/// upstream's b3*Joints_Overflow wrappers: the port has one graph colour, so
/// the wrappers would each be a loop with nothing to choose between.
void b3PrepareJoint( b3JointSim* joint, b3StepContext* context );
void b3WarmStartJoint( b3JointSim* joint, b3StepContext* context );
void b3SolveJoint( b3JointSim* joint, b3StepContext* context, bool useBias );

/// The axial force a joint is currently applying, in world space.
///
/// Dispatched on type by b3GetJointReaction in joint.c. A filter joint applies
/// none and returns zero; every solved type reports its accumulated impulse
/// divided by the sub-step, which is what b3MulImpFToF spells.
b3Vec3 b3GetJointReaction( b3World* world, b3JointSim* base );

// -------------------------------------------------------------------------
// Per-type solver entry points
// -------------------------------------------------------------------------
//
// One trio per joint type, called only from the dispatchers above. They are
// declared here rather than in a per-type header because the dispatchers are
// the only callers and upstream does the same.

void b3PrepareDistanceJoint( b3JointSim* base, b3StepContext* context );
void b3WarmStartDistanceJoint( b3JointSim* base, b3StepContext* context );
void b3SolveDistanceJoint( b3JointSim* base, b3StepContext* context, bool useBias );
b3Vec3 b3GetDistanceJointForce( b3World* world, b3JointSim* base );

void b3PrepareRevoluteJoint( b3JointSim* base, b3StepContext* context );
void b3WarmStartRevoluteJoint( b3JointSim* base, b3StepContext* context );
void b3SolveRevoluteJoint( b3JointSim* base, b3StepContext* context, bool useBias );
b3Vec3 b3GetRevoluteJointForce( b3World* world, b3JointSim* base );
b3Vec3 b3GetRevoluteJointTorque( b3World* world, b3JointSim* base );

void b3PrepareSphericalJoint( b3JointSim* base, b3StepContext* context );
void b3WarmStartSphericalJoint( b3JointSim* base, b3StepContext* context );
void b3SolveSphericalJoint( b3JointSim* base, b3StepContext* context, bool useBias );
b3Vec3 b3GetSphericalJointForce( b3World* world, b3JointSim* base );
b3Vec3 b3GetSphericalJointTorque( b3World* world, b3JointSim* base );

void b3PrepareWeldJoint( b3JointSim* base, b3StepContext* context );
void b3WarmStartWeldJoint( b3JointSim* base, b3StepContext* context );
void b3SolveWeldJoint( b3JointSim* base, b3StepContext* context, bool useBias );
b3Vec3 b3GetWeldJointForce( b3World* world, b3JointSim* base );
b3Vec3 b3GetWeldJointTorque( b3World* world, b3JointSim* base );

// The motor joint takes `useBias` like every other type and ignores it: it has
// no position constraint to relax, only springs whose bias is their own. The
// parameter is kept so the three dispatchers stay one shape.
void b3PrepareMotorJoint( b3JointSim* base, b3StepContext* context );
void b3WarmStartMotorJoint( b3JointSim* base, b3StepContext* context );
void b3SolveMotorJoint( b3JointSim* base, b3StepContext* context, bool useBias );
b3Vec3 b3GetMotorJointForce( b3World* world, b3JointSim* base );
b3Vec3 b3GetMotorJointTorque( b3World* world, b3JointSim* base );

void b3PreparePrismaticJoint( b3JointSim* base, b3StepContext* context );
void b3WarmStartPrismaticJoint( b3JointSim* base, b3StepContext* context );
void b3SolvePrismaticJoint( b3JointSim* base, b3StepContext* context, bool useBias );
b3Vec3 b3GetPrismaticJointForce( b3World* world, b3JointSim* base );
b3Vec3 b3GetPrismaticJointTorque( b3World* world, b3JointSim* base );

// The parallel joint takes `useBias` and ignores it, for the motor joint's
// reason: the constraint is a spring and has no rigid part to relax. It has no
// b3GetParallelJointForce at all -- it applies no linear impulse, so zero is
// the true answer rather than a placeholder, and b3GetJointReaction returns it
// directly instead of calling into this file.
void b3PrepareParallelJoint( b3JointSim* base, b3StepContext* context );
void b3WarmStartParallelJoint( b3JointSim* base, b3StepContext* context );
void b3SolveParallelJoint( b3JointSim* base, b3StepContext* context, bool useBias );
b3Vec3 b3GetParallelJointTorque( b3World* world, b3JointSim* base );

void b3PrepareWheelJoint( b3JointSim* base, b3StepContext* context );
void b3WarmStartWheelJoint( b3JointSim* base, b3StepContext* context );
void b3SolveWheelJoint( b3JointSim* base, b3StepContext* context, bool useBias );
b3Vec3 b3GetWheelJointForce( b3World* world, b3JointSim* base );
b3Vec3 b3GetWheelJointTorque( b3World* world, b3JointSim* base );

/// The scalar reaction magnitudes, for the joint-event threshold test in
/// b3SolveJointsTask. Derived from the two public vector queries rather than
/// from a third per-type switch -- see the definition.
void b3GetJointReactionScalars( b3World* world, b3JointSim* base, b3f* forceOut, b3f* torqueOut );

/// The wheel case of b3Joint_GetAngularSeparation, which lives in wheel_joint.c
/// rather than in joint.c's switch because it needs the steering frame the file
/// keeps to itself. Upstream has no implementation to port.
b3a b3GetWheelJointAngularSeparation( b3World* world, b3JointSim* base );
