// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Erin Catto        (original Box3D)
// Copyright (c) 2026 Warioware64       (Nitro Engine Advanced fixed-point port)
//
// This file is part of Nitro Engine Advanced

/// @file   types.c
/// @brief  Default world, body, shape, filter, material and joint definitions.
///
/// Upstream reads every default through b3GetLengthUnitsPerMeter(), a runtime
/// float. Here the length unit is frozen at 1 by nea_config.h, so each default
/// is built directly at its own scale by the b3*From* constructors, which fold
/// to a literal in device mode and carry a shadow double under B3_FIXED_DEBUG.
///
/// b3DefaultDebugDraw and its nine empty draw callbacks are gone with the rest
/// of the debug renderer.

#include "box3d/types.h"

#include "core.h"

#include "box3d/constants.h"

b3WorldDef b3DefaultWorldDef( void )
{
	b3WorldDef def = { 0 };

	// Box3D has no up-vector; -Y matches every NEA example and the DS's own
	// screen orientation.
	def.gravity = b3MakeVec3( b3f_zero, b3fFromInt( -10 ), b3f_zero );

	def.hitEventThreshold = b3fFromInt( 1 );
	def.restitutionThreshold = b3fFromInt( 1 );
	def.contactSpeed = b3fFromInt( 3 );
	def.contactHertz = b3fFromInt( 30 );
	def.contactDampingRatio = b3fFromInt( 10 );

	// 400 units per second, faster than the speed of sound. Q19.12 tops out
	// at 524288, so this leaves three orders of magnitude of headroom -- the
	// cap exists to keep the solver sane, not to fit the format.
	def.maximumLinearSpeed = b3fFromInt( 400 );

	// 16,384 units. Far enough out that no level reaches it -- B3_HUGE, where
	// the solver stops being accurate, is 2000 -- and far enough in that the
	// AABB arithmetic is still exact, which it stops being at 262,144.
	def.maximumWorldExtent = b3fFromInt( 16384 );

	def.enableSleep = true;
	def.enableContinuous = true;

	// capacity stays zeroed. b3CreateWorld substitutes its own minimums for
	// any field left at zero, so a caller who only wants defaults gets a
	// small world rather than one that cannot hold a body.

	def.internalValue = B3_SECRET_COOKIE;
	return def;
}

b3BodyDef b3DefaultBodyDef( void )
{
	b3BodyDef def = { 0 };
	def.type = b3_staticBody;
	def.rotation = b3Quat_identity;

	// 0.05 units per second. Exact as a fraction; 0.05f is not exact as a
	// float either, so this is the more faithful conversion of the two.
	def.sleepThreshold = b3fFromFrac( 5, 100 );

	def.gravityScale = b3f_one;
	def.enableSleep = true;
	def.isAwake = true;
	def.isEnabled = true;
	def.enableContactRecycling = true;
	def.internalValue = B3_SECRET_COOKIE;
	return def;
}

b3Filter b3DefaultFilter( void )
{
	b3Filter filter = { B3_DEFAULT_CATEGORY_BITS, B3_DEFAULT_MASK_BITS, 0 };
	return filter;
}

b3QueryFilter b3DefaultQueryFilter( void )
{
	// Two fields, not upstream's four: Phase 2 dropped the query group index
	// and the user context along with the world query API that read them.
	b3QueryFilter filter = { B3_DEFAULT_CATEGORY_BITS, B3_DEFAULT_MASK_BITS };
	return filter;
}

b3SurfaceMaterial b3DefaultSurfaceMaterial( void )
{
	b3SurfaceMaterial surfaceMaterial = { 0 };
	surfaceMaterial.friction = b3cFromFrac( 6, 10 );
	return surfaceMaterial;
}

b3ShapeDef b3DefaultShapeDef( void )
{
	b3ShapeDef def = { 0 };
	def.baseMaterial = b3DefaultSurfaceMaterial();

	// Density of water, in kg per cubic unit. A 1-unit cube therefore masses
	// 1000, and Q19.12 holds masses up to 524287 -- so a solid body larger
	// than about 8 units on a side at this density is out of range. Author
	// large level geometry as static (zero mass) or lower its density.
	def.density = b3fFromInt( 1000 );

	def.filter = b3DefaultFilter();
	def.updateBodyMass = true;
	def.invokeContactCreation = true;
	def.enableSpeculativeContact = true;
	def.internalValue = B3_SECRET_COOKIE;
	return def;
}

/// The port's stand-in for upstream's FLT_MAX, wherever FLT_MAX means "no
/// bound" rather than a real number.
///
/// Not B3_F_MAX, despite it being the obvious translation: b3IsValidFloat
/// excludes its own endpoints (`> -(INT32_MAX/2) && < INT32_MAX/2`), on
/// purpose, because that value is what a saturating op returns and the
/// validator exists to catch exactly that. So B3_F_MAX would fail the
/// b3IsValidFloat assert in b3CreateJoint -- it did, on the first run of
/// test_joint_plumbing. One quantum below it is the largest value that is both
/// unreachable in practice and legal to store.
///
/// B3_NO_BOUND itself now lives in types.h -- see the commentary there for why
/// it moved.

/// The base every b3*JointDef wraps. Static because upstream's is: a caller
/// initializes the type-specific def, never this one.
static b3JointDef b3DefaultJointDef( void )
{
	b3JointDef def = { 0 };
	def.localFrameA = b3Transform_identity;
	def.localFrameB = b3Transform_identity;

	// Upstream uses FLT_MAX to mean "never report a joint event".
	def.forceThreshold = B3_NO_BOUND;
	def.torqueThreshold = B3_NO_BOUND;

	def.constraintHertz = b3fFromInt( 60 );
	def.constraintDampingRatio = b3fFromInt( 2 );

	// Upstream's drawScale is absent: no debug renderer, as in b3DefaultWorldDef.

	def.internalValue = B3_SECRET_COOKIE;
	return def;
}

b3FilterJointDef b3DefaultFilterJointDef( void )
{
	b3FilterJointDef def = { 0 };
	def.base = b3DefaultJointDef();
	return def;
}

b3DistanceJointDef b3DefaultDistanceJointDef( void )
{
	b3DistanceJointDef def = { 0 };
	def.base = b3DefaultJointDef();
	def.length = b3fFromInt( 1 );

	// Upstream's default range is 0 to B3_HUGE, and its B3_HUGE is 100000 *
	// b3_lengthUnitsPerMeter. The port's is 2000 (constants.h:76), chosen when
	// B3_LINEAR_SLOP stops being small relative to a body -- a finite bound
	// rather than a stand-in for infinity, so it is the right one here too.
	def.maxLength = B3_HUGE;

	// Upstream's -FLT_MAX / +FLT_MAX: the spring sustains any tension and any
	// compression until a caller says otherwise.
	def.lowerSpringForce = b3NegF( B3_NO_BOUND );
	def.upperSpringForce = B3_NO_BOUND;

	return def;
}

b3RevoluteJointDef b3DefaultRevoluteJointDef( void )
{
	b3RevoluteJointDef def = { 0 };
	def.base = b3DefaultJointDef();

	// Upstream's default range is +-0.99*pi radians, which it documents as the
	// widest a hinge limit may be: the twist angle is recovered through an
	// atan2 and is only single valued on the open half turn, so a limit at
	// exactly +-pi could be reported on either side of the wrap. 0.99 of a
	// half turn is 16220 brads.
	def.lowerAngle = (b3a)-16220;
	def.upperAngle = (b3a)16220;

	return def;
}

b3SphericalJointDef b3DefaultSphericalJointDef( void )
{
	b3SphericalJointDef def = { 0 };
	def.base = b3DefaultJointDef();

	// Upstream sets only this, and it is the one field a zeroed def gets
	// actively wrong rather than merely unhelpfully: a zero quaternion is not a
	// rotation, and the spring would drive toward it.
	def.targetRotation = b3Quat_identity;

	// Upstream leaves the twist limits at zero and relies on the caller setting
	// them before enabling the limit. The port defaults them to the same
	// +-0.99 half turn b3DefaultRevoluteJointDef uses, for the same reason and
	// with the same value: a twist recovered through an atan2 is single valued
	// only on the open half turn. A caller who enables the twist limit without
	// setting a range then gets a limit that does nothing, rather than one
	// clamped shut at zero.
	def.lowerTwistAngle = (b3a)-16220;
	def.upperTwistAngle = (b3a)16220;

	// Likewise the cone: zero would be a cone of zero half-angle, which pins
	// the axes together the moment the limit is enabled. Half a turn is the
	// widest valid value and means "no effective limit".
	def.coneAngle = B3_BRAD_PI;

	return def;
}

b3WeldJointDef b3DefaultWeldJointDef( void )
{
	b3WeldJointDef def = { 0 };
	def.base = b3DefaultJointDef();

	// Every remaining field is deliberately zero, and unlike the spherical's
	// targetRotation none of them is a value a zeroed def gets *wrong*: zero
	// hertz means a rigid weld, which is what a caller asking for a weld
	// almost always wants. The damping ratios are unused while the hertz are
	// zero, so leaving them there is not a trap either.
	return def;
}

b3MotorJointDef b3DefaultMotorJointDef( void )
{
	b3MotorJointDef def = { 0 };
	def.base = b3DefaultJointDef();

	// All four bounds stay zero, which disables all four drives -- so a default
	// motor joint does nothing until the caller gives a branch a budget.
	//
	// That is upstream's default and it is the right one here for a reason
	// upstream does not have: in fixed point a bound is not just a safety
	// limit, it is what keeps each accumulator inside the impulse scale. A
	// non-zero default would be an arbitrary force applied to a joint the
	// caller has not configured yet.
	return def;
}

b3PrismaticJointDef b3DefaultPrismaticJointDef( void )
{
	b3PrismaticJointDef def = { 0 };
	def.base = b3DefaultJointDef();

	// Every branch off and both limits at zero, so a default prismatic joint is
	// a pure point-to-line constraint: it slides freely along frame A's x and is
	// locked in every other degree.
	//
	// The limits deliberately do **not** default to +/-B3_HUGE the way the
	// distance joint's maxLength does. See b3PrismaticJointDef::lowerTranslation
	// -- a 4000-unit range puts the speculative band at 1000 m and the bias it
	// asks for past what Q12 holds. Zero is inert until the caller sets a range,
	// which is the safe default rather than merely upstream's.
	return def;
}

b3ParallelJointDef b3DefaultParallelJointDef( void )
{
	b3ParallelJointDef def = { 0 };
	def.base = b3DefaultJointDef();

	// Upstream's hertz is 1 and dampingRatio 1, and both are kept: a spring
	// with a zero frequency is not a spring, so unlike the enable flags on
	// every other joint there is no "off" state a zeroed def could mean.
	def.hertz = b3fFromFrac( 1, 1 );
	def.dampingRatio = b3fFromFrac( 1, 1 );

	// maxTorque stays at zero, where upstream has FLT_MAX. That is the one
	// substantive difference from upstream's default and it makes a default
	// parallel joint **inert**, which b3ParallelJointDef::maxTorque says
	// plainly. FLT_MAX has no fixed-point counterpart, and picking a large
	// finite stand-in would hand a caller who never configured the joint an
	// impulse budget nobody chose -- the same argument the prismatic's limits
	// settled one stage ago, reaching the same answer.
	return def;
}

b3WheelJointDef b3DefaultWheelJointDef( void )
{
	b3WheelJointDef def = { 0 };
	def.base = b3DefaultJointDef();

	// Upstream's four non-zero fields, kept exactly. The suspension spring is
	// **on** by default because a wheel joint with a dead suspension is a
	// prismatic with extra steps -- this is the one enable flag in the joint set
	// whose useful default is true.
	def.enableSuspensionSpring = true;
	def.suspensionHertz = b3fFromFrac( 1, 1 );
	def.suspensionDampingRatio = b3fFromFrac( 7, 10 );
	def.steeringHertz = b3fFromFrac( 1, 1 );
	def.steeringDampingRatio = b3fFromFrac( 7, 10 );

	// Everything else stays at zero, and each zero is the right answer rather
	// than an omission:
	//
	//   - both suspension limits, for b3PrismaticJointDef::lowerTranslation's
	//     reason -- a +/-B3_HUGE range would put the speculative band at 1000 m
	//     and its bias past what Q12 holds. Inert until the caller sets a range.
	//   - both steering limits and targetSteeringAngle: zero brads is straight
	//     ahead, which is the only sensible default steering target.
	//   - maxSpinTorque and maxSteeringTorque, which upstream also leaves at
	//     zero -- so a default wheel joint has a suspension and a rigid
	//     steering lock, and neither motor does anything until given a budget.
	return def;
}
