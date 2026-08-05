// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Erin Catto        (original Box3D)
// Copyright (c) 2026 Warioware64       (Nitro Engine Advanced fixed-point port)
//
// This file is part of Nitro Engine Advanced

/// @file   joint.c
/// @brief  Joint creation, destruction, accessors, and the type dispatchers.
///
/// @section history How this file grew
///
/// Phase 3A wrote b3GetJointSim and b3DestroyJointInternal here and nothing
/// else, because b3DestroyBody walks the joint list -- a list that was always
/// empty, so the file was compiled and linked but never executed. Both were
/// transliterated in full rather than stubbed: a stub that asserted would have
/// been a landmine the moment a joint was created, and neither is joint-type
/// specific.
///
/// Phase 6 Stage 1 added the rest of the generic layer -- b3CreateJoint,
/// b3DestroyJoint, the b3Joint_* accessors and the three dispatchers -- plus
/// b3_filterJoint, whose entire behaviour is "these two bodies do not collide"
/// and which therefore has no case in any dispatcher. That is the point of it:
/// it turns the plumbing on with no constraint math anywhere near it, so a
/// failure in the first joint-carrying scene is unambiguously a plumbing bug.
///
/// Stage 2 added b3CreateDistanceJoint, the first entry point whose joint has
/// constraint math (in distance_joint.c), the per-type cases in the three
/// dispatchers, and the fourth switch -- b3GetJointReaction, which finally has
/// an impulse to report.
///
/// Stage 3 added b3CreateRevoluteJoint (revolute_joint.c), its cases in the
/// dispatchers, and the fifth switch -- b3GetJointConstraintTorque, which the
/// first two stages both deferred because nothing yet constrained a rotation.
///
/// @section stage7 What Phase 6 Stage 7 added
///
/// The last two creators -- b3CreateParallelJoint and b3CreateWheelJoint -- so
/// **all nine joint types now exist**. That completeness is the trigger Stage 1
/// named for the two things this file deferred rather than grew a case at a
/// time: b3Joint_GetLinearSeparation and b3Joint_GetAngularSeparation, which
/// upstream writes as one switch over every type each, and which arrive here as
/// the sixth and seventh switches.
///
/// @section absent What is still not here
///
/// b3Joint_SetSpringRotationTarget. b3DrawJoint is gone for good along with the
/// rest of the debug renderer.

#include "joint.h"

#include "body.h"
#include "broad_phase.h"
#include "constraint_graph.h"
#include "contact.h"
#include "core.h"
#include "id_pool.h"
#include "island.h"
#include "physics_world.h"
#include "shape.h"
#include "solver.h"
#include "solver_set.h"

b3Joint* b3GetJointFullId( b3World* world, b3JointId jointId )
{
	int id = jointId.index1 - 1;
	b3Joint* joint = b3Array_Get( world->joints, id );
	B3_ASSERT( joint->jointId == id && joint->generation == jointId.generation );
	return joint;
}

b3JointSim* b3GetJointSim( b3World* world, b3Joint* joint )
{
	if ( joint->setIndex == b3_awakeSet )
	{
		B3_ASSERT( 0 <= joint->colorIndex && joint->colorIndex < B3_GRAPH_COLOR_COUNT );
		b3GraphColor* color = world->constraintGraph.colors + joint->colorIndex;
		return b3Array_Get( color->jointSims, joint->localIndex );
	}

	b3SolverSet* set = b3Array_Get( world->solverSets, joint->setIndex );
	return b3Array_Get( set->jointSims, joint->localIndex );
}

/// Upstream takes only the id and looks the world up itself. The port passes
/// the world in, because every caller already has it and b3GetWorld is not
/// free at one world -- it still range-checks and generation-checks.
b3JointSim* b3GetJointSimCheckType( b3World* world, b3JointId jointId, b3JointType type )
{
	B3_UNUSED( type );
	b3Joint* joint = b3GetJointFullId( world, jointId );
	B3_ASSERT( joint->type == type );
	b3JointSim* jointSim = b3GetJointSim( world, joint );
	B3_ASSERT( jointSim->type == type );
	return jointSim;
}

typedef struct b3JointPair
{
	b3Joint* joint;
	b3JointSim* jointSim;
} b3JointPair;

/// Allocate a joint, thread it into both bodies' joint lists, and place its sim
/// in whichever solver set the two bodies' states imply.
///
/// The four placement branches are upstream's and are load-bearing: a joint
/// touching a disabled body goes to the disabled set, one touching no dynamic
/// body goes to the static set, one touching an awake body goes into the
/// constraint graph, and one bridging two sleeping sets merges them.
static b3JointPair b3CreateJoint( b3World* world, const b3JointDef* def, b3JointType type )
{
	B3_ASSERT( b3IsValidTransform( def->localFrameA ) );
	B3_ASSERT( b3IsValidTransform( def->localFrameB ) );

	b3Body* bodyA = b3GetBodyFullId( world, def->bodyIdA );
	b3Body* bodyB = b3GetBodyFullId( world, def->bodyIdB );

	int bodyIdA = bodyA->id;
	int bodyIdB = bodyB->id;
	int maxSetIndex = b3MaxInt( bodyA->setIndex, bodyB->setIndex );

	// Create joint id and joint
	int jointId = b3AllocId( &world->jointIdPool );
	if ( jointId == world->joints.count )
	{
		b3Array_Push( world->joints, ( b3Joint ){ 0 } );
	}

	b3Joint* joint = b3Array_Get( world->joints, jointId );
	joint->jointId = jointId;
	joint->userData = def->userData;
	joint->generation += 1;
	joint->setIndex = B3_NULL_INDEX;
	joint->colorIndex = B3_NULL_INDEX;
	joint->localIndex = B3_NULL_INDEX;
	joint->islandId = B3_NULL_INDEX;
	joint->islandIndex = B3_NULL_INDEX;
	joint->type = type;
	joint->collideConnected = def->collideConnected;

	// Doubly linked list on bodyA
	joint->edges[0].bodyId = bodyIdA;
	joint->edges[0].prevKey = B3_NULL_INDEX;
	joint->edges[0].nextKey = bodyA->headJointKey;

	int keyA = ( jointId << 1 ) | 0;
	if ( bodyA->headJointKey != B3_NULL_INDEX )
	{
		b3Joint* jointA = b3Array_Get( world->joints, bodyA->headJointKey >> 1 );
		b3JointEdge* edgeA = jointA->edges + ( bodyA->headJointKey & 1 );
		edgeA->prevKey = keyA;
	}
	bodyA->headJointKey = keyA;
	bodyA->jointCount += 1;

	// Doubly linked list on bodyB
	joint->edges[1].bodyId = bodyIdB;
	joint->edges[1].prevKey = B3_NULL_INDEX;
	joint->edges[1].nextKey = bodyB->headJointKey;

	int keyB = ( jointId << 1 ) | 1;
	if ( bodyB->headJointKey != B3_NULL_INDEX )
	{
		b3Joint* jointB = b3Array_Get( world->joints, bodyB->headJointKey >> 1 );
		b3JointEdge* edgeB = jointB->edges + ( bodyB->headJointKey & 1 );
		edgeB->prevKey = keyB;
	}
	bodyB->headJointKey = keyB;
	bodyB->jointCount += 1;

	b3JointSim* jointSim;

	if ( bodyA->setIndex == b3_disabledSet || bodyB->setIndex == b3_disabledSet )
	{
		// if either body is disabled, create in disabled set
		b3SolverSet* set = b3Array_Get( world->solverSets, b3_disabledSet );
		joint->setIndex = b3_disabledSet;
		joint->localIndex = set->jointSims.count;

		jointSim = b3Array_Emplace( set->jointSims );
		memset( jointSim, 0, sizeof( b3JointSim ) );

		jointSim->jointId = jointId;
		jointSim->bodyIdA = bodyIdA;
		jointSim->bodyIdB = bodyIdB;
	}
	else if ( bodyA->type != b3_dynamicBody && bodyB->type != b3_dynamicBody )
	{
		// joint is not attached to a dynamic body
		b3SolverSet* set = b3Array_Get( world->solverSets, b3_staticSet );
		joint->setIndex = b3_staticSet;
		joint->localIndex = set->jointSims.count;

		jointSim = b3Array_Emplace( set->jointSims );
		memset( jointSim, 0, sizeof( b3JointSim ) );

		jointSim->jointId = jointId;
		jointSim->bodyIdA = bodyIdA;
		jointSim->bodyIdB = bodyIdB;
	}
	else if ( bodyA->setIndex == b3_awakeSet || bodyB->setIndex == b3_awakeSet )
	{
		// if either body is sleeping, wake it
		if ( maxSetIndex >= b3_firstSleepingSet )
		{
			b3WakeSolverSet( world, maxSetIndex );
		}

		joint->setIndex = b3_awakeSet;

		jointSim = b3CreateJointInGraph( world, joint );
		jointSim->jointId = jointId;
		jointSim->bodyIdA = bodyIdA;
		jointSim->bodyIdB = bodyIdB;
	}
	else
	{
		// joint connected between sleeping and/or static bodies
		B3_ASSERT( bodyA->setIndex >= b3_firstSleepingSet || bodyB->setIndex >= b3_firstSleepingSet );
		B3_ASSERT( bodyA->setIndex != b3_staticSet || bodyB->setIndex != b3_staticSet );

		// joint should go into the sleeping set (not static set)
		int setIndex = maxSetIndex;

		b3SolverSet* set = b3Array_Get( world->solverSets, setIndex );
		joint->setIndex = setIndex;
		joint->localIndex = set->jointSims.count;

		jointSim = b3Array_Emplace( set->jointSims );
		memset( jointSim, 0, sizeof( b3JointSim ) );

		jointSim->jointId = jointId;
		jointSim->bodyIdA = bodyIdA;
		jointSim->bodyIdB = bodyIdB;

		if ( bodyA->setIndex != bodyB->setIndex && bodyA->setIndex >= b3_firstSleepingSet &&
			 bodyB->setIndex >= b3_firstSleepingSet )
		{
			// merge sleeping sets
			b3MergeSolverSets( world, bodyA->setIndex, bodyB->setIndex );
			B3_ASSERT( bodyA->setIndex == bodyB->setIndex );

			// fix potentially invalid set index
			setIndex = bodyA->setIndex;

			b3SolverSet* mergedSet = b3Array_Get( world->solverSets, setIndex );

			// Careful! The joint sim pointer was orphaned by the set merge.
			//
			// Still true in the port. b3DestroySolverSet was changed to clear
			// its arrays rather than free them, to stop a sleep/wake cycle
			// allocating inside b3World_Step -- but a *merge* still appends one
			// set's sims to the other's, which reallocates, so this re-fetch is
			// load-bearing exactly as it is upstream.
			jointSim = b3Array_Get( mergedSet->jointSims, joint->localIndex );
		}

		B3_ASSERT( joint->setIndex == setIndex );
	}

	jointSim->localFrameA = def->localFrameA;
	jointSim->localFrameB = def->localFrameB;
	jointSim->type = type;
	jointSim->constraintHertz = def->constraintHertz;
	jointSim->constraintDampingRatio = def->constraintDampingRatio;

	// Not zeroed: massScale of zero would make the first prepare a no-op.
	jointSim->constraintSoftness = ( b3Softness ){
		.biasRate = b3f_zero,
		.massScale = b3c_one,
		.impulseScale = b3c_zero,
	};

	B3_ASSERT( b3IsValidFloat( def->forceThreshold ) && b3Raw( def->forceThreshold ) >= 0 );
	B3_ASSERT( b3IsValidFloat( def->torqueThreshold ) && b3Raw( def->torqueThreshold ) >= 0 );

	jointSim->forceThreshold = def->forceThreshold;
	jointSim->torqueThreshold = def->torqueThreshold;

	B3_ASSERT( jointSim->jointId == jointId );
	B3_ASSERT( jointSim->bodyIdA == bodyIdA );
	B3_ASSERT( jointSim->bodyIdB == bodyIdB );

	if ( joint->setIndex > b3_disabledSet )
	{
		// Add edge to island graph
		b3LinkJoint( world, joint );
	}

	b3ValidateSolverSets( world );

	return ( b3JointPair ){ joint, jointSim };
}

/// Destroy every contact between two bodies that a joint has just decided
/// should not collide.
static void b3DestroyContactsBetweenBodies( b3World* world, b3Body* bodyA, b3Body* bodyB )
{
	int contactKey;
	int otherBodyId;

	// The bodies must be awake for the contacts to be in the awake set, and the
	// shorter list is the cheaper one to walk.
	if ( bodyA->contactCount < bodyB->contactCount )
	{
		contactKey = bodyA->headContactKey;
		otherBodyId = bodyB->id;
	}
	else
	{
		contactKey = bodyB->headContactKey;
		otherBodyId = bodyA->id;
	}

	// No need to wake bodies when a joint removes a contact.
	bool wakeBodies = false;

	while ( contactKey != B3_NULL_INDEX )
	{
		int contactId = contactKey >> 1;
		int edgeIndex = contactKey & 1;

		b3Contact* contact = b3Array_Get( world->contacts, contactId );
		contactKey = contact->edges[edgeIndex].nextKey;

		int otherEdgeIndex = edgeIndex ^ 1;
		if ( contact->edges[otherEdgeIndex].bodyId == otherBodyId )
		{
			// Careful, this removes the contact from the current doubly linked
			// list of contacts, which is why contactKey was read first.
			b3DestroyContact( world, contact, wakeBodies );
		}
	}

	b3ValidateSolverSets( world );
}

b3JointId b3CreateFilterJoint( b3WorldId worldId, const b3FilterJointDef* def )
{
	B3_CHECK_JOINT_DEF( def );
	b3World* world = b3GetUnlockedWorldFromId( worldId );
	if ( world == NULL )
	{
		return ( b3JointId ){ 0 };
	}

	b3JointPair pair = b3CreateJoint( world, &def->base, b3_filterJoint );

	b3JointSim* joint = pair.jointSim;

	b3JointId jointId = { joint->jointId + 1, world->worldId, pair.joint->generation };
	return jointId;
}

b3JointId b3CreateDistanceJoint( b3WorldId worldId, const b3DistanceJointDef* def )
{
	B3_CHECK_JOINT_DEF( def );
	B3_ASSERT( b3IsValidFloat( def->length ) && b3Raw( def->length ) > 0 );

	b3World* world = b3GetUnlockedWorldFromId( worldId );
	if ( world == NULL )
	{
		return ( b3JointId ){ 0 };
	}

	b3JointPair pair = b3CreateJoint( world, &def->base, b3_distanceJoint );

	b3JointSim* joint = pair.jointSim;

	// Every placement branch of b3CreateJoint memsets the whole b3JointSim,
	// union included, so the fields not set here -- the four accumulators, the
	// anchors, deltaCenter, axialMass and distanceSoftness -- start at zero.
	// The first b3PrepareDistanceJoint fills the prepared ones, and it runs
	// before the first solve.
	b3DistanceJoint* distance = &joint->distanceJoint;
	distance->length = b3MaxF( def->length, B3_LINEAR_SLOP );
	distance->hertz = def->hertz;
	distance->dampingRatio = def->dampingRatio;
	distance->lowerSpringForce = def->lowerSpringForce;
	distance->upperSpringForce = def->upperSpringForce;
	distance->minLength = b3ClampF( def->minLength, B3_LINEAR_SLOP, B3_HUGE );
	distance->maxLength = b3ClampF( def->maxLength, B3_LINEAR_SLOP, B3_HUGE );
	distance->maxMotorForce = def->maxMotorForce;
	distance->motorSpeed = def->motorSpeed;
	distance->enableSpring = def->enableSpring;
	distance->enableLimit = def->enableLimit;
	distance->enableMotor = def->enableMotor;

	// The four accumulators and the prepared state (indexA/indexB, anchors,
	// deltaCenter, axialMass, distanceSoftness) stay zero: the first
	// b3PrepareDistanceJoint fills them, and it runs before the first solve.
	distance->indexA = B3_NULL_INDEX;
	distance->indexB = B3_NULL_INDEX;

	b3JointId jointId = { joint->jointId + 1, world->worldId, pair.joint->generation };
	return jointId;
}

b3JointId b3CreateRevoluteJoint( b3WorldId worldId, const b3RevoluteJointDef* def )
{
	B3_CHECK_JOINT_DEF( def );

	b3World* world = b3GetUnlockedWorldFromId( worldId );
	if ( world == NULL )
	{
		return ( b3JointId ){ 0 };
	}

	b3JointPair pair = b3CreateJoint( world, &def->base, b3_revoluteJoint );

	b3JointSim* joint = pair.jointSim;

	// The whole sim was memset by b3CreateJoint, union included, so only the
	// fields the definition carries are written here. Everything else -- the
	// six accumulators, the frames, the three cached axes, deltaCenter,
	// axialMass and springSoftness -- is filled by the first
	// b3PrepareRevoluteJoint, which runs before the first solve.
	b3RevoluteJoint* revolute = &joint->revoluteJoint;
	revolute->targetAngle = def->targetAngle;
	revolute->hertz = def->hertz;
	revolute->dampingRatio = def->dampingRatio;
	revolute->maxMotorTorque = def->maxMotorTorque;
	revolute->motorSpeed = def->motorSpeed;
	revolute->enableSpring = def->enableSpring;
	revolute->enableLimit = def->enableLimit;
	revolute->enableMotor = def->enableMotor;

	revolute->lowerAngle = def->lowerAngle < def->upperAngle ? def->lowerAngle : def->upperAngle;
	revolute->upperAngle = def->lowerAngle < def->upperAngle ? def->upperAngle : def->lowerAngle;

	revolute->indexA = B3_NULL_INDEX;
	revolute->indexB = B3_NULL_INDEX;

	b3JointId jointId = { joint->jointId + 1, world->worldId, pair.joint->generation };
	return jointId;
}

b3JointId b3CreateSphericalJoint( b3WorldId worldId, const b3SphericalJointDef* def )
{
	B3_CHECK_JOINT_DEF( def );
	B3_ASSERT( b3IsValidQuat( def->targetRotation ) );

	b3World* world = b3GetUnlockedWorldFromId( worldId );
	if ( world == NULL )
	{
		return ( b3JointId ){ 0 };
	}

	b3JointPair pair = b3CreateJoint( world, &def->base, b3_sphericalJoint );

	b3JointSim* joint = pair.jointSim;

	// As for the revolute: the whole sim was memset by b3CreateJoint, union
	// included, so only what the definition carries is written here. The six
	// accumulators, the frames, the two cached axes, twistScale, deltaCenter,
	// the three masses and springSoftness are filled by the first
	// b3PrepareSphericalJoint, which runs before the first solve.
	b3SphericalJoint* spherical = &joint->sphericalJoint;
	spherical->hertz = def->hertz;
	spherical->dampingRatio = def->dampingRatio;
	spherical->targetRotation = def->targetRotation;
	spherical->maxMotorTorque = def->maxMotorTorque;
	spherical->motorVelocity = def->motorVelocity;
	spherical->enableSpring = def->enableSpring;
	spherical->enableConeLimit = def->enableConeLimit;
	spherical->enableTwistLimit = def->enableTwistLimit;
	spherical->enableMotor = def->enableMotor;

	// The cone is one-sided and unsigned, so it is clamped rather than ordered.
	spherical->coneAngle = def->coneAngle < 0 ? 0 : ( def->coneAngle > B3_BRAD_PI ? B3_BRAD_PI : def->coneAngle );

	spherical->lowerTwistAngle = def->lowerTwistAngle < def->upperTwistAngle ? def->lowerTwistAngle : def->upperTwistAngle;
	spherical->upperTwistAngle = def->lowerTwistAngle < def->upperTwistAngle ? def->upperTwistAngle : def->lowerTwistAngle;

	spherical->indexA = B3_NULL_INDEX;
	spherical->indexB = B3_NULL_INDEX;

	b3JointId jointId = { joint->jointId + 1, world->worldId, pair.joint->generation };
	return jointId;
}

b3JointId b3CreateWeldJoint( b3WorldId worldId, const b3WeldJointDef* def )
{
	B3_CHECK_JOINT_DEF( def );
	B3_ASSERT( b3IsValidFloat( def->linearHertz ) && b3Raw( def->linearHertz ) >= 0 );
	B3_ASSERT( b3IsValidFloat( def->angularHertz ) && b3Raw( def->angularHertz ) >= 0 );

	b3World* world = b3GetUnlockedWorldFromId( worldId );
	if ( world == NULL )
	{
		return ( b3JointId ){ 0 };
	}

	b3JointPair pair = b3CreateJoint( world, &def->base, b3_weldJoint );

	b3JointSim* joint = pair.jointSim;

	// The whole sim was memset by b3CreateJoint, union included, so only what
	// the definition carries is written here. The two accumulators, the frames,
	// deltaCenter, angularMass and the two softnesses are filled by the first
	// b3PrepareWeldJoint, which runs before the first solve.
	b3WeldJoint* weld = &joint->weldJoint;
	weld->linearHertz = def->linearHertz;
	weld->linearDampingRatio = def->linearDampingRatio;
	weld->angularHertz = def->angularHertz;
	weld->angularDampingRatio = def->angularDampingRatio;

	weld->indexA = B3_NULL_INDEX;
	weld->indexB = B3_NULL_INDEX;

	b3JointId jointId = { joint->jointId + 1, world->worldId, pair.joint->generation };
	return jointId;
}

b3JointId b3CreateMotorJoint( b3WorldId worldId, const b3MotorJointDef* def )
{
	B3_CHECK_JOINT_DEF( def );
	B3_ASSERT( b3IsValidVec3( def->linearVelocity ) && b3IsValidVec3( def->angularVelocity ) );
	B3_ASSERT( b3IsValidFloat( def->linearHertz ) && b3Raw( def->linearHertz ) >= 0 );
	B3_ASSERT( b3IsValidFloat( def->angularHertz ) && b3Raw( def->angularHertz ) >= 0 );

	b3World* world = b3GetUnlockedWorldFromId( worldId );
	if ( world == NULL )
	{
		return ( b3JointId ){ 0 };
	}

	b3JointPair pair = b3CreateJoint( world, &def->base, b3_motorJoint );

	b3JointSim* joint = pair.jointSim;

	b3MotorJoint* motor = &joint->motorJoint;
	motor->linearVelocity = def->linearVelocity;
	motor->angularVelocity = def->angularVelocity;
	motor->linearHertz = def->linearHertz;
	motor->linearDampingRatio = def->linearDampingRatio;
	motor->angularHertz = def->angularHertz;
	motor->angularDampingRatio = def->angularDampingRatio;

	// Clamped rather than asserted, matching the accessors: a negative bound is
	// a caller mistake with an obvious right answer -- no budget -- and the
	// four branches read `> 0` to decide whether to run at all, so a negative
	// left unclamped would enable a drive with a negative allowance.
	motor->maxVelocityForce = b3MaxF( b3f_zero, def->maxVelocityForce );
	motor->maxVelocityTorque = b3MaxF( b3f_zero, def->maxVelocityTorque );
	motor->maxSpringForce = b3MaxF( b3f_zero, def->maxSpringForce );
	motor->maxSpringTorque = b3MaxF( b3f_zero, def->maxSpringTorque );

	motor->indexA = B3_NULL_INDEX;
	motor->indexB = B3_NULL_INDEX;

	b3JointId jointId = { joint->jointId + 1, world->worldId, pair.joint->generation };
	return jointId;
}

b3JointId b3CreatePrismaticJoint( b3WorldId worldId, const b3PrismaticJointDef* def )
{
	B3_CHECK_JOINT_DEF( def );
	B3_ASSERT( b3IsValidFloat( def->hertz ) && b3Raw( def->hertz ) >= 0 );
	B3_ASSERT( b3IsValidFloat( def->dampingRatio ) && b3Raw( def->dampingRatio ) >= 0 );
	B3_ASSERT( b3IsValidFloat( def->targetTranslation ) );
	B3_ASSERT( b3IsValidFloat( def->lowerTranslation ) && b3IsValidFloat( def->upperTranslation ) );

	b3World* world = b3GetUnlockedWorldFromId( worldId );
	if ( world == NULL )
	{
		return ( b3JointId ){ 0 };
	}

	b3JointPair pair = b3CreateJoint( world, &def->base, b3_prismaticJoint );

	b3JointSim* joint = pair.jointSim;

	b3PrismaticJoint* prismatic = &joint->prismaticJoint;
	prismatic->targetTranslation = def->targetTranslation;
	prismatic->hertz = def->hertz;
	prismatic->dampingRatio = def->dampingRatio;
	prismatic->motorSpeed = def->motorSpeed;

	// Sorted at creation as the revolute's angles are, so the range is the one
	// the caller meant regardless of the order they gave it in.
	prismatic->lowerTranslation =
		b3Raw( def->lowerTranslation ) < b3Raw( def->upperTranslation ) ? def->lowerTranslation : def->upperTranslation;
	prismatic->upperTranslation =
		b3Raw( def->lowerTranslation ) < b3Raw( def->upperTranslation ) ? def->upperTranslation : def->lowerTranslation;

	// Clamped rather than asserted, matching the motor joint's bounds and the
	// accessor: a negative budget has one obvious right answer, which is none.
	prismatic->maxMotorForce = b3MaxF( b3f_zero, def->maxMotorForce );

	prismatic->enableSpring = def->enableSpring;
	prismatic->enableLimit = def->enableLimit;
	prismatic->enableMotor = def->enableMotor;

	prismatic->indexA = B3_NULL_INDEX;
	prismatic->indexB = B3_NULL_INDEX;

	b3JointId jointId = { joint->jointId + 1, world->worldId, pair.joint->generation };
	return jointId;
}

b3JointId b3CreateParallelJoint( b3WorldId worldId, const b3ParallelJointDef* def )
{
	B3_CHECK_JOINT_DEF( def );
	B3_ASSERT( b3IsValidFloat( def->hertz ) && b3Raw( def->hertz ) >= 0 );
	B3_ASSERT( b3IsValidFloat( def->dampingRatio ) && b3Raw( def->dampingRatio ) >= 0 );
	B3_ASSERT( b3IsValidFloat( def->maxTorque ) );

	b3World* world = b3GetUnlockedWorldFromId( worldId );
	if ( world == NULL )
	{
		return ( b3JointId ){ 0 };
	}

	b3JointPair pair = b3CreateJoint( world, &def->base, b3_parallelJoint );

	b3JointSim* joint = pair.jointSim;

	b3ParallelJoint* parallel = &joint->parallelJoint;
	parallel->hertz = def->hertz;
	parallel->dampingRatio = def->dampingRatio;

	// Clamped rather than asserted, matching every other joint's budget. Note
	// that zero is not merely the floor here -- it is the default, and it makes
	// the joint inert. See b3ParallelJointDef::maxTorque.
	parallel->maxTorque = b3MaxF( b3f_zero, def->maxTorque );

	parallel->indexA = B3_NULL_INDEX;
	parallel->indexB = B3_NULL_INDEX;

	b3JointId jointId = { joint->jointId + 1, world->worldId, pair.joint->generation };
	return jointId;
}

b3JointId b3CreateWheelJoint( b3WorldId worldId, const b3WheelJointDef* def )
{
	B3_CHECK_JOINT_DEF( def );
	B3_ASSERT( b3IsValidFloat( def->suspensionHertz ) && b3Raw( def->suspensionHertz ) >= 0 );
	B3_ASSERT( b3IsValidFloat( def->suspensionDampingRatio ) && b3Raw( def->suspensionDampingRatio ) >= 0 );
	B3_ASSERT( b3IsValidFloat( def->steeringHertz ) && b3Raw( def->steeringHertz ) >= 0 );
	B3_ASSERT( b3IsValidFloat( def->steeringDampingRatio ) && b3Raw( def->steeringDampingRatio ) >= 0 );
	B3_ASSERT( b3IsValidFloat( def->lowerSuspensionLimit ) && b3IsValidFloat( def->upperSuspensionLimit ) );
	B3_ASSERT( b3IsValidFloat( def->spinSpeed ) );

	b3World* world = b3GetUnlockedWorldFromId( worldId );
	if ( world == NULL )
	{
		return ( b3JointId ){ 0 };
	}

	b3JointPair pair = b3CreateJoint( world, &def->base, b3_wheelJoint );

	b3JointSim* joint = pair.jointSim;

	b3WheelJoint* wheel = &joint->wheelJoint;

	wheel->suspensionHertz = def->suspensionHertz;
	wheel->suspensionDampingRatio = def->suspensionDampingRatio;
	wheel->steeringHertz = def->steeringHertz;
	wheel->steeringDampingRatio = def->steeringDampingRatio;
	wheel->spinSpeed = def->spinSpeed;
	wheel->targetSteeringAngle = def->targetSteeringAngle;

	// Both ranges sorted at creation, as the revolute's angles and the
	// prismatic's translations are: the range the caller meant, whichever order
	// they gave it in.
	wheel->lowerSuspensionLimit = b3Raw( def->lowerSuspensionLimit ) < b3Raw( def->upperSuspensionLimit )
									  ? def->lowerSuspensionLimit
									  : def->upperSuspensionLimit;
	wheel->upperSuspensionLimit = b3Raw( def->lowerSuspensionLimit ) < b3Raw( def->upperSuspensionLimit )
									  ? def->upperSuspensionLimit
									  : def->lowerSuspensionLimit;

	wheel->lowerSteeringLimit = def->lowerSteeringLimit < def->upperSteeringLimit ? def->lowerSteeringLimit
																				 : def->upperSteeringLimit;
	wheel->upperSteeringLimit = def->lowerSteeringLimit < def->upperSteeringLimit ? def->upperSteeringLimit
																				 : def->lowerSteeringLimit;

	// Clamped rather than asserted, matching every other joint's budget.
	wheel->maxSpinTorque = b3MaxF( b3f_zero, def->maxSpinTorque );
	wheel->maxSteeringTorque = b3MaxF( b3f_zero, def->maxSteeringTorque );

	wheel->enableSuspensionSpring = def->enableSuspensionSpring;
	wheel->enableSuspensionLimit = def->enableSuspensionLimit;
	wheel->enableSpinMotor = def->enableSpinMotor;
	wheel->enableSteering = def->enableSteering;
	wheel->enableSteeringLimit = def->enableSteeringLimit;

	wheel->indexA = B3_NULL_INDEX;
	wheel->indexB = B3_NULL_INDEX;

	b3JointId wheelJointId = { joint->jointId + 1, world->worldId, pair.joint->generation };
	return wheelJointId;
}

void b3DestroyJoint( b3JointId jointId, bool wakeAttached )
{
	b3World* world = b3GetUnlockedWorld( jointId.world0 );
	if ( world == NULL )
	{
		return;
	}

	b3Joint* joint = b3GetJointFullId( world, jointId );

	b3DestroyJointInternal( world, joint, wakeAttached );
}

void b3DestroyJointInternal( b3World* world, b3Joint* joint, bool wakeBodies )
{
	int jointId = joint->jointId;

	b3JointEdge* edgeA = joint->edges + 0;
	b3JointEdge* edgeB = joint->edges + 1;

	int idA = edgeA->bodyId;
	int idB = edgeB->bodyId;
	b3Body* bodyA = b3Array_Get( world->bodies, idA );
	b3Body* bodyB = b3Array_Get( world->bodies, idB );

	// Remove from body A's joint list.
	if ( edgeA->prevKey != B3_NULL_INDEX )
	{
		b3Joint* prevJoint = b3Array_Get( world->joints, edgeA->prevKey >> 1 );
		b3JointEdge* prevEdge = prevJoint->edges + ( edgeA->prevKey & 1 );
		prevEdge->nextKey = edgeA->nextKey;
	}

	if ( edgeA->nextKey != B3_NULL_INDEX )
	{
		b3Joint* nextJoint = b3Array_Get( world->joints, edgeA->nextKey >> 1 );
		b3JointEdge* nextEdge = nextJoint->edges + ( edgeA->nextKey & 1 );
		nextEdge->prevKey = edgeA->prevKey;
	}

	int edgeKeyA = ( jointId << 1 ) | 0;
	if ( bodyA->headJointKey == edgeKeyA )
	{
		bodyA->headJointKey = edgeA->nextKey;
	}

	bodyA->jointCount -= 1;

	// Remove from body B's joint list.
	if ( edgeB->prevKey != B3_NULL_INDEX )
	{
		b3Joint* prevJoint = b3Array_Get( world->joints, edgeB->prevKey >> 1 );
		b3JointEdge* prevEdge = prevJoint->edges + ( edgeB->prevKey & 1 );
		prevEdge->nextKey = edgeB->nextKey;
	}

	if ( edgeB->nextKey != B3_NULL_INDEX )
	{
		b3Joint* nextJoint = b3Array_Get( world->joints, edgeB->nextKey >> 1 );
		b3JointEdge* nextEdge = nextJoint->edges + ( edgeB->nextKey & 1 );
		nextEdge->prevKey = edgeB->prevKey;
	}

	int edgeKeyB = ( jointId << 1 ) | 1;
	if ( bodyB->headJointKey == edgeKeyB )
	{
		bodyB->headJointKey = edgeB->nextKey;
	}

	bodyB->jointCount -= 1;

	if ( joint->islandId != B3_NULL_INDEX )
	{
		B3_ASSERT( joint->setIndex > b3_disabledSet );
		b3UnlinkJoint( world, joint );
	}
	else
	{
		B3_ASSERT( joint->setIndex <= b3_disabledSet );
	}

	// Remove the joint sim from whatever owns it.
	int setIndex = joint->setIndex;
	int localIndex = joint->localIndex;

	if ( setIndex == b3_awakeSet )
	{
		b3RemoveJointFromGraph( world, joint->edges[0].bodyId, joint->edges[1].bodyId, joint->colorIndex, localIndex );
	}
	else
	{
		b3SolverSet* set = b3Array_Get( world->solverSets, setIndex );
		int movedIndex = b3Array_RemoveSwap( set->jointSims, localIndex );
		if ( movedIndex != B3_NULL_INDEX )
		{
			b3JointSim* movedJointSim = set->jointSims.data + localIndex;
			int movedId = movedJointSim->jointId;
			b3Joint* movedJoint = b3Array_Get( world->joints, movedId );
			B3_ASSERT( movedJoint->localIndex == movedIndex );
			movedJoint->localIndex = localIndex;
		}
	}

	// Free the joint and its id, preserving the generation.
	joint->setIndex = B3_NULL_INDEX;
	joint->localIndex = B3_NULL_INDEX;
	joint->colorIndex = B3_NULL_INDEX;
	joint->jointId = B3_NULL_INDEX;
	b3FreeId( &world->jointIdPool, jointId );

	if ( wakeBodies )
	{
		b3WakeBody( world, bodyA );
		b3WakeBody( world, bodyB );
	}

	b3ValidateSolverSets( world );
}

// =========================================================================
// Accessors
// =========================================================================

void b3Joint_SetConstraintTuning( b3JointId jointId, b3f hertz, b3f dampingRatio )
{
	B3_ASSERT( b3IsValidFloat( hertz ) && b3Raw( hertz ) >= 0 );
	B3_ASSERT( b3IsValidFloat( dampingRatio ) && b3Raw( dampingRatio ) >= 0 );

	b3World* world = b3GetWorld( jointId.world0 );
	b3Joint* joint = b3GetJointFullId( world, jointId );
	b3JointSim* base = b3GetJointSim( world, joint );
	base->constraintHertz = hertz;
	base->constraintDampingRatio = dampingRatio;
}

void b3Joint_GetConstraintTuning( b3JointId jointId, b3f* hertz, b3f* dampingRatio )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3Joint* joint = b3GetJointFullId( world, jointId );
	b3JointSim* base = b3GetJointSim( world, joint );
	*hertz = base->constraintHertz;
	*dampingRatio = base->constraintDampingRatio;
}

void b3Joint_SetForceThreshold( b3JointId jointId, b3f threshold )
{
	B3_ASSERT( b3IsValidFloat( threshold ) && b3Raw( threshold ) >= 0 );

	b3World* world = b3GetWorld( jointId.world0 );
	b3Joint* joint = b3GetJointFullId( world, jointId );
	b3JointSim* base = b3GetJointSim( world, joint );
	base->forceThreshold = threshold;
}

b3f b3Joint_GetForceThreshold( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3Joint* joint = b3GetJointFullId( world, jointId );
	b3JointSim* base = b3GetJointSim( world, joint );
	return base->forceThreshold;
}

void b3Joint_SetTorqueThreshold( b3JointId jointId, b3f threshold )
{
	B3_ASSERT( b3IsValidFloat( threshold ) && b3Raw( threshold ) >= 0 );

	b3World* world = b3GetWorld( jointId.world0 );
	b3Joint* joint = b3GetJointFullId( world, jointId );
	b3JointSim* base = b3GetJointSim( world, joint );
	base->torqueThreshold = threshold;
}

b3f b3Joint_GetTorqueThreshold( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3Joint* joint = b3GetJointFullId( world, jointId );
	b3JointSim* base = b3GetJointSim( world, joint );
	return base->torqueThreshold;
}

b3JointType b3Joint_GetType( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3Joint* joint = b3GetJointFullId( world, jointId );
	return joint->type;
}

b3BodyId b3Joint_GetBodyA( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3Joint* joint = b3GetJointFullId( world, jointId );
	return b3MakeBodyId( world, joint->edges[0].bodyId );
}

b3BodyId b3Joint_GetBodyB( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3Joint* joint = b3GetJointFullId( world, jointId );
	return b3MakeBodyId( world, joint->edges[1].bodyId );
}

b3WorldId b3Joint_GetWorld( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	return ( b3WorldId ){ (uint16_t)( jointId.world0 + 1 ), world->generation };
}

void b3Joint_SetLocalFrameA( b3JointId jointId, b3Transform localFrame )
{
	B3_ASSERT( b3IsValidTransform( localFrame ) );

	b3World* world = b3GetWorld( jointId.world0 );
	b3Joint* joint = b3GetJointFullId( world, jointId );
	b3JointSim* jointSim = b3GetJointSim( world, joint );
	jointSim->localFrameA = localFrame;
}

b3Transform b3Joint_GetLocalFrameA( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3Joint* joint = b3GetJointFullId( world, jointId );
	b3JointSim* jointSim = b3GetJointSim( world, joint );
	return jointSim->localFrameA;
}

void b3Joint_SetLocalFrameB( b3JointId jointId, b3Transform localFrame )
{
	B3_ASSERT( b3IsValidTransform( localFrame ) );

	b3World* world = b3GetWorld( jointId.world0 );
	b3Joint* joint = b3GetJointFullId( world, jointId );
	b3JointSim* jointSim = b3GetJointSim( world, joint );
	jointSim->localFrameB = localFrame;
}

b3Transform b3Joint_GetLocalFrameB( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3Joint* joint = b3GetJointFullId( world, jointId );
	b3JointSim* jointSim = b3GetJointSim( world, joint );
	return jointSim->localFrameB;
}

void b3Joint_SetCollideConnected( b3JointId jointId, bool shouldCollide )
{
	b3World* world = b3GetUnlockedWorld( jointId.world0 );
	if ( world == NULL )
	{
		return;
	}

	b3Joint* joint = b3GetJointFullId( world, jointId );
	if ( joint->collideConnected == shouldCollide )
	{
		return;
	}

	joint->collideConnected = shouldCollide;

	b3Body* bodyA = b3Array_Get( world->bodies, joint->edges[0].bodyId );
	b3Body* bodyB = b3Array_Get( world->bodies, joint->edges[1].bodyId );

	if ( shouldCollide )
	{
		// Tell the broad phase to look for new pairs for one of the two
		// bodies. Pick the one with the fewest shapes.
		int shapeCountA = bodyA->shapeCount;
		int shapeCountB = bodyB->shapeCount;

		int shapeId = shapeCountA < shapeCountB ? bodyA->headShapeId : bodyB->headShapeId;
		while ( shapeId != B3_NULL_INDEX )
		{
			b3Shape* shape = b3Array_Get( world->shapes, shapeId );

			if ( shape->proxyKey != B3_NULL_INDEX )
			{
				b3BufferMove( &world->broadPhase, shape->proxyKey );
			}

			shapeId = shape->nextShapeId;
		}
	}
	else
	{
		b3DestroyContactsBetweenBodies( world, bodyA, bodyB );
	}
}

bool b3Joint_GetCollideConnected( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3Joint* joint = b3GetJointFullId( world, jointId );
	return joint->collideConnected;
}

void b3Joint_SetUserData( b3JointId jointId, void* userData )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3Joint* joint = b3GetJointFullId( world, jointId );
	joint->userData = userData;
}

void* b3Joint_GetUserData( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3Joint* joint = b3GetJointFullId( world, jointId );
	return joint->userData;
}

void b3Joint_WakeBodies( b3JointId jointId )
{
	b3World* world = b3GetUnlockedWorld( jointId.world0 );
	if ( world == NULL )
	{
		return;
	}

	world->locked = true;

	b3Joint* joint = b3GetJointFullId( world, jointId );
	b3Body* bodyA = b3Array_Get( world->bodies, joint->edges[0].bodyId );
	b3Body* bodyB = b3Array_Get( world->bodies, joint->edges[1].bodyId );

	b3WakeBody( world, bodyA );
	b3WakeBody( world, bodyB );

	world->locked = false;
}

// =========================================================================
// Solver dispatch
// =========================================================================
//
// One switch per solver stage. Every joint type that lands adds one case to
// each; b3_filterJoint adds none, because it has nothing to solve. The
// `default: B3_ASSERT( false )` is what catches a type whose creator landed
// ahead of its math.

void b3PrepareJoint( b3JointSim* joint, b3StepContext* context )
{
	// Clamp joint hertz against the sub-step to reduce jitter. inv_h is Q12 and
	// reaches 240 at four sub-steps, so the quarter of it lands at 60 -- and
	// b3MulFC by a quarter is exact, being a shift of a Q12 value by two.
	b3f hertz = b3MinF( joint->constraintHertz, b3MulFC( context->inv_h, b3cFromFrac( 1, 4 ) ) );
	joint->constraintSoftness = b3MakeSoft( hertz, joint->constraintDampingRatio, context->h );

	switch ( joint->type )
	{
		case b3_distanceJoint:
			b3PrepareDistanceJoint( joint, context );
			break;

		case b3_revoluteJoint:
			b3PrepareRevoluteJoint( joint, context );
			break;

		case b3_sphericalJoint:
			b3PrepareSphericalJoint( joint, context );
			break;

		case b3_weldJoint:
			b3PrepareWeldJoint( joint, context );
			break;

		case b3_motorJoint:
			b3PrepareMotorJoint( joint, context );
			break;

		case b3_prismaticJoint:
			b3PreparePrismaticJoint( joint, context );
			break;

		case b3_parallelJoint:
			b3PrepareParallelJoint( joint, context );
			break;

		case b3_wheelJoint:
			b3PrepareWheelJoint( joint, context );
			break;

		case b3_filterJoint:
			break;

		default:
			B3_ASSERT( false );
	}
}

void b3WarmStartJoint( b3JointSim* joint, b3StepContext* context )
{
	B3_UNUSED( context );

	switch ( joint->type )
	{
		case b3_distanceJoint:
			b3WarmStartDistanceJoint( joint, context );
			break;

		case b3_revoluteJoint:
			b3WarmStartRevoluteJoint( joint, context );
			break;

		case b3_sphericalJoint:
			b3WarmStartSphericalJoint( joint, context );
			break;

		case b3_weldJoint:
			b3WarmStartWeldJoint( joint, context );
			break;

		case b3_motorJoint:
			b3WarmStartMotorJoint( joint, context );
			break;

		case b3_prismaticJoint:
			b3WarmStartPrismaticJoint( joint, context );
			break;

		case b3_parallelJoint:
			b3WarmStartParallelJoint( joint, context );
			break;

		case b3_wheelJoint:
			b3WarmStartWheelJoint( joint, context );
			break;

		case b3_filterJoint:
			break;

		default:
			B3_ASSERT( false );
	}
}

void b3SolveJoint( b3JointSim* joint, b3StepContext* context, bool useBias )
{
	B3_UNUSED( context );
	B3_UNUSED( useBias );

	switch ( joint->type )
	{
		case b3_distanceJoint:
			b3SolveDistanceJoint( joint, context, useBias );
			break;

		case b3_revoluteJoint:
			b3SolveRevoluteJoint( joint, context, useBias );
			break;

		case b3_sphericalJoint:
			b3SolveSphericalJoint( joint, context, useBias );
			break;

		case b3_weldJoint:
			b3SolveWeldJoint( joint, context, useBias );
			break;

		case b3_motorJoint:
			b3SolveMotorJoint( joint, context, useBias );
			break;

		case b3_prismaticJoint:
			b3SolvePrismaticJoint( joint, context, useBias );
			break;

		case b3_parallelJoint:
			b3SolveParallelJoint( joint, context, useBias );
			break;

		case b3_wheelJoint:
			b3SolveWheelJoint( joint, context, useBias );
			break;

		case b3_filterJoint:
			break;

		default:
			B3_ASSERT( false );
	}
}

/// The fourth switch: what force a joint is applying, for the reaction query.
///
/// Unlike the three above it runs outside the step, so it reads the world's
/// stored inv_h rather than a step context's. A filter joint applies no force
/// and is the one type that answers zero rather than being an error.
b3Vec3 b3GetJointReaction( b3World* world, b3JointSim* base )
{
	switch ( base->type )
	{
		case b3_distanceJoint:
			return b3GetDistanceJointForce( world, base );

		case b3_revoluteJoint:
			return b3GetRevoluteJointForce( world, base );

		case b3_sphericalJoint:
			return b3GetSphericalJointForce( world, base );

		case b3_weldJoint:
			return b3GetWeldJointForce( world, base );

		case b3_motorJoint:
			return b3GetMotorJointForce( world, base );

		case b3_prismaticJoint:
			return b3GetPrismaticJointForce( world, base );

		case b3_wheelJoint:
			return b3GetWheelJointForce( world, base );

		// A parallel joint constrains orientation only, so it applies no linear
		// impulse at all -- zero is the true answer here, not a placeholder for
		// a query that has not been written. It joins the filter joint on that
		// footing rather than getting a b3GetParallelJointForce that would only
		// ever return the same thing.
		case b3_parallelJoint:
		case b3_filterJoint:
			return b3Vec3_zero;

		default:
			B3_ASSERT( false );
			return b3Vec3_zero;
	}
}

b3Vec3 b3Joint_GetConstraintForce( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3Joint* joint = b3GetJointFullId( world, jointId );
	b3JointSim* base = b3GetJointSim( world, joint );
	return b3GetJointReaction( world, base );
}

/// The fifth switch: what torque a joint is applying.
///
/// Stage 3's counterpart to b3GetJointReaction, and the query Stages 1 and 2
/// both deferred because nothing yet constrained a rotation to report on. The
/// filter and distance joints answer zero -- neither constrains one, so zero
/// is the true answer rather than a placeholder.
static b3Vec3 b3GetJointConstraintTorque( b3World* world, b3JointSim* base )
{
	switch ( base->type )
	{
		case b3_revoluteJoint:
			return b3GetRevoluteJointTorque( world, base );

		case b3_sphericalJoint:
			return b3GetSphericalJointTorque( world, base );

		case b3_weldJoint:
			return b3GetWeldJointTorque( world, base );

		case b3_motorJoint:
			return b3GetMotorJointTorque( world, base );

		case b3_prismaticJoint:
			return b3GetPrismaticJointTorque( world, base );

		case b3_parallelJoint:
			return b3GetParallelJointTorque( world, base );

		case b3_wheelJoint:
			return b3GetWheelJointTorque( world, base );

		case b3_distanceJoint:
		case b3_filterJoint:
			return b3Vec3_zero;

		default:
			B3_ASSERT( false );
			return b3Vec3_zero;
	}
}

b3Vec3 b3Joint_GetConstraintTorque( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3Joint* joint = b3GetJointFullId( world, jointId );
	b3JointSim* base = b3GetJointSim( world, joint );
	return b3GetJointConstraintTorque( world, base );
}

/// The point-to-line separation shared by the prismatic and the wheel, plus
/// whatever their axial limit is violated by, combined as a single distance.
///
/// @param axis
///     The rail direction in world space, unit at Q12.
/// @param dp
///     The offset between the two joint anchor points, in world space.
/// @param limitSeparation
///     How far past its stop the axial translation is, or zero.
///
/// **This is where the port diverges from upstream, deliberately.** Upstream
/// measures the off-rail offset as `b3AbsFloat( b3Dot( b3Perp( axisA ), dp ) )`
/// -- the length of the offset along **one** arbitrary perpendicular, where a
/// point-to-line constraint removes **two**. It therefore under-reports every
/// off-axis offset that is not aligned with whichever perpendicular b3Perp
/// happened to pick, and returns **exactly zero** for one perpendicular to it.
/// The wheel joint shares the identical code and the identical defect.
///
/// The true perpendicular distance needs no perpendicular vector at all:
///
///     offAxisSquared = |dp|^2 - dot( axis, dp )^2
///
/// which is Pythagoras on the rail, accumulated wide. Sidestepping b3Perp also
/// sidesteps the Q12 short-vector conditioning b3ArbitraryPerp exists to handle,
/// so the fix is cheaper than the bug.
///
/// This is the same class as the prismatic joint's permuted reaction force that
/// Stage 6 fixed: a plain error, not a convention a caller could have tuned
/// against. Recorded here rather than silently corrected.
///
/// The two terms are combined under a single root because they are already
/// squares -- upstream takes two square roots and this takes one.
static b3f b3PointLineSeparation( b3Vec3 axis, b3Vec3 dp, b3f limitSeparation )
{
	// |dp|^2 at Q24, exact, never narrowed.
	int64_t lengthSq = b3LengthSquaredWide( dp );

	// The axial component at Q12, which is the same b3Dot the limit test above
	// uses -- so the decomposition is consistent with the number the caller
	// compared against the stops.
	int32_t axial = b3Raw( b3Dot( axis, dp ) );
	int64_t offAxisSq = lengthSq - (int64_t)axial * axial;

	// Rounding in b3Dot's narrowing can push a purely axial offset a quantum
	// past |dp|^2. That is a negative of a few units at Q24, not a geometry
	// error, and the answer it stands for is zero.
	if ( offAxisSq < 0 )
	{
		offAxisSq = 0;
	}

	int64_t limitRaw = (int64_t)b3Raw( limitSeparation );
	return b3SqrtWide( offAxisSq + limitRaw * limitRaw );
}

/// The sixth switch: how far a joint has been pulled from the position it
/// constrains, as a length.
///
/// Stage 1 deferred both separation queries until every joint type existed,
/// because each is a switch over all nine. That condition is met.
///
/// The types answering zero are answering it truly. A motor joint and a filter
/// joint constrain no position; a parallel joint constrains orientation only; a
/// distance joint with a spring and no limits is *meant* to stretch, and a weld
/// joint with a linear spring likewise.
static b3f b3GetJointLinearSeparation( b3World* world, b3Joint* joint, b3JointSim* base )
{
	b3WorldTransform xfA = b3GetBodyTransform( world, joint->edges[0].bodyId );
	b3WorldTransform xfB = b3GetBodyTransform( world, joint->edges[1].bodyId );

	b3Vec3 pA = b3TransformWorldPoint( xfA, base->localFrameA.p );
	b3Vec3 pB = b3TransformWorldPoint( xfB, base->localFrameB.p );
	b3Vec3 dp = b3SubPos( pB, pA );

	// **Frame A in world space, and this is the second upstream fix in this
	// function.** Upstream takes the rail direction as `b3RotateVector( xfA.q,
	// b3Vec3_axisX )` -- body A's own x -- while the solver's rail is the *joint
	// frame's* x, `b3MulQuat( bodyA.q, localFrameA.q )` rotated onto x
	// (prismatic_joint.c:447,463). The two agree only when localFrameA.q is the
	// identity, which is the case a joint frame exists to avoid: rotating the
	// frame is exactly how a caller aims a slider or a wheel's suspension.
	//
	// Upstream contradicts itself here rather than following a convention --
	// b3RevoluteJoint_GetAngle composes the frames, and so does every solve. A
	// slider aimed along world y through its frame therefore had upstream
	// measuring against world x, reporting travel *along* the rail as
	// perpendicular separation. Caught by test_joint_linear_separation_wheel,
	// whose rig aims the suspension axis with a quarter-turn frame.
	b3Quat quatA = b3MulQuat( xfA.q, base->localFrameA.q );

	switch ( joint->type )
	{
		case b3_parallelJoint:
		case b3_motorJoint:
		case b3_filterJoint:
			return b3f_zero;

		case b3_distanceJoint:
		{
			b3DistanceJoint* distanceJoint = &base->distanceJoint;
			b3f length = b3Length( dp );

			if ( distanceJoint->enableSpring )
			{
				if ( distanceJoint->enableLimit == false )
				{
					return b3f_zero;
				}

				if ( b3Raw( length ) < b3Raw( distanceJoint->minLength ) )
				{
					return b3SubF( distanceJoint->minLength, length );
				}

				if ( b3Raw( length ) > b3Raw( distanceJoint->maxLength ) )
				{
					return b3SubF( length, distanceJoint->maxLength );
				}

				return b3f_zero;
			}

			return b3AbsF( b3SubF( length, distanceJoint->length ) );
		}

		case b3_prismaticJoint:
		{
			b3PrismaticJoint* prismaticJoint = &base->prismaticJoint;
			b3Vec3 axisA = b3RotateVector( quatA, b3Vec3_axisX );
			b3f limitSeparation = b3f_zero;

			if ( prismaticJoint->enableLimit )
			{
				b3f translation = b3Dot( axisA, dp );

				if ( b3Raw( translation ) < b3Raw( prismaticJoint->lowerTranslation ) )
				{
					limitSeparation = b3SubF( prismaticJoint->lowerTranslation, translation );
				}

				if ( b3Raw( prismaticJoint->upperTranslation ) < b3Raw( translation ) )
				{
					limitSeparation = b3SubF( translation, prismaticJoint->upperTranslation );
				}
			}

			return b3PointLineSeparation( axisA, dp, limitSeparation );
		}

		case b3_revoluteJoint:
		case b3_sphericalJoint:
			return b3Length( dp );

		case b3_weldJoint:
		{
			b3WeldJoint* weldJoint = &base->weldJoint;
			return b3Raw( weldJoint->linearHertz ) == 0 ? b3Length( dp ) : b3f_zero;
		}

		case b3_wheelJoint:
		{
			b3WheelJoint* wheelJoint = &base->wheelJoint;
			b3Vec3 axisA = b3RotateVector( quatA, b3Vec3_axisX );
			b3f limitSeparation = b3f_zero;

			if ( wheelJoint->enableSuspensionLimit )
			{
				b3f translation = b3Dot( axisA, dp );

				if ( b3Raw( translation ) < b3Raw( wheelJoint->lowerSuspensionLimit ) )
				{
					limitSeparation = b3SubF( wheelJoint->lowerSuspensionLimit, translation );
				}

				if ( b3Raw( wheelJoint->upperSuspensionLimit ) < b3Raw( translation ) )
				{
					limitSeparation = b3SubF( translation, wheelJoint->upperSuspensionLimit );
				}
			}

			return b3PointLineSeparation( axisA, dp, limitSeparation );
		}

		default:
			B3_ASSERT( false );
			return b3f_zero;
	}
}

/// The seventh switch: the same question about orientation, as an angle.
///
/// **Returns b3a brads, not a length.** Upstream returns `float` from both
/// separation queries because a float carries either. The port's angles are
/// brads everywhere a caller sees one -- b3RevoluteJoint_GetAngle,
/// b3SphericalJoint_GetConeAngle, b3WheelJoint_GetSteeringAngle -- and a query
/// that returned a radian-valued b3f here would be the one exception.
///
/// The `relQ.v.z = b3n_zero` lines are upstream's "remove the hinge angle", and
/// **they only became meaningful in this stage**. Against the old
/// `b3GetQuatAngle` -- `2*acos(|s|)`, which reads the scalar part alone -- zeroing
/// a vector component changed nothing at all, so upstream's line was a silent
/// no-op. Step 1 rewrote it to `2*atan2(|v|, s)`, which reads the vector part, so
/// the line now does what it says. No second entry point was needed, which is a
/// simplification against what this step was originally planned to require.
static b3a b3GetJointAngularSeparation( b3World* world, b3Joint* joint, b3JointSim* base )
{
	b3WorldTransform xfA = b3GetBodyTransform( world, joint->edges[0].bodyId );
	b3WorldTransform xfB = b3GetBodyTransform( world, joint->edges[1].bodyId );

	// The two joint frames in world space, then B relative to A -- the same
	// composition b3RevoluteJoint_GetAngle does, and the same fix the linear
	// query needed. Upstream writes `b3InvMulQuat( xfA.q, xfB.q )` here and so
	// measures between the *bodies* rather than between the frames they are
	// jointed by, which is only the same thing when both local frames are the
	// identity.
	b3Quat quatA = b3MulQuat( xfA.q, base->localFrameA.q );
	b3Quat quatB = b3MulQuat( xfB.q, base->localFrameB.q );

	b3Quat relQ = b3InvMulQuat( quatA, quatB );

	switch ( joint->type )
	{
		case b3_distanceJoint:
		case b3_motorJoint:
		case b3_filterJoint:
			return 0;

		case b3_parallelJoint:
			// Remove the hinge angle: a parallel joint permits free rotation
			// about z and constrains the other two.
			relQ.v.z = b3n_zero;
			return b3GetQuatAngle( relQ );

		case b3_prismaticJoint:
			return b3GetQuatAngle( relQ );

		case b3_revoluteJoint:
		{
			b3RevoluteJoint* revoluteJoint = &base->revoluteJoint;

			// Outside its limit the hinge angle is itself a violation, so it is
			// kept; inside, only the off-axis error counts.
			if ( revoluteJoint->enableLimit )
			{
				b3a angle = b3GetTwistAngle( relQ );
				if ( angle < revoluteJoint->lowerAngle || angle > revoluteJoint->upperAngle )
				{
					return b3GetQuatAngle( relQ );
				}
			}

			relQ.v.z = b3n_zero;
			return b3GetQuatAngle( relQ );
		}

		case b3_sphericalJoint:
		{
			b3SphericalJoint* sphericalJoint = &base->sphericalJoint;

			// Up to three excesses summed, so int32_t rather than b3a: a cone
			// term and two twist terms of a half turn each overflow an int16,
			// and a separation past a half turn says nothing a half turn does
			// not. Upstream sums in float and never meets the question.
			int32_t sum = 0;

			if ( sphericalJoint->enableConeLimit )
			{
				b3a swingAngle = b3GetSwingAngle( relQ );
				if ( swingAngle > sphericalJoint->coneAngle )
				{
					sum += (int32_t)swingAngle - (int32_t)sphericalJoint->coneAngle;
				}
			}

			if ( sphericalJoint->enableTwistLimit )
			{
				b3a twistAngle = b3GetTwistAngle( relQ );
				if ( twistAngle < sphericalJoint->lowerTwistAngle )
				{
					sum += (int32_t)sphericalJoint->lowerTwistAngle - (int32_t)twistAngle;
				}

				if ( twistAngle > sphericalJoint->upperTwistAngle )
				{
					sum += (int32_t)twistAngle - (int32_t)sphericalJoint->upperTwistAngle;
				}
			}

			const int32_t halfTurn = B3_BRAD_CIRCLE / 2;
			return (b3a)( sum > halfTurn ? halfTurn : sum );
		}

		case b3_weldJoint:
		{
			b3WeldJoint* weldJoint = &base->weldJoint;
			return b3Raw( weldJoint->angularHertz ) == 0 ? b3GetQuatAngle( relQ ) : 0;
		}

		case b3_wheelJoint:
			// Upstream asserts false here. See b3GetWheelJointAngularSeparation.
			return b3GetWheelJointAngularSeparation( world, base );

		default:
			B3_ASSERT( false );
			return 0;
	}
}

/// The scalar reaction magnitudes, for the joint-event threshold test.
///
/// **Derived from the two vector queries rather than transliterated from
/// upstream's third switch.** Upstream carries a *separate*
/// `b3GetJointReaction( world, sim, invTimeStep, float*, float* )`
/// (`helpSrc/box3d-main/src/joint.c:1001-1110`) that recomposes raw impulse
/// components per type -- and whose wheel case is marked `// todo probably
/// wrong` by its own author. Importing a switch upstream distrusts, to sit
/// alongside two switches the port already has, would give three places for the
/// same answer to drift apart.
///
/// Taking the length of what b3Joint_GetConstraintForce and
/// b3Joint_GetConstraintTorque report costs one switch instead of three and is
/// automatically consistent with the public API: a caller who sets a threshold
/// of 500 N and then reads the force back gets the number the threshold was
/// compared against.
///
/// The one honest difference: upstream composes scalar impulse components
/// directly (the distance joint sums scalars and takes b3AbsFloat), where this
/// takes the magnitude of an assembled world-space vector. For a joint applying
/// force along a single axis the two agree; where upstream's per-type
/// composition and its own vector query disagree, this follows the vector query.
void b3GetJointReactionScalars( b3World* world, b3JointSim* base, b3f* forceOut, b3f* torqueOut )
{
	*forceOut = b3Length( b3GetJointReaction( world, base ) );
	*torqueOut = b3Length( b3GetJointConstraintTorque( world, base ) );
}

b3f b3Joint_GetLinearSeparation( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3Joint* joint = b3GetJointFullId( world, jointId );
	b3JointSim* base = b3GetJointSim( world, joint );
	return b3GetJointLinearSeparation( world, joint, base );
}

b3a b3Joint_GetAngularSeparation( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3Joint* joint = b3GetJointFullId( world, jointId );
	b3JointSim* base = b3GetJointSim( world, joint );
	return b3GetJointAngularSeparation( world, joint, base );
}
