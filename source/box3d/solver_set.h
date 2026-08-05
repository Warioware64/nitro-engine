// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Erin Catto        (original Box3D)
// Copyright (c) 2026 Warioware64       (Nitro Engine Advanced fixed-point port)
//
// This file is part of Nitro Engine Advanced

#pragma once

/// @file   solver_set.h
/// @brief  Solver sets: the port's answer to "where does a body live".
///
/// A set is a group of bodies stored in contiguous arrays. There are three
/// fixed sets and then one per sleeping island:
///
///   b3_staticSet     every static body
///   b3_disabledSet   every disabled body, plus non-touching contacts whose
///                    bodies have gone to sleep
///   b3_awakeSet      every active body, with velocity state, plus the awake
///                    set's non-touching contacts
///   3 and up         one per sleeping island, with its touching contacts
///
/// The point is memory locality: the solver walks the awake set's arrays and
/// touches nothing else. Sleeping a body is a move between arrays rather than
/// a flag, which is why so much of this file is index fixups.

#include "container.h"

typedef struct b3Body b3Body;
typedef struct b3BodySim b3BodySim;
typedef struct b3BodyState b3BodyState;
typedef struct b3IslandSim b3IslandSim;
typedef struct b3Joint b3Joint;
typedef struct b3JointSim b3JointSim;
typedef struct b3World b3World;

b3DeclareArray( b3BodySim );
b3DeclareArray( b3BodyState );
b3DeclareArray( b3JointSim );
b3DeclareArray( b3IslandSim );

typedef struct b3SolverSet
{
	/// Body sims. Empty for an unused set.
	b3Array( b3BodySim ) bodySims;

	/// Velocity state. Only the awake set has any.
	b3Array( b3BodyState ) bodyStates;

	/// Sleeping and disabled joints. Empty for the static and awake sets.
	b3Array( b3JointSim ) jointSims;

	/// For a sleeping set, all of its contacts. For the awake set, only the
	/// non-touching ones -- the touching ones live in the constraint graph.
	/// Empty for the static set.
	b3Array( int ) contactIndices;

	/// The awake set holds every awake island. A sleeping set normally holds
	/// one, but joints created between sleeping sets merge them, so it can
	/// hold several until the set is woken.
	b3Array( b3IslandSim ) islandSims;

	/// Aligns with b3World::solverSetIdPool.
	int setIndex;
} b3SolverSet;

/// Retire a set: empty it and return its index to the id pool, **keeping its
/// array capacity** so the next island to sleep reuses it rather than
/// allocating inside b3World_Step. Slots retired this way still own memory --
/// b3DestroyWorld frees every slot through b3FreeSolverSetArrays.
void b3DestroySolverSet( b3World* world, int setIndex );

/// Release a slot's five arrays. World teardown only.
void b3FreeSolverSetArrays( b3SolverSet* set );

void b3WakeSolverSet( b3World* world, int setIndex );
/// Move an island and everything on it to a sleeping set.
///
/// @return true if the island slept and was swap-removed from the awake set,
/// false if it declined -- which it does while a split is pending. A caller
/// iterating the awake island array must not assume removal; see b3Solve.
bool b3TrySleepIsland( b3World* world, int islandId );

/// Merge set 2 into set 1, then destroy set 2.
/// @warning Any pointers into either set are orphaned.
void b3MergeSolverSets( b3World* world, int setIndex1, int setIndex2 );

void b3TransferBody( b3World* world, b3SolverSet* targetSet, b3SolverSet* sourceSet, b3Body* body );
void b3TransferJoint( b3World* world, b3SolverSet* targetSet, b3SolverSet* sourceSet, b3Joint* joint );
