// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Erin Catto        (original Box3D)
// Copyright (c) 2026 Warioware64       (Nitro Engine Advanced fixed-point port)
//
// This file is part of Nitro Engine Advanced

#pragma once

/// @file   island.h
/// @brief  Connected components of touching bodies.
///
/// An island is a set of awake bodies joined by touching contacts and joints,
/// and it exists so the whole group can be put to sleep or woken at once. It
/// is a dynamic-connectivity problem: contacts appearing merge islands, which
/// is cheap, and contacts disappearing may split one, which is not -- so a
/// split is deferred behind `constraintRemoveCount` and only paid when the
/// island is actually a candidate for sleeping.
///
/// Contacts and joints may connect to static bodies, but static bodies are
/// never *in* an island. That is what keeps the ground from joining every
/// island in the scene into one.
///
/// Nothing here is fixed-point: it is all indices, ids and counts, and it
/// transliterates unchanged. Absent: `b3SplitIslandTask`, the task-system
/// wrapper that also stamps a profile timer -- there is one core here and
/// `b3SplitIsland` is called directly.

#include "container.h"

#include <stdint.h>

typedef struct b3Contact b3Contact;
typedef struct b3Joint b3Joint;
typedef struct b3World b3World;

/// Contact data cached in the island for contiguous iteration.
///
/// Carrying the two body ids here is what lets b3SplitIsland's union-find pass
/// run without touching b3Contact at all -- the split walks these arrays
/// repeatedly and b3Contact is 200-odd bytes of mostly irrelevant manifold.
typedef struct b3ContactLink
{
	int contactId;
	int bodyIdA;
	int bodyIdB;
} b3ContactLink;

b3DeclareArray( b3ContactLink );

/// Joint data cached in the island, for the same reason.
typedef struct b3JointLink
{
	int jointId;
	int bodyIdA;
	int bodyIdB;
} b3JointLink;

b3DeclareArray( b3JointLink );

typedef struct b3Island
{
	/// Index of the solver set holding this island. May be B3_NULL_INDEX.
	int setIndex;

	/// Island index within that set. May be B3_NULL_INDEX.
	int localIndex;

	int islandId;

	/// How many constraints have been removed from this island since the last
	/// split. Non-zero makes the island a split candidate; it does not mean
	/// the island is actually disconnected.
	int constraintRemoveCount;

	/// Upstream's note, worth keeping: a stack array does not work here,
	/// because the data pointer goes out of sync when the world island array
	/// grows.
	b3Array( int ) bodies;

	/// Contacts and joints belonging to this island. Either may connect to a
	/// static body that is not in it.
	b3Array( b3ContactLink ) contacts;
	b3Array( b3JointLink ) joints;

} b3Island;

/// The island's presence in a solver set. Moving an island between sets is
/// moving one of these.
typedef struct b3IslandSim
{
	int islandId;
} b3IslandSim;

b3Island* b3CreateIsland( b3World* world, int setIndex );
void b3DestroyIsland( b3World* world, int islandId );

/// Link a contact into the island graph when it starts having contact points.
void b3LinkContact( b3World* world, b3Contact* contact );

/// Unlink a contact when it stops having contact points, or is destroyed.
void b3UnlinkContact( b3World* world, b3Contact* contact );

/// Link a joint into the island graph when it is created.
void b3LinkJoint( b3World* world, b3Joint* joint );

/// Unlink a joint when it is destroyed.
void b3UnlinkJoint( b3World* world, b3Joint* joint );

void b3SplitIsland( b3World* world, int baseId );

void b3ValidateIsland( b3World* world, int islandId );
