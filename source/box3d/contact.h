// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Erin Catto        (original Box3D)
// Copyright (c) 2026 Warioware64       (Nitro Engine Advanced fixed-point port)
//
// This file is part of Nitro Engine Advanced

#pragma once

/// @file   contact.h
/// @brief  The persistent interaction between two shapes.
///
/// The struct and the flags land in Phase 3A because solver_set.c, island.c,
/// body.c and shape.c all move contacts between sets, unlink them from islands
/// and destroy them. contact.c -- creation, the narrow-phase dispatch table
/// and b3UpdateContact -- is Phase 3B, so in this increment `world->contacts`
/// is always empty and every loop over it is a no-op.
///
/// Phase 5 added b3MeshContact and the triangle cache, and with them the union
/// upstream needs: a convex contact carries one b3ContactCache, a mesh contact
/// carries one per overlapping triangle.

#include "arena_allocator.h"
#include "container.h"

#include "box3d/box3d.h"
#include "box3d/collision.h"
#include "box3d/nea_config.h"
#include "box3d/types.h"

typedef struct b3Shape b3Shape;
typedef struct b3World b3World;

/// The temporal cache the narrow phase keeps between steps.
///
/// b3SATCache for a hull pair, b3SimplexCache for anything GJK answered. Which
/// one is live follows from the shape pair, which does not change over the life
/// of the contact -- and Phase 2B already made the SAT cache bounds-check its
/// indices rather than trust them, because a shape swap can leave it stale.
typedef union b3ContactCache
{
	b3SATCache satCache;
	b3SimplexCache simplexCache;
} b3ContactCache;

/// One overlapping triangle's warm cache, keyed by its index in the mesh.
///
/// The array of these is kept sorted by triangleIndex, which is what lets the
/// refresh match last step's caches to this step's triangles with a linear
/// merge join rather than a search. b3QueryMesh emits ascending, duplicate-free
/// indices because the baker stores triangles in depth-first leaf order; break
/// that and nothing asserts, the cache simply stops matching.
typedef struct b3TriangleCache
{
	int triangleIndex;
	b3ContactCache cache;
} b3TriangleCache;

/// The mesh half of a contact: which triangles were overlapping when the query
/// last ran, and the bounds that query covered.
///
/// `triangleCache` points at B3_NEA_MAX_MESH_CONTACT_TRIANGLES entries taken
/// from the world's triangle-cache block allocator at contact creation. Upstream
/// grows a b3Array here instead, which is a reallocation inside the step -- the
/// exact thing test_no_allocation_while_stepping holds to zero. The array is
/// therefore fixed capacity, which the BVH query already bounds it to anyway,
/// and `triangleCount` moves freely inside it.
typedef struct b3MeshContact
{
	b3TriangleCache* triangleCache;
	int triangleCount;

	/// The world-space box the last query covered, enlarged so that small
	/// movement does not force a re-query. Zeroed at creation, which contains
	/// nothing, so the first refresh always queries.
	b3AABB queryBounds;
} b3MeshContact;

enum b3ContactFlags
{
	// Set when the solid shapes are touching.
	b3_contactTouchingFlag = 0x00000001,

	// This contact has a hit event.
	b3_contactHitEventFlag = 0x00000002,

	// This contact wants contact events.
	b3_contactEnableContactEvents = 0x00000004,

	// This contact is between a dynamic and a static body.
	b3_contactStaticFlag = 0x00000008,

	b3_contactRecycleFlag = 0x00000010,

	// Set when the shapes are touching.
	b3_simTouchingFlag = 0x00010000,

	// This contact no longer has overlapping AABBs.
	b3_simDisjoint = 0x00020000,

	// This contact started touching.
	b3_simStartedTouching = 0x00040000,

	// This contact stopped touching.
	b3_simStoppedTouching = 0x00080000,

	// This contact has a hit event.
	b3_simEnableHitEvent = 0x00100000,

	// This contact wants pre-solve events.
	b3_simEnablePreSolveEvents = 0x00200000,

	// Shape A is a triangle mesh, so this contact carries several manifolds and
	// a per-triangle cache. Set by b3CreateContact from the shape type; nothing
	// sets it until the mesh narrow phase lands in stage 3, but the code that
	// has to behave differently for a mesh contact reads it already -- see the
	// spec refresh in b3Collide.
	b3_simMeshContact = 0x00400000,

	// The relative transform is cached for contact recycling.
	b3_relativeTransformValid = 0x00800000,

	// Enable speculative contact points.
	b3_enableSpeculativePoints = 0x01000000,
};

/// One end of a contact in a body's doubly linked contact list.
typedef struct b3ContactEdge
{
	int bodyId;
	int prevKey;
	int nextKey;
} b3ContactEdge;

/// The persistent interaction between two shapes.
typedef struct b3Contact
{
	/// Index of the solver set holding this contact. B3_NULL_INDEX when free.
	int setIndex;

	/// Index into the constraint graph colour array. B3_NULL_INDEX for a
	/// non-touching or sleeping contact, and when free.
	int colorIndex;

	/// Contact index within its set or graph colour. B3_NULL_INDEX when free.
	int localIndex;

	b3ContactEdge edges[2];
	int shapeIdA;
	int shapeIdB;

	/// Child shape index. Always zero here: only compound shapes create child
	/// proxies, and a mesh is *one* proxy with an internal BVH, so a mesh
	/// contact addresses its triangles through b3MeshContact rather than
	/// through this. Kept because the broad phase pair key carries it.
	int childIndex;

	/// A contact belongs to an island only while touching.
	int islandId;

	/// Index into the island's contacts array, for O(1) swap removal.
	int islandIndex;

	/// Back index into b3World::contacts.
	int contactId;

	/// Transient, cached per step. B3_NULL_INDEX for a static body.
	int bodySimIndexA;
	int bodySimIndexB;

	/// b3ContactFlags
	uint32_t flags;

	b3Manifold* manifolds;

	/// Manifolds carrying contact points this step. Exactly 1 while a convex
	/// contact is touching; 0 to `manifoldCapacity` for a mesh contact, whose
	/// normal clusters come and go as the shape slides across the level.
	int manifoldCount;

	/// Manifolds `manifolds` was allocated for, and therefore the block
	/// allocator size class it must be returned to.
	///
	/// Distinct from manifoldCount on purpose. Upstream reallocates a mesh
	/// contact whenever its cluster count changes, which is an allocator round
	/// trip *inside the step* -- the class of bug Phase 4 exists to have
	/// removed. Here a mesh contact takes B3_NEA_MAX_MESH_MANIFOLDS once and
	/// keeps them, so manifoldCount moves freely and the allocator is untouched.
	uint16_t manifoldCapacity;

	/// Cache for contact recycling: if neither body has moved or turned much
	/// since the last step, the manifold is reused rather than rebuilt.
	b3Quat cachedRotationA;
	b3Quat cachedRotationB;
	b3Transform cachedRelativePose;

	/// Mixed material properties. Friction and restitution are dimensionless
	/// and bounded by 1, so b3c.
	b3c friction;
	b3c restitution;

	/// Rolling resistance is *not* dimensionless, despite the material field
	/// of the same name being so. It is the material coefficient multiplied by
	/// a shape radius, and the solver forms `rollingResistance * normalImpulse`
	/// as an angular impulse -- which only balances if this carries units of
	/// length. So b3f, at the length scale, not b3c.
	b3f rollingResistance;

	/// Conveyor-belt surface velocity, in world space.
	b3Vec3 tangentVelocity;

	/// The narrow-phase temporal cache, in the two forms a contact can want it.
	///
	/// Which half is live follows from b3_simMeshContact, which b3CreateContact
	/// sets from the shape pair and nothing ever changes -- so b3DestroyContact
	/// must test the flag before returning `meshContact.triangleCache`, or it
	/// hands the allocator a b3SATCache's bit pattern.
	union
	{
		b3ContactCache cache;
		b3MeshContact meshContact;
	};

	/// Monotonically advanced when a contact is allocated in this slot, so a
	/// stale b3ContactId can be detected.
	uint32_t generation;
} b3Contact;

/// A contact as the solver's prepare pass sees it: an id plus where its
/// manifold constraints start in the global array.
typedef struct b3ContactSpec
{
	int contactId;

	/// Start of the global manifold constraint array.
	int manifoldStart;
	uint16_t manifoldCount;
} b3ContactSpec;

b3DeclareArray( b3ContactSpec );

// b3Contact_GetData is declared in include/box3d/box3d.h with the rest of the
// public API.

void b3InitializeContactRegisters( void );

/// Register a new contact for a pair the broad phase just found overlapping.
///
/// Silently does nothing when the shape pair has no narrow phase, and recurses
/// once with the arguments swapped when the pair was registered the other way
/// round -- so callers need not order the shapes.
///
/// The contact is created *non-touching*. Finding it touching is b3Collide's
/// job, and is what links the islands and moves it into the constraint graph.
void b3CreateContact( b3World* world, b3Shape* shapeA, b3Shape* shapeB, int childIndex );

void b3DestroyContact( b3World* world, b3Contact* contact, bool wakeBodies );

/// Rebuild the contact's manifold against the given body transforms and
/// update its touching state.
///
/// Does not assume the shape AABBs still overlap, or are even valid.
///
/// `arena` is taken by value: the narrow phase bumps a scratch buffer out of
/// it and relies on the copy going out of scope to release it.
///
/// Upstream also takes `workerIndex`, for the per-worker counters and arena.
/// That does not survive the port; `isFast` does, and is read only by the mesh
/// path -- a hull rotating fast about a triangle edge can tunnel through it, so
/// the cached edge axis is discarded rather than replayed.
///
/// @return true when the shapes are touching. A convex contact then owns
/// exactly one manifold; a mesh contact owns one per normal cluster, up to
/// B3_NEA_MAX_MESH_MANIFOLDS. When false, neither owns any points -- but a mesh
/// contact keeps its allocation, see b3Contact::manifoldCapacity.
bool b3UpdateContact( b3World* world, b3Contact* contact, b3Shape* shapeA, b3Vec3 localCenterA, b3WorldTransform xfA,
					  b3Shape* shapeB, b3Vec3 localCenterB, b3WorldTransform xfB, bool isFast, b3Arena arena );
