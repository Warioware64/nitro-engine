// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Erin Catto        (original Box3D)
// Copyright (c) 2026 Warioware64       (Nitro Engine Advanced fixed-point port)
//
// This file is part of Nitro Engine Advanced

#pragma once

/// @file   mesh_contact.h
/// @brief  The shape-versus-triangle-mesh narrow phase.
///
/// Upstream declares b3ComputeMeshManifolds in contact.h. It has its own header
/// here because the port's b3CreateWorld has to size the arena from it before a
/// single mesh contact exists, and physics_world.c should not have to include
/// the narrow phase to ask a question about memory.

#include "arena_allocator.h"

#include "box3d/base.h"
#include "box3d/math_fixed.h"

typedef struct b3Contact b3Contact;
typedef struct b3Shape b3Shape;
typedef struct b3World b3World;

/// Peak arena bytes one call to b3ComputeMeshManifolds may hold at once.
///
/// b3CreateWorld reserves this when the world declares any mesh contacts, so
/// that the narrow phase never grows the arena mid-step. It is defined next to
/// the b3Bump calls it accounts for, which is the same rule
/// b3SolverStackDemand (solver.c) and b3BroadPhaseStackDemand (broad_phase.c)
/// follow: adding a scratch buffer and forgetting to account for it stays a
/// one-file mistake.
int b3MeshContactArenaDemand( void );

/// Collide every triangle of shape A's mesh that overlaps shape B, and reduce
/// the result to at most B3_NEA_MAX_MESH_MANIFOLDS manifolds on the contact.
///
/// Shape A is the mesh -- the contact registers guarantee it, so unlike the
/// convex path there is no flipped case. Shape B is a sphere, capsule or hull.
///
/// The manifolds are already allocated (b3CreateContact takes them at the cap
/// and keeps them), so this only ever writes `contact->manifoldCount` and the
/// manifolds themselves; it never reaches the block allocator.
///
/// `arena` is by value: everything this bumps is released by the copy going out
/// of scope. `isFast` comes from the bodies' continuous-motion flags and only
/// suppresses the cached edge axis, which a fast hull can tunnel through.
///
/// @return true when at least one triangle produced points.
bool b3ComputeMeshManifolds( b3World* world, b3Contact* contact, const b3Shape* shapeA, b3WorldTransform xfA,
							 const b3Shape* shapeB, b3WorldTransform xfB, bool isFast, b3Arena arena );
