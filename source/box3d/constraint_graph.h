// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Erin Catto        (original Box3D)
// Copyright (c) 2026 Warioware64       (Nitro Engine Advanced fixed-point port)
//
// This file is part of Nitro Engine Advanced

#pragma once

/// @file   constraint_graph.h
/// @brief  Where touching contacts and joints live while awake.
///
/// @section colors One colour
///
/// Upstream colours the constraint graph so that no two constraints in a
/// colour share a body, which lets worker threads solve a colour in parallel
/// without two of them writing the same body. `nea_config.h` sets
/// `B3_GRAPH_COLOR_COUNT` to 1: there is one core, so there is nothing to
/// parallelize and every constraint goes into the single overflow colour.
///
/// Three things fall out of that, and all three are gone here:
///
///   - **The per-colour `bodySet` bitset.** Its only job is answering "is
///     either body already in this colour", and with one colour the answer
///     never changes the outcome. Upstream sizes one bitset per colour to the
///     body capacity, so this is also the largest single allocation the graph
///     would have made.
///   - **`convexContacts` and `wideConstraints`.** The separate array and the
///     4-wide constraint block exist to feed the SIMD solver path, which the
///     port dropped in Phase 2B. Every contact is scalar here.
///   - **`B3_DYNAMIC_COLOR_COUNT`**, which is `B3_GRAPH_COLOR_COUNT - 4` and
///     evaluates to **-3** at one colour. It reserved the last few colours for
///     dynamic-versus-static constraints so they solve at higher priority; with
///     one colour there is no priority to express.
///
/// The file and its API are kept so the diff against upstream stays legible
/// and so Phase 3C's solver reads like upstream's.

#include "container.h"
#include "contact.h"
#include "solver_set.h"

#include "box3d/constants.h"

typedef struct b3Body b3Body;
typedef struct b3Contact b3Contact;
typedef struct b3Joint b3Joint;
typedef struct b3JointSim b3JointSim;
typedef struct b3World b3World;

_Static_assert( B3_GRAPH_COLOR_COUNT == 1, "the port assumes a single, overflow-only graph colour" );
_Static_assert( B3_OVERFLOW_INDEX == 0, "B3_OVERFLOW_INDEX must be the only colour" );

typedef struct b3GraphColor
{
	/// Awake touching contacts, cache friendly. Upstream splits these between
	/// `contacts` and `convexContacts`; there is one array here because there
	/// is one solver path.
	b3Array( b3ContactSpec ) contacts;

	b3Array( b3JointSim ) jointSims;

	/// Filled by the solver's prepare pass each step. Phase 3C.
	struct b3ManifoldConstraint* manifoldConstraints;
	int manifoldConstraintCount;
	struct b3ContactConstraint* contactConstraints;
	int contactConstraintCount;
} b3GraphColor;

typedef struct b3ConstraintGraph
{
	b3GraphColor colors[B3_GRAPH_COLOR_COUNT];
} b3ConstraintGraph;

/// @param contactCapacity Touching contacts to reserve the overflow colour for.
///                        Upstream takes a body capacity here and uses it to
///                        size per-colour bitsets that this port does not have.
void b3CreateGraph( b3ConstraintGraph* graph, int contactCapacity, int jointCapacity );
void b3DestroyGraph( b3ConstraintGraph* graph );

void b3AddContactToGraph( b3World* world, b3Contact* contact );

/// @param bodyIdA,bodyIdB  unused at one colour -- upstream clears their bits
///                         in the colour's body set. Kept in the signature so
///                         the call sites still read like upstream's.
void b3RemoveContactFromGraph( b3World* world, int bodyIdA, int bodyIdB, int colorIndex, int localIndex );

b3JointSim* b3CreateJointInGraph( b3World* world, b3Joint* joint );
void b3AddJointToGraph( b3World* world, b3JointSim* jointSim, b3Joint* joint );
void b3RemoveJointFromGraph( b3World* world, int bodyIdA, int bodyIdB, int colorIndex, int localIndex );
