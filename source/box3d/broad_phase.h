// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Erin Catto        (original Box3D)
// Copyright (c) 2026 Warioware64       (Nitro Engine Advanced fixed-point port)
//
// This file is part of Nitro Engine Advanced

#pragma once

#include "bitset.h"
#include "container.h"
#include "table.h"

#include "box3d/collision.h"
#include "box3d/types.h"

typedef struct b3MovePair b3MovePair;
typedef struct b3MoveResult b3MoveResult;
typedef struct b3World b3World;

// Store the proxy type in the lower 2 bits of the proxy key. This leaves 30 bits for the id.
#define B3_PROXY_TYPE( KEY ) ( (b3BodyType)( ( KEY ) & 3 ) )
#define B3_PROXY_ID( KEY ) ( ( KEY ) >> 2 )
#define B3_PROXY_KEY( ID, TYPE ) ( ( ( ID ) << 2 ) | ( TYPE ) )

/// The broad-phase is used for computing pairs and performing volume queries and ray casts.
/// This broad-phase does not persist pairs. Instead, this reports potentially new pairs.
/// It is up to the client to consume the new pairs and to track subsequent overlap.
typedef struct b3BroadPhase
{
	b3DynamicTree trees[b3_bodyTypeCount];

	// Per body-type bit sets indexed by proxyId, marking proxies moved this step.
	// Paired with moveArray which preserves deterministic insertion order for pair queries.
	b3BitSet movedProxies[b3_bodyTypeCount];
	b3Array( int ) moveArray;

	// Tracks shape pairs that have a b3Contact
	b3HashSet pairSet;

	// The output of b3UpdateBroadPhasePairs, live only for the duration of
	// that call: one b3MoveResult per moved proxy, each heading a linked list
	// of pairs drawn from the flat movePairs block. Both are stack-allocated
	// at the top of the call and freed at the bottom, so these are NULL
	// between steps.
	//
	// The list-per-result shape is what makes contact creation deterministic:
	// pairs are discovered in whatever order the tree walk finds them, but
	// consumed in moveArray order, which is insertion order.
	b3MoveResult* moveResults;
	b3MovePair* movePairs;
	int movePairCapacity;

	// Upstream's is a b3AtomicInt claimed by b3AtomicFetchAddInt across
	// workers. One worker here, so it is a plain counter -- platform.h already
	// maps the atomic ops to plain ones.
	int movePairIndex;
} b3BroadPhase;

void b3CreateBroadPhase( b3BroadPhase* bp, const b3Capacity* capacity );
void b3DestroyBroadPhase( b3BroadPhase* bp );

int b3BroadPhase_CreateProxy( b3BroadPhase* bp, b3BodyType proxyType, b3AABB aabb, uint64_t categoryBits, int shapeIndex,
							  bool forcePairCreation );
void b3BroadPhase_DestroyProxy( b3BroadPhase* bp, int proxyKey );

void b3BroadPhase_MoveProxy( b3BroadPhase* bp, int proxyKey, b3AABB aabb );
void b3BroadPhase_EnlargeProxy( b3BroadPhase* bp, int proxyKey, b3AABB aabb );

int b3BroadPhase_GetShapeIndex( b3BroadPhase* bp, int proxyKey );

/// Find every new overlapping pair among the proxies that moved this step and
/// create a contact for each. Clears the move buffer.
///
/// "New" is doing real work: a pair that already has a contact is rejected by
/// the pairSet, and a pair whose *both* proxies moved is reported once rather
/// than twice. Contacts are created in moveArray order, not discovery order,
/// so the result does not depend on the shape of the tree.
void b3UpdateBroadPhasePairs( b3World* world );

/// Bytes of b3Stack the pair update holds live at its peak, for `moveCount`
/// moved proxies.
///
/// b3CreateWorld sizes the stack from this so that b3GrowStack never has to
/// run -- growing calls b3Alloc, and NEA_Phys3D's pool allocator refuses to
/// allocate once the world exists. The definition lives here rather than at the
/// call site because b3MovePair and b3MoveResult are private to broad_phase.c.
int b3BroadPhaseStackDemand( int moveCount );

bool b3BroadPhase_TestOverlap( const b3BroadPhase* bp, int proxyKeyA, int proxyKeyB );

void b3ValidateBroadPhase( const b3BroadPhase* bp );
void b3ValidateNoEnlarged( const b3BroadPhase* bp );

// This is what triggers new contact pairs to be created
// Warning: this must be called in deterministic order
static inline void b3BufferMove( b3BroadPhase* bp, int queryProxy )
{
	b3BodyType proxyType = B3_PROXY_TYPE( queryProxy );
	int proxyId = B3_PROXY_ID( queryProxy );
	b3BitSet* set = &bp->movedProxies[proxyType];
	if ( b3GetBit( set, proxyId ) == false )
	{
		b3SetBitGrow( set, proxyId );
		b3Array_Push( bp->moveArray, queryProxy );
	}
}
