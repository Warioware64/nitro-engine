// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Erin Catto        (original Box3D)
// Copyright (c) 2026 Warioware64       (Nitro Engine Advanced fixed-point port)
//
// This file is part of Nitro Engine Advanced

// Proxy management and pair finding for the broad phase.
//
// This is the second file the port converts in two passes, after
// convex_manifold.c, and for the same reason: the file does not split along
// the phase boundary. The proxy half below converts with essentially no
// fixed-point content -- the AABBs pass straight through to the tree, and the
// rest is bit packing, bitsets and a hash set, all of which landed in Phase 1.
//
// The pair-finding half (b3PairQueryCallback, b3FindPairsTask,
// b3UpdateTreesTask, b3UpdateBroadPhasePairs) waited for Phase 3B, because it
// needs b3World, b3Shape, b3Body, b3CreateContact and b3ShouldBodiesCollide --
// none of which existed before 3A -- and could not be meaningfully tested
// without them. It is equally free of fixed-point arithmetic: the one
// geometric operation is the tree query against a fat AABB, which Phase 2
// converted and verified.
//
// The Phase 2A plan had this file "sitting entirely on the hash set, bitsets
// and tree already converted". That was true of the proxy half, and is true of
// this one for a different reason.

#include "broad_phase.h"

#include "aabb.h"
#include "body.h"
#include "contact.h"
#include "core.h"
#include "physics_world.h"
#include "shape.h"

#include <string.h>

void b3CreateBroadPhase( b3BroadPhase* bp, const b3Capacity* capacity )
{
	_Static_assert( b3_bodyTypeCount == 3, "must be three body types" );

	bp->movedProxies[b3_staticBody] = b3CreateBitSet( b3MaxInt( 16, capacity->staticShapeCount ) );
	bp->movedProxies[b3_kinematicBody] = b3CreateBitSet( 16 );
	bp->movedProxies[b3_dynamicBody] = b3CreateBitSet( b3MaxInt( 16, capacity->dynamicShapeCount ) );

	b3Array_Reserve( bp->moveArray, capacity->dynamicShapeCount );

	bp->pairSet = b3CreateSet( 2 * capacity->contactCount );

	int staticCapacity = b3MaxInt( 16, capacity->staticShapeCount );
	bp->trees[b3_staticBody] = b3DynamicTree_Create( staticCapacity );

	int kinematicCapacity = 16;
	bp->trees[b3_kinematicBody] = b3DynamicTree_Create( kinematicCapacity );

	int dynamicCapacity = b3MaxInt( 16, capacity->dynamicShapeCount );
	bp->trees[b3_dynamicBody] = b3DynamicTree_Create( dynamicCapacity );
}

void b3DestroyBroadPhase( b3BroadPhase* bp )
{
	for ( int i = 0; i < b3_bodyTypeCount; ++i )
	{
		b3DynamicTree_Destroy( bp->trees + i );
	}

	for ( int i = 0; i < b3_bodyTypeCount; ++i )
	{
		b3DestroyBitSet( &bp->movedProxies[i] );
	}

	b3Array_Destroy( bp->moveArray );
	b3DestroySet( &bp->pairSet );

	memset( bp, 0, sizeof( b3BroadPhase ) );
}

static void b3UnBufferMove( b3BroadPhase* bp, int proxyKey )
{
	b3BodyType proxyType = B3_PROXY_TYPE( proxyKey );
	int proxyId = B3_PROXY_ID( proxyKey );
	b3BitSet* set = &bp->movedProxies[proxyType];

	if ( b3GetBit( set, proxyId ) )
	{
		b3ClearBit( set, proxyId );

		// Purge from move buffer. Linear search.
		int count = bp->moveArray.count;
		for ( int i = 0; i < count; ++i )
		{
			if ( bp->moveArray.data[i] == proxyKey )
			{
				b3Array_RemoveSwap( bp->moveArray, i );
				break;
			}
		}
	}
}

int b3BroadPhase_CreateProxy( b3BroadPhase* bp, b3BodyType proxyType, b3AABB aabb, uint64_t categoryBits, int shapeIndex,
							  bool forcePairCreation )
{
	B3_ASSERT( 0 <= proxyType && proxyType < b3_bodyTypeCount );

	int proxyId = b3DynamicTree_CreateProxy( bp->trees + proxyType, aabb, categoryBits, shapeIndex );

	if ( proxyId == B3_NULL_INDEX )
	{
		// The tree refused to grow. Propagate that rather than manufacturing a
		// key for a proxy that does not exist; upstream cannot reach this
		// because it allocates unconditionally.
		return B3_NULL_INDEX;
	}

	int proxyKey = B3_PROXY_KEY( proxyId, proxyType );

	if ( proxyType != b3_staticBody || forcePairCreation )
	{
		b3BufferMove( bp, proxyKey );
	}

	return proxyKey;
}

void b3BroadPhase_DestroyProxy( b3BroadPhase* bp, int proxyKey )
{
	b3UnBufferMove( bp, proxyKey );

	b3BodyType proxyType = B3_PROXY_TYPE( proxyKey );
	int proxyId = B3_PROXY_ID( proxyKey );

	B3_ASSERT( 0 <= proxyType && proxyType <= b3_bodyTypeCount );
	b3DynamicTree_DestroyProxy( bp->trees + proxyType, proxyId );
}

void b3BroadPhase_MoveProxy( b3BroadPhase* bp, int proxyKey, b3AABB aabb )
{
	b3BodyType proxyType = B3_PROXY_TYPE( proxyKey );
	int proxyId = B3_PROXY_ID( proxyKey );

	b3DynamicTree_MoveProxy( bp->trees + proxyType, proxyId, aabb );
	b3BufferMove( bp, proxyKey );
}

void b3BroadPhase_EnlargeProxy( b3BroadPhase* bp, int proxyKey, b3AABB aabb )
{
	B3_ASSERT( proxyKey != B3_NULL_INDEX );
	int typeIndex = B3_PROXY_TYPE( proxyKey );
	int proxyId = B3_PROXY_ID( proxyKey );

	B3_ASSERT( typeIndex != b3_staticBody );

	b3DynamicTree_EnlargeProxy( bp->trees + typeIndex, proxyId, aabb );
	b3BufferMove( bp, proxyKey );
}

bool b3BroadPhase_TestOverlap( const b3BroadPhase* bp, int proxyKeyA, int proxyKeyB )
{
	int typeIndexA = B3_PROXY_TYPE( proxyKeyA );
	int proxyIdA = B3_PROXY_ID( proxyKeyA );
	int typeIndexB = B3_PROXY_TYPE( proxyKeyB );
	int proxyIdB = B3_PROXY_ID( proxyKeyB );

	b3AABB aabbA = b3DynamicTree_GetAABB( bp->trees + typeIndexA, proxyIdA );
	b3AABB aabbB = b3DynamicTree_GetAABB( bp->trees + typeIndexB, proxyIdB );
	return b3AABB_Overlaps( aabbA, aabbB );
}

int b3BroadPhase_GetShapeIndex( b3BroadPhase* bp, int proxyKey )
{
	int typeIndex = B3_PROXY_TYPE( proxyKey );
	int proxyId = B3_PROXY_ID( proxyKey );

	return (int)b3DynamicTree_GetUserData( bp->trees + typeIndex, proxyId );
}

// =========================================================================
// Pair finding
// =========================================================================

/// One candidate pair, in a singly linked list per moved proxy.
///
/// Upstream carries a `heap` flag, because its overflow path could allocate a
/// pair outside the block. That path is commented out upstream and refused
/// here, so every pair comes from the flat block and the flag is gone.
typedef struct b3MovePair
{
	int shapeIndexA;
	int shapeIndexB;
	int childIndex;
	b3MovePair* next;
} b3MovePair;

typedef struct b3MoveResult
{
	b3MovePair* pairList;
} b3MoveResult;

/// Pair slots reserved per moved proxy. Upstream's literal 16, named so that
/// b3BroadPhaseStackDemand and the allocation cannot drift apart.
#define B3_MOVE_PAIRS_PER_PROXY 16

int b3BroadPhaseStackDemand( int moveCount )
{
	if ( moveCount <= 0 )
	{
		return 0;
	}

	// b3StackAlloc rounds each entry up to B3_ALIGNMENT, so account for that per
	// entry rather than on the total -- two entries, so up to two roundings.
	int bytes = moveCount * (int)sizeof( b3MoveResult );
	bytes = ( ( bytes - 1 ) | ( B3_ALIGNMENT - 1 ) ) + 1;

	int pairBytes = B3_MOVE_PAIRS_PER_PROXY * moveCount * (int)sizeof( b3MovePair );
	pairBytes = ( ( pairBytes - 1 ) | ( B3_ALIGNMENT - 1 ) ) + 1;

	return bytes + pairBytes;
}

typedef struct b3QueryPairContext
{
	b3World* world;
	b3MoveResult* moveResult;
	b3AABB aabb;
	b3BodyType queryTreeType;
	int queryProxyKey;
	int queryShapeIndex;

	// Upstream also carries compoundProxyId / compoundShapeIndex, which mark
	// whether the callback is running as an inner query into a compound
	// shape's own tree. B3_NEA_NO_COMPOUND, so the callback is always the
	// outer query, `userData` is always a shape index, and childIndex is
	// always zero.
} b3QueryPairContext;

// Called from b3DynamicTree_Query while gathering pairs.
static bool b3PairQueryCallback( int proxyId, uint64_t userData, void* context )
{
	b3QueryPairContext* queryContext = (b3QueryPairContext*)context;
	b3World* world = queryContext->world;

	// Outer query only: userData is a shape index. Upstream recurses into a
	// compound shape's tree here, and only then is userData a child index.
	int shapeIndex = (int)userData;
	int childIndex = 0;

	// A proxy cannot form a pair with itself.
	if ( shapeIndex == queryContext->queryShapeIndex )
	{
		return true;
	}

	b3BroadPhase* broadPhase = &world->broadPhase;

	int proxyKey = B3_PROXY_KEY( proxyId, queryContext->queryTreeType );
	int queryProxyKey = queryContext->queryProxyKey;

	B3_ASSERT( proxyKey != queryProxyKey );

	b3BodyType treeType = queryContext->queryTreeType;
	b3BodyType queryProxyType = B3_PROXY_TYPE( queryProxyKey );

	// De-duplication. Both proxies of a pair can have moved this step, in
	// which case the pair is found twice -- once from each side -- and only
	// one contact may be created. The rule is to keep the find made from the
	// higher proxy key, so exactly one of the two sides drops it.
	//
	// The moved bit has to be consulted rather than assumed: most of the time
	// movedProxies holds only dynamic and kinematic proxies, but a static
	// proxy lands in there too when a static shape is modified or created with
	// b3ShapeDef::invokeContactCreation.
	if ( queryProxyType == b3_dynamicBody )
	{
		if ( treeType == b3_dynamicBody && proxyKey < queryProxyKey )
		{
			if ( b3GetBit( &broadPhase->movedProxies[treeType], proxyId ) )
			{
				return true;
			}
		}
	}
	else
	{
		B3_ASSERT( treeType == b3_dynamicBody );
		if ( b3GetBit( &broadPhase->movedProxies[treeType], proxyId ) )
		{
			return true;
		}
	}

	b3ShapeKey pairKey = b3ShapePairKey( shapeIndex, queryContext->queryShapeIndex, childIndex );
	if ( b3ContainsKey( &broadPhase->pairSet, pairKey ) )
	{
		// A contact already exists for this pair.
		return true;
	}

	// Order the shapes so b3ShapePairKey agrees with what b3CreateContact will
	// store.
	int shapeIdA = shapeIndex;
	int shapeIdB = queryContext->queryShapeIndex;
	b3Shape* shapeA = b3Array_Get( world->shapes, shapeIdA );
	b3Shape* shapeB = b3Array_Get( world->shapes, shapeIdB );
	int bodyIdA = shapeA->bodyId;
	int bodyIdB = shapeB->bodyId;

	// Two shapes on the same body never collide.
	if ( bodyIdA == bodyIdB )
	{
		return true;
	}

	// Upstream skips sensors here; the port has none (Phase 7), so this is an
	// assert rather than a branch -- same treatment as 3A gave shape creation.
	B3_ASSERT( shapeA->sensorIndex == B3_NULL_INDEX && shapeB->sensorIndex == B3_NULL_INDEX );

	if ( b3ShouldShapesCollide( shapeA->filter, shapeB->filter ) == false )
	{
		return true;
	}

	// Does a joint between the bodies suppress collision?
	b3Body* bodyA = b3Array_Get( world->bodies, bodyIdA );
	b3Body* bodyB = b3Array_Get( world->bodies, bodyIdB );
	if ( b3ShouldBodiesCollide( world, bodyA, bodyB ) == false )
	{
		return true;
	}

	// Custom user filter.
	if ( ( shapeA->flags & b3_enableCustomFiltering ) || ( shapeB->flags & b3_enableCustomFiltering ) )
	{
		b3CustomFilterFcn* customFilterFcn = world->customFilterFcn;
		if ( customFilterFcn != NULL )
		{
			b3ShapeId idA = { shapeIdA + 1, world->worldId, shapeA->generation };
			b3ShapeId idB = { shapeIdB + 1, world->worldId, shapeB->generation };
			if ( customFilterFcn( idA, idB, world->customFilterContext ) == false )
			{
				return true;
			}
		}
	}

	int pairIndex = broadPhase->movePairIndex;
	broadPhase->movePairIndex = pairIndex + 1;

	if ( pairIndex >= broadPhase->movePairCapacity )
	{
		// Out of pair slots. Upstream has a commented-out heap fallback and
		// drops the pair; the port drops it too, but raises the refusal flag,
		// because a silently missing contact is a body falling through a floor
		// with nothing to point at. The capacity is 16 per moved proxy, so
		// reaching this means one proxy overlapped far more than 16 others.
		b3OutOfMemory();
		b3Log( "broad phase: movePairs exhausted at %d pairs, dropping pair (%d, %d)", broadPhase->movePairCapacity,
			   shapeIdA, shapeIdB );
		return true;
	}

	b3MovePair* pair = broadPhase->movePairs + pairIndex;
	pair->shapeIndexA = shapeIdA;
	pair->shapeIndexB = shapeIdB;
	pair->childIndex = childIndex;
	pair->next = queryContext->moveResult->pairList;
	queryContext->moveResult->pairList = pair;

	// Continue the query.
	return true;
}

// Upstream runs this over a sub-range per worker through b3ParallelFor. One
// worker here, so it is called once over the whole move array and the
// startIndex/endIndex parameters are kept only to make the diff readable.
static void b3FindPairsTask( int startIndex, int endIndex, b3World* world )
{
	b3BroadPhase* bp = &world->broadPhase;

	b3QueryPairContext queryContext = { 0 };
	queryContext.world = world;

	for ( int i = startIndex; i < endIndex; ++i )
	{
		queryContext.moveResult = bp->moveResults + i;
		queryContext.moveResult->pairList = NULL;

		int proxyKey = bp->moveArray.data[i];
		b3BodyType proxyType = B3_PROXY_TYPE( proxyKey );
		int proxyId = B3_PROXY_ID( proxyKey );

		queryContext.queryProxyKey = proxyKey;

		const b3DynamicTree* baseTree = bp->trees + proxyType;

		// Query with the *fat* AABB, not the tight one, so a pair that will
		// touch a few steps from now already has its contact and its warm
		// start by the time it does.
		b3AABB fatAABB = b3DynamicTree_GetAABB( baseTree, proxyId );
		queryContext.queryShapeIndex = (int)b3DynamicTree_GetUserData( baseTree, proxyId );
		queryContext.aabb = fatAABB;

		// B3_DEFAULT_MASK_BITS rather than the shape's own mask, so that
		// b3Filter::groupIndex keeps working -- the real filtering is
		// b3ShouldShapesCollide inside the callback.
		bool requireAllBits = false;

		// Only dynamic proxies collide with kinematic and static proxies.
		if ( proxyType == b3_dynamicBody )
		{
			queryContext.queryTreeType = b3_kinematicBody;
			b3DynamicTree_Query( bp->trees + b3_kinematicBody, fatAABB, B3_DEFAULT_MASK_BITS, requireAllBits,
								 b3PairQueryCallback, &queryContext );

			queryContext.queryTreeType = b3_staticBody;
			b3DynamicTree_Query( bp->trees + b3_staticBody, fatAABB, B3_DEFAULT_MASK_BITS, requireAllBits,
								 b3PairQueryCallback, &queryContext );
		}

		// Every proxy collides with dynamic proxies.
		queryContext.queryTreeType = b3_dynamicBody;
		b3DynamicTree_Query( bp->trees + b3_dynamicBody, fatAABB, B3_DEFAULT_MASK_BITS, requireAllBits, b3PairQueryCallback,
							 &queryContext );
	}
}

// Upstream enqueues this to run alongside the narrow phase, falling back to an
// inline call when the task pool is full. The port always takes the fallback.
static void b3UpdateTreesTask( b3World* world )
{
	b3DynamicTree_Rebuild( world->broadPhase.trees + b3_dynamicBody, false );
	b3DynamicTree_Rebuild( world->broadPhase.trees + b3_kinematicBody, false );
}

void b3UpdateBroadPhasePairs( b3World* world )
{
	b3BroadPhase* bp = &world->broadPhase;

	int moveCount = bp->moveArray.count;

	if ( moveCount == 0 )
	{
		return;
	}

	b3Stack* alloc = &world->stack;

	// Both of these are accounted for by b3BroadPhaseStackDemand below. Keep the
	// two in step: an allocation added here without adding it there turns into a
	// b3GrowStack call, which NEA_Phys3D's pool allocator rejects.
	bp->moveResults = (b3MoveResult*)b3StackAlloc( alloc, moveCount * (int)sizeof( b3MoveResult ), "move results" );
	bp->movePairCapacity = B3_MOVE_PAIRS_PER_PROXY * moveCount;
	bp->movePairs = (b3MovePair*)b3StackAlloc( alloc, bp->movePairCapacity * (int)sizeof( b3MovePair ), "move pairs" );
	bp->movePairIndex = 0;

	b3FindPairsTask( 0, moveCount, world );

	b3UpdateTreesTask( world );

	// Create the contacts serially, walking moveArray in order. Discovery
	// order depends on the shape of the tree; this does not.
	for ( int i = 0; i < moveCount; ++i )
	{
		b3MoveResult* result = bp->moveResults + i;
		b3MovePair* pair = result->pairList;
		while ( pair != NULL )
		{
			b3Shape* shapeA = b3Array_Get( world->shapes, pair->shapeIndexA );
			b3Shape* shapeB = b3Array_Get( world->shapes, pair->shapeIndexB );

			b3CreateContact( world, shapeA, shapeB, pair->childIndex );

			pair = pair->next;
		}
	}

	// Reset the move buffer: clear only the bits set this step.
	// Invariant: a bit is set in movedProxies[type] exactly when its proxyKey
	// is in moveArray.
	for ( int i = 0; i < bp->moveArray.count; ++i )
	{
		int proxyKey = bp->moveArray.data[i];
		b3ClearBit( &bp->movedProxies[B3_PROXY_TYPE( proxyKey )], B3_PROXY_ID( proxyKey ) );
	}
	b3Array_Clear( bp->moveArray );

	b3StackFree( alloc, bp->movePairs );
	bp->movePairs = NULL;
	b3StackFree( alloc, bp->moveResults );
	bp->moveResults = NULL;

	b3ValidateSolverSets( world );
}

void b3ValidateBroadPhase( const b3BroadPhase* bp )
{
	b3DynamicTree_Validate( bp->trees + b3_dynamicBody );
	b3DynamicTree_Validate( bp->trees + b3_kinematicBody );
}

void b3ValidateNoEnlarged( const b3BroadPhase* bp )
{
#if B3_ENABLE_VALIDATION
	for ( int j = 0; j < b3_bodyTypeCount; ++j )
	{
		const b3DynamicTree* tree = bp->trees + j;
		b3DynamicTree_ValidateNoEnlarged( tree );
	}
#else
	B3_UNUSED( bp );
#endif
}
