// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Erin Catto        (original Box3D)
// Copyright (c) 2026 Warioware64       (Nitro Engine Advanced fixed-point port)
//
// This file is part of Nitro Engine Advanced

// The broad-phase AABB tree.
//
// Far less of this is fixed-point than its size suggests. Node allocation, the
// free list, parent and child indices, heights, the rotation bookkeeping and
// the rebuild's stack machine are all integer index manipulation and convert
// untouched. What does need care is the cost model and the ray tests:
//
//   * Every SAH cost is a surface area, and a Q12 surface area overflows int32
//     almost immediately -- b3Perimeter already returns int64 for that reason
//     (see aabb.h). All of its consumers follow it wide: the descent costs in
//     b3FindBestSibling, the rotation costs in b3RotateNodes, the inherited
//     cost accumulator, and the area ratio. FLT_MAX becomes INT64_MAX.
//
//   * b3DistanceToNodeSqr is a squared distance, so it is wide too, and that
//     propagates out through the whole QueryClosest interface including the
//     user callback signature.
//
//   * The ray and box casts reject nodes with b3TestBoundsRayOverlap, which
//     upstream keeps in simd.h. The port's scalar form lives in aabb.h.
//
// Two pieces of upstream are deliberately absent:
//
//   * b3DynamicTree_Save / b3DynamicTree_Load. They are built on FILE* and
//     fopen, and take a float scale factor. There is no stdio on the target.
//
//   * b3PartitionSAH and its bin machinery, upstream's B3_TREE_HEURISTIC == 1
//     branch. Upstream sets B3_TREE_HEURISTIC to 0, so that branch does not
//     compile -- and it has rotted: it is a Box2D leftover that never gained a
//     third axis. It selects the longest axis from `d.x > d.y` alone, bins on
//     `float cArray[2]`, and initializes bin bounds with
//     `(b3Vec3){ FLT_MAX, FLT_MAX }`, two initializers for a three-vector.
//     Porting it would have meant either transliterating a partition that
//     ignores Z or quietly rewriting it, neither of which is worth doing for
//     code that is not built. b3PartitionMid, the branch that is live, handles
//     all three axes correctly and is ported in full.

#include "aabb.h"
#include "core.h"

#include "box3d/collision.h"
#include "box3d/constants.h"
#include "box3d/math_fixed.h"

#include <string.h>

#define B3_TREE_STACK_SIZE 1024

// The aabb is deliberately not listed: it must be zero, and a static
// initializer zero-fills whatever it omits. Spelling it out is what upstream
// does, but a literal zero vector cannot be written portably across the port's
// three build modes -- b3f is a struct under B3_FIXED_STRICT and a bare
// int32_t on device, so any brace nesting is wrong in one of them.
static b3TreeNode b3_defaultTreeNode = {
	.categoryBits = B3_DEFAULT_CATEGORY_BITS,
	.children =
		{
			.child1 = B3_NULL_INDEX,
			.child2 = B3_NULL_INDEX,
		},
	.parent = B3_NULL_INDEX,
	.height = 0,
	.flags = b3_allocatedNode,
};

static inline bool b3IsLeaf( const b3TreeNode* node )
{
	return node->flags & b3_leafNode;
}

static inline bool b3IsAllocated( const b3TreeNode* node )
{
	return node->flags & b3_allocatedNode;
}

static inline uint16_t b3MaxUInt16( uint16_t a, uint16_t b )
{
	return a > b ? a : b;
}

static inline int64_t b3MinWide( int64_t a, int64_t b )
{
	return a < b ? a : b;
}

b3DynamicTree b3DynamicTree_Create( int proxyCapacity )
{
	int capacity = b3MaxInt( proxyCapacity, 16 );

	b3DynamicTree tree;

	memset( &tree, 0, sizeof( b3DynamicTree ) );

	tree.root = B3_NULL_INDEX;

	// Maximum node count for a full binary tree is 2 * leafCount - 1.
	tree.nodeCapacity = 2 * capacity - 1;
	tree.nodeCount = 0;

	tree.nodes = (b3TreeNode*)b3Alloc( tree.nodeCapacity * sizeof( b3TreeNode ) );

	if ( tree.nodes == NULL )
	{
		// The refusal policy from Phase 1: b3Alloc has already logged and
		// raised the out-of-memory flag. Hand back a tree that refuses work
		// rather than one that will dereference null on first insert.
		tree.nodeCapacity = 0;
		tree.freeList = B3_NULL_INDEX;
		return tree;
	}

	memset( tree.nodes, 0, tree.nodeCapacity * sizeof( b3TreeNode ) );

	// Build a linked list for the free list.
	for ( int i = 0; i < tree.nodeCapacity - 1; ++i )
	{
		tree.nodes[i].next = i + 1;
	}

	tree.nodes[tree.nodeCapacity - 1].next = B3_NULL_INDEX;
	tree.freeList = 0;

	tree.proxyCount = 0;

	// The rebuild scratch, taken here rather than on the first rebuild.
	//
	// Upstream leaves these NULL and lets b3DynamicTree_Rebuild allocate them
	// when it first runs -- which is inside b3World_Step, by way of
	// b3UpdateTreesTask. The port cannot: NEA_Phys3D seals its pool allocator
	// once the world is running, so a first rebuild that allocates is an
	// assert on a real console rather than one slow frame.
	//
	// The bound is the proxy capacity, which is what the tree was sized from,
	// and the rebuild's own growth rule (proxyCount + proxyCount / 2) is
	// reproduced so that reaching it later is a no-op rather than a realloc.
	tree.rebuildCapacity = capacity + capacity / 2;
	tree.leafIndices = (int*)b3Alloc( tree.rebuildCapacity * sizeof( int ) );
	tree.leafBoxes = (b3AABB*)b3Alloc( tree.rebuildCapacity * sizeof( b3AABB ) );
	tree.leafCenters = (b3Vec3*)b3Alloc( tree.rebuildCapacity * sizeof( b3Vec3 ) );

	if ( tree.leafIndices == NULL || tree.leafBoxes == NULL || tree.leafCenters == NULL )
	{
		// Same refusal policy as the node array above, except that the tree is
		// still fully usable without rebuild scratch -- b3DynamicTree_Rebuild
		// simply allocates it later, or refuses in turn. Drop back to the
		// upstream lazy state rather than failing the whole tree.
		b3Free( tree.leafIndices, tree.rebuildCapacity * sizeof( int ) );
		b3Free( tree.leafBoxes, tree.rebuildCapacity * sizeof( b3AABB ) );
		b3Free( tree.leafCenters, tree.rebuildCapacity * sizeof( b3Vec3 ) );

		tree.leafIndices = NULL;
		tree.leafBoxes = NULL;
		tree.leafCenters = NULL;
		tree.rebuildCapacity = 0;
	}

	return tree;
}

void b3DynamicTree_Destroy( b3DynamicTree* tree )
{
	b3Free( tree->nodes, tree->nodeCapacity * sizeof( b3TreeNode ) );
	b3Free( tree->leafIndices, tree->rebuildCapacity * sizeof( int ) );
	b3Free( tree->leafBoxes, tree->rebuildCapacity * sizeof( b3AABB ) );
	b3Free( tree->leafCenters, tree->rebuildCapacity * sizeof( b3Vec3 ) );

	memset( tree, 0, sizeof( b3DynamicTree ) );
}

// Allocate a node from the pool. Grow the pool if necessary.
static int b3AllocateNode( b3DynamicTree* tree )
{
	// Expand the node pool as needed.
	if ( tree->freeList == B3_NULL_INDEX )
	{
		B3_ASSERT( tree->nodeCount == tree->nodeCapacity );

		// The free list is empty. Rebuild a bigger pool.
		b3TreeNode* oldNodes = tree->nodes;
		int oldCapacity = tree->nodeCapacity;
		int newCapacity = oldCapacity + ( oldCapacity >> 1 );

		if ( newCapacity <= oldCapacity )
		{
			// A tree whose creation was refused has nodeCapacity 0, and 0 does
			// not grow: 0 + (0 >> 1) is still 0. Without this guard b3Alloc(0)
			// returns a non-null pointer on most allocators, the refusal check
			// below passes, and the free-list terminator writes to nodes[-1].
			//
			// Upstream cannot reach this because it allocates unconditionally
			// in b3DynamicTree_Create and so never holds an empty pool.
			return B3_NULL_INDEX;
		}

		b3TreeNode* newNodes = (b3TreeNode*)b3Alloc( newCapacity * sizeof( b3TreeNode ) );

		if ( newNodes == NULL )
		{
			// Refused. Keep the existing pool intact and report failure; the
			// caller turns this into a proxy that was never created rather
			// than a corrupt tree.
			return B3_NULL_INDEX;
		}

		tree->nodeCapacity = newCapacity;
		tree->nodes = newNodes;

		B3_ASSERT( oldNodes != NULL );
		memcpy( tree->nodes, oldNodes, tree->nodeCount * sizeof( b3TreeNode ) );
		memset( tree->nodes + tree->nodeCount, 0, ( tree->nodeCapacity - tree->nodeCount ) * sizeof( b3TreeNode ) );
		b3Free( oldNodes, oldCapacity * sizeof( b3TreeNode ) );

		// Build a linked list for the free list. The parent index doubles as
		// the "next" index while a node is free.
		for ( int i = tree->nodeCount; i < tree->nodeCapacity - 1; ++i )
		{
			tree->nodes[i].next = i + 1;
		}

		tree->nodes[tree->nodeCapacity - 1].next = B3_NULL_INDEX;
		tree->freeList = tree->nodeCount;
	}

	// Peel a node off the free list.
	int nodeIndex = tree->freeList;
	b3TreeNode* node = tree->nodes + nodeIndex;
	tree->freeList = node->next;
	*node = b3_defaultTreeNode;
	++tree->nodeCount;
	return nodeIndex;
}

// Return a node to the pool.
static void b3FreeNode( b3DynamicTree* tree, int nodeId )
{
	B3_ASSERT( 0 <= nodeId && nodeId < tree->nodeCapacity );
	B3_ASSERT( 0 < tree->nodeCount );
	tree->nodes[nodeId].next = tree->freeList;
	tree->nodes[nodeId].flags = 0;
	tree->freeList = nodeId;
	--tree->nodeCount;
}

// Greedy algorithm for sibling selection using the SAH
// We have three nodes A-(B,C) and want to add a leaf D, there are three choices.
// 1: make a new parent for A and D : E-(A-(B,C), D)
// 2: associate D with B
//   a: B is a leaf : A-(E-(B,D), C)
//   b: B is an internal node: A-(B{D},C)
// 3: associate D with C
//   a: C is a leaf : A-(B, E-(C,D))
//   b: C is an internal node: A-(B, C{D})
// All of these have a clear cost except when B or C is an internal node. Hence we need to be greedy.

// The cost for cases 1, 2a, and 3a can be computed using the sibling cost formula.
// cost of sibling H = area(union(H, D)) + increased area of ancestors

// Suppose B (or C) is an internal node, then the lowest cost would be one of two cases:
// case1: D becomes a sibling of B
// case2: D becomes a descendant of B along with a new internal node of area(D).
//
// Every cost here is a surface area and stays int64. See the file header.
static int b3FindBestSibling( const b3DynamicTree* tree, b3AABB boxD )
{
	b3Vec3 centerD = b3AABB_Center( boxD );
	int64_t areaD = b3Perimeter( boxD );

	const b3TreeNode* nodes = tree->nodes;
	int rootIndex = tree->root;

	b3AABB rootBox = nodes[rootIndex].aabb;

	// Area of current node
	int64_t areaBase = b3Perimeter( rootBox );

	// Area of inflated node
	int64_t directCost = b3Perimeter( b3AABB_Union( rootBox, boxD ) );
	int64_t inheritedCost = 0;

	int bestSibling = rootIndex;
	int64_t bestCost = directCost;

	// Descend the tree from root, following a single greedy path.
	int index = rootIndex;
	while ( b3IsLeaf( nodes + index ) == false )
	{
		int child1 = nodes[index].children.child1;
		int child2 = nodes[index].children.child2;

		// Cost of creating a new parent for this node and the new leaf
		int64_t cost = directCost + inheritedCost;

		// Sometimes there are multiple identical costs within tolerance.
		// This breaks the ties using the centroid distance.
		if ( cost < bestCost )
		{
			bestSibling = index;
			bestCost = cost;
		}

		// Inheritance cost seen by children
		inheritedCost += directCost - areaBase;

		bool leaf1 = b3IsLeaf( nodes + child1 );
		bool leaf2 = b3IsLeaf( nodes + child2 );

		// Cost of descending into child 1
		int64_t lowerCost1 = INT64_MAX;
		b3AABB box1 = nodes[child1].aabb;
		int64_t directCost1 = b3Perimeter( b3AABB_Union( box1, boxD ) );
		int64_t area1 = 0;
		if ( leaf1 )
		{
			// Child 1 is a leaf
			// Cost of creating new node and increasing area of node P
			int64_t cost1 = directCost1 + inheritedCost;

			// Need this here due to while condition above
			if ( cost1 < bestCost )
			{
				bestSibling = child1;
				bestCost = cost1;
			}
		}
		else
		{
			// Child 1 is an internal node
			area1 = b3Perimeter( box1 );

			// Lower bound cost of inserting under child 1.
			lowerCost1 = inheritedCost + directCost1 + b3MinWide( areaD - area1, 0 );
		}

		// Cost of descending into child 2
		int64_t lowerCost2 = INT64_MAX;
		b3AABB box2 = nodes[child2].aabb;
		int64_t directCost2 = b3Perimeter( b3AABB_Union( box2, boxD ) );
		int64_t area2 = 0;
		if ( leaf2 )
		{
			// Child 2 is a leaf
			// Cost of creating new node and increasing area of node P
			int64_t cost2 = directCost2 + inheritedCost;

			// Need this here due to while condition above
			if ( cost2 < bestCost )
			{
				bestSibling = child2;
				bestCost = cost2;
			}
		}
		else
		{
			// Child 2 is an internal node
			area2 = b3Perimeter( box2 );

			// Lower bound cost of inserting under child 2. This is not the cost
			// of child 2, it is the best we can hope for under child 2.
			lowerCost2 = inheritedCost + directCost2 + b3MinWide( areaD - area2, 0 );
		}

		if ( leaf1 && leaf2 )
		{
			break;
		}

		// Can the cost possibly be decreased?
		if ( bestCost <= lowerCost1 && bestCost <= lowerCost2 )
		{
			break;
		}

		if ( lowerCost1 == lowerCost2 && leaf1 == false )
		{
			B3_ASSERT( lowerCost1 < INT64_MAX );
			B3_ASSERT( lowerCost2 < INT64_MAX );

			// No clear choice based on lower bound surface area. This can happen when both
			// children fully contain D. Fall back to node distance.
			//
			// A squared length rather than an area, which upstream mixes into
			// the same variable too. It is only ever compared against the
			// other tie-break value, and both are Q24, so the substitution is
			// scale-consistent as well as faithful.
			b3Vec3 d1 = b3Sub( b3AABB_Center( box1 ), centerD );
			b3Vec3 d2 = b3Sub( b3AABB_Center( box2 ), centerD );
			lowerCost1 = b3LengthSquaredWide( d1 );
			lowerCost2 = b3LengthSquaredWide( d2 );
		}

		// Descend
		if ( lowerCost1 < lowerCost2 && leaf1 == false )
		{
			index = child1;
			areaBase = area1;
			directCost = directCost1;
		}
		else
		{
			index = child2;
			areaBase = area2;
			directCost = directCost2;
		}

		B3_ASSERT( b3IsLeaf( nodes + index ) == false );
	}

	return bestSibling;
}

enum b3RotateType
{
	b3_rotateNone,
	b3_rotateBF,
	b3_rotateBG,
	b3_rotateCD,
	b3_rotateCE
};

// Perform a left or right rotation if node A is imbalanced.
static void b3RotateNodes( b3DynamicTree* tree, int iA )
{
	B3_ASSERT( iA != B3_NULL_INDEX );

	b3TreeNode* nodes = tree->nodes;

	b3TreeNode* A = nodes + iA;
	if ( b3IsLeaf( A ) == true )
	{
		return;
	}

	int iB = A->children.child1;
	int iC = A->children.child2;
	B3_ASSERT( 0 <= iB && iB < tree->nodeCapacity );
	B3_ASSERT( 0 <= iC && iC < tree->nodeCapacity );

	b3TreeNode* B = nodes + iB;
	b3TreeNode* C = nodes + iC;

	bool isLeafB = b3IsLeaf( B );
	bool isLeafC = b3IsLeaf( C );

	if ( isLeafB == true && isLeafC == false )
	{
		int iF = C->children.child1;
		int iG = C->children.child2;
		b3TreeNode* F = nodes + iF;
		b3TreeNode* G = nodes + iG;
		B3_ASSERT( 0 <= iF && iF < tree->nodeCapacity );
		B3_ASSERT( 0 <= iG && iG < tree->nodeCapacity );

		// Base cost
		int64_t costBase = b3Perimeter( C->aabb );

		// Cost of swapping B and F
		b3AABB aabbBG = b3AABB_Union( B->aabb, G->aabb );
		int64_t costBF = b3Perimeter( aabbBG );

		// Cost of swapping B and G
		b3AABB aabbBF = b3AABB_Union( B->aabb, F->aabb );
		int64_t costBG = b3Perimeter( aabbBF );

		if ( costBase < costBF && costBase < costBG )
		{
			// Rotation does not improve cost
			return;
		}

		if ( costBF < costBG )
		{
			// Swap B and F
			A->children.child1 = iF;
			C->children.child1 = iB;

			B->parent = iC;
			F->parent = iA;

			C->aabb = aabbBG;

			C->height = 1 + b3MaxUInt16( B->height, G->height );
			A->height = 1 + b3MaxUInt16( C->height, F->height );
			C->categoryBits = B->categoryBits | G->categoryBits;
			A->categoryBits = C->categoryBits | F->categoryBits;
			C->flags |= ( B->flags | G->flags ) & b3_enlargedNode;
			A->flags |= ( C->flags | F->flags ) & b3_enlargedNode;
		}
		else
		{
			// Swap B and G
			A->children.child1 = iG;
			C->children.child2 = iB;

			B->parent = iC;
			G->parent = iA;

			C->aabb = aabbBF;

			C->height = 1 + b3MaxUInt16( B->height, F->height );
			A->height = 1 + b3MaxUInt16( C->height, G->height );
			C->categoryBits = B->categoryBits | F->categoryBits;
			A->categoryBits = C->categoryBits | G->categoryBits;
			C->flags |= ( B->flags | F->flags ) & b3_enlargedNode;
			A->flags |= ( C->flags | G->flags ) & b3_enlargedNode;
		}
	}
	else if ( isLeafC == true && isLeafB == false )
	{
		// C is a leaf and B is internal

		int iD = B->children.child1;
		int iE = B->children.child2;
		b3TreeNode* D = nodes + iD;
		b3TreeNode* E = nodes + iE;
		B3_ASSERT( 0 <= iD && iD < tree->nodeCapacity );
		B3_ASSERT( 0 <= iE && iE < tree->nodeCapacity );

		// Base cost
		int64_t costBase = b3Perimeter( B->aabb );

		// Cost of swapping C and D
		b3AABB aabbCE = b3AABB_Union( C->aabb, E->aabb );
		int64_t costCD = b3Perimeter( aabbCE );

		// Cost of swapping C and E
		b3AABB aabbCD = b3AABB_Union( C->aabb, D->aabb );
		int64_t costCE = b3Perimeter( aabbCD );

		if ( costBase < costCD && costBase < costCE )
		{
			// Rotation does not improve cost
			return;
		}

		if ( costCD < costCE )
		{
			// Swap C and D
			A->children.child2 = iD;
			B->children.child1 = iC;

			C->parent = iB;
			D->parent = iA;

			B->aabb = aabbCE;

			B->height = 1 + b3MaxUInt16( C->height, E->height );
			A->height = 1 + b3MaxUInt16( B->height, D->height );
			B->categoryBits = C->categoryBits | E->categoryBits;
			A->categoryBits = B->categoryBits | D->categoryBits;
			B->flags |= ( C->flags | E->flags ) & b3_enlargedNode;
			A->flags |= ( B->flags | D->flags ) & b3_enlargedNode;
		}
		else
		{
			// Swap C and E
			A->children.child2 = iE;
			B->children.child2 = iC;

			C->parent = iB;
			E->parent = iA;

			B->aabb = aabbCD;

			B->height = 1 + b3MaxUInt16( C->height, D->height );
			A->height = 1 + b3MaxUInt16( B->height, E->height );
			B->categoryBits = C->categoryBits | D->categoryBits;
			A->categoryBits = B->categoryBits | E->categoryBits;
			B->flags |= ( C->flags | D->flags ) & b3_enlargedNode;
			A->flags |= ( B->flags | E->flags ) & b3_enlargedNode;
		}
	}
	else if ( isLeafB == false && isLeafC == false )
	{
		// All grand children exist so there are many options for rotation
		int iD = B->children.child1;
		int iE = B->children.child2;
		int iF = C->children.child1;
		int iG = C->children.child2;

		B3_ASSERT( 0 <= iD && iD < tree->nodeCapacity );
		B3_ASSERT( 0 <= iE && iE < tree->nodeCapacity );
		B3_ASSERT( 0 <= iF && iF < tree->nodeCapacity );
		B3_ASSERT( 0 <= iG && iG < tree->nodeCapacity );

		b3TreeNode* D = nodes + iD;
		b3TreeNode* E = nodes + iE;
		b3TreeNode* F = nodes + iF;
		b3TreeNode* G = nodes + iG;

		// Base cost
		int64_t areaB = b3Perimeter( B->aabb );
		int64_t areaC = b3Perimeter( C->aabb );
		int64_t costBase = areaB + areaC;
		enum b3RotateType bestRotation = b3_rotateNone;
		int64_t bestCost = costBase;

		// Cost of swapping B and F
		b3AABB aabbBG = b3AABB_Union( B->aabb, G->aabb );
		int64_t costBF = areaB + b3Perimeter( aabbBG );
		if ( costBF < bestCost )
		{
			bestRotation = b3_rotateBF;
			bestCost = costBF;
		}

		// Cost of swapping B and G
		b3AABB aabbBF = b3AABB_Union( B->aabb, F->aabb );
		int64_t costBG = areaB + b3Perimeter( aabbBF );
		if ( costBG < bestCost )
		{
			bestRotation = b3_rotateBG;
			bestCost = costBG;
		}

		// Cost of swapping C and D
		b3AABB aabbCE = b3AABB_Union( C->aabb, E->aabb );
		int64_t costCD = areaC + b3Perimeter( aabbCE );
		if ( costCD < bestCost )
		{
			bestRotation = b3_rotateCD;
			bestCost = costCD;
		}

		// Cost of swapping C and E
		b3AABB aabbCD = b3AABB_Union( C->aabb, D->aabb );
		int64_t costCE = areaC + b3Perimeter( aabbCD );
		if ( costCE < bestCost )
		{
			bestRotation = b3_rotateCE;
			// bestCost = costCE;
		}

		switch ( bestRotation )
		{
			case b3_rotateNone:
				break;

			case b3_rotateBF:
				A->children.child1 = iF;
				C->children.child1 = iB;

				B->parent = iC;
				F->parent = iA;

				C->aabb = aabbBG;

				C->height = 1 + b3MaxUInt16( B->height, G->height );
				A->height = 1 + b3MaxUInt16( C->height, F->height );
				C->categoryBits = B->categoryBits | G->categoryBits;
				A->categoryBits = C->categoryBits | F->categoryBits;
				C->flags |= ( B->flags | G->flags ) & b3_enlargedNode;
				A->flags |= ( C->flags | F->flags ) & b3_enlargedNode;
				break;

			case b3_rotateBG:
				A->children.child1 = iG;
				C->children.child2 = iB;

				B->parent = iC;
				G->parent = iA;

				C->aabb = aabbBF;

				C->height = 1 + b3MaxUInt16( B->height, F->height );
				A->height = 1 + b3MaxUInt16( C->height, G->height );
				C->categoryBits = B->categoryBits | F->categoryBits;
				A->categoryBits = C->categoryBits | G->categoryBits;
				C->flags |= ( B->flags | F->flags ) & b3_enlargedNode;
				A->flags |= ( C->flags | G->flags ) & b3_enlargedNode;
				break;

			case b3_rotateCD:
				A->children.child2 = iD;
				B->children.child1 = iC;

				C->parent = iB;
				D->parent = iA;

				B->aabb = aabbCE;

				B->height = 1 + b3MaxUInt16( C->height, E->height );
				A->height = 1 + b3MaxUInt16( B->height, D->height );
				B->categoryBits = C->categoryBits | E->categoryBits;
				A->categoryBits = B->categoryBits | D->categoryBits;
				B->flags |= ( C->flags | E->flags ) & b3_enlargedNode;
				A->flags |= ( B->flags | D->flags ) & b3_enlargedNode;
				break;

			case b3_rotateCE:
				A->children.child2 = iE;
				B->children.child2 = iC;

				C->parent = iB;
				E->parent = iA;

				B->aabb = aabbCD;

				B->height = 1 + b3MaxUInt16( C->height, D->height );
				A->height = 1 + b3MaxUInt16( B->height, E->height );
				B->categoryBits = C->categoryBits | D->categoryBits;
				A->categoryBits = B->categoryBits | E->categoryBits;
				B->flags |= ( C->flags | D->flags ) & b3_enlargedNode;
				A->flags |= ( B->flags | E->flags ) & b3_enlargedNode;
				break;

			default:
				B3_ASSERT( false );
				break;
		}
	}
}

// It would be nicer if the root had zero height but maintaining this would drastically increase
// insertion cost because whole sub-trees would need the height to be updated.
static void b3InsertLeaf( b3DynamicTree* tree, int leaf, bool shouldRotate )
{
	if ( tree->root == B3_NULL_INDEX )
	{
		tree->root = leaf;
		tree->nodes[tree->root].parent = B3_NULL_INDEX;
		return;
	}

	// Stage 1: find the best sibling for this node
	b3AABB leafAABB = tree->nodes[leaf].aabb;
	int sibling = b3FindBestSibling( tree, leafAABB );

	// Stage 2: create a new parent for the leaf and sibling
	int oldParent = tree->nodes[sibling].parent;
	int newParent = b3AllocateNode( tree );

	if ( newParent == B3_NULL_INDEX )
	{
		// Refused allocation. The leaf stays out of the tree; the caller has
		// already been told the proxy could not be created.
		return;
	}

	// warning: node pointer can change after allocation
	b3TreeNode* nodes = tree->nodes;
	nodes[newParent].parent = oldParent;
	nodes[newParent].userData = UINT64_MAX;
	nodes[newParent].aabb = b3AABB_Union( leafAABB, nodes[sibling].aabb );
	nodes[newParent].categoryBits = nodes[leaf].categoryBits | nodes[sibling].categoryBits;
	nodes[newParent].height = nodes[sibling].height + 1;

	if ( oldParent != B3_NULL_INDEX )
	{
		// The sibling was not the root.
		if ( nodes[oldParent].children.child1 == sibling )
		{
			nodes[oldParent].children.child1 = newParent;
		}
		else
		{
			nodes[oldParent].children.child2 = newParent;
		}

		nodes[newParent].children.child1 = sibling;
		nodes[newParent].children.child2 = leaf;
		nodes[sibling].parent = newParent;
		nodes[leaf].parent = newParent;
	}
	else
	{
		// The sibling was the root.
		nodes[newParent].children.child1 = sibling;
		nodes[newParent].children.child2 = leaf;
		nodes[sibling].parent = newParent;
		nodes[leaf].parent = newParent;
		tree->root = newParent;
	}

	// Stage 3: walk back up the tree fixing heights and AABBs
	int index = nodes[leaf].parent;
	while ( index != B3_NULL_INDEX )
	{
		int child1 = nodes[index].children.child1;
		int child2 = nodes[index].children.child2;

		B3_ASSERT( child1 != B3_NULL_INDEX );
		B3_ASSERT( child2 != B3_NULL_INDEX );

		nodes[index].aabb = b3AABB_Union( nodes[child1].aabb, nodes[child2].aabb );
		nodes[index].categoryBits = nodes[child1].categoryBits | nodes[child2].categoryBits;
		nodes[index].height = 1 + b3MaxUInt16( nodes[child1].height, nodes[child2].height );
		nodes[index].flags |= ( nodes[child1].flags | nodes[child2].flags ) & b3_enlargedNode;

		if ( shouldRotate )
		{
			b3RotateNodes( tree, index );
		}

		index = nodes[index].parent;
	}
}

static void b3RemoveLeaf( b3DynamicTree* tree, int leaf )
{
	if ( leaf == tree->root )
	{
		tree->root = B3_NULL_INDEX;
		return;
	}

	b3TreeNode* nodes = tree->nodes;

	int parent = nodes[leaf].parent;
	int grandParent = nodes[parent].parent;
	int sibling;
	if ( nodes[parent].children.child1 == leaf )
	{
		sibling = nodes[parent].children.child2;
	}
	else
	{
		sibling = nodes[parent].children.child1;
	}

	if ( grandParent != B3_NULL_INDEX )
	{
		// Destroy parent and connect sibling to grandParent.
		if ( nodes[grandParent].children.child1 == parent )
		{
			nodes[grandParent].children.child1 = sibling;
		}
		else
		{
			nodes[grandParent].children.child2 = sibling;
		}
		nodes[sibling].parent = grandParent;
		b3FreeNode( tree, parent );

		// Adjust ancestor bounds.
		int index = grandParent;
		while ( index != B3_NULL_INDEX )
		{
			b3TreeNode* node = nodes + index;
			b3TreeNode* child1 = nodes + node->children.child1;
			b3TreeNode* child2 = nodes + node->children.child2;

			node->aabb = b3AABB_Union( child1->aabb, child2->aabb );
			node->categoryBits = child1->categoryBits | child2->categoryBits;
			node->height = 1 + b3MaxUInt16( child1->height, child2->height );

			index = node->parent;
		}
	}
	else
	{
		tree->root = sibling;
		tree->nodes[sibling].parent = B3_NULL_INDEX;
		b3FreeNode( tree, parent );
	}
}

// Create a proxy in the tree as a leaf node. We return the index of the node instead of a pointer so that we can grow
// the node pool.
int b3DynamicTree_CreateProxy( b3DynamicTree* tree, b3AABB aabb, uint64_t categoryBits, uint64_t userData )
{
	B3_ASSERT( b3IsValidAABB( aabb ) );

	int proxyId = b3AllocateNode( tree );

	if ( proxyId == B3_NULL_INDEX )
	{
		// Refused. B3_NULL_INDEX propagates to the caller as "no proxy".
		return B3_NULL_INDEX;
	}

	b3TreeNode* node = tree->nodes + proxyId;

	node->aabb = aabb;
	node->userData = userData;
	node->categoryBits = categoryBits;
	node->height = 0;
	node->flags = b3_allocatedNode | b3_leafNode;

	bool shouldRotate = true;
	b3InsertLeaf( tree, proxyId, shouldRotate );

	tree->proxyCount += 1;

	return proxyId;
}

void b3DynamicTree_DestroyProxy( b3DynamicTree* tree, int proxyId )
{
	B3_ASSERT( 0 <= proxyId && proxyId < tree->nodeCapacity );
	B3_ASSERT( b3IsLeaf( tree->nodes + proxyId ) );

	b3RemoveLeaf( tree, proxyId );
	b3FreeNode( tree, proxyId );

	B3_ASSERT( tree->proxyCount > 0 );
	tree->proxyCount -= 1;
}

int b3DynamicTree_GetProxyCount( const b3DynamicTree* tree )
{
	return tree->proxyCount;
}

void b3DynamicTree_MoveProxy( b3DynamicTree* tree, int proxyId, b3AABB aabb )
{
	B3_ASSERT( b3IsValidAABB( aabb ) );
	B3_ASSERT( 0 <= proxyId && proxyId < tree->nodeCapacity );
	B3_ASSERT( b3IsLeaf( tree->nodes + proxyId ) );

	b3RemoveLeaf( tree, proxyId );

	tree->nodes[proxyId].aabb = aabb;

	bool shouldRotate = false;
	b3InsertLeaf( tree, proxyId, shouldRotate );
}

void b3DynamicTree_EnlargeProxy( b3DynamicTree* tree, int proxyId, b3AABB aabb )
{
	b3TreeNode* nodes = tree->nodes;
	B3_ASSERT( 0 <= proxyId && proxyId < tree->nodeCapacity );

	b3TreeNode* node = nodes + proxyId;
	node->aabb = aabb;

	int parentIndex = node->parent;
	while ( parentIndex != B3_NULL_INDEX )
	{
		node = nodes + parentIndex;
		bool changed = b3EnlargeAABB( &node->aabb, aabb );

		node->flags |= b3_enlargedNode;

		parentIndex = node->parent;

		if ( changed == false )
		{
			break;
		}
	}

	while ( parentIndex != B3_NULL_INDEX )
	{
		node = nodes + parentIndex;
		if ( node->flags & b3_enlargedNode )
		{
			// early out because this ancestor was previously ascended and marked as enlarged
			break;
		}

		node->flags |= b3_enlargedNode;
		parentIndex = node->parent;
	}
}

void b3DynamicTree_SetCategoryBits( b3DynamicTree* tree, int proxyId, uint64_t categoryBits )
{
	b3TreeNode* nodes = tree->nodes;

	B3_ASSERT( b3IsLeaf( nodes + proxyId ) );

	nodes[proxyId].categoryBits = categoryBits;

	// Fix up category bits in ancestor internal nodes
	int nodeIndex = nodes[proxyId].parent;
	while ( nodeIndex != B3_NULL_INDEX )
	{
		b3TreeNode* node = nodes + nodeIndex;
		int child1 = node->children.child1;
		B3_ASSERT( child1 != B3_NULL_INDEX );
		int child2 = node->children.child2;
		B3_ASSERT( child2 != B3_NULL_INDEX );
		node->categoryBits = nodes[child1].categoryBits | nodes[child2].categoryBits;

		nodeIndex = node->parent;
	}
}

uint64_t b3DynamicTree_GetCategoryBits( b3DynamicTree* tree, int proxyId )
{
	B3_ASSERT( 0 <= proxyId && proxyId < tree->nodeCapacity );
	return tree->nodes[proxyId].categoryBits;
}

int b3DynamicTree_GetHeight( const b3DynamicTree* tree )
{
	if ( tree->root == B3_NULL_INDEX )
	{
		return 0;
	}

	return tree->nodes[tree->root].height;
}

/// Ratio of total internal node area to root area, a tree quality metric.
///
/// Both areas are Q24 in int64 and the ratio is dimensionless, so the result
/// is a plain Q12 b3f. The division cannot go straight through b3HwDiv64
/// because that takes a 32-bit denominator and a root area at world scale does
/// not fit one. Both operands are shifted down together until it does, which
/// leaves the ratio unchanged -- scaling numerator and denominator by the same
/// power of two is exact.
b3f b3DynamicTree_GetAreaRatio( const b3DynamicTree* tree )
{
	if ( tree->root == B3_NULL_INDEX )
	{
		return b3f_zero;
	}

	const b3TreeNode* root = tree->nodes + tree->root;
	int64_t rootArea = b3Perimeter( root->aabb );

	if ( rootArea == 0 )
	{
		return b3f_zero;
	}

	int64_t totalArea = 0;
	for ( int i = 0; i < tree->nodeCapacity; ++i )
	{
		const b3TreeNode* node = tree->nodes + i;
		if ( b3IsAllocated( node ) == false || b3IsLeaf( node ) || i == tree->root )
		{
			continue;
		}

		totalArea += b3Perimeter( node->aabb );
	}

	// Bring the numerator up to Q12 first, then shift the pair down together
	// until the denominator is divisible and the quotient fits.
	int64_t num = totalArea << B3_F_SHIFT;
	int64_t den = rootArea;

	while ( den > INT32_MAX || num > ( (int64_t)INT32_MAX * den ) )
	{
		num >>= 1;
		den >>= 1;

		if ( den == 0 )
		{
			return b3f_zero;
		}
	}

	return b3Makeb3f( b3HwDiv64( num, (int32_t)den ) );
}

b3AABB b3DynamicTree_GetRootBounds( const b3DynamicTree* tree )
{
	if ( tree->root != B3_NULL_INDEX )
	{
		return tree->nodes[tree->root].aabb;
	}

	b3AABB empty = { b3Vec3_zero, b3Vec3_zero };
	return empty;
}

#if B3_ENABLE_VALIDATION
// Compute the height of a sub-tree.
static int b3ComputeHeightRecurse( const b3DynamicTree* tree, int nodeId )
{
	B3_ASSERT( 0 <= nodeId && nodeId < tree->nodeCapacity );
	b3TreeNode* node = tree->nodes + nodeId;

	if ( b3IsLeaf( node ) )
	{
		return 0;
	}

	int height1 = b3ComputeHeightRecurse( tree, node->children.child1 );
	int height2 = b3ComputeHeightRecurse( tree, node->children.child2 );
	return 1 + b3MaxInt( height1, height2 );
}

static int b3ComputeHeight( const b3DynamicTree* tree )
{
	int height = b3ComputeHeightRecurse( tree, tree->root );
	return height;
}

static void b3ValidateStructure( const b3DynamicTree* tree, int index )
{
	if ( index == B3_NULL_INDEX )
	{
		return;
	}

	if ( index == tree->root )
	{
		B3_ASSERT( tree->nodes[index].parent == B3_NULL_INDEX );
	}

	const b3TreeNode* node = tree->nodes + index;

	B3_ASSERT( node->flags == 0 || ( node->flags & b3_allocatedNode ) != 0 );

	if ( b3IsLeaf( node ) )
	{
		B3_ASSERT( node->height == 0 );
		return;
	}

	int child1 = node->children.child1;
	int child2 = node->children.child2;

	B3_ASSERT( 0 <= child1 && child1 < tree->nodeCapacity );
	B3_ASSERT( 0 <= child2 && child2 < tree->nodeCapacity );

	B3_ASSERT( tree->nodes[child1].parent == index );
	B3_ASSERT( tree->nodes[child2].parent == index );

	if ( ( tree->nodes[child1].flags | tree->nodes[child2].flags ) & b3_enlargedNode )
	{
		B3_ASSERT( node->flags & b3_enlargedNode );
	}

	b3ValidateStructure( tree, child1 );
	b3ValidateStructure( tree, child2 );
}

static void b3ValidateMetrics( const b3DynamicTree* tree, int index )
{
	if ( index == B3_NULL_INDEX )
	{
		return;
	}

	const b3TreeNode* node = tree->nodes + index;

	B3_ASSERT( b3IsValidAABB( node->aabb ) );

	if ( b3IsLeaf( node ) )
	{
		B3_ASSERT( node->height == 0 );
		return;
	}

	int child1 = node->children.child1;
	int child2 = node->children.child2;

	B3_ASSERT( 0 <= child1 && child1 < tree->nodeCapacity );
	B3_ASSERT( 0 <= child2 && child2 < tree->nodeCapacity );

	int height1 = tree->nodes[child1].height;
	int height2 = tree->nodes[child2].height;
	int height = 1 + b3MaxInt( height1, height2 );
	B3_ASSERT( node->height == height );

	B3_ASSERT( b3AABB_Contains( node->aabb, tree->nodes[child1].aabb ) );
	B3_ASSERT( b3AABB_Contains( node->aabb, tree->nodes[child2].aabb ) );

	uint64_t categoryBits = tree->nodes[child1].categoryBits | tree->nodes[child2].categoryBits;
	B3_ASSERT( node->categoryBits == categoryBits );

	b3ValidateMetrics( tree, child1 );
	b3ValidateMetrics( tree, child2 );
}
#endif

void b3DynamicTree_Validate( const b3DynamicTree* tree )
{
#if B3_ENABLE_VALIDATION
	if ( tree->root == B3_NULL_INDEX )
	{
		return;
	}

	b3ValidateStructure( tree, tree->root );
	b3ValidateMetrics( tree, tree->root );

	int freeCount = 0;
	int freeIndex = tree->freeList;
	while ( freeIndex != B3_NULL_INDEX )
	{
		B3_ASSERT( 0 <= freeIndex && freeIndex < tree->nodeCapacity );
		freeIndex = tree->nodes[freeIndex].next;
		++freeCount;
	}

	int height = b3DynamicTree_GetHeight( tree );
	int computedHeight = b3ComputeHeight( tree );
	B3_ASSERT( height == computedHeight );

	B3_ASSERT( tree->nodeCount + freeCount == tree->nodeCapacity );
#else
	B3_UNUSED( tree );
#endif
}

void b3DynamicTree_ValidateNoEnlarged( const b3DynamicTree* tree )
{
#if B3_ENABLE_VALIDATION
	int capacity = tree->nodeCapacity;
	const b3TreeNode* nodes = tree->nodes;
	for ( int i = 0; i < capacity; ++i )
	{
		const b3TreeNode* node = nodes + i;
		if ( node->flags & b3_allocatedNode )
		{
			B3_ASSERT( ( node->flags & b3_enlargedNode ) == 0 );
		}
	}
#else
	B3_UNUSED( tree );
#endif
}

int b3DynamicTree_GetByteCount( const b3DynamicTree* tree )
{
	size_t size = sizeof( b3DynamicTree ) + sizeof( b3TreeNode ) * tree->nodeCapacity +
				  tree->rebuildCapacity * ( sizeof( int ) + sizeof( b3AABB ) + sizeof( b3Vec3 ) );

	return (int)size;
}

b3TreeStats b3DynamicTree_Query( const b3DynamicTree* tree, b3AABB aabb, uint64_t maskBits, bool requireAllBits,
								 b3TreeQueryCallbackFcn* callback, void* context )
{
	b3TreeStats result = { 0 };

	if ( tree->nodeCount == 0 )
	{
		return result;
	}

	int stack[B3_TREE_STACK_SIZE];
	int stackCount = 0;
	stack[stackCount++] = tree->root;

	while ( stackCount > 0 )
	{
		int nodeId = stack[--stackCount];
		if ( nodeId == B3_NULL_INDEX )
		{
			B3_ASSERT( false );
			continue;
		}

		const b3TreeNode* node = tree->nodes + nodeId;
		result.nodeVisits += 1;

		uint64_t bitMatch = requireAllBits ? ( node->categoryBits & maskBits ) == maskBits : ( node->categoryBits & maskBits );

		if ( bitMatch && b3AABB_Overlaps( node->aabb, aabb ) )
		{
			if ( b3IsLeaf( node ) )
			{
				bool proceed = callback( nodeId, node->userData, context );
				result.leafVisits += 1;

				if ( proceed == false )
				{
					return result;
				}
			}
			else
			{
				B3_ASSERT( stackCount < B3_TREE_STACK_SIZE - 1 );
				if ( stackCount < B3_TREE_STACK_SIZE - 1 )
				{
					stack[stackCount++] = node->children.child1;
					stack[stackCount++] = node->children.child2;
				}
			}
		}
	}

	return result;
}

/// Squared distance from a point to a node's box, wide.
///
/// Q12 squared is Q24, which does not fit an int32 at world scale, and the
/// nearest-neighbour search compares these against each other constantly.
/// Narrowing would both overflow and throw away the resolution the comparison
/// depends on, so the whole QueryClosest path carries int64.
static inline int64_t b3DistanceToNodeSqr( b3Vec3 point, const b3TreeNode* node )
{
	b3Vec3 r = b3Sub( point, b3Clamp( point, node->aabb.lowerBound, node->aabb.upperBound ) );
	return b3LengthSquaredWide( r );
}

struct b3QueryClosestItem
{
	int nodeIndex;
	int64_t distanceToNodeSqr;
};

b3TreeStats b3DynamicTree_QueryClosest( const b3DynamicTree* tree, b3Vec3 point, uint64_t maskBits, bool requireAllBits,
										b3TreeQueryClosestCallbackFcn* callback, void* context, int64_t* minDistanceSqr )
{
	b3TreeStats result = { 0 };

	if ( tree->nodeCount == 0 )
	{
		return result;
	}

	int64_t minSqr = *minDistanceSqr;
	struct b3QueryClosestItem stack[B3_TREE_STACK_SIZE];
	int stackCount = 0;

	int64_t rootDistanceSqr = b3DistanceToNodeSqr( point, tree->nodes + tree->root );
	stack[stackCount++] = ( struct b3QueryClosestItem ){
		.nodeIndex = tree->root,
		.distanceToNodeSqr = rootDistanceSqr,
	};

	while ( stackCount > 0 )
	{
		struct b3QueryClosestItem item = stack[--stackCount];
		const b3TreeNode* node = tree->nodes + item.nodeIndex;
		result.nodeVisits += 1;

		uint64_t bitMatch = requireAllBits ? ( node->categoryBits & maskBits ) == maskBits : ( node->categoryBits & maskBits );

		if ( bitMatch )
		{
			if ( item.distanceToNodeSqr < minSqr )
			{
				if ( b3IsLeaf( node ) )
				{
					// callback to user code with minimum distance squared so far and proxy id
					int64_t dd = callback( minSqr, item.nodeIndex, node->userData, context );

					if ( dd < minSqr )
					{
						minSqr = dd;
					}

					result.leafVisits += 1;
				}
				else
				{
					B3_ASSERT( stackCount < B3_TREE_STACK_SIZE - 1 );
					if ( stackCount < B3_TREE_STACK_SIZE - 1 )
					{
						int child1 = node->children.child1;
						int child2 = node->children.child2;

						// Store the distance to node in the stack instead of recomputing after pop
						struct b3QueryClosestItem item1 = {
							.nodeIndex = child1,
							.distanceToNodeSqr = b3DistanceToNodeSqr( point, tree->nodes + child1 ),
						};

						struct b3QueryClosestItem item2 = {
							.nodeIndex = child2,
							.distanceToNodeSqr = b3DistanceToNodeSqr( point, tree->nodes + child2 ),
						};

						// Ensure we iterate the closest child first as we pop off the stack
						if ( item2.distanceToNodeSqr < item1.distanceToNodeSqr )
						{
							stack[stackCount++] = item1;
							stack[stackCount++] = item2;
						}
						else
						{
							stack[stackCount++] = item2;
							stack[stackCount++] = item1;
						}
					}
				}
			}
		}
	}

	*minDistanceSqr = minSqr;

	return result;
}

b3TreeStats b3DynamicTree_RayCast( const b3DynamicTree* tree, const b3RayCastInput* input, uint64_t maskBits, bool requireAllBits,
								   b3TreeRayCastCallbackFcn* callback, void* context )
{
	b3TreeStats result = { 0 };

	if ( tree->nodeCount == 0 )
	{
		return result;
	}

	b3Vec3 p1 = input->origin;
	b3Vec3 d = input->translation;

	b3c maxFraction = input->maxFraction;

	b3Vec3 p2 = b3Add( p1, b3MulCV( maxFraction, d ) );

	// Build a bounding box for the segment.
	b3AABB segmentAABB = { b3Min( p1, p2 ), b3Max( p1, p2 ) };

	int stack[B3_TREE_STACK_SIZE];
	int stackCount = 0;
	stack[stackCount++] = tree->root;

	const b3TreeNode* nodes = tree->nodes;

	b3RayCastInput subInput = *input;

	while ( stackCount > 0 )
	{
		int nodeId = stack[--stackCount];
		if ( nodeId == B3_NULL_INDEX )
		{
			B3_ASSERT( false );
			continue;
		}

		const b3TreeNode* node = nodes + nodeId;
		result.nodeVisits += 1;

		b3AABB nodeAABB = node->aabb;

		uint64_t bitMatch = requireAllBits ? ( node->categoryBits & maskBits ) == maskBits : ( node->categoryBits & maskBits );

		if ( bitMatch == 0 || b3AABB_Overlaps( nodeAABB, segmentAABB ) == false )
		{
			continue;
		}

		// The segment AABB test above is what bounds the ray's length; this
		// one is a separating-axis test on the infinite line, and the two are
		// only sufficient together.
		bool edgeOverlap = b3TestBoundsRayOverlap( nodeAABB.lowerBound, nodeAABB.upperBound, p1, d );
		if ( edgeOverlap == false )
		{
			continue;
		}

		if ( b3IsLeaf( node ) )
		{
			subInput.maxFraction = maxFraction;

			b3c value = callback( &subInput, nodeId, node->userData, context );
			result.leafVisits += 1;

			if ( b3Raw( value ) == 0 )
			{
				// The client has terminated the ray cast.
				return result;
			}

			if ( 0 < b3Raw( value ) && b3Raw( value ) <= b3Raw( maxFraction ) )
			{
				// Update segment bounding box.
				maxFraction = value;
				p2 = b3Add( p1, b3MulCV( maxFraction, d ) );
				segmentAABB.lowerBound = b3Min( p1, p2 );
				segmentAABB.upperBound = b3Max( p1, p2 );
			}
		}
		else
		{
			B3_ASSERT( stackCount < B3_TREE_STACK_SIZE - 1 );
			if ( stackCount < B3_TREE_STACK_SIZE - 1 )
			{
				b3Vec3 c1 = b3AABB_Center( nodes[node->children.child1].aabb );
				b3Vec3 c2 = b3AABB_Center( nodes[node->children.child2].aabb );

				// Wide, because this is a squared distance between arbitrary
				// world points. It only orders the traversal, but a wrapped
				// comparison would order it badly for exactly the distant
				// nodes where ordering matters most.
				if ( b3LengthSquaredWide( b3Sub( c1, p1 ) ) < b3LengthSquaredWide( b3Sub( c2, p1 ) ) )
				{
					stack[stackCount++] = node->children.child2;
					stack[stackCount++] = node->children.child1;
				}
				else
				{
					stack[stackCount++] = node->children.child1;
					stack[stackCount++] = node->children.child2;
				}
			}
		}
	}

	return result;
}

b3TreeStats b3DynamicTree_BoxCast( const b3DynamicTree* tree, const b3BoxCastInput* input, uint64_t maskBits, bool requireAllBits,
								   b3TreeBoxCastCallbackFcn* callback, void* context )
{
	b3TreeStats stats = { 0 };

	if ( tree->nodeCount == 0 )
	{
		return stats;
	}

	// The caller folds the shape radius and the world origin into the box
	b3AABB originAABB = input->box;

	b3Vec3 p1 = b3AABB_Center( originAABB );
	b3Vec3 extension = b3AABB_Extents( originAABB );

	b3Vec3 d = input->translation;

	b3c maxFraction = input->maxFraction;

	// Build total box for the cast
	b3Vec3 t = b3MulCV( maxFraction, input->translation );
	b3AABB totalAABB = {
		b3Min( originAABB.lowerBound, b3Add( originAABB.lowerBound, t ) ),
		b3Max( originAABB.upperBound, b3Add( originAABB.upperBound, t ) ),
	};

	b3BoxCastInput subInput = *input;
	const b3TreeNode* nodes = tree->nodes;

	int stack[B3_TREE_STACK_SIZE];
	int stackCount = 0;
	stack[stackCount++] = tree->root;

	while ( stackCount > 0 )
	{
		int nodeId = stack[--stackCount];
		if ( nodeId == B3_NULL_INDEX )
		{
			B3_ASSERT( false );
			continue;
		}

		const b3TreeNode* node = nodes + nodeId;
		stats.nodeVisits += 1;

		uint64_t bitMatch = requireAllBits ? ( node->categoryBits & maskBits ) == maskBits : ( node->categoryBits & maskBits );

		if ( bitMatch == 0 || b3AABB_Overlaps( node->aabb, totalAABB ) == false )
		{
			continue;
		}

		// The swept box is reduced to a ray by growing the node instead.
		b3Vec3 lower = b3Sub( node->aabb.lowerBound, extension );
		b3Vec3 upper = b3Add( node->aabb.upperBound, extension );
		bool edgeOverlap = b3TestBoundsRayOverlap( lower, upper, p1, d );
		if ( edgeOverlap == false )
		{
			continue;
		}

		if ( b3IsLeaf( node ) )
		{
			subInput.maxFraction = maxFraction;

			b3c value = callback( &subInput, nodeId, node->userData, context );
			stats.leafVisits += 1;

			if ( b3Raw( value ) == 0 )
			{
				// The client has terminated the cast.
				return stats;
			}

			if ( 0 < b3Raw( value ) && b3Raw( value ) < b3Raw( maxFraction ) )
			{
				maxFraction = value;
				t = b3MulCV( maxFraction, input->translation );
				totalAABB.lowerBound = b3Min( originAABB.lowerBound, b3Add( originAABB.lowerBound, t ) );
				totalAABB.upperBound = b3Max( originAABB.upperBound, b3Add( originAABB.upperBound, t ) );
			}
		}
		else
		{
			B3_ASSERT( stackCount < B3_TREE_STACK_SIZE - 1 );
			if ( stackCount < B3_TREE_STACK_SIZE - 1 )
			{
				b3Vec3 c1 = b3AABB_Center( nodes[node->children.child1].aabb );
				b3Vec3 c2 = b3AABB_Center( nodes[node->children.child2].aabb );
				if ( b3LengthSquaredWide( b3Sub( c1, p1 ) ) < b3LengthSquaredWide( b3Sub( c2, p1 ) ) )
				{
					stack[stackCount++] = node->children.child2;
					stack[stackCount++] = node->children.child1;
				}
				else
				{
					stack[stackCount++] = node->children.child1;
					stack[stackCount++] = node->children.child2;
				}
			}
		}
	}

	return stats;
}

// Median split. Upstream's binned-SAH alternative is not ported; see the file
// header for why.
//
// The pivot is a coordinate, so it is an ordinary b3f. b3AABB_Center already
// halves by shift, which is exact.
static int b3PartitionMid( int* indices, b3Vec3* centers, int count )
{
	// Handle trivial case
	if ( count <= 2 )
	{
		return count / 2;
	}

	b3Vec3 lowerBound = centers[0];
	b3Vec3 upperBound = centers[0];

	for ( int i = 1; i < count; ++i )
	{
		lowerBound = b3Min( lowerBound, centers[i] );
		upperBound = b3Max( upperBound, centers[i] );
	}

	b3Vec3 d = b3Sub( upperBound, lowerBound );
	b3Vec3 c = b3AABB_Center( ( b3AABB ){ lowerBound, upperBound } );

	// Partition longest axis using the Hoare partition scheme
	// https://en.wikipedia.org/wiki/Quicksort
	// https://nicholasvadivelu.com/2021/01/11/array-partition/
	int i1 = 0, i2 = count;
	if ( b3Raw( d.x ) >= b3Raw( d.y ) && b3Raw( d.x ) >= b3Raw( d.z ) )
	{
		b3f pivot = c.x;

		while ( i1 < i2 )
		{
			while ( i1 < i2 && b3Raw( centers[i1].x ) < b3Raw( pivot ) )
			{
				i1 += 1;
			};

			while ( i1 < i2 && b3Raw( centers[i2 - 1].x ) >= b3Raw( pivot ) )
			{
				i2 -= 1;
			};

			if ( i1 < i2 )
			{
				// Swap indices
				{
					int temp = indices[i1];
					indices[i1] = indices[i2 - 1];
					indices[i2 - 1] = temp;
				}

				// Swap centers
				{
					b3Vec3 temp = centers[i1];
					centers[i1] = centers[i2 - 1];
					centers[i2 - 1] = temp;
				}

				i1 += 1;
				i2 -= 1;
			}
		}
	}
	else if ( b3Raw( d.y ) >= b3Raw( d.z ) )
	{
		b3f pivot = c.y;

		while ( i1 < i2 )
		{
			while ( i1 < i2 && b3Raw( centers[i1].y ) < b3Raw( pivot ) )
			{
				i1 += 1;
			};

			while ( i1 < i2 && b3Raw( centers[i2 - 1].y ) >= b3Raw( pivot ) )
			{
				i2 -= 1;
			};

			if ( i1 < i2 )
			{
				// Swap indices
				{
					int temp = indices[i1];
					indices[i1] = indices[i2 - 1];
					indices[i2 - 1] = temp;
				}

				// Swap centers
				{
					b3Vec3 temp = centers[i1];
					centers[i1] = centers[i2 - 1];
					centers[i2 - 1] = temp;
				}

				i1 += 1;
				i2 -= 1;
			}
		}
	}
	else
	{
		b3f pivot = c.z;

		while ( i1 < i2 )
		{
			while ( i1 < i2 && b3Raw( centers[i1].z ) < b3Raw( pivot ) )
			{
				i1 += 1;
			};

			while ( i1 < i2 && b3Raw( centers[i2 - 1].z ) >= b3Raw( pivot ) )
			{
				i2 -= 1;
			};

			if ( i1 < i2 )
			{
				// Swap indices
				{
					int temp = indices[i1];
					indices[i1] = indices[i2 - 1];
					indices[i2 - 1] = temp;
				}

				// Swap centers
				{
					b3Vec3 temp = centers[i1];
					centers[i1] = centers[i2 - 1];
					centers[i2 - 1] = temp;
				}

				i1 += 1;
				i2 -= 1;
			}
		}
	}
	B3_ASSERT( i1 == i2 );

	if ( i1 > 0 && i1 < count )
	{
		return i1;
	}

	return count / 2;
}

struct b3RebuildItem
{
	int nodeIndex;
	int childCount;

	// Leaf indices
	int startIndex;
	int splitIndex;
	int endIndex;
};

static int b3BuildTree( b3DynamicTree* tree, int leafCount )
{
	b3TreeNode* nodes = tree->nodes;
	int* leafIndices = tree->leafIndices;

	if ( leafCount == 1 )
	{
		nodes[leafIndices[0]].parent = B3_NULL_INDEX;
		return leafIndices[0];
	}

	b3Vec3* leafCenters = tree->leafCenters;

	struct b3RebuildItem stack[B3_TREE_STACK_SIZE];
	int top = 0;

	stack[0].nodeIndex = b3AllocateNode( tree );

	if ( stack[0].nodeIndex == B3_NULL_INDEX )
	{
		return B3_NULL_INDEX;
	}

	// b3AllocateNode can move the pool, so the cached base is refreshed after
	// every call to it in this function.
	//
	// It cannot actually move here: b3DynamicTree_Rebuild frees all the
	// internal nodes before calling in, and the build allocates exactly the
	// leafCount - 1 it just released. But that is an invariant of the caller,
	// not of this function, and upstream leaves it unwritten -- so refresh
	// consistently rather than relying on a property from another file.
	nodes = tree->nodes;

	stack[0].childCount = -1;
	stack[0].startIndex = 0;
	stack[0].endIndex = leafCount;
	stack[0].splitIndex = b3PartitionMid( leafIndices, leafCenters, leafCount );

	while ( true )
	{
		struct b3RebuildItem* item = stack + top;

		item->childCount += 1;

		if ( item->childCount == 2 )
		{
			// This internal node has both children established

			if ( top == 0 )
			{
				// all done
				break;
			}

			struct b3RebuildItem* parentItem = stack + ( top - 1 );
			b3TreeNode* parentNode = nodes + parentItem->nodeIndex;

			if ( parentItem->childCount == 0 )
			{
				B3_ASSERT( parentNode->children.child1 == B3_NULL_INDEX );
				parentNode->children.child1 = item->nodeIndex;
			}
			else
			{
				B3_ASSERT( parentItem->childCount == 1 );
				B3_ASSERT( parentNode->children.child2 == B3_NULL_INDEX );
				parentNode->children.child2 = item->nodeIndex;
			}

			b3TreeNode* node = nodes + item->nodeIndex;

			B3_ASSERT( node->parent == B3_NULL_INDEX );
			node->parent = parentItem->nodeIndex;

			B3_ASSERT( node->children.child1 != B3_NULL_INDEX );
			B3_ASSERT( node->children.child2 != B3_NULL_INDEX );
			b3TreeNode* child1 = nodes + node->children.child1;
			b3TreeNode* child2 = nodes + node->children.child2;

			node->aabb = b3AABB_Union( child1->aabb, child2->aabb );
			node->height = 1 + b3MaxUInt16( child1->height, child2->height );
			node->categoryBits = child1->categoryBits | child2->categoryBits;

			// Pop stack
			top -= 1;
		}
		else
		{
			int startIndex, endIndex;
			if ( item->childCount == 0 )
			{
				startIndex = item->startIndex;
				endIndex = item->splitIndex;
			}
			else
			{
				B3_ASSERT( item->childCount == 1 );
				startIndex = item->splitIndex;
				endIndex = item->endIndex;
			}

			int count = endIndex - startIndex;

			if ( count == 1 )
			{
				int childIndex = leafIndices[startIndex];
				b3TreeNode* node = nodes + item->nodeIndex;

				if ( item->childCount == 0 )
				{
					B3_ASSERT( node->children.child1 == B3_NULL_INDEX );
					node->children.child1 = childIndex;
				}
				else
				{
					B3_ASSERT( item->childCount == 1 );
					B3_ASSERT( node->children.child2 == B3_NULL_INDEX );
					node->children.child2 = childIndex;
				}

				b3TreeNode* childNode = nodes + childIndex;
				B3_ASSERT( childNode->parent == B3_NULL_INDEX );
				childNode->parent = item->nodeIndex;
			}
			else
			{
				B3_ASSERT( count > 0 );
				B3_ASSERT( top < B3_TREE_STACK_SIZE );

				top += 1;
				struct b3RebuildItem* newItem = stack + top;
				newItem->nodeIndex = b3AllocateNode( tree );

				if ( newItem->nodeIndex == B3_NULL_INDEX )
				{
					return B3_NULL_INDEX;
				}

				// The pool may have moved.
				nodes = tree->nodes;

				newItem->childCount = -1;
				newItem->startIndex = startIndex;
				newItem->endIndex = endIndex;
				newItem->splitIndex = b3PartitionMid( leafIndices + startIndex, leafCenters + startIndex, count );
				newItem->splitIndex += startIndex;
			}
		}
	}

	b3TreeNode* rootNode = nodes + stack[0].nodeIndex;
	B3_ASSERT( rootNode->parent == B3_NULL_INDEX );
	B3_ASSERT( rootNode->children.child1 != B3_NULL_INDEX );
	B3_ASSERT( rootNode->children.child2 != B3_NULL_INDEX );

	b3TreeNode* child1 = nodes + rootNode->children.child1;
	b3TreeNode* child2 = nodes + rootNode->children.child2;

	rootNode->aabb = b3AABB_Union( child1->aabb, child2->aabb );
	rootNode->height = 1 + b3MaxUInt16( child1->height, child2->height );
	rootNode->categoryBits = child1->categoryBits | child2->categoryBits;

	return stack[0].nodeIndex;
}

// Not safe to access tree during this operation because it may grow
int b3DynamicTree_Rebuild( b3DynamicTree* tree, bool fullBuild )
{
	int proxyCount = tree->proxyCount;
	if ( proxyCount == 0 )
	{
		return 0;
	}

	// Ensure capacity for rebuild space
	if ( proxyCount > tree->rebuildCapacity )
	{
		int newCapacity = proxyCount + proxyCount / 2;

		int* newIndices = (int*)b3Alloc( newCapacity * sizeof( int ) );
		b3Vec3* newCenters = (b3Vec3*)b3Alloc( newCapacity * sizeof( b3Vec3 ) );

		if ( newIndices == NULL || newCenters == NULL )
		{
			// Refused. Leave the existing tree alone -- it is still valid, just
			// not rebuilt -- rather than tearing down the scratch buffers.
			b3Free( newIndices, newCapacity * sizeof( int ) );
			b3Free( newCenters, newCapacity * sizeof( b3Vec3 ) );
			return 0;
		}

		b3Free( tree->leafIndices, tree->rebuildCapacity * sizeof( int ) );
		tree->leafIndices = newIndices;

		b3Free( tree->leafCenters, tree->rebuildCapacity * sizeof( b3Vec3 ) );
		tree->leafCenters = newCenters;

		tree->rebuildCapacity = newCapacity;
	}

	int leafCount = 0;
	int stack[B3_TREE_STACK_SIZE];
	int stackCount = 0;

	int nodeIndex = tree->root;
	b3TreeNode* nodes = tree->nodes;
	b3TreeNode* node = nodes + nodeIndex;

	// These are the nodes that get sorted to rebuild the tree.
	// I'm using indices because the node pool may grow during the build.
	int* leafIndices = tree->leafIndices;
	b3Vec3* leafCenters = tree->leafCenters;

	// Gather all proxy nodes that have grown and all internal nodes that haven't grown. Both are
	// considered leaves in the tree rebuild.
	// Free all internal nodes that have grown.
	while ( true )
	{
		if ( b3IsLeaf( node ) == true || ( ( node->flags & b3_enlargedNode ) == 0 && fullBuild == false ) )
		{
			leafIndices[leafCount] = nodeIndex;
			leafCenters[leafCount] = b3AABB_Center( node->aabb );
			leafCount += 1;

			// Detach
			node->parent = B3_NULL_INDEX;
		}
		else
		{
			int doomedNodeIndex = nodeIndex;

			// Handle children
			nodeIndex = node->children.child1;

			B3_ASSERT( stackCount < B3_TREE_STACK_SIZE );
			if ( stackCount < B3_TREE_STACK_SIZE )
			{
				stack[stackCount++] = node->children.child2;
			}

			node = nodes + nodeIndex;

			// Remove doomed node
			b3FreeNode( tree, doomedNodeIndex );

			continue;
		}

		if ( stackCount == 0 )
		{
			break;
		}

		nodeIndex = stack[--stackCount];
		node = nodes + nodeIndex;
	}

#if B3_ENABLE_VALIDATION
	int capacity = tree->nodeCapacity;
	for ( int i = 0; i < capacity; ++i )
	{
		if ( nodes[i].flags & b3_allocatedNode )
		{
			B3_ASSERT( ( nodes[i].flags & b3_enlargedNode ) == 0 );
		}
	}
#endif

	B3_ASSERT( leafCount <= proxyCount );

	tree->root = b3BuildTree( tree, leafCount );

	b3DynamicTree_Validate( tree );

	return leafCount;
}
