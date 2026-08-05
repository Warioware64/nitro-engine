// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Erin Catto        (original Box3D)
// Copyright (c) 2026 Warioware64       (Nitro Engine Advanced fixed-point port)
//
// This file is part of Nitro Engine Advanced

#pragma once

/// @file   mesh.h
/// @brief  Triangle mesh internals.
///
/// The public mesh surface -- the blob accessors, b3IsValidMesh,
/// b3ComputeMeshAABB and b3QueryMesh -- is in include/box3d/collision.h. What
/// is here is what only the collision tier needs: the leaf tag, the child
/// walk, and the triangle fetch that applies the mesh scale.

#include "box3d/collision.h"
#include "box3d/types.h"

/// Tag value in b3MeshNode::data marking a leaf.
///
/// An internal node stores its split axis in the same two bits, and an axis is
/// 0, 1 or 2 -- so 3 is the one value left over and needs no extra storage.
#define B3_LEAF_NODE 3

/// Node pointers a traversal may stack, and therefore the tallest tree the
/// device will walk.
///
/// Upstream uses 256, which is 1 KB of C stack per query. Nothing a DS level
/// contains comes close: the baker puts at most 4 triangles in a leaf, so even
/// a badly balanced 4,000-triangle mesh is about 12 deep. 32 is the bound the
/// baker enforces at bake time, which turns upstream's `todo` about rejecting
/// tall trees into something the device can rely on rather than assert about.
#define B3_MESH_STACK_SIZE 32

/// One triangle of a mesh, in the mesh's local frame with scale applied.
///
/// `i1`/`i2`/`i3` are the vertex indices the triangle was built from and
/// `flags` its b3MeshEdgeFlags; the narrow phase needs both to decide whether
/// a contact against a shared edge has already been claimed by the neighbour.
typedef struct b3Triangle
{
	b3Vec3 vertices[3];
	int i1, i2, i3;
	int flags;
} b3Triangle;

static inline bool b3IsLeaf( const b3MeshNode* node )
{
	return node->data.asLeaf.type == B3_LEAF_NODE;
}

/// The left child follows its parent, because the builder emits nodes in
/// pre-order. That is why only the right child needs a stored offset.
static inline const b3MeshNode* b3GetLeftChild( const b3MeshNode* node )
{
	B3_ASSERT( b3IsLeaf( node ) == false );
	return node + 1;
}

/// The right child's offset is relative to this node, which is what keeps it
/// inside 30 bits.
static inline const b3MeshNode* b3GetRightChild( const b3MeshNode* node )
{
	B3_ASSERT( b3IsLeaf( node ) == false );
	return node + node->data.asNode.childOffset;
}

/// The first node is the root.
static inline const b3MeshNode* b3GetMeshRoot( const b3MeshData* mesh )
{
	return b3GetMeshNodes( mesh );
}

/// One triangle in the mesh's local frame, with b3Mesh::scale applied.
///
/// A scale with a negative determinant reflects the mesh, which reverses
/// winding: vertices 2 and 3 swap, their indices swap with them, and what was
/// concave becomes convex -- so the inverse-concave flags become the concave
/// ones. Internal, because b3Triangle is.
b3Triangle b3GetMeshTriangle( const b3Mesh* mesh, int triangleIndex );
