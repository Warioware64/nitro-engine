// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "container.h"

#include "box3d/base.h"

/// The **largest** granularity an allocator may use, as an exponent.
///
/// Upstream uses 8 (256 elements) with no per-allocator override, which is
/// sized for a desktop. It is the wrong unit here, and not by a little: the
/// only user is the manifold pool in physics_world.h, `sizeof( b3Manifold )`
/// is 268 bytes, and 256 of those is a **68 KB block** that the allocator
/// reserves no matter how small a capacity is asked for.
///
/// 5 (32 elements, ~8.5 KB for one-manifold entries) is a granularity a scene
/// can actually be sized in. A world with 32 contacts then reserves one block
/// instead of rounding up to 256 manifolds it will never use.
#define B3_BLOCK_EXPONENT 5
#define B3_BLOCK_SIZE ( 1 << B3_BLOCK_EXPONENT )

typedef struct b3Block
{
	char* memory;
} b3Block;

b3DeclareArray( b3Block );

typedef struct b3BlockAllocator
{
	b3Array( b3Block ) blocks;
	void* freeList;
	int elementSize;

	/// This allocator's own granularity, as an exponent, at most
	/// B3_BLOCK_EXPONENT. Chosen from `initialCount` at creation.
	///
	/// Upstream has one global granularity, which forces every size class to
	/// reserve the same *element count* however few elements it will hold.
	/// That is affordable when there is one size class; the mesh manifold
	/// class is 8 * sizeof( b3Manifold ) = 2,144 bytes per element, and a
	/// scene with eight mesh contacts would reserve thirty-two of them. Per
	/// allocator, it reserves eight.
	int blockExponent;

	int nextIndex;
	int allocationCount;
} b3BlockAllocator;

/// Element must be large enough to hold a pointer. The element size is rounded up to B3_ALIGNMENT.
///
/// `initialCount` elements are reserved up front, and it also picks the
/// granularity: blocks hold `initialCount` rounded up to a power of two,
/// capped at B3_BLOCK_SIZE. Pass 0 to reserve nothing -- which on this port
/// means the allocator is not expected to be used at all, so growth is in the
/// smallest steps there are rather than the largest.
b3BlockAllocator b3CreateBlockAllocator( int elementSize, int initialCount );
void b3DestroyBlockAllocator( b3BlockAllocator* allocator );

// Returns one element of elementSize contiguous bytes. Address is stable until freed.
void* b3AllocateElement( b3BlockAllocator* allocator );
void b3FreeElement( b3BlockAllocator* allocator, void* element );
