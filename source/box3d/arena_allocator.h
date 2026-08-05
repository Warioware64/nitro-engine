// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once
#include "box3d/base.h"
#include "container.h"

#include <stdbool.h>
#include <stddef.h>

#define B3_MAX_STACK_ENTRIES 32

typedef struct b3StackEntry
{
	char* data;
	const char* name;
	int size;
	bool usedMalloc;
} b3StackEntry;

// This is a stack-like arena allocator used for fast per step allocations.
// You must nest allocate/free pairs. The code will B3_ASSERT
// if you try to interleave multiple allocate/free pairs.
// This allocator uses the heap if space is insufficient.
// I could remove the need to free entries individually.
typedef struct b3Stack
{
	char* memory;
	int capacity;
	int index;

	int allocation;
	int maxAllocation;

	b3StackEntry entries[B3_MAX_STACK_ENTRIES];
	int entryCount;
} b3Stack;

// Heap-allocated fallback block tracked when an arena bump overflows.
typedef struct b3OverflowBlock
{
	char* data;
	int size;
} b3OverflowBlock;

b3DeclareArray( b3OverflowBlock );

// Shared, heap-allocated state co-owned by every copy of a b3Arena.
// b3Arena is passed by value so its bump pointer auto-restores on
// function return, but overflow tracking and watermarks must persist
// across copies -- hence this pointer-shared block.
typedef struct b3ArenaSharedState
{
	b3Array( b3OverflowBlock ) overflows;
	int maxIndex;          // high water mark of the bump pointer this step
	int overflowBytes;     // total bytes in overflow blocks this step
	int peakDemand;        // all-time peak of (maxIndex + overflowBytes), survives sync
} b3ArenaSharedState;

typedef struct b3Arena
{
	char* memory;
	int capacity;
	int index;
	b3ArenaSharedState* shared;
} b3Arena;

/// 8-byte alignment: what LDRD/STRD require on ARM9, and nothing here is
/// wider. Upstream uses 16 for SSE2, which would waste up to 8 bytes on every
/// bump in a 4 MB machine.
#define B3_ARENA_ALIGNMENT 8

b3Stack b3CreateStack( int capacity );
void b3DestroyStack( b3Stack* stack );

void* b3StackAlloc( b3Stack* stack, int size, const char* name );
void b3StackFree( b3Stack* stack, void* mem );

// Grow the stack based on usage
void b3GrowStack( b3Stack* stack );

int b3GetStackCapacity( b3Stack* stack );
int b3GetStackAllocation( b3Stack* stack );
int b3GetMaxStackAllocation( b3Stack* stack );

b3Arena b3CreateArena( int capacity );
void b3DestroyArena( b3Arena* arena );

/// Heap-allocate an overflow block, register it in the shared state, return it.
///
/// This is the one allocation path the port deliberately keeps rather than
/// refusing, and it is worth being precise about why.
///
/// Refusing here is not an option: the solver's callers write through the
/// returned pointer without checking, so NULL would be a crash. But unlike a
/// dynamic array, the arena is self-correcting -- b3ArenaSync records the
/// step's peak demand and grows the backing block to match before the next
/// step. So an under-sized arena costs a few heap allocations on the first
/// frame or two and then settles to zero for the rest of the run.
///
/// It is still a sizing mistake, so it logs and asserts under NEA_DEBUG.
/// Reaching a steady state is the intended behaviour; reaching it every frame
/// means arenaBytes is genuinely too small.
void* b3ArenaOverflowAlloc( b3Arena* arena, int size );

// Call between simulation steps. Frees this step's overflow blocks and grows
// the backing capacity if last step's demand (maxIndex + overflowBytes) exceeded it.
void b3ArenaSync( b3Arena* arena );

static inline void* b3Bump( b3Arena* arena, int size )
{
	if ( size == 0 )
	{
		return NULL;
	}

	int aligned = ( arena->index + ( B3_ARENA_ALIGNMENT - 1 ) ) & ~( B3_ARENA_ALIGNMENT - 1 );

	if ( aligned + size > arena->capacity )
	{
		return b3ArenaOverflowAlloc( arena, size );
	}

	arena->index = aligned + size;
	if ( arena->index > arena->shared->maxIndex )
	{
		arena->shared->maxIndex = arena->index;
	}
	return arena->memory + aligned;
}
