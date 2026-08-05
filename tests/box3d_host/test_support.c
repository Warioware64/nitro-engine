// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Warioware64
//
// This file is part of Nitro Engine Advanced

// Verification of the Box3D support layer: bit manipulation, containers,
// id recycling, the shape-pair hash set, and the two allocators.
//
// None of this involves fixed-point arithmetic, so the tests are about
// behaviour rather than precision. Two areas get particular attention because
// the port changed them:
//
//   - The 64-bit CTZ and popcount, which were rewritten to avoid libgcc calls
//     on ARMv5TE. A wrong bit index here would corrupt island traversal in a
//     way that is very hard to trace back.
//
//   - The refused-allocation path, which does not exist upstream at all.
//     Upstream's allocator cannot fail; the port's can, and every container
//     has to survive it without dereferencing NULL.

#include "box3d/constants.h"

#include "arena_allocator.h"
#include "bitset.h"
#include "block_allocator.h"
#include "container.h"
#include "core.h"
#include "ctz.h"
#include "id_pool.h"
#include "table.h"

#include <stdio.h>
#include <string.h>

#include "assert_trap.h"

static int s_failures = 0;
static int s_checks = 0;

static void check( const char* what, bool ok )
{
	s_checks++;
	if ( !ok )
	{
		printf( "  FAIL %s\n", what );
		s_failures++;
	}
}

static void checkInt( const char* what, long long got, long long want )
{
	s_checks++;
	if ( got != want )
	{
		printf( "  FAIL %-40s got %lld want %lld\n", what, got, want );
		s_failures++;
	}
}

static void section( const char* name )
{
	printf( "%s\n", name );
}

// -------------------------------------------------------------------------

static void test_ctz( void )
{
	section( "bit scan and population count" );

	checkInt( "ctz32(1)", b3CTZ32( 1 ), 0 );
	checkInt( "ctz32(1<<17)", b3CTZ32( 1u << 17 ), 17 );
	checkInt( "ctz32(1<<31)", b3CTZ32( 1u << 31 ), 31 );
	checkInt( "clz32(1)", b3CLZ32( 1 ), 31 );
	checkInt( "clz32(1<<31)", b3CLZ32( 1u << 31 ), 0 );

	// The rewritten 64-bit form. The word boundary at bit 32 is where a
	// naive lo/hi split goes wrong, so it is checked from both sides.
	checkInt( "ctz64(1)", b3CTZ64( 1 ), 0 );
	checkInt( "ctz64(1<<31)", b3CTZ64( (uint64_t)1 << 31 ), 31 );
	checkInt( "ctz64(1<<32)", b3CTZ64( (uint64_t)1 << 32 ), 32 );
	checkInt( "ctz64(1<<33)", b3CTZ64( (uint64_t)1 << 33 ), 33 );
	checkInt( "ctz64(1<<63)", b3CTZ64( (uint64_t)1 << 63 ), 63 );

	// Exhaustive over every single-bit value, which is the only pattern
	// bitset iteration ever presents.
	for ( int i = 0; i < 64; i++ )
	{
		if ( b3CTZ64( (uint64_t)1 << i ) != (uint32_t)i )
		{
			printf( "  FAIL ctz64 of bit %d\n", i );
			s_failures++;
		}
		s_checks++;
	}

	// Low bit must win when several are set.
	checkInt( "ctz64 lowest set bit", b3CTZ64( 0xFFFFFFFF00000000ull | ( (uint64_t)1 << 40 ) ), 32 );

	checkInt( "popcount32(0)", b3PopCount32( 0 ), 0 );
	checkInt( "popcount32(~0)", b3PopCount32( 0xFFFFFFFFu ), 32 );
	checkInt( "popcount32(0xF0F0)", b3PopCount32( 0xF0F0u ), 8 );
	checkInt( "popcount64(~0)", b3PopCount64( ~(uint64_t)0 ), 64 );
	checkInt( "popcount64 split", b3PopCount64( 0x00FF00FF00FF00FFull ), 32 );

	checkInt( "roundUpPow2(1)", b3RoundUpPowerOf2( 1 ), 1 );
	checkInt( "roundUpPow2(5)", b3RoundUpPowerOf2( 5 ), 8 );
	checkInt( "roundUpPow2(64)", b3RoundUpPowerOf2( 64 ), 64 );
	check( "isPowerOf2(64)", b3IsPowerOf2( 64 ) );
	check( "!isPowerOf2(63)", !b3IsPowerOf2( 63 ) );
}

static void test_bitset( void )
{
	section( "bit set" );

	b3BitSet set = b3CreateBitSet( 200 );
	b3SetBitCountAndClear( &set, 200 );

	check( "starts clear", b3GetBit( &set, 0 ) == false );

	b3SetBit( &set, 0 );
	b3SetBit( &set, 63 );
	b3SetBit( &set, 64 );
	b3SetBit( &set, 199 );

	check( "bit 0", b3GetBit( &set, 0 ) );
	check( "bit 63", b3GetBit( &set, 63 ) );
	check( "bit 64", b3GetBit( &set, 64 ) );
	check( "bit 199", b3GetBit( &set, 199 ) );
	check( "bit 1 still clear", !b3GetBit( &set, 1 ) );
	checkInt( "count", b3CountSetBits( &set ), 4 );

	b3ClearBit( &set, 63 );
	check( "cleared bit 63", !b3GetBit( &set, 63 ) );
	checkInt( "count after clear", b3CountSetBits( &set ), 3 );

	// Growth beyond the initial capacity.
	b3SetBitGrow( &set, 500 );
	check( "grown bit 500", b3GetBit( &set, 500 ) );
	check( "bit 0 survived growth", b3GetBit( &set, 0 ) );

	// Union, used to merge per-worker sets. One worker here, but the
	// operation is still on the step path.
	//
	// b3InPlaceUnion requires equal block counts -- it iterates setA's count
	// and indexes setB with it, so a smaller setB is read out of bounds. `set`
	// has just been grown to hold bit 500, so `other` is sized from what `set`
	// actually ended up with rather than from the 200 it started at.
	b3BitSet other = b3CreateBitSet( set.blockCount * 64 );
	b3SetBitCountAndClear( &other, set.blockCount * 64 );
	checkInt( "union operands have equal block counts", other.blockCount, set.blockCount );
	b3SetBit( &other, 5 );
	b3SetBit( &other, 6 );
	b3InPlaceUnion( &set, &other );
	check( "union brought bit 5", b3GetBit( &set, 5 ) );
	check( "union brought bit 6", b3GetBit( &set, 6 ) );
	check( "union kept bit 0", b3GetBit( &set, 0 ) );

	// Reading past the end must be false, not a crash: b3GetBit is called
	// with body ids that may exceed the set's current size.
	check( "out of range reads false", !b3GetBit( &set, 100000 ) );

	b3DestroyBitSet( &other );
	b3DestroyBitSet( &set );
}

static void test_id_pool( void )
{
	section( "id pool" );

	b3IdPool pool = b3CreateIdPool();

	int a = b3AllocId( &pool );
	int b = b3AllocId( &pool );
	int c = b3AllocId( &pool );
	checkInt( "first id", a, 0 );
	checkInt( "second id", b, 1 );
	checkInt( "third id", c, 2 );
	checkInt( "count", b3GetIdCount( &pool ), 3 );

	// A freed id must come back before a fresh one is minted, which is what
	// keeps the id space dense and the arrays small.
	b3FreeId( &pool, b );
	checkInt( "count after free", b3GetIdCount( &pool ), 2 );
	checkInt( "recycled id", b3AllocId( &pool ), 1 );
	checkInt( "count after recycle", b3GetIdCount( &pool ), 3 );
	checkInt( "capacity unchanged by recycle", b3GetIdCapacity( &pool ), 3 );

	b3DestroyIdPool( &pool );
}

static void test_hash_set( void )
{
	section( "shape pair hash set" );

	// The key must not depend on the order the pair is given in, or the
	// broad phase would create the same contact twice.
	b3ShapeKey k1 = b3ShapePairKey( 7, 42, 0 );
	b3ShapeKey k2 = b3ShapePairKey( 42, 7, 0 );
	checkInt( "pair key is order independent", k1, k2 );

	// Distinct pairs and distinct children must not collide.
	check( "different pair differs", b3ShapePairKey( 7, 43, 0 ) != k1 );
	check( "different child differs", b3ShapePairKey( 7, 42, 1 ) != k1 );

	// The port packs this into 32 bits; confirm the width actually shrank
	// rather than the change silently reverting.
	checkInt( "key width", B3_SHAPE_KEY_BITS, 32 );
	checkInt( "key type size", (long long)sizeof( b3ShapeKey ), 4 );

	// Extreme indices must still fit their fields.
	b3ShapeKey kmax = b3ShapePairKey( B3_MAX_SHAPES - 1, B3_MAX_SHAPES - 2, B3_MAX_CHILD_SHAPES - 1 );
	check( "max indices produce a non-zero key", kmax != 0 );

	b3HashSet set = b3CreateSet( 32 );

	check( "empty set does not contain", !b3ContainsKey( &set, k1 ) );
	check( "add reports not-present", !b3AddKey( &set, k1 ) );
	check( "contains after add", b3ContainsKey( &set, k1 ) );
	check( "re-add reports present", b3AddKey( &set, k1 ) );

	// Enough insertions to force a rehash, checking nothing is lost.
	for ( int i = 1; i < 200; i++ )
	{
		b3AddKey( &set, b3ShapePairKey( i, i + 1, 0 ) );
	}
	for ( int i = 1; i < 200; i++ )
	{
		if ( !b3ContainsKey( &set, b3ShapePairKey( i, i + 1, 0 ) ) )
		{
			printf( "  FAIL key %d lost across rehash\n", i );
			s_failures++;
			break;
		}
	}
	s_checks++;
	check( "original key survived rehash", b3ContainsKey( &set, k1 ) );

	check( "remove reports found", b3RemoveKey( &set, k1 ) );
	check( "gone after remove", !b3ContainsKey( &set, k1 ) );
	check( "remove of absent reports false", !b3RemoveKey( &set, k1 ) );

	b3DestroySet( &set );
}

static void test_containers( void )
{
	section( "dynamic arrays" );

	b3Array( int ) arr = { 0 };

	for ( int i = 0; i < 100; i++ )
	{
		b3Array_Push( arr, i * 3 );
	}
	checkInt( "count after pushes", arr.count, 100 );
	checkInt( "element 0", *b3Array_Get( arr, 0 ), 0 );
	checkInt( "element 99", *b3Array_Get( arr, 99 ), 297 );

	checkInt( "pop returns last", b3Array_Pop( arr ), 297 );
	checkInt( "count after pop", arr.count, 99 );

	// Swap-remove moves the last element into the hole.
	int last = *b3Array_Get( arr, arr.count - 1 );
	b3Array_RemoveSwap( arr, 0 );
	checkInt( "swap remove moved last into 0", *b3Array_Get( arr, 0 ), last );
	checkInt( "count after swap remove", arr.count, 98 );

	b3Array_Destroy( arr );

	// Reserve up front, which is how the port actually uses these.
	b3Array( int ) reserved;
	b3Array_CreateN( reserved, 64 );
	checkInt( "reserved capacity", reserved.capacity, 64 );
	checkInt( "reserved count", reserved.count, 0 );
	for ( int i = 0; i < 64; i++ )
	{
		b3Array_Push( reserved, i );
	}
	checkInt( "filled to capacity without growing", reserved.capacity, 64 );
	b3Array_Destroy( reserved );
}

// -------------------------------------------------------------------------
// Refused allocation
// -------------------------------------------------------------------------
//
// This path does not exist upstream, where allocation cannot fail. Here a
// pre-reserved world can run out, and the contract is that nothing
// dereferences NULL and the out-of-memory flag is raised.

static void* failingAlloc( int32_t size, int32_t alignment )
{
	(void)size;
	(void)alignment;
	return NULL;
}

static void failingFree( void* mem )
{
	(void)mem;
}

static void test_allocation_refusal( void )
{
	section( "refused allocation" );

	b3ClearOutOfMemory();
	check( "flag starts clear", !b3OutOfMemory() );

	// b3RaiseOutOfMemory ends in B3_ASSERT( false ) on purpose, so that an
	// *accidental* refusal is loud. This section causes refusals deliberately,
	// so its assertions are the behaviour under test rather than a failure.
	int expectedBefore = b3TestExpectedAsserts();
	b3TestExpectAsserts( true );

	b3SetAllocator( failingAlloc, failingFree );

	// A push whose growth is refused must drop the value rather than write
	// through a null pointer.
	b3Array( int ) arr = { 0 };
	b3Array_Push( arr, 42 );
	checkInt( "refused push left count at 0", arr.count, 0 );
	check( "refused push raised the flag", b3OutOfMemory() );
	check( "refused push left data NULL", arr.data == NULL );

	// Emplace must hand back writable scratch, not NULL, because upstream
	// call sites write through the result unchecked.
	b3ClearOutOfMemory();
	int* slot = (int*)b3EmplaceHelper( (void**)&arr.data, &arr.count, &arr.capacity, sizeof( int ) );
	check( "refused emplace returned non-NULL", slot != NULL );
	if ( slot != NULL )
	{
		// Writing here must be safe; the value is discarded.
		*slot = 1234;
	}
	checkInt( "refused emplace did not grow count", arr.count, 0 );
	check( "refused emplace raised the flag", b3OutOfMemory() );

	// Reserve must leave capacity honest, so later indexing stays in bounds.
	b3ClearOutOfMemory();
	b3Array( int ) res = { 0 };
	b3Array_CreateN( res, 32 );
	checkInt( "refused reserve reports zero capacity", res.capacity, 0 );

	// Resize must clamp count to the capacity actually obtained rather than
	// trusting the requested size.
	b3ClearOutOfMemory();
	b3Array( int ) rz = { 0 };
	b3Array_Resize( rz, 16 );
	check( "refused resize kept count within capacity", rz.count <= rz.capacity );

	// Restore the normal allocator for anything that follows.
	b3SetAllocator( NULL, NULL );
	b3ClearOutOfMemory();
	check( "flag cleared for later tests", !b3OutOfMemory() );

	b3TestExpectAsserts( false );

	// The refusals above must actually have asserted. If b3RaiseOutOfMemory
	// ever stops being loud, this section would still pass on its flag checks
	// alone while the diagnostic that makes an accidental refusal findable had
	// quietly gone away.
	check( "the refusal path asserted", b3TestExpectedAsserts() > expectedBefore );
}

static void test_arena( void )
{
	section( "arena and block allocator" );

	b3Arena arena = b3CreateArena( 4096 );

	void* a = b3Bump( &arena, 100 );
	void* b = b3Bump( &arena, 100 );
	check( "arena returns distinct blocks", a != NULL && b != NULL && a != b );
	check( "arena blocks do not overlap", (char*)b >= (char*)a + 100 );
	check( "arena honours alignment", ( (uintptr_t)b & ( B3_ARENA_ALIGNMENT - 1 ) ) == 0 );

	// Passing the arena by value is how upstream gets automatic restore on
	// return; the copy's bump pointer must not disturb the original.
	//
	// The index is captured rather than asserted against a literal, because
	// the exact value depends on B3_ARENA_ALIGNMENT padding between bumps --
	// two 100-byte allocations land at 0 and 104, not 0 and 100.
	int indexBeforeCopy = arena.index;
	b3Arena copy = arena;
	b3Bump( &copy, 500 );
	checkInt( "original index unchanged by copy", arena.index, indexBeforeCopy );
	check( "copy advanced independently", copy.index > indexBeforeCopy );

	// A request past capacity must still return usable memory. See the note
	// on b3ArenaOverflowAlloc: refusing here would crash callers, and the
	// arena grows itself on the next sync instead.
	//
	// That path also asserts, deliberately -- an overflow means the arena was
	// sized too small for the step, which is a real sizing mistake even though
	// it is recoverable. Driving it on purpose here, so the assertion is
	// counted rather than reported as a failure.
	int overflowAssertsBefore = b3TestExpectedAsserts();
	b3TestExpectAsserts( true );

	void* big = b3Bump( &arena, 8192 );
	check( "oversized bump still returns memory", big != NULL );
	if ( big != NULL )
	{
		memset( big, 0xAB, 8192 );
	}

	b3TestExpectAsserts( false );
	check( "the arena overflow path asserted", b3TestExpectedAsserts() > overflowAssertsBefore );

	b3ArenaSync( &arena );
	check( "arena grew to meet demand", arena.capacity >= 8192 );

	b3DestroyArena( &arena );

	// Block allocator: stable addresses, and freed elements are reused.
	b3BlockAllocator alloc = b3CreateBlockAllocator( 32, 4 );
	void* e1 = b3AllocateElement( &alloc );
	void* e2 = b3AllocateElement( &alloc );
	check( "block elements are distinct", e1 != e2 );

	b3FreeElement( &alloc, e1 );
	void* e3 = b3AllocateElement( &alloc );
	check( "freed block element is reused", e3 == e1 );

	// Exceed the initial block so a second one has to be created.
	for ( int i = 0; i < 100; i++ )
	{
		check( "block allocation succeeded", b3AllocateElement( &alloc ) != NULL );
		s_checks--; // counted once below instead of a hundred times
	}
	s_checks++;

	b3DestroyBlockAllocator( &alloc );
}

int main( void )
{
	b3TestInstallAssertTrap();
	printf( "box3d support layer verification\n\n" );

	test_ctz();
	test_bitset();
	test_id_pool();
	test_hash_set();
	test_containers();
	test_allocation_refusal();
	test_arena();

	// Assertions are part of the result, not a trap: assert_trap.h keeps the
	// run going and this turns any that fired into a reported failure.
	s_checks++;
	if ( b3TestUnexpectedAsserts() != 0 )
	{
		printf( "  FAIL %d unexpected assertion(s) fired\n", b3TestUnexpectedAsserts() );
		s_failures++;
	}

	printf( "\n%d checks, %d failures\n", s_checks, s_failures );
	return s_failures == 0 ? 0 : 1;
}
