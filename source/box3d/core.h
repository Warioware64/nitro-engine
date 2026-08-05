// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Erin Catto        (original Box3D)
// Copyright (c) 2026 Warioware64       (Nitro Engine Advanced fixed-point port)
//
// This file is part of Nitro Engine Advanced

#pragma once

/// @file   core.h
/// @brief  Internal allocation, logging and platform definitions.
///
/// Upstream's platform/CPU/SIMD detection block is gone: the target is known,
/// singular, and has no SIMD unit. What remains is the allocation contract
/// and the small set of macros the rest of the library leans on.

#include "box3d/base.h"

#include <stddef.h>

// The only platform. Kept as a define because a few ported files test it.
#define B3_PLATFORM_NDS
#define B3_CPU_ARM
#define B3_SIMD_NONE

#if defined( __clang__ )
#define B3_COMPILER_CLANG
#elif defined( __GNUC__ )
#define B3_COMPILER_GCC
#endif

#if defined( NEA_DEBUG )
#define B3_DEBUG 1
#else
#define B3_DEBUG 0
#endif

// Tracy profiler instrumentation. There is no Tracy on a DS.
// clang-format off
#define b3TracyCZoneC( ctx, color, active )
#define b3TracyCZoneNC( ctx, name, color, active )
#define b3TracyCZoneEnd( ctx )
#define b3TracyCFrame
#define b3TracyCAlloc( ptr, size )
#define b3TracyCFree( ptr )
// clang-format on

/// Single-core, and the solver never yields inside a step, so the atomic
/// types are plain values. See platform.h for why that is correct here rather
/// than merely expedient.
typedef struct b3AtomicInt
{
	int value;
} b3AtomicInt;

typedef struct b3AtomicU32
{
	uint32_t value;
} b3AtomicU32;

/// Minimum memory alignment used for all allocations.
///
/// Upstream uses 16 for SSE2. ARM9 needs 8 -- that is what LDRD/STRD require,
/// and nothing here is wider. Halving it saves up to 8 bytes on each of
/// several hundred allocations, which is worth having when the whole machine
/// has 4 MB.
#define B3_ALIGNMENT 8

/// Returns the number of elements of an array.
#define B3_ARRAY_COUNT( A ) (int)( sizeof( A ) / sizeof( A[0] ) )

/// Used to prevent the compiler from warning about unused variables.
///
/// Upstream spells this `(void)sizeof((__VA_ARGS__, 0))`, which under -Wall
/// warns about the comma operand having no effect -- the very warning it
/// exists to suppress. Casting each argument to void individually says the
/// same thing without the comma expression. Arities up to six are used.
// clang-format off
#define B3_UNUSED_1( a ) ( (void)( a ) )
#define B3_UNUSED_2( a, b ) ( (void)( a ), (void)( b ) )
#define B3_UNUSED_3( a, b, c ) ( (void)( a ), (void)( b ), (void)( c ) )
#define B3_UNUSED_4( a, b, c, d ) ( (void)( a ), (void)( b ), (void)( c ), (void)( d ) )
#define B3_UNUSED_5( a, b, c, d, e ) ( (void)( a ), (void)( b ), (void)( c ), (void)( d ), (void)( e ) )
#define B3_UNUSED_6( a, b, c, d, e, f ) ( (void)( a ), (void)( b ), (void)( c ), (void)( d ), (void)( e ), (void)( f ) )

#define B3_UNUSED_PICK( _1, _2, _3, _4, _5, _6, NAME, ... ) NAME
#define B3_UNUSED( ... )                                                                                                         \
	B3_UNUSED_PICK( __VA_ARGS__, B3_UNUSED_6, B3_UNUSED_5, B3_UNUSED_4, B3_UNUSED_3, B3_UNUSED_2, B3_UNUSED_1 )( __VA_ARGS__ )
// clang-format on

/// Used to validate definitions came from the b3Default*Def constructors.
#define B3_SECRET_COOKIE 1152023

#define B3_CHECK_DEF( DEF ) B3_ASSERT( DEF->internalValue == B3_SECRET_COOKIE )
#define B3_CHECK_JOINT_DEF( DEF ) B3_ASSERT( DEF->base.internalValue == B3_SECRET_COOKIE )

// These macros help avoid sizeof bugs
#define B3_ALLOC( T, N ) (T*)b3Alloc( N * sizeof( T ) );
#define B3_FREE( M, T, N ) b3Free( M, N * sizeof( T ) );

void* b3Alloc( size_t size );
void* b3AllocZeroed( size_t size );
void b3Free( void* mem, size_t size );

/// Grow an allocation, copying the old contents.
///
/// This is the single choke point for every dynamic array in the library --
/// b3Array_Push, _Reserve, _Resize and _CreateN all route through it -- which
/// is what makes the pre-reserved memory model enforceable in one place.
///
/// The port's world reserves all its capacity up front, so reaching here
/// during a step means a budget was set too low:
///
///   - NEA_DEBUG: asserts, naming the size that did not fit.
///   - Release:   returns NULL and raises the out-of-memory flag. Callers
///                that can degrade gracefully check b3OutOfMemory().
///
/// @return New memory, or NULL if the allocation was refused.
void* b3GrowAlloc( void* oldMem, int oldSize, int newSize );

/// True if any allocation has been refused since the flag was last cleared.
///
/// Entity creation checks this and returns a null id rather than proceeding
/// with a half-built object, so a world that runs out of room stops growing
/// instead of corrupting itself.
bool b3OutOfMemory( void );

/// Clear the out-of-memory flag. Called when a world is created.
void b3ClearOutOfMemory( void );

/// Record that an allocation was refused. Exposed so the arena and block
/// allocators, which do not go through b3GrowAlloc, can raise the same flag.
void b3RaiseOutOfMemory( const char* what, int size );

B3_PRINTF_FORMAT( 1, 2 )
void b3Log( const char* format, ... );

/// Geometry content hashes reserve zero to mean unhashed.
static inline uint32_t b3NonZeroHash( uint32_t hash )
{
	return hash != 0 ? hash : 1;
}

// -------------------------------------------------------------------------
// Threading
// -------------------------------------------------------------------------
//
// Declared so the ported sources that mention them still compile, and
// implemented as no-ops. There is one core; the scheduler and parallel-for
// are not part of the port.

typedef struct b3Mutex b3Mutex;
b3Mutex* b3CreateMutex( void );
void b3DestroyMutex( b3Mutex* m );
void b3LockMutex( b3Mutex* m );
void b3UnlockMutex( b3Mutex* m );

void b3StrCpy( char* dst, int size, const char* src );
