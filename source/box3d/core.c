// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Erin Catto        (original Box3D)
// Copyright (c) 2026 Warioware64       (Nitro Engine Advanced fixed-point port)
//
// This file is part of Nitro Engine Advanced

#include "core.h"

#include "container.h"
#include "platform.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Logging and assertions reach the outside world differently on each target.
// Routing them through b3Log rather than including NEAGeneral.h directly is
// what keeps the whole box3d tree buildable on the host, where nds.h does not
// exist.
#ifdef __NDS__
#include <nds.h>

#include "NEAGeneral.h"
#endif

// =========================================================================
// Allocation
// =========================================================================

static b3AllocFcn* b3_allocFcn = NULL;
static b3FreeFcn* b3_freeFcn = NULL;

b3AtomicInt b3_byteCount;

// Raised when an allocation is refused. A world that trips this stops
// accepting new entities rather than continuing with a half-built one.
static bool b3_outOfMemory = false;

void b3SetAllocator( b3AllocFcn* allocFcn, b3FreeFcn* freeFcn )
{
	b3_allocFcn = allocFcn;
	b3_freeFcn = freeFcn;
}

bool b3OutOfMemory( void )
{
	return b3_outOfMemory;
}

void b3ClearOutOfMemory( void )
{
	b3_outOfMemory = false;
}

void b3RaiseOutOfMemory( const char* what, int size )
{
	b3_outOfMemory = true;
	b3Log( "out of memory: %s needed %d bytes", what, size );
	B3_ASSERT( false );
}

void* b3Alloc( size_t size )
{
	if ( size == 0 )
	{
		return NULL;
	}

	int alignedSize = ( ( (int)size - 1 ) | ( B3_ALIGNMENT - 1 ) ) + 1;

	void* ptr;

	if ( b3_allocFcn != NULL )
	{
		ptr = b3_allocFcn( alignedSize, B3_ALIGNMENT );
	}
	else
	{
		// No pool installed yet. This happens during world construction,
		// before NEA_Phys3DWorldCreate has its arena, and on the host where
		// the tests run against plain malloc.
		//
		// BlocksDS provides aligned_alloc, but malloc already returns
		// 8-byte-aligned memory on ARM EABI, so the plain form is enough for
		// B3_ALIGNMENT of 8 and avoids a second allocator path.
		ptr = malloc( alignedSize );
	}

	if ( ptr == NULL )
	{
		b3RaiseOutOfMemory( "b3Alloc", alignedSize );
		return NULL;
	}

	B3_ASSERT( ( (uintptr_t)ptr & ( B3_ALIGNMENT - 1 ) ) == 0 );

	b3AtomicFetchAddInt( &b3_byteCount, alignedSize );
	return ptr;
}

void* b3AllocZeroed( size_t size )
{
	void* mem = b3Alloc( size );
	if ( mem != NULL )
	{
		memset( mem, 0, size );
	}
	return mem;
}

void b3Free( void* mem, size_t size )
{
	if ( mem == NULL )
	{
		return;
	}

	if ( b3_freeFcn != NULL )
	{
		b3_freeFcn( mem );
	}
	else
	{
		free( mem );
	}

	b3AtomicFetchAddInt( &b3_byteCount, -(int)size );
}

void* b3GrowAlloc( void* oldMem, int oldSize, int newSize )
{
	B3_ASSERT( newSize > oldSize );

	void* newMem = b3Alloc( newSize );
	if ( newMem == NULL )
	{
		// b3Alloc has already raised the flag and asserted. Returning the old
		// pointer would be worse than returning NULL: the caller would write
		// past the end of a buffer it believes was grown.
		return NULL;
	}

	if ( oldSize > 0 )
	{
		memcpy( newMem, oldMem, oldSize );
		b3Free( oldMem, oldSize );
	}

	return newMem;
}

int b3GetByteCount( void )
{
	return b3AtomicLoadInt( &b3_byteCount );
}

// =========================================================================
// Diagnostics
// =========================================================================

static void b3DefaultLogFcn( const char* message )
{
#ifdef __NDS__
	// NEA_DebugPrint forwards to whatever handler the game installed with
	// NEA_DebugSetHandler. It expands to nothing outside NEA_DEBUG builds,
	// which is NEA's convention for all its diagnostics -- so on a release
	// device build this log is silent by design. The functional half of the
	// out-of-memory contract is the flag and the null return, not the
	// message.
	NEA_DebugPrint( "box3d: %s", message );
	(void)message;
#else
	fprintf( stderr, "box3d: %s\n", message );
#endif
}

b3LogFcn* b3LogHandler = b3DefaultLogFcn;

void b3SetLogFcn( b3LogFcn* logFcn )
{
	B3_ASSERT( logFcn != NULL );
	b3LogHandler = logFcn;
}

void b3Log( const char* format, ... )
{
	va_list args;
	va_start( args, format );

	// Upstream uses 512 bytes. 256 matches what NEA's own debug macros
	// allocate on the stack, and the DS stack is not large.
	char buffer[256];
	vsnprintf( buffer, sizeof( buffer ), format, args );
	va_end( args );

	b3LogHandler( buffer );
}

#if defined( NEA_DEBUG ) || defined( B3_ENABLE_ASSERT )

static int b3DefaultAssertFcn( const char* condition, const char* fileName, int lineNumber )
{
	b3Log( "ASSERTION FAILED: %s, %s, line %d", condition, fileName, lineNumber );

	// Non-zero breaks to the debugger.
	return 1;
}

b3AssertFcn* b3AssertHandler = b3DefaultAssertFcn;

void b3SetAssertFcn( b3AssertFcn* assertFcn )
{
	B3_ASSERT( assertFcn != NULL );
	b3AssertHandler = assertFcn;
}

int b3InternalAssert( const char* condition, const char* fileName, int lineNumber )
{
	int result = b3AssertHandler( condition, fileName, lineNumber );
	if ( result )
	{
		B3_BREAKPOINT;
	}
	return result;
}

#else

void b3SetAssertFcn( b3AssertFcn* assertFcn )
{
	(void)assertFcn;
}

#endif

// =========================================================================
// Threading stubs
// =========================================================================
//
// One core, and no scheduler in the port. These exist because ported sources
// still name them; every one is a no-op.

struct b3Mutex
{
	int unused;
};

static struct b3Mutex b3_theMutex;

b3Mutex* b3CreateMutex( void )
{
	return &b3_theMutex;
}

void b3DestroyMutex( b3Mutex* m )
{
	(void)m;
}

void b3LockMutex( b3Mutex* m )
{
	(void)m;
}

void b3UnlockMutex( b3Mutex* m )
{
	(void)m;
}

// =========================================================================
// Misc
// =========================================================================

b3Version b3GetVersion( void )
{
	return (b3Version){ 0, 1, 0 };
}

uint32_t b3Hash( uint32_t hash, const uint8_t* data, int count )
{
	uint32_t result = hash;
	for ( int i = 0; i < count; i++ )
	{
		result = ( result << 5 ) + result + data[i];
	}
	return result;
}

void b3StrCpy( char* dst, int size, const char* src )
{
	B3_ASSERT( size > 0 );

	if ( src != NULL )
	{
		strncpy( dst, src, size - 1 );
		dst[size - 1] = 0;
	}
	else
	{
		memset( dst, 0, size );
	}
}

// Discard target for refused emplacements. Declared in container.h; see the
// refusal note at the top of that file for why this exists rather than a NULL
// return.
char b3_emplaceScratch[B3_EMPLACE_SCRATCH_SIZE];
