// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Erin Catto        (original Box3D)
// Copyright (c) 2026 Warioware64       (Nitro Engine Advanced fixed-point port)
//
// This file is part of Nitro Engine Advanced

#pragma once

/// @file   container.h
/// @brief  Type-safe dynamic arrays.
///
/// Structurally identical to upstream. What changes is the behaviour when a
/// growth is refused, which upstream never has to consider because it can
/// always malloc.
///
/// @section refusal Refused growth
///
/// The port pre-reserves every array when the world is created, so reaching a
/// growth path during simulation means a budget was set too low. Under
/// NEA_DEBUG that asserts. In release, b3GrowAlloc returns NULL and raises
/// the out-of-memory flag.
///
/// NULL cannot simply be propagated, though: these macros assign the result
/// and then immediately write through it, so a refusal would become a null
/// dereference -- turning a recoverable budget mistake into a crash, which is
/// precisely backwards.
///
/// So the two paths differ:
///
///   - Macros that only append (Push, Append, Resize) check for NULL and
///     leave the array untouched. The value is dropped.
///
///   - b3Array_Emplace returns a pointer that hundreds of upstream call sites
///     use without checking. It returns a scratch element instead, which the
///     caller may write freely and which is then discarded.
///
/// Either way the write lands somewhere valid, the flag is set, and the
/// public creation APIs turn that into a null id for the caller. A world that
/// exhausts its budget stops growing rather than dying.

#include "algorithm.h"
#include "core.h"

#include <stddef.h>
#include <string.h>

#define b3DeclareArray( T )                                                                                                      \
	typedef struct b3DynamicArray_##T                                                                                            \
	{                                                                                                                            \
		struct T* data;                                                                                                          \
		int count;                                                                                                               \
		int capacity;                                                                                                            \
	} b3DynamicArray_##T

#define b3DeclareArrayNative( T )                                                                                                \
	typedef struct b3DynamicArray_##T                                                                                            \
	{                                                                                                                            \
		T* data;                                                                                                                 \
		int count;                                                                                                               \
		int capacity;                                                                                                            \
	} b3DynamicArray_##T

// Define an array.
// It may be zero initialized:
// b3Array(int) myArray = { 0 };
#define b3Array( T ) b3DynamicArray_##T

// Alternative to zero initialization
#define b3Array_Create( a )                                                                                                      \
	do                                                                                                                           \
	{                                                                                                                            \
		( a ).data = NULL;                                                                                                       \
		( a ).count = 0;                                                                                                         \
		( a ).capacity = 0;                                                                                                      \
	}                                                                                                                            \
	while ( 0 )

/// Create with a reserved capacity.
///
/// This is the constructor the port actually uses: NEA_Phys3DWorldCreate
/// sizes every array here, once, so nothing below ever has to grow.
#define b3Array_CreateN( a, n )                                                                                                  \
	do                                                                                                                           \
	{                                                                                                                            \
		( a ).data = ( n ) > 0 ? b3GrowAlloc( NULL, 0, ( n ) * sizeof( *( a ).data ) ) : NULL;                                   \
		( a ).count = 0;                                                                                                         \
		( a ).capacity = ( a ).data != NULL ? ( n ) : 0;                                                                         \
	}                                                                                                                            \
	while ( 0 )

#define b3Array_Destroy( a )                                                                                                     \
	do                                                                                                                           \
	{                                                                                                                            \
		b3Free( ( a ).data, ( a ).capacity * sizeof( *( a ).data ) );                                                            \
		( a ).data = NULL;                                                                                                       \
		( a ).count = 0;                                                                                                         \
		( a ).capacity = 0;                                                                                                      \
	}                                                                                                                            \
	while ( 0 )

#define b3Array_Reserve( a, n )                                                                                                  \
	do                                                                                                                           \
	{                                                                                                                            \
		if ( ( a ).capacity < ( n ) )                                                                                            \
		{                                                                                                                        \
			int oldSize = ( a ).capacity * (int)sizeof( *( a ).data );                                                           \
			int newSize = ( n ) * (int)sizeof( *( a ).data );                                                                    \
			void* newData = b3GrowAlloc( ( a ).data, oldSize, newSize );                                                         \
			if ( newData != NULL )                                                                                               \
			{                                                                                                                    \
				( a ).data = newData;                                                                                            \
				( a ).capacity = ( n );                                                                                          \
			}                                                                                                                    \
		}                                                                                                                        \
	}                                                                                                                            \
	while ( 0 )

/// Resize, clamping the count to whatever capacity was actually obtained.
///
/// The clamp matters: if the reserve above was refused, setting count past
/// capacity would leave every subsequent index out of bounds.
#define b3Array_Resize( a, n )                                                                                                   \
	do                                                                                                                           \
	{                                                                                                                            \
		b3Array_Reserve( a, n );                                                                                                 \
		( a ).count = ( n ) < ( a ).capacity ? ( n ) : ( a ).capacity;                                                           \
	}                                                                                                                            \
	while ( 0 )

// Push a new element by value. Dropped if growth is refused.
#define b3Array_Push( a, value )                                                                                                 \
	do                                                                                                                           \
	{                                                                                                                            \
		if ( ( a ).count >= ( a ).capacity )                                                                                     \
		{                                                                                                                        \
			int oldSize = ( a ).capacity * (int)sizeof( *( a ).data );                                                           \
			int newCapacity = ( a ).capacity == 0 ? 8 : 2 * ( a ).capacity;                                                      \
			int newSize = newCapacity * (int)sizeof( *( a ).data );                                                              \
			void* newData = b3GrowAlloc( ( a ).data, oldSize, newSize );                                                         \
			if ( newData == NULL )                                                                                               \
			{                                                                                                                    \
				break;                                                                                                           \
			}                                                                                                                    \
			( a ).data = newData;                                                                                                \
			( a ).capacity = newCapacity;                                                                                        \
		}                                                                                                                        \
		( a ).data[( a ).count++] = ( value );                                                                                   \
	}                                                                                                                            \
	while ( 0 )

// Get a pointer to an element
#define b3Array_Get( a, index ) ( B3_ASSERT( 0 <= ( index ) && ( index ) < ( a ).count ), ( a ).data + ( index ) )

// Create a new uninitialized element and return a pointer to it
#define b3Array_Emplace( a )                                                                                                     \
	( b3EmplaceHelper( (void**)&( a ).data, &( a ).count, &( a ).capacity, sizeof( *( a ).data ) ) )

// Remove the last element and return it by value.
#define b3Array_Pop( a ) ( B3_ASSERT( 0 < ( a ).count ), ( a ).data[-1 + ( a ).count--] )

// Add an uninitialized element and return its index.
#define b3Array_AddIndex( a )                                                                                                    \
	( b3EmplaceHelper( (void**)&( a ).data, &( a ).count, &( a ).capacity, sizeof( *( a ).data ) ), ( a ).count - 1 )

// Append a contiguous run of values. _n caches the input count while avoiding naming conflicts.
#define b3Array_Append( a, src, n )                                                                                              \
	do                                                                                                                           \
	{                                                                                                                            \
		int _n = ( n );                                                                                                          \
		if ( ( a ).count + _n > ( a ).capacity )                                                                                 \
		{                                                                                                                        \
			int req = ( a ).count + _n;                                                                                          \
			int newCapacity = req > 2 ? req + ( req >> 1 ) : 8;                                                                  \
			int oldSize = ( a ).capacity * (int)sizeof( *( a ).data );                                                           \
			int newSize = newCapacity * (int)sizeof( *( a ).data );                                                              \
			void* newData = b3GrowAlloc( ( a ).data, oldSize, newSize );                                                         \
			if ( newData == NULL )                                                                                               \
			{                                                                                                                    \
				break;                                                                                                           \
			}                                                                                                                    \
			( a ).data = newData;                                                                                                \
			( a ).capacity = newCapacity;                                                                                        \
		}                                                                                                                        \
		memcpy( ( a ).data + ( a ).count, ( src ), _n * sizeof( *( a ).data ) );                                                 \
		( a ).count += _n;                                                                                                       \
	}                                                                                                                            \
	while ( 0 )

// Zero the entire allocated buffer (capacity, not just count).
#define b3Array_MemZero( a )                                                                                                     \
	do                                                                                                                           \
	{                                                                                                                            \
		if ( ( a ).capacity > 0 )                                                                                                \
		{                                                                                                                        \
			memset( ( a ).data, 0, ( a ).capacity * sizeof( *( a ).data ) );                                                     \
		}                                                                                                                        \
	}                                                                                                                            \
	while ( 0 )

// Remove an element by swapping with the last element. If the index is the last element it returns
// B3_NULL_INDEX, otherwise it returns the index of the last element (which is now out of bounds).
#define b3Array_RemoveSwap( a, index ) b3RemoveHelper( ( a ).data, &( a ).count, ( index ), sizeof( *( a ).data ) )

/// Largest element b3Array_Emplace can hand back when growth is refused.
///
/// Comfortably above the biggest thing the library emplaces (b3BodySim and
/// b3ContactConstraint are the largest, both well under this). The assert in
/// b3EmplaceHelper catches it if that ever stops being true.
#define B3_EMPLACE_SCRATCH_SIZE 512

/// Discard target for refused emplacements. See the note at the top of the
/// file: returning NULL here would crash call sites that upstream wrote
/// against an allocator that cannot fail.
extern char b3_emplaceScratch[B3_EMPLACE_SCRATCH_SIZE];

B3_INLINE void* b3EmplaceHelper( void** data, int* count, int* capacity, int elem_size )
{
	if ( *count >= *capacity )
	{
		int oldCapacity = *capacity;
		int oldSize = oldCapacity * elem_size;
		int newCapacity = ( oldCapacity == 0 ? 16 : 2 * oldCapacity );
		int newSize = newCapacity * elem_size;

		void* newData = b3GrowAlloc( *data, oldSize, newSize );
		if ( newData == NULL )
		{
			// Growth refused. Hand back scratch so the caller's writes land
			// somewhere valid; the element is never added, and the
			// out-of-memory flag b3GrowAlloc raised is what the creation API
			// reports to the user.
			B3_ASSERT( elem_size <= B3_EMPLACE_SCRATCH_SIZE );
			return b3_emplaceScratch;
		}

		*data = newData;
		*capacity = newCapacity;
	}
	return (char*)*data + ( *count )++ * elem_size;
}

B3_INLINE int b3RemoveHelper( void* data, int* count, int index, int elementSize )
{
	B3_ASSERT( 0 <= index && index < *count && "Array index out of bounds" );

	( *count )--;
	if ( index != *count )
	{
		memcpy( (char*)data + index * elementSize, (char*)data + ( *count ) * elementSize, elementSize );
		return *count;
	}

	return B3_NULL_INDEX;
}

#define b3Array_Clear( a )                                                                                                       \
	do                                                                                                                           \
	{                                                                                                                            \
		( a ).count = 0;                                                                                                         \
	}                                                                                                                            \
	while ( 0 )

#define b3Array_ByteCount( a ) ( ( a ).capacity * (int)sizeof( *( a ).data ) )

b3DeclareArrayNative( int );
