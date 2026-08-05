// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Erin Catto        (original Box3D)
// Copyright (c) 2026 Warioware64       (Nitro Engine Advanced fixed-point port)
//
// This file is part of Nitro Engine Advanced

#pragma once

/// @file   id.h
/// @brief  Opaque handles to world, body, shape, joint and contact instances.
///
/// Ported from Box3D's id.h. Deliberately stand-alone: nothing here needs the
/// fixed-point layer, so a translation unit that only passes handles around
/// does not have to pull in math_fixed.h.
///
/// The layout is unchanged from upstream. It is tempting to shrink these on a
/// 4 MB console -- a b3BodyId is 8 bytes to address at most a few hundred
/// bodies -- but the index/generation pair is what makes a stale handle
/// *detectable* rather than silently aliasing a recycled slot, and that is
/// worth more than the bytes. Handles live in user code, not in arrays the
/// engine sizes.

#include <stdint.h>

/**
 * @defgroup id Ids
 * These ids serve as handles to internal Box3D objects.
 * These should be considered opaque data and passed by value.
 * All ids are considered null if initialized to zero.
 *
 * @warning Do not use the internals of these ids. They are subject to change.
 * @{
 */

/// World id references a world instance. This should be treated as an opaque handle.
///
/// B3_MAX_WORLDS is 1 in this port, so index1 is always 0 or 1. The field is
/// kept at its upstream width rather than dropped: b3StoreWorldId round trips
/// through a uint32_t and user code may hold one.
typedef struct b3WorldId
{
	uint16_t index1;
	uint16_t generation;
} b3WorldId;

/// Body id references a body instance. This should be treated as an opaque handle.
typedef struct b3BodyId
{
	int32_t index1;
	uint16_t world0;
	uint16_t generation;
} b3BodyId;

/// Shape id references a shape instance. This should be treated as an opaque handle.
typedef struct b3ShapeId
{
	int32_t index1;
	uint16_t world0;
	uint16_t generation;
} b3ShapeId;

/// Joint id references a joint instance. This should be treated as an opaque handle.
typedef struct b3JointId
{
	int32_t index1;
	uint16_t world0;
	uint16_t generation;
} b3JointId;

/// Contact id references a contact instance. This should be treated as an opaque handle.
typedef struct b3ContactId
{
	int32_t index1;
	uint16_t world0;
	int16_t padding;
	uint32_t generation;
} b3ContactId;

// clang-format off

/// A null id. Works for any id type.
#define B3_NULL_ID { 0 }

/// Bridges C and C++ inline functions upstream. C only here.
#define B3_ID_INLINE static inline

// clang-format on

/// Use these to make your identifiers null.
/// You may also use zero initialization to get null.
static const b3WorldId b3_nullWorldId = B3_NULL_ID;
static const b3BodyId b3_nullBodyId = B3_NULL_ID;
static const b3ShapeId b3_nullShapeId = B3_NULL_ID;
static const b3JointId b3_nullJointId = B3_NULL_ID;
static const b3ContactId b3_nullContactId = B3_NULL_ID;

/// Macro to determine if any id is null.
#define B3_IS_NULL( id ) ( ( id ).index1 == 0 )

/// Macro to determine if any id is non-null.
#define B3_IS_NON_NULL( id ) ( ( id ).index1 != 0 )

/// Compare two ids for equality. Doesn't work for b3WorldId. Don't mix types.
#define B3_ID_EQUALS( id1, id2 )                                                                                                 \
	( ( id1 ).index1 == ( id2 ).index1 && ( id1 ).world0 == ( id2 ).world0 && ( id1 ).generation == ( id2 ).generation )

/// Store a world id into a uint32_t.
B3_ID_INLINE uint32_t b3StoreWorldId( b3WorldId id )
{
	return ( (uint32_t)id.index1 << 16 ) | (uint32_t)id.generation;
}

/// Load a uint32_t into a world id.
B3_ID_INLINE b3WorldId b3LoadWorldId( uint32_t x )
{
	b3WorldId id = { (uint16_t)( x >> 16 ), (uint16_t)( x ) };
	return id;
}

/// Store a body id into a uint64_t.
B3_ID_INLINE uint64_t b3StoreBodyId( b3BodyId id )
{
	return ( (uint64_t)id.index1 << 32 ) | ( (uint64_t)id.world0 ) << 16 | (uint64_t)id.generation;
}

/// Load a uint64_t into a body id.
B3_ID_INLINE b3BodyId b3LoadBodyId( uint64_t x )
{
	b3BodyId id = { (int32_t)( x >> 32 ), (uint16_t)( x >> 16 ), (uint16_t)( x ) };
	return id;
}

/// Store a shape id into a uint64_t.
B3_ID_INLINE uint64_t b3StoreShapeId( b3ShapeId id )
{
	return ( (uint64_t)id.index1 << 32 ) | ( (uint64_t)id.world0 ) << 16 | (uint64_t)id.generation;
}

/// Load a uint64_t into a shape id.
B3_ID_INLINE b3ShapeId b3LoadShapeId( uint64_t x )
{
	b3ShapeId id = { (int32_t)( x >> 32 ), (uint16_t)( x >> 16 ), (uint16_t)( x ) };
	return id;
}

/// Store a joint id into a uint64_t.
B3_ID_INLINE uint64_t b3StoreJointId( b3JointId id )
{
	return ( (uint64_t)id.index1 << 32 ) | ( (uint64_t)id.world0 ) << 16 | (uint64_t)id.generation;
}

/// Load a uint64_t into a joint id.
B3_ID_INLINE b3JointId b3LoadJointId( uint64_t x )
{
	b3JointId id = { (int32_t)( x >> 32 ), (uint16_t)( x >> 16 ), (uint16_t)( x ) };
	return id;
}

/// Store a contact id into three uint32 values
B3_ID_INLINE void b3StoreContactId( b3ContactId id, uint32_t values[3] )
{
	values[0] = (uint32_t)id.index1;
	values[1] = (uint32_t)id.world0;
	values[2] = (uint32_t)id.generation;
}

/// Load a contact id from three uint32 values.
B3_ID_INLINE b3ContactId b3LoadContactId( uint32_t values[3] )
{
	b3ContactId id;
	id.index1 = (int32_t)values[0];
	id.world0 = (uint16_t)values[1];
	id.padding = 0;
	id.generation = (uint32_t)values[2];
	return id;
}

/**@}*/
