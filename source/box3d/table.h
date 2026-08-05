// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Erin Catto        (original Box3D)
// Copyright (c) 2026 Warioware64       (Nitro Engine Advanced fixed-point port)
//
// This file is part of Nitro Engine Advanced

#pragma once

/// @file   table.h
/// @brief  Open-addressed hash set of shape pair keys.
///
/// The contact hash set is consulted once per broad-phase pair per step, so
/// its key width is on the hot path.
///
/// Upstream packs a pair key into 64 bits because it reserves 22 bits per
/// shape (4 million shapes). The port reserves 12 (4096 shapes, far more than
/// a DS scene will hold), which leaves 8 bits of child index and brings the
/// whole key to exactly 32 bits. On a 32-bit CPU that halves the compare
/// width and, more importantly, lets the hash finalizer use 32-bit multiplies
/// instead of 64-bit ones -- the latter cost several instructions each on
/// ARMv5TE.
///
/// The width is derived rather than hard-coded, so raising B3_SHAPE_POWER
/// past the point where a key fits automatically restores upstream's 64-bit
/// behaviour.

#include "box3d/constants.h"

#include <stdbool.h>
#include <stdint.h>

#if 2 * B3_SHAPE_POWER + B3_CHILD_POWER <= 32
typedef uint32_t b3ShapeKey;
#define B3_SHAPE_KEY_BITS 32
#else
typedef uint64_t b3ShapeKey;
#define B3_SHAPE_KEY_BITS 64
#endif

typedef struct b3SetItem
{
	b3ShapeKey key;
	uint32_t hash;
} b3SetItem;

typedef struct b3HashSet
{
	b3SetItem* items;
	uint32_t capacity;
	uint32_t count;
} b3HashSet;

#define B3_SHAPE_MASK ( B3_MAX_SHAPES - 1 )
#define B3_CHILD_MASK ( B3_MAX_CHILD_SHAPES - 1 )

/// Pack a shape pair and child index into one key.
///
/// The lower shape index always goes in the high field, so the key is
/// independent of the order the pair is presented in.
static inline b3ShapeKey b3ShapePairKey( int s1, int s2, int c )
{
	int lo = s1 < s2 ? s1 : s2;
	int hi = s1 < s2 ? s2 : s1;

	return ( (b3ShapeKey)( B3_SHAPE_MASK & lo ) << ( B3_SHAPE_KEY_BITS - B3_SHAPE_POWER ) ) |
		   ( (b3ShapeKey)( B3_SHAPE_MASK & hi ) << ( B3_SHAPE_KEY_BITS - 2 * B3_SHAPE_POWER ) ) |
		   ( (b3ShapeKey)( B3_CHILD_MASK & c ) );
}

b3HashSet b3CreateSet( int32_t capacity );
void b3DestroySet( b3HashSet* set );

void b3ClearSet( b3HashSet* set );

// Returns true if key was already in set
bool b3AddKey( b3HashSet* set, b3ShapeKey key );

// Returns true if the key was found
bool b3RemoveKey( b3HashSet* set, b3ShapeKey key );

bool b3ContainsKey( const b3HashSet* set, b3ShapeKey key );

int b3GetHashSetBytes( b3HashSet* set );
