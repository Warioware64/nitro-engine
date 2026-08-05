// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Erin Catto        (original Box3D)
// Copyright (c) 2026 Warioware64       (Nitro Engine Advanced fixed-point port)
//
// This file is part of Nitro Engine Advanced

#pragma once

/// @file   ctz.h
/// @brief  Bit-scan and population-count helpers.
///
/// The 64-bit forms are hand-written rather than left as GCC builtins.
/// Verified on the actual toolchain: on ARMv5TE, __builtin_ctzll emits a call
/// to __ctzdi2 and __builtin_popcountll a call to __popcountdi2. Both live in
/// libgcc so they link, but these sit in the bitset iteration loop that
/// island traversal and the awake-set scan run over every step, and a
/// function call there is pure overhead when the operation is four
/// instructions of inline code.
///
/// The 32-bit forms are left as builtins: __builtin_clz maps straight onto
/// ARMv5TE's CLZ instruction, and __builtin_ctz to the standard
/// negate-and-CLZ sequence, both inline.

#include "core.h"

#include <stdbool.h>
#include <stdint.h>

static inline uint32_t b3CTZ32( uint32_t block )
{
	return __builtin_ctz( block );
}

static inline uint32_t b3CLZ32( uint32_t value )
{
	return __builtin_clz( value );
}

/// Count trailing zeros of a 64-bit block.
///
/// Undefined for zero, matching the builtin it replaces. Callers reach this
/// only after testing that a bitset block is non-zero.
static inline uint32_t b3CTZ64( uint64_t block )
{
	uint32_t low = (uint32_t)block;
	if ( low != 0 )
	{
		return b3CTZ32( low );
	}
	return 32 + b3CTZ32( (uint32_t)( block >> 32 ) );
}

/// Population count of a 32-bit word.
///
/// ARMv5TE has no popcount instruction, so this is the usual SWAR reduction:
/// branch-free, no memory access, and no call.
static inline int b3PopCount32( uint32_t x )
{
	x = x - ( ( x >> 1 ) & 0x55555555u );
	x = ( x & 0x33333333u ) + ( ( x >> 2 ) & 0x33333333u );
	x = ( x + ( x >> 4 ) ) & 0x0f0f0f0fu;
	return (int)( ( x * 0x01010101u ) >> 24 );
}

static inline int b3PopCount64( uint64_t block )
{
	return b3PopCount32( (uint32_t)block ) + b3PopCount32( (uint32_t)( block >> 32 ) );
}

static inline bool b3IsPowerOf2( int x )
{
	return ( x & ( x - 1 ) ) == 0;
}

static inline int b3BoundingPowerOf2( int x )
{
	if ( x <= 1 )
	{
		return 1;
	}

	return 32 - (int)b3CLZ32( (uint32_t)x - 1 );
}

static inline int b3RoundUpPowerOf2( int x )
{
	if ( x <= 1 )
	{
		return 1;
	}

	return 1 << ( 32 - (int)b3CLZ32( (uint32_t)x - 1 ) );
}

static inline int b3LowerPowerOf2Exponent( int x )
{
	B3_ASSERT( x > 0 );
	int clz = (int)b3CLZ32( (uint32_t)x );

	// Position of most significant bit = floor(log2(M))
	return 31 - clz;
}
