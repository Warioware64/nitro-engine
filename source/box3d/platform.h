// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Erin Catto        (original Box3D)
// Copyright (c) 2026 Warioware64       (Nitro Engine Advanced fixed-point port)
//
// This file is part of Nitro Engine Advanced

#pragma once

/// @file   platform.h
/// @brief  Atomics and prefetch for a single-core target.

#include "core.h"

#include <stdbool.h>
#include <stdint.h>

// -------------------------------------------------------------------------
// Prefetch
// -------------------------------------------------------------------------

/// Software prefetch hint.
///
/// Kept, unlike the rest of this header. ARMv5TE has a real PLD instruction
/// and GCC maps __builtin_prefetch onto it, so this is a genuine win for the
/// pointer-chasing in the dynamic tree traversal rather than a no-op carried
/// over from upstream.
#if defined( B3_COMPILER_CLANG ) || defined( B3_COMPILER_GCC )
#define b3Prefetch( addr ) __builtin_prefetch( (const void*)( addr ), 0, 3 )
#else
#define b3Prefetch( addr ) ( (void)( addr ) )
#endif

// -------------------------------------------------------------------------
// Atomics
// -------------------------------------------------------------------------
//
// These are plain loads and stores, and that is correct rather than a
// shortcut. Box3D's atomics exist so worker threads can claim solver blocks
// by compare-and-swap; the port has one core, no worker threads, and the
// solver never yields inside a step, so there is no other observer for a
// barrier to order against.
//
// It is also a necessity. __atomic_compare_exchange_n on a 32-bit int lowers
// to a call into libatomic on ARMv5TE, which BlocksDS does not link -- the
// build would fail at link time with an undefined __atomic_compare_exchange_4.
//
// The counters these guard (b3_byteCount, solver block syncIndex) stay
// meaningful as ordinary integers.

static inline void b3AtomicStoreInt( b3AtomicInt* a, int value )
{
	a->value = value;
}

static inline int b3AtomicLoadInt( b3AtomicInt* a )
{
	return a->value;
}

static inline int b3AtomicFetchAddInt( b3AtomicInt* a, int increment )
{
	int old = a->value;
	a->value = old + increment;
	return old;
}

static inline bool b3AtomicCompareExchangeInt( b3AtomicInt* a, int expected, int desired )
{
	if ( a->value == expected )
	{
		a->value = desired;
		return true;
	}
	return false;
}

static inline void b3AtomicStoreU32( b3AtomicU32* a, uint32_t value )
{
	a->value = value;
}

static inline uint32_t b3AtomicLoadU32( b3AtomicU32* a )
{
	return a->value;
}
