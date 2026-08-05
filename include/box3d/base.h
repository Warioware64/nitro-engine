// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Erin Catto        (original Box3D)
// Copyright (c) 2026 Warioware64       (Nitro Engine Advanced fixed-point port)
//
// This file is part of Nitro Engine Advanced

#pragma once

/// @file   base.h
/// @brief  Core macros, assertions and allocation hooks.
///
/// Ported from Box3D's base.h. The shared-library export machinery, the
/// timing API and the C++ interop macros are gone -- this builds as a static
/// library into libNEA.a, and the pieces they served do not exist here.

#include <stdbool.h>
#include <stdint.h>

// The port's compile-time configuration.
//
// Upstream Box3D leaves BOX3D_USER_CONFIG undefined by default and falls back
// to its own config.h. This port has no such fallback: nea_config.h is what
// sets B3_MAX_WORLDS to 1, selects fixed point, and fixes the timestep, and
// every one of those changes a struct layout. A game that compiled without it
// would agree with libNEA_box3d.a about nothing.
//
// So it defaults to on. Makefile.blocksds still passes it explicitly, and a
// project may override it to point somewhere else, but the common case -- a
// game that includes <box3d/box3d.h> with nothing but -I on the installed
// headers -- gets the same configuration the library was built with.
#ifndef BOX3D_USER_CONFIG
#define BOX3D_USER_CONFIG "box3d/nea_config.h"
#endif

#include BOX3D_USER_CONFIG

// clang-format off

// Static library only. Upstream's dllexport/dllimport/visibility selection has
// nothing to choose between here.
#define B3_API
#define B3_INLINE static inline

#if defined( __GNUC__ ) || defined( __clang__ )
	#define B3_FORCE_INLINE static inline __attribute__((always_inline))
#else
	#define B3_FORCE_INLINE static inline
#endif

// Places a function in ITCM, the ARM9's 32 KB zero-wait-state instruction
// memory. Reserved for b3hot.c -- the handful of leaf helpers that every joint
// and contact path calls, where one shared copy in ITCM beats one copy per
// caller competing for the 8 KB instruction cache.
//
// ITCM_FUNC rather than libnds's ITCM_CODE, which the BlocksDS tutorial also
// recommends: ITCM_CODE carries __long_call__, forcing every call through a
// literal pool. ITCM sits at 0x01000000 and .text just past 0x02000000, 17 MB
// apart and well inside the +/-32 MB range of a direct BL, so the indirection
// would buy nothing.
//
// ITCM_FUNC comes from <nds.h>, which base.h does not include -- b3fixed.h
// does. That is fine because a macro body is only expanded where it is used,
// and every use is in b3hot.c, which reaches nds.h through math_fixed.h. Off
// device the macro vanishes and the definitions are ordinary functions, which
// is what the host test build in tests/box3d_host/ compiles.
//
// B3_NO_ITCM is the opt-out, wired to NEA_BOX3D_NO_ITCM in Makefile.blocksds,
// for a game that has already budgeted ITCM for its own code.
#if defined( __NDS__ ) && !defined( B3_NO_ITCM )
	#define B3_ITCM( NAME ) ITCM_FUNC( NAME )
#else
	#define B3_ITCM( NAME ) NAME
#endif

// Per-group ITCM placement, for the solver code that is too large to place
// unconditionally.
//
// FLAG is one of the B3_ITCM_* switches in nea_config.h, each 0 or 1, each
// overridable by -DB3_ITCM_<GROUP>=<0|1> from Makefile.blocksds. Usage:
//
//     void B3_ITCM_IF( B3_ITCM_WHEEL, b3SolveWheelJoint )( ... )
//
// Groups exist because ITCM is one 32 KB budget shared with the whole game and
// b3SolveJoint's type switch drags every joint implementation into every ROM
// that uses joints at all -- so a ragdoll would otherwise pay 14 KB for a wheel
// solver it never calls. Which groups are worth their bytes depends on the
// scene, which only the game knows. `make itcm-report` prints the costs.
//
// The two-level indirection is required rather than stylistic: FLAG is an
// operand of ## inside B3_ITCM_EXPAND and so is not macro-expanded there. Going
// through B3_ITCM_IF first gets FLAG expanded to its 0 or 1 as an ordinary
// argument, and only then pasted.
//
// Guarding B3_ITCM_IF_1 rather than each group keeps B3_NO_ITCM as the single
// master switch, and means the host build in tests/box3d_host/ needs no
// changes -- __NDS__ is undefined there, so every group compiles away.
#if defined( __NDS__ ) && !defined( B3_NO_ITCM )
	#define B3_ITCM_IF_1( NAME ) ITCM_FUNC( NAME )
#else
	#define B3_ITCM_IF_1( NAME ) NAME
#endif
#define B3_ITCM_IF_0( NAME ) NAME
#define B3_ITCM_EXPAND( FLAG, NAME ) B3_ITCM_IF_##FLAG( NAME )
#define B3_ITCM_IF( FLAG, NAME ) B3_ITCM_EXPAND( FLAG, NAME )

#define B3_ALIGN_AS(N) _Alignas(N)

/// Used for C literals like (b3Vec3){1, 2, 3}.
#define B3_LITERAL(T) (T)
#define B3_ZERO_INIT {0}

// This is used to validate arguments for functions similar to printf.
#if defined( __GNUC__ ) || defined( __clang__ )
#define B3_PRINTF_FORMAT( INDEX1, INDEX2 ) __attribute__( ( format( printf, INDEX1, INDEX2 ) ) )
#else
#define B3_PRINTF_FORMAT( INDEX1, INDEX2 )
#endif

// clang-format on

/**
 * @defgroup base Base
 * Base functionality
 * @{
 */

/// This is used to indicate null for interfaces that work with indices instead of pointers
#define B3_NULL_INDEX -1

/// Prototype for user allocation function.
///	@param size the allocation size in bytes
///	@param alignment the required alignment, guaranteed to be a power of 2
typedef void* b3AllocFcn( int32_t size, int32_t alignment );

/// Prototype for user free function.
///	@param mem the memory previously allocated through `b3AllocFcn`
typedef void b3FreeFcn( void* mem );

/// Prototype for the user assert callback. Return 0 to skip the debugger break.
typedef int b3AssertFcn( const char* condition, const char* fileName, int lineNumber );

/// Prototype for user log callback. Used to log warnings.
typedef void b3LogFcn( const char* message );

/// This allows the user to override the allocation functions. These should be
///	set during application startup.
///
/// NEA_Phys3DWorldCreate installs a pool allocator through this hook, which is
/// how the world's memory ends up pre-reserved and free of mid-step
/// allocation.
B3_API void b3SetAllocator( b3AllocFcn* allocFcn, b3FreeFcn* freeFcn );

/// Total bytes allocated by Box3D.
B3_API int b3GetByteCount( void );

/// Override the default assert callback.
///	@param assertFcn a non-null assert callback
B3_API void b3SetAssertFcn( b3AssertFcn* assertFcn );

/// Override the default logging callback.
B3_API void b3SetLogFcn( b3LogFcn* logFcn );

// -------------------------------------------------------------------------
// Assertions
// -------------------------------------------------------------------------
//
// B3_ASSERT follows NEA_DEBUG, matching how the rest of Nitro Engine Advanced
// works: live when the library is built with NEA_DEBUG=1, compiled out
// entirely otherwise. Messages route through b3Log, which forwards to
// NEA_DebugPrint on device and to stderr on the host, so whatever handler a
// game installed with NEA_DebugSetHandler receives them.
//
// Note this is deliberately *not* keyed on NDEBUG. Box3D uses NDEBUG, but
// tying the port's assertions to NEA's own debug switch means one flag
// controls the whole library.

#if defined( __GNUC__ ) || defined( __clang__ )
#define B3_BREAKPOINT __builtin_trap()
#else
#include <assert.h>
#define B3_BREAKPOINT assert( 0 )
#endif

#if defined( NEA_DEBUG ) || defined( B3_ENABLE_ASSERT )

/// Internal assertion handler. Allows for host intervention.
B3_API int b3InternalAssert( const char* condition, const char* fileName, int lineNumber );

/// Assert that a condition is true.
#define B3_ASSERT( condition )                                                                                                   \
	( (void)( ( !!( condition ) ) || ( b3InternalAssert( #condition, __FILE__, (int)( __LINE__ ) ), 0 ) ) )

#else
#define B3_ASSERT( ... ) ( (void)0 )
#endif

/// Floating point tolerance check.
///
/// Always compiled out. Upstream uses this for checks like "is this value
/// within 1e-6 of unit length", which are statements about float rounding.
/// Fixed point quantizes deterministically instead, so the checks would
/// either be trivially true or trivially false depending on the scale, and
/// carry no information either way.
#define B3_VALIDATE( ... ) ( (void)0 )

/// Heavy structural validation, off unless assertions are.
///
/// Distinct from B3_VALIDATE: that one is a float-tolerance tier with no
/// meaning in fixed point, whereas this guards whole-structure walks such as
/// b3DynamicTree_Validate. Those are exact integer checks -- parent pointers,
/// heights, containment -- so they remain worth running, just not at 60 Hz on
/// a 67 MHz CPU.
#if !defined( B3_ENABLE_VALIDATION )
#if defined( NEA_DEBUG ) || defined( B3_ENABLE_ASSERT )
#define B3_ENABLE_VALIDATION 1
#else
#define B3_ENABLE_VALIDATION 0
#endif
#endif

/// Version numbering scheme.
/// See https://semver.org/
typedef struct b3Version
{
	/// Significant changes
	int major;

	/// Incremental changes
	int minor;

	/// Bug fixes
	int revision;
} b3Version;

/// Get the current version of Box3D.
B3_API b3Version b3GetVersion( void );

/**@}*/

//! @cond

// Simple djb2 hash, used for geometry content hashes.
#define B3_HASH_INIT 5381
B3_API uint32_t b3Hash( uint32_t hash, const uint8_t* data, int count );

//! @endcond
