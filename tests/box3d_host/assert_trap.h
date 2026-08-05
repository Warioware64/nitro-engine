// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Warioware64, 2026

#pragma once

/// Route the port's assertions into the test result instead of into a trap.
///
/// @section why Why this exists
///
/// Until Phase 3B the host suite defined neither NEA_DEBUG nor
/// B3_ENABLE_ASSERT, so B3_ASSERT expanded to `(void)0` and all four
/// structural validators -- b3ValidateConnectivity, b3ValidateSolverSets,
/// b3ValidateContacts, b3ValidateIsland -- compiled to empty stubs. Every test
/// that called them was calling nothing. Turning them on immediately found a
/// real out-of-bounds read in test_support.c's bitset union.
///
/// With assertions live, the default handler returns non-zero and
/// b3InternalAssert executes __builtin_trap(). That kills the process on the
/// first failure and reports no counts, which is worse for a test binary than
/// a named failure and a full run. This handler returns 0 instead -- no
/// breakpoint -- and records the assertion so main can report it.
///
/// @section expected Deliberate assertions
///
/// Some tests exercise paths that assert on purpose: test_support.c's refusal
/// cases drive b3RaiseOutOfMemory, which ends in B3_ASSERT(false) precisely so
/// an *accidental* refusal is loud. Wrap those in
/// b3TestExpectAsserts(true)/(false) so they are counted rather than failed.

#include "box3d/base.h"

#include <stdio.h>

// Not every test file uses every helper -- only test_support.c has a section
// whose assertions are deliberate.
#define B3_TEST_UNUSED __attribute__( ( unused ) )

static int s_assertUnexpected = 0;
static int s_assertExpected = 0;
static bool s_assertExpecting = false;
static char s_assertFirst[256] = { 0 };

static int b3TestAssertFcn( const char* condition, const char* fileName, int lineNumber )
{
	if ( s_assertExpecting )
	{
		s_assertExpected += 1;
	}
	else
	{
		s_assertUnexpected += 1;
		if ( s_assertFirst[0] == 0 )
		{
			snprintf( s_assertFirst, sizeof( s_assertFirst ), "%s  (%s:%d)", condition, fileName, lineNumber );
		}
		printf( "  ASSERT %s  (%s:%d)\n", condition, fileName, lineNumber );
	}

	// Zero means "do not break". The test keeps running so the whole suite
	// reports rather than dying on the first problem.
	return 0;
}

/// Install the trapping handler. Call once at the top of main.
B3_TEST_UNUSED static void b3TestInstallAssertTrap( void )
{
	b3SetAssertFcn( b3TestAssertFcn );
}

/// Mark a region whose assertions are the behaviour under test.
B3_TEST_UNUSED static void b3TestExpectAsserts( bool expecting )
{
	s_assertExpecting = expecting;
}

/// How many assertions fired outside an expected region.
B3_TEST_UNUSED static int b3TestUnexpectedAsserts( void )
{
	return s_assertUnexpected;
}

/// How many fired inside one. Reported so a test that expects an assertion and
/// stops producing it is not silently downgraded.
B3_TEST_UNUSED static int b3TestExpectedAsserts( void )
{
	return s_assertExpected;
}
