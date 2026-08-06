// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Erin Catto        (original Box3D)
// Copyright (c) 2026 Warioware64       (Nitro Engine Advanced fixed-point port)
//
// This file is part of Nitro Engine Advanced

/// @file   mover.c
/// @brief  The character mover's plane solver.
///
/// Two functions, and the second is four lines of arithmetic. This is the one
/// file in Phase 7 that is a **straight transliteration** -- no divergence, not
/// even a spelling one beyond b3f replacing float.
///
/// @section why Why a solver at all
///
/// A rigid body resolves penetration by accumulating impulses over substeps
/// and letting the integrator carry the result. A mover has no integrator and
/// no substeps: it is handed a set of planes and a translation it would like to
/// make, and it has to produce a translation that satisfies all of them at
/// once. Satisfying them one at a time does not work -- pushing out of the
/// floor pushes into the wall -- so this relaxes over the whole set until the
/// pushes stop changing, which is Gauss-Seidel with the same accumulate-and-
/// clamp that the contact solver uses, minus everything that needs a mass.
///
/// @section iterations The 20-iteration cap is load bearing, and not because
///          of fixed point
///
/// This loop runs the cap out **often**: over 160,000 random plane sets of one
/// to eight planes at depths up to a quarter unit, it reached 20 iterations
/// **52.4%** of the time. That is not a Q12 artifact and it was measured rather
/// than assumed -- the identical scenarios solved in double precision run the
/// cap out **52.3%** of the time, and 83,660 of the ~83,800 cases are the same
/// scenarios on both sides.
///
/// The cause is the algorithm. Gauss-Seidel over two nearly opposing planes
/// converges geometrically at a ratio near 0.76, so `totalPush` falls by about
/// a quarter per iteration and simply does not reach B3_LINEAR_SLOP inside
/// twenty. Upstream knows: its own test/test_mover.c `GamePlanes` asserts
/// `iterationCount == 20` and its comment says "this scenario takes many
/// iterations because the target is deep into the plane".
///
/// So the cap is a **budget, not a failure**. The delta returned after twenty
/// iterations is a good answer that is still creeping toward a slightly better
/// one, which is exactly what a character controller wants and why the caller
/// runs its own slide loop on top. Anything that looks like a convergence fix
/// here -- a deadband on small pushes was tried, and moved 52.38% to 52.36% --
/// is treating the wrong thing.
///
/// @section absent What is not here
///
/// Upstream's b3SolvePlanes takes float pushes and this takes b3f ones, which
/// is the only signature change. There is no b3Body_CollideMover in the port
/// and so no b3BodyPlaneResult; see body.h.

#include "box3d/collision.h"
#include "box3d/constants.h"
#include "box3d/math_fixed.h"
#include "box3d/types.h"

b3PlaneSolverResult b3SolvePlanes( b3Vec3 targetDelta, b3CollisionPlane* planes, int count )
{
	B3_ASSERT( count >= 0 );
	B3_ASSERT( count == 0 || planes != NULL );

	for ( int i = 0; i < count; ++i )
	{
		planes[i].push = b3f_zero;
	}

	b3Vec3 delta = targetDelta;
	b3f tolerance = B3_LINEAR_SLOP;

	int iteration;
	for ( iteration = 0; iteration < 20; ++iteration )
	{
		b3f totalPush = b3f_zero;

		for ( int planeIndex = 0; planeIndex < count; ++planeIndex )
		{
			b3CollisionPlane* plane = planes + planeIndex;

			// `delta` is a translation and this is a plane whose offset is a
			// depth, so the two compose: the separation of the mover after
			// moving by `delta`. Add slop to prevent jitter.
			b3f separation = b3AddF( b3PlaneSeparation( plane->plane, delta ), B3_LINEAR_SLOP );
			b3f push = b3NegF( separation );

			// Clamp the accumulated push, then take what that actually
			// permitted rather than what was asked for.
			b3f accumulatedPush = plane->push;
			plane->push = b3ClampFloat( b3AddF( plane->push, push ), b3f_zero, plane->pushLimit );
			push = b3SubF( plane->push, accumulatedPush );

			delta = b3MulAdd( delta, push, plane->plane.normal );
			totalPush = b3AddF( totalPush, b3AbsFloat( push ) );
		}

		if ( b3Raw( totalPush ) < b3Raw( tolerance ) )
		{
			break;
		}
	}

	b3PlaneSolverResult result;
	result.delta = delta;
	result.iterationCount = iteration;
	return result;
}

b3Vec3 b3ClipVector( b3Vec3 vector, const b3CollisionPlane* planes, int count )
{
	B3_ASSERT( count >= 0 );
	B3_ASSERT( count == 0 || planes != NULL );

	b3Vec3 v = vector;

	for ( int planeIndex = 0; planeIndex < count; ++planeIndex )
	{
		const b3CollisionPlane* plane = planes + planeIndex;

		if ( b3Raw( plane->push ) == 0 || plane->clipVelocity == false )
		{
			continue;
		}

		// Only the inward component goes. b3MinFloat against zero is what makes
		// this a clip rather than a projection: a velocity already leaving the
		// plane is left exactly as it was.
		v = b3MulSub( v, b3MinFloat( b3f_zero, b3Dot( v, plane->plane.normal ) ), plane->plane.normal );
	}

	return v;
}
