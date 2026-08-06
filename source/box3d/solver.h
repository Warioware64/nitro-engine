// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Erin Catto        (original Box3D)
// Copyright (c) 2026 Warioware64       (Nitro Engine Advanced fixed-point port)
//
// This file is part of Nitro Engine Advanced

#pragma once

/// @file   solver.h
/// @brief  The per-step context and the soft-constraint parameterisation.
///
/// @section threading What upstream's solver.h is mostly about, and why none
///          of it is here
///
/// Upstream opens this header with fifty lines describing a work-stealing
/// scheduler: fixed-size blocks claimed by atomic CAS on a per-block sync
/// index, monotonic indices so iterative stages need no barrier, and start
/// offsets chosen to keep each worker re-hitting the same blocks for L2
/// affinity. It is good engineering and it is entirely about having more than
/// one core.
///
/// The DS has one ARM9. So `b3SolverStage`, `b3SolverBlock`, `b3SyncBlock`,
/// `b3SolverStageType`, `b3SolverBlockType`, `b3BlockDim` and the whole
/// `atomicSyncBits` protocol are absent, and every "task" in solver.c is a
/// plain loop over an index range.
///
/// @section colors The other collapse
///
/// B3_GRAPH_COLOR_COUNT is 1 and B3_OVERFLOW_INDEX is 0, so upstream's
/// per-colour loops -- all written `for ( i < B3_GRAPH_COLOR_COUNT - 1 )` --
/// iterate **zero** times and `activeColorCount` is always zero. Every
/// constraint lives in the overflow colour. That is why this context carries
/// no per-colour spans, no wide-constraint arrays and no active colour count.

#include "container.h"
#include "core.h"

#include "box3d/b3profile.h"
#include "box3d/math_fixed.h"

#include <stdint.h>

typedef struct b3BodySim b3BodySim;
typedef struct b3BodyState b3BodyState;
typedef struct b3ConstraintGraph b3ConstraintGraph;
typedef struct b3ContactConstraint b3ContactConstraint;
typedef struct b3ManifoldConstraint b3ManifoldConstraint;
typedef struct b3World b3World;

// =========================================================================
// The fixed time step
// =========================================================================
//
// dt is a compile-time constant (B3_NEA_STEP_HZ), which is why b3World_Step
// takes only a sub-step count. h and inv_h still depend on that count, so
// they are computed once per step rather than folded.

/// The time step, as a b3t (Q24). 1/60 s.
#define B3_NEA_DT b3tFromFrac( 1, B3_NEA_STEP_HZ )

/// The inverse time step. **A length scale, not the inverse-weight scale.**
///
/// Q7.24 tops out at 128 and inv_h reaches `subStepCount / dt` -- 240 at the
/// default four sub-steps, and more if a caller asks for more. So every
/// inverse-time quantity in the solver is carried at Q12, where 240 is 983040
/// raw and the format holds five more decimal digits of headroom.
///
/// Multiplying one of these by a length gives a velocity, which is b3MulFF --
/// dimensionally right and at the correct scale on both ends.
#define B3_NEA_INV_DT b3fFromInt( B3_NEA_STEP_HZ )

/// 2*pi, at Q12. Only b3MakeSoft needs it.
///
/// 710/113 is 6.2831858, which differs from 2*pi by 2.7e-7 -- two orders below
/// what Q12 can represent -- so this is the exact rational path b3fFromFrac
/// exists for rather than a rounded literal.
#define B3_TWO_PI b3fFromFrac( 710, 113 )

// =========================================================================
// Soft constraints
// =========================================================================

/// The soft-constraint coefficients a constraint is solved with.
///
/// Upstream stores three floats. The port cannot: the three quantities have
/// genuinely different ranges and two of them are dimensionless while the
/// third is not.
typedef struct b3Softness
{
	/// Multiplies a separation to produce a bias velocity, so it has units of
	/// inverse time -- and it reaches `1 / h` when the damping ratio is zero,
	/// which upstream's own comment states. At the default four sub-steps that
	/// is 240, four times what Q7.24 can hold. Hence Q12, per B3_NEA_INV_DT.
	b3f biasRate;

	/// Both dimensionless and both in [0,1], so Q30.
	///
	/// They sum to exactly one -- upstream derives massScale as `a2 * a3` and
	/// impulseScale as `a3`, and notes the identity holds. The port computes
	/// `massScale = 1 - impulseScale` instead, which makes the identity exact
	/// by construction rather than up to rounding, and costs one multiply
	/// less. b3MakeSoft's test asserts the sum.
	b3c massScale;
	b3c impulseScale;
} b3Softness;

/// Soft-constraint coefficients for a spring of `hertz` and damping ratio
/// `zeta` solved at sub-step `h`.
///
/// A zero hertz returns all three zero, which is how a caller disables soft
/// contacts through b3World_SetContactTuning.
///
/// Defined in b3hot.c.
b3Softness b3MakeSoft( b3f hertz, b3f zeta, b3t h );

// =========================================================================
// The step context
// =========================================================================

/// Everything the solver needs for one step, gathered once so the inner loops
/// do not chase it through the world.
///
/// Absent, relative to upstream: every `wide*` field and the SIMD constraint
/// arrays (no SIMD); `widePrepareSpans` / `contactPrepareSpans` /
/// `overflowSpans` / `jointPrepareSpans` and `activeColorCount` (one colour,
/// so there are no spans to describe); `stages` / `stageCount` / `workerCount`
/// / `atomicSyncBits` / `mainClaimed` (one worker); `bulletBodies` and
/// `bulletBodyCount` (CCD, deferred); and all three `padding[64]` members,
/// which exist to stop two cores sharing a cache line.
typedef struct b3StepContext
{
	/// The full step and its inverse. Constants, but carried here so the
	/// inner loops read one struct.
	b3t dt;
	b3f inv_dt;

	/// The sub-step and its inverse.
	b3t h;
	b3f inv_h;

	int subStepCount;

	b3Softness contactSoftness;
	b3Softness staticSoftness;

	b3f restitutionThreshold;
	b3f maxLinearVelocity;

	b3World* world;
	b3ConstraintGraph* graph;

	/// Shortcuts into the awake solver set. Both are invalidated by anything
	/// that grows the awake set, which is why nothing between b3Solve's setup
	/// and its finalize pass may create a body.
	b3BodyState* states;
	b3BodySim* sims;

	/// Contact ids for the narrow phase's single flat pass. These contacts may
	/// or may not be touching; all belong to awake bodies.
	int* awakeContactIndices;

	/// Filled by the solver's prepare pass. Phase 3C-ii.
	b3ManifoldConstraint* manifoldConstraints;
	b3ContactConstraint* contactConstraints;

	bool enableWarmStarting;
} b3StepContext;

/// Integrate velocities, solve the constraints, integrate positions, then
/// finalize the bodies -- update transforms and world inertia, emit move
/// events, enlarge broad-phase proxies, and sleep whatever islands settled.
///
/// Marks its own eight sub-phases against b3World::profileTimer rather than
/// being bracketed from outside: it is where most of the step goes, and one
/// number for the whole of it would answer nothing.
void b3Solve( b3World* world, b3StepContext* context );

/// Bytes of b3Stack the solve stage holds live at its peak.
///
/// The counterpart of b3BroadPhaseStackDemand, and the larger of the two for
/// any scene with roughly one contact per two moving shapes. b3CreateWorld
/// sizes the stack from the maximum of them so b3GrowStack never runs.
int b3SolverStackDemand( int contactCount, int manifoldCount );
