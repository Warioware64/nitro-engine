// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Erin Catto        (original Box3D)
// Copyright (c) 2026 Warioware64       (Nitro Engine Advanced fixed-point port)
//
// This file is part of Nitro Engine Advanced

#pragma once

/// @file   contact_solver.h
/// @brief  The soft-constraint contact solver: prepare, warm start, solve,
///         restitution, store.
///
/// @section paths Why there is one solver path and not three
///
/// Upstream declares each stage three times -- `_Overflow`, `_Mesh` and
/// `_Convex`. `_Convex` is the SIMD path and was dropped in Phase 2B along
/// with simd.h. `_Overflow` and `_Mesh` differ only in where they find their
/// constraint arrays, and with `B3_GRAPH_COLOR_COUNT` at 1 every constraint
/// lives in the overflow colour, so the two collapse into each other.
///
/// What is left is upstream's scalar "mesh" solver under an unsuffixed name,
/// which is what 3C-i's handover asked for. `b3SolverBlock` and
/// `b3_overflowBlock` are gone with the scheduler; each stage takes an index
/// range like every other task in solver.c.
///
/// @section scales The fixed-point shape of a contact constraint
///
/// Four scales meet here and the choice of each is load bearing:
///
///   - **Q16 (b3imp)** for every impulse. These are warm-started -- written
///     back into themselves each sub-step and each step -- so they are the one
///     place a coarse scale compounds rather than averaging out.
///   - **Q24 (b3iw)** for inverse masses and inverse inertias, matching
///     b3BodySim, because the tensor is the solver's divisor.
///   - **Q30 (b3Dir3)** for the contact frame. The manifold stores its normal
///     at Q12 and that is right for a value rebuilt from geometry every step,
///     but inside the constraint it is the axis every impulse is projected
///     onto thousands of times per step, so prepare converts it once.
///   - **Q12 (b3f)** for everything else -- separations, velocities, lever
///     arms, and the effective masses, which are reciprocals of Q24 sums.

#include "solver.h"

#include "box3d/constants.h"
#include "box3d/math_fixed.h"

typedef struct b3Contact b3Contact;

/// One contact point's constraint state.
typedef struct b3ManifoldConstraintPoint
{
	/// Anchors relative to each body's centre of mass, fixed for the step.
	b3Vec3 rA, rB;

	/// Separation with the anchor contribution removed, so that the sub-step
	/// separation is `baseSeparation + dot( dp + rot(dqB,rB) - rot(dqA,rA), n )`
	/// with the normal held constant.
	b3f baseSeparation;

	/// Normal velocity before the solve, kept for restitution.
	b3f relativeVelocity;

	/// The warm-started accumulator, and the sum across sub-steps used to tell
	/// a speculative point that never touched from one that did.
	b3imp normalImpulse;
	b3imp totalNormalImpulse;

	/// 1 / kNormal. A mass, so Q12.
	b3f normalMass;

	/// Distance from this point to the friction centre, which bounds how much
	/// twist the point may resist.
	b3f leverArm;
} b3ManifoldConstraintPoint;

/// One manifold's constraint state: its frame, its points, and the three
/// central friction accumulators that act at the friction centre rather than
/// at any one point.
typedef struct b3ManifoldConstraint
{
	b3ManifoldConstraintPoint points[B3_MAX_MANIFOLD_POINTS];
	int pointCount;

	/// The contact frame at Q30. tangent1/tangent2 complete a right-handed
	/// basis with the normal.
	b3Dir3 normal;
	b3Dir3 tangent1;
	b3Dir3 tangent2;

	/// Friction centres, a separation-weighted mean of the anchors. Friction
	/// acts here rather than per point, which is what stops a spinning top
	/// drifting.
	b3Vec3 centerA, centerB;

	b3f twistMass;
	b3imp twistImpulse;

	b3SymMatrix2 tangentMass;
	b3Imp2 frictionImpulse;
	b3Imp3 rollingImpulse;

	/// Conveyor-belt surface velocity resolved onto the two tangents.
	b3f tangentVelocity1, tangentVelocity2;
} b3ManifoldConstraint;

/// One contact's constraint state -- the per-body quantities its manifolds
/// share.
typedef struct b3ContactConstraint
{
	b3ManifoldConstraint* constraints;
	b3Contact* contact;

	int indexA;
	int indexB;

	b3iw invMassA, invMassB;
	b3MatrixW invIA, invIB;

	b3Softness softness;

	/// inverse( invIA + invIB ) -- an inertia, so Q12.
	b3Matrix3 rollingMass;

	b3c friction;
	b3c restitution;

	/// A length, not a coefficient. See b3Contact::rollingResistance.
	b3f rollingResistance;

	int manifoldCount;
} b3ContactConstraint;

void b3PrepareContacts( int startIndex, int endIndex, b3StepContext* context );
void b3WarmStartContacts( int startIndex, int endIndex, b3StepContext* context );
void b3SolveContacts( int startIndex, int endIndex, b3StepContext* context, bool useBias );
void b3ApplyRestitution( int startIndex, int endIndex, b3StepContext* context );
void b3StoreImpulses( int startIndex, int endIndex, b3StepContext* context );
