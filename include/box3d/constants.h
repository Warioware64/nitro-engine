// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Erin Catto        (original Box3D)
// Copyright (c) 2026 Warioware64       (Nitro Engine Advanced fixed-point port)
//
// This file is part of Nitro Engine Advanced

#pragma once

/// @file   constants.h
/// @brief  Simulation tolerances and fixed capacities.
///
/// Every tolerance here is a literal. Upstream computes them at each use site
/// from `b3GetLengthUnitsPerMeter()` -- a *function call*, expanded at 21
/// places, several inside the solver's inner loops. The port fixes the length
/// unit at compile time in nea_config.h, so each constant collapses to the
/// integer the machine actually wants.
///
/// The raw values below assume B3_NEA_LENGTH_UNITS_PER_METER == 1. The
/// static assertion at the bottom fails if that is changed without
/// recomputing them, so the two cannot drift apart silently.

#include "base.h"
#include "math_fixed.h"

// =========================================================================
// Length tolerances (Q12)
// =========================================================================

/// A small length used as a collision and constraint tolerance. Chosen to be
/// numerically significant but visually insignificant.
/// 0.005 m -> 0.005 * 4096 = 20.5
/// @warning modifying this has a significant impact on stability
#define B3_LINEAR_SLOP b3Makeb3f( 20 )

/// The minimum length of a capsule. Shorter ones should be spheres.
#define B3_MIN_CAPSULE_LENGTH B3_LINEAR_SLOP

/// The distance at which shapes are considered overlapped. Needed because GJK
/// can return small positive values for overlapping shapes in degenerate
/// configurations.
/// 0.1 * slop -> 2.05
#define B3_OVERLAP_SLOP b3Makeb3f( 2 )

/// Speculative contact distance.
/// 4 * slop -> 82
/// @warning modifying this has a significant impact on performance and stability
#define B3_SPECULATIVE_DISTANCE b3Makeb3f( 82 )

/// Rest offset for mesh contact, reducing ghost collisions and assisting CCD.
/// Must be at least B3_LINEAR_SLOP and less than B3_SPECULATIVE_DISTANCE.
/// 1 * slop -> 20
#define B3_MESH_REST_OFFSET b3Makeb3f( 20 )

/// The default contact recycling distance.
/// 10 * slop -> 205
#define B3_CONTACT_RECYCLE_DISTANCE b3Makeb3f( 205 )

/// Used to fatten AABBs in the dynamic tree so proxies can move a little
/// without triggering a tree adjustment.
/// 0.05 m -> 205
/// @warning modifying this has a significant impact on performance
#define B3_MAX_AABB_MARGIN b3Makeb3f( 205 )

/// Minimum scale for collision meshes.
/// 0.01 -> 41
#define B3_MIN_SCALE b3Makeb3f( 41 )

/// Used to detect bad values.
///
/// Upstream sets this to 1e5 metres, well inside float's usable range. Q12
/// runs out much sooner: the format tops out at 524288, and long before that
/// B3_LINEAR_SLOP stops being small relative to a body. 2000 units is the
/// documented practical world size, and this is the sanity bound that goes
/// with it.
#define B3_HUGE b3fFromInt( 2000 )

// =========================================================================
// Dimensionless coefficients (Q30)
// =========================================================================

/// Per-shape AABB margin as a fraction of the shape extent, capped by
/// B3_MAX_AABB_MARGIN.
/// 0.125 * 2^30
#define B3_AABB_MARGIN_FRACTION b3Makeb3c( 134217728 )

/// Relative tolerance for deciding two edges are parallel.
/// 0.005 * 2^30
#define B3_PARALLEL_EDGE_TOL b3Makeb3c( 5368709 )

/// Contact recycling angular threshold, stored as cos(angle/2)^2 for speed.
/// This value corresponds to 10 degrees.
/// 0.99240388 * 2^30
#define B3_CONTACT_RECYCLE_ANGULAR_DISTANCE b3Makeb3c( 1065760972 )

/// Minimum contact point friction weight, a lower bound for speculative
/// points.
///
/// Upstream uses 1e-10, described as "small enough to be washed away by
/// weights that hit 1". That is a statement about float's dynamic range, and
/// it underflows every scale here -- Q30's smallest positive value is
/// 9.3e-10. One quantum is the faithful translation of the intent: the
/// smallest non-zero weight the representation has.
#define B3_MIN_FRICTION_WEIGHT b3Makeb3c( 1 )

// =========================================================================
// Time (Q24) and angle (brad)
// =========================================================================

/// How long a body must be still before it sleeps.
/// 0.5 s * 2^24
#define B3_TIME_TO_SLEEP b3Makeb3t( 8388608 )

/// The maximum rotation of a body per time step, a numerical safety limit.
/// 0.25 * pi rad, which is exactly an eighth of a circle: 32768 / 8.
/// @warning raising this to half pi or more breaks continuous collision.
#define B3_MAX_ROTATION ( (b3a)4096 )

// =========================================================================
// Fixed capacities
// =========================================================================
//
// The values themselves come from nea_config.h, which is force-included ahead
// of this header. Only the derived ones live here.

/// The maximum number of contact points between two touching shapes.
#define B3_MAX_MANIFOLD_POINTS 4

/// Number of contact point buckets, for reporting only.
#define B3_CONTACT_MANIFOLD_COUNT_BUCKETS 8

/// Iterations for gyroscopic torque.
#ifndef B3_GYROSCOPIC_ITERATIONS
#define B3_GYROSCOPIC_ITERATIONS 1
#endif

/// Increase for more accurate restitution, at a cost in speed. Must be >= 1.
#ifndef B3_RESTITUTION_ITERATIONS
#define B3_RESTITUTION_ITERATIONS 1
#endif

/// Shape pair key packing. B3_SHAPE_POWER comes from nea_config.h (12 here,
/// against upstream's 22), which is what lets b3ShapePairKey fit in 32 bits.
#define B3_CHILD_POWER ( 32 - 2 * B3_SHAPE_POWER )
#define B3_MAX_SHAPES ( 1 << B3_SHAPE_POWER )
#define B3_MAX_CHILD_SHAPES ( 1 << B3_CHILD_POWER )

/// This holds constraints that cannot fit the graph colour limit.
///
/// With B3_GRAPH_COLOR_COUNT reduced to 1 for single-threaded solving, every
/// constraint lands in colour 0, which is the overflow colour.
#define B3_OVERFLOW_INDEX ( B3_GRAPH_COLOR_COUNT - 1 )

// The literals above are only correct for a length unit of one metre.
_Static_assert( B3_NEA_LENGTH_UNITS_PER_METER == 1,
				"tolerance literals in constants.h assume 1 unit == 1 metre; recompute them if this changes" );

// A child index must still fit alongside two shape indices in one 32-bit key.
_Static_assert( B3_CHILD_POWER > 0, "B3_SHAPE_POWER too large for a 32-bit shape pair key" );
