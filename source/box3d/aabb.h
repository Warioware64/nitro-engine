// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Erin Catto        (original Box3D)
// Copyright (c) 2026 Warioware64       (Nitro Engine Advanced fixed-point port)
//
// This file is part of Nitro Engine Advanced

#pragma once

#include "box3d/types.h"

/// Ray cast an AABB.
///
/// The fractions are in [0, 1] along p1 -> p2.
bool b3RayCastAABB( b3AABB a, b3Vec3 p1, b3Vec3 p2, b3c* minFraction, b3c* maxFraction );

/// Does a ray segment overlap a box? A separating-axis test on the three edge
/// cross products (Gino, p80). Used by the dynamic tree to reject nodes.
///
/// Upstream keeps this in simd.h and writes it against the 4-wide b3V32. The
/// port has no simd.h, and this is one of two things in dynamic_tree.c that
/// needed it, so it lands here beside the other AABB ray code. Arguments keep
/// upstream's order so the call sites still read like upstream.
///
/// It is exact -- there is no rounding anywhere in it. Upstream forms the box
/// centre and half-extent, which in fixed point would each shed half a
/// quantum; instead every term is doubled:
///
///     2*start  = 2*rayStart - (min + max)
///     2*extent = max - min
///
/// Doubling both sides of a comparison against zero cannot change its result,
/// and both expressions are integer differences of the raw inputs. So the test
/// answers the exact geometric question rather than a rounded one, which
/// matters because a false negative here silently drops a proxy from a tree
/// query rather than merely shifting a number.
///
/// Scale: the cross products are Q12 x Q12 -> Q24, held in int64. At the
/// documented +/-2000 unit world edge the terms reach about 1.4e14, four
/// orders of magnitude below the int64 ceiling.
static inline bool b3TestBoundsRayOverlap( b3Vec3 nodeMin, b3Vec3 nodeMax, b3Vec3 rayStart, b3Vec3 rayDelta )
{
	// Doubled ray origin relative to the box centre.
	int64_t sx = 2 * (int64_t)b3Raw( rayStart.x ) - ( (int64_t)b3Raw( nodeMin.x ) + b3Raw( nodeMax.x ) );
	int64_t sy = 2 * (int64_t)b3Raw( rayStart.y ) - ( (int64_t)b3Raw( nodeMin.y ) + b3Raw( nodeMax.y ) );
	int64_t sz = 2 * (int64_t)b3Raw( rayStart.z ) - ( (int64_t)b3Raw( nodeMin.z ) + b3Raw( nodeMax.z ) );

	// Doubled half-extent, i.e. the full box size. Non-negative for a valid
	// AABB, so no absolute value is needed on it.
	int64_t ex = (int64_t)b3Raw( nodeMax.x ) - b3Raw( nodeMin.x );
	int64_t ey = (int64_t)b3Raw( nodeMax.y ) - b3Raw( nodeMin.y );
	int64_t ez = (int64_t)b3Raw( nodeMax.z ) - b3Raw( nodeMin.z );

	int64_t dx = (int64_t)b3Raw( rayDelta.x );
	int64_t dy = (int64_t)b3Raw( rayDelta.y );
	int64_t dz = (int64_t)b3Raw( rayDelta.z );

	int64_t ax = dx < 0 ? -dx : dx;
	int64_t ay = dy < 0 ? -dy : dy;
	int64_t az = dz < 0 ? -dz : dz;

	// |cross(delta, start)| against the modified cross of the magnitudes,
	// which is the box's projected radius on each edge axis.
	int64_t cx = dy * sz - dz * sy;
	int64_t cy = dz * sx - dx * sz;
	int64_t cz = dx * sy - dy * sx;

	if ( cx < 0 )
	{
		cx = -cx;
	}
	if ( cy < 0 )
	{
		cy = -cy;
	}
	if ( cz < 0 )
	{
		cz = -cz;
	}

	return cx <= ay * ez + az * ey && cy <= az * ex + ax * ez && cz <= ax * ey + ay * ex;
}

/// Surface area of an AABB, the dynamic tree's cost metric.
///
/// Upstream names this b3Perimeter but computes a surface area; the name is
/// kept so the call sites in dynamic_tree.c still read like upstream.
///
/// The return type is not: it is int64 because a Q12 surface area overflows
/// almost immediately. A 2000-unit box -- the documented edge of the usable
/// world -- has an area of 2.4e7, which is 9.8e10 once scaled, forty times
/// past what an int32 holds. The tree only ever compares these values against
/// each other and accumulates differences of them, so carrying them wide
/// costs one extra compare and removes the ceiling entirely.
static inline int64_t b3Perimeter( b3AABB a )
{
	return b3AABB_AreaWide( a );
}

/// Enlarge a to contain b.
/// @return true if the AABB grew
static inline bool b3EnlargeAABB( b3AABB* a, b3AABB b )
{
	bool changed = false;
	if ( b3Raw( b.lowerBound.x ) < b3Raw( a->lowerBound.x ) )
	{
		a->lowerBound.x = b.lowerBound.x;
		changed = true;
	}

	if ( b3Raw( b.lowerBound.y ) < b3Raw( a->lowerBound.y ) )
	{
		a->lowerBound.y = b.lowerBound.y;
		changed = true;
	}

	if ( b3Raw( b.lowerBound.z ) < b3Raw( a->lowerBound.z ) )
	{
		a->lowerBound.z = b.lowerBound.z;
		changed = true;
	}

	if ( b3Raw( a->upperBound.x ) < b3Raw( b.upperBound.x ) )
	{
		a->upperBound.x = b.upperBound.x;
		changed = true;
	}

	if ( b3Raw( a->upperBound.y ) < b3Raw( b.upperBound.y ) )
	{
		a->upperBound.y = b.upperBound.y;
		changed = true;
	}

	if ( b3Raw( a->upperBound.z ) < b3Raw( b.upperBound.z ) )
	{
		a->upperBound.z = b.upperBound.z;
		changed = true;
	}

	return changed;
}

static inline b3Vec3 b3FarthestPointOnAABB( b3AABB b, b3Vec3 p )
{
	b3Vec3 r;
	r.x = b3Raw( b3SubF( p.x, b.lowerBound.x ) ) > b3Raw( b3SubF( b.upperBound.x, p.x ) ) ? b.lowerBound.x : b.upperBound.x;
	r.y = b3Raw( b3SubF( p.y, b.lowerBound.y ) ) > b3Raw( b3SubF( b.upperBound.y, p.y ) ) ? b.lowerBound.y : b.upperBound.y;
	r.z = b3Raw( b3SubF( p.z, b.lowerBound.z ) ) > b3Raw( b3SubF( b.upperBound.z, p.z ) ) ? b.lowerBound.z : b.upperBound.z;
	return r;
}
