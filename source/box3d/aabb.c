// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Erin Catto        (original Box3D)
// Copyright (c) 2026 Warioware64       (Nitro Engine Advanced fixed-point port)
//
// This file is part of Nitro Engine Advanced

#include "aabb.h"

#include "box3d/math_fixed.h"

// Ray-slab intersection, following Real-time Collision Detection p179.
//
// The structure is upstream's. What is different is how the slab division is
// guarded, and that is the whole difficulty of this function in fixed point.
//
// Upstream computes t = (boxEdge - rayStart) / rayComponent and relies on
// float to cope when rayComponent is tiny: the quotient simply becomes huge,
// and the later min/max against tMin/tMax discards it. Fixed point has no
// such escape. The hardware divider returns 32 bits, so a quotient that does
// not fit is not merely imprecise -- it is undefined, and a ray very nearly
// parallel to a slab produces exactly that.
//
// Guarding by "is rayComponent smaller than some epsilon" does not work
// either, because whether the quotient overflows depends on the numerator as
// much as the denominator, and both range over the whole world size.
//
// So the test is made directly against the thing that matters. The
// intersection interval is clamped to [0, rayLength] regardless, so any |t|
// beyond rayLength is indistinguishable from the slab being parallel. That
// condition is |num| > |den| * rayLength, which is a multiply and a compare
// in int64 -- no division, no overflow, and exact.

/// Clamped slab intersection.
///
/// Returns false when the crossing lies outside [-rayLength, rayLength], in
/// which case the caller treats the slab as parallel. Otherwise *t receives
/// the crossing distance along the ray.
static bool b3SlabCross( b3f num, b3f den, b3f rayLength, b3f* t )
{
	int64_t n = b3Raw( num );
	int64_t d = b3Raw( den );

	if ( d == 0 )
	{
		return false;
	}

	int64_t absN = n < 0 ? -n : n;
	int64_t absD = d < 0 ? -d : d;

	// |num / den| > rayLength  <=>  |num| > |den| * rayLength.
	// Both sides are scaled by 2^12 on the right, so shift the left to match.
	if ( ( absN << B3_F_SHIFT ) > absD * (int64_t)b3Raw( rayLength ) )
	{
		return false;
	}

	// The quotient is now known to fit, so the hardware divider is safe.
	*t = b3DivFF( num, den );
	return true;
}

bool b3RayCastAABB( b3AABB a, b3Vec3 p1, b3Vec3 p2, b3c* minFraction, b3c* maxFraction )
{
	b3Vec3 d = b3Sub( p2, p1 );
	b3f rayLength = b3Length( d );

	// Degenerate ray: upstream compares against FLT_EPSILON, which here means
	// "shorter than one quantum", i.e. zero.
	if ( b3Raw( rayLength ) == 0 )
	{
		if ( b3Raw( p1.x ) >= b3Raw( a.lowerBound.x ) && b3Raw( p1.x ) <= b3Raw( a.upperBound.x ) &&
			 b3Raw( p1.y ) >= b3Raw( a.lowerBound.y ) && b3Raw( p1.y ) <= b3Raw( a.upperBound.y ) &&
			 b3Raw( p1.z ) >= b3Raw( a.lowerBound.z ) && b3Raw( p1.z ) <= b3Raw( a.upperBound.z ) )
		{
			*minFraction = b3c_zero;
			*maxFraction = b3c_zero;
			return true;
		}

		return false;
	}

	b3Vec3 rayDir = b3Normalize( d );

	b3f tMin = b3f_zero;
	b3f tMax = rayLength;

	const b3f starts[3] = { p1.x, p1.y, p1.z };
	const b3f dirs[3] = { rayDir.x, rayDir.y, rayDir.z };
	const b3f mins[3] = { a.lowerBound.x, a.lowerBound.y, a.lowerBound.z };
	const b3f maxs[3] = { a.upperBound.x, a.upperBound.y, a.upperBound.z };

	// Upstream unrolls the three axes into three identical blocks. A loop is
	// the same work and a third of the code, which matters more here than the
	// unrolling would: this runs from the tree traversal, and instruction
	// cache is a scarcer resource on this machine than branch slots.
	for ( int axis = 0; axis < 3; axis++ )
	{
		b3f rayStart = starts[axis];
		b3f rayComponent = dirs[axis];
		b3f boxMin = mins[axis];
		b3f boxMax = maxs[axis];

		b3f t1, t2;
		bool cross1 = b3SlabCross( b3SubF( boxMin, rayStart ), rayComponent, rayLength, &t1 );
		bool cross2 = b3SlabCross( b3SubF( boxMax, rayStart ), rayComponent, rayLength, &t2 );

		if ( cross1 == false && cross2 == false )
		{
			// Effectively parallel to this slab: the ray either lies within
			// it for its whole length or misses entirely.
			if ( b3Raw( rayStart ) < b3Raw( boxMin ) || b3Raw( rayStart ) > b3Raw( boxMax ) )
			{
				return false;
			}
			continue;
		}

		// One side crossing but not the other means the ray starts inside the
		// slab and leaves through the far face within its length. Pin the
		// missing end to the ray extent, which is where the clamp would have
		// put it anyway.
		if ( cross1 == false )
		{
			t1 = b3Raw( rayComponent ) > 0 ? b3NegF( rayLength ) : rayLength;
		}
		if ( cross2 == false )
		{
			t2 = b3Raw( rayComponent ) > 0 ? rayLength : b3NegF( rayLength );
		}

		if ( b3Raw( t1 ) > b3Raw( t2 ) )
		{
			b3f temp = t1;
			t1 = t2;
			t2 = temp;
		}

		tMin = b3MaxF( tMin, t1 );
		tMax = b3MinF( tMax, t2 );

		if ( b3Raw( tMin ) > b3Raw( tMax ) )
		{
			return false;
		}
	}

	// Intersection entirely behind the ray start.
	if ( b3Raw( tMax ) < 0 )
	{
		return false;
	}

	// Distances to fractions. Both are known to be within [0, rayLength] by
	// construction, so the quotient is in [0, 1] and lands in b3c exactly.
	*minFraction = b3ClampC( b3DivFFToC( tMin, rayLength ), b3c_zero, b3c_one );
	*maxFraction = b3ClampC( b3DivFFToC( tMax, rayLength ), b3c_zero, b3c_one );

	return true;
}
