// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Erin Catto        (original Box3D)
// Copyright (c) 2026 Warioware64       (Nitro Engine Advanced fixed-point port)
//
// Dirk Gregorius contributed portions of this code
//
// This file is part of Nitro Engine Advanced

#include "box3d/collision.h"
#include "box3d/constants.h"
#include "box3d/math_fixed.h"
#include "box3d/types.h"

#include "core.h"

b3MassData b3ComputeSphereMass( const b3Sphere* shape, b3f density )
{
	b3f radius = shape->radius;

	// volume = 4/3 * pi * r^3
	//
	// The cube is the reason this runs wide. A radius of 50 gives a volume of
	// 523,000, which is 2.1e9 in Q12 -- right at the int32 ceiling -- and the
	// intermediate r^3 gets there first. Accumulating in int64 and narrowing
	// once at the end pushes the limit out to where the radius itself stops
	// being representable.
	int64_t r = (int64_t)b3Raw( radius );
	int64_t r3 = ( ( r * r ) >> B3_F_SHIFT ) * r; // Q24

	// 4/3 * pi = 4.18879, at Q24 to match r3's scale on the way down.
	const int64_t fourThirdsPi = 68629; // 4.1887902 * 2^14
	int64_t volume = ( r3 * fourThirdsPi ) >> ( 14 + B3_F_SHIFT ); // Q12

	b3MassData out;
	out.mass = b3Makeb3f( (int32_t)( ( volume * b3Raw( density ) ) >> B3_F_SHIFT ) );
	out.center = shape->center;

	// Inertia per unit mass: 0.4 * r^2. See the note in math_fixed.h -- the
	// absolute inertia 0.4*m*r^2 grows as r^5 and overflows Q12 at radius 13,
	// while this form is bounded by the world size.
	out.inertia = b3SphereUnitInertia( radius );

	return out;
}

b3AABB b3ComputeSphereAABB( const b3Sphere* shape, b3Transform transform )
{
	b3Vec3 center = b3TransformPoint( transform, shape->center );
	b3f radius = shape->radius;
	b3Vec3 extent = b3MakeVec3( radius, radius, radius );
	return b3MakeAABB( b3Sub( center, extent ), b3Add( center, extent ) );
}

b3AABB b3ComputeSweptSphereAABB( const b3Sphere* shape, b3Transform xf1, b3Transform xf2 )
{
	b3f radius = shape->radius;
	b3Vec3 r = b3MakeVec3( radius, radius, radius );
	b3Vec3 center1 = b3TransformPoint( xf1, shape->center );
	b3Vec3 center2 = b3TransformPoint( xf2, shape->center );
	return b3MakeAABB( b3Sub( b3Min( center1, center2 ), r ), b3Add( b3Max( center1, center2 ), r ) );
}

bool b3OverlapSphere( const b3Sphere* shape, b3Transform shapeTransform, const b3ShapeProxy* proxy )
{
	b3DistanceInput input;
	input.proxyA = ( b3ShapeProxy ){ &shape->center, 1, shape->radius };
	input.proxyB = *proxy;
	input.transform = b3InvMulTransforms( shapeTransform, b3Transform_identity );
	input.useRadii = true;

	b3SimplexCache cache = { 0 };
	b3DistanceOutput output = b3ShapeDistance( &input, &cache, NULL, 0 );
	return b3Raw( output.distance ) < b3Raw( B3_OVERLAP_SLOP );
}

// Precision Improvements for Ray / Sphere Intersection - Ray Tracing Gems 2019
// http://www.codercorner.com/blog/?p=321
//
// The float version's precision argument -- shift the ray so the sphere centre
// is the origin, then work with the closest point rather than solving the
// quadratic directly -- matters more here than it does upstream, because it
// keeps every intermediate small relative to the radius instead of relative to
// the world origin.
b3CastOutput b3RayCastSphere( const b3Sphere* shape, const b3RayCastInput* input )
{
	b3CastOutput output = { 0 };

	b3Vec3 p = shape->center;

	// Shift the ray so the sphere centre is the origin.
	b3Vec3 s = b3Sub( input->origin, p );

	b3f r = shape->radius;

	// rr and cc are squared lengths, kept at Q24 in int64 rather than
	// narrowed to Q12. Their difference feeds a square root, and narrowing
	// first would throw away exactly the low bits the root needs -- for a
	// grazing hit, rr and cc are nearly equal and the difference is small.
	int64_t rr = (int64_t)b3Raw( r ) * b3Raw( r );

	b3f length;
	b3Vec3 d = b3GetLengthAndNormalize( &length, input->translation );

	if ( b3Raw( length ) == 0 )
	{
		// Zero length ray: a hit only if the origin is already inside.
		if ( b3LengthSquaredWide( s ) < rr )
		{
			output.point = input->origin;
			output.hit = true;
		}

		return output;
	}

	// Closest point on the ray to the origin: solve dot(s + t*d, d) = 0.
	b3f t = b3NegF( b3Dot( s, d ) );
	b3Vec3 c = b3MulAdd( s, t, d );

	int64_t cc = b3LengthSquaredWide( c );

	if ( cc > rr )
	{
		// The closest approach is outside the sphere.
		return output;
	}

	// Pythagoras. Both operands are Q24, so b3SqrtWide gives a Q12 length.
	b3f h = b3SqrtWide( rr - cc );

	b3f fraction = b3SubF( t, h );

	// maxFraction is a coefficient and length is a length, so this compares
	// two lengths -- no scale mixing.
	if ( b3Raw( fraction ) < 0 || b3Raw( b3MulFC( length, input->maxFraction ) ) < b3Raw( fraction ) )
	{
		// The intersection lies outside the ray segment. Still a hit if the
		// ray started inside.
		if ( b3LengthSquaredWide( s ) < rr )
		{
			output.point = input->origin;
			output.hit = true;
		}

		return output;
	}

	b3Vec3 hitPoint = b3MulAdd( s, fraction, d );

	// fraction and length are both lengths and fraction <= length by the test
	// above, so the quotient lands in [0, 1] exactly where b3c wants it.
	output.fraction = b3DivFFToC( fraction, length );

	if ( b3Raw( output.fraction ) > b3Raw( input->maxFraction ) )
	{
		output.fraction = input->maxFraction;
	}

	output.normal = b3Normalize( hitPoint );
	output.point = b3MulAdd( p, shape->radius, output.normal );
	output.hit = true;

	return output;
}

// The inside-out variant: hits the shell from either side, so a ray starting
// inside reports the far wall rather than missing.
//
// One deliberate divergence from upstream. The float version normalizes the
// ray direction, so its `fraction` is a *length* along that direction -- but it
// then compares that length against input->maxFraction and stores it in
// output.fraction, both of which are coefficients in [0, 1] everywhere else in
// the library. It is inconsistent with b3RayCastSphere three lines above it and
// with every other cast. Upstream never calls this function, which is why the
// mismatch has survived.
//
// Here it cannot survive: strict mode gives b3f and b3c distinct types, so the
// transliteration does not compile. Both the test and the result are therefore
// in fractions of the ray, matching b3RayCastSphere.
b3CastOutput b3RayCastHollowSphere( const b3Sphere* sphere, const b3RayCastInput* input )
{
	b3CastOutput output = { 0 };

	b3Vec3 p = sphere->center;

	// Shift the ray so the sphere centre is the origin.
	b3Vec3 s = b3Sub( input->origin, p );

	b3f length;
	b3Vec3 d = b3GetLengthAndNormalize( &length, input->translation );

	if ( b3Raw( length ) == 0 )
	{
		// Zero length ray. Unlike the solid case there is no initial-overlap
		// hit to report: being inside the shell is not touching it.
		return output;
	}

	// Closest point on the ray to the origin: solve dot(s + t*d, d) = 0.
	b3f t = b3NegF( b3Dot( s, d ) );
	b3Vec3 c = b3MulAdd( s, t, d );

	int64_t cc = b3LengthSquaredWide( c );

	b3f r = sphere->radius;
	int64_t rr = (int64_t)b3Raw( r ) * b3Raw( r );

	if ( cc > rr )
	{
		// The closest approach misses the shell entirely.
		return output;
	}

	// Pythagoras, wide for the same reason as in b3RayCastSphere: for a
	// grazing hit rr and cc are nearly equal and the difference is where all
	// the information is.
	b3f h = b3SqrtWide( rr - cc );

	// Near wall first. If that lies behind the origin the ray started inside,
	// so the far wall is the hit.
	b3f fraction = b3SubF( t, h );

	if ( b3Raw( fraction ) < 0 )
	{
		fraction = b3AddF( t, h );
	}

	if ( b3Raw( fraction ) < 0 )
	{
		// Both walls are behind the ray.
		return output;
	}

	// Same comparison as b3RayCastSphere: a length against a length, so the
	// coefficient is scaled up rather than the distance scaled down.
	if ( b3Raw( b3MulFC( length, input->maxFraction ) ) < b3Raw( fraction ) )
	{
		return output;
	}

	b3Vec3 hitPoint = b3MulAdd( s, fraction, d );

	// fraction <= maxFraction * length <= length by the test above, so the
	// quotient is in [0, 1] and cannot saturate Q30.
	output.fraction = b3DivFFToC( fraction, length );
	output.normal = b3Normalize( hitPoint );
	output.point = b3MulAdd( p, sphere->radius, output.normal );
	output.hit = true;

	return output;
}

b3CastOutput b3ShapeCastSphere( const b3Sphere* sphere, const b3ShapeCastInput* input )
{
	b3ShapeCastPairInput pairInput;
	pairInput.proxyA = ( b3ShapeProxy ){ &sphere->center, 1, sphere->radius };
	pairInput.proxyB = input->proxy;
	pairInput.transform = b3Transform_identity;
	pairInput.translationB = input->translation;
	pairInput.maxFraction = input->maxFraction;
	pairInput.canEncroach = input->canEncroach;

	return b3ShapeCast( &pairInput );
}
