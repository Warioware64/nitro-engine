// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Erin Catto        (original Box3D)
// Copyright (c) 2026 Warioware64       (Nitro Engine Advanced fixed-point port)
//
// Dirk Gregorius contributed portions of this code
//
// This file is part of Nitro Engine Advanced

// NOTE: b3CollideMoverAndCapsule arrived with the character controller in
// Phase 7 Stage 4. It sits at the end of this file and depends on nothing else
// in it -- b3SegmentDistance is math_fixed's, not the capsule's.

#include "box3d/collision.h"
#include "box3d/constants.h"
#include "box3d/math_fixed.h"
#include "box3d/types.h"

#include "core.h"

b3MassData b3ComputeCapsuleMass( const b3Capsule* shape, b3f density )
{
	b3Vec3 c1 = shape->center1;
	b3Vec3 c2 = shape->center2;
	b3f r = shape->radius;

	b3f height = b3Distance( c1, c2 );

	// --- masses -------------------------------------------------------
	//
	// Both volumes cube a length, so both are computed wide for the same
	// reason as b3ComputeSphereMass: the intermediate reaches the Q12
	// ceiling before the result does.

	int64_t rr = (int64_t)b3Raw( r ) * b3Raw( r ); // Q24

	// Cylinder: pi * r^2 * h.  pi at Q14 = 51472.
	const int64_t piQ14 = 51472;
	int64_t cylinderVolume = ( ( ( rr * piQ14 ) >> ( 14 + B3_F_SHIFT ) ) * b3Raw( height ) ) >> B3_F_SHIFT; // Q12

	// Sphere: 4/3 * pi * r^3.  4/3 pi at Q14 = 68629.
	const int64_t fourThirdsPiQ14 = 68629; // 4.1887902 * 2^14
	int64_t r3 = ( rr >> B3_F_SHIFT ) * b3Raw( r );							 // Q24
	int64_t sphereVolume = ( r3 * fourThirdsPiQ14 ) >> ( 14 + B3_F_SHIFT );	 // Q12

	int64_t d = (int64_t)b3Raw( density );
	int64_t cylinderMass = ( cylinderVolume * d ) >> B3_F_SHIFT; // Q12
	int64_t sphereMass = ( sphereVolume * d ) >> B3_F_SHIFT;	 // Q12
	int64_t totalMass = cylinderMass + sphereMass;

	// --- inertia ------------------------------------------------------
	//
	// This is the one place the per-unit-mass convention costs something.
	// Upstream simply adds the cylinder and sphere inertia tensors, which is
	// valid because they are absolute. Per-unit-mass values cannot be added:
	// they have to be weighted by the mass each part contributes.
	//
	// The weighting is done in an int64 accumulator, and that accumulator
	// holds Q24 -- which is precisely the *absolute* inertia, the quantity
	// that overflows Q12 at radius 13. Dividing by the total mass at the end
	// brings it back to a per-unit-mass length² that cannot overflow. So the
	// dangerous value exists only as a wide intermediate and is never stored.

	b3Matrix3 uCyl = b3CylinderUnitInertia( r, height );
	b3Matrix3 uSph = b3SphereUnitInertia( r );

	// Parallel axis term: the hemispheres sit half a cylinder-height off the
	// centre, so their contribution gains 0.125*(3r + 2h)*h per unit mass.
	// This is upstream's `steiner` with the mass factored out.
	b3f steiner = b3MulFC( b3MulFF( b3AddF( b3MulFF( b3fFromInt( 3 ), r ), b3MulFF( b3fFromInt( 2 ), height ) ), height ),
						   b3cFromFrac( 1, 8 ) );

	b3f uSphX = b3AddF( uSph.cx.x, steiner );
	b3f uSphZ = b3AddF( uSph.cz.z, steiner );

	b3Matrix3 inertia = b3Mat3_zeroFn();

	if ( totalMass > 0 )
	{
		// Σ mᵢ·uᵢ at Q24, then / mass at Q12 -> Q12.
		int64_t xx = cylinderMass * (int64_t)b3Raw( uCyl.cx.x ) + sphereMass * (int64_t)b3Raw( uSphX );
		int64_t yy = cylinderMass * (int64_t)b3Raw( uCyl.cy.y ) + sphereMass * (int64_t)b3Raw( uSph.cy.y );
		int64_t zz = cylinderMass * (int64_t)b3Raw( uCyl.cz.z ) + sphereMass * (int64_t)b3Raw( uSphZ );

		inertia = b3MakeDiagonalMatrix( b3Makeb3f( b3HwDiv64( xx, (int32_t)totalMass ) ),
										b3Makeb3f( b3HwDiv64( yy, (int32_t)totalMass ) ),
										b3Makeb3f( b3HwDiv64( zz, (int32_t)totalMass ) ) );
	}

	// Align the capsule axis with the chosen up-axis. Upstream guards this
	// with height² against a float denormal threshold; here the meaningful
	// test is whether the axis direction survives Q12 at all.
	if ( b3Raw( height ) > 0 )
	{
		b3Vec3 direction = b3Normalize( b3Sub( c2, c1 ) );
		b3Quat q = b3ComputeQuatBetweenUnitVectors( b3Vec3_axisY, direction );
		b3Matrix3 rotation = b3MakeMatrixFromQuat( q );

		// Rotate the central inertia into the shape frame. Still per unit
		// mass -- a similarity transform does not change the scale.
		inertia = b3MulMM( rotation, b3MulMM( inertia, b3Transpose( rotation ) ) );
	}

	b3MassData out;
	out.mass = b3Makeb3f( (int32_t)totalMass );
	out.center = b3MulCV( b3cFromFrac( 1, 2 ), b3Add( c1, c2 ) );
	out.inertia = inertia;

	return out;
}

b3AABB b3ComputeCapsuleAABB( const b3Capsule* shape, b3Transform transform )
{
	b3f r = shape->radius;

	b3Vec3 center1 = b3TransformPoint( transform, shape->center1 );
	b3Vec3 center2 = b3TransformPoint( transform, shape->center2 );
	b3Vec3 extent = b3MakeVec3( r, r, r );

	return b3MakeAABB( b3Sub( b3Min( center1, center2 ), extent ), b3Add( b3Max( center1, center2 ), extent ) );
}

b3AABB b3ComputeSweptCapsuleAABB( const b3Capsule* shape, b3Transform xf1, b3Transform xf2 )
{
	b3f radius = shape->radius;
	b3Vec3 r = b3MakeVec3( radius, radius, radius );

	b3Vec3 a = b3TransformPoint( xf1, shape->center1 );
	b3Vec3 b = b3TransformPoint( xf1, shape->center2 );
	b3Vec3 c = b3TransformPoint( xf2, shape->center1 );
	b3Vec3 d = b3TransformPoint( xf2, shape->center2 );

	return b3MakeAABB( b3Sub( b3Min( b3Min( a, b ), b3Min( c, d ) ), r ), b3Add( b3Max( b3Max( a, b ), b3Max( c, d ) ), r ) );
}

bool b3OverlapCapsule( const b3Capsule* shape, b3Transform shapeTransform, const b3ShapeProxy* proxy )
{
	b3DistanceInput input;
	input.proxyA = ( b3ShapeProxy ){ &shape->center1, 2, shape->radius };
	input.proxyB = *proxy;
	input.transform = b3InvMulTransforms( shapeTransform, b3Transform_identity );
	input.useRadii = true;

	b3SimplexCache cache = { 0 };
	b3DistanceOutput output = b3ShapeDistance( &input, &cache, NULL, 0 );
	return b3Raw( output.distance ) < b3Raw( B3_OVERLAP_SLOP );
}

// The smallest determinant the non-parallel branch is allowed to trust.
//
// det = 1 - dot(a1, a2)^2, where a1 and a2 are Q12 unit vectors. A Q12
// component carries at most half a quantum of error, so
//
//   |d a12| <= 2^-13 * ( sum|a1| + sum|a2| ) <= 2^-13 * 2*sqrt(3) = 4.2e-4
//   |d det| = 2*|a12|*|d a12|                <= 8.5e-4   (worst case, a12 -> 1)
//
// so a determinant below about 1e-3 is entirely made of quantization noise, and
// dividing by it produces a closest-point pair with no relationship to the real
// geometry. Upstream's threshold is FLT_EPSILON, 1.19e-7 -- four orders of
// magnitude below what Q12 inputs can resolve, so transliterating it would send
// exactly the ill-conditioned cases down the branch that cannot handle them.
//
// 2^-10 = 9.8e-4 is the next power of two above the bound. Below it the
// dedicated near-parallel solver runs, which is well conditioned there.
#define B3_CAPSULE_RAY_MIN_DET ( (int64_t)1 << ( B3_W_SHIFT - 10 ) )

// Ceiling on |h| in the non-parallel branch, as a multiplier on the radius.
//
// h = sqrt( (r^2 - g2) / det ) <= r / sqrt(det), and det >= 2^-10 by the branch
// guard, so h <= 32r. Used to bound t2 before dividing.
#define B3_CAPSULE_RAY_MAX_H_SCALE 32

b3CastOutput b3RayCastCapsule( const b3Capsule* shape, const b3RayCastInput* input )
{
	b3CastOutput output = { 0 };

	b3Vec3 c1 = shape->center1;
	b3Vec3 c2 = shape->center2;
	b3f r = shape->radius;

	b3Vec3 d = b3Sub( c2, c1 );

	// Fall back to a sphere when the capsule is too short to have an axis.
	//
	// Upstream tests `lengthSquared < tol*tol` with `tol = 0.01f *
	// B3_LINEAR_SLOP`, i.e. 5e-5 units. That guard does not survive the move to
	// fixed point: 5e-5 is 0.2 in Q12 and truncates to zero, and squaring makes
	// it worse -- 2.5e-9 is 0.04 even at the Q24 of a wide squared length.
	// Transliterated the test would read `lengthSquared < 0`, never fire, and
	// let a degenerate capsule go on to normalize a zero-length axis.
	//
	// B3_MIN_CAPSULE_LENGTH is used instead. It is upstream's own name for the
	// length below which a capsule should have been a sphere, so the intent is
	// unchanged; only the threshold moves, 100x up, which is the sole direction
	// Q12 can express and the safe one.
	int64_t lengthSquared = b3LengthSquaredWide( d );
	int64_t minLength = (int64_t)b3Raw( B3_MIN_CAPSULE_LENGTH );

	if ( lengthSquared < minLength * minLength )
	{
		b3Vec3 sphereCenter = b3MulCV( b3cFromFrac( 1, 2 ), b3Add( c1, c2 ) );
		b3Sphere sphere = { sphereCenter, r };
		return b3RayCastSphere( &sphere, input );
	}

	// Vector from the first centre to the ray origin.
	b3Vec3 s = b3Sub( input->origin, c1 );

	// Capsule axis, unit length.
	b3f length;
	b3Vec3 axis = b3GetLengthAndNormalize( &length, d );

	// Project the ray origin onto the capsule axis.
	b3f u = b3Dot( s, axis );

	// Closest point on the infinite axis, relative to c1.
	b3Vec3 c = b3MulSV( u, axis );

	// Vector from that point to the ray origin, and its squared length. Wide,
	// as everywhere a squared length appears in this port.
	b3Vec3 sc = b3Sub( s, c );
	int64_t sc2 = b3LengthSquaredWide( sc );

	int64_t rr = (int64_t)b3Raw( r ) * b3Raw( r );

	// Is the ray origin inside the infinite cylinder around the axis?
	if ( sc2 < rr )
	{
		// Clamp to the bounded segment.
		b3f uClamped = b3ClampF( u, b3f_zero, length );
		b3Vec3 cp = b3MulSV( uClamped, axis );

		b3Vec3 scp = b3Sub( s, cp );
		int64_t scp2 = b3LengthSquaredWide( scp );

		if ( scp2 < rr )
		{
			// The origin is inside the capsule itself.
			output.hit = true;
			output.point = input->origin;
			return output;
		}

		// Inside the cylinder but outside the capsule means the only thing the
		// ray can strike is an endcap.
		b3Sphere sphere = { b3Add( c1, cp ), r };
		return b3RayCastSphere( &sphere, input );
	}

	// Ray axis. A zero length ray reaching here starts outside the capsule, so
	// it misses -- the same convention b3RayCastSphere uses.
	b3Vec3 dr = input->translation;
	b3f rayLength;
	b3Vec3 rayAxis = b3GetLengthAndNormalize( &rayLength, dr );

	if ( b3Raw( rayLength ) == 0 )
	{
		return output;
	}

	// Axial coordinate of the ray end point.
	b3f v = b3AddF( u, b3MulFC( b3Dot( dr, axis ), input->maxFraction ) );

	// Early out: the projected ray falls entirely off one end.
	b3f negR = b3NegF( r );
	b3f lengthPlusR = b3AddF( length, r );

	if ( ( b3Raw( u ) < b3Raw( negR ) && b3Raw( v ) < b3Raw( negR ) ) ||
		 ( b3Raw( lengthPlusR ) < b3Raw( u ) && b3Raw( lengthPlusR ) < b3Raw( v ) ) )
	{
		return output;
	}

	// Closest point between the ray and the capsule segment.
	// See Real-Time Collision Detection, section 5.1.9.
	//
	// With a1 the capsule axis and a2 the ray axis, both unit:
	//   det = 1 - a12^2
	//   t1  = (sa1 - a12*sa2) / det
	//   t2  = (a12*sa1 - sa2) / det
	//
	// a12 is a dot product of two unit vectors, so it is dimensionless and in
	// [-1, 1] -- b3iw territory. Computing it wide rather than as a Q12 b3f
	// matters: det subtracts a12^2 from one, and at Q12 a value of a12 = 0.999
	// leaves det with eight quanta of resolution. It adds no information the
	// Q12 inputs did not have, but it stops the cancellation from throwing away
	// what is there.
	b3Vec3 a1 = axis;
	b3Vec3 a2 = rayAxis;

	int64_t a12Wide = b3DotWide( a1, a2 ); // Q24
	b3iw a12 = b3Makeb3iw( (int32_t)a12Wide );

	int64_t det = ( (int64_t)1 << B3_W_SHIFT ) - ( ( a12Wide * a12Wide ) >> B3_W_SHIFT ); // Q24

	// Distance along the ray to the near intersection with the infinite
	// cylinder. A length, not a fraction.
	b3f tr;

	if ( det < B3_CAPSULE_RAY_MIN_DET )
	{
		// Near parallel. Solve the 2D problem of a ray versus the circle that
		// is the axial view of the infinite cylinder.

		// Subtracting the parallel part is cheaper than a cross product and
		// leaves a dimensionless perpendicular.
		b3Vec3 perp = b3Sub( a2, b3MulWV( a12, a1 ) );
		int64_t perp2 = b3LengthSquaredWide( perp ); // Q24

		// Origin-to-axis vector projected onto the perpendicular. A length.
		b3f beta = b3Dot( sc, perp );

		// Quadratic setup. gamma is non-negative here: the sc2 < rr case
		// returned above.
		int64_t gamma = sc2 - rr; // Q24, length^2

		int64_t betaWide = (int64_t)b3Raw( beta );
		int64_t disc = betaWide * betaWide - ( ( perp2 * gamma ) >> B3_W_SHIFT ); // Q24, length^2

		// Casting away from the axis, or the gap never closes to the radius.
		if ( b3Raw( beta ) >= 0 || disc < 0 )
		{
			return output;
		}

		// Near root, in the form that avoids the (-beta - sqrt) cancellation as
		// the ray approaches parallel.
		b3f den = b3AddF( b3NegF( beta ), b3SqrtWide( disc ) );

		if ( b3Raw( den ) == 0 )
		{
			return output;
		}

		// gamma / den is Q24 length^2 over Q12 length, which lands in Q12
		// directly. Guard the magnitude first: the divider returns 32 bits, so
		// an out-of-range quotient is undefined rather than merely large, and
		// the same comparison is the miss test anyway.
		int64_t trMax = (int64_t)b3Raw( b3MulFC( rayLength, input->maxFraction ) );

		if ( gamma > trMax * b3Raw( den ) )
		{
			return output;
		}

		tr = b3Makeb3f( b3HwDiv64( gamma, b3Raw( den ) ) );
	}
	else
	{
		// The axes are far enough from parallel to solve directly.
		b3f sa1 = u;
		b3f sa2 = b3Dot( s, a2 );

		b3f num1 = b3SubF( sa1, b3MulFW( sa2, a12 ) );
		b3f num2 = b3SubF( b3MulFW( sa1, a12 ), sa2 );

		// Bound both quotients before dividing, for the same reason as above.
		//
		// A hit needs 0 <= tr <= maxFraction*rayLength, and tr = t2 - h with
		// 0 <= h <= 32r, so t2 lies in [0, rayLength + 32r]. Given such a t2,
		// |p2| <= |s| + t2, and g2 <= r^2 requires |t1| = |p1| <= |p2| + r.
		// Anything outside those ranges cannot reach the g2 test as a hit.
		int64_t t2Max = (int64_t)b3Raw( rayLength ) + B3_CAPSULE_RAY_MAX_H_SCALE * (int64_t)b3Raw( r );
		int64_t t1Max = (int64_t)b3Raw( b3Length( s ) ) + t2Max + (int64_t)b3Raw( r );

		int64_t n1 = (int64_t)b3Raw( num1 );
		int64_t n2 = (int64_t)b3Raw( num2 );

		if ( n2 < 0 || ( n2 << B3_W_SHIFT ) > t2Max * det )
		{
			return output;
		}

		int64_t n1Abs = n1 < 0 ? -n1 : n1;

		if ( ( n1Abs << B3_W_SHIFT ) > t1Max * det )
		{
			return output;
		}

		b3f t1 = b3Makeb3f( b3HwDiv64( n1 << B3_W_SHIFT, (int32_t)det ) );
		b3f t2 = b3Makeb3f( b3HwDiv64( n2 << B3_W_SHIFT, (int32_t)det ) );

		// Closest points on the two infinite lines.
		b3Vec3 p1 = b3MulSV( t1, a1 );
		b3Vec3 p2 = b3MulAdd( s, t2, a2 );

		b3Vec3 g = b3Sub( p2, p1 );
		int64_t g2 = b3LengthSquaredWide( g );

		if ( g2 > rr )
		{
			// The closest point on the infinite ray is outside the cylinder.
			return output;
		}

		// Intersect the infinite ray with the infinite cylinder, relative to
		// the closest point, as in the sphere cast.
		//
		// h = sqrt( (r^2 - g2) / det ) is taken as two roots rather than one.
		// Shifting (r^2 - g2) up by 24 to divide at Q24 first would overflow
		// int64 for a large radius; rooting each factor keeps every
		// intermediate small, and the DS roots in hardware.
		b3f hNum = b3SqrtWide( rr - g2 );	  // Q12 length
		b3f sqrtDet = b3SqrtWide( det );	  // Q12, dimensionless

		if ( b3Raw( sqrtDet ) == 0 )
		{
			return output;
		}

		b3f h = b3Makeb3f( b3HwDiv64( (int64_t)b3Raw( hNum ) << B3_F_SHIFT, b3Raw( sqrtDet ) ) );

		tr = b3SubF( t2, h );
	}

	// Outside the ray segment?
	if ( b3Raw( tr ) < 0 || b3Raw( b3MulFC( rayLength, input->maxFraction ) ) < b3Raw( tr ) )
	{
		return output;
	}

	// The corresponding coordinate along the capsule axis.
	b3f tc = b3AddF( u, b3MulFW( tr, a12 ) );

	if ( b3Raw( tc ) < 0 )
	{
		// Past the c1 end: the hit, if any, is on that endcap.
		b3Sphere sphere = { c1, r };
		return b3RayCastSphere( &sphere, input );
	}

	if ( b3Raw( length ) < b3Raw( tc ) )
	{
		b3Sphere sphere = { c2, r };
		return b3RayCastSphere( &sphere, input );
	}

	// Hit point on the capsule side, relative to c1.
	b3Vec3 p = b3MulAdd( s, tr, rayAxis );

	// Normal points from the axis out to the hit point.
	b3Vec3 normal = b3Normalize( b3MulSub( p, tc, axis ) );

	output.point = b3Add( c1, p );
	output.normal = normal;

	// tr <= maxFraction*rayLength <= rayLength by the test above, so the
	// quotient is in [0, 1] and cannot saturate Q30.
	output.fraction = b3DivFFToC( tr, rayLength );

	if ( b3Raw( output.fraction ) > b3Raw( input->maxFraction ) )
	{
		output.fraction = input->maxFraction;
	}

	output.hit = true;
	return output;
}

int b3CollideMoverAndCapsule( b3PlaneResult* result, const b3Capsule* shape, const b3Capsule* mover )
{
	b3f totalRadius = b3AddF( mover->radius, shape->radius );

	b3SegmentDistanceResult approach =
		b3SegmentDistance( shape->center1, shape->center2, mover->center1, mover->center2 );

	// The normal points from the shape toward the mover.
	b3f distance;
	b3Vec3 normal = b3GetLengthAndNormalize( &distance, b3Sub( approach.point2, approach.point1 ) );

	if ( b3Raw( distance ) > b3Raw( totalRadius ) )
	{
		return 0;
	}

	if ( b3Raw( distance ) < b3Raw( B3_LINEAR_SLOP ) )
	{
		// Deep overlap: the core segments intersect, so pick an arbitrary
		// direction perpendicular to the mover axis. b3Perp and not
		// b3ArbitraryPerp, for the reason given in b3CollideMoverAndSphere.
		b3f moverLength;
		b3Vec3 moverAxis = b3GetLengthAndNormalize( &moverLength, b3Sub( mover->center2, mover->center1 ) );
		normal = b3Raw( moverLength ) > b3Raw( B3_LINEAR_SLOP ) ? b3Perp( moverAxis ) : b3Vec3_axisY;
		distance = b3f_zero;
	}

	b3Plane plane = { normal, b3SubF( totalRadius, distance ) };
	*result = ( b3PlaneResult ){ plane, approach.point1 };
	return 1;
}

b3CastOutput b3ShapeCastCapsule( const b3Capsule* capsule, const b3ShapeCastInput* input )
{
	b3ShapeCastPairInput pairInput;
	pairInput.proxyA = ( b3ShapeProxy ){ &capsule->center1, 2, capsule->radius };
	pairInput.proxyB = input->proxy;
	pairInput.transform = b3Transform_identity;
	pairInput.translationB = input->translation;
	pairInput.maxFraction = input->maxFraction;
	pairInput.canEncroach = input->canEncroach;

	return b3ShapeCast( &pairInput );
}
