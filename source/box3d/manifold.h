// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Erin Catto        (original Box3D)
// Copyright (c) 2026 Warioware64       (Nitro Engine Advanced fixed-point port)
//
// This file is part of Nitro Engine Advanced

#pragma once

/// @file   manifold.h
/// @brief  Internal types shared by the manifold generators.
///
/// The types are here; the hull-oriented helpers that operate on them live in
/// manifold.c (b3FindIncidentFace, b3ClipPolygon, b3ValidatePolygon) and in
/// convex_manifold.c (b3ComputeSeparatingAxis and the contact builders below
/// it), which is upstream's split.

#include "box3d/math_fixed.h"
#include "box3d/types.h"

/// Upstream sizes the clip buffer at 64. Face polygons are bounded by the
/// hull vertex count, which nea_config.h caps at 32, so 32 is the real
/// bound -- and it halves a buffer that lives on the stack. The DS stack is
/// small enough that a b3ClipVertex[64] pair is worth not having.
///
/// This is not a *proved* bound, and b3ClipPolygon guards accordingly.
/// Sutherland-Hodgman on an n-gon against m half-spaces yields at most n + m
/// vertices, so the buffer is provably enough exactly when the two faces
/// involved have degree at most 16 apiece. B3_MAX_HULL_EDGES is 32 full
/// edges, and a face of degree d forces E >= 3d/2 -- its d vertices each need
/// a third edge, shared by at most two of them -- so d may in principle reach
/// 21 and two such faces would want 42 slots. Every hull the port can
/// actually produce is far below that: a box face is 4, a prism cap at most
/// 10, the baked cylinder and cone 8, a quickhull rock 3. The guard is what
/// makes the gap safe rather than merely unlikely.
#define B3_MAX_CLIP_POINTS 32

typedef struct b3SeparatingAxis
{
	b3Vec3 normal;
	b3f separation;
	int indexA;
	int indexB;
	b3SeparatingFeature type;
} b3SeparatingAxis;

typedef struct b3AxisQuery
{
	b3SeparatingAxis faceA;
	b3SeparatingAxis faceB;
	b3SeparatingAxis edge;
	b3SeparatingFeature separatedFeature;
} b3AxisQuery;

typedef struct b3ClipVertex
{
	b3Vec3 position;
	b3f separation;
	b3FeaturePair pair;
} b3ClipVertex;

typedef enum b3FeatureOwner
{
	b3_featureShapeA = 0,
	b3_featureShapeB = 1
} b3FeatureOwner;

static inline b3FeaturePair b3MakeFeaturePair( b3FeatureOwner owner1, int index1, b3FeatureOwner owner2, int index2 )
{
	b3FeaturePair pair;
	pair.owner1 = (uint8_t)owner1;
	pair.index1 = (uint8_t)index1;
	pair.owner2 = (uint8_t)owner2;
	pair.index2 = (uint8_t)index2;
	return pair;
}

/// Swap the roles of A and B in a feature pair.
///
/// The owner complement is the half that looks wrong, and it is the half that
/// is load bearing: it makes the pair -- and therefore b3MakeFeatureId, and
/// therefore the warm-started impulse -- independent of which hull supplied
/// the reference face. (A,i,B,j) maps to (A,j,B,i), *not* to (B,j,A,i). A
/// reference face flip-flopping between A and B from one step to the next
/// must not look like a new contact to the solver.
static inline b3FeaturePair b3FlipPair( b3FeaturePair pair )
{
	B3_ASSERT( pair.owner1 <= 1 && pair.owner2 <= 1 );

	b3FeaturePair flipped;
	flipped.owner1 = (uint8_t)( 1 - pair.owner2 );
	flipped.index1 = pair.index2;
	flipped.owner2 = (uint8_t)( 1 - pair.owner1 );
	flipped.index2 = pair.index1;
	return flipped;
}

/// Pack a feature pair into the identifier the solver matches warm-start
/// impulses on.
static inline uint32_t b3MakeFeatureId( b3FeaturePair pair )
{
	return ( (uint32_t)pair.owner1 << 24 ) | ( (uint32_t)pair.index1 << 16 ) | ( (uint32_t)pair.owner2 << 8 ) |
		   (uint32_t)pair.index2;
}

/// The axis of least penetration among the three candidates.
///
/// Note this is *not* how b3CollideHulls chooses its contact builder. That
/// compares the two face axes, builds the face contact, and only then
/// reconsiders the edge axis against the *clipped* separation -- a different
/// and better decision, because the clip knows something the axis query does
/// not. This is here for consumers of b3ComputeSeparatingAxis that only want
/// the axis, and for the triangle path in Phase 5.
static inline b3SeparatingAxis b3GetBestAxis( const b3AxisQuery* query )
{
	if ( b3Raw( query->faceA.separation ) > b3Raw( query->faceB.separation ) )
	{
		return b3Raw( query->edge.separation ) > b3Raw( query->faceA.separation ) ? query->edge : query->faceA;
	}

	return b3Raw( query->edge.separation ) > b3Raw( query->faceB.separation ) ? query->edge : query->faceB;
}

// =========================================================================
// manifold.c
// =========================================================================

/// The face of `hull` most opposed to a reference normal, searched from the
/// support vertex outwards.
///
/// Not simply "the most anti-parallel face": upstream extended this to pick
/// the edge at `vertexIndex` most perpendicular to the reference normal and
/// then the more opposed of that edge's two faces, because the naive search
/// picks the wrong face on wedges. See the implementation.
int b3FindIncidentFace( const b3HullData* hull, b3Vec3 refNormal, int vertexIndex );

/// Clip one convex polygon against one plane (Sutherland-Hodgman).
///
/// Separations on the output are measured against `refPlane`, not against the
/// clip plane, so `clipPlane` need not carry a unit normal.
///
/// @note The clip edge is recorded on the output pairs as belonging to
/// **shape A**, which is to say this assumes the reference face is A's. A
/// caller for which the reference is B -- b3BuildFaceBContact, and the
/// triangle path's hull-face collider -- must run b3FlipPair over the result.
///
/// @return the output vertex count, or B3_NULL_INDEX if the result would not
/// fit B3_MAX_CLIP_POINTS. Upstream has no output bound at all; a blob that
/// reaches here unvalidated must fail rather than overrun.
int b3ClipPolygon( b3ClipVertex* out, const b3ClipVertex* polygon, int count, b3Plane clipPlane, int edge,
				   b3Plane refPlane );

/// Clip a segment against a plane, keeping the part behind it.
///
/// @return the number of vertices written back into `segment`, 0 to 2.
int b3ClipSegment( b3ClipVertex segment[2], b3Plane plane );

/// Unit normal from an offset whose squared length is already known.
///
/// The fixed-point answer to upstream's `distance*distance > 1000*FLT_MIN`:
/// an offset is degenerate exactly when its squared length quantizes to zero,
/// there being no denormal range here. Falls back to upstream's arbitrary +Y.
b3Vec3 b3NormalOrUp( b3Vec3 offset, int64_t distanceSq );

/// Contact point midway between the two surfaces.
b3Vec3 b3MidpointContact( b3Vec3 pA, b3f rA, b3Vec3 pB, b3f rB, b3Vec3 normal );

/// Narrow a separation held wide at Q24 down to the b3f a manifold stores.
///
/// Candidate separations stay wide until they leave the axis test, because
/// the margin between two face axes can be a single Q12 quantum and narrowing
/// inside the loop would let rounding choose the reference face. This is the
/// one place the narrowing happens.
b3f b3SeparationFromWide( int64_t wide );

/// Twice the signed area of the triangle spanned by u and v, about `normal`,
/// at Q24.
///
/// Not `b3Dot( normal, b3Cross( u, v ) )`: a cross product is an area and
/// b3Cross narrows back to Q12, where two operands a hundredth of a unit long
/// cross to exactly zero. Kept wide throughout.
int64_t b3SignedAreaWide( b3Vec3 normal, b3Vec3 u, b3Vec3 v );

/// Cut a manifold down to at most four points, keeping the ones that hold the
/// contact patch open.
///
/// @note Modifies `points`: each accepted point is removed by swapping the
/// last one over it, which is why `count` is by value.
void b3ReduceManifoldPoints( b3LocalManifold* manifold, int capacity, b3LocalManifoldPoint* points, int count );

#if B3_ENABLE_VALIDATION
/// The incoming and outgoing feature edges must chain around the polygon.
///
/// An exact integer check on indices, so unlike B3_VALIDATE -- which is a
/// float-tolerance tier with no meaning here -- this one survives the port.
bool b3ValidatePolygon( const b3ClipVertex* polygon, int count );
#endif

// =========================================================================
// convex_manifold.c
// =========================================================================

/// Separating axis test over both hulls' faces and all of their edge pairs.
///
/// @param earlyReturn stop at the first axis separating by more than
/// B3_SPECULATIVE_DISTANCE, recording it in `separatedFeature`. With this
/// false every axis is scored, which is what the b3_manual* cache types and
/// the tests want.
b3AxisQuery b3ComputeSeparatingAxis( const b3HullData* hullA, const b3HullData* hullB, b3Transform transformBtoA,
									 bool earlyReturn );
