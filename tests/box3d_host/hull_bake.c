// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Warioware64, 2026

// Baking a hull into the port's fixed-point blob.
//
// Runs on the host, so it computes in double and emits Q12 -- which is what
// makes the whole off-device hull design work: the geometry is decided at full
// precision and only the *result* is quantized.
//
// Three things here are decisions rather than transcription:
//
//   1. Points round to nearest, not toward zero. Truncation bias has been the
//      port's recurring hazard since Phase 0, and here it would pull every
//      vertex of a hull toward the origin -- shrinking it systematically.
//
//   2. Plane normals are re-derived from the quantized points rather than
//      quantized themselves. Rounding a point and rounding its face's plane
//      independently makes the two disagree: the vertex ends up in front of
//      the plane it is supposed to lie on, and every separation measured
//      against that face is wrong by the discrepancy. Fitting the plane to
//      the points that will actually be stored keeps dot(n, p) - offset near
//      zero for the face's own vertices, which is the property the clipper and
//      the separating axis test both rely on. Newell's method does the fit,
//      since a quantized face is no longer exactly planar.
//
//   3. The offset is the mean of dot(n, p) over the face's vertices, not the
//      value at one of them. That spreads the residual instead of pinning it
//      to whichever vertex was picked.
//
// The blob is then handed to the port's own b3IsValidHull. A hull that does
// not survive quantization is rejected here, at bake time, rather than
// producing quietly wrong contacts later.

#include "hull_bake.h"

#include "box3d/collision.h"
#include "box3d/math_fixed.h"
#include "box3d/types.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

static b3f quantize( double v )
{
	return b3fFromDouble( v );
}

static b3Vec3 quantizeVec( pdVec3 v )
{
	return b3MakeVec3( quantize( v.x ), quantize( v.y ), quantize( v.z ) );
}

static double dotd( const double a[3], const double b[3] )
{
	return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

/// Layout of a baked hull: header, then the five arrays.
typedef struct
{
	int vertexOffset;
	int pointOffset;
	int edgeOffset;
	int planeOffset;
	int faceOffset;
	int byteCount;
} pdBakeLayout;

static pdBakeLayout bakeLayout( const pdHull* desc )
{
	pdBakeLayout layout;
	int cursor = (int)sizeof( b3HullData );

	// b3Vec3 and b3Plane are 4-byte aligned in device mode and wider under the
	// shadow-value build, so alignment comes from the types rather than from a
	// hard-coded number.
	layout.vertexOffset = cursor;
	cursor += desc->vertexCount * (int)sizeof( b3HullVertex );

	cursor = (int)( ( cursor + sizeof( b3Vec3 ) - 1 ) / sizeof( b3Vec3 ) * sizeof( b3Vec3 ) );
	layout.pointOffset = cursor;
	cursor += desc->vertexCount * (int)sizeof( b3Vec3 );

	layout.edgeOffset = cursor;
	cursor += desc->edgeCount * (int)sizeof( b3HullHalfEdge );

	cursor = (int)( ( cursor + sizeof( b3Plane ) - 1 ) / sizeof( b3Plane ) * sizeof( b3Plane ) );
	layout.planeOffset = cursor;
	cursor += desc->faceCount * (int)sizeof( b3Plane );

	layout.faceOffset = cursor;
	cursor += desc->faceCount * (int)sizeof( b3HullFace );

	// Round the total up so an array of hulls stays aligned.
	layout.byteCount = ( cursor + 7 ) & ~7;
	return layout;
}

int pdBakeHullSize( const pdHull* desc )
{
	return bakeLayout( desc ).byteCount;
}

int pdBakeHull( const pdHull* desc, void* blob, int size )
{
	if ( desc->vertexCount <= 0 || desc->faceCount <= 0 || ( desc->edgeCount & 1 ) != 0 )
	{
		return 0;
	}

	if ( desc->vertexCount > B3_MAX_HULL_VERTICES || desc->faceCount > B3_MAX_HULL_FACES ||
		 desc->edgeCount / 2 > B3_MAX_HULL_EDGES )
	{
		return 0;
	}

	pdBakeLayout layout = bakeLayout( desc );
	if ( size < layout.byteCount )
	{
		return 0;
	}

	memset( blob, 0, (size_t)layout.byteCount );

	b3HullData* hull = (b3HullData*)blob;
	hull->version = B3_HULL_VERSION;
	hull->byteCount = layout.byteCount;
	hull->hash = 0;
	hull->vertexCount = desc->vertexCount;
	hull->edgeCount = desc->edgeCount;
	hull->faceCount = desc->faceCount;
	hull->vertexOffset = layout.vertexOffset;
	hull->pointOffset = layout.pointOffset;
	hull->edgeOffset = layout.edgeOffset;
	hull->planeOffset = layout.planeOffset;
	hull->faceOffset = layout.faceOffset;

	b3HullVertex* vertices = (b3HullVertex*)( (char*)blob + layout.vertexOffset );
	b3Vec3* points = (b3Vec3*)( (char*)blob + layout.pointOffset );
	b3HullHalfEdge* edges = (b3HullHalfEdge*)( (char*)blob + layout.edgeOffset );
	b3Plane* planes = (b3Plane*)( (char*)blob + layout.planeOffset );
	b3HullFace* faces = (b3HullFace*)( (char*)blob + layout.faceOffset );

	// Quantized points, kept alongside in double so the plane fit sees exactly
	// what was stored rather than what was asked for.
	double quantized[B3_MAX_HULL_VERTICES][3];

	for ( int i = 0; i < desc->vertexCount; ++i )
	{
		points[i] = quantizeVec( desc->points[i] );
		quantized[i][0] = b3fToDouble( points[i].x );
		quantized[i][1] = b3fToDouble( points[i].y );
		quantized[i][2] = b3fToDouble( points[i].z );

		vertices[i].edge = desc->vertexEdge[i];
	}

	for ( int i = 0; i < desc->edgeCount; ++i )
	{
		edges[i].next = desc->edges[i].next;
		edges[i].twin = desc->edges[i].twin;
		edges[i].origin = desc->edges[i].origin;
		edges[i].face = desc->edges[i].face;
	}

	for ( int f = 0; f < desc->faceCount; ++f )
	{
		faces[f].edge = desc->faceEdge[f];

		// Newell's method over the quantized face loop. A face that was planar
		// in float is not quite planar once its vertices move, and Newell is
		// the fit that minimizes the residual over all of them rather than
		// trusting three.
		double normal[3] = { 0.0, 0.0, 0.0 };
		double centroid[3] = { 0.0, 0.0, 0.0 };
		int count = 0;

		int base = desc->faceEdge[f];
		int e = base;
		do
		{
			int origin = desc->edges[e].origin;
			int nextOrigin = desc->edges[desc->edges[e].next].origin;

			const double* p = quantized[origin];
			const double* q = quantized[nextOrigin];

			normal[0] += ( p[1] - q[1] ) * ( p[2] + q[2] );
			normal[1] += ( p[2] - q[2] ) * ( p[0] + q[0] );
			normal[2] += ( p[0] - q[0] ) * ( p[1] + q[1] );

			centroid[0] += p[0];
			centroid[1] += p[1];
			centroid[2] += p[2];
			count++;

			e = desc->edges[e].next;

			if ( count > desc->edgeCount )
			{
				// The face loop does not close: the description is broken.
				return 0;
			}
		}
		while ( e != base );

		double length = sqrt( dotd( normal, normal ) );
		if ( length == 0.0 || count == 0 )
		{
			// The face collapsed under quantization.
			return 0;
		}

		normal[0] /= length;
		normal[1] /= length;
		normal[2] /= length;

		centroid[0] /= count;
		centroid[1] /= count;
		centroid[2] /= count;

		// Offset from the face centroid, which is the mean of dot(n, p) for a
		// planar face and the least-squares answer for one that is not.
		double offset = dotd( normal, centroid );

		planes[f].normal = b3MakeVec3( quantize( normal[0] ), quantize( normal[1] ), quantize( normal[2] ) );
		planes[f].offset = quantize( offset );
	}

	// Bulk properties. These are computed in double by whoever produced the
	// description, so they only need quantizing -- and checking, because Q12
	// runs out of room for a volume long before the world does.
	hull->center = quantizeVec( desc->center );
	hull->volume = quantize( desc->volume );
	hull->surfaceArea = quantize( desc->surfaceArea );
	hull->innerRadius = quantize( desc->innerRadius );

	if ( desc->volume > 500000.0 || desc->surfaceArea > 500000.0 )
	{
		// Past the Q12 ceiling. A cube of side 80 is the practical limit.
		return 0;
	}

	b3f* inertia = &hull->centralInertia.cx.x;
	for ( int i = 0; i < 9; ++i )
	{
		inertia[i] = quantize( desc->unitInertia[i] );
	}

	// Bounds from the quantized points, so they contain what is stored rather
	// than what was asked for.
	b3AABB bounds = b3MakeAABB( points[0], points[0] );
	for ( int i = 1; i < desc->vertexCount; ++i )
	{
		bounds = b3AABB_AddPoint( bounds, points[i] );
	}
	hull->aabb = bounds;

	// The port's own validator has the last word. A hull that quantization has
	// made non-convex is rejected here rather than at contact time.
	if ( b3IsValidHull( hull ) == false )
	{
		return 0;
	}

	return layout.byteCount;
}
