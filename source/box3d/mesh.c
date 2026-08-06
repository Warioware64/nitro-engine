// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Erin Catto        (original Box3D)
// Copyright (c) 2026 Warioware64       (Nitro Engine Advanced fixed-point port)
//
// This file is part of Nitro Engine Advanced

/// @file   mesh.c
/// @brief  Triangle mesh queries. The read half of upstream's mesh.c.
///
/// @section scope What is here, and what is deliberately not
///
/// Upstream's mesh.c is 2415 lines, and about 1100 of them are the builder:
/// vertex welding behind a 2000-line hash-table header, a surface-area-heuristic
/// BVH split, the depth-first triangle sort and the shared-edge classification.
/// **None of that is ported**, for the same reason none of the quickhull
/// builder is (see hull.c): meshes are built on the host at full precision and
/// baked into the flat blob that b3MeshData already is, so the device gets the
/// query half and nothing here allocates.
///
/// Absent, and where each one went:
///
///   b3CreateMesh, b3DestroyMesh, b3WeldVertices, b3SplitBinnedSah,
///   b3SplitMedian, b3BuildRecursive, b3SortMeshTriangles,
///   b3IdentifyEdges                                  -- host baking, see
///                                                       tests/box3d_host/mesh_bake.c
///   b3CreateGridMesh / Wave / Torus / Box / HollowBox
///   / Platform                                       -- dropped: upstream's
///                                                       sample generators
///   b3RayCastMesh, b3ShapeCastMesh, b3OverlapMesh    -- Phase 7, with the rest
///                                                       of the query layer
///   b3CollideMoverAndMesh                            -- Phase 7 Stage 4, with
///                                                       the character mover
///
/// @section simd No b3V32
///
/// Upstream funnels the traversal through a `b3V32` vector type that has an
/// SSE2 form and a scalar fallback -- and the scalar fallback is literally
/// `struct { float x, y, z; }`, which is `b3Vec3`. There is nothing for a
/// second type to buy here, so the two overlap tests below are written against
/// b3Vec3 directly and the port has no simd.h at all.

#include "mesh.h"

#include "aabb.h"
#include "core.h"

#include "box3d/constants.h"

#include <stddef.h>

// =========================================================================
// Scale
// =========================================================================

b3Vec3 b3SafeScale( b3Vec3 scale )
{
	// Keep the sign -- a negative component is a reflection, which is
	// meaningful -- but never let the magnitude reach zero. b3QueryMesh takes
	// a reciprocal of this, and B3_MIN_SCALE is what bounds it at 100.
	b3f min = B3_MIN_SCALE;

	b3f x = b3AbsF( scale.x );
	b3f y = b3AbsF( scale.y );
	b3f z = b3AbsF( scale.z );

	if ( b3Raw( x ) < b3Raw( min ) )
	{
		x = min;
	}

	if ( b3Raw( y ) < b3Raw( min ) )
	{
		y = min;
	}

	if ( b3Raw( z ) < b3Raw( min ) )
	{
		z = min;
	}

	return b3MakeVec3( b3Raw( scale.x ) < 0 ? b3NegF( x ) : x, b3Raw( scale.y ) < 0 ? b3NegF( y ) : y,
					   b3Raw( scale.z ) < 0 ? b3NegF( z ) : z );
}

/// Does this scale reflect the mesh?
///
/// Upstream asks `scale.x * scale.y * scale.z < 0`. The port must not: that
/// product is cubic in the scale and would overflow or underflow Q12 for no
/// reason, when all the question ever wanted was the parity of three signs.
static inline bool b3IsReflected( b3Vec3 scale )
{
	int signs = ( b3Raw( scale.x ) < 0 ) + ( b3Raw( scale.y ) < 0 ) + ( b3Raw( scale.z ) < 0 );
	return ( signs & 1 ) != 0;
}

/// The overwhelmingly common case: a static level, baked at the size it is
/// used. Worth testing for, because it skips a reciprocal, the min/max dance
/// that a possibly-reflected scale needs, and a multiply per vertex.
static inline bool b3IsUnitScale( b3Vec3 scale )
{
	return b3Raw( scale.x ) == B3_F_ONE && b3Raw( scale.y ) == B3_F_ONE && b3Raw( scale.z ) == B3_F_ONE;
}

// =========================================================================
// Bounds
// =========================================================================

b3AABB b3ComputeMeshAABB( const b3MeshData* shape, b3Transform transform, b3Vec3 scale )
{
	// The baked bounds, scaled and then transformed. No traversal: the header
	// already holds the union over every vertex.
	b3Vec3 scaledLower = b3Mul( scale, shape->bounds.lowerBound );
	b3Vec3 scaledUpper = b3Mul( scale, shape->bounds.upperBound );

	// A negative scale component swaps that axis's bounds, so re-normalize
	// rather than assuming lower <= upper.
	b3AABB bounds = b3MakeAABB( b3Min( scaledLower, scaledUpper ), b3Max( scaledLower, scaledUpper ) );
	return b3AABB_Transform( transform, bounds );
}

// =========================================================================
// The two overlap tests
// =========================================================================
//
// Both are exact integer predicates: every comparison is against zero, so
// nothing is ever rescaled and no rounding decision is taken at all. What the
// port has to supply is *range*, and that is where the work is.

/// A vector of raw fixed-point components, widened.
///
/// The separating axis test below multiplies coordinates by each other twice
/// over, so its terms are quadratic and cubic in a Q12 value. They do not fit
/// in 32 bits and there is no scale at which they would; carrying them wide is
/// the whole technique.
typedef struct
{
	int64_t x, y, z;
} b3SatVec;

static inline b3SatVec b3SatFromVec( b3Vec3 v )
{
	b3SatVec r = { b3Raw( v.x ), b3Raw( v.y ), b3Raw( v.z ) };
	return r;
}

static inline b3SatVec b3SatSub( b3SatVec a, b3SatVec b )
{
	b3SatVec r = { a.x - b.x, a.y - b.y, a.z - b.z };
	return r;
}

static inline b3SatVec b3SatAdd( b3SatVec a, b3SatVec b )
{
	b3SatVec r = { a.x + b.x, a.y + b.y, a.z + b.z };
	return r;
}

static inline b3SatVec b3SatNeg( b3SatVec a )
{
	b3SatVec r = { -a.x, -a.y, -a.z };
	return r;
}

static inline b3SatVec b3SatAbs( b3SatVec a )
{
	b3SatVec r = { a.x < 0 ? -a.x : a.x, a.y < 0 ? -a.y : a.y, a.z < 0 ? -a.z : a.z };
	return r;
}

static inline b3SatVec b3SatMin( b3SatVec a, b3SatVec b )
{
	b3SatVec r = { a.x < b.x ? a.x : b.x, a.y < b.y ? a.y : b.y, a.z < b.z ? a.z : b.z };
	return r;
}

static inline b3SatVec b3SatMax( b3SatVec a, b3SatVec b )
{
	b3SatVec r = { a.x > b.x ? a.x : b.x, a.y > b.y ? a.y : b.y, a.z > b.z ? a.z : b.z };
	return r;
}

static inline b3SatVec b3SatCross( b3SatVec a, b3SatVec b )
{
	b3SatVec r = { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x };
	return r;
}

/// Cross product with the subtractions turned into additions.
///
/// Upstream's name. It is what projects a box's extent onto an edge-cross
/// axis without needing three absolute values afterwards.
static inline b3SatVec b3SatModifiedCross( b3SatVec a, b3SatVec b )
{
	b3SatVec r = { a.y * b.z + a.z * b.y, a.z * b.x + a.x * b.z, a.x * b.y + a.y * b.x };
	return r;
}

static inline int64_t b3SatDot( b3SatVec a, b3SatVec b )
{
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

static inline bool b3SatAnyPositive( b3SatVec a )
{
	return a.x > 0 || a.y > 0 || a.z > 0;
}

/// Do two AABBs overlap? Touching counts as overlapping.
static inline bool b3TestBoundsOverlap( b3Vec3 min1, b3Vec3 max1, b3Vec3 min2, b3Vec3 max2 )
{
	b3SatVec separation =
		b3SatMax( b3SatSub( b3SatFromVec( min2 ), b3SatFromVec( max1 ) ), b3SatSub( b3SatFromVec( min1 ), b3SatFromVec( max2 ) ) );

	return b3SatAnyPositive( separation ) == false;
}

/// Does a triangle overlap an axis-aligned box, given the box's centre and
/// half-extent?
///
/// The thirteen-axis separating axis test: three box faces, the triangle's own
/// plane, and the nine edge-edge crosses. Early-out on the first separating
/// axis found, cheapest first.
///
/// @section range What bounds the inputs
///
/// Everything is measured relative to the box centre, and the terms grow with
/// the *triangle's* reach from that centre rather than with the world size --
/// so a level may be any size, but one triangle may not be arbitrarily large.
///
/// Let R be the furthest any vertex sits from the box centre, in raw Q12 units.
/// The edge terms are quadratic, at worst 4R², which is nothing. The triangle's
/// own plane costs a scalar triple product: the normal is at worst 8R² and
/// dotting it with a vertex gives at worst 24R³. Against int64's 9.2e18 that
/// needs R < 7.3e5, or about **177 units** -- a triangle up to ~350 units
/// across. A DS level is built from triangles far smaller than that, and the
/// debug assert below says so if one is not.
///
/// @section rounding Which way borderline cases fall
///
/// There is no rounding: the products are exact and the comparisons are
/// against zero. What is inexact is the *input* -- the vertices were quantized
/// when the mesh was baked, so a triangle can sit a fraction of a quantum on
/// either side of the truth. That is resolved by testing `> 0` rather than
/// `>= 0`, so a triangle exactly touching the box is reported as overlapping.
/// Erring toward overlap costs one wasted triangle collide; erring the other
/// way costs a contact.
static bool b3TestBoundsTriangleOverlap( b3Vec3 boxCenter, b3Vec3 boxExtent, b3Vec3 vertex1, b3Vec3 vertex2, b3Vec3 vertex3 )
{
	b3SatVec center = b3SatFromVec( boxCenter );
	b3SatVec extent = b3SatFromVec( boxExtent );

	// Everything is relative to the box centre from here on, which is what
	// keeps the cubic term bounded by the triangle's size rather than by its
	// distance from the world origin.
	b3SatVec v1 = b3SatSub( b3SatFromVec( vertex1 ), center );
	b3SatVec v2 = b3SatSub( b3SatFromVec( vertex2 ), center );
	b3SatVec v3 = b3SatSub( b3SatFromVec( vertex3 ), center );

#if defined( B3_ENABLE_ASSERT )
	{
		// The range budget above, checked rather than assumed. 7.3e5 raw is
		// 177 units from the box centre.
		const int64_t reach = 730000;
		b3SatVec worst = b3SatMax( b3SatAbs( v1 ), b3SatMax( b3SatAbs( v2 ), b3SatAbs( v3 ) ) );
		B3_ASSERT( worst.x < reach && worst.y < reach && worst.z < reach );
	}
#endif

	// Axes 1-3: the box's own faces.
	b3SatVec triangleMin = b3SatMin( v1, b3SatMin( v2, v3 ) );
	b3SatVec triangleMax = b3SatMax( v1, b3SatMax( v2, v3 ) );

	b3SatVec faceSeparation = b3SatMax( b3SatSub( triangleMin, extent ), b3SatNeg( b3SatAdd( triangleMax, extent ) ) );
	if ( b3SatAnyPositive( faceSeparation ) )
	{
		return false;
	}

	b3SatVec edge1 = b3SatSub( v2, v1 );
	b3SatVec edge2 = b3SatSub( v3, v2 );
	b3SatVec edge3 = b3SatSub( v1, v3 );

	// Axis 4: the triangle's own plane. The only cubic term in the test --
	// normal is Q24, dotting it with a Q12 vertex gives Q36.
	{
		b3SatVec normal = b3SatCross( edge1, edge2 );

		int64_t distance = b3SatDot( normal, v1 );
		if ( distance < 0 )
		{
			distance = -distance;
		}

		if ( distance > b3SatDot( b3SatAbs( normal ), extent ) )
		{
			return false;
		}
	}

	// Axes 5-13: each triangle edge crossed with each box axis, nine at once
	// because the box axes are the coordinate axes. Quadratic, so these are
	// the cheap ones.
	//
	// Each is |cross(edge, sum of the two vertices not on it)| minus the
	// triangle's own reach along that axis, minus the box's.
	{
		b3SatVec separation =
			b3SatSub( b3SatSub( b3SatAbs( b3SatCross( edge1, b3SatAdd( v1, v3 ) ) ), b3SatAbs( b3SatCross( edge1, edge3 ) ) ),
					  b3SatModifiedCross( b3SatAbs( edge1 ), b3SatAdd( extent, extent ) ) );
		if ( b3SatAnyPositive( separation ) )
		{
			return false;
		}
	}

	{
		b3SatVec separation =
			b3SatSub( b3SatSub( b3SatAbs( b3SatCross( edge2, b3SatAdd( v1, v2 ) ) ), b3SatAbs( b3SatCross( edge2, edge1 ) ) ),
					  b3SatModifiedCross( b3SatAbs( edge2 ), b3SatAdd( extent, extent ) ) );
		if ( b3SatAnyPositive( separation ) )
		{
			return false;
		}
	}

	{
		b3SatVec separation =
			b3SatSub( b3SatSub( b3SatAbs( b3SatCross( edge3, b3SatAdd( v2, v3 ) ) ), b3SatAbs( b3SatCross( edge3, edge2 ) ) ),
					  b3SatModifiedCross( b3SatAbs( edge3 ), b3SatAdd( extent, extent ) ) );
		if ( b3SatAnyPositive( separation ) )
		{
			return false;
		}
	}

	return true;
}

// =========================================================================
// Triangle fetch
// =========================================================================

b3Triangle b3GetMeshTriangle( const b3Mesh* mesh, int triangleIndex )
{
	B3_ASSERT( 0 <= triangleIndex && triangleIndex < mesh->data->triangleCount );

	const b3MeshTriangle* triangles = b3GetMeshTriangles( mesh->data );
	const uint8_t* flags = b3GetMeshFlags( mesh->data );
	const b3Vec3* vertices = b3GetMeshVertices( mesh->data );

	b3MeshTriangle triangle = triangles[triangleIndex];
	uint8_t triangleFlags = flags != NULL ? flags[triangleIndex] : 0;

	b3Vec3 scale = mesh->scale;

	b3Triangle result;
	result.vertices[0] = b3Mul( scale, vertices[triangle.index1] );
	result.i1 = triangle.index1;

	if ( b3IsReflected( scale ) )
	{
		result.vertices[1] = b3Mul( scale, vertices[triangle.index3] );
		result.vertices[2] = b3Mul( scale, vertices[triangle.index2] );

		result.i2 = triangle.index3;
		result.i3 = triangle.index2;

		// The mesh is inside out, so what was concave is now convex: the
		// inverse-concave bits become the concave ones and the inverse bits
		// are dropped. Upstream's asymmetry, kept -- a reflected mesh has no
		// second reflection to describe.
		result.flags = 0;
		result.flags |= ( triangleFlags & b3_inverseConcaveEdge1 ) ? b3_concaveEdge1 : 0;
		result.flags |= ( triangleFlags & b3_inverseConcaveEdge2 ) ? b3_concaveEdge2 : 0;
		result.flags |= ( triangleFlags & b3_inverseConcaveEdge3 ) ? b3_concaveEdge3 : 0;
	}
	else
	{
		result.vertices[1] = b3Mul( scale, vertices[triangle.index2] );
		result.vertices[2] = b3Mul( scale, vertices[triangle.index3] );

		result.i2 = triangle.index2;
		result.i3 = triangle.index3;
		result.flags = triangleFlags;
	}

	return result;
}

// =========================================================================
// Traversal
// =========================================================================

void b3QueryMesh( const b3Mesh* mesh, b3AABB bounds, b3MeshQueryFcn* fcn, void* context )
{
	const b3MeshData* data = mesh->data;
	B3_ASSERT( data != NULL && data->nodeCount > 0 );

	b3Vec3 scale = mesh->scale;
	bool unitScale = b3IsUnitScale( scale );
	bool reflected = b3IsReflected( scale );

	// The whole descent happens in the blob's own unscaled space, so the query
	// box comes to the mesh rather than every node going to the query. That is
	// one reciprocal per query instead of six multiplies per node.
	//
	// A negative scale component swaps that axis's bounds, hence the min/max
	// after dividing rather than a straight copy.
	b3AABB local = bounds;
	if ( unitScale == false )
	{
		b3Vec3 invScale = b3MakeVec3( b3DivFF( b3f_one, scale.x ), b3DivFF( b3f_one, scale.y ), b3DivFF( b3f_one, scale.z ) );
		b3Vec3 corner1 = b3Mul( invScale, bounds.lowerBound );
		b3Vec3 corner2 = b3Mul( invScale, bounds.upperBound );
		local = b3MakeAABB( b3Min( corner1, corner2 ), b3Max( corner1, corner2 ) );
	}

	b3Vec3 localCenter = b3AABB_Center( local );
	b3Vec3 localExtent = b3AABB_Extents( local );

	const b3MeshTriangle* triangles = b3GetMeshTriangles( data );
	const b3Vec3* vertices = b3GetMeshVertices( data );

	const b3MeshNode* stack[B3_MESH_STACK_SIZE];
	int count = 0;

	const b3MeshNode* node = b3GetMeshRoot( data );

	while ( true )
	{
		if ( b3TestBoundsOverlap( node->lowerBound, node->upperBound, local.lowerBound, local.upperBound ) )
		{
			if ( b3IsLeaf( node ) )
			{
				int triangleCount = (int)node->data.asLeaf.triangleCount;
				int triangleOffset = (int)node->triangleOffset;

				for ( int index = 0; index < triangleCount; ++index )
				{
					int triangleIndex = triangleOffset + index;
					b3MeshTriangle triangle = triangles[triangleIndex];

					b3Vec3 vertex1 = vertices[triangle.index1];
					b3Vec3 vertex2 = vertices[triangle.index2];
					b3Vec3 vertex3 = vertices[triangle.index3];

					// Unscaled space again, and winding does not matter to an
					// overlap test, so the reflected case needs no special
					// handling until the vertices are handed out.
					if ( b3TestBoundsTriangleOverlap( localCenter, localExtent, vertex1, vertex2, vertex3 ) == false )
					{
						continue;
					}

					bool keepGoing;
					if ( unitScale )
					{
						keepGoing = fcn( vertex1, vertex2, vertex3, triangleIndex, context );
					}
					else
					{
						b3Vec3 a = b3Mul( scale, vertex1 );
						b3Vec3 b = b3Mul( scale, reflected ? vertex3 : vertex2 );
						b3Vec3 c = b3Mul( scale, reflected ? vertex2 : vertex3 );
						keepGoing = fcn( a, b, c, triangleIndex, context );
					}

					if ( keepGoing == false )
					{
						return;
					}
				}
			}
			else
			{
				// Left child first, always. Combined with the baker storing
				// triangles in depth-first leaf order, that is what makes the
				// indices this emits ascending -- see the note on b3QueryMesh.
				B3_ASSERT( count < B3_MESH_STACK_SIZE );
				stack[count++] = b3GetRightChild( node );
				node = b3GetLeftChild( node );
				continue;
			}
		}

		if ( count == 0 )
		{
			break;
		}

		node = stack[--count];
	}
}

// =========================================================================
// Queries
// =========================================================================
//
// b3OverlapMesh, b3RayCastMesh and b3ShapeCastMesh: the three query entry
// points that Phase 5 deferred to Phase 7 with the rest of the query layer.
//
// All three are the same descent b3QueryMesh above already does, with a
// different leaf test. What differs between them and b3QueryMesh is the
// *order* they visit children in, and that is a deliberate divergence -- see
// b3DescendRayFirst.

/// The mesh scale, resolved once per query.
///
/// The descent happens in the blob's own unscaled space, so the query comes to
/// the mesh rather than every node going to the query. That is one reciprocal
/// per query instead of six multiplies per node, and it is why this is computed
/// up front rather than inside the loop.
typedef struct
{
	b3Vec3 scale;	 ///< As given.
	b3Vec3 invScale; ///< 1/scale, componentwise. Identity when unitScale.
	bool unitScale;	 ///< Skip every multiply and the reciprocals.
	bool reflected;	 ///< Negative determinant: winding reverses.
} b3MeshScaling;

static inline b3MeshScaling b3MakeMeshScaling( b3Vec3 scale )
{
	b3MeshScaling s;
	s.scale = scale;
	s.unitScale = b3IsUnitScale( scale );
	s.reflected = b3IsReflected( scale );

	if ( s.unitScale )
	{
		s.invScale = b3MakeVec3( b3f_one, b3f_one, b3f_one );
	}
	else
	{
		s.invScale = b3MakeVec3( b3DivFF( b3f_one, scale.x ), b3DivFF( b3f_one, scale.y ), b3DivFF( b3f_one, scale.z ) );
	}

	return s;
}

/// A point taken into the blob's unscaled space.
static inline b3Vec3 b3ToMeshLocal( const b3MeshScaling* s, b3Vec3 v )
{
	return s->unitScale ? v : b3Mul( s->invScale, v );
}

/// A blob vertex taken out into the shape's frame.
static inline b3Vec3 b3FromMeshLocal( const b3MeshScaling* s, b3Vec3 v )
{
	return s->unitScale ? v : b3Mul( s->scale, v );
}

/// Which child a ray-like query should descend into first.
///
/// **This is where the query traversals part company with b3QueryMesh.** That
/// one always takes the left child, and its comment explains why: combined with
/// the baker's depth-first triangle order, left-first is what makes the indices
/// it reports ascending, which the contact code relies on.
///
/// A ray has no use for ascending indices and a large use for *front to back*:
/// every triangle it hits shortens the segment, and a shortened segment rejects
/// nodes that a full-length one would have descended into. Visiting the near
/// child first is what makes that pruning actually happen rather than merely be
/// possible. The node's stored split axis and the sign of the ray's direction
/// along it are enough to know which child is near.
///
/// The consequence, and it is a real one: **ray and shape cast results are not
/// index-ordered**, where b3QueryMesh's are. Nothing reads them that way -- both
/// return a single best hit -- but a future caller that wants a *list* of hits
/// must not assume the order.
///
/// @return true to push the right child and descend left, false for the reverse.
static inline bool b3DescendRayFirst( const b3MeshNode* node, b3Vec3 rayDelta )
{
	int axis = (int)node->data.asNode.axis;
	int32_t component = axis == 0 ? b3Raw( rayDelta.x ) : ( axis == 1 ? b3Raw( rayDelta.y ) : b3Raw( rayDelta.z ) );
	return component > 0;
}

bool b3OverlapMesh( const b3Mesh* shape, b3Transform shapeTransform, const b3ShapeProxy* proxy )
{
	B3_ASSERT( proxy->count > 0 );

	const b3MeshData* data = shape->data;
	B3_ASSERT( data != NULL && data->nodeCount > 0 );

	b3MeshScaling scaling = b3MakeMeshScaling( shape->scale );

	// The query shape, brought into the mesh's frame once.
	b3Vec3 buffer[B3_MAX_SHAPE_CAST_POINTS];
	b3ShapeProxy localProxy = b3MakeLocalProxy( proxy, shapeTransform, buffer );
	b3AABB bounds = b3ComputeProxyAABB( &localProxy );

	// And then into the blob's unscaled space for the descent. A negative scale
	// component swaps that axis's bounds, hence the min/max rather than a copy.
	b3Vec3 corner1 = b3ToMeshLocal( &scaling, bounds.lowerBound );
	b3Vec3 corner2 = b3ToMeshLocal( &scaling, bounds.upperBound );
	b3AABB local = b3MakeAABB( b3Min( corner1, corner2 ), b3Max( corner1, corner2 ) );

	b3Vec3 localCenter = b3AABB_Center( local );
	b3Vec3 localExtent = b3AABB_Extents( local );

	// A tenth of the linear slop, as upstream. The question is "are these
	// touching", and the slop is the scale at which this engine stops caring
	// about the difference.
	const b3f tolerance = b3Makeb3f( b3Raw( B3_LINEAR_SLOP ) / 10 );

	b3DistanceInput distanceInput;
	distanceInput.proxyB = localProxy;
	distanceInput.transform = b3Transform_identity;
	distanceInput.useRadii = true;

	const b3MeshTriangle* triangles = b3GetMeshTriangles( data );
	const b3Vec3* vertices = b3GetMeshVertices( data );

	const b3MeshNode* stack[B3_MESH_STACK_SIZE];
	int count = 0;

	const b3MeshNode* node = b3GetMeshRoot( data );

	while ( true )
	{
		if ( b3TestBoundsOverlap( node->lowerBound, node->upperBound, local.lowerBound, local.upperBound ) )
		{
			if ( b3IsLeaf( node ) )
			{
				int triangleCount = (int)node->data.asLeaf.triangleCount;
				int triangleOffset = (int)node->triangleOffset;

				for ( int index = 0; index < triangleCount; ++index )
				{
					b3MeshTriangle triangle = triangles[triangleOffset + index];

					b3Vec3 vertex1 = vertices[triangle.index1];
					b3Vec3 vertex2 = vertices[triangle.index2];
					b3Vec3 vertex3 = vertices[triangle.index3];

					// Cheap reject in unscaled space. Winding does not matter to
					// an overlap test, so the reflected case needs no handling.
					if ( b3TestBoundsTriangleOverlap( localCenter, localExtent, vertex1, vertex2, vertex3 ) == false )
					{
						continue;
					}

					b3Vec3 triangleVertices[3] = {
						b3FromMeshLocal( &scaling, vertex1 ),
						b3FromMeshLocal( &scaling, vertex2 ),
						b3FromMeshLocal( &scaling, vertex3 ),
					};

					distanceInput.proxyA = ( b3ShapeProxy ){ triangleVertices, 3, b3f_zero };

					b3SimplexCache cache = { 0 };
					b3DistanceOutput distanceOutput = b3ShapeDistance( &distanceInput, &cache, NULL, 0 );

					if ( b3Raw( distanceOutput.distance ) < b3Raw( tolerance ) )
					{
						return true;
					}
				}
			}
			else
			{
				B3_ASSERT( count < B3_MESH_STACK_SIZE );
				stack[count++] = b3GetRightChild( node );
				node = b3GetLeftChild( node );
				continue;
			}
		}

		if ( count == 0 )
		{
			break;
		}

		node = stack[--count];
	}

	return false;
}

/// Where along `rayDelta` from `rayStart` the ray crosses this triangle, as a
/// fraction. b3c_one means no crossing, which is also the answer for a ray that
/// reaches the triangle exactly at its far end.
///
/// @section exact Only the last step divides
///
/// This is the same shape as the GJK region tests in distance.c, and for the
/// same reason: **every decision before the final one is a sign test on an
/// exact integer**, so no tolerance has to be invented and no rounding can flip
/// an answer. Three edge-plane tests reject a ray passing outside the triangle,
/// a fourth rejects a back face, and only then is there a division -- one, of a
/// quantity that is a *ratio* and therefore lands naturally in Q30 through
/// b3DivWideToC.
///
/// Upstream halves the three edge midpoints. The port does not: it forms
/// `v_a + v_b - 2*rayStart` instead, which is twice the same vector. Doubling
/// scales the volume that is compared against zero, and a scaled comparison
/// against zero has the same answer -- so the halving is pure precision loss
/// with nothing bought, and dropping it keeps the whole test on integers.
///
/// @section range What bounds the inputs
///
/// The edge-plane volumes are the binding term: an edge (Q12) crossed with a
/// vertex-minus-origin (Q12) gives Q24, dotted with the ray delta (Q12) gives
/// Q36. Writing E for the longest triangle edge, D for how far the triangle sits
/// from the ray's origin and L for the ray's length, all raw, the worst term is
/// 6*E*D*L against int64's 9.2e18.
///
/// For the 20-unit triangles a DS level is built from, that allows D and L of
/// about **1000 units each** -- the same order as the +/-2000 unit world the rest
/// of the engine documents, and far past any ray a 256x192 screen justifies
/// firing. The assert below states it rather than trusting it.
static b3c b3IntersectRayTriangle( b3Vec3 rayStart, b3Vec3 rayDelta, b3Vec3 vertex1, b3Vec3 vertex2, b3Vec3 vertex3 )
{
	b3SatVec start = b3SatFromVec( rayStart );
	b3SatVec delta = b3SatFromVec( rayDelta );

	// Vertices relative to the ray origin. Everything below is a difference or
	// a sum of these, so the terms grow with the ray's reach rather than with
	// the world coordinates the triangle happens to sit at.
	b3SatVec v1 = b3SatSub( b3SatFromVec( vertex1 ), start );
	b3SatVec v2 = b3SatSub( b3SatFromVec( vertex2 ), start );
	b3SatVec v3 = b3SatSub( b3SatFromVec( vertex3 ), start );

#if defined( B3_ENABLE_ASSERT )
	{
		// 6*E*D*L < 2^63 with E bounded by the triangle-overlap test's own
		// 7.3e5 reach. 4.3e6 raw is 1050 units.
		const int64_t reach = 4300000;
		b3SatVec worst = b3SatMax( b3SatAbs( v1 ), b3SatMax( b3SatAbs( v2 ), b3SatAbs( v3 ) ) );
		B3_ASSERT( worst.x < reach && worst.y < reach && worst.z < reach );
		B3_ASSERT( b3SatAbs( delta ).x < reach && b3SatAbs( delta ).y < reach && b3SatAbs( delta ).z < reach );
	}
#endif

	// Does the ray pass inside all three edge planes? Each edge, crossed with
	// twice the midpoint of the two vertices bounding it, gives a plane through
	// the ray origin; the ray is inside when it has non-negative volume against
	// all three. Q24 cross, Q36 dot, sign only.
	{
		b3SatVec edge1 = b3SatSub( v3, v2 );
		b3SatVec edge2 = b3SatSub( v1, v3 );
		b3SatVec edge3 = b3SatSub( v2, v1 );

		if ( b3SatDot( b3SatCross( edge1, b3SatAdd( v2, v3 ) ), delta ) < 0 )
		{
			return b3c_one;
		}

		if ( b3SatDot( b3SatCross( edge2, b3SatAdd( v3, v1 ) ), delta ) < 0 )
		{
			return b3c_one;
		}

		if ( b3SatDot( b3SatCross( edge3, b3SatAdd( v1, v2 ) ), delta ) < 0 )
		{
			return b3c_one;
		}
	}

	// The triangle's plane. The normal is Q24 and stays wide -- narrowing it
	// would be the b3CrossWide hazard, an area underflowing for small triangles.
	b3SatVec normal = b3SatCross( b3SatSub( v2, v1 ), b3SatSub( v3, v1 ) );

	int64_t denominator = b3SatDot( normal, delta );
	if ( denominator >= 0 )
	{
		// Parallel, or approaching the back face. Upstream culls both here and
		// the port keeps that: a mesh is a surface with an outside, and a ray
		// that starts behind it has already passed through.
		return b3c_one;
	}

	// dot(normal, vertex1 - rayStart), and v1 is already that difference.
	int64_t numerator = b3SatDot( normal, v1 );

	// Both terms are Q36 and the quotient is dimensionless, which is exactly
	// what b3DivWideToC is for: it normalizes the pair before dividing, so
	// neither operand has to fit 32 bits.
	b3c lambda = b3DivWideToC( numerator, denominator );

	if ( b3Raw( lambda ) <= 0 )
	{
		// The crossing is behind the ray's origin.
		return b3c_one;
	}

	return b3Raw( lambda ) > b3Raw( b3c_one ) ? b3c_one : lambda;
}

b3CastOutput b3RayCastMesh( const b3Mesh* mesh, const b3RayCastInput* input )
{
	const b3MeshData* data = mesh->data;
	B3_ASSERT( data != NULL && data->nodeCount > 0 );

	b3CastOutput output = { 0 };
	output.fraction = input->maxFraction;
	output.triangleIndex = B3_NULL_INDEX;

	b3MeshScaling scaling = b3MakeMeshScaling( mesh->scale );

	// The descent runs on the inverse-scaled ray, in the blob's own space.
	b3Vec3 localStart = b3ToMeshLocal( &scaling, input->origin );
	b3Vec3 localDelta = b3ToMeshLocal( &scaling, input->translation );

	b3Vec3 localEnd = b3MulAdd( localStart, b3CToF( output.fraction ), localDelta );
	b3AABB localBounds = b3MakeAABB( b3Min( localStart, localEnd ), b3Max( localStart, localEnd ) );

	const b3MeshTriangle* triangles = b3GetMeshTriangles( data );
	const b3Vec3* vertices = b3GetMeshVertices( data );
	const uint8_t* materialIndices = b3GetMeshMaterialIndices( data );

	const b3MeshNode* stack[B3_MESH_STACK_SIZE];
	int count = 0;

	const b3MeshNode* node = b3GetMeshRoot( data );

	while ( true )
	{
		// Two tests, and they are only sufficient together: the first bounds the
		// ray's *length*, the second is a separating-axis test on the infinite
		// line. Same pairing b3DynamicTree_RayCast uses.
		if ( b3TestBoundsOverlap( node->lowerBound, node->upperBound, localBounds.lowerBound, localBounds.upperBound ) &&
			 b3TestBoundsRayOverlap( node->lowerBound, node->upperBound, localStart, localDelta ) )
		{
			if ( b3IsLeaf( node ) )
			{
				int triangleCount = (int)node->data.asLeaf.triangleCount;
				int triangleOffset = (int)node->triangleOffset;

				for ( int index = 0; index < triangleCount; ++index )
				{
					int triangleIndex = triangleOffset + index;
					b3MeshTriangle triangle = triangles[triangleIndex];

					// The triangle test runs in the shape's frame, not the
					// blob's: a non-uniform scale does not preserve the angles
					// the edge-plane tests are asking about.
					b3Vec3 vertex1 = b3FromMeshLocal( &scaling, vertices[triangle.index1] );
					b3Vec3 vertex2, vertex3;

					// A reflecting scale reverses winding, and this test is
					// one-sided, so the swap is what keeps the front face front.
					if ( scaling.reflected )
					{
						vertex2 = b3FromMeshLocal( &scaling, vertices[triangle.index3] );
						vertex3 = b3FromMeshLocal( &scaling, vertices[triangle.index2] );
					}
					else
					{
						vertex2 = b3FromMeshLocal( &scaling, vertices[triangle.index2] );
						vertex3 = b3FromMeshLocal( &scaling, vertices[triangle.index3] );
					}

					b3c fraction = b3IntersectRayTriangle( input->origin, input->translation, vertex1, vertex2, vertex3 );

					if ( b3Raw( fraction ) >= b3Raw( output.fraction ) )
					{
						continue;
					}

					output.normal = b3Normalize( b3CrossDirection( b3Sub( vertex2, vertex1 ), b3Sub( vertex3, vertex1 ) ) );
					output.point = b3MulAdd( input->origin, b3CToF( fraction ), input->translation );
					output.fraction = fraction;
					output.triangleIndex = triangleIndex;
					output.materialIndex = materialIndices != NULL ? (int)materialIndices[triangleIndex] : 0;
					output.hit = true;

					// Shorten the segment. This is the pruning that
					// b3DescendRayFirst exists to make effective.
					localEnd = b3MulAdd( localStart, b3CToF( fraction ), localDelta );
					localBounds = b3MakeAABB( b3Min( localStart, localEnd ), b3Max( localStart, localEnd ) );
				}
			}
			else
			{
				B3_ASSERT( count < B3_MESH_STACK_SIZE );

				if ( b3DescendRayFirst( node, localDelta ) )
				{
					stack[count++] = b3GetRightChild( node );
					node = b3GetLeftChild( node );
				}
				else
				{
					stack[count++] = b3GetLeftChild( node );
					node = b3GetRightChild( node );
				}

				continue;
			}
		}

		if ( count == 0 )
		{
			break;
		}

		node = stack[--count];
	}

	return output;
}

b3CastOutput b3ShapeCastMesh( const b3Mesh* mesh, const b3ShapeCastInput* input )
{
	const b3MeshData* data = mesh->data;
	B3_ASSERT( data != NULL && data->nodeCount > 0 );

	b3CastOutput output = { 0 };
	output.fraction = input->maxFraction;
	output.triangleIndex = B3_NULL_INDEX;

	b3MeshScaling scaling = b3MakeMeshScaling( mesh->scale );

	// The swept shape is treated as a ray from its own centre, with every node
	// and triangle fattened by its half-extent. That turns a shape cast's
	// broad-phase question back into the ray one above.
	b3AABB shapeBounds = b3ComputeProxyAABB( &input->proxy );
	b3Vec3 center = b3AABB_Center( shapeBounds );
	b3Vec3 extent = b3AABB_Extents( shapeBounds );

	b3Vec3 rayEnd = b3MulAdd( center, b3CToF( output.fraction ), input->translation );
	b3AABB rayBounds = b3MakeAABB( b3Min( center, rayEnd ), b3Max( center, rayEnd ) );

	b3Vec3 localStart = b3ToMeshLocal( &scaling, center );
	b3Vec3 localDelta = b3ToMeshLocal( &scaling, input->translation );

	// The extent has to come into the blob's space too, and a reflecting scale
	// must not turn a half-extent negative -- it is a radius, not a position.
	b3Vec3 localExtent = scaling.unitScale ? extent : b3Abs( b3Mul( scaling.invScale, extent ) );

	b3Vec3 localEnd = b3MulAdd( localStart, b3CToF( output.fraction ), localDelta );
	b3AABB localBounds = b3MakeAABB( b3Min( localStart, localEnd ), b3Max( localStart, localEnd ) );

	const b3MeshTriangle* triangles = b3GetMeshTriangles( data );
	const b3Vec3* vertices = b3GetMeshVertices( data );
	const uint8_t* materialIndices = b3GetMeshMaterialIndices( data );

	const b3MeshNode* stack[B3_MESH_STACK_SIZE];
	int count = 0;

	const b3MeshNode* node = b3GetMeshRoot( data );

	while ( true )
	{
		b3Vec3 nodeMin = b3Sub( node->lowerBound, localExtent );
		b3Vec3 nodeMax = b3Add( node->upperBound, localExtent );

		if ( b3TestBoundsOverlap( nodeMin, nodeMax, localBounds.lowerBound, localBounds.upperBound ) &&
			 b3TestBoundsRayOverlap( nodeMin, nodeMax, localStart, localDelta ) )
		{
			if ( b3IsLeaf( node ) )
			{
				int triangleCount = (int)node->data.asLeaf.triangleCount;
				int triangleOffset = (int)node->triangleOffset;

				for ( int index = 0; index < triangleCount; ++index )
				{
					int triangleIndex = triangleOffset + index;
					b3MeshTriangle triangle = triangles[triangleIndex];

					b3Vec3 vertex1 = b3FromMeshLocal( &scaling, vertices[triangle.index1] );
					b3Vec3 vertex2, vertex3;

					if ( scaling.reflected )
					{
						vertex2 = b3FromMeshLocal( &scaling, vertices[triangle.index3] );
						vertex3 = b3FromMeshLocal( &scaling, vertices[triangle.index2] );
					}
					else
					{
						vertex2 = b3FromMeshLocal( &scaling, vertices[triangle.index2] );
						vertex3 = b3FromMeshLocal( &scaling, vertices[triangle.index3] );
					}

					b3Vec3 triangleMin = b3Sub( b3Min( vertex1, b3Min( vertex2, vertex3 ) ), extent );
					b3Vec3 triangleMax = b3Add( b3Max( vertex1, b3Max( vertex2, vertex3 ) ), extent );

					if ( b3TestBoundsOverlap( triangleMin, triangleMax, rayBounds.lowerBound, rayBounds.upperBound ) == false )
					{
						continue;
					}

					// Shift the pair so the triangle's first vertex is the
					// origin. Upstream does this for float precision far from
					// the origin; in Q12 it is worth strictly more, because the
					// GJK below resolves differences of these points and a
					// difference of two large near-equal Q12 values is where the
					// bits go. It costs one negation.
					b3Vec3 origin = vertex1;
					b3Vec3 triangleVertices[3] = { b3Vec3_zero, b3Sub( vertex2, origin ), b3Sub( vertex3, origin ) };

					b3ShapeCastPairInput pairInput;
					pairInput.proxyA = ( b3ShapeProxy ){ triangleVertices, 3, b3f_zero };
					pairInput.proxyB = input->proxy;
					pairInput.transform = ( b3Transform ){ b3Neg( origin ), b3Quat_identity };
					pairInput.translationB = input->translation;
					pairInput.maxFraction = output.fraction;
					pairInput.canEncroach = input->canEncroach;

					b3CastOutput pairOutput = b3ShapeCast( &pairInput );

					if ( pairOutput.hit == false )
					{
						continue;
					}

					pairOutput.point = b3Add( pairOutput.point, origin );

					output = pairOutput;
					output.triangleIndex = triangleIndex;
					output.materialIndex = materialIndices != NULL ? (int)materialIndices[triangleIndex] : 0;

					// Shorten the sweep, in both spaces.
					rayEnd = b3MulAdd( center, b3CToF( output.fraction ), input->translation );
					rayBounds = b3MakeAABB( b3Min( center, rayEnd ), b3Max( center, rayEnd ) );

					localEnd = b3MulAdd( localStart, b3CToF( output.fraction ), localDelta );
					localBounds = b3MakeAABB( b3Min( localStart, localEnd ), b3Max( localStart, localEnd ) );
				}
			}
			else
			{
				B3_ASSERT( count < B3_MESH_STACK_SIZE );

				if ( b3DescendRayFirst( node, localDelta ) )
				{
					stack[count++] = b3GetRightChild( node );
					node = b3GetLeftChild( node );
				}
				else
				{
					stack[count++] = b3GetLeftChild( node );
					node = b3GetRightChild( node );
				}

				continue;
			}
		}

		if ( count == 0 )
		{
			break;
		}

		node = stack[--count];
	}

	return output;
}

// =========================================================================
// Character mover
// =========================================================================
//
// This is b3QueryMesh's third caller, after b3MeshTimeOfImpactFcn in shape.c
// and the mesh narrow phase -- and unlike the three query traversals above it
// does *not* open-code the descent.
//
// Upstream's b3CollideMoverAndMesh does open-code it, because it wants SIMD
// bounds in unscaled space. b3QueryMesh already does everything that descent
// does and nothing it does not: it takes bounds in the shape's frame and
// divides by the scale once, with the min/max that a negative component needs;
// it applies the scale to the vertices before the callback; it rejects a
// triangle whose bounds miss; and returning false from the callback stops it,
// which is exactly what a plane capacity wants and stops it *at* the cap rather
// than one leaf late.
//
// Two things differ from upstream's copy, both in this file's favour:
//
//   * b3QueryMesh swaps the second and third vertex for a reflected scale.
//     Upstream does not, and says so ("Winding order doesn't matter"). It is
//     inert here for the same reason: the callback builds a three-point
//     b3ShapeProxy and GJK's support function does not care what order the
//     points arrive in.
//   * b3QueryMesh promises ascending triangle index. Upstream's descent happens
//     to produce it but does not promise it. That matters more than it looks:
//     the plane set feeds a Gauss-Seidel solver, and a set in a different order
//     is a different fixed point, so the promise is what makes a mover standing
//     still on a mesh actually stand still.

/// Everything a triangle needs to become a mover plane.
typedef struct b3MoverMeshContext
{
	/// proxyB is the mover's core segment with **radius zero**, and the radius
	/// is applied by hand below. That is upstream's spelling and it is load
	/// bearing: useRadii would fold the radius into `distance` and the depth,
	/// which is radius minus distance, could no longer be recovered from it.
	b3DistanceInput distanceInput;

	/// The three triangle vertices the current callback is looking at, in the
	/// mesh shape's frame. distanceInput.proxyA points here.
	b3Vec3 triangle[3];

	b3PlaneResult* planes;
	int capacity;
	int count;
	b3f radius;
} b3MoverMeshContext;

/// Implements b3MeshQueryFcn.
static bool b3CollideMoverMeshFcn( b3Vec3 a, b3Vec3 b, b3Vec3 c, int triangleIndex, void* context )
{
	B3_UNUSED( triangleIndex );

	b3MoverMeshContext* moverContext = (b3MoverMeshContext*)context;

	moverContext->triangle[0] = a;
	moverContext->triangle[1] = b;
	moverContext->triangle[2] = c;
	moverContext->distanceInput.proxyA = ( b3ShapeProxy ){ moverContext->triangle, 3, b3f_zero };

	// A fresh cache per triangle. Consecutive triangles are unrelated, so a
	// warm start across them is worse than useless -- b3OverlapMesh's leaf does
	// the same, for the same reason.
	b3SimplexCache cache = { 0 };
	b3DistanceOutput output = b3ShapeDistance( &moverContext->distanceInput, &cache, NULL, 0 );

	if ( b3Raw( output.distance ) == 0 )
	{
		// Deep overlap. Upstream writes `// todo SAT` and drops it, and the
		// port drops it too rather than inventing an answer -- a plane built
		// from the zero normal GJK returns here is worse than no plane, because
		// b3SolvePlanes will accumulate push against it forever and move
		// nothing. b3CollideMoverAndHull declines the same case so that a hull
		// and the same shape baked as a mesh agree.
		return true;
	}

	if ( b3Raw( output.distance ) > b3Raw( moverContext->radius ) )
	{
		return true;
	}

	B3_ASSERT( b3IsNormalized( output.normal ) );

	// A depth, not a dot( normal, point ) -- see types.h above b3PlaneResult.
	b3Plane plane = { output.normal, b3SubF( moverContext->radius, output.distance ) };
	moverContext->planes[moverContext->count] = ( b3PlaneResult ){ plane, output.pointA };
	moverContext->count += 1;

	// False here is what makes the capacity a real bound: the traversal stops
	// and the remaining triangles are never tested.
	return moverContext->count < moverContext->capacity;
}

int b3CollideMoverAndMesh( b3PlaneResult* planes, int capacity, const b3Mesh* shape, const b3Capsule* mover )
{
	if ( capacity == 0 )
	{
		return 0;
	}

	B3_ASSERT( planes != NULL );

	b3MoverMeshContext context = { 0 };
	context.distanceInput.proxyB = ( b3ShapeProxy ){ &mover->center1, 2, b3f_zero };
	context.distanceInput.transform = b3Transform_identity;
	context.distanceInput.useRadii = false;
	context.planes = planes;
	context.capacity = capacity;
	context.radius = mover->radius;

	// Bounds in the *shape's* frame, not the blob's. b3QueryMesh divides by the
	// scale itself; pre-dividing here would apply it twice.
	b3AABB bounds = b3MakeAABB( b3Min( mover->center1, mover->center2 ), b3Max( mover->center1, mover->center2 ) );
	b3QueryMesh( shape, b3AABB_Inflate( bounds, mover->radius ), b3CollideMoverMeshFcn, &context );

	return context.count;
}

// =========================================================================
// Validation
// =========================================================================

/// Does every node's box contain what the node points at?
///
/// The property the traversal rests on. A node whose bounds are too small does
/// not corrupt anything -- it silently stops reporting triangles that are
/// really there, which is a bug that presents as a body falling through a
/// floor an hour later.
static bool b3IsMeshConsistent( const b3MeshData* mesh )
{
	const b3MeshTriangle* triangles = b3GetMeshTriangles( mesh );
	const b3Vec3* vertices = b3GetMeshVertices( mesh );
	const b3MeshNode* nodes = b3GetMeshNodes( mesh );

	const b3MeshNode* stack[B3_MESH_STACK_SIZE];
	int count = 0;
	stack[count++] = b3GetMeshRoot( mesh );

	// A blob that arrived as bytes may describe a cycle. Every node is meant
	// to be visited once, so more visits than there are nodes means the tree
	// is not one.
	int guard = 0;

	while ( count > 0 )
	{
		if ( ++guard > mesh->nodeCount )
		{
			return false;
		}

		const b3MeshNode* node = stack[--count];

		if ( node < nodes || node >= nodes + mesh->nodeCount )
		{
			return false;
		}

		b3AABB nodeBounds = b3MakeAABB( node->lowerBound, node->upperBound );

		if ( b3IsLeaf( node ) == false )
		{
			if ( count + 2 > B3_MESH_STACK_SIZE )
			{
				// Taller than the traversal stack, so b3QueryMesh could not
				// walk it safely even though it is otherwise well formed.
				return false;
			}

			const b3MeshNode* child1 = b3GetLeftChild( node );
			const b3MeshNode* child2 = b3GetRightChild( node );

			if ( child2 <= child1 || child2 >= nodes + mesh->nodeCount )
			{
				return false;
			}

			if ( b3AABB_Contains( nodeBounds, b3MakeAABB( child1->lowerBound, child1->upperBound ) ) == false )
			{
				return false;
			}

			if ( b3AABB_Contains( nodeBounds, b3MakeAABB( child2->lowerBound, child2->upperBound ) ) == false )
			{
				return false;
			}

			stack[count++] = child2;
			stack[count++] = child1;
			continue;
		}

		int triangleCount = (int)node->data.asLeaf.triangleCount;
		int triangleOffset = (int)node->triangleOffset;

		if ( triangleCount <= 0 || triangleOffset < 0 || triangleOffset + triangleCount > mesh->triangleCount )
		{
			return false;
		}

		for ( int index = 0; index < triangleCount; ++index )
		{
			b3MeshTriangle triangle = triangles[triangleOffset + index];

			if ( triangle.index1 < 0 || triangle.index1 >= mesh->vertexCount || triangle.index2 < 0 ||
				 triangle.index2 >= mesh->vertexCount || triangle.index3 < 0 || triangle.index3 >= mesh->vertexCount )
			{
				return false;
			}

			b3AABB triangleBounds = b3MakeAABB( vertices[triangle.index1], vertices[triangle.index1] );
			triangleBounds = b3AABB_AddPoint( triangleBounds, vertices[triangle.index2] );
			triangleBounds = b3AABB_AddPoint( triangleBounds, vertices[triangle.index3] );

			if ( b3AABB_Contains( nodeBounds, triangleBounds ) == false )
			{
				return false;
			}
		}
	}

	return true;
}

bool b3IsValidMesh( const b3MeshData* mesh )
{
	// Compiled in unconditionally rather than behind B3_ENABLE_VALIDATION,
	// which is the same decision b3IsValidHull records: this is not a per-frame
	// check, it is the one gate a blob that arrived as bytes passes through.
	if ( mesh == NULL )
	{
		return false;
	}

	if ( mesh->version != B3_MESH_VERSION )
	{
		return false;
	}

	if ( mesh->byteCount < (int)sizeof( b3MeshData ) )
	{
		return false;
	}

	if ( mesh->nodeCount <= 0 || mesh->vertexCount < 3 || mesh->triangleCount <= 0 )
	{
		return false;
	}

	if ( mesh->treeHeight <= 0 || mesh->treeHeight > B3_MESH_STACK_SIZE )
	{
		return false;
	}

	// Every array the traversal reads has to be present. The flags and the
	// material indices are optional -- a mesh with no edge classification
	// collides, it just ghosts on internal edges.
	if ( b3GetMeshNodes( mesh ) == NULL || b3GetMeshVertices( mesh ) == NULL || b3GetMeshTriangles( mesh ) == NULL )
	{
		return false;
	}

	// Offsets must land inside the blob, header first.
	int header = (int)sizeof( b3MeshData );
	if ( mesh->nodeOffset < header || mesh->vertexOffset < header || mesh->triangleOffset < header )
	{
		return false;
	}

	if ( mesh->nodeOffset + mesh->nodeCount * (int)sizeof( b3MeshNode ) > mesh->byteCount ||
		 mesh->vertexOffset + mesh->vertexCount * (int)sizeof( b3Vec3 ) > mesh->byteCount ||
		 mesh->triangleOffset + mesh->triangleCount * (int)sizeof( b3MeshTriangle ) > mesh->byteCount )
	{
		return false;
	}

	if ( mesh->flagsOffset != 0 && mesh->flagsOffset + mesh->triangleCount > mesh->byteCount )
	{
		return false;
	}

	if ( mesh->materialOffset != 0 && mesh->materialOffset + mesh->triangleCount > mesh->byteCount )
	{
		return false;
	}

	return b3IsMeshConsistent( mesh );
}
