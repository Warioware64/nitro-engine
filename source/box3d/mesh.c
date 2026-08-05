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
///   b3RayCastMesh, b3ShapeCastMesh, b3OverlapMesh,
///   b3CollideMoverAndMesh                            -- Phase 7, with the rest
///                                                       of the query layer
///
/// @section simd No b3V32
///
/// Upstream funnels the traversal through a `b3V32` vector type that has an
/// SSE2 form and a scalar fallback -- and the scalar fallback is literally
/// `struct { float x, y, z; }`, which is `b3Vec3`. There is nothing for a
/// second type to buy here, so the two overlap tests below are written against
/// b3Vec3 directly and the port has no simd.h at all.

#include "mesh.h"

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
