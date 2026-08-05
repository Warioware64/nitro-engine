// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Warioware64, 2026

// Baking a triangle mesh into the port's fixed-point blob.
//
// Runs on the host, so it computes in double and emits Q12 -- which is what
// makes the whole off-device mesh design work: the tree, the welding and the
// edge classification are decided at full precision and only the *result* is
// quantized.
//
// Four things here are decisions rather than transcription. The first three are
// hull_bake.c's, restated because they apply unchanged:
//
//   1. Vertices round to nearest, not toward zero. Truncation would pull every
//      vertex toward the origin and shrink the level systematically.
//
//   2. Everything derived from a vertex is derived from the *quantized*
//      vertex. For a hull that meant fitting face planes to the stored points.
//      For a mesh it reaches further, because two derived things decide what is
//      *in* the blob rather than only what its numbers are:
//
//        - the degeneracy test. A triangle with real area in double can
//          collapse to nothing at Q12. Judged in double, that triangle ships,
//          and the runtime later takes a normal from a zero-length cross
//          product.
//
//        - the edge flags. A "flat" or "concave" bit derived from exact
//          geometry can contradict the quantized triangle it labels, and the
//          narrow phase trusts those bits to decide which contacts to discard.
//          That is the ghost-collision failure mode, arriving from the baker.
//
//   3. Spread the residual rather than pinning it to one vertex. Here that is
//      the welding: near-duplicate vertices are merged *after* quantization, so
//      two vertices that land on the same lattice point become one rather than
//      two coincident ones.
//
// And one that is new to meshes:
//
//   4. Node bounds round outward. b3AABB_Contains is an exact comparison with
//      no tolerance, so a node bound rounded inward both fails the port's own
//      consistency check and, worse, silently culls a triangle the node really
//      does contain. Since the bounds here are min/max over already-quantized
//      vertices they are exact, and this rule costs nothing -- but it is the
//      rule, and computing bounds from the doubles would break it.
//
// The tree is split at the **median** rather than by upstream's surface area
// heuristic. For a level of a few hundred triangles the traversal difference is
// not measurable on a 67 MHz ARM9, and it removes a float cost function that
// would otherwise have to be justified. The two sides therefore do NOT produce
// the same tree, which is why run_pair compares the set of triangles a query
// returns and never the traversal that found them.
//
// The blob is then handed to the port's own b3IsValidMesh. A mesh that does not
// survive quantization is rejected here, at bake time, rather than producing
// quietly wrong contacts later.

#include "mesh_bake.h"

#include "box3d/collision.h"
#include "box3d/constants.h"
#include "box3d/types.h"

#include "mesh.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

// =========================================================================
// Quantization
// =========================================================================

static b3f quantize( double v )
{
	return b3fFromDouble( v );
}

/// A vertex on the Q12 lattice, kept in both forms.
///
/// The double is not the input value -- it is the stored value read back. Every
/// derived quantity below reads `d`, never the caller's coordinate, which is
/// decision 2.
typedef struct
{
	b3Vec3 q;
	double d[3];
} pdBakedVertex;

static pdBakedVertex bakeVertex( pdVec3 v )
{
	pdBakedVertex out;
	out.q = b3MakeVec3( quantize( v.x ), quantize( v.y ), quantize( v.z ) );
	out.d[0] = b3fToDouble( out.q.x );
	out.d[1] = b3fToDouble( out.q.y );
	out.d[2] = b3fToDouble( out.q.z );
	return out;
}

// =========================================================================
// Build-time scratch
// =========================================================================

#define PD_BAKE_MAX_NODES ( 2 * PD_MAX_MESH_TRIANGLES )

typedef struct
{
	double lower[3];
	double upper[3];
	double center[3];
	int triangle; ///< Index into s_triangles.
} pdPrimitive;

typedef struct
{
	int index1, index2, index3;
} pdTriangle;

/// One entry per half-edge: 3 per triangle, matched up by vertex pair.
typedef struct
{
	int v1, v2;		   ///< Sorted vertex pair, so both halves hash together.
	int triangle;	   ///< Owning triangle.
	int edgeIndex;	   ///< Which of the triangle's three edges, 0..2.
} pdEdgeRef;

static pdBakedVertex s_vertices[PD_MAX_MESH_VERTICES];
static int s_vertexCount;

static pdTriangle s_triangles[PD_MAX_MESH_TRIANGLES];
static unsigned char s_flags[PD_MAX_MESH_TRIANGLES];
static int s_triangleCount;

static pdPrimitive s_primitives[PD_MAX_MESH_TRIANGLES];

static b3MeshNode s_nodes[PD_BAKE_MAX_NODES];
static int s_nodeCount;

static pdEdgeRef s_edges[3 * PD_MAX_MESH_TRIANGLES];
static double s_normals[PD_MAX_MESH_TRIANGLES][3];

// =========================================================================
// Small vector helpers, all in double over quantized coordinates
// =========================================================================

static void vsub( const double a[3], const double b[3], double out[3] )
{
	out[0] = a[0] - b[0];
	out[1] = a[1] - b[1];
	out[2] = a[2] - b[2];
}

static void vcross( const double a[3], const double b[3], double out[3] )
{
	out[0] = a[1] * b[2] - a[2] * b[1];
	out[1] = a[2] * b[0] - a[0] * b[2];
	out[2] = a[0] * b[1] - a[1] * b[0];
}

static double vdot( const double a[3], const double b[3] )
{
	return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

static double vlen( const double a[3] )
{
	return sqrt( vdot( a, a ) );
}

// =========================================================================
// Welding
// =========================================================================

/// Merge vertices that land on the same Q12 lattice point.
///
/// Not a tolerance weld -- upstream's spatial hash and its `weldTolerance` are
/// a modelling convenience, and a level authored for a DS does not need one.
/// What this does need to do is collapse vertices that quantization made
/// identical, because two coincident vertices give an edge no shared-edge
/// partner and the ghost filter then has nothing to work with.
///
/// O(n²) over at most PD_MAX_MESH_VERTICES, which is a host-side cost nobody
/// will notice.
static void weldQuantized( const pdMesh* desc, int* remap )
{
	s_vertexCount = 0;

	for ( int i = 0; i < desc->vertexCount; ++i )
	{
		pdBakedVertex v = bakeVertex( desc->vertices[i] );

		int found = -1;
		for ( int k = 0; k < s_vertexCount; ++k )
		{
			if ( b3Raw( s_vertices[k].q.x ) == b3Raw( v.q.x ) && b3Raw( s_vertices[k].q.y ) == b3Raw( v.q.y ) &&
				 b3Raw( s_vertices[k].q.z ) == b3Raw( v.q.z ) )
			{
				found = k;
				break;
			}
		}

		if ( found >= 0 )
		{
			remap[i] = found;
			continue;
		}

		s_vertices[s_vertexCount] = v;
		remap[i] = s_vertexCount;
		s_vertexCount++;
	}
}

// =========================================================================
// Degeneracy
// =========================================================================

/// The smallest cross-product length a triangle may have and still be kept.
///
/// Upstream compares `|cross|²` against `(0.01 * B3_LINEAR_SLOP²)²`, which
/// works out to 6.25e-14 -- a threshold from a world where the smallest
/// representable number is 1e-38. It cannot be converted, only re-derived.
///
/// The question a fixed-point port actually has to answer is "can this triangle
/// produce a usable normal?", and the answer is bounded by the lattice: the
/// smallest non-degenerate triangle on a Q12 grid has two edges one quantum
/// apart, so |cross| is one quantum squared. Anything below that has no normal
/// direction left after quantization. One quantum is 1/4096, so the bound is
/// 1/4096² -- and this asks for four times that, so a triangle has to be
/// clearly rather than marginally non-degenerate to survive.
#define PD_MIN_CROSS_LENGTH ( 4.0 / ( 4096.0 * 4096.0 ) )

static bool isDegenerate( const double* v1, const double* v2, const double* v3 )
{
	double e1[3], e2[3], n[3];
	vsub( v2, v1, e1 );
	vsub( v3, v1, e2 );
	vcross( e1, e2, n );
	return vlen( n ) < PD_MIN_CROSS_LENGTH;
}

// =========================================================================
// The BVH
// =========================================================================

#define PD_TRIANGLES_PER_LEAF 4

static void primitiveBounds( int first, int count, double lower[3], double upper[3] )
{
	for ( int axis = 0; axis < 3; ++axis )
	{
		lower[axis] = s_primitives[first].lower[axis];
		upper[axis] = s_primitives[first].upper[axis];
	}

	for ( int i = first + 1; i < first + count; ++i )
	{
		for ( int axis = 0; axis < 3; ++axis )
		{
			if ( s_primitives[i].lower[axis] < lower[axis] )
			{
				lower[axis] = s_primitives[i].lower[axis];
			}

			if ( s_primitives[i].upper[axis] > upper[axis] )
			{
				upper[axis] = s_primitives[i].upper[axis];
			}
		}
	}
}

static void storeBounds( b3MeshNode* node, const double lower[3], const double upper[3] )
{
	// Decision 4. The values are min/max over vertices that were already
	// quantized, so quantize() here is exact and rounds nothing -- but going
	// through the doubles at all is what would break it, so the invariant is
	// stated rather than assumed.
	node->lowerBound = b3MakeVec3( quantize( lower[0] ), quantize( lower[1] ), quantize( lower[2] ) );
	node->upperBound = b3MakeVec3( quantize( upper[0] ), quantize( upper[1] ), quantize( upper[2] ) );
}

/// Partition [first, first+count) about the median of the widest axis.
///
/// @return the size of the left half, or 0 if no split was possible.
static int splitMedian( int first, int count )
{
	double lower[3], upper[3];
	for ( int axis = 0; axis < 3; ++axis )
	{
		lower[axis] = s_primitives[first].center[axis];
		upper[axis] = s_primitives[first].center[axis];
	}

	for ( int i = first + 1; i < first + count; ++i )
	{
		for ( int axis = 0; axis < 3; ++axis )
		{
			if ( s_primitives[i].center[axis] < lower[axis] )
			{
				lower[axis] = s_primitives[i].center[axis];
			}

			if ( s_primitives[i].center[axis] > upper[axis] )
			{
				upper[axis] = s_primitives[i].center[axis];
			}
		}
	}

	int axis = 0;
	double widest = upper[0] - lower[0];
	if ( upper[1] - lower[1] > widest )
	{
		axis = 1;
		widest = upper[1] - lower[1];
	}
	if ( upper[2] - lower[2] > widest )
	{
		axis = 2;
	}

	double pivot = 0.5 * ( lower[axis] + upper[axis] );

	// Hoare partition, as upstream's median split does.
	int i1 = first;
	int i2 = first + count;
	while ( i1 < i2 )
	{
		while ( i1 < i2 && s_primitives[i1].center[axis] < pivot )
		{
			i1++;
		}

		while ( i1 < i2 && s_primitives[i2 - 1].center[axis] >= pivot )
		{
			i2--;
		}

		if ( i1 < i2 )
		{
			pdPrimitive tmp = s_primitives[i1];
			s_primitives[i1] = s_primitives[i2 - 1];
			s_primitives[i2 - 1] = tmp;
			i1++;
			i2--;
		}
	}

	int leftCount = i1 - first;

	// Every centroid on one side of the pivot -- coincident centroids, or a
	// degenerate spread. Halve it so the recursion still terminates.
	if ( leftCount == 0 || leftCount == count )
	{
		leftCount = count / 2;
	}

	return leftCount;
}

/// @return the node index, or -1 if the node budget or the height bound was
/// exceeded.
static int buildRecursive( int first, int count, int depth, int* treeHeight )
{
	if ( s_nodeCount >= PD_BAKE_MAX_NODES )
	{
		return -1;
	}

	// The device walks this with a fixed stack and no recursion, so a tree it
	// could not walk is rejected here rather than overflowing there. Upstream
	// carries a `todo` asking for exactly this.
	if ( depth >= B3_MESH_STACK_SIZE )
	{
		return -1;
	}

	if ( depth + 1 > *treeHeight )
	{
		*treeHeight = depth + 1;
	}

	int index = s_nodeCount++;

	double lower[3], upper[3];
	primitiveBounds( first, count, lower, upper );

	if ( count <= PD_TRIANGLES_PER_LEAF )
	{
		b3MeshNode* node = s_nodes + index;
		memset( node, 0, sizeof( *node ) );
		storeBounds( node, lower, upper );
		node->data.asLeaf.type = B3_LEAF_NODE;
		node->data.asLeaf.triangleCount = (uint32_t)count;
		node->triangleOffset = (uint32_t)first;
		return index;
	}

	int leftCount = splitMedian( first, count );

	int leftIndex = buildRecursive( first, leftCount, depth + 1, treeHeight );
	if ( leftIndex < 0 )
	{
		return -1;
	}

	int rightIndex = buildRecursive( first + leftCount, count - leftCount, depth + 1, treeHeight );
	if ( rightIndex < 0 )
	{
		return -1;
	}

	// Pre-order emission is what lets the left child be implicit.
	if ( leftIndex != index + 1 || rightIndex <= index + 1 )
	{
		return -1;
	}

	b3MeshNode* node = s_nodes + index;
	memset( node, 0, sizeof( *node ) );
	storeBounds( node, lower, upper );
	node->data.asNode.axis = 0;
	node->data.asNode.childOffset = (uint32_t)( rightIndex - index );

	// Zero for an internal node, so the blob's bytes are deterministic and its
	// hash is a function of the geometry alone.
	node->triangleOffset = 0;

	return index;
}

/// Reorder the triangle array into depth-first leaf order and rewrite each
/// leaf's offset to match.
///
/// This is the invariant b3QueryMesh's ascending-index guarantee rests on, and
/// nothing downstream asserts it -- the mesh narrow phase just quietly stops
/// matching its per-triangle cache. So it is done here and checked here.
static bool sortTrianglesDepthFirst( void )
{
	pdTriangle sorted[PD_MAX_MESH_TRIANGLES];
	unsigned char sortedFlags[PD_MAX_MESH_TRIANGLES];
	int written = 0;

	int stack[B3_MESH_STACK_SIZE];
	int count = 0;
	stack[count++] = 0;

	while ( count > 0 )
	{
		int index = stack[--count];
		b3MeshNode* node = s_nodes + index;

		if ( node->data.asLeaf.type != B3_LEAF_NODE )
		{
			if ( count + 2 > B3_MESH_STACK_SIZE )
			{
				return false;
			}

			// Right pushed first so left pops first: the same order the
			// device's descent takes.
			stack[count++] = index + (int)node->data.asNode.childOffset;
			stack[count++] = index + 1;
			continue;
		}

		int triangleCount = (int)node->data.asLeaf.triangleCount;
		int triangleOffset = (int)node->triangleOffset;

		for ( int i = 0; i < triangleCount; ++i )
		{
			sorted[written] = s_triangles[s_primitives[triangleOffset + i].triangle];
			sortedFlags[written] = 0;
			written++;
		}

		node->triangleOffset = (uint32_t)( written - triangleCount );
	}

	if ( written != s_triangleCount )
	{
		return false;
	}

	memcpy( s_triangles, sorted, (size_t)written * sizeof( pdTriangle ) );
	memcpy( s_flags, sortedFlags, (size_t)written );
	return true;
}

// =========================================================================
// Shared-edge classification
// =========================================================================

/// Set the concave / inverse-concave bits that let the narrow phase tell an
/// interior edge from a silhouette.
///
/// Run on the quantized vertices (decision 2), in double. This is the single
/// biggest reason baking off-device is worth doing: `signedVolume` is an
/// unnormalized scalar triple product, so it is cubic in the coordinates and
/// only its sign is ever used -- exactly the shape of expression that is
/// awkward in Q12 and free here.
static void identifyEdges( void )
{
	for ( int i = 0; i < s_triangleCount; ++i )
	{
		const double* v1 = s_vertices[s_triangles[i].index1].d;
		const double* v2 = s_vertices[s_triangles[i].index2].d;
		const double* v3 = s_vertices[s_triangles[i].index3].d;

		double e1[3], e2[3], n[3];
		vsub( v2, v1, e1 );
		vsub( v3, v1, e2 );
		vcross( e1, e2, n );

		double length = vlen( n );
		if ( length > 0.0 )
		{
			s_normals[i][0] = n[0] / length;
			s_normals[i][1] = n[1] / length;
			s_normals[i][2] = n[2] / length;
		}
		else
		{
			s_normals[i][0] = 0.0;
			s_normals[i][1] = 1.0;
			s_normals[i][2] = 0.0;
		}

		int idx[3] = { s_triangles[i].index1, s_triangles[i].index2, s_triangles[i].index3 };
		for ( int e = 0; e < 3; ++e )
		{
			int a = idx[e];
			int b = idx[( e + 1 ) % 3];

			pdEdgeRef* ref = s_edges + 3 * i + e;
			ref->v1 = a < b ? a : b;
			ref->v2 = a < b ? b : a;
			ref->triangle = i;
			ref->edgeIndex = e;
		}
	}

	int edgeCount = 3 * s_triangleCount;

	// O(n²) pairing rather than upstream's hash map. The map is the heaviest
	// dependency of upstream's builder -- a 2000-line header instantiated twice
	// -- and at PD_MAX_MESH_TRIANGLES this loop is a few million comparisons on
	// a desktop. b3mesh.py, which bakes what ships, indexes it properly.
	static const int concaveBits[3] = { b3_concaveEdge1, b3_concaveEdge2, b3_concaveEdge3 };
	static const int inverseBits[3] = { b3_inverseConcaveEdge1, b3_inverseConcaveEdge2, b3_inverseConcaveEdge3 };

	for ( int i = 0; i < edgeCount; ++i )
	{
		const pdEdgeRef* a = s_edges + i;

		for ( int k = i + 1; k < edgeCount; ++k )
		{
			const pdEdgeRef* b = s_edges + k;
			if ( a->v1 != b->v1 || a->v2 != b->v2 )
			{
				continue;
			}

			// The opposite vertex of the neighbouring triangle: edge 0 is
			// v1->v2 so v3 is opposite, edge 1 is v2->v3 so v1 is, edge 2 is
			// v3->v1 so v2 is.
			const pdTriangle* other = s_triangles + b->triangle;
			int otherIdx[3] = { other->index1, other->index2, other->index3 };
			int opposite = otherIdx[( b->edgeIndex + 2 ) % 3];

			const pdTriangle* self = s_triangles + a->triangle;
			const double* v1 = s_vertices[self->index1].d;
			const double* v2 = s_vertices[self->index2].d;
			const double* v3 = s_vertices[self->index3].d;
			const double* p = s_vertices[opposite].d;

			// signedVolume = dot( cross( v2-v1, v3-v1 ), p-v1 ). Negative when
			// the neighbour lies below this triangle's plane.
			double e1[3], e2[3], n[3], d[3];
			vsub( v2, v1, e1 );
			vsub( v3, v1, e2 );
			vcross( e1, e2, n );
			vsub( p, v1, d );
			double signedVolume = vdot( n, d );

			const double cos5Deg = 0.9962;
			double cosAngle = vdot( s_normals[a->triangle], s_normals[b->triangle] );

			if ( signedVolume > 0.0 || cosAngle > cos5Deg )
			{
				s_flags[a->triangle] |= (unsigned char)concaveBits[a->edgeIndex];
				s_flags[b->triangle] |= (unsigned char)concaveBits[b->edgeIndex];
			}

			if ( signedVolume < 0.0 || cosAngle > cos5Deg )
			{
				s_flags[a->triangle] |= (unsigned char)inverseBits[a->edgeIndex];
				s_flags[b->triangle] |= (unsigned char)inverseBits[b->edgeIndex];
			}

			// Only the first partner counts, as upstream. A non-manifold edge
			// with three or more incident triangles gets no useful answer, and
			// pretending otherwise would be worse than leaving it unflagged.
			break;
		}
	}
}

// =========================================================================
// Layout
// =========================================================================

typedef struct
{
	int nodeOffset;
	int vertexOffset;
	int triangleOffset;
	int materialOffset;
	int flagsOffset;
	int byteCount;
} pdMeshLayout;

static int alignTo( int cursor, size_t alignment )
{
	int a = (int)alignment;
	return ( ( cursor + a - 1 ) / a ) * a;
}

static pdMeshLayout meshLayout( int nodeCount, int vertexCount, int triangleCount )
{
	pdMeshLayout layout;
	int cursor = (int)sizeof( b3MeshData );

	// Alignment comes from the types rather than from a hard-coded number:
	// b3Vec3 is 12 bytes in device mode and much wider under the shadow-value
	// build, and the same source has to lay out both.
	cursor = alignTo( cursor, sizeof( b3MeshNode ) );
	layout.nodeOffset = cursor;
	cursor += nodeCount * (int)sizeof( b3MeshNode );

	cursor = alignTo( cursor, sizeof( b3Vec3 ) );
	layout.vertexOffset = cursor;
	cursor += vertexCount * (int)sizeof( b3Vec3 );

	cursor = alignTo( cursor, sizeof( b3MeshTriangle ) );
	layout.triangleOffset = cursor;
	cursor += triangleCount * (int)sizeof( b3MeshTriangle );

	layout.materialOffset = cursor;
	cursor += triangleCount;

	layout.flagsOffset = cursor;
	cursor += triangleCount;

	// Round the total up so an array of meshes stays aligned.
	layout.byteCount = ( cursor + 7 ) & ~7;
	return layout;
}

int pdBakeMeshSize( const pdMesh* desc )
{
	// An upper bound: nothing has been welded or dropped yet, and the tree has
	// not been built, so assume the worst on all three counts.
	return meshLayout( PD_BAKE_MAX_NODES, desc->vertexCount, desc->triangleCount ).byteCount;
}

// =========================================================================
// The bake
// =========================================================================

int pdBakeMesh( const pdMesh* desc, void* blob, int size )
{
	if ( desc->vertexCount < 3 || desc->triangleCount < 1 )
	{
		return 0;
	}

	if ( desc->vertexCount > PD_MAX_MESH_VERTICES || desc->triangleCount > PD_MAX_MESH_TRIANGLES )
	{
		return 0;
	}

	// Quantize and weld first, so that everything after this reads stored
	// coordinates only.
	int remap[PD_MAX_MESH_VERTICES];
	weldQuantized( desc, remap );

	if ( s_vertexCount < 3 )
	{
		return 0;
	}

	// Q12 range. b3Makeb3f tops out around 524288, and the practical world is
	// far smaller; B3_HUGE is the port's own statement of where that is.
	for ( int i = 0; i < s_vertexCount; ++i )
	{
		for ( int axis = 0; axis < 3; ++axis )
		{
			if ( fabs( s_vertices[i].d[axis] ) > b3fToDouble( B3_HUGE ) )
			{
				return 0;
			}
		}
	}

	// Keep the triangles that still have area on the lattice.
	s_triangleCount = 0;
	int degenerateCount = 0;
	double surfaceArea = 0.0;

	for ( int i = 0; i < desc->triangleCount; ++i )
	{
		int i1 = remap[desc->indices[3 * i + 0]];
		int i2 = remap[desc->indices[3 * i + 1]];
		int i3 = remap[desc->indices[3 * i + 2]];

		if ( i1 == i2 || i1 == i3 || i2 == i3 )
		{
			degenerateCount++;
			continue;
		}

		const double* v1 = s_vertices[i1].d;
		const double* v2 = s_vertices[i2].d;
		const double* v3 = s_vertices[i3].d;

		if ( isDegenerate( v1, v2, v3 ) )
		{
			degenerateCount++;
			continue;
		}

		double e1[3], e2[3], n[3];
		vsub( v2, v1, e1 );
		vsub( v3, v1, e2 );
		vcross( e1, e2, n );
		surfaceArea += 0.5 * vlen( n );

		s_triangles[s_triangleCount].index1 = i1;
		s_triangles[s_triangleCount].index2 = i2;
		s_triangles[s_triangleCount].index3 = i3;
		s_flags[s_triangleCount] = 0;
		s_triangleCount++;
	}

	if ( s_triangleCount < 1 )
	{
		return 0;
	}

	// Primitives, whose bounds are min/max over the stored vertices and are
	// therefore exact on the lattice.
	for ( int i = 0; i < s_triangleCount; ++i )
	{
		const double* v1 = s_vertices[s_triangles[i].index1].d;
		const double* v2 = s_vertices[s_triangles[i].index2].d;
		const double* v3 = s_vertices[s_triangles[i].index3].d;

		for ( int axis = 0; axis < 3; ++axis )
		{
			double lo = v1[axis] < v2[axis] ? v1[axis] : v2[axis];
			lo = lo < v3[axis] ? lo : v3[axis];

			double hi = v1[axis] > v2[axis] ? v1[axis] : v2[axis];
			hi = hi > v3[axis] ? hi : v3[axis];

			s_primitives[i].lower[axis] = lo;
			s_primitives[i].upper[axis] = hi;
			s_primitives[i].center[axis] = 0.5 * ( lo + hi );
		}

		s_primitives[i].triangle = i;
	}

	s_nodeCount = 0;
	int treeHeight = 0;
	if ( buildRecursive( 0, s_triangleCount, 0, &treeHeight ) != 0 )
	{
		return 0;
	}

	if ( sortTrianglesDepthFirst() == false )
	{
		return 0;
	}

	identifyEdges();

	pdMeshLayout layout = meshLayout( s_nodeCount, s_vertexCount, s_triangleCount );
	if ( size < layout.byteCount )
	{
		return 0;
	}

	// Zero everything, padding included: the hash is over raw bytes.
	memset( blob, 0, (size_t)layout.byteCount );

	b3MeshData* mesh = (b3MeshData*)blob;
	mesh->version = B3_MESH_VERSION;
	mesh->byteCount = layout.byteCount;
	mesh->hash = 0;
	mesh->surfaceArea = quantize( surfaceArea );
	mesh->treeHeight = treeHeight;
	mesh->degenerateCount = degenerateCount;
	mesh->nodeOffset = layout.nodeOffset;
	mesh->nodeCount = s_nodeCount;
	mesh->vertexOffset = layout.vertexOffset;
	mesh->vertexCount = s_vertexCount;
	mesh->triangleOffset = layout.triangleOffset;
	mesh->triangleCount = s_triangleCount;
	mesh->materialOffset = layout.materialOffset;
	mesh->materialCount = 1;
	mesh->flagsOffset = layout.flagsOffset;

	b3MeshNode* nodes = (b3MeshNode*)( (char*)blob + layout.nodeOffset );
	b3Vec3* vertices = (b3Vec3*)( (char*)blob + layout.vertexOffset );
	b3MeshTriangle* triangles = (b3MeshTriangle*)( (char*)blob + layout.triangleOffset );
	uint8_t* flags = (uint8_t*)( (char*)blob + layout.flagsOffset );

	memcpy( nodes, s_nodes, (size_t)s_nodeCount * sizeof( b3MeshNode ) );

	for ( int i = 0; i < s_vertexCount; ++i )
	{
		vertices[i] = s_vertices[i].q;
	}

	for ( int i = 0; i < s_triangleCount; ++i )
	{
		triangles[i].index1 = s_triangles[i].index1;
		triangles[i].index2 = s_triangles[i].index2;
		triangles[i].index3 = s_triangles[i].index3;
		flags[i] = s_flags[i];
	}

	// Bounds from the stored vertices, so they contain what is in the blob
	// rather than what was asked for.
	b3AABB bounds = b3MakeAABB( vertices[0], vertices[0] );
	for ( int i = 1; i < s_vertexCount; ++i )
	{
		bounds = b3AABB_AddPoint( bounds, vertices[i] );
	}
	mesh->bounds = bounds;

	// djb2 over the whole blob with the hash field zero, which it already is.
	// Nothing on device verifies this -- see the note at b3MeshData::hash --
	// but a .b3mesh is a file, and a file can go stale.
	{
		uint32_t h = 5381u;
		const uint8_t* bytes = (const uint8_t*)blob;
		for ( int i = 0; i < layout.byteCount; ++i )
		{
			h = h * 33u + bytes[i];
		}
		mesh->hash = h != 0 ? h : 1u;
	}

	// The port's own validator has the last word. A mesh whose tree no longer
	// contains its triangles after quantization is rejected here rather than
	// losing contacts at run time.
	if ( b3IsValidMesh( mesh ) == false )
	{
		return 0;
	}

	return layout.byteCount;
}
