// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Warioware64, 2026

// The fixed-point side of run_pair.
//
// Mirrors pair_ref.c function for function, but sees only the port's headers.
// Nothing here is renamed -- the port keeps its b3* symbols, and the reference
// is the side that moves out of the way.

#include "pair_iface.h"

#include "hull_bake.h"
#include "mesh_bake.h"

#include "body.h"
#include "broad_phase.h"
#include "core.h"
#include "mesh.h"
#include "physics_world.h"
#include "shape.h"

#include "box3d/collision.h"
#include "box3d/math_fixed.h"
#include "box3d/types.h"

#include <stdlib.h>
#include <string.h>

static b3Vec3 toV( pdVec3 v )
{
	return b3MakeVec3( b3fFromDouble( v.x ), b3fFromDouble( v.y ), b3fFromDouble( v.z ) );
}

static pdVec3 fromV( b3Vec3 v )
{
	pdVec3 r = { b3fToDouble( v.x ), b3fToDouble( v.y ), b3fToDouble( v.z ) };
	return r;
}

static b3Transform toXf( const pdTransform* xf )
{
	b3Transform t;
	t.p = toV( xf->p );
	t.q.v = b3MakeDir3( b3nFromDouble( xf->qx ), b3nFromDouble( xf->qy ), b3nFromDouble( xf->qz ) );
	t.q.s = b3nFromDouble( xf->qw );
	return t;
}

static pdTransform fromXf( b3WorldTransform t )
{
	pdTransform xf;
	xf.p = fromV( t.p );
	xf.qx = b3nToDouble( t.q.v.x );
	xf.qy = b3nToDouble( t.q.v.y );
	xf.qz = b3nToDouble( t.q.v.z );
	xf.qw = b3nToDouble( t.q.s );
	return xf;
}

static b3ShapeProxy toProxy( const pdProxy* p, b3Vec3* storage )
{
	for ( int i = 0; i < p->count; ++i )
	{
		storage[i] = toV( p->points[i] );
	}

	b3ShapeProxy proxy;
	proxy.points = storage;
	proxy.count = p->count;
	proxy.radius = b3fFromDouble( p->radius );
	return proxy;
}

static void portDistance( const pdProxy* a, const pdProxy* b, const pdTransform* xf, bool useRadii, pdDistanceOut* out )
{
	b3Vec3 sa[PD_MAX_POINTS], sb[PD_MAX_POINTS];

	b3DistanceInput input;
	input.proxyA = toProxy( a, sa );
	input.proxyB = toProxy( b, sb );
	input.transform = toXf( xf );
	input.useRadii = useRadii;

	b3SimplexCache cache = { 0 };
	b3DistanceOutput o = b3ShapeDistance( &input, &cache, NULL, 0 );

	memset( out, 0, sizeof( *out ) );
	out->distance = b3fToDouble( o.distance );
	out->pointA = fromV( o.pointA );
	out->pointB = fromV( o.pointB );
	out->normal = fromV( o.normal );
}

static void portShapeCast( const pdProxy* a, const pdProxy* b, const pdTransform* xf, pdVec3 translation, pdCastOut* out )
{
	b3Vec3 sa[PD_MAX_POINTS], sb[PD_MAX_POINTS];

	b3ShapeCastPairInput input;
	input.proxyA = toProxy( a, sa );
	input.proxyB = toProxy( b, sb );
	input.transform = toXf( xf );
	input.translationB = toV( translation );
	input.maxFraction = b3c_one;
	input.canEncroach = false;

	b3CastOutput o = b3ShapeCast( &input );

	memset( out, 0, sizeof( *out ) );
	out->hit = o.hit;
	out->fraction = b3cToDouble( o.fraction );
	out->point = fromV( o.point );
	out->normal = fromV( o.normal );
}

// Both libraries pack a feature pair into a warm-starting id the same way;
// spelling it out here avoids pulling either one's internal manifold.h in.
static unsigned packFeatureId( b3FeaturePair pair )
{
	return ( (unsigned)pair.owner1 << 24 ) | ( (unsigned)pair.index1 << 16 ) | ( (unsigned)pair.owner2 << 8 ) |
		   (unsigned)pair.index2;
}

static void grabManifold( const b3LocalManifold* m, pdManifoldOut* out )
{
	memset( out, 0, sizeof( *out ) );
	out->pointCount = m->pointCount;
	out->normal = fromV( m->normal );

	for ( int i = 0; i < m->pointCount && i < PD_MAX_MANIFOLD_POINTS; ++i )
	{
		out->points[i] = fromV( m->points[i].point );
		out->featureIds[i] = packFeatureId( m->points[i].pair );
		out->separations[i] = b3fToDouble( m->points[i].separation );
	}
}

static void portSphereSphere( pdVec3 cA, double rA, pdVec3 cB, double rB, const pdTransform* xf, pdManifoldOut* out )
{
	b3Sphere sa = { toV( cA ), b3fFromDouble( rA ) };
	b3Sphere sb = { toV( cB ), b3fFromDouble( rB ) };

	b3LocalManifoldPoint buffer[PD_MAX_MANIFOLD_POINTS] = { 0 };
	b3LocalManifold m = { 0 };
	m.points = buffer;

	b3CollideSpheres( &m, PD_MAX_MANIFOLD_POINTS, &sa, &sb, toXf( xf ) );
	grabManifold( &m, out );
}

static void portCapsuleSphere( pdVec3 a1, pdVec3 a2, double rA, pdVec3 cB, double rB, const pdTransform* xf,
							   pdManifoldOut* out )
{
	b3Capsule ca = { toV( a1 ), toV( a2 ), b3fFromDouble( rA ) };
	b3Sphere sb = { toV( cB ), b3fFromDouble( rB ) };

	b3LocalManifoldPoint buffer[PD_MAX_MANIFOLD_POINTS] = { 0 };
	b3LocalManifold m = { 0 };
	m.points = buffer;

	b3CollideCapsuleAndSphere( &m, PD_MAX_MANIFOLD_POINTS, &ca, &sb, toXf( xf ) );
	grabManifold( &m, out );
}

static void portCapsuleCapsule( pdVec3 a1, pdVec3 a2, double rA, pdVec3 b1, pdVec3 b2, double rB, const pdTransform* xf,
								pdManifoldOut* out )
{
	b3Capsule ca = { toV( a1 ), toV( a2 ), b3fFromDouble( rA ) };
	b3Capsule cb = { toV( b1 ), toV( b2 ), b3fFromDouble( rB ) };

	b3LocalManifoldPoint buffer[PD_MAX_MANIFOLD_POINTS] = { 0 };
	b3LocalManifold m = { 0 };
	m.points = buffer;

	b3CollideCapsules( &m, PD_MAX_MANIFOLD_POINTS, &ca, &cb, toXf( xf ) );
	grabManifold( &m, out );
}

// --- hulls ----------------------------------------------------------------
//
// The port cannot build a hull, so every one arrives as a description that the
// reference produced and hull_bake.c quantizes. Baking on each call rather
// than caching keeps this side stateless, and puts the baker on the path of
// every hull case rather than only the first.

static bool bakeInto( const pdHull* hull, void* blob, int size )
{
	return pdBakeHull( hull, blob, size ) > 0;
}

/// Scratch large enough for any hull within the port's configured limits.
typedef union
{
	b3HullData header;
	char bytes[4096];
	double align;
} pdHullBlob;

// --- triangles ------------------------------------------------------------
//
// Nothing is baked here: the triangle crosses the boundary as three points and
// is used directly, so a divergence is the collide function's and not the
// baker's.

static void portTriangleSphere( const pdVec3 tri[3], pdVec3 center, double radius, pdManifoldOut* out )
{
	memset( out, 0, sizeof( *out ) );

	b3Vec3 points[3] = { toV( tri[0] ), toV( tri[1] ), toV( tri[2] ) };
	b3Sphere sphere = { toV( center ), b3fFromDouble( radius ) };

	b3LocalManifoldPoint buffer[PD_MAX_MANIFOLD_POINTS] = { 0 };
	b3LocalManifold m = { 0 };
	m.points = buffer;

	b3CollideTriangleAndSphere( &m, PD_MAX_MANIFOLD_POINTS, points, &sphere );
	grabManifold( &m, out );
	out->triangleFeature = (int)m.feature;
}

static void portTriangleCapsule( const pdVec3 tri[3], pdVec3 c1, pdVec3 c2, double radius, bool reuse,
								 pdManifoldOut* out )
{
	memset( out, 0, sizeof( *out ) );

	b3Vec3 points[3] = { toV( tri[0] ), toV( tri[1] ), toV( tri[2] ) };
	b3Capsule capsule = { toV( c1 ), toV( c2 ), b3fFromDouble( radius ) };

	b3LocalManifoldPoint buffer[PD_MAX_MANIFOLD_POINTS] = { 0 };
	b3LocalManifold m = { 0 };
	m.points = buffer;

	b3SimplexCache cache = { 0 };
	if ( reuse )
	{
		b3LocalManifoldPoint warm[PD_MAX_MANIFOLD_POINTS] = { 0 };
		b3LocalManifold first = { 0 };
		first.points = warm;
		b3CollideTriangleAndCapsule( &first, PD_MAX_MANIFOLD_POINTS, points, &capsule, &cache );
	}

	b3CollideTriangleAndCapsule( &m, PD_MAX_MANIFOLD_POINTS, points, &capsule, &cache );
	grabManifold( &m, out );
	out->triangleFeature = (int)m.feature;
}

#define PD_TRIANGLE_HULL_CAPACITY 4

// Defined with the hull-versus-hull block below; the triangle path reaches
// the same b3SATCache and reports it the same way.
static void grabFeature( const b3SATCache* cache, pdManifoldOut* out );

static void portTriangleHull( const pdVec3 tri[3], const pdHull* hull, const pdTransform* xf, bool reuse,
							  pdManifoldOut* out, bool* cacheHit )
{
	pdHullBlob blob;
	memset( out, 0, sizeof( *out ) );
	*cacheHit = false;

	if ( bakeInto( hull, &blob, (int)sizeof( blob ) ) == false )
	{
		return;
	}

	// The hull arrives in its own frame; the triangle path has no transform
	// parameter, so the hull's points are what move. Baking then transforming
	// would need a second blob, so the transform is folded into the triangle
	// instead -- inverse-transforming the triangle into the hull's frame is
	// the same collision.
	b3Transform t = toXf( xf );
	b3Vec3 points[3] = { b3InvTransformPoint( t, toV( tri[0] ) ), b3InvTransformPoint( t, toV( tri[1] ) ),
						 b3InvTransformPoint( t, toV( tri[2] ) ) };

	b3LocalManifoldPoint buffer[PD_MAX_MANIFOLD_POINTS] = { 0 };
	b3LocalManifold m = { 0 };
	m.points = buffer;

	b3SATCache cache = { 0 };
	if ( reuse )
	{
		b3LocalManifoldPoint warm[PD_MAX_MANIFOLD_POINTS] = { 0 };
		b3LocalManifold first = { 0 };
		first.points = warm;
		b3CollideTriangleAndHull( &first, PD_TRIANGLE_HULL_CAPACITY, points[0], points[1], points[2], 0, &blob.header,
								  &cache, true );
	}

	b3CollideTriangleAndHull( &m, PD_TRIANGLE_HULL_CAPACITY, points[0], points[1], points[2], 0, &blob.header, &cache,
							  true );
	grabManifold( &m, out );
	grabFeature( &cache, out );
	out->triangleFeature = (int)m.feature;
	*cacheHit = cache.hit != 0;
}

// --- triangle meshes ------------------------------------------------------
//
// Same shape as hulls, with one difference worth stating: a mesh blob is two
// orders of magnitude larger than a hull's, so it is baked once per pdMesh and
// cached rather than re-baked per call. The baker is still on the path of every
// mesh scenario, just not of every query inside one.

/// Scratch for one baked mesh at PD_MAX_MESH_* -- 512 triangles is ~1000 nodes
/// at 32 bytes, plus the vertices and triangles.
typedef union
{
	b3MeshData header;
	char bytes[65536];
	double align;
} pdMeshBlob;

static pdMeshBlob s_meshBlobs[8];
static bool s_meshBaked[8];

/// @return NULL if the description does not survive baking, which every caller
/// must treat as "skip this case" rather than as a divergence.
static const b3MeshData* portMesh( const pdMesh* mesh )
{
	int id = mesh->refId;
	if ( id < 0 || id >= (int)( sizeof( s_meshBlobs ) / sizeof( s_meshBlobs[0] ) ) )
	{
		return NULL;
	}

	if ( s_meshBaked[id] == false )
	{
		if ( pdBakeMesh( mesh, &s_meshBlobs[id], (int)sizeof( s_meshBlobs[id] ) ) == 0 )
		{
			return NULL;
		}

		s_meshBaked[id] = true;
	}

	return &s_meshBlobs[id].header;
}

static void portMeshAABB( const pdMesh* mesh, const pdTransform* xf, pdVec3 scale, pdAABB* out )
{
	memset( out, 0, sizeof( *out ) );

	const b3MeshData* data = portMesh( mesh );
	if ( data == NULL )
	{
		return;
	}

	b3AABB aabb = b3ComputeMeshAABB( data, toXf( xf ), b3SafeScale( toV( scale ) ) );
	out->lower = fromV( aabb.lowerBound );
	out->upper = fromV( aabb.upperBound );
}

typedef struct
{
	int* indices;
	int capacity;
	int count;
} portQueryContext;

static bool portCollectTriangle( b3Vec3 a, b3Vec3 b, b3Vec3 c, int triangleIndex, void* context )
{
	B3_UNUSED( a, b, c );

	portQueryContext* ctx = context;
	if ( ctx->count < ctx->capacity )
	{
		ctx->indices[ctx->count] = triangleIndex;
	}
	ctx->count++;
	return true;
}

static int portMeshQuery( const pdMesh* mesh, pdVec3 lower, pdVec3 upper, pdVec3 scale, int* indices, int capacity )
{
	const b3MeshData* data = portMesh( mesh );
	if ( data == NULL )
	{
		return -1;
	}

	b3Mesh m = { data, b3SafeScale( toV( scale ) ) };
	b3AABB bounds = b3MakeAABB( toV( lower ), toV( upper ) );

	portQueryContext ctx = { indices, capacity, 0 };
	b3QueryMesh( &m, bounds, portCollectTriangle, &ctx );
	return ctx.count;
}

static bool portMeshTriangle( const pdMesh* mesh, int triangleIndex, pdVec3 scale, pdVec3 out[3], int* flags )
{
	const b3MeshData* data = portMesh( mesh );
	if ( data == NULL )
	{
		return false;
	}

	if ( triangleIndex < 0 || triangleIndex >= data->triangleCount )
	{
		return false;
	}

	b3Mesh m = { data, b3SafeScale( toV( scale ) ) };
	b3Triangle tri = b3GetMeshTriangle( &m, triangleIndex );

	for ( int i = 0; i < 3; ++i )
	{
		out[i] = fromV( tri.vertices[i] );
	}

	*flags = tri.flags;
	return true;
}

// =========================================================================
// Worlds and bodies
// =========================================================================

/// Build a world, create one dynamic body from the description, and read back
/// what the port computed. The world is torn down before returning, so a leak
/// in the world tier shows up as a growing heap over the scenario.
///
/// Hull shapes need their blob to outlive the body, because the port aliases
/// the caller's hull data rather than copying it -- which is the design, and
/// this is the first place that contract is exercised end to end.
static void portWorldBody( const pdBodyDesc* desc, const pdHull* hulls, int hullCount, pdBodyOut* out )
{
	memset( out, 0, sizeof( *out ) );

	pdHullBlob blobs[PD_MAX_BODY_SHAPES];

	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = toV( desc->xf.p );
	bodyDef.rotation = toXf( &desc->xf ).q;

	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

	b3ShapeId shapeIds[PD_MAX_BODY_SHAPES];
	int shapeCount = 0;

	for ( int i = 0; i < desc->shapeCount && i < PD_MAX_BODY_SHAPES; ++i )
	{
		const pdBodyShape* s = desc->shapes + i;

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.density = b3fFromDouble( s->density );

		switch ( s->kind )
		{
			case pd_bodyShapeSphere:
			{
				b3Sphere sphere = { toV( s->p1 ), b3fFromDouble( s->radius ) };
				shapeIds[shapeCount++] = b3CreateSphereShape( bodyId, &shapeDef, &sphere );
			}
			break;

			case pd_bodyShapeCapsule:
			{
				b3Capsule capsule = { toV( s->p1 ), toV( s->p2 ), b3fFromDouble( s->radius ) };
				shapeIds[shapeCount++] = b3CreateCapsuleShape( bodyId, &shapeDef, &capsule );
			}
			break;

			case pd_bodyShapeHull:
			{
				if ( s->hullIndex < hullCount && bakeInto( hulls + s->hullIndex, &blobs[i], (int)sizeof( blobs[i] ) ) )
				{
					shapeIds[shapeCount++] = b3CreateHullShape( bodyId, &shapeDef, &blobs[i].header );
				}
			}
			break;

			case pd_bodyShapeMesh:
				// Not reachable, and deliberately not supported here: this
				// scenario compares mass, inertia and bounds for a *dynamic*
				// body, and a mesh shape has no mass at all. It belongs to the
				// scene comparison, which steps a world.
				break;
		}
	}

	b3MassData md = b3Body_GetMassData( bodyId );
	out->mass = b3fToDouble( md.mass );
	out->localCenter = fromV( md.center );
	out->worldCenter = fromV( b3Body_GetWorldCenter( bodyId ) );
	out->invMass = b3iwToDouble( b3Body_GetInverseMass( bodyId ) );

	// Already per unit mass on this side.
	{
		const b3f* inertia = &md.inertia.cx.x;
		for ( int i = 0; i < 9; ++i )
		{
			out->unitInertia[i] = b3fToDouble( inertia[i] );
		}
	}

	{
		b3MatrixW invWorld = b3Body_GetWorldInverseRotationalInertia( bodyId );
		const b3iw* m = &invWorld.cx.x;
		for ( int i = 0; i < 9; ++i )
		{
			out->invInertiaWorld[i] = b3iwToDouble( m[i] );
		}
	}

	{
		b3World* world = b3GetWorldFromId( worldId );
		b3Body* body = b3GetBodyFullId( world, bodyId );
		b3BodySim* sim = b3GetBodySim( world, body );

		out->minExtent = b3fToDouble( sim->minExtent );
		out->maxExtent = fromV( sim->maxExtent );

		out->shapeCount = shapeCount;
		int shapeIndex = 0;
		int shapeId = body->headShapeId;
		while ( shapeId != B3_NULL_INDEX && shapeIndex < shapeCount )
		{
			b3Shape* shape = b3Array_Get( world->shapes, shapeId );

			// The list is built by prepending, so it runs in reverse creation
			// order on both sides. Report in creation order.
			int slot = shapeCount - 1 - shapeIndex;
			out->aabb[slot].lower = fromV( shape->aabb.lowerBound );
			out->aabb[slot].upper = fromV( shape->aabb.upperBound );
			out->fatAABB[slot].lower = fromV( shape->fatAABB.lowerBound );
			out->fatAABB[slot].upper = fromV( shape->fatAABB.upperBound );

			shapeIndex += 1;
			shapeId = shape->nextShapeId;
		}
	}

	b3DestroyWorld( worldId );

	(void)shapeIds;
}

static void portHullMass( const pdHull* hull, double density, pdMassOut* out )
{
	pdHullBlob blob;
	memset( out, 0, sizeof( *out ) );
	if ( bakeInto( hull, &blob, (int)sizeof( blob ) ) == false )
	{
		return;
	}

	b3MassData md = b3ComputeHullMass( &blob.header, b3fFromDouble( density ) );
	out->mass = b3fToDouble( md.mass );
	out->center = fromV( md.center );

	// Already per unit mass on this side; b3ComputeHullMass copies rather than
	// scaling, which is the point of the convention.
	const b3f* inertia = &md.inertia.cx.x;
	for ( int i = 0; i < 9; ++i )
	{
		out->unitInertia[i] = b3fToDouble( inertia[i] );
	}
}

static void portHullAABB( const pdHull* hull, const pdTransform* xf, pdAABB* out )
{
	pdHullBlob blob;
	memset( out, 0, sizeof( *out ) );
	if ( bakeInto( hull, &blob, (int)sizeof( blob ) ) == false )
	{
		return;
	}

	b3AABB aabb = b3ComputeHullAABB( &blob.header, toXf( xf ) );
	out->lower = fromV( aabb.lowerBound );
	out->upper = fromV( aabb.upperBound );
}

static void portHullRayCast( const pdHull* hull, pdVec3 origin, pdVec3 translation, pdCastOut* out )
{
	pdHullBlob blob;
	memset( out, 0, sizeof( *out ) );
	if ( bakeInto( hull, &blob, (int)sizeof( blob ) ) == false )
	{
		return;
	}

	b3RayCastInput input;
	input.origin = toV( origin );
	input.translation = toV( translation );
	input.maxFraction = b3c_one;

	b3CastOutput o = b3RayCastHull( &blob.header, &input );

	out->hit = o.hit;
	out->fraction = b3cToDouble( o.fraction );
	out->point = fromV( o.point );
	out->normal = fromV( o.normal );
}

static void portHullShapeCast( const pdHull* hull, const pdProxy* b, pdVec3 translation, pdCastOut* out )
{
	pdHullBlob blob;
	b3Vec3 sb[PD_MAX_POINTS];

	memset( out, 0, sizeof( *out ) );
	if ( bakeInto( hull, &blob, (int)sizeof( blob ) ) == false )
	{
		return;
	}

	b3ShapeCastInput input;
	input.proxy = toProxy( b, sb );
	input.translation = toV( translation );
	input.maxFraction = b3c_one;
	input.canEncroach = false;

	b3CastOutput o = b3ShapeCastHull( &blob.header, &input );

	out->hit = o.hit;
	out->fraction = b3cToDouble( o.fraction );
	out->point = fromV( o.point );
	out->normal = fromV( o.normal );
}

static bool portHullOverlap( const pdHull* hull, const pdTransform* xf, const pdProxy* b )
{
	pdHullBlob blob;
	b3Vec3 sb[PD_MAX_POINTS];

	if ( bakeInto( hull, &blob, (int)sizeof( blob ) ) == false )
	{
		return false;
	}

	b3ShapeProxy proxy = toProxy( b, sb );
	return b3OverlapHull( &blob.header, toXf( xf ), &proxy );
}

static void portHullSphere( const pdHull* hull, pdVec3 cB, double rB, const pdTransform* xf, pdManifoldOut* out )
{
	pdHullBlob blob;
	memset( out, 0, sizeof( *out ) );
	if ( bakeInto( hull, &blob, (int)sizeof( blob ) ) == false )
	{
		return;
	}

	b3Sphere sb = { toV( cB ), b3fFromDouble( rB ) };

	b3LocalManifoldPoint buffer[PD_MAX_MANIFOLD_POINTS] = { 0 };
	b3LocalManifold m = { 0 };
	m.points = buffer;

	b3SimplexCache cache = { 0 };
	b3CollideHullAndSphere( &m, PD_MAX_MANIFOLD_POINTS, &blob.header, &sb, toXf( xf ), &cache );
	grabManifold( &m, out );
}

static void portHullCapsule( const pdHull* hull, pdVec3 b1, pdVec3 b2, double rB, const pdTransform* xf,
							 pdManifoldOut* out )
{
	pdHullBlob blob;
	memset( out, 0, sizeof( *out ) );
	if ( bakeInto( hull, &blob, (int)sizeof( blob ) ) == false )
	{
		return;
	}

	b3Capsule cb = { toV( b1 ), toV( b2 ), b3fFromDouble( rB ) };

	b3LocalManifoldPoint buffer[PD_MAX_MANIFOLD_POINTS] = { 0 };
	b3LocalManifold m = { 0 };
	m.points = buffer;

	b3SimplexCache cache = { 0 };
	b3CollideHullAndCapsule( &m, PD_MAX_MANIFOLD_POINTS, &blob.header, &cb, toXf( xf ), &cache );
	grabManifold( &m, out );
}

// Capacity 4, not PD_MAX_MANIFOLD_POINTS: b3CollideHulls never writes more
// than four, and passing eight would hide a capacity bug on either side.
#define PD_HULL_HULL_CAPACITY 4

/// Bake both descriptions. Two blobs, not one reused -- they are live at the
/// same time. 8 KiB of host stack, against the ~1330 bytes a 32-vertex hull
/// actually needs.
static bool bakePair( const pdHull* a, const pdHull* b, pdHullBlob* blobA, pdHullBlob* blobB )
{
	return bakeInto( a, blobA, (int)sizeof( *blobA ) ) && bakeInto( b, blobB, (int)sizeof( *blobB ) );
}

static void grabFeature( const b3SATCache* cache, pdManifoldOut* out )
{
	out->feature = (int)cache->type;
	out->featureIndexA = (int)cache->indexA;
	out->featureIndexB = (int)cache->indexB;
}

static void portHullHull( const pdHull* a, const pdHull* b, const pdTransform* xf, pdManifoldOut* out )
{
	pdHullBlob blobA, blobB;
	memset( out, 0, sizeof( *out ) );
	if ( bakePair( a, b, &blobA, &blobB ) == false )
	{
		return;
	}

	b3LocalManifoldPoint buffer[PD_MAX_MANIFOLD_POINTS] = { 0 };
	b3LocalManifold m = { 0 };
	m.points = buffer;

	b3SATCache cache = { 0 };
	b3CollideHulls( &m, PD_HULL_HULL_CAPACITY, &blobA.header, &blobB.header, toXf( xf ), &cache );
	grabManifold( &m, out );
	grabFeature( &cache, out );
}

static void portHullHullCached( const pdHull* a, const pdHull* b, const pdTransform* xf1, const pdTransform* xf2,
								pdManifoldOut* out1, pdManifoldOut* out2, bool* hit1, bool* hit2 )
{
	pdHullBlob blobA, blobB;
	memset( out1, 0, sizeof( *out1 ) );
	memset( out2, 0, sizeof( *out2 ) );
	*hit1 = false;
	*hit2 = false;

	if ( bakePair( a, b, &blobA, &blobB ) == false )
	{
		return;
	}

	b3LocalManifoldPoint buffer[PD_MAX_MANIFOLD_POINTS] = { 0 };
	b3LocalManifold m = { 0 };
	m.points = buffer;

	// One cache carried across both calls -- that is the whole point.
	b3SATCache cache = { 0 };

	b3CollideHulls( &m, PD_HULL_HULL_CAPACITY, &blobA.header, &blobB.header, toXf( xf1 ), &cache );
	grabManifold( &m, out1 );
	grabFeature( &cache, out1 );
	*hit1 = cache.hit != 0;

	b3CollideHulls( &m, PD_HULL_HULL_CAPACITY, &blobA.header, &blobB.header, toXf( xf2 ), &cache );
	grabManifold( &m, out2 );
	grabFeature( &cache, out2 );
	*hit2 = cache.hit != 0;
}

// --- tree -----------------------------------------------------------------

static bool collect( int proxyId, uint64_t userData, void* context )
{
	(void)proxyId;
	pdTreeOut* out = (pdTreeOut*)context;
	if ( out->count < PD_MAX_TREE_RESULTS )
	{
		out->userData[out->count++] = (int)userData;
	}
	return true;
}

static b3c keepAll( const b3RayCastInput* input, int proxyId, uint64_t userData, void* context )
{
	(void)proxyId;
	pdTreeOut* out = (pdTreeOut*)context;
	if ( out->count < PD_MAX_TREE_RESULTS )
	{
		out->userData[out->count++] = (int)userData;
	}
	return input->maxFraction;
}

static b3DynamicTree buildTree( const pdAABB* boxes, int n )
{
	b3DynamicTree tree = b3DynamicTree_Create( n );
	for ( int i = 0; i < n; ++i )
	{
		b3AABB box = b3MakeAABB( toV( boxes[i].lower ), toV( boxes[i].upper ) );
		b3DynamicTree_CreateProxy( &tree, box, B3_DEFAULT_CATEGORY_BITS, (uint64_t)i );
	}
	return tree;
}

static void portTreeQuery( const pdAABB* boxes, int n, pdAABB query, pdTreeOut* out )
{
	b3DynamicTree tree = buildTree( boxes, n );

	memset( out, 0, sizeof( *out ) );
	b3AABB q = b3MakeAABB( toV( query.lower ), toV( query.upper ) );
	b3DynamicTree_Query( &tree, q, B3_DEFAULT_CATEGORY_BITS, false, collect, out );

	b3DynamicTree_Destroy( &tree );
}

static void portTreeRayCast( const pdAABB* boxes, int n, pdVec3 origin, pdVec3 translation, pdTreeOut* out )
{
	b3DynamicTree tree = buildTree( boxes, n );

	b3RayCastInput input;
	input.origin = toV( origin );
	input.translation = toV( translation );
	input.maxFraction = b3c_one;

	memset( out, 0, sizeof( *out ) );
	b3DynamicTree_RayCast( &tree, &input, B3_DEFAULT_CATEGORY_BITS, false, keepAll, out );

	b3DynamicTree_Destroy( &tree );
}

// --- scripted scenes ------------------------------------------------------

/// Map a shape id back to the creation-order index the scene description used.
static int sceneShapeIndex( const b3ShapeId* ids, int count, int rawShapeId )
{
	for ( int i = 0; i < count; ++i )
	{
		if ( ids[i].index1 - 1 == rawShapeId )
		{
			return i;
		}
	}
	return -1;
}

static int compareSceneContacts( const void* a, const void* b )
{
	const pdSceneContact* ca = (const pdSceneContact*)a;
	const pdSceneContact* cb = (const pdSceneContact*)b;
	if ( ca->shapeA != cb->shapeA )
	{
		return ca->shapeA < cb->shapeA ? -1 : 1;
	}
	if ( ca->shapeB != cb->shapeB )
	{
		return ca->shapeB < cb->shapeB ? -1 : 1;
	}
	return 0;
}

static void portWorldScene( const pdSceneDesc* desc, const pdHull* hulls, int hullCount, const pdMesh* meshes, int meshCount,
							pdSceneOut* out )
{
	memset( out, 0, sizeof( *out ) );

	// One blob per shape, alive for the whole scene: the port aliases the
	// caller's hull data rather than copying it, so these must outlive every
	// shape that references them.
	static pdHullBlob blobs[PD_MAX_SCENE_BODIES * PD_MAX_BODY_SHAPES];

	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = toV( desc->gravity );

	// Unlike the reference, the port sizes its pools from these and reaching
	// past one is an assert rather than a realloc. A mesh contact additionally
	// needs its manifold size class and its triangle cache reserved before the
	// first step -- meshContactCount is what does that.
	worldDef.capacity.staticBodyCount = PD_MAX_SCENE_BODIES;
	worldDef.capacity.dynamicBodyCount = PD_MAX_SCENE_BODIES;
	worldDef.capacity.staticShapeCount = PD_MAX_SCENE_BODIES * PD_MAX_BODY_SHAPES;
	worldDef.capacity.dynamicShapeCount = PD_MAX_SCENE_BODIES * PD_MAX_BODY_SHAPES;
	worldDef.capacity.contactCount = PD_MAX_SCENE_CONTACTS;
	worldDef.capacity.meshContactCount = meshCount > 0 ? PD_MAX_SCENE_CONTACTS : 0;
	worldDef.capacity.jointCount = PD_MAX_SCENE_JOINTS;

	b3WorldId worldId = b3CreateWorld( &worldDef );
	b3World* world = b3GetWorldFromId( worldId );

	// Zero means the port's default. Resolved here rather than at the call so
	// that the reference is driven with the same number.
	int subStepCount = desc->subStepCount > 0 ? desc->subStepCount : 4;

	b3BodyId bodyIds[PD_MAX_SCENE_BODIES];
	b3ShapeId shapeIds[PD_MAX_SCENE_BODIES * PD_MAX_BODY_SHAPES];
	int shapeCount = 0;

	for ( int b = 0; b < desc->bodyCount && b < PD_MAX_SCENE_BODIES; ++b )
	{
		const pdSceneBody* sb = desc->bodies + b;

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = sb->isStatic ? b3_staticBody : b3_dynamicBody;
		bodyDef.position = toV( sb->body.xf.p );
		bodyDef.rotation = toXf( &sb->body.xf ).q;
		bodyDef.enableSleep = sb->disableSleep == false;
		bodyIds[b] = b3CreateBody( worldId, &bodyDef );

		for ( int i = 0; i < sb->body.shapeCount && i < PD_MAX_BODY_SHAPES; ++i )
		{
			const pdBodyShape* s = sb->body.shapes + i;

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			shapeDef.density = b3fFromDouble( s->density );
			shapeDef.enableContactEvents = true;

			switch ( s->kind )
			{
				case pd_bodyShapeSphere:
				{
					b3Sphere sphere = { toV( s->p1 ), b3fFromDouble( s->radius ) };
					shapeIds[shapeCount++] = b3CreateSphereShape( bodyIds[b], &shapeDef, &sphere );
				}
				break;

				case pd_bodyShapeCapsule:
				{
					b3Capsule capsule = { toV( s->p1 ), toV( s->p2 ), b3fFromDouble( s->radius ) };
					shapeIds[shapeCount++] = b3CreateCapsuleShape( bodyIds[b], &shapeDef, &capsule );
				}
				break;

				case pd_bodyShapeHull:
				{
					if ( s->hullIndex < hullCount &&
						 bakeInto( hulls + s->hullIndex, &blobs[shapeCount], (int)sizeof( blobs[0] ) ) )
					{
						shapeIds[shapeCount] = b3CreateHullShape( bodyIds[b], &shapeDef, &blobs[shapeCount].header );
						shapeCount += 1;
					}
				}
				break;

				case pd_bodyShapeMesh:
				{
					// portMesh caches one baked blob per pdMesh in a static
					// array that outlives the world, which is the contract
					// b3CreateMeshShape needs -- it aliases the blob.
					const b3MeshData* data = s->meshIndex < meshCount ? portMesh( meshes + s->meshIndex ) : NULL;
					if ( data != NULL )
					{
						shapeIds[shapeCount] = b3CreateMeshShape( bodyIds[b], &shapeDef, data, b3MakeVec3( b3f_one, b3f_one, b3f_one ) );
						shapeCount += 1;
					}
				}
				break;
			}
		}

		if ( sb->isStatic == false )
		{
			b3Body_SetLinearVelocity( bodyIds[b], toV( sb->linearVelocity ) );
			b3Body_SetAngularVelocity( bodyIds[b], toV( sb->angularVelocity ) );
		}
	}

	b3JointId jointIds[PD_MAX_SCENE_JOINTS];
	int jointCount = desc->jointCount < PD_MAX_SCENE_JOINTS ? desc->jointCount : PD_MAX_SCENE_JOINTS;

	for ( int j = 0; j < jointCount; ++j )
	{
		const pdSceneJoint* sj = desc->joints + j;

		if ( sj->kind == pd_jointRevolute )
		{
			b3RevoluteJointDef rev = b3DefaultRevoluteJointDef();
			rev.base.bodyIdA = bodyIds[sj->bodyA];
			rev.base.bodyIdB = bodyIds[sj->bodyB];
			rev.base.localFrameA.p = toV( sj->localAnchorA );
			rev.base.localFrameB.p = toV( sj->localAnchorB );
			rev.base.collideConnected = sj->collideConnected;

			// Degrees to brads: 32768 to a full turn.
			rev.lowerAngle = (b3a)( sj->lowerAngleDeg * ( 32768.0 / 360.0 ) );
			rev.upperAngle = (b3a)( sj->upperAngleDeg * ( 32768.0 / 360.0 ) );
			rev.targetAngle = (b3a)( sj->targetAngleDeg * ( 32768.0 / 360.0 ) );

			rev.enableLimit = sj->enableAngleLimit;
			rev.enableMotor = sj->enableAngleMotor;
			rev.enableSpring = sj->enableAngleSpring;
			rev.hertz = b3fFromDouble( sj->angleHertz );
			rev.dampingRatio = b3fFromDouble( sj->angleDampingRatio );
			rev.motorSpeed = b3fFromDouble( sj->motorAngularSpeed );
			rev.maxMotorTorque = b3fFromDouble( sj->maxMotorTorque );

			jointIds[j] = b3CreateRevoluteJoint( worldId, &rev );
			continue;
		}

		if ( sj->kind == pd_jointSpherical )
		{
			b3SphericalJointDef ball = b3DefaultSphericalJointDef();
			ball.base.bodyIdA = bodyIds[sj->bodyA];
			ball.base.bodyIdB = bodyIds[sj->bodyB];
			ball.base.localFrameA.p = toV( sj->localAnchorA );
			ball.base.localFrameB.p = toV( sj->localAnchorB );
			ball.base.collideConnected = sj->collideConnected;

			// Degrees to brads, as for the revolute.
			ball.coneAngle = (b3a)( sj->coneAngleDeg * ( 32768.0 / 360.0 ) );
			ball.lowerTwistAngle = (b3a)( sj->lowerTwistDeg * ( 32768.0 / 360.0 ) );
			ball.upperTwistAngle = (b3a)( sj->upperTwistDeg * ( 32768.0 / 360.0 ) );

			ball.enableConeLimit = sj->enableConeLimit;
			ball.enableTwistLimit = sj->enableTwistLimit;
			ball.enableSpring = sj->enableBallSpring;
			ball.hertz = b3fFromDouble( sj->ballHertz );
			ball.dampingRatio = b3fFromDouble( sj->ballDampingRatio );
			ball.enableMotor = sj->enableBallMotor;
			ball.motorVelocity = toV( sj->ballMotorVelocity );
			ball.maxMotorTorque = b3fFromDouble( sj->ballMaxMotorTorque );

			jointIds[j] = b3CreateSphericalJoint( worldId, &ball );
			continue;
		}

		if ( sj->kind == pd_jointWeld )
		{
			b3WeldJointDef weld = b3DefaultWeldJointDef();
			weld.base.bodyIdA = bodyIds[sj->bodyA];
			weld.base.bodyIdB = bodyIds[sj->bodyB];
			weld.base.localFrameA.p = toV( sj->localAnchorA );
			weld.base.localFrameB.p = toV( sj->localAnchorB );
			weld.base.collideConnected = sj->collideConnected;

			weld.linearHertz = b3fFromDouble( sj->weldLinearHertz );
			weld.linearDampingRatio = b3fFromDouble( sj->weldLinearDampingRatio );
			weld.angularHertz = b3fFromDouble( sj->weldAngularHertz );
			weld.angularDampingRatio = b3fFromDouble( sj->weldAngularDampingRatio );

			jointIds[j] = b3CreateWeldJoint( worldId, &weld );
			continue;
		}

		if ( sj->kind == pd_jointMotor )
		{
			b3MotorJointDef motor = b3DefaultMotorJointDef();
			motor.base.bodyIdA = bodyIds[sj->bodyA];
			motor.base.bodyIdB = bodyIds[sj->bodyB];
			motor.base.localFrameA.p = toV( sj->localAnchorA );
			motor.base.localFrameB.p = toV( sj->localAnchorB );
			motor.base.collideConnected = sj->collideConnected;

			motor.linearVelocity = toV( sj->motorLinearVelocity );
			motor.angularVelocity = toV( sj->motorAngularVelocity );
			motor.maxVelocityForce = b3fFromDouble( sj->motorMaxVelocityForce );
			motor.maxVelocityTorque = b3fFromDouble( sj->motorMaxVelocityTorque );
			motor.linearHertz = b3fFromDouble( sj->motorLinearHertz );
			motor.linearDampingRatio = b3fFromDouble( sj->motorLinearDampingRatio );
			motor.angularHertz = b3fFromDouble( sj->motorAngularHertz );
			motor.angularDampingRatio = b3fFromDouble( sj->motorAngularDampingRatio );
			motor.maxSpringForce = b3fFromDouble( sj->motorMaxSpringForce );
			motor.maxSpringTorque = b3fFromDouble( sj->motorMaxSpringTorque );

			jointIds[j] = b3CreateMotorJoint( worldId, &motor );
			continue;
		}

		if ( sj->kind == pd_jointPrismatic )
		{
			b3PrismaticJointDef slide = b3DefaultPrismaticJointDef();
			slide.base.bodyIdA = bodyIds[sj->bodyA];
			slide.base.bodyIdB = bodyIds[sj->bodyB];
			slide.base.localFrameA.p = toV( sj->localAnchorA );
			slide.base.localFrameB.p = toV( sj->localAnchorB );
			slide.base.collideConnected = sj->collideConnected;

			// Every field a length or a speed, on both sides. No conversion.
			slide.targetTranslation = b3fFromDouble( sj->slideTargetTranslation );
			slide.lowerTranslation = b3fFromDouble( sj->slideLowerTranslation );
			slide.upperTranslation = b3fFromDouble( sj->slideUpperTranslation );
			slide.hertz = b3fFromDouble( sj->slideHertz );
			slide.dampingRatio = b3fFromDouble( sj->slideDampingRatio );
			slide.motorSpeed = b3fFromDouble( sj->slideMotorSpeed );
			slide.maxMotorForce = b3fFromDouble( sj->slideMaxMotorForce );
			slide.enableSpring = sj->enableSlideSpring;
			slide.enableLimit = sj->enableSlideLimit;
			slide.enableMotor = sj->enableSlideMotor;

			jointIds[j] = b3CreatePrismaticJoint( worldId, &slide );
			continue;
		}

		if ( sj->kind == pd_jointParallel )
		{
			b3ParallelJointDef parallel = b3DefaultParallelJointDef();
			parallel.base.bodyIdA = bodyIds[sj->bodyA];
			parallel.base.bodyIdB = bodyIds[sj->bodyB];
			parallel.base.localFrameA.p = toV( sj->localAnchorA );
			parallel.base.localFrameB.p = toV( sj->localAnchorB );
			parallel.base.collideConnected = sj->collideConnected;

			// A frequency, a dimensionless ratio and a torque budget: nothing
			// to convert, and no angle in the description at all. The angle the
			// joint acts on comes from the bodies' own orientations.
			parallel.hertz = b3fFromDouble( sj->parallelHertz );
			parallel.dampingRatio = b3fFromDouble( sj->parallelDampingRatio );
			parallel.maxTorque = b3fFromDouble( sj->parallelMaxTorque );

			jointIds[j] = b3CreateParallelJoint( worldId, &parallel );
			continue;
		}

		if ( sj->kind == pd_jointWheel )
		{
			b3WheelJointDef wheel = b3DefaultWheelJointDef();
			wheel.base.bodyIdA = bodyIds[sj->bodyA];
			wheel.base.bodyIdB = bodyIds[sj->bodyB];
			wheel.base.localFrameA.p = toV( sj->localAnchorA );
			wheel.base.localFrameB.p = toV( sj->localAnchorB );
			wheel.base.collideConnected = sj->collideConnected;

			// Lengths and speeds: no conversion.
			wheel.suspensionHertz = b3fFromDouble( sj->wheelSuspensionHertz );
			wheel.suspensionDampingRatio = b3fFromDouble( sj->wheelSuspensionDampingRatio );
			wheel.lowerSuspensionLimit = b3fFromDouble( sj->wheelLowerSuspensionLimit );
			wheel.upperSuspensionLimit = b3fFromDouble( sj->wheelUpperSuspensionLimit );
			wheel.spinSpeed = b3fFromDouble( sj->wheelSpinSpeed );
			wheel.maxSpinTorque = b3fFromDouble( sj->wheelMaxSpinTorque );
			wheel.steeringHertz = b3fFromDouble( sj->wheelSteeringHertz );
			wheel.steeringDampingRatio = b3fFromDouble( sj->wheelSteeringDampingRatio );
			wheel.maxSteeringTorque = b3fFromDouble( sj->wheelMaxSteeringTorque );

			// Degrees in, brads out.
			wheel.targetSteeringAngle = (b3a)( sj->wheelTargetSteeringDeg * ( 32768.0 / 360.0 ) );
			wheel.lowerSteeringLimit = (b3a)( sj->wheelLowerSteeringDeg * ( 32768.0 / 360.0 ) );
			wheel.upperSteeringLimit = (b3a)( sj->wheelUpperSteeringDeg * ( 32768.0 / 360.0 ) );

			wheel.enableSuspensionSpring = sj->enableWheelSuspensionSpring;
			wheel.enableSuspensionLimit = sj->enableWheelSuspensionLimit;
			wheel.enableSpinMotor = sj->enableWheelSpinMotor;
			wheel.enableSteering = sj->enableWheelSteering;
			wheel.enableSteeringLimit = sj->enableWheelSteeringLimit;

			jointIds[j] = b3CreateWheelJoint( worldId, &wheel );
			continue;
		}

		b3DistanceJointDef jointDef = b3DefaultDistanceJointDef();
		jointDef.base.bodyIdA = bodyIds[sj->bodyA];
		jointDef.base.bodyIdB = bodyIds[sj->bodyB];
		jointDef.base.localFrameA.p = toV( sj->localAnchorA );
		jointDef.base.localFrameB.p = toV( sj->localAnchorB );
		jointDef.base.collideConnected = sj->collideConnected;

		jointDef.length = b3fFromDouble( sj->length );
		jointDef.enableSpring = sj->enableSpring;
		jointDef.hertz = b3fFromDouble( sj->hertz );
		jointDef.dampingRatio = b3fFromDouble( sj->dampingRatio );
		jointDef.enableLimit = sj->enableLimit;
		jointDef.minLength = b3fFromDouble( sj->minLength );
		jointDef.maxLength = b3fFromDouble( sj->maxLength );
		jointDef.enableMotor = sj->enableMotor;
		jointDef.maxMotorForce = b3fFromDouble( sj->maxMotorForce );
		jointDef.motorSpeed = b3fFromDouble( sj->motorSpeed );

		// Zero means "leave the default", which is the port's unbounded
		// sentinel. A scene that wants a real bound sets both.
		if ( sj->lowerSpringForce != 0.0 || sj->upperSpringForce != 0.0 )
		{
			jointDef.lowerSpringForce = b3fFromDouble( sj->lowerSpringForce );
			jointDef.upperSpringForce = b3fFromDouble( sj->upperSpringForce );
		}

		jointIds[j] = b3CreateDistanceJoint( worldId, &jointDef );
	}

	int passCount = desc->passCount < PD_MAX_SCENE_PASSES ? desc->passCount : PD_MAX_SCENE_PASSES;
	out->passCount = passCount;

	for ( int p = 0; p < passCount; ++p )
	{
		const pdScenePass* pass = desc->passes + p;

		for ( int m = 0; m < pass->moveCount && m < PD_MAX_SCENE_BODIES; ++m )
		{
			const pdSceneMove* move = pass->moves + m;
			if ( move->bodyIndex < 0 || move->bodyIndex >= desc->bodyCount )
			{
				continue;
			}
			b3Transform xf = toXf( &move->xf );
			b3Body_SetTransform( bodyIds[move->bodyIndex], xf.p, xf.q );
		}

		// Phase 3C-i moved the contact-event lifecycle into b3World_Step, so
		// even a collide-only pass has to go through it rather than calling
		// b3Collide directly. A collide-only scene zeroes gravity and starts
		// every body at rest, so the step integrates nothing and the
		// comparison is unchanged -- which is also what lets the reference be
		// driven by b3World_Step( 0, 1 ).
		//
		// The one-sub-step spelling is deliberate for that case: with nothing
		// to integrate, more sub-steps only cost time, and it is the count the
		// reference's zero-dt call uses.
		if ( pass->stepCount <= 0 )
		{
			b3World_Step( worldId, 1 );
		}
		else
		{
			for ( int s = 0; s < pass->stepCount; ++s )
			{
				b3World_Step( worldId, subStepCount );
			}
		}

		pdScenePassOut* po = out->passes + p;

		for ( int i = 0; i < world->contacts.count && po->contactCount < PD_MAX_SCENE_CONTACTS; ++i )
		{
			b3Contact* contact = world->contacts.data + i;
			if ( contact->contactId == B3_NULL_INDEX )
			{
				continue;
			}

			pdSceneContact* sc = po->contacts + po->contactCount;
			po->contactCount += 1;

			sc->shapeA = sceneShapeIndex( shapeIds, shapeCount, contact->shapeIdA );
			sc->shapeB = sceneShapeIndex( shapeIds, shapeCount, contact->shapeIdB );
			sc->touching = ( contact->flags & b3_contactTouchingFlag ) != 0;

			sc->manifoldCount = contact->manifoldCount < PD_MAX_SCENE_MANIFOLDS ? contact->manifoldCount
																			  : PD_MAX_SCENE_MANIFOLDS;
			for ( int m = 0; m < sc->manifoldCount; ++m )
			{
				const b3Manifold* manifold = contact->manifolds + m;
				sc->manifolds[m].normal = fromV( manifold->normal );
				sc->manifolds[m].pointCount = manifold->pointCount;
				for ( int k = 0; k < manifold->pointCount && k < PD_MAX_MANIFOLD_POINTS; ++k )
				{
					sc->manifolds[m].points[k] = fromV( manifold->points[k].anchorA );
					sc->manifolds[m].separations[k] = b3fToDouble( manifold->points[k].separation );
					sc->manifolds[m].featureIds[k] = manifold->points[k].featureId;
					sc->manifolds[m].triangleIndices[k] = manifold->points[k].triangleIndex;
				}
			}
		}

		qsort( po->contacts, (size_t)po->contactCount, sizeof( pdSceneContact ), compareSceneContacts );

		b3ContactEvents events = b3World_GetContactEvents( worldId );
		po->beginCount = events.beginCount;
		po->endCount = events.endCount;

		po->bodyCount = desc->bodyCount < PD_MAX_SCENE_BODIES ? desc->bodyCount : PD_MAX_SCENE_BODIES;
		for ( int b = 0; b < po->bodyCount; ++b )
		{
			pdSceneBodyOut* bo = po->bodies + b;
			bo->xf = fromXf( b3Body_GetTransform( bodyIds[b] ) );
			bo->linearVelocity = fromV( b3Body_GetLinearVelocity( bodyIds[b] ) );
			bo->angularVelocity = fromV( b3Body_GetAngularVelocity( bodyIds[b] ) );
			bo->awake = b3Body_IsAwake( bodyIds[b] );
		}

		po->jointCount = jointCount;
		for ( int j = 0; j < jointCount; ++j )
		{
			pdSceneJointOut* jo = po->joints + j;
			jo->force = fromV( b3Joint_GetConstraintForce( jointIds[j] ) );
			jo->torque = fromV( b3Joint_GetConstraintTorque( jointIds[j] ) );

			if ( desc->joints[j].kind == pd_jointRevolute )
			{
				jo->angleDeg = (double)b3RevoluteJoint_GetAngle( jointIds[j] ) * ( 360.0 / 32768.0 );
			}
			else if ( desc->joints[j].kind == pd_jointSpherical )
			{
				jo->coneAngleDeg = (double)b3SphericalJoint_GetConeAngle( jointIds[j] ) * ( 360.0 / 32768.0 );
				jo->twistAngleDeg = (double)b3SphericalJoint_GetTwistAngle( jointIds[j] ) * ( 360.0 / 32768.0 );
			}
			else if ( desc->joints[j].kind == pd_jointPrismatic )
			{
				jo->translation = b3fToDouble( b3PrismaticJoint_GetTranslation( jointIds[j] ) );
				jo->slideSpeed = b3fToDouble( b3PrismaticJoint_GetSpeed( jointIds[j] ) );
			}
			else if ( desc->joints[j].kind == pd_jointWheel )
			{
				jo->suspensionTranslation =
					b3fToDouble( b3WheelJoint_GetSuspensionTranslation( jointIds[j] ) );
				jo->spinSpeed = b3fToDouble( b3WheelJoint_GetSpinSpeed( jointIds[j] ) );
				jo->steeringAngleDeg = (double)b3WheelJoint_GetSteeringAngle( jointIds[j] ) * ( 360.0 / 32768.0 );
			}
			else if ( desc->joints[j].kind == pd_jointWeld || desc->joints[j].kind == pd_jointMotor ||
					  desc->joints[j].kind == pd_jointParallel )
			{
				// None of the three has an angle or a length of its own: a weld
				// locks the whole relative transform, a motor constrains
				// nothing, and a parallel joint's state *is* the two bodies'
				// relative orientation, which the scene already compares body
				// by body. The reaction force and torque read above are the
				// comparison, plus the body trajectories every scene compares.
			}
			else
			{
				jo->currentLength = b3fToDouble( b3DistanceJoint_GetCurrentLength( jointIds[j] ) );
			}
		}
	}

	b3DestroyWorld( worldId );
}

const pdBackend pdPortBackend = {
	.name = "fixed",
	.distance = portDistance,
	.shapeCast = portShapeCast,
	.sphereSphere = portSphereSphere,
	.capsuleSphere = portCapsuleSphere,
	.capsuleCapsule = portCapsuleCapsule,
	.treeQuery = portTreeQuery,
	.treeRayCast = portTreeRayCast,
	.hullMass = portHullMass,
	.hullAABB = portHullAABB,
	.hullRayCast = portHullRayCast,
	.hullShapeCast = portHullShapeCast,
	.hullOverlap = portHullOverlap,
	.hullSphere = portHullSphere,
	.hullCapsule = portHullCapsule,
	.hullHull = portHullHull,
	.hullHullCached = portHullHullCached,
	.triangleSphere = portTriangleSphere,
	.triangleCapsule = portTriangleCapsule,
	.triangleHull = portTriangleHull,

	.meshAABB = portMeshAABB,
	.meshQuery = portMeshQuery,
	.meshTriangle = portMeshTriangle,

	.worldBody = portWorldBody,
	.worldScene = portWorldScene,
};
