// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Warioware64, 2026

// The float reference side of run_pair.
//
// This translation unit sees ONLY pristine upstream Box3D headers. It calls
// upstream by its ordinary names; the build then runs objcopy over this object
// with the same rename list applied to libbox3d_ref.a, so every b3* reference
// here resolves to up_b3* and cannot collide with the port's symbols of the
// same name. Nothing in this file needs to know that happened.
//
// The functions it exports are named pdRef*, which does not start with b3, so
// the rename leaves them alone and run_pair.c can find them.

#include "pair_iface.h"

#include "box3d/box3d.h"
#include "box3d/collision.h"
#include "box3d/math_functions.h"
#include "box3d/types.h"

// Upstream's internal headers. The world-body comparison reads minExtent,
// maxExtent and fatAABB, which are on b3BodySim and b3Shape and have no public
// accessor. That is fine here for the same reason the rest of this file is:
// the object is compiled against upstream alone and then renamed wholesale, so
// nothing internal escapes into run_pair.
#include "body.h"
#include "physics_world.h"
#include "shape.h"

#include <stdlib.h>
#include <string.h>

static b3Vec3 toV( pdVec3 v )
{
	return B3_LITERAL( b3Vec3 ){ (float)v.x, (float)v.y, (float)v.z };
}

static pdVec3 fromV( b3Vec3 v )
{
	pdVec3 r = { v.x, v.y, v.z };
	return r;
}

static b3Transform toXf( const pdTransform* xf )
{
	b3Transform t;
	t.p = toV( xf->p );
	t.q.v = B3_LITERAL( b3Vec3 ){ (float)xf->qx, (float)xf->qy, (float)xf->qz };
	t.q.s = (float)xf->qw;
	return t;
}

static pdTransform fromXf( b3WorldTransform t )
{
	pdTransform xf;
	xf.p = fromV( t.p );
	xf.qx = t.q.v.x;
	xf.qy = t.q.v.y;
	xf.qz = t.q.v.z;
	xf.qw = t.q.s;
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
	proxy.radius = (float)p->radius;
	return proxy;
}

static void refDistance( const pdProxy* a, const pdProxy* b, const pdTransform* xf, bool useRadii, pdDistanceOut* out )
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
	out->distance = o.distance;
	out->pointA = fromV( o.pointA );
	out->pointB = fromV( o.pointB );
	out->normal = fromV( o.normal );
}

static void refShapeCast( const pdProxy* a, const pdProxy* b, const pdTransform* xf, pdVec3 translation, pdCastOut* out )
{
	b3Vec3 sa[PD_MAX_POINTS], sb[PD_MAX_POINTS];

	b3ShapeCastPairInput input;
	input.proxyA = toProxy( a, sa );
	input.proxyB = toProxy( b, sb );
	input.transform = toXf( xf );
	input.translationB = toV( translation );
	input.maxFraction = 1.0f;
	input.canEncroach = false;

	b3CastOutput o = b3ShapeCast( &input );

	memset( out, 0, sizeof( *out ) );
	out->hit = o.hit;
	out->fraction = o.fraction;
	out->point = fromV( o.point );
	out->normal = fromV( o.normal );
}

static b3Sweep toSweep( const pdSweep* s )
{
	b3Sweep out;
	out.localCenter = toV( s->localCenter );
	out.c1 = toV( s->c1 );
	out.c2 = toV( s->c2 );
	out.q1 = ( b3Quat ){ { (float)s->q1x, (float)s->q1y, (float)s->q1z }, (float)s->q1w };
	out.q2 = ( b3Quat ){ { (float)s->q2x, (float)s->q2y, (float)s->q2z }, (float)s->q2w };
	return out;
}

static void refTimeOfImpact( const pdProxy* a, const pdProxy* b, const pdSweep* sweepA, const pdSweep* sweepB,
							 double maxFraction, pdTOIOut* out )
{
	b3Vec3 sa[PD_MAX_POINTS], sb[PD_MAX_POINTS];

	b3TOIInput input;
	input.proxyA = toProxy( a, sa );
	input.proxyB = toProxy( b, sb );
	input.sweepA = toSweep( sweepA );
	input.sweepB = toSweep( sweepB );
	input.maxFraction = (float)maxFraction;

	b3TOIOutput o = b3TimeOfImpact( &input );

	memset( out, 0, sizeof( *out ) );
	out->state = (int)o.state;
	out->fraction = o.fraction;
	out->distance = o.distance;
	out->point = fromV( o.point );
	out->normal = fromV( o.normal );
	out->distanceIterations = o.distanceIterations;
	out->pushBackIterations = o.pushBackIterations;
	out->rootIterations = o.rootIterations;
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
		out->separations[i] = m->points[i].separation;
	}
}

static void refSphereSphere( pdVec3 cA, double rA, pdVec3 cB, double rB, const pdTransform* xf, pdManifoldOut* out )
{
	b3Sphere sa = { toV( cA ), (float)rA };
	b3Sphere sb = { toV( cB ), (float)rB };

	b3LocalManifoldPoint buffer[PD_MAX_MANIFOLD_POINTS] = { 0 };
	b3LocalManifold m = { 0 };
	m.points = buffer;

	b3CollideSpheres( &m, PD_MAX_MANIFOLD_POINTS, &sa, &sb, toXf( xf ) );
	grabManifold( &m, out );
}

static void refCapsuleSphere( pdVec3 a1, pdVec3 a2, double rA, pdVec3 cB, double rB, const pdTransform* xf,
							  pdManifoldOut* out )
{
	b3Capsule ca = { toV( a1 ), toV( a2 ), (float)rA };
	b3Sphere sb = { toV( cB ), (float)rB };

	b3LocalManifoldPoint buffer[PD_MAX_MANIFOLD_POINTS] = { 0 };
	b3LocalManifold m = { 0 };
	m.points = buffer;

	b3CollideCapsuleAndSphere( &m, PD_MAX_MANIFOLD_POINTS, &ca, &sb, toXf( xf ) );
	grabManifold( &m, out );
}

static void refCapsuleCapsule( pdVec3 a1, pdVec3 a2, double rA, pdVec3 b1, pdVec3 b2, double rB, const pdTransform* xf,
							   pdManifoldOut* out )
{
	b3Capsule ca = { toV( a1 ), toV( a2 ), (float)rA };
	b3Capsule cb = { toV( b1 ), toV( b2 ), (float)rB };

	b3LocalManifoldPoint buffer[PD_MAX_MANIFOLD_POINTS] = { 0 };
	b3LocalManifold m = { 0 };
	m.points = buffer;

	b3CollideCapsules( &m, PD_MAX_MANIFOLD_POINTS, &ca, &cb, toXf( xf ) );
	grabManifold( &m, out );
}

// --- hulls ----------------------------------------------------------------
//
// The reference owns every hull, because it owns the only hull builder in the
// process. It hands out a description and keeps the hull itself; the port
// bakes the description. Nothing here is freed -- run_pair is a test binary
// that builds a handful of hulls and exits.

#define PD_REF_MAX_HULLS 32

static b3HullData* s_hulls[PD_REF_MAX_HULLS];
static b3BoxHull s_boxHulls[PD_REF_MAX_HULLS];
static int s_hullCount = 0;

static const b3HullData* refHull( const pdHull* hull )
{
	return s_hulls[hull->refId];
}

/// Describe a built hull in neutral terms, and record the per-unit-mass form
/// of its inertia -- upstream stores the volume-weighted tensor, the port
/// stores it divided through, so the conversion belongs on this side where
/// the volume is still a float rather than a Q12 value.
static bool describeHull( const b3HullData* hull, int refId, pdHull* out )
{
	if ( hull->vertexCount > PD_MAX_HULL_VERTICES || hull->faceCount > PD_MAX_HULL_FACES ||
		 hull->edgeCount > PD_MAX_HULL_EDGES )
	{
		return false;
	}

	memset( out, 0, sizeof( *out ) );
	out->refId = refId;
	out->vertexCount = hull->vertexCount;
	out->edgeCount = hull->edgeCount;
	out->faceCount = hull->faceCount;

	const b3Vec3* points = b3GetHullPoints( hull );
	const b3HullVertex* vertices = b3GetHullVertices( hull );
	const b3HullHalfEdge* edges = b3GetHullEdges( hull );
	const b3HullFace* faces = b3GetHullFaces( hull );
	const b3Plane* planes = b3GetHullPlanes( hull );

	for ( int i = 0; i < hull->vertexCount; ++i )
	{
		out->points[i] = fromV( points[i] );
		out->vertexEdge[i] = vertices[i].edge;
	}

	for ( int i = 0; i < hull->edgeCount; ++i )
	{
		out->edges[i].next = edges[i].next;
		out->edges[i].twin = edges[i].twin;
		out->edges[i].origin = edges[i].origin;
		out->edges[i].face = edges[i].face;
	}

	for ( int i = 0; i < hull->faceCount; ++i )
	{
		out->faceEdge[i] = faces[i].edge;
		out->planeNormal[i] = fromV( planes[i].normal );
		out->planeOffset[i] = planes[i].offset;
	}

	out->center = fromV( hull->center );
	out->volume = hull->volume;
	out->surfaceArea = hull->surfaceArea;
	out->innerRadius = hull->innerRadius;

	const float* inertia = &hull->centralInertia.cx.x;
	for ( int i = 0; i < 9; ++i )
	{
		out->unitInertia[i] = inertia[i] / hull->volume;
	}

	return true;
}

bool pdRefMakeHull( pdHullKind kind, const double* params, pdHull* out )
{
	if ( s_hullCount >= PD_REF_MAX_HULLS )
	{
		return false;
	}

	int id = s_hullCount;
	const b3HullData* hull = NULL;

	switch ( kind )
	{
		case pd_hullBox:
			s_boxHulls[id] = b3MakeBoxHull( (float)params[0], (float)params[1], (float)params[2] );
			s_hulls[id] = &s_boxHulls[id].base;
			hull = s_hulls[id];
			break;

		case pd_hullCylinder:
			s_hulls[id] = b3CreateCylinder( (float)params[0], (float)params[1], (float)params[2], (int)params[3] );
			hull = s_hulls[id];
			break;

		case pd_hullCone:
			s_hulls[id] = b3CreateCone( (float)params[0], (float)params[1], (float)params[2], (int)params[3] );
			hull = s_hulls[id];
			break;

		case pd_hullRock:
			s_hulls[id] = b3CreateRock( (float)params[0] );
			hull = s_hulls[id];
			break;

		default:
			return false;
	}

	if ( hull == NULL || describeHull( hull, id, out ) == false )
	{
		return false;
	}

	s_hullCount++;
	return true;
}

// =========================================================================
// Worlds and bodies
// =========================================================================

/// Build a world, create one dynamic body from the description, and read back
/// what upstream computed. The world is torn down before returning.
static void refWorldBody( const pdBodyDesc* desc, const pdHull* hulls, int hullCount, pdBodyOut* out )
{
	memset( out, 0, sizeof( *out ) );

	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = toV( desc->xf.p );
	bodyDef.rotation.v = B3_LITERAL( b3Vec3 ){ (float)desc->xf.qx, (float)desc->xf.qy, (float)desc->xf.qz };
	bodyDef.rotation.s = (float)desc->xf.qw;

	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

	b3ShapeId shapeIds[PD_MAX_BODY_SHAPES];
	int shapeCount = 0;

	for ( int i = 0; i < desc->shapeCount && i < PD_MAX_BODY_SHAPES; ++i )
	{
		const pdBodyShape* s = desc->shapes + i;

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.density = (float)s->density;

		switch ( s->kind )
		{
			case pd_bodyShapeSphere:
			{
				b3Sphere sphere = { toV( s->p1 ), (float)s->radius };
				shapeIds[shapeCount++] = b3CreateSphereShape( bodyId, &shapeDef, &sphere );
			}
			break;

			case pd_bodyShapeCapsule:
			{
				b3Capsule capsule = { toV( s->p1 ), toV( s->p2 ), (float)s->radius };
				shapeIds[shapeCount++] = b3CreateCapsuleShape( bodyId, &shapeDef, &capsule );
			}
			break;

			case pd_bodyShapeHull:
			{
				if ( s->hullIndex < hullCount )
				{
					shapeIds[shapeCount++] = b3CreateHullShape( bodyId, &shapeDef, refHull( hulls + s->hullIndex ) );
				}
			}
			break;
		}
	}

	b3MassData md = b3Body_GetMassData( bodyId );
	out->mass = md.mass;
	out->localCenter = fromV( md.center );
	out->worldCenter = fromV( b3Body_GetWorldCenter( bodyId ) );
	out->invMass = b3Body_GetInverseMass( bodyId );

	// Upstream's tensor is absolute, so divide the mass back out to compare
	// against the port's convention -- the same conversion describeHull does.
	{
		const float* inertia = &md.inertia.cx.x;
		for ( int i = 0; i < 9; ++i )
		{
			out->unitInertia[i] = md.mass != 0.0f ? inertia[i] / md.mass : 0.0;
		}
	}

	{
		b3Matrix3 invWorld = b3Body_GetWorldInverseRotationalInertia( bodyId );
		const float* m = &invWorld.cx.x;
		for ( int i = 0; i < 9; ++i )
		{
			out->invInertiaWorld[i] = m[i];
		}
	}

	// minExtent, maxExtent and fatAABB live on b3BodySim and b3Shape, which are
	// internal. Reaching them is legitimate here -- pair_ref.o is compiled with
	// upstream's src/ on the include path and then renamed wholesale -- and it
	// is the honest thing to compare: reproducing the accumulation on this side
	// would test this file rather than the library.
	{
		b3World* world = b3GetWorldFromId( worldId );
		b3Body* body = b3GetBodyFullId( world, bodyId );
		b3BodySim* sim = b3GetBodySim( world, body );

		out->minExtent = sim->minExtent;
		out->maxExtent = fromV( sim->maxExtent );

		out->shapeCount = shapeCount;
		int shapeIndex = 0;
		int shapeId = body->headShapeId;
		while ( shapeId != B3_NULL_INDEX && shapeIndex < shapeCount )
		{
			b3Shape* shape = b3Array_Get( world->shapes, shapeId );

			// The body's shape list is built by prepending, so it runs in the
			// reverse of creation order. Report in creation order so the two
			// sides line up index for index.
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
}

// --- triangles ------------------------------------------------------------

static void refTriangleSphere( const pdVec3 tri[3], pdVec3 center, double radius, pdManifoldOut* out )
{
	memset( out, 0, sizeof( *out ) );

	b3Vec3 points[3] = { toV( tri[0] ), toV( tri[1] ), toV( tri[2] ) };
	b3Sphere sphere = { toV( center ), (float)radius };

	b3LocalManifoldPoint buffer[PD_MAX_MANIFOLD_POINTS] = { 0 };
	b3LocalManifold m = { 0 };
	m.points = buffer;

	b3CollideTriangleAndSphere( &m, PD_MAX_MANIFOLD_POINTS, points, &sphere );
	grabManifold( &m, out );
	out->triangleFeature = (int)m.feature;
}

static void refTriangleCapsule( const pdVec3 tri[3], pdVec3 c1, pdVec3 c2, double radius, bool reuse,
								 pdManifoldOut* out )
{
	memset( out, 0, sizeof( *out ) );

	b3Vec3 points[3] = { toV( tri[0] ), toV( tri[1] ), toV( tri[2] ) };
	b3Capsule capsule = { toV( c1 ), toV( c2 ), (float)radius };

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

// Defined with the hull-versus-hull block below.
static void grabFeature( const b3SATCache* cache, pdManifoldOut* out );

static void refTriangleHull( const pdVec3 tri[3], const pdHull* hull, const pdTransform* xf, bool reuse,
							  pdManifoldOut* out, bool* cacheHit )
{
	const b3HullData* refData = refHull( hull );
	memset( out, 0, sizeof( *out ) );
	*cacheHit = false;

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
		b3CollideTriangleAndHull( &first, PD_TRIANGLE_HULL_CAPACITY, points[0], points[1], points[2], 0, refData,
								  &cache, true );
	}

	b3CollideTriangleAndHull( &m, PD_TRIANGLE_HULL_CAPACITY, points[0], points[1], points[2], 0, refData, &cache,
							  true );
	grabManifold( &m, out );
	grabFeature( &cache, out );
	out->triangleFeature = (int)m.feature;
	*cacheHit = cache.hit != 0;
}

// --- triangle meshes ------------------------------------------------------
//
// Same arrangement as hulls: the reference owns the mesh, hands out a
// description, and the port bakes it. The generators below are plain geometry
// rather than a builder, but they still live here so that upstream's
// b3CreateMesh sees byte-for-byte the vertices the baker is given.

#define PD_REF_MAX_MESHES 8

static b3MeshData* s_meshes[PD_REF_MAX_MESHES];
static int s_meshCount = 0;

static b3Mesh refMesh( const pdMesh* mesh, pdVec3 scale )
{
	b3Mesh out;
	out.data = s_meshes[mesh->refId];
	out.scale = b3SafeScale( toV( scale ) );
	return out;
}

static void meshPush( pdMesh* out, double x, double y, double z )
{
	out->vertices[out->vertexCount].x = x;
	out->vertices[out->vertexCount].y = y;
	out->vertices[out->vertexCount].z = z;
	out->vertexCount++;
}

static void meshTri( pdMesh* out, int a, int b, int c )
{
	out->indices[3 * out->triangleCount + 0] = a;
	out->indices[3 * out->triangleCount + 1] = b;
	out->indices[3 * out->triangleCount + 2] = c;
	out->triangleCount++;
}

/// A grid in the XZ plane, optionally displaced in Y so the tree has real
/// depth to it and the shared edges are not all coplanar.
static bool genGrid( const double* params, pdMesh* out )
{
	double half = params[0];
	double halfZ = params[1];
	int divisions = (int)params[2];
	double wave = params[3];

	if ( divisions < 1 || ( divisions + 1 ) * ( divisions + 1 ) > PD_MAX_MESH_VERTICES ||
		 2 * divisions * divisions > PD_MAX_MESH_TRIANGLES )
	{
		return false;
	}

	int n = divisions + 1;
	for ( int z = 0; z < n; ++z )
	{
		for ( int x = 0; x < n; ++x )
		{
			double fx = -half + 2.0 * half * x / divisions;
			double fz = -halfZ + 2.0 * halfZ * z / divisions;
			meshPush( out, fx, wave * sin( 1.7 * fx ) * cos( 1.3 * fz ), fz );
		}
	}

	for ( int z = 0; z < divisions; ++z )
	{
		for ( int x = 0; x < divisions; ++x )
		{
			int i0 = z * n + x;
			int i1 = z * n + x + 1;
			int i2 = ( z + 1 ) * n + x;
			int i3 = ( z + 1 ) * n + x + 1;

			meshTri( out, i0, i2, i1 );
			meshTri( out, i1, i2, i3 );
		}
	}

	return true;
}

/// A flat inclined plane. Every interior edge is coplanar, which is the case
/// the ghost filter cares about most.
static bool genRamp( const double* params, pdMesh* out )
{
	double half = params[0];
	double halfZ = params[1];
	double height = params[2];
	int divisions = (int)params[3];

	if ( divisions < 1 || ( divisions + 1 ) * ( divisions + 1 ) > PD_MAX_MESH_VERTICES ||
		 2 * divisions * divisions > PD_MAX_MESH_TRIANGLES )
	{
		return false;
	}

	int n = divisions + 1;
	for ( int z = 0; z < n; ++z )
	{
		for ( int x = 0; x < n; ++x )
		{
			double fx = -half + 2.0 * half * x / divisions;
			double fz = -halfZ + 2.0 * halfZ * z / divisions;
			meshPush( out, fx, height * ( fz + halfZ ) / ( 2.0 * halfZ ), fz );
		}
	}

	for ( int z = 0; z < divisions; ++z )
	{
		for ( int x = 0; x < divisions; ++x )
		{
			int i0 = z * n + x;
			int i1 = z * n + x + 1;
			int i2 = ( z + 1 ) * n + x;
			int i3 = ( z + 1 ) * n + x + 1;

			meshTri( out, i0, i2, i1 );
			meshTri( out, i1, i2, i3 );
		}
	}

	return true;
}

/// A staircase: alternating concave and convex right-angle edges, which is
/// what makes the concave / inverse-concave classification observable.
static bool genStairs( const double* params, pdMesh* out )
{
	double width = params[0];
	double run = params[1];
	double rise = params[2];
	int steps = (int)params[3];

	if ( steps < 1 || 4 * steps > PD_MAX_MESH_VERTICES || 4 * steps > PD_MAX_MESH_TRIANGLES )
	{
		return false;
	}

	for ( int s = 0; s < steps; ++s )
	{
		double z0 = s * run;
		double z1 = z0 + run;
		double y0 = s * rise;
		double y1 = y0 + rise;

		int base = out->vertexCount;

		// Tread, then riser.
		meshPush( out, -width, y0, z0 );
		meshPush( out, width, y0, z0 );
		meshPush( out, -width, y0, z1 );
		meshPush( out, width, y0, z1 );
		meshPush( out, -width, y1, z1 );
		meshPush( out, width, y1, z1 );

		meshTri( out, base + 0, base + 2, base + 1 );
		meshTri( out, base + 1, base + 2, base + 3 );
		meshTri( out, base + 2, base + 4, base + 3 );
		meshTri( out, base + 3, base + 4, base + 5 );
	}

	return true;
}

/// A hemispherical bowl, so the tree has to separate curved geometry rather
/// than an axis-aligned slab.
static bool genBowl( const double* params, pdMesh* out )
{
	double radius = params[0];
	double depth = params[1];
	int rings = (int)params[2];
	int segments = (int)params[3];

	if ( rings < 1 || segments < 3 || ( rings + 1 ) * segments + 1 > PD_MAX_MESH_VERTICES ||
		 2 * rings * segments > PD_MAX_MESH_TRIANGLES )
	{
		return false;
	}

	meshPush( out, 0.0, -depth, 0.0 );

	for ( int r = 1; r <= rings; ++r )
	{
		double t = (double)r / rings;
		for ( int s = 0; s < segments; ++s )
		{
			double a = 2.0 * 3.14159265358979323846 * s / segments;
			meshPush( out, radius * t * cos( a ), -depth * ( 1.0 - t * t ), radius * t * sin( a ) );
		}
	}

	for ( int s = 0; s < segments; ++s )
	{
		meshTri( out, 0, 1 + s, 1 + ( s + 1 ) % segments );
	}

	for ( int r = 1; r < rings; ++r )
	{
		int inner = 1 + ( r - 1 ) * segments;
		int outer = 1 + r * segments;
		for ( int s = 0; s < segments; ++s )
		{
			int s2 = ( s + 1 ) % segments;
			meshTri( out, inner + s, outer + s, inner + s2 );
			meshTri( out, inner + s2, outer + s, outer + s2 );
		}
	}

	return true;
}

bool pdRefMakeMesh( pdMeshKind kind, const double* params, pdMesh* out )
{
	if ( s_meshCount >= PD_REF_MAX_MESHES )
	{
		return false;
	}

	memset( out, 0, sizeof( *out ) );

	bool built = false;
	switch ( kind )
	{
		case pd_meshGrid:
			built = genGrid( params, out );
			break;

		case pd_meshRamp:
			built = genRamp( params, out );
			break;

		case pd_meshStairs:
			built = genStairs( params, out );
			break;

		case pd_meshBowl:
			built = genBowl( params, out );
			break;

		default:
			return false;
	}

	if ( built == false || out->vertexCount < 3 || out->triangleCount < 1 )
	{
		return false;
	}

	out->lower = out->vertices[0];
	out->upper = out->vertices[0];
	for ( int i = 1; i < out->vertexCount; ++i )
	{
		out->lower.x = fmin( out->lower.x, out->vertices[i].x );
		out->lower.y = fmin( out->lower.y, out->vertices[i].y );
		out->lower.z = fmin( out->lower.z, out->vertices[i].z );
		out->upper.x = fmax( out->upper.x, out->vertices[i].x );
		out->upper.y = fmax( out->upper.y, out->vertices[i].y );
		out->upper.z = fmax( out->upper.z, out->vertices[i].z );
	}

	// Upstream builds from exactly the coordinates the description carries, so
	// any difference from the port is quantization and the split heuristic --
	// which is what the scenario is there to separate.
	static b3Vec3 vertices[PD_MAX_MESH_VERTICES];
	static int32_t indices[3 * PD_MAX_MESH_TRIANGLES];

	for ( int i = 0; i < out->vertexCount; ++i )
	{
		vertices[i] = toV( out->vertices[i] );
	}

	for ( int i = 0; i < 3 * out->triangleCount; ++i )
	{
		indices[i] = out->indices[i];
	}

	b3MeshDef def = { 0 };
	def.vertices = vertices;
	def.indices = indices;
	def.vertexCount = out->vertexCount;
	def.triangleCount = out->triangleCount;
	def.identifyEdges = true;

	// Welding on, with a tolerance below one Q12 quantum.
	//
	// The port's baker always merges vertices that land on the same lattice
	// point -- an unwelded seam is a pair of edges with no shared-edge partner,
	// which is precisely where ghost collisions come from, so it is not
	// optional there. Upstream makes it a modelling convenience and defaults it
	// off. Left off, the staircase's step seams pair up on one side and not the
	// other and 12% of edge flags "differ" for reasons that have nothing to do
	// with fixed point. Sub-quantum, so it merges only vertices that were
	// already identical.
	def.weldVertices = true;
	def.weldTolerance = 1e-4f;

	b3MeshData* mesh = b3CreateMesh( &def, NULL, 0 );
	if ( mesh == NULL )
	{
		return false;
	}

	out->refId = s_meshCount;
	s_meshes[s_meshCount] = mesh;
	s_meshCount++;
	return true;
}

static void refMeshAABB( const pdMesh* mesh, const pdTransform* xf, pdVec3 scale, pdAABB* out )
{
	b3Mesh m = refMesh( mesh, scale );
	b3AABB aabb = b3ComputeMeshAABB( m.data, toXf( xf ), m.scale );

	memset( out, 0, sizeof( *out ) );
	out->lower = fromV( aabb.lowerBound );
	out->upper = fromV( aabb.upperBound );
}

typedef struct
{
	int* indices;
	int capacity;
	int count;
} refQueryContext;

static bool refCollectTriangle( b3Vec3 a, b3Vec3 b, b3Vec3 c, int triangleIndex, void* context )
{
	(void)a;
	(void)b;
	(void)c;

	refQueryContext* ctx = context;
	if ( ctx->count < ctx->capacity )
	{
		ctx->indices[ctx->count] = triangleIndex;
	}
	ctx->count++;
	return true;
}

static int refMeshQuery( const pdMesh* mesh, pdVec3 lower, pdVec3 upper, pdVec3 scale, int* indices, int capacity )
{
	b3Mesh m = refMesh( mesh, scale );
	b3AABB bounds = { toV( lower ), toV( upper ) };

	refQueryContext ctx = { indices, capacity, 0 };
	b3QueryMesh( &m, bounds, refCollectTriangle, &ctx );
	return ctx.count;
}

static bool refMeshTriangle( const pdMesh* mesh, int triangleIndex, pdVec3 scale, pdVec3 out[3], int* flags )
{
	b3Mesh m = refMesh( mesh, scale );
	if ( triangleIndex < 0 || triangleIndex >= m.data->triangleCount )
	{
		return false;
	}

	b3Triangle tri = b3GetMeshTriangle( &m, triangleIndex );
	for ( int i = 0; i < 3; ++i )
	{
		out[i] = fromV( tri.vertices[i] );
	}
	*flags = tri.flags;
	return true;
}

static void refHullMass( const pdHull* hull, double density, pdMassOut* out )
{
	b3MassData md = b3ComputeHullMass( refHull( hull ), (float)density );

	memset( out, 0, sizeof( *out ) );
	out->mass = md.mass;
	out->center = fromV( md.center );

	// Upstream's tensor is absolute, so divide the mass back out to compare
	// against the port's convention.
	const float* inertia = &md.inertia.cx.x;
	for ( int i = 0; i < 9; ++i )
	{
		out->unitInertia[i] = md.mass != 0.0f ? inertia[i] / md.mass : 0.0;
	}
}

static void refHullAABB( const pdHull* hull, const pdTransform* xf, pdAABB* out )
{
	b3AABB aabb = b3ComputeHullAABB( refHull( hull ), toXf( xf ) );
	out->lower = fromV( aabb.lowerBound );
	out->upper = fromV( aabb.upperBound );
}

static void refHullRayCast( const pdHull* hull, pdVec3 origin, pdVec3 translation, pdCastOut* out )
{
	b3RayCastInput input;
	input.origin = toV( origin );
	input.translation = toV( translation );
	input.maxFraction = 1.0f;

	b3CastOutput o = b3RayCastHull( refHull( hull ), &input );

	memset( out, 0, sizeof( *out ) );
	out->hit = o.hit;
	out->fraction = o.fraction;
	out->point = fromV( o.point );
	out->normal = fromV( o.normal );
}

static void refHullShapeCast( const pdHull* hull, const pdProxy* b, pdVec3 translation, pdCastOut* out )
{
	b3Vec3 sb[PD_MAX_POINTS];

	b3ShapeCastInput input;
	input.proxy = toProxy( b, sb );
	input.translation = toV( translation );
	input.maxFraction = 1.0f;
	input.canEncroach = false;

	b3CastOutput o = b3ShapeCastHull( refHull( hull ), &input );

	memset( out, 0, sizeof( *out ) );
	out->hit = o.hit;
	out->fraction = o.fraction;
	out->point = fromV( o.point );
	out->normal = fromV( o.normal );
}

static bool refHullOverlap( const pdHull* hull, const pdTransform* xf, const pdProxy* b )
{
	b3Vec3 sb[PD_MAX_POINTS];
	b3ShapeProxy proxy = toProxy( b, sb );
	return b3OverlapHull( refHull( hull ), toXf( xf ), &proxy );
}

static void refHullSphere( const pdHull* hull, pdVec3 cB, double rB, const pdTransform* xf, pdManifoldOut* out )
{
	b3Sphere sb = { toV( cB ), (float)rB };

	b3LocalManifoldPoint buffer[PD_MAX_MANIFOLD_POINTS] = { 0 };
	b3LocalManifold m = { 0 };
	m.points = buffer;

	b3SimplexCache cache = { 0 };
	b3CollideHullAndSphere( &m, PD_MAX_MANIFOLD_POINTS, refHull( hull ), &sb, toXf( xf ), &cache );
	grabManifold( &m, out );
}

static void refHullCapsule( const pdHull* hull, pdVec3 b1, pdVec3 b2, double rB, const pdTransform* xf,
							pdManifoldOut* out )
{
	b3Capsule cb = { toV( b1 ), toV( b2 ), (float)rB };

	b3LocalManifoldPoint buffer[PD_MAX_MANIFOLD_POINTS] = { 0 };
	b3LocalManifold m = { 0 };
	m.points = buffer;

	b3SimplexCache cache = { 0 };
	b3CollideHullAndCapsule( &m, PD_MAX_MANIFOLD_POINTS, refHull( hull ), &cb, toXf( xf ), &cache );
	grabManifold( &m, out );
}

// Capacity 4, not PD_MAX_MANIFOLD_POINTS: b3CollideHulls never writes more
// than four, and passing eight would hide a capacity bug on either side.
#define PD_HULL_HULL_CAPACITY 4

static void grabFeature( const b3SATCache* cache, pdManifoldOut* out )
{
	out->feature = (int)cache->type;
	out->featureIndexA = (int)cache->indexA;
	out->featureIndexB = (int)cache->indexB;
}

static void refHullHull( const pdHull* a, const pdHull* b, const pdTransform* xf, pdManifoldOut* out )
{
	b3LocalManifoldPoint buffer[PD_MAX_MANIFOLD_POINTS] = { 0 };
	b3LocalManifold m = { 0 };
	m.points = buffer;

	b3SATCache cache = { 0 };
	b3CollideHulls( &m, PD_HULL_HULL_CAPACITY, refHull( a ), refHull( b ), toXf( xf ), &cache );
	grabManifold( &m, out );
	grabFeature( &cache, out );
}

static void refHullHullCached( const pdHull* a, const pdHull* b, const pdTransform* xf1, const pdTransform* xf2,
							   pdManifoldOut* out1, pdManifoldOut* out2, bool* hit1, bool* hit2 )
{
	b3LocalManifoldPoint buffer[PD_MAX_MANIFOLD_POINTS] = { 0 };
	b3LocalManifold m = { 0 };
	m.points = buffer;

	// One cache carried across both calls -- that is the whole point.
	b3SATCache cache = { 0 };

	b3CollideHulls( &m, PD_HULL_HULL_CAPACITY, refHull( a ), refHull( b ), toXf( xf1 ), &cache );
	grabManifold( &m, out1 );
	grabFeature( &cache, out1 );
	*hit1 = cache.hit != 0;

	b3CollideHulls( &m, PD_HULL_HULL_CAPACITY, refHull( a ), refHull( b ), toXf( xf2 ), &cache );
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

static float keepAll( const b3RayCastInput* input, int proxyId, uint64_t userData, void* context )
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
		b3AABB box = { toV( boxes[i].lower ), toV( boxes[i].upper ) };
		b3DynamicTree_CreateProxy( &tree, box, B3_DEFAULT_CATEGORY_BITS, (uint64_t)i );
	}
	return tree;
}

static void refTreeQuery( const pdAABB* boxes, int n, pdAABB query, pdTreeOut* out )
{
	b3DynamicTree tree = buildTree( boxes, n );

	memset( out, 0, sizeof( *out ) );
	b3AABB q = { toV( query.lower ), toV( query.upper ) };
	b3DynamicTree_Query( &tree, q, B3_DEFAULT_CATEGORY_BITS, false, collect, out );

	b3DynamicTree_Destroy( &tree );
}

static void refTreeRayCast( const pdAABB* boxes, int n, pdVec3 origin, pdVec3 translation, pdTreeOut* out )
{
	b3DynamicTree tree = buildTree( boxes, n );

	b3RayCastInput input;
	input.origin = toV( origin );
	input.translation = toV( translation );
	input.maxFraction = 1.0f;

	memset( out, 0, sizeof( *out ) );
	b3DynamicTree_RayCast( &tree, &input, B3_DEFAULT_CATEGORY_BITS, false, keepAll, out );

	b3DynamicTree_Destroy( &tree );
}

// --- scripted scenes ------------------------------------------------------

static int refSceneShapeIndex( const b3ShapeId* ids, int count, int rawShapeId )
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

static int refCompareSceneContacts( const void* a, const void* b )
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

/// Sensor transitions in a canonical order, so the comparison is of the *set*
/// each library produced rather than of its publishing order.
static int refCompareSensorEvents( const void* a, const void* b )
{
	const pdSensorEvent* ea = (const pdSensorEvent*)a;
	const pdSensorEvent* eb = (const pdSensorEvent*)b;
	if ( ea->sensorShape != eb->sensorShape )
	{
		return ea->sensorShape < eb->sensorShape ? -1 : 1;
	}
	if ( ea->visitorShape != eb->visitorShape )
	{
		return ea->visitorShape < eb->visitorShape ? -1 : 1;
	}
	return 0;
}

/// Append this step's sensor transitions to the pass.
///
/// Called after *every* step of a pass rather than once at the end, and that is
/// not a detail: both libraries clear the begin-event array at the top of each
/// step, so a twenty-step pass read once would report only whatever happened on
/// the twentieth -- which for a visitor crossing a trigger volume is nothing at
/// all. The first version of the sensors scenario did exactly that and showed
/// occupancy changing with zero events to explain it.
///
/// The contact counts alongside keep the read-once behaviour every scenario
/// before this one was written against; only the sensor arrays accumulate.
static void refCollectSensorEvents( b3WorldId worldId, const b3ShapeId* shapeIds, int shapeCount, pdScenePassOut* po )
{
	b3SensorEvents events = b3World_GetSensorEvents( worldId );

	for ( int i = 0; i < events.beginCount && po->sensorBeginCount < PD_MAX_SCENE_SENSOR_EVENTS; ++i )
	{
		pdSensorEvent* se = po->sensorBegins + po->sensorBeginCount;
		po->sensorBeginCount += 1;
		se->sensorShape = refSceneShapeIndex( shapeIds, shapeCount, events.beginEvents[i].sensorShapeId.index1 - 1 );
		se->visitorShape = refSceneShapeIndex( shapeIds, shapeCount, events.beginEvents[i].visitorShapeId.index1 - 1 );
	}

	for ( int i = 0; i < events.endCount && po->sensorEndCount < PD_MAX_SCENE_SENSOR_EVENTS; ++i )
	{
		pdSensorEvent* se = po->sensorEnds + po->sensorEndCount;
		po->sensorEndCount += 1;
		se->sensorShape = refSceneShapeIndex( shapeIds, shapeCount, events.endEvents[i].sensorShapeId.index1 - 1 );
		se->visitorShape = refSceneShapeIndex( shapeIds, shapeCount, events.endEvents[i].visitorShapeId.index1 - 1 );
	}
}

static void refWorldScene( const pdSceneDesc* desc, const pdHull* hulls, int hullCount, const pdMesh* meshes, int meshCount,
						   pdSceneOut* out )
{
	memset( out, 0, sizeof( *out ) );

	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = toV( desc->gravity );
	b3WorldId worldId = b3CreateWorld( &worldDef );
	b3World* world = b3GetWorldFromId( worldId );

	// Must resolve to the same number portWorldScene resolves it to: the
	// sub-step count changes the answer on both sides.
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
		bodyDef.rotation.v = B3_LITERAL( b3Vec3 ){ (float)sb->body.xf.qx, (float)sb->body.xf.qy, (float)sb->body.xf.qz };
		bodyDef.rotation.s = (float)sb->body.xf.qw;
		bodyDef.enableSleep = sb->disableSleep == false;
		bodyIds[b] = b3CreateBody( worldId, &bodyDef );

		for ( int i = 0; i < sb->body.shapeCount && i < PD_MAX_BODY_SHAPES; ++i )
		{
			const pdBodyShape* s = sb->body.shapes + i;

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			shapeDef.density = (float)s->density;
			shapeDef.enableContactEvents = true;
			shapeDef.isSensor = s->isSensor;
			shapeDef.enableSensorEvents = s->enableSensorEvents;

			switch ( s->kind )
			{
				case pd_bodyShapeSphere:
				{
					b3Sphere sphere = { toV( s->p1 ), (float)s->radius };
					shapeIds[shapeCount++] = b3CreateSphereShape( bodyIds[b], &shapeDef, &sphere );
				}
				break;

				case pd_bodyShapeCapsule:
				{
					b3Capsule capsule = { toV( s->p1 ), toV( s->p2 ), (float)s->radius };
					shapeIds[shapeCount++] = b3CreateCapsuleShape( bodyIds[b], &shapeDef, &capsule );
				}
				break;

				case pd_bodyShapeHull:
				{
					if ( s->hullIndex < hullCount )
					{
						shapeIds[shapeCount++] = b3CreateHullShape( bodyIds[b], &shapeDef, refHull( hulls + s->hullIndex ) );
					}
				}
				break;

				case pd_bodyShapeMesh:
				{
					if ( s->meshIndex < meshCount )
					{
						b3Mesh m = refMesh( meshes + s->meshIndex, B3_LITERAL( pdVec3 ){ 1.0, 1.0, 1.0 } );
						shapeIds[shapeCount++] = b3CreateMeshShape( bodyIds[b], &shapeDef, m.data, m.scale );
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

			// Degrees to radians. This is the one field where the two
			// descriptions genuinely differ -- the port takes brads -- so the
			// conversion lives on each side rather than in the scene.
			const double toRad = 3.14159265358979323846 / 180.0;
			rev.lowerAngle = (float)( sj->lowerAngleDeg * toRad );
			rev.upperAngle = (float)( sj->upperAngleDeg * toRad );
			rev.targetAngle = (float)( sj->targetAngleDeg * toRad );

			rev.enableLimit = sj->enableAngleLimit;
			rev.enableMotor = sj->enableAngleMotor;
			rev.enableSpring = sj->enableAngleSpring;
			rev.hertz = (float)sj->angleHertz;
			rev.dampingRatio = (float)sj->angleDampingRatio;
			rev.motorSpeed = (float)sj->motorAngularSpeed;
			rev.maxMotorTorque = (float)sj->maxMotorTorque;

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

			const double toRad = 3.14159265358979323846 / 180.0;
			ball.coneAngle = (float)( sj->coneAngleDeg * toRad );
			ball.lowerTwistAngle = (float)( sj->lowerTwistDeg * toRad );
			ball.upperTwistAngle = (float)( sj->upperTwistDeg * toRad );

			ball.enableConeLimit = sj->enableConeLimit;
			ball.enableTwistLimit = sj->enableTwistLimit;
			ball.enableSpring = sj->enableBallSpring;
			ball.hertz = (float)sj->ballHertz;
			ball.dampingRatio = (float)sj->ballDampingRatio;
			ball.enableMotor = sj->enableBallMotor;
			ball.motorVelocity = toV( sj->ballMotorVelocity );
			ball.maxMotorTorque = (float)sj->ballMaxMotorTorque;

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

			weld.linearHertz = (float)sj->weldLinearHertz;
			weld.linearDampingRatio = (float)sj->weldLinearDampingRatio;
			weld.angularHertz = (float)sj->weldAngularHertz;
			weld.angularDampingRatio = (float)sj->weldAngularDampingRatio;

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
			motor.maxVelocityForce = (float)sj->motorMaxVelocityForce;
			motor.maxVelocityTorque = (float)sj->motorMaxVelocityTorque;
			motor.linearHertz = (float)sj->motorLinearHertz;
			motor.linearDampingRatio = (float)sj->motorLinearDampingRatio;
			motor.angularHertz = (float)sj->motorAngularHertz;
			motor.angularDampingRatio = (float)sj->motorAngularDampingRatio;
			motor.maxSpringForce = (float)sj->motorMaxSpringForce;
			motor.maxSpringTorque = (float)sj->motorMaxSpringTorque;

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

			slide.targetTranslation = (float)sj->slideTargetTranslation;
			slide.lowerTranslation = (float)sj->slideLowerTranslation;
			slide.upperTranslation = (float)sj->slideUpperTranslation;
			slide.hertz = (float)sj->slideHertz;
			slide.dampingRatio = (float)sj->slideDampingRatio;
			slide.motorSpeed = (float)sj->slideMotorSpeed;
			slide.maxMotorForce = (float)sj->slideMaxMotorForce;
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

			parallel.hertz = (float)sj->parallelHertz;
			parallel.dampingRatio = (float)sj->parallelDampingRatio;

			// Upstream defaults maxTorque to FLT_MAX and the port to zero, so
			// this is always written explicitly on both sides -- a scene that
			// left it at the default would be comparing an unbounded joint
			// against an inert one.
			parallel.maxTorque = (float)sj->parallelMaxTorque;

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

			wheel.suspensionHertz = (float)sj->wheelSuspensionHertz;
			wheel.suspensionDampingRatio = (float)sj->wheelSuspensionDampingRatio;
			wheel.lowerSuspensionLimit = (float)sj->wheelLowerSuspensionLimit;
			wheel.upperSuspensionLimit = (float)sj->wheelUpperSuspensionLimit;
			wheel.spinSpeed = (float)sj->wheelSpinSpeed;
			wheel.maxSpinTorque = (float)sj->wheelMaxSpinTorque;
			wheel.steeringHertz = (float)sj->wheelSteeringHertz;
			wheel.steeringDampingRatio = (float)sj->wheelSteeringDampingRatio;
			wheel.maxSteeringTorque = (float)sj->wheelMaxSteeringTorque;

			// Degrees in, radians out.
			const double toRad = 3.14159265358979323846 / 180.0;
			wheel.targetSteeringAngle = (float)( sj->wheelTargetSteeringDeg * toRad );
			wheel.lowerSteeringLimit = (float)( sj->wheelLowerSteeringDeg * toRad );
			wheel.upperSteeringLimit = (float)( sj->wheelUpperSteeringDeg * toRad );

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

		jointDef.length = (float)sj->length;
		jointDef.enableSpring = sj->enableSpring;
		jointDef.hertz = (float)sj->hertz;
		jointDef.dampingRatio = (float)sj->dampingRatio;
		jointDef.enableLimit = sj->enableLimit;
		jointDef.minLength = (float)sj->minLength;
		jointDef.maxLength = (float)sj->maxLength;
		jointDef.enableMotor = sj->enableMotor;
		jointDef.maxMotorForce = (float)sj->maxMotorForce;
		jointDef.motorSpeed = (float)sj->motorSpeed;

		// Zero on both means "leave the default", which upstream spells
		// -FLT_MAX / +FLT_MAX and the port spells with a large finite
		// sentinel. Neither is reachable, so the two agree in effect.
		if ( sj->lowerSpringForce != 0.0 || sj->upperSpringForce != 0.0 )
		{
			jointDef.lowerSpringForce = (float)sj->lowerSpringForce;
			jointDef.upperSpringForce = (float)sj->upperSpringForce;
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

		// b3Collide is static upstream, so a collide-only pass is reached
		// through b3World_Step with a zero time step. Upstream handles that
		// case explicitly -- inv_dt, h and inv_h are all set to zero -- so this
		// runs the broad phase, the narrow phase and the contact-state pass and
		// integrates nothing. Such a scene also zeroes gravity, so the solver
		// that also runs has nothing to act on.
		//
		// A stepping pass uses 1/60 explicitly. The port has no time-step
		// argument at all -- dt is B3_NEA_STEP_HZ, fixed at compile time -- so
		// this literal is the one place the two sides' notion of a step has to
		// be kept in agreement by hand.
		pdScenePassOut* po = out->passes + p;

		if ( pass->stepCount <= 0 )
		{
			b3World_Step( worldId, 0.0f, 1 );
			refCollectSensorEvents( worldId, shapeIds, shapeCount, po );
		}
		else
		{
			for ( int s = 0; s < pass->stepCount; ++s )
			{
				b3World_Step( worldId, 1.0f / 60.0f, subStepCount );
				refCollectSensorEvents( worldId, shapeIds, shapeCount, po );
			}
		}

		for ( int i = 0; i < world->contacts.count && po->contactCount < PD_MAX_SCENE_CONTACTS; ++i )
		{
			b3Contact* contact = world->contacts.data + i;
			if ( contact->contactId == B3_NULL_INDEX )
			{
				continue;
			}

			pdSceneContact* sc = po->contacts + po->contactCount;
			po->contactCount += 1;

			sc->shapeA = refSceneShapeIndex( shapeIds, shapeCount, contact->shapeIdA );
			sc->shapeB = refSceneShapeIndex( shapeIds, shapeCount, contact->shapeIdB );
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
					sc->manifolds[m].separations[k] = manifold->points[k].separation;
					sc->manifolds[m].featureIds[k] = manifold->points[k].featureId;
					sc->manifolds[m].triangleIndices[k] = manifold->points[k].triangleIndex;
				}
			}
		}

		qsort( po->contacts, (size_t)po->contactCount, sizeof( pdSceneContact ), refCompareSceneContacts );

		b3ContactEvents events = b3World_GetContactEvents( worldId );
		po->beginCount = events.beginCount;
		po->endCount = events.endCount;

		qsort( po->sensorBegins, (size_t)po->sensorBeginCount, sizeof( pdSensorEvent ), refCompareSensorEvents );
		qsort( po->sensorEnds, (size_t)po->sensorEndCount, sizeof( pdSensorEvent ), refCompareSensorEvents );

		for ( int i = 0; i < shapeCount; ++i )
		{
			po->sensorOccupancy[i] = b3Shape_IsSensor( shapeIds[i] ) ? b3Shape_GetSensorCapacity( shapeIds[i] ) : -1;
		}

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
				jo->angleDeg = b3RevoluteJoint_GetAngle( jointIds[j] ) * ( 180.0 / 3.14159265358979323846 );
			}
			else if ( desc->joints[j].kind == pd_jointSpherical )
			{
				const double toDeg = 180.0 / 3.14159265358979323846;
				jo->coneAngleDeg = b3SphericalJoint_GetConeAngle( jointIds[j] ) * toDeg;
				jo->twistAngleDeg = b3SphericalJoint_GetTwistAngle( jointIds[j] ) * toDeg;
			}
			else if ( desc->joints[j].kind == pd_jointPrismatic )
			{
				jo->translation = b3PrismaticJoint_GetTranslation( jointIds[j] );
				jo->slideSpeed = b3PrismaticJoint_GetSpeed( jointIds[j] );
			}
			else if ( desc->joints[j].kind == pd_jointWheel )
			{
				const double toDeg = 180.0 / 3.14159265358979323846;

				// Upstream has no suspension-translation query at all -- the
				// port added one. The same quantity is recovered here from the
				// joint frames, which is what the port's accessor computes.
				b3WorldTransform xfA = b3Body_GetTransform( bodyIds[desc->joints[j].bodyA] );
				b3WorldTransform xfB = b3Body_GetTransform( bodyIds[desc->joints[j].bodyB] );
				b3Vec3 rA = b3RotateVector( xfA.q, toV( desc->joints[j].localAnchorA ) );
				b3Vec3 rB = b3RotateVector( xfB.q, toV( desc->joints[j].localAnchorB ) );
				b3Vec3 d = b3Add( b3SubPos( xfB.p, xfA.p ), b3Sub( rB, rA ) );
				jo->suspensionTranslation = b3Dot( d, b3RotateVector( xfA.q, b3Vec3_axisX ) );

				jo->spinSpeed = b3WheelJoint_GetSpinSpeed( jointIds[j] );
				jo->steeringAngleDeg = b3WheelJoint_GetSteeringAngle( jointIds[j] ) * toDeg;
			}
			else if ( desc->joints[j].kind == pd_jointWeld || desc->joints[j].kind == pd_jointMotor ||
					  desc->joints[j].kind == pd_jointParallel )
			{
				// None has an angle or a length of its own -- see the port
				// side's note. Force, torque and the body trajectories are the
				// comparison.
			}
			else
			{
				jo->currentLength = b3DistanceJoint_GetCurrentLength( jointIds[j] );
			}
		}
	}

	b3DestroyWorld( worldId );
}

const pdBackend pdRefBackend = {
	.name = "float",
	.distance = refDistance,
	.shapeCast = refShapeCast,
	.timeOfImpact = refTimeOfImpact,
	.sphereSphere = refSphereSphere,
	.capsuleSphere = refCapsuleSphere,
	.capsuleCapsule = refCapsuleCapsule,
	.treeQuery = refTreeQuery,
	.treeRayCast = refTreeRayCast,
	.hullMass = refHullMass,
	.hullAABB = refHullAABB,
	.hullRayCast = refHullRayCast,
	.hullShapeCast = refHullShapeCast,
	.hullOverlap = refHullOverlap,
	.hullSphere = refHullSphere,
	.hullCapsule = refHullCapsule,
	.hullHull = refHullHull,
	.hullHullCached = refHullHullCached,
	.triangleSphere = refTriangleSphere,
	.triangleCapsule = refTriangleCapsule,
	.triangleHull = refTriangleHull,

	.meshAABB = refMeshAABB,
	.meshQuery = refMeshQuery,
	.meshTriangle = refMeshTriangle,

	.worldBody = refWorldBody,
	.worldScene = refWorldScene,
};
