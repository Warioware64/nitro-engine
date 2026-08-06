// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Erin Catto        (original Box3D)
// Copyright (c) 2026 Warioware64       (Nitro Engine Advanced fixed-point port)
//
// This file is part of Nitro Engine Advanced

#pragma once

/// @file   types.h
/// @brief  Public data types: shapes, queries, manifolds, the dynamic tree.
///
/// @section scope What is here so far
///
/// Upstream's types.h is one 3000-line header covering everything from
/// b3Sphere to b3WheelJointDef. This is the *collision tier* of it -- the
/// types the broad phase, the shape primitives and the distance queries need
/// -- plus, since Phase 3A, the world/body/shape definitions and the event
/// structs, and since Phase 6 the joint type enum and base definition. The
/// per-type joint definitions land one per stage with their constraint math.
///
/// @section scales Choosing a scale per field
///
/// Every `float` upstream becomes one of the fixed-point types from
/// b3fixed.h, chosen by what the field measures rather than by rote:
///
///   b3f    lengths, radii, positions, separations, distances, masses
///   b3c    dimensionless values bounded by 1 -- ray fractions, barycentric
///          coordinates, friction and restitution coefficients
///   b3imp  impulses, which accumulate across substeps
///   b3a    angles
///
/// Where a quantity does not fit any scale cleanly, the field carries a note
/// saying so. There are two such cases in this file: b3SimplexCache::metric
/// and the inertia tensor in b3MassData.

#include "base.h"
#include "constants.h"
#include "id.h"
#include "math_fixed.h"

#include <stdbool.h>
#include <stdint.h>

// =========================================================================
// Query filtering
// =========================================================================

/// The query filter is used to filter collisions between queries and shapes.
typedef struct b3QueryFilter
{
	/// The collision category bits of this query. Normally you would just set one bit.
	uint64_t categoryBits;

	/// The collision mask bits. This states the shape categories that this
	/// query would accept for collision.
	uint64_t maskBits;
} b3QueryFilter;

/// Use this to initialize your query filter.
B3_API b3QueryFilter b3DefaultQueryFilter( void );

// =========================================================================
// World query results and callbacks
// =========================================================================
//
// Phase 7. Upstream gives every one of these a `b3Pos origin` argument and
// re-differences each shape against it, so a query stays accurate far from the
// world origin in float. This port deleted that path in Phase 1 -- b3Pos is
// b3Vec3 and b3WorldTransform is b3Transform -- so `origin` would be a
// parameter that is added and then subtracted again. It is **dropped from
// every signature**, the same call b3World_Step made when it lost `timeStep`.
// Fixed point's accuracy far from the origin is a question of Q12 range, which
// constants.h bounds at +/-2000 units for the whole engine, and no per-query
// argument changes that.

/// Called once per shape an overlap query finds.
/// @return false to terminate the query.
typedef bool b3OverlapResultFcn( b3ShapeId shapeId, void* context );

/// Called once per shape a ray or shape cast hits, in no particular order.
///
/// The return value steers the rest of the cast:
///   - **-1** (or anything negative): ignore this shape and keep going
///   - **0**: stop the cast here
///   - **a fraction in (0, 1)**: clip the cast to this point, which is what
///     finds the closest hit
///   - **1**: keep the full length and continue
///
/// @param shapeId        the shape that was hit
/// @param point          the world point of intersection
/// @param normal         the surface normal there
/// @param fraction       how far along the cast the hit is
/// @param userMaterialId the surface material at the hit
/// @param triangleIndex  the triangle for a mesh shape, B3_NULL_INDEX otherwise
/// @param childIndex     the child for a compound shape; always 0 in this port
typedef b3c b3CastResultFcn( b3ShapeId shapeId, b3Vec3 point, b3Vec3 normal, b3c fraction, uint64_t userMaterialId,
							 int triangleIndex, int childIndex, void* context );

/// The single nearest hit, from b3World_CastRayClosest.
typedef struct b3RayResult
{
	/// The shape hit. Only meaningful when `hit` is true.
	b3ShapeId shapeId;

	/// The world point of the hit.
	b3Vec3 point;

	/// The world surface normal there.
	b3Vec3 normal;

	/// The surface material at the hit point. Per triangle for a mesh, if the
	/// blob carries per-triangle materials.
	uint64_t userMaterialId;

	/// How far along the input ray the hit is.
	b3c fraction;

	/// The triangle index for a mesh shape, B3_NULL_INDEX for the others.
	int triangleIndex;

	/// The child index for a compound shape. Always 0 -- the port has none.
	int childIndex;

	/// BVH internal nodes visited. Diagnostic; useful for sizing a level.
	int nodeVisits;

	/// BVH leaves visited. Diagnostic.
	int leafVisits;

	/// Did the ray hit anything at all?
	bool hit;
} b3RayResult;

// =========================================================================
// Character mover
// =========================================================================
//
// Phase 7 Stage 4. A mover is a capsule that is **not** a body: it is moved by
// depenetration and shape casting rather than by the solver, which is what
// keeps a player from sliding down ramps, tipping over and picking up spin.
// The types below are the whole of the shared vocabulary; the controller loop
// that drives them is the caller's, or NEA_Phys3DMover's.
//
// The one thing to understand before reading the rest: **a plane's `offset` is
// a penetration depth, not a `dot( normal, point )`**. Every b3CollideMoverAnd*
// builds it as `totalRadius - distance`, measured against the mover where it is
// *now*. So the plane set is expressed relative to the mover's current
// position, which is why b3SolvePlanes hands b3PlaneSeparation a *translation*
// rather than a point, and why b3CollideMover rotates a plane's normal and
// transforms its point but deliberately leaves its offset alone. It is also
// what makes the whole subsystem origin-independent, and therefore what lets
// this port drop `b3Pos origin` from the two world entry points the way it did
// from every other query.

/// A collision plane between a character mover and one shape.
typedef struct b3PlaneResult
{
	/// Outward pointing plane. `offset` is a depth -- see the note above.
	b3Plane plane;

	/// The closest point on the shape. May not be unique.
	b3Vec3 point;
} b3PlaneResult;

/// A plane fed to b3SolvePlanes. Normally assembled by the caller from the
/// b3PlaneResult batches b3World_CollideMover hands it, which is where the
/// per-shape softness a game wants gets applied.
typedef struct b3CollisionPlane
{
	/// The collision plane between the mover and some shape.
	b3Plane plane;

	/// How far this plane may push in total. B3_F_MAX makes it as rigid as
	/// possible; lower values make the collision soft.
	///
	/// B3_F_MAX rather than B3_HUGE, and the choice is load bearing: b3AddF is
	/// a bare int32 add in device mode, and B3_F_MAX is INT32_MAX/2 precisely
	/// so that an add on the sentinel cannot wrap. B3_HUGE would work
	/// arithmetically but is a sanity bound on *real lengths*, so using it here
	/// would make a plane that genuinely needs a 2001-unit push behave
	/// differently from a rigid one, for a reason the caller cannot see.
	b3f pushLimit;

	/// The push b3SolvePlanes decided on. Written by the solver, read by
	/// b3ClipVector, and zero means this plane did nothing.
	b3f push;

	/// Should b3ClipVector clip against this plane? False for soft collision.
	bool clipVelocity;
} b3CollisionPlane;

/// What b3SolvePlanes decided.
typedef struct b3PlaneSolverResult
{
	/// The final relative translation.
	b3Vec3 delta;

	/// Iterations the solver used, out of a cap of 20.
	///
	/// Reaching the cap is ordinary, not a failure: measured over 160,000
	/// random plane sets it happens 52% of the time, and the same scenarios in
	/// double precision reach it 52% of the time too. See the note in mover.c.
	int iterationCount;
} b3PlaneSolverResult;

/// Called once per shape b3World_CollideMover finds, with that shape's whole
/// batch of planes.
/// @return false to stop gathering.
typedef bool b3PlaneResultFcn( b3ShapeId shapeId, const b3PlaneResult* planes, int planeCount, void* context );

/// Called once per shape b3World_CastMover is about to sweep against.
/// @return true to accept the shape.
typedef bool b3MoverFilterFcn( b3ShapeId shapeId, void* context );

// =========================================================================
// Ray and shape casts
// =========================================================================

/// Low level ray cast input data.
typedef struct b3RayCastInput
{
	/// Start point of the ray cast.
	b3Vec3 origin;

	/// Translation of the ray cast. end = start + translation.
	b3Vec3 translation;

	/// The maximum fraction of the translation to consider, typically 1.
	b3c maxFraction;
} b3RayCastInput;

/// A generic point cloud with a radius, used as a cast or distance query
/// shape. A sphere is one point with a radius; a capsule is two points with a
/// radius; a box is eight points with zero radius.
typedef struct b3ShapeProxy
{
	/// The point cloud.
	const b3Vec3* points;

	/// The number of points. Do not exceed B3_MAX_SHAPE_CAST_POINTS.
	int count;

	/// The external radius of the point cloud.
	b3f radius;
} b3ShapeProxy;

/// Low level shape cast input in generic form.
typedef struct b3ShapeCastInput
{
	/// A generic query shape.
	b3ShapeProxy proxy;

	/// The translation of the shape cast.
	b3Vec3 translation;

	/// The maximum fraction of the translation to consider, typically 1.
	b3c maxFraction;

	/// Allow the cast to encroach when initially touching. Only meaningful
	/// when the radius is greater than zero.
	bool canEncroach;
} b3ShapeCastInput;

/// Input for sweeping an AABB through a dynamic tree.
typedef struct b3BoxCastInput
{
	/// The AABB to cast, in the tree's frame.
	b3AABB box;

	/// The sweep translation.
	b3Vec3 translation;

	/// The maximum fraction of the translation to consider, typically 1.
	b3c maxFraction;
} b3BoxCastInput;

/// Low level ray cast or shape-cast output data.
typedef struct b3CastOutput
{
	/// The surface normal at the hit point.
	b3Vec3 normal;

	/// The surface hit point.
	b3Vec3 point;

	/// The fraction of the input translation at collision.
	b3c fraction;

	/// The number of iterations used.
	int iterations;

	/// The index of the mesh or height field triangle hit.
	int triangleIndex;

	/// The index of the compound child shape.
	int childIndex;

	/// The material index. May be -1 for null.
	int materialIndex;

	/// Did the cast hit?
	bool hit;
} b3CastOutput;

// =========================================================================
// Distance queries
// =========================================================================

/// Cached simplex from a previous distance query, used to warm start the
/// next one.
typedef struct b3SimplexCache
{
	/// Scale-invariant measure of the cached simplex, used only to decide
	/// whether the cache still describes the same configuration.
	///
	/// This is the one field in the file with no honest scale. Upstream
	/// stores a length, an area or a volume here depending on how many
	/// points the simplex has -- so the same field spans three different
	/// dimensions, and a volume in Q12 overflows past about 128 units.
	///
	/// It is never used as a length: every read compares it against another
	/// metric from the same query. So it is stored wide and unscaled, which
	/// keeps the comparison exact and sidesteps the dimensional problem
	/// entirely.
	int64_t metric;

	/// The number of stored simplex points.
	uint16_t count;

	/// The cached simplex indices on shape A.
	uint8_t indexA[4];

	/// The cached simplex indices on shape B.
	uint8_t indexB[4];
} b3SimplexCache;

/// Input for b3ShapeDistance.
typedef struct b3DistanceInput
{
	/// The proxy for shape A.
	b3ShapeProxy proxyA;

	/// The proxy for shape B.
	b3ShapeProxy proxyB;

	/// Transform of shape B in shape A's frame. The query is origin
	/// independent and runs in frame A, which also keeps the coordinates
	/// small -- worth more in fixed point than it is in float.
	b3Transform transform;

	/// Should the proxy radius be considered?
	bool useRadii;
} b3DistanceInput;

/// Output for b3ShapeDistance.
typedef struct b3DistanceOutput
{
	b3Vec3 pointA;	  ///< Closest point on shape A, in shape A's frame.
	b3Vec3 pointB;	  ///< Closest point on shape B, in shape A's frame.
	b3Vec3 normal;	  ///< A to B normal in shape A's frame. Invalid if the distance is zero.
	b3f distance;	  ///< The final distance, zero if overlapped.
	int iterations;	  ///< Number of GJK iterations used.
	int simplexCount; ///< The number of simplexes stored in the simplex array.
} b3DistanceOutput;

/// Input for b3ShapeCast.
typedef struct b3ShapeCastPairInput
{
	b3ShapeProxy proxyA;   ///< The proxy for shape A.
	b3ShapeProxy proxyB;   ///< The proxy for shape B.
	b3Transform transform; ///< Transform of shape B in shape A's frame.
	b3Vec3 translationB;   ///< The translation of shape B, in A's frame.
	b3c maxFraction;	   ///< The fraction of the translation to consider, typically 1.
	bool canEncroach;	   ///< Allow shapes with a radius to move closer if already touching.
} b3ShapeCastPairInput;

/// Simplex vertex, for debugging the GJK algorithm.
typedef struct b3SimplexVertex
{
	b3Vec3 wA;	///< Support point in proxy A.
	b3Vec3 wB;	///< Support point in proxy B.
	b3Vec3 w;	///< wB - wA.
	b3c a;		///< Barycentric coordinate, in [0, 1].
	int indexA; ///< wA index.
	int indexB; ///< wB index.
} b3SimplexVertex;

/// Simplex from the GJK algorithm.
typedef struct b3Simplex
{
	b3SimplexVertex vertices[4]; ///< Vertices.
	int count;					 ///< Number of valid vertices.
} b3Simplex;

/// Describes the motion of a body for time-of-impact computation.
typedef struct b3Sweep
{
	b3Vec3 localCenter; ///< Local center of mass position.
	b3Vec3 c1;			///< Starting center of mass world position.
	b3Vec3 c2;			///< Ending center of mass world position.
	b3Quat q1;			///< Starting world rotation.
	b3Quat q2;			///< Ending world rotation.
} b3Sweep;

/// Input for b3TimeOfImpact.
typedef struct b3TOIInput
{
	b3ShapeProxy proxyA; ///< The proxy for shape A.
	b3ShapeProxy proxyB; ///< The proxy for shape B.
	b3Sweep sweepA;		 ///< The motion of shape A.
	b3Sweep sweepB;		 ///< The motion of shape B.
	b3c maxFraction;	 ///< The sweep interval is [0, maxFraction].
} b3TOIInput;

/// How a time-of-impact query ended.
typedef enum b3TOIState
{
	/// Never returned. The output is initialized to this so the exit asserts
	/// can tell "no branch set the state" from any real answer.
	b3_toiStateUnknown,

	/// The root finder ran out of iterations. The fraction is the last one
	/// known separated, so acting on it is conservative rather than wrong.
	b3_toiStateFailed,

	/// The shapes were already touching at the start of the sweep. Continuous
	/// collision has nothing to contribute; the fraction is zero.
	b3_toiStateOverlapped,

	/// The shapes touch during the sweep, at `fraction`.
	b3_toiStateHit,

	/// The shapes stay apart for the whole sweep. The fraction is maxFraction.
	b3_toiStateSeparated
} b3TOIState;

/// Output for b3TimeOfImpact.
typedef struct b3TOIOutput
{
	/// How the query ended. Read this before the fraction: the fraction means
	/// something different in each state.
	b3TOIState state;

	b3Vec3 point;  ///< The hit point, in world space.
	b3Vec3 normal; ///< The hit normal, pointing from A to B.
	b3c fraction;  ///< The sweep fraction of the collision.
	b3f distance;  ///< The final distance between the shapes.

	/// Iteration counts, so the caps can be set from measurement rather than
	/// from upstream's float-era guesses. The port is deterministic integer
	/// code, so these read the same on host and on hardware.
	int distanceIterations; ///< Outer (separating axis) iterations.
	int pushBackIterations; ///< Total deepest-point resolutions.
	int rootIterations;		///< Total false-position/bisection steps.

	/// The query found initial overlap and fell back to a sphere around the
	/// fast shape's centroid, as a last effort to prevent tunnelling.
	bool usedFallback;
} b3TOIOutput;

// =========================================================================
// Mass properties
// =========================================================================

typedef struct b3MassData
{
	/// The shape mass.
	b3f mass;

	/// The local center of mass position.
	b3Vec3 center;

	/// The inertia tensor about the shape center of mass.
	///
	/// Inertia scales as mass times length squared, so this overflows Q12
	/// sooner than anything else here: a 10 kg body with a 10 unit extent
	/// already reaches 1000. Shapes that large are outside the documented
	/// world scale, but the mass computations that fill this in must
	/// accumulate in int64 before narrowing, and the solver stores the
	/// *inverse* in Q24 rather than inverting this.
	b3Matrix3 inertia;
} b3MassData;

/// The extent of a shape measured from a reference point, usually the body
/// centre of mass.
///
/// Upstream keeps this in the private math_internal.h. It lives here because
/// b3BodySim stores it and the solver reads it: minExtent sizes speculative
/// contacts and the CCD sweep, maxExtent turns an angular velocity into the
/// linear speed of the fastest-moving point on the body.
///
/// Both are lengths at Q12. maxExtent is a distance from the reference point
/// rather than a half-width, so a shape offset far from the centre of mass
/// gives a value near the world-scale bound of B3_HUGE rather than near the
/// shape's own size.
typedef struct b3ShapeExtent
{
	/// Smallest distance from the reference point to the shape surface.
	b3f minExtent;

	/// Per-axis largest distance from the reference point to the shape.
	b3Vec3 maxExtent;
} b3ShapeExtent;

// =========================================================================
// Shape primitives
// =========================================================================

/// A solid sphere.
typedef struct b3Sphere
{
	/// The local center.
	b3Vec3 center;

	/// The radius.
	b3f radius;
} b3Sphere;

/// A solid capsule: two hemispheres joined by a cylinder.
typedef struct b3Capsule
{
	/// Local center of the first hemisphere.
	b3Vec3 center1;

	/// Local center of the second hemisphere.
	b3Vec3 center2;

	/// The radius of the hemispheres.
	b3f radius;
} b3Capsule;

/// A line segment.
typedef struct b3Segment
{
	b3Vec3 point1; ///< The first point.
	b3Vec3 point2; ///< The second point.
} b3Segment;

// =========================================================================
// Convex hulls
// =========================================================================
//
// A hull is a half-edge mesh in one flat relocatable block: the arrays hang
// off the end of the header and are reached by byte offset rather than by
// pointer, so a hull baked on the host loads with no fixup and can live in
// ROM. See source/box3d/hull.c for what the port does and does not build.
//
// Two departures from upstream, both recorded where they bite:
//
//  - There are no SoA (split x/y/z) vertex and normal arrays. They exist
//    upstream to feed a 4-wide support function that recovers its argmax
//    index from float mantissa bits; ARMv5TE has no SIMD, and the index
//    trick would cost 5 of Q12's 12 fraction bits. The port takes the
//    support point with a scalar loop and saves ~380 bytes per hull.
//
//  - centralInertia is per unit mass, matching b3MassData.

/// Identifies a baked hull as this port's, at this layout.
///
/// Deliberately not upstream's B3_HULL_VERSION: the layouts differ, and a
/// float hull loaded as a fixed-point one would be silently wrong rather
/// than obviously wrong. Spells "NEAHULL" followed by a layout revision.
#define B3_HULL_VERSION 0x4E454148554C4C01ull

/// A hull vertex, identified by one half-edge leaving it. Following that
/// edge's twin and winding order walks every edge at this vertex.
typedef struct b3HullVertex
{
	uint8_t edge;
} b3HullVertex;

/// A half-edge. Edges are stored in twin pairs, so edge i's twin is i^1.
typedef struct b3HullHalfEdge
{
	uint8_t next;	///< Next edge index, counter-clockwise around the face.
	uint8_t twin;	///< Twin edge index.
	uint8_t origin; ///< Index of the origin vertex and point.
	uint8_t face;	///< Face to the left of this edge.
} b3HullHalfEdge;

/// A convex polygon face, identified by an arbitrary half-edge on it.
typedef struct b3HullFace
{
	uint8_t edge;
} b3HullFace;

/// A convex hull.
///
/// @note Data hangs off the end of this struct, so it cannot be copied by
/// assignment. Copy `byteCount` bytes instead.
typedef struct b3HullData
{
	/// Must be first and must equal B3_HULL_VERSION.
	uint64_t version;

	/// Total size of the hull including the arrays that follow it.
	int byteCount;

	/// Content hash, computed with this field zero.
	uint32_t hash;

	/// Axis-aligned box in local space.
	b3AABB aabb;

	/// Surface area, in length².
	b3f surfaceArea;

	/// Volume, in length³. Q12 caps this at a cube of side ~80 units, which
	/// is the practical ceiling on hull size and is checked when baking.
	b3f volume;

	/// Radius of the largest sphere centred at `center` and inside the hull.
	b3f innerRadius;

	/// The local centroid.
	b3Vec3 center;

	/// Inertia about the centroid, **per unit mass** -- radius of gyration
	/// squared, in length². Upstream stores a volume-weighted tensor and
	/// multiplies by density per query; this is that divided by volume, so
	/// b3ComputeHullMass needs no division and nothing here grows as r⁵.
	b3Matrix3 centralInertia;

	int vertexCount;  ///< Number of vertices.
	int vertexOffset; ///< Byte offset of the b3HullVertex array.
	int pointOffset;  ///< Byte offset of the b3Vec3 point array.
	int edgeCount;	  ///< Number of *half* edges, so twice the edge count.
	int edgeOffset;	  ///< Byte offset of the b3HullHalfEdge array.
	int faceCount;	  ///< Number of faces.
	int planeOffset;  ///< Byte offset of the b3Plane array, one per face.
	int faceOffset;	  ///< Byte offset of the b3HullFace array.

	/// Explicit padding. Hull identity is a content hash over the raw bytes,
	/// so no unnamed padding may exist for a struct copy to leave scrambled.
	int padding;
} b3HullData;

/// A box hull, built analytically rather than baked.
///
/// Boxes are the one hull the device can construct for itself: the topology
/// is constant and the planes and points are direct, so no hull builder is
/// needed. Everything hangs off the embedded header at fixed offsets.
typedef struct b3BoxHull
{
	b3HullData base;			 ///< The embedded hull. Offsets index the arrays below.
	b3HullVertex boxVertices[8]; ///< Box vertices.
	b3Vec3 boxPoints[8];		 ///< Box points.
	b3HullHalfEdge boxEdges[24]; ///< Box half-edges.
	b3Plane boxPlanes[6];		 ///< Box face planes.
	b3HullFace boxFaces[6];		 ///< Box faces.
	uint8_t padding[2];			 ///< Explicit padding, see b3HullData::padding.
} b3BoxHull;

/// The most sides a prism hull may have.
///
/// A prism with s sides has 2s vertices, 3s full edges and s+2 faces, so
/// B3_MAX_HULL_EDGES at 32 is what binds: 3s <= 32 gives s <= 10. Raising it
/// means raising B3_MAX_HULL_EDGES, which costs stack in the separating axis
/// test rather than anything here.
#define B3_MAX_PRISM_SIDES 10

/// A right prism with a regular polygon cross-section, axis along Y.
///
/// The second hull the device builds for itself. Like the box its topology is
/// a function of the side count alone, so it needs no builder -- but unlike
/// the box the arrays are sized for the maximum and only the first
/// `base.vertexCount` etc. entries are live. That wastes at most a few
/// hundred bytes in a value the caller owns, and it keeps the header offsets
/// constant, which is what lets one struct describe every side count.
typedef struct b3PrismHull
{
	b3HullData base;									 ///< The embedded hull. Offsets index the arrays below.
	b3HullVertex prismVertices[2 * B3_MAX_PRISM_SIDES];	 ///< Prism vertices.
	b3Vec3 prismPoints[2 * B3_MAX_PRISM_SIDES];			 ///< Prism points.
	b3HullHalfEdge prismEdges[6 * B3_MAX_PRISM_SIDES];	 ///< Prism half-edges.
	b3Plane prismPlanes[B3_MAX_PRISM_SIDES + 2];		 ///< Prism face planes.
	b3HullFace prismFaces[B3_MAX_PRISM_SIDES + 2];		 ///< Prism faces.
	uint8_t padding[2];									 ///< Explicit padding, see b3HullData::padding.
} b3PrismHull;

// =========================================================================
// Triangle meshes
// =========================================================================
//
// A mesh is a triangle soup plus a bounding volume hierarchy over it, in one
// flat relocatable block -- the same shape as a hull, and for the same reason:
// the arrays hang off the end of the header and are reached by byte offset, so
// a mesh baked on the host loads with no fixup and can live in ROM.
//
// The port has no mesh builder at all, exactly as it has no hull builder. The
// welding, the BVH split and the edge classification all run on the host at
// full precision and only the result is quantized; see
// tests/box3d_host/mesh_bake.c. What is here, and in source/box3d/mesh.c, is
// the read side.
//
// The layout is deliberately byte-identical to upstream's, which costs nothing
// because every float in it is a b3Vec3 component. The _Static_asserts below
// are what keeps it that way.

/// Identifies a baked mesh as this port's, at this layout.
///
/// Deliberately not upstream's B3_MESH_VERSION, for the reason B3_HULL_VERSION
/// gives: a float mesh loaded as a fixed-point one would be silently wrong
/// rather than obviously wrong. Spells "NEAMESH" followed by a layout revision.
#define B3_MESH_VERSION 0x4E45414D45534801ull

/// Which part of a triangle a contact resolved against.
///
/// Written by the triangle collide functions and read by the mesh narrow
/// phase's ghost filter, which drops a contact whose feature a neighbouring
/// triangle has already claimed. That is why this is public rather than
/// internal to triangle_manifold.c: it crosses from one to the other through
/// b3LocalManifold.
///
/// The edge numbering matches b3MeshEdgeFlags -- edge 1 is v1->v2.
typedef enum b3TriangleFeature
{
	b3_featureNone = 0,
	b3_featureTriangleFace,
	b3_featureHullFace,
	b3_featureEdge1, ///< v1-v2
	b3_featureEdge2, ///< v2-v3
	b3_featureEdge3, ///< v3-v1
	b3_featureVertex1,
	b3_featureVertex2,
	b3_featureVertex3,
} b3TriangleFeature;

/// A point on a triangle, and which feature of it that point lies on.
typedef struct b3TrianglePoint
{
	b3Vec3 point;
	b3TriangleFeature feature;
} b3TrianglePoint;

/// Per-triangle edge classification, computed when the mesh is baked.
///
/// This is what stops a body catching on the interior edges of a flat floor.
/// An edge shared by two triangles is *concave* when the neighbour lies above
/// this triangle's plane, *inverse concave* when it lies below, and both --
/// which is what b3_flatEdgeN means -- when the two faces are within about 5
/// degrees of coplanar. The narrow phase drops contacts against edges the
/// neighbouring triangle has already claimed.
///
/// Edge N runs from vertex N to vertex N+1, so edge 1 is v1->v2, edge 2 is
/// v2->v3 and edge 3 is v3->v1.
typedef enum b3MeshEdgeFlags
{
	b3_concaveEdge1 = 0x01,
	b3_concaveEdge2 = 0x02,
	b3_concaveEdge3 = 0x04,

	b3_inverseConcaveEdge1 = 0x10,
	b3_inverseConcaveEdge2 = 0x20,
	b3_inverseConcaveEdge3 = 0x40,

	b3_allConcaveEdges = b3_concaveEdge1 | b3_concaveEdge2 | b3_concaveEdge3,

	b3_flatEdge1 = b3_concaveEdge1 | b3_inverseConcaveEdge1,
	b3_flatEdge2 = b3_concaveEdge2 | b3_inverseConcaveEdge2,
	b3_flatEdge3 = b3_concaveEdge3 | b3_inverseConcaveEdge3,

	b3_allFlatEdges = b3_flatEdge1 | b3_flatEdge2 | b3_flatEdge3,
} b3MeshEdgeFlags;

/// One triangle, as three indices into the mesh's vertex array.
typedef struct b3MeshTriangle
{
	int32_t index1; ///< Index of vertex 1.
	int32_t index2; ///< Index of vertex 2.
	int32_t index3; ///< Index of vertex 3.
} b3MeshTriangle;

/// A node of the mesh's bounding volume hierarchy.
///
/// @section encoding How a node says what it is
///
/// The two low bits of `data` are the tag. An internal node stores its split
/// axis there, which is 0, 1 or 2; the unused value 3 (B3_LEAF_NODE) marks a
/// leaf. So there is no separate flag and no wasted word.
///
/// An internal node's **left child is implicit**: it is the next node in the
/// array, because the builder emits nodes in pre-order. Only the right child
/// needs an offset, and it is relative to this node rather than absolute,
/// which is what keeps it inside 30 bits.
///
/// A leaf names a contiguous run of `triangleCount` triangles starting at
/// `triangleOffset`. Those runs are in depth-first order across the whole
/// tree, which is why a left-child-first descent emits ascending triangle
/// indices -- see b3QueryMesh.
///
/// The bounds straddle the tag word rather than sitting together. That is
/// upstream's layout, chosen so each is a single 12-byte SIMD load; the port
/// has no SIMD but keeps the layout so the blob stays byte-compatible.
typedef struct b3MeshNode
{
	/// The lower bound of the node AABB.
	b3Vec3 lowerBound;

	union
	{
		/// Internal node.
		struct
		{
			uint32_t axis : 2;		  ///< Split axis, 0, 1 or 2.
			uint32_t childOffset : 30; ///< Right child, relative to this node.
		} asNode;

		/// Leaf node.
		struct
		{
			uint32_t type : 2;			///< B3_LEAF_NODE (3) if this is a leaf.
			uint32_t triangleCount : 30; ///< Triangles in this leaf.
		} asLeaf;
	} data;

	/// The upper bound of the node AABB.
	b3Vec3 upperBound;

	/// First triangle of a leaf's run. Zero for an internal node, which the
	/// baker enforces so the blob's bytes are deterministic.
	uint32_t triangleOffset;
} b3MeshNode;

/// A baked triangle mesh: a BVH, the vertices, the triangles, and per-triangle
/// materials and edge flags.
///
/// @note Data hangs off the end of this struct, so it cannot be copied by
/// assignment. Copy `byteCount` bytes instead.
///
/// Every `*Offset` is a byte offset from the address of this struct, and zero
/// means the array is absent.
typedef struct b3MeshData
{
	/// Must be first and must equal B3_MESH_VERSION.
	uint64_t version;

	/// Total size of the mesh including the arrays that follow it.
	int byteCount;

	/// Content hash, computed with this field zero.
	///
	/// Written by the baker and **not checked here**: verifying it means
	/// scanning the whole blob, which is real time on a 67 MHz ARM9 for a
	/// guarantee the toolchain can give more cheaply. It exists so a build
	/// step can tell a stale .b3mesh from a current one.
	uint32_t hash;

	/// Axis-aligned box in local space, before b3Mesh::scale.
	b3AABB bounds;

	/// Combined single-sided area of every triangle, in length².
	///
	/// Nothing reads this -- upstream computes and stores it and then never
	/// looks at it either. Kept because dropping a field from the middle of a
	/// blob layout buys 4 bytes and costs the diff against upstream, and
	/// because it is a useful number for the converter to print.
	b3f surfaceArea;

	/// Height of the bounding volume hierarchy. The baker rejects a mesh whose
	/// height would exceed B3_MESH_STACK_SIZE, so a traversal never overflows.
	int treeHeight;

	/// Triangles dropped at bake time for being degenerate. Diagnostic.
	int degenerateCount;

	int nodeOffset;		///< Byte offset of the b3MeshNode array.
	int nodeCount;		///< Number of BVH nodes.
	int vertexOffset;	///< Byte offset of the b3Vec3 vertex array.
	int vertexCount;	///< Number of vertices.
	int triangleOffset; ///< Byte offset of the b3MeshTriangle array.
	int triangleCount;	///< Number of triangles.
	int materialOffset; ///< Byte offset of the per-triangle material index array.
	int materialCount;	///< Number of distinct materials the indices select from.
	int flagsOffset;	///< Byte offset of the per-triangle b3MeshEdgeFlags array.
} b3MeshData;

// The blob layout is the contract between tests/box3d_host/mesh_bake.c and
// everything that reads a mesh. These sizes are that contract, and they are
// also upstream's, so a baked mesh is byte-compatible in both directions.
//
// Only in device mode: the shadow-value builds carry a double alongside every
// fixed-point number, so b3Vec3 and therefore b3AABB are much wider there.
// That is also why the baker computes its offsets from sizeof() rather than
// from these numbers.
_Static_assert( sizeof( b3MeshTriangle ) == 12, "b3MeshTriangle layout changed" );

#if !defined( B3_FIXED_DEBUG ) && !defined( B3_FIXED_STRICT )
_Static_assert( sizeof( b3MeshNode ) == 32, "b3MeshNode layout changed" );
_Static_assert( sizeof( b3MeshData ) == 88, "b3MeshData layout changed" );
#endif

/// A mesh instance: the baked data, plus the scale it is used at.
///
/// The data is **not owned** and **not copied** -- the shape keeps this
/// pointer, exactly as a hull shape does, so the blob must outlive every shape
/// that references it.
///
/// One blob can back several shapes at different scales, which is the point of
/// keeping `scale` here rather than baking it in. Components may be negative
/// (a reflection, which flips triangle winding) but not near zero; b3SafeScale
/// enforces a floor of B3_MIN_SCALE.
typedef struct b3Mesh
{
	/// Immutable pointer to the baked mesh data.
	const b3MeshData* data;

	/// Per-axis scale applied to every vertex on the way out.
	b3Vec3 scale;
} b3Mesh;

// =========================================================================
// Contact manifolds
// =========================================================================

/// Identifies a contact point across steps, so warm-started impulses can be
/// matched to the point they belong to.
typedef struct b3FeaturePair
{
	uint8_t owner1; ///< Incoming type (edge on shape A or shape B).
	uint8_t index1; ///< Incoming edge index.
	uint8_t owner2; ///< Outgoing type (edge on shape A or shape B).
	uint8_t index2; ///< Outgoing edge index.
} b3FeaturePair;

/// Which feature pair produced the separating axis.
///
/// b3_invalidAxis must stay zero: b3SATCache is zero-initialized to mean
/// "nothing cached", and the hull collider reads that as the type field.
/// The names follow upstream so the SAT code stays diffable.
typedef enum b3SeparatingFeature
{
	b3_invalidAxis = 0,
	b3_backsideAxis,
	b3_faceAxisA,
	b3_faceAxisB,
	b3_edgePairAxis,
	b3_closestPointsAxis,

	/// These are for testing.
	b3_manualFaceAxisA,
	b3_manualFaceAxisB,
	b3_manualEdgePairAxis,
} b3SeparatingFeature;

/// Separating axis cache. Carries the feature that decided the last query
/// between a shape pair, so the next step can try it first.
typedef struct b3SATCache
{
	/// The separation when the cache was populated. Negative for overlap.
	b3f separation;

	uint8_t type;	///< A b3SeparatingFeature.
	uint8_t indexA; ///< Index of the feature on shape A.
	uint8_t indexB; ///< Index of the feature on shape B.
	uint8_t hit;	///< Was the cache re-used?
} b3SATCache;

typedef struct b3ManifoldPoint
{
	/// Location of the contact point relative to the body A center of mass, in world space.
	b3Vec3 anchorA;

	/// Location of the contact point relative to the body B center of mass, in world space.
	b3Vec3 anchorB;

	/// The separation of the contact point, negative if penetrating.
	b3f separation;

	/// Cached separation, used for contact recycling.
	b3f baseSeparation;

	/// The impulse along the manifold normal. Box3D sub-steps, so this is the
	/// result of the final sub-step.
	b3imp normalImpulse;

	/// The total normal impulse applied across sub-stepping. Used to identify
	/// speculative points that actually interacted during the step.
	b3imp totalNormalImpulse;

	/// Relative normal velocity before solving. Used for hit events. Negative
	/// means the shapes are approaching.
	b3f normalVelocity;

	/// Uniquely identifies this contact point between two shapes, so the
	/// warm-started impulse follows the right point across steps.
	uint32_t featureId;

	/// Triangle index if one of the shapes is a mesh or height field.
	int triangleIndex;

	/// Did this contact point exist in the previous step?
	bool persisted;
} b3ManifoldPoint;

/// A contact manifold describing the contact points between two shapes.
///
/// @note Box3D uses speculative collision, so some points may be separated.
typedef struct b3Manifold
{
	/// The manifold points. There may be 1 to 4 valid points.
	b3ManifoldPoint points[B3_MAX_MANIFOLD_POINTS];

	/// The unit normal in world space, pointing from shape A to shape B.
	b3Vec3 normal;

	/// Central friction angular impulse, applied about the normal.
	b3imp twistImpulse;

	/// Central friction linear impulse.
	///
	/// b3Imp3, not b3Vec3: like the per-point normalImpulse above this is a
	/// warm-start accumulator, carried from step to step by the manifold not
	/// being reallocated while the contact stays touching, and fed back into
	/// itself every sub-step. Q12 would quantize a resting box's hundredths of
	/// a unit to a few dozen quanta and compound the error.
	b3Imp3 frictionImpulse;

	/// Rolling resistance angular impulse. Same reasoning.
	b3Imp3 rollingImpulse;

	/// The number of contact points, 0 to 4.
	int pointCount;
} b3Manifold;

/// A contact point produced by a b3Collide function, before the solver
/// attaches any dynamic state to it.
typedef struct b3LocalManifoldPoint
{
	/// Local point in frame A.
	b3Vec3 point;

	/// The contact point separation. Negative for overlap.
	b3f separation;

	/// The feature pair for this point.
	b3FeaturePair pair;

	/// The triangle index when colliding with a mesh or height field.
	int triangleIndex;
} b3LocalManifoldPoint;

/// A manifold with no dynamic information, produced by the b3Collide
/// functions and consumed by contact.c.
typedef struct b3LocalManifold
{
	/// Local normal in frame A.
	b3Vec3 normal;

	/// The triangle normal, for mesh contacts.
	b3Vec3 triangleNormal;

	/// The manifold points, written into a caller-provided buffer.
	b3LocalManifoldPoint* points;

	/// The number of points written. Bounded by the buffer capacity.
	int pointCount;

	/// The index of the triangle, for mesh contacts.
	int triangleIndex;

	int i1; ///< Vertex 1 index.
	int i2; ///< Vertex 2 index.
	int i3; ///< Vertex 3 index.

	/// Squared distance of a sphere from a triangle, for ghost collision
	/// reduction. Wide, because a squared distance is what it says.
	int64_t squaredDistance;

	/// Which feature of the triangle this manifold resolved against.
	///
	/// b3_featureNone for a convex pair. The mesh narrow phase's ghost filter
	/// reads this to decide whether a neighbouring triangle has already
	/// claimed the edge or vertex a contact sits on.
	b3TriangleFeature feature;

	/// b3MeshEdgeFlags.
	int triangleFlags;
} b3LocalManifold;

/// Sentinel feature pair for manifolds with a single, unambiguous point.
#define b3FeaturePair_single ( ( b3FeaturePair ){ 0, 0, 0, 0 } )

// =========================================================================
// World, body and shape definitions
// =========================================================================
//
// b3BodyType and b3Capacity came forward into Phase 2 ahead of the rest,
// because the broad phase keeps one tree per body type and sizes itself from
// a capacity. Everything after them arrived with Phase 3A.

/// The body simulation type. Determines how a body behaves in the simulation.
typedef enum b3BodyType
{
	/// zero mass, zero velocity, may be manually moved
	b3_staticBody = 0,

	/// zero mass, velocity set by user, moved by solver
	b3_kinematicBody = 1,

	/// positive mass, velocity determined by forces, moved by solver
	b3_dynamicBody = 2,

	/// number of body types
	b3_bodyTypeCount,
} b3BodyType;

/// Expected sizes for a world, used to pre-reserve every pool up front.
///
/// The port allocates nothing during a step, so these are what the whole
/// simulation is sized from rather than a hint.
typedef struct b3Capacity
{
	/// Number of expected static shapes.
	int staticShapeCount;

	/// Number of expected dynamic and kinematic shapes.
	int dynamicShapeCount;

	/// Number of expected static bodies.
	int staticBodyCount;

	/// Number of expected dynamic and kinematic bodies.
	int dynamicBodyCount;

	/// Number of expected contacts.
	int contactCount;

	/// How many of `contactCount` involve a mesh shape.
	///
	/// A port-only field; upstream has no equivalent because upstream grows
	/// on demand. It exists because a mesh contact is not one contact's worth
	/// of memory: it carries up to B3_NEA_MAX_MESH_MANIFOLDS manifolds where a
	/// convex contact carries one, so it needs its own manifold size class and
	/// its own share of the solver's manifold-constraint array. Assuming every
	/// contact might be a mesh contact would multiply both by the cap for
	/// scenes that have no mesh at all.
	///
	/// Count the shapes that can touch the level at once, not the triangles.
	/// Zero -- the default -- means no mesh, and costs nothing.
	int meshContactCount;

	/// Number of expected joints.
	///
	/// A port-only field, for the same reason as meshContactCount: upstream
	/// grows its joint arrays on demand, and here a grow during a step is an
	/// allocation the pool allocator refuses. This sizes world->joints and the
	/// jointSims array of the constraint graph and of all three fixed solver
	/// sets, since a joint's sim can live in any of them depending on whether
	/// its bodies are awake, sleeping, static or disabled.
	///
	/// Zero -- the default -- costs nothing, and is safe: joints are created
	/// while the scene is built, not during a step, so an undeclared one grows
	/// the arrays as build memory rather than as a mid-frame allocation.
	int jointCount;

	/// Number of expected sensor shapes.
	///
	/// A port-only field, and the third of this kind. It sizes world->sensors
	/// and the bitset the sensor pass flags changed overlap sets in. Each
	/// sensor additionally reserves its own overlap arrays when its shape is
	/// created -- B3_NEA_MAX_SENSOR_VISITORS twice and
	/// B3_NEA_MAX_CONTINUOUS_SENSOR_HITS once -- so a sensor is not free even
	/// when nothing ever enters it.
	///
	/// Zero -- the default -- costs nothing. A scene that creates a sensor
	/// without declaring one here grows the array as build memory rather than
	/// mid-frame, since shapes are created while the scene is built.
	int sensorCount;
} b3Capacity;

// -------------------------------------------------------------------------
// Callbacks
// -------------------------------------------------------------------------
//
// Upstream's task callbacks (b3TaskCallback, b3EnqueueTaskCallback,
// b3FinishTaskCallback) and debug-shape callbacks are gone: there is one core
// and no debug renderer here. What remains are the four hooks that change
// simulation behaviour rather than how work is scheduled.

/// Optional friction mixing callback. The default is sqrt(frictionA * frictionB).
typedef b3c b3FrictionCallback( b3c frictionA, uint64_t userMaterialIdA, b3c frictionB, uint64_t userMaterialIdB );

/// Optional restitution mixing callback. The default is max(restitutionA, restitutionB).
typedef b3c b3RestitutionCallback( b3c restitutionA, uint64_t userMaterialIdA, b3c restitutionB, uint64_t userMaterialIdB );

/// Prototype for a contact filter callback. Return false to disable the collision.
///
/// Called only when one of the two shapes has b3_enableCustomFiltering set,
/// and only for awake dynamic bodies.
/// @warning Do not modify the world inside this callback.
typedef bool b3CustomFilterFcn( b3ShapeId shapeIdA, b3ShapeId shapeIdB, void* context );

/// Prototype for a pre-solve callback. Return false to disable the contact this step.
///
/// Called after a contact is updated and before it reaches the solver, only
/// for shapes with b3_enablePreSolveEvents.
/// @warning Do not modify the world inside this callback.
typedef bool b3PreSolveFcn( b3ShapeId shapeIdA, b3ShapeId shapeIdB, b3Pos point, b3Vec3 normal, void* context );

// -------------------------------------------------------------------------
// World definition
// -------------------------------------------------------------------------

/// World definition used to create a simulation world.
/// Must be initialized using b3DefaultWorldDef().
///
/// The time step is not here. It is fixed at compile time by B3_NEA_STEP_HZ
/// (see nea_config.h) so that 1/h, h*h and the solver's soft-constraint
/// coefficients fold into constants instead of costing a hardware divide
/// every substep -- which is why b3World_Step() takes only a substep count.
typedef struct b3WorldDef
{
	/// Gravity vector. Box3D has no up-vector defined.
	b3Vec3 gravity;

	/// Restitution speed threshold. Collisions above this speed bounce.
	b3f restitutionThreshold;

	/// Hit event speed threshold. Collisions above this speed can generate hit
	/// events if the shape also enables them.
	b3f hitEventThreshold;

	/// Contact stiffness in cycles per second. Raising it recovers overlap
	/// faster but can introduce jitter.
	///
	/// A frequency rather than a coefficient, and the default is 30, so this
	/// is b3f and not b3c. The solver turns it into a Q30 softness once per
	/// step, not once per contact.
	b3f contactHertz;

	/// Contact bounciness. Non-dimensional, but the default is 10 -- well
	/// outside the +/-2 range of b3c -- so this is b3f.
	b3f contactDampingRatio;

	/// Cap on the overlap resolution speed. The speed itself is raised by
	/// increasing contactHertz and/or lowering contactDampingRatio.
	b3f contactSpeed;

	/// Maximum linear speed.
	b3f maximumLinearSpeed;

	/// How far from the origin a dynamic body may go before it is parked.
	///
	/// **Not upstream's, and specific to fixed point.** In float a body that
	/// falls out of the level falls forever and costs nothing but its own step
	/// time. In Q19.12 it eventually leaves the range the arithmetic is
	/// documented to work over: b3IsValidFloat declares a b3f valid only while
	/// |raw| < INT32_MAX/2, which is **262,144 units**, and that bound is load
	/// bearing -- b3AABB_Center and its like add two coordinates and rely on
	/// the sum fitting int32. Past it the results are simply wrong, and the
	/// position itself wraps at 524,288.
	///
	/// Nothing enforced that range before this field. A dynamic body with
	/// nothing under it falls at maximumLinearSpeed indefinitely and crosses
	/// 262,144 in about eleven minutes; every b3IsValid* check is a B3_ASSERT
	/// and compiles away in a release build, so the first sign was a physics
	/// example freezing with no diagnostic. Reproduced in
	/// tests/box3d_host/test_world.c, which now also checks the invariant
	/// directly rather than only its symptom.
	///
	/// A body past this distance has its velocity zeroed and is put to sleep
	/// where it is -- not destroyed, not teleported, so a game can find it with
	/// b3Body_GetPosition and decide what to do. Waking it moves it again;
	/// it will simply park once more unless it has been moved back in bounds.
	///
	/// The default is 16,384 units: two orders of magnitude past B3_HUGE, which
	/// is where the *solver* stops being accurate, and a factor of 16 below
	/// where the *arithmetic* stops being correct. Set it to 0 to disable the
	/// check, which restores the old behaviour and the old failure with it.
	b3f maximumWorldExtent;

	/// Optional mixing callback for friction. NULL selects the default.
	b3FrictionCallback* frictionCallback;

	/// Optional mixing callback for restitution. NULL selects the default.
	b3RestitutionCallback* restitutionCallback;

	/// Can bodies go to sleep to improve performance?
	bool enableSleep;

	/// Enable continuous collision. The solver half of this arrives in Phase 7.
	bool enableContinuous;

	/// User data associated with a world.
	void* userData;

	/// Initial capacities. Unlike upstream these are not a hint: the port
	/// pre-reserves every pool from them and refuses to allocate during a step.
	b3Capacity capacity;

	/// Used internally to detect a valid definition. DO NOT SET.
	int internalValue;
} b3WorldDef;

/// Use this to initialize your world definition.
B3_API b3WorldDef b3DefaultWorldDef( void );

// -------------------------------------------------------------------------
// Body definition
// -------------------------------------------------------------------------

/// Motion locks to restrict body movement.
typedef struct b3MotionLocks
{
	/// Prevent translation along the x-axis
	bool linearX;

	/// Prevent translation along the y-axis
	bool linearY;

	/// Prevent translation along the z-axis
	bool linearZ;

	/// Prevent rotation around the x-axis
	bool angularX;

	/// Prevent rotation around the y-axis
	bool angularY;

	/// Prevent rotation around the z-axis
	bool angularZ;
} b3MotionLocks;

/// A body definition holds everything needed to construct a rigid body.
/// Shapes are added to a body after construction.
/// Must be initialized using b3DefaultBodyDef().
typedef struct b3BodyDef
{
	/// The body type: static, kinematic, or dynamic.
	b3BodyType type;

	/// The initial world position of the body. Create bodies where you want
	/// them: moving a body after its shapes are attached costs far more than
	/// creating it in place, because every proxy has to move with it.
	b3Pos position;

	/// The initial world rotation of the body.
	b3Quat rotation;

	/// The initial linear velocity of the body origin.
	b3Vec3 linearVelocity;

	/// The initial angular velocity of the body, in radians per second.
	b3Vec3 angularVelocity;

	/// Linear damping, used to reduce linear velocity. May exceed 1, at which
	/// point the effect becomes sensitive to the time step.
	b3f linearDamping;

	/// Angular damping, used to reduce angular velocity.
	b3f angularDamping;

	/// Scale the gravity applied to this body. Non-dimensional, but may be
	/// negative or larger than 1, so b3f rather than b3c.
	b3f gravityScale;

	/// Sleep speed threshold, default 0.05 units per second.
	b3f sleepThreshold;

	/// Use this to store application specific body data.
	void* userData;

	/// Motion locks restricting linear and angular movement.
	b3MotionLocks motionLocks;

	/// Set false if this body should never fall asleep.
	bool enableSleep;

	/// Is this body initially awake or sleeping?
	bool isAwake;

	/// Treat this body as a high speed object performing continuous collision
	/// against dynamic and kinematic bodies, but not other bullets.
	/// Use sparingly; the solver half arrives in Phase 7.
	bool isBullet;

	/// Used to disable a body. A disabled body does not move or collide.
	bool isEnabled;

	/// Allow this body to bypass rotational speed limits. For wheels and other
	/// circular objects only.
	bool allowFastRotation;

	/// Enable contact recycling. True by default: it improves performance but
	/// can produce ghost collisions that characters should avoid.
	bool enableContactRecycling;

	/// Used internally to detect a valid definition. DO NOT SET.
	int internalValue;
} b3BodyDef;

/// Use this to initialize your body definition.
B3_API b3BodyDef b3DefaultBodyDef( void );

// -------------------------------------------------------------------------
// Shape definition
// -------------------------------------------------------------------------

/// Collision filtering for shape-versus-shape and shape-versus-query tests.
typedef struct b3Filter
{
	/// The collision category bits. Normally a single bit representing an
	/// application object type.
	uint64_t categoryBits;

	/// The categories this shape accepts for collision.
	uint64_t maskBits;

	/// Collision groups make a set of objects never collide (negative) or
	/// always collide (positive). Zero has no effect. Non-zero group filtering
	/// always wins against the mask bits.
	int groupIndex;
} b3Filter;

/// Use this to initialize your filter.
B3_API b3Filter b3DefaultFilter( void );

/// Surface properties. Supported per triangle on meshes; one per shape otherwise.
typedef struct b3SurfaceMaterial
{
	/// The Coulomb (dry) friction coefficient, usually in [0,1].
	b3c friction;

	/// The coefficient of restitution (bounce), usually in [0,1].
	b3c restitution;

	/// The rolling resistance, usually in [0,1]. Spheres and capsules only.
	b3c rollingResistance;

	/// Tangent velocity for conveyor belts, local to the shape and projected
	/// onto the contact surface.
	b3Vec3 tangentVelocity;

	/// User material identifier, passed to query results and to the friction
	/// and restitution mixing callbacks. Not used internally.
	uint64_t userMaterialId;
} b3SurfaceMaterial;

/// Use this to initialize your surface material.
B3_API b3SurfaceMaterial b3DefaultSurfaceMaterial( void );

/// Shape type.
///
/// Every enumerator upstream has is kept, including the three the port does
/// not implement yet, because the contact register table in contact.c is
/// indexed by this pair and its shape must not shift between phases.
typedef enum b3ShapeType
{
	/// A capsule is an extruded sphere
	b3_capsuleShape,

	/// A baked compound shape. Not implemented.
	b3_compoundShape,

	/// A height field useful for terrain. Not implemented.
	b3_heightShape,

	/// A convex hull
	b3_hullShape,

	/// A triangle soup. Phase 5.
	b3_meshShape,

	/// A sphere with an offset
	b3_sphereShape,

	/// The number of shape types
	b3_shapeTypeCount
} b3ShapeType;

/// Used to create a shape. Must be initialized using b3DefaultShapeDef().
typedef struct b3ShapeDef
{
	/// Use this to store application specific shape data.
	void* userData;

	/// Per-triangle surface materials for mesh shapes (Phase 5). Ignored for
	/// convex shapes, which use baseMaterial.
	b3SurfaceMaterial* materials;

	/// Surface material count.
	int materialCount;

	/// The base surface material.
	b3SurfaceMaterial baseMaterial;

	/// The density, usually in kg per cubic unit.
	///
	/// Upstream's explosionScale sits beside this. b3World_Explode is not on
	/// any phase of the port, so neither the field nor the b3Shape copy of it
	/// is carried.
	b3f density;

	/// Contact filtering data.
	b3Filter filter;

	/// Enable custom filtering. Only one of the two shapes needs to enable it.
	bool enableCustomFiltering;

	/// A sensor shape generates overlap events but never a collision response.
	/// Sensors still contribute mass if their density is non-zero. Phase 7.
	bool isSensor;

	/// Enable sensor events for this shape. Phase 7.
	bool enableSensorEvents;

	/// Enable contact events. Kinematic and dynamic bodies only.
	bool enableContactEvents;

	/// Enable hit events. Kinematic and dynamic bodies only.
	bool enableHitEvents;

	/// Enable pre-solve contact events. Dynamic bodies only.
	bool enablePreSolveEvents;

	/// Scan the environment for collision on the next step. Ignored for
	/// dynamic and kinematic shapes, which always invoke contact creation.
	/// Leaving it false makes creating many static shapes much cheaper.
	bool invokeContactCreation;

	/// Should the body update its mass properties when this shape is created?
	/// @warning If false you MUST call b3Body_ApplyMassFromShapes or
	/// b3Body_SetMassData before stepping the world.
	bool updateBodyMass;

	/// Enable speculative collision. Leave true unless ghost collision matters
	/// more than continuous collision under rotation.
	bool enableSpeculativeContact;

	/// Used internally to detect a valid definition. DO NOT SET.
	int internalValue;
} b3ShapeDef;

/// Use this to initialize your shape definition.
B3_API b3ShapeDef b3DefaultShapeDef( void );

// =========================================================================
// Joint definitions
// =========================================================================
//
// One def struct per joint type, each wrapping the base b3JointDef. The base
// arrives whole in Phase 6 Stage 1; the per-type defs arrive one at a time
// with the constraint math that implements them, so this header never
// declares a joint a caller cannot create.

/// Joint type enumeration. Useful because every joint type shares b3JointId
/// and sometimes you want to ask what a joint is.
///
/// The order differs from upstream's, which leads with b3_parallelJoint: the
/// port sorted the list alphabetically when it landed the enum ahead of the
/// types in Phase 3A, and renumbering it now would change nothing except
/// every b3Joint already written to a save. b3_jointTypeCount is a port
/// addition, for switch coverage and array sizing.
typedef enum b3JointType
{
	b3_distanceJoint,
	b3_filterJoint,
	b3_motorJoint,
	b3_parallelJoint,
	b3_prismaticJoint,
	b3_revoluteJoint,
	b3_sphericalJoint,
	b3_weldJoint,
	b3_wheelJoint,
	b3_jointTypeCount,
} b3JointType;

/// The "no bound" sentinel, used for every optional force, torque and threshold
/// bound a joint carries. Upstream writes FLT_MAX; this is the b3f equivalent.
///
/// It is a magnitude, not a flag: nothing compares against it for equality to
/// mean "disabled". It is also small enough to survive the conversions applied
/// to it -- a spring force bound is multiplied by the sub-step to reach the
/// impulse scale, and 262,143 / 240 is 1,092, which is nowhere near Q16's
/// ceiling.
///
/// Public because a caller reads it straight back out of
/// b3Joint_GetForceThreshold and needs something to compare against. It was a
/// private define in types.c until Stage 7's joint events gave the solver a
/// reason to test it too, and one sentinel in two files is one that drifts.
#define B3_NO_BOUND b3Makeb3f( INT32_MAX / 2 - 1 )

/// Base joint definition, shared by every joint type.
///
/// The local frames are measured from the body origin rather than from the
/// centre of mass, for upstream's two reasons: you may not know where the
/// centre of mass will end up, and adding or removing a shape recomputes it,
/// which would silently break every joint already attached.
///
/// Upstream's `drawScale` is absent -- the port has no b3DebugDraw.
typedef struct b3JointDef
{
	/// User data pointer.
	void* userData;

	/// The first attached body.
	b3BodyId bodyIdA;

	/// The second attached body.
	b3BodyId bodyIdB;

	/// The first local joint frame.
	b3Transform localFrameA;

	/// The second local joint frame.
	b3Transform localFrameB;

	/// Force threshold for joint events, in newtons. A force, so b3f.
	///
	/// Defaults to B3_NO_BOUND, which reports nothing. A threshold of zero
	/// reports every awake joint every step, which is a legitimate way to watch
	/// everything at once rather than a degenerate case.
	b3f forceThreshold;

	/// Torque threshold for joint events, in newton-metres. b3f for the same
	/// reason, and the same defaults.
	b3f torqueThreshold;

	/// Constraint hertz (advanced). A frequency, so b3f -- b3MakeSoft takes it
	/// at that scale and clamps it against inv_h, which is also Q12.
	b3f constraintHertz;

	/// Constraint damping ratio (advanced). Dimensionless but *not* bounded by
	/// one -- the joint default is 2 and upstream's world default is 10 -- so
	/// b3c would clip it. b3f.
	b3f constraintDampingRatio;

	/// Set true if the attached bodies should still collide.
	bool collideConnected;

	/// Used internally to detect a valid definition. DO NOT SET.
	int internalValue;
} b3JointDef;

/// A filter joint disables collision between two specific bodies and does
/// nothing else. It has no constraint to solve.
typedef struct b3FilterJointDef
{
	/// Base joint definition.
	b3JointDef base;
} b3FilterJointDef;

/// Use this to initialize your joint definition.
B3_API b3FilterJointDef b3DefaultFilterJointDef( void );

/// A distance joint holds two points on two bodies a fixed distance apart --
/// rigidly by default, or as a spring with an optional length range and an
/// optional motor along the axis.
///
/// One linear degree of freedom, no rotational constraint at all: the bodies
/// are free to spin. A chain of these is a rope; one to a static body is a
/// pendulum.
typedef struct b3DistanceJointDef
{
	/// Base joint definition.
	b3JointDef base;

	/// The rest length. Clamped to B3_LINEAR_SLOP from below.
	b3f length;

	/// Enable the spring. When false the joint is rigid and the limit and
	/// motor are both ignored, which is upstream's rule and not an omission.
	bool enableSpring;

	/// Lower spring force -- how much tension the spring may sustain. Negative
	/// by convention, since tension pulls the bodies together.
	b3f lowerSpringForce;

	/// Upper spring force -- how much compression the spring may sustain.
	b3f upperSpringForce;

	/// Spring stiffness in cycles per second. Zero disables the spring term
	/// while leaving the limit and motor active.
	b3f hertz;

	/// Spring damping ratio, dimensionless. b3f rather than b3c: like
	/// b3JointDef::constraintDampingRatio, it is routinely above one.
	b3f dampingRatio;

	/// Enable the length range. Only consulted when the spring is enabled.
	bool enableLimit;

	/// Minimum length. Clamped to B3_LINEAR_SLOP from below.
	b3f minLength;

	/// Maximum length. Defaults to B3_HUGE, the port's finite stand-in for
	/// upstream's unbounded one.
	b3f maxLength;

	/// Enable the motor. Only consulted when the spring is enabled.
	bool enableMotor;

	/// Maximum motor force, in newtons.
	b3f maxMotorForce;

	/// Desired motor speed along the joint axis, in metres per second.
	b3f motorSpeed;
} b3DistanceJointDef;

/// Use this to initialize your joint definition.
B3_API b3DistanceJointDef b3DefaultDistanceJointDef( void );

/// A revolute joint is a hinge: the two bodies share a point and an axis, and
/// the only freedom left is rotation about that axis.
///
/// The hinge axis is the **z axis of the joint frames**, so a hinge in another
/// direction is made by rotating `base.localFrameA.q` / `localFrameB.q`, not by
/// naming an axis. That is upstream's convention and the port keeps it.
///
/// The remaining degree of freedom takes a spring, an angle limit and a motor,
/// the same three the distance joint offers along its length.
typedef struct b3RevoluteJointDef
{
	/// Base joint definition.
	b3JointDef base;

	/// The angle of body B relative to body A in the reference state, which is
	/// the zero the limits are measured from.
	///
	/// Angles here are **brads**, the port's angle unit everywhere -- 32768 to
	/// a full turn, so B3_BRAD_HALF_PI is a right angle. Upstream uses radians;
	/// the port does not, because every other angle a caller passes (notably
	/// b3MakeQuatFromAxisAngle's) is a brad and mixing the two invites exactly
	/// the mistake that is hardest to see in a screenshot.
	b3a targetAngle;

	/// Enable a rotational spring about the hinge axis.
	bool enableSpring;

	/// Spring stiffness in cycles per second.
	b3f hertz;

	/// Spring damping ratio, dimensionless. b3f rather than b3c for the same
	/// reason as everywhere else: it is routinely above one.
	b3f dampingRatio;

	/// Enable the angle limit.
	bool enableLimit;

	/// Lower angle limit, in brads. Must be at least -0.99 of a half turn.
	b3a lowerAngle;

	/// Upper angle limit, in brads. At most 0.99 of a half turn.
	b3a upperAngle;

	/// Enable the motor.
	bool enableMotor;

	/// Maximum motor torque, in newton-metres.
	b3f maxMotorTorque;

	/// Desired motor speed, in **radians per second** -- an angular velocity,
	/// not an angle, so it shares its units with b3Body_GetAngularVelocity
	/// rather than with the limits above.
	b3f motorSpeed;
} b3RevoluteJointDef;

/// Use this to initialize your joint definition.
B3_API b3RevoluteJointDef b3DefaultRevoluteJointDef( void );

/// Spherical joint definition -- a ball joint.
///
/// Locks the two joint frame origins together and leaves all three rotational
/// degrees free, then bounds them: a **cone limit** on how far frame B's z-axis
/// may tilt away from frame A's, and a **twist limit** on the rotation about
/// that axis. Both are optional, as are the spring and the motor.
typedef struct b3SphericalJointDef
{
	/// Base joint definition.
	b3JointDef base;

	/// Enable a rotational spring that pulls the two joint frames toward
	/// targetRotation. Unlike the revolute's, this spring acts on all three
	/// rotational degrees.
	bool enableSpring;

	/// Spring stiffness in cycles per second.
	b3f hertz;

	/// Spring damping ratio, dimensionless. b3f rather than b3c for the same
	/// reason as everywhere else: it is routinely above one.
	b3f dampingRatio;

	/// Target spring rotation, joint frame B relative to joint frame A.
	/// b3DefaultSphericalJointDef sets this to the identity; a zeroed def would
	/// leave a zero quaternion here, which is not a rotation.
	b3Quat targetRotation;

	/// Enable the cone limit, centred on frame A's z-axis.
	bool enableConeLimit;

	/// Cone half-angle, in **brads**, in [0, half a turn]. As with the
	/// revolute's limits, angles a caller passes are brads throughout the port
	/// -- 32768 to a full turn -- because b3MakeQuatFromAxisAngle takes brads
	/// and mixing units is the mistake hardest to see in a screenshot.
	b3a coneAngle;

	/// Enable the twist limit, about frame B's z-axis.
	bool enableTwistLimit;

	/// Lower twist limit, in brads. Must be at least -0.99 of a half turn, for
	/// the reason b3DefaultRevoluteJointDef gives: the twist is recovered
	/// through an atan2 and is single valued only on the open half turn.
	b3a lowerTwistAngle;

	/// Upper twist limit, in brads. At most 0.99 of a half turn.
	b3a upperTwistAngle;

	/// Enable the motor.
	bool enableMotor;

	/// Maximum motor torque, in newton-metres. Bounds the *magnitude* of the
	/// 3-vector motor impulse, not each axis separately.
	b3f maxMotorTorque;

	/// Desired relative angular velocity, in **radians per second** -- a
	/// velocity, not an angle, so it shares its units with
	/// b3Body_GetAngularVelocity rather than with the limits above.
	b3Vec3 motorVelocity;
} b3SphericalJointDef;

/// Use this to initialize your joint definition.
B3_API b3SphericalJointDef b3DefaultSphericalJointDef( void );

/// Weld joint definition -- two bodies made one.
///
/// Locks both the two joint frame origins and their orientations together,
/// removing all six degrees of freedom. Solved rather than assumed, so the
/// joint can be destroyed at runtime and the two halves come apart -- which is
/// the reason to use one instead of merging the shapes onto a single body.
///
/// Either half can be softened into a spring by giving it a non-zero hertz;
/// leaving both at zero, which is the default, gives a rigid weld. The two are
/// independent: a rigid position with a springy orientation is legal.
typedef struct b3WeldJointDef
{
	/// Base joint definition.
	b3JointDef base;

	/// Linear spring stiffness in cycles per second. **Zero means rigid**, not
	/// "no constraint" -- the position is then held by the joint's own
	/// constraint softness, as every other joint's anchors are.
	b3f linearHertz;

	/// Linear spring damping ratio, dimensionless. b3f rather than b3c for the
	/// reason every other joint's is: it is routinely above one.
	b3f linearDampingRatio;

	/// Angular spring stiffness in cycles per second. Zero means rigid.
	b3f angularHertz;

	/// Angular spring damping ratio, dimensionless.
	b3f angularDampingRatio;
} b3WeldJointDef;

/// Use this to initialize your joint definition.
B3_API b3WeldJointDef b3DefaultWeldJointDef( void );

/// Motor joint definition -- a bounded 6-DOF drive.
///
/// Constrains nothing. Instead it *pushes* body B toward a target pose and a
/// target velocity relative to body A, spending at most the force and torque
/// the four bounds allow. That ceiling is the point: a platform driven by a
/// motor joint goes where it is told and still loses to a wall, where one moved
/// with b3Body_SetTransform teleports through whatever is in the way.
///
/// **Every bound defaults to zero, and a zero bound disables its branch.** A
/// zeroed def therefore does nothing at all, which is deliberate -- each of the
/// four drives is opted into by giving it a budget.
typedef struct b3MotorJointDef
{
	/// Base joint definition. The target *pose* is the two joint frames: the
	/// drive works to bring frame B onto frame A.
	b3JointDef base;

	/// Target linear velocity of B relative to A, in metres per second.
	b3Vec3 linearVelocity;

	/// Maximum force the linear velocity drive may use, in newtons. Zero
	/// disables it.
	b3f maxVelocityForce;

	/// Target angular velocity of B relative to A, in **radians per second** --
	/// a velocity, not an angle, so it shares its units with
	/// b3Body_GetAngularVelocity rather than being brads.
	b3Vec3 angularVelocity;

	/// Maximum torque the angular velocity drive may use, in newton-metres.
	/// Zero disables it.
	///
	/// Note a bound below about 0.004 N-m at the default four sub-steps lands
	/// under half an impulse quantum and clamps the drive to nothing. That is a
	/// limit of the fixed-point impulse scale, not a bug, and it is the usual
	/// reason a motor with a small-looking bound appears dead.
	b3f maxVelocityTorque;

	/// Linear spring stiffness in cycles per second. The linear spring runs
	/// only when this **and** maxSpringForce are both non-zero.
	b3f linearHertz;

	/// Linear spring damping ratio, dimensionless.
	b3f linearDampingRatio;

	/// Maximum force the linear spring may use, in newtons. Zero disables it.
	b3f maxSpringForce;

	/// Angular spring stiffness in cycles per second. The angular spring runs
	/// only when this **and** maxSpringTorque are both non-zero.
	b3f angularHertz;

	/// Angular spring damping ratio, dimensionless.
	b3f angularDampingRatio;

	/// Maximum torque the angular spring may use, in newton-metres. Zero
	/// disables it.
	b3f maxSpringTorque;
} b3MotorJointDef;

/// Use this to initialize your joint definition.
B3_API b3MotorJointDef b3DefaultMotorJointDef( void );

// -------------------------------------------------------------------------
// Prismatic joint
// -------------------------------------------------------------------------

/// A slider: one translational degree of freedom along the **x axis of the
/// joint frames**, with everything else locked.
///
/// The revolute joint with the angle-unit problem deleted. Every quantity here
/// is a length in metres or a speed in metres per second, at the same Q12 as
/// every other position in the port -- so unlike b3RevoluteJointDef there is no
/// brad-versus-radian split to keep straight, and no conversion happens at the
/// constraint.
typedef struct b3PrismaticJointDef
{
	/// Base joint definition.
	b3JointDef base;

	/// The translation the spring pulls toward, in metres, measured from the
	/// reference state.
	b3f targetTranslation;

	/// Enable a spring along the slide axis.
	bool enableSpring;

	/// Spring stiffness in cycles per second.
	b3f hertz;

	/// Spring damping ratio, dimensionless. b3f rather than b3c for the reason
	/// every other joint's is: it is routinely above one.
	b3f dampingRatio;

	/// Enable the translation limit.
	bool enableLimit;

	/// Lower translation limit, in metres.
	///
	/// **A default def with enableLimit set is a slider locked shut**, because
	/// both limits default to zero. Set the range.
	///
	/// Unlike b3DistanceJointDef::maxLength this does *not* default to B3_HUGE,
	/// and the reason is fixed point rather than taste. The limit's speculative
	/// band is a quarter of the range, so a 4000-unit range would put the band
	/// at 1000 m -- the limit branch would then run from anywhere in the world,
	/// and its speculative bias `C * inv_h` would reach 240,000 against Q12's
	/// ceiling of 524,288. That is an impulse ask the solver cannot represent,
	/// on a joint the caller never configured.
	b3f lowerTranslation;

	/// Upper translation limit, in metres.
	b3f upperTranslation;

	/// Enable the motor.
	bool enableMotor;

	/// Maximum motor force, in newtons.
	b3f maxMotorForce;

	/// Desired motor speed, in metres per second.
	b3f motorSpeed;
} b3PrismaticJointDef;

/// Use this to initialize your joint definition.
B3_API b3PrismaticJointDef b3DefaultPrismaticJointDef( void );

/// A parallel joint: keeps body B's z axis aligned with body A's z axis, with a
/// spring, leaving the twist about z free.
///
/// The upright-keeper. It constrains **orientation only** -- the two bodies may
/// be anywhere relative to one another -- so it is the joint for a mast that
/// should stay vertical over bumps, or a turret that should stay level while
/// its platform pitches. If the heading should also be held, that is a weld
/// joint.
///
/// The constraint is always soft and always bounded, which is the point rather
/// than a limitation: a strong enough disturbance overcomes `maxTorque` and the
/// body tips, and then the spring rights it.
typedef struct b3ParallelJointDef
{
	/// Base joint definition.
	b3JointDef base;

	/// Spring stiffness in cycles per second.
	b3f hertz;

	/// Spring damping ratio, dimensionless. b3f rather than b3c for the reason
	/// every other joint's is: it is routinely above one.
	b3f dampingRatio;

	/// The joint's torque budget.
	///
	/// **A default def does nothing at all**, because this defaults to zero and
	/// the constraint is skipped entirely when it is. That is deliberate and it
	/// is the same reasoning b3PrismaticJointDef::lowerTranslation gives:
	/// upstream defaults this to FLT_MAX, which has no fixed-point counterpart,
	/// and inventing a large finite default would hand a caller who never
	/// configured the joint an impulse budget nobody chose. Set it.
	///
	/// @warning **This is not the torque in newton-metres, and the factor is
	/// two.** It bounds the constraint's impulse *coefficient*, and that
	/// coefficient is applied along a Jacobian row of length
	/// `0.5 * sqrt(1 - (v.e)^2)` -- one half when the joint is near aligned. So
	/// the peak torque b3Joint_GetConstraintTorque reports is about
	/// `0.5 * maxTorque`, falling further as the joint tips away from aligned.
	///
	/// This is upstream's behaviour, reproduced deliberately rather than
	/// corrected. The factor is a bounded constant and it errs *conservatively*
	/// -- the joint applies less than the number suggests, never more -- so
	/// rescaling it here would buy a nicer unit at the cost of putting the port
	/// out of step with upstream on a quantity a caller might have tuned
	/// against it. Measured, not inferred: a saturated joint with maxTorque 0.5
	/// reports a peak of 0.2486. Budget accordingly, and read the number the
	/// accessor gives you rather than the one you set.
	b3f maxTorque;
} b3ParallelJointDef;

/// Use this to initialize your joint definition.
B3_API b3ParallelJointDef b3DefaultParallelJointDef( void );

/// A wheel joint: a suspension, a spin axis and a steering axis in one.
///
/// The largest joint in the library, and the only one that is two joints at
/// once — a prismatic along frame A's **x** carrying the suspension, and a
/// revolute about frame **B's z** carrying the wheel's spin, with a steering
/// twist about A's x coupling them. Seven constraint blocks, up to nine
/// coupled impulses per sub-step.
///
/// The axis convention is worth stating plainly because it is easy to get
/// backwards: **the suspension travels along frame A's x**, so a vertical
/// suspension needs frame A rotated to put x where you want the travel — the
/// same arrangement `b3PrismaticJointDef` needs and for the same reason. The
/// wheel spins about frame **B's** z, not A's, so steering the wheel moves the
/// spin axis with it.
typedef struct b3WheelJointDef
{
	/// Base joint definition.
	b3JointDef base;

	/// Enable the suspension spring along the travel axis.
	bool enableSuspensionSpring;

	/// Suspension stiffness in cycles per second.
	b3f suspensionHertz;

	/// Suspension damping ratio, dimensionless. b3f rather than b3c for the
	/// reason every other joint's is: it is routinely above one.
	b3f suspensionDampingRatio;

	/// Enable the suspension travel limit.
	bool enableSuspensionLimit;

	/// Lower suspension travel limit, in **metres**.
	///
	/// A length, not an angle — the suspension is the joint's linear half. Both
	/// limits default to zero, so a default def with `enableSuspensionLimit`
	/// set is a suspension locked solid; set the range. The reasoning is
	/// `b3PrismaticJointDef::lowerTranslation`'s: +/-B3_HUGE would put the
	/// speculative band at 1000 m and its bias past what Q12 holds.
	b3f lowerSuspensionLimit;

	/// Upper suspension travel limit, in metres.
	b3f upperSuspensionLimit;

	/// Enable the spin motor that drives the wheel.
	bool enableSpinMotor;

	/// Maximum spin torque, in newton-metres.
	b3f maxSpinTorque;

	/// Desired spin speed, in radians per second.
	///
	/// A velocity, so it shares units and scale with b3BodyState::angularVelocity
	/// and is **not** in brads — the same call the revolute's motorSpeed makes.
	b3f spinSpeed;

	/// Enable steering. With it off the wheel is held pointing forward.
	bool enableSteering;

	/// Steering stiffness in cycles per second.
	b3f steeringHertz;

	/// Steering damping ratio, dimensionless.
	b3f steeringDampingRatio;

	/// The steering angle the spring pulls toward, in **brads**.
	///
	/// An angle, so it is a b3a: 32768 to a full turn, B3_BRAD_HALF_PI a right
	/// angle. Upstream uses radians. This is the port's angle convention and it
	/// applies to all three steering angles below.
	b3a targetSteeringAngle;

	/// Maximum steering torque, in newton-metres.
	b3f maxSteeringTorque;

	/// Enable the steering angle limit.
	bool enableSteeringLimit;

	/// Lower steering limit, in brads.
	b3a lowerSteeringLimit;

	/// Upper steering limit, in brads.
	b3a upperSteeringLimit;
} b3WheelJointDef;

/// Use this to initialize your joint definition.
B3_API b3WheelJointDef b3DefaultWheelJointDef( void );

// =========================================================================
// Events
// =========================================================================
//
// Buffered in the world during a step and read back afterwards. Joint events
// arrived with Phase 6 Stage 7, once every joint type existed -- the threshold
// test reads the reaction of whatever type the joint happens to be, so it could
// not be written before the last one. Sensor events arrived with Phase 7 Stage
// 3, and are the only ones here produced by comparing two steps rather than by
// something that happened during one.

/// A begin-touch event is generated when two shapes begin touching.
typedef struct b3ContactBeginTouchEvent
{
	/// Id of the first shape
	b3ShapeId shapeIdA;

	/// Id of the second shape
	b3ShapeId shapeIdB;

	/// The transient contact id. The contact may be destroyed automatically
	/// when the world is modified or stepped -- check b3Contact_IsValid first.
	b3ContactId contactId;
} b3ContactBeginTouchEvent;

/// An end-touch event is generated when two shapes stop touching.
///
/// You also get one for anything that destroys a contact outside the step:
/// setting a transform, destroying a body or shape, changing a filter or a
/// body type.
typedef struct b3ContactEndTouchEvent
{
	/// Id of the first shape. @warning may have been destroyed
	b3ShapeId shapeIdA;

	/// Id of the second shape. @warning may have been destroyed
	b3ShapeId shapeIdB;

	/// Id of the contact. @warning may have been destroyed
	b3ContactId contactId;
} b3ContactEndTouchEvent;

/// A hit event is generated when two shapes collide faster than the world's
/// hit event threshold. May be reported for a speculative contact once its
/// impulse is confirmed.
typedef struct b3ContactHitEvent
{
	/// Id of the first shape
	b3ShapeId shapeIdA;

	/// Id of the second shape
	b3ShapeId shapeIdB;

	/// Id of the contact. @warning may have been destroyed
	b3ContactId contactId;

	/// Where the shapes hit at the beginning of the time step: a mid-point
	/// between the two surfaces, possibly a speculative point at which they
	/// were not yet touching.
	b3Pos point;

	/// Normal vector pointing from shape A to shape B
	b3Vec3 normal;

	/// The approach speed. Always positive.
	b3f approachSpeed;

	/// User material on shape A
	uint64_t userMaterialIdA;

	/// User material on shape B
	uint64_t userMaterialIdB;
} b3ContactHitEvent;

/// Contact events buffered in the world, available after the step completes.
/// @note These may become invalid if bodies or shapes are destroyed.
typedef struct b3ContactEvents
{
	/// Array of begin touch events
	b3ContactBeginTouchEvent* beginEvents;

	/// Array of end touch events
	b3ContactEndTouchEvent* endEvents;

	/// Array of hit events
	b3ContactHitEvent* hitEvents;

	/// Number of begin touch events
	int beginCount;

	/// Number of end touch events
	int endCount;

	/// Number of hit events
	int hitCount;
} b3ContactEvents;

/// A begin-touch event is generated when a shape starts overlapping a sensor.
///
/// Both shapes must have sensor events enabled: the sensor to report, and the
/// visitor to be reported. A shape never trips a sensor on its own body.
typedef struct b3SensorBeginTouchEvent
{
	/// The id of the sensor shape.
	b3ShapeId sensorShapeId;

	/// The id of the shape that entered the sensor.
	b3ShapeId visitorShapeId;
} b3SensorBeginTouchEvent;

/// An end-touch event is generated when a shape stops overlapping a sensor.
///
/// You also get one when the overlap is ended by something other than motion:
/// either shape being destroyed or disabled, or sensor events being turned off
/// on either of them.
typedef struct b3SensorEndTouchEvent
{
	/// The id of the sensor shape. @warning may have been destroyed
	b3ShapeId sensorShapeId;

	/// The id of the shape that left the sensor. @warning may have been
	/// destroyed
	b3ShapeId visitorShapeId;
} b3SensorEndTouchEvent;

/// Sensor events buffered in the world, available after the step completes.
///
/// Only transitions are reported. A shape that stays inside a sensor produces
/// one begin event and then nothing until it leaves, so the current contents of
/// a sensor are b3Shape_GetSensorData rather than an event count.
/// @note These may become invalid if bodies or shapes are destroyed.
typedef struct b3SensorEvents
{
	/// Array of sensor begin touch events
	b3SensorBeginTouchEvent* beginEvents;

	/// Array of sensor end touch events
	b3SensorEndTouchEvent* endEvents;

	/// Number of begin touch events
	int beginCount;

	/// Number of end touch events
	int endCount;
} b3SensorEvents;

/// A joint event is generated when a joint's reaction exceeds a threshold set
/// on it, which is how a game notices a joint being overloaded -- a suspension
/// bottoming out on a landing, a rope about to snap, a door being forced.
///
/// Both thresholds default to B3_NO_BOUND, so a joint reports nothing until one
/// is set with b3Joint_SetForceThreshold or b3Joint_SetTorqueThreshold. Either
/// one exceeding fires the event; the event says which.
typedef struct b3JointEvent
{
	/// Id of the joint. @warning may have been destroyed
	b3JointId jointId;

	/// The joint user data.
	void* userData;

	/// The magnitude of the constraint force at the moment it tripped, in
	/// newtons. This is the same quantity b3Joint_GetConstraintForce reports the
	/// direction of.
	b3f force;

	/// The magnitude of the constraint torque, in newton-metres.
	b3f torque;

	/// True when the force threshold was the one exceeded.
	bool forceExceeded;

	/// True when the torque threshold was the one exceeded. Both may be true.
	bool torqueExceeded;
} b3JointEvent;

/// Joint events buffered in the world, available after the step completes.
/// @note These may become invalid if bodies or joints are destroyed.
typedef struct b3JointEvents
{
	/// Array of joint events
	b3JointEvent* jointEvents;

	/// Number of joint events
	int jointCount;
} b3JointEvents;

/// Reported for a body moved by the simulation, not for one the user moved.
///
/// This is the efficient way to drive game object transforms: one contiguous
/// array holding only the bodies that actually moved, rather than a
/// b3Body_GetTransform call per entity. NEA_Phys3DSyncModels walks it.
typedef struct b3BodyMoveEvent
{
	/// The body user data.
	void* userData;

	/// The body transform.
	b3WorldTransform transform;

	/// The body id.
	b3BodyId bodyId;

	/// Did the body fall asleep this time step?
	bool fellAsleep;
} b3BodyMoveEvent;

/// Body events buffered in the world, available after the step completes.
/// @note This data becomes invalid if bodies are destroyed.
typedef struct b3BodyEvents
{
	/// Array of move events
	b3BodyMoveEvent* moveEvents;

	/// Number of move events
	int moveCount;
} b3BodyEvents;

/// The contact data for two shapes. By convention the manifold normal points
/// from shape A to shape B.
typedef struct b3ContactData
{
	/// The contact id. May become orphaned -- check b3Contact_IsValid before
	/// passing it anywhere.
	b3ContactId contactId;

	/// The first shape id.
	b3ShapeId shapeIdA;

	/// The second shape id.
	b3ShapeId shapeIdB;

	/// The contact manifolds. Points at internal data that may move; do not
	/// store this pointer.
	const struct b3Manifold* manifolds;

	/// The number of manifolds. Above one only for mesh collision.
	int manifoldCount;
} b3ContactData;

// =========================================================================
// Dynamic tree
// =========================================================================

/// Category bits matching every filter. Shared with b3Filter above; it lives
/// here because the dynamic tree reads it directly.
#define B3_DEFAULT_CATEGORY_BITS UINT64_MAX

/// Mask bits accepting every category.
#define B3_DEFAULT_MASK_BITS UINT64_MAX

typedef struct b3TreeNodeChildren
{
	int child1;
	int child2;
} b3TreeNodeChildren;

enum b3TreeNodeFlags
{
	b3_allocatedNode = 0x0001,
	b3_enlargedNode = 0x0002,
	b3_leafNode = 0x0004,
};

/// A node in the dynamic bounding volume tree.
typedef struct b3TreeNode
{
	/// The node bounding box.
	b3AABB aabb;

	/// Category bits for collision filtering.
	uint64_t categoryBits;

	union
	{
		/// Child indices for an internal node.
		b3TreeNodeChildren children;

		/// User data for a leaf node.
		uint64_t userData;
	};

	union
	{
		/// The node parent index, for an allocated node.
		int parent;

		/// The next free index, while the node is on the free list.
		int next;
	};

	/// Height of the node. Leaves have a height of 0.
	uint16_t height;

	/// See b3TreeNodeFlags.
	uint16_t flags;
} b3TreeNode;

/// The dynamic tree: a hierarchical AABB tree for broad-phase queries.
typedef struct b3DynamicTree
{
	b3TreeNode* nodes;

	int root;
	int nodeCount;
	int nodeCapacity;
	int freeList;
	int proxyCount;

	int* leafIndices;
	b3AABB* leafBoxes;
	b3Vec3* leafCenters;
	int rebuildCapacity;

	// Upstream also carries `binIndices` and a `version` stamp. Neither is
	// here: binIndices belongs to the binned-SAH rebuild heuristic, which is
	// dead upstream and is not ported (see dynamic_tree.c), and the version
	// exists for the tree serializer, which needs stdio. Keeping either would
	// mean an allocation of rebuildCapacity ints that nothing ever reads.
} b3DynamicTree;

/// Performance data returned by dynamic tree queries.
typedef struct b3TreeStats
{
	/// Number of internal nodes visited during the query.
	int nodeVisits;

	/// Number of leaf nodes visited during the query.
	int leafVisits;
} b3TreeStats;

/// Callback signature for b3DynamicTree_Query. Return false to terminate.
typedef bool b3TreeQueryCallbackFcn( int proxyId, uint64_t userData, void* context );

/// Callback signature for b3DynamicTree_QueryClosest. Receives the best
/// squared distance found so far and returns the caller's own, or the value it
/// was given to leave the search unchanged.
///
/// The distance is squared and therefore wide: a squared length is Q24, and at
/// the far end of the world it does not fit an int32. Nothing here is a b3f,
/// because narrowing it to Q12 would discard exactly the resolution the
/// nearest-neighbour comparison depends on.
typedef int64_t b3TreeQueryClosestCallbackFcn( int64_t distanceSqrMin, int proxyId, uint64_t userData, void* context );

/// Callback signature for ray casts against the tree. Return the new ray
/// clip fraction, or zero to terminate.
typedef b3c b3TreeRayCastCallbackFcn( const b3RayCastInput* input, int proxyId, uint64_t userData, void* context );

/// Callback signature for shape casts against the tree.
typedef b3c b3TreeShapeCastCallbackFcn( const b3ShapeCastInput* input, int proxyId, uint64_t userData, void* context );

/// Callback signature for b3DynamicTree_BoxCast. Distinct from the shape cast
/// above because the tree sweeps a box, not a proxy: it takes b3BoxCastInput.
typedef b3c b3TreeBoxCastCallbackFcn( const b3BoxCastInput* input, int proxyId, uint64_t userData, void* context );

// =========================================================================
// Distance and cast queries
// =========================================================================
//
// Implemented in distance.c. Declared here because the shape primitives call
// them for overlap tests, and they are the shared entry points continuous
// collision reuses.

/// Compute the closest points between two convex shapes, using GJK.
B3_API b3DistanceOutput b3ShapeDistance( const b3DistanceInput* input, b3SimplexCache* cache, b3Simplex* simplexes,
										 int simplexCapacity );

/// Cast one convex shape against another, using conservative advancement.
B3_API b3CastOutput b3ShapeCast( const b3ShapeCastPairInput* input );

/// Re-express a proxy in a frame, writing its points into `buffer`, which must
/// hold at least B3_MAX_SHAPE_CAST_POINTS of them.
B3_API b3ShapeProxy b3MakeLocalProxy( const b3ShapeProxy* proxy, b3Transform transform, b3Vec3* buffer );

/// The proxy's bounds, radius included.
B3_API b3AABB b3ComputeProxyAABB( const b3ShapeProxy* proxy );

/// The pose a sweep has reached at `time`, which must be in [0, 1].
B3_API b3Transform b3GetSweepTransform( const b3Sweep* sweep, b3c time );

/// The first time in [0, maxFraction] at which two moving convex shapes touch,
/// by root finding on a separating axis.
B3_API b3TOIOutput b3TimeOfImpact( const b3TOIInput* input );
