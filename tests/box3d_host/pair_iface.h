// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Warioware64, 2026

// The neutral interface between the fixed-point port and pristine float Box3D.
//
// Both libraries define b3Vec3, b3AABB, b3ShapeDistance and so on, with
// incompatible definitions -- one is float, the other is a set of fixed-point
// scales that are distinct struct types under B3_FIXED_STRICT. They therefore
// cannot appear in the same translation unit.
//
// So they never do. pair_ref.c sees only the upstream headers, pair_port.c
// sees only the port's, and this file -- which mentions neither -- is the only
// thing run_pair.c includes. Everything crossing the boundary is a plain
// double, which is wide enough to represent either side exactly.

#pragma once

#include <stdbool.h>

typedef struct
{
	double x, y, z;
} pdVec3;

typedef struct
{
	pdVec3 lower, upper;
} pdAABB;

/// A rigid transform. The quaternion is (v.xyz, s), matching both libraries.
typedef struct
{
	pdVec3 p;
	double qx, qy, qz, qw;
} pdTransform;

#define PD_MAX_POINTS 8
#define PD_MAX_MANIFOLD_POINTS 8
#define PD_MAX_TREE_RESULTS 256

/// A point cloud with a radius: one point is a sphere, two a capsule, eight a
/// box. The same shape both libraries use for a distance query.
typedef struct
{
	pdVec3 points[PD_MAX_POINTS];
	int count;
	double radius;
} pdProxy;

typedef struct
{
	double distance;
	pdVec3 pointA, pointB, normal;
} pdDistanceOut;

typedef struct
{
	bool hit;
	double fraction;
	pdVec3 point, normal;
} pdCastOut;

/// A sweep: where a body's centre of mass and rotation start and end.
///
/// Both libraries spell this the same way, so it crosses as five plain vectors
/// and two quaternions rather than needing a conversion either side.
typedef struct
{
	pdVec3 localCenter;
	pdVec3 c1, c2;
	double q1x, q1y, q1z, q1w;
	double q2x, q2y, q2z, q2w;
} pdSweep;

/// The state enumerators, in the order both libraries declare them, so the
/// integer crosses unchanged and a mismatch is a real disagreement about how
/// the query ended rather than a numbering difference.
typedef enum
{
	pd_toiUnknown = 0,
	pd_toiFailed,
	pd_toiOverlapped,
	pd_toiHit,
	pd_toiSeparated,
} pdTOIState;

typedef struct
{
	int state;
	double fraction;
	double distance;
	pdVec3 point, normal;

	/// What the root finder actually cost. Not compared -- the two libraries
	/// converge differently by construction -- but reported, because it is the
	/// only direct evidence about whether the iteration caps are a safety net
	/// or the thing doing the stopping.
	int distanceIterations, pushBackIterations, rootIterations;
} pdTOIOut;

typedef struct
{
	int pointCount;
	pdVec3 normal;
	pdVec3 points[PD_MAX_MANIFOLD_POINTS];
	double separations[PD_MAX_MANIFOLD_POINTS];

	/// Which separating feature produced this manifold, from the b3SATCache.
	/// Only the hull-versus-hull entry points fill these; everything else
	/// leaves them zero.
	///
	/// They exist because there is no threshold on the normal that separates
	/// "the two libraries picked different tied features" from "the two
	/// libraries rounded the same feature differently". For two nearly
	/// parallel faces those produce the same size of disagreement -- a
	/// fraction of a degree -- so the only honest way to tell them apart is to
	/// ask which feature won. Hull topology indices match one-for-one across
	/// the two libraries, because the baker copies the reference's arrays in
	/// order.
	int feature;
	int featureIndexA;
	int featureIndexB;

	/// Which feature of the triangle the manifold resolved against, as a
	/// b3TriangleFeature. Only the triangle entry points fill this.
	int triangleFeature;

	/// Per-point feature identifier, packed from the contact's b3FeaturePair
	/// the same way both libraries pack it for warm starting.
	///
	/// This is the finest-grained statement of "which features touched", and
	/// it sees a tie the b3SATCache cannot: the cache records the reference
	/// face and the support vertex, but *not* the incident face, and two faces
	/// of the incident hull can be exactly equally opposed -- an octagonal
	/// prism meeting a flat face square on has two. Matching points by
	/// identity rather than by proximity is also simply the right
	/// correspondence to compare over.
	unsigned featureIds[PD_MAX_MANIFOLD_POINTS];

	/// Which triangle of the mesh each point came from, or -1 for a convex
	/// pair. Filled only by the scene entry points.
	///
	/// A feature id is unique *within one triangle collide*, not across a mesh
	/// contact: two triangles can both report "reference face 0, incident
	/// vertex 2" and pack the same id. Both libraries therefore key their
	/// warm-start match on the pair, and so must any comparison -- matching on
	/// the id alone pairs points from different triangles and reports drifts of
	/// a whole unit for two contacts that are simply not the same contact.
	int triangleIndices[PD_MAX_MANIFOLD_POINTS];
} pdManifoldOut;

// --- convex hulls ---------------------------------------------------------
//
// A hull cannot cross the boundary the way a sphere can, because the two
// libraries store one differently and neither stores it as anything a plain
// double can carry. It also cannot be rebuilt independently on each side: the
// port has no hull builder at all -- hulls are built on the host at float
// precision and baked, which is the whole shape of the design.
//
// So the reference builds it, keeps it, and describes it here. The port bakes
// that description into a fixed-point blob. Any difference between the two
// sides is then quantization and nothing else, which is exactly the term
// run_pair is trying to measure.

#define PD_MAX_HULL_VERTICES 32
#define PD_MAX_HULL_FACES 32
#define PD_MAX_HULL_EDGES 128 ///< Half-edges, so twice the edge count.

typedef struct
{
	unsigned char next, twin, origin, face;
} pdHalfEdge;

typedef enum
{
	pd_hullBox,		 ///< params: hx, hy, hz
	pd_hullCylinder, ///< params: height, radius, yOffset, sides
	pd_hullCone,	 ///< params: height, radius1, radius2, slices
	pd_hullRock,	 ///< params: radius
} pdHullKind;

typedef struct
{
	/// Index into the reference's own table of built hulls. Opaque to
	/// run_pair and meaningless to the port.
	int refId;

	int vertexCount;
	int edgeCount; ///< Half-edge count.
	int faceCount;

	pdVec3 points[PD_MAX_HULL_VERTICES];
	unsigned char vertexEdge[PD_MAX_HULL_VERTICES];
	pdHalfEdge edges[PD_MAX_HULL_EDGES];
	unsigned char faceEdge[PD_MAX_HULL_FACES];
	pdVec3 planeNormal[PD_MAX_HULL_FACES];
	double planeOffset[PD_MAX_HULL_FACES];

	pdVec3 center;
	double volume;
	double surfaceArea;
	double innerRadius;

	/// Inertia about the centroid, **per unit mass**, column-major. The port
	/// stores hulls this way; the reference divides its own tensor by the
	/// volume to fill this in.
	double unitInertia[9];
} pdHull;

// --- triangle meshes ------------------------------------------------------
//
// Same arrangement as hulls, and for the same reason: the port has no mesh
// builder, so the reference builds one and describes it, and the port bakes
// that description.
//
// One difference matters when reading the results. Two baked hulls of the same
// description have the same topology, so a divergence can only be
// quantization. Two *meshes* do not: the reference splits its BVH with the
// surface area heuristic and the baker splits at the median, so the two trees
// have different shapes and different node counts. What must agree is the
// **set of triangles a query returns**, never the traversal that found them.

// Overridable from the command line, and 512 everywhere in this harness: a
// pdMesh is a by-value struct and run_pair holds several, so the caps are what
// keeps it a few hundred kilobytes. bake_ref compiles mesh_bake.c against
// these with a level's worth of triangles instead -- it is the same baker, it
// just does not carry run_pair's struct around.
#ifndef PD_MAX_MESH_VERTICES
#define PD_MAX_MESH_VERTICES 512
#endif

#ifndef PD_MAX_MESH_TRIANGLES
#define PD_MAX_MESH_TRIANGLES 512
#endif

// The b3MeshEdgeFlags bit layout, restated. run_pair.c sees neither library's
// headers, and both spell these the same way, so the values are part of the
// boundary rather than of either side.
// b3SeparatingFeature values, restated for the same reason.
#define PD_FACE_AXIS_A 2
#define PD_FACE_AXIS_B 3
#define PD_EDGE_PAIR_AXIS 4

#define PD_CONCAVE_EDGE1 0x01
#define PD_CONCAVE_EDGE2 0x02
#define PD_CONCAVE_EDGE3 0x04
#define PD_INVERSE_CONCAVE_EDGE1 0x10
#define PD_INVERSE_CONCAVE_EDGE2 0x20
#define PD_INVERSE_CONCAVE_EDGE3 0x40

typedef enum
{
	pd_meshGrid,   ///< params: halfWidth, halfDepth, divisions, yWave
	pd_meshRamp,   ///< params: halfWidth, halfDepth, height, divisions
	pd_meshStairs, ///< params: width, stepRun, stepRise, steps
	pd_meshBowl,   ///< params: radius, depth, rings, segments
} pdMeshKind;

typedef struct
{
	/// Index into the reference's own table of built meshes. Opaque to
	/// run_pair and meaningless to the port.
	int refId;

	int vertexCount;
	int triangleCount;

	pdVec3 vertices[PD_MAX_MESH_VERTICES];

	/// Three vertex indices per triangle, counter-clockwise.
	int indices[3 * PD_MAX_MESH_TRIANGLES];

	/// Local bounds over `vertices`, for placing probes.
	pdVec3 lower, upper;
} pdMesh;

typedef struct
{
	double mass;
	pdVec3 center;
	double unitInertia[9];
} pdMassOut;

// --- worlds and bodies ----------------------------------------------------
//
// Everything above compares one geometric query. This compares the *object
// model*: build the same world, the same body and the same shapes on both
// sides, and read back what each computed before anything is stepped.
//
// That is a different kind of comparison and it needs a different tolerance
// story. A manifold's drift is budgeted against a lever arm because the
// quantities are the output of an iterative search; these are closed-form
// computations of mass, centre of mass and bounds, so the comparison is a
// straight relative tolerance and the interesting number is the worst case
// actually observed.

#define PD_MAX_BODY_SHAPES 4

typedef enum
{
	pd_bodyShapeSphere,	 ///< p1 is the centre, radius the radius
	pd_bodyShapeCapsule, ///< p1 and p2 are the segment ends
	pd_bodyShapeHull,	 ///< hullIndex selects from the hulls passed alongside
	pd_bodyShapeMesh,	 ///< meshIndex selects from the meshes passed alongside
} pdBodyShapeKind;

typedef struct
{
	pdBodyShapeKind kind;
	pdVec3 p1, p2;
	double radius;
	double density;
	int hullIndex;

	/// For pd_bodyShapeMesh. The mesh is not scaled: both libraries apply
	/// b3SafeScale to (1,1,1) and get it back, so a scale here would only add a
	/// quantization difference to a comparison that is about the narrow phase.
	int meshIndex;

	/// Make this shape a sensor: it reports what overlaps it and resolves
	/// nothing. Zero for every scene predating Phase 7 Stage 3, which is why
	/// they are unaffected by the field existing.
	bool isSensor;

	/// Sensor events on this shape. Both ends need it -- a sensor with it off
	/// reports nothing and an ordinary shape with it off is invisible to every
	/// sensor -- so a sensor scene sets it on the sensor *and* on each visitor
	/// it expects to see.
	bool enableSensorEvents;
} pdBodyShape;

typedef struct
{
	pdTransform xf;
	int shapeCount;
	pdBodyShape shapes[PD_MAX_BODY_SHAPES];
} pdBodyDesc;

typedef struct
{
	double mass;
	pdVec3 localCenter;
	pdVec3 worldCenter;

	/// Per unit mass on both sides. The reference divides its absolute tensor
	/// by the mass to fill this in, exactly as it does for pdHull.
	double unitInertia[9];

	/// The world-space inverse inertia, which is where the port's Q24 matrix
	/// and its Q30 rotation differ from float most visibly.
	double invInertiaWorld[9];

	double invMass;
	double minExtent;
	pdVec3 maxExtent;

	int shapeCount;
	pdAABB aabb[PD_MAX_BODY_SHAPES];
	pdAABB fatAABB[PD_MAX_BODY_SHAPES];
} pdBodyOut;

// --- scripted scenes ------------------------------------------------------
//
// pdBodyDesc compares what a world computes about one body standing still.
// This compares what a world computes about bodies *interacting*: which pairs
// the broad phase finds, which of them the narrow phase says are touching,
// what manifold it builds, and which begin and end touch events come out.
//
// A pass has two halves. It teleports zero or more bodies, and then it either
// runs a single collide pass (`stepCount == 0`) or advances the simulation by
// `stepCount` real steps. The first form is what every scene wrote before
// Phase 3C-ii: motion scripted rather than simulated, because the port had no
// solver, so both libraries saw byte-identical inputs each pass and any
// difference was the collision tier's alone.
//
// The second form is the point of the `step` scenario. Inputs are identical
// only on the *first* step; after that each library is integrating its own
// state forward and the comparison is of two trajectories that are free to
// drift apart. That is a weaker guarantee and a slower-failing one, which is
// why it did not exist until there was a solver to justify it -- and why the
// scenario begins with bodies that move but do not touch, so that a divergence
// is attributable to the integrator 3C-i already verified in closed form
// before contacts are allowed to enter.
//
// The reference's b3Collide is static, so the reference side is driven through
// b3World_Step in both forms: with a time step of *zero* for a collide-only
// pass, which upstream handles explicitly (inv_dt = h = inv_h = 0) so it runs
// the broad phase, the narrow phase and the contact-state pass and integrates
// nothing; and with 1/60 for a stepping pass, matching the port's compile-time
// B3_NEA_STEP_HZ, which is why the port's b3World_Step takes no time step at
// all.

#define PD_MAX_SCENE_BODIES 6
#define PD_MAX_SCENE_PASSES 8
#define PD_MAX_SCENE_CONTACTS 12

/// Manifolds one contact may report. B3_NEA_MAX_MESH_MANIFOLDS on the port
/// side; the reference is unbounded, so a scene that produced more would be
/// reporting a cluster count the port cannot represent -- which the comparison
/// counts rather than hides.
#define PD_MAX_SCENE_MANIFOLDS 8

typedef struct
{
	pdBodyDesc body;
	bool isStatic;

	/// Initial velocity, applied once after creation. Lets a scene put bodies
	/// in motion without gravity, which is how the non-touching cases move.
	pdVec3 linearVelocity;
	pdVec3 angularVelocity;

	/// Keep this body awake regardless of how still it becomes.
	///
	/// Spelled as a *disable* so that zero is the existing behaviour and no
	/// scene predating it changes. It exists because sleeping freezes a body's
	/// warm-started accumulators at whatever they held on the last solved
	/// step, and the two libraries have no reason to sleep on the same step --
	/// `awake` is reported rather than compared for exactly that reason. A
	/// comparison of *accumulators* (a joint reaction force, say) between one
	/// library that has converged and another that froze mid-settle is
	/// therefore a comparison of two different moments, and reads as a large
	/// systematic error when nothing is wrong. Any scene comparing an
	/// accumulator at rest must set this.
	bool disableSleep;
} pdSceneBody;

typedef struct
{
	int bodyIndex;
	pdTransform xf;
} pdSceneMove;

/// Joints a scene may build. Each joint type adds one as it lands.
typedef enum
{
	pd_jointDistance,
	pd_jointRevolute,
	pd_jointSpherical,
	pd_jointWeld,
	pd_jointMotor,
	pd_jointPrismatic,
	pd_jointParallel,
	pd_jointWheel,
} pdSceneJointKind;

#define PD_MAX_SCENE_JOINTS 6

/// One joint in a scripted scene.
///
/// Anchors are the *origins of the joint frames*, in each body's local space
/// and measured from the body origin -- which is where b3JointDef puts them,
/// deliberately, so that adding a shape and moving the centre of mass does not
/// silently move the joint. Both libraries are handed the same numbers.
typedef struct
{
	pdSceneJointKind kind;
	int bodyA, bodyB;
	pdVec3 localAnchorA;
	pdVec3 localAnchorB;
	bool collideConnected;

	// pd_jointDistance
	double length;
	bool enableSpring;
	double hertz;
	double dampingRatio;
	double lowerSpringForce, upperSpringForce;
	bool enableLimit;
	double minLength, maxLength;
	bool enableMotor;
	double maxMotorForce, motorSpeed;

	// pd_jointRevolute.
	//
	// Angles are **degrees** here, not the port's brads and not upstream's
	// radians. Neither library's unit is neutral, so the description picks a
	// third that both convert from -- which also means a scene reads as the
	// hinge a person would describe rather than as 4096 of something.
	double lowerAngleDeg, upperAngleDeg, targetAngleDeg;
	bool enableAngleLimit;
	bool enableAngleMotor;
	bool enableAngleSpring;
	double angleHertz, angleDampingRatio;

	/// Motor speed in radians per second -- an angular velocity, so unlike the
	/// angles above it is the same number on both sides.
	double motorAngularSpeed;
	double maxMotorTorque;

	// pd_jointSpherical.
	//
	// Angles are degrees here for the same reason the revolute's are. The cone
	// angle is a half-angle measured from frame A's z axis and is unsigned; the
	// twist limits are signed and measured about it.
	double coneAngleDeg;
	double lowerTwistDeg, upperTwistDeg;
	bool enableConeLimit;
	bool enableTwistLimit;
	bool enableBallSpring;
	double ballHertz, ballDampingRatio;

	/// Motor angular velocity in rad/s, per axis. Unlike the revolute's scalar
	/// motor a ball joint drives all three rotational degrees at once, which is
	/// the thing these scenes exist to exercise.
	bool enableBallMotor;
	pdVec3 ballMotorVelocity;
	double ballMaxMotorTorque;

	// pd_jointWeld.
	//
	// No angles at all: a weld locks the whole relative transform, so there is
	// nothing to express in degrees and nothing to convert. Zero hertz on
	// either half means that half is rigid rather than a spring, on both sides.
	double weldLinearHertz, weldLinearDampingRatio;
	double weldAngularHertz, weldAngularDampingRatio;

	// pd_jointMotor.
	//
	// Velocities and force bounds, all in SI on both sides -- so unlike the
	// revolute's and the ball joint's angles there is no unit question here.
	//
	// Every bound defaults to zero and a zero bound disables its branch, which
	// is upstream's behaviour and the port's, so a scene opts each drive in by
	// giving it a budget.
	pdVec3 motorLinearVelocity;
	pdVec3 motorAngularVelocity;
	double motorMaxVelocityForce, motorMaxVelocityTorque;
	double motorLinearHertz, motorLinearDampingRatio;
	double motorAngularHertz, motorAngularDampingRatio;
	double motorMaxSpringForce, motorMaxSpringTorque;

	// pd_jointPrismatic.
	//
	// **The one joint whose description needs no angle unit at all.** Every
	// field is a length in metres or a speed in metres per second, identical on
	// both sides -- where the revolute's limits are degrees here and brads in
	// the port, and the ball joint's are degrees and brads too. A slider's
	// translation is a length in both libraries, so nothing is converted.
	double slideTargetTranslation;
	double slideLowerTranslation, slideUpperTranslation;
	double slideHertz, slideDampingRatio;
	double slideMotorSpeed, slideMaxMotorForce;
	int enableSlideSpring, enableSlideLimit, enableSlideMotor;

	// pd_jointParallel.
	//
	// Three numbers and no unit conversion anywhere, for a different reason
	// than the slider's: two are a frequency and a dimensionless ratio, and the
	// third is a torque budget. The joint has no angle in its *description* at
	// all -- the angle it acts on is the relative rotation the scene's body
	// orientations already imply.
	//
	// `parallelMaxTorque` bounds the impulse coefficient rather than the torque
	// in newton-metres, identically on both sides, so it needs no conversion
	// either. See b3ParallelJointDef::maxTorque.
	double parallelHertz, parallelDampingRatio, parallelMaxTorque;

	// pd_jointWheel.
	//
	// The suspension fields are all **metres and metres per second** -- the
	// joint's linear half is a prismatic and needs no conversion. The steering
	// limits and target are **degrees** here, a third unit neither library
	// uses, exactly as the revolute's and the ball joint's angles are: the port
	// wants brads and the reference wants radians, so the scene description
	// commits to neither.
	//
	// `wheelSpinSpeed` is a *velocity* in rad/s and so is unit-neutral, the
	// same call the revolute's motor speed makes.
	//
	// **`wheelLowerSuspensionLimit` is held at zero in every scene**, and that
	// is deliberate rather than incidental: upstream's b3GetWheelJointForce
	// adds the lower *limit* where the lower *impulse* is meant, which is the
	// one wheel defect this harness could see. Keeping it at zero leaves the
	// defect dormant on both sides so the comparison stays about the solver,
	// and test_wheel_joint_reaction_ignores_limits is what settles it instead.
	double wheelSuspensionHertz, wheelSuspensionDampingRatio;
	double wheelLowerSuspensionLimit, wheelUpperSuspensionLimit;
	double wheelSpinSpeed, wheelMaxSpinTorque;
	double wheelSteeringHertz, wheelSteeringDampingRatio;
	double wheelTargetSteeringDeg, wheelLowerSteeringDeg, wheelUpperSteeringDeg;
	double wheelMaxSteeringTorque;
	int enableWheelSuspensionSpring, enableWheelSuspensionLimit;
	int enableWheelSpinMotor, enableWheelSteering, enableWheelSteeringLimit;
} pdSceneJoint;

/// What one joint looks like at the end of a pass.
///
/// The current length is the constraint's own error measure -- what the joint
/// is holding versus what it was asked to hold -- and the reaction force is
/// the accumulated impulse the solver reached, which is the one number that
/// exposes a fixed-point impulse scale directly rather than through a
/// trajectory.
typedef struct
{
	double currentLength;
	pdVec3 force;

	/// Hinge angle in degrees, and the torque the joint is applying. Zero for
	/// a joint that has neither.
	double angleDeg;
	pdVec3 torque;

	/// The ball joint's two angles, in degrees. A spherical joint has no single
	/// angle, so it reports both rather than reusing angleDeg for one of them.
	double coneAngleDeg;
	double twistAngleDeg;

	/// The slider's translation in metres and its speed in metres per second.
	/// Both are lengths, so both compare directly with no unit conversion.
	double translation;
	double slideSpeed;

	/// The wheel's suspension travel in metres and its spin rate in radians per
	/// second -- both unit-neutral -- plus its steering angle in degrees.
	double suspensionTranslation;
	double spinSpeed;
	double steeringAngleDeg;
} pdSceneJointOut;

typedef struct
{
	int moveCount;
	pdSceneMove moves[PD_MAX_SCENE_BODIES];

	/// Steps to advance after the moves land. Zero runs one collide pass and
	/// integrates nothing, which is what every scene predating the solver
	/// wants and gets from a zeroed descriptor.
	int stepCount;
} pdScenePass;

typedef struct
{
	/// Zero on both sides unless a scene asks otherwise, so a scene that only
	/// teleports is unaffected by the field existing.
	pdVec3 gravity;

	/// Sub-steps per step. Zero means the port's default of four; the
	/// reference is driven with whatever this resolves to, since sub-step
	/// count changes the answer on both sides.
	int subStepCount;

	int bodyCount;
	pdSceneBody bodies[PD_MAX_SCENE_BODIES];

	/// Created after every body, before the first pass. Zero for every scene
	/// that predates Phase 6 Stage 2, which is why they are unaffected by the
	/// field existing.
	int jointCount;
	pdSceneJoint joints[PD_MAX_SCENE_JOINTS];

	int passCount;
	pdScenePass passes[PD_MAX_SCENE_PASSES];
} pdSceneDesc;

typedef struct
{
	/// Creation-order shape indices, resolved from the shape ids each library
	/// handed back rather than assumed to be the ids themselves.
	int shapeA, shapeB;

	bool touching;

	/// Manifolds carrying points. One for a convex pair; one per normal
	/// *cluster* for a mesh contact, which is why this is an array -- a box
	/// wedged into a mesh corner produces three, and reporting only the first
	/// would compare a third of the contact.
	///
	/// Their order is each library's cluster creation order and the two have no
	/// reason to agree on it, so a comparison must pair them up rather than
	/// index them. Empty when not touching. Anchors are relative to each body's
	/// centre of mass, which is the frame the solver will read them in.
	int manifoldCount;
	pdManifoldOut manifolds[PD_MAX_SCENE_MANIFOLDS];
} pdSceneContact;

/// What one body looks like at the end of a pass. Unlike the contacts, these
/// are reported in creation order and never sorted: a body's identity is the
/// index the scene gave it.
typedef struct
{
	pdTransform xf;
	pdVec3 linearVelocity;
	pdVec3 angularVelocity;

	/// Reported rather than compared. Two libraries can legitimately settle on
	/// different steps, and a scenario that asserted this would be asserting
	/// the sleep threshold rather than the solver.
	bool awake;
} pdSceneBodyOut;

/// One sensor transition, by creation-order shape index on both ends.
///
/// Unlike a contact this carries no geometry at all, and that is the point:
/// a sensor's whole output is *which shapes* crossed its boundary and *when*.
/// There is nothing here to budget a tolerance against -- the two libraries
/// either name the same pair on the same pass or they disagree.
typedef struct
{
	int sensorShape;
	int visitorShape;
} pdSensorEvent;

#define PD_MAX_SCENE_SENSOR_EVENTS 12

typedef struct
{
	int contactCount;

	/// Sorted by (shapeA, shapeB). Discovery order depends on the shape of the
	/// dynamic tree, which the two libraries have no reason to agree on;
	/// sorting makes the comparison independent of it without hiding a
	/// genuinely different *set* of pairs.
	pdSceneContact contacts[PD_MAX_SCENE_CONTACTS];

	int beginCount;
	int endCount;

	/// Sensor transitions from this pass, each sorted by
	/// (sensorShape, visitorShape) for the reason the contacts are: the port
	/// publishes them in sensor order and then in shapeId order, the reference
	/// in its own, and neither promises the other's.
	int sensorBeginCount;
	pdSensorEvent sensorBegins[PD_MAX_SCENE_SENSOR_EVENTS];

	int sensorEndCount;
	pdSensorEvent sensorEnds[PD_MAX_SCENE_SENSOR_EVENTS];

	/// Shapes inside each sensor at the end of the pass, in creation order of
	/// the sensor. -1 for a shape that is not a sensor, so a scene can tell
	/// "empty" from "not a sensor" without a second array.
	int sensorOccupancy[PD_MAX_SCENE_BODIES * PD_MAX_BODY_SHAPES];

	/// Post-pass body states, in creation order. Filled by every scene; only
	/// the stepping ones have anything interesting in them.
	int bodyCount;
	pdSceneBodyOut bodies[PD_MAX_SCENE_BODIES];

	/// Post-pass joint states, in creation order.
	int jointCount;
	pdSceneJointOut joints[PD_MAX_SCENE_JOINTS];
} pdScenePassOut;

typedef struct
{
	int passCount;
	pdScenePassOut passes[PD_MAX_SCENE_PASSES];
} pdSceneOut;

/// Build a hull with the reference's float hull builder and describe it.
/// Returns false if the kind is not supported or the hull exceeds the port's
/// configured limits.
bool pdRefMakeHull( pdHullKind kind, const double* params, pdHull* out );

/// Generate a mesh and describe it. Returns false if the kind is not supported
/// or the mesh exceeds PD_MAX_MESH_*.
///
/// Unlike pdRefMakeHull this generates geometry rather than running a builder
/// -- the triangles are a plain function of the parameters, so both sides could
/// in principle produce them. It stays on the reference side anyway, so that
/// the reference's own b3CreateMesh sees exactly the vertices the baker does.
bool pdRefMakeMesh( pdMeshKind kind, const double* params, pdMesh* out );

/// Tree results are reported as the user data each proxy was created with,
/// never as proxy ids. The two libraries allocate nodes independently and
/// there is no reason their indices should agree; comparing ids would be
/// asserting an implementation detail neither side promises.
typedef struct
{
	int count;
	int userData[PD_MAX_TREE_RESULTS];
} pdTreeOut;

/// One implementation per library. run_pair drives both through this and never
/// learns which is which beyond the name.
typedef struct
{
	const char* name;

	void ( *distance )( const pdProxy* a, const pdProxy* b, const pdTransform* xf, bool useRadii, pdDistanceOut* out );

	void ( *shapeCast )( const pdProxy* a, const pdProxy* b, const pdTransform* xf, pdVec3 translation, pdCastOut* out );

	/// The first time two swept convex shapes touch.
	void ( *timeOfImpact )( const pdProxy* a, const pdProxy* b, const pdSweep* sweepA, const pdSweep* sweepB,
							double maxFraction, pdTOIOut* out );

	void ( *sphereSphere )( pdVec3 cA, double rA, pdVec3 cB, double rB, const pdTransform* xf, pdManifoldOut* out );

	void ( *capsuleSphere )( pdVec3 a1, pdVec3 a2, double rA, pdVec3 cB, double rB, const pdTransform* xf,
							 pdManifoldOut* out );

	void ( *capsuleCapsule )( pdVec3 a1, pdVec3 a2, double rA, pdVec3 b1, pdVec3 b2, double rB, const pdTransform* xf,
							  pdManifoldOut* out );

	void ( *treeQuery )( const pdAABB* boxes, int n, pdAABB query, pdTreeOut* out );

	void ( *treeRayCast )( const pdAABB* boxes, int n, pdVec3 origin, pdVec3 translation, pdTreeOut* out );

	void ( *hullMass )( const pdHull* hull, double density, pdMassOut* out );

	void ( *hullAABB )( const pdHull* hull, const pdTransform* xf, pdAABB* out );

	void ( *hullRayCast )( const pdHull* hull, pdVec3 origin, pdVec3 translation, pdCastOut* out );

	void ( *hullShapeCast )( const pdHull* hull, const pdProxy* b, pdVec3 translation, pdCastOut* out );

	bool ( *hullOverlap )( const pdHull* hull, const pdTransform* xf, const pdProxy* b );

	void ( *hullSphere )( const pdHull* hull, pdVec3 cB, double rB, const pdTransform* xf, pdManifoldOut* out );

	void ( *hullCapsule )( const pdHull* hull, pdVec3 b1, pdVec3 b2, double rB, const pdTransform* xf,
						   pdManifoldOut* out );

	/// Hull versus hull, one shot with a cold separating-axis cache.
	void ( *hullHull )( const pdHull* a, const pdHull* b, const pdTransform* xf, pdManifoldOut* out );

	/// Two consecutive calls sharing one separating-axis cache, reporting
	/// whether each took the cached path.
	///
	/// This exists because a manifold-only comparison cannot see the cache at
	/// all: a port whose slop window never accepts a re-use still produces the
	/// right manifold every time, through the full test, and run_pair would be
	/// green while half of b3CollideHulls had never executed. Comparing hit
	/// *rates* is what makes that visible.
	void ( *hullHullCached )( const pdHull* a, const pdHull* b, const pdTransform* xf1, const pdTransform* xf2,
							  pdManifoldOut* out1, pdManifoldOut* out2, bool* hit1, bool* hit2 );

	/// Collide a triangle and a sphere. The triangle is three points in its
	/// own frame; the sphere centre is in that frame too, so unlike the hull
	/// entry points there is no transform.
	void ( *triangleSphere )( const pdVec3 tri[3], pdVec3 center, double radius, pdManifoldOut* out );

	/// Collide a triangle and a capsule, in the triangle's frame.
	///
	/// `reuse` runs the call twice with one shared simplex cache and reports
	/// the second manifold, so the scenario can see the warm-started path --
	/// a cache that never hits still produces the right answer through the
	/// cold path, and a manifold-only comparison would never notice.
	void ( *triangleCapsule )( const pdVec3 tri[3], pdVec3 c1, pdVec3 c2, double radius, bool reuse,
							   pdManifoldOut* out );

	/// Collide a triangle and a hull, both in the triangle's frame.
	///
	/// `reuse` runs the call twice sharing one b3SATCache and reports the
	/// second result, plus whether the replay hit. A cache that never hits
	/// still produces the right manifold through the full test, so the hit
	/// *rate* is the only thing that can see the replay switch execute.
	void ( *triangleHull )( const pdVec3 tri[3], const pdHull* hull, const pdTransform* xf, bool reuse,
							pdManifoldOut* out, bool* cacheHit );

	/// The mesh's world bounds under a transform and a scale.
	void ( *meshAABB )( const pdMesh* mesh, const pdTransform* xf, pdVec3 scale, pdAABB* out );

	/// Triangle indices whose triangles overlap the given local-space box, in
	/// the order the library's traversal emitted them.
	///
	/// The order is reported rather than sorted away on purpose: emitting
	/// ascending indices is an invariant the mesh narrow phase depends on, and
	/// a comparison that sorted first could not see it break.
	///
	/// @return the number found, which may exceed `capacity`; only the first
	///         `capacity` are written.
	int ( *meshQuery )( const pdMesh* mesh, pdVec3 lower, pdVec3 upper, pdVec3 scale, int* indices, int capacity );

	/// One triangle's vertices and edge flags, with scale applied.
	///
	/// `flags` uses the b3MeshEdgeFlags bit layout, which both libraries share.
	/// @return false if the mesh could not be realized on this side.
	bool ( *meshTriangle )( const pdMesh* mesh, int triangleIndex, pdVec3 scale, pdVec3 out[3], int* flags );

	/// Create a world, create one dynamic body from `desc`, and report what
	/// the library computed for it. The world is destroyed before returning,
	/// so each call is independent and a leak shows up as a growing heap.
	void ( *worldBody )( const pdBodyDesc* desc, const pdHull* hulls, int hullCount, pdBodyOut* out );

	/// Build a world from `desc`, then run one collide pass per scripted pass,
	/// reporting the contacts and events each produced. The world is destroyed
	/// before returning.
	///
	/// `meshes` backs pd_bodyShapeMesh the way `hulls` backs pd_bodyShapeHull.
	/// A scene with no mesh shapes passes NULL and 0.
	void ( *worldScene )( const pdSceneDesc* desc, const pdHull* hulls, int hullCount, const pdMesh* meshes, int meshCount,
						  pdSceneOut* out );

} pdBackend;

extern const pdBackend pdRefBackend;  ///< pristine float Box3D
extern const pdBackend pdPortBackend; ///< the fixed-point port
