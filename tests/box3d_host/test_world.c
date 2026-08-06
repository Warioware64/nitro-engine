// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Warioware64
//
// This file is part of Nitro Engine Advanced

// Verification of the Phase 3A object model: worlds, bodies, shapes, solver
// sets and islands.
//
// Nothing here steps the world, because Phase 3A has no solver. What it has
// instead is upstream's own oracle: b3ValidateSolverSets, b3ValidateConnectivity
// and b3ValidateBroadPhase assert that the sets, islands, id pools and linked
// lists agree with each other. Those are called after *every* mutation below,
// not once at the end, so a failure names the operation that broke the
// structure rather than the one that noticed.
//
// The mass tests are the other half. They check the round trip
// invInertiaLocal * inertia == identity, which is what catches a scale error
// in b3InvertInertia, and they assert the two documented limits -- the Q24
// resolution floor for a very heavy body, and the uniform down-scale for a
// body too small for Q7.24 to hold its inverse -- rather than only describing
// them.

#include "body.h"
#include "broad_phase.h"
#include "constraint_graph.h"
#include "contact.h"
#include "core.h"
#include "island.h"
#include "joint.h"
#include "mesh_bake.h"
#include "mesh_contact.h"
#include "physics_world.h"
#include "shape.h"
#include "solver.h"
#include "solver_set.h"

#include "box3d/base.h"
#include "box3d/collision.h"
#include "box3d/types.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "assert_trap.h"

static int s_failures = 0;
static int s_checks = 0;

static void check( const char* what, bool ok )
{
	s_checks++;
	if ( !ok )
	{
		printf( "  FAIL %s\n", what );
		s_failures++;
	}
}

static void expect( const char* what, double got, double want, double tol )
{
	s_checks++;
	if ( fabs( got - want ) > tol )
	{
		printf( "  FAIL %-46s got %-14.9g want %-14.9g\n", what, got, want );
		s_failures++;
	}
}

static void checkInt( const char* what, long long got, long long want )
{
	s_checks++;
	if ( got != want )
	{
		printf( "  FAIL %-46s got %lld want %lld\n", what, got, want );
		s_failures++;
	}
}

static b3Vec3 V( double x, double y, double z )
{
	return b3MakeVec3( b3fFromDouble( x ), b3fFromDouble( y ), b3fFromDouble( z ) );
}

static double F( b3f x )
{
	return (double)b3Raw( x ) / (double)B3_F_ONE;
}

static double W( b3iw x )
{
	return (double)b3Raw( x ) / (double)B3_W_ONE;
}

/// Run both structural validators. Compiled to nothing without assertions, so
/// the host build has to be an assert build for this file to mean anything --
/// which B3_ENABLE_VALIDATION under NEA_DEBUG gives us.
static void validate( b3World* world )
{
	b3ValidateSolverSets( world );
	b3ValidateConnectivity( world );
	b3ValidateContacts( world );
	b3ValidateBroadPhase( &world->broadPhase );
}

static b3WorldId makeWorld( void )
{
	b3WorldDef def = b3DefaultWorldDef();
	def.capacity.staticBodyCount = 8;
	def.capacity.dynamicBodyCount = 16;
	def.capacity.staticShapeCount = 8;
	def.capacity.dynamicShapeCount = 16;
	def.capacity.contactCount = 32;
	def.capacity.jointCount = 16;

	// The contact cases move bodies by teleport and assert on what the collide
	// pass makes of the result. Gravity would add motion they do not model, so
	// it is off here; the ballistics cases set it explicitly.
	def.gravity = V( 0, 0, 0 );

	return b3CreateWorld( &def );
}

// =========================================================================
// World lifetime
// =========================================================================

static void test_world_lifetime( void )
{
	printf( "world lifetime\n" );

	b3WorldId worldId = makeWorld();
	check( "world id is non-null", B3_IS_NON_NULL( worldId ) );
	check( "world is valid", b3World_IsValid( worldId ) );

	b3World* world = b3GetWorldFromId( worldId );
	validate( world );

	// The three fixed sets exist and are in the order b3SetType names. Every
	// set-index comparison in the engine depends on this.
	checkInt( "three solver sets", world->solverSets.count, 3 );
	checkInt( "static set index", world->solverSets.data[b3_staticSet].setIndex, b3_staticSet );
	checkInt( "disabled set index", world->solverSets.data[b3_disabledSet].setIndex, b3_disabledSet );
	checkInt( "awake set index", world->solverSets.data[b3_awakeSet].setIndex, b3_awakeSet );

	checkInt( "no bodies yet", world->bodies.count, 0 );
	checkInt( "no awake bodies", b3World_GetAwakeBodyCount( worldId ), 0 );

	// Tuning round trips.
	b3World_SetGravity( worldId, V( 0, -20, 0 ) );
	expect( "gravity y", F( b3World_GetGravity( worldId ).y ), -20.0, 1e-3 );

	b3World_SetMaximumLinearSpeed( worldId, b3fFromInt( 100 ) );
	expect( "max linear speed", F( b3World_GetMaximumLinearSpeed( worldId ) ), 100.0, 1e-3 );

	b3World_EnableSleeping( worldId, false );
	check( "sleeping disabled", b3World_IsSleepingEnabled( worldId ) == false );
	b3World_EnableSleeping( worldId, true );
	check( "sleeping enabled", b3World_IsSleepingEnabled( worldId ) );

	b3DestroyWorld( worldId );

	// The generation advanced, so the old id is detectably stale.
	check( "destroyed world id is stale", b3World_IsValid( worldId ) == false );
}

// =========================================================================
// Mass properties
// =========================================================================

/// Worst absolute entry of ( mass * unitInertia ) * invInertiaLocal - I.
///
/// This is the round trip that matters: it is insensitive to how the tensor is
/// represented and sensitive to every scale error in forming or inverting it.
static double inertiaRoundTrip( b3Matrix3 unitInertia, b3f mass, b3MatrixW inv )
{
	double I[3][3] = {
		{ F( unitInertia.cx.x ), F( unitInertia.cy.x ), F( unitInertia.cz.x ) },
		{ F( unitInertia.cx.y ), F( unitInertia.cy.y ), F( unitInertia.cz.y ) },
		{ F( unitInertia.cx.z ), F( unitInertia.cy.z ), F( unitInertia.cz.z ) },
	};
	double J[3][3] = {
		{ W( inv.cx.x ), W( inv.cy.x ), W( inv.cz.x ) },
		{ W( inv.cx.y ), W( inv.cy.y ), W( inv.cz.y ) },
		{ W( inv.cx.z ), W( inv.cy.z ), W( inv.cz.z ) },
	};
	double m = F( mass );

	double worst = 0.0;
	for ( int r = 0; r < 3; ++r )
	{
		for ( int c = 0; c < 3; ++c )
		{
			double s = 0.0;
			for ( int k = 0; k < 3; ++k )
			{
				s += m * I[r][k] * J[k][c];
			}
			double err = fabs( s - ( r == c ? 1.0 : 0.0 ) );
			if ( err > worst )
			{
				worst = err;
			}
		}
	}

	return worst;
}

static void test_steiner( void )
{
	printf( "parallel axis theorem\n" );

	// b3SteinerUnit returned only its diagonal until Phase 3A, and every test
	// through Phase 2 passed anyway: its sole caller was b3ComputeCapsuleMass,
	// which shifts along the capsule axis in a frame where that axis is a
	// coordinate axis, so two components are zero and all three off-diagonal
	// terms vanish. Nothing noticed until a body accumulated shapes at an
	// arbitrary offset from its centre of mass.
	//
	// So the off-diagonal terms are asserted directly, on an offset with three
	// non-zero components, rather than only through a body.
	{
		b3Vec3 d = V( 0.5, -0.25, 0.75 );
		b3Matrix3 s = b3SteinerUnit( d );

		expect( "steiner xx = y^2 + z^2", F( s.cx.x ), 0.0625 + 0.5625, 1e-4 );
		expect( "steiner yy = x^2 + z^2", F( s.cy.y ), 0.25 + 0.5625, 1e-4 );
		expect( "steiner zz = x^2 + y^2", F( s.cz.z ), 0.25 + 0.0625, 1e-4 );

		expect( "steiner xy = -x*y", F( s.cy.x ), 0.125, 1e-4 );
		expect( "steiner xz = -x*z", F( s.cz.x ), -0.375, 1e-4 );
		expect( "steiner yz = -y*z", F( s.cz.y ), 0.1875, 1e-4 );

		check( "steiner is symmetric", b3Raw( s.cy.x ) == b3Raw( s.cx.y ) && b3Raw( s.cz.x ) == b3Raw( s.cx.z ) &&
										   b3Raw( s.cz.y ) == b3Raw( s.cy.z ) );
	}

	// An offset along a single axis has no off-diagonal terms at all. This is
	// the configuration that hid the bug, so it is worth stating that it is
	// genuinely diagonal rather than merely untested.
	{
		b3Matrix3 s = b3SteinerUnit( V( 0, 1.5, 0 ) );
		check( "axis-aligned offset stays diagonal",
			   b3Raw( s.cy.x ) == 0 && b3Raw( s.cz.x ) == 0 && b3Raw( s.cz.y ) == 0 );
		expect( "axis-aligned xx", F( s.cx.x ), 2.25, 1e-4 );
		expect( "axis-aligned yy", F( s.cy.y ), 0.0, 1e-6 );
	}
}

static void test_invert_inertia( void )
{
	printf( "b3InvertInertia\n" );

	// Unit sphere, unit mass. The inverse is 2.5 up to the Q12 quantization of
	// the unit tensor itself (0.4 is 1638/4096 = 0.39990).
	{
		b3Matrix3 u = b3SphereUnitInertia( b3fFromInt( 1 ) );
		b3f mass = b3fFromInt( 1 );
		b3MatrixW inv = b3InvertInertia( u, mass );
		expect( "sphere r=1 m=1 inverse", W( inv.cx.x ), 2.5, 2e-3 );
		check( "sphere r=1 m=1 isotropic", b3Raw( inv.cx.x ) == b3Raw( inv.cy.y ) && b3Raw( inv.cy.y ) == b3Raw( inv.cz.z ) );
		check( "sphere r=1 m=1 no off-diagonal", b3Raw( inv.cy.x ) == 0 && b3Raw( inv.cz.x ) == 0 && b3Raw( inv.cz.y ) == 0 );
		expect( "sphere r=1 m=1 round trip", inertiaRoundTrip( u, mass, inv ), 0.0, 1e-6 );
	}

	// 2x2x2 box, mass 8. Unit inertia 8/12, inverse 0.1875.
	{
		b3Matrix3 u = b3BoxUnitInertia( V( -1, -1, -1 ), V( 1, 1, 1 ) );
		b3f mass = b3fFromInt( 8 );
		b3MatrixW inv = b3InvertInertia( u, mass );
		expect( "box 2^3 m=8 inverse", W( inv.cx.x ), 0.1875, 1e-4 );
		expect( "box 2^3 m=8 round trip", inertiaRoundTrip( u, mass, inv ), 0.0, 1e-6 );
	}

	// A non-diagonal tensor: a 4x2x2 box rotated 45 degrees about z. The two
	// short axes are equal and the long one is not, so a scale error in the
	// off-diagonal terms shows up in the round trip.
	{
		// b3a is 32768 brads per circle, the libnds convention, so a quarter
		// of that is 45 degrees. At 90 degrees this test would pass
		// vacuously: a right-angle rotation about z exactly swaps the x and y
		// principal axes and produces no off-diagonal terms at all.
		b3Matrix3 box = b3BoxUnitInertia( V( -2, -1, -1 ), V( 2, 1, 1 ) );
		b3Quat q = b3MakeQuatFromAxisAngle( V( 0, 0, 1 ), (b3a)( 32768 / 8 ) );
		b3Matrix3 u = b3RotateInertia( q, box );
		b3f mass = b3fFromInt( 16 );
		b3MatrixW inv = b3InvertInertia( u, mass );
		check( "rotated box has off-diagonal terms", b3Raw( inv.cy.x ) != 0 );
		expect( "rotated box round trip", inertiaRoundTrip( u, mass, inv ), 0.0, 1e-4 );
	}

	// Documented limit 1: a very heavy, very large body sits on Q24's
	// resolution floor. The inverse is about 6e-7, which is ten quanta, so the
	// round trip is good to about a part in a hundred and no better. This is
	// asserted rather than described so a future change to the scale is caught.
	{
		b3Matrix3 u = b3BoxUnitInertia( V( -5, -5, -5 ), V( 5, 5, 5 ) );
		b3f mass = b3fFromInt( 100000 );
		b3MatrixW inv = b3InvertInertia( u, mass );
		double err = inertiaRoundTrip( u, mass, inv );
		check( "heavy box inverse is non-zero", b3Raw( inv.cx.x ) > 0 );
		check( "heavy box round trip is coarse but bounded", err > 1e-4 && err < 5e-2 );
	}

	// Documented limit 2: a 5 cm sphere at the density of water wants an
	// inverse inertia near 1900, and Q7.24 tops out at 128. The whole tensor is
	// scaled down uniformly, so it stays isotropic and positive definite -- the
	// body behaves as though heavier in rotation, never lighter.
	{
		b3Matrix3 u = b3SphereUnitInertia( b3fFromFrac( 5, 100 ) );
		b3f mass = b3fFromFrac( 524, 1000 );
		b3MatrixW inv = b3InvertInertia( u, mass );
		check( "tiny sphere inverse is clamped below the true value", W( inv.cx.x ) < 1000.0 );
		check( "tiny sphere inverse is still large", W( inv.cx.x ) > 32.0 );
		check( "tiny sphere stays isotropic under the clamp",
			   b3Raw( inv.cx.x ) == b3Raw( inv.cy.y ) && b3Raw( inv.cy.y ) == b3Raw( inv.cz.z ) );
	}

	// Degenerate inputs give zero, which is what "cannot rotate" means to the
	// solver -- the same answer a fixed-rotation body wants.
	{
		b3MatrixW zeroTensor = b3InvertInertia( b3Mat3_zero, b3fFromInt( 1 ) );
		check( "zero tensor inverts to zero", b3Raw( zeroTensor.cx.x ) == 0 && b3Raw( zeroTensor.cy.y ) == 0 );

		b3MatrixW zeroMass = b3InvertInertia( b3SphereUnitInertia( b3fFromInt( 1 ) ), b3f_zero );
		check( "zero mass inverts to zero", b3Raw( zeroMass.cx.x ) == 0 );
	}

	// Rotating an isotropic inverse inertia by any rotation must leave it
	// isotropic. The tolerance is the Q12 rotation matrix, not the tensor.
	{
		b3MatrixW inv = b3InvertInertia( b3SphereUnitInertia( b3fFromInt( 1 ) ), b3fFromInt( 1 ) );

		b3MatrixW same = b3RotateInertiaW( b3Quat_identity, inv );
		checkInt( "rotate by identity is exact", b3Raw( same.cx.x ), b3Raw( inv.cx.x ) );

		b3Quat q = b3MakeQuatFromAxisAngle( b3Normalize( V( 1, 2, 3 ) ), (b3a)( 65536 / 8 ) );
		b3MatrixW rotated = b3RotateInertiaW( q, inv );
		expect( "rotated isotropic diagonal", W( rotated.cx.x ), W( inv.cx.x ), 1e-2 );
		expect( "rotated isotropic off-diagonal", W( rotated.cy.x ), 0.0, 1e-2 );
	}
}

static void test_body_mass( void )
{
	printf( "body mass properties\n" );

	b3WorldId worldId = makeWorld();
	b3World* world = b3GetWorldFromId( worldId );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );
	validate( world );

	// A body with no shapes has no mass, and therefore no inverse mass.
	expect( "empty body mass", F( b3Body_GetMass( bodyId ) ), 0.0, 1e-9 );
	check( "empty body invMass is zero", b3Raw( b3Body_GetInverseMass( bodyId ) ) == 0 );

	// One unit sphere at density 1. Volume 4/3 pi, so mass 4.18879.
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = b3fFromInt( 1 );
	b3Sphere sphere = { V( 0, 0, 0 ), b3fFromInt( 1 ) };
	b3ShapeId shapeId = b3CreateSphereShape( bodyId, &shapeDef, &sphere );
	validate( world );

	check( "shape id valid", b3Shape_IsValid( shapeId ) );
	checkInt( "body has one shape", b3Body_GetShapeCount( bodyId ), 1 );

	double mass = F( b3Body_GetMass( bodyId ) );
	expect( "unit sphere mass", mass, 4.1887902, 2e-3 );
	expect( "invMass * mass", W( b3Body_GetInverseMass( bodyId ) ) * mass, 1.0, 1e-3 );

	// The centre of mass is the sphere centre, which is the body origin.
	b3Vec3 localCenter = b3Body_GetLocalCenter( bodyId );
	expect( "local center x", F( localCenter.x ), 0.0, 1e-3 );
	expect( "local center y", F( localCenter.y ), 0.0, 1e-3 );

	// The stored tensor is per unit mass, so it is 0.4 r^2 regardless of the
	// density -- that is the whole reason the convention exists.
	b3Matrix3 unitInertia = b3Body_GetLocalRotationalInertia( bodyId );
	expect( "unit inertia is 0.4 r^2", F( unitInertia.cx.x ), 0.4, 2e-3 );

	expect( "sphere body inertia round trip",
			inertiaRoundTrip( unitInertia, b3Body_GetMass( bodyId ), b3Body_GetWorldInverseRotationalInertia( bodyId ) ), 0.0,
			1e-4 );

	// A second sphere, offset. The centre of mass moves to the midpoint and
	// the Steiner terms make the tensor anisotropic.
	b3Sphere sphere2 = { V( 2, 0, 0 ), b3fFromInt( 1 ) };
	b3CreateSphereShape( bodyId, &shapeDef, &sphere2 );
	validate( world );

	checkInt( "body has two shapes", b3Body_GetShapeCount( bodyId ), 2 );
	expect( "two-sphere mass", F( b3Body_GetMass( bodyId ) ), 2.0 * 4.1887902, 5e-3 );

	localCenter = b3Body_GetLocalCenter( bodyId );
	expect( "two-sphere center x", F( localCenter.x ), 1.0, 5e-3 );

	unitInertia = b3Body_GetLocalRotationalInertia( bodyId );
	check( "Steiner makes yy larger than xx", b3Raw( unitInertia.cy.y ) > b3Raw( unitInertia.cx.x ) );
	expect( "two-sphere inertia round trip",
			inertiaRoundTrip( unitInertia, b3Body_GetMass( bodyId ), b3Body_GetWorldInverseRotationalInertia( bodyId ) ), 0.0,
			1e-3 );

	// A zero-density shape contributes nothing but still exists.
	b3ShapeDef ghostDef = b3DefaultShapeDef();
	ghostDef.density = b3f_zero;
	b3Sphere ghost = { V( 0, 5, 0 ), b3fFromInt( 1 ) };
	b3CreateSphereShape( bodyId, &ghostDef, &ghost );
	validate( world );

	checkInt( "body has three shapes", b3Body_GetShapeCount( bodyId ), 3 );
	expect( "zero-density shape adds no mass", F( b3Body_GetMass( bodyId ) ), 2.0 * 4.1887902, 5e-3 );

	// Fixed rotation zeroes the tensor but leaves the mass alone.
	b3MotionLocks locks = { 0 };
	locks.angularX = true;
	locks.angularY = true;
	locks.angularZ = true;
	b3Body_SetMotionLocks( bodyId, locks );
	validate( world );

	check( "fixed rotation zeroes the inverse inertia", b3Raw( b3Body_GetWorldInverseRotationalInertia( bodyId ).cx.x ) == 0 );
	check( "fixed rotation keeps the mass", b3Raw( b3Body_GetMass( bodyId ) ) != 0 );

	b3DestroyWorld( worldId );
}

static void test_mass_data_round_trip( void )
{
	printf( "b3Body_SetMassData round trip\n" );

	b3WorldId worldId = makeWorld();
	b3World* world = b3GetWorldFromId( worldId );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = b3fFromInt( 1 );
	b3Sphere sphere = { V( 0, 0, 0 ), b3fFromInt( 1 ) };
	b3CreateSphereShape( bodyId, &shapeDef, &sphere );
	validate( world );

	// What b3Body_GetMassData hands back must be what b3Body_SetMassData
	// accepts. Both speak the per-unit-mass convention, so this is the check
	// that the two halves agree.
	b3MassData got = b3Body_GetMassData( bodyId );
	b3Body_SetMassData( bodyId, got );
	validate( world );

	b3MassData again = b3Body_GetMassData( bodyId );
	checkInt( "mass survives the round trip", b3Raw( again.mass ), b3Raw( got.mass ) );
	checkInt( "center survives the round trip", b3Raw( again.center.x ), b3Raw( got.center.x ) );
	checkInt( "inertia survives the round trip", b3Raw( again.inertia.cx.x ), b3Raw( got.inertia.cx.x ) );

	expect( "set mass data inverse is consistent",
			inertiaRoundTrip( again.inertia, again.mass, b3Body_GetWorldInverseRotationalInertia( bodyId ) ), 0.0, 1e-4 );

	b3DestroyWorld( worldId );
}

// =========================================================================
// Set transitions
// =========================================================================

/// How many proxies a shape has in each of the three trees.
static int proxyCountForBody( b3World* world, b3BodyId bodyId )
{
	b3Body* body = b3GetBodyFullId( world, bodyId );
	int count = 0;
	int shapeId = body->headShapeId;
	while ( shapeId != B3_NULL_INDEX )
	{
		b3Shape* shape = b3Array_Get( world->shapes, shapeId );
		if ( shape->proxyKey != B3_NULL_INDEX )
		{
			count += 1;
		}
		shapeId = shape->nextShapeId;
	}
	return count;
}

static b3BodyType proxyTypeForBody( b3World* world, b3BodyId bodyId )
{
	b3Body* body = b3GetBodyFullId( world, bodyId );
	b3Shape* shape = b3Array_Get( world->shapes, body->headShapeId );
	return B3_PROXY_TYPE( shape->proxyKey );
}

static void test_set_transitions( void )
{
	printf( "solver set transitions\n" );

	b3WorldId worldId = makeWorld();
	b3World* world = b3GetWorldFromId( worldId );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3Sphere sphere = { V( 0, 0, 0 ), b3fFromInt( 1 ) };

	// A static body lands in the static set and takes no island.
	b3BodyDef staticDef = b3DefaultBodyDef();
	staticDef.type = b3_staticBody;
	b3BodyId staticId = b3CreateBody( worldId, &staticDef );
	b3CreateSphereShape( staticId, &shapeDef, &sphere );
	validate( world );

	checkInt( "static body is in the static set", b3GetBodyFullId( world, staticId )->setIndex, b3_staticSet );
	checkInt( "static body has no island", b3GetBodyFullId( world, staticId )->islandId, B3_NULL_INDEX );
	checkInt( "static proxy is in the static tree", proxyTypeForBody( world, staticId ), b3_staticBody );

	// A dynamic body lands in the awake set with an island of its own.
	b3BodyDef dynamicDef = b3DefaultBodyDef();
	dynamicDef.type = b3_dynamicBody;
	dynamicDef.position = V( 0, 5, 0 );
	b3BodyId dynamicId = b3CreateBody( worldId, &dynamicDef );
	b3CreateSphereShape( dynamicId, &shapeDef, &sphere );
	validate( world );

	checkInt( "dynamic body is in the awake set", b3GetBodyFullId( world, dynamicId )->setIndex, b3_awakeSet );
	check( "dynamic body has an island", b3GetBodyFullId( world, dynamicId )->islandId != B3_NULL_INDEX );
	checkInt( "dynamic proxy is in the dynamic tree", proxyTypeForBody( world, dynamicId ), b3_dynamicBody );
	checkInt( "one awake body", b3World_GetAwakeBodyCount( worldId ), 1 );

	// dynamic -> static: the island goes, the proxy moves tree.
	b3Body_SetType( dynamicId, b3_staticBody );
	validate( world );
	checkInt( "re-typed to static set", b3GetBodyFullId( world, dynamicId )->setIndex, b3_staticSet );
	checkInt( "re-typed loses its island", b3GetBodyFullId( world, dynamicId )->islandId, B3_NULL_INDEX );
	checkInt( "re-typed proxy moved to the static tree", proxyTypeForBody( world, dynamicId ), b3_staticBody );

	// static -> kinematic: an island again, and the kinematic tree.
	b3Body_SetType( dynamicId, b3_kinematicBody );
	validate( world );
	checkInt( "kinematic is in the awake set", b3GetBodyFullId( world, dynamicId )->setIndex, b3_awakeSet );
	check( "kinematic has an island", b3GetBodyFullId( world, dynamicId )->islandId != B3_NULL_INDEX );
	checkInt( "kinematic proxy is in the kinematic tree", proxyTypeForBody( world, dynamicId ), b3_kinematicBody );

	b3Body_SetType( dynamicId, b3_dynamicBody );
	validate( world );

	// Disable: no proxy at all, no island, and the disabled set.
	b3Body_Disable( dynamicId );
	validate( world );
	checkInt( "disabled body is in the disabled set", b3GetBodyFullId( world, dynamicId )->setIndex, b3_disabledSet );
	checkInt( "disabled body has no island", b3GetBodyFullId( world, dynamicId )->islandId, B3_NULL_INDEX );
	checkInt( "disabled body has no proxies", proxyCountForBody( world, dynamicId ), 0 );
	check( "disabled body reports disabled", b3Body_IsEnabled( dynamicId ) == false );

	// Re-enable and the proxy comes back.
	b3Body_Enable( dynamicId );
	validate( world );
	checkInt( "enabled body is awake again", b3GetBodyFullId( world, dynamicId )->setIndex, b3_awakeSet );
	checkInt( "enabled body has its proxy back", proxyCountForBody( world, dynamicId ), 1 );
	check( "enabled body reports enabled", b3Body_IsEnabled( dynamicId ) );

	// Sleep and wake. With no contacts the island is the body alone, so this
	// exercises the set create/destroy path rather than island splitting.
	b3Body_SetAwake( dynamicId, false );
	validate( world );
	check( "slept body left the awake set", b3GetBodyFullId( world, dynamicId )->setIndex >= b3_firstSleepingSet );
	check( "slept body reports asleep", b3Body_IsAwake( dynamicId ) == false );
	checkInt( "no awake bodies", b3World_GetAwakeBodyCount( worldId ), 0 );

	b3Body_SetAwake( dynamicId, true );
	validate( world );
	checkInt( "woken body is in the awake set", b3GetBodyFullId( world, dynamicId )->setIndex, b3_awakeSet );
	checkInt( "the sleeping set was destroyed", world->solverSets.count, 4 );
	check( "the destroyed sleeping set slot is free", world->solverSets.data[3].setIndex == B3_NULL_INDEX );

	// Destroy a body that still has shapes attached.
	b3DestroyBody( dynamicId );
	validate( world );
	check( "destroyed body id is stale", b3Body_IsValid( dynamicId ) == false );

	b3DestroyBody( staticId );
	validate( world );

	checkInt( "no bodies remain", b3GetIdCount( &world->bodyIdPool ), 0 );
	checkInt( "no shapes remain", b3GetIdCount( &world->shapeIdPool ), 0 );

	b3DestroyWorld( worldId );
}

static void test_sleep_from_sleeping_set( void )
{
	printf( "destroy from a sleeping set\n" );

	b3WorldId worldId = makeWorld();
	b3World* world = b3GetWorldFromId( worldId );

	b3BodyDef def = b3DefaultBodyDef();
	def.type = b3_dynamicBody;

	// Created asleep: this takes the b3CreateBody branch that allocates a
	// fresh sleeping set, which nothing else in this file reaches.
	def.isAwake = false;
	b3BodyId sleeper = b3CreateBody( worldId, &def );
	validate( world );

	check( "body created asleep", b3GetBodyFullId( world, sleeper )->setIndex >= b3_firstSleepingSet );
	checkInt( "no awake bodies", b3World_GetAwakeBodyCount( worldId ), 0 );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3Sphere sphere = { V( 0, 0, 0 ), b3fFromInt( 1 ) };
	b3CreateSphereShape( sleeper, &shapeDef, &sphere );
	validate( world );

	// Destroying the last body in a sleeping set must destroy the set too, or
	// the set id pool and the array disagree -- which b3ValidateSolverSets
	// checks on its first line.
	b3DestroyBody( sleeper );
	validate( world );
	checkInt( "no bodies remain", b3GetIdCount( &world->bodyIdPool ), 0 );

	b3DestroyWorld( worldId );
}

// =========================================================================
// Ids and recycling
// =========================================================================

static void test_id_recycling( void )
{
	printf( "id recycling\n" );

	b3WorldId worldId = makeWorld();
	b3World* world = b3GetWorldFromId( worldId );

	b3BodyDef def = b3DefaultBodyDef();
	def.type = b3_dynamicBody;

	b3BodyId first = b3CreateBody( worldId, &def );
	int firstIndex = first.index1;
	b3DestroyBody( first );
	validate( world );

	// The slot is recycled, so the new body has the same index and a different
	// generation. That is exactly the case a stale handle has to survive.
	b3BodyId second = b3CreateBody( worldId, &def );
	checkInt( "slot was recycled", second.index1, firstIndex );
	check( "generation advanced", second.generation != first.generation );
	check( "stale id is rejected", b3Body_IsValid( first ) == false );
	check( "fresh id is accepted", b3Body_IsValid( second ) );

	// Shapes recycle the same way.
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3Sphere sphere = { V( 0, 0, 0 ), b3fFromInt( 1 ) };
	b3ShapeId shapeA = b3CreateSphereShape( second, &shapeDef, &sphere );
	b3DestroyShape( shapeA, true );
	validate( world );

	b3ShapeId shapeB = b3CreateSphereShape( second, &shapeDef, &sphere );
	checkInt( "shape slot was recycled", shapeB.index1, shapeA.index1 );
	check( "stale shape id is rejected", b3Shape_IsValid( shapeA ) == false );
	check( "fresh shape id is accepted", b3Shape_IsValid( shapeB ) );

	// A handle from a destroyed world must not validate against a new one.
	b3DestroyWorld( worldId );
	check( "body id from a destroyed world is rejected", b3Body_IsValid( second ) == false );

	b3WorldId worldId2 = makeWorld();
	check( "body id does not survive into a new world", b3Body_IsValid( second ) == false );
	b3DestroyWorld( worldId2 );
}

// =========================================================================
// Shapes
// =========================================================================

static void test_shape_properties( void )
{
	printf( "shape properties\n" );

	b3WorldId worldId = makeWorld();
	b3World* world = b3GetWorldFromId( worldId );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = b3fFromInt( 2 );
	shapeDef.baseMaterial.friction = b3cFromFrac( 3, 10 );

	b3Capsule capsule = { V( 0, -1, 0 ), V( 0, 1, 0 ), b3fFromFrac( 5, 10 ) };
	b3ShapeId capsuleId = b3CreateCapsuleShape( bodyId, &shapeDef, &capsule );
	validate( world );

	checkInt( "capsule stayed a capsule", b3Shape_GetType( capsuleId ), b3_capsuleShape );
	expect( "density round trips", F( b3Shape_GetDensity( capsuleId ) ), 2.0, 1e-3 );
	expect( "friction round trips", (double)b3Raw( b3Shape_GetFriction( capsuleId ) ) / (double)B3_C_ONE, 0.3, 1e-6 );

	// A capsule shorter than the linear slop is created as a sphere instead,
	// because the narrow phase cannot derive an axis from a zero-length
	// segment.
	b3Capsule degenerate = { V( 0, 0, 0 ), V( 0, 0, 0 ), b3fFromFrac( 5, 10 ) };
	b3ShapeId degenerateId = b3CreateCapsuleShape( bodyId, &shapeDef, &degenerate );
	validate( world );
	checkInt( "degenerate capsule became a sphere", b3Shape_GetType( degenerateId ), b3_sphereShape );

	// A hull shape keeps the caller's pointer -- there is no hull database.
	b3BoxHull box = b3MakeBoxHull( b3fFromInt( 1 ), b3fFromInt( 1 ), b3fFromInt( 1 ) );
	b3ShapeId hullId = b3CreateHullShape( bodyId, &shapeDef, &box.base );
	validate( world );
	checkInt( "hull stayed a hull", b3Shape_GetType( hullId ), b3_hullShape );
	check( "hull shape aliases the caller's data", b3Shape_GetHull( hullId ) == &box.base );

	// The fat AABB must contain the tight one, on every axis.
	b3Shape* shape = b3Array_Get( world->shapes, hullId.index1 - 1 );
	check( "fat AABB contains the tight AABB", b3AABB_Contains( shape->fatAABB, shape->aabb ) );

	// Changing the filter with invokeContacts destroys and rebuilds the proxy.
	b3Filter filter = b3DefaultFilter();
	filter.categoryBits = 2;
	filter.maskBits = 2;
	b3Shape_SetFilter( hullId, filter, true );
	validate( world );
	checkInt( "filter round trips", (long long)b3Shape_GetFilter( hullId ).categoryBits, 2 );

	b3DestroyWorld( worldId );
}

static void test_body_transform( void )
{
	printf( "body transform\n" );

	b3WorldId worldId = makeWorld();
	b3World* world = b3GetWorldFromId( worldId );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = V( 3, 4, 5 );
	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3Sphere sphere = { V( 0, 0, 0 ), b3fFromInt( 1 ) };
	b3CreateSphereShape( bodyId, &shapeDef, &sphere );
	validate( world );

	expect( "created at the requested position", F( b3Body_GetPosition( bodyId ).x ), 3.0, 1e-3 );

	// The AABB has to follow the body, or the broad phase reports pairs for
	// where the body used to be.
	b3Body_SetTransform( bodyId, V( -10, 0, 0 ), b3Quat_identity );
	validate( world );

	expect( "moved to the new position", F( b3Body_GetPosition( bodyId ).x ), -10.0, 1e-3 );

	b3Body* body = b3GetBodyFullId( world, bodyId );
	b3Shape* shape = b3Array_Get( world->shapes, body->headShapeId );
	check( "AABB followed the body", F( shape->aabb.upperBound.x ) < -8.0 );
	check( "fat AABB still contains the tight one", b3AABB_Contains( shape->fatAABB, shape->aabb ) );

	// World and local point conversions must invert each other.
	b3Vec3 local = b3Body_GetLocalPoint( bodyId, V( -8, 2, 0 ) );
	b3Pos back = b3Body_GetWorldPoint( bodyId, local );
	expect( "world -> local -> world x", F( back.x ), -8.0, 5e-3 );
	expect( "world -> local -> world y", F( back.y ), 2.0, 5e-3 );

	// Velocity on a dynamic body sticks; on a static body it is ignored.
	b3Body_SetLinearVelocity( bodyId, V( 1, 2, 3 ) );
	expect( "linear velocity y", F( b3Body_GetLinearVelocity( bodyId ).y ), 2.0, 1e-3 );

	b3Body_SetType( bodyId, b3_staticBody );
	validate( world );
	b3Body_SetLinearVelocity( bodyId, V( 9, 9, 9 ) );
	expect( "static body ignores velocity", F( b3Body_GetLinearVelocity( bodyId ).x ), 0.0, 1e-9 );

	b3DestroyWorld( worldId );
}

static void test_impulses( void )
{
	printf( "impulses\n" );

	b3WorldId worldId = makeWorld();
	b3World* world = b3GetWorldFromId( worldId );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = b3fFromInt( 1 );
	b3Sphere sphere = { V( 0, 0, 0 ), b3fFromInt( 1 ) };
	b3CreateSphereShape( bodyId, &shapeDef, &sphere );
	validate( world );

	double mass = F( b3Body_GetMass( bodyId ) );

	// An impulse through the centre of mass produces v = P / m and no spin.
	b3Body_ApplyLinearImpulseToCenter( bodyId, V( 0, 0, 4 ), true );
	expect( "central impulse velocity", F( b3Body_GetLinearVelocity( bodyId ).z ), 4.0 / mass, 5e-3 );
	expect( "central impulse causes no spin", F( b3Body_GetAngularVelocity( bodyId ).x ), 0.0, 1e-6 );

	b3Body_SetLinearVelocity( bodyId, V( 0, 0, 0 ) );
	b3Body_SetAngularVelocity( bodyId, V( 0, 0, 0 ) );

	// Off-centre, the same impulse also spins the body: w = I^-1 (r x P).
	// r = (1,0,0), P = (0,0,4), so r x P = (0,-4,0).
	b3Body_ApplyLinearImpulse( bodyId, V( 0, 0, 4 ), V( 1, 0, 0 ), true );
	expect( "off-centre impulse still moves the centre", F( b3Body_GetLinearVelocity( bodyId ).z ), 4.0 / mass, 5e-3 );

	double unitInertia = F( b3Body_GetLocalRotationalInertia( bodyId ).cy.y );
	expect( "off-centre impulse spins about -y", F( b3Body_GetAngularVelocity( bodyId ).y ), -4.0 / ( mass * unitInertia ),
			5e-2 );

	// The linear speed cap applies to impulses, not only to the solver.
	b3World_SetMaximumLinearSpeed( worldId, b3fFromInt( 5 ) );
	b3Body_ApplyLinearImpulseToCenter( bodyId, V( 0, 0, 1000 ), true );
	check( "impulse is speed capped", F( b3Body_GetLinearVelocity( bodyId ).z ) <= 5.01 );

	b3DestroyWorld( worldId );
}

// =========================================================================
// Phase 3B: contacts
// =========================================================================

/// One step. Phase 3B drove b3UpdateBroadPhasePairs and b3Collide directly,
/// because there was no b3World_Step; 3C-i has one, and it owns the event
/// lifecycle -- so the contact cases below go through the real API.
///
/// makeWorld() zeroes gravity, so a body with no velocity does not move and
/// every teleport-driven case still says what it said before. What changes is
/// that the begin and end touch events are now published the way a caller
/// actually sees them.
static void step( b3World* world )
{
	b3WorldId worldId = { (uint16_t)( world->worldId + 1 ), world->generation };
	b3World_Step( worldId, 1 );
	validate( world );
}

static int contactCount( b3World* world )
{
	return b3GetIdCount( &world->contactIdPool );
}

/// How many contacts on this body are flagged as touching.
static int touchingCountForBody( b3World* world, b3BodyId bodyId )
{
	b3Body* body = b3GetBodyFullId( world, bodyId );
	int count = 0;
	int key = body->headContactKey;
	while ( key != B3_NULL_INDEX )
	{
		b3Contact* contact = b3Array_Get( world->contacts, key >> 1 );
		if ( contact->flags & b3_contactTouchingFlag )
		{
			count += 1;
		}
		key = contact->edges[key & 1].nextKey;
	}
	return count;
}

/// The world's single contact, or NULL. Every case below is built so that
/// there is at most one.
static b3Contact* onlyContact( b3World* world )
{
	for ( int i = 0; i < world->contacts.count; ++i )
	{
		b3Contact* contact = world->contacts.data + i;
		if ( contact->contactId != B3_NULL_INDEX )
		{
			return contact;
		}
	}
	return NULL;
}

static b3BodyId makeDynamicSphere( b3WorldId worldId, b3Vec3 position, double radius )
{
	b3BodyDef def = b3DefaultBodyDef();
	def.type = b3_dynamicBody;
	def.position = position;
	b3BodyId bodyId = b3CreateBody( worldId, &def );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.enableContactEvents = true;
	b3Sphere sphere = { V( 0, 0, 0 ), b3fFromDouble( radius ) };
	b3CreateSphereShape( bodyId, &shapeDef, &sphere );
	return bodyId;
}

// -------------------------------------------------------------------------

static void test_pair_discovery( void )
{
	printf( "pair discovery\n" );

	b3WorldId worldId = makeWorld();
	b3World* world = b3GetWorldFromId( worldId );

	// Two overlapping dynamic spheres: exactly one contact, no more.
	b3BodyId a = makeDynamicSphere( worldId, V( 0, 0, 0 ), 1.0 );
	b3BodyId b = makeDynamicSphere( worldId, V( 1.5, 0, 0 ), 1.0 );
	step( world );
	checkInt( "overlapping pair makes one contact", contactCount( world ), 1 );
	checkInt( "the contact is touching", touchingCountForBody( world, a ), 1 );

	// Running the pass again must not create a second contact: the pair set
	// is what prevents that, and it is the branch a duplicate would slip past.
	step( world );
	checkInt( "re-running the pass makes no duplicate", contactCount( world ), 1 );

	// Both proxies moving in the same pass is the de-duplication branch
	// proper -- the pair is discovered from each side and only one may
	// survive. Move both, in both orders.
	b3Body_SetTransform( a, V( 0.1, 0, 0 ), b3Quat_identity );
	b3Body_SetTransform( b, V( 1.6, 0, 0 ), b3Quat_identity );
	step( world );
	checkInt( "both proxies moved: still one contact", contactCount( world ), 1 );

	b3Body_SetTransform( b, V( 1.7, 0, 0 ), b3Quat_identity );
	b3Body_SetTransform( a, V( 0.2, 0, 0 ), b3Quat_identity );
	step( world );
	checkInt( "moved in the other order: still one contact", contactCount( world ), 1 );

	b3DestroyBody( a );
	b3DestroyBody( b );
	validate( world );
	checkInt( "destroying the bodies destroys the contact", contactCount( world ), 0 );

	// Two static bodies never pair, however much they overlap.
	b3BodyDef staticDef = b3DefaultBodyDef();
	staticDef.type = b3_staticBody;
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3Sphere sphere = { V( 0, 0, 0 ), b3fFromInt( 1 ) };

	b3BodyId s1 = b3CreateBody( worldId, &staticDef );
	b3CreateSphereShape( s1, &shapeDef, &sphere );
	b3BodyId s2 = b3CreateBody( worldId, &staticDef );
	b3CreateSphereShape( s2, &shapeDef, &sphere );
	step( world );
	checkInt( "static versus static makes no contact", contactCount( world ), 0 );

	b3DestroyBody( s1 );
	b3DestroyBody( s2 );

	// Two shapes on one body never pair either.
	b3BodyDef dynamicDef = b3DefaultBodyDef();
	dynamicDef.type = b3_dynamicBody;
	b3BodyId one = b3CreateBody( worldId, &dynamicDef );
	b3CreateSphereShape( one, &shapeDef, &sphere );
	b3CreateSphereShape( one, &shapeDef, &sphere );
	step( world );
	checkInt( "two shapes on one body make no contact", contactCount( world ), 0 );
	b3DestroyBody( one );

	// Filtering: disjoint mask bits suppress the pair.
	b3BodyDef defA = b3DefaultBodyDef();
	defA.type = b3_dynamicBody;
	b3BodyId f1 = b3CreateBody( worldId, &defA );
	b3ShapeDef filtered = b3DefaultShapeDef();
	filtered.filter.categoryBits = 0x0001;
	filtered.filter.maskBits = 0x0002;
	b3CreateSphereShape( f1, &filtered, &sphere );

	b3BodyId f2 = b3CreateBody( worldId, &defA );
	filtered.filter.categoryBits = 0x0004;
	filtered.filter.maskBits = 0x0008;
	b3CreateSphereShape( f2, &filtered, &sphere );

	step( world );
	checkInt( "disjoint filters suppress the pair", contactCount( world ), 0 );

	b3DestroyBody( f1 );
	b3DestroyBody( f2 );
	validate( world );

	b3DestroyWorld( worldId );
}

// -------------------------------------------------------------------------

static bool s_filterCalled = false;
static bool s_filterAnswer = false;

static bool testCustomFilter( b3ShapeId shapeIdA, b3ShapeId shapeIdB, void* context )
{
	(void)shapeIdA;
	(void)shapeIdB;
	(void)context;
	s_filterCalled = true;
	return s_filterAnswer;
}

static void test_custom_filter( void )
{
	printf( "custom contact filter\n" );

	b3WorldId worldId = makeWorld();
	b3World* world = b3GetWorldFromId( worldId );
	b3World_SetCustomFilterCallback( worldId, testCustomFilter, NULL );

	b3BodyDef def = b3DefaultBodyDef();
	def.type = b3_dynamicBody;
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.enableCustomFiltering = true;
	b3Sphere sphere = { V( 0, 0, 0 ), b3fFromInt( 1 ) };

	b3BodyId a = b3CreateBody( worldId, &def );
	b3CreateSphereShape( a, &shapeDef, &sphere );
	def.position = V( 1.5, 0, 0 );
	b3BodyId b = b3CreateBody( worldId, &def );
	b3CreateSphereShape( b, &shapeDef, &sphere );

	s_filterCalled = false;
	s_filterAnswer = false;
	step( world );
	check( "the custom filter was consulted", s_filterCalled );
	checkInt( "a refusing filter suppresses the pair", contactCount( world ), 0 );

	// Saying yes lets the same pair through on the next pass -- but the
	// proxy has to genuinely move first, because the failed query cleared the
	// move array and a shape whose tight AABB is still inside its fat one
	// never re-buffers. Half a radius is well past the margin.
	s_filterAnswer = true;
	b3Body_SetTransform( a, V( 0.5, 0, 0 ), b3Quat_identity );
	step( world );
	checkInt( "an accepting filter admits the pair", contactCount( world ), 1 );

	b3World_SetCustomFilterCallback( worldId, NULL, NULL );
	b3DestroyWorld( worldId );
}

// -------------------------------------------------------------------------

static void test_touch_transitions( void )
{
	printf( "touch transitions\n" );

	b3WorldId worldId = makeWorld();
	b3World* world = b3GetWorldFromId( worldId );

	// Far enough apart that even the fat AABBs miss.
	b3BodyId a = makeDynamicSphere( worldId, V( 0, 0, 0 ), 1.0 );
	b3BodyId b = makeDynamicSphere( worldId, V( 8, 0, 0 ), 1.0 );
	step( world );
	checkInt( "distant bodies make no contact", contactCount( world ), 0 );

	// Close enough for the fat AABBs to overlap, but not touching: a contact
	// exists and lives in the awake set's non-touching list, not the graph.
	b3Body_SetTransform( b, V( 2.05, 0, 0 ), b3Quat_identity );
	step( world );
	checkInt( "near bodies make a contact", contactCount( world ), 1 );
	{
		b3Contact* contact = onlyContact( world );
		check( "the contact exists", contact != NULL );
		if ( contact != NULL )
		{
			checkInt( "not yet touching", ( contact->flags & b3_contactTouchingFlag ) != 0, 0 );
			checkInt( "not in the graph", contact->colorIndex, B3_NULL_INDEX );
			checkInt( "not in an island", contact->islandId, B3_NULL_INDEX );
			checkInt( "in the awake set", contact->setIndex, b3_awakeSet );
			checkInt( "no manifold", contact->manifoldCount, 0 );
		}
	}

	// Overlap: begin touching. The contact links an island and enters the
	// graph, and a begin event is reported.
	b3Body_SetTransform( b, V( 1.5, 0, 0 ), b3Quat_identity );
	step( world );
	{
		b3Contact* contact = onlyContact( world );
		check( "still one contact", contact != NULL );
		if ( contact != NULL )
		{
			check( "now touching", ( contact->flags & b3_contactTouchingFlag ) != 0 );
			check( "now in the graph", contact->colorIndex != B3_NULL_INDEX );
			check( "now in an island", contact->islandId != B3_NULL_INDEX );
			checkInt( "one manifold", contact->manifoldCount, 1 );
			check( "the manifold has points", contact->manifolds[0].pointCount > 0 );
		}

		b3ContactEvents events = b3World_GetContactEvents( worldId );
		checkInt( "one begin touch event", events.beginCount, 1 );
	}

	// The two bodies are now one island.
	checkInt( "the touching bodies share an island", b3GetBodyFullId( world, a )->islandId,
			  b3GetBodyFullId( world, b )->islandId );

	// Separate, but stay inside the fat AABBs: stop touching. The contact
	// survives, back in the non-touching list.
	b3Body_SetTransform( b, V( 2.05, 0, 0 ), b3Quat_identity );
	step( world );
	{
		b3Contact* contact = onlyContact( world );
		check( "the contact survives separation", contact != NULL );
		if ( contact != NULL )
		{
			checkInt( "no longer touching", ( contact->flags & b3_contactTouchingFlag ) != 0, 0 );
			checkInt( "out of the graph", contact->colorIndex, B3_NULL_INDEX );
			checkInt( "out of the island", contact->islandId, B3_NULL_INDEX );
			checkInt( "manifold released", contact->manifoldCount, 0 );
		}

		b3ContactEvents events = b3World_GetContactEvents( worldId );
		checkInt( "one end touch event", events.endCount, 1 );
	}

	// Separate past the fat AABBs: the contact is destroyed outright.
	b3Body_SetTransform( b, V( 8, 0, 0 ), b3Quat_identity );
	step( world );
	checkInt( "separating past the fat AABB destroys the contact", contactCount( world ), 0 );

	b3DestroyWorld( worldId );
}

// -------------------------------------------------------------------------

static void test_manifold_persistence( void )
{
	printf( "manifold persistence\n" );

	b3WorldId worldId = makeWorld();
	b3World* world = b3GetWorldFromId( worldId );

	// A box resting on a box: the case a stack is made of, and the case whose
	// feature ids must not renumber between steps.
	b3BoxHull ground = b3MakeBoxHull( b3fFromInt( 4 ), b3fFromInt( 1 ), b3fFromInt( 4 ) );
	b3BoxHull crate = b3MakeBoxHull( b3fFromInt( 1 ), b3fFromInt( 1 ), b3fFromInt( 1 ) );

	b3BodyDef staticDef = b3DefaultBodyDef();
	staticDef.type = b3_staticBody;
	b3BodyId groundId = b3CreateBody( worldId, &staticDef );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3CreateHullShape( groundId, &shapeDef, &ground.base );

	b3BodyDef dynamicDef = b3DefaultBodyDef();
	dynamicDef.type = b3_dynamicBody;
	// Resting: the crate's underside sits on the ground's top face.
	dynamicDef.position = V( 0, 2.0, 0 );
	b3BodyId crateId = b3CreateBody( worldId, &dynamicDef );
	b3CreateHullShape( crateId, &shapeDef, &crate.base );

	step( world );

	uint32_t firstIds[B3_MAX_MANIFOLD_POINTS];
	int firstCount = 0;
	{
		b3Contact* contact = onlyContact( world );
		check( "box on box touches", contact != NULL && ( contact->flags & b3_contactTouchingFlag ) );
		if ( contact != NULL && contact->manifoldCount > 0 )
		{
			firstCount = contact->manifolds[0].pointCount;
			checkInt( "a resting box face gives four points", firstCount, 4 );
			for ( int i = 0; i < firstCount; ++i )
			{
				firstIds[i] = contact->manifolds[0].points[i].featureId;
			}
		}
	}

	// Re-collide without moving anything. The feature ids must be identical
	// and every point must report as persisted -- this is the property warm
	// starting depends on, and the one a cross-library feature flip cannot
	// threaten because it is measured inside a single library.
	step( world );
	{
		b3Contact* contact = onlyContact( world );
		if ( contact != NULL && contact->manifoldCount > 0 && firstCount > 0 )
		{
			checkInt( "point count is stable", contact->manifolds[0].pointCount, firstCount );

			bool sameIds = true;
			bool allPersisted = true;
			for ( int i = 0; i < contact->manifolds[0].pointCount; ++i )
			{
				if ( contact->manifolds[0].points[i].featureId != firstIds[i] )
				{
					sameIds = false;
				}
				if ( contact->manifolds[0].points[i].persisted == false )
				{
					allPersisted = false;
				}
			}
			check( "feature ids are stable across passes", sameIds );
			check( "every point persisted", allPersisted );
		}
	}

	// A nudge far below a slop must not renumber the points either.
	b3Body_SetTransform( crateId, V( 0.001, 2.0, 0 ), b3Quat_identity );
	step( world );
	{
		b3Contact* contact = onlyContact( world );
		if ( contact != NULL && contact->manifoldCount > 0 && firstCount > 0 )
		{
			bool sameIds = contact->manifolds[0].pointCount == firstCount;
			for ( int i = 0; sameIds && i < firstCount; ++i )
			{
				sameIds = contact->manifolds[0].points[i].featureId == firstIds[i];
			}
			check( "a sub-slop nudge keeps the feature ids", sameIds );
		}
	}

	b3DestroyWorld( worldId );
}

// -------------------------------------------------------------------------

static void test_contact_recycling( void )
{
	printf( "contact recycling\n" );

	// The recycling path updates separation rather than recomputing the
	// manifold. When it is working the answer is *supposed* to be the same as
	// the full narrow phase, so the only way to see the arithmetic is to run
	// the same scene twice with recycling off and on and compare.
	//
	// This is what exercises the three thresholds -- the Q30 angular distance,
	// the Q24 squared translation, and the Q24 conservative-advancement arc --
	// each of which sits at a different scale.
	b3f separationsOff[B3_MAX_MANIFOLD_POINTS] = { 0 };
	b3f separationsOn[B3_MAX_MANIFOLD_POINTS] = { 0 };
	int countOff = 0;
	int countOn = 0;
	int recycledOn = 0;
	int recycledOff = 0;

	for ( int pass = 0; pass < 2; ++pass )
	{
		bool recycling = ( pass == 1 );

		b3WorldId worldId = makeWorld();
		b3World* world = b3GetWorldFromId( worldId );

		if ( recycling == false )
		{
			b3World_SetContactRecycleDistance( worldId, b3f_zero );
		}

		b3BoxHull ground = b3MakeBoxHull( b3fFromInt( 4 ), b3fFromInt( 1 ), b3fFromInt( 4 ) );
		b3BoxHull crate = b3MakeBoxHull( b3fFromInt( 1 ), b3fFromInt( 1 ), b3fFromInt( 1 ) );

		b3BodyDef staticDef = b3DefaultBodyDef();
		staticDef.type = b3_staticBody;
		b3BodyId groundId = b3CreateBody( worldId, &staticDef );
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		b3CreateHullShape( groundId, &shapeDef, &ground.base );

		b3BodyDef dynamicDef = b3DefaultBodyDef();
		dynamicDef.type = b3_dynamicBody;
		dynamicDef.position = V( 0, 2.0, 0 );
		b3BodyId crateId = b3CreateBody( worldId, &dynamicDef );
		b3CreateHullShape( crateId, &shapeDef, &crate.base );

		// Settle, then creep down by a fraction of a slop per pass -- small
		// enough that recycling accepts every one of them, which is exactly
		// the regime where a biased separation increment would walk the box
		// into the floor.
		step( world );
		int recycled = 0;
		for ( int i = 1; i <= 8; ++i )
		{
			b3Body_SetTransform( crateId, V( 0, 2.0 - 0.0005 * i, 0 ), b3Quat_identity );
			step( world );
			recycled += world->recycledContactCount;
		}

		b3Contact* contact = onlyContact( world );
		check( "the resting contact survived the creep", contact != NULL && contact->manifoldCount == 1 );
		if ( contact != NULL && contact->manifoldCount == 1 )
		{
			b3Manifold* manifold = contact->manifolds;
			if ( recycling )
			{
				recycledOn = recycled;
				countOn = manifold->pointCount;
				for ( int i = 0; i < countOn; ++i )
				{
					separationsOn[i] = manifold->points[i].separation;
				}
			}
			else
			{
				recycledOff = recycled;
				countOff = manifold->pointCount;
				for ( int i = 0; i < countOff; ++i )
				{
					separationsOff[i] = manifold->points[i].separation;
				}
			}
		}

		b3DestroyWorld( worldId );
	}

	checkInt( "same point count with and without recycling", countOn, countOff );

	// Without these two the comparison below is vacuous: if the fast path
	// never fired, both runs took the same code and agreeing proves nothing.
	checkInt( "recycling off means nothing is recycled", recycledOff, 0 );
	check( "the recycling fast path actually fired", recycledOn > 0 );

	// One slop of tolerance: the recycled separation is a first-order estimate
	// of the recomputed one, and the point of the bound is that it does not
	// drift, not that it is exact.
	double tol = 4.0 * (double)b3Raw( B3_LINEAR_SLOP ) / (double)B3_F_ONE;
	for ( int i = 0; i < countOn && i < countOff; ++i )
	{
		expect( "recycled separation matches the recomputed one", F( separationsOn[i] ), F( separationsOff[i] ), tol );
	}
}

// -------------------------------------------------------------------------

static void test_contact_set_transitions( void )
{
	printf( "contacts across set transitions\n" );

	b3WorldId worldId = makeWorld();
	b3World* world = b3GetWorldFromId( worldId );

	b3BodyId a = makeDynamicSphere( worldId, V( 0, 0, 0 ), 1.0 );
	b3BodyId b = makeDynamicSphere( worldId, V( 1.5, 0, 0 ), 1.0 );
	step( world );
	checkInt( "one touching contact to start", touchingCountForBody( world, a ), 1 );

	// Sleeping the island moves the contact into a sleeping set, and waking it
	// brings it back. Both are 3A machinery, exercised here for the first time
	// with a contact actually present.
	int islandId = b3GetBodyFullId( world, a )->islandId;
	b3TrySleepIsland( world, islandId );
	validate( world );
	{
		b3Contact* contact = onlyContact( world );
		check( "a slept contact leaves the awake set", contact != NULL && contact->setIndex >= b3_firstSleepingSet );
		check( "a slept contact leaves the graph", contact != NULL && contact->colorIndex == B3_NULL_INDEX );
	}

	b3Body_SetAwake( a, true );
	validate( world );
	{
		b3Contact* contact = onlyContact( world );
		check( "a woken contact returns to the awake set", contact != NULL && contact->setIndex == b3_awakeSet );
		check( "a woken touching contact returns to the graph", contact != NULL && contact->colorIndex != B3_NULL_INDEX );
	}

	// Disabling a body tears its contacts down.
	b3Body_Disable( b );
	validate( world );
	checkInt( "disabling a body destroys its contacts", contactCount( world ), 0 );

	b3Body_Enable( b );
	step( world );
	checkInt( "re-enabling remakes the contact", contactCount( world ), 1 );

	// dynamic -> static with a live contact against another dynamic body:
	// the contact survives, because dynamic-versus-static still collides.
	b3Body_SetType( b, b3_staticBody );
	validate( world );
	step( world );
	checkInt( "a dynamic-static pair still has its contact", contactCount( world ), 1 );

	// Destroying the shape takes the contact with it.
	{
		b3Body* body = b3GetBodyFullId( world, b );
		b3ShapeId shapeId = { body->headShapeId + 1, world->worldId,
							  b3Array_Get( world->shapes, body->headShapeId )->generation };
		b3DestroyShape( shapeId, true );
	}
	validate( world );
	checkInt( "destroying a shape destroys its contacts", contactCount( world ), 0 );

	b3DestroyWorld( worldId );
}

// -------------------------------------------------------------------------

static void test_contact_between_sleeping_bodies( void )
{
	printf( "contact between sleeping bodies\n" );

	// b3CreateContact parks a pair in the *disabled* set when neither body is
	// awake. Nothing else in the suite reaches that branch, and it is the one
	// where a wrong set index would not be caught by b3ValidateSolverSets
	// until much later.
	b3WorldId worldId = makeWorld();
	b3World* world = b3GetWorldFromId( worldId );

	b3BodyId a = makeDynamicSphere( worldId, V( 0, 0, 0 ), 1.0 );
	b3BodyId b = makeDynamicSphere( worldId, V( 8, 0, 0 ), 1.0 );
	step( world );
	checkInt( "no contact while apart", contactCount( world ), 0 );

	// Move b into range, then put both islands to sleep before the pass that
	// would find the pair.
	b3Body_SetTransform( b, V( 2.05, 0, 0 ), b3Quat_identity );
	b3TrySleepIsland( world, b3GetBodyFullId( world, a )->islandId );
	b3TrySleepIsland( world, b3GetBodyFullId( world, b )->islandId );
	validate( world );

	b3UpdateBroadPhasePairs( world );
	validate( world );

	{
		b3Contact* contact = onlyContact( world );
		check( "a pair between two sleeping bodies is created", contact != NULL );
		if ( contact != NULL )
		{
			checkInt( "and parked in the disabled set", contact->setIndex, b3_disabledSet );
			checkInt( "not in the graph", contact->colorIndex, B3_NULL_INDEX );
			checkInt( "not touching", ( contact->flags & b3_contactTouchingFlag ) != 0, 0 );
		}
	}

	// Waking either body must bring the contact back to the awake set.
	b3Body_SetAwake( a, true );
	validate( world );
	{
		b3Contact* contact = onlyContact( world );
		check( "waking a body wakes its contact", contact != NULL && contact->setIndex == b3_awakeSet );
	}

	step( world );
	validate( world );

	b3DestroyWorld( worldId );
}


// =========================================================================
// Phase 3C-i: the step
// =========================================================================

/// A world that actually steps: gravity as given, and a dynamic sphere.
static b3WorldId makeStepWorld( b3Vec3 gravity )
{
	b3WorldDef def = b3DefaultWorldDef();
	def.capacity.staticBodyCount = 8;
	def.capacity.dynamicBodyCount = 16;
	def.capacity.staticShapeCount = 8;
	def.capacity.dynamicShapeCount = 16;
	def.capacity.contactCount = 32;
	def.capacity.jointCount = 16;
	def.gravity = gravity;
	return b3CreateWorld( &def );
}

static void test_ballistics( void )
{
	printf( "ballistics against the closed form\n" );

	// The first end-to-end check that dt, h and the sub-step count are wired
	// together correctly. Under constant gravity the answer is exactly
	// s = 1/2 g t^2 and v = g t, so any disagreement is the integrator's.
	//
	// It is also the case Phase 1 finding 2 predicts trouble for: the velocity
	// increment is applied 240 times per second, and a narrowing multiply that
	// truncated rather than rounded would bias every one of them the same way.
	const double g = -10.0;
	b3WorldId worldId = makeStepWorld( V( 0, g, 0 ) );
	b3World* world = b3GetWorldFromId( worldId );

	b3BodyDef def = b3DefaultBodyDef();
	def.type = b3_dynamicBody;
	def.position = V( 0, 0, 0 );
	// Sleeping would stop the fall partway through and is tested separately.
	def.enableSleep = false;
	b3BodyId bodyId = b3CreateBody( worldId, &def );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3Sphere sphere = { V( 0, 0, 0 ), b3fFromDouble( 0.5 ) };
	b3CreateSphereShape( bodyId, &shapeDef, &sphere );

	const int steps = 60;
	const double dt = 1.0 / 60.0;
	double relVelHalf = 0.0;
	double relVelEnd = 0.0;
	double relPosEnd = 0.0;
	double quantaVelEnd = 0.0;

	for ( int i = 1; i <= steps; ++i )
	{
		b3World_Step( worldId, 4 );

		double t = i * dt;
		double wantV = g * t;

		// The sub-stepped Euler integrator applies gravity at the *start* of
		// each sub-step, so the position it produces is the exact sum
		// h^2 * g * (1 + 2 + ... + n), not the continuous 1/2 g t^2. Comparing
		// against the continuous form would charge the integrator for a
		// discretisation the reference makes too.
		int n = i * 4;
		double h = dt / 4.0;
		double wantP = g * h * h * ( (double)n * ( n + 1 ) / 2.0 );

		double gotV = F( b3Body_GetLinearVelocity( bodyId ).y );
		double gotP = F( b3Body_GetPosition( bodyId ).y );

		// What is worth asserting is the *relative* error, and that it does not
		// grow. The absolute error necessarily does: the velocity grows
		// linearly, so a constant relative error is a linearly growing
		// absolute one, and reporting quanta alone would look like a drift.
		double relV = fabs( gotV - wantV ) / fabs( wantV );
		double relP = fabs( gotP - wantP ) / fabs( wantP );

		if ( i == steps / 2 )
		{
			relVelHalf = relV;
		}
		if ( i == steps )
		{
			relVelEnd = relV;
			relPosEnd = relP;
			quantaVelEnd = fabs( gotV - wantV ) * 4096.0;
		}
	}

	// The floor here is a representation limit, not an integrator error. One
	// sub-step of gravity is 10/240 = 0.0416667, which is 170.667 quanta at
	// Q12 -- and Q12 can only hold 170 or 171. Rounding to nearest takes the
	// 0.333 error rather than truncation's 0.667, so the increment runs
	// 0.195% high and the velocity inherits exactly that, forever. It does not
	// compound: the error is proportional to the velocity, not to the step
	// count. More sub-steps make it slightly worse, because a smaller
	// increment has a relatively larger unrepresentable remainder.
	printf( "  after %d steps: relative error  velocity %.3e  position %.3e   (%.0f quanta; Q12 floor is 1.95e-03)\n",
			steps, relVelEnd, relPosEnd, quantaVelEnd );

	check( "velocity tracks g*t to the Q12 floor", relVelEnd < 3e-3 );
	check( "position tracks the discrete sum", relPosEnd < 3e-3 );

	// The real invariant: the relative error is flat. A truncating narrowing
	// multiply -- Phase 1 finding 2 -- would instead show it climbing with the
	// step count, which is what this comparison sees.
	check( "relative error does not grow with the step count", relVelEnd < 1.5 * relVelHalf + 1e-4 );

	validate( world );
	b3DestroyWorld( worldId );
}

static void test_constant_velocity( void )
{
	printf( "constant velocity against the closed form\n" );

	// Ballistics above accelerates, which hides this: when the velocity changes
	// every sub-step, the rounding of the position increment changes with it
	// and the errors partly cancel. Hold the velocity *constant* and they stop
	// cancelling, because the same increment is rounded the same way 240 times
	// a second.
	//
	// That is what a Q12 position increment cannot survive. At 0.1 m/s the
	// increment is 1.7 quanta per sub-step, and no rounding of 1.7 is close:
	// the position ran 17% long. The fix is that the accumulator is Q24 and the
	// finalize pass carries the sub-quantum remainder forward instead of
	// discarding it -- see b3BodyState::deltaPosition.
	//
	// The remaining floor is not the accumulator but `h` itself: 2^24/240 is
	// 69905.07 and b3World_Step stores 69905, so the sub-step is 9.5e-7 short
	// and the distance inherits exactly that. Nine parts per million, and it is
	// the same on every one of these speeds, which is the point of testing more
	// than one.
	const double speeds[3] = { 0.1, 1.5, 12.0 };

	for ( int k = 0; k < 3; ++k )
	{
		double speed = speeds[k];

		// No gravity: the only thing moving this body is the position
		// integrator, so the closed form is exactly v * t.
		b3WorldId worldId = makeStepWorld( V( 0, 0, 0 ) );
		b3World* world = b3GetWorldFromId( worldId );

		b3BodyDef def = b3DefaultBodyDef();
		def.type = b3_dynamicBody;
		def.position = V( 0, 0, 0 );
		def.enableSleep = false;
		b3BodyId bodyId = b3CreateBody( worldId, &def );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		b3Sphere sphere = { V( 0, 0, 0 ), b3fFromDouble( 0.5 ) };
		b3CreateSphereShape( bodyId, &shapeDef, &sphere );

		b3Body_SetLinearVelocity( bodyId, V( speed, 0, 0 ) );

		// Read the velocity back rather than trusting the request. 0.1 m/s is
		// 409.6 quanta and the body stores 410, so a closed form built on 0.1
		// would charge the integrator 1.6 quanta for a rounding that happened
		// before it ran -- the same reason ballistics above compares against
		// the discrete sum rather than the continuous 1/2 g t^2. Nothing
		// changes the velocity after this point, so one read is enough.
		speed = F( b3Body_GetLinearVelocity( bodyId ).x );

		const int steps = 240;
		const double dt = 1.0 / 60.0;
		double relHalf = 0.0;
		double relEnd = 0.0;
		double quantaEnd = 0.0;

		for ( int i = 1; i <= steps; ++i )
		{
			b3World_Step( worldId, 4 );

			double want = speed * ( i * dt );
			double got = F( b3Body_GetPosition( bodyId ).x );
			double rel = fabs( got - want ) / want;

			if ( i == steps / 2 )
			{
				relHalf = rel;
			}
			if ( i == steps )
			{
				relEnd = rel;
				quantaEnd = fabs( got - want ) * 4096.0;
			}
		}

		printf( "  %5.1f m/s over %d steps: relative error %.3e  (%.2f quanta)\n", speed, steps, relEnd, quantaEnd );

		// Generous against the 9e-6 floor and brutal against the defect: the
		// Q12 accumulator missed this by 1.7e-1 at 0.1 m/s and 1.6e-2 at
		// 1.5 m/s, both orders above this line.
		check( "distance tracks v*t", relEnd < 1e-3 );

		// The invariant that separates a floor from a drift, as in ballistics.
		// A biased increment shows up here as an error growing with the step
		// count rather than staying flat -- and unlike ballistics, nothing else
		// in this scene is changing to disguise it.
		check( "relative error does not grow with the step count", relEnd < 1.5 * relHalf + 1e-5 );

		// The absolute error is bounded too, and by a constant rather than by
		// anything that scales with distance: the unapplied remainder is under
		// one Q12 quantum by construction, and h's shortfall contributes the
		// rest. Asserting quanta directly is what would have caught the defect
		// at 12 m/s, where the *relative* error was only 0.2%.
		check( "absolute error stays inside a quantum plus h's own shortfall", quantaEnd < 1.0 + speed * 0.05 );

		validate( world );
		b3DestroyWorld( worldId );
	}
}

static void test_damping( void )
{
	printf( "damping against the closed form\n" );

	// The integrator uses a Pade approximant, v /= (1 + h*c), not exp(-c*t).
	// Asserting against the exponential would charge the port for upstream's
	// approximation, so the closed form here is the Pade recurrence itself.
	const double c = 2.0;
	b3WorldId worldId = makeStepWorld( V( 0, 0, 0 ) );
	b3World* world = b3GetWorldFromId( worldId );

	b3BodyDef def = b3DefaultBodyDef();
	def.type = b3_dynamicBody;
	def.linearDamping = b3fFromDouble( c );
	def.enableSleep = false;
	b3BodyId bodyId = b3CreateBody( worldId, &def );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3Sphere sphere = { V( 0, 0, 0 ), b3fFromDouble( 0.5 ) };
	b3CreateSphereShape( bodyId, &shapeDef, &sphere );

	b3Body_SetLinearVelocity( bodyId, V( 10, 0, 0 ) );

	const int steps = 30;
	const int substeps = 4;
	const double h = 1.0 / ( 60.0 * substeps );
	double want = 10.0;

	for ( int i = 0; i < steps; ++i )
	{
		b3World_Step( worldId, substeps );
		for ( int k = 0; k < substeps; ++k )
		{
			want /= ( 1.0 + h * c );
		}
	}

	double got = F( b3Body_GetLinearVelocity( bodyId ).x );
	printf( "  after %d steps: v = %.6f, closed form %.6f\n", steps, got, want );
	expect( "damped velocity matches the Pade recurrence", got, want, 2e-2 );

	validate( world );
	b3DestroyWorld( worldId );
}

static void test_soft_constraints( void )
{
	printf( "b3MakeSoft\n" );

	// massScale + impulseScale == 1 is an identity upstream states and the port
	// gets by construction (massScale is computed as 1 - impulseScale). It is
	// the cheapest check that sees a scale error in either.
	const double hertzes[4] = { 30.0, 60.0, 2.0, 240.0 };
	const double zetas[3] = { 10.0, 1.0, 0.0 };

	for ( int i = 0; i < 4; ++i )
	{
		for ( int j = 0; j < 3; ++j )
		{
			b3Softness soft = b3MakeSoft( b3fFromDouble( hertzes[i] ), b3fFromDouble( zetas[j] ), b3tFromFrac( 1, 240 ) );
			checkInt( "massScale + impulseScale == 1", b3Raw( soft.massScale ) + b3Raw( soft.impulseScale ), B3_C_ONE );
		}
	}

	// Zero hertz is how a caller turns soft contacts off.
	{
		b3Softness soft = b3MakeSoft( b3f_zero, b3fFromDouble( 10.0 ), b3tFromFrac( 1, 240 ) );
		checkInt( "hertz 0 gives biasRate 0", b3Raw( soft.biasRate ), 0 );
		checkInt( "hertz 0 gives massScale 0", b3Raw( soft.massScale ), 0 );
		checkInt( "hertz 0 gives impulseScale 0", b3Raw( soft.impulseScale ), 0 );
	}

	// biasRate at the port's own defaults, and at the zeta = 0 limit where
	// upstream says it equals 1/h. That second case is the one that decided the
	// field's scale: 240 does not fit Q7.24, which tops out at 128.
	{
		b3Softness soft = b3MakeSoft( b3fFromDouble( 30.0 ), b3fFromDouble( 10.0 ), b3tFromFrac( 1, 240 ) );
		double omega = 2.0 * 3.14159265358979 * 30.0;
		double a1 = 2.0 * 10.0 + omega / 240.0;
		expect( "biasRate at the defaults", F( soft.biasRate ), omega / a1, 1e-3 );
	}
	{
		b3Softness soft = b3MakeSoft( b3fFromDouble( 30.0 ), b3f_zero, b3tFromFrac( 1, 240 ) );
		expect( "biasRate at zeta = 0 is 1/h", F( soft.biasRate ), 240.0, 0.05 );
	}
}

static void test_speed_caps( void )
{
	printf( "speed caps\n" );

	b3WorldId worldId = makeStepWorld( V( 0, 0, 0 ) );
	b3World* world = b3GetWorldFromId( worldId );
	b3World_SetMaximumLinearSpeed( worldId, b3fFromInt( 10 ) );

	b3BodyDef def = b3DefaultBodyDef();
	def.type = b3_dynamicBody;
	def.enableSleep = false;
	b3BodyId bodyId = b3CreateBody( worldId, &def );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3Sphere sphere = { V( 0, 0, 0 ), b3fFromDouble( 0.5 ) };
	b3CreateSphereShape( bodyId, &shapeDef, &sphere );

	b3Body_SetLinearVelocity( bodyId, V( 100, 0, 0 ) );
	b3World_Step( worldId, 1 );

	// The clamp is v * (maxSpeed / |v|), and that ratio is itself a Q12 value:
	// 10/100 is 0.1, which quantizes to 409/4096 and lands the result 1.5e-3
	// low. The cap is a numerical safety limit, and erring on the low side of
	// it is the harmless direction, so the tolerance is set from the ratio's
	// resolution rather than tightened.
	expect( "linear speed clamped", F( b3Body_GetLinearVelocity( bodyId ).x ), 10.0, 0.02 );
	check( "speed cap flagged", ( b3GetBodyFullId( world, bodyId )->flags & b3_isSpeedCapped ) != 0 );

	// The angular cap is B3_MAX_ROTATION * inv_dt, about 47 rad/s. Its square
	// is 2219, which at Q24 overflows int32 -- so this case is specifically
	// what a narrow comparison would fail, by wrapping and clamping everything.
	b3Body_SetLinearVelocity( bodyId, V( 0, 0, 0 ) );
	b3Body_SetAngularVelocity( bodyId, V( 0, 0, 200 ) );
	b3World_Step( worldId, 1 );

	double wz = F( b3Body_GetAngularVelocity( bodyId ).z );
	printf( "  angular speed clamped to %.3f rad/s\n", wz );
	// B3_MAX_ROTATION is pi/4 per step, so the cap is (pi/4) * 60 = 47.1 rad/s.
	// Dividing the brad count by half a circle instead of a whole one doubles
	// this to 94, which is what the first draft did -- and pi/2 per step is
	// exactly what B3_MAX_ROTATION's comment says breaks continuous collision.
	expect( "angular speed clamped to (pi/4)/dt", wz, 47.12, 0.2 );

	// A body well under both caps must not be touched.
	b3Body_SetAngularVelocity( bodyId, V( 0, 0, 1 ) );
	b3World_Step( worldId, 1 );
	expect( "slow body is left alone", F( b3Body_GetAngularVelocity( bodyId ).z ), 1.0, 1e-3 );

	validate( world );
	b3DestroyWorld( worldId );
}

static void test_motion_locks_integration( void )
{
	printf( "motion locks through the integrator\n" );

	b3WorldId worldId = makeStepWorld( V( 0, -10, 0 ) );
	b3World* world = b3GetWorldFromId( worldId );

	b3BodyDef def = b3DefaultBodyDef();
	def.type = b3_dynamicBody;
	def.enableSleep = false;
	def.motionLocks.linearY = true;
	def.motionLocks.angularX = true;
	b3BodyId bodyId = b3CreateBody( worldId, &def );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3Sphere sphere = { V( 0, 0, 0 ), b3fFromDouble( 0.5 ) };
	b3CreateSphereShape( bodyId, &shapeDef, &sphere );

	b3Body_SetLinearVelocity( bodyId, V( 3, 5, 7 ) );
	b3Body_SetAngularVelocity( bodyId, V( 2, 4, 6 ) );

	for ( int i = 0; i < 10; ++i )
	{
		b3World_Step( worldId, 4 );
	}

	// The lock is applied after integration, so the locked components are zero
	// and the body has not moved along y despite ten steps of gravity.
	expect( "locked linear y velocity is zero", F( b3Body_GetLinearVelocity( bodyId ).y ), 0.0, 1e-6 );
	expect( "locked linear y position unchanged", F( b3Body_GetPosition( bodyId ).y ), 0.0, 1e-3 );
	expect( "locked angular x is zero", F( b3Body_GetAngularVelocity( bodyId ).x ), 0.0, 1e-6 );

	check( "unlocked linear x survives", fabs( F( b3Body_GetLinearVelocity( bodyId ).x ) - 3.0 ) < 1e-2 );
	check( "unlocked angular z survives", fabs( F( b3Body_GetAngularVelocity( bodyId ).z ) - 6.0 ) < 1e-2 );

	validate( world );
	b3DestroyWorld( worldId );
}

static void test_sleeping_by_step_count( void )
{
	printf( "sleeping, by step count\n" );

	// B3_TIME_TO_SLEEP is 0.5 s and sleepTime accumulates dt per step, so a
	// motionless body must sleep on step 30 -- not merely eventually. Asserting
	// the step number is what would catch an accumulation bias in sleepTime,
	// which a "does it sleep at all" test cannot see.
	b3WorldId worldId = makeStepWorld( V( 0, 0, 0 ) );
	b3World* world = b3GetWorldFromId( worldId );

	b3BodyDef def = b3DefaultBodyDef();
	def.type = b3_dynamicBody;
	b3BodyId bodyId = b3CreateBody( worldId, &def );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3Sphere sphere = { V( 0, 0, 0 ), b3fFromDouble( 0.5 ) };
	b3CreateSphereShape( bodyId, &shapeDef, &sphere );

	int sleepStep = -1;
	bool reportedFellAsleep = false;
	for ( int i = 1; i <= 60 && sleepStep < 0; ++i )
	{
		b3World_Step( worldId, 4 );
		validate( world );

		if ( b3Body_IsAwake( bodyId ) == false )
		{
			sleepStep = i;

			b3BodyEvents events = b3World_GetBodyEvents( worldId );
			for ( int k = 0; k < events.moveCount; ++k )
			{
				if ( events.moveEvents[k].fellAsleep )
				{
					reportedFellAsleep = true;
				}
			}
		}
	}

	double expectedStep = 0.5 * 60.0;
	printf( "  fell asleep on step %d (B3_TIME_TO_SLEEP predicts %.0f)\n", sleepStep, expectedStep );
	check( "slept at the predicted step", sleepStep == (int)expectedStep || sleepStep == (int)expectedStep + 1 );
	check( "the move event reported fellAsleep", reportedFellAsleep );
	checkInt( "no awake bodies remain", b3World_GetAwakeBodyCount( worldId ), 0 );

	// Waking it puts it back in the awake set and it starts accumulating again.
	b3Body_SetAwake( bodyId, true );
	validate( world );
	check( "woken body is awake", b3Body_IsAwake( bodyId ) );
	checkInt( "one awake body again", b3World_GetAwakeBodyCount( worldId ), 1 );

	// A body that keeps moving never sleeps.
	b3Body_SetLinearVelocity( bodyId, V( 5, 0, 0 ) );
	for ( int i = 0; i < 60; ++i )
	{
		b3World_Step( worldId, 4 );
	}
	check( "a moving body stays awake", b3Body_IsAwake( bodyId ) );

	validate( world );
	b3DestroyWorld( worldId );
}

// =========================================================================
// Phase 3C-ii: the contact solver
// =========================================================================
//
// Everything above this line runs with no constraints solved. These are the
// first tests where two bodies meeting has any consequence at all.

/// A static ground box plus `count` dynamic boxes stacked above it, each half
/// extent 0.5, the lowest resting at y = 1.0 with a small gap.
///
/// Returns the world; body ids come back through `bodyIds`, ground first.
static b3WorldId makeStackWorld( int count, b3BodyId* bodyIds, double gap )
{
	// **Static, and that is not a style choice.** b3CreateHullShape aliases the
	// hull rather than copying it, exactly as pair_port.c's blob array notes,
	// so a hull local to this helper would be dangling by the time the caller
	// steps the world -- which shows up as a segfault deep in
	// b3BuildFaceAContact reading a garbage reference-face index, not as
	// anything that points here.
	//
	// Copying by assignment is safe: b3BoxHull addresses its arrays by byte
	// offset from its own header rather than by pointer, so a struct copy stays
	// self-consistent.
	static b3BoxHull s_groundHull;
	static b3BoxHull s_boxHull;

	b3WorldId worldId = makeStepWorld( V( 0, -10, 0 ) );

	b3BodyDef groundDef = b3DefaultBodyDef();
	groundDef.type = b3_staticBody;
	groundDef.position = V( 0, 0, 0 );
	bodyIds[0] = b3CreateBody( worldId, &groundDef );

	b3ShapeDef groundShape = b3DefaultShapeDef();
	s_groundHull = b3MakeBoxHull( b3fFromDouble( 4.0 ), b3fFromDouble( 0.5 ), b3fFromDouble( 4.0 ) );
	b3CreateHullShape( bodyIds[0], &groundShape, &s_groundHull.base );

	for ( int i = 0; i < count; ++i )
	{
		b3BodyDef def = b3DefaultBodyDef();
		def.type = b3_dynamicBody;
		def.position = V( 0, 1.0 + i * ( 1.0 + gap ), 0 );
		bodyIds[1 + i] = b3CreateBody( worldId, &def );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		s_boxHull = b3MakeBoxHull( b3fFromDouble( 0.5 ), b3fFromDouble( 0.5 ), b3fFromDouble( 0.5 ) );
		b3CreateHullShape( bodyIds[1 + i], &shapeDef, &s_boxHull.base );
	}

	return worldId;
}

static void test_box_on_ground( void )
{
	printf( "a box comes to rest on the ground\n" );

	// The first thing the solver has ever been asked to do. Before 3C-ii this
	// box fell through the ground and kept going.
	b3BodyId ids[2];
	b3WorldId worldId = makeStackWorld( 1, ids, 0.02 );
	b3World* world = b3GetWorldFromId( worldId );

	double minY = 1e9;
	for ( int i = 0; i < 180; ++i )
	{
		b3World_Step( worldId, 4 );
		double y = F( b3Body_GetPosition( ids[1] ).y );
		if ( y < minY )
		{
			minY = y;
		}
	}

	double restY = F( b3Body_GetPosition( ids[1] ).y );
	double speed = F( b3Length( b3Body_GetLinearVelocity( ids[1] ) ) );
	double slop = b3fToDouble( B3_LINEAR_SLOP );

	printf( "  rest y = %.6f (want 1.0), lowest %.6f, speed %.2e, slop %.5f\n", restY, minY, speed, slop );

	// The contact is at y = 1.0 exactly: ground top at 0.5, box half extent
	// 0.5. Soft contacts allow a slop of penetration by design, and the
	// speculative margin allows a little separation, so the window is stated
	// in slops rather than as an equality.
	check( "box rests on the ground rather than passing through", restY > 1.0 - 4.0 * slop );
	check( "box does not hover above the ground", restY < 1.0 + 4.0 * slop );

	// The stronger statement: it never dipped far below the rest height at any
	// point, so the solver caught it rather than letting it sink and pushing it
	// back out.
	check( "box never sank more than a few slops", minY > 1.0 - 8.0 * slop );

	check( "box has come to rest", speed < 0.05 );

	validate( world );
	b3DestroyWorld( worldId );
}

static void test_warm_starting( void )
{
	printf( "warm starting earns its keep\n" );

	// Without this the accumulators could be dead code and every test above
	// would still pass: the manifold impulses would be written, never read, and
	// the solver would converge anyway from a cold start each step. Running the
	// same scene both ways is what makes them observable.
	double penetration[2];

	for ( int mode = 0; mode < 2; ++mode )
	{
		b3BodyId ids[4];
		b3WorldId worldId = makeStackWorld( 3, ids, 0.02 );
		b3World* world = b3GetWorldFromId( worldId );
		world->enableWarmStarting = ( mode == 1 );

		for ( int i = 0; i < 120; ++i )
		{
			b3World_Step( worldId, 4 );
		}

		// The top box of a three-high stack is where warm starting shows,
		// because its contact has to hold up everything above the ground
		// through two intermediate contacts.
		penetration[mode] = 3.0 - F( b3Body_GetPosition( ids[3] ).y );

		validate( world );
		b3DestroyWorld( worldId );
	}

	printf( "  top box drop: cold %.6f, warm %.6f\n", penetration[0], penetration[1] );

	// Asserting the direction rather than a magnitude: warm starting is a
	// convergence aid, and how much it buys depends on the stack.
	check( "warm starting leaves the stack no lower than a cold start", penetration[1] <= penetration[0] + 1e-4 );
}

static void test_restitution( void )
{
	printf( "restitution against the drop height\n" );

	b3WorldId worldId = makeStepWorld( V( 0, -10, 0 ) );
	b3World* world = b3GetWorldFromId( worldId );

	b3BodyDef groundDef = b3DefaultBodyDef();
	groundDef.type = b3_staticBody;
	groundDef.position = V( 0, 0, 0 );
	b3BodyId groundId = b3CreateBody( worldId, &groundDef );

	b3ShapeDef groundShape = b3DefaultShapeDef();
	groundShape.baseMaterial.restitution = b3cFromFrac( 1, 2 );
	static b3BoxHull s_restitutionGround;
	s_restitutionGround = b3MakeBoxHull( b3fFromDouble( 4.0 ), b3fFromDouble( 0.5 ), b3fFromDouble( 4.0 ) );
	b3CreateHullShape( groundId, &groundShape, &s_restitutionGround.base );

	// Dropped from 2.5, so the sphere falls 1.5 before its surface meets the
	// ground at y = 1.0.
	const double dropHeight = 1.5;

	b3BodyDef def = b3DefaultBodyDef();
	def.type = b3_dynamicBody;
	def.position = V( 0, 2.5, 0 );
	def.enableSleep = false;
	b3BodyId ballId = b3CreateBody( worldId, &def );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.baseMaterial.restitution = b3cFromFrac( 1, 2 );
	b3Sphere sphere = { V( 0, 0, 0 ), b3fFromDouble( 0.5 ) };
	b3CreateSphereShape( ballId, &shapeDef, &sphere );

	// The apex after the first bounce. A coefficient of restitution e returns
	// e^2 of the drop height, so 0.5 predicts 0.375 -- but only after the ball
	// has actually left the ground, so the peak is taken after the bounce
	// rather than over the whole run.
	bool bounced = false;
	double apex = 0.0;
	for ( int i = 0; i < 240; ++i )
	{
		b3World_Step( worldId, 4 );
		double y = F( b3Body_GetPosition( ballId ).y );
		double vy = F( b3Body_GetLinearVelocity( ballId ).y );

		if ( bounced == false && vy > 0.0 && y < 1.2 )
		{
			bounced = true;
		}
		if ( bounced && y - 1.0 > apex )
		{
			apex = y - 1.0;
		}
		if ( bounced && vy < 0.0 && apex > 0.0 )
		{
			break;
		}
	}

	double predicted = 0.25 * dropHeight;
	printf( "  bounce apex %.4f, e^2 * h predicts %.4f (ratio %.3f)\n", apex, predicted, apex / predicted );

	check( "the ball bounced at all", bounced && apex > 0.05 );

	// Budgeted, not tuned. A sub-stepped solver applies restitution once per
	// step against the velocity at that moment, so it under-returns; upstream
	// has the same property. Half to double the closed form is the window this
	// asserts, which still separates 0.5 from 0 and from 1.
	check( "bounce height is within a factor of two of e^2 * h", apex > 0.5 * predicted && apex < 2.0 * predicted );

	validate( world );
	b3DestroyWorld( worldId );
}

static void test_box_stack( void )
{
	printf( "a five-box stack over 600 steps\n" );

	// The headline test. Five boxes, ten seconds, and the question is whether
	// the stack is still a stack -- neither sinking into the ground nor gaining
	// energy and throwing itself apart.
	b3BodyId ids[6];
	b3WorldId worldId = makeStackWorld( 5, ids, 0.01 );
	b3World* world = b3GetWorldFromId( worldId );

	double slop = b3fToDouble( B3_LINEAR_SLOP );

	for ( int i = 0; i < 600; ++i )
	{
		b3World_Step( worldId, 4 );
	}

	double lowest = F( b3Body_GetPosition( ids[1] ).y );
	double top = F( b3Body_GetPosition( ids[5] ).y );

	// Each box is one unit tall, so a settled stack has its boxes at 1, 2, 3,
	// 4, 5 minus whatever penetration the soft contacts allow.
	double sink = 5.0 - top;

	double maxSpeed = 0.0;
	double maxDrift = 0.0;
	for ( int b = 1; b <= 5; ++b )
	{
		double speed = F( b3Length( b3Body_GetLinearVelocity( ids[b] ) ) );
		if ( speed > maxSpeed )
		{
			maxSpeed = speed;
		}

		b3Vec3 p = b3Body_GetPosition( ids[b] );
		double drift = sqrt( F( p.x ) * F( p.x ) + F( p.z ) * F( p.z ) );
		if ( drift > maxDrift )
		{
			maxDrift = drift;
		}
	}

	printf( "  bottom %.4f, top %.4f, total sink %.4f (%.1f slops), max speed %.2e, max lateral drift %.4f\n", lowest,
			top, sink, sink / slop, maxSpeed, maxDrift );

	check( "the stack has not collapsed into the ground", lowest > 1.0 - 8.0 * slop );

	// Five contacts each allowed a slop of penetration is the honest budget,
	// with headroom for the sub-stepping.
	check( "total sink is bounded by the per-contact slop", sink < 20.0 * slop );

	// The energy test. A stack that has gained energy is the classic
	// fixed-point solver failure -- an impulse that overshoots feeds the next
	// step -- and it shows as motion that never stops.
	check( "the stack is at rest, not jittering", maxSpeed < 0.05 );

	// And it is still a stack rather than a heap.
	check( "the boxes have not slid off each other", maxDrift < 0.25 );

	validate( world );
	b3DestroyWorld( worldId );
}

// Counts allocations while a world runs. NEA_Phys3DWorldCreate installs a pool
// allocator with the same shape on the device -- see source/NEAPhysics3D.c --
// and this is the invariant that allocator asserts on.
static int s_allocCount;
static int s_allocBytes;
static bool s_allocCounting;

static void* countingAlloc( int32_t size, int32_t alignment )
{
	if ( s_allocCounting )
	{
		s_allocCount++;
		s_allocBytes += size;
	}

	void* mem = NULL;
	if ( posix_memalign( &mem, (size_t)( alignment < 8 ? 8 : alignment ), (size_t)size ) != 0 )
	{
		return NULL;
	}
	return mem;
}

static void countingFree( void* mem )
{
	free( mem );
}

static void test_no_allocation_while_stepping( void )
{
	printf( "a settled world allocates nothing per step\n" );

	// What NEA_Phys3D's pool allocator needs to be true on the device.
	//
	// The invariant is deliberately *steady state*, not "never". Box3D's
	// containers grow on demand by design, and a scene reaching its working set
	// -- islands forming, contacts linking into the graph, the first island
	// falling asleep and taking a solver set with it -- legitimately allocates
	// a bounded amount on the way there. Demanding zero from step one would
	// mean pre-sizing every upstream container, which is a large change to a
	// transliterated port whose value rests on staying diffable.
	//
	// What a game actually cannot tolerate is a *running* simulation reaching
	// the allocator every frame. So this measures two phases and holds the
	// second to zero:
	//
	//   warm-up   the scene settles and sleeps; growth here is reported so it
	//             can be budgeted into NEA_Phys3DWorldDef::poolBytes
	//   steady    the same scene, more steps, and nothing may allocate
	//
	// Two bugs were found by exactly this. b3AllocateManifolds created its
	// block allocator lazily, so the first contact cost **479 KB** on a machine
	// with 4 MB; and b3DynamicTree_Rebuild took its scratch on first use inside
	// the step. Both are now taken in b3CreateWorld.
	b3SetAllocator( countingAlloc, countingFree );

	s_allocCount = 0;
	s_allocBytes = 0;
	s_allocCounting = false;

	b3BodyId ids[6];
	b3WorldId worldId = makeStackWorld( 5, ids, 0.01 );
	b3World* world = b3GetWorldFromId( worldId );

	// Scene construction is allowed to allocate; it happens before the pool is
	// sealed on the device. Count from the first step.
	s_allocCounting = true;

	// Warm-up. 240 steps is four seconds -- long enough for the stack to
	// settle, sleep, and for every container it touches to reach its size.
	for ( int i = 0; i < 240; ++i )
	{
		b3World_Step( worldId, 4 );
	}

	int warmAllocs = s_allocCount;
	int warmBytes = s_allocBytes;

	s_allocCount = 0;
	s_allocBytes = 0;

	// Steady state. The stack is asleep; wake it once so the run exercises the
	// solver rather than the early-out, and let it settle again -- so a
	// sleep/wake cycle is inside the window that must not allocate.
	b3Body_SetAwake( ids[1], true );

	for ( int i = 0; i < 240; ++i )
	{
		b3World_Step( worldId, 4 );
	}

	int steadyAllocs = s_allocCount;
	int steadyBytes = s_allocBytes;

	printf( "  warm-up (240 steps): %d allocations, %d bytes\n", warmAllocs, warmBytes );
	printf( "  steady  (240 steps, one sleep/wake cycle): %d allocations, %d bytes\n", steadyAllocs, steadyBytes );
	printf( "  per-step stack: %d of %d bytes reserved\n", b3GetMaxStackAllocation( &world->stack ),
			b3GetStackCapacity( &world->stack ) );

	check( "a settled world does not allocate per step", steadyAllocs == 0 );

	// Warm-up is allowed but must stay small. The regression this guards is a
	// return of the lazily-created manifold pool, which was two orders of
	// magnitude above this bound.
	check( "warm-up growth stays bounded", warmBytes < 64 * 1024 );

	// The reserve has to be an over-estimate, not an estimate: b3GrowStack is
	// what runs when it is not, and that is an allocation.
	check( "the per-step stack was reserved large enough",
		   b3GetMaxStackAllocation( &world->stack ) <= b3GetStackCapacity( &world->stack ) );

	s_allocCounting = false;

	validate( world );
	b3DestroyWorld( worldId );

	b3SetAllocator( NULL, NULL );
}

/// The same invariant for a scene carrying joints.
///
/// Worth its own case since Phase 6 Stage 2: a b3JointSim went from 184 bytes
/// to 300 when the per-type union landed, and it is copied between four homes
/// -- the graph colour, the awake set, a sleeping set and the disabled set --
/// as its bodies sleep and wake. The sleeping-set reserve in b3TrySleepIsland
/// is a *mid-step* reserve sized from the island's joint count, so if
/// b3Capacity::jointCount is under-declared the growth lands inside the step,
/// which is exactly what a game cannot afford.
static void test_no_allocation_with_joints( void )
{
	printf( "a settled world with joints allocates nothing per step\n" );

	b3SetAllocator( countingAlloc, countingFree );

	s_allocCount = 0;
	s_allocBytes = 0;
	s_allocCounting = false;

	b3WorldDef def = b3DefaultWorldDef();
	def.capacity.staticBodyCount = 4;
	def.capacity.dynamicBodyCount = 8;
	def.capacity.staticShapeCount = 4;
	def.capacity.dynamicShapeCount = 8;
	def.capacity.contactCount = 16;
	def.capacity.jointCount = 8;
	def.gravity = V( 0, -10, 0 );
	b3WorldId worldId = b3CreateWorld( &def );
	b3World* world = b3GetWorldFromId( worldId );

	// A four-link chain hung from a static anchor: four joints, one island,
	// and it settles into a straight vertical line and sleeps.
	b3BodyDef anchorDef = b3DefaultBodyDef();
	anchorDef.type = b3_staticBody;
	anchorDef.position = V( 0, 0, 0 );
	b3BodyId previous = b3CreateBody( worldId, &anchorDef );

	b3BodyId links[4];
	for ( int i = 0; i < 4; ++i )
	{
		b3BodyDef linkDef = b3DefaultBodyDef();
		linkDef.type = b3_dynamicBody;
		linkDef.position = V( 0, -1.0 * ( i + 1 ), 0 );
		links[i] = b3CreateBody( worldId, &linkDef );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		b3Sphere ball = { V( 0, 0, 0 ), b3fFromDouble( 0.2 ) };
		b3CreateSphereShape( links[i], &shapeDef, &ball );

		b3DistanceJointDef jointDef = b3DefaultDistanceJointDef();
		jointDef.base.bodyIdA = previous;
		jointDef.base.bodyIdB = links[i];
		jointDef.length = b3fFromInt( 1 );
		b3CreateDistanceJoint( worldId, &jointDef );

		previous = links[i];
	}
	validate( world );

	// Construction may allocate; the pool is sealed after it on the device.
	s_allocCounting = true;

	for ( int i = 0; i < 480; ++i )
	{
		b3World_Step( worldId, 4 );
	}

	int warmAllocs = s_allocCount;
	int warmBytes = s_allocBytes;

	s_allocCount = 0;
	s_allocBytes = 0;

	// The chain is asleep. Waking it moves four joint sims back into the graph
	// colour and settling puts them back -- the migration this case exists for.
	b3Body_SetAwake( links[3], true );

	for ( int i = 0; i < 480; ++i )
	{
		b3World_Step( worldId, 4 );
	}

	printf( "  warm-up (480 steps, 4 joints): %d allocations, %d bytes\n", warmAllocs, warmBytes );
	printf( "  steady  (480 steps, one sleep/wake cycle): %d allocations, %d bytes\n", s_allocCount, s_allocBytes );

	check( "a settled jointed world does not allocate per step", s_allocCount == 0 );
	check( "and its warm-up growth stays bounded", warmBytes < 64 * 1024 );

	s_allocCounting = false;

	validate( world );
	b3DestroyWorld( worldId );

	b3SetAllocator( NULL, NULL );
}

// Every manifold size class a contact can ask for exists before the first step.
//
// This is the mesh-shaped twin of the bug above. A convex contact takes one
// manifold, so size class 0 was the only one any scene reached and the higher
// classes were created lazily -- inside the step, from a b3CreateBlockAllocator
// call that is a heap allocation however small the block is. A mesh contact
// takes B3_NEA_MAX_MESH_MANIFOLDS, so it reaches the top class on the first
// frame a body touches the level.
//
// There is no mesh narrow phase yet, so the allocator is driven directly. That
// is the point: the guarantee is a property of b3CreateWorld, and it can and
// should be tested before the code that depends on it exists.
static void test_manifold_capacity_is_preallocated( void )
{
	printf( "every manifold size class is reserved at world creation\n" );

	b3SetAllocator( countingAlloc, countingFree );

	s_allocCount = 0;
	s_allocBytes = 0;
	s_allocCounting = false;

	b3WorldDef def = b3DefaultWorldDef();
	def.capacity.staticBodyCount = 1;
	def.capacity.dynamicBodyCount = 4;
	def.capacity.staticShapeCount = 1;
	def.capacity.dynamicShapeCount = 4;
	def.capacity.contactCount = 16;
	def.capacity.meshContactCount = 4;
	def.gravity = V( 0, 0, 0 );

	b3WorldId worldId = b3CreateWorld( &def );
	b3World* world = b3GetWorldFromId( worldId );

	checkInt( "one allocator per manifold size class", world->manifoldAllocators.count, B3_NEA_MAX_MESH_MANIFOLDS );

	// Creation is allowed to allocate. Everything after this point is what the
	// device's sealed pool allocator would refuse.
	s_allocCounting = true;

	// Take and return the declared mesh contacts' worth, twice over, at the
	// full cap. The second round also proves the free list is doing its job:
	// returning an element and taking it again must not reach b3Alloc.
	b3Manifold* held[4];

	for ( int round = 0; round < 2; ++round )
	{
		for ( int i = 0; i < 4; ++i )
		{
			held[i] = b3AllocateManifolds( world, B3_NEA_MAX_MESH_MANIFOLDS );
			check( "the mesh size class handed out an element", held[i] != NULL );
			s_checks--; // counted once below rather than eight times
		}

		for ( int i = 0; i < 4; ++i )
		{
			b3FreeManifolds( world, held[i], B3_NEA_MAX_MESH_MANIFOLDS );
		}
	}

	s_checks++;

	// And the convex class, which shares the array with them.
	b3Manifold* one = b3AllocateManifolds( world, 1 );
	check( "the convex size class handed out an element", one != NULL );
	b3FreeManifolds( world, one, 1 );

	printf( "  %d allocations, %d bytes for %d mesh manifolds at capacity %d\n", s_allocCount, s_allocBytes, 4,
			B3_NEA_MAX_MESH_MANIFOLDS );

	check( "no manifold size class was created on demand", s_allocCount == 0 );

	s_allocCounting = false;

	validate( world );
	b3DestroyWorld( worldId );

	b3SetAllocator( NULL, NULL );
}

// =========================================================================
// Triangle meshes
// =========================================================================
//
// A level, rather than a box pretending to be one. What these test is the part
// of the mesh narrow phase that only a *stepping* world can see: whether the
// clusters it produces hold a body up, whether the warm-start impulses survive
// a body sliding from one triangle to the next, and whether the ghost filter
// really stops a body catching on the interior edges of a flat floor.
//
// The blobs come from the baker (tests/box3d_host/mesh_bake.c) rather than
// being written by hand as test_collision.c's do. The two files are asking
// different questions: test_collision verifies the *reader*, and there a blob
// checked against its own writer would agree with itself no matter what either
// did; here the blob is a fixture and hand-rolling one means reimplementing the
// baker's median split where nothing would check it.

/// A baked mesh grid, sized for the widest build.
///
/// Under B3_FIXED_DEBUG every fixed value carries a shadow double, so a b3Vec3
/// is 48 bytes rather than 12 and the blob is four times the size. Sized for
/// that, not for device mode, which is how test_collision.c's meshBlob arrived
/// at its own number.
typedef struct
{
	char bytes[192 * 1024];
	double align;
} gridBlob;

/// Build an n x n quad grid over [-half, half]^2 in the XZ plane, with the
/// height of each vertex given by `height`, and bake it.
///
/// `height == NULL` is a flat floor: every interior edge is then coplanar and
/// therefore flat, which is exactly the configuration the ghost filter has to
/// handle and the one a body catches on when it does not.
static const b3MeshData* buildGrid( gridBlob* blob, int n, double half, double ( *height )( double x, double z ) )
{
	static pdMesh desc;
	memset( &desc, 0, sizeof( desc ) );

	int side = n + 1;
	B3_ASSERT( side * side <= PD_MAX_MESH_VERTICES );
	B3_ASSERT( 2 * n * n <= PD_MAX_MESH_TRIANGLES );

	desc.vertexCount = side * side;
	for ( int iz = 0; iz < side; ++iz )
	{
		for ( int ix = 0; ix < side; ++ix )
		{
			double x = -half + 2.0 * half * ix / n;
			double z = -half + 2.0 * half * iz / n;
			double y = height != NULL ? height( x, z ) : 0.0;
			desc.vertices[iz * side + ix] = ( pdVec3 ){ x, y, z };
		}
	}

	desc.triangleCount = 2 * n * n;
	int t = 0;
	for ( int iz = 0; iz < n; ++iz )
	{
		for ( int ix = 0; ix < n; ++ix )
		{
			int v00 = iz * side + ix;
			int v10 = v00 + 1;
			int v01 = v00 + side;
			int v11 = v01 + 1;

			// Counter-clockwise seen from +Y, so the face normals point up and
			// a body above the grid meets the front side.
			desc.indices[3 * t + 0] = v00;
			desc.indices[3 * t + 1] = v01;
			desc.indices[3 * t + 2] = v10;
			t += 1;

			desc.indices[3 * t + 0] = v10;
			desc.indices[3 * t + 1] = v01;
			desc.indices[3 * t + 2] = v11;
			t += 1;
		}
	}

	int bytes = pdBakeMesh( &desc, blob, (int)sizeof( blob->bytes ) );
	check( "the grid baked", bytes > 0 );
	if ( bytes == 0 )
	{
		return NULL;
	}

	return (const b3MeshData*)blob;
}

static double rampHeight( double x, double z )
{
	B3_UNUSED( z );

	// A 20-degree ramp over the negative half, flat over the positive half.
	// tan(20 deg) = 0.364, comfortably above any friction this uses.
	return x < 0.0 ? -x * 0.364 : 0.0;
}

/// A world with a mesh floor. The blob is `static` for the reason
/// makeStackWorld's hulls are: b3CreateMeshShape aliases it.
static b3WorldId makeMeshWorld( const b3MeshData** meshOut, int n, double half, double ( *height )( double x, double z ) )
{
	static gridBlob s_blob;

	b3WorldDef def = b3DefaultWorldDef();
	def.capacity.staticBodyCount = 8;
	def.capacity.dynamicBodyCount = 16;
	def.capacity.staticShapeCount = 8;
	def.capacity.dynamicShapeCount = 16;
	def.capacity.contactCount = 32;

	// Without this the mesh size class and the triangle-cache allocator reserve
	// nothing, and the first mesh contact allocates mid-step.
	def.capacity.meshContactCount = 4;
	def.gravity = V( 0, -10, 0 );

	b3WorldId worldId = b3CreateWorld( &def );

	const b3MeshData* mesh = buildGrid( &s_blob, n, half, height );
	if ( mesh == NULL )
	{
		*meshOut = NULL;
		return worldId;
	}

	b3BodyDef groundDef = b3DefaultBodyDef();
	groundDef.type = b3_staticBody;
	groundDef.position = V( 0, 0, 0 );
	b3BodyId groundId = b3CreateBody( worldId, &groundDef );

	b3ShapeDef groundShape = b3DefaultShapeDef();
	b3CreateMeshShape( groundId, &groundShape, mesh, V( 1, 1, 1 ) );

	*meshOut = mesh;
	return worldId;
}

static b3BodyId addMeshWorldBox( b3WorldId worldId, b3Vec3 position, double halfExtent, double friction )
{
	static b3BoxHull s_boxHull;

	b3BodyDef def = b3DefaultBodyDef();
	def.type = b3_dynamicBody;
	def.position = position;
	b3BodyId bodyId = b3CreateBody( worldId, &def );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.baseMaterial.friction = b3cFromFrac( (int)( friction * 100.0 + 0.5 ), 100 );
	s_boxHull = b3MakeBoxHull( b3fFromDouble( halfExtent ), b3fFromDouble( halfExtent ), b3fFromDouble( halfExtent ) );
	b3CreateHullShape( bodyId, &shapeDef, &s_boxHull.base );

	return bodyId;
}

static void test_mesh_rest( void )
{
	printf( "a box rests on a mesh floor\n" );

	// The first thing the mesh narrow phase has ever been asked to do. A 0.5
	// half-extent box spans four triangles of this grid, and every one of them
	// has the same normal, so the whole contact must reduce to *one* cluster --
	// which is the argument B3_NEA_MAX_MESH_MANIFOLDS is sized on.
	const b3MeshData* mesh = NULL;
	b3WorldId worldId = makeMeshWorld( &mesh, 8, 4.0, NULL );
	b3World* world = b3GetWorldFromId( worldId );
	if ( mesh == NULL )
	{
		b3DestroyWorld( worldId );
		return;
	}

	b3BodyId boxId = addMeshWorldBox( worldId, V( 0.35, 1.2, 0.35 ), 0.5, 0.6 );

	double minY = 1e9;
	int maxManifolds = 0;
	int maxPoints = 0;

	for ( int i = 0; i < 240; ++i )
	{
		b3World_Step( worldId, 4 );

		double y = b3fToDouble( b3Body_GetPosition( boxId ).y );
		if ( y < minY )
		{
			minY = y;
		}

		for ( int c = 0; c < world->contacts.count; ++c )
		{
			const b3Contact* contact = world->contacts.data + c;
			if ( contact->contactId == B3_NULL_INDEX || ( contact->flags & b3_simMeshContact ) == 0 )
			{
				continue;
			}

			if ( contact->manifoldCount > maxManifolds )
			{
				maxManifolds = contact->manifoldCount;
			}

			for ( int m = 0; m < contact->manifoldCount; ++m )
			{
				if ( contact->manifolds[m].pointCount > maxPoints )
				{
					maxPoints = contact->manifolds[m].pointCount;
				}
			}
		}
	}

	// The cull's output, not just its point count. Four triangles each hand the
	// cluster up to four points, so b3CullPoints is reducing about sixteen to
	// four -- and the four it keeps must be the ones that hold the patch open.
	// A reducer that kept four coincident points would still report four and
	// would still let the box rest; it would fall over the moment the box was
	// tipped. So the surviving quad has to span the box's own footprint.
	double patchSpan = 0.0;
	for ( int c = 0; c < world->contacts.count; ++c )
	{
		const b3Contact* contact = world->contacts.data + c;
		if ( contact->contactId == B3_NULL_INDEX || contact->manifoldCount == 0 )
		{
			continue;
		}

		const b3Manifold* mm = contact->manifolds;
		for ( int i = 0; i < mm->pointCount; ++i )
		{
			for ( int j = i + 1; j < mm->pointCount; ++j )
			{
				double d = F( b3Length( b3Sub( mm->points[i].anchorA, mm->points[j].anchorA ) ) );
				if ( d > patchSpan )
				{
					patchSpan = d;
				}
			}
		}
	}

	double restY = b3fToDouble( b3Body_GetPosition( boxId ).y );
	double speed = F( b3Length( b3Body_GetLinearVelocity( boxId ) ) );

	printf( "  rest y = %.6f (want 0.5), lowest %.6f, clusters %d, points %d, patch span %.4f, awake %d\n", restY, minY,
			maxManifolds, maxPoints, patchSpan, (int)b3Body_IsAwake( boxId ) );

	// The floor is at y = 0 and the box is 0.5 deep. Two slops of tolerance:
	// one for the solver's own resting penetration, one for the rest offset the
	// mesh path subtracts, which upstream accepts as a small visual gap.
	double slop = b3fToDouble( B3_LINEAR_SLOP );
	check( "the box rests on the mesh floor", fabs( restY - 0.5 ) < 3.0 * slop );
	check( "the box never fell through", minY > 0.5 - 3.0 * slop );
	check( "the box came to rest", speed < 0.01 );
	checkInt( "four coplanar triangles make one cluster", maxManifolds, 1 );
	check( "the cluster is a full quad", maxPoints == 4 );

	// The box is 1.0 across, so its footprint's diagonal is 1.41. Anything much
	// under that means the reducer kept points from one corner of the patch.
	check( "the reduced quad spans the box footprint", patchSpan > 1.2 );
	check( "the box went to sleep", b3Body_IsAwake( boxId ) == false );

	validate( world );
	b3DestroyWorld( worldId );
}

static void test_mesh_ramp( void )
{
	printf( "a box slides down a mesh ramp\n" );

	const b3MeshData* mesh = NULL;
	b3WorldId worldId = makeMeshWorld( &mesh, 8, 4.0, rampHeight );
	b3World* world = b3GetWorldFromId( worldId );
	if ( mesh == NULL )
	{
		b3DestroyWorld( worldId );
		return;
	}

	// Started high on the ramp, with friction well below tan(20 deg) = 0.364 so
	// that it must slide, and must then stop on the flat rather than sail off.
	b3BodyId boxId = addMeshWorldBox( worldId, V( -3.0, 1.6, 0.0 ), 0.4, 0.15 );

	for ( int i = 0; i < 480; ++i )
	{
		b3World_Step( worldId, 4 );
	}

	b3Vec3 p = b3Body_GetPosition( boxId );
	double x = b3fToDouble( p.x );
	double y = b3fToDouble( p.y );
	double speed = F( b3Length( b3Body_GetLinearVelocity( boxId ) ) );

	printf( "  slid from x=-3.00 to x=%.4f, y=%.4f, speed %.4f\n", x, y, speed );

	check( "the box slid down the ramp", x > -2.0 );
	check( "the box stayed on the level", x < 4.0 );
	check( "the box did not sink through the ramp", y > 0.2 );
	check( "the box stopped on the flat", speed < 0.05 );

	validate( world );
	b3DestroyWorld( worldId );
}

static void test_mesh_internal_edge( void )
{
	printf( "a box crosses an internal edge without catching\n" );

	// The failure mode this whole feature is most likely to ship with. A flat
	// grid's interior edges are coplanar, so every one of them is a b3_flatEdge
	// and the ghost filter must drop the edge contacts the neighbouring
	// triangle already claimed. When it does not, the box hits each seam like a
	// step: it loses speed, and it gets a vertical kick.
	//
	// So this measures both. A box given a push across six triangle seams must
	// keep *all* of its speed and must not be launched.
	//
	// @section rim Why the grid is much bigger than the traverse
	//
	// The first version of this crossed an 8x8 grid over [-4,4] from x=-3 to
	// x=+3 and failed hard -- 62% of the speed gone, a 1.3 m/s upward kick. The
	// cause was not the interior at all: the *boundary* column's outer edges
	// have only one incident triangle, so the baker cannot call them flat, and
	// the ghost filter therefore -- correctly -- keeps their contacts. The box's
	// speculative query bounds reach 0.34 units past its own surface, which was
	// enough to pull the rim column in and collide against a free edge.
	//
	// That is upstream's behaviour and it is right: a mesh with an open rim has
	// an open rim, and a level whose floor simply stops needs a wall or a skirt
	// there. What it is not is an interior-edge ghost collision, so the test
	// keeps the traverse three units clear of the boundary and measures the
	// thing it is named after.
	const b3MeshData* mesh = NULL;
	b3WorldId worldId = makeMeshWorld( &mesh, 16, 8.0, NULL );
	b3World* world = b3GetWorldFromId( worldId );
	if ( mesh == NULL )
	{
		b3DestroyWorld( worldId );
		return;
	}

	// Friction is zero, so on a genuinely flat floor the horizontal speed is
	// conserved exactly and anything lost was lost to a seam.
	b3BodyId boxId = addMeshWorldBox( worldId, V( -3.0, 0.55, 0.0 ), 0.25, 0.0 );

	// Settle first, so the measurement starts from a resting contact rather
	// than from the drop.
	for ( int i = 0; i < 60; ++i )
	{
		b3World_Step( worldId, 4 );
	}

	const double launchSpeed = 4.0;
	b3Body_SetLinearVelocity( boxId, V( launchSpeed, 0, 0 ) );

	double maxUpward = 0.0;
	double maxY = 0.0;

	for ( int i = 0; i < 90; ++i )
	{
		b3World_Step( worldId, 4 );

		double vy = b3fToDouble( b3Body_GetLinearVelocity( boxId ).y );
		if ( vy > maxUpward )
		{
			maxUpward = vy;
		}

		double y = b3fToDouble( b3Body_GetPosition( boxId ).y );
		if ( y > maxY )
		{
			maxY = y;
		}
	}

	double vx = b3fToDouble( b3Body_GetLinearVelocity( boxId ).x );
	double x = b3fToDouble( b3Body_GetPosition( boxId ).x );

	printf( "  crossed to x=%.4f keeping vx=%.4f of %.1f, peak vy %.4f, peak y %.4f (rest 0.25)\n", x, vx, launchSpeed,
			maxUpward, maxY );

	// Six seams are crossed in 90 steps at 4 units/s. With zero friction on a
	// flat floor the horizontal speed is conserved *exactly*, so this is a
	// tight bound rather than a generous one -- anything the seams take shows
	// up immediately.
	check( "the box crossed several triangle seams", x > 0.0 );
	check( "no seam ate the horizontal speed", vx > 0.99 * launchSpeed );
	check( "no seam kicked the box upward", maxUpward < 0.05 );
	check( "the box stayed on the floor", maxY < 0.25 + 4.0 * b3fToDouble( B3_LINEAR_SLOP ) );

	validate( world );
	b3DestroyWorld( worldId );
}

static void test_no_allocation_while_stepping_on_a_mesh( void )
{
	printf( "a world resting on a mesh allocates nothing per step\n" );

	// The mesh-shaped twin of test_no_allocation_while_stepping, and the reason
	// the triangle cache is a fixed-capacity block rather than the b3Array
	// upstream grows: a mesh contact re-queries the BVH whenever the body
	// leaves its cached bounds, which for a sliding body is most frames.
	b3SetAllocator( countingAlloc, countingFree );

	s_allocCount = 0;
	s_allocBytes = 0;
	s_allocCounting = false;

	const b3MeshData* mesh = NULL;
	b3WorldId worldId = makeMeshWorld( &mesh, 8, 4.0, NULL );
	b3World* world = b3GetWorldFromId( worldId );
	if ( mesh == NULL )
	{
		b3DestroyWorld( worldId );
		b3SetAllocator( NULL, NULL );
		return;
	}

	// One body, deliberately. Several would keep splitting and merging islands
	// as they slid past each other, and a *new* island shape wants a sleeping
	// solver set whose arrays have never been grown -- which allocates once,
	// legitimately, and has nothing to do with meshes. One body is one island
	// forever, and one mesh contact exercises the whole narrow phase anyway.
	b3BodyId boxId = addMeshWorldBox( worldId, V( -1.0, 1.0, 0.0 ), 0.4, 0.3 );

	s_allocCounting = true;

	// The warm-up has to contain the *same shape of event* the steady window
	// does, not merely the same number of steps. Pushing a body away from the
	// others splits the island, and a new island wants a sleeping solver set
	// whose arrays have never been grown -- which allocates once, legitimately,
	// the first time that island exists. Doing it here means the second one
	// finds the set recycled and its capacity already paid for.
	for ( int i = 0; i < 120; ++i )
	{
		b3World_Step( worldId, 4 );
	}

	for ( int pass = 0; pass < 2; ++pass )
	{
		b3Body_SetAwake( boxId, true );
		b3Body_SetLinearVelocity( boxId, V( pass == 0 ? 2.0 : -2.0, 0, 0 ) );

		for ( int i = 0; i < 120; ++i )
		{
			b3World_Step( worldId, 4 );
		}
	}

	int warmAllocs = s_allocCount;
	int warmBytes = s_allocBytes;

	s_allocCount = 0;
	s_allocBytes = 0;

	// Now the same again, and this time nothing may allocate. The push is what
	// makes this test about meshes rather than about sleeping: a body *moving*
	// across the level leaves its cached query bounds every few steps, which is
	// what drives b3RefreshCache's BVH query and the triangle-cache merge join.
	// A resting contact would be answered by the recycling fast path and never
	// reach the mesh narrow phase at all.
	for ( int pass = 0; pass < 2; ++pass )
	{
		b3Body_SetAwake( boxId, true );
		b3Body_SetLinearVelocity( boxId, V( pass == 0 ? 2.0 : -2.0, 0, 0 ) );

		for ( int i = 0; i < 120; ++i )
		{
			b3World_Step( worldId, 4 );
		}
	}

	printf( "  warm-up (360 steps, two slides): %d allocations, %d bytes\n", warmAllocs, warmBytes );
	printf( "  steady  (240 steps, the same two slides): %d allocations, %d bytes\n", s_allocCount, s_allocBytes );
	printf( "  arena: peak %d of %d bytes, mesh demand %d\n", world->arena.shared->peakDemand, world->arena.capacity,
			b3MeshContactArenaDemand() );

	check( "a mesh world does not allocate per step", s_allocCount == 0 );

	// The demand is what b3CreateWorld sized the arena from, so a peak above it
	// means b3ComputeMeshManifolds bumped something the demand does not account
	// for -- which on the device is a heap allocation inside the step.
	check( "the arena demand covers what the narrow phase used", world->arena.shared->peakDemand <= b3MeshContactArenaDemand() );

	s_allocCounting = false;

	validate( world );
	b3DestroyWorld( worldId );

	b3SetAllocator( NULL, NULL );
}

/// A body pushed off the world and left to fall, as the examples' player does.
///
/// This is the box3d_basic scenario reduced to what breaks it. That example's
/// floor is a box of half-extent 8 with **no walls**, and holding a direction
/// applies 40 N to one unit-mass cube -- so the player's box leaves the floor in
/// seconds and then falls with nothing to stop it, while the force keeps being
/// applied. Measured on device: 2,400 units out after 30 seconds, 127,000 after
/// six minutes, still climbing at the 400 u/s speed cap.
///
/// Nothing in the port bounds this. b3f is Q19.12, so a coordinate **wraps** at
/// 524,288 units -- about 22 minutes of falling, which is exactly the "it
/// softlocks after a while, sometimes" the examples show. Long before that the
/// body is past B3_HUGE (2,000), the bound every range comment in the manifold
/// and SAT code is written against.
///
/// So this test drives the same thing without the 60 Hz clock: it steps until
/// the body has fallen far enough to wrap, checking on every step that the
/// world's structures are still coherent. Under MODE=debug the shadow-value
/// checker is armed as well, and aborts on the first saturating operation.
static void test_body_pushed_out_of_the_world( void )
{
	printf( "a body pushed out of the world\n" );

	// Built here rather than from makeStackWorld, because two of that helper's
	// choices suppress the very thing being reproduced:
	//
	//   - it stacks the boxes vertically, so ids[1] is at the *bottom* under
	//     seven others, while the example spreads them across the floor;
	//   - it takes b3DefaultShapeDef's density, which is **1000**. That makes a
	//     unit box weigh a tonne, and 40 N against ~6 kN of static friction
	//     moves nothing at all. The example passes density 1.0.
	//
	// So this is box3d_basic's own geometry: a floor of half-extents
	// (8, 0.25, 8) and one 1 kg cube, pushed with the same 40 N.
	static b3BoxHull s_floorHull;
	static b3BoxHull s_boxHull;

	b3WorldId worldId = makeStepWorld( V( 0, -9.8, 0 ) );
	b3World* world = b3GetWorldFromId( worldId );

	b3BodyDef floorDef = b3DefaultBodyDef();
	floorDef.type = b3_staticBody;
	floorDef.position = V( 0, -0.25, 0 );
	b3BodyId floorId = b3CreateBody( worldId, &floorDef );

	b3ShapeDef floorShape = b3DefaultShapeDef();
	s_floorHull = b3MakeBoxHull( b3fFromDouble( 8.0 ), b3fFromDouble( 0.25 ), b3fFromDouble( 8.0 ) );
	b3CreateHullShape( floorId, &floorShape, &s_floorHull.base );

	b3BodyDef boxDef = b3DefaultBodyDef();
	boxDef.type = b3_dynamicBody;
	boxDef.position = V( 0, 1.0, 0 );
	b3BodyId player = b3CreateBody( worldId, &boxDef );

	b3ShapeDef boxShape = b3DefaultShapeDef();
	boxShape.density = b3fFromDouble( 1.0 );
	s_boxHull = b3MakeBoxHull( b3fFromDouble( 0.5 ), b3fFromDouble( 0.5 ), b3fFromDouble( 0.5 ) );
	b3CreateHullShape( player, &boxShape, &s_boxHull.base );

	// Long enough to reach both cliffs if nothing stops the body: the AABB
	// centre overflow at 2^18 = 262,144 units, and the position wrap at 2^19.
	// At the 400 u/s speed cap and 60 Hz that is about 40,000 and 80,000 steps.
	int steps = 0;
	int maxSteps = 120000;
	double worst = 0.0;
	int firstPastHuge = -1;
	int parkedAt = -1;
	b3Vec3 parkedPos = b3Vec3_zeroFn();
	int assertsBefore = b3TestUnexpectedAsserts();

	for ( ; steps < maxSteps; ++steps )
	{
		// The example's 40 N, always along +x, so the body leaves the floor and
		// keeps accelerating sideways as well as falling.
		//
		// Stopped once the body parks, which is the honest version of what the
		// player does: waking a parked body is *allowed* -- it just parks again
		// -- but hammering it with a wake every step measures the ratchet
		// rather than the bound. The second phase below is what checks that a
		// parked body left alone stays where it is.
		if ( parkedAt < 0 )
		{
			b3Body_ApplyForceToCenter( player, V( 40, 0, 0 ), true );
		}

		b3World_Step( worldId, 4 );

		b3Vec3 p = b3Body_GetPosition( player );
		double px = F( p.x ), py = F( p.y ), pz = F( p.z );

		double m = fabs( px );
		m = fabs( py ) > m ? fabs( py ) : m;
		m = fabs( pz ) > m ? fabs( pz ) : m;

		if ( m > worst )
		{
			worst = m;
		}

		// Stop on the *first* assertion. assert_trap.h keeps the run going, so
		// without this the output is thousands of copies of the same line and
		// the step that broke is buried.
		if ( b3TestUnexpectedAsserts() != assertsBefore )
		{
			printf( "    FIRST ASSERTION at step %d, pos (%.0f %.0f %.0f)\n", steps, px, py, pz );
			break;
		}

		// Parked: the body left the world and the solver stopped it where it
		// was. Keep stepping afterwards -- a body that parks and then drifts,
		// or that wakes itself, is as broken as one that never parked.
		if ( parkedAt < 0 && b3Body_IsAwake( player ) == false )
		{
			parkedAt = steps;
			parkedPos = p;
			printf( "    parked at step %d, pos (%.0f %.0f %.0f)\n", steps, px, py, pz );
		}

		if ( firstPastHuge < 0 && m > F( B3_HUGE ) )
		{
			firstPastHuge = steps;
		}

		// A wrap shows up as the magnitude *collapsing* after having been huge:
		// Q19.12 rolls from +524287 to -524288.
		// Every step, not every hundredth: the point is to catch corruption on
		// the step that creates it, not N steps later.
		validate( world );
	}

	double extent = F( b3DefaultWorldDef().maximumWorldExtent );

	printf( "  %d steps, worst coordinate %.0f units (extent %.0f), past B3_HUGE at step %d\n", steps,
			worst, extent, firstPastHuge );

	// It has to actually leave: a test where the box never slid off the floor
	// would pass everything below while proving nothing. This caught exactly
	// that -- makeStackWorld's density is 1000, so 40 N never moved the box.
	check( "the body leaves the world at all", firstPastHuge >= 0 );

	// The fix. Before it, the body fell until b3AABB_Center's sum overflowed at
	// 262,144 units and the broad phase started reporting invalid AABBs from
	// its own nodes; assert_trap.h turns those into the failure below.
	checkInt( "no assertion fired", b3TestUnexpectedAsserts() - assertsBefore, 0 );
	check( "the body was parked", parkedAt >= 0 );

	// Parked *near* the bound, not far past it. This is what says the check
	// runs every step rather than eventually: at 400 u/s a body covers 6.7
	// units per step, so anything beyond a few units of overshoot means it was
	// missed for a while.
	check( "parked close to the bound", worst < extent + 64.0 );

	// And nowhere near either cliff.
	check( "never reached the AABB centre overflow", worst < 262144.0 );

	// A parked body left alone does not drift. The loop above kept stepping
	// long after the park -- tens of thousands of steps -- so this is the check
	// that the body is genuinely out of the simulation rather than merely
	// moving slowly.
	b3Vec3 finalPos = b3Body_GetPosition( player );
	expect( "parked body did not drift in x", F( finalPos.x ), F( parkedPos.x ), 0.01 );
	expect( "parked body did not drift in y", F( finalPos.y ), F( parkedPos.y ), 0.01 );
	expect( "parked body did not drift in z", F( finalPos.z ), F( parkedPos.z ), 0.01 );

	// Still asleep, and still findable: parking does not destroy the body, so a
	// game can notice it and respawn it.
	check( "parked body is still asleep", b3Body_IsAwake( player ) == false );

	b3DestroyWorld( worldId );

	// ---------------------------------------------------------------------
	// What the bound is actually for
	// ---------------------------------------------------------------------
	//
	// The check above says the body stopped. This one says it stopped somewhere
	// the arithmetic is still correct, which is the whole point and is a
	// property of b3IsValidFloat rather than of any one call site: a b3f is
	// valid while |raw| < INT32_MAX/2, and functions like b3AABB_Center add two
	// coordinates relying on the sum fitting int32. A bound that parked bodies
	// outside that range would be no fix at all.
	check( "parked inside b3IsValidFloat's range", b3IsValidPosition( finalPos ) );
	check( "the bound is inside b3IsValidFloat's range",
		   extent < (double)( INT32_MAX / 2 ) / (double)B3_F_ONE );

	// And with the bound switched off the old behaviour is still there -- which
	// is what makes the field meaningful rather than decorative. Driving a body
	// past 262,144 units *should* trip b3IsValidFloat; a build where it does not
	// would mean the validity checks had stopped running and this whole test
	// was measuring nothing.
	b3WorldId freeId = makeStepWorld( V( 0, -9.8, 0 ) );
	b3World* freeWorld = b3GetWorldFromId( freeId );
	freeWorld->maxWorldExtent = b3f_zero;

	b3BodyDef fallDef = b3DefaultBodyDef();
	fallDef.type = b3_dynamicBody;
	fallDef.position = V( 0, 0, 0 );
	b3BodyId faller = b3CreateBody( freeId, &fallDef );

	b3ShapeDef fallShape = b3DefaultShapeDef();
	fallShape.density = b3fFromDouble( 1.0 );
	s_boxHull = b3MakeBoxHull( b3fFromDouble( 0.5 ), b3fFromDouble( 0.5 ), b3fFromDouble( 0.5 ) );
	b3CreateHullShape( faller, &fallShape, &s_boxHull.base );

	// The assertions this phase produces are the behaviour under test: driving
	// a body past 262,144 units is *supposed* to trip b3IsValidFloat, and the
	// tree's validator is where it surfaces.
	b3TestExpectAsserts( true );

	double reached = 0.0;
	bool leftValidRange = false;

	for ( int i = 0; i < 60000; ++i )
	{
		b3World_Step( freeId, 4 );

		b3Vec3 p = b3Body_GetPosition( faller );
		reached = fabs( F( p.y ) );

		if ( b3IsValidPosition( p ) == false )
		{
			leftValidRange = true;
			printf( "  with the bound off: left b3IsValidFloat's range at %.0f units, step %d\n",
					reached, i );
			break;
		}
	}

	b3TestExpectAsserts( false );

	check( "without the bound a body leaves the valid range", leftValidRange );

	b3DestroyWorld( freeId );
}

// =========================================================================
// Hang watchdog
// =========================================================================
//
// A test that loops forever reports nothing: the suite is killed from outside
// and the output buffer dies with it. This turns a hang into a backtrace.
//
// It exists because attaching gdb is not available here -- ptrace_scope blocks
// it, and relaxing that is a system-wide setting a test has no business
// changing -- so the process has to report on itself.

#include <execinfo.h>
#include <signal.h>
#include <unistd.h>

static void b3TestHangHandler( int sig )
{
	B3_UNUSED( sig );

	static const char msg[] = "\n*** HANG: still running when the watchdog fired. Backtrace:\n";
	ssize_t ignored = write( STDERR_FILENO, msg, sizeof( msg ) - 1 );
	B3_UNUSED( ignored );

	void* frames[32];
	int count = backtrace( frames, 32 );
	backtrace_symbols_fd( frames, count, STDERR_FILENO );

	_exit( 99 );
}

/// Abort with a backtrace if the next `seconds` of work do not finish.
static void b3TestWatchdog( unsigned seconds )
{
	signal( SIGALRM, b3TestHangHandler );
	alarm( seconds );
}

/// box3d_basic's walled scene, which hangs the ROM in about forty seconds.
///
/// A second failure, distinct from test_body_pushed_out_of_the_world above: no
/// body goes anywhere near the world bound, and adding four static wall bodies
/// to a scene that was stable without them is the whole difference. On device
/// the readout freezes on one frame -- tick count and all -- with three bodies
/// awake, and `defaultExceptionHandler` produces no guru screen, which points at
/// a loop that never exits rather than a bad access.
///
/// The geometry is copied from the example rather than tidied, because the
/// details are what reproduce it:
///
///   - the walls **overlap at the corners** (each spans the full floor width
///     plus both wall thicknesses), so two static boxes share a volume;
///   - the boxes are **density 1.0**, not b3DefaultShapeDef's 1000. At a tonne
///     each, 40 N never moves them and the run proves nothing -- which is
///     exactly the wasted run that preceded this test.
static void test_walled_box_pit( void )
{
	printf( "eight boxes in a walled pit\n" );

	const double floorHalf = 8.0;
	const double wallHalf = 0.5;
	const double wallHeight = 1.5;
	const double boxHalf = 0.5;
	const double wallOffset = floorHalf + wallHalf;

	static b3BoxHull s_floorHull;
	static b3BoxHull s_wallHull[4];
	static b3BoxHull s_boxHull;

	b3WorldId worldId = makeStepWorld( V( 0, -9.8, 0 ) );
	b3World* world = b3GetWorldFromId( worldId );

	b3BodyDef floorDef = b3DefaultBodyDef();
	floorDef.type = b3_staticBody;
	floorDef.position = V( 0, -0.25, 0 );
	b3BodyId floorId = b3CreateBody( worldId, &floorDef );

	b3ShapeDef floorShape = b3DefaultShapeDef();
	s_floorHull = b3MakeBoxHull( b3fFromDouble( floorHalf ), b3fFromDouble( 0.25 ), b3fFromDouble( floorHalf ) );
	b3CreateHullShape( floorId, &floorShape, &s_floorHull.base );

	const double wallPos[4][3] = {
		{ wallOffset, wallHeight, 0 },
		{ -wallOffset, wallHeight, 0 },
		{ 0, wallHeight, wallOffset },
		{ 0, wallHeight, -wallOffset },
	};
	const double wallSize[4][3] = {
		{ wallHalf, wallHeight, floorHalf + 2 * wallHalf },
		{ wallHalf, wallHeight, floorHalf + 2 * wallHalf },
		{ floorHalf + 2 * wallHalf, wallHeight, wallHalf },
		{ floorHalf + 2 * wallHalf, wallHeight, wallHalf },
	};

	for ( int i = 0; i < 4; ++i )
	{
		b3BodyDef wallDef = b3DefaultBodyDef();
		wallDef.type = b3_staticBody;
		wallDef.position = V( wallPos[i][0], wallPos[i][1], wallPos[i][2] );
		b3BodyId wallId = b3CreateBody( worldId, &wallDef );

		b3ShapeDef wallShape = b3DefaultShapeDef();
		s_wallHull[i] = b3MakeBoxHull( b3fFromDouble( wallSize[i][0] ), b3fFromDouble( wallSize[i][1] ),
									   b3fFromDouble( wallSize[i][2] ) );
		b3CreateHullShape( wallId, &wallShape, &s_wallHull[i].base );
	}

	b3BodyId boxes[8];
	for ( int i = 0; i < 8; ++i )
	{
		// BoxStartPosition, verbatim.
		double x = ( i & 1 ) ? 0.6 : -0.6;
		double y = 1.5 + (double)i * 1.4;
		double z = ( i & 2 ) ? 0.35 : -0.35;

		b3BodyDef boxDef = b3DefaultBodyDef();
		boxDef.type = b3_dynamicBody;
		boxDef.position = V( x, y, z );
		boxes[i] = b3CreateBody( worldId, &boxDef );

		b3ShapeDef boxShape = b3DefaultShapeDef();
		boxShape.density = b3fFromDouble( 1.0 );
		s_boxHull = b3MakeBoxHull( b3fFromDouble( boxHalf ), b3fFromDouble( boxHalf ), b3fFromDouble( boxHalf ) );
		b3CreateHullShape( boxes[i], &boxShape, &s_boxHull.base );
	}

	int assertsBefore = b3TestUnexpectedAsserts();

	// Generous even for MODE=debug, where every arithmetic operation carries a
	// shadow double. If it fires, the backtrace names where it is stuck.
	b3TestWatchdog( 300 );

	// Forty seconds of device time is 2,400 steps; the hang was reachable
	// inside the first few hundred and the stale-index assertion fired at 92.
	// 20,000 is an order of magnitude past both, and the cost matters: this
	// runs in all three modes and validate() walks every solver set, contact
	// and broad-phase tree.
	const int steps = 20000;
	static const double sweep[4][3] = { { 40, 0, 0 }, { 0, 0, -40 }, { -40, 0, 0 }, { 0, 0, 40 } };

	for ( int i = 0; i < steps; ++i )
	{
		const double* f = sweep[( i / 240 ) & 3];
		b3Body_ApplyForceToCenter( boxes[0], V( f[0], f[1], f[2] ), true );

		b3World_Step( worldId, 4 );

		// Every step through the window where both failures live, then
		// sparsely: the point of the tail is that the loop still terminates,
		// not that every one of 20,000 steps is structurally re-checked.
		if ( i < 3000 || ( i % 50 ) == 0 )
		{
			validate( world );
		}

		if ( b3TestUnexpectedAsserts() != assertsBefore )
		{
			printf( "  FIRST ASSERTION at step %d: awake %d bodies, %d solver sets, "
					"islands awake %d\n",
					i, b3World_GetAwakeBodyCount( worldId ), world->solverSets.count,
					world->solverSets.data[b3_awakeSet].islandSims.count );
			break;
		}
	}

	// Reaching here at all is the result: on device this scene stops responding.
	// If it stops here instead, it stops inside a loop and the process has to be
	// killed -- which is what running under timeout(1) turns into a report.
	alarm( 0 );
	printf( "  survived %d steps\n", steps );

	checkInt( "no assertion fired in the walled pit", b3TestUnexpectedAsserts() - assertsBefore, 0 );

	// Every box still inside the pit it cannot leave.
	for ( int i = 0; i < 8; ++i )
	{
		b3Vec3 p = b3Body_GetPosition( boxes[i] );
		bool inside = F( p.y ) > -8.0 && fabs( F( p.x ) ) < 32.0 && fabs( F( p.z ) ) < 32.0;

		if ( inside == false )
		{
			printf( "  box %d escaped to (%.1f %.1f %.1f)\n", i, F( p.x ), F( p.y ), F( p.z ) );
		}

		check( "box stayed in the pit", inside );
	}

	b3DestroyWorld( worldId );
}

// =========================================================================
// Joints
// =========================================================================

/// Every path through the joint plumbing that Phases 3A-5 compiled and never
/// ran.
///
/// b3_filterJoint is the whole point: it solves nothing, so anything that
/// fails here is bookkeeping -- a sim in the wrong solver set, a stale local
/// index, an island edge that outlived its joint -- and not fixed-point
/// arithmetic. Every later joint type inherits this code untouched, so this is
/// the case that has to be right first.
static void test_joint_plumbing( void )
{
	printf( "joint plumbing\n" );

	b3WorldId worldId = makeWorld();
	b3World* world = b3GetWorldFromId( worldId );

	// Start apart, because b3ShouldBodiesCollide is consulted in exactly one
	// place -- b3UpdateBroadPhasePairs, at pair *creation*
	// (broad_phase.c:328). Creating a filter joint over an existing contact
	// therefore does not retroactively destroy it, upstream included; only
	// b3Joint_SetCollideConnected does that, which is why it is the one
	// accessor that calls b3DestroyContactsBetweenBodies. Asserting otherwise
	// was this test's first draft, and the failure was the test's, not the
	// port's.
	b3BodyId a = makeDynamicSphere( worldId, V( 0, 0, 0 ), 1.0 );
	b3BodyId b = makeDynamicSphere( worldId, V( 8, 0, 0 ), 1.0 );

	step( world );
	checkInt( "no contact while apart", contactCount( world ), 0 );

	b3FilterJointDef def = b3DefaultFilterJointDef();
	def.base.bodyIdA = a;
	def.base.bodyIdB = b;
	b3JointId jointId = b3CreateFilterJoint( worldId, &def );
	validate( world );

	check( "the new joint is valid", b3Joint_IsValid( jointId ) );
	checkInt( "and reports its type", b3Joint_GetType( jointId ), b3_filterJoint );

	// Creating the joint destroys the contact between the two bodies. Both are
	// awake, so the sim goes into the constraint graph rather than a set.
	{
		b3Joint* joint = b3GetJointFullId( world, jointId );
		checkInt( "an awake joint lives in the awake set", joint->setIndex, b3_awakeSet );
		checkInt( "and in the one graph colour", joint->colorIndex, B3_OVERFLOW_INDEX );
		check( "and is linked into an island", joint->islandId != B3_NULL_INDEX );

		b3Body* bodyA = b3GetBodyFullId( world, a );
		b3Body* bodyB = b3GetBodyFullId( world, b );
		checkInt( "body A counts the joint", bodyA->jointCount, 1 );
		checkInt( "body B counts the joint", bodyB->jointCount, 1 );
		checkInt( "b3ShouldBodiesCollide says no", b3ShouldBodiesCollide( world, bodyA, bodyB ), 0 );
	}

	// Now bring them into contact range. The pair is offered to
	// b3UpdateBroadPhasePairs and the filter has to refuse it.
	b3Body_SetTransform( b, V( 1.0, 0, 0 ), b3Quat_identity );
	step( world );
	validate( world );
	checkInt( "a filter joint suppresses the pair", contactCount( world ), 0 );

	// The three dispatchers run over this joint every sub-step from here on. A
	// filter joint takes no case in any of them, so what this proves is that
	// the loops walk the right array with the right bounds.
	//
	// This world has no gravity and the bodies are at rest, so they fall
	// asleep partway through -- which is the point: the joint has to follow
	// its island out of the graph and into the sleeping set, and it is the
	// solver that has to keep stepping correctly once it has. Stepping a fixed
	// count and asserting the joint was still in the graph afterwards was this
	// test's second wrong draft.
	int stepsToSleep = 0;
	while ( stepsToSleep < 600 && b3GetBodyFullId( world, a )->setIndex == b3_awakeSet )
	{
		step( world );
		stepsToSleep++;
	}
	validate( world );
	check( "a jointed island falls asleep", stepsToSleep < 600 );
	{
		b3Joint* joint = b3GetJointFullId( world, jointId );
		check( "and takes its joint to a sleeping set", joint->setIndex >= b3_firstSleepingSet );
		checkInt( "leaving the graph", joint->colorIndex, B3_NULL_INDEX );
	}

	// Keep stepping while it sleeps: the dispatch loops now run over an empty
	// graph colour, which is the bound that was wrong for five phases.
	for ( int i = 0; i < 30; ++i )
	{
		step( world );
	}
	validate( world );

	b3Body_SetAwake( a, true );
	validate( world );
	checkInt( "waking returns the joint to the graph", b3GetJointFullId( world, jointId )->colorIndex,
			  B3_OVERFLOW_INDEX );

	// Accessors round-trip through whichever set holds the sim.
	{
		b3Transform frame = { V( 0.25, 0.5, -0.75 ), b3Quat_identity };
		b3Joint_SetLocalFrameA( jointId, frame );
		expect( "local frame A round-trips", F( b3Joint_GetLocalFrameA( jointId ).p.y ), 0.5, 1e-3 );

		b3Joint_SetUserData( jointId, (void*)(intptr_t)0x1234 );
		checkInt( "user data round-trips", (intptr_t)b3Joint_GetUserData( jointId ), 0x1234 );

		b3Joint_SetConstraintTuning( jointId, b3fFromDouble( 45.0 ), b3fFromDouble( 3.0 ) );
		b3f hertz, damping;
		b3Joint_GetConstraintTuning( jointId, &hertz, &damping );
		expect( "constraint hertz round-trips", F( hertz ), 45.0, 1e-2 );
		expect( "damping ratio round-trips", F( damping ), 3.0, 1e-2 );

		checkInt( "body A comes back", b3Joint_GetBodyA( jointId ).index1, a.index1 );
		checkInt( "body B comes back", b3Joint_GetBodyB( jointId ).index1, b.index1 );
	}

	// Disabling either body parks the joint in the disabled set and unlinks it
	// from its island; enabling puts it back.
	b3Body_Disable( b );
	validate( world );
	{
		b3Joint* joint = b3GetJointFullId( world, jointId );
		checkInt( "a joint on a disabled body is parked", joint->setIndex, b3_disabledSet );
		checkInt( "and has no island", joint->islandId, B3_NULL_INDEX );
	}

	b3Body_Enable( b );
	validate( world );
	{
		b3Joint* joint = b3GetJointFullId( world, jointId );
		checkInt( "re-enabling returns the joint to the awake set", joint->setIndex, b3_awakeSet );
		check( "and relinks its island", joint->islandId != B3_NULL_INDEX );
	}

	// Both bodies static: the joint belongs to the static set, which is the
	// one branch of b3CreateJoint that no other path here reaches.
	b3Body_SetType( a, b3_staticBody );
	b3Body_SetType( b, b3_staticBody );
	validate( world );
	checkInt( "a joint between two static bodies is static", b3GetJointFullId( world, jointId )->setIndex,
			  b3_staticSet );

	b3Body_SetType( a, b3_dynamicBody );
	b3Body_SetType( b, b3_dynamicBody );
	validate( world );
	checkInt( "and comes back when a body turns dynamic", b3GetJointFullId( world, jointId )->setIndex,
			  b3_awakeSet );

	// Turning the filter off has to bring the contact back, which means
	// telling the broad phase to re-find the pair.
	b3Joint_SetCollideConnected( jointId, true );
	step( world );
	validate( world );
	checkInt( "collideConnected restores the contact", contactCount( world ), 1 );

	b3Joint_SetCollideConnected( jointId, false );
	step( world );
	validate( world );
	checkInt( "and clearing it removes the contact again", contactCount( world ), 0 );

	b3DestroyJoint( jointId, true );
	validate( world );
	check( "a destroyed joint is not valid", b3Joint_IsValid( jointId ) == false );
	checkInt( "body A no longer counts it", b3GetBodyFullId( world, a )->jointCount, 0 );
	checkInt( "the joint list head is cleared", b3GetBodyFullId( world, a )->headJointKey, B3_NULL_INDEX );

	// Destroying a joint does not re-offer the pair by itself -- only
	// b3Joint_SetCollideConnected buffers a move for that. The shapes have to
	// move before the broad phase looks again, which is what a real scene does
	// anyway.
	//
	// Out and back, not a nudge: a small displacement stays inside the proxy's
	// fat AABB and buffers no move at all, so the pair is never re-offered.
	b3Body_SetTransform( b, V( 8, 0, 0 ), b3Quat_identity );
	step( world );
	b3Body_SetTransform( b, V( 1.0, 0, 0 ), b3Quat_identity );
	step( world );
	validate( world );
	checkInt( "the contact returns once the joint is gone", contactCount( world ), 1 );

	// A body carrying joints is destroyed. b3DestroyBody walks the list and
	// calls b3DestroyJointInternal per edge -- the code Phase 3A wrote blind.
	{
		b3BodyId c = makeDynamicSphere( worldId, V( 4, 0, 0 ), 1.0 );
		b3FilterJointDef d1 = b3DefaultFilterJointDef();
		d1.base.bodyIdA = a;
		d1.base.bodyIdB = c;
		b3JointId j1 = b3CreateFilterJoint( worldId, &d1 );

		b3FilterJointDef d2 = b3DefaultFilterJointDef();
		d2.base.bodyIdA = b;
		d2.base.bodyIdB = c;
		b3JointId j2 = b3CreateFilterJoint( worldId, &d2 );
		validate( world );
		checkInt( "a body can carry two joints", b3GetBodyFullId( world, c )->jointCount, 2 );

		b3DestroyBody( c );
		validate( world );
		check( "destroying a body invalidates its first joint", b3Joint_IsValid( j1 ) == false );
		check( "and its second", b3Joint_IsValid( j2 ) == false );
		checkInt( "and leaves the other bodies clean", b3GetBodyFullId( world, a )->jointCount, 0 );
	}

	b3DestroyWorld( worldId );
}

/// A joint created across two *separately sleeping* islands, which is the one
/// branch of b3CreateJoint that merges solver sets -- and the branch whose
/// re-fetch after b3MergeSolverSets is easy to drop, because the pointer it
/// repairs is only dangling on this path.
static void test_joint_merges_sleeping_sets( void )
{
	printf( "joint across two sleeping sets\n" );

	b3WorldId worldId = makeWorld();
	b3World* world = b3GetWorldFromId( worldId );

	// Far apart, so they are two islands rather than one.
	b3BodyId a = makeDynamicSphere( worldId, V( 0, 0, 0 ), 1.0 );
	b3BodyId b = makeDynamicSphere( worldId, V( 20, 0, 0 ), 1.0 );
	step( world );

	int islandA = b3GetBodyFullId( world, a )->islandId;
	int islandB = b3GetBodyFullId( world, b )->islandId;
	check( "the two bodies are separate islands", islandA != islandB );

	b3TrySleepIsland( world, islandA );
	b3TrySleepIsland( world, islandB );
	validate( world );

	int setA = b3GetBodyFullId( world, a )->setIndex;
	int setB = b3GetBodyFullId( world, b )->setIndex;
	check( "both bodies are asleep in different sets", setA >= b3_firstSleepingSet && setB >= b3_firstSleepingSet &&
														  setA != setB );

	b3FilterJointDef def = b3DefaultFilterJointDef();
	def.base.bodyIdA = a;
	def.base.bodyIdB = b;
	b3JointId jointId = b3CreateFilterJoint( worldId, &def );
	validate( world );

	checkInt( "the sleeping sets merged", b3GetBodyFullId( world, a )->setIndex,
			  b3GetBodyFullId( world, b )->setIndex );

	// The re-fetched sim must be the joint's own, not whatever landed at its
	// old address after the merge reallocated.
	{
		b3Joint* joint = b3GetJointFullId( world, jointId );
		b3JointSim* sim = b3GetJointSim( world, joint );
		checkInt( "and the joint sim survived the merge", sim->jointId, joint->jointId );
		checkInt( "with body A intact", sim->bodyIdA, b3GetBodyFullId( world, a )->id );
		checkInt( "with body B intact", sim->bodyIdB, b3GetBodyFullId( world, b )->id );
	}

	b3Joint_WakeBodies( jointId );
	validate( world );
	checkInt( "waking through the joint returns it to the graph", b3GetJointFullId( world, jointId )->colorIndex,
			  B3_OVERFLOW_INDEX );

	b3DestroyWorld( worldId );
}

// =========================================================================
// Distance joint
// =========================================================================
//
// Phase 6 Stage 2. Where test_joint_plumbing proved the bookkeeping, these
// prove the *arithmetic* -- and they are deliberately closed-form, because a
// fixed-point constraint that is wrong by a factor of two still looks like a
// plausible simulation. Every scale in distance_joint.c is inherited untouched
// by the revolute, spherical, prismatic and wheel joints, so an error here
// would propagate into 3,800 more lines before anything caught it.

/// A static anchor and one dynamic sphere hanging beneath it.
///
/// The scene every case below starts from. Sleeping is disabled on the sphere:
/// most of these settle, and a settled body that falls asleep stops reporting
/// the reaction force the case is measuring.
static b3JointId hangSphere( b3WorldId worldId, b3World* world, double length, b3BodyId* anchorOut,
							 b3BodyId* sphereOut, b3DistanceJointDef* defOut )
{
	b3BodyDef anchorDef = b3DefaultBodyDef();
	anchorDef.type = b3_staticBody;
	anchorDef.position = V( 0, 0, 0 );
	b3BodyId anchor = b3CreateBody( worldId, &anchorDef );

	b3BodyDef sphereDef = b3DefaultBodyDef();
	sphereDef.type = b3_dynamicBody;
	sphereDef.position = V( 0, -length, 0 );
	sphereDef.enableSleep = false;
	b3BodyId sphere = b3CreateBody( worldId, &sphereDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3Sphere ball = { V( 0, 0, 0 ), b3fFromDouble( 0.25 ) };
	b3CreateSphereShape( sphere, &shapeDef, &ball );

	b3DistanceJointDef def = b3DefaultDistanceJointDef();
	def.base.bodyIdA = anchor;
	def.base.bodyIdB = sphere;
	def.length = b3fFromDouble( length );

	if ( defOut != NULL )
	{
		*defOut = def;
	}

	b3JointId jointId = b3CreateDistanceJoint( worldId, &def );
	validate( world );

	*anchorOut = anchor;
	*sphereOut = sphere;
	return jointId;
}

/// The rigid constraint, against the two things it must not do: stretch, and
/// creep.
///
/// A rope that is 1% long is a rope that looks right in a screenshot and is
/// wrong in motion; one that grows by a quantum per step is a rope that is
/// right for ten seconds and wrong for a minute. The second is the failure a
/// fixed-point impulse accumulator invites, because every clamp and every
/// narrowing is a chance to bias one direction, so the length is sampled at
/// two widely separated times and compared to each other as well as to the
/// rest length.
static void test_distance_joint_rigid( void )
{
	printf( "distance joint, rigid\n" );

	b3WorldId worldId = makeStepWorld( V( 0, -10, 0 ) );
	b3World* world = b3GetWorldFromId( worldId );

	b3BodyId anchor, sphere;
	b3JointId jointId = hangSphere( worldId, world, 2.0, &anchor, &sphere, NULL );

	checkInt( "the joint reports its type", b3Joint_GetType( jointId ), b3_distanceJoint );
	expect( "and its rest length", F( b3DistanceJoint_GetLength( jointId ) ), 2.0, 1e-3 );

	// Swing it: released from the side, this is a pendulum, which loads the
	// constraint with a centripetal term rather than just gravity.
	b3Body_SetTransform( sphere, V( 2.0, 0, 0 ), b3Quat_identity );

	for ( int i = 0; i < 60; ++i )
	{
		b3World_Step( worldId, 4 );
	}
	validate( world );
	double early = F( b3DistanceJoint_GetCurrentLength( jointId ) );

	for ( int i = 0; i < 540; ++i )
	{
		b3World_Step( worldId, 4 );
	}
	validate( world );
	double late = F( b3DistanceJoint_GetCurrentLength( jointId ) );

	// B3_LINEAR_SLOP is 20 raw at Q12, so just under 0.005. A rigid constraint
	// solved with bias holds to a small multiple of it.
	printf( "  length at 1 s %.6f, at 10 s %.6f (rest 2.0, creep %+.6f)\n", early, late, late - early );
	expect( "a swinging pendulum holds its length after 1 s", early, 2.0, 0.02 );
	expect( "and still holds it after 10 s", late, 2.0, 0.02 );
	expect( "with no creep between the two", late - early, 0.0, 0.01 );

	b3DestroyWorld( worldId );
}

/// The reaction force, against the one number that needs no solver tuning to
/// predict: a mass hanging at rest pulls on its rope with exactly m*g.
///
/// This is the case that validates the whole impulse chain end to end -- the
/// Q24 effective mass, the Q16 accumulator, b3MulImpFToF, and the world's
/// stored inv_h. Any one of them off by a power of two shows up here as a
/// clean factor, which is why it is worth a test of its own rather than being
/// folded into the rigid case.
static void test_distance_joint_reaction( void )
{
	printf( "distance joint reaction force\n" );

	b3WorldId worldId = makeStepWorld( V( 0, -10, 0 ) );
	b3World* world = b3GetWorldFromId( worldId );

	b3BodyId anchor, sphere;
	b3JointId jointId = hangSphere( worldId, world, 2.0, &anchor, &sphere, NULL );

	// Hanging straight down and released from rest, so it settles rather than
	// swinging -- a swinging mass adds a centripetal term this closed form
	// does not model.
	for ( int i = 0; i < 240; ++i )
	{
		b3World_Step( worldId, 4 );
	}
	validate( world );

	double mass = F( b3Body_GetMass( sphere ) );
	check( "the sphere has a mass", mass > 0.0 );

	b3Vec3 force = b3Joint_GetConstraintForce( jointId );
	double magnitude = sqrt( F( force.x ) * F( force.x ) + F( force.y ) * F( force.y ) + F( force.z ) * F( force.z ) );

	printf( "  reaction %.4f N (m*g = %.4f), lateral (%.4f, %.4f)\n", magnitude, mass * 10.0, F( force.x ),
			F( force.z ) );

	// 2% of m*g. The residual is the settling velocity the constraint is still
	// removing, not a scale error -- a scale error here is a factor, not a
	// percentage.
	expect( "a hanging mass pulls with m*g", magnitude, mass * 10.0, mass * 10.0 * 0.02 );

	// It pulls *along* the rope, which for a vertical hang means the force is
	// vertical and the two lateral components are noise.
	expect( "and pulls vertically", F( force.x ), 0.0, 0.05 );
	expect( "in x and z alike", F( force.z ), 0.0, 0.05 );

	// A filter joint has no constraint and must answer zero rather than
	// reading a union it never wrote.
	{
		b3BodyId other = makeDynamicSphere( worldId, V( 8, 0, 0 ), 0.5 );
		b3FilterJointDef fdef = b3DefaultFilterJointDef();
		fdef.base.bodyIdA = sphere;
		fdef.base.bodyIdB = other;
		b3JointId filterId = b3CreateFilterJoint( worldId, &fdef );
		b3World_Step( worldId, 4 );
		validate( world );

		b3Vec3 none = b3Joint_GetConstraintForce( filterId );
		expect( "a filter joint reports no force", F( none.x ) + F( none.y ) + F( none.z ), 0.0, 1e-6 );
	}

	b3DestroyWorld( worldId );
}

/// The reaction force through a chain, where each joint carries the weight of
/// everything below it: 3mg, 2mg, mg.
///
/// The single-mass case above cannot distinguish "the impulse scale is right"
/// from "the impulse scale is right at one magnitude", and a chain is where
/// the port's first joint scenario in run_pair reported the port 11% below the
/// float reference. Sleeping is disabled on every link, which is the whole
/// point of having this alongside that scenario: an impulse frozen by an early
/// sleep is not the same fault as an impulse computed wrongly, and only one of
/// them is the port's.
static void test_distance_joint_chain_reaction( void )
{
	printf( "distance joint chain reaction forces\n" );

	b3WorldId worldId = makeStepWorld( V( 0, -10, 0 ) );
	b3World* world = b3GetWorldFromId( worldId );

	b3BodyDef anchorDef = b3DefaultBodyDef();
	anchorDef.type = b3_staticBody;
	anchorDef.position = V( 0, 0, 0 );
	b3BodyId previous = b3CreateBody( worldId, &anchorDef );

	b3BodyId links[3];
	b3JointId joints[3];
	for ( int i = 0; i < 3; ++i )
	{
		b3BodyDef linkDef = b3DefaultBodyDef();
		linkDef.type = b3_dynamicBody;
		linkDef.position = V( 0, -1.0 * ( i + 1 ), 0 );
		linkDef.enableSleep = false;
		links[i] = b3CreateBody( worldId, &linkDef );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		b3Sphere ball = { V( 0, 0, 0 ), b3fFromDouble( 0.2 ) };
		b3CreateSphereShape( links[i], &shapeDef, &ball );

		b3DistanceJointDef jointDef = b3DefaultDistanceJointDef();
		jointDef.base.bodyIdA = previous;
		jointDef.base.bodyIdB = links[i];
		jointDef.length = b3fFromInt( 1 );
		joints[i] = b3CreateDistanceJoint( worldId, &jointDef );

		previous = links[i];
	}
	validate( world );

	// Settle first.
	for ( int i = 0; i < 600; ++i )
	{
		b3World_Step( worldId, 4 );
	}
	validate( world );

	// Then *average* the reported force over a further second.
	//
	// A chain at rest does not reach an exact fixed point here the way a float
	// solver does: the equilibrium impulse falls between two Q15.16 values, so
	// the accumulator dithers between them and the relaxation pass carries the
	// difference forward. A single sample of the top joint is up to 9% off; the
	// mean is not, because a limit cycle averages out and a scale error would
	// not. Sampling one instant and asserting 3% was this test's first draft,
	// and the run_pair joint scenario found the same thing independently.
	double sum[3] = { 0 };
	const int samples = 60;
	for ( int s = 0; s < samples; ++s )
	{
		b3World_Step( worldId, 4 );
		for ( int i = 0; i < 3; ++i )
		{
			b3Vec3 force = b3Joint_GetConstraintForce( joints[i] );
			sum[i] += sqrt( F( force.x ) * F( force.x ) + F( force.y ) * F( force.y ) + F( force.z ) * F( force.z ) );
		}
	}
	validate( world );

	double mass = F( b3Body_GetMass( links[0] ) );

	for ( int i = 0; i < 3; ++i )
	{
		double mean = sum[i] / samples;
		double want = mass * 10.0 * ( 3 - i );

		printf( "  joint %d carries %d link(s): mean %.4f N, %d*m*g = %.4f (ratio %.4f)\n", i, 3 - i, mean, 3 - i, want,
				mean / want );

		char what[64];
		snprintf( what, sizeof( what ), "joint %d carries %d m*g", i, 3 - i );
		expect( what, mean, want, want * 0.02 );
	}

	b3DestroyWorld( worldId );
}

/// The spring, against the closed form that makes it worth having: a mass on a
/// spring of natural frequency f hangs at a static deflection of g / (2*pi*f)^2
/// -- **independent of the mass**, which is what makes it a check on the
/// softness coefficients rather than on b3MakeSoft's inputs.
///
/// b3MakeSoft's biasRate, massScale and impulseScale are three quantities at
/// three different scales (Q12, Q30, Q30), and the deflection is the only
/// place their relationship is observable from outside.
static void test_distance_joint_spring( void )
{
	printf( "distance joint spring\n" );

	const double hertz[2] = { 1.0, 2.0 };
	double deflection[2];

	for ( int k = 0; k < 2; ++k )
	{
		b3WorldId worldId = makeStepWorld( V( 0, -10, 0 ) );
		b3World* world = b3GetWorldFromId( worldId );

		b3BodyId anchor, sphere;
		b3JointId jointId = hangSphere( worldId, world, 2.0, &anchor, &sphere, NULL );

		b3DistanceJoint_EnableSpring( jointId, true );
		b3DistanceJoint_SetSpringHertz( jointId, b3fFromDouble( hertz[k] ) );

		// Critically damped, so it settles rather than ringing and the sampled
		// value is the static deflection rather than a point on a waveform.
		b3DistanceJoint_SetSpringDampingRatio( jointId, b3fFromDouble( 1.0 ) );

		check( "the spring reports enabled", b3DistanceJoint_IsSpringEnabled( jointId ) );

		for ( int i = 0; i < 600; ++i )
		{
			b3World_Step( worldId, 4 );
		}
		validate( world );

		deflection[k] = F( b3DistanceJoint_GetCurrentLength( jointId ) ) - 2.0;

		// Settled, not still moving: the closed form is a static one.
		b3Vec3 v = b3Body_GetLinearVelocity( sphere );
		expect( "the spring settles", F( v.y ), 0.0, 0.05 );

		b3DestroyWorld( worldId );
	}

	// x = g / omega^2, omega = 2*pi*f. 0.2533 m at 1 Hz, 0.0633 m at 2 Hz.
	const double twoPi = 6.283185307179586;
	for ( int k = 0; k < 2; ++k )
	{
		double omega = twoPi * hertz[k];
		double want = 10.0 / ( omega * omega );
		char what[64];
		snprintf( what, sizeof( what ), "static deflection at %.0f Hz", hertz[k] );
		printf( "  %.0f Hz: deflection %.6f, g/omega^2 predicts %.6f (ratio %.3f)\n", hertz[k], deflection[k], want,
				deflection[k] / want );
		// 3% -- the measured error is 0.1%, and the tolerance is set from that
		// rather than from a guess. A wrong scale in the softness coefficients
		// is a factor, not a few percent, so this has room to spare and still
		// catches everything it is here for.
		expect( what, deflection[k], want, want * 0.03 );
	}

	// Quartering the deflection when the frequency doubles is the 1/f^2 law,
	// and it holds tighter still than either absolute value does, because the
	// solver's own damping bias cancels between the two.
	expect( "and it scales as 1/f^2", deflection[0] / deflection[1], 4.0, 0.05 );

	b3f lower, upper;
	b3WorldId worldId = makeStepWorld( V( 0, -10, 0 ) );
	b3World* world = b3GetWorldFromId( worldId );
	b3BodyId anchor, sphere;
	b3JointId jointId = hangSphere( worldId, world, 2.0, &anchor, &sphere, NULL );

	// The default force range is the port's "no bound" sentinel, and it must
	// survive being multiplied by the sub-step to reach the impulse scale --
	// the conversion that would saturate if B3_F_MAX had been used instead.
	b3DistanceJoint_GetSpringForceRange( jointId, &lower, &upper );
	check( "the default spring force range is unbounded below", F( lower ) < -100000.0 );
	check( "and above", F( upper ) > 100000.0 );

	// A tension bound the spring cannot exceed: the mass then falls, because
	// 1 N cannot hold a body that weighs more.
	b3DistanceJoint_EnableSpring( jointId, true );
	b3DistanceJoint_SetSpringHertz( jointId, b3fFromDouble( 4.0 ) );
	b3DistanceJoint_SetSpringDampingRatio( jointId, b3fFromDouble( 1.0 ) );
	b3DistanceJoint_SetSpringForceRange( jointId, b3fFromDouble( -1.0 ), b3fFromDouble( 1.0 ) );

	for ( int i = 0; i < 120; ++i )
	{
		b3World_Step( worldId, 4 );
	}
	validate( world );
	check( "a spring force bound below the weight lets the mass fall",
		   F( b3DistanceJoint_GetCurrentLength( jointId ) ) > 2.5 );

	b3DestroyWorld( worldId );
}

/// The two-sided limit, including the speculative branch.
///
/// A limit is the port's first *one-sided* joint constraint, so it is the
/// first place b3MaxImp on an accumulator has to hold under quantization. The
/// speculative branch (C > 0) is reached whenever the body is inside the range
/// and closing, which is most steps -- it is the common path, not the corner.
static void test_distance_joint_limit( void )
{
	printf( "distance joint limit\n" );

	b3WorldId worldId = makeStepWorld( V( 0, -10, 0 ) );
	b3World* world = b3GetWorldFromId( worldId );

	b3BodyId anchor, sphere;
	b3JointId jointId = hangSphere( worldId, world, 2.0, &anchor, &sphere, NULL );

	// Spring on with zero hertz is upstream's "limit only" configuration: the
	// spring block is skipped, but the joint still takes the soft branch, and
	// the limit runs. Asserting this here because it looks like a mistake.
	b3DistanceJoint_EnableSpring( jointId, true );
	b3DistanceJoint_SetSpringHertz( jointId, b3f_zero );
	b3DistanceJoint_EnableLimit( jointId, true );
	b3DistanceJoint_SetLengthRange( jointId, b3fFromDouble( 1.0 ), b3fFromDouble( 3.0 ) );

	check( "the limit reports enabled", b3DistanceJoint_IsLimitEnabled( jointId ) );
	expect( "min length round-trips", F( b3DistanceJoint_GetMinLength( jointId ) ), 1.0, 1e-3 );
	expect( "max length round-trips", F( b3DistanceJoint_GetMaxLength( jointId ) ), 3.0, 1e-3 );

	// Falling under gravity with no spring to hold it, the mass runs out to
	// the upper limit and stops there.
	for ( int i = 0; i < 300; ++i )
	{
		b3World_Step( worldId, 4 );
	}
	validate( world );

	double atUpper = F( b3DistanceJoint_GetCurrentLength( jointId ) );
	printf( "  upper limit 3.0: settled at %.6f\n", atUpper );
	expect( "gravity carries the mass to the upper limit", atUpper, 3.0, 0.03 );
	check( "and no further", atUpper < 3.05 );

	// Thrown upward past the anchor, it runs *in* to the lower limit instead.
	// Gravity reversed rather than an impulse, so the load is sustained: an
	// impulse would test one step of the constraint, this tests three hundred.
	b3World_SetGravity( worldId, V( 0, 30, 0 ) );
	for ( int i = 0; i < 300; ++i )
	{
		b3World_Step( worldId, 4 );
	}
	validate( world );

	double atLower = F( b3DistanceJoint_GetCurrentLength( jointId ) );
	printf( "  lower limit 1.0: settled at %.6f\n", atLower );
	expect( "reversed gravity carries it to the lower limit", atLower, 1.0, 0.03 );
	check( "and no closer", atLower > 0.95 );

	// Sorted rather than rejected: a caller passing the range backwards gets
	// the range they meant.
	b3DistanceJoint_SetLengthRange( jointId, b3fFromDouble( 4.0 ), b3fFromDouble( 2.0 ) );
	expect( "a reversed range is sorted, min", F( b3DistanceJoint_GetMinLength( jointId ) ), 2.0, 1e-3 );
	expect( "and max", F( b3DistanceJoint_GetMaxLength( jointId ) ), 4.0, 1e-3 );

	b3DestroyWorld( worldId );
}

/// The motor, against its two closed forms: it reaches the speed asked for,
/// and it cannot exceed the force allowed.
static void test_distance_joint_motor( void )
{
	printf( "distance joint motor\n" );

	// No gravity: the motor speed is then the whole of the axial velocity, so
	// the measurement is direct rather than a difference against free fall.
	b3WorldId worldId = makeStepWorld( V( 0, 0, 0 ) );
	b3World* world = b3GetWorldFromId( worldId );

	b3BodyId anchor, sphere;
	b3JointId jointId = hangSphere( worldId, world, 2.0, &anchor, &sphere, NULL );

	b3DistanceJoint_EnableSpring( jointId, true );
	b3DistanceJoint_SetSpringHertz( jointId, b3f_zero );
	b3DistanceJoint_EnableMotor( jointId, true );
	b3DistanceJoint_SetMotorSpeed( jointId, b3fFromDouble( 1.0 ) );
	b3DistanceJoint_SetMaxMotorForce( jointId, b3fFromDouble( 500.0 ) );

	check( "the motor reports enabled", b3DistanceJoint_IsMotorEnabled( jointId ) );
	expect( "motor speed round-trips", F( b3DistanceJoint_GetMotorSpeed( jointId ) ), 1.0, 1e-3 );
	expect( "max motor force round-trips", F( b3DistanceJoint_GetMaxMotorForce( jointId ) ), 500.0, 1e-2 );

	double before = F( b3DistanceJoint_GetCurrentLength( jointId ) );
	for ( int i = 0; i < 60; ++i )
	{
		b3World_Step( worldId, 4 );
	}
	validate( world );
	double after = F( b3DistanceJoint_GetCurrentLength( jointId ) );

	// One second at 1 m/s. The first few steps spend accelerating from rest,
	// so the travel is a little under a metre.
	b3Vec3 v = b3Body_GetLinearVelocity( sphere );
	printf( "  motor at 1 m/s: travelled %.6f m in 1 s, axial speed %.6f\n", after - before, -F( v.y ) );
	expect( "a motor at 1 m/s extends the joint by ~1 m in 1 s", after - before, 1.0, 0.1 );
	expect( "and holds the speed asked for", -F( v.y ), 1.0, 0.05 );

	// The motor force is the accumulated motor impulse over the sub-step, and
	// at a constant speed against no load it is near zero -- there is nothing
	// left to accelerate.
	check( "the motor force is bounded by its maximum", fabs( F( b3DistanceJoint_GetMotorForce( jointId ) ) ) <= 500.0 );

	b3DestroyWorld( worldId );

	// Now the bound. A motor limited to 1 N cannot lift a body that weighs
	// more, and the length must grow instead of shrinking.
	worldId = makeStepWorld( V( 0, -10, 0 ) );
	world = b3GetWorldFromId( worldId );
	jointId = hangSphere( worldId, world, 2.0, &anchor, &sphere, NULL );

	b3DistanceJoint_EnableSpring( jointId, true );
	b3DistanceJoint_SetSpringHertz( jointId, b3f_zero );
	b3DistanceJoint_EnableMotor( jointId, true );
	b3DistanceJoint_SetMotorSpeed( jointId, b3fFromDouble( -1.0 ) );
	b3DistanceJoint_SetMaxMotorForce( jointId, b3fFromDouble( 1.0 ) );

	for ( int i = 0; i < 120; ++i )
	{
		b3World_Step( worldId, 4 );
	}
	validate( world );

	check( "a motor bounded below the weight cannot retract the joint",
		   F( b3DistanceJoint_GetCurrentLength( jointId ) ) > 2.0 );
	check( "and its reported force respects the bound",
		   fabs( F( b3DistanceJoint_GetMotorForce( jointId ) ) ) <= 1.05 );

	// Disabling the motor clears its accumulator, so a re-enable does not warm
	// start from a force the caller has since changed their mind about.
	b3DistanceJoint_EnableMotor( jointId, false );
	check( "disabling the motor reports it off", b3DistanceJoint_IsMotorEnabled( jointId ) == false );
	expect( "and clears its impulse", F( b3DistanceJoint_GetMotorForce( jointId ) ), 0.0, 1e-6 );

	b3DestroyWorld( worldId );
}

/// The paths Stage 1 proved for a joint that stores nothing, now walked by one
/// that stores four accumulators and a prepared anchor pair.
///
/// Sleeping moves the sim out of the graph colour and into a solver set, and
/// waking moves it back -- both are memcpy of a b3JointSim, which is 116 bytes
/// larger since the union landed. A rope that jolts on waking is this copy
/// going wrong.
static void test_distance_joint_sleep_and_accessors( void )
{
	printf( "distance joint through sleep, and accessor round-trips\n" );

	b3WorldId worldId = makeStepWorld( V( 0, -10, 0 ) );
	b3World* world = b3GetWorldFromId( worldId );

	b3BodyId anchor, sphere;
	b3JointId jointId = hangSphere( worldId, world, 2.0, &anchor, &sphere, NULL );

	// Sleep is disabled by hangSphere, since most cases measure a settled
	// body. This one wants it back.
	b3Body_EnableSleep( sphere, true );

	int steps = 0;
	while ( steps < 900 && b3GetBodyFullId( world, sphere )->setIndex == b3_awakeSet )
	{
		b3World_Step( worldId, 4 );
		steps++;
	}
	validate( world );
	check( "a jointed body settles and sleeps", steps < 900 );
	check( "taking its joint out of the graph", b3GetJointFullId( world, jointId )->colorIndex == B3_NULL_INDEX );

	double asleep = F( b3DistanceJoint_GetCurrentLength( jointId ) );

	b3Body_SetAwake( sphere, true );
	validate( world );
	checkInt( "waking returns the joint to the graph", b3GetJointFullId( world, jointId )->colorIndex,
			  B3_OVERFLOW_INDEX );

	// The first step after waking is where a mis-copied sim shows up: the
	// anchors and accumulators have to be the ones the joint went to sleep
	// with, or the constraint yanks.
	b3World_Step( worldId, 4 );
	validate( world );
	expect( "and the joint resumes without a jolt", F( b3DistanceJoint_GetCurrentLength( jointId ) ), asleep, 0.01 );

	// Accessors, each round-tripping through whichever set holds the sim.
	b3DistanceJoint_SetLength( jointId, b3fFromDouble( 3.5 ) );
	expect( "length round-trips", F( b3DistanceJoint_GetLength( jointId ) ), 3.5, 1e-3 );

	// Clamped from below rather than accepted: a rest length under the linear
	// slop is a constraint the solver cannot represent.
	b3DistanceJoint_SetLength( jointId, b3f_zero );
	check( "a zero rest length is clamped up to the slop", F( b3DistanceJoint_GetLength( jointId ) ) > 0.0 );
	check( "and not far above it", F( b3DistanceJoint_GetLength( jointId ) ) < 0.01 );

	b3DistanceJoint_SetSpringHertz( jointId, b3fFromDouble( 7.5 ) );
	expect( "spring hertz round-trips", F( b3DistanceJoint_GetSpringHertz( jointId ) ), 7.5, 1e-2 );

	b3DistanceJoint_SetSpringDampingRatio( jointId, b3fFromDouble( 3.25 ) );
	expect( "spring damping round-trips", F( b3DistanceJoint_GetSpringDampingRatio( jointId ) ), 3.25, 1e-2 );

	b3DistanceJoint_SetSpringForceRange( jointId, b3fFromDouble( -12.0 ), b3fFromDouble( 34.0 ) );
	{
		b3f lower, upper;
		b3DistanceJoint_GetSpringForceRange( jointId, &lower, &upper );
		expect( "spring force range round-trips, lower", F( lower ), -12.0, 1e-2 );
		expect( "and upper", F( upper ), 34.0, 1e-2 );
	}

	b3DistanceJoint_SetMotorSpeed( jointId, b3fFromDouble( -2.5 ) );
	expect( "motor speed round-trips", F( b3DistanceJoint_GetMotorSpeed( jointId ) ), -2.5, 1e-3 );

	// The base accessors still work on a typed joint: the union sits after
	// them, so a wrong offset would corrupt one or the other.
	b3Joint_SetUserData( jointId, (void*)(intptr_t)0xABCD );
	checkInt( "base user data still round-trips", (intptr_t)b3Joint_GetUserData( jointId ), 0xABCD );
	checkInt( "and body A", b3Joint_GetBodyA( jointId ).index1, anchor.index1 );

	// Destruction, with the union in play.
	b3DestroyJoint( jointId, true );
	validate( world );
	check( "a destroyed distance joint is not valid", b3Joint_IsValid( jointId ) == false );
	checkInt( "and the body no longer counts it", b3GetBodyFullId( world, sphere )->jointCount, 0 );

	b3DestroyWorld( worldId );
}

// =========================================================================
// Revolute joint
// =========================================================================
//
// Phase 6 Stage 3, and the first joint that constrains a rotation. Where the
// distance joint's four solve branches were near-identical, a hinge's three
// constraints are structurally different and couple through the shared
// velocity state -- so a sign error in one of them reads as a slightly loose
// hinge rather than as a failure. That is what the pendulum-period case below
// is for: a closed form the solver's tuning cannot move.

/// A hinge to a static anchor, with an arm hanging off it.
///
/// The hinge axis is the **z axis of the joint frames**, so this scene has the
/// arm swinging in the x-y plane. `armLength` puts the arm's mass at that
/// distance along +x from the pivot, which makes the pendulum's closed form a
/// point mass on a massless rod to within the sphere's own inertia.
static b3JointId hingeArm( b3WorldId worldId, b3World* world, double armLength, bool startHorizontal,
						   b3BodyId* anchorOut, b3BodyId* armOut )
{
	b3BodyDef anchorDef = b3DefaultBodyDef();
	anchorDef.type = b3_staticBody;
	anchorDef.position = V( 0, 0, 0 );
	b3BodyId anchor = b3CreateBody( worldId, &anchorDef );

	b3BodyDef armDef = b3DefaultBodyDef();
	armDef.type = b3_dynamicBody;
	armDef.position = startHorizontal ? V( armLength, 0, 0 ) : V( 0, -armLength, 0 );

	// Sleep off by default: most of these measure an accumulator, and a
	// sleeping body freezes one mid-settle. Stage 2 paid for learning that.
	armDef.enableSleep = false;
	b3BodyId arm = b3CreateBody( worldId, &armDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3Sphere ball = { V( 0, 0, 0 ), b3fFromDouble( 0.1 ) };
	b3CreateSphereShape( arm, &shapeDef, &ball );

	b3RevoluteJointDef def = b3DefaultRevoluteJointDef();
	def.base.bodyIdA = anchor;
	def.base.bodyIdB = arm;

	// Frame A sits at the pivot (the anchor's origin); frame B sits at the
	// pivot expressed in the arm's local space, which is -armLength along
	// whichever axis the arm was placed on.
	def.base.localFrameB.p = startHorizontal ? V( -armLength, 0, 0 ) : V( 0, armLength, 0 );

	b3JointId jointId = b3CreateRevoluteJoint( worldId, &def );
	validate( world );

	*anchorOut = anchor;
	*armOut = arm;
	return jointId;
}

/// The point-to-point and collinearity constraints, against the two things a
/// hinge must not do: let the pivot drift, and let the axis tilt.
///
/// Both are checked over 600 steps and compared early against late, because a
/// hinge that is 1% loose looks right in a screenshot and a hinge that drifts
/// by a quantum per step looks right for ten seconds.
static void test_revolute_joint_holds( void )
{
	printf( "revolute joint holds its pivot and axis\n" );

	b3WorldId worldId = makeStepWorld( V( 0, -10, 0 ) );
	b3World* world = b3GetWorldFromId( worldId );

	b3BodyId anchor, arm;
	b3JointId jointId = hingeArm( worldId, world, 2.0, true, &anchor, &arm );

	checkInt( "the joint reports its type", b3Joint_GetType( jointId ), b3_revoluteJoint );

	// Load it off-axis as well as under gravity: a torque about x and z is
	// what the collinearity constraint has to absorb, and a purely planar
	// swing would never exercise it.
	b3Body_SetAngularVelocity( arm, V( 3.0, 0, 1.5 ) );

	for ( int i = 0; i < 60; ++i )
	{
		b3World_Step( worldId, 4 );
	}
	validate( world );

	// The pivot: frame A is at the origin, so the arm's own pivot point must
	// stay there. Measured through the arm's transform rather than through the
	// joint, so this is independent of anything the joint reports.
	b3Vec3 pivotEarly = b3TransformPoint( b3Body_GetTransform( arm ), V( -2.0, 0, 0 ) );
	double axisEarly = F( b3RotateVector( b3Body_GetRotation( arm ), V( 0, 0, 1 ) ).z );

	for ( int i = 0; i < 540; ++i )
	{
		b3World_Step( worldId, 4 );
	}
	validate( world );

	b3Vec3 pivotLate = b3TransformPoint( b3Body_GetTransform( arm ), V( -2.0, 0, 0 ) );
	double axisLate = F( b3RotateVector( b3Body_GetRotation( arm ), V( 0, 0, 1 ) ).z );

	printf( "  pivot at 1 s (%.5f %.5f %.5f), at 10 s (%.5f %.5f %.5f)\n", F( pivotEarly.x ), F( pivotEarly.y ),
			F( pivotEarly.z ), F( pivotLate.x ), F( pivotLate.y ), F( pivotLate.z ) );
	printf( "  hinge axis z-component: %.6f then %.6f (1.0 is perfectly aligned)\n", axisEarly, axisLate );

	expect( "the pivot holds at 1 s, x", F( pivotEarly.x ), 0.0, 0.02 );
	expect( "and y", F( pivotEarly.y ), 0.0, 0.02 );
	expect( "and z", F( pivotEarly.z ), 0.0, 0.02 );

	expect( "the pivot still holds at 10 s, x", F( pivotLate.x ), 0.0, 0.02 );
	expect( "and y", F( pivotLate.y ), 0.0, 0.02 );
	expect( "and z", F( pivotLate.z ), 0.0, 0.02 );

	// The collinearity constraint: the arm's z axis must stay the anchor's z
	// axis, which means its z component stays 1. An off-axis torque that leaks
	// through shows here and nowhere else.
	expect( "the hinge axis stays aligned at 1 s", axisEarly, 1.0, 0.02 );
	expect( "and at 10 s", axisLate, 1.0, 0.02 );

	b3DestroyWorld( worldId );
}

/// The pendulum period, which is the rotational counterpart of the distance
/// joint's spring-deflection check: a closed form that depends on the
/// effective mass being right and on nothing the test controls.
///
/// A point mass m at distance d on a frictionless hinge swings at
/// T = 2*pi*sqrt(d/g) for small amplitudes -- the mass cancels, which is
/// exactly what makes it a check on the constraint rather than on the scene.
static void test_revolute_joint_period( void )
{
	printf( "revolute joint pendulum period\n" );

	b3WorldId worldId = makeStepWorld( V( 0, -10, 0 ) );
	b3World* world = b3GetWorldFromId( worldId );

	const double d = 2.0;

	b3BodyId anchor, arm;
	hingeArm( worldId, world, d, false, &anchor, &arm );

	// Displaced by about 10 degrees from the bottom, which is small enough for
	// the small-angle form to hold to well under a percent.
	const double theta0 = 0.175;
	b3Body_SetTransform( arm, V( d * sin( theta0 ), -d * cos( theta0 ), 0 ), b3Quat_identity );

	// Count zero crossings of the x coordinate. Two crossings is one full
	// period, and counting them over many swings averages the sampling error
	// down rather than letting one step's resolution set the answer.
	int crossings = 0;
	int firstCrossStep = -1;
	int lastCrossStep = -1;
	double previousX = F( b3Body_GetPosition( arm ).x );

	const int steps = 1800; // 30 s
	for ( int i = 0; i < steps; ++i )
	{
		b3World_Step( worldId, 4 );
		double x = F( b3Body_GetPosition( arm ).x );

		if ( ( previousX > 0.0 && x <= 0.0 ) || ( previousX < 0.0 && x >= 0.0 ) )
		{
			crossings++;
			if ( firstCrossStep < 0 )
			{
				firstCrossStep = i;
			}
			lastCrossStep = i;
		}
		previousX = x;
	}
	validate( world );

	check( "the pendulum swings", crossings >= 4 );

	// Measured between the first and last crossing, so the initial transient
	// is outside the window.
	double halfPeriods = (double)( crossings - 1 );
	double period = 2.0 * ( (double)( lastCrossStep - firstCrossStep ) / 60.0 ) / halfPeriods;
	double want = 2.0 * M_PI * sqrt( d / 10.0 );

	printf( "  %d crossings over %d steps: period %.5f s, 2*pi*sqrt(d/g) predicts %.5f (ratio %.4f)\n", crossings,
			steps, period, want, period / want );

	// 3%. The residual is the sphere's own rotational inertia, which the point
	// mass form ignores, plus the finite amplitude -- both systematic and both
	// small. A wrong effective mass is a factor, not a few percent.
	expect( "the period matches the pendulum closed form", period, want, want * 0.03 );

	b3DestroyWorld( worldId );
}

/// The reaction torque, against the one number a hinge motor makes obvious: a
/// motor holding a horizontal arm still is applying exactly m*g*d.
///
/// This is the torque chain's `m*g`, and it validates b3MulImpFToF through the
/// angular path, the axial effective mass, and the motor's torque bound
/// together. Averaged over many samples for the reason Stage 2 established:
/// the equilibrium impulse falls between two Q15.16 values and dithers.
static void test_revolute_joint_torque( void )
{
	printf( "revolute joint reaction torque\n" );

	b3WorldId worldId = makeStepWorld( V( 0, -10, 0 ) );
	b3World* world = b3GetWorldFromId( worldId );

	const double d = 1.5;

	b3BodyId anchor, arm;
	b3JointId jointId = hingeArm( worldId, world, d, true, &anchor, &arm );

	// A motor with speed zero is a brake: it holds the arm where it is, and
	// the torque it needs to do that is the arm's weight times its lever.
	b3RevoluteJoint_EnableMotor( jointId, true );
	b3RevoluteJoint_SetMotorSpeed( jointId, b3f_zero );
	b3RevoluteJoint_SetMaxMotorTorque( jointId, b3fFromDouble( 5000.0 ) );

	for ( int i = 0; i < 300; ++i )
	{
		b3World_Step( worldId, 4 );
	}
	validate( world );

	double mass = F( b3Body_GetMass( arm ) );

	// The arm must actually still be near horizontal, or the lever arm in the
	// closed form is not d and the comparison means nothing.
	//
	// It sags about 8 cm on a 1.5 m arm -- three degrees, which is the soft
	// constraint's give under a full gravity load and not slack in the motor.
	// The tolerance is set from that: at three degrees the true lever is
	// d*cos(3deg) = 0.9986*d, which is the 0.13% the torque ratio below
	// actually shows.
	expect( "the motor holds the arm near horizontal", F( b3Body_GetPosition( arm ).y ), 0.0, 0.12 );

	double sumTorque = 0.0;
	double sumMotor = 0.0;
	const int samples = 60;
	for ( int s = 0; s < samples; ++s )
	{
		b3World_Step( worldId, 4 );

		b3Vec3 t = b3Joint_GetConstraintTorque( jointId );
		sumTorque += sqrt( F( t.x ) * F( t.x ) + F( t.y ) * F( t.y ) + F( t.z ) * F( t.z ) );
		sumMotor += fabs( F( b3RevoluteJoint_GetMotorTorque( jointId ) ) );
	}
	validate( world );

	double meanTorque = sumTorque / samples;
	double meanMotor = sumMotor / samples;
	double want = mass * 10.0 * d;

	printf( "  mean torque %.4f N-m, motor %.4f, m*g*d = %.4f (ratio %.4f)\n", meanTorque, meanMotor, want,
			meanTorque / want );

	expect( "a held horizontal arm needs m*g*d", meanTorque, want, want * 0.05 );
	expect( "and the motor is supplying it", meanMotor, want, want * 0.05 );

	// A distance joint constrains no rotation, so its torque is a true zero
	// rather than a placeholder -- the case the fifth switch exists to answer.
	{
		b3BodyId other = makeDynamicSphere( worldId, V( 10, 0, 0 ), 0.25 );
		b3DistanceJointDef ddef = b3DefaultDistanceJointDef();
		ddef.base.bodyIdA = anchor;
		ddef.base.bodyIdB = other;
		ddef.length = b3fFromInt( 10 );
		b3JointId distanceId = b3CreateDistanceJoint( worldId, &ddef );

		b3World_Step( worldId, 4 );
		validate( world );

		b3Vec3 none = b3Joint_GetConstraintTorque( distanceId );
		expect( "a distance joint reports no torque", F( none.x ) + F( none.y ) + F( none.z ), 0.0, 1e-6 );
	}

	b3DestroyWorld( worldId );
}

/// The angle limit, including the speculative branch, and the brad round trip.
static void test_revolute_joint_limit( void )
{
	printf( "revolute joint angle limit\n" );

	b3WorldId worldId = makeStepWorld( V( 0, -10, 0 ) );
	b3World* world = b3GetWorldFromId( worldId );

	b3BodyId anchor, arm;
	b3JointId jointId = hingeArm( worldId, world, 1.5, true, &anchor, &arm );

	// The arm starts along +x, which is angle zero, and gravity swings it
	// down -- toward negative angles. A lower limit at -45 degrees stops it
	// well before the bottom, which is a position no other force explains.
	const b3a lower = (b3a)-4096; // -45 degrees
	const b3a upper = (b3a)4096;  // +45 degrees

	b3RevoluteJoint_EnableLimit( jointId, true );
	b3RevoluteJoint_SetLimits( jointId, lower, upper );

	check( "the limit reports enabled", b3RevoluteJoint_IsLimitEnabled( jointId ) );
	checkInt( "lower limit round-trips", b3RevoluteJoint_GetLowerLimit( jointId ), lower );
	checkInt( "upper limit round-trips", b3RevoluteJoint_GetUpperLimit( jointId ), upper );

	// Reversed input is sorted rather than rejected, as the distance joint's
	// length range is.
	b3RevoluteJoint_SetLimits( jointId, upper, lower );
	checkInt( "a reversed range is sorted, lower", b3RevoluteJoint_GetLowerLimit( jointId ), lower );
	checkInt( "and upper", b3RevoluteJoint_GetUpperLimit( jointId ), upper );

	for ( int i = 0; i < 480; ++i )
	{
		b3World_Step( worldId, 4 );
	}
	validate( world );

	b3a settled = b3RevoluteJoint_GetAngle( jointId );
	double settledDeg = (double)settled * ( 360.0 / 32768.0 );
	printf( "  settled at %d brads (%.2f deg), limit is %d (-45.00 deg)\n", (int)settled, settledDeg, (int)lower );

	// 256 brads is 2.8 degrees, which is the soft constraint's give under a
	// full gravity load plus the brad-to-radian narrowing.
	expect( "gravity carries the arm to the lower limit", (double)settled, (double)lower, 256.0 );
	check( "and no further", settled > lower - 256 );

	// Reversed gravity drives it to the upper limit instead, which exercises
	// the other branch and its flipped signs.
	//
	// The same *magnitude* as before, deliberately: a soft limit is pushed
	// further past its stop by a heavier load, so a 3g reversal would overshoot
	// three times as far and the two halves of this test would not be
	// comparable. Matching the loads is what makes one tolerance right for both.
	b3World_SetGravity( worldId, V( 0, 10, 0 ) );
	for ( int i = 0; i < 480; ++i )
	{
		b3World_Step( worldId, 4 );
	}
	validate( world );

	b3a settledUpper = b3RevoluteJoint_GetAngle( jointId );
	printf( "  reversed: settled at %d brads (%.2f deg), limit is %d\n", (int)settledUpper,
			(double)settledUpper * ( 360.0 / 32768.0 ), (int)upper );
	expect( "reversed gravity carries it to the upper limit", (double)settledUpper, (double)upper, 256.0 );

	b3DestroyWorld( worldId );
}

/// The motor, against its two closed forms: it reaches the speed asked for in
/// rad/s, and it stalls when the load exceeds the torque allowed.
static void test_revolute_joint_motor( void )
{
	printf( "revolute joint motor\n" );

	// No gravity, so the motor speed is the whole of the angular velocity.
	b3WorldId worldId = makeStepWorld( V( 0, 0, 0 ) );
	b3World* world = b3GetWorldFromId( worldId );

	b3BodyId anchor, arm;
	b3JointId jointId = hingeArm( worldId, world, 1.0, true, &anchor, &arm );

	b3RevoluteJoint_EnableMotor( jointId, true );
	b3RevoluteJoint_SetMotorSpeed( jointId, b3fFromDouble( 2.0 ) );
	b3RevoluteJoint_SetMaxMotorTorque( jointId, b3fFromDouble( 2000.0 ) );

	check( "the motor reports enabled", b3RevoluteJoint_IsMotorEnabled( jointId ) );
	expect( "motor speed round-trips", F( b3RevoluteJoint_GetMotorSpeed( jointId ) ), 2.0, 1e-3 );
	expect( "max motor torque round-trips", F( b3RevoluteJoint_GetMaxMotorTorque( jointId ) ), 2000.0, 1e-1 );

	for ( int i = 0; i < 120; ++i )
	{
		b3World_Step( worldId, 4 );
	}
	validate( world );

	// The whole body spins with the hinge, so its angular velocity about z is
	// the motor speed.
	double spin = F( b3Body_GetAngularVelocity( arm ).z );
	printf( "  motor at 2 rad/s: arm spinning at %.5f rad/s\n", spin );
	expect( "the motor reaches the speed asked for", spin, 2.0, 0.05 );

	b3DestroyWorld( worldId );

	// The bound. A motor limited to a torque below m*g*d cannot hold a
	// horizontal arm up, so the arm falls.
	worldId = makeStepWorld( V( 0, -10, 0 ) );
	world = b3GetWorldFromId( worldId );
	jointId = hingeArm( worldId, world, 1.5, true, &anchor, &arm );

	b3RevoluteJoint_EnableMotor( jointId, true );
	b3RevoluteJoint_SetMotorSpeed( jointId, b3f_zero );
	b3RevoluteJoint_SetMaxMotorTorque( jointId, b3fFromDouble( 0.5 ) );

	for ( int i = 0; i < 240; ++i )
	{
		b3World_Step( worldId, 4 );
	}
	validate( world );

	printf( "  motor bounded at 0.5 N-m: arm y = %.4f (started at 0)\n", F( b3Body_GetPosition( arm ).y ) );
	check( "a motor bounded below the load cannot hold the arm", F( b3Body_GetPosition( arm ).y ) < -0.3 );
	check( "and its reported torque respects the bound",
		   fabs( F( b3RevoluteJoint_GetMotorTorque( jointId ) ) ) <= 0.55 );

	// Disabling clears the accumulator, so a re-enable does not warm start
	// from a torque the caller has since changed their mind about.
	b3RevoluteJoint_EnableMotor( jointId, false );
	check( "disabling the motor reports it off", b3RevoluteJoint_IsMotorEnabled( jointId ) == false );
	expect( "and clears its impulse", F( b3RevoluteJoint_GetMotorTorque( jointId ) ), 0.0, 1e-6 );

	b3DestroyWorld( worldId );
}

/// The singular branch: a hinge between two bodies that cannot rotate at all.
///
/// `fixedRotation` guards the axial and collinearity constraints, whose
/// effective masses are singular here. A normal scene never reaches it, which
/// is exactly why it is worth a test -- the alternative is finding it on
/// hardware the first time somebody locks a body's rotation.
static void test_revolute_joint_fixed_rotation( void )
{
	printf( "revolute joint between rotation-locked bodies\n" );

	b3WorldId worldId = makeStepWorld( V( 0, -10, 0 ) );
	b3World* world = b3GetWorldFromId( worldId );

	b3BodyDef anchorDef = b3DefaultBodyDef();
	anchorDef.type = b3_staticBody;
	anchorDef.position = V( 0, 0, 0 );
	b3BodyId anchor = b3CreateBody( worldId, &anchorDef );

	b3BodyDef armDef = b3DefaultBodyDef();
	armDef.type = b3_dynamicBody;
	armDef.position = V( 1.0, 0, 0 );
	armDef.motionLocks.angularX = true;
	armDef.motionLocks.angularY = true;
	armDef.motionLocks.angularZ = true;
	armDef.enableSleep = false;
	b3BodyId arm = b3CreateBody( worldId, &armDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3Sphere ball = { V( 0, 0, 0 ), b3fFromDouble( 0.1 ) };
	b3CreateSphereShape( arm, &shapeDef, &ball );

	b3RevoluteJointDef def = b3DefaultRevoluteJointDef();
	def.base.bodyIdA = anchor;
	def.base.bodyIdB = arm;
	def.base.localFrameB.p = V( -1.0, 0, 0 );
	def.enableLimit = true;
	def.enableMotor = true;
	def.maxMotorTorque = b3fFromDouble( 100.0 );

	b3JointId jointId = b3CreateRevoluteJoint( worldId, &def );
	validate( world );

	// The point-to-point constraint still applies -- only the rotational half
	// is disabled -- so the arm hangs from the pivot and swings, it just
	// cannot spin about its own centre.
	for ( int i = 0; i < 300; ++i )
	{
		b3World_Step( worldId, 4 );
	}
	validate( world );

	b3Vec3 pivot = b3TransformPoint( b3Body_GetTransform( arm ), V( -1.0, 0, 0 ) );
	printf( "  pivot held at (%.5f %.5f %.5f) with rotation locked\n", F( pivot.x ), F( pivot.y ), F( pivot.z ) );

	check( "the joint survives a singular rotational mass", b3Joint_IsValid( jointId ) );
	expect( "and the point constraint still holds, x", F( pivot.x ), 0.0, 0.05 );
	expect( "and y", F( pivot.y ), 0.0, 0.05 );

	b3DestroyWorld( worldId );
}

/// Two light bodies on one hinge -- the case that used to wrap.
///
/// Stage 5's regression, and the revolute's counterpart of the density-1
/// ragdoll below. b3InvertInertia caps a *single* body's inverse inertia at
/// Q7.24's ceiling of 128 by scaling rather than wrapping, so no body can
/// overflow -- but a joint divides by the **sum** of two, and two light bodies
/// sum past it. Stage 3 wrote this file against `b3AddMWMW( iA, iB )`, which
/// wrapped that sum to a large negative and inverted the sign of the effective
/// mass: the constraint pushed the bodies apart instead of holding them.
///
/// Both the axial mass and the 2x2 collinearity block had it, so the scene
/// drives both -- a motor loads the axial term, and the arm is started off-axis
/// so the perpendicular term has work to do.
static void test_revolute_joint_light_bodies( void )
{
	printf( "revolute joint between two light bodies\n" );

	b3WorldId worldId = makeStepWorld( V( 0, -10, 0 ) );
	b3World* world = b3GetWorldFromId( worldId );

	// Sized against the ceiling deliberately: a 0.6 m cube at density 1 weighs
	// 0.215 kg -- the ragdoll's weight class -- and has an inverse inertia of
	// 77.8. Comfortably inside Q7.24 on its own, and **155.6 in a pair**,
	// which is past the ceiling of 128. Neither body's own inverse mass or
	// inertia saturates: b3InvertInertia's uniform down-scale for a body too
	// small to represent is a separate limit and is not what this test is
	// about, so the scene is kept clear of it.
	//
	// The hulls are declared here, not inside a loop: b3CreateHullShape stores
	// the caller's pointer (shape.c:155), which cost Stage 4 a wrong diagnosis.
	b3BoxHull hullA = b3MakeBoxHull( b3fFromDouble( 0.3 ), b3fFromDouble( 0.3 ), b3fFromDouble( 0.3 ) );
	b3BoxHull hullB = b3MakeBoxHull( b3fFromDouble( 0.3 ), b3fFromDouble( 0.3 ), b3fFromDouble( 0.3 ) );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = b3fFromDouble( 1.0 );

	b3BodyDef defA = b3DefaultBodyDef();
	defA.type = b3_dynamicBody;
	defA.position = V( 0, 0, 0 );
	defA.enableSleep = false;
	b3BodyId bodyA = b3CreateBody( worldId, &defA );
	b3CreateHullShape( bodyA, &shapeDef, &hullA.base );

	b3BodyDef defB = b3DefaultBodyDef();
	defB.type = b3_dynamicBody;
	defB.position = V( 1.0, 0, 0 );
	defB.enableSleep = false;
	b3BodyId bodyB = b3CreateBody( worldId, &defB );
	b3CreateHullShape( bodyB, &shapeDef, &hullB.base );

	// Assert the scene actually reaches the condition rather than describing
	// it. A regression whose trigger has quietly stopped triggering is worse
	// than no regression at all -- it reports success for the wrong reason --
	// so the pair's inverse inertia is read straight off the sims and checked
	// against Q7.24's ceiling.
	{
		b3Body* ba = b3GetBodyFullId( world, bodyA );
		b3Body* bb = b3GetBodyFullId( world, bodyB );
		b3SolverSet* setA = b3Array_Get( world->solverSets, ba->setIndex );
		b3SolverSet* setB = b3Array_Get( world->solverSets, bb->setIndex );
		b3BodySim* simA = b3Array_Get( setA->bodySims, ba->localIndex );
		b3BodySim* simB = b3Array_Get( setB->bodySims, bb->localIndex );

		double iA = b3iwToDouble( simA->invInertiaWorld.cx.x );
		double iB = b3iwToDouble( simB->invInertiaWorld.cx.x );
		const double ceiling = 128.0;

		printf( "  mass %.3f kg each, inverse inertia %.1f + %.1f = %.1f against a ceiling of %.0f\n",
				F( b3Body_GetMass( bodyA ) ), iA, iB, iA + iB, ceiling );

		check( "neither body is on its own past the ceiling", iA < ceiling && iB < ceiling );
		check( "but the pair is -- the scene reaches the case that used to wrap", iA + iB > ceiling );
	}

	b3RevoluteJointDef def = b3DefaultRevoluteJointDef();
	def.base.bodyIdA = bodyA;
	def.base.bodyIdB = bodyB;
	def.base.localFrameB.p = V( -1.0, 0, 0 );
	def.base.collideConnected = false;

	// A motor loads the axial term, and the perpendicular term has work to do
	// because the pair is free to tumble -- so both wrapped quantities are
	// driven, not just the one prepare computes.
	def.enableMotor = true;
	def.maxMotorTorque = b3fFromDouble( 1.0 );
	def.motorSpeed = b3fFromDouble( 2.0 );

	b3JointId jointId = b3CreateRevoluteJoint( worldId, &def );
	validate( world );

	for ( int i = 0; i < 600; ++i )
	{
		b3World_Step( worldId, 4 );
	}
	validate( world );

	// The pair falls together under gravity, so their *separation* is the
	// measure rather than either position. A wrapped effective mass showed up
	// as this growing without bound within a few frames.
	b3Vec3 pa = b3Body_GetPosition( bodyA );
	b3Vec3 pb = b3Body_GetPosition( bodyB );
	double dx = F( pb.x ) - F( pa.x );
	double dy = F( pb.y ) - F( pa.y );
	double dz = F( pb.z ) - F( pa.z );
	double separation = sqrt( dx * dx + dy * dy + dz * dz );

	printf( "  separation after 600 steps %.5f (built at 1.0), centre fell to y %.3f\n", separation, F( pa.y ) );

	check( "the joint survives two light bodies", b3Joint_IsValid( jointId ) );
	expect( "and the hinge holds its length rather than inverting", separation, 1.0, 0.03 );

	// Free fall: 600 steps at 1/60 s is 10 s, so -0.5*g*t^2 is -500. Checked
	// loosely -- the point is that the pair is falling *together*, not that
	// gravity integrates exactly, which other tests cover.
	check( "and the pair is in free fall rather than flung apart", F( pa.y ) < -300.0 && F( pa.y ) > -700.0 );

	b3DestroyWorld( worldId );
}

/// Two light spheres in rolling contact -- the contact solver's counterpart of
/// the case above, and Stage 6's opening regression.
///
/// Stage 5 closed `b3AddMWMW( iA, iB )` in the revolute joint and recorded that
/// contact_solver.c still formed it in two places: `rollingMass` and the twist
/// term. Same defect family, same threshold of 128, same reachability by two
/// ordinary light bodies -- but on the hot path, with baselines behind every
/// contact number in the suite, so it was carried rather than fixed blind.
///
/// This is the scene that reaches it. Both quantities are angular effective
/// masses divided by the **sum** of two inverse inertias, and both are read only
/// when a contact has rolling resistance or twist friction -- which is why no
/// existing test moved when they wrapped, and why the ragdoll (five jointed
/// bodies resting in contact) was the one scene in the whole suite whose numbers
/// did change when this was fixed.
///
/// Spheres rather than boxes because rollingResistance is supported on spheres
/// and capsules only (types.h:1206), and rolling resistance is what makes
/// `rollingMass` a divisor rather than dead weight.
static void test_contact_light_bodies_rolling( void )
{
	printf( "rolling contact between two light bodies\n" );

	// No gravity: the joint supplies the contact load, so the pair hangs in
	// place and the only thing acting on the spin is the rolling resistance the
	// wrapped mass divides. Gravity would add a falling trajectory to read
	// through and would prove nothing extra.
	b3WorldId worldId = makeStepWorld( V( 0, 0, 0 ) );
	b3World* world = b3GetWorldFromId( worldId );

	// Sized to isolate *this* wrap from its neighbour, which is the whole
	// difficulty of the scene and worth stating precisely.
	//
	// Two sums can overflow Q7.24's ceiling of 128 in a contact, and they are
	// independent defects:
	//
	//   - `invIA + invIB`, the angular one, which is what rollingMass and the
	//     twist mass divide by and what Stage 6 closed;
	//   - `invMassA + invMassB`, the linear one, in kNormal and the tangent
	//     block -- **still open**, and reachable on its own because
	//     B3_MIN_MASS_RAW caps a single inverse mass at ~124, so any two bodies
	//     below about 16 g sum past the ceiling.
	//
	// A first draft of this test used 7 g spheres and hit both at once: the pair
	// was launched to x = 3862 after 600 steps, which proves the linear wrap is
	// live but says nothing about the angular one. So the radius and density
	// here are chosen to put the inertia sum well past 128 while keeping the
	// mass sum near 10 -- two orders of magnitude clear of it. Both bounds are
	// asserted below rather than trusted.
	b3Sphere sphereA = { V( 0, 0, 0 ), b3fFromDouble( 0.38 ) };
	b3Sphere sphereB = { V( 0, 0, 0 ), b3fFromDouble( 0.38 ) };

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = b3fFromDouble( 0.9 );
	shapeDef.baseMaterial.friction = b3cFromFrac( 8, 10 );
	shapeDef.baseMaterial.rollingResistance = b3cFromFrac( 1, 2 );

	// Both bodies must be **dynamic**, and this is the trap the first draft fell
	// into: a sphere resting on a static floor has `iB = b3MatW_zero`, so
	// `iA + iB` is just `iA`, which b3InvertInertia already guarantees is inside
	// the ceiling. A static contact can never reach this wrap. Only two dynamic
	// bodies touching each other can, which is why the ragdoll -- five jointed
	// dynamic bodies resting on one another -- was the one scene in the suite
	// whose numbers moved when this was fixed.
	//
	// So the pair is held together by a rigid distance joint shorter than the two
	// radii, with collideConnected on: the joint pulls them together, the contact
	// pushes them apart, and the contact therefore carries a sustained normal
	// impulse for as long as the test runs. Deliberately the "welded assembly
	// packed flush self-collides" configuration Stage 5's crate found by
	// accident -- here it is exactly the load the test needs.
	b3BodyDef defA = b3DefaultBodyDef();
	defA.type = b3_dynamicBody;
	defA.position = V( 0, 0.38, 0 );
	defA.enableSleep = false;
	// Spun hard about all three axes, so the twist term and the rolling term are
	// both loaded. A pair at rest divides by neither.
	b3BodyId bodyA = b3CreateBody( worldId, &defA );
	b3CreateSphereShape( bodyA, &shapeDef, &sphereA );

	b3BodyDef defB = b3DefaultBodyDef();
	defB.type = b3_dynamicBody;
	defB.position = V( 0.745, 0.38, 0 );
	defB.enableSleep = false;
	defB.angularVelocity = V( -6.0, -6.0, -6.0 );
	b3BodyId bodyB = b3CreateBody( worldId, &defB );
	b3CreateSphereShape( bodyB, &shapeDef, &sphereB );

	// Rigid, and shorter than the 0.76 the two radii want -- so the pair is held
	// in permanent compression and the contact never goes idle.
	b3DistanceJointDef squeeze = b3DefaultDistanceJointDef();
	squeeze.base.bodyIdA = bodyA;
	squeeze.base.bodyIdB = bodyB;
	squeeze.base.collideConnected = true;
	squeeze.length = b3fFromDouble( 0.70 );
	squeeze.enableSpring = false;
	b3CreateDistanceJoint( worldId, &squeeze );

	// Assert the scene reaches the condition rather than describing it -- the
	// revolute regression's rule, and the reason that one still earns its keep.
	{
		b3Body* ba = b3GetBodyFullId( world, bodyA );
		b3Body* bb = b3GetBodyFullId( world, bodyB );
		b3SolverSet* setA = b3Array_Get( world->solverSets, ba->setIndex );
		b3SolverSet* setB = b3Array_Get( world->solverSets, bb->setIndex );
		b3BodySim* simA = b3Array_Get( setA->bodySims, ba->localIndex );
		b3BodySim* simB = b3Array_Get( setB->bodySims, bb->localIndex );

		double iA = b3iwToDouble( simA->invInertiaWorld.cx.x );
		double iB = b3iwToDouble( simB->invInertiaWorld.cx.x );
		double mA = b3iwToDouble( simA->invMass );
		double mB = b3iwToDouble( simB->invMass );
		const double ceiling = 128.0;

		printf( "  mass %.4f kg each; inverse inertia %.1f + %.1f = %.1f, inverse mass %.2f + %.2f = %.2f"
				" (ceiling %.0f)\n",
				F( b3Body_GetMass( bodyA ) ), iA, iB, iA + iB, mA, mB, mA + mB, ceiling );

		check( "neither sphere is on its own past the ceiling", iA < ceiling && iB < ceiling );
		check( "but the inertia pair is -- the scene reaches the wrap Stage 6 closed", iA + iB > ceiling );

		// The bound that keeps this test about one defect. If a later change to
		// the scene lets the mass sum approach 128 too, the linear wrap joins in
		// and a failure here stops meaning what it says.
		check( "and the mass sum is nowhere near it, so the linear wrap stays out of this test",
			   mA + mB < 0.25 * ceiling );
	}

	validate( world );

	for ( int i = 0; i < 600; ++i )
	{
		b3World_Step( worldId, 4 );
	}
	validate( world );

	b3Vec3 pa = b3Body_GetPosition( bodyA );
	b3Vec3 pb = b3Body_GetPosition( bodyB );
	b3Vec3 wa = b3Body_GetAngularVelocity( bodyA );
	double spin = sqrt( F( wa.x ) * F( wa.x ) + F( wa.y ) * F( wa.y ) + F( wa.z ) * F( wa.z ) );

	double dx = F( pb.x ) - F( pa.x );
	double dy = F( pb.y ) - F( pa.y );
	double dz = F( pb.z ) - F( pa.z );
	double separation = sqrt( dx * dx + dy * dy + dz * dz );

	printf( "  after 600 steps: spin %.4f rad/s (started 10.392), separation %.4f (joint asks 0.70)\n", spin,
			separation );

	// Rolling resistance is a *sink*: it can only remove angular momentum, never
	// add it. A wrapped rollingMass inverts that sign, so every sub-step fed the
	// spin instead of damping it and the pair spun up without bound. This is the
	// assertion the fix is for, and it is a one-sided bound rather than a value
	// because how *fast* it decays is a tuning question and the sign is not.
	check( "rolling resistance removed spin rather than adding it", spin < 10.392 );
	check( "and did not run away", spin < 1000.0 );

	// The joint still holds the pair at its length. A negative twist or rolling
	// mass showed up here as the two spheres being driven apart.
	expect( "the pair is still held together", separation, 0.70, 0.10 );

	b3DestroyWorld( worldId );
}

/// Two spheres light enough that their *inverse masses* sum past the ceiling.
///
/// The linear half of Stage 6's overflow work, and the one no existing scene
/// reached -- closing it moved not a single number anywhere else in the suite,
/// which is exactly why it needs a test of its own rather than a baseline.
///
/// B3_MIN_MASS_RAW caps a single inverse mass at about 124, just inside Q7.24's
/// ceiling of 128. So one body always fits, and no amount of testing one body
/// finds anything. **Two** bodies under roughly 16 g each do not, and `b3AddW`
/// is a plain int32 add in a device build, so `mA + mB` came back negative and
/// the normal mass with it -- a contact that pushed the pair together in
/// proportion to how hard it was separating them.
///
/// Measured before the fix, on this scene: the pair reached x = 3862 after 600
/// steps instead of resting. That is what the bound below is written against.
static void test_contact_light_mass_pair( void )
{
	printf( "contact between two bodies light enough to wrap the inverse-mass sum\n" );

	b3WorldId worldId = makeStepWorld( V( 0, -10, 0 ) );
	b3World* world = b3GetWorldFromId( worldId );

	// 7 g each: a 0.12 m sphere at density 1, which is an ordinary pebble and
	// not a contrivance.
	b3Sphere sphereA = { V( 0, 0, 0 ), b3fFromDouble( 0.12 ) };
	b3Sphere sphereB = { V( 0, 0, 0 ), b3fFromDouble( 0.12 ) };

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = b3fFromDouble( 1.0 );
	shapeDef.baseMaterial.friction = b3cFromFrac( 8, 10 );
	shapeDef.baseMaterial.rollingResistance = b3cFromFrac( 1, 2 );

	// **No joint here, unlike the rolling test above, and that is deliberate.**
	// A rigid joint between the pair overpowers the contact and hides the wrap
	// entirely -- tried, and the pair stayed put either way. The two spheres
	// must be free to be flung, which means resting on a floor and touching
	// each other with nothing else holding them.
	b3BodyDef floorDef = b3DefaultBodyDef();
	floorDef.type = b3_staticBody;
	floorDef.position = V( 0, -0.5, 0 );
	b3BodyId floor = b3CreateBody( worldId, &floorDef );
	b3BoxHull floorHull = b3MakeBoxHull( b3fFromDouble( 20.0 ), b3fFromDouble( 0.5 ), b3fFromDouble( 20.0 ) );
	b3ShapeDef floorShapeDef = b3DefaultShapeDef();
	floorShapeDef.baseMaterial.friction = b3cFromFrac( 8, 10 );
	floorShapeDef.baseMaterial.rollingResistance = b3cFromFrac( 1, 2 );
	b3CreateHullShape( floor, &floorShapeDef, &floorHull.base );

	// Note which contact carries the defect: sphere-against-floor cannot, since
	// a static body's inverse mass is zero and 124 + 0 is inside the ceiling.
	// It is the **sphere-against-sphere** contact that sums 248 and wraps.
	b3BodyDef defA = b3DefaultBodyDef();
	defA.type = b3_dynamicBody;
	defA.position = V( 0, 0.12, 0 );
	defA.enableSleep = false;
	defA.angularVelocity = V( 6.0, 6.0, 6.0 );
	b3BodyId bodyA = b3CreateBody( worldId, &defA );
	b3CreateSphereShape( bodyA, &shapeDef, &sphereA );

	b3BodyDef defB = b3DefaultBodyDef();
	defB.type = b3_dynamicBody;
	defB.position = V( 0.235, 0.12, 0 );
	defB.enableSleep = false;
	defB.angularVelocity = V( -6.0, -6.0, -6.0 );
	b3BodyId bodyB = b3CreateBody( worldId, &defB );
	b3CreateSphereShape( bodyB, &shapeDef, &sphereB );

	{
		b3Body* ba = b3GetBodyFullId( world, bodyA );
		b3Body* bb = b3GetBodyFullId( world, bodyB );
		b3SolverSet* setA = b3Array_Get( world->solverSets, ba->setIndex );
		b3SolverSet* setB = b3Array_Get( world->solverSets, bb->setIndex );
		b3BodySim* simA = b3Array_Get( setA->bodySims, ba->localIndex );
		b3BodySim* simB = b3Array_Get( setB->bodySims, bb->localIndex );

		double mA = b3iwToDouble( simA->invMass );
		double mB = b3iwToDouble( simB->invMass );
		const double ceiling = 128.0;

		printf( "  mass %.4f kg each, inverse mass %.1f + %.1f = %.1f against a ceiling of %.0f\n",
				F( b3Body_GetMass( bodyA ) ), mA, mB, mA + mB, ceiling );

		check( "neither sphere's inverse mass is on its own past the ceiling", mA < ceiling && mB < ceiling );
		check( "but the pair's is -- the scene reaches the linear wrap", mA + mB > ceiling );
	}

	validate( world );

	for ( int i = 0; i < 600; ++i )
	{
		b3World_Step( worldId, 4 );
	}
	validate( world );

	b3Vec3 pa = b3Body_GetPosition( bodyA );
	b3Vec3 pb = b3Body_GetPosition( bodyB );
	printf( "  after 600 steps: A at (%.3f %.3f %.3f), B at (%.3f %.3f %.3f)\n", F( pa.x ), F( pa.y ), F( pa.z ),
			F( pb.x ), F( pb.y ), F( pb.z ) );

	// The two spheres are spun into each other, so they *do* roll apart, and how
	// far is a friction question rather than a correctness one. The bound is set
	// between the two measured behaviours rather than fitted to either:
	//
	//   fixed:    A at x = -7.05,    B at x = +7.29,   both at y = 0.119
	//   wrapped:  A at x = +3864,    B at x = -3950,   both at y = -490
	//
	// Two and a half orders of magnitude apart, so 100 separates them with room
	// on both sides and does not pretend to measure the rolling.
	check( "sphere A rolled rather than launching", fabs( F( pa.x ) ) < 100.0 && fabs( F( pa.z ) ) < 100.0 );
	check( "and so did sphere B", fabs( F( pb.x ) ) < 100.0 && fabs( F( pb.z ) ) < 100.0 );

	// And both are still on the floor rather than through it. A pair flung hard
	// enough tunnels, which is how the first measurement reached y = -490.
	check( "both spheres are still resting on the floor", F( pa.y ) > 0.0 && F( pa.y ) < 0.5 &&
														  F( pb.y ) > 0.0 && F( pb.y ) < 0.5 );

	b3DestroyWorld( worldId );
}

/// Sleep, wake, and the accessor round trips, with a sim that is larger again.
static void test_revolute_joint_sleep_and_accessors( void )
{
	printf( "revolute joint through sleep, and accessor round-trips\n" );

	b3WorldId worldId = makeStepWorld( V( 0, -10, 0 ) );
	b3World* world = b3GetWorldFromId( worldId );

	b3BodyId anchor, arm;
	b3JointId jointId = hingeArm( worldId, world, 1.0, false, &anchor, &arm );

	b3Body_EnableSleep( arm, true );

	int steps = 0;
	while ( steps < 1200 && b3GetBodyFullId( world, arm )->setIndex == b3_awakeSet )
	{
		b3World_Step( worldId, 4 );
		steps++;
	}
	validate( world );
	check( "a hinged body settles and sleeps", steps < 1200 );

	b3a asleep = b3RevoluteJoint_GetAngle( jointId );

	b3Body_SetAwake( arm, true );
	validate( world );
	checkInt( "waking returns the joint to the graph", b3GetJointFullId( world, jointId )->colorIndex,
			  B3_OVERFLOW_INDEX );

	// The step after waking is where a mis-copied 376-byte sim shows up: the
	// frames and the six accumulators have to be the ones it slept with.
	b3World_Step( worldId, 4 );
	validate( world );
	expect( "and the hinge resumes without a jolt", (double)b3RevoluteJoint_GetAngle( jointId ), (double)asleep,
			128.0 );

	// Accessors.
	b3RevoluteJoint_SetSpringHertz( jointId, b3fFromDouble( 4.5 ) );
	expect( "spring hertz round-trips", F( b3RevoluteJoint_GetSpringHertz( jointId ) ), 4.5, 1e-2 );

	b3RevoluteJoint_SetSpringDampingRatio( jointId, b3fFromDouble( 1.75 ) );
	expect( "spring damping round-trips", F( b3RevoluteJoint_GetSpringDampingRatio( jointId ) ), 1.75, 1e-2 );

	b3RevoluteJoint_SetTargetAngle( jointId, (b3a)2048 );
	checkInt( "target angle round-trips", b3RevoluteJoint_GetTargetAngle( jointId ), 2048 );

	b3RevoluteJoint_EnableSpring( jointId, true );
	check( "spring reports enabled", b3RevoluteJoint_IsSpringEnabled( jointId ) );

	// The base accessors still work on a typed joint -- the union sits after
	// them, so a wrong offset would corrupt one or the other.
	b3Joint_SetUserData( jointId, (void*)(intptr_t)0x5EED );
	checkInt( "base user data still round-trips", (intptr_t)b3Joint_GetUserData( jointId ), 0x5EED );
	checkInt( "and body A", b3Joint_GetBodyA( jointId ).index1, anchor.index1 );

	b3DestroyJoint( jointId, true );
	validate( world );
	check( "a destroyed revolute joint is not valid", b3Joint_IsValid( jointId ) == false );
	checkInt( "and the body no longer counts it", b3GetBodyFullId( world, arm )->jointCount, 0 );

	b3DestroyWorld( worldId );
}

// =========================================================================
// Spherical joint
// =========================================================================
//
// Phase 6 Stage 4. The revolute's point-to-point constraint, unchanged, plus a
// 3-vector spring, a 3-vector motor, and two rotational *bounds* -- a cone on
// how far the axes may tilt apart and a twist about them.
//
// The case worth reading first is the conical pendulum. It is Stage 4's
// counterpart to Stage 2's spring deflection and Stage 3's pendulum period: a
// closed form in which the mass cancels, so it tests the effective mass and
// nothing the scene controls -- and unlike the hinge's period it exercises all
// three rotational degrees at once rather than one at a time.

/// A ball joint to a static anchor, with a mass hanging beneath it.
///
/// The joint frames' z axis is the cone axis, and this scene points it **down**
/// (-y) so that the hanging rest pose sits at the *centre* of the cone rather
/// than at its edge. Getting that wrong gives a scene whose cone limit is
/// already violated at rest, which looks like a solver fault and is not.
static b3JointId ballArm( b3WorldId worldId, b3World* world, double armLength, b3BodyId* anchorOut, b3BodyId* armOut )
{
	b3BodyDef anchorDef = b3DefaultBodyDef();
	anchorDef.type = b3_staticBody;
	anchorDef.position = V( 0, 0, 0 );
	b3BodyId anchor = b3CreateBody( worldId, &anchorDef );

	b3BodyDef armDef = b3DefaultBodyDef();
	armDef.type = b3_dynamicBody;
	armDef.position = V( 0, -armLength, 0 );

	// Sleep off, for the reason hingeArm gives: most of these measure an
	// accumulator, and a sleeping body freezes one mid-settle.
	armDef.enableSleep = false;
	b3BodyId arm = b3CreateBody( worldId, &armDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3Sphere ball = { V( 0, 0, 0 ), b3fFromDouble( 0.1 ) };
	b3CreateSphereShape( arm, &shapeDef, &ball );

	b3SphericalJointDef def = b3DefaultSphericalJointDef();
	def.base.bodyIdA = anchor;
	def.base.bodyIdB = arm;

	// Rotate both frames so their z axis points along -y, the direction the arm
	// hangs. A quarter turn about x takes +z to -y.
	b3Quat downward = b3MakeQuatFromAxisAngle( V( 1, 0, 0 ), (b3a)B3_BRAD_HALF_PI );
	def.base.localFrameA.q = downward;
	def.base.localFrameB.q = downward;

	// Frame A sits at the pivot (the anchor's origin); frame B sits at the pivot
	// expressed in the arm's local space, which is +armLength along y.
	def.base.localFrameB.p = V( 0, armLength, 0 );

	b3JointId jointId = b3CreateSphericalJoint( worldId, &def );
	validate( world );

	*anchorOut = anchor;
	*armOut = arm;
	return jointId;
}

/// The point-to-point constraint: a ball joint may rotate freely but must not
/// let its anchor drift.
static void test_spherical_joint_holds( void )
{
	printf( "spherical joint holds its anchor\n" );

	b3WorldId worldId = makeStepWorld( V( 0, -10, 0 ) );
	b3World* world = b3GetWorldFromId( worldId );

	const double d = 1.5;

	b3BodyId anchor, arm;
	ballArm( worldId, world, d, &anchor, &arm );

	// Loaded off-axis, so the constraint is carrying a torque as well as a
	// force -- a pivot that only holds under a symmetric load holds nothing.
	b3Body_ApplyLinearImpulse( arm, V( 3.0, 0, 2.0 ), b3Body_GetWorldCenter( arm ), true );

	double firstError = 0.0;
	double lastError = 0.0;

	for ( int i = 0; i < 600; ++i )
	{
		b3World_Step( worldId, 4 );

		// The pivot is where frame B's origin has ended up. It must stay at the
		// anchor's origin, which is the world origin here.
		b3Vec3 p = b3Body_GetPosition( arm );
		double radius = sqrt( F( p.x ) * F( p.x ) + F( p.y ) * F( p.y ) + F( p.z ) * F( p.z ) );
		double error = fabs( radius - d );

		if ( i == 59 )
		{
			firstError = error;
		}
		lastError = error;
	}
	validate( world );

	printf( "  pivot radius error at 1 s %.6f, at 10 s %.6f (arm is %.1f m)\n", firstError, lastError, d );

	// One linear slop is 0.005. Holding to well inside that over 600 steps is
	// the constraint working; the creep check is what would catch a bias that a
	// single sample cannot distinguish from noise.
	expect( "the pivot holds", lastError, 0.0, 0.005 );
	check( "and does not creep", lastError <= firstError + 0.002 );

	b3DestroyWorld( worldId );
}

/// The conical pendulum, and the closed form this stage is worth.
///
/// A mass on a ball joint swinging in a horizontal circle at angle theta from
/// the vertical satisfies omega^2 = g / (L * cos(theta)) -- the mass cancels,
/// so this tests the effective mass and nothing the scene controls. It also
/// drives all three rotational degrees at once, unlike the hinge's period.
static void test_spherical_joint_conical_pendulum( void )
{
	printf( "spherical joint conical pendulum\n" );

	b3WorldId worldId = makeStepWorld( V( 0, -10, 0 ) );
	b3World* world = b3GetWorldFromId( worldId );

	const double g = 10.0;
	const double L = 1.5;

	// Aim for a cone half-angle of 30 degrees. The steady state needs both the
	// position and the tangential speed to match, or the orbit wobbles and the
	// period is not the conical one.
	const double theta = M_PI / 6.0;
	const double r = L * sin( theta );
	const double y = -L * cos( theta );
	const double omega = sqrt( g / ( L * cos( theta ) ) );

	b3BodyId anchor, arm;
	ballArm( worldId, world, L, &anchor, &arm );

	b3Body_SetTransform( arm, V( r, y, 0 ), b3Quat_identity );
	b3Body_SetLinearVelocity( arm, V( 0, 0, omega * r ) );

	// Count crossings of the z-x half plane, as the hinge's period test counts
	// zero crossings -- averaging over many orbits rather than trusting one.
	int crossings = 0;
	int firstCrossStep = -1;
	int lastCrossStep = -1;
	double previousZ = F( b3Body_GetPosition( arm ).z );

	double sumRadius = 0.0;
	double sumHeight = 0.0;
	int samples = 0;

	const int steps = 1800; // 30 s
	for ( int i = 0; i < steps; ++i )
	{
		b3World_Step( worldId, 4 );

		b3Vec3 p = b3Body_GetPosition( arm );
		double z = F( p.z );

		if ( ( previousZ > 0.0 && z <= 0.0 ) || ( previousZ < 0.0 && z >= 0.0 ) )
		{
			crossings++;
			if ( firstCrossStep < 0 )
			{
				firstCrossStep = i;
			}
			lastCrossStep = i;
		}
		previousZ = z;

		if ( i >= 300 )
		{
			sumRadius += sqrt( F( p.x ) * F( p.x ) + F( p.z ) * F( p.z ) );
			sumHeight += F( p.y );
			samples++;
		}
	}
	validate( world );

	check( "the pendulum orbits", crossings >= 4 );

	double halfPeriods = (double)( crossings - 1 );
	double period = 2.0 * ( (double)( lastCrossStep - firstCrossStep ) / 60.0 ) / halfPeriods;
	double want = 2.0 * M_PI / omega;

	double meanRadius = sumRadius / samples;
	double meanHeight = sumHeight / samples;

	printf( "  %d crossings: period %.5f s, 2*pi*sqrt(L cos(t)/g) predicts %.5f (ratio %.4f)\n", crossings, period,
			want, period / want );
	printf( "  orbit radius %.5f (want %.5f), height %.5f (want %.5f)\n", meanRadius, r, meanHeight, y );

	// The orbit must actually still be conical, or the period is not the
	// conical one and the comparison means nothing.
	expect( "the orbit keeps its radius", meanRadius, r, 0.10 );
	expect( "and its height", meanHeight, y, 0.10 );

	// 4%. The residual is the sphere's own rotational inertia and the finite
	// orbit, both systematic and both small. A wrong effective mass would be a
	// factor.
	expect( "the period matches the conical pendulum form", period, want, want * 0.04 );

	b3DestroyWorld( worldId );
}

/// The cone limit, including the speculative branch and the brad round trip.
static void test_spherical_joint_cone_limit( void )
{
	printf( "spherical joint cone limit\n" );

	b3WorldId worldId = makeStepWorld( V( 0, -10, 0 ) );
	b3World* world = b3GetWorldFromId( worldId );

	const double L = 1.0;

	b3BodyId anchor, arm;
	b3JointId jointId = ballArm( worldId, world, L, &anchor, &arm );

	// A 45 degree cone. Gravity pulls the arm to the bottom of the cone, so the
	// limit is reached by pushing sideways rather than by letting go.
	const b3a coneLimit = (b3a)4096; // 45 degrees
	b3SphericalJoint_EnableConeLimit( jointId, true );
	b3SphericalJoint_SetConeLimit( jointId, coneLimit );

	checkInt( "cone limit round-trips in brads", b3SphericalJoint_GetConeLimit( jointId ), coneLimit );

	// Push hard enough to pin the arm against the limit and hold it there.
	for ( int i = 0; i < 900; ++i )
	{
		b3Body_ApplyForceToCenter( arm, V( 60.0, 0, 0 ), true );
		b3World_Step( worldId, 4 );
	}
	validate( world );

	b3a settledPositive = b3SphericalJoint_GetConeAngle( jointId );

	// And the other way, which must be the same angle -- the cone is unsigned
	// and one-sided, so both directions are the *same* branch. That is the
	// difference from the revolute's two mirrored limit branches, and checking
	// it is what would catch a swing axis whose sign depended on direction.
	for ( int i = 0; i < 900; ++i )
	{
		b3Body_ApplyForceToCenter( arm, V( -60.0, 0, 0 ), true );
		b3World_Step( worldId, 4 );
	}
	validate( world );

	b3a settledNegative = b3SphericalJoint_GetConeAngle( jointId );

	printf( "  pushed +x: cone at %d brads (%.2f deg), limit is %d (%.2f deg)\n", (int)settledPositive,
			(double)settledPositive * 360.0 / 32768.0, (int)coneLimit, (double)coneLimit * 360.0 / 32768.0 );
	printf( "  pushed -x: cone at %d brads (%.2f deg)\n", (int)settledNegative,
			(double)settledNegative * 360.0 / 32768.0 );

	// 256 brads is 2.8 degrees of overshoot, which is the soft constraint's give
	// under a load six times the arm's weight. What matters more than the
	// absolute figure is that both directions overshoot by the *same* amount:
	// an asymmetry there would mean the swing axis, not the limit.
	expect( "the cone limit holds pushing +x", (double)settledPositive, (double)coneLimit, 256.0 );
	expect( "and pushing -x", (double)settledNegative, (double)coneLimit, 256.0 );
	expect( "symmetrically", (double)settledNegative, (double)settledPositive, 64.0 );

	// Released, the arm must come back inside the cone -- the limit is a bound,
	// not a weld.
	//
	// Damped first, and deliberately: a ball joint is as lossless as a distance
	// joint, so an undamped arm released at the limit swings through the cone
	// centre and back out to the far side forever. Sampling that at a fixed
	// step measures the phase of the swing rather than whether the limit
	// releases -- which is the mistake box3d_rope made and this test made too.
	b3Body_SetLinearDamping( arm, b3fFromDouble( 1.5 ) );
	b3Body_SetAngularDamping( arm, b3fFromDouble( 1.5 ) );

	for ( int i = 0; i < 1200; ++i )
	{
		b3World_Step( worldId, 4 );
	}
	validate( world );

	check( "and the arm falls back inside the cone", b3SphericalJoint_GetConeAngle( jointId ) < coneLimit / 2 );

	b3DestroyWorld( worldId );
}

/// The twist limit -- the constraint whose Jacobian the port normalizes.
static void test_spherical_joint_twist_limit( void )
{
	printf( "spherical joint twist limit\n" );

	// No gravity: this measures the twist about the cone axis, and a swinging
	// arm would move the axis while it was being measured.
	b3WorldId worldId = makeStepWorld( V( 0, 0, 0 ) );
	b3World* world = b3GetWorldFromId( worldId );

	b3BodyId anchor, arm;
	b3JointId jointId = ballArm( worldId, world, 1.0, &anchor, &arm );

	const b3a lower = (b3a)-2731; // -30 degrees
	const b3a upper = (b3a)2731;  // +30 degrees
	b3SphericalJoint_EnableTwistLimit( jointId, true );
	b3SphericalJoint_SetTwistLimits( jointId, lower, upper );

	checkInt( "lower twist limit round-trips", b3SphericalJoint_GetLowerTwistLimit( jointId ), lower );
	checkInt( "upper twist limit round-trips", b3SphericalJoint_GetUpperTwistLimit( jointId ), upper );

	// Twist about the cone axis, which this scene points along -y.
	for ( int i = 0; i < 900; ++i )
	{
		b3Body_ApplyTorque( arm, V( 0, -12.0, 0 ), true );
		b3World_Step( worldId, 4 );
	}
	validate( world );

	b3a settledUpper = b3SphericalJoint_GetTwistAngle( jointId );

	for ( int i = 0; i < 1200; ++i )
	{
		b3Body_ApplyTorque( arm, V( 0, 12.0, 0 ), true );
		b3World_Step( worldId, 4 );
	}
	validate( world );

	b3a settledLower = b3SphericalJoint_GetTwistAngle( jointId );

	printf( "  driven +: twist at %d brads (%.2f deg), limit is %d\n", (int)settledUpper,
			(double)settledUpper * 360.0 / 32768.0, (int)upper );
	printf( "  driven -: twist at %d brads (%.2f deg), limit is %d\n", (int)settledLower,
			(double)settledLower * 360.0 / 32768.0, (int)lower );

	expect( "the upper twist limit holds", (double)settledUpper, (double)upper, 384.0 );
	expect( "the lower twist limit holds", (double)settledLower, (double)lower, 384.0 );

	// The two branches are mirrors, so their overshoot should match in
	// magnitude. This is the twist's counterpart to the cone's symmetry check,
	// and the thing a sign error in one branch would break.
	double overshootUpper = (double)settledUpper - (double)upper;
	double overshootLower = (double)lower - (double)settledLower;
	expect( "and overshoot symmetrically", overshootLower, overshootUpper, 192.0 );

	b3DestroyWorld( worldId );
}

/// The 3-vector spring, which drives an *orientation* rather than an angle.
static void test_spherical_joint_spring( void )
{
	printf( "spherical joint rotational spring\n" );

	b3WorldId worldId = makeStepWorld( V( 0, 0, 0 ) );
	b3World* world = b3GetWorldFromId( worldId );

	b3BodyId anchor, arm;
	b3JointId jointId = ballArm( worldId, world, 1.0, &anchor, &arm );

	// A spring toward the identity: whatever pose the arm is released in, it
	// must come back to frame alignment. The target is the *relative* rotation
	// of frame B against frame A, so the identity means "frames aligned".
	b3SphericalJoint_EnableSpring( jointId, true );
	b3SphericalJoint_SetSpringHertz( jointId, b3fFromDouble( 2.0 ) );
	b3SphericalJoint_SetSpringDampingRatio( jointId, b3fFromDouble( 1.0 ) );
	b3SphericalJoint_SetTargetRotation( jointId, b3Quat_identity );

	// Released from a pose rotated about an axis that is not one of the frame
	// axes, so no single component of the 3-vector spring can carry it alone.
	//
	// The body is rotated **about the pivot**, not about its own centre: frame
	// B's origin is offset from the centre of mass, so spinning the body in
	// place drags the pivot almost a metre off the anchor and the first step
	// has to close that. That is a scene bug, not a solver one -- it is
	// box3d_rope's anchor mistake in another form -- and here it showed up as
	// B3_FIXED_DEBUG aborting on an overflow in the spring's effective mass.
	b3Quat start = b3MakeQuatFromAxisAngle( b3Normalize( V( 1, 1, 0.5 ) ), (b3a)5000 );
	b3Body_SetTransform( arm, b3RotateVector( start, V( 0, -1.0, 0 ) ), start );

	// Damped, and for a reason specific to this scene rather than out of habit.
	// The spring is critically damped in *orientation*, but the body's centre of
	// mass sits a metre from the pivot, so any residual linear motion is turned
	// into torque by the point constraint and fed straight back into the
	// orientation. With nothing removing that energy the pose circulates around
	// its target forever instead of settling on it.
	b3Body_SetLinearDamping( arm, b3fFromDouble( 1.5 ) );
	b3Body_SetAngularDamping( arm, b3fFromDouble( 1.5 ) );

	for ( int i = 0; i < 900; ++i )
	{
		b3World_Step( worldId, 4 );
	}
	validate( world );

	// Checked as the settled orientation, not the path: a critically damped
	// spring's route to its target is its own business.
	b3a cone = b3SphericalJoint_GetConeAngle( jointId );
	b3a twist = b3SphericalJoint_GetTwistAngle( jointId );

	printf( "  settled at cone %d brads, twist %d brads (both want 0)\n", (int)cone, (int)twist );

	expect( "the spring pulls the cone angle to its target", (double)cone, 0.0, 384.0 );
	expect( "and the twist angle", (double)twist, 0.0, 384.0 );

	// A non-identity target, so the test cannot pass by the spring simply
	// pulling everything to zero.
	b3Quat target = b3MakeQuatFromAxisAngle( V( 0, 0, 1 ), (b3a)3000 );
	b3SphericalJoint_SetTargetRotation( jointId, target );

	for ( int i = 0; i < 900; ++i )
	{
		b3World_Step( worldId, 4 );
	}
	validate( world );

	// A rotation about the frames' shared z is pure twist and no swing, which
	// is what makes this a check that the spring drives the *orientation*
	// rather than an angle magnitude.
	b3a twistToTarget = b3SphericalJoint_GetTwistAngle( jointId );
	printf( "  retargeted: twist %d brads (want 3000), cone %d (want 0)\n", (int)twistToTarget,
			(int)b3SphericalJoint_GetConeAngle( jointId ) );

	expect( "and follows a non-identity target", (double)twistToTarget, 3000.0, 384.0 );

	b3DestroyWorld( worldId );
}

/// The 3-vector motor, and the spherical bound on its torque.
static void test_spherical_joint_motor( void )
{
	printf( "spherical joint motor\n" );

	b3WorldId worldId = makeStepWorld( V( 0, 0, 0 ) );
	b3World* world = b3GetWorldFromId( worldId );

	b3BodyId anchor, arm;
	b3JointId jointId = ballArm( worldId, world, 1.0, &anchor, &arm );

	// Driven about an axis that is not a frame axis, so all three components of
	// the motor are doing work and the cone clamp is the thing under test
	// rather than a single interval.
	const double wx = 1.2, wy = -0.8, wz = 1.6;
	b3SphericalJoint_EnableMotor( jointId, true );
	b3SphericalJoint_SetMotorVelocity( jointId, V( wx, wy, wz ) );
	b3SphericalJoint_SetMaxMotorTorque( jointId, b3fFromDouble( 500.0 ) );

	for ( int i = 0; i < 600; ++i )
	{
		b3World_Step( worldId, 4 );
	}
	validate( world );

	b3Vec3 w = b3Body_GetAngularVelocity( arm );
	printf( "  motor at (%.2f %.2f %.2f): arm spinning at (%.4f %.4f %.4f)\n", wx, wy, wz, F( w.x ), F( w.y ),
			F( w.z ) );

	expect( "the motor reaches its target about x", F( w.x ), wx, 0.05 );
	expect( "about y", F( w.y ), wy, 0.05 );
	expect( "about z", F( w.z ), wz, 0.05 );

	b3Vec3 velocity = b3SphericalJoint_GetMotorVelocity( jointId );
	expect( "motor velocity round-trips", F( velocity.z ), wz, 1e-2 );

	// The bound.
	//
	// Two things have to be got right before a torque bound is even observable,
	// and the first draft of this test missed both.
	//
	// **It only shows while the motor is saturated.** With no load, any bound
	// above zero eventually reaches any target speed and then needs no torque at
	// all, so a steady-state sample reads exactly zero and says nothing. The
	// target is therefore set ten times higher than the bound can reach in the
	// window, and the window is short.
	//
	// **The bound cannot be arbitrarily small.** It is converted to an impulse
	// by multiplying by the sub-step, and an impulse is Q16: at 240 Hz a bound
	// below about 0.004 N-m lands under half a quantum and clamps the motor to
	// literally nothing. 0.0005 did exactly that -- the motor applied no impulse
	// and the arm still came back to speed, which is what sent this test looking
	// for a solver bug that was not there.
	const double bound = 0.05;
	b3SphericalJoint_SetMaxMotorTorque( jointId, b3fFromDouble( bound ) );
	b3SphericalJoint_SetMotorVelocity( jointId, V( 10.0 * wx, 10.0 * wy, 10.0 * wz ) );

	// Toggling the motor clears its accumulator, and that has to happen before
	// the velocities are reset: warm start applies the *stored* impulse at the
	// head of the next step, so the large one left from the previous phase put
	// the arm straight back to speed.
	b3SphericalJoint_EnableMotor( jointId, false );
	b3SphericalJoint_EnableMotor( jointId, true );

	// Both velocities, not just the angular one. The arm's centre of mass sits a
	// metre from the pivot, so zeroing the spin while leaving the orbit violates
	// the point constraint, and it re-establishes itself in a single step by
	// converting that linear motion straight back into spin -- 2 rad/s of it,
	// which read exactly like a motor ignoring its bound.
	b3Body_SetAngularVelocity( arm, V( 0, 0, 0 ) );
	b3Body_SetLinearVelocity( arm, V( 0, 0, 0 ) );

	double sumTorque = 0.0;
	const int samples = 120;
	for ( int s = 0; s < samples; ++s )
	{
		b3World_Step( worldId, 4 );
		b3Vec3 t = b3SphericalJoint_GetMotorTorque( jointId );
		sumTorque += sqrt( F( t.x ) * F( t.x ) + F( t.y ) * F( t.y ) + F( t.z ) * F( t.z ) );
	}
	validate( world );

	b3Vec3 stalled = b3Body_GetAngularVelocity( arm );
	double stalledSpeed =
		sqrt( F( stalled.x ) * F( stalled.x ) + F( stalled.y ) * F( stalled.y ) + F( stalled.z ) * F( stalled.z ) );
	double targetSpeed = 10.0 * sqrt( wx * wx + wy * wy + wz * wz );
	double meanTorque = sumTorque / samples;

	printf( "  bounded at %.3f N-m: reached %.4f rad/s of %.4f, mean torque %.5f\n", bound, stalledSpeed,
			targetSpeed, meanTorque );

	check( "a bounded motor stalls short of its target", stalledSpeed < targetSpeed * 0.5 );

	// The saturated torque must sit *at* the bound, not above it. This is the
	// check a component-wise clamp fails: three equal components each inside the
	// bound have a magnitude sqrt(3) times it, and the motor is driven about a
	// diagonal axis here precisely so all three are in play.
	//
	// Averaged over the window, for Stage 2's reason: the impulse falls between
	// two representable values and dithers, so a single sample is not a
	// measurement.
	check( "and its torque magnitude respects the bound", meanTorque <= bound * 1.05 );
	check( "and actually reaches it", meanTorque >= bound * 0.90 );

	b3DestroyWorld( worldId );
}

/// A ball joint between two rotation-locked bodies: every rotational mass is
/// singular at once, and a normal scene never goes here.
static void test_spherical_joint_fixed_rotation( void )
{
	printf( "spherical joint between rotation-locked bodies\n" );

	b3WorldId worldId = makeStepWorld( V( 0, -10, 0 ) );
	b3World* world = b3GetWorldFromId( worldId );

	b3BodyDef anchorDef = b3DefaultBodyDef();
	anchorDef.type = b3_staticBody;
	b3BodyId anchor = b3CreateBody( worldId, &anchorDef );

	b3BodyDef armDef = b3DefaultBodyDef();
	armDef.type = b3_dynamicBody;
	armDef.position = V( 0, -1.0, 0 );
	armDef.motionLocks.angularX = true;
	armDef.motionLocks.angularY = true;
	armDef.motionLocks.angularZ = true;
	armDef.enableSleep = false;
	b3BodyId arm = b3CreateBody( worldId, &armDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3Sphere ball = { V( 0, 0, 0 ), b3fFromDouble( 0.1 ) };
	b3CreateSphereShape( arm, &shapeDef, &ball );

	b3SphericalJointDef def = b3DefaultSphericalJointDef();
	def.base.bodyIdA = anchor;
	def.base.bodyIdB = arm;
	def.base.localFrameB.p = V( 0, 1.0, 0 );

	// Every rotational branch enabled at once, so all three singular masses --
	// rotationMass, swingMass and twistMass -- are reached in the same step.
	def.enableSpring = true;
	def.hertz = b3fFromDouble( 2.0 );
	def.dampingRatio = b3fFromDouble( 1.0 );
	def.enableMotor = true;
	def.maxMotorTorque = b3fFromDouble( 10.0 );
	def.motorVelocity = V( 1.0, 0, 0 );
	def.enableConeLimit = true;
	def.coneAngle = (b3a)4096;
	def.enableTwistLimit = true;
	def.lowerTwistAngle = (b3a)-2731;
	def.upperTwistAngle = (b3a)2731;

	b3JointId jointId = b3CreateSphericalJoint( worldId, &def );
	validate( world );

	for ( int i = 0; i < 600; ++i )
	{
		b3World_Step( worldId, 4 );
	}
	validate( world );

	b3Vec3 p = b3Body_GetPosition( arm );
	printf( "  pivot held at (%.5f %.5f %.5f) with rotation locked\n", F( p.x ), F( p.y ), F( p.z ) );

	// The point-to-point constraint is unaffected by fixedRotation and must
	// still hold; the rotational branches must simply do nothing rather than
	// divide by a singular matrix.
	expect( "the pivot still holds", F( p.y ), -1.0, 0.01 );
	expect( "and does not drift sideways", F( p.x ), 0.0, 0.01 );

	b3Vec3 torque = b3Joint_GetConstraintTorque( jointId );
	expect( "and no rotational impulse accumulated", F( torque.x ) + F( torque.y ) + F( torque.z ), 0.0, 1e-6 );

	b3DestroyWorld( worldId );
}

static void test_spherical_joint_sleep_and_accessors( void )
{
	printf( "spherical joint through sleep, and accessor round-trips\n" );

	b3WorldId worldId = makeStepWorld( V( 0, -10, 0 ) );
	b3World* world = b3GetWorldFromId( worldId );

	b3BodyId anchor, arm;
	b3JointId jointId = ballArm( worldId, world, 1.0, &anchor, &arm );

	b3Body_EnableSleep( arm, true );

	int steps = 0;
	while ( steps < 1200 && b3GetBodyFullId( world, arm )->setIndex == b3_awakeSet )
	{
		b3World_Step( worldId, 4 );
		steps++;
	}
	validate( world );
	check( "a ball-jointed body settles and sleeps", steps < 1200 );

	b3a asleep = b3SphericalJoint_GetConeAngle( jointId );

	b3Body_SetAwake( arm, true );
	validate( world );
	checkInt( "waking returns the joint to the graph", b3GetJointFullId( world, jointId )->colorIndex,
			  B3_OVERFLOW_INDEX );

	// The step after waking is where a mis-copied 444-byte sim shows up: the
	// frames, the two cached axes, twistScale and the six accumulators all have
	// to be the ones it slept with.
	b3World_Step( worldId, 4 );
	validate( world );
	expect( "and the ball joint resumes without a jolt", (double)b3SphericalJoint_GetConeAngle( jointId ),
			(double)asleep, 128.0 );

	// Accessors.
	b3SphericalJoint_SetSpringHertz( jointId, b3fFromDouble( 4.5 ) );
	expect( "spring hertz round-trips", F( b3SphericalJoint_GetSpringHertz( jointId ) ), 4.5, 1e-2 );

	b3SphericalJoint_SetSpringDampingRatio( jointId, b3fFromDouble( 1.75 ) );
	expect( "spring damping round-trips", F( b3SphericalJoint_GetSpringDampingRatio( jointId ) ), 1.75, 1e-2 );

	b3Quat target = b3MakeQuatFromAxisAngle( V( 0, 0, 1 ), (b3a)1234 );
	b3SphericalJoint_SetTargetRotation( jointId, target );
	b3Quat gotTarget = b3SphericalJoint_GetTargetRotation( jointId );
	expect( "target rotation round-trips", (double)b3Raw( gotTarget.v.z ), (double)b3Raw( target.v.z ), 2.0 );

	b3SphericalJoint_SetMaxMotorTorque( jointId, b3fFromDouble( 7.5 ) );
	expect( "max motor torque round-trips", F( b3SphericalJoint_GetMaxMotorTorque( jointId ) ), 7.5, 1e-2 );

	b3SphericalJoint_EnableSpring( jointId, true );
	check( "spring reports enabled", b3SphericalJoint_IsSpringEnabled( jointId ) );
	b3SphericalJoint_EnableConeLimit( jointId, true );
	check( "cone limit reports enabled", b3SphericalJoint_IsConeLimitEnabled( jointId ) );
	b3SphericalJoint_EnableTwistLimit( jointId, true );
	check( "twist limit reports enabled", b3SphericalJoint_IsTwistLimitEnabled( jointId ) );
	b3SphericalJoint_EnableMotor( jointId, true );
	check( "motor reports enabled", b3SphericalJoint_IsMotorEnabled( jointId ) );

	// The base accessors still work on a typed joint -- the union sits after
	// them, so a wrong offset would corrupt one or the other.
	b3Joint_SetUserData( jointId, (void*)(intptr_t)0xBA11 );
	checkInt( "base user data still round-trips", (intptr_t)b3Joint_GetUserData( jointId ), 0xBA11 );
	checkInt( "and body A", b3Joint_GetBodyA( jointId ).index1, anchor.index1 );

	b3DestroyJoint( jointId, true );
	validate( world );
	check( "a destroyed spherical joint is not valid", b3Joint_IsValid( jointId ) == false );
	checkInt( "and the body no longer counts it", b3GetBodyFullId( world, arm )->jointCount, 0 );

	b3DestroyWorld( worldId );
}

/// The ragdoll box3d_ragdoll builds, on the host where it can be inspected.
///
/// Five limbs on ball joints to a torso, every one with a cone and a twist
/// limit, dropped onto a floor. This is the scene the device example runs, and
/// it is here because the example came apart and a screenshot cannot say why.
///
/// Run at two densities, and the light one is the point. b3InvertInertia caps a
/// single body's inverse inertia at Q7.24's ceiling of 128, so no *body* can
/// overflow -- but a joint divides by the sum of two, and at density 1 this
/// figure's head and torso sum to 128.197. That wrapped to -128 in b3AddMWMW
/// and inverted the constraint, which is what tore the device example apart.
/// See b3AxisInertiaSumWide.
static void ragdollScene( double density )
{
	printf( "spherical joint ragdoll (density %g)\n", density );

	b3WorldDef wd = b3DefaultWorldDef();
	wd.capacity.staticBodyCount = 4;
	wd.capacity.dynamicBodyCount = 8;
	wd.capacity.staticShapeCount = 4;
	wd.capacity.dynamicShapeCount = 8;
	wd.capacity.contactCount = 64;
	wd.capacity.jointCount = 8;
	wd.gravity = V( 0, -9.8, 0 );
	b3WorldId worldId = b3CreateWorld( &wd );
	b3World* world = b3GetWorldFromId( worldId );

	const double startY = 4.2;
	const double torsoHalfH = 0.65;
	const double limbOffset[6][3] = {
		{ 0.0, 0.0, 0.0 },   { 0.0, 0.95, 0.0 },   { -0.72, 0.25, 0.0 },
		{ 0.72, 0.25, 0.0 }, { -0.26, -1.2, 0.0 }, { 0.26, -1.2, 0.0 },
	};
	const double limbHalf[6][3] = {
		{ 0.45, 0.65, 0.25 }, { 0.28, 0.28, 0.28 }, { 0.16, 0.55, 0.16 },
		{ 0.16, 0.55, 0.16 }, { 0.16, 0.55, 0.16 }, { 0.16, 0.55, 0.16 },
	};

	// The hulls live here, not inside the loops below, because b3CreateHullShape
	// stores the *caller's* pointer -- hulls are baked into ROM on this port, so
	// a shape never owns one (shape.c:155). A hull declared inside the loop dies
	// at the end of the iteration and the shape is left pointing into reclaimed
	// stack.
	//
	// That is worth the comment because of how it presents: the scene falls
	// correctly, and only diverges at the frame the hull is first *read*, which
	// is first contact. And because the dead stack is reused differently by
	// different builds, it diverged between MODE=debug and MODE=device -- which
	// reads as a fixed-point mode bug and is nothing of the kind.
	b3BoxHull floorHull = b3MakeBoxHull( b3fFromDouble( 6.0 ), b3fFromDouble( 0.25 ), b3fFromDouble( 6.0 ) );
	b3BoxHull limbHull[6];

	{
		b3BodyDef bd = b3DefaultBodyDef();
		bd.type = b3_staticBody;
		bd.position = V( 0, -0.25, 0 );
		b3BodyId floorBody = b3CreateBody( worldId, &bd );
		b3ShapeDef sd = b3DefaultShapeDef();
		b3CreateHullShape( floorBody, &sd, &floorHull.base );
	}

	b3BodyId limb[6];
	for ( int i = 0; i < 6; ++i )
	{
		b3BodyDef bd = b3DefaultBodyDef();
		bd.type = b3_dynamicBody;
		bd.position = V( limbOffset[i][0], startY + limbOffset[i][1], limbOffset[i][2] );
		bd.linearDamping = b3fFromDouble( 0.4 );
		bd.angularDamping = b3fFromDouble( 0.8 );
		limb[i] = b3CreateBody( worldId, &bd );

		b3ShapeDef sd = b3DefaultShapeDef();
		sd.density = b3fFromDouble( density );
		limbHull[i] = b3MakeBoxHull( b3fFromDouble( limbHalf[i][0] ), b3fFromDouble( limbHalf[i][1] ),
									 b3fFromDouble( limbHalf[i][2] ) );
		b3CreateHullShape( limb[i], &sd, &limbHull[i].base );
	}

	const double coneDeg[6] = { 0, 25, 70, 70, 45, 45 };
	const double twistDeg[6] = { 0, 20, 45, 45, 25, 25 };

	b3JointId joints[6];
	for ( int i = 1; i < 6; ++i )
	{
		double socketX = limbOffset[i][0] * 0.5;
		double socketY = ( i == 1 ) ? torsoHalfH : ( ( i >= 4 ) ? -torsoHalfH : 0.35 );

		b3SphericalJointDef def = b3DefaultSphericalJointDef();
		def.base.bodyIdA = limb[0];
		def.base.bodyIdB = limb[i];
		def.base.localFrameA.p = V( socketX, socketY, 0 );
		def.base.localFrameB.p = V( socketX - limbOffset[i][0], socketY - limbOffset[i][1], 0 );

		b3a aim = ( i == 1 ) ? (b3a)B3_BRAD_HALF_PI : (b3a)( -B3_BRAD_HALF_PI );
		b3Quat toAxis = b3MakeQuatFromAxisAngle( V( 1, 0, 0 ), aim );
		def.base.localFrameA.q = toAxis;
		def.base.localFrameB.q = toAxis;

		def.enableConeLimit = true;
		def.coneAngle = (b3a)( coneDeg[i] * ( 32768.0 / 360.0 ) );
		def.enableTwistLimit = true;
		def.lowerTwistAngle = (b3a)( -twistDeg[i] * ( 32768.0 / 360.0 ) );
		def.upperTwistAngle = (b3a)( twistDeg[i] * ( 32768.0 / 360.0 ) );

		joints[i] = b3CreateSphericalJoint( worldId, &def );
	}
	validate( world );

	// The invariant that matters: each limb's socket must stay on the torso's.
	// Anything that leaves is the point constraint failing, which is the figure
	// coming apart.
	double worstGap = 0.0;
	int worstStep = -1;
	for ( int step = 0; step < 900; ++step )
	{
		b3World_Step( worldId, 4 );
		for ( int i = 1; i < 6; ++i )
		{
			double socketX = limbOffset[i][0] * 0.5;
			double socketY = ( i == 1 ) ? torsoHalfH : ( ( i >= 4 ) ? -torsoHalfH : 0.35 );

			b3Vec3 a = b3Body_GetWorldPoint( limb[0], V( socketX, socketY, 0 ) );
			b3Vec3 b = b3Body_GetWorldPoint(
				limb[i], V( socketX - limbOffset[i][0], socketY - limbOffset[i][1], 0 ) );

			double dx = F( a.x ) - F( b.x );
			double dy = F( a.y ) - F( b.y );
			double dz = F( a.z ) - F( b.z );
			double gap = sqrt( dx * dx + dy * dy + dz * dz );
			if ( gap > worstGap )
			{
				worstGap = gap;
				worstStep = step;
			}
		}
	}
	validate( world );

	b3Vec3 torso = b3Body_GetPosition( limb[0] );
	printf( "  worst socket gap %.5f at step %d; torso rests at (%.3f %.3f %.3f)\n", worstGap, worstStep,
			F( torso.x ), F( torso.y ), F( torso.z ) );

	// The limits, which the socket gap says nothing about. The device example
	// reported an arm twisted 171 degrees against a 45 degree limit, and this
	// is where that either reproduces or does not.
	for ( int i = 1; i < 6; ++i )
	{
		b3a cone = b3SphericalJoint_GetConeAngle( joints[i] );
		b3a twist = b3SphericalJoint_GetTwistAngle( joints[i] );
		printf( "    limb %d: cone %5.1f deg (limit %4.1f), twist %6.1f deg (limit %4.1f)\n", i,
				(double)cone * 360.0 / 32768.0, coneDeg[i], (double)twist * 360.0 / 32768.0, twistDeg[i] );

		// Two degrees of slack over the limit, which is the soft constraint's
		// give under a landing impact -- the same order as the cone and twist
		// tests measure in isolation.
		char label[80];
		snprintf( label, sizeof( label ), "limb %d stays inside its cone", i );
		check( label, (double)cone * 360.0 / 32768.0 <= coneDeg[i] + 2.0 );
		snprintf( label, sizeof( label ), "limb %d stays inside its twist range", i );
		check( label, fabs( (double)twist * 360.0 / 32768.0 ) <= twistDeg[i] + 2.0 );
	}

	check( "the ragdoll stays assembled", worstGap < 0.10 );
	check( "and lands on the floor rather than through it", F( torso.y ) > 0.0 );

	b3DestroyWorld( worldId );
}

// =========================================================================
// Stage 5: the weld joint
// =========================================================================

/// Build a box on a weld joint to a static anchor, offset along +x.
///
/// The anchor is at the origin and the box hangs one metre out along x, so
/// gravity puts the weld under both a *force* and a *torque* -- which is what
/// separates a weld from a ball joint. A spherical joint in this pose would let
/// the box swing down; a weld holds it out level.
static b3JointId weldArm( b3WorldId worldId, double armLength, double linearHertz, double angularHertz,
						  b3BoxHull* hull, b3BodyId* anchorOut, b3BodyId* armOut )
{
	b3BodyDef anchorDef = b3DefaultBodyDef();
	anchorDef.type = b3_staticBody;
	anchorDef.position = V( 0, 0, 0 );
	b3BodyId anchor = b3CreateBody( worldId, &anchorDef );

	b3BodyDef armDef = b3DefaultBodyDef();
	armDef.type = b3_dynamicBody;
	armDef.position = V( armLength, 0, 0 );
	armDef.enableSleep = false;
	b3BodyId arm = b3CreateBody( worldId, &armDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3CreateHullShape( arm, &shapeDef, &hull->base );

	b3WeldJointDef def = b3DefaultWeldJointDef();
	def.base.bodyIdA = anchor;
	def.base.bodyIdB = arm;

	// Frame A at the anchor's origin, frame B at the same world point expressed
	// in the arm's local space.
	def.base.localFrameB.p = V( -armLength, 0, 0 );
	def.linearHertz = b3fFromDouble( linearHertz );
	def.linearDampingRatio = b3fFromDouble( 2.0 );
	def.angularHertz = b3fFromDouble( angularHertz );
	def.angularDampingRatio = b3fFromDouble( 2.0 );

	*anchorOut = anchor;
	*armOut = arm;
	return b3CreateWeldJoint( worldId, &def );
}

/// A rigid weld holds position *and* orientation, and does not creep.
///
/// The counterpart of test_revolute_joint_holds and test_spherical_joint_holds,
/// and it checks the thing those two cannot: that the box is still *level*
/// after ten seconds of gravity pulling on a one-metre lever arm.
static void test_weld_joint_holds( void )
{
	printf( "weld joint holds its pose\n" );

	b3WorldId worldId = makeStepWorld( V( 0, -10, 0 ) );
	b3World* world = b3GetWorldFromId( worldId );

	b3BoxHull hull = b3MakeBoxHull( b3fFromDouble( 0.2 ), b3fFromDouble( 0.2 ), b3fFromDouble( 0.2 ) );
	b3BodyId anchor, arm;
	b3JointId jointId = weldArm( worldId, 1.0, 0.0, 0.0, &hull, &anchor, &arm );
	validate( world );

	double posErr1 = 0.0, tiltDeg1 = 0.0;
	for ( int i = 0; i < 600; ++i )
	{
		b3World_Step( worldId, 4 );
		if ( i == 59 )
		{
			b3Vec3 p = b3Body_GetPosition( arm );
			posErr1 = sqrt( ( F( p.x ) - 1.0 ) * ( F( p.x ) - 1.0 ) + F( p.y ) * F( p.y ) + F( p.z ) * F( p.z ) );

			// The tilt is how far the box's own x axis has left horizontal.
			b3Vec3 axis = b3RotateVector( b3Body_GetRotation( arm ), V( 1, 0, 0 ) );
			tiltDeg1 = acos( fmin( 1.0, fmax( -1.0, F( axis.x ) ) ) ) * 180.0 / 3.14159265358979;
		}
	}
	validate( world );

	b3Vec3 p = b3Body_GetPosition( arm );
	double posErr10 = sqrt( ( F( p.x ) - 1.0 ) * ( F( p.x ) - 1.0 ) + F( p.y ) * F( p.y ) + F( p.z ) * F( p.z ) );
	b3Vec3 axis = b3RotateVector( b3Body_GetRotation( arm ), V( 1, 0, 0 ) );
	double tiltDeg10 = acos( fmin( 1.0, fmax( -1.0, F( axis.x ) ) ) ) * 180.0 / 3.14159265358979;

	printf( "  position error at 1 s %.5f, at 10 s %.5f\n", posErr1, posErr10 );
	printf( "  tilt off level at 1 s %.3f deg, at 10 s %.3f deg\n", tiltDeg1, tiltDeg10 );

	check( "the weld holds its anchor", posErr10 < 0.02 );
	check( "and holds the box level, which a ball joint would not", tiltDeg10 < 2.0 );

	// The creep check: a constraint that leaks a little every step reads as
	// correct at one second and is visibly wrong at ten. Stage 2's rule.
	check( "and does not creep between 1 s and 10 s", posErr10 < posErr1 + 0.01 );
	check( "nor tilt further", tiltDeg10 < tiltDeg1 + 1.0 );

	// The reaction force must carry the box's weight: m*g, straight up.
	b3Vec3 force = b3Joint_GetConstraintForce( jointId );
	double weight = F( b3Body_GetMass( arm ) ) * 10.0;
	printf( "  reaction force y %.4f N against a weight of %.4f N\n", F( force.y ), weight );
	expect( "and reports the weight it is carrying", fabs( F( force.y ) ), weight, 0.15 * weight );

	b3DestroyWorld( worldId );
}

/// A softened weld is a spring, and deflects by the amount a spring should.
///
/// Stage 2 measured the distance joint's spring deflection; this is the same
/// closed form on a joint that carries a whole 3-vector. At equilibrium the
/// spring force balances gravity, so `k * x = m * g` with `k = m * omega^2`,
/// and the mass cancels: **x = g / (2*pi*f)^2**, independent of the box.
///
/// That independence is the point -- it tests the softness coefficients and the
/// effective mass, not anything the scene chose.
static void test_weld_joint_linear_spring( void )
{
	printf( "weld joint as a linear spring\n" );

	const double hertz = 1.5;
	const double g = 10.0;
	const double omega = 2.0 * 3.14159265358979 * hertz;
	const double predicted = g / ( omega * omega );

	b3WorldId worldId = makeStepWorld( V( 0, -g, 0 ) );
	b3World* world = b3GetWorldFromId( worldId );

	b3BoxHull hull = b3MakeBoxHull( b3fFromDouble( 0.2 ), b3fFromDouble( 0.2 ), b3fFromDouble( 0.2 ) );
	b3BodyId anchor, arm;

	// The arm is directly below the anchor rather than out to the side, so the
	// deflection is pure sag with no torque in it -- the closed form above is
	// about the linear spring alone. The angular half stays rigid.
	b3BodyDef anchorDef = b3DefaultBodyDef();
	anchorDef.type = b3_staticBody;
	anchorDef.position = V( 0, 0, 0 );
	anchor = b3CreateBody( worldId, &anchorDef );

	b3BodyDef armDef = b3DefaultBodyDef();
	armDef.type = b3_dynamicBody;
	armDef.position = V( 0, -1.0, 0 );
	armDef.enableSleep = false;
	arm = b3CreateBody( worldId, &armDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3CreateHullShape( arm, &shapeDef, &hull.base );

	b3WeldJointDef def = b3DefaultWeldJointDef();
	def.base.bodyIdA = anchor;
	def.base.bodyIdB = arm;
	def.base.localFrameB.p = V( 0, 1.0, 0 );
	def.linearHertz = b3fFromDouble( hertz );

	// Critically damped, so the sag settles rather than oscillating -- the
	// closed form is the equilibrium, not the first swing.
	def.linearDampingRatio = b3fFromDouble( 1.0 );

	b3JointId jointId = b3CreateWeldJoint( worldId, &def );
	validate( world );

	for ( int i = 0; i < 900; ++i )
	{
		b3World_Step( worldId, 4 );
	}
	validate( world );

	b3Vec3 p = b3Body_GetPosition( arm );
	double sag = -1.0 - F( p.y );

	printf( "  sag at %.1f Hz: %.5f m, g/(2*pi*f)^2 predicts %.5f (ratio %.4f)\n", hertz, sag, predicted,
			sag / predicted );

	check( "a softened weld sags rather than holding rigid", sag > 0.5 * predicted );
	expect( "and by the amount the closed form predicts", sag, predicted, 0.20 * predicted );
	check( "the joint survives", b3Joint_IsValid( jointId ) );

	b3DestroyWorld( worldId );
}

/// Both bodies rotation-locked: the angular branch is skipped, the linear one
/// still runs. The weld's counterpart of test_revolute_joint_fixed_rotation.
static void test_weld_joint_fixed_rotation( void )
{
	printf( "weld joint between rotation-locked bodies\n" );

	b3WorldId worldId = makeStepWorld( V( 0, -10, 0 ) );
	b3World* world = b3GetWorldFromId( worldId );

	b3BodyDef anchorDef = b3DefaultBodyDef();
	anchorDef.type = b3_staticBody;
	anchorDef.position = V( 0, 0, 0 );
	b3BodyId anchor = b3CreateBody( worldId, &anchorDef );

	b3BodyDef armDef = b3DefaultBodyDef();
	armDef.type = b3_dynamicBody;
	armDef.position = V( 1.0, 0, 0 );
	armDef.motionLocks.angularX = true;
	armDef.motionLocks.angularY = true;
	armDef.motionLocks.angularZ = true;
	armDef.enableSleep = false;
	b3BodyId arm = b3CreateBody( worldId, &armDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3Sphere ball = { V( 0, 0, 0 ), b3fFromDouble( 0.1 ) };
	b3CreateSphereShape( arm, &shapeDef, &ball );

	b3WeldJointDef def = b3DefaultWeldJointDef();
	def.base.bodyIdA = anchor;
	def.base.bodyIdB = arm;
	def.base.localFrameB.p = V( -1.0, 0, 0 );

	b3JointId jointId = b3CreateWeldJoint( worldId, &def );
	validate( world );

	for ( int i = 0; i < 300; ++i )
	{
		b3World_Step( worldId, 4 );
	}
	validate( world );

	b3Vec3 p = b3Body_GetPosition( arm );
	printf( "  held at (%.5f %.5f %.5f) with rotation locked\n", F( p.x ), F( p.y ), F( p.z ) );

	check( "the joint survives a singular rotational mass", b3Joint_IsValid( jointId ) );
	expect( "and the linear half still holds, x", F( p.x ), 1.0, 0.05 );
	expect( "and y", F( p.y ), 0.0, 0.05 );

	b3DestroyWorld( worldId );
}

static void test_weld_joint_sleep_and_accessors( void )
{
	printf( "weld joint through sleep, and accessor round-trips\n" );

	b3WorldId worldId = makeStepWorld( V( 0, -10, 0 ) );
	b3World* world = b3GetWorldFromId( worldId );

	b3BoxHull hull = b3MakeBoxHull( b3fFromDouble( 0.2 ), b3fFromDouble( 0.2 ), b3fFromDouble( 0.2 ) );
	b3BodyId anchor, arm;
	b3JointId jointId = weldArm( worldId, 1.0, 0.0, 0.0, &hull, &anchor, &arm );

	b3Body_EnableSleep( arm, true );

	int steps = 0;
	while ( steps < 1200 && b3GetBodyFullId( world, arm )->setIndex == b3_awakeSet )
	{
		b3World_Step( worldId, 4 );
		steps++;
	}
	validate( world );

	printf( "  slept after %d steps\n", steps );
	check( "a welded body sleeps", b3GetBodyFullId( world, arm )->setIndex != b3_awakeSet );
	check( "and its joint is still valid", b3Joint_IsValid( jointId ) );

	b3Body_SetAwake( arm, true );
	b3World_Step( worldId, 4 );
	validate( world );
	check( "and wakes again", b3GetBodyFullId( world, arm )->setIndex == b3_awakeSet );

	checkInt( "type round-trips", b3Joint_GetType( jointId ), b3_weldJoint );

	b3WeldJoint_SetLinearHertz( jointId, b3fFromDouble( 4.0 ) );
	expect( "linear hertz round-trips", F( b3WeldJoint_GetLinearHertz( jointId ) ), 4.0, 0.01 );
	b3WeldJoint_SetLinearDampingRatio( jointId, b3fFromDouble( 1.5 ) );
	expect( "linear damping round-trips", F( b3WeldJoint_GetLinearDampingRatio( jointId ) ), 1.5, 0.01 );
	b3WeldJoint_SetAngularHertz( jointId, b3fFromDouble( 6.0 ) );
	expect( "angular hertz round-trips", F( b3WeldJoint_GetAngularHertz( jointId ) ), 6.0, 0.01 );
	b3WeldJoint_SetAngularDampingRatio( jointId, b3fFromDouble( 0.5 ) );
	expect( "angular damping round-trips", F( b3WeldJoint_GetAngularDampingRatio( jointId ) ), 0.5, 0.01 );

	b3DestroyJoint( jointId, true );
	validate( world );
	check( "a destroyed weld joint is not valid", b3Joint_IsValid( jointId ) == false );
	checkInt( "and the body no longer counts it", b3GetBodyFullId( world, arm )->jointCount, 0 );

	b3DestroyWorld( worldId );
}

// =========================================================================
// Stage 5: the motor joint
// =========================================================================

/// A linear velocity drive reaches its target, and its force is bounded.
///
/// Two measurements in one scene, because they are the same branch seen from
/// two sides. With a generous bound the body reaches the commanded velocity;
/// with a bound below what gravity demands the drive **saturates** and the body
/// falls anyway -- which is the only condition under which a force bound is
/// observable at all. Stage 4 learned that the hard way on the spherical motor:
/// an unsaturated drive reaches its target and then applies zero.
static void test_motor_joint_linear_velocity( void )
{
	printf( "motor joint linear velocity drive\n" );

	const double g = 10.0;
	const double target = 2.0;

	// -- generous bound: the target is reached ---------------------------
	{
		b3WorldId worldId = makeStepWorld( V( 0, -g, 0 ) );
		b3World* world = b3GetWorldFromId( worldId );

		b3BodyDef anchorDef = b3DefaultBodyDef();
		anchorDef.type = b3_staticBody;
		anchorDef.position = V( 0, 0, 0 );
		b3BodyId anchor = b3CreateBody( worldId, &anchorDef );

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = V( 0, 0, 0 );
		bodyDef.enableSleep = false;
		b3BodyId body = b3CreateBody( worldId, &bodyDef );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		b3Sphere ball = { V( 0, 0, 0 ), b3fFromDouble( 0.2 ) };
		b3CreateSphereShape( body, &shapeDef, &ball );

		b3MotorJointDef def = b3DefaultMotorJointDef();
		def.base.bodyIdA = anchor;
		def.base.bodyIdB = body;
		def.linearVelocity = V( target, 0, 0 );

		// The bound must cover the weight, not just the acceleration. This is a
		// **3-vector** drive under a single magnitude bound, so holding the
		// 33 kg sphere up against gravity spends the same budget that moving it
		// along x does -- and a bound below the 335 N weight leaves nothing for
		// the x component, which reads as "the drive cannot reach its target"
		// when the real answer is that it is busy falling.
		def.maxVelocityForce = b3fFromDouble( 2000.0 );

		b3JointId jointId = b3CreateMotorJoint( worldId, &def );
		validate( world );

		for ( int i = 0; i < 300; ++i )
		{
			b3World_Step( worldId, 4 );
		}
		validate( world );

		b3Vec3 v = b3Body_GetLinearVelocity( body );
		printf( "  target %.1f m/s: reached %.5f m/s\n", target, F( v.x ) );
		expect( "the drive reaches its commanded velocity", F( v.x ), target, 0.05 );
		check( "the joint survives", b3Joint_IsValid( jointId ) );

		b3DestroyWorld( worldId );
	}

	// -- tight bound: the drive saturates and reports its ceiling ---------
	{
		const double bound = 0.5;

		b3WorldId worldId = makeStepWorld( V( 0, -g, 0 ) );
		b3World* world = b3GetWorldFromId( worldId );

		b3BodyDef anchorDef = b3DefaultBodyDef();
		anchorDef.type = b3_staticBody;
		anchorDef.position = V( 0, 0, 0 );
		b3BodyId anchor = b3CreateBody( worldId, &anchorDef );

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = V( 0, 0, 0 );
		bodyDef.enableSleep = false;
		b3BodyId body = b3CreateBody( worldId, &bodyDef );

		// Heavy enough that 0.5 N cannot hold it: weight is about 33 N.
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		b3Sphere ball = { V( 0, 0, 0 ), b3fFromDouble( 0.2 ) };
		b3CreateSphereShape( body, &shapeDef, &ball );

		b3MotorJointDef def = b3DefaultMotorJointDef();
		def.base.bodyIdA = anchor;
		def.base.bodyIdB = body;
		def.linearVelocity = V( 0, 0, 0 );
		def.maxVelocityForce = b3fFromDouble( bound );

		b3JointId jointId = b3CreateMotorJoint( worldId, &def );
		validate( world );

		for ( int i = 0; i < 300; ++i )
		{
			b3World_Step( worldId, 4 );
		}
		validate( world );

		b3Vec3 v = b3Body_GetLinearVelocity( body );
		b3Vec3 force = b3Joint_GetConstraintForce( jointId );
		double magnitude = sqrt( F( force.x ) * F( force.x ) + F( force.y ) * F( force.y ) + F( force.z ) * F( force.z ) );

		printf( "  bounded at %.2f N: applied %.5f N, body still falling at %.3f m/s\n", bound, magnitude, F( v.y ) );

		check( "a saturated drive cannot hold the body", F( v.y ) < -1.0 );
		expect( "and applies exactly its bound", magnitude, bound, 0.10 * bound );

		b3DestroyWorld( worldId );
	}
}

/// An angular velocity drive, and the magnitude bound on a diagonal.
///
/// The diagonal is the case that separates a sphere bound from a box bound:
/// three equal components each inside a per-axis limit have magnitude sqrt(3)
/// times it. Stage 4 proved b3ClampImp3 does the right thing in isolation; this
/// proves the motor joint actually calls it that way.
static void test_motor_joint_angular_velocity( void )
{
	printf( "motor joint angular velocity drive\n" );

	// No gravity: this is about the drive, and a falling body adds nothing.
	b3WorldId worldId = makeStepWorld( V( 0, 0, 0 ) );
	b3World* world = b3GetWorldFromId( worldId );

	b3BodyDef anchorDef = b3DefaultBodyDef();
	anchorDef.type = b3_staticBody;
	anchorDef.position = V( 0, 0, 0 );
	b3BodyId anchor = b3CreateBody( worldId, &anchorDef );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = V( 0, 0, 0 );
	bodyDef.enableSleep = false;
	b3BodyId body = b3CreateBody( worldId, &bodyDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3Sphere ball = { V( 0, 0, 0 ), b3fFromDouble( 0.3 ) };
	b3CreateSphereShape( body, &shapeDef, &ball );

	// Equal on all three axes, so the commanded velocity is a diagonal.
	const double each = 1.0;

	b3MotorJointDef def = b3DefaultMotorJointDef();
	def.base.bodyIdA = anchor;
	def.base.bodyIdB = body;
	def.angularVelocity = V( each, each, each );
	def.maxVelocityTorque = b3fFromDouble( 20.0 );

	b3JointId jointId = b3CreateMotorJoint( worldId, &def );
	validate( world );

	for ( int i = 0; i < 300; ++i )
	{
		b3World_Step( worldId, 4 );
	}
	validate( world );

	b3Vec3 w = b3Body_GetAngularVelocity( body );
	printf( "  target (%.1f %.1f %.1f) rad/s: reached (%.4f %.4f %.4f)\n", each, each, each, F( w.x ), F( w.y ),
			F( w.z ) );

	expect( "the drive reaches its target on x", F( w.x ), each, 0.05 );
	expect( "on y", F( w.y ), each, 0.05 );
	expect( "on z", F( w.z ), each, 0.05 );
	check( "the joint survives", b3Joint_IsValid( jointId ) );

	b3DestroyWorld( worldId );

	// -- the magnitude bound, on a diagonal -------------------------------
	{
		const double bound = 0.5;

		worldId = makeStepWorld( V( 0, 0, 0 ) );
		world = b3GetWorldFromId( worldId );

		anchorDef = b3DefaultBodyDef();
		anchorDef.type = b3_staticBody;
		anchorDef.position = V( 0, 0, 0 );
		anchor = b3CreateBody( worldId, &anchorDef );

		bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = V( 0, 0, 0 );
		bodyDef.enableSleep = false;
		body = b3CreateBody( worldId, &bodyDef );

		b3Sphere big = { V( 0, 0, 0 ), b3fFromDouble( 0.5 ) };
		b3CreateSphereShape( body, &shapeDef, &big );

		// A target far beyond what the bound can reach in the time given, so
		// the drive stays saturated and the torque is observable.
		b3MotorJointDef bdef = b3DefaultMotorJointDef();
		bdef.base.bodyIdA = anchor;
		bdef.base.bodyIdB = body;
		bdef.angularVelocity = V( 20.0, 20.0, 20.0 );
		bdef.maxVelocityTorque = b3fFromDouble( bound );

		b3JointId boundedId = b3CreateMotorJoint( worldId, &bdef );
		validate( world );

		for ( int i = 0; i < 120; ++i )
		{
			b3World_Step( worldId, 4 );
		}
		validate( world );

		b3Vec3 torque = b3Joint_GetConstraintTorque( boundedId );
		double magnitude =
			sqrt( F( torque.x ) * F( torque.x ) + F( torque.y ) * F( torque.y ) + F( torque.z ) * F( torque.z ) );
		double largest = fmax( fabs( F( torque.x ) ), fmax( fabs( F( torque.y ) ), fabs( F( torque.z ) ) ) );

		printf( "  torque bounded at %.2f N-m: magnitude %.5f, largest component %.5f\n", bound, magnitude, largest );

		expect( "a saturated diagonal drive spends exactly its bound", magnitude, bound, 0.10 * bound );

		// The bound is a sphere, not a box: a per-component clamp would have
		// let this reach sqrt(3) * bound in magnitude.
		check( "and the bound is on the magnitude, not per axis", magnitude < 1.2 * bound );

		b3DestroyWorld( worldId );
	}
}

/// The motor joint as a moving platform: a spring drive that carries a load.
///
/// This is the joint's actual use, and the thing the device example is built
/// on. A dynamic platform is spring-driven toward a static anchor's frame while
/// a free box rests on it; the platform must hold station under the load, and
/// the box must be carried rather than launched or dropped through.
static void test_motor_joint_platform( void )
{
	printf( "motor joint as a loaded platform\n" );

	const double g = 10.0;
	b3WorldId worldId = makeStepWorld( V( 0, -g, 0 ) );
	b3World* world = b3GetWorldFromId( worldId );

	// Hulls declared outside everything that follows: b3CreateHullShape stores
	// the caller's pointer, which cost Stage 4 a wrong diagnosis.
	b3BoxHull platformHull = b3MakeBoxHull( b3fFromDouble( 0.5 ), b3fFromDouble( 0.05 ), b3fFromDouble( 0.5 ) );
	b3BoxHull cargoHull = b3MakeBoxHull( b3fFromDouble( 0.2 ), b3fFromDouble( 0.2 ), b3fFromDouble( 0.2 ) );

	b3BodyDef anchorDef = b3DefaultBodyDef();
	anchorDef.type = b3_staticBody;
	anchorDef.position = V( 0, 2.0, 0 );
	b3BodyId anchor = b3CreateBody( worldId, &anchorDef );

	b3BodyDef platformDef = b3DefaultBodyDef();
	platformDef.type = b3_dynamicBody;
	platformDef.position = V( 0, 2.0, 0 );
	platformDef.enableSleep = false;
	b3BodyId platform = b3CreateBody( worldId, &platformDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3CreateHullShape( platform, &shapeDef, &platformHull.base );

	b3BodyDef cargoDef = b3DefaultBodyDef();
	cargoDef.type = b3_dynamicBody;
	cargoDef.position = V( 0, 2.5, 0 );
	cargoDef.enableSleep = false;
	b3BodyId cargo = b3CreateBody( worldId, &cargoDef );
	b3CreateHullShape( cargo, &shapeDef, &cargoHull.base );

	b3MotorJointDef def = b3DefaultMotorJointDef();
	def.base.bodyIdA = anchor;
	def.base.bodyIdB = platform;

	// The target pose is the two joint frames coinciding, which they do at
	// build time -- so the spring's whole job is to resist the sag that gravity
	// and the cargo's weight introduce.
	def.linearHertz = b3fFromDouble( 5.0 );
	def.linearDampingRatio = b3fFromDouble( 2.0 );
	def.angularHertz = b3fFromDouble( 5.0 );
	def.angularDampingRatio = b3fFromDouble( 2.0 );

	// The bounds must actually cover the load, which is the whole point of the
	// scene. At the default density of 1000 the platform masses 50 kg and the
	// cargo 64, so the spring is holding about 1140 N -- a bound below that
	// would not be a stiff platform, it would be a platform slowly falling.
	def.maxSpringForce = b3fFromDouble( 6000.0 );
	def.maxSpringTorque = b3fFromDouble( 6000.0 );

	b3JointId jointId = b3CreateMotorJoint( worldId, &def );
	validate( world );

	for ( int i = 0; i < 600; ++i )
	{
		b3World_Step( worldId, 4 );
	}
	validate( world );

	b3Vec3 pp = b3Body_GetPosition( platform );
	b3Vec3 cp = b3Body_GetPosition( cargo );
	printf( "  platform holds y %.4f (built at 2.0), cargo rests at y %.4f\n", F( pp.y ), F( cp.y ) );

	check( "the platform holds station under load", fabs( F( pp.y ) - 2.0 ) < 0.15 );
	check( "the cargo is carried rather than dropped through", F( cp.y ) > F( pp.y ) );
	check( "and is not launched", F( cp.y ) < F( pp.y ) + 1.0 );
	check( "the joint survives", b3Joint_IsValid( jointId ) );

	b3DestroyWorld( worldId );
}

static void test_motor_joint_sleep_and_accessors( void )
{
	printf( "motor joint through sleep, and accessor round-trips\n" );

	b3WorldId worldId = makeStepWorld( V( 0, -10, 0 ) );
	b3World* world = b3GetWorldFromId( worldId );

	b3BodyDef anchorDef = b3DefaultBodyDef();
	anchorDef.type = b3_staticBody;
	anchorDef.position = V( 0, 0, 0 );
	b3BodyId anchor = b3CreateBody( worldId, &anchorDef );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = V( 0, 0, 0 );
	b3BodyId body = b3CreateBody( worldId, &bodyDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3Sphere ball = { V( 0, 0, 0 ), b3fFromDouble( 0.2 ) };
	b3CreateSphereShape( body, &shapeDef, &ball );

	b3MotorJointDef def = b3DefaultMotorJointDef();
	def.base.bodyIdA = anchor;
	def.base.bodyIdB = body;
	def.linearHertz = b3fFromDouble( 5.0 );
	def.linearDampingRatio = b3fFromDouble( 2.0 );

	// Above the sphere's 335 N weight, so the spring actually settles. An
	// under-budgeted spring sags forever, and a body that never stops moving
	// never sleeps -- which reads as a sleeping bug and is a scene bug.
	def.maxSpringForce = b3fFromDouble( 2000.0 );

	b3JointId jointId = b3CreateMotorJoint( worldId, &def );
	validate( world );

	b3Body_EnableSleep( body, true );

	int steps = 0;
	while ( steps < 1200 && b3GetBodyFullId( world, body )->setIndex == b3_awakeSet )
	{
		b3World_Step( worldId, 4 );
		steps++;
	}
	validate( world );

	printf( "  slept after %d steps\n", steps );
	check( "a motor-driven body sleeps once it settles", b3GetBodyFullId( world, body )->setIndex != b3_awakeSet );
	check( "and its joint is still valid", b3Joint_IsValid( jointId ) );

	b3Body_SetAwake( body, true );
	b3World_Step( worldId, 4 );
	validate( world );
	check( "and wakes again", b3GetBodyFullId( world, body )->setIndex == b3_awakeSet );

	checkInt( "type round-trips", b3Joint_GetType( jointId ), b3_motorJoint );

	b3MotorJoint_SetLinearVelocity( jointId, V( 1.0, -2.0, 0.5 ) );
	b3Vec3 lv = b3MotorJoint_GetLinearVelocity( jointId );
	expect( "linear velocity round-trips x", F( lv.x ), 1.0, 0.01 );
	expect( "and y", F( lv.y ), -2.0, 0.01 );

	b3MotorJoint_SetAngularVelocity( jointId, V( 0.25, 0.5, -0.75 ) );
	b3Vec3 av = b3MotorJoint_GetAngularVelocity( jointId );
	expect( "angular velocity round-trips z", F( av.z ), -0.75, 0.01 );

	b3MotorJoint_SetMaxVelocityForce( jointId, b3fFromDouble( 12.0 ) );
	expect( "max velocity force round-trips", F( b3MotorJoint_GetMaxVelocityForce( jointId ) ), 12.0, 0.01 );
	b3MotorJoint_SetMaxVelocityTorque( jointId, b3fFromDouble( 3.5 ) );
	expect( "max velocity torque round-trips", F( b3MotorJoint_GetMaxVelocityTorque( jointId ) ), 3.5, 0.01 );
	b3MotorJoint_SetMaxSpringForce( jointId, b3fFromDouble( 7.0 ) );
	expect( "max spring force round-trips", F( b3MotorJoint_GetMaxSpringForce( jointId ) ), 7.0, 0.01 );
	b3MotorJoint_SetMaxSpringTorque( jointId, b3fFromDouble( 2.5 ) );
	expect( "max spring torque round-trips", F( b3MotorJoint_GetMaxSpringTorque( jointId ) ), 2.5, 0.01 );

	// Negative bounds are clamped to zero rather than asserted -- the four
	// branches read `> 0` to decide whether to run, so a negative left
	// unclamped would enable a drive with a negative allowance.
	b3MotorJoint_SetMaxVelocityForce( jointId, b3fFromDouble( -5.0 ) );
	expect( "a negative bound clamps to zero", F( b3MotorJoint_GetMaxVelocityForce( jointId ) ), 0.0, 0.001 );

	b3MotorJoint_SetLinearHertz( jointId, b3fFromDouble( 3.0 ) );
	expect( "linear hertz round-trips", F( b3MotorJoint_GetLinearHertz( jointId ) ), 3.0, 0.01 );
	b3MotorJoint_SetAngularHertz( jointId, b3fFromDouble( 8.0 ) );
	expect( "angular hertz round-trips", F( b3MotorJoint_GetAngularHertz( jointId ) ), 8.0, 0.01 );
	b3MotorJoint_SetLinearDampingRatio( jointId, b3fFromDouble( 1.25 ) );
	expect( "linear damping round-trips", F( b3MotorJoint_GetLinearDampingRatio( jointId ) ), 1.25, 0.01 );
	b3MotorJoint_SetAngularDampingRatio( jointId, b3fFromDouble( 0.75 ) );
	expect( "angular damping round-trips", F( b3MotorJoint_GetAngularDampingRatio( jointId ) ), 0.75, 0.01 );

	b3DestroyJoint( jointId, true );
	validate( world );
	check( "a destroyed motor joint is not valid", b3Joint_IsValid( jointId ) == false );
	checkInt( "and the body no longer counts it", b3GetBodyFullId( world, body )->jointCount, 0 );

	b3DestroyWorld( worldId );
}

/// A default motor joint does nothing, which is the documented behaviour.
///
/// Every bound defaults to zero and a zero bound disables its branch, so a
/// motor joint the caller has not configured must be indistinguishable from no
/// joint at all. Worth a test because it is the one default that would be
/// silently wrong in the *safe-looking* direction if a branch ran anyway.
static void test_motor_joint_zero_bounds_do_nothing( void )
{
	printf( "motor joint with no budget applies nothing\n" );

	const double g = 10.0;
	b3WorldId worldId = makeStepWorld( V( 0, -g, 0 ) );
	b3World* world = b3GetWorldFromId( worldId );

	b3BodyDef anchorDef = b3DefaultBodyDef();
	anchorDef.type = b3_staticBody;
	anchorDef.position = V( 0, 0, 0 );
	b3BodyId anchor = b3CreateBody( worldId, &anchorDef );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = V( 0, 0, 0 );
	bodyDef.enableSleep = false;
	b3BodyId body = b3CreateBody( worldId, &bodyDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3Sphere ball = { V( 0, 0, 0 ), b3fFromDouble( 0.2 ) };
	b3CreateSphereShape( body, &shapeDef, &ball );

	// Hertz set but no bounds: the springs must still stay off, because each
	// runs only when its hertz **and** its bound are non-zero.
	b3MotorJointDef def = b3DefaultMotorJointDef();
	def.base.bodyIdA = anchor;
	def.base.bodyIdB = body;
	def.linearHertz = b3fFromDouble( 10.0 );
	def.angularHertz = b3fFromDouble( 10.0 );

	b3JointId jointId = b3CreateMotorJoint( worldId, &def );
	validate( world );

	const int steps = 120;
	for ( int i = 0; i < steps; ++i )
	{
		b3World_Step( worldId, 4 );
	}
	validate( world );

	b3Vec3 p = b3Body_GetPosition( body );
	const double t = (double)steps / 60.0;
	const double freefall = -0.5 * g * t * t;

	printf( "  fell to y %.4f after %.1f s; free fall predicts %.4f\n", F( p.y ), t, freefall );

	check( "the joint survives", b3Joint_IsValid( jointId ) );
	expect( "an unbudgeted motor joint leaves the body in free fall", F( p.y ), freefall, 0.05 * fabs( freefall ) );

	b3DestroyWorld( worldId );
}

// =========================================================================
// Prismatic joint -- Phase 6 Stage 6
// =========================================================================

/// A slider on a rail: a static anchor, a dynamic box, and a prismatic joint
/// whose x axis points along `axis`.
///
/// The joint frames are built by rotating frame A's x onto the wanted
/// direction, which is how a caller aims a slider -- there is no axis
/// parameter, exactly as the revolute has no hinge-axis parameter.
static b3JointId sliderRail( b3WorldId worldId, b3Quat frameQ, b3Vec3 startPos, b3BoxHull* hull,
							 b3PrismaticJointDef* defOut, b3BodyId* anchorOut, b3BodyId* sliderOut )
{
	b3BodyDef anchorDef = b3DefaultBodyDef();
	anchorDef.type = b3_staticBody;
	anchorDef.position = V( 0, 0, 0 );
	b3BodyId anchor = b3CreateBody( worldId, &anchorDef );

	b3BodyDef sliderDef = b3DefaultBodyDef();
	sliderDef.type = b3_dynamicBody;
	sliderDef.position = startPos;
	sliderDef.enableSleep = false;
	b3BodyId slider = b3CreateBody( worldId, &sliderDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3CreateHullShape( slider, &shapeDef, &hull->base );

	b3PrismaticJointDef def = b3DefaultPrismaticJointDef();
	def.base.bodyIdA = anchor;
	def.base.bodyIdB = slider;
	def.base.localFrameA.q = frameQ;
	def.base.localFrameB.q = frameQ;

	// Frame B sits at the same world point as frame A, expressed in the
	// slider's local space -- so the joint's reference translation is zero and
	// startPos is where the slider begins along the rail.
	def.base.localFrameB.p = b3Neg( startPos );

	*defOut = def;
	*anchorOut = anchor;
	*sliderOut = slider;
	return b3CreatePrismaticJoint( worldId, &def );
}

/// A frictionless rail at 30 degrees: the slider must accelerate at exactly
/// `g * sin(30) = 4.9 m/s^2`.
///
/// The stage's headline closed form, and the counterpart of the revolute's
/// pendulum period. Nothing else in the suite checks that the axial degree is
/// genuinely **free** rather than merely soft -- every other prismatic test
/// constrains it with a spring, a motor or a limit.
static void test_prismatic_joint_free_slide( void )
{
	printf( "prismatic joint slides free on an inclined rail\n" );

	const double g = 9.8;
	b3WorldId worldId = makeStepWorld( V( 0, -g, 0 ) );
	b3World* world = b3GetWorldFromId( worldId );

	// A 30-degree incline in the x-y plane: frame A's x rotated 30 degrees
	// about z. B3_BRAD_PI is a half turn, so a sixth of that is 30 degrees.
	b3Quat frameQ = b3MakeQuatFromAxisAngle( b3Vec3_axisZ, (b3a)( B3_BRAD_PI / 6 ) );

	b3BoxHull hull = b3MakeBoxHull( b3fFromDouble( 0.15 ), b3fFromDouble( 0.15 ), b3fFromDouble( 0.15 ) );
	b3PrismaticJointDef def;
	b3BodyId anchor, slider;
	b3JointId jointId = sliderRail( worldId, frameQ, V( 0, 0, 0 ), &hull, &def, &anchor, &slider );
	validate( world );

	const int steps = 120;
	for ( int i = 0; i < steps; ++i )
	{
		b3World_Step( worldId, 4 );
	}
	validate( world );

	const double t = (double)steps / 60.0;

	// Down the slope, so the translation is negative: s = -1/2 * g sin(a) * t^2.
	const double accel = g * 0.5;
	const double predicted = -0.5 * accel * t * t;

	double translation = F( b3PrismaticJoint_GetTranslation( jointId ) );
	double speed = F( b3PrismaticJoint_GetSpeed( jointId ) );

	printf( "  after %.1f s: translation %.5f (predicts %.5f), speed %.5f (predicts %.5f)\n", t, translation,
			predicted, speed, -accel * t );

	expect( "the slider accelerates at g*sin(30) down the rail", translation, predicted, 0.05 );
	expect( "and its speed matches", speed, -accel * t, 0.05 );

	// And it stayed *on* the rail: the position must be the axis times the
	// translation, with no perpendicular excursion.
	b3Vec3 p = b3Body_GetPosition( slider );
	double along = F( p.x ) * 0.8660254 + F( p.y ) * 0.5;
	double perpY = F( p.x ) * -0.5 + F( p.y ) * 0.8660254;
	printf( "  position along the rail %.5f, off it %.5f / %.5f\n", along, perpY, F( p.z ) );

	expect( "no excursion off the rail in the plane", perpY, 0.0, 0.01 );
	expect( "nor out of it", F( p.z ), 0.0, 0.01 );

	b3DestroyWorld( worldId );
}

/// The reaction force points where the constraint actually pushes.
///
/// **This is the test that catches the upstream component permutation**, and it
/// exists because `run_pair` cannot: the harness compares the reaction force by
/// *magnitude*, and a permutation of three components preserves magnitude
/// exactly. Two libraries can therefore agree to the last bit while one of them
/// reports the force along the rail and the other reports it holding the
/// slider up.
///
/// The scene is the simplest one that separates them: a horizontal rail, a
/// slider at rest under gravity. The rail runs along **x**, so the axial degree
/// carries no load at all -- the entire reaction is the point-to-line
/// constraint holding the box against gravity, and it must point **+y** with
/// magnitude `m * g`. Upstream's `(perp.x, perp.y, axial)` ordering reports the
/// same magnitude along the wrong axis.
static void test_prismatic_joint_reaction_direction( void )
{
	printf( "prismatic joint reaction force points along the constraint\n" );

	const double g = 9.8;
	b3WorldId worldId = makeStepWorld( V( 0, -g, 0 ) );
	b3World* world = b3GetWorldFromId( worldId );

	b3BoxHull hull = b3MakeBoxHull( b3fFromDouble( 0.2 ), b3fFromDouble( 0.2 ), b3fFromDouble( 0.2 ) );
	b3PrismaticJointDef def;
	b3BodyId anchor, slider;
	b3JointId jointId = sliderRail( worldId, b3Quat_identity, V( 0, 0, 0 ), &hull, &def, &anchor, &slider );
	validate( world );

	for ( int i = 0; i < 300; ++i )
	{
		b3World_Step( worldId, 4 );
	}
	validate( world );

	double mass = F( b3Body_GetMass( slider ) );
	b3Vec3 force = b3Joint_GetConstraintForce( jointId );
	double weight = mass * g;
	double magnitude = sqrt( F( force.x ) * F( force.x ) + F( force.y ) * F( force.y ) + F( force.z ) * F( force.z ) );

	printf( "  mass %.4f kg, weight %.4f N; reaction (%.4f %.4f %.4f), magnitude %.4f\n", mass, weight, F( force.x ),
			F( force.y ), F( force.z ), magnitude );

	// The magnitude both orderings agree on, so it proves the solve but not the
	// readout. Checked anyway, because a wrong magnitude would mean something
	// else entirely.
	expect( "the reaction magnitude is the weight it carries", magnitude, weight, 0.05 * weight );

	// The direction is the whole point. y carries all of it.
	expect( "and it points +y, holding the slider up", F( force.y ), weight, 0.05 * weight );

	// x is the rail: a free axis carries no reaction at all. This is the
	// component upstream's permutation puts the weight into.
	check( "with nothing along the free axis", fabs( F( force.x ) ) < 0.05 * weight );
	check( "and nothing across it", fabs( F( force.z ) ) < 0.05 * weight );

	b3DestroyWorld( worldId );
}

/// A sprung slider settles at `x = g / (2*pi*f)^2` below its target.
///
/// The mass cancels at equilibrium -- `k*x = m*g` with `k = m*w^2` -- so the sag
/// is independent of the scene, exactly as Stage 5's weld spring and Stage 4's
/// conical pendulum were. A closed form that cannot be fitted.
static void test_prismatic_joint_spring( void )
{
	printf( "prismatic joint as a spring\n" );

	const double g = 9.8;
	const double hertz = 1.5;
	b3WorldId worldId = makeStepWorld( V( 0, -g, 0 ) );
	b3World* world = b3GetWorldFromId( worldId );

	// A vertical rail, so gravity acts entirely along the slide axis and the
	// spring is the only thing resisting it.
	b3BoxHull hull = b3MakeBoxHull( b3fFromDouble( 0.2 ), b3fFromDouble( 0.2 ), b3fFromDouble( 0.2 ) );
	b3PrismaticJointDef def;
	b3BodyId anchor, slider;
	b3JointId jointId = sliderRail( worldId, b3MakeQuatFromAxisAngle( b3Vec3_axisZ, B3_BRAD_HALF_PI ), V( 0, 0, 0 ), &hull, &def, &anchor, &slider );

	b3PrismaticJoint_EnableSpring( jointId, true );
	b3PrismaticJoint_SetSpringHertz( jointId, b3fFromDouble( hertz ) );
	b3PrismaticJoint_SetSpringDampingRatio( jointId, b3fFromDouble( 2.0 ) );
	b3PrismaticJoint_SetTargetTranslation( jointId, b3f_zero );
	validate( world );

	for ( int i = 0; i < 900; ++i )
	{
		b3World_Step( worldId, 4 );
	}
	validate( world );

	double sag = -F( b3PrismaticJoint_GetTranslation( jointId ) );
	const double omega = 2.0 * 3.14159265358979 * hertz;
	const double predicted = g / ( omega * omega );

	printf( "  sag at %.1f Hz: %.5f m, g/(2*pi*f)^2 predicts %.5f (ratio %.4f)\n", hertz, sag, predicted,
			sag / predicted );

	expect( "the spring sags by g/(2*pi*f)^2", sag, predicted, 0.1 * predicted );

	b3DestroyWorld( worldId );
}

/// A motor holding a load reports exactly the force it is spending.
///
/// A vertical rail with the motor asked to hold station: at equilibrium the
/// only thing it fights is the weight, so `GetMotorForce()` must be `m*g`. The
/// closed form the revolute's torque test uses, one dimension over, and the
/// second check on the impulse-to-force scale.
static void test_prismatic_joint_motor( void )
{
	printf( "prismatic joint motor holds a load\n" );

	const double g = 9.8;
	b3WorldId worldId = makeStepWorld( V( 0, -g, 0 ) );
	b3World* world = b3GetWorldFromId( worldId );

	b3BoxHull hull = b3MakeBoxHull( b3fFromDouble( 0.2 ), b3fFromDouble( 0.2 ), b3fFromDouble( 0.2 ) );
	b3PrismaticJointDef def;
	b3BodyId anchor, slider;
	b3JointId jointId = sliderRail( worldId, b3MakeQuatFromAxisAngle( b3Vec3_axisZ, B3_BRAD_HALF_PI ), V( 0, 0, 0 ), &hull, &def, &anchor, &slider );

	double mass = F( b3Body_GetMass( slider ) );
	double weight = mass * g;

	// Budget well above the weight, so the drive is unsaturated and holds.
	b3PrismaticJoint_EnableMotor( jointId, true );
	b3PrismaticJoint_SetMotorSpeed( jointId, b3f_zero );
	b3PrismaticJoint_SetMaxMotorForce( jointId, b3fFromDouble( 10.0 * weight ) );
	validate( world );

	for ( int i = 0; i < 600; ++i )
	{
		b3World_Step( worldId, 4 );
	}
	validate( world );

	double motorForce = F( b3PrismaticJoint_GetMotorForce( jointId ) );
	double translation = F( b3PrismaticJoint_GetTranslation( jointId ) );

	printf( "  weight %.4f N; motor force %.4f N, held at translation %.5f\n", weight, motorForce, translation );

	expect( "the motor spends exactly the weight it holds", fabs( motorForce ), weight, 0.05 * weight );
	expect( "and the slider stays where it was put", translation, 0.0, 0.02 );

	// Now starve it: a budget below the weight must lose, and lose at its bound
	// rather than at some other number.
	const double budget = 0.25 * weight;
	b3PrismaticJoint_SetMaxMotorForce( jointId, b3fFromDouble( budget ) );

	for ( int i = 0; i < 120; ++i )
	{
		b3World_Step( worldId, 4 );
	}
	validate( world );

	double saturated = fabs( F( b3PrismaticJoint_GetMotorForce( jointId ) ) );
	double fallen = F( b3PrismaticJoint_GetTranslation( jointId ) );
	printf( "  starved to %.4f N: spends %.4f N and the load falls to %.4f\n", budget, saturated, fallen );

	expect( "a starved motor spends exactly its bound", saturated, budget, 0.1 * budget );
	check( "and the load falls rather than being held", fallen < -0.1 );

	b3DestroyWorld( worldId );
}

/// Both limits stop the slider, and the lower one carries the load.
static void test_prismatic_joint_limits( void )
{
	printf( "prismatic joint limits stop the slide\n" );

	const double g = 9.8;
	b3WorldId worldId = makeStepWorld( V( 0, -g, 0 ) );
	b3World* world = b3GetWorldFromId( worldId );

	b3BoxHull hull = b3MakeBoxHull( b3fFromDouble( 0.2 ), b3fFromDouble( 0.2 ), b3fFromDouble( 0.2 ) );
	b3PrismaticJointDef def;
	b3BodyId anchor, slider;
	b3JointId jointId = sliderRail( worldId, b3MakeQuatFromAxisAngle( b3Vec3_axisZ, B3_BRAD_HALF_PI ), V( 0, 0, 0 ), &hull, &def, &anchor, &slider );

	// A 3 m range, so the speculative band is 0.75 m -- traversed rather than
	// jumped, which is what makes the band do real work.
	b3PrismaticJoint_EnableLimit( jointId, true );
	b3PrismaticJoint_SetLimits( jointId, b3fFromDouble( -1.5 ), b3fFromDouble( 1.5 ) );
	validate( world );

	// Falls to the lower limit and rests there.
	for ( int i = 0; i < 900; ++i )
	{
		b3World_Step( worldId, 4 );
	}
	validate( world );

	double atLower = F( b3PrismaticJoint_GetTranslation( jointId ) );
	printf( "  fell to the lower limit: translation %.5f (limit -1.5)\n", atLower );

	expect( "the lower limit stops the fall", atLower, -1.5, 0.05 );

	// Driven up hard against the upper one. The budget is scaled to the weight
	// rather than written as a number: a drive that cannot lift its own load
	// never reaches the limit, and the test would then be measuring the motor.
	double weight = F( b3Body_GetMass( slider ) ) * g;
	b3PrismaticJoint_EnableMotor( jointId, true );
	b3PrismaticJoint_SetMotorSpeed( jointId, b3fFromDouble( 4.0 ) );
	b3PrismaticJoint_SetMaxMotorForce( jointId, b3fFromDouble( 4.0 * weight ) );

	for ( int i = 0; i < 900; ++i )
	{
		b3World_Step( worldId, 4 );
	}
	validate( world );

	double atUpper = F( b3PrismaticJoint_GetTranslation( jointId ) );
	printf( "  driven up against the upper limit: translation %.5f (limit 1.5)\n", atUpper );

	expect( "the upper limit stops the drive", atUpper, 1.5, 0.05 );

	// Sorted, not asserted: passing them the wrong way round gives the range
	// the caller meant.
	b3PrismaticJoint_SetLimits( jointId, b3fFromDouble( 2.0 ), b3fFromDouble( -2.0 ) );
	expect( "SetLimits sorts its arguments, lower", F( b3PrismaticJoint_GetLowerLimit( jointId ) ), -2.0, 0.01 );
	expect( "and upper", F( b3PrismaticJoint_GetUpperLimit( jointId ) ), 2.0, 0.01 );

	b3DestroyWorld( worldId );
}

/// The orientation lock holds, and the point-to-line lock holds.
///
/// Constraints 4 and 5 in isolation: body B is given a large initial angular
/// velocity about all three axes and a shove across the rail. A slider does not
/// turn and does not leave its line, however hard it is pushed.
static void test_prismatic_joint_locks( void )
{
	printf( "prismatic joint holds its orientation and its line\n" );

	b3WorldId worldId = makeStepWorld( V( 0, 0, 0 ) );
	b3World* world = b3GetWorldFromId( worldId );

	b3BoxHull hull = b3MakeBoxHull( b3fFromDouble( 0.2 ), b3fFromDouble( 0.2 ), b3fFromDouble( 0.2 ) );
	b3PrismaticJointDef def;
	b3BodyId anchor, slider;
	// The joint id is deliberately dropped: this test asserts on the *body*, not
	// on anything the joint reports, so holding the id only produced a warning.
	sliderRail( worldId, b3Quat_identity, V( 0, 0, 0 ), &hull, &def, &anchor, &slider );

	b3Body_SetAngularVelocity( slider, V( 5.0, -4.0, 3.0 ) );
	b3Body_SetLinearVelocity( slider, V( 1.0, 2.5, -2.0 ) );
	validate( world );

	for ( int i = 0; i < 600; ++i )
	{
		b3World_Step( worldId, 4 );
	}
	validate( world );

	b3Vec3 p = b3Body_GetPosition( slider );
	b3Quat q = b3Body_GetRotation( slider );
	b3Vec3 w = b3Body_GetAngularVelocity( slider );

	// The rail is x, so y and z must be untouched however hard it was shoved.
	printf( "  after a shove: position (%.5f %.5f %.5f), spin %.5f\n", F( p.x ), F( p.y ), F( p.z ),
			sqrt( F( w.x ) * F( w.x ) + F( w.y ) * F( w.y ) + F( w.z ) * F( w.z ) ) );

	expect( "the point-to-line lock holds y", F( p.y ), 0.0, 0.02 );
	expect( "and z", F( p.z ), 0.0, 0.02 );

	// The orientation lock: the slider's rotation must still be the identity,
	// so the quaternion's vector part is zero.
	double tilt = sqrt( F( b3FromDir3( q.v ).x ) * F( b3FromDir3( q.v ).x ) +
						F( b3FromDir3( q.v ).y ) * F( b3FromDir3( q.v ).y ) +
						F( b3FromDir3( q.v ).z ) * F( b3FromDir3( q.v ).z ) );
	printf( "  orientation error (quaternion vector part) %.6f\n", tilt );
	check( "the orientation lock holds the slider level", tilt < 0.02 );

	// But it did slide, which is the degree it is supposed to leave alone.
	check( "while the free axis still moved", fabs( F( p.x ) ) > 0.1 );

	b3DestroyWorld( worldId );
}

/// A default prismatic joint is a pure point-to-line constraint.
///
/// Every branch defaults off and both limits to zero, so an unconfigured slider
/// must fall freely along its rail -- indistinguishable from a joint with no
/// spring, motor or limit at all. Stage 5's default-motor free-fall test is the
/// model, and it is the one that catches a branch running when it should not:
/// if the limit branch ran on its zero-width default range, the slider would be
/// pinned at the origin instead of falling.
static void test_prismatic_joint_default_is_free( void )
{
	printf( "prismatic joint with no configuration slides freely\n" );

	const double g = 9.8;
	b3WorldId worldId = makeStepWorld( V( 0, -g, 0 ) );
	b3World* world = b3GetWorldFromId( worldId );

	b3BoxHull hull = b3MakeBoxHull( b3fFromDouble( 0.2 ), b3fFromDouble( 0.2 ), b3fFromDouble( 0.2 ) );
	b3PrismaticJointDef def;
	b3BodyId anchor, slider;
	b3JointId jointId = sliderRail( worldId, b3MakeQuatFromAxisAngle( b3Vec3_axisZ, B3_BRAD_HALF_PI ), V( 0, 0, 0 ), &hull, &def, &anchor, &slider );

	check( "the spring is off by default", b3PrismaticJoint_IsSpringEnabled( jointId ) == false );
	check( "the motor is off by default", b3PrismaticJoint_IsMotorEnabled( jointId ) == false );
	check( "the limit is off by default", b3PrismaticJoint_IsLimitEnabled( jointId ) == false );

	const int steps = 120;
	for ( int i = 0; i < steps; ++i )
	{
		b3World_Step( worldId, 4 );
	}
	validate( world );

	const double t = (double)steps / 60.0;
	const double freefall = -0.5 * g * t * t;
	double translation = F( b3PrismaticJoint_GetTranslation( jointId ) );

	printf( "  slid to %.4f after %.1f s; free fall predicts %.4f\n", translation, t, freefall );

	expect( "an unconfigured slider is in free fall along its rail", translation, freefall, 0.05 );

	b3DestroyWorld( worldId );
}

/// The same rail at the origin and 1500 units out must behave identically.
///
/// The world-size note made executable. Upstream differences the two body
/// centres in `double` so that GetSpeed stays exact far from the origin; the
/// port does not, because Q12's quantum is **absolute** rather than relative --
/// there is no exponent to lose, so the difference of two positions is exact
/// wherever they sit. This asserts that claim instead of merely documenting it.
static void test_prismatic_joint_far_from_origin( void )
{
	printf( "prismatic joint reads the same far from the origin\n" );

	const double g = 9.8;
	const double offsets[2] = { 0.0, 1500.0 };
	double translation[2] = { 0.0, 0.0 };
	double speed[2] = { 0.0, 0.0 };

	for ( int k = 0; k < 2; ++k )
	{
		b3WorldId worldId = makeStepWorld( V( 0, -g, 0 ) );
		b3World* world = b3GetWorldFromId( worldId );

		b3BodyDef anchorDef = b3DefaultBodyDef();
		anchorDef.type = b3_staticBody;
		anchorDef.position = V( offsets[k], 0, 0 );
		b3BodyId anchor = b3CreateBody( worldId, &anchorDef );

		b3BodyDef sliderDef = b3DefaultBodyDef();
		sliderDef.type = b3_dynamicBody;
		sliderDef.position = V( offsets[k], 0, 0 );
		sliderDef.enableSleep = false;
		b3BodyId slider = b3CreateBody( worldId, &sliderDef );

		b3BoxHull hull = b3MakeBoxHull( b3fFromDouble( 0.2 ), b3fFromDouble( 0.2 ), b3fFromDouble( 0.2 ) );
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		b3CreateHullShape( slider, &shapeDef, &hull.base );

		// A vertical rail: rotate x onto y.
		b3Quat frameQ = b3MakeQuatFromAxisAngle( b3Vec3_axisZ, B3_BRAD_HALF_PI );

		b3PrismaticJointDef def = b3DefaultPrismaticJointDef();
		def.base.bodyIdA = anchor;
		def.base.bodyIdB = slider;
		def.base.localFrameA.q = frameQ;
		def.base.localFrameB.q = frameQ;

		b3JointId jointId = b3CreatePrismaticJoint( worldId, &def );
		validate( world );

		for ( int i = 0; i < 120; ++i )
		{
			b3World_Step( worldId, 4 );
		}
		validate( world );

		translation[k] = F( b3PrismaticJoint_GetTranslation( jointId ) );
		speed[k] = F( b3PrismaticJoint_GetSpeed( jointId ) );

		b3DestroyWorld( worldId );
	}

	printf( "  at x=0:    translation %.5f, speed %.5f\n", translation[0], speed[0] );
	printf( "  at x=1500: translation %.5f, speed %.5f\n", translation[1], speed[1] );

	// One Q12 quantum is 1/4096 = 0.000244. Two quanta of slack, which is what
	// the rotation of the anchor offset can legitimately cost.
	const double quantum = 2.0 / 4096.0;
	expect( "the translation is the same 1500 units out", translation[1], translation[0], quantum );
	expect( "and so is the speed", speed[1], speed[0], quantum );
}

/// Sleep, wake, and the accessor round trips.
static void test_prismatic_joint_sleep_and_accessors( void )
{
	printf( "prismatic joint through sleep, and accessor round-trips\n" );

	b3WorldId worldId = makeStepWorld( V( 0, -10, 0 ) );
	b3World* world = b3GetWorldFromId( worldId );

	b3BoxHull hull = b3MakeBoxHull( b3fFromDouble( 0.2 ), b3fFromDouble( 0.2 ), b3fFromDouble( 0.2 ) );
	b3PrismaticJointDef def;
	b3BodyId anchor, slider;
	b3JointId jointId = sliderRail( worldId, b3Quat_identity, V( 0, 0, 0 ), &hull, &def, &anchor, &slider );

	b3PrismaticJoint_SetSpringHertz( jointId, b3fFromDouble( 4.0 ) );
	expect( "spring hertz round-trips", F( b3PrismaticJoint_GetSpringHertz( jointId ) ), 4.0, 0.01 );
	b3PrismaticJoint_SetSpringDampingRatio( jointId, b3fFromDouble( 1.5 ) );
	expect( "damping ratio round-trips", F( b3PrismaticJoint_GetSpringDampingRatio( jointId ) ), 1.5, 0.01 );
	b3PrismaticJoint_SetTargetTranslation( jointId, b3fFromDouble( -0.75 ) );
	expect( "target translation round-trips", F( b3PrismaticJoint_GetTargetTranslation( jointId ) ), -0.75, 0.01 );
	b3PrismaticJoint_SetMotorSpeed( jointId, b3fFromDouble( 2.25 ) );
	expect( "motor speed round-trips", F( b3PrismaticJoint_GetMotorSpeed( jointId ) ), 2.25, 0.01 );
	b3PrismaticJoint_SetMaxMotorForce( jointId, b3fFromDouble( 30.0 ) );
	expect( "max motor force round-trips", F( b3PrismaticJoint_GetMaxMotorForce( jointId ) ), 30.0, 0.01 );

	// Clamped rather than asserted, as the motor joint's bounds are.
	b3PrismaticJoint_SetMaxMotorForce( jointId, b3fFromDouble( -5.0 ) );
	expect( "a negative force budget clamps to zero", F( b3PrismaticJoint_GetMaxMotorForce( jointId ) ), 0.0, 0.001 );
	b3PrismaticJoint_SetMaxMotorForce( jointId, b3fFromDouble( 30.0 ) );

	// Toggling a branch clears its accumulator, and only on a *change*.
	b3PrismaticJoint_EnableSpring( jointId, true );
	check( "the spring toggles on", b3PrismaticJoint_IsSpringEnabled( jointId ) );
	b3PrismaticJoint_EnableMotor( jointId, true );
	check( "the motor toggles on", b3PrismaticJoint_IsMotorEnabled( jointId ) );
	b3PrismaticJoint_EnableLimit( jointId, true );
	check( "the limit toggles on", b3PrismaticJoint_IsLimitEnabled( jointId ) );

	b3Body_EnableSleep( slider, true );
	b3PrismaticJoint_EnableMotor( jointId, false );
	b3PrismaticJoint_EnableSpring( jointId, false );
	b3PrismaticJoint_SetLimits( jointId, b3fFromDouble( -0.5 ), b3fFromDouble( 0.5 ) );

	int steps = 0;
	while ( steps < 1200 && b3GetBodyFullId( world, slider )->setIndex == b3_awakeSet )
	{
		b3World_Step( worldId, 4 );
		steps++;
	}
	validate( world );

	printf( "  slept after %d steps\n", steps );
	check( "a slider on its limit goes to sleep", steps < 1200 );
	check( "and the joint is still valid", b3Joint_IsValid( jointId ) );

	b3DestroyJoint( jointId, true );
	validate( world );
	check( "a destroyed prismatic joint is not valid", b3Joint_IsValid( jointId ) == false );

	b3DestroyWorld( worldId );
}

// -------------------------------------------------------------------------
// Phase 6 Stage 7: the parallel joint
// -------------------------------------------------------------------------

/// A static anchor with its z up, and a dynamic box joined to it by a parallel
/// joint. The box carries no shape offset and no linear constraint, so it is in
/// free fall linearly and only its orientation is coupled.
///
/// `boxHalf` sizes the box, which is how the tests vary its inertia -- the
/// determinant threshold in @section silent is a statement about inertia, so it
/// has to be reachable from the scene.
static b3JointId uprightRig( b3WorldId worldId, double boxHalf, double hertz, double dampingRatio, double maxTorque,
							 b3BoxHull* hull, b3BodyId* anchorOut, b3BodyId* bodyOut )
{
	b3BodyDef anchorDef = b3DefaultBodyDef();
	anchorDef.type = b3_staticBody;
	anchorDef.position = V( 0, 0, 0 );
	b3BodyId anchor = b3CreateBody( worldId, &anchorDef );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = V( 0, 0, 0 );
	bodyDef.enableSleep = false;
	b3BodyId body = b3CreateBody( worldId, &bodyDef );

	*hull = b3MakeBoxHull( b3fFromDouble( boxHalf ), b3fFromDouble( boxHalf ), b3fFromDouble( boxHalf ) );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3CreateHullShape( body, &shapeDef, &hull->base );

	b3ParallelJointDef def = b3DefaultParallelJointDef();
	def.base.bodyIdA = anchor;
	def.base.bodyIdB = body;
	def.hertz = b3fFromDouble( hertz );
	def.dampingRatio = b3fFromDouble( dampingRatio );
	def.maxTorque = b3fFromDouble( maxTorque );

	*anchorOut = anchor;
	*bodyOut = body;
	return b3CreateParallelJoint( worldId, &def );
}

/// The joint's defining property: the torque it applies is bounded by a **disc**
/// and not by a box.
///
/// This is the test b3ClampImp2 exists for, and it is built to fail against the
/// obvious wrong implementation. The body is spun about the diagonal axis
/// (1,1,0)/sqrt(2), so the two constrained components of the impulse are loaded
/// equally; a per-component clamp would then bound each at `maxTorque` and let
/// the magnitude reach `sqrt(2) * maxTorque`. Clamping the pair radially bounds
/// the magnitude itself, which is what the accessor's name promises.
///
/// The spin is set far higher than the budget can arrest in the time given, so
/// the joint is *saturated* throughout -- two libraries agreeing on an
/// unsaturated bound would prove nothing.
static void test_parallel_joint_torque_bound_is_a_disc( void )
{
	printf( "parallel joint bounds its torque by magnitude, not per axis\n" );

	const double maxTorque = 0.5;
	b3WorldId worldId = makeStepWorld( V( 0, 0, 0 ) );
	b3World* world = b3GetWorldFromId( worldId );

	b3BoxHull hull;
	b3BodyId anchor, body;
	b3JointId jointId = uprightRig( worldId, 0.25, 4.0, 1.0, maxTorque, &hull, &anchor, &body );
	validate( world );

	// Equal about x and y, so both constrained components are driven together.
	const double spin = 40.0;
	const double c = spin / sqrt( 2.0 );
	b3Body_SetAngularVelocity( body, V( c, c, 0 ) );

	double worstMagnitude = 0.0;
	double worstComponent = 0.0;
	for ( int i = 0; i < 60; ++i )
	{
		b3World_Step( worldId, 4 );

		b3Vec3 torque = b3Joint_GetConstraintTorque( jointId );
		double tx = b3fToDouble( torque.x );
		double ty = b3fToDouble( torque.y );
		double tz = b3fToDouble( torque.z );
		double magnitude = sqrt( tx * tx + ty * ty + tz * tz );

		if ( magnitude > worstMagnitude )
		{
			worstMagnitude = magnitude;
		}
		double largest = fabs( tx ) > fabs( ty ) ? fabs( tx ) : fabs( ty );
		if ( largest > worstComponent )
		{
			worstComponent = largest;
		}
	}
	validate( world );

	// `maxTorque` bounds the impulse *coefficient*, and that coefficient is
	// applied along a Jacobian row of length 0.5 near alignment -- so the
	// torque that actually reaches the body is half the stated budget. Upstream
	// does exactly the same thing; see b3ParallelJointDef::maxTorque for why the
	// port reproduces it rather than rescaling. This is the test that pins the
	// factor, so it cannot drift unnoticed in either direction.
	const double effectiveBound = 0.5 * maxTorque;

	printf( "  budget %.2f: peak magnitude %.5f (effective bound %.4f), peak component %.5f\n", maxTorque,
			worstMagnitude, effectiveBound, worstComponent );

	// The bound holds. One percent of slack covers the Q16 root and ratio
	// inside b3ClampImp2.
	check( "the applied torque never exceeds the effective bound", worstMagnitude <= effectiveBound * 1.01 );

	// And it is genuinely saturated, so the clamp is doing work rather than
	// sitting comfortably clear of the load. Two libraries agreeing on an
	// unsaturated bound would prove nothing about the clamp.
	check( "the joint is actually saturated", worstMagnitude > effectiveBound * 0.95 );

	// The discriminating half, and the reason the load is diagonal. A radial
	// clamp bounds the magnitude, so each of two equally loaded components
	// lands at bound/sqrt(2) = 0.707 of it. A per-component clamp would bound
	// each component instead, letting the magnitude reach sqrt(2) times the
	// bound -- which the check above would catch -- and putting each component
	// at the full bound, which this one catches. Both halves are needed: the
	// magnitude check alone passes for a joint that simply applies too little.
	check( "no single component carries the whole bound", worstComponent < effectiveBound * 0.85 );
	check( "and the two components share it as a disc requires", worstComponent > effectiveBound * 0.6 );

	b3DestroyWorld( worldId );
}

/// The half-turn degeneracy: a tumbling body must be arrested, not driven.
///
/// b3CollinearityPerpAxes' rows have length `0.5 * sqrt(1 - (v.e)^2)`, which
/// **vanishes at a half turn**. A body tumbling fast therefore sweeps the 2x2's
/// determinant through zero every rotation, and its inverse through infinity --
/// so the impulse the solve asks for goes far past Q15.16. Measured at 301,342
/// N-s against a ceiling of 32,768.
///
/// Upstream never notices, because float holds the ask comfortably and the
/// clamp that follows replaces it. The port cannot: a wrapped intermediate
/// reaches b3ClampImp2 with the wrong sign, and the joint then spends its whole
/// budget *driving* the tumble. b3MulSym2VSat saturates instead, preserving the
/// direction that the clamp will keep.
///
/// **What this regression catches, stated exactly.** Swapping b3MulSym2VSat
/// back to b3MulSym2V makes the *debug* build's shadow checker abort here, on
/// the ask above. It changes device-mode behaviour by 0.004 rad/s over five
/// seconds, and a sweep of sustained near-inverted starts (150 to 175 degrees,
/// ten seconds each) came back bit-identical either way. So the checks below do
/// **not** bite without the fix -- the debug mode does, and that is the port's
/// designated instrument for exactly this class.
///
/// Recorded that way rather than dressed up: the ask is nine times past the
/// scale and wraps its sign, which is not something to leave in whether or not
/// a scene has yet been found where the clamp fails to mask it. The behavioural
/// checks below are kept because they are true, cheap, and would catch a
/// coarser regression in the same block.
///
/// The tumble rate is chosen to sweep several half turns inside the window so
/// the degenerate configuration is certain to be hit rather than merely likely.
static void test_parallel_joint_tumbling_does_not_wrap( void )
{
	printf( "parallel joint arrests a tumble rather than driving it\n" );

	b3WorldId worldId = makeStepWorld( V( 0, 0, 0 ) );
	b3World* world = b3GetWorldFromId( worldId );

	b3BoxHull hull;
	b3BodyId anchor, body;
	b3JointId jointId = uprightRig( worldId, 0.25, 4.0, 1.0, 20.0, &hull, &anchor, &body );
	validate( world );

	// 40 rad/s is 6.4 turns a second: the joint passes through the half turn
	// where its Jacobian collapses roughly thirteen times a second.
	const double spin = 40.0;
	b3Body_SetAngularVelocity( body, V( spin, 0, 0 ) );

	double peak = spin;
	for ( int i = 0; i < 300; ++i )
	{
		b3World_Step( worldId, 4 );

		b3Vec3 w = b3Body_GetAngularVelocity( body );
		double rate = fabs( b3fToDouble( w.x ) );
		if ( rate > peak )
		{
			peak = rate;
		}
	}
	validate( world );

	b3Vec3 w = b3Body_GetAngularVelocity( body );
	printf( "  tumble %.1f -> %.4f rad/s over 5.0 s, peak %.4f\n", spin, b3fToDouble( w.x ), peak );

	// The joint is a sink. A wrapped impulse turns it into a source, which is
	// what this pins -- the same failure shape as the Stage 6 rolling-resistance
	// wrap, where spin grew from 10.4 to 44.6 instead of decaying. The bound is
	// tight on purpose: the measured peak is the starting rate to four decimal
	// places, so any amplification at all fails this.
	check( "a tumble is never accelerated by the joint", peak <= spin * 1.001 );

	// And it bleeds off rather than merely not growing. It bleeds *slowly*, and
	// that is physics rather than a defect worth chasing: a soft constraint
	// passes through aligned and anti-aligned twice per turn, so its restoring
	// direction reverses with it and the net work per revolution is a small
	// residue. Measured: 40 -> 35.8 rad/s over five seconds. A rigid constraint
	// would arrest it at once, and a parallel joint is deliberately not one.
	check( "and the tumble bleeds off rather than persisting", fabs( b3fToDouble( w.x ) ) < spin * 0.95 );

	B3_UNUSED( jointId );
	b3DestroyWorld( worldId );
}

/// The twist about z is free, and that is what separates this joint from a weld.
///
/// A spin purely about the joint axis must survive untouched, however hard the
/// joint is driven -- the constraint reads only the x and y components of the
/// relative rotation's vector part, and z is deliberately not among them.
static void test_parallel_joint_twist_is_free( void )
{
	printf( "parallel joint leaves the twist about z free\n" );

	b3WorldId worldId = makeStepWorld( V( 0, 0, 0 ) );
	b3World* world = b3GetWorldFromId( worldId );

	b3BoxHull hull;
	b3BodyId anchor, body;
	b3JointId jointId = uprightRig( worldId, 0.25, 8.0, 1.0, 50.0, &hull, &anchor, &body );
	validate( world );

	const double spin = 6.0;
	b3Body_SetAngularVelocity( body, V( 0, 0, spin ) );

	for ( int i = 0; i < 180; ++i )
	{
		b3World_Step( worldId, 4 );
	}
	validate( world );

	b3Vec3 w = b3Body_GetAngularVelocity( body );
	printf( "  after 3.0 s of free twist: w = (%.4f %.4f %.4f)\n", b3fToDouble( w.x ), b3fToDouble( w.y ),
			b3fToDouble( w.z ) );

	expect( "the twist rate is untouched", b3fToDouble( w.z ), spin, 0.05 );
	expect( "and nothing leaks into x", b3fToDouble( w.x ), 0.0, 0.05 );
	expect( "and nothing leaks into y", b3fToDouble( w.y ), 0.0, 0.05 );

	// The joint should report essentially no torque, because there is nothing
	// for it to correct.
	b3Vec3 torque = b3Joint_GetConstraintTorque( jointId );
	double magnitude = sqrt( b3fToDouble( torque.x ) * b3fToDouble( torque.x ) +
							 b3fToDouble( torque.y ) * b3fToDouble( torque.y ) +
							 b3fToDouble( torque.z ) * b3fToDouble( torque.z ) );
	check( "and the joint applies no torque to a pure twist", magnitude < 0.05 );

	b3DestroyWorld( worldId );
}

/// Given enough budget, the joint does what it is for: a tilted body comes back
/// upright and stays there.
///
/// The residual is the number that matters. A soft constraint does not reach
/// zero error -- it reaches the point where its bias balances what is left --
/// so this pins how close, in brads, rather than asserting an exact alignment
/// the joint never claims.
static void test_parallel_joint_rights_a_tilt( void )
{
	printf( "parallel joint brings a tilted body upright\n" );

	b3WorldId worldId = makeStepWorld( V( 0, 0, 0 ) );
	b3World* world = b3GetWorldFromId( worldId );

	b3BoxHull hull;
	b3BodyId anchor, body;
	b3JointId jointId = uprightRig( worldId, 0.25, 6.0, 1.0, 200.0, &hull, &anchor, &body );
	validate( world );

	// Tilt 30 degrees about x, so B's z leaves A's z by that much.
	b3Quat tilt = b3MakeQuatFromAxisAngle( b3Vec3_axisX, (b3a)( B3_BRAD_PI / 6 ) );
	b3Body_SetTransform( body, V( 0, 0, 0 ), tilt );
	validate( world );

	b3a startTilt = b3GetQuatAngle( b3Body_GetRotation( body ) );

	for ( int i = 0; i < 240; ++i )
	{
		b3World_Step( worldId, 4 );
	}
	validate( world );

	b3a endTilt = b3GetQuatAngle( b3Body_GetRotation( body ) );

	printf( "  tilt %d brads -> %d brads after 4.0 s\n", (int)startTilt, (int)endTilt );

	// 30 degrees is 2730 brads: a full turn is 32768, not 2*pi.
	check( "the body started tilted", startTilt > 2500 && startTilt < 3000 );
	check( "and the joint brought it upright", endTilt < 200 );

	// Held, not merely passed through: another second must not undo it.
	for ( int i = 0; i < 60; ++i )
	{
		b3World_Step( worldId, 4 );
	}
	b3a heldTilt = b3GetQuatAngle( b3Body_GetRotation( body ) );
	check( "and holds it there", heldTilt < 200 );

	B3_UNUSED( jointId );
	b3DestroyWorld( worldId );
}

/// A default-constructed parallel joint does nothing at all.
///
/// `maxTorque` defaults to zero, where upstream has FLT_MAX, and the solve is
/// skipped entirely when it is. That divergence is deliberate -- see
/// b3ParallelJointDef::maxTorque -- so it is pinned here rather than left to be
/// discovered as a surprise.
static void test_parallel_joint_default_is_inert( void )
{
	printf( "parallel joint with no torque budget does nothing\n" );

	b3WorldId worldId = makeStepWorld( V( 0, 0, 0 ) );
	b3World* world = b3GetWorldFromId( worldId );

	b3BodyId anchor, body;
	b3BoxHull hull;

	// Everything from the default def except the two bodies -- in particular
	// maxTorque is left where b3DefaultParallelJointDef puts it.
	b3BodyDef anchorDef = b3DefaultBodyDef();
	anchorDef.type = b3_staticBody;
	anchor = b3CreateBody( worldId, &anchorDef );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.enableSleep = false;
	body = b3CreateBody( worldId, &bodyDef );

	hull = b3MakeBoxHull( b3fFromDouble( 0.25 ), b3fFromDouble( 0.25 ), b3fFromDouble( 0.25 ) );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3CreateHullShape( body, &shapeDef, &hull.base );

	b3ParallelJointDef def = b3DefaultParallelJointDef();
	def.base.bodyIdA = anchor;
	def.base.bodyIdB = body;
	b3JointId jointId = b3CreateParallelJoint( worldId, &def );
	validate( world );

	checkInt( "a default def leaves the torque budget at zero", b3Raw( b3ParallelJoint_GetMaxTorque( jointId ) ), 0 );

	const double spin = 3.0;
	b3Body_SetAngularVelocity( body, V( spin, 0, 0 ) );

	for ( int i = 0; i < 120; ++i )
	{
		b3World_Step( worldId, 4 );
	}
	validate( world );

	b3Vec3 w = b3Body_GetAngularVelocity( body );
	printf( "  spun at %.1f rad/s for 2.0 s: w.x = %.4f\n", spin, b3fToDouble( w.x ) );

	// Untouched: an inert joint is indistinguishable from no joint at all.
	expect( "an inert joint does not slow the spin", b3fToDouble( w.x ), spin, 0.05 );

	b3Vec3 torque = b3Joint_GetConstraintTorque( jointId );
	checkInt( "and reports no torque", b3Raw( torque.x ) | b3Raw( torque.y ) | b3Raw( torque.z ), 0 );

	b3DestroyWorld( worldId );
}

/// The pair this joint cannot hold, pinned so the threshold cannot drift.
///
/// b3CollinearityPerpAxes' rows are half-length, so the 2x2's determinant
/// carries a factor of one sixteenth and b3InvertPerpMass returns zero once it
/// underflows Q24 -- at a combined inverse inertia of roughly 1e-3, i.e. a
/// rotational inertia above about 1000 kg m^2. Zero means "apply no impulse",
/// which is the safe direction, but it is silent: the body simply is not held.
///
/// The test asserts the *failure mode*, not that it does not happen. A heavy
/// pair must go quiet rather than misbehave -- an indefinite or wrapped inverse
/// would show up here as a joint that drives the body harder instead of less,
/// which is exactly what the Stage 6 sweep found one type over.
static void test_parallel_joint_heavy_pair_goes_quiet( void )
{
	printf( "parallel joint on a very heavy pair applies nothing rather than misbehaving\n" );

	b3WorldId worldId = makeStepWorld( V( 0, 0, 0 ) );
	b3World* world = b3GetWorldFromId( worldId );

	// A 6 m box at the default density: inertia well past the 1000 kg m^2 the
	// determinant threshold sits at, so the 2x2 underflows.
	b3BoxHull hull;
	b3BodyId anchor, body;
	b3JointId jointId = uprightRig( worldId, 3.0, 4.0, 1.0, 5000.0, &hull, &anchor, &body );
	validate( world );

	const double spin = 1.0;
	b3Body_SetAngularVelocity( body, V( spin, 0, 0 ) );

	for ( int i = 0; i < 120; ++i )
	{
		b3World_Step( worldId, 4 );
	}
	validate( world );

	b3Vec3 w = b3Body_GetAngularVelocity( body );
	b3Vec3 torque = b3Joint_GetConstraintTorque( jointId );
	double magnitude = sqrt( b3fToDouble( torque.x ) * b3fToDouble( torque.x ) +
							 b3fToDouble( torque.y ) * b3fToDouble( torque.y ) +
							 b3fToDouble( torque.z ) * b3fToDouble( torque.z ) );

	printf( "  spin %.2f -> %.4f rad/s, torque magnitude %.4f\n", spin, b3fToDouble( w.x ), magnitude );

	// The spin must not *grow*. A sign-inverted or wrapped effective mass is a
	// source rather than a sink, and that is the failure this pins -- the
	// Stage 6 rolling-resistance wrap took 10.4 rad/s to 44.6 the same way.
	check( "a heavy pair is never driven faster than it started", fabs( b3fToDouble( w.x ) ) <= spin * 1.05 );

	// And the body is still there rather than flung: with no linear constraint
	// the only thing that can go wrong is the orientation, so a finite,
	// bounded angular velocity is the whole statement of health.
	check( "and its angular velocity stays finite and small", fabs( b3fToDouble( w.x ) ) < 10.0 );

	b3DestroyWorld( worldId );
}

/// Every accessor round-trips, the negative budget clamps, and the joint
/// survives a sleep cycle.
static void test_parallel_joint_sleep_and_accessors( void )
{
	printf( "parallel joint through sleep, and accessor round-trips\n" );

	b3WorldId worldId = makeStepWorld( V( 0, 0, 0 ) );
	b3World* world = b3GetWorldFromId( worldId );

	b3BodyDef anchorDef = b3DefaultBodyDef();
	anchorDef.type = b3_staticBody;
	b3BodyId anchor = b3CreateBody( worldId, &anchorDef );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	b3BodyId body = b3CreateBody( worldId, &bodyDef );

	b3BoxHull hull = b3MakeBoxHull( b3fFromDouble( 0.25 ), b3fFromDouble( 0.25 ), b3fFromDouble( 0.25 ) );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3CreateHullShape( body, &shapeDef, &hull.base );

	b3ParallelJointDef def = b3DefaultParallelJointDef();
	def.base.bodyIdA = anchor;
	def.base.bodyIdB = body;
	def.maxTorque = b3fFromDouble( 10.0 );
	b3JointId jointId = b3CreateParallelJoint( worldId, &def );
	validate( world );

	// The default def's spring, which unlike every other joint's has no enable
	// flag -- a zero frequency is how it is turned off.
	expect( "default hertz", b3fToDouble( b3ParallelJoint_GetSpringHertz( jointId ) ), 1.0, 1e-3 );
	expect( "default damping ratio", b3fToDouble( b3ParallelJoint_GetSpringDampingRatio( jointId ) ), 1.0, 1e-3 );

	b3ParallelJoint_SetSpringHertz( jointId, b3fFromDouble( 3.5 ) );
	expect( "hertz round-trips", b3fToDouble( b3ParallelJoint_GetSpringHertz( jointId ) ), 3.5, 1e-3 );

	b3ParallelJoint_SetSpringDampingRatio( jointId, b3fFromDouble( 2.25 ) );
	expect( "damping ratio round-trips", b3fToDouble( b3ParallelJoint_GetSpringDampingRatio( jointId ) ), 2.25, 1e-3 );

	b3ParallelJoint_SetMaxTorque( jointId, b3fFromDouble( 7.5 ) );
	expect( "max torque round-trips", b3fToDouble( b3ParallelJoint_GetMaxTorque( jointId ) ), 7.5, 1e-3 );

	// Clamped rather than asserted, matching every other joint's budget.
	b3ParallelJoint_SetMaxTorque( jointId, b3fFromDouble( -4.0 ) );
	checkInt( "a negative torque budget clamps to zero", b3Raw( b3ParallelJoint_GetMaxTorque( jointId ) ), 0 );
	b3ParallelJoint_SetMaxTorque( jointId, b3fFromDouble( 7.5 ) );

	int steps = 0;
	while ( steps < 1200 && b3Body_IsAwake( body ) )
	{
		b3World_Step( worldId, 4 );
		steps += 1;
	}
	validate( world );

	printf( "  slept after %d steps\n", steps );
	check( "a body held upright goes to sleep", steps < 1200 );
	check( "and the joint is still valid", b3Joint_IsValid( jointId ) );

	// The accessors must still answer on a sleeping joint -- they read the sim
	// wherever it currently lives, and a sleeping joint has moved solver set.
	expect( "max torque survives the move to a sleeping set",
			b3fToDouble( b3ParallelJoint_GetMaxTorque( jointId ) ), 7.5, 1e-3 );

	b3DestroyJoint( jointId, true );
	validate( world );
	check( "a destroyed parallel joint is not valid", b3Joint_IsValid( jointId ) == false );

	b3DestroyWorld( worldId );
}

// -------------------------------------------------------------------------
// Phase 6 Stage 7: the wheel joint
// -------------------------------------------------------------------------

/// A static anchor and a dynamic wheel on a **vertical** suspension.
///
/// The suspension travels along frame A's x, so a vertical suspension needs
/// both frames rotated to put x onto +y -- exactly what `sliderRail` does for a
/// vertical slider, and the single most common way to get this joint wrong.
/// `b3WheelJoint_GetSuspensionTranslation` then reads directly as height.
static b3JointId wheelRig( b3WorldId worldId, b3Vec3 startPos, b3BoxHull* hull, b3WheelJointDef* defOut,
						   b3BodyId* anchorOut, b3BodyId* wheelOut )
{
	b3BodyDef anchorDef = b3DefaultBodyDef();
	anchorDef.type = b3_staticBody;
	anchorDef.position = V( 0, 0, 0 );
	b3BodyId anchor = b3CreateBody( worldId, &anchorDef );

	b3BodyDef wheelDef = b3DefaultBodyDef();
	wheelDef.type = b3_dynamicBody;
	wheelDef.position = startPos;
	wheelDef.enableSleep = false;
	b3BodyId wheel = b3CreateBody( worldId, &wheelDef );

	*hull = b3MakeBoxHull( b3fFromDouble( 0.2 ), b3fFromDouble( 0.2 ), b3fFromDouble( 0.2 ) );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3CreateHullShape( wheel, &shapeDef, &hull->base );

	// x onto +y: the travel axis points up.
	b3Quat frameQ = b3MakeQuatFromAxisAngle( b3Vec3_axisZ, (b3a)B3_BRAD_HALF_PI );

	b3WheelJointDef def = b3DefaultWheelJointDef();
	def.base.bodyIdA = anchor;
	def.base.bodyIdB = wheel;
	def.base.localFrameA.q = frameQ;
	def.base.localFrameB.q = frameQ;
	def.base.localFrameB.p = b3Neg( startPos );

	*defOut = def;
	*anchorOut = anchor;
	*wheelOut = wheel;
	return b3CreateWheelJoint( worldId, &def );
}

/// The stage's headline closed form: a sprung wheel sags by `g / (2*pi*f)^2`.
///
/// Mass-independent, so it is a statement about the spring and nothing else --
/// the same closed form the prismatic's spring test uses, on this joint's
/// linear half.
static void test_wheel_joint_suspension_sag( void )
{
	printf( "wheel joint suspension sags by g/(2 pi f)^2\n" );

	const double g = 9.8;
	const double hertz = 1.5;
	b3WorldId worldId = makeStepWorld( V( 0, -g, 0 ) );
	b3World* world = b3GetWorldFromId( worldId );

	b3BoxHull hull;
	b3WheelJointDef def;
	b3BodyId anchor, wheel;
	b3JointId jointId = wheelRig( worldId, V( 0, 0, 0 ), &hull, &def, &anchor, &wheel );

	b3WheelJoint_SetSuspensionHertz( jointId, b3fFromDouble( hertz ) );
	b3WheelJoint_SetSuspensionDampingRatio( jointId, b3fFromDouble( 2.0 ) );
	validate( world );

	for ( int i = 0; i < 300; ++i )
	{
		b3World_Step( worldId, 4 );
	}
	validate( world );

	double sag = b3fToDouble( b3WheelJoint_GetSuspensionTranslation( jointId ) );
	double predicted = -g / ( 4.0 * M_PI * M_PI * hertz * hertz );

	printf( "  sag at %.1f Hz: %.5f m, g/(2*pi*f)^2 predicts %.5f (ratio %.4f)\n", hertz, sag, predicted,
			sag / predicted );

	expect( "the sag matches the closed form", sag, predicted, 0.012 );

	b3DestroyWorld( worldId );
}

/// **The test that catches upstream defect 4**, the permuted reaction force.
///
/// A wheel hanging at rest on a vertical suspension carries its own weight
/// through the suspension axis, which is frame A's x -- and this rig points
/// that at world **+y**. So the reaction must be `(0, m*g, 0)` with nothing
/// across it.
///
/// Upstream assembles `(perp.x, perp.y, axial)` where the solve applies
/// `(axial, perp.x, perp.y)`, a cyclic permutation left over from an older
/// convention. That puts the weight along frame A's **z**, which this rig
/// leaves pointing at world z -- a different axis, same magnitude. `run_pair`
/// compares the reaction by magnitude only, so it agrees with both orderings to
/// the last bit and cannot see this. Only a direction test can.
static void test_wheel_joint_reaction_direction( void )
{
	printf( "wheel joint reaction force points along the suspension axis\n" );

	const double g = 9.8;
	b3WorldId worldId = makeStepWorld( V( 0, -g, 0 ) );
	b3World* world = b3GetWorldFromId( worldId );

	b3BoxHull hull;
	b3WheelJointDef def;
	b3BodyId anchor, wheel;
	b3JointId jointId = wheelRig( worldId, V( 0, 0, 0 ), &hull, &def, &anchor, &wheel );

	b3WheelJoint_SetSuspensionHertz( jointId, b3fFromDouble( 4.0 ) );
	b3WheelJoint_SetSuspensionDampingRatio( jointId, b3fFromDouble( 2.0 ) );
	validate( world );

	for ( int i = 0; i < 300; ++i )
	{
		b3World_Step( worldId, 4 );
	}
	validate( world );

	double mass = b3fToDouble( b3Body_GetMass( wheel ) );
	double weight = mass * g;

	b3Vec3 force = b3Joint_GetConstraintForce( jointId );
	double fx = b3fToDouble( force.x );
	double fy = b3fToDouble( force.y );
	double fz = b3fToDouble( force.z );

	printf( "  mass %.4f kg, weight %.4f N; reaction (%.4f %.4f %.4f)\n", mass, weight, fx, fy, fz );

	// Along +y, which is where frame A's x points in this rig.
	expect( "the reaction carries the weight along the suspension axis", fy, weight, weight * 0.06 + 0.05 );

	// And nothing across it. Upstream's permutation puts the whole weight into
	// z, so this is the half that discriminates.
	check( "and nothing across it", fabs( fx ) < weight * 0.15 + 0.05 && fabs( fz ) < weight * 0.15 + 0.05 );

	b3DestroyWorld( worldId );
}

/// **The test that catches upstream defect 1**, a length used as an impulse.
///
/// `b3GetWheelJointForce` upstream adds `lowerSuspensionLimit` -- a distance in
/// metres -- where `lowerSuspensionImpulse` is meant, and gets the upper term's
/// sign wrong as well. At the default lower limit of zero it is invisible,
/// which is why this scene sets a non-zero one: at `inv_h` = 240, a lower limit
/// of -0.3 fabricates about 72 N of reaction that no body ever felt.
///
/// This is the one wheel defect `run_pair` *could* see, since it changes a
/// magnitude -- which is exactly why every wheel scene there is kept at a lower
/// limit of zero, so the harness stays clean and this test is what settles it.
static void test_wheel_joint_reaction_ignores_limits( void )
{
	printf( "wheel joint reaction reports impulses, not limit distances\n" );

	const double g = 9.8;
	b3WorldId worldId = makeStepWorld( V( 0, -g, 0 ) );
	b3World* world = b3GetWorldFromId( worldId );

	b3BoxHull hull;
	b3WheelJointDef def;
	b3BodyId anchor, wheel;
	b3JointId jointId = wheelRig( worldId, V( 0, 0, 0 ), &hull, &def, &anchor, &wheel );

	b3WheelJoint_SetSuspensionHertz( jointId, b3fFromDouble( 4.0 ) );
	b3WheelJoint_SetSuspensionDampingRatio( jointId, b3fFromDouble( 2.0 ) );

	// A non-zero lower limit, well clear of where the wheel settles, so it is
	// never engaged and contributes no impulse at all.
	b3WheelJoint_EnableSuspensionLimit( jointId, true );
	b3WheelJoint_SetSuspensionLimits( jointId, b3fFromDouble( -0.3 ), b3fFromDouble( 0.3 ) );
	validate( world );

	for ( int i = 0; i < 300; ++i )
	{
		b3World_Step( worldId, 4 );
	}
	validate( world );

	double mass = b3fToDouble( b3Body_GetMass( wheel ) );
	double weight = mass * g;

	b3Vec3 force = b3Joint_GetConstraintForce( jointId );
	double magnitude = sqrt( b3fToDouble( force.x ) * b3fToDouble( force.x ) +
							 b3fToDouble( force.y ) * b3fToDouble( force.y ) +
							 b3fToDouble( force.z ) * b3fToDouble( force.z ) );

	double translation = b3fToDouble( b3WheelJoint_GetSuspensionTranslation( jointId ) );

	printf( "  settled at %.5f m (limits -0.3 .. 0.3), weight %.4f N, reaction %.4f N\n", translation, weight,
			magnitude );

	check( "the wheel settles clear of both limits", translation > -0.3 && translation < 0.3 );

	// Upstream would report weight + 0.3 * 240 = weight + 72 N here.
	expect( "the reaction is the weight, not the weight plus a limit distance", magnitude, weight,
			weight * 0.06 + 0.05 );

	b3DestroyWorld( worldId );
}

/// **The test that catches upstream defect 3**, the spin torque's axis.
///
/// The spin impulse is applied along frame **B's** z and upstream reports it
/// along frame **A's** z. Those coincide only while the wheel points straight
/// ahead, so this steers it a long way off and checks which one the readout
/// followed.
static void test_wheel_joint_spin_torque_axis( void )
{
	printf( "wheel joint spin torque follows body B's axis, not body A's\n" );

	b3WorldId worldId = makeStepWorld( V( 0, 0, 0 ) );
	b3World* world = b3GetWorldFromId( worldId );

	b3BoxHull hull;
	b3WheelJointDef def;
	b3BodyId anchor, wheel;
	b3JointId jointId = wheelRig( worldId, V( 0, 0, 0 ), &hull, &def, &anchor, &wheel );

	b3WheelJoint_EnableSpinMotor( jointId, true );
	b3WheelJoint_SetSpinMotorSpeed( jointId, b3fFromDouble( 20.0 ) );
	b3WheelJoint_SetMaxSpinTorque( jointId, b3fFromDouble( 2.0 ) );

	// Steer a long way off straight, so A's z and B's z are far apart.
	b3WheelJoint_EnableSteering( jointId, true );
	b3WheelJoint_SetSteeringHertz( jointId, b3fFromDouble( 6.0 ) );
	b3WheelJoint_SetSteeringDampingRatio( jointId, b3fFromDouble( 1.0 ) );
	b3WheelJoint_SetMaxSteeringTorque( jointId, b3fFromDouble( 500.0 ) );
	b3WheelJoint_SetTargetSteeringAngle( jointId, (b3a)( B3_BRAD_PI / 6 ) );
	validate( world );

	for ( int i = 0; i < 240; ++i )
	{
		b3World_Step( worldId, 4 );
	}
	validate( world );

	b3a steer = b3WheelJoint_GetSteeringAngle( jointId );
	printf( "  steered to %d brads (target %d)\n", (int)steer, (int)( B3_BRAD_PI / 6 ) );
	check( "the wheel actually steered off straight", steer > 1200 );

	b3Quat qB = b3Body_GetRotation( wheel );
	b3Vec3 bz = b3RotateVector( qB, b3Vec3_axisZ );

	b3Vec3 torque = b3Joint_GetConstraintTorque( jointId );
	double tMag = sqrt( b3fToDouble( torque.x ) * b3fToDouble( torque.x ) +
						b3fToDouble( torque.y ) * b3fToDouble( torque.y ) +
						b3fToDouble( torque.z ) * b3fToDouble( torque.z ) );

	check( "the joint is applying a torque at all", tMag > 0.01 );

	// The spin component projected onto B's z must be a real share of the
	// total. Upstream, reporting along A's z, loses it as the wheel steers.
	double alongB = fabs( b3fToDouble( torque.x ) * b3fToDouble( bz.x ) + b3fToDouble( torque.y ) * b3fToDouble( bz.y ) +
						  b3fToDouble( torque.z ) * b3fToDouble( bz.z ) );

	printf( "  torque magnitude %.4f, component along B's z %.4f\n", tMag, alongB );
	check( "the spin torque is reported along B's z", alongB > 0.15 * tMag );

	b3DestroyWorld( worldId );
}

/// The steering limit stops the steer, and the two torque readouts answer the
/// two different questions they are named for.
///
/// This scene was written expecting to catch a defect and instead **settled a
/// design question against the first attempt**, which is worth recording. The
/// port briefly had `GetSteeringTorque` report `spring + lower - upper`, on the
/// grounds that upstream reporting the spring alone ignores the limits. But a
/// wheel resting on a steering stop is in *equilibrium*: the spring pushes into
/// the limit and the limit pushes back, so the sum measured 0.0037 N-m while
/// the spring was spending its whole 500 N-m budget. The combined number says
/// "nothing is happening" about a joint under maximum load, so the port matches
/// upstream here and says why in wheel_joint.c.
///
/// The limits are not lost: `b3Joint_GetConstraintTorque` includes them, and
/// that is the query for the net torque about every axis.
static void test_wheel_joint_steering_torque_on_a_limit( void )
{
	printf( "wheel joint reports the torque a steering limit is carrying\n" );

	b3WorldId worldId = makeStepWorld( V( 0, 0, 0 ) );
	b3World* world = b3GetWorldFromId( worldId );

	b3BoxHull hull;
	b3WheelJointDef def;
	b3BodyId anchor, wheel;
	b3JointId jointId = wheelRig( worldId, V( 0, 0, 0 ), &hull, &def, &anchor, &wheel );

	b3WheelJoint_EnableSteering( jointId, true );
	b3WheelJoint_SetSteeringHertz( jointId, b3fFromDouble( 6.0 ) );
	b3WheelJoint_SetSteeringDampingRatio( jointId, b3fFromDouble( 1.0 ) );
	b3WheelJoint_SetMaxSteeringTorque( jointId, b3fFromDouble( 500.0 ) );

	// Ask for far more steer than the limit allows, so the limit carries the
	// difference and the spring sits satisfied against it.
	b3WheelJoint_EnableSteeringLimit( jointId, true );
	b3WheelJoint_SetSteeringLimits( jointId, (b3a)( -B3_BRAD_PI / 8 ), (b3a)( B3_BRAD_PI / 8 ) );
	b3WheelJoint_SetTargetSteeringAngle( jointId, (b3a)( B3_BRAD_PI / 2 ) );
	validate( world );

	for ( int i = 0; i < 300; ++i )
	{
		b3World_Step( worldId, 4 );
	}
	validate( world );

	b3a steer = b3WheelJoint_GetSteeringAngle( jointId );
	double springTorque = b3fToDouble( b3WheelJoint_GetSteeringTorque( jointId ) );

	b3Vec3 netTorque = b3Joint_GetConstraintTorque( jointId );
	double netMag = sqrt( b3fToDouble( netTorque.x ) * b3fToDouble( netTorque.x ) +
						  b3fToDouble( netTorque.y ) * b3fToDouble( netTorque.y ) +
						  b3fToDouble( netTorque.z ) * b3fToDouble( netTorque.z ) );

	printf( "  held at %d brads against a limit of %d; spring torque %.4f, net %.4f\n", (int)steer,
			(int)( B3_BRAD_PI / 8 ), springTorque, netMag );

	// The limit stops the steer well short of the 90-degree target it is asked
	// for -- which is the behaviour under test.
	check( "the steering stopped at its limit", steer < (b3a)( B3_BRAD_PI / 8 ) + 600 );
	check( "and well short of the target it was given", steer < (b3a)( B3_BRAD_PI / 4 ) );

	// The spring is spending real effort holding against the stop.
	check( "the spring reports the effort it is spending", fabs( springTorque ) > 0.5 );

	// And the *net* is small, because the limit is cancelling the spring. That
	// is the equilibrium this scene is in, and it is why the two readouts
	// cannot be the same number.
	check( "while the net torque is small, the two balancing", netMag < fabs( springTorque ) );

	b3DestroyWorld( worldId );
}

/// The suspension travel stops, driven onto from both directions.
static void test_wheel_joint_suspension_limits( void )
{
	printf( "wheel joint suspension limits stop the travel\n" );

	const double g = 9.8;
	b3WorldId worldId = makeStepWorld( V( 0, -g, 0 ) );
	b3World* world = b3GetWorldFromId( worldId );

	b3BoxHull hull;
	b3WheelJointDef def;
	b3BodyId anchor, wheel;
	b3JointId jointId = wheelRig( worldId, V( 0, 0, 0 ), &hull, &def, &anchor, &wheel );

	// Spring off, so only the limit stops it: a limit test with a spring in the
	// scene measures the spring.
	b3WheelJoint_EnableSuspensionSpring( jointId, false );
	b3WheelJoint_EnableSuspensionLimit( jointId, true );
	b3WheelJoint_SetSuspensionLimits( jointId, b3fFromDouble( -0.5 ), b3fFromDouble( 0.5 ) );
	validate( world );

	for ( int i = 0; i < 300; ++i )
	{
		b3World_Step( worldId, 4 );
	}
	validate( world );

	double lower = b3fToDouble( b3WheelJoint_GetSuspensionTranslation( jointId ) );
	printf( "  fell to the lower stop: %.5f (limit -0.5)\n", lower );
	expect( "gravity drives it onto the lower stop", lower, -0.5, 0.03 );

	// Now push it the other way with a strong spring pulling up past the stop.
	b3WheelJoint_EnableSuspensionSpring( jointId, true );
	b3WheelJoint_SetSuspensionHertz( jointId, b3fFromDouble( 8.0 ) );
	b3WheelJoint_SetSuspensionDampingRatio( jointId, b3fFromDouble( 2.0 ) );
	b3WheelJoint_SetSuspensionLimits( jointId, b3fFromDouble( -0.5 ), b3fFromDouble( -0.05 ) );
	b3Body_SetAwake( wheel, true );

	for ( int i = 0; i < 300; ++i )
	{
		b3World_Step( worldId, 4 );
	}
	validate( world );

	double upper = b3fToDouble( b3WheelJoint_GetSuspensionTranslation( jointId ) );
	printf( "  sprung up onto the upper stop: %.5f (limit -0.05)\n", upper );
	expect( "the spring drives it onto the upper stop", upper, -0.05, 0.03 );

	b3DestroyWorld( worldId );
}

/// The spin motor reaches its commanded speed, and saturates when starved.
static void test_wheel_joint_spin_motor( void )
{
	printf( "wheel joint spin motor drives, and saturates against its budget\n" );

	b3WorldId worldId = makeStepWorld( V( 0, 0, 0 ) );
	b3World* world = b3GetWorldFromId( worldId );

	b3BoxHull hull;
	b3WheelJointDef def;
	b3BodyId anchor, wheel;
	b3JointId jointId = wheelRig( worldId, V( 0, 0, 0 ), &hull, &def, &anchor, &wheel );

	const double target = 12.0;
	b3WheelJoint_EnableSpinMotor( jointId, true );
	b3WheelJoint_SetSpinMotorSpeed( jointId, b3fFromDouble( target ) );
	b3WheelJoint_SetMaxSpinTorque( jointId, b3fFromDouble( 50.0 ) );
	validate( world );

	for ( int i = 0; i < 240; ++i )
	{
		b3World_Step( worldId, 4 );
	}
	validate( world );

	double reached = b3fToDouble( b3WheelJoint_GetSpinSpeed( jointId ) );
	printf( "  commanded %.1f rad/s, reached %.4f\n", target, reached );
	expect( "the motor reaches its commanded spin", reached, target, 0.6 );

	// Now starve it and check the torque saturates at the bound rather than
	// exceeding it.
	const double bound = 0.25;
	b3WheelJoint_SetMaxSpinTorque( jointId, b3fFromDouble( bound ) );
	b3WheelJoint_SetSpinMotorSpeed( jointId, b3fFromDouble( -target ) );
	b3Body_SetAwake( wheel, true );

	double peak = 0.0;
	for ( int i = 0; i < 60; ++i )
	{
		b3World_Step( worldId, 4 );
		double t = fabs( b3fToDouble( b3WheelJoint_GetSpinTorque( jointId ) ) );
		if ( t > peak )
		{
			peak = t;
		}
	}
	validate( world );

	printf( "  starved to %.2f N-m: peak reported torque %.4f\n", bound, peak );
	check( "the spin torque never exceeds its budget", peak <= bound * 1.05 );
	check( "and the budget is actually reached", peak > bound * 0.9 );

	b3DestroyWorld( worldId );
}

/// The degeneracy of wheel_joint.c's @section degenerate, swept rather than
/// spot-checked.
///
/// As the wheel tips toward its own suspension axis, `den0 = 1 - dot(A.cx,
/// B.cz)^2` goes to zero: upstream's steering axis grows without bound and the
/// collinearity mass runs into `b3RcpWide`'s floor, where it **saturates rather
/// than reporting failure**. The port guards both at `den0 < 1/64`.
///
/// A spot check either side of a threshold would not have found the indefinite
/// 2x2 in Stage 6 nor the half-turn overflow in Step 2, so this sweeps the
/// whole approach and asserts what must hold everywhere: nothing drives these
/// scenes -- no gravity, no motor, no initial velocity -- so **any** speed at
/// all is the joint inventing energy, and a saturated effective mass shows up
/// here as a large number.
static void test_wheel_joint_degenerate_sweep( void )
{
	printf( "wheel joint stays finite as it tips onto its own suspension axis\n" );

	double worstSpeed = 0.0;
	int scenes = 0;

	// From square-on to fully tipped, straight through the 7.2-degree floor.
	for ( int deg = 90; deg >= 0; deg -= 6 )
	{
		b3WorldId worldId = makeStepWorld( V( 0, 0, 0 ) );
		b3World* world = b3GetWorldFromId( worldId );

		b3BoxHull hull;
		b3WheelJointDef def;
		b3BodyId anchor, wheel;
		b3JointId jointId = wheelRig( worldId, V( 0, 0, 0 ), &hull, &def, &anchor, &wheel );

		b3WheelJoint_EnableSteering( jointId, true );
		b3WheelJoint_SetSteeringHertz( jointId, b3fFromDouble( 4.0 ) );
		b3WheelJoint_SetSteeringDampingRatio( jointId, b3fFromDouble( 1.0 ) );
		b3WheelJoint_SetMaxSteeringTorque( jointId, b3fFromDouble( 100.0 ) );

		// Tip the wheel's spin axis toward the suspension axis. At 0 degrees
		// the two are collinear and den0 is zero.
		b3a tip = (b3a)( (double)deg * 32768.0 / 360.0 );
		b3Body_SetTransform( wheel, V( 0, 0, 0 ), b3MakeQuatFromAxisAngle( b3Vec3_axisY, tip ) );
		validate( world );

		for ( int i = 0; i < 120; ++i )
		{
			b3World_Step( worldId, 4 );
		}
		validate( world );

		b3Vec3 w = b3Body_GetAngularVelocity( wheel );
		b3Vec3 v = b3Body_GetLinearVelocity( wheel );
		double speed = sqrt( b3fToDouble( w.x ) * b3fToDouble( w.x ) + b3fToDouble( w.y ) * b3fToDouble( w.y ) +
							 b3fToDouble( w.z ) * b3fToDouble( w.z ) ) +
					   sqrt( b3fToDouble( v.x ) * b3fToDouble( v.x ) + b3fToDouble( v.y ) * b3fToDouble( v.y ) +
							 b3fToDouble( v.z ) * b3fToDouble( v.z ) );

		if ( speed > worstSpeed )
		{
			worstSpeed = speed;
		}
		scenes += 1;

		B3_UNUSED( jointId );
		b3DestroyWorld( worldId );
	}

	printf( "  %d tip angles from 90 to 0 degrees: worst residual speed %.4f\n", scenes, worstSpeed );
	check( "the joint never launches the wheel at any tip angle", worstSpeed < 1.0 );
}

/// Accessor round-trips, both sorts, and survival through a sleep cycle.
static void test_wheel_joint_sleep_and_accessors( void )
{
	printf( "wheel joint through sleep, and accessor round-trips\n" );

	b3WorldId worldId = makeStepWorld( V( 0, -9.8, 0 ) );
	b3World* world = b3GetWorldFromId( worldId );

	b3BodyDef anchorDef = b3DefaultBodyDef();
	anchorDef.type = b3_staticBody;
	b3BodyId anchor = b3CreateBody( worldId, &anchorDef );

	b3BodyDef wheelDef = b3DefaultBodyDef();
	wheelDef.type = b3_dynamicBody;
	b3BodyId wheel = b3CreateBody( worldId, &wheelDef );

	b3BoxHull hull = b3MakeBoxHull( b3fFromDouble( 0.2 ), b3fFromDouble( 0.2 ), b3fFromDouble( 0.2 ) );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3CreateHullShape( wheel, &shapeDef, &hull.base );

	b3Quat frameQ = b3MakeQuatFromAxisAngle( b3Vec3_axisZ, (b3a)B3_BRAD_HALF_PI );
	b3WheelJointDef def = b3DefaultWheelJointDef();
	def.base.bodyIdA = anchor;
	def.base.bodyIdB = wheel;
	def.base.localFrameA.q = frameQ;
	def.base.localFrameB.q = frameQ;
	b3JointId jointId = b3CreateWheelJoint( worldId, &def );
	validate( world );

	// The default def's four non-zero fields, kept from upstream.
	check( "the suspension spring is on by default", b3WheelJoint_IsSuspensionSpringEnabled( jointId ) );
	expect( "default suspension hertz", b3fToDouble( b3WheelJoint_GetSuspensionHertz( jointId ) ), 1.0, 1e-3 );
	expect( "default suspension damping", b3fToDouble( b3WheelJoint_GetSuspensionDampingRatio( jointId ) ), 0.7, 1e-3 );
	expect( "default steering hertz", b3fToDouble( b3WheelJoint_GetSteeringHertz( jointId ) ), 1.0, 1e-3 );
	expect( "default steering damping", b3fToDouble( b3WheelJoint_GetSteeringDampingRatio( jointId ) ), 0.7, 1e-3 );

	// And the zeros that are answers rather than omissions.
	checkInt( "default max spin torque is zero", b3Raw( b3WheelJoint_GetMaxSpinTorque( jointId ) ), 0 );
	checkInt( "default max steering torque is zero", b3Raw( b3WheelJoint_GetMaxSteeringTorque( jointId ) ), 0 );
	checkInt( "default target steering angle is straight ahead", b3WheelJoint_GetTargetSteeringAngle( jointId ), 0 );

	b3WheelJoint_SetSuspensionHertz( jointId, b3fFromDouble( 2.5 ) );
	expect( "suspension hertz round-trips", b3fToDouble( b3WheelJoint_GetSuspensionHertz( jointId ) ), 2.5, 1e-3 );

	b3WheelJoint_SetSpinMotorSpeed( jointId, b3fFromDouble( -3.25 ) );
	expect( "spin speed round-trips", b3fToDouble( b3WheelJoint_GetSpinMotorSpeed( jointId ) ), -3.25, 1e-3 );

	// Both budgets clamp a negative to zero, as every other joint's do.
	b3WheelJoint_SetMaxSpinTorque( jointId, b3fFromDouble( -5.0 ) );
	checkInt( "a negative spin budget clamps to zero", b3Raw( b3WheelJoint_GetMaxSpinTorque( jointId ) ), 0 );
	b3WheelJoint_SetMaxSteeringTorque( jointId, b3fFromDouble( -5.0 ) );
	checkInt( "a negative steering budget clamps to zero", b3Raw( b3WheelJoint_GetMaxSteeringTorque( jointId ) ), 0 );

	// Both ranges sort, so the wrong order gives the range the caller meant.
	b3WheelJoint_SetSuspensionLimits( jointId, b3fFromDouble( 0.4 ), b3fFromDouble( -0.2 ) );
	expect( "suspension limits sort: lower", b3fToDouble( b3WheelJoint_GetLowerSuspensionLimit( jointId ) ), -0.2, 1e-3 );
	expect( "suspension limits sort: upper", b3fToDouble( b3WheelJoint_GetUpperSuspensionLimit( jointId ) ), 0.4, 1e-3 );

	b3WheelJoint_SetSteeringLimits( jointId, (b3a)4000, (b3a)-1000 );
	checkInt( "steering limits sort: lower", b3WheelJoint_GetLowerSteeringLimit( jointId ), -1000 );
	checkInt( "steering limits sort: upper", b3WheelJoint_GetUpperSteeringLimit( jointId ), 4000 );

	int steps = 0;
	while ( steps < 1200 && b3Body_IsAwake( wheel ) )
	{
		b3World_Step( worldId, 4 );
		steps += 1;
	}
	validate( world );

	printf( "  slept after %d steps\n", steps );
	check( "a settled wheel goes to sleep", steps < 1200 );
	check( "and the joint is still valid", b3Joint_IsValid( jointId ) );

	expect( "accessors still answer on a sleeping joint", b3fToDouble( b3WheelJoint_GetSuspensionHertz( jointId ) ), 2.5,
			1e-3 );

	b3DestroyJoint( jointId, true );
	validate( world );
	check( "a destroyed wheel joint is not valid", b3Joint_IsValid( jointId ) == false );

	b3DestroyWorld( worldId );
}

static void test_spherical_joint_ragdoll( void )
{
	// The shape default, and the light case that found the overflow.
	ragdollScene( 1000.0 );
	ragdollScene( 1.0 );
}

// =========================================================================
// Separation queries -- Stage 7 Step 4
// =========================================================================
//
// **These are host-only on purpose.** run_pair is not given a separation
// scenario and will not be, because the port and the reference disagree by
// construction on exactly the joints whose separation is interesting: upstream
// measures a slider's off-rail offset along one arbitrary perpendicular where
// the constraint removes two. A lockstep comparison would report that
// disagreement as a divergence every single run, and the port would spend
// forever re-defending a fix it made deliberately. Closed forms instead -- the
// same treatment, for the same reason, that
// test_prismatic_joint_reaction_direction already gets.
//
// Both queries read the body transforms rather than accumulated impulses, so
// they are meaningful without stepping. Every test here places the bodies with
// b3Body_SetTransform and reads the answer directly, which is what makes the
// expected value an exact closed form rather than something a solver settled to.

/// **The test that proves the off-rail fix**, and the reason it is a sweep.
///
/// A point-to-line constraint removes two degrees of freedom, so the separation
/// of an off-rail slider must be the *perpendicular distance* to the rail --
/// which depends only on how far off the rail the body is, never on which way it
/// went. Upstream projects onto `b3Perp( axisA )`, a single arbitrary
/// perpendicular, so its answer varies as |cos| of the offset direction and
/// falls to **exactly zero** a quarter turn away from whatever b3Perp picked.
///
/// Sweeping the offset direction all the way around the rail is what turns that
/// from an argument into a measurement: the port's answer must be flat, and
/// upstream's would trace a rectified cosine through two zeros. A spot check at
/// one angle proves nothing, because upstream is *correct* at the angle b3Perp
/// happens to choose.
static void test_joint_linear_separation_is_perpendicular_distance( void )
{
	printf( "slider separation is the true perpendicular distance, all the way around\n" );

	b3WorldId worldId = makeStepWorld( V( 0, 0, 0 ) );
	b3World* world = b3GetWorldFromId( worldId );

	// The rail runs along world x: frame A's x is already x, so no rotation.
	b3BoxHull hull = b3MakeBoxHull( b3fFromDouble( 0.15 ), b3fFromDouble( 0.15 ), b3fFromDouble( 0.15 ) );
	b3PrismaticJointDef def;
	b3BodyId anchor, slider;
	b3JointId jointId = sliderRail( worldId, b3Quat_identity, V( 0, 0, 0 ), &hull, &def, &anchor, &slider );
	validate( world );

	const double offset = 0.4;
	double worst = 0.0;
	double smallest = 1e9;

	for ( int i = 0; i < 16; ++i )
	{
		// Around the rail in the y-z plane, plus a metre of travel *along* it to
		// prove the axial component is removed rather than mixed in.
		const double theta = ( 2.0 * 3.14159265358979323846 * i ) / 16.0;
		b3Body_SetTransform( slider, V( 1.0, offset * cos( theta ), offset * sin( theta ) ), b3Quat_identity );

		const double got = b3fToDouble( b3Joint_GetLinearSeparation( jointId ) );
		if ( fabs( got - offset ) > worst )
		{
			worst = fabs( got - offset );
		}
		if ( got < smallest )
		{
			smallest = got;
		}
	}

	printf( "  16 offset directions at %.2f m: worst error %.5f, smallest reading %.5f\n", offset, worst, smallest );

	// One Q12 quantum is 0.000244; the tolerance is a few of them, for the
	// narrowing in b3Dot plus the root.
	check( "the perpendicular distance is flat around the rail", worst < 0.002 );

	// The assertion that upstream fails outright. Its smallest reading over this
	// sweep is 0.
	check( "and never collapses to zero on any bearing", smallest > 0.39 );

	// On the rail exactly, the answer is zero -- the query is not merely
	// reporting |dp|.
	b3Body_SetTransform( slider, V( 2.5, 0, 0 ), b3Quat_identity );
	expect( "a slider on its rail reports no separation", b3fToDouble( b3Joint_GetLinearSeparation( jointId ) ), 0.0,
			0.002 );

	b3DestroyWorld( worldId );
}

/// A limit violation is a separation, and it combines with the off-rail one
/// under a single root rather than replacing it.
static void test_joint_linear_separation_limits( void )
{
	printf( "slider separation adds the limit excess in quadrature\n" );

	b3WorldId worldId = makeStepWorld( V( 0, 0, 0 ) );
	b3World* world = b3GetWorldFromId( worldId );

	b3BoxHull hull = b3MakeBoxHull( b3fFromDouble( 0.15 ), b3fFromDouble( 0.15 ), b3fFromDouble( 0.15 ) );
	b3PrismaticJointDef def;
	b3BodyId anchor, slider;
	b3JointId jointId = sliderRail( worldId, b3Quat_identity, V( 0, 0, 0 ), &hull, &def, &anchor, &slider );

	b3PrismaticJoint_EnableLimit( jointId, true );
	b3PrismaticJoint_SetLimits( jointId, b3fFromDouble( -0.5 ), b3fFromDouble( 0.5 ) );
	validate( world );

	// Inside the limits and on the rail: nothing to report.
	b3Body_SetTransform( slider, V( 0.25, 0, 0 ), b3Quat_identity );
	expect( "inside its limits a slider reports zero", b3fToDouble( b3Joint_GetLinearSeparation( jointId ) ), 0.0,
			0.002 );

	// Past the upper stop, on the rail: the excess alone.
	b3Body_SetTransform( slider, V( 0.8, 0, 0 ), b3Quat_identity );
	expect( "past the upper stop it reports the excess", b3fToDouble( b3Joint_GetLinearSeparation( jointId ) ),
			0.8 - 0.5, 0.002 );

	// Past the lower stop, on the rail.
	b3Body_SetTransform( slider, V( -0.9, 0, 0 ), b3Quat_identity );
	expect( "and past the lower stop likewise", b3fToDouble( b3Joint_GetLinearSeparation( jointId ) ), 0.9 - 0.5,
			0.002 );

	// Both at once: sqrt(0.3^2 + 0.4^2) = 0.5, which is the whole reason the two
	// terms are combined under one root rather than added.
	b3Body_SetTransform( slider, V( 0.8, 0.4, 0 ), b3Quat_identity );
	expect( "off-rail and past the stop combine in quadrature",
			b3fToDouble( b3Joint_GetLinearSeparation( jointId ) ), 0.5, 0.003 );

	b3DestroyWorld( worldId );
}

/// The types whose linear separation is just |dp|, and the ones that answer zero
/// truly rather than because nobody wrote the case.
static void test_joint_linear_separation_by_type( void )
{
	printf( "linear separation across the joint types\n" );

	b3WorldId worldId = makeStepWorld( V( 0, 0, 0 ) );
	b3World* world = b3GetWorldFromId( worldId );

	b3BodyDef anchorDef = b3DefaultBodyDef();
	anchorDef.type = b3_staticBody;
	anchorDef.position = V( 0, 0, 0 );
	b3BodyId anchor = b3CreateBody( worldId, &anchorDef );

	b3BodyDef movingDef = b3DefaultBodyDef();
	movingDef.type = b3_dynamicBody;
	movingDef.position = V( 0, 0, 0 );
	movingDef.enableSleep = false;
	b3BodyId moving = b3CreateBody( worldId, &movingDef );

	b3BoxHull hull = b3MakeBoxHull( b3fFromDouble( 0.15 ), b3fFromDouble( 0.15 ), b3fFromDouble( 0.15 ) );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3CreateHullShape( moving, &shapeDef, &hull.base );

	// 3-4-5 again, so the expected length is exact rather than a decimal.
	const b3Vec3 pulled = V( 0.3, 0.4, 0.0 );

	{
		b3RevoluteJointDef def = b3DefaultRevoluteJointDef();
		def.base.bodyIdA = anchor;
		def.base.bodyIdB = moving;
		b3JointId jointId = b3CreateRevoluteJoint( worldId, &def );

		b3Body_SetTransform( moving, pulled, b3Quat_identity );
		expect( "a hinge pulled apart reports the whole offset",
				b3fToDouble( b3Joint_GetLinearSeparation( jointId ) ), 0.5, 0.002 );
		b3DestroyJoint( jointId, true );
	}

	{
		b3SphericalJointDef def = b3DefaultSphericalJointDef();
		def.base.bodyIdA = anchor;
		def.base.bodyIdB = moving;
		b3JointId jointId = b3CreateSphericalJoint( worldId, &def );

		b3Body_SetTransform( moving, pulled, b3Quat_identity );
		expect( "a ball joint likewise", b3fToDouble( b3Joint_GetLinearSeparation( jointId ) ), 0.5, 0.002 );
		b3DestroyJoint( jointId, true );
	}

	{
		// A weld with a rigid linear axis reports the offset; give it a linear
		// spring and the offset is what it is *for*, so the answer is zero.
		b3WeldJointDef def = b3DefaultWeldJointDef();
		def.base.bodyIdA = anchor;
		def.base.bodyIdB = moving;
		b3JointId jointId = b3CreateWeldJoint( worldId, &def );

		b3Body_SetTransform( moving, pulled, b3Quat_identity );
		expect( "a rigid weld reports the offset", b3fToDouble( b3Joint_GetLinearSeparation( jointId ) ), 0.5,
				0.002 );

		b3WeldJoint_SetLinearHertz( jointId, b3fFromDouble( 5.0 ) );
		expect( "a sprung weld reports none, because stretching is its job",
				b3fToDouble( b3Joint_GetLinearSeparation( jointId ) ), 0.0, 1e-9 );
		b3DestroyJoint( jointId, true );
	}

	{
		// A distance joint with neither spring nor limit: |length - rest|.
		b3DistanceJointDef def = b3DefaultDistanceJointDef();
		def.base.bodyIdA = anchor;
		def.base.bodyIdB = moving;
		def.length = b3fFromDouble( 0.2 );
		b3JointId jointId = b3CreateDistanceJoint( worldId, &def );

		b3Body_SetTransform( moving, pulled, b3Quat_identity );
		expect( "a rigid distance joint reports the stretch past its rest length",
				b3fToDouble( b3Joint_GetLinearSeparation( jointId ) ), 0.5 - 0.2, 0.002 );

		// Spring, no limit: free to stretch, so zero.
		b3DistanceJoint_EnableSpring( jointId, true );
		expect( "a spring without limits reports none", b3fToDouble( b3Joint_GetLinearSeparation( jointId ) ), 0.0,
				1e-9 );

		// Spring with limits: only the excess past the band counts.
		b3DistanceJoint_EnableLimit( jointId, true );
		b3DistanceJoint_SetLengthRange( jointId, b3fFromDouble( 0.1 ), b3fFromDouble( 0.3 ) );
		expect( "a sprung joint past its max reports the excess",
				b3fToDouble( b3Joint_GetLinearSeparation( jointId ) ), 0.5 - 0.3, 0.002 );
		b3DestroyJoint( jointId, true );
	}

	{
		// The three that constrain no position at all.
		b3MotorJointDef motorDef = b3DefaultMotorJointDef();
		motorDef.base.bodyIdA = anchor;
		motorDef.base.bodyIdB = moving;
		b3JointId motorId = b3CreateMotorJoint( worldId, &motorDef );

		b3ParallelJointDef parallelDef = b3DefaultParallelJointDef();
		parallelDef.base.bodyIdA = anchor;
		parallelDef.base.bodyIdB = moving;
		b3JointId parallelId = b3CreateParallelJoint( worldId, &parallelDef );

		b3FilterJointDef filterDef = b3DefaultFilterJointDef();
		filterDef.base.bodyIdA = anchor;
		filterDef.base.bodyIdB = moving;
		b3JointId filterId = b3CreateFilterJoint( worldId, &filterDef );

		b3Body_SetTransform( moving, pulled, b3Quat_identity );
		check( "a motor joint constrains no position", b3Raw( b3Joint_GetLinearSeparation( motorId ) ) == 0 );
		check( "nor does a parallel joint", b3Raw( b3Joint_GetLinearSeparation( parallelId ) ) == 0 );
		check( "nor a filter joint", b3Raw( b3Joint_GetLinearSeparation( filterId ) ) == 0 );

		b3DestroyJoint( motorId, true );
		b3DestroyJoint( parallelId, true );
		b3DestroyJoint( filterId, true );
	}

	validate( world );
	b3DestroyWorld( worldId );
}

/// The wheel's suspension limit, which shares b3PointLineSeparation with the
/// slider and therefore inherits the same fix.
static void test_joint_linear_separation_wheel( void )
{
	printf( "wheel separation covers suspension travel and off-axis offset\n" );

	b3WorldId worldId = makeStepWorld( V( 0, 0, 0 ) );
	b3World* world = b3GetWorldFromId( worldId );

	b3BoxHull hull;
	b3WheelJointDef def;
	b3BodyId anchor, wheel;
	b3JointId jointId = wheelRig( worldId, V( 0, 0, 0 ), &hull, &def, &anchor, &wheel );

	// The rig aims the travel axis along +y.
	b3WheelJoint_EnableSuspensionLimit( jointId, true );
	b3WheelJoint_SetSuspensionLimits( jointId, b3fFromDouble( -0.3 ), b3fFromDouble( 0.3 ) );
	validate( world );

	b3Body_SetTransform( wheel, V( 0, 0.2, 0 ), b3Quat_identity );
	expect( "inside its travel the wheel reports zero", b3fToDouble( b3Joint_GetLinearSeparation( jointId ) ), 0.0,
			0.002 );

	b3Body_SetTransform( wheel, V( 0, 0.5, 0 ), b3Quat_identity );
	expect( "past the upper stop it reports the excess", b3fToDouble( b3Joint_GetLinearSeparation( jointId ) ),
			0.2, 0.002 );

	// Off the suspension axis, inside the travel: the point-to-line term alone,
	// and it is the term upstream would under-report.
	b3Body_SetTransform( wheel, V( 0.35, 0.1, 0 ), b3Quat_identity );
	expect( "off the suspension axis it reports the perpendicular distance",
			b3fToDouble( b3Joint_GetLinearSeparation( jointId ) ), 0.35, 0.003 );

	b3DestroyWorld( worldId );
}

/// The angular query, which returns brads rather than radians and has to fold
/// the double cover, the limits, and two joints' worth of summing.
static void test_joint_angular_separation( void )
{
	printf( "angular separation across the joint types\n" );

	b3WorldId worldId = makeStepWorld( V( 0, 0, 0 ) );
	b3World* world = b3GetWorldFromId( worldId );

	b3BodyDef anchorDef = b3DefaultBodyDef();
	anchorDef.type = b3_staticBody;
	anchorDef.position = V( 0, 0, 0 );
	b3BodyId anchor = b3CreateBody( worldId, &anchorDef );

	b3BodyDef movingDef = b3DefaultBodyDef();
	movingDef.type = b3_dynamicBody;
	movingDef.position = V( 0, 0, 0 );
	movingDef.enableSleep = false;
	b3BodyId moving = b3CreateBody( worldId, &movingDef );

	b3BoxHull hull = b3MakeBoxHull( b3fFromDouble( 0.15 ), b3fFromDouble( 0.15 ), b3fFromDouble( 0.15 ) );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3CreateHullShape( moving, &shapeDef, &hull.base );

	// A tenth of a turn, in brads, which is what the query returns.
	const b3a tenth = (b3a)( B3_BRAD_CIRCLE / 10 );

	{
		// **The line Step 1 made meaningful.** A parallel joint permits free
		// rotation about z and constrains the other two, so a pure z twist is
		// zero separation and a tilt about x is the tilt.
		//
		// Against the old b3GetQuatAngle -- 2*acos(|s|), which reads the scalar
		// part alone -- upstream's `relQ.v.z = 0` changed nothing, so the twist
		// case below would have reported the whole tenth of a turn.
		b3ParallelJointDef def = b3DefaultParallelJointDef();
		def.base.bodyIdA = anchor;
		def.base.bodyIdB = moving;
		b3JointId jointId = b3CreateParallelJoint( worldId, &def );

		b3Body_SetTransform( moving, V( 0, 0, 0 ), b3MakeQuatFromAxisAngle( b3Vec3_axisZ, tenth ) );
		checkInt( "a parallel joint's free twist is not a separation",
				  b3Joint_GetAngularSeparation( jointId ) < 40, 1 );

		b3Body_SetTransform( moving, V( 0, 0, 0 ), b3MakeQuatFromAxisAngle( b3Vec3_axisX, tenth ) );
		expect( "but a tilt off its plane is", (double)b3Joint_GetAngularSeparation( jointId ), (double)tenth, 40.0 );
		b3DestroyJoint( jointId, true );
	}

	{
		// A hinge inside its limit reports only the off-axis error, which for a
		// pure hinge rotation is zero; outside, the hinge angle counts too.
		b3RevoluteJointDef def = b3DefaultRevoluteJointDef();
		def.base.bodyIdA = anchor;
		def.base.bodyIdB = moving;
		b3JointId jointId = b3CreateRevoluteJoint( worldId, &def );

		b3RevoluteJoint_EnableLimit( jointId, true );
		b3RevoluteJoint_SetLimits( jointId, (b3a)( -B3_BRAD_CIRCLE / 8 ), (b3a)( B3_BRAD_CIRCLE / 8 ) );

		b3Body_SetTransform( moving, V( 0, 0, 0 ), b3MakeQuatFromAxisAngle( b3Vec3_axisZ, tenth ) );
		checkInt( "a hinge inside its limit reports nothing",
				  b3Joint_GetAngularSeparation( jointId ) < 40, 1 );

		const b3a past = (b3a)( B3_BRAD_CIRCLE / 5 );
		b3Body_SetTransform( moving, V( 0, 0, 0 ), b3MakeQuatFromAxisAngle( b3Vec3_axisZ, past ) );
		expect( "outside it, the hinge angle itself is the separation",
				(double)b3Joint_GetAngularSeparation( jointId ), (double)past, 60.0 );
		b3DestroyJoint( jointId, true );
	}

	{
		// The spherical joint's sum, which is the case that can overflow a b3a
		// and so is accumulated in int32_t.
		b3SphericalJointDef def = b3DefaultSphericalJointDef();
		def.base.bodyIdA = anchor;
		def.base.bodyIdB = moving;
		b3JointId jointId = b3CreateSphericalJoint( worldId, &def );

		b3SphericalJoint_EnableConeLimit( jointId, true );
		b3SphericalJoint_SetConeLimit( jointId, (b3a)( B3_BRAD_CIRCLE / 16 ) );

		// A swing well past the cone. The cone axis is the frame's z, so a
		// rotation about x swings it.
		const b3a swing = (b3a)( B3_BRAD_CIRCLE / 6 );
		b3Body_SetTransform( moving, V( 0, 0, 0 ), b3MakeQuatFromAxisAngle( b3Vec3_axisX, swing ) );

		const b3a coneOnly = b3Joint_GetAngularSeparation( jointId );
		expect( "a ball joint past its cone reports the excess", (double)coneOnly,
				(double)( swing - B3_BRAD_CIRCLE / 16 ), 80.0 );

		// Adding a twist limit must *increase* the reading, because the two
		// excesses sum. That is the property, not the exact second value.
		b3SphericalJoint_EnableTwistLimit( jointId, true );
		b3SphericalJoint_SetTwistLimits( jointId, (b3a)( -B3_BRAD_CIRCLE / 64 ), (b3a)( B3_BRAD_CIRCLE / 64 ) );
		b3Body_SetTransform( moving, V( 0, 0, 0 ),
							 b3MulQuat( b3MakeQuatFromAxisAngle( b3Vec3_axisX, swing ),
										b3MakeQuatFromAxisAngle( b3Vec3_axisZ, (b3a)( B3_BRAD_CIRCLE / 8 ) ) ) );

		const b3a both = b3Joint_GetAngularSeparation( jointId );
		printf( "  cone excess %d brads, cone plus twist %d brads\n", (int)coneOnly, (int)both );
		check( "and a twist excess adds to it rather than replacing it", both > coneOnly );
		check( "the sum saturates at a half turn rather than wrapping",
			   both > 0 && both <= (b3a)( B3_BRAD_CIRCLE / 2 ) );
		b3DestroyJoint( jointId, true );
	}

	{
		// The types with no rotational constraint, and the sprung weld.
		b3DistanceJointDef distanceDef = b3DefaultDistanceJointDef();
		distanceDef.base.bodyIdA = anchor;
		distanceDef.base.bodyIdB = moving;
		b3JointId distanceId = b3CreateDistanceJoint( worldId, &distanceDef );

		b3WeldJointDef weldDef = b3DefaultWeldJointDef();
		weldDef.base.bodyIdA = anchor;
		weldDef.base.bodyIdB = moving;
		b3JointId weldId = b3CreateWeldJoint( worldId, &weldDef );

		b3Body_SetTransform( moving, V( 0, 0, 0 ), b3MakeQuatFromAxisAngle( b3Vec3_axisX, tenth ) );

		checkInt( "a distance joint constrains no rotation", b3Joint_GetAngularSeparation( distanceId ), 0 );
		expect( "a rigid weld reports the whole misalignment", (double)b3Joint_GetAngularSeparation( weldId ),
				(double)tenth, 40.0 );

		b3WeldJoint_SetAngularHertz( weldId, b3fFromDouble( 5.0 ) );
		checkInt( "a sprung weld reports none", b3Joint_GetAngularSeparation( weldId ), 0 );

		b3DestroyJoint( distanceId, true );
		b3DestroyJoint( weldId, true );
	}

	validate( world );
	b3DestroyWorld( worldId );
}

/// The wheel's angular case, which upstream does not implement at all -- it
/// asserts false and returns zero. The port answers, so the port is tested.
static void test_joint_angular_separation_wheel( void )
{
	printf( "wheel angular separation, which upstream asserts rather than answers\n" );

	b3WorldId worldId = makeStepWorld( V( 0, 0, 0 ) );
	b3World* world = b3GetWorldFromId( worldId );

	b3BoxHull hull;
	b3WheelJointDef def;
	b3BodyId anchor, wheel;
	b3JointId jointId = wheelRig( worldId, V( 0, 0, 0 ), &hull, &def, &anchor, &wheel );
	validate( world );

	// Aligned: the spin axis lies in the plane it belongs in, so nothing.
	b3Body_SetTransform( wheel, V( 0, 0, 0 ), b3Quat_identity );
	const b3a aligned = b3Joint_GetAngularSeparation( jointId );
	check( "an aligned wheel reports essentially nothing", aligned < 40 );

	// Steering is free with no limit set, so a twist about the suspension axis
	// must *not* register. The rig aims the suspension axis along world +y.
	b3Body_SetTransform( wheel, V( 0, 0, 0 ), b3MakeQuatFromAxisAngle( b3Vec3_axisY, (b3a)( B3_BRAD_CIRCLE / 8 ) ) );
	check( "free steering is not a separation", b3Joint_GetAngularSeparation( jointId ) < 60 );

	// Tipping the wheel takes the spin axis out of its plane, which is the
	// collinearity error, and it is the term that grows monotonically as the
	// wheel tips. Sweep it rather than spot-check, the way the degenerate sweep
	// does -- a monotone rise is the property.
	//
	// About world **x**, and the axis matters: the rig aims the suspension axis
	// (frame A's x) along world +y and the spin axis (frame B's z) along world
	// +z, so tipping means giving the spin axis a component along +y. A rotation
	// about world z would leave the spin axis exactly where it is and report a
	// flat zero -- which is what this test did first, and what it looked like.
	int rises = 0;
	b3a previous = 0;
	for ( int i = 1; i <= 8; ++i )
	{
		const b3a tip = (b3a)( ( B3_BRAD_CIRCLE / 4 ) * i / 8 );
		b3Body_SetTransform( wheel, V( 0, 0, 0 ), b3MakeQuatFromAxisAngle( b3Vec3_axisX, tip ) );

		const b3a got = b3Joint_GetAngularSeparation( jointId );
		if ( got > previous )
		{
			rises++;
		}
		previous = got;
	}

	printf( "  8 tip angles from 0 to 90 degrees: %d monotone rises, final %d brads\n", rises, (int)previous );
	checkInt( "tipping the wheel raises the collinearity error monotonically", rises, 8 );

	// A quarter turn puts the spin axis fully onto the suspension axis, which is
	// a quarter turn of error.
	expect( "and a fully tipped wheel reports a quarter turn", (double)previous,
			(double)( B3_BRAD_CIRCLE / 4 ), 80.0 );

	// The steering limit excess, on top.
	b3Body_SetTransform( wheel, V( 0, 0, 0 ), b3MakeQuatFromAxisAngle( b3Vec3_axisY, (b3a)( B3_BRAD_CIRCLE / 8 ) ) );
	b3WheelJoint_EnableSteeringLimit( jointId, true );
	b3WheelJoint_SetSteeringLimits( jointId, (b3a)( -B3_BRAD_CIRCLE / 32 ), (b3a)( B3_BRAD_CIRCLE / 32 ) );

	const b3a steered = b3Joint_GetAngularSeparation( jointId );
	printf( "  steered an eighth of a turn against a limit of a thirty-second: %d brads\n", (int)steered );
	expect( "a wheel past its steering stop reports the excess", (double)steered,
			(double)( B3_BRAD_CIRCLE / 8 - B3_BRAD_CIRCLE / 32 ), 80.0 );

	b3DestroyWorld( worldId );
}

// =========================================================================
// Joint events -- Stage 7 Step 5
// =========================================================================
//
// `run_pair` has no event plumbing beyond begin/end touch counts -- even hit
// events were never pair-tested -- so this is host-only, like the separation
// queries but for a duller reason: there is nothing on the reference side to
// compare against.

/// A weight hanging from a rigid distance joint, which is the simplest scene
/// with a reaction force that has a closed form: the joint carries the weight.
/// @param x
///     Where along the x axis to hang it. Several of these in one world must be
///     spaced apart -- three at the same point interpenetrate, and 125 kg cubes
///     sharing a volume produce a contact impulse that leaves Q16 outright.
static b3JointId hangingWeight( b3WorldId worldId, double x, b3BodyId* anchorOut, b3BodyId* weightOut,
								b3BoxHull* hull )
{
	b3BodyDef anchorDef = b3DefaultBodyDef();
	anchorDef.type = b3_staticBody;
	anchorDef.position = V( x, 0, 0 );
	b3BodyId anchor = b3CreateBody( worldId, &anchorDef );

	b3BodyDef weightDef = b3DefaultBodyDef();
	weightDef.type = b3_dynamicBody;
	weightDef.position = V( x, -1, 0 );
	weightDef.enableSleep = false;
	b3BodyId weight = b3CreateBody( worldId, &weightDef );

	*hull = b3MakeBoxHull( b3fFromDouble( 0.25 ), b3fFromDouble( 0.25 ), b3fFromDouble( 0.25 ) );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3CreateHullShape( weight, &shapeDef, &hull->base );

	b3DistanceJointDef def = b3DefaultDistanceJointDef();
	def.base.bodyIdA = anchor;
	def.base.bodyIdB = weight;
	def.base.localFrameB.p = V( 0, 1, 0 );
	def.length = b3fFromDouble( 1.0 );

	*anchorOut = anchor;
	*weightOut = weight;
	return b3CreateDistanceJoint( worldId, &def );
}

static void test_joint_events( void )
{
	printf( "joint events fire against the force threshold\n" );

	const double g = 9.8;
	b3WorldId worldId = makeStepWorld( V( 0, -g, 0 ) );
	b3World* world = b3GetWorldFromId( worldId );

	b3BodyId anchor, weight;
	b3BoxHull hull;
	b3JointId jointId = hangingWeight( worldId, 0.0, &anchor, &weight, &hull );

	int marker = 0;
	b3Joint_SetUserData( jointId, &marker );
	validate( world );

	// Settle, so the reaction is the steady weight rather than a transient.
	for ( int i = 0; i < 60; ++i )
	{
		b3World_Step( worldId, 4 );
	}

	const double weightN = b3fToDouble( b3Body_GetMass( weight ) ) * g;
	const double reaction = b3fToDouble( b3Length( b3Joint_GetConstraintForce( jointId ) ) );
	printf( "  hanging mass carries %.2f N (weight %.2f N)\n", reaction, weightN );

	// **The default reports nothing.** Both thresholds are B3_NO_BOUND, and this
	// is the state every joint in every previous stage has been in -- the fields
	// existed, were copied, had four accessors, and were read by nothing.
	checkInt( "with no threshold set, no events", b3World_GetJointEvents( worldId ).jointCount, 0 );

	// A threshold above the load: still nothing.
	b3Joint_SetForceThreshold( jointId, b3fFromDouble( reaction * 2.0 ) );
	b3World_Step( worldId, 4 );
	checkInt( "a threshold above the load does not trip", b3World_GetJointEvents( worldId ).jointCount, 0 );

	// A threshold below it: one event, and only one, however many sub-steps
	// crossed it. That is what the bitset is for.
	b3Joint_SetForceThreshold( jointId, b3fFromDouble( reaction * 0.5 ) );
	b3World_Step( worldId, 4 );

	b3JointEvents events = b3World_GetJointEvents( worldId );
	checkInt( "a threshold below the load trips exactly once", events.jointCount, 1 );

	if ( events.jointCount == 1 )
	{
		b3JointEvent* event = events.jointEvents;

		printf( "  event: force %.2f N, torque %.4f N-m, forceExceeded %d, torqueExceeded %d\n",
				b3fToDouble( event->force ), b3fToDouble( event->torque ), (int)event->forceExceeded,
				(int)event->torqueExceeded );

		// The id must still resolve after the step, which is the difference
		// between a usable event and a number.
		check( "the event's joint id still resolves", b3Joint_IsValid( event->jointId ) );
		check( "and identifies the joint that tripped",
			   b3Joint_GetUserData( event->jointId ) == (void*)&marker );
		check( "the event carries the joint's user data", event->userData == (void*)&marker );

		check( "it is the force that was exceeded, not the torque", event->forceExceeded );
		check( "a distance joint applies no torque, so that flag is clear", event->torqueExceeded == false );

		// The reported force is the one standing at the end of the step, so it
		// agrees with what a caller reads back from the public query.
		expect( "the reported force matches b3Joint_GetConstraintForce", b3fToDouble( event->force ),
				b3fToDouble( b3Length( b3Joint_GetConstraintForce( jointId ) ) ), 0.01 );
	}

	// **The array clears between steps.** A stale event would look exactly like
	// a joint that tripped again.
	b3Joint_SetForceThreshold( jointId, B3_NO_BOUND );
	b3World_Step( worldId, 4 );
	checkInt( "clearing the threshold clears the events", b3World_GetJointEvents( worldId ).jointCount, 0 );

	// A zero threshold reports every awake joint, which is deliberate rather
	// than degenerate -- the comparison is `>=`.
	b3Joint_SetForceThreshold( jointId, b3f_zero );
	b3World_Step( worldId, 4 );
	checkInt( "a zero threshold reports the joint every step", b3World_GetJointEvents( worldId ).jointCount, 1 );

	validate( world );
	b3DestroyWorld( worldId );
}

/// The torque side, and the second joint -- so the drain is exercised with more
/// than one bit set and the count is not accidentally right.
static void test_joint_events_torque_and_multiple( void )
{
	printf( "joint events on torque, and with several joints at once\n" );

	const double g = 9.8;
	b3WorldId worldId = makeStepWorld( V( 0, -g, 0 ) );
	b3World* world = b3GetWorldFromId( worldId );

	// Three weights on three joints, so the bitset drain has to find all three
	// and the event array has to hold them.
	b3BodyId anchors[3], weights[3];
	b3BoxHull hulls[3];
	b3JointId joints[3];

	for ( int i = 0; i < 3; ++i )
	{
		joints[i] = hangingWeight( worldId, 2.0 * i, &anchors[i], &weights[i], &hulls[i] );
	}
	validate( world );

	for ( int i = 0; i < 60; ++i )
	{
		b3World_Step( worldId, 4 );
	}

	// Only the middle one gets a threshold: the count proves the drain reports
	// what tripped rather than everything awake.
	b3Joint_SetForceThreshold( joints[1], b3f_zero );
	b3World_Step( worldId, 4 );
	checkInt( "one of three joints armed reports one event", b3World_GetJointEvents( worldId ).jointCount, 1 );

	for ( int i = 0; i < 3; ++i )
	{
		b3Joint_SetForceThreshold( joints[i], b3f_zero );
	}
	b3World_Step( worldId, 4 );
	checkInt( "all three armed reports three", b3World_GetJointEvents( worldId ).jointCount, 3 );

	// A weld joint holding a cantilever carries a real torque, which is what the
	// torque threshold is for -- the distance joint above has none.
	//
	// The lever's centre of mass must sit **away** from the weld or there is no
	// gravity torque at all: welding a symmetric box at its own centre gives a
	// perfectly balanced lever, which is what this scene did first and why it
	// measured nothing. The body is offset a metre from the anchor and the joint
	// frame on B is pulled back the same metre, so the weld lands at the anchor
	// and the mass hangs off the end of it.
	//
	// Light, too: at the default density a 2.4-metre box is 96 kg and its
	// reaction impulse leaves Q16 outright -- the debug shadow checker caught
	// that as an OVERFLOW in mulFFToImp before any assertion did.
	const double arm = 1.0;

	b3BodyDef anchorDef = b3DefaultBodyDef();
	anchorDef.type = b3_staticBody;
	anchorDef.position = V( 5, 0, 0 );
	b3BodyId leverAnchor = b3CreateBody( worldId, &anchorDef );

	b3BodyDef leverDef = b3DefaultBodyDef();
	leverDef.type = b3_dynamicBody;
	leverDef.position = V( 5 + arm, 0, 0 );
	leverDef.enableSleep = false;
	b3BodyId lever = b3CreateBody( worldId, &leverDef );

	b3BoxHull leverHull = b3MakeBoxHull( b3fFromDouble( 0.2 ), b3fFromDouble( 0.1 ), b3fFromDouble( 0.1 ) );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = b3fFromDouble( 100.0 );
	b3CreateHullShape( lever, &shapeDef, &leverHull.base );

	b3WeldJointDef weldDef = b3DefaultWeldJointDef();
	weldDef.base.bodyIdA = leverAnchor;
	weldDef.base.bodyIdB = lever;
	weldDef.base.localFrameB.p = V( -arm, 0, 0 );
	b3JointId weldId = b3CreateWeldJoint( worldId, &weldDef );

	for ( int i = 0; i < 3; ++i )
	{
		b3Joint_SetForceThreshold( joints[i], B3_NO_BOUND );
	}

	for ( int i = 0; i < 30; ++i )
	{
		b3World_Step( worldId, 4 );
	}

	const double torque = b3fToDouble( b3Length( b3Joint_GetConstraintTorque( weldId ) ) );
	printf( "  welded lever carries %.3f N-m\n", torque );
	check( "the lever really is loaded in torque", torque > 0.1 );

	b3Joint_SetTorqueThreshold( weldId, b3fFromDouble( torque * 0.5 ) );
	b3World_Step( worldId, 4 );

	b3JointEvents events = b3World_GetJointEvents( worldId );
	checkInt( "the torque threshold trips", events.jointCount, 1 );

	if ( events.jointCount == 1 )
	{
		check( "and reports torque rather than force", events.jointEvents[0].torqueExceeded );
		check( "with the force flag clear, its threshold being unset",
			   events.jointEvents[0].forceExceeded == false );
	}

	validate( world );
	b3DestroyWorld( worldId );
}

static void test_friction_on_a_slope( void )
{
	printf( "friction on a slope\n" );

	// The only test that exercises the 2x2 tangent mass and the friction cone.
	// A box on a slope of angle t slides when tan(t) exceeds the friction
	// coefficient, and holds when it does not -- a closed form that does not
	// depend on the solver's tuning, only on whether the cone is the right
	// size.
	//
	// The slope is 30 degrees, so tan is 0.577. Run it either side of that.
	const double mu[2] = { 0.2, 0.9 };
	double travelled[2];

	for ( int k = 0; k < 2; ++k )
	{
		b3WorldId worldId = makeStepWorld( V( 0, -10, 0 ) );
		b3World* world = b3GetWorldFromId( worldId );

		static b3BoxHull s_slope;
		static b3BoxHull s_crate;
		s_slope = b3MakeBoxHull( b3fFromDouble( 4.0 ), b3fFromDouble( 0.5 ), b3fFromDouble( 4.0 ) );
		s_crate = b3MakeBoxHull( b3fFromDouble( 0.5 ), b3fFromDouble( 0.5 ), b3fFromDouble( 0.5 ) );

		b3Quat tilt = b3MakeQuatFromAxisAngle( b3Vec3_axisZ, (b3a)( B3_BRAD_CIRCLE / 12 ) );

		b3BodyDef groundDef = b3DefaultBodyDef();
		groundDef.type = b3_staticBody;
		groundDef.position = V( 0, 0, 0 );
		groundDef.rotation = tilt;
		b3BodyId groundId = b3CreateBody( worldId, &groundDef );

		b3ShapeDef groundShape = b3DefaultShapeDef();
		groundShape.baseMaterial.friction = b3cFromFrac( (int)( mu[k] * 100.0 + 0.5 ), 100 );
		b3CreateHullShape( groundId, &groundShape, &s_slope.base );

		// Placed on the slope surface, tilted to match, so it starts resting
		// rather than tumbling into position.
		b3BodyDef def = b3DefaultBodyDef();
		def.type = b3_dynamicBody;
		def.position = b3RotateVector( tilt, V( 0, 1.0, 0 ) );
		def.rotation = tilt;
		def.enableSleep = false;
		b3BodyId crateId = b3CreateBody( worldId, &def );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.baseMaterial.friction = b3cFromFrac( (int)( mu[k] * 100.0 + 0.5 ), 100 );
		b3CreateHullShape( crateId, &shapeDef, &s_crate.base );

		b3Vec3 start = b3Body_GetPosition( crateId );

		for ( int i = 0; i < 180; ++i )
		{
			b3World_Step( worldId, 4 );
		}

		b3Vec3 end = b3Body_GetPosition( crateId );
		double dx = F( end.x ) - F( start.x );
		double dy = F( end.y ) - F( start.y );
		travelled[k] = sqrt( dx * dx + dy * dy );

		validate( world );
		b3DestroyWorld( worldId );
	}

	printf( "  30 degree slope over 3 s: mu=0.2 slid %.4f, mu=0.9 slid %.4f\n", travelled[0], travelled[1] );

	// tan(30) = 0.577, so 0.2 is below the holding threshold and 0.9 is above.
	check( "a box slides when friction is below tan(slope)", travelled[0] > 0.25 );
	check( "a box holds when friction is above tan(slope)", travelled[1] < 0.05 );
	check( "friction makes a difference of at least an order of magnitude", travelled[0] > 10.0 * travelled[1] );
}

static void test_hit_events( void )
{
	printf( "hit events\n" );

	// The event API's last unfed corner. Two runs of the same scene, one fast
	// enough to clear the threshold and one not, so the test distinguishes
	// "hit events work" from "hit events always fire".
	const double speeds[2] = { 0.5, 12.0 };
	int hitCounts[2] = { 0, 0 };
	double approach[2] = { 0.0, 0.0 };

	for ( int k = 0; k < 2; ++k )
	{
		b3WorldId worldId = makeStepWorld( V( 0, 0, 0 ) );
		b3World* world = b3GetWorldFromId( worldId );

		static b3BoxHull s_wall;
		s_wall = b3MakeBoxHull( b3fFromDouble( 4.0 ), b3fFromDouble( 0.5 ), b3fFromDouble( 4.0 ) );

		b3BodyDef groundDef = b3DefaultBodyDef();
		groundDef.type = b3_staticBody;
		b3BodyId groundId = b3CreateBody( worldId, &groundDef );
		b3ShapeDef groundShape = b3DefaultShapeDef();
		groundShape.enableHitEvents = true;
		b3CreateHullShape( groundId, &groundShape, &s_wall.base );

		b3BodyDef def = b3DefaultBodyDef();
		def.type = b3_dynamicBody;
		def.position = V( 0, 2.0, 0 );
		def.enableSleep = false;
		b3BodyId ballId = b3CreateBody( worldId, &def );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.enableHitEvents = true;
		b3Sphere sphere = { V( 0, 0, 0 ), b3fFromDouble( 0.5 ) };
		b3CreateSphereShape( ballId, &shapeDef, &sphere );

		b3Body_SetLinearVelocity( ballId, V( 0, -speeds[k], 0 ) );

		for ( int i = 0; i < 120; ++i )
		{
			b3World_Step( worldId, 4 );

			b3ContactEvents events = b3World_GetContactEvents( worldId );
			for ( int e = 0; e < events.hitCount; ++e )
			{
				hitCounts[k] += 1;
				double speed = b3fToDouble( events.hitEvents[e].approachSpeed );
				if ( speed > approach[k] )
				{
					approach[k] = speed;
					// The normal points from A to B. A is the ground here, so
					// it points up out of the ground.
					check( "hit event normal is the contact normal",
						   fabs( fabs( F( events.hitEvents[e].normal.y ) ) - 1.0 ) < 0.05 );
				}
			}
		}

		validate( world );
		b3DestroyWorld( worldId );
	}

	printf( "  slow (%.1f m/s): %d events; fast (%.1f m/s): %d events, peak approach %.2f m/s\n", speeds[0],
			hitCounts[0], speeds[1], hitCounts[1], approach[1] );

	check( "a fast impact reports a hit event", hitCounts[1] > 0 );
	check( "a slow impact reports none", hitCounts[0] == 0 );
	check( "the reported approach speed is close to the impact speed", approach[1] > 0.5 * speeds[1] );
}

static void test_move_events_and_proxies( void )
{
	printf( "move events and enlarged proxies\n" );

	b3WorldId worldId = makeStepWorld( V( 0, 0, 0 ) );
	b3World* world = b3GetWorldFromId( worldId );

	b3BodyDef def = b3DefaultBodyDef();
	def.type = b3_dynamicBody;
	def.enableSleep = false;
	b3BodyId bodyId = b3CreateBody( worldId, &def );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3Sphere sphere = { V( 0, 0, 0 ), b3fFromDouble( 0.5 ) };
	b3CreateSphereShape( bodyId, &shapeDef, &sphere );

	// A body that barely moves must not enlarge its proxy: the fat AABB has a
	// margin precisely so that small motion costs nothing in the broad phase.
	b3Body_SetLinearVelocity( bodyId, V( 0.01, 0, 0 ) );
	b3World_Step( worldId, 4 );
	validate( world );

	{
		b3BodyEvents events = b3World_GetBodyEvents( worldId );
		checkInt( "one move event for one awake body", events.moveCount, 1 );
		if ( events.moveCount == 1 )
		{
			expect( "the move event carries the body's transform", F( events.moveEvents[0].transform.p.x ),
					F( b3Body_GetPosition( bodyId ).x ), 1e-6 );
			check( "the move event names the body", B3_ID_EQUALS( events.moveEvents[0].bodyId, bodyId ) );
		}
		checkInt( "a small move buffers no proxy", world->broadPhase.moveArray.count, 0 );
	}

	// A body that crosses its margin must enlarge, and land in the move array
	// so the next broad phase re-queries it.
	b3Body_SetLinearVelocity( bodyId, V( 50, 0, 0 ) );
	b3World_Step( worldId, 4 );
	validate( world );

	check( "a large move buffers the proxy", world->broadPhase.moveArray.count > 0 );

	// Every enlarged flag must have been consumed by the broad-phase pass;
	// leaving one set would enlarge the same proxy again next step.
	{
		b3Body* body = b3GetBodyFullId( world, bodyId );
		b3Shape* shape = b3Array_Get( world->shapes, body->headShapeId );
		checkInt( "the enlarged flag was cleared", ( shape->flags & b3_enlargedAABB ) != 0, 0 );
		check( "the fat AABB contains the tight one", b3AABB_Contains( shape->fatAABB, shape->aabb ) );
	}

	validate( world );
	b3DestroyWorld( worldId );
}

// =========================================================================
// World queries -- Phase 7, Stage 1b
// =========================================================================

typedef struct
{
	b3ShapeId ids[8];
	int count;
} queryHits;

static bool collectOverlap( b3ShapeId shapeId, void* context )
{
	queryHits* hits = context;
	if ( hits->count < 8 )
	{
		hits->ids[hits->count++] = shapeId;
	}
	return true;
}

typedef struct
{
	int count;
	b3ShapeId lastId;
	b3Vec3 lastPoint;
	b3Vec3 lastNormal;
	b3c lastFraction;
	/// What to hand back to the cast, so the tests can drive every branch of
	/// b3CastResultFcn's contract.
	b3c reply;
} castHits;

static b3c collectCast( b3ShapeId shapeId, b3Vec3 point, b3Vec3 normal, b3c fraction, uint64_t userMaterialId,
						int triangleIndex, int childIndex, void* context )
{
	B3_UNUSED( userMaterialId, triangleIndex, childIndex );

	castHits* hits = context;
	hits->count += 1;
	hits->lastId = shapeId;
	hits->lastPoint = point;
	hits->lastNormal = normal;
	hits->lastFraction = fraction;
	return hits->reply;
}

/// The five b3World_* queries, against closed forms.
///
/// Not compared against upstream, and deliberately: the port's signatures drop
/// the `b3Pos origin` argument, so there is no call that means the same thing
/// on both sides to compare. What is checkable is the geometry, and a sphere of
/// known radius at a known place has an answer that does not need a reference
/// implementation to state.
static void test_world_queries( void )
{
	printf( "world overlap and cast queries\n" );

	b3WorldId worldId = makeStepWorld( V( 0, 0, 0 ) );
	b3World* world = b3GetWorldFromId( worldId );

	// Three unit spheres in a row on the x axis, at x = 0, 4 and 8. Static, so
	// nothing moves under the queries and every answer stays a closed form.
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_staticBody;

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3Sphere unit = { V( 0, 0, 0 ), b3fFromDouble( 1.0 ) };

	b3ShapeId shapes[3];
	for ( int i = 0; i < 3; ++i )
	{
		bodyDef.position = V( 4.0 * i, 0, 0 );
		b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );
		shapes[i] = b3CreateSphereShape( bodyId, &shapeDef, &unit );
	}

	validate( world );

	b3QueryFilter any = b3DefaultQueryFilter();

	// --- b3World_OverlapAABB ---
	{
		queryHits hits = { 0 };
		b3World_OverlapAABB( worldId, b3MakeAABB( V( -2, -2, -2 ), V( 10, 2, 2 ) ), any, collectOverlap, &hits );
		checkInt( "a box over all three spheres finds three", hits.count, 3 );

		hits.count = 0;
		b3World_OverlapAABB( worldId, b3MakeAABB( V( -2, -2, -2 ), V( 2, 2, 2 ) ), any, collectOverlap, &hits );
		checkInt( "a box over the first finds one", hits.count, 1 );
		check( "and names it", B3_ID_EQUALS( hits.ids[0], shapes[0] ) );

		hits.count = 0;
		b3World_OverlapAABB( worldId, b3MakeAABB( V( 0, 20, 0 ), V( 1, 21, 1 ) ), any, collectOverlap, &hits );
		checkInt( "a box nowhere near them finds none", hits.count, 0 );
	}

	// --- the filter ---
	{
		// A mask that matches nothing must reject every shape, and it is the
		// callback that must not fire rather than the tree that must not look.
		b3QueryFilter none = b3DefaultQueryFilter();
		none.maskBits = 0;

		queryHits hits = { 0 };
		b3World_OverlapAABB( worldId, b3MakeAABB( V( -2, -2, -2 ), V( 10, 2, 2 ) ), none, collectOverlap, &hits );
		checkInt( "a filter matching nothing reports nothing", hits.count, 0 );
	}

	// --- b3World_OverlapShape, which is the exact test the AABB one is not ---
	{
		// The corner of a sphere's bounding box is the cheapest place to see
		// the difference between the two forms. The unit sphere at the origin
		// has AABB [-1, 1] cubed, and (0.9, 0.9, 0.9) sits inside that box but
		// 1.559 from the centre -- well outside the sphere itself.
		b3Vec3 corner = V( 0.9, 0.9, 0.9 );
		b3ShapeProxy cornerProxy = { &corner, 1, b3fFromDouble( 0.05 ) };

		queryHits exact = { 0 };
		b3World_OverlapShape( worldId, &cornerProxy, any, collectOverlap, &exact );
		checkInt( "a probe in a bounding box corner touches nothing", exact.count, 0 );

		queryHits bounds = { 0 };
		b3World_OverlapAABB( worldId, b3MakeAABB( V( 0.85, 0.85, 0.85 ), V( 0.95, 0.95, 0.95 ) ), any, collectOverlap, &bounds );
		checkInt( "where the AABB form, testing bounds only, reports it", bounds.count, 1 );

		// Move the probe onto the sphere and the exact test agrees with the
		// bounds one again -- which is what says the miss above is the exact
		// test working, not the query failing to find anything at all.
		b3Vec3 probe = V( 1.25, 0, 0 );
		b3ShapeProxy proxy = { &probe, 1, b3fFromDouble( 0.5 ) };

		queryHits hits = { 0 };
		b3World_OverlapShape( worldId, &proxy, any, collectOverlap, &hits );
		checkInt( "a probe overlapping a sphere finds it", hits.count, 1 );
		check( "and names the right one", B3_ID_EQUALS( hits.ids[0], shapes[0] ) );

		// And clear of it in the gap between two spheres.
		probe = V( 2, 0, 0 );
		hits.count = 0;
		b3World_OverlapShape( worldId, &proxy, any, collectOverlap, &hits );
		checkInt( "a probe in the gap between two touches neither", hits.count, 0 );
	}

	// --- b3World_CastRay ---
	{
		// Along +x from x = -4, length 16: crosses all three spheres.
		b3Vec3 origin = V( -4, 0, 0 );
		b3Vec3 translation = V( 16, 0, 0 );

		// reply = 1 keeps the full length, so every shape reports.
		castHits all = { .reply = b3c_one };
		b3World_CastRay( worldId, origin, translation, any, collectCast, &all );
		checkInt( "a ray returning 1 sees every shape on it", all.count, 3 );

		// reply = 0 terminates at the first.
		castHits first = { .reply = b3c_zero };
		b3World_CastRay( worldId, origin, translation, any, collectCast, &first );
		checkInt( "a ray returning 0 stops at the first", first.count, 1 );

		// A negative reply means "skip this shape and keep the full ray", so
		// the count is unchanged from the reply = 1 case. This is the branch
		// that must NOT clip, and it is the one an off-by-one would break.
		castHits skipped = { .reply = b3Makeb3c( -B3_C_ONE ) };
		b3World_CastRay( worldId, origin, translation, any, collectCast, &skipped );
		checkInt( "a ray returning -1 skips without clipping", skipped.count, 3 );
	}

	// --- b3World_CastRayClosest ---
	{
		// From x = -4 along +x. The first sphere is centred at 0 with radius 1,
		// so the surface is at x = -1: three of the sixteen units.
		b3RayResult hit = b3World_CastRayClosest( worldId, V( -4, 0, 0 ), V( 16, 0, 0 ), any );

		check( "a ray down the row hits", hit.hit );
		check( "the nearest sphere", B3_ID_EQUALS( hit.shapeId, shapes[0] ) );
		expect( "at three sixteenths of the way", b3cToDouble( hit.fraction ), 3.0 / 16.0, 1e-3 );
		expect( "on that sphere's surface", F( hit.point.x ), -1.0, 0.02 );
		expect( "with the normal facing back down the ray", F( hit.normal.x ), -1.0, 0.02 );
		check( "and reports what the traversal cost", hit.nodeVisits > 0 );

		// Fired the other way from the far side, the *last* sphere is nearest.
		b3RayResult reverse = b3World_CastRayClosest( worldId, V( 12, 0, 0 ), V( -16, 0, 0 ), any );
		check( "a ray from the far side hits", reverse.hit );
		check( "the sphere nearest that end", B3_ID_EQUALS( reverse.shapeId, shapes[2] ) );
		expect( "with the normal facing back down that ray", F( reverse.normal.x ), 1.0, 0.02 );

		// Parallel to the row but two units off it.
		b3RayResult miss = b3World_CastRayClosest( worldId, V( -4, 3, 0 ), V( 16, 0, 0 ), any );
		check( "a ray passing above them misses", miss.hit == false );
		checkInt( "and a miss leaves triangleIndex null", miss.triangleIndex, B3_NULL_INDEX );

		// A *hit* on a convex shape must report no triangle either, and this is
		// the one that was wrong. The primitives build their b3CastOutput from
		// { 0 }, so triangleIndex read back as 0 -- which is a real triangle
		// index, and left a caller unable to tell a sphere hit from a hit on a
		// mesh's first triangle. Caught on device by box3d_pick, which reported
		// `hit level, tri 0` for a ray landing on the top face of a box.
		checkInt( "a hit on a sphere reports no triangle", hit.triangleIndex, B3_NULL_INDEX );
	}

	// --- b3World_CastShape ---
	{
		// A sphere of radius 0.5 swept along +x from x = -4. It meets the first
		// sphere when the centres are 1.5 apart, i.e. at x = -1.5, which is
		// 2.5 of the 16 units.
		b3Vec3 center = V( -4, 0, 0 );
		b3ShapeProxy proxy = { &center, 1, b3fFromDouble( 0.5 ) };

		castHits hits = { .reply = b3c_zero };
		b3World_CastShape( worldId, &proxy, V( 16, 0, 0 ), any, collectCast, &hits );

		checkInt( "a swept sphere meets the first one", hits.count, 1 );
		check( "and names it", B3_ID_EQUALS( hits.lastId, shapes[0] ) );
		expect( "at the fraction the two radii give", b3cToDouble( hits.lastFraction ), 2.5 / 16.0, 4e-3 );

		// Swept along a line that clears them.
		b3Vec3 high = V( -4, 3, 0 );
		b3ShapeProxy highProxy = { &high, 1, b3fFromDouble( 0.5 ) };
		castHits none = { .reply = b3c_zero };
		b3World_CastShape( worldId, &highProxy, V( 16, 0, 0 ), any, collectCast, &none );
		checkInt( "and a sweep that clears them meets nothing", none.count, 0 );
	}

	validate( world );
	b3DestroyWorld( worldId );
}

// =========================================================================
// Continuous collision -- Phase 7, Stage 2
// =========================================================================

/// Fire one small sphere at one thin static slab, in a single step.
///
/// Returns where the sphere ended up, and fills in the world's per-step CCD
/// counters. `useMesh` picks which of the two b3ShapeTimeOfImpact paths is
/// exercised: a hull slab goes through b3TimeOfImpact directly, a baked mesh
/// goes through the triangle traversal.
typedef struct
{
	double endY;
	int toiEvents;
	int distanceIterations;
	int pushBackIterations;
	int rootIterations;
	bool hadTimeOfImpact;
} continuousResult;

static continuousResult fireAtSlab( bool continuous, bool useMesh, bool bullet, double speed )
{
	b3WorldDef def = b3DefaultWorldDef();
	def.capacity.staticBodyCount = 8;
	def.capacity.dynamicBodyCount = 8;
	def.capacity.staticShapeCount = 8;
	def.capacity.dynamicShapeCount = 8;
	def.capacity.contactCount = 16;
	def.capacity.meshContactCount = 4;

	// No gravity: the only motion is the velocity set below, so the distance
	// travelled in one step is exactly speed * dt and every expectation is a
	// closed form.
	def.gravity = V( 0, 0, 0 );
	def.enableContinuous = continuous;

	b3WorldId worldId = b3CreateWorld( &def );
	b3World* world = b3GetWorldFromId( worldId );

	b3BodyDef groundDef = b3DefaultBodyDef();
	groundDef.type = b3_staticBody;
	b3BodyId groundId = b3CreateBody( worldId, &groundDef );
	b3ShapeDef groundShape = b3DefaultShapeDef();

	static gridBlob s_blob;
	static b3BoxHull s_slab;

	if ( useMesh )
	{
		// A flat grid in the y = 0 plane: two triangles thick in the only
		// sense that matters, which is none at all.
		const b3MeshData* mesh = buildGrid( &s_blob, 4, 4.0, NULL );
		b3CreateMeshShape( groundId, &groundShape, mesh, V( 1, 1, 1 ) );
	}
	else
	{
		// A tenth of a unit thick, centred on y = 0.
		s_slab = b3MakeBoxHull( b3fFromDouble( 4.0 ), b3fFromDouble( 0.05 ), b3fFromDouble( 4.0 ) );
		b3CreateHullShape( groundId, &groundShape, &s_slab.base );
	}

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = V( 0, 2, 0 );
	bodyDef.enableSleep = false;
	bodyDef.isBullet = bullet;
	b3BodyId ballId = b3CreateBody( worldId, &bodyDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3Sphere sphere = { V( 0, 0, 0 ), b3fFromDouble( 0.1 ) };
	b3CreateSphereShape( ballId, &shapeDef, &sphere );

	b3Body_SetLinearVelocity( ballId, V( 0, -speed, 0 ) );

	validate( world );
	b3World_Step( worldId, 4 );
	validate( world );

	// Read off the sim rather than the b3Body. Finalize harvests the transient
	// flags into the b3Body *before* it calls the continuous pass, so the
	// b3Body's copy of b3_hadTimeOfImpact is always one step behind the sim's.
	// That is upstream's ordering as well, and worth pinning here so a later
	// change to it is a test failure rather than a surprise.
	b3Body* ball = b3GetBodyFullId( world, ballId );

	continuousResult result = {
		.endY = b3fToDouble( b3Body_GetPosition( ballId ).y ),
		.toiEvents = world->toiEventCount,
		.distanceIterations = world->toiDistanceIterations,
		.pushBackIterations = world->toiPushBackIterations,
		.rootIterations = world->toiRootIterations,
		.hadTimeOfImpact = ( b3GetBodySim( world, ball )->flags & b3_hadTimeOfImpact ) != 0,
	};

	b3DestroyWorld( worldId );
	return result;
}

static void test_continuous( void )
{
	printf( "continuous collision\n" );

	const double quantum = 1.0 / 4096.0;

	// 300 m/s over a 1/60 s step is 5 units of travel, against a slab a tenth
	// of a unit thick. There is no sub-step small enough to catch that: the
	// solver integrates 4 sub-steps of 1.25 units each, and the slab is
	// nowhere near any of the five poses. Only a sweep sees it.
	const double speed = 300.0;

	// --- the hull slab, which is b3TimeOfImpact directly ---
	{
		continuousResult off = fireAtSlab( false, false, false, speed );
		check( "without continuous collision a fast sphere ends up under the slab", off.endY < -1.0 );
		checkInt( "having reported no time of impact", off.toiEvents, 0 );

		continuousResult on = fireAtSlab( true, false, false, speed );
		check( "with it, the same sphere stops above the slab", on.endY > 0.0 );
		// The query stops when the *core* distance reaches
		// max( slop, radiusA + radiusB - slop ). Here that is the sphere's own
		// radius less a slop, so the centre comes to rest a slop nearer the
		// slab than a surface-to-surface touch would put it.
		expect( "a radius less a slop clear of the surface", on.endY, 0.05 + 0.1 - b3fToDouble( B3_LINEAR_SLOP ),
				8 * quantum );
		checkInt( "and one time of impact was recorded", on.toiEvents, 1 );
		check( "the body carries the flag that says so", on.hadTimeOfImpact );

		// The caps are meant to be a safety net, not the thing that stops the
		// search. If a scene this ordinary reaches them, they are load bearing
		// and the numbers below are what would say so. The port is integer
		// code, so these counts are the counts the hardware runs.
		printf( "  hull slab at %.0f m/s: %d distance, %d push back, %d root iterations\n", speed, on.distanceIterations,
				on.pushBackIterations, on.rootIterations );
		check( "the distance loop stays well inside its cap", on.distanceIterations < 25 );
		check( "and so does the root finder", on.rootIterations < 50 );
	}

	// --- the baked mesh, which is the triangle traversal ---
	{
		continuousResult off = fireAtSlab( false, true, false, speed );
		check( "without continuous collision a fast sphere passes through a mesh", off.endY < -1.0 );

		continuousResult on = fireAtSlab( true, true, false, speed );
		check( "with it, the same sphere stops above the mesh", on.endY > 0.0 );
		expect( "a radius less a slop clear of the triangles", on.endY, 0.1 - b3fToDouble( B3_LINEAR_SLOP ), 8 * quantum );
		checkInt( "and one time of impact was recorded", on.toiEvents, 1 );

		printf( "  mesh at %.0f m/s: %d distance, %d push back, %d root iterations\n", speed, on.distanceIterations,
				on.pushBackIterations, on.rootIterations );
		check( "the traversal's worst triangle stays inside the caps", on.distanceIterations < 25 && on.rootIterations < 50 );
	}

	// --- a bullet, which takes the second pass rather than the first ---
	{
		// Against static geometry a bullet must reach the same answer as an
		// ordinary fast body. The difference between the two passes is which
		// trees they query, and the static tree is in both -- so this is the
		// check that the deferred pass runs at all, and that deferring it did
		// not lose the body.
		continuousResult plain = fireAtSlab( true, false, false, speed );
		continuousResult shot = fireAtSlab( true, false, true, speed );

		check( "a bullet is stopped by the slab too", shot.endY > 0.0 );
		expect( "at the same place an ordinary fast body stops", shot.endY, plain.endY, 1e-9 );
		checkInt( "through the deferred second pass", shot.toiEvents, 1 );
	}

	// --- and the pass does not fire when it is not needed ---
	{
		// A tenth of the speed still crosses the slab in one step, but the
		// body is no longer fast relative to its own extent, so nothing sweeps
		// and the narrow phase handles it. The point is that the flag is a
		// threshold and not a synonym for "moving".
		continuousResult slow = fireAtSlab( true, false, false, 1.0 );
		checkInt( "a slow body triggers no time of impact", slow.toiEvents, 0 );
		check( "and does not carry the flag", slow.hadTimeOfImpact == false );
	}
}

// =========================================================================
// Sensors
// =========================================================================

/// A 2-unit static box sensor at the origin plus one dynamic sphere visitor.
///
/// No gravity, so the only motion is what each test sets -- every position in
/// this section is then a closed form rather than a settling result.
typedef struct sensorScene
{
	b3WorldId worldId;
	b3World* world;
	b3BodyId triggerBodyId;
	b3ShapeId sensorId;
	b3BodyId visitorBodyId;
	b3ShapeId visitorId;
} sensorScene;

static b3BoxHull s_triggerHull;

static sensorScene makeSensorScene( double startX, double halfExtent, bool bulletVisitor )
{
	b3WorldDef def = b3DefaultWorldDef();
	def.capacity.staticBodyCount = 8;
	def.capacity.dynamicBodyCount = 24;
	def.capacity.staticShapeCount = 8;
	def.capacity.dynamicShapeCount = 24;
	def.capacity.contactCount = 32;
	def.capacity.sensorCount = 2;
	def.gravity = V( 0, 0, 0 );

	sensorScene scene = { 0 };
	scene.worldId = b3CreateWorld( &def );
	scene.world = b3GetWorldFromId( scene.worldId );

	b3BodyDef triggerBody = b3DefaultBodyDef();
	triggerBody.type = b3_staticBody;
	scene.triggerBodyId = b3CreateBody( scene.worldId, &triggerBody );

	s_triggerHull = b3MakeBoxHull( b3fFromDouble( 1.0 ), b3fFromDouble( halfExtent ), b3fFromDouble( 1.0 ) );

	// Both ends have to opt in: the sensor to report, the visitor to be seen.
	b3ShapeDef sensorDef = b3DefaultShapeDef();
	sensorDef.isSensor = true;
	sensorDef.enableSensorEvents = true;
	scene.sensorId = b3CreateHullShape( scene.triggerBodyId, &sensorDef, &s_triggerHull.base );

	b3BodyDef visitorBody = b3DefaultBodyDef();
	visitorBody.type = b3_dynamicBody;
	visitorBody.position = V( startX, 0, 0 );
	visitorBody.enableSleep = false;
	visitorBody.isBullet = bulletVisitor;
	scene.visitorBodyId = b3CreateBody( scene.worldId, &visitorBody );

	b3ShapeDef visitorDef = b3DefaultShapeDef();
	visitorDef.enableSensorEvents = true;
	b3Sphere sphere = { V( 0, 0, 0 ), b3fFromDouble( 0.25 ) };
	scene.visitorId = b3CreateSphereShape( scene.visitorBodyId, &visitorDef, &sphere );

	validate( scene.world );
	return scene;
}

/// True when `events` names exactly this sensor and this visitor once.
static bool namesPair( b3ShapeId got, b3ShapeId want )
{
	return got.index1 == want.index1 && got.world0 == want.world0 && got.generation == want.generation;
}

static void test_sensors( void )
{
	printf( "sensors\n" );

	// --- one traverse: in, stay, out ---
	{
		// From x = -3.03125 at 3.75 units/s, which is 1/16 of a unit per step.
		// The 0.25-radius sphere overlaps the 1-unit box while |x| <= 1.25, so
		// with x_i = -3.03125 + (i + 1)/16 it is inside for i = 28 through 67:
		// **forty** steps, with each boundary falling half a step away from a
		// transition. That offset is the point of the odd start: b3OverlapSphere
		// compares against B3_OVERLAP_SLOP, two raw quanta, so a boundary landing
		// on a step edge would be decided by accumulated rounding rather than by
		// geometry.
		sensorScene scene = makeSensorScene( -3.03125, 1.0, false );

		check( "a shape created with isSensor is one", b3Shape_IsSensor( scene.sensorId ) );
		check( "an ordinary shape is not", b3Shape_IsSensor( scene.visitorId ) == false );
		check( "sensor events are on for the sensor", b3Shape_AreSensorEventsEnabled( scene.sensorId ) );

		b3Body_SetLinearVelocity( scene.visitorBodyId, V( 3.75, 0, 0 ) );

		int beginTotal = 0;
		int endTotal = 0;
		int stepsInside = 0;
		int peakContacts = 0;
		bool beginNamedTheVisitor = false;
		bool endNamedTheVisitor = false;

		for ( int i = 0; i < 100; ++i )
		{
			b3World_Step( scene.worldId, 4 );
			validate( scene.world );

			b3SensorEvents events = b3World_GetSensorEvents( scene.worldId );
			for ( int e = 0; e < events.beginCount; ++e )
			{
				beginNamedTheVisitor = namesPair( events.beginEvents[e].sensorShapeId, scene.sensorId ) &&
									   namesPair( events.beginEvents[e].visitorShapeId, scene.visitorId );
			}
			for ( int e = 0; e < events.endCount; ++e )
			{
				endNamedTheVisitor = namesPair( events.endEvents[e].sensorShapeId, scene.sensorId ) &&
									 namesPair( events.endEvents[e].visitorShapeId, scene.visitorId );
			}

			beginTotal += events.beginCount;
			endTotal += events.endCount;

			if ( b3Shape_GetSensorCapacity( scene.sensorId ) > 0 )
			{
				stepsInside += 1;
			}

			if ( scene.world->contacts.count > peakContacts )
			{
				peakContacts = scene.world->contacts.count;
			}
		}

		checkInt( "crossing a sensor produces one begin event", beginTotal, 1 );
		checkInt( "and one end event", endTotal, 1 );
		check( "the begin event named the sensor and the visitor", beginNamedTheVisitor );
		check( "so did the end event", endNamedTheVisitor );

		// The interesting half of "one begin": the shape was inside for forty
		// steps and reported nothing on thirty-nine of them. A sensor reports
		// transitions, not presence.
		checkInt( "reporting nothing on the steps in between", stepsInside, 40 );

		// A sensor is skipped in b3PairQueryCallback, so nothing downstream of
		// the broad phase ever hears about it.
		checkInt( "and creating no contacts at all", peakContacts, 0 );

		b3DestroyWorld( scene.worldId );
	}

	// --- what is inside, right now ---
	{
		sensorScene scene = makeSensorScene( 0.0, 1.0, false );
		b3World_Step( scene.worldId, 4 );
		validate( scene.world );

		checkInt( "a sensor holding one shape says so", b3Shape_GetSensorCapacity( scene.sensorId ), 1 );

		b3ShapeId visitors[4] = { 0 };
		int count = b3Shape_GetSensorData( scene.sensorId, visitors, 4 );
		checkInt( "and hands it back", count, 1 );
		check( "naming the visitor", namesPair( visitors[0], scene.visitorId ) );

		// The capacity argument is a bound, not a request.
		checkInt( "a zero-capacity read writes nothing", b3Shape_GetSensorData( scene.sensorId, visitors, 0 ), 0 );

		// A shape that is not a sensor has no overlap set rather than an empty
		// one, and both queries have to say so without reaching for an index
		// that is B3_NULL_INDEX.
		checkInt( "an ordinary shape holds nothing", b3Shape_GetSensorCapacity( scene.visitorId ), 0 );
		checkInt( "and hands back nothing", b3Shape_GetSensorData( scene.visitorId, visitors, 4 ), 0 );

		// Nothing pushed it: a sensor has no collision response.
		b3World_Step( scene.worldId, 4 );
		expect( "and a body resting inside one is not pushed out", F( b3Body_GetPosition( scene.visitorBodyId ).x ), 0.0,
				1e-9 );

		b3DestroyWorld( scene.worldId );
	}

	// --- an overlap can end without anything moving ---
	{
		// Three ways to leave a sensor while standing still, and each one has
		// to produce the same end event as walking out would.
		const char* what[3] = {
			"muting the visitor ends the overlap",
			"muting the sensor ends the overlap",
			"disabling the sensor's body ends the overlap",
		};

		for ( int variant = 0; variant < 3; ++variant )
		{
			sensorScene scene = makeSensorScene( 0.0, 1.0, false );

			b3World_Step( scene.worldId, 4 );
			checkInt( "the visitor is inside to start with", b3Shape_GetSensorCapacity( scene.sensorId ), 1 );

			if ( variant == 0 )
			{
				b3Shape_EnableSensorEvents( scene.visitorId, false );
			}
			else if ( variant == 1 )
			{
				b3Shape_EnableSensorEvents( scene.sensorId, false );
			}
			else
			{
				b3Body_Disable( scene.triggerBodyId );
			}

			// Not immediate: the overlap set is rebuilt once per step, and that
			// is where the change turns into an event.
			b3World_Step( scene.worldId, 4 );
			validate( scene.world );

			b3SensorEvents events = b3World_GetSensorEvents( scene.worldId );
			checkInt( what[variant], events.endCount, 1 );
			checkInt( "  and reports nothing beginning", events.beginCount, 0 );
			checkInt( "  leaving the sensor empty", b3Shape_GetSensorCapacity( scene.sensorId ), 0 );

			b3DestroyWorld( scene.worldId );
		}
	}

	// --- destroying the sensor while something is inside it ---
	{
		sensorScene scene = makeSensorScene( 0.0, 1.0, false );
		b3World_Step( scene.worldId, 4 );
		checkInt( "the visitor is inside to start with", b3Shape_GetSensorCapacity( scene.sensorId ), 1 );

		b3DestroyShape( scene.sensorId, true );
		validate( scene.world );

		// b3DestroySensor writes into the buffer the *next* step publishes,
		// which is the whole point of double buffering the end events: the
		// caller still hears about an overlap ended by a destroy.
		b3SensorEvents immediate = b3World_GetSensorEvents( scene.worldId );
		checkInt( "a destroy is not published before the next step", immediate.endCount, 0 );

		b3World_Step( scene.worldId, 4 );
		b3SensorEvents events = b3World_GetSensorEvents( scene.worldId );
		checkInt( "destroying a sensor ends its overlaps", events.endCount, 1 );
		check( "naming the visitor that was inside", namesPair( events.endEvents[0].visitorShapeId, scene.visitorId ) );

		b3DestroyWorld( scene.worldId );
	}

	// --- two sensors, so the swap in b3DestroySensor is exercised ---
	{
		sensorScene scene = makeSensorScene( 0.0, 1.0, false );

		// A second sensor, well clear of the first, on its own static body.
		b3BodyDef otherBody = b3DefaultBodyDef();
		otherBody.type = b3_staticBody;
		otherBody.position = V( 10, 0, 0 );
		b3BodyId otherBodyId = b3CreateBody( scene.worldId, &otherBody );

		b3ShapeDef otherDef = b3DefaultShapeDef();
		otherDef.isSensor = true;
		otherDef.enableSensorEvents = true;
		b3ShapeId otherSensorId = b3CreateHullShape( otherBodyId, &otherDef, &s_triggerHull.base );

		b3World_Step( scene.worldId, 4 );
		checkInt( "the first sensor holds the visitor", b3Shape_GetSensorCapacity( scene.sensorId ), 1 );
		checkInt( "the second holds nothing", b3Shape_GetSensorCapacity( otherSensorId ), 0 );

		// Destroying the *first* moves the second down into its slot, and the
		// second's shape has to be told. If the fixup were missing, the query
		// below would read the wrong sensor -- or an index past the end.
		b3DestroyShape( scene.sensorId, true );
		validate( scene.world );

		b3World_Step( scene.worldId, 4 );
		validate( scene.world );
		checkInt( "the surviving sensor still answers after a swap", b3Shape_GetSensorCapacity( otherSensorId ), 0 );
		check( "and is still a sensor", b3Shape_IsSensor( otherSensorId ) );

		// It still works, which is the real test of the index fixup: put
		// something in it.
		b3Body_SetTransform( scene.visitorBodyId, V( 10, 0, 0 ), b3Quat_identity );
		b3World_Step( scene.worldId, 4 );
		validate( scene.world );
		checkInt( "and reports what enters it", b3Shape_GetSensorCapacity( otherSensorId ), 1 );

		b3DestroyWorld( scene.worldId );
	}

	// --- the continuous path: a body that crosses a sensor within one step ---
	{
		// This is the seam Stage 2 left open. 300 units/s over a 1/60 s step
		// is five units of travel against a sensor a tenth of a unit thick:
		// the end-of-step overlap query is looking at a body two units past
		// it, so without the sweep the trigger never fires at all.
		sensorScene scene = makeSensorScene( 0.0, 0.05, false );
		b3Body_SetTransform( scene.visitorBodyId, V( 0, 2, 0 ), b3Quat_identity );
		b3Body_SetLinearVelocity( scene.visitorBodyId, V( 0, -300, 0 ) );

		b3World_Step( scene.worldId, 4 );
		validate( scene.world );

		check( "the body ends the step well past the sensor", F( b3Body_GetPosition( scene.visitorBodyId ).y ) < -2.0 );

		b3SensorEvents crossed = b3World_GetSensorEvents( scene.worldId );
		checkInt( "a body sweeping through a sensor still trips it", crossed.beginCount, 1 );
		check( "naming the visitor", namesPair( crossed.beginEvents[0].visitorShapeId, scene.visitorId ) );

		// And having tripped it, it is no longer in it -- the hit is folded
		// into this step's overlap set, so the set empties on the next one.
		b3World_Step( scene.worldId, 4 );
		validate( scene.world );
		b3SensorEvents after = b3World_GetSensorEvents( scene.worldId );
		checkInt( "and leaves it on the following step", after.endCount, 1 );
		checkInt( "the sensor is empty afterwards", b3Shape_GetSensorCapacity( scene.sensorId ), 0 );

		b3DestroyWorld( scene.worldId );
	}

	// --- a sensor the visitor cannot see ---
	{
		sensorScene scene = makeSensorScene( 0.0, 1.0, false );

		// Same geometry, same overlap, one flag off.
		b3Shape_EnableSensorEvents( scene.visitorId, false );
		b3World_Step( scene.worldId, 4 );
		validate( scene.world );

		b3SensorEvents events = b3World_GetSensorEvents( scene.worldId );
		checkInt( "a shape with sensor events off trips nothing", events.beginCount, 0 );
		checkInt( "and does not appear in the sensor", b3Shape_GetSensorCapacity( scene.sensorId ), 0 );

		// Turning it back on is a begin event, not a silent appearance.
		b3Shape_EnableSensorEvents( scene.visitorId, true );
		b3World_Step( scene.worldId, 4 );
		checkInt( "turning it back on is a begin event", b3World_GetSensorEvents( scene.worldId ).beginCount, 1 );

		b3DestroyWorld( scene.worldId );
	}

	// --- the cap, and the counter that says when it binds ---
	{
		sensorScene scene = makeSensorScene( 0.0, 1.0, false );

		checkInt( "a sized scene drops no visitors", scene.world->sensorOverlapDropCount, 0 );

		// One visitor already exists at the origin. Add enough to exceed the
		// cap, all inside the sensor at once, each on its own body so none is
		// skipped as a same-body shape.
		//
		// On a lattice rather than in a heap: coincident bodies are pushed
		// apart hard enough in one step to become fast, and then they reach the
		// sensor through the *continuous* path, whose separate cap
		// (B3_NEA_MAX_CONTINUOUS_SENSOR_HITS) feeds the same counter -- so the
		// first version of this check was measuring two caps at once and
		// neither of them cleanly. 0.3 apart with a 0.05 radius, so nothing
		// touches anything and nothing moves.
		const int extra = B3_NEA_MAX_SENSOR_VISITORS;
		for ( int i = 0; i < extra; ++i )
		{
			double x = -0.6 + 0.3 * ( i % 5 );
			double y = -0.6 + 0.3 * ( i / 5 );

			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.type = b3_dynamicBody;
			bodyDef.position = V( x, y, 0.5 );
			bodyDef.enableSleep = false;
			b3BodyId id = b3CreateBody( scene.worldId, &bodyDef );

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			shapeDef.enableSensorEvents = true;
			b3Sphere sphere = { V( 0, 0, 0 ), b3fFromDouble( 0.05 ) };
			b3CreateSphereShape( id, &shapeDef, &sphere );
		}

		b3World_Step( scene.worldId, 4 );
		validate( scene.world );

		checkInt( "the overlap set stops at the cap", b3Shape_GetSensorCapacity( scene.sensorId ),
				  B3_NEA_MAX_SENSOR_VISITORS );
		checkInt( "and the world counts what it refused", scene.world->sensorOverlapDropCount, 1 );

		b3DestroyWorld( scene.worldId );
	}
}

// =========================================================================
// Character mover -- Phase 7, Stage 4
// =========================================================================

/// What b3World_CollideMover handed back, flattened.
typedef struct
{
	b3ShapeId shapes[16];
	b3Vec3 normals[16];
	b3f offsets[16];
	int planeCount;
	int batchCount;
	bool stop;
} moverHits;

/// Implements b3PlaneResultFcn.
static bool collectMoverPlanes( b3ShapeId shapeId, const b3PlaneResult* planes, int planeCount, void* context )
{
	moverHits* hits = context;
	hits->batchCount += 1;

	for ( int i = 0; i < planeCount && hits->planeCount < 16; ++i )
	{
		hits->shapes[hits->planeCount] = shapeId;
		hits->normals[hits->planeCount] = planes[i].plane.normal;
		hits->offsets[hits->planeCount] = planes[i].plane.offset;
		hits->planeCount += 1;
	}

	return hits->stop == false;
}

/// Implements b3MoverFilterFcn: refuse everything.
static bool refuseEveryShape( b3ShapeId shapeId, void* context )
{
	B3_UNUSED( shapeId, context );
	return false;
}

/// b3World_CollideMover and b3World_CastMover against a floor and a wall.
///
/// The scene is static and gravity-free, so every answer is a closed form
/// rather than a settling result.
static void test_mover_world( void )
{
	printf( "character mover world queries\n" );

	b3WorldDef def = b3DefaultWorldDef();
	def.capacity.staticBodyCount = 8;
	def.capacity.dynamicBodyCount = 8;
	def.capacity.staticShapeCount = 8;
	def.capacity.dynamicShapeCount = 8;
	def.capacity.contactCount = 16;
	def.gravity = V( 0, 0, 0 );

	b3WorldId worldId = b3CreateWorld( &def );
	b3World* world = b3GetWorldFromId( worldId );

	// A floor whose top face is y = 0, and a wall whose face is x = 1.
	static b3BoxHull s_floor;
	static b3BoxHull s_wall;
	s_floor = b3MakeBoxHull( b3fFromDouble( 4.0 ), b3fFromDouble( 0.5 ), b3fFromDouble( 4.0 ) );
	s_wall = b3MakeBoxHull( b3fFromDouble( 0.5 ), b3fFromDouble( 2.0 ), b3fFromDouble( 4.0 ) );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_staticBody;
	b3ShapeDef shapeDef = b3DefaultShapeDef();

	bodyDef.position = V( 0, -0.5, 0 );
	b3BodyId floorBody = b3CreateBody( worldId, &bodyDef );
	b3ShapeId floorShape = b3CreateHullShape( floorBody, &shapeDef, &s_floor.base );

	bodyDef.position = V( 1.5, 2, 0 );
	b3BodyId wallBody = b3CreateBody( worldId, &bodyDef );
	b3ShapeId wallShape = b3CreateHullShape( wallBody, &shapeDef, &s_wall.base );

	validate( world );
	b3QueryFilter any = b3DefaultQueryFilter();

	// A capsule of radius 0.3 whose lower cap centre sits 0.2 above the floor.
	b3Capsule standing = { V( 0, 0.2, 0 ), V( 0, 1.2, 0 ), b3fFromDouble( 0.3 ) };

	// --- standing on the floor, clear of the wall ---
	{
		moverHits hits = { 0 };
		b3World_CollideMover( worldId, &standing, any, collectMoverPlanes, &hits );

		checkInt( "a mover on the floor gets one batch", hits.batchCount, 1 );
		checkInt( "of one plane", hits.planeCount, 1 );
		check( "from the floor", hits.shapes[0].index1 == floorShape.index1 );
		check( "pointing up", b3fToDouble( hits.normals[0].y ) > 0.99 );

		// The core segment is 0.2 above a face the radius reaches 0.3 past.
		expect( "at the overlap depth", b3fToDouble( hits.offsets[0] ), 0.1, 4.0 / 4096.0 );
	}

	// --- in the corner, touching both ---
	{
		// 0.2 clear of the wall face at x = 1, so the 0.3 radius reaches 0.1
		// into it. The core segment has to stay *outside* the wall: put it
		// inside and b3CollideMoverAndHull declines the whole shape as a deep
		// overlap and this reads as one batch, not two.
		b3Capsule cornered = { V( 0.8, 0.2, 0 ), V( 0.8, 1.2, 0 ), b3fFromDouble( 0.3 ) };

		moverHits hits = { 0 };
		b3World_CollideMover( worldId, &cornered, any, collectMoverPlanes, &hits );

		checkInt( "a mover in the corner gets two batches", hits.batchCount, 2 );
		checkInt( "of one plane each", hits.planeCount, 2 );

		bool sawFloor = false;
		bool sawWall = false;
		for ( int i = 0; i < hits.planeCount; ++i )
		{
			sawFloor = sawFloor || hits.shapes[i].index1 == floorShape.index1;
			sawWall = sawWall || hits.shapes[i].index1 == wallShape.index1;
		}
		check( "naming the floor", sawFloor );
		check( "and the wall", sawWall );
	}

	// --- in free space ---
	{
		b3Capsule floating = { V( 0, 8, 0 ), V( 0, 9, 0 ), b3fFromDouble( 0.3 ) };

		moverHits hits = { 0 };
		b3World_CollideMover( worldId, &floating, any, collectMoverPlanes, &hits );

		checkInt( "a mover in free space is never called back", hits.batchCount, 0 );
		checkInt( "and gets no planes", hits.planeCount, 0 );
	}

	// --- the filter ---
	{
		b3Capsule cornered = { V( 0.8, 0.2, 0 ), V( 0.8, 1.2, 0 ), b3fFromDouble( 0.3 ) };

		// Move the wall out of the query's mask, leaving only the floor.
		b3Filter wallFilter = b3Shape_GetFilter( wallShape );
		wallFilter.categoryBits = 2;
		b3Shape_SetFilter( wallShape, wallFilter, false );

		b3QueryFilter floorOnly = b3DefaultQueryFilter();
		floorOnly.maskBits = ~(uint64_t)2;

		moverHits hits = { 0 };
		b3World_CollideMover( worldId, &cornered, floorOnly, collectMoverPlanes, &hits );

		checkInt( "a filter that excludes the wall leaves one batch", hits.batchCount, 1 );
		check( "and it is the floor", hits.shapes[0].index1 == floorShape.index1 );

		wallFilter.categoryBits = 1;
		b3Shape_SetFilter( wallShape, wallFilter, false );
	}

	// --- returning false stops the query ---
	{
		b3Capsule cornered = { V( 0.8, 0.2, 0 ), V( 0.8, 1.2, 0 ), b3fFromDouble( 0.3 ) };

		moverHits hits = { 0 };
		hits.stop = true;
		b3World_CollideMover( worldId, &cornered, any, collectMoverPlanes, &hits );

		checkInt( "a callback that returns false is called once", hits.batchCount, 1 );
	}

	// --- b3World_CastMover straight down at the floor ---
	{
		// Start a unit up and sweep two down. The lower cap centre is at
		// y = 1.2 and it stops with its 0.3 radius on the face at y = 0, so it
		// travels 0.9 of the 2.0 available.
		b3Capsule high = { V( 0, 1.2, 0 ), V( 0, 2.2, 0 ), b3fFromDouble( 0.3 ) };
		b3c fraction = b3World_CastMover( worldId, &high, V( 0, -2, 0 ), any, NULL, NULL );

		expect( "a mover dropped at the floor stops on it", b3cToDouble( fraction ), 0.9 / 2.0, 0.01 );
	}

	// --- and into open air ---
	{
		b3Capsule high = { V( 0, 8, 0 ), V( 0, 9, 0 ), b3fFromDouble( 0.3 ) };
		b3c fraction = b3World_CastMover( worldId, &high, V( 0, 0, 3 ), any, NULL, NULL );

		checkInt( "a sweep that hits nothing returns the whole length", b3Raw( fraction ), B3_C_ONE );
	}

	// --- the mover filter ---
	{
		b3Capsule high = { V( 0, 1.2, 0 ), V( 0, 2.2, 0 ), b3fFromDouble( 0.3 ) };
		b3c fraction = b3World_CastMover( worldId, &high, V( 0, -2, 0 ), any, refuseEveryShape, NULL );

		checkInt( "a filter that refuses everything lets it through", b3Raw( fraction ), B3_C_ONE );
	}

	// --- canEncroach, which is the one that matters ---
	{
		// This mover is already resting on the floor: it overlaps it by the
		// slop, exactly as one does on every frame of every game. Sweeping it
		// *sideways* must be allowed.
		//
		// Without canEncroach the shape cast reports an initial overlap, the
		// callback reads fraction 0, and a character standing on the ground can
		// never walk. Nothing else in this suite exercises that branch --
		// b3World_CastShape sets canEncroach false.
		b3c fraction = b3World_CastMover( worldId, &standing, V( 0.5, 0, 0 ), any, NULL, NULL );

		check( "a mover resting on the floor can still slide along it", b3Raw( fraction ) > 0 );
		checkInt( "for the whole distance asked for", b3Raw( fraction ), B3_C_ONE );
	}

	// --- and it still stops at a wall it is walking into ---
	{
		b3Capsule approaching = { V( 0, 0.2, 0 ), V( 0, 1.2, 0 ), b3fFromDouble( 0.3 ) };
		b3c fraction = b3World_CastMover( worldId, &approaching, V( 2, 0, 0 ), any, NULL, NULL );

		check( "walking into a wall stops short of it", b3Raw( fraction ) < B3_C_ONE );
		check( "but not at zero", b3Raw( fraction ) > 0 );
	}

	// --- the plane cap, and the counter that says it bound ---
	{
		checkInt( "a scene inside the cap drops nothing", world->moverPlaneDropCount, 0 );

		// A mesh is the only backend that can produce more than one plane per
		// shape, so it is the only way to reach the cap. The mode mesh is a
		// grid, so a wide capsule lying across it touches many triangles.
		b3BodyDef meshBodyDef = b3DefaultBodyDef();
		meshBodyDef.type = b3_staticBody;
		meshBodyDef.position = V( 0, 20, 0 );
		b3BodyId meshBody = b3CreateBody( worldId, &meshBodyDef );

		static gridBlob s_grid;
		const b3MeshData* grid = buildGrid( &s_grid, 8, 4.0, NULL );

		b3ShapeDef meshShapeDef = b3DefaultShapeDef();
		b3CreateMeshShape( meshBody, &meshShapeDef, grid, V( 1, 1, 1 ) );
		validate( world );

		// Long enough to lie across a good stretch of the grid, and fat enough
		// to reach every triangle under it.
		b3Capsule sprawled = { V( -2, 20.1, 0 ), V( 2, 20.1, 0 ), b3fFromDouble( 0.6 ) };

		moverHits hits = { 0 };
		b3World_CollideMover( worldId, &sprawled, any, collectMoverPlanes, &hits );

		printf( "  sprawled on the grid: %d batches, %d planes, %d dropped\n", hits.batchCount, hits.planeCount,
				world->moverPlaneDropCount );

		checkInt( "a saturated batch stops at the cap", hits.planeCount, B3_NEA_MAX_MOVER_PLANES );
		checkInt( "and the world counts that it did", world->moverPlaneDropCount, 1 );
	}

	// --- the teleport counter ---
	{
		checkInt( "nothing has teleported yet", world->teleportCount, 0 );

		b3Body_SetTransform( wallBody, V( 1.5, 2, 0 ), b3Quat_identity );
		checkInt( "one b3Body_SetTransform is one teleport", world->teleportCount, 1 );

		b3Body_SetTransform( wallBody, V( 1.5, 2, 0 ), b3Quat_identity );
		b3World_Step( worldId, 4 );
		validate( world );

		// Cumulative, unlike every other counter here: a step does not clear
		// it, which is the whole point -- a respawn on frame 400 is still
		// visible on frame 4000.
		checkInt( "and a step does not clear the count", world->teleportCount, 2 );
	}

	validate( world );
	b3DestroyWorld( worldId );
}

int main( void )
{
	b3TestInstallAssertTrap();

	test_world_lifetime();
	test_steiner();
	test_invert_inertia();
	test_body_mass();
	test_mass_data_round_trip();
	test_set_transitions();
	test_sleep_from_sleeping_set();
	test_id_recycling();
	test_shape_properties();
	test_body_transform();
	test_impulses();

	test_pair_discovery();
	test_custom_filter();
	test_touch_transitions();
	test_manifold_persistence();
	test_contact_recycling();
	test_contact_set_transitions();
	test_contact_between_sleeping_bodies();

	test_ballistics();
	test_constant_velocity();
	test_damping();
	test_soft_constraints();
	test_speed_caps();
	test_motion_locks_integration();
	test_sleeping_by_step_count();
	test_move_events_and_proxies();
	test_world_queries();
	test_continuous();
	test_sensors();
	test_mover_world();

	test_box_on_ground();
	test_warm_starting();
	test_restitution();
	test_box_stack();
	test_no_allocation_while_stepping();
	test_no_allocation_with_joints();
	test_manifold_capacity_is_preallocated();

	test_mesh_rest();
	test_mesh_ramp();
	test_mesh_internal_edge();
	test_no_allocation_while_stepping_on_a_mesh();

	test_body_pushed_out_of_the_world();
	test_walled_box_pit();

	test_joint_plumbing();
	test_joint_merges_sleeping_sets();

	test_distance_joint_rigid();
	test_distance_joint_reaction();
	test_distance_joint_chain_reaction();
	test_distance_joint_spring();
	test_distance_joint_limit();
	test_distance_joint_motor();
	test_distance_joint_sleep_and_accessors();

	test_revolute_joint_holds();
	test_revolute_joint_period();
	test_revolute_joint_torque();
	test_revolute_joint_limit();
	test_revolute_joint_motor();
	test_revolute_joint_fixed_rotation();
	test_revolute_joint_light_bodies();
	test_contact_light_bodies_rolling();
	test_contact_light_mass_pair();
	test_revolute_joint_sleep_and_accessors();

	test_spherical_joint_holds();
	test_spherical_joint_conical_pendulum();
	test_spherical_joint_cone_limit();
	test_spherical_joint_twist_limit();
	test_spherical_joint_spring();
	test_spherical_joint_motor();
	test_spherical_joint_fixed_rotation();
	test_spherical_joint_sleep_and_accessors();
	test_spherical_joint_ragdoll();

	test_weld_joint_holds();
	test_weld_joint_linear_spring();
	test_weld_joint_fixed_rotation();
	test_weld_joint_sleep_and_accessors();

	test_motor_joint_linear_velocity();
	test_motor_joint_angular_velocity();
	test_motor_joint_platform();
	test_motor_joint_zero_bounds_do_nothing();
	test_motor_joint_sleep_and_accessors();

	test_prismatic_joint_free_slide();
	test_prismatic_joint_reaction_direction();
	test_prismatic_joint_spring();
	test_prismatic_joint_motor();
	test_prismatic_joint_limits();
	test_prismatic_joint_locks();
	test_prismatic_joint_default_is_free();
	test_prismatic_joint_far_from_origin();
	test_prismatic_joint_sleep_and_accessors();

	test_parallel_joint_torque_bound_is_a_disc();
	test_parallel_joint_tumbling_does_not_wrap();
	test_parallel_joint_twist_is_free();
	test_parallel_joint_rights_a_tilt();
	test_parallel_joint_default_is_inert();
	test_parallel_joint_heavy_pair_goes_quiet();
	test_parallel_joint_sleep_and_accessors();

	test_wheel_joint_suspension_sag();
	test_wheel_joint_reaction_direction();
	test_wheel_joint_reaction_ignores_limits();
	test_wheel_joint_spin_torque_axis();
	test_wheel_joint_steering_torque_on_a_limit();
	test_wheel_joint_suspension_limits();
	test_wheel_joint_spin_motor();
	test_wheel_joint_degenerate_sweep();
	test_wheel_joint_sleep_and_accessors();

	test_joint_linear_separation_is_perpendicular_distance();
	test_joint_linear_separation_limits();
	test_joint_linear_separation_by_type();
	test_joint_linear_separation_wheel();
	test_joint_angular_separation();
	test_joint_angular_separation_wheel();

	test_joint_events();
	test_joint_events_torque_and_multiple();

	test_friction_on_a_slope();
	test_hit_events();

	// Assertions are part of the result, not a trap: assert_trap.h keeps the
	// run going and this turns any that fired into a reported failure.
	s_checks++;
	if ( b3TestUnexpectedAsserts() != 0 )
	{
		printf( "  FAIL %d unexpected assertion(s) fired\n", b3TestUnexpectedAsserts() );
		s_failures++;
	}

	printf( "\n%d checks, %d failures\n", s_checks, s_failures );
	return s_failures == 0 ? 0 : 1;
}
