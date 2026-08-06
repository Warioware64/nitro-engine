// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Erin Catto        (original Box3D)
// Copyright (c) 2026 Warioware64       (Nitro Engine Advanced fixed-point port)
//
// This file is part of Nitro Engine Advanced

/// @file   sensor.c
/// @brief  The sensor pass: query, sort, diff, publish.
///
/// Four steps per sensor per step, and the order matters:
///
///   1. swap overlaps1/overlaps2, so last step's set is preserved and this
///      step's is empty;
///   2. fold in whatever the continuous pass deposited in `hits` -- a bullet
///      that crossed the sensor entirely within the step and is nowhere near
///      it now;
///   3. query the three broad-phase trees and run the exact overlap test on
///      each candidate;
///   4. sort by shapeId, drop duplicates, and compare against overlaps1.
///
/// Only if step 4 finds a difference is a bit set, and only set bits produce
/// events -- so a sensor with a settled set of visitors costs the query and
/// nothing else.
///
/// See sensor.h for what upstream has here that this file does not: the
/// parallel task, the per-worker context, and the bitset union.

#include "sensor.h"

#include "body.h"
#include "broad_phase.h"
#include "core.h"
#include "ctz.h"
#include "physics_world.h"
#include "shape.h"

#include "box3d/collision.h"
#include "box3d/types.h"

// =========================================================================
// The exact overlap test
// =========================================================================

typedef struct b3SensorQueryContext
{
	b3World* world;
	b3Sensor* sensor;
	const b3Shape* sensorShape;
	b3Transform transform;
} b3SensorQueryContext;

/// Is `visitorShape` inside `sensorShape`?
///
/// Upstream builds the visitor's proxy, transforms its points into the
/// sensor's frame by hand, and then runs its own six-case switch over the
/// sensor's type. Both halves already exist here: b3MakeLocalProxy does the
/// transform loop (distance.c), and b3OverlapShape is the switch (shape.c),
/// which b3World_OverlapShape drives for exactly the same purpose. A second
/// copy of the switch would have to be kept in step with the first every time
/// a shape type moves, and this stage is not the one that would notice.
static bool b3OverlapSensor( const b3Shape* sensorShape, b3Transform sensorTransform, const b3Shape* visitorShape,
							 b3Transform visitorTransform )
{
	b3ShapeProxy proxy = b3MakeShapeProxy( visitorShape );

	// b3MakeLocalProxy applies the *inverse* of the frame it is handed, so the
	// frame wanted here is visitorXf^-1 * sensorXf: its inverse is
	// sensorXf^-1 * visitorXf, which is upstream's relativeTransform, and the
	// points land in the sensor's frame either way. One rotation per point,
	// same as upstream.
	b3Vec3 localPoints[B3_MAX_SHAPE_CAST_POINTS];
	b3Transform relativeTransform = b3InvMulTransforms( visitorTransform, sensorTransform );
	b3ShapeProxy localProxy = b3MakeLocalProxy( &proxy, relativeTransform, localPoints );

	return b3OverlapShape( sensorShape, b3Transform_identity, &localProxy );
}

/// Implements b3TreeQueryCallbackFcn.
static bool b3SensorQueryCallback( int proxyId, uint64_t userData, void* context )
{
	B3_UNUSED( proxyId );

	int shapeId = (int)userData;

	b3SensorQueryContext* queryContext = context;
	const b3Shape* sensorShape = queryContext->sensorShape;
	int sensorShapeId = sensorShape->id;

	// The sensor cannot visit itself.
	if ( shapeId == sensorShapeId )
	{
		return true;
	}

	b3World* world = queryContext->world;
	b3Shape* otherShape = b3Array_Get( world->shapes, shapeId );

	// Visitors must be convex, because b3MakeShapeProxy is a convex point
	// cloud and a triangle mesh is not one. Upstream tests b3IsConvex over six
	// shape types; the port has four and exactly one of them fails, so the
	// test is the type compare rather than a helper with one caller.
	if ( otherShape->type == b3_meshShape )
	{
		return true;
	}

	// Does the visitor want to be seen?
	if ( ( otherShape->flags & b3_enableSensorEvents ) == 0 )
	{
		return true;
	}

	// A body does not trip its own sensors.
	if ( otherShape->bodyId == sensorShape->bodyId )
	{
		return true;
	}

	if ( b3ShouldShapesCollide( sensorShape->filter, otherShape->filter ) == false )
	{
		return true;
	}

	if ( ( sensorShape->flags & b3_enableCustomFiltering ) || ( otherShape->flags & b3_enableCustomFiltering ) )
	{
		b3CustomFilterFcn* customFilterFcn = world->customFilterFcn;
		if ( customFilterFcn != NULL )
		{
			b3ShapeId idA = { sensorShapeId + 1, world->worldId, sensorShape->generation };
			b3ShapeId idB = { shapeId + 1, world->worldId, otherShape->generation };
			if ( customFilterFcn( idA, idB, world->customFilterContext ) == false )
			{
				return true;
			}
		}
	}

	// The tree only said the fat AABBs overlap. This is the exact test.
	b3Transform otherTransform = b3GetBodyTransform( world, otherShape->bodyId );
	if ( b3OverlapSensor( sensorShape, queryContext->transform, otherShape, otherTransform ) == false )
	{
		return true;
	}

	// Record the overlap. Upstream emplaces and lets the array grow; here the
	// capacity is the budget, and going past it drops the visitor rather than
	// allocating mid-step. A dropped visitor is a begin or end event that
	// silently never fires and leaves nothing else in the world looking
	// different, which is why it is counted -- the same treatment, for the
	// same reason, as B3_NEA_MAX_MESH_MANIFOLDS and meshManifoldDropCount.
	b3Sensor* sensor = queryContext->sensor;
	if ( sensor->overlaps2.count == sensor->overlaps2.capacity )
	{
		world->sensorOverlapDropCount += 1;
		return true;
	}

	b3Visitor visitor = { shapeId, otherShape->generation };
	b3Array_Push( sensor->overlaps2, visitor );

	return true;
}

// =========================================================================
// Sorting the overlap set
// =========================================================================

/// Sort visitors by ascending shapeId, in place.
///
/// Upstream calls qsort with a comparator. The port sorts by hand: the array
/// holds at most B3_NEA_MAX_SENSOR_VISITORS eight-byte elements, so an
/// insertion sort beats a generic sort with an indirect call per comparison,
/// and it keeps <stdlib.h> out of the library entirely.
///
/// It also sidesteps an upstream wart. b3CompareVisitors returns 1 -- not 0 --
/// for equal keys, which is not a strict weak ordering; qsort is free to do
/// anything with that, and upstream gets away with it only because the dedup
/// pass runs immediately afterwards. `>` here means equal keys keep their
/// relative order, so the sort is a genuine stable sort by shapeId.
static void b3SortVisitors( b3Visitor* visitors, int count )
{
	for ( int i = 1; i < count; ++i )
	{
		b3Visitor key = visitors[i];
		int j = i - 1;
		while ( j >= 0 && visitors[j].shapeId > key.shapeId )
		{
			visitors[j + 1] = visitors[j];
			j -= 1;
		}
		visitors[j + 1] = key;
	}
}

// =========================================================================
// The pass
// =========================================================================

void b3CreateSensor( b3World* world, b3Shape* sensorShape )
{
	B3_ASSERT( sensorShape->sensorIndex == B3_NULL_INDEX );

	sensorShape->sensorIndex = world->sensors.count;

	b3Sensor* sensor = b3Array_Emplace( world->sensors );
	b3Array_Create( sensor->hits );
	b3Array_Create( sensor->overlaps1 );
	b3Array_Create( sensor->overlaps2 );

	// Reserved once, here, because a shape is created while the scene is being
	// built. Nothing in the pass may grow them afterwards.
	b3Array_Reserve( sensor->hits, B3_NEA_MAX_CONTINUOUS_SENSOR_HITS );
	b3Array_Reserve( sensor->overlaps1, B3_NEA_MAX_SENSOR_VISITORS );
	b3Array_Reserve( sensor->overlaps2, B3_NEA_MAX_SENSOR_VISITORS );

	sensor->shapeId = sensorShape->id;
}

void b3OverlapSensors( b3World* world )
{
	int sensorCount = world->sensors.count;
	if ( sensorCount == 0 )
	{
		return;
	}

	// One bit per sensor: set means the overlap set changed and events are
	// owed. Upstream keeps one of these per worker and unions them afterwards.
	b3BitSet* eventBits = &world->sensorEventBitSet;
	b3SetBitCountAndClear( eventBits, (uint32_t)sensorCount );

	b3DynamicTree* trees = world->broadPhase.trees;

	for ( int sensorIndex = 0; sensorIndex < sensorCount; ++sensorIndex )
	{
		b3Sensor* sensor = b3Array_Get( world->sensors, sensorIndex );
		b3Shape* sensorShape = b3Array_Get( world->shapes, sensor->shapeId );
		B3_ASSERT( sensorShape->sensorIndex == sensorIndex );

		// Last step's set becomes overlaps1; this step builds overlaps2.
		B3_SWAP( sensor->overlaps1, sensor->overlaps2 );
		b3Array_Clear( sensor->overlaps2 );

		// Whatever the continuous pass caught passing through during the step.
		// It goes in before the query so the sort and the dedup below cover it
		// too: a bullet can be both swept through the sensor and still inside
		// it at the end of the step.
		b3Array_Append( sensor->overlaps2, sensor->hits.data, sensor->hits.count );
		b3Array_Clear( sensor->hits );

		b3Body* body = b3Array_Get( world->bodies, sensorShape->bodyId );
		if ( body->setIndex == b3_disabledSet || ( sensorShape->flags & b3_enableSensorEvents ) == 0 )
		{
			// A disabled or muted sensor keeps no overlaps. If it had any, they
			// all end this step.
			if ( sensor->overlaps1.count != 0 )
			{
				b3SetBit( eventBits, (uint32_t)sensorIndex );
			}
			continue;
		}

		b3SensorQueryContext queryContext = {
			.world = world,
			.sensor = sensor,
			.sensorShape = sensorShape,
			.transform = b3GetBodyTransformQuick( world, body ),
		};

		b3AABB queryBounds = sensorShape->aabb;
		uint64_t maskBits = sensorShape->filter.maskBits;

		b3DynamicTree_Query( trees + 0, queryBounds, maskBits, false, b3SensorQueryCallback, &queryContext );
		b3DynamicTree_Query( trees + 1, queryBounds, maskBits, false, b3SensorQueryCallback, &queryContext );
		b3DynamicTree_Query( trees + 2, queryBounds, maskBits, false, b3SensorQueryCallback, &queryContext );

		// Sorted so the two sets can be walked together below, and so the
		// events come out in a deterministic order rather than in whatever
		// order the trees happened to be laid out in.
		b3SortVisitors( sensor->overlaps2.data, sensor->overlaps2.count );

		// Drop duplicates, which the appended hits can introduce.
		int uniqueCount = 0;
		int overlapCount = sensor->overlaps2.count;
		b3Visitor* overlapData = sensor->overlaps2.data;
		for ( int i = 0; i < overlapCount; ++i )
		{
			if ( uniqueCount == 0 || overlapData[i].shapeId != overlapData[uniqueCount - 1].shapeId )
			{
				overlapData[uniqueCount] = overlapData[i];
				uniqueCount += 1;
			}
		}
		sensor->overlaps2.count = uniqueCount;

		// Did anything change? Both sets are sorted, so this is a compare in
		// order rather than a search.
		int count1 = sensor->overlaps1.count;
		int count2 = sensor->overlaps2.count;
		if ( count1 != count2 )
		{
			b3SetBit( eventBits, (uint32_t)sensorIndex );
		}
		else
		{
			for ( int i = 0; i < count1; ++i )
			{
				const b3Visitor* s1 = sensor->overlaps1.data + i;
				const b3Visitor* s2 = sensor->overlaps2.data + i;

				if ( s1->shapeId != s2->shapeId || s1->generation != s2->generation )
				{
					b3SetBit( eventBits, (uint32_t)sensorIndex );
					break;
				}
			}
		}
	}

	// -----------------------------------------------------------------
	// Publish
	// -----------------------------------------------------------------
	//
	// Only the sensors whose bit is set, walked by trailing-zero count -- the
	// same drain the hit and joint events use in solver.c.

	int endEventArrayIndex = world->endEventArrayIndex;
	uint16_t worldIdShort = world->worldId;

	for ( uint32_t k = 0; k < eventBits->blockCount; ++k )
	{
		uint64_t word = eventBits->bits[k];
		while ( word != 0 )
		{
			uint32_t ctz = b3CTZ64( word );
			int sensorIndex = (int)( 64 * k + ctz );

			b3Sensor* sensor = b3Array_Get( world->sensors, sensorIndex );
			b3Shape* sensorShape = b3Array_Get( world->shapes, sensor->shapeId );
			b3ShapeId sensorId = { sensor->shapeId + 1, worldIdShort, sensorShape->generation };

			int count1 = sensor->overlaps1.count;
			int count2 = sensor->overlaps2.count;
			const b3Visitor* refs1 = sensor->overlaps1.data;
			const b3Visitor* refs2 = sensor->overlaps2.data;

			// A merge join over two sorted sets: what is only in overlaps1
			// ended, what is only in overlaps2 began, what is in both with the
			// same generation persisted. Matching ids with *different*
			// generations are two different shapes that reused one slot, and
			// they produce both events -- which is the whole reason a b3Visitor
			// carries a generation.
			int index1 = 0, index2 = 0;
			while ( index1 < count1 && index2 < count2 )
			{
				const b3Visitor* r1 = refs1 + index1;
				const b3Visitor* r2 = refs2 + index2;

				if ( r1->shapeId == r2->shapeId )
				{
					if ( r1->generation < r2->generation )
					{
						b3ShapeId visitorId = { r1->shapeId + 1, worldIdShort, r1->generation };
						b3SensorEndTouchEvent event = { sensorId, visitorId };
						b3Array_Push( world->sensorEndEvents[endEventArrayIndex], event );
						index1 += 1;
					}
					else if ( r1->generation > r2->generation )
					{
						b3ShapeId visitorId = { r2->shapeId + 1, worldIdShort, r2->generation };
						b3SensorBeginTouchEvent event = { sensorId, visitorId };
						b3Array_Push( world->sensorBeginEvents, event );
						index2 += 1;
					}
					else
					{
						// Persisted: no event.
						index1 += 1;
						index2 += 1;
					}
				}
				else if ( r1->shapeId < r2->shapeId )
				{
					b3ShapeId visitorId = { r1->shapeId + 1, worldIdShort, r1->generation };
					b3SensorEndTouchEvent event = { sensorId, visitorId };
					b3Array_Push( world->sensorEndEvents[endEventArrayIndex], event );
					index1 += 1;
				}
				else
				{
					b3ShapeId visitorId = { r2->shapeId + 1, worldIdShort, r2->generation };
					b3SensorBeginTouchEvent event = { sensorId, visitorId };
					b3Array_Push( world->sensorBeginEvents, event );
					index2 += 1;
				}
			}

			// Tails: whatever one set still has, the other does not.
			while ( index1 < count1 )
			{
				const b3Visitor* r1 = refs1 + index1;
				b3ShapeId visitorId = { r1->shapeId + 1, worldIdShort, r1->generation };
				b3SensorEndTouchEvent event = { sensorId, visitorId };
				b3Array_Push( world->sensorEndEvents[endEventArrayIndex], event );
				index1 += 1;
			}

			while ( index2 < count2 )
			{
				const b3Visitor* r2 = refs2 + index2;
				b3ShapeId visitorId = { r2->shapeId + 1, worldIdShort, r2->generation };
				b3SensorBeginTouchEvent event = { sensorId, visitorId };
				b3Array_Push( world->sensorBeginEvents, event );
				index2 += 1;
			}

			// Clear the lowest set bit.
			word = word & ( word - 1 );
		}
	}
}

void b3DestroySensor( b3World* world, b3Shape* sensorShape )
{
	B3_ASSERT( sensorShape->sensorIndex != B3_NULL_INDEX );

	b3Sensor* sensor = b3Array_Get( world->sensors, sensorShape->sensorIndex );

	// Everything still inside it stops being inside it.
	for ( int i = 0; i < sensor->overlaps2.count; ++i )
	{
		const b3Visitor* ref = sensor->overlaps2.data + i;
		b3SensorEndTouchEvent event = {
			.sensorShapeId = { sensorShape->id + 1, world->worldId, sensorShape->generation },
			.visitorShapeId = { ref->shapeId + 1, world->worldId, ref->generation },
		};

		b3Array_Push( world->sensorEndEvents[world->endEventArrayIndex], event );
	}

	b3Array_Destroy( sensor->hits );
	b3Array_Destroy( sensor->overlaps1 );
	b3Array_Destroy( sensor->overlaps2 );

	int movedIndex = b3Array_RemoveSwap( world->sensors, sensorShape->sensorIndex );
	if ( movedIndex != B3_NULL_INDEX )
	{
		// The sensor that filled the hole has to be told where it now lives.
		b3Sensor* movedSensor = b3Array_Get( world->sensors, sensorShape->sensorIndex );
		b3Shape* otherSensorShape = b3Array_Get( world->shapes, movedSensor->shapeId );
		otherSensorShape->sensorIndex = sensorShape->sensorIndex;
	}

	sensorShape->sensorIndex = B3_NULL_INDEX;
}
