// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Erin Catto        (original Box3D)
// Copyright (c) 2026 Warioware64       (Nitro Engine Advanced fixed-point port)
//
// This file is part of Nitro Engine Advanced

#pragma once

/// @file   sensor.h
/// @brief  Sensor shapes: overlap detection without a collision response.
///
/// A sensor is a shape that reports what is inside it rather than pushing it
/// out. It never forms a contact -- the broad phase skips any pair with one --
/// so it is not part of the solver at all, and its whole cost is one query per
/// sensor per step.
///
/// @section sets What makes this the awkward subsystem
///
/// Every other event in the library is produced by something that happened:
/// a contact began touching, a joint crossed its threshold. A sensor event is
/// produced by a *difference between two sets* -- what overlapped last step
/// against what overlaps now -- so a sensor has to keep both, keep them
/// sorted, and walk them together. That is why b3Sensor carries two overlap
/// arrays and swaps them, and it is the reason this file has fixed capacities
/// where upstream has none: the port cannot grow an array during a step, and
/// these are the only arrays in the library whose length is decided by the
/// scene rather than by a b3Capacity count of objects.
///
/// See B3_NEA_MAX_SENSOR_VISITORS for the cap, and
/// b3World::sensorOverlapDropCount for what happens when it binds.
///
/// @section threads What is not here
///
/// Upstream runs the sensor pass as a b3ParallelFor over b3SensorTask with a
/// b3SensorTaskContext per worker, each holding its own b3BitSet of "this
/// sensor's overlap set changed", unioned afterwards. There is one worker
/// here, so the task, the context and the union all collapse: the bitset is
/// b3World::sensorEventBitSet, and b3OverlapSensors is a plain loop. Same
/// treatment, and the same argument, as solver.c's bullet pass.

#include "bitset.h"
#include "container.h"

#include <stdint.h>

typedef struct b3Shape b3Shape;
typedef struct b3World b3World;

/// A shape overlapping a sensor, by index and generation.
///
/// The generation is what makes this a reference rather than an index: a shape
/// can be destroyed and its slot reused between steps, and an end-touch event
/// for the shape that left must not be swallowed by the begin-touch of the one
/// that took its place. The event walk in b3OverlapSensors compares both.
typedef struct b3Visitor
{
	int shapeId;
	uint16_t generation;
} b3Visitor;

/// A shape that a continuous sweep passed through without ending inside it.
///
/// Produced by solver.c's continuous pass and consumed here. It exists because
/// a bullet can cross a trigger volume completely within one step, so the
/// end-of-step overlap query -- which is all a sensor otherwise does -- would
/// never see it.
typedef struct b3SensorHit
{
	int sensorId;
	int visitorId;
} b3SensorHit;

b3DeclareArray( b3Visitor );

typedef struct b3Sensor
{
	/// Visitors found by the continuous pass during the step, folded into
	/// overlaps2 at the start of the next sensor pass and then cleared.
	b3Array( b3Visitor ) hits;

	/// Last step's overlap set, and this step's. Swapped every pass, both kept
	/// sorted by shapeId, and walked together to produce begin and end events.
	b3Array( b3Visitor ) overlaps1;
	b3Array( b3Visitor ) overlaps2;

	int shapeId;
} b3Sensor;

b3DeclareArray( b3Sensor );

/// Reserve one sensor's three arrays at their fixed capacities.
///
/// Called from b3CreateShapeInternal when a shape def asks to be a sensor,
/// which happens while the scene is being built rather than during a step.
void b3CreateSensor( b3World* world, b3Shape* sensorShape );

/// Rebuild every sensor's overlap set and publish the begin and end events.
///
/// Runs once per b3World_Step, after the solve -- so the poses it queries are
/// the ones the step ended at, and so the continuous pass has already
/// deposited its hits.
void b3OverlapSensors( b3World* world );

/// End every overlap this sensor still has, then remove it from the world.
void b3DestroySensor( b3World* world, b3Shape* sensorShape );
