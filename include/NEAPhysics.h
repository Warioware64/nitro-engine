// SPDX-License-Identifier: MIT
//
// Copyright (c) 2008-2022 Antonio Niño Díaz
//
// This file is part of Nitro Engine Advanced

#ifndef NEA_PHYSICS_H__
#define NEA_PHYSICS_H__

/// @file   NEAPhysics.h
/// @brief  Simple physics engine.

#include "NEAModel.h"

/// @defgroup physics Physics engine
///
/// A very simple physics engine. It only supports axis-aligned bounding boxes
/// at the moment.
///
/// @{

#define NEA_DEFAULT_PHYSICS  64 ///< Default max number of physic objects

/// Minimun speed that an object needs to have to rebound after a collision.
///
/// If the object has less speed than this, it will stop after a collision.
#define NEA_MIN_BOUNCE_SPEED (floattof32(0.01))

/// Object types supported by the physics engine.
typedef enum {
    NEA_BoundingBox    = 1, ///< Axis-aligned bounding box.
    NEA_BoundingSphere = 2, ///< Bounding sphere.
    NEA_Dot            = 3  ///< Dot. Use this for really small objects.
} NEA_PhysicsTypes;

/// Possible actions that can happen to an object after a collision.
typedef enum {
    NEA_ColNothing = 0, ///< Ignore the collision.
    NEA_ColBounce,      ///< ReboundBounce against the object.
    NEA_ColStop         ///< Stop.
} NEA_OnCollision;

/// Holds information of a physics object.
//
/// Values are in fixed point (f32).
///
/// The position of the object is obtained from the NEA_Model object, it's not
/// stored inside the physics object.
typedef struct {
    NEA_Model *model;      ///< Model this physics object affects.
    NEA_PhysicsTypes type; ///< Type of physics object (AABB, sphere, dot).
    bool enabled;         ///< True if this object is enabled

    int xspeed;           ///< X speed of model.
    int yspeed;           ///< Y speed of model.
    int zspeed;           ///< Z speed of model.

    int radius;           ///< Radius of a spheres

    int xsize;            ///< X size of an AABB
    int ysize;            ///< Y size of an AABB
    int zsize;            ///< Z size of an AABB

    int gravity;          ///< Intensity of gravity
    int friction;         ///< Intensity of friction

    int keptpercent;  ///< Percentage of energy remaining after a bounce.
    NEA_OnCollision oncollision; ///< Action to do if there is a collision.
    bool iscolliding; ///< True if a collision has been detected.
    int physicsgroup; ///< Objects interact with others in the same group only
} NEA_Physics;

/// Creates a new physics object.
///
/// @param type Type of physics object.
/// @return Pointer to the newly created object.
NEA_Physics *NEA_PhysicsCreate(NEA_PhysicsTypes type);

/// Deletes a physics object.
///
/// @param pointer Pointer to the object.
void NEA_PhysicsDelete(NEA_Physics *pointer);

/// Deletes all physic objects and frees all memory used by them.
void NEA_PhysicsDeleteAll(void);

/// Resets the physics engine and sets the maximun number of objects.
///
/// @param max_objects Number of objects. If it is lower than 1, it will create
///                    space for NEA_DEFAULT_PHYSICS.
/// @return Returns 0 on success.
int NEA_PhysicsSystemReset(int max_objects);

/// Ends physics engine and frees all memory used by it.
void NEA_PhysicsSystemEnd(void);

/// Set radius of a physics object that is a bounding sphere.
///
/// @param pointer Pointer to the object.
/// @param radius Radius of the sphere (f32).
void NEA_PhysicsSetRadiusI(NEA_Physics *pointer, int radius);

/// Set radius of a physics object that is a bounding sphere.
///
/// @param p Pointer to the object.
/// @param r Radius of the sphere (float).
#define NEA_PhysicsSetRadius(p, r) \
    NEA_PhysicsSetRadiusI(p, floattof32(r))

/// Set speed of a physics object.
///
/// @param pointer Pointer to the object.
/// @param x (x, y, z) Speed vector (f32).
/// @param y (x, y, z) Speed vector (f32).
/// @param z (x, y, z) Speed vector (f32).
void NEA_PhysicsSetSpeedI(NEA_Physics *pointer, int x, int y, int z);

/// Set speed of a physics object.
///
/// @param p  Pointer to the object.
/// @param x (x, y, z) Speed vector (float).
/// @param y (x, y, z) Speed vector (float).
/// @param z (x, y, z) Speed vector (float).
#define NEA_PhysicsSetSpeed(p, x, y, z) \
    NEA_PhysicsSetSpeedI(p, floattof32(x), floattof32(y), floattof32(z))

/// Set size of a physics object that is an bounding box.
///
/// @param pointer Pointer to the physics object.
/// @param x (x, y, z) Size (f32).
/// @param y (x, y, z) Size (f32).
/// @param z (x, y, z) Size (f32).
void NEA_PhysicsSetSizeI(NEA_Physics *pointer, int x, int y, int z);

/// Set size of a physics object that is an bounding box.
///
/// @param p Pointer to the physics object.
/// @param x (x, y, z) Size (float).
/// @param y (x, y, z) Size (float).
/// @param z (x, y, z) Size (float).
#define NEA_PhysicsSetSize(p, x, y, z) \
    NEA_PhysicsSetSizeI(p, floattof32(x), floattof32(y), floattof32(z))

/// Set gravity of a physics object.
///
/// @param pointer Pointer to the physics object.
/// @param gravity Gravity on the Y axis (f32).
void NEA_PhysicsSetGravityI(NEA_Physics *pointer, int gravity);

/// Set gravity of a physics object.
///
/// @param p Pointer to the physics object.
/// @param g Gravity on the Y axis (float).
#define NEA_PhysicsSetGravity(p, g) \
    NEA_PhysicsSetGravityI(p, floattof32(g))

/// Set friction of a physics object.
///
/// @param pointer Pointer to the physics object.
/// @param friction Friction (f32).
void NEA_PhysicsSetFrictionI(NEA_Physics *pointer, int friction);

/// Set friction of a physics object.
///
/// @param p Pointer to the physics object.
/// @param f Friction (float).
#define NEA_PhysicsSetFriction(p, f) \
    NEA_PhysicsSetFrictionI(p, floattof32(f))

/// Set percentage of energy kept after a bounce.
///
/// @param pointer Pointer to the physics object.
/// @param percent Percentage of energy kept.
void NEA_PhysicsSetBounceEnergy(NEA_Physics *pointer, int percent);

/// Enable movement of a physics object.
///
/// If disabled, it will never update the position of this object. Use it for
/// objects that interact with others, but that are fixed, like the floor.
///
/// @param pointer Pointer to the physics object.
/// @param value True enables movelent, false disables it.
void NEA_PhysicsEnable(NEA_Physics *pointer, bool value);

/// Assign a model object to a physics object.
///
/// @param physics Pointer to the physics object.
/// @param modelpointer Pointer to the model.
void NEA_PhysicsSetModel(NEA_Physics *physics, NEA_Model *modelpointer);

/// Sets physics group of an object.
///
/// Collisions do only work between objects with the same group. This can be
/// useful when there are two sets of objects that never interact with each
/// other. This way the engine can stop checking interactions between them.
///
/// @param physics Pointer to the object.
/// @param group New physics group number.
void NEA_PhysicsSetGroup(NEA_Physics *physics, int group);

/// Set action to do if this object collides with another one.
///
/// @param physics Pointer to the object.
/// @param action Action.
void NEA_PhysicsOnCollision(NEA_Physics *physics, NEA_OnCollision action);

/// Returns true if given object is colliding.
///
/// This doesn't work with objects that have been disabled with
/// NEA_PhysicsEnable().
///
/// @param pointer Pointer to the object.
/// @return True if there is a collision, false otherwise.
bool NEA_PhysicsIsColliding(const NEA_Physics *pointer);

/// Updates all physics objects.
void NEA_PhysicsUpdateAll(void);

/// Updates the provided physics object.
///
/// @param pointer Pointer to the object.
void NEA_PhysicsUpdate(NEA_Physics *pointer);

/// Returns true if the given objects are colliding.
///
/// It doesn't check physic groups. Two objects in different groups can still
/// collide according to this function.
///
/// @param pointer1 Pointer to first object.
/// @param pointer2 Pointer to second object.
/// @return Returns true if two objects are colliding.
bool NEA_PhysicsCheckCollision(const NEA_Physics *pointer1,
                              const NEA_Physics *pointer2);

/// @}

#endif // NEA_PHYSICS_H__
