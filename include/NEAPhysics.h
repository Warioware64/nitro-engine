// SPDX-License-Identifier: MIT
//
// Copyright (c) 2008-2022 Antonio Niño Díaz
// Copyright (c) 2024-2026 Warioware64
//
// This file is part of Nitro Engine Advanced

#ifndef NEA_PHYSICS_H__
#define NEA_PHYSICS_H__

/// @file   NEAPhysics.h
/// @brief  Physics engine with multiple collision shapes and impulse response.

#include "NEAModel.h"
#include "NEACollision.h"

/// @defgroup physics Physics engine
///
/// Physics engine supporting AABB, sphere, capsule, and triangle mesh
/// colliders with gravity, friction, bounce/stop/slide responses, mass-based
/// impulse, and collision group filtering.
///
/// @{

#define NEA_DEFAULT_PHYSICS  64 ///< Default max number of physics objects.

/// Minimum speed that an object needs to have to rebound after a collision.
///
/// If the object has less speed than this, it will stop after a collision.
#define NEA_MIN_BOUNCE_SPEED (floattof32(0.01))

/// Legacy object types (kept for backward compatibility).
///
/// Prefer using NEA_PhysicsCreateEx() with NEA_ColShapeType for new code.
typedef enum {
    NEA_BoundingBox    = 1, ///< Axis-aligned bounding box.
    NEA_BoundingSphere = 2, ///< Bounding sphere.
    NEA_Dot            = 3  ///< Dot. Use this for really small objects.
} NEA_PhysicsTypes;

/// Possible actions that can happen to an object after a collision.
typedef enum {
    NEA_ColNothing = 0, ///< Ignore the collision (sensor/trigger).
    NEA_ColBounce,      ///< Bounce against the object.
    NEA_ColStop,        ///< Stop.
    NEA_ColSlide        ///< Slide along the collision surface.
} NEA_OnCollision;

/// Forward declaration for collision callback.
typedef struct NEA_Physics_ NEA_Physics;

/// Collision callback function type.
///
/// Called when this object collides with another. The callback receives
/// pointers to both objects and the collision result.
///
/// @param self Pointer to this physics object.
/// @param other Pointer to the other physics object.
/// @param result Pointer to the collision result.
typedef void (*NEA_CollisionCallback)(NEA_Physics *self,
                                      NEA_Physics *other,
                                      const NEA_ColResult *result);

/// Holds information of a physics object.
///
/// Values are in fixed point (f32). The position of the object is obtained
/// from the NEA_Model object, it's not stored inside the physics object.
struct NEA_Physics_ {
    NEA_Model *model;      ///< Model this physics object affects.
    NEA_ColShape col_shape; ///< Collision shape used for detection.

    NEA_PhysicsTypes type; ///< Legacy type (synced with col_shape).
    bool enabled;          ///< True if this object is enabled.

    int xspeed;            ///< X speed of model (f32).
    int yspeed;            ///< Y speed of model (f32).
    int zspeed;            ///< Z speed of model (f32).

    int radius;            ///< Legacy: radius of sphere (f32).

    int xsize;             ///< Legacy: X size of AABB (f32).
    int ysize;             ///< Legacy: Y size of AABB (f32).
    int zsize;             ///< Legacy: Z size of AABB (f32).

    int gravity;           ///< Intensity of gravity on Y axis (f32).
    int friction;          ///< Intensity of friction (f32).

    int keptpercent;       ///< Percentage of energy remaining after bounce.
    NEA_OnCollision oncollision; ///< Action to do if there is a collision.
    bool iscolliding;      ///< True if a collision has been detected.

    uint32_t groupmask;    ///< Group bitmask. Collides if (a & b) != 0.

    // --- Extended fields ---

    int32_t mass;          ///< Mass (f32). 0 = infinite/static.
    int32_t restitution;   ///< Coefficient of restitution (f32, 0..4096).

    NEA_CollisionCallback on_collision_cb; ///< Optional collision callback.

    bool is_static;        ///< If true, never moves (floor, walls, etc.).
};

/// Creates a new physics object (legacy API).
///
/// Supports NEA_BoundingBox (AABB), NEA_BoundingSphere (sphere), and
/// NEA_Dot (small sphere). For new code, prefer NEA_PhysicsCreateEx().
///
/// @param type Type of physics object.
/// @return Pointer to the newly created object.
NEA_Physics *NEA_PhysicsCreate(NEA_PhysicsTypes type);

/// Creates a new physics object with a specific collision shape type.
///
/// @param type Collision shape type (NEA_COL_AABB, NEA_COL_SPHERE, etc.).
/// @return Pointer to the newly created object.
NEA_Physics *NEA_PhysicsCreateEx(NEA_ColShapeType type);

/// Deletes a physics object.
///
/// @param pointer Pointer to the object.
void NEA_PhysicsDelete(NEA_Physics *pointer);

/// Deletes all physics objects and frees all memory used by them.
void NEA_PhysicsDeleteAll(void);

/// Resets the physics engine and sets the maximum number of objects.
///
/// @param max_objects Number of objects. If it is lower than 1, it will create
///                    space for NEA_DEFAULT_PHYSICS.
/// @return Returns 0 on success.
int NEA_PhysicsSystemReset(int max_objects);

/// Ends physics engine and frees all memory used by it.
void NEA_PhysicsSystemEnd(void);

/// Set the collision shape of a physics object.
///
/// @param physics Pointer to the physics object.
/// @param shape Pointer to the collision shape to copy.
void NEA_PhysicsSetColShape(NEA_Physics *physics, const NEA_ColShape *shape);

/// Set radius of a physics object that is a bounding sphere (f32).
///
/// @param pointer Pointer to the object.
/// @param radius Radius of the sphere (f32).
void NEA_PhysicsSetRadiusI(NEA_Physics *pointer, int radius);

/// Set radius of a physics object that is a bounding sphere (float).
///
/// @param p Pointer to the object.
/// @param r Radius of the sphere (float).
#define NEA_PhysicsSetRadius(p, r) \
    NEA_PhysicsSetRadiusI(p, floattof32(r))

/// Set speed of a physics object (f32).
///
/// @param pointer Pointer to the object.
/// @param x (x, y, z) Speed vector (f32).
/// @param y (x, y, z) Speed vector (f32).
/// @param z (x, y, z) Speed vector (f32).
void NEA_PhysicsSetSpeedI(NEA_Physics *pointer, int x, int y, int z);

/// Set speed of a physics object (float).
///
/// @param p  Pointer to the object.
/// @param x (x, y, z) Speed vector (float).
/// @param y (x, y, z) Speed vector (float).
/// @param z (x, y, z) Speed vector (float).
#define NEA_PhysicsSetSpeed(p, x, y, z) \
    NEA_PhysicsSetSpeedI(p, floattof32(x), floattof32(y), floattof32(z))

/// Set size of a physics object that is a bounding box (f32).
///
/// @param pointer Pointer to the physics object.
/// @param x (x, y, z) Size (f32).
/// @param y (x, y, z) Size (f32).
/// @param z (x, y, z) Size (f32).
void NEA_PhysicsSetSizeI(NEA_Physics *pointer, int x, int y, int z);

/// Set size of a physics object that is a bounding box (float).
///
/// @param p Pointer to the physics object.
/// @param x (x, y, z) Size (float).
/// @param y (x, y, z) Size (float).
/// @param z (x, y, z) Size (float).
#define NEA_PhysicsSetSize(p, x, y, z) \
    NEA_PhysicsSetSizeI(p, floattof32(x), floattof32(y), floattof32(z))

/// Set gravity of a physics object (f32).
///
/// @param pointer Pointer to the physics object.
/// @param gravity Gravity on the Y axis (f32).
void NEA_PhysicsSetGravityI(NEA_Physics *pointer, int gravity);

/// Set gravity of a physics object (float).
///
/// @param p Pointer to the physics object.
/// @param g Gravity on the Y axis (float).
#define NEA_PhysicsSetGravity(p, g) \
    NEA_PhysicsSetGravityI(p, floattof32(g))

/// Set friction of a physics object (f32).
///
/// @param pointer Pointer to the physics object.
/// @param friction Friction (f32).
void NEA_PhysicsSetFrictionI(NEA_Physics *pointer, int friction);

/// Set friction of a physics object (float).
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
/// @param value True enables movement, false disables it.
void NEA_PhysicsEnable(NEA_Physics *pointer, bool value);

/// Assign a model object to a physics object.
///
/// @param physics Pointer to the physics object.
/// @param modelpointer Pointer to the model.
void NEA_PhysicsSetModel(NEA_Physics *physics, NEA_Model *modelpointer);

/// Sets physics group of an object (legacy integer API).
///
/// Internally sets groupmask to (1 << group). For more flexible filtering,
/// use NEA_PhysicsSetGroupMask().
///
/// @param physics Pointer to the object.
/// @param group New physics group number (0-31).
void NEA_PhysicsSetGroup(NEA_Physics *physics, int group);

/// Sets physics group mask of an object.
///
/// Collisions only happen between objects whose group masks overlap:
/// (mask_a & mask_b) != 0. This allows objects to be in multiple groups.
///
/// @param physics Pointer to the object.
/// @param mask Bitmask of groups this object belongs to.
void NEA_PhysicsSetGroupMask(NEA_Physics *physics, uint32_t mask);

/// Set action to do if this object collides with another one.
///
/// @param physics Pointer to the object.
/// @param action Action (NEA_ColNothing, NEA_ColBounce, NEA_ColStop,
///               NEA_ColSlide).
void NEA_PhysicsOnCollision(NEA_Physics *physics, NEA_OnCollision action);

/// Set mass of a physics object (f32).
///
/// Mass affects impulse-based collision response. A mass of 0 means
/// infinite mass (the object won't be pushed by collisions).
///
/// @param physics Pointer to the physics object.
/// @param mass Mass value (f32).
void NEA_PhysicsSetMassI(NEA_Physics *physics, int32_t mass);

/// Set mass of a physics object (float).
///
/// @param p Pointer to the physics object.
/// @param m Mass value (float).
#define NEA_PhysicsSetMass(p, m) \
    NEA_PhysicsSetMassI(p, floattof32(m))

/// Set coefficient of restitution (f32).
///
/// Controls how bouncy a collision is. 0 = perfectly inelastic,
/// 4096 (= 1.0 in f32) = perfectly elastic.
///
/// @param physics Pointer to the physics object.
/// @param rest Restitution coefficient (f32, 0..4096).
void NEA_PhysicsSetRestitutionI(NEA_Physics *physics, int32_t rest);

/// Set coefficient of restitution (float).
///
/// @param p Pointer to the physics object.
/// @param r Restitution coefficient (float, 0.0..1.0).
#define NEA_PhysicsSetRestitution(p, r) \
    NEA_PhysicsSetRestitutionI(p, floattof32(r))

/// Set a collision callback function.
///
/// The callback is called every time this object collides with another.
/// Set to NULL to disable.
///
/// @param physics Pointer to the physics object.
/// @param cb Callback function, or NULL to disable.
void NEA_PhysicsSetCallback(NEA_Physics *physics, NEA_CollisionCallback cb);

/// Set whether this object is static (never moves).
///
/// Static objects act as immovable obstacles. They participate in collision
/// detection but their position and velocity are never modified.
///
/// @param physics Pointer to the physics object.
/// @param is_static True for static, false for dynamic.
void NEA_PhysicsSetStatic(NEA_Physics *physics, bool is_static);

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
/// Uses the new collision system for all shape pairs. It doesn't check
/// physics groups — two objects in different groups can still collide
/// according to this function.
///
/// @param pointer1 Pointer to first object.
/// @param pointer2 Pointer to second object.
/// @return Returns true if two objects are colliding.
bool NEA_PhysicsCheckCollision(const NEA_Physics *pointer1,
                              const NEA_Physics *pointer2);

/// @}

#endif // NEA_PHYSICS_H__
