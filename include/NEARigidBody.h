// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Warioware64
//
// This file is part of Nitro Engine Advanced

#ifndef NEA_RIGIDBODY_H__
#define NEA_RIGIDBODY_H__

/// @file   NEARigidBody.h
/// @brief  ARM7-accelerated rigid body physics with OBB collision.
///
/// This module provides a rigid body physics engine that runs on the ARM7 CPU.
/// The ARM9 API sends commands via FIFO and receives position/rotation updates
/// each frame. Requires using one of the NEA ARM7 binaries:
///
///   - arm7_nea.elf         (basic, no audio)
///   - arm7_nea_maxmod.elf  (with Maxmod audio)
///
/// Set ARM7ELF in your Makefile:
///   ARM7ELF := $(BLOCKSDSEXT)/nitro-engine-advanced/arm7/arm7_nea.elf

#include "NEAModel.h"
#include "NEACollision.h"
#include "NEA_RB_IPC.h"

/// @defgroup rigidbody Rigid body physics (ARM7)
///
/// ARM7-accelerated OBB rigid body physics with angular dynamics, impulse-based
/// collision response, Coulomb friction, and object sleeping.
///
/// @{

/// Rigid body proxy (ARM9 side). Position and rotation are synced from ARM7
/// each frame via NEA_RigidBodySync().
typedef struct NEA_RigidBody_ {
    uint8_t id;            ///< Index in ARM7's body array.
    bool active;           ///< True if this body exists on ARM7.
    NEA_Model *model;      ///< Visual model to update.

    /// Cached state from ARM7 (read-only on ARM9, updated by Sync).
    NEA_Vec3 position;     ///< Last known position (f32).
    int32_t rotation[9];   ///< Last known 3x3 rotation matrix (row-major, f32).
    bool sleeping;         ///< True if ARM7 has put this body to sleep.

    int32_t half_extents[3]; ///< OBB half-extents (f32), cached for collision bridge.
} NEA_RigidBody;

// =========================================================================
// System
// =========================================================================

/// Initialize the rigid body system and send START to ARM7.
///
/// Call this once after your 3D system is initialized. Requires that the
/// NDS ROM uses an NEA ARM7 binary (arm7_nea.elf or arm7_nea_maxmod.elf).
///
/// @return 0 on success.
int NEA_RigidBodyInit(void);

/// Shut down the rigid body system. Sends RESET to ARM7 and frees resources.
void NEA_RigidBodyEnd(void);

/// Synchronize rigid body state from ARM7.
///
/// Reads position and rotation updates sent by ARM7 via FIFO and updates
/// the linked NEA_Model transforms. Call this once per frame, typically
/// before rendering.
void NEA_RigidBodySync(void);

// =========================================================================
// Body creation / destruction
// =========================================================================

/// Create a rigid body (OBB shape).
///
/// Sends ADD_BODY command to ARM7. The body starts at the origin with
/// zero velocity. Use SetPosition/SetVelocity to configure initial state.
///
/// @param mass Mass (f32). Must be > 0 for dynamic bodies.
/// @param hx Half-extent on X axis (f32).
/// @param hy Half-extent on Y axis (f32).
/// @param hz Half-extent on Z axis (f32).
/// @return Pointer to the rigid body proxy, or NULL if no slots available.
NEA_RigidBody *NEA_RigidBodyCreateI(int32_t mass,
                                     int32_t hx, int32_t hy, int32_t hz);

/// Create a rigid body (float parameters).
#define NEA_RigidBodyCreate(mass, hx, hy, hz) \
    NEA_RigidBodyCreateI(floattof32(mass), \
                          floattof32(hx), floattof32(hy), floattof32(hz))

/// Delete a rigid body.
///
/// @param rb Pointer to the rigid body proxy.
void NEA_RigidBodyDelete(NEA_RigidBody *rb);

// =========================================================================
// Configuration
// =========================================================================

/// Link a visual model to this rigid body.
///
/// The model's position and rotation will be updated automatically by
/// NEA_RigidBodySync().
///
/// @param rb Pointer to the rigid body.
/// @param model Pointer to the model.
void NEA_RigidBodySetModel(NEA_RigidBody *rb, NEA_Model *model);

/// Set position (teleport). Wakes the body if sleeping.
void NEA_RigidBodySetPositionI(NEA_RigidBody *rb,
                                int32_t x, int32_t y, int32_t z);

/// Set position (float).
#define NEA_RigidBodySetPosition(rb, x, y, z) \
    NEA_RigidBodySetPositionI(rb, floattof32(x), floattof32(y), floattof32(z))

/// Set linear velocity. Wakes the body if sleeping.
void NEA_RigidBodySetVelocityI(NEA_RigidBody *rb,
                                int32_t vx, int32_t vy, int32_t vz);

/// Set linear velocity (float).
#define NEA_RigidBodySetVelocity(rb, vx, vy, vz) \
    NEA_RigidBodySetVelocityI(rb, floattof32(vx), floattof32(vy), floattof32(vz))

/// Set coefficient of restitution (bounciness, f32, 0.0 to 1.0).
void NEA_RigidBodySetRestitutionI(NEA_RigidBody *rb, int32_t e);

/// Set coefficient of restitution (float).
#define NEA_RigidBodySetRestitution(rb, e) \
    NEA_RigidBodySetRestitutionI(rb, floattof32(e))

/// Set Coulomb friction coefficient (f32, 0.0 to 1.0).
void NEA_RigidBodySetFrictionI(NEA_RigidBody *rb, int32_t mu);

/// Set friction (float).
#define NEA_RigidBodySetFriction(rb, mu) \
    NEA_RigidBodySetFrictionI(rb, floattof32(mu))

// =========================================================================
// Forces and impulses
// =========================================================================

/// Apply a force at a world-space point (f32).
///
/// The force is accumulated for the current simulation step. The offset
/// from the body's center of mass produces torque.
void NEA_RigidBodyApplyForceI(NEA_RigidBody *rb,
                               int32_t fx, int32_t fy, int32_t fz,
                               int32_t px, int32_t py, int32_t pz);

/// Apply a force at a point (float).
#define NEA_RigidBodyApplyForce(rb, fx, fy, fz, px, py, pz) \
    NEA_RigidBodyApplyForceI(rb, \
        floattof32(fx), floattof32(fy), floattof32(fz), \
        floattof32(px), floattof32(py), floattof32(pz))

/// Apply an instantaneous impulse at a world-space point (f32).
///
/// Immediately changes the body's linear and angular velocity.
void NEA_RigidBodyApplyImpulseI(NEA_RigidBody *rb,
                                 int32_t jx, int32_t jy, int32_t jz,
                                 int32_t px, int32_t py, int32_t pz);

/// Apply an impulse at a point (float).
#define NEA_RigidBodyApplyImpulse(rb, jx, jy, jz, px, py, pz) \
    NEA_RigidBodyApplyImpulseI(rb, \
        floattof32(jx), floattof32(jy), floattof32(jz), \
        floattof32(px), floattof32(py), floattof32(pz))

// =========================================================================
// Global settings
// =========================================================================

/// Set global gravity (Y-axis, f32). Applied to all dynamic bodies.
///
/// Typical value: floattof32(-0.1) for gentle gravity.
void NEA_RigidBodySetGravityI(int32_t g);

/// Set global gravity (float).
#define NEA_RigidBodySetGravity(g) \
    NEA_RigidBodySetGravityI(floattof32(g))

// =========================================================================
// Static colliders (walls, floors, ceilings)
// =========================================================================

/// Add a static axis-aligned rectangle collider (f32).
///
/// Static colliders are infinite-mass surfaces that rigid bodies bounce off.
/// The normal determines which side of the rectangle is solid.
///
/// @param px,py,pz Center position (f32).
/// @param sx,sy,sz Half-extents on each axis (f32). Set the axis aligned
///                 with normal to 0 (it's a flat surface).
/// @param nx,ny,nz Surface normal (will be normalized, f32).
/// @return Static collider ID (>= 0), or -1 on error.
int NEA_RigidBodyAddStaticI(int32_t px, int32_t py, int32_t pz,
                             int32_t sx, int32_t sy, int32_t sz,
                             int32_t nx, int32_t ny, int32_t nz);

/// Add a static collider (float).
#define NEA_RigidBodyAddStatic(px, py, pz, sx, sy, sz, nx, ny, nz) \
    NEA_RigidBodyAddStaticI( \
        floattof32(px), floattof32(py), floattof32(pz), \
        floattof32(sx), floattof32(sy), floattof32(sz), \
        floattof32(nx), floattof32(ny), floattof32(nz))

/// Remove a static collider by ID.
void NEA_RigidBodyRemoveStatic(int id);

// =========================================================================
// Queries
// =========================================================================

/// Get the last known position (from ARM7 sync).
NEA_Vec3 NEA_RigidBodyGetPosition(const NEA_RigidBody *rb);

/// Returns true if ARM7 has put this body to sleep (very low energy).
bool NEA_RigidBodyIsSleeping(const NEA_RigidBody *rb);

// =========================================================================
// Collision system integration
// =========================================================================

/// Test a rigid body against a NEA collision shape and inject contacts.
///
/// Constructs an AABB from the rigid body's position and half-extents,
/// tests it against the given collision shape using NEA_ColTest(), and
/// sends any resulting contact to ARM7 via FIFO for impulse resolution.
///
/// Call this once per frame for each (rigid body, shape) pair you want
/// to interact. Typically used for ColMesh level geometry.
///
/// @param rb   Pointer to the rigid body.
/// @param shape Pointer to the NEA collision shape.
/// @param sx,sy,sz World-space position of the collision shape (f32).
/// @return Number of contacts injected (0 or 1).
int NEA_RigidBodyCollideWithI(NEA_RigidBody *rb,
                               const NEA_ColShape *shape,
                               int32_t sx, int32_t sy, int32_t sz);

/// Test rigid body against collision shape (float positions).
#define NEA_RigidBodyCollideWith(rb, shape, x, y, z) \
    NEA_RigidBodyCollideWithI((rb), (shape), \
        floattof32(x), floattof32(y), floattof32(z))

/// @}

#endif // NEA_RIGIDBODY_H__
