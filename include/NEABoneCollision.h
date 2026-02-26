// SPDX-License-Identifier: MIT
//
// Copyright (c) 2024-2026 Warioware64
//
// This file is part of Nitro Engine Advanced

#ifndef NEA_BONE_COLLISION_H__
#define NEA_BONE_COLLISION_H__

/// @file   NEABoneCollision.h
/// @brief  Per-bone collision data for animated models.

#include <nds.h>
#include "NEACollision.h"
#include "NEAModel.h"

/// @defgroup bone_collision Bone collision system
///
/// Per-bone collision primitives for animated (skeletal) models. Each bone
/// can have a collision shape (sphere, capsule, or AABB) attached to it.
/// At runtime, the shapes are transformed by the bone's animation state
/// to provide accurate collision detection for animated characters.
///
/// @{

/// Maximum bones supported for collision (matches DSMA matrix stack limit).
#define NEA_MAX_COLLISION_BONES 30

/// Bone collision shape types in the binary .boncol format.
#define NEA_BONCOL_TYPE_NONE    0
#define NEA_BONCOL_TYPE_SPHERE  1
#define NEA_BONCOL_TYPE_CAPSULE 2
#define NEA_BONCOL_TYPE_AABB    3

/// Per-bone collision primitive.
typedef struct {
    NEA_ColShape shape;      ///< Collision shape for this bone.
    NEA_Vec3     offset;     ///< Offset in bone-local space (f32).
    uint8_t      joint_idx;  ///< Actual joint index in the animation skeleton.
} NEA_BoneColPrimitive;

/// Bone collision data for an animated model.
typedef struct {
    uint32_t num_bones;  ///< Number of bones with collision data.
    NEA_BoneColPrimitive bones[NEA_MAX_COLLISION_BONES]; ///< Per-bone data.
} NEA_BoneCollisionData;

/// Load bone collision data from a .boncol binary in RAM.
///
/// @param data Pointer to the .boncol binary data.
/// @return Pointer to the loaded data, or NULL on error.
NEA_BoneCollisionData *NEA_BoneCollisionLoad(const void *data);

/// Load bone collision data from a .boncol file on the filesystem (FAT).
///
/// @param path Path to the .boncol file.
/// @return Pointer to the loaded data, or NULL on error.
NEA_BoneCollisionData *NEA_BoneCollisionLoadFAT(const char *path);

/// Free bone collision data.
///
/// @param bcd Pointer to the data to free.
void NEA_BoneCollisionFree(NEA_BoneCollisionData *bcd);

/// Get the world-space position of a bone's collision shape.
///
/// Uses the model's current animation state to compute the bone's
/// world-space transform, then applies the collision shape offset.
///
/// @param model Pointer to the animated model.
/// @param bcd Bone collision data for this model.
/// @param bone_idx Bone index (0-based).
/// @param out_pos Output: world-space position of the collision shape.
/// @param out_shape Output: the collision shape (local params, no transform).
/// @return 1 on success, 0 on error.
int NEA_BoneCollisionGetWorldShape(const NEA_Model *model,
                                   const NEA_BoneCollisionData *bcd,
                                   int bone_idx,
                                   NEA_Vec3 *out_pos,
                                   NEA_ColShape *out_shape);

/// Test collision between an animated model's bones and a collision shape.
///
/// Tests all bones that have collision data against the given shape.
/// Returns the deepest collision found and the index of the colliding bone.
///
/// @param model The animated model.
/// @param bcd Bone collision data for the model.
/// @param other The other collision shape to test against.
/// @param other_pos World position of the other shape (f32).
/// @param out_bone Output: index of the colliding bone (-1 if none).
/// @return Collision result (deepest penetration among all bones).
NEA_ColResult NEA_BoneCollisionTest(const NEA_Model *model,
                                    const NEA_BoneCollisionData *bcd,
                                    const NEA_ColShape *other,
                                    NEA_Vec3 other_pos,
                                    int *out_bone);

/// @}

#endif // NEA_BONE_COLLISION_H__
