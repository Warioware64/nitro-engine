// SPDX-License-Identifier: MIT
//
// Copyright (c) 2024-2026 Warioware64
//
// This file is part of Nitro Engine Advanced

#include "NEAMain.h"
#include "dsma/dsma.h"

/// @file NEABoneCollision.c

// =========================================================================
// .boncol binary format constants
// =========================================================================

#define BNCL_MAGIC   0x4C434E42  // "BNCL" little-endian
#define BNCL_VERSION 1

// .boncol header
typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t num_bones;
    uint32_t reserved;
} boncol_header_t;

// .boncol per-bone entry (32 bytes)
typedef struct {
    uint8_t  type;      // 0=none, 1=sphere, 2=capsule, 3=aabb
    uint8_t  joint_idx; // actual joint index in the animation skeleton
    uint8_t  pad[2];
    int32_t  param1;    // radius (sphere/capsule) or half_x (aabb)
    int32_t  param2;    // half_height (capsule) or half_y (aabb)
    int32_t  param3;    // half_z (aabb) or 0
    int32_t  offset_x;
    int32_t  offset_y;
    int32_t  offset_z;
    int32_t  reserved;
} boncol_bone_t;

// =========================================================================
// CPU-side bone transform helpers
// =========================================================================

// Reuse the quaternion math from dsma.c for CPU-side bone position computation.
// These are simplified versions that only compute the world-space position
// of a bone's collision shape offset, not a full matrix.

// Rotate a vector by a quaternion: v' = q * v * q^-1
// For unit quaternions, q^-1 = conjugate(q) = (w, -x, -y, -z).
static inline NEA_Vec3 ne_quat_rotate_vec(const int32_t *q, NEA_Vec3 v)
{
    // q = [w, x, y, z]
    int32_t qw = q[0], qx = q[1], qy = q[2], qz = q[3];

    // t = 2 * cross(q.xyz, v)
    // cross(q.xyz, v) = (qy*vz - qz*vy, qz*vx - qx*vz, qx*vy - qy*vx)
    int32_t tx = 2 * (mulf32(qy, v.z) - mulf32(qz, v.y));
    int32_t ty = 2 * (mulf32(qz, v.x) - mulf32(qx, v.z));
    int32_t tz = 2 * (mulf32(qx, v.y) - mulf32(qy, v.x));

    // result = v + qw * t + cross(q.xyz, t)
    NEA_Vec3 r;
    r.x = v.x + mulf32(qw, tx) + (mulf32(qy, tz) - mulf32(qz, ty));
    r.y = v.y + mulf32(qw, ty) + (mulf32(qz, tx) - mulf32(qx, tz));
    r.z = v.z + mulf32(qw, tz) + (mulf32(qx, ty) - mulf32(qy, tx));
    return r;
}

// Linear interpolation (same as dsma.c)
static inline int32_t ne_lerp(int32_t start, int32_t end, int32_t pos)
{
    int32_t diff = end - start;
    return start + ((diff * pos) >> 12);
}

// =========================================================================
// DSA file format (replicated from dsma.c for CPU-side access)
// =========================================================================

typedef struct {
    int32_t pos[3];
    int32_t orient[4];
} ne_dsa_joint_t;

typedef struct {
    uint32_t version;
    uint32_t num_frames;
    uint32_t num_joints;
    ne_dsa_joint_t joints[0];
} ne_dsa_t;

static inline const ne_dsa_joint_t *ne_dsa_get_frame(const ne_dsa_t *dsa,
                                                      uint32_t frame)
{
    return &dsa->joints[frame * dsa->num_joints];
}

// Get the interpolated position and orientation for a specific bone.
static void ne_get_bone_transform(const void *dsa_file, uint32_t frame_interp,
                                  uint32_t bone_idx,
                                  int32_t *out_pos, int32_t *out_orient)
{
    const ne_dsa_t *dsa = (const ne_dsa_t *)dsa_file;

    uint32_t frame = frame_interp >> 12;
    uint32_t interp = frame_interp & 0xFFF;

    if (frame >= dsa->num_frames)
        frame = 0;

    if (bone_idx >= dsa->num_joints)
    {
        out_pos[0] = out_pos[1] = out_pos[2] = 0;
        out_orient[0] = inttof32(1);
        out_orient[1] = out_orient[2] = out_orient[3] = 0;
        return;
    }

    const ne_dsa_joint_t *j1 = &ne_dsa_get_frame(dsa, frame)[bone_idx];

    if (interp != 0)
    {
        uint32_t next_frame = frame + 1;
        if (next_frame >= dsa->num_frames)
            next_frame = 0;

        const ne_dsa_joint_t *j2 = &ne_dsa_get_frame(dsa, next_frame)[bone_idx];

        out_pos[0] = ne_lerp(j1->pos[0], j2->pos[0], interp);
        out_pos[1] = ne_lerp(j1->pos[1], j2->pos[1], interp);
        out_pos[2] = ne_lerp(j1->pos[2], j2->pos[2], interp);

        out_orient[0] = ne_lerp(j1->orient[0], j2->orient[0], interp);
        out_orient[1] = ne_lerp(j1->orient[1], j2->orient[1], interp);
        out_orient[2] = ne_lerp(j1->orient[2], j2->orient[2], interp);
        out_orient[3] = ne_lerp(j1->orient[3], j2->orient[3], interp);
    }
    else
    {
        out_pos[0] = j1->pos[0];
        out_pos[1] = j1->pos[1];
        out_pos[2] = j1->pos[2];

        out_orient[0] = j1->orient[0];
        out_orient[1] = j1->orient[1];
        out_orient[2] = j1->orient[2];
        out_orient[3] = j1->orient[3];
    }
}

// =========================================================================
// Loading
// =========================================================================

NEA_BoneCollisionData *NEA_BoneCollisionLoad(const void *data)
{
    NEA_AssertPointer(data, "NULL data pointer");

    const boncol_header_t *hdr = (const boncol_header_t *)data;

    if (hdr->magic != BNCL_MAGIC)
    {
        NEA_DebugPrint("Invalid .boncol magic");
        return NULL;
    }
    if (hdr->version != BNCL_VERSION)
    {
        NEA_DebugPrint("Unsupported .boncol version");
        return NULL;
    }

    NEA_BoneCollisionData *bcd = calloc(1, sizeof(NEA_BoneCollisionData));
    if (bcd == NULL)
    {
        NEA_DebugPrint("Not enough memory");
        return NULL;
    }

    bcd->num_bones = hdr->num_bones;
    if (bcd->num_bones > NEA_MAX_COLLISION_BONES)
        bcd->num_bones = NEA_MAX_COLLISION_BONES;

    const boncol_bone_t *src =
        (const boncol_bone_t *)((const uint8_t *)data + sizeof(boncol_header_t));

    for (uint32_t i = 0; i < bcd->num_bones; i++)
    {
        bcd->bones[i].offset = NEA_Vec3Make(src[i].offset_x,
                                             src[i].offset_y,
                                             src[i].offset_z);
        bcd->bones[i].joint_idx = src[i].joint_idx;

        switch (src[i].type)
        {
        case NEA_BONCOL_TYPE_SPHERE:
            NEA_ColShapeInitSphereI(&bcd->bones[i].shape, src[i].param1);
            break;
        case NEA_BONCOL_TYPE_CAPSULE:
            NEA_ColShapeInitCapsuleI(&bcd->bones[i].shape,
                                     src[i].param2, src[i].param1);
            break;
        case NEA_BONCOL_TYPE_AABB:
            NEA_ColShapeInitAABBI(&bcd->bones[i].shape,
                                  src[i].param1, src[i].param2,
                                  src[i].param3);
            break;
        default:
            bcd->bones[i].shape.type = NEA_COL_NONE;
            break;
        }
    }

    return bcd;
}

NEA_BoneCollisionData *NEA_BoneCollisionLoadFAT(const char *path)
{
    NEA_AssertPointer(path, "NULL path");

    FILE *f = fopen(path, "rb");
    if (f == NULL)
    {
        NEA_DebugPrint("Can't open %s", path);
        return NULL;
    }

    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    fseek(f, 0, SEEK_SET);

    void *data = malloc(size);
    if (data == NULL)
    {
        NEA_DebugPrint("Not enough memory");
        fclose(f);
        return NULL;
    }

    fread(data, 1, size, f);
    fclose(f);

    NEA_BoneCollisionData *bcd = NEA_BoneCollisionLoad(data);
    free(data);
    return bcd;
}

void NEA_BoneCollisionFree(NEA_BoneCollisionData *bcd)
{
    free(bcd);
}

// =========================================================================
// World-space bone collision queries
// =========================================================================

int NEA_BoneCollisionGetWorldShape(const NEA_Model *model,
                                   const NEA_BoneCollisionData *bcd,
                                   int bone_idx,
                                   NEA_Vec3 *out_pos,
                                   NEA_ColShape *out_shape)
{
    NEA_AssertPointer(model, "NULL model pointer");
    NEA_AssertPointer(bcd, "NULL bone collision data");
    NEA_AssertPointer(out_pos, "NULL out_pos");
    NEA_AssertPointer(out_shape, "NULL out_shape");

    if (bone_idx < 0 || (uint32_t)bone_idx >= bcd->num_bones)
        return 0;

    if (bcd->bones[bone_idx].shape.type == NEA_COL_NONE)
        return 0;

    // Get animation data from model
    const NEA_AnimInfo *anim = model->animinfo[0];
    if (anim == NULL || anim->animation == NULL)
        return 0;

    // Use the actual joint index from the binary (not the array index)
    // to look up the correct bone in the animation data.
    uint32_t joint = bcd->bones[bone_idx].joint_idx;

    int32_t bone_pos[3];
    int32_t bone_orient[4];
    ne_get_bone_transform(anim->animation->data, anim->currframe,
                          joint, bone_pos, bone_orient);

    // Transform the collision offset by the bone's quaternion rotation
    NEA_Vec3 rotated_offset = ne_quat_rotate_vec(bone_orient,
                                                  bcd->bones[bone_idx].offset);

    // Local position = bone position + rotated offset (in model space)
    NEA_Vec3 local_pos = NEA_Vec3Add(
        NEA_Vec3Make(bone_pos[0], bone_pos[1], bone_pos[2]),
        rotated_offset);

    // Apply model scale to the local-space position
    local_pos.x = mulf32(local_pos.x, model->sx);
    local_pos.y = mulf32(local_pos.y, model->sy);
    local_pos.z = mulf32(local_pos.z, model->sz);

    // World position = model position + scaled local position
    *out_pos = NEA_Vec3Add(NEA_Vec3Make(model->x, model->y, model->z),
                           local_pos);
    *out_shape = bcd->bones[bone_idx].shape;

    return 1;
}

NEA_ColResult NEA_BoneCollisionTest(const NEA_Model *model,
                                    const NEA_BoneCollisionData *bcd,
                                    const NEA_ColShape *other,
                                    NEA_Vec3 other_pos,
                                    int *out_bone)
{
    NEA_ColResult best = { .hit = false, .depth = 0 };

    NEA_AssertPointer(model, "NULL model pointer");
    NEA_AssertPointer(bcd, "NULL bone collision data");
    NEA_AssertPointer(other, "NULL other shape");

    if (out_bone != NULL)
        *out_bone = -1;

    for (uint32_t i = 0; i < bcd->num_bones; i++)
    {
        if (bcd->bones[i].shape.type == NEA_COL_NONE)
            continue;

        NEA_Vec3 bone_pos;
        NEA_ColShape bone_shape;

        if (!NEA_BoneCollisionGetWorldShape(model, bcd, (int)i,
                                            &bone_pos, &bone_shape))
            continue;

        NEA_ColResult r = NEA_ColTest(&bone_shape, bone_pos, other, other_pos);

        if (r.hit && r.depth > best.depth)
        {
            best = r;
            if (out_bone != NULL)
                *out_bone = (int)i;
        }
    }

    return best;
}
