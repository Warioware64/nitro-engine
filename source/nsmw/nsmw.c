// SPDX-License-Identifier: MIT
//
// Copyright (c) 2008-2024 Antonio Niño Díaz
//
// This file is part of Nitro Engine Advanced

// NSMW (NitroSkin MultiWeight) two-weight skinning runtime.
//
// See include/NEANodeSkin.h for an overview of the technique. In short: per
// frame, this builds for every node a 4x3 matrix that is the weighted sum of
// the skinning matrices (M_anim * M_bind_inverse) of one or two joints, and
// stores each into the hardware matrix stack so that the model's display list
// (whose MTX_RESTORE commands reference node slots) can be drawn.

#include <nds.h>

#include "NEAMain.h"

// Shared DSA structures and interpolation helpers (also used by dsma.c).
#include "../dsma/dsma_internal.h"

// Private helpers
// ===============

// Default scale for v1 joints (1.0 in f32, per axis).
static const int32_t nsmw_unit_scale[3] = { 1 << 12, 1 << 12, 1 << 12 };

// Builds a 4x3 matrix from a quaternion orientation, a translation and a
// per-axis scale, writing it into 'out' (12 int32 values). The element order
// matches what MATRIX_LOAD4x3 / MATRIX_MULT4x3 expect: three columns of the 3x3
// rotation followed by the translation. This mirrors matrix_mult_by_joint_scaled
// in dsma.c, but writes to memory instead of the geometry engine FIFO.
//
// Using unit scale produces results identical to the unscaled DSMA path, so this
// single builder handles both v1 and v2 joints.
ITCM_CODE ARM_CODE static inline
void cpu_joint_to_m4x3(const int32_t *v, const int32_t *q, const int32_t *s,
                       int32_t *out)
{
    int32_t wx = mulf32_by_2(q[0], q[1]);
    int32_t wy = mulf32_by_2(q[0], q[2]);
    int32_t wz = mulf32_by_2(q[0], q[3]);
    int32_t x2 = mulf32_by_2(q[1], q[1]);
    int32_t xy = mulf32_by_2(q[1], q[2]);
    int32_t xz = mulf32_by_2(q[1], q[3]);
    int32_t y2 = mulf32_by_2(q[2], q[2]);
    int32_t yz = mulf32_by_2(q[2], q[3]);
    int32_t z2 = mulf32_by_2(q[3], q[3]);

    int32_t one = inttof32(1);

    out[0] = mulf32(one - y2 - z2, s[0]);
    out[1] = mulf32(xy + wz, s[0]);
    out[2] = mulf32(xz - wy, s[0]);

    out[3] = mulf32(xy - wz, s[1]);
    out[4] = mulf32(one - x2 - z2, s[1]);
    out[5] = mulf32(yz + wx, s[1]);

    out[6] = mulf32(xz + wy, s[2]);
    out[7] = mulf32(yz - wx, s[2]);
    out[8] = mulf32(one - x2 - y2, s[2]);

    out[9] = v[0];
    out[10] = v[1];
    out[11] = v[2];
}

// Multiplies two affine 4x3 matrices: out = a * b (apply b, then a). Both are
// stored as three columns of the 3x3 rotation followed by the translation.
ITCM_CODE ARM_CODE static inline
void m4x3_mul(const int32_t *a, const int32_t *b, int32_t *out)
{
    for (int col = 0; col < 3; col++)
    {
        for (int row = 0; row < 3; row++)
        {
            int32_t acc = mulf32(a[0 * 3 + row], b[col * 3 + 0])
                        + mulf32(a[1 * 3 + row], b[col * 3 + 1])
                        + mulf32(a[2 * 3 + row], b[col * 3 + 2]);
            out[col * 3 + row] = acc;
        }
    }

    for (int row = 0; row < 3; row++)
    {
        int32_t acc = a[9 + row]
                    + mulf32(a[0 * 3 + row], b[9 + 0])
                    + mulf32(a[1 * 3 + row], b[9 + 1])
                    + mulf32(a[2 * 3 + row], b[9 + 2]);
        out[9 + row] = acc;
    }
}

// Interpolates a single joint's transform from one animation, storing the
// position, orientation quaternion and scale into the output arrays. v1 joints
// get a unit scale.
ITCM_CODE ARM_CODE static inline
void nsmw_interp_one_joint(const dsa_header_t *hdr, uint32_t version,
                           uint32_t frame, uint32_t next_frame, uint32_t interp,
                           uint32_t j,
                           int32_t *v_pos, int32_t *q_orient, int32_t *v_scale)
{
    if (version == DSA_VERSION_2)
    {
        uint32_t stride = sizeof(dsa_joint_v2_t);
        const dsa_joint_v2_t *p1 = (const dsa_joint_v2_t *)
            ((const uint8_t *)dsa_get_frame_ptr(hdr, frame, stride) + j * stride);

        if (interp != 0)
        {
            const dsa_joint_v2_t *p2 = (const dsa_joint_v2_t *)
                ((const uint8_t *)dsa_get_frame_ptr(hdr, next_frame, stride)
                 + j * stride);

            dsa_interpolate_frames_v2(p1->pos, p1->orient, p1->scale,
                                      p2->pos, p2->orient, p2->scale,
                                      interp, v_pos, q_orient, v_scale);
        }
        else
        {
            for (int k = 0; k < 3; k++) v_pos[k] = p1->pos[k];
            for (int k = 0; k < 4; k++) q_orient[k] = p1->orient[k];
            for (int k = 0; k < 3; k++) v_scale[k] = p1->scale[k];
        }
    }
    else // DSA_VERSION_1
    {
        uint32_t stride = sizeof(dsa_joint_v1_t);
        const dsa_joint_v1_t *p1 = (const dsa_joint_v1_t *)
            ((const uint8_t *)dsa_get_frame_ptr(hdr, frame, stride) + j * stride);

        v_scale[0] = nsmw_unit_scale[0];
        v_scale[1] = nsmw_unit_scale[1];
        v_scale[2] = nsmw_unit_scale[2];

        if (interp != 0)
        {
            const dsa_joint_v1_t *p2 = (const dsa_joint_v1_t *)
                ((const uint8_t *)dsa_get_frame_ptr(hdr, next_frame, stride)
                 + j * stride);

            dsa_interpolate_frames(p1->pos, p1->orient, p2->pos, p2->orient,
                                   interp, v_pos, q_orient);
        }
        else
        {
            for (int k = 0; k < 3; k++) v_pos[k] = p1->pos[k];
            for (int k = 0; k < 4; k++) q_orient[k] = p1->orient[k];
        }
    }
}

// Computes the skinning matrix (M_anim * invbind) for one joint of a single
// animation, writing it into 'out' (12 int32 values).
ITCM_CODE ARM_CODE static inline
void nsmw_joint_skin_matrix(const dsa_header_t *hdr, uint32_t version,
                            uint32_t frame, uint32_t next_frame, uint32_t interp,
                            uint32_t j, const int32_t *invbind_j, int32_t *out)
{
    int32_t v_pos[3], q_orient[4], v_scale[3];
    nsmw_interp_one_joint(hdr, version, frame, next_frame, interp, j,
                          v_pos, q_orient, v_scale);

    int32_t manim[12];
    cpu_joint_to_m4x3(v_pos, q_orient, v_scale, manim);

    m4x3_mul(manim, invbind_j, out);
}

// Computes the skinning matrix (M_anim * invbind) for one joint, blending two
// animations by 'blend' (0.0 - 1.0 in f32) before applying the inverse-bind.
ITCM_CODE ARM_CODE static inline
void nsmw_joint_skin_matrix_blend(
        const dsa_header_t *hdr1, uint32_t ver1, uint32_t frame1,
        uint32_t next1, uint32_t interp1,
        const dsa_header_t *hdr2, uint32_t ver2, uint32_t frame2,
        uint32_t next2, uint32_t interp2,
        uint32_t blend, uint32_t j, const int32_t *invbind_j, int32_t *out)
{
    int32_t vp1[3], q1[4], s1[3];
    nsmw_interp_one_joint(hdr1, ver1, frame1, next1, interp1, j, vp1, q1, s1);

    int32_t vp2[3], q2[4], s2[3];
    nsmw_interp_one_joint(hdr2, ver2, frame2, next2, interp2, j, vp2, q2, s2);

    int32_t v_pos[3], q_orient[4], v_scale[3];
    dsa_interpolate_frames_v2(vp1, q1, s1, vp2, q2, s2,
                              blend, v_pos, q_orient, v_scale);

    int32_t manim[12];
    cpu_joint_to_m4x3(v_pos, q_orient, v_scale, manim);

    m4x3_mul(manim, invbind_j, out);
}

// Folds one joint's skinning matrix (single weight) or two joints' skinning
// matrices (blended) into the final node matrix, then stores it into the
// hardware matrix stack at 'base_matrix + node_index'. 'curr_stack_level' is the
// stack slot holding the current (view * model) matrix.
ITCM_CODE ARM_CODE static inline
void nsmw_store_node(const NEA_SkinNode *n, const int32_t *j0, const int32_t *j1,
                     uint32_t curr_stack_level, uint32_t slot)
{
    int32_t node_mtx[12];

    if (n->num_weights == 1)
    {
        for (int k = 0; k < 12; k++)
            node_mtx[k] = j0[k];
    }
    else
    {
        for (int k = 0; k < 12; k++)
            node_mtx[k] = mulf32(j0[k], n->weight0) + mulf32(j1[k], n->weight1);
    }

    MATRIX_RESTORE = curr_stack_level;
    for (int k = 0; k < 12; k++)
        MATRIX_MULT4x3 = node_mtx[k];
    MATRIX_STORE = slot;
}

// Public functions
// ================

uint32_t NSMW_GetNumNodes(const void *nsmw_file)
{
    const uint32_t *p = nsmw_file;
    return p[2]; // magic, version, num_nodes
}

ARM_CODE
int ITCM_FUNC(NSMW_PrepareNodes)(const NEA_NodeSkinData *skin, const void *dsa_file,
                      uint32_t frame_interp)
{
    const dsa_header_t *hdr = dsa_file;

    uint32_t version = hdr->version;
    if (version != DSA_VERSION_1 && version != DSA_VERSION_2)
        return NSMW_INVALID_VERSION;

    uint32_t num_frames = hdr->num_frames;

    uint32_t frame = frame_interp >> 12;
    uint32_t interp = frame_interp & 0xFFF;

    if (frame >= num_frames)
        return NSMW_INVALID_FRAME;

    uint32_t next_frame = frame + 1;
    if (next_frame == num_frames)
        next_frame = 0;

    uint32_t num_nodes = skin->num_nodes;
    uint32_t base_matrix = 30 - num_nodes + 1;

    // Wait for matrix push/pop operations to end
    while (GFX_STATUS & BIT(14));

    uint32_t curr_stack_level = (GFX_STATUS >> 8) & 0x1F;
    if (curr_stack_level >= base_matrix)
        return NSMW_MATRIX_STACK_FULL;

    MATRIX_PUSH = 0;

    for (uint32_t i = 0; i < num_nodes; i++)
    {
        const NEA_SkinNode *n = &skin->nodes[i];

        int32_t j0[12];
        nsmw_joint_skin_matrix(hdr, version, frame, next_frame, interp,
                               n->joint0, &skin->invbind[n->joint0 * 12], j0);

        int32_t j1[12];
        if (n->num_weights != 1)
        {
            nsmw_joint_skin_matrix(hdr, version, frame, next_frame, interp,
                                   n->joint1, &skin->invbind[n->joint1 * 12], j1);
        }

        nsmw_store_node(n, j0, j1, curr_stack_level, base_matrix + i);
    }

    return NSMW_SUCCESS;
}

ARM_CODE
int ITCM_FUNC(NSMW_PrepareNodesBlend)(const NEA_NodeSkinData *skin,
                           const void *dsa_file_1, uint32_t frame_interp_1,
                           const void *dsa_file_2, uint32_t frame_interp_2,
                           uint32_t blend)
{
    const dsa_header_t *hdr1 = dsa_file_1;
    const dsa_header_t *hdr2 = dsa_file_2;

    uint32_t ver1 = hdr1->version;
    uint32_t ver2 = hdr2->version;

    if (ver1 != DSA_VERSION_1 && ver1 != DSA_VERSION_2)
        return NSMW_INVALID_VERSION;
    if (ver2 != DSA_VERSION_1 && ver2 != DSA_VERSION_2)
        return NSMW_INVALID_VERSION;

    if (hdr1->num_joints != hdr2->num_joints)
        return NSMW_INCOMPATIBLE_ANIM;

    uint32_t num_frames_1 = hdr1->num_frames;
    uint32_t num_frames_2 = hdr2->num_frames;

    uint32_t frame1 = frame_interp_1 >> 12;
    uint32_t interp1 = frame_interp_1 & 0xFFF;
    if (frame1 >= num_frames_1)
        return NSMW_INVALID_FRAME;

    uint32_t frame2 = frame_interp_2 >> 12;
    uint32_t interp2 = frame_interp_2 & 0xFFF;
    if (frame2 >= num_frames_2)
        return NSMW_INVALID_FRAME;

    if (blend > inttof32(1))
        return NSMW_INVALID_BLENDING;

    uint32_t next1 = frame1 + 1;
    if (next1 == num_frames_1)
        next1 = 0;

    uint32_t next2 = frame2 + 1;
    if (next2 == num_frames_2)
        next2 = 0;

    uint32_t num_nodes = skin->num_nodes;
    uint32_t base_matrix = 30 - num_nodes + 1;

    // Wait for matrix push/pop operations to end
    while (GFX_STATUS & BIT(14));

    uint32_t curr_stack_level = (GFX_STATUS >> 8) & 0x1F;
    if (curr_stack_level >= base_matrix)
        return NSMW_MATRIX_STACK_FULL;

    MATRIX_PUSH = 0;

    for (uint32_t i = 0; i < num_nodes; i++)
    {
        const NEA_SkinNode *n = &skin->nodes[i];

        int32_t j0[12];
        nsmw_joint_skin_matrix_blend(hdr1, ver1, frame1, next1, interp1,
                                     hdr2, ver2, frame2, next2, interp2,
                                     blend, n->joint0,
                                     &skin->invbind[n->joint0 * 12], j0);

        int32_t j1[12];
        if (n->num_weights != 1)
        {
            nsmw_joint_skin_matrix_blend(hdr1, ver1, frame1, next1, interp1,
                                         hdr2, ver2, frame2, next2, interp2,
                                         blend, n->joint1,
                                         &skin->invbind[n->joint1 * 12], j1);
        }

        nsmw_store_node(n, j0, j1, curr_stack_level, base_matrix + i);
    }

    return NSMW_SUCCESS;
}

void NSMW_FinishDraw(void)
{
    MATRIX_POP = 1;
}
