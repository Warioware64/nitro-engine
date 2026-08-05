// SPDX-License-Identifier: MIT
//
// Copyright (c) 2022 Antonio Niño Díaz <antonio_nd@outlook.com>

// DS Model Animation Library v0.3.0

#include <nds.h>

#include "dsma.h"

// Because of Nitro Engine Advanced's safe dual 3D mode, it is required to use Nitro
// Engine's functions to draw display lists instead of relying on libnds.
#include "NEAMain.h"

// Shared DSA structures and interpolation helpers (also used by nsmw.c).
#include "dsma_internal.h"

// Private functions
// =================

// Generates a 4x3 matrix from the orientation in the provided quaternion and
// the translation in the provided vector. Then, it multiplies the matrix that
// is currently active in the geometry engine by the generated matrix.
ITCM_CODE ARM_CODE static inline
void matrix_mult_by_joint(const int32_t *v, const int32_t *q)
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

    MATRIX_MULT4x3 = inttof32(1) - y2 - z2;
    MATRIX_MULT4x3 = xy + wz;
    MATRIX_MULT4x3 = xz - wy;

    MATRIX_MULT4x3 = xy - wz;
    MATRIX_MULT4x3 = inttof32(1) - x2 - z2;
    MATRIX_MULT4x3 = yz + wx;

    MATRIX_MULT4x3 = xz + wy;
    MATRIX_MULT4x3 = yz - wx;
    MATRIX_MULT4x3 = inttof32(1) - x2 - y2;

    MATRIX_MULT4x3 = v[0];
    MATRIX_MULT4x3 = v[1];
    MATRIX_MULT4x3 = v[2];
}

// Generates a 4x3 matrix from quaternion, translation, and scale, then
// multiplies the current geometry engine matrix by it.
ITCM_CODE ARM_CODE static inline
void matrix_mult_by_joint_scaled(const int32_t *v, const int32_t *q,
                                 const int32_t *s)
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

    // Rotation columns scaled by per-axis scale
    MATRIX_MULT4x3 = mulf32(inttof32(1) - y2 - z2, s[0]);
    MATRIX_MULT4x3 = mulf32(xy + wz, s[0]);
    MATRIX_MULT4x3 = mulf32(xz - wy, s[0]);

    MATRIX_MULT4x3 = mulf32(xy - wz, s[1]);
    MATRIX_MULT4x3 = mulf32(inttof32(1) - x2 - z2, s[1]);
    MATRIX_MULT4x3 = mulf32(yz + wx, s[1]);

    MATRIX_MULT4x3 = mulf32(xz + wy, s[2]);
    MATRIX_MULT4x3 = mulf32(yz - wx, s[2]);
    MATRIX_MULT4x3 = mulf32(inttof32(1) - x2 - y2, s[2]);

    MATRIX_MULT4x3 = v[0];
    MATRIX_MULT4x3 = v[1];
    MATRIX_MULT4x3 = v[2];
}

// Public functions
// ================

uint32_t DSMA_GetNumFrames(const void *dsa_file)
{
    const dsa_header_t *hdr = dsa_file;
    return hdr->num_frames;
}

ARM_CODE
int ITCM_FUNC(DSMA_PrepareBones)(const void *dsa_file, uint32_t frame_interp)
{
    const dsa_header_t *hdr = dsa_file;

    uint32_t version = hdr->version;
    if (version != DSA_VERSION_1 && version != DSA_VERSION_2)
        return DSMA_INVALID_VERSION;

    uint32_t num_joints = hdr->num_joints;
    uint32_t num_frames = hdr->num_frames;

    uint32_t frame = frame_interp >> 12;
    uint32_t interp = frame_interp & 0xFFF;

    if (frame >= num_frames)
        return DSMA_INVALID_FRAME;

    // Make sure that there is enough space in the matrix stack
    // --------------------------------------------------------

    uint32_t base_matrix = 30 - num_joints + 1;

    // Wait for matrix push/pop operations to end
    while (GFX_STATUS & BIT(14));

    uint32_t curr_stack_level = (GFX_STATUS >> 8) & 0x1F;
    if (curr_stack_level >= base_matrix)
        return DSMA_MATRIX_STACK_FULL;

    MATRIX_PUSH = 0;

    // Generate matrices with bone transformations
    // -------------------------------------------

    if (version == DSA_VERSION_2)
    {
        uint32_t stride = sizeof(dsa_joint_v2_t);
        const dsa_joint_v2_t *frame_ptr_1 =
            dsa_get_frame_ptr(hdr, frame, stride);

        if (interp != 0)
        {
            uint32_t next_frame = frame + 1;
            if (next_frame == num_frames)
                next_frame = 0;

            const dsa_joint_v2_t *frame_ptr_2 =
                dsa_get_frame_ptr(hdr, next_frame, stride);

            for (uint32_t i = 0; i < num_joints; i++)
            {
                int32_t v_pos[3];
                int32_t q_orient[4];
                int32_t v_scale[3];

                dsa_interpolate_frames_v2(
                    frame_ptr_1->pos, frame_ptr_1->orient, frame_ptr_1->scale,
                    frame_ptr_2->pos, frame_ptr_2->orient, frame_ptr_2->scale,
                    interp, v_pos, q_orient, v_scale);
                frame_ptr_1++;
                frame_ptr_2++;

                MATRIX_RESTORE = curr_stack_level;
                matrix_mult_by_joint_scaled(v_pos, q_orient, v_scale);
                MATRIX_STORE = base_matrix + i;
            }
        }
        else
        {
            for (uint32_t i = 0; i < num_joints; i++)
            {
                MATRIX_RESTORE = curr_stack_level;
                matrix_mult_by_joint_scaled(frame_ptr_1->pos,
                                            frame_ptr_1->orient,
                                            frame_ptr_1->scale);
                frame_ptr_1++;
                MATRIX_STORE = base_matrix + i;
            }
        }
    }
    else // DSA_VERSION_1
    {
        uint32_t stride = sizeof(dsa_joint_v1_t);
        const dsa_joint_v1_t *frame_ptr_1 =
            dsa_get_frame_ptr(hdr, frame, stride);

        if (interp != 0)
        {
            uint32_t next_frame = frame + 1;
            if (next_frame == num_frames)
                next_frame = 0;

            const dsa_joint_v1_t *frame_ptr_2 =
                dsa_get_frame_ptr(hdr, next_frame, stride);

            for (uint32_t i = 0; i < num_joints; i++)
            {
                int32_t v_pos[3];
                int32_t q_orient[4];

                dsa_interpolate_frames(frame_ptr_1->pos, frame_ptr_1->orient,
                                       frame_ptr_2->pos, frame_ptr_2->orient,
                                       interp, v_pos, q_orient);
                frame_ptr_1++;
                frame_ptr_2++;

                MATRIX_RESTORE = curr_stack_level;
                matrix_mult_by_joint(v_pos, q_orient);
                MATRIX_STORE = base_matrix + i;
            }
        }
        else
        {
            for (uint32_t i = 0; i < num_joints; i++)
            {
                MATRIX_RESTORE = curr_stack_level;
                matrix_mult_by_joint(frame_ptr_1->pos, frame_ptr_1->orient);
                frame_ptr_1++;
                MATRIX_STORE = base_matrix + i;
            }
        }
    }

    return DSMA_SUCCESS;
}

void DSMA_FinishDraw(void)
{
    MATRIX_POP = 1;
}

ARM_CODE
int ITCM_FUNC(DSMA_DrawModel)(const void *dsm_file, const void *dsa_file, uint32_t frame_interp)
{
    int ret = DSMA_PrepareBones(dsa_file, frame_interp);
    if (ret != DSMA_SUCCESS)
        return ret;

    NEA_DisplayListDrawDefault(dsm_file);
    DSMA_FinishDraw();

    return DSMA_SUCCESS;
}

ARM_CODE
int ITCM_FUNC(DSMA_PrepareBonesBlend)(const void *dsa_file_1, uint32_t frame_interp_1,
        const void *dsa_file_2, uint32_t frame_interp_2,
        uint32_t blend)
{
    const dsa_header_t *hdr_1 = dsa_file_1;
    const dsa_header_t *hdr_2 = dsa_file_2;

    uint32_t ver_1 = hdr_1->version;
    uint32_t ver_2 = hdr_2->version;

    if (ver_1 != DSA_VERSION_1 && ver_1 != DSA_VERSION_2)
        return DSMA_INVALID_VERSION;
    if (ver_2 != DSA_VERSION_1 && ver_2 != DSA_VERSION_2)
        return DSMA_INVALID_VERSION;

    uint32_t num_joints = hdr_1->num_joints;

    if (num_joints != hdr_2->num_joints)
        return DSMA_INCOMPATIBLE_ANIMATIONS;

    uint32_t num_frames_1 = hdr_1->num_frames;
    uint32_t num_frames_2 = hdr_2->num_frames;

    uint32_t frame_1 = frame_interp_1 >> 12;
    uint32_t interp_1 = frame_interp_1 & 0xFFF;

    if (frame_1 >= num_frames_1)
        return DSMA_INVALID_FRAME;

    uint32_t frame_2 = frame_interp_2 >> 12;
    uint32_t interp_2 = frame_interp_2 & 0xFFF;

    if (frame_2 >= num_frames_2)
        return DSMA_INVALID_FRAME;

    if (blend > inttof32(1))
        return DSMA_INVALID_BLENDING;

    // Make sure that there is enough space in the matrix stack
    // --------------------------------------------------------

    uint32_t base_matrix = 30 - num_joints + 1;

    // Wait for matrix push/pop operations to end
    while (GFX_STATUS & BIT(14));

    uint32_t curr_stack_level = (GFX_STATUS >> 8) & 0x1F;
    if (curr_stack_level >= base_matrix)
        return DSMA_MATRIX_STACK_FULL;

    MATRIX_PUSH = 0;

    // Generate matrices with bone transformations
    // -------------------------------------------

    uint32_t next_frame_1 = frame_1 + 1;
    if (next_frame_1 == num_frames_1)
        next_frame_1 = 0;

    uint32_t next_frame_2 = frame_2 + 1;
    if (next_frame_2 == num_frames_2)
        next_frame_2 = 0;

    // Use v2 path if either animation has scale data
    bool use_scale = (ver_1 == DSA_VERSION_2) || (ver_2 == DSA_VERSION_2);

    if (use_scale)
    {
        // Default scale for v1 joints: 1.0 in f32
        static const int32_t unit_scale[3] = {
            1 << 12, 1 << 12, 1 << 12
        };

        uint32_t stride_1 = (ver_1 == DSA_VERSION_2)
            ? sizeof(dsa_joint_v2_t) : sizeof(dsa_joint_v1_t);
        uint32_t stride_2 = (ver_2 == DSA_VERSION_2)
            ? sizeof(dsa_joint_v2_t) : sizeof(dsa_joint_v1_t);

        const uint8_t *f1_p1 = dsa_get_frame_ptr(hdr_1, frame_1, stride_1);
        const uint8_t *f1_p2 = dsa_get_frame_ptr(hdr_1, next_frame_1, stride_1);
        const uint8_t *f2_p1 = dsa_get_frame_ptr(hdr_2, frame_2, stride_2);
        const uint8_t *f2_p2 = dsa_get_frame_ptr(hdr_2, next_frame_2, stride_2);

        for (uint32_t i = 0; i < num_joints; i++)
        {
            const dsa_joint_v1_t *j1_1 = (const dsa_joint_v1_t *)f1_p1;
            const dsa_joint_v1_t *j1_2 = (const dsa_joint_v1_t *)f1_p2;
            const int32_t *s1_1 = (ver_1 == DSA_VERSION_2)
                ? ((const dsa_joint_v2_t *)f1_p1)->scale : unit_scale;
            const int32_t *s1_2 = (ver_1 == DSA_VERSION_2)
                ? ((const dsa_joint_v2_t *)f1_p2)->scale : unit_scale;

            int32_t v_pos_1[3], q_orient_1[4], v_scale_1[3];
            dsa_interpolate_frames_v2(j1_1->pos, j1_1->orient, s1_1,
                                      j1_2->pos, j1_2->orient, s1_2,
                                      interp_1, v_pos_1, q_orient_1, v_scale_1);
            f1_p1 += stride_1;
            f1_p2 += stride_1;

            const dsa_joint_v1_t *j2_1 = (const dsa_joint_v1_t *)f2_p1;
            const dsa_joint_v1_t *j2_2 = (const dsa_joint_v1_t *)f2_p2;
            const int32_t *s2_1 = (ver_2 == DSA_VERSION_2)
                ? ((const dsa_joint_v2_t *)f2_p1)->scale : unit_scale;
            const int32_t *s2_2 = (ver_2 == DSA_VERSION_2)
                ? ((const dsa_joint_v2_t *)f2_p2)->scale : unit_scale;

            int32_t v_pos_2[3], q_orient_2[4], v_scale_2[3];
            dsa_interpolate_frames_v2(j2_1->pos, j2_1->orient, s2_1,
                                      j2_2->pos, j2_2->orient, s2_2,
                                      interp_2, v_pos_2, q_orient_2, v_scale_2);
            f2_p1 += stride_2;
            f2_p2 += stride_2;

            int32_t v_pos[3], q_orient[4], v_scale[3];
            dsa_interpolate_frames_v2(v_pos_1, q_orient_1, v_scale_1,
                                      v_pos_2, q_orient_2, v_scale_2,
                                      blend, v_pos, q_orient, v_scale);

            MATRIX_RESTORE = curr_stack_level;
            matrix_mult_by_joint_scaled(v_pos, q_orient, v_scale);
            MATRIX_STORE = base_matrix + i;
        }
    }
    else
    {
        // Both v1 — original fast path, no scale
        uint32_t stride = sizeof(dsa_joint_v1_t);

        const dsa_joint_v1_t *f1_p1 = dsa_get_frame_ptr(hdr_1, frame_1, stride);
        const dsa_joint_v1_t *f1_p2 = dsa_get_frame_ptr(hdr_1, next_frame_1, stride);
        const dsa_joint_v1_t *f2_p1 = dsa_get_frame_ptr(hdr_2, frame_2, stride);
        const dsa_joint_v1_t *f2_p2 = dsa_get_frame_ptr(hdr_2, next_frame_2, stride);

        for (uint32_t i = 0; i < num_joints; i++)
        {
            int32_t v_pos_1[3], q_orient_1[4];
            dsa_interpolate_frames(f1_p1->pos, f1_p1->orient,
                                   f1_p2->pos, f1_p2->orient,
                                   interp_1, v_pos_1, q_orient_1);
            f1_p1++;
            f1_p2++;

            int32_t v_pos_2[3], q_orient_2[4];
            dsa_interpolate_frames(f2_p1->pos, f2_p1->orient,
                                   f2_p2->pos, f2_p2->orient,
                                   interp_2, v_pos_2, q_orient_2);
            f2_p1++;
            f2_p2++;

            int32_t v_pos[3], q_orient[4];
            dsa_interpolate_frames(v_pos_1, q_orient_1,
                                   v_pos_2, q_orient_2,
                                   blend, v_pos, q_orient);

            MATRIX_RESTORE = curr_stack_level;
            matrix_mult_by_joint(v_pos, q_orient);
            MATRIX_STORE = base_matrix + i;
        }
    }

    return DSMA_SUCCESS;
}

ARM_CODE
int ITCM_FUNC(DSMA_DrawModelBlendAnimation)(const void *dsm_file,
        const void *dsa_file_1, uint32_t frame_interp_1,
        const void *dsa_file_2, uint32_t frame_interp_2,
        uint32_t blend)
{
    int ret = DSMA_PrepareBonesBlend(dsa_file_1, frame_interp_1,
                                     dsa_file_2, frame_interp_2, blend);
    if (ret != DSMA_SUCCESS)
        return ret;

    NEA_DisplayListDrawDefault(dsm_file);
    DSMA_FinishDraw();

    return DSMA_SUCCESS;
}
