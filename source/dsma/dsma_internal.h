// SPDX-License-Identifier: MIT
//
// Copyright (c) 2022 Antonio Niño Díaz <antonio_nd@outlook.com>

// Shared internal helpers and data structures for the DSA animation format.
//
// These are used both by the DSMA library (dsma.c, rigid single-weight
// skinning) and by the NSMW library (nsmw.c, two-weight smooth skinning). The
// DSA animation file format is shared between them: NSMW reuses the exact same
// per-joint animation data and only adds a node table on top of it.

#ifndef DSMA_INTERNAL_H__
#define DSMA_INTERNAL_H__

#include <nds.h>

#ifndef ARM_CODE
# define ARM_CODE __attribute__((target("arm")))
#endif

// Format of a joint in a DSA v1 file.
typedef struct {
    int32_t pos[3];    // Translation (x, y, z)
    int32_t orient[4]; // Orientation (w, x, y, z)
} dsa_joint_v1_t;

// Format of a joint in a DSA v2 file (with per-bone scale).
typedef struct {
    int32_t pos[3];    // Translation (x, y, z)
    int32_t orient[4]; // Orientation (w, x, y, z)
    int32_t scale[3];  // Scale (x, y, z)
} dsa_joint_v2_t;

#define DSA_VERSION_1 1
#define DSA_VERSION_2 2

// Format of a DSA file header (shared by v1 and v2).
typedef struct {
    uint32_t version;      // Version number (1 or 2)
    uint32_t num_frames;   // Frames in the file
    uint32_t num_joints;   // Joints per frame
    // Followed by joint data (v1 or v2 depending on version)
} dsa_header_t;

// Helper that multiplies two fixed point values in 20.12 format and multiplies
// the result again by 2.
ITCM_CODE ARM_CODE static inline
int32_t mulf32_by_2(int32_t a, int32_t b)
{
    return (a * b) >> (12 - 1);
}

// Gets a pointer to the joint data for the specified frame.
// For v1: each joint is 7 int32s (pos[3] + orient[4])
// For v2: each joint is 10 int32s (pos[3] + orient[4] + scale[3])
ITCM_CODE ARM_CODE static inline
const void *dsa_get_frame_ptr(const dsa_header_t *hdr, uint32_t frame,
                              uint32_t joint_stride)
{
    const uint8_t *base = (const uint8_t *)(hdr + 1);
    return base + frame * hdr->num_joints * joint_stride;
}

// Interpolates linearly between 'start' and 'end'. The position is a floating
// point number in 20.12 format, and it should be between 0.0 and 1.0 (the
// function doesn't check bounds).
ITCM_CODE ARM_CODE static inline
int32_t lerp(int32_t start, int32_t end, int32_t pos)
{
    int32_t diff = end - start;
    return start + ((diff * pos) >> 12);
}

// Interpolates between quaternions 'q1' and 'q2. The position is a floating
// point number in 20.12 format, and it should be between 0.0 and 1.0 (the
// function doesn't check bounds). It stores the result in 'qdest'.
ITCM_CODE ARM_CODE static inline
void q_nlerp(const int32_t *q1, const int32_t *q2, int32_t pos, int32_t *qdest)
{
    qdest[0] = lerp(q1[0], q2[0], pos);
    qdest[1] = lerp(q1[1], q2[1], pos);
    qdest[2] = lerp(q1[2], q2[2], pos);
    qdest[3] = lerp(q1[3], q2[3], pos);

    // TODO: Normalize? It needs way too much CPU time (at least we need one
    // square root and one division), but it may be needed in the future if the
    // animations look bad. Maybe it can be optional.
}

// Interpolate between two positions and two orientations.
ITCM_CODE ARM_CODE static inline
void dsa_interpolate_frames(const int32_t *v_pos_1, const int32_t *q_orient_1,
                            const int32_t *v_pos_2, const int32_t *q_orient_2,
                            uint32_t interp, int32_t *v_pos, int32_t *q_orient)
{
    v_pos[0] = lerp(v_pos_1[0], v_pos_2[0], interp);
    v_pos[1] = lerp(v_pos_1[1], v_pos_2[1], interp);
    v_pos[2] = lerp(v_pos_1[2], v_pos_2[2], interp);

    q_nlerp(q_orient_1, q_orient_2, interp, q_orient);
}

// Interpolate between two positions, two orientations, and two scales.
ITCM_CODE ARM_CODE static inline
void dsa_interpolate_frames_v2(const int32_t *v_pos_1, const int32_t *q_orient_1,
                               const int32_t *v_scale_1,
                               const int32_t *v_pos_2, const int32_t *q_orient_2,
                               const int32_t *v_scale_2,
                               uint32_t interp,
                               int32_t *v_pos, int32_t *q_orient, int32_t *v_scale)
{
    v_pos[0] = lerp(v_pos_1[0], v_pos_2[0], interp);
    v_pos[1] = lerp(v_pos_1[1], v_pos_2[1], interp);
    v_pos[2] = lerp(v_pos_1[2], v_pos_2[2], interp);

    q_nlerp(q_orient_1, q_orient_2, interp, q_orient);

    v_scale[0] = lerp(v_scale_1[0], v_scale_2[0], interp);
    v_scale[1] = lerp(v_scale_1[1], v_scale_2[1], interp);
    v_scale[2] = lerp(v_scale_1[2], v_scale_2[2], interp);
}

#endif // DSMA_INTERNAL_H__
