// SPDX-License-Identifier: MIT
//
// Copyright (c) 2008-2024 Antonio Niño Díaz
//
// This file is part of Nitro Engine Advanced

#ifndef NEA_NODE_SKIN_H__
#define NEA_NODE_SKIN_H__

/// @file   NEANodeSkin.h
/// @brief  NSMW (NitroSkin MultiWeight) two-weight skinning runtime.

/// @defgroup nsmw_system NSMW two-weight skinning
///
/// NSMW (NitroSkin MultiWeight) is an animated mesh format that supports up to
/// two bone weights per vertex (smooth skinning), unlike DSMA which only does
/// rigid single-weight skinning.
///
/// The Nintendo DS GPU has no per-vertex hardware skinning, so NSMW uses a
/// matrix-palette ("node") scheme, just like the retail NSBMD format. A "node"
/// is one matrix-stack slot whose matrix is a weighted blend of the *skinning
/// matrices* of one or two joints:
///
///     N = w0 * (M_joint0_anim * M_joint0_bind_inv)
///       + w1 * (M_joint1_anim * M_joint1_bind_inv)
///
/// Vertices are stored in model/bind space; drawing a vertex through @c N is
/// exact linear-blend skinning because every vertex that shares a weight combo
/// shares one node matrix. The per-joint inverse-bind matrices are stored in the
/// NSMW file (computed at export time from the rest pose).
///
/// The animation itself is stored in the regular DSA file format, exactly the
/// same as DSMA: NSMW only adds a node-table indirection between joints and
/// matrix-stack slots.
///
/// @{

/// NSMW file magic number ("NSMW" in little-endian).
#define NEA_NSMW_MAGIC 0x574D534E

/// Maximum number of nodes (matrix-palette slots) in an NSMW model.
///
/// One matrix-stack slot is used per node, and the hardware matrix stack has 31
/// levels (0-30), so this is the practical upper bound.
#define NEA_MAX_SKIN_NODES 30

#define NSMW_SUCCESS             0  ///< Operation succeeded.
#define NSMW_INVALID_VERSION    -1  ///< The DSA file has an unsupported version.
#define NSMW_INVALID_FRAME      -2  ///< The requested frame is out of range.
#define NSMW_INVALID_BLENDING   -3  ///< The blend factor is out of range.
#define NSMW_MATRIX_STACK_FULL  -4  ///< Not enough free matrix-stack slots.
#define NSMW_INCOMPATIBLE_ANIM  -5  ///< The two animations are not compatible.

/// A node is one matrix-palette slot: either a single joint matrix
/// (num_weights == 1) or a weighted blend of two joint matrices.
typedef struct {
    uint8_t num_weights; ///< 1 (rigid) or 2 (blended).
    uint8_t joint0;      ///< First joint index.
    uint8_t joint1;      ///< Second joint index (== joint0 if num_weights == 1).
    uint8_t pad;         ///< Padding (must be 0).
    int32_t weight0;     ///< Weight for joint0 (f32). inttof32(1) if rigid.
    int32_t weight1;     ///< Weight for joint1 (f32). 0 if rigid.
} NEA_SkinNode;

/// Parsed NSMW skinning data kept alongside an animated model.
typedef struct {
    uint32_t      num_nodes;   ///< Number of nodes (matrix-palette slots).
    uint32_t      num_joints;  ///< Number of joints (must match the DSA file).
    NEA_SkinNode  nodes[NEA_MAX_SKIN_NODES]; ///< Node table.
    const int32_t *invbind;    ///< Inverse-bind matrices: num_joints * 12 (f32),
                               ///< pointing into the loaded file buffer.
} NEA_NodeSkinData;

/// Returns the number of nodes stored in the specified NSMW file.
///
/// @param nsmw_file Pointer to the NSMW file data.
/// @return Number of nodes.
uint32_t NSMW_GetNumNodes(const void *nsmw_file);

/// Sets up node matrices in the hardware matrix stack for the given animation
/// frame, but does NOT draw any display list. Call NSMW_FinishDraw() after
/// drawing your display list(s) to clean up the matrix stack.
///
/// The frame is a fixed point value in 20.12 format (e.g. 3 << 12 is frame 3).
/// It wraps around: when going past the last frame it interpolates with frame 0.
///
/// @param skin Parsed NSMW skinning data.
/// @param dsa_file Pointer to the DSA animation file.
/// @param frame_interp Frame (20.12 fixed point).
/// @return An NSMW_* code (0 for success).
ITCM_CODE ARM_CODE
int NSMW_PrepareNodes(const NEA_NodeSkinData *skin, const void *dsa_file,
                      uint32_t frame_interp);

/// Same as NSMW_PrepareNodes() but blends between two animations.
///
/// The frames are in 20.12 fixed point and wrap around. The blend factor is in
/// 20.12 fixed point and goes from 0.0 (DSA file 1) to 1.0 (DSA file 2).
///
/// @param skin Parsed NSMW skinning data.
/// @param dsa_file_1 First DSA animation file.
/// @param frame_interp_1 Frame for the first animation (20.12).
/// @param dsa_file_2 Second DSA animation file.
/// @param frame_interp_2 Frame for the second animation (20.12).
/// @param blend Blend factor (20.12, 0.0 - 1.0).
/// @return An NSMW_* code (0 for success).
ITCM_CODE ARM_CODE
int NSMW_PrepareNodesBlend(const NEA_NodeSkinData *skin,
                           const void *dsa_file_1, uint32_t frame_interp_1,
                           const void *dsa_file_2, uint32_t frame_interp_2,
                           uint32_t blend);

/// Pops the matrix stack after NSMW_PrepareNodes / NSMW_PrepareNodesBlend.
/// Must be called once after you are done drawing all display lists.
void NSMW_FinishDraw(void);

/// @}

#endif // NEA_NODE_SKIN_H__
