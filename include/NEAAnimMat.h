// SPDX-License-Identifier: MIT
//
// Copyright (c) 2024-2026 Warioware64
//
// This file is part of Nitro Engine Advanced

#ifndef NEA_ANIMMAT_H__
#define NEA_ANIMMAT_H__

/// @file   NEAAnimMat.h
/// @brief  Animated material system for keyframe-driven GPU state changes.
///
/// Provides keyframe animation for material properties (alpha, lights,
/// culling, colors) and material swapping. Designed for NDS resource
/// constraints with ROM-friendly binary format and hardware-accelerated
/// interpolation.

#include <nds.h>

#include "NEATexture.h"
#include "NEAPolygon.h"
#include "NEAModel.h"

/// @defgroup animmat_system Animated material system
///
/// Keyframe animation for material properties and material swapping.
/// Animations are loaded from compact binary .neaanimmat files. Multiple
/// playback instances can share the same animation data.
///
/// @{

// =========================================================================
// Constants
// =========================================================================

#define NEA_DEFAULT_ANIMMAT       32  ///< Default max animated material instances.
#define NEA_ANIMMAT_MAX_TRACKS    16  ///< Max property tracks per animation.
#define NEA_ANIMMAT_MAX_KEYFRAMES 64  ///< Max keyframes per track.

/// Binary file magic: "AMKF" in little-endian.
#define NEA_ANIMMAT_MAGIC 0x464B4D41

/// Binary file version.
#define NEA_ANIMMAT_VERSION 1

// =========================================================================
// Enums
// =========================================================================

/// Animatable property types for material animation tracks.
typedef enum {
    NEA_AMTRACK_ALPHA             = 0, ///< Alpha (0-31), linear interpolation.
    NEA_AMTRACK_LIGHTS            = 1, ///< NEA_LightEnum mask, step interp.
    NEA_AMTRACK_CULLING           = 2, ///< NEA_CullingEnum, step interp.
    NEA_AMTRACK_COLOR             = 3, ///< Vertex color (RGB15), per-channel lerp.
    NEA_AMTRACK_DIFFUSE_AMBIENT   = 4, ///< Packed diffuse/ambient, per-channel lerp.
    NEA_AMTRACK_SPECULAR_EMISSION = 5, ///< Packed spec/emission, per-channel lerp.
    NEA_AMTRACK_MATERIAL_SWAP     = 6, ///< Material table index, step interp.
    NEA_AMTRACK_POLYID            = 7, ///< Polygon ID (0-63), step interp.
    NEA_AMTRACK_TEX_SCROLL_X      = 8, ///< Texture scroll X (f32). Linear.
    NEA_AMTRACK_TEX_SCROLL_Y      = 9, ///< Texture scroll Y (f32). Linear.
    NEA_AMTRACK_TEX_ROTATE        = 10, ///< Texture rotation angle (0-511). Linear.
    NEA_AMTRACK_TEX_SCALE_X       = 11, ///< Texture scale X (f32). Linear.
    NEA_AMTRACK_TEX_SCALE_Y       = 12, ///< Texture scale Y (f32). Linear.
} NEA_AnimMatTrackType;

/// Interpolation mode for a track.
typedef enum {
    NEA_AMINTERP_STEP   = 0, ///< Hold value until next keyframe.
    NEA_AMINTERP_LINEAR = 1, ///< Linear interpolation between keyframes.
} NEA_AnimMatInterp;

// =========================================================================
// Keyframe and track structures
// =========================================================================

/// A single keyframe in an animation track (8 bytes, ROM-friendly).
typedef struct {
    uint16_t frame;  ///< Frame number (integer, 0-based).
    uint16_t _pad;   ///< Padding for alignment.
    uint32_t value;  ///< Encoded property value (interpretation depends on track).
} NEA_AnimMatKeyframe;

/// A single property track within an animation.
typedef struct {
    NEA_AnimMatTrackType type;    ///< Which property this track animates.
    NEA_AnimMatInterp    interp;  ///< Interpolation mode.
    uint16_t             num_keys; ///< Number of keyframes.
    const NEA_AnimMatKeyframe *keys; ///< Pointer to keyframe array.
} NEA_AnimMatTrack;

/// Animation data template (can be shared by multiple instances).
///
/// Loaded from a .neaanimmat file or defined in code.
typedef struct {
    uint16_t num_tracks;   ///< Number of property tracks (1-16).
    uint16_t num_frames;   ///< Total animation length in frames.
    NEA_AnimMatTrack tracks[NEA_ANIMMAT_MAX_TRACKS]; ///< Track array.
    void *_base_data;      ///< Base pointer for file data (for free).
    bool _loaded_from_fat; ///< Whether _base_data needs free().
} NEA_AnimMatData;

// =========================================================================
// Playback instance
// =========================================================================

/// Runtime playback state for one animated material.
///
/// Each instance references shared animation data and maintains its own
/// playback position, speed, and output state.
typedef struct NEA_AnimMatInstance_ {
    const NEA_AnimMatData *data; ///< Shared animation data.
    NEA_AnimationType type;      ///< Loop or oneshot.
    int32_t speed;               ///< Playback speed (f32). 1<<12 = 1 frame/VBL.
    int32_t currframe;           ///< Current frame (f32 fixed-point).
    bool paused;                 ///< If true, currframe doesn't advance.
    bool active;                 ///< If false, apply does nothing.

    /// Material table for MATERIAL_SWAP track.
    NEA_Material **mat_table;
    uint8_t mat_table_size;

    /// Resolved output state (computed by NEA_AnimMatEvaluate).
    uint32_t out_poly_format;     ///< Computed GFX_POLY_FORMAT value.
    NEA_Material *out_material;   ///< Computed material to use (or NULL).
    uint32_t out_color;           ///< Computed vertex color.
    uint32_t out_diff_amb;        ///< Computed diffuse/ambient.
    uint32_t out_spec_emi;        ///< Computed specular/emission.
    bool has_poly_format;         ///< True if any poly-format track exists.
    bool has_material_swap;       ///< True if material swap track exists.
    bool has_color_props;         ///< True if color property tracks exist.

    /// Resolved texture matrix output state.
    int32_t out_tex_scroll_x;     ///< Texture scroll X (f32).
    int32_t out_tex_scroll_y;     ///< Texture scroll Y (f32).
    int32_t out_tex_rotate;       ///< Texture rotation angle (0-511).
    int32_t out_tex_scale_x;      ///< Texture scale X (f32).
    int32_t out_tex_scale_y;      ///< Texture scale Y (f32).
    bool has_tex_transform;       ///< True if any texture matrix track exists.

    /// Base polygon format values (used when tracks don't override).
    uint32_t base_alpha;
    uint32_t base_polyid;
    NEA_LightEnum base_lights;
    NEA_CullingEnum base_culling;
    NEA_OtherFormatEnum base_other;
} NEA_AnimMatInstance;

// =========================================================================
// System lifecycle
// =========================================================================

/// Reset the animated material system.
///
/// @param max_instances Max simultaneous playback instances. If < 1,
///                      uses NEA_DEFAULT_ANIMMAT.
/// @return 0 on success, -1 on failure.
int NEA_AnimMatSystemReset(int max_instances);

/// End the animated material system and free all memory.
void NEA_AnimMatSystemEnd(void);

// =========================================================================
// Data loading
// =========================================================================

/// Load animated material data from a .neaanimmat file on filesystem.
///
/// @param path Path to the file.
/// @return Pointer to loaded data, or NULL on error.
NEA_AnimMatData *NEA_AnimMatDataLoadFAT(const char *path);

/// Load animated material data from RAM.
///
/// The data pointer must remain valid for the lifetime of the returned
/// NEA_AnimMatData (keyframe pointers reference into it directly).
///
/// @param pointer Pointer to file data in RAM.
/// @return Pointer to parsed data, or NULL on error.
NEA_AnimMatData *NEA_AnimMatDataLoad(const void *pointer);

/// Free animated material data.
///
/// @param data Pointer to the data.
void NEA_AnimMatDataFree(NEA_AnimMatData *data);

// =========================================================================
// Instance management
// =========================================================================

/// Create a new animated material playback instance.
///
/// @return Pointer to instance, or NULL if pool is full.
NEA_AnimMatInstance *NEA_AnimMatCreate(void);

/// Delete an animated material instance.
///
/// @param inst Pointer to the instance.
void NEA_AnimMatDelete(NEA_AnimMatInstance *inst);

/// Assign animation data to an instance.
///
/// @param inst Instance.
/// @param data Animation data.
void NEA_AnimMatSetData(NEA_AnimMatInstance *inst,
                        const NEA_AnimMatData *data);

/// Set the material table for MATERIAL_SWAP tracks.
///
/// When a MATERIAL_SWAP keyframe is evaluated, its value is used as an
/// index into this table to determine which material to use.
///
/// @param inst Instance.
/// @param table Array of NEA_Material pointers.
/// @param count Number of entries in the table (max 255).
void NEA_AnimMatSetMaterialTable(NEA_AnimMatInstance *inst,
                                 NEA_Material **table, int count);

/// Set base polygon format values (used when no track overrides them).
///
/// @param inst Instance.
/// @param alpha Base alpha (0-31).
/// @param id Base polygon ID (0-63).
/// @param lights Base light mask.
/// @param culling Base culling mode.
/// @param other Base other format flags.
void NEA_AnimMatSetBasePolyFormat(NEA_AnimMatInstance *inst,
                                  u32 alpha, u32 id,
                                  NEA_LightEnum lights,
                                  NEA_CullingEnum culling,
                                  NEA_OtherFormatEnum other);

// =========================================================================
// Playback control
// =========================================================================

/// Start playback.
///
/// @param inst Instance.
/// @param type Loop or oneshot.
/// @param speed Playback speed (f32). 1<<12 = 1 frame per VBL.
void NEA_AnimMatStart(NEA_AnimMatInstance *inst,
                      NEA_AnimationType type, int32_t speed);

/// Stop playback and reset to frame 0.
///
/// @param inst Instance.
void NEA_AnimMatStop(NEA_AnimMatInstance *inst);

/// Pause or resume playback.
///
/// @param inst Instance.
/// @param paused True to pause, false to resume.
void NEA_AnimMatPause(NEA_AnimMatInstance *inst, bool paused);

/// Set playback speed.
///
/// @param inst Instance.
/// @param speed New speed (f32).
void NEA_AnimMatSetSpeed(NEA_AnimMatInstance *inst, int32_t speed);

/// Set current frame.
///
/// @param inst Instance.
/// @param frame Frame to set (f32).
void NEA_AnimMatSetFrame(NEA_AnimMatInstance *inst, int32_t frame);

/// Get current frame.
///
/// @param inst Instance.
/// @return Current frame (f32).
int32_t NEA_AnimMatGetFrame(const NEA_AnimMatInstance *inst);

// =========================================================================
// Core update and apply
// =========================================================================

/// Advance all active animated material instances by one tick.
///
/// Called from NEA_WaitForVBL() when NEA_UPDATE_ANIM_MAT is set.
void NEA_AnimMatUpdateAll(void);

/// Evaluate the current frame and compute output values.
///
/// Called internally by NEA_AnimMatUpdateAll(), or can be called manually
/// after setting the frame with NEA_AnimMatSetFrame().
///
/// @param inst Instance.
void NEA_AnimMatEvaluate(NEA_AnimMatInstance *inst);

/// Apply the computed material animation state to GPU registers.
///
/// Call this before NEA_ModelDraw() for the affected model.
/// Sets GFX_POLY_FORMAT, calls NEA_MaterialUse(), sets color registers,
/// and/or applies texture matrix transforms depending on which tracks
/// are active.
///
/// @param inst Instance.
void NEA_AnimMatApply(const NEA_AnimMatInstance *inst);

/// @}

#endif // NEA_ANIMMAT_H__
