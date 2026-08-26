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
#define NEA_ANIMMAT_MAX_TRACKS    16  ///< Max property tracks per target.
#define NEA_ANIMMAT_MAX_KEYFRAMES 64  ///< Max keyframes per track.
#define NEA_ANIMMAT_MAX_TARGETS   16  ///< Max materials one animation can drive.

/// Binary file magic: "AMKF" in little-endian.
#define NEA_ANIMMAT_MAGIC 0x464B4D41

/// Original single-material file version.
#define NEA_ANIMMAT_VERSION_1 1

/// Multi-target file version: tracks are grouped under named material targets.
#define NEA_ANIMMAT_VERSION_2 2

/// Version written by the current tools.
#define NEA_ANIMMAT_VERSION NEA_ANIMMAT_VERSION_2

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

    /// Texture and palette index, packed as (texture_idx << 8) | palette_idx,
    /// looked up in the tables set by NEA_AnimMatSetTexPalTables(). Step interp.
    ///
    /// This is the light way to do a flipbook. NEA_AMTRACK_MATERIAL_SWAP
    /// exchanges a whole NEA_Material; this exchanges only the image and its
    /// palette, which is what blinking eyes, scrolling water and sprite-sheet
    /// effects actually need.
    NEA_AMTRACK_TEXPAL_SWAP       = 13,
} NEA_AnimMatTrackType;

/// How a track's values are stored.
///
/// Picking the right one matters more than it looks. A keyframed track costs a
/// binary search, a division and an interpolation every frame; the other two
/// modes cost almost nothing. The exporter should pick the cheapest mode that
/// can represent the track, and the editor shows which one it chose.
typedef enum {
    /// Keyframes with search and interpolation. The right mode for sparse
    /// tracks, where a value changes every twenty or thirty frames.
    NEA_AMSTORE_KEYS  = 0,

    /// One value for the whole animation, stored in the track itself. No array,
    /// no search, no arithmetic. Use it for a channel that never changes but
    /// still has to override the base value.
    NEA_AMSTORE_CONST = 1,

    /// One value per frame in a flat array, indexed directly. Two bytes a frame
    /// and a single load at runtime, which beats keyframes for anything dense.
    ///
    /// Values are 16 bit, so the two packed-colour tracks
    /// (NEA_AMTRACK_DIFFUSE_AMBIENT and NEA_AMTRACK_SPECULAR_EMISSION) cannot
    /// use this mode; they need 32 bits. Every other track can.
    ///
    /// The four texture translate/scale tracks are stored as 1.10.5 fixed point
    /// (the same encoding retail DS material animations use) and widened to f32
    /// when read, so their range is +/-1024 with 1/32 precision.
    NEA_AMSTORE_BAKED = 2,
} NEA_AnimMatStorage;

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
    NEA_AnimMatInterp    interp;  ///< Interpolation mode (NEA_AMSTORE_KEYS only).
    NEA_AnimMatStorage   storage; ///< How the values below are stored.
    uint16_t             count;   ///< KEYS: keyframes. BAKED: frames. CONST: 0.

    union {
        const NEA_AnimMatKeyframe *keys;  ///< NEA_AMSTORE_KEYS value array.
        const int16_t             *baked; ///< NEA_AMSTORE_BAKED value array.
        uint32_t                   value; ///< NEA_AMSTORE_CONST value.
    };
} NEA_AnimMatTrack;

/// One material's worth of tracks within an animation.
///
/// Retail DS material animations target a material by name, and so does this:
/// one animation file can drive every material in a model, with the runtime
/// matching each target against a submesh's material name once at bind time.
typedef struct {
    char name[NEA_MATERIAL_NAME_LEN]; ///< Material name this target drives.
    uint16_t num_tracks;              ///< Number of property tracks (1-16).
    NEA_AnimMatTrack tracks[NEA_ANIMMAT_MAX_TRACKS]; ///< Track array.
} NEA_AnimMatTarget;

/// Animation data template (can be shared by multiple instances).
///
/// Loaded from a .neaanimmat file or defined in code.
typedef struct {
    uint16_t num_targets;  ///< Number of material targets (1-16).
    uint16_t num_frames;   ///< Total animation length in frames.

    /// Target array, allocated for exactly num_targets entries. A version 1
    /// file parses into a single unnamed target, so nothing that used the old
    /// single-material API has to change.
    NEA_AnimMatTarget *targets;

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

    /// Texture and palette tables for the TEXPAL_SWAP track.
    NEA_Material **tex_table;
    NEA_Palette **pal_table;
    uint8_t tex_table_size;
    uint8_t pal_table_size;

    /// Resolved output state (computed by NEA_AnimMatEvaluate).
    uint32_t out_poly_format;     ///< Computed GFX_POLY_FORMAT value.
    NEA_Material *out_material;   ///< Computed material to use (or NULL).
    uint32_t out_color;           ///< Computed vertex color.
    uint32_t out_diff_amb;        ///< Computed diffuse/ambient.
    uint32_t out_spec_emi;        ///< Computed specular/emission.
    bool has_poly_format;         ///< True if any poly-format track exists.
    bool has_material_swap;       ///< True if material swap track exists.
    /// One flag per register, not one for all three. A target that animates
    /// only the vertex colour must not also clobber the material's diffuse and
    /// specular, which is what a single flag would do -- and with several
    /// targets in one animation, partial track sets are the normal case.
    bool has_color;               ///< True if a vertex color track exists.
    bool has_diff_amb;            ///< True if a diffuse/ambient track exists.
    bool has_spec_emi;            ///< True if a specular/emission track exists.

    /// Resolved texture matrix output state.
    int32_t out_tex_scroll_x;     ///< Texture scroll X (f32).
    int32_t out_tex_scroll_y;     ///< Texture scroll Y (f32).
    int32_t out_tex_rotate;       ///< Texture rotation angle (0-511).
    int32_t out_tex_scale_x;      ///< Texture scale X (f32).
    int32_t out_tex_scale_y;      ///< Texture scale Y (f32).
    bool has_tex_transform;       ///< True if any texture matrix track exists.
    bool has_texpal_swap;         ///< True if a texture/palette swap track exists.

    /// Resolved texture/palette swap output.
    NEA_Material *out_texture;    ///< Texture material from TEXPAL_SWAP, or NULL.
    NEA_Palette *out_palette;     ///< Palette from TEXPAL_SWAP, or NULL.

    /// Set once a target has left a non-identity texture matrix loaded, so the
    /// next apply knows it has to reset it. Without this the reset has to happen
    /// unconditionally, which costs an identity load per apply per frame even
    /// for the targets that never touch the texture matrix.
    bool tex_matrix_dirty;

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

/// Asynchronously load animated material data from a .neaanimmat file.
///
/// Unlike NEA_AnimMatDataLoadFAT(), this returns right away and the file is
/// read in the background. Because this loader creates the object instead of
/// filling in an existing one, the result is stored in '*out' by
/// NEA_AsyncProcess() once the file has been parsed. See @ref async for
/// details.
///
/// '*out' is only written on success, and the storage it points at must stay
/// valid until the load finishes. If it is about to go away, release the handle
/// with NEA_AsyncRelease() first: unlike the other async loaders, this one
/// cannot detect that on its own.
///
/// @param out Where to store the pointer to the loaded data.
/// @param path Path to the file.
/// @return Async handle to poll the operation, or NULL on error.
NEA_AsyncFile *NEA_AnimMatDataLoadFATAsync(NEA_AnimMatData **out,
                                           const char *path);

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

/// Set the texture and palette tables for NEA_AMTRACK_TEXPAL_SWAP tracks.
///
/// A keyframe's value is split into a texture index and a palette index, each
/// looked up in the matching table. An index of 0xFF in either half means "leave
/// this one alone", so a track can flip the palette while holding the texture.
///
/// The textures are given as materials because that is where a texture lives in
/// NEA; only the image is taken from them, not their colours or properties.
///
/// @param inst Instance.
/// @param textures Array of materials to take textures from (may be NULL).
/// @param num_tex Number of texture entries (max 255).
/// @param palettes Array of palettes (may be NULL).
/// @param num_pal Number of palette entries (max 255).
void NEA_AnimMatSetTexPalTables(NEA_AnimMatInstance *inst,
                                NEA_Material **textures, int num_tex,
                                NEA_Palette **palettes, int num_pal);

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

/// Evaluate one material target of the animation at the current frame.
///
/// NEA_AnimMatEvaluate() is this with a target of 0. An animation loaded from a
/// version 1 file has exactly one target, so the two are the same there.
///
/// @param inst Instance.
/// @param target Target index (0 to num_targets - 1).
void NEA_AnimMatEvaluateTarget(NEA_AnimMatInstance *inst, int target);

/// Returns the index of the target driving the named material.
///
/// Used by NEA_ModelSetAnimMat() to resolve submesh names once at bind time, so
/// that no string comparison happens while drawing.
///
/// @param data Animation data.
/// @param name Material name to look for.
/// @return Target index, or -1 if no target has that name.
int NEA_AnimMatFindTarget(const NEA_AnimMatData *data, const char *name);

/// Apply the computed material animation state to GPU registers.
///
/// Call this before NEA_ModelDraw() for the affected model.
/// Sets GFX_POLY_FORMAT, calls NEA_MaterialUse(), sets color registers,
/// and/or applies texture matrix transforms depending on which tracks
/// are active.
///
/// @param inst Instance.
void NEA_AnimMatApply(NEA_AnimMatInstance *inst);

/// Evaluate one material target and apply it to the GPU registers.
///
/// This is what NEA_ModelDraw() calls for each submesh whose material name
/// matched a target. Evaluating at apply time rather than once per update is
/// what keeps a multi-target instance from needing an output block per target:
/// each target is evaluated only if something is about to draw with it.
///
/// @param inst Instance.
/// @param target Target index (0 to num_targets - 1).
void NEA_AnimMatApplyTarget(NEA_AnimMatInstance *inst, int target);

/// @}

#endif // NEA_ANIMMAT_H__
