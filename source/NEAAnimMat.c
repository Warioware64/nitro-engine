// SPDX-License-Identifier: MIT
//
// Copyright (c) 2024-2026 Warioware64
//
// This file is part of Nitro Engine Advanced

#include "NEAMain.h"

/// @file NEAAnimMat.c

// =========================================================================
// Pool management
// =========================================================================

static NEA_AnimMatInstance **NEA_AnimMatPointers = NULL;
static int NEA_MAX_ANIMMAT = 0;
static bool ne_animmat_system_inited = false;

// =========================================================================
// Binary format header
// =========================================================================

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint16_t num_tracks;
    uint16_t num_frames;
    uint8_t  reserved[4];
} neaanimmat_header_t;

typedef struct {
    uint8_t  track_type;
    uint8_t  interp_mode;
    uint16_t num_keys;
    uint32_t key_offset;
    uint32_t reserved;
} neaanimmat_track_header_t;

// =========================================================================
// System lifecycle
// =========================================================================

int NEA_AnimMatSystemReset(int max_instances)
{
    if (ne_animmat_system_inited)
        NEA_AnimMatSystemEnd();

    if (max_instances < 1)
        NEA_MAX_ANIMMAT = NEA_DEFAULT_ANIMMAT;
    else
        NEA_MAX_ANIMMAT = max_instances;

    NEA_AnimMatPointers = calloc(NEA_MAX_ANIMMAT,
                                 sizeof(NEA_AnimMatInstance *));
    if (NEA_AnimMatPointers == NULL)
    {
        NEA_DebugPrint("Not enough memory for animmat pool");
        return -1;
    }

    ne_animmat_system_inited = true;
    return 0;
}

void NEA_AnimMatSystemEnd(void)
{
    if (!ne_animmat_system_inited)
        return;

    for (int i = 0; i < NEA_MAX_ANIMMAT; i++)
    {
        if (NEA_AnimMatPointers[i] != NULL)
        {
            free(NEA_AnimMatPointers[i]);
            NEA_AnimMatPointers[i] = NULL;
        }
    }

    free(NEA_AnimMatPointers);
    NEA_AnimMatPointers = NULL;
    NEA_MAX_ANIMMAT = 0;
    ne_animmat_system_inited = false;
}

// =========================================================================
// Data loading
// =========================================================================

static NEA_AnimMatData *ne_animmat_parse(const void *data, bool from_fat)
{
    const uint8_t *base = (const uint8_t *)data;
    const neaanimmat_header_t *hdr = (const neaanimmat_header_t *)base;

    if (hdr->magic != NEA_ANIMMAT_MAGIC)
    {
        NEA_DebugPrint("Invalid .neaanimmat magic");
        return NULL;
    }
    if (hdr->version != NEA_ANIMMAT_VERSION)
    {
        NEA_DebugPrint("Unsupported .neaanimmat version");
        return NULL;
    }
    if (hdr->num_tracks < 1 || hdr->num_tracks > NEA_ANIMMAT_MAX_TRACKS)
    {
        NEA_DebugPrint("Invalid track count");
        return NULL;
    }

    NEA_AnimMatData *amd = calloc(1, sizeof(NEA_AnimMatData));
    if (amd == NULL)
    {
        NEA_DebugPrint("Not enough memory for animmat data");
        return NULL;
    }

    amd->num_tracks = hdr->num_tracks;
    amd->num_frames = hdr->num_frames;
    amd->_base_data = (void *)data;
    amd->_loaded_from_fat = from_fat;

    // Parse track headers (immediately after file header)
    const neaanimmat_track_header_t *track_hdrs =
        (const neaanimmat_track_header_t *)(base + sizeof(neaanimmat_header_t));

    for (int i = 0; i < amd->num_tracks; i++)
    {
        NEA_AnimMatTrack *track = &amd->tracks[i];
        track->type = (NEA_AnimMatTrackType)track_hdrs[i].track_type;
        track->interp = (NEA_AnimMatInterp)track_hdrs[i].interp_mode;
        track->num_keys = track_hdrs[i].num_keys;

        if (track->num_keys > NEA_ANIMMAT_MAX_KEYFRAMES)
            track->num_keys = NEA_ANIMMAT_MAX_KEYFRAMES;

        // Keyframe data is at the offset specified in the track header.
        // Pointers reference directly into the loaded data buffer.
        track->keys = (const NEA_AnimMatKeyframe *)(base +
                                                     track_hdrs[i].key_offset);
    }

    return amd;
}

NEA_AnimMatData *NEA_AnimMatDataLoad(const void *pointer)
{
    NEA_AssertPointer(pointer, "NULL data pointer");
    return ne_animmat_parse(pointer, false);
}

NEA_AnimMatData *NEA_AnimMatDataLoadFAT(const char *path)
{
    NEA_AssertPointer(path, "NULL path");

    char *data = NEA_FATLoadData(path);
    if (data == NULL)
    {
        NEA_DebugPrint("Can't load %s", path);
        return NULL;
    }

    NEA_AnimMatData *amd = ne_animmat_parse(data, true);
    if (amd == NULL)
    {
        free(data);
        return NULL;
    }

    return amd;
}

void NEA_AnimMatDataFree(NEA_AnimMatData *data)
{
    if (data == NULL)
        return;

    if (data->_loaded_from_fat && data->_base_data != NULL)
        free(data->_base_data);

    free(data);
}

// =========================================================================
// Instance management
// =========================================================================

NEA_AnimMatInstance *NEA_AnimMatCreate(void)
{
    if (!ne_animmat_system_inited)
    {
        NEA_DebugPrint("System not initialized");
        return NULL;
    }

    NEA_AnimMatInstance *inst = calloc(1, sizeof(NEA_AnimMatInstance));
    if (inst == NULL)
    {
        NEA_DebugPrint("Not enough memory");
        return NULL;
    }

    // Find free pool slot
    for (int i = 0; i < NEA_MAX_ANIMMAT; i++)
    {
        if (NEA_AnimMatPointers[i] == NULL)
        {
            NEA_AnimMatPointers[i] = inst;

            // Sensible defaults
            inst->base_alpha = 31;
            inst->base_polyid = 0;
            inst->base_lights = NEA_LIGHT_0;
            inst->base_culling = NEA_CULL_BACK;
            inst->base_other = 0;

            return inst;
        }
    }

    NEA_DebugPrint("No free animmat slots");
    free(inst);
    return NULL;
}

void NEA_AnimMatDelete(NEA_AnimMatInstance *inst)
{
    if (inst == NULL)
        return;

    if (!ne_animmat_system_inited)
        return;

    for (int i = 0; i < NEA_MAX_ANIMMAT; i++)
    {
        if (NEA_AnimMatPointers[i] == inst)
        {
            NEA_AnimMatPointers[i] = NULL;
            break;
        }
    }

    free(inst);
}

void NEA_AnimMatSetData(NEA_AnimMatInstance *inst,
                        const NEA_AnimMatData *data)
{
    NEA_AssertPointer(inst, "NULL instance");

    inst->data = data;
    inst->currframe = 0;

    // Pre-scan tracks to set capability flags
    inst->has_poly_format = false;
    inst->has_material_swap = false;
    inst->has_color_props = false;
    inst->has_tex_transform = false;

    if (data != NULL)
    {
        for (int i = 0; i < data->num_tracks; i++)
        {
            switch (data->tracks[i].type)
            {
                case NEA_AMTRACK_ALPHA:
                case NEA_AMTRACK_LIGHTS:
                case NEA_AMTRACK_CULLING:
                case NEA_AMTRACK_POLYID:
                    inst->has_poly_format = true;
                    break;
                case NEA_AMTRACK_MATERIAL_SWAP:
                    inst->has_material_swap = true;
                    break;
                case NEA_AMTRACK_COLOR:
                case NEA_AMTRACK_DIFFUSE_AMBIENT:
                case NEA_AMTRACK_SPECULAR_EMISSION:
                    inst->has_color_props = true;
                    break;
                case NEA_AMTRACK_TEX_SCROLL_X:
                case NEA_AMTRACK_TEX_SCROLL_Y:
                case NEA_AMTRACK_TEX_ROTATE:
                case NEA_AMTRACK_TEX_SCALE_X:
                case NEA_AMTRACK_TEX_SCALE_Y:
                    inst->has_tex_transform = true;
                    break;
            }
        }
    }
}

void NEA_AnimMatSetMaterialTable(NEA_AnimMatInstance *inst,
                                 NEA_Material **table, int count)
{
    NEA_AssertPointer(inst, "NULL instance");

    inst->mat_table = table;
    inst->mat_table_size = (count > 255) ? 255 : (uint8_t)count;
}

void NEA_AnimMatSetBasePolyFormat(NEA_AnimMatInstance *inst,
                                  u32 alpha, u32 id,
                                  NEA_LightEnum lights,
                                  NEA_CullingEnum culling,
                                  NEA_OtherFormatEnum other)
{
    NEA_AssertPointer(inst, "NULL instance");

    inst->base_alpha = alpha;
    inst->base_polyid = id;
    inst->base_lights = lights;
    inst->base_culling = culling;
    inst->base_other = other;
}

// =========================================================================
// Playback control
// =========================================================================

void NEA_AnimMatStart(NEA_AnimMatInstance *inst,
                      NEA_AnimationType type, int32_t speed)
{
    NEA_AssertPointer(inst, "NULL instance");

    inst->type = type;
    inst->speed = speed;
    inst->currframe = 0;
    inst->paused = false;
    inst->active = true;

    // Evaluate initial state
    NEA_AnimMatEvaluate(inst);
}

void NEA_AnimMatStop(NEA_AnimMatInstance *inst)
{
    NEA_AssertPointer(inst, "NULL instance");

    inst->currframe = 0;
    inst->speed = 0;
    inst->active = false;
    inst->paused = false;
}

void NEA_AnimMatPause(NEA_AnimMatInstance *inst, bool paused)
{
    NEA_AssertPointer(inst, "NULL instance");
    inst->paused = paused;
}

void NEA_AnimMatSetSpeed(NEA_AnimMatInstance *inst, int32_t speed)
{
    NEA_AssertPointer(inst, "NULL instance");
    inst->speed = speed;
}

void NEA_AnimMatSetFrame(NEA_AnimMatInstance *inst, int32_t frame)
{
    NEA_AssertPointer(inst, "NULL instance");
    inst->currframe = frame;
    NEA_AnimMatEvaluate(inst);
}

int32_t NEA_AnimMatGetFrame(const NEA_AnimMatInstance *inst)
{
    NEA_AssertPointer(inst, "NULL instance");
    return inst->currframe;
}

// =========================================================================
// Interpolation helpers
// =========================================================================

// Lerp between two RGB15 colors using a fixed-point fraction (0..4096).
static inline uint32_t ne_rgb15_lerp(uint32_t c0, uint32_t c1, int32_t frac)
{
    int r0 = c0 & 0x1F;
    int g0 = (c0 >> 5) & 0x1F;
    int b0 = (c0 >> 10) & 0x1F;
    int r1 = c1 & 0x1F;
    int g1 = (c1 >> 5) & 0x1F;
    int b1 = (c1 >> 10) & 0x1F;

    int r = r0 + ((mulf32(r1 - r0, frac)) >> 12);
    int g = g0 + ((mulf32(g1 - g0, frac)) >> 12);
    int b = b0 + ((mulf32(b1 - b0, frac)) >> 12);

    // Clamp
    if (r < 0) r = 0; else if (r > 31) r = 31;
    if (g < 0) g = 0; else if (g > 31) g = 31;
    if (b < 0) b = 0; else if (b > 31) b = 31;

    return (uint32_t)(r | (g << 5) | (b << 10));
}

// Lerp between two packed dual-RGB15 colors (e.g. diffuse_ambient).
// Lower 15 bits = color A, upper 16 bits = color B (bit 15 = set diffuse flag).
static inline uint32_t ne_packed_color_lerp(uint32_t a, uint32_t b,
                                             int32_t frac)
{
    uint32_t lo = ne_rgb15_lerp(a & 0x7FFF, b & 0x7FFF, frac);
    uint32_t hi = ne_rgb15_lerp((a >> 16) & 0x7FFF, (b >> 16) & 0x7FFF, frac);

    // Preserve the flag bits (bit 15 of each half)
    lo |= (a & 0x8000);
    hi |= (a & 0x80000000) >> 16;

    return lo | (hi << 16);
}

// =========================================================================
// Track evaluation
// =========================================================================

// Evaluate a single track at the given frame. Returns the interpolated value.
ARM_CODE static uint32_t ne_animmat_eval_track(
    const NEA_AnimMatTrack *track, int32_t frame_f32)
{
    if (track->num_keys == 0)
        return 0;

    if (track->num_keys == 1)
        return track->keys[0].value;

    int frame_int = frame_f32 >> 12; // Integer part
    const NEA_AnimMatKeyframe *keys = track->keys;
    int num_keys = track->num_keys;

    // If before first keyframe, return first value
    if (frame_int <= (int)keys[0].frame)
        return keys[0].value;

    // If at or after last keyframe, return last value
    if (frame_int >= (int)keys[num_keys - 1].frame)
        return keys[num_keys - 1].value;

    // Binary search for the bracketing keyframes.
    // With max 64 keyframes, this is at most 6 iterations.
    int lo = 0, hi = num_keys - 1;
    while (lo < hi - 1)
    {
        int mid = (lo + hi) >> 1;
        if ((int)keys[mid].frame <= frame_int)
            lo = mid;
        else
            hi = mid;
    }

    // Step interpolation: return lo's value
    if (track->interp == NEA_AMINTERP_STEP)
        return keys[lo].value;

    // Linear interpolation
    int span = (int)keys[hi].frame - (int)keys[lo].frame;
    if (span <= 0)
        return keys[lo].value;

    // Fixed-point fraction: (frame - lo_frame) / span
    int32_t frac = divf32(
        frame_f32 - inttof32((int)keys[lo].frame),
        inttof32(span)
    );

    // Clamp fraction to [0, 4096]
    if (frac < 0) frac = 0;
    if (frac > inttof32(1)) frac = inttof32(1);

    uint32_t a = keys[lo].value;
    uint32_t b = keys[hi].value;

    // Track-type-specific lerp
    switch (track->type)
    {
        case NEA_AMTRACK_ALPHA:
        {
            int32_t va = (int32_t)(a & 0x1F);
            int32_t vb = (int32_t)(b & 0x1F);
            int32_t result = va + (mulf32(vb - va, frac) >> 12);
            if (result < 0) result = 0;
            if (result > 31) result = 31;
            return (uint32_t)result;
        }
        case NEA_AMTRACK_POLYID:
        {
            int32_t va = (int32_t)(a & 0x3F);
            int32_t vb = (int32_t)(b & 0x3F);
            int32_t result = va + (mulf32(vb - va, frac) >> 12);
            if (result < 0) result = 0;
            if (result > 63) result = 63;
            return (uint32_t)result;
        }
        case NEA_AMTRACK_COLOR:
            return ne_rgb15_lerp(a, b, frac);

        case NEA_AMTRACK_DIFFUSE_AMBIENT:
        case NEA_AMTRACK_SPECULAR_EMISSION:
            return ne_packed_color_lerp(a, b, frac);

        case NEA_AMTRACK_TEX_SCROLL_X:
        case NEA_AMTRACK_TEX_SCROLL_Y:
        case NEA_AMTRACK_TEX_SCALE_X:
        case NEA_AMTRACK_TEX_SCALE_Y:
        {
            // f32 linear interpolation: a + (b - a) * frac
            int32_t sa = (int32_t)a;
            int32_t sb = (int32_t)b;
            return (uint32_t)(sa + mulf32(sb - sa, frac));
        }
        case NEA_AMTRACK_TEX_ROTATE:
        {
            // Integer angle lerp (0-511)
            int32_t va = (int32_t)(a & 0x1FF);
            int32_t vb = (int32_t)(b & 0x1FF);
            int32_t result = va + (mulf32(vb - va, frac) >> 12);
            return (uint32_t)(result & 0x1FF);
        }

        default:
            // Step for lights, culling, material swap
            return a;
    }
}

// =========================================================================
// Evaluate
// =========================================================================

void NEA_AnimMatEvaluate(NEA_AnimMatInstance *inst)
{
    NEA_AssertPointer(inst, "NULL instance");

    if (inst->data == NULL)
        return;

    // Start from base values
    uint32_t alpha = inst->base_alpha;
    uint32_t polyid = inst->base_polyid;
    NEA_LightEnum lights = inst->base_lights;
    NEA_CullingEnum culling = inst->base_culling;
    NEA_OtherFormatEnum other = inst->base_other;

    inst->out_material = NULL;
    inst->out_color = NEA_White;
    inst->out_diff_amb = 0;
    inst->out_spec_emi = 0;
    inst->out_tex_scroll_x = 0;
    inst->out_tex_scroll_y = 0;
    inst->out_tex_rotate = 0;
    inst->out_tex_scale_x = inttof32(1);
    inst->out_tex_scale_y = inttof32(1);

    for (int i = 0; i < inst->data->num_tracks; i++)
    {
        const NEA_AnimMatTrack *track = &inst->data->tracks[i];
        uint32_t val = ne_animmat_eval_track(track, inst->currframe);

        switch (track->type)
        {
            case NEA_AMTRACK_ALPHA:
                alpha = val & 0x1F;
                break;
            case NEA_AMTRACK_LIGHTS:
                lights = (NEA_LightEnum)(val & 0x0F);
                break;
            case NEA_AMTRACK_CULLING:
                culling = (NEA_CullingEnum)(val & 0xC0);
                break;
            case NEA_AMTRACK_COLOR:
                inst->out_color = val;
                break;
            case NEA_AMTRACK_DIFFUSE_AMBIENT:
                inst->out_diff_amb = val;
                break;
            case NEA_AMTRACK_SPECULAR_EMISSION:
                inst->out_spec_emi = val;
                break;
            case NEA_AMTRACK_MATERIAL_SWAP:
            {
                uint8_t idx = (uint8_t)(val & 0xFF);
                if (inst->mat_table != NULL && idx < inst->mat_table_size)
                    inst->out_material = inst->mat_table[idx];
                break;
            }
            case NEA_AMTRACK_POLYID:
                polyid = val & 0x3F;
                break;
            case NEA_AMTRACK_TEX_SCROLL_X:
                inst->out_tex_scroll_x = (int32_t)val;
                break;
            case NEA_AMTRACK_TEX_SCROLL_Y:
                inst->out_tex_scroll_y = (int32_t)val;
                break;
            case NEA_AMTRACK_TEX_ROTATE:
                inst->out_tex_rotate = (int32_t)val;
                break;
            case NEA_AMTRACK_TEX_SCALE_X:
                inst->out_tex_scale_x = (int32_t)val;
                break;
            case NEA_AMTRACK_TEX_SCALE_Y:
                inst->out_tex_scale_y = (int32_t)val;
                break;
        }
    }

    // Build the poly format register value
    inst->out_poly_format = POLY_ALPHA(alpha) | POLY_ID(polyid)
                          | (uint32_t)lights | (uint32_t)culling
                          | (uint32_t)other;
}

// =========================================================================
// Update all instances
// =========================================================================

void NEA_AnimMatUpdateAll(void)
{
    if (!ne_animmat_system_inited)
        return;

    for (int i = 0; i < NEA_MAX_ANIMMAT; i++)
    {
        NEA_AnimMatInstance *inst = NEA_AnimMatPointers[i];
        if (inst == NULL || !inst->active || inst->paused)
            continue;
        if (inst->data == NULL)
            continue;

        // Advance frame
        inst->currframe += inst->speed;

        int32_t endval = inttof32(inst->data->num_frames);

        if (inst->type == NEA_ANIM_LOOP)
        {
            if (inst->currframe >= endval)
                inst->currframe -= endval;
            else if (inst->currframe < 0)
                inst->currframe += endval;
        }
        else if (inst->type == NEA_ANIM_ONESHOT)
        {
            int32_t last = inttof32(inst->data->num_frames - 1);
            if (inst->currframe > last)
            {
                inst->currframe = last;
                inst->speed = 0;
            }
            else if (inst->currframe < 0)
            {
                inst->currframe = 0;
                inst->speed = 0;
            }
        }

        // Evaluate output state
        NEA_AnimMatEvaluate(inst);
    }
}

// =========================================================================
// Apply to GPU
// =========================================================================

void NEA_AnimMatApply(const NEA_AnimMatInstance *inst)
{
    if (inst == NULL || !inst->active)
        return;

    if (inst->has_poly_format)
        GFX_POLY_FORMAT = inst->out_poly_format;

    if (inst->has_material_swap && inst->out_material != NULL)
        NEA_MaterialUse(inst->out_material);

    if (inst->has_color_props)
    {
        GFX_COLOR = inst->out_color;
        GFX_DIFFUSE_AMBIENT = inst->out_diff_amb;
        GFX_SPECULAR_EMISSION = inst->out_spec_emi;
    }

    // Always reset the texture matrix so that a previous apply's transform
    // does not leak into this draw call.  When this instance has its own
    // texture tracks the matrix is then rebuilt; otherwise it stays at identity.
    NEA_TextureMatrixIdentity();

    if (inst->has_tex_transform)
    {
        if (inst->out_tex_scroll_x != 0 || inst->out_tex_scroll_y != 0)
            NEA_TextureMatrixTranslateI(inst->out_tex_scroll_x,
                                         inst->out_tex_scroll_y);
        if (inst->out_tex_rotate != 0)
            NEA_TextureMatrixRotate(inst->out_tex_rotate);
        if (inst->out_tex_scale_x != inttof32(1) ||
            inst->out_tex_scale_y != inttof32(1))
            NEA_TextureMatrixScaleI(inst->out_tex_scale_x,
                                     inst->out_tex_scale_y);
    }
}
