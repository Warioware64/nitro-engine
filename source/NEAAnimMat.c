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

// Version 1: one material, tracks straight after the header.
//
//   header_t | track_hdr_v1[num_tracks] | keyframe data
//
// Version 2: tracks grouped under named material targets, and each track says
// how its values are stored.
//
//   header_t | target_hdr[num_targets] | track_hdr_v2[] ... | value data
//
// The two file headers are the same 16 bytes, and num_tracks/num_targets sit in
// the same place, so the version field is all that has to be read to tell them
// apart.

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint16_t count;        // v1: number of tracks. v2: number of targets.
    uint16_t num_frames;
    uint8_t  reserved[4];
} neaanimmat_header_t;

typedef struct {
    uint8_t  track_type;
    uint8_t  interp_mode;
    uint16_t num_keys;
    uint32_t key_offset;
    uint32_t reserved;
} neaanimmat_track_header_v1_t;

typedef struct {
    uint8_t  track_type;
    uint8_t  interp_mode;
    uint8_t  storage;
    uint8_t  reserved;
    uint16_t count;        // KEYS: keyframes. BAKED: frames. CONST: unused.
    uint16_t pad;
    uint32_t data;         // KEYS/BAKED: byte offset. CONST: the value itself.
} neaanimmat_track_header_v2_t;

typedef struct {
    char     name[NEA_MATERIAL_NAME_LEN];
    uint16_t num_tracks;
    uint16_t pad;
    uint32_t track_offset; // Byte offset of this target's track headers.
} neaanimmat_target_header_t;

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

// A baked track stores 16 bit values, so the two packed dual-colour tracks
// cannot use it: they need a full 32 bits. Everything else fits.
static bool ne_animmat_baked_ok(uint8_t type)
{
    return (type != NEA_AMTRACK_DIFFUSE_AMBIENT) &&
           (type != NEA_AMTRACK_SPECULAR_EMISSION);
}

// Fills in one target's tracks from a version 2 track header array.
static void ne_animmat_parse_tracks_v2(NEA_AnimMatTarget *target,
                                       const uint8_t *base,
                                       const neaanimmat_track_header_v2_t *hdrs,
                                       int num_tracks)
{
    for (int i = 0; i < num_tracks; i++)
    {
        NEA_AnimMatTrack *track = &target->tracks[i];

        track->type = (NEA_AnimMatTrackType)hdrs[i].track_type;
        track->interp = (NEA_AnimMatInterp)hdrs[i].interp_mode;
        track->storage = (NEA_AnimMatStorage)hdrs[i].storage;
        track->count = hdrs[i].count;

        switch (track->storage)
        {
            case NEA_AMSTORE_CONST:
                track->value = hdrs[i].data;
                track->count = 0;
                break;

            case NEA_AMSTORE_BAKED:
                if (!ne_animmat_baked_ok(hdrs[i].track_type))
                {
                    // A 32 bit value in a 16 bit slot would be silently wrong,
                    // so refuse the track rather than render something odd.
                    NEA_DebugPrint("Track type %d cannot be baked",
                                   hdrs[i].track_type);
                    track->storage = NEA_AMSTORE_CONST;
                    track->value = 0;
                    track->count = 0;
                    break;
                }
                track->baked = (const int16_t *)(base + hdrs[i].data);
                break;

            case NEA_AMSTORE_KEYS:
            default:
                track->storage = NEA_AMSTORE_KEYS;
                if (track->count > NEA_ANIMMAT_MAX_KEYFRAMES)
                    track->count = NEA_ANIMMAT_MAX_KEYFRAMES;
                track->keys = (const NEA_AnimMatKeyframe *)(base + hdrs[i].data);
                break;
        }
    }
}

// Allocates the data object and its target array.
static NEA_AnimMatData *ne_animmat_alloc(int num_targets, int num_frames,
                                         const void *data, bool from_fat)
{
    NEA_AnimMatData *amd = calloc(1, sizeof(NEA_AnimMatData));
    if (amd == NULL)
    {
        NEA_DebugPrint("Not enough memory for animmat data");
        return NULL;
    }

    // Sized to what the file actually holds, so a one-material animation costs
    // one target rather than the maximum.
    amd->targets = calloc(num_targets, sizeof(NEA_AnimMatTarget));
    if (amd->targets == NULL)
    {
        NEA_DebugPrint("Not enough memory for animmat targets");
        free(amd);
        return NULL;
    }

    amd->num_targets = num_targets;
    amd->num_frames = num_frames;
    amd->_base_data = (void *)data;
    amd->_loaded_from_fat = from_fat;

    return amd;
}

// Version 1 files hold one material's tracks with no name and no storage field.
// They parse into a single unnamed target, which is exactly what a version 2
// file with one target looks like, so everything downstream is version-blind.
static NEA_AnimMatData *ne_animmat_parse_v1(const uint8_t *base,
                                            const neaanimmat_header_t *hdr,
                                            bool from_fat)
{
    if (hdr->count < 1 || hdr->count > NEA_ANIMMAT_MAX_TRACKS)
    {
        NEA_DebugPrint("Invalid track count");
        return NULL;
    }

    NEA_AnimMatData *amd = ne_animmat_alloc(1, hdr->num_frames, base, from_fat);
    if (amd == NULL)
        return NULL;

    NEA_AnimMatTarget *target = &amd->targets[0];
    target->name[0] = '\0';
    target->num_tracks = hdr->count;

    const neaanimmat_track_header_v1_t *track_hdrs =
        (const neaanimmat_track_header_v1_t *)(base + sizeof(neaanimmat_header_t));

    for (int i = 0; i < target->num_tracks; i++)
    {
        NEA_AnimMatTrack *track = &target->tracks[i];

        track->type = (NEA_AnimMatTrackType)track_hdrs[i].track_type;
        track->interp = (NEA_AnimMatInterp)track_hdrs[i].interp_mode;
        track->storage = NEA_AMSTORE_KEYS;
        track->count = track_hdrs[i].num_keys;

        if (track->count > NEA_ANIMMAT_MAX_KEYFRAMES)
            track->count = NEA_ANIMMAT_MAX_KEYFRAMES;

        // Keyframe data is at the offset specified in the track header.
        // Pointers reference directly into the loaded data buffer.
        track->keys = (const NEA_AnimMatKeyframe *)(base +
                                                    track_hdrs[i].key_offset);
    }

    return amd;
}

static NEA_AnimMatData *ne_animmat_parse_v2(const uint8_t *base,
                                            const neaanimmat_header_t *hdr,
                                            bool from_fat)
{
    if (hdr->count < 1 || hdr->count > NEA_ANIMMAT_MAX_TARGETS)
    {
        NEA_DebugPrint("Invalid target count");
        return NULL;
    }

    NEA_AnimMatData *amd = ne_animmat_alloc(hdr->count, hdr->num_frames,
                                            base, from_fat);
    if (amd == NULL)
        return NULL;

    const neaanimmat_target_header_t *target_hdrs =
        (const neaanimmat_target_header_t *)(base + sizeof(neaanimmat_header_t));

    for (int t = 0; t < amd->num_targets; t++)
    {
        NEA_AnimMatTarget *target = &amd->targets[t];

        memcpy(target->name, target_hdrs[t].name, NEA_MATERIAL_NAME_LEN);
        target->name[NEA_MATERIAL_NAME_LEN - 1] = '\0';

        target->num_tracks = target_hdrs[t].num_tracks;
        if (target->num_tracks > NEA_ANIMMAT_MAX_TRACKS)
            target->num_tracks = NEA_ANIMMAT_MAX_TRACKS;

        ne_animmat_parse_tracks_v2(
            target, base,
            (const neaanimmat_track_header_v2_t *)(base +
                                                   target_hdrs[t].track_offset),
            target->num_tracks);
    }

    return amd;
}

static NEA_AnimMatData *ne_animmat_parse(const void *data, bool from_fat)
{
    const uint8_t *base = (const uint8_t *)data;
    const neaanimmat_header_t *hdr = (const neaanimmat_header_t *)base;

    if (hdr->magic != NEA_ANIMMAT_MAGIC)
    {
        NEA_DebugPrint("Invalid .neaanimmat magic");
        return NULL;
    }

    if (hdr->version == NEA_ANIMMAT_VERSION_1)
        return ne_animmat_parse_v1(base, hdr, from_fat);

    if (hdr->version == NEA_ANIMMAT_VERSION_2)
        return ne_animmat_parse_v2(base, hdr, from_fat);

    NEA_DebugPrint("Unsupported .neaanimmat version");
    return NULL;
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

// Parameters of an asynchronous NEA_AnimMatDataLoadFATAsync() job.
typedef struct {
    NEA_AnimMatData **out;
} ne_async_animmat_param;

// Runs on the main thread during the vertical blank: parses the loaded file and
// hands the resulting object to the caller's storage slot.
static void ne_async_animmat_finalize(NEA_AsyncFile *job)
{
    ne_async_animmat_param *p = __NEA_AsyncParam(job);

    // Take ownership of the buffer: the keyframe pointers of the parsed object
    // reference it directly, so NEA_AnimMatDataFree() frees it.
    char *data = __NEA_AsyncTakeBuffer(job, NULL);
    if (data == NULL)
    {
        __NEA_AsyncSetResult(job, 0);
        return;
    }

    NEA_AnimMatData *amd = ne_animmat_parse(data, true);
    if (amd == NULL)
    {
        free(data);
        __NEA_AsyncSetResult(job, 0);
        return;
    }

    *p->out = amd;
    __NEA_AsyncSetResult(job, 1);
}

NEA_AsyncFile *NEA_AnimMatDataLoadFATAsync(NEA_AnimMatData **out,
                                           const char *path)
{
    NEA_AssertPointer(out, "NULL out pointer");
    NEA_AssertPointer(path, "NULL path");

    ne_async_animmat_param *p = malloc(sizeof(ne_async_animmat_param));
    if (p == NULL)
    {
        NEA_DebugPrint("Not enough memory");
        return NULL;
    }

    p->out = out;

    // The target is the storage slot itself: this loader creates the object
    // instead of writing into an existing one.
    NEA_AsyncFile *job = __NEA_AsyncQueue(path, NULL,
                                          ne_async_animmat_finalize, NULL, p,
                                          out);
    if (job == NULL)
        free(p);

    return job;
}

void NEA_AnimMatDataFree(NEA_AnimMatData *data)
{
    if (data == NULL)
        return;

    if (data->_loaded_from_fat && data->_base_data != NULL)
        free(data->_base_data);

    free(data->targets);
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

    // The has_* flags are not set here any more. With several targets in one
    // animation a union of their tracks would be wrong: a target with no
    // texture tracks would still take the texture-matrix path because some
    // other target had one. They are computed per target in
    // NEA_AnimMatEvaluateTarget() instead, where they describe exactly the
    // target that is about to be applied.
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

void NEA_AnimMatSetTexPalTables(NEA_AnimMatInstance *inst,
                                NEA_Material **textures, int num_tex,
                                NEA_Palette **palettes, int num_pal)
{
    NEA_AssertPointer(inst, "NULL instance");
    NEA_AssertMinMax(0, num_tex, 255, "Invalid texture table size %d", num_tex);
    NEA_AssertMinMax(0, num_pal, 255, "Invalid palette table size %d", num_pal);

    inst->tex_table = textures;
    inst->tex_table_size = (uint8_t)num_tex;
    inst->pal_table = palettes;
    inst->pal_table_size = (uint8_t)num_pal;
}

int NEA_AnimMatFindTarget(const NEA_AnimMatData *data, const char *name)
{
    if (data == NULL || name == NULL)
        return -1;

    for (int i = 0; i < data->num_targets; i++)
    {
        if (strcmp(data->targets[i].name, name) == 0)
            return i;
    }

    return -1;
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

    // mulf32() already brings the product back down by 12 bits. Shifting the
    // result again would divide it by a further 4096, which leaves the channel
    // sitting on its start value for the whole span.
    int r = r0 + mulf32(r1 - r0, frac);
    int g = g0 + mulf32(g1 - g0, frac);
    int b = b0 + mulf32(b1 - b0, frac);

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

// Widens a baked 16 bit value to the track's runtime representation.
//
// The four texture translate/scale tracks are f32 at runtime but are baked as
// 1.10.5, the encoding retail DS material animations use for the same channels,
// so they need a shift. Every other track's value already fits in 16 bits and
// is used as-is.
ARM_CODE static inline uint32_t ne_animmat_widen_baked(uint8_t type, int16_t v)
{
    switch (type)
    {
        case NEA_AMTRACK_TEX_SCROLL_X:
        case NEA_AMTRACK_TEX_SCROLL_Y:
        case NEA_AMTRACK_TEX_SCALE_X:
        case NEA_AMTRACK_TEX_SCALE_Y:
            // 1.10.5 -> 1.19.12
            return (uint32_t)((int32_t)v << 7);

        default:
            return (uint32_t)(uint16_t)v;
    }
}

// Evaluate a single track at the given frame. Returns the interpolated value.
ARM_CODE static uint32_t ne_animmat_eval_track(
    const NEA_AnimMatTrack *track, int32_t frame_f32)
{
    // A constant track is the whole point of having storage modes: no array to
    // reach into, nothing to search, nothing to interpolate.
    if (track->storage == NEA_AMSTORE_CONST)
        return track->value;

    if (track->count == 0)
        return 0;

    // A baked track is one indexed load. No search, no division, no lerp: the
    // exporter has already done the interpolation for every frame.
    if (track->storage == NEA_AMSTORE_BAKED)
    {
        int frame_int = frame_f32 >> 12;

        if (frame_int < 0)
            frame_int = 0;
        else if (frame_int >= (int)track->count)
            frame_int = track->count - 1;

        return ne_animmat_widen_baked((uint8_t)track->type,
                                      track->baked[frame_int]);
    }

    if (track->count == 1)
        return track->keys[0].value;

    int frame_int = frame_f32 >> 12; // Integer part
    const NEA_AnimMatKeyframe *keys = track->keys;
    int num_keys = track->count;

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
            int32_t result = va + mulf32(vb - va, frac);
            if (result < 0) result = 0;
            if (result > 31) result = 31;
            return (uint32_t)result;
        }
        case NEA_AMTRACK_POLYID:
        {
            int32_t va = (int32_t)(a & 0x3F);
            int32_t vb = (int32_t)(b & 0x3F);
            int32_t result = va + mulf32(vb - va, frac);
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
            int32_t result = va + mulf32(vb - va, frac);
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

// ARM mode: 13 of the 32x32->64 multiplies in this file are in here, and
// Thumb-1 on ARMv5TE has no SMULL, so each one was an __aeabi_lmul call.
ARM_CODE
void NEA_AnimMatEvaluateTarget(NEA_AnimMatInstance *inst, int target)
{
    NEA_AssertPointer(inst, "NULL instance");

    if (inst->data == NULL)
        return;

    if (target < 0 || target >= (int)inst->data->num_targets)
        return;

    const NEA_AnimMatTarget *tgt = &inst->data->targets[target];

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
    inst->out_texture = NULL;
    inst->out_palette = NULL;

    inst->has_poly_format = false;
    inst->has_material_swap = false;
    inst->has_color = false;
    inst->has_diff_amb = false;
    inst->has_spec_emi = false;
    inst->has_tex_transform = false;
    inst->has_texpal_swap = false;

    for (int i = 0; i < tgt->num_tracks; i++)
    {
        const NEA_AnimMatTrack *track = &tgt->tracks[i];
        uint32_t val = ne_animmat_eval_track(track, inst->currframe);

        switch (track->type)
        {
            case NEA_AMTRACK_ALPHA:
                alpha = val & 0x1F;
                inst->has_poly_format = true;
                break;
            case NEA_AMTRACK_LIGHTS:
                lights = (NEA_LightEnum)(val & 0x0F);
                inst->has_poly_format = true;
                break;
            case NEA_AMTRACK_CULLING:
                culling = (NEA_CullingEnum)(val & 0xC0);
                inst->has_poly_format = true;
                break;
            case NEA_AMTRACK_COLOR:
                inst->out_color = val;
                inst->has_color = true;
                break;
            case NEA_AMTRACK_DIFFUSE_AMBIENT:
                inst->out_diff_amb = val;
                inst->has_diff_amb = true;
                break;
            case NEA_AMTRACK_SPECULAR_EMISSION:
                inst->out_spec_emi = val;
                inst->has_spec_emi = true;
                break;
            case NEA_AMTRACK_MATERIAL_SWAP:
            {
                uint8_t idx = (uint8_t)(val & 0xFF);
                if (inst->mat_table != NULL && idx < inst->mat_table_size)
                    inst->out_material = inst->mat_table[idx];
                inst->has_material_swap = true;
                break;
            }
            case NEA_AMTRACK_TEXPAL_SWAP:
            {
                // 0xFF in either half means "leave this one alone", so a track
                // can flip the palette while holding the texture, or the other
                // way round.
                uint8_t tex_idx = (uint8_t)((val >> 8) & 0xFF);
                uint8_t pal_idx = (uint8_t)(val & 0xFF);

                if (inst->tex_table != NULL && tex_idx < inst->tex_table_size)
                    inst->out_texture = inst->tex_table[tex_idx];
                if (inst->pal_table != NULL && pal_idx < inst->pal_table_size)
                    inst->out_palette = inst->pal_table[pal_idx];
                inst->has_texpal_swap = true;
                break;
            }
            case NEA_AMTRACK_POLYID:
                polyid = val & 0x3F;
                inst->has_poly_format = true;
                break;
            case NEA_AMTRACK_TEX_SCROLL_X:
                inst->out_tex_scroll_x = (int32_t)val;
                inst->has_tex_transform = true;
                break;
            case NEA_AMTRACK_TEX_SCROLL_Y:
                inst->out_tex_scroll_y = (int32_t)val;
                inst->has_tex_transform = true;
                break;
            case NEA_AMTRACK_TEX_ROTATE:
                inst->out_tex_rotate = (int32_t)val;
                inst->has_tex_transform = true;
                break;
            case NEA_AMTRACK_TEX_SCALE_X:
                inst->out_tex_scale_x = (int32_t)val;
                inst->has_tex_transform = true;
                break;
            case NEA_AMTRACK_TEX_SCALE_Y:
                inst->out_tex_scale_y = (int32_t)val;
                inst->has_tex_transform = true;
                break;
        }
    }

    // Build the poly format register value
    inst->out_poly_format = POLY_ALPHA(alpha) | POLY_ID(polyid)
                          | (uint32_t)lights | (uint32_t)culling
                          | (uint32_t)other;
}

void NEA_AnimMatEvaluate(NEA_AnimMatInstance *inst)
{
    NEA_AnimMatEvaluateTarget(inst, 0);
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

// Loads the whole texture transform in one go.
//
// The per-operation helpers in NEATexture.c each switch MATRIX_CONTROL to
// GL_TEXTURE and back, so building a translate + rotate + scale through them
// costs six mode switches. This switches once.
static void ne_animmat_load_tex_matrix(const NEA_AnimMatInstance *inst)
{
    MATRIX_CONTROL = GL_TEXTURE;
    MATRIX_IDENTITY = 0;

    if (inst->out_tex_scroll_x != 0 || inst->out_tex_scroll_y != 0)
    {
        MATRIX_TRANSLATE = inst->out_tex_scroll_x;
        MATRIX_TRANSLATE = inst->out_tex_scroll_y;
        MATRIX_TRANSLATE = 0;
    }

    if (inst->out_tex_rotate != 0)
        glRotateZi(inst->out_tex_rotate << 6);

    if (inst->out_tex_scale_x != inttof32(1) ||
        inst->out_tex_scale_y != inttof32(1))
    {
        MATRIX_SCALE = inst->out_tex_scale_x;
        MATRIX_SCALE = inst->out_tex_scale_y;
        MATRIX_SCALE = inttof32(1);
    }

    MATRIX_CONTROL = GL_MODELVIEW;
}

void NEA_AnimMatApply(NEA_AnimMatInstance *inst)
{
    if (inst == NULL || !inst->active)
        return;

    if (inst->has_poly_format)
        GFX_POLY_FORMAT = inst->out_poly_format;

    if (inst->has_material_swap && inst->out_material != NULL)
        NEA_MaterialUse(inst->out_material);

    // A texture/palette swap changes only the image and its palette, leaving
    // whatever material the caller (or a material swap above) already bound.
    if (inst->has_texpal_swap)
    {
        if (inst->out_texture != NULL)
            NEA_MaterialTexUse(inst->out_texture);
        if (inst->out_palette != NULL)
            NEA_PaletteUse(inst->out_palette);
    }

    // Each register is written only if a track actually drives it. Writing all
    // three whenever any one of them is animated would wipe out the material's
    // own diffuse and specular the moment a target animated just its colour.
    if (inst->has_color)
        GFX_COLOR = inst->out_color;
    if (inst->has_diff_amb)
        GFX_DIFFUSE_AMBIENT = inst->out_diff_amb;
    if (inst->has_spec_emi)
        GFX_SPECULAR_EMISSION = inst->out_spec_emi;

    if (inst->has_tex_transform)
    {
        ne_animmat_load_tex_matrix(inst);
        inst->tex_matrix_dirty = true;
    }
    else if (inst->tex_matrix_dirty)
    {
        // Only reset when a previous apply actually left a transform loaded.
        // Resetting unconditionally, which is what this used to do, cost an
        // identity load every frame for every instance that never touches the
        // texture matrix at all.
        NEA_TextureMatrixIdentity();
        inst->tex_matrix_dirty = false;
    }
}

void NEA_AnimMatApplyTarget(NEA_AnimMatInstance *inst, int target)
{
    if (inst == NULL || !inst->active)
        return;

    NEA_AnimMatEvaluateTarget(inst, target);
    NEA_AnimMatApply(inst);
}
