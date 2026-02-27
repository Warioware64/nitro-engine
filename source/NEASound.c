// SPDX-License-Identifier: MIT
//
// Copyright (c) 2024-2026 Warioware64
//
// This file is part of Nitro Engine Advanced

#ifdef NEA_MAXMOD

#include "NEAMain.h"

/// @file NEASound.c

static NEA_SoundSource **ne_sound_sources;
static int ne_max_sound_sources;
static bool ne_sound_system_inited = false;
static NEA_Camera *ne_sound_listener = NULL;

// =========================================================================
// System lifecycle
// =========================================================================

static int ne_sound_alloc_pool(int max_sources)
{
    if (ne_sound_system_inited)
        NEA_SoundSystemEnd();

    if (max_sources < 1)
        ne_max_sound_sources = NEA_DEFAULT_SOUND_SOURCES;
    else
        ne_max_sound_sources = max_sources;

    ne_sound_sources = calloc(ne_max_sound_sources,
                              sizeof(NEA_SoundSource *));
    if (ne_sound_sources == NULL)
    {
        NEA_DebugPrint("Not enough memory");
        return -1;
    }

    ne_sound_listener = NULL;
    ne_sound_system_inited = true;
    return 0;
}

int NEA_SoundSystemResetPool(int max_sources)
{
    return ne_sound_alloc_pool(max_sources);
}

int NEA_SoundSystemReset(mm_addr soundbank, int max_sources)
{
    int ret = ne_sound_alloc_pool(max_sources);
    if (ret != 0)
        return ret;

    soundEnable();
    mmInitDefaultMem(soundbank);
    return 0;
}

int NEA_SoundSystemResetFAT(const char *soundbank_path, int max_sources)
{
    int ret = ne_sound_alloc_pool(max_sources);
    if (ret != 0)
        return ret;

    soundEnable();

    if (!mmInitDefault((char *)soundbank_path))
    {
        NEA_DebugPrint("mmInitDefault failed");
        free(ne_sound_sources);
        ne_sound_sources = NULL;
        ne_sound_system_inited = false;
        return -1;
    }
    return 0;
}

void NEA_SoundSystemEnd(void)
{
    if (!ne_sound_system_inited)
        return;

    NEA_SoundSourceDeleteAll();
    free(ne_sound_sources);
    ne_sound_sources = NULL;
    ne_sound_listener = NULL;
    ne_sound_system_inited = false;
}

// =========================================================================
// Listener
// =========================================================================

void NEA_SoundSetListener(NEA_Camera *cam)
{
    NEA_AssertPointer(cam, "NULL camera pointer");
    ne_sound_listener = cam;
}

// =========================================================================
// Sound source management
// =========================================================================

NEA_SoundSource *NEA_SoundSourceCreate(mm_word sample_id)
{
    if (!ne_sound_system_inited)
    {
        NEA_DebugPrint("System not initialized");
        return NULL;
    }

    for (int i = 0; i < ne_max_sound_sources; i++)
    {
        if (ne_sound_sources[i] != NULL)
            continue;

        NEA_SoundSource *src = calloc(1, sizeof(NEA_SoundSource));
        if (src == NULL)
        {
            NEA_DebugPrint("Not enough memory");
            return NULL;
        }

        src->active = true;
        src->sample_id = sample_id;
        src->ref_volume = 255;
        src->ref_rate = 1024;
        src->min_dist = floattof32(1.0);
        src->max_dist = floattof32(20.0);
        src->computed_panning = 128;
        src->loop_delay = 60; // Default: re-trigger every 1 second

        ne_sound_sources[i] = src;
        return src;
    }

    NEA_DebugPrint("No free slots");
    return NULL;
}

void NEA_SoundSourceDelete(NEA_SoundSource *source)
{
    if (!ne_sound_system_inited)
        return;

    NEA_AssertPointer(source, "NULL pointer");

    // Stop if playing
    if (source->handle != 0)
        mmEffectCancel(source->handle);

    for (int i = 0; i < ne_max_sound_sources; i++)
    {
        if (ne_sound_sources[i] == source)
        {
            ne_sound_sources[i] = NULL;
            free(source);
            return;
        }
    }

    NEA_DebugPrint("Source not found");
}

void NEA_SoundSourceDeleteAll(void)
{
    if (!ne_sound_system_inited)
        return;

    for (int i = 0; i < ne_max_sound_sources; i++)
    {
        if (ne_sound_sources[i] != NULL)
            NEA_SoundSourceDelete(ne_sound_sources[i]);
    }
}

// =========================================================================
// Sound source configuration
// =========================================================================

void NEA_SoundSourceSetModel(NEA_SoundSource *source, NEA_Model *model)
{
    NEA_AssertPointer(source, "NULL pointer");
    source->model = model;
}

void NEA_SoundSourceSetPositionI(NEA_SoundSource *source,
                                  int32_t x, int32_t y, int32_t z)
{
    NEA_AssertPointer(source, "NULL pointer");
    source->position.x = x;
    source->position.y = y;
    source->position.z = z;
}

void NEA_SoundSourceSetDistanceI(NEA_SoundSource *source,
                                  int32_t min_dist, int32_t max_dist)
{
    NEA_AssertPointer(source, "NULL pointer");
    NEA_Assert(min_dist >= 0, "min_dist must be non-negative");
    NEA_Assert(max_dist > min_dist, "max_dist must be greater than min_dist");
    source->min_dist = min_dist;
    source->max_dist = max_dist;
}

void NEA_SoundSourceSetVolume(NEA_SoundSource *source, int volume)
{
    NEA_AssertPointer(source, "NULL pointer");
    NEA_Assert(volume >= 0 && volume <= 255, "Volume must be 0-255");
    source->ref_volume = volume;
}

void NEA_SoundSourceSetRate(NEA_SoundSource *source, mm_hword rate)
{
    NEA_AssertPointer(source, "NULL pointer");
    source->ref_rate = rate;
}

void NEA_SoundSourceSetLoop(NEA_SoundSource *source, bool loop)
{
    NEA_AssertPointer(source, "NULL pointer");
    source->looping = loop;
}

void NEA_SoundSourceSetLoopDelay(NEA_SoundSource *source, uint16_t frames)
{
    NEA_AssertPointer(source, "NULL pointer");
    NEA_Assert(frames > 0, "Loop delay must be > 0");
    source->loop_delay = frames;
}

// =========================================================================
// Sound source playback
// =========================================================================

void NEA_SoundSourcePlay(NEA_SoundSource *source)
{
    NEA_AssertPointer(source, "NULL pointer");

    // Stop previous playback if any
    if (source->handle != 0)
        mmEffectCancel(source->handle);

    mm_sound_effect sfx;
    sfx.id = source->sample_id;
    sfx.rate = source->ref_rate;
    sfx.handle = 0;
    sfx.volume = source->computed_volume;
    sfx.panning = source->computed_panning;

    source->handle = mmEffectEx(&sfx);
    source->playing = true;
    source->loop_counter = source->loop_delay;
}

void NEA_SoundSourceStop(NEA_SoundSource *source)
{
    NEA_AssertPointer(source, "NULL pointer");

    if (source->handle != 0)
    {
        mmEffectCancel(source->handle);
        source->handle = 0;
    }
    source->playing = false;
}

bool NEA_SoundSourceIsPlaying(const NEA_SoundSource *source)
{
    NEA_AssertPointer(source, "NULL pointer");
    return source->playing;
}

// =========================================================================
// Spatial update
// =========================================================================

// Compute volume and panning for a single source relative to the listener.
static void ne_sound_compute_spatial(NEA_SoundSource *source,
                                      NEA_Vec3 cam_pos, NEA_Vec3 right_vec)
{
    // Get source world position
    NEA_Vec3 src_pos;
    if (source->model != NULL)
        src_pos = NEA_Vec3Make(source->model->x,
                               source->model->y,
                               source->model->z);
    else
        src_pos = source->position;

    // Vector from listener to source
    NEA_Vec3 diff = NEA_Vec3Sub(src_pos, cam_pos);

    // Distance (64-bit precision, same as NEACollision.c)
    int64_t dist_sq_64 = (int64_t)diff.x * diff.x
                       + (int64_t)diff.y * diff.y
                       + (int64_t)diff.z * diff.z;
    uint32_t dist = (uint32_t)sqrt64((uint64_t)dist_sq_64);

    // --- Volume attenuation (linear) ---

    int32_t vol;
    if (dist <= (uint32_t)source->min_dist)
    {
        vol = source->ref_volume;
    }
    else if (dist >= (uint32_t)source->max_dist)
    {
        vol = 0;
    }
    else
    {
        int32_t range = source->max_dist - source->min_dist;
        int32_t num = source->max_dist - (int32_t)dist;
        vol = (int32_t)(((int64_t)source->ref_volume * num) / range);
        if (vol < 0)
            vol = 0;
        if (vol > 255)
            vol = 255;
    }
    source->computed_volume = (mm_byte)vol;

    // --- Panning (dot product with camera right vector) ---

    int32_t pan;
    if (dist > 0)
    {
        int32_t dot = NEA_Vec3Dot(diff, right_vec);
        int32_t pan_f32 = divf32(dot, (int32_t)dist);

        // Map from f32 range [-4096, +4096] to [0, 255]
        // 128 = center, 0 = full left, 255 = full right
        pan = 128 + ((pan_f32 * 127) >> 12);
        if (pan < 0)
            pan = 0;
        if (pan > 255)
            pan = 255;
    }
    else
    {
        pan = 128; // Source at listener position: center
    }
    source->computed_panning = (mm_byte)pan;
}

ARM_CODE void NEA_SoundUpdateAll(void)
{
    if (!ne_sound_system_inited)
        return;
    if (ne_sound_listener == NULL)
        return;

    NEA_Camera *cam = ne_sound_listener;
    NEA_Vec3 cam_pos = NEA_Vec3Make(cam->from[0], cam->from[1], cam->from[2]);

    // Compute camera right vector (same pattern as NEA_CameraMoveFreeI)
    int32_t vec_front[3], vec_right[3];
    for (int i = 0; i < 3; i++)
        vec_front[i] = cam->to[i] - cam->from[i];

    crossf32(vec_front, cam->up, vec_right);
    normalizef32(vec_right);

    NEA_Vec3 right = NEA_Vec3Make(vec_right[0], vec_right[1], vec_right[2]);

    for (int i = 0; i < ne_max_sound_sources; i++)
    {
        NEA_SoundSource *src = ne_sound_sources[i];
        if (src == NULL || !src->active || !src->playing)
            continue;

        ne_sound_compute_spatial(src, cam_pos, right);

        if (src->handle != 0)
        {
            mmEffectVolume(src->handle, src->computed_volume);
            mmEffectPanning(src->handle, src->computed_panning);
        }

        // Re-trigger looping sources using a frame countdown timer.
        // DS maxmod has no mmEffectActive(), so we count frames and
        // re-trigger after loop_delay frames have elapsed.
        if (src->looping)
        {
            if (src->loop_counter > 0)
            {
                src->loop_counter--;
            }
            else
            {
                mm_sound_effect sfx;
                sfx.id = src->sample_id;
                sfx.rate = src->ref_rate;
                sfx.handle = 0;
                sfx.volume = src->computed_volume;
                sfx.panning = src->computed_panning;
                src->handle = mmEffectEx(&sfx);
                src->loop_counter = src->loop_delay;
            }
        }
    }
}

// =========================================================================
// Music (non-spatial wrappers)
// =========================================================================

void NEA_MusicStart(mm_word module_id, mm_pmode mode)
{
    mmLoad(module_id);
    mmStart(module_id, mode);
}

void NEA_MusicStop(void)
{
    mmStop();
}

void NEA_MusicPause(void)
{
    mmPause();
}

void NEA_MusicResume(void)
{
    mmResume();
}

void NEA_MusicSetVolume(mm_word volume)
{
    mmSetModuleVolume(volume);
}

void NEA_MusicSetTempo(mm_word tempo)
{
    mmSetModuleTempo(tempo);
}

void NEA_MusicSetPitch(mm_word pitch)
{
    mmSetModulePitch(pitch);
}

bool NEA_MusicIsPlaying(void)
{
    return mmActive();
}

// =========================================================================
// Non-spatial SFX convenience
// =========================================================================

void NEA_SfxLoad(mm_word sample_id)
{
    mmLoadEffect(sample_id);
}

void NEA_SfxUnload(mm_word sample_id)
{
    mmUnloadEffect(sample_id);
}

mm_sfxhand NEA_SfxPlay(mm_word sample_id)
{
    return mmEffect(sample_id);
}

mm_sfxhand NEA_SfxPlayEx(mm_word sample_id, mm_byte volume,
                          mm_byte panning, mm_hword rate)
{
    mm_sound_effect sfx;
    sfx.id = sample_id;
    sfx.rate = rate;
    sfx.handle = 0;
    sfx.volume = volume;
    sfx.panning = panning;
    return mmEffectEx(&sfx);
}

void NEA_SfxSetRate(mm_sfxhand handle, mm_hword rate)
{
    mmEffectRate(handle, rate);
}

void NEA_SfxSetVolume(mm_sfxhand handle, mm_word volume)
{
    mmEffectVolume(handle, volume);
}

void NEA_SfxSetPanning(mm_sfxhand handle, mm_byte panning)
{
    mmEffectPanning(handle, panning);
}

void NEA_SfxRelease(mm_sfxhand handle)
{
    mmEffectRelease(handle);
}

void NEA_SfxStop(mm_sfxhand handle)
{
    mmEffectCancel(handle);
}

void NEA_SfxStopAll(void)
{
    mmEffectCancelAll();
}

// =========================================================================
// WAV streaming
// =========================================================================

void NEA_StreamOpen(mm_word sampling_rate, mm_word buffer_length,
                    mm_stream_func callback, mm_word format,
                    mm_word timer)
{
    mm_stream stream;
    stream.sampling_rate = sampling_rate;
    stream.buffer_length = buffer_length;
    stream.callback = callback;
    stream.format = format;
    stream.timer = timer;
    stream.manual = false;
    mmStreamOpen(&stream);
}

void NEA_StreamClose(void)
{
    mmStreamClose();
}

mm_word NEA_StreamGetPosition(void)
{
    return mmStreamGetPosition();
}

#endif // NEA_MAXMOD
