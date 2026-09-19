// SPDX-License-Identifier: MIT
//
// Copyright (c) 2024-2026 Warioware64
//
// This file is part of Nitro Engine Advanced

#ifndef NEA_SOUND_H__
#define NEA_SOUND_H__

/// @file   NEASound.h
/// @brief  Optional spatial sound system using Maxmod.
///
/// This module is only available when NEA_MAXMOD is defined. It wraps the
/// Maxmod audio library and adds spatial sound: sources attached to models
/// or world positions that compute left/right panning and distance-based
/// volume attenuation relative to a camera listener.
///
/// To enable, build the library with NEA_MAXMOD=1 and link your project
/// with -lmm9 -DNEA_MAXMOD.
///
/// @section sndexcnt The DSi sound extension
///
/// A DSi has a stage after the DS sound mixer that a DS does not: the I2S link
/// to the TSC2117 codec, controlled by the SNDEXCNT register. Two of its fields
/// are worth reaching, and they happen to be the only two the ARM9 can reach at
/// all: the I2S output rate, and the ratio in which the DSP and the ARM sound
/// hardware are mixed into the speakers.
///
/// Both are off the beaten path, so this is opt-in and defaults to exactly what
/// a DS does. Nothing here changes unless you ask for it.
///
/// @subsection sndexcnt_avail When it does nothing
///
/// On a DS there is no such hardware and every function here is a no-op. Less
/// obviously, the same is true on a DSi running a DS-only (NTR) ROM: the TWL
/// codec is not reachable, and the ARM7 drops the request. Checking
/// isDSiMode() will not tell you the difference, because it reports the
/// console rather than the mode. NEA_SoundDSiOutputAvailable() will.
///
/// A stock BlocksDS build already qualifies: ndstool sets unit code 2 and the
/// application flag from the ELF, so the examples here and a project from the
/// templates need no extra flags. It is worth knowing which flag is load
/// bearing, though, because a ROM repacked by hand without `-p 0x01` will
/// build, boot and play sound, and silently lose this one feature.
///
/// @subsection sndexcnt_ratio The mix ratio is a volume control for Maxmod
///
/// Maxmod plays through the DS sound channels, which are the *ARM* half of the
/// DSP/ARM ratio. A ratio of 8 is 100% ARM and is the default; lowering it
/// attenuates your music and your effects along with everything else Maxmod is
/// doing. Only lower it if you are running audio on the Teak DSP and want to
/// hear both.
///
/// @subsection sndexcnt_rate Changing the rate is not free
///
/// Selecting a rate reprograms the codec's PLL over SPI, and everything clocked
/// off I2S moves with it. Do it with audio stopped, touch input idle and the
/// microphone not recording -- in practice, once, before
/// NEA_SoundSystemReset(). Your setting is remembered and re-applied across
/// sound system re-initialization, so calling it first is safe and is the
/// recommended order.
///
/// NEA does not stop playback for you, and the omission is deliberate: Maxmod
/// on DS cannot report which module is loaded or how far into it playback has
/// got, so stopping and restarting would drop the track back to its first bar.
/// If you must change the rate mid-game, do it yourself:
///
///     NEA_MusicPause();
///     NEA_SfxStopAll();
///     NEA_SoundEnableDSiOutput(NEA_DSI_SOUND_FREQ_47KHZ);
///     NEA_MusicResume();
///
/// A stream has no pause, because it is refilled off a hardware timer. Close it
/// and open it again instead, leaving your own buffer alone so playback carries
/// on from where it was:
///
///     NEA_StreamClose();
///     NEA_SoundEnableDSiOutput(NEA_DSI_SOUND_FREQ_47KHZ);
///     NEA_StreamOpen(rate, len, callback, format, timer);
///
/// Only NEA_StreamGetPosition() restarts across that, since Maxmod counts
/// samples per stream rather than per file.
///
/// Asking for the rate that is already set costs nothing -- the ARM7 compares
/// against the register and returns without touching the codec -- so the dance
/// above is only needed when the rate actually changes.
///
/// Note also that the DSi microphone rates are divisions of this rate, so
/// choosing 47.61 kHz replaces the 32730/16360/10910/8180 Hz ladder with
/// 47610/23810/15870/11900 Hz.
///
/// @subsection sndexcnt_mute What is missing, and why
///
/// SNDEXCNT also has a mute bit and an output enable bit. Neither is exposed
/// here, because neither can be reached: SNDEXCNT is an ARM7 register, the ARM9
/// can only ask the ARM7 to change it, and libnds offers no request for those
/// two fields. Reaching them would take a custom ARM7 core. For muting, use
/// NEA_MusicSetVolume() or soundSetMasterVolume() instead.

#ifdef NEA_MAXMOD

#include <maxmod9.h>
#include "NEACamera.h"
#include "NEAModel.h"
#include "NEACollision.h"

/// @defgroup sound Spatial sound system
///
/// Optional spatial sound system powered by Maxmod. Provides 3D-positioned
/// sound effects with automatic panning and distance attenuation, plus
/// wrappers for music playback, non-spatial SFX, and WAV streaming.
///
/// @{

#define NEA_DEFAULT_SOUND_SOURCES 32 ///< Default max number of sound sources.

/// Holds information for a spatial sound source.
typedef struct {
    bool active;              ///< True if this source is in use.
    bool looping;             ///< True if the effect should auto-restart.
    bool playing;             ///< True if currently playing.

    mm_word sample_id;        ///< Maxmod sample ID (SFX_xxx).
    mm_sfxhand handle;        ///< Current Maxmod effect handle (0 if none).

    NEA_Model *model;         ///< If non-NULL, position follows this model.
    NEA_Vec3 position;        ///< Fixed position when model is NULL (f32).

    int32_t min_dist;         ///< Distance at which volume is full (f32).
    int32_t max_dist;         ///< Distance beyond which volume is zero (f32).
    int32_t ref_volume;       ///< Base volume before attenuation (0-255).
    mm_hword ref_rate;        ///< Base playback rate (6.10 fixed, 1024 = normal).

    uint16_t loop_delay;      ///< Frames between loop re-triggers.
    uint16_t loop_counter;    ///< Countdown until next re-trigger.

    mm_byte computed_volume;  ///< Attenuated volume after spatial update.
    mm_byte computed_panning; ///< Spatial panning after spatial update.
} NEA_SoundSource;

// =========================================================================
// System lifecycle
// =========================================================================

/// Initialize the sound system from an embedded soundbank (bin2c/mmutil).
///
/// Calls mmInitDefaultMem() with the provided data pointer. Use this when
/// the soundbank is embedded in the ARM9 binary via BINDIRS/bin2c.
///
/// @param soundbank Address of the embedded soundbank data.
/// @param max_sources Max spatial sources. If < 1, uses
///                    NEA_DEFAULT_SOUND_SOURCES.
/// @return 0 on success, -1 on failure.
int NEA_SoundSystemReset(mm_addr soundbank, int max_sources);

/// Initialize the sound system from a FAT/NitroFS soundbank file.
///
/// Calls mmInitDefault() with the provided file path. Use this when
/// the soundbank is stored in NitroFS or on the FAT filesystem.
///
/// @param soundbank_path Path to the soundbank file (e.g.
///                       "nitro:/soundbank.bin").
/// @param max_sources Max spatial sources. If < 1, uses
///                    NEA_DEFAULT_SOUND_SOURCES.
/// @return 0 on success, -1 on failure.
int NEA_SoundSystemResetFAT(const char *soundbank_path, int max_sources);

/// Initialize the sound system pool only (no Maxmod initialization).
///
/// Use this when you initialize Maxmod manually (e.g. mmInit() for WAV
/// streaming). Only allocates the source pool.
///
/// @param max_sources Max spatial sources. If < 1, uses
///                    NEA_DEFAULT_SOUND_SOURCES.
/// @return 0 on success, -1 on failure.
int NEA_SoundSystemResetPool(int max_sources);

/// Shut down the sound system and free all sources.
void NEA_SoundSystemEnd(void);

// =========================================================================
// DSi extended audio output (SNDEXCNT)
// =========================================================================

/// I2S output rate of the DSi sound extension.
///
/// The values are the frequencies in KHz that libnds expects, so they can be
/// passed straight through to soundExtSetFrequency().
typedef enum {
    NEA_DSI_SOUND_FREQ_32KHZ = 32, ///< 32.73 kHz. DS-compatible, the default.
    NEA_DSI_SOUND_FREQ_47KHZ = 47, ///< 47.61 kHz. DSi extended output rate.
} NEA_DSiSoundFreq;

/// Returns true if the DSi extended audio output can actually be controlled.
///
/// True only on a DSi running a ROM that carries the TWL codec application
/// flag, which a stock BlocksDS build does. A DS returns false, and so does a
/// DS-only ROM on a DSi. When this returns false every other function in this
/// section is a no-op: the setters return -1 and record nothing, so the getters
/// keep reporting the defaults rather than a rate the hardware isn't running.
///
/// The answer is fixed before main() runs and cannot change, so there is no
/// reason to call this more than once.
///
/// @return True if requests will reach the hardware.
bool NEA_SoundDSiOutputAvailable(void);

/// Select the I2S output rate of the DSi sound extension.
///
/// The rate is remembered and re-applied every time the sound system is
/// initialized, because libnds resets it to 32.73 kHz as part of enabling
/// sound. That makes it safe to call this before NEA_SoundSystemReset(), which
/// is the recommended order: see @ref sndexcnt_rate for why changing the rate
/// with audio running is a bad idea.
///
/// @param freq Requested output rate.
/// @return 0 on success, -1 if unavailable or the rate isn't a valid value.
int NEA_SoundEnableDSiOutput(NEA_DSiSoundFreq freq);

/// Return the I2S output rate to the DS-compatible 32.73 kHz.
///
/// Restores both fields to what libnds establishes at startup: 32.73 kHz and a
/// mix ratio of 8. It does not switch the extension off -- the SNDEXCNT enable
/// bit is not reachable from the ARM9 -- it returns the output stage to what a
/// DS does.
void NEA_SoundDisableDSiOutput(void);

/// Set how the DSP and the ARM sound hardware are mixed into the speakers.
///
/// Maxmod is the ARM side, so this doubles as a master volume for it: see
/// @ref sndexcnt_ratio. Unlike the output rate, the ratio can be changed at any
/// time, including mid-playback.
///
/// @param ratio 0 to 8, where 0 is 100% DSP, 8 is 100% ARM (the default) and 4
///              is an even split. Values outside the range are clamped.
/// @return 0 on success, -1 if unavailable.
int NEA_SoundSetDSiMixRatio(int ratio);

/// Get the requested I2S output rate.
///
/// This is NEA's own record of what it applied, not a read of SNDEXCNT, which
/// the ARM9 cannot read. Requests that were refused are not recorded, so where
/// the extension is unavailable this stays at the default.
///
/// @return The rate in effect, defaulting to NEA_DSI_SOUND_FREQ_32KHZ.
NEA_DSiSoundFreq NEA_SoundGetDSiOutputFreq(void);

/// Get the requested DSP/ARM mix ratio.
///
/// As with NEA_SoundGetDSiOutputFreq(), this is NEA's record rather than a
/// register read.
///
/// @return The ratio in effect (0-8), defaulting to 8.
int NEA_SoundGetDSiMixRatio(void);

// =========================================================================
// Listener
// =========================================================================

/// Set the camera used as the audio listener.
///
/// @param cam Pointer to the camera.
void NEA_SoundSetListener(NEA_Camera *cam);

// =========================================================================
// Sound source management
// =========================================================================

/// Create a new spatial sound source.
///
/// The effect must already be loaded via mmLoadEffect(). Default distance
/// range is 1.0 (full volume) to 20.0 (silence).
///
/// @param sample_id Maxmod sample ID (SFX_xxx).
/// @return Pointer to the new source, or NULL if pool is full.
NEA_SoundSource *NEA_SoundSourceCreate(mm_word sample_id);

/// Delete a spatial sound source. Stops playback if active.
///
/// @param source Pointer to the source.
void NEA_SoundSourceDelete(NEA_SoundSource *source);

/// Delete all spatial sound sources.
void NEA_SoundSourceDeleteAll(void);

// =========================================================================
// Sound source configuration
// =========================================================================

/// Attach a source to a model (position follows model each update).
///
/// @param source Pointer to the source.
/// @param model Pointer to the model. Pass NULL to detach.
void NEA_SoundSourceSetModel(NEA_SoundSource *source, NEA_Model *model);

/// Set a fixed world position for a source (f32).
///
/// Only used if no model is attached.
///
/// @param source Pointer to the source.
/// @param x (x, y, z) Position (f32).
/// @param y (x, y, z) Position (f32).
/// @param z (x, y, z) Position (f32).
void NEA_SoundSourceSetPositionI(NEA_SoundSource *source,
                                  int32_t x, int32_t y, int32_t z);

/// Set a fixed world position for a source (float).
///
/// @param s Pointer to the source.
/// @param x (x, y, z) Position (float).
/// @param y (x, y, z) Position (float).
/// @param z (x, y, z) Position (float).
#define NEA_SoundSourceSetPosition(s, x, y, z) \
    NEA_SoundSourceSetPositionI(s, floattof32(x), floattof32(y), floattof32(z))

/// Set the distance range for volume attenuation (f32).
///
/// Volume is 100% at min_dist, 0% at max_dist, linearly interpolated.
///
/// @param source Pointer to the source.
/// @param min_dist Minimum distance for full volume (f32).
/// @param max_dist Maximum distance for silence (f32).
void NEA_SoundSourceSetDistanceI(NEA_SoundSource *source,
                                  int32_t min_dist, int32_t max_dist);

/// Set the distance range for volume attenuation (float).
///
/// @param s Pointer to the source.
/// @param mind Minimum distance for full volume (float).
/// @param maxd Maximum distance for silence (float).
#define NEA_SoundSourceSetDistance(s, mind, maxd) \
    NEA_SoundSourceSetDistanceI(s, floattof32(mind), floattof32(maxd))

/// Set the base volume before spatial attenuation.
///
/// @param source Pointer to the source.
/// @param volume Volume (0-255).
void NEA_SoundSourceSetVolume(NEA_SoundSource *source, int volume);

/// Set the base playback rate.
///
/// @param source Pointer to the source.
/// @param rate Playback rate (6.10 fixed point, 1024 = normal).
void NEA_SoundSourceSetRate(NEA_SoundSource *source, mm_hword rate);

/// Set whether this source loops.
///
/// @param source Pointer to the source.
/// @param loop True to loop, false for one-shot.
void NEA_SoundSourceSetLoop(NEA_SoundSource *source, bool loop);

/// Set the delay between loop re-triggers (in frames).
///
/// Only used when looping is enabled. Default is 60 frames (1 second at
/// 60 fps). Set to a value matching the length of the sound effect.
///
/// @param source Pointer to the source.
/// @param frames Number of frames between re-triggers.
void NEA_SoundSourceSetLoopDelay(NEA_SoundSource *source, uint16_t frames);

// =========================================================================
// Sound source playback
// =========================================================================

/// Start (or restart) playback of a spatial source.
///
/// @param source Pointer to the source.
void NEA_SoundSourcePlay(NEA_SoundSource *source);

/// Stop playback of a spatial source.
///
/// @param source Pointer to the source.
void NEA_SoundSourceStop(NEA_SoundSource *source);

/// Returns true if a source is currently playing.
///
/// @param source Pointer to the source.
/// @return True if playing.
bool NEA_SoundSourceIsPlaying(const NEA_SoundSource *source);

// =========================================================================
// Spatial update
// =========================================================================

/// Update all spatial sound sources (recompute volume and panning).
///
/// Called automatically by NEA_WaitForVBL(NEA_UPDATE_SOUND), or call
/// manually if you need more control over timing.
void NEA_SoundUpdateAll(void);

// =========================================================================
// Music (non-spatial wrappers)
// =========================================================================

/// Load and start music playback.
///
/// @param module_id Module ID (MOD_xxx).
/// @param mode MM_PLAY_LOOP or MM_PLAY_ONCE.
void NEA_MusicStart(mm_word module_id, mm_pmode mode);

/// Stop music playback.
void NEA_MusicStop(void);

/// Pause music.
void NEA_MusicPause(void);

/// Resume music.
void NEA_MusicResume(void);

/// Set music volume.
///
/// @param volume Volume (0-1024, 1024 = normal).
void NEA_MusicSetVolume(mm_word volume);

/// Set music tempo.
///
/// @param tempo Tempo (512-2048, 1024 = normal).
void NEA_MusicSetTempo(mm_word tempo);

/// Set music pitch.
///
/// @param pitch Pitch (512-2048, 1024 = normal).
void NEA_MusicSetPitch(mm_word pitch);

/// Returns true if music is playing.
///
/// @return True if playing.
bool NEA_MusicIsPlaying(void);

// =========================================================================
// Non-spatial SFX convenience
// =========================================================================

/// Load a sound effect into memory. Must be called before playing.
///
/// @param sample_id Sample ID (SFX_xxx).
void NEA_SfxLoad(mm_word sample_id);

/// Unload a sound effect from memory.
///
/// @param sample_id Sample ID (SFX_xxx).
void NEA_SfxUnload(mm_word sample_id);

/// Play a non-spatial sound effect at center panning and full volume.
///
/// @param sample_id Sample ID (SFX_xxx).
/// @return Maxmod handle for further control.
mm_sfxhand NEA_SfxPlay(mm_word sample_id);

/// Play a non-spatial sound effect with custom parameters.
///
/// @param sample_id Sample ID (SFX_xxx).
/// @param volume Volume (0-255).
/// @param panning Panning (0-255, 128 = center).
/// @param rate Playback rate (6.10 fixed, 1024 = normal).
/// @return Maxmod handle for further control.
mm_sfxhand NEA_SfxPlayEx(mm_word sample_id, mm_byte volume,
                          mm_byte panning, mm_hword rate);

/// Set the playback rate of a playing sound effect.
///
/// @param handle Handle from NEA_SfxPlay() or NEA_SfxPlayEx().
/// @param rate Playback rate (6.10 fixed, 1024 = normal).
void NEA_SfxSetRate(mm_sfxhand handle, mm_hword rate);

/// Set the volume of a playing sound effect.
///
/// @param handle Handle from NEA_SfxPlay() or NEA_SfxPlayEx().
/// @param volume Volume (0-255).
void NEA_SfxSetVolume(mm_sfxhand handle, mm_word volume);

/// Set the panning of a playing sound effect.
///
/// @param handle Handle from NEA_SfxPlay() or NEA_SfxPlayEx().
/// @param panning Panning (0-255, 128 = center).
void NEA_SfxSetPanning(mm_sfxhand handle, mm_byte panning);

/// Release a sound effect handle for channel reuse by maxmod.
///
/// @param handle Handle from NEA_SfxPlay() or NEA_SfxPlayEx().
void NEA_SfxRelease(mm_sfxhand handle);

/// Stop a non-spatial sound effect.
///
/// @param handle Handle from NEA_SfxPlay() or NEA_SfxPlayEx().
void NEA_SfxStop(mm_sfxhand handle);

/// Stop all currently playing sound effects.
void NEA_SfxStopAll(void);

// =========================================================================
// WAV streaming
// =========================================================================

/// Open an audio stream.
///
/// @param sampling_rate Sampling rate in Hz (1024-32768).
/// @param buffer_length Buffer size in samples (multiple of 16).
/// @param callback Function called to fill the buffer.
/// @param format Stream format (MM_STREAM_8BIT_MONO, etc).
/// @param timer Hardware timer to use (MM_TIMER0, etc).
void NEA_StreamOpen(mm_word sampling_rate, mm_word buffer_length,
                    mm_stream_func callback, mm_word format,
                    mm_word timer);

/// Close audio stream.
void NEA_StreamClose(void);

/// Get stream position in elapsed samples.
///
/// @return Elapsed samples.
mm_word NEA_StreamGetPosition(void);

/// @}

#endif // NEA_MAXMOD

#endif // NEA_SOUND_H__
