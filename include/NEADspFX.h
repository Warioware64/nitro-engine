// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Nitro Engine Advanced contributors
//
// This file is part of Nitro Engine Advanced

#ifndef NEA_DSPFX_H__
#define NEA_DSPFX_H__

#include <nds.h>

#include "dsp/nea_fx_bloom.h"
#include "dsp/nea_fx_blur.h"
#include "dsp/nea_fx_displace.h"
#include "dsp/nea_fx_grade.h"

/// @file NEADspFX.h
/// @brief Image effects run on the DSi's DSP, with identical ARM9 fallbacks.

/// @defgroup dspfx DSP image effects
///
/// Effects applied to whole 16-bit images, such as a captured frame between
/// two render passes. On a DSi with NEA_DspInit() done, the DSP moves the
/// pixels itself and the ARM9 is free while it works. Everywhere else the same
/// effect runs on the ARM9 and gives the same pixels, bit for bit.
///
/// @section dspfx_grade Colour grading
///
/// A grade (NEA_DspGrade) is a 3x3 colour matrix with offsets, followed by a
/// tone curve per channel. Build it with the NEA_DspGrade*() helpers, which
/// compose, turn it into tables with NEA_DspGradeBuild(), then apply the tables
/// to images:
///
///     NEA_DspGrade g;
///     NEA_DspGradeInit(&g);
///     NEA_DspGradeSaturate(&g, 64);   // Mostly grey
///     NEA_DspGradeSepia(&g, 256);     // Then fully sepia
///     NEA_DspGradeTables *t = NEA_DspGradeTablesCreate();
///     NEA_DspGradeBuild(t, &g);
///     NEA_DspGradeApply(t, src, dst, 256 * 192);
///
/// Building costs about a millisecond of ARM9 time, so an animated effect
/// (a gradual desaturation) rebuilds once per frame at most. The tables are
/// sent to the DSP only when they change.
///
/// @{

/// A colour grade. All fields can be edited directly as well.
typedef struct {
    s16 matrix[3][3];   ///< Q8 (256 = 1.0). Row = output channel (R, G, B)
    s16 offset[3];      ///< Q8, in 5-bit steps (256 = one step of 0..31)
    u8 curve[3][32];    ///< Output (0..31) for each input (0..31), per channel
} NEA_DspGrade;

/// Tables built from a grade. Allocate with NEA_DspGradeTablesCreate(), or
/// statically, but never as a local variable: the DSP reads them with DMA and
/// cannot see DTCM, where locals live.
typedef struct {
    u16 blob[NEA_FX_GRADE_WORDS]; ///< Layout in dsp/nea_fx_grade.h
    u16 generation;               ///< Changes on every build, never 0
} __attribute__((aligned(64))) NEA_DspGradeTables;

/// Sets a grade to identity: no change to any pixel.
void NEA_DspGradeInit(NEA_DspGrade *g);

/// Scales saturation. 0 = greyscale, 256 = unchanged, above 256 = more vivid.
void NEA_DspGradeSaturate(NEA_DspGrade *g, int saturation);

/// Blends towards sepia. 0 = unchanged, 256 = full sepia.
void NEA_DspGradeSepia(NEA_DspGrade *g, int amount);

/// Rotates hues, keeping luminance.
///
/// @param angle libnds angle (see degreesToAngle()).
void NEA_DspGradeHueRotate(NEA_DspGrade *g, int angle);

/// Multiplies each channel (Q8) and adds an offset (Q8, 5-bit steps).
void NEA_DspGradeTint(NEA_DspGrade *g, int r, int gr, int b, int offset);

/// Inverts the image (a photographic negative).
void NEA_DspGradeInvert(NEA_DspGrade *g);

/// Applies contrast and brightness to the curves of all three channels.
///
/// @param contrast Q8, 256 = unchanged, around mid grey (16).
/// @param brightness Added after contrast, in 5-bit steps.
void NEA_DspGradeContrast(NEA_DspGrade *g, int contrast, int brightness);

/// Allocates tables. Returns NULL if out of memory.
NEA_DspGradeTables *NEA_DspGradeTablesCreate(void);

/// Frees tables from NEA_DspGradeTablesCreate().
void NEA_DspGradeTablesDelete(NEA_DspGradeTables *t);

/// Builds the tables for a grade.
///
/// The matrix output must stay between -1 and +3 (in units of full scale) for
/// every input colour, which any sensible grade does. Otherwise the tables are
/// left unchanged and this returns -1.
///
/// @return 0 on success, -1 if the grade is out of range.
int NEA_DspGradeBuild(NEA_DspGradeTables *t, const NEA_DspGrade *g);

/// Starts grading an image and returns without waiting, if the DSP takes it.
///
/// src and dst may be the same buffer. Both may be in main RAM or in VRAM
/// mapped to the ARM9 (LCD or BG). Until NEA_DspGradeEnd(), don't touch dst,
/// and in main RAM don't write anything that shares a 32-byte cache line with
/// it either (its first and last bytes, if dst isn't 32-byte aligned): the
/// DSP writes behind the ARM9's data cache.
///
/// @return 1 if the DSP is working on it (collect it with NEA_DspGradeEnd()),
///         2 if it has already been done (on the ARM9, or through the FIFO),
///         0 on failure (DSP busy with another job).
int NEA_DspGradeBegin(const NEA_DspGradeTables *t, const u16 *src, u16 *dst,
                      int pixels);

/// Same as NEA_DspGradeBegin(), for a rectangle inside a larger image.
///
/// @param src First pixel of the rectangle.
/// @param src_stride Distance between rows of src, in pixels.
/// @param dst First pixel of the destination rectangle.
/// @param dst_stride Distance between rows of dst, in pixels.
/// @param width Pixels per row.
/// @param height Rows. width * height must be at most 65535.
int NEA_DspGradeBeginRect(const NEA_DspGradeTables *t, const u16 *src,
                          int src_stride, u16 *dst, int dst_stride, int width,
                          int height);

/// Waits for the grading started by NEA_DspGradeBegin().
///
/// If the DSP job fails, the image is graded on the ARM9 instead and this
/// returns 2, so dst is always right. The exception is grading in place
/// (src == dst): the DSP may have graded part of the image already, so it is
/// left as it is and this returns 0.
///
/// @return 1 if a DSP job was collected, 2 if there was none or the ARM9 did
///         the work, 0 on failure.
int NEA_DspGradeEnd(void);

/// Grades an image and waits for the result (Begin + End).
int NEA_DspGradeApply(const NEA_DspGradeTables *t, const u16 *src, u16 *dst,
                      int pixels);

/// Grades an image on the ARM9, whether or not a DSP is available.
void NEA_DspGradeApplyCPU(const NEA_DspGradeTables *t, const u16 *src,
                          u16 *dst, int pixels);

/// Whether the DSP overlaps its transfers with the grading (default: true).
///
/// With overlap, the DSP reads the next pixels and writes the graded ones
/// while it grades, so a job costs about the grading alone. Turning it off
/// exists for measuring the difference.
void NEA_DspGradeSetOverlap(bool overlap);

/// Status of the last DSP grading job (NEA_DSP_STATUS_*, -1 for no answer).
/// Diagnostic.
///
/// For a DMA error (NEA_DSP_STATUS_DMA_ERROR), also where the DSP's transfer
/// stuck. detail: bits 0-15 position in the image, 16-17 direction (1 read,
/// 2 write), 18 AHBM error, 19 AHBM busy, 20 DMA channel done, 21-30 length in
/// words. addr: ARM9 address of that transfer.
int NEA_DspGradeGetLastStatus(u32 *detail, u32 *addr);

/// Number of DSP grading jobs that failed since the program started.
u32 NEA_DspGradeGetFailureCount(void);

/// DSP cycles of the last DSP grading: the kernel alone, and the whole job
/// (transfers included). See NEA_DspCyclesToUs().
void NEA_DspGradeGetStats(u32 *kernel_cycles, u32 *job_cycles);

/// @}

/// @defgroup dspfx_displace DSP displacement
///
/// Moves each pixel of an image by an offset that varies smoothly across it:
/// heat haze, underwater wobble, ripples on a reflection. Each output pixel
/// reads the source at (x + dx, y + dy), clamped to the image, where
///
///     dx = ax[x] + ay[y]      dy = bx[x] + by[y]
///
/// A per-column and a per-row term per offset are enough for crossed waves,
/// and they cost no multiplication per pixel. Offsets are whole pixels, at
/// most NEA_FX_DISP_XMAX sideways and NEA_FX_DISP_YMAX vertically. Images are
/// at most 256x192, and the result can't be written over the source.
///
///     NEA_DspDisplace d;
///     NEA_DspDisplaceInit(&d, 256, 192);
///     NEA_DspDisplaceHeatHaze(&d, frame, 3);
///     NEA_DspDisplaceBuild(tables, &d);          // Every frame, ~0.1 ms
///     NEA_DspDisplaceApply(tables, src, 256, dst, 256);
///
/// Without a DSP the same displacement runs on the ARM9, pixel for pixel.
///
/// @{

/// Which offset a wave adds to, and along which axis it varies.
typedef enum {
    NEA_DISPLACE_DX_ALONG_X, ///< Sideways offset, varying across columns
    NEA_DISPLACE_DX_ALONG_Y, ///< Sideways offset, varying down rows (sway)
    NEA_DISPLACE_DY_ALONG_X, ///< Vertical offset, varying across columns
    NEA_DISPLACE_DY_ALONG_Y  ///< Vertical offset, varying down rows
} NEA_DspDisplaceTerm;

/// A displacement field. Its arrays can also be written directly.
typedef struct {
    s16 ax[NEA_FX_DISP_MAX_W];  ///< dx term per column
    s16 ay[NEA_FX_DISP_MAX_H];  ///< dx term per row
    s16 bx[NEA_FX_DISP_MAX_W];  ///< dy term per column
    s16 by[NEA_FX_DISP_MAX_H];  ///< dy term per row
    u16 width, height;
} NEA_DspDisplace;

/// Tables built from a field. Allocate with NEA_DspDisplaceTablesCreate() or
/// statically, never as a local variable (the DSP reads them with DMA).
typedef struct {
    u16 blob[NEA_FX_DISP_WORDS]; ///< Layout in dsp/nea_fx_displace.h
    u16 width, height;           ///< Output size
    u16 src_width;               ///< Source row length
} __attribute__((aligned(64))) NEA_DspDisplaceTables;

/// Clears a field for an image of the given size (at most 256x192).
void NEA_DspDisplaceInit(NEA_DspDisplace *d, int width, int height);

/// Adds a sine wave to one term of a field.
///
/// @param amplitude Peak offset, in pixels.
/// @param wavelength Pixels per period.
/// @param phase libnds angle (see degreesToAngle()); animate it for motion.
void NEA_DspDisplaceAddWave(NEA_DspDisplace *d, NEA_DspDisplaceTerm term,
                            int amplitude, int wavelength, int phase);

/// Adds a heat-haze shimmer. Call on a cleared field every frame with an
/// increasing time. strength: peak sideways sway in pixels (2-4 is subtle).
void NEA_DspDisplaceHeatHaze(NEA_DspDisplace *d, int time, int strength);

/// Adds a water ripple, as for a reflection. Same parameters as above.
void NEA_DspDisplaceRipple(NEA_DspDisplace *d, int time, int strength);

/// Allocates tables. Returns NULL if out of memory.
NEA_DspDisplaceTables *NEA_DspDisplaceTablesCreate(void);

/// Frees tables from NEA_DspDisplaceTablesCreate().
void NEA_DspDisplaceTablesDelete(NEA_DspDisplaceTables *t);

/// Builds the tables for a field. Safe while a DSP job uses them: it waits
/// only until the DSP has its own copy.
///
/// @return 0 on success, -1 if an offset could go out of range: for each
///         offset, the largest column term plus the largest row term must stay
///         within NEA_FX_DISP_XMAX (dx) or NEA_FX_DISP_YMAX (dy).
int NEA_DspDisplaceBuild(NEA_DspDisplaceTables *t, const NEA_DspDisplace *d);

/// Builds tables that also scale a wider source down to the field's width:
/// output column x reads source column x * src_width / width (plus the
/// displacement). Rows are not scaled. NEA_DspDisplaceBuild() is the case
/// src_width == width.
///
/// @return 0 on success, -1 if an offset could go out of range.
int NEA_DspDisplaceBuildScaled(NEA_DspDisplaceTables *t,
                               const NEA_DspDisplace *d, int src_width);

/// Starts displacing an image and returns without waiting, if the DSP takes
/// it. The image size is the one the tables were built for.
///
/// src_stride may be negative: the source rows are then read upwards, from src
/// as the first row, which mirrors the image vertically. src and dst may be in
/// main RAM or in VRAM mapped to the ARM9, and must not overlap. Until NEA_DspDisplaceEnd(), don't touch dst or anything sharing a
/// 32-byte cache line with it.
///
/// @return 1 if the DSP is working on it, 2 if it has already been done (on
///         the ARM9, or through the FIFO), 0 if the DSP is busy with another
///         job.
int NEA_DspDisplaceBegin(const NEA_DspDisplaceTables *t, const u16 *src,
                         int src_stride, u16 *dst, int dst_stride);

/// Waits for the displacement started by NEA_DspDisplaceBegin(). If the DSP
/// job fails, the image is displaced on the ARM9 instead (and the DSP is
/// restarted), so dst is always right.
///
/// @return 1 if a DSP job was collected, 2 if there was none or the ARM9 did
///         the work.
int NEA_DspDisplaceEnd(void);

/// Displaces an image and waits for the result. Uses the ARM9 if the DSP is
/// busy with another job.
int NEA_DspDisplaceApply(const NEA_DspDisplaceTables *t, const u16 *src,
                         int src_stride, u16 *dst, int dst_stride);

/// Displaces an image on the ARM9, whether or not a DSP is available.
void NEA_DspDisplaceApplyCPU(const NEA_DspDisplaceTables *t, const u16 *src,
                             int src_stride, u16 *dst, int dst_stride);

/// DSP cycles of the last DSP displacement: kernel alone, and whole job.
void NEA_DspDisplaceGetStats(u32 *kernel_cycles, u32 *job_cycles);

/// Number of DSP displacement jobs that failed (and were done on the ARM9).
u32 NEA_DspDisplaceGetFailureCount(void);

/// @}

/// @defgroup dspfx_blur DSP blur
///
/// A soft blur: a separable [1 2 1] / 4 filter across and down, edges clamped,
/// on 16-bit images up to 256x192. Blurring twice gives the classic
/// [1 4 6 4 1] / 16 Gaussian; for wider blurs, blur a half-size copy (which is
/// what bloom does). The result can overwrite the source. Without a DSP the
/// same blur runs on the ARM9, pixel for pixel.
///
/// @{

/// Starts blurring a rectangle and returns without waiting, if the DSP takes
/// it. Rows are src_stride / dst_stride pixels apart; dst may be src. Until
/// NEA_DspBlurEnd(), don't touch dst or anything sharing a 32-byte cache line
/// with it.
///
/// @return 1 if the DSP is working on it, 2 if it has already been done (on
///         the ARM9, or through the FIFO), 0 if the DSP is busy with another
///         job.
int NEA_DspBlurBegin(const u16 *src, int src_stride, u16 *dst, int dst_stride,
                     int width, int height);

/// Waits for the blur started by NEA_DspBlurBegin(). If the DSP job fails, the
/// image is blurred on the ARM9 instead (and the DSP restarted), unless it was
/// in place: then part of it may be blurred already, and this returns 0.
///
/// @return 1 if a DSP job was collected, 2 if there was none or the ARM9 did
///         the work, 0 on failure.
int NEA_DspBlurEnd(void);

/// Blurs a rectangle and waits for the result. Uses the ARM9 if the DSP is
/// busy with another job.
int NEA_DspBlurApply(const u16 *src, int src_stride, u16 *dst, int dst_stride,
                     int width, int height);

/// Blurs a rectangle on the ARM9, whether or not a DSP is available.
void NEA_DspBlurApplyCPU(const u16 *src, int src_stride, u16 *dst,
                         int dst_stride, int width, int height);

/// DSP cycles of the last DSP blur: kernels alone, and whole job.
void NEA_DspBlurGetStats(u32 *kernel_cycles, u32 *job_cycles);

/// Number of DSP blur jobs that failed.
u32 NEA_DspBlurGetFailureCount(void);

/// @}

/// @defgroup dspfx_bloom DSP bloom
///
/// Makes bright areas glow into their surroundings. For each 2x2 block, the
/// brightness above a threshold (per channel) is kept; that half-size image is
/// blurred with [1 4 6 4 1] / 16 across and down, and added back to the image,
/// each channel saturating at white. Images up to 256x192, with an even width
/// and height; the result can overwrite the source. Without a DSP the ARM9
/// gives the same pixels.
///
/// @{

/// Starts adding bloom to a rectangle and returns without waiting, if the DSP
/// takes it. Rows are src_stride / dst_stride pixels apart; dst may be src.
/// Until NEA_DspBloomEnd(), don't touch dst or anything sharing a 32-byte cache
/// line with it.
///
/// @param threshold 0-31: only brightness above it glows (per channel, in
///        5-bit steps). Around 20 keeps it to highlights.
/// @return 1 if the DSP is working on it, 2 if it has already been done (on
///         the ARM9, or through the FIFO), 0 if the DSP is busy with another
///         job.
int NEA_DspBloomBegin(const u16 *src, int src_stride, u16 *dst, int dst_stride,
                      int width, int height, int threshold);

/// Waits for the bloom started by NEA_DspBloomBegin(). If the DSP job fails,
/// the image is processed on the ARM9 instead (and the DSP restarted), unless
/// it was in place: then part of it may be done already, and this returns 0.
///
/// @return 1 if a DSP job was collected, 2 if there was none or the ARM9 did
///         the work, 0 on failure.
int NEA_DspBloomEnd(void);

/// Adds bloom and waits for the result. Uses the ARM9 if the DSP is busy with
/// another job.
int NEA_DspBloomApply(const u16 *src, int src_stride, u16 *dst, int dst_stride,
                      int width, int height, int threshold);

/// Adds bloom on the ARM9, whether or not a DSP is available.
void NEA_DspBloomApplyCPU(const u16 *src, int src_stride, u16 *dst,
                          int dst_stride, int width, int height,
                          int threshold);

/// DSP cycles of the last DSP bloom: kernels alone, and whole job.
void NEA_DspBloomGetStats(u32 *kernel_cycles, u32 *job_cycles);

/// Number of DSP bloom jobs that failed.
u32 NEA_DspBloomGetFailureCount(void);

/// @}

/// @defgroup dspfx_screen DSP render target and full-screen effects
///
/// Both capture the 3D image with the display capture, so they need single 3D
/// mode, can't run together, and can't run with NEAPostFX's capture effects.
/// Each reserves VRAM banks: initialize it before NEA_TextureSystemReset(),
/// which then leaves those banks out of the texture allocator. Both are driven
/// by NEA_WaitForVBL(NEA_UPDATE_DSPFX).
///
/// **Reflection** (screen-space mirror): every frame, the top 128 lines of the
/// 3D image are captured into one bank (two halves, alternately). The DSP then
/// mirrors the lines above a water line, scales them to 128 pixels wide,
/// ripples them (a displacement field) and optionally tints them (a grade),
/// into a 128-pixel-wide texture the next frame can draw on the water. It
/// reflects only what is on screen, one frame late, which is the classic trick.
///
/// **Full-screen effects**: the whole 3D image is captured into one of two
/// banks, blurred, bloomed and graded in place by the DSP, then shown as a
/// 16-bit bitmap background in front of the live 3D layer. An image takes two
/// frames when the effects fit in one (30 fps), more otherwise.
///
/// @{

/// Starts the reflection. Call it before NEA_TextureSystemReset().
///
/// @param bank One bank of A-D, reserved for the captures.
/// @param water_line Screen line of the water surface (1-128).
/// @param rows Lines above it to reflect (1 - water_line, at most 128).
/// @return 1 on success, 0 on failure.
int NEA_DspReflectInit(NEA_VRAMBankFlags bank, int water_line, int rows);

/// Sets the material that receives the reflection (NULL for none).
///
/// Make it with NEA_MaterialTexBlank(mat, NEA_A1RGB5, 128, 128, flags) after
/// NEA_TextureSystemReset(). Row 0 of its texture is the screen line just
/// above the water line, and later rows go up: drawn from the water line down,
/// with NEA_2DDrawTexturedQuadColorCanvas() for example, it is a mirror.
///
/// @return 1 on success, 0 if the material isn't RAM-backed.
int NEA_DspReflectSetTarget(NEA_Material *mat);

/// Stops the reflection and releases its bank.
void NEA_DspReflectEnd(void);

/// Sets the ripple: a field of 128 x rows (NEA_DspDisplaceInit(d, 128, rows)),
/// for example NEA_DspDisplaceRipple(). Call it every frame to animate.
///
/// @return 0 on success, -1 if the field is out of range.
int NEA_DspReflectSetRipple(const NEA_DspDisplace *d);

/// Sets a tint for the reflection (a colour grade), or NULL for none. A tinted
/// reflection is opaque; without a tint, what was transparent in the 3D image
/// (a clear colour with alpha 0) stays transparent.
///
/// @return 0 on success, -1 on failure.
int NEA_DspReflectSetTint(const NEA_DspGrade *g);

/// Builds the reflection from the last frame. Call it once per frame after
/// the scene is submitted (after NEA_Process()); the texture is uploaded at the
/// next NEA_WaitForVBL(NEA_UPDATE_DSPFX).
///
/// @return 1 if a new image was made, 0 if there was no capture yet.
int NEA_DspReflectProcess(void);

/// DSP cycles the last NEA_DspReflectProcess() took (0 without a DSP).
u32 NEA_DspReflectGetStats(void);

/// Starts the full-screen effects.
///
/// @param banks Exactly two banks of A-D, reserved.
/// @return 1 on success, 0 on failure.
int NEA_DspScreenFXInit(NEA_VRAMBankFlags banks);

/// Stops the full-screen effects and releases their banks.
void NEA_DspScreenFXEnd(void);

/// Grades the image, as the last effect. NULL for none. The tables must stay
/// valid while set.
void NEA_DspScreenFXSetGrade(const NEA_DspGradeTables *t);

/// Blurs the image this many times, as the first effect (0 for none).
void NEA_DspScreenFXSetBlur(int passes);

/// Adds bloom with this threshold (0-31), after the blur. -1 for none.
void NEA_DspScreenFXSetBloom(int threshold);

/// Moves the effects on without waiting. NEA_WaitForVBL(NEA_UPDATE_DSPFX)
/// does it every frame; calling it after the scene as well starts the next
/// effect sooner.
void NEA_DspScreenFXPoll(void);

/// True in a frame whose image will be captured. The others are never shown:
/// a game can skip drawing them.
bool NEA_DspScreenFXFrameCaptured(void);

/// Rate of processed images, in frames per second (30 when the effects fit in
/// one frame).
int NEA_DspScreenFXGetRate(void);

/// Advances the reflection and the full-screen effects. Called by
/// NEA_WaitForVBL(NEA_UPDATE_DSPFX).
void NEA_DspFXUpdate(void);

/// Banks reserved by the reflection and the full-screen effects.
NEA_VRAMBankFlags NEA_DspFXGetReservedBanks(void);

/// @}

#endif // NEA_DSPFX_H__
