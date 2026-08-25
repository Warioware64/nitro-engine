// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Nitro Engine Advanced contributors
//
// This file is part of Nitro Engine Advanced

#ifndef NEA_POSTFX_H__
#define NEA_POSTFX_H__

#include <nds.h>

#include "NEATexture.h" // NEA_VRAMBankFlags

/// @file NEAPostFX.h
/// @brief PPU-native post-process and compositing effects.

/// @defgroup postfx Post-process effects (2D PPU)
///
/// Screen effects built entirely out of 2D PPU register writes: colored glow,
/// full-screen flashes and fades. No per-pixel CPU work, no DSP, and (for the
/// flash/fade path) no VRAM at all.
///
/// @section postfx_blend_unit One blend unit, one owner
///
/// The DS has exactly **one** color special effect unit per 2D engine. BLDCNT
/// bits 6-7 select a single mode -- alpha blending, brightness increase or
/// brightness decrease -- for the whole screen. That means the effects in this
/// module which use it are **mutually exclusive**, and so are any effects added
/// later that need it.
///
/// This module arbitrates rather than letting them silently corrupt each other:
/// enabling one blend-unit effect takes ownership, and enabling another one
/// releases the first (with a debug message in debug builds). Use
/// NEA_PostFXBlendOwner() to find out who currently holds it.
///
/// Effects that do *not* touch the blend unit -- mosaic, and the window
/// registers when used purely to mask layers -- compose freely with whichever
/// one holds it.
///
/// @section postfx_3d_layer The 3D layer is not an ordinary BG
///
/// The 3D image reaches the 2D engine as BG0, but it does not behave like a
/// normal background (GBATEK, "DS 3D Final 2D Output"):
///
/// - Mosaic has no effect on it.
/// - It cannot be rotated, scaled or scrolled vertically.
/// - Blending with BG0 as the **1st** target uses the per-pixel 3D alpha and
///   ignores EVA/EVB entirely.
///
/// The last point is why the glow layer here sits *above* the 3D layer and is
/// the 1st target, with the 3D image as the 2nd target. Arranged the other way
/// around, the EVA/EVB coefficients would be ignored and the intensity control
/// would do nothing.
///
/// @{

/// Which effect currently owns the 2D blend unit (BLDCNT).
typedef enum {
    NEA_POSTFX_BLEND_NONE = 0, ///< Nobody. BLDCNT is disabled.
    NEA_POSTFX_BLEND_GLOW,     ///< NEA_PostFXGlowEnable()
    NEA_POSTFX_BLEND_FLASH,    ///< NEA_PostFXFlashSet()
    NEA_POSTFX_BLEND_VIGNETTE, ///< NEA_PostFXVignetteEnable()
    NEA_POSTFX_BLEND_CAPTURE   ///< NEA_PostFXCaptureEnable()
} NEA_PostFXBlendOwner;

/// Returns which effect currently owns the blend unit.
NEA_PostFXBlendOwner NEA_PostFXBlendOwner_Get(void);

// ---------------------------------------------------------------------------
// Full-screen flash / fade
// ---------------------------------------------------------------------------

/// Full-screen brightness effects applied to the 3D layer.
typedef enum {
    NEA_POSTFX_FLASH_NONE = 0, ///< Disabled
    NEA_POSTFX_FLASH_WHITE,    ///< Brightness increase: I = I + (31-I)*EVY
    NEA_POSTFX_FLASH_BLACK     ///< Brightness decrease: I = I - I*EVY
} NEA_PostFXFlashMode;

/// Applies a full-screen brightness increase or decrease to the 3D layer.
///
/// This is the cheapest effect in the module: two register writes, **no VRAM
/// and no BG layer**. GBATEK confirms brightness up/down works on the 3D layer
/// with BG0 as 1st target "as for 2D", unlike alpha blending.
///
/// Use it for muzzle flashes, lightning, damage flashes and fade-to-black
/// transitions. For a *colored* wash use NEA_PostFXGlow* instead.
///
/// Every main-engine layer is affected (all four BGs, sprites and the
/// backdrop), so a fade actually reaches black rather than leaving the HUD lit.
/// **The sub engine has its own separate blend unit and is not touched**, so a
/// debug console on the bottom screen stays readable through a fade.
///
/// Takes ownership of the blend unit (see @ref postfx_blend_unit). Passing
/// NEA_POSTFX_FLASH_NONE releases it.
///
/// @param mode Brightness direction, or NEA_POSTFX_FLASH_NONE to disable.
/// @param strength Effect strength, 0 (none) to 16 (full white / full black).
void NEA_PostFXFlashSet(NEA_PostFXFlashMode mode, int strength);

// ---------------------------------------------------------------------------
// Colored glow overlay
// ---------------------------------------------------------------------------

/// Sets up a colored overlay layer composited over the 3D scene.
///
/// Creates a 4bpp tiled background on the given main-engine layer and fills it
/// with a 15-band vertical ramp, so the same layer can act as a flat wash or a
/// vertical gradient with no extra cost. The color comes from one 16-entry
/// palette slot, so changing the glow is a handful of palette writes rather
/// than touching any pixels.
///
/// Compositing is `I = min(31, I_glow*EVA + I_3D*EVB)` with EVB pinned to 16/16,
/// so the scene keeps its full brightness and the glow is *added* on top,
/// saturating at white. The DS has no true additive blend mode; this is as
/// close as the hardware gets.
///
/// **Requires NEA_ModeSingle3D.** The dual 3D and two-pass modes rewrite BG0,
/// BG1 and BG2 priorities every frame to composite their captures, which would
/// fight the layer ordering this needs. It also requires NEA_Hw2DInit() or
/// NEA_Hw2DAutoInit() to have claimed a main-engine BG bank.
///
/// Sets the glow layer to priority 0 and the 3D layer (BG0) to priority 1, so
/// the glow sits above the scene as the blend's 1st target.
///
/// Cost: one BG layer, one 16-color palette slot, and **18 KB reserved** in the
/// main BG VRAM bank. The ramp itself is tiny -- 480 bytes of tiles and a 2 KB
/// map -- but NEA_Hw2DBGCreate() allocates tile space one 16 KB block at a
/// time, so 16 KB + 2 KB is what becomes unavailable to other backgrounds.
/// Per frame: nothing, unless you change the color or intensity, which is a
/// few register writes.
///
/// @param bg_layer Main engine BG layer to use (1 - 3; 0 is the 3D layer).
/// @param palette_slot 16-color palette slot to own (0 - 15).
/// @return 1 on success, 0 on error.
int NEA_PostFXGlowInit(int bg_layer, int palette_slot);

/// Tears down the glow layer and frees its VRAM.
void NEA_PostFXGlowEnd(void);

/// Sets the glow to a single flat color.
///
/// @param color Glow color (RGB15).
void NEA_PostFXGlowSetColor(u16 color);

/// Sets the glow to a vertical gradient.
///
/// The ramp is quantised to 15 bands over the height of the screen, which is
/// enough that a glow reads as smooth. Band 0 is at the top.
///
/// @param top_color Color at the top of the screen (RGB15).
/// @param bottom_color Color at the bottom of the screen (RGB15).
void NEA_PostFXGlowSetGradient(u16 top_color, u16 bottom_color);

/// Sets how strongly the glow is added to the scene.
///
/// This is a single register write, so it is cheap enough to drive from a
/// per-frame noise value for firelight flicker.
///
/// @param intensity 0 (invisible) to 16 (full strength).
void NEA_PostFXGlowSetIntensity(int intensity);

/// Shows or hides the glow overlay.
///
/// Enabling takes ownership of the blend unit (see @ref postfx_blend_unit).
///
/// @param enable true to show the glow, false to hide it.
void NEA_PostFXGlowEnable(bool enable);

// ---------------------------------------------------------------------------
// Mosaic
// ---------------------------------------------------------------------------

/// Sets the mosaic (pixelation) block size for the main engine.
///
/// **Mosaic does not work on the 3D layer.** GBATEK, "DS 3D Final 2D Output":
/// "All other bits in BG0CNT have no effect on 3D, namely, mosaic cannot be
/// used on the 3D layer." Only 2D backgrounds and sprites can be pixelated, so
/// this is for HUDs, 2D overlays and full-screen 2D transitions. To pixelate
/// the rendered 3D scene you have to capture it to a bitmap background first.
///
/// A size of 0 means no pixelation. Sizes are in pixels: 3 means each 4x1,
/// 1x4 or 4x4 block takes the color of its top-left pixel.
///
/// Costs nothing: one register write, no VRAM, no blend unit. Composes freely
/// with every other effect in this module.
///
/// @param bg_h Horizontal BG mosaic size (0 - 15).
/// @param bg_v Vertical BG mosaic size (0 - 15).
/// @param obj_h Horizontal sprite mosaic size (0 - 15).
/// @param obj_v Vertical sprite mosaic size (0 - 15).
void NEA_PostFXMosaicSet(int bg_h, int bg_v, int obj_h, int obj_v);

/// Enables or disables mosaic on one main-engine background layer.
///
/// Layer 0 is the 3D layer, where this has no effect (see
/// NEA_PostFXMosaicSet()); asking for it prints a debug message.
///
/// @param bg_layer BG layer (1 - 3).
/// @param enable true to pixelate this layer.
void NEA_PostFXMosaicLayerEnable(int bg_layer, bool enable);

// ---------------------------------------------------------------------------
// Windows
// ---------------------------------------------------------------------------

/// Hardware window regions on the main engine.
typedef enum {
    NEA_POSTFX_WIN0 = 0, ///< Window 0. Highest priority where windows overlap.
    NEA_POSTFX_WIN1 = 1, ///< Window 1. Medium priority.
    NEA_POSTFX_WINOBJ = 2 ///< OBJ window, shaped by sprites. Lowest priority.
} NEA_PostFXWindow;

/// Layer mask bits for NEA_PostFXWindowSetLayers().
#define NEA_POSTFX_LAYER_BG0    (1 << 0) ///< The 3D layer
#define NEA_POSTFX_LAYER_BG1    (1 << 1)
#define NEA_POSTFX_LAYER_BG2    (1 << 2)
#define NEA_POSTFX_LAYER_BG3    (1 << 3)
#define NEA_POSTFX_LAYER_OBJ    (1 << 4)
#define NEA_POSTFX_LAYER_ALL    (0x1F)

/// Sets the rectangle covered by window 0 or 1.
///
/// Coordinates are inclusive pixel coordinates on a 256x192 screen. The
/// hardware stores the right and bottom edges as "coordinate plus one" in an
/// 8 bit field, so a window whose right edge is column 255 has to write 256,
/// which wraps to 0. That wrap is believed to be interpreted as "full width",
/// but GBATEK only documents the GBA's 240-pixel behaviour, so a full-width
/// window is the one case worth checking on hardware.
///
/// Has no effect for NEA_POSTFX_WINOBJ, whose shape comes from sprites.
///
/// @param win NEA_POSTFX_WIN0 or NEA_POSTFX_WIN1.
/// @param x1 Left edge (0 - 255).
/// @param y1 Top edge (0 - 191).
/// @param x2 Right edge, inclusive (0 - 255).
/// @param y2 Bottom edge, inclusive (0 - 191).
void NEA_PostFXWindowSetRect(NEA_PostFXWindow win, int x1, int y1,
                            int x2, int y2);

/// Chooses which layers are drawn inside a window, and whether the blend unit
/// applies there.
///
/// The `color_effect` flag is the interesting one: it gates the color special
/// effect per region, which is what makes a vignette cost zero BG layers -- set
/// a fade to black globally, then switch the effect *off* inside the window and
/// only the surrounding area darkens. NEA_PostFXVignetteEnable() does exactly
/// that.
///
/// @param win Which window to configure.
/// @param layer_mask OR of NEA_POSTFX_LAYER_*, or NEA_POSTFX_LAYER_ALL.
/// @param color_effect true to let the blend unit act inside this region.
void NEA_PostFXWindowSetLayers(NEA_PostFXWindow win, u16 layer_mask,
                              bool color_effect);

/// Chooses which layers are drawn outside every window.
///
/// @param layer_mask OR of NEA_POSTFX_LAYER_*, or NEA_POSTFX_LAYER_ALL.
/// @param color_effect true to let the blend unit act outside all windows.
void NEA_PostFXWindowSetOutsideLayers(u16 layer_mask, bool color_effect);

/// Turns a window region on or off.
///
/// Enabling any window automatically enables the "outside" region too, so
/// remember to configure it with NEA_PostFXWindowSetOutsideLayers() or
/// everything outside the window will keep its default layer set.
///
/// Costs nothing: one DISPCNT bit, no VRAM, no BG layer.
///
/// @param win Which window.
/// @param enable true to enable the region.
void NEA_PostFXWindowEnable(NEA_PostFXWindow win, bool enable);

/// Darkens everything outside a rectangle, leaving the inside untouched.
///
/// Built on window 0 plus the blend unit's brightness-decrease mode: the fade
/// is enabled globally and then disabled inside the window. Costs **no BG
/// layer and no VRAM**, which is what makes it worth doing this way instead of
/// drawing a pre-rendered vignette overlay.
///
/// The lit rectangle defaults to the middle half of the screen; call
/// NEA_PostFXWindowSetRect(NEA_POSTFX_WIN0, ...) afterwards to move or resize
/// it. A round flashlight cone needs the window's left and right edges rewritten
/// per scanline, which is what the HBlank scanline system is for.
///
/// Takes ownership of the blend unit (see @ref postfx_blend_unit).
///
/// @note GBATEK raises, and then disowns, a doubt about this technique: "If the
/// 3D screen has highest priority, then alpha-blending is always enabled,
/// regardless of the Window Control register's color effect enable flag ...
/// **not sure if that is true**". Tested under melonDS with the 3D layer at
/// priority 0, the window's color-effect bit *is* honoured and the fade stops
/// at the window edge, which is what this function relies on. Two caveats
/// remain: that was an emulator rather than hardware, and GBATEK's claim is
/// specifically about *alpha blending* while this uses brightness decrease. If
/// you combine a window with NEA_PostFXGlow* (which does alpha blend) and the
/// glow ignores the window, that untested case is the reason.
///
/// @param enable true to enable the vignette.
/// @param strength How dark the outside gets, 0 (none) to 16 (black).
void NEA_PostFXVignetteEnable(bool enable, int strength);

// ---------------------------------------------------------------------------
// Frame capture: motion blur and afterimage
// ---------------------------------------------------------------------------

/// Decay presets for NEA_PostFXMotionBlurPreset().
typedef enum {
    /// Short trails that disappear in a few frames. Reads as motion blur.
    NEA_POSTFX_BLUR_FAST = 0,
    /// Long-lived smear. Reads as a camera with a slow shutter.
    NEA_POSTFX_BLUR_MEDIUM,
    /// Very slow decay: moving objects leave persistent ghosts behind them.
    NEA_POSTFX_BLUR_GHOST
} NEA_PostFXBlurPreset;

/// Sets up the frame capture pipeline used by motion blur and afterimage.
///
/// Each frame the composited output is captured into one VRAM bank while the
/// previous frame's capture is displayed from the other as a bitmap background
/// and blended with the live 3D image. Because what gets captured is the
/// *blended* result, the history decays exponentially rather than being a
/// single stale frame.
///
/// @section postfx_capture_cost What this costs
///
/// **Two VRAM banks, 256 KB, half of the A-D pool.** That is by far the most
/// expensive thing in this module and it comes straight out of the 3D texture
/// budget, so call NEA_TextureSystemReset() with the two banks you are *not*
/// giving to the capture (e.g. NEA_VRAM_AB if capture gets C and D) before
/// loading any textures. This function checks that the requested banks are not
/// already owned by the texture system and fails rather than corrupting it.
///
/// It also takes the blend unit and BG2, and switches the main engine to video
/// mode 5 so BG2 can be a direct-color bitmap.
///
/// Per frame it costs a bank re-map and three register writes, done from
/// NEA_PostFXUpdate(). There is no per-pixel CPU work at all: the PPU does the
/// blending and the capture unit does the write-back.
///
/// @section postfx_capture_limits Limits
///
/// - **Requires NEA_ModeSingle3D.** The dual 3D and two-pass modes already own
///   DISPCAPCNT and ping-pong VRAM_C/D themselves; there is no room to share.
/// - Capture is main-engine only; the sub screen is unaffected.
/// - The captured image is 15 bit color, so long ghost trails band slightly.
/// - Texture uploads call vramSetPrimaryBanks() to reach texture VRAM, which
///   briefly re-maps the capture banks too. Uploading textures mid-gameplay can
///   therefore corrupt one frame of history. Load textures up front, or accept
///   a one-frame glitch.
///
/// @param banks Exactly two of NEA_VRAM_A / B / C / D.
/// @return 1 on success, 0 on error.
int NEA_PostFXCaptureInit(NEA_VRAMBankFlags banks);

/// Tears the capture pipeline down and returns the main engine to mode 0.
///
/// Does not give the VRAM banks back to the texture system; call
/// NEA_TextureSystemReset() yourself if you want them back.
void NEA_PostFXCaptureEnd(void);

/// Returns the VRAM banks currently owned by the capture pipeline.
///
/// NEA_TextureSystemReset() calls this through a weak reference so it never
/// hands a capture bank out for textures.
///
/// @return Bitmask of NEA_VRAM_A / B / C / D, or 0 if capture is not running.
NEA_VRAMBankFlags NEA_PostFXGetCaptureBanks(void);

/// Turns motion blur on or off.
///
/// Enabling takes ownership of the blend unit (see @ref postfx_blend_unit) and
/// requires NEA_UPDATE_POSTFX to be passed to NEA_WaitForVBL(), which is what
/// drives the per-frame bank swap.
///
/// @param enable true to start blending frames together.
void NEA_PostFXCaptureEnable(bool enable);

/// Sets how much of the previous frame survives into the next one.
///
/// The composite is `out = history*decay/16 + current*(16-decay)/16`, so 0
/// disables the effect entirely and 16 would freeze the image. Useful values
/// are roughly 4 (a hint of smear) to 13 (long ghosting).
///
/// @param decay 0 - 15.
void NEA_PostFXMotionBlurSetDecay(int decay);

/// Applies one of the named decay presets.
///
/// @param preset Which preset to use.
void NEA_PostFXMotionBlurPreset(NEA_PostFXBlurPreset preset);

/// Pixelates the captured 3D image.
///
/// This is the workaround for mosaic not working on the 3D layer: once the
/// scene has been captured into a bitmap background, that background is an
/// ordinary 2D layer and the mosaic hardware applies to it normally. It only
/// works while the capture pipeline is running.
///
/// Note that this pixelates the *blended* image, so with a non-zero decay the
/// trails are pixelated too.
///
/// @param h Horizontal block size (0 - 15, 0 disables).
/// @param v Vertical block size (0 - 15, 0 disables).
void NEA_PostFXCaptureMosaic(int h, int v);

/// Per-frame work for the capture effects. Called by NEA_WaitForVBL() when
/// NEA_UPDATE_POSTFX is passed; you do not normally call this yourself.
void NEA_PostFXUpdate(void);

/// @}

#endif // NEA_POSTFX_H__
