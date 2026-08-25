// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Nitro Engine Advanced contributors
//
// This file is part of Nitro Engine Advanced

#ifndef NEA_POSTFX_H__
#define NEA_POSTFX_H__

#include <nds.h>

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
    NEA_POSTFX_BLEND_FLASH     ///< NEA_PostFXFlashSet()
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

/// @}

#endif // NEA_POSTFX_H__
