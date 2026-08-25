// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Nitro Engine Advanced contributors
//
// This file is part of Nitro Engine Advanced

#include "NEAMain.h"

/// @file NEAPostFX.c

// The 2D blend unit (BLDCNT) is a single shared resource: bits 6-7 pick one
// mode for the whole engine. Rather than let two effects quietly fight over it,
// track who holds it and hand it over explicitly.
static NEA_PostFXBlendOwner ne_postfx_blend_owner = NEA_POSTFX_BLEND_NONE;

NEA_PostFXBlendOwner NEA_PostFXBlendOwner_Get(void)
{
    return ne_postfx_blend_owner;
}

// Called on the outgoing owner when someone else takes the blend unit. The glow
// needs this: its overlay layer is fully opaque, so if it stayed visible without
// the blend unit it would simply hide the scene instead of tinting it.
static void ne_glow_on_blend_lost(void);

static void ne_postfx_take_blend_unit(NEA_PostFXBlendOwner owner)
{
    if (ne_postfx_blend_owner != NEA_POSTFX_BLEND_NONE
        && ne_postfx_blend_owner != owner)
    {
        NEA_DebugPrint("PostFX: blend unit taken from owner %d by %d",
                      ne_postfx_blend_owner, owner);

        if (ne_postfx_blend_owner == NEA_POSTFX_BLEND_GLOW)
            ne_glow_on_blend_lost();
    }

    ne_postfx_blend_owner = owner;
}

static void ne_postfx_release_blend_unit(NEA_PostFXBlendOwner owner)
{
    // Only the current owner may switch the unit off, so that disabling a
    // effect that already lost the unit doesn't clobber the new holder.
    if (ne_postfx_blend_owner != owner)
        return;

    REG_BLDCNT = 0;
    REG_BLDY = 0;
    ne_postfx_blend_owner = NEA_POSTFX_BLEND_NONE;
}

//-----------------------------------------------------------------------------
// Full-screen flash / fade
//-----------------------------------------------------------------------------

void NEA_PostFXFlashSet(NEA_PostFXFlashMode mode, int strength)
{
    if (mode == NEA_POSTFX_FLASH_NONE)
    {
        ne_postfx_release_blend_unit(NEA_POSTFX_BLEND_FLASH);
        return;
    }

    if (strength < 0)
        strength = 0;
    if (strength > 16)
        strength = 16;

    ne_postfx_take_blend_unit(NEA_POSTFX_BLEND_FLASH);

    // Every layer is a 1st target. GBATEK: brightness up/down works on BG0
    // "as for 2D", unlike alpha blending, so EVY behaves normally on the 3D
    // layer too. All the other layers and the backdrop are included because a
    // full-screen flash or fade that leaves the HUD or the clear color at full
    // brightness does not read as a flash at all.
    u16 targets = BLEND_SRC_BG0 | BLEND_SRC_BG1 | BLEND_SRC_BG2
                | BLEND_SRC_BG3 | BLEND_SRC_SPRITE | BLEND_SRC_BACKDROP;

    REG_BLDCNT = targets | ((mode == NEA_POSTFX_FLASH_WHITE)
                            ? BLEND_FADE_WHITE : BLEND_FADE_BLACK);
    REG_BLDY = strength;
}

//-----------------------------------------------------------------------------
// Colored glow overlay
//-----------------------------------------------------------------------------

// 15 usable bands: a 4bpp tile can index 16 palette entries and entry 0 is
// transparent, so the ramp runs over indices 1..15.
#define NE_GLOW_BANDS       15
#define NE_GLOW_MAP_ROWS    32
#define NE_GLOW_TILE_BYTES  32  // 8x8 pixels at 4bpp

static NEA_Hw2DBG *ne_glow_bg = NULL;
static int ne_glow_layer = -1;
static int ne_glow_palette_slot = 0;
static int ne_glow_intensity = 8;
static bool ne_glow_enabled = false;

// Palette staging buffer. Static rather than on the stack because
// NEA_Hw2DBGLoadPalette() may DMA out of it.
static u16 ne_glow_palette[16];

static u16 ne_glow_blend_src_bit(int layer)
{
    switch (layer)
    {
        case 1:  return BLEND_SRC_BG1;
        case 2:  return BLEND_SRC_BG2;
        case 3:  return BLEND_SRC_BG3;
        default: return 0;
    }
}

static void ne_glow_apply_blend(void)
{
    if (!ne_glow_enabled || ne_glow_bg == NULL)
        return;

    // Glow layer is the 1st target and sits above the scene; the 3D layer is
    // the 2nd target. This ordering is mandatory: with BG0 as the 1st target
    // the hardware would use the per-pixel 3D alpha and ignore EVA/EVB, so the
    // intensity control would stop working.
    //
    // EVB is pinned at 16/16 so the scene keeps full brightness and the glow is
    // added on top: I = min(31, I_glow*EVA + I_3D*EVB).
    REG_BLDCNT = ne_glow_blend_src_bit(ne_glow_layer)
               | BLEND_ALPHA
               | BLEND_DST_BG0 | BLEND_DST_BACKDROP;

    REG_BLDALPHA = (ne_glow_intensity & 0x1F) | (16 << 8);
}

int NEA_PostFXGlowInit(int bg_layer, int palette_slot)
{
    if (NEA_CurrentExecutionMode() != NEA_ModeSingle3D)
    {
        // Dual 3D and two-pass rewrite BG0/BG1/BG2 priorities every frame to
        // composite their captures, which would fight the layer ordering the
        // blend needs.
        NEA_DebugPrint("PostFX glow needs single 3D mode");
        return 0;
    }

    if (bg_layer < 1 || bg_layer > 3)
    {
        NEA_DebugPrint("Invalid glow BG layer %d (1-3, 0 is the 3D layer)",
                      bg_layer);
        return 0;
    }

    if (palette_slot < 0 || palette_slot > 15)
    {
        NEA_DebugPrint("Invalid glow palette slot %d", palette_slot);
        return 0;
    }

    NEA_PostFXGlowEnd();

    ne_glow_bg = NEA_Hw2DBGCreate(NEA_ENGINE_MAIN, bg_layer,
                                 NEA_HW2D_BG_TILED_4BPP, 256, 256);
    if (ne_glow_bg == NULL)
    {
        NEA_DebugPrint("Could not create glow BG (is Hw2D initialized?)");
        return 0;
    }

    ne_glow_layer = bg_layer;
    ne_glow_palette_slot = palette_slot;

    // One flat tile per band. At 4bpp a byte holds two pixels, so filling a
    // tile with palette index v means writing v | (v << 4) everywhere.
    u16 *gfx = ne_glow_bg->gfx_ptr;
    for (int band = 0; band < NE_GLOW_BANDS; band++)
    {
        u16 v = (u16)(band + 1);
        u16 fill = v | (v << 4) | (v << 8) | (v << 12);

        for (int w = 0; w < NE_GLOW_TILE_BYTES / 2; w++)
            gfx[band * (NE_GLOW_TILE_BYTES / 2) + w] = fill;
    }

    // Map each tile row to a band. Only 24 of the 32 rows are on screen, so
    // spread the ramp across those and let the off-screen rows hold the last
    // band; that way a scrolled glow doesn't show a seam.
    u16 *map = ne_glow_bg->map_ptr;
    for (int row = 0; row < NE_GLOW_MAP_ROWS; row++)
    {
        int band = (row * NE_GLOW_BANDS) / 24;
        if (band > NE_GLOW_BANDS - 1)
            band = NE_GLOW_BANDS - 1;

        u16 entry = (u16)band | (u16)(palette_slot << 12);

        for (int col = 0; col < 32; col++)
            map[row * 32 + col] = entry;
    }

    // The glow has to be in front of the scene to be the blend's 1st target.
    NEA_Hw2DBGSetPriority(ne_glow_bg, 0);
    REG_BG0CNT = (REG_BG0CNT & ~BG_PRIORITY(3)) | BG_PRIORITY(1);

    NEA_PostFXGlowSetColor(RGB15(31, 20, 8)); // A warm firelight default
    NEA_PostFXGlowEnable(false);

    return 1;
}

void NEA_PostFXGlowEnd(void)
{
    if (ne_glow_bg == NULL)
        return;

    NEA_PostFXGlowEnable(false);
    NEA_Hw2DBGDelete(ne_glow_bg);

    ne_glow_bg = NULL;
    ne_glow_layer = -1;
}

static void ne_glow_upload_palette(void)
{
    if (ne_glow_bg == NULL)
        return;

    // Third argument is a color count, not a byte count.
    NEA_Hw2DBGLoadPalette(ne_glow_bg, ne_glow_palette,
                         16, ne_glow_palette_slot);
}

void NEA_PostFXGlowSetColor(u16 color)
{
    if (ne_glow_bg == NULL)
        return;

    // Index 0 stays transparent; the ramp indices all take the same color.
    ne_glow_palette[0] = 0;
    for (int i = 1; i < 16; i++)
        ne_glow_palette[i] = color;

    ne_glow_upload_palette();
}

void NEA_PostFXGlowSetGradient(u16 top_color, u16 bottom_color)
{
    if (ne_glow_bg == NULL)
        return;

    int r0 = top_color & 0x1F;
    int g0 = (top_color >> 5) & 0x1F;
    int b0 = (top_color >> 10) & 0x1F;

    int r1 = bottom_color & 0x1F;
    int g1 = (bottom_color >> 5) & 0x1F;
    int b1 = (bottom_color >> 10) & 0x1F;

    ne_glow_palette[0] = 0;

    for (int i = 0; i < NE_GLOW_BANDS; i++)
    {
        // Lerp in 8.8 with a reciprocal folded into the constant, so there is
        // no division: 1/14 in 8.8 is 18.28, and 18 is close enough over a
        // 15 step ramp that the rounding never shows.
        int t = (i * 18);
        if (t > 256)
            t = 256;

        int r = r0 + (((r1 - r0) * t) >> 8);
        int g = g0 + (((g1 - g0) * t) >> 8);
        int b = b0 + (((b1 - b0) * t) >> 8);

        ne_glow_palette[i + 1] = RGB15(r, g, b);
    }

    ne_glow_upload_palette();
}

void NEA_PostFXGlowSetIntensity(int intensity)
{
    if (intensity < 0)
        intensity = 0;
    if (intensity > 16)
        intensity = 16;

    ne_glow_intensity = intensity;

    if (ne_glow_enabled)
        REG_BLDALPHA = (ne_glow_intensity & 0x1F) | (16 << 8);
}

static void ne_glow_on_blend_lost(void)
{
    // Keep ne_glow_enabled set: the caller may hand the unit back, and
    // NEA_PostFXGlowEnable(true) will make the layer visible again.
    if (ne_glow_bg != NULL)
        NEA_Hw2DBGSetVisible(ne_glow_bg, false);
}

void NEA_PostFXGlowEnable(bool enable)
{
    if (ne_glow_bg == NULL)
        return;

    if (enable)
    {
        ne_glow_enabled = true;
        ne_postfx_take_blend_unit(NEA_POSTFX_BLEND_GLOW);
        NEA_Hw2DBGSetVisible(ne_glow_bg, true);
        ne_glow_apply_blend();
    }
    else
    {
        ne_glow_enabled = false;
        NEA_Hw2DBGSetVisible(ne_glow_bg, false);
        ne_postfx_release_blend_unit(NEA_POSTFX_BLEND_GLOW);
    }
}
