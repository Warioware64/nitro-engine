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

// The vignette leaves WININ bit 5 clear so the fade is suppressed inside the
// window. That gating applies to whatever owns the blend unit, so if the
// vignette loses it the window has to go away too, or the incoming effect
// silently stops working inside that rectangle.
static void ne_vignette_on_blend_lost(void);

static void ne_postfx_take_blend_unit(NEA_PostFXBlendOwner owner)
{
    if (ne_postfx_blend_owner != NEA_POSTFX_BLEND_NONE
        && ne_postfx_blend_owner != owner)
    {
        NEA_DebugPrint("PostFX: blend unit taken from owner %d by %d",
                      ne_postfx_blend_owner, owner);

        if (ne_postfx_blend_owner == NEA_POSTFX_BLEND_GLOW)
            ne_glow_on_blend_lost();
        else if (ne_postfx_blend_owner == NEA_POSTFX_BLEND_VIGNETTE)
            ne_vignette_on_blend_lost();
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

//-----------------------------------------------------------------------------
// Mosaic
//-----------------------------------------------------------------------------

void NEA_PostFXMosaicSet(int bg_h, int bg_v, int obj_h, int obj_v)
{
    if (bg_h < 0) bg_h = 0;
    if (bg_h > 15) bg_h = 15;
    if (bg_v < 0) bg_v = 0;
    if (bg_v > 15) bg_v = 15;
    if (obj_h < 0) obj_h = 0;
    if (obj_h > 15) obj_h = 15;
    if (obj_v < 0) obj_v = 0;
    if (obj_v > 15) obj_v = 15;

    REG_MOSAIC = (u16)(bg_h | (bg_v << 4) | (obj_h << 8) | (obj_v << 12));
}

void NEA_PostFXMosaicLayerEnable(int bg_layer, bool enable)
{
    if (bg_layer == 0)
    {
        // GBATEK: "mosaic cannot be used on the 3D layer". Setting the bit in
        // BG0CNT would be silently ignored by the hardware, so say so instead.
        NEA_DebugPrint("Mosaic has no effect on the 3D layer (BG0)");
        return;
    }

    if (bg_layer < 1 || bg_layer > 3)
    {
        NEA_DebugPrint("Invalid mosaic BG layer %d", bg_layer);
        return;
    }

    vu16 *cnt;
    switch (bg_layer)
    {
        case 1:  cnt = &REG_BG1CNT; break;
        case 2:  cnt = &REG_BG2CNT; break;
        default: cnt = &REG_BG3CNT; break;
    }

    if (enable)
        *cnt |= BIT(6);
    else
        *cnt &= ~BIT(6);
}

//-----------------------------------------------------------------------------
// Windows
//-----------------------------------------------------------------------------

// WININ holds window 0 in bits 0-7 and window 1 in bits 8-15; WINOUT holds the
// outside region in bits 0-7 and the OBJ window in bits 8-15. Both halves have
// the same layout: layer enables in bits 0-4, color effect enable in bit 5.
#define NE_WIN_COLOR_EFFECT_BIT BIT(5)

static u16 ne_win_field(u16 layer_mask, bool color_effect)
{
    u16 v = layer_mask & 0x1F;
    if (color_effect)
        v |= NE_WIN_COLOR_EFFECT_BIT;
    return v;
}

void NEA_PostFXWindowSetRect(NEA_PostFXWindow win, int x1, int y1,
                            int x2, int y2)
{
    if (win == NEA_POSTFX_WINOBJ)
    {
        NEA_DebugPrint("The OBJ window's shape comes from sprites, not a rect");
        return;
    }

    if (x1 < 0) x1 = 0;
    if (x1 > 255) x1 = 255;
    if (x2 < x1) x2 = x1;
    if (x2 > 255) x2 = 255;
    if (y1 < 0) y1 = 0;
    if (y1 > 191) y1 = 191;
    if (y2 < y1) y2 = y1;
    if (y2 > 191) y2 = 191;

    // The hardware wants the right/bottom edge plus one. A right edge of 255
    // therefore writes 256, which wraps to 0 in the 8 bit field; that is
    // believed to read as "full width", but see the header note.
    u16 h = (u16)((x1 << 8) | ((x2 + 1) & 0xFF));
    u16 v = (u16)((y1 << 8) | ((y2 + 1) & 0xFF));

    if (win == NEA_POSTFX_WIN0)
    {
        REG_WIN0H = h;
        REG_WIN0V = v;
    }
    else
    {
        REG_WIN1H = h;
        REG_WIN1V = v;
    }
}

void NEA_PostFXWindowSetLayers(NEA_PostFXWindow win, u16 layer_mask,
                              bool color_effect)
{
    u16 field = ne_win_field(layer_mask, color_effect);

    switch (win)
    {
        case NEA_POSTFX_WIN0:
            REG_WININ = (REG_WININ & 0xFF00) | field;
            break;
        case NEA_POSTFX_WIN1:
            REG_WININ = (REG_WININ & 0x00FF) | (field << 8);
            break;
        case NEA_POSTFX_WINOBJ:
            REG_WINOUT = (REG_WINOUT & 0x00FF) | (field << 8);
            break;
    }
}

void NEA_PostFXWindowSetOutsideLayers(u16 layer_mask, bool color_effect)
{
    u16 field = ne_win_field(layer_mask, color_effect);
    REG_WINOUT = (REG_WINOUT & 0xFF00) | field;
}

void NEA_PostFXWindowEnable(NEA_PostFXWindow win, bool enable)
{
    u32 bit;
    switch (win)
    {
        case NEA_POSTFX_WIN0:   bit = DISPLAY_WIN0_ON; break;
        case NEA_POSTFX_WIN1:   bit = DISPLAY_WIN1_ON; break;
        default:                bit = DISPLAY_SPR_WIN_ON; break;
    }

    if (enable)
        REG_DISPCNT |= bit;
    else
        REG_DISPCNT &= ~bit;
}

//-----------------------------------------------------------------------------
// Vignette
//-----------------------------------------------------------------------------

static bool ne_vignette_enabled = false;

static void ne_vignette_on_blend_lost(void)
{
    if (!ne_vignette_enabled)
        return;

    NEA_PostFXWindowEnable(NEA_POSTFX_WIN0, false);
    ne_vignette_enabled = false;
}

void NEA_PostFXVignetteEnable(bool enable, int strength)
{
    if (!enable)
    {
        if (ne_vignette_enabled)
        {
            NEA_PostFXWindowEnable(NEA_POSTFX_WIN0, false);
            ne_vignette_enabled = false;
        }
        ne_postfx_release_blend_unit(NEA_POSTFX_BLEND_VIGNETTE);
        return;
    }

    if (strength < 0)
        strength = 0;
    if (strength > 16)
        strength = 16;

    ne_postfx_take_blend_unit(NEA_POSTFX_BLEND_VIGNETTE);

    if (!ne_vignette_enabled)
    {
        // Default lit area: the middle half of the screen. The caller can move
        // it afterwards with NEA_PostFXWindowSetRect().
        NEA_PostFXWindowSetRect(NEA_POSTFX_WIN0, 64, 48, 191, 143);
        ne_vignette_enabled = true;
    }

    // Everything is drawn in both regions; the only difference is that the
    // brightness-decrease effect is switched off inside the window. That is
    // what makes this cost no BG layer at all.
    NEA_PostFXWindowSetLayers(NEA_POSTFX_WIN0, NEA_POSTFX_LAYER_ALL, false);
    NEA_PostFXWindowSetOutsideLayers(NEA_POSTFX_LAYER_ALL, true);
    NEA_PostFXWindowEnable(NEA_POSTFX_WIN0, true);

    REG_BLDCNT = BLEND_SRC_BG0 | BLEND_SRC_BG1 | BLEND_SRC_BG2
               | BLEND_SRC_BG3 | BLEND_SRC_SPRITE | BLEND_SRC_BACKDROP
               | BLEND_FADE_BLACK;
    REG_BLDY = strength;
}

//-----------------------------------------------------------------------------
// Frame capture: motion blur and afterimage
//-----------------------------------------------------------------------------

static bool ne_capture_inited = false;
static bool ne_capture_enabled = false;
static int ne_capture_decay = 8;
static int ne_capture_bank[2] = { -1, -1 }; // 0=A, 1=B, 2=C, 3=D
static int ne_capture_write = 0;            // which slot is the capture target
static u32 ne_capture_saved_mode = 0;

// Maps a bank index to its LCD / main-BG-slot-0 configuration. Both slots have
// to be able to play either role, which is why this is a table rather than the
// hardcoded C/D pair that the two-pass modes use.
static void ne_capture_set_bank(int bank, bool as_bg)
{
    switch (bank)
    {
        case 0: vramSetBankA(as_bg ? VRAM_A_MAIN_BG_0x06000000 : VRAM_A_LCD); break;
        case 1: vramSetBankB(as_bg ? VRAM_B_MAIN_BG_0x06000000 : VRAM_B_LCD); break;
        case 2: vramSetBankC(as_bg ? VRAM_C_MAIN_BG_0x06000000 : VRAM_C_LCD); break;
        default: vramSetBankD(as_bg ? VRAM_D_MAIN_BG_0x06000000 : VRAM_D_LCD); break;
    }
}

// DCAP_BANK_VRAM_A..D happen to be 0..3, the same order as our bank indices.
static u32 ne_capture_dcap_bank(int bank)
{
    return (u32)bank & 3;
}

NEA_VRAMBankFlags NEA_PostFXGetCaptureBanks(void)
{
    if (!ne_capture_inited)
        return 0;

    return (NEA_VRAMBankFlags)((1 << ne_capture_bank[0])
                             | (1 << ne_capture_bank[1]));
}

int NEA_PostFXCaptureInit(NEA_VRAMBankFlags banks)
{
    if (NEA_CurrentExecutionMode() != NEA_ModeSingle3D)
    {
        // Dual 3D and two-pass already drive DISPCAPCNT and swap VRAM_C/D every
        // frame for their own compositing. Sharing is not possible.
        NEA_DebugPrint("PostFX capture needs single 3D mode");
        return 0;
    }

    // Exactly two of A-D. One bank cannot both hold the history and receive the
    // new capture in the same frame.
    int found = 0;
    int idx[2] = { -1, -1 };
    for (int i = 0; i < 4; i++)
    {
        if (banks & (1 << i))
        {
            if (found < 2)
                idx[found] = i;
            found++;
        }
    }

    if (found != 2)
    {
        NEA_DebugPrint("PostFX capture needs exactly 2 banks of A-D, got %d",
                      found);
        return 0;
    }

    NEA_PostFXCaptureEnd();

    ne_capture_bank[0] = idx[0];
    ne_capture_bank[1] = idx[1];
    ne_capture_write = 0;

    ne_capture_saved_mode = REG_DISPCNT;

    // BG2 as a 16 bit direct-color bitmap reading from 0x06000000, which is
    // whichever bank is currently mapped to main BG slot 0. Identity affine
    // transform: the scanline distortion system is what changes these later.
    REG_BG2CNT = BG_BMP16_256x256 | BG_BMP_BASE(0) | BG_PRIORITY(0);
    REG_BG2PA = 1 << 8;
    REG_BG2PB = 0;
    REG_BG2PC = 0;
    REG_BG2PD = 1 << 8;
    REG_BG2X = 0;
    REG_BG2Y = 0;

    // Mode 5 is what makes BG2 an extended-affine bitmap layer. Preserve the
    // window enables, which live in the same register and which the vignette
    // may already have set up.
    u32 win_bits = ne_capture_saved_mode
                 & (DISPLAY_WIN0_ON | DISPLAY_WIN1_ON | DISPLAY_SPR_WIN_ON);

    videoSetMode(MODE_5_3D | DISPLAY_BG2_ACTIVE | win_bits);

    // History layer in front of the live 3D image: alpha blending needs the
    // 1st target to be the topmost pixel.
    REG_BG0CNT = (REG_BG0CNT & ~BG_PRIORITY(3)) | BG_PRIORITY(1);

    ne_capture_set_bank(ne_capture_bank[0], false); // capture destination
    ne_capture_set_bank(ne_capture_bank[1], true);  // displayed history

    ne_capture_inited = true;
    ne_capture_enabled = false;

    return 1;
}

void NEA_PostFXCaptureEnd(void)
{
    if (!ne_capture_inited)
        return;

    NEA_PostFXCaptureEnable(false);

    REG_DISPCAPCNT = 0;

    ne_capture_set_bank(ne_capture_bank[0], false);
    ne_capture_set_bank(ne_capture_bank[1], false);

    videoSetMode(ne_capture_saved_mode);
    REG_BG0CNT = (REG_BG0CNT & ~BG_PRIORITY(3)) | BG_PRIORITY(0);

    ne_capture_bank[0] = -1;
    ne_capture_bank[1] = -1;
    ne_capture_inited = false;
}

void NEA_PostFXMotionBlurSetDecay(int decay)
{
    if (decay < 0)
        decay = 0;
    if (decay > 15)
        decay = 15;

    ne_capture_decay = decay;
}

void NEA_PostFXMotionBlurPreset(NEA_PostFXBlurPreset preset)
{
    switch (preset)
    {
        case NEA_POSTFX_BLUR_FAST:   NEA_PostFXMotionBlurSetDecay(5); break;
        case NEA_POSTFX_BLUR_MEDIUM: NEA_PostFXMotionBlurSetDecay(9); break;
        default:                     NEA_PostFXMotionBlurSetDecay(13); break;
    }
}

void NEA_PostFXCaptureMosaic(int h, int v)
{
    if (!ne_capture_inited)
    {
        NEA_DebugPrint("Capture mosaic needs the capture pipeline running");
        return;
    }

    // Once the 3D image lives in a bitmap background it is an ordinary 2D
    // layer, so the mosaic hardware applies to it -- which is the only way to
    // pixelate the rendered scene at all.
    NEA_PostFXMosaicSet(h, v, 0, 0);
    NEA_PostFXMosaicLayerEnable(2, (h > 0) || (v > 0));
}

void NEA_PostFXCaptureEnable(bool enable)
{
    if (!ne_capture_inited)
    {
        if (enable)
            NEA_DebugPrint("Call NEA_PostFXCaptureInit() first");
        return;
    }

    if (enable)
    {
        ne_capture_enabled = true;
        ne_postfx_take_blend_unit(NEA_POSTFX_BLEND_CAPTURE);
    }
    else
    {
        ne_capture_enabled = false;
        REG_DISPCAPCNT = 0;
        ne_postfx_release_blend_unit(NEA_POSTFX_BLEND_CAPTURE);
    }
}

void NEA_PostFXUpdate(void)
{
    if (!ne_capture_inited || !ne_capture_enabled)
        return;

    // Another effect took the blend unit; stop compositing rather than fighting
    // over BLDCNT every frame.
    if (ne_postfx_blend_owner != NEA_POSTFX_BLEND_CAPTURE)
        return;

    // Swap roles: last frame's capture destination becomes this frame's
    // displayed history, and the bank that was on screen is written into.
    ne_capture_write ^= 1;

    int write_bank = ne_capture_bank[ne_capture_write];
    int show_bank = ne_capture_bank[ne_capture_write ^ 1];

    ne_capture_set_bank(write_bank, false);
    ne_capture_set_bank(show_bank, true);

    // History (BG2, in front) over the live 3D image (BG0, behind):
    //   out = history*EVA + current*EVB
    // with EVA = decay. Capturing that composite is what makes the decay
    // exponential instead of a single stale frame.
    REG_BLDCNT = BLEND_SRC_BG2 | BLEND_ALPHA
               | BLEND_DST_BG0 | BLEND_DST_BACKDROP;
    REG_BLDALPHA = (u16)((ne_capture_decay & 0x1F)
                        | (((16 - ne_capture_decay) & 0x1F) << 8));

    // Capture the composited screen (not DCAP_SRC_A_3DONLY: the whole point is
    // to feed the blend result back in). Capture begins at the next line 0 and
    // the enable bit clears itself at line 192.
    REG_DISPCAPCNT = DCAP_BANK(ne_capture_dcap_bank(write_bank))
                   | DCAP_SIZE(DCAP_SIZE_256x192)
                   | DCAP_MODE(DCAP_MODE_A)
                   | DCAP_SRC_A(DCAP_SRC_A_COMPOSITED)
                   | DCAP_ENABLE;
}

//-----------------------------------------------------------------------------
// Per-scanline distortion (HBlank driven)
//-----------------------------------------------------------------------------

// Static storage, sized at compile time: 3 tables of 192 entries is 1152 bytes.
static s16 ne_scanline_table[NEA_SCANLINE_TARGET_COUNT][NEA_SCANLINE_ENTRIES];
static u8 ne_scanline_on[NEA_SCANLINE_TARGET_COUNT];
static int ne_scanline_win_center = 128;

// Remembered sine parameters so AdvancePhase() can refill without the caller
// having to keep them.
static s16 ne_scanline_amp[NEA_SCANLINE_TARGET_COUNT];
static s16 ne_scanline_freq[NEA_SCANLINE_TARGET_COUNT];
static s16 ne_scanline_phase[NEA_SCANLINE_TARGET_COUNT];

extern void __NEA_SetHBLPostFXHook(void (*hook)(int vcount));

// Runs on every visible scanline. Kept in ITCM and in ARM mode: at 192 calls a
// frame the instruction fetch path matters more than code size, and a cache
// miss here shows up as a visible tear rather than as a slow frame.
ITCM_CODE ARM_CODE
static void ne_postfx_hbl(int vcount)
{
    if ((unsigned)vcount >= NEA_SCANLINE_ENTRIES)
        return;

    if (ne_scanline_on[NEA_SCANLINE_BG0HOFS])
        REG_BG0HOFS = (u16)ne_scanline_table[NEA_SCANLINE_BG0HOFS][vcount];

    if (ne_scanline_on[NEA_SCANLINE_BG2X])
    {
        // BG2X is 20.8 fixed point. Writing it mid-frame updates the internal
        // register for the following lines; BG2Y keeps auto-incrementing by
        // BG2PD on its own, so the image still advances a row per scanline.
        REG_BG2X = ((s32)ne_scanline_table[NEA_SCANLINE_BG2X][vcount]) << 8;
    }

    if (ne_scanline_on[NEA_SCANLINE_WIN0H])
    {
        int half = ne_scanline_table[NEA_SCANLINE_WIN0H][vcount];

        if (half <= 0)
        {
            // Closed on this line. x1 == x2 leaves an empty window.
            REG_WIN0H = 0;
        }
        else
        {
            int x1 = ne_scanline_win_center - half;
            int x2 = ne_scanline_win_center + half;

            if (x1 < 0)
                x1 = 0;
            if (x2 > 255)
                x2 = 255;

            REG_WIN0H = (u16)((x1 << 8) | ((x2 + 1) & 0xFF));
        }
    }
}

static void ne_scanline_refresh_hook(void)
{
    for (int i = 0; i < NEA_SCANLINE_TARGET_COUNT; i++)
    {
        if (ne_scanline_on[i])
        {
            __NEA_SetHBLPostFXHook(ne_postfx_hbl);
            return;
        }
    }

    __NEA_SetHBLPostFXHook(NULL);
}

void NEA_PostFXScanlineSetTable(NEA_ScanlineTarget target, const s16 *table)
{
    if ((unsigned)target >= NEA_SCANLINE_TARGET_COUNT)
        return;

    if (table == NULL)
    {
        for (int i = 0; i < NEA_SCANLINE_ENTRIES; i++)
            ne_scanline_table[target][i] = 0;
        return;
    }

    for (int i = 0; i < NEA_SCANLINE_ENTRIES; i++)
        ne_scanline_table[target][i] = table[i];
}

void NEA_PostFXScanlineGenerateSine(NEA_ScanlineTarget target, int amplitude,
                                   int freq, int phase)
{
    if ((unsigned)target >= NEA_SCANLINE_TARGET_COUNT)
        return;

    ne_scanline_amp[target] = (s16)amplitude;
    ne_scanline_freq[target] = (s16)freq;
    ne_scanline_phase[target] = (s16)(phase & 0x1FF);

    for (int y = 0; y < NEA_SCANLINE_ENTRIES; y++)
    {
        // freq is in 1/16 periods per screen: 16 means one full period over the
        // 192 lines. (y * freq) >> 4 keeps that in the 0-511 angle space
        // without a division.
        int angle = (((y * freq) >> 4) + phase) & 0x1FF;

        // sinLerp() takes a 0-0x7FFF angle and returns f32 (4096 = 1.0).
        int32_t s = sinLerp(angle << 6);

        ne_scanline_table[target][y] = (s16)((s * amplitude) >> 12);
    }
}

void NEA_PostFXScanlineAdvancePhase(NEA_ScanlineTarget target, int delta)
{
    if ((unsigned)target >= NEA_SCANLINE_TARGET_COUNT)
        return;

    int phase = (ne_scanline_phase[target] + delta) & 0x1FF;

    NEA_PostFXScanlineGenerateSine(target, ne_scanline_amp[target],
                                  ne_scanline_freq[target], phase);
}

void NEA_PostFXScanlineSetWindowCenter(int x)
{
    if (x < 0)
        x = 0;
    if (x > 255)
        x = 255;

    ne_scanline_win_center = x;
}

void NEA_PostFXScanlineGenerateCircle(int center_y, int radius)
{
    // half_width(y) = sqrt(r^2 - (y - cy)^2). sqrt32() is libnds' integer
    // square root, so this stays free of both division and floating point.
    int r2 = radius * radius;

    for (int y = 0; y < NEA_SCANLINE_ENTRIES; y++)
    {
        int dy = y - center_y;
        int d2 = dy * dy;

        if (d2 >= r2)
        {
            ne_scanline_table[NEA_SCANLINE_WIN0H][y] = 0;
            continue;
        }

        ne_scanline_table[NEA_SCANLINE_WIN0H][y] = (s16)sqrt32(r2 - d2);
    }
}

void NEA_PostFXScanlineEnable(NEA_ScanlineTarget target, bool enable)
{
    if ((unsigned)target >= NEA_SCANLINE_TARGET_COUNT)
        return;

    if (!enable && ne_scanline_on[target])
    {
        // Put the register back where the rest of the engine expects it,
        // otherwise the last scanline's value sticks for the whole screen.
        if (target == NEA_SCANLINE_BG0HOFS)
            REG_BG0HOFS = 0;
        else if (target == NEA_SCANLINE_BG2X)
            REG_BG2X = 0;
    }

    ne_scanline_on[target] = enable ? 1 : 0;
    ne_scanline_refresh_hook();
}

//-----------------------------------------------------------------------------
// Sub-pixel jitter and temporal accumulation
//-----------------------------------------------------------------------------

static bool ne_jitter_enabled = false;
static int ne_jitter_index = 0;
static int ne_jitter_count = 4;

// Offsets in f32 pixels. The default is the standard 4-sample rotated grid
// (+/- 1/8 and 3/8 of a pixel), which spreads the samples more evenly than an
// axis-aligned 2x2 box.
static int32_t ne_jitter_offsets[NEA_JITTER_MAX_SAMPLES * 2] = {
    -1536,  -512,   // -3/8, -1/8
      512, -1536,   //  1/8, -3/8
     1536,   512,   //  3/8,  1/8
     -512,  1536,   // -1/8,  3/8
};

void NEA_PostFXJitterEnable(bool enable)
{
    ne_jitter_enabled = enable;

    if (!enable)
        ne_jitter_index = 0;
}

bool NEA_PostFXJitterIsEnabled(void)
{
    return ne_jitter_enabled;
}

void NEA_PostFXJitterSetPattern(const int32_t *offsets_xy, int count)
{
    if (offsets_xy == NULL || count < 1)
        return;

    if (count > NEA_JITTER_MAX_SAMPLES)
        count = NEA_JITTER_MAX_SAMPLES;

    for (int i = 0; i < count * 2; i++)
        ne_jitter_offsets[i] = offsets_xy[i];

    ne_jitter_count = count;
    ne_jitter_index = 0;
}

// Called from ne_process_common() between loading the identity projection and
// multiplying in the frustum, so the translation ends up pre-multiplied.
void __NEA_PostFXApplyJitter(void)
{
    if (!ne_jitter_enabled)
        return;

    int32_t px = ne_jitter_offsets[ne_jitter_index * 2];
    int32_t py = ne_jitter_offsets[ne_jitter_index * 2 + 1];

    ne_jitter_index++;
    if (ne_jitter_index >= ne_jitter_count)
        ne_jitter_index = 0;

    // Pixels -> normalized device coordinates. The NDC range is 2 units across
    // 256 columns and 192 rows, so x scales by 1/128 (a shift) and y by 1/96
    // (a multiply by 1/96 in 16.16, which is 683). No division either way.
    int32_t nx = px >> 7;
    int32_t ny = (int32_t)(((int64_t)py * 683) >> 16);

    MATRIX_TRANSLATE = nx;
    MATRIX_TRANSLATE = ny;
    MATRIX_TRANSLATE = 0;
}
