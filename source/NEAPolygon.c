// SPDX-License-Identifier: MIT
//
// Copyright (c) 2008-2022 Antonio Niño Díaz
//
// This file is part of Nitro Engine Advanced

#include "NEAMain.h"

/// @file NEAPolygon.c

void NEA_LightOff(int index)
{
    NEA_AssertMinMax(0, index, 3, "Invalid light index %d", index);

    GFX_LIGHT_VECTOR = (index & 3) << 30;
    GFX_LIGHT_COLOR = (index & 3) << 30;
}

void NEA_LightSetColor(int index, u32 color)
{
    NEA_AssertMinMax(0, index, 3, "Invalid light number %d", index);

    GFX_LIGHT_COLOR = ((index & 3) << 30) | color;
}

void NEA_LightSetI(int index, u32 color, int x, int y, int z)
{
    NEA_AssertMinMax(0, index, 3, "Invalid light number %d", index);

    GFX_LIGHT_VECTOR = ((index & 3) << 30)
                     | ((z & 0x3FF) << 20) | ((y & 0x3FF) << 10) | (x & 0x3FF);
    GFX_LIGHT_COLOR = ((index & 3) << 30) | color;
}

void NEA_ShininessTableGenerate(NEA_ShininessFunction function)
{
    uint32_t table[128 / 4];
    uint8_t *bytes = (uint8_t *)table;

    if (function == NEA_SHININESS_LINEAR)
    {
        for (int i = 0; i < 128; i++)
            bytes[i] = i * 2;
            //bytes[i] = 128 - i;
    }
    else if (function == NEA_SHININESS_QUADRATIC)
    {
        for (int i = 0; i < 128; i++)
        {
            int v = i * i;
            int div = 128;
            bytes[i] = v * 2 / div;
        }
    }
    else if (function == NEA_SHININESS_CUBIC)
    {
        for (int i = 0; i < 128; i++)
        {
            int v = i * i * i;
            int div = 128 * 128;
            bytes[i] = v * 2 / div;
        }
    }
    else if (function == NEA_SHININESS_QUARTIC)
    {
        for (int i = 0; i < 128; i++)
        {
            int v = i * i * i * i;
            int div = 128 * 128 * 128;
            bytes[i] = v * 2 / div;
        }
    }
    else
    {
        for (int i = 0; i < 128 / 4; i++)
            GFX_SHININESS = 0;

        return;
    }

    for (int i = 0; i < 128 / 4; i++)
        GFX_SHININESS = table[i];
}

void NEA_PolyFormat(u32 alpha, u32 id, NEA_LightEnum lights,
                   NEA_CullingEnum culling, NEA_OtherFormatEnum other)
{
    NEA_AssertMinMax(0, alpha, 31, "Invalid alpha value %lu", alpha);
    NEA_AssertMinMax(0, id, 63, "Invalid polygon ID %lu", id);

    GFX_POLY_FORMAT = POLY_ALPHA(alpha) | POLY_ID(id)
                    | lights | culling | other;
}

void NEA_OutliningSetColor(u32 index, u32 color)
{
    NEA_AssertMinMax(0, index, 7, "Invalaid outlining color index %lu", index);

    GFX_EDGE_TABLE[index] = color;
}

void NEA_SetupToonShadingTables(bool value)
{
    if (value)
    {
        for (int i = 0; i < 16; i++)
            GFX_TOON_TABLE[i] = RGB15(8, 8, 8);
        for (int i = 16; i < 32; i++)
            GFX_TOON_TABLE[i] = RGB15(24, 24, 24);
    }
    else
    {
        for (int i = 0; i < 32; i++)
            GFX_TOON_TABLE[i] = 0;
    }
}

void NEA_FogEnable(u32 shift, u32 color, u32 alpha, int mass, int depth)
{
    NEA_FogDisable();

    GFX_CONTROL |= GL_FOG | ((shift & 0xF) << 8);

    GFX_FOG_COLOR = color | ((alpha) << 16);
    GFX_FOG_OFFSET = depth;

    // Another option:
    //for (int i = 0; i < 32; i++)
    //    GFX_FOG_TABLE[i] = i << 2;

    // We need a 0 in the first fog table entry!
    int32_t density = -(mass << 1);

    for (int i = 0; i < 32; i++)
    {
        density += mass << 1;
        // Entries are 7 bit, so cap the density to 127
        density = ((density > 127) ? 127 : density);
        GFX_FOG_TABLE[i] = density;
    }
}

// Fog density curves
// ------------------
//
// The hardware fog unit (GBATEK "FOG_TABLE") has 32 density entries covering
// depth boundaries
//
//     FogDepthBoundary[n] = FOG_OFFSET + FOG_STEP * (n + 1)   , n = 0..31
//     FOG_STEP            = 0x400 >> FOG_SHIFT
//
// so the band covers FOG_OFFSET .. FOG_OFFSET + (0x8000 >> FOG_SHIFT). Density
// is linearly interpolated between entries, and the blend is
//
//     out = (fog * density + pixel * (128 - density)) / 128
//
// Entry 0 must stay at zero: GBATEK documents a hardware glitch where the fog
// alpha is treated as 0x1F in the region before the first density boundary,
// which is only invisible because density[0] is normally 0.
//
// None of the curve maths below divides. The tables are built once per
// configuration call, never per frame.

// Picks the largest FOG_SHIFT whose 32 bands still span `range` depth units.
// Larger shift means tighter bands, so this is the finest resolution that still
// reaches as far as the caller asked for.
static u32 ne_fog_shift_for_range(int range)
{
    if (range <= 0)
        return 10;

    // span(shift) = 0x8000 >> shift. Find the largest shift with span >= range.
    for (u32 shift = 10; shift > 0; shift--)
    {
        if ((0x8000 >> shift) >= (u32)range)
            return shift;
    }

    return 0;
}

static void ne_fog_build_table(NEA_FogCurve curve, u8 table[32])
{
    // Entry 0 is always zero, see the glitch note above.
    table[0] = 0;

    for (int i = 1; i < 32; i++)
    {
        int density;

        switch (curve)
        {
            case NEA_FOG_SQUARED:
            {
                // t^2, where t = i/31. 127/(31*31) = 0.13215 ~= 4331/32768.
                density = (i * i * 4331) >> 15;
                break;
            }
            case NEA_FOG_EXP:
            {
                // 1 - (7/8)^i, approached iteratively so there is no pow().
                // Dense close to the camera, which is what a smoke-filled or
                // dust-filled interior looks like.
                density = 0;
                for (int k = 0; k < i; k++)
                    density += (127 - density + 7) >> 3;
                break;
            }
            case NEA_FOG_SMOOTHSTEP:
            {
                // 3t^2 - 2t^3 in 1.12 fixed point. t = i/31 ~= i * 132 / 4096.
                int t  = i * 132;
                int t2 = (t * t) >> 12;
                int t3 = (t2 * t) >> 12;
                int s  = 3 * t2 - 2 * t3;
                density = (s * 127) >> 12;
                break;
            }
            case NEA_FOG_LINEAR:
            default:
            {
                // t, where t = i/31. 127/31 = 4.0968 ~= 4196/1024.
                density = (i * 4196) >> 10;
                break;
            }
        }

        // Entries are 7 bit
        if (density > 127)
            density = 127;
        if (density < 0)
            density = 0;

        table[i] = (u8)density;
    }
}

void NEA_FogEnableCurve(NEA_FogCurve curve, u32 color, u32 alpha,
                       u32 near_depth, u32 far_depth)
{
    NEA_AssertMinMax(0, alpha, 31, "Invalid alpha value %lu", alpha);

    if (far_depth <= near_depth)
    {
        NEA_DebugPrint("Fog far depth is not beyond near depth");
        return;
    }

    if (near_depth > 0x7FFF)
        near_depth = 0x7FFF;
    if (far_depth > 0x7FFF)
        far_depth = 0x7FFF;

    NEA_FogDisable();

    u32 shift = ne_fog_shift_for_range((int)(far_depth - near_depth));

    GFX_CONTROL |= GL_FOG | ((shift & 0xF) << 8);

    GFX_FOG_COLOR = color | (alpha << 16);
    GFX_FOG_OFFSET = near_depth;

    u8 table[32];
    ne_fog_build_table(curve, table);

    for (int i = 0; i < 32; i++)
        GFX_FOG_TABLE[i] = table[i];
}

// The GFX_CLEAR_COLOR register is write-only. This holds a copy of its value in
// order for the code to be able to modify individual fields.
static u32 ne_clearcolor = 0;

void NEA_FogEnableBackground(bool value)
{
    if (value)
        ne_clearcolor |= BIT(15);
    else
        ne_clearcolor &= ~BIT(15);

    GFX_CLEAR_COLOR = ne_clearcolor;
}

void NEA_ClearColorSet(u32 color, u32 alpha, u32 id)
{
    NEA_AssertMinMax(0, alpha, 31, "Invalid alpha value %lu", alpha);
    NEA_AssertMinMax(0, id, 63, "Invalid polygon ID %lu", id);

    ne_clearcolor &= BIT(15);

    ne_clearcolor |= 0x7FFF & color;
    ne_clearcolor |= (0x1F & alpha) << 16;
    ne_clearcolor |= (0x3F & id) << 24;

    GFX_CLEAR_COLOR = ne_clearcolor;
}

u32 NEA_ClearColorGet(void)
{
    return ne_clearcolor;
}

void NEA_ClearBMPEnable(bool value)
{
    if (NEA_CurrentExecutionMode() != NEA_ModeSingle3D)
    {
        // It needs two banks that are used for the display capture
        NEA_DebugPrint("Not available in dual 3D mode");
        return;
    }

    REG_CLRIMAGE_OFFSET = 0; // Set horizontal and vertical scroll to 0

    if (value)
    {
        vramSetBankC(VRAM_C_TEXTURE_SLOT2); // Slot 2 = clear color
        vramSetBankD(VRAM_D_TEXTURE_SLOT3); // Slot 3 = clear depth
        GFX_CONTROL |= GL_CLEAR_BMP;
    }
    else
    {
        GFX_CONTROL &= ~GL_CLEAR_BMP;
        // Let the user decide if they are used for textures or something else.
        //vramSetBankC(VRAM_C_LCD);
        //vramSetBankD(VRAM_D_LCD);
    }
}
