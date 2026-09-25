// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Nitro Engine Advanced contributors
//
// This file is part of Nitro Engine Advanced

#ifndef NEA_FX_BLUR_H__
#define NEA_FX_BLUR_H__

/// @file nea_fx_blur.h
/// @brief Blur arithmetic, shared by the ARM9 and the DSP.
///
/// A separable [1 2 1] / 4 filter: horizontal, then vertical, each rounded,
/// with the image edges clamped. Applying it twice gives [1 4 6 4 1] / 16.
///
/// Channels are never unpacked. A pixel is split into two "planes":
///
///     rb = p & 0x7C1F     R in bits 0-4, B in bits 10-14
///     g  = p & 0x03E0     G in bits 5-9
///
/// A sum of weights up to 4 keeps R below 128, inside the 5 free bits above
/// it, so R never carries into B. Each plane is filtered with additions and
/// shifts, rounded, shifted back and masked; the planes are then recombined.

#define NEA_FX_BLUR_RB_MASK     0x7C1F
#define NEA_FX_BLUR_G_MASK      0x03E0
#define NEA_FX_BLUR_RB_ROUND    0x0802  // 2 in the R and B fields: /4 rounded
#define NEA_FX_BLUR_G_ROUND     0x0040  // 2 in the G field

#define NEA_FX_BLUR_MAX_W       256
#define NEA_FX_BLUR_MAX_H       192

// Horizontally blurred rows the DSP keeps at once, planes interleaved
// (rb, g, rb, g...): 2 words per pixel
#define NEA_FX_BLUR_RING_ROWS   4

// Layout of the DSP work area (the colour grading LRG table, which blur jobs
// borrow), in words
#define NEA_FX_BLUR_WA_IN       0       ///< Raw row + 1 edge pixel each side
#define NEA_FX_BLUR_WA_RING     260     ///< RING_ROWS rows of planes
#define NEA_FX_BLUR_WA_OUT      (NEA_FX_BLUR_WA_RING \
                                 + NEA_FX_BLUR_RING_ROWS * 2 * NEA_FX_BLUR_MAX_W)
#define NEA_FX_BLUR_WA_LOCAL_OUT (NEA_FX_BLUR_WA_OUT + 2 * NEA_FX_BLUR_MAX_W)
#define NEA_FX_BLUR_WA_LOCAL_PLANES (NEA_FX_BLUR_WA_LOCAL_OUT + 4096)

#ifndef __ASSEMBLER__

#include <stdint.h>

static inline uint16_t nea_fx_blur_rb(uint16_t a, uint16_t b, uint16_t c)
{
    uint32_t s = (a & NEA_FX_BLUR_RB_MASK) + ((b & NEA_FX_BLUR_RB_MASK) << 1)
                 + (c & NEA_FX_BLUR_RB_MASK) + NEA_FX_BLUR_RB_ROUND;
    return (uint16_t)((s >> 2) & NEA_FX_BLUR_RB_MASK);
}

static inline uint16_t nea_fx_blur_g(uint16_t a, uint16_t b, uint16_t c)
{
    uint32_t s = (a & NEA_FX_BLUR_G_MASK) + ((b & NEA_FX_BLUR_G_MASK) << 1)
                 + (c & NEA_FX_BLUR_G_MASK) + NEA_FX_BLUR_G_ROUND;
    return (uint16_t)((s >> 2) & NEA_FX_BLUR_G_MASK);
}

/// Horizontal pass of one row: planes of the blurred row, interleaved.
/// Reference for the DSP kernel.
static inline void nea_fx_blur_row_h(const uint16_t *src, uint16_t *planes,
                                     int w)
{
    for (int x = 0; x < w; x++)
    {
        uint16_t a = src[x > 0 ? x - 1 : 0];
        uint16_t b = src[x];
        uint16_t c = src[x < w - 1 ? x + 1 : w - 1];
        planes[2 * x] = nea_fx_blur_rb(a, b, c);
        planes[2 * x + 1] = nea_fx_blur_g(a, b, c);
    }
}

/// Vertical pass: combines three horizontally blurred rows (above, this one,
/// below) into a finished row. Bit 15 is set (opaque).
static inline void nea_fx_blur_row_v(const uint16_t *up, const uint16_t *mid,
                                     const uint16_t *down, uint16_t *dst,
                                     int w)
{
    for (int x = 0; x < w; x++)
    {
        uint16_t rb = nea_fx_blur_rb(up[2 * x], mid[2 * x], down[2 * x]);
        uint16_t g = nea_fx_blur_g(up[2 * x + 1], mid[2 * x + 1],
                                   down[2 * x + 1]);
        dst[x] = rb | g | 0x8000;
    }
}

#endif // __ASSEMBLER__

#endif // NEA_FX_BLUR_H__
