// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Nitro Engine Advanced contributors
//
// This file is part of Nitro Engine Advanced

#ifndef NEA_FX_BLOOM_H__
#define NEA_FX_BLOOM_H__

/// @file nea_fx_bloom.h
/// @brief Bloom arithmetic, shared by the ARM9 and the DSP.
///
/// For an image of W x H pixels (both even), at half size (W/2 x H/2):
///
/// 1. Bright-pass and downsample: each 2x2 block is averaged, then each channel
///    becomes max(0, c - threshold).
/// 2. Blur: [1 4 6 4 1] / 16 across, then down, edges clamped.
/// 3. Add: each pixel gets the bloom pixel of its 2x2 block added, per
///    channel, saturating at 31.
///
/// As for blur (nea_fx_blur.h), channels are handled as two planes:
/// rb = p & 0x7C1F and g = p & 0x03E0. A sum of weights up to 16 keeps R
/// below 512, inside the 5 free bits above it.
///
/// Subtracting with a floor at 0, and adding with a ceiling at 31, use a guard
/// bit above each field: bit 5 (R) and 15 (B) of the rb plane, bit 10 of the g
/// plane. m = v & guards; m - (m >> 5) turns each guard into a mask of its
/// field.

#include "dsp/nea_fx_blur.h"

#define NEA_FX_BLOOM_RB_GUARD   0x8020
#define NEA_FX_BLOOM_G_GUARD    0x0400
#define NEA_FX_BLOOM_RB_ROUND16 0x2008  // 8 in the R and B fields: /16 rounded
#define NEA_FX_BLOOM_G_ROUND16  0x0100  // 8 in the G field

#define NEA_FX_BLOOM_MAX_W      256
#define NEA_FX_BLOOM_MAX_H      192

// Rows the DSP keeps at once. A half row holds its rb plane row, then its g
// plane row, NEA_FX_BLOOM_MAX_W / 2 words apart.
#define NEA_FX_BLOOM_SRC_ROWS   8   ///< Full-size source rows
#define NEA_FX_BLOOM_HALF_ROWS  8   ///< Half-size rows, blurred across

// Layout of the DSP work area (the colour grading LRG table), in words
#define NEA_FX_BLOOM_WA_SRC     0
#define NEA_FX_BLOOM_WA_TMP     (NEA_FX_BLOOM_WA_SRC \
                                 + NEA_FX_BLOOM_SRC_ROWS * NEA_FX_BLOOM_MAX_W)
// Rows of the rb and g planes of one half row, each with 2 edge values on
// each side, NEA_FX_BLOOM_TMP_STRIDE words apart
#define NEA_FX_BLOOM_TMP_STRIDE (NEA_FX_BLOOM_MAX_W / 2 + 4)
#define NEA_FX_BLOOM_WA_HALF    (NEA_FX_BLOOM_WA_TMP \
                                 + 2 * NEA_FX_BLOOM_TMP_STRIDE)
#define NEA_FX_BLOOM_WA_BLOOM   (NEA_FX_BLOOM_WA_HALF \
                                 + NEA_FX_BLOOM_HALF_ROWS * NEA_FX_BLOOM_MAX_W)
#define NEA_FX_BLOOM_WA_OUT     (NEA_FX_BLOOM_WA_BLOOM + 2 * NEA_FX_BLOOM_MAX_W)
#define NEA_FX_BLOOM_WA_LOCAL_OUT (NEA_FX_BLOOM_WA_OUT + 2 * NEA_FX_BLOOM_MAX_W)

#ifndef __ASSEMBLER__

#include <stdint.h>

/// Bright-pass of one plane value: each field minus t, floored at 0.
static inline uint16_t nea_fx_bloom_bright(uint16_t v, uint16_t guard,
                                           uint16_t t)
{
    uint16_t d = (uint16_t)((v | guard) - t);
    uint16_t m = d & guard;
    m = (uint16_t)(m - (m >> 5));
    return d & m;
}

/// One plane value plus another, each field saturating at 31.
static inline uint16_t nea_fx_bloom_add(uint16_t a, uint16_t b, uint16_t guard,
                                        uint16_t mask)
{
    uint16_t s = (uint16_t)(a + b);
    uint16_t m = s & guard;
    m = (uint16_t)(m - (m >> 5));
    return (s | m) & mask;
}

/// Step 1 for half row k: the rb and g planes of W/2 pixels, as two rows.
static inline void nea_fx_bloom_down(const uint16_t *row0, const uint16_t *row1,
                                     uint16_t *rb_out, uint16_t *g_out,
                                     int half_w, int t)
{
    uint16_t t_rb = (uint16_t)(t | (t << 10));
    uint16_t t_g = (uint16_t)(t << 5);

    for (int x = 0; x < half_w; x++)
    {
        uint16_t a = row0[2 * x], b = row0[2 * x + 1];
        uint16_t c = row1[2 * x], d = row1[2 * x + 1];

        uint32_t rb = (a & 0x7C1F) + (b & 0x7C1F) + (c & 0x7C1F)
                      + (d & 0x7C1F) + NEA_FX_BLUR_RB_ROUND;
        uint32_t g = (a & 0x03E0) + (b & 0x03E0) + (c & 0x03E0)
                     + (d & 0x03E0) + NEA_FX_BLUR_G_ROUND;

        rb_out[x] = nea_fx_bloom_bright((rb >> 2) & 0x7C1F,
                                        NEA_FX_BLOOM_RB_GUARD, t_rb);
        g_out[x] = nea_fx_bloom_bright((g >> 2) & 0x03E0,
                                       NEA_FX_BLOOM_G_GUARD, t_g);
    }
}

// [1 4 6 4 1] of five plane values, rounded, masked
static inline uint16_t nea_fx_bloom_5(uint16_t a, uint16_t b, uint16_t c,
                                      uint16_t d, uint16_t e, uint16_t round,
                                      uint16_t mask)
{
    uint32_t s = a + e + ((uint32_t)(b + d) << 2) + ((uint32_t)c << 2)
                 + ((uint32_t)c << 1) + round;
    return (uint16_t)((s >> 4) & mask);
}

/// Step 2 across one plane row. "in" has 2 copies of its edge values on each
/// side: in[0], in[1] = in[2] and in[n+3], in[n+2] = in[n+1].
static inline void nea_fx_bloom_blur_h(const uint16_t *in, uint16_t *out,
                                       int n, uint16_t round, uint16_t mask)
{
    for (int x = 0; x < n; x++)
        out[x] = nea_fx_bloom_5(in[x], in[x + 1], in[x + 2], in[x + 3],
                                in[x + 4], round, mask);
}

/// Step 2 down: five plane rows (k-2 .. k+2, clamped) into one.
static inline void nea_fx_bloom_blur_v(const uint16_t *r0, const uint16_t *r1,
                                       const uint16_t *r2, const uint16_t *r3,
                                       const uint16_t *r4, uint16_t *out, int n,
                                       uint16_t round, uint16_t mask)
{
    for (int x = 0; x < n; x++)
        out[x] = nea_fx_bloom_5(r0[x], r1[x], r2[x], r3[x], r4[x], round,
                                mask);
}

/// Step 3 for one full row: src plus the bloom planes of its half row.
static inline void nea_fx_bloom_add_row(const uint16_t *src,
                                        const uint16_t *bloom_rb,
                                        const uint16_t *bloom_g, uint16_t *dst,
                                        int w)
{
    for (int x = 0; x < w; x++)
    {
        uint16_t p = src[x];
        uint16_t rb = nea_fx_bloom_add(p & 0x7C1F, bloom_rb[x >> 1],
                                       NEA_FX_BLOOM_RB_GUARD, 0x7C1F);
        uint16_t g = nea_fx_bloom_add(p & 0x03E0, bloom_g[x >> 1],
                                      NEA_FX_BLOOM_G_GUARD, 0x03E0);
        dst[x] = rb | g | 0x8000;
    }
}

#endif // __ASSEMBLER__

#endif // NEA_FX_BLOOM_H__
