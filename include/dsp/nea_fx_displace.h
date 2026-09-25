// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Nitro Engine Advanced contributors
//
// This file is part of Nitro Engine Advanced

#ifndef NEA_FX_DISPLACE_H__
#define NEA_FX_DISPLACE_H__

/// @file nea_fx_displace.h
/// @brief Displacement tables, shared by the ARM9 and the DSP.
///
/// Each output pixel reads the source at an offset:
///
///     dst(x, y) = src(x + dx, y + dy)
///     dx = ax[x] + ay[y]      (|dx| <= NEA_FX_DISP_XMAX)
///     dy = bx[x] + by[y]      (|dy| <= NEA_FX_DISP_YMAX)
///
/// with the source coordinates clamped to the image. Sums of a per-column and a
/// per-row term are enough for heat haze and ripples (two crossed waves), and
/// they cost no multiplication per pixel on either processor.
///
/// The table blob holds, for an image of width W and height H:
///
///     XA[x] = x + NEA_FX_DISP_PAD + ax[x]
///     XB[x] = bx[x] + NEA_FX_DISP_YMAX
///     YA[y] = ay[y]
///     YB[y] = by[y]
///
/// so that the source column is XA[x] + YA[y] - PAD and the source row is
/// y + XB[x] + YB[y] - YMAX. On the DSP each source row is stored with PAD
/// copies of its edge pixels on both sides, which is the horizontal clamp.

// Largest offsets, in pixels
#define NEA_FX_DISP_XMAX        32
#define NEA_FX_DISP_YMAX        6

// Edge pixels stored on each side of a source row on the DSP
#define NEA_FX_DISP_PAD         32

// Largest image
#define NEA_FX_DISP_MAX_W       256
#define NEA_FX_DISP_MAX_H       192

// Word offsets inside a table blob
#define NEA_FX_DISP_XA          0
#define NEA_FX_DISP_XB          256
#define NEA_FX_DISP_YA          512
#define NEA_FX_DISP_YB          704
#define NEA_FX_DISP_WORDS       896

// DSP row ring: source rows kept at once, and the words each one takes
#define NEA_FX_DISP_RING_ROWS   16
#define NEA_FX_DISP_SLOT_WORDS  (NEA_FX_DISP_MAX_W + 2 * NEA_FX_DISP_PAD)

// Row address table used by the kernel: one entry per possible dy
#define NEA_FX_DISP_ROWS_SPAN   (2 * NEA_FX_DISP_YMAX + 1)

#ifndef __ASSEMBLER__

#include <stdint.h>

/// Displaces row y of an image with a table blob. Reference for the DSP
/// kernel: same arithmetic, same clamping.
///
/// The source rows are src_w pixels long and src_stride apart; a negative
/// stride reads them upwards (row y of the source is src + y * src_stride).
/// The output rows are w pixels long; XA maps each output column to a source
/// column, which is how the source can be wider (see NEA_DspDisplaceBuildScaled()).
static inline void nea_fx_displace_row(const uint16_t *blob, const uint16_t *src,
                                       int src_stride, int src_w, uint16_t *dst,
                                       int w, int h, int y)
{
    const int16_t *xa = (const int16_t *)(blob + NEA_FX_DISP_XA);
    const int16_t *xb = (const int16_t *)(blob + NEA_FX_DISP_XB);
    int ya = (int16_t)blob[NEA_FX_DISP_YA + y];
    int yb = (int16_t)blob[NEA_FX_DISP_YB + y];

    for (int x = 0; x < w; x++)
    {
        int sx = xa[x] + ya - NEA_FX_DISP_PAD;
        int sy = y + xb[x] + yb - NEA_FX_DISP_YMAX;

        if (sx < 0)
            sx = 0;
        else if (sx > src_w - 1)
            sx = src_w - 1;
        if (sy < 0)
            sy = 0;
        else if (sy > h - 1)
            sy = h - 1;

        dst[x] = src[sy * src_stride + sx];
    }
}

#endif // __ASSEMBLER__

#endif // NEA_FX_DISPLACE_H__
