// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Nitro Engine Advanced contributors
//
// This file is part of Nitro Engine Advanced

#ifndef NEA_FX_GRADE_H__
#define NEA_FX_GRADE_H__

/// @file nea_fx_grade.h
/// @brief Colour grading tables, shared by the ARM9 and the DSP.
///
/// A grade is a 3x3 colour matrix with offsets, followed by one tone curve per
/// channel. Neither processor multiplies per pixel: the ARM9 folds the matrix
/// into two tables of packed contributions, and the curves into lookups.
///
/// For a pixel with 5-bit channels r, g, b:
///
///     T1[r | g << 5] = {A, B}     T2[b] = {A, B}
///     A = (T1.A + T2.A) & 0x3FFF  = R' | G' << 7
///     B =  T1.B + T2.B            = B'
///
/// R', G', B' are the matrix outputs rounded to whole 5-bit steps and biased
/// by +32, so that -32..95 maps to 0..127. The packed additions are exact as
/// long as each final field stays in 0..127, which NEA_DspGradeBuild() checks
/// for every input. Then:
///
///     out = LR[R'] | LG[G'] << 5 | LB[B']
///
/// where each curve table clamps its biased input back to 0..31 before
/// applying the curve, and LB also carries bit 15 (opaque).
///
/// On the DSP, LR and LG are expanded once per table generation into LRG, a
/// 16384-entry table indexed by A directly, which saves a lookup per pixel.
/// This header's reference gives bit-identical results without it.

// Word offsets inside a table blob
#define NEA_FX_GRADE_T1         0       ///< 1024 x {A, B}
#define NEA_FX_GRADE_T2         2048    ///< 32 x {A, B}
#define NEA_FX_GRADE_LR         2112    ///< 128 words: 0..31
#define NEA_FX_GRADE_LG         2240    ///< 128 words: 0..31 (unshifted)
#define NEA_FX_GRADE_LB         2368    ///< 128 words: value << 10 | 0x8000
#define NEA_FX_GRADE_WORDS      2496

/// Size of the DSP's expanded LR/LG table.
#define NEA_FX_GRADE_LRG_WORDS  16384

/// Bias of the packed fields: field = value + 32.
#define NEA_FX_GRADE_BIAS       32

// The assembly kernel includes this header for the constants above
#ifndef __ASSEMBLER__

#include <stdint.h>

/// Grades one pixel with a table blob. Reference for the DSP kernel.
static inline uint16_t nea_fx_grade_pixel(const uint16_t *blob, uint16_t p)
{
    const uint16_t *t1 = blob + NEA_FX_GRADE_T1 + 2 * (p & 0x3FF);
    const uint16_t *t2 = blob + NEA_FX_GRADE_T2 + 2 * ((p >> 10) & 0x1F);

    uint16_t a = (uint16_t)(t1[0] + t2[0]) & 0x3FFF;
    uint16_t b = (uint16_t)(t1[1] + t2[1]);

    return (uint16_t)(blob[NEA_FX_GRADE_LR + (a & 0x7F)]
                      | (blob[NEA_FX_GRADE_LG + (a >> 7)] << 5)
                      | blob[NEA_FX_GRADE_LB + b]);
}

#endif // __ASSEMBLER__

#endif // NEA_FX_GRADE_H__
