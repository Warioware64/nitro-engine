// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Nitro Engine Advanced contributors
//
// This file is part of Nitro Engine Advanced

// Bloom kernels (same results as the functions of include/dsp/nea_fx_bloom.h).
//
// Calling convention: 16-bit arguments in a0l, a1l, b0l, b1l; y0, y1 and r0-r7
// preserved. Only forms listed as working on DSi hardware in libteak's
// dsp-bench readme are used: no multiplications, one memory operand per
// instruction, no [r7 + offset].
//
// A value loaded from memory is sign-extended, and an immediate with bit 15
// set may be too. So the guard bit 15 of the rb plane is added as 2 x 0x4000,
// and every value that goes through ">>" is known to be positive.

#include <teak/asminc.h>

#include "dsp/nea_fx_bloom.h"

    .section .bss.nea_bloom, "aw", %nobits
    .align 2
    .global nea_bloom_trb
    .global nea_bloom_tg
    .global nea_bloom_rows
    .global nea_bloom_add_rb
    .global nea_bloom_add_g
nea_bloom_trb:      .space 2    // Threshold in the R and B fields
nea_bloom_tg:       .space 2    // Threshold in the G field
nea_bloom_rows:     .space 2 * 5
nea_bloom_add_rb:   .space 2    // Bloom planes of the half row being added
nea_bloom_add_g:    .space 2

// Adds the rb and g planes of one source pixel at [\ptr] to a1 and b1 ("first"
// = 1 sets them instead). Uses a0.
.macro DOWN_PIXEL ptr, first
    mov     [\ptr], a0
    and     0x7c1f, a0
.if \first
    mov     a0, a1
.else
    add     a0, a1
.endif
    mov     [\ptr\()++], a0
    and     0x03e0, a0
.if \first
    mov     a0, b1
.else
    add     a0, b1
.endif
.endm

// a1 = a1 & (guard bits of a1, each widened to its field). Uses a0, b0.
.macro FLOOR_FIELDS guard
    mov     a1, a0
    and     \guard, a0
    shfi    a0, b0, -5
    sub     b0, a0
    and     a0, a1
.endm

// void nea_bloom_down_asm(const uint16_t *row0, const uint16_t *row1,
//                         uint16_t *out_rb, uint16_t count)
//
// Step 1 for "count" half pixels: 2x2 average, then bright-pass against
// nea_bloom_trb / nea_bloom_tg. The g plane goes NEA_FX_BLOOM_TMP_STRIDE words
// after out_rb.

BEGIN_ASM_FUNC nea_bloom_down_asm

    push    r0
    push    r1
    push    r3
    push    r5
    push    r6
    push    r7

    mov     a0l, r0
    mov     a1l, r1
    mov     b0l, r3
    mov     b0l, r5
    addv    NEA_FX_BLOOM_TMP_STRIDE, r5
    mov     [nea_bloom_trb], a0
    mov     a0l, r6
    mov     [nea_bloom_tg], a0
    mov     a0l, r7
    addv    0xffff, b1l             // "bkrep" runs its count + 1 times

    bkrep   b1l, 1f

    DOWN_PIXEL r0, 1
    DOWN_PIXEL r0, 0
    DOWN_PIXEL r1, 0
    DOWN_PIXEL r1, 0

    add     0x0802, a1              // rb: average, rounded
    shfi    a1, a1, -2
    and     0x7c1f, a1
    or      0x0020, a1              // Guards: bit 5, and bit 15 as 2 x 0x4000
    add     0x4000, a1
    add     0x4000, a1
    sub     r6, a1                  // Minus the threshold
    FLOOR_FIELDS 0x8020
    mov     a1l, [r3++]

    mov     b1, a1                  // g: same
    add     0x0040, a1
    shfi    a1, a1, -2
    and     0x03e0, a1
    or      0x0400, a1
    sub     r7, a1
    FLOOR_FIELDS 0x0400
    mov     a1l, [r5++]
1:

    pop     r7
    pop     r6
    pop     r5
    pop     r3
    pop     r1
    pop     r0
    ret     always

// One output of [1 4 6 4 1] / 16 from the five pointers r0, r1, r2, r4, r5
.macro TAP5 round, mask
    mov     [r0++], a0
    add     [r5++], a0
    mov     [r1++], a1
    add     [r4++], a1
    shfi    a1, a1, 2
    add     a1, a0
    mov     [r2++], a1
    shfi    a1, a1, 1
    add     a1, a0
    shfi    a1, a1, 1
    add     a1, a0
    add     \round, a0
    shfi    a0, a0, -4
    and     \mask, a0
    mov     a0l, [r3++]
.endm

// Runs TAP5 b0l + 1 times for the plane in b1l (0 = rb, else g)
.macro TAP5_LOOP
    cmpv    0x0, b1l
    brr     2f, neq
    bkrep   b0l, 1f
    TAP5    NEA_FX_BLOOM_RB_ROUND16, 0x7c1f
1:
    brr     3f, always
2:
    bkrep   b0l, 1f
    TAP5    NEA_FX_BLOOM_G_ROUND16, 0x03e0
1:
3:
.endm

// void nea_bloom_h5_asm(const uint16_t *in, uint16_t *out, uint16_t count,
//                       uint16_t plane)
//
// Step 2 across one plane row: out[x] = [1 4 6 4 1] of in[x .. x + 4] (in
// has 2 edge values on each side). 15 instructions per value.

BEGIN_ASM_FUNC nea_bloom_h5_asm

    push    r0
    push    r1
    push    r2
    push    r3
    push    r4
    push    r5

    mov     a0l, r0
    mov     a0l, r1
    modr    [r1++]
    mov     a0l, r2
    modr    [r2++]
    modr    [r2++]
    mov     a0l, r4
    addv    3, r4
    mov     a0l, r5
    addv    4, r5
    mov     a1l, r3
    addv    0xffff, b0l

    TAP5_LOOP

    pop     r5
    pop     r4
    pop     r3
    pop     r2
    pop     r1
    pop     r0
    ret     always

// void nea_bloom_v5_asm(uint16_t *out, uint16_t count, uint16_t plane)
//
// Step 2 down: out[x] = [1 4 6 4 1] of the five plane rows in nea_bloom_rows
// (k-2 .. k+2). 15 instructions per value.

BEGIN_ASM_FUNC nea_bloom_v5_asm

    push    r0
    push    r1
    push    r2
    push    r3
    push    r4
    push    r5

    mov     a0l, r3
    mov     b0l, b1l                // Plane
    mov     a1l, b0l                // Count
    mov     [nea_bloom_rows], a0
    mov     a0l, r0
    mov     [nea_bloom_rows + 1], a0
    mov     a0l, r1
    mov     [nea_bloom_rows + 2], a0
    mov     a0l, r2
    mov     [nea_bloom_rows + 3], a0
    mov     a0l, r4
    mov     [nea_bloom_rows + 4], a0
    mov     a0l, r5
    addv    0xffff, b0l

    TAP5_LOOP

    pop     r5
    pop     r4
    pop     r3
    pop     r2
    pop     r1
    pop     r0
    ret     always

// One output pixel: [r0++] plus the bloom planes at [r4] / [r5], each field
// saturating at 31, to [r3++]. Uses a0, a1, b0, b1.
.macro ADD_PIXEL
    mov     [r0++], a0
    mov     a0, b1                  // Source pixel

    and     0x7c1f, a0              // rb
    add     [r4], a0
    mov     a0, a1
    and     0x8020, a1
    shfi    a1, b0, -5
    sub     b0, a1
    or      a1, a0
    and     0x7c1f, a0
    mov     a0l, [r3]

    mov     b1, a0                  // g
    and     0x03e0, a0
    add     [r5], a0
    mov     a0, a1
    and     0x0400, a1
    shfi    a1, b0, -5
    sub     b0, a1
    or      a1, a0
    and     0x03e0, a0

    add     [r3], a0                // The planes don't overlap: add = or
    or      0x8000, a0
    mov     a0l, [r3++]
.endm

// void nea_bloom_add_asm(const uint16_t *src, uint16_t *out, uint16_t count)
//
// Step 3 for 2 * count pixels: each pair gets the bloom planes of one half
// pixel, from nea_bloom_add_rb / nea_bloom_add_g. 22 instructions per pixel.

BEGIN_ASM_FUNC nea_bloom_add_asm

    push    r0
    push    r3
    push    r4
    push    r5

    mov     a0l, r0
    mov     a1l, r3
    mov     [nea_bloom_add_rb], a0
    mov     a0l, r4
    mov     [nea_bloom_add_g], a0
    mov     a0l, r5
    addv    0xffff, b0l

    bkrep   b0l, 1f
    ADD_PIXEL
    ADD_PIXEL
    modr    [r4++]
    modr    [r5++]
1:

    pop     r5
    pop     r4
    pop     r3
    pop     r0
    ret     always
