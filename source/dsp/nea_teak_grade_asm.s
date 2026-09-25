// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Nitro Engine Advanced contributors
//
// This file is part of Nitro Engine Advanced

// Colour grading kernel (same results as nea_fx_grade_pixel(), see
// include/dsp/nea_fx_grade.h for how the tables work).
//
// Calling convention: 16-bit arguments in a0l, a1l, b0l, b1l; y0, y1 and r0-r7
// preserved. Only forms listed as working on DSi hardware in libteak's
// dsp-bench readme are used: no multiplications, one memory operand per
// instruction, no [r7 + offset].

#include <teak/asminc.h>

#include "dsp/nea_fx_grade.h"

    .section .bss.nea_grade, "aw", %nobits
    .align 2
    .global nea_grade_blob
    .global nea_grade_lrg
nea_grade_blob: .space 2 * NEA_FX_GRADE_WORDS
nea_grade_lrg:  .space 2 * NEA_FX_GRADE_LRG_WORDS

// void nea_grade_expand_asm(void)
//
// LRG[g << 7 | r] = LR[r] | LG[g] << 5, for r, g in 0..127.

BEGIN_ASM_FUNC nea_grade_expand_asm

    push    r0
    push    r1
    push    r2

    mov     nea_grade_lrg, r0
    mov     nea_grade_blob + NEA_FX_GRADE_LG, r2

    bkrep   127, 2f
    mov     [r2++], a1              // LG[g] << 5
    shfi    a1, a1, 5
    mov     nea_grade_blob + NEA_FX_GRADE_LR, r1
    bkrep   127, 1f
    mov     [r1++], a0
    add     a1, a0
    mov     a0l, [r0++]
1:
    nop
2:

    pop     r2
    pop     r1
    pop     r0
    ret     always

// void nea_grade_run_asm(uint16_t *buf, uint16_t count)
//
// Grades "count" pixels in place (count >= 1). 24 instructions per pixel.

BEGIN_ASM_FUNC nea_grade_run_asm

    push    r0
    push    r1
    push    r2
    push    r3
    push    r4
    push    r6
    push    r7

    mov     a0l, r0                 // Pixels
    mov     a1l, b1l
    addv    0xffff, b1l             // "bkrep" runs its count + 1 times

    mov     nea_grade_blob + NEA_FX_GRADE_T1, r1
    mov     nea_grade_blob + NEA_FX_GRADE_T2, r2
    mov     nea_grade_lrg, r4
    mov     nea_grade_blob + NEA_FX_GRADE_LB, r6

    bkrep   b1l, 1f

    mov     [r0], a0                // p
    mov     a0, b0

    and     0x3ff, a0               // r3 = T1 + 2 * (r | g << 5)
    shfi    a0, a0, 1
    add     r1, a0
    mov     a0l, r3

    mov     b0, a0                  // r7 = T2 + 2 * b
    shfi    a0, a0, -10
    and     0x1f, a0
    shfi    a0, a0, 1
    add     r2, a0
    mov     a0l, r7

    mov     [r3++], a0              // A = T1.A + T2.A
    add     [r7++], a0
    mov     [r3], a1                // B = T1.B + T2.B
    add     [r7], a1

    and     0x3fff, a0              // LRG[A]
    add     r4, a0
    mov     a0l, r3
    add     r6, a1                  // LB[B]
    mov     a1l, r7
    mov     [r3], a0
    add     [r7], a0

    mov     a0l, [r0++]
1:

    pop     r7
    pop     r6
    pop     r4
    pop     r3
    pop     r2
    pop     r1
    pop     r0
    ret     always
