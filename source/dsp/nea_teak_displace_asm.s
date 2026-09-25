// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Nitro Engine Advanced contributors
//
// This file is part of Nitro Engine Advanced

// Displacement kernel (same results as nea_fx_displace_row(), see
// include/dsp/nea_fx_displace.h for the tables).
//
// Calling convention: 16-bit arguments in a0l, a1l, b0l, b1l; y0, y1 and r0-r7
// preserved. Only forms listed as working on DSi hardware in libteak's
// dsp-bench readme are used: no multiplications, one memory operand per
// instruction, no [r7 + offset].

#include <teak/asminc.h>

#include "dsp/nea_fx_displace.h"

    .section .bss.nea_disp, "aw", %nobits
    .align 2
    .global nea_disp_blob
    .global nea_disp_ring
    .global nea_disp_out
    .global nea_disp_rowptr
    .global nea_disp_ya
    .global nea_disp_yb
nea_disp_blob:      .space 2 * NEA_FX_DISP_WORDS
nea_disp_ring:      .space 2 * NEA_FX_DISP_RING_ROWS * NEA_FX_DISP_SLOT_WORDS
nea_disp_out:       .space 2 * 2 * NEA_FX_DISP_MAX_W
// Address of the ring slot of source row y - YMAX + k, clamped to the image
nea_disp_rowptr:    .space 2 * NEA_FX_DISP_ROWS_SPAN
// YA[y] and YB[y] of the row being displaced
nea_disp_ya:        .space 2
nea_disp_yb:        .space 2

// void nea_displace_row_asm(uint16_t *out, uint16_t count)
//
// Displaces one row of "count" pixels (count >= 1) into "out", from the ring
// slots in nea_disp_rowptr and the constants nea_disp_ya / nea_disp_yb. Per
// pixel: k = XB[x] + YB, then src = rowptr[k] + XA[x] + YA. 10 instructions.

BEGIN_ASM_FUNC nea_displace_row_asm

    push    r0
    push    r1
    push    r2
    push    r3
    push    r5

    mov     a0l, r0                 // Output
    mov     a1l, b1l                // Count - 1 ("bkrep" runs count + 1 times)
    addv    0xffff, b1l

    mov     nea_disp_blob + NEA_FX_DISP_XB, r1
    mov     nea_disp_blob + NEA_FX_DISP_XA, r2
    mov     nea_disp_rowptr, r5

    mov     [nea_disp_yb], a0       // YB in b0, sign-extended
    mov     a0, b0
    mov     [nea_disp_ya], a1       // YA in a1, sign-extended

    bkrep   b1l, 1f

    mov     [r1++], a0              // XB[x]
    add     b0, a0                  // + YB: k in 0 .. 2 * YMAX
    add     r5, a0                  // &rowptr[k]
    mov     a0l, r3
    mov     [r3], a0                // Source row slot
    add     [r2++], a0              // + XA[x]
    add     a1, a0                  // + YA
    mov     a0l, r3
    mov     [r3], a0                // Source pixel
    mov     a0l, [r0++]
1:

    pop     r5
    pop     r3
    pop     r2
    pop     r1
    pop     r0
    ret     always
