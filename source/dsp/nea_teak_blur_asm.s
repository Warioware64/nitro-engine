// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Nitro Engine Advanced contributors
//
// This file is part of Nitro Engine Advanced

// Blur kernels (same results as nea_fx_blur_row_h() / nea_fx_blur_row_v(),
// see include/dsp/nea_fx_blur.h).
//
// Calling convention: 16-bit arguments in a0l, a1l, b0l, b1l; y0, y1 and r0-r7
// preserved. Only forms listed as working on DSi hardware in libteak's
// dsp-bench readme are used: no multiplications, one memory operand per
// instruction, no [r7 + offset].

#include <teak/asminc.h>

#include "dsp/nea_fx_blur.h"

    .section .bss.nea_blur, "aw", %nobits
    .align 2
    .global nea_blur_rows
// Plane rows above, at and below the output row (see nea_blur_v_asm)
nea_blur_rows:  .space 2 * 3

// One plane of three raw pixels: a0 = (p0 + 2 p1 + p2 + round) / 4, masked.
// "inc" is "" or "++" for the pointer increment on the last plane.
.macro H_PLANE mask, round, inc
    mov     [r0\inc], a0
    and     \mask, a0
    mov     [r1\inc], a1
    and     \mask, a1
    shfi    a1, a1, 1
    add     a1, a0
    mov     [r2\inc], a1
    and     \mask, a1
    add     a1, a0
    add     \round, a0
    shfi    a0, a0, -2
    and     \mask, a0
    mov     a0l, [r3++]
.endm

// void nea_blur_h_asm(const uint16_t *src, uint16_t *planes, uint16_t count)
//
// Horizontal pass. src points at the pixel LEFT of the row's first pixel: the
// row is padded with one copy of its edge pixel on each side. Writes "count"
// pixels as interleaved planes (rb, g). 26 instructions per pixel.

BEGIN_ASM_FUNC nea_blur_h_asm

    push    r0
    push    r1
    push    r2
    push    r3

    mov     a0l, r0
    mov     a0l, r1
    modr    [r1++]
    mov     a0l, r2
    modr    [r2++]
    modr    [r2++]
    mov     a1l, r3                 // Planes
    addv    0xffff, b0l             // "bkrep" runs its count + 1 times

    bkrep   b0l, 1f
    H_PLANE 0x7c1f, 0x0802,
    H_PLANE 0x03e0, 0x0040, ++
1:

    pop     r3
    pop     r2
    pop     r1
    pop     r0
    ret     always

// One plane of three plane rows: a0 = (up + 2 mid + down + round) / 4, masked
.macro V_PLANE mask, round
    mov     [r0++], a0
    add     [r2++], a0
    mov     [r1++], a1
    shfi    a1, a1, 1
    add     a1, a0
    add     \round, a0
    shfi    a0, a0, -2
    and     \mask, a0
.endm

// void nea_blur_v_asm(uint16_t *out, uint16_t count)
//
// Vertical pass: combines the plane rows in nea_blur_rows (above, at, below)
// into "count" finished pixels, bit 15 set. 20 instructions per pixel.

BEGIN_ASM_FUNC nea_blur_v_asm

    push    r0
    push    r1
    push    r2
    push    r3

    mov     a0l, r3                 // Output
    mov     [nea_blur_rows], a0
    mov     a0l, r0
    mov     [nea_blur_rows + 1], a0
    mov     a0l, r1
    mov     [nea_blur_rows + 2], a0
    mov     a0l, r2
    mov     a1l, b0l
    addv    0xffff, b0l             // "bkrep" runs its count + 1 times

    bkrep   b0l, 1f
    V_PLANE 0x7c1f, 0x0802
    mov     a0, b1
    V_PLANE 0x03e0, 0x0040
    add     b1, a0                  // The planes don't overlap: add = or
    or      0x8000, a0
    mov     a0l, [r3++]
1:

    pop     r3
    pop     r2
    pop     r1
    pop     r0
    ret     always
