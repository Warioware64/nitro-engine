// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Nitro Engine Advanced contributors
//
// This file is part of Nitro Engine Advanced

// One DSP <-> ARM9 DMA transfer, in hand-written Teak assembly.
//
// The register sequence is libteak's dmaTransferArm9ToDsp() and
// dmaTransferDspToArm9() (source/dma.c), which work on hardware, written the
// way the compiler writes them there: direct 16-bit stores to the MMIO
// registers. Two things differ:
//
// - The DMA speed (nea_dma_xfer_speed, already in place for XFER_CONFIG) and
//   the AHBM burst type (nea_dma_ahbm_burst) come from nea_dma_config().
// - The wait for completion is bounded: on timeout the channel is stopped and
//   the function returns NEA_DSP_STATUS_DMA_ERROR instead of hanging.
//
// The caller guarantees 1 <= len <= 512 and that the ARM9 range doesn't cross
// a 1 KB boundary (nea_dma_in() / nea_dma_out() split transfers).
//
// Calling convention: 16-bit arguments in a0l, a1l, b0l, b1l; result in a0;
// y0, y1 and r0-r7 preserved.
//
// Registers (word addresses):
//
//   0x80E0 AHBM status (bit 2: busy)
//   0x80E2 + 6n  AHBM channel n: CFG1 (+0), CFG2 (+2), CFG_DMA (+4)
//   0x818C DMA DIM2 end flags (all done), one bit per channel
//   0x81BE DMA channel select
//   0x81C0 SRC_LO, 0x81C2 SRC_HI, 0x81C4 DST_LO, 0x81C6 DST_HI
//   0x81C8 DIM0_LEN, 0x81CA DIM1_LEN, 0x81CC DIM2_LEN
//   0x81CE DIM0 src step, 0x81D0 DIM0 dst step, 0x81D2..0x81D8 DIM1/DIM2 steps
//   0x81DA XFER_CONFIG, 0x81DC unknown (0x300), 0x81DE CONTROL

#include <teak/asminc.h>

// DMA channel 1: channel 0 belongs to the FIFO
#define DMA_CH_BIT          0x2

#define STATUS_OK           0
#define STATUS_DMA_ERROR    3

    .section .bss.nea_dma_cfg, "aw", %nobits
    .align 2
    .global nea_dma_xfer_speed
    .global nea_dma_ahbm_burst
nea_dma_xfer_speed: .space 2    // DMA_CH_XFER_SPEED_* (bits 12-13)
nea_dma_ahbm_burst: .space 2    // AHBM_CH_CFG1_BURST_* (bits 0-2)

// Waits until AHBM is idle, at most ~32K polls. Uses a0 and r4 (a1l, which
// holds arm_lo, is untouched).
.macro WAIT_AHBM_IDLE
    mov     0x80e0, r4
    mov     0x7fff, a0
1:
    tst0    0x4, [r4]
    brr     2f, eq
    dec     a0, always
    brr     1b, neq
2:
.endm

// Starts the transfer set up above.
.macro START
    mov     0x4000, a0              // DMA_CH_CONTROL_START
    mov     a0l, [0x81de]
.endm

// Waits for the transfer started by START (bounded) and returns the status in
// a0. "cfgdma" is the CFG_DMA register of the AHBM channel used.
.macro WAIT_DONE cfgdma
    mov     0x818c, r4
    mov     0x40, a0                // Outer count: 64 x 32767 polls
3:
    mov     0x7fff, a1
4:
    tst0    DMA_CH_BIT, [r4]
    brr     5f, neq                 // Channel done
    dec     a1, always
    brr     4b, neq
    dec     a0, always
    brr     3b, neq

    // Timed out: stop the channel
    mov     0x1, a0
    mov     a0l, [0x81be]
    mov     0x8000, a0l             // DMA_CH_CONTROL_STOP
    mov     a0l, [0x81de]
    mov     0x0, a0
    mov     a0l, [\cfgdma]
    mov     STATUS_DMA_ERROR, a0
    brr     6f, always
5:
    // The channel reports done when it has handed the last word to AHBM,
    // which may still be writing it out (much longer when the ARM9's own DMA
    // holds the bus). Disconnecting AHBM before it drains can leave it busy
    // for good, so wait for it first.
    WAIT_AHBM_IDLE
    mov     0x0, a0
    mov     a0l, [\cfgdma]          // Reset the AHBM channel
    mov     STATUS_OK, a0
6:
.endm

// Steps and dimensions shared by both directions. "srcstep" and "dststep" are
// the DIM0 steps: 2 on the ARM9 side (bytes), 1 on the DSP side (words).
.macro SET_DIMS srcstep, dststep
    mov     r3, a0l
    mov     a0l, [0x81c8]           // DIM0_LEN = len
    mov     0x1, a0
    mov     a0l, [0x81ca]           // DIM1_LEN
    mov     a0l, [0x81cc]           // DIM2_LEN
    mov     a0l, [0x81d2]
    mov     a0l, [0x81d4]
    mov     a0l, [0x81d6]
    mov     a0l, [0x81d8]
    mov     \srcstep, a0
    mov     a0l, [0x81ce]
    mov     \dststep, a0
    mov     a0l, [0x81d0]
.endm

.macro PROGRAM_NEA_DMA_IN_CHUNK
    WAIT_AHBM_IDLE

    mov     0x0, a0
    mov     a0l, [0x80ec]           // Reset AHBM channel 1

    mov     0x1, a0
    mov     a0l, [0x81be]           // Select DMA channel 1

    mov     a1l, [0x81c0]           // SRC = ARM9 address
    mov     r6, a0l
    mov     a0l, [0x81c2]
    mov     r5, a0l
    mov     a0l, [0x81c4]           // DST = DSP address
    mov     0x0, a0
    mov     a0l, [0x81c6]

    SET_DIMS 0x2, 0x1

    // XFER_CONFIG = SRC_ARM_AHBM | DST_DSP_DATA | RW_SIMULTANEOUS | speed
    mov     [nea_dma_xfer_speed], a0
    or      0x207, a0
    mov     a0l, [0x81da]
    mov     0x300, a0
    mov     a0l, [0x81dc]

    // AHBM channel 1: 16-bit + burst, read, connected to DMA channel 1
    mov     0x0, a0
    mov     a0l, [0x80ec]
    mov     [nea_dma_ahbm_burst], a0
    or      0x10, a0
    mov     a0l, [0x80e8]
    mov     0x200, a0
    mov     a0l, [0x80ea]
    mov     DMA_CH_BIT, a0
    mov     a0l, [0x80ec]

.endm

// uint16_t nea_dma_in_chunk(uint16_t dsp, uint16_t arm_lo, uint16_t arm_hi,
//                           uint16_t len)
//
// ARM9 -> DSP, AHBM channel 1.

BEGIN_ASM_FUNC nea_dma_in_chunk

    push    r3
    push    r4
    push    r5
    push    r6
    mov     a0l, r5                 // dsp
    mov     b0l, r6                 // arm_hi
    mov     b1l, r3                 // len

    PROGRAM_NEA_DMA_IN_CHUNK
    START
    WAIT_DONE 0x80ec

    pop     r6
    pop     r5
    pop     r4
    pop     r3
    ret     always

// void nea_dma_in_start(uint16_t dsp, uint16_t arm_lo, uint16_t arm_hi,
//                        uint16_t len)
//
// Same, but returns as soon as the transfer has started. Poll it with
// nea_dma_poll().

BEGIN_ASM_FUNC nea_dma_in_start

    push    r3
    push    r4
    push    r5
    push    r6
    mov     a0l, r5                 // dsp
    mov     b0l, r6                 // arm_hi
    mov     b1l, r3                 // len

    PROGRAM_NEA_DMA_IN_CHUNK
    START

    pop     r6
    pop     r5
    pop     r4
    pop     r3
    ret     always

.macro PROGRAM_NEA_DMA_OUT_CHUNK
    WAIT_AHBM_IDLE

    mov     0x0, a0
    mov     a0l, [0x80f2]           // Reset AHBM channel 2

    mov     0x1, a0
    mov     a0l, [0x81be]           // Select DMA channel 1

    mov     r5, a0l
    mov     a0l, [0x81c0]           // SRC = DSP address
    mov     0x0, a0
    mov     a0l, [0x81c2]
    mov     a1l, [0x81c4]           // DST = ARM9 address
    mov     r6, a0l
    mov     a0l, [0x81c6]

    SET_DIMS 0x1, 0x2

    // XFER_CONFIG = SRC_DSP_DATA | DST_ARM_AHBM | RW_SIMULTANEOUS | speed
    mov     [nea_dma_xfer_speed], a0
    or      0x270, a0
    mov     a0l, [0x81da]
    mov     0x300, a0
    mov     a0l, [0x81dc]

    // AHBM channel 2: 16-bit + burst, write, connected to DMA channel 1
    mov     0x0, a0
    mov     a0l, [0x80f2]
    mov     [nea_dma_ahbm_burst], a0
    or      0x10, a0
    mov     a0l, [0x80ee]
    mov     0x300, a0
    mov     a0l, [0x80f0]
    mov     DMA_CH_BIT, a0
    mov     a0l, [0x80f2]

.endm

// uint16_t nea_dma_out_chunk(uint16_t dsp, uint16_t arm_lo, uint16_t arm_hi,
//                            uint16_t len)
//
// DSP -> ARM9, AHBM channel 2.

BEGIN_ASM_FUNC nea_dma_out_chunk

    push    r3
    push    r4
    push    r5
    push    r6
    mov     a0l, r5                 // dsp
    mov     b0l, r6                 // arm_hi
    mov     b1l, r3                 // len

    PROGRAM_NEA_DMA_OUT_CHUNK
    START
    WAIT_DONE 0x80f2

    pop     r6
    pop     r5
    pop     r4
    pop     r3
    ret     always

// void nea_dma_out_start(uint16_t dsp, uint16_t arm_lo, uint16_t arm_hi,
//                        uint16_t len)
//
// Same, but returns as soon as the transfer has started. Poll it with
// nea_dma_poll().

BEGIN_ASM_FUNC nea_dma_out_start

    push    r3
    push    r4
    push    r5
    push    r6
    mov     a0l, r5                 // dsp
    mov     b0l, r6                 // arm_hi
    mov     b1l, r3                 // len

    PROGRAM_NEA_DMA_OUT_CHUNK
    START

    pop     r6
    pop     r5
    pop     r4
    pop     r3
    ret     always


// uint16_t nea_dma_poll(void)
//
// Returns 1 if the transfer started last has finished (then both AHBM channels
// are disconnected, as the synchronous functions do), 0 if it's still running.
// Finished means the DMA channel is done AND AHBM has drained, see WAIT_DONE.

BEGIN_ASM_FUNC nea_dma_poll

    push    r4
    mov     0x818c, r4
    tst0    DMA_CH_BIT, [r4]
    brr     1f, neq
2:
    mov     0x0, a0
    pop     r4
    ret     always
1:
    mov     0x80e0, r4
    tst0    0x4, [r4]               // AHBM still busy: not finished yet
    brr     2b, neq
    mov     0x0, a0
    mov     a0l, [0x80ec]
    mov     a0l, [0x80f2]
    mov     0x1, a0
    pop     r4
    ret     always

// void nea_dma_stop(void)
//
// Stops the transfer in progress, if any, and disconnects the AHBM channels.

BEGIN_ASM_FUNC nea_dma_stop

    mov     0x1, a0
    mov     a0l, [0x81be]
    mov     0x8000, a0l             // DMA_CH_CONTROL_STOP
    mov     a0l, [0x81de]
    mov     0x0, a0
    mov     a0l, [0x80ec]
    mov     a0l, [0x80f2]
    ret     always
