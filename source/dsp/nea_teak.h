// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Nitro Engine Advanced contributors
//
// This file is part of Nitro Engine Advanced

#ifndef NEA_TEAK_H__
#define NEA_TEAK_H__

// Internal header of the DSP program. Teak only.
//
// Rules of the Teak C compiler that shape this code (libteak readme, section 5):
// no local arrays and no address of a local variable ("Cannot select:
// FrameIndex"), at most 4 function arguments, no calls through function
// pointers, no 64-bit integers, no division by a variable. So state lives in
// statics and the job dispatcher is a switch.

#include <teak/teak.h>

#include "dsp/nea_dsp_proto.h"

// DMA channel used for all transfers. Channel 0 belongs to the FIFO.
#define NEA_TEAK_DMA_CHANNEL    1

// The descriptor of the job being run, fetched from ARM9 memory.
extern uint16_t nea_desc[NEA_DSP_DESC_WORDS];

// Shared scratch buffer. Also the FIFO transport's window into DSP memory.
extern uint16_t nea_scratch[NEA_DSP_SCRATCH_WORDS];

static inline uint32_t nea_desc_u32(uint16_t index)
{
    return (uint32_t)nea_desc[index] | ((uint32_t)nea_desc[index + 1] << 16);
}

// DMA (nea_teak_dma.c)
// --------------------

// Sets the configuration (NEA_DSP_XFER_*) used by the transfers below.
void nea_dma_config(uint16_t xfer);

// Copies "words" words between DSP memory and ARM9 address "arm", split into
// chunks that respect the chunk size, the 512-word limit and the 1 KB pages of
// ARM9 memory. "arm" must be even. Return a NEA_DSP_STATUS_* value.
uint16_t nea_dma_in(uint16_t *dsp, uint32_t arm, uint16_t words);
uint16_t nea_dma_out(const uint16_t *dsp, uint32_t arm, uint16_t words);

// Asynchronous transfers (one at a time, same limits as the chunk functions:
// len <= 512, no 1 KB boundary of ARM9 memory crossed). Poll with
// nea_dma_poll(), which returns 1 when the transfer has finished.
void nea_dma_in_start(uint16_t dsp, uint16_t arm_lo, uint16_t arm_hi,
                      uint16_t len);
void nea_dma_out_start(uint16_t dsp, uint16_t arm_lo, uint16_t arm_hi,
                       uint16_t len);
uint16_t nea_dma_poll(void);
void nea_dma_stop(void);

// DMA gate, see NEA_DSP_SEM_GATE. Call with no transfer in flight: if the ARM9
// holds the gate, acknowledge and wait until it lets go.
void nea_gate_wait(void);

static inline uint16_t nea_gate_requested(void)
{
    return REG_APBP_SEM & NEA_DSP_SEM_GATE;
}

// Timer 0 counts down one step per DSP cycle.
void nea_timer_start(void);
uint32_t nea_timer_read(void);
void nea_timer_stop(void);

// Colour grading (nea_teak_grade_asm.s)
// -------------------------------------

extern uint16_t nea_grade_blob[];
void nea_grade_expand_asm(void);
void nea_grade_run_asm(uint16_t *buf, uint16_t count);

// Displacement (nea_teak_displace_asm.s)
// --------------------------------------

extern uint16_t nea_disp_blob[];
extern uint16_t nea_disp_ring[];
extern uint16_t nea_disp_out[];
extern uint16_t nea_disp_rowptr[];
extern int16_t nea_disp_ya, nea_disp_yb;
void nea_displace_row_asm(uint16_t *out, uint16_t count);

// Blur (nea_teak_blur_asm.s), working in the grading LRG table
// ------------------------------------------------------------

extern uint16_t nea_grade_lrg[];
extern uint16_t nea_blur_rows[3];
void nea_blur_h_asm(const uint16_t *src, uint16_t *planes, uint16_t count);
void nea_blur_v_asm(uint16_t *out, uint16_t count);

// Bloom (nea_teak_bloom_asm.s), working in the grading LRG table
// -------------------------------------------------------------

extern uint16_t nea_bloom_trb, nea_bloom_tg;
extern uint16_t nea_bloom_rows[5];
extern uint16_t nea_bloom_add_rb, nea_bloom_add_g;
void nea_bloom_down_asm(const uint16_t *row0, const uint16_t *row1,
                        uint16_t *out_rb, uint16_t count);
void nea_bloom_h5_asm(const uint16_t *in, uint16_t *out, uint16_t count,
                      uint16_t plane);
void nea_bloom_v5_asm(uint16_t *out, uint16_t count, uint16_t plane);
void nea_bloom_add_asm(const uint16_t *src, uint16_t *out, uint16_t count);

// Jobs (nea_teak_jobs.c)
// ----------------------

// Runs the job in nea_desc. Returns a status; the result goes to *nea_job_result.
uint16_t nea_job_run(uint16_t kind);
extern uint32_t nea_job_result;

// Whether the job measured its own cycles (the transfer bench counts only DMA
// time). Otherwise the dispatcher reports the whole job.
extern uint16_t nea_job_timed;
extern uint32_t nea_job_cycles;

#endif // NEA_TEAK_H__
