// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Nitro Engine Advanced contributors
//
// This file is part of Nitro Engine Advanced

// DSP <-> ARM9 DMA: configuration, splitting and timing.
//
// Each transfer is done by nea_dma_in_chunk() / nea_dma_out_chunk(), which are
// hand-written assembly (nea_teak_dma_asm.s). A C version of them compiled to
// code that never moved a word on hardware, so this file only does what C can
// be trusted with here: 16-bit arithmetic to split a transfer into chunks that
// respect the chunk size, the 512-word limit and the 1 KB pages of ARM9 memory.

#include "nea_teak.h"

// Set by nea_dma_config(), read by the assembly
extern uint16_t nea_dma_xfer_speed;
extern uint16_t nea_dma_ahbm_burst;

uint16_t nea_dma_in_chunk(uint16_t dsp, uint16_t arm_lo, uint16_t arm_hi,
                          uint16_t len);
uint16_t nea_dma_out_chunk(uint16_t dsp, uint16_t arm_lo, uint16_t arm_hi,
                           uint16_t len);

static uint16_t xfer_chunk;     // Words

// Direction of transfer_split(): a static rather than a fifth argument, which
// the Teak compiler doesn't support
static uint16_t split_out;

void nea_dma_config(uint16_t xfer)
{
    nea_dma_xfer_speed = (uint16_t)(((xfer & NEA_DSP_XFER_SPEED_MASK)
                                     >> NEA_DSP_XFER_SPEED_SHIFT) << 12);

    switch (xfer & NEA_DSP_XFER_BURST_MASK)
    {
        case NEA_DSP_XFER_BURST_INCR4:
            nea_dma_ahbm_burst = AHBM_CH_CFG1_BURST_INCR4;
            break;
        case NEA_DSP_XFER_BURST_INCR8:
            nea_dma_ahbm_burst = AHBM_CH_CFG1_BURST_INCR8;
            break;
        default:
            nea_dma_ahbm_burst = AHBM_CH_CFG1_BURST_INCR;
            break;
    }

    xfer_chunk = nea_dsp_xfer_chunk_words(xfer);
}

void nea_timer_start(void)
{
    timerStart(0, TMR_CONTROL_PRESCALE_1 | TMR_CONTROL_MODE_ONCE
                  | TMR_CONTROL_UNPAUSE | TMR_CONTROL_UNFREEZE_COUNTER
                  | TMR_CONTROL_CLOCK_INTERNAL | TMR_CONTROL_AUTOCLEAR_OFF,
               0xFFFFFFFF);
}

uint32_t nea_timer_read(void)
{
    return timerRead(0);
}

void nea_timer_stop(void)
{
    timerStop(0);
}

void nea_gate_wait(void)
{
    if (!nea_gate_requested())
        return;

    apbpSetSemaphore(NEA_DSP_SEM_GATE_ACK);
    while (nea_gate_requested())
        ;
    apbpClearSemaphore(NEA_DSP_SEM_GATE_ACK);
}

// The ARM9 address travels as two 16-bit halves. A chunk never crosses a 1 KB
// page, so adding its length to the low half can only carry when the chunk
// ends exactly on a 64 KB boundary, which is when the low half wraps to 0.
static uint16_t transfer_split(uint16_t dsp, uint16_t arm_lo, uint16_t arm_hi,
                               uint16_t words)
{
    while (words > 0)
    {
        // Words until the next 1 KB boundary of ARM9 memory
        uint16_t n = (uint16_t)(512 - ((arm_lo & 1023) >> 1));
        if (n > xfer_chunk)
            n = xfer_chunk;
        if (n > words)
            n = words;

        nea_gate_wait();

        uint16_t status;
        if (split_out)
            status = nea_dma_out_chunk(dsp, arm_lo, arm_hi, n);
        else
            status = nea_dma_in_chunk(dsp, arm_lo, arm_hi, n);

        if (status != NEA_DSP_STATUS_OK)
            return status;

        dsp += n;
        arm_lo += (uint16_t)(n << 1);
        if (arm_lo == 0)
            arm_hi++;
        words -= n;
    }

    return NEA_DSP_STATUS_OK;
}

uint16_t nea_dma_in(uint16_t *dsp, uint32_t arm, uint16_t words)
{
    split_out = 0;
    return transfer_split((uint16_t)dsp, (uint16_t)arm,
                          (uint16_t)(arm >> 16), words);
}

uint16_t nea_dma_out(const uint16_t *dsp, uint32_t arm, uint16_t words)
{
    split_out = 1;
    return transfer_split((uint16_t)dsp, (uint16_t)arm,
                          (uint16_t)(arm >> 16), words);
}
