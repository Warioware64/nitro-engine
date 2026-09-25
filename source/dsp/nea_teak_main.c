// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Nitro Engine Advanced contributors
//
// This file is part of Nitro Engine Advanced

// Command loop of the DSP program (protocol: include/dsp/nea_dsp_proto.h).

#include "nea_teak.h"

uint16_t nea_desc[NEA_DSP_DESC_WORDS];
uint16_t nea_scratch[NEA_DSP_SCRATCH_WORDS];

static void reply(uint16_t rep0, uint32_t value)
{
    apbpSendData(0, rep0);
    apbpSendData(1, value >> 16);
    apbpSendData(2, value & 0xFFFF);
}

static void run_job(uint16_t seq, uint16_t kind, uint32_t desc_addr)
{
    // The descriptor always travels with the default transfer settings: the
    // settings being benchmarked are inside it.
    nea_dma_config(NEA_DSP_XFER_DEFAULT);

    nea_timer_start();
    uint32_t start = nea_timer_read();

    uint16_t status = NEA_DSP_STATUS_OK;
    if (desc_addr != 0)
        status = nea_dma_in(nea_desc, desc_addr, NEA_DSP_DESC_WORDS);

    if (status == NEA_DSP_STATUS_OK && nea_desc[NEA_DSP_D_KIND] != kind)
        status = NEA_DSP_STATUS_BAD_ARG;

    if (status == NEA_DSP_STATUS_OK)
    {
        nea_dma_config(nea_desc[NEA_DSP_D_XFER]);
        status = nea_job_run(kind);
    }

    uint32_t cycles = start - nea_timer_read();
    nea_timer_stop();

    if (nea_job_timed)
        cycles = nea_job_cycles;

    // No transfer happens until the next job, whatever the gate says
    apbpClearSemaphore(NEA_DSP_SEM_GATE_ACK | NEA_DSP_SEM_TABLES_READ);

    reply(seq | status, nea_job_result);
    reply(seq | NEA_DSP_REPLY_SECOND | status, cycles);
}

int main(void)
{
    teakInit();

    uint16_t last_seq = 0xFFFF;
    uint32_t repeated_cmds = 0;

    while (1)
    {
        uint16_t cmd = apbpReceiveData(0);
        uint16_t seq = cmd & NEA_DSP_SEQ_MASK;

        // The same command seen twice
        if (seq == last_seq)
        {
            repeated_cmds++;
            continue;
        }
        last_seq = seq;

        // Written by the ARM9 before CMD0. Read directly: waiting for their
        // "new" flags returned stale values on hardware.
        uint32_t arg = ((uint32_t)REG_APBP_CMD1 << 16) | REG_APBP_CMD2;

        uint16_t operand = cmd & NEA_DSP_OPERAND_MASK;

        switch (cmd & NEA_DSP_CMD_MASK)
        {
            case NEA_DSP_CMD_STATS:
                reply(seq | NEA_DSP_STATUS_OK, repeated_cmds);
                break;
            case NEA_DSP_CMD_PING:
                reply(seq | NEA_DSP_STATUS_OK, NEA_DSP_MAGIC);
                break;
            case NEA_DSP_CMD_INFO:
                if (operand == NEA_DSP_INFO_GRADE)
                    reply(seq | NEA_DSP_STATUS_OK, (uint16_t)nea_grade_blob);
                else if (operand == NEA_DSP_INFO_BLUR
                         || operand == NEA_DSP_INFO_BLOOM)
                    reply(seq | NEA_DSP_STATUS_OK, (uint16_t)nea_grade_lrg);
                else if (operand == NEA_DSP_INFO_DISPLACE)
                    reply(seq | NEA_DSP_STATUS_OK,
                          ((uint32_t)(uint16_t)nea_disp_ring << 16)
                          | (uint16_t)nea_disp_blob);
                else
                    reply(seq | NEA_DSP_STATUS_OK,
                          ((uint32_t)(uint16_t)nea_desc << 16)
                          | (uint16_t)nea_scratch);
                break;
            case NEA_DSP_CMD_JOB:
                run_job(seq, operand, arg);
                break;
            default:
                reply(seq | NEA_DSP_STATUS_BAD_CMD, 0);
                break;
        }
    }

    return 0;
}
