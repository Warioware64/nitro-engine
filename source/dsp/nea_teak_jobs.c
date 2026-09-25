// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Nitro Engine Advanced contributors
//
// This file is part of Nitro Engine Advanced

// Jobs of the DSP program. Each one reads its parameters from nea_desc.

#include "nea_teak.h"

#include "dsp/nea_fx_bloom.h"
#include "dsp/nea_fx_blur.h"
#include "dsp/nea_fx_displace.h"
#include "dsp/nea_fx_grade.h"

uint32_t nea_job_result;
uint16_t nea_job_timed;
uint32_t nea_job_cycles;

// Not a local: the Teak compiler cannot take the address of one
static nea_dsp_checksum sum;

// Reads D_WORDS words from A into the scratch buffer, returns their checksum,
// and writes each one XOR NEA_DSP_SELFTEST_XOR to B.
static uint16_t job_selftest(void)
{
    uint16_t words = nea_desc[NEA_DSP_D_WORDS];
    if (words == 0 || words > NEA_DSP_SCRATCH_WORDS)
        return NEA_DSP_STATUS_BAD_ARG;

    for (uint16_t i = 0; i < words; i++)
        nea_scratch[i] = 0;

    uint16_t status = nea_dma_in(nea_scratch, nea_desc_u32(NEA_DSP_D_ADDR_A),
                                 words);
    if (status != NEA_DSP_STATUS_OK)
        return status;

    nea_dsp_checksum_init(&sum);
    for (uint16_t i = 0; i < words; i++)
    {
        nea_dsp_checksum_add16(&sum, nea_scratch[i]);
        nea_scratch[i] ^= NEA_DSP_SELFTEST_XOR;
    }
    nea_job_result = nea_dsp_checksum_get(&sum);

    return nea_dma_out(nea_scratch, nea_desc_u32(NEA_DSP_D_ADDR_B), words);
}

// Moves D_WORDS words between A and the scratch buffer, which it treats as a
// ring. See NEA_DSP_JOB_XFER_BENCH.
static uint16_t job_xfer_bench(void)
{
    uint16_t words = nea_desc[NEA_DSP_D_WORDS];
    uint16_t out = nea_desc[NEA_DSP_XB_DIR] == NEA_DSP_XB_DIR_OUT;
    uint16_t seed = nea_desc[NEA_DSP_XB_SEED];
    uint32_t arm = nea_desc_u32(NEA_DSP_D_ADDR_A);

    if (words == 0)
        return NEA_DSP_STATUS_BAD_ARG;

    // The bench moves at most the whole scratch buffer per piece, so pieces
    // also never wrap inside a transfer.
    uint16_t piece = NEA_DSP_SCRATCH_WORDS;

    if (out)
    {
        // The pattern is written once, outside the timed part. Word i of the
        // stream is scratch[i % SCRATCH], so for streams longer than the
        // scratch buffer the pattern index must wrap the same way. That is
        // what the ARM9 checks against.
        for (uint16_t i = 0; i < NEA_DSP_SCRATCH_WORDS; i++)
            nea_scratch[i] = nea_dsp_bench_pattern(i, seed);
    }

    uint32_t cycles = 0;
    uint32_t mismatches = 0;
    uint16_t done = 0;
    while (done < words)
    {
        uint16_t n = words - done;
        if (n > piece)
            n = piece;

        uint16_t status;
        uint32_t t0 = nea_timer_read();
        if (out)
        {
            status = nea_dma_out(nea_scratch, arm, n);
        }
        else
        {
            // Clear first: a transfer that delivers nothing must not pass
            // because the buffer still holds the pattern from an earlier job.
            for (uint16_t i = 0; i < n; i++)
                nea_scratch[i] = 0;
            t0 = nea_timer_read();
            status = nea_dma_in(nea_scratch, arm, n);
        }
        cycles += t0 - nea_timer_read();

        if (status != NEA_DSP_STATUS_OK)
            return status;

        if (!out)
        {
            for (uint16_t i = 0; i < n; i++)
            {
                if (nea_scratch[i] != nea_dsp_bench_pattern(done + i, seed))
                    mismatches++;
            }
        }

        arm += (uint32_t)n * 2;
        done += n;
    }

    nea_job_result = mismatches;
    nea_job_timed = 1;
    nea_job_cycles = cycles;

    return mismatches ? NEA_DSP_STATUS_MISMATCH : NEA_DSP_STATUS_OK;
}

// Generations of the grading tables held in nea_grade_blob and expanded into
// nea_grade_lrg. 0 means none.
static uint16_t grade_loaded_gen;
static uint16_t grade_expanded_gen;

// Streaming: the kernel grades 128-pixel blocks of a ring (the scratch
// buffer) while the one DMA channel alternates between filling the ring from
// the source and draining graded pixels to the destination. A 512-pixel block
// costs ~100 us of kernel and ~90 us of DMA in and out (measured on a DSi), so
// one channel keeps up and the transfers disappear behind the kernel.
//
// Positions count pixels from the start of the image (at most 65535), and the
// ring holds position p at scratch[p % RING].

#define RING            NEA_DSP_SCRATCH_WORDS
#define RING_MASK       (RING - 1)
#define KERNEL_BLOCK    128
// Polls without progress before giving up (~0.1 s). A 512-word transfer takes
// ~45 us, but the ARM9 can hold the bus for milliseconds.
#define STALL_LIMIT     0x40000

static uint32_t st_src, st_dst;     // Start of the rectangle
static uint16_t st_count;           // Pixels, a whole number of rows
static uint16_t st_width;           // Pixels per row
static uint32_t st_src_skip;        // Bytes from the end of a row to the next
static uint32_t st_dst_skip;

// Next ARM9 address to read / write, and its column in the row. Kept as a
// walk rather than computed from the position, which would need a division.
static uint32_t st_in_addr, st_out_addr;
static uint16_t st_in_col, st_out_col;
static uint16_t st_in_issued, st_in_done, st_k_done, st_out_issued, st_out_done;
static uint16_t st_dma;         // 0 idle, 1 reading, 2 writing
static uint32_t st_last_addr;   // ARM9 address of the transfer in flight
static uint16_t st_last_len;
static uint32_t st_kernel;

// Words from ARM9 address "addr" to the next 1 KB boundary
static uint16_t page_words(uint32_t addr)
{
    return (uint16_t)(512 - (((uint16_t)addr & 1023) >> 1));
}

static uint16_t min16(uint16_t a, uint16_t b)
{
    return a < b ? a : b;
}

static void st_start_in(void)
{
    uint16_t pos = st_in_issued;
    uint16_t n = st_count - pos;
    n = min16(n, RING - (uint16_t)(pos - st_out_done));  // Free ring space
    n = min16(n, RING - (pos & RING_MASK));             // Up to the ring end
    n = min16(n, NEA_DSP_DMA_MAX_WORDS);
    n = min16(n, st_width - st_in_col);                 // Up to the row end

    uint32_t addr = st_in_addr;
    n = min16(n, page_words(addr));

    st_last_addr = addr;
    st_last_len = n;
    nea_dma_in_start((uint16_t)(nea_scratch + (pos & RING_MASK)),
                     (uint16_t)addr, (uint16_t)(addr >> 16), n);
    st_in_issued = pos + n;
    st_dma = 1;

    st_in_addr = addr + ((uint32_t)n << 1);
    st_in_col += n;
    if (st_in_col == st_width)
    {
        st_in_col = 0;
        st_in_addr += st_src_skip;
    }
}

static void st_start_out(void)
{
    uint16_t pos = st_out_issued;
    uint16_t n = st_k_done - pos;                       // Graded, not sent
    n = min16(n, RING - (pos & RING_MASK));
    n = min16(n, NEA_DSP_DMA_MAX_WORDS);
    n = min16(n, st_width - st_out_col);

    uint32_t addr = st_out_addr;
    n = min16(n, page_words(addr));

    st_last_addr = addr;
    st_last_len = n;
    nea_dma_out_start((uint16_t)(nea_scratch + (pos & RING_MASK)),
                      (uint16_t)addr, (uint16_t)(addr >> 16), n);
    st_out_issued = pos + n;
    st_dma = 2;

    st_out_addr = addr + ((uint32_t)n << 1);
    st_out_col += n;
    if (st_out_col == st_width)
    {
        st_out_col = 0;
        st_out_addr += st_dst_skip;
    }
}

static uint16_t grade_stream(void)
{
    st_in_issued = st_in_done = st_k_done = 0;
    st_out_issued = st_out_done = 0;
    st_dma = 0;
    st_kernel = 0;
    st_in_addr = st_src;
    st_out_addr = st_dst;
    st_in_col = st_out_col = 0;

    uint32_t stall = 0;
    uint16_t gated = 0;

    while (st_out_done < st_count)
    {
        uint16_t progress = 0;

        if (st_dma != 0 && nea_dma_poll())
        {
            if (st_dma == 1)
                st_in_done = st_in_issued;
            else
                st_out_done = st_out_issued;
            st_dma = 0;
            progress = 1;
        }

        if (st_dma == 0 && nea_gate_requested())
        {
            // The ARM9 is feeding the GFX FIFO by DMA: no transfer until it
            // is done. The kernel keeps going on what is already here.
            apbpSetSemaphore(NEA_DSP_SEM_GATE_ACK);
            gated = 1;
        }
        else if (st_dma == 0)
        {
            if (gated)
            {
                apbpClearSemaphore(NEA_DSP_SEM_GATE_ACK);
                gated = 0;
            }

            uint16_t backlog = st_k_done - st_out_issued;
            uint16_t can_in = st_in_issued < st_count
                              && (uint16_t)(st_in_issued - st_out_done) < RING;

            // Drain once a full transfer's worth is graded, or when reading
            // can't proceed; otherwise keep the kernel fed.
            if (backlog >= NEA_DSP_DMA_MAX_WORDS || (backlog > 0 && !can_in))
                st_start_out();
            else if (can_in)
                st_start_in();
        }

        if (st_k_done < st_in_done)
        {
            uint16_t pos = st_k_done;
            uint16_t m = st_in_done - pos;
            m = min16(m, KERNEL_BLOCK);
            m = min16(m, RING - (pos & RING_MASK));

            uint32_t t0 = nea_timer_read();
            nea_grade_run_asm(nea_scratch + (pos & RING_MASK), m);
            st_kernel += t0 - nea_timer_read();

            st_k_done = pos + m;
            progress = 1;
        }

        if (progress || gated)
        {
            stall = 0;  // Waiting at the gate is not a stall
        }
        else if (++stall > STALL_LIMIT)
        {
            // Where it stuck, for the ARM9 to report. Result: the ARM9
            // address of the transfer in flight. Second reply (instead of the
            // cycles): bits 0-15 = its position in the image, 16-17 = its
            // direction (1 reading, 2 writing), 18 = AHBM error, 19 = AHBM
            // busy, 20 = DMA channel done flag, 21-30 = its length.
            uint16_t ahbm = *(volatile uint16_t *)0x80E0;
            uint16_t end = *(volatile uint16_t *)0x818C;
            uint32_t detail = (st_dma == 1) ? st_in_done : st_out_done;
            detail |= (uint32_t)st_dma << 16;
            if (ahbm & 0x10)
                detail |= (uint32_t)1 << 18;
            if (ahbm & 0x04)
                detail |= (uint32_t)1 << 19;
            if (end & 0x02)
                detail |= (uint32_t)1 << 20;
            detail |= (uint32_t)st_last_len << 21;

            nea_dma_stop();
            nea_job_result = st_last_addr;
            nea_job_timed = 1;
            nea_job_cycles = detail;
            return NEA_DSP_STATUS_DMA_ERROR;
        }
    }

    nea_job_result = st_kernel;
    return NEA_DSP_STATUS_OK;
}

static uint16_t job_grade(void)
{
    uint16_t flags = nea_desc[NEA_DSP_D_FLAGS];
    uint16_t gen = nea_desc[NEA_DSP_GR_GEN];
    uint16_t count = nea_desc[NEA_DSP_D_WORDS];

    if (gen == 0 || count == 0)
        return NEA_DSP_STATUS_BAD_ARG;

    if (flags & NEA_DSP_FLAG_TABLES_LOCAL)
    {
        grade_loaded_gen = gen;
    }
    else if (gen != grade_loaded_gen)
    {
        grade_loaded_gen = 0;
        uint16_t status = nea_dma_in(nea_grade_blob,
                                     nea_desc_u32(NEA_DSP_GR_TABLES),
                                     NEA_FX_GRADE_WORDS);
        if (status != NEA_DSP_STATUS_OK)
            return status;
        grade_loaded_gen = gen;
    }

    // The ARM9 may rebuild its tables from now on
    apbpSetSemaphore(NEA_DSP_SEM_TABLES_READ);

    if (gen != grade_expanded_gen)
    {
        nea_grade_expand_asm();
        grade_expanded_gen = gen;
    }

    uint32_t kernel = 0;

    if (flags & NEA_DSP_FLAG_LOCAL)
    {
        if (count > NEA_DSP_SCRATCH_WORDS)
            return NEA_DSP_STATUS_BAD_ARG;

        uint32_t t0 = nea_timer_read();
        nea_grade_run_asm(nea_scratch, count);
        kernel = t0 - nea_timer_read();
    }
    else if (!(flags & NEA_DSP_FLAG_NO_OVERLAP))
    {
        uint16_t width = nea_desc[NEA_DSP_GR_WIDTH];
        uint16_t src_stride = nea_desc[NEA_DSP_GR_SRC_STRIDE];
        uint16_t dst_stride = nea_desc[NEA_DSP_GR_DST_STRIDE];

        if (width == 0 || src_stride < width || dst_stride < width)
            return NEA_DSP_STATUS_BAD_ARG;

        st_src = nea_desc_u32(NEA_DSP_D_ADDR_A);
        st_dst = nea_desc_u32(NEA_DSP_D_ADDR_B);
        st_count = count;
        st_width = width;
        st_src_skip = (uint32_t)(src_stride - width) << 1;
        st_dst_skip = (uint32_t)(dst_stride - width) << 1;
        return grade_stream();
    }
    else
    {
        // One row at a time is all this path does
        if (nea_desc[NEA_DSP_GR_WIDTH] != count)
            return NEA_DSP_STATUS_BAD_ARG;

        uint32_t src = nea_desc_u32(NEA_DSP_D_ADDR_A);
        uint32_t dst = nea_desc_u32(NEA_DSP_D_ADDR_B);
        uint16_t done = 0;

        while (done < count)
        {
            uint16_t n = count - done;
            if (n > NEA_DSP_SCRATCH_WORDS)
                n = NEA_DSP_SCRATCH_WORDS;

            uint16_t status = nea_dma_in(nea_scratch, src, n);
            if (status != NEA_DSP_STATUS_OK)
                return status;

            uint32_t t0 = nea_timer_read();
            nea_grade_run_asm(nea_scratch, n);
            kernel += t0 - nea_timer_read();

            status = nea_dma_out(nea_scratch, dst, n);
            if (status != NEA_DSP_STATUS_OK)
                return status;

            src += (uint32_t)n * 2;
            dst += (uint32_t)n * 2;
            done += n;
        }
    }

    nea_job_result = kernel;
    return NEA_DSP_STATUS_OK;
}

// Displacement
// ------------
//
// Source rows stream into a ring of NEA_FX_DISP_RING_ROWS slots, each with
// NEA_FX_DISP_PAD copies of its edge pixels on both sides (the horizontal
// clamp). Output row y needs source rows y - YMAX .. y + YMAX, so a row's slot
// is reused only once every output row that reads it is done. Output rows go
// to one of two row buffers and out with DMA. As for grading, the one DMA
// channel alternates between reading and writing while the kernel runs.

#define DP_RING         NEA_FX_DISP_RING_ROWS
#define DP_SLOT         NEA_FX_DISP_SLOT_WORDS
#define DP_PAD          NEA_FX_DISP_PAD
#define DP_YMAX         NEA_FX_DISP_YMAX

static uint16_t *dp_slot[DP_RING];  // Slot of source row r: dp_slot[r % RING]
static uint16_t dp_w, dp_h;         // Output size
static uint16_t dp_sw;              // Source row length
static uint32_t dp_src_skip;        // Bytes from the end of a row to the next
                                    // (two's complement when reading upwards)
static uint32_t dp_dst_skip;

static uint16_t dp_in_row, dp_in_col;   // Next source words to read
static uint32_t dp_in_addr;
static uint16_t dp_rows_in;             // Source rows read and padded
static uint16_t dp_next_y;              // Next output row to compute
static uint16_t dp_out_row, dp_out_col; // Next output words to write
static uint32_t dp_out_addr;
static uint16_t dp_rows_out;            // Output rows written
static uint16_t dp_dma;                 // 0 idle, 1 reading, 2 writing
static uint16_t dp_last_n;
static uint32_t dp_last_addr;
static uint32_t dp_kernel;

static void dp_pad_row(uint16_t *slot)
{
    uint16_t left = slot[DP_PAD];
    uint16_t right = slot[DP_PAD + dp_sw - 1];
    uint16_t *r = slot + DP_PAD + dp_sw;

    for (uint16_t i = 0; i < DP_PAD; i++)
    {
        slot[i] = left;
        r[i] = right;
    }
}

// Displaces output row y into "out", from the rows in the ring
static void dp_compute_row(uint16_t y, uint16_t *out)
{
    for (uint16_t k = 0; k < NEA_FX_DISP_ROWS_SPAN; k++)
    {
        int16_t r = (int16_t)(y + k) - DP_YMAX;
        if (r < 0)
            r = 0;
        else if (r > (int16_t)(dp_h - 1))
            r = dp_h - 1;
        nea_disp_rowptr[k] = (uint16_t)dp_slot[(uint16_t)r & (DP_RING - 1)];
    }

    nea_disp_ya = (int16_t)nea_disp_blob[NEA_FX_DISP_YA + y];
    nea_disp_yb = (int16_t)nea_disp_blob[NEA_FX_DISP_YB + y];

    uint32_t t0 = nea_timer_read();
    nea_displace_row_asm(out, dp_w);
    dp_kernel += t0 - nea_timer_read();
}

static uint16_t *dp_out_buffer(uint16_t y)
{
    return nea_disp_out + ((y & 1) ? NEA_FX_DISP_MAX_W : 0);
}

static void dp_start_in(void)
{
    uint16_t n = dp_sw - dp_in_col;
    n = min16(n, NEA_DSP_DMA_MAX_WORDS);
    n = min16(n, page_words(dp_in_addr));

    uint16_t *dst = dp_slot[dp_in_row & (DP_RING - 1)] + DP_PAD + dp_in_col;

    dp_last_addr = dp_in_addr;
    dp_last_n = n;
    nea_dma_in_start((uint16_t)dst, (uint16_t)dp_in_addr,
                     (uint16_t)(dp_in_addr >> 16), n);
    dp_dma = 1;
}

static void dp_start_out(void)
{
    uint16_t n = dp_w - dp_out_col;
    n = min16(n, NEA_DSP_DMA_MAX_WORDS);
    n = min16(n, page_words(dp_out_addr));

    uint16_t *src = dp_out_buffer(dp_out_row) + dp_out_col;

    dp_last_addr = dp_out_addr;
    dp_last_n = n;
    nea_dma_out_start((uint16_t)src, (uint16_t)dp_out_addr,
                      (uint16_t)(dp_out_addr >> 16), n);
    dp_dma = 2;
}

// The transfer in flight has finished: account for it
static void dp_transfer_done(void)
{
    uint16_t n = dp_last_n;

    if (dp_dma == 1)
    {
        dp_in_col += n;
        dp_in_addr += (uint32_t)n << 1;
        if (dp_in_col == dp_sw)
        {
            dp_pad_row(dp_slot[dp_in_row & (DP_RING - 1)]);
            dp_in_col = 0;
            dp_in_addr += dp_src_skip;
            dp_in_row++;
            dp_rows_in++;
        }
    }
    else
    {
        dp_out_col += n;
        dp_out_addr += (uint32_t)n << 1;
        if (dp_out_col == dp_w)
        {
            dp_out_col = 0;
            dp_out_addr += dp_dst_skip;
            dp_out_row++;
            dp_rows_out++;
        }
    }

    dp_dma = 0;
}

static uint16_t displace_stream(void)
{
    dp_in_row = dp_in_col = dp_rows_in = 0;
    dp_next_y = 0;
    dp_out_row = dp_out_col = dp_rows_out = 0;
    dp_dma = 0;
    dp_kernel = 0;

    uint32_t stall = 0;
    uint16_t gated = 0;

    while (dp_rows_out < dp_h)
    {
        uint16_t progress = 0;

        if (dp_dma != 0 && nea_dma_poll())
        {
            dp_transfer_done();
            progress = 1;
        }

        // Output row dp_next_y needs source rows up to y + YMAX (clamped)
        uint16_t need = dp_next_y + DP_YMAX;
        if (need > dp_h - 1)
            need = dp_h - 1;
        uint16_t inputs_ready = dp_next_y < dp_h && dp_rows_in > need;

        if (dp_dma == 0 && nea_gate_requested())
        {
            apbpSetSemaphore(NEA_DSP_SEM_GATE_ACK);
            gated = 1;
        }
        else if (dp_dma == 0)
        {
            if (gated)
            {
                apbpClearSemaphore(NEA_DSP_SEM_GATE_ACK);
                gated = 0;
            }

            // A row's slot is free once no output row left to compute reads
            // the row that was in it before
            uint16_t can_in = dp_in_row < dp_h
                              && dp_in_row + DP_YMAX < dp_next_y + DP_RING;
            uint16_t can_out = dp_out_row < dp_next_y;

            if (can_in && (!inputs_ready || !can_out))
                dp_start_in();
            else if (can_out)
                dp_start_out();
        }

        // Its output buffer is free once row y - 2 has gone out
        if (inputs_ready && dp_next_y < dp_rows_out + 2)
        {
            dp_compute_row(dp_next_y, dp_out_buffer(dp_next_y));
            dp_next_y++;
            progress = 1;
        }

        if (progress || gated)
        {
            stall = 0;
        }
        else if (++stall > STALL_LIMIT)
        {
            // Same report as grading, see job_grade
            uint16_t ahbm = *(volatile uint16_t *)0x80E0;
            uint16_t end = *(volatile uint16_t *)0x818C;
            uint32_t detail = (dp_dma == 1) ? dp_rows_in : dp_rows_out;
            detail |= (uint32_t)dp_dma << 16;
            if (ahbm & 0x10)
                detail |= (uint32_t)1 << 18;
            if (ahbm & 0x04)
                detail |= (uint32_t)1 << 19;
            if (end & 0x02)
                detail |= (uint32_t)1 << 20;
            detail |= (uint32_t)dp_last_n << 21;

            nea_dma_stop();
            nea_job_result = dp_last_addr;
            nea_job_timed = 1;
            nea_job_cycles = detail;
            return NEA_DSP_STATUS_DMA_ERROR;
        }
    }

    nea_job_result = dp_kernel;
    return NEA_DSP_STATUS_OK;
}

static uint16_t job_displace(void)
{
    uint16_t flags = nea_desc[NEA_DSP_D_FLAGS];
    dp_w = nea_desc[NEA_DSP_D_WORDS];
    dp_h = nea_desc[NEA_DSP_DP_HEIGHT];
    dp_sw = nea_desc[NEA_DSP_DP_SRC_W];

    if (dp_w == 0 || dp_w > NEA_FX_DISP_MAX_W || dp_h == 0
        || dp_h > NEA_FX_DISP_MAX_H || dp_sw == 0
        || dp_sw > NEA_FX_DISP_MAX_W)
        return NEA_DSP_STATUS_BAD_ARG;

    uint16_t *slot = nea_disp_ring;
    for (uint16_t i = 0; i < DP_RING; i++)
    {
        dp_slot[i] = slot;
        slot += DP_SLOT;
    }

    if (flags & NEA_DSP_FLAG_LOCAL)
    {
        // Rows 0 .. h-1 are in slots 0 .. h-1, the result goes to scratch
        if (dp_h > DP_RING || (uint32_t)dp_w * dp_h > NEA_DSP_SCRATCH_WORDS)
            return NEA_DSP_STATUS_BAD_ARG;

        // The ARM9 wrote the rows in order, whatever the stride

        for (uint16_t r = 0; r < dp_h; r++)
            dp_pad_row(dp_slot[r]);

        dp_kernel = 0;
        uint16_t *out = nea_scratch;
        for (uint16_t y = 0; y < dp_h; y++)
        {
            dp_compute_row(y, out);
            out += dp_w;
        }

        nea_job_result = dp_kernel;
        return NEA_DSP_STATUS_OK;
    }

    uint16_t status = nea_dma_in(nea_disp_blob,
                                 nea_desc_u32(NEA_DSP_DP_TABLES),
                                 NEA_FX_DISP_WORDS);
    if (status != NEA_DSP_STATUS_OK)
        return status;

    // The ARM9 may rebuild its tables from now on
    apbpSetSemaphore(NEA_DSP_SEM_TABLES_READ);

    int16_t src_stride = (int16_t)nea_desc[NEA_DSP_DP_SRC_STRIDE];
    uint16_t dst_stride = nea_desc[NEA_DSP_DP_DST_STRIDE];
    int16_t src_len = src_stride < 0 ? -src_stride : src_stride;
    if (src_len < (int16_t)dp_sw || dst_stride < dp_w)
        return NEA_DSP_STATUS_BAD_ARG;

    dp_in_addr = nea_desc_u32(NEA_DSP_D_ADDR_A);
    dp_out_addr = nea_desc_u32(NEA_DSP_D_ADDR_B);
    // After a row, the address is at its end: step to the next row's start,
    // which is behind it when reading upwards. Unsigned arithmetic wraps to
    // the same result as a signed one.
    dp_src_skip = (uint32_t)(int32_t)(src_stride - (int16_t)dp_sw) << 1;
    dp_dst_skip = (uint32_t)(dst_stride - dp_w) << 1;

    return displace_stream();
}

// Blur
// ----
//
// Each source row is read into a padded row buffer, blurred horizontally into
// a ring of RING_ROWS rows of planes, and output row y is blurred vertically
// from ring rows y-1, y, y+1 into one of two output row buffers, which go out
// with DMA. Same scheduling as displacement.

#define BL_RING         NEA_FX_BLUR_RING_ROWS
#define BL_PLANES       (2 * NEA_FX_BLUR_MAX_W)

static uint16_t *bl_in, *bl_out;
static uint16_t *bl_slot[BL_RING];
static uint16_t bl_w, bl_h;
static uint32_t bl_src_skip, bl_dst_skip;

static uint16_t bl_in_row, bl_in_col;
static uint32_t bl_in_addr;
static uint16_t bl_rows_in;
static uint16_t bl_next_y;
static uint16_t bl_out_row, bl_out_col;
static uint32_t bl_out_addr;
static uint16_t bl_rows_out;
static uint16_t bl_dma;
static uint16_t bl_last_n;
static uint32_t bl_last_addr;
static uint32_t bl_kernel;

// Pads the raw row in bl_in and blurs it horizontally into "planes"
static void bl_row_h(uint16_t *planes)
{
    bl_in[0] = bl_in[1];
    bl_in[bl_w + 1] = bl_in[bl_w];

    uint32_t t0 = nea_timer_read();
    nea_blur_h_asm(bl_in, planes, bl_w);
    bl_kernel += t0 - nea_timer_read();
}

// Blurs output row y vertically into "out" from the given plane rows
static void bl_row_v(uint16_t *up, uint16_t *mid, uint16_t *down,
                     uint16_t *out)
{
    nea_blur_rows[0] = (uint16_t)up;
    nea_blur_rows[1] = (uint16_t)mid;
    nea_blur_rows[2] = (uint16_t)down;

    uint32_t t0 = nea_timer_read();
    nea_blur_v_asm(out, bl_w);
    bl_kernel += t0 - nea_timer_read();
}

static uint16_t *bl_ring_row(uint16_t r)
{
    return bl_slot[r & (BL_RING - 1)];
}

static uint16_t *bl_out_buffer(uint16_t y)
{
    return bl_out + ((y & 1) ? NEA_FX_BLUR_MAX_W : 0);
}

static void bl_start_in(void)
{
    uint16_t n = bl_w - bl_in_col;
    n = min16(n, NEA_DSP_DMA_MAX_WORDS);
    n = min16(n, page_words(bl_in_addr));

    bl_last_addr = bl_in_addr;
    bl_last_n = n;
    nea_dma_in_start((uint16_t)(bl_in + 1 + bl_in_col), (uint16_t)bl_in_addr,
                     (uint16_t)(bl_in_addr >> 16), n);
    bl_dma = 1;
}

static void bl_start_out(void)
{
    uint16_t n = bl_w - bl_out_col;
    n = min16(n, NEA_DSP_DMA_MAX_WORDS);
    n = min16(n, page_words(bl_out_addr));

    bl_last_addr = bl_out_addr;
    bl_last_n = n;
    nea_dma_out_start((uint16_t)(bl_out_buffer(bl_out_row) + bl_out_col),
                      (uint16_t)bl_out_addr, (uint16_t)(bl_out_addr >> 16), n);
    bl_dma = 2;
}

static void bl_transfer_done(void)
{
    uint16_t n = bl_last_n;

    if (bl_dma == 1)
    {
        bl_in_col += n;
        bl_in_addr += (uint32_t)n << 1;
        if (bl_in_col == bl_w)
        {
            bl_row_h(bl_ring_row(bl_in_row));
            bl_in_col = 0;
            bl_in_addr += bl_src_skip;
            bl_in_row++;
            bl_rows_in++;
        }
    }
    else
    {
        bl_out_col += n;
        bl_out_addr += (uint32_t)n << 1;
        if (bl_out_col == bl_w)
        {
            bl_out_col = 0;
            bl_out_addr += bl_dst_skip;
            bl_out_row++;
            bl_rows_out++;
        }
    }

    bl_dma = 0;
}

static uint16_t blur_stream(void)
{
    bl_in_row = bl_in_col = bl_rows_in = 0;
    bl_next_y = 0;
    bl_out_row = bl_out_col = bl_rows_out = 0;
    bl_dma = 0;

    uint32_t stall = 0;
    uint16_t gated = 0;

    while (bl_rows_out < bl_h)
    {
        uint16_t progress = 0;

        if (bl_dma != 0 && nea_dma_poll())
        {
            bl_transfer_done();
            progress = 1;
        }

        // Output row y needs source rows up to y + 1 (clamped)
        uint16_t need = bl_next_y + 1;
        if (need > bl_h - 1)
            need = bl_h - 1;
        uint16_t inputs_ready = bl_next_y < bl_h && bl_rows_in > need;

        if (bl_dma == 0 && nea_gate_requested())
        {
            apbpSetSemaphore(NEA_DSP_SEM_GATE_ACK);
            gated = 1;
        }
        else if (bl_dma == 0)
        {
            if (gated)
            {
                apbpClearSemaphore(NEA_DSP_SEM_GATE_ACK);
                gated = 0;
            }

            // A ring slot is free once no output row left reads its old row
            uint16_t can_in = bl_in_row < bl_h && bl_in_row < bl_next_y + 3;
            uint16_t can_out = bl_out_row < bl_next_y;

            if (can_in && (!inputs_ready || !can_out))
                bl_start_in();
            else if (can_out)
                bl_start_out();
        }

        if (inputs_ready && bl_next_y < bl_rows_out + 2)
        {
            uint16_t y = bl_next_y;
            uint16_t up = y > 0 ? y - 1 : 0;
            uint16_t down = y < bl_h - 1 ? y + 1 : bl_h - 1;
            bl_row_v(bl_ring_row(up), bl_ring_row(y), bl_ring_row(down),
                     bl_out_buffer(y));
            bl_next_y++;
            progress = 1;
        }

        if (progress || gated)
        {
            stall = 0;
        }
        else if (++stall > STALL_LIMIT)
        {
            // Same report as grading, see job_grade
            uint16_t ahbm = *(volatile uint16_t *)0x80E0;
            uint16_t end = *(volatile uint16_t *)0x818C;
            uint32_t detail = (bl_dma == 1) ? bl_rows_in : bl_rows_out;
            detail |= (uint32_t)bl_dma << 16;
            if (ahbm & 0x10)
                detail |= (uint32_t)1 << 18;
            if (ahbm & 0x04)
                detail |= (uint32_t)1 << 19;
            if (end & 0x02)
                detail |= (uint32_t)1 << 20;
            detail |= (uint32_t)bl_last_n << 21;

            nea_dma_stop();
            nea_job_result = bl_last_addr;
            nea_job_timed = 1;
            nea_job_cycles = detail;
            return NEA_DSP_STATUS_DMA_ERROR;
        }
    }

    nea_job_result = bl_kernel;
    return NEA_DSP_STATUS_OK;
}

static uint16_t job_blur(void)
{
    uint16_t flags = nea_desc[NEA_DSP_D_FLAGS];
    bl_w = nea_desc[NEA_DSP_D_WORDS];
    bl_h = nea_desc[NEA_DSP_BL_HEIGHT];

    if (bl_w == 0 || bl_w > NEA_FX_BLUR_MAX_W || bl_h == 0
        || bl_h > NEA_FX_BLUR_MAX_H)
        return NEA_DSP_STATUS_BAD_ARG;

    // The work area is the grading LRG table: the next grading job rebuilds it
    grade_expanded_gen = 0;

    uint16_t *wa = nea_grade_lrg;
    bl_in = wa + NEA_FX_BLUR_WA_IN;
    bl_out = wa + NEA_FX_BLUR_WA_OUT;
    uint16_t *slot = wa + NEA_FX_BLUR_WA_RING;
    for (uint16_t i = 0; i < BL_RING; i++)
    {
        bl_slot[i] = slot;
        slot += BL_PLANES;
    }

    bl_kernel = 0;

    if (flags & NEA_DSP_FLAG_LOCAL)
    {
        if ((uint32_t)bl_w * bl_h > NEA_DSP_SCRATCH_WORDS)
            return NEA_DSP_STATUS_BAD_ARG;

        // Every row blurred horizontally first, then vertically
        uint16_t *planes = wa + NEA_FX_BLUR_WA_LOCAL_PLANES;
        uint16_t *src = nea_scratch;
        uint16_t *p = planes;
        for (uint16_t y = 0; y < bl_h; y++)
        {
            for (uint16_t x = 0; x < bl_w; x++)
                bl_in[1 + x] = src[x];
            bl_row_h(p);
            src += bl_w;
            p += 2 * bl_w;
        }

        uint16_t *out = wa + NEA_FX_BLUR_WA_LOCAL_OUT;
        uint16_t *up = planes;
        uint16_t *mid = planes;
        for (uint16_t y = 0; y < bl_h; y++)
        {
            uint16_t *down = (y < bl_h - 1) ? mid + 2 * bl_w : mid;
            bl_row_v(up, mid, down, out);
            up = mid;
            mid = down;
            out += bl_w;
        }

        nea_job_result = bl_kernel;
        return NEA_DSP_STATUS_OK;
    }

    uint16_t src_stride = nea_desc[NEA_DSP_BL_SRC_STRIDE];
    uint16_t dst_stride = nea_desc[NEA_DSP_BL_DST_STRIDE];
    if (src_stride < bl_w || dst_stride < bl_w)
        return NEA_DSP_STATUS_BAD_ARG;

    bl_in_addr = nea_desc_u32(NEA_DSP_D_ADDR_A);
    bl_out_addr = nea_desc_u32(NEA_DSP_D_ADDR_B);
    bl_src_skip = (uint32_t)(src_stride - bl_w) << 1;
    bl_dst_skip = (uint32_t)(dst_stride - bl_w) << 1;

    return blur_stream();
}

// Bloom
// -----
//
// Source rows stream into a ring of SRC_ROWS rows. Each pair of rows becomes
// one half row (step 1, into a padded temporary row, then blurred across into
// a ring of HALF_ROWS half rows), five half rows make one bloom row (blurred
// down, into one of two bloom rows), and each source row plus its bloom row
// makes an output row (step 3), which goes out with DMA.

#define BM_SRC          NEA_FX_BLOOM_SRC_ROWS
#define BM_HALF         NEA_FX_BLOOM_HALF_ROWS
#define BM_PLANE_G      (NEA_FX_BLOOM_MAX_W / 2)   // g row after the rb row
#define BM_TMP          NEA_FX_BLOOM_TMP_STRIDE

static uint16_t *bm_wa;
static uint16_t bm_w, bm_h, bm_n, bm_hh;   // Size, half width, half height
static uint32_t bm_src_skip, bm_dst_skip;

static uint16_t bm_in_row, bm_in_col;
static uint32_t bm_in_addr;
static uint16_t bm_rows_in;             // Source rows read
static uint16_t bm_half_ready;          // Half rows made (step 1 + across)
static uint16_t bm_bloom_next;          // Next bloom row to make
static uint16_t bm_next_y;              // Next output row to make
static uint16_t bm_out_row, bm_out_col;
static uint32_t bm_out_addr;
static uint16_t bm_rows_out;
static uint16_t bm_dma;
static uint16_t bm_last_n;
static uint32_t bm_last_addr;
static uint32_t bm_kernel;

static uint16_t *bm_src_row(uint16_t r)
{
    return bm_wa + NEA_FX_BLOOM_WA_SRC + (r & (BM_SRC - 1)) * NEA_FX_BLOOM_MAX_W;
}

static uint16_t *bm_half_row(uint16_t k)
{
    return bm_wa + NEA_FX_BLOOM_WA_HALF
           + (k & (BM_HALF - 1)) * NEA_FX_BLOOM_MAX_W;
}

static uint16_t *bm_bloom_row(uint16_t k)
{
    return bm_wa + NEA_FX_BLOOM_WA_BLOOM + (k & 1) * NEA_FX_BLOOM_MAX_W;
}

static uint16_t *bm_out_buffer(uint16_t y)
{
    return bm_wa + NEA_FX_BLOOM_WA_OUT + (y & 1) * NEA_FX_BLOOM_MAX_W;
}

// Copies the edge values of a padded plane row (data at [2 .. n + 1])
static void bm_pad(uint16_t *row)
{
    row[0] = row[1] = row[2];
    row[bm_n + 3] = row[bm_n + 2] = row[bm_n + 1];
}

// Step 1 and the blur across: source rows 2k and 2k+1 into half row k
static void bm_make_half(uint16_t k, const uint16_t *row0,
                         const uint16_t *row1)
{
    uint16_t *tmp_rb = bm_wa + NEA_FX_BLOOM_WA_TMP;
    uint16_t *tmp_g = tmp_rb + BM_TMP;
    uint16_t *half = bm_half_row(k);

    uint32_t t0 = nea_timer_read();
    nea_bloom_down_asm(row0, row1, tmp_rb + 2, bm_n);
    bm_pad(tmp_rb);
    bm_pad(tmp_g);
    nea_bloom_h5_asm(tmp_rb, half, bm_n, 0);
    nea_bloom_h5_asm(tmp_g, half + BM_PLANE_G, bm_n, 1);
    bm_kernel += t0 - nea_timer_read();
}

// The blur down: half rows k-2 .. k+2 (clamped) into bloom row k
static uint16_t *bm_rows[5];   // Not a local: no local arrays on the Teak

static void bm_make_bloom(uint16_t k)
{
    for (uint16_t i = 0; i < 5; i++)
    {
        int16_t r = (int16_t)(k + i) - 2;
        if (r < 0)
            r = 0;
        else if (r > (int16_t)(bm_hh - 1))
            r = bm_hh - 1;
        bm_rows[i] = bm_half_row((uint16_t)r);
        nea_bloom_rows[i] = (uint16_t)bm_rows[i];
    }

    uint16_t *bloom = bm_bloom_row(k);

    uint32_t t0 = nea_timer_read();
    nea_bloom_v5_asm(bloom, bm_n, 0);
    // The g plane rows follow the rb ones. Pointer arithmetic, not integer:
    // the addresses are in words on the DSP.
    for (uint16_t i = 0; i < 5; i++)
        nea_bloom_rows[i] = (uint16_t)(bm_rows[i] + BM_PLANE_G);
    nea_bloom_v5_asm(bloom + BM_PLANE_G, bm_n, 1);
    bm_kernel += t0 - nea_timer_read();
}

// Step 3: source row y plus its bloom row into "out"
static void bm_make_out(uint16_t y, const uint16_t *src, uint16_t *out)
{
    uint16_t *bloom = bm_bloom_row(y >> 1);
    nea_bloom_add_rb = (uint16_t)bloom;
    nea_bloom_add_g = (uint16_t)(bloom + BM_PLANE_G);

    uint32_t t0 = nea_timer_read();
    nea_bloom_add_asm(src, out, bm_n);
    bm_kernel += t0 - nea_timer_read();
}

static void bm_start_in(void)
{
    uint16_t n = bm_w - bm_in_col;
    n = min16(n, NEA_DSP_DMA_MAX_WORDS);
    n = min16(n, page_words(bm_in_addr));

    bm_last_addr = bm_in_addr;
    bm_last_n = n;
    nea_dma_in_start((uint16_t)(bm_src_row(bm_in_row) + bm_in_col),
                     (uint16_t)bm_in_addr, (uint16_t)(bm_in_addr >> 16), n);
    bm_dma = 1;
}

static void bm_start_out(void)
{
    uint16_t n = bm_w - bm_out_col;
    n = min16(n, NEA_DSP_DMA_MAX_WORDS);
    n = min16(n, page_words(bm_out_addr));

    bm_last_addr = bm_out_addr;
    bm_last_n = n;
    nea_dma_out_start((uint16_t)(bm_out_buffer(bm_out_row) + bm_out_col),
                      (uint16_t)bm_out_addr, (uint16_t)(bm_out_addr >> 16), n);
    bm_dma = 2;
}

static void bm_transfer_done(void)
{
    uint16_t n = bm_last_n;

    if (bm_dma == 1)
    {
        bm_in_col += n;
        bm_in_addr += (uint32_t)n << 1;
        if (bm_in_col == bm_w)
        {
            uint16_t r = bm_in_row;
            if (r & 1)
            {
                bm_make_half(r >> 1, bm_src_row(r - 1), bm_src_row(r));
                bm_half_ready++;
            }
            bm_in_col = 0;
            bm_in_addr += bm_src_skip;
            bm_in_row++;
            bm_rows_in++;
        }
    }
    else
    {
        bm_out_col += n;
        bm_out_addr += (uint32_t)n << 1;
        if (bm_out_col == bm_w)
        {
            bm_out_col = 0;
            bm_out_addr += bm_dst_skip;
            bm_out_row++;
            bm_rows_out++;
        }
    }

    bm_dma = 0;
}

static uint16_t bloom_stream(void)
{
    bm_in_row = bm_in_col = bm_rows_in = 0;
    bm_half_ready = bm_bloom_next = bm_next_y = 0;
    bm_out_row = bm_out_col = bm_rows_out = 0;
    bm_dma = 0;

    uint32_t stall = 0;
    uint16_t gated = 0;

    while (bm_rows_out < bm_h)
    {
        uint16_t progress = 0;

        if (bm_dma != 0 && nea_dma_poll())
        {
            bm_transfer_done();
            progress = 1;
        }

        // Bloom row k needs half rows up to k + 2, and its buffer is free once
        // the output rows of bloom row k - 2 are made
        uint16_t k = bm_bloom_next;
        uint16_t need_half = k + 2;
        if (need_half > bm_hh - 1)
            need_half = bm_hh - 1;
        uint16_t bloom_ready = k < bm_hh && bm_half_ready > need_half;
        uint16_t bloom_free = bm_next_y + 2 >= 2 * k;

        // Output row y needs its bloom row and its source row
        uint16_t y = bm_next_y;
        uint16_t out_ready = y < bm_h && bm_bloom_next > (y >> 1)
                             && bm_rows_in > y;

        if (bm_dma == 0 && nea_gate_requested())
        {
            apbpSetSemaphore(NEA_DSP_SEM_GATE_ACK);
            gated = 1;
        }
        else if (bm_dma == 0)
        {
            if (gated)
            {
                apbpClearSemaphore(NEA_DSP_SEM_GATE_ACK);
                gated = 0;
            }

            // A source row's slot is free once the output row of the row
            // before it in that slot is made; a half row's slot once the
            // bloom rows that read the half row before it are made.
            uint16_t r = bm_in_row;
            uint16_t can_in = r < bm_h && r < bm_next_y + BM_SRC
                              && (r >> 1) < bm_bloom_next + BM_HALF - 2;
            uint16_t can_out = bm_out_row < bm_next_y;
            uint16_t starved = !(bloom_ready && bloom_free) && !out_ready;

            if (can_in && (starved || !can_out))
                bm_start_in();
            else if (can_out)
                bm_start_out();
        }

        if (bloom_ready && bloom_free)
        {
            bm_make_bloom(k);
            bm_bloom_next++;
            progress = 1;
        }
        else if (out_ready && y < bm_rows_out + 2)
        {
            bm_make_out(y, bm_src_row(y), bm_out_buffer(y));
            bm_next_y++;
            progress = 1;
        }

        if (progress || gated)
        {
            stall = 0;
        }
        else if (++stall > STALL_LIMIT)
        {
            // Same report as grading, see job_grade
            uint16_t ahbm = *(volatile uint16_t *)0x80E0;
            uint16_t end = *(volatile uint16_t *)0x818C;
            uint32_t detail = (bm_dma == 1) ? bm_rows_in : bm_rows_out;
            detail |= (uint32_t)bm_dma << 16;
            if (ahbm & 0x10)
                detail |= (uint32_t)1 << 18;
            if (ahbm & 0x04)
                detail |= (uint32_t)1 << 19;
            if (end & 0x02)
                detail |= (uint32_t)1 << 20;
            detail |= (uint32_t)bm_last_n << 21;

            nea_dma_stop();
            nea_job_result = bm_last_addr;
            nea_job_timed = 1;
            nea_job_cycles = detail;
            return NEA_DSP_STATUS_DMA_ERROR;
        }
    }

    nea_job_result = bm_kernel;
    return NEA_DSP_STATUS_OK;
}

static uint16_t job_bloom(void)
{
    uint16_t flags = nea_desc[NEA_DSP_D_FLAGS];
    bm_w = nea_desc[NEA_DSP_D_WORDS];
    bm_h = nea_desc[NEA_DSP_BM_HEIGHT];
    uint16_t t = nea_desc[NEA_DSP_BM_THRESHOLD];

    if (bm_w < 2 || bm_w > NEA_FX_BLOOM_MAX_W || (bm_w & 1)
        || bm_h < 2 || bm_h > NEA_FX_BLOOM_MAX_H || (bm_h & 1) || t > 31)
        return NEA_DSP_STATUS_BAD_ARG;

    bm_n = bm_w >> 1;
    bm_hh = bm_h >> 1;
    nea_bloom_trb = t | (t << 10);
    nea_bloom_tg = t << 5;

    // The work area is the grading LRG table: the next grading job rebuilds it
    grade_expanded_gen = 0;
    bm_wa = nea_grade_lrg;
    bm_kernel = 0;

    if (flags & NEA_DSP_FLAG_LOCAL)
    {
        // At most 16 rows: every half row fits in the ring at once
        if (bm_h > 2 * BM_HALF || (uint32_t)bm_w * bm_h > NEA_DSP_SCRATCH_WORDS)
            return NEA_DSP_STATUS_BAD_ARG;

        uint16_t *src = nea_scratch;
        for (uint16_t k = 0; k < bm_hh; k++)
        {
            bm_make_half(k, src, src + bm_w);
            src += 2 * bm_w;
        }

        uint16_t *out = bm_wa + NEA_FX_BLOOM_WA_LOCAL_OUT;
        src = nea_scratch;
        for (uint16_t k = 0; k < bm_hh; k++)
        {
            bm_make_bloom(k);
            for (uint16_t i = 0; i < 2; i++)
            {
                bm_make_out(2 * k + i, src, out);
                src += bm_w;
                out += bm_w;
            }
        }

        nea_job_result = bm_kernel;
        return NEA_DSP_STATUS_OK;
    }

    uint16_t src_stride = nea_desc[NEA_DSP_BM_SRC_STRIDE];
    uint16_t dst_stride = nea_desc[NEA_DSP_BM_DST_STRIDE];
    if (src_stride < bm_w || dst_stride < bm_w)
        return NEA_DSP_STATUS_BAD_ARG;

    bm_in_addr = nea_desc_u32(NEA_DSP_D_ADDR_A);
    bm_out_addr = nea_desc_u32(NEA_DSP_D_ADDR_B);
    bm_src_skip = (uint32_t)(src_stride - bm_w) << 1;
    bm_dst_skip = (uint32_t)(dst_stride - bm_w) << 1;

    return bloom_stream();
}

uint16_t nea_job_run(uint16_t kind)
{
    nea_job_result = 0;
    nea_job_timed = 0;

    switch (kind)
    {
        case NEA_DSP_JOB_NOP:
            return NEA_DSP_STATUS_OK;
        case NEA_DSP_JOB_SELFTEST:
            return job_selftest();
        case NEA_DSP_JOB_XFER_BENCH:
            return job_xfer_bench();
        case NEA_DSP_JOB_GRADE:
            return job_grade();
        case NEA_DSP_JOB_DISPLACE:
            return job_displace();
        case NEA_DSP_JOB_BLUR:
            return job_blur();
        case NEA_DSP_JOB_BLOOM:
            return job_bloom();
        default:
            return NEA_DSP_STATUS_BAD_CMD;
    }
}
