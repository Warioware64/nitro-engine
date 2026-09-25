// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Nitro Engine Advanced contributors
//
// This file is part of Nitro Engine Advanced

#include <malloc.h>

#include "NEAMain.h"

/// @file NEADspFX.c

//-----------------------------------------------------------------------------
// Grade description
//-----------------------------------------------------------------------------

void NEA_DspGradeInit(NEA_DspGrade *g)
{
    NEA_AssertPointer(g, "NULL grade");

    memset(g, 0, sizeof(*g));
    for (int i = 0; i < 3; i++)
    {
        g->matrix[i][i] = 256;
        for (int v = 0; v < 32; v++)
            g->curve[i][v] = v;
    }
}

// g = m applied after g: matrix = m * matrix, offset = m * offset + o
static void ne_grade_compose(NEA_DspGrade *g, const s32 m[3][3],
                             const s32 o[3])
{
    s16 mat[3][3];
    s16 off[3];

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            s32 sum = 0;
            for (int k = 0; k < 3; k++)
                sum += m[i][k] * g->matrix[k][j];
            mat[i][j] = (s16)((sum + 128) >> 8);
        }

        s32 sum = 0;
        for (int k = 0; k < 3; k++)
            sum += m[i][k] * g->offset[k];
        off[i] = (s16)(((sum + 128) >> 8) + o[i]);
    }

    memcpy(g->matrix, mat, sizeof(mat));
    memcpy(g->offset, off, sizeof(off));
}

// Rec. 601 luma, Q8
#define NE_LUMA_R 77
#define NE_LUMA_G 150
#define NE_LUMA_B 29

void NEA_DspGradeSaturate(NEA_DspGrade *g, int s)
{
    NEA_AssertPointer(g, "NULL grade");

    static const s32 luma[3] = { NE_LUMA_R, NE_LUMA_G, NE_LUMA_B };
    s32 m[3][3];
    const s32 o[3] = { 0, 0, 0 };

    // s * identity + (1 - s) * luma
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            m[i][j] = ((256 - s) * luma[j]) >> 8;
            if (i == j)
                m[i][j] += s;
        }
    }

    ne_grade_compose(g, m, o);
}

void NEA_DspGradeSepia(NEA_DspGrade *g, int amount)
{
    NEA_AssertPointer(g, "NULL grade");

    // The usual sepia matrix, Q8
    static const s32 sepia[3][3] = {
        { 101, 197, 48 },
        { 89, 176, 43 },
        { 70, 137, 34 },
    };
    s32 m[3][3];
    const s32 o[3] = { 0, 0, 0 };

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            s32 id = (i == j) ? 256 : 0;
            m[i][j] = id + (((sepia[i][j] - id) * amount) >> 8);
        }
    }

    ne_grade_compose(g, m, o);
}

void NEA_DspGradeHueRotate(NEA_DspGrade *g, int angle)
{
    NEA_AssertPointer(g, "NULL grade");

    // Luminance-preserving hue rotation, coefficients in Q8 (the Q12 sine and
    // cosine are reduced when multiplied in).
    s32 c = cosLerp(angle);
    s32 s = sinLerp(angle);

    static const s32 base[3] = { 54, 183, 18 };
    static const s32 kc[3][3] = {
        { 201, -183, -18 },
        { -54, 73, -18 },
        { -54, -183, 238 },
    };
    static const s32 ks[3][3] = {
        { -54, -183, 238 },
        { 37, 36, -72 },
        { -201, 183, 18 },
    };

    s32 m[3][3];
    const s32 o[3] = { 0, 0, 0 };

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
            m[i][j] = base[j] + ((kc[i][j] * c + ks[i][j] * s) >> 12);
    }

    ne_grade_compose(g, m, o);
}

void NEA_DspGradeTint(NEA_DspGrade *g, int r, int gr, int b, int offset)
{
    NEA_AssertPointer(g, "NULL grade");

    const s32 m[3][3] = {
        { r, 0, 0 },
        { 0, gr, 0 },
        { 0, 0, b },
    };
    const s32 o[3] = { offset, offset, offset };

    ne_grade_compose(g, m, o);
}

void NEA_DspGradeInvert(NEA_DspGrade *g)
{
    NEA_AssertPointer(g, "NULL grade");

    const s32 m[3][3] = {
        { -256, 0, 0 },
        { 0, -256, 0 },
        { 0, 0, -256 },
    };
    const s32 o[3] = { 31 << 8, 31 << 8, 31 << 8 };

    ne_grade_compose(g, m, o);
}

void NEA_DspGradeContrast(NEA_DspGrade *g, int contrast, int brightness)
{
    NEA_AssertPointer(g, "NULL grade");

    for (int c = 0; c < 3; c++)
    {
        for (int v = 0; v < 32; v++)
        {
            int x = g->curve[c][v];
            x = (((x - 16) * contrast + 128) >> 8) + 16 + brightness;
            if (x < 0)
                x = 0;
            if (x > 31)
                x = 31;
            g->curve[c][v] = x;
        }
    }
}

//-----------------------------------------------------------------------------
// Tables
//-----------------------------------------------------------------------------

static u16 ne_grade_generation;

NEA_DspGradeTables *NEA_DspGradeTablesCreate(void)
{
    NEA_DspGradeTables *t = memalign(64, sizeof(NEA_DspGradeTables));
    if (t == NULL)
    {
        NEA_DebugPrint("Not enough memory");
        return NULL;
    }

    NEA_DspGrade g;
    NEA_DspGradeInit(&g);
    NEA_DspGradeBuild(t, &g);
    return t;
}

void NEA_DspGradeTablesDelete(NEA_DspGradeTables *t)
{
    free(t);
}

// Matrix output of one term, in whole 5-bit steps, rounded
static inline int ne_grade_term(s32 q8)
{
    return (q8 + 128) >> 8;
}

static void ne_grade_wait_tables_read(const NEA_DspGradeTables *t);

int NEA_DspGradeBuild(NEA_DspGradeTables *t, const NEA_DspGrade *g)
{
    NEA_AssertPointer(t, "NULL tables");
    NEA_AssertPointer(g, "NULL grade");

    // A DSP job may be reading these very tables
    ne_grade_wait_tables_read(t);

    // Range of each output channel over all inputs, before the curves. The
    // packed fields hold value + 32 in 7 bits, so it must stay in -32..95.
    int lo1[3], hi1[3], lo2[3], hi2[3];
    for (int c = 0; c < 3; c++)
    {
        lo1[c] = lo2[c] = 0x7FFFFFFF;
        hi1[c] = hi2[c] = -0x7FFFFFFF;
    }

    // Fill into a scratch copy first, so a failed build leaves t unchanged
    u16 *blob = malloc(NEA_FX_GRADE_WORDS * sizeof(u16));
    if (blob == NULL)
        return -1;

    for (int rg = 0; rg < 1024; rg++)
    {
        int r = rg & 31;
        int gg = rg >> 5;
        int v[3];

        for (int c = 0; c < 3; c++)
        {
            v[c] = ne_grade_term(g->matrix[c][0] * r + g->matrix[c][1] * gg
                                 + g->offset[c]);
            if (v[c] < lo1[c])
                lo1[c] = v[c];
            if (v[c] > hi1[c])
                hi1[c] = v[c];
        }

        int a = (v[0] + NEA_FX_GRADE_BIAS) + 128 * (v[1] + NEA_FX_GRADE_BIAS);
        blob[NEA_FX_GRADE_T1 + 2 * rg] = (u16)a;
        blob[NEA_FX_GRADE_T1 + 2 * rg + 1] = (u16)(v[2] + NEA_FX_GRADE_BIAS);
    }

    for (int b = 0; b < 32; b++)
    {
        int w[3];

        for (int c = 0; c < 3; c++)
        {
            w[c] = ne_grade_term(g->matrix[c][2] * b);
            if (w[c] < lo2[c])
                lo2[c] = w[c];
            if (w[c] > hi2[c])
                hi2[c] = w[c];
        }

        blob[NEA_FX_GRADE_T2 + 2 * b] = (u16)(w[0] + 128 * w[1]);
        blob[NEA_FX_GRADE_T2 + 2 * b + 1] = (u16)w[2];
    }

    for (int c = 0; c < 3; c++)
    {
        if (lo1[c] + lo2[c] < -NEA_FX_GRADE_BIAS
            || hi1[c] + hi2[c] > 127 - NEA_FX_GRADE_BIAS)
        {
            NEA_DebugPrint("Grade out of range: channel %d, %d..%d", c,
                           lo1[c] + lo2[c], hi1[c] + hi2[c]);
            free(blob);
            return -1;
        }
    }

    for (int i = 0; i < 128; i++)
    {
        int v = i - NEA_FX_GRADE_BIAS;
        if (v < 0)
            v = 0;
        if (v > 31)
            v = 31;

        blob[NEA_FX_GRADE_LR + i] = g->curve[0][v];
        blob[NEA_FX_GRADE_LG + i] = g->curve[1][v];
        blob[NEA_FX_GRADE_LB + i] = (g->curve[2][v] << 10) | 0x8000;
    }

    memcpy(t->blob, blob, sizeof(t->blob));
    free(blob);

    ne_grade_generation++;
    if (ne_grade_generation == 0)
        ne_grade_generation = 1;
    t->generation = ne_grade_generation;

    return 0;
}

//-----------------------------------------------------------------------------
// Applying
//-----------------------------------------------------------------------------

void NEA_DspGradeApplyCPU(const NEA_DspGradeTables *t, const u16 *src,
                          u16 *dst, int pixels)
{
    NEA_AssertPointer(t, "NULL tables");

    const u16 *blob = t->blob;
    for (int i = 0; i < pixels; i++)
        dst[i] = nea_fx_grade_pixel(blob, src[i]);
}

// Static, not local: the DSP reads it with DMA and can't see DTCM
static NEA_DspDesc ne_grade_desc;

static bool ne_grade_pending;
static int ne_grade_last_status;
static u32 ne_grade_failures;

// The job in flight, to grade it on the ARM9 if the DSP fails
static const NEA_DspGradeTables *ne_grade_job_tables;
static const u16 *ne_grade_job_src;
static u16 *ne_grade_job_dst;
static int ne_grade_job_src_stride, ne_grade_job_dst_stride;
static int ne_grade_job_width, ne_grade_job_height;
static u32 ne_grade_last_detail;    // On a DMA error, where it stuck
static u32 ne_grade_last_addr;      // On a DMA error, the ARM9 address
static bool ne_grade_pending_main_ram;

static bool ne_grade_overlap = true;

static u32 ne_grade_kernel_cycles;
static u32 ne_grade_job_cycles;

// FIFO transport: what the DSP holds, and where
static u16 ne_grade_fifo_gen;
static u16 ne_grade_fifo_blob_addr;

static bool ne_grade_in_main_ram(const void *p)
{
    uintptr_t a = (uintptr_t)p;
    return a >= 0x02000000 && a < 0x03000000;
}

// Grades "n" pixels already in the DSP scratch buffer, in place.
static bool ne_grade_fifo_run(const NEA_DspGradeTables *t, int n)
{
    NEA_DspDescInit(&ne_grade_desc, NEA_DSP_JOB_GRADE);
    ne_grade_desc.w[NEA_DSP_D_FLAGS] = NEA_DSP_FLAG_LOCAL
                                       | NEA_DSP_FLAG_TABLES_LOCAL;
    ne_grade_desc.w[NEA_DSP_D_WORDS] = n;
    ne_grade_desc.w[NEA_DSP_GR_GEN] = t->generation;

    NEA_DspJobResult r;
    if (NEA_DspJobRun(&ne_grade_desc, &r) != NEA_DSP_STATUS_OK)
        return false;

    ne_grade_kernel_cycles += r.result;
    ne_grade_job_cycles += r.cycles;
    return true;
}

// Emulators: the ARM9 moves every word through the FIFO and the DSP grades in
// place, as many whole rows at a time as fit in its scratch buffer. Slow, and
// only here so that the DSP code can be checked where DMA doesn't work.
static int ne_grade_fifo(const NEA_DspGradeTables *t, const u16 *src,
                         int src_stride, u16 *dst, int dst_stride, int width,
                         int height)
{
    if (ne_grade_fifo_blob_addr == 0)
    {
        u32 addr = 0;
        if (__NEA_DspInfo(NEA_DSP_INFO_GRADE, &addr) != NEA_DSP_STATUS_OK)
            return 0;
        ne_grade_fifo_blob_addr = addr;
    }

    if (ne_grade_fifo_gen != t->generation)
    {
        dspFifoWriteData(t->blob, ne_grade_fifo_blob_addr, NEA_FX_GRADE_WORDS);
        ne_grade_fifo_gen = t->generation;
    }

    u16 scratch = __NEA_DspScratchAddr();

    ne_grade_kernel_cycles = 0;
    ne_grade_job_cycles = 0;

    if (width > NEA_DSP_SCRATCH_WORDS)
    {
        // Long rows: pieces of one row at a time
        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; )
            {
                int n = width - x;
                if (n > NEA_DSP_SCRATCH_WORDS)
                    n = NEA_DSP_SCRATCH_WORDS;

                dspFifoWriteData(src + y * src_stride + x, scratch, n);
                if (!ne_grade_fifo_run(t, n))
                    return 0;
                dspFifoReadData(scratch, dst + y * dst_stride + x, n);
                x += n;
            }
        }
        return 2;
    }

    int rows_per_piece = NEA_DSP_SCRATCH_WORDS / width;

    for (int y = 0; y < height; )
    {
        int rows = height - y;
        if (rows > rows_per_piece)
            rows = rows_per_piece;

        for (int i = 0; i < rows; i++)
        {
            dspFifoWriteData(src + (y + i) * src_stride, scratch + i * width,
                             width);
        }

        if (!ne_grade_fifo_run(t, rows * width))
            return 0;

        for (int i = 0; i < rows; i++)
        {
            dspFifoReadData(scratch + i * width, dst + (y + i) * dst_stride,
                            width);
        }

        y += rows;
    }

    return 2;
}

// Waits until the DSP job in flight, if it was given these tables, has its own
// copy of them (NEA_DSP_SEM_TABLES_READ). That is the first thing a job does,
// so this is short; it never waits for the whole job.
static void ne_grade_wait_tables_read(const NEA_DspGradeTables *t)
{
    if (!ne_grade_pending || ne_grade_job_tables != t)
        return;

    for (int n = 0x100000; n > 0; n--)
    {
        if (dspGetSemaphore() & NEA_DSP_SEM_TABLES_READ)
            return;
        if (dspReceiveDataReady(0)) // The job has finished
            return;
    }

    // No sign of life: collect the job, which also recovers the DSP
    NEA_DspGradeEnd();
}

static void ne_grade_cpu_rect(const NEA_DspGradeTables *t, const u16 *src,
                              int src_stride, u16 *dst, int dst_stride,
                              int width, int height)
{
    for (int y = 0; y < height; y++)
    {
        NEA_DspGradeApplyCPU(t, src, dst, width);
        src += src_stride;
        dst += dst_stride;
    }
}

int NEA_DspGradeBeginRect(const NEA_DspGradeTables *t, const u16 *src,
                          int src_stride, u16 *dst, int dst_stride, int width,
                          int height)
{
    NEA_AssertPointer(t, "NULL tables");
    NEA_AssertPointer(src, "NULL source");
    NEA_AssertPointer(dst, "NULL destination");
    NEA_Assert(width > 0 && height > 0 && width * height <= 0xFFFF
               && src_stride >= width && dst_stride >= width
               && src_stride <= 0xFFFF && dst_stride <= 0xFFFF,
               "Invalid rectangle");

    if (!NEA_DspIsUsedForWork())
    {
        ne_grade_cpu_rect(t, src, src_stride, dst, dst_stride, width, height);
        return 2;
    }

    if (NEA_DspJobIsPending())
        return 0;

    if (NEA_DspGetTransport() == NEA_DSP_TRANSPORT_FIFO)
    {
        return ne_grade_fifo(t, src, src_stride, dst, dst_stride, width,
                             height);
    }

    bool contiguous = (height == 1)
                      || (width == src_stride && width == dst_stride);

    // Written back now: the DSP reads src and the tables from memory, and no
    // dirty line of dst may land on its output later. After this no line of
    // dst is left in the cache either. The ARM9 data cache is 4 KB, so this
    // costs a few microseconds, where flushing a 256x192 source and
    // destination by range cost 544 us (measured on a DSi).
    DC_FlushAll();

    NEA_DspDescInit(&ne_grade_desc, NEA_DSP_JOB_GRADE);
    NEA_DspDescSetAddr(&ne_grade_desc, NEA_DSP_D_ADDR_A, src);
    NEA_DspDescSetAddr(&ne_grade_desc, NEA_DSP_D_ADDR_B, dst);
    NEA_DspDescSetAddr(&ne_grade_desc, NEA_DSP_GR_TABLES, t->blob);
    ne_grade_desc.w[NEA_DSP_D_WORDS] = width * height;
    ne_grade_desc.w[NEA_DSP_GR_GEN] = t->generation;

    if (contiguous)
    {
        // One long row: the DSP never has to find a row end
        ne_grade_desc.w[NEA_DSP_GR_WIDTH] = width * height;
        ne_grade_desc.w[NEA_DSP_GR_SRC_STRIDE] = width * height;
        ne_grade_desc.w[NEA_DSP_GR_DST_STRIDE] = width * height;
        if (!ne_grade_overlap)
            ne_grade_desc.w[NEA_DSP_D_FLAGS] = NEA_DSP_FLAG_NO_OVERLAP;
    }
    else
    {
        ne_grade_desc.w[NEA_DSP_GR_WIDTH] = width;
        ne_grade_desc.w[NEA_DSP_GR_SRC_STRIDE] = src_stride;
        ne_grade_desc.w[NEA_DSP_GR_DST_STRIDE] = dst_stride;
    }

    if (!NEA_DspJobBegin(&ne_grade_desc))
        return 0;

    ne_grade_pending = true;
    ne_grade_pending_main_ram = ne_grade_in_main_ram(dst);

    ne_grade_job_tables = t;
    ne_grade_job_src = src;
    ne_grade_job_dst = dst;
    ne_grade_job_src_stride = src_stride;
    ne_grade_job_dst_stride = dst_stride;
    ne_grade_job_width = width;
    ne_grade_job_height = height;
    return 1;
}

int NEA_DspGradeBegin(const NEA_DspGradeTables *t, const u16 *src, u16 *dst,
                      int pixels)
{
    return NEA_DspGradeBeginRect(t, src, pixels, dst, pixels, pixels, 1);
}

int NEA_DspGradeEnd(void)
{
    if (!ne_grade_pending)
        return 2;

    NEA_DspJobResult r;
    int status = NEA_DspJobWait(&r);

    // Drops any line of dst that got into the cache during the job
    if (ne_grade_pending_main_ram)
        DC_FlushAll();

    ne_grade_pending = false;
    ne_grade_last_status = status;
    // On a DMA error the result is the stuck transfer's ARM9 address and the
    // second reply its details, see nea_teak_jobs.c
    ne_grade_last_detail = r.cycles;
    ne_grade_last_addr = r.result;

    if (status != NEA_DSP_STATUS_OK)
    {
        NEA_DebugPrint("DSP grade failed: %d", status);
        ne_grade_failures++;

        // A stalled transfer can leave the DSP's AHBM busy for good, and then
        // every later job fails as well: restart the DSP. It has lost its
        // tables, which it fetches again because their generation won't match.
        if (status == NEA_DSP_STATUS_DMA_ERROR || status < 0)
        {
            NEA_DspReset();
            ne_grade_fifo_gen = 0;
            ne_grade_fifo_blob_addr = 0;
        }

        // In place, the DSP may already have graded part of the image, and
        // grading that part again would be wrong. Otherwise the source is
        // intact: grade it here so the caller still gets the right pixels.
        if (ne_grade_job_src == ne_grade_job_dst)
            return 0;

        ne_grade_cpu_rect(ne_grade_job_tables, ne_grade_job_src,
                          ne_grade_job_src_stride, ne_grade_job_dst,
                          ne_grade_job_dst_stride, ne_grade_job_width,
                          ne_grade_job_height);
        return 2;
    }

    ne_grade_kernel_cycles = r.result;
    ne_grade_job_cycles = r.cycles;
    return 1;
}

int NEA_DspGradeApply(const NEA_DspGradeTables *t, const u16 *src, u16 *dst,
                      int pixels)
{
    int ret = NEA_DspGradeBegin(t, src, dst, pixels);
    if (ret == 1)
        return NEA_DspGradeEnd();
    return ret;
}

void NEA_DspGradeSetOverlap(bool overlap)
{
    ne_grade_overlap = overlap;
}

u32 NEA_DspGradeGetFailureCount(void)
{
    return ne_grade_failures;
}

int NEA_DspGradeGetLastStatus(u32 *detail, u32 *addr)
{
    if (detail)
        *detail = ne_grade_last_detail;
    if (addr)
        *addr = ne_grade_last_addr;
    return ne_grade_last_status;
}

void NEA_DspGradeGetStats(u32 *kernel_cycles, u32 *job_cycles)
{
    if (kernel_cycles)
        *kernel_cycles = ne_grade_kernel_cycles;
    if (job_cycles)
        *job_cycles = ne_grade_job_cycles;
}

//-----------------------------------------------------------------------------
// Displacement
//-----------------------------------------------------------------------------

void NEA_DspDisplaceInit(NEA_DspDisplace *d, int width, int height)
{
    NEA_AssertPointer(d, "NULL displacement");
    NEA_Assert(width > 0 && width <= NEA_FX_DISP_MAX_W
               && height > 0 && height <= NEA_FX_DISP_MAX_H,
               "Invalid displacement size");

    memset(d, 0, sizeof(*d));
    d->width = width;
    d->height = height;
}

void NEA_DspDisplaceAddWave(NEA_DspDisplace *d, NEA_DspDisplaceTerm term,
                            int amplitude, int wavelength, int phase)
{
    NEA_AssertPointer(d, "NULL displacement");
    NEA_Assert(wavelength > 0, "Invalid wavelength");

    s16 *values;
    int count;

    switch (term)
    {
        case NEA_DISPLACE_DX_ALONG_X:
            values = d->ax;
            count = d->width;
            break;
        case NEA_DISPLACE_DX_ALONG_Y:
            values = d->ay;
            count = d->height;
            break;
        case NEA_DISPLACE_DY_ALONG_X:
            values = d->bx;
            count = d->width;
            break;
        default:
            values = d->by;
            count = d->height;
            break;
    }

    for (int i = 0; i < count; i++)
    {
        int angle = phase + (i * DEGREES_IN_CIRCLE) / wavelength;
        values[i] += (amplitude * sinLerp(angle) + 2048) >> 12;
    }
}

void NEA_DspDisplaceHeatHaze(NEA_DspDisplace *d, int time, int strength)
{
    // Rows sway sideways, with a slower column wobble and a slight vertical
    // shimmer, all moving at different speeds so the pattern never repeats
    // visibly
    NEA_DspDisplaceAddWave(d, NEA_DISPLACE_DX_ALONG_Y, strength, 23,
                           time * 700);
    NEA_DspDisplaceAddWave(d, NEA_DISPLACE_DX_ALONG_X, (strength + 1) / 2, 41,
                           -time * 300);
    NEA_DspDisplaceAddWave(d, NEA_DISPLACE_DY_ALONG_X, 1, 29, time * 500);
}

void NEA_DspDisplaceRipple(NEA_DspDisplace *d, int time, int strength)
{
    NEA_DspDisplaceAddWave(d, NEA_DISPLACE_DX_ALONG_Y, strength, 32,
                           time * 400);
    NEA_DspDisplaceAddWave(d, NEA_DISPLACE_DY_ALONG_X, (strength + 1) / 2, 48,
                           time * 300);
    NEA_DspDisplaceAddWave(d, NEA_DISPLACE_DY_ALONG_Y, 1, 19, -time * 250);
}

NEA_DspDisplaceTables *NEA_DspDisplaceTablesCreate(void)
{
    NEA_DspDisplaceTables *t = memalign(64, sizeof(NEA_DspDisplaceTables));
    if (t == NULL)
    {
        NEA_DebugPrint("Not enough memory");
        return NULL;
    }
    memset(t, 0, sizeof(*t));
    return t;
}

void NEA_DspDisplaceTablesDelete(NEA_DspDisplaceTables *t)
{
    free(t);
}

static int ne_max_abs(const s16 *v, int n)
{
    int m = 0;
    for (int i = 0; i < n; i++)
    {
        int a = v[i] < 0 ? -v[i] : v[i];
        if (a > m)
            m = a;
    }
    return m;
}

static bool ne_disp_pending;
static const NEA_DspDisplaceTables *ne_disp_job_tables;

// See ne_grade_wait_tables_read()
static void ne_disp_wait_tables_read(const NEA_DspDisplaceTables *t);

int NEA_DspDisplaceBuild(NEA_DspDisplaceTables *t, const NEA_DspDisplace *d)
{
    NEA_AssertPointer(d, "NULL displacement");
    return NEA_DspDisplaceBuildScaled(t, d, d->width);
}

int NEA_DspDisplaceBuildScaled(NEA_DspDisplaceTables *t,
                               const NEA_DspDisplace *d, int src_width)
{
    NEA_AssertPointer(t, "NULL tables");
    NEA_AssertPointer(d, "NULL displacement");
    NEA_Assert(src_width > 0 && src_width <= NEA_FX_DISP_MAX_W,
               "Invalid source width");

    int w = d->width, h = d->height;

    // The sum of the two terms of each offset must stay in range for every
    // pixel. Checking the largest of each term is simple and a little strict.
    if (ne_max_abs(d->ax, w) + ne_max_abs(d->ay, h) > NEA_FX_DISP_XMAX
        || ne_max_abs(d->bx, w) + ne_max_abs(d->by, h) > NEA_FX_DISP_YMAX)
    {
        NEA_DebugPrint("Displacement out of range");
        return -1;
    }

    // A DSP job may be reading these very tables
    ne_disp_wait_tables_read(t);

    for (int x = 0; x < NEA_FX_DISP_MAX_W; x++)
    {
        int ax = x < w ? d->ax[x] : 0;
        int bx = x < w ? d->bx[x] : 0;
        // Output column x reads source column x * src_width / width (nearest
        // lower): a wider source is scaled down
        int sx = x < w ? (x * src_width) / w : 0;
        t->blob[NEA_FX_DISP_XA + x] = (u16)(sx + NEA_FX_DISP_PAD + ax);
        t->blob[NEA_FX_DISP_XB + x] = (u16)(bx + NEA_FX_DISP_YMAX);
    }
    for (int y = 0; y < NEA_FX_DISP_MAX_H; y++)
    {
        t->blob[NEA_FX_DISP_YA + y] = (u16)(y < h ? d->ay[y] : 0);
        t->blob[NEA_FX_DISP_YB + y] = (u16)(y < h ? d->by[y] : 0);
    }

    t->width = w;
    t->height = h;
    t->src_width = src_width;
    return 0;
}

void NEA_DspDisplaceApplyCPU(const NEA_DspDisplaceTables *t, const u16 *src,
                             int src_stride, u16 *dst, int dst_stride)
{
    NEA_AssertPointer(t, "NULL tables");

    for (int y = 0; y < t->height; y++)
    {
        nea_fx_displace_row(t->blob, src, src_stride, t->src_width,
                            dst + y * dst_stride, t->width, t->height, y);
    }
}

// Static, not local: the DSP reads it with DMA and can't see DTCM
static NEA_DspDesc ne_disp_desc;

static const u16 *ne_disp_job_src;
static u16 *ne_disp_job_dst;
static int ne_disp_job_src_stride, ne_disp_job_dst_stride;
static u32 ne_disp_kernel_cycles, ne_disp_job_cycles;
static u32 ne_disp_failures;

static u16 ne_disp_fifo_ring, ne_disp_fifo_blob;

static void ne_disp_wait_tables_read(const NEA_DspDisplaceTables *t)
{
    if (!ne_disp_pending || ne_disp_job_tables != t)
        return;

    for (int n = 0x100000; n > 0; n--)
    {
        if (dspGetSemaphore() & NEA_DSP_SEM_TABLES_READ)
            return;
        if (dspReceiveDataReady(0)) // The job has finished
            return;
    }

    NEA_DspDisplaceEnd();
}

// Emulators: small images only (the DSP holds all their rows at once), moved
// through the FIFO. Enough to check the DSP code where DMA doesn't work.
static int ne_disp_fifo(const NEA_DspDisplaceTables *t, const u16 *src,
                        int src_stride, u16 *dst, int dst_stride)
{
    int w = t->width, h = t->height;

    if (h > NEA_FX_DISP_RING_ROWS || w * h > NEA_DSP_SCRATCH_WORDS)
        return 0;

    if (ne_disp_fifo_ring == 0)
    {
        u32 info = 0;
        if (__NEA_DspInfo(NEA_DSP_INFO_DISPLACE, &info) != NEA_DSP_STATUS_OK)
            return 0;
        ne_disp_fifo_ring = info >> 16;
        ne_disp_fifo_blob = info & 0xFFFF;
    }

    dspFifoWriteData(t->blob, ne_disp_fifo_blob, NEA_FX_DISP_WORDS);

    for (int y = 0; y < h; y++)
    {
        u16 slot = ne_disp_fifo_ring + y * NEA_FX_DISP_SLOT_WORDS;
        dspFifoWriteData(src + y * src_stride, slot + NEA_FX_DISP_PAD,
                         t->src_width);
    }

    NEA_DspDescInit(&ne_disp_desc, NEA_DSP_JOB_DISPLACE);
    ne_disp_desc.w[NEA_DSP_D_FLAGS] = NEA_DSP_FLAG_LOCAL;
    ne_disp_desc.w[NEA_DSP_D_WORDS] = w;
    ne_disp_desc.w[NEA_DSP_DP_HEIGHT] = h;
    ne_disp_desc.w[NEA_DSP_DP_SRC_W] = t->src_width;

    NEA_DspJobResult r;
    if (NEA_DspJobRun(&ne_disp_desc, &r) != NEA_DSP_STATUS_OK)
        return 0;

    ne_disp_kernel_cycles = r.result;
    ne_disp_job_cycles = r.cycles;

    u16 scratch = __NEA_DspScratchAddr();
    for (int y = 0; y < h; y++)
        dspFifoReadData(scratch + y * w, dst + y * dst_stride, w);

    return 2;
}

int NEA_DspDisplaceBegin(const NEA_DspDisplaceTables *t, const u16 *src,
                         int src_stride, u16 *dst, int dst_stride)
{
    NEA_AssertPointer(t, "NULL tables");
    NEA_AssertPointer(src, "NULL source");
    NEA_AssertPointer(dst, "NULL destination");
    NEA_Assert(t->width > 0, "Tables not built");
    NEA_Assert((src_stride >= t->src_width || -src_stride >= t->src_width)
               && dst_stride >= t->width, "Invalid strides");
    NEA_Assert(src != dst, "Displacement can't work in place");

    if (!NEA_DspIsUsedForWork())
    {
        NEA_DspDisplaceApplyCPU(t, src, src_stride, dst, dst_stride);
        return 2;
    }

    if (NEA_DspJobIsPending())
        return 0;

    if (NEA_DspGetTransport() == NEA_DSP_TRANSPORT_FIFO)
    {
        int ret = ne_disp_fifo(t, src, src_stride, dst, dst_stride);
        if (ret == 0)
            NEA_DspDisplaceApplyCPU(t, src, src_stride, dst, dst_stride);
        return 2;
    }

    // See NEA_DspGradeBeginRect()
    DC_FlushAll();

    NEA_DspDescInit(&ne_disp_desc, NEA_DSP_JOB_DISPLACE);
    NEA_DspDescSetAddr(&ne_disp_desc, NEA_DSP_D_ADDR_A, src);
    NEA_DspDescSetAddr(&ne_disp_desc, NEA_DSP_D_ADDR_B, dst);
    NEA_DspDescSetAddr(&ne_disp_desc, NEA_DSP_DP_TABLES, t->blob);
    ne_disp_desc.w[NEA_DSP_D_WORDS] = t->width;
    ne_disp_desc.w[NEA_DSP_DP_HEIGHT] = t->height;
    ne_disp_desc.w[NEA_DSP_DP_SRC_STRIDE] = (u16)(s16)src_stride;
    ne_disp_desc.w[NEA_DSP_DP_DST_STRIDE] = dst_stride;
    ne_disp_desc.w[NEA_DSP_DP_SRC_W] = t->src_width;

    if (!NEA_DspJobBegin(&ne_disp_desc))
        return 0;

    ne_disp_pending = true;
    ne_disp_job_tables = t;
    ne_disp_job_src = src;
    ne_disp_job_dst = dst;
    ne_disp_job_src_stride = src_stride;
    ne_disp_job_dst_stride = dst_stride;
    return 1;
}

int NEA_DspDisplaceEnd(void)
{
    if (!ne_disp_pending)
        return 2;

    NEA_DspJobResult r;
    int status = NEA_DspJobWait(&r);

    // Drops any line of dst that got into the cache during the job
    DC_FlushAll();

    ne_disp_pending = false;

    if (status != NEA_DSP_STATUS_OK)
    {
        NEA_DebugPrint("DSP displacement failed: %d", status);
        ne_disp_failures++;

        // A stalled transfer can leave the DSP's bus access wedged
        if (status == NEA_DSP_STATUS_DMA_ERROR || status < 0)
            NEA_DspReset();

        // The source is intact (displacement is never in place)
        NEA_DspDisplaceApplyCPU(ne_disp_job_tables, ne_disp_job_src,
                                ne_disp_job_src_stride, ne_disp_job_dst,
                                ne_disp_job_dst_stride);
        return 2;
    }

    ne_disp_kernel_cycles = r.result;
    ne_disp_job_cycles = r.cycles;
    return 1;
}

int NEA_DspDisplaceApply(const NEA_DspDisplaceTables *t, const u16 *src,
                         int src_stride, u16 *dst, int dst_stride)
{
    int ret = NEA_DspDisplaceBegin(t, src, src_stride, dst, dst_stride);
    if (ret == 1)
        return NEA_DspDisplaceEnd();
    if (ret == 0)
    {
        // The DSP is busy with another job
        NEA_DspDisplaceApplyCPU(t, src, src_stride, dst, dst_stride);
        return 2;
    }
    return ret;
}

void NEA_DspDisplaceGetStats(u32 *kernel_cycles, u32 *job_cycles)
{
    if (kernel_cycles)
        *kernel_cycles = ne_disp_kernel_cycles;
    if (job_cycles)
        *job_cycles = ne_disp_job_cycles;
}

u32 NEA_DspDisplaceGetFailureCount(void)
{
    return ne_disp_failures;
}

//-----------------------------------------------------------------------------
// Blur
//-----------------------------------------------------------------------------

void NEA_DspBlurApplyCPU(const u16 *src, int src_stride, u16 *dst,
                         int dst_stride, int width, int height)
{
    NEA_Assert(width > 0 && width <= NEA_FX_BLUR_MAX_W
               && height > 0 && height <= NEA_FX_BLUR_MAX_H,
               "Invalid blur size");

    // Three rows of planes, recycled; the source rows are read before the
    // output row that could overwrite them (in place) is written
    u16 *planes = malloc(3 * 2 * width * sizeof(u16));
    if (planes == NULL)
        return;

    u16 *rows[3] = { planes, planes + 2 * width, planes + 4 * width };

    nea_fx_blur_row_h(src, rows[1], width);                 // Row 0
    if (height > 1)
        nea_fx_blur_row_h(src + src_stride, rows[2], width); // Row 1
    else
        memcpy(rows[2], rows[1], 2 * width * sizeof(u16));
    memcpy(rows[0], rows[1], 2 * width * sizeof(u16));       // Row -1 = row 0

    for (int y = 0; y < height; y++)
    {
        nea_fx_blur_row_v(rows[0], rows[1], rows[2], dst + y * dst_stride,
                          width);

        // Shift the window down: the new bottom row is y + 2, clamped
        u16 *old = rows[0];
        rows[0] = rows[1];
        rows[1] = rows[2];
        rows[2] = old;
        if (y + 2 < height)
            nea_fx_blur_row_h(src + (y + 2) * src_stride, rows[2], width);
        else
            memcpy(rows[2], rows[1], 2 * width * sizeof(u16));
    }

    free(planes);
}

// Static, not local: the DSP reads it with DMA and can't see DTCM
static NEA_DspDesc ne_blur_desc;

static bool ne_blur_pending;
static const u16 *ne_blur_job_src;
static u16 *ne_blur_job_dst;
static int ne_blur_job_src_stride, ne_blur_job_dst_stride;
static int ne_blur_job_w, ne_blur_job_h;
static u32 ne_blur_kernel_cycles, ne_blur_job_cycles;
static u32 ne_blur_failures;
static u16 ne_blur_fifo_wa;

// Emulators: small images only (w * h <= NEA_DSP_SCRATCH_WORDS), moved
// through the FIFO. Enough to check the DSP code where DMA doesn't work.
static int ne_blur_fifo(const u16 *src, int src_stride, u16 *dst,
                        int dst_stride, int w, int h)
{
    if (w * h > NEA_DSP_SCRATCH_WORDS)
        return 0;

    if (ne_blur_fifo_wa == 0)
    {
        u32 wa = 0;
        if (__NEA_DspInfo(NEA_DSP_INFO_BLUR, &wa) != NEA_DSP_STATUS_OK)
            return 0;
        ne_blur_fifo_wa = wa;
    }

    u16 scratch = __NEA_DspScratchAddr();
    for (int y = 0; y < h; y++)
        dspFifoWriteData(src + y * src_stride, scratch + y * w, w);

    NEA_DspDescInit(&ne_blur_desc, NEA_DSP_JOB_BLUR);
    ne_blur_desc.w[NEA_DSP_D_FLAGS] = NEA_DSP_FLAG_LOCAL;
    ne_blur_desc.w[NEA_DSP_D_WORDS] = w;
    ne_blur_desc.w[NEA_DSP_BL_HEIGHT] = h;

    NEA_DspJobResult r;
    if (NEA_DspJobRun(&ne_blur_desc, &r) != NEA_DSP_STATUS_OK)
        return 0;

    ne_blur_kernel_cycles = r.result;
    ne_blur_job_cycles = r.cycles;

    u16 out = ne_blur_fifo_wa + NEA_FX_BLUR_WA_LOCAL_OUT;
    for (int y = 0; y < h; y++)
        dspFifoReadData(out + y * w, dst + y * dst_stride, w);

    return 2;
}

int NEA_DspBlurBegin(const u16 *src, int src_stride, u16 *dst, int dst_stride,
                     int width, int height)
{
    NEA_AssertPointer(src, "NULL source");
    NEA_AssertPointer(dst, "NULL destination");
    NEA_Assert(width > 0 && width <= NEA_FX_BLUR_MAX_W
               && height > 0 && height <= NEA_FX_BLUR_MAX_H
               && src_stride >= width && dst_stride >= width,
               "Invalid blur rectangle");

    if (!NEA_DspIsUsedForWork())
    {
        NEA_DspBlurApplyCPU(src, src_stride, dst, dst_stride, width, height);
        return 2;
    }

    if (NEA_DspJobIsPending())
        return 0;

    if (NEA_DspGetTransport() == NEA_DSP_TRANSPORT_FIFO)
    {
        if (ne_blur_fifo(src, src_stride, dst, dst_stride, width, height) == 0)
            NEA_DspBlurApplyCPU(src, src_stride, dst, dst_stride, width,
                                height);
        return 2;
    }

    // See NEA_DspGradeBeginRect()
    DC_FlushAll();

    NEA_DspDescInit(&ne_blur_desc, NEA_DSP_JOB_BLUR);
    NEA_DspDescSetAddr(&ne_blur_desc, NEA_DSP_D_ADDR_A, src);
    NEA_DspDescSetAddr(&ne_blur_desc, NEA_DSP_D_ADDR_B, dst);
    ne_blur_desc.w[NEA_DSP_D_WORDS] = width;
    ne_blur_desc.w[NEA_DSP_BL_HEIGHT] = height;
    ne_blur_desc.w[NEA_DSP_BL_SRC_STRIDE] = src_stride;
    ne_blur_desc.w[NEA_DSP_BL_DST_STRIDE] = dst_stride;

    if (!NEA_DspJobBegin(&ne_blur_desc))
        return 0;

    ne_blur_pending = true;
    ne_blur_job_src = src;
    ne_blur_job_dst = dst;
    ne_blur_job_src_stride = src_stride;
    ne_blur_job_dst_stride = dst_stride;
    ne_blur_job_w = width;
    ne_blur_job_h = height;
    return 1;
}

int NEA_DspBlurEnd(void)
{
    if (!ne_blur_pending)
        return 2;

    NEA_DspJobResult r;
    int status = NEA_DspJobWait(&r);

    // Drops any line of dst that got into the cache during the job
    DC_FlushAll();

    ne_blur_pending = false;

    if (status != NEA_DSP_STATUS_OK)
    {
        NEA_DebugPrint("DSP blur failed: %d", status);
        ne_blur_failures++;

        // A stalled transfer can leave the DSP's bus access wedged
        if (status == NEA_DSP_STATUS_DMA_ERROR || status < 0)
            NEA_DspReset();

        // In place, part of the image may be blurred already
        if (ne_blur_job_src == ne_blur_job_dst)
            return 0;

        NEA_DspBlurApplyCPU(ne_blur_job_src, ne_blur_job_src_stride,
                            ne_blur_job_dst, ne_blur_job_dst_stride,
                            ne_blur_job_w, ne_blur_job_h);
        return 2;
    }

    ne_blur_kernel_cycles = r.result;
    ne_blur_job_cycles = r.cycles;
    return 1;
}

int NEA_DspBlurApply(const u16 *src, int src_stride, u16 *dst, int dst_stride,
                     int width, int height)
{
    int ret = NEA_DspBlurBegin(src, src_stride, dst, dst_stride, width, height);
    if (ret == 1)
        return NEA_DspBlurEnd();
    if (ret == 0)
    {
        // The DSP is busy with another job
        NEA_DspBlurApplyCPU(src, src_stride, dst, dst_stride, width, height);
        return 2;
    }
    return ret;
}

void NEA_DspBlurGetStats(u32 *kernel_cycles, u32 *job_cycles)
{
    if (kernel_cycles)
        *kernel_cycles = ne_blur_kernel_cycles;
    if (job_cycles)
        *job_cycles = ne_blur_job_cycles;
}

u32 NEA_DspBlurGetFailureCount(void)
{
    return ne_blur_failures;
}

//-----------------------------------------------------------------------------
// Bloom
//-----------------------------------------------------------------------------

void NEA_DspBloomApplyCPU(const u16 *src, int src_stride, u16 *dst,
                          int dst_stride, int width, int height, int threshold)
{
    NEA_Assert(width >= 2 && width <= NEA_FX_BLOOM_MAX_W && !(width & 1)
               && height >= 2 && height <= NEA_FX_BLOOM_MAX_H && !(height & 1)
               && threshold >= 0 && threshold <= 31, "Invalid bloom");

    int n = width / 2, hh = height / 2;

    // Every half row blurred across, then each bloom row blurred down and
    // added. The whole source is read before any of it is written, so this
    // also works in place.
    u16 *half = malloc(hh * 2 * n * sizeof(u16));
    u16 *tmp = malloc((2 * NEA_FX_BLOOM_TMP_STRIDE + 2 * n) * sizeof(u16));
    if (half == NULL || tmp == NULL)
    {
        free(half);
        free(tmp);
        return;
    }

    u16 *tmp_rb = tmp;
    u16 *tmp_g = tmp + NEA_FX_BLOOM_TMP_STRIDE;
    u16 *bloom_rb = tmp + 2 * NEA_FX_BLOOM_TMP_STRIDE;
    u16 *bloom_g = bloom_rb + n;

    for (int k = 0; k < hh; k++)
    {
        nea_fx_bloom_down(src + 2 * k * src_stride,
                          src + (2 * k + 1) * src_stride, tmp_rb + 2,
                          tmp_g + 2, n, threshold);
        tmp_rb[0] = tmp_rb[1] = tmp_rb[2];
        tmp_rb[n + 3] = tmp_rb[n + 2] = tmp_rb[n + 1];
        tmp_g[0] = tmp_g[1] = tmp_g[2];
        tmp_g[n + 3] = tmp_g[n + 2] = tmp_g[n + 1];
        nea_fx_bloom_blur_h(tmp_rb, half + k * 2 * n, n,
                            NEA_FX_BLOOM_RB_ROUND16, 0x7C1F);
        nea_fx_bloom_blur_h(tmp_g, half + k * 2 * n + n, n,
                            NEA_FX_BLOOM_G_ROUND16, 0x03E0);
    }

    for (int k = 0; k < hh; k++)
    {
        const u16 *r[5];
        for (int i = 0; i < 5; i++)
        {
            int q = k + i - 2;
            q = q < 0 ? 0 : (q > hh - 1 ? hh - 1 : q);
            r[i] = half + q * 2 * n;
        }
        nea_fx_bloom_blur_v(r[0], r[1], r[2], r[3], r[4], bloom_rb, n,
                            NEA_FX_BLOOM_RB_ROUND16, 0x7C1F);
        nea_fx_bloom_blur_v(r[0] + n, r[1] + n, r[2] + n, r[3] + n, r[4] + n,
                            bloom_g, n, NEA_FX_BLOOM_G_ROUND16, 0x03E0);

        for (int i = 0; i < 2; i++)
        {
            int y = 2 * k + i;
            nea_fx_bloom_add_row(src + y * src_stride, bloom_rb, bloom_g,
                                 dst + y * dst_stride, width);
        }
    }

    free(half);
    free(tmp);
}

// Static, not local: the DSP reads it with DMA and can't see DTCM
static NEA_DspDesc ne_bloom_desc;

static bool ne_bloom_pending;
static const u16 *ne_bloom_job_src;
static u16 *ne_bloom_job_dst;
static int ne_bloom_job_src_stride, ne_bloom_job_dst_stride;
static int ne_bloom_job_w, ne_bloom_job_h, ne_bloom_job_t;
static u32 ne_bloom_kernel_cycles, ne_bloom_job_cycles;
static u32 ne_bloom_failures;
static u16 ne_bloom_fifo_wa;

// Emulators: small images only (at most 16 rows, w * h <= the scratch
// buffer), moved through the FIFO. Enough to check the DSP code.
static int ne_bloom_fifo(const u16 *src, int src_stride, u16 *dst,
                         int dst_stride, int w, int h, int t)
{
    if (h > 2 * NEA_FX_BLOOM_HALF_ROWS || w * h > NEA_DSP_SCRATCH_WORDS)
        return 0;

    if (ne_bloom_fifo_wa == 0)
    {
        u32 wa = 0;
        if (__NEA_DspInfo(NEA_DSP_INFO_BLOOM, &wa) != NEA_DSP_STATUS_OK)
            return 0;
        ne_bloom_fifo_wa = wa;
    }

    u16 scratch = __NEA_DspScratchAddr();
    for (int y = 0; y < h; y++)
        dspFifoWriteData(src + y * src_stride, scratch + y * w, w);

    NEA_DspDescInit(&ne_bloom_desc, NEA_DSP_JOB_BLOOM);
    ne_bloom_desc.w[NEA_DSP_D_FLAGS] = NEA_DSP_FLAG_LOCAL;
    ne_bloom_desc.w[NEA_DSP_D_WORDS] = w;
    ne_bloom_desc.w[NEA_DSP_BM_HEIGHT] = h;
    ne_bloom_desc.w[NEA_DSP_BM_THRESHOLD] = t;

    NEA_DspJobResult r;
    if (NEA_DspJobRun(&ne_bloom_desc, &r) != NEA_DSP_STATUS_OK)
        return 0;

    ne_bloom_kernel_cycles = r.result;
    ne_bloom_job_cycles = r.cycles;

    u16 out = ne_bloom_fifo_wa + NEA_FX_BLOOM_WA_LOCAL_OUT;
    for (int y = 0; y < h; y++)
        dspFifoReadData(out + y * w, dst + y * dst_stride, w);

    return 2;
}

int NEA_DspBloomBegin(const u16 *src, int src_stride, u16 *dst, int dst_stride,
                      int width, int height, int threshold)
{
    NEA_AssertPointer(src, "NULL source");
    NEA_AssertPointer(dst, "NULL destination");
    NEA_Assert(width >= 2 && width <= NEA_FX_BLOOM_MAX_W && !(width & 1)
               && height >= 2 && height <= NEA_FX_BLOOM_MAX_H && !(height & 1)
               && src_stride >= width && dst_stride >= width
               && threshold >= 0 && threshold <= 31, "Invalid bloom");

    if (!NEA_DspIsUsedForWork())
    {
        NEA_DspBloomApplyCPU(src, src_stride, dst, dst_stride, width, height,
                             threshold);
        return 2;
    }

    if (NEA_DspJobIsPending())
        return 0;

    if (NEA_DspGetTransport() == NEA_DSP_TRANSPORT_FIFO)
    {
        if (ne_bloom_fifo(src, src_stride, dst, dst_stride, width, height,
                          threshold) == 0)
            NEA_DspBloomApplyCPU(src, src_stride, dst, dst_stride, width,
                                 height, threshold);
        return 2;
    }

    // See NEA_DspGradeBeginRect()
    DC_FlushAll();

    NEA_DspDescInit(&ne_bloom_desc, NEA_DSP_JOB_BLOOM);
    NEA_DspDescSetAddr(&ne_bloom_desc, NEA_DSP_D_ADDR_A, src);
    NEA_DspDescSetAddr(&ne_bloom_desc, NEA_DSP_D_ADDR_B, dst);
    ne_bloom_desc.w[NEA_DSP_D_WORDS] = width;
    ne_bloom_desc.w[NEA_DSP_BM_HEIGHT] = height;
    ne_bloom_desc.w[NEA_DSP_BM_SRC_STRIDE] = src_stride;
    ne_bloom_desc.w[NEA_DSP_BM_DST_STRIDE] = dst_stride;
    ne_bloom_desc.w[NEA_DSP_BM_THRESHOLD] = threshold;

    if (!NEA_DspJobBegin(&ne_bloom_desc))
        return 0;

    ne_bloom_pending = true;
    ne_bloom_job_src = src;
    ne_bloom_job_dst = dst;
    ne_bloom_job_src_stride = src_stride;
    ne_bloom_job_dst_stride = dst_stride;
    ne_bloom_job_w = width;
    ne_bloom_job_h = height;
    ne_bloom_job_t = threshold;
    return 1;
}

int NEA_DspBloomEnd(void)
{
    if (!ne_bloom_pending)
        return 2;

    NEA_DspJobResult r;
    int status = NEA_DspJobWait(&r);

    // Drops any line of dst that got into the cache during the job
    DC_FlushAll();

    ne_bloom_pending = false;

    if (status != NEA_DSP_STATUS_OK)
    {
        NEA_DebugPrint("DSP bloom failed: %d", status);
        ne_bloom_failures++;

        // A stalled transfer can leave the DSP's bus access wedged
        if (status == NEA_DSP_STATUS_DMA_ERROR || status < 0)
            NEA_DspReset();

        // In place, part of the image may be done already
        if (ne_bloom_job_src == ne_bloom_job_dst)
            return 0;

        NEA_DspBloomApplyCPU(ne_bloom_job_src, ne_bloom_job_src_stride,
                             ne_bloom_job_dst, ne_bloom_job_dst_stride,
                             ne_bloom_job_w, ne_bloom_job_h, ne_bloom_job_t);
        return 2;
    }

    ne_bloom_kernel_cycles = r.result;
    ne_bloom_job_cycles = r.cycles;
    return 1;
}

int NEA_DspBloomApply(const u16 *src, int src_stride, u16 *dst, int dst_stride,
                      int width, int height, int threshold)
{
    int ret = NEA_DspBloomBegin(src, src_stride, dst, dst_stride, width,
                                height, threshold);
    if (ret == 1)
        return NEA_DspBloomEnd();
    if (ret == 0)
    {
        // The DSP is busy with another job
        NEA_DspBloomApplyCPU(src, src_stride, dst, dst_stride, width, height,
                             threshold);
        return 2;
    }
    return ret;
}

void NEA_DspBloomGetStats(u32 *kernel_cycles, u32 *job_cycles)
{
    if (kernel_cycles)
        *kernel_cycles = ne_bloom_kernel_cycles;
    if (job_cycles)
        *job_cycles = ne_bloom_job_cycles;
}

u32 NEA_DspBloomGetFailureCount(void)
{
    return ne_bloom_failures;
}

//-----------------------------------------------------------------------------
// Render target and full-screen effects: shared
//-----------------------------------------------------------------------------

// VRAM banks the reflection and the full-screen effects own, for
// NEA_TextureSystemReset() (weak reference there)
static NEA_VRAMBankFlags ne_dspfx_reserved;

NEA_VRAMBankFlags NEA_DspFXGetReservedBanks(void)
{
    return ne_dspfx_reserved;
}

// Bank index (0-3 for A-D) of a single bank flag, or -1
static int ne_dspfx_bank_index(NEA_VRAMBankFlags bank)
{
    for (int i = 0; i < 4; i++)
    {
        if (bank == (NEA_VRAMBankFlags)(1 << i))
            return i;
    }
    return -1;
}

// ARM9 address of a bank mapped as LCD
static u16 *ne_dspfx_lcd_address(int bank)
{
    return VRAM_A + bank * (128 * 1024 / 2);
}

// Maps a bank as LCD (capture destination, DSP access) or as main BG slot 0
static void ne_dspfx_map_bank(int bank, bool as_bg)
{
    switch (bank)
    {
        case 0:
            vramSetBankA(as_bg ? VRAM_A_MAIN_BG_0x06000000 : VRAM_A_LCD);
            break;
        case 1:
            vramSetBankB(as_bg ? VRAM_B_MAIN_BG_0x06000000 : VRAM_B_LCD);
            break;
        case 2:
            vramSetBankC(as_bg ? VRAM_C_MAIN_BG_0x06000000 : VRAM_C_LCD);
            break;
        default:
            vramSetBankD(as_bg ? VRAM_D_MAIN_BG_0x06000000 : VRAM_D_LCD);
            break;
    }
}

// Whether another system already drives the display capture
static bool ne_dspfx_capture_busy(void)
{
    extern NEA_VRAMBankFlags NEA_PostFXGetCaptureBanks(void) __attribute__((weak));
    if (NEA_PostFXGetCaptureBanks && NEA_PostFXGetCaptureBanks())
        return true;
    return ne_dspfx_reserved != 0;
}

//-----------------------------------------------------------------------------
// Reflection render target
//-----------------------------------------------------------------------------

#define NE_REFLECT_W        128     // Texture width
#define NE_REFLECT_HALF     (64 * 1024 / 2) // One capture half, in pixels

static struct {
    bool active;
    int bank;
    NEA_Material *mat;
    int water_line, rows;
    int armed_half;         // Half the armed capture writes, or -1
    int ready_half;         // Half holding a complete capture, or -1
    bool upload;            // The texture has a new image to upload
    NEA_DspDisplaceTables *ripple;
    NEA_DspGradeTables *tint;
    bool tint_on;
    u32 dsp_cycles;
} ne_reflect;

int NEA_DspReflectInit(NEA_VRAMBankFlags bank, int water_line, int rows)
{
    int b = ne_dspfx_bank_index(bank);
    if (b < 0 || water_line < 1 || water_line > 128 || rows < 1
        || rows > water_line || rows > 128)
    {
        NEA_DebugPrint("Invalid reflection parameters");
        return 0;
    }
    if (NEA_CurrentExecutionMode() != NEA_ModeSingle3D)
    {
        NEA_DebugPrint("The reflection needs single 3D mode");
        return 0;
    }
    if (ne_dspfx_capture_busy())
    {
        NEA_DebugPrint("The display capture is already in use");
        return 0;
    }
    ne_reflect.ripple = NEA_DspDisplaceTablesCreate();
    if (ne_reflect.ripple == NULL)
        return 0;

    // No ripple until one is set: a still mirror
    NEA_DspDisplace d;
    NEA_DspDisplaceInit(&d, NE_REFLECT_W, rows);
    NEA_DspDisplaceBuildScaled(ne_reflect.ripple, &d, 256);

    ne_reflect.active = true;
    ne_reflect.bank = b;
    ne_reflect.mat = NULL;
    ne_reflect.water_line = water_line;
    ne_reflect.rows = rows;
    ne_reflect.armed_half = -1;
    ne_reflect.ready_half = -1;
    ne_reflect.upload = false;
    ne_reflect.tint = NULL;
    ne_reflect.tint_on = false;

    ne_dspfx_map_bank(b, false);
    ne_dspfx_reserved |= bank;
    return 1;
}

int NEA_DspReflectSetTarget(NEA_Material *mat)
{
    if (!ne_reflect.active)
        return 0;

    if (mat != NULL && NEA_MaterialTexGetData(mat) == NULL)
    {
        NEA_DebugPrint("The material must come from NEA_MaterialTexBlank()");
        return 0;
    }

    ne_reflect.mat = mat;
    ne_reflect.upload = false;
    return 1;
}

void NEA_DspReflectEnd(void)
{
    if (!ne_reflect.active)
        return;

    REG_DISPCAPCNT = 0;
    NEA_DspDisplaceTablesDelete(ne_reflect.ripple);
    NEA_DspGradeTablesDelete(ne_reflect.tint);
    ne_dspfx_reserved &= ~(NEA_VRAMBankFlags)(1 << ne_reflect.bank);
    ne_reflect.active = false;
}

int NEA_DspReflectSetRipple(const NEA_DspDisplace *d)
{
    NEA_AssertPointer(d, "NULL displacement");

    if (!ne_reflect.active)
        return -1;

    NEA_Assert(d->width == NE_REFLECT_W && d->height == ne_reflect.rows,
               "The ripple field must be 128 x rows");

    return NEA_DspDisplaceBuildScaled(ne_reflect.ripple, d, 256);
}

int NEA_DspReflectSetTint(const NEA_DspGrade *g)
{
    if (!ne_reflect.active)
        return -1;

    if (g == NULL)
    {
        ne_reflect.tint_on = false;
        return 0;
    }

    if (ne_reflect.tint == NULL)
    {
        ne_reflect.tint = NEA_DspGradeTablesCreate();
        if (ne_reflect.tint == NULL)
            return -1;
    }

    if (NEA_DspGradeBuild(ne_reflect.tint, g) != 0)
        return -1;

    ne_reflect.tint_on = true;
    return 0;
}

// In VBlank: upload the last image, then capture the frame about to be shown
static void ne_reflect_update(void)
{
    if (ne_reflect.upload && ne_reflect.mat != NULL)
    {
        NEA_MaterialTexVramUpdate(ne_reflect.mat);
        ne_reflect.upload = false;
    }

    // The capture armed at the previous VBlank covered the top 128 lines of
    // the frame just shown, so it is complete now
    if (ne_reflect.armed_half >= 0)
        ne_reflect.ready_half = ne_reflect.armed_half;

    int half = (ne_reflect.armed_half == 0) ? 1 : 0;
    ne_reflect.armed_half = half;

    REG_DISPCAPCNT = DCAP_BANK(ne_reflect.bank)
                   | DCAP_SIZE(DCAP_SIZE_256x128)
                   | DCAP_MODE(DCAP_MODE_A)
                   | DCAP_SRC_A(DCAP_SRC_A_3DONLY)
                   | DCAP_OFFSET(half * 2)  // In 32 KB units: 0 or 64 KB
                   | DCAP_ENABLE;
}

int NEA_DspReflectProcess(void)
{
    if (!ne_reflect.active || ne_reflect.ready_half < 0
        || ne_reflect.mat == NULL)
        return 0;

    const u16 *half = ne_dspfx_lcd_address(ne_reflect.bank)
                      + ne_reflect.ready_half * NE_REFLECT_HALF;

    // The texture's row 0 is the screen row just above the water line, and
    // later rows go up: the source is read upwards, from that row
    const u16 *src = half + (ne_reflect.water_line - 1) * 256;
    u16 *dst = NEA_MaterialTexGetData(ne_reflect.mat);

    ne_reflect.dsp_cycles = 0;
    u32 kernel, job;

    int r = NEA_DspDisplaceApply(ne_reflect.ripple, src, -256, dst,
                                 NE_REFLECT_W);
    if (r == 1)
    {
        NEA_DspDisplaceGetStats(&kernel, &job);
        ne_reflect.dsp_cycles += job;
    }

    if (ne_reflect.tint_on)
    {
        r = NEA_DspGradeApply(ne_reflect.tint, dst, dst,
                              NE_REFLECT_W * ne_reflect.rows);
        if (r == 1)
        {
            NEA_DspGradeGetStats(&kernel, &job);
            ne_reflect.dsp_cycles += job;
        }
    }

    NEA_MaterialTexSetDirty(ne_reflect.mat);
    ne_reflect.upload = true;
    return 1;
}

u32 NEA_DspReflectGetStats(void)
{
    return ne_reflect.dsp_cycles;
}

//-----------------------------------------------------------------------------
// Full-screen effects
//-----------------------------------------------------------------------------

typedef enum {
    NE_SFX_ARM,         // Capture the next frame into the hidden bank
    NE_SFX_CAPTURING,   // Waiting for the capture to complete
    NE_SFX_PROCESSING,  // Effect jobs running on the hidden bank
    NE_SFX_DONE         // Ready to be shown at the next VBlank
} ne_sfx_state;

typedef enum {
    NE_SFX_STEP_BLUR,
    NE_SFX_STEP_BLOOM,
    NE_SFX_STEP_GRADE,
    NE_SFX_STEP_END
} ne_sfx_step;

static struct {
    bool active;
    int bank[2];
    int shown;              // Index in bank[] of the bank displayed as BG2
    ne_sfx_state state;
    u32 saved_mode;

    const NEA_DspGradeTables *grade;
    int blur_passes;
    int bloom_threshold;    // -1: off

    ne_sfx_step step;       // Chain position
    int step_pass;          // Blur pass within the blur step
    bool job_pending;       // A DSP job of the current step is running
    ne_sfx_step job_step;

    u32 frames;             // VBlanks since the last swap
    u32 last_frames;        // VBlanks the last image took: 2 = 30 fps
    u32 captured;           // Frames captured so far
} ne_sfx;

static u16 *ne_sfx_work(void)
{
    return ne_dspfx_lcd_address(ne_sfx.bank[ne_sfx.shown ^ 1]);
}

int NEA_DspScreenFXInit(NEA_VRAMBankFlags banks)
{
    int idx[2], found = 0;
    for (int i = 0; i < 4; i++)
    {
        if (banks & (1 << i))
        {
            if (found < 2)
                idx[found] = i;
            found++;
        }
    }

    if (found != 2 || (banks & ~NEA_VRAM_ABCD))
    {
        NEA_DebugPrint("Full-screen effects need exactly 2 banks of A-D");
        return 0;
    }
    if (NEA_CurrentExecutionMode() != NEA_ModeSingle3D)
    {
        NEA_DebugPrint("Full-screen effects need single 3D mode");
        return 0;
    }
    if (ne_dspfx_capture_busy())
    {
        NEA_DebugPrint("The display capture is already in use");
        return 0;
    }

    ne_sfx.active = true;
    ne_sfx.bank[0] = idx[0];
    ne_sfx.bank[1] = idx[1];
    ne_sfx.shown = 1;
    ne_sfx.state = NE_SFX_ARM;
    ne_sfx.grade = NULL;
    ne_sfx.blur_passes = 0;
    ne_sfx.bloom_threshold = -1;
    ne_sfx.job_pending = false;
    ne_sfx.frames = 0;
    ne_sfx.last_frames = 0;
    ne_sfx.captured = 0;

    // Black until the first processed frame
    ne_dspfx_map_bank(idx[0], false);
    ne_dspfx_map_bank(idx[1], false);
    dmaFillWords(0, ne_dspfx_lcd_address(idx[1]), 256 * 192 * 2);
    dmaFillWords(0, ne_dspfx_lcd_address(idx[0]), 256 * 192 * 2);
    ne_dspfx_map_bank(idx[1], true);

    // BG2: the processed frame, as a 16-bit bitmap in front of the live 3D
    // image (BG0), which it hides: every processed pixel is opaque
    ne_sfx.saved_mode = REG_DISPCNT;
    REG_BG2CNT = BG_BMP16_256x256 | BG_BMP_BASE(0) | BG_PRIORITY(0);
    REG_BG2PA = 1 << 8;
    REG_BG2PB = 0;
    REG_BG2PC = 0;
    REG_BG2PD = 1 << 8;
    REG_BG2X = 0;
    REG_BG2Y = 0;
    videoSetMode(MODE_5_3D | DISPLAY_BG2_ACTIVE
                 | (ne_sfx.saved_mode & (DISPLAY_WIN0_ON | DISPLAY_WIN1_ON
                                         | DISPLAY_SPR_WIN_ON)));
    REG_BG0CNT = (REG_BG0CNT & ~BG_PRIORITY(3)) | BG_PRIORITY(1);

    ne_dspfx_reserved |= banks;
    return 1;
}

void NEA_DspScreenFXEnd(void)
{
    if (!ne_sfx.active)
        return;

    // Let a running job finish before its bank goes away
    NEA_DspScreenFXPoll();
    if (ne_sfx.job_pending)
    {
        switch (ne_sfx.job_step)
        {
            case NE_SFX_STEP_BLUR:
                NEA_DspBlurEnd();
                break;
            case NE_SFX_STEP_BLOOM:
                NEA_DspBloomEnd();
                break;
            default:
                NEA_DspGradeEnd();
                break;
        }
        ne_sfx.job_pending = false;
    }

    REG_DISPCAPCNT = 0;
    ne_dspfx_map_bank(ne_sfx.bank[0], false);
    ne_dspfx_map_bank(ne_sfx.bank[1], false);
    videoSetMode(ne_sfx.saved_mode);
    REG_BG0CNT = (REG_BG0CNT & ~BG_PRIORITY(3)) | BG_PRIORITY(0);

    ne_dspfx_reserved &= ~(NEA_VRAMBankFlags)((1 << ne_sfx.bank[0])
                                              | (1 << ne_sfx.bank[1]));
    ne_sfx.active = false;
}

void NEA_DspScreenFXSetGrade(const NEA_DspGradeTables *t)
{
    ne_sfx.grade = t;
}

void NEA_DspScreenFXSetBlur(int passes)
{
    ne_sfx.blur_passes = passes < 0 ? 0 : passes;
}

void NEA_DspScreenFXSetBloom(int threshold)
{
    ne_sfx.bloom_threshold = (threshold < 0 || threshold > 31) ? -1 : threshold;
}

// Starts the job of the current step, if it has one. Returns false when the
// DSP is busy with someone else's job (try again later).
static bool ne_sfx_start_step(void)
{
    u16 *img = ne_sfx_work();

    while (ne_sfx.step != NE_SFX_STEP_END)
    {
        int r = 2;

        switch (ne_sfx.step)
        {
            case NE_SFX_STEP_BLUR:
                if (ne_sfx.step_pass < ne_sfx.blur_passes)
                    r = NEA_DspBlurBegin(img, 256, img, 256, 256, 192);
                else
                    r = -1;
                break;
            case NE_SFX_STEP_BLOOM:
                if (ne_sfx.bloom_threshold >= 0)
                    r = NEA_DspBloomBegin(img, 256, img, 256, 256, 192,
                                          ne_sfx.bloom_threshold);
                else
                    r = -1;
                break;
            default:
                if (ne_sfx.grade != NULL)
                    r = NEA_DspGradeBegin(ne_sfx.grade, img, img, 256 * 192);
                else
                    r = -1;
                break;
        }

        if (r == 0)
            return false;   // Busy

        if (r == 1)
        {
            ne_sfx.job_pending = true;
            ne_sfx.job_step = ne_sfx.step;
            return true;
        }

        // Done already (on the ARM9), or nothing to do: next pass or step
        if (r == 2 && ne_sfx.step == NE_SFX_STEP_BLUR)
        {
            ne_sfx.step_pass++;
            continue;
        }
        ne_sfx.step++;
        ne_sfx.step_pass = 0;
    }

    return true;
}

void NEA_DspScreenFXPoll(void)
{
    if (!ne_sfx.active || ne_sfx.state != NE_SFX_PROCESSING)
        return;

    while (1)
    {
        if (ne_sfx.job_pending)
        {
            if (!NEA_DspJobIsDone())
                return;

            switch (ne_sfx.job_step)
            {
                case NE_SFX_STEP_BLUR:
                    NEA_DspBlurEnd();
                    ne_sfx.step_pass++;
                    break;
                case NE_SFX_STEP_BLOOM:
                    NEA_DspBloomEnd();
                    ne_sfx.step++;
                    break;
                default:
                    NEA_DspGradeEnd();
                    ne_sfx.step++;
                    break;
            }
            ne_sfx.job_pending = false;
        }

        if (!ne_sfx_start_step())
            return;     // DSP busy: try again at the next poll

        if (ne_sfx.job_pending)
            return;     // Running

        if (ne_sfx.step == NE_SFX_STEP_END)
        {
            ne_sfx.state = NE_SFX_DONE;
            return;
        }
    }
}

// In VBlank
static void ne_sfx_update(void)
{
    ne_sfx.frames++;

    switch (ne_sfx.state)
    {
        case NE_SFX_DONE:
            // Show the processed bank, and capture into the other one
            ne_sfx.shown ^= 1;
            ne_dspfx_map_bank(ne_sfx.bank[ne_sfx.shown ^ 1], false);
            ne_dspfx_map_bank(ne_sfx.bank[ne_sfx.shown], true);
            ne_sfx.last_frames = ne_sfx.frames;
            ne_sfx.frames = 0;
            // Fall through: arm now
        case NE_SFX_ARM:
            REG_DISPCAPCNT = DCAP_BANK(ne_sfx.bank[ne_sfx.shown ^ 1])
                           | DCAP_SIZE(DCAP_SIZE_256x192)
                           | DCAP_MODE(DCAP_MODE_A)
                           | DCAP_SRC_A(DCAP_SRC_A_3DONLY)
                           | DCAP_ENABLE;
            ne_sfx.state = NE_SFX_CAPTURING;
            break;

        case NE_SFX_CAPTURING:
            // The frame just shown has been captured
            ne_sfx.captured++;
            ne_sfx.step = NE_SFX_STEP_BLUR;
            ne_sfx.step_pass = 0;
            ne_sfx.state = NE_SFX_PROCESSING;
            NEA_DspScreenFXPoll();
            break;

        case NE_SFX_PROCESSING:
            NEA_DspScreenFXPoll();
            break;
    }
}

bool NEA_DspScreenFXFrameCaptured(void)
{
    // True in the frame whose image the capture armed at this VBlank records
    return ne_sfx.active && ne_sfx.state == NE_SFX_CAPTURING;
}

int NEA_DspScreenFXGetRate(void)
{
    if (ne_sfx.last_frames == 0)
        return 0;
    return 60 / ne_sfx.last_frames;
}

void NEA_DspFXUpdate(void)
{
    if (ne_reflect.active)
        ne_reflect_update();
    if (ne_sfx.active)
        ne_sfx_update();
}
