// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Nitro Engine Advanced contributors, 2026
//
// This file is part of Nitro Engine Advanced
//
// DSP transport benchmark (DSi hardware only).
//
// Measures how fast NEA's DSP program moves data between ARM9 memory and DSP
// memory, for every DMA speed and AHBM burst setting, and checks that each
// setting actually delivers correct data. The results decide how large an image
// the DSP effects can process per frame, so please photograph both screens.
//
// Bottom screen: the speed x burst matrix, in microseconds per 32 KB. "!" means
// the data arrived wrong (or the transfer failed), so that setting is unusable
// however fast it looks.
//
// Top screen: the command round trip, the effect of the chunk size, whole
// images (256x192 and 128x128, 16-bit) with the fastest correct settings,
// VRAM access, and how much the DSP's DMA slows the ARM9's own memory reads.

#include <malloc.h>
#include <stdio.h>

#include <NEAMain.h>

#define BENCH_WORDS     16384           // 32 KB, the matrix transfer size
#define FRAME_WORDS     (256 * 192)     // 96 KB, a whole 16-bit screen
#define RT_WORDS        (128 * 128)     // 32 KB, a 128x128 render target
#define SEED            0x5A17

static PrintConsole top, bottom;

// Static, not local: locals live in DTCM, which the DSP's DMA can't reach
static NEA_DspDesc desc;

static u16 *buf;        // FRAME_WORDS words, 1 KB aligned
static u16 *buf2;       // FRAME_WORDS words, 1 KB aligned

static const char *burst_name[3] = { "INCR ", "INCR4", "INCR8" };

static u32 arm9_ticks_to_us(u32 ticks)
{
    return (u32)(((u64)ticks * 1000000) / BUS_CLOCK);
}

static void fill_pattern(u16 *p, int words)
{
    for (int i = 0; i < words; i++)
        p[i] = nea_dsp_bench_pattern(i, SEED);
    DC_FlushRange(p, words * sizeof(u16));
}

// The DSP streams its scratch buffer, so word i of its output is pattern
// (i % NEA_DSP_SCRATCH_WORDS).
static int check_out_pattern(const u16 *p, int words)
{
    int bad = 0;
    for (int i = 0; i < words; i++)
    {
        if (p[i] != nea_dsp_bench_pattern(i % NEA_DSP_SCRATCH_WORDS, SEED))
            bad++;
    }
    return bad;
}

typedef struct {
    int status;     // NEA_DSP_STATUS_*, or -1
    int bad;        // Words that arrived wrong
    u32 dsp_us;     // DMA time only, measured by the DSP
    u32 arm9_us;    // Whole job seen from the ARM9, including the handshake
} xfer_result;

static xfer_result run_xfer(u16 *arm, int words, int dir, u16 xfer)
{
    xfer_result x = { -1, 0, 0, 0 };

    if (dir == NEA_DSP_XB_DIR_IN)
    {
        fill_pattern(arm, words);
    }
    else
    {
        for (int i = 0; i < words; i++)
            arm[i] = 0;
        DC_FlushRange(arm, words * sizeof(u16));
    }

    NEA_DspDescInit(&desc, NEA_DSP_JOB_XFER_BENCH);
    NEA_DspDescSetAddr(&desc, NEA_DSP_D_ADDR_A, arm);
    desc.w[NEA_DSP_D_WORDS] = words;
    desc.w[NEA_DSP_D_XFER] = xfer;
    desc.w[NEA_DSP_XB_DIR] = dir;
    desc.w[NEA_DSP_XB_SEED] = SEED;

    NEA_DspJobResult r;

    cpuStartTiming(0);
    x.status = NEA_DspJobRun(&desc, &r);
    x.arm9_us = arm9_ticks_to_us(cpuEndTiming());

    x.dsp_us = NEA_DspCyclesToUs(r.cycles);

    if (dir == NEA_DSP_XB_DIR_IN)
    {
        x.bad = (int)r.result;
    }
    else
    {
        DC_InvalidateRange(arm, words * sizeof(u16));
        x.bad = check_out_pattern(arm, words);
    }

    if (x.status != NEA_DSP_STATUS_OK && x.bad == 0)
        x.bad = -1;

    return x;
}

static bool xfer_ok(const xfer_result *x)
{
    return x->status == NEA_DSP_STATUS_OK && x->bad == 0;
}

// KB per millisecond is the same number as MB per second
static void print_rate(const char *label, int words, const xfer_result *x)
{
    if (!xfer_ok(x))
    {
        printf("%s FAIL st%d bad%d\n", label, x->status, x->bad);
        return;
    }

    u32 kb_x10 = (u32)words * 2 * 10 / 1024;
    u32 rate_x10 = x->dsp_us ? (kb_x10 * 1000) / x->dsp_us : 0;
    printf("%s %5lu us %2lu.%lu MB/s\n", label, x->dsp_us,
           rate_x10 / 10, rate_x10 % 10);
}

static u16 make_xfer(int speed, int burst, int chunk)
{
    return (u16)((speed << NEA_DSP_XFER_SPEED_SHIFT)
                 | (burst << NEA_DSP_XFER_BURST_SHIFT)
                 | (chunk << NEA_DSP_XFER_CHUNK_SHIFT));
}

// Reads "words" 32-bit words through the uncached mirror, so every read goes
// out on the bus and competes with the DSP's DMA.
static u32 arm9_bus_reads(const u32 *src, int words)
{
    const vu32 *p = memUncached((void *)src);
    u32 sum = 0;
    for (int i = 0; i < words; i++)
        sum += p[i];
    return sum;
}

static void print_xfer(const char *label, int words, int dir,
                       const xfer_result *x)
{
    printf("%s st%d bad%d/%d\n", label, x->status, x->bad, words);
    if (dir == NEA_DSP_XB_DIR_OUT && x->bad > 0)
    {
        printf("  [0..3] %04X %04X %04X %04X\n", buf[0], buf[1], buf[2], buf[3]);
        printf("  want   %04X %04X %04X %04X\n",
               nea_dsp_bench_pattern(0, SEED), nea_dsp_bench_pattern(1, SEED),
               nea_dsp_bench_pattern(2, SEED), nea_dsp_bench_pattern(3, SEED));
    }
}

// The init probe fetches its descriptor with DMA, then reads and writes data
// with DMA, so one failure says little. Here the descriptor goes through the
// FIFO instead (which works, or the transport would be "none"), and each DMA
// direction is tried on its own.
static void diagnose_dma(void)
{
    const NEA_DspProbeReport *rep = NEA_DspGetProbeReport();

    printf("\nDMA probe at init:\n");
    printf(" status %d, bad %d/256\n", rep->status, rep->bad);
    printf(" sum %08lX want %08lX\n", rep->result, rep->expected);
    if (rep->first_bad >= 0)
        printf(" [%d] = %04X want %04X\n", rep->first_bad, rep->got, rep->want);

    printf("\nDescriptor via FIFO:\n");
    NEA_DspAllowFifoTransport(true);

    NEA_DspDescInit(&desc, NEA_DSP_JOB_NOP);
    NEA_DspJobResult r;
    printf("NOP job: %d\n", NEA_DspJobRun(&desc, &r));

    // Status 3 = DMA error (timed out), 4 = wrong data
    for (int speed = 0; speed < 4; speed += 3)
    {
        u16 xfer = make_xfer(speed, 0, 3);
        xfer_result x;
        char label[24];

        snprintf(label, sizeof(label), "sp%d ARM9->DSP", speed);
        x = run_xfer(buf, 256, NEA_DSP_XB_DIR_IN, xfer);
        print_xfer(label, 256, NEA_DSP_XB_DIR_IN, &x);

        snprintf(label, sizeof(label), "sp%d DSP->ARM9", speed);
        x = run_xfer(buf, 256, NEA_DSP_XB_DIR_OUT, xfer);
        print_xfer(label, 256, NEA_DSP_XB_DIR_OUT, &x);
    }

    printf("Stale %lu, repeats %lu\n", NEA_DspGetStaleReplies(),
           NEA_DspGetRepeatedCommands());
}

int main(void)
{
    videoSetMode(MODE_0_2D);
    videoSetModeSub(MODE_0_2D);
    vramSetBankA(VRAM_A_MAIN_BG);
    vramSetBankC(VRAM_C_SUB_BG);

    consoleInit(&top, 0, BgType_Text4bpp, BgSize_T_256x256, 31, 0, true, true);
    consoleInit(&bottom, 0, BgType_Text4bpp, BgSize_T_256x256, 31, 0, false,
                true);

    consoleSelect(&top);
    printf("NEA DSP transport bench\n");

    buf = memalign(1024, FRAME_WORDS * sizeof(u16));
    buf2 = memalign(1024, FRAME_WORDS * sizeof(u16));
    if (!buf || !buf2)
    {
        printf("Out of memory\n");
        goto wait_exit;
    }

    if (!NEA_DspInit())
    {
        printf("NEA_DspInit() failed.\n"
               "Needs a DSi, a DSi-mode ROM and\n"
               "libNEA built with NEA_TEAK=1.\n");
        goto wait_exit;
    }

    printf("Transport: %s\n", NEA_DspGetTransportName());
    if (NEA_DspGetTransport() != NEA_DSP_TRANSPORT_DMA)
    {
        diagnose_dma();
        goto wait_exit;
    }

    printf("Self-test 16 KB: %d bad\n", NEA_DspSelfTest(NEA_DSP_SCRATCH_WORDS));

    // Command round trip
    {
        NEA_DspDescInit(&desc, NEA_DSP_JOB_NOP);
        NEA_DspJobResult r;
        const int n = 64;

        cpuStartTiming(0);
        for (int i = 0; i < n; i++)
            NEA_DspJobRun(&desc, &r);
        u32 us = arm9_ticks_to_us(cpuEndTiming());

        printf("NOP job round trip: %lu us\n", us / n);
    }

    // Matrix: speed x burst, both directions
    consoleSelect(&bottom);
    printf("us per 32 KB (DMA time only)\n");
    printf("sp burst   ARM9->DSP DSP->ARM9\n");

    u16 best_in = NEA_DSP_XFER_DEFAULT, best_out = NEA_DSP_XFER_DEFAULT;
    u32 best_in_us = 0xFFFFFFFF, best_out_us = 0xFFFFFFFF;

    for (int speed = 0; speed < 4; speed++)
    {
        for (int burst = 0; burst < 3; burst++)
        {
            u16 xfer = make_xfer(speed, burst, 3);

            xfer_result in = run_xfer(buf, BENCH_WORDS, NEA_DSP_XB_DIR_IN, xfer);
            xfer_result out = run_xfer(buf, BENCH_WORDS, NEA_DSP_XB_DIR_OUT, xfer);

            printf("%d  %s  %6lu%c   %6lu%c\n", speed, burst_name[burst],
                   in.dsp_us, xfer_ok(&in) ? ' ' : '!',
                   out.dsp_us, xfer_ok(&out) ? ' ' : '!');

            if (xfer_ok(&in) && in.dsp_us < best_in_us)
            {
                best_in_us = in.dsp_us;
                best_in = xfer;
            }
            if (xfer_ok(&out) && out.dsp_us < best_out_us)
            {
                best_out_us = out.dsp_us;
                best_out = xfer;
            }
        }
    }

    printf("! = wrong data or failed\n");
    printf("Best in: sp%d %s\n", best_in & 3, burst_name[(best_in >> 2) & 3]);
    printf("Best out: sp%d %s\n", best_out & 3,
           burst_name[(best_out >> 2) & 3]);

    // Chunk size, with the best speed and burst for reading
    printf("\nChunk (ARM9->DSP, best)\n");
    for (int chunk = 0; chunk < 4; chunk++)
    {
        u16 xfer = (best_in & ~NEA_DSP_XFER_CHUNK_MASK)
                   | (chunk << NEA_DSP_XFER_CHUNK_SHIFT);
        xfer_result x = run_xfer(buf, BENCH_WORDS, NEA_DSP_XB_DIR_IN, xfer);
        char label[16];
        snprintf(label, sizeof(label), " %3d words", 64 << chunk);
        print_rate(label, BENCH_WORDS, &x);
    }

    consoleSelect(&top);

    // Whole images with the best settings
    printf("-- Images, best settings --\n");
    {
        xfer_result x;

        x = run_xfer(buf, FRAME_WORDS, NEA_DSP_XB_DIR_IN, best_in);
        print_rate("256x192 in ", FRAME_WORDS, &x);
        u32 frame_in = x.dsp_us;
        x = run_xfer(buf, FRAME_WORDS, NEA_DSP_XB_DIR_OUT, best_out);
        print_rate("256x192 out", FRAME_WORDS, &x);
        u32 frame_out = x.dsp_us;
        printf("256x192 round trip %lu us\n", frame_in + frame_out);

        x = run_xfer(buf, RT_WORDS, NEA_DSP_XB_DIR_IN, best_in);
        print_rate("128x128 in ", RT_WORDS, &x);
        u32 rt_in = x.dsp_us;
        x = run_xfer(buf, RT_WORDS, NEA_DSP_XB_DIR_OUT, best_out);
        print_rate("128x128 out", RT_WORDS, &x);
        printf("128x128 round trip %lu us\n", rt_in + x.dsp_us);
    }

    // VRAM in LCD mode, as a captured frame would be
    printf("-- VRAM D (LCD), best --\n");
    {
        vramSetBankD(VRAM_D_LCD);
        u16 *vram = VRAM_D;
        xfer_result x;

        x = run_xfer(vram, BENCH_WORDS, NEA_DSP_XB_DIR_IN, best_in);
        print_rate("VRAM in ", BENCH_WORDS, &x);
        x = run_xfer(vram, BENCH_WORDS, NEA_DSP_XB_DIR_OUT, best_out);
        print_rate("VRAM out", BENCH_WORDS, &x);
    }

    // What the ARM9 pays: its own bus reads while the DSP writes a frame
    printf("-- ARM9 bus contention --\n");
    {
        const int words32 = FRAME_WORDS / 2;

        cpuStartTiming(0);
        arm9_bus_reads((const u32 *)buf2, words32);
        u32 idle_us = arm9_ticks_to_us(cpuEndTiming());

        // A frame-sized DSP -> ARM9 transfer into buf, while the ARM9 reads buf2
        for (int i = 0; i < FRAME_WORDS; i++)
            buf[i] = 0;
        DC_FlushRange(buf, FRAME_WORDS * sizeof(u16));

        NEA_DspDescInit(&desc, NEA_DSP_JOB_XFER_BENCH);
        NEA_DspDescSetAddr(&desc, NEA_DSP_D_ADDR_A, buf);
        desc.w[NEA_DSP_D_WORDS] = FRAME_WORDS;
        desc.w[NEA_DSP_D_XFER] = best_out;
        desc.w[NEA_DSP_XB_DIR] = NEA_DSP_XB_DIR_OUT;
        desc.w[NEA_DSP_XB_SEED] = SEED;

        NEA_DspJobResult r;
        NEA_DspJobBegin(&desc);
        cpuStartTiming(0);
        arm9_bus_reads((const u32 *)buf2, words32);
        u32 busy_us = arm9_ticks_to_us(cpuEndTiming());
        NEA_DspJobWait(&r);

        printf("ARM9 96 KB reads: %lu us idle\n", idle_us);
        printf("  %lu us during DSP DMA\n", busy_us);
        printf("DSP 96 KB out: %lu us\n", NEA_DspCyclesToUs(r.cycles));

        // Reference: the ARM9's own DMA moving the same amount
        DC_FlushRange(buf2, FRAME_WORDS * sizeof(u16));
        cpuStartTiming(0);
        dmaCopyWords(3, buf2, buf, FRAME_WORDS * sizeof(u16));
        printf("ARM9 DMA 96 KB copy: %lu us\n",
               arm9_ticks_to_us(cpuEndTiming()));
    }

    printf("Stale replies %lu, repeats %lu\n", NEA_DspGetStaleReplies(),
           NEA_DspGetRepeatedCommands());

wait_exit:
    printf("START: exit");
    while (1)
    {
        swiWaitForVBlank();
        scanKeys();
        if (keysDown() & KEY_START)
            break;
    }

    NEA_DspEnd();
    return 0;
}
