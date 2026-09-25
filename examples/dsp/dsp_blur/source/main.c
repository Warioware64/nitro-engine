// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Nitro Engine Advanced contributors, 2026
//
// This file is part of Nitro Engine Advanced
//
// Blur on the DSi's DSP, checked against the ARM9.
//
// Top screen: ARM9 time and DSP time (whole job, then the kernels alone) to
// blur a 256x192 image, into another buffer and in place; into VRAM; a
// 128x96 image (bloom works at half size); and ten random 256x16 images (in
// melonDS those go through the FIFO, which checks the DSP code there). The
// number of pixels where DSP and ARM9 disagree must be 0 everywhere.
//
// Bottom screen: the image blurred 0 to 4 times. A: more passes. B (held):
// original. START: exit.

#include <malloc.h>
#include <stdio.h>
#include <stdlib.h>

#include <NEAMain.h>

#define W       256
#define H       192
#define PIXELS  (W * H)

static PrintConsole top;
static u16 *src, *cpu_out, *dsp_out;

static u32 ticks_to_us(u32 ticks)
{
    return (u32)(((u64)ticks * 1000000) / BUS_CLOCK);
}

// Sharp edges and fine stripes, where a blur shows
static void make_image(u16 *p)
{
    for (int y = 0; y < H; y++)
    {
        for (int x = 0; x < W; x++)
        {
            int r, g, b;
            if (((x >> 5) ^ (y >> 5)) & 1)
            {
                r = 31; g = 28; b = 4;
            }
            else
            {
                r = 2; g = 6; b = 20;
            }
            if ((x & 7) == 0 || (y & 7) == 0)
            {
                r = 31 - r; g = 31 - g; b = 31 - b;
            }
            p[y * W + x] = RGB15(r, g, b) | BIT(15);
        }
    }
}

static int count_bad(const u16 *a, int a_stride, const u16 *b, int b_stride,
                     int w, int h)
{
    int bad = 0;
    for (int y = 0; y < h; y++)
    {
        for (int x = 0; x < w; x++)
        {
            if (a[y * a_stride + x] != b[y * b_stride + x])
                bad++;
        }
    }
    return bad;
}

static void print_row(const char *name, u32 cpu_us, u32 dsp_us, int bad)
{
    u32 kernel, job;
    NEA_DspBlurGetStats(&kernel, &job);
    printf("%-8s%6lu%6lu%6lu %d\n", name, cpu_us, dsp_us,
           NEA_DspCyclesToUs(kernel), bad);
}

int main(void)
{
    videoSetMode(MODE_0_2D);
    vramSetBankA(VRAM_A_MAIN_BG);
    consoleInit(&top, 0, BgType_Text4bpp, BgSize_T_256x256, 31, 0, true, true);

    videoSetModeSub(MODE_5_2D);
    vramSetBankC(VRAM_C_SUB_BG);
    int bg = bgInitSub(3, BgType_Bmp16, BgSize_B16_256x256, 0, 0);
    u16 *screen = bgGetGfxPtr(bg);

    src = memalign(32, PIXELS * sizeof(u16));
    cpu_out = memalign(32, PIXELS * sizeof(u16));
    dsp_out = memalign(32, PIXELS * sizeof(u16));
    if (!src || !cpu_out || !dsp_out)
    {
        printf("Out of memory\n");
        while (1)
            swiWaitForVBlank();
    }

    make_image(src);

    printf("NEA DSP blur\n");

    bool dsp = NEA_DspInit();
    if (!dsp)
    {
        printf("No DSP: ARM9 only\n");
    }
    else if (NEA_DspGetTransport() == NEA_DSP_TRANSPORT_FIFO)
    {
        printf("FIFO (emulator): only small\nimages go to the DSP\n");
        NEA_DspAllowFifoTransport(true);
    }
    else
    {
        printf("Transport: DMA\n");
    }

    printf("us:       ARM9   DSP kernl bad\n");

    // 256x192, into another buffer
    cpuStartTiming(0);
    NEA_DspBlurApplyCPU(src, W, cpu_out, W, W, H);
    u32 cpu_us = ticks_to_us(cpuEndTiming());
    cpuStartTiming(0);
    NEA_DspBlurApply(src, W, dsp_out, W, W, H);
    u32 dsp_us = ticks_to_us(cpuEndTiming());
    print_row("256x192", cpu_us, dsp_us, count_bad(cpu_out, W, dsp_out, W, W, H));

    // In place
    dmaCopy(src, dsp_out, PIXELS * sizeof(u16));
    cpuStartTiming(0);
    NEA_DspBlurApply(dsp_out, W, dsp_out, W, W, H);
    dsp_us = ticks_to_us(cpuEndTiming());
    print_row("in place", cpu_us, dsp_us,
              count_bad(cpu_out, W, dsp_out, W, W, H));

    // Into VRAM
    cpuStartTiming(0);
    NEA_DspBlurApply(src, W, screen, W, W, H);
    dsp_us = ticks_to_us(cpuEndTiming());
    print_row("->VRAM", cpu_us, dsp_us, count_bad(cpu_out, W, screen, W, W, H));

    // 128x96, the size bloom works at (a corner of the image, rows 256 apart)
    cpuStartTiming(0);
    NEA_DspBlurApplyCPU(src, W, cpu_out, 128, 128, 96);
    cpu_us = ticks_to_us(cpuEndTiming());
    cpuStartTiming(0);
    NEA_DspBlurApply(src, W, dsp_out, 128, 128, 96);
    dsp_us = ticks_to_us(cpuEndTiming());
    print_row("128x96", cpu_us, dsp_us,
              count_bad(cpu_out, 128, dsp_out, 128, 128, 96));

    // Random small images: in melonDS these are the ones the DSP runs
    {
        int bad = 0;
        srand(99);
        for (int n = 0; n < 10; n++)
        {
            // Random pixels as the source, written by the CPU: a DMA copy
            // here would read memory behind the data cache
            u16 *s = cpu_out + W * 32;
            for (int i = 0; i < W * 16; i++)
                s[i] = rand();
            NEA_DspBlurApplyCPU(s, W, cpu_out, W, W, 16);
            NEA_DspBlurApply(s, W, dsp_out, W, W, 16);
            bad += count_bad(cpu_out, W, dsp_out, W, W, 16);
        }
        printf("10 random 256x16: bad %d\n", bad);
    }

    printf("Failures %lu, restarts %lu\n", NEA_DspBlurGetFailureCount(),
           NEA_DspGetResetCount());
    printf("A: more passes  B: original\n");

    int passes = 2;
    bool redraw = true;

    while (1)
    {
        swiWaitForVBlank();
        scanKeys();
        u16 down = keysDown();

        if (down & KEY_START)
            break;
        if (down & KEY_A)
        {
            passes = (passes + 1) % 5;
            redraw = true;
        }
        if (down & KEY_B || keysUp() & KEY_B)
            redraw = true;

        if (!redraw)
            continue;
        redraw = false;

        dmaCopy(src, screen, PIXELS * sizeof(u16));
        u32 total = 0;
        if (!(keysHeld() & KEY_B))
        {
            for (int i = 0; i < passes; i++)
            {
                cpuStartTiming(0);
                NEA_DspBlurApply(screen, W, screen, W, W, H);
                total += ticks_to_us(cpuEndTiming());
            }
        }

        consoleSetCursor(&top, 0, 23);
        printf("%d passes: %6lu us     ", passes, total);
    }

    NEA_DspEnd();
    return 0;
}
