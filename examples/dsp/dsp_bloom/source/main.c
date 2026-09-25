// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Nitro Engine Advanced contributors, 2026
//
// This file is part of Nitro Engine Advanced
//
// Bloom on the DSi's DSP, checked against the ARM9.
//
// Top screen: ARM9 time and DSP time (whole job, then the kernels alone) to
// add bloom to a 256x192 image, into another buffer and in place; into VRAM;
// a 128x96 image; and ten random 256x16 images (in melonDS those go through
// the FIFO, which checks the DSP code there). The number of pixels where DSP
// and ARM9 disagree must be 0 everywhere.
//
// Bottom screen: the image with bloom. A: change threshold. B (held):
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

// A dark scene with bright lamps and a white ramp: what glows is obvious
static void make_image(u16 *p)
{
    for (int y = 0; y < H; y++)
    {
        for (int x = 0; x < W; x++)
        {
            int r = 3 + ((x + y) & 3), g = 4, b = 8;

            // Four lamps
            for (int i = 0; i < 4; i++)
            {
                int cx = 40 + i * 58, cy = 60 + (i & 1) * 40;
                int dx = x - cx, dy = y - cy;
                if (dx * dx + dy * dy < 64)
                {
                    r = (i == 0 || i == 3) ? 31 : 28;
                    g = (i == 1 || i == 3) ? 31 : 20;
                    b = (i == 2 || i == 3) ? 31 : 12;
                }
            }

            // A bar going from black to white along the bottom
            if (y >= 150 && y < 160)
                r = g = b = x >> 3;

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
    NEA_DspBloomGetStats(&kernel, &job);
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

    printf("NEA DSP bloom\n");

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

    const int t = 18;

    printf("us:       ARM9   DSP kernl bad\n");

    cpuStartTiming(0);
    NEA_DspBloomApplyCPU(src, W, cpu_out, W, W, H, t);
    u32 cpu_us = ticks_to_us(cpuEndTiming());
    cpuStartTiming(0);
    NEA_DspBloomApply(src, W, dsp_out, W, W, H, t);
    u32 dsp_us = ticks_to_us(cpuEndTiming());
    print_row("256x192", cpu_us, dsp_us, count_bad(cpu_out, W, dsp_out, W, W, H));

    // In place (the source is copied by the CPU: no stale cache for the DSP)
    for (int i = 0; i < PIXELS; i++)
        dsp_out[i] = src[i];
    cpuStartTiming(0);
    NEA_DspBloomApply(dsp_out, W, dsp_out, W, W, H, t);
    dsp_us = ticks_to_us(cpuEndTiming());
    print_row("in place", cpu_us, dsp_us,
              count_bad(cpu_out, W, dsp_out, W, W, H));

    cpuStartTiming(0);
    NEA_DspBloomApply(src, W, screen, W, W, H, t);
    dsp_us = ticks_to_us(cpuEndTiming());
    print_row("->VRAM", cpu_us, dsp_us, count_bad(cpu_out, W, screen, W, W, H));

    // 128x96 from the middle of the image, rows 256 apart
    const u16 *mid = src + 48 * W + 64;
    cpuStartTiming(0);
    NEA_DspBloomApplyCPU(mid, W, cpu_out, 128, 128, 96, t);
    cpu_us = ticks_to_us(cpuEndTiming());
    cpuStartTiming(0);
    NEA_DspBloomApply(mid, W, dsp_out, 128, 128, 96, t);
    dsp_us = ticks_to_us(cpuEndTiming());
    print_row("128x96", cpu_us, dsp_us,
              count_bad(cpu_out, 128, dsp_out, 128, 128, 96));

    // Random small images: in melonDS these are the ones the DSP runs
    {
        int bad = 0;
        srand(7);
        for (int n = 0; n < 10; n++)
        {
            // Written by the CPU: a DMA copy would read behind the cache
            u16 *s = cpu_out + W * 32;
            for (int i = 0; i < W * 16; i++)
                s[i] = rand();
            int tt = rand() % 32;
            NEA_DspBloomApplyCPU(s, W, cpu_out, W, W, 16, tt);
            NEA_DspBloomApply(s, W, dsp_out, W, W, 16, tt);
            bad += count_bad(cpu_out, W, dsp_out, W, W, 16);
        }
        printf("10 random 256x16: bad %d\n", bad);
    }

    printf("Failures %lu, restarts %lu\n", NEA_DspBloomGetFailureCount(),
           NEA_DspGetResetCount());
    printf("A: threshold  B: original\n");

    static const int thresholds[] = { 12, 18, 24, 28 };
    int ti = 1;
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
            ti = (ti + 1) % 4;
            redraw = true;
        }
        if ((down & KEY_B) || (keysUp() & KEY_B))
            redraw = true;

        if (!redraw)
            continue;
        redraw = false;

        u32 us = 0;
        if (keysHeld() & KEY_B)
        {
            dmaCopy(src, screen, PIXELS * sizeof(u16));
        }
        else
        {
            cpuStartTiming(0);
            NEA_DspBloomApply(src, W, screen, W, W, H, thresholds[ti]);
            us = ticks_to_us(cpuEndTiming());
        }

        consoleSetCursor(&top, 0, 23);
        printf("threshold %2d: %6lu us     ", thresholds[ti], us);
    }

    NEA_DspEnd();
    return 0;
}
