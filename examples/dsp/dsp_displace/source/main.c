// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Nitro Engine Advanced contributors, 2026
//
// This file is part of Nitro Engine Advanced
//
// Displacement (heat haze, ripples) on the DSi's DSP, checked against the
// ARM9.
//
// Top screen: for three fields, the time the ARM9 takes to displace a 256x192
// image, the time the DSP takes (whole job, then the kernel alone), and the
// number of pixels where they disagree, which must be 0. Then the DSP writing
// straight into VRAM, a 128x128 render-target-sized piece, and ten random
// 256x16 fields (in melonDS these go through the FIFO, which checks the DSP
// code there).
//
// Bottom screen: the effect, animated. A: heat haze / ripple. B (held): show
// the original. START: exit.

#include <malloc.h>
#include <stdio.h>
#include <stdlib.h>

#include <NEAMain.h>

#define W       256
#define H       192
#define PIXELS  (W * H)

static PrintConsole top;

static u16 *src, *cpu_out, *dsp_out;
static NEA_DspDisplace field;
static NEA_DspDisplaceTables *tables;

static u32 ticks_to_us(u32 ticks)
{
    return (u32)(((u64)ticks * 1000000) / BUS_CLOCK);
}

// A checkerboard with coloured stripes: easy to see the displacement on
static void make_image(u16 *p)
{
    for (int y = 0; y < H; y++)
    {
        for (int x = 0; x < W; x++)
        {
            int check = ((x >> 4) ^ (y >> 4)) & 1;
            int r = check ? 28 : 6;
            int g = check ? 26 : 8;
            int b = check ? 20 : 14;

            if (((x + y) & 31) < 4)
            {
                r = 31;
                g = (x * 31) / W;
                b = (y * 31) / H;
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

typedef enum {
    FIELD_HEAT,
    FIELD_RIPPLE,
    FIELD_MAX,
    FIELD_COUNT
} field_id;

static const char *field_names[FIELD_COUNT] = { "heat", "ripple", "max" };

static void set_field(field_id id, int w, int h, int time)
{
    NEA_DspDisplaceInit(&field, w, h);

    switch (id)
    {
        case FIELD_HEAT:
            NEA_DspDisplaceHeatHaze(&field, time, 3);
            break;
        case FIELD_RIPPLE:
            NEA_DspDisplaceRipple(&field, time, 4);
            break;
        default:
            // The largest offsets allowed, in both directions
            NEA_DspDisplaceAddWave(&field, NEA_DISPLACE_DX_ALONG_X, 16, 37, 0);
            NEA_DspDisplaceAddWave(&field, NEA_DISPLACE_DX_ALONG_Y, 16, 21, 0);
            NEA_DspDisplaceAddWave(&field, NEA_DISPLACE_DY_ALONG_X, 3, 13, 0);
            NEA_DspDisplaceAddWave(&field, NEA_DISPLACE_DY_ALONG_Y, 3, 17, 0);
            break;
    }

    if (NEA_DspDisplaceBuild(tables, &field) != 0)
        printf("%s: out of range!\n", field_names[id]);
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
    tables = NEA_DspDisplaceTablesCreate();
    if (!src || !cpu_out || !dsp_out || !tables)
    {
        printf("Out of memory\n");
        while (1)
            swiWaitForVBlank();
    }

    make_image(src);

    printf("NEA DSP displacement 256x192\n");

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

    printf("us:      ARM9   DSP kernl bad\n");

    for (int i = 0; i < FIELD_COUNT; i++)
    {
        set_field(i, W, H, 10);

        cpuStartTiming(0);
        NEA_DspDisplaceApplyCPU(tables, src, W, cpu_out, W);
        u32 cpu_us = ticks_to_us(cpuEndTiming());

        for (int j = 0; j < PIXELS; j++)
            dsp_out[j] = 0;

        u32 fails = NEA_DspDisplaceGetFailureCount();
        cpuStartTiming(0);
        int r = NEA_DspDisplaceApply(tables, src, W, dsp_out, W);
        u32 dsp_us = ticks_to_us(cpuEndTiming());

        u32 kernel, job;
        NEA_DspDisplaceGetStats(&kernel, &job);

        printf("%-7s%6lu%6lu%6lu %d%s\n", field_names[i], cpu_us, dsp_us,
               r == 1 ? NEA_DspCyclesToUs(kernel) : 0,
               count_bad(cpu_out, W, dsp_out, W, W, H),
               NEA_DspDisplaceGetFailureCount() != fails ? " F" : "");
    }

    // cpu_out holds the "max" field's ARM9 result from here on

    for (int j = 0; j < PIXELS; j++)
        screen[j] = 0;
    cpuStartTiming(0);
    NEA_DspDisplaceApply(tables, src, W, screen, W);
    u32 us = ticks_to_us(cpuEndTiming());
    printf("main->VRAM %5lu us, bad %d\n", us,
           count_bad(cpu_out, W, screen, W, W, H));

    // A 128x128 render target cut from the middle of the image
    {
        const u16 *rt_src = src + 32 * W + 64;
        set_field(FIELD_RIPPLE, 128, 128, 20);
        NEA_DspDisplaceApplyCPU(tables, rt_src, W, cpu_out, 128);
        for (int j = 0; j < 128 * 128; j++)
            dsp_out[j] = 0;
        cpuStartTiming(0);
        NEA_DspDisplaceApply(tables, rt_src, W, dsp_out, 128);
        us = ticks_to_us(cpuEndTiming());
        printf("128x128    %5lu us, bad %d\n", us,
               count_bad(cpu_out, 128, dsp_out, 128, 128, 128));
    }

    // Small random fields: in melonDS these are the ones the DSP runs
    {
        int bad = 0;
        u32 fails = NEA_DspDisplaceGetFailureCount();
        srand(1234);
        for (int n = 0; n < 10; n++)
        {
            NEA_DspDisplaceInit(&field, W, 16);
            for (int x = 0; x < W; x++)
            {
                field.ax[x] = (rand() % 33) - 16;
                field.bx[x] = (rand() % 7) - 3;
            }
            for (int y = 0; y < 16; y++)
            {
                field.ay[y] = (rand() % 33) - 16;
                field.by[y] = (rand() % 7) - 3;
            }
            NEA_DspDisplaceBuild(tables, &field);

            const u16 *s = src + (n * 16) * W;
            NEA_DspDisplaceApplyCPU(tables, s, W, cpu_out, W);
            NEA_DspDisplaceApply(tables, s, W, dsp_out, W);
            bad += count_bad(cpu_out, W, dsp_out, W, W, 16);
        }
        printf("10 random 256x16: bad %d%s\n", bad,
               NEA_DspDisplaceGetFailureCount() != fails ? " F" : "");
    }

    printf("DSP restarts: %lu\n", NEA_DspGetResetCount());
    printf("A: heat/ripple  B: original\n");

    field_id current = FIELD_HEAT;
    int time = 0;

    while (1)
    {
        swiWaitForVBlank();
        scanKeys();
        u16 down = keysDown();
        u16 held = keysHeld();

        if (down & KEY_START)
            break;
        if (down & KEY_A)
            current = (current == FIELD_HEAT) ? FIELD_RIPPLE : FIELD_HEAT;

        time++;

        if (held & KEY_B)
        {
            dmaCopy(src, screen, PIXELS * sizeof(u16));
        }
        else
        {
            set_field(current, W, H, time);
            cpuStartTiming(0);
            NEA_DspDisplaceApply(tables, src, W, screen, W);
            us = ticks_to_us(cpuEndTiming());
        }

        u32 kernel, job;
        NEA_DspDisplaceGetStats(&kernel, &job);
        consoleSetCursor(&top, 0, 22);
        // Both lines under 32 columns: a wrapped line would scroll the
        // results away
        printf("%-6s %5lu us, DSP job %5lu\n", field_names[current], us,
               NEA_DspCyclesToUs(job));
        printf("fails %lu, restarts %lu", NEA_DspDisplaceGetFailureCount(),
               NEA_DspGetResetCount());
    }

    NEA_DspEnd();
    return 0;
}
