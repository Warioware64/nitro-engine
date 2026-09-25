// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Nitro Engine Advanced contributors, 2026
//
// This file is part of Nitro Engine Advanced
//
// Colour grading on the DSi's DSP, checked against the ARM9.
//
// Top screen: for each preset, the time the ARM9 takes to grade a 256x192
// image, the time the DSP takes (whole job, then the kernel alone), and the
// number of pixels where the two disagree, which must be 0. Then two VRAM
// tests: the DSP writing straight into the bottom screen's bitmap, and grading
// VRAM in place, as it would a captured frame.
//
// Bottom screen: the graded image. It changes preset every 2 seconds. A: pause,
// B (held): show the original.
//
// On a DSi with DMA the numbers are the real ones. Elsewhere (melonDS) the DSP
// is driven through the FIFO, which is slow but still checks its results.

#include <malloc.h>
#include <stdio.h>

#include <NEAMain.h>

#define W       256
#define H       192
#define PIXELS  (W * H)

static PrintConsole top;

static u16 *src;        // Test image
static u16 *cpu_out;    // ARM9 result
static u16 *dsp_out;    // DSP result

static NEA_DspGradeTables *tables;

typedef struct {
    const char *name;
    void (*setup)(NEA_DspGrade *g);
} preset;

static void p_identity(NEA_DspGrade *g) { (void)g; }
static void p_grey(NEA_DspGrade *g) { NEA_DspGradeSaturate(g, 0); }
static void p_sepia(NEA_DspGrade *g) { NEA_DspGradeSepia(g, 256); }
static void p_flashback(NEA_DspGrade *g)
{
    NEA_DspGradeSaturate(g, 96);
    NEA_DspGradeSepia(g, 160);
    NEA_DspGradeContrast(g, 320, 1);
}
static void p_hue(NEA_DspGrade *g) { NEA_DspGradeHueRotate(g, degreesToAngle(120)); }
static void p_invert(NEA_DspGrade *g) { NEA_DspGradeInvert(g); }
static void p_night(NEA_DspGrade *g)
{
    NEA_DspGradeSaturate(g, 128);
    NEA_DspGradeTint(g, 150, 180, 256, 0);
    NEA_DspGradeContrast(g, 300, -3);
}
static void p_vivid(NEA_DspGrade *g) { NEA_DspGradeSaturate(g, 400); }

static const preset presets[] = {
    { "identity", p_identity },
    { "grey", p_grey },
    { "sepia", p_sepia },
    { "flashbk", p_flashback },
    { "hue+120", p_hue },
    { "invert", p_invert },
    { "night", p_night },
    { "vivid", p_vivid },
};
#define PRESET_COUNT (int)(sizeof(presets) / sizeof(presets[0]))

static u32 ticks_to_us(u32 ticks)
{
    return (u32)(((u64)ticks * 1000000) / BUS_CLOCK);
}

// Hue across, brightness down, and a grey ramp along the bottom
static void make_image(u16 *p)
{
    for (int y = 0; y < H; y++)
    {
        for (int x = 0; x < W; x++)
        {
            int r, g, b;

            if (y >= H - 24)
            {
                r = g = b = x >> 3;
            }
            else
            {
                int h = (x * 6 * 32) / W;   // 0..191, 32 per sextant
                int f = h & 31;
                switch (h >> 5)
                {
                    case 0: r = 31; g = f; b = 0; break;
                    case 1: r = 31 - f; g = 31; b = 0; break;
                    case 2: r = 0; g = 31; b = f; break;
                    case 3: r = 0; g = 31 - f; b = 31; break;
                    case 4: r = f; g = 0; b = 31; break;
                    default: r = 31; g = 0; b = 31 - f; break;
                }
                // Top half: towards white. Bottom half: towards black.
                int t = (y * 64) / (H - 24);    // 0..63
                if (t < 32)
                {
                    r += ((31 - r) * (31 - t)) >> 5;
                    g += ((31 - g) * (31 - t)) >> 5;
                    b += ((31 - b) * (31 - t)) >> 5;
                }
                else
                {
                    r = (r * (63 - t)) >> 5;
                    g = (g * (63 - t)) >> 5;
                    b = (b * (63 - t)) >> 5;
                }
            }

            p[y * W + x] = RGB15(r, g, b) | BIT(15);
        }
    }
}

static int count_bad(const u16 *a, const u16 *b, int n)
{
    int bad = 0;
    for (int i = 0; i < n; i++)
    {
        if (a[i] != b[i])
            bad++;
    }
    return bad;
}

static void build(int index)
{
    NEA_DspGrade g;
    NEA_DspGradeInit(&g);
    presets[index].setup(&g);
    if (NEA_DspGradeBuild(tables, &g) != 0)
        printf("%s: out of range!\n", presets[index].name);
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
    tables = NEA_DspGradeTablesCreate();
    if (!src || !cpu_out || !dsp_out || !tables)
    {
        printf("Out of memory\n");
        while (1)
            swiWaitForVBlank();
    }

    make_image(src);

    printf("NEA DSP colour grading 256x192\n");

    bool dsp = NEA_DspInit();
    if (!dsp)
    {
        printf("No DSP: ARM9 only\n");
    }
    else if (NEA_DspGetTransport() == NEA_DSP_TRANSPORT_FIFO)
    {
        printf("FIFO (emulator): DSP checked,\ntimes meaningless\n");
        NEA_DspAllowFifoTransport(true);
    }
    else
    {
        printf("Transport: DMA\n");
    }

    printf("us:       ARM9   DSP kernl bad\n");

    for (int i = 0; i < PRESET_COUNT; i++)
    {
        build(i);

        cpuStartTiming(0);
        NEA_DspGradeApplyCPU(tables, src, cpu_out, PIXELS);
        u32 cpu_us = ticks_to_us(cpuEndTiming());

        for (int j = 0; j < PIXELS; j++)
            dsp_out[j] = 0;

        cpuStartTiming(0);
        NEA_DspGradeApply(tables, src, dsp_out, PIXELS);
        u32 dsp_us = ticks_to_us(cpuEndTiming());

        u32 kernel, job;
        NEA_DspGradeGetStats(&kernel, &job);

        printf("%-8s%6lu%6lu%6lu %d\n", presets[i].name, cpu_us, dsp_us,
               NEA_DspCyclesToUs(kernel), count_bad(cpu_out, dsp_out, PIXELS));
    }

    // cpu_out holds the last preset's ARM9 result from here on

    // Transfers overlapped with grading (the default) against one after the
    // other, main RAM to main RAM
    for (int overlap = 0; overlap < 2; overlap++)
    {
        NEA_DspGradeSetOverlap(overlap);
        for (int j = 0; j < PIXELS; j++)
            dsp_out[j] = 0;
        cpuStartTiming(0);
        NEA_DspGradeApply(tables, src, dsp_out, PIXELS);
        u32 us = ticks_to_us(cpuEndTiming());
        u32 kernel;
        NEA_DspGradeGetStats(&kernel, NULL);
        // 32 columns: "overlap  20000 kern 9909 bad 0"
        printf("%s%6lu kern%5lu bad %d\n",
               overlap ? "overlap " : "serial  ", us,
               NEA_DspCyclesToUs(kernel), count_bad(cpu_out, dsp_out, PIXELS));
    }

    // The DSP writing straight into BG-mapped VRAM
    for (int j = 0; j < PIXELS; j++)
        screen[j] = 0;
    cpuStartTiming(0);
    NEA_DspGradeApply(tables, src, screen, PIXELS);
    u32 us = ticks_to_us(cpuEndTiming());
    printf("main->VRAM %5lu us, bad %d\n", us, count_bad(cpu_out, screen, PIXELS));

    // In place in VRAM, like a captured frame
    dmaCopy(src, screen, PIXELS * sizeof(u16));
    cpuStartTiming(0);
    NEA_DspGradeApply(tables, screen, screen, PIXELS);
    us = ticks_to_us(cpuEndTiming());
    printf("VRAM->VRAM %5lu us, bad %d\n", us, count_bad(cpu_out, screen, PIXELS));

    // What the ARM9 is left doing while the DSP works: starting the job
    {
        cpuStartTiming(0);
        int r = NEA_DspGradeBegin(tables, src, dsp_out, PIXELS);
        u32 begin_us = ticks_to_us(cpuEndTiming());
        NEA_DspGradeEnd();
        if (r == 1)
            printf("ARM9 time to start: %lu us\n", begin_us);
    }

    // The two-pass grading job (a 128-wide half, rows 256 apart) under what
    // two-pass mode runs at the same time: an HBlank DMA from main RAM, and
    // display capture. Each job starts right after VBlank, as it does there,
    // so it is still running when line 0 starts both.
    {
        static const char *names[4] = { "plain  ", "HBL DMA", "capture",
                                        "both   " };
        vramSetBankB(VRAM_B_LCD);
        vramSetBankD(VRAM_D_LCD);
        printf("128x192 rect, 20 jobs each:\n");

        for (int cond = 0; cond < 4; cond++)
        {
            int fails = 0;
            u32 detail = 0;

            for (int n = 0; n < 20; n++)
            {
                swiWaitForVBlank();

                if (cond & 1)
                {
                    DMA_CR(2) = 0;
                    DMA_SRC(2) = (u32)src;
                    DMA_DEST(2) = (u32)VRAM_B;
                    DMA_CR(2) = DMA_COPY_WORDS | DMA_START_HBL | DMA_REPEAT
                              | DMA_SRC_INC | DMA_DST_RESET | (256 * 2 / 4);
                }
                if (cond & 2)
                {
                    REG_DISPCAPCNT = DCAP_BANK(DCAP_BANK_VRAM_D)
                                   | DCAP_SIZE(DCAP_SIZE_256x192)
                                   | DCAP_MODE(DCAP_MODE_A)
                                   | DCAP_SRC_A(DCAP_SRC_A_COMPOSITED)
                                   | DCAP_ENABLE;
                }

                int r = NEA_DspGradeBeginRect(tables, src, W, dsp_out, W,
                                              128, H);
                if (r == 1 && NEA_DspGradeEnd() != 1)
                {
                    fails++;
                    NEA_DspGradeGetLastStatus(&detail, NULL);
                }
            }

            DMA_CR(2) = 0;
            REG_DISPCAPCNT = 0;
            printf(" %s %2d/20 fail %08lX\n", names[cond], fails, detail);
        }
    }

    printf("A: pause  B: original\n");

    int current = 0;
    int timer = 0;
    bool paused = false;
    bool showing_original = false;

    build(current);
    NEA_DspGradeApply(tables, src, screen, PIXELS);

    while (1)
    {
        swiWaitForVBlank();
        scanKeys();
        u16 down = keysDown();
        u16 held = keysHeld();

        if (down & KEY_START)
            break;
        if (down & KEY_A)
            paused = !paused;

        bool want_original = (held & KEY_B) != 0;
        bool next = false;

        if (!paused && ++timer >= 120)
        {
            timer = 0;
            current = (current + 1) % PRESET_COUNT;
            build(current);
            next = true;
        }

        if (want_original != showing_original || next)
        {
            showing_original = want_original;
            if (showing_original)
                dmaCopy(src, screen, PIXELS * sizeof(u16));
            else
                NEA_DspGradeApply(tables, src, screen, PIXELS);
        }

        consoleSetCursor(&top, 0, 23);
        printf("Showing: %-8s %s", presets[current].name,
               paused ? "(paused)" : "        ");
    }

    NEA_DspEnd();
    return 0;
}
