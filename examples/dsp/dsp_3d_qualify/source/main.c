// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Nitro Engine Advanced contributors, 2026
//
// This file is part of Nitro Engine Advanced
//
// Display lists and DSP jobs in the same frame (DSi hardware only)
//
// A DMA feeding the GFX FIFO while the DSP moves data wedges the DSP's bus
// access. NEA_DisplayListDrawDefault() now pauses the DSP's transfers for each
// display list it sends by DMA while a DSP job runs (a gate on the APBP
// semaphores; the DSP keeps computing). 150 frames each:
//
//   1 no 3D    Control: job while an empty scene is processed
//   2 after    Scene first, then the job
//   3 gt DMA   Job during the scene, legacy DMA display lists, gated (default)
//   4 gt NDMA  Job during the scene, NDMA display lists, gated
//   5 CPU      Job during the scene, CPU display lists (the slow safe path)
//   6 DMA bug  Job during the scene, legacy DMA, no gate: must still fail
//
// "scene" is the ARM9 time spent submitting the scene and "job" the DSP time
// of the job, both averaged, in microseconds. A failed job restarts the DSP
// and is graded on the ARM9, so "bad" stays 0: "fail" is the answer.

#include <malloc.h>
#include <stdio.h>

#include <NEAMain.h>

#include "teapot_bin.h"
#include "teapot.h"

#define W       256
#define H       192
#define PIXELS  (W * H)

// More than single-screen 3D can hold in polygon RAM (it is why the two-pass
// demo uses them), so some teapots don't show: the point is to keep the
// geometry engine and its display lists at full load.
#define NUM_MODELS 6
#define FRAMES_PER_CONDITION 150

typedef struct {
    NEA_Camera *Camera;
    NEA_Model *Model[NUM_MODELS];
    NEA_Material *Material;
} SceneData;

static void Draw3DScene(void *arg)
{
    SceneData *Scene = arg;

    NEA_CameraUse(Scene->Camera);

    for (int i = 0; i < NUM_MODELS; i++)
        NEA_ModelDraw(Scene->Model[i]);
}

static void DrawNothing(void *arg)
{
    (void)arg;
}

static void init_scene(SceneData *Scene)
{
    for (int i = 0; i < NUM_MODELS; i++)
        Scene->Model[i] = NEA_ModelCreate(NEA_Static);

    Scene->Camera = NEA_CameraCreate();
    Scene->Material = NEA_MaterialCreate();

    NEA_CameraSet(Scene->Camera, 0, 3, -8, 0, 0, 0, 0, 1, 0);

    NEA_ModelLoadStaticMesh(Scene->Model[0], teapot_bin);
    NEA_MaterialTexLoad(Scene->Material, NEA_A1RGB5, 256, 256,
                        NEA_TEXGEN_TEXCOORD | NEA_TEXTURE_WRAP_S
                        | NEA_TEXTURE_WRAP_T,
                        teapotBitmap);
    NEA_ModelSetMaterial(Scene->Model[0], Scene->Material);

    for (int i = 1; i < NUM_MODELS; i++)
        NEA_ModelClone(Scene->Model[i], Scene->Model[0]);

    int idx = 0;
    for (int row = -1; row <= 0; row++)
    {
        for (int col = -1; col <= 1; col++)
        {
            NEA_ModelSetCoord(Scene->Model[idx], col * 3, row * 3, 0);
            idx++;
        }
    }

    NEA_LightSet(0, NEA_White, -0.5, -0.5, -0.5);
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
                int h = (x * 6 * 32) / W;
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
                int t = (y * 64) / (H - 24);
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

// Pixels of a width x H rectangle (rows W apart) that differ
static int count_bad_rect(const u16 *a, const u16 *b, int width)
{
    int bad = 0;
    for (int y = 0; y < H; y++)
    {
        for (int x = 0; x < width; x++)
        {
            if (a[y * W + x] != b[y * W + x])
                bad++;
        }
    }
    return bad;
}

typedef struct {
    const char *name;
    NEA_DisplayListDrawFunction dl;
    bool guard;
    u32 jobs;       // Jobs the DSP took
    u32 fails;      // Of those, failed (graded on the ARM9 instead)
    u32 bad;        // Wrong pixels in the checked results
    u32 scene_us;   // ARM9 time submitting the scene, summed
    u32 job_us;     // DSP time of the successful jobs, summed
    u32 frames;
} condition;

#define NUM_CONDS 6

static condition conds[NUM_CONDS] = {
    { "no 3D", NEA_DL_DMA_GFX_FIFO, true },
    { "after", NEA_DL_DMA_GFX_FIFO, true },
    { "gt DMA", NEA_DL_DMA_GFX_FIFO, true },
    { "gt NDMA", NEA_DL_NDMA_GFX_FIFO, true },
    { "CPU", NEA_DL_CPU, true },
    { "DMA bug", NEA_DL_DMA_GFX_FIFO, false },
};

static u32 last_detail, last_addr;
static int last_cond = -1;

static u32 ticks_to_us(u32 ticks)
{
    return (u32)(((u64)ticks * 1000000) / BUS_CLOCK);
}

int main(int argc, char *argv[])
{
    SceneData Scene = { 0 };

    irqEnable(IRQ_HBLANK);
    irqSet(IRQ_VBLANK, NEA_VBLFunc);
    irqSet(IRQ_HBLANK, NEA_HBLFunc);

    NEA_Init3D();
    consoleDemoInit(); // Sub screen: the main one shows the 3D scene
    init_scene(&Scene);

    u16 *src = memalign(32, PIXELS * sizeof(u16));
    u16 *dst = memalign(32, PIXELS * sizeof(u16));
    u16 *ref = memalign(32, PIXELS * sizeof(u16));
    NEA_DspGradeTables *tables = NEA_DspGradeTablesCreate();
    if (!src || !dst || !ref || !tables)
    {
        printf("Out of memory\n");
        while (1)
            swiWaitForVBlank();
    }

    make_image(src);

    NEA_DspGrade g;
    NEA_DspGradeInit(&g);
    NEA_DspGradeSepia(&g, 256);
    NEA_DspGradeBuild(tables, &g);
    NEA_DspGradeApplyCPU(tables, src, ref, PIXELS);

    bool dsp = NEA_DspInit()
               && NEA_DspGetTransport() == NEA_DSP_TRANSPORT_DMA;

    u32 frame = 0;
    int current = -1;

    while (1)
    {
        NEA_WaitForVBL(0);

        scanKeys();
        if (keysDown() & KEY_START)
            break;

        int c = (frame / FRAMES_PER_CONDITION) % NUM_CONDS;
        condition *cond = &conds[c];

        if (c != current)
        {
            current = c;
            NEA_DisplayListSetDefaultFunction(cond->dl);
            NEA_DisplayListSetDspGuard(cond->guard);
        }

        u32 fails_before = NEA_DspGradeGetFailureCount();
        int began = 0;

        cpuStartTiming(0);
        u32 scene_ticks = 0;

        if (c == 1)
        {
            // After the scene
            NEA_ProcessArg(Draw3DScene, &Scene);
            scene_ticks = cpuEndTiming();
            began = NEA_DspGradeBegin(tables, src, dst, PIXELS);
            NEA_DspGradeEnd();
        }
        else
        {
            began = NEA_DspGradeBegin(tables, src, dst, PIXELS);

            cpuStartTiming(0);
            if (c == 0)
                NEA_ProcessArg(DrawNothing, NULL);
            else
                NEA_ProcessArg(Draw3DScene, &Scene);
            scene_ticks = cpuEndTiming();

            NEA_DspGradeEnd();
        }

        cond->scene_us += ticks_to_us(scene_ticks);
        cond->frames++;

        if (began == 1)
        {
            cond->jobs++;

            if (NEA_DspGradeGetFailureCount() != fails_before)
            {
                cond->fails++;
                NEA_DspGradeGetLastStatus(&last_detail, &last_addr);
                last_cond = c;
            }
            else
            {
                u32 job;
                NEA_DspGradeGetStats(NULL, &job);
                cond->job_us += NEA_DspCyclesToUs(job);
            }

            if ((cond->jobs % 10) == 0)
                cond->bad += count_bad_rect(ref, dst, W);
        }

        for (int i = 0; i < NUM_MODELS; i++)
            NEA_ModelRotate(Scene.Model[i], 0, 2, 1);

        printf("\x1b[0;0H"
               "Display lists vs DSP jobs\n"
               "DSP: %s\n"
               "\n"
               "  cond    jobs fail scene   job\n",
               dsp ? "DMA (DSi)" : "no DSP DMA here, ARM9 only");

        u32 bad_total = 0;
        for (int i = 0; i < NUM_CONDS; i++)
        {
            u32 us = conds[i].frames ? conds[i].scene_us / conds[i].frames : 0;
            u32 ok = conds[i].jobs - conds[i].fails;
            u32 job = ok ? conds[i].job_us / ok : 0;
            printf("%d%c%-7.7s%5lu%5lu%6lu%6lu\n", i + 1,
                   i == c ? '>' : ' ', conds[i].name, conds[i].jobs,
                   conds[i].fails, us, job);
            bad_total += conds[i].bad;
        }

        printf("\n(scene: ARM9 us, job: DSP us)\n"
               "Wrong pixels checked: %lu\n"
               "\nLast failure: cond %d\n"
               "  %08lX @%08lX\n"
               "DSP restarts: %lu  FPS: %d  \n"
               "\n"
               "Conditions change every %d\n"
               "frames. Wait for 2 rounds.\n"
               "START: exit\n",
               bad_total, last_cond + 1, last_detail, last_addr,
               NEA_DspGetResetCount(),
               NEA_GetFPS(), FRAMES_PER_CONDITION);

        frame++;
    }

    NEA_DspEnd();
    NEA_End();

    return 0;
}
