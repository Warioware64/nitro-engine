// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Nitro Engine Advanced contributors, 2026
//
// This file is part of Nitro Engine Advanced
//
// A water reflection made on the DSi's DSP (screen-space mirror).
//
// Every frame the top of the 3D image is captured. The DSP mirrors the lines
// above the water line, halves their width, ripples them and tints them into
// a 128-wide texture, which the next frame draws under the water line.
//
// A: tint on/off. B: ripple on/off. START: exit.
//
// Without a DSP (or in melonDS) the ARM9 makes the same image.

#include <stdio.h>

#include <NEAMain.h>

#include "teapot_bin.h"
#include "teapot.h"

#define WATER_LINE  112             // Screen line of the water surface
#define ROWS        (192 - WATER_LINE) // Lines reflected: down to the bottom
#define NUM_MODELS  3

typedef struct {
    NEA_Camera *Camera;
    NEA_Model *Model[NUM_MODELS];
    NEA_Material *Material;
    NEA_Material *Water;
} SceneData;

static PrintConsole con;

static void Draw3DScene(void *arg)
{
    SceneData *Scene = arg;

    NEA_CameraUse(Scene->Camera);
    NEA_PolyFormat(31, 0, NEA_LIGHT_0, NEA_CULL_BACK, 0);

    for (int i = 0; i < NUM_MODELS; i++)
        NEA_ModelDraw(Scene->Model[i]);

    // The water: last frame's reflection, from the water line down
    NEA_2DViewInit();
    NEA_PolyFormat(31, 0, 0, NEA_CULL_NONE, 0);
    NEA_2DDrawTexturedQuadColorCanvas(0, WATER_LINE, 256, 192, 0,
                                      0, 0, 128, ROWS, Scene->Water,
                                      NEA_White);
}

static void init_scene(SceneData *Scene)
{
    for (int i = 0; i < NUM_MODELS; i++)
        Scene->Model[i] = NEA_ModelCreate(NEA_Static);

    Scene->Camera = NEA_CameraCreate();
    Scene->Material = NEA_MaterialCreate();

    NEA_CameraSet(Scene->Camera, 0, 0, -6, 0, 0, 0, 0, 1, 0);

    NEA_ModelLoadStaticMesh(Scene->Model[0], teapot_bin);
    NEA_MaterialTexLoad(Scene->Material, NEA_A1RGB5, 256, 256,
                        NEA_TEXGEN_TEXCOORD | NEA_TEXTURE_WRAP_S
                        | NEA_TEXTURE_WRAP_T,
                        teapotBitmap);
    NEA_ModelSetMaterial(Scene->Model[0], Scene->Material);

    for (int i = 1; i < NUM_MODELS; i++)
        NEA_ModelClone(Scene->Model[i], Scene->Model[0]);

    // In a row above the water line
    for (int i = 0; i < NUM_MODELS; i++)
        NEA_ModelSetCoord(Scene->Model[i], (i - 1) * 2.5, 0.7, 0);

    NEA_LightSet(0, NEA_White, -0.5, -0.5, -0.5);

    // A dusk sky: what isn't a teapot reflects as this colour
    NEA_ClearColorSet(RGB15(6, 8, 18), 31, 63);

    // The texture the reflection is written into
    Scene->Water = NEA_MaterialCreate();
    NEA_MaterialTexBlank(Scene->Water, NEA_A1RGB5, 128, 128,
                         NEA_TEXGEN_TEXCOORD);
}

int main(int argc, char *argv[])
{
    SceneData Scene = { 0 };

    irqEnable(IRQ_HBLANK);
    irqSet(IRQ_VBLANK, NEA_VBLFunc);
    irqSet(IRQ_HBLANK, NEA_HBLFunc);

    NEA_Init3D();

    // Console on the sub screen, on VRAM H (the 3D textures use A-C)
    videoSetModeSub(MODE_0_2D);
    vramSetBankH(VRAM_H_SUB_BG);
    consoleInit(&con, 0, BgType_Text4bpp, BgSize_T_256x256, 15, 0, false,
                true);

    bool dsp = NEA_DspInit()
               && NEA_DspGetTransport() == NEA_DSP_TRANSPORT_DMA;

    // The reflection owns bank D: before the texture system is set up
    if (!NEA_DspReflectInit(NEA_VRAM_D, WATER_LINE, ROWS))
    {
        printf("NEA_DspReflectInit() failed\n");
        while (1)
            swiWaitForVBlank();
    }
    NEA_TextureSystemReset(0, 0, NEA_VRAM_ABC);

    init_scene(&Scene);
    NEA_DspReflectSetTarget(Scene.Water);

    // Tint: a darker, bluer, less saturated reflection
    NEA_DspGrade tint;
    NEA_DspGradeInit(&tint);
    NEA_DspGradeSaturate(&tint, 180);
    NEA_DspGradeTint(&tint, 170, 200, 256, 0);
    NEA_DspGradeContrast(&tint, 256, -2);
    NEA_DspReflectSetTint(&tint);

    bool tint_on = true;
    bool ripple_on = true;
    int time = 0;

    NEA_DspDisplace ripple;

    while (1)
    {
        NEA_WaitForVBL(NEA_UPDATE_DSPFX);

        scanKeys();
        u16 down = keysDown();
        if (down & KEY_START)
            break;
        if (down & KEY_A)
        {
            tint_on = !tint_on;
            NEA_DspReflectSetTint(tint_on ? &tint : NULL);
        }
        if (down & KEY_B)
            ripple_on = !ripple_on;

        time++;
        NEA_DspDisplaceInit(&ripple, 128, ROWS);
        if (ripple_on)
            NEA_DspDisplaceRipple(&ripple, time, 2);
        NEA_DspReflectSetRipple(&ripple);

        for (int i = 0; i < NUM_MODELS; i++)
            NEA_ModelRotate(Scene.Model[i], 0, 2 + i, 1);

        NEA_ProcessArg(Draw3DScene, &Scene);

        // After the scene: the DSP never overlaps its display lists
        NEA_DspReflectProcess();

        printf("\x1b[0;0H"
               "DSP water reflection\n"
               "\n"
               "Runs on: %s\n"
               "DSP per frame: %5lu us\n"
               "Failures: %lu  restarts: %lu\n"
               "FPS: %d  \n"
               "\n"
               "A: tint %s\n"
               "B: ripple %s\n"
               "START: exit\n",
               dsp ? "DSP " : "ARM9",
               NEA_DspCyclesToUs(NEA_DspReflectGetStats()),
               NEA_DspDisplaceGetFailureCount()
               + NEA_DspGradeGetFailureCount(),
               NEA_DspGetResetCount(), NEA_GetFPS(),
               tint_on ? "on " : "off", ripple_on ? "on " : "off");
    }

    NEA_DspReflectEnd();
    NEA_DspEnd();
    NEA_End();

    return 0;
}
