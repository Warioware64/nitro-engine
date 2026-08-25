// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Nitro Engine Advanced contributors, 2026
//
// This file is part of Nitro Engine Advanced

// Per-scanline distortion driven from the HBlank interrupt (NEAPostFX).
//
// Two tables are demonstrated:
//
//   NEA_SCANLINE_BG0HOFS - a sine wave written to REG_BG0HOFS every scanline,
//                          giving a heat-haze / underwater warp of the 3D
//                          image. This is the *only* per-scanline distortion
//                          the 3D layer supports without capturing it first:
//                          GBATEK says the 3D layer cannot be rotated, scaled
//                          or scrolled vertically.
//
//   NEA_SCANLINE_WIN0H   - a circular iris written to REG_WIN0H every scanline,
//                          which turns the rectangular vignette window into a
//                          round flashlight cone.
//
// This example also *measures* the HBlank handler, because the margin is the
// one number the documentation does not give. TIMER0 free-runs at the bus
// clock; the handler is wrapped to sample it on entry and exit, and the worst
// case seen is printed in both ticks and as a percentage of the HBlank window.
//
// Cost: no VRAM and no BG layer. Only HBlank CPU time -- see the readout.

#include <NEAMain.h>

#include "texture.h"
#include "sphere_bin.h"

typedef struct {
    NEA_Camera *Camera;
    NEA_Model *Model, *Model2;
} SceneData;

// ---- HBlank handler instrumentation ---------------------------------------
//
// TIMER0 counts at 33.513 MHz (no prescaler), so one tick is ~29.8 ns. A
// scanline is 2130 ticks and the HBlank portion of it is 99 dots = 594 ticks.

#define HBLANK_TICKS 594

static vu16 hbl_worst = 0;

static void MeasuredHBL(void)
{
    u16 t0 = TIMER0_DATA;
    NEA_HBLFunc();
    u16 dt = TIMER0_DATA - t0;

    if (dt > hbl_worst)
        hbl_worst = dt;
}

void Draw3DScene(void *arg)
{
    SceneData *Scene = arg;

    NEA_CameraUse(Scene->Camera);
    NEA_PolyFormat(31, 0, NEA_LIGHT_ALL, NEA_CULL_BACK, 0);
    NEA_ModelDraw(Scene->Model);
    NEA_ModelDraw(Scene->Model2);
}

int main(int argc, char *argv[])
{
    SceneData Scene = { 0 };

    // Free-running counter used to time the HBlank handler.
    TIMER0_DATA = 0;
    TIMER0_CR = TIMER_DIV_1 | TIMER_ENABLE;

    irqEnable(IRQ_HBLANK);
    irqSet(IRQ_VBLANK, NEA_VBLFunc);
    irqSet(IRQ_HBLANK, MeasuredHBL);

    NEA_Init3D();
    NEA_TextureSystemReset(0, 0, NEA_VRAM_AB);
    consoleDemoInit();

    Scene.Model = NEA_ModelCreate(NEA_Static);
    Scene.Model2 = NEA_ModelCreate(NEA_Static);
    Scene.Camera = NEA_CameraCreate();
    NEA_Material *Material = NEA_MaterialCreate();

    NEA_CameraSet(Scene.Camera, -1, 2, -1, 1, 1, 1, 0, 1, 0);

    NEA_ModelLoadStaticMesh(Scene.Model, sphere_bin);
    NEA_ModelLoadStaticMesh(Scene.Model2, sphere_bin);
    NEA_MaterialTexLoad(Material, NEA_A1RGB5, 256, 256, NEA_TEXGEN_TEXCOORD,
                       textureBitmap);
    NEA_ModelSetMaterial(Scene.Model, Material);
    NEA_ModelSetMaterial(Scene.Model2, Material);

    NEA_LightSet(0, NEA_White, 0, -1, -1);
    NEA_ModelSetCoord(Scene.Model, 1, 0, 1);
    NEA_ModelSetCoord(Scene.Model2, 3, 1, 3);

    bool haze = true;
    bool iris = false;
    int amplitude = 4;
    int freq = 48;
    int radius = 70;

    NEA_PostFXScanlineGenerateSine(NEA_SCANLINE_BG0HOFS, amplitude, freq, 0);
    NEA_PostFXScanlineEnable(NEA_SCANLINE_BG0HOFS, true);

    while (1)
    {
        NEA_WaitForVBL(0);

        scanKeys();
        uint32_t keys = keysDown();
        uint32_t held = keysHeld();

        if (keys & KEY_A)
        {
            haze = !haze;
            NEA_PostFXScanlineEnable(NEA_SCANLINE_BG0HOFS, haze);
            hbl_worst = 0;
        }
        if (keys & KEY_B)
        {
            iris = !iris;
            if (iris)
            {
                NEA_PostFXScanlineGenerateCircle(96, radius);
                NEA_PostFXScanlineSetWindowCenter(128);
                NEA_PostFXVignetteEnable(true, 16);
            }
            else
            {
                NEA_PostFXVignetteEnable(false, 0);
            }
            NEA_PostFXScanlineEnable(NEA_SCANLINE_WIN0H, iris);
            hbl_worst = 0;
        }
        if (keys & KEY_Y)
            hbl_worst = 0;

        if (held & KEY_UP) amplitude++;
        if (held & KEY_DOWN) amplitude--;
        if (amplitude < 0) amplitude = 0;
        if (amplitude > 32) amplitude = 32;

        if (held & KEY_RIGHT) radius += 2;
        if (held & KEY_LEFT) radius -= 2;
        if (radius < 8) radius = 8;
        if (radius > 120) radius = 120;

        if (haze)
        {
            // Refilling all 192 entries every frame is what animates the warp.
            // It is ~4000 cycles, well under 1% of the frame.
            NEA_PostFXScanlineGenerateSine(NEA_SCANLINE_BG0HOFS, amplitude,
                                          freq, 0);
            NEA_PostFXScanlineAdvancePhase(NEA_SCANLINE_BG0HOFS, 6);
        }
        if (iris)
            NEA_PostFXScanlineGenerateCircle(96, radius);

        u32 worst = hbl_worst;
        u32 pct = (worst * 100) / HBLANK_TICKS;

        printf("\x1b[0;0H"
               "PostFX per-scanline\n"
               "===================\n\n"
               "A  Heat haze:  %s\n"
               "   Up/Down amp:  %2d\n"
               "B  Iris/cone:  %s\n"
               "   L/R radius:  %3d\n"
               "Y  Reset measurement\n\n"
               "HBlank handler, worst case\n"
               "  %4lu ticks of %d\n"
               "  %3lu%% of the HBlank window\n\n"
               "1 tick = ~29.8 ns\n"
               "HBlank = 99 dots = 594 ticks\n"
               "CPU: %d%%\n",
               haze ? "on " : "off", amplitude,
               iris ? "on " : "off", radius,
               worst, HBLANK_TICKS, pct,
               NEA_GetCPUPercent());

        NEA_ProcessArg(Draw3DScene, &Scene);
    }

    return 0;
}
