// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Antonio Niño Díaz, 2008-2024
//
// This file is part of Nitro Engine Advanced

// The special screen effects of this is demo don't work on DesMuME. It works on
// melonDS and hardware.
//
// Dual 3D:
// - When the CPU hangs, the last drawn screen is shown in both screens.
// - The debug console works.
// - Special effects work normally.
//
// Dual 3D FB:
// - When the CPU hangs, the output is stable.
// - The debug console doesn't work.
// - Special effects don't work properly.
//
// Dual 3D DMA:
// - When the CPU hangs, the output is stable.
// - The debug console works.
// - Special effects work normally.
// - The CPU performance is lower. It requires more DMA and CPU copies.

#include <NEAMain.h>

#include "teapot_bin.h"
#include "sphere_bin.h"

typedef struct {
    NEA_Camera *Camera;
    NEA_Model *Teapot, *Sphere;
} SceneData;

void Draw3DScene(void *arg)
{
    SceneData *Scene = arg;

    NEA_ClearColorSet(NEA_Red, 31, 63);

    NEA_CameraUse(Scene->Camera);
    NEA_ModelDraw(Scene->Teapot);
}

void Draw3DScene2(void *arg)
{
    SceneData *Scene = arg;

    NEA_ClearColorSet(NEA_Green, 31, 63);

    NEA_CameraUse(Scene->Camera);
    NEA_ModelDraw(Scene->Sphere);
}

void init_all(SceneData *Scene)
{
    // Allocate objects...
    Scene->Teapot = NEA_ModelCreate(NEA_Static);
    Scene->Sphere = NEA_ModelCreate(NEA_Static);
    Scene->Camera = NEA_CameraCreate();

    // Setup camera
    NEA_CameraSet(Scene->Camera,
                 0, 0, -2,
                 0, 0, 0,
                 0, 1, 0);

    // Load models
    NEA_ModelLoadStaticMesh(Scene->Teapot, teapot_bin);
    NEA_ModelLoadStaticMesh(Scene->Sphere, sphere_bin);

    // Set light color and direction
    NEA_LightSet(0, NEA_White, -0.5, -0.5, -0.5);
}

int main(int argc, char *argv[])
{
    SceneData Scene = { 0 };

    // This is needed for special screen effects
    irqEnable(IRQ_HBLANK);
    irqSet(IRQ_VBLANK, NEA_VBLFunc);
    irqSet(IRQ_HBLANK, NEA_HBLFunc);

    // Init dual 3D mode and console
    NEA_InitDual3D();
    NEA_InitConsole();

    init_all(&Scene);

    bool console = true;

    while (1)
    {
        NEA_WaitForVBL(0);

        // Draw 3D scenes
        NEA_ProcessDualArg(Draw3DScene, Draw3DScene2, &Scene, &Scene);

        // Refresh keys
        scanKeys();
        uint32_t keys = keysHeld();
        uint32_t kdown = keysDown();

        if (console)
        {
            printf("\x1b[0;0H"
                   "START: Lock CPU until released\n"
                   "A: Sine effect.\n"
                   "B: Deactivate effect.\n"
                   "SELECT: Dual 3D DMA mode\n"
                   "X: Dual 3D FB mode (no console)\n"
                   "Y: Dual 3D mode\n"
                   "Pad: Rotate.\n");
        }

        // Lock CPU in an infinite loop to simulate a drop in framerate
        while (keys & KEY_START)
        {
            scanKeys();
            keys = keysHeld();
        }

        // Rotate model
        if (keys & KEY_UP)
        {
            NEA_ModelRotate(Scene.Sphere, 0, 0, 2);
            NEA_ModelRotate(Scene.Teapot, 0, 0, 2);
        }
        if (keys & KEY_DOWN)
        {
            NEA_ModelRotate(Scene.Sphere, 0, 0, -2);
            NEA_ModelRotate(Scene.Teapot, 0, 0, -2);
        }
        if (keys & KEY_RIGHT)
        {
            NEA_ModelRotate(Scene.Sphere, 0, 2, 0);
            NEA_ModelRotate(Scene.Teapot, 0, 2, 0);
        }
        if (keys & KEY_LEFT)
        {
            NEA_ModelRotate(Scene.Sphere, 0, -2, 0);
            NEA_ModelRotate(Scene.Teapot, 0, -2, 0);
        }

        // Deactivate effect
        if (kdown & KEY_B)
            NEA_SpecialEffectSet(0);
        // Activate effect
        if (kdown & KEY_A)
            NEA_SpecialEffectSet(NEA_SINE);

        if (kdown & KEY_Y)
        {
            NEA_SpecialEffectSet(0);
            NEA_InitDual3D();
            NEA_InitConsole();
            init_all(&Scene);
            console = true;
        }
        if (kdown & KEY_X)
        {
            NEA_SpecialEffectSet(0);
            NEA_InitDual3D_FB();
            init_all(&Scene);
            console = false;
        }
        if (kdown & KEY_SELECT)
        {
            NEA_SpecialEffectSet(0);
            NEA_InitDual3D_DMA();
            NEA_InitConsole();
            init_all(&Scene);
            console = true;
        }
    }

    return 0;
}
