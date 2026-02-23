// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Antonio Niño Díaz, 2008-2024
//
// This file is part of Nitro Engine Advanced

// This demo doesn't work on DesMuME. It works on melonDS and hardware.

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

int main(int argc, char *argv[])
{
    SceneData Scene = { 0 };

    // This is needed for special screen effects
    irqEnable(IRQ_HBLANK);
    irqSet(IRQ_VBLANK, NEA_VBLFunc);
    irqSet(IRQ_HBLANK, NEA_HBLFunc);

    // Init dual 3D mode and console
    NEA_InitDual3D_DMA();
    NEA_InitConsole();

    // Allocate objects...
    Scene.Teapot = NEA_ModelCreate(NEA_Static);
    Scene.Sphere = NEA_ModelCreate(NEA_Static);
    Scene.Camera = NEA_CameraCreate();

    // Setup camera
    NEA_CameraSet(Scene.Camera,
                 0, 0, -2,
                 0, 0, 0,
                 0, 1, 0);

    // Load models
    NEA_ModelLoadStaticMesh(Scene.Teapot, teapot_bin);
    NEA_ModelLoadStaticMesh(Scene.Sphere, sphere_bin);

    // Set light color and direction
    NEA_LightSet(0, NEA_White, -0.5, -0.5, -0.5);

    // Other test configurations
    //NEA_SpecialEffectNoiseConfig(31);
    //NEA_SpecialEffectSineConfig(3, 8);

    while (1)
    {
        NEA_WaitForVBL(0);

        // Draw 3D scenes
        NEA_ProcessDualArg(Draw3DScene, Draw3DScene2, &Scene, &Scene);

        // Refresh keys
        scanKeys();
        uint32_t keys = keysHeld();
        uint32_t kdown = keysDown();

        printf("\x1b[0;0H"
               "START: Lock CPU until released\n"
               "Pad: Rotate.\nA: Sine effect.\nB: Noise effect.\n"
               "X: Deactivate effects.\nL/R: Pause/Unpause.");

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

        // Activate effects
        if (kdown & KEY_B)
            NEA_SpecialEffectSet(NEA_NOISE);
        if (kdown & KEY_A)
            NEA_SpecialEffectSet(NEA_SINE);
        // Deactivate effects
        if (kdown & KEY_X)
            NEA_SpecialEffectSet(0);

        // Pause effects
        if (kdown & KEY_L)
            NEA_SpecialEffectPause(true);
        if (kdown & KEY_R)
            NEA_SpecialEffectPause(false);
    }

    return 0;
}
