// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Antonio Niño Díaz, 2008-2024
//
// This file is part of Nitro Engine Advanced

// This is an example of passing arguments to the functions passed to
// NEA_ProcessDualArg(). This passes a pointer to a struct with the information
// to render in that screen and anything else that the developer wants.

#include <NEAMain.h>

#include "teapot_bin.h"
#include "sphere_bin.h"

typedef struct {
    NEA_Camera *Camera;
    NEA_Model *Teapot;
    uint16_t clear_color;
} SceneData1;

typedef struct {
    NEA_Camera *Camera;
    NEA_Model *Sphere;
    uint16_t clear_color;
} SceneData2;

void Draw3DScene(void *arg)
{
    SceneData1 *Scene = arg;

    NEA_ClearColorSet(Scene->clear_color, 31, 63);

    NEA_CameraUse(Scene->Camera);
    NEA_ModelDraw(Scene->Teapot);
}

void Draw3DScene2(void *arg)
{
    SceneData2 *Scene = arg;

    NEA_ClearColorSet(Scene->clear_color, 31, 63);

    NEA_CameraUse(Scene->Camera);
    NEA_ModelDraw(Scene->Sphere);
}

void init_all(SceneData1 *Scene1, SceneData2 *Scene2)
{
    // Allocate objects...
    Scene1->Teapot = NEA_ModelCreate(NEA_Static);
    Scene2->Sphere = NEA_ModelCreate(NEA_Static);
    Scene1->Camera = NEA_CameraCreate();
    Scene2->Camera = NEA_CameraCreate();

    // Setup camera
    NEA_CameraSet(Scene1->Camera,
                 0, 0, -2,
                 0, 0, 0,
                 0, 1, 0);
    NEA_CameraSet(Scene2->Camera,
                 0, 0, -2,
                 0, 0, 0,
                 0, 1, 0);

    // Load models
    NEA_ModelLoadStaticMesh(Scene1->Teapot, teapot_bin);
    NEA_ModelLoadStaticMesh(Scene2->Sphere, sphere_bin);

    // Set light color and direction
    NEA_LightSet(0, NEA_White, -0.5, -0.5, -0.5);
}

int main(int argc, char *argv[])
{
    SceneData1 Scene1 = { 0 };
    SceneData2 Scene2 = { 0 };

    // This is needed for special screen effects
    irqEnable(IRQ_HBLANK);
    irqSet(IRQ_VBLANK, NEA_VBLFunc);
    irqSet(IRQ_HBLANK, NEA_HBLFunc);

    // Init dual 3D mode and console
    NEA_InitDual3D();
    NEA_InitConsole();

    init_all(&Scene1, &Scene2);

    Scene1.clear_color = NEA_Red;
    Scene2.clear_color = NEA_Green;

    bool console = true;

    while (1)
    {
        NEA_WaitForVBL(0);

        // Draw 3D scenes
        NEA_ProcessDualArg(Draw3DScene, Draw3DScene2, &Scene1, &Scene2);

        // Refresh keys
        scanKeys();
        uint32_t keys = keysHeld();
        uint32_t kdown = keysDown();

        if (console)
        {
            printf("\x1b[0;0H"
                   "A: Dual 3D DMA mode\n"
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
            NEA_ModelRotate(Scene2.Sphere, 0, 0, 2);
            NEA_ModelRotate(Scene1.Teapot, 0, 0, 2);
        }
        if (keys & KEY_DOWN)
        {
            NEA_ModelRotate(Scene2.Sphere, 0, 0, -2);
            NEA_ModelRotate(Scene1.Teapot, 0, 0, -2);
        }
        if (keys & KEY_RIGHT)
        {
            NEA_ModelRotate(Scene2.Sphere, 0, 2, 0);
            NEA_ModelRotate(Scene1.Teapot, 0, 2, 0);
        }
        if (keys & KEY_LEFT)
        {
            NEA_ModelRotate(Scene2.Sphere, 0, -2, 0);
            NEA_ModelRotate(Scene1.Teapot, 0, -2, 0);
        }

        if (kdown & KEY_Y)
        {
            NEA_InitDual3D();
            NEA_InitConsole();
            init_all(&Scene1, &Scene2);
            console = true;
        }
        if (kdown & KEY_X)
        {
            NEA_InitDual3D_FB();
            init_all(&Scene1, &Scene2);
            console = false;
        }
        if (kdown & KEY_A)
        {
            NEA_InitDual3D_DMA();
            NEA_InitConsole();
            init_all(&Scene1, &Scene2);
            console = true;
        }
    }

    return 0;
}
