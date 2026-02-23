// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Antonio Niño Díaz, 2008-2024
//
// This file is part of Nitro Engine Advanced

#include <NEAMain.h>

#include "sphere_bin.h"

#define NUM_MODELS 16

typedef struct {
    NEA_Camera *Camera;
    NEA_Model *Model[NUM_MODELS];
} SceneData;

void Draw3DScene(void *arg)
{
    SceneData *Scene = arg;

    NEA_CameraUse(Scene->Camera);

    for (int i = 0; i < NUM_MODELS; i++)
        NEA_ModelDraw(Scene->Model[i]);

    // Get some information after drawing but before returning from the
    // function
    printf("\x1b[0;0HPolygon RAM: %d   \nVertex RAM: %d   ",
           NEA_GetPolygonCount(), NEA_GetVertexCount());
}

int main(int argc, char *argv[])
{
    SceneData Scene = { 0 };

    irqEnable(IRQ_HBLANK);
    irqSet(IRQ_VBLANK, NEA_VBLFunc);
    irqSet(IRQ_HBLANK, NEA_HBLFunc);

    NEA_Init3D();
    // libnds uses VRAM_C for the text console, reserve A and B only
    NEA_TextureSystemReset(0, 0, NEA_VRAM_AB);
    // Init console in non-3D screen
    consoleDemoInit();

    // Allocate space for everything
    for (int i = 0; i < NUM_MODELS; i++)
        Scene.Model[i] = NEA_ModelCreate(NEA_Static);

    Scene.Camera = NEA_CameraCreate();

    // Setup camera
    NEA_CameraSet(Scene.Camera,
                 -3.5, 1.5, 1.25,
                    0, 1.5, 1.25,
                    0, 1, 0);

    // Load model
    for (int i = 0; i < NUM_MODELS; i++)
        NEA_ModelLoadStaticMesh(Scene.Model[i], sphere_bin);

    // Setup light
    NEA_LightSet(0, NEA_Yellow, 0, -0.5, -0.5);

    // Set start coordinates/rotation of models
    for (int i = 0; i < NUM_MODELS; i++)
    {
        NEA_ModelSetRot(Scene.Model[i], i, i * 30, i * 20);
        NEA_ModelSetCoord(Scene.Model[i], 0, i % 4, i / 4);
    }

    while (1)
    {
        NEA_WaitForVBL(0);

        // Rotate every model
        for (int i = 0; i < NUM_MODELS; i++)
            NEA_ModelRotate(Scene.Model[i], -i, i % 5, 5 - i);

        // Draw scene
        NEA_ProcessArg(Draw3DScene, &Scene);
    }

    return 0;
}
