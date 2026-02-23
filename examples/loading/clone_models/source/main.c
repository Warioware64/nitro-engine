// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Antonio Niño Díaz, 2008-2024
//
// This file is part of Nitro Engine Advanced

// Cloning models will avoid loading into memory the same mesh many times. This
// is really useful when you want to draw lots of the same model in different
// locations, with different animations, etc. You won't need to store multiple
// copies of the mesh in RAM.
//
// If you clone an animated model you will be able to set different animations
// for each model.

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

    // Setup camera and draw all objects.
    NEA_CameraUse(Scene->Camera);

    for (int i = 0; i < NUM_MODELS; i++)
        NEA_ModelDraw(Scene->Model[i]);

    // Get some information AFTER drawing but BEFORE returning from the
    // function.
    printf("\x1b[0;0HPolygon RAM: %d   \nVertex RAM: %d   ",
           NEA_GetPolygonCount(), NEA_GetVertexCount());
}

int main(int argc, char *argv[])
{
    SceneData Scene = { 0 };

    irqEnable(IRQ_HBLANK);
    irqSet(IRQ_VBLANK, NEA_VBLFunc);
    irqSet(IRQ_HBLANK, NEA_HBLFunc);

    // Init Nitro Engine Advanced.
    NEA_Init3D();
    // libnds uses VRAM_C for the text console, reserve A and B only
    NEA_TextureSystemReset(0, 0, NEA_VRAM_AB);
    // Init console in non-3D screen
    consoleDemoInit();

    // Allocate space for everything.
    for (int i = 0; i < NUM_MODELS; i++)
        Scene.Model[i] = NEA_ModelCreate(NEA_Static);

    Scene.Camera = NEA_CameraCreate();

    // Setup camera
    NEA_CameraSet(Scene.Camera,
                 -3.5, 1.5, 1.25,
                    0, 1.5, 1.25,
                    0, 1, 0);

    // Load model once
    NEA_ModelLoadStaticMesh(Scene.Model[0], sphere_bin);

    // Clone model to the test of the objects
    for (int i = 1; i < NUM_MODELS; i++)
    {
        NEA_ModelClone(Scene.Model[i],  // Destination
                      Scene.Model[0]); // Source model
    }

    // Set up light
    NEA_LightSet(0, NEA_Yellow, 0, -0.5, -0.5);

    // Set start coordinates/rotation of the models
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
