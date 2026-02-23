// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Antonio Niño Díaz, 2008-2024
//
// This file is part of Nitro Engine Advanced

//  TODO

#include <NEAMain.h>

#include "sphere_bin.h"

typedef struct {
    NEA_Camera *Camera;
    NEA_Model *ModelWithoutMatrix;
    NEA_Model *ModelWithMatrix;
} SceneData;

void Draw3DScene(void *arg)
{
    SceneData *Scene = arg;

    NEA_CameraUse(Scene->Camera);

    NEA_ModelDraw(Scene->ModelWithoutMatrix);
    NEA_ModelDraw(Scene->ModelWithMatrix);
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
    Scene.ModelWithoutMatrix = NEA_ModelCreate(NEA_Static);
    Scene.ModelWithMatrix = NEA_ModelCreate(NEA_Static);

    Scene.Camera = NEA_CameraCreate();

    // Setup camera
    NEA_CameraSet(Scene.Camera,
                 0, 0, 3,
                 0, 0, 0,
                 0, 1, 0);

    // Load models
    NEA_ModelLoadStaticMesh(Scene.ModelWithoutMatrix, sphere_bin);
    NEA_ModelLoadStaticMesh(Scene.ModelWithMatrix, sphere_bin);

    // Set up light
    NEA_LightSet(0, NEA_Yellow, 0, -0.5, -0.5);

    // Set start coordinates/rotation of the models
    NEA_ModelSetCoord(Scene.ModelWithoutMatrix, -1, 0, 0);

    // Transformation matrix we are going to use for a model. Note that this
    // matrix is transposed compared to what most 3D documentation describes.
    m4x3 matrix = {{
        // 3x3 transformation
        inttof32(1), 0,                     0,
        0,           inttof32(1),           0,
        0,                     0, inttof32(1),
        // Translation vector
        inttof32(2),           0,           0
    }};

    int translation = inttof32(2);

    printf("The right ball uses a matrix\n"
           "assigned by the user, the left\n"
           "one has rotation managed by\n"
           "Nitro Engine Advanced.");

    while (1)
    {
        NEA_WaitForVBL(0);

        // Rotate the first model
        NEA_ModelRotate(Scene.ModelWithoutMatrix, -1, 2, 1);

        // Update matrix manually
        translation += floattof32(0.05);
        if (translation > inttof32(3))
            translation = inttof32(1);
        matrix.m[9] = translation;

        // Assign matrix again
        NEA_ModelSetMatrix(Scene.ModelWithMatrix, &matrix);

        // Draw scene
        NEA_ProcessArg(Draw3DScene, &Scene);
    }

    return 0;
}
