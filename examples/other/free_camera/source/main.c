// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Antonio Niño Díaz, 2008-2024
//
// This file is part of Nitro Engine Advanced

#include <NEAMain.h>

#include "robot_bin.h"
#include "texture.h"

typedef struct {
    NEA_Camera *Camera;
    NEA_Model *Model;
} SceneData;

void Draw3DScene(void *arg)
{
    SceneData *Scene = arg;

    NEA_CameraUse(Scene->Camera);
    NEA_PolyFormat(31, 0, NEA_LIGHT_0, NEA_CULL_NONE, 0);
    NEA_ModelDraw(Scene->Model);
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

    // Allocate objects
    Scene.Model = NEA_ModelCreate(NEA_Static);
    Scene.Camera = NEA_CameraCreate();
    NEA_Material *Material = NEA_MaterialCreate();

    // Set coordinates for the camera
    NEA_CameraSet(Scene.Camera,
                 -8, 3, 0,  // Position
                  0, 3, 0,  // Look at
                  0, 1, 0); // Up direction

    // Load mesh from RAM and assign it to the object "Model".
    NEA_ModelLoadStaticMesh(Scene.Model, robot_bin);
    // Load a RGB texture from RAM and assign it to "Material".
    NEA_MaterialTexLoad(Material, NEA_A1RGB5, 256, 256, NEA_TEXGEN_TEXCOORD,
                       textureBitmap);

    // Assign texture to model...
    NEA_ModelSetMaterial(Scene.Model, Material);

    // We set up a light and its color
    NEA_LightSet(0, NEA_White, -0.5, -0.5, -0.5);

    int angle = 0;

    while (1)
    {
        NEA_WaitForVBL(0);

        // Get keys information
        scanKeys();
        uint32_t keys = keysHeld();

        printf("\x1b[0;0HPad: Rotate.\nA/B: Move forward/back.");

        if (keys & KEY_UP && angle < 92)
        {
            angle += 3;
            NEA_CameraRotateFree(Scene.Camera, 3, 0, 0);
        }
        else if (keys & KEY_DOWN && angle > -92)
        {
            angle -= 3;
            NEA_CameraRotateFree(Scene.Camera, -3, 0, 0);
        }

        if (keys & KEY_LEFT)
            NEA_CameraRotateFree(Scene.Camera, 0, -3, 0);
        else if (keys & KEY_RIGHT)
            NEA_CameraRotateFree(Scene.Camera, 0, 3, 0);

        if (keys & KEY_A)
            NEA_CameraMoveFree(Scene.Camera, 0.05, 0, 0);
        else if (keys & KEY_B)
            NEA_CameraMoveFree(Scene.Camera, -0.05, 0, 0);

        NEA_ProcessArg(Draw3DScene, &Scene);
    }

    return 0;
}
