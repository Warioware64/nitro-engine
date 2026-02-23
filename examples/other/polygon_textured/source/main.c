// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Antonio Niño Díaz, 2008-2024
//
// This file is part of Nitro Engine Advanced

#include <NEAMain.h>

#include "texture.h"

typedef struct {
    NEA_Camera *Camera;
    NEA_Material *Material;
} SceneData;

void Draw3DScene(void *arg)
{
    SceneData *Scene = arg;

    NEA_CameraUse(Scene->Camera);

    // This set material's color to drawing color (default = white)
    NEA_MaterialUse(Scene->Material);

    // In general you should avoid using the functions below for drawing models
    // because they have a much lower performance than precompiled models.

    // Begin drawing
    NEA_PolyBegin(GL_QUAD);

        NEA_PolyColor(NEA_Red);    // Set next vertices color
        NEA_PolyTexCoord(0, 0);   // Texture coordinates
        NEA_PolyVertex(-1, 1, 0); // Send new vertex

        NEA_PolyColor(NEA_Blue);
        NEA_PolyTexCoord(0, 64);
        NEA_PolyVertex(-1, -1, 0);

        NEA_PolyColor(NEA_Green);
        NEA_PolyTexCoord(64, 64);
        NEA_PolyVertex(1, -1, 0);

        NEA_PolyColor(NEA_Yellow);
        NEA_PolyTexCoord(64, 0);
        NEA_PolyVertex(1, 1, 0);

    NEA_PolyEnd();
}

int main(int argc, char *argv[])
{
    SceneData Scene = { 0 };

    irqEnable(IRQ_HBLANK);
    irqSet(IRQ_VBLANK, NEA_VBLFunc);
    irqSet(IRQ_HBLANK, NEA_HBLFunc);

    NEA_Init3D();

    Scene.Camera = NEA_CameraCreate();
    Scene.Material = NEA_MaterialCreate();

    NEA_CameraSet(Scene.Camera,
                 0, 0, 2,
                 0, 0, 0,
                 0, 1, 0);

    NEA_MaterialTexLoad(Scene.Material, NEA_A1RGB5, 128, 128, NEA_TEXGEN_TEXCOORD,
                       textureBitmap);

    while (1)
    {
        NEA_WaitForVBL(0);

        NEA_ProcessArg(Draw3DScene, &Scene);
    }

    return 0;
}
