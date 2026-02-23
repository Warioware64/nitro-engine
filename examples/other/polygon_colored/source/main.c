// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Antonio Niño Díaz, 2008-2024
//
// This file is part of Nitro Engine Advanced

#include <NEAMain.h>

typedef struct {
    NEA_Camera *Camera;
} SceneData;

void Draw3DScene(void *arg)
{
    SceneData *Scene = arg;

    // Use camera and draw polygon.
    NEA_CameraUse(Scene->Camera);

    // Begin drawing
    NEA_PolyBegin(GL_QUAD);

        NEA_PolyColor(NEA_Red);    // Set next vertices color
        NEA_PolyVertex(-1, 1, 0); // Send vertex

        NEA_PolyColor(NEA_Green);
        NEA_PolyVertex(-1, -1, 0);

        NEA_PolyColor(NEA_Yellow);
        NEA_PolyVertex(1, -1, 0);

        NEA_PolyColor(NEA_Blue);
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

    NEA_CameraSet(Scene.Camera,
                 0, 0, 2,
                 0, 0, 0,
                 0, 1, 0);

    while (1)
    {
        NEA_WaitForVBL(0);

        NEA_ProcessArg(Draw3DScene, &Scene);
    }

    return 0;
}
