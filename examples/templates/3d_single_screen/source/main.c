// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Antonio Niño Díaz, 2008-2024
//
// This file is part of Nitro Engine Advanced

#include <NEAMain.h>

typedef struct {
    int placeholder;
} SceneData;

void Draw3DScene(void *arg)
{
    SceneData *Scene = arg;

    (void)Scene; // Silence unused variable warning
}

int main(int argc, char *argv[])
{
    SceneData Scene = { 0 };

    irqEnable(IRQ_HBLANK);
    irqSet(IRQ_VBLANK, NEA_VBLFunc);
    irqSet(IRQ_HBLANK, NEA_HBLFunc);

    NEA_Init3D();
    NEA_InitConsole();

    while (1)
    {
        NEA_WaitForVBL(0);
        NEA_ProcessArg(Draw3DScene, &Scene);

        scanKeys();

        // Your code goes here
    }

    return 0;
}
