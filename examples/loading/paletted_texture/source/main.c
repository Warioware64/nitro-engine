// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Antonio Niño Díaz, 2008-2024
//
// This file is part of Nitro Engine Advanced

#include <NEAMain.h>

#include "a5pal8.h"
#include "a3pal32.h"

typedef struct {
    NEA_Material *Material1;
} SceneData1;

typedef struct {
    NEA_Material *Material2;
} SceneData2;

void Draw3DScene(void *arg)
{
    SceneData1 *Scene = arg;

    NEA_2DViewInit();

    NEA_2DDrawTexturedQuad(0, 0,
                          256, 192,
                          0, Scene->Material1);
}

void Draw3DScene2(void *arg)
{
    SceneData2 *Scene = arg;

    NEA_2DViewInit();

    NEA_2DDrawTexturedQuad(64, 32,
                          64 + 128, 32 + 128,
                          0, Scene->Material2);
}

int main(int argc, char *argv[])
{
    SceneData1 Scene1 = { 0 };
    SceneData2 Scene2 = { 0 };

    irqEnable(IRQ_HBLANK);
    irqSet(IRQ_VBLANK, NEA_VBLFunc);
    irqSet(IRQ_HBLANK, NEA_HBLFunc);

    // Init 3D mode
    NEA_InitDual3D();

    // Allocate objects
    Scene1.Material1 = NEA_MaterialCreate();
    Scene2.Material2 = NEA_MaterialCreate();
    NEA_Palette *Palette1 = NEA_PaletteCreate();
    NEA_Palette *Palette2 = NEA_PaletteCreate();

    // Load part of the texture ignoring some of its height. You can't do
    // this with width because of how textures are laid out in VRAM.
    NEA_MaterialTexLoad(Scene1.Material1, NEA_A3PAL32, 256, 192,
                       NEA_TEXGEN_TEXCOORD, a3pal32Bitmap);

    // Load complete texture
    NEA_MaterialTexLoad(Scene2.Material2, NEA_A5PAL8, 256, 256,
                       NEA_TEXGEN_TEXCOORD, a5pal8Bitmap);

    NEA_PaletteLoad(Palette1, a3pal32Pal, 32, NEA_A3PAL32);
    NEA_PaletteLoad(Palette2, a5pal8Pal, 32, NEA_A5PAL8);

    NEA_MaterialSetPalette(Scene1.Material1, Palette1);
    NEA_MaterialSetPalette(Scene2.Material2, Palette2);

    while (1)
    {
        NEA_WaitForVBL(0);

        NEA_ProcessDualArg(Draw3DScene, Draw3DScene2, &Scene1, &Scene2);
    }

    return 0;
}
