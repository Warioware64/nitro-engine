// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Antonio Niño Díaz, 2008-2024
//
// This file is part of Nitro Engine Advanced

// This is an example to show how Nitro Engine Advanced can load textures of any height.
// Internally, the NDS thinks that the texture is bigger, but Nitro Engine Advanced only
// uses the parts that the user has loaded.
//
// The width needs to be a power of two because:
//
// - Supporting them complicates the loading code a lot.
//
// - Compressed textures can't really be expanded because they are composed
//   by many 4x4 subimages.
//
// - They don't save space in VRAM.
//
// - They save space in the final ROM, but you can achieve the same effect
//   compressing them with LZSS compression, for example.

#include <NEAMain.h>

#include "a3pal32.h"
#include "pal4.h"

typedef struct {
    NEA_Material *Material, *Material2;
} SceneData;

void Draw3DScene(void *arg)
{
    SceneData *Scene = arg;

    NEA_2DViewInit();

    NEA_2DDrawTexturedQuad(40, 10,
                          40 + 32, 10 + 100,
                          0, Scene->Material);

    NEA_2DDrawTexturedQuad(128, 10,
                          128 + 64, 10 + 100,
                          0, Scene->Material2);
}

int main(int argc, char *argv[])
{
    SceneData Scene = { 0 };

    irqEnable(IRQ_HBLANK);
    irqSet(IRQ_VBLANK, NEA_VBLFunc);
    irqSet(IRQ_HBLANK, NEA_HBLFunc);

    // Init 3D mode
    NEA_Init3D();

    // Allocate objects for a material
    Scene.Material = NEA_MaterialCreate();
    NEA_Palette *Palette = NEA_PaletteCreate();

    NEA_MaterialTexLoad(Scene.Material,
                       NEA_A3PAL32, // Texture type
                       64, 200,    // Width, height (in pixels)
                       NEA_TEXGEN_TEXCOORD, a3pal32Bitmap);
    NEA_PaletteLoad(Palette, a3pal32Pal, 32, NEA_A3PAL32);
    NEA_MaterialSetPalette(Scene.Material, Palette);

    // Allocate objects for another material
    Scene.Material2 = NEA_MaterialCreate();
    NEA_Palette *Palette2 = NEA_PaletteCreate();

    NEA_MaterialTexLoad(Scene.Material2, NEA_PAL4, 64, 100, NEA_TEXGEN_TEXCOORD,
                       pal4Bitmap);
    NEA_PaletteLoad(Palette2, pal4Pal, 4, NEA_PAL4);
    NEA_MaterialSetPalette(Scene.Material2, Palette2);

    while (1)
    {
        NEA_WaitForVBL(0);

        // Draw 3D scene
        NEA_ProcessArg(Draw3DScene, &Scene);
    }

    return 0;
}
