// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Antonio Niño Díaz, 2008-2024
//
// This file is part of Nitro Engine Advanced

// This example shows that Nitro Engine Advanced fixes the strange texture mapping of the
// DS of any size of texture.
//
// For example, a naive call to glTexCoord2t16(inttot16(32), inttot16(64)) will
// cause strange issues next to the vertex. Nitro Engine Advanced helpers compensate for
// this effect.

#include <NEAMain.h>

#include "s8.h"
#include "s16.h"
#include "s64.h"
#include "s256.h"

typedef struct {
    NEA_Material *Material_s8, *Material_s16, *Material_s64, *Material_s256;
    NEA_Palette *Palette_s8, *Palette_s16, *Palette_s64, *Palette_s256;
} SceneData;

void Draw3DScene(void *arg)
{
    SceneData *Scene = arg;

    NEA_ClearColorSet(NEA_DarkGray, 31, 63);

    NEA_2DViewInit();

    // Texture scaled from 64x64 to 128x128
    NEA_2DDrawTexturedQuad(10, 10,
                          10 + 128, 10 + 128,
                          2, Scene->Material_s64);

    // Texture scaled from 8x8 to 32x32
    NEA_2DDrawTexturedQuad(150, 10,
                          150 + 32, 10 + 32,
                          0, Scene->Material_s8);

    // Texture not scaled
    NEA_2DDrawTexturedQuad(160, 50,
                          160 + 64, 50 + 64,
                          10, Scene->Material_s64);


    // Texture scaled from 16x16 to 64x64
    NEA_2DDrawTexturedQuad(150, 120,
                          150 + 64, 120 + 64,
                          1, Scene->Material_s16);
}

void Draw3DScene2(void *arg)
{
    SceneData *Scene = arg;

    NEA_ClearColorSet(NEA_Magenta, 31, 63);

    NEA_2DViewInit();

    // Texture not scaled
    NEA_2DDrawTexturedQuad(-10, -70,
                          -10 + 256, -70 + 256,
                          3, Scene->Material_s256);
}

int main(int argc, char *argv[])
{
    SceneData Scene = { 0 };

    irqEnable(IRQ_HBLANK);
    irqSet(IRQ_VBLANK, NEA_VBLFunc);
    irqSet(IRQ_HBLANK, NEA_HBLFunc);

    NEA_InitDual3D();

    Scene.Material_s8 = NEA_MaterialCreate();
    Scene.Material_s16 = NEA_MaterialCreate();
    Scene.Material_s64 = NEA_MaterialCreate();
    Scene.Material_s256 = NEA_MaterialCreate();

    Scene.Palette_s8 = NEA_PaletteCreate();
    Scene.Palette_s16 = NEA_PaletteCreate();
    Scene.Palette_s64 = NEA_PaletteCreate();
    Scene.Palette_s256 = NEA_PaletteCreate();

    NEA_MaterialTexLoad(Scene.Material_s8, NEA_PAL16, 8, 8,
                       NEA_TEXGEN_TEXCOORD | NEA_TEXTURE_WRAP_S | NEA_TEXTURE_WRAP_T,
                       s8Bitmap);
    NEA_MaterialTexLoad(Scene.Material_s16, NEA_PAL16, 16, 16,
                       NEA_TEXGEN_TEXCOORD | NEA_TEXTURE_WRAP_S | NEA_TEXTURE_WRAP_T,
                       s16Bitmap);
    NEA_MaterialTexLoad(Scene.Material_s64, NEA_PAL16, 64, 64,
                       NEA_TEXGEN_TEXCOORD | NEA_TEXTURE_WRAP_S | NEA_TEXTURE_WRAP_T,
                       s64Bitmap);
    NEA_MaterialTexLoad(Scene.Material_s256, NEA_PAL16, 256, 256,
                       NEA_TEXGEN_TEXCOORD | NEA_TEXTURE_WRAP_S | NEA_TEXTURE_WRAP_T,
                       s256Bitmap);

    NEA_PaletteLoad(Scene.Palette_s8, s8Pal, 16, NEA_PAL16);
    NEA_PaletteLoad(Scene.Palette_s16, s16Pal, 16, NEA_PAL16);
    NEA_PaletteLoad(Scene.Palette_s64, s64Pal, 16, NEA_PAL16);
    NEA_PaletteLoad(Scene.Palette_s256, s256Pal, 16, NEA_PAL16);

    NEA_MaterialSetPalette(Scene.Material_s8, Scene.Palette_s8);
    NEA_MaterialSetPalette(Scene.Material_s16, Scene.Palette_s16);
    NEA_MaterialSetPalette(Scene.Material_s64, Scene.Palette_s64);
    NEA_MaterialSetPalette(Scene.Material_s256, Scene.Palette_s256);

    while (1)
    {
        NEA_WaitForVBL(0);

        NEA_ProcessDualArg(Draw3DScene, Draw3DScene2, &Scene, &Scene);
    }

    return 0;
}
