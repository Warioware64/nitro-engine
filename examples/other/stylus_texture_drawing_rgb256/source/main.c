// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Antonio Niño Díaz, 2008-2024
//
// This file is part of Nitro Engine Advanced

#include <NEAMain.h>

#include "pal256.h"

typedef struct {
    NEA_Material *Material;
} SceneData;

void Draw3DScene(void *arg)
{
    SceneData *Scene = arg;

    NEA_2DViewInit();
    NEA_2DDrawTexturedQuad(0, 0,
                          256, 256,
                          0, Scene->Material);
}

int main(int argc, char *argv[])
{
    SceneData Scene = { 0 };

    irqEnable(IRQ_HBLANK);
    irqSet(IRQ_VBLANK, NEA_VBLFunc);
    irqSet(IRQ_HBLANK, NEA_HBLFunc);

    // Init Nitro Engine Advanced, normal 3D mode, and move the 3D screen to the
    // bottom screen
    NEA_Init3D();
    NEA_SwapScreens();

    // Allocate objects
    Scene.Material = NEA_MaterialCreate();
    NEA_Palette *Palette = NEA_PaletteCreate();

    // Load texture
    NEA_MaterialTexLoad(Scene.Material, NEA_PAL256, 256, 256, NEA_TEXGEN_TEXCOORD,
                       pal256Bitmap);
    NEA_PaletteLoad(Palette, pal256Pal, 32, NEA_PAL256);
    NEA_MaterialSetPalette(Scene.Material, Palette);

    // Modify color 254 of the palette so that we can use it to draw with a
    // known color
    NEA_PaletteModificationStart(Palette);
    NEA_PaletteRGB256SetColor(254, RGB15(0, 0, 31));
    NEA_PaletteModificationEnd();

    touchPosition touch;

    while (1)
    {
        NEA_WaitForVBL(0);

        scanKeys();
        touchRead(&touch);

        if (keysHeld() & KEY_TOUCH)
        {
            NEA_TextureDrawingStart(Scene.Material);

            // The function NEA_TexturePutPixelRGB256() makes sure to not draw
            // outside of the function, so we don't have to check here.
            NEA_TexturePutPixelRGB256(touch.px, touch.py, 254);
            NEA_TexturePutPixelRGB256(touch.px + 1, touch.py, 254);
            NEA_TexturePutPixelRGB256(touch.px, touch.py + 1, 254);
            NEA_TexturePutPixelRGB256(touch.px + 1, touch.py + 1, 254);

            NEA_TextureDrawingEnd();
        }

        NEA_ProcessArg(Draw3DScene, &Scene);
    }

    return 0;
}
