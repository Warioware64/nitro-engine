// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Antonio Niño Díaz, 2008-2024
//
// This file is part of Nitro Engine Advanced

#include <NEAMain.h>

#include "a1rgb5.h"

typedef struct {
    NEA_Material *Material;
} SceneData;

void Draw3DScene(void *arg)
{
    SceneData *Scene = arg;

    NEA_2DViewInit();
    NEA_2DDrawTexturedQuad(0, 0, 256, 256, 0, Scene->Material);
}

int main(int argc, char *argv[])
{
    SceneData Scene = { 0 };

    irqEnable(IRQ_HBLANK);
    irqSet(IRQ_VBLANK, NEA_VBLFunc);
    irqSet(IRQ_HBLANK, NEA_HBLFunc);

    NEA_Init3D();
    NEA_MainScreenSetOnBottom();

    Scene.Material = NEA_MaterialCreate();
    NEA_MaterialTexLoad(Scene.Material, NEA_A1RGB5, 256, 256, NEA_TEXGEN_TEXCOORD,
                       a1rgb5Bitmap);

    while (1)
    {
        NEA_WaitForVBL(0);

        scanKeys();
        touchPosition touch;

        if (keysHeld() & KEY_TOUCH)
        {
            // Update stylus coordinates when screen is pressed
            touchRead(&touch);
            NEA_TextureDrawingStart(Scene.Material);

            // Draw blue pixels with no transparency. The function
            // NEA_TexturePutPixelRGBA() makes sure to not draw outside of the
            // function, so we don't have to check here.
            uint16_t color = RGB15(0, 0, 31) | BIT(15);
            NEA_TexturePutPixelRGBA(touch.px, touch.py, color);
            NEA_TexturePutPixelRGBA(touch.px + 1, touch.py, color);
            NEA_TexturePutPixelRGBA(touch.px, touch.py + 1, color);
            NEA_TexturePutPixelRGBA(touch.px + 1, touch.py + 1, color);

            NEA_TextureDrawingEnd();
        }

        NEA_ProcessArg(Draw3DScene, &Scene);
    }

    return 0;
}
