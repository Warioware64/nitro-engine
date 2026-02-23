// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Antonio Niño Díaz, 2008-2024
//
// This file is part of Nitro Engine Advanced

#include <NEAMain.h>

#include "a5pal8.h"
#include "a5pal8.h"

typedef struct {
    NEA_Material *Material;
} SceneData;

void Draw3DScene(void *arg)
{
    SceneData *Scene = arg;

    NEA_2DViewInit();

    NEA_2DDrawTexturedQuad(64, 32,
                          64 + 128, 32 + 128,
                          0, Scene->Material);
}

int main(int argc, char *argv[])
{
    SceneData Scene = { 0 };

    irqEnable(IRQ_HBLANK);
    irqSet(IRQ_VBLANK, NEA_VBLFunc);
    irqSet(IRQ_HBLANK, NEA_HBLFunc);

    // Init Nitro Engine Advanced in normal 3D mode
    NEA_Init3D();

    // libnds uses VRAM_C for the text console, reserve A and B only
    NEA_TextureSystemReset(0, 0, NEA_VRAM_AB);
    // Init console in non-3D screen
    consoleDemoInit();

    // Allocate objects
    Scene.Material = NEA_MaterialCreate();
    NEA_Palette *Palette = NEA_PaletteCreate();

    NEA_MaterialTexLoad(Scene.Material, NEA_A5PAL8, 256, 256, NEA_TEXGEN_TEXCOORD,
                       a5pal8Bitmap);

    NEA_PaletteLoad(Palette, a5pal8Pal, 32, NEA_A5PAL8);

    NEA_MaterialSetPalette(Scene.Material, Palette);

    printf("UP:   Set top screen as main\n"
           "DOWN: Set bottom screen as main\n"
           "A:    Swap screens");

    while (1)
    {
        NEA_WaitForVBL(0);

        scanKeys();
        uint16_t keys = keysDown();

        if (keys & KEY_A)
            NEA_SwapScreens();
        if (keys & KEY_UP)
            NEA_MainScreenSetOnTop();
        if (keys & KEY_DOWN)
            NEA_MainScreenSetOnBottom();

        NEA_ProcessArg(Draw3DScene, &Scene);
    }

    return 0;
}
