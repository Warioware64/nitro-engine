// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Antonio Niño Díaz, 2008-2024
//
// This file is part of Nitro Engine Advanced

#include <NEAMain.h>

#include "assets.h"
#include "icon.h"

NEA_Sprite *Sprite[6];

void Draw3DScene1(void)
{
    NEA_2DViewInit();

    NEA_ClearColorSet(NEA_White, 31, 63);

    NEA_SpriteDraw(Sprite[3]);
    NEA_SpriteDraw(Sprite[4]);
    NEA_SpriteDraw(Sprite[5]);
}

void Draw3DScene2(void)
{
    NEA_2DViewInit();

    NEA_ClearColorSet(NEA_Gray, 31, 63);

    NEA_SpriteDraw(Sprite[0]);
    NEA_SpriteDraw(Sprite[1]);
    NEA_SpriteDraw(Sprite[2]);
}

int main(int argc, char *argv[])
{
    irqEnable(IRQ_HBLANK);
    irqSet(IRQ_VBLANK, NEA_VBLFunc);
    irqSet(IRQ_HBLANK, NEA_HBLFunc);

    NEA_InitDual3D();
    NEA_InitConsole();

    NEA_Material *MaterialIcon = NEA_MaterialCreate();
    NEA_Palette *PaletteIcon = NEA_PaletteCreate();

    NEA_MaterialTexLoad(MaterialIcon, NEA_PAL16, 128, 128,
                       NEA_TEXGEN_TEXCOORD | NEA_TEXTURE_COLOR0_TRANSPARENT,
                       iconBitmap);
    NEA_PaletteLoad(PaletteIcon, iconPal, iconPalLen / 2, NEA_PAL16);
    NEA_MaterialSetPalette(MaterialIcon, PaletteIcon);

    NEA_Material *MaterialAssets = NEA_MaterialCreate();
    NEA_Palette *PaletteAssets = NEA_PaletteCreate();

    NEA_MaterialTexLoad(MaterialAssets, NEA_PAL256, 512, 256,
                       NEA_TEXGEN_TEXCOORD | NEA_TEXTURE_COLOR0_TRANSPARENT,
                       assetsBitmap);
    NEA_PaletteLoad(PaletteAssets, assetsPal, assetsPalLen / 2, NEA_PAL16);
    NEA_MaterialSetPalette(MaterialAssets, PaletteAssets);

    Sprite[0] = NEA_SpriteCreate();
    Sprite[1] = NEA_SpriteCreate();
    Sprite[2] = NEA_SpriteCreate();
    Sprite[3] = NEA_SpriteCreate();
    Sprite[4] = NEA_SpriteCreate();
    Sprite[5] = NEA_SpriteCreate();

    // Sprite with the same size as the texture
    NEA_SpriteSetMaterial(Sprite[0], MaterialIcon);
    NEA_SpriteSetPos(Sprite[0], 10, 40);
    NEA_SpriteSetPriority(Sprite[0], 10);

    // Sprite with a different size than the texture (scaled down) and a
    // different color
    NEA_SpriteSetMaterial(Sprite[1], MaterialIcon);
    NEA_SpriteSetPos(Sprite[1], 114, 32);
    NEA_SpriteSetPriority(Sprite[1], 5);
    NEA_SpriteSetParams(Sprite[1], 15, 1, NEA_Green);
    NEA_SpriteSetSize(Sprite[1], 56, 56);

    // Sprite with a different size than the texture (scaled down), and with
    // transparency.
    NEA_SpriteSetMaterial(Sprite[2], MaterialIcon);
    NEA_SpriteSetPos(Sprite[2], 100, 50);
    NEA_SpriteSetPriority(Sprite[2], 1);
    NEA_SpriteSetParams(Sprite[2], 15, 2, NEA_White);
    NEA_SpriteSetSize(Sprite[2], 56, 56);

    // The following sprites will only use a small part of the texture

    NEA_SpriteSetMaterial(Sprite[3], MaterialAssets);
    NEA_SpriteSetPos(Sprite[3], 50, 60);
    NEA_SpriteSetMaterialCanvas(Sprite[3], 384, 0, 484, 118);
    NEA_SpriteSetSize(Sprite[3], 484 - 384, 118 - 0);

    NEA_SpriteSetMaterial(Sprite[4], MaterialAssets);
    NEA_SpriteSetPos(Sprite[4], 0, 0);
    NEA_SpriteSetMaterialCanvas(Sprite[4], 73, 0, 152, 75);
    NEA_SpriteSetSize(Sprite[4], 152 - 73, 75 - 0);

    NEA_SpriteSetMaterial(Sprite[5], MaterialAssets);
    NEA_SpriteSetPos(Sprite[5], 170, 20);
    NEA_SpriteSetMaterialCanvas(Sprite[5], 0, 77, 72, 175);
    NEA_SpriteSetSize(Sprite[5], 72 - 0, 175 - 77);

    int rot = 0;
    int x = 100, y = 50;

    printf("PAD:   Move\n");
    printf("START: Exit to loader\n");

    while (1)
    {
        NEA_WaitForVBL(0);

        scanKeys();
        uint32_t keys = keysHeld();

        rot = (rot + 2) & 511;
        NEA_SpriteSetRot(Sprite[2], rot);

        if (keys & KEY_UP)
            y--;
        if (keys & KEY_DOWN)
            y++;
        if (keys & KEY_RIGHT)
            x++;
        if (keys & KEY_LEFT)
            x--;

        if (keys & KEY_START)
            break;

        NEA_SpriteSetPos(Sprite[2], x, y);

        NEA_ProcessDual(Draw3DScene1, Draw3DScene2);
    }

    return 0;
}
