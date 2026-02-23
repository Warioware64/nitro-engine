// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Antonio Niño Díaz, 2024

#include <stdio.h>

#include <nds.h>
#include <NEAMain.h>

#include "font_fnt_bin.h"
#include "font_16.h"
#include "font_256.h"

typedef struct {
    NEA_Sprite *TextSprite;
} SceneData;

void Draw3DScene(void *arg)
{
    (void)arg;

    NEA_ClearColorSet(RGB15(0, 7, 7), 31, 63);

    NEA_2DViewInit();

    NEA_RichTextRender3D(1, "VAWATa\ntajl", 0, 0);

    NEA_RichTextRender3DAlpha(0, "Text with alpha", 10, 80,
                             POLY_ALPHA(20) | POLY_CULL_BACK, 30);
}

void Draw3DScene2(void *arg)
{
    SceneData *Scene = arg;

    NEA_ClearColorSet(RGB15(7, 0, 7), 31, 63);

    NEA_2DViewInit();

    NEA_SpriteSetPos(Scene->TextSprite, 16, 32);
    NEA_SpriteDraw(Scene->TextSprite);
}

int main(int argc, char *argv[])
{
    SceneData Scene = { 0 };

    irqEnable(IRQ_HBLANK);
    irqSet(IRQ_VBLANK, NEA_VBLFunc);
    irqSet(IRQ_HBLANK, NEA_HBLFunc);

    // Init 3D mode
    NEA_InitDual3D();
    NEA_InitConsole();

    // Load a 16-color font to be used for rendering text as quads

    NEA_RichTextInit(0);
    NEA_RichTextMetadataLoadMemory(0, font_fnt_bin, font_fnt_bin_size);

    {
        NEA_Material *Font16 = NEA_MaterialCreate();
        NEA_MaterialTexLoad(Font16, NEA_PAL16, 256, 256,
                           NEA_TEXGEN_TEXCOORD | NEA_TEXTURE_COLOR0_TRANSPARENT,
                           font_16Bitmap);

        NEA_Palette *Pal16 = NEA_PaletteCreate();
        NEA_PaletteLoad(Pal16, font_16Pal, 16, NEA_PAL16);

        NEA_MaterialSetPalette(Font16, Pal16);

        // The material and palette will be deleted when the rich text font is
        // deleted.
        NEA_RichTextMaterialSet(0, Font16, Pal16);
    }

    // Load a 256-color font to be used for rendering text as quads

    NEA_RichTextInit(1);
    NEA_RichTextMetadataLoadMemory(1, font_fnt_bin, font_fnt_bin_size);

    {
        NEA_Material *Font256 = NEA_MaterialCreate();
        NEA_MaterialTexLoad(Font256, NEA_PAL256, 256, 256,
                           NEA_TEXGEN_TEXCOORD | NEA_TEXTURE_COLOR0_TRANSPARENT,
                           font_256Bitmap);

        NEA_Palette *Pal256 = NEA_PaletteCreate();
        NEA_PaletteLoad(Pal256, font_256Pal, 256, NEA_PAL256);

        NEA_MaterialSetPalette(Font256, Pal256);

        // The material and palette will be deleted when the rich text font is
        // deleted.
        NEA_RichTextMaterialSet(1, Font256, Pal256);
    }

    // Load a 16-color font to be used for rendering text to textures.

    NEA_RichTextInit(2);
    NEA_RichTextMetadataLoadMemory(2, font_fnt_bin, font_fnt_bin_size);
    NEA_RichTextBitmapSet(2, font_16Bitmap, 256, 256, NEA_PAL16,
                         font_16Pal, font_16PalLen);

    // Render text to a texture using the last font we've loaded

    // We don't care about managing the palette. Passing NULL will tell Nitro
    // Engine to delete the palete automatically when the material is deleted.
    NEA_Material *Material = NULL;
    NEA_RichTextRenderMaterial(2,
                "Sample: AWAV.\nÿ_ßðñÑü(o´Áá)|\nInvalid char: ŋ",
                &Material, NULL);

    // Create a sprite to be used to render the texture we've rendered

    Scene.TextSprite = NEA_SpriteCreate();
    NEA_SpriteSetMaterial(Scene.TextSprite, Material);

    while (1)
    {
        NEA_WaitForVBL(0);

        NEA_ProcessDualArg(Draw3DScene, Draw3DScene2, &Scene, &Scene);

        scanKeys();
        if (keysHeld() & KEY_START)
            break;
    }

    NEA_SpriteDelete(Scene.TextSprite);
    NEA_MaterialDelete(Material);

    NEA_RichTextResetSystem();

    return 0;
}
