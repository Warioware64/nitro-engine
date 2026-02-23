// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Antonio Niño Díaz, 2024

#include <stdio.h>

#include <filesystem.h>
#include <nds.h>
#include <NEAMain.h>

typedef struct {
    NEA_Sprite *TextSprite;
} SceneData;

void Draw3DScene1(void *arg)
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

__attribute__((noreturn)) void WaitLoop(void)
{
    printf("Press START to exit");
    while (1)
    {
        swiWaitForVBlank();
        scanKeys();
        if (keysHeld() & KEY_START)
            exit(0);
    }
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

    if (!nitroFSInit(NULL))
    {
        printf("nitroFSInit failed.\n");
        WaitLoop();
    }

    // Load a 16-color font to be used for rendering text as quads

    NEA_RichTextInit(0);
    NEA_RichTextMetadataLoadFAT(0, "fonts/font.fnt");
    NEA_RichTextMaterialLoadGRF(0, "fonts/font_16_png.grf");

    // Load a 256-color font to be used for rendering text as quads

    NEA_RichTextInit(1);
    NEA_RichTextMetadataLoadFAT(1, "fonts/font.fnt");
    NEA_RichTextMaterialLoadGRF(1, "fonts/font_256_png.grf");

    // Load a 16-color font to be used for rendering text to textures.

    NEA_RichTextInit(2);
    NEA_RichTextMetadataLoadFAT(2, "fonts/font.fnt");
    NEA_RichTextBitmapLoadGRF(2, "fonts/font_16_png.grf");

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

        NEA_ProcessDualArg(Draw3DScene1, Draw3DScene2, &Scene, &Scene);

        scanKeys();
        if (keysHeld() & KEY_START)
            break;
    }

    NEA_SpriteDelete(Scene.TextSprite);
    NEA_MaterialDelete(Material);

    NEA_RichTextResetSystem();

    return 0;
}
