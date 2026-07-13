// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Nitro Engine Advanced, 2024
//
// This file is part of Nitro Engine Advanced

// RAM-backed material / palette example.
//
// VRAM for textures and palettes is scarce on the DS. This example shows how to
// keep a material's texture AND a palette in main RAM and stream them into and
// out of VRAM on demand:
//
//   Texture (banks A-D):
//     NEA_MaterialRamInit()       : mark a material RAM-backed (before loading).
//     NEA_MaterialTexVramLoad()   : upload the RAM copy into texture VRAM.
//     NEA_MaterialTexVramUnload() : free the texture VRAM, keep the RAM copy.
//
//   Palette (banks E/F/G):
//     NEA_PaletteRamInit()        : mark a palette RAM-backed (before loading).
//     NEA_PaletteVramLoad()       : upload the RAM copy into palette VRAM.
//     NEA_PaletteVramUnload()     : free the palette VRAM, keep the RAM copy.
//
// On boot it runs automated self-checks (results on the bottom screen) and then
// lets you stream each resource in/out with the keys. When a texture is not
// resident, its quad is drawn untextured; when a palette is not resident, the
// paletted quad keeps its texture but the colors are left unchanged.

#include <NEAMain.h>

#include "texture.h"
#include "a3pal32.h"

// Left quad: 128x128 direct-color texture (NEA_A1RGB5, 16 bpp), RAM-backed.
#define TEX_W           128
#define TEX_H           128
#define TEX_BYTES       (TEX_W * TEX_H * 2)

// Right quad: 64x200 paletted texture (NEA_A3PAL32) with a 32-color palette.
// The texture stays resident; only the palette is RAM-backed here.
#define PAL_TEX_W       64
#define PAL_TEX_H       200
#define PAL_COLORS      32
#define PAL_BYTES       (PAL_COLORS * 2)

typedef struct {
    NEA_Material *TexMat;   // RAM-backed texture, no palette
    NEA_Material *PalMat;   // resident texture + RAM-backed palette
    NEA_Palette *Palette;
} SceneData;

void Draw3DScene(void *arg)
{
    SceneData *Scene = arg;

    NEA_2DViewInit();

    NEA_2DDrawTexturedQuad(16, 32,
                          16 + TEX_W, 32 + TEX_H,
                          0, Scene->TexMat);

    NEA_2DDrawTexturedQuad(176, 20,
                          176 + PAL_TEX_W, 20 + PAL_TEX_H,
                          0, Scene->PalMat);
}

int main(int argc, char *argv[])
{
    SceneData Scene = { 0 };

    irqEnable(IRQ_HBLANK);
    irqSet(IRQ_VBLANK, NEA_VBLFunc);
    irqSet(IRQ_HBLANK, NEA_HBLFunc);

    NEA_Init3D();
    // libnds uses VRAM_C for the text console, so reserve only banks A and B for
    // textures. Palettes use bank E, which is unaffected.
    NEA_TextureSystemReset(0, 0, NEA_VRAM_AB);
    consoleDemoInit();

    // ---- Material 2: resident paletted texture + RAM-backed palette ---------
    //
    // This texture stays resident in VRAM the whole time. It is set up FIRST so
    // that the texture-VRAM baseline captured below (for Material 1) already
    // accounts for it, and the RAM-streaming deltas measure only Material 1.

    Scene.PalMat = NEA_MaterialCreate();
    NEA_MaterialTexLoad(Scene.PalMat, NEA_A3PAL32, PAL_TEX_W, PAL_TEX_H,
                       NEA_TEXGEN_TEXCOORD, a3pal32Bitmap);

    Scene.Palette = NEA_PaletteCreate();
    NEA_PaletteRamInit(Scene.Palette);

    int palBaseline = NEA_PaletteFreeMem();
    NEA_PaletteLoad(Scene.Palette, a3pal32Pal, PAL_COLORS, NEA_A3PAL32);
    int palAfterLoad = NEA_PaletteFreeMem();

    NEA_MaterialSetPalette(Scene.PalMat, Scene.Palette);

    // ---- Material 1: RAM-backed texture (no palette) ------------------------
    //
    // The texture baseline is captured here, with Material 2's texture already
    // resident, so the deltas below isolate Material 1's texture.

    Scene.TexMat = NEA_MaterialCreate();
    NEA_MaterialRamInit(Scene.TexMat);

    int texBaseline = NEA_TextureFreeMem();
    NEA_MaterialTexLoad(Scene.TexMat, NEA_A1RGB5, TEX_W, TEX_H,
                       NEA_TEXGEN_TEXCOORD, textureBitmap);
    int texAfterLoad = NEA_TextureFreeMem();

    // ---- Automated self-check ----------------------------------------------

    printf("RAM-backed texture + palette\n");
    printf("============================\n\n");

    // Texture: RAM-backed load must not touch texture VRAM.
    bool t1 = (texAfterLoad == texBaseline)
              && (Scene.TexMat->texindex == NEA_NO_TEXTURE)
              && (Scene.TexMat->ram_data != NULL);
    printf("[%s] tex load keeps VRAM free\n", t1 ? "PASS" : "FAIL");

    // Texture: insert uses exactly the texture size.
    NEA_MaterialTexVramLoad(Scene.TexMat);
    int texResident = NEA_TextureFreeMem();
    bool t2 = (texBaseline - texResident == TEX_BYTES)
              && (Scene.TexMat->texindex != NEA_NO_TEXTURE);
    printf("[%s] tex insert uses %d B\n", t2 ? "PASS" : "FAIL",
           texBaseline - texResident);

    // Texture: remove returns VRAM, keeps RAM copy.
    NEA_MaterialTexVramUnload(Scene.TexMat);
    bool t3 = (NEA_TextureFreeMem() == texBaseline)
              && (Scene.TexMat->ram_data != NULL);
    printf("[%s] tex remove frees VRAM\n", t3 ? "PASS" : "FAIL");

    // Texture: re-insert from the surviving RAM copy.
    NEA_MaterialTexVramLoad(Scene.TexMat);
    bool t4 = (NEA_TextureFreeMem() == texResident)
              && (Scene.TexMat->texindex != NEA_NO_TEXTURE);
    printf("[%s] tex re-insert from RAM\n\n", t4 ? "PASS" : "FAIL");

    // Palette: RAM-backed load must not touch palette VRAM.
    bool p1 = (palAfterLoad == palBaseline)
              && (Scene.Palette->index == NEA_NO_PALETTE)
              && (Scene.Palette->ram_data != NULL);
    printf("[%s] pal load keeps VRAM free\n", p1 ? "PASS" : "FAIL");

    // Palette: insert uses exactly numcolors*2 bytes.
    NEA_PaletteVramLoad(Scene.Palette);
    int palResident = NEA_PaletteFreeMem();
    bool p2 = (palBaseline - palResident == PAL_BYTES)
              && (Scene.Palette->index != NEA_NO_PALETTE);
    printf("[%s] pal insert uses %d B\n", p2 ? "PASS" : "FAIL",
           palBaseline - palResident);

    // Palette: remove returns VRAM, keeps RAM copy.
    NEA_PaletteVramUnload(Scene.Palette);
    bool p3 = (NEA_PaletteFreeMem() == palBaseline)
              && (Scene.Palette->ram_data != NULL);
    printf("[%s] pal remove frees VRAM\n", p3 ? "PASS" : "FAIL");

    // Palette: re-insert from the surviving RAM copy.
    NEA_PaletteVramLoad(Scene.Palette);
    bool p4 = (NEA_PaletteFreeMem() == palResident)
              && (Scene.Palette->index != NEA_NO_PALETTE);
    printf("[%s] pal re-insert from RAM\n\n", p4 ? "PASS" : "FAIL");

    bool all = t1 && t2 && t3 && t4 && p1 && p2 && p3 && p4;
    printf("%s\n\n", all ? "ALL TESTS PASSED" : "SOME TESTS FAILED");

    printf("A/B: insert/remove texture\n");
    printf("X/Y: insert/remove palette\n");

    // Both resources are resident after the self-check above.
    while (1)
    {
        NEA_WaitForVBL(0);

        scanKeys();
        uint32_t keys = keysDown();

        if (keys & KEY_A)
            NEA_MaterialTexVramLoad(Scene.TexMat);
        if (keys & KEY_B)
            NEA_MaterialTexVramUnload(Scene.TexMat);
        if (keys & KEY_X)
            NEA_PaletteVramLoad(Scene.Palette);
        if (keys & KEY_Y)
            NEA_PaletteVramUnload(Scene.Palette);

        NEA_ProcessArg(Draw3DScene, &Scene);
    }

    return 0;
}
