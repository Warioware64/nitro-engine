// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Nitro Engine Advanced Contributors, 2026
//
// This file is part of Nitro Engine Advanced

// This example shows how to get a textured model into VRAM without paying for
// the padding of a texture whose height isn't a power of two.
//
// The GPU can only be told power-of-two texture sizes, so a 256x160 image has
// to be declared as 256x256. It does not have to be *stored* as 256x256: the
// rows are contiguous, so the last 96 rows can simply be left out. That is what
// "ptexconv -tt" does, and what Nitro Engine Advanced expects when you pass a
// height that isn't a power of two: it allocates only the rows you gave it and
// tells the GPU the padded height. The rows in between belong to whatever
// texture is allocated next.
//
// Two things have to hold for this to be safe, and both are true here:
//
// - The texture coordinates must stay inside the real height. obj2dl scales
//   UVs by the height passed to "--texture", so the same display list works
//   for the trimmed and the padded texture.
//
// - NEA_TEXTURE_WRAP_T must be off. The GPU wraps at the height it was told
//   (256), not at the real one (160), so repeating on the T axis would sample
//   the rows that were never stored.
//
// The width is a different matter: see examples/loading/incomplete_texture for
// why it always has to be a power of two.
//
// The same idea applied elsewhere:
//
// - trimmed_texture_model     a UV mapped model instead of a quad
// - trimmed_texture_animated  an animated model, UVs from md5_to_dsma
// - trimmed_texture_tex4x4    the compressed format, whose height rule differs
// - trimmed_texture_grf       trimmed textures inside GRF files

#include <NEAMain.h>

#include "banner_bin.h"
#include "banner_trim_tex_bin.h"
#include "banner_trim_pal_bin.h"
#include "banner_pad_tex_bin.h"
#include "banner_pad_pal_bin.h"

// The source image. Its height is deliberately not a power of two.
#define BANNER_WIDTH    256
#define BANNER_HEIGHT   160
#define BANNER_PADDED   256

typedef struct {
    NEA_Camera *Camera;
    NEA_Model *Trimmed, *Padded;
} SceneData;

void Draw3DScene(void *arg)
{
    SceneData *Scene = arg;

    NEA_CameraUse(Scene->Camera);

    NEA_ModelDraw(Scene->Trimmed);
    NEA_ModelDraw(Scene->Padded);
}

int main(int argc, char *argv[])
{
    SceneData Scene = { 0 };

    irqEnable(IRQ_HBLANK);
    irqSet(IRQ_VBLANK, NEA_VBLFunc);
    irqSet(IRQ_HBLANK, NEA_HBLFunc);

    NEA_Init3D();

    // libnds uses VRAM_C for the text console, reserve A and B only
    NEA_TextureSystemReset(0, 0, NEA_VRAM_AB);
    consoleDemoInit();

    Scene.Camera = NEA_CameraCreate();
    NEA_CameraSet(Scene.Camera,
                  0, 0, 4.6, // Position
                  0, 0, 0,  // Look at
                  0, 1, 0); // Up direction

    NEA_Material *TrimmedMat = NEA_MaterialCreate();
    NEA_Palette *TrimmedPal = NEA_PaletteCreate();
    NEA_Material *PaddedMat = NEA_MaterialCreate();
    NEA_Palette *PaddedPal = NEA_PaletteCreate();

    // Load the trimmed texture. It holds 160 rows, and that is the height we
    // declare: Nitro Engine Advanced allocates 256 * 160 bytes and tells the
    // GPU the texture is 256 * 256. Note there is no NEA_TEXTURE_WRAP_T here.
    int free_before = NEA_TextureFreeMem();
    NEA_MaterialTexLoad(TrimmedMat, NEA_PAL256, BANNER_WIDTH, BANNER_HEIGHT,
                        NEA_TEXGEN_TEXCOORD, banner_trim_tex_bin);
    int trimmed_bytes = free_before - NEA_TextureFreeMem();

    // The same image padded up to 256x256 by ptexconv. Rows 160 to 255 are
    // stored, uploaded and never sampled.
    free_before = NEA_TextureFreeMem();
    NEA_MaterialTexLoad(PaddedMat, NEA_PAL256, BANNER_WIDTH, BANNER_PADDED,
                        NEA_TEXGEN_TEXCOORD, banner_pad_tex_bin);
    int padded_bytes = free_before - NEA_TextureFreeMem();

    NEA_PaletteLoad(TrimmedPal, (void *)banner_trim_pal_bin, 256, NEA_PAL256);
    NEA_PaletteLoad(PaddedPal, (void *)banner_pad_pal_bin, 256, NEA_PAL256);
    NEA_MaterialSetPalette(TrimmedMat, TrimmedPal);
    NEA_MaterialSetPalette(PaddedMat, PaddedPal);

    // Both models load the very same display list. Its texture coordinates run
    // from 0 to 160 on the T axis, which is correct for either texture.
    Scene.Trimmed = NEA_ModelCreate(NEA_Static);
    Scene.Padded = NEA_ModelCreate(NEA_Static);
    NEA_ModelLoadStaticMesh(Scene.Trimmed, banner_bin);
    NEA_ModelLoadStaticMesh(Scene.Padded, banner_bin);
    NEA_ModelSetMaterial(Scene.Trimmed, TrimmedMat);
    NEA_ModelSetMaterial(Scene.Padded, PaddedMat);

    NEA_ModelTranslate(Scene.Trimmed, -1.70, 0, 0);
    NEA_ModelTranslate(Scene.Padded, 1.70, 0, 0);

    // The quads face +Z, so the light has to travel towards -Z
    NEA_LightSet(0, NEA_White, 0, 0, -1);

    printf("\x1b[0;0HTrimmed vs padded T axis\n");
    printf("========================\n\n");
    printf("banner.png is %dx%d\n\n", BANNER_WIDTH, BANNER_HEIGHT);

    printf("left   trimmed (ptexconv -tt)\n");
    printf("  rows stored: %d\n", NEA_TextureGetSizeY(TrimmedMat));
    printf("  GPU is told: %d\n", NEA_TextureGetRealSizeY(TrimmedMat));
    printf("  VRAM:        %d B\n\n", trimmed_bytes);

    printf("right  padded\n");
    printf("  rows stored: %d\n", NEA_TextureGetSizeY(PaddedMat));
    printf("  GPU is told: %d\n", NEA_TextureGetRealSizeY(PaddedMat));
    printf("  VRAM:        %d B\n\n", padded_bytes);

    printf("saved %d B (%d%%)\n\n",
           padded_bytes - trimmed_bytes,
           100 * (padded_bytes - trimmed_bytes) / padded_bytes);

    printf("Both quads use the same display\n");
    printf("list and must look identical.\n");

    while (1)
    {
        NEA_WaitForVBL(0);

        scanKeys();

        NEA_ProcessArg(Draw3DScene, &Scene);
    }

    return 0;
}
