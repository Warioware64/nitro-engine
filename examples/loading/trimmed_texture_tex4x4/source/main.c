// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Nitro Engine Advanced Contributors, 2026
//
// This file is part of Nitro Engine Advanced

// This example trims a Texel 4x4 (compressed) texture.
//
// The T axis of a tex4x4 texture can be trimmed like any other, but its height
// rule is different, and that is the reason this example is separate from
// examples/loading/trimmed_texture.
//
// A tex4x4 texture is stored as rows of 4x4 blocks, so its height has to be a
// multiple of 4 rather than a power of two. Anything from 4 to 1024 in steps of
// 4 is a valid height, and Nitro Engine Advanced tells the GPU the next power of
// two, exactly as it does for the uncompressed formats. ptexconv rounds up to
// the block grid when it trims: give it a 66 pixel tall image and it writes 68
// rows, of which the last two are padding inside the final block row. The
// texture coordinates still have to be scaled by 66, the real image height.
//
// Here the image is 160 tall, which is already a multiple of 4, so there is no
// rounding. The data comes in two halves, as it always does for tex4x4:
//
//              trimmed (160 rows)      padded (256 rows)
//   texels     256 * 160 / 4 = 10240   256 * 256 / 4 = 16384
//   indices             / 2 =   5120           / 2 =   8192
//   total                      15360                    24576
//
// The two halves land in different texture slots, so both shrink together; the
// saving is the same 37.5% the uncompressed formats get. The palette is not
// affected by trimming, and both files below share an identical one.
//
// NEA_TEXTURE_WRAP_T must stay off, as with any trimmed texture: the GPU wraps
// at the height it was told (256), not at the real one (160).

#include <NEAMain.h>

#include "landscape_trim_tex_bin.h"
#include "landscape_trim_idx_bin.h"
#include "landscape_trim_pal_bin.h"
#include "landscape_pad_tex_bin.h"
#include "landscape_pad_idx_bin.h"
#include "landscape_pad_pal_bin.h"

// The source image. Its height is a multiple of 4 but not a power of two.
#define LANDSCAPE_WIDTH     256
#define LANDSCAPE_HEIGHT    160
#define LANDSCAPE_PADDED    256

typedef struct {
    NEA_Material *Trimmed, *Padded;
} SceneData;

// Both quads are drawn at exactly half the size of the image, so the aspect
// ratio is preserved and the two are scaled identically. Any row that went
// missing from the trimmed texture would show up as a band along the bottom of
// the left quad.
//
// The texture coordinates are given explicitly, running from 0 to 160 on the T
// axis for *both* materials. That is the whole comparison: 160 is every row the
// trimmed texture has, and the only rows of the padded one worth sampling. The
// plain NEA_2DDrawTexturedQuad() would map each material's stored height onto
// the quad instead, which for the padded texture means dragging in the 96 rows
// of filler ptexconv put there, and those are not a copy of anything.
void Draw3DScene(void *arg)
{
    SceneData *Scene = arg;

    NEA_ClearColorSet(RGB15(5, 5, 10), 31, 63);

    NEA_2DViewInit();

    NEA_2DDrawTexturedQuadColorCanvas(0, 56, 128, 56 + 80, 0,
                                      0, 0, LANDSCAPE_WIDTH, LANDSCAPE_HEIGHT,
                                      Scene->Trimmed, NEA_White);

    NEA_2DDrawTexturedQuadColorCanvas(128, 56, 256, 56 + 80, 0,
                                      0, 0, LANDSCAPE_WIDTH, LANDSCAPE_HEIGHT,
                                      Scene->Padded, NEA_White);
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

    Scene.Trimmed = NEA_MaterialCreate();
    Scene.Padded = NEA_MaterialCreate();

    NEA_Palette *TrimmedPal = NEA_PaletteCreate();
    NEA_Palette *PaddedPal = NEA_PaletteCreate();

    // 160 is the height we declare, and it is the height the data really has.
    // Both halves are sized from it: 256 * 160 / 4 texels and half that many
    // palette indices. Note there is no NEA_TEXTURE_WRAP_T here.
    int free_before = NEA_TextureFreeMem();
    NEA_MaterialTex4x4Load(Scene.Trimmed, LANDSCAPE_WIDTH, LANDSCAPE_HEIGHT,
                           NEA_TEXGEN_TEXCOORD,
                           landscape_trim_tex_bin, landscape_trim_idx_bin);
    int trimmed_bytes = free_before - NEA_TextureFreeMem();

    NEA_PaletteLoadSize(TrimmedPal, landscape_trim_pal_bin,
                        landscape_trim_pal_bin_size, NEA_TEX4X4);
    NEA_MaterialSetPalette(Scene.Trimmed, TrimmedPal);

    // The same image padded up to 256x256 by ptexconv. Block rows 40 to 63 are
    // stored, uploaded and never sampled.
    free_before = NEA_TextureFreeMem();
    NEA_MaterialTex4x4Load(Scene.Padded, LANDSCAPE_WIDTH, LANDSCAPE_PADDED,
                           NEA_TEXGEN_TEXCOORD,
                           landscape_pad_tex_bin, landscape_pad_idx_bin);
    int padded_bytes = free_before - NEA_TextureFreeMem();

    NEA_PaletteLoadSize(PaddedPal, landscape_pad_pal_bin,
                        landscape_pad_pal_bin_size, NEA_TEX4X4);
    NEA_MaterialSetPalette(Scene.Padded, PaddedPal);

    printf("\x1b[0;0HTrimmed tex4x4\n");
    printf("==============\n\n");
    printf("landscape is %dx%d\n\n", LANDSCAPE_WIDTH, LANDSCAPE_HEIGHT);

    printf("left   trimmed (ptexconv -tt)\n");
    printf("  rows stored: %d\n", NEA_TextureGetSizeY(Scene.Trimmed));
    printf("  GPU is told: %d\n", NEA_TextureGetRealSizeY(Scene.Trimmed));
    printf("  VRAM:        %d B\n\n", trimmed_bytes);

    printf("right  padded\n");
    printf("  rows stored: %d\n", NEA_TextureGetSizeY(Scene.Padded));
    printf("  GPU is told: %d\n", NEA_TextureGetRealSizeY(Scene.Padded));
    printf("  VRAM:        %d B\n\n", padded_bytes);

    printf("saved %d B (%d%%)\n\n",
           padded_bytes - trimmed_bytes,
           100 * (padded_bytes - trimmed_bytes) / padded_bytes);

    printf("Both quads must look the same.\n");
    printf("A tex4x4 height must be a\n");
    printf("multiple of 4, not a power of\n");
    printf("two.\n");

    while (1)
    {
        NEA_WaitForVBL(0);

        NEA_ProcessArg(Draw3DScene, &Scene);
    }

    return 0;
}
