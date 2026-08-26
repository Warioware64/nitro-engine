// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Nitro Engine Advanced Contributors, 2026
//
// This file is part of Nitro Engine Advanced

// Important: This example won't work on devkitPro. GRF loading functions are
// only supported in BlocksDS.

// This example loads trimmed textures from GRF files.
//
// A GRF is a container: it holds the texels, the palette indices and the
// palette in one file, each of them behind a BIOS compression header. The size
// and format come from the GRF header rather than from the caller, which is why
// NEA_MaterialTexLoadGRF() has no sizeX/sizeY arguments.
//
// That is also why trimming needs nothing new here. "ptexconv -og -tt" writes
// the trimmed rows into the GFX chunk and the *real* height into the header, so
// the loader gets 160 the same way it would get 256, and takes the usual
// trimmed path: allocate the rows that exist, tell the GPU the next power of
// two. See examples/loading/trimmed_texture for what that path does.
//
// The four files here separate two savings that are easy to confuse:
//
// - Trimming is a *VRAM* saving. It removes rows that were never sampled.
// - GRF compression ("-clz") is a *ROM* saving. The texture is decompressed
//   into main RAM on load and reaches VRAM at full size.
//
// They are independent and they stack, which the numbers on the sub screen show
// directly: the two trimmed paletted files occupy the same VRAM and very
// different amounts of ROM.
//
// The fourth file is tex4x4, whose GRF carries a PIDX chunk for the palette
// indices on top of the usual GFX and PAL. Its height is trimmed on the 4 row
// block grid; see examples/loading/trimmed_texture_tex4x4.
//
// As with any trimmed texture, NEA_TEXTURE_WRAP_T must stay off: the GPU wraps
// at the height it was told, not at the real one.

#include <filesystem.h>
#include <NEAMain.h>

// The source image. Its height is deliberately not a power of two.
#define BRICK_WIDTH     256
#define BRICK_HEIGHT    160

typedef struct {
    NEA_Material *Trimmed, *Padded, *TrimmedLZ, *TrimmedTex4x4;
} SceneData;

// Four quads in a 2x2 grid, all at the same scale. The trimmed ones must be
// indistinguishable from the padded one.
//
// The texture coordinates run from 0 to 160 on the T axis for all four, which
// is every row the trimmed files have and the only rows of the padded one worth
// sampling. The plain NEA_2DDrawTexturedQuad() would map each material's stored
// height onto its quad instead, dragging the padded file's 96 rows of filler
// into the comparison.
static void draw_quad(int x, int y, const NEA_Material *mat)
{
    NEA_2DDrawTexturedQuadColorCanvas(x, y, x + 128, y + 80, 0,
                                      0, 0, BRICK_WIDTH, BRICK_HEIGHT,
                                      mat, NEA_White);
}

void Draw3DScene(void *arg)
{
    SceneData *Scene = arg;

    NEA_2DViewInit();

    draw_quad(0, 16, Scene->Trimmed);
    draw_quad(128, 16, Scene->Padded);
    draw_quad(0, 104, Scene->TrimmedLZ);
    draw_quad(128, 104, Scene->TrimmedTex4x4);
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

// Size of a file in NitroFS, which for these files is the size they take up in
// the ROM. This is the number that GRF compression changes.
static int rom_size(const char *path)
{
    FILE *f = fopen(path, "rb");
    if (f == NULL)
        return -1;

    fseek(f, 0, SEEK_END);
    int size = ftell(f);
    fclose(f);
    return size;
}

// Loads one GRF and reports what it cost in VRAM. Returns 0 on failure.
static int load_grf(NEA_Material *mat, NEA_Palette *pal, const char *path,
                    int *vram_bytes)
{
    int free_before = NEA_TextureFreeMem();

    // No NEA_TEXTURE_WRAP_T: these textures are trimmed.
    if (NEA_MaterialTexLoadGRF(mat, pal, NEA_TEXGEN_TEXCOORD, path) == 0)
        return 0;

    *vram_bytes = free_before - NEA_TextureFreeMem();
    return 1;
}

int main(int argc, char *argv[])
{
    SceneData Scene = { 0 };

    irqEnable(IRQ_HBLANK);
    irqSet(IRQ_VBLANK, NEA_VBLFunc);
    irqSet(IRQ_HBLANK, NEA_HBLFunc);

    consoleDemoInit();

    if (!nitroFSInit(NULL))
    {
        printf("nitroFSInit failed.\n");
        WaitLoop();
    }

    NEA_Init3D();

    // libnds uses VRAM_C for the text console, reserve A and B only
    NEA_TextureSystemReset(0, 0, NEA_VRAM_AB);

    Scene.Trimmed = NEA_MaterialCreate();
    Scene.Padded = NEA_MaterialCreate();
    Scene.TrimmedLZ = NEA_MaterialCreate();
    Scene.TrimmedTex4x4 = NEA_MaterialCreate();

    NEA_Palette *TrimmedPal = NEA_PaletteCreate();
    NEA_Palette *PaddedPal = NEA_PaletteCreate();
    NEA_Palette *TrimmedLZPal = NEA_PaletteCreate();
    NEA_Palette *TrimmedTex4x4Pal = NEA_PaletteCreate();

    int trim_vram = 0, pad_vram = 0, lz_vram = 0, t4_vram = 0;

    // The tex4x4 texture is loaded first, and that is not cosmetic. Its two
    // halves cannot go just anywhere: the texels live in one texture slot and
    // the palette indices in another, at an address the hardware derives from
    // the first, so the loader needs a *matching pair* of free addresses rather
    // than one free block. Load it after the others and that pairing can fail
    // while VRAM still reports over a hundred kilobytes free, because the
    // unconstrained textures have landed across both slots in the meantime.
    // Allocate the constrained texture while the space is still clean.
    if (!load_grf(Scene.TrimmedTex4x4, TrimmedTex4x4Pal,
                  "brick_t4_trim_lz.grf", &t4_vram)
        || !load_grf(Scene.Trimmed, TrimmedPal, "brick_trim.grf", &trim_vram)
        || !load_grf(Scene.Padded, PaddedPal, "brick_pad.grf", &pad_vram)
        || !load_grf(Scene.TrimmedLZ, TrimmedLZPal, "brick_trim_lz.grf",
                     &lz_vram))
    {
        printf("Failed to load a GRF file\n");
        WaitLoop();
    }

    printf("\x1b[0;0HTrimmed textures from GRF\n");
    printf("=========================\n\n");
    printf("brick is %dx%d\n\n", BRICK_WIDTH, BRICK_HEIGHT);
    printf("            rows  ROM   VRAM\n");
    printf("trim pal256 %4d %5d %6d\n",
           NEA_TextureGetSizeY(Scene.Trimmed),
           rom_size("brick_trim.grf"), trim_vram);
    printf("pad  pal256 %4d %5d %6d\n",
           NEA_TextureGetSizeY(Scene.Padded),
           rom_size("brick_pad.grf"), pad_vram);
    printf("trim +lz    %4d %5d %6d\n",
           NEA_TextureGetSizeY(Scene.TrimmedLZ),
           rom_size("brick_trim_lz.grf"), lz_vram);
    printf("trim tex4x4 %4d %5d %6d\n\n",
           NEA_TextureGetSizeY(Scene.TrimmedTex4x4),
           rom_size("brick_t4_trim_lz.grf"), t4_vram);

    printf("GPU is told %d rows for all\n", NEA_TextureGetRealSizeY(Scene.Trimmed));
    printf("four of them.\n\n");
    printf("Trimming saves VRAM, -clz\n");
    printf("saves ROM. They stack.\n\n");
    printf("All four quads must look the\n");
    printf("same (tex4x4 is lossy).\n");

    while (1)
    {
        NEA_WaitForVBL(0);

        NEA_ProcessArg(Draw3DScene, &Scene);
    }

    return 0;
}
