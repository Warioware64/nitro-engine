// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Nitro Engine Advanced Contributors, 2026
//
// This file is part of Nitro Engine Advanced

// This example puts a trimmed texture on a real model.
//
// examples/loading/trimmed_texture makes the same point on a flat quad, which
// only ever samples the texture head on. A rotating, UV mapped model samples it
// from every angle and across the whole seam, so if trimming were lossy in any
// way, this is where it would show.
//
// The mesh is the robot rather than the teapot for a reason worth knowing. The
// teapot's texture coordinates run up to 2, so it expects the texture to repeat
// on both axes, and repeating on T is the one thing a trimmed texture cannot
// do. NEA_TEXTURE_WRAP_S is always safe, because the width really is a power of
// two; NEA_TEXTURE_WRAP_T is not, because the GPU repeats at the height it was
// told. The robot's coordinates stay inside 0..1, so it needs neither, but
// NEA_TEXTURE_WRAP_S is switched on below anyway to show that it costs a trimmed
// texture nothing.
//
// brick_256x160.png is the brick wall from examples/assets resized to 160 rows,
// which is all this model needs. Its courses run horizontally, so any mistake on
// the T axis would show up as shifted or squashed bricks. The GPU can only be
// told power of two sizes, so the texture is declared as 256x256, but only the
// 160 real rows are stored:
//
//   trimmed  256 * 160 = 40960 bytes
//   padded   256 * 256 = 65536 bytes
//
// obj2dl was given "--texture 256 160", so it scaled the V coordinates by 160,
// and ptexconv stores the image at rows 0..159 whether it trims or pads. The
// two textures must therefore be interchangeable.
//
// That is what the A button tests. Rather than stand two models side by side,
// where perspective shows each of them from a different angle and small
// differences are impossible to judge, this swaps the material under one model.
// Everything else -- mesh, display list, position, angle, light -- is held
// fixed, so any difference at all between trimmed and padded would show up as a
// flicker the moment the button goes down. There is none.
//
// The rule to respect is that the texture coordinates stay inside the real
// height, which is exactly what obj2dl guarantees here, and that
// NEA_TEXTURE_WRAP_T stays off: the GPU wraps at 256, not at 160, so repeating
// on the T axis would sample the rows that were never stored.

#include <NEAMain.h>

#include "robot_bin.h"
#include "brick_trim_tex_bin.h"
#include "brick_trim_pal_bin.h"
#include "brick_pad_tex_bin.h"
#include "brick_pad_pal_bin.h"

// The source image. Its height is deliberately not a power of two.
#define BRICK_WIDTH    256
#define BRICK_HEIGHT   160
#define BRICK_PADDED   256

typedef struct {
    NEA_Camera *Camera;
    NEA_Model *Model;
} SceneData;

void Draw3DScene(void *arg)
{
    SceneData *Scene = arg;

    NEA_CameraUse(Scene->Camera);

    NEA_PolyFormat(31, 0, NEA_LIGHT_0, NEA_CULL_BACK, 0);

    NEA_ModelDraw(Scene->Model);
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
                  -8, 3.4, 0, // Position
                  0, 3.4, 0,  // Look at
                  0, 1, 0);   // Up direction

    NEA_Material *TrimmedMat = NEA_MaterialCreate();
    NEA_Palette *TrimmedPal = NEA_PaletteCreate();
    NEA_Material *PaddedMat = NEA_MaterialCreate();
    NEA_Palette *PaddedPal = NEA_PaletteCreate();

    // Load the trimmed texture. It holds 160 rows, and that is the height we
    // declare: Nitro Engine Advanced allocates 256 * 160 bytes and tells the
    // GPU the texture is 256 * 256.
    //
    // NEA_TEXTURE_WRAP_S is fine: the width is a real power of two, so the GPU
    // repeats it where the data actually ends. NEA_TEXTURE_WRAP_T would not be,
    // which is why it is absent. Build with NEA_DEBUG to be told if you add it.
    int free_before = NEA_TextureFreeMem();
    NEA_MaterialTexLoad(TrimmedMat, NEA_PAL256, BRICK_WIDTH, BRICK_HEIGHT,
                        NEA_TEXGEN_TEXCOORD | NEA_TEXTURE_WRAP_S, brick_trim_tex_bin);
    int trimmed_bytes = free_before - NEA_TextureFreeMem();

    // The same image padded up to 256x256 by ptexconv. Rows 160 to 255 are
    // stored, uploaded and never sampled.
    free_before = NEA_TextureFreeMem();
    NEA_MaterialTexLoad(PaddedMat, NEA_PAL256, BRICK_WIDTH, BRICK_PADDED,
                        NEA_TEXGEN_TEXCOORD | NEA_TEXTURE_WRAP_S, brick_pad_tex_bin);
    int padded_bytes = free_before - NEA_TextureFreeMem();

    NEA_PaletteLoad(TrimmedPal, (void *)brick_trim_pal_bin, 256, NEA_PAL256);
    NEA_PaletteLoad(PaddedPal, (void *)brick_pad_pal_bin, 256, NEA_PAL256);
    NEA_MaterialSetPalette(TrimmedMat, TrimmedPal);
    NEA_MaterialSetPalette(PaddedMat, PaddedPal);

    // One model, whose display list has texture coordinates running from 0 to
    // 160 on the T axis. That is correct for either texture, which is the point.
    Scene.Model = NEA_ModelCreate(NEA_Static);
    NEA_ModelLoadStaticMesh(Scene.Model, robot_bin);
    NEA_ModelSetMaterial(Scene.Model, TrimmedMat);

    NEA_LightSet(0, NEA_White, -0.9, 0, 0);

    printf("\x1b[0;0HTrimmed texture on a model\n");
    printf("==========================\n\n");
    printf("brick texture is %dx%d\n\n", BRICK_WIDTH, BRICK_HEIGHT);

    printf("trimmed (ptexconv -tt)\n");
    printf("  rows stored: %d\n", NEA_TextureGetSizeY(TrimmedMat));
    printf("  GPU is told: %d\n", NEA_TextureGetRealSizeY(TrimmedMat));
    printf("  VRAM:        %d B\n\n", trimmed_bytes);

    printf("padded\n");
    printf("  rows stored: %d\n", NEA_TextureGetSizeY(PaddedMat));
    printf("  GPU is told: %d\n", NEA_TextureGetRealSizeY(PaddedMat));
    printf("  VRAM:        %d B\n\n", padded_bytes);

    printf("saved %d B (%d%%)\n\n",
           padded_bytes - trimmed_bytes,
           100 * (padded_bytes - trimmed_bytes) / padded_bytes);

    printf("Hold A for the padded texture.\n");
    printf("Nothing must change.\n");
    printf("Pad: rotate.\n");

    int rot_x = 0, rot_y = 0;

    while (1)
    {
        NEA_WaitForVBL(0);

        scanKeys();
        uint32_t keys = keysHeld();

        if (keys & KEY_UP)
            rot_x += 2;
        if (keys & KEY_DOWN)
            rot_x -= 2;
        if (keys & KEY_LEFT)
            rot_y += 2;
        if (keys & KEY_RIGHT)
            rot_y -= 2;

        // Turn slowly on its own so the swap can be tried from every angle
        // without touching the pad.
        rot_y += 1;

        NEA_ModelSetRot(Scene.Model, rot_x, rot_y, 0);

        // The whole test: swap the material, change nothing else.
        NEA_ModelSetMaterial(Scene.Model,
                             (keys & KEY_A) ? PaddedMat : TrimmedMat);

        printf("\x1b[21;0H  %s   ", (keys & KEY_A) ? "padded " : "trimmed");

        NEA_ProcessArg(Draw3DScene, &Scene);
    }

    return 0;
}
