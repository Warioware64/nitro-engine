// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Nitro Engine Advanced Contributors, 2026
//
// This file is part of Nitro Engine Advanced

// This example puts a trimmed texture on an animated model.
//
// Nothing about the animation interacts with trimming: the texture coordinates
// live in the DSM display list and are the same on every frame, while the
// animation only moves the joints. The reason this example exists is that the
// texture coordinates come from a different tool here.
//
// md5_to_dsma scales the V coordinates by the height passed to "--texture", the
// same way obj2dl does. Passing the *real* height (160) rather than the padded
// one (256) is what makes the model sample rows 0..159 and nothing else. Get
// that wrong and the texture would creep up the model, which is the kind of
// thing an animation makes very obvious.
//
//   trimmed  256 * 160 = 40960 bytes
//   padded   256 * 256 = 65536 bytes
//
// See examples/loading/trimmed_texture_model for the trimmed/padded comparison
// side by side, and examples/loading/trimmed_texture for the underlying idea.
//
// As always with a trimmed texture, NEA_TEXTURE_WRAP_T must stay off: the GPU
// wraps at the height it was told (256), not at the real one (160).

#include <NEAMain.h>

#include "robot_dsm_bin.h"
#include "robot_walk_dsa_bin.h"
#include "robot_wave_dsa_bin.h"
#include "brick_trim_tex_bin.h"
#include "brick_trim_pal_bin.h"

// The source image. Its height is deliberately not a power of two.
#define BRICK_WIDTH     256
#define BRICK_HEIGHT    160

typedef struct {
    NEA_Camera *Camera;
    NEA_Model *Model;
} SceneData;

void Draw3DScene(void *arg)
{
    SceneData *Scene = arg;

    NEA_PolyFormat(31, 0, NEA_LIGHT_0, NEA_CULL_BACK, 0);

    NEA_CameraUse(Scene->Camera);

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
                  6, 3, -4, // Position
                  0, 3, 0,  // Look at
                  0, 1, 0); // Up direction

    Scene.Model = NEA_ModelCreate(NEA_Animated);

    NEA_Animation *Walk = NEA_AnimationCreate();
    NEA_Animation *Wave = NEA_AnimationCreate();
    NEA_AnimationLoad(Walk, robot_walk_dsa_bin);
    NEA_AnimationLoad(Wave, robot_wave_dsa_bin);

    NEA_ModelLoadDSM(Scene.Model, robot_dsm_bin);
    NEA_ModelSetAnimation(Scene.Model, Wave);
    NEA_ModelAnimStart(Scene.Model, NEA_ANIM_LOOP, floattof32(0.1));

    NEA_Material *Material = NEA_MaterialCreate();
    NEA_Palette *Palette = NEA_PaletteCreate();

    // 160 is the height we declare, and it is the height the data really has.
    // Nitro Engine Advanced allocates 256 * 160 bytes and tells the GPU the
    // texture is 256 * 256. Note there is no NEA_TEXTURE_WRAP_T here.
    int free_before = NEA_TextureFreeMem();
    NEA_MaterialTexLoad(Material, NEA_PAL256, BRICK_WIDTH, BRICK_HEIGHT,
                        NEA_TEXGEN_TEXCOORD, brick_trim_tex_bin);
    int trimmed_bytes = free_before - NEA_TextureFreeMem();

    NEA_PaletteLoad(Palette, (void *)brick_trim_pal_bin, 256, NEA_PAL256);
    NEA_MaterialSetPalette(Material, Palette);

    NEA_ModelSetMaterial(Scene.Model, Material);

    NEA_LightSet(0, NEA_White, -0.9, 0, 0);

    // What the same texture would have cost if it had been padded up to the
    // height the GPU is told about.
    int padded_bytes = BRICK_WIDTH * NEA_TextureGetRealSizeY(Material);

    printf("\x1b[0;0HTrimmed texture, animated\n");
    printf("=========================\n\n");
    printf("texture is %dx%d\n\n", BRICK_WIDTH, BRICK_HEIGHT);
    printf("rows stored: %d\n", NEA_TextureGetSizeY(Material));
    printf("GPU is told: %d\n", NEA_TextureGetRealSizeY(Material));
    printf("VRAM used:   %d B\n", trimmed_bytes);
    printf("if padded:   %d B\n\n", padded_bytes);
    printf("saved %d B (%d%%)\n\n",
           padded_bytes - trimmed_bytes,
           100 * (padded_bytes - trimmed_bytes) / padded_bytes);

    printf("A: wave    B: walk\n\n");
    printf("The texture must stay put on\n");
    printf("the model as it animates.\n");

    while (1)
    {
        // NEA_UPDATE_ANIMATIONS is what advances the animation. Without it the
        // model would sit on its first frame.
        NEA_WaitForVBL(NEA_UPDATE_ANIMATIONS);

        scanKeys();
        uint32_t keys = keysDown();

        if (keys & KEY_A)
        {
            NEA_ModelSetAnimation(Scene.Model, Wave);
            NEA_ModelAnimStart(Scene.Model, NEA_ANIM_LOOP, floattof32(0.1));
        }
        if (keys & KEY_B)
        {
            NEA_ModelSetAnimation(Scene.Model, Walk);
            NEA_ModelAnimStart(Scene.Model, NEA_ANIM_LOOP, floattof32(0.1));
        }

        printf("\x1b[17;0Hframe: %.2f  ",
               f32tofloat(NEA_ModelAnimGetFrame(Scene.Model)));

        NEA_ProcessArg(Draw3DScene, &Scene);
    }

    return 0;
}
