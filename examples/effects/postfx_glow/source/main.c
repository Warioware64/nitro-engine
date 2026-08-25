// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Nitro Engine Advanced contributors, 2026
//
// This file is part of Nitro Engine Advanced

// PPU-native glow and flash (NEAPostFX).
//
// Two effects, both built out of the 2D blend unit:
//
//   NEA_PostFXFlashSet()  - brightness up/down on the 3D layer. Costs no VRAM
//                           and no BG layer at all, just two register writes.
//                           Used here for a lightning flash and a fade to black.
//
//   NEA_PostFXGlow*()     - a colored overlay layer added on top of the scene,
//                           flat or as a vertical gradient. Used here as a
//                           flickering firelight wash.
//
// The DS has only one color special effect unit per engine, so these two cannot
// both be on at once. NEAPostFX arbitrates: whichever was enabled last owns the
// blend unit, and NEA_PostFXBlendOwner_Get() reports who that is. This example
// shows that directly -- turning on the flash visibly steals the unit from the
// glow.
//
// Cost: glow reserves 18 KB of the main BG bank (a 16 KB tile block plus a 2 KB
// map) and one BG layer. Flash costs nothing.

#include <NEAMain.h>

#include "texture.h"
#include "sphere_bin.h"

typedef struct {
    NEA_Camera *Camera;
    NEA_Model *Model, *Model2, *Model3;
} SceneData;

void Draw3DScene(void *arg)
{
    SceneData *Scene = arg;

    NEA_CameraUse(Scene->Camera);
    NEA_PolyFormat(31, 0, NEA_LIGHT_ALL, NEA_CULL_BACK, 0);

    NEA_ModelDraw(Scene->Model);
    NEA_ModelDraw(Scene->Model2);
    NEA_ModelDraw(Scene->Model3);
}

// Cheap deterministic noise for the firelight flicker. No division, no float.
static u32 rng_state = 0x2545F491;
static int flicker_next(void)
{
    rng_state = rng_state * 1103515245u + 12345u;
    return (int)((rng_state >> 16) & 0xFF);
}

int main(int argc, char *argv[])
{
    SceneData Scene = { 0 };

    irqEnable(IRQ_HBLANK);
    irqSet(IRQ_VBLANK, NEA_VBLFunc);
    irqSet(IRQ_HBLANK, NEA_HBLFunc);

    // NEA_Hw2DAutoInit() can only claim bank E for main-engine backgrounds if
    // E is not being used for texture palettes, and E is the default. Move the
    // texture palettes to F before initializing 3D so the glow layer has a bank
    // to live in.
    NEA_SetTexPaletteBank(NEA_VRAM_F);

    NEA_Init3D();
    NEA_TextureSystemReset(0, 0, NEA_VRAM_AB);
    consoleDemoInit();

    // Claim bank E for main-engine backgrounds only. NEA_Hw2DAutoInit() would
    // also reconfigure the sub engine for 2D, which undoes consoleDemoInit()
    // and leaves the bottom screen blank.
    NEA_Hw2DVRAMConfig vram = { 0 };
    vram.main_bg = NEA_VRAM_E;

    if (NEA_Hw2DInit(&vram) != 0)
    {
        printf("Hw2D init failed\n");
        while (1)
            swiWaitForVBlank();
    }

    Scene.Model = NEA_ModelCreate(NEA_Static);
    Scene.Model2 = NEA_ModelCreate(NEA_Static);
    Scene.Model3 = NEA_ModelCreate(NEA_Static);
    Scene.Camera = NEA_CameraCreate();
    NEA_Material *Material = NEA_MaterialCreate();

    NEA_CameraSet(Scene.Camera,
                 -1, 2, -1,
                  1, 1, 1,
                  0, 1, 0);

    NEA_ModelLoadStaticMesh(Scene.Model, sphere_bin);
    NEA_ModelLoadStaticMesh(Scene.Model2, sphere_bin);
    NEA_ModelLoadStaticMesh(Scene.Model3, sphere_bin);

    NEA_MaterialTexLoad(Material, NEA_A1RGB5, 256, 256, NEA_TEXGEN_TEXCOORD,
                       textureBitmap);

    NEA_ModelSetMaterial(Scene.Model, Material);
    NEA_ModelSetMaterial(Scene.Model2, Material);
    NEA_ModelSetMaterial(Scene.Model3, Material);

    NEA_LightSet(0, NEA_White, 0, -1, -1);

    NEA_ModelSetCoord(Scene.Model, 1, 0, 1);
    NEA_ModelSetCoord(Scene.Model2, 3, 1, 3);
    NEA_ModelSetCoord(Scene.Model3, 7, 2, 7);

    // Glow on BG layer 1, owning palette slot 0.
    if (!NEA_PostFXGlowInit(1, 0))
    {
        printf("Glow init failed\n");
        while (1)
            swiWaitForVBlank();
    }

    // Warm at the bottom (the fire), cooler further up.
    NEA_PostFXGlowSetGradient(RGB15(6, 2, 0), RGB15(31, 16, 4));

    bool glow_on = true;
    bool gradient = true;
    bool flicker = true;
    int base_intensity = 7;
    int flash_timer = 0;
    int fade = 0;

    NEA_PostFXGlowEnable(true);

    while (1)
    {
        NEA_WaitForVBL(0);

        scanKeys();
        uint32_t keys = keysDown();
        uint32_t held = keysHeld();

        if (keys & KEY_A)
        {
            glow_on = !glow_on;
            NEA_PostFXGlowEnable(glow_on);
        }
        if (keys & KEY_B)
        {
            gradient = !gradient;
            if (gradient)
                NEA_PostFXGlowSetGradient(RGB15(6, 2, 0), RGB15(31, 16, 4));
            else
                NEA_PostFXGlowSetColor(RGB15(31, 16, 4));
        }
        if (keys & KEY_X)
            flicker = !flicker;

        if (held & KEY_UP)
            base_intensity++;
        if (held & KEY_DOWN)
            base_intensity--;
        if (base_intensity < 0)
            base_intensity = 0;
        if (base_intensity > 16)
            base_intensity = 16;

        // Lightning: steals the blend unit from the glow for a few frames.
        if (keys & KEY_Y)
            flash_timer = 8;

        // Fade to black, also on the blend unit.
        if (held & KEY_R)
            fade++;
        if (held & KEY_L)
            fade--;
        if (fade < 0)
            fade = 0;
        if (fade > 16)
            fade = 16;

        if (flash_timer > 0)
        {
            NEA_PostFXFlashSet(NEA_POSTFX_FLASH_WHITE, flash_timer * 2);
            flash_timer--;

            // Hand the unit back to the glow on the last frame.
            if (flash_timer == 0 && glow_on)
                NEA_PostFXGlowEnable(true);
        }
        else if (fade > 0)
        {
            NEA_PostFXFlashSet(NEA_POSTFX_FLASH_BLACK, fade);
        }
        else if (glow_on)
        {
            // Re-take the unit if the fade just ended.
            if (NEA_PostFXBlendOwner_Get() != NEA_POSTFX_BLEND_GLOW)
                NEA_PostFXGlowEnable(true);

            int i = base_intensity;
            if (flicker)
                i += (flicker_next() & 3) - 1;
            NEA_PostFXGlowSetIntensity(i);
        }

        static const char *OWNERS[] = { "none ", "glow ", "flash" };

        printf("\x1b[0;0H"
               "PostFX glow / flash\n"
               "===================\n\n"
               "A  Glow:      %s\n"
               "B  Mode:      %s\n"
               "X  Flicker:   %s\n"
               "Up/Down Int:  %2d\n"
               "Y  Lightning flash\n"
               "L/R Fade:     %2d\n\n"
               "Blend unit owner: %s\n\n"
               "Glow costs 18 KB of the\n"
               "main BG bank + 1 layer.\n"
               "Flash costs nothing.\n",
               glow_on ? "on " : "off",
               gradient ? "gradient" : "flat    ",
               flicker ? "on " : "off",
               base_intensity, fade,
               OWNERS[NEA_PostFXBlendOwner_Get()]);

        NEA_ProcessArg(Draw3DScene, &Scene);
    }

    return 0;
}
