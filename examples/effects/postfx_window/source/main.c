// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Nitro Engine Advanced contributors, 2026
//
// This file is part of Nitro Engine Advanced

// PPU-native windowing and mosaic (NEAPostFX).
//
// Vignette / flashlight (NEA_PostFXVignetteEnable): window 0 marks the lit
// rectangle, a brightness-decrease fade is enabled globally, and the window's
// color-effect bit switches that fade *off* inside the rectangle. The result
// costs no BG layer and no VRAM -- only the window and blend registers.
//
// Mosaic (NEA_PostFXMosaicSet): pixelates 2D layers. It deliberately does
// nothing to the 3D scene, because the hardware cannot do that: GBATEK says
// "mosaic cannot be used on the 3D layer". This example makes that concrete by
// mosaicking a 2D overlay layer while the 3D spheres stay sharp.
//
// Cost: vignette and windows are free (register writes only). Mosaic is free.
// The 2D overlay used to demonstrate mosaic costs one BG layer and 18 KB.

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

int main(int argc, char *argv[])
{
    SceneData Scene = { 0 };

    irqEnable(IRQ_HBLANK);
    irqSet(IRQ_VBLANK, NEA_VBLFunc);
    irqSet(IRQ_HBLANK, NEA_HBLFunc);

    NEA_SetTexPaletteBank(NEA_VRAM_F);

    NEA_Init3D();
    NEA_TextureSystemReset(0, 0, NEA_VRAM_AB);
    consoleDemoInit();

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

    NEA_CameraSet(Scene.Camera, -1, 2, -1, 1, 1, 1, 0, 1, 0);

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

    // A 2D overlay on layer 1, used purely to show that mosaic affects 2D
    // layers and not the 3D scene. It is a checkerboard of palette indices so
    // the pixelation is obvious.
    NEA_Hw2DBG *overlay = NEA_Hw2DBGCreate(NEA_ENGINE_MAIN, 1,
                                          NEA_HW2D_BG_TILED_4BPP, 256, 256);
    if (overlay != NULL)
    {
        u16 *gfx = overlay->gfx_ptr;
        // Tile 0 transparent, tile 1 a fine 2x2 checker of indices 1 and 2.
        for (int w = 0; w < 16; w++)
            gfx[w] = 0;
        for (int w = 0; w < 16; w++)
            gfx[16 + w] = (w & 1) ? 0x1212 : 0x2121;

        u16 *map = overlay->map_ptr;
        for (int row = 0; row < 32; row++)
            for (int col = 0; col < 32; col++)
                map[row * 32 + col] = ((row + col) & 1) ? 1 : 0;

        static u16 pal[16];
        pal[0] = 0;
        pal[1] = RGB15(0, 31, 10);
        pal[2] = RGB15(0, 12, 31);
        NEA_Hw2DBGLoadPalette(overlay, pal, 16, 1);
        for (int row = 0; row < 32; row++)
            for (int col = 0; col < 32; col++)
                map[row * 32 + col] |= (1 << 12);

        // The overlay has to be in front of the 3D layer to be visible at all:
        // the 3D rear plane is opaque, so anything behind BG0 is covered.
        NEA_Hw2DBGSetPriority(overlay, 0);
        REG_BG0CNT = (REG_BG0CNT & ~BG_PRIORITY(3)) | BG_PRIORITY(1);
        NEA_Hw2DBGSetVisible(overlay, false);
    }

    bool vignette = true;
    bool overlay_on = false;
    bool mosaic_on = false;
    int strength = 12;
    int radius = 64;
    int mos = 3;

    NEA_PostFXVignetteEnable(true, strength);

    while (1)
    {
        NEA_WaitForVBL(0);

        scanKeys();
        uint32_t keys = keysDown();
        uint32_t held = keysHeld();

        if (keys & KEY_A)
        {
            vignette = !vignette;
            NEA_PostFXVignetteEnable(vignette, strength);
        }
        if (keys & KEY_B)
        {
            overlay_on = !overlay_on;
            if (overlay != NULL)
                NEA_Hw2DBGSetVisible(overlay, overlay_on);
        }
        if (keys & KEY_X)
        {
            mosaic_on = !mosaic_on;
            // Deliberately ask for the 3D layer too: it prints a debug note and
            // does nothing, which is the point being demonstrated.
            NEA_PostFXMosaicLayerEnable(1, mosaic_on);
            NEA_PostFXMosaicSet(mosaic_on ? mos : 0, mosaic_on ? mos : 0, 0, 0);
        }

        if (held & KEY_UP) strength++;
        if (held & KEY_DOWN) strength--;
        if (strength < 0) strength = 0;
        if (strength > 16) strength = 16;

        if (held & KEY_RIGHT) radius += 2;
        if (held & KEY_LEFT) radius -= 2;
        if (radius < 8) radius = 8;
        if (radius > 128) radius = 128;

        if (vignette)
        {
            NEA_PostFXVignetteEnable(true, strength);
            // Centred lit rectangle. A round cone needs the window edges
            // rewritten per scanline, which the HBlank system will add.
            NEA_PostFXWindowSetRect(NEA_POSTFX_WIN0,
                                   128 - radius, 96 - (radius * 3) / 4,
                                   128 + radius - 1, 96 + (radius * 3) / 4 - 1);
        }

        if (mosaic_on)
        {
            if (held & KEY_R) mos++;
            if (held & KEY_L) mos--;
            if (mos < 0) mos = 0;
            if (mos > 15) mos = 15;
            NEA_PostFXMosaicSet(mos, mos, 0, 0);
        }

        printf("\x1b[0;0H"
               "PostFX window / mosaic\n"
               "======================\n\n"
               "A  Vignette:  %s\n"
               "   Up/Down strength: %2d\n"
               "   Left/Right size:  %3d\n"
               "B  2D overlay: %s\n"
               "X  Mosaic:     %s\n"
               "   L/R size:   %2d\n\n"
               "Mosaic pixelates the 2D\n"
               "overlay only. The 3D layer\n"
               "cannot be mosaicked at all\n"
               "(GBATEK: no effect on BG0).\n\n"
               "Vignette costs 0 BG layers\n"
               "and 0 VRAM.\n",
               vignette ? "on " : "off", strength, radius,
               overlay_on ? "on " : "off",
               mosaic_on ? "on " : "off", mos);

        NEA_ProcessArg(Draw3DScene, &Scene);
    }

    return 0;
}
