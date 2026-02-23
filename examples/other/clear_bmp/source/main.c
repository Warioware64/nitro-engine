// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Antonio Niño Díaz, 2008-2024
//
// This file is part of Nitro Engine Advanced

// The depth bitmap is a bit complicated... It is quite difficult to regulate
// the depth so that the image and the result make sense.
//
// With img2ds, the color you have to use to set the depth is blue. The clear
// bitmap is in its farthest point if blue is 255.
//
// If you want to hide everything, including the 2D projection, use red (any
// value overrides the value in the blue channel).
//
// Check assets.sh to see how to convert the images.

#include <NEAMain.h>

#include "background.h"
#include "depth_tex_bin.h"
#include "cube_bin.h"

typedef struct {
    NEA_Camera *Camera;
    NEA_Model *Model;
} SceneData;

void Draw3DScene(void *arg)
{
    SceneData *Scene = arg;

    NEA_CameraUse(Scene->Camera);

    NEA_PolyFormat(31, 0, NEA_LIGHT_01, NEA_CULL_BACK, 0);
    NEA_ModelDraw(Scene->Model);

    NEA_2DViewInit();
    NEA_2DDrawQuad(0, 0,
                  100, 100,
                  0, NEA_White);
}

int main(int argc, char *argv[])
{
    SceneData Scene = { 0 };

    irqEnable(IRQ_HBLANK);
    irqSet(IRQ_VBLANK, NEA_VBLFunc);
    irqSet(IRQ_HBLANK, NEA_HBLFunc);

    NEA_Init3D();

    // The clear bitmap is placed in VRAM_C and VRAM_D, so it is needed to
    // preserve them.
    NEA_TextureSystemReset(0, 0, NEA_VRAM_AB);

    // Copy data into VRAM
    vramSetBankC(VRAM_C_LCD);
    dmaCopy(backgroundBitmap, VRAM_C, backgroundBitmapLen * 2);
    vramSetBankD(VRAM_D_LCD);
    dmaCopy(depth_tex_bin, VRAM_D, depth_tex_bin_size);

    NEA_ClearBMPEnable(true);

    Scene.Camera = NEA_CameraCreate();
    NEA_CameraSet(Scene.Camera,
                 1, 1, 1,
                 0, 0, 0,
                 0, 1, 0);

    Scene.Model = NEA_ModelCreate(NEA_Static);
    NEA_ModelLoadStaticMesh(Scene.Model, cube_bin);

    NEA_LightSet(0, NEA_Yellow, -1, -1, 0);
    NEA_LightSet(1, NEA_Red, -1, 1, 0);

    NEA_ClearColorSet(0,       // Color not used when clear BMP
                     31, 63); // ID and alpha are used

    int scrollx = 0, scrolly = 0;

    while (1)
    {
        NEA_WaitForVBL(0);

        scanKeys();
        uint32_t keys = keysHeld();

        NEA_ModelRotate(Scene.Model, 0, 2, 1);

        if (keys & KEY_A)
            NEA_ClearBMPEnable(true);
        if (keys & KEY_B)
            NEA_ClearBMPEnable(false);

        NEA_ClearBMPScroll(scrollx, scrolly);

        if (keys & KEY_UP)
            scrolly--;
        if (keys & KEY_DOWN)
            scrolly++;
        if (keys & KEY_RIGHT)
            scrollx++;
        if (keys & KEY_LEFT)
            scrollx--;

        NEA_ProcessArg(Draw3DScene, &Scene);
    }

    return 0;
}
