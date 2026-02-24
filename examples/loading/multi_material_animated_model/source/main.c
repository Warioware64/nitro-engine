// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Nitro Engine Advanced Contributors, 2024
//
// This file is part of Nitro Engine Advanced

#include <NEAMain.h>

#include "robot_dsm_bin.h"
#include "robot_wave_dsa_bin.h"
#include "texture.h"
#include "texture1.h"

typedef struct {
    NEA_Camera *Camera;
    NEA_Model *Model;
} SceneData;

void Draw3DScene(void *arg)
{
    SceneData *Scene = arg;

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

    // Create camera
    Scene.Camera = NEA_CameraCreate();
    NEA_CameraSet(Scene.Camera,
                 6, 3, -4,
                 0, 3, 0,
                 0, 1, 0);

    // Create model
    Scene.Model = NEA_ModelCreate(NEA_Animated);
    NEA_Animation *Animation = NEA_AnimationCreate();

    NEA_AnimationLoad(Animation, robot_wave_dsa_bin);

    // ------ Load multi-material animated model (DSM format) -------
    NEA_ModelLoadMultiMesh(Scene.Model, robot_dsm_bin);
    NEA_ModelSetAnimation(Scene.Model, Animation);

    NEA_ModelAnimStart(Scene.Model, NEA_ANIM_LOOP, floattof32(0.1));

    // ---- Set up "teapot" material ----
    NEA_Material *Mat0 = NEA_MaterialCreate();
    NEA_MaterialSetName(Mat0, "teapot");
    NEA_MaterialTexLoad(Mat0, NEA_A1RGB5, 256, 256,
                       NEA_TEXGEN_TEXCOORD | NEA_TEXTURE_WRAP_S | NEA_TEXTURE_WRAP_T, textureBitmap);


    // ---- Set up "a3pal32" material ----
    NEA_Material *Mat1 = NEA_MaterialCreate();
    NEA_MaterialSetName(Mat1, "a3pal32");
    NEA_MaterialTexLoad(Mat1, NEA_A3PAL32, 256, 256,
                       NEA_TEXGEN_TEXCOORD | NEA_TEXTURE_WRAP_S | NEA_TEXTURE_WRAP_T, texture1Bitmap);
    
    NEA_Palette *PalMat1 = NEA_PaletteCreate();
    NEA_PaletteLoadSize(PalMat1, texture1Pal, texture1PalLen, NEA_A3PAL32);
    NEA_MaterialSetPalette(Mat1, PalMat1);


    // ---- Auto-bind animated mesh ----
    NEA_ModelAutoBindMaterials(Scene.Model);

    // Set up a light
    NEA_LightSet(0, NEA_White, -0.5, -0.5, -0.5);

    while (1)
    {
        NEA_WaitForVBL(NEA_UPDATE_ANIMATIONS);

        scanKeys();
        uint32_t keys = keysHeld();

        printf("\x1b[0;0HPad: Rotate.");

        if (keys & KEY_UP)
            NEA_ModelRotate(Scene.Model, 0, 0, -2);
        if (keys & KEY_DOWN)
            NEA_ModelRotate(Scene.Model, 0, 0, 2);
        if (keys & KEY_RIGHT)
            NEA_ModelRotate(Scene.Model, 0, 2, 0);
        if (keys & KEY_LEFT)
            NEA_ModelRotate(Scene.Model, 0, -2, 0);

        NEA_ProcessArg(Draw3DScene, &Scene);
    }

    return 0;
}
