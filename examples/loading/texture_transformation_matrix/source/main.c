// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Antonio Niño Díaz, 2008-2024
//
// This file is part of Nitro Engine

#include <NEMain.h>

// First you have to put the .bin files in the data folder. This will generate
// (after doing "make") some files named "binfilename_bin.h". For example,
// "model.bin" will generate a file named "model_bin.h". You have to include
// this in "main.c".
//
// The name you will have to use is "binfilename_bin". For example, for loading
// "model.bin" you will have to use:
//
//     NE_ModelLoadStaticMesh(Model, binfilename_bin);
//
#include "teapot_bin.h"
#include "teapot.h"

#include "cube_bin.h"
#include "spiral_blue_pal32.h"

typedef struct {
    NE_Camera *Camera;
    NE_Model *Model[2];
    NE_Material *Material[2];
} SceneData;

void Draw3DScene(void *arg)
{
    SceneData *Scene = arg;

    NE_CameraUse(Scene->Camera);

    NE_PolyFormat(31, 0, NE_LIGHT_0, NE_CULL_NONE, 0);

    NE_ModelDraw(Scene->Model[0]);
    NE_ModelDraw(Scene->Model[1]);
}

int main(int argc, char *argv[])
{
    SceneData Scene = { 0 };

    irqEnable(IRQ_HBLANK);
    irqSet(IRQ_VBLANK, NE_VBLFunc);
    irqSet(IRQ_HBLANK, NE_HBLFunc);

    // Init Nitro Engine in normal 3D mode
    NE_Init3D();

    // libnds uses VRAM_C for the text console, reserve A and B only
    NE_TextureSystemReset(0, 0, NE_VRAM_AB);
    // Init console in non-3D screen
    consoleDemoInit();

    // Allocate space for the objects we'll use
    Scene.Model[0] = NE_ModelCreate(NE_Static);
    Scene.Model[1] = NE_ModelCreate(NE_Static);

    Scene.Camera = NE_CameraCreate();

    Scene.Material[0] = NE_MaterialCreate();
    Scene.Material[1] = NE_MaterialCreate();

    NE_Palette *Palette = NE_PaletteCreate();

    // Set coordinates for the camera
    NE_CameraSet(Scene.Camera,
                 0, 0, 3,  // Position
                 0, 0, 0,  // Look at
                 0, 1, 0); // Up direction

    // Load mesh from RAM and assign it to the object "Model".
    NE_ModelLoadStaticMesh(Scene.Model[0], teapot_bin);
    NE_ModelLoadStaticMesh(Scene.Model[1], cube_bin);



    // Load a RGB texture from RAM and assign it to "Material".
    NE_MaterialTexLoad(Scene.Material[0], NE_A1RGB5, 256, 256, NE_TEXGEN_TEXCOORD | NE_TEXTURE_WRAP_S | NE_TEXTURE_WRAP_T,
                       teapotBitmap);

    NE_MaterialTexLoad(Scene.Material[1], NE_A3PAL32, 64, 64, NE_TEXGEN_TEXCOORD | NE_TEXTURE_WRAP_S | NE_TEXTURE_WRAP_T,
                       spiral_blue_pal32Bitmap);

    NE_PaletteLoad(Palette, spiral_blue_pal32Pal, 32, NE_A3PAL32);

    NE_MaterialSetPalette(Scene.Material[1], Palette);

    // Assign texture to model...
    NE_ModelSetMaterial(Scene.Model[0], Scene.Material[0]);
    NE_ModelSetMaterial(Scene.Model[1], Scene.Material[1]);

    NE_ModelSetCoord(Scene.Model[0], -1.25, 0, 0);
    NE_ModelSetCoord(Scene.Model[1], 1.5, 0, 0);
    NE_ModelSetRot(Scene.Model[1], 10, 2, 0);
    
    // We set up a light and its color
    NE_LightSet(0, NE_White, -0.5, -0.5, -0.5);

    int model_selected = 0;

    while (1)
    {
        // Wait for next frame
        NE_WaitForVBL(0);

        // Get keys information
        scanKeys();
        uint32_t keys = keysHeld();

        printf("\x1b[0;0HPad: Rotate.   \n\nA/B/X/Y: Move texture.   \n\nL/R: Scale texture.   \n\nSTART: Reset transformations of current texture.   \n\nSELECT: Switch texture.");

        // Rotate model using the pad
        if (keys & KEY_UP)
            NE_ModelRotate(Scene.Model[model_selected], 0, 0, -2);
        if (keys & KEY_DOWN)
            NE_ModelRotate(Scene.Model[model_selected], 0, 0, 2);
        if (keys & KEY_RIGHT)
            NE_ModelRotate(Scene.Model[model_selected], 0, 2, 0);
        if (keys & KEY_LEFT)
            NE_ModelRotate(Scene.Model[model_selected], 0, -2, 0);

        
        
        // Translate texture
        if (keys & KEY_A)
            NE_TextureTranslate(Scene.Material[model_selected], 0, 1 << 6);

        if (keys & KEY_Y)
            NE_TextureTranslate(Scene.Material[model_selected], 0, -1 << 6);

        if (keys & KEY_B)
            NE_TextureTranslate(Scene.Material[model_selected], 1 << 6, 0);

        if (keys & KEY_X)
            NE_TextureTranslate(Scene.Material[model_selected], -1 << 6, 0);

        // Scale texture
        if (keys & KEY_L)
            NE_TextureScale(Scene.Material[model_selected], 0.125, 0.125);

        if (keys & KEY_R)
            NE_TextureScale(Scene.Material[model_selected], -0.125, -0.125);

        if (keys & KEY_START)
            NE_TextureResetTransformations(Scene.Material[model_selected]);
        
        if (keys & KEY_SELECT)
        {
            if (model_selected == 0) model_selected = 1;
            if (model_selected == 0) model_selected = 0;
        }
        // Draw scene
        NE_ProcessArg(Draw3DScene, &Scene);
    }

    return 0;
}
