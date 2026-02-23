// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Antonio Niño Díaz, 2008-2024
//
// This file is part of Nitro Engine Advanced

#include <NEAMain.h>

// First you have to put the .bin files in the data folder. This will generate
// (after doing "make") some files named "binfilename_bin.h". For example,
// "model.bin" will generate a file named "model_bin.h". You have to include
// this in "main.c".
//
// The name you will have to use is "binfilename_bin". For example, for loading
// "model.bin" you will have to use:
//
//     NEA_ModelLoadStaticMesh(Model, binfilename_bin);
//
#include "teapot_bin.h"
#include "teapot.h"

#include "cube_bin.h"
#include "spiral_blue_pal32.h"

typedef struct {
    NEA_Camera *Camera;
    NEA_Model *Model[2];
    NEA_Material *Material[2];
} SceneData;

void Draw3DScene(void *arg)
{
    SceneData *Scene = arg;

    NEA_CameraUse(Scene->Camera);

    NEA_PolyFormat(31, 0, NEA_LIGHT_0, NEA_CULL_NONE, 0);

    NEA_ModelDraw(Scene->Model[0]);
    NEA_ModelDraw(Scene->Model[1]);
}

int main(int argc, char *argv[])
{
    SceneData Scene = { 0 };

    irqEnable(IRQ_HBLANK);
    irqSet(IRQ_VBLANK, NEA_VBLFunc);
    irqSet(IRQ_HBLANK, NEA_HBLFunc);

    // Init Nitro Engine Advanced in normal 3D mode
    NEA_Init3D();

    // libnds uses VRAM_C for the text console, reserve A and B only
    NEA_TextureSystemReset(0, 0, NEA_VRAM_AB);
    // Init console in non-3D screen
    consoleDemoInit();

    // Allocate space for the objects we'll use
    Scene.Model[0] = NEA_ModelCreate(NEA_Static);
    Scene.Model[1] = NEA_ModelCreate(NEA_Static);

    Scene.Camera = NEA_CameraCreate();

    Scene.Material[0] = NEA_MaterialCreate();
    Scene.Material[1] = NEA_MaterialCreate();

    NEA_Palette *Palette = NEA_PaletteCreate();

    // Set coordinates for the camera
    NEA_CameraSet(Scene.Camera,
                 0, 0, 3,  // Position
                 0, 0, 0,  // Look at
                 0, 1, 0); // Up direction

    // Load mesh from RAM and assign it to the object "Model".
    NEA_ModelLoadStaticMesh(Scene.Model[0], teapot_bin);
    NEA_ModelLoadStaticMesh(Scene.Model[1], cube_bin);



    // Load a RGB texture from RAM and assign it to "Material".
    NEA_MaterialTexLoad(Scene.Material[0], NEA_A1RGB5, 256, 256, NEA_TEXGEN_TEXCOORD | NEA_TEXTURE_WRAP_S | NEA_TEXTURE_WRAP_T,
                       teapotBitmap);

    NEA_MaterialTexLoad(Scene.Material[1], NEA_A3PAL32, 64, 64, NEA_TEXGEN_TEXCOORD | NEA_TEXTURE_WRAP_S | NEA_TEXTURE_WRAP_T,
                       spiral_blue_pal32Bitmap);

    NEA_PaletteLoad(Palette, spiral_blue_pal32Pal, 32, NEA_A3PAL32);

    NEA_MaterialSetPalette(Scene.Material[1], Palette);

    // Assign texture to model...
    NEA_ModelSetMaterial(Scene.Model[0], Scene.Material[0]);
    NEA_ModelSetMaterial(Scene.Model[1], Scene.Material[1]);

    NEA_ModelSetCoord(Scene.Model[0], -1.25, 0, 0);
    NEA_ModelSetCoord(Scene.Model[1], 1.5, 0, 0);
    NEA_ModelSetRot(Scene.Model[1], 10, 2, 0);
    
    // We set up a light and its color
    NEA_LightSet(0, NEA_White, -0.5, -0.5, -0.5);

    int model_selected = 0;

    while (1)
    {
        // Wait for next frame
        NEA_WaitForVBL(0);

        // Get keys information
        scanKeys();
        uint32_t keys = keysHeld();

        printf("\x1b[0;0HPad: Rotate.   \n\nA/B/X/Y: Move texture.   \n\nL/R: Scale texture.   \n\nSTART: Reset transformations of current texture.   \n\nSELECT: Switch texture.");

        // Rotate model using the pad
        if (keys & KEY_UP)
            NEA_ModelRotate(Scene.Model[model_selected], 0, 0, -2);
        if (keys & KEY_DOWN)
            NEA_ModelRotate(Scene.Model[model_selected], 0, 0, 2);
        if (keys & KEY_RIGHT)
            NEA_ModelRotate(Scene.Model[model_selected], 0, 2, 0);
        if (keys & KEY_LEFT)
            NEA_ModelRotate(Scene.Model[model_selected], 0, -2, 0);

        
        
        // Translate texture
        if (keys & KEY_A)
            NEA_TextureTranslate(Scene.Material[model_selected], 0, 1 << 6);

        if (keys & KEY_Y)
            NEA_TextureTranslate(Scene.Material[model_selected], 0, -1 << 6);

        if (keys & KEY_B)
            NEA_TextureTranslate(Scene.Material[model_selected], 1 << 6, 0);

        if (keys & KEY_X)
            NEA_TextureTranslate(Scene.Material[model_selected], -1 << 6, 0);

        // Scale texture
        if (keys & KEY_L)
            NEA_TextureScale(Scene.Material[model_selected], 0.125, 0.125);

        if (keys & KEY_R)
            NEA_TextureScale(Scene.Material[model_selected], -0.125, -0.125);

        if (keys & KEY_START)
            NEA_TextureResetTransformations(Scene.Material[model_selected]);
        
        if (keys & KEY_SELECT)
        {
            if (model_selected == 0) model_selected = 1;
            if (model_selected == 0) model_selected = 0;
        }
        // Draw scene
        NEA_ProcessArg(Draw3DScene, &Scene);
    }

    return 0;
}
