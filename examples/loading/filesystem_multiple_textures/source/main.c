// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Antonio Niño Díaz, 2008-2024
//
// This file is part of Nitro Engine Advanced

#include <stdbool.h>

#include <filesystem.h>

#include <NEAMain.h>

typedef struct {
    NEA_Camera *Camera;
    NEA_Model *Model;
} SceneData;

void Draw3DScene(void *arg)
{
    SceneData *Scene = arg;

    NEA_CameraUse(Scene->Camera);

    NEA_PolyFormat(31, 0, NEA_LIGHT_0, NEA_CULL_NONE, 0);
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
    // Init console in non-3D screen
    consoleDemoInit();

    if (!nitroFSInit(NULL))
    {
        printf("nitroFSInit failed.\nPress START to exit");
        while (1)
        {
            swiWaitForVBlank();
            scanKeys();
            if (keysHeld() & KEY_START)
                return 0;
        }
    }

    // Allocate space for objects...
    Scene.Model = NEA_ModelCreate(NEA_Static);
    Scene.Camera = NEA_CameraCreate();
    NEA_Material *MaterialBlue = NEA_MaterialCreate();
    NEA_Material *MaterialRed = NEA_MaterialCreate();
    NEA_Palette *PaletteBlue = NEA_PaletteCreate();
    NEA_Palette *PaletteRed = NEA_PaletteCreate();

    // Setup camera
    NEA_CameraSet(Scene.Camera,
                 -1, -1, -1,
                  0, 0, 0,
                  0, 1, 0);

    // Load things from FAT
    NEA_ModelLoadStaticMeshFAT(Scene.Model, "cube.bin");

    NEA_MaterialTexLoadFAT(MaterialBlue, NEA_A3PAL32, 64, 64, NEA_TEXGEN_TEXCOORD,
                          "spiral_blue_pal32.img.bin");
    NEA_MaterialTexLoadFAT(MaterialRed, NEA_A3PAL32, 64, 64, NEA_TEXGEN_TEXCOORD,
                          "spiral_red_pal32.img.bin");

    NEA_PaletteLoadFAT(PaletteBlue, "spiral_blue_pal32.pal.bin", NEA_A3PAL32);
    NEA_PaletteLoadFAT(PaletteRed, "spiral_red_pal32.pal.bin", NEA_A3PAL32);

    NEA_MaterialSetPalette(MaterialBlue, PaletteBlue);
    NEA_MaterialSetPalette(MaterialRed, PaletteRed);

    // Assign material to model
    NEA_ModelSetMaterial(Scene.Model, MaterialBlue);

    // Set up light
    NEA_LightSet(0, NEA_White, 0, -1, -1);

    // Background color
    NEA_ClearColorSet(NEA_Gray, 31, 63);

    while (1)
    {
        NEA_WaitForVBL(0);

        scanKeys(); //Get keys information...
        uint32_t keys = keysDown();

        if (keys & KEY_START)
            return 0;

        // Change material if pressed
        if (keys & KEY_B)
            NEA_ModelSetMaterial(Scene.Model, MaterialBlue);
        if (keys & KEY_A)
            NEA_ModelSetMaterial(Scene.Model, MaterialRed);

        printf("\x1b[0;0HA/B: Change material.\n\nSTART: Exit.");

        // Increase rotation, you can't get the rotation angle after
        // this. If you want to know always the angle, you should use
        // NEA_ModelSetRot().
        NEA_ModelRotate(Scene.Model, 1, 2, 0);

        NEA_ProcessArg(Draw3DScene, &Scene);
    }

    return 0;
}
