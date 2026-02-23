// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Antonio Niño Díaz, 2008-2024
//
// This file is part of Nitro Engine Advanced

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

    // Set camera
    NEA_CameraUse(Scene->Camera);

    // This has to be used to use fog
    NEA_PolyFormat(31, 0, NEA_LIGHT_ALL, NEA_CULL_BACK, NEA_FOG_ENABLE);

    // Draw models
    NEA_ModelDraw(Scene->Model);
    NEA_ModelDraw(Scene->Model2);
    NEA_ModelDraw(Scene->Model3);
}

int main(int argc, char *argv[])
{
    SceneData Scene = { 0 };

    irqEnable(IRQ_HBLANK);
    irqSet(IRQ_VBLANK, NEA_VBLFunc);
    irqSet(IRQ_VBLANK, NEA_HBLFunc);

    // Init console and Nitro Engine Advanced
    NEA_Init3D();
    // Use banks A and B for textures. libnds uses bank C for the demo text
    // console.
    NEA_TextureSystemReset(0, 0, NEA_VRAM_AB);
    consoleDemoInit();

    // Allocate objects
    Scene.Model = NEA_ModelCreate(NEA_Static);
    Scene.Model2 = NEA_ModelCreate(NEA_Static);
    Scene.Model3 = NEA_ModelCreate(NEA_Static);
    Scene.Camera = NEA_CameraCreate();
    NEA_Material *Material = NEA_MaterialCreate();

    // Set camera coordinates
    NEA_CameraSet(Scene.Camera,
                 -1, 2, -1,
                  1, 1, 1,
                  0, 1, 0);

    // Load models
    NEA_ModelLoadStaticMesh(Scene.Model, sphere_bin);
    NEA_ModelLoadStaticMesh(Scene.Model2, sphere_bin);
    NEA_ModelLoadStaticMesh(Scene.Model3, sphere_bin);

    // Load texture
    NEA_MaterialTexLoad(Material, NEA_A1RGB5, 256, 256, NEA_TEXGEN_TEXCOORD,
                       textureBitmap);

    // Assign the same material to every model object.
    NEA_ModelSetMaterial(Scene.Model, Material);
    NEA_ModelSetMaterial(Scene.Model2, Material);
    NEA_ModelSetMaterial(Scene.Model3, Material);

    // Set light and vector of light 0
    NEA_LightSet(0, NEA_White, 0, -1, -1);

    // Set position of every object
    NEA_ModelSetCoord(Scene.Model, 1, 0, 1);
    NEA_ModelSetCoord(Scene.Model2, 3, 1, 3);
    NEA_ModelSetCoord(Scene.Model3, 7, 2, 7);

    // Set initial fog color to black
    u32 color = NEA_Black;

    // Some parameters
    u16 depth = 0x7C00;
    u8 shift = 5;
    u8 mass = 2;

    while (1)
    {
        NEA_WaitForVBL(0);

        // Refresh keys
        scanKeys();
        uint32_t keys = keysDown();

        // Modify parameters
        if (keys & KEY_UP)
            shift ++;
        if (keys & KEY_DOWN)
            shift --;
        if (keys & KEY_X)
            mass ++;
        if(keys & KEY_B)
            mass --;
        if (keysHeld() & KEY_R)
            depth += 0x20;
        if(keysHeld() & KEY_L)
            depth -= 0x20;

        // Wrap values of parameters
        shift &= 0xF;
        mass &= 7;
        depth = (depth & 0x0FFF) + 0x7000;

        // Set fog color
        if (keys & KEY_START)
            color = NEA_Black;
        if (keys & KEY_SELECT)
            color = NEA_White;

        // Enable/update fog
        NEA_FogEnable(shift, color, 31, mass, depth);

        printf("\x1b[0;0H"
               "Up/Down - Shift: %d \nX/B - Mass: %d  \n"
               "L/R - Depth: 0x%x   \nSelect/Start - Change color.",
               shift, mass, depth);

        // Draw scene
        NEA_ProcessArg(Draw3DScene, &Scene);
    }

    return 0;
}
