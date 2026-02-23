// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Antonio Niño Díaz, 2008-2024
//
// This file is part of Nitro Engine Advanced

#include <NEAMain.h>

#include "teapot_bin.h"
#include "teapot.h"

typedef struct {
    NEA_Camera *Camera;
    NEA_Model *Model;

    int shading, alpha, id;
} SceneData;

void Draw3DScene(void *arg)
{
    SceneData *Scene = arg;

    // Set camera
    NEA_CameraUse(Scene->Camera);

    // Set polygon format
    NEA_PolyFormat(Scene->alpha, Scene->id, NEA_LIGHT_0, NEA_CULL_BACK,
                  Scene->shading);

    // Draw model
    NEA_ModelDraw(Scene->Model);
}

int main(int argc, char *argv[])
{
    SceneData Scene = { 0 };

    // This is needed for special screen effects
    irqEnable(IRQ_HBLANK);
    irqSet(IRQ_VBLANK, NEA_VBLFunc);
    irqSet(IRQ_VBLANK, NEA_HBLFunc);

    // Init console and Nitro Engine Advanced
    NEA_Init3D();
    // Use banks A and B for teapots. libnds uses bank C for the demo text
    // console.
    NEA_TextureSystemReset(0, 0, NEA_VRAM_AB);
    // This is needed to print text
    consoleDemoInit();

    // Allocate the objects we will use
    Scene.Model = NEA_ModelCreate(NEA_Static);
    Scene.Camera = NEA_CameraCreate();
    NEA_Material *Material = NEA_MaterialCreate();

    // Set camera coordinates
    NEA_CameraSet(Scene.Camera,
                 0, 0, -3,
                 0, 0, 0,
                 0, 1, 0);

    // Load mesh from RAM and assign it to a model
    NEA_ModelLoadStaticMesh(Scene.Model, teapot_bin);
    // Load teapot texture from RAM and assign it to a material
    NEA_MaterialTexLoad(Material, NEA_A1RGB5, 256, 256,
                       NEA_TEXGEN_TEXCOORD | NEA_TEXTURE_WRAP_S | NEA_TEXTURE_WRAP_T,
                       teapotBitmap);

    // Assign material to the model
    NEA_ModelSetMaterial(Scene.Model, Material);

    // Set some properties to the material
    NEA_MaterialSetProperties(Material,
                  RGB15(24, 24, 24), // Diffuse
                  RGB15(8, 8, 8),    // Ambient
                  RGB15(0, 0, 0),    // Specular
                  RGB15(0, 0, 0),    // Emission
                  false, false);     // Vertex color, use shininess table

    // Setup a light and its color
    NEA_LightSet(0, NEA_White, -0.5, -0.5, -0.5);

    // This enables shading (you can choose normal or toon).
    NEA_SetupToonShadingTables(true);
    // This enables outlining in all polygons, so be careful
    NEA_OutliningEnable(true);

    // We set the second outlining color to red.
    // This will be used by polygons with ID 8 - 15.
    NEA_OutliningSetColor(1, NEA_Red);

    while (1)
    {
        NEA_WaitForVBL(0);

        // Refresh keys
        scanKeys();
        uint32_t keys = keysHeld();

        printf("\x1b[0;0H"
               "Pad: Rotate.\nA: Toon shading.\n"
               "B: Change alpha value.\nY: Wireframe mode (alpha = 0)\n"
               "X: Outlining.");

        // Rotate model using the pad
        if (keys & KEY_UP)
            NEA_ModelRotate(Scene.Model, 0, 0, 2);
        if (keys & KEY_DOWN)
            NEA_ModelRotate(Scene.Model, 0, 0, -2);
        if (keys & KEY_RIGHT)
            NEA_ModelRotate(Scene.Model, 0, 2, 0);
        if (keys & KEY_LEFT)
            NEA_ModelRotate(Scene.Model, 0, -2, 0);

        // Change shading type
        if (keys & KEY_A)
            Scene.shading = NEA_TOON_HIGHLIGHT_SHADING;
        else
            Scene.shading = NEA_MODULATION;

        if (keys & KEY_B)
            Scene.alpha = 15; // Transparent
        else if (keys & KEY_Y)
            Scene.alpha = 0;  // Wireframe
        else
            Scene.alpha = 31; // Opaque

        // Change polygon ID to change outlining color
        if (keys & KEY_X)
            Scene.id = 8;
        else
            Scene.id = 0;

        // Draw scene
        NEA_ProcessArg(Draw3DScene, &Scene);
    }

    return 0;
}
