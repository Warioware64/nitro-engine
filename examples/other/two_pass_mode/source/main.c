// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: 2024-2026
//
// This file is part of Nitro Engine Advanced
//
// Two-pass 3D rendering example. This mode doubles the polygon budget by
// splitting the screen into left and right halves, rendering each half in a
// separate hardware frame. The effective framerate is 30 FPS.
//
// This example renders multiple teapot models to demonstrate the increased
// polygon budget that two-pass mode provides. Press A/X/Y to switch between
// the three two-pass modes.

#include <NEAMain.h>

#include "teapot_bin.h"
#include "teapot.h"

#define NUM_MODELS 6

typedef struct {
    NEA_Camera *Camera;
    NEA_Model *Model[NUM_MODELS];
    NEA_Material *Material;
    int ram_poly_pass0;
    int ram_vertex_pass0;
    int ram_poly_pass1;
    int ram_vertex_pass1;
} SceneData;

void Draw3DScene(void *arg)
{
    SceneData *Scene = arg;

    NEA_CameraUse(Scene->Camera);

    for (int i = 0; i < NUM_MODELS; i++)
        NEA_ModelDraw(Scene->Model[i]);

    if (NEA_TwoPassGetPass() == 0)
    {
        Scene->ram_poly_pass0 = NEA_GetPolygonCount();
        Scene->ram_vertex_pass0 = NEA_GetVertexCount();
    }
    else
    {
        Scene->ram_poly_pass1 = NEA_GetPolygonCount();
        Scene->ram_vertex_pass1 = NEA_GetVertexCount();
    }
}

void init_all(SceneData *Scene)
{
    for (int i = 0; i < NUM_MODELS; i++)
        Scene->Model[i] = NEA_ModelCreate(NEA_Static);

    Scene->Camera = NEA_CameraCreate();
    Scene->Material = NEA_MaterialCreate();

    NEA_CameraSet(Scene->Camera,
                 0, 3, -8,  // Position
                 0, 0, 0,   // Look at
                 0, 1, 0);  // Up direction

    NEA_ModelLoadStaticMesh(Scene->Model[0], teapot_bin);

    NEA_MaterialTexLoad(Scene->Material, NEA_A1RGB5, 256, 256,
                       NEA_TEXGEN_TEXCOORD | NEA_TEXTURE_WRAP_S
                       | NEA_TEXTURE_WRAP_T,
                       teapotBitmap);

    // Set material BEFORE cloning so clones inherit the material reference
    NEA_ModelSetMaterial(Scene->Model[0], Scene->Material);

    // Clone model 0 to the rest (copies mesh and material references)
    for (int i = 1; i < NUM_MODELS; i++)
        NEA_ModelClone(Scene->Model[i], Scene->Model[0]);

    // Arrange models in a 2x3 grid
    int idx = 0;
    for (int row = -1; row <= 0; row++)
    {
        for (int col = -1; col <= 1; col++)
        {
            NEA_ModelSetCoord(Scene->Model[idx], col * 3, row * 3, 0);
            idx++;
        }
    }

    NEA_LightSet(0, NEA_White, -0.5, -0.5, -0.5);
}

int main(int argc, char *argv[])
{
    SceneData Scene = { 0 };

    irqEnable(IRQ_HBLANK);
    irqSet(IRQ_VBLANK, NEA_VBLFunc);
    irqSet(IRQ_HBLANK, NEA_HBLFunc);

    // Start in DMA mode (no line artifacts, 75% texture VRAM)
    NEA_Init3D_TwoPass_DMA();
    NEA_InitConsole();
    init_all(&Scene);

    while (1)
    {
        NEA_WaitForVBL(NEA_TwoPassGetPass() == 1 ? NEA_UPDATE_ANIMATIONS : 0);

        NEA_ProcessTwoPassArg(Draw3DScene, &Scene);

        printf("\x1b[0;0H"
               "Two-pass 3D mode demo\n"
               "\n"
               "A: DMA mode (recommended)\n"
               "X: FB mode (50%% tex VRAM)\n"
               "Y: FIFO mode (line artifacts)\n"
               "\n"
               "Pad: Rotate  START: Exit\n"
               "\n"
               "Polygon RAM (pass 0): %d   \n"
               "Vertex RAM (pass 0):  %d   \n"
               "\n"
               "Polygon RAM (pass 1): %d   \n"
               "Vertex RAM (pass 1):  %d   \n",
               Scene.ram_poly_pass0, Scene.ram_vertex_pass0,
               Scene.ram_poly_pass1, Scene.ram_vertex_pass1);

        scanKeys();
        uint32_t keys = keysHeld();
        uint32_t kdown = keysDown();

        if (kdown & KEY_START)
            break;

        // Switch two-pass modes
        if (kdown & KEY_A)
        {
            NEA_Init3D_TwoPass_DMA();
            NEA_InitConsole();
            init_all(&Scene);
        }
        if (kdown & KEY_X)
        {
            NEA_Init3D_TwoPass_FB();
            NEA_InitConsole();
            init_all(&Scene);
        }
        if (kdown & KEY_Y)
        {
            NEA_Init3D_TwoPass();
            NEA_InitConsole();
            init_all(&Scene);
        }

        // Update rotations only after both passes are done
        if (NEA_TwoPassGetPass() == 1)
        {
            if (keys & KEY_UP)
                for (int i = 0; i < NUM_MODELS; i++)
                    NEA_ModelRotate(Scene.Model[i], 2, 0, 0);
            if (keys & KEY_DOWN)
                for (int i = 0; i < NUM_MODELS; i++)
                    NEA_ModelRotate(Scene.Model[i], -2, 0, 0);
            if (keys & KEY_RIGHT)
                for (int i = 0; i < NUM_MODELS; i++)
                    NEA_ModelRotate(Scene.Model[i], 0, 4, 0);
            if (keys & KEY_LEFT)
                for (int i = 0; i < NUM_MODELS; i++)
                    NEA_ModelRotate(Scene.Model[i], 0, -4, 0);
        }
    }

    NEA_End();

    return 0;
}
