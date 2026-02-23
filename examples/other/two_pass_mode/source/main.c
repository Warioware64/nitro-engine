// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: 2024-2026
//
// This file is part of Nitro Engine
//
// Two-pass 3D rendering example. This mode doubles the polygon budget by
// splitting the screen into left and right halves, rendering each half in a
// separate hardware frame. The effective framerate is 30 FPS.
//
// This example renders multiple teapot models to demonstrate the increased
// polygon budget that two-pass mode provides. Press A/X/Y to switch between
// the three two-pass modes.

#include <NEMain.h>

#include "teapot_bin.h"
#include "teapot.h"

#define NUM_MODELS 6

typedef struct {
    NE_Camera *Camera;
    NE_Model *Model[NUM_MODELS];
    NE_Material *Material;
    int ram_poly_pass0;
    int ram_vertex_pass0;
    int ram_poly_pass1;
    int ram_vertex_pass1;
} SceneData;

void Draw3DScene(void *arg)
{
    SceneData *Scene = arg;

    NE_CameraUse(Scene->Camera);

    for (int i = 0; i < NUM_MODELS; i++)
        NE_ModelDraw(Scene->Model[i]);

    if (NE_TwoPassGetPass() == 0)
    {
        Scene->ram_poly_pass0 = NE_GetPolygonCount();
        Scene->ram_vertex_pass0 = NE_GetVertexCount();
    }
    else
    {
        Scene->ram_poly_pass1 = NE_GetPolygonCount();
        Scene->ram_vertex_pass1 = NE_GetVertexCount();
    }
}

void init_all(SceneData *Scene)
{
    for (int i = 0; i < NUM_MODELS; i++)
        Scene->Model[i] = NE_ModelCreate(NE_Static);

    Scene->Camera = NE_CameraCreate();
    Scene->Material = NE_MaterialCreate();

    NE_CameraSet(Scene->Camera,
                 0, 3, -8,  // Position
                 0, 0, 0,   // Look at
                 0, 1, 0);  // Up direction

    NE_ModelLoadStaticMesh(Scene->Model[0], teapot_bin);

    NE_MaterialTexLoad(Scene->Material, NE_A1RGB5, 256, 256,
                       NE_TEXGEN_TEXCOORD | NE_TEXTURE_WRAP_S
                       | NE_TEXTURE_WRAP_T,
                       teapotBitmap);

    // Set material BEFORE cloning so clones inherit the material reference
    NE_ModelSetMaterial(Scene->Model[0], Scene->Material);

    // Clone model 0 to the rest (copies mesh and material references)
    for (int i = 1; i < NUM_MODELS; i++)
        NE_ModelClone(Scene->Model[i], Scene->Model[0]);

    // Arrange models in a 2x3 grid
    int idx = 0;
    for (int row = -1; row <= 0; row++)
    {
        for (int col = -1; col <= 1; col++)
        {
            NE_ModelSetCoord(Scene->Model[idx], col * 3, row * 3, 0);
            idx++;
        }
    }

    NE_LightSet(0, NE_White, -0.5, -0.5, -0.5);
}

int main(int argc, char *argv[])
{
    SceneData Scene = { 0 };

    irqEnable(IRQ_HBLANK);
    irqSet(IRQ_VBLANK, NE_VBLFunc);
    irqSet(IRQ_HBLANK, NE_HBLFunc);

    // Start in DMA mode (no line artifacts, 75% texture VRAM)
    NE_Init3D_TwoPass_DMA();
    NE_InitConsole();
    init_all(&Scene);

    while (1)
    {
        NE_WaitForVBL(NE_TwoPassGetPass() == 1 ? NE_UPDATE_ANIMATIONS : 0);

        NE_ProcessTwoPassArg(Draw3DScene, &Scene);

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
            NE_Init3D_TwoPass_DMA();
            NE_InitConsole();
            init_all(&Scene);
        }
        if (kdown & KEY_X)
        {
            NE_Init3D_TwoPass_FB();
            NE_InitConsole();
            init_all(&Scene);
        }
        if (kdown & KEY_Y)
        {
            NE_Init3D_TwoPass();
            NE_InitConsole();
            init_all(&Scene);
        }

        // Update rotations only after both passes are done
        if (NE_TwoPassGetPass() == 1)
        {
            if (keys & KEY_UP)
                for (int i = 0; i < NUM_MODELS; i++)
                    NE_ModelRotate(Scene.Model[i], 2, 0, 0);
            if (keys & KEY_DOWN)
                for (int i = 0; i < NUM_MODELS; i++)
                    NE_ModelRotate(Scene.Model[i], -2, 0, 0);
            if (keys & KEY_RIGHT)
                for (int i = 0; i < NUM_MODELS; i++)
                    NE_ModelRotate(Scene.Model[i], 0, 4, 0);
            if (keys & KEY_LEFT)
                for (int i = 0; i < NUM_MODELS; i++)
                    NE_ModelRotate(Scene.Model[i], 0, -4, 0);
        }
    }

    NE_End();

    return 0;
}
