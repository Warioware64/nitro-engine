// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Warioware64, 2026
//
// This file is part of Nitro Engine Advanced
//
// Texture matrix example: demonstrates NEA_TextureMatrixTranslate(),
// NEA_TextureMatrixRotate(), and NEA_TextureMatrixScale() on a teapot model.
// Only the material loaded with NEA_TEXGEN_TEXCOORD is affected by the texture
// matrix. Materials with NEA_TEXGEN_OFF would not be affected.

#include <NEAMain.h>

#include "teapot_bin.h"
#include "teapot.h"

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
    NEA_TextureSystemReset(0, 0, NEA_VRAM_AB);

    // Init console on sub screen for instructions
    consoleDemoInit();

    Scene.Model = NEA_ModelCreate(NEA_Static);
    Scene.Camera = NEA_CameraCreate();
    NEA_Material *Material = NEA_MaterialCreate();

    NEA_CameraSet(Scene.Camera,
                 0, 0, -3,  // Position
                 0, 0, 0,   // Look at
                 0, 1, 0);  // Up direction

    NEA_ModelLoadStaticMesh(Scene.Model, teapot_bin);

    // Load texture with NEA_TEXGEN_TEXCOORD so the texture matrix applies.
    // Materials loaded with NEA_TEXGEN_OFF (the default) would NOT be affected
    // by the texture matrix, so there is no conflict between the two.
    NEA_MaterialTexLoad(Material, NEA_A1RGB5, 256, 256,
                       NEA_TEXGEN_TEXCOORD | NEA_TEXTURE_WRAP_S | NEA_TEXTURE_WRAP_T,
                       teapotBitmap);

    NEA_ModelSetMaterial(Scene.Model, Material);

    NEA_LightSet(0, NEA_White, -0.5, -0.5, -0.5);

    printf("\x1b[0;0H"
           "Texture Matrix Example\n"
           "\n"
           "ABXY:    Move texture\n"
           "Pad:     Rotate model\n"
           "L/R:     Rotate texture\n"
           "START:   Reset texture\n");

    int32_t tex_x = 0;
    int32_t tex_y = 0;
    int tex_rot = 0;

    while (1)
    {
        NEA_WaitForVBL(0);

        scanKeys();
        uint32_t keys = keysHeld();

        // Rotate model using D-pad
        if (keys & KEY_UP)
            NEA_ModelRotate(Scene.Model, 0, 0, 2);
        if (keys & KEY_DOWN)
            NEA_ModelRotate(Scene.Model, 0, 0, -2);
        if (keys & KEY_RIGHT)
            NEA_ModelRotate(Scene.Model, 0, 2, 0);
        if (keys & KEY_LEFT)
            NEA_ModelRotate(Scene.Model, 0, -2, 0);

        // Move texture using ABXY (increment = 32 texels per frame in f32)
        if (keys & KEY_A)
            tex_x += 32 << 12;
        if (keys & KEY_Y)
            tex_x -= 32 << 12;
        if (keys & KEY_B)
            tex_y += 32 << 12;
        if (keys & KEY_X)
            tex_y -= 32 << 12;

        // Rotate texture using L/R
        if (keys & KEY_L)
            tex_rot += 1;
        if (keys & KEY_R)
            tex_rot -= 1;

        // Reset texture matrix with START
        if (keysDown() & KEY_START)
        {
            tex_x = 0;
            tex_y = 0;
            tex_rot = 0;
        }

        // Apply texture matrix: identity first, then translate, then rotate
        NEA_TextureMatrixIdentity();
        if (tex_x != 0 || tex_y != 0)
            NEA_TextureMatrixTranslateI(tex_x, tex_y);
        if (tex_rot != 0)
            NEA_TextureMatrixRotate(tex_rot);

        NEA_ProcessArg(Draw3DScene, &Scene);
    }

    return 0;
}
