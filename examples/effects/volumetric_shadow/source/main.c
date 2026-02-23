// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Antonio Niño Díaz, 2024
//
// This file is part of Nitro Engine Advanced

#include <NEAMain.h>

#include "teapot_bin.h"
#include "teapot.h"

typedef struct {
    NEA_Camera *Camera;
    NEA_Model *Teapot;
    NEA_Material *Material;

    bool draw_edges;
} SceneData;

void DrawFloor(void)
{
    NEA_PolyNormal(0, -0.97, 0);

    NEA_PolyBegin(GL_QUAD);

        NEA_PolyTexCoord(0, 0);
        NEA_PolyVertex(-10, 0, -10);

        NEA_PolyTexCoord(0, 256);
        NEA_PolyVertex(-10, 0, 10);

        NEA_PolyTexCoord(256, 256);
        NEA_PolyVertex(10, 0, 10);

        NEA_PolyTexCoord(256, 0);
        NEA_PolyVertex(10, 0, -10);

    NEA_PolyEnd();
}

void DrawLid(void)
{
    NEA_PolyNormal(0, -0.97, 0);

    NEA_PolyBegin(GL_QUAD);

        NEA_PolyTexCoord(0, 0);
        NEA_PolyVertex(-0.75, 3, -0.75);

        NEA_PolyTexCoord(0, 256);
        NEA_PolyVertex(-0.75, 3,  0.75);

        NEA_PolyTexCoord(256, 256);
        NEA_PolyVertex( 0.75, 3,  0.75);

        NEA_PolyTexCoord(256, 0);
        NEA_PolyVertex( 0.75, 3, -0.75);

    NEA_PolyEnd();
}

void DrawShadowVolume(void)
{
    // Lid

    NEA_PolyBegin(GL_QUAD);

        NEA_PolyVertex(-0.75, 3, -0.75);
        NEA_PolyVertex(-0.75, 3,  0.75);
        NEA_PolyVertex( 0.75, 3,  0.75);
        NEA_PolyVertex( 0.75, 3, -0.75);

    NEA_PolyEnd();

    // Walls

    NEA_PolyBegin(GL_QUAD_STRIP);

        NEA_PolyVertex(-0.75, 3, -0.75);
        NEA_PolyVertex(-0.75, 0, -0.75);
        NEA_PolyVertex(-0.75, 3,  0.75);
        NEA_PolyVertex(-0.75, 0,  0.75);

        NEA_PolyVertex( 0.75, 3,  0.75);
        NEA_PolyVertex( 0.75, 0,  0.75);

        NEA_PolyVertex( 0.75, 3, -0.75);
        NEA_PolyVertex( 0.75, 0, -0.75);

        NEA_PolyVertex(-0.75, 3, -0.75);
        NEA_PolyVertex(-0.75, 0, -0.75);

    NEA_PolyEnd();

    // Bottom

    NEA_PolyBegin(GL_QUAD);

        NEA_PolyVertex(-0.75, 0, -0.75);
        NEA_PolyVertex(-0.75, 0,  0.75);
        NEA_PolyVertex( 0.75, 0,  0.75);
        NEA_PolyVertex( 0.75, 0, -0.75);

    NEA_PolyEnd();
}

void Draw3DSceneBright(void *arg)
{
    NEA_LightSet(0, NEA_White, 0, -0.97, -0.0);

    SceneData *Scene = arg;

    // Set camera
    NEA_CameraUse(Scene->Camera);

    // Set polygon format for regular models
    NEA_PolyFormat(31, 0, NEA_LIGHT_0, NEA_CULL_BACK, NEA_MODULATION);

    // Draw regular models
    NEA_ModelDraw(Scene->Teapot);

    NEA_MaterialUse(Scene->Material);
    DrawFloor();
    DrawLid();

    // Draw shadow volume as a black volume (shadow)
    NEA_MaterialUse(NULL);
    NEA_PolyColor(NEA_Black);

    if (Scene->draw_edges)
    {
        // Draw the shadow volume in wireframe mode to see where it is
        NEA_PolyFormat(0, 0, 0, NEA_CULL_NONE, NEA_MODULATION);
        DrawShadowVolume();
    }

    NEA_PolyFormat(1, 0, 0, NEA_CULL_NONE, NEA_SHADOW_POLYGONS);
    DrawShadowVolume();

    NEA_PolyFormat(20, 63, 0, NEA_CULL_NONE, NEA_SHADOW_POLYGONS);
    DrawShadowVolume();
}

void Draw3DSceneDark(void *arg)
{
    NEA_LightSet(0, RGB15(8, 8, 8), 0, -0.97, -0.0);

    SceneData *Scene = arg;

    // Set camera
    NEA_CameraUse(Scene->Camera);

    // Set polygon format for regular models
    NEA_PolyFormat(31, 0, NEA_LIGHT_0, NEA_CULL_BACK, NEA_MODULATION);

    // Draw regular models
    NEA_ModelDraw(Scene->Teapot);

    NEA_MaterialUse(Scene->Material);
    DrawFloor();
    DrawLid();

    // Draw shadow volume as a yellow volume (light)
    NEA_MaterialUse(NULL);
    NEA_PolyColor(RGB15(15, 15, 0));

    if (Scene->draw_edges)
    {
        // Draw the shadow volume in wireframe mode to see where it is
        NEA_PolyFormat(0, 0, 0, NEA_CULL_NONE, NEA_MODULATION);
        DrawShadowVolume();
    }

    NEA_PolyFormat(1, 0, 0, NEA_CULL_NONE, NEA_SHADOW_POLYGONS);
    DrawShadowVolume();

    NEA_PolyFormat(20, 63, 0, NEA_CULL_NONE, NEA_SHADOW_POLYGONS);
    DrawShadowVolume();
}

int main(int argc, char *argv[])
{
    SceneData Scene = { 0 };

    // This is needed for special screen effects
    irqEnable(IRQ_HBLANK);
    irqSet(IRQ_VBLANK, NEA_VBLFunc);
    irqSet(IRQ_VBLANK, NEA_HBLFunc);

    // Init console and Nitro Engine Advanced
    NEA_InitDual3D();
    NEA_InitConsole();

    // Setup camera
    Scene.Camera = NEA_CameraCreate();
    NEA_CameraSet(Scene.Camera,
                 0, 3.25, -3.25,
                 0, 1.25, 0,
                 0, 1, 0);

    // Load teapot and texture
    {
        Scene.Teapot = NEA_ModelCreate(NEA_Static);
        Scene.Material = NEA_MaterialCreate();

        // Load mesh from RAM and assign it to a model
        NEA_ModelLoadStaticMesh(Scene.Teapot, teapot_bin);

        // Load teapot texture from RAM and assign it to a material
        NEA_MaterialTexLoad(Scene.Material, NEA_A1RGB5, 256, 256,
                           NEA_TEXGEN_TEXCOORD | NEA_TEXTURE_WRAP_S | NEA_TEXTURE_WRAP_T,
                           teapotBitmap);

        // Assign material to the model
        NEA_ModelSetMaterial(Scene.Teapot, Scene.Material);

        // Set some properties to the material
        NEA_MaterialSetProperties(Scene.Material,
                        RGB15(24, 24, 24), // Diffuse
                        RGB15(8, 8, 8),    // Ambient
                        RGB15(0, 0, 0),    // Specular
                        RGB15(0, 0, 0),    // Emission
                        false, false);     // Vertex color, use shininess table

        // Set initial position of the object
        NEA_ModelSetCoordI(Scene.Teapot,
                          floattof32(0), floattof32(1.5), floattof32(0));
    }

    printf("\x1b[0;0H"
           "ABXY:    Rotate\n"
           "Pad:     Move\n"
           "SELECT:  Show edges of shadow\n"
           "START:   Exit to loader\n");

    while (1)
    {
        NEA_WaitForVBL(0);

        // Refresh keys
        scanKeys();
        uint32_t keys = keysHeld();

        // Move model using the pad
        if (keys & KEY_UP)
            NEA_ModelTranslate(Scene.Teapot, 0, 0, 0.05);
        if (keys & KEY_DOWN)
            NEA_ModelTranslate(Scene.Teapot, 0, 0, -0.05);
        if (keys & KEY_RIGHT)
            NEA_ModelTranslate(Scene.Teapot, -0.05, 0, 0);
        if (keys & KEY_LEFT)
            NEA_ModelTranslate(Scene.Teapot, 0.05, 0, 0);

        // Rotate model using the pad
        if (keys & KEY_Y)
            NEA_ModelRotate(Scene.Teapot, 0, 0, 2);
        if (keys & KEY_B)
            NEA_ModelRotate(Scene.Teapot, 0, 0, -2);
        if (keys & KEY_X)
            NEA_ModelRotate(Scene.Teapot, 0, 2, 0);
        if (keys & KEY_A)
            NEA_ModelRotate(Scene.Teapot, 0, -2, 0);

        if (keys & KEY_SELECT)
            Scene.draw_edges = true;
        else
            Scene.draw_edges = false;

        if (keys & KEY_START)
            break;

        // Draw Scene
        NEA_ProcessDualArg(Draw3DSceneBright, Draw3DSceneDark, &Scene, &Scene);
    }

    return 0;
}
