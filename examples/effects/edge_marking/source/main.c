// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Nitro Engine Advanced contributors, 2026
//
// This file is part of Nitro Engine Advanced

// Edge marking, and the polygon IDs it actually depends on.
//
// The GPU can outline polygons for free, but only where a pixel's neighbour has
// a *different* polygon ID. That makes the ID the thing to plan, not the colour.
// Give every model the same ID and the scene comes back with one silhouette
// around the whole pile; give each model its own and they separate.
//
// Two more rules fall out of how the hardware does it:
//
// - The rear plane has ID 63, so anything with another ID is outlined against
//   the background at no cost.
// - Edge colours are grouped in eights: colour 0 serves IDs 0-7, colour 1 serves
//   8-15, and so on. Objects that should share an outline colour go in the same
//   group.
//
// Antialiasing fights with edge marking over the same pixels. It is on by
// default; press X to see the difference.
//
// Cost: nothing per frame. Eight edge colours are written once, and each model
// costs two extra writes to the polygon format register when it has its own ID.

#include <NEAMain.h>

#include "surface.h"
#include "sphere_bin.h"
#include "teapot_bin.h"

#define NUM_MODELS 4

typedef struct {
    NEA_Camera *Camera;
    NEA_Model *Model[NUM_MODELS];
} SceneData;

// IDs 8 apart, so each model lands in a different edge-colour group and picks
// up a different outline colour. Note that none of them is 63 (the rear plane)
// or 62/61 (the GUI).
static const int GROUPED_IDS[NUM_MODELS] = { 0, 8, 16, 24 };

void Draw3DScene(void *arg)
{
    SceneData *Scene = arg;

    NEA_CameraUse(Scene->Camera);

    NEA_PolyFormat(31, 0, NEA_LIGHT_0, NEA_CULL_BACK, 0);

    for (int i = 0; i < NUM_MODELS; i++)
        NEA_ModelDraw(Scene->Model[i]);
}

int main(int argc, char *argv[])
{
    SceneData Scene = { 0 };

    irqEnable(IRQ_HBLANK);
    irqSet(IRQ_VBLANK, NEA_VBLFunc);
    irqSet(IRQ_HBLANK, NEA_HBLFunc);

    NEA_Init3D();
    // Use banks A and B for textures. libnds uses bank C for the text console.
    NEA_TextureSystemReset(0, 0, NEA_VRAM_AB);
    consoleDemoInit();

    Scene.Camera = NEA_CameraCreate();
    NEA_Material *Material = NEA_MaterialCreate();

    NEA_CameraSet(Scene.Camera,
                 0, 2, -5,
                 0, 0, 0,
                 0, 1, 0);

    NEA_MaterialTexLoad(Material, NEA_A1RGB5, 64, 64, NEA_TEXGEN_TEXCOORD,
                       surfaceBitmap);

    NEA_MaterialSetProperties(Material,
                             RGB15(20, 20, 22),
                             RGB15(10, 10, 12),
                             RGB15(0, 0, 0),
                             RGB15(0, 0, 0),
                             false, false);

    static const int px[NUM_MODELS] = { -3, -1, 1, 3 };

    for (int i = 0; i < NUM_MODELS; i++)
    {
        Scene.Model[i] = NEA_ModelCreate(NEA_Static);
        NEA_ModelLoadStaticMesh(Scene.Model[i],
                               (i & 1) ? teapot_bin : sphere_bin);
        NEA_ModelSetMaterial(Scene.Model[i], Material);
        NEA_ModelSetCoord(Scene.Model[i], px[i], (i & 1) ? -0.4 : 0, 0);
        NEA_ModelSetPolyID(Scene.Model[i], GROUPED_IDS[i]);
    }

    NEA_LightSet(0, NEA_White, -0.4, -0.6, -0.6);

    // One colour per group of eight IDs.
    NEA_OutliningSetColor(0, NEA_Black);              // IDs 0 - 7
    NEA_OutliningSetColor(1, RGB15(31, 4, 4));        // IDs 8 - 15
    NEA_OutliningSetColor(2, RGB15(4, 31, 8));        // IDs 16 - 23
    NEA_OutliningSetColor(3, RGB15(8, 12, 31));       // IDs 24 - 31

    NEA_OutliningEnable(true);
    NEA_ClearColorSet(RGB15(24, 24, 28), 31, 63);

    bool outlines = true;
    bool per_object_ids = true;
    bool antialias = true;
    bool one_color = false;
    bool spin = true;
    int angle = 0;

    while (1)
    {
        NEA_WaitForVBL(0);

        scanKeys();
        uint32_t keys = keysDown();

        if (keys & KEY_A)
        {
            outlines = !outlines;
            NEA_OutliningEnable(outlines);
        }
        if (keys & KEY_B)
        {
            // The whole point: with one shared ID the models stop being
            // outlined against each other and read as a single blob.
            per_object_ids = !per_object_ids;
            for (int i = 0; i < NUM_MODELS; i++)
            {
                NEA_ModelSetPolyID(Scene.Model[i],
                                  per_object_ids ? GROUPED_IDS[i] : 0);
            }
        }
        if (keys & KEY_X)
        {
            antialias = !antialias;
            NEA_AntialiasEnable(antialias);
        }
        if (keys & KEY_Y)
        {
            one_color = !one_color;
            if (one_color)
            {
                NEA_OutliningSetColorAll(NEA_Black);
            }
            else
            {
                NEA_OutliningSetColor(0, NEA_Black);
                NEA_OutliningSetColor(1, RGB15(31, 4, 4));
                NEA_OutliningSetColor(2, RGB15(4, 31, 8));
                NEA_OutliningSetColor(3, RGB15(8, 12, 31));
            }
        }
        if (keys & KEY_START)
            spin = !spin;

        if (spin)
            angle = (angle + 2) & 511;

        for (int i = 0; i < NUM_MODELS; i++)
            NEA_ModelSetRot(Scene.Model[i], 0, angle, 0);

        printf("\x1b[0;0H"
               "Edge marking\n"
               "============\n\n"
               "A  Outlines:      %s\n"
               "B  Per-object ID: %s\n"
               "X  Antialiasing:  %s\n"
               "Y  Colors:        %s\n"
               "Start  Spin: %s\n\n"
               "Share one ID and the models\n"
               "stop being outlined against\n"
               "each other. The rear plane  \n"
               "is ID 63, so every object   \n"
               "is outlined against the sky.\n",
               outlines ? "on " : "off",
               per_object_ids ? "yes" : "no (all 0)",
               antialias ? "on " : "off",
               one_color ? "one    " : "grouped",
               spin ? "on " : "off");

        NEA_ProcessArg(Draw3DScene, &Scene);
    }

    return 0;
}
