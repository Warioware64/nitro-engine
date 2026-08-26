// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Nitro Engine Advanced contributors, 2026
//
// This file is part of Nitro Engine Advanced

// Fitting a smoothly weighted mesh into the matrix-palette budget.
//
// NSMW spends one matrix-stack slot -- one "node" -- per distinct combination
// of bone pair and blend weight, and the hardware stack has 31 levels. A rigidly
// weighted mesh needs one slot per bone and never gets close. This tentacle
// blends every vertex between two bones with the blend varying per vertex, the
// way a real limb's weights do, and wants 194 slots for its 11 bone pairs.
//
// The exporter used to give up at that point and tell you to simplify the rig.
// It now clusters the nodes down to whatever budget it is given, merging the
// closest weights first and weighting each merge by how many vertices it
// affects, and it reports the blend error that cost.
//
// The same mesh is loaded here at three budgets so the trade is visible rather
// than described. The banded texture is deliberate: a skinning error shows up as
// a kink in what should be even stripes.
//
//   30 slots  the full budget       0.230 worst-case blend error
//   16 slots                        0.504
//   11 slots  one per bone pair,    0.511
//             the floor clustering can reach
//
// Below 11 no amount of clustering helps, because a bone pair needs at least one
// node. Going lower means splitting the mesh into several draws.

#include <NEAMain.h>

#include "skin.h"
#include "tentacle30_nsmw_bin.h"
#include "tentacle16_nsmw_bin.h"
#include "tentacle11_nsmw_bin.h"
#include "tentacle30_wave_dsa_bin.h"

#define NUM_BUDGETS 3

typedef struct {
    NEA_Camera *Camera;
    NEA_Model *Model[NUM_BUDGETS];
    int shown;
    bool side_by_side;
} SceneData;

static const char *BUDGET_NAMES[NUM_BUDGETS] = {
    "30 (full budget)",
    "16              ",
    "11 (the floor)  ",
};

static const char *BUDGET_ERRORS[NUM_BUDGETS] = {
    "0.230 worst 0.096 avg",
    "0.504 worst 0.186 avg",
    "0.511 worst 0.257 avg",
};

void Draw3DScene(void *arg)
{
    SceneData *Scene = arg;

    NEA_CameraUse(Scene->Camera);

    NEA_PolyFormat(31, 0, NEA_LIGHT_0, NEA_CULL_NONE, 0);

    if (Scene->side_by_side)
    {
        // All three at once. They are identical at the rest pose -- clustering
        // only changes how the mesh deforms, never where it sits when the
        // animation is at rest -- so any visible difference is the blend error.
        for (int i = 0; i < NUM_BUDGETS; i++)
            NEA_ModelDraw(Scene->Model[i]);
    }
    else
    {
        NEA_ModelDraw(Scene->Model[Scene->shown]);
    }
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
    consoleDemoInit();

    // The comparison is the point of the example, so start on it.
    Scene.side_by_side = true;

    Scene.Camera = NEA_CameraCreate();
    NEA_CameraSet(Scene.Camera,
                 0, 3.0, -11,
                 0, 2.5, 0,
                 0, 1, 0);

    NEA_Material *Material = NEA_MaterialCreate();
    NEA_MaterialTexLoad(Material, NEA_A1RGB5, 64, 64, NEA_TEXGEN_TEXCOORD,
                       skinBitmap);

    NEA_MaterialSetProperties(Material,
                             RGB15(26, 26, 26), // diffuse
                             RGB15(10, 10, 10), // ambient
                             RGB15(0, 0, 0),    // specular
                             RGB15(0, 0, 0),    // emission
                             false, false);

    // One animation drives all three: a DSA file holds only per-joint data, and
    // the node table sits on top of it, so it is budget-independent.
    NEA_Animation *Wave = NEA_AnimationCreate();
    NEA_AnimationLoad(Wave, tentacle30_wave_dsa_bin);

    const void *meshes[NUM_BUDGETS] = {
        tentacle30_nsmw_bin, tentacle16_nsmw_bin, tentacle11_nsmw_bin
    };

    for (int i = 0; i < NUM_BUDGETS; i++)
    {
        Scene.Model[i] = NEA_ModelCreate(NEA_AnimatedMW);
        NEA_ModelLoadNSMW(Scene.Model[i], meshes[i]);
        NEA_ModelSetSubMeshMaterial(Scene.Model[i], 0, Material);
        NEA_ModelSetAnimation(Scene.Model[i], Wave);
        NEA_ModelAnimStart(Scene.Model[i], NEA_ANIM_LOOP, floattof32(0.4));
    }

    NEA_LightSet(0, NEA_White, -0.4, -0.6, -0.6);

    bool spin = true;
    int angle = 0;

    while (1)
    {
        NEA_WaitForVBL(NEA_UPDATE_ANIMATIONS);

        scanKeys();
        uint32_t keys = keysDown();
        uint32_t held = keysHeld();

        if (keys & KEY_A)
            Scene.shown = (Scene.shown + 1) % NUM_BUDGETS;
        if (keys & KEY_B)
            Scene.side_by_side = !Scene.side_by_side;
        if (keys & KEY_START)
            spin = !spin;

        if (spin)
            angle = (angle + 1) & 511;
        if (held & KEY_LEFT)
            angle = (angle - 3) & 511;
        if (held & KEY_RIGHT)
            angle = (angle + 3) & 511;

        for (int i = 0; i < NUM_BUDGETS; i++)
        {
            NEA_ModelSetRot(Scene.Model[i], 0, angle, 0);
            NEA_ModelSetCoord(Scene.Model[i],
                              Scene.side_by_side ? (i - 1) * 3.2 : 0, 0, 0);
        }

        printf("\x1b[0;0H"
               "NSMW node budget\n"
               "================\n\n"
               "Mesh wants 194 slots.\n"
               "The stack has 30.\n\n"
               "A  Budget: %s\n"
               "   Error:  %s\n"
               "B  Side by side: %s\n"
               "Start  Spin: %s\n\n"
               "Fewer slots means coarser\n"
               "blending: the stripes kink\n"
               "where weights were merged\n"
               "too far.\n",
               BUDGET_NAMES[Scene.shown],
               BUDGET_ERRORS[Scene.shown],
               Scene.side_by_side ? "yes" : "no ",
               spin ? "on " : "off");

        NEA_ProcessArg(Draw3DScene, &Scene);
    }

    return 0;
}
