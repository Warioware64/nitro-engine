// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Nitro Engine Advanced contributors, 2026
//
// This file is part of Nitro Engine Advanced

// Sub-pixel jitter + temporal accumulation (NEAPostFX).
//
// The projection is shifted by a fraction of a pixel each frame following a
// 4-sample rotated grid, and the capture pipeline's frame blending averages
// those samples together. The result softens the DS's hard polygon edges
// without any per-pixel CPU work.
//
// This only works while things are still. There are no motion vectors on the
// DS, so the history is just the previous screen: anything that moves fast
// smears instead of resolving. The example makes that trade-off visible --
// press A to spin the model and watch jitter stop helping and start hurting,
// then press B to let the demo disable jitter automatically while it moves,
// which is what a game should do.
//
// Cost: the jitter itself is one matrix translate per frame. It is only useful
// together with the capture pipeline, which costs two VRAM banks (256 KB).

#include <NEAMain.h>

#include "texture.h"
#include "sphere_bin.h"

typedef struct {
    NEA_Camera *Camera;
    NEA_Model *Model;
    int angle;
} SceneData;

void Draw3DScene(void *arg)
{
    SceneData *Scene = arg;

    NEA_CameraUse(Scene->Camera);
    NEA_PolyFormat(31, 0, NEA_LIGHT_ALL, NEA_CULL_BACK, 0);
    NEA_ModelDraw(Scene->Model);
}

int main(int argc, char *argv[])
{
    SceneData Scene = { 0 };

    irqEnable(IRQ_HBLANK);
    irqSet(IRQ_VBLANK, NEA_VBLFunc);
    irqSet(IRQ_HBLANK, NEA_HBLFunc);

    NEA_Init3D();

    // Capture into A/B, console on C, textures in D.
    if (!NEA_PostFXCaptureInit(NEA_VRAM_A | NEA_VRAM_B))
    {
        consoleDemoInit();
        printf("Capture init failed\n");
        while (1)
            swiWaitForVBlank();
    }

    NEA_TextureSystemReset(0, 0, NEA_VRAM_D);
    consoleDemoInit();

    Scene.Model = NEA_ModelCreate(NEA_Static);
    Scene.Camera = NEA_CameraCreate();
    NEA_Material *Material = NEA_MaterialCreate();

    NEA_CameraSet(Scene.Camera, 0, 1, -3, 0, 0, 0, 0, 1, 0);

    NEA_ModelLoadStaticMesh(Scene.Model, sphere_bin);
    NEA_MaterialTexLoad(Material, NEA_A1RGB5, 256, 256, NEA_TEXGEN_TEXCOORD,
                       textureBitmap);
    NEA_ModelSetMaterial(Scene.Model, Material);
    NEA_LightSet(0, NEA_White, 0, -1, -1);

    // A decay of 8 averages roughly evenly over the recent frames, which is
    // what makes the jittered samples resolve instead of just ghosting.
    NEA_PostFXMotionBlurSetDecay(8);
    NEA_PostFXCaptureEnable(true);
    NEA_PostFXJitterEnable(true);

    bool want_jitter = true;
    bool spinning = false;
    bool auto_disable = false;

    while (1)
    {
        NEA_WaitForVBL(NEA_UPDATE_POSTFX);

        scanKeys();
        uint32_t keys = keysDown();

        if (keys & KEY_A)
            spinning = !spinning;
        if (keys & KEY_B)
            auto_disable = !auto_disable;
        if (keys & KEY_X)
            want_jitter = !want_jitter;

        if (spinning)
            Scene.angle = (Scene.angle + 3) & 0x1FF;

        NEA_ModelSetRot(Scene.Model, 0, Scene.angle >> 1, 0);

        // What a game should actually do: jitter only helps a near-still
        // image, so gate it on whether anything is moving.
        bool jitter_now = want_jitter && !(auto_disable && spinning);
        NEA_PostFXJitterEnable(jitter_now);

        printf("\x1b[0;0H"
               "PostFX jitter + accumulation\n"
               "============================\n\n"
               "X  Jitter wanted:  %s\n"
               "A  Model spinning: %s\n"
               "B  Auto-off when moving: %s\n\n"
               "Jitter active now: %s\n\n"
               "No motion vectors exist on\n"
               "the DS, so a moving image\n"
               "smears instead of resolving.\n"
               "Turning jitter off while\n"
               "things move is the fix, and\n"
               "it is one boolean per frame.\n\n"
               "Capture: 2 banks / 256 KB\n"
               "Jitter: 1 matrix translate\n",
               want_jitter ? "yes" : "no ",
               spinning ? "yes" : "no ",
               auto_disable ? "on " : "off",
               NEA_PostFXJitterIsEnabled() ? "yes" : "no ");

        NEA_ProcessArg(Draw3DScene, &Scene);
    }

    return 0;
}
