// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Nitro Engine Advanced contributors, 2026
//
// This file is part of Nitro Engine Advanced

// PPU-native motion blur / afterimage (NEAPostFX frame capture).
//
// Every frame the composited screen is captured into one VRAM bank while the
// previous capture is shown from the other as a bitmap background and blended
// with the live 3D image. Because what gets captured is the *blended* result,
// the history decays exponentially and moving objects leave a trail.
//
// The model orbits continuously so the trail is visible without touching the
// controls.
//
// Cost, and it is the big one in this module: TWO VRAM banks, 256 KB, half of
// the A-D pool. Textures here are restricted to banks A and B as a result. It
// also takes BG2 and the blend unit, and switches the main engine to mode 5.
// Per frame it is a bank re-map plus three register writes -- no per-pixel CPU
// work at all.

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

    // Bank budget for this example:
    //   A + B  capture history and destination (256 KB)
    //   C      libnds text console on the sub screen
    //   D      3D textures (128 KB)
    //
    // The capture banks are a parameter, not hardcoded, so a game that needs
    // more texture space can move the console elsewhere and use C/D instead.
    // Set the capture up first: NEA_TextureSystemReset() asks the capture
    // subsystem which banks it owns and excludes them automatically.
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

    NEA_CameraSet(Scene.Camera, 0, 1, -4, 0, 0, 0, 0, 1, 0);

    NEA_ModelLoadStaticMesh(Scene.Model, sphere_bin);
    NEA_MaterialTexLoad(Material, NEA_A1RGB5, 256, 256, NEA_TEXGEN_TEXCOORD,
                       textureBitmap);
    NEA_ModelSetMaterial(Scene.Model, Material);

    NEA_LightSet(0, NEA_White, 0, -1, -1);

    NEA_PostFXMotionBlurPreset(NEA_POSTFX_BLUR_MEDIUM);
    NEA_PostFXCaptureEnable(true);

    int decay = 9;
    bool on = true;
    int mos = 0;

    while (1)
    {
        // NEA_UPDATE_POSTFX is what drives the per-frame bank swap and the
        // DISPCAPCNT arming. Without it nothing is captured.
        NEA_WaitForVBL(NEA_UPDATE_POSTFX);

        scanKeys();
        uint32_t keys = keysDown();
        uint32_t held = keysHeld();

        if (keys & KEY_A)
        {
            on = !on;
            NEA_PostFXCaptureEnable(on);
        }
        if (held & KEY_UP) decay++;
        if (held & KEY_DOWN) decay--;
        if (decay < 0) decay = 0;
        if (decay > 15) decay = 15;
        NEA_PostFXMotionBlurSetDecay(decay);

        if (keys & KEY_X)
        {
            // Pixelating the captured image is the only way to mosaic the 3D
            // scene: the hardware cannot mosaic the 3D layer directly.
            mos = mos ? 0 : 3;
            NEA_PostFXCaptureMosaic(mos, mos);
        }

        // Orbit the model so there is always something moving to smear.
        Scene.angle = (Scene.angle + 4) & 0x1FF;
        int32_t x = mulf32(floattof32(1.6f), cosLerp(Scene.angle << 6));
        int32_t y = mulf32(floattof32(1.0f), sinLerp(Scene.angle << 6));
        NEA_ModelSetCoordI(Scene.Model, x, y, 0);

        printf("\x1b[0;0H"
               "PostFX motion blur\n"
               "==================\n\n"
               "A  Capture:  %s\n"
               "Up/Down decay: %2d/15\n"
               "X  Mosaic 3D:  %s\n\n"
               "The model orbits on its\n"
               "own so the trail is always\n"
               "visible.\n\n"
               "Costs 2 VRAM banks (256 KB),\n"
               "BG2 and the blend unit.\n"
               "Per frame: a bank re-map\n"
               "and 3 register writes.\n",
               on ? "on " : "off", decay, mos ? "on " : "off");

        NEA_ProcessArg(Draw3DScene, &Scene);
    }

    return 0;
}
