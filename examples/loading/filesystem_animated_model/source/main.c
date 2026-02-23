// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Antonio Niño Díaz, 2008-2024
//
// This file is part of Nitro Engine Advanced

#include <stdbool.h>
#include <stdio.h>

#include <filesystem.h>

#include <NEAMain.h>

typedef struct {
    NEA_Camera *Camera;
    NEA_Model *Model;
} SceneData;

void Draw3DScene(void *arg)
{
    SceneData *Scene = arg;

    NEA_CameraUse(Scene->Camera);

    NEA_PolyFormat(31, 0, NEA_LIGHT_0, NEA_CULL_NONE, 0);
    NEA_ModelDraw(Scene->Model);
}

void WaitLoop(void)
{
    while (1)
    {
        swiWaitForVBlank();
        scanKeys();
        if (keysHeld() & KEY_START)
            return;
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
    // Init console in non-3D screen
    consoleDemoInit();

    if (!nitroFSInit(NULL))
    {
        printf("nitroFSInit failed.\nPress START to exit");
        WaitLoop();
        return 0;
    }

    // Allocate space for objects...
    Scene.Model = NEA_ModelCreate(NEA_Animated);
    Scene.Camera = NEA_CameraCreate();
    NEA_Material *Material = NEA_MaterialCreate();
    NEA_Animation *Animation = NEA_AnimationCreate();

    // Setup camera
    NEA_CameraSet(Scene.Camera,
                 6, 3, -4,
                 0, 3, 0,
                 0, 1, 0);

    if (NEA_ModelLoadDSMFAT(Scene.Model, "robot.dsm") == 0)
    {
        printf("Couldn't load model...");
        WaitLoop();
        return 0;
    }

    if (NEA_AnimationLoadFAT(Animation, "robot_wave.dsa") == 0)
    {
        printf("Couldn't load animation...");
        WaitLoop();
        return 0;
    }

    if (NEA_MaterialTexLoadFAT(Material, NEA_A1RGB5, 256, 256, NEA_TEXGEN_TEXCOORD,
                              "texture.img.bin") == 0)
    {
        printf("Couldn't load texture...");
        WaitLoop();
        return 0;
    }

    // Assign material to the model
    NEA_ModelSetMaterial(Scene.Model, Material);

    NEA_ModelSetAnimation(Scene.Model, Animation);
    NEA_ModelAnimStart(Scene.Model, NEA_ANIM_LOOP, floattof32(0.1));

    NEA_LightSet(0, NEA_White, 0, -1, -1);

    NEA_ClearColorSet(NEA_Black, 31, 63);

    printf("\x1b[0;0HPad: Rotate\nSTART: Exit");

    while (1)
    {
        NEA_WaitForVBL(NEA_UPDATE_ANIMATIONS);

        scanKeys();
        uint32_t keys = keysHeld();

        if (keys & KEY_START)
            break;

        if (keys & KEY_RIGHT)
            NEA_ModelRotate(Scene.Model, 0, 2, 0);
        if (keys & KEY_LEFT)
            NEA_ModelRotate(Scene.Model, 0, -2, 0);
        if (keys & KEY_UP)
            NEA_ModelRotate(Scene.Model, 0, 0, 2);
        if (keys & KEY_DOWN)
            NEA_ModelRotate(Scene.Model, 0, 0, -2);

        NEA_ProcessArg(Draw3DScene, &Scene);
    }

    return 0;
}
