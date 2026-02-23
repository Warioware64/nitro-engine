// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Antonio Niño Díaz, 2008-2024
//
// This file is part of Nitro Engine Advanced

#include <NEAMain.h>

#include "robot_dsm_bin.h"
#include "robot_walk_dsa_bin.h"
#include "robot_wave_dsa_bin.h"
#include "texture.h"

typedef struct {
    NEA_Camera *Camera;
    NEA_Model *Model;
} SceneData;

void Draw3DScene(void *arg)
{
    SceneData *Scene = arg;

    NEA_PolyFormat(31, 0, NEA_LIGHT_0, NEA_CULL_BACK, 0);

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
    NEA_InitConsole();

    Scene.Camera = NEA_CameraCreate();
    Scene.Model = NEA_ModelCreate(NEA_Animated);

    NEA_Animation *Animation[2];
    Animation[0] = NEA_AnimationCreate();
    Animation[1] = NEA_AnimationCreate();

    NEA_AnimationLoad(Animation[0], robot_walk_dsa_bin);
    NEA_AnimationLoad(Animation[1], robot_wave_dsa_bin);
    NEA_ModelLoadDSM(Scene.Model, robot_dsm_bin);
    NEA_ModelSetAnimation(Scene.Model, Animation[0]);
    NEA_ModelSetAnimationSecondary(Scene.Model, Animation[1]);
    NEA_ModelAnimStart(Scene.Model, NEA_ANIM_LOOP, floattof32(0.1));
    NEA_ModelAnimSecondaryStart(Scene.Model, NEA_ANIM_LOOP, floattof32(0.1));

    NEA_CameraSet(Scene.Camera,
                 6, 3, -4,
                 0, 3, 0,
                 0, 1, 0);

    NEA_Material *Texture = NEA_MaterialCreate();
    NEA_MaterialTexLoad(Texture, NEA_A1RGB5, 256, 256, NEA_TEXGEN_TEXCOORD,
                       textureBitmap);

    NEA_ModelSetMaterial(Scene.Model, Texture);

    NEA_LightSet(0, NEA_White, -0.9, 0, 0);
    NEA_ClearColorSet(NEA_Black, 31, 63);

    int32_t blend = floattof32(0.5);

    printf("\x1b[0;0H"
           "L/R: Remove one animation\n"
           "A/B: Blend factor\n"
           "Pad: Rotate\n");

    while (1)
    {
        NEA_WaitForVBL(NEA_UPDATE_ANIMATIONS);

        scanKeys();
        uint32_t keys = keysHeld();

        if (keys & KEY_RIGHT)
            NEA_ModelRotate(Scene.Model, 0, 2, 0);
        if (keys & KEY_LEFT)
            NEA_ModelRotate(Scene.Model, 0, -2, 0);
        if (keys & KEY_UP)
            NEA_ModelRotate(Scene.Model, 0, 0, 2);
        if (keys & KEY_DOWN)
            NEA_ModelRotate(Scene.Model, 0, 0, -2);

        if (keys & KEY_A)
        {
            blend += floattof32(0.02);
            if (blend > inttof32(1))
                blend = inttof32(1);
        }
        if (keys & KEY_B)
        {
            blend -= floattof32(0.02);
            if (blend < 0)
                blend = 0;
        }

        if (keys & KEY_L)
            NEA_ModelAnimSecondaryClear(Scene.Model, true);
        if (keys & KEY_R)
            NEA_ModelAnimSecondaryClear(Scene.Model, false);

        NEA_ModelAnimSecondarySetFactor(Scene.Model, blend);

        printf("\x1b[20;0H"
               "CPU%%: %d  \n"
               "Blend factor:      %.3f  \n"
               "Frame (main):      %.3f  \n"
               "Frame (secondary): %.3f  ",
               NEA_GetCPUPercent(), f32tofloat(blend),
               f32tofloat(NEA_ModelAnimGetFrame(Scene.Model)),
               f32tofloat(NEA_ModelAnimSecondaryGetFrame(Scene.Model)));

        NEA_ProcessArg(Draw3DScene, &Scene);
    }

    return 0;
}
