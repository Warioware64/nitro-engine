// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Warioware64, 2026
//
// This file is part of Nitro Engine Advanced

// Example: Collision group filtering with NEA_PhysicsSetGroupMask()
// Group 1 (green) and Group 2 (blue) cubes only collide within their
// own group. They pass through cubes of the other group.

#include <NEAMain.h>

#include "cube_bin.h"

#define NUM_CUBES 6

typedef struct {
    NEA_Camera *Camera;
    NEA_Model *Model[NUM_CUBES + 1];    // 6 cubes + floor
    NEA_Physics *Physics[NUM_CUBES + 1];
} SceneData;

void Draw3DScene(void *arg)
{
    SceneData *Scene = arg;

    NEA_CameraUse(Scene->Camera);

    // Group 1 cubes (green)
    NEA_PolyFormat(31, 0, NEA_LIGHT_0, NEA_CULL_BACK, 0);
    for (int i = 0; i < 3; i++)
        NEA_ModelDraw(Scene->Model[i]);

    // Group 2 cubes (blue)
    NEA_PolyFormat(31, 0, NEA_LIGHT_1, NEA_CULL_BACK, 0);
    for (int i = 3; i < NUM_CUBES; i++)
        NEA_ModelDraw(Scene->Model[i]);

    // Floor
    NEA_ModelDraw(Scene->Model[NUM_CUBES]);
}

int main(int argc, char *argv[])
{
    SceneData Scene = { 0 };

    irqEnable(IRQ_HBLANK);
    irqSet(IRQ_VBLANK, NEA_VBLFunc);
    irqSet(IRQ_HBLANK, NEA_HBLFunc);

    NEA_Init3D();
    consoleDemoInit();

    Scene.Camera = NEA_CameraCreate();
    NEA_CameraSet(Scene.Camera,
                  0, 5, -7,
                  0, 2, 0,
                  0, 1, 0);

    // Group 1 cubes (green, mask 0x01) - stacked at x=-1
    for (int i = 0; i < 3; i++)
    {
        Scene.Model[i] = NEA_ModelCreate(NEA_Static);
        NEA_ModelLoadStaticMesh(Scene.Model[i], cube_bin);
        NEA_ModelSetCoord(Scene.Model[i], -1, 3 + i * 2, 0);

        Scene.Physics[i] = NEA_PhysicsCreateEx(NEA_COL_AABB);
        NEA_PhysicsSetModel(Scene.Physics[i], Scene.Model[i]);
        NEA_PhysicsSetSize(Scene.Physics[i], 1, 1, 1);
        NEA_PhysicsEnable(Scene.Physics[i], true);
        NEA_PhysicsSetGravity(Scene.Physics[i], 0.001);
        NEA_PhysicsOnCollision(Scene.Physics[i], NEA_ColBounce);
        NEA_PhysicsSetBounceEnergy(Scene.Physics[i], 50);
        NEA_PhysicsSetGroupMask(Scene.Physics[i], 0x01);
    }

    // Group 2 cubes (blue, mask 0x02) - stacked at x=1, offset so they
    // overlap with group 1 in the middle
    for (int i = 0; i < 3; i++)
    {
        int idx = 3 + i;
        Scene.Model[idx] = NEA_ModelCreate(NEA_Static);
        NEA_ModelLoadStaticMesh(Scene.Model[idx], cube_bin);
        NEA_ModelSetCoord(Scene.Model[idx], 1, 4 + i * 2, 0);

        Scene.Physics[idx] = NEA_PhysicsCreateEx(NEA_COL_AABB);
        NEA_PhysicsSetModel(Scene.Physics[idx], Scene.Model[idx]);
        NEA_PhysicsSetSize(Scene.Physics[idx], 1, 1, 1);
        NEA_PhysicsEnable(Scene.Physics[idx], true);
        NEA_PhysicsSetGravity(Scene.Physics[idx], 0.001);
        NEA_PhysicsOnCollision(Scene.Physics[idx], NEA_ColBounce);
        NEA_PhysicsSetBounceEnergy(Scene.Physics[idx], 50);
        NEA_PhysicsSetGroupMask(Scene.Physics[idx], 0x02);
    }

    // Floor (collides with both groups, mask 0x03)
    Scene.Model[NUM_CUBES] = NEA_ModelCreate(NEA_Static);
    NEA_ModelLoadStaticMesh(Scene.Model[NUM_CUBES], cube_bin);
    NEA_ModelSetCoord(Scene.Model[NUM_CUBES], 0, 0, 0);
    NEA_ModelScale(Scene.Model[NUM_CUBES], 10, 1, 10);

    Scene.Physics[NUM_CUBES] = NEA_PhysicsCreateEx(NEA_COL_AABB);
    NEA_PhysicsSetModel(Scene.Physics[NUM_CUBES], Scene.Model[NUM_CUBES]);
    NEA_PhysicsSetSize(Scene.Physics[NUM_CUBES], 10, 1, 10);
    NEA_PhysicsSetGroupMask(Scene.Physics[NUM_CUBES], 0x03);
    NEA_PhysicsEnable(Scene.Physics[NUM_CUBES], false);

    printf("Collision Groups Demo\n\n");
    printf("Green cubes: Group 1\n");
    printf("Blue cubes:  Group 2\n\n");
    printf("Same group collides.\n");
    printf("Different groups pass\n");
    printf("through each other.\n\n");
    printf("D-pad:Rotate L/R:Move X/Y:Up/Dn\n");

    NEA_LightSet(0, NEA_Green, -1, -1, 0);
    NEA_LightSet(1, NEA_Blue, -1, -1, 0);
    NEA_ClearColorSet(NEA_Red, 31, 63);

    while (1)
    {
        NEA_WaitForVBL(NEA_UPDATE_PHYSICS);

        scanKeys();
        uint32_t keys = keysHeld();

        if (keys & KEY_UP)    NEA_CameraRotateFree(Scene.Camera, 2, 0, 0);
        if (keys & KEY_DOWN)  NEA_CameraRotateFree(Scene.Camera, -2, 0, 0);
        if (keys & KEY_LEFT)  NEA_CameraRotateFree(Scene.Camera, 0, 2, 0);
        if (keys & KEY_RIGHT) NEA_CameraRotateFree(Scene.Camera, 0, -2, 0);
        if (keys & KEY_L)     NEA_CameraMoveFree(Scene.Camera, 0.1, 0, 0);
        if (keys & KEY_R)     NEA_CameraMoveFree(Scene.Camera, -0.1, 0, 0);
        if (keys & KEY_X)     NEA_CameraMoveFree(Scene.Camera, 0, 0, 0.1);
        if (keys & KEY_Y)     NEA_CameraMoveFree(Scene.Camera, 0, 0, -0.1);

        NEA_ProcessArg(Draw3DScene, &Scene);
    }

    return 0;
}
