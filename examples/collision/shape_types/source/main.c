// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Warioware64, 2026
//
// This file is part of Nitro Engine Advanced

// Example: Different collision shape types (Sphere, AABB, Capsule)

#include <NEAMain.h>

#include "cube_bin.h"
#include "sphere_bin.h"
#include "teapot_bin.h"

typedef struct {
    NEA_Camera *Camera;
    NEA_Model *Model[4];    // sphere, cube, teapot, floor
    NEA_Physics *Physics[4];
} SceneData;

void Draw3DScene(void *arg)
{
    SceneData *Scene = arg;

    NEA_CameraUse(Scene->Camera);

    NEA_PolyFormat(31, 0, NEA_LIGHT_0, NEA_CULL_BACK, 0);
    for (int i = 0; i < 3; i++)
        NEA_ModelDraw(Scene->Model[i]);

    NEA_PolyFormat(31, 0, NEA_LIGHT_1, NEA_CULL_BACK, 0);
    NEA_ModelDraw(Scene->Model[3]);
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
                  -6, 4, 0,
                   0, 2, 0,
                   0, 1, 0);

    // Sphere model with sphere collider
    Scene.Model[0] = NEA_ModelCreate(NEA_Static);
    NEA_ModelLoadStaticMesh(Scene.Model[0], sphere_bin);
    NEA_ModelSetCoord(Scene.Model[0], -2, 5, 0);

    Scene.Physics[0] = NEA_PhysicsCreateEx(NEA_COL_SPHERE);
    NEA_PhysicsSetModel(Scene.Physics[0], Scene.Model[0]);
    NEA_PhysicsSetRadius(Scene.Physics[0], 0.5);
    NEA_PhysicsEnable(Scene.Physics[0], true);
    NEA_PhysicsSetGravity(Scene.Physics[0], 0.001);
    NEA_PhysicsOnCollision(Scene.Physics[0], NEA_ColBounce);
    NEA_PhysicsSetBounceEnergy(Scene.Physics[0], 75);

    // Cube model with AABB collider
    Scene.Model[1] = NEA_ModelCreate(NEA_Static);
    NEA_ModelLoadStaticMesh(Scene.Model[1], cube_bin);
    NEA_ModelSetCoord(Scene.Model[1], 0, 5, 0);

    Scene.Physics[1] = NEA_PhysicsCreateEx(NEA_COL_AABB);
    NEA_PhysicsSetModel(Scene.Physics[1], Scene.Model[1]);
    NEA_PhysicsSetSize(Scene.Physics[1], 1, 1, 1);
    NEA_PhysicsEnable(Scene.Physics[1], true);
    NEA_PhysicsSetGravity(Scene.Physics[1], 0.001);
    NEA_PhysicsOnCollision(Scene.Physics[1], NEA_ColBounce);
    NEA_PhysicsSetBounceEnergy(Scene.Physics[1], 75);

    // Teapot model with capsule collider
    Scene.Model[2] = NEA_ModelCreate(NEA_Static);
    NEA_ModelLoadStaticMesh(Scene.Model[2], teapot_bin);
    NEA_ModelSetCoord(Scene.Model[2], 2, 5, 0);

    Scene.Physics[2] = NEA_PhysicsCreateEx(NEA_COL_CAPSULE);
    NEA_PhysicsSetModel(Scene.Physics[2], Scene.Model[2]);
    {
        NEA_ColShape capsule;
        NEA_ColShapeInitCapsule(&capsule, 0.3, 0.3);
        NEA_PhysicsSetColShape(Scene.Physics[2], &capsule);
    }
    NEA_PhysicsEnable(Scene.Physics[2], true);
    NEA_PhysicsSetGravity(Scene.Physics[2], 0.001);
    NEA_PhysicsOnCollision(Scene.Physics[2], NEA_ColBounce);
    NEA_PhysicsSetBounceEnergy(Scene.Physics[2], 75);

    // Floor (wide static AABB)
    Scene.Model[3] = NEA_ModelCreate(NEA_Static);
    NEA_ModelLoadStaticMesh(Scene.Model[3], cube_bin);
    NEA_ModelSetCoord(Scene.Model[3], 0, 0, 0);
    NEA_ModelScale(Scene.Model[3], 10, 1, 10);

    Scene.Physics[3] = NEA_PhysicsCreateEx(NEA_COL_AABB);
    NEA_PhysicsSetModel(Scene.Physics[3], Scene.Model[3]);
    NEA_PhysicsSetSize(Scene.Physics[3], 10, 1, 10);
    NEA_PhysicsEnable(Scene.Physics[3], false);

    printf("Shape Types Demo\n\n");
    printf("Left:  Sphere collider\n");
    printf("Mid:   AABB collider\n");
    printf("Right: Capsule collider\n\n");
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
