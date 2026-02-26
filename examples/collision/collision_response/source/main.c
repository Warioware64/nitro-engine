// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Warioware64, 2026
//
// This file is part of Nitro Engine Advanced

// Example: Collision response types (Bounce, Stop, Slide, Nothing)

#include <NEAMain.h>

#include "sphere_bin.h"
#include "cube_bin.h"

typedef struct {
    NEA_Camera *Camera;
    NEA_Model *Model[5];    // 4 spheres + floor
    NEA_Physics *Physics[5];
} SceneData;

void Draw3DScene(void *arg)
{
    SceneData *Scene = arg;

    NEA_CameraUse(Scene->Camera);

    NEA_PolyFormat(31, 0, NEA_LIGHT_0, NEA_CULL_BACK, 0);
    for (int i = 0; i < 4; i++)
        NEA_ModelDraw(Scene->Model[i]);

    NEA_PolyFormat(31, 0, NEA_LIGHT_1, NEA_CULL_BACK, 0);
    NEA_ModelDraw(Scene->Model[4]);
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
                  0, 4, -8,
                  0, 2, 0,
                  0, 1, 0);

    const char *labels[] = { "Bounce", "Stop", "Slide", "Nothing" };
    NEA_OnCollision responses[] = {
        NEA_ColBounce, NEA_ColStop, NEA_ColSlide, NEA_ColNothing
    };

    // Create 4 spheres with different collision responses
    for (int i = 0; i < 4; i++)
    {
        Scene.Model[i] = NEA_ModelCreate(NEA_Static);
        NEA_ModelLoadStaticMesh(Scene.Model[i], sphere_bin);
        NEA_ModelSetCoord(Scene.Model[i], -3 + i * 2, 5, 0);

        Scene.Physics[i] = NEA_PhysicsCreateEx(NEA_COL_SPHERE);
        NEA_PhysicsSetModel(Scene.Physics[i], Scene.Model[i]);
        NEA_PhysicsSetRadius(Scene.Physics[i], 0.5);
        NEA_PhysicsEnable(Scene.Physics[i], true);
        NEA_PhysicsSetGravity(Scene.Physics[i], 0.001);
        NEA_PhysicsOnCollision(Scene.Physics[i], responses[i]);
        NEA_PhysicsSetBounceEnergy(Scene.Physics[i], 75);
    }

    // Give the Slide sphere some horizontal speed so sliding is visible
    NEA_PhysicsSetSpeed(Scene.Physics[2], 0.01, 0, 0);

    // Floor (wide static AABB)
    Scene.Model[4] = NEA_ModelCreate(NEA_Static);
    NEA_ModelLoadStaticMesh(Scene.Model[4], cube_bin);
    NEA_ModelSetCoord(Scene.Model[4], 0, 0, 0);
    NEA_ModelScale(Scene.Model[4], 10, 1, 10);

    Scene.Physics[4] = NEA_PhysicsCreateEx(NEA_COL_AABB);
    NEA_PhysicsSetModel(Scene.Physics[4], Scene.Model[4]);
    NEA_PhysicsSetSize(Scene.Physics[4], 10, 1, 10);
    NEA_PhysicsEnable(Scene.Physics[4], false);

    printf("Collision Response Demo\n\n");
    for (int i = 0; i < 4; i++)
        printf("Sphere %d: %s\n", i + 1, labels[i]);
    printf("\nD-pad:Rotate L/R:Move X/Y:Up/Dn\n");

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
