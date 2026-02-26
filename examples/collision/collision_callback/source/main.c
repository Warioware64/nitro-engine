// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Warioware64, 2026
//
// This file is part of Nitro Engine Advanced

// Example: Collision callbacks with NEA_PhysicsSetCallback()

#include <NEAMain.h>

#include "sphere_bin.h"
#include "cube_bin.h"

#define NUM_SPHERES 4

static NEA_Physics *g_physics[NUM_SPHERES];
static int g_collision_count[NUM_SPHERES];

static void OnCollision(NEA_Physics *self, NEA_Physics *other,
                        const NEA_ColResult *result)
{
    for (int i = 0; i < NUM_SPHERES; i++)
    {
        if (self == g_physics[i])
        {
            g_collision_count[i]++;
            break;
        }
    }
}

typedef struct {
    NEA_Camera *Camera;
    NEA_Model *Model[NUM_SPHERES + 1]; // spheres + floor
    NEA_Physics *Physics[NUM_SPHERES + 1];
} SceneData;

void Draw3DScene(void *arg)
{
    SceneData *Scene = arg;

    NEA_CameraUse(Scene->Camera);

    NEA_PolyFormat(31, 0, NEA_LIGHT_0, NEA_CULL_BACK, 0);
    for (int i = 0; i < NUM_SPHERES; i++)
        NEA_ModelDraw(Scene->Model[i]);

    NEA_PolyFormat(31, 0, NEA_LIGHT_1, NEA_CULL_BACK, 0);
    NEA_ModelDraw(Scene->Model[NUM_SPHERES]);
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

    // Create spheres at different heights so they hit at different times
    for (int i = 0; i < NUM_SPHERES; i++)
    {
        Scene.Model[i] = NEA_ModelCreate(NEA_Static);
        NEA_ModelLoadStaticMesh(Scene.Model[i], sphere_bin);
        NEA_ModelSetCoord(Scene.Model[i], -3 + i * 2, 3 + i, 0);

        Scene.Physics[i] = NEA_PhysicsCreateEx(NEA_COL_SPHERE);
        NEA_PhysicsSetModel(Scene.Physics[i], Scene.Model[i]);
        NEA_PhysicsSetRadius(Scene.Physics[i], 0.5);
        NEA_PhysicsEnable(Scene.Physics[i], true);
        NEA_PhysicsSetGravity(Scene.Physics[i], 0.001);
        NEA_PhysicsOnCollision(Scene.Physics[i], NEA_ColBounce);
        NEA_PhysicsSetBounceEnergy(Scene.Physics[i], 75);
        NEA_PhysicsSetCallback(Scene.Physics[i], OnCollision);

        g_physics[i] = Scene.Physics[i];
        g_collision_count[i] = 0;
    }

    // Floor (wide static AABB)
    Scene.Model[NUM_SPHERES] = NEA_ModelCreate(NEA_Static);
    NEA_ModelLoadStaticMesh(Scene.Model[NUM_SPHERES], cube_bin);
    NEA_ModelSetCoord(Scene.Model[NUM_SPHERES], 0, 0, 0);
    NEA_ModelScale(Scene.Model[NUM_SPHERES], 10, 1, 10);

    Scene.Physics[NUM_SPHERES] = NEA_PhysicsCreateEx(NEA_COL_AABB);
    NEA_PhysicsSetModel(Scene.Physics[NUM_SPHERES], Scene.Model[NUM_SPHERES]);
    NEA_PhysicsSetSize(Scene.Physics[NUM_SPHERES], 10, 1, 10);
    NEA_PhysicsEnable(Scene.Physics[NUM_SPHERES], false);

    printf("Collision Callback Demo\n\n");

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

        // Display collision counts
        printf("\x1b[3;0H"); // Move cursor to row 3
        for (int i = 0; i < NUM_SPHERES; i++)
            printf("Sphere %d: %d hits   \n", i + 1, g_collision_count[i]);
        printf("\nD-pad:Rotate L/R:Move X/Y:Up/Dn\n");

        NEA_ProcessArg(Draw3DScene, &Scene);
    }

    return 0;
}
