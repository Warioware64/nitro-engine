// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Warioware64, 2026
//
// This file is part of Nitro Engine Advanced

// Example: Mass-based impulse and restitution

#include <NEAMain.h>

#include "sphere_bin.h"
#include "cube_bin.h"

typedef struct {
    NEA_Camera *Camera;
    NEA_Model *Model[3];    // heavy sphere, light sphere, floor
    NEA_Physics *Physics[3];
} SceneData;

static void ResetScene(SceneData *Scene)
{
    NEA_ModelSetCoord(Scene->Model[0], -3, 3, 0);
    NEA_PhysicsSetSpeed(Scene->Physics[0], 0.02, 0, 0);

    NEA_ModelSetCoord(Scene->Model[1], 1, 3, 0);
    NEA_PhysicsSetSpeed(Scene->Physics[1], 0, 0, 0);
}

void Draw3DScene(void *arg)
{
    SceneData *Scene = arg;

    NEA_CameraUse(Scene->Camera);

    // Heavy sphere in green
    NEA_PolyFormat(31, 0, NEA_LIGHT_0, NEA_CULL_BACK, 0);
    NEA_ModelDraw(Scene->Model[0]);

    // Light sphere in blue
    NEA_PolyFormat(31, 0, NEA_LIGHT_1, NEA_CULL_BACK, 0);
    NEA_ModelDraw(Scene->Model[1]);

    // Floor
    NEA_ModelDraw(Scene->Model[2]);
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

    // Heavy sphere (mass 5.0, green)
    Scene.Model[0] = NEA_ModelCreate(NEA_Static);
    NEA_ModelLoadStaticMesh(Scene.Model[0], sphere_bin);

    Scene.Physics[0] = NEA_PhysicsCreateEx(NEA_COL_SPHERE);
    NEA_PhysicsSetModel(Scene.Physics[0], Scene.Model[0]);
    NEA_PhysicsSetRadius(Scene.Physics[0], 0.5);
    NEA_PhysicsEnable(Scene.Physics[0], true);
    NEA_PhysicsSetGravity(Scene.Physics[0], 0.001);
    NEA_PhysicsOnCollision(Scene.Physics[0], NEA_ColBounce);
    NEA_PhysicsSetBounceEnergy(Scene.Physics[0], 80);
    NEA_PhysicsSetMass(Scene.Physics[0], 5.0);
    NEA_PhysicsSetRestitution(Scene.Physics[0], 0.8);

    // Light sphere (mass 1.0, blue)
    Scene.Model[1] = NEA_ModelCreate(NEA_Static);
    NEA_ModelLoadStaticMesh(Scene.Model[1], sphere_bin);

    Scene.Physics[1] = NEA_PhysicsCreateEx(NEA_COL_SPHERE);
    NEA_PhysicsSetModel(Scene.Physics[1], Scene.Model[1]);
    NEA_PhysicsSetRadius(Scene.Physics[1], 0.5);
    NEA_PhysicsEnable(Scene.Physics[1], true);
    NEA_PhysicsSetGravity(Scene.Physics[1], 0.001);
    NEA_PhysicsOnCollision(Scene.Physics[1], NEA_ColBounce);
    NEA_PhysicsSetBounceEnergy(Scene.Physics[1], 80);
    NEA_PhysicsSetMass(Scene.Physics[1], 1.0);
    NEA_PhysicsSetRestitution(Scene.Physics[1], 0.8);

    // Floor (wide static AABB)
    Scene.Model[2] = NEA_ModelCreate(NEA_Static);
    NEA_ModelLoadStaticMesh(Scene.Model[2], cube_bin);
    NEA_ModelSetCoord(Scene.Model[2], 0, 0, 0);
    NEA_ModelScale(Scene.Model[2], 10, 1, 10);

    Scene.Physics[2] = NEA_PhysicsCreateEx(NEA_COL_AABB);
    NEA_PhysicsSetModel(Scene.Physics[2], Scene.Model[2]);
    NEA_PhysicsSetSize(Scene.Physics[2], 10, 1, 10);
    NEA_PhysicsEnable(Scene.Physics[2], false);

    ResetScene(&Scene);

    printf("Mass Impulse Demo\n\n");
    printf("Green: Heavy (mass 5.0)\n");
    printf("Blue:  Light (mass 1.0)\n\n");
    printf("Heavy sphere moves right\n");
    printf("and hits the light one.\n\n");
    printf("START:Reset\n");
    printf("D-pad:Rotate L/R:Move X/Y:Up/Dn\n");

    NEA_LightSet(0, NEA_Green, -1, -1, 0);
    NEA_LightSet(1, NEA_Blue, -1, -1, 0);
    NEA_ClearColorSet(NEA_Red, 31, 63);

    while (1)
    {
        NEA_WaitForVBL(NEA_UPDATE_PHYSICS);

        scanKeys();
        uint32_t keys = keysHeld();

        if (keysDown() & KEY_START)
            ResetScene(&Scene);

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
