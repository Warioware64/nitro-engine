// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Antonio Niño Díaz, 2008-2024
//
// This file is part of Nitro Engine Advanced

#include <NEAMain.h>

#include "cube_bin.h"

typedef struct {
    NEA_Camera *Camara;
    NEA_Model *Model[6];
    NEA_Physics *Physics[6];
} SceneData;

void Draw3DScene(void *arg)
{
    SceneData *Scene = arg;

    NEA_CameraUse(Scene->Camara);

    NEA_PolyFormat(31, 0, NEA_LIGHT_0, NEA_CULL_BACK, 0);
    for (int i = 0; i < 5; i++)
        NEA_ModelDraw(Scene->Model[i]);

    NEA_PolyFormat(31, 0, NEA_LIGHT_1, NEA_CULL_BACK, 0);
    NEA_ModelDraw(Scene->Model[5]);
}

int main(int argc, char *argv[])
{
    SceneData Scene = { 0 };

    irqEnable(IRQ_HBLANK);
    irqSet(IRQ_VBLANK, NEA_VBLFunc);
    irqSet(IRQ_HBLANK, NEA_HBLFunc);

    NEA_Init3D();

    Scene.Camara = NEA_CameraCreate();
    NEA_CameraSet(Scene.Camara,
                 -9, 7, 5,
                  0, 6, 0,
                  0, 1, 0);

    // Create objects
    for (int i = 0; i < 6; i++)
    {
        Scene.Model[i] = NEA_ModelCreate(NEA_Static);
        Scene.Physics[i] = NEA_PhysicsCreate(NEA_BoundingBox);

        NEA_ModelLoadStaticMesh(Scene.Model[i], cube_bin);

        NEA_PhysicsSetModel(Scene.Physics[i], Scene.Model[i]);

        NEA_PhysicsSetSize(Scene.Physics[i], 1, 1, 1);
    }

    NEA_PhysicsEnable(Scene.Physics[5], false);

    // Object coordinates
    NEA_ModelSetCoord(Scene.Model[0], 0, 2, 0);
    NEA_ModelSetCoord(Scene.Model[1], 0, 4, 0);
    NEA_ModelSetCoord(Scene.Model[2], 0, 6, 0);
    NEA_ModelSetCoord(Scene.Model[3], 0, 8, 0);
    NEA_ModelSetCoord(Scene.Model[4], 0, 10, 0);
    NEA_ModelSetCoord(Scene.Model[5], 0, 0, 0);

    // Set gravity
    NEA_PhysicsSetGravity(Scene.Physics[0], 0.001);
    NEA_PhysicsSetGravity(Scene.Physics[1], 0.001);
    NEA_PhysicsSetGravity(Scene.Physics[2], 0.001);
    NEA_PhysicsSetGravity(Scene.Physics[3], 0.001);
    NEA_PhysicsSetGravity(Scene.Physics[4], 0.001);

    // Tell the engine what to do if there is a collision
    NEA_PhysicsOnCollision(Scene.Physics[0], NEA_ColBounce);
    NEA_PhysicsOnCollision(Scene.Physics[1], NEA_ColBounce);
    NEA_PhysicsOnCollision(Scene.Physics[2], NEA_ColBounce);
    NEA_PhysicsOnCollision(Scene.Physics[3], NEA_ColBounce);
    NEA_PhysicsOnCollision(Scene.Physics[4], NEA_ColBounce);

    // Set percent of energy kept after a bounce
    // Default is 50, 100 = no energy lost
    NEA_PhysicsSetBounceEnergy(Scene.Physics[0], 100);
    NEA_PhysicsSetBounceEnergy(Scene.Physics[1], 100);
    NEA_PhysicsSetBounceEnergy(Scene.Physics[2], 100);
    NEA_PhysicsSetBounceEnergy(Scene.Physics[3], 100);
    NEA_PhysicsSetBounceEnergy(Scene.Physics[4], 100);

    // Lights
    NEA_LightSet(0, NEA_Green, -1, -1, 0);
    NEA_LightSet(1, NEA_Blue, -1, -1, 0);

    // Background
    NEA_ClearColorSet(NEA_Red, 31, 63);

    while (1)
    {
        NEA_WaitForVBL(NEA_UPDATE_PHYSICS);

        NEA_ProcessArg(Draw3DScene, &Scene);
    }

    return 0;
}
