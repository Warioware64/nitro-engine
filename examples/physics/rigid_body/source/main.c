// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Warioware64, 2026
//
// This file is part of Nitro Engine Advanced
//
// Rigid body physics example
// --------------------------
// Drops several OBB boxes onto a static floor plane. They tumble, bounce, and
// settle to sleep. D-pad applies forces; A button resets positions.

#include <NEAMain.h>
#include <NEARigidBody.h>

#include "cube_bin.h"

#define NUM_BOXES 4

typedef struct {
    NEA_Camera *camera;
    NEA_Model *model[NUM_BOXES];
    NEA_Model *floor_model;
    NEA_RigidBody *rb[NUM_BOXES];
} SceneData;

void Draw3DScene(void *arg)
{
    SceneData *scene = arg;

    NEA_CameraUse(scene->camera);

    NEA_PolyFormat(31, 0, NEA_LIGHT_0 | NEA_LIGHT_1, NEA_CULL_BACK, 0);

    for (int i = 0; i < NUM_BOXES; i++)
        NEA_ModelDraw(scene->model[i]);

    // Draw floor as a flat scaled cube
    NEA_PolyFormat(20, 0, NEA_LIGHT_0 | NEA_LIGHT_1, NEA_CULL_BACK, 0);
    NEA_ModelDraw(scene->floor_model);
}

int main(int argc, char *argv[])
{
    SceneData scene = { 0 };

    irqEnable(IRQ_HBLANK);
    irqSet(IRQ_VBLANK, NEA_VBLFunc);
    irqSet(IRQ_HBLANK, NEA_HBLFunc);

    NEA_Init3D();

    // Initialize rigid body system (sends START to ARM7)
    NEA_RigidBodyInit();

    // Camera
    scene.camera = NEA_CameraCreate();
    NEA_CameraSet(scene.camera,
                  -6, 5, 6,
                   0, 1, 0,
                   0, 1, 0);

    // Lights
    NEA_LightSet(0, NEA_White, 0, -1, -1);
    NEA_LightSet(1, NEA_Blue, -1, 0, 0);

    // Background
    NEA_ClearColorSet(RGB15(4, 4, 8), 31, 63);

    // Create a visible floor (flat cube scaled wide and thin)
    scene.floor_model = NEA_ModelCreate(NEA_Static);
    NEA_ModelLoadStaticMesh(scene.floor_model, cube_bin);
    NEA_ModelSetCoord(scene.floor_model, 0.0, -0.1, 0.0);
    NEA_ModelScaleI(scene.floor_model, inttof32(10), inttof32(1) / 10, inttof32(10));

    // Create rigid body boxes
    for (int i = 0; i < NUM_BOXES; i++)
    {
        scene.model[i] = NEA_ModelCreate(NEA_Static);
        NEA_ModelLoadStaticMesh(scene.model[i], cube_bin);

        // Create OBB: mass=1.0, half-extents 0.5 each (1x1x1 cube)
        scene.rb[i] = NEA_RigidBodyCreate(1.0, 0.5, 0.5, 0.5);
        NEA_RigidBodySetModel(scene.rb[i], scene.model[i]);

        // Stagger initial positions
        NEA_RigidBodySetPosition(scene.rb[i],
            (float)(i - 1) * 1.5f,  // X: spread apart
            4.0f + (float)i * 2.0f, // Y: stagger heights
            0.0f);                   // Z: centered

        // Bounciness and friction
        NEA_RigidBodySetRestitution(scene.rb[i], 0.3);
        NEA_RigidBodySetFriction(scene.rb[i], 0.5);
    }

    // Set global gravity
    NEA_RigidBodySetGravity(-0.5);

    // Add a static floor plane at Y=0
    // Position (0, 0, 0), half-extents (20, 0, 20), normal (0, 1, 0)
    NEA_RigidBodyAddStatic(
        0.0, 0.0, 0.0,    // center
        20.0, 0.0, 20.0,  // half-size (flat on XZ)
        0.0, 1.0, 0.0);   // normal (up)

    consoleDemoInit();
    printf("NEA Rigid Body Demo\n");
    printf("-------------------\n");
    printf("D-pad: push boxes\n");
    printf("A: reset positions\n");
    printf("B: apply upward impulse\n");

    while (1)
    {
        NEA_WaitForVBL(NEA_UPDATE_RIGIDBODY);

        scanKeys();
        uint32_t keys = keysHeld();
        uint32_t down = keysDown();

        // D-pad: apply force to first box
        if (scene.rb[0] && scene.rb[0]->active)
        {
            if (keys & KEY_RIGHT)
                NEA_RigidBodyApplyForce(scene.rb[0],
                    3.0, 0, 0,  0, 0, 0);
            if (keys & KEY_LEFT)
                NEA_RigidBodyApplyForce(scene.rb[0],
                    -3.0, 0, 0,  0, 0, 0);
            if (keys & KEY_UP)
                NEA_RigidBodyApplyForce(scene.rb[0],
                    0, 0, -3.0,  0, 0, 0);
            if (keys & KEY_DOWN)
                NEA_RigidBodyApplyForce(scene.rb[0],
                    0, 0, 3.0,  0, 0, 0);
        }

        // B: upward impulse on all boxes
        if (down & KEY_B)
        {
            for (int i = 0; i < NUM_BOXES; i++)
            {
                if (scene.rb[i] && scene.rb[i]->active)
                    NEA_RigidBodyApplyImpulse(scene.rb[i],
                        0, 3.0, 0,  0, 0, 0);
            }
        }

        // A: reset all positions
        if (down & KEY_A)
        {
            for (int i = 0; i < NUM_BOXES; i++)
            {
                if (scene.rb[i] && scene.rb[i]->active)
                {
                    NEA_RigidBodySetPosition(scene.rb[i],
                        (float)(i - 1) * 1.5f,
                        4.0f + (float)i * 2.0f,
                        0.0f);
                    NEA_RigidBodySetVelocity(scene.rb[i], 0, 0, 0);
                }
            }
        }

        // Print status
        printf("\x1b[7;0H");
        for (int i = 0; i < NUM_BOXES; i++)
        {
            if (scene.rb[i] && scene.rb[i]->active)
            {
                NEA_Vec3 pos = NEA_RigidBodyGetPosition(scene.rb[i]);
                printf("Box%d: %ld %ld %ld %s   \n", i,
                    pos.x >> 12, pos.y >> 12, pos.z >> 12,
                    NEA_RigidBodyIsSleeping(scene.rb[i]) ? "zzz" : "   ");
            }
        }

        NEA_ProcessArg(Draw3DScene, &scene);
    }

    return 0;
}
