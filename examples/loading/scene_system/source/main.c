// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Warioware64, 2026
//
// This file is part of Nitro Engine Advanced
//
// Scene system example: demonstrates loading a .neascene binary, finding
// nodes by name and tag, setting up trigger callbacks, camera management,
// and per-frame scene update/draw.

#include <NEAMain.h>

#include "level_neascene_bin.h"
#include "cube_bin.h"

// Trigger callback: called when something enters/exits a trigger zone.
void OnTrigger(NEA_SceneNode *trigger, NEA_TriggerEvent event, void *user_data)
{
    (void)user_data;

    if (event == NEA_TRIGGER_ENTER)
        printf("  >> ENTER trigger '%s' (id=%d)\n",
               trigger->name, trigger->trigger->script_id);
    else if (event == NEA_TRIGGER_EXIT)
        printf("  << EXIT  trigger '%s'\n", trigger->name);
}

// Tag visitor: prints the name of each matching node.
void PrintTaggedNode(NEA_SceneNode *node, void *arg)
{
    (void)arg;
    printf("  - %s\n", node->name);
}

int main(int argc, char *argv[])
{
    (void)argc;
    (void)argv;

    irqEnable(IRQ_HBLANK);
    irqSet(IRQ_VBLANK, NEA_VBLFunc);
    irqSet(IRQ_HBLANK, NEA_HBLFunc);

    NEA_Init3D();
    NEA_TextureSystemReset(0, 0, NEA_VRAM_AB);
    consoleDemoInit();

    // Initialize the scene system (64 nodes max)
    NEA_SceneSystemReset(64);

    // Load the scene from embedded binary
    NEA_Scene *scene = NEA_SceneLoad(level_neascene_bin,
                                     level_neascene_bin_size);
    if (scene == NULL)
    {
        printf("Failed to load scene!\n");
        while (1)
            swiWaitForVBlank();
    }

    // Find mesh nodes and assign geometry
    NEA_SceneNode *cubeA = NEA_SceneFindNode(scene, "CubeA");
    NEA_SceneNode *cubeB = NEA_SceneFindNode(scene, "CubeB");

    if (cubeA && cubeA->model)
        NEA_ModelLoadStaticMesh(cubeA->model, cube_bin);
    if (cubeB && cubeB->model)
        NEA_ModelLoadStaticMesh(cubeB->model, cube_bin);

    // Set up trigger callbacks
    NEA_SceneNode *trigNode = NEA_SceneFindNode(scene, "TriggerZone");
    if (trigNode && trigNode->trigger)
        trigNode->trigger->on_event = OnTrigger;

    // Light setup
    NEA_LightSet(0, NEA_White, -0.5, -0.5, -0.5);

    // Print scene info
    printf("Scene System Example\n");
    printf("====================\n\n");
    printf("Nodes: %d\n", scene->num_nodes);
    printf("Camera: %s\n",
           scene->active_camera ? scene->active_camera->name : "none");

    // Tag query demo
    int n_pickup = NEA_SceneCountByTag(scene, "pickup");
    printf("\nTagged 'pickup' (%d):\n", n_pickup);
    NEA_SceneForEachTag(scene, "pickup", PrintTaggedNode, NULL);

    printf("\nD-Pad: Move CubeA\n");
    printf("A/B:   Show/Hide CubeB\n");
    printf("START: Exit\n\n");

    // Collision shape for trigger testing
    NEA_ColShape player_shape;
    NEA_ColShapeInitSphere(&player_shape, 0.5);

    int ry = 0;

    while (1)
    {
        NEA_WaitForVBL(0);
        scanKeys();
        uint16_t down = keysDown();
        uint16_t held = keysHeld();

        if (down & KEY_START)
            break;

        // Move CubeA with D-pad
        if (cubeA)
        {
            int32_t cx = cubeA->x;
            int32_t cz = cubeA->z;

            if (held & KEY_UP)    cz -= floattof32(0.1);
            if (held & KEY_DOWN)  cz += floattof32(0.1);
            if (held & KEY_LEFT)  cx -= floattof32(0.1);
            if (held & KEY_RIGHT) cx += floattof32(0.1);

            NEA_SceneNodeSetCoordI(cubeA, cx, cubeA->y, cz);
        }

        // Toggle CubeB visibility
        if (cubeB)
        {
            if (down & KEY_A)
                NEA_SceneNodeSetVisible(cubeB, true);
            if (down & KEY_B)
                NEA_SceneNodeSetVisible(cubeB, false);
        }

        // Rotate both cubes
        ry = (ry + 2) & 0x1FF;
        if (cubeA)
            NEA_SceneNodeSetRot(cubeA, 0, ry, 0);
        if (cubeB)
            NEA_SceneNodeSetRot(cubeB, 0, (512 - ry) & 0x1FF, 0);

        // Update scene hierarchy (propagates transforms)
        NEA_SceneUpdate(scene);

        // Test triggers against CubeA's world position
        if (cubeA)
        {
            NEA_Vec3 pos = { cubeA->wx, cubeA->wy, cubeA->wz };
            NEA_SceneTestTriggers(scene, &player_shape, pos, NULL);
        }

        // Draw the scene (uses active camera, draws all visible meshes)
        NEA_ProcessArg(NEA_SceneDraw, scene);

        printf("\x1b[18;0HCubeA pos: (%d, %d)   \n",
               (int)(cubeA ? cubeA->x >> 12 : 0),
               (int)(cubeA ? cubeA->z >> 12 : 0));
    }

    NEA_SceneFree(scene);
    NEA_SceneSystemEnd();

    return 0;
}
