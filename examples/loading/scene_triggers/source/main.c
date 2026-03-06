// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Warioware64, 2026
//
// This file is part of Nitro Engine Advanced
//
// Scene triggers example: demonstrates multiple trigger zones with
// enter/exit/tick callbacks, AABB and sphere triggers, script_id
// identification, and tag-based trigger queries.

#include <NEAMain.h>

#include "triggers_neascene_bin.h"
#include "cube_bin.h"

// Track which triggers the player is inside
static bool inside[4] = { false, false, false, false };
static const char *trigger_labels[] = { "Boundary", "A", "B", "C" };

void OnTrigger(NEA_SceneNode *trigger, NEA_TriggerEvent event, void *user_data)
{
    (void)user_data;
    int id = trigger->trigger->script_id;
    if (id < 0 || id > 3)
        return;

    if (event == NEA_TRIGGER_ENTER)
    {
        inside[id] = true;
        printf(">> ENTER %s (id=%d)\n", trigger->name, id);
    }
    else if (event == NEA_TRIGGER_EXIT)
    {
        inside[id] = false;
        printf("<< EXIT  %s (id=%d)\n", trigger->name, id);
    }
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

    NEA_SceneSystemReset(64);

    NEA_Scene *scene = NEA_SceneLoad(triggers_neascene_bin,
                                     triggers_neascene_bin_size);
    if (scene == NULL)
    {
        printf("Failed to load scene!\n");
        while (1)
            swiWaitForVBlank();
    }

    // Load cube mesh into all mesh nodes
    for (int i = 0; i < scene->num_nodes; i++)
    {
        NEA_SceneNode *n = &scene->nodes[i];
        if (n->type == NEA_NODE_MESH && n->model)
            NEA_ModelLoadStaticMesh(n->model, cube_bin);
    }

    // Set up trigger callbacks for all trigger nodes
    int num_triggers = NEA_SceneCountByTag(scene, "trigger");
    for (int i = 0; i < scene->num_nodes; i++)
    {
        NEA_SceneNode *n = &scene->nodes[i];
        if (n->type == NEA_NODE_TRIGGER && n->trigger)
            n->trigger->on_event = OnTrigger;
    }

    // Also set up the boundary trigger
    NEA_SceneNode *boundary = NEA_SceneFindNode(scene, "Boundary");
    if (boundary && boundary->trigger)
        boundary->trigger->on_event = OnTrigger;

    NEA_SceneNode *player = NEA_SceneFindNode(scene, "Player");

    NEA_LightSet(0, NEA_White, -0.5, -0.5, -0.5);

    printf("Scene Triggers Example\n");
    printf("======================\n\n");
    printf("Nodes: %d\n", scene->num_nodes);
    printf("Trigger zones: %d\n", num_triggers);
    printf("Boundary (AABB): %s\n\n",
           boundary ? "yes" : "no");
    printf("D-Pad: Move player\n");
    printf("START: Exit\n\n");

    // Player collision shape for trigger testing
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

        // Move player with D-pad
        if (player)
        {
            int32_t px = player->x;
            int32_t pz = player->z;

            if (held & KEY_UP)    pz -= floattof32(0.15);
            if (held & KEY_DOWN)  pz += floattof32(0.15);
            if (held & KEY_LEFT)  px -= floattof32(0.15);
            if (held & KEY_RIGHT) px += floattof32(0.15);

            NEA_SceneNodeSetCoordI(player, px, player->y, pz);

            // Rotate player
            ry = (ry + 3) & 0x1FF;
            NEA_SceneNodeSetRot(player, 0, ry, 0);
        }

        NEA_SceneUpdate(scene);

        // Test player against all triggers
        if (player)
        {
            NEA_Vec3 pos = { player->wx, player->wy, player->wz };
            NEA_SceneTestTriggers(scene, &player_shape, pos, NULL);
        }

        NEA_ProcessArg(NEA_SceneDraw, scene);

        // Status display
        printf("\x1b[16;0HPlayer: (%d, %d)      \n",
               (int)(player ? player->x >> 12 : 0),
               (int)(player ? player->z >> 12 : 0));
        printf("Zones: ");
        for (int i = 0; i < 4; i++)
        {
            if (inside[i])
                printf("[%s] ", trigger_labels[i]);
        }
        printf("        \n");
    }

    NEA_SceneFree(scene);
    NEA_SceneSystemEnd();

    return 0;
}
