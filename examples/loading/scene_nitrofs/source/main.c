// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Warioware64, 2026
//
// This file is part of Nitro Engine Advanced
//
// NitroFS scene example: demonstrates fully automatic scene loading from
// NitroFS. Meshes and GRF textures are auto-loaded by NEA_SceneLoadFAT().

#include <NEAMain.h>

#include <filesystem.h>

int main(int argc, char *argv[])
{
    (void)argc;
    (void)argv;

    irqEnable(IRQ_HBLANK);
    irqSet(IRQ_VBLANK, NEA_VBLFunc);
    irqSet(IRQ_HBLANK, NEA_HBLFunc);

    NEA_Init3D();
    // Use VRAM_AB only — VRAM_C is reserved for consoleDemoInit()
    NEA_TextureSystemReset(0, 0, NEA_VRAM_AB);
    consoleDemoInit();

    // Initialize NitroFS
    if (!nitroFSInit(NULL))
    {
        printf("nitroFSInit failed.\nPress START to exit");
        while (1)
        {
            swiWaitForVBlank();
            scanKeys();
            if (keysHeld() & KEY_START)
                return 0;
        }
    }

    // Initialize the scene system
    NEA_SceneSystemReset(16);

    // Load the entire scene from NitroFS — meshes and textures are
    // automatically loaded from the asset and material reference tables.
    NEA_Scene *scene = NEA_SceneLoadFAT("level.neascene");
    if (scene == NULL)
    {
        printf("Failed to load scene!\n");
        while (1)
            swiWaitForVBlank();
    }

    // Set up a light
    NEA_LightSet(0, NEA_White, -0.5, -0.5, -0.5);

    printf("NitroFS Scene Example\n");
    printf("=====================\n\n");
    printf("Nodes: %d\n", scene->num_nodes);
    printf("Assets: %d\n", scene->num_assets);
    printf("Mat refs: %d\n", scene->num_mat_refs);

    // Show auto-loaded material status
    for (int i = 0; i < scene->num_mat_refs; i++)
    {
        printf("  mat[%d] '%s': %s\n", i,
               scene->mat_refs[i].name,
               scene->materials[i] ? "OK" : "FAIL");
    }

    // Show mesh node material bindings
    for (int i = 0; i < scene->num_nodes; i++)
    {
        NEA_SceneNode *node = &scene->nodes[i];
        if (node->type == NEA_NODE_MESH && node->model)
            printf("  %s: mat_idx=%d tex=%s\n", node->name,
                   node->ref.mesh.material_index,
                   node->model->texture ? "yes" : "no");
    }

    printf("\nD-Pad: Rotate cubes\n");
    printf("START: Exit\n");

    int ry = 0;

    while (1)
    {
        NEA_WaitForVBL(0);
        scanKeys();
        uint16_t keys = keysHeld();

        if (keysDown() & KEY_START)
            break;

        // Rotate all cubes with D-Pad
        if (keys & KEY_RIGHT) ry = (ry + 3) & 0x1FF;
        if (keys & KEY_LEFT)  ry = (ry - 3) & 0x1FF;

        // Apply rotation to all prop-tagged nodes
        for (int i = 0; i < scene->num_nodes; i++)
        {
            NEA_SceneNode *node = &scene->nodes[i];
            if (node->type == NEA_NODE_MESH)
                NEA_SceneNodeSetRot(node, 0, ry, 0);
        }

        // Update transforms and draw the scene
        NEA_SceneUpdate(scene);
        NEA_ProcessArg(NEA_SceneDraw, scene);
    }

    NEA_SceneFree(scene);
    NEA_SceneSystemEnd();

    return 0;
}
