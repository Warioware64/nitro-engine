// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Warioware64, 2026
//
// This file is part of Nitro Engine Advanced
//
// Scene camera example: demonstrates loading a scene with multiple cameras
// and switching between them at runtime using NEA_SceneSetActiveCamera().

#include <NEAMain.h>

#include "multicam_neascene_bin.h"
#include "cube_bin.h"

// Camera names for display
static const char *cam_names[] = { "CamFront", "CamTop", "CamSide" };
#define NUM_CAMERAS 3

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

    // Load scene
    NEA_Scene *scene = NEA_SceneLoad(multicam_neascene_bin,
                                     multicam_neascene_bin_size);
    if (scene == NULL)
    {
        printf("Failed to load scene!\n");
        while (1)
            swiWaitForVBlank();
    }

    // Find all camera nodes
    NEA_SceneNode *cameras[NUM_CAMERAS];
    for (int i = 0; i < NUM_CAMERAS; i++)
    {
        cameras[i] = NEA_SceneFindNode(scene, cam_names[i]);
        if (cameras[i] == NULL)
            printf("Warning: %s not found\n", cam_names[i]);
    }

    // Find mesh nodes and load cube geometry
    NEA_SceneNode *cubes[3];
    cubes[0] = NEA_SceneFindNode(scene, "CubeCenter");
    cubes[1] = NEA_SceneFindNode(scene, "CubeLeft");
    cubes[2] = NEA_SceneFindNode(scene, "CubeRight");

    for (int i = 0; i < 3; i++)
    {
        if (cubes[i] && cubes[i]->model)
            NEA_ModelLoadStaticMesh(cubes[i]->model, cube_bin);
    }

    // Light
    NEA_LightSet(0, NEA_White, -0.5, -0.5, -0.5);

    int cam_idx = 0;
    int ry = 0;

    printf("Scene Camera Example\n");
    printf("====================\n\n");
    printf("Nodes: %d\n", scene->num_nodes);
    printf("Cameras: %d\n\n", NUM_CAMERAS);
    printf("L/R: Switch camera\n");
    printf("Pad: Rotate cubes\n");
    printf("START: Exit\n");

    while (1)
    {
        NEA_WaitForVBL(0);
        scanKeys();
        uint16_t down = keysDown();
        uint16_t held = keysHeld();

        if (down & KEY_START)
            break;

        // Cycle cameras with L/R
        if (down & KEY_R)
        {
            cam_idx = (cam_idx + 1) % NUM_CAMERAS;
            if (cameras[cam_idx])
                NEA_SceneSetActiveCamera(scene, cameras[cam_idx]);
        }
        if (down & KEY_L)
        {
            cam_idx = (cam_idx + NUM_CAMERAS - 1) % NUM_CAMERAS;
            if (cameras[cam_idx])
                NEA_SceneSetActiveCamera(scene, cameras[cam_idx]);
        }

        // Rotate cubes with D-pad
        if (held & KEY_LEFT)
            ry = (ry - 3) & 0x1FF;
        if (held & KEY_RIGHT)
            ry = (ry + 3) & 0x1FF;

        for (int i = 0; i < 3; i++)
        {
            if (cubes[i])
            {
                int offset = (i * 170) & 0x1FF;  // 120-degree spacing
                NEA_SceneNodeSetRot(cubes[i], 0,
                                     (ry + offset) & 0x1FF, 0);
            }
        }

        NEA_SceneUpdate(scene);
        NEA_ProcessArg(NEA_SceneDraw, scene);

        printf("\x1b[16;0HCamera: [%d] %s    \n",
               cam_idx, cam_names[cam_idx]);
    }

    NEA_SceneFree(scene);
    NEA_SceneSystemEnd();

    return 0;
}
