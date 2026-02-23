// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Antonio Niño Díaz, 2008-2024
//
// This file is part of Nitro Engine Advanced

#include <NEAMain.h>

#include "sphere_bin.h"

typedef struct {
    NEA_Camera *Camera;
    NEA_Model *Model[10];

    int distancetocamera[10];

    int object_touched;

    int rotz, roty;

    bool touching;
} SceneData;

void Draw3DScene(void *arg)
{
    SceneData *Scene = arg;

    NEA_CameraUse(Scene->Camera);
    NEA_ViewRotate(0, Scene->roty, Scene->rotz);

    // Draw everything
    for (int i = 0; i < 10; i++)
    {
        if (i == Scene->object_touched)
            NEA_PolyFormat(31, 0, NEA_LIGHT_1, NEA_CULL_BACK, 0);
        else
            NEA_PolyFormat(31, 0, NEA_LIGHT_0, NEA_CULL_BACK, 0);

        NEA_ModelDraw(Scene->Model[i]);
    }

    if (Scene->touching)
    {
        // Get the information
        NEA_TouchTestStart();
        for (int i = 0; i < 10; i++)
        {
            // Models being drawn during the touch test aren't
            // actually drawn. That means you can use less detailed
            // objects, with no textures, etc, in order to make it
            // easier for the GPU to handle.
            NEA_TouchTestObject();
            NEA_ModelDraw(Scene->Model[i]);
            Scene->distancetocamera[i] = NEA_TouchTestResult();
        }
        NEA_TouchTestEnd();
    }
}

int main(int argc, char *argv[])
{
    SceneData Scene = { 0 };

    irqEnable(IRQ_HBLANK);
    irqSet(IRQ_VBLANK, NEA_VBLFunc);
    irqSet(IRQ_HBLANK, NEA_HBLFunc);

    NEA_Init3D();
    // Move 3D screen to lower screen
    NEA_SwapScreens();
    // libnds uses VRAM_C for the text console, reserve A and B only
    NEA_TextureSystemReset(0, 0, NEA_VRAM_AB);
    // Init console in non-3D screen
    consoleDemoInit();

    for (int i = 0; i < 10; i++)
        Scene.Model[i] = NEA_ModelCreate(NEA_Static);

    // Allocate everything
    Scene.Camera = NEA_CameraCreate();
    NEA_CameraSet(Scene.Camera,
                 -4, 0, 0,
                  0, 0, 0,
                  0, 1, 0);

    // Load model
    for (int i = 0; i < 10; i++)
        NEA_ModelLoadStaticMesh(Scene.Model[i], sphere_bin);

    // Set up lights
    NEA_LightSet(0, NEA_Yellow, 0, -0.5, -0.5);
    NEA_LightSet(1, NEA_Red, 0, -0.5, -0.5);

    printf("Press any key to start...");

    int framecount = 0;

    while (1)
    {
        if (framecount < 30)
            printf("\x1b[1;0H_");
        else
            printf("\x1b[1;0H ");

        if (framecount == 60)
            framecount = 0;

        framecount++;

        scanKeys();

        // Set random coordinates
        for (int i = 0; i < 10; i++)
        {
            NEA_ModelSetCoordI(Scene.Model[i],
                (rand() & (inttof32(3) - 1)) - floattof32(1.5),
                (rand() & (inttof32(3) - 1)) - floattof32(1.5),
                (rand() & (inttof32(3) - 1)) - floattof32(1.5));
        }

        if (keysHeld())
            break;

        swiWaitForVBlank();
    }

    printf("\x1b[0;0H                         ");
    printf("\x1b[1;0H ");

    printf("\x1b[0;0HNote: If two objects overlap,\n"
           "it may fail to diferenciate\nwhich is closer to the camera.");
    printf("\x1b[22;0HPAD: Rotate.");
    printf("\x1b[23;0HSTART: New positions.");

    while (1)
    {
        NEA_WaitForVBL(0);

        scanKeys();
        uint32_t keys = keysHeld();

        // Rotate view
        if (keys & KEY_RIGHT)
            Scene.roty--;
        if (keys & KEY_LEFT)
            Scene.roty++;
        if (keys & KEY_UP)
            Scene.rotz++;
        if (keys & KEY_DOWN)
            Scene.rotz--;

        if (keys & KEY_TOUCH)
            Scene.touching = true;
        else
            Scene.touching = false;

        if (keysDown() & KEY_START)
        {
            // Set random coordinates
            for (int i = 0; i < 10; i++)
            {
                NEA_ModelSetCoordI(Scene.Model[i],
                    (rand() & (inttof32(3) - 1)) - floattof32(1.5),
                    (rand() & (inttof32(3) - 1)) - floattof32(1.5) ,
                    (rand() & (inttof32(3) - 1)) - floattof32(1.5));
            }
        }

        // Reset object being touched, let's test if we're wrong
        Scene.object_touched = -1;

        if (keys & KEY_TOUCH)
        {
            // This is the part that checks if there are objects being touched

            // GL_MAX_DEPTH is the max possible distance
            int min_distance = GL_MAX_DEPTH;

            for (int j = 0; j < 10; j++)
            {
                // If the distance is greater than 0, the object has been
                // touched. Note that this array is filled in the drawing
                // function
                if (Scene.distancetocamera[j] >= 0)
                {
                    // If the object is closer than any previously detected
                    // object, replace it
                    if (Scene.distancetocamera[j] < min_distance)
                    {
                        Scene.object_touched = j;
                        min_distance = Scene.distancetocamera[j];
                    }
                }
            }
        }
        else
        {
            // Reset distances if screen is not being touched
            for (int j = 0; j < 10; j++)
                Scene.distancetocamera[j] = GL_MAX_DEPTH;
        }

        NEA_ProcessArg(Draw3DScene, &Scene);
    }

    return 0;
}
