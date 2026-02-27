// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Warioware64, 2026
//
// This file is part of Nitro Engine Advanced
//
// Spatial sound example: four cubes placed in the scene emit positioned sound
// effects. The listener is attached to the camera, so walking around makes the
// volume and panning change in real time.

#include <NEAMain.h>

#include "soundbank.h"
#include "soundbank_bin.h"
#include "cube_bin.h"

#define NUM_SOURCES 4

typedef struct {
    NEA_Camera *Camera;
    NEA_Model *Cubes[NUM_SOURCES];
} SceneData;

void Draw3DScene(void *arg)
{
    SceneData *Scene = arg;

    NEA_CameraUse(Scene->Camera);

    for (int i = 0; i < NUM_SOURCES; i++)
        NEA_ModelDraw(Scene->Cubes[i]);
}

int main(int argc, char *argv[])
{
    SceneData Scene = { 0 };

    irqEnable(IRQ_HBLANK);
    irqSet(IRQ_VBLANK, NEA_VBLFunc);
    irqSet(IRQ_HBLANK, NEA_HBLFunc);

    NEA_Init3D();

    NEA_TextureSystemReset(0, 0, NEA_VRAM_AB);
    consoleDemoInit();

    // Initialize sound system with the embedded soundbank
    NEA_SoundSystemReset((mm_addr)soundbank_bin, 8);

    // Load effects
    NEA_SfxLoad(SFX_FIRE_EXPLOSION);
    NEA_SfxLoad(SFX_NATURE);

    // Create camera (the listener)
    Scene.Camera = NEA_CameraCreate();
    NEA_CameraSet(Scene.Camera,
                  0, 3, -10,   // Position
                  0, 3, 0,     // Look at
                  0, 1, 0);    // Up direction

    // Attach the camera as the audio listener
    NEA_SoundSetListener(Scene.Camera);

    // Create four cubes at different positions
    float positions[NUM_SOURCES][3] = {
        { -6, 1, -6 },  // Front-left
        {  6, 1, -6 },  // Front-right
        { -6, 1,  6 },  // Back-left
        {  6, 1,  6 },  // Back-right
    };

    NEA_SoundSource *sources[NUM_SOURCES];

    for (int i = 0; i < NUM_SOURCES; i++)
    {
        Scene.Cubes[i] = NEA_ModelCreate(NEA_Static);
        NEA_ModelLoadStaticMesh(Scene.Cubes[i], cube_bin);
        NEA_ModelSetCoord(Scene.Cubes[i],
                          positions[i][0],
                          positions[i][1],
                          positions[i][2]);

        // Alternate between two different sound effects
        mm_word sfx_id = (i % 2 == 0) ? SFX_FIRE_EXPLOSION : SFX_NATURE;
        sources[i] = NEA_SoundSourceCreate(sfx_id);
        NEA_SoundSourceSetModel(sources[i], Scene.Cubes[i]);
        NEA_SoundSourceSetDistance(sources[i], 2.0, 15.0);
        NEA_SoundSourceSetVolume(sources[i], 255);
        NEA_SoundSourceSetLoop(sources[i], true);
        // Set loop delay to match approximate sample duration
        NEA_SoundSourceSetLoopDelay(sources[i],
                                     (i % 2 == 0) ? 90 : 180);
        NEA_SoundSourcePlay(sources[i]);
    }

    printf("Spatial Sound Example\n");
    printf("=====================\n\n");
    printf("D-Pad: Move camera\n");
    printf("L/R:   Rotate camera\n");
    printf("A:     Move forward\n");
    printf("B:     Move backward\n\n");
    printf("Walk around the cubes to\n");
    printf("hear spatial panning and\n");
    printf("distance attenuation.\n\n");
    printf("START: Return to loader\n");

    while (1)
    {
        NEA_WaitForVBL(NEA_UPDATE_ANIMATIONS | NEA_UPDATE_SOUND);

        NEA_ProcessArg(Draw3DScene, &Scene);

        scanKeys();
        uint16_t keys = keysHeld();

        // Camera movement (forward, right, up)
        if (keys & KEY_UP)
            NEA_CameraMoveFree(Scene.Camera, 0.15, 0, 0);
        if (keys & KEY_DOWN)
            NEA_CameraMoveFree(Scene.Camera, -0.15, 0, 0);
        if (keys & KEY_LEFT)
            NEA_CameraMoveFree(Scene.Camera, 0, -0.15, 0);
        if (keys & KEY_RIGHT)
            NEA_CameraMoveFree(Scene.Camera, 0, 0.15, 0);
        if (keys & KEY_L)
            NEA_CameraRotateFree(Scene.Camera, 0, -64, 0);
        if (keys & KEY_R)
            NEA_CameraRotateFree(Scene.Camera, 0, 64, 0);
        if (keys & KEY_A)
            NEA_CameraMoveFree(Scene.Camera, 0.25, 0, 0);
        if (keys & KEY_B)
            NEA_CameraMoveFree(Scene.Camera, -0.25, 0, 0);

        if (keys & KEY_START)
            break;
    }

    NEA_SoundSystemEnd();

    return 0;
}
