// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Warioware64, 2026
//
// This file is part of Nitro Engine Advanced
//
// Spatial sound example: four cubes placed in the scene emit positioned sound
// effects. The listener is attached to the camera, so walking around makes the
// volume and panning change in real time.
//
// On a DSi it also drives the extended audio output (SNDEXCNT): SELECT switches
// the I2S rate, SELECT plus L/R sweeps the DSP/ARM mix ratio. The rate switch
// stops and restarts every spatial source first, which is what the header asks
// for -- the codec PLL is reprogrammed and nothing should be playing across it.
// On a DS the readout says the output is not available and both do nothing.

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

    // Ask for the DSi extended output rate before initializing sound: the
    // request is remembered and re-applied by NEA_SoundSystemReset(), and
    // nothing is playing yet. A no-op on a DS.
    NEA_SoundEnableDSiOutput(NEA_DSI_SOUND_FREQ_47KHZ);

    // Initialize sound system with the embedded soundbank
    NEA_SoundSystemReset((mm_addr)soundbank_bin, 8);

    bool dsi_audio = NEA_SoundDSiOutputAvailable();

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
    printf("SELECT:     DSi rate\n");
    printf("SELECT+L/R: DSi mix ratio\n\n");
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
        uint16_t keys_down = keysDown();

        // DSi extended output. SELECT is a modifier, so the camera controls
        // below ignore L/R while it is held.
        if (keys_down & KEY_SELECT)
        {
            NEA_DSiSoundFreq next =
                (NEA_SoundGetDSiOutputFreq() == NEA_DSI_SOUND_FREQ_32KHZ) ?
                NEA_DSI_SOUND_FREQ_47KHZ : NEA_DSI_SOUND_FREQ_32KHZ;

            // Nothing may be playing across the codec reprogram, and with
            // looping spatial sources that means stopping them explicitly --
            // they re-trigger on a frame counter, so letting the effects
            // expire on their own wouldn't be enough.
            for (int i = 0; i < NUM_SOURCES; i++)
                NEA_SoundSourceStop(sources[i]);

            NEA_SoundEnableDSiOutput(next);

            for (int i = 0; i < NUM_SOURCES; i++)
                NEA_SoundSourcePlay(sources[i]);
        }

        if (keys & KEY_SELECT)
        {
            if (keys_down & KEY_L)
                NEA_SoundSetDSiMixRatio(NEA_SoundGetDSiMixRatio() - 1);
            if (keys_down & KEY_R)
                NEA_SoundSetDSiMixRatio(NEA_SoundGetDSiMixRatio() + 1);
        }

        // Camera movement (forward, right, up)
        if (keys & KEY_UP)
            NEA_CameraMoveFree(Scene.Camera, 0.15, 0, 0);
        if (keys & KEY_DOWN)
            NEA_CameraMoveFree(Scene.Camera, -0.15, 0, 0);
        if (keys & KEY_LEFT)
            NEA_CameraMoveFree(Scene.Camera, 0, -0.15, 0);
        if (keys & KEY_RIGHT)
            NEA_CameraMoveFree(Scene.Camera, 0, 0.15, 0);
        if ((keys & KEY_L) && !(keys & KEY_SELECT))
            NEA_CameraRotateFree(Scene.Camera, 0, -64, 0);
        if ((keys & KEY_R) && !(keys & KEY_SELECT))
            NEA_CameraRotateFree(Scene.Camera, 0, 64, 0);
        if (keys & KEY_A)
            NEA_CameraMoveFree(Scene.Camera, 0.25, 0, 0);
        if (keys & KEY_B)
            NEA_CameraMoveFree(Scene.Camera, -0.25, 0, 0);

        if (dsi_audio)
        {
            printf("\x1b[21;0HDSi: %s  mix %d/8\n",
                   NEA_SoundGetDSiOutputFreq() == NEA_DSI_SOUND_FREQ_47KHZ ?
                   "47.61kHz" : "32.73kHz",
                   NEA_SoundGetDSiMixRatio());
        }
        else
        {
            printf("\x1b[21;0HDSi: output not available\n");
        }

        if (keys & KEY_START)
            break;
    }

    NEA_SoundSystemEnd();

    return 0;
}
