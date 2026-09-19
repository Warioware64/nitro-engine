// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Warioware64, 2026
//
// This file is part of Nitro Engine Advanced
//
// Music and SFX example: demonstrates background music playback with volume,
// tempo, and pitch controls, plus non-spatial sound effects triggered by
// buttons.
//
// It also drives the DSi extended audio output (SNDEXCNT): SELECT switches the
// I2S rate between 32.73 and 47.61 kHz, and SELECT plus L/R sweeps the DSP/ARM
// mix ratio, which doubles as a master volume for Maxmod.
//
// The mix ratio is the control to try first: sweeping it from 8 down to 0 fades
// the music to silence and back, which is unmistakable. The rate switch is
// deliberately subtle -- expect a short gap while the codec PLL relocks and
// then the same music at the same pitch and tempo. Pitch or tempo moving would
// mean something is wrong.
//
// On a DS, or from a DS-only ROM on a DSi, the readout says the output is not
// available and both controls do nothing.

#include <NEAMain.h>

#include "soundbank.h"
#include "soundbank_bin.h"
#include "cube_bin.h"

typedef struct {
    NEA_Camera *Camera;
    NEA_Model *Model;
} SceneData;

void Draw3DScene(void *arg)
{
    SceneData *Scene = arg;

    NEA_CameraUse(Scene->Camera);
    NEA_ModelDraw(Scene->Model);
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

    // Ask for the DSi extended output rate before initializing sound. The
    // request is remembered and re-applied by NEA_SoundSystemReset(), and this
    // is the only moment at which nothing is playing -- which is what changing
    // the I2S rate wants. This is a no-op on a DS.
    NEA_SoundEnableDSiOutput(NEA_DSI_SOUND_FREQ_47KHZ);

    // Initialize sound system with the embedded soundbank
    NEA_SoundSystemReset((mm_addr)soundbank_bin, 4);

    bool dsi_audio = NEA_SoundDSiOutputAvailable();

    // Load the SFX
    NEA_SfxLoad(SFX_FIRE_EXPLOSION);

    // Create a simple 3D scene
    Scene.Camera = NEA_CameraCreate();
    NEA_CameraSet(Scene.Camera,
                  -6, 3, 0,
                  0, 1, 0,
                  0, 1, 0);

    Scene.Model = NEA_ModelCreate(NEA_Static);
    NEA_ModelLoadStaticMesh(Scene.Model, cube_bin);

    // Start background music (looping)
    NEA_MusicStart(MOD_JOINT_PEOPLE, MM_PLAY_LOOP);

    int music_volume = 512;   // 0-1024
    int music_tempo = 1024;   // 512-2048
    int music_pitch = 1024;   // 512-2048
    bool music_paused = false;

    printf("Music & SFX Example\n");
    printf("===================\n\n");
    printf("A:     Play explosion SFX\n");
    printf("B:     Stop all SFX\n");
    printf("X:     Pause/Resume music\n");
    printf("Y:     Stop/Restart music\n\n");
    printf("Up/Dn: Volume +/-\n");
    printf("L/R:   Tempo +/-\n");
    printf("Left/Right: Pitch +/-\n\n");
    printf("SELECT:     Toggle DSi rate\n");
    printf("SELECT+L/R: DSi mix ratio\n");
    printf("START: Return to loader\n");

    int ry = 0;

    while (1)
    {
        NEA_WaitForVBL(0);

        // Rotate the cube
        ry = (ry + 2) & 0x1FF;
        NEA_ModelSetRot(Scene.Model, 0, ry, 0);

        NEA_ProcessArg(Draw3DScene, &Scene);

        scanKeys();
        uint16_t keys_down = keysDown();
        uint16_t keys_held = keysHeld();

        // Play explosion SFX (non-spatial)
        if (keys_down & KEY_A)
        {
            mm_sfxhand h = NEA_SfxPlay(SFX_FIRE_EXPLOSION);
            NEA_SfxSetRate(h, 512 + (rand() & 1023));
        }

        // Cancel all SFX
        if (keys_down & KEY_B)
            NEA_SfxStopAll();

        // Pause / Resume music
        if (keys_down & KEY_X)
        {
            if (music_paused)
                NEA_MusicResume();
            else
                NEA_MusicPause();
            music_paused = !music_paused;
        }

        // Stop / Restart music
        if (keys_down & KEY_Y)
        {
            if (NEA_MusicIsPlaying())
            {
                NEA_MusicStop();
            }
            else
            {
                NEA_MusicStart(MOD_JOINT_PEOPLE, MM_PLAY_LOOP);
                music_paused = false;
            }
        }

        // Volume control
        if (keys_held & KEY_UP)
        {
            music_volume += 8;
            if (music_volume > 1024)
                music_volume = 1024;
            NEA_MusicSetVolume(music_volume);
        }
        if (keys_held & KEY_DOWN)
        {
            music_volume -= 8;
            if (music_volume < 0)
                music_volume = 0;
            NEA_MusicSetVolume(music_volume);
        }

        // DSi extended output. SELECT alone toggles the I2S rate; SELECT
        // with L/R sweeps the DSP/ARM mix ratio, which is a master volume for
        // Maxmod because Maxmod is the ARM side of that ratio.
        if (keys_down & KEY_SELECT)
        {
            NEA_DSiSoundFreq next =
                (NEA_SoundGetDSiOutputFreq() == NEA_DSI_SOUND_FREQ_32KHZ) ?
                NEA_DSI_SOUND_FREQ_47KHZ : NEA_DSI_SOUND_FREQ_32KHZ;

            // The rate change reprograms the codec PLL, so stop everything
            // Maxmod is driving first. NEA won't do it for you: it can't
            // restart a module where it left off.
            bool was_paused = music_paused;
            NEA_MusicPause();
            NEA_SfxStopAll();

            NEA_SoundEnableDSiOutput(next);

            if (!was_paused)
                NEA_MusicResume();
        }

        if (keys_held & KEY_SELECT)
        {
            if (keys_down & KEY_L)
                NEA_SoundSetDSiMixRatio(NEA_SoundGetDSiMixRatio() - 1);
            if (keys_down & KEY_R)
                NEA_SoundSetDSiMixRatio(NEA_SoundGetDSiMixRatio() + 1);
        }

        // Tempo control
        if ((keys_held & KEY_L) && !(keys_held & KEY_SELECT))
        {
            music_tempo -= 8;
            if (music_tempo < 512)
                music_tempo = 512;
            NEA_MusicSetTempo(music_tempo);
        }
        if ((keys_held & KEY_R) && !(keys_held & KEY_SELECT))
        {
            music_tempo += 8;
            if (music_tempo > 2048)
                music_tempo = 2048;
            NEA_MusicSetTempo(music_tempo);
        }

        // Pitch control
        if (keys_held & KEY_LEFT)
        {
            music_pitch -= 8;
            if (music_pitch < 512)
                music_pitch = 512;
            NEA_MusicSetPitch(music_pitch);
        }
        if (keys_held & KEY_RIGHT)
        {
            music_pitch += 8;
            if (music_pitch > 2048)
                music_pitch = 2048;
            NEA_MusicSetPitch(music_pitch);
        }

        // Print status on bottom screen
        printf("\x1b[17;0H");
        printf("Vol: %4d / 1024  %s\n", music_volume,
               music_paused ? "[PAUSED]" : "        ");
        printf("Tempo: %4d  Pitch: %4d\n", music_tempo, music_pitch);
        printf("Music: %s\n", NEA_MusicIsPlaying() ? "Playing" : "Stopped");
        if (dsi_audio)
        {
            printf("DSi: %s  mix %d/8\n",
                   NEA_SoundGetDSiOutputFreq() == NEA_DSI_SOUND_FREQ_47KHZ ?
                   "47.61kHz" : "32.73kHz",
                   NEA_SoundGetDSiMixRatio());
        }
        else
        {
            printf("DSi: output not available\n");
        }

        if (keys_down & KEY_START)
            break;
    }

    NEA_MusicStop();
    NEA_SoundSystemEnd();

    return 0;
}
