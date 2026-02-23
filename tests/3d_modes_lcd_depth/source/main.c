// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Antonio Niño Díaz, 2023-2024
//
// This file is part of Nitro Engine Advanced

// The LCD of the DS actually supports 6 bits for each one of the RGB components
// of a color. You can't access the least significative bits by software
// because all colors are defined with 5 bits per component. However, the 6th
// bit is connected to the video unit of the DS, and the 3D GPU also uses that
// additional bit when rendering.
//
// However, when video is captured (which is used in dual 3D modes) the last bit
// is lost. This demo shows the difference between different 3D modes. In order
// to do that it draws a colored quad that goes from a gray to a slightly
// different gray. This quad spans the whole screen, which means that there is a
// lot of space to interpolate. This makes the gradient steps easier to be seen
// by the human eye. Also, note that this is easier to see in a DSi than in a
// regular DS.
//
// This special output of this is demo can't be seen in most emulators (like
// DesMuME or no$gba). It works in melonDS and hardware.
//
// - Single 3D:
//
// The screen shows the direct output from the GPU (6 bits).
//
// - Dual 3D:
//
// The screens alternate between showing the direct output from the GPU (6 bit),
// or a stored image in VRAM (5 bit). This means that the output will be seen as
// something in between the two bit depths. It may end up blended or appear as a
// small flicker.
//
// - Dual 3D FB:
//
// Both screens show captured output that has been stored in VRAM, so the output
// is always using 5 bit depth images.
//
// The debug console doesn't work in this mode
//
// - Dual 3D DMA:
//
// Both screens show captured output that has been stored in VRAM, so the output
// is always using 5 bit depth images.

#include <NEAMain.h>

#define ID_1 10
#define ID_2 11

typedef struct {
    uint32_t x;
} SceneData;

void Draw3DSceneBands(void *arg)
{
    SceneData *Scene = arg;

    NEA_ClearColorSet(NEA_Black, 31, 63);

    NEA_2DViewInit();

    NEA_PolyFormat(31, ID_1, 0, NEA_CULL_NONE, 0);

    NEA_2DDrawQuadGradient(0, 0, 256, 192,
                          10,
                          RGB15(16, 16, 16), RGB15(12, 12, 12),
                          RGB15(12, 12, 12), RGB15(16, 16, 16));

    NEA_PolyFormat(15, ID_2, 0, NEA_CULL_NONE, 0);

    NEA_2DDrawQuadGradient(0 + Scene->x, 128, 256 + Scene->x, 192,
                          9,
                          RGB15(16, 16, 16), RGB15(12, 12, 12),
                          RGB15(12, 12, 12), RGB15(16, 16, 16));
}

void Draw3DSceneEmpty(void *arg)
{
    (void)arg;

    NEA_ClearColorSet(NEA_Black, 31, 63);
}

int main(int argc, char *argv[])
{
    SceneData Scene = { 0 };

    // This is needed for special screen effects
    irqEnable(IRQ_HBLANK);
    irqSet(IRQ_VBLANK, NEA_VBLFunc);
    irqSet(IRQ_HBLANK, NEA_HBLFunc);

    // Init 3D mode and console
    NEA_Init3D();
    NEA_MainScreenSetOnTop();
    consoleDemoInit();

    Scene.x = 80;

    while (1)
    {
        NEA_WaitForVBL(0);

        // Draw 3D scenes
        switch (NEA_CurrentExecutionMode())
        {
            case NEA_ModeSingle3D:
                NEA_ProcessArg(Draw3DSceneBands, &Scene);
                break;

            case NEA_ModeDual3D:
            case NEA_ModeDual3D_FB:
            case NEA_ModeDual3D_DMA:
                NEA_ProcessDualArg(Draw3DSceneBands, Draw3DSceneEmpty, &Scene, &Scene);
                break;

            case NEA_ModeUninitialized:
                break;
        }

        // Refresh keys
        scanKeys();
        uint32_t keys = keysHeld();
        uint32_t kdown = keysDown();

        if (NEA_CurrentExecutionMode() != NEA_ModeDual3D_FB)
        {
            printf("\x1b[0;0H"
                   "A: One screen 3D (6 bit)\n"
                   "B: Dual 3D DMA (5 bit)\n"
                   "X: Dual 3D FB (5 bit, no text)\n"
                   "Y: Dual 3D (both 5 and 6 bit)\n"
                   "Left/Right: Move layer\n"
                   "\n"
                   "6 bit modes will display more\n"
                   "bands than 5 bit modes.\n"
                   "\n"
                   "Dual 3D mode alternates 5 and 6\n"
                   "bits, so it shows a blend or\n"
                   "flicker between more and fewer\n"
                   "color bands.\n"
                   "\n"
                   "Only melonDS emulates this.\n"
                   "\n"
                   "Note: Dual 3D FB disables the\n"
                   "text console\n"
                   "\n"
                   "\n"
                   "\n");

            const char *modes[] = {
                [NEA_ModeSingle3D] = "Single 3D",
                [NEA_ModeDual3D] = "Dual 3D",
                [NEA_ModeDual3D_FB] = "Dual 3D FB",
                [NEA_ModeDual3D_DMA] = "Dual 3D DMA"
            };
            printf("Current mode: %s", modes[NEA_CurrentExecutionMode()]);
        }

        if (keys & KEY_LEFT)
            Scene.x--;
        if (keys & KEY_RIGHT)
            Scene.x++;

        if (kdown & KEY_Y)
        {
            NEA_InitDual3D();
            NEA_MainScreenSetOnBottom();
            NEA_InitConsole();
        }
        if (kdown & KEY_X)
        {
            NEA_InitDual3D_FB();
            NEA_MainScreenSetOnBottom();
        }
        if (kdown & KEY_B)
        {
            NEA_InitDual3D_DMA();
            NEA_MainScreenSetOnBottom();
            NEA_InitConsole();
        }
        if (kdown & KEY_A)
        {
            NEA_Init3D();
            NEA_MainScreenSetOnTop();
            consoleDemoInit();
        }
    }

    return 0;
}
