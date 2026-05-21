// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Nitro Engine Advanced contributors, 2026
//
// This file is part of Nitro Engine Advanced

// Example of asynchronous asset loading.
//
// A frame counter and a spinner are updated every frame. Watch them to see
// whether the main loop keeps running at 60 FPS.
//
//   - Press A to load a texture asynchronously (NEA_MaterialTexLoadGRFAsync).
//     The file is read in the background and the frame counter keeps advancing.
//
//   - Press B to load the same texture synchronously (NEA_MaterialTexLoadGRF).
//     The whole main loop blocks until the load finishes, so the frame counter
//     freezes for a moment.
//
// The finalize step (uploading the texture to VRAM) is run automatically by
// NEA_WaitForVBL() because NEA_UPDATE_ASSETS is passed to it.
//
// Important: On a DS flashcard, filesystem access through DLDI runs on the
// ARM9 by default and blocks it during reads. For the loading to truly overlap
// with the main loop, call dldiSetMode(DLDI_MODE_ARM7) before nitroFSInit().
// This is not needed on DSi (SD card) or when reading from a cartridge.

#include <filesystem.h>
#include <nds/arm9/dldi.h>

#include <NEAMain.h>

#define TEXTURE_PATH "a1rgb5_png.grf"

typedef struct {
    NEA_Material *material;
    bool textured;
    int quad_x;
} SceneData;

void Draw3DScene(void *arg)
{
    SceneData *Scene = arg;

    NEA_2DViewInit();

    if (Scene->textured)
    {
        NEA_2DDrawTexturedQuad(Scene->quad_x, 56,
                               Scene->quad_x + 128, 56 + 128,
                               0, Scene->material);
    }
}

int main(int argc, char *argv[])
{
    SceneData Scene = { 0 };

    irqEnable(IRQ_HBLANK);
    irqSet(IRQ_VBLANK, NEA_VBLFunc);
    irqSet(IRQ_HBLANK, NEA_HBLFunc);

    NEA_Init3D();
    NEA_TextureSystemReset(0, 0, NEA_VRAM_ABC);
    NEA_InitConsole();

    // On a DS flashcard, move filesystem access to the ARM7 so that reads can
    // overlap with the main loop. Harmless on DSi and on emulators.
    if (!isDSiMode())
        dldiSetMode(DLDI_MODE_ARM7);

    if (!nitroFSInit(NULL))
    {
        printf("nitroFSInit() failed\n");
        while (1)
            NEA_WaitForVBL(0);
    }

    Scene.material = NEA_MaterialCreate();

    NEA_AsyncFile *load = NULL;
    int frame = 0;
    const char spinner[4] = { '|', '/', '-', '\\' };

    while (1)
    {
        // NEA_UPDATE_ASSETS makes NEA_WaitForVBL() advance async loads and run
        // their finalize steps (the texture VRAM upload) during the VBL.
        NEA_WaitForVBL(NEA_UPDATE_ASSETS);

        NEA_ProcessArg(Draw3DScene, &Scene);

        scanKeys();
        uint32_t keys = keysDown();

        // Animate something every frame so that stutter is easy to see.
        frame++;
        Scene.quad_x = 64 + (((frame >> 1) & 63) - 32);

        if ((keys & KEY_A) && (load == NULL) && !Scene.textured)
        {
            load = NEA_MaterialTexLoadGRFAsync(Scene.material, NULL,
                                               NEA_TEXGEN_TEXCOORD,
                                               TEXTURE_PATH);
        }

        if ((keys & KEY_B) && !Scene.textured)
        {
            // Synchronous load: this blocks the main loop until it finishes.
            if (NEA_MaterialTexLoadGRF(Scene.material, NULL,
                                       NEA_TEXGEN_TEXCOORD, TEXTURE_PATH))
            {
                Scene.textured = true;
            }
        }

        // Poll the asynchronous load.
        if (load != NULL)
        {
            NEA_AsyncState state = NEA_AsyncGetState(load);
            if (state == NEA_ASYNC_DONE)
            {
                Scene.textured = true;
                NEA_AsyncRelease(load);
                load = NULL;
            }
            else if (state == NEA_ASYNC_ERROR)
            {
                printf("Async load failed!\n");
                NEA_AsyncRelease(load);
                load = NULL;
            }
        }

        consoleClear();
        printf("Asynchronous asset loading\n");
        printf("==========================\n\n");
        printf("Frame:    %d %c\n\n", frame, spinner[frame & 3]);
        printf("Pending:  %d\n", NEA_AsyncPendingCount());
        printf("Textured: %s\n\n", Scene.textured ? "yes" : "no");
        printf("A: load async (smooth)\n");
        printf("B: load sync  (blocks!)\n");
    }

    return 0;
}
