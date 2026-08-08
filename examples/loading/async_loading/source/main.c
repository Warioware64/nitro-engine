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
//   - Press X to queue a texture and a palette at the same time
//     (NEA_MaterialTexLoadFATAsync + NEA_PaletteLoadFATAsync). They share a
//     single worker, so they are read one after the other while the main loop
//     keeps running. Watch the pending counter go 2 -> 1 -> 0.
//
//   - Press Y to delete the material while a load into it is still pending.
//     The load is aborted instead of writing into freed memory: it reports
//     NEA_ASYNC_ERROR and the program keeps running.
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
#define SPIRAL_TEX_PATH "spiral_red_pal32.img.bin"
#define SPIRAL_PAL_PATH "spiral_red_pal32.pal.bin"

typedef struct {
    NEA_Material *material;
    NEA_Material *spiral;
    bool textured;
    bool spiral_ready;
    int quad_x;
} SceneData;

void Draw3DScene(void *arg)
{
    SceneData *Scene = arg;

    NEA_2DViewInit();

    if (Scene->textured)
    {
        NEA_2DDrawTexturedQuad(Scene->quad_x, 16,
                               Scene->quad_x + 96, 16 + 96,
                               0, Scene->material);
    }

    if (Scene->spiral_ready)
    {
        NEA_2DDrawTexturedQuad(96, 120, 96 + 64, 120 + 64,
                               0, Scene->spiral);
    }
}

// Prints the name of a state so that the abort case is easy to see on screen.
static const char *StateName(NEA_AsyncState state)
{
    switch (state)
    {
        case NEA_ASYNC_PENDING: return "pending";
        case NEA_ASYNC_READY:   return "ready";
        case NEA_ASYNC_DONE:    return "done";
        default:                return "ERROR";
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
    Scene.spiral = NEA_MaterialCreate();
    NEA_Palette *SpiralPalette = NEA_PaletteCreate();

    NEA_AsyncFile *load = NULL;
    NEA_AsyncFile *tex_load = NULL;
    NEA_AsyncFile *pal_load = NULL;
    const char *last_event = "none";
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

        // Two loads queued at once. The worker runs them one after the other,
        // and the palette is uploaded to VRAM by NEA_AsyncProcess() just like
        // the texture.
        if ((keys & KEY_X) && (tex_load == NULL) && (pal_load == NULL)
            && !Scene.spiral_ready)
        {
            tex_load = NEA_MaterialTexLoadFATAsync(Scene.spiral, NEA_A3PAL32,
                                                   64, 64, NEA_TEXGEN_TEXCOORD,
                                                   SPIRAL_TEX_PATH);
            pal_load = NEA_PaletteLoadFATAsync(SpiralPalette, SPIRAL_PAL_PATH,
                                               NEA_A3PAL32);
        }

        // Deleting the target of a pending load aborts it. The handle stays
        // valid and reports NEA_ASYNC_ERROR, so this is safe at any time.
        if ((keys & KEY_Y) && (load != NULL))
        {
            NEA_MaterialDelete(Scene.material);
            Scene.material = NULL;
            Scene.textured = false;
            last_event = "material deleted";
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
                last_event = "texture loaded";
            }
            else if (state == NEA_ASYNC_ERROR)
            {
                NEA_AsyncRelease(load);
                load = NULL;
                if (Scene.material != NULL)
                    last_event = "load failed";
                else
                    last_event = "load aborted (safe)";
            }
        }

        if (tex_load != NULL && NEA_AsyncGetState(tex_load) != NEA_ASYNC_PENDING
            && NEA_AsyncGetState(tex_load) != NEA_ASYNC_READY)
        {
            NEA_AsyncRelease(tex_load);
            tex_load = NULL;
        }

        if (pal_load != NULL)
        {
            NEA_AsyncState state = NEA_AsyncGetState(pal_load);
            if (state == NEA_ASYNC_DONE)
            {
                // Both halves are in VRAM now, so the material can be drawn.
                NEA_MaterialTexSetPal(Scene.spiral, SpiralPalette);
                Scene.spiral_ready = true;
                NEA_AsyncRelease(pal_load);
                pal_load = NULL;
                last_event = "tex + palette loaded";
            }
            else if (state == NEA_ASYNC_ERROR)
            {
                NEA_AsyncRelease(pal_load);
                pal_load = NULL;
                last_event = "palette failed";
            }
        }

        consoleClear();
        printf("Asynchronous asset loading\n");
        printf("==========================\n\n");
        printf("Frame:    %d %c\n\n", frame, spinner[frame & 3]);
        printf("Pending:  %d\n", NEA_AsyncPendingCount());
        printf("GRF:      %s\n",
               load ? StateName(NEA_AsyncGetState(load)) : "-");
        printf("Last:     %s\n\n", last_event);
        printf("A: load async (smooth)\n");
        printf("B: load sync  (blocks!)\n");
        printf("X: tex + palette async\n");
        printf("Y: delete target mid-load\n");
    }

    return 0;
}
