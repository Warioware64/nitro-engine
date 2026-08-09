// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Nitro Engine Advanced contributors, 2026
//
// This file is part of Nitro Engine Advanced

// Example of asynchronous file writing (saving without stalling the game).
//
// A frame counter and a spinner are updated every frame. Watch them to see
// whether the main loop keeps running at 60 FPS while a file is written.
//
//   - Press A to save asynchronously (NEA_FATWriteDataAsync). The data is
//     written in the background and the frame counter keeps advancing.
//
//   - Press B to save synchronously (fwrite in one go). The whole main loop
//     blocks until the write finishes, so the frame counter freezes.
//
//   - Press X to queue a save and then cancel it half way through with
//     NEA_AsyncRelease(). This is the interesting one: the write went to
//     "<path>.temp", so cancelling it leaves the previous save file completely
//     intact instead of truncated. The readback below proves it.
//
//   - Press Y to read the file back and report which payload it holds, and
//     whether a stray ".temp" file was left behind.
//
// Each save writes a different byte over the whole buffer ('A' the first time,
// then 'B', 'C'...), so the readback can tell which save won.
//
// This example needs a writable filesystem, so it calls fatInitDefault()
// instead of nitroFSInit(). In an emulator, point it at a DLDI SD folder or an
// SD image.

#include <stdio.h>

#include <fat.h>
#include <nds/arm9/dldi.h>

#include <NEAMain.h>

#define SAVE_PATH  "nea_async_save.bin"
#define TEMP_PATH  SAVE_PATH ".temp"

// Big enough that the write takes several chunks (and therefore several
// frames), which is what makes the cancellation test meaningful.
#define SAVE_SIZE  (48 * 1024)

// Frames to wait before cancelling the save started with X.
#define CANCEL_AFTER_FRAMES 2

typedef struct {
    int quad_x;
} SceneData;

void Draw3DScene(void *arg)
{
    SceneData *Scene = arg;

    NEA_2DViewInit();
    NEA_2DDrawQuad(Scene->quad_x, 40, Scene->quad_x + 64, 40 + 64, 0, NEA_Red);
}

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

// Reads the file back and reports the byte it is filled with, its size, and
// whether a temporary file was left behind.
static void CheckFile(char *out, size_t out_size)
{
    size_t size = 0;
    char *data = NEA_FATLoadData(SAVE_PATH);
    if (data == NULL)
    {
        snprintf(out, out_size, "no file");
        return;
    }

    size = NEA_FATFileSize(SAVE_PATH);

    // A truncated file would be the failure this example is testing for, so
    // report the size as well as the payload byte.
    char payload = data[0];
    free(data);

    FILE *tmp = fopen(TEMP_PATH, "rb");
    bool temp_left = (tmp != NULL);
    if (tmp != NULL)
        fclose(tmp);

    snprintf(out, out_size, "'%c' %d B%s", payload, (int)size,
             temp_left ? " +TEMP!" : "");
}

int main(int argc, char *argv[])
{
    SceneData Scene = { 0 };

    irqEnable(IRQ_HBLANK);
    irqSet(IRQ_VBLANK, NEA_VBLFunc);
    irqSet(IRQ_HBLANK, NEA_HBLFunc);

    NEA_Init3D();
    NEA_InitConsole();

    // On a DS flashcard, move filesystem access to the ARM7 so that writes can
    // overlap with the main loop. Harmless on DSi and on emulators.
    if (!isDSiMode())
        dldiSetMode(DLDI_MODE_ARM7);

    if (!fatInitDefault())
    {
        printf("fatInitDefault() failed.\n\n");
        printf("This example needs a\n");
        printf("writable filesystem.\n");
        while (1)
            NEA_WaitForVBL(0);
    }

    char *payload = malloc(SAVE_SIZE);
    if (payload == NULL)
    {
        printf("Out of memory\n");
        while (1)
            NEA_WaitForVBL(0);
    }

    NEA_AsyncFile *save = NULL;
    // Frame at which the save started with X has to be cancelled, or -1.
    int cancel_at = -1;
    char next_payload = 'A';
    const char *last_event = "none";
    char file_state[32] = "unknown";
    int frame = 0;
    const char spinner[4] = { '|', '/', '-', '\\' };

    while (1)
    {
        // NEA_UPDATE_ASSETS makes NEA_WaitForVBL() advance asynchronous jobs,
        // writes included.
        NEA_WaitForVBL(NEA_UPDATE_ASSETS);

        NEA_ProcessArg(Draw3DScene, &Scene);

        scanKeys();
        uint32_t keys = keysDown();

        // Animate something every frame so that stutter is easy to see.
        frame++;
        Scene.quad_x = 96 + (((frame >> 1) & 63) - 32);

        if ((keys & KEY_A) && (save == NULL))
        {
            memset(payload, next_payload, SAVE_SIZE);

            // COPY: the engine takes its own copy, so 'payload' can be reused
            // (or freed) as soon as this returns. That is what a game wants for
            // a save state it is about to keep mutating.
            save = NEA_FATWriteDataAsync(SAVE_PATH, payload, SAVE_SIZE,
                                         NEA_ASYNC_WRITE_COPY);
            last_event = save ? "save queued" : "save failed to queue";
            if (save != NULL)
                next_payload++;
        }

        if (keys & KEY_B)
        {
            // Synchronous write for comparison: blocks the main loop.
            memset(payload, next_payload, SAVE_SIZE);
            FILE *f = fopen(SAVE_PATH, "wb");
            if (f != NULL)
            {
                fwrite(payload, 1, SAVE_SIZE, f);
                fclose(f);
                next_payload++;
                last_event = "saved (blocking)";
            }
            else
            {
                last_event = "blocking save failed";
            }
        }

        // Queue a save and cancel it a couple of frames later, while the worker
        // is still writing. The temporary file is deleted and the previous save
        // survives untouched.
        if ((keys & KEY_X) && (save == NULL))
        {
            memset(payload, next_payload, SAVE_SIZE);
            save = NEA_FATWriteDataAsync(SAVE_PATH, payload, SAVE_SIZE,
                                         NEA_ASYNC_WRITE_COPY);
            if (save != NULL)
            {
                next_payload++;
                cancel_at = frame + CANCEL_AFTER_FRAMES;
                last_event = "save queued, will cancel";
            }
        }

        if ((cancel_at >= 0) && (frame >= cancel_at) && (save != NULL))
        {
            NEA_AsyncRelease(save);
            save = NULL;
            cancel_at = -1;
            last_event = "save cancelled";
        }

        if (keys & KEY_Y)
        {
            CheckFile(file_state, sizeof(file_state));
            last_event = "file checked";
        }

        if (save != NULL)
        {
            NEA_AsyncState state = NEA_AsyncGetState(save);
            if (state == NEA_ASYNC_DONE)
            {
                NEA_AsyncRelease(save);
                save = NULL;
                last_event = "saved (async)";
            }
            else if (state == NEA_ASYNC_ERROR)
            {
                NEA_AsyncRelease(save);
                save = NULL;
                last_event = "save failed";
            }
        }

        consoleClear();
        printf("Asynchronous file writing\n");
        printf("=========================\n\n");
        printf("Frame:    %d %c\n\n", frame, spinner[frame & 3]);
        printf("Pending:  %d\n", NEA_AsyncPendingCount());
        printf("Save:     %s\n",
               save ? StateName(NEA_AsyncGetState(save)) : "-");
        printf("On disk:  %s\n", file_state);
        printf("Last:     %s\n\n", last_event);
        printf("A: save async (smooth)\n");
        printf("B: save sync  (blocks!)\n");
        printf("X: save then cancel it\n");
        printf("Y: read the file back\n");
    }

    return 0;
}
