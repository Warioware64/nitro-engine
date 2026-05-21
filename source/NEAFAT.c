// SPDX-License-Identifier: MIT
//
// Copyright (c) 2008-2022 Antonio Niño Díaz
//
// This file is part of Nitro Engine Advanced

#include <nds/cothread.h>

#include "NEAMain.h"

/// @file NEAFAT.c

// Size of each chunk read by the asynchronous worker. The worker yields to
// other threads after every chunk, so a smaller value gives a smoother main
// loop at the cost of slightly more overhead.
#define NEA_ASYNC_CHUNK_SIZE (8 * 1024)

// Stack size of the asynchronous worker thread. It must be big enough for
// filesystem access and GRF decompression.
#define NEA_ASYNC_STACK_SIZE (16 * 1024)

// Forward declaration. The full definition is below, after struct NEA_AsyncFile.
static bool ne_async_is_cancelled(const NEA_AsyncFile *job);

// Reads a whole file into a freshly allocated buffer.
//
// If 'job' is NULL the file is read in a single blocking call. If 'job' is not
// NULL the file is read in chunks, yielding to other cothreads between chunks,
// and the read is aborted if job->cancelled becomes true.
static char *ne_fat_read_file(const char *filename, size_t *size_out,
                              NEA_AsyncFile *job)
{
    FILE *f = fopen(filename, "rb");
    if (f == NULL)
    {
        NEA_DebugPrint("%s could't be opened", filename);
        return NULL;
    }

    if (fseek(f, 0, SEEK_END) != 0)
    {
        NEA_DebugPrint("Failed to fseek: %s", filename);
        fclose(f);
        return NULL;
    }

    size_t size = ftell(f);
    rewind(f);

    char *buffer = malloc(size);
    if (buffer == NULL)
    {
        NEA_DebugPrint("Not enought memory to load %s", filename);
        fclose(f);
        return NULL;
    }

    if (job == NULL)
    {
        if (fread(buffer, 1, size, f) != size)
        {
            NEA_DebugPrint("Failed to read data of %s", filename);
            free(buffer);
            fclose(f);
            return NULL;
        }
    }
    else
    {
        // Chunked read so that the worker thread yields regularly and the main
        // loop keeps running while a large file is loaded.
        size_t done = 0;
        while (done < size)
        {
            if (ne_async_is_cancelled(job))
            {
                free(buffer);
                fclose(f);
                return NULL;
            }

            size_t chunk = size - done;
            if (chunk > NEA_ASYNC_CHUNK_SIZE)
                chunk = NEA_ASYNC_CHUNK_SIZE;

            if (fread(buffer + done, 1, chunk, f) != chunk)
            {
                NEA_DebugPrint("Failed to read data of %s", filename);
                free(buffer);
                fclose(f);
                return NULL;
            }

            done += chunk;
            cothread_yield();
        }
    }

    fclose(f);

    if (size_out != NULL)
        *size_out = size;

    return buffer;
}

char *NEA_FATLoadData(const char *filename)
{
    return ne_fat_read_file(filename, NULL, NULL);
}

size_t NEA_FATFileSize(const char *filename)
{
    FILE *f = fopen(filename, "rb");
    if (f == NULL)
    {
        NEA_DebugPrint("%s could't be opened", filename);
        return -1;
    }

    if (fseek(f, 0, SEEK_END) != 0)
    {
        NEA_DebugPrint("Failed to fseek: %s", filename);
        fclose(f);
        return -1;
    }

    size_t size = ftell(f);
    fclose(f);
    return size;
}

//--------------------------------------------------------------------------
// Asynchronous loading
//--------------------------------------------------------------------------

struct NEA_AsyncFile {
    char *filename;             // File to load (owned)
    char *buffer;               // File contents in RAM (owned if buffer_owned)
    size_t size;                // Size of buffer

    volatile NEA_AsyncState state;
    volatile bool cancelled;    // Set by NEA_AsyncRelease() while in progress
    bool buffer_owned;          // True while this handle owns 'buffer'
    bool finalized;             // True once finalize() has run

    int result;                 // Result of finalize() (1 = success)

    // Optional second-stage processing, run in the worker thread right after
    // the file has been read (used to decode GRF files).
    __NEA_AsyncWorkerFn worker_stage2;
    // Optional finalize step, run on the main thread by NEA_AsyncProcess().
    __NEA_AsyncFinalizeFn finalize;
    // Optional cleanup, run when a handle is destroyed before finalize() runs.
    __NEA_AsyncFinalizeFn discard;
    void *param;                // Module-specific parameters (owned)

    NEA_AsyncCallback user_cb;
    void *user_data;

    NEA_AsyncFile *next;        // Next handle in the global list
};

// List of all live async handles.
static NEA_AsyncFile *ne_async_list = NULL;
// Handle whose worker thread is currently running (NULL if none).
static NEA_AsyncFile *ne_async_active = NULL;
// Thread ID of the active worker.
static cothread_t ne_async_thread = -1;

// Used by ne_fat_read_file() to poll the cancel flag without exposing the
// internals of NEA_AsyncFile to that helper.
static bool ne_async_is_cancelled(const NEA_AsyncFile *job)
{
    return job->cancelled;
}

// Worker thread entrypoint. Reads the file and runs the optional second-stage
// processing. Runs in its own cothread.
static int ne_async_worker_entry(void *arg)
{
    NEA_AsyncFile *job = arg;

    if (job->cancelled)
        return 0;

    size_t size = 0;
    char *buffer = ne_fat_read_file(job->filename, &size, job);
    if (buffer == NULL)
    {
        if (!job->cancelled)
            job->state = NEA_ASYNC_ERROR;
        return 0;
    }

    job->buffer = buffer;
    job->size = size;
    job->buffer_owned = true;

    if (job->cancelled)
        return 0;

    if (job->worker_stage2 != NULL)
    {
        if (!job->worker_stage2(job))
        {
            job->state = NEA_ASYNC_ERROR;
            return 0;
        }
    }

    job->state = NEA_ASYNC_READY;
    return 0;
}

// Starts the worker thread for the next pending handle, if the worker is idle.
static void ne_async_try_start(void)
{
    if (ne_async_active != NULL)
        return;

    for (NEA_AsyncFile *job = ne_async_list; job != NULL; job = job->next)
    {
        if (job->state != NEA_ASYNC_PENDING || job->cancelled)
            continue;

        cothread_t thread = cothread_create(ne_async_worker_entry, job,
                                            NEA_ASYNC_STACK_SIZE, 0);
        if (thread == -1)
        {
            // Not enough memory for the thread now. Try again on a later call.
            NEA_DebugPrint("Couldn't create async worker thread");
            return;
        }

        ne_async_thread = thread;
        ne_async_active = job;
        return;
    }
}

// Appends a handle to the end of the global list.
static void ne_async_append(NEA_AsyncFile *job)
{
    job->next = NULL;

    if (ne_async_list == NULL)
    {
        ne_async_list = job;
        return;
    }

    NEA_AsyncFile *tail = ne_async_list;
    while (tail->next != NULL)
        tail = tail->next;
    tail->next = job;
}

// Removes a handle from the global list.
static void ne_async_unlink(NEA_AsyncFile *job)
{
    if (ne_async_list == job)
    {
        ne_async_list = job->next;
        return;
    }

    for (NEA_AsyncFile *it = ne_async_list; it != NULL; it = it->next)
    {
        if (it->next == job)
        {
            it->next = job->next;
            return;
        }
    }
}

// Frees a handle and all the memory it owns.
static void ne_async_destroy(NEA_AsyncFile *job)
{
    // If the module-specific finalize step never ran, let the module free any
    // allocations it made (for example, decoded GRF buffers).
    if (!job->finalized && job->discard != NULL)
        job->discard(job);

    if (job->buffer != NULL && job->buffer_owned)
        free(job->buffer);

    free(job->param);
    free(job->filename);
    free(job);
}

NEA_AsyncFile *__NEA_AsyncQueue(const char *filename,
                                __NEA_AsyncWorkerFn worker_stage2,
                                __NEA_AsyncFinalizeFn finalize,
                                __NEA_AsyncFinalizeFn discard,
                                void *param)
{
    NEA_AssertPointer(filename, "NULL filename pointer");

    NEA_AsyncFile *job = calloc(1, sizeof(NEA_AsyncFile));
    if (job == NULL)
    {
        NEA_DebugPrint("Not enough memory");
        return NULL;
    }

    size_t len = strlen(filename) + 1;
    job->filename = malloc(len);
    if (job->filename == NULL)
    {
        NEA_DebugPrint("Not enough memory");
        free(job);
        return NULL;
    }
    memcpy(job->filename, filename, len);

    job->state = NEA_ASYNC_PENDING;
    job->worker_stage2 = worker_stage2;
    job->finalize = finalize;
    job->discard = discard;
    job->param = param;

    ne_async_append(job);
    ne_async_try_start();

    return job;
}

NEA_AsyncFile *NEA_FATLoadDataAsync(const char *filename)
{
    return __NEA_AsyncQueue(filename, NULL, NULL, NULL, NULL);
}

void NEA_AsyncProcess(void)
{
    // Reap the active worker thread if it has finished.
    if (ne_async_active != NULL && cothread_has_joined(ne_async_thread))
    {
        cothread_delete(ne_async_thread);
        ne_async_thread = -1;

        NEA_AsyncFile *job = ne_async_active;
        ne_async_active = NULL;

        if (!job->cancelled)
        {
            if (job->state == NEA_ASYNC_READY)
            {
                if (job->finalize != NULL)
                {
                    // Run the finalize step (such as the VRAM upload of a
                    // texture) on the main thread, during the vertical blank.
                    job->finalize(job);
                    job->finalized = true;
                    job->state = job->result ? NEA_ASYNC_DONE
                                              : NEA_ASYNC_ERROR;
                }
                else
                {
                    job->state = NEA_ASYNC_DONE;
                }
            }

            if (job->user_cb != NULL)
                job->user_cb(job, job->user_data);
        }
    }

    // Free handles that were released while still in progress.
    NEA_AsyncFile *job = ne_async_list;
    while (job != NULL)
    {
        NEA_AsyncFile *next = job->next;
        if (job->cancelled && job != ne_async_active)
        {
            ne_async_unlink(job);
            ne_async_destroy(job);
        }
        job = next;
    }

    // Start the next pending load.
    ne_async_try_start();
}

NEA_AsyncState NEA_AsyncGetState(const NEA_AsyncFile *handle)
{
    NEA_AssertPointer(handle, "NULL handle pointer");
    if (handle == NULL)
        return NEA_ASYNC_ERROR;
    return handle->state;
}

char *NEA_AsyncGetData(NEA_AsyncFile *handle, size_t *size)
{
    NEA_AssertPointer(handle, "NULL handle pointer");
    if (handle == NULL)
        return NULL;

    if (handle->state != NEA_ASYNC_READY && handle->state != NEA_ASYNC_DONE)
        return NULL;

    if (size != NULL)
        *size = handle->size;

    // Transfer ownership of the buffer to the caller.
    handle->buffer_owned = false;
    return handle->buffer;
}

void NEA_AsyncSetCallback(NEA_AsyncFile *handle, NEA_AsyncCallback callback,
                          void *user)
{
    NEA_AssertPointer(handle, "NULL handle pointer");
    if (handle == NULL)
        return;
    handle->user_cb = callback;
    handle->user_data = user;
}

void NEA_AsyncRelease(NEA_AsyncFile *handle)
{
    if (handle == NULL)
        return;

    // If the worker thread is reading this file it can't be freed yet. Mark it
    // cancelled and let NEA_AsyncProcess() free it once the thread has stopped.
    if (handle == ne_async_active)
    {
        handle->cancelled = true;
        handle->user_cb = NULL;
        return;
    }

    ne_async_unlink(handle);
    ne_async_destroy(handle);
}

int NEA_AsyncPendingCount(void)
{
    int count = 0;
    for (NEA_AsyncFile *job = ne_async_list; job != NULL; job = job->next)
    {
        if (job->state == NEA_ASYNC_PENDING || job->state == NEA_ASYNC_READY)
            count++;
    }
    return count;
}

char *__NEA_AsyncBuffer(NEA_AsyncFile *handle, size_t *size)
{
    if (handle == NULL)
        return NULL;
    if (size != NULL)
        *size = handle->size;
    return handle->buffer;
}

char *__NEA_AsyncTakeBuffer(NEA_AsyncFile *handle, size_t *size)
{
    if (handle == NULL)
        return NULL;
    if (size != NULL)
        *size = handle->size;
    handle->buffer_owned = false;
    return handle->buffer;
}

void __NEA_AsyncFreeBuffer(NEA_AsyncFile *handle)
{
    if (handle == NULL)
        return;
    if (handle->buffer != NULL && handle->buffer_owned)
        free(handle->buffer);
    handle->buffer = NULL;
    handle->buffer_owned = false;
}

void *__NEA_AsyncParam(NEA_AsyncFile *handle)
{
    if (handle == NULL)
        return NULL;
    return handle->param;
}

void __NEA_AsyncSetResult(NEA_AsyncFile *handle, int result)
{
    if (handle == NULL)
        return;
    handle->result = result;
}

void __NEA_AsyncEnd(void)
{
    if (ne_async_active != NULL)
    {
        // The worker thread may still be running. It is not the current
        // thread, so it can be deleted. Any partially loaded data is lost.
        cothread_delete(ne_async_thread);
        ne_async_thread = -1;
        ne_async_active = NULL;
    }

    NEA_AsyncFile *job = ne_async_list;
    while (job != NULL)
    {
        NEA_AsyncFile *next = job->next;
        ne_async_destroy(job);
        job = next;
    }
    ne_async_list = NULL;
}

//--------------------------------------------------------------------------
// Screenshots
//--------------------------------------------------------------------------

static void NEA_write16(u16 *address, u16 value)
{
    u8 *first = (u8 *)address;
    u8 *second = first + 1;

    *first = value & 0xff;
    *second = value >> 8;
}

static void NEA_write32(u32 *address, u32 value)
{
    u8 *first = (u8 *) address;
    u8 *second = first + 1;
    u8 *third = first + 2;
    u8 *fourth = first + 3;

    *first = value & 0xff;
    *second = (value >> 8) & 0xff;
    *third = (value >> 16) & 0xff;
    *fourth = (value >> 24) & 0xff;
}

int NEA_ScreenshotBMP(const char *filename)
{
    FILE *f = fopen(filename, "wb");

    if (f == NULL)
    {
        NEA_DebugPrint("%s could't be opened", filename);
        return 0;
    }

    NEA_SpecialEffectPause(true);

    // In normal 3D mode we need to capture the composited (3D+2D output)
    // and save it to VRAM. In dual 3D mode it already is in VRAM.
    if (NEA_CurrentExecutionMode() == NEA_ModeSingle3D)
    {
        // TODO: VRAM_D needs to be saved somewhere and then restored!

        vramSetBankD(VRAM_D_LCD);

        REG_DISPCAPCNT = DCAP_BANK(DCAP_BANK_VRAM_D)
                       | DCAP_SIZE(DCAP_SIZE_256x192)
                       | DCAP_MODE(DCAP_MODE_A)
                       | DCAP_SRC_A(DCAP_SRC_A_COMPOSITED)
                       | DCAP_ENABLE;

        while (REG_DISPCAPCNT & DCAP_ENABLE);
    }

    int ysize = 0;

    if (NEA_CurrentExecutionMode() == NEA_ModeSingle3D)
        ysize = 192;
    else
        ysize = 384;

    u8 *temp = malloc(256 * ysize * 3
                      + sizeof(NEA_BMPInfoHeader)
                      + sizeof(NEA_BMPHeader));

    NEA_BMPHeader *header = (NEA_BMPHeader *) temp;
    NEA_BMPInfoHeader *infoheader =
        (NEA_BMPInfoHeader *)(temp + sizeof(NEA_BMPHeader));

    NEA_write16(&header->type, 0x4D42);
    NEA_write32(&header->size, 256 * ysize * 3 + sizeof(NEA_BMPInfoHeader)
                              + sizeof(NEA_BMPHeader));
    NEA_write32(&header->offset,
           sizeof(NEA_BMPInfoHeader) + sizeof(NEA_BMPHeader));
    NEA_write16(&header->reserved1, 0);
    NEA_write16(&header->reserved2, 0);

    NEA_write16(&infoheader->bits, 24);
    NEA_write32(&infoheader->size, sizeof(NEA_BMPInfoHeader));
    NEA_write32(&infoheader->compression, 0);
    NEA_write32(&infoheader->width, 256);
    NEA_write32(&infoheader->height, ysize);
    NEA_write16(&infoheader->planes, 1);
    NEA_write32(&infoheader->imagesize, 256 * ysize * 3);
    NEA_write32(&infoheader->xresolution, 0);
    NEA_write32(&infoheader->yresolution, 0);
    NEA_write32(&infoheader->importantcolors, 0);
    NEA_write32(&infoheader->ncolors, 0);

    // Allow CPU to access VRAM
    uint32_t vramTemp = 0;
    if (NEA_CurrentExecutionMode() != NEA_ModeSingle3D)
    {
        vramTemp = vramSetPrimaryBanks(VRAM_A_LCD, VRAM_B_LCD,
                                       VRAM_C_LCD, VRAM_D_LCD);
    }

    for (int y = 0; y < ysize; y++)
    {
        for (int x = 0; x < 256; x++)
        {
            u16 color = 0;

            if (NEA_CurrentExecutionMode() == NEA_ModeSingle3D)
            {
                color = VRAM_D[256 * 192 - (y + 1) * 256 + x];
            }
            else
            {
                if (y > 191)
                    color = VRAM_C[256 * 192 - (y - 192 + 1) * 256 + x];
                else
                    color = VRAM_D[256 * 192 - (y + 1) * 256 + x];
            }

            u8 b = (color & 31) << 3;
            u8 g = ((color >> 5) & 31) << 3;
            u8 r = ((color >> 10) & 31) << 3;

            int index = ((y * 256) + x) * 3
                      + sizeof(NEA_BMPInfoHeader)
                      + sizeof(NEA_BMPHeader);

            temp[index + 0] = r;
            temp[index + 1] = g;
            temp[index + 2] = b;
        }
    }

    if (NEA_CurrentExecutionMode() != NEA_ModeSingle3D)
        vramRestorePrimaryBanks(vramTemp);

    fwrite(temp, 1, 256 * ysize * 3 + sizeof(NEA_BMPInfoHeader)
                    + sizeof(NEA_BMPHeader), f);
    fclose(f);
    free(temp);

    // TODO: Restore previous value, not just unpause
    NEA_SpecialEffectPause(false);

    return 1;
}
