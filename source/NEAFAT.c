// SPDX-License-Identifier: MIT
//
// Copyright (c) 2008-2022 Antonio Niño Díaz
//
// This file is part of Nitro Engine Advanced

#include <malloc.h>

#include <nds/arm9/dldi.h>
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

// Extension appended to the destination path while an asynchronous write is in
// progress. The file only takes its real name once it is complete.
#define NEA_ASYNC_TEMP_SUFFIX ".temp"

// Number of engine objects a single job can be registered against. Two is
// enough for the GRF loader, which writes into a material and a palette.
#define NEA_ASYNC_MAX_TARGETS 2

// How many loads may be read at the same time. Concurrency lets one job's
// decompression overlap the next job's file read, but it costs: each worker has
// its own stack, its own copy of the file in RAM, and (for GRF files) its own
// decoded buffers until the finalize step runs. It also means this many VRAM
// uploads can land in the same vertical blank, so raise it with care.
#define NEA_ASYNC_MAX_WORKERS 2

// How many times cothread_create() may fail for a job before it is reported as
// failed instead of being retried forever.
#define NEA_ASYNC_MAX_START_ATTEMPTS 8

#ifdef NEA_DEBUG
// Pattern the worker stacks are filled with, so that the untouched part of a
// stack can be told apart from the used part. Same trick as the task pool in
// NEAThread.c.
#define NEA_ASYNC_STACK_PATTERN 0xA5A5A5A5

// Fraction of a worker stack that may be used before it is worth complaining
// about, in eighths. Overflowing a cothread stack corrupts the neighbouring
// heap block instead of faulting, so the crash surfaces somewhere unrelated and
// moves whenever the allocation layout does. Warning early is the only way to
// catch it near the cause.
#define NEA_ASYNC_STACK_WARN_EIGHTHS 7
#endif

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

    long len = ftell(f);
    if (len < 0)
    {
        NEA_DebugPrint("Failed to ftell: %s", filename);
        fclose(f);
        return NULL;
    }
    rewind(f);

    size_t size = len;

    // Always allocate at least one byte so that an empty file isn't reported as
    // an out of memory error (malloc(0) is allowed to return NULL).
    char *buffer = malloc(size > 0 ? size : 1);
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

char *__NEA_FATLoadDataSize(const char *filename, size_t *size)
{
    return ne_fat_read_file(filename, size, NULL);
}

char *__NEA_AsyncReadFile(NEA_AsyncFile *job, const char *filename,
                          size_t *size)
{
    return ne_fat_read_file(filename, size, job);
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

    // True if this job writes 'buffer' out to 'filename' instead of reading the
    // file in. The data is written to 'temp_filename' and only renamed over
    // 'filename' once it is complete, so an interrupted write can't destroy the
    // previous contents of the file.
    bool is_write;
    char *temp_filename;        // "<filename>.temp" (owned, write jobs only)

    volatile NEA_AsyncState state;
    // Set by NEA_AsyncRelease() while in progress. The app has given up the
    // handle, so it is destroyed as soon as the worker thread stops.
    volatile bool cancelled;
    // Set by __NEA_AsyncCancelTarget() when the object this job would write
    // into has been deleted. Unlike 'cancelled', the app may still be holding
    // the handle, so it stays alive and reports NEA_ASYNC_ERROR instead.
    volatile bool aborted;
    bool buffer_owned;          // True while this handle owns 'buffer'
    bool finalized;             // True once finalize() has run
    // Set when the worker has been joined but the main-thread finalize step and
    // the user callback haven't run yet. See NEA_AsyncProcess().
    bool needs_finish;

    int result;                 // Result of finalize() (1 = success)

    // True from the moment a worker is created for this job until its exit has
    // been observed. Workers are detached, so there is no thread ID to keep:
    // see the comment above cothread_create() in ne_async_try_start().
    bool worker_active;
    // Set by the worker itself as its very last action. This is what replaces
    // cothread_has_joined(), which can't be used on a detached thread.
    volatile bool worker_done;
    // Number of times cothread_create() has failed for this job. A job that can
    // never get a worker is failed instead of staying pending forever.
    int start_attempts;

    // Number of NEA_AsyncWait() calls parked on this handle. While it is not
    // zero the handle must not be freed: the waiter reads it again after every
    // NEA_AsyncProcess(), and that call can run a user callback that releases
    // this very handle. Freeing it there would leave the waiter reading freed
    // memory, so a release with waiters pending only marks it cancelled and
    // NEA_AsyncWait() does the destroying on its way out.
    int waiters;

    // Index into ne_async_stacks of the stack this job's worker is running on,
    // or -1 when it has no worker. See ne_async_stack_acquire().
    int stack_slot;

    // Engine objects that finalize() writes into (NEA_Material, NEA_Model...).
    // All NULL for generic NEA_FATLoadDataAsync() jobs. Deleting any of them
    // must abort the job, see __NEA_AsyncCancelTarget(). Two slots are enough
    // for the loaders that write into a material and its palette.
    void *targets[NEA_ASYNC_MAX_TARGETS];

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
// Number of worker threads currently running.
static int ne_async_running = 0;

// Worker stacks, owned by this module rather than by the scheduler.
//
// cothread_create() allocates and frees the stack itself, which leaves no way
// to see how much of it a worker actually used. That matters here: a worker
// that overflows its stack does not fault, it quietly writes over the next heap
// block, and the resulting crash lands somewhere unrelated and moves whenever
// the allocation layout changes. Owning the stacks makes the usage measurable.
//
// One per concurrent worker, allocated on first use and kept for the lifetime
// of the program: a worker that could not get a stack would have to fail its
// job, and reusing them costs nothing.
static struct {
    void *base;
    bool in_use;
} ne_async_stacks[NEA_ASYNC_MAX_WORKERS];

// Claims a worker stack, returning its index or -1 if none is free or one could
// not be allocated.
static int ne_async_stack_acquire(void)
{
    for (int i = 0; i < NEA_ASYNC_MAX_WORKERS; i++)
    {
        if (ne_async_stacks[i].in_use)
            continue;

        if (ne_async_stacks[i].base == NULL)
        {
            // cothread_create_manual() requires 8 byte alignment.
            ne_async_stacks[i].base = memalign(8, NEA_ASYNC_STACK_SIZE);
            if (ne_async_stacks[i].base == NULL)
                return -1;
        }

#ifdef NEA_DEBUG
        // Refilled on every claim, so the peak reported below is this worker's
        // and not the high-water mark of every job that used the stack before.
        uint32_t *words = ne_async_stacks[i].base;
        for (size_t j = 0; j < NEA_ASYNC_STACK_SIZE / 4; j++)
            words[j] = NEA_ASYNC_STACK_PATTERN;
#endif

        ne_async_stacks[i].in_use = true;
        return i;
    }

    return -1;
}

// Releases a worker stack. Only ever called from the main thread, once the
// worker has been observed to have finished, so the stack is no longer in use.
static void ne_async_stack_release(int slot)
{
    if (slot < 0 || slot >= NEA_ASYNC_MAX_WORKERS)
        return;

#ifdef NEA_DEBUG
    // The stack grows down from the top, so the untouched region is at the
    // bottom: the peak is whatever the fill pattern no longer covers.
    const uint32_t *words = ne_async_stacks[slot].base;
    size_t total = NEA_ASYNC_STACK_SIZE / 4;
    size_t untouched = 0;
    while (untouched < total && words[untouched] == NEA_ASYNC_STACK_PATTERN)
        untouched++;

    size_t used = (total - untouched) * 4;
    if (used >= (NEA_ASYNC_STACK_SIZE / 8) * NEA_ASYNC_STACK_WARN_EIGHTHS)
    {
        NEA_DebugPrint("Async worker used %d of %d bytes of stack. Raise "
                       "NEA_ASYNC_STACK_SIZE.", (int)used,
                       (int)NEA_ASYNC_STACK_SIZE);
    }
#endif

    ne_async_stacks[slot].in_use = false;
}

// Records that a job's worker has finished. Every path that observes a worker
// exiting goes through here, so the running count and the stack it borrowed are
// always released exactly once.
static void ne_async_worker_reap(NEA_AsyncFile *job)
{
    job->worker_active = false;
    ne_async_running--;

    ne_async_stack_release(job->stack_slot);
    job->stack_slot = -1;
}

// Signal ID a job's worker sends when it exits, so that code waiting for that
// worker can block instead of polling. libnds reserves bit 31 of signal IDs for
// its own use (comutex/cosema), and handles live in main RAM, so the address of
// the job is directly usable as a signal ID.
static uint32_t ne_async_signal_id(const NEA_AsyncFile *job)
{
    NEA_Assert(((uintptr_t)job & BIT(31)) == 0,
               "Async handle outside of the usable signal ID range");
    return (uint32_t)(uintptr_t)job;
}

// True if 'handle' is still a live job. Handles are bare heap pointers handed
// to the app, and both __NEA_AsyncEnd() and a completion callback can destroy
// one while the app still holds it, so every public entry point checks
// membership of the list before touching the handle. It also catches the case
// where the allocator hands a freed handle's address back to a new job: a stale
// pointer then addresses a different, live job instead of freed memory.
static bool ne_async_handle_valid(const NEA_AsyncFile *handle)
{
    for (const NEA_AsyncFile *job = ne_async_list; job != NULL; job = job->next)
    {
        if (job == handle)
            return true;
    }

    return false;
}

// Used by ne_fat_read_file() to poll the cancel flags without exposing the
// internals of NEA_AsyncFile to that helper. Either flag means the result of
// the read is no longer wanted, so the read stops at the next chunk boundary.
static bool ne_async_is_cancelled(const NEA_AsyncFile *job)
{
    return job->cancelled || job->aborted;
}

// Writes the payload of a write job out to the filesystem, in chunks, yielding
// to other cothreads between them.
//
// The data goes to a temporary file first and is only renamed over the real one
// once all of it is on disk. That way a write that fails, or that is cancelled
// half way through, leaves the previous contents of the file untouched instead
// of replacing them with a truncated version.
static bool ne_fat_write_file(NEA_AsyncFile *job)
{
    FILE *f = fopen(job->temp_filename, "wb");
    if (f == NULL)
    {
        NEA_DebugPrint("%s could't be opened for writing", job->temp_filename);
        return false;
    }

    size_t done = 0;
    while (done < job->size)
    {
        if (ne_async_is_cancelled(job))
            goto fail;

        size_t chunk = job->size - done;
        if (chunk > NEA_ASYNC_CHUNK_SIZE)
            chunk = NEA_ASYNC_CHUNK_SIZE;

        if (fwrite(job->buffer + done, 1, chunk, f) != chunk)
        {
            NEA_DebugPrint("Failed to write data of %s", job->temp_filename);
            goto fail;
        }

        done += chunk;
        cothread_yield();
    }

    // Only now is the data guaranteed to have reached the filesystem, so this
    // is the last point at which the write can still fail without touching the
    // file the app cares about.
    if (fclose(f) != 0)
    {
        NEA_DebugPrint("Failed to flush %s", job->temp_filename);
        remove(job->temp_filename);
        return false;
    }

    // FAT rename() won't overwrite an existing target, so the old file has to go
    // first. It may not exist at all, so its failure isn't an error.
    remove(job->filename);

    if (rename(job->temp_filename, job->filename) != 0)
    {
        NEA_DebugPrint("Failed to rename %s", job->temp_filename);
        remove(job->temp_filename);
        return false;
    }

    return true;

fail:
    fclose(f);
    remove(job->temp_filename);
    return false;
}

// Worker thread entrypoint. Reads the file (or writes it, for a write job) and
// runs the optional second-stage processing. Runs in its own cothread.
static int ne_async_worker_entry(void *arg)
{
    NEA_AsyncFile *job = arg;

    // Every exit path has to go through the end of this function, so that the
    // signal is sent exactly once and nothing waiting for this worker is left
    // blocked forever.
    if (job->is_write)
    {
        if (!ne_async_is_cancelled(job))
        {
            // Write jobs have no finalize step, so NEA_AsyncProcess() turns this
            // NEA_ASYNC_READY into NEA_ASYNC_DONE on the main thread.
            if (ne_fat_write_file(job))
                job->state = NEA_ASYNC_READY;
            else if (!ne_async_is_cancelled(job))
                job->state = NEA_ASYNC_ERROR;
        }
    }
    else if (!ne_async_is_cancelled(job))
    {
        size_t size = 0;
        char *buffer = ne_fat_read_file(job->filename, &size, job);
        if (buffer == NULL)
        {
            if (!ne_async_is_cancelled(job))
                job->state = NEA_ASYNC_ERROR;
        }
        else
        {
            job->buffer = buffer;
            job->size = size;
            job->buffer_owned = true;

            if (!ne_async_is_cancelled(job))
            {
                if (job->worker_stage2 != NULL && !job->worker_stage2(job))
                    job->state = NEA_ASYNC_ERROR;
                else
                    job->state = NEA_ASYNC_READY;
            }
        }
    }

    // Must be the last thing this thread touches in the job: it is what tells
    // the main thread that this worker is finished. The scheduler deletes the
    // thread itself once this function returns (it is detached).
    job->worker_done = true;
    cothread_send_signal(ne_async_signal_id(job));
    return 0;
}

// Starts worker threads for pending handles, up to the concurrency limit.
static void ne_async_try_start(void)
{
    for (NEA_AsyncFile *job = ne_async_list; job != NULL; job = job->next)
    {
        if (ne_async_running >= NEA_ASYNC_MAX_WORKERS)
            return;

        if (job->state != NEA_ASYNC_PENDING || ne_async_is_cancelled(job))
            continue;

        // Already being worked on.
        if (job->worker_active)
            continue;

        // Workers are created detached on purpose, so that this code never has
        // to call cothread_delete() itself.
        //
        // The libnds scheduler caches the next thread of its list *before*
        // resuming a thread ("next_ctx"). If the resumed thread (this one, from
        // inside NEA_AsyncProcess()) deleted the thread that pointer refers to,
        // and a new thread reused that memory, the scheduler would resume a
        // dangling context and jump into the reallocated block. A detached
        // thread is instead deleted by the scheduler itself right after it
        // returns, which is the case libnds handles correctly.
        int slot = ne_async_stack_acquire();
        cothread_t thread = -1;

        if (slot >= 0)
        {
            thread = cothread_create_manual(ne_async_worker_entry, job,
                                            ne_async_stacks[slot].base,
                                            NEA_ASYNC_STACK_SIZE,
                                            COTHREAD_DETACHED);
        }

        if (thread == -1)
        {
            ne_async_stack_release(slot);
            NEA_DebugPrint("Couldn't create async worker thread");

            // Out of memory. Retry on later calls, but don't let the job sit
            // pending forever: an app polling NEA_AsyncPendingCount() would
            // never see the queue drain.
            if (++job->start_attempts >= NEA_ASYNC_MAX_START_ATTEMPTS)
                job->state = NEA_ASYNC_ERROR;

            // No point trying the other jobs this time round.
            return;
        }

        (void)thread; // Detached: the ID must not be used after this point.
        job->stack_slot = slot;
        job->worker_active = true;
        ne_async_running++;
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
    free(job->temp_filename);
    free(job->filename);
    free(job);
}

// Allocates a job and puts it at the end of the queue. Shared by the read and
// write entry points; everything specific to one of them is set by the caller
// on the returned handle before the worker can pick it up.
static NEA_AsyncFile *ne_async_queue_common(const char *filename)
{
    NEA_AssertPointer(filename, "NULL filename pointer");

#ifdef NEA_DEBUG
    // With DLDI running on the ARM9 the filesystem read blocks the ARM9, so the
    // worker can't actually overlap with the main loop and this degrades into a
    // slower synchronous load. Warn once, the first time anything is queued.
    {
        static bool warned = false;
        if (!warned && !isDSiMode() && dldiGetMode() == DLDI_MODE_ARM9)
        {
            warned = true;
            NEA_DebugPrint("DLDI runs on the ARM9: async loads won't overlap. "
                           "Call dldiSetMode(DLDI_MODE_ARM7) before nitroFSInit()");
        }
    }
#endif

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
    // calloc() zeroed this, and 0 is a valid stack index.
    job->stack_slot = -1;

    return job;
}

NEA_AsyncFile *__NEA_AsyncQueue(const char *filename,
                                __NEA_AsyncWorkerFn worker_stage2,
                                __NEA_AsyncFinalizeFn finalize,
                                __NEA_AsyncFinalizeFn discard,
                                void *param, void *target)
{
    NEA_AsyncFile *job = ne_async_queue_common(filename);
    if (job == NULL)
        return NULL;

    job->worker_stage2 = worker_stage2;
    job->finalize = finalize;
    job->discard = discard;
    job->param = param;
    job->targets[0] = target;

    ne_async_append(job);
    ne_async_try_start();

    return job;
}

NEA_AsyncFile *NEA_FATWriteDataAsync(const char *filename, const void *data,
                                     size_t size, NEA_AsyncWriteMode mode)
{
    NEA_AssertPointer(filename, "NULL filename pointer");
    NEA_AssertPointer(data, "NULL data pointer");

    if (data == NULL)
        return NULL;

    NEA_AsyncFile *job = ne_async_queue_common(filename);
    if (job == NULL)
        return NULL;

    // The temporary name is built here, on the main thread, so that the worker
    // never has to allocate and can't fail half way through the write.
    size_t len = strlen(job->filename);
    job->temp_filename = malloc(len + sizeof(NEA_ASYNC_TEMP_SUFFIX));
    if (job->temp_filename == NULL)
    {
        NEA_DebugPrint("Not enough memory");
        goto fail;
    }
    memcpy(job->temp_filename, job->filename, len);
    memcpy(job->temp_filename + len, NEA_ASYNC_TEMP_SUFFIX,
           sizeof(NEA_ASYNC_TEMP_SUFFIX));

    if (mode == NEA_ASYNC_WRITE_COPY)
    {
        // Always allocate at least one byte: malloc(0) may return NULL, which
        // would look like an out of memory error for an empty file.
        job->buffer = malloc(size > 0 ? size : 1);
        if (job->buffer == NULL)
        {
            NEA_DebugPrint("Not enough memory");
            goto fail;
        }
        memcpy(job->buffer, data, size);
        job->buffer_owned = true;
    }
    else
    {
        // TAKE and BORROW both write straight out of the caller's buffer. They
        // only differ in who frees it, which is what buffer_owned decides.
        job->buffer = (char *)data;
        job->buffer_owned = (mode == NEA_ASYNC_WRITE_TAKE);
    }

    job->size = size;
    job->is_write = true;

    ne_async_append(job);
    ne_async_try_start();

    return job;

fail:
    free(job->temp_filename);
    free(job->filename);
    free(job);
    return NULL;
}

// Drops everything a job owns except the handle itself, and marks it as failed.
// Used when the object the job would write into is gone.
static void ne_async_abort_finish(NEA_AsyncFile *job)
{
    if (!job->finalized && job->discard != NULL)
    {
        job->discard(job);
        job->finalized = true;
    }

    __NEA_AsyncFreeBuffer(job);
    job->state = NEA_ASYNC_ERROR;
}

void __NEA_AsyncAddTarget(NEA_AsyncFile *handle, void *target)
{
    if (handle == NULL || target == NULL)
        return;

    for (int i = 0; i < NEA_ASYNC_MAX_TARGETS; i++)
    {
        if (handle->targets[i] == NULL || handle->targets[i] == target)
        {
            handle->targets[i] = target;
            return;
        }
    }

    NEA_DebugPrint("Too many async targets");
}

void __NEA_AsyncCancelTarget(void *target)
{
    if (target == NULL)
        return;

    for (NEA_AsyncFile *job = ne_async_list; job != NULL; job = job->next)
    {
        bool match = false;

        for (int i = 0; i < NEA_ASYNC_MAX_TARGETS; i++)
        {
            if (job->targets[i] != target)
                continue;

            job->targets[i] = NULL;
            match = true;
        }

        if (!match)
            continue;

        // A job that already ran its finalize step has handed its result to the
        // object and is no longer aimed at anything. Aborting it here would free
        // a buffer the finalize may still reference and would report a load that
        // actually succeeded as NEA_ASYNC_ERROR.
        if (job->finalized || job->state == NEA_ASYNC_DONE
            || job->state == NEA_ASYNC_ERROR)
            continue;

        job->aborted = true;

        // The worker thread may be reading this file right now. It stops at the
        // next chunk boundary; NEA_AsyncProcess() does the cleanup once the
        // thread has joined. Touching the job here would race with it.
        if (job->worker_active)
            continue;

        ne_async_abort_finish(job);
    }
}

NEA_AsyncFile *NEA_FATLoadDataAsync(const char *filename)
{
    return __NEA_AsyncQueue(filename, NULL, NULL, NULL, NULL, NULL);
}

void NEA_AsyncProcess(void)
{
    // Reap every worker that has finished. With several workers running, more
    // than one can finish in the same frame, so this walks the whole list
    // instead of looking at a single active job.
    //
    // Joining is done first, on its own, because no user code runs here and the
    // list can't change under the walk.
    for (NEA_AsyncFile *job = ne_async_list; job != NULL; job = job->next)
    {
        if (!job->worker_active || !job->worker_done)
            continue;

        // Detached: the scheduler frees the thread itself, nothing to delete.
        ne_async_worker_reap(job);
        job->needs_finish = true;
    }

    // Now finish the reaped jobs. finalize() and the user callback are free to
    // release handles (their own included), which unlinks and frees them, so
    // the scan restarts from the head after every one instead of holding a
    // 'next' pointer across the call.
    bool finished_one = true;
    while (finished_one)
    {
        finished_one = false;

        for (NEA_AsyncFile *job = ne_async_list; job != NULL; job = job->next)
        {
            if (!job->needs_finish)
                continue;

            // A finalize step uploads to VRAM, flips bank modes and can
            // reallocate texture VRAM. None of that is safe while the app holds
            // a raw VRAM pointer from NEA_TextureDrawingStart() or
            // NEA_PaletteModificationStart(), so leave this job marked and
            // finish it on a later call. 'finished_one' stays false, so the
            // outer loop ends here too.
            if (__NEA_VramSessionOpen())
            {
                NEA_DebugPrint("Finalize deferred: a VRAM editing session is "
                               "open. Close it before pumping the frame.");
                break;
            }

            job->needs_finish = false;
            finished_one = true;

            if (job->cancelled)
                break;

            if (job->aborted)
            {
                // The object this load was aimed at has been deleted. Never run
                // finalize, it would write into freed memory. The app still owns
                // the handle, so keep it alive and report the failure.
                ne_async_abort_finish(job);
            }
            else if (job->state == NEA_ASYNC_READY)
            {
                if (job->finalize != NULL)
                {
                    // Run the finalize step (such as the VRAM upload of a
                    // texture) on the main thread, during the vertical blank.
                    job->finalize(job);
                    job->finalized = true;
                    job->state = job->result ? NEA_ASYNC_DONE : NEA_ASYNC_ERROR;
                }
                else
                {
                    job->state = NEA_ASYNC_DONE;
                }
            }

            if (job->user_cb != NULL)
                job->user_cb(job, job->user_data);

            break;
        }
    }

    // Free handles that were released while still in progress. A handle whose
    // worker is still running can't be freed yet.
    NEA_AsyncFile *job = ne_async_list;
    while (job != NULL)
    {
        NEA_AsyncFile *next = job->next;
        // A handle NEA_AsyncWait() is parked on stays alive until the waiter
        // returns: it reads the handle again after this call, and the waiter
        // destroys it itself on the way out.
        if (job->cancelled && !job->worker_active && job->waiters == 0)
        {
            ne_async_unlink(job);
            ne_async_destroy(job);
        }
        job = next;
    }

    // Start as many pending loads as the concurrency limit allows.
    ne_async_try_start();
}

NEA_AsyncState NEA_AsyncGetState(const NEA_AsyncFile *handle)
{
    NEA_AssertPointer(handle, "NULL handle pointer");
    if (!ne_async_handle_valid(handle))
        return NEA_ASYNC_ERROR;
    return handle->state;
}

char *NEA_AsyncGetData(NEA_AsyncFile *handle, size_t *size)
{
    NEA_AssertPointer(handle, "NULL handle pointer");
    if (!ne_async_handle_valid(handle))
        return NULL;

    if (handle->state != NEA_ASYNC_READY && handle->state != NEA_ASYNC_DONE)
        return NULL;

    // Handles created by the texture, model, palette... loaders manage their
    // data internally: the finalize step still needs the buffer, and some of
    // them hand it over to the engine object. Never let it be stolen here.
    if (handle->finalize != NULL)
        return NULL;

    // A write job's buffer belongs to the caller, not to the async system.
    // Handing it back would also clear buffer_owned and leak a COPY/TAKE
    // buffer.
    if (handle->is_write)
        return NULL;

    if (size != NULL)
        *size = handle->size;

    // Transfer ownership of the buffer to the caller. The handle drops its own
    // pointer as well, so calling this twice can't hand the same buffer to two
    // owners and have them both free it.
    char *buffer = handle->buffer;
    handle->buffer = NULL;
    handle->buffer_owned = false;
    return buffer;
}

NEA_AsyncState NEA_AsyncWait(NEA_AsyncFile *handle)
{
    NEA_AssertPointer(handle, "NULL handle pointer");
    if (!ne_async_handle_valid(handle))
        return NEA_ASYNC_ERROR;

    // Nothing here could ever make progress: NEA_AsyncProcess() refuses to run
    // the finalize steps while a VRAM editing session is open, so this would
    // spin until the frame budget ran out and then keep spinning.
    NEA_Assert(!__NEA_VramSessionOpen(),
               "NEA_AsyncWait() called inside a VRAM editing session");

    // NEA_AsyncProcess() below runs the user callback, and releasing your own
    // handle from it is the ordinary fire-and-forget pattern. Claiming the
    // handle here turns that release into a deferred one, so the loop can keep
    // reading the handle instead of dereferencing freed memory.
    handle->waiters++;

    NEA_AsyncState state;

    while (1)
    {
        state = handle->state;

        // Released from under us. The handle is ours to destroy below, and its
        // state means nothing to a caller that has given it up.
        if (handle->cancelled)
        {
            state = NEA_ASYNC_ERROR;
            break;
        }

        if (state != NEA_ASYNC_PENDING && state != NEA_ASYNC_READY)
            break;

        // The worker only runs while the main thread is yielding, so a plain
        // poll loop on NEA_AsyncGetState() would hang forever.
        //
        // 'worker_done' has to be checked as well as 'worker_active': a signal
        // is delivered to whoever is parked at the time and is not remembered,
        // so a worker that has already finished and signalled -- which is the
        // case whenever frames were pumped without NEA_UPDATE_ASSETS, leaving
        // it unreaped -- would never send another one and this would park for
        // good. In that state there is nothing to wait for anyway: the vertical
        // blank below is what reaps it.
        if (handle->worker_active && !handle->worker_done)
        {
            // A worker is reading this file: block until it signals that it has
            // finished, instead of spinning.
            cothread_yield_signal(ne_async_signal_id(handle));
        }

        // Only ever finish jobs from the vertical blank, never straight off the
        // signal wake. A finalize step uploads to VRAM and flips banks to LCD
        // mode, and the signal fires at whatever scanline the worker happened to
        // exit on -- in the dual 3D modes those banks are the framebuffers being
        // displayed. This is also what makes progress when there is no worker to
        // wait for, because the job is queued behind others or its data is in
        // RAM already and only the main-thread finalize step is left.
        cothread_yield_irq(IRQ_VBLANK);

        NEA_AsyncProcess();
    }

    handle->waiters--;

    // Nothing else frees a handle that has a waiter, so a release that arrived
    // while this loop was running left the job here to be destroyed.
    if (handle->waiters == 0 && handle->cancelled && !handle->worker_active)
    {
        ne_async_unlink(handle);
        ne_async_destroy(handle);
    }

    return state;
}

void NEA_AsyncSetCallback(NEA_AsyncFile *handle, NEA_AsyncCallback callback,
                          void *user)
{
    NEA_AssertPointer(handle, "NULL handle pointer");
    if (!ne_async_handle_valid(handle))
        return;
    handle->user_cb = callback;
    handle->user_data = user;
}

void NEA_AsyncRelease(NEA_AsyncFile *handle)
{
    if (handle == NULL)
        return;

    // Releasing a handle twice, or holding one across NEA_End(), used to read
    // and then free memory that is already gone.
    if (!ne_async_handle_valid(handle))
    {
        NEA_DebugPrint("Async handle released twice or after NEA_End()");
        return;
    }

    // A write job that borrowed the caller's buffer is the one case that can't
    // simply be abandoned: the worker may be parked inside fwrite() and would
    // resume reading from a buffer the app frees as soon as this returns. It
    // only polls the cancel flag at chunk boundaries, so wait it out here --
    // which is what NEA_FATWriteDataAsync() already promises for BORROW.
    if (handle->worker_active && handle->is_write && !handle->buffer_owned)
    {
        handle->cancelled = true;
        handle->user_cb = NULL;

        while (!handle->worker_done)
            cothread_yield_signal(ne_async_signal_id(handle));

        // Reaped here, so NEA_AsyncProcess() must not count this worker again.
        ne_async_worker_reap(handle);
    }

    // If a worker thread is reading this file it can't be freed yet. Mark it
    // cancelled and let NEA_AsyncProcess() free it once the thread has stopped.
    // A handle NEA_AsyncWait() is parked on is deferred the same way, and is
    // destroyed by the waiter instead.
    if (handle->worker_active || handle->waiters > 0)
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
    // Ask every worker to stop first, so they can all unwind in parallel instead
    // of being joined one after the other.
    for (NEA_AsyncFile *job = ne_async_list; job != NULL; job = job->next)
    {
        if (job->worker_active)
            job->cancelled = true;
    }

    for (NEA_AsyncFile *job = ne_async_list; job != NULL; job = job->next)
    {
        if (!job->worker_active)
            continue;

        // A worker may be parked inside the file read, holding an open FILE *
        // and a partially filled buffer, so let it unwind itself instead of
        // being killed. Wait for the signal it sends on the way out; the
        // scheduler frees the (detached) thread once it returns.
        while (!job->worker_done)
            cothread_yield_signal(ne_async_signal_id(job));

        ne_async_worker_reap(job);
    }

    NEA_AsyncFile *job = ne_async_list;
    while (job != NULL)
    {
        NEA_AsyncFile *next = job->next;
        ne_async_destroy(job);
        job = next;
    }
    ne_async_list = NULL;

    // Every worker has been waited out above, so this is already zero unless
    // the count drifted. Force it, so that re-initialising the engine can't
    // start with a phantom worker throttling the queue forever.
    ne_async_running = 0;
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
