// SPDX-License-Identifier: MIT
//
// Copyright (c) 2008-2022 Antonio Niño Díaz
//
// This file is part of Nitro Engine Advanced

#ifndef NEA_FAT_H__
#define NEA_FAT_H__

#include <nds.h>

/// @file   NEAFAT.h
/// @brief  Used to load data from FAT.

/// @defgroup fat FAT functions
///
/// Functions to load data from FAT, and to take screenshots.
///
/// @{

/// Loads a file to RAM from a filesystem.
///
/// @param filename Path to the file.
/// @return Returns a pointer to the location of the file that will have to be
///         freed with free().
char *NEA_FATLoadData(const char *filename);

/// Returns size of a file.
///
/// @param filename File to check.
/// @return Returns the file of the size, or -1 on error.
size_t NEA_FATFileSize(const char *filename);

/// Takes a screenshot of the 3D screen.
///
/// It takes a screenshot of the 3D screen (or both screens if in dual 3D mode)
/// and saves it as a BMP file.
///
/// Warning: This function hasn't been tested with all dual 3D modes.
///
/// @param filename File to save the screenshot.
/// @return Returns 1 on success, 0 on error.
int NEA_ScreenshotBMP(const char *filename);

/// @}

/// @defgroup async Asynchronous asset loading
///
/// Loads files in the background using BlocksDS cooperative threads, so that
/// the main loop (and therefore audio streaming and rendering) keeps running
/// while a file is read from the filesystem. NEA_FATWriteDataAsync() does the
/// same for saving a file.
///
/// Loading happens in two phases. The file is first read into RAM by a worker
/// thread that yields between chunks. Once the data is in RAM, a finalize step
/// (for example, the upload of a texture to VRAM) is run on the main thread
/// during the vertical blank by NEA_AsyncProcess().
///
/// Call NEA_AsyncProcess() once per frame, or pass NEA_UPDATE_ASSETS to
/// NEA_WaitForVBL() to have it called automatically.
///
/// Object lifetime: a pending load holds a pointer to the object it will write
/// into (a material, a model, a palette...). Deleting that object aborts the
/// load, which then reports NEA_ASYNC_ERROR. The handle itself stays valid, so
/// it is always safe to delete an object with a load in flight, and the handle
/// must still be released with NEA_AsyncRelease().
///
/// Never poll NEA_AsyncGetState() in a loop that doesn't yield: the worker only
/// runs while the main thread is waiting. Use NEA_WaitForVBL() as usual, or
/// NEA_AsyncWait() if you really need to block.
///
/// Note: On DS hardware, filesystem access through DLDI runs on the ARM9 by
/// default and blocks it during reads. For loading to truly overlap with the
/// main loop the filesystem driver must run on the ARM7. This is already the
/// case for DSi SD card access and cartridge NitroFS; on a DS flashcart you can
/// call dldiSetMode(DLDI_MODE_ARM7) before nitroFSInit().
///
/// @{

/// State of an asynchronous load operation.
typedef enum {
    NEA_ASYNC_PENDING = 0, ///< Queued or being read from the filesystem.
    NEA_ASYNC_READY,       ///< Data is in RAM, the finalize step hasn't run yet.
    NEA_ASYNC_DONE,        ///< The operation finished successfully.
    NEA_ASYNC_ERROR        ///< The operation failed.
} NEA_AsyncState;

/// Opaque handle that represents one asynchronous load operation.
typedef struct NEA_AsyncFile NEA_AsyncFile;

/// Callback invoked on the main thread by NEA_AsyncProcess() when an
/// asynchronous load reaches a terminal state (NEA_ASYNC_DONE or
/// NEA_ASYNC_ERROR).
///
/// @param handle The async handle that finished.
/// @param user The pointer passed to NEA_AsyncSetCallback().
typedef void (*NEA_AsyncCallback)(NEA_AsyncFile *handle, void *user);

/// Queues a file to be read into RAM in the background.
///
/// The returned handle must be released with NEA_AsyncRelease() once you are
/// done with it.
///
/// @param filename Path to the file.
/// @return Handle to poll the operation, or NULL on error.
NEA_AsyncFile *NEA_FATLoadDataAsync(const char *filename);

/// What NEA_FATWriteDataAsync() does with the buffer it is given.
typedef enum {
    /// The engine takes a copy of the data. The caller may free or reuse its
    /// buffer as soon as the function returns.
    NEA_ASYNC_WRITE_COPY = 0,
    /// The engine takes ownership of the buffer and free()s it when the write
    /// ends. The caller must not touch, reuse or free it again. It must have
    /// been allocated with malloc()/calloc()/realloc().
    NEA_ASYNC_WRITE_TAKE,
    /// The engine writes straight out of the caller's buffer without copying
    /// it. The buffer must stay valid and unmodified until the handle reaches
    /// NEA_ASYNC_DONE or NEA_ASYNC_ERROR, or until it is released.
    NEA_ASYNC_WRITE_BORROW
} NEA_AsyncWriteMode;

/// Writes a buffer to a file in the background.
///
/// Useful for saving without interrupting the main loop: the data is written by
/// a worker thread that yields between chunks, so audio and rendering keep
/// running for the whole duration of the save.
///
/// The write is atomic from the point of view of the file: the data goes to
/// "<filename>.temp" first and is only renamed over @p filename once all of it
/// has reached the filesystem. If the write fails, or if the handle is released
/// while it is still running, the temporary file is deleted and the previous
/// contents of @p filename are left untouched. A file is therefore never seen
/// in a half-written state, and a cancelled save can't destroy the last good
/// one.
///
/// The returned handle must be released with NEA_AsyncRelease() once you are
/// done with it.
///
/// If this function fails and returns NULL, ownership of @p data is not
/// transferred: the caller still owns it in NEA_ASYNC_WRITE_TAKE mode too.
///
/// @param filename Path of the file to write.
/// @param data Data to write.
/// @param size Number of bytes to write.
/// @param mode What to do with @p data, see @ref NEA_AsyncWriteMode.
/// @return Handle to poll the operation, or NULL on error.
NEA_AsyncFile *NEA_FATWriteDataAsync(const char *filename, const void *data,
                                     size_t size, NEA_AsyncWriteMode mode);

/// Returns the current state of an asynchronous load operation.
///
/// @param handle Async handle.
/// @return The current NEA_AsyncState.
NEA_AsyncState NEA_AsyncGetState(const NEA_AsyncFile *handle);

/// Retrieves the data buffer of a generic file load (NEA_FATLoadDataAsync()).
///
/// This can be called once the state is NEA_ASYNC_READY or NEA_ASYNC_DONE. The
/// ownership of the buffer is transferred to the caller, which must free it
/// with free(). It returns NULL for handles created by the texture, model,
/// palette (and similar) async loaders, which manage their data internally.
///
/// @param handle Async handle.
/// @param size If not NULL, the size of the file is stored here.
/// @return Pointer to the file data, or NULL if it isn't available.
char *NEA_AsyncGetData(NEA_AsyncFile *handle, size_t *size);

/// Blocks until an asynchronous load has finished.
///
/// The worker thread only runs while the main thread yields, so polling
/// NEA_AsyncGetState() in a tight loop hangs forever. This function yields to
/// the worker and runs NEA_AsyncProcess() until the load reaches a terminal
/// state. It defeats the point of loading asynchronously, so it is only meant
/// for shutdown paths and loading screens.
///
/// @param handle Async handle.
/// @return The final state, NEA_ASYNC_DONE or NEA_ASYNC_ERROR.
NEA_AsyncState NEA_AsyncWait(NEA_AsyncFile *handle);

/// Sets a callback to be invoked when an asynchronous load finishes.
///
/// The callback runs on the main thread, from within NEA_AsyncProcess().
///
/// @param handle Async handle.
/// @param callback Function to call, or NULL to remove a previous callback.
/// @param user Arbitrary pointer passed to the callback.
void NEA_AsyncSetCallback(NEA_AsyncFile *handle, NEA_AsyncCallback callback,
                          void *user);

/// Releases an asynchronous load handle.
///
/// If the operation is still in progress it is cancelled. It is safe to call
/// this at any time. After this call the handle must not be used again.
///
/// @param handle Async handle.
void NEA_AsyncRelease(NEA_AsyncFile *handle);

/// Returns the number of asynchronous loads that haven't finished yet.
///
/// @return Number of operations still pending.
int NEA_AsyncPendingCount(void);

/// Advances pending asynchronous loads and runs finalize steps.
///
/// Call this once per frame. It reaps finished worker threads and performs the
/// finalize step (such as uploading a texture to VRAM) on the main thread. It
/// is also called automatically when NEA_UPDATE_ASSETS is passed to
/// NEA_WaitForVBL().
void NEA_AsyncProcess(void);

/// @}

/// @cond INTERNAL

// Internal API used by other Nitro Engine Advanced modules to implement
// asynchronous texture and model loading. Don't use these functions directly.

// Same as NEA_FATLoadData(), but it also returns the size of the file. Used by
// the loaders that need to know how many bytes they got.
char *__NEA_FATLoadDataSize(const char *filename, size_t *size);

// Checks that a buffer really holds a complete GRF file. grfLoadMemEx() trusts
// the lengths stored inside the file, so this must be called on any buffer that
// didn't come from grfLoadPath() before decoding it.
bool __NEA_GRFBufferIsSane(const void *buffer, size_t size);

/// Second-stage worker function, run in the worker thread after the file read.
typedef bool (*__NEA_AsyncWorkerFn)(NEA_AsyncFile *handle);

/// Finalize/cleanup function, run on the main thread by NEA_AsyncProcess().
typedef void (*__NEA_AsyncFinalizeFn)(NEA_AsyncFile *handle);

// Queues an async job with module-defined worker, finalize and discard hooks.
// On failure it returns NULL and does not free 'param'.
//
// 'target' is the engine object that the finalize step writes into (or NULL for
// a plain file read). Deleting that object must call __NEA_AsyncCancelTarget()
// so that finalize never runs against freed memory.
NEA_AsyncFile *__NEA_AsyncQueue(const char *filename,
                                __NEA_AsyncWorkerFn worker_stage2,
                                __NEA_AsyncFinalizeFn finalize,
                                __NEA_AsyncFinalizeFn discard,
                                void *param, void *target);

// Registers an extra object as a target of a job, on top of the one passed to
// __NEA_AsyncQueue(). Used by loaders that write into more than one object,
// such as the GRF loader (a material and its palette).
void __NEA_AsyncAddTarget(NEA_AsyncFile *handle, void *target);

// Aborts every queued or in-flight load registered against 'target'. Their
// finalize step never runs. The handles stay valid and report NEA_ASYNC_ERROR,
// because the app may still be holding them; it releases them as usual with
// NEA_AsyncRelease(). Call this from the destructor of any object that can be
// the target of an asynchronous load, before freeing it.
void __NEA_AsyncCancelTarget(void *target);

// Returns the file data buffer of a job (no ownership transfer).
char *__NEA_AsyncBuffer(NEA_AsyncFile *handle, size_t *size);

// Returns the file data buffer of a job and transfers its ownership to the
// caller (the async system will no longer free it).
char *__NEA_AsyncTakeBuffer(NEA_AsyncFile *handle, size_t *size);

// Frees the file data buffer of a job right away.
void __NEA_AsyncFreeBuffer(NEA_AsyncFile *handle);

// Returns the module-specific parameter block of a job.
void *__NEA_AsyncParam(NEA_AsyncFile *handle);

// Stores the result of a finalize step (1 = success, 0 = failure).
void __NEA_AsyncSetResult(NEA_AsyncFile *handle, int result);

// Stops the async system and frees all its resources. Called by NEA_End().
void __NEA_AsyncEnd(void);

/// @endcond

#endif // NEA_FAT_H__
