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
/// while a file is read from the filesystem.
///
/// Loading happens in two phases. The file is first read into RAM by a worker
/// thread that yields between chunks. Once the data is in RAM, a finalize step
/// (for example, the upload of a texture to VRAM) is run on the main thread
/// during the vertical blank by NEA_AsyncProcess().
///
/// Call NEA_AsyncProcess() once per frame, or pass NEA_UPDATE_ASSETS to
/// NEA_WaitForVBL() to have it called automatically.
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

/// Returns the current state of an asynchronous load operation.
///
/// @param handle Async handle.
/// @return The current NEA_AsyncState.
NEA_AsyncState NEA_AsyncGetState(const NEA_AsyncFile *handle);

/// Retrieves the data buffer of a generic file load (NEA_FATLoadDataAsync()).
///
/// This can be called once the state is NEA_ASYNC_READY or NEA_ASYNC_DONE. The
/// ownership of the buffer is transferred to the caller, which must free it
/// with free(). It returns NULL for handles created by the texture or model
/// async loaders, which manage their data internally.
///
/// @param handle Async handle.
/// @param size If not NULL, the size of the file is stored here.
/// @return Pointer to the file data, or NULL if it isn't available.
char *NEA_AsyncGetData(NEA_AsyncFile *handle, size_t *size);

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

/// Second-stage worker function, run in the worker thread after the file read.
typedef bool (*__NEA_AsyncWorkerFn)(NEA_AsyncFile *handle);

/// Finalize/cleanup function, run on the main thread by NEA_AsyncProcess().
typedef void (*__NEA_AsyncFinalizeFn)(NEA_AsyncFile *handle);

// Queues an async job with module-defined worker, finalize and discard hooks.
// On failure it returns NULL and does not free 'param'.
NEA_AsyncFile *__NEA_AsyncQueue(const char *filename,
                                __NEA_AsyncWorkerFn worker_stage2,
                                __NEA_AsyncFinalizeFn finalize,
                                __NEA_AsyncFinalizeFn discard,
                                void *param);

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
