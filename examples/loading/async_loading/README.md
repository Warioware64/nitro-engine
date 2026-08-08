# Asynchronous asset loading

This example shows how to load assets in the background with the Nitro Engine
Advanced async API, so that the main loop (and therefore audio streaming and
rendering) keeps running while a file is read from the filesystem.

## Controls

- **A**: Load the texture asynchronously with `NEA_MaterialTexLoadGRFAsync()`.
  The on-screen frame counter and spinner keep advancing while the file loads.
- **B**: Load the same texture synchronously with `NEA_MaterialTexLoadGRF()`.
  The whole main loop blocks until the load finishes, so the counter freezes.
- **X**: Queue a texture and a palette at the same time
  (`NEA_MaterialTexLoadFATAsync()` + `NEA_PaletteLoadFATAsync()`). They share one
  worker thread, so they are read one after the other while the main loop keeps
  running. Watch the pending counter go 2 → 1 → 0.
- **Y**: Delete the material while a load into it is still pending. The load is
  aborted rather than writing into freed memory: it reports `NEA_ASYNC_ERROR`
  and the program keeps running.

## How it works

Loading happens in two phases:

1. A worker cothread reads the file into RAM, yielding between chunks so the
   main loop keeps running.
2. `NEA_AsyncProcess()` runs the finalize step (the texture VRAM upload) on the
   main thread during the vertical blank. It is called automatically because
   `NEA_UPDATE_ASSETS` is passed to `NEA_WaitForVBL()`.

## Object lifetime

A pending load holds a pointer to the object it will write into. Deleting that
object (`NEA_MaterialDelete()`, `NEA_PaletteDelete()`, `NEA_ModelDelete()`, ...)
aborts the load, which then reports `NEA_ASYNC_ERROR`. The handle itself stays
valid, so deleting an object with a load in flight is always safe, but the
handle must still be released with `NEA_AsyncRelease()`.

Never poll `NEA_AsyncGetState()` in a loop that doesn't yield: the worker only
runs while the main thread is waiting. Use `NEA_WaitForVBL()` as usual, or
`NEA_AsyncWait()` if you really need to block.

## DS hardware note

On a DS flashcard, filesystem access through DLDI runs on the ARM9 by default
and blocks it during reads, which prevents the loading from overlapping with the
main loop. This example calls `dldiSetMode(DLDI_MODE_ARM7)` before
`nitroFSInit()` so that reads run on the ARM7 instead. This is not needed on DSi
(SD card access already runs on the ARM7) or when reading from a cartridge.
