# Asynchronous file writing

This example shows how to save a file in the background with
`NEA_FATWriteDataAsync()`, so that the main loop (and therefore audio streaming
and rendering) keeps running while the data is written to the filesystem.

It also demonstrates the crash safety the API gives you: the data is written to
`<path>.temp` and only renamed over the real file once all of it has reached the
filesystem, so an interrupted save can never destroy the previous one.

## Controls

- **A**: Save asynchronously with `NEA_FATWriteDataAsync()`. The on-screen frame
  counter and spinner keep advancing while the file is written.
- **B**: Save synchronously with a plain `fwrite()`. The whole main loop blocks
  until the write finishes, so the counter freezes.
- **X**: Queue a save and cancel it two frames later with `NEA_AsyncRelease()`,
  while the worker is still writing.
- **Y**: Read the file back. It reports the byte the file is filled with, its
  size, and whether a stray `.temp` file was left behind.

Every save writes a different byte over the whole buffer (`'A'`, then `'B'`,
`'C'`...), so the readback tells you which save actually landed.

## The atomicity test

Press **A** (saves `'A'`), then **Y** — the file reads back as `'A'`.

Now press **X** (queues `'B'` and cancels it mid-write), then **Y** again. The
file still reads back as `'A'`, at its full size, with no `+TEMP!` marker: the
cancelled save deleted its temporary file and left the previous one alone. A
naive `fopen(path, "wb")` would have truncated it to a few kilobytes of `'B'`.

## Buffer ownership

`NEA_FATWriteDataAsync()` takes a mode that decides what happens to your buffer:

- `NEA_ASYNC_WRITE_COPY` — the engine copies the data, so you can reuse or free
  your buffer as soon as the call returns. This is what a game usually wants for
  a save state it keeps mutating, and what this example uses.
- `NEA_ASYNC_WRITE_TAKE` — the engine takes ownership of a `malloc()`ed buffer
  and frees it when the write ends. Zero copy, for large one-shot payloads.
- `NEA_ASYNC_WRITE_BORROW` — the engine writes straight out of your buffer. Zero
  copy, but the buffer must stay valid and unmodified until the handle reaches
  `NEA_ASYNC_DONE` or `NEA_ASYNC_ERROR`.

## Filesystem note

Unlike the loading examples, this one needs a **writable** filesystem, so it
calls `fatInitDefault()` rather than `nitroFSInit()`. In an emulator, point it at
a DLDI SD folder or an SD image. On a DS flashcard it calls
`dldiSetMode(DLDI_MODE_ARM7)` first so that the writes actually overlap with the
main loop; this is not needed on DSi, where SD access already runs on the ARM7.
