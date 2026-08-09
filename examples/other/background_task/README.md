# Background tasks

This example runs the same long computation two ways so the difference is
visible on screen: as a background task, and directly on the main thread.

## Controls

- **A**: Run it as a background task with `NEA_TaskSubmit()`. The sliding block
  keeps moving, the frame counter keeps climbing and the FPS stays at 60 while a
  progress bar fills.
- **B**: Cancel the running task. It stops at the next `NEA_TaskShouldStop()`
  check and reports `NEA_TASK_CANCELLED`.
- **X**: Run exactly the same work synchronously. Everything freezes until it
  finishes — no movement, no counter, no response to the pad.
- **Y**: Toggle between the default frame budget and no budget at all.

## What this does and does not buy you

The threads underneath are BlocksDS cooperative threads on the ARM9. There is
one CPU and no preemption, so **a task is not faster than the same work run
inline** — press A and X and compare the "Took: N frames" figure against how
long the freeze lasts. The totals are in the same ballpark.

What changes is that the game stays alive while the work happens: it draws, it
reads the pad, it can show progress, and the player can cancel. That is the
entire point of the feature.

## The frame budget

With **Y** you can watch the trade directly. With the default budget the workers
stop once they have used their slice of the frame, so the frame rate is
untouched and the task takes more frames. With no budget the task finishes in
fewer frames but the frame rate drops, because a worker that is mid-run when the
vertical blank arrives keeps the main thread waiting.

A loading screen wants no budget. A game running behind the task wants one.

## Writing a task

The work function runs on a worker thread and must:

- Call `NEA_TaskYield()` regularly — this is what keeps the game responsive.
  Without it the game freezes exactly like the **X** case.
- Return as soon as `NEA_TaskShouldStop()` is true.
- Never touch VRAM, the GPU, OAM or the palettes. Do that in the completion
  callback, which runs on the main thread during the vertical blank.

In a debug build (`-DNEA_DEBUG` on the app plus `-lNEA_debug`, and a call to
`NEA_DebugSetHandlerConsole()`) the engine names any task that runs several
frames without yielding, which is the quickest way to find a missing
`NEA_TaskYield()`.
