// SPDX-License-Identifier: MIT
//
// Copyright (c) 2024-2026 Warioware64
//
// This file is part of Nitro Engine Advanced

#ifndef NEA_THREAD_H__
#define NEA_THREAD_H__

/// @file   NEAThread.h
/// @brief  Background task system.

#include <nds.h>

#include "NEAFAT.h"

/// @defgroup thread_system Background tasks
///
/// Runs a unit of work on a pooled worker thread so that a long job can be
/// spread over many frames without turning it into a state machine by hand.
/// When the work finishes, an optional callback runs on the main thread during
/// the vertical blank, which is where its results may safely be pushed to VRAM.
///
/// @section thread_not_parallel This does not make anything faster
///
/// The threads underneath are BlocksDS cooperative threads on the ARM9. There
/// is one CPU and no time slicing, so nothing here runs in parallel with
/// anything else:
///
/// - A task runs until it yields. A task that never yields freezes the whole
///   game, and no amount of engine code can take the CPU back from it.
/// - Tasks only get to run while the main thread is waiting, which in practice
///   means inside NEA_WaitForVBL().
///
/// What this buys is *responsiveness*, not throughput. A three second job
/// submitted as a task still takes at least three seconds, but the game keeps
/// drawing, keeps streaming audio and keeps reading the pad while it runs, and
/// the player can cancel it. If you want something to take less total time,
/// make the algorithm faster; a task will not do it.
///
/// @section thread_rules What a task may do
///
/// The work function runs on a worker thread. It may:
///
/// - Do arbitrary computation.
/// - Call malloc()/free(). Cooperative scheduling means no other thread can be
///   inside the allocator at the same time.
/// - Read and write memory the app owns, as long as the main thread isn't
///   touching it at the same time.
/// - Access the filesystem.
///
/// It must **not** touch VRAM, the GPU, OAM or the palette regions, or call any
/// NEA_* function that does. Those writes have to happen at a defined point in
/// the frame, which is exactly what the completion callback is for.
///
/// It must call NEA_TaskYield() regularly, and return as soon as it sees
/// NEA_TaskShouldStop().
///
/// @section thread_usage Usage
///
/// Reserve the pool once at startup, next to the other NEA_*SystemReset()
/// calls, and pass NEA_UPDATE_TASKS to NEA_WaitForVBL() so completions run:
///
/// ```
/// NEA_ThreadSystemReset(2, 16 * 1024);
/// ...
/// while (1)
/// {
///     NEA_WaitForVBL(NEA_UPDATE_TASKS);
///     ...
/// }
/// ```
///
/// @{

/// Opaque handle representing one submitted task.
typedef struct NEA_Task NEA_Task;

/// State of a task.
typedef enum {
    NEA_TASK_PENDING = 0, ///< Queued, no worker has picked it up yet.
    NEA_TASK_RUNNING,     ///< A worker is executing it right now.
    NEA_TASK_DONE,        ///< Finished; the work function returned 0.
    NEA_TASK_ERROR,       ///< Finished; the work function returned non-zero.
    NEA_TASK_CANCELLED    ///< Stopped early because it was cancelled.
} NEA_TaskState;

/// Work function, run on a worker thread.
///
/// Call NEA_TaskYield() regularly from any long loop, and return as soon as
/// NEA_TaskShouldStop() is true. See @ref thread_rules for what this function
/// is allowed to touch.
///
/// @param user The pointer passed to NEA_TaskSubmit().
/// @return 0 on success, any other value to report NEA_TASK_ERROR.
typedef int (*NEA_TaskFn)(void *user);

/// Completion callback, run on the main thread by NEA_ThreadProcess().
///
/// It runs during the vertical blank, so this is the place to upload whatever
/// the task produced to VRAM. It is called for every terminal state, including
/// NEA_TASK_CANCELLED, so check NEA_TaskGetState() before using the results.
///
/// @param task The task that finished.
/// @param user The pointer passed to NEA_TaskSubmit().
typedef void (*NEA_TaskDoneFn)(NEA_Task *task, void *user);

/// Reserves the worker pool. Call once, at startup.
///
/// Every stack is allocated here and never grows, so the memory cost of the
/// task system is fixed and visible at startup rather than appearing gradually
/// during play. After this succeeds, NEA_TaskSubmit() can only fail because
/// too many tasks are already live, never because the heap ran out.
///
/// Calling it again tears down the existing pool first. Tasks still in flight
/// are cancelled and waited for.
///
/// @param max_workers Number of worker threads (1 to NEA_MAX_TASK_WORKERS).
///                    More workers only help if tasks block on the filesystem;
///                    for pure computation one is enough.
/// @param stack_size Bytes of stack per worker, rounded up to 8. Needs to be
///                   generous (16 KB or more) if tasks touch the filesystem.
/// @return 1 on success, 0 on error.
int NEA_ThreadSystemReset(int max_workers, size_t stack_size);

/// Stops every worker and frees the pool. Called automatically by NEA_End().
///
/// Tasks that are still running are asked to stop and waited for, so a task
/// that ignores NEA_TaskShouldStop() will hang here. In debug builds this also
/// prints the stack high-water mark of each worker.
void NEA_ThreadSystemEnd(void);

/// Maximum number of workers the pool can be asked for.
#define NEA_MAX_TASK_WORKERS 4

/// Submits a task to be run on a worker thread.
///
/// The returned handle must be released with NEA_TaskRelease() once you are
/// done with it, exactly like an async load handle.
///
/// @param work Function to run on the worker. Must not be NULL.
/// @param done Optional callback run on the main thread when it finishes.
/// @param user Arbitrary pointer passed to both functions.
/// @return Handle to poll the task, or NULL on error.
NEA_Task *NEA_TaskSubmit(NEA_TaskFn work, NEA_TaskDoneFn done, void *user);

/// Asks a task to stop.
///
/// The work function sees NEA_TaskShouldStop() become true and is expected to
/// return promptly; the task then reports NEA_TASK_CANCELLED. A task that has
/// not started yet is cancelled immediately and never runs. The completion
/// callback still runs, so it can clean up whatever the task allocated.
///
/// @param task Task handle.
void NEA_TaskCancel(NEA_Task *task);

/// Releases a task handle.
///
/// If the task is still running it is cancelled first. It is safe to call at
/// any time. After this call the handle must not be used again.
///
/// @param task Task handle.
void NEA_TaskRelease(NEA_Task *task);

/// Returns the current state of a task.
///
/// @param task Task handle.
/// @return The current NEA_TaskState.
NEA_TaskState NEA_TaskGetState(const NEA_Task *task);

/// Returns the value the work function returned.
///
/// Only meaningful once the task has finished.
///
/// @param task Task handle.
/// @return The work function's return value, or -1 if it never ran.
int NEA_TaskGetResult(const NEA_Task *task);

/// Returns the number of tasks that haven't finished yet.
///
/// @return Number of tasks queued or running.
int NEA_TaskPendingCount(void);

/// Gives the rest of the frame's work a chance to run. Call from inside a task.
///
/// This is what makes a task cooperative. Call it often enough that the gap
/// between two calls is short compared to a frame; a few hundred microseconds
/// is a good target. It does nothing if called from outside a task.
void NEA_TaskYield(void);

/// Returns true if the running task has been asked to stop.
///
/// Check this in any long loop and return as soon as it is true. It returns
/// false if called from outside a task.
///
/// @return True if the task should return as soon as possible.
bool NEA_TaskShouldStop(void);

/// Records how far along the running task is, for the main thread to display.
///
/// @param permille Progress from 0 to 1000.
void NEA_TaskSetProgress(int permille);

/// Reads the progress reported by a task.
///
/// @param task Task handle.
/// @return Progress from 0 to 1000.
int NEA_TaskGetProgress(const NEA_Task *task);

/// Runs the completion callbacks of finished tasks and starts queued ones.
///
/// Call once per frame. It is called automatically when NEA_UPDATE_TASKS is
/// passed to NEA_WaitForVBL().
void NEA_ThreadProcess(void);

/// Sets how much of each frame the workers may use, in scanlines.
///
/// Once the tasks running in a frame have used this many scanlines between
/// them, NEA_TaskYield() parks them until the next frame. This bounds the
/// amount by which a task that is mid-run at the vertical blank can delay the
/// main thread.
///
/// The default (NEA_TASK_BUDGET_DEFAULT) suits a game that wants its frame rate
/// untouched. Raise it on a loading screen where finishing sooner matters more
/// than a smooth frame rate, or pass 0 to remove the limit entirely.
///
/// @param scanlines Scanlines per frame, or 0 for no limit. 263 is a frame.
void NEA_ThreadSetFrameBudget(int scanlines);

/// Default per-frame budget for background tasks, in scanlines.
#define NEA_TASK_BUDGET_DEFAULT 96

/// Returns the deepest stack use observed for a worker, in bytes.
///
/// Use it to size NEA_ThreadSystemReset()'s stack_size from measurement. It
/// needs the whole stack to have been filled with a known pattern at startup,
/// so it only works in debug builds.
///
/// Debug builds also warn about a task that runs several frames without
/// yielding, naming the task. To see that (or anything else the engine reports)
/// the *application* must be compiled with -DNEA_DEBUG as well as linked
/// against libNEA_debug.a, and must call NEA_DebugSetHandlerConsole(): without
/// the define, that call compiles to nothing and every message is dropped.
///
/// @param worker Worker index, from 0 to the count passed to
///               NEA_ThreadSystemReset() minus one.
/// @return Peak stack use in bytes, or -1 if it can't be measured.
int NEA_ThreadStackPeak(int worker);

/// @}

/// @cond INTERNAL

// Called from NEA_VBLFunc() on the vertical blank interrupt. Detects a task
// that has stopped yielding, which a cooperative scheduler can't see any other
// way: once a task stops yielding, interrupt handlers are the only code that
// still runs.
//
// It only records the stall; NEA_ThreadProcess() prints it, because printing
// from an interrupt handler doesn't reliably reach the console. That means a
// task which never yields *again* is never reported - but at that point the
// game is hung anyway, and nothing running on the main thread could report it
// either. What this reliably catches is the common case: a task that yields
// too rarely, which costs frame rate rather than hanging outright.
//
// Does nothing outside debug builds.
void __NEA_ThreadStallCheck(void);

/// @endcond

#endif // NEA_THREAD_H__
