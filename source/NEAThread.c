// SPDX-License-Identifier: MIT
//
// Copyright (c) 2024-2026 Warioware64
//
// This file is part of Nitro Engine Advanced

#include <malloc.h>

#include <nds/cothread.h>

#include "NEAMain.h"

/// @file NEAThread.c

// Scanlines in a full frame, including the vertical blank. REG_VCOUNT counts
// 0-262 and wraps, so this is also its period.
#define NEA_TASK_SCANLINES_PER_FRAME 263

// Pattern the worker stacks are filled with, so that the untouched part of a
// stack can be told apart from the used part. Only in debug builds: filling
// tens of kilobytes at startup isn't free.
#define NEA_TASK_STACK_PATTERN 0xA5A5A5A5

// Vertical blanks a task may go without yielding before the stall detector
// reports it. Two frames is long enough not to fire on a task that is merely
// slow, short enough to catch a hang while the screen is still updating.
#define NEA_TASK_STALL_FRAMES 2

// One worker thread and the stack it owns for the lifetime of the pool.
typedef struct {
    void *stack;            // Owned by the pool, not by the scheduler
    size_t stack_size;
    // Only ever compared against cothread_get_current(), to work out which
    // worker is running. Never passed to cothread_delete() or
    // cothread_has_joined(): these threads are detached, and they live as long
    // as the pool does, so the ID can't go stale underneath us.
    cothread_t thread;
    // Set by the pool when the thread is created, cleared by the thread itself
    // as its very last action. Workers are detached, so cothread_has_joined()
    // can't be used: this flag is what tells the main thread that a worker has
    // really finished.
    volatile bool running;

    // Everything below is per-worker on purpose. Several workers interleave at
    // yield points, so a single global would have one worker's yield restore
    // another's state: the wrong task would answer NEA_TaskShouldStop(), and
    // the budget would be charged from another worker's mark.
    NEA_Task *current;      // Task this worker is running, or NULL
    int budget_mark;        // REG_VCOUNT when this worker last resumed
    volatile uint32_t last_yield_vbl; // ne_vbl_counter at its last yield
} ne_task_worker;

struct NEA_Task {
    NEA_TaskFn work;
    NEA_TaskDoneFn done;
    void *user;

    volatile NEA_TaskState state;
    // Set by NEA_TaskCancel() or NEA_TaskRelease(). The work function sees it
    // through NEA_TaskShouldStop() and is expected to return.
    volatile bool stop_requested;
    // Set by NEA_TaskRelease() while the task is still running: the app has
    // given up the handle, so it is destroyed once the worker lets go of it.
    volatile bool released;
    // Set once the worker is finished with the task and the main thread still
    // has to run the completion callback.
    volatile bool needs_completion;

    int result;
    volatile int progress;  // 0-1000

    NEA_Task *next;
};

static ne_task_worker ne_workers[NEA_MAX_TASK_WORKERS];
static int ne_worker_count = 0;
static bool ne_thread_inited = false;
// Set by NEA_ThreadSystemEnd() to make the worker loops return.
static volatile bool ne_thread_shutdown = false;

// All live task handles, in submission order.
static NEA_Task *ne_task_list = NULL;

// Finds the worker running right now, so that the in-task helpers don't need
// the caller to pass anything around. Returns NULL on the main thread, which is
// what makes NEA_TaskYield() and friends safe to call from anywhere.
static ne_task_worker *ne_current_worker(void)
{
    cothread_t self = cothread_get_current();

    for (int i = 0; i < ne_worker_count; i++)
    {
        if (ne_workers[i].running && ne_workers[i].thread == self)
            return &ne_workers[i];
    }

    return NULL;
}

static int ne_frame_budget = NEA_TASK_BUDGET_DEFAULT;
// Scanlines every worker has used between them so far this frame. Shared on
// purpose: the budget is a slice of the frame, not a per-worker allowance.
static int ne_budget_used = 0;

// Incremented by the vertical blank handler, sampled by the stall detector.
static volatile uint32_t ne_vbl_counter = 0;
#ifdef NEA_DEBUG
// A stall spotted by the vertical blank handler, waiting to be reported.
//
// The interrupt only records it. Printing from an interrupt handler doesn't
// reach the console reliably, so the message is emitted by NEA_ThreadProcess()
// on the main thread instead.
static volatile bool ne_stall_pending = false;
static NEA_Task *ne_stall_task = NULL;
static int ne_stall_worker = 0;
static uint32_t ne_stall_frames = 0;
// So that one badly behaved task is reported once, not every frame.
static bool ne_stall_reported = false;
#endif

// Every wait below uses a signal ID unique to one worker, and the code that
// wakes them walks the pool sending to each in turn. That is deliberate.
//
// libnds' cothread_send_signal() mishandles two threads waiting on the *same*
// signal ID: when it unlinks a match it leaves its "previous" pointer aimed at
// the node it just removed, so a second match behind a non-matching waiter is
// marked awake but left in the wait list. The next time that thread parks, its
// next_signal ends up pointing into a list that already contains it, and the
// list becomes a cycle. Giving each worker its own IDs sidesteps it entirely.
//
// libnds reserves bit 31 of signal IDs, and these all live in main RAM, so
// their addresses are directly usable as IDs.

// Signal a worker parks on when there is nothing to do.
static uint32_t ne_wake_signal(const ne_task_worker *w)
{
    return (uint32_t)(uintptr_t)w;
}

// Wakes every worker parked waiting for something to do.
static void ne_wake_all_workers(void)
{
    for (int i = 0; i < ne_worker_count; i++)
        cothread_send_signal(ne_wake_signal(&ne_workers[i]));
}

// Scanlines elapsed since 'mark', accounting for REG_VCOUNT wrapping once per
// frame. A gap longer than a frame can't be told apart from a short one, which
// is fine: the budget is reset every frame anyway.
static int ne_scanlines_since(int mark)
{
    int now = REG_VCOUNT;
    int delta = now - mark;
    if (delta < 0)
        delta += NEA_TASK_SCANLINES_PER_FRAME;
    return delta;
}

// Removes a task from the global list.
static void ne_task_unlink(NEA_Task *task)
{
    if (ne_task_list == task)
    {
        ne_task_list = task->next;
        return;
    }

    for (NEA_Task *it = ne_task_list; it != NULL; it = it->next)
    {
        if (it->next == task)
        {
            it->next = task->next;
            return;
        }
    }
}

// Picks the next task waiting to be run, or NULL.
static NEA_Task *ne_task_pop_pending(void)
{
    for (NEA_Task *task = ne_task_list; task != NULL; task = task->next)
    {
        if (task->state != NEA_TASK_PENDING)
            continue;

        // Cancelled before anything picked it up: it never runs, but the main
        // thread still has to run its completion callback.
        if (task->stop_requested)
        {
            task->state = NEA_TASK_CANCELLED;
            task->needs_completion = true;
            continue;
        }

        return task;
    }

    return NULL;
}

// Worker thread entrypoint. Runs tasks until the pool is torn down.
static int ne_task_worker_entry(void *arg)
{
    ne_task_worker *self = arg;

    while (!ne_thread_shutdown)
    {
        NEA_Task *task = ne_task_pop_pending();
        if (task == NULL)
        {
            // Nothing to do: park until something is submitted. The main thread
            // also sends this signal on shutdown so this doesn't sleep forever.
            cothread_yield_signal(ne_wake_signal(self));
            continue;
        }

        task->state = NEA_TASK_RUNNING;
        self->current = task;
        self->last_yield_vbl = ne_vbl_counter;
        self->budget_mark = REG_VCOUNT;

        int ret = task->work != NULL ? task->work(task->user) : 0;

        // Charge whatever the task ran since its last yield to this frame.
        ne_budget_used += ne_scanlines_since(self->budget_mark);
        self->current = NULL;

        task->result = ret;
        if (task->stop_requested)
            task->state = NEA_TASK_CANCELLED;
        else
            task->state = ret == 0 ? NEA_TASK_DONE : NEA_TASK_ERROR;

        // Must be the last thing this worker touches in the task: from here on
        // the main thread may run its completion callback and free it.
        task->needs_completion = true;
    }

    // Must be the last thing this thread touches: it is what tells
    // NEA_ThreadSystemEnd() that this worker is really finished and its stack
    // can be freed. The scheduler deletes the thread itself once this returns,
    // because it is detached.
    self->running = false;
    return 0;
}

int NEA_ThreadSystemReset(int max_workers, size_t stack_size)
{
    if (ne_thread_inited)
        NEA_ThreadSystemEnd();

    if (max_workers < 1 || max_workers > NEA_MAX_TASK_WORKERS)
    {
        NEA_DebugPrint("Invalid worker count: %d", max_workers);
        return 0;
    }

    // cothread_create_manual() requires a 64 bit aligned size.
    stack_size = (stack_size + 7) & ~(size_t)7;
    if (stack_size < 1024)
    {
        NEA_DebugPrint("Worker stack too small: %d", (int)stack_size);
        return 0;
    }

    memset(ne_workers, 0, sizeof(ne_workers));
    ne_task_list = NULL;
    ne_thread_shutdown = false;
    ne_frame_budget = NEA_TASK_BUDGET_DEFAULT;
    ne_budget_used = 0;
#ifdef NEA_DEBUG
    ne_stall_reported = false;
    ne_stall_pending = false;
#endif

    // Everything is allocated here so that submitting a task later can't fail
    // for lack of memory.
    for (int i = 0; i < max_workers; i++)
    {
        ne_task_worker *w = &ne_workers[i];

        w->stack = memalign(8, stack_size);
        if (w->stack == NULL)
        {
            NEA_DebugPrint("Not enough memory for worker stack %d", i);
            ne_worker_count = i;
            ne_thread_inited = true;
            NEA_ThreadSystemEnd();
            return 0;
        }
        w->stack_size = stack_size;

#ifdef NEA_DEBUG
        // Fill the stack so that the untouched part can be recognised later.
        uint32_t *words = w->stack;
        for (size_t j = 0; j < stack_size / 4; j++)
            words[j] = NEA_TASK_STACK_PATTERN;
#endif

        // Two things are going on here:
        //
        // cothread_create_manual() lets the pool own the stack. It leaves the
        // scheduler's stack_base NULL, so the scheduler frees the thread's own
        // bookkeeping but never touches our stack.
        //
        // COTHREAD_DETACHED is not optional. The scheduler caches the *next*
        // thread in its list before resuming one, so deleting a thread from
        // another thread can leave it resuming freed memory. Detached threads
        // are deleted by the scheduler itself, right after they return, which
        // is the one case it handles correctly. Nothing here may ever call
        // cothread_delete().
        w->running = true;
        w->thread = cothread_create_manual(ne_task_worker_entry, w,
                                           w->stack, stack_size,
                                           COTHREAD_DETACHED);
        if (w->thread == -1)
        {
            w->running = false;
            NEA_DebugPrint("Couldn't create worker thread %d", i);
            free(w->stack);
            w->stack = NULL;
            ne_worker_count = i;
            ne_thread_inited = true;
            NEA_ThreadSystemEnd();
            return 0;
        }
    }

    ne_worker_count = max_workers;
    ne_thread_inited = true;
    return 1;
}

void NEA_ThreadSystemEnd(void)
{
    if (!ne_thread_inited)
        return;

    // Ask everything to stop: the tasks so their work functions return, then
    // the workers so their loops exit.
    for (NEA_Task *task = ne_task_list; task != NULL; task = task->next)
        task->stop_requested = true;

    ne_thread_shutdown = true;

    // Let the workers run to completion. They only see the shutdown flag when
    // they get the CPU, so this yields until every one of them has cleared its
    // own 'running' flag on the way out.
    //
    // A task that ignores NEA_TaskShouldStop() never lets go, and this waits
    // for it forever -- which is the documented consequence of writing one.
    // Deleting the thread instead is not an option: see the comment next to
    // COTHREAD_DETACHED in NEA_ThreadSystemReset().
    bool any_running = true;
    while (any_running)
    {
        any_running = false;
        for (int i = 0; i < ne_worker_count; i++)
        {
            if (ne_workers[i].running)
                any_running = true;
        }

        if (any_running)
        {
            // Wake anything parked waiting for work or for the next frame, then
            // give them the CPU so they can see the shutdown flag.
            ne_wake_all_workers();
            cothread_yield();
        }
    }

#ifdef NEA_DEBUG
    // Reported now that the workers have stopped, so the figure covers every
    // task they ever ran.
    for (int i = 0; i < ne_worker_count; i++)
    {
        int peak = NEA_ThreadStackPeak(i);
        if (peak >= 0)
        {
            NEA_DebugPrint("Task worker %d peaked at %d of %d bytes of stack",
                           i, peak, (int)ne_workers[i].stack_size);
        }
    }
#endif

    // Every worker has stopped, so no task can still be in use.
    NEA_Task *task = ne_task_list;
    while (task != NULL)
    {
        NEA_Task *next = task->next;
        free(task);
        task = next;
    }
    ne_task_list = NULL;

    for (int i = 0; i < ne_worker_count; i++)
    {
        free(ne_workers[i].stack);
        ne_workers[i].stack = NULL;
    }

    ne_worker_count = 0;
    ne_thread_inited = false;
    ne_thread_shutdown = false;
}

NEA_Task *NEA_TaskSubmit(NEA_TaskFn work, NEA_TaskDoneFn done, void *user)
{
    NEA_AssertPointer(work, "NULL work function");

    if (!ne_thread_inited)
    {
        NEA_DebugPrint("Task system not initialized, "
                       "call NEA_ThreadSystemReset() first");
        return NULL;
    }

    if (work == NULL)
        return NULL;

    NEA_Task *task = calloc(1, sizeof(NEA_Task));
    if (task == NULL)
    {
        NEA_DebugPrint("Not enough memory");
        return NULL;
    }

    task->work = work;
    task->done = done;
    task->user = user;
    task->state = NEA_TASK_PENDING;
    task->result = -1;

    // Appended at the end so that tasks are picked up in submission order.
    if (ne_task_list == NULL)
    {
        ne_task_list = task;
    }
    else
    {
        NEA_Task *tail = ne_task_list;
        while (tail->next != NULL)
            tail = tail->next;
        tail->next = task;
    }

    // Wake the workers so one of them picks this up at the next yield.
    ne_wake_all_workers();

    return task;
}

void NEA_TaskCancel(NEA_Task *task)
{
    if (task == NULL)
        return;

    task->stop_requested = true;

    // If it hasn't started there is no work function to notice the flag, so
    // finish it here and let the main thread run its callback.
    if (task->state == NEA_TASK_PENDING)
    {
        task->state = NEA_TASK_CANCELLED;
        task->needs_completion = true;
    }
}

void NEA_TaskRelease(NEA_Task *task)
{
    if (task == NULL)
        return;

    // Still owned by a worker: mark it and let NEA_ThreadProcess() free it once
    // the worker has let go. Dropping the callback here stops it running
    // against user data the app has probably just freed.
    if (task->state == NEA_TASK_RUNNING || task->state == NEA_TASK_PENDING)
    {
        NEA_TaskCancel(task);
        task->released = true;
        task->done = NULL;
        return;
    }

    ne_task_unlink(task);
    free(task);
}

NEA_TaskState NEA_TaskGetState(const NEA_Task *task)
{
    NEA_AssertPointer(task, "NULL task pointer");
    if (task == NULL)
        return NEA_TASK_ERROR;
    return task->state;
}

int NEA_TaskGetResult(const NEA_Task *task)
{
    NEA_AssertPointer(task, "NULL task pointer");
    if (task == NULL)
        return -1;
    return task->result;
}

int NEA_TaskPendingCount(void)
{
    int count = 0;
    for (NEA_Task *task = ne_task_list; task != NULL; task = task->next)
    {
        if (task->state == NEA_TASK_PENDING || task->state == NEA_TASK_RUNNING)
            count++;
    }
    return count;
}

void NEA_TaskYield(void)
{
    ne_task_worker *self = ne_current_worker();
    if (self == NULL || self->current == NULL)
        return;

    // Charge the run since the last yield to this frame's budget.
    ne_budget_used += ne_scanlines_since(self->budget_mark);

    // Tell the stall detector this task is still alive.
    self->last_yield_vbl = ne_vbl_counter;

    if (ne_frame_budget > 0 && ne_budget_used >= ne_frame_budget)
    {
        // This frame's slice is spent, so wait for the next vertical blank
        // rather than taking time the main thread wants. Background work then
        // costs loading time instead of frame rate.
        //
        // This waits on the interrupt directly rather than on a signal sent by
        // NEA_ThreadProcess(). "Until the next frame" is exactly what an
        // IRQ_VBLANK wait means, and it needs no cooperation from the main
        // thread.
        //
        // It also keeps the workers off the cothread signal list. An earlier
        // version parked them there and hung as soon as a second worker joined
        // in. libnds keeps one list per interrupt and unlinks from it with a
        // pointer-to-pointer walk, which is correct for any number of waiters;
        // the signal list is a single list unlinked with a trailing "previous"
        // pointer that is left aimed at the node it just removed. This path is
        // the one that is safe by construction.
        cothread_yield_irq(IRQ_VBLANK);
        self->last_yield_vbl = ne_vbl_counter;
    }
    else
    {
        cothread_yield();
    }

    self->budget_mark = REG_VCOUNT;
}

bool NEA_TaskShouldStop(void)
{
    ne_task_worker *self = ne_current_worker();
    if (self == NULL || self->current == NULL)
        return false;
    return self->current->stop_requested || ne_thread_shutdown;
}

void NEA_TaskSetProgress(int permille)
{
    ne_task_worker *self = ne_current_worker();
    if (self == NULL || self->current == NULL)
        return;

    NEA_Task *task = self->current;

    if (permille < 0)
        permille = 0;
    if (permille > 1000)
        permille = 1000;

    task->progress = permille;
}

int NEA_TaskGetProgress(const NEA_Task *task)
{
    NEA_AssertPointer(task, "NULL task pointer");
    if (task == NULL)
        return 0;
    return task->progress;
}

void NEA_ThreadProcess(void)
{
    if (!ne_thread_inited)
        return;

    // Open a new budget window and release whatever parked when the last one
    // ran out.
    // Workers waiting out an exhausted budget wake on the vertical blank by
    // themselves, so opening the new window is just resetting the counter.
    ne_budget_used = 0;

#ifdef NEA_DEBUG
    // Report a stall the vertical blank handler spotted. Doing it here, on the
    // main thread, is what makes the message actually appear: printing from an
    // interrupt handler doesn't reliably reach the console.
    if (ne_stall_pending)
    {
        ne_stall_pending = false;
        ne_stall_reported = true;
        NEA_DebugPrint("Task %p on worker %d ran %d frames without yielding. "
                       "Call NEA_TaskYield() more often.",
                       ne_stall_task, ne_stall_worker, (int)ne_stall_frames);
    }
#endif

    // Run the completion callbacks. A callback may release its own handle and
    // any other, which unlinks and frees them, so the scan restarts from the
    // head after every one instead of holding a 'next' pointer across the call.
    bool finished_one = true;
    while (finished_one)
    {
        finished_one = false;

        for (NEA_Task *task = ne_task_list; task != NULL; task = task->next)
        {
            if (!task->needs_completion)
                continue;

            task->needs_completion = false;
            finished_one = true;

            if (task->done != NULL)
                task->done(task, task->user);

            break;
        }
    }

    // Free the handles the app gave up while they were still running.
    NEA_Task *task = ne_task_list;
    while (task != NULL)
    {
        NEA_Task *next = task->next;
        if (task->released && task->state != NEA_TASK_RUNNING
            && !task->needs_completion)
        {
            ne_task_unlink(task);
            free(task);
        }
        task = next;
    }

    // Wake the workers if there is anything left that still needs running.
    if (NEA_TaskPendingCount() > 0)
        ne_wake_all_workers();
}

void NEA_ThreadSetFrameBudget(int scanlines)
{
    if (scanlines < 0)
        scanlines = 0;
    ne_frame_budget = scanlines;
}

int NEA_ThreadStackPeak(int worker)
{
#ifndef NEA_DEBUG
    (void)worker;
    return -1;
#else
    if (worker < 0 || worker >= ne_worker_count)
        return -1;

    ne_task_worker *w = &ne_workers[worker];
    if (w->stack == NULL)
        return -1;

    // The stack grows down from the top, so the untouched region is at the
    // bottom. Find where the fill pattern stops.
    const uint32_t *words = w->stack;
    size_t total = w->stack_size / 4;
    size_t untouched = 0;
    while (untouched < total && words[untouched] == NEA_TASK_STACK_PATTERN)
        untouched++;

    return (int)((total - untouched) * 4);
#endif
}

void __NEA_ThreadStallCheck(void)
{
    // Runs on the vertical blank interrupt, which is the only thing that still
    // runs when a task stops yielding: at that point the scheduler never gets
    // the CPU back, so no ordinary engine code could ever notice.
    ne_vbl_counter++;

#ifdef NEA_DEBUG
    if (ne_stall_reported || ne_stall_pending)
        return;

    // Check every worker: with more than one, the stalled task isn't
    // necessarily the one that yielded most recently.
    for (int i = 0; i < ne_worker_count; i++)
    {
        const ne_task_worker *w = &ne_workers[i];
        if (!w->running || w->current == NULL)
            continue;

        uint32_t stalled = ne_vbl_counter - w->last_yield_vbl;
        if (stalled < NEA_TASK_STALL_FRAMES)
            continue;

        // Record it and get out. The message itself is printed by
        // NEA_ThreadProcess(), because printing from an interrupt handler
        // doesn't reliably reach the console.
        ne_stall_task = w->current;
        ne_stall_worker = i;
        ne_stall_frames = stalled;
        ne_stall_pending = true;
        return;
    }
#endif
}
