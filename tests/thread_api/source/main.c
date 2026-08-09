// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Nitro Engine Advanced contributors, 2026
//
// This file is part of Nitro Engine Advanced

// Runtime tests for the NEAThread background task system.
//
// Only failures are printed, so the whole run fits on one console screen and
// every section's result stays readable.

#include <stdio.h>

#include <NEAMain.h>

static int failures = 0;
static int checks = 0;

#define CHECK(cond, name)                                       \
    do {                                                        \
        checks++;                                               \
        if (!(cond))                                            \
        {                                                       \
            printf("  FAIL %s (L%d)\n", name, __LINE__);        \
            failures++;                                         \
        }                                                       \
    } while (0)

#define SECTION(fn)                                             \
    do {                                                        \
        int before_checks = checks;                             \
        int before_fails = failures;                            \
        fn();                                                   \
        printf("%-17s %2d/%2d ok\n", #fn + 5,                   \
               (checks - before_checks) - (failures - before_fails), \
               checks - before_checks);                         \
    } while (0)

// A task that never finishes would hang the test with nothing on screen.
#define PUMP_TIMEOUT_FRAMES 600

static bool TaskFinished(NEA_TaskState s)
{
    return s == NEA_TASK_DONE || s == NEA_TASK_ERROR || s == NEA_TASK_CANCELLED;
}

static NEA_TaskState PumpUntilDone(NEA_Task *task)
{
    for (int i = 0; i < PUMP_TIMEOUT_FRAMES; i++)
    {
        NEA_TaskState s = NEA_TaskGetState(task);
        if (TaskFinished(s))
            return s;
        NEA_WaitForVBL(NEA_UPDATE_TASKS);
    }

    printf("  TIMEOUT state %d\n", (int)NEA_TaskGetState(task));
    return NEA_TASK_ERROR;
}

static void PumpFrames(int count)
{
    for (int i = 0; i < count; i++)
        NEA_WaitForVBL(NEA_UPDATE_TASKS);
}

// ---------------------------------------------------------------------------
// Test tasks
// ---------------------------------------------------------------------------

typedef struct {
    int iterations;     // How much work to do
    int counter;        // Written by the task
    int result;         // What the work function returns
    bool saw_stop;      // The task noticed it was cancelled
    bool done_called;   // The completion callback ran
    NEA_TaskState done_state; // State the callback observed
    bool done_on_main;  // The callback ran outside any task
} TestJob;

// Counts up, yielding as it goes, and stops early if asked to.
static int job_count(void *user)
{
    TestJob *j = user;

    for (int i = 0; i < j->iterations; i++)
    {
        if (NEA_TaskShouldStop())
        {
            j->saw_stop = true;
            return 0;
        }

        j->counter++;
        NEA_TaskSetProgress((i * 1000) / j->iterations);
        NEA_TaskYield();
    }

    return j->result;
}

static void job_done(NEA_Task *task, void *user)
{
    TestJob *j = user;
    j->done_called = true;
    j->done_state = NEA_TaskGetState(task);

    // The callback is documented to run on the main thread, so from its point
    // of view there is no task running.
    j->done_on_main = !NEA_TaskShouldStop() && NEA_TaskGetProgress(task) >= 0;
}

// ---------------------------------------------------------------------------

static void test_basic(void)
{
    TestJob j = { .iterations = 200, .result = 0 };

    NEA_Task *task = NEA_TaskSubmit(job_count, job_done, &j);
    CHECK(task != NULL, "submitted");
    if (task == NULL)
        return;

    CHECK(NEA_TaskPendingCount() == 1, "one pending");
    CHECK(PumpUntilDone(task) == NEA_TASK_DONE, "reached DONE");
    CHECK(j.counter == 200, "work ran to completion");
    CHECK(NEA_TaskGetResult(task) == 0, "result is 0");

    // The completion callback runs from NEA_ThreadProcess(), so it has landed
    // by the time the state was observed as terminal plus one more frame.
    PumpFrames(2);
    CHECK(j.done_called, "completion callback ran");
    CHECK(j.done_state == NEA_TASK_DONE, "callback saw DONE");
    CHECK(j.done_on_main, "callback ran on the main thread");

    NEA_TaskRelease(task);
    PumpFrames(2);
    CHECK(NEA_TaskPendingCount() == 0, "queue drained");
}

static void test_error_result(void)
{
    TestJob j = { .iterations = 10, .result = 42 };

    NEA_Task *task = NEA_TaskSubmit(job_count, job_done, &j);
    CHECK(task != NULL, "submitted");
    if (task == NULL)
        return;

    CHECK(PumpUntilDone(task) == NEA_TASK_ERROR, "non-zero return is ERROR");
    CHECK(NEA_TaskGetResult(task) == 42, "result preserved");
    PumpFrames(2);
    CHECK(j.done_state == NEA_TASK_ERROR, "callback saw ERROR");
    NEA_TaskRelease(task);
}

static void test_cancel_running(void)
{
    // Long enough that it can't finish before the cancel lands.
    TestJob j = { .iterations = 1000000, .result = 0 };

    NEA_Task *task = NEA_TaskSubmit(job_count, job_done, &j);
    CHECK(task != NULL, "submitted");
    if (task == NULL)
        return;

    // Let it actually start.
    PumpFrames(2);
    CHECK(j.counter > 0, "task started running");

    NEA_TaskCancel(task);
    CHECK(PumpUntilDone(task) == NEA_TASK_CANCELLED, "reached CANCELLED");
    CHECK(j.saw_stop, "task observed NEA_TaskShouldStop()");
    CHECK(j.counter < 1000000, "task stopped early");

    PumpFrames(2);
    CHECK(j.done_called, "callback runs for a cancelled task");
    CHECK(j.done_state == NEA_TASK_CANCELLED, "callback saw CANCELLED");

    NEA_TaskRelease(task);
    PumpFrames(2);
    CHECK(NEA_TaskPendingCount() == 0, "queue drained");
}

static void test_cancel_before_start(void)
{
    TestJob j = { .iterations = 100, .result = 0 };

    NEA_Task *task = NEA_TaskSubmit(job_count, job_done, &j);
    CHECK(task != NULL, "submitted");
    if (task == NULL)
        return;

    // Cancelled without ever yielding, so no worker can have picked it up.
    NEA_TaskCancel(task);
    CHECK(NEA_TaskGetState(task) == NEA_TASK_CANCELLED, "cancelled at once");

    PumpFrames(3);
    CHECK(j.counter == 0, "work function never ran");
    CHECK(j.done_called, "callback still ran");

    NEA_TaskRelease(task);
}

// Releasing a running task must cancel it and free the handle without the
// completion callback firing against data the app may have dropped.
static void test_release_running(void)
{
    static TestJob j;
    j = (TestJob){ .iterations = 1000000, .result = 0 };

    NEA_Task *task = NEA_TaskSubmit(job_count, job_done, &j);
    CHECK(task != NULL, "submitted");
    if (task == NULL)
        return;

    PumpFrames(2);
    NEA_TaskRelease(task);   // handle must not be touched after this
    PumpFrames(10);

    CHECK(j.saw_stop, "released task was asked to stop");
    CHECK(!j.done_called, "callback dropped for a released task");
    CHECK(NEA_TaskPendingCount() == 0, "handle reaped");
}

static void test_many_tasks(void)
{
    enum { N = 8 };
    static TestJob jobs[N];
    NEA_Task *tasks[N];

    for (int i = 0; i < N; i++)
    {
        jobs[i] = (TestJob){ .iterations = 50 + i * 10, .result = 0 };
        tasks[i] = NEA_TaskSubmit(job_count, job_done, &jobs[i]);
    }

    bool all_submitted = true;
    for (int i = 0; i < N; i++)
    {
        if (tasks[i] == NULL)
            all_submitted = false;
    }
    CHECK(all_submitted, "more tasks than workers all submitted");
    CHECK(NEA_TaskPendingCount() == N, "all queued");

    bool all_done = true;
    for (int i = 0; i < N; i++)
    {
        if (tasks[i] == NULL)
            continue;
        if (PumpUntilDone(tasks[i]) != NEA_TASK_DONE)
            all_done = false;
    }
    CHECK(all_done, "all finished");

    bool all_ran = true;
    for (int i = 0; i < N; i++)
    {
        if (jobs[i].counter != 50 + i * 10)
            all_ran = false;
    }
    CHECK(all_ran, "every task did its own work");

    PumpFrames(2);
    for (int i = 0; i < N; i++)
    {
        if (tasks[i] != NULL)
            NEA_TaskRelease(tasks[i]);
    }
    PumpFrames(2);
    CHECK(NEA_TaskPendingCount() == 0, "queue drained");
}

static void test_progress(void)
{
    TestJob j = { .iterations = 5000, .result = 0 };

    NEA_Task *task = NEA_TaskSubmit(job_count, NULL, &j);
    CHECK(task != NULL, "submitted");
    if (task == NULL)
        return;

    PumpFrames(2);
    int mid = NEA_TaskGetProgress(task);
    CHECK(mid >= 0 && mid <= 1000, "progress in range");

    CHECK(PumpUntilDone(task) == NEA_TASK_DONE, "finished");
    NEA_TaskRelease(task);
}

// The frame budget bounds how much of a frame the workers may use, so a task
// with a small budget must take more frames than the same task with a big one.
static void test_frame_budget(void)
{
    TestJob a = { .iterations = 4000, .result = 0 };
    TestJob b = { .iterations = 4000, .result = 0 };

    NEA_ThreadSetFrameBudget(8);
    NEA_Task *slow = NEA_TaskSubmit(job_count, NULL, &a);
    int slow_frames = 0;
    while (slow != NULL && !TaskFinished(NEA_TaskGetState(slow))
           && slow_frames < PUMP_TIMEOUT_FRAMES)
    {
        NEA_WaitForVBL(NEA_UPDATE_TASKS);
        slow_frames++;
    }
    if (slow != NULL)
        NEA_TaskRelease(slow);

    NEA_ThreadSetFrameBudget(0); // no limit
    NEA_Task *fast = NEA_TaskSubmit(job_count, NULL, &b);
    int fast_frames = 0;
    while (fast != NULL && !TaskFinished(NEA_TaskGetState(fast))
           && fast_frames < PUMP_TIMEOUT_FRAMES)
    {
        NEA_WaitForVBL(NEA_UPDATE_TASKS);
        fast_frames++;
    }
    if (fast != NULL)
        NEA_TaskRelease(fast);

    CHECK(a.counter == 4000 && b.counter == 4000, "both finished the work");
    CHECK(slow_frames > fast_frames, "a tighter budget takes more frames");

    NEA_ThreadSetFrameBudget(NEA_TASK_BUDGET_DEFAULT);
    PumpFrames(2);
}

static void test_misuse(void)
{
    // A NULL work function is refused rather than crashing a worker.
    CHECK(NEA_TaskSubmit(NULL, NULL, NULL) == NULL, "NULL work refused");

    // The in-task helpers do nothing useful from the main thread, but must not
    // crash or claim a task is running.
    CHECK(!NEA_TaskShouldStop(), "ShouldStop false on main thread");
    NEA_TaskYield();
    NEA_TaskSetProgress(500);
    CHECK(true, "in-task helpers safe on the main thread");
}

// Tearing the pool down with work in flight must stop cleanly, and the system
// must come back up afterwards.
static void test_teardown(void)
{
    static TestJob j;
    j = (TestJob){ .iterations = 1000000, .result = 0 };

    NEA_Task *task = NEA_TaskSubmit(job_count, job_done, &j);
    CHECK(task != NULL, "submitted");
    PumpFrames(2);
    CHECK(j.counter > 0, "task running at teardown");

    // Cancels everything, waits for the workers and frees the pool. The handle
    // is destroyed with the pool, so it must not be used after this.
    NEA_ThreadSystemEnd();
    CHECK(j.saw_stop, "running task was stopped by teardown");

    // Submitting with no pool fails instead of crashing.
    CHECK(NEA_TaskSubmit(job_count, NULL, &j) == NULL,
          "submit refused after teardown");

    CHECK(NEA_ThreadSystemReset(2, 8 * 1024) == 1, "pool comes back up");

    TestJob k = { .iterations = 50, .result = 0 };
    NEA_Task *again = NEA_TaskSubmit(job_count, NULL, &k);
    CHECK(again != NULL, "submit works again");
    if (again != NULL)
    {
        CHECK(PumpUntilDone(again) == NEA_TASK_DONE, "task runs again");
        CHECK(k.counter == 50, "work ran after restart");
        NEA_TaskRelease(again);
    }
}

static void test_stack_peak(void)
{
    int peak = NEA_ThreadStackPeak(0);

#ifdef NEA_DEBUG
    CHECK(peak > 0, "worker 0 reports a stack peak");
    CHECK(peak < 8 * 1024, "peak is below the stack size");
#else
    CHECK(peak == -1, "stack peak unavailable in release builds");
#endif

    CHECK(NEA_ThreadStackPeak(-1) == -1, "bad worker index rejected");
    CHECK(NEA_ThreadStackPeak(99) == -1, "out of range worker rejected");
}

int main(int argc, char *argv[])
{
    irqEnable(IRQ_HBLANK);
    irqSet(IRQ_VBLANK, NEA_VBLFunc);
    irqSet(IRQ_HBLANK, NEA_HBLFunc);

    NEA_Init3D();
    NEA_InitConsole();

    printf("NEAThread tests\n\n");

    if (!NEA_ThreadSystemReset(2, 8 * 1024))
    {
        printf("NEA_ThreadSystemReset failed\n");
        while (1)
            NEA_WaitForVBL(0);
    }

    SECTION(test_basic);
    SECTION(test_error_result);
    SECTION(test_cancel_running);
    SECTION(test_cancel_before_start);
    SECTION(test_release_running);
    SECTION(test_many_tasks);
    SECTION(test_progress);
    SECTION(test_frame_budget);
    SECTION(test_misuse);
    SECTION(test_stack_peak);
    SECTION(test_teardown);

    printf("\n%d checks, %d failed\n", checks, failures);
    if (failures == 0)
        printf("ALL TESTS PASSED\n");
    else
        printf("TESTS FAILED\n");

    while (1)
        NEA_WaitForVBL(0);

    return 0;
}
