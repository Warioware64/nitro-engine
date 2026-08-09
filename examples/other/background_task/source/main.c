// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Nitro Engine Advanced contributors, 2026
//
// This file is part of Nitro Engine Advanced

// Example of the background task system.
//
// The same long computation is run two ways so the difference is visible on
// screen. A sliding block and a frame counter are updated every frame:
//
//   - Press A to run it as a background task (NEA_TaskSubmit). The block keeps
//     sliding and the frame counter keeps climbing while a progress bar fills.
//     Press B at any time to cancel it.
//
//   - Press X to run exactly the same work synchronously. The whole game
//     freezes until it finishes: no spinning, no counter, no response to the
//     pad. This is what the task system exists to avoid.
//
//   - Press Y to switch between the default frame budget and no budget at all.
//     With no budget the task finishes in fewer frames but the frame rate drops,
//     which is the trade the budget exists to make.
//
// Note that a task does not make the work *faster* - there is one CPU and
// cothreads are cooperative. It makes the game stay responsive while it runs.

#include <stdio.h>

#include <NEAMain.h>

// How much work to do. Big enough to take a few seconds on hardware.
#define WORK_ITEMS 200000

typedef struct {
    int x;      // Animated every frame, so a freeze is obvious on screen
    int phase;
} SceneData;

// The result the computation produces, so there is something to hand back.
typedef struct {
    uint32_t checksum;
    int items_done;
} WorkResult;

static WorkResult g_result;
static bool g_have_result = false;
static const char *g_last_event = "none";

void Draw3DScene(void *arg)
{
    SceneData *Scene = arg;

    NEA_2DViewInit();

    // A block that slides back and forth. If the game ever stops responding,
    // this is what visibly stops moving.
    NEA_2DDrawQuad(Scene->x, 32, Scene->x + 48, 32 + 48, 0, NEA_Red);
}

// A deliberately slow checksum, so there is something real to spread over
// frames. The only thing that makes it a good task is that it yields.
static uint32_t ne_mix(uint32_t x)
{
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    return x;
}

// Runs on a worker thread.
static int compute_task(void *user)
{
    WorkResult *out = user;
    uint32_t sum = 1;

    for (int i = 0; i < WORK_ITEMS; i++)
    {
        // Give the rest of the frame a chance to run. Without this the game
        // would freeze exactly like the synchronous version below.
        if ((i & 255) == 0)
        {
            if (NEA_TaskShouldStop())
            {
                out->items_done = i;
                return 1;   // report it as unfinished
            }

            NEA_TaskSetProgress((i * 1000) / WORK_ITEMS);
            NEA_TaskYield();
        }

        sum = ne_mix(sum + i);
    }

    out->checksum = sum;
    out->items_done = WORK_ITEMS;
    return 0;
}

// Runs on the main thread during the vertical blank. This is where it would be
// safe to upload whatever the task produced to VRAM.
static void compute_done(NEA_Task *task, void *user)
{
    (void)user;

    switch (NEA_TaskGetState(task))
    {
        case NEA_TASK_DONE:
            g_have_result = true;
            g_last_event = "finished";
            break;
        case NEA_TASK_CANCELLED:
            g_last_event = "cancelled";
            break;
        default:
            g_last_event = "failed";
            break;
    }
}

// The same work, run directly on the main thread for comparison.
static void compute_blocking(void)
{
    uint32_t sum = 1;
    for (int i = 0; i < WORK_ITEMS; i++)
        sum = ne_mix(sum + i);

    g_result.checksum = sum;
    g_result.items_done = WORK_ITEMS;
    g_have_result = true;
}

int main(int argc, char *argv[])
{
    SceneData Scene = { 0 };

    irqEnable(IRQ_HBLANK);
    irqSet(IRQ_VBLANK, NEA_VBLFunc);
    irqSet(IRQ_HBLANK, NEA_HBLFunc);

    NEA_Init3D();
    NEA_InitConsole();

    // Reserve the worker pool once, up front. Two workers is plenty here; one
    // would do, since this task is pure computation.
    if (!NEA_ThreadSystemReset(2, 8 * 1024))
    {
        printf("Couldn't create the task pool\n");
        while (1)
            NEA_WaitForVBL(0);
    }

    NEA_Task *task = NULL;
    bool budget_on = true;
    int frame = 0;
    int task_frames = 0;
    int last_task_frames = 0;
    const char spinner[4] = { '|', '/', '-', '\\' };

    while (1)
    {
        // NEA_UPDATE_TASKS runs the completion callbacks of finished tasks.
        NEA_WaitForVBL(NEA_UPDATE_TASKS);
        NEA_ProcessArg(Draw3DScene, &Scene);

        frame++;
        Scene.phase = (Scene.phase + 3) & 255;
        // Triangle wave across the screen.
        int t = Scene.phase < 128 ? Scene.phase : 255 - Scene.phase;
        Scene.x = 16 + (t * 3) / 2;

        scanKeys();
        uint32_t keys = keysDown();

        if ((keys & KEY_A) && task == NULL)
        {
            g_have_result = false;
            task_frames = 0;
            task = NEA_TaskSubmit(compute_task, compute_done, &g_result);
            g_last_event = task ? "task started" : "submit failed";
        }

        if ((keys & KEY_B) && task != NULL)
            NEA_TaskCancel(task);

        if (keys & KEY_X)
        {
            // Blocking version: nothing else runs until this returns.
            g_last_event = "ran on main thread";
            compute_blocking();
        }

        if (keys & KEY_Y)
        {
            budget_on = !budget_on;
            NEA_ThreadSetFrameBudget(budget_on ? NEA_TASK_BUDGET_DEFAULT : 0);
        }

        if (task != NULL)
        {
            task_frames++;

            NEA_TaskState state = NEA_TaskGetState(task);
            if (state != NEA_TASK_PENDING && state != NEA_TASK_RUNNING)
            {
                last_task_frames = task_frames;
                NEA_TaskRelease(task);
                task = NULL;
            }
        }

        consoleClear();
        printf("Background tasks\n");
        printf("================\n\n");
        printf("Frame:    %d %c\n", frame, spinner[frame & 3]);
        printf("FPS:      %d\n\n", NEA_GetFPS());

        if (task != NULL)
        {
            int permille = NEA_TaskGetProgress(task);

            // A 20 character progress bar.
            char bar[21];
            int filled = (permille * 20) / 1000;
            for (int i = 0; i < 20; i++)
                bar[i] = i < filled ? '#' : '.';
            bar[20] = '\0';

            printf("Running:  [%s]\n", bar);
            printf("Progress: %d.%d%%\n", permille / 10, permille % 10);
        }
        else
        {
            printf("Running:  -\n");
            printf("Progress: -\n");
        }

        printf("Took:     %d frames\n", last_task_frames);
        printf("Budget:   %s\n", budget_on ? "default" : "unlimited");
        printf("Result:   ");
        if (g_have_result)
            printf("%08lX\n", (unsigned long)g_result.checksum);
        else
            printf("-\n");
        printf("Last:     %s\n\n", g_last_event);

        printf("A: run as a task (smooth)\n");
        printf("B: cancel it\n");
        printf("X: run blocking (freezes!)\n");
        printf("Y: toggle frame budget\n");
    }

    return 0;
}
