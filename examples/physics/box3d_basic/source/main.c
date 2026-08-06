// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Warioware64, 2026
//
// This file is part of Nitro Engine Advanced
//
// Box3D physics example
// ---------------------
// Eight boxes dropped onto a static floor. They collide, stack and settle to
// sleep. D-pad pushes the first box, A resets the scene, B launches everything
// upward.
//
// The bottom screen prints what this example exists to measure: microseconds
// per step, how many bodies are awake, and how much of the physics memory pool
// is in use. Those are the numbers to size a real scene from.
//
// Note the Makefile: Box3D lives in libNEA_box3d.a, which a project links by
// name. Nothing here is pulled into a ROM that does not ask for it.

#include <NEAMain.h>
#include <NEAPhysics3D.h>

#include <nds/arm9/exceptions.h>

#include "cube_bin.h"

#define NUM_BOXES   8
#define BOX_HALF    0.5f
#define FLOOR_HALF  8.0f

// Walls around the floor, so the player cannot push a box out of the world.
//
// These once froze the ROM in about forty seconds. Not their fault: four static
// bodies were enough to give the scene a second simultaneous awake island, and
// b3Solve's sleep loop stepped its index back after every b3TrySleepIsland
// whether or not the island had actually slept -- so an island that declined
// (as one with a pending split does) was retried at the same index forever.
// tests/box3d_host/test_world.c's `eight boxes in a walled pit` is this scene,
// and it is what keeps that fixed.
#define NUM_WALLS   4
#define WALL_HALF   0.5f
#define WALL_HEIGHT 1.5f

// Below this, a box counts as lost and is put back at the top.
//
// A game needs *something* here. Box3D parks a body that leaves
// b3WorldDef::maximumWorldExtent -- see that field -- so a lost box no longer
// corrupts anything, but it also never comes back on its own. This is the other
// half: notice it and respawn it.
#define RESPAWN_BELOW  (-20.0f)

typedef struct {
    NEA_Camera *camera;
    NEA_Model *model[NUM_BOXES];
    NEA_Model *floor_model;
    NEA_Model *wall_model[4];
    NEA_Phys3DBody *box[NUM_BOXES];
    NEA_Phys3DBody *floor_body;
} SceneData;

void Draw3DScene(void *arg)
{
    SceneData *scene = arg;

    NEA_CameraUse(scene->camera);

    NEA_PolyFormat(31, 0, NEA_LIGHT_0 | NEA_LIGHT_1, NEA_CULL_BACK, 0);

    for (int i = 0; i < NUM_BOXES; i++)
        NEA_ModelDraw(scene->model[i]);

    NEA_PolyFormat(20, 0, NEA_LIGHT_0 | NEA_LIGHT_1, NEA_CULL_BACK, 0);
    NEA_ModelDraw(scene->floor_model);

    for (int i = 0; i < NUM_WALLS; i++)
        NEA_ModelDraw(scene->wall_model[i]);
}

// Where box i starts. Two columns of four, offset so they do not land as a
// perfect tower -- a tower that never tips proves less than one that does.
static void BoxStartPosition(int i, float *x, float *y, float *z)
{
    *x = (i & 1) ? 0.6f : -0.6f;
    *y = 1.5f + (float)i * 1.4f;
    *z = (i & 2) ? 0.35f : -0.35f;
}

static void ResetScene(SceneData *scene)
{
    for (int i = 0; i < NUM_BOXES; i++)
    {
        float x, y, z;
        BoxStartPosition(i, &x, &y, &z);

        NEA_Phys3DBodySetPosition(scene->box[i], x, y, z);
        NEA_Phys3DBodySetVelocity(scene->box[i], 0, 0, 0);
        NEA_Phys3DBodySetAwake(scene->box[i], true);
    }
}

int main(int argc, char *argv[])
{
    SceneData scene = { 0 };

    irqEnable(IRQ_HBLANK);
    irqSet(IRQ_VBLANK, NEA_VBLFunc);
    irqSet(IRQ_HBLANK, NEA_HBLFunc);

    NEA_Init3D();

    // A data abort prints registers and the faulting address instead of
    // freezing on the last frame with nothing to go on. Cheap, and the
    // difference between a bug report and a shrug.
    defaultExceptionHandler();

    // ---------------------------------------------------------------------
    // The physics world
    // ---------------------------------------------------------------------
    //
    // The capacities are not a hint. Every pool -- bodies, shapes, contacts,
    // islands, the broad-phase trees, the per-step solver scratch -- is sized
    // from them before the first step, and NEA_Phys3DWorldGetLateAllocCount()
    // reports anything that allocates afterwards. Count the scene and add
    // headroom.

    NEA_Phys3DWorldDef def = NEA_Phys3DDefaultWorldDef();

    def.box3d.capacity.dynamicBodyCount = NUM_BOXES;
    def.box3d.capacity.dynamicShapeCount = NUM_BOXES;
    // The floor plus four walls, one body and one shape each, with headroom.
    // Sized exactly at 5 the last two walls were silently not created --
    // NEA_Phys3DBodyCreate returns NULL when the pool is full and this loop did
    // not look, so boxes escaped through the two missing sides.
    def.box3d.capacity.staticBodyCount = 8;
    def.box3d.capacity.staticShapeCount = 8;
    // Eight boxes in a heap touch each other and the floor. 4 pairs per box is
    // generous; being generous here costs bytes, being short costs a frame.
    def.box3d.capacity.contactCount = NUM_BOXES * 4;

    // 96 KB was enough before the walls. Four more static bodies and their
    // hulls, and the contacts eight boxes make against them, are not free:
    // at 96 KB the pool ran dry *during scene construction*, the box bodies
    // were created into an exhausted allocator, and the result was a demo
    // showing eight boxes that never moved and a 2 us step time.
    def.poolBytes = 160 * 1024;

    if (NEA_Phys3DWorldInit(&def) != 0)
    {
        consoleDemoInit();
        printf("Could not create the physics world.\n");
        while (1)
            swiWaitForVBlank();
    }

    NEA_Phys3DWorldSetGravity(0, -9.8, 0);

    // Camera
    scene.camera = NEA_CameraCreate();
    NEA_CameraSet(scene.camera,
                  -7, 6, 7,
                   0, 2, 0,
                   0, 1, 0);

    // Lights
    NEA_LightSet(0, NEA_White, 0, -1, -1);
    NEA_LightSet(1, NEA_Blue, -1, 0, 0);

    NEA_ClearColorSet(RGB15(4, 4, 8), 31, 63);

    // ---------------------------------------------------------------------
    // The floor: one static body with a wide flat box shape
    // ---------------------------------------------------------------------

    scene.floor_model = NEA_ModelCreate(NEA_Static);
    NEA_ModelLoadStaticMesh(scene.floor_model, cube_bin);
    NEA_ModelSetCoord(scene.floor_model, 0, -0.25, 0);
    NEA_ModelScaleI(scene.floor_model, floattof32(FLOOR_HALF), inttof32(1) / 4,
                    floattof32(FLOOR_HALF));

    scene.floor_body = NEA_Phys3DBodyCreate(b3_staticBody, 0, -0.25, 0);
    NEA_Phys3DBodyAddBox(scene.floor_body, FLOOR_HALF, 0.25, FLOOR_HALF, 1.0);
    NEA_Phys3DBodySetMaterial(scene.floor_body, 0.6, 0.1);

    // Walls, so the player cannot push a box out of the world.
    //
    // Worth having in the *example* even though the engine now copes: Box3D
    // parks a body that leaves b3WorldDef::maximumWorldExtent, so a lost box no
    // longer corrupts anything -- but a box parked 16,000 units away has still
    // left the game. An enclosed play area is what a real level does.
    //
    // One static body per wall, because NEA_Phys3DBodyAddBox centres its shape
    // on the body origin; a body is the cheapest way to place one somewhere
    // else.
    int wallsMissing = 0;
    int boxesMissing = 0;
    const float wallOffset = FLOOR_HALF + WALL_HALF;
    const float wallPos[4][3] = {
        {  wallOffset, WALL_HEIGHT, 0 },
        { -wallOffset, WALL_HEIGHT, 0 },
        { 0, WALL_HEIGHT,  wallOffset },
        { 0, WALL_HEIGHT, -wallOffset },
    };
    const float wallSize[4][3] = {
        { WALL_HALF, WALL_HEIGHT, FLOOR_HALF + 2 * WALL_HALF },
        { WALL_HALF, WALL_HEIGHT, FLOOR_HALF + 2 * WALL_HALF },
        { FLOOR_HALF + 2 * WALL_HALF, WALL_HEIGHT, WALL_HALF },
        { FLOOR_HALF + 2 * WALL_HALF, WALL_HEIGHT, WALL_HALF },
    };

    for (int i = 0; i < NUM_WALLS; i++)
    {
        NEA_Phys3DBody *wall = NEA_Phys3DBodyCreate(b3_staticBody, wallPos[i][0],
                                                    wallPos[i][1], wallPos[i][2]);
        if (wall == NULL)
        {
            // Worth checking rather than assuming: a wall that quietly failed
            // to exist is a hole in the level that looks like a physics bug.
            wallsMissing++;
            continue;
        }

        NEA_Phys3DBodyAddBox(wall, wallSize[i][0], wallSize[i][1],
                             wallSize[i][2], 1.0);
        NEA_Phys3DBodySetMaterial(wall, 0.6, 0.1);

        scene.wall_model[i] = NEA_ModelCreate(NEA_Static);
        NEA_ModelLoadStaticMesh(scene.wall_model[i], cube_bin);
        NEA_ModelSetCoord(scene.wall_model[i], wallPos[i][0], wallPos[i][1],
                          wallPos[i][2]);
        NEA_ModelScaleI(scene.wall_model[i], floattof32(wallSize[i][0]),
                        floattof32(wallSize[i][1]), floattof32(wallSize[i][2]));
    }

    // A static body never moves, so it never emits a move event and the model
    // is never synced. Place it once, by hand, above.

    // ---------------------------------------------------------------------
    // The boxes
    // ---------------------------------------------------------------------

    for (int i = 0; i < NUM_BOXES; i++)
    {
        scene.model[i] = NEA_ModelCreate(NEA_Static);
        NEA_ModelLoadStaticMesh(scene.model[i], cube_bin);
        NEA_ModelScaleI(scene.model[i], floattof32(BOX_HALF),
                        floattof32(BOX_HALF), floattof32(BOX_HALF));

        float x, y, z;
        BoxStartPosition(i, &x, &y, &z);

        scene.box[i] = NEA_Phys3DBodyCreate(b3_dynamicBody, x, y, z);
        if (scene.box[i] == NULL)
        {
            // Same reason the walls are checked: a body that quietly failed to
            // exist looks like a physics bug, not a sizing one.
            boxesMissing++;
            continue;
        }

        NEA_Phys3DBodyAddBox(scene.box[i], BOX_HALF, BOX_HALF, BOX_HALF, 1.0);
        NEA_Phys3DBodySetMaterial(scene.box[i], 0.5, 0.15);
        NEA_Phys3DBodySetModel(scene.box[i], scene.model[i]);
    }

    // Everything is built. Whatever the pool holds now is the cost of the
    // scene; anything it grows by later is the simulation allocating, which is
    // what the "late" counter below reports.
    int32_t buildBytes = NEA_Phys3DWorldGetMemoryUsage();

    consoleDemoInit();

    printf("NEA Box3D Demo\n");
    printf("--------------\n");
    printf("D-pad: push box 0\n");
    printf("A: reset   B: launch\n\n");

    uint32_t stepTicks = 0;

    // Peak step cost since boot, and how many bodies were awake when it
    // happened. The settled figure is nearly free (everything is asleep), so
    // the peak is the number that actually sizes a frame budget -- and unlike
    // the instantaneous reading it can still be read off the screen once the
    // scene has come to rest.
    uint32_t peakTicks = 0;
    int peakAwake = 0;

    // Should stay at 0 with the walls in. A rising count means something is
    // escaping the play area.
    int respawns = 0;

    // Counts frames that completed a step, and nothing else. Every other line
    // of the readout goes constant once the scene falls asleep -- step time,
    // tick count, pool, cpu, all of them -- so a screenshot of a settled scene
    // and a screenshot of a frozen ROM are the same image. That ambiguity cost
    // a day during the softlock hunt. This is the one number that separates
    // them, so it is deliberately the last thing computed before it is printed.
    uint32_t frames = 0;

    while (1)
    {
        scanKeys();
        uint32_t keys = keysHeld();
        uint32_t down = keysDown();

#ifdef BOX3D_NO_INPUT
        // Baseline measurement build: melonDS injects stray keyboard events into
        // the focused window, and in this scene those land on the D-pad (push
        // box 0) and B (launch every box), which knocks boxes around the pit and
        // lifts both the pool high-water mark and the late-allocation counter.
        // Gating the input is what makes the recorded numbers a property of the
        // simulation rather than of the emulator.
        keys = 0;
        down = 0;
#endif

        if (keys & KEY_RIGHT)
            NEA_Phys3DBodyApplyForce(scene.box[0], 40, 0, 0);
        if (keys & KEY_LEFT)
            NEA_Phys3DBodyApplyForce(scene.box[0], -40, 0, 0);
        if (keys & KEY_UP)
            NEA_Phys3DBodyApplyForce(scene.box[0], 0, 0, -40);
        if (keys & KEY_DOWN)
            NEA_Phys3DBodyApplyForce(scene.box[0], 0, 0, 40);

        if (down & KEY_B)
        {
            for (int i = 0; i < NUM_BOXES; i++)
                NEA_Phys3DBodyApplyImpulse(scene.box[i], 0, 4, 0);
        }

        if (down & KEY_A)
            ResetScene(&scene);

        // Step the world by hand and time it. NEA_WaitForVBL(NEA_UPDATE_PHYS3D)
        // would do the same thing, but timing it needs the call in the open --
        // which is why the wait below passes NEA_CAN_SKIP_VBL instead. Setting
        // both would step the world twice per frame.
        cpuStartTiming(0);
        NEA_Phys3DWorldStep();
        stepTicks = cpuEndTiming();
        frames++;

        NEA_Phys3DSyncModels();

        // Catch anything that got out anyway.
        //
        // The walls make this hard to trigger by hand, which is the point --
        // but a game should have it regardless, because the engine's own
        // safety net (b3WorldDef::maximumWorldExtent) only stops a lost body
        // from breaking the simulation. Bringing it back is the game's job.
        for (int i = 0; i < NUM_BOXES; i++)
        {
            if (scene.box[i] == NULL)
                continue;

            int32_t bx, by, bz;
            NEA_Phys3DBodyGetPositionI(scene.box[i], &bx, &by, &bz);

            if (by < floattof32(RESPAWN_BELOW))
            {
                float x, y, z;
                BoxStartPosition(i, &x, &y, &z);

                NEA_Phys3DBodySetPosition(scene.box[i], x, y, z);
                NEA_Phys3DBodySetVelocity(scene.box[i], 0, 0, 0);
                NEA_Phys3DBodySetAwake(scene.box[i], true);
                respawns++;
            }
        }

        int awake = NEA_Phys3DWorldGetAwakeBodyCount();
        if (stepTicks > peakTicks)
        {
            peakTicks = stepTicks;
            peakAwake = awake;
        }

        // cpuStartTiming(0) counts at 33.514 MHz, so ticks / 33.514 is
        // microseconds. Multiply before dividing to keep it in integers.
        // The timer runs off the bus clock, which is the same on DS and DSi,
        // so a tick is the same amount of real time on both.
        uint32_t stepMicros = (stepTicks * 1000u) / 33514u;
        uint32_t peakMicros = (peakTicks * 1000u) / 33514u;

        // Each line positions its own cursor rather than relying on newlines.
        // Printing a newline on the last row scrolls the console, which shifts
        // every subsequent frame up by one and leaves the previous frame's
        // text interleaved with this one.
        printf("\x1b[6;0Hstep   %5lu us (%lu ticks)  ",
               (unsigned long)stepMicros, (unsigned long)stepTicks);
        printf("\x1b[7;0Hpeak   %5lu us @ %d awake   ",
               (unsigned long)peakMicros, peakAwake);
        printf("\x1b[8;0Hawake  %2d / %d bodies       ", awake, NUM_BOXES + 1);
        printf("\x1b[9;0Hpool   %5ld / %ld bytes     ",
               (long)NEA_Phys3DWorldGetMemoryUsage(),
               (long)NEA_Phys3DWorldGetMemoryCapacity());
        printf("\x1b[10;0Hbuild  %5ld bytes           ", (long)buildBytes);
        printf("\x1b[11;0Hstack  %5ld / %ld bytes     ",
               (long)NEA_Phys3DWorldGetStackUsage(),
               (long)NEA_Phys3DWorldGetStackCapacity());
        printf("\x1b[12;0Hlate   %5d allocs, %ld ovf  ",
               NEA_Phys3DWorldGetLateAllocCount(),
               (long)NEA_Phys3DWorldGetOverflowBytes());
        printf("\x1b[13;0Hcpu %3d %%  fps %2d             ",
               NEA_GetCPUPercent(), NEA_GetFPS());
        printf("\x1b[14;0Hrespawn %4d  missing %d/%d  ", respawns, wallsMissing, boxesMissing);
        printf("\x1b[15;0Hframe  %8lu               ", (unsigned long)frames);

        NEA_WaitForVBL(NEA_CAN_SKIP_VBL);
        NEA_ProcessArg(Draw3DScene, &scene);
    }

    return 0;
}
