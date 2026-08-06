// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Warioware64, 2026
//
// This file is part of Nitro Engine Advanced
//
// Box3D triangle mesh level
// -------------------------
// Eight boxes dropped onto a baked triangle mesh -- a subdivided floor, a ramp
// and four walls -- instead of onto a single box. They slide down the ramp,
// cross the floor and settle to sleep. D-pad pushes the first box, A resets,
// B launches everything upward.
//
// It is box3d_basic with the floor replaced, deliberately: the same eight
// bodies and the same readout, so the microseconds-per-step figure can be read
// against that example's number and the difference attributed to the mesh.
//
// Two things to watch on the bottom screen that box3d_basic does not have:
//
//   drop   Mesh manifolds discarded last step. Should be 0. A non-zero reading
//          means a shape touched the level along more distinct normals than the
//          port preallocates, the shallowest were thrown away, and a body may
//          be sinking into a crease.
//
//   The seams. A body sliding across the floor must not catch on the interior
//   edges between its triangles. That is what the baker's shared-edge
//   classification exists to prevent, and it is why the level's floor is
//   subdivided rather than two big triangles.
//
// The collision mesh is baked offline -- see assets.sh. There is no run-time
// mesh builder: a bounding volume hierarchy on a 67 MHz ARM9 is work for the
// asset pipeline.

#include <NEAMain.h>
#include <NEAPhysics3D.h>

#include <nds/arm9/exceptions.h>

#include "cube_bin.h"
#include "level_bin.h"

// The collision mesh, as a C array in ROM. Not a data/ file: bin2c emits
// aligned(4), b3MeshData opens with a uint64_t, and the ARM9 reads one with
// LDRD -- which faults on a 4-aligned address. obj2dl --collision-b3-c writes
// this one aligned(8).
#include "level_b3mesh.h"

#define NUM_BOXES   8
#define BOX_HALF    0.5f

// The display list was authored at half size to fit v16's 8-unit limit, so the
// model is drawn at 2x to line up with the collision mesh, which is at true
// scale. assets.sh explains where the two scales come from.
#define LEVEL_DRAW_SCALE 2.0f

// Below this, a box counts as lost and is put back at the top of the ramp.
//
// This level has walls on three sides, but the ramp side is open on purpose --
// a level with no way out is not a realistic one. Box3D parks a body that
// leaves b3WorldDef::maximumWorldExtent so a lost box cannot break the
// simulation, but bringing it back is the game's job.
#define RESPAWN_BELOW  (-20.0f)

typedef struct {
    NEA_Camera *camera;
    NEA_Model *model[NUM_BOXES];
    NEA_Model *level_model;
    NEA_Phys3DBody *box[NUM_BOXES];
    NEA_Phys3DBody *level_body;
} SceneData;

void Draw3DScene(void *arg)
{
    SceneData *scene = arg;

    NEA_CameraUse(scene->camera);

    NEA_PolyFormat(31, 0, NEA_LIGHT_0 | NEA_LIGHT_1, NEA_CULL_BACK, 0);

    for (int i = 0; i < NUM_BOXES; i++)
        NEA_ModelDraw(scene->model[i]);

    NEA_PolyFormat(20, 0, NEA_LIGHT_0 | NEA_LIGHT_1, NEA_CULL_BACK, 0);
    NEA_ModelDraw(scene->level_model);
}

// Where box i starts: up the ramp, so every box slides down it and then travels
// across the floor's interior edges before coming to rest. A box dropped
// straight onto flat ground would never test the seams.
static void BoxStartPosition(int i, float *x, float *y, float *z)
{
    *x = (i & 1) ? 1.2f : -1.2f;
    *y = 5.0f + (float)i * 1.4f;
    *z = -9.0f + (float)(i & 2) * 0.4f;
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
    // freezing on the last frame with nothing to go on.
    defaultExceptionHandler();

    // ---------------------------------------------------------------------
    // The physics world
    // ---------------------------------------------------------------------

    NEA_Phys3DWorldDef def = NEA_Phys3DDefaultWorldDef();

    def.box3d.capacity.dynamicBodyCount = NUM_BOXES;
    def.box3d.capacity.dynamicShapeCount = NUM_BOXES;
    def.box3d.capacity.staticBodyCount = 1;
    def.box3d.capacity.staticShapeCount = 1;
    def.box3d.capacity.contactCount = NUM_BOXES * 4;

    // The one capacity a mesh scene needs and a convex one does not: how many
    // shapes may touch the level at once. A mesh contact carries a manifold per
    // contact *normal* where a convex contact carries one in total, and the
    // manifold allocators, the triangle caches and the arena are all sized from
    // this before the first step. Leave it at zero and the first body to land
    // allocates in the middle of a frame.
    //
    // It is also the expensive one: it sizes the per-step solver scratch, and
    // doubling it here doubled NEA_Phys3DWorldGetStackCapacity() from 35 KB to
    // 71 KB and pushed the world out of a 192 KB pool. One per box is what this
    // scene needs; measure before paying for more.
    def.box3d.capacity.meshContactCount = NUM_BOXES;

    // A mesh scene costs more pool than a convex one: the per-shape triangle
    // caches and the extra manifolds meshContactCount buys are all in here.
    // Measured -- the readout's "build" line is what the scene costs once it is
    // constructed, and this leaves room above it.
    def.poolBytes = 192 * 1024;

    if (NEA_Phys3DWorldInit(&def) != 0)
    {
        consoleDemoInit();
        printf("Could not create the physics world.\n");
        while (1)
            swiWaitForVBlank();
    }

    NEA_Phys3DWorldSetGravity(0, -9.8, 0);

    // Camera: back and above, looking down the ramp.
    scene.camera = NEA_CameraCreate();
    NEA_CameraSet(scene.camera,
                  -11, 10, 13,
                    0,  0, -2,
                    0,  1,  0);

    NEA_LightSet(0, NEA_White, 0, -1, -1);
    NEA_LightSet(1, NEA_Blue, -1, 0, 0);

    NEA_ClearColorSet(RGB15(4, 4, 8), 31, 63);

    // ---------------------------------------------------------------------
    // The level: one static body carrying the baked mesh
    // ---------------------------------------------------------------------

    scene.level_model = NEA_ModelCreate(NEA_Static);
    NEA_ModelLoadStaticMesh(scene.level_model, level_bin);
    NEA_ModelSetCoord(scene.level_model, 0, 0, 0);
    NEA_ModelScale(scene.level_model, LEVEL_DRAW_SCALE, LEVEL_DRAW_SCALE,
                   LEVEL_DRAW_SCALE);

    scene.level_body = NEA_Phys3DBodyCreate(b3_staticBody, 0, 0, 0);

    // The blob is not copied -- the shape keeps this pointer -- so it has to
    // outlive every shape referencing it. Here it is a const array in ROM,
    // which outlives everything. The scale is 1: the bake already put the mesh
    // at true size.
    if (NEA_Phys3DBodyAddMesh(scene.level_body, level_b3mesh, 1, 1, 1) != 0)
    {
        consoleDemoInit();
        printf("The level mesh was refused.\n");
        printf("Rebuild it: ./assets.sh\n");
        while (1)
            swiWaitForVBlank();
    }

    NEA_Phys3DBodySetMaterial(scene.level_body, 0.6, 0.1);

    // A static body never moves, so it never emits a move event and its model
    // is never synced. Placed once, by hand, above.

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
        NEA_Phys3DBodyAddBox(scene.box[i], BOX_HALF, BOX_HALF, BOX_HALF, 1.0);
        NEA_Phys3DBodySetMaterial(scene.box[i], 0.5, 0.15);
        NEA_Phys3DBodySetModel(scene.box[i], scene.model[i]);
    }

    int32_t buildBytes = NEA_Phys3DWorldGetMemoryUsage();

    consoleDemoInit();
    printf("NEA Box3D Mesh Level\n");
    printf("--------------------\n");
    printf("D-pad: push box 0\n");
    printf("A: reset   B: launch\n\n");

    uint32_t stepTicks = 0;

    // Peak step cost since boot, and how many bodies were awake when it
    // happened. The settled figure is nearly free, so the peak is the number
    // that sizes a frame budget -- and unlike the instantaneous reading it can
    // still be read off the screen once the scene has come to rest.
    uint32_t peakTicks = 0;
    int peakAwake = 0;

    // The drop count is per step, so a peak is the only way to see one that
    // happened while a box was landing and was gone by the time you looked.
    int peakDrops = 0;

    // Boxes recovered from off the open side of the level.
    int respawns = 0;

    while (1)
    {
        scanKeys();
        uint32_t keys = keysHeld();
        uint32_t down = keysDown();

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
        {
            ResetScene(&scene);
            peakDrops = 0;
        }

        // Step the world by hand and time it. NEA_WaitForVBL(NEA_UPDATE_PHYS3D)
        // would do the same thing, but timing it needs the call in the open.
        cpuStartTiming(0);
        NEA_Phys3DWorldStep();
        stepTicks = cpuEndTiming();

        NEA_Phys3DSyncModels();

        // Anything that went off the open side comes back.
        for (int i = 0; i < NUM_BOXES; i++)
        {
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

        int drops = NEA_Phys3DWorldGetMeshManifoldDropCount();
        if (drops > peakDrops)
            peakDrops = drops;

        // cpuStartTiming(0) counts at 33.514 MHz, so ticks / 33.514 is
        // microseconds. Multiply before dividing to keep it in integers.
        uint32_t stepMicros = (stepTicks * 1000u) / 33514u;
        uint32_t peakMicros = (peakTicks * 1000u) / 33514u;

        // Each line positions its own cursor rather than relying on newlines.
        // Printing a newline on the last row scrolls the console, which shifts
        // every subsequent frame up by one.
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
        printf("\x1b[13;0Hdrop   %5d now, %d peak     ", drops, peakDrops);
        printf("\x1b[14;0Hmesh   %5d bytes            ", level_b3mesh_size);
        printf("\x1b[15;0Hcpu %3d %%  fps %2d             ",
               NEA_GetCPUPercent(), NEA_GetFPS());
        printf("\x1b[16;0Hrespawn %4d                ", respawns);

        NEA_WaitForVBL(NEA_CAN_SKIP_VBL);
        NEA_ProcessArg(Draw3DScene, &scene);
    }

    return 0;
}
