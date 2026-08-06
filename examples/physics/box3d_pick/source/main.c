// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Warioware64, 2026
//
// This file is part of Nitro Engine Advanced
//
// Box3D world queries: ray casts and box overlaps
// -----------------------------------------------
// The same baked triangle level as box3d_pick's parent example, with four boxes
// resting on it, and a ray fired into the scene every frame from a point you
// steer with the D-pad. Whatever the ray hits is marked and reported.
//
// This is the example for Phase 7's query layer, and it exercises all of it in
// one ROM:
//
//   * NEA_Phys3DRayCast against a **triangle mesh**, which is b3RayCastMesh --
//     the BVH descent, the fixed-point ray/triangle test and the front-to-back
//     traversal that makes it cheap.
//   * The same call against **convex shapes**, which is b3RayCastHull, so the
//     one entry point covers both and the readout says which it found.
//   * NEA_Phys3DOverlapBox, which is b3World_OverlapAABB, counting the boxes
//     inside a region that follows the hit point around.
//
// What to watch on the bottom screen:
//
//   nodes  BVH nodes and leaves the ray visited. This is the number that says
//          whether a per-frame ray is affordable on a 67 MHz ARM9, and it is
//          why b3RayCastMesh descends front-to-back: each triangle it hits
//          shortens the segment, so the rest of the tree is rejected rather
//          than walked. Aim at the far wall and watch it climb; aim at the
//          floor under the camera and watch it fall.
//
//   tri    The triangle index within the level mesh, or -1 for a box. Free
//          evidence that the mesh path and the convex path are the same call.
//
//   us     Microseconds for the cast alone, not the step.

#include <NEAMain.h>
#include <NEAPhysics3D.h>

#include <nds/arm9/exceptions.h>

#include "cube_bin.h"
#include "level_bin.h"

#include "level_b3mesh.h"

#define NUM_BOXES   4
#define BOX_HALF    0.5f

// Matches box3d_level: the display list was authored at half size to fit v16's
// 8-unit limit, so the model draws at 2x to line up with the collision mesh.
#define LEVEL_DRAW_SCALE 2.0f

// Where the ray starts: directly above the middle of the level, where the four
// boxes come to rest. So the default aim lands on a **box** and the convex path
// is what the example shows first; steering away from centre walks off them and
// onto the mesh, which is the point -- one call, two shape kinds, and the `tri`
// readout is what tells them apart.
#define EYE_X   0.0f
#define EYE_Y   9.0f
#define EYE_Z   0.0f

// How long the ray is. Also its unit: NEA_Phys3DRayCast takes a direction whose
// *length* is the ray's length, and reports a fraction of it -- so a hit at
// fraction 0.25 of a 40-unit ray is 10 units away.
#define RAY_LENGTH  40.0f

typedef struct {
    NEA_Camera *camera;
    NEA_Model *model[NUM_BOXES];
    NEA_Model *level_model;
    NEA_Model *marker;
    NEA_Phys3DBody *box[NUM_BOXES];
    NEA_Phys3DBody *level_body;

    // The body the ray is currently on, or NULL for the level. Drawn brighter.
    NEA_Phys3DBody *picked;

    bool showMarker;
} SceneData;

void Draw3DScene(void *arg)
{
    SceneData *scene = arg;

    NEA_CameraUse(scene->camera);

    for (int i = 0; i < NUM_BOXES; i++)
    {
        // The picked box is drawn at full brightness against the others, so the
        // ray's answer is visible on the top screen and not only in the numbers.
        bool lit = scene->box[i] == scene->picked;
        NEA_PolyFormat(31, 0, lit ? (NEA_LIGHT_0 | NEA_LIGHT_1) : NEA_LIGHT_1,
                       NEA_CULL_BACK, 0);
        NEA_ModelDraw(scene->model[i]);
    }

    NEA_PolyFormat(20, 0, NEA_LIGHT_0 | NEA_LIGHT_1, NEA_CULL_BACK, 0);
    NEA_ModelDraw(scene->level_model);

    // The hit point itself.
    if (scene->showMarker)
    {
        NEA_PolyFormat(31, 0, NEA_LIGHT_0 | NEA_LIGHT_1, NEA_CULL_NONE, 0);
        NEA_ModelDraw(scene->marker);
    }
}

static void BoxStartPosition(int i, float *x, float *y, float *z)
{
    *x = (i & 1) ? 2.0f : -2.0f;
    *y = 4.0f + (float)i * 1.2f;
    *z = (i & 2) ? 2.0f : -2.0f;
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

// Counts bodies inside the overlap box. Returning true keeps the query going,
// which is what makes the count a total rather than a "found one".
static bool CountBody(NEA_Phys3DBody *body, void *context)
{
    (void)body;
    (void)context;
    return true;
}

int main(int argc, char *argv[])
{
    SceneData scene = { 0 };

    irqEnable(IRQ_HBLANK);
    irqSet(IRQ_VBLANK, NEA_VBLFunc);
    irqSet(IRQ_HBLANK, NEA_HBLFunc);

    NEA_Init3D();
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
    def.box3d.capacity.meshContactCount = NUM_BOXES;

    // Queries allocate nothing -- every one of them runs on the trees and the
    // blob that already exist, with its context on the C stack. So this is
    // box3d_level's pool scaled to half the bodies.
    //
    // The evidence for "nothing" is on the readout, and it is not that `late`
    // reads zero: it reads a small constant from building and settling the
    // scene, the same as every other physics example here. It is that the
    // number **does not move** while a ray cast and a box overlap run every
    // frame. Measured on device: 16 after the boxes land, still 16 some 2,400
    // frames later.
    def.poolBytes = 160 * 1024;

    if (NEA_Phys3DWorldInit(&def) != 0)
    {
        consoleDemoInit();
        printf("Could not create the physics world.\n");
        while (1)
            swiWaitForVBlank();
    }

    NEA_Phys3DWorldSetGravity(0, -9.8, 0);

    scene.camera = NEA_CameraCreate();
    NEA_CameraSet(scene.camera,
                  -13, 12, 15,
                    0,  0, -2,
                    0,  1,  0);

    NEA_LightSet(0, NEA_White, 0, -1, -1);
    NEA_LightSet(1, NEA_Blue, -1, 0, 0);

    NEA_ClearColorSet(RGB15(4, 4, 8), 31, 63);

    // ---------------------------------------------------------------------
    // The level
    // ---------------------------------------------------------------------

    scene.level_model = NEA_ModelCreate(NEA_Static);
    NEA_ModelLoadStaticMesh(scene.level_model, level_bin);
    NEA_ModelSetCoord(scene.level_model, 0, 0, 0);
    NEA_ModelScale(scene.level_model, LEVEL_DRAW_SCALE, LEVEL_DRAW_SCALE,
                   LEVEL_DRAW_SCALE);

    scene.level_body = NEA_Phys3DBodyCreate(b3_staticBody, 0, 0, 0);

    if (NEA_Phys3DBodyAddMesh(scene.level_body, level_b3mesh, 1, 1, 1) != 0)
    {
        consoleDemoInit();
        printf("The level mesh was refused.\n");
        printf("Rebuild it: ./assets.sh\n");
        while (1)
            swiWaitForVBlank();
    }

    NEA_Phys3DBodySetMaterial(scene.level_body, 0.6, 0.1);

    // ---------------------------------------------------------------------
    // The boxes, and the hit marker
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

    // A small cube reused as the hit marker. Not a physics body -- it is a
    // readout, and giving it a shape would let it collide with what it marks.
    scene.marker = NEA_ModelCreate(NEA_Static);
    NEA_ModelLoadStaticMesh(scene.marker, cube_bin);
    NEA_ModelScale(scene.marker, 0.15, 0.15, 0.15);

    int32_t buildBytes = NEA_Phys3DWorldGetMemoryUsage();

    consoleDemoInit();
    printf("NEA Box3D Ray Cast\n");
    printf("------------------\n");
    printf("D-pad: aim   A: reset\n");
    printf("B: drop the boxes\n\n");

    // The ray's aim, as an offset from straight down. Steered with the D-pad
    // and clamped, so the ray always points somewhere into the level.
    // Aimed at the box that settles near (-2, -2).
    int aimX = -4;
    int aimZ = -4;

    uint32_t peakNodes = 0;

    while (1)
    {
        scanKeys();
        uint32_t keys = keysHeld();
        uint32_t down = keysDown();

        if (keys & KEY_RIGHT) aimX += 1;
        if (keys & KEY_LEFT)  aimX -= 1;
        if (keys & KEY_UP)    aimZ -= 1;
        if (keys & KEY_DOWN)  aimZ += 1;

        if (aimX < -14) aimX = -14;
        if (aimX >  14) aimX =  14;
        if (aimZ < -14) aimZ = -14;
        if (aimZ >  14) aimZ =  14;

        if (down & KEY_B)
        {
            for (int i = 0; i < NUM_BOXES; i++)
                NEA_Phys3DBodyApplyImpulse(scene.box[i], 0, 4, 0);
        }

        if (down & KEY_A)
        {
            ResetScene(&scene);
            peakNodes = 0;
        }

        NEA_Phys3DWorldStep();
        NEA_Phys3DSyncModels();

        // -----------------------------------------------------------------
        // The ray
        // -----------------------------------------------------------------
        //
        // Direction is not normalized on purpose: its length is the ray's
        // length, so `fraction` reads back against RAY_LENGTH directly.

        NEA_Phys3DRayHit hit;

        int32_t dirX = inttof32(aimX);
        int32_t dirY = floattof32(-RAY_LENGTH * 0.5f);
        int32_t dirZ = inttof32(aimZ);

        cpuStartTiming(0);
        NEA_Phys3DRayCastI(floattof32(EYE_X), floattof32(EYE_Y), floattof32(EYE_Z),
                           dirX, dirY, dirZ, &hit);
        uint32_t castTicks = cpuEndTiming();

        // The direction is not a unit vector and its length changes as the aim
        // does, so `fraction` alone is not a distance. Measure the direction
        // and scale by it -- which is the whole reason the API hands back a
        // fraction rather than a length it would have had to compute anyway.
        int32_t rayLen = sqrtf32(mulf32(dirX, dirX) + mulf32(dirY, dirY)
                                 + mulf32(dirZ, dirZ));

        // Which body, if any. A mesh hit reports the level, whose triangleIndex
        // is a real index; a box hit reports -1 there. That is the whole of the
        // difference at this level of the API.
        scene.picked = NULL;
        scene.showMarker = hit.hit;

        if (hit.hit)
        {
            NEA_ModelSetCoordI(scene.marker, hit.x, hit.y, hit.z);

            if (hit.triangleIndex < 0)
            {
                // A convex shape. Find which box by position, since the ray
                // API reports geometry rather than a NEA_Phys3DBody -- see the
                // overlap query below for the call that does report bodies.
                for (int i = 0; i < NUM_BOXES; i++)
                {
                    int32_t bx, by, bz;
                    NEA_Phys3DBodyGetPositionI(scene.box[i], &bx, &by, &bz);

                    int32_t dx = bx - hit.x, dy = by - hit.y, dz = bz - hit.z;
                    int32_t r = floattof32(BOX_HALF * 1.8f);

                    if (dx > -r && dx < r && dy > -r && dy < r && dz > -r && dz < r)
                    {
                        scene.picked = scene.box[i];
                        break;
                    }
                }
            }
        }

        if ((uint32_t)hit.nodeVisits > peakNodes)
            peakNodes = (uint32_t)hit.nodeVisits;

        // -----------------------------------------------------------------
        // The overlap box, following the hit point
        // -----------------------------------------------------------------

        int nearby = 0;
        if (hit.hit)
        {
            const int32_t reach = floattof32(3.0f);
            nearby = NEA_Phys3DOverlapBoxI(hit.x - reach, hit.y - reach, hit.z - reach,
                                           hit.x + reach, hit.y + reach, hit.z + reach,
                                           CountBody, NULL);
        }

        uint32_t castMicros = (castTicks * 1000u) / 33514u;

        printf("\x1b[6;0Haim    %3d, %3d               ", aimX, aimZ);

        if (hit.hit)
        {
            printf("\x1b[7;0Hhit    %s                  ",
                   hit.triangleIndex < 0 ? "box  " : "level");
            printf("\x1b[8;0Hpoint  %6d %6d %6d   ",
                   (int)(hit.x >> 12), (int)(hit.y >> 12), (int)(hit.z >> 12));
            printf("\x1b[9;0Hnormal %6d %6d %6d   ",
                   (int)hit.nx, (int)hit.ny, (int)hit.nz);
            printf("\x1b[10;0Hdist   %5d mm            ",
                   (int)((((int64_t)hit.fraction * rayLen) >> 12) * 1000 >> 12));
            printf("\x1b[11;0Htri    %5d                ", hit.triangleIndex);
        }
        else
        {
            printf("\x1b[7;0Hhit    none                ");
            printf("\x1b[8;0H                            ");
            printf("\x1b[9;0H                            ");
            printf("\x1b[10;0H                           ");
            printf("\x1b[11;0H                           ");
        }

        printf("\x1b[12;0Hnodes  %4d n, %3d lf, pk %3lu ",
               hit.nodeVisits, hit.leafVisits, (unsigned long)peakNodes);
        printf("\x1b[13;0Hcast   %5lu us              ",
               (unsigned long)castMicros);
        printf("\x1b[14;0Hnearby %5d bodies           ", nearby);
        printf("\x1b[15;0Hpool   %5ld / %ld bytes      ",
               (long)NEA_Phys3DWorldGetMemoryUsage(),
               (long)NEA_Phys3DWorldGetMemoryCapacity());
        printf("\x1b[16;0Hbuild  %5ld bytes            ", (long)buildBytes);
        printf("\x1b[17;0Hlate   %5d allocs, %ld ovf   ",
               NEA_Phys3DWorldGetLateAllocCount(),
               (long)NEA_Phys3DWorldGetOverflowBytes());
        printf("\x1b[18;0Hcpu    %3d %%                 ", NEA_GetCPUPercent());

        NEA_WaitForVBL(NEA_CAN_SKIP_VBL);
        NEA_ProcessArg(Draw3DScene, &scene);
    }

    return 0;
}
