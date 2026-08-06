// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Warioware64, 2026
//
// This file is part of Nitro Engine Advanced
//
// Box3D character mover: walking a capsule on a triangle level
// ------------------------------------------------------------
// A player you steer with the D-pad, on the same baked triangle level the
// other physics examples use, with a staircase to climb, a wall to run into
// and a few crates to shove around.
//
// The player is **not a rigid body**, and that is the whole point of this
// example. A player made of a dynamic body has to fight the solver for every
// step, slope and ledge, and loses: it slides down ramps, tips over, spins
// when it clips a crate corner, and no amount of friction and angular damping
// fixes it, because the solver is doing exactly what it is asked to. A mover
// decides where the capsule goes and then puts it there:
//
//   * NEA_Phys3DMoverStepYaw() runs the whole controller -- friction,
//     acceleration, gravity, a ground probe, and a slide loop that
//     depenetrates and sweeps up to five times.
//   * Underneath it, b3World_CollideMover collects the collision planes,
//     b3SolvePlanes finds the translation that satisfies all of them at once,
//     and b3World_CastMover sweeps along the result.
//
// What to watch on the bottom screen:
//
//   ground  Whether the ground probe found floor under the capsule. This is a
//           downward ray driving a soft spring rather than a contact test, and
//           it is what climbs the staircase with no step-up hack and no
//           walkable-slope threshold: a 0.2-unit step is a 0.2-unit spring
//           compression. Walk off the edge and watch it drop to 0 before the
//           fall is visible.
//
//   planes  Collision planes the last step solved against. One for open floor,
//           two in a corner, more against the mesh -- the mesh backend emits
//           one per *triangle*, not one per surface. If this ever reads 8 it
//           has saturated B3_NEA_MAX_MOVER_PLANES; `drop` below is what says so
//           for certain.
//
//   solver  Gauss-Seidel iterations summed over the slide loop. Reaching the
//           per-call cap of 20 is ordinary, not a failure -- measured over
//           160,000 random plane sets it happens 52% of the time, and the same
//           scenarios in double precision reach it 52% of the time too.
//
//   slide   Slide-loop iterations actually run, 1 to 5. **The number to
//           watch.** Pinned at 5 means the mover never reached its target on
//           any frame, which is what a jammed character looks like from the
//           outside; 1 in the open is what walking should cost.
//
//   drop    Shapes whose plane batch saturated. A dropped plane is a surface
//           the mover is never pushed away from -- so it is a wall walked
//           through, not a body sinking slowly. It should read 0 always.
//
//   port    Teleports. b3Body_SetTransform is not free: the body's shapes
//           re-enter the broad phase and re-form their island, which is where
//           a late allocation on an otherwise quiet frame comes from. Press A
//           to respawn and watch this and `late` move together.
//
//   mover   Microseconds for the mover alone, separate from `step`. The two
//           are different budgets: `step` is the simulation and `mover` is one
//           character on top of it.
//
// Build with -DBOX3D_NO_INPUT for a hands-off measurement run, or
// -DBOX3D_AUTO_WALK to have it drive itself in a circuit.

#include <NEAMain.h>
#include <NEAPhysics3D.h>

#include <nds/arm9/exceptions.h>

#include "cube_bin.h"
#include "level_bin.h"

#include "level_b3mesh.h"

// Matches box3d_level and box3d_pick: the display list was authored at half
// size to fit v16's 8-unit limit, so the model draws at 2x to line up with the
// collision mesh, which was baked at true scale.
#define LEVEL_DRAW_SCALE 2.0f

#define NUM_CRATES  4
#define CRATE_HALF  0.4f

// The staircase. Four steps of 0.25, which is well inside the ground spring's
// reach (3 * radius = 0.9) and therefore climbed rather than blocked -- the
// step height a mover can take is set by the probe, not by a constant.
#define NUM_STEPS   4
#define STEP_RISE   0.25f
#define STEP_HALF_X 1.5f
#define STEP_HALF_Z 0.6f
#define STEP_X      5.0f
#define STEP_Z      0.0f

// The wall, to prove the mover stops at something it cannot climb.
#define WALL_X      (-6.0f)
#define WALL_HALF_X 0.3f
#define WALL_HALF_Y 1.5f
#define WALL_HALF_Z 3.0f

// Where the player starts, and where A puts it back.
#define SPAWN_X     0.0f
#define SPAWN_Y     2.0f
#define SPAWN_Z     0.0f

// Turn rate in brad per frame. 32768 is a full circle, so 300 is about
// 3.3 degrees a frame -- a full turn in a bit under two seconds.
#define TURN_RATE   300

typedef struct {
    NEA_Camera *camera;

    NEA_Model *level_model;
    NEA_Phys3DBody *level_body;

    NEA_Model *step_model[NUM_STEPS];
    NEA_Phys3DBody *step_body[NUM_STEPS];

    NEA_Model *wall_model;
    NEA_Phys3DBody *wall_body;

    NEA_Model *crate_model[NUM_CRATES];
    NEA_Phys3DBody *crate[NUM_CRATES];

    NEA_Model *player_model;
    NEA_Phys3DMover mover;
} SceneData;

void Draw3DScene(void *arg)
{
    SceneData *scene = arg;

    NEA_CameraUse(scene->camera);

    NEA_PolyFormat(31, 0, NEA_LIGHT_0 | NEA_LIGHT_1, NEA_CULL_BACK, 0);
    NEA_ModelDraw(scene->level_model);

    for (int i = 0; i < NUM_STEPS; i++)
        NEA_ModelDraw(scene->step_model[i]);

    NEA_ModelDraw(scene->wall_model);

    for (int i = 0; i < NUM_CRATES; i++)
        NEA_ModelDraw(scene->crate_model[i]);

    // The player last, in its own format, so it reads against the level.
    NEA_PolyFormat(31, 0, NEA_LIGHT_0, NEA_CULL_BACK, 0);
    NEA_ModelDraw(scene->player_model);
}

static void CrateStartPosition(int i, float *x, float *y, float *z)
{
    *x = -2.0f + 1.2f * (i % 2);
    *y = 1.0f + 0.9f * i;
    *z = -2.5f + 1.2f * (i / 2);
}

int main(int argc, char *argv[])
{
    (void)argc;
    (void)argv;

    irqEnable(IRQ_HBLANK);
    irqSet(IRQ_VBLANK, NEA_VBLFunc);
    irqSet(IRQ_HBLANK, NEA_HBLFunc);

    NEA_Init3D();
    defaultExceptionHandler();

    static SceneData scene;

    // ---------------------------------------------------------------------
    // The physics world
    // ---------------------------------------------------------------------
    //
    // The mover is not a body and takes no capacity at all -- it is a value in
    // `scene`. What it does need is the world it queries, so everything below
    // sizes the *scene*, not the player.

    NEA_Phys3DWorldDef def = NEA_Phys3DDefaultWorldDef();

    def.box3d.capacity.dynamicBodyCount = NUM_CRATES;
    def.box3d.capacity.dynamicShapeCount = NUM_CRATES;

    // The level mesh, the wall, and one per step.
    def.box3d.capacity.staticBodyCount = 2 + NUM_STEPS;
    def.box3d.capacity.staticShapeCount = 2 + NUM_STEPS;

    def.box3d.capacity.contactCount = (NUM_CRATES + NUM_STEPS) * 8;
    def.box3d.capacity.meshContactCount = NUM_CRATES;

    def.maxBoxHulls = NUM_CRATES + NUM_STEPS + 1;
    def.poolBytes = 192 * 1024;

    if (NEA_Phys3DWorldInit(&def) != 0)
    {
        consoleDemoInit();
        printf("Could not create the physics world.\n");
        while (1)
            swiWaitForVBlank();
    }

    NEA_Phys3DWorldSetGravity(0, -9.8, 0);

    scene.camera = NEA_CameraCreate();

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
    // The staircase
    // ---------------------------------------------------------------------

    for (int i = 0; i < NUM_STEPS; i++)
    {
        float half = STEP_RISE * (i + 1) * 0.5f;
        float y = half;

        scene.step_model[i] = NEA_ModelCreate(NEA_Static);
        NEA_ModelLoadStaticMesh(scene.step_model[i], cube_bin);
        NEA_ModelScaleI(scene.step_model[i], floattof32(STEP_HALF_X),
                        floattof32(half), floattof32(STEP_HALF_Z));

        float z = STEP_Z + STEP_HALF_Z * 2.0f * i;

        scene.step_body[i] = NEA_Phys3DBodyCreate(b3_staticBody, STEP_X, y, z);
        NEA_Phys3DBodyAddBox(scene.step_body[i], STEP_HALF_X, half, STEP_HALF_Z, 1.0);
        NEA_Phys3DBodySetMaterial(scene.step_body[i], 0.6, 0.0);
        NEA_Phys3DBodySetModel(scene.step_body[i], scene.step_model[i]);
    }

    // ---------------------------------------------------------------------
    // The wall
    // ---------------------------------------------------------------------

    scene.wall_model = NEA_ModelCreate(NEA_Static);
    NEA_ModelLoadStaticMesh(scene.wall_model, cube_bin);
    NEA_ModelScaleI(scene.wall_model, floattof32(WALL_HALF_X),
                    floattof32(WALL_HALF_Y), floattof32(WALL_HALF_Z));

    scene.wall_body = NEA_Phys3DBodyCreate(b3_staticBody, WALL_X, WALL_HALF_Y, 0);
    NEA_Phys3DBodyAddBox(scene.wall_body, WALL_HALF_X, WALL_HALF_Y, WALL_HALF_Z, 1.0);
    NEA_Phys3DBodySetMaterial(scene.wall_body, 0.6, 0.0);
    NEA_Phys3DBodySetModel(scene.wall_body, scene.wall_model);

    // ---------------------------------------------------------------------
    // The crates
    // ---------------------------------------------------------------------

    for (int i = 0; i < NUM_CRATES; i++)
    {
        scene.crate_model[i] = NEA_ModelCreate(NEA_Static);
        NEA_ModelLoadStaticMesh(scene.crate_model[i], cube_bin);
        NEA_ModelScaleI(scene.crate_model[i], floattof32(CRATE_HALF),
                        floattof32(CRATE_HALF), floattof32(CRATE_HALF));

        float x, y, z;
        CrateStartPosition(i, &x, &y, &z);

        scene.crate[i] = NEA_Phys3DBodyCreate(b3_dynamicBody, x, y, z);
        NEA_Phys3DBodyAddBox(scene.crate[i], CRATE_HALF, CRATE_HALF, CRATE_HALF, 1.0);
        NEA_Phys3DBodySetMaterial(scene.crate[i], 0.5, 0.05);
        NEA_Phys3DBodySetModel(scene.crate[i], scene.crate_model[i]);
    }

    // ---------------------------------------------------------------------
    // The player
    // ---------------------------------------------------------------------
    //
    // A capsule and a cube to draw for it. No body, no shape, no capacity --
    // NEA_Phys3DMoverInit is the whole of it.

    scene.player_model = NEA_ModelCreate(NEA_Static);
    NEA_ModelLoadStaticMesh(scene.player_model, cube_bin);

    NEA_Phys3DMoverDef moverDef = NEA_Phys3DDefaultMoverDef();
    NEA_Phys3DMoverInit(&scene.mover, &moverDef, SPAWN_X, SPAWN_Y, SPAWN_Z);

    // The cube is drawn at the capsule's proportions, so what is on screen is
    // what is being collided.
    NEA_ModelScaleI(scene.player_model, moverDef.radius,
                    moverDef.halfHeight + moverDef.radius, moverDef.radius);

    // The model's origin is its centre, so the default offset -- which puts a
    // feet-origin model on the ground -- is wrong for a cube. Zero it.
    scene.mover.def.modelOffsetY = 0;
    NEA_Phys3DMoverSetModel(&scene.mover, scene.player_model);

    int32_t buildBytes = NEA_Phys3DWorldGetMemoryUsage();

    consoleDemoInit();
    printf("NEA Box3D Character Mover\n");
    printf("-------------------------\n");
    printf("Left/Right: turn  Up/Down: walk\n");
    printf("B: jump   A: respawn\n\n");

    uint32_t peakTicks = 0;
    uint32_t peakMoverTicks = 0;
    int frames = 0;

    while (1)
    {
        frames++;

        scanKeys();
        uint32_t keys = keysHeld();
        uint32_t down = keysDown();

#ifdef BOX3D_NO_INPUT
        // melonDS injects stray keyboard events into the focused window, which
        // is enough to move a character and ruin a measurement run.
        keys = 0;
        down = 0;
#endif

#ifdef BOX3D_AUTO_WALK
        // A circuit rather than a fixed script: walk forward always, turn
        // steadily, and jump every couple of seconds. Over a few thousand
        // frames that visits the stairs, the wall and the crates without
        // anyone holding the D-pad.
        keys |= KEY_UP | KEY_RIGHT;
        if ((frames % 150) == 0)
            down |= KEY_B;
        if ((frames % 1800) == 0)
            down |= KEY_A;
#endif

        if (keys & KEY_LEFT)
            scene.mover.yaw += TURN_RATE;
        if (keys & KEY_RIGHT)
            scene.mover.yaw -= TURN_RATE;

        int32_t forward = 0;
        if (keys & KEY_UP)
            forward = inttof32(1);
        if (keys & KEY_DOWN)
            forward = -inttof32(1);

        if (down & KEY_B)
            NEA_Phys3DMoverJump(&scene.mover);

        if (down & KEY_A)
        {
            NEA_Phys3DMoverSetPosition(&scene.mover, SPAWN_X, SPAWN_Y, SPAWN_Z);
            NEA_Phys3DMoverSetVelocity(&scene.mover, 0, 0, 0);

            // Teleport the crates back too, which is what makes `port` move --
            // the mover itself is not a body and does not touch that counter.
            for (int i = 0; i < NUM_CRATES; i++)
            {
                float x, y, z;
                CrateStartPosition(i, &x, &y, &z);
                NEA_Phys3DBodySetPosition(scene.crate[i], x, y, z);
            }
        }

        // The world first, then the mover: the mover queries the world, and it
        // wants the world the step just produced rather than the one before.
        cpuStartTiming(0);
        NEA_Phys3DWorldStep();
        uint32_t stepTicks = cpuEndTiming();

        NEA_Phys3DSyncModels();

        cpuStartTiming(0);
        NEA_Phys3DMoverStepYawI(&scene.mover, scene.mover.yaw, forward, 0);
        uint32_t moverTicks = cpuEndTiming();

        if (stepTicks > peakTicks)
            peakTicks = stepTicks;
        if (moverTicks > peakMoverTicks)
            peakMoverTicks = moverTicks;

        // A chase camera, behind and above the player, looking at its head.
        int32_t s = sinLerp(scene.mover.yaw);
        int32_t c = cosLerp(scene.mover.yaw);

        NEA_CameraSetI(scene.camera,
                       scene.mover.x - mulf32(s, inttof32(7)),
                       scene.mover.y + inttof32(4),
                       scene.mover.z - mulf32(c, inttof32(7)),
                       scene.mover.x, scene.mover.y + inttof32(1), scene.mover.z,
                       0, inttof32(1), 0);

        uint32_t stepMicros = (stepTicks * 1000u) / 33514u;
        uint32_t moverMicros = (moverTicks * 1000u) / 33514u;
        uint32_t peakMicros = (peakTicks * 1000u) / 33514u;
        uint32_t peakMoverMicros = (peakMoverTicks * 1000u) / 33514u;

        printf("\x1b[6;0Hground %d   planes %d          ",
               scene.mover.onGround ? 1 : 0, scene.mover.planeCount);
        printf("\x1b[7;0Hsolver %3d  slide %d           ",
               scene.mover.solverIterations, scene.mover.castIterations);
        printf("\x1b[8;0Hpos    %4ld %4ld %4ld          ",
               (long)f32toint(scene.mover.x), (long)f32toint(scene.mover.y),
               (long)f32toint(scene.mover.z));
        printf("\x1b[9;0Hstep   %5lu us  peak %5lu   ",
               (unsigned long)stepMicros, (unsigned long)peakMicros);
        printf("\x1b[10;0Hmover  %5lu us  peak %5lu   ",
               (unsigned long)moverMicros, (unsigned long)peakMoverMicros);
        printf("\x1b[11;0Hdrop   %5d   port %5d      ",
               NEA_Phys3DWorldGetMoverPlaneDropCount(),
               NEA_Phys3DWorldGetTeleportCount());
        printf("\x1b[12;0Hpool   %5ld / %ld bytes      ",
               (long)NEA_Phys3DWorldGetMemoryUsage(),
               (long)NEA_Phys3DWorldGetMemoryCapacity());
        printf("\x1b[13;0Hbuild  %5ld bytes            ", (long)buildBytes);
        printf("\x1b[14;0Hlate   %5d allocs, %ld ovf   ",
               NEA_Phys3DWorldGetLateAllocCount(),
               (long)NEA_Phys3DWorldGetOverflowBytes());
        printf("\x1b[15;0Hcpu %3d %%  fps %2d             ",
               NEA_GetCPUPercent(), NEA_GetFPS());
        printf("\x1b[16;0Hframes %5d                  ", frames);

        NEA_WaitForVBL(NEA_CAN_SKIP_VBL);
        NEA_ProcessArg(Draw3DScene, &scene);
    }

    return 0;
}
