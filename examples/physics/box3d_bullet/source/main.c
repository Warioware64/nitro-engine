// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Warioware64, 2026
//
// This file is part of Nitro Engine Advanced
//
// Box3D continuous collision: firing a small sphere at a triangle level
// ---------------------------------------------------------------------
// A projectile fired straight down at the baked level from nine units up, at a
// speed you set, with continuous collision on a toggle. The same shot either
// lands on the floor or passes straight through it, and which one happens is
// the whole of what this example shows.
//
// Why a triangle mesh is the hard case: a mesh triangle has no thickness at
// all. There is no speed at which a discrete solver "mostly" catches it -- the
// narrow phase runs once per step and asks whether the shapes overlap *now*, so
// a sphere that was above the floor at the start of the step and below it at
// the end produces no contact, no impulse, and no evidence that anything went
// wrong. It simply keeps falling until the world-extent guard parks it.
//
// What the readout is for:
//
//   travel  How far the projectile moves in one step, in millimetres, next to
//           its own radius. This is the number that predicts tunnelling: once
//           travel exceeds roughly the radius, the discrete path has nothing
//           left to catch.
//
//   state   FLYING, LANDED, or THROUGH. THROUGH is not a failure message --
//           with continuous collision off it is the correct behaviour, and
//           seeing it is the point.
//
//   ccd     Off, On, or Bullet. Bullet additionally sweeps against kinematic
//           and dynamic bodies rather than static geometry alone; here the
//           level is static, so it changes which pass runs and not the answer,
//           and the two are printed side by side to say so.
//
//   late    Allocations made after the world was sealed. The sweep runs inside
//           the step with its context on the C stack and allocates nothing, so
//           the number that matters is not that this reads zero -- it reads a
//           small constant from building the scene, like every example here --
//           but that it does not move while a shot is being swept every frame.
//
// The bottom three lines are the comparison itself: the outcome of the last
// shot fired in each mode, kept so that one screen shows the same shot landing
// and not landing. Build with -DBOX3D_AUTO_FIRE to have all three fired on
// fixed frames without touching a button.

#include <NEAMain.h>
#include <NEAPhysics3D.h>

#include <nds/arm9/exceptions.h>

#include "cube_bin.h"
#include "level_bin.h"

#include "level_b3mesh.h"

// Matches box3d_level: the display list was authored at half size to fit v16's
// 8-unit limit, so the model draws at 2x to line up with the collision mesh.
#define LEVEL_DRAW_SCALE 2.0f

// Small enough that it tunnels at a speed the level can still be seen at. A
// projectile the size of the boxes in the other examples would need to be
// moving too fast to watch.
#define SHOT_RADIUS 0.15f

#define SHOT_X 0.0f
#define SHOT_Y 9.0f
#define SHOT_Z 0.0f

// Anything below this has left the level through the floor. The floor is at
// y = 0 and the level's walls do not reach down here.
#define THROUGH_Y (-3.0f)

typedef enum {
    CCD_OFF = 0,
    CCD_ON,
    CCD_BULLET,
    CCD_MODE_COUNT
} CcdMode;

typedef enum {
    SHOT_IDLE = 0,
    SHOT_FLYING,
    SHOT_LANDED,
    SHOT_THROUGH
} ShotState;

typedef struct {
    NEA_Camera *camera;
    NEA_Model *level_model;
    NEA_Model *shot_model;
    NEA_Phys3DBody *shot;
    NEA_Phys3DBody *level_body;
} SceneData;

void Draw3DScene(void *arg)
{
    SceneData *scene = arg;

    NEA_CameraUse(scene->camera);

    NEA_PolyFormat(31, 0, NEA_LIGHT_0 | NEA_LIGHT_1, NEA_CULL_BACK, 0);
    NEA_ModelDraw(scene->shot_model);

    NEA_PolyFormat(20, 0, NEA_LIGHT_0 | NEA_LIGHT_1, NEA_CULL_BACK, 0);
    NEA_ModelDraw(scene->level_model);
}

static void ApplyCcdMode(SceneData *scene, CcdMode mode)
{
    // Both calls are Box3D's rather than NEA's. NEA_Phys3DWorldGetId() and the
    // body's `id` field exist for exactly this -- the same door the joint
    // examples go through for b3CreateWheelJoint.
    b3World_EnableContinuous(NEA_Phys3DWorldGetId(), mode != CCD_OFF);
    b3Body_SetBullet(scene->shot->id, mode == CCD_BULLET);
}

static void Fire(SceneData *scene, float speed)
{
    NEA_Phys3DBodySetPosition(scene->shot, SHOT_X, SHOT_Y, SHOT_Z);
    NEA_Phys3DBodySetVelocity(scene->shot, 0, -speed, 0);
    NEA_Phys3DBodySetAwake(scene->shot, true);
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

    def.box3d.capacity.dynamicBodyCount = 1;
    def.box3d.capacity.dynamicShapeCount = 1;
    def.box3d.capacity.staticBodyCount = 1;
    def.box3d.capacity.staticShapeCount = 1;
    def.box3d.capacity.contactCount = 8;
    def.box3d.capacity.meshContactCount = 2;

    // One body and one mesh, so this is the smallest pool any of the mesh
    // examples run on. The continuous pass does not change it: b3SolveContinuous
    // keeps its query context on the C stack and reuses the broad-phase trees
    // and the mesh blob that the scene already carries, so a swept step
    // allocates exactly what an ordinary one does.
    def.poolBytes = 96 * 1024;

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
                  -13, 8, 15,
                    0, 1, -2,
                    0, 1,  0);

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
    // The projectile
    // ---------------------------------------------------------------------

    scene.shot_model = NEA_ModelCreate(NEA_Static);
    NEA_ModelLoadStaticMesh(scene.shot_model, cube_bin);
    NEA_ModelScaleI(scene.shot_model, floattof32(SHOT_RADIUS),
                    floattof32(SHOT_RADIUS), floattof32(SHOT_RADIUS));

    scene.shot = NEA_Phys3DBodyCreate(b3_dynamicBody, SHOT_X, SHOT_Y, SHOT_Z);
    NEA_Phys3DBodyAddSphere(scene.shot, SHOT_RADIUS, 1.0);
    NEA_Phys3DBodySetMaterial(scene.shot, 0.4, 0.1);
    NEA_Phys3DBodySetModel(scene.shot, scene.shot_model);

    // Sleeping would end a shot the moment it settled, and the readout wants to
    // keep showing where it stopped. Box3D's call again, through the body's own
    // b3BodyId, as with the two in ApplyCcdMode above.
    b3Body_EnableSleep(scene.shot->id, false);

    int32_t buildBytes = NEA_Phys3DWorldGetMemoryUsage();

    consoleDemoInit();
    printf("NEA Box3D Continuous\n");
    printf("--------------------\n");
    printf("A: fire   B: ccd mode\n");
    printf("Up/Down: speed\n\n");

    // Metres per second. The default is chosen to be past the threshold but
    // not absurdly: at 60 Hz it is 1.25 units of travel per step against a
    // 0.15 radius, so the discrete path misses the floor by a wide margin and
    // the swept path catches it comfortably.
    int speed = 75;

    CcdMode mode = CCD_ON;
    ApplyCcdMode(&scene, mode);

    ShotState state = SHOT_IDLE;

    // The lowest the projectile reached on the last shot, which is what says
    // how far past the floor a tunnelling shot actually got.
    float lowest = SHOT_Y;

    // What the last shot in each mode did. Kept per mode so that one screen
    // carries the whole comparison rather than asking the reader to remember
    // what the previous shot did.
    ShotState result[CCD_MODE_COUNT] = { SHOT_IDLE, SHOT_IDLE, SHOT_IDLE };
    int resultLowest[CCD_MODE_COUNT] = { 0, 0, 0 };

    uint32_t peakTicks = 0;
    int frames = 0;

    while (1)
    {
        frames++;

        scanKeys();
        uint32_t keys = keysHeld();
        uint32_t down = keysDown();

#ifdef BOX3D_NO_INPUT
        // Baseline measurement build. melonDS injects stray keyboard events
        // into the focused window, and here they land on A (fire) and B (a
        // mode change), either of which changes what is being measured. Same
        // switch as box3d_basic, box3d_motor and box3d_ragdoll.
        keys = 0;
        down = 0;
#endif

#ifdef BOX3D_AUTO_FIRE
        // Verification build only: fire the same shot in each mode on fixed
        // frames, so the three-way comparison at the bottom of the readout
        // fills itself in without depending on emulator keyboard input.
        //
        // 240 frames is four seconds, which is long enough for a tunnelling
        // shot to fall past THROUGH_Y and for a landing one to settle.
        if (frames == 60 || frames == 300 || frames == 540)
        {
            mode = (frames == 60) ? CCD_OFF : (frames == 300) ? CCD_ON : CCD_BULLET;
            ApplyCcdMode(&scene, mode);
            down |= KEY_A;
        }
#endif

        if (keys & KEY_UP)   speed += 1;
        if (keys & KEY_DOWN) speed -= 1;

        if (speed < 2)   speed = 2;
        if (speed > 300) speed = 300;

        if (down & KEY_B)
        {
            mode = (CcdMode)((mode + 1) % CCD_MODE_COUNT);
            ApplyCcdMode(&scene, mode);
        }

        if (down & KEY_A)
        {
            Fire(&scene, (float)speed);
            state = SHOT_FLYING;
            lowest = SHOT_Y;
            peakTicks = 0;
        }

        cpuStartTiming(0);
        NEA_Phys3DWorldStep();
        uint32_t stepTicks = cpuEndTiming();

        NEA_Phys3DSyncModels();

        if (stepTicks > peakTicks)
            peakTicks = stepTicks;

        int32_t px, py, pz;
        NEA_Phys3DBodyGetPositionI(scene.shot, &px, &py, &pz);

        float y = f32tofloat(py);
        if (y < lowest)
            lowest = y;

        if (state == SHOT_FLYING)
        {
            int32_t vx, vy, vz;
            NEA_Phys3DBodyGetVelocityI(scene.shot, &vx, &vy, &vz);

            if (y < THROUGH_Y)
            {
                state = SHOT_THROUGH;
            }
            else if (vy > floattof32(-0.5f) && vy < floattof32(0.5f)
                     && y > 0.0f)
            {
                state = SHOT_LANDED;
            }

            if (state != SHOT_FLYING)
            {
                result[mode] = state;
                resultLowest[mode] = (int)(lowest * 1000.0f);
            }
        }

        uint32_t stepMicros = (stepTicks * 1000u) / 33514u;
        uint32_t peakMicros = (peakTicks * 1000u) / 33514u;

        // Travel per step against the radius, both in millimetres, because
        // that comparison is the whole prediction and a ratio hides which of
        // the two moved.
        int travelMm = (speed * 1000) / 60;
        int radiusMm = (int)(SHOT_RADIUS * 1000.0f);

        static const char *modeNames[CCD_MODE_COUNT] = { "off   ", "on    ", "bullet" };
        static const char *stateNames[4] = { "idle   ", "FLYING ", "LANDED ", "THROUGH" };

        printf("\x1b[6;0Hspeed  %3d m/s                ", speed);
        printf("\x1b[7;0Htravel %4d mm / step, r %d mm ", travelMm, radiusMm);
        printf("\x1b[8;0Hccd    %s                  ", modeNames[mode]);
        printf("\x1b[9;0Hstate  %s                  ", stateNames[state]);
        printf("\x1b[10;0Hy      %6d mm             ",
               (int)(((int64_t)py * 1000) >> 12));
        printf("\x1b[11;0Hlowest %6d mm             ",
               (int)(lowest * 1000.0f));
        printf("\x1b[12;0Hstep   %5lu us              ",
               (unsigned long)stepMicros);
        printf("\x1b[13;0Hpeak   %5lu us              ",
               (unsigned long)peakMicros);
        printf("\x1b[14;0Hpool   %5ld / %ld bytes      ",
               (long)NEA_Phys3DWorldGetMemoryUsage(),
               (long)NEA_Phys3DWorldGetMemoryCapacity());
        printf("\x1b[15;0Hbuild  %5ld bytes            ", (long)buildBytes);
        printf("\x1b[16;0Hlate   %5d allocs, %ld ovf   ",
               NEA_Phys3DWorldGetLateAllocCount(),
               (long)NEA_Phys3DWorldGetOverflowBytes());
        printf("\x1b[17;0Hcpu %3d %%  fps %2d             ",
               NEA_GetCPUPercent(), NEA_GetFPS());

        // The comparison, in three lines: the same shot, the same speed, the
        // same floor, and one of them goes through.
        for (int m = 0; m < CCD_MODE_COUNT; m++)
        {
            printf("\x1b[%d;0H%s %s  low %5d mm    ", 19 + m, modeNames[m],
                   stateNames[result[m]], resultLowest[m]);
        }

        NEA_WaitForVBL(NEA_CAN_SKIP_VBL);
        NEA_ProcessArg(Draw3DScene, &scene);
    }

    return 0;
}
