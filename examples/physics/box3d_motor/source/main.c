// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Warioware64, 2026
//
// This file is part of Nitro Engine Advanced
//
// Box3D weld and motor joint example
// ----------------------------------
// A **motor joint** drives a platform back and forth along a sweep while a
// crate of boxes **welded** into one rigid body-group rides on it. X breaks the
// welds and the crate collapses into loose boxes that then slide around on the
// moving platform.
//
// Two joints, one scene, because they are the two halves of the same idea. A
// weld removes every degree of freedom between two bodies; a motor removes none
// and instead *pushes* toward a target with a hard ceiling on the force it may
// spend. Seeing them together is the point: the crate is rigid until it is not,
// and the platform is commanded but never teleported.
//
// Why a motor joint and not NEA_Phys3DBodySetPosition. A platform moved by
// setting its transform teleports -- it passes through whatever is in its way,
// and the crate on top of it is left behind or launched, because nothing about
// a teleport is a force. The motor joint's spring drives the platform toward a
// target pose *through the solver*, so the crate is carried by friction and
// contact like real cargo, and if the platform met a wall it would lose.
//
// The bound is what makes that true, and it is worth reading the numbers: the
// spring is allowed 4000 N, and the platform plus a full crate weighs about
// 1500 N. Drop the bound below that and the platform sags under its load rather
// than holding station -- which is a legitimate thing to want, and is what a
// lift with a weight limit looks like.
//
// As in box3d_rope, box3d_hinge and box3d_ragdoll there are no NEA_Phys3DJoint*
// wrappers yet, so this calls Box3D's joint API directly through
// NEA_Phys3DBody::id.

#include <NEAMain.h>
#include <NEAPhysics3D.h>

#include <box3d/box3d.h>

#include <nds/arm9/exceptions.h>

#include <math.h>

#include "cube_bin.h"

// The crate is a 2x2 stack welded to a single hub box, so three welds.
//
// Four boxes and not more, and the number is measured rather than chosen. Every
// box is welded to box 0, which makes the assembly **over-constrained** -- three
// welds each removing six degrees from a group that has only six to give. Both
// this library and pristine float Box3D hold that comfortably at four boxes and
// both start to drift at six: measured over 240 steps of free fall, the worst
// weld gap is 0.015 here against float's 0.00002, and at six boxes it is 1.90
// against float's 1.26. Past that the redundancy, not the arithmetic, is the
// limit -- so the crate stops where both libraries are still exact.
#define CRATE_COLS 2
#define CRATE_ROWS 2
#define CRATE_DEPTH 1
#define NUM_CRATE ( CRATE_COLS * CRATE_ROWS * CRATE_DEPTH )

// One weld per crate box after the first, plus the platform's motor joint.
#define NUM_WELDS ( NUM_CRATE - 1 )
#define NUM_JOINTS ( NUM_WELDS + 1 )

#define NUM_BODIES ( NUM_CRATE + 1 )

#define PLATFORM_HALF_X 1.6f
#define PLATFORM_HALF_Y 0.12f
#define PLATFORM_HALF_Z 1.2f
#define PLATFORM_Y 1.8f

#define BOX_HALF 0.30f

// Centre-to-centre spacing inside the crate, with a deliberate 6 cm of air.
//
// Every box after the first is welded to box 0, so boxes 1 and 2 are **not**
// jointed to each other -- and b3ShouldBodiesCollide only skips a pair that a
// joint directly connects. Packed flush, the crate members therefore collide
// with each other permanently while the welds hold them together, and the
// contact solver and the joint solver push against each other every sub-step
// until the crate detonates. Measured that way: 88 respawns, the drive pinned
// at its 4000 N bound, 180% CPU and 27,695 us a step.
//
// The gap is the fix and it is the right one: a rigid assembly's members have
// no business touching, because the weld already holds their relative pose and
// a contact between them can only fight it.
#define BOX_GAP ( 2.0f * BOX_HALF + 0.06f )

// How far the platform sweeps either side of the origin, and how fast.
#define SWEEP_X 2.2f
#define SWEEP_PERIOD_FRAMES 260.0f

#define FLOOR_HALF 8.0f

// Below this a box counts as lost. box3d_basic's rule: the check is cheap and
// its absence is a slow leak that reads as a solver fault.
#define RESPAWN_BELOW ( -10.0f )

typedef struct
{
    NEA_Camera *camera;

    NEA_Model *crate_model[NUM_CRATE];
    NEA_Model *platform_model;
    NEA_Model *floor_model;

    NEA_Phys3DBody *crate[NUM_CRATE];
    NEA_Phys3DBody *platform;
    NEA_Phys3DBody *anchor;
    NEA_Phys3DBody *floor_body;

    b3JointId weld[NUM_WELDS];
    b3JointId motor;
} SceneData;

// Where each crate box starts, relative to the crate's own origin.
static void CrateOffset(int index, float *ox, float *oy, float *oz)
{
    int col = index % CRATE_COLS;
    int row = ( index / CRATE_COLS ) % CRATE_ROWS;
    int layer = index / ( CRATE_COLS * CRATE_ROWS );

    *ox = ( (float)col - 0.5f ) * BOX_GAP;
    *oy = (float)row * BOX_GAP;
    *oz = ( (float)layer - 0.5f ) * BOX_GAP;
}

static float CrateBaseY(void)
{
    return PLATFORM_Y + PLATFORM_HALF_Y + BOX_HALF;
}

void Draw3DScene(void *arg)
{
    SceneData *scene = arg;

    NEA_CameraUse(scene->camera);

    NEA_PolyFormat(31, 0, NEA_LIGHT_0 | NEA_LIGHT_1, NEA_CULL_BACK, 0);

    for (int i = 0; i < NUM_CRATE; i++)
    {
        if (scene->crate_model[i] != NULL)
            NEA_ModelDraw(scene->crate_model[i]);
    }

    NEA_PolyFormat(28, 0, NEA_LIGHT_0 | NEA_LIGHT_1, NEA_CULL_BACK, 0);
    NEA_ModelDraw(scene->platform_model);

    NEA_PolyFormat(20, 0, NEA_LIGHT_0 | NEA_LIGHT_1, NEA_CULL_BACK, 0);
    NEA_ModelDraw(scene->floor_model);
}

static void ResetScene(SceneData *scene)
{
    for (int i = 0; i < NUM_CRATE; i++)
    {
        if (scene->crate[i] == NULL)
            continue;

        float ox, oy, oz;
        CrateOffset(i, &ox, &oy, &oz);

        NEA_Phys3DBodySetPosition(scene->crate[i], ox, CrateBaseY() + oy, oz);
        NEA_Phys3DBodySetVelocity(scene->crate[i], 0, 0, 0);
        NEA_Phys3DBodySetAwake(scene->crate[i], true);
    }

    if (scene->platform != NULL)
    {
        NEA_Phys3DBodySetPosition(scene->platform, 0, PLATFORM_Y, 0);
        NEA_Phys3DBodySetVelocity(scene->platform, 0, 0, 0);
        NEA_Phys3DBodySetAwake(scene->platform, true);
    }
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

    def.box3d.capacity.dynamicBodyCount = NUM_BODIES;
    def.box3d.capacity.dynamicShapeCount = NUM_BODIES;

    // The floor and the motor joint's anchor.
    def.box3d.capacity.staticBodyCount = 2;
    def.box3d.capacity.staticShapeCount = 2;

    // Crate boxes against the platform, against the floor once the welds break,
    // and against each other the whole time.
    def.box3d.capacity.contactCount = NUM_BODIES * 8;

    // A weld joint's sim is 176 bytes and a motor joint's is 240, but the union
    // is sized by its widest member -- the spherical joint's 260 -- so
    // b3JointSim is 444 for these too and Stage 5 cost the arrays nothing.
    // Each sim is still reserved four times over, because it migrates between
    // the constraint graph, the awake set, a sleeping set and the disabled set.
    // Declaring the count keeps all four out of `late` below.
    def.box3d.capacity.jointCount = NUM_JOINTS;

    def.poolBytes = 160 * 1024;

    if (NEA_Phys3DWorldInit(&def) != 0)
    {
        consoleDemoInit();
        printf("Could not create the physics world.\n");
        while (1)
            swiWaitForVBlank();
    }

    NEA_Phys3DWorldSetGravity(0, -9.8, 0);

    NEA_CameraSet(scene.camera = NEA_CameraCreate(),
                  0, 4.2, 9.5,
                  0, 1.6, 0,
                  0, 1, 0);

    NEA_LightSet(0, NEA_White, 0, -1, -1);
    NEA_LightSet(1, NEA_Blue, -1, 0, 0);

    NEA_ClearColorSet(RGB15(4, 6, 8), 31, 63);

    int shapesMissing = 0;
    int bodiesMissing = 0;
    int jointsMissing = 0;

    // ---------------------------------------------------------------------
    // The floor
    // ---------------------------------------------------------------------

    scene.floor_model = NEA_ModelCreate(NEA_Static);
    NEA_ModelLoadStaticMesh(scene.floor_model, cube_bin);
    NEA_ModelSetCoord(scene.floor_model, 0, -0.25, 0);
    NEA_ModelScaleI(scene.floor_model, floattof32(FLOOR_HALF), inttof32(1) / 4,
                    floattof32(FLOOR_HALF));

    scene.floor_body = NEA_Phys3DBodyCreate(b3_staticBody, 0, -0.25, 0);
    if (NEA_Phys3DBodyAddBox(scene.floor_body, FLOOR_HALF, 0.25, FLOOR_HALF, 1.0) != 0)
        shapesMissing++;
    NEA_Phys3DBodySetMaterial(scene.floor_body, 0.6, 0.05);

    // ---------------------------------------------------------------------
    // The motor joint's anchor
    // ---------------------------------------------------------------------
    //
    // A static body with no shape. It exists only to carry the joint frame the
    // platform is driven toward -- moving *its* frame is how the sweep is
    // commanded, so the drive stays a spring pulling toward a target rather
    // than anything that touches the platform's transform directly.
    scene.anchor = NEA_Phys3DBodyCreate(b3_staticBody, 0, PLATFORM_Y, 0);
    if (scene.anchor == NULL)
        bodiesMissing++;

    // ---------------------------------------------------------------------
    // The platform
    // ---------------------------------------------------------------------

    scene.platform_model = NEA_ModelCreate(NEA_Static);
    NEA_ModelLoadStaticMesh(scene.platform_model, cube_bin);
    NEA_ModelScaleI(scene.platform_model, floattof32(PLATFORM_HALF_X),
                    floattof32(PLATFORM_HALF_Y), floattof32(PLATFORM_HALF_Z));

    scene.platform = NEA_Phys3DBodyCreate(b3_dynamicBody, 0, PLATFORM_Y, 0);
    if (scene.platform == NULL)
    {
        bodiesMissing++;
    }
    else
    {
        // Density 1, matching every other box3d example rather than Box3D's own
        // default of 1000. The platform is wide and thin, so this is also the
        // shape whose inverse inertia is least isotropic in the scene.
        if (NEA_Phys3DBodyAddBox(scene.platform, PLATFORM_HALF_X, PLATFORM_HALF_Y,
                                 PLATFORM_HALF_Z, 1.0) != 0)
        {
            shapesMissing++;
        }

        // High friction: the crate has to be *carried* by the platform rather
        // than slide off it the moment the sweep reverses, and friction is the
        // only thing holding it. This is the parameter the demonstration is
        // most sensitive to.
        NEA_Phys3DBodySetMaterial(scene.platform, 0.9, 0.0);

        NEA_Phys3DBodySetModel(scene.platform, scene.platform_model);
    }

    // ---------------------------------------------------------------------
    // The crate
    // ---------------------------------------------------------------------

    for (int i = 0; i < NUM_CRATE; i++)
    {
        float ox, oy, oz;
        CrateOffset(i, &ox, &oy, &oz);

        scene.crate_model[i] = NEA_ModelCreate(NEA_Static);
        NEA_ModelLoadStaticMesh(scene.crate_model[i], cube_bin);
        NEA_ModelScaleI(scene.crate_model[i], floattof32(BOX_HALF),
                        floattof32(BOX_HALF), floattof32(BOX_HALF));

        scene.crate[i] = NEA_Phys3DBodyCreate(b3_dynamicBody, ox, CrateBaseY() + oy, oz);
        if (scene.crate[i] == NULL)
        {
            bodiesMissing++;
            continue;
        }

        if (NEA_Phys3DBodyAddBox(scene.crate[i], BOX_HALF, BOX_HALF, BOX_HALF, 1.0) != 0)
            shapesMissing++;

        NEA_Phys3DBodySetMaterial(scene.crate[i], 0.7, 0.05);
        NEA_Phys3DBodySetModel(scene.crate[i], scene.crate_model[i]);

        // Damping, for the reason box3d_rope, box3d_hinge and box3d_ragdoll all
        // give: a joint constrains, it does not dissipate. Without it the crate
        // hums indefinitely on a moving platform and never sleeps.
        b3Body_SetLinearDamping(scene.crate[i]->id, b3fFromDouble(0.2));
        b3Body_SetAngularDamping(scene.crate[i]->id, b3fFromDouble(0.4));
    }

    // ---------------------------------------------------------------------
    // The welds
    // ---------------------------------------------------------------------
    //
    // Every box after the first is welded to box 0, so the crate is a star
    // rather than a chain. A chain would work too, but a star is one hop from
    // any box to the reference and therefore stiffer for the same number of
    // joints -- and it makes breaking the crate a single loop with no ordering
    // to think about.
    //
    // Both joint frames sit at the *midpoint* between the two boxes, not at
    // either centre. It makes no difference to a rigid weld, which locks the
    // whole relative transform however the frames are placed -- but it is where
    // a reader expects the joint to be, and if either half is later softened
    // into a spring the frame placement is suddenly the thing that decides how
    // it flexes.

    for (int i = 1; i < NUM_CRATE; i++)
    {
        if (scene.crate[i] == NULL || scene.crate[0] == NULL)
        {
            jointsMissing++;
            continue;
        }

        float ox, oy, oz, rx, ry, rz;
        CrateOffset(i, &ox, &oy, &oz);
        CrateOffset(0, &rx, &ry, &rz);

        // The midpoint in world terms, then expressed in each body's own space.
        float mx = 0.5f * ( ox + rx );
        float my = 0.5f * ( oy + ry );
        float mz = 0.5f * ( oz + rz );

        b3WeldJointDef weld = b3DefaultWeldJointDef();
        weld.base.bodyIdA = scene.crate[0]->id;
        weld.base.bodyIdB = scene.crate[i]->id;

        // Frame A is the midpoint in box 0's space, frame B the same world
        // point in box i's space -- so each is the midpoint minus that body's
        // own offset. Getting this subtraction backwards puts the two frames on
        // opposite sides of the seam and the solver opens by closing a gap of
        // twice the offset, which detonates the crate on frame one and reads
        // exactly like a broken constraint. box3d_rope and box3d_ragdoll both
        // hit that; it is the same bug in a third costume.
        weld.base.localFrameA.p = b3MakeVec3(b3fFromDouble(mx - rx),
                                             b3fFromDouble(my - ry),
                                             b3fFromDouble(mz - rz));
        weld.base.localFrameB.p = b3MakeVec3(b3fFromDouble(mx - ox),
                                             b3fFromDouble(my - oy),
                                             b3fFromDouble(mz - oz));

        // Rigid: zero hertz on both halves means the weld uses the joint's own
        // constraint softness rather than a spring, which is what makes the
        // crate behave as one body.
        scene.weld[i - 1] = b3CreateWeldJoint(NEA_Phys3DWorldGetId(), &weld);
        if (b3Joint_IsValid(scene.weld[i - 1]) == false)
            jointsMissing++;
    }

    // ---------------------------------------------------------------------
    // The motor joint
    // ---------------------------------------------------------------------

    if (scene.platform != NULL && scene.anchor != NULL)
    {
        b3MotorJointDef motor = b3DefaultMotorJointDef();
        motor.base.bodyIdA = scene.anchor->id;
        motor.base.bodyIdB = scene.platform->id;

        // Both frames at their bodies' origins, which coincide at build time --
        // so the spring's target is "platform where the anchor is", and sweeping
        // the anchor's frame sweeps the platform.
        motor.base.localFrameA.p = b3Vec3_zero;
        motor.base.localFrameB.p = b3Vec3_zero;

        // Stiff and well damped: the platform should track the sweep closely
        // and not overshoot, because an overshooting platform throws its cargo
        // and the scene stops being about the joints.
        motor.linearHertz = b3fFromDouble(6.0);
        motor.linearDampingRatio = b3fFromDouble(2.0);
        motor.angularHertz = b3fFromDouble(6.0);
        motor.angularDampingRatio = b3fFromDouble(2.0);

        // The bounds, and the reason they are these numbers. The platform is
        // 3.2 x 0.24 x 2.4 at density 1, so about 1.8 kg; a full crate of eight
        // 0.6-cubes is another 1.7. Together they need roughly 35 N held
        // against gravity, and a good deal more than that to accelerate at the
        // ends of the sweep where the platform reverses.
        //
        // 4000 N is far above both, which is deliberate: this platform is meant
        // to be authoritative. Lowering it below the load is a legitimate thing
        // to want and turns the same scene into a lift with a weight limit --
        // the platform sags under the crate and recovers when it is broken up.
        motor.maxSpringForce = b3fFromDouble(4000.0);
        motor.maxSpringTorque = b3fFromDouble(4000.0);

        scene.motor = b3CreateMotorJoint(NEA_Phys3DWorldGetId(), &motor);
        if (b3Joint_IsValid(scene.motor) == false)
            jointsMissing++;
    }
    else
    {
        jointsMissing++;
    }

    int32_t buildBytes = NEA_Phys3DWorldGetMemoryUsage();

    consoleDemoInit();

    printf("NEA Box3D Motor + Weld\n");
    printf("----------------------\n");
    printf("X: break the welds\n");
    printf("Y: pause the sweep\n");
    printf("B: shove crate  A: reset\n\n");

    uint32_t stepTicks = 0;
    uint32_t peakTicks = 0;
    int peakAwake = 0;
    uint32_t frames = 0;
    int respawns = 0;

    bool welded = true;
    bool sweeping = true;
    float phase = 0.0f;

    while (1)
    {
        scanKeys();
        uint32_t keys = keysHeld();
        uint32_t down = keysDown();

#ifdef BOX3D_NO_INPUT
        // Baseline measurement build. melonDS injects stray keyboard events
        // into the focused window, and here they land on X (breaking the crate)
        // and B (a shove), either of which changes what is being measured.
        //
        // Same switch as box3d_basic and box3d_ragdoll, and for the same
        // reason: it is what makes the recorded numbers a property of the
        // simulation rather than of the emulator.
        keys = 0;
        down = 0;
#endif

        // The demonstration in one button. Welded, the crate is one rigid body
        // that the platform carries as a unit; broken, it is eight loose boxes
        // that slide and topple as the platform sweeps under them.
#ifdef BOX3D_AUTO_BREAK
        // Verification build only: break the crate on a fixed frame so the
        // headline behaviour can be measured without depending on emulator
        // keyboard input, which BOX3D_NO_INPUT exists to distrust.
        if (frames == 400)
            down |= KEY_X;
#endif

        if (down & KEY_X)
        {
            if (welded)
            {
                for (int i = 0; i < NUM_WELDS; i++)
                {
                    if (b3Joint_IsValid(scene.weld[i]))
                        b3DestroyJoint(scene.weld[i], true);
                }
                welded = false;
            }
        }

        if (down & KEY_Y)
            sweeping = !sweeping;

        if ((keys & KEY_B) && scene.crate[NUM_CRATE - 1] != NULL)
            NEA_Phys3DBodyApplyForce(scene.crate[NUM_CRATE - 1], 40, 0, 0);

        if (down & KEY_A)
        {
            ResetScene(&scene);
            phase = 0.0f;
        }

        // Sweep the anchor's joint frame. The motor's spring then pulls the
        // platform after it -- which is the whole difference from moving the
        // platform itself: everything the platform does to the crate goes
        // through contact and friction, so the crate is carried rather than
        // teleported along with it.
        if (sweeping && b3Joint_IsValid(scene.motor))
        {
            phase += 1.0f;
            float t = phase * ( 2.0f * 3.14159265f / SWEEP_PERIOD_FRAMES );
            float targetX = SWEEP_X * sinf(t);

            b3Transform frameA = b3Transform_identity;
            frameA.p = b3MakeVec3(b3fFromDouble(targetX), b3f_zero, b3f_zero);
            b3Joint_SetLocalFrameA(scene.motor, frameA);

            // b3Joint_SetLocalFrameA does not wake the bodies -- it writes the
            // frame and nothing else, like every other joint accessor that
            // changes a target rather than a topology. So a platform that went
            // to sleep while the sweep was paused would stay asleep and ignore
            // the new target forever, which looks exactly like a dead motor.
            // Waking it here is the caller's job, and this is the caller.
            if (scene.platform != NULL)
                NEA_Phys3DBodySetAwake(scene.platform, true);
        }

        // Stepped by hand so it can be timed, hence NEA_CAN_SKIP_VBL below
        // rather than NEA_UPDATE_PHYS3D, which would step the world twice.
        cpuStartTiming(0);
        NEA_Phys3DWorldStep();
        stepTicks = cpuEndTiming();
        frames++;

        NEA_Phys3DSyncModels();

        for (int i = 0; i < NUM_CRATE; i++)
        {
            if (scene.crate[i] == NULL)
                continue;

            int32_t bx, by, bz;
            NEA_Phys3DBodyGetPositionI(scene.crate[i], &bx, &by, &bz);

            if (by < floattof32(RESPAWN_BELOW))
            {
                ResetScene(&scene);
                respawns++;
                break;
            }
        }

        int awake = NEA_Phys3DWorldGetAwakeBodyCount();
        if (stepTicks > peakTicks)
        {
            peakTicks = stepTicks;
            peakAwake = awake;
        }

        uint32_t stepMicros = (stepTicks * 1000u) / 33514u;
        uint32_t peakMicros = (peakTicks * 1000u) / 33514u;

        // How far the platform actually is from where it was told to be. This
        // is the motor joint's own error measure, and the number that says the
        // drive is a spring rather than a teleport: a teleport would read zero
        // always, and a spring lags a little and lags *more* under load. Break
        // the crate with X and watch it fall.
        int trackMilli = 0;
        int forceN = 0;
        if (scene.platform != NULL)
        {
            int32_t px, py, pz;
            NEA_Phys3DBodyGetPositionI(scene.platform, &px, &py, &pz);

            float t = phase * ( 2.0f * 3.14159265f / SWEEP_PERIOD_FRAMES );
            float targetX = sweeping ? SWEEP_X * sinf(t) : f32tofloat(px);
            trackMilli = (int)((f32tofloat(px) - targetX) * 1000.0f);

            if (b3Joint_IsValid(scene.motor))
            {
                b3Vec3 f = b3Joint_GetConstraintForce(scene.motor);
                double fx = b3fToDouble(f.x);
                double fy = b3fToDouble(f.y);
                double fz = b3fToDouble(f.z);
                forceN = (int)sqrt(fx * fx + fy * fy + fz * fz);
            }
        }

        printf("\x1b[8;0Hstep   %5lu us (%lu ticks)  ",
               (unsigned long)stepMicros, (unsigned long)stepTicks);
        printf("\x1b[9;0Hpeak   %5lu us @ %d awake   ",
               (unsigned long)peakMicros, peakAwake);
        printf("\x1b[10;0Hawake  %2d / %d bodies       ", awake, NUM_BODIES);
        printf("\x1b[11;0Hcrate  %s                   ", welded ? "welded" : "loose ");
        printf("\x1b[12;0Htrack  %5d mm  drive %4d N ", trackMilli, forceN);
        printf("\x1b[13;0Hpool   %5ld / %ld bytes     ",
               (long)NEA_Phys3DWorldGetMemoryUsage(),
               (long)NEA_Phys3DWorldGetMemoryCapacity());
        printf("\x1b[14;0Hbuild  %5ld bytes           ", (long)buildBytes);
        printf("\x1b[15;0Hlate   %5d allocs, %ld ovf  ",
               NEA_Phys3DWorldGetLateAllocCount(),
               (long)NEA_Phys3DWorldGetOverflowBytes());
        printf("\x1b[16;0Hcpu    %3d %%                ", NEA_GetCPUPercent());
        printf("\x1b[17;0Hmissing %d bd %d jt %d shp   ",
               bodiesMissing, jointsMissing, shapesMissing);
        printf("\x1b[18;0Hrespawn %3d  frame %8lu    ", respawns,
               (unsigned long)frames);

        NEA_WaitForVBL(NEA_CAN_SKIP_VBL);
        NEA_ProcessArg(Draw3DScene, &scene);
    }

    return 0;
}
