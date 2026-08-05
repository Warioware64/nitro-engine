// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Warioware64, 2026
//
// This file is part of Nitro Engine Advanced
//
// Box3D prismatic joint example
// -----------------------------
// A **prismatic joint** is a slider: one translational degree of freedom and
// everything else locked. Two of them here, doing the two different jobs a
// slider does.
//
// The **lift** is a platform on a vertical rail, driven by the joint's motor
// and stopped at each end by the joint's limits, carrying four loose boxes. The
// **suspension leg** beside it is a pad on a sprung rail that the cargo can be
// dropped onto, and Y switches its spring between stiff and soft.
//
// Three things are on screen that no earlier joint example could show:
//
//   - **The limit is a constraint, not a clamp.** X turns it off while the
//     motor is still driving, and the platform runs off the end of the rail
//     rather than stopping at a number. A clamp would pin the value; a limit is
//     an impulse that something can be stronger than.
//   - **The orientation lock.** The cargo is stacked deliberately off-centre,
//     so the platform is under a constant tipping torque -- and it stays level.
//     That is the prismatic's 3-vector angular constraint doing visible work,
//     and a hinge example structurally cannot show it because a hinge is *for*
//     turning.
//   - **The drive can lose.** The motor's force budget is a hard ceiling, so an
//     overloaded lift sags instead of winning. See MAX_MOTOR_FORCE below.
//
// Why the rail points where it does. The slide axis is **frame A's x**, not the
// revolute's z -- upstream's choice and the port's. So a vertical lift needs
// both joint frames rotated to take x onto +y, which is what VerticalFrame()
// does. With the anchor at the origin and both frame positions at their bodies'
// origins, b3PrismaticJoint_GetTranslation then reads directly as the
// platform's height above the floor, which is what makes the readout legible.
//
// As in box3d_rope, box3d_hinge, box3d_ragdoll and box3d_motor there are no
// NEA_Phys3DJoint* wrappers yet, so this calls Box3D's joint API directly
// through NEA_Phys3DBody::id.

#include <NEAMain.h>
#include <NEAPhysics3D.h>

#include <box3d/box3d.h>

#include <nds/arm9/exceptions.h>

#include <math.h>

#include "cube_bin.h"

#define NUM_CARGO 4

// The lift platform, the suspension pad, and the cargo.
#define NUM_BODIES ( NUM_CARGO + 2 )

// The lift's prismatic and the suspension's prismatic.
#define NUM_JOINTS 2

#define PLATFORM_HALF_X 1.3f
#define PLATFORM_HALF_Y 0.12f
#define PLATFORM_HALF_Z 1.0f

// Where the platform starts, and the range the limits allow it. The anchor sits
// at the origin, so these *are* the translations -- the readout on screen is
// the platform's height above the floor with no conversion.
#define LIFT_START_Y 1.0f
#define LIFT_LOWER 0.5f
#define LIFT_UPPER 4.0f

#define LIFT_SPEED 1.6f

// The drive's ceiling, and the number worth reading.
//
// The platform is 2.6 x 0.24 x 2.0 at density 1, so about 1.25 kg, and four
// 0.5-cubes are another 0.5 -- together roughly 17 N held against gravity, and
// more than that while accelerating. 400 N is far above it, so the lift is
// authoritative and tracks its command.
//
// Drop this below the load and the same scene becomes a lift with a weight
// limit: the platform sags under its cargo and recovers when the cargo is
// dropped off. That is a legitimate thing to want, and it is what the bound is
// *for* -- a drive that cannot lose is a teleport with extra steps.
#define MAX_MOTOR_FORCE 400.0f

#define BOX_HALF 0.25f

// The cargo sits to one side of the platform's centre on purpose. A centred
// load leaves the orientation lock with nothing to do; off-centre it carries a
// steady tipping torque for as long as the example runs, which is the only way
// to see that constraint working rather than merely present.
#define CARGO_OFFSET_X 0.6f

#define PAD_X 3.2f
#define PAD_HALF_X 0.8f
#define PAD_HALF_Y 0.1f
#define PAD_HALF_Z 0.8f
#define PAD_START_Y 1.2f

// The suspension's two settings. Stiff barely moves under the pad's own weight;
// soft visibly sags and oscillates when something lands on it.
#define SUSPENSION_STIFF 4.0f
#define SUSPENSION_SOFT 1.0f
#define SUSPENSION_DAMPING 0.7f

#define FLOOR_HALF 8.0f

// Below this a body counts as lost. box3d_basic's rule: the check is cheap and
// its absence is a slow leak that reads as a solver fault.
#define RESPAWN_BELOW ( -10.0f )

typedef struct
{
    NEA_Camera *camera;

    NEA_Model *cargo_model[NUM_CARGO];
    NEA_Model *platform_model;
    NEA_Model *pad_model;
    NEA_Model *floor_model;

    NEA_Phys3DBody *cargo[NUM_CARGO];
    NEA_Phys3DBody *platform;
    NEA_Phys3DBody *pad;
    NEA_Phys3DBody *lift_anchor;
    NEA_Phys3DBody *pad_anchor;
    NEA_Phys3DBody *floor_body;

    b3JointId lift;
    b3JointId suspension;
} SceneData;

// The joint frame that turns a slider into a *lift*.
//
// A prismatic slides along frame A's x. Rotating the frame a quarter turn about
// z takes x onto +y, so both bodies' frames are built with this and the rail
// stands upright. There is no axis parameter to set instead -- the frames are
// the axis, which is upstream's design and the same one the revolute uses for
// its hinge.
static b3Quat VerticalFrame(void)
{
    return b3MakeQuatFromAxisAngle(b3Vec3_axisZ, B3_BRAD_HALF_PI);
}

static float CargoBaseY(void)
{
    return LIFT_START_Y + PLATFORM_HALF_Y + BOX_HALF;
}

static void CargoOffset(int index, float *ox, float *oz)
{
    *ox = CARGO_OFFSET_X + (float)( index % 2 ) * ( 2.0f * BOX_HALF + 0.05f );
    *oz = ( (float)( index / 2 ) - 0.5f ) * ( 2.0f * BOX_HALF + 0.05f );
}

void Draw3DScene(void *arg)
{
    SceneData *scene = arg;

    NEA_CameraUse(scene->camera);

    NEA_PolyFormat(31, 0, NEA_LIGHT_0 | NEA_LIGHT_1, NEA_CULL_BACK, 0);

    for (int i = 0; i < NUM_CARGO; i++)
    {
        if (scene->cargo_model[i] != NULL)
            NEA_ModelDraw(scene->cargo_model[i]);
    }

    NEA_PolyFormat(28, 0, NEA_LIGHT_0 | NEA_LIGHT_1, NEA_CULL_BACK, 0);
    NEA_ModelDraw(scene->platform_model);
    NEA_ModelDraw(scene->pad_model);

    NEA_PolyFormat(20, 0, NEA_LIGHT_0 | NEA_LIGHT_1, NEA_CULL_BACK, 0);
    NEA_ModelDraw(scene->floor_model);
}

static void ResetScene(SceneData *scene)
{
    for (int i = 0; i < NUM_CARGO; i++)
    {
        if (scene->cargo[i] == NULL)
            continue;

        float ox, oz;
        CargoOffset(i, &ox, &oz);

        NEA_Phys3DBodySetPosition(scene->cargo[i], ox, CargoBaseY(), oz);
        NEA_Phys3DBodySetVelocity(scene->cargo[i], 0, 0, 0);
        NEA_Phys3DBodySetAwake(scene->cargo[i], true);
    }

    if (scene->platform != NULL)
    {
        NEA_Phys3DBodySetPosition(scene->platform, 0, LIFT_START_Y, 0);
        NEA_Phys3DBodySetVelocity(scene->platform, 0, 0, 0);
        NEA_Phys3DBodySetAwake(scene->platform, true);
    }

    if (scene->pad != NULL)
    {
        NEA_Phys3DBodySetPosition(scene->pad, PAD_X, PAD_START_Y, 0);
        NEA_Phys3DBodySetVelocity(scene->pad, 0, 0, 0);
        NEA_Phys3DBodySetAwake(scene->pad, true);
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

    // The floor and the two joint anchors.
    def.box3d.capacity.staticBodyCount = 3;
    def.box3d.capacity.staticShapeCount = 3;

    // Cargo against the platform, against the pad, against the floor, and
    // against each other.
    //
    // Sixteen per body rather than box3d_motor's eight, and the difference is
    // measured rather than guessed. Cargo that slides off the platform lands on
    // a large floor and spreads out, which is a *more* contact-rich state than
    // the crate example ever reaches -- with eight the arena grew to 27 late
    // allocations before it settled. They were benign (0 overflow bytes; the
    // arena tracks peak demand and grows once, by design) but a reservation
    // that has to be corrected at runtime is a reservation that was wrong.
    def.box3d.capacity.contactCount = NUM_BODIES * 16;

    // A prismatic joint's sim is 228 bytes, inside the spherical joint's 260,
    // so b3JointSim is still 444 and Stage 6 cost these arrays nothing. Each
    // sim is reserved four times over regardless, because it migrates between
    // the constraint graph, the awake set, a sleeping set and the disabled set.
    // Declaring the count keeps all four out of `late` below.
    def.box3d.capacity.jointCount = NUM_JOINTS;

    // 192 KB rather than box3d_motor's 160, because the contact reservation
    // above is twice as large and the reservation is what the pool holds: at
    // 160 KB this scene built to 145 KB and sat at 89% of its own budget, which
    // leaves nothing for a scene that gets briefly busier. Headroom in a pool
    // that is sized once at startup costs nothing at runtime.
    def.poolBytes = 192 * 1024;

    if (NEA_Phys3DWorldInit(&def) != 0)
    {
        consoleDemoInit();
        printf("Could not create the physics world.\n");
        while (1)
            swiWaitForVBlank();
    }

    NEA_Phys3DWorldSetGravity(0, -9.8, 0);

    NEA_CameraSet(scene.camera = NEA_CameraCreate(),
                  1.5, 4.0, 10.0,
                  1.0, 2.0, 0,
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
    // The two rails' anchors
    // ---------------------------------------------------------------------
    //
    // Static bodies with no shape, at the origin of each rail. A prismatic's
    // translation is measured from where the two joint frames coincide, so
    // putting the anchor at floor level makes the lift's translation read as
    // its height -- and the limits below read as the floors it stops at.
    scene.lift_anchor = NEA_Phys3DBodyCreate(b3_staticBody, 0, 0, 0);
    if (scene.lift_anchor == NULL)
        bodiesMissing++;

    scene.pad_anchor = NEA_Phys3DBodyCreate(b3_staticBody, PAD_X, 0, 0);
    if (scene.pad_anchor == NULL)
        bodiesMissing++;

    // ---------------------------------------------------------------------
    // The lift platform
    // ---------------------------------------------------------------------

    scene.platform_model = NEA_ModelCreate(NEA_Static);
    NEA_ModelLoadStaticMesh(scene.platform_model, cube_bin);
    NEA_ModelScaleI(scene.platform_model, floattof32(PLATFORM_HALF_X),
                    floattof32(PLATFORM_HALF_Y), floattof32(PLATFORM_HALF_Z));

    scene.platform = NEA_Phys3DBodyCreate(b3_dynamicBody, 0, LIFT_START_Y, 0);
    if (scene.platform == NULL)
    {
        bodiesMissing++;
    }
    else
    {
        // Density 1, matching every other box3d example rather than Box3D's own
        // default of 1000.
        if (NEA_Phys3DBodyAddBox(scene.platform, PLATFORM_HALF_X, PLATFORM_HALF_Y,
                                 PLATFORM_HALF_Z, 1.0) != 0)
        {
            shapesMissing++;
        }

        // High friction: the cargo has to ride the platform rather than slide
        // off it the moment the lift accelerates.
        NEA_Phys3DBodySetMaterial(scene.platform, 0.9, 0.0);

        NEA_Phys3DBodySetModel(scene.platform, scene.platform_model);
    }

    // ---------------------------------------------------------------------
    // The suspension pad
    // ---------------------------------------------------------------------

    scene.pad_model = NEA_ModelCreate(NEA_Static);
    NEA_ModelLoadStaticMesh(scene.pad_model, cube_bin);
    NEA_ModelScaleI(scene.pad_model, floattof32(PAD_HALF_X), floattof32(PAD_HALF_Y),
                    floattof32(PAD_HALF_Z));

    scene.pad = NEA_Phys3DBodyCreate(b3_dynamicBody, PAD_X, PAD_START_Y, 0);
    if (scene.pad == NULL)
    {
        bodiesMissing++;
    }
    else
    {
        if (NEA_Phys3DBodyAddBox(scene.pad, PAD_HALF_X, PAD_HALF_Y, PAD_HALF_Z, 1.0) != 0)
            shapesMissing++;

        NEA_Phys3DBodySetMaterial(scene.pad, 0.7, 0.1);
        NEA_Phys3DBodySetModel(scene.pad, scene.pad_model);
    }

    // ---------------------------------------------------------------------
    // The cargo
    // ---------------------------------------------------------------------

    for (int i = 0; i < NUM_CARGO; i++)
    {
        float ox, oz;
        CargoOffset(i, &ox, &oz);

        scene.cargo_model[i] = NEA_ModelCreate(NEA_Static);
        NEA_ModelLoadStaticMesh(scene.cargo_model[i], cube_bin);
        NEA_ModelScaleI(scene.cargo_model[i], floattof32(BOX_HALF), floattof32(BOX_HALF),
                        floattof32(BOX_HALF));

        scene.cargo[i] = NEA_Phys3DBodyCreate(b3_dynamicBody, ox, CargoBaseY(), oz);
        if (scene.cargo[i] == NULL)
        {
            bodiesMissing++;
            continue;
        }

        if (NEA_Phys3DBodyAddBox(scene.cargo[i], BOX_HALF, BOX_HALF, BOX_HALF, 1.0) != 0)
            shapesMissing++;

        NEA_Phys3DBodySetMaterial(scene.cargo[i], 0.7, 0.05);
        NEA_Phys3DBodySetModel(scene.cargo[i], scene.cargo_model[i]);
    }

    // ---------------------------------------------------------------------
    // The lift's prismatic joint
    // ---------------------------------------------------------------------

    if (scene.platform != NULL && scene.lift_anchor != NULL)
    {
        b3PrismaticJointDef lift = b3DefaultPrismaticJointDef();
        lift.base.bodyIdA = scene.lift_anchor->id;
        lift.base.bodyIdB = scene.platform->id;

        // Both frames rotated so the rail stands upright, and both frame
        // origins left at their bodies' origins -- so the translation is the
        // platform's height above the anchor, which sits on the floor.
        lift.base.localFrameA.q = VerticalFrame();
        lift.base.localFrameB.q = VerticalFrame();

        lift.enableLimit = true;
        lift.lowerTranslation = b3fFromDouble(LIFT_LOWER);
        lift.upperTranslation = b3fFromDouble(LIFT_UPPER);

        lift.enableMotor = true;
        lift.motorSpeed = b3f_zero;
        lift.maxMotorForce = b3fFromDouble(MAX_MOTOR_FORCE);

        scene.lift = b3CreatePrismaticJoint(NEA_Phys3DWorldGetId(), &lift);
        if (b3Joint_IsValid(scene.lift) == false)
            jointsMissing++;
    }
    else
    {
        jointsMissing++;
    }

    // ---------------------------------------------------------------------
    // The suspension leg's prismatic joint
    // ---------------------------------------------------------------------

    if (scene.pad != NULL && scene.pad_anchor != NULL)
    {
        b3PrismaticJointDef susp = b3DefaultPrismaticJointDef();
        susp.base.bodyIdA = scene.pad_anchor->id;
        susp.base.bodyIdB = scene.pad->id;

        susp.base.localFrameA.q = VerticalFrame();
        susp.base.localFrameB.q = VerticalFrame();

        // The spring pulls the pad toward its rest height. Zero hertz would
        // *disable* the spring here rather than making it rigid -- a prismatic
        // has no rest length of its own, so an unsprung leg simply falls. That
        // is the difference from a weld joint, whose zero hertz means rigid.
        susp.enableSpring = true;
        susp.hertz = b3fFromDouble(SUSPENSION_STIFF);
        susp.dampingRatio = b3fFromDouble(SUSPENSION_DAMPING);
        susp.targetTranslation = b3fFromDouble(PAD_START_Y);

        // Stops, so a hard landing cannot drive the pad through the floor.
        susp.enableLimit = true;
        susp.lowerTranslation = b3fFromDouble(0.3);
        susp.upperTranslation = b3fFromDouble(PAD_START_Y + 0.4f);

        scene.suspension = b3CreatePrismaticJoint(NEA_Phys3DWorldGetId(), &susp);
        if (b3Joint_IsValid(scene.suspension) == false)
            jointsMissing++;
    }
    else
    {
        jointsMissing++;
    }

    int32_t buildBytes = NEA_Phys3DWorldGetMemoryUsage();

    consoleDemoInit();

    printf("NEA Box3D Prismatic\n");
    printf("-------------------\n");
    printf("A/B: drive lift up/down\n");
    printf("X: toggle limit  Y: spring\n");
    printf("SELECT: reset\n\n");

    uint32_t stepTicks = 0;
    uint32_t peakTicks = 0;
    int peakAwake = 0;
    uint32_t frames = 0;
    int respawns = 0;

    bool limited = true;
    bool stiff = true;

    // The ramped motor command; see the ramp below.
    float commanded = 0.0f;

    while (1)
    {
        scanKeys();
        uint32_t keys = keysHeld();
        uint32_t down = keysDown();

#ifdef BOX3D_NO_INPUT
        // Baseline measurement build. melonDS injects stray keyboard events
        // into the focused window, and here they would land on X (removing the
        // limit) or A/B (driving the lift), either of which changes what is
        // being measured.
        //
        // Same switch as box3d_basic, box3d_ragdoll and box3d_motor, and for
        // the same reason: it is what makes the recorded numbers a property of
        // the simulation rather than of the emulator.
        keys = 0;
        down = 0;
#endif

#ifdef BOX3D_AUTO_DRIVE
        // Verification build only: sweep the lift on a fixed schedule so the
        // headline behaviour can be measured without depending on emulator
        // keyboard input, which BOX3D_NO_INPUT exists to distrust. Up for two
        // seconds, down for two, forever -- which drives the platform onto both
        // limits in turn.
        if (( frames / 120 ) % 2 == 0)
            keys |= KEY_A;
        else
            keys |= KEY_B;
#endif

        // The motor's target speed, set from input. Zero means "hold station",
        // and the drive then spends exactly the weight it is carrying -- which
        // is what the force readout below shows.
        float wanted = 0.0f;
        if (keys & KEY_A)
            wanted = LIFT_SPEED;
        else if (keys & KEY_B)
            wanted = -LIFT_SPEED;

        // Ramped rather than commanded outright, and the reason is the cargo.
        //
        // The force bound is 400 N against a 17 N load, so a step change in the
        // target is a step change in *acceleration* -- around 200 m/s^2, which
        // launches four loose boxes off the platform on the first frame of every
        // reversal. Friction cannot hold what gravity is not pressing down.
        //
        // A real lift ramps, and so does this: the command approaches its target
        // over about a third of a second. The bound stays where it is, because
        // the bound is a *ceiling* on authority and not a substitute for a
        // controller -- lowering it to smooth the ride would also make the lift
        // sag under load, which is a different behaviour that X and the header
        // already describe.
        commanded += ( wanted - commanded ) * 0.05f;

        if (b3Joint_IsValid(scene.lift))
        {
            b3PrismaticJoint_SetMotorSpeed(scene.lift, b3fFromDouble(commanded));

            // The accessors write the target and nothing else, like every other
            // joint accessor that changes a target rather than a topology. A
            // platform that had gone to sleep would therefore ignore a new
            // command forever, which looks exactly like a dead motor. Waking it
            // is the caller's job, and this is the caller.
            if (( wanted != 0.0f || commanded != 0.0f ) && scene.platform != NULL)
                NEA_Phys3DBodySetAwake(scene.platform, true);
        }

        // The headline, in one button. With the limit on, the driven platform
        // stops at the top of its rail and stays there under power. With it off
        // it keeps going -- because a limit is an impulse the solver applies,
        // not a clamp on a number, and nothing is left to apply it.
        if (down & KEY_X)
        {
            limited = !limited;
            if (b3Joint_IsValid(scene.lift))
            {
                b3PrismaticJoint_EnableLimit(scene.lift, limited);
                if (scene.platform != NULL)
                    NEA_Phys3DBodySetAwake(scene.platform, true);
            }
        }

        if (down & KEY_Y)
        {
            stiff = !stiff;
            if (b3Joint_IsValid(scene.suspension))
            {
                b3PrismaticJoint_SetSpringHertz(
                    scene.suspension,
                    b3fFromDouble(stiff ? SUSPENSION_STIFF : SUSPENSION_SOFT));
                if (scene.pad != NULL)
                    NEA_Phys3DBodySetAwake(scene.pad, true);
            }
        }

        if (down & KEY_SELECT)
            ResetScene(&scene);

        // Stepped by hand so it can be timed, hence NEA_CAN_SKIP_VBL below
        // rather than NEA_UPDATE_PHYS3D, which would step the world twice.
        cpuStartTiming(0);
        NEA_Phys3DWorldStep();
        stepTicks = cpuEndTiming();
        frames++;

        NEA_Phys3DSyncModels();

        for (int i = 0; i < NUM_CARGO; i++)
        {
            if (scene.cargo[i] == NULL)
                continue;

            int32_t bx, by, bz;
            NEA_Phys3DBodyGetPositionI(scene.cargo[i], &bx, &by, &bz);

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

        // The joint's own readouts. The height is b3PrismaticJoint_GetTranslation
        // straight out -- the anchor is on the floor, so no conversion. The
        // drive is what the motor is currently spending, and at rest it equals
        // the weight the lift carries: watch it rise as the cargo settles and
        // fall to nothing when the platform is parked on its lower limit, where
        // the *limit* takes the load instead of the motor.
        int heightMilli = 0;
        int driveN = 0;
        int sagMilli = 0;

        if (b3Joint_IsValid(scene.lift))
        {
            heightMilli = (int)(b3fToDouble(b3PrismaticJoint_GetTranslation(scene.lift)) * 1000.0);
            driveN = (int)fabs(b3fToDouble(b3PrismaticJoint_GetMotorForce(scene.lift)));
        }

        if (b3Joint_IsValid(scene.suspension))
        {
            double rest = PAD_START_Y;
            double now = b3fToDouble(b3PrismaticJoint_GetTranslation(scene.suspension));
            sagMilli = (int)(( rest - now ) * 1000.0);
        }

        printf("\x1b[8;0Hstep   %5lu us (%lu ticks)  ",
               (unsigned long)stepMicros, (unsigned long)stepTicks);
        printf("\x1b[9;0Hpeak   %5lu us @ %d awake   ",
               (unsigned long)peakMicros, peakAwake);
        printf("\x1b[10;0Hawake  %2d / %d bodies       ", awake, NUM_BODIES);
        printf("\x1b[11;0Hlift   %5d mm  drive %4d N ", heightMilli, driveN);
        printf("\x1b[12;0Hlimit  %-3s   spring %-5s   ",
               limited ? "on" : "OFF", stiff ? "stiff" : "soft");
        printf("\x1b[13;0Hsag    %5d mm              ", sagMilli);
        printf("\x1b[14;0Hpool   %5ld / %ld bytes     ",
               (long)NEA_Phys3DWorldGetMemoryUsage(),
               (long)NEA_Phys3DWorldGetMemoryCapacity());
        printf("\x1b[15;0Hbuild  %5ld bytes           ", (long)buildBytes);
        printf("\x1b[16;0Hlate   %5d allocs, %ld ovf  ",
               NEA_Phys3DWorldGetLateAllocCount(),
               (long)NEA_Phys3DWorldGetOverflowBytes());
        printf("\x1b[17;0Hcpu    %3d %%                ", NEA_GetCPUPercent());
        printf("\x1b[18;0Hmissing %d bd %d jt %d shp   ",
               bodiesMissing, jointsMissing, shapesMissing);
        printf("\x1b[19;0Hrespawn %3d  frame %8lu    ", respawns,
               (unsigned long)frames);

        NEA_WaitForVBL(NEA_CAN_SKIP_VBL);
        NEA_ProcessArg(Draw3DScene, &scene);
    }

    return 0;
}
