// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Warioware64, 2026
//
// This file is part of Nitro Engine Advanced
//
// Box3D spherical joint example
// -----------------------------
// A ragdoll: a torso with a head, two arms and two legs, every limb hung on a
// **ball joint** with a cone limit and a twist limit. It falls onto a floor and
// settles. D-pad shoves the torso, X toggles the limits, B lifts the whole
// thing, A resets the pose.
//
// The counterpart of box3d_hinge. A hinge leaves one rotational degree free and
// this leaves all three, then *bounds* them -- which is what a shoulder is, and
// why a ragdoll rather than a plainer scene: a chain of ball joints with no
// limits would exercise the point constraint and nothing else, and the point
// constraint is the revolute's, already measured.
//
// Turning the limits off with X is the whole demonstration in one button. With
// them on the figure keeps its shape; with them off the limbs fold through
// themselves and it collapses into a heap.
//
// As in box3d_rope and box3d_hinge there are no NEA_Phys3DJoint* wrappers yet,
// so this calls Box3D's joint API directly through NEA_Phys3DBody::id.

#include <NEAMain.h>
#include <NEAPhysics3D.h>

#include <box3d/box3d.h>

#include <nds/arm9/exceptions.h>

#include <math.h>

#include "cube_bin.h"

// Torso, head, two arms, two legs.
#define NUM_LIMBS  6
#define NUM_BODIES ( NUM_LIMBS + 1 )
#define NUM_JOINTS NUM_LIMBS

#define TORSO_HALF_W 0.45f
#define TORSO_HALF_H 0.65f
#define TORSO_HALF_D 0.25f

#define HEAD_HALF 0.28f
#define LIMB_HALF_W 0.16f
#define LIMB_HALF_H 0.55f

#define START_Y 4.2f

#define FLOOR_HALF 6.0f

// Below this a limb counts as lost. The floor makes it hard to reach, which is
// the point -- but a game needs the check regardless, for box3d_basic's reason.
#define RESPAWN_BELOW ( -12.0f )

typedef struct
{
    NEA_Camera *camera;

    NEA_Model *limb_model[NUM_LIMBS];
    NEA_Model *floor_model;

    NEA_Phys3DBody *limb[NUM_LIMBS];
    NEA_Phys3DBody *floor_body;

    b3JointId joint[NUM_JOINTS];
} SceneData;

// Where each limb sits relative to the torso's centre, and how big it is.
// Index 0 is the torso itself, which hangs off nothing.
static const float s_limbOffset[NUM_LIMBS][3] = {
    {  0.0f,  0.0f,               0.0f },  // torso
    {  0.0f,  TORSO_HALF_H + 0.30f, 0.0f },  // head
    { -0.72f, 0.25f,              0.0f },  // left arm
    {  0.72f, 0.25f,              0.0f },  // right arm
    { -0.26f, -( TORSO_HALF_H + LIMB_HALF_H ), 0.0f }, // left leg
    {  0.26f, -( TORSO_HALF_H + LIMB_HALF_H ), 0.0f }, // right leg
};

static const float s_limbHalf[NUM_LIMBS][3] = {
    { TORSO_HALF_W, TORSO_HALF_H, TORSO_HALF_D },
    { HEAD_HALF, HEAD_HALF, HEAD_HALF },
    { LIMB_HALF_W, LIMB_HALF_H, LIMB_HALF_W },
    { LIMB_HALF_W, LIMB_HALF_H, LIMB_HALF_W },
    { LIMB_HALF_W, LIMB_HALF_H, LIMB_HALF_W },
    { LIMB_HALF_W, LIMB_HALF_H, LIMB_HALF_W },
};

void Draw3DScene(void *arg)
{
    SceneData *scene = arg;

    NEA_CameraUse(scene->camera);

    NEA_PolyFormat(31, 0, NEA_LIGHT_0 | NEA_LIGHT_1, NEA_CULL_BACK, 0);

    for (int i = 0; i < NUM_LIMBS; i++)
    {
        if (scene->limb_model[i] != NULL)
            NEA_ModelDraw(scene->limb_model[i]);
    }

    NEA_PolyFormat(20, 0, NEA_LIGHT_0 | NEA_LIGHT_1, NEA_CULL_BACK, 0);
    NEA_ModelDraw(scene->floor_model);
}

static void ResetScene(SceneData *scene)
{
    for (int i = 0; i < NUM_LIMBS; i++)
    {
        if (scene->limb[i] == NULL)
            continue;

        NEA_Phys3DBodySetPosition(scene->limb[i], s_limbOffset[i][0],
                                  START_Y + s_limbOffset[i][1],
                                  s_limbOffset[i][2]);
        NEA_Phys3DBodySetVelocity(scene->limb[i], 0, 0, 0);
        NEA_Phys3DBodySetAwake(scene->limb[i], true);
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

    def.box3d.capacity.dynamicBodyCount = NUM_LIMBS;
    def.box3d.capacity.dynamicShapeCount = NUM_LIMBS;
    def.box3d.capacity.staticBodyCount = 2;
    def.box3d.capacity.staticShapeCount = 2;

    // A ragdoll on a floor touches a lot: limbs against the floor, and limbs
    // against each other once the limits are off and it folds up.
    def.box3d.capacity.contactCount = NUM_LIMBS * 6;

    // A spherical joint's sim is the largest in the union so far -- 444 bytes
    // against the revolute's 376 and the distance joint's 300 -- and it is
    // reserved in four places, because it migrates between the constraint
    // graph, the awake set, a sleeping set and the disabled set as its bodies
    // change state. Declaring the count keeps all four out of `late` below.
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
                  0, 3.4, 8.5,
                  0, 1.8, 0,
                  0, 1, 0);

    NEA_LightSet(0, NEA_White, 0, -1, -1);
    NEA_LightSet(1, NEA_Blue, -1, 0, 0);

    NEA_ClearColorSet(RGB15(4, 4, 8), 31, 63);

    // ---------------------------------------------------------------------
    // The floor
    // ---------------------------------------------------------------------

    scene.floor_model = NEA_ModelCreate(NEA_Static);
    NEA_ModelLoadStaticMesh(scene.floor_model, cube_bin);
    NEA_ModelSetCoord(scene.floor_model, 0, -0.25, 0);
    NEA_ModelScaleI(scene.floor_model, floattof32(FLOOR_HALF), inttof32(1) / 4,
                    floattof32(FLOOR_HALF));

    // NEA_Phys3DBodyAddBox returns -1 when the box-hull pool is full -- it is
    // sized staticShapeCount + dynamicShapeCount, which is 8 here against the 7
    // this scene needs. Checked rather than assumed, because the failure is
    // invisible: the floor *model* draws whether or not the floor body ever got
    // a shape, so the scene would look right while everything fell through it.
    int shapesMissing = 0;

    scene.floor_body = NEA_Phys3DBodyCreate(b3_staticBody, 0, -0.25, 0);
    if (NEA_Phys3DBodyAddBox(scene.floor_body, FLOOR_HALF, 0.25, FLOOR_HALF, 1.0) != 0)
        shapesMissing++;
    NEA_Phys3DBodySetMaterial(scene.floor_body, 0.6, 0.05);

    // ---------------------------------------------------------------------
    // The limbs
    // ---------------------------------------------------------------------

    int bodiesMissing = 0;

    for (int i = 0; i < NUM_LIMBS; i++)
    {
        scene.limb_model[i] = NEA_ModelCreate(NEA_Static);
        NEA_ModelLoadStaticMesh(scene.limb_model[i], cube_bin);
        NEA_ModelScaleI(scene.limb_model[i], floattof32(s_limbHalf[i][0]),
                        floattof32(s_limbHalf[i][1]), floattof32(s_limbHalf[i][2]));

        scene.limb[i] = NEA_Phys3DBodyCreate(b3_dynamicBody, s_limbOffset[i][0],
                                             START_Y + s_limbOffset[i][1],
                                             s_limbOffset[i][2]);
        if (scene.limb[i] == NULL)
        {
            // Checked rather than assumed: a limb that quietly failed to exist
            // is a hole in the figure that reads as a joint bug.
            bodiesMissing++;
            continue;
        }

        // Density 1, matching box3d_basic and box3d_hinge rather than Box3D's
        // own default of 1000 -- and deliberately, because it is the harder
        // case. These limbs are small *and* thin, so at this density the head's
        // inverse inertia is 109 and an arm's is 81, against the ceiling of 128
        // that b3iw's Q7.24 imposes. A joint divides by the sum of two of
        // those, and head-plus-torso lands at 128.197.
        //
        // That used to wrap negative in b3AddMWMW and fling the figure apart on
        // the first upright frame. b3AxisInertiaSumWide now carries the sum
        // wide, and tests/box3d_host/test_world.c runs this scene at both
        // densities so the light path stays covered.
        if (NEA_Phys3DBodyAddBox(scene.limb[i], s_limbHalf[i][0], s_limbHalf[i][1],
                                 s_limbHalf[i][2], 1.0) != 0)
        {
            shapesMissing++;
        }
        NEA_Phys3DBodySetMaterial(scene.limb[i], 0.5, 0.05);
        NEA_Phys3DBodySetModel(scene.limb[i], scene.limb_model[i]);

        // A spherical joint constrains, it does not dissipate -- the same
        // property box3d_rope and box3d_hinge both had to answer for. Without
        // damping the figure jiggles indefinitely and never sleeps, which would
        // cost this example half of what it measures. A ragdoll wants it
        // anyway: undamped limbs read as a puppet on strings rather than as a
        // body.
        b3Body_SetLinearDamping(scene.limb[i]->id, b3fFromDouble(0.4));
        b3Body_SetAngularDamping(scene.limb[i]->id, b3fFromDouble(0.8));
    }

    // ---------------------------------------------------------------------
    // The joints
    // ---------------------------------------------------------------------
    //
    // One ball joint per limb, all to the torso. Each joint's frames are placed
    // at the *socket* -- the point on the torso the limb pivots about, and the
    // matching point on the limb -- which is the thing box3d_rope got wrong the
    // first time and which reads exactly like a solver fault when it is wrong.
    //
    // The cone axis is the joint frames' z, and each socket wants its cone
    // pointing the way the limb hangs, so every frame here is rotated to aim it.

    int jointsMissing = 0;

    // Cone half-angles, per limb: a head barely moves, an arm swings widely, a
    // leg is somewhere between.
    static const float coneDeg[NUM_LIMBS] = { 0.0f, 25.0f, 70.0f, 70.0f, 45.0f, 45.0f };
    static const float twistDeg[NUM_LIMBS] = { 0.0f, 20.0f, 45.0f, 45.0f, 25.0f, 25.0f };

    for (int i = 1; i < NUM_LIMBS; i++)
    {
        if (scene.limb[i] == NULL || scene.limb[0] == NULL)
        {
            jointsMissing++;
            continue;
        }

        b3SphericalJointDef ball = b3DefaultSphericalJointDef();
        ball.base.bodyIdA = scene.limb[0]->id;
        ball.base.bodyIdB = scene.limb[i]->id;

        // The socket, in each body's own space. The head sits above the torso,
        // the arms out to the sides, the leg below -- and in every case the
        // joint frame goes at the surface between them, not at either centre.
        float socketX = s_limbOffset[i][0] * 0.5f;
        float socketY = ( i == 1 ) ? TORSO_HALF_H
                                   : ( ( i >= 4 ) ? -TORSO_HALF_H : 0.35f );

        // Frame A is the socket in the torso's space; frame B is the **same
        // world point** in the limb's space, which is socket minus limb, not
        // limb minus socket. Getting that subtraction backwards puts the two
        // frames on opposite sides of the socket, so the solver opens by
        // closing a gap of twice the offset -- the figure detonates on frame
        // one and reads exactly like a broken constraint. It is box3d_rope's
        // anchor bug in its other form, and this example hit it too: 21
        // respawns, and every joint pinned at its limit.
        ball.base.localFrameA.p = b3MakeVec3(b3fFromDouble(socketX),
                                             b3fFromDouble(socketY), b3f_zero);
        ball.base.localFrameB.p =
            b3MakeVec3(b3fFromDouble(socketX - s_limbOffset[i][0]),
                       b3fFromDouble(socketY - s_limbOffset[i][1]), b3f_zero);

        // Aim the cone. The head's cone points up (+y); the arms' and the leg's
        // point down (-y), which is the way they hang. A quarter turn about x
        // takes +z to -y, and the opposite quarter takes it to +y.
        b3a aim = ( i == 1 ) ? (b3a)B3_BRAD_HALF_PI : (b3a)( -B3_BRAD_HALF_PI );
        b3Quat toAxis = b3MakeQuatFromAxisAngle(
            b3MakeVec3(b3f_one, b3f_zero, b3f_zero), aim);
        ball.base.localFrameA.q = toAxis;
        ball.base.localFrameB.q = toAxis;

        ball.enableConeLimit = true;
        ball.coneAngle = (b3a)(coneDeg[i] * (32768.0f / 360.0f));

        ball.enableTwistLimit = true;
        ball.lowerTwistAngle = (b3a)(-twistDeg[i] * (32768.0f / 360.0f));
        ball.upperTwistAngle = (b3a)(twistDeg[i] * (32768.0f / 360.0f));

        scene.joint[i] = b3CreateSphericalJoint(NEA_Phys3DWorldGetId(), &ball);
        if (b3Joint_IsValid(scene.joint[i]) == false)
            jointsMissing++;
    }

    int32_t buildBytes = NEA_Phys3DWorldGetMemoryUsage();

    consoleDemoInit();

    printf("NEA Box3D Ragdoll\n");
    printf("-----------------\n");
    printf("D-pad: shove torso\n");
    printf("X: toggle limits\n");
    printf("B: lift   A: reset\n\n");

    uint32_t stepTicks = 0;
    uint32_t peakTicks = 0;
    int peakAwake = 0;
    uint32_t frames = 0;
    int respawns = 0;

    bool limitsOn = true;

    while (1)
    {
        scanKeys();
        uint32_t keys = keysHeld();
        uint32_t down = keysDown();

#ifdef BOX3D_NO_INPUT
        // Baseline measurement build. melonDS injects stray keyboard events
        // into the focused window, and in this scene they land on the D-pad
        // (a 60 N shove on the torso) and B (a 5 m/s lift on every limb) --
        // which tears the figure off the floor, sends limbs past the respawn
        // plane and drives the joints to their limits. Measured that way the
        // ragdoll reads as a broken constraint; gated, it settles.
        //
        // Same switch as box3d_basic, and for the same reason: it is what
        // makes the recorded numbers a property of the simulation rather than
        // of the emulator.
        keys = 0;
        down = 0;
#endif

        if (scene.limb[0] != NULL)
        {
            if (keys & KEY_RIGHT)
                NEA_Phys3DBodyApplyForce(scene.limb[0], 60, 0, 0);
            if (keys & KEY_LEFT)
                NEA_Phys3DBodyApplyForce(scene.limb[0], -60, 0, 0);
            if (keys & KEY_UP)
                NEA_Phys3DBodyApplyForce(scene.limb[0], 0, 0, -60);
            if (keys & KEY_DOWN)
                NEA_Phys3DBodyApplyForce(scene.limb[0], 0, 0, 60);

            if (down & KEY_B)
            {
                for (int i = 0; i < NUM_LIMBS; i++)
                {
                    if (scene.limb[i] != NULL)
                        NEA_Phys3DBodyApplyImpulse(scene.limb[i], 0, 5, 0);
                }
            }
        }

        // The demonstration in one button. With the limits on the figure keeps
        // its shape; with them off the limbs fold through themselves and it
        // collapses -- which is the difference between a ball joint that only
        // holds a point and one that also bounds a cone and a twist.
        if (down & KEY_X)
        {
            limitsOn = !limitsOn;
            for (int i = 1; i < NUM_LIMBS; i++)
            {
                if (b3Joint_IsValid(scene.joint[i]))
                {
                    b3SphericalJoint_EnableConeLimit(scene.joint[i], limitsOn);
                    b3SphericalJoint_EnableTwistLimit(scene.joint[i], limitsOn);
                }
            }
        }

        if (down & KEY_A)
            ResetScene(&scene);

        // Stepped by hand so it can be timed, hence NEA_CAN_SKIP_VBL below
        // rather than NEA_UPDATE_PHYS3D, which would step the world twice.
        cpuStartTiming(0);
        NEA_Phys3DWorldStep();
        stepTicks = cpuEndTiming();
        frames++;

        NEA_Phys3DSyncModels();

        for (int i = 0; i < NUM_LIMBS; i++)
        {
            if (scene.limb[i] == NULL)
                continue;

            int32_t bx, by, bz;
            NEA_Phys3DBodyGetPositionI(scene.limb[i], &bx, &by, &bz);

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

        // The right arm's two angles, which are the joint's own error measure:
        // with the limits on the cone must stay inside 70 degrees and the twist
        // inside 45, and both are what you can check against the figure on
        // screen. Neither dithers the way a reaction torque does -- an angle is
        // read from the transform, not from an accumulator.
        int coneAngle = 0;
        int twistAngle = 0;
        if (b3Joint_IsValid(scene.joint[3]))
        {
            coneAngle = (int)((double)b3SphericalJoint_GetConeAngle(scene.joint[3])
                              * (360.0 / 32768.0));
            twistAngle = (int)((double)b3SphericalJoint_GetTwistAngle(scene.joint[3])
                               * (360.0 / 32768.0));
        }

        printf("\x1b[8;0Hstep   %5lu us (%lu ticks)  ",
               (unsigned long)stepMicros, (unsigned long)stepTicks);
        printf("\x1b[9;0Hpeak   %5lu us @ %d awake   ",
               (unsigned long)peakMicros, peakAwake);
        printf("\x1b[10;0Hawake  %2d / %d bodies       ", awake, NUM_LIMBS);
        printf("\x1b[11;0Hlimits %s                   ", limitsOn ? "on " : "off");
        printf("\x1b[12;0Harm   cone %3d  twist %4d  ", coneAngle, twistAngle);
        printf("\x1b[13;0Hpool   %5ld / %ld bytes     ",
               (long)NEA_Phys3DWorldGetMemoryUsage(),
               (long)NEA_Phys3DWorldGetMemoryCapacity());
        printf("\x1b[14;0Hbuild  %5ld bytes           ", (long)buildBytes);
        printf("\x1b[15;0Hlate   %5d allocs, %ld ovf  ",
               NEA_Phys3DWorldGetLateAllocCount(),
               (long)NEA_Phys3DWorldGetOverflowBytes());
        printf("\x1b[16;0Hcpu %3d %%  fps %2d             ",
               NEA_GetCPUPercent(), NEA_GetFPS());
        printf("\x1b[17;0Hmissing %d bd %d jt %d shp   ",
               bodiesMissing, jointsMissing, shapesMissing);
        printf("\x1b[18;0Hrespawn %3d  frame %8lu    ", respawns,
               (unsigned long)frames);

        NEA_WaitForVBL(NEA_CAN_SKIP_VBL);
        NEA_ProcessArg(Draw3DScene, &scene);
    }

    return 0;
}
