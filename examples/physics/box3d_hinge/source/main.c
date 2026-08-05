// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Warioware64, 2026
//
// This file is part of Nitro Engine Advanced
//
// Box3D revolute joint example
// ----------------------------
// Three hinges, which are the three things one joint can be: a **door** on a
// limited hinge that swings and stops, a **motor-driven turntable**, and a
// **free-swinging arm**. D-pad pushes the door, X toggles the motor, Y toggles
// the door's limit, A resets.
//
// The counterpart of box3d_rope. That example measures a joint that constrains
// a *length*; this one measures the first that constrains a *rotation*, which
// is three constraints solved together rather than one -- a 3x3 point-to-point
// lock, a 2x2 axis lock, and the hinge angle itself. The step time here is
// what that costs.
//
// As in box3d_rope there are no NEA_Phys3DJoint* wrappers yet, so this calls
// Box3D's joint API directly through NEA_Phys3DBody::id.

#include <NEAMain.h>
#include <NEAPhysics3D.h>

#include <box3d/box3d.h>

#include <nds/arm9/exceptions.h>

#include <math.h>

#include "cube_bin.h"

#define NUM_BODIES 3
#define NUM_JOINTS 3

// The door: a tall thin panel hinged along its left edge.
#define DOOR_HALF_W 0.9f
#define DOOR_HALF_H 1.1f
#define DOOR_HALF_D 0.12f
#define DOOR_X      (-2.6f)
#define DOOR_Y      2.2f

// The turntable: a wide flat plate spun by a motor.
#define TABLE_HALF  1.0f
#define TABLE_THICK 0.12f
#define TABLE_X     0.6f
#define TABLE_Y     2.2f

// The free arm: hinged at the top, swinging like a pendulum.
#define ARM_HALF_W  0.14f
#define ARM_HALF_H  1.0f
#define ARM_X       3.4f
#define ARM_PIVOT_Y 4.0f

typedef struct {
    NEA_Camera *camera;

    NEA_Model *door_model;
    NEA_Model *table_model;
    NEA_Model *arm_model;
    NEA_Model *post_model;

    NEA_Phys3DBody *door;
    NEA_Phys3DBody *table;
    NEA_Phys3DBody *arm;
    NEA_Phys3DBody *anchor;

    b3JointId doorJoint;
    b3JointId tableJoint;
    b3JointId armJoint;
} SceneData;

void Draw3DScene(void *arg)
{
    SceneData *scene = arg;

    NEA_CameraUse(scene->camera);

    NEA_PolyFormat(31, 0, NEA_LIGHT_0 | NEA_LIGHT_1, NEA_CULL_BACK, 0);

    NEA_ModelDraw(scene->door_model);
    NEA_ModelDraw(scene->table_model);
    NEA_ModelDraw(scene->arm_model);

    NEA_PolyFormat(20, 0, NEA_LIGHT_0 | NEA_LIGHT_1, NEA_CULL_BACK, 0);
    NEA_ModelDraw(scene->post_model);
}

static void ResetScene(SceneData *scene)
{
    if (scene->door != NULL)
    {
        NEA_Phys3DBodySetPosition(scene->door, DOOR_X + DOOR_HALF_W, DOOR_Y, 0);
        NEA_Phys3DBodySetVelocity(scene->door, 0, 0, 0);
        NEA_Phys3DBodySetAwake(scene->door, true);
    }

    if (scene->arm != NULL)
    {
        NEA_Phys3DBodySetPosition(scene->arm, ARM_X, ARM_PIVOT_Y - ARM_HALF_H, 0);
        NEA_Phys3DBodySetVelocity(scene->arm, 0, 0, 0);
        NEA_Phys3DBodySetAwake(scene->arm, true);
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
    def.box3d.capacity.staticBodyCount = 4;
    def.box3d.capacity.staticShapeCount = 4;
    def.box3d.capacity.contactCount = NUM_BODIES * 2;

    // A revolute joint's sim is the largest in the union so far -- 376 bytes
    // against the distance joint's 300 -- and it is reserved in four places,
    // because it migrates between the constraint graph, the awake set, a
    // sleeping set and the disabled set as its bodies change state. Declaring
    // the count keeps all four out of the `late` counter below.
    def.box3d.capacity.jointCount = NUM_JOINTS;

    def.poolBytes = 128 * 1024;

    if (NEA_Phys3DWorldInit(&def) != 0)
    {
        consoleDemoInit();
        printf("Could not create the physics world.\n");
        while (1)
            swiWaitForVBlank();
    }

    NEA_Phys3DWorldSetGravity(0, -9.8, 0);

    // Close enough that the door reads as a door. The three hinges span x from
    // -2.6 to 3.4, so the camera is centred on that and pulled in until the
    // panel's swing is legible -- a demo whose measurement you can read but
    // whose motion you cannot is only half a demo.
    NEA_CameraSet(scene.camera = NEA_CameraCreate(),
                  0.4, 2.6, 8.0,
                  0.4, 2.2, 0,
                  0, 1, 0);

    NEA_LightSet(0, NEA_White, 0, -1, -1);
    NEA_LightSet(1, NEA_Blue, -1, 0, 0);

    NEA_ClearColorSet(RGB15(4, 4, 8), 31, 63);

    // ---------------------------------------------------------------------
    // The anchor
    // ---------------------------------------------------------------------
    //
    // One static body at the origin carrying all three hinges. Each joint
    // offsets its own frame A to put its pivot where it belongs, which is what
    // b3JointDef's local frames are for -- and is the thing box3d_rope got
    // wrong the first time, hauling its whole rope sideways because the top
    // joint measured from the bar's centre instead of from the hang point.
    scene.anchor = NEA_Phys3DBodyCreate(b3_staticBody, 0, 0, 0);

    scene.post_model = NEA_ModelCreate(NEA_Static);
    NEA_ModelLoadStaticMesh(scene.post_model, cube_bin);
    NEA_ModelSetCoord(scene.post_model, ARM_X, ARM_PIVOT_Y, 0);
    NEA_ModelScaleI(scene.post_model, floattof32(0.16f), floattof32(0.16f),
                    floattof32(0.16f));

    int bodiesMissing = 0;

    // ---------------------------------------------------------------------
    // The door -- a hinge with an angle limit
    // ---------------------------------------------------------------------

    scene.door_model = NEA_ModelCreate(NEA_Static);
    NEA_ModelLoadStaticMesh(scene.door_model, cube_bin);
    NEA_ModelScaleI(scene.door_model, floattof32(DOOR_HALF_W),
                    floattof32(DOOR_HALF_H), floattof32(DOOR_HALF_D));

    scene.door = NEA_Phys3DBodyCreate(b3_dynamicBody, DOOR_X + DOOR_HALF_W, DOOR_Y, 0);
    if (scene.door == NULL)
    {
        bodiesMissing++;
    }
    else
    {
        NEA_Phys3DBodyAddBox(scene.door, DOOR_HALF_W, DOOR_HALF_H, DOOR_HALF_D, 1.0);
        NEA_Phys3DBodySetModel(scene.door, scene.door_model);

        // A revolute joint is as lossless as a distance joint: it constrains,
        // it does not dissipate. Without damping the door swings forever and
        // the scene never sleeps, which would cost this example half of what
        // it measures. box3d_rope learned the same lesson.
        b3Body_SetLinearDamping(scene.door->id, b3fFromDouble(0.3));
        b3Body_SetAngularDamping(scene.door->id, b3fFromDouble(0.5));

        b3RevoluteJointDef hinge = b3DefaultRevoluteJointDef();
        hinge.base.bodyIdA = scene.anchor->id;
        hinge.base.bodyIdB = scene.door->id;

        // The hinge axis is the joint frame's **z**, and a door hinges about
        // vertical -- so both frames are rotated to put z along +y. There is
        // no axis parameter; this is how a hinge is aimed.
        b3Quat toY = b3MakeQuatFromAxisAngle(b3MakeVec3(b3f_one, b3f_zero, b3f_zero),
                                             (b3a)(-B3_BRAD_HALF_PI));
        hinge.base.localFrameA.q = toY;
        hinge.base.localFrameB.q = toY;

        // Frame A at the hinge line on the anchor; frame B at the same point
        // in the door's own space, which is its left edge.
        hinge.base.localFrameA.p = b3MakeVec3(b3fFromDouble(DOOR_X),
                                              b3fFromDouble(DOOR_Y), b3f_zero);
        hinge.base.localFrameB.p = b3MakeVec3(b3fFromDouble(-DOOR_HALF_W),
                                              b3f_zero, b3f_zero);

        // Swings 80 degrees each way and stops. Y toggles this at run time.
        hinge.enableLimit = true;
        hinge.lowerAngle = (b3a)(-80.0 * (32768.0 / 360.0));
        hinge.upperAngle = (b3a)(80.0 * (32768.0 / 360.0));

        scene.doorJoint = b3CreateRevoluteJoint(NEA_Phys3DWorldGetId(), &hinge);
    }

    // ---------------------------------------------------------------------
    // The turntable -- a hinge driven by a motor
    // ---------------------------------------------------------------------

    scene.table_model = NEA_ModelCreate(NEA_Static);
    NEA_ModelLoadStaticMesh(scene.table_model, cube_bin);
    NEA_ModelScaleI(scene.table_model, floattof32(TABLE_HALF),
                    floattof32(TABLE_THICK), floattof32(TABLE_HALF));

    scene.table = NEA_Phys3DBodyCreate(b3_dynamicBody, TABLE_X, TABLE_Y, 0);
    if (scene.table == NULL)
    {
        bodiesMissing++;
    }
    else
    {
        NEA_Phys3DBodyAddBox(scene.table, TABLE_HALF, TABLE_THICK, TABLE_HALF, 1.0);
        NEA_Phys3DBodySetModel(scene.table, scene.table_model);

        b3RevoluteJointDef hinge = b3DefaultRevoluteJointDef();
        hinge.base.bodyIdA = scene.anchor->id;
        hinge.base.bodyIdB = scene.table->id;

        // Spins about vertical, same aiming as the door.
        b3Quat toY = b3MakeQuatFromAxisAngle(b3MakeVec3(b3f_one, b3f_zero, b3f_zero),
                                             (b3a)(-B3_BRAD_HALF_PI));
        hinge.base.localFrameA.q = toY;
        hinge.base.localFrameB.q = toY;

        hinge.base.localFrameA.p = b3MakeVec3(b3fFromDouble(TABLE_X),
                                              b3fFromDouble(TABLE_Y), b3f_zero);

        // Motor speed is in **radians per second**, unlike the limits above,
        // which are brads -- it is an angular velocity, not an angle.
        hinge.enableMotor = true;
        hinge.motorSpeed = b3fFromDouble(1.5);
        hinge.maxMotorTorque = b3fFromDouble(400.0);

        scene.tableJoint = b3CreateRevoluteJoint(NEA_Phys3DWorldGetId(), &hinge);
    }

    // ---------------------------------------------------------------------
    // The free arm -- a plain hinge, no limit and no motor
    // ---------------------------------------------------------------------

    scene.arm_model = NEA_ModelCreate(NEA_Static);
    NEA_ModelLoadStaticMesh(scene.arm_model, cube_bin);
    NEA_ModelScaleI(scene.arm_model, floattof32(ARM_HALF_W),
                    floattof32(ARM_HALF_H), floattof32(ARM_HALF_W));

    scene.arm = NEA_Phys3DBodyCreate(b3_dynamicBody, ARM_X, ARM_PIVOT_Y - ARM_HALF_H, 0);
    if (scene.arm == NULL)
    {
        bodiesMissing++;
    }
    else
    {
        NEA_Phys3DBodyAddBox(scene.arm, ARM_HALF_W, ARM_HALF_H, ARM_HALF_W, 1.0);
        NEA_Phys3DBodySetModel(scene.arm, scene.arm_model);

        b3Body_SetAngularDamping(scene.arm->id, b3fFromDouble(0.15));

        b3RevoluteJointDef hinge = b3DefaultRevoluteJointDef();
        hinge.base.bodyIdA = scene.anchor->id;
        hinge.base.bodyIdB = scene.arm->id;

        // Swings in the screen plane, so the hinge axis is z and both frames
        // keep their default identity rotation. The simplest of the three,
        // and the one whose period is the pendulum closed form the host tests
        // check against.
        hinge.base.localFrameA.p = b3MakeVec3(b3fFromDouble(ARM_X),
                                              b3fFromDouble(ARM_PIVOT_Y), b3f_zero);
        hinge.base.localFrameB.p = b3MakeVec3(b3f_zero, b3fFromDouble(ARM_HALF_H),
                                              b3f_zero);

        scene.armJoint = b3CreateRevoluteJoint(NEA_Phys3DWorldGetId(), &hinge);
    }

    int jointsMissing = 0;
    if (b3Joint_IsValid(scene.doorJoint) == false)
        jointsMissing++;
    if (b3Joint_IsValid(scene.tableJoint) == false)
        jointsMissing++;
    if (b3Joint_IsValid(scene.armJoint) == false)
        jointsMissing++;

    int32_t buildBytes = NEA_Phys3DWorldGetMemoryUsage();

    consoleDemoInit();

    printf("NEA Box3D Hinge Demo\n");
    printf("--------------------\n");
    printf("D-pad: push the door\n");
    printf("X: motor   Y: door limit\n");
    printf("A: reset\n\n");

    uint32_t stepTicks = 0;
    uint32_t peakTicks = 0;
    int peakAwake = 0;
    uint32_t frames = 0;

    bool motorOn = true;
    bool limitOn = true;

    while (1)
    {
        scanKeys();
        uint32_t keys = keysHeld();
        uint32_t down = keysDown();

        if (scene.door != NULL)
        {
            // Pushed at the outer edge, so the force makes a torque about the
            // hinge rather than trying to drag the door off its pivot -- which
            // the point-to-point constraint would simply absorb.
            if (keys & KEY_RIGHT)
                NEA_Phys3DBodyApplyForce(scene.door, 0, 0, 60);
            if (keys & KEY_LEFT)
                NEA_Phys3DBodyApplyForce(scene.door, 0, 0, -60);
        }

        if (down & KEY_X)
        {
            motorOn = !motorOn;
            if (b3Joint_IsValid(scene.tableJoint))
                b3RevoluteJoint_EnableMotor(scene.tableJoint, motorOn);
        }

        if (down & KEY_Y)
        {
            limitOn = !limitOn;
            if (b3Joint_IsValid(scene.doorJoint))
                b3RevoluteJoint_EnableLimit(scene.doorJoint, limitOn);
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

        int awake = NEA_Phys3DWorldGetAwakeBodyCount();
        if (stepTicks > peakTicks)
        {
            peakTicks = stepTicks;
            peakAwake = awake;
        }

        uint32_t stepMicros = (stepTicks * 1000u) / 33514u;
        uint32_t peakMicros = (peakTicks * 1000u) / 33514u;

        // The door's angle, in degrees, and the torque the turntable's motor
        // is applying.
        //
        // The angle is the hinge's own error measure: with the limit on it
        // must stay inside +-80, and it is what you can check against the
        // door on screen. The torque jitters by a few percent even at rest,
        // for the reason box3d_rope's readout notes -- the accumulator sits
        // between two representable values and dithers. The angle does not.
        int doorDeg = 0;
        if (b3Joint_IsValid(scene.doorJoint))
        {
            doorDeg = (int)((double)b3RevoluteJoint_GetAngle(scene.doorJoint)
                            * (360.0 / 32768.0));
        }

        int motorTorque = 0;
        if (b3Joint_IsValid(scene.tableJoint))
        {
            motorTorque = (int)fabs(b3fToDouble(
                b3RevoluteJoint_GetMotorTorque(scene.tableJoint)));
        }

        printf("\x1b[8;0Hstep   %5lu us (%lu ticks)  ",
               (unsigned long)stepMicros, (unsigned long)stepTicks);
        printf("\x1b[9;0Hpeak   %5lu us @ %d awake   ",
               (unsigned long)peakMicros, peakAwake);
        printf("\x1b[10;0Hawake  %2d / %d bodies       ", awake, NUM_BODIES);
        printf("\x1b[11;0Hmotor %s  limit %s        ",
               motorOn ? "on " : "off", limitOn ? "on " : "off");
        printf("\x1b[12;0Hdoor  %4d deg   torque %4d ", doorDeg, motorTorque);
        printf("\x1b[13;0Hpool   %5ld / %ld bytes     ",
               (long)NEA_Phys3DWorldGetMemoryUsage(),
               (long)NEA_Phys3DWorldGetMemoryCapacity());
        printf("\x1b[14;0Hbuild  %5ld bytes           ", (long)buildBytes);
        printf("\x1b[15;0Hlate   %5d allocs, %ld ovf  ",
               NEA_Phys3DWorldGetLateAllocCount(),
               (long)NEA_Phys3DWorldGetOverflowBytes());
        printf("\x1b[16;0Hcpu    %3d %%                ", NEA_GetCPUPercent());
        printf("\x1b[17;0Hmissing %d body %d joint     ",
               bodiesMissing, jointsMissing);
        printf("\x1b[18;0Hframe  %8lu               ", (unsigned long)frames);

        NEA_WaitForVBL(NEA_CAN_SKIP_VBL);
        NEA_ProcessArg(Draw3DScene, &scene);
    }

    return 0;
}
