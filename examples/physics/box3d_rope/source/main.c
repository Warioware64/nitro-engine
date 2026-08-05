// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Warioware64, 2026
//
// This file is part of Nitro Engine Advanced
//
// Box3D distance joint example
// ----------------------------
// A rope of six links hanging from a fixed point, and a pendulum beside it,
// both built from distance joints. D-pad swings the rope's end, X turns the
// rope into a spring, Y adds a length limit, A resets, B kicks everything.
//
// This is the joint counterpart of box3d_basic. That example measures what
// *contacts* cost per step; this one measures what a constraint costs, in a
// scene where nothing touches anything -- so the step time here is the joint
// solver and the integrator, with the narrow phase doing nothing.
//
// There are no NEA_Phys3DJoint* wrappers yet, so this calls Box3D's joint API
// directly through NEA_Phys3DBody::id. That is a supported thing to do -- `id`
// is documented as the raw handle -- and it is the same way this example
// already names b3_staticBody.

#include <NEAMain.h>
#include <NEAPhysics3D.h>

#include <box3d/box3d.h>

#include <nds/arm9/exceptions.h>

#include <math.h>

#include "cube_bin.h"

// Six links plus the pendulum bob.
#define NUM_LINKS   6
#define NUM_BODIES  (NUM_LINKS + 1)

// Joints: one per link (the first ties to the anchor), plus the pendulum's.
#define NUM_JOINTS  (NUM_LINKS + 1)

#define LINK_HALF   0.18f
#define LINK_SPACE  0.75f

#define ANCHOR_X    (-1.6f)
#define ANCHOR_Y    5.0f
#define PENDULUM_X  1.8f
#define PENDULUM_LEN 3.0f

typedef struct {
    NEA_Camera *camera;

    NEA_Model *link_model[NUM_LINKS];
    NEA_Model *bob_model;
    NEA_Model *anchor_model;

    NEA_Phys3DBody *link[NUM_LINKS];
    NEA_Phys3DBody *bob;
    NEA_Phys3DBody *anchor;

    b3JointId joint[NUM_JOINTS];
    int jointCount;
} SceneData;

void Draw3DScene(void *arg)
{
    SceneData *scene = arg;

    NEA_CameraUse(scene->camera);

    NEA_PolyFormat(31, 0, NEA_LIGHT_0 | NEA_LIGHT_1, NEA_CULL_BACK, 0);

    for (int i = 0; i < NUM_LINKS; i++)
        NEA_ModelDraw(scene->link_model[i]);

    NEA_ModelDraw(scene->bob_model);

    NEA_PolyFormat(20, 0, NEA_LIGHT_0 | NEA_LIGHT_1, NEA_CULL_BACK, 0);
    NEA_ModelDraw(scene->anchor_model);
}

static void LinkStartPosition(int i, float *x, float *y, float *z)
{
    *x = ANCHOR_X;
    *y = ANCHOR_Y - LINK_SPACE * (float)(i + 1);
    *z = 0.0f;
}

static void ResetScene(SceneData *scene)
{
    for (int i = 0; i < NUM_LINKS; i++)
    {
        if (scene->link[i] == NULL)
            continue;

        float x, y, z;
        LinkStartPosition(i, &x, &y, &z);

        NEA_Phys3DBodySetPosition(scene->link[i], x, y, z);
        NEA_Phys3DBodySetVelocity(scene->link[i], 0, 0, 0);
        NEA_Phys3DBodySetAwake(scene->link[i], true);
    }

    if (scene->bob != NULL)
    {
        NEA_Phys3DBodySetPosition(scene->bob, PENDULUM_X + PENDULUM_LEN,
                                  ANCHOR_Y, 0);
        NEA_Phys3DBodySetVelocity(scene->bob, 0, 0, 0);
        NEA_Phys3DBodySetAwake(scene->bob, true);
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

    // Nothing in this scene is meant to touch anything, but the links can
    // swing into each other when the rope is kicked -- adjacent ones cannot,
    // since a joint suppresses its own pair, but the ends can meet in the
    // middle.
    def.box3d.capacity.contactCount = NUM_BODIES * 2;

    // The one capacity a jointed scene needs and a jointless one does not.
    //
    // It defaults to zero, deliberately: a joint sim is reserved in *four*
    // places, because it migrates between the constraint graph, the awake set,
    // a sleeping set and the disabled set as its bodies change state, so a
    // floor would charge every jointless world four arrays it never uses.
    // Leaving it unset here would not break anything -- joints are created
    // while the scene is built, so the arrays would grow as build memory --
    // but it would show up in the `build` figure below rather than being
    // planned for.
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

    scene.camera = NEA_CameraCreate();
    // Framed to hold the rope (left, hanging to y ~= 0.5) and the pendulum's
    // full sweep (right, reaching x = PENDULUM_X + PENDULUM_LEN) at once. A
    // demo whose measurement you can read but whose motion is half off-screen
    // is not showing you the thing it measured.
    NEA_CameraSet(scene.camera,
                  0.5, 2.5, 15,
                  0.5, 2.0, 0,
                  0, 1, 0);

    NEA_LightSet(0, NEA_White, 0, -1, -1);
    NEA_LightSet(1, NEA_Blue, -1, 0, 0);

    NEA_ClearColorSet(RGB15(4, 4, 8), 31, 63);

    // ---------------------------------------------------------------------
    // The anchor
    // ---------------------------------------------------------------------
    //
    // One static body, carrying both the rope's top joint and the pendulum's.
    // It needs no shape: a joint attaches to a *body*, and a body with no shape
    // has no mass and no proxy, which is exactly what an anchor should be.

    scene.anchor = NEA_Phys3DBodyCreate(b3_staticBody, 0, ANCHOR_Y, 0);

    scene.anchor_model = NEA_ModelCreate(NEA_Static);
    NEA_ModelLoadStaticMesh(scene.anchor_model, cube_bin);
    NEA_ModelSetCoord(scene.anchor_model, 0, ANCHOR_Y, 0);
    NEA_ModelScaleI(scene.anchor_model, floattof32(2.2f), floattof32(0.12f),
                    floattof32(0.12f));

    // ---------------------------------------------------------------------
    // The rope
    // ---------------------------------------------------------------------

    int bodiesMissing = 0;
    int jointsMissing = 0;

    NEA_Phys3DBody *previous = scene.anchor;

    for (int i = 0; i < NUM_LINKS; i++)
    {
        scene.link_model[i] = NEA_ModelCreate(NEA_Static);
        NEA_ModelLoadStaticMesh(scene.link_model[i], cube_bin);
        NEA_ModelScaleI(scene.link_model[i], floattof32(LINK_HALF),
                        floattof32(LINK_HALF), floattof32(LINK_HALF));

        float x, y, z;
        LinkStartPosition(i, &x, &y, &z);

        scene.link[i] = NEA_Phys3DBodyCreate(b3_dynamicBody, x, y, z);
        if (scene.link[i] == NULL)
        {
            bodiesMissing++;
            continue;
        }

        NEA_Phys3DBodyAddSphere(scene.link[i], LINK_HALF, 1.0);
        NEA_Phys3DBodySetModel(scene.link[i], scene.link_model[i]);

        // Damping, because a distance joint is lossless.
        //
        // It constrains a length and nothing else: no friction, no restitution,
        // no angular constraint. A chain of them swings forever, and a scene
        // that never comes to rest never sleeps -- so without this the example
        // could not show what a settled rope costs, which is half of what it
        // is here to measure. The angular term is the larger one because the
        // links are spheres and nothing else resists them spinning.
        b3Body_SetLinearDamping(scene.link[i]->id, b3fFromDouble(0.4));
        b3Body_SetAngularDamping(scene.link[i]->id, b3fFromDouble(0.8));

        // The joint tying this link to the one above it.
        //
        // The frames are measured from each body's *origin*, not its centre of
        // mass -- which for these is the same point, but would not be for a
        // body whose shape is offset, and is why b3JointDef is specified that
        // way. Both frames are at the origin, so the rope hangs from centre to
        // centre and the rest length is the spacing.
        b3DistanceJointDef jointDef = b3DefaultDistanceJointDef();
        jointDef.base.bodyIdA = previous->id;
        jointDef.base.bodyIdB = scene.link[i]->id;

        // The top joint hangs from a *point on the bar*, not from the bar's
        // origin. The anchor body sits at x = 0 and the rope hangs at
        // x = ANCHOR_X, so without this offset the first joint measures the
        // 1.77 m diagonal from the bar's centre to the first link and hauls
        // the whole rope sideways to make it 0.75 -- which is what the first
        // build of this example did, and it looked like a solver bug rather
        // than a scene one. Offsetting the frame is what b3JointDef's local
        // frames are for, and it is cheaper than a second static body.
        if (i == 0)
        {
            jointDef.base.localFrameA.p = b3MakeVec3(b3fFromDouble(ANCHOR_X),
                                                     b3f_zero, b3f_zero);
        }

        jointDef.length = b3fFromDouble(LINK_SPACE);

        // Rigid by default. X turns the spring on at run time.
        jointDef.maxLength = jointDef.length;

        scene.joint[scene.jointCount++] = b3CreateDistanceJoint(NEA_Phys3DWorldGetId(),
                                                                &jointDef);

        previous = scene.link[i];
    }

    // ---------------------------------------------------------------------
    // The pendulum
    // ---------------------------------------------------------------------
    //
    // One link, released from the horizontal. A pendulum is the clearest thing
    // to look at when checking a rigid distance constraint by eye: the bob
    // must sweep an exact arc, and a constraint that stretches shows up as the
    // arc bulging outward at the bottom, where the load is highest.

    scene.bob_model = NEA_ModelCreate(NEA_Static);
    NEA_ModelLoadStaticMesh(scene.bob_model, cube_bin);
    NEA_ModelScaleI(scene.bob_model, floattof32(0.3f), floattof32(0.3f),
                    floattof32(0.3f));

    scene.bob = NEA_Phys3DBodyCreate(b3_dynamicBody, PENDULUM_X + PENDULUM_LEN,
                                     ANCHOR_Y, 0);
    if (scene.bob == NULL)
    {
        bodiesMissing++;
    }
    else
    {
        NEA_Phys3DBodyAddSphere(scene.bob, 0.3f, 1.0);
        NEA_Phys3DBodySetModel(scene.bob, scene.bob_model);

        // Lighter than the rope's, so the pendulum keeps swinging visibly for
        // a while after the rope has settled -- the two are showing different
        // things and want different amounts of loss.
        b3Body_SetLinearDamping(scene.bob->id, b3fFromDouble(0.1));
        b3Body_SetAngularDamping(scene.bob->id, b3fFromDouble(0.5));

        b3DistanceJointDef jointDef = b3DefaultDistanceJointDef();
        jointDef.base.bodyIdA = scene.anchor->id;
        jointDef.base.bodyIdB = scene.bob->id;

        // The anchor body sits at x = 0, so the pendulum's pivot is placed by
        // offsetting the frame on the anchor rather than by adding a second
        // static body for it.
        jointDef.base.localFrameA.p = b3MakeVec3(b3fFromDouble(PENDULUM_X),
                                                 b3f_zero, b3f_zero);
        jointDef.length = b3fFromDouble(PENDULUM_LEN);
        jointDef.maxLength = jointDef.length;

        scene.joint[scene.jointCount++] = b3CreateDistanceJoint(NEA_Phys3DWorldGetId(),
                                                                &jointDef);
    }

    for (int i = 0; i < scene.jointCount; i++)
    {
        if (b3Joint_IsValid(scene.joint[i]) == false)
            jointsMissing++;
    }

    int32_t buildBytes = NEA_Phys3DWorldGetMemoryUsage();

    consoleDemoInit();

    printf("NEA Box3D Rope Demo\n");
    printf("-------------------\n");
    printf("D-pad: swing rope end\n");
    printf("X: spring  Y: limit\n");
    printf("A: reset   B: kick\n\n");

    uint32_t stepTicks = 0;
    uint32_t peakTicks = 0;
    int peakAwake = 0;
    uint32_t frames = 0;

    bool springOn = false;
    bool limitOn = false;

    while (1)
    {
        scanKeys();
        uint32_t keys = keysHeld();
        uint32_t down = keysDown();

        NEA_Phys3DBody *tip = scene.link[NUM_LINKS - 1];

        if (tip != NULL)
        {
            if (keys & KEY_RIGHT)
                NEA_Phys3DBodyApplyForce(tip, 25, 0, 0);
            if (keys & KEY_LEFT)
                NEA_Phys3DBodyApplyForce(tip, -25, 0, 0);
            if (keys & KEY_UP)
                NEA_Phys3DBodyApplyForce(tip, 0, 0, -25);
            if (keys & KEY_DOWN)
                NEA_Phys3DBodyApplyForce(tip, 0, 0, 25);
        }

        // A rope that can stretch and a rope that cannot are the same code
        // path with one flag, which is the point of showing it here: the
        // spring, the limit and the rigid constraint are three configurations
        // of one joint, not three joint types.
        if (down & KEY_X)
        {
            springOn = !springOn;
            for (int i = 0; i < NUM_JOINTS - 1 && i < scene.jointCount; i++)
            {
                b3DistanceJoint_EnableSpring(scene.joint[i], springOn);
                b3DistanceJoint_SetSpringHertz(scene.joint[i], b3fFromDouble(3.0));
                b3DistanceJoint_SetSpringDampingRatio(scene.joint[i],
                                                      b3fFromDouble(0.4));
            }
        }

        if (down & KEY_Y)
        {
            limitOn = !limitOn;
            for (int i = 0; i < NUM_JOINTS - 1 && i < scene.jointCount; i++)
            {
                b3DistanceJoint_EnableLimit(scene.joint[i], limitOn);
                b3DistanceJoint_SetLengthRange(scene.joint[i],
                                               b3fFromDouble(0.4 * LINK_SPACE),
                                               b3fFromDouble(1.3 * LINK_SPACE));
            }
        }

        if (down & KEY_B)
        {
            for (int i = 0; i < NUM_LINKS; i++)
            {
                if (scene.link[i] != NULL)
                    NEA_Phys3DBodyApplyImpulse(scene.link[i], 3, 2, 0);
            }
        }

        if (down & KEY_A)
            ResetScene(&scene);

        // Stepped by hand so it can be timed, exactly as box3d_basic does --
        // hence NEA_CAN_SKIP_VBL below rather than NEA_UPDATE_PHYS3D, which
        // would step the world a second time.
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

        // What the top joint is holding, and how hard.
        //
        // The length is the constraint's own error measure: rigid, it should
        // read the rest length whatever the rope is doing. The force is the
        // rope's tension, and at rest it is the weight of everything below
        // that joint -- the closed form the host tests check against.
        //
        // The tension jitters by a few percent at rest even after the rope has
        // stopped moving. That is not a bug to chase: the equilibrium impulse
        // falls between two representable Q15.16 values and the accumulator
        // dithers between them. The length does not jitter, and the length is
        // what you see.
        double topLength = 0.0;
        double topForce = 0.0;
        if (scene.jointCount > 0 && b3Joint_IsValid(scene.joint[0]))
        {
            topLength = b3fToDouble(b3DistanceJoint_GetCurrentLength(scene.joint[0]));

            b3Vec3 f = b3Joint_GetConstraintForce(scene.joint[0]);
            double fx = b3fToDouble(f.x);
            double fy = b3fToDouble(f.y);
            double fz = b3fToDouble(f.z);
            topForce = sqrt(fx * fx + fy * fy + fz * fz);
        }

        printf("\x1b[8;0Hstep   %5lu us (%lu ticks)  ",
               (unsigned long)stepMicros, (unsigned long)stepTicks);
        printf("\x1b[9;0Hpeak   %5lu us @ %d awake   ",
               (unsigned long)peakMicros, peakAwake);
        printf("\x1b[10;0Hawake  %2d / %d bodies       ", awake, NUM_BODIES);
        printf("\x1b[11;0Hjoints %2d  spring %s limit %s",
               scene.jointCount, springOn ? "on " : "off", limitOn ? "on " : "off");
        printf("\x1b[12;0Hlen    %6d /1000  N %6d ",
               (int)(topLength * 1000.0), (int)topForce);
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
