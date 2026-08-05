// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Warioware64, 2026
//
// This file is part of Nitro Engine Advanced
//
// Box3D wheel joint example -- a car on terrain
// ---------------------------------------------
// A **wheel joint** is a prismatic and a revolute in one: a sprung suspension
// travelling along one axis, a spin motor about another, and a steering twist
// between them. Four of them here, carrying a chassis over the box3d_level
// terrain, which is the first example in the port to use two joint types at
// once and the only one that runs the wheel joint at frame rate with real
// contact underneath.
//
// **This example is not decoration.** The spherical joint's degenerate-axis
// disaster was found by *its* device example rather than by the host suite,
// because a host scene only tests the configurations somebody thought to write
// down. The wheel joint has two degeneracies of its own -- the steering axis
// collapses as the wheel tips onto its own suspension axis, and the
// collinearity mass runs into b3RcpW's floor at the same place -- and a wheel
// on a ramp reaches both without being asked. That is what the terrain is for.
//
// Four things are on screen that no earlier example could show:
//
//   - **Two joint types cooperating.** The mast is held to the chassis by a
//     spherical joint and kept aligned by a **parallel joint**, which is the
//     only joint that constrains orientation alone. On flat ground it does
//     nothing visible; over the ramps it visibly fights the mast's inertia,
//     which is the entire reason that joint exists.
//   - **Steering in brads, with no conversion.** b3WheelJoint_SetTargetSteeringAngle
//     takes a b3a, and a D-pad reading maps straight into it. This is the one
//     place the port's choice of angle unit is an outright advantage over
//     upstream's radians.
//   - **Joint events, consumed.** A force threshold on each suspension counts
//     landings that overload it. Those two threshold fields existed from Stage
//     1, were copied by b3CreateJoint, had four accessors, and were read by
//     nothing until Stage 7 -- this is the first code anywhere that reads one.
//   - **The suspension is a spring, not a strut.** Y switches it stiff/soft and
//     the ride height and the overload count both change with it.
//
// Which axis is which, because a wheel joint has three and they are not
// parameters. The suspension travels along **frame A's x**, the wheel spins
// about **frame B's z**, and steering is the twist between them about A's x.
// Both frames are built with VerticalFrame(), a quarter turn about z, which
// takes A's x onto world +y -- so the suspension stands up, the axles lie along
// world z, and the car drives along world x. Steering is then a twist about
// vertical, which is what steering is.
//
// As in every other physics example there are no NEA_Phys3DJoint* wrappers yet,
// so this calls Box3D's joint API directly through NEA_Phys3DBody::id.

#include <NEAMain.h>
#include <NEAPhysics3D.h>

#include <box3d/box3d.h>

#include <nds/arm9/exceptions.h>

#include <math.h>

#include "cube_bin.h"
#include "level_bin.h"
#include "sphere_bin.h"

#include "level_b3mesh.h"

#define NUM_WHEELS 4

// Chassis, mast and four wheels.
#define NUM_BODIES ( NUM_WHEELS + 2 )

// Four wheel joints, plus the mast's spherical and parallel pair.
#define NUM_JOINTS ( NUM_WHEELS + 2 )

// The display list was authored at half size to fit v16's 8-unit limit, so the
// model is drawn at 2x to line up with the collision mesh, which is at true
// scale. assets.sh explains where the two scales come from.
#define LEVEL_DRAW_SCALE 2.0f

#define CHASSIS_HALF_X 0.80f
#define CHASSIS_HALF_Y 0.20f
#define CHASSIS_HALF_Z 0.45f

#define WHEEL_HALF 0.28f

// Where the wheels hang off the chassis, in chassis-local space. These are the
// wheel joints' localFrameA.p, so the suspension translation reads as zero when
// a wheel sits exactly here -- which is what makes the travel readout legible.
#define WHEEL_OFFSET_X 0.62f
#define WHEEL_OFFSET_Y ( -0.30f )
#define WHEEL_OFFSET_Z 0.52f

// Where the car starts. High enough over the level's flat end that it drops
// onto its suspension rather than starting inside the terrain -- the first
// landing is also the first thing that exercises the overload counter.
#define START_X ( -3.0f )
#define START_Y 2.6f
#define START_Z 0.0f

// The suspension's two settings, and the travel its limits allow.
//
// Soft is the interesting one: it sags visibly under the chassis' own weight,
// bottoms out on the ramps, and is what makes the overload counter move.
#define SUSPENSION_STIFF 4.0f
#define SUSPENSION_SOFT 1.6f
#define SUSPENSION_DAMPING 0.7f

// The same two values as rationals, for the Y-key path. That one selects
// between them at run time, which is the one context where a float literal
// cannot be folded away -- see the comment at the key handler.
#define SUSPENSION_HZ_DEN 10
#define SUSPENSION_STIFF_NUM 40
#define SUSPENSION_SOFT_NUM 16

#define SUSPENSION_LOWER ( -0.28f )
#define SUSPENSION_UPPER 0.10f

// The drive. The spin motor's torque budget is a ceiling on authority, not a
// controller: a car that cannot lose traction is a car on rails.
//
// Stated as a rational rather than a float literal because the throttle is on
// the per-frame path: b3fFromFrac folds to a constant at compile time, where
// b3fFromDouble on a runtime value cannot, and a DS has no FPU to fall back on.
#define DRIVE_SPEED_NUM 14
#define DRIVE_SPEED_DEN 1
#define MAX_SPIN_TORQUE 3.0f

// The throttle ramp, as a Q30 coefficient. 0.06 per frame, exactly the rate the
// float version used, expressed the way b3MulFC wants it.
#define THROTTLE_RAMP b3cFromFrac( 6, 100 )

// Steering, in **brads** -- 32768 to the turn, so 2048 is 22.5 degrees. The
// D-pad maps straight into b3WheelJoint_SetTargetSteeringAngle with no
// conversion at all, which is the point being made.
#define STEER_MAX 2600
#define STEER_RATE 160

#define MAX_STEERING_TORQUE 40.0f
#define STEERING_HERTZ 8.0f
#define STEERING_DAMPING 0.9f

// The mast: a tall thin body on a ball joint at the chassis roof, kept aligned
// by the parallel joint. Tall so that a small angular error is a large visible
// one at the tip.
#define MAST_HALF_X 0.06f
#define MAST_HALF_Y 0.55f
#define MAST_HALF_Z 0.06f
#define MAST_BASE_Y 0.20f

#define MAST_HERTZ 3.0f
#define MAST_DAMPING 0.5f
#define MAST_MAX_TORQUE 8.0f

// The suspension force that counts as an overload.
//
// The whole car is about 1.9 kg, so a wheel carries roughly 4.7 N standing
// still. 30 N is comfortably above anything the car reaches rolling and is
// reached by a landing -- which is exactly the event worth counting. Set this
// near the static load instead and every wheel reports every step, which the
// threshold's `>=` comparison permits on purpose.
#define OVERLOAD_N 30.0f

// Below this a body counts as lost. The level is open on its ramp side by
// design, so this is reachable in ordinary play.
#define RESPAWN_BELOW ( -20.0f )

typedef struct
{
    NEA_Camera *camera;

    NEA_Model *chassis_model;
    NEA_Model *mast_model;
    NEA_Model *wheel_model[NUM_WHEELS];
    NEA_Model *level_model;

    NEA_Phys3DBody *chassis;
    NEA_Phys3DBody *mast;
    NEA_Phys3DBody *wheel[NUM_WHEELS];
    NEA_Phys3DBody *level_body;

    b3JointId wheel_joint[NUM_WHEELS];
    b3JointId mast_ball;
    b3JointId mast_parallel;
} SceneData;

// The joint frame shared by all four wheel joints.
//
// A quarter turn about z takes frame x onto world +y. Applied to *both* joint
// frames it stands the suspension up and leaves the spin axis -- frame B's z --
// pointing along world z, which is where a car's axles go. There is no axis
// parameter to set instead: the frames are the axes, as they are for every
// other joint in the library.
static b3Quat VerticalFrame(void)
{
    return b3MakeQuatFromAxisAngle(b3Vec3_axisZ, B3_BRAD_HALF_PI);
}

// The joint frame that makes a parallel joint hold the *mast* upright.
//
// **A parallel joint constrains frame z and leaves the twist about it free**,
// which is the whole shape of the joint -- and it means the frame decides which
// axis is held. With identity frames the held axis is the bodies' own z, and the
// mast, which is long along y, was left free in precisely the direction it
// needed holding: it fell flat in the x-y plane on the first ramp while the
// joint reported it was doing its job.
//
// A quarter turn about x takes frame z onto +y, so what is held is the mast's
// length and what stays free is its spin about that length. That is the right
// pair for a mast, and it is the same "the frames *are* the axes" rule the wheel
// joints follow above.
static b3Quat MastFrame(void)
{
    return b3MakeQuatFromAxisAngle(b3Vec3_axisX, (b3a)(-B3_BRAD_HALF_PI));
}

// Front wheels are index 0 and 1, so `index < 2` is "steered".
//
// Handed back as f32 rather than float. Every consumer wants Q12 in the end --
// NEA_Phys3DBodySetPositionI takes it, and a b3f *is* an f32, which is the
// property b3fixed.h picks Q19.12 for -- so producing float here only bought a
// pair of soft-float conversions on the way back out.
static void WheelOffset(int index, int32_t *ox, int32_t *oy, int32_t *oz)
{
    *ox = ( index < 2 ) ? floattof32(WHEEL_OFFSET_X) : -floattof32(WHEEL_OFFSET_X);
    *oy = floattof32(WHEEL_OFFSET_Y);
    *oz = ( index % 2 == 0 ) ? floattof32(WHEEL_OFFSET_Z) : -floattof32(WHEEL_OFFSET_Z);
}

void Draw3DScene(void *arg)
{
    SceneData *scene = arg;

    NEA_CameraUse(scene->camera);

    NEA_PolyFormat(31, 0, NEA_LIGHT_0 | NEA_LIGHT_1, NEA_CULL_BACK, 0);
    NEA_ModelDraw(scene->chassis_model);

    NEA_PolyFormat(29, 0, NEA_LIGHT_0 | NEA_LIGHT_1, NEA_CULL_BACK, 0);
    NEA_ModelDraw(scene->mast_model);

    NEA_PolyFormat(26, 0, NEA_LIGHT_0 | NEA_LIGHT_1, NEA_CULL_BACK, 0);
    for (int i = 0; i < NUM_WHEELS; i++)
    {
        if (scene->wheel_model[i] != NULL)
            NEA_ModelDraw(scene->wheel_model[i]);
    }

    NEA_PolyFormat(20, 0, NEA_LIGHT_0 | NEA_LIGHT_1, NEA_CULL_BACK, 0);
    NEA_ModelDraw(scene->level_model);
}

static void ResetScene(SceneData *scene)
{
    if (scene->chassis != NULL)
    {
        NEA_Phys3DBodySetPosition(scene->chassis, START_X, START_Y, START_Z);
        NEA_Phys3DBodySetVelocity(scene->chassis, 0, 0, 0);
        NEA_Phys3DBodySetAwake(scene->chassis, true);
    }

    if (scene->mast != NULL)
    {
        NEA_Phys3DBodySetPosition(scene->mast, START_X,
                                  START_Y + MAST_BASE_Y + MAST_HALF_Y, START_Z);
        NEA_Phys3DBodySetVelocity(scene->mast, 0, 0, 0);
        NEA_Phys3DBodySetAwake(scene->mast, true);
    }

    for (int i = 0; i < NUM_WHEELS; i++)
    {
        if (scene->wheel[i] == NULL)
            continue;

        int32_t ox, oy, oz;
        WheelOffset(i, &ox, &oy, &oz);

        NEA_Phys3DBodySetPositionI(scene->wheel[i], floattof32(START_X) + ox,
                                   floattof32(START_Y) + oy,
                                   floattof32(START_Z) + oz);
        NEA_Phys3DBodySetVelocity(scene->wheel[i], 0, 0, 0);
        NEA_Phys3DBodySetAwake(scene->wheel[i], true);
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

    // Just the level.
    def.box3d.capacity.staticBodyCount = 1;
    def.box3d.capacity.staticShapeCount = 1;

    // Sixteen per body, which is box3d_slider's corrected figure rather than
    // box3d_motor's eight.
    //
    // The slider's lesson was that cargo spreading over a large floor is more
    // contact-rich than any crate scene, and that eight drove `late` to 27
    // before the arena settled. A car on terrain is more contact-rich still:
    // four wheels each touching several mesh triangles at once, plus the
    // chassis whenever it grounds out on a ramp edge. Starting at sixteen and
    // correcting from the measured `late` is the rule here -- a reservation
    // that has to be fixed at runtime is a reservation that was wrong.
    //
    // **Measured: `late` settles at 1 and stays there**, across runs of several
    // thousand frames with the car driving, reversing and steering to full lock,
    // with 0 overflow bytes. One is not zero and is reported rather than
    // rounded down -- but it is a single growth on the first busy frame, not the
    // climbing count that sent box3d_slider from eight to sixteen.
    def.box3d.capacity.contactCount = NUM_BODIES * 16;

    // The one capacity a mesh scene needs and a convex one does not. Every
    // dynamic body can be against the terrain at once, and a wheel usually is.
    //
    // NUM_BODIES exactly, and doubling it was tried and reverted: it cost 41 KB
    // of pool -- 252 KB of the 262 KB budget, 96% full -- and left the one late
    // allocation below exactly where it was. So that allocation does not come
    // from here, and paying 41 KB to find that out is not a trade worth keeping.
    def.box3d.capacity.meshContactCount = NUM_BODIES;

    // A wheel joint's sim is 196 bytes, inside the spherical joint's 260, so
    // b3JointSim is still 444 and Stage 7 cost these arrays nothing. Each sim
    // is reserved four times over regardless, because it migrates between the
    // constraint graph, the awake set, a sleeping set and the disabled set.
    def.box3d.capacity.jointCount = NUM_JOINTS;

    // 256 KB rather than the 192 that box3d_level and box3d_slider use: this
    // scene has both their demands at once -- a baked mesh and its contact
    // machinery, and six joints across six bodies.
    def.poolBytes = 256 * 1024;

    if (NEA_Phys3DWorldInit(&def) != 0)
    {
        consoleDemoInit();
        printf("Could not create the physics world.\n");
        while (1)
            swiWaitForVBlank();
    }

    NEA_Phys3DWorldSetGravity(0, -9.8, 0);

    NEA_CameraSet(scene.camera = NEA_CameraCreate(),
                  -8.0, 5.0, 8.0,
                  0.0, 1.0, 0.0,
                  0, 1, 0);

    NEA_LightSet(0, NEA_White, 0, -1, -1);
    NEA_LightSet(1, NEA_Blue, -1, 0, 0);

    NEA_ClearColorSet(RGB15(4, 6, 8), 31, 63);

    int shapesMissing = 0;
    int bodiesMissing = 0;
    int jointsMissing = 0;

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
    // outlive every shape referencing it. Here it is a const array in ROM.
    if (NEA_Phys3DBodyAddMesh(scene.level_body, level_b3mesh, 1, 1, 1) != 0)
    {
        consoleDemoInit();
        printf("The level mesh was refused.\n");
        printf("Rebuild it: ./assets.sh\n");
        while (1)
            swiWaitForVBlank();
    }

    NEA_Phys3DBodySetMaterial(scene.level_body, 0.9, 0.0);

    // ---------------------------------------------------------------------
    // The chassis
    // ---------------------------------------------------------------------

    scene.chassis_model = NEA_ModelCreate(NEA_Static);
    NEA_ModelLoadStaticMesh(scene.chassis_model, cube_bin);
    NEA_ModelScaleI(scene.chassis_model, floattof32(CHASSIS_HALF_X),
                    floattof32(CHASSIS_HALF_Y), floattof32(CHASSIS_HALF_Z));

    scene.chassis = NEA_Phys3DBodyCreate(b3_dynamicBody, START_X, START_Y, START_Z);
    if (scene.chassis == NULL)
    {
        bodiesMissing++;
    }
    else
    {
        if (NEA_Phys3DBodyAddBox(scene.chassis, CHASSIS_HALF_X, CHASSIS_HALF_Y,
                                 CHASSIS_HALF_Z, 1.0) != 0)
            shapesMissing++;

        NEA_Phys3DBodySetMaterial(scene.chassis, 0.4, 0.0);
        NEA_Phys3DBodySetModel(scene.chassis, scene.chassis_model);
    }

    // ---------------------------------------------------------------------
    // The four wheels and their joints
    // ---------------------------------------------------------------------

    for (int i = 0; i < NUM_WHEELS; i++)
    {
        int32_t ox, oy, oz;
        WheelOffset(i, &ox, &oy, &oz);

        scene.wheel_model[i] = NEA_ModelCreate(NEA_Static);
        NEA_ModelLoadStaticMesh(scene.wheel_model[i], sphere_bin);
        NEA_ModelScaleI(scene.wheel_model[i], floattof32(WHEEL_HALF),
                        floattof32(WHEEL_HALF), floattof32(WHEEL_HALF));

        scene.wheel[i] = NEA_Phys3DBodyCreateI(b3_dynamicBody,
                                               floattof32(START_X) + ox,
                                               floattof32(START_Y) + oy,
                                               floattof32(START_Z) + oz);
        if (scene.wheel[i] == NULL)
        {
            bodiesMissing++;
            continue;
        }

        // **A sphere, and the first version of this example got it wrong.**
        //
        // The wheel joint constrains the axle, so the shape's only job is to
        // roll. A box was tried first, thin along x -- and the axle is frame B's
        // z, so the slab was being spun about an axis lying in its own face. It
        // ground rather than rolled: the car sat still with the spin motor
        // reporting 0.014 N-m of a 3 N-m budget, which is what "already at the
        // commanded speed, doing no work" looks like.
        //
        // A sphere has no such orientation to get wrong, and no corners to
        // judder between either -- a box wheel's effective radius swings between
        // its half-extent and its diagonal every quarter turn, which is a
        // property of the mesh rather than of the joint being demonstrated.
        if (NEA_Phys3DBodyAddSphere(scene.wheel[i], WHEEL_HALF, 1.0) != 0)
            shapesMissing++;

        // High friction and no bounce: a wheel that skates is a wheel whose
        // spin motor does nothing visible.
        NEA_Phys3DBodySetMaterial(scene.wheel[i], 1.2, 0.0);
        NEA_Phys3DBodySetModel(scene.wheel[i], scene.wheel_model[i]);

        if (scene.chassis == NULL)
        {
            jointsMissing++;
            continue;
        }

        b3WheelJointDef wdef = b3DefaultWheelJointDef();
        wdef.base.bodyIdA = scene.chassis->id;
        wdef.base.bodyIdB = scene.wheel[i]->id;

        // Frame A sits where the wheel hangs from the chassis; frame B is at
        // the wheel's own origin. With the wheel starting exactly at the mount
        // the suspension translation begins at zero, so the travel readout is
        // displacement from rest with no conversion.
        // b3Makeb3f rather than b3fFromDouble: ox/oy/oz are already f32, and a
        // b3f is an f32, so this is a relabel and not a conversion.
        wdef.base.localFrameA.p = b3MakeVec3(b3Makeb3f(ox), b3Makeb3f(oy),
                                             b3Makeb3f(oz));
        wdef.base.localFrameA.q = VerticalFrame();
        wdef.base.localFrameB.q = VerticalFrame();

        // The chassis and its own wheels must not collide -- they overlap by
        // construction at full droop.
        wdef.base.collideConnected = false;

        wdef.enableSuspensionSpring = true;
        wdef.suspensionHertz = b3fFromDouble(SUSPENSION_STIFF);
        wdef.suspensionDampingRatio = b3fFromDouble(SUSPENSION_DAMPING);

        wdef.enableSuspensionLimit = true;
        wdef.lowerSuspensionLimit = b3fFromDouble(SUSPENSION_LOWER);
        wdef.upperSuspensionLimit = b3fFromDouble(SUSPENSION_UPPER);

        // Every wheel drives. Rear-wheel drive is one line away -- gate this on
        // `i >= 2` -- but four driven wheels climb the level's ramps, which is
        // where the tipped-wheel degeneracies live.
        wdef.enableSpinMotor = true;
        wdef.maxSpinTorque = b3fFromDouble(MAX_SPIN_TORQUE);
        wdef.spinSpeed = b3f_zero;

        // Steering on all four, but only the front two are ever commanded away
        // from zero. The rear two are held straight *by the same spring*, which
        // is what keeps them from castoring.
        wdef.enableSteering = true;
        wdef.steeringHertz = b3fFromDouble(STEERING_HERTZ);
        wdef.steeringDampingRatio = b3fFromDouble(STEERING_DAMPING);
        wdef.maxSteeringTorque = b3fFromDouble(MAX_STEERING_TORQUE);
        wdef.targetSteeringAngle = 0;

        wdef.enableSteeringLimit = true;
        wdef.lowerSteeringLimit = -STEER_MAX;
        wdef.upperSteeringLimit = STEER_MAX;

        // **The first reader of a threshold field anywhere.** Set here, tested
        // by the solver every sub-step, and drained into b3World_GetJointEvents
        // below. Both fields have existed since Stage 1 and nothing read either
        // until Stage 7.
        wdef.base.forceThreshold = b3fFromDouble(OVERLOAD_N);

        scene.wheel_joint[i] = b3CreateWheelJoint(NEA_Phys3DWorldGetId(), &wdef);
        if (b3Joint_IsValid(scene.wheel_joint[i]) == false)
            jointsMissing++;
    }

    // ---------------------------------------------------------------------
    // The mast, and the parallel joint that keeps it aligned
    // ---------------------------------------------------------------------

    scene.mast_model = NEA_ModelCreate(NEA_Static);
    NEA_ModelLoadStaticMesh(scene.mast_model, cube_bin);
    NEA_ModelScaleI(scene.mast_model, floattof32(MAST_HALF_X),
                    floattof32(MAST_HALF_Y), floattof32(MAST_HALF_Z));

    scene.mast = NEA_Phys3DBodyCreate(b3_dynamicBody, START_X,
                                      START_Y + MAST_BASE_Y + MAST_HALF_Y, START_Z);
    if (scene.mast == NULL)
    {
        bodiesMissing++;
    }
    else
    {
        if (NEA_Phys3DBodyAddBox(scene.mast, MAST_HALF_X, MAST_HALF_Y,
                                 MAST_HALF_Z, 1.0) != 0)
            shapesMissing++;

        NEA_Phys3DBodySetModel(scene.mast, scene.mast_model);
    }

    if (scene.chassis != NULL && scene.mast != NULL)
    {
        // The ball joint carries the mast's *position*: it pins the mast's foot
        // to a point on the chassis roof and leaves all three rotations free.
        b3SphericalJointDef sdef = b3DefaultSphericalJointDef();
        sdef.base.bodyIdA = scene.chassis->id;
        sdef.base.bodyIdB = scene.mast->id;
        sdef.base.localFrameA.p = b3MakeVec3(b3f_zero, b3fFromDouble(MAST_BASE_Y),
                                             b3f_zero);
        sdef.base.localFrameB.p = b3MakeVec3(b3f_zero, b3fFromDouble(-MAST_HALF_Y),
                                             b3f_zero);
        sdef.base.collideConnected = false;

        scene.mast_ball = b3CreateSphericalJoint(NEA_Phys3DWorldGetId(), &sdef);
        if (b3Joint_IsValid(scene.mast_ball) == false)
            jointsMissing++;

        // And the parallel joint carries its *orientation*, which is the only
        // thing this joint type does. Without it the mast is a pendulum on a
        // ball joint and flops over on the first ramp; with it the mast tracks
        // the chassis and you watch the joint's torque budget lose and recover
        // on every bump. Two joints on one pair of bodies, each constraining
        // what the other does not -- which is why the port has both.
        b3ParallelJointDef pdef = b3DefaultParallelJointDef();
        pdef.base.bodyIdA = scene.chassis->id;
        pdef.base.bodyIdB = scene.mast->id;
        pdef.base.collideConnected = false;
        pdef.base.localFrameA.q = MastFrame();
        pdef.base.localFrameB.q = MastFrame();
        pdef.hertz = b3fFromDouble(MAST_HERTZ);
        pdef.dampingRatio = b3fFromDouble(MAST_DAMPING);
        pdef.maxTorque = b3fFromDouble(MAST_MAX_TORQUE);

        scene.mast_parallel = b3CreateParallelJoint(NEA_Phys3DWorldGetId(), &pdef);
        if (b3Joint_IsValid(scene.mast_parallel) == false)
            jointsMissing++;
    }
    else
    {
        jointsMissing += 2;
    }

    int32_t buildBytes = NEA_Phys3DWorldGetMemoryUsage();

    consoleDemoInit();

    printf("NEA Box3D Wheel Joint\n");
    printf("---------------------\n");
    printf("A/B: drive fwd/back\n");
    printf("Left/Right: steer\n");
    printf("X: susp limits  Y: spring\n");
    printf("SELECT: reset\n\n");

    uint32_t stepTicks = 0;
    uint32_t peakTicks = 0;
    int peakAwake = 0;
    uint32_t frames = 0;
    int respawns = 0;
    int overloads = 0;

#ifdef BOX3D_PHASE_PROFILE
    // Whole-frame attribution, for answering "where does the frame go" with a
    // number instead of a guess.
    //
    // NEA_GetCPUPercent() is a scanline count over the whole frame, so it can
    // say the frame costs 200% but not which part of it does. These break the
    // same frame into its phases using the timer the step measurement already
    // uses. cpuStartTiming/cpuEndTiming drive timers 0 and 1 as one cascaded
    // 33.514 MHz counter, so they cannot nest -- every region below is opened
    // and closed before the next one starts.
    //
    // floatTicks is deliberately its own phase: it brackets exactly the
    // soft-float work on the input path, so its cost is read directly rather
    // than inferred from an A/B of two ROMs.
    uint32_t syncTicks = 0;
    uint32_t floatTicks = 0;
    uint32_t convTicks = 0;
    uint32_t hudTicks = 0;
    uint32_t drawTicks = 0;

    // Averaged over a second. Per-frame numbers at this resolution jitter far
    // more than the differences being looked for.
    uint32_t accStep = 0, accSync = 0, accFloat = 0;
    uint32_t accConv = 0, accHud = 0, accDraw = 0, accFrames = 0;
    uint32_t avgStep = 0, avgSync = 0, avgFloat = 0;
    uint32_t avgConv = 0, avgHud = 0, avgDraw = 0;

    // The pool and event readouts give up their console lines to the phase
    // table in this build. Still collected, so the code paths that produce
    // them are still being measured -- just not displayed.
    (void)buildBytes;
#endif

    bool limited = true;
    bool stiff = true;

    // The ramped throttle, and the commanded steering angle in brads. Both
    // integer: the throttle is a b3f and the steering is brads, so nothing on
    // the input path needs an FPU the DS does not have.
    b3f commanded = b3f_zero;
    int steerTarget = 0;

    while (1)
    {
        scanKeys();
        uint32_t keys = keysHeld();
        uint32_t down = keysDown();
/*
#ifdef BOX3D_NO_INPUT
        // Baseline measurement build. melonDS injects stray keyboard events
        // into the focused window, and here they would land on the throttle or
        // the steering, either of which changes what is being measured.
        //
        // Same switch as box3d_basic, box3d_ragdoll, box3d_motor and
        // box3d_slider, and for the same reason: it is what makes the recorded
        // numbers a property of the simulation rather than of the emulator.
        keys = 0;
        down = 0;
#endif
*/
#ifdef BOX3D_AUTO_DRIVE
        // Verification build only: drive forward and sweep the steering on a
        // fixed schedule, so the headline behaviour can be measured without
        // depending on emulator keyboard input. The sweep is what walks the car
        // up the ramps and onto the tipped-wheel configurations that this
        // example exists to reach.
        // Forward, then back, so the car works the length of the level instead
        // of pinning against the far wall with its wheels turning -- which is
        // what a plain "always forward" schedule does after about five seconds,
        // and which measures the wall rather than the car.
        keys |= (( frames / 420 ) % 2 == 0) ? KEY_A : KEY_B;

        // Straight for most of each cycle, then a burst of lock. Steering held
        // at full lock the whole time just turns the car on the spot, which
        // measures nothing -- the car has to *travel* to reach the ramps, and
        // the ramps are where the tipped-wheel configurations are.
        if (( frames % 420 ) >= 330)
            keys |= (( frames / 420 ) % 2 == 0) ? KEY_LEFT : KEY_RIGHT;
#endif

        // **Negative is forward, and the sign is geometry rather than taste.**
        // The axle is world +z, so a wheel spinning at +w about it drives its
        // contact patch toward +x and pushes the car toward -x. A drives the car
        // along +x, into the level, so A commands a negative spin. Getting this
        // backwards is not subtle on screen -- the car reverses into the wall
        // behind it and sits there with the wheels turning, which is what the
        // first run of this example did.
        b3f wanted = b3f_zero;
        if (keys & KEY_A)
            wanted = b3fFromFrac(-DRIVE_SPEED_NUM, DRIVE_SPEED_DEN);
        else if (keys & KEY_B)
            wanted = b3fFromFrac(DRIVE_SPEED_NUM, DRIVE_SPEED_DEN);

        // Ramped, for box3d_slider's reason: the torque budget is well above
        // the load, so a step change in the target speed is a step change in
        // *acceleration*, which here spins the wheels rather than moving the
        // car and throws the mast about on the first frame of every reversal.
        // A real throttle ramps.
        //
        // The ramp is fixed-point rather than float. b3MulFC takes a b3f
        // against a Q30 coefficient, which is what THROTTLE_RAMP is, so the
        // whole input path is integer from the D-pad to the joint -- the same
        // property the steering path already had by being in brads throughout.
#ifdef BOX3D_PHASE_PROFILE
        cpuStartTiming(0);
#endif
        commanded = b3AddF(commanded, b3MulFC(b3SubF(wanted, commanded), THROTTLE_RAMP));

        // Hoisted out of the wheel loop below, where the conversion used to be
        // evaluated once per wheel for a value that does not depend on the
        // wheel.
        b3f commandedFix = commanded;
#ifdef BOX3D_PHASE_PROFILE
        floatTicks = cpuEndTiming();
#endif

        // Steering, in brads throughout. No conversion anywhere on this path:
        // the D-pad moves an integer, the integer is clamped to an integer, and
        // b3WheelJoint_SetTargetSteeringAngle takes it. Upstream's radians
        // would need a float and two conversions to do the same thing.
        if (keys & KEY_LEFT)
            steerTarget += STEER_RATE;
        else if (keys & KEY_RIGHT)
            steerTarget -= STEER_RATE;
        else
            steerTarget -= steerTarget / 8;

        if (steerTarget > STEER_MAX)
            steerTarget = STEER_MAX;
        if (steerTarget < -STEER_MAX)
            steerTarget = -STEER_MAX;

        for (int i = 0; i < NUM_WHEELS; i++)
        {
            if (b3Joint_IsValid(scene.wheel_joint[i]) == false)
                continue;

            b3WheelJoint_SetSpinMotorSpeed(scene.wheel_joint[i], commandedFix);

            // Only the front pair steers. The rear pair keeps a target of zero,
            // which is what holds it straight rather than letting it castor.
            b3WheelJoint_SetTargetSteeringAngle(scene.wheel_joint[i],
                                                ( i < 2 ) ? (b3a)steerTarget : 0);
        }

        // Joint accessors write a target and nothing else, so a car that has
        // gone to sleep would ignore a new command forever -- which looks
        // exactly like a dead motor. Waking it is the caller's job.
        if (( b3Raw(wanted) != 0 || b3Raw(commanded) != 0 || steerTarget != 0 ))
        {
            if (scene.chassis != NULL)
                NEA_Phys3DBodySetAwake(scene.chassis, true);
            for (int i = 0; i < NUM_WHEELS; i++)
            {
                if (scene.wheel[i] != NULL)
                    NEA_Phys3DBodySetAwake(scene.wheel[i], true);
            }
        }

        if (down & KEY_X)
        {
            limited = !limited;
            for (int i = 0; i < NUM_WHEELS; i++)
            {
                if (b3Joint_IsValid(scene.wheel_joint[i]))
                    b3WheelJoint_EnableSuspensionLimit(scene.wheel_joint[i], limited);
            }
        }

        if (down & KEY_Y)
        {
            stiff = !stiff;

            // Selected *after* conversion, not before. Written as
            // b3fFromDouble(stiff ? STIFF : SOFT) the ternary picks between two
            // doubles at run time and GCC has to keep a real __aeabi_d2iz to
            // narrow the winner; picking between two already-folded constants
            // leaves no float in the binary at all.
            b3f hertz = stiff ? b3fFromFrac(SUSPENSION_STIFF_NUM, SUSPENSION_HZ_DEN)
                              : b3fFromFrac(SUSPENSION_SOFT_NUM, SUSPENSION_HZ_DEN);

            for (int i = 0; i < NUM_WHEELS; i++)
            {
                if (b3Joint_IsValid(scene.wheel_joint[i]))
                {
                    b3WheelJoint_SetSuspensionHertz(scene.wheel_joint[i], hertz);
                }
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

#ifdef BOX3D_PHASE_PROFILE
        cpuStartTiming(0);
        NEA_Phys3DSyncModels();
        syncTicks = cpuEndTiming();
#else
        NEA_Phys3DSyncModels();
#endif

        // **Joint events, consumed.** Every suspension carries a force
        // threshold, so this counts the landings that overloaded one. The array
        // is cleared by the next step, so it has to be read every frame -- the
        // same contract the contact events have.
        b3JointEvents jointEvents = b3World_GetJointEvents(NEA_Phys3DWorldGetId());
        overloads += jointEvents.jointCount;
#ifdef BOX3D_PHASE_PROFILE
        (void)overloads;
#endif

        if (scene.chassis != NULL)
        {
            int32_t bx, by, bz;
            NEA_Phys3DBodyGetPositionI(scene.chassis, &bx, &by, &bz);

            if (by < floattof32(RESPAWN_BELOW))
            {
                ResetScene(&scene);
                respawns++;
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

        // The joints' own readouts. Travel is the front-left suspension's
        // displacement from its mount; steer is the angle that wheel actually
        // reached, which lags the target through the steering spring; tilt is
        // the mast's angular separation from the chassis, which is the parallel
        // joint's error and the number that moves on every bump.
        int travelMilli = 0;
        int steerNow = 0;
        int spinMilli = 0;
        int tiltBrads = 0;
        int posXMilli = 0;
        int posZMilli = 0;

#ifdef BOX3D_PHASE_PROFILE
        cpuStartTiming(0);
#endif

        // Millis straight off the raw Q12, no float in the middle. A b3f is
        // metres in Q12, so scaling by 1000 and shifting off the fraction is
        // the whole conversion -- and the product stays inside int32 for any
        // value this example can produce (1000 * 2^12 * 500 m is 2.0e9).
        if (b3Joint_IsValid(scene.wheel_joint[0]))
        {
            travelMilli = (int)(( b3Raw(b3WheelJoint_GetSuspensionTranslation(
                                      scene.wheel_joint[0])) * 1000 ) >> B3_F_SHIFT);
            steerNow = (int)b3WheelJoint_GetSteeringAngle(scene.wheel_joint[0]);
            spinMilli = (int)(( b3Raw(b3WheelJoint_GetSpinSpeed(
                                    scene.wheel_joint[0])) * 1000 ) >> B3_F_SHIFT);
        }

        if (b3Joint_IsValid(scene.mast_parallel))
            tiltBrads = (int)b3Joint_GetAngularSeparation(scene.mast_parallel);

        if (scene.chassis != NULL)
        {
            int32_t px, py, pz;
            NEA_Phys3DBodyGetPositionI(scene.chassis, &px, &py, &pz);
            // f32 is Q12 as well -- that is the whole point of b3f being
            // Q19.12 -- so the same shift serves here.
            posXMilli = (int)(( px * 1000 ) >> 12);
            posZMilli = (int)(( pz * 1000 ) >> 12);
        }

#ifdef BOX3D_PHASE_PROFILE
        convTicks = cpuEndTiming();

        // Averaged and republished once a second. The console is redrawn every
        // frame regardless, so this only steadies the numbers, it does not
        // reduce the printf cost being measured.
        accStep += stepTicks;
        accSync += syncTicks;
        accFloat += floatTicks;
        accConv += convTicks;
        accHud += hudTicks;
        accDraw += drawTicks;
        accFrames++;

        if (accFrames >= 60)
        {
            avgStep = accStep / accFrames;
            avgSync = accSync / accFrames;
            avgFloat = accFloat / accFrames;
            avgConv = accConv / accFrames;
            avgHud = accHud / accFrames;
            avgDraw = accDraw / accFrames;
            accStep = accSync = accFloat = 0;
            accConv = accHud = accDraw = 0;
            accFrames = 0;
        }

        // Ticks to microseconds ahead of the timer, not inside it. The divide
        // is a __aeabi_uidiv call in Thumb, and six of them landing inside the
        // window would be measuring the profiler.
        unsigned long usStep = (unsigned long)(avgStep * 1000u / 33514u);
        unsigned long usSync = (unsigned long)(avgSync * 1000u / 33514u);
        unsigned long usDraw = (unsigned long)(avgDraw * 1000u / 33514u);
        unsigned long usHud = (unsigned long)(avgHud * 1000u / 33514u);
        unsigned long usFloat = (unsigned long)(avgFloat * 1000u / 33514u);
        unsigned long usConv = (unsigned long)(avgConv * 1000u / 33514u);
        unsigned long usSum = usStep + usSync + usDraw + usHud + usFloat + usConv;

        cpuStartTiming(0);
#endif

        printf("\x1b[9;0Hstep   %5lu us (%lu ticks)  ",
               (unsigned long)stepMicros, (unsigned long)stepTicks);
        printf("\x1b[10;0Hpeak   %5lu us @ %d awake  ",
               (unsigned long)peakMicros, peakAwake);
        printf("\x1b[11;0Hawake  %2d / %d bodies      ", awake, NUM_BODIES);
        printf("\x1b[12;0Htravel %5d mm  steer %5d  ", travelMilli, steerNow);
        printf("\x1b[13;0Htarget %5d br  spin %5d  ", steerTarget, spinMilli);
        printf("\x1b[14;0Hmast %4d br  at %5d,%5d ", tiltBrads, posXMilli, posZMilli);
        printf("\x1b[15;0Hlimit  %-3s   spring %-5s  ",
               limited ? "on" : "OFF", stiff ? "stiff" : "soft");
#ifdef BOX3D_PHASE_PROFILE
        // The frame, attributed. All six in microseconds, averaged over the
        // last second. A 60 Hz frame is 16667 us, so these summed against that
        // say directly which phase is spending the frame -- and `float` is the
        // soft-float conversions on the input path, measured rather than
        // argued about.
        printf("\x1b[16;0Hphys %5lu  sync %5lu us  ", usStep, usSync);
        printf("\x1b[17;0Hdraw %5lu  hud  %5lu us  ", usDraw, usHud);
        printf("\x1b[18;0Hfloat %4lu  conv %5lu us  ", usFloat, usConv);
        printf("\x1b[19;0Hsum  %5lu us of 16667    ", usSum);
#else
        printf("\x1b[16;0Hoverld %5d events         ", overloads);
        printf("\x1b[17;0Hpool   %5ld / %ld bytes    ",
               (long)NEA_Phys3DWorldGetMemoryUsage(),
               (long)NEA_Phys3DWorldGetMemoryCapacity());
        printf("\x1b[18;0Hbuild  %5ld  mesh %5d     ", (long)buildBytes,
               level_b3mesh_size);
        printf("\x1b[19;0Hlate   %5d allocs, %ld ovf ",
               NEA_Phys3DWorldGetLateAllocCount(),
               (long)NEA_Phys3DWorldGetOverflowBytes());
#endif
        printf("\x1b[20;0Hcpu    %3d %%               ", NEA_GetCPUPercent());
        printf("\x1b[21;0Hmissing %d bd %d jt %d shp  ",
               bodiesMissing, jointsMissing, shapesMissing);
        printf("\x1b[22;0Hrespawn %3d  frame %8lu   ", respawns,
               (unsigned long)frames);

#ifdef BOX3D_PHASE_PROFILE
        hudTicks = cpuEndTiming();
#endif

        // The vblank wait is deliberately outside every phase timer. It is
        // slack, not work: over budget it returns immediately, so timing it
        // would measure what is left of the frame rather than what was spent.
        NEA_WaitForVBL(NEA_CAN_SKIP_VBL);

#ifdef BOX3D_PHASE_PROFILE
        cpuStartTiming(0);
        NEA_ProcessArg(Draw3DScene, &scene);
        drawTicks = cpuEndTiming();
#else
        NEA_ProcessArg(Draw3DScene, &scene);
#endif
    }

    return 0;
}
