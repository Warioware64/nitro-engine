// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Warioware64, 2026
//
// This file is part of Nitro Engine Advanced
//
// Box3D sensors: a trigger volume over a triangle level
// -----------------------------------------------------
// A box-shaped sensor floats above the baked level and crates fall through it.
// The sensor resolves nothing -- nothing bounces off it, nothing rests on it,
// and the crates fall exactly as they would if it were not there. What it does
// is *report*: one begin event as each crate enters, one end event as it leaves,
// and a list of what is inside it right now.
//
// Why that is a different thing from a contact: a contact exists because two
// shapes are touching, and the solver's job is to stop them touching. A sensor
// event exists because a *set changed between two steps* -- so the interesting
// number here is not how many crates are inside, it is that a crate sitting
// still inside the volume reports nothing at all. Only crossings are events.
//
// What the readout is for:
//
//   inside  How many crates are in the volume as of the last step, read with
//           b3Shape_GetSensorData. This is state, not events.
//
//   begin   Running totals of the transitions. A crate that falls through the
//   end     volume adds one to each, several steps apart. They are equal once
//           everything has fallen past, and that is the check: every entry was
//           matched by an exit.
//
//   swept   Crates caught by the *continuous* path -- entering and leaving
//           within a single step, too fast for the end-of-step overlap query to
//           have seen them anywhere near the volume. Press A to fire one and
//           watch this rise while `inside` never does.
//
//   late    Allocations made after the world was sealed. The sensor pass runs
//           inside the step, reuses the broad-phase trees the scene already
//           carries, and writes into arrays reserved when the sensor shape was
//           created -- so the number that matters is not that this reads zero,
//           it reads a small constant like every example here, but that it does
//           not move while crates cross the volume every frame.
//
// The sensor is created through Box3D's own API rather than NEA's, because
// `isSensor` lives on b3ShapeDef and NEA's NEA_Phys3DBodyAdd*() helpers do not
// hand back the b3ShapeId a sensor has to be polled with. NEA_Phys3DWorldGetId()
// and the body's `id` field are the same door the joint examples go through for
// b3CreateWheelJoint.
//
// Build with -DBOX3D_AUTO_FIRE to fire the fast crate on a fixed frame without
// touching a button, the way box3d_bullet does for its measurement build.

#include <NEAMain.h>
#include <NEAPhysics3D.h>

#include <nds/arm9/exceptions.h>

#include "cube_bin.h"
#include "level_bin.h"

#include "level_b3mesh.h"

// Matches box3d_level: the display list was authored at half size to fit v16's
// 8-unit limit, so the model draws at 2x to line up with the collision mesh.
#define LEVEL_DRAW_SCALE 2.0f

#define NUM_CRATES 6

#define CRATE_HALF 0.4f

// The trigger volume: wide enough that every crate falls through it, shallow
// enough that a crate is inside for a recognisable handful of steps rather than
// for its whole descent.
#define TRIGGER_HX 3.0f
#define TRIGGER_HY 0.6f
#define TRIGGER_HZ 3.0f

#define TRIGGER_X 0.0f
#define TRIGGER_Y 4.0f
#define TRIGGER_Z 0.0f

// Where a crate is dropped from at the start.
#define DROP_Y 9.0f

// How the scene keeps producing crossings: a crate that has fallen back below
// the volume and lost its momentum is *punted upward again*, by setting its
// velocity. Nothing is teleported.
//
// That is deliberate, and it is what the `late` line below is able to say
// anything with. The first version of this example lifted a settled crate back
// to the top with NEA_Phys3DBodySetPosition, and `late` then climbed by roughly
// one allocation per lift, forever -- a teleport destroys the body's contacts
// and its island and both are rebuilt from scratch on the next step. Measured:
// with the lift removed, `late` sat at 27 and did not move across 3,000 frames
// of crates crossing the volume. Punting by velocity leaves the contacts and
// the island alone, so the counter means what this file claims it means.
#define LAUNCH_BELOW 2.0f
#define LAUNCH_SPEED 12.0f

// The fast crate, and the number is not arbitrary. To be caught *only* by the
// sweep, a crate has to be clear of the volume at *both* ends of the step: it
// must start above y = 5.0 (the volume's top plus its own half depth) and end
// below y = 3.0.
//
// That makes the speed and the launch height one constraint, and picking them
// separately is how the first two tries failed. 60 m/s is one unit per step and
// never clears the volume at all -- the crate simply ends up inside it and is
// found by the ordinary query. 200 m/s is 3.33 units per step, which only works
// from a start between 5.0 and 6.33: a window a punted crate passes through in
// a few frames, so a shot fired on a timer almost always missed it. 350 m/s is
// 5.83 units per step and widens the window to 5.0 through 8.4, which is
// everything above the volume that a crate launched at LAUNCH_SPEED can reach.
#define FAST_SPEED 350.0f

// The lowest start a fast shot can sweep cleanly from, per the note above.
#define FAST_MIN_Y (TRIGGER_Y + TRIGGER_HY + CRATE_HALF)

typedef struct {
    NEA_Camera *camera;
    NEA_Model *level_model;
    NEA_Model *trigger_model;
    NEA_Model *crate_model[NUM_CRATES];
    NEA_Phys3DBody *crate[NUM_CRATES];
    NEA_Phys3DBody *level_body;
    NEA_Phys3DBody *trigger_body;

    // The sensor's shape id, which is what b3Shape_GetSensorData is asked
    // with. NEA has no equivalent -- see the note at the top.
    b3ShapeId sensor;

    // Each crate's shape id, so a visitor reported by the sensor can be named
    // as "crate 3" rather than as a raw index.
    b3ShapeId crate_shape[NUM_CRATES];
} SceneData;

// The hulls a b3ShapeId is built from are not copied: the shape keeps the
// caller's pointer, so both of these have to outlive every shape using them.
// File scope is what keeps that promise here.
static b3BoxHull s_triggerHull;
static b3BoxHull s_crateHull;

void Draw3DScene(void *arg)
{
    SceneData *scene = arg;

    NEA_CameraUse(scene->camera);

    NEA_PolyFormat(31, 0, NEA_LIGHT_0 | NEA_LIGHT_1, NEA_CULL_BACK, 0);
    for (int i = 0; i < NUM_CRATES; i++)
        NEA_ModelDraw(scene->crate_model[i]);

    NEA_PolyFormat(20, 0, NEA_LIGHT_0 | NEA_LIGHT_1, NEA_CULL_BACK, 0);
    NEA_ModelDraw(scene->level_model);

    // The volume itself, drawn last and translucent so the crates inside it
    // stay visible through it. A sensor has no appearance of its own -- this is
    // a cube model placed where the sensor shape is, and nothing connects the
    // two but the coordinates above.
    NEA_PolyFormat(10, 0, NEA_LIGHT_0, NEA_CULL_NONE, 0);
    NEA_ModelDraw(scene->trigger_model);
}

/// Drop a crate from the top, with a small spread so they do not stack.
static void DropCrate(SceneData *scene, int i)
{
    float x = -1.5f + 0.6f * (float)i;
    float z = -0.9f + 0.35f * (float)i;

    NEA_Phys3DBodySetPosition(scene->crate[i], x, DROP_Y, z);
    NEA_Phys3DBodySetVelocity(scene->crate[i], 0, 0, 0);
    NEA_Phys3DBodySetAwake(scene->crate[i], true);
}

/// Slam whichever crate is currently highest straight down through the volume.
///
/// By velocity, not by teleport, for the reason LAUNCH_SPEED gives -- and it has
/// to pick a crate that is already clear above the volume, because a sweep can
/// only be a sweep if it starts on one side and ends on the other.
static void FireFast(SceneData *scene)
{
    int best = -1;
    int32_t bestY = 0;

    for (int i = 0; i < NUM_CRATES; i++)
    {
        int32_t px, py, pz;
        NEA_Phys3DBodyGetPositionI(scene->crate[i], &px, &py, &pz);

        if (py < floattof32(FAST_MIN_Y))
            continue;

        if (best < 0 || py > bestY)
        {
            best = i;
            bestY = py;
        }
    }

    if (best >= 0)
        NEA_Phys3DBodySetVelocity(scene->crate[best], 0, -FAST_SPEED, 0);
}

/// Which crate a reported visitor shape is, or -1 for anything else.
static int CrateOfShape(const SceneData *scene, b3ShapeId id)
{
    for (int i = 0; i < NUM_CRATES; i++)
    {
        if (B3_ID_EQUALS(scene->crate_shape[i], id))
            return i;
    }
    return -1;
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

    def.box3d.capacity.dynamicBodyCount = NUM_CRATES;
    def.box3d.capacity.dynamicShapeCount = NUM_CRATES;

    // Two static bodies: the level and the body the sensor shape hangs on.
    def.box3d.capacity.staticBodyCount = 2;
    def.box3d.capacity.staticShapeCount = 2;

    // Every crate can be touching the level and several of its neighbours at
    // once, and the pile churns constantly because crates are lifted back to
    // the top rather than left to settle. `* 4` was the first try and it left
    // `late` climbing all through a run -- which is exactly what that counter
    // is for, and the diagnosis was quicker than the guess: with the sensor
    // *muted* the number climbed faster, so it was never the sensor pass.
    def.box3d.capacity.contactCount = NUM_CRATES * 8;
    def.box3d.capacity.meshContactCount = NUM_CRATES;

    // The one field this example exists to exercise. A sensor costs a slot here
    // plus its own overlap arrays, taken when the shape is created; leaving it
    // at zero would make the sensor grow the array as build memory instead,
    // which works but is not what a sized scene looks like.
    def.box3d.capacity.sensorCount = 1;

    // One, not NUM_CRATES: nothing here calls NEA_Phys3DBodyAddBox, because
    // both box shapes in this scene are created through b3CreateHullShape for
    // the flags they need. Zero would mean "one per shape" rather than none, so
    // one is the least this pool can be asked for.
    def.maxBoxHulls = 1;

    // 128 KB ran the pool to exactly full and then overflowed. box3d_level
    // reserves 192 KB for eight crates on this same mesh and never reaches it;
    // this scene has fewer crates but keeps them all awake and colliding, so it
    // gets the same figure.
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
    NEA_CameraSet(scene.camera,
                  -13, 8, 15,
                    0, 2, -2,
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
    // The trigger volume
    // ---------------------------------------------------------------------

    scene.trigger_model = NEA_ModelCreate(NEA_Static);
    NEA_ModelLoadStaticMesh(scene.trigger_model, cube_bin);
    NEA_ModelSetCoord(scene.trigger_model, TRIGGER_X, TRIGGER_Y, TRIGGER_Z);
    NEA_ModelScale(scene.trigger_model, TRIGGER_HX, TRIGGER_HY, TRIGGER_HZ);

    scene.trigger_body = NEA_Phys3DBodyCreate(b3_staticBody,
                                              TRIGGER_X, TRIGGER_Y, TRIGGER_Z);

    s_triggerHull = b3MakeBoxHull(b3Makeb3f(floattof32(TRIGGER_HX)),
                                  b3Makeb3f(floattof32(TRIGGER_HY)),
                                  b3Makeb3f(floattof32(TRIGGER_HZ)));

    {
        b3ShapeDef sensorDef = b3DefaultShapeDef();

        // The two flags that make this a sensor rather than a wall. Both ends
        // have to opt in -- see the crates below, which set the second one on
        // themselves.
        sensorDef.isSensor = true;
        sensorDef.enableSensorEvents = true;

        scene.sensor = b3CreateHullShape(scene.trigger_body->id, &sensorDef,
                                         &s_triggerHull.base);
    }

    // ---------------------------------------------------------------------
    // The crates
    // ---------------------------------------------------------------------

    s_crateHull = b3MakeBoxHull(b3Makeb3f(floattof32(CRATE_HALF)),
                                b3Makeb3f(floattof32(CRATE_HALF)),
                                b3Makeb3f(floattof32(CRATE_HALF)));

    for (int i = 0; i < NUM_CRATES; i++)
    {
        scene.crate_model[i] = NEA_ModelCreate(NEA_Static);
        NEA_ModelLoadStaticMesh(scene.crate_model[i], cube_bin);
        NEA_ModelScale(scene.crate_model[i], CRATE_HALF, CRATE_HALF, CRATE_HALF);

        scene.crate[i] = NEA_Phys3DBodyCreate(b3_dynamicBody, 0, DROP_Y, 0);

        // Through Box3D rather than NEA_Phys3DBodyAddBox, for the one reason
        // that matters here: a crate has to have enableSensorEvents set, and
        // that is a b3ShapeDef field. Keeping the returned id also lets the
        // readout name which crate the sensor is reporting.
        b3ShapeDef crateDef = b3DefaultShapeDef();
        crateDef.density = b3fFromInt(1000);
        crateDef.enableSensorEvents = true;
        scene.crate_shape[i] = b3CreateHullShape(scene.crate[i]->id, &crateDef,
                                                 &s_crateHull.base);

        NEA_Phys3DBodySetMaterial(scene.crate[i], 0.5, 0.1);
        NEA_Phys3DBodySetModel(scene.crate[i], scene.crate_model[i]);

        // A crate that falls asleep has to be woken to be punted, and waking a
        // body moves its sim between solver sets and rebuilds its island. This
        // scene never wants a crate to stop, so it never lets one sleep --
        // which keeps the wake path, and the allocations behind it, out of the
        // `late` reading entirely.
        b3Body_EnableSleep(scene.crate[i]->id, false);

        DropCrate(&scene, i);
    }

    int32_t buildBytes = NEA_Phys3DWorldGetMemoryUsage();

    consoleDemoInit();
    printf("NEA Box3D Sensors\n");
    printf("-----------------\n");
    printf("A: fast shot  B: drop all\n\n");

    uint32_t peakTicks = 0;
    int frames = 0;

    int beginTotal = 0;
    int endTotal = 0;
    int sweptTotal = 0;

    while (1)
    {
        frames++;

        scanKeys();
        uint32_t down = keysDown();

#ifdef BOX3D_NO_INPUT
        // Baseline measurement build. melonDS injects stray keyboard events
        // into the focused window, and here they land on A and B. Same switch
        // as box3d_basic, box3d_motor and box3d_bullet.
        down = 0;
#endif

#ifdef BOX3D_AUTO_FIRE
        // Verification build only: a fast shot every few seconds, so `swept`
        // fills itself in without depending on emulator keyboard input.
        //
        // Periodic rather than on two fixed frames, because FireFast needs a
        // crate that happens to be above the volume and a fixed frame is a
        // coin toss on whether one is -- the first version fired twice and
        // landed one shot.
        if (frames > 240 && (frames % 300) == 0)
            down |= KEY_A;
#endif

        if (down & KEY_A)
            FireFast(&scene);

        if (down & KEY_B)
        {
            // The one teleport left, and it is the user asking for one.
            for (int i = 0; i < NUM_CRATES; i++)
                DropCrate(&scene, i);
        }

        // Punt anything that has fallen back below the volume and run out of
        // momentum, so the scene keeps producing crossings instead of settling
        // into a pile. By velocity -- see LAUNCH_SPEED for why that matters.
        for (int i = 0; i < NUM_CRATES; i++)
        {
            int32_t px, py, pz;
            NEA_Phys3DBodyGetPositionI(scene.crate[i], &px, &py, &pz);

            if (py >= floattof32(LAUNCH_BELOW))
                continue;

            int32_t vx, vy, vz;
            NEA_Phys3DBodyGetVelocityI(scene.crate[i], &vx, &vy, &vz);

            bool settled = (vy > floattof32(-1.0f) && vy < floattof32(1.0f));

            // A crate that has gone over the edge of the level is falling fast
            // and would never meet the "settled" test, so it is caught by
            // height instead -- while it is just below the lip and a punt can
            // still bring it back.
            bool escaping = py < floattof32(-1.0f);

            if (settled || escaping)
            {
                // Steered back toward the middle rather than launched straight
                // up, and this is the second thing that went wrong here. The
                // first version gave each crate a fixed sideways nudge to keep
                // them from stacking; because every launch pushed the same way,
                // the offsets accumulated and within a couple of thousand
                // frames all six had walked off the level and the scene sat
                // empty with `begin` frozen. Straight up was no better -- the
                // level is not flat, so a crate landing on a slope slides.
                //
                // A pull proportional to how far out the crate is cannot
                // accumulate: it is zero in the middle and strongest at the
                // edge.
                float x = f32tofloat(px);
                float z = f32tofloat(pz);

                NEA_Phys3DBodySetVelocity(scene.crate[i], -0.8f * x, LAUNCH_SPEED,
                                          -0.8f * z);
            }
        }

        cpuStartTiming(0);
        NEA_Phys3DWorldStep();
        uint32_t stepTicks = cpuEndTiming();

        NEA_Phys3DSyncModels();

        if (stepTicks > peakTicks)
            peakTicks = stepTicks;

        // ------------------------------------------------------------------
        // The events
        // ------------------------------------------------------------------
        //
        // Read once per step, because that is how long they live: both arrays
        // are cleared at the top of the next b3World_Step.

        b3SensorEvents events = b3World_GetSensorEvents(NEA_Phys3DWorldGetId());

        beginTotal += events.beginCount;
        endTotal += events.endCount;

        // A swept crossing, told apart from an ordinary one by how fast the
        // visitor was going when its begin event arrived.
        //
        // The discriminator is the definition rather than a symptom: the
        // end-of-step overlap query can only report a crate it finds inside the
        // volume at the end of a step, so a crate covering more than the whole
        // depth it has to cross -- the volume's 1.2 plus its own 0.8 -- cannot
        // have been found that way. Above that speed a begin event can only
        // have come from the sweep.
        //
        // Two earlier versions of this counter were wrong, and both in the same
        // direction: a begin and an end for the same crate on the same step
        // (which never happens -- the hit is folded into *this* step's overlap
        // set, so the crate leaves on the next one, as test_sensors pins), and
        // then a position window, which counted every tumbling crate that
        // entered at a centre height the window called clear. A speed is not a
        // proxy for the thing; it is the thing.
        const float sweptSpeed = (TRIGGER_HY * 2.0f + CRATE_HALF * 2.0f) * 60.0f;

        for (int b = 0; b < events.beginCount; b++)
        {
            int crate = CrateOfShape(&scene, events.beginEvents[b].visitorShapeId);
            if (crate < 0)
                continue;

            int32_t vx, vy, vz;
            NEA_Phys3DBodyGetVelocityI(scene.crate[crate], &vx, &vy, &vz);

            float speed = f32tofloat(vy);
            if (speed < 0)
                speed = -speed;

            if (speed > sweptSpeed)
                sweptTotal++;
        }

        // ------------------------------------------------------------------
        // What is inside right now
        // ------------------------------------------------------------------

        b3ShapeId visitors[NUM_CRATES];
        int inside = b3Shape_GetSensorData(scene.sensor, visitors, NUM_CRATES);

        char insideList[NUM_CRATES * 2 + 1];
        int listLen = 0;
        for (int i = 0; i < inside && listLen < (int)sizeof(insideList) - 2; i++)
        {
            int crate = CrateOfShape(&scene, visitors[i]);
            insideList[listLen++] = (crate >= 0) ? (char)('0' + crate) : '?';
            insideList[listLen++] = ' ';
        }
        insideList[listLen] = '\0';

        uint32_t stepMicros = (stepTicks * 1000u) / 33514u;
        uint32_t peakMicros = (peakTicks * 1000u) / 33514u;

        printf("\x1b[6;0Hinside %d  [%s]              ", inside, insideList);
        printf("\x1b[7;0Hbegin  %4d                  ", beginTotal);
        printf("\x1b[8;0Hend    %4d                  ", endTotal);
        printf("\x1b[9;0Hswept  %4d                  ", sweptTotal);
        printf("\x1b[10;0Hstep   %5lu us              ",
               (unsigned long)stepMicros);
        printf("\x1b[11;0Hpeak   %5lu us              ",
               (unsigned long)peakMicros);
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
