// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Warioware64
//
// This file is part of Nitro Engine Advanced

/// @file   NEAPhysics3D.c
/// @brief  The NEA-facing half of the Box3D port.
///
/// Three jobs, in the order they matter:
///
///   1. **Own the memory.** One block from the heap at world creation, handed
///      to Box3D through b3SetAllocator, so the simulation is contiguous and
///      does not fragment the heap models and textures live in. The world
///      seals itself on the first step, and every allocation after that is
///      counted and (under NEA_DEBUG) asserted.
///   2. **Bind bodies to models.** Driven off the move events the solver
///      already publishes, so a settled scene costs nothing to sync.
///   3. **Speak NEA's conventions.** f32 in, f32 out, `I` suffixes, and the
///      Box3D ids left visible so the raw API stays available.
///
/// Shapes are boxes, spheres, capsules and baked triangle meshes. The first
/// three are built here from numbers; a mesh is a blob the caller owns, because
/// there is no run-time mesh builder -- see obj2dl.py --collision-b3.
///
/// This translation unit lives in **libNEA_box3d.a**, not libNEA.a. It calls
/// into libNEA.a for NEA_ModelSetMatrix; the reverse direction goes through a
/// registered function pointer (NEA_Phys3DGetUpdateHook) rather than a direct
/// call, so libNEA.a never acquires an undefined Box3D symbol and a ROM that
/// does not link this archive is byte-identical to one built before it existed.

// NEAMain.h first, the way every other module in source/ does it: NEAModel.h
// and friends rely on it having pulled in nds.h and the shared typedefs.
#include "NEAMain.h"
#include "NEAPhysics3D.h"

// Internal Box3D headers. Legitimate here and nowhere else: this file is part
// of the library, so it may see b3World's layout to report stack usage. A game
// gets <box3d/box3d.h> and the ids.
#include "physics_world.h"

#include <stdlib.h>
#include <string.h>

// =========================================================================
// The pool
// =========================================================================
//
// A bump allocator over one heap block. Not a general allocator, and it does
// not try to be: Box3D allocates almost entirely while the world is being
// built, and what it frees it frees in the reverse order (arrays growing,
// b3ArenaSync releasing overflow blocks). So a bump pointer with a rewind for
// the most recent block recovers nearly all of the churn for a few
// instructions, and everything else is reclaimed wholesale at world teardown.
//
// B3_MAX_WORLDS is 1 -- see nea_config.h -- and b3SetAllocator is global, so a
// single file-static pool is the whole of the state.

typedef struct {
    char *memory;       // The block. NULL when no world exists.
    int32_t capacity;
    int32_t offset;     // Bump pointer.
    int32_t peak;       // High water mark of offset.

    int32_t overflowBytes;  // Served from the heap because the pool ran dry.
    int lateAllocCount;     // Allocations made after the world sealed.
    bool sealed;

    // The most recent pool allocation, so a matching free can rewind. Box3D's
    // array growth is realloc-shaped -- allocate new, copy, free old -- and
    // without this every b3Array_Reserve past its initial size would leak the
    // old block until teardown.
    char *lastBlock;
    int32_t lastOffset;
} nea_phys3d_pool;

static nea_phys3d_pool nea_pool;

// Alignment Box3D asks for is 4 or 8 here; honour whatever it passes.
static void *nea_phys3d_alloc(int32_t size, int32_t alignment)
{
    if (size <= 0)
        return NULL;

    if (nea_pool.sealed)
    {
        nea_pool.lateAllocCount++;
        NEA_Assert(0, "Phys3D: %ld bytes allocated after the world sealed. "
                      "A capacity in b3WorldDef.capacity is too small.",
                   (long)size);
    }

    if (nea_pool.memory != NULL)
    {
        int32_t mask = alignment - 1;
        int32_t start = (nea_pool.offset + mask) & ~mask;

        if (start + size <= nea_pool.capacity)
        {
            char *block = nea_pool.memory + start;

            nea_pool.lastBlock = block;
            nea_pool.lastOffset = nea_pool.offset;
            nea_pool.offset = start + size;

            if (nea_pool.offset > nea_pool.peak)
                nea_pool.peak = nea_pool.offset;

            return block;
        }
    }

    // Out of pool. Serve it from the heap rather than returning NULL: Box3D's
    // callers write through the pointer without checking, so a refusal here is
    // a crash, and a crash is a worse diagnostic than a counter.
    //
    // malloc is enough for the alignment Box3D asks for. It requests 4 or 8,
    // and newlib's malloc returns 8-byte aligned blocks; the assert says so
    // rather than assuming it.
    nea_pool.overflowBytes += size;
    NEA_Assert(nea_pool.memory == NULL,
               "Phys3D: pool exhausted, %ld bytes from the heap. "
               "Raise NEA_Phys3DWorldDef.poolBytes.", (long)size);

    void *mem = malloc((size_t)size);
    NEA_Assert(((uintptr_t)mem & (uintptr_t)(alignment - 1)) == 0,
               "Phys3D: heap block under-aligned (wanted %ld)", (long)alignment);
    return mem;
}

static void nea_phys3d_free(void *mem)
{
    if (mem == NULL)
        return;

    if (nea_pool.memory != NULL &&
        (char *)mem >= nea_pool.memory &&
        (char *)mem < nea_pool.memory + nea_pool.capacity)
    {
        // Inside the pool. Rewind if this was the most recent block; otherwise
        // the space is reclaimed when the world is destroyed.
        if (mem == nea_pool.lastBlock)
        {
            nea_pool.offset = nea_pool.lastOffset;
            nea_pool.lastBlock = NULL;
        }
        return;
    }

    free(mem);
}

// =========================================================================
// World state
// =========================================================================

static b3WorldId nea_worldId;
static bool nea_worldExists;

// Body records, and the box hulls their shapes point at. Both are reserved
// from the pool at creation, because a hull has to outlive every shape
// referencing it and a body record has to outlive the body.
static NEA_Phys3DBody *nea_bodies;
static int nea_bodyCapacity;
static int nea_bodyCount;

static b3BoxHull *nea_boxHulls;
static int nea_boxHullCapacity;
static int nea_boxHullCount;

static int nea_subStepCount = B3_NEA_DEFAULT_SUBSTEPS;

// Kept from the world definition so that NEA_Phys3DBodyAddMeshI can tell an
// author that the world was not sized for a mesh. b3World does not store the
// b3Capacity it was built from -- it spends it and keeps the results -- and
// this is the one field a caller can still act on afterwards.
static int nea_meshContactCapacity;

NEA_Phys3DWorldDef NEA_Phys3DDefaultWorldDef(void)
{
    NEA_Phys3DWorldDef def = { 0 };

    def.box3d = b3DefaultWorldDef();

    // 64 KB holds a few dozen bodies comfortably. The example prints the real
    // number; this is only a starting point that does not immediately overflow.
    def.poolBytes = 64 * 1024;
    def.maxBoxHulls = 0;
    def.maxBodies = 0;

    // Box3D's own recommendation, and not conditioned on the console -- see
    // the "On DSi" note in NEAPhysics3D.h for why isDSiMode() is the wrong
    // thing to branch on here.
    def.subStepCount = B3_NEA_DEFAULT_SUBSTEPS;

    return def;
}

int NEA_Phys3DWorldInit(const NEA_Phys3DWorldDef *def)
{
    if (def == NULL)
        return -1;

    if (nea_worldExists)
    {
        NEA_DebugPrint("Phys3D: a world already exists (B3_MAX_WORLDS is 1)");
        return -1;
    }

    int32_t poolBytes = def->poolBytes > 0 ? def->poolBytes : 64 * 1024;

    memset(&nea_pool, 0, sizeof(nea_pool));
    nea_pool.memory = malloc((size_t)poolBytes);
    if (nea_pool.memory == NULL)
    {
        NEA_DebugPrint("Phys3D: could not reserve %ld byte pool",
                       (long)poolBytes);
        return -1;
    }
    nea_pool.capacity = poolBytes;

    // From here on everything Box3D allocates comes from the pool.
    b3SetAllocator(nea_phys3d_alloc, nea_phys3d_free);

    int bodyCapacity = def->maxBodies > 0
        ? def->maxBodies
        : def->box3d.capacity.staticBodyCount + def->box3d.capacity.dynamicBodyCount;
    if (bodyCapacity < 1)
        bodyCapacity = 16;

    int hullCapacity = def->maxBoxHulls > 0
        ? def->maxBoxHulls
        : def->box3d.capacity.staticShapeCount + def->box3d.capacity.dynamicShapeCount;
    if (hullCapacity < 1)
        hullCapacity = 16;

    // Taken before b3CreateWorld so that a pool too small to hold even the
    // bookkeeping fails here rather than halfway through building the world.
    nea_bodies = nea_phys3d_alloc((int32_t)(bodyCapacity * (int)sizeof(NEA_Phys3DBody)), 4);
    nea_boxHulls = nea_phys3d_alloc((int32_t)(hullCapacity * (int)sizeof(b3BoxHull)), 8);

    if (nea_bodies == NULL || nea_boxHulls == NULL)
    {
        b3SetAllocator(NULL, NULL);
        free(nea_pool.memory);
        memset(&nea_pool, 0, sizeof(nea_pool));
        return -1;
    }

    memset(nea_bodies, 0, (size_t)bodyCapacity * sizeof(NEA_Phys3DBody));

    nea_bodyCapacity = bodyCapacity;
    nea_bodyCount = 0;
    nea_boxHullCapacity = hullCapacity;
    nea_boxHullCount = 0;

    nea_subStepCount = def->subStepCount > 0 ? def->subStepCount : B3_NEA_DEFAULT_SUBSTEPS;
    nea_meshContactCapacity = def->box3d.capacity.meshContactCount;

    nea_worldId = b3CreateWorld(&def->box3d);
    if (B3_IS_NULL(nea_worldId))
    {
        b3SetAllocator(NULL, NULL);
        free(nea_pool.memory);
        memset(&nea_pool, 0, sizeof(nea_pool));
        nea_bodies = NULL;
        nea_boxHulls = NULL;
        return -1;
    }

    nea_worldExists = true;
    return 0;
}

void NEA_Phys3DWorldEnd(void)
{
    if (!nea_worldExists)
        return;

    b3DestroyWorld(nea_worldId);

    nea_worldExists = false;
    nea_worldId = b3_nullWorldId;
    nea_bodies = NULL;
    nea_bodyCapacity = 0;
    nea_bodyCount = 0;
    nea_boxHulls = NULL;
    nea_boxHullCapacity = 0;
    nea_boxHullCount = 0;
    nea_meshContactCapacity = 0;

    b3SetAllocator(NULL, NULL);

    free(nea_pool.memory);
    memset(&nea_pool, 0, sizeof(nea_pool));
}

bool NEA_Phys3DWorldExists(void)
{
    return nea_worldExists;
}

b3WorldId NEA_Phys3DWorldGetId(void)
{
    return nea_worldId;
}

void NEA_Phys3DWorldStep(void)
{
    NEA_Phys3DWorldStepI(nea_subStepCount);
}

void NEA_Phys3DWorldStepI(int subStepCount)
{
    if (!nea_worldExists)
        return;

    // The seal goes down on the first step, not at creation: building the
    // scene legitimately allocates, running it should not. Everything after
    // this point that reaches the allocator is a sizing mistake worth naming.
    nea_pool.sealed = true;

    nea_subStepCount = subStepCount;
    b3World_Step(nea_worldId, subStepCount);
}

void NEA_Phys3DWorldSetGravityI(int32_t x, int32_t y, int32_t z)
{
    if (!nea_worldExists)
        return;

    b3World_SetGravity(nea_worldId, b3MakeVec3(b3Makeb3f(x), b3Makeb3f(y),
                                               b3Makeb3f(z)));
}

int NEA_Phys3DWorldGetAwakeBodyCount(void)
{
    if (!nea_worldExists)
        return 0;

    return b3World_GetAwakeBodyCount(nea_worldId);
}

// =========================================================================
// Model binding
// =========================================================================

void NEA_Phys3DBodySetModel(NEA_Phys3DBody *body, NEA_Model *model)
{
    NEA_AssertPointer(body, "NULL body");

    body->model = model;
}

/// Push one body transform into its model.
///
/// The transpose is the one NEA_RigidBodySync already proves out
/// (NEARigidBody.c): b3MakeMatrixFromQuat returns row-major, m4x3 is
/// column-major, and m->x/y/z has to be written as well or the user-facing
/// position queries go stale while the drawn model moves.
static void nea_phys3d_apply_transform(NEA_Model *m, b3WorldTransform xf)
{
    b3Matrix3 r = b3MakeMatrixFromQuat(xf.q);

    int32_t px = b3Raw(xf.p.x);
    int32_t py = b3Raw(xf.p.y);
    int32_t pz = b3Raw(xf.p.z);

    m->x = px;
    m->y = py;
    m->z = pz;

    // The model's own scale, folded into the rotation columns.
    //
    // NEA_ModelSetMatrix is documented to ignore a model's scale, position and
    // rotation -- a matrix replaces all three. That is right for the position
    // and rotation, which is exactly what this function is here to overwrite,
    // and wrong for the scale, which describes the *mesh* rather than where
    // the body is: a caller who says NEA_ModelScaleI( m, 2, 1, 1 ) is
    // describing a shape, not a pose, and has no reason to expect attaching
    // the model to a body to silently discard it.
    //
    // Without this every physics-driven model draws at scale 1 whatever it
    // was given. box3d_basic and box3d_rope never showed it -- their models
    // are cubes and small spheres, so an ignored scale looks like a slightly
    // different camera -- and box3d_hinge exposed it immediately, drawing a
    // door, a turntable and a thin arm as three identical boxes.
    //
    // Scaling the columns is the correct composition: the matrix is applied
    // as M * v, so scaling column i scales the contribution of the mesh's
    // i-th axis, which is a scale in the *model's* frame and then a rotation.
    int32_t sx = m->sx;
    int32_t sy = m->sy;
    int32_t sz = m->sz;

    m4x3 mat;
    // Column 0
    mat.m[0]  = mulf32(b3Raw(r.cx.x), sx);
    mat.m[1]  = mulf32(b3Raw(r.cx.y), sx);
    mat.m[2]  = mulf32(b3Raw(r.cx.z), sx);
    // Column 1
    mat.m[3]  = mulf32(b3Raw(r.cy.x), sy);
    mat.m[4]  = mulf32(b3Raw(r.cy.y), sy);
    mat.m[5]  = mulf32(b3Raw(r.cy.z), sy);
    // Column 2
    mat.m[6]  = mulf32(b3Raw(r.cz.x), sz);
    mat.m[7]  = mulf32(b3Raw(r.cz.y), sz);
    mat.m[8]  = mulf32(b3Raw(r.cz.z), sz);
    // Translation
    mat.m[9]  = px;
    mat.m[10] = py;
    mat.m[11] = pz;

    NEA_ModelSetMatrix(m, &mat);
}

void NEA_Phys3DSyncModels(void)
{
    if (!nea_worldExists)
        return;

    // One entry per awake body that moved, so a settled scene iterates zero
    // times. That is the whole reason to drive this off events rather than
    // walking the body array.
    b3BodyEvents events = b3World_GetBodyEvents(nea_worldId);

    for (int i = 0; i < events.moveCount; i++)
    {
        NEA_Phys3DBody *body = events.moveEvents[i].userData;

        if (body != NULL && body->model != NULL)
            nea_phys3d_apply_transform(body->model, events.moveEvents[i].transform);
    }
}

void NEA_Phys3DUpdate(void)
{
    NEA_Phys3DWorldStepI(nea_subStepCount);
    NEA_Phys3DSyncModels();
}

void NEA_Phys3DSetSubStepCount(int subStepCount)
{
    nea_subStepCount = subStepCount < 1 ? 1 : subStepCount;
}

// =========================================================================
// Memory reporting
// =========================================================================

int32_t NEA_Phys3DWorldGetMemoryUsage(void)
{
    return nea_pool.peak;
}

int32_t NEA_Phys3DWorldGetMemoryCapacity(void)
{
    return nea_pool.capacity;
}

int32_t NEA_Phys3DWorldGetOverflowBytes(void)
{
    return nea_pool.overflowBytes;
}

int NEA_Phys3DWorldGetLateAllocCount(void)
{
    return nea_pool.lateAllocCount;
}

int32_t NEA_Phys3DWorldGetStackUsage(void)
{
    if (!nea_worldExists)
        return 0;

    b3World *world = b3GetWorldFromId(nea_worldId);
    return b3GetMaxStackAllocation(&world->stack);
}

int NEA_Phys3DWorldGetMeshManifoldDropCount(void)
{
    if (!nea_worldExists)
        return 0;

    // Per step, not cumulative: b3Collide zeroes this at the top of every
    // collide pass. Reaching into b3World for it is the same liberty
    // NEA_Phys3DWorldGetStackUsage takes, and legitimate for the same reason --
    // this file is part of the library.
    b3World *world = b3GetWorldFromId(nea_worldId);
    return world->meshManifoldDropCount;
}

int32_t NEA_Phys3DWorldGetStackCapacity(void)
{
    if (!nea_worldExists)
        return 0;

    b3World *world = b3GetWorldFromId(nea_worldId);
    return b3GetStackCapacity(&world->stack);
}

// =========================================================================
// Bodies
// =========================================================================

NEA_Phys3DBody *NEA_Phys3DBodyCreateI(b3BodyType type,
                                      int32_t x, int32_t y, int32_t z)
{
    if (!nea_worldExists)
        return NULL;

    if (nea_bodyCount == nea_bodyCapacity)
    {
        NEA_DebugPrint("Phys3D: body pool full (%d). Raise maxBodies.",
                       nea_bodyCapacity);
        return NULL;
    }

    NEA_Phys3DBody *body = &nea_bodies[nea_bodyCount];

    b3BodyDef def = b3DefaultBodyDef();
    def.type = type;
    def.position = b3MakeVec3(b3Makeb3f(x), b3Makeb3f(y), b3Makeb3f(z));
    // The record is the body's user data, which is how a move event finds its
    // way back to the bound model without a search.
    def.userData = body;

    b3BodyId id = b3CreateBody(nea_worldId, &def);
    if (B3_IS_NULL(id))
        return NULL;

    body->id = id;
    body->model = NULL;
    body->userData = NULL;

    nea_bodyCount++;
    return body;
}

void NEA_Phys3DBodyDelete(NEA_Phys3DBody *body)
{
    if (body == NULL || !nea_worldExists)
        return;

    b3DestroyBody(body->id);

    // Swap the last record down, and repoint the body that owned it. The
    // records are pooled by index, so leaving a hole would strand the slot for
    // the life of the world.
    int index = (int)(body - nea_bodies);
    int last = nea_bodyCount - 1;

    if (index != last)
    {
        nea_bodies[index] = nea_bodies[last];
        b3Body_SetUserData(nea_bodies[index].id, &nea_bodies[index]);
    }

    memset(&nea_bodies[last], 0, sizeof(NEA_Phys3DBody));
    nea_bodyCount--;
}

int NEA_Phys3DBodyAddBoxI(NEA_Phys3DBody *body,
                          int32_t hx, int32_t hy, int32_t hz, int32_t density)
{
    if (body == NULL || !nea_worldExists)
        return -1;

    if (nea_boxHullCount == nea_boxHullCapacity)
    {
        NEA_DebugPrint("Phys3D: box hull pool full (%d). Raise maxBoxHulls.",
                       nea_boxHullCapacity);
        return -1;
    }

    // The hull must outlive the shape: b3Shape keeps the caller's pointer
    // rather than copying, because DS hulls are normally baked blobs in ROM.
    // This one is not in ROM, so the pool is what keeps the promise.
    b3BoxHull *hull = &nea_boxHulls[nea_boxHullCount];
    *hull = b3MakeBoxHull(b3Makeb3f(hx), b3Makeb3f(hy), b3Makeb3f(hz));

    b3ShapeDef def = b3DefaultShapeDef();
    def.density = b3Makeb3f(density);

    b3ShapeId shape = b3CreateHullShape(body->id, &def, &hull->base);
    if (B3_IS_NULL(shape))
        return -1;

    nea_boxHullCount++;
    return 0;
}

int NEA_Phys3DBodyAddSphereI(NEA_Phys3DBody *body, int32_t radius,
                             int32_t density)
{
    if (body == NULL || !nea_worldExists)
        return -1;

    b3Sphere sphere;
    sphere.center = b3Vec3_zero;
    sphere.radius = b3Makeb3f(radius);

    b3ShapeDef def = b3DefaultShapeDef();
    def.density = b3Makeb3f(density);

    b3ShapeId shape = b3CreateSphereShape(body->id, &def, &sphere);
    return B3_IS_NULL(shape) ? -1 : 0;
}

int NEA_Phys3DBodyAddCapsuleI(NEA_Phys3DBody *body, int32_t radius,
                              int32_t halfHeight, int32_t density)
{
    if (body == NULL || !nea_worldExists)
        return -1;

    b3f half = b3Makeb3f(halfHeight);

    b3Capsule capsule;
    capsule.center1 = b3MakeVec3(b3f_zero, b3NegF(half), b3f_zero);
    capsule.center2 = b3MakeVec3(b3f_zero, half, b3f_zero);
    capsule.radius = b3Makeb3f(radius);

    b3ShapeDef def = b3DefaultShapeDef();
    def.density = b3Makeb3f(density);

    b3ShapeId shape = b3CreateCapsuleShape(body->id, &def, &capsule);
    return B3_IS_NULL(shape) ? -1 : 0;
}

int NEA_Phys3DBodyAddMeshI(NEA_Phys3DBody *body, const void *blob,
                           int32_t sx, int32_t sy, int32_t sz)
{
    if (body == NULL || !nea_worldExists)
        return -1;

    if (blob == NULL)
    {
        NEA_DebugPrint("Phys3D: NULL mesh blob");
        return -1;
    }

    // b3MeshData opens with a uint64_t, which the ARM9 reads with LDRD, which
    // requires an 8-byte-aligned address. bin2c emits aligned(4), so a .b3mesh
    // converted as a data/ file is exactly the pointer that arrives here and
    // faults. Refused rather than asserted: NEA_Assert compiles out.
    if (((uintptr_t)blob & 7) != 0)
    {
        NEA_DebugPrint("Phys3D: mesh blob at %p is not 8-byte aligned. "
                       "Embed it with obj2dl --collision-b3-c, not bin2c.",
                       blob);
        return -1;
    }

    const b3MeshData *mesh = blob;

    // Version, offsets, and that every BVH node's bounds contain what it points
    // at. Compiled in unconditionally in the port for this exact case -- a
    // baked blob is untrusted input and can be stale.
    if (b3IsValidMesh(mesh) == false)
    {
        NEA_DebugPrint("Phys3D: not a valid .b3mesh. Rebuild it with "
                       "obj2dl --collision-b3.");
        return -1;
    }

    // A mesh has zero mass and zero inertia, so a dynamic body carrying one has
    // nothing to solve with and falls through the world. Upstream asserts;
    // refusing says the same thing in a release build.
    if (b3Body_GetType(body->id) == b3_dynamicBody)
    {
        NEA_DebugPrint("Phys3D: a mesh shape has no mass and belongs on a "
                       "static or kinematic body.");
        return -1;
    }

    // The one sizing mistake worth catching at the call site. A mesh contact
    // carries up to B3_NEA_MAX_MESH_MANIFOLDS manifolds where a convex contact
    // carries one, and b3CreateWorld sized the manifold allocators, the
    // triangle caches and the arena from this number -- at zero, none of that
    // exists and the first body to touch the level allocates mid-step.
    if (nea_meshContactCapacity < 1)
    {
        NEA_DebugPrint("Phys3D: mesh added but capacity.meshContactCount is 0. "
                       "Set it to the number of shapes that can touch the "
                       "level at once, or the first contact allocates.");
    }

    b3ShapeDef def = b3DefaultShapeDef();

    b3Vec3 scale = b3MakeVec3(b3Makeb3f(sx), b3Makeb3f(sy), b3Makeb3f(sz));

    // The blob is not copied -- b3Shape keeps this pointer, the same contract
    // hulls have. Nothing is taken from the pool here, because the caller owns
    // the storage: a .b3mesh is normally an aligned array in ROM.
    b3ShapeId shape = b3CreateMeshShape(body->id, &def, mesh, scale);
    return B3_IS_NULL(shape) ? -1 : 0;
}

// =========================================================================
// Bone shapes -- the .b3col table
// =========================================================================

#define B3CL_MAGIC   0x4C433342  // "B3CL" little-endian
#define B3CL_VERSION 1

// .b3col header, 16 bytes.
typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t num_bones;
    uint32_t reserved;
} b3col_header_t;

// .b3col per-bone entry, 48 bytes.
//
// Everything is f32 -- which is b3f -- except the rotation, which is b3n. A
// quaternion component lives in [-1, 1], so f32 would spend 19 of its 32 bits
// on an integer part that is always zero and leave about a third of a degree of
// resolution. b3n is the scale b3Quat itself uses, so these read straight in.
typedef struct {
    uint8_t  type;
    uint8_t  joint;
    uint8_t  pad[2];
    int32_t  param[3];
    int32_t  offset[3];
    int32_t  rotation[4];   // x, y, z, w
    uint32_t reserved;
} b3col_bone_t;

/// The entry at `index`, or NULL if the table or the index is unusable.
///
/// Every accessor goes through this rather than trusting the caller, because a
/// .b3col is an asset and a stale asset is exactly the input that reaches a
/// release ROM.
static const b3col_bone_t *nea_b3col_entry(const void *b3col, int index)
{
    if (b3col == NULL)
        return NULL;

    const b3col_header_t *hdr = b3col;

    if (hdr->magic != B3CL_MAGIC || hdr->version != B3CL_VERSION)
        return NULL;

    if (index < 0 || (uint32_t)index >= hdr->num_bones)
        return NULL;

    const b3col_bone_t *bones =
        (const b3col_bone_t *)((const uint8_t *)b3col + sizeof(b3col_header_t));

    return &bones[index];
}

int NEA_Phys3DBoneShapeCount(const void *b3col)
{
    if (b3col == NULL)
        return 0;

    const b3col_header_t *hdr = b3col;

    if (hdr->magic != B3CL_MAGIC || hdr->version != B3CL_VERSION)
    {
        NEA_DebugPrint("Phys3D: not a valid .b3col");
        return 0;
    }

    return (int)hdr->num_bones;
}

int NEA_Phys3DBoneShapeJoint(const void *b3col, int index)
{
    const b3col_bone_t *bone = nea_b3col_entry(b3col, index);
    return bone == NULL ? -1 : (int)bone->joint;
}

NEA_Phys3DBoneShapeType NEA_Phys3DBoneShapeGetType(const void *b3col, int index)
{
    const b3col_bone_t *bone = nea_b3col_entry(b3col, index);
    return bone == NULL ? NEA_B3COL_TYPE_NONE
                        : (NEA_Phys3DBoneShapeType)bone->type;
}

int NEA_Phys3DBodyAddBoneShapeI(NEA_Phys3DBody *body, const void *b3col,
                                int index, int32_t density)
{
    if (body == NULL || !nea_worldExists)
        return -1;

    const b3col_bone_t *bone = nea_b3col_entry(b3col, index);
    if (bone == NULL)
    {
        NEA_DebugPrint("Phys3D: .b3col entry %d is not readable", index);
        return -1;
    }

    // The bone-local placement, baked into the shape so that the body's
    // transform is the bone's transform with nothing added.
    b3Vec3 offset = b3MakeVec3(b3Makeb3f(bone->offset[0]),
                               b3Makeb3f(bone->offset[1]),
                               b3Makeb3f(bone->offset[2]));

    b3Quat rotation = b3MakeQuat(b3Makeb3n(bone->rotation[0]),
                                 b3Makeb3n(bone->rotation[1]),
                                 b3Makeb3n(bone->rotation[2]),
                                 b3Makeb3n(bone->rotation[3]));

    b3ShapeDef def = b3DefaultShapeDef();
    def.density = b3Makeb3f(density);

    b3ShapeId shape = b3_nullShapeId;

    switch (bone->type)
    {
        case NEA_B3COL_TYPE_NONE:
            // A bone the author gave no shape. Not an error -- a .md5collimesh
            // can name a joint and then decline to hit-test it.
            return 0;

        case NEA_B3COL_TYPE_SPHERE:
        {
            b3Sphere sphere;
            sphere.center = offset;
            sphere.radius = b3Makeb3f(bone->param[0]);

            shape = b3CreateSphereShape(body->id, &def, &sphere);
            break;
        }

        case NEA_B3COL_TYPE_CAPSULE:
        {
            // The capsule runs between two hemisphere centres, so the stored
            // half height becomes a vector along the shape's own axis --
            // whatever the .md5collimesh's `axis` rotated +Y onto.
            b3Vec3 axis = b3RotateVector(
                rotation,
                b3MakeVec3(b3f_zero, b3Makeb3f(bone->param[1]), b3f_zero));

            b3Capsule capsule;
            capsule.center1 = b3Sub(offset, axis);
            capsule.center2 = b3Add(offset, axis);
            capsule.radius = b3Makeb3f(bone->param[0]);

            shape = b3CreateCapsuleShape(body->id, &def, &capsule);
            break;
        }

        case NEA_B3COL_TYPE_BOX:
        {
            // Same pool discipline as NEA_Phys3DBodyAddBoxI, and for the same
            // reason: b3Shape keeps the caller's hull pointer rather than
            // copying, so the hull must outlive the shape.
            if (nea_boxHullCount == nea_boxHullCapacity)
            {
                NEA_DebugPrint("Phys3D: box hull pool full (%d). Bone boxes "
                               "count towards maxBoxHulls too.",
                               nea_boxHullCapacity);
                return -1;
            }

            b3Transform placement;
            placement.p = offset;
            placement.q = rotation;

            b3BoxHull *hull = &nea_boxHulls[nea_boxHullCount];
            *hull = b3MakeTransformedBoxHull(b3Makeb3f(bone->param[0]),
                                             b3Makeb3f(bone->param[1]),
                                             b3Makeb3f(bone->param[2]),
                                             placement);

            shape = b3CreateHullShape(body->id, &def, &hull->base);
            if (B3_IS_NULL(shape))
                return -1;

            nea_boxHullCount++;
            return 0;
        }

        default:
            NEA_DebugPrint("Phys3D: .b3col entry %d has unknown type %d",
                           index, (int)bone->type);
            return -1;
    }

    return B3_IS_NULL(shape) ? -1 : 0;
}

void NEA_Phys3DBodySetPositionI(NEA_Phys3DBody *body,
                                int32_t x, int32_t y, int32_t z)
{
    if (body == NULL)
        return;

    b3Body_SetTransform(body->id,
                        b3MakeVec3(b3Makeb3f(x), b3Makeb3f(y), b3Makeb3f(z)),
                        b3Quat_identity);
}

void NEA_Phys3DBodyGetPositionI(const NEA_Phys3DBody *body,
                                int32_t *x, int32_t *y, int32_t *z)
{
    if (body == NULL)
        return;

    b3Pos p = b3Body_GetPosition(body->id);

    if (x != NULL)
        *x = b3Raw(p.x);
    if (y != NULL)
        *y = b3Raw(p.y);
    if (z != NULL)
        *z = b3Raw(p.z);
}

void NEA_Phys3DBodySetVelocityI(NEA_Phys3DBody *body,
                                int32_t x, int32_t y, int32_t z)
{
    if (body == NULL)
        return;

    b3Body_SetLinearVelocity(body->id,
                             b3MakeVec3(b3Makeb3f(x), b3Makeb3f(y), b3Makeb3f(z)));
}

void NEA_Phys3DBodyGetVelocityI(const NEA_Phys3DBody *body,
                                int32_t *x, int32_t *y, int32_t *z)
{
    if (body == NULL)
        return;

    b3Vec3 v = b3Body_GetLinearVelocity(body->id);

    if (x != NULL)
        *x = b3Raw(v.x);
    if (y != NULL)
        *y = b3Raw(v.y);
    if (z != NULL)
        *z = b3Raw(v.z);
}

void NEA_Phys3DBodyApplyForceI(NEA_Phys3DBody *body,
                               int32_t x, int32_t y, int32_t z)
{
    if (body == NULL)
        return;

    b3Body_ApplyForceToCenter(body->id,
                              b3MakeVec3(b3Makeb3f(x), b3Makeb3f(y), b3Makeb3f(z)),
                              true);
}

void NEA_Phys3DBodyApplyImpulseI(NEA_Phys3DBody *body,
                                 int32_t x, int32_t y, int32_t z)
{
    if (body == NULL)
        return;

    b3Body_ApplyLinearImpulseToCenter(
        body->id, b3MakeVec3(b3Makeb3f(x), b3Makeb3f(y), b3Makeb3f(z)), true);
}

void NEA_Phys3DBodyApplyTorqueI(NEA_Phys3DBody *body,
                                int32_t x, int32_t y, int32_t z)
{
    if (body == NULL)
        return;

    b3Body_ApplyTorque(body->id,
                       b3MakeVec3(b3Makeb3f(x), b3Makeb3f(y), b3Makeb3f(z)),
                       true);
}

void NEA_Phys3DBodySetMaterialI(NEA_Phys3DBody *body, int32_t friction,
                                int32_t restitution)
{
    if (body == NULL)
        return;

    // friction and restitution are b3c (Q30) rather than b3f: both live in
    // [0, 1] and the solver wants the extra bits. The caller speaks f32, so
    // convert rather than reinterpret.
    b3c f = b3FToCSat(b3Makeb3f(friction));
    b3c r = b3FToCSat(b3Makeb3f(restitution));

    b3ShapeId shapes[B3_MAX_HULL_FACES];
    int count = b3Body_GetShapes(body->id, shapes,
                                 (int)(sizeof(shapes) / sizeof(shapes[0])));

    for (int i = 0; i < count; i++)
    {
        b3Shape_SetFriction(shapes[i], f);
        b3Shape_SetRestitution(shapes[i], r);
    }
}

bool NEA_Phys3DBodyIsAwake(const NEA_Phys3DBody *body)
{
    if (body == NULL)
        return false;

    return b3Body_IsAwake(body->id);
}

void NEA_Phys3DBodySetAwake(NEA_Phys3DBody *body, bool awake)
{
    if (body == NULL)
        return;

    b3Body_SetAwake(body->id, awake);
}
