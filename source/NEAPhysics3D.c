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
// Queries
// =========================================================================

bool NEA_Phys3DRayCastI(int32_t ox, int32_t oy, int32_t oz,
                        int32_t dx, int32_t dy, int32_t dz,
                        NEA_Phys3DRayHit *out)
{
    if (out == NULL)
        return false;

    memset(out, 0, sizeof(*out));
    out->triangleIndex = -1;

    if (nea_worldExists == false)
        return false;

    b3Vec3 origin = b3MakeVec3(b3Makeb3f(ox), b3Makeb3f(oy), b3Makeb3f(oz));
    b3Vec3 translation = b3MakeVec3(b3Makeb3f(dx), b3Makeb3f(dy), b3Makeb3f(dz));

    b3RayResult result = b3World_CastRayClosest(nea_worldId, origin, translation,
                                                b3DefaultQueryFilter());

    // Reported whether or not anything was hit: a ray that finds nothing still
    // cost a traversal, and that is exactly the case worth being able to see.
    out->nodeVisits = result.nodeVisits;
    out->leafVisits = result.leafVisits;

    if (result.hit == false)
        return false;

    out->hit = true;
    out->x = b3Raw(result.point.x);
    out->y = b3Raw(result.point.y);
    out->z = b3Raw(result.point.z);
    out->nx = b3Raw(result.normal.x);
    out->ny = b3Raw(result.normal.y);
    out->nz = b3Raw(result.normal.z);

    // b3c is Q30 and this module speaks f32 (Q12), so the fraction narrows
    // here rather than being handed out at a scale nothing else in the header
    // uses. A fraction is in [0, 1], so the 18 bits dropped are below the
    // precision anything f32 could act on anyway.
    out->fraction = b3Raw(b3CToF(result.fraction));

    out->userMaterialId = result.userMaterialId;
    out->triangleIndex = result.triangleIndex;

    return true;
}

typedef struct {
    NEA_Phys3DOverlapFn fn;
    void *context;
    int count;
} nea_phys3d_overlap_ctx;

static bool nea_phys3d_overlap_cb(b3ShapeId shapeId, void *context)
{
    nea_phys3d_overlap_ctx *ctx = context;

    // Every body this module creates installs its NEA_Phys3DBody as the Box3D
    // body's user data, which is the same hop NEA_Phys3DSyncModels makes from a
    // move event. A body created through the raw b3 API has none, and is
    // skipped rather than reported as NULL.
    NEA_Phys3DBody *body = b3Body_GetUserData(b3Shape_GetBody(shapeId));
    if (body == NULL)
        return true;

    ctx->count++;
    return ctx->fn(body, ctx->context);
}

int NEA_Phys3DOverlapBoxI(int32_t lx, int32_t ly, int32_t lz,
                          int32_t ux, int32_t uy, int32_t uz,
                          NEA_Phys3DOverlapFn fn, void *context)
{
    if (nea_worldExists == false || fn == NULL)
        return 0;

    // A caller who passes the corners the other way round gets an empty box
    // from b3MakeAABB and no hits, which reads as "nothing there" rather than
    // as a bug. Normalize instead -- it costs six compares once per query.
    b3Vec3 a = b3MakeVec3(b3Makeb3f(lx), b3Makeb3f(ly), b3Makeb3f(lz));
    b3Vec3 b = b3MakeVec3(b3Makeb3f(ux), b3Makeb3f(uy), b3Makeb3f(uz));
    b3AABB aabb = b3MakeAABB(b3Min(a, b), b3Max(a, b));

    nea_phys3d_overlap_ctx ctx = { fn, context, 0 };
    b3World_OverlapAABB(nea_worldId, aabb, b3DefaultQueryFilter(),
                        nea_phys3d_overlap_cb, &ctx);

    return ctx.count;
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

// =========================================================================
// Character mover
// =========================================================================
//
// Upstream's samples/mover.cpp, in this module's conventions. Seven steps per
// frame, and the order is the whole design:
//
//   1. friction, applied to the horizontal velocity only;
//   2. accelerate toward what the throttles ask for, clamped to maxSpeed;
//   3. gravity;
//   4. the ground probe, which is a ray and a soft spring;
//   5. the slide loop -- collect planes, solve, sweep, advance, up to five
//      times;
//   6. push the dynamic bodies the planes named;
//   7. fix up the velocity, either by clipping it against the planes or by
//      deriving it from the distance actually covered.
//
// @section fixed What Q12 changed
//
// Three things, each commented at its site: the ground spring's coefficients
// fold to two compile-time constants because the timestep is fixed, the
// slide-loop's break test has to be taken wide because its threshold squared
// rounds to zero in Q12, and the friction ratio goes through b3DivFFToC
// because it is a genuine ratio and Q30 keeps eighteen more bits of it.
//
// @section absent What is not here
//
// Upstream's per-shape MoverShapeUserData -- a maxPush and a clipVelocity read
// back from each shape's user data, so a game can make particular surfaces
// soft. Every plane here is rigid (pushLimit B3_F_MAX) and clipVelocity comes
// from the def. Adding it needs a place to put per-shape user data that
// NEA_Phys3DBodyAdd*I does not currently offer, which is a shape-API question
// rather than a mover one.

/// The mover's slide loop iteration cap. Upstream's number.
#define NEA_PHYS3D_MOVER_SLIDE_ITERATIONS 5

/// Squared distance below which the slide loop stops, in raw Q12 units.
///
/// Upstream tests `b3LengthSquared(delta) < 0.01f * 0.01f`. Transliterating
/// that is a silent bug: 0.0001 in Q12 rounds to **zero**, the test becomes
/// `x < 0`, and the loop runs all five iterations forever. Taken wide instead,
/// where 0.01 units is 41 raw and the comparison happens at Q24.
#define NEA_PHYS3D_MOVER_TOLERANCE_SQ ( (int64_t)41 * 41 )

// The ground spring, folded to two constants because the timestep is fixed at
// compile time -- which is the reason the port fixed it at all. Upstream
// recomputes omega, omegaH and the denominator every frame from `h`:
//
//   omega  = 2*pi*hertz,  omegaH = omega*h
//   denom  = 1 + 2*zeta*omegaH + omegaH^2
//   v      = (v - omega*omegaH*(length - rest)) / denom
//
// With zeta 0.7, hertz 4 and h = 1/60 that is v*massCoef - biasCoef*(length -
// rest), and there is no per-frame divide at all.
//
//   omega*omegaH / denom = 10.527578 / 1.76189026 = 5.975270  [1/s]
//   1 / denom            =                          0.5675727 [-]
//
// biasCoef is a b3f and not a b3c because it is 5.975 -- outside Q30's [-1, 1]
// range, and dimensionally a rate rather than a coefficient. massCoef is a b3c
// and not a b3f because it is a true dimensionless ratio, and Q30 keeps
// eighteen more bits of it than Q12 would.
//
// Resolution: one raw unit of length error produces 24475 >> 12 = 5 raw of
// velocity, so the spring resolves length at the Q12 quantum with 5x headroom.
// The equilibrium compression against gravity is gravity*h/biasCoef = 0.042
// units, or 171 raw.
_Static_assert(B3_NEA_STEP_HZ == 60,
               "the pogo constants below are precomputed for a 1/60 timestep");

#define NEA_PHYS3D_POGO_BIAS b3Makeb3f(24475)       // 5.975270 * 4096
#define NEA_PHYS3D_POGO_MASS b3Makeb3c(609426868)   // 0.5675727 * 2^30

/// Rest length of the ground spring, as a multiple of the capsule radius.
#define NEA_PHYS3D_POGO_REST 3

/// Probe length, as a multiple of the capsule radius. One radius further than
/// the rest length, which is the slack that lets the mover find ground it has
/// stepped off.
#define NEA_PHYS3D_POGO_REACH 4

NEA_Phys3DMoverDef NEA_Phys3DDefaultMoverDef(void)
{
    NEA_Phys3DMoverDef def;
    memset(&def, 0, sizeof(def));

    def.radius = floattof32(0.3f);
    def.halfHeight = floattof32(0.5f);
    def.maxSpeed = floattof32(6.0f);
    def.minSpeed = floattof32(0.01f);
    def.stopSpeed = floattof32(1.0f);
    def.accelerate = floattof32(30.0f);
    def.friction = floattof32(4.0f);
    def.gravity = floattof32(15.0f);
    def.jumpSpeed = floattof32(5.0f);
    def.pogo = true;
    def.clipVelocity = true;

    b3QueryFilter filter = b3DefaultQueryFilter();
    def.categoryBits = filter.categoryBits;
    def.maskBits = filter.maskBits;

    def.modelOffsetY = -(def.halfHeight + def.radius);

    return def;
}

void NEA_Phys3DMoverInitI(NEA_Phys3DMover *mover, const NEA_Phys3DMoverDef *def,
                          int32_t x, int32_t y, int32_t z)
{
    NEA_AssertPointer(mover, "NULL mover");

    memset(mover, 0, sizeof(*mover));

    mover->def = (def != NULL) ? *def : NEA_Phys3DDefaultMoverDef();
    mover->x = x;
    mover->y = y;
    mover->z = z;
}

void NEA_Phys3DMoverSetPositionI(NEA_Phys3DMover *mover, int32_t x, int32_t y, int32_t z)
{
    if (mover == NULL)
        return;

    mover->x = x;
    mover->y = y;
    mover->z = z;

    // The old ground contact says nothing about the new position, and a spring
    // velocity carried across a teleport launches the mover on the next step.
    mover->onGround = false;
    mover->pogoVelocity = 0;
}

void NEA_Phys3DMoverSetVelocityI(NEA_Phys3DMover *mover, int32_t x, int32_t y, int32_t z)
{
    if (mover == NULL)
        return;

    mover->vx = x;
    mover->vy = y;
    mover->vz = z;
}

void NEA_Phys3DMoverSetModel(NEA_Phys3DMover *mover, NEA_Model *model)
{
    NEA_AssertPointer(mover, "NULL mover");

    mover->model = model;
}

void NEA_Phys3DMoverIgnoreShape(NEA_Phys3DMover *mover, b3ShapeId shapeId)
{
    if (mover == NULL || mover->ignoreCount >= NEA_PHYS3D_MOVER_MAX_IGNORE)
        return;

    mover->ignore[mover->ignoreCount++] = shapeId;
}

void NEA_Phys3DMoverClearIgnored(NEA_Phys3DMover *mover)
{
    if (mover == NULL)
        return;

    mover->ignoreCount = 0;
}

bool NEA_Phys3DMoverJump(NEA_Phys3DMover *mover)
{
    if (mover == NULL || mover->onGround == false)
        return false;

    mover->vy = mover->def.jumpSpeed;
    mover->onGround = false;

    // Zeroed so the spring does not spend the first frames of the jump pulling
    // the mover back down toward the ground it just left.
    mover->pogoVelocity = 0;
    return true;
}

/// Is this shape on the mover's ignore list?
static bool nea_phys3d_mover_ignores(const NEA_Phys3DMover *mover, b3ShapeId shapeId)
{
    for (int i = 0; i < mover->ignoreCount; i++)
    {
        if (mover->ignore[i].index1 == shapeId.index1 &&
            mover->ignore[i].world0 == shapeId.world0 &&
            mover->ignore[i].generation == shapeId.generation)
        {
            return true;
        }
    }

    return false;
}

/// Implements b3PlaneResultFcn.
static bool nea_phys3d_mover_planes(b3ShapeId shapeId, const b3PlaneResult *planes,
                                    int planeCount, void *context)
{
    NEA_Phys3DMover *mover = context;

    if (nea_phys3d_mover_ignores(mover, shapeId))
        return true;

    for (int i = 0; i < planeCount; i++)
    {
        if (mover->planeCount >= B3_NEA_MAX_MOVER_PLANES)
            return false;

        // A zero normal would be accepted by b3SolvePlanes and then push
        // nothing forever, so it is worth an assert rather than a silent slot.
        // Every backend either produces a unit normal or produces no plane.
        B3_ASSERT(b3IsNormalized(planes[i].plane.normal));

        b3CollisionPlane *plane = &mover->planes[mover->planeCount];
        plane->plane = planes[i].plane;
        plane->pushLimit = B3_F_MAX;
        plane->push = b3f_zero;
        plane->clipVelocity = mover->def.clipVelocity;

        mover->planeShapes[mover->planeCount] = shapeId;
        mover->planePoints[mover->planeCount] = planes[i].point;
        mover->planeCount += 1;
    }

    return true;
}

/// Implements b3MoverFilterFcn.
static bool nea_phys3d_mover_cast_filter(b3ShapeId shapeId, void *context)
{
    return nea_phys3d_mover_ignores((const NEA_Phys3DMover *)context, shapeId) == false;
}

/// The mover's capsule at its current position, in world space.
static b3Capsule nea_phys3d_mover_capsule(const NEA_Phys3DMover *mover)
{
    b3f half = b3Makeb3f(mover->def.halfHeight);
    b3Vec3 centre = b3MakeVec3(b3Makeb3f(mover->x), b3Makeb3f(mover->y), b3Makeb3f(mover->z));

    b3Capsule capsule;
    capsule.center1 = b3MakeVec3(centre.x, b3SubF(centre.y, half), centre.z);
    capsule.center2 = b3MakeVec3(centre.x, b3AddF(centre.y, half), centre.z);
    capsule.radius = b3Makeb3f(mover->def.radius);
    return capsule;
}

static b3QueryFilter nea_phys3d_mover_filter(const NEA_Phys3DMover *mover)
{
    b3QueryFilter filter = b3DefaultQueryFilter();
    filter.categoryBits = mover->def.categoryBits;
    filter.maskBits = mover->def.maskBits;
    return filter;
}

/// Push the mover's model matrix, which nothing else will.
///
/// NEA_Phys3DSyncModels walks the step's move events; a mover has no body and
/// produces none, so it drives its own model. A yaw rotation about Y plus a
/// translation, through the same NEA_ModelSetMatrix path bodies use.
static void nea_phys3d_mover_sync_model(NEA_Phys3DMover *mover)
{
    NEA_Model *m = mover->model;
    if (m == NULL)
        return;

    int32_t s = sinLerp(mover->yaw);
    int32_t c = cosLerp(mover->yaw);

    int32_t px = mover->x;
    int32_t py = mover->y + mover->def.modelOffsetY;
    int32_t pz = mover->z;

    m->x = px;
    m->y = py;
    m->z = pz;

    // The model's own scale folded into the rotation columns, exactly as
    // nea_phys3d_apply_transform does and for the same reason.
    m4x3 mat;
    mat.m[0]  = mulf32(c, m->sx);
    mat.m[1]  = 0;
    mat.m[2]  = mulf32(-s, m->sx);
    mat.m[3]  = 0;
    mat.m[4]  = m->sy;
    mat.m[5]  = 0;
    mat.m[6]  = mulf32(s, m->sz);
    mat.m[7]  = 0;
    mat.m[8]  = mulf32(c, m->sz);
    mat.m[9]  = px;
    mat.m[10] = py;
    mat.m[11] = pz;

    NEA_ModelSetMatrix(m, &mat);
}

void NEA_Phys3DMoverStepI(NEA_Phys3DMover *mover,
                          int32_t fx, int32_t fy, int32_t fz,
                          int32_t rx, int32_t ry, int32_t rz,
                          int32_t forwardThrottle, int32_t strafeThrottle)
{
    NEA_AssertPointer(mover, "NULL mover");

    if (!nea_worldExists)
        return;

    // dt as a b3f. The timestep is fixed, so this is a constant.
    const b3f dt = b3fFromFrac(1, B3_NEA_STEP_HZ);

    b3Vec3 velocity = b3MakeVec3(b3Makeb3f(mover->vx), b3Makeb3f(mover->vy), b3Makeb3f(mover->vz));
    b3Vec3 position = b3MakeVec3(b3Makeb3f(mover->x), b3Makeb3f(mover->y), b3Makeb3f(mover->z));
    b3Vec3 startPosition = position;

    b3f radius = b3Makeb3f(mover->def.radius);
    b3QueryFilter filter = nea_phys3d_mover_filter(mover);

    // ---- 1. Friction, horizontal only ----
    {
        b3Vec3 flat = b3MakeVec3(velocity.x, b3f_zero, velocity.z);
        b3f speed = b3Length(flat);

        if (b3Raw(speed) < b3Raw(b3Makeb3f(mover->def.minSpeed)))
        {
            velocity.x = b3f_zero;
            velocity.z = b3f_zero;
        }
        else
        {
            b3f control = b3MaxFloat(speed, b3Makeb3f(mover->def.stopSpeed));
            b3f drop = b3MulFF(b3MulFF(control, b3Makeb3f(mover->def.friction)), dt);
            b3f newSpeed = b3MaxFloat(b3f_zero, b3SubF(speed, drop));

            // A genuine dimensionless ratio in [0, 1], so b3DivFFToC and not
            // b3DivFF: the quotient is usually 0.98, and Q30 keeps eighteen
            // more bits of it than Q12 would.
            b3c ratio = b3DivFFToC(newSpeed, speed);
            velocity.x = b3MulFC(velocity.x, ratio);
            velocity.z = b3MulFC(velocity.z, ratio);
        }
    }

    // ---- 2. Accelerate ----
    {
        // Only the horizontal part of the basis steers, and neither vector has
        // to arrive unit length.
        b3Vec3 forward = b3MakeVec3(b3Makeb3f(fx), b3f_zero, b3Makeb3f(fz));
        b3Vec3 right = b3MakeVec3(b3Makeb3f(rx), b3f_zero, b3Makeb3f(rz));
        B3_UNUSED(fy, ry);

        forward = b3Normalize(forward);
        right = b3Normalize(right);

        b3Vec3 wish = b3Add(b3MulSV(b3Makeb3f(forwardThrottle), forward),
                            b3MulSV(b3Makeb3f(strafeThrottle), right));

        b3f wishSpeed;
        b3Vec3 wishDir = b3GetLengthAndNormalize(&wishSpeed, wish);

        b3f maxSpeed = b3Makeb3f(mover->def.maxSpeed);
        wishSpeed = b3MulFF(b3MinFloat(wishSpeed, b3f_one), maxSpeed);

        if (b3Raw(wishSpeed) > 0)
        {
            // How much of the wanted speed is missing along the wanted
            // direction. Quake's formulation, and the reason a mover cannot
            // accelerate past maxSpeed by holding two directions at once.
            b3f current = b3Dot(velocity, wishDir);
            b3f add = b3SubF(wishSpeed, current);

            if (b3Raw(add) > 0)
            {
                b3f accel = b3MulFF(b3MulFF(b3Makeb3f(mover->def.accelerate), maxSpeed), dt);
                velocity = b3MulAdd(velocity, b3MinFloat(accel, add), wishDir);
            }
        }
    }

    // ---- 3. Gravity ----
    velocity.y = b3SubF(velocity.y, b3MulFF(b3Makeb3f(mover->def.gravity), dt));

    // ---- 4. The ground probe ----
    //
    // A ray straight down from the lower cap centre, driving a soft spring
    // toward a rest length of three radii. This is what rides stairs and slopes
    // without a step-up hack: a 0.2-unit step is a 0.2-unit compression.
    //
    // It also keeps the capsule off the floor entirely, which matters more here
    // than upstream. Resting *on* the floor means a floor plane every frame and
    // a solver push of B3_LINEAR_SLOP -- a real 0.0049 units in Q12, 0.29
    // units per second at 60 Hz, injected into the velocity forever.
    if (mover->def.pogo)
    {
        b3Capsule probeCapsule = nea_phys3d_mover_capsule(mover);
        b3f reach = b3MulFF(radius, b3fFromInt(NEA_PHYS3D_POGO_REACH));
        b3f rest = b3MulFF(radius, b3fFromInt(NEA_PHYS3D_POGO_REST));

        b3RayResult ground = b3World_CastRayClosest(nea_worldId, probeCapsule.center1,
                                                    b3MakeVec3(b3f_zero, b3NegF(reach), b3f_zero),
                                                    filter);

        // Rising, or nothing under us: no ground, and the spring is released
        // rather than left holding a stale compression.
        if (ground.hit == false || b3Raw(velocity.y) > 0)
        {
            mover->onGround = false;
            mover->pogoVelocity = 0;
        }
        else
        {
            // b3RayResult::fraction is b3c, so this is Q12 x Q30 -> Q12 with
            // no conversion on either side.
            b3f length = b3MulFC(reach, ground.fraction);

            b3f pogo = b3Makeb3f(mover->pogoVelocity);
            pogo = b3SubF(b3MulFC(pogo, NEA_PHYS3D_POGO_MASS),
                          b3MulFF(NEA_PHYS3D_POGO_BIAS, b3SubF(length, rest)));

            mover->pogoVelocity = b3Raw(pogo);
            mover->onGround = true;

            // The spring drives the mover, not the velocity: writing it into
            // velocity.y would have it fight gravity and friction both.
            position.y = b3AddF(position.y, b3MulFF(pogo, dt));
            velocity.y = b3f_zero;
        }
    }
    else
    {
        mover->onGround = false;
    }

    // ---- 5. The slide loop ----
    mover->solverIterations = 0;
    mover->castIterations = 0;

    b3Vec3 target = b3MulAdd(position, dt, velocity);

    for (int iteration = 0; iteration < NEA_PHYS3D_MOVER_SLIDE_ITERATIONS; iteration++)
    {
        mover->castIterations = iteration + 1;

        // Collect the planes where the mover is *now*. Their offsets are
        // depths relative to this position, which is why this has to be redone
        // every iteration rather than hoisted.
        mover->x = b3Raw(position.x);
        mover->y = b3Raw(position.y);
        mover->z = b3Raw(position.z);

        b3Capsule capsule = nea_phys3d_mover_capsule(mover);

        mover->planeCount = 0;
        b3World_CollideMover(nea_worldId, &capsule, filter, nea_phys3d_mover_planes, mover);

        b3PlaneSolverResult solved = b3SolvePlanes(b3Sub(target, position), mover->planes,
                                                   mover->planeCount);
        mover->solverIterations += solved.iterationCount;

        b3c fraction = b3World_CastMover(nea_worldId, &capsule, solved.delta, filter,
                                         nea_phys3d_mover_cast_filter, mover);

        b3Vec3 delta = b3MulCV(fraction, solved.delta);
        position = b3Add(position, delta);

        // Wide, and that is not a style choice: 0.01 squared is 1e-4, which in
        // Q12 rounds to zero, so the narrow spelling never terminates and the
        // loop always runs its full five.
        if (b3LengthSquaredWide(delta) < NEA_PHYS3D_MOVER_TOLERANCE_SQ)
            break;
    }

    mover->x = b3Raw(position.x);
    mover->y = b3Raw(position.y);
    mover->z = b3Raw(position.z);

    // ---- 6. Push the dynamic bodies we leaned on ----
    for (int i = 0; i < mover->planeCount; i++)
    {
        b3BodyId bodyId = b3Shape_GetBody(mover->planeShapes[i]);
        if (b3Body_GetType(bodyId) != b3_dynamicBody)
            continue;

        // Only planes the solver actually pushed against: a plane it never
        // used describes a surface the mover is not leaning on.
        if (b3Raw(mover->planes[i].push) == 0)
            continue;

        b3Vec3 normal = mover->planes[i].plane.normal;
        b3f approach = b3Dot(velocity, normal);
        if (b3Raw(approach) >= 0)
            continue;

        // A crude but stable exchange: shove the body along the plane normal
        // in proportion to how hard the mover is moving into it. Upstream
        // computes a proper normal mass from the body's inverse mass and world
        // inverse inertia; that needs b3Body_GetWorldInverseRotationalInertia,
        // which the port does not expose, and the visible difference on a crate
        // is not worth the surface.
        b3f mass = b3f_one;
        b3f inverseMass = b3Body_GetMass(bodyId);
        if (b3Raw(inverseMass) > 0)
            mass = inverseMass;

        b3Vec3 impulse = b3MulSV(b3MulFF(b3NegF(approach), mass), normal);
        b3Body_ApplyLinearImpulse(bodyId, impulse, mover->planePoints[i], true);

        // And take it out of the mover, so pushing a heavy crate slows it.
        velocity = b3MulSub(velocity, b3NegF(approach), normal);
    }

    // ---- 7. Velocity fix-up ----
    if (mover->def.clipVelocity)
    {
        // Clip rather than derive, so the mover does not inherit the
        // depenetration push as speed and launch off what it was resting on.
        velocity = b3ClipVector(velocity, mover->planes, mover->planeCount);
    }
    else
    {
        // The honest alternative: whatever distance was actually covered, over
        // the step. Costs a divide, and it does pick up depenetration.
        b3Vec3 moved = b3Sub(position, startPosition);
        velocity = b3MakeVec3(b3DivFF(moved.x, dt), b3DivFF(moved.y, dt), b3DivFF(moved.z, dt));
    }

    mover->vx = b3Raw(velocity.x);
    mover->vy = b3Raw(velocity.y);
    mover->vz = b3Raw(velocity.z);

    nea_phys3d_mover_sync_model(mover);
}

void NEA_Phys3DMoverStepYawI(NEA_Phys3DMover *mover, int yaw,
                             int32_t forwardThrottle, int32_t strafeThrottle)
{
    NEA_AssertPointer(mover, "NULL mover");

    mover->yaw = (int16_t)yaw;

    // Two table lookups, exactly unit length by construction, no normalize and
    // no divide -- which is why this and not the explicit-basis form is what a
    // DS character should steer by. right = forward rotated -90 degrees about
    // Y, so that positive strafe is to the player's right.
    int32_t s = sinLerp((int16_t)yaw);
    int32_t c = cosLerp((int16_t)yaw);

    NEA_Phys3DMoverStepI(mover, s, 0, c, c, 0, -s, forwardThrottle, strafeThrottle);
}

int NEA_Phys3DWorldGetMoverPlaneDropCount(void)
{
    if (!nea_worldExists)
        return 0;

    b3World *world = b3GetWorldFromId(nea_worldId);
    return world->moverPlaneDropCount;
}

int NEA_Phys3DWorldGetTeleportCount(void)
{
    if (!nea_worldExists)
        return 0;

    b3World *world = b3GetWorldFromId(nea_worldId);
    return world->teleportCount;
}
