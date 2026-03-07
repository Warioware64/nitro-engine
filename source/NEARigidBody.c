// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Warioware64
//
// This file is part of Nitro Engine Advanced

/// @file   NEARigidBody.c
/// @brief  ARM9 rigid body API: FIFO command sender and state synchronization.

#include "NEAMain.h"
#include "NEARigidBody.h"

static NEA_RigidBody nea_rb_proxies[NEA_RB_MAX_BODIES];
static int nea_rb_static_next = 0;
static bool nea_rb_inited = false;

// =========================================================================
// Helpers
// =========================================================================

static inline void nea_rb_send_cmd(NEA_RB_Command cmd, uint32_t id)
{
    fifoSendValue32(NEA_RB_FIFO_CMD, NEA_RB_ENCODE_CMD(cmd, id));
}

static inline void nea_rb_send_val(int32_t val)
{
    fifoSendValue32(NEA_RB_FIFO_CMD, (u32)val);
}

// =========================================================================
// System
// =========================================================================

int NEA_RigidBodyInit(void)
{
    if (nea_rb_inited)
        NEA_RigidBodyEnd();

    for (int i = 0; i < NEA_RB_MAX_BODIES; i++)
    {
        nea_rb_proxies[i].active = false;
        nea_rb_proxies[i].model = NULL;
        nea_rb_proxies[i].sleeping = false;
    }
    nea_rb_static_next = 0;
    nea_rb_inited = true;

    // Tell ARM7 to start simulation
    nea_rb_send_cmd(NEA_RB_CMD_START, 0);

    return 0;
}

void NEA_RigidBodyEnd(void)
{
    if (!nea_rb_inited)
        return;

    nea_rb_send_cmd(NEA_RB_CMD_RESET, 0);

    for (int i = 0; i < NEA_RB_MAX_BODIES; i++)
        nea_rb_proxies[i].active = false;

    nea_rb_static_next = 0;
    nea_rb_inited = false;
}

void NEA_RigidBodySync(void)
{
    if (!nea_rb_inited)
        return;

    // Read all state packets from ARM7
    while (fifoCheckValue32(NEA_RB_FIFO_STATE))
    {
        u32 hdr = fifoGetValue32(NEA_RB_FIFO_STATE);

        // Check for end-of-frame sentinel
        if (hdr == NEA_RB_STATE_END)
            break;

        u32 id = NEA_RB_STATE_DECODE_ID(hdr);
        u32 flags = NEA_RB_STATE_DECODE_FLAGS(hdr);

        if (id >= NEA_RB_MAX_BODIES)
        {
            // Skip invalid body — drain remaining words
            for (int w = 0; w < NEA_RB_STATE_WORDS - 1; w++)
            {
                fifoWaitValue32(NEA_RB_FIFO_STATE);
                fifoGetValue32(NEA_RB_FIFO_STATE);
            }
            continue;
        }

        NEA_RigidBody *rb = &nea_rb_proxies[id];

        // Read position (3 words)
        fifoWaitValue32(NEA_RB_FIFO_STATE);
        rb->position.x = (int32_t)fifoGetValue32(NEA_RB_FIFO_STATE);
        fifoWaitValue32(NEA_RB_FIFO_STATE);
        rb->position.y = (int32_t)fifoGetValue32(NEA_RB_FIFO_STATE);
        fifoWaitValue32(NEA_RB_FIFO_STATE);
        rb->position.z = (int32_t)fifoGetValue32(NEA_RB_FIFO_STATE);

        // Read rotation matrix (9 words)
        for (int j = 0; j < 9; j++)
        {
            fifoWaitValue32(NEA_RB_FIFO_STATE);
            rb->rotation[j] = (int32_t)fifoGetValue32(NEA_RB_FIFO_STATE);
        }

        rb->sleeping = (flags & NEA_RB_STATE_FLAG_SLEEP) != 0;

        // Update linked model if present
        if (rb->active && rb->model != NULL)
        {
            NEA_Model *m = rb->model;

            // Also keep x/y/z updated for user queries
            m->x = rb->position.x;
            m->y = rb->position.y;
            m->z = rb->position.z;

            // Build a m4x3 matrix from rotation + position.
            // m4x3 is column-major with int m[12]:
            //   Col 0: m[0..2], Col 1: m[3..5], Col 2: m[6..8]
            //   Translation: m[9..11]
            // Our rotation is row-major:
            //   rb->rotation[0..2] = row 0
            //   rb->rotation[3..5] = row 1
            //   rb->rotation[6..8] = row 2

            m4x3 mat;
            // Column 0
            mat.m[0]  = rb->rotation[0];
            mat.m[1]  = rb->rotation[3];
            mat.m[2]  = rb->rotation[6];
            // Column 1
            mat.m[3]  = rb->rotation[1];
            mat.m[4]  = rb->rotation[4];
            mat.m[5]  = rb->rotation[7];
            // Column 2
            mat.m[6]  = rb->rotation[2];
            mat.m[7]  = rb->rotation[5];
            mat.m[8]  = rb->rotation[8];
            // Translation
            mat.m[9]  = rb->position.x;
            mat.m[10] = rb->position.y;
            mat.m[11] = rb->position.z;

            NEA_ModelSetMatrix(m, &mat);
        }
    }
}

// =========================================================================
// Body creation / destruction
// =========================================================================

NEA_RigidBody *NEA_RigidBodyCreateI(int32_t mass,
                                     int32_t hx, int32_t hy, int32_t hz)
{
    if (!nea_rb_inited)
    {
        NEA_DebugPrint("RigidBody system not initialized");
        return NULL;
    }

    // Find free proxy slot
    int slot = -1;
    for (int i = 0; i < NEA_RB_MAX_BODIES; i++)
    {
        if (!nea_rb_proxies[i].active)
        {
            slot = i;
            break;
        }
    }

    if (slot < 0)
    {
        NEA_DebugPrint("No free rigid body slots");
        return NULL;
    }

    NEA_RigidBody *rb = &nea_rb_proxies[slot];
    rb->id = (uint8_t)slot;
    rb->active = true;
    rb->model = NULL;
    rb->position = NEA_Vec3Make(0, 0, 0);
    rb->sleeping = false;
    for (int i = 0; i < 9; i++)
        rb->rotation[i] = 0;
    rb->rotation[0] = rb->rotation[4] = rb->rotation[8] = 1 << 12;

    // Cache half-extents for collision bridge
    rb->half_extents[0] = hx;
    rb->half_extents[1] = hy;
    rb->half_extents[2] = hz;

    // Send ADD_BODY command to ARM7
    nea_rb_send_cmd(NEA_RB_CMD_ADD_BODY, (uint32_t)slot);
    nea_rb_send_val(hx);
    nea_rb_send_val(hy);
    nea_rb_send_val(hz);
    nea_rb_send_val(mass);
    nea_rb_send_val(0); // px
    nea_rb_send_val(0); // py
    nea_rb_send_val(0); // pz

    return rb;
}

void NEA_RigidBodyDelete(NEA_RigidBody *rb)
{
    if (rb == NULL || !rb->active)
        return;

    nea_rb_send_cmd(NEA_RB_CMD_KILL_BODY, (uint32_t)rb->id);
    rb->active = false;
    rb->model = NULL;
}

// =========================================================================
// Configuration
// =========================================================================

void NEA_RigidBodySetModel(NEA_RigidBody *rb, NEA_Model *model)
{
    NEA_AssertPointer(rb, "NULL rb");
    NEA_AssertPointer(model, "NULL model");
    rb->model = model;
}

void NEA_RigidBodySetPositionI(NEA_RigidBody *rb,
                                int32_t x, int32_t y, int32_t z)
{
    NEA_AssertPointer(rb, "NULL rb");
    if (!rb->active)
        return;

    nea_rb_send_cmd(NEA_RB_CMD_SET_POSITION, (uint32_t)rb->id);
    nea_rb_send_val(x);
    nea_rb_send_val(y);
    nea_rb_send_val(z);

    // Also update local cache so model moves immediately
    rb->position = NEA_Vec3Make(x, y, z);
    if (rb->model)
    {
        rb->model->x = x;
        rb->model->y = y;
        rb->model->z = z;
    }
}

void NEA_RigidBodySetVelocityI(NEA_RigidBody *rb,
                                int32_t vx, int32_t vy, int32_t vz)
{
    NEA_AssertPointer(rb, "NULL rb");
    if (!rb->active)
        return;

    nea_rb_send_cmd(NEA_RB_CMD_SET_VELOCITY, (uint32_t)rb->id);
    nea_rb_send_val(vx);
    nea_rb_send_val(vy);
    nea_rb_send_val(vz);
}

void NEA_RigidBodySetRestitutionI(NEA_RigidBody *rb, int32_t e)
{
    NEA_AssertPointer(rb, "NULL rb");
    if (!rb->active)
        return;

    nea_rb_send_cmd(NEA_RB_CMD_SET_RESTITUTION, (uint32_t)rb->id);
    nea_rb_send_val(e);
}

void NEA_RigidBodySetFrictionI(NEA_RigidBody *rb, int32_t mu)
{
    NEA_AssertPointer(rb, "NULL rb");
    if (!rb->active)
        return;

    nea_rb_send_cmd(NEA_RB_CMD_SET_FRICTION, (uint32_t)rb->id);
    nea_rb_send_val(mu);
}

// =========================================================================
// Forces and impulses
// =========================================================================

void NEA_RigidBodyApplyForceI(NEA_RigidBody *rb,
                               int32_t fx, int32_t fy, int32_t fz,
                               int32_t px, int32_t py, int32_t pz)
{
    NEA_AssertPointer(rb, "NULL rb");
    if (!rb->active)
        return;

    nea_rb_send_cmd(NEA_RB_CMD_APPLY_FORCE, (uint32_t)rb->id);
    nea_rb_send_val(fx);
    nea_rb_send_val(fy);
    nea_rb_send_val(fz);
    nea_rb_send_val(px);
    nea_rb_send_val(py);
    nea_rb_send_val(pz);
}

void NEA_RigidBodyApplyImpulseI(NEA_RigidBody *rb,
                                 int32_t jx, int32_t jy, int32_t jz,
                                 int32_t px, int32_t py, int32_t pz)
{
    NEA_AssertPointer(rb, "NULL rb");
    if (!rb->active)
        return;

    nea_rb_send_cmd(NEA_RB_CMD_APPLY_IMPULSE, (uint32_t)rb->id);
    nea_rb_send_val(jx);
    nea_rb_send_val(jy);
    nea_rb_send_val(jz);
    nea_rb_send_val(px);
    nea_rb_send_val(py);
    nea_rb_send_val(pz);
}

// =========================================================================
// Global settings
// =========================================================================

void NEA_RigidBodySetGravityI(int32_t g)
{
    nea_rb_send_cmd(NEA_RB_CMD_SET_GRAVITY, 0);
    nea_rb_send_val(g);
}

// =========================================================================
// Static colliders
// =========================================================================

int NEA_RigidBodyAddStaticI(int32_t px, int32_t py, int32_t pz,
                             int32_t sx, int32_t sy, int32_t sz,
                             int32_t nx, int32_t ny, int32_t nz)
{
    if (nea_rb_static_next >= NEA_RB_MAX_STATICS)
    {
        NEA_DebugPrint("No free static collider slots");
        return -1;
    }

    int id = nea_rb_static_next++;

    nea_rb_send_cmd(NEA_RB_CMD_ADD_STATIC, (uint32_t)id);
    nea_rb_send_val(px);
    nea_rb_send_val(py);
    nea_rb_send_val(pz);
    nea_rb_send_val(sx);
    nea_rb_send_val(sy);
    nea_rb_send_val(sz);
    nea_rb_send_val(nx);
    nea_rb_send_val(ny);
    nea_rb_send_val(nz);

    return id;
}

void NEA_RigidBodyRemoveStatic(int id)
{
    if (id < 0 || id >= NEA_RB_MAX_STATICS)
        return;

    nea_rb_send_cmd(NEA_RB_CMD_REMOVE_STATIC, (uint32_t)id);
}

// =========================================================================
// Queries
// =========================================================================

NEA_Vec3 NEA_RigidBodyGetPosition(const NEA_RigidBody *rb)
{
    NEA_AssertPointer(rb, "NULL rb");
    return rb->position;
}

bool NEA_RigidBodyIsSleeping(const NEA_RigidBody *rb)
{
    NEA_AssertPointer(rb, "NULL rb");
    return rb->sleeping;
}

// =========================================================================
// Collision system integration
// =========================================================================

int NEA_RigidBodyCollideWithI(NEA_RigidBody *rb,
                               const NEA_ColShape *shape,
                               int32_t sx, int32_t sy, int32_t sz)
{
    NEA_AssertPointer(rb, "NULL rb");
    NEA_AssertPointer(shape, "NULL shape");

    if (!rb->active || !nea_rb_inited)
        return 0;

    // Construct AABB from rigid body's position + half-extents
    NEA_ColShape rb_shape;
    rb_shape.type = NEA_COL_AABB;
    rb_shape.shape.aabb.half = NEA_Vec3Make(
        rb->half_extents[0], rb->half_extents[1], rb->half_extents[2]);

    NEA_Vec3 rb_pos = rb->position;
    NEA_Vec3 shape_pos = NEA_Vec3Make(sx, sy, sz);

    // Test collision
    NEA_ColResult result = NEA_ColTest(&rb_shape, rb_pos, shape, shape_pos);

    if (!result.hit || result.depth <= 0)
        return 0;

    // Inject contact into ARM7 via FIFO
    nea_rb_send_cmd(NEA_RB_CMD_INJECT_CONTACT, (uint32_t)rb->id);
    nea_rb_send_val(result.normal.x);
    nea_rb_send_val(result.normal.y);
    nea_rb_send_val(result.normal.z);
    nea_rb_send_val(result.point.x);
    nea_rb_send_val(result.point.y);
    nea_rb_send_val(result.point.z);
    nea_rb_send_val(result.depth);

    return 1;
}
