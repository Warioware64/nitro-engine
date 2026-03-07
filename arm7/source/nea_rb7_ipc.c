// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Warioware64
//
// This file is part of Nitro Engine Advanced

/// @file   nea_rb7_ipc.c
/// @brief  ARM7 FIFO command listener and state sender for rigid body physics.

#include "nea_rb7.h"

// =========================================================================
// Helper: wait for next FIFO value (blocking)
// =========================================================================

static inline u32 nea_rb7_fifo_wait(void)
{
    while (!fifoCheckValue32(NEA_RB_FIFO_CMD))
        ;
    return fifoGetValue32(NEA_RB_FIFO_CMD);
}

// =========================================================================
// Command listener (ARM9 -> ARM7)
// =========================================================================

ARM_CODE void nea_rb7_listen(void)
{
    while (fifoCheckValue32(NEA_RB_FIFO_CMD))
    {
        u32 signal = fifoGetValue32(NEA_RB_FIFO_CMD);
        u32 cmd = NEA_RB_DECODE_CMD(signal);
        u32 id  = NEA_RB_DECODE_ID(signal);

        switch (cmd)
        {
        case NEA_RB_CMD_START:
            nea_rb7_running = true;
            break;

        case NEA_RB_CMD_PAUSE:
            nea_rb7_running = false;
            break;

        case NEA_RB_CMD_RESET:
            nea_rb7_reset();
            break;

        case NEA_RB_CMD_ADD_BODY:
        {
            // Read 7 data words: hx, hy, hz, mass, px, py, pz
            int32_t hx   = (int32_t)nea_rb7_fifo_wait();
            int32_t hy   = (int32_t)nea_rb7_fifo_wait();
            int32_t hz   = (int32_t)nea_rb7_fifo_wait();
            int32_t mass = (int32_t)nea_rb7_fifo_wait();
            int32_t px   = (int32_t)nea_rb7_fifo_wait();
            int32_t py   = (int32_t)nea_rb7_fifo_wait();
            int32_t pz   = (int32_t)nea_rb7_fifo_wait();

            // The body is created at the slot index specified by id
            // If id is valid, destroy any existing body there first
            if (id < NEA_RB_MAX_BODIES)
            {
                nea_rb7_bodies[id].used = false;
                nea_rb7_body_t *b = &nea_rb7_bodies[id];

                // Zero it
                for (int j = 0; j < (int)(sizeof(*b) / sizeof(int32_t)); j++)
                    ((int32_t *)b)[j] = 0;

                b->used = true;
                b->sleep = false;
                b->position = nea_v3(px, py, pz);
                b->size = nea_v3(hx, hy, hz);
                b->mass = mass;
                b->restitution = (1 << 12) / 2;
                b->friction = (1 << 12) / 2;
                nea_mat3_identity(b->transform);

                // Compute inertia
                if (mass > 0)
                {
                    int32_t sx2 = (int32_t)(((int64_t)hx * hx) >> 12);
                    int32_t sy2 = (int32_t)(((int64_t)hy * hy) >> 12);
                    int32_t sz2 = (int32_t)(((int64_t)hz * hz) >> 12);

                    int32_t dx = (int32_t)(((int64_t)mass * (sy2 + sz2)) >> 12);
                    int32_t dy = (int32_t)(((int64_t)mass * (sx2 + sz2)) >> 12);
                    int32_t dz = (int32_t)(((int64_t)mass * (sx2 + sy2)) >> 12);

                    int32_t three_f32 = 3 << 12;
                    b->invInertia[0] = dx > 0 ?
                        (int32_t)(((int64_t)three_f32 << 12) / dx) : 0;
                    b->invInertia[1] = dy > 0 ?
                        (int32_t)(((int64_t)three_f32 << 12) / dy) : 0;
                    b->invInertia[2] = dz > 0 ?
                        (int32_t)(((int64_t)three_f32 << 12) / dz) : 0;
                }

                for (int j = 0; j < 9; j++)
                    b->invWInertia[j] = 0;
                b->invWInertia[0] = b->invInertia[0];
                b->invWInertia[4] = b->invInertia[1];
                b->invWInertia[8] = b->invInertia[2];
            }
            break;
        }

        case NEA_RB_CMD_KILL_BODY:
            nea_rb7_destroy_body((int)id);
            break;

        case NEA_RB_CMD_APPLY_FORCE:
        {
            int32_t fx = (int32_t)nea_rb7_fifo_wait();
            int32_t fy = (int32_t)nea_rb7_fifo_wait();
            int32_t fz = (int32_t)nea_rb7_fifo_wait();
            int32_t px = (int32_t)nea_rb7_fifo_wait();
            int32_t py = (int32_t)nea_rb7_fifo_wait();
            int32_t pz = (int32_t)nea_rb7_fifo_wait();

            nea_rb7_apply_force((int)id, nea_v3(fx, fy, fz),
                                nea_v3(px, py, pz));
            break;
        }

        case NEA_RB_CMD_APPLY_IMPULSE:
        {
            int32_t jx = (int32_t)nea_rb7_fifo_wait();
            int32_t jy = (int32_t)nea_rb7_fifo_wait();
            int32_t jz = (int32_t)nea_rb7_fifo_wait();
            int32_t px = (int32_t)nea_rb7_fifo_wait();
            int32_t py = (int32_t)nea_rb7_fifo_wait();
            int32_t pz = (int32_t)nea_rb7_fifo_wait();

            nea_rb7_apply_impulse((int)id, nea_v3(jx, jy, jz),
                                  nea_v3(px, py, pz));
            break;
        }

        case NEA_RB_CMD_SET_VELOCITY:
        {
            int32_t vx = (int32_t)nea_rb7_fifo_wait();
            int32_t vy = (int32_t)nea_rb7_fifo_wait();
            int32_t vz = (int32_t)nea_rb7_fifo_wait();

            if (id < NEA_RB_MAX_BODIES && nea_rb7_bodies[id].used)
            {
                nea_rb7_bodies[id].velocity = nea_v3(vx, vy, vz);
                nea_rb7_bodies[id].sleep = false;
                nea_rb7_bodies[id].sleepCounter = 0;
            }
            break;
        }

        case NEA_RB_CMD_SET_GRAVITY:
        {
            int32_t g = (int32_t)nea_rb7_fifo_wait();
            nea_rb7_gravity = g;
            break;
        }

        case NEA_RB_CMD_ADD_STATIC:
        {
            int32_t px = (int32_t)nea_rb7_fifo_wait();
            int32_t py = (int32_t)nea_rb7_fifo_wait();
            int32_t pz = (int32_t)nea_rb7_fifo_wait();
            int32_t sx = (int32_t)nea_rb7_fifo_wait();
            int32_t sy = (int32_t)nea_rb7_fifo_wait();
            int32_t sz = (int32_t)nea_rb7_fifo_wait();
            int32_t nx = (int32_t)nea_rb7_fifo_wait();
            int32_t ny = (int32_t)nea_rb7_fifo_wait();
            int32_t nz = (int32_t)nea_rb7_fifo_wait();

            // Create static at the requested slot
            if (id < NEA_RB_MAX_STATICS)
            {
                nea_rb7_statics[id].position = nea_v3(px, py, pz);
                nea_rb7_statics[id].size = nea_v3(sx, sy, sz);
                nea_rb7_statics[id].normal = nea_v3_normalize(nea_v3(nx, ny, nz));
                nea_rb7_statics[id].used = true;
            }
            break;
        }

        case NEA_RB_CMD_REMOVE_STATIC:
            nea_rb7_remove_static((int)id);
            break;

        case NEA_RB_CMD_SET_RESTITUTION:
        {
            int32_t e = (int32_t)nea_rb7_fifo_wait();
            if (id < NEA_RB_MAX_BODIES && nea_rb7_bodies[id].used)
                nea_rb7_bodies[id].restitution = e;
            break;
        }

        case NEA_RB_CMD_SET_FRICTION:
        {
            int32_t mu = (int32_t)nea_rb7_fifo_wait();
            if (id < NEA_RB_MAX_BODIES && nea_rb7_bodies[id].used)
                nea_rb7_bodies[id].friction = mu;
            break;
        }

        case NEA_RB_CMD_SET_POSITION:
        {
            int32_t px = (int32_t)nea_rb7_fifo_wait();
            int32_t py = (int32_t)nea_rb7_fifo_wait();
            int32_t pz = (int32_t)nea_rb7_fifo_wait();

            if (id < NEA_RB_MAX_BODIES && nea_rb7_bodies[id].used)
            {
                nea_rb7_bodies[id].position = nea_v3(px, py, pz);
                nea_rb7_bodies[id].sleep = false;
                nea_rb7_bodies[id].sleepCounter = 0;
            }
            break;
        }

        case NEA_RB_CMD_INJECT_CONTACT:
        {
            // External contact injected from ARM9 (e.g. NEA_ColTest result)
            // +7 words: nx, ny, nz, px, py, pz, depth
            int32_t nx    = (int32_t)nea_rb7_fifo_wait();
            int32_t ny    = (int32_t)nea_rb7_fifo_wait();
            int32_t nz    = (int32_t)nea_rb7_fifo_wait();
            int32_t px    = (int32_t)nea_rb7_fifo_wait();
            int32_t py    = (int32_t)nea_rb7_fifo_wait();
            int32_t pz    = (int32_t)nea_rb7_fifo_wait();
            int32_t depth = (int32_t)nea_rb7_fifo_wait();

            if (id < NEA_RB_MAX_BODIES && nea_rb7_bodies[id].used)
            {
                // Add contact with type=2 (external, treated as body-static)
                if (nea_rb7_num_contacts < NEA_RB_MAX_CONTACTS)
                {
                    nea_rb7_contact_t *cp =
                        &nea_rb7_contacts[nea_rb7_num_contacts++];
                    cp->normal = nea_v3(nx, ny, nz);
                    cp->point = nea_v3(px, py, pz);
                    cp->penetration = depth;
                    cp->body = &nea_rb7_bodies[id];
                    cp->target = NULL;
                    cp->type = 2; // External injection

                    nea_rb7_bodies[id].numContacts++;
                    if (depth > nea_rb7_bodies[id].maxPenetration)
                        nea_rb7_bodies[id].maxPenetration = depth;

                    // Wake up if sleeping
                    nea_rb7_bodies[id].sleep = false;
                    nea_rb7_bodies[id].sleepCounter = 0;
                }
            }
            break;
        }

        default:
            break;
        }
    }
}

// =========================================================================
// State sender (ARM7 -> ARM9)
// =========================================================================

ARM_CODE void nea_rb7_send_state(void)
{
    for (int i = 0; i < NEA_RB_MAX_BODIES; i++)
    {
        nea_rb7_body_t *b = &nea_rb7_bodies[i];
        if (!b->used)
            continue;

        // Send header: id | flags
        u32 flags = b->sleep ? NEA_RB_STATE_FLAG_SLEEP : 0;
        fifoSendValue32(NEA_RB_FIFO_STATE,
                        NEA_RB_STATE_ENCODE_HDR((u32)i, flags));

        // Position (3 words)
        fifoSendValue32(NEA_RB_FIFO_STATE, (u32)b->position.x);
        fifoSendValue32(NEA_RB_FIFO_STATE, (u32)b->position.y);
        fifoSendValue32(NEA_RB_FIFO_STATE, (u32)b->position.z);

        // Rotation matrix (9 words, full precision)
        for (int j = 0; j < 9; j++)
            fifoSendValue32(NEA_RB_FIFO_STATE, (u32)b->transform[j]);
    }

    // End-of-frame sentinel
    fifoSendValue32(NEA_RB_FIFO_STATE, NEA_RB_STATE_END);
}
