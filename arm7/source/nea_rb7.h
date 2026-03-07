// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Warioware64
//
// This file is part of Nitro Engine Advanced

#ifndef NEA_RB7_H__
#define NEA_RB7_H__

/// @file   nea_rb7.h
/// @brief  ARM7-side rigid body simulation internals.

#include <nds.h>
#include "NEA_RB_IPC.h"

// =========================================================================
// Fixed-point helpers (f32 = 20.12)
// =========================================================================

typedef struct {
    int32_t x, y, z;
} nea_vec3_t;

static inline nea_vec3_t nea_v3(int32_t x, int32_t y, int32_t z)
{
    nea_vec3_t v = { x, y, z };
    return v;
}

static inline nea_vec3_t nea_v3_add(nea_vec3_t a, nea_vec3_t b)
{
    return nea_v3(a.x + b.x, a.y + b.y, a.z + b.z);
}

static inline nea_vec3_t nea_v3_sub(nea_vec3_t a, nea_vec3_t b)
{
    return nea_v3(a.x - b.x, a.y - b.y, a.z - b.z);
}

static inline nea_vec3_t nea_v3_neg(nea_vec3_t a)
{
    return nea_v3(-a.x, -a.y, -a.z);
}

static inline nea_vec3_t nea_v3_scale(nea_vec3_t a, int32_t s)
{
    return nea_v3(
        (int32_t)(((int64_t)a.x * s) >> 12),
        (int32_t)(((int64_t)a.y * s) >> 12),
        (int32_t)(((int64_t)a.z * s) >> 12)
    );
}

static inline int32_t nea_v3_dot(nea_vec3_t a, nea_vec3_t b)
{
    return (int32_t)((
        (int64_t)a.x * b.x +
        (int64_t)a.y * b.y +
        (int64_t)a.z * b.z
    ) >> 12);
}

static inline nea_vec3_t nea_v3_cross(nea_vec3_t a, nea_vec3_t b)
{
    return nea_v3(
        (int32_t)(((int64_t)a.y * b.z - (int64_t)a.z * b.y) >> 12),
        (int32_t)(((int64_t)a.z * b.x - (int64_t)a.x * b.z) >> 12),
        (int32_t)(((int64_t)a.x * b.y - (int64_t)a.y * b.x) >> 12)
    );
}

static inline int32_t nea_v3_length_sq(nea_vec3_t a)
{
    return nea_v3_dot(a, a);
}

// Fast integer square root for f32 values (bit-scanning, no division).
// Input: f32 value (20.12). Output: sqrt in f32.
// Uses only shifts, compares, and adds — no division at all.
ARM_CODE static inline int32_t nea_sqrtf32(int32_t a)
{
    if (a <= 0)
        return 0;
    uint32_t n = (uint32_t)a;
    uint32_t r = 0;
    uint32_t bit = 1u << 30;
    while (bit > n)
        bit >>= 2;
    while (bit != 0)
    {
        if (n >= r + bit)
        {
            n -= r + bit;
            r = (r >> 1) + bit;
        }
        else
        {
            r >>= 1;
        }
        bit >>= 2;
    }
    // Correct for f32 fixed-point: sqrt(x * 4096) = sqrt(x) * 64
    return (int32_t)(r << 6);
}

static inline int32_t nea_v3_length(nea_vec3_t a)
{
    return nea_sqrtf32(nea_v3_dot(a, a));
}

static inline nea_vec3_t nea_v3_normalize(nea_vec3_t a)
{
    int32_t len = nea_v3_length(a);
    if (len == 0)
        return nea_v3(0, 0, 0);
    return nea_v3(
        (int32_t)(((int64_t)a.x << 12) / len),
        (int32_t)(((int64_t)a.y << 12) / len),
        (int32_t)(((int64_t)a.z << 12) / len)
    );
}

static inline int32_t nea_abs(int32_t x)
{
    return x < 0 ? -x : x;
}

static inline int32_t nea_clamp(int32_t val, int32_t lo, int32_t hi)
{
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

// =========================================================================
// 3x3 Matrix helpers (row-major, f32)
// =========================================================================

// Multiply two 3x3 matrices: out = a * b
void nea_mat3_mul(const int32_t *a, const int32_t *b, int32_t *out);

// Transpose 3x3 matrix: out = aᵀ
void nea_mat3_transpose(const int32_t *a, int32_t *out);

// Set identity
void nea_mat3_identity(int32_t *m);

// Gram-Schmidt orthonormalization (fixes drift from rotation updates)
void nea_mat3_orthonormalize(int32_t *m);

// Transform vector by matrix: out = M * v
nea_vec3_t nea_mat3_mul_vec(const int32_t *m, nea_vec3_t v);

// =========================================================================
// Rigid body
// =========================================================================

#define NEA_RB7_SLEEP_THRESHOLD    50   // Energy threshold (f32)
#define NEA_RB7_SLEEP_TIME         48   // Frames below threshold to sleep
#define NEA_RB7_WAKE_MULTIPLIER    10   // Wake if energy > threshold * this
#define NEA_RB7_PENETRATION_THRESHOLD (1 << 6) // 64 in f32 ~ 0.015 units
#define NEA_RB7_SUBSTEPS           4    // Physics sub-steps per frame

typedef struct {
    nea_vec3_t position;
    nea_vec3_t size;              // OBB half-extents (f32)
    int32_t transform[9];         // 3x3 rotation matrix (row-major, f32)
    int32_t invInertia[3];        // Diagonal inverse inertia (local frame)
    int32_t invWInertia[9];       // World-space inverse inertia: R I⁻¹ Rᵀ

    nea_vec3_t velocity;
    nea_vec3_t angularVelocity;
    nea_vec3_t angularMomentum;
    nea_vec3_t forces;            // Accumulated force this step
    nea_vec3_t moment;            // Accumulated torque this step

    int32_t mass;                 // f32. 0 = infinite/static.
    int32_t restitution;          // f32. Default 0.5.
    int32_t friction;             // f32. Default 0.5.

    uint8_t numContacts;
    bool used;
    bool sleep;
    uint16_t sleepCounter;
    int32_t energy;
    int32_t maxPenetration;       // Deepest penetration this step
} nea_rb7_body_t;

// Static axis-aligned rectangle (wall/floor/ceiling)
typedef struct {
    nea_vec3_t position;          // Center position (f32)
    nea_vec3_t size;              // Half-extents on plane axes (f32)
    nea_vec3_t normal;            // Surface normal (unit, f32)
    bool used;
} nea_rb7_aar_t;

// Contact point
typedef struct {
    nea_vec3_t point;             // World-space contact position
    nea_vec3_t normal;            // Contact normal (body A -> B)
    int32_t penetration;          // Penetration depth (positive = overlap)
    void *body;                   // Owning body (body A)
    void *target;                 // Other body pointer, or NULL for static
    uint8_t type;                 // 0 = body-body, 1 = body-AAR
} nea_rb7_contact_t;

// Body state backup for adaptive timestep bisection
typedef struct {
    nea_vec3_t position;
    nea_vec3_t velocity;
    nea_vec3_t angularVelocity;
    nea_vec3_t angularMomentum;
    nea_vec3_t forces;
    nea_vec3_t moment;
    int32_t transform[9];
} nea_rb7_backup_t;

#define NEA_RB7_MAX_BISECTIONS  2    // Max bisection retries (min dt = original/4)

// =========================================================================
// Global state
// =========================================================================

extern nea_rb7_body_t    nea_rb7_bodies[NEA_RB_MAX_BODIES];
extern nea_rb7_aar_t     nea_rb7_statics[NEA_RB_MAX_STATICS];
extern nea_rb7_contact_t nea_rb7_contacts[NEA_RB_MAX_CONTACTS];
extern int               nea_rb7_num_contacts;
extern int32_t           nea_rb7_gravity;     // f32, Y-axis
extern bool              nea_rb7_running;

// =========================================================================
// Public functions (ARM7 internal)
// =========================================================================

// Simulation
void nea_rb7_init(void);
void nea_rb7_reset(void);
void nea_rb7_update(void);            // One sub-step
void nea_rb7_sleep_check_all(void);   // Run once per frame (not per sub-step)

// Body management
int  nea_rb7_create_body(nea_vec3_t size, int32_t mass, nea_vec3_t pos);
void nea_rb7_destroy_body(int id);

// Force / impulse
void nea_rb7_apply_force(int id, nea_vec3_t force, nea_vec3_t point);
void nea_rb7_apply_impulse(int id, nea_vec3_t impulse, nea_vec3_t point);

// Static colliders
int  nea_rb7_add_static(nea_vec3_t pos, nea_vec3_t size, nea_vec3_t normal);
void nea_rb7_remove_static(int id);

// IPC
void nea_rb7_listen(void);       // Process ARM9 commands
void nea_rb7_send_state(void);   // Send body states to ARM9

#endif // NEA_RB7_H__
