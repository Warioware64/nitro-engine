// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Warioware64
//
// This file is part of Nitro Engine Advanced

/// @file   nea_rb7.c
/// @brief  ARM7 rigid body simulation: OBB dynamics, collision, impulse response.

#include "nea_rb7.h"

// =========================================================================
// Global state
// =========================================================================

nea_rb7_body_t    nea_rb7_bodies[NEA_RB_MAX_BODIES];
nea_rb7_aar_t     nea_rb7_statics[NEA_RB_MAX_STATICS];
nea_rb7_contact_t nea_rb7_contacts[NEA_RB_MAX_CONTACTS];
int32_t           nea_rb7_gravity = 0;
bool              nea_rb7_running = false;

int nea_rb7_num_contacts = 0;

// =========================================================================
// 3x3 Matrix operations (row-major, f32)
// =========================================================================

void nea_mat3_identity(int32_t *m)
{
    m[0] = 1 << 12; m[1] = 0;        m[2] = 0;
    m[3] = 0;        m[4] = 1 << 12; m[5] = 0;
    m[6] = 0;        m[7] = 0;        m[8] = 1 << 12;
}

ARM_CODE void nea_mat3_mul(const int32_t *a, const int32_t *b, int32_t *out)
{
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            out[j + i * 3] = (int32_t)((
                (int64_t)a[0 + i * 3] * b[j + 0 * 3] +
                (int64_t)a[1 + i * 3] * b[j + 1 * 3] +
                (int64_t)a[2 + i * 3] * b[j + 2 * 3]
            ) >> 12);
        }
    }
}

void nea_mat3_transpose(const int32_t *a, int32_t *out)
{
    out[0] = a[0]; out[1] = a[3]; out[2] = a[6];
    out[3] = a[1]; out[4] = a[4]; out[5] = a[7];
    out[6] = a[2]; out[7] = a[5]; out[8] = a[8];
}

nea_vec3_t nea_mat3_mul_vec(const int32_t *m, nea_vec3_t v)
{
    return nea_v3(
        (int32_t)(((int64_t)m[0] * v.x + (int64_t)m[1] * v.y + (int64_t)m[2] * v.z) >> 12),
        (int32_t)(((int64_t)m[3] * v.x + (int64_t)m[4] * v.y + (int64_t)m[5] * v.z) >> 12),
        (int32_t)(((int64_t)m[6] * v.x + (int64_t)m[7] * v.y + (int64_t)m[8] * v.z) >> 12)
    );
}

// Gram-Schmidt orthonormalization to fix rotation matrix drift
ARM_CODE void nea_mat3_orthonormalize(int32_t *m)
{
    // Row 0
    nea_vec3_t r0 = nea_v3(m[0], m[1], m[2]);
    r0 = nea_v3_normalize(r0);

    // Row 1: subtract projection onto r0
    nea_vec3_t r1 = nea_v3(m[3], m[4], m[5]);
    int32_t d = nea_v3_dot(r1, r0);
    r1 = nea_v3_sub(r1, nea_v3_scale(r0, d));
    r1 = nea_v3_normalize(r1);

    // Row 2: cross product of r0 and r1
    nea_vec3_t r2 = nea_v3_cross(r0, r1);

    m[0] = r0.x; m[1] = r0.y; m[2] = r0.z;
    m[3] = r1.x; m[4] = r1.y; m[5] = r1.z;
    m[6] = r2.x; m[7] = r2.y; m[8] = r2.z;
}

// Add two 3x3 matrices element-wise
static void nea_mat3_add(const int32_t *a, const int32_t *b, int32_t *out)
{
    for (int i = 0; i < 9; i++)
        out[i] = a[i] + b[i];
}

// =========================================================================
// Initialization
// =========================================================================

void nea_rb7_init(void)
{
    nea_rb7_reset();
}

void nea_rb7_reset(void)
{
    for (int i = 0; i < NEA_RB_MAX_BODIES; i++)
        nea_rb7_bodies[i].used = false;
    for (int i = 0; i < NEA_RB_MAX_STATICS; i++)
        nea_rb7_statics[i].used = false;
    nea_rb7_num_contacts = 0;
    nea_rb7_gravity = 0;
    nea_rb7_running = false;
}

// =========================================================================
// Body management
// =========================================================================

int nea_rb7_create_body(nea_vec3_t size, int32_t mass, nea_vec3_t pos)
{
    for (int i = 0; i < NEA_RB_MAX_BODIES; i++)
    {
        if (!nea_rb7_bodies[i].used)
        {
            nea_rb7_body_t *b = &nea_rb7_bodies[i];

            // Zero everything
            for (int j = 0; j < (int)(sizeof(*b) / sizeof(int32_t)); j++)
                ((int32_t *)b)[j] = 0;

            b->used = true;
            b->sleep = false;
            b->position = pos;
            b->size = size;
            b->mass = mass;
            b->restitution = (1 << 12) / 2; // 0.5
            b->friction = (1 << 12) / 2;    // 0.5

            // Identity rotation
            nea_mat3_identity(b->transform);

            // Compute diagonal inverse inertia for a box:
            // I = (m/12) * diag(y²+z², x²+z², x²+y²)
            // I⁻¹ = diag(12/(m*(y²+z²)), ...)
            if (mass > 0)
            {
                int32_t sx2 = (int32_t)(((int64_t)size.x * size.x) >> 12);
                int32_t sy2 = (int32_t)(((int64_t)size.y * size.y) >> 12);
                int32_t sz2 = (int32_t)(((int64_t)size.z * size.z) >> 12);

                int32_t denom_x = (int32_t)(((int64_t)mass * (sy2 + sz2)) >> 12);
                int32_t denom_y = (int32_t)(((int64_t)mass * (sx2 + sz2)) >> 12);
                int32_t denom_z = (int32_t)(((int64_t)mass * (sx2 + sy2)) >> 12);

                // I⁻¹ = 3 / denom (using half-extents, so factor is 3 not 12)
                int32_t three_f32 = 3 << 12;
                b->invInertia[0] = denom_x > 0 ?
                    (int32_t)(((int64_t)three_f32 << 12) / denom_x) : 0;
                b->invInertia[1] = denom_y > 0 ?
                    (int32_t)(((int64_t)three_f32 << 12) / denom_y) : 0;
                b->invInertia[2] = denom_z > 0 ?
                    (int32_t)(((int64_t)three_f32 << 12) / denom_z) : 0;
            }

            // Initial world-space inverse inertia = identity * I⁻¹
            // (since transform is identity)
            for (int j = 0; j < 9; j++)
                b->invWInertia[j] = 0;
            b->invWInertia[0] = b->invInertia[0];
            b->invWInertia[4] = b->invInertia[1];
            b->invWInertia[8] = b->invInertia[2];

            return i;
        }
    }
    return -1; // No free slot
}

void nea_rb7_destroy_body(int id)
{
    if (id >= 0 && id < NEA_RB_MAX_BODIES)
        nea_rb7_bodies[id].used = false;
}

// =========================================================================
// Static collider management
// =========================================================================

int nea_rb7_add_static(nea_vec3_t pos, nea_vec3_t size, nea_vec3_t normal)
{
    for (int i = 0; i < NEA_RB_MAX_STATICS; i++)
    {
        if (!nea_rb7_statics[i].used)
        {
            nea_rb7_statics[i].position = pos;
            nea_rb7_statics[i].size = size;
            nea_rb7_statics[i].normal = nea_v3_normalize(normal);
            nea_rb7_statics[i].used = true;
            return i;
        }
    }
    return -1;
}

void nea_rb7_remove_static(int id)
{
    if (id >= 0 && id < NEA_RB_MAX_STATICS)
        nea_rb7_statics[id].used = false;
}

// =========================================================================
// Force / impulse application
// =========================================================================

void nea_rb7_apply_force(int id, nea_vec3_t force, nea_vec3_t point)
{
    if (id < 0 || id >= NEA_RB_MAX_BODIES)
        return;
    nea_rb7_body_t *b = &nea_rb7_bodies[id];
    if (!b->used)
        return;

    // Wake up if sleeping
    b->sleep = false;
    b->sleepCounter = 0;

    // Accumulate force
    b->forces = nea_v3_add(b->forces, force);

    // Torque = (point - center) × force
    nea_vec3_t r = nea_v3_sub(point, b->position);
    nea_vec3_t torque = nea_v3_cross(r, force);
    b->moment = nea_v3_add(b->moment, torque);
}

void nea_rb7_apply_impulse(int id, nea_vec3_t impulse, nea_vec3_t point)
{
    if (id < 0 || id >= NEA_RB_MAX_BODIES)
        return;
    nea_rb7_body_t *b = &nea_rb7_bodies[id];
    if (!b->used || b->mass == 0)
        return;

    // Wake up
    b->sleep = false;
    b->sleepCounter = 0;

    // Linear: v += J / m
    int32_t inv_mass = (int32_t)(((int64_t)(1 << 12) << 12) / b->mass);
    b->velocity = nea_v3_add(b->velocity, nea_v3_scale(impulse, inv_mass));

    // Angular: L += r × J
    nea_vec3_t r = nea_v3_sub(point, b->position);
    nea_vec3_t dL = nea_v3_cross(r, impulse);
    b->angularMomentum = nea_v3_add(b->angularMomentum, dL);

    // Update angular velocity: ω = I_w⁻¹ · L
    b->angularVelocity = nea_mat3_mul_vec(b->invWInertia, b->angularMomentum);
}

// =========================================================================
// Integration (semi-implicit Euler)
// =========================================================================

ARM_CODE static void nea_rb7_integrate(nea_rb7_body_t *o, int32_t dt)
{
    if (o->mass == 0)
        return; // Static body

    // --- Linear motion ---
    // a = F / m
    int32_t inv_mass = (int32_t)(((int64_t)(1 << 12) << 12) / o->mass);

    // v_new = v_old + a * dt
    // Using semi-implicit: position uses new velocity
    nea_vec3_t accel = nea_v3_scale(o->forces, inv_mass);
    nea_vec3_t dv;
    dv.x = (int32_t)(((int64_t)accel.x * dt) >> 12);
    dv.y = (int32_t)(((int64_t)accel.y * dt) >> 12);
    dv.z = (int32_t)(((int64_t)accel.z * dt) >> 12);

    nea_vec3_t new_vel = nea_v3_add(o->velocity, dv);

    // x_new = x_old + v_new * dt
    nea_vec3_t dx;
    dx.x = (int32_t)(((int64_t)new_vel.x * dt) >> 12);
    dx.y = (int32_t)(((int64_t)new_vel.y * dt) >> 12);
    dx.z = (int32_t)(((int64_t)new_vel.z * dt) >> 12);

    o->position = nea_v3_add(o->position, dx);
    o->velocity = new_vel;

    // --- Angular motion ---
    // Torque: L += τ * dt
    nea_vec3_t dL;
    dL.x = (int32_t)(((int64_t)o->moment.x * dt) >> 12);
    dL.y = (int32_t)(((int64_t)o->moment.y * dt) >> 12);
    dL.z = (int32_t)(((int64_t)o->moment.z * dt) >> 12);
    o->angularMomentum = nea_v3_add(o->angularMomentum, dL);

    // ω = I_w⁻¹ · L
    o->angularVelocity = nea_mat3_mul_vec(o->invWInertia, o->angularMomentum);

    // Update rotation matrix via skew-symmetric matrix:
    // R_new = R_old + [ω]× * R_old * dt
    // where [ω]× = [0 -ωz ωy; ωz 0 -ωx; -ωy ωx 0]
    int32_t wx = (int32_t)(((int64_t)o->angularVelocity.x * dt) >> 12);
    int32_t wy = (int32_t)(((int64_t)o->angularVelocity.y * dt) >> 12);
    int32_t wz = (int32_t)(((int64_t)o->angularVelocity.z * dt) >> 12);

    int32_t skew[9] = {
        0,   -wz,  wy,
        wz,   0,  -wx,
       -wy,  wx,   0
    };

    int32_t dR[9];
    nea_mat3_mul(skew, o->transform, dR);
    nea_mat3_add(o->transform, dR, o->transform);

    // Orthonormalize to prevent drift
    nea_mat3_orthonormalize(o->transform);

    // Update world-space inverse inertia: I_w⁻¹ = R · I_local⁻¹ · Rᵀ
    // Since I_local⁻¹ is diagonal, we can optimize:
    // temp = I_local⁻¹ · Rᵀ = diag(I⁻¹) * Rᵀ
    int32_t rt[9], temp[9];
    nea_mat3_transpose(o->transform, rt);

    // Scale each row of Rᵀ by diagonal element
    for (int i = 0; i < 3; i++)
    {
        temp[i * 3 + 0] = (int32_t)(((int64_t)o->invInertia[i] * rt[i * 3 + 0]) >> 12);
        temp[i * 3 + 1] = (int32_t)(((int64_t)o->invInertia[i] * rt[i * 3 + 1]) >> 12);
        temp[i * 3 + 2] = (int32_t)(((int64_t)o->invInertia[i] * rt[i * 3 + 2]) >> 12);
    }
    nea_mat3_mul(temp, o->transform, o->invWInertia);

    // Clear accumulated forces for next step
    o->forces = nea_v3(0, 0, 0);
    o->moment = nea_v3(0, 0, 0);
}

// =========================================================================
// OBB vertex computation
// =========================================================================

static void nea_rb7_obb_vertices(const nea_rb7_body_t *b, nea_vec3_t verts[8])
{
    // 8 corners of an OBB = position ± R * half_extent on each axis
    for (int i = 0; i < 8; i++)
    {
        nea_vec3_t local = nea_v3(
            (i & 1) ? b->size.x : -b->size.x,
            (i & 2) ? b->size.y : -b->size.y,
            (i & 4) ? b->size.z : -b->size.z
        );
        nea_vec3_t world = nea_mat3_mul_vec(b->transform, local);
        verts[i] = nea_v3_add(b->position, world);
    }
}

// Get OBB axes (columns of rotation matrix = row-major rows)
static void nea_rb7_obb_axes(const nea_rb7_body_t *b, nea_vec3_t axes[3])
{
    axes[0] = nea_v3(b->transform[0], b->transform[1], b->transform[2]);
    axes[1] = nea_v3(b->transform[3], b->transform[4], b->transform[5]);
    axes[2] = nea_v3(b->transform[6], b->transform[7], b->transform[8]);
}

// =========================================================================
// AABB quick-reject for OBB pairs
// =========================================================================

static bool nea_rb7_aabb_overlap(const nea_rb7_body_t *a, const nea_rb7_body_t *b)
{
    // Compute world-space AABB for each OBB
    // AABB half-extent on each axis = sum of absolute projections
    int32_t a_hx = 0, a_hy = 0, a_hz = 0;
    int32_t b_hx = 0, b_hy = 0, b_hz = 0;

    for (int i = 0; i < 3; i++)
    {
        int32_t half = (i == 0) ? a->size.x : (i == 1) ? a->size.y : a->size.z;
        a_hx += (int32_t)(((int64_t)nea_abs(a->transform[i * 3 + 0]) * half) >> 12);
        a_hy += (int32_t)(((int64_t)nea_abs(a->transform[i * 3 + 1]) * half) >> 12);
        a_hz += (int32_t)(((int64_t)nea_abs(a->transform[i * 3 + 2]) * half) >> 12);

        half = (i == 0) ? b->size.x : (i == 1) ? b->size.y : b->size.z;
        b_hx += (int32_t)(((int64_t)nea_abs(b->transform[i * 3 + 0]) * half) >> 12);
        b_hy += (int32_t)(((int64_t)nea_abs(b->transform[i * 3 + 1]) * half) >> 12);
        b_hz += (int32_t)(((int64_t)nea_abs(b->transform[i * 3 + 2]) * half) >> 12);
    }

    int32_t margin = NEA_RB7_PENETRATION_THRESHOLD;
    int32_t dx = nea_abs(a->position.x - b->position.x);
    int32_t dy = nea_abs(a->position.y - b->position.y);
    int32_t dz = nea_abs(a->position.z - b->position.z);

    if (dx > a_hx + b_hx + margin) return false;
    if (dy > a_hy + b_hy + margin) return false;
    if (dz > a_hz + b_hz + margin) return false;

    return true;
}

// =========================================================================
// Edge-segment clipping helpers for multi-contact OBB-OBB
// =========================================================================

// OBB edge table: 12 edges connecting vertices that differ by 1 bit
// Vertex i: x=(i&1)?+:-,  y=(i&2)?+:-,  z=(i&4)?+:-
static const uint8_t nea_obb_edges[12][2] = {
    {0, 1}, {2, 3}, {4, 5}, {6, 7},  // X-axis edges (bit 0)
    {0, 2}, {1, 3}, {4, 6}, {5, 7},  // Y-axis edges (bit 1)
    {0, 4}, {1, 5}, {2, 6}, {3, 7}   // Z-axis edges (bit 2)
};

// Transform world-space point into OBB's local frame (applies Rᵀ * delta)
// R maps local→world, so R⁻¹ = Rᵀ maps world→local.
// Rᵀ * delta = dot each column of R with delta.
// Column k in row-major: (m[k], m[3+k], m[6+k])
static inline nea_vec3_t nea_world_to_local(const nea_rb7_body_t *body, nea_vec3_t world_pt)
{
    nea_vec3_t delta = nea_v3_sub(world_pt, body->position);
    const int32_t *m = body->transform;
    return nea_v3(
        (int32_t)(((int64_t)m[0] * delta.x + (int64_t)m[3] * delta.y + (int64_t)m[6] * delta.z) >> 12),
        (int32_t)(((int64_t)m[1] * delta.x + (int64_t)m[4] * delta.y + (int64_t)m[7] * delta.z) >> 12),
        (int32_t)(((int64_t)m[2] * delta.x + (int64_t)m[5] * delta.y + (int64_t)m[8] * delta.z) >> 12)
    );
}

// Clip segment (local0→local1) against axis-aligned box [-hx,+hx]×[-hy,+hy]×[-hz,+hz].
// Returns parametric interval [t_min, t_max] (f32) of the segment inside the box.
// Returns false if the segment is fully outside.
ARM_CODE static bool nea_clip_segment_aabb(
    nea_vec3_t local0, nea_vec3_t local1,
    int32_t hx, int32_t hy, int32_t hz,
    int32_t *t_min_out, int32_t *t_max_out)
{
    int32_t t0 = 0;
    int32_t t1 = 1 << 12; // 1.0 in f32

    int32_t p[3] = { local0.x, local0.y, local0.z };
    int32_t d[3] = { local1.x - local0.x, local1.y - local0.y, local1.z - local0.z };
    int32_t h[3] = { hx, hy, hz };

    for (int i = 0; i < 3; i++)
    {
        if (nea_abs(d[i]) < 4) // Nearly parallel to slab
        {
            if (p[i] < -h[i] || p[i] > h[i])
                return false;
        }
        else
        {
            int32_t t_lo = (int32_t)(((int64_t)(-h[i] - p[i]) << 12) / d[i]);
            int32_t t_hi = (int32_t)(((int64_t)( h[i] - p[i]) << 12) / d[i]);

            if (t_lo > t_hi)
            {
                int32_t tmp = t_lo;
                t_lo = t_hi;
                t_hi = tmp;
            }

            if (t_lo > t0) t0 = t_lo;
            if (t_hi < t1) t1 = t_hi;

            if (t0 > t1)
                return false;
        }
    }

    *t_min_out = t0;
    *t_max_out = t1;
    return true;
}

// Max contacts generated per OBB-OBB pair
#define NEA_RB7_MAX_PAIR_CONTACTS 4

// =========================================================================
// OBB-OBB collision (SAT with 15 axes + multi-contact clipping)
// =========================================================================

ARM_CODE static bool nea_rb7_collide_obb_obb(nea_rb7_body_t *a, nea_rb7_body_t *b)
{
    if (!nea_rb7_aabb_overlap(a, b))
        return false;

    nea_vec3_t axes_a[3], axes_b[3];
    nea_rb7_obb_axes(a, axes_a);
    nea_rb7_obb_axes(b, axes_b);

    nea_vec3_t d = nea_v3_sub(b->position, a->position);
    int32_t half_a[3] = { a->size.x, a->size.y, a->size.z };
    int32_t half_b[3] = { b->size.x, b->size.y, b->size.z };

    // Track best axis WITHOUT normalizing. We compare overlaps using
    // squared ratios: overlap²/len_sq, avoiding sqrt entirely in the loop.
    // Only the winning axis gets normalized at the end.
    int32_t min_overlap_raw = 0;  // Unnormalized overlap of best axis
    int32_t min_len_sq = 1;       // Length² of best axis
    nea_vec3_t best_axis_raw = nea_v3(0, 1 << 12, 0);
    bool best_d_neg = false;
    bool found_any = false;

    // Test 15 SAT axes: 3 from A, 3 from B, 9 cross products
    // All projections use unnormalized axes; separation test is scale-invariant.

    #define TEST_AXIS(axis_vec)                                         \
    do {                                                                \
        nea_vec3_t ax = (axis_vec);                                     \
        int32_t ax_len_sq = nea_v3_length_sq(ax);                      \
        if (ax_len_sq < 10) break; /* degenerate axis */               \
        int32_t proj_d = nea_abs(nea_v3_dot(d, ax));                   \
        int32_t proj_a = 0, proj_b = 0;                                \
        for (int k = 0; k < 3; k++) {                                  \
            proj_a += (int32_t)(((int64_t)nea_abs(nea_v3_dot(axes_a[k], ax)) * half_a[k]) >> 12); \
            proj_b += (int32_t)(((int64_t)nea_abs(nea_v3_dot(axes_b[k], ax)) * half_b[k]) >> 12); \
        }                                                              \
        int32_t overlap = (proj_a + proj_b) - proj_d;                  \
        if (overlap < 0) return false; /* separating axis found */     \
        /* Compare: overlap/|ax| < min_overlap_raw/|best_ax|           \
           => overlap² * min_len_sq < min_overlap_raw² * ax_len_sq */  \
        if (!found_any ||                                              \
            (int64_t)overlap * overlap * (int64_t)min_len_sq <         \
            (int64_t)min_overlap_raw * min_overlap_raw * (int64_t)ax_len_sq) { \
            min_overlap_raw = overlap;                                 \
            min_len_sq = ax_len_sq;                                    \
            best_axis_raw = ax;                                        \
            best_d_neg = nea_v3_dot(d, ax) < 0;                       \
            found_any = true;                                          \
        }                                                              \
    } while (0)

    // 3 face normals of A
    for (int i = 0; i < 3; i++)
        TEST_AXIS(axes_a[i]);

    // 3 face normals of B
    for (int i = 0; i < 3; i++)
        TEST_AXIS(axes_b[i]);

    // 9 edge cross products
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            TEST_AXIS(nea_v3_cross(axes_a[i], axes_b[j]));

    #undef TEST_AXIS

    // Normalize only the winning axis (1 normalize instead of 15)
    int32_t ax_len = nea_v3_length(best_axis_raw);
    nea_vec3_t best_axis;
    int32_t min_overlap;
    if (ax_len > 0)
    {
        best_axis = nea_v3(
            (int32_t)(((int64_t)best_axis_raw.x << 12) / ax_len),
            (int32_t)(((int64_t)best_axis_raw.y << 12) / ax_len),
            (int32_t)(((int64_t)best_axis_raw.z << 12) / ax_len));
        min_overlap = (int32_t)(((int64_t)min_overlap_raw << 12) / ax_len);
    }
    else
    {
        best_axis = nea_v3(0, 1 << 12, 0);
        min_overlap = min_overlap_raw;
    }
    if (best_d_neg)
        best_axis = nea_v3_neg(best_axis);

    // ---- Multi-contact generation via vertex-in-box + edge clipping ----
    nea_vec3_t verts_a[8], verts_b[8];
    nea_rb7_obb_vertices(a, verts_a);
    nea_rb7_obb_vertices(b, verts_b);

    // Temp contact buffer for this pair (keep deepest 4)
    nea_vec3_t pair_points[NEA_RB7_MAX_PAIR_CONTACTS];
    int32_t pair_depths[NEA_RB7_MAX_PAIR_CONTACTS];
    int pair_count = 0;

    // Helper: insert a contact, replacing shallowest if full
    #define PAIR_ADD(pt, depth)                                          \
    do {                                                                 \
        if (pair_count < NEA_RB7_MAX_PAIR_CONTACTS) {                    \
            pair_points[pair_count] = (pt);                              \
            pair_depths[pair_count] = (depth);                           \
            pair_count++;                                                \
        } else {                                                         \
            int _sh = 0;                                                 \
            for (int _q = 1; _q < NEA_RB7_MAX_PAIR_CONTACTS; _q++)      \
                if (pair_depths[_q] < pair_depths[_sh]) _sh = _q;       \
            if ((depth) > pair_depths[_sh]) {                            \
                pair_points[_sh] = (pt);                                 \
                pair_depths[_sh] = (depth);                              \
            }                                                            \
        }                                                                \
    } while (0)

    // 1) Vertex-in-box: test A's vertices against B
    for (int i = 0; i < 8; i++)
    {
        nea_vec3_t local = nea_world_to_local(b, verts_a[i]);
        int32_t px = b->size.x - nea_abs(local.x);
        int32_t py = b->size.y - nea_abs(local.y);
        int32_t pz = b->size.z - nea_abs(local.z);
        if (px > 0 && py > 0 && pz > 0)
        {
            int32_t pen = px;
            if (py < pen) pen = py;
            if (pz < pen) pen = pz;
            PAIR_ADD(verts_a[i], pen);
        }
    }

    // 2) Vertex-in-box: test B's vertices against A
    for (int i = 0; i < 8; i++)
    {
        nea_vec3_t local = nea_world_to_local(a, verts_b[i]);
        int32_t px = a->size.x - nea_abs(local.x);
        int32_t py = a->size.y - nea_abs(local.y);
        int32_t pz = a->size.z - nea_abs(local.z);
        if (px > 0 && py > 0 && pz > 0)
        {
            int32_t pen = px;
            if (py < pen) pen = py;
            if (pz < pen) pen = pz;
            PAIR_ADD(verts_b[i], pen);
        }
    }

    // 3) Edge clipping: only if vertex-in-box found fewer than 2 contacts
    //    (needed for edge-edge collisions where no vertex is inside)
    if (pair_count < 2)
    {
        // Clip edges of A against B
        for (int e = 0; e < 12; e++)
        {
            nea_vec3_t w0 = verts_a[nea_obb_edges[e][0]];
            nea_vec3_t w1 = verts_a[nea_obb_edges[e][1]];
            nea_vec3_t l0 = nea_world_to_local(b, w0);
            nea_vec3_t l1 = nea_world_to_local(b, w1);

            // Quick reject: both endpoints outside same slab
            if ((l0.x < -b->size.x && l1.x < -b->size.x) ||
                (l0.x >  b->size.x && l1.x >  b->size.x) ||
                (l0.y < -b->size.y && l1.y < -b->size.y) ||
                (l0.y >  b->size.y && l1.y >  b->size.y) ||
                (l0.z < -b->size.z && l1.z < -b->size.z) ||
                (l0.z >  b->size.z && l1.z >  b->size.z))
                continue;

            int32_t t_min, t_max;
            if (!nea_clip_segment_aabb(l0, l1, b->size.x, b->size.y, b->size.z,
                                       &t_min, &t_max))
                continue;

            // Contact at midpoint of clipped segment (world space)
            int32_t t_mid = (t_min + t_max) >> 1;
            nea_vec3_t contact = nea_v3(
                w0.x + (int32_t)(((int64_t)(w1.x - w0.x) * t_mid) >> 12),
                w0.y + (int32_t)(((int64_t)(w1.y - w0.y) * t_mid) >> 12),
                w0.z + (int32_t)(((int64_t)(w1.z - w0.z) * t_mid) >> 12));

            // Penetration: distance from midpoint to B's nearest face
            nea_vec3_t lm = nea_v3(
                l0.x + (int32_t)(((int64_t)(l1.x - l0.x) * t_mid) >> 12),
                l0.y + (int32_t)(((int64_t)(l1.y - l0.y) * t_mid) >> 12),
                l0.z + (int32_t)(((int64_t)(l1.z - l0.z) * t_mid) >> 12));
            int32_t px = b->size.x - nea_abs(lm.x);
            int32_t py = b->size.y - nea_abs(lm.y);
            int32_t pz = b->size.z - nea_abs(lm.z);
            int32_t pen = px;
            if (py < pen) pen = py;
            if (pz < pen) pen = pz;
            if (pen > 0)
                PAIR_ADD(contact, pen);
        }

        // Clip edges of B against A
        for (int e = 0; e < 12; e++)
        {
            nea_vec3_t w0 = verts_b[nea_obb_edges[e][0]];
            nea_vec3_t w1 = verts_b[nea_obb_edges[e][1]];
            nea_vec3_t l0 = nea_world_to_local(a, w0);
            nea_vec3_t l1 = nea_world_to_local(a, w1);

            if ((l0.x < -a->size.x && l1.x < -a->size.x) ||
                (l0.x >  a->size.x && l1.x >  a->size.x) ||
                (l0.y < -a->size.y && l1.y < -a->size.y) ||
                (l0.y >  a->size.y && l1.y >  a->size.y) ||
                (l0.z < -a->size.z && l1.z < -a->size.z) ||
                (l0.z >  a->size.z && l1.z >  a->size.z))
                continue;

            int32_t t_min, t_max;
            if (!nea_clip_segment_aabb(l0, l1, a->size.x, a->size.y, a->size.z,
                                       &t_min, &t_max))
                continue;

            int32_t t_mid = (t_min + t_max) >> 1;
            nea_vec3_t contact = nea_v3(
                w0.x + (int32_t)(((int64_t)(w1.x - w0.x) * t_mid) >> 12),
                w0.y + (int32_t)(((int64_t)(w1.y - w0.y) * t_mid) >> 12),
                w0.z + (int32_t)(((int64_t)(w1.z - w0.z) * t_mid) >> 12));

            nea_vec3_t lm = nea_v3(
                l0.x + (int32_t)(((int64_t)(l1.x - l0.x) * t_mid) >> 12),
                l0.y + (int32_t)(((int64_t)(l1.y - l0.y) * t_mid) >> 12),
                l0.z + (int32_t)(((int64_t)(l1.z - l0.z) * t_mid) >> 12));
            int32_t px = a->size.x - nea_abs(lm.x);
            int32_t py = a->size.y - nea_abs(lm.y);
            int32_t pz = a->size.z - nea_abs(lm.z);
            int32_t pen = px;
            if (py < pen) pen = py;
            if (pz < pen) pen = pz;
            if (pen > 0)
                PAIR_ADD(contact, pen);
        }
    }

    // Add contacts to global list
    if (pair_count > 0)
    {
        for (int i = 0; i < pair_count && nea_rb7_num_contacts < NEA_RB_MAX_CONTACTS; i++)
        {
            nea_rb7_contact_t *cp = &nea_rb7_contacts[nea_rb7_num_contacts++];
            cp->normal = best_axis;
            cp->penetration = pair_depths[i];
            cp->point = pair_points[i];
            cp->body = a;
            cp->target = b;
            cp->type = 0;

            if (pair_depths[i] > a->maxPenetration)
                a->maxPenetration = pair_depths[i];
            if (pair_depths[i] > b->maxPenetration)
                b->maxPenetration = pair_depths[i];

            a->numContacts++;
            b->numContacts++;
        }
    }
    else
    {
        // Fallback: SAT midpoint (near-parallel face-face with no vertex inside)
        if (nea_rb7_num_contacts < NEA_RB_MAX_CONTACTS)
        {
            nea_rb7_contact_t *cp = &nea_rb7_contacts[nea_rb7_num_contacts++];
            cp->normal = best_axis;
            cp->penetration = min_overlap;
            int32_t proj_a_center = nea_v3_dot(a->position, best_axis);
            int32_t proj_b_center = nea_v3_dot(b->position, best_axis);
            int32_t contact_proj = (proj_a_center + proj_b_center) >> 1;
            cp->point = nea_v3_add(a->position,
                nea_v3_scale(best_axis, contact_proj - proj_a_center));
            cp->body = a;
            cp->target = b;
            cp->type = 0;

            if (min_overlap > a->maxPenetration)
                a->maxPenetration = min_overlap;
            if (min_overlap > b->maxPenetration)
                b->maxPenetration = min_overlap;

            a->numContacts++;
            b->numContacts++;
        }
    }

    #undef PAIR_ADD

    return true;
}

// =========================================================================
// OBB-AAR collision (body vs static plane)
// =========================================================================

ARM_CODE static bool nea_rb7_collide_obb_aar(nea_rb7_body_t *body, nea_rb7_aar_t *aar)
{
    nea_vec3_t verts[8];
    nea_rb7_obb_vertices(body, verts);

    bool any_contact = false;

    for (int i = 0; i < 8; i++)
    {
        // Distance from vertex to AAR plane
        nea_vec3_t diff = nea_v3_sub(verts[i], aar->position);
        int32_t dist = nea_v3_dot(diff, aar->normal);

        if (dist < NEA_RB7_PENETRATION_THRESHOLD)
        {
            // Check if vertex is within AAR bounds
            // Project diff onto the two tangent axes of the AAR
            // For axis-aligned normals, the tangent axes are the other two world axes
            nea_vec3_t proj = nea_v3_sub(diff, nea_v3_scale(aar->normal, dist));
            bool inside = true;

            // Check bounds on all 3 axes (only the 2 non-normal ones matter,
            // the normal axis size is effectively infinite)
            if (aar->normal.x == 0)
            {
                if (nea_abs(proj.x) > aar->size.x) inside = false;
            }
            if (aar->normal.y == 0)
            {
                if (nea_abs(proj.y) > aar->size.y) inside = false;
            }
            if (aar->normal.z == 0)
            {
                if (nea_abs(proj.z) > aar->size.z) inside = false;
            }

            if (inside && nea_rb7_num_contacts < NEA_RB_MAX_CONTACTS)
            {
                nea_rb7_contact_t *cp = &nea_rb7_contacts[nea_rb7_num_contacts++];
                cp->point = verts[i];
                cp->normal = aar->normal;
                cp->penetration = -dist; // Positive = overlap
                cp->body = body;         // Owning body
                cp->target = NULL;       // Static
                cp->type = 1;            // body-AAR

                if (cp->penetration > body->maxPenetration)
                    body->maxPenetration = cp->penetration;

                body->numContacts++;
                any_contact = true;
            }
        }
    }

    return any_contact;
}

// =========================================================================
// Collision detection (all pairs)
// =========================================================================

ARM_CODE static void nea_rb7_detect_collisions(void)
{
    nea_rb7_num_contacts = 0;

    for (int i = 0; i < NEA_RB_MAX_BODIES; i++)
    {
        nea_rb7_body_t *a = &nea_rb7_bodies[i];
        if (!a->used || a->sleep)
            continue;

        a->numContacts = 0;
        a->maxPenetration = 0;

        // Body vs body
        for (int j = i + 1; j < NEA_RB_MAX_BODIES; j++)
        {
            nea_rb7_body_t *b = &nea_rb7_bodies[j];
            if (!b->used)
                continue;

            nea_rb7_collide_obb_obb(a, b);
        }

        // Body vs statics
        for (int j = 0; j < NEA_RB_MAX_STATICS; j++)
        {
            if (!nea_rb7_statics[j].used)
                continue;

            nea_rb7_collide_obb_aar(a, &nea_rb7_statics[j]);
        }
    }
}

// =========================================================================
// Impulse-based collision response
// =========================================================================

ARM_CODE static void nea_rb7_resolve_contact(nea_rb7_body_t *body, nea_rb7_contact_t *cp)
{
    nea_vec3_t r1 = nea_v3_sub(cp->point, body->position);

    nea_vec3_t v1 = nea_v3_add(body->velocity,
                                nea_v3_cross(body->angularVelocity, r1));

    int32_t inv_mass1 = body->mass > 0 ?
        (int32_t)(((int64_t)(1 << 12) << 12) / body->mass) : 0;

    if (cp->type == 0 && cp->target != NULL)
    {
        // Body-body contact
        nea_rb7_body_t *other = (nea_rb7_body_t *)cp->target;
        nea_vec3_t r2 = nea_v3_sub(cp->point, other->position);
        nea_vec3_t v2 = nea_v3_add(other->velocity,
                                    nea_v3_cross(other->angularVelocity, r2));

        nea_vec3_t dv = nea_v3_sub(v1, v2);
        int32_t vn = nea_v3_dot(dv, cp->normal);

        // Only resolve if approaching (vn < 0 means moving into contact)
        if (vn >= 0)
            return;

        int32_t inv_mass2 = other->mass > 0 ?
            (int32_t)(((int64_t)(1 << 12) << 12) / other->mass) : 0;

        // Effective mass along normal:
        // 1/m_eff = 1/m1 + 1/m2 + (r1×n)ᵀ I1⁻¹ (r1×n) + (r2×n)ᵀ I2⁻¹ (r2×n)
        nea_vec3_t r1xn = nea_v3_cross(r1, cp->normal);
        nea_vec3_t r2xn = nea_v3_cross(r2, cp->normal);
        nea_vec3_t Ir1 = nea_mat3_mul_vec(body->invWInertia, r1xn);
        nea_vec3_t Ir2 = nea_mat3_mul_vec(other->invWInertia, r2xn);

        int32_t inv_meff = inv_mass1 + inv_mass2 +
            nea_v3_dot(nea_v3_cross(Ir1, r1), cp->normal) +
            nea_v3_dot(nea_v3_cross(Ir2, r2), cp->normal);

        if (inv_meff == 0)
            return;

        // Use average restitution
        int32_t e = (body->restitution + other->restitution) >> 1;

        // Normal impulse: J = -(1+e) * vn / inv_meff
        int32_t jn = (int32_t)(-((int64_t)((1 << 12) + e) * vn) >> 12);
        jn = (int32_t)(((int64_t)jn << 12) / inv_meff);

        // Add penetration bias to prevent sinking
        jn += cp->penetration >> 1;

        if (jn < 0)
            jn = 0;

        nea_vec3_t impulse = nea_v3_scale(cp->normal, jn);

        // Apply to body 1
        body->velocity = nea_v3_add(body->velocity,
                                     nea_v3_scale(impulse, inv_mass1));
        body->angularMomentum = nea_v3_add(body->angularMomentum,
                                            nea_v3_cross(r1, impulse));

        // Apply to body 2 (opposite)
        nea_vec3_t neg_impulse = nea_v3_neg(impulse);
        other->velocity = nea_v3_add(other->velocity,
                                      nea_v3_scale(neg_impulse, inv_mass2));
        other->angularMomentum = nea_v3_add(other->angularMomentum,
                                             nea_v3_cross(r2, neg_impulse));

        // --- Coulomb friction ---
        nea_vec3_t tangent = nea_v3_sub(dv,
            nea_v3_scale(cp->normal, nea_v3_dot(dv, cp->normal)));
        int32_t tang_len = nea_v3_length(tangent);

        if (tang_len > 10)
        {
            tangent = nea_v3_normalize(tangent);

            int32_t vt = nea_v3_dot(dv, tangent);

            // Friction impulse
            nea_vec3_t r1xt = nea_v3_cross(r1, tangent);
            nea_vec3_t r2xt = nea_v3_cross(r2, tangent);
            nea_vec3_t It1 = nea_mat3_mul_vec(body->invWInertia, r1xt);
            nea_vec3_t It2 = nea_mat3_mul_vec(other->invWInertia, r2xt);

            int32_t inv_meff_t = inv_mass1 + inv_mass2 +
                nea_v3_dot(nea_v3_cross(It1, r1), tangent) +
                nea_v3_dot(nea_v3_cross(It2, r2), tangent);

            if (inv_meff_t > 0)
            {
                int32_t jt = (int32_t)(-((int64_t)vt << 12) / inv_meff_t);

                // Coulomb clamp: |jt| <= μ * |jn|
                int32_t mu = (body->friction + other->friction) >> 1;
                int32_t max_jt = (int32_t)(((int64_t)mu * nea_abs(jn)) >> 12);
                jt = nea_clamp(jt, -max_jt, max_jt);

                nea_vec3_t friction_imp = nea_v3_scale(tangent, jt);
                body->velocity = nea_v3_add(body->velocity,
                    nea_v3_scale(friction_imp, inv_mass1));
                body->angularMomentum = nea_v3_add(body->angularMomentum,
                    nea_v3_cross(r1, friction_imp));

                nea_vec3_t neg_fric = nea_v3_neg(friction_imp);
                other->velocity = nea_v3_add(other->velocity,
                    nea_v3_scale(neg_fric, inv_mass2));
                other->angularMomentum = nea_v3_add(other->angularMomentum,
                    nea_v3_cross(r2, neg_fric));
            }
        }

        // Update angular velocities
        body->angularVelocity = nea_mat3_mul_vec(body->invWInertia,
                                                  body->angularMomentum);
        other->angularVelocity = nea_mat3_mul_vec(other->invWInertia,
                                                   other->angularMomentum);
    }
    else
    {
        // Body-AAR (static) contact
        int32_t vn = nea_v3_dot(v1, cp->normal);

        // Only resolve if approaching (vn < 0 means moving into surface)
        if (vn >= 0)
            return;

        nea_vec3_t r1xn = nea_v3_cross(r1, cp->normal);
        nea_vec3_t Ir1 = nea_mat3_mul_vec(body->invWInertia, r1xn);

        int32_t inv_meff = inv_mass1 +
            nea_v3_dot(nea_v3_cross(Ir1, r1), cp->normal);

        if (inv_meff == 0)
            return;

        // Lower restitution for static contacts (less bouncy)
        int32_t e = body->restitution >> 1; // Half of body's restitution

        int32_t jn = (int32_t)(-((int64_t)((1 << 12) + e) * vn) >> 12);
        jn = (int32_t)(((int64_t)jn << 12) / inv_meff);
        jn += cp->penetration >> 1;

        if (jn < 0)
            jn = 0;

        nea_vec3_t impulse = nea_v3_scale(cp->normal, jn);

        body->velocity = nea_v3_add(body->velocity,
                                     nea_v3_scale(impulse, inv_mass1));
        body->angularMomentum = nea_v3_add(body->angularMomentum,
                                            nea_v3_cross(r1, impulse));

        // Coulomb friction against static surface
        nea_vec3_t tangent = nea_v3_sub(v1,
            nea_v3_scale(cp->normal, nea_v3_dot(v1, cp->normal)));
        int32_t tang_len = nea_v3_length(tangent);

        if (tang_len > 10)
        {
            tangent = nea_v3_normalize(tangent);
            int32_t vt = nea_v3_dot(v1, tangent);

            nea_vec3_t r1xt = nea_v3_cross(r1, tangent);
            nea_vec3_t It1 = nea_mat3_mul_vec(body->invWInertia, r1xt);
            int32_t inv_meff_t = inv_mass1 +
                nea_v3_dot(nea_v3_cross(It1, r1), tangent);

            if (inv_meff_t > 0)
            {
                int32_t jt = (int32_t)(-((int64_t)vt << 12) / inv_meff_t);
                int32_t max_jt = (int32_t)(((int64_t)body->friction * nea_abs(jn)) >> 12);
                jt = nea_clamp(jt, -max_jt, max_jt);

                nea_vec3_t friction_imp = nea_v3_scale(tangent, jt);
                body->velocity = nea_v3_add(body->velocity,
                    nea_v3_scale(friction_imp, inv_mass1));
                body->angularMomentum = nea_v3_add(body->angularMomentum,
                    nea_v3_cross(r1, friction_imp));
            }
        }

        body->angularVelocity = nea_mat3_mul_vec(body->invWInertia,
                                                  body->angularMomentum);
    }
}

// Resolve all contacts — simple O(contacts) loop using stored body owner
ARM_CODE static void nea_rb7_resolve_all(void)
{
    for (int c = 0; c < nea_rb7_num_contacts; c++)
    {
        nea_rb7_contact_t *cp = &nea_rb7_contacts[c];
        nea_rb7_body_t *body = (nea_rb7_body_t *)cp->body;

        if (body == NULL || !body->used)
            continue;

        nea_rb7_resolve_contact(body, cp);
    }

    // Position correction: push bodies out of penetration.
    // Apply ONCE per body using the deepest contact to avoid multi-contact
    // over-correction (4 contacts × 80% = 320% pushout without this).
    for (int i = 0; i < NEA_RB_MAX_BODIES; i++)
    {
        nea_rb7_body_t *b = &nea_rb7_bodies[i];
        if (!b->used || b->mass == 0)
            continue;

        // Find the deepest contact involving this body (as owner or target)
        nea_rb7_contact_t *deepest = NULL;
        int32_t max_pen = 0;
        bool as_target = false;

        for (int c = 0; c < nea_rb7_num_contacts; c++)
        {
            nea_rb7_contact_t *cp = &nea_rb7_contacts[c];
            if (cp->body == b)
            {
                if (cp->penetration > max_pen)
                {
                    max_pen = cp->penetration;
                    deepest = cp;
                    as_target = false;
                }
            }
            else if (cp->type == 0 && cp->target == b)
            {
                if (cp->penetration > max_pen)
                {
                    max_pen = cp->penetration;
                    deepest = cp;
                    as_target = true;
                }
            }
        }

        if (deepest == NULL)
            continue;

        // Only correct if penetrating beyond a small slop
        int32_t slop = 20; // ~0.005 in f32, allows small overlap
        int32_t correction = max_pen - slop;
        if (correction <= 0)
            continue;

        // Push out 80% of penetration along normal
        int32_t push = (int32_t)(((int64_t)correction * 3277) >> 12); // 3277/4096 ≈ 0.8

        // Determine push direction (normal points from body to target)
        nea_vec3_t normal = deepest->normal;
        if (as_target)
            normal = nea_v3_neg(normal); // Target gets pushed opposite

        if (deepest->type == 0 && deepest->target != NULL)
        {
            // Body-body: scale push by inverse mass ratio
            nea_rb7_body_t *other = as_target ?
                (nea_rb7_body_t *)deepest->body : (nea_rb7_body_t *)deepest->target;
            int32_t inv_m_self = (int32_t)(((int64_t)(1 << 12) << 12) / b->mass);
            int32_t inv_m_other = other->mass > 0 ?
                (int32_t)(((int64_t)(1 << 12) << 12) / other->mass) : 0;
            int32_t total_inv = inv_m_self + inv_m_other;
            if (total_inv > 0)
            {
                int32_t ratio = (int32_t)(((int64_t)inv_m_self << 12) / total_inv);
                push = (int32_t)(((int64_t)push * ratio) >> 12);
            }
        }

        b->position = nea_v3_add(b->position,
            nea_v3_scale(normal, push));
    }
}

// =========================================================================
// Sleep management
// =========================================================================

static void nea_rb7_check_sleep(nea_rb7_body_t *body)
{
    if (body->mass == 0)
        return;

    // Kinetic energy = ½mv² + ½ω·I·ω
    int32_t lin_e = nea_v3_length_sq(body->velocity);
    int32_t ang_e = nea_v3_dot(body->angularVelocity, body->angularMomentum);
    body->energy = (lin_e >> 1) + (ang_e >> 1);

    if (body->energy >= NEA_RB7_SLEEP_THRESHOLD * NEA_RB7_WAKE_MULTIPLIER)
    {
        body->sleep = false;
        body->sleepCounter = 0;
    }
    else if (body->energy <= NEA_RB7_SLEEP_THRESHOLD)
    {
        body->sleepCounter++;
        if (body->sleepCounter >= NEA_RB7_SLEEP_TIME)
        {
            body->sleep = true;
            body->velocity = nea_v3(0, 0, 0);
            body->angularVelocity = nea_v3(0, 0, 0);
            body->angularMomentum = nea_v3(0, 0, 0);
        }
    }
    else
    {
        body->sleepCounter = 0;
    }
}

// =========================================================================
// Main simulation step
// =========================================================================

// Save/restore helpers for adaptive bisection
static void nea_rb7_save_state(nea_rb7_backup_t backups[])
{
    for (int i = 0; i < NEA_RB_MAX_BODIES; i++)
    {
        nea_rb7_body_t *b = &nea_rb7_bodies[i];
        if (!b->used || b->sleep)
            continue;
        backups[i].position = b->position;
        backups[i].velocity = b->velocity;
        backups[i].angularVelocity = b->angularVelocity;
        backups[i].angularMomentum = b->angularMomentum;
        backups[i].forces = b->forces;
        backups[i].moment = b->moment;
        for (int j = 0; j < 9; j++)
            backups[i].transform[j] = b->transform[j];
    }
}

static void nea_rb7_restore_state(const nea_rb7_backup_t backups[])
{
    for (int i = 0; i < NEA_RB_MAX_BODIES; i++)
    {
        nea_rb7_body_t *b = &nea_rb7_bodies[i];
        if (!b->used || b->sleep)
            continue;
        b->position = backups[i].position;
        b->velocity = backups[i].velocity;
        b->angularVelocity = backups[i].angularVelocity;
        b->angularMomentum = backups[i].angularMomentum;
        b->forces = backups[i].forces;
        b->moment = backups[i].moment;
        for (int j = 0; j < 9; j++)
            b->transform[j] = backups[i].transform[j];
    }
}

// Run one sub-step: apply gravity, integrate, detect, resolve
ARM_CODE static void nea_rb7_substep(int32_t dt)
{
    // Apply gravity
    for (int i = 0; i < NEA_RB_MAX_BODIES; i++)
    {
        nea_rb7_body_t *b = &nea_rb7_bodies[i];
        if (!b->used || b->sleep || b->mass == 0)
            continue;
        int32_t fy = (int32_t)(((int64_t)b->mass * nea_rb7_gravity) >> 12);
        b->forces.y += fy;
    }

    // Integrate
    for (int i = 0; i < NEA_RB_MAX_BODIES; i++)
    {
        nea_rb7_body_t *b = &nea_rb7_bodies[i];
        if (!b->used || b->sleep)
            continue;
        nea_rb7_integrate(b, dt);
    }

    // Detect and resolve
    nea_rb7_detect_collisions();
    nea_rb7_resolve_all();
}

ARM_CODE void nea_rb7_update(void)
{
    if (!nea_rb7_running)
        return;

    // Fixed timestep: dt = 1.0 / (60 * substeps) in f32
    // 60 FPS, 4 substeps = dt ~= 0.00417 = 17 in f32
    int32_t dt = (1 << 12) / (60 * NEA_RB7_SUBSTEPS);
    if (dt < 1)
        dt = 1;

    // Save state before this substep for potential bisection
    static nea_rb7_backup_t backups[NEA_RB_MAX_BODIES];
    nea_rb7_save_state(backups);

    // Try full substep
    nea_rb7_substep(dt);

    // Check for deep penetration — if found, bisect
    int32_t pen_threshold = NEA_RB7_PENETRATION_THRESHOLD * 4;
    for (int bisect = 0; bisect < NEA_RB7_MAX_BISECTIONS; bisect++)
    {
        // Find worst penetration across all bodies
        int32_t worst_pen = 0;
        for (int i = 0; i < NEA_RB_MAX_BODIES; i++)
        {
            if (nea_rb7_bodies[i].used && !nea_rb7_bodies[i].sleep &&
                nea_rb7_bodies[i].maxPenetration > worst_pen)
            {
                worst_pen = nea_rb7_bodies[i].maxPenetration;
            }
        }

        if (worst_pen <= pen_threshold)
            break; // Penetration acceptable

        // Restore and redo with halved timestep.
        // bisect=0: dt/2, 2 steps. bisect=1: dt/4, 4 steps. etc.
        nea_rb7_restore_state(backups);
        dt >>= 1;
        if (dt < 1)
            dt = 1;

        int num_steps = 1 << (bisect + 1); // 2, 4, ...
        for (int s = 0; s < num_steps; s++)
            nea_rb7_substep(dt);
    }
}

void nea_rb7_sleep_check_all(void)
{
    for (int i = 0; i < NEA_RB_MAX_BODIES; i++)
    {
        nea_rb7_body_t *b = &nea_rb7_bodies[i];
        if (!b->used)
            continue;

        nea_rb7_check_sleep(b);
    }
}
