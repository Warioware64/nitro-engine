// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Warioware64
//
// This file is part of Nitro Engine Advanced

#include "NEAMain.h"

/// @file NEAParticle.c
///
/// Particle system runtime. See NEAParticle.h for the public API and design.

//-----------------------------------------------------------------------------
// NPE binary file constants
//-----------------------------------------------------------------------------

#define NEA_NPE_MAGIC   0x3145504E  // 'N','P','E','1' little-endian
#define NEA_NPE_VERSION 1

#define NEA_NPE_FLAG_ADDITIVE     (1u << 0)
#define NEA_NPE_FLAG_CONTINUOUS   (1u << 1)
#define NEA_NPE_FLAG_AXIS_ALIGNED (1u << 2)
#define NEA_NPE_FLAG_SPRITESHEET  (1u << 4)
#define NEA_NPE_FLAG_STRETCH      (1u << 5)

#define NEA_NPE_MAX_KEYS 16

//-----------------------------------------------------------------------------
// Internal types
//-----------------------------------------------------------------------------

typedef struct {
    uint16_t t;          // 0..1000 (per-mille of life)
    uint8_t  r, g, b, a;
} ne_color_key_t;

typedef struct {
    uint16_t t;          // 0..1000
    uint16_t size;       // 8.8 fixed
} ne_size_key_t;

typedef struct {
    uint32_t flags;

    // Spawn
    uint16_t max_particles;
    uint16_t emit_rate;      // particles per second, 8.8 fixed
    uint16_t burst_count;
    uint16_t life_min;       // frames
    uint16_t life_max;
    uint16_t cone_spread;    // 0..511

    int32_t  speed_min;      // f32
    int32_t  speed_max;
    int32_t  pos_min[3];     // f32, offset from emitter origin
    int32_t  pos_max[3];
    int16_t  initial_dir[3]; // 1.15 fixed normalized

    // Physics
    int32_t  gravity[3];     // f32 per frame (added to velocity)
    int32_t  drag;           // f32 per frame, velocity *= (1 - drag)

    // Appearance
    uint16_t base_size;      // 8.8 fixed
    uint16_t base_rotation;  // 0..511
    int16_t  ang_vel_min;    // delta rotation per frame
    int16_t  ang_vel_max;

    // Sprite sheet (cols=rows=1 means no sheet)
    uint8_t  sheet_cols;
    uint8_t  sheet_rows;
    uint16_t sheet_fps;

    char     mat_name[32];

    // Keyframes
    uint16_t       num_color_keys;
    uint16_t       num_size_keys;
    ne_color_key_t color_keys[NEA_NPE_MAX_KEYS];
    ne_size_key_t  size_keys[NEA_NPE_MAX_KEYS];
} ne_emitter_params_t;

typedef struct {
    int32_t  px, py, pz;     // position (f32)
    int32_t  vx, vy, vz;     // velocity (f32 per frame)
    uint16_t age;            // frames
    uint16_t life;           // frames
    int16_t  rot;            // 0..511
    int16_t  rot_vel;        // delta per frame
    uint16_t size;           // 8.8 fixed
    uint8_t  alive;
    uint8_t  pad;
    uint8_t  r, g, b, a;
} ne_particle_t;

struct NEA_ParticleEmitter {
    ne_emitter_params_t params;
    ne_particle_t      *pool;
    int                 pool_size;
    int                 alive_count;

    NEA_Material *material;
    int32_t       origin_x, origin_y, origin_z;
    NEA_Model    *attach;

    bool     playing;
    bool     params_loaded;
    int32_t  emit_acc;       // 16.16 accumulator (particles ready to spawn)
    uint32_t seed;
};

//-----------------------------------------------------------------------------
// Globals
//-----------------------------------------------------------------------------

static NEA_ParticleEmitter **ne_part_emitters = NULL;
static int                   ne_part_max = 0;
static NEA_Camera           *ne_part_camera = NULL;
static bool                  ne_part_inited = false;

// PRNG seed bumped each emitter create so emitters start desynced.
static uint32_t ne_part_global_seed = 0x12345678u;

//-----------------------------------------------------------------------------
// xorshift32 PRNG (no malloc, no division, per-emitter state)
//-----------------------------------------------------------------------------

static uint32_t ne_prng(uint32_t *s)
{
    uint32_t x = *s;
    if (x == 0) x = 0xACE1u;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *s = x;
    return x;
}

// Uniform random in [lo, hi].
static int32_t ne_rand_range_i32(uint32_t *s, int32_t lo, int32_t hi)
{
    if (lo >= hi) return lo;
    uint32_t r = ne_prng(s);
    uint64_t span = (uint64_t)(uint32_t)(hi - lo) + 1u;
    return lo + (int32_t)((uint64_t)r * span >> 32);
}

static uint16_t ne_rand_range_u16(uint32_t *s, uint16_t lo, uint16_t hi)
{
    if (lo >= hi) return lo;
    uint32_t r = ne_prng(s);
    return lo + (uint16_t)(r % (uint32_t)(hi - lo + 1));
}

//-----------------------------------------------------------------------------
// NPE binary parsing (alignment-safe byte reads)
//-----------------------------------------------------------------------------

static inline uint16_t ne_rd_u16(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static inline uint32_t ne_rd_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8)
         | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static inline int16_t  ne_rd_s16(const uint8_t *p) { return (int16_t)ne_rd_u16(p); }
static inline int32_t  ne_rd_s32(const uint8_t *p) { return (int32_t)ne_rd_u32(p); }

//-----------------------------------------------------------------------------
// System lifecycle
//-----------------------------------------------------------------------------

int NEA_ParticleSystemReset(int max_emitters)
{
    NEA_ParticleSystemEnd();

    if (max_emitters < 1)
        max_emitters = NEA_DEFAULT_PARTICLE_EMITTERS;

    ne_part_emitters = calloc(max_emitters, sizeof(NEA_ParticleEmitter *));
    if (ne_part_emitters == NULL)
    {
        NEA_DebugPrint("Not enough memory");
        return -1;
    }

    ne_part_max = max_emitters;
    ne_part_camera = NULL;
    ne_part_inited = true;
    return 0;
}

void NEA_ParticleSystemEnd(void)
{
    if (!ne_part_inited)
        return;

    for (int i = 0; i < ne_part_max; i++)
    {
        if (ne_part_emitters[i] != NULL)
        {
            free(ne_part_emitters[i]->pool);
            free(ne_part_emitters[i]);
            ne_part_emitters[i] = NULL;
        }
    }
    free(ne_part_emitters);
    ne_part_emitters = NULL;
    ne_part_max = 0;
    ne_part_camera = NULL;
    ne_part_inited = false;
}

void NEA_ParticleSystemSetCamera(NEA_Camera *cam)
{
    ne_part_camera = cam;
}

NEA_ParticleEmitter *NEA_ParticleEmitterCreate(void)
{
    if (!ne_part_inited)
    {
        NEA_DebugPrint("System not initialized");
        return NULL;
    }

    for (int i = 0; i < ne_part_max; i++)
    {
        if (ne_part_emitters[i] != NULL)
            continue;

        NEA_ParticleEmitter *e = calloc(1, sizeof(NEA_ParticleEmitter));
        if (e == NULL)
        {
            NEA_DebugPrint("Not enough memory");
            return NULL;
        }

        e->seed = ne_part_global_seed;
        ne_part_global_seed = ne_part_global_seed * 1103515245u + 12345u;

        ne_part_emitters[i] = e;
        return e;
    }

    NEA_DebugPrint("No free emitter slot");
    return NULL;
}

void NEA_ParticleEmitterDelete(NEA_ParticleEmitter *emitter)
{
    if (!ne_part_inited || emitter == NULL)
        return;

    for (int i = 0; i < ne_part_max; i++)
    {
        if (ne_part_emitters[i] == emitter)
        {
            free(emitter->pool);
            free(emitter);
            ne_part_emitters[i] = NULL;
            return;
        }
    }
}

//-----------------------------------------------------------------------------
// NPE loader
//-----------------------------------------------------------------------------

// Layout (all little-endian):
//
//   HEADER (16 B)
//     u32 magic ('NPE1'), u32 version, u32 flags, u32 reserved
//
//   EMITTER (80 B)
//     s32 speed_min, speed_max, drag
//     s32[3] pos_min, pos_max, gravity
//     u16 max_particles, emit_rate, burst_count, cone_spread
//     u16 life_min, life_max, base_size, base_rotation
//     s16 ang_vel_min, ang_vel_max
//     s16[3] initial_dir
//     u8 sheet_cols, sheet_rows
//     u16 sheet_fps
//     u8[2] pad
//
//   MATERIAL REF (52 B)
//     char[32] mat_name   // looked up at load time via NEA_MaterialFindByName
//     u8[20] pad
//
//   COLOR KEYS
//     u16 num
//     num * { u16 t, u8 r, u8 g, u8 b, u8 a }   (6 B each)
//
//   SIZE KEYS
//     u16 num
//     num * { u16 t, u16 size }                  (4 B each)

int NEA_ParticleEmitterLoad(NEA_ParticleEmitter *emitter, const void *data)
{
    NEA_AssertPointer(emitter, "NULL emitter");
    NEA_AssertPointer(data, "NULL data");

    if (emitter == NULL || data == NULL)
        return 0;

    const uint8_t *p = data;

    uint32_t magic   = ne_rd_u32(p + 0);
    uint32_t version = ne_rd_u32(p + 4);
    uint32_t flags   = ne_rd_u32(p + 8);

    if (magic != NEA_NPE_MAGIC)
    {
        NEA_DebugPrint("NPE bad magic");
        return 0;
    }
    if (version != NEA_NPE_VERSION)
    {
        NEA_DebugPrint("NPE bad version");
        return 0;
    }
    p += 16;

    ne_emitter_params_t *pp = &emitter->params;
    memset(pp, 0, sizeof(*pp));
    pp->flags = flags;

    pp->speed_min      = ne_rd_s32(p +  0);
    pp->speed_max      = ne_rd_s32(p +  4);
    pp->drag           = ne_rd_s32(p +  8);
    for (int i = 0; i < 3; i++) pp->pos_min[i]  = ne_rd_s32(p + 12 + i*4);
    for (int i = 0; i < 3; i++) pp->pos_max[i]  = ne_rd_s32(p + 24 + i*4);
    for (int i = 0; i < 3; i++) pp->gravity[i]  = ne_rd_s32(p + 36 + i*4);
    pp->max_particles  = ne_rd_u16(p + 48);
    pp->emit_rate      = ne_rd_u16(p + 50);
    pp->burst_count    = ne_rd_u16(p + 52);
    pp->cone_spread    = ne_rd_u16(p + 54);
    pp->life_min       = ne_rd_u16(p + 56);
    pp->life_max       = ne_rd_u16(p + 58);
    pp->base_size      = ne_rd_u16(p + 60);
    pp->base_rotation  = ne_rd_u16(p + 62);
    pp->ang_vel_min    = ne_rd_s16(p + 64);
    pp->ang_vel_max    = ne_rd_s16(p + 66);
    for (int i = 0; i < 3; i++) pp->initial_dir[i] = ne_rd_s16(p + 68 + i*2);
    pp->sheet_cols     = p[74];
    pp->sheet_rows     = p[75];
    pp->sheet_fps      = ne_rd_u16(p + 76);
    // p[78..79] pad

    p += 80;

    // Material reference (52 bytes: 32 name + 20 pad). Empty name = leave the
    // material unbound; the caller is expected to assign one with
    // NEA_ParticleEmitterSetMaterial().
    memcpy(pp->mat_name, p, 32);
    pp->mat_name[31] = '\0';
    p += 52;

    if (pp->mat_name[0] != '\0')
    {
        NEA_Material *mat = NEA_MaterialFindByName(pp->mat_name);
        if (mat != NULL)
            emitter->material = mat;
    }

    // Color keys
    uint16_t nck = ne_rd_u16(p); p += 2;
    if (nck > NEA_NPE_MAX_KEYS)
    {
        NEA_DebugPrint("Too many color keys");
        return 0;
    }
    pp->num_color_keys = nck;
    for (int i = 0; i < nck; i++)
    {
        pp->color_keys[i].t = ne_rd_u16(p);
        pp->color_keys[i].r = p[2];
        pp->color_keys[i].g = p[3];
        pp->color_keys[i].b = p[4];
        pp->color_keys[i].a = p[5];
        p += 6;
    }

    // Size keys
    uint16_t nsk = ne_rd_u16(p); p += 2;
    if (nsk > NEA_NPE_MAX_KEYS)
    {
        NEA_DebugPrint("Too many size keys");
        return 0;
    }
    pp->num_size_keys = nsk;
    for (int i = 0; i < nsk; i++)
    {
        pp->size_keys[i].t    = ne_rd_u16(p);
        pp->size_keys[i].size = ne_rd_u16(p + 2);
        p += 4;
    }

    // Allocate / reset the particle pool.
    int desired = pp->max_particles ? pp->max_particles : NEA_PARTICLE_DEFAULT_POOL;
    if (emitter->pool == NULL || emitter->pool_size != desired)
    {
        free(emitter->pool);
        emitter->pool = calloc(desired, sizeof(ne_particle_t));
        if (emitter->pool == NULL)
        {
            emitter->pool_size = 0;
            NEA_DebugPrint("Pool allocation failed");
            return 0;
        }
        emitter->pool_size = desired;
    }
    else
    {
        memset(emitter->pool, 0, sizeof(ne_particle_t) * desired);
    }

    emitter->alive_count   = 0;
    emitter->emit_acc      = 0;
    emitter->params_loaded = true;

    return 1;
}

int NEA_ParticleEmitterLoadFAT(NEA_ParticleEmitter *emitter, const char *path)
{
    NEA_AssertPointer(emitter, "NULL emitter");
    NEA_AssertPointer(path, "NULL path");

    void *buf = NEA_FATLoadData(path);
    if (buf == NULL)
        return 0;

    int ok = NEA_ParticleEmitterLoad(emitter, buf);
    free(buf);
    return ok;
}

typedef struct {
    NEA_ParticleEmitter *emitter;
} ne_async_part_param;

static void ne_async_part_finalize(NEA_AsyncFile *job)
{
    ne_async_part_param *p = __NEA_AsyncParam(job);
    char *buf = __NEA_AsyncBuffer(job, NULL);
    int ok = (buf != NULL) ? NEA_ParticleEmitterLoad(p->emitter, buf) : 0;
    __NEA_AsyncSetResult(job, ok);
}

NEA_AsyncFile *NEA_ParticleEmitterLoadFATAsync(NEA_ParticleEmitter *emitter,
                                               const char *path)
{
    NEA_AssertPointer(emitter, "NULL emitter");
    NEA_AssertPointer(path, "NULL path");

    ne_async_part_param *p = malloc(sizeof(ne_async_part_param));
    if (p == NULL)
    {
        NEA_DebugPrint("Not enough memory");
        return NULL;
    }
    p->emitter = emitter;

    NEA_AsyncFile *job = __NEA_AsyncQueue(path, NULL, ne_async_part_finalize,
                                          NULL, p);
    if (job == NULL)
        free(p);
    return job;
}

//-----------------------------------------------------------------------------
// Setters / control
//-----------------------------------------------------------------------------

void NEA_ParticleEmitterSetMaterial(NEA_ParticleEmitter *emitter,
                                    NEA_Material *material)
{
    if (emitter == NULL) return;
    emitter->material = material;
}

void NEA_ParticleEmitterSetPosition(NEA_ParticleEmitter *emitter,
                                    int x, int y, int z)
{
    if (emitter == NULL) return;
    emitter->origin_x = x;
    emitter->origin_y = y;
    emitter->origin_z = z;
}

void NEA_ParticleEmitterAttachToModel(NEA_ParticleEmitter *emitter,
                                      NEA_Model *model)
{
    if (emitter == NULL) return;
    emitter->attach = model;
}

void NEA_ParticleEmitterPlay(NEA_ParticleEmitter *emitter)
{
    if (emitter == NULL) return;
    emitter->playing = true;
}

void NEA_ParticleEmitterPause(NEA_ParticleEmitter *emitter)
{
    if (emitter == NULL) return;
    emitter->playing = false;
}

void NEA_ParticleEmitterStop(NEA_ParticleEmitter *emitter)
{
    if (emitter == NULL) return;
    emitter->playing = false;
    emitter->emit_acc = 0;
    if (emitter->pool != NULL)
        memset(emitter->pool, 0, sizeof(ne_particle_t) * emitter->pool_size);
    emitter->alive_count = 0;
}

int NEA_ParticleEmitterAliveCount(const NEA_ParticleEmitter *emitter)
{
    return (emitter != NULL) ? emitter->alive_count : 0;
}

//-----------------------------------------------------------------------------
// Spawning
//-----------------------------------------------------------------------------

// Builds a unit vector inside a cone whose axis is `initial_dir` (1.15 fixed
// normalized) and whose half-angle is cone_spread (0..511). Returns the
// vector as f32.
static void ne_random_direction(NEA_ParticleEmitter *e, int32_t out[3])
{
    int16_t dx = e->params.initial_dir[0];
    int16_t dy = e->params.initial_dir[1];
    int16_t dz = e->params.initial_dir[2];

    // If no initial direction, pick a random unit vector on the sphere.
    if (dx == 0 && dy == 0 && dz == 0)
    {
        int theta = (int)ne_rand_range_u16(&e->seed, 0, 511);
        int phi   = (int)ne_rand_range_u16(&e->seed, 0, 511);
        int32_t st = sinLerp(theta << 6);
        int32_t ct = cosLerp(theta << 6);
        int32_t sp = sinLerp(phi << 6);
        int32_t cp = cosLerp(phi << 6);
        out[0] = mulf32(st, cp);
        out[1] = ct;
        out[2] = mulf32(st, sp);
        return;
    }

    // Convert initial dir from 1.15 fixed to f32 (1.19.12): drop 3 fraction
    // bits (15 - 12 = 3).
    int32_t fx = (int32_t)dx >> 3;
    int32_t fy = (int32_t)dy >> 3;
    int32_t fz = (int32_t)dz >> 3;

    if (e->params.cone_spread == 0)
    {
        out[0] = fx; out[1] = fy; out[2] = fz;
        return;
    }

    // Build an orthonormal basis (right, up) perpendicular to the direction.
    // up_hint = world up if dir is not nearly vertical, else world X.
    int32_t up_hint[3];
    if ((fy > floattof32(0.98f)) || (fy < -floattof32(0.98f)))
    {
        up_hint[0] = floattof32(1.0f); up_hint[1] = 0; up_hint[2] = 0;
    }
    else
    {
        up_hint[0] = 0; up_hint[1] = floattof32(1.0f); up_hint[2] = 0;
    }
    int32_t dir_v[3] = { fx, fy, fz };
    int32_t right_v[3], up_v[3];
    crossf32(dir_v, up_hint, right_v);
    normalizef32(right_v);
    crossf32(dir_v, right_v, up_v);
    normalizef32(up_v);

    // Pick a random angle within the cone.
    int half = (int)e->params.cone_spread;
    int theta = (int)ne_rand_range_u16(&e->seed, 0, (uint16_t)half);
    int phi   = (int)ne_rand_range_u16(&e->seed, 0, 511);

    int32_t st = sinLerp(theta << 6);
    int32_t ct = cosLerp(theta << 6);
    int32_t sp = sinLerp(phi << 6);
    int32_t cp = cosLerp(phi << 6);

    int32_t off_r = mulf32(st, cp);
    int32_t off_u = mulf32(st, sp);

    for (int k = 0; k < 3; k++)
    {
        out[k] = mulf32(ct, dir_v[k])
               + mulf32(off_r, right_v[k])
               + mulf32(off_u, up_v[k]);
    }
}

// Locates a free particle slot, returns its index or -1 if pool is full.
static int ne_find_free_slot(NEA_ParticleEmitter *e)
{
    for (int i = 0; i < e->pool_size; i++)
        if (!e->pool[i].alive)
            return i;
    return -1;
}

// ARM mode: 15 __aeabi_lmul calls in Thumb, paid once per particle spawned.
ARM_CODE
static void ne_spawn_one(NEA_ParticleEmitter *e)
{
    int slot = ne_find_free_slot(e);
    if (slot < 0)
        return;

    ne_particle_t *pa = &e->pool[slot];

    // Initial position: emitter origin + random box offset.
    pa->px = e->origin_x + ne_rand_range_i32(&e->seed,
                                e->params.pos_min[0], e->params.pos_max[0]);
    pa->py = e->origin_y + ne_rand_range_i32(&e->seed,
                                e->params.pos_min[1], e->params.pos_max[1]);
    pa->pz = e->origin_z + ne_rand_range_i32(&e->seed,
                                e->params.pos_min[2], e->params.pos_max[2]);

    // Direction within cone, scaled by random speed.
    int32_t dir[3];
    ne_random_direction(e, dir);
    int32_t speed = ne_rand_range_i32(&e->seed,
                                e->params.speed_min, e->params.speed_max);
    pa->vx = mulf32(dir[0], speed);
    pa->vy = mulf32(dir[1], speed);
    pa->vz = mulf32(dir[2], speed);

    pa->age  = 0;
    pa->life = ne_rand_range_u16(&e->seed,
                                e->params.life_min, e->params.life_max);
    if (pa->life == 0)
        pa->life = 60;

    pa->rot     = (int16_t)e->params.base_rotation;
    pa->rot_vel = (int16_t)ne_rand_range_i32(&e->seed,
                            e->params.ang_vel_min, e->params.ang_vel_max);
    pa->size    = e->params.base_size ? e->params.base_size : (1 << 8);

    // Initial color: sample at t=0 if there are keys, else white.
    if (e->params.num_color_keys > 0)
    {
        pa->r = e->params.color_keys[0].r;
        pa->g = e->params.color_keys[0].g;
        pa->b = e->params.color_keys[0].b;
        pa->a = e->params.color_keys[0].a;
    }
    else
    {
        pa->r = pa->g = pa->b = pa->a = 255;
    }

    pa->alive = 1;
    e->alive_count++;
}

void NEA_ParticleEmitterEmitBurst(NEA_ParticleEmitter *emitter, int count)
{
    if (emitter == NULL || !emitter->params_loaded)
        return;
    for (int i = 0; i < count; i++)
        ne_spawn_one(emitter);
}

//-----------------------------------------------------------------------------
// Per-frame update
//-----------------------------------------------------------------------------

// Sample color over life. `t` is 0..1000 (per-mille of life).
static void ne_sample_color(const ne_emitter_params_t *pp, uint16_t t,
                            uint8_t out[4])
{
    if (pp->num_color_keys == 0)
    {
        out[0] = out[1] = out[2] = out[3] = 255;
        return;
    }
    if (pp->num_color_keys == 1 || t <= pp->color_keys[0].t)
    {
        out[0] = pp->color_keys[0].r;
        out[1] = pp->color_keys[0].g;
        out[2] = pp->color_keys[0].b;
        out[3] = pp->color_keys[0].a;
        return;
    }

    int last = pp->num_color_keys - 1;
    if (t >= pp->color_keys[last].t)
    {
        out[0] = pp->color_keys[last].r;
        out[1] = pp->color_keys[last].g;
        out[2] = pp->color_keys[last].b;
        out[3] = pp->color_keys[last].a;
        return;
    }

    // Find the segment containing t.
    int i = 0;
    while (i < last && pp->color_keys[i + 1].t < t)
        i++;

    const ne_color_key_t *a = &pp->color_keys[i];
    const ne_color_key_t *b = &pp->color_keys[i + 1];
    uint32_t span = (uint32_t)(b->t - a->t);
    uint32_t u    = (uint32_t)(t - a->t);
    out[0] = (uint8_t)(a->r + ((int32_t)(b->r - a->r) * (int32_t)u) / (int32_t)span);
    out[1] = (uint8_t)(a->g + ((int32_t)(b->g - a->g) * (int32_t)u) / (int32_t)span);
    out[2] = (uint8_t)(a->b + ((int32_t)(b->b - a->b) * (int32_t)u) / (int32_t)span);
    out[3] = (uint8_t)(a->a + ((int32_t)(b->a - a->a) * (int32_t)u) / (int32_t)span);
}

// Sample size over life. Returns 8.8 fixed.
static uint16_t ne_sample_size(const ne_emitter_params_t *pp, uint16_t t)
{
    if (pp->num_size_keys == 0)
        return pp->base_size ? pp->base_size : (1 << 8);
    if (pp->num_size_keys == 1 || t <= pp->size_keys[0].t)
        return pp->size_keys[0].size;

    int last = pp->num_size_keys - 1;
    if (t >= pp->size_keys[last].t)
        return pp->size_keys[last].size;

    int i = 0;
    while (i < last && pp->size_keys[i + 1].t < t)
        i++;

    const ne_size_key_t *a = &pp->size_keys[i];
    const ne_size_key_t *b = &pp->size_keys[i + 1];
    int32_t span = (int32_t)(b->t - a->t);
    int32_t u    = (int32_t)(t - a->t);
    int32_t v    = (int32_t)a->size + (((int32_t)b->size - (int32_t)a->size) * u) / span;
    if (v < 0) v = 0;
    if (v > 0xFFFF) v = 0xFFFF;
    return (uint16_t)v;
}

static void ne_update_emitter(NEA_ParticleEmitter *e)
{
    if (!e->params_loaded)
        return;

    // Follow attached model.
    if (e->attach != NULL)
    {
        e->origin_x = e->attach->x;
        e->origin_y = e->attach->y;
        e->origin_z = e->attach->z;
    }

    // Continuous emission: accumulate fractional particles. emit_rate is
    // particles per second in 8.8 fixed; convert to per-frame (60 fps) in
    // 16.16 fixed = (rate * 256) / 60.
    if (e->playing && (e->params.flags & NEA_NPE_FLAG_CONTINUOUS)
        && e->params.emit_rate > 0)
    {
        int32_t per_frame_16_16 = ((int32_t)e->params.emit_rate << 8) / 60;
        e->emit_acc += per_frame_16_16;
        while (e->emit_acc >= (1 << 16))
        {
            ne_spawn_one(e);
            e->emit_acc -= (1 << 16);
        }
    }

    // Advance particles.
    const ne_emitter_params_t *pp = &e->params;
    for (int i = 0; i < e->pool_size; i++)
    {
        ne_particle_t *pa = &e->pool[i];
        if (!pa->alive)
            continue;

        // Velocity += gravity, then * (1 - drag).
        pa->vx += pp->gravity[0];
        pa->vy += pp->gravity[1];
        pa->vz += pp->gravity[2];
        if (pp->drag != 0)
        {
            int32_t k = (1 << 12) - pp->drag;
            pa->vx = mulf32(pa->vx, k);
            pa->vy = mulf32(pa->vy, k);
            pa->vz = mulf32(pa->vz, k);
        }

        // Position += velocity.
        pa->px += pa->vx;
        pa->py += pa->vy;
        pa->pz += pa->vz;

        // Rotation.
        pa->rot = (int16_t)((pa->rot + pa->rot_vel) & 0x1FF);

        // Age.
        pa->age++;
        if (pa->age >= pa->life)
        {
            pa->alive = 0;
            e->alive_count--;
            continue;
        }

        // Per-mille time.
        uint16_t t = (uint16_t)(((uint32_t)pa->age * 1000u) / pa->life);
        uint8_t col[4];
        ne_sample_color(pp, t, col);
        pa->r = col[0];
        pa->g = col[1];
        pa->b = col[2];
        pa->a = col[3];
        pa->size = ne_sample_size(pp, t);
    }
}

// ARM mode: only 3 __aeabi_lmul calls, but they are inside the per-particle
// integration loop rather than beside it.
ARM_CODE
void NEA_ParticleUpdateAll(void)
{
    if (!ne_part_inited)
        return;
    for (int i = 0; i < ne_part_max; i++)
        if (ne_part_emitters[i] != NULL)
            ne_update_emitter(ne_part_emitters[i]);
}

//-----------------------------------------------------------------------------
// Draw
//-----------------------------------------------------------------------------

// Compute world-space right and up vectors for billboarding from the camera.
// Returns false if no camera was set.
static bool ne_billboard_basis(int32_t right[3], int32_t up[3])
{
    if (ne_part_camera == NULL)
        return false;

    // forward = normalize(to - from)
    int32_t f[3] = {
        ne_part_camera->to[0] - ne_part_camera->from[0],
        ne_part_camera->to[1] - ne_part_camera->from[1],
        ne_part_camera->to[2] - ne_part_camera->from[2]
    };
    normalizef32(f);

    int32_t up_cam[3] = {
        ne_part_camera->up[0],
        ne_part_camera->up[1],
        ne_part_camera->up[2]
    };

    // right = normalize(cross(forward, up))
    crossf32(f, up_cam, right);
    normalizef32(right);

    // up' = cross(right, forward) — re-orthogonalize.
    crossf32(right, f, up);

    return true;
}

// ARM mode: 8 __aeabi_lmul calls in Thumb, paid per emitter per frame.
ARM_CODE
void NEA_ParticleEmitterDraw(NEA_ParticleEmitter *emitter)
{
    if (emitter == NULL || !emitter->params_loaded
        || emitter->alive_count == 0)
        return;

    // Bind material once.
    NEA_MaterialUse(emitter->material);

    // Choose the billboard basis. Axis-aligned mode uses world X/Y.
    bool axis_aligned = (emitter->params.flags & NEA_NPE_FLAG_AXIS_ALIGNED) != 0;
    int32_t rvec[3], uvec[3];
    if (axis_aligned || !ne_billboard_basis(rvec, uvec))
    {
        rvec[0] = floattof32(1.0f); rvec[1] = 0; rvec[2] = 0;
        uvec[0] = 0; uvec[1] = floattof32(1.0f); uvec[2] = 0;
    }

    // Texture sub-rect computation for sprite-sheet animation.
    int cols  = emitter->params.sheet_cols ? emitter->params.sheet_cols : 1;
    int rows  = emitter->params.sheet_rows ? emitter->params.sheet_rows : 1;
    int nfr   = cols * rows;
    int tw    = (emitter->material != NULL) ? NEA_TextureGetSizeX(emitter->material) : 0;
    int th    = (emitter->material != NULL) ? NEA_TextureGetSizeY(emitter->material) : 0;
    int fw    = (cols > 0) ? tw / cols : tw;
    int fh    = (rows > 0) ? th / rows : th;

    for (int i = 0; i < emitter->pool_size; i++)
    {
        ne_particle_t *pa = &emitter->pool[i];
        if (!pa->alive)
            continue;

        // Half-size as f32: (size_8_8 << 4) gives 12.12, then >>1 for half.
        int32_t hs = ((int32_t)pa->size << 4) >> 1;

        // Rotated quad basis.
        int32_t rs[3], us[3];
        if (pa->rot == 0)
        {
            for (int k = 0; k < 3; k++)
            {
                rs[k] = mulf32(rvec[k], hs);
                us[k] = mulf32(uvec[k], hs);
            }
        }
        else
        {
            int32_t c = cosLerp((int)pa->rot << 6);
            int32_t s = sinLerp((int)pa->rot << 6);
            for (int k = 0; k < 3; k++)
            {
                int32_t rk = mulf32(c, rvec[k]) + mulf32(s, uvec[k]);
                int32_t uk = -mulf32(s, rvec[k]) + mulf32(c, uvec[k]);
                rs[k] = mulf32(rk, hs);
                us[k] = mulf32(uk, hs);
            }
        }

        // Per-particle alpha through the poly format register.
        uint32_t alpha = (uint32_t)(pa->a >> 3) & 0x1F;
        if (alpha == 0)
            continue; // fully transparent: skip submission
        GFX_POLY_FORMAT = POLY_ALPHA(alpha)
                        | POLY_ID(0)
                        | POLY_CULL_NONE;

        // Color modulation (RGB15, no alpha component here).
        GFX_COLOR = RGB15(pa->r >> 3, pa->g >> 3, pa->b >> 3);

        // Sprite-sheet UV (in 1.4 texel-fixed format).
        int s_u0, s_v0, s_u1, s_v1;
        if (nfr <= 1 || fw == 0 || fh == 0)
        {
            s_u0 = 0;
            s_v0 = 0;
            s_u1 = tw << 4;
            s_v1 = th << 4;
        }
        else
        {
            int idx = 0;
            if (emitter->params.sheet_fps > 0)
            {
                uint32_t f = ((uint32_t)pa->age * (uint32_t)emitter->params.sheet_fps) / 60u;
                idx = (int)(f % (uint32_t)nfr);
            }
            int fc = idx % cols;
            int fr = idx / cols;
            s_u0 = (fc * fw) << 4;
            s_v0 = (fr * fh) << 4;
            s_u1 = ((fc + 1) * fw) << 4;
            s_v1 = ((fr + 1) * fh) << 4;
        }

        // Four corners in world space.
        int32_t v00[3], v10[3], v11[3], v01[3];
        for (int k = 0; k < 3; k++)
        {
            int32_t P = (k == 0) ? pa->px : (k == 1) ? pa->py : pa->pz;
            v00[k] = P - rs[k] - us[k]; // bottom-left
            v10[k] = P + rs[k] - us[k]; // bottom-right
            v11[k] = P + rs[k] + us[k]; // top-right
            v01[k] = P - rs[k] + us[k]; // top-left
        }

        // Submit one quad.
        NEA_PolyBegin(GL_QUAD);

        GFX_TEX_COORD = TEXTURE_PACK(s_u0, s_v1);
        NEA_PolyVertexI(v00[0], v00[1], v00[2]);

        GFX_TEX_COORD = TEXTURE_PACK(s_u1, s_v1);
        NEA_PolyVertexI(v10[0], v10[1], v10[2]);

        GFX_TEX_COORD = TEXTURE_PACK(s_u1, s_v0);
        NEA_PolyVertexI(v11[0], v11[1], v11[2]);

        GFX_TEX_COORD = TEXTURE_PACK(s_u0, s_v0);
        NEA_PolyVertexI(v01[0], v01[1], v01[2]);
    }
}
