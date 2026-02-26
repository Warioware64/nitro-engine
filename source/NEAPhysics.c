// SPDX-License-Identifier: MIT
//
// Copyright (c) 2008-2022 Antonio Niño Díaz
// Copyright (c) 2024-2026 Warioware64
//
// This file is part of Nitro Engine Advanced

#include "NEAMain.h"

/// @file NEAPhysics.c

static NEA_Physics **NEA_PhysicsPointers;
static bool ne_physics_system_inited = false;

static int NEA_MAX_PHYSICS;

// =========================================================================
// Internal helpers
// =========================================================================

// Get world-space position from a physics object's model.
static inline NEA_Vec3 ne_physics_get_pos(const NEA_Physics *p)
{
    return NEA_Vec3Make(p->model->x, p->model->y, p->model->z);
}

// Set world-space position of a physics object's model.
static inline void ne_physics_set_pos(NEA_Physics *p, NEA_Vec3 pos)
{
    p->model->x = pos.x;
    p->model->y = pos.y;
    p->model->z = pos.z;
}

// Compute separation vector: normal * neg_depth with ceiling for positive
// products. mulf32 truncates toward zero, which under-pushes for positive
// products (non-axis-aligned normals). Ceiling those ensures the push-out
// magnitude is always >= depth. Negative products already over-push via
// arithmetic right shift. For axis-aligned normals (e.g. flat floor),
// products are exact multiples of 4096 so no adjustment is needed.
static inline NEA_Vec3 ne_separation_vec(NEA_Vec3 normal, int32_t neg_depth)
{
    NEA_Vec3 v;
    int64_t px = (int64_t)normal.x * neg_depth;
    int64_t py = (int64_t)normal.y * neg_depth;
    int64_t pz = (int64_t)normal.z * neg_depth;
    v.x = (int32_t)(px >> 12) + (px > 0 && (px & 0xFFF) ? 1 : 0);
    v.y = (int32_t)(py >> 12) + (py > 0 && (py & 0xFFF) ? 1 : 0);
    v.z = (int32_t)(pz >> 12) + (pz > 0 && (pz & 0xFFF) ? 1 : 0);
    return v;
}

// =========================================================================
// Object creation and destruction
// =========================================================================

// Common initialization for both legacy and new creation paths.
static NEA_Physics *ne_physics_create_common(void)
{
    if (!ne_physics_system_inited)
    {
        NEA_DebugPrint("System not initialized");
        return NULL;
    }

    NEA_Physics *temp = calloc(1, sizeof(NEA_Physics));
    if (temp == NULL)
    {
        NEA_DebugPrint("Not enough memory");
        return NULL;
    }

    int i = 0;
    while (1)
    {
        if (i == NEA_MAX_PHYSICS)
        {
            free(temp);
            NEA_DebugPrint("No free slots");
            return NULL;
        }
        if (NEA_PhysicsPointers[i] == NULL)
        {
            NEA_PhysicsPointers[i] = temp;
            break;
        }
        i++;
    }

    // Defaults
    temp->keptpercent = 50;
    temp->enabled = true;
    temp->groupmask = 1;          // Default: group 0 bit set
    temp->oncollision = NEA_ColNothing;
    temp->mass = 0;               // 0 = infinite/static
    temp->restitution = inttof32(1) / 2; // 0.5 default
    temp->on_collision_cb = NULL;
    temp->is_static = false;

    return temp;
}

NEA_Physics *NEA_PhysicsCreate(NEA_PhysicsTypes type)
{
    NEA_Physics *temp = ne_physics_create_common();
    if (temp == NULL)
        return NULL;

    temp->type = type;

    // Map legacy type to new collision shape
    switch (type)
    {
    case NEA_BoundingBox:
        temp->col_shape.type = NEA_COL_AABB;
        break;
    case NEA_BoundingSphere:
        temp->col_shape.type = NEA_COL_SPHERE;
        break;
    case NEA_Dot:
        // Dot is a very small sphere
        temp->col_shape.type = NEA_COL_SPHERE;
        temp->col_shape.shape.sphere.radius = floattof32(0.01);
        break;
    default:
        NEA_DebugPrint("Unknown physics type");
        break;
    }

    return temp;
}

NEA_Physics *NEA_PhysicsCreateEx(NEA_ColShapeType type)
{
    NEA_Physics *temp = ne_physics_create_common();
    if (temp == NULL)
        return NULL;

    temp->col_shape.type = type;

    // Map to legacy type for backward compat
    switch (type)
    {
    case NEA_COL_AABB:
        temp->type = NEA_BoundingBox;
        break;
    case NEA_COL_SPHERE:
        temp->type = NEA_BoundingSphere;
        break;
    case NEA_COL_CAPSULE:
        temp->type = NEA_BoundingSphere; // Closest legacy equivalent
        break;
    default:
        temp->type = NEA_BoundingBox;
        break;
    }

    return temp;
}

void NEA_PhysicsDelete(NEA_Physics *pointer)
{
    if (!ne_physics_system_inited)
        return;

    NEA_AssertPointer(pointer, "NULL pointer");

    int i = 0;
    while (1)
    {
        if (i == NEA_MAX_PHYSICS)
        {
            NEA_DebugPrint("Object not found");
            return;
        }

        if (NEA_PhysicsPointers[i] == pointer)
        {
            NEA_PhysicsPointers[i] = NULL;
            free(pointer);
            return;
        }
        i++;
    }
}

void NEA_PhysicsDeleteAll(void)
{
    if (!ne_physics_system_inited)
        return;

    for (int i = 0; i < NEA_MAX_PHYSICS; i++)
        NEA_PhysicsDelete(NEA_PhysicsPointers[i]);
}

int NEA_PhysicsSystemReset(int max_objects)
{
    if (ne_physics_system_inited)
        NEA_PhysicsSystemEnd();

    if (max_objects < 1)
        NEA_MAX_PHYSICS = NEA_DEFAULT_PHYSICS;
    else
        NEA_MAX_PHYSICS = max_objects;

    NEA_PhysicsPointers = calloc(NEA_MAX_PHYSICS, sizeof(NEA_PhysicsPointers));
    if (NEA_PhysicsPointers == NULL)
    {
        NEA_DebugPrint("Not enough memory");
        return -1;
    }

    ne_physics_system_inited = true;
    return 0;
}

void NEA_PhysicsSystemEnd(void)
{
    if (!ne_physics_system_inited)
        return;

    NEA_PhysicsDeleteAll();

    free(NEA_PhysicsPointers);

    ne_physics_system_inited = false;
}

// =========================================================================
// Setters
// =========================================================================

void NEA_PhysicsSetColShape(NEA_Physics *physics, const NEA_ColShape *shape)
{
    NEA_AssertPointer(physics, "NULL physics pointer");
    NEA_AssertPointer(shape, "NULL shape pointer");
    physics->col_shape = *shape;
}

void NEA_PhysicsSetRadiusI(NEA_Physics *pointer, int radius)
{
    NEA_AssertPointer(pointer, "NULL pointer");
    NEA_Assert(radius >= 0, "Radius must be positive");
    pointer->radius = radius;

    // Sync to collision shape
    if (pointer->col_shape.type == NEA_COL_SPHERE)
        pointer->col_shape.shape.sphere.radius = radius;
    else if (pointer->col_shape.type == NEA_COL_CAPSULE)
        pointer->col_shape.shape.capsule.radius = radius;
}

void NEA_PhysicsSetSpeedI(NEA_Physics *pointer, int x, int y, int z)
{
    NEA_AssertPointer(pointer, "NULL pointer");
    pointer->xspeed = x;
    pointer->yspeed = y;
    pointer->zspeed = z;
}

void NEA_PhysicsSetSizeI(NEA_Physics *pointer, int x, int y, int z)
{
    NEA_AssertPointer(pointer, "NULL pointer");
    NEA_Assert(x >= 0 && y >= 0 && z >= 0, "Size must be positive!!");
    pointer->xsize = x;
    pointer->ysize = y;
    pointer->zsize = z;

    // Sync to collision shape: size is full extent, collision uses half-extent
    if (pointer->col_shape.type == NEA_COL_AABB)
    {
        pointer->col_shape.shape.aabb.half.x = x >> 1;
        pointer->col_shape.shape.aabb.half.y = y >> 1;
        pointer->col_shape.shape.aabb.half.z = z >> 1;
    }
}

void NEA_PhysicsSetGravityI(NEA_Physics *pointer, int gravity)
{
    NEA_AssertPointer(pointer, "NULL pointer");
    pointer->gravity = gravity;
}

void NEA_PhysicsSetFrictionI(NEA_Physics *pointer, int friction)
{
    NEA_AssertPointer(pointer, "NULL pointer");
    NEA_Assert(friction >= 0, "Friction must be positive");
    pointer->friction = friction;
}

void NEA_PhysicsSetBounceEnergy(NEA_Physics *pointer, int percent)
{
    NEA_AssertPointer(pointer, "NULL pointer");
    NEA_Assert(percent >= 0, "Percentage must be positive");
    pointer->keptpercent = percent;
}

void NEA_PhysicsEnable(NEA_Physics *pointer, bool value)
{
    NEA_AssertPointer(pointer, "NULL pointer");
    pointer->enabled = value;
}

void NEA_PhysicsSetModel(NEA_Physics *physics, NEA_Model *modelpointer)
{
    NEA_AssertPointer(physics, "NULL physics pointer");
    NEA_AssertPointer(modelpointer, "NULL model pointer");
    physics->model = modelpointer;
}

void NEA_PhysicsSetGroup(NEA_Physics *physics, int group)
{
    NEA_AssertPointer(physics, "NULL pointer");
    NEA_Assert(group >= 0 && group < 32, "Group must be 0-31");
    physics->groupmask = (uint32_t)1 << group;
}

void NEA_PhysicsSetGroupMask(NEA_Physics *physics, uint32_t mask)
{
    NEA_AssertPointer(physics, "NULL pointer");
    physics->groupmask = mask;
}

void NEA_PhysicsOnCollision(NEA_Physics *physics, NEA_OnCollision action)
{
    NEA_AssertPointer(physics, "NULL pointer");
    physics->oncollision = action;
}

void NEA_PhysicsSetMassI(NEA_Physics *physics, int32_t mass)
{
    NEA_AssertPointer(physics, "NULL pointer");
    NEA_Assert(mass >= 0, "Mass must be non-negative");
    physics->mass = mass;
}

void NEA_PhysicsSetRestitutionI(NEA_Physics *physics, int32_t rest)
{
    NEA_AssertPointer(physics, "NULL pointer");
    physics->restitution = rest;
}

void NEA_PhysicsSetCallback(NEA_Physics *physics, NEA_CollisionCallback cb)
{
    NEA_AssertPointer(physics, "NULL pointer");
    physics->on_collision_cb = cb;
}

void NEA_PhysicsSetStatic(NEA_Physics *physics, bool is_static)
{
    NEA_AssertPointer(physics, "NULL pointer");
    physics->is_static = is_static;
}

bool NEA_PhysicsIsColliding(const NEA_Physics *pointer)
{
    NEA_AssertPointer(pointer, "NULL pointer");
    return pointer->iscolliding;
}

// =========================================================================
// Update
// =========================================================================

void NEA_PhysicsUpdateAll(void)
{
    if (!ne_physics_system_inited)
        return;

    for (int i = 0; i < NEA_MAX_PHYSICS; i++)
    {
        if (NEA_PhysicsPointers[i] != NULL)
            NEA_PhysicsUpdate(NEA_PhysicsPointers[i]);
    }
}

ARM_CODE void NEA_PhysicsUpdate(NEA_Physics *pointer)
{
    if (!ne_physics_system_inited)
        return;

    NEA_AssertPointer(pointer, "NULL pointer");
    NEA_AssertPointer(pointer->model, "NULL model pointer");
    NEA_Assert(pointer->col_shape.type != NEA_COL_NONE, "Object has no shape");

    if (pointer->enabled == false)
        return;

    // Static objects don't update velocity or position
    if (pointer->is_static)
        return;

    pointer->iscolliding = false;

    // Apply gravity on Y axis
    pointer->yspeed -= pointer->gravity;

    // Apply velocity
    NEA_Model *model = pointer->model;
    model->x += pointer->xspeed;
    model->y += pointer->yspeed;
    model->z += pointer->zspeed;

    NEA_Vec3 new_pos = ne_physics_get_pos(pointer);

    // --- Collision detection and response ---

    NEA_Vec3 velocity = NEA_Vec3Make(pointer->xspeed, pointer->yspeed,
                                     pointer->zspeed);

    for (int i = 0; i < NEA_MAX_PHYSICS; i++)
    {
        if (NEA_PhysicsPointers[i] == NULL)
            continue;

        NEA_Physics *other = NEA_PhysicsPointers[i];

        // Skip self
        if (other == pointer)
            continue;

        // Group mask check: collide if masks overlap
        if ((pointer->groupmask & other->groupmask) == 0)
            continue;

        NEA_Vec3 other_pos = ne_physics_get_pos(other);

        // Perform collision test using the new system
        NEA_ColResult result = NEA_ColTest(&pointer->col_shape, new_pos,
                                           &other->col_shape, other_pos);

        if (!result.hit)
            continue;

        pointer->iscolliding = true;

        // Fire callback if set
        if (pointer->on_collision_cb != NULL)
            pointer->on_collision_cb(pointer, other, &result);

        // --- Collision response ---

        if (pointer->oncollision == NEA_ColBounce)
        {
            // Separate objects: push this object out along collision normal
            NEA_Vec3 separation = ne_separation_vec(result.normal,
                                                    -result.depth);
            new_pos = NEA_Vec3Add(new_pos, separation);
            ne_physics_set_pos(pointer, new_pos);

            // Compute velocity component along collision normal
            int32_t v_dot_n = NEA_Vec3Dot(velocity, result.normal);

            if (v_dot_n > 0) // Only respond if moving toward the surface
            {
                // Determine restitution
                int32_t e = (pointer->keptpercent << 12) / 100;

                // Remove normal velocity component and apply restitution
                // v_new = v - (1 + e) * (v . n) * n
                int32_t impulse_scale = mulf32(inttof32(1) + e, v_dot_n);

                // Mass-based impulse distribution
                if (other->mass > 0 && !other->is_static
                    && other->enabled)
                {
                    if (pointer->mass > 0)
                    {
                        // Both dynamic: ratio = m_other / (m_self + m_other)
                        int32_t mass_sum = pointer->mass + other->mass;
                        int32_t ratio_self = divf32(other->mass, mass_sum);
                        int32_t ratio_other = inttof32(1) - ratio_self;

                        // Apply to self
                        NEA_Vec3 self_impulse = NEA_Vec3Scale(
                            result.normal,
                            mulf32(impulse_scale, ratio_self));
                        velocity = NEA_Vec3Sub(velocity, self_impulse);

                        // Apply to other
                        NEA_Vec3 other_vel = NEA_Vec3Make(
                            other->xspeed, other->yspeed, other->zspeed);
                        NEA_Vec3 other_impulse = NEA_Vec3Scale(
                            result.normal,
                            mulf32(-impulse_scale, ratio_other));
                        other_vel = NEA_Vec3Sub(other_vel, other_impulse);
                        other->xspeed = other_vel.x;
                        other->yspeed = other_vel.y;
                        other->zspeed = other_vel.z;
                    }
                    else
                    {
                        // This object has infinite mass, push other fully
                        NEA_Vec3 other_vel = NEA_Vec3Make(
                            other->xspeed, other->yspeed, other->zspeed);
                        NEA_Vec3 other_impulse = NEA_Vec3Scale(
                            result.normal, -impulse_scale);
                        other_vel = NEA_Vec3Sub(other_vel, other_impulse);
                        other->xspeed = other_vel.x;
                        other->yspeed = other_vel.y;
                        other->zspeed = other_vel.z;
                    }
                }
                else
                {
                    // Other object is static/disabled: full impulse on self
                    NEA_Vec3 impulse_vec = NEA_Vec3Scale(result.normal,
                                                        impulse_scale);
                    velocity = NEA_Vec3Sub(velocity, impulse_vec);
                }

                // Minimum bounce speed check on gravity axis
                if (pointer->gravity != 0)
                {
                    if (abs(velocity.y) <= NEA_MIN_BOUNCE_SPEED)
                        velocity.y = 0;
                }
            }

            pointer->xspeed = velocity.x;
            pointer->yspeed = velocity.y;
            pointer->zspeed = velocity.z;
        }
        else if (pointer->oncollision == NEA_ColStop)
        {
            // Push out along normal
            NEA_Vec3 separation = ne_separation_vec(result.normal,
                                                    -result.depth);
            new_pos = NEA_Vec3Add(new_pos, separation);
            ne_physics_set_pos(pointer, new_pos);

            // Zero all velocity
            pointer->xspeed = 0;
            pointer->yspeed = 0;
            pointer->zspeed = 0;
            velocity = NEA_Vec3Make(0, 0, 0);
        }
        else if (pointer->oncollision == NEA_ColSlide)
        {
            // Push out along normal
            NEA_Vec3 separation = ne_separation_vec(result.normal,
                                                    -result.depth);
            new_pos = NEA_Vec3Add(new_pos, separation);
            ne_physics_set_pos(pointer, new_pos);

            // Remove velocity component along the normal (slide)
            int32_t v_dot_n = NEA_Vec3Dot(velocity, result.normal);
            if (v_dot_n > 0)
            {
                NEA_Vec3 normal_vel = NEA_Vec3Scale(result.normal, v_dot_n);
                velocity = NEA_Vec3Sub(velocity, normal_vel);
            }

            pointer->xspeed = velocity.x;
            pointer->yspeed = velocity.y;
            pointer->zspeed = velocity.z;
        }
        // NEA_ColNothing: do nothing (sensor), iscolliding is already set
    }

    // Clamp tiny velocities to zero to prevent drift from fixed-point errors
    if (pointer->iscolliding)
    {
        if (abs(pointer->xspeed) < NEA_MIN_BOUNCE_SPEED)
            pointer->xspeed = 0;
        if (abs(pointer->yspeed) < NEA_MIN_BOUNCE_SPEED)
            pointer->yspeed = 0;
        if (abs(pointer->zspeed) < NEA_MIN_BOUNCE_SPEED)
            pointer->zspeed = 0;
    }

    // --- Friction ---

    if (pointer->friction != 0)
    {
        int32_t spd[3] = { pointer->xspeed, pointer->yspeed,
                           pointer->zspeed };
        int64_t modsqrd = (int64_t)spd[0] * spd[0]
                        + (int64_t)spd[1] * spd[1]
                        + (int64_t)spd[2] * spd[2];

        int32_t friction = pointer->friction;
        int64_t diff = modsqrd + (int64_t)-friction * friction;

        if (__builtin_expect(diff <= 0, 0))
        {
            pointer->xspeed = pointer->yspeed = pointer->zspeed = 0;
        }
        else
        {
            uint32_t mod = sqrt64(modsqrd);
            div64_asynch((int64_t)(mod - friction) << 32, mod);
            uint32_t correction_factor = div64_result();
            int32_t nspd[3];
            #pragma GCC unroll 3
            for (int j = 0; j < 3; j++)
            {
                int32_t t = spd[j];
                int32_t st = t;
                if (t < 0)
                    st = -st;
                st = ((uint64_t)(uint32_t)st * correction_factor) >> 32;
                if (t < 0)
                    st = -st;
                nspd[j] = st;
            }

            pointer->xspeed = nspd[0];
            pointer->yspeed = nspd[1];
            pointer->zspeed = nspd[2];
        }
    }
}

// =========================================================================
// Manual collision check
// =========================================================================

bool NEA_PhysicsCheckCollision(const NEA_Physics *pointer1,
                              const NEA_Physics *pointer2)
{
    NEA_AssertPointer(pointer1, "NULL pointer 1");
    NEA_AssertPointer(pointer2, "NULL pointer 2");
    NEA_Assert(pointer1 != pointer2, "Both objects are the same one");

    NEA_Vec3 pos1 = ne_physics_get_pos(pointer1);
    NEA_Vec3 pos2 = ne_physics_get_pos(pointer2);

    NEA_ColResult result = NEA_ColTest(&pointer1->col_shape, pos1,
                                       &pointer2->col_shape, pos2);
    return result.hit;
}
