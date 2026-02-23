// SPDX-License-Identifier: MIT
//
// Copyright (c) 2008-2022 Antonio Niño Díaz
//
// This file is part of Nitro Engine Advanced

#include "NEAMain.h"

/// @file NEAPhysics.c

static NEA_Physics **NEA_PhysicsPointers;
static bool ne_physics_system_inited = false;

static int NEA_MAX_PHYSICS;

NEA_Physics *NEA_PhysicsCreate(NEA_PhysicsTypes type)
{
    if (!ne_physics_system_inited)
    {
        NEA_DebugPrint("System not initialized");
        return NULL;
    }

    // TODO
    if (type == NEA_BoundingSphere)
    {
        NEA_DebugPrint("Bounding spheres not supported");
        return NULL;
    }
    if (type == NEA_Dot)
    {
        NEA_DebugPrint("Dots not supported");
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

    temp->type = type;
    temp->keptpercent = 50;
    temp->enabled = true;
    temp->physicsgroup = 0;
    temp->oncollision = NEA_ColNothing;

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

void NEA_PhysicsSetRadiusI(NEA_Physics *pointer, int radius)
{
    NEA_AssertPointer(pointer, "NULL pointer");
    NEA_Assert(pointer->type == NEA_BoundingSphere, "Not a bounding shpere");
    NEA_Assert(radius >= 0, "Radius must be positive");
    pointer->radius = radius;
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
    NEA_Assert(pointer->type == NEA_BoundingBox, "Not a bounding box");
    NEA_Assert(x >= 0 && y >= 0 && z >= 0, "Size must be positive!!");
    pointer->xsize = x;
    pointer->ysize = y;
    pointer->zsize = z;
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
    physics->physicsgroup = group;
}

void NEA_PhysicsOnCollision(NEA_Physics *physics, NEA_OnCollision action)
{
    NEA_AssertPointer(physics, "NULL pointer");
    physics->oncollision = action;
}

bool NEA_PhysicsIsColliding(const NEA_Physics *pointer)
{
    NEA_AssertPointer(pointer, "NULL pointer");
    return pointer->iscolliding;
}

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
    NEA_Assert(pointer->type != 0, "Object has no type");

    if (pointer->enabled == false)
        return;

    pointer->iscolliding = false;

    // We change Y speed depending on gravity.
    pointer->yspeed -= pointer->gravity;

    // Now, let's move the object

    // Used in collision checking to simplify the code
    int posx = 0, posy = 0, posz = 0;
    // Position before movement
    int bposx = 0, bposy = 0, bposz = 0;

    NEA_Model *model = pointer->model;
    bposx = model->x;
    bposy = model->y;
    bposz = model->z;
    posx = model->x = model->x + pointer->xspeed;
    posy = model->y = model->y + pointer->yspeed;
    posz = model->z = model->z + pointer->zspeed;

    // Gravity and movement have been applied, time to check collisions...
    bool xenabled = true, yenabled = true, zenabled = true;
    if (bposx == posx)
        xenabled = false;
    if (bposy == posy)
        yenabled = false;
    if (bposz == posz)
        zenabled = false;

    for (int i = 0; i < NEA_MAX_PHYSICS; i++)
    {
        if (NEA_PhysicsPointers[i] == NULL)
            continue;

        // Check that we aren't checking an object with itself
        if (NEA_PhysicsPointers[i] == pointer)
            continue;

        // Check that both objects are in the same group
        if (NEA_PhysicsPointers[i]->physicsgroup != pointer->physicsgroup)
            continue;

        NEA_Physics *otherpointer = NEA_PhysicsPointers[i];
        //Get coordinates
        int otherposx = 0, otherposy = 0, otherposz = 0;

        model = otherpointer->model;
        otherposx = model->x;
        otherposy = model->y;
        otherposz = model->z;

        // Both are boxes
        if (pointer->type == NEA_BoundingBox && otherpointer->type == NEA_BoundingBox)
        {
            bool collision =
                ((abs(posx - otherposx) < (pointer->xsize + otherpointer->xsize) >> 1) &&
                (abs(posy - otherposy) < (pointer->ysize + otherpointer->ysize) >> 1) &&
                (abs(posz - otherposz) < (pointer->zsize + otherpointer->zsize) >> 1));

            if (!collision)
                continue;

            pointer->iscolliding = true;

            if (pointer->oncollision == NEA_ColBounce)
            {
                // Used to reduce speed:
                int temp = (pointer->keptpercent << 12) / 100; // f32 format
                if ((yenabled) && ((abs(bposy - otherposy) >= (pointer->ysize + otherpointer->ysize) >> 1)))
                {
                    yenabled = false;
                    pointer->yspeed += pointer->gravity;

                    if (posy > otherposy)
                        (pointer->model)->y = otherposy + ((pointer->ysize + otherpointer->ysize) >> 1);
                    if (posy < otherposy)
                        (pointer->model)->y = otherposy - ((pointer->ysize + otherpointer->ysize) >> 1);

                    if (pointer->gravity == 0)
                    {
                        pointer->yspeed =
                            -mulf32(temp, pointer->yspeed);
                    }
                    else
                    {
                        if (abs(pointer->yspeed) > NEA_MIN_BOUNCE_SPEED)
                        {
                            pointer->yspeed = -mulf32(temp, pointer->yspeed - pointer->gravity);
                        }
                        else
                        {
                            pointer->yspeed = 0;
                        }
                    }
                }
                else if ((xenabled) && ((abs(bposx - otherposx) >= (pointer->xsize + otherpointer->xsize) >> 1)))
                {
                    xenabled = false;

                    if (posx > otherposx)
                        (pointer->model)->x = otherposx + ((pointer->xsize + otherpointer->xsize) >> 1);
                    if (posx < otherposx)
                        (pointer->model)->x = otherposx - ((pointer->xsize + otherpointer->xsize) >> 1);

                    pointer->xspeed = -mulf32(temp, pointer->xspeed);
                }
                else if ((zenabled) && ((abs(bposz - otherposz) >= (pointer->zsize + otherpointer->zsize) >> 1)))
                {
                    zenabled = false;

                    if (posz > otherposz)
                        (pointer->model)->z = otherposz + ((pointer->zsize + otherpointer->zsize) >> 1);
                    if (posz < otherposz)
                        (pointer->model)->z = otherposz - ((pointer->zsize + otherpointer->zsize) >> 1);

                    pointer->zspeed = -mulf32(temp, pointer->zspeed);
                }
            }
            else if (pointer->oncollision == NEA_ColStop)
            {
                if ((yenabled) && ((abs(bposy - otherposy) >= (pointer->ysize + otherpointer->ysize) >> 1)))
                {
                    yenabled = false;

                    if (posy > otherposy)
                        (pointer->model)->y = otherposy + ((pointer->ysize + otherpointer->ysize) >> 1);
                    if (posy < otherposy)
                        (pointer->model)->y = otherposy - ((pointer->ysize + otherpointer->ysize) >> 1);
                }
                if ((xenabled) && ((abs(bposx - otherposx) >= (pointer->xsize + otherpointer->xsize) >> 1)))
                {
                    xenabled = false;

                    if (posx > otherposx)
                        (pointer->model)->x = otherposx + ((pointer->xsize + otherpointer->xsize) >> 1);
                    if (posx < otherposx)
                        (pointer->model)->x = otherposx - ((pointer->xsize + otherpointer->xsize) >> 1);
                }
                if ((zenabled) && ((abs(bposz - otherposz) >= (pointer->zsize + otherpointer->zsize) >> 1)))
                {
                    zenabled = false;

                    if (posz > otherposz)
                        (pointer->model)->z = otherposz + ((pointer->zsize + otherpointer->zsize) >> 1);
                    if (posz < otherposz)
                        (pointer->model)->z = otherposz - ((pointer->zsize + otherpointer->zsize) >> 1);
                }
                pointer->xspeed = pointer->yspeed = pointer->zspeed = 0;
            }
        }
    }

    // Now, we get the module of speed in order to apply friction.
    if (pointer->friction != 0)
    {
        int32_t spd[3] = { pointer->xspeed, pointer->yspeed, pointer->zspeed };
        int64_t modsqrd = (int64_t)spd[0] * spd[0] + (int64_t)spd[1] * spd[1]
                        + (int64_t)spd[2] * spd[2];

        // This value should be chosen based on time since last update.
        int32_t friction = pointer->friction;
        int64_t diff = modsqrd + (int64_t)-friction * friction;

        // Computing the above is faster than waiting on hw

        // Check if module is very small -> speed = 0
        if (__builtin_expect(diff <= 0, 0))
        {
            pointer->xspeed = pointer->yspeed = pointer->zspeed = 0;
        }
        else
        {
            uint32_t mod = sqrt64(modsqrd);
            div64_asynch((int64_t)(mod-friction) << 32, mod);
            // f < m therefore ((m - f) / m) < 1, therefore ((m - f) << 32 ) / m < (2^32)
            // i.e. the result fits in 32-bit
            uint32_t correction_factor = div64_result();
            int32_t nspd[3];
            #pragma GCC unroll 3
            for (int i = 0; i < 3; i++)
            {
                int32_t t = spd[i];
                int32_t st = t;
                if (t < 0)
                    st = -st;
                st = ((uint64_t)(uint32_t)st * correction_factor) >> 32;
                if (t < 0)
                    st = -st;
                nspd[i] = st;
            }

            pointer->xspeed = nspd[0];
            pointer->yspeed = nspd[1];
            pointer->zspeed = nspd[2];
        }
    }
}

bool NEA_PhysicsCheckCollision(const NEA_Physics *pointer1,
                              const NEA_Physics *pointer2)
{
    NEA_AssertPointer(pointer1, "NULL pointer 1");
    NEA_AssertPointer(pointer2, "NULL pointer 2");
    NEA_Assert(pointer1 != pointer2, "Both objects are the same one");

    // Get coordinates
    int posx = 0, posy = 0, posz = 0;

    NEA_Model *model = pointer1->model;
    posx = model->x;
    posy = model->y;
    posz = model->z;

    int otherposx = 0, otherposy = 0, otherposz = 0;

    model = pointer2->model;
    otherposx = model->x;
    otherposy = model->y;
    otherposz = model->z;

    // Both are boxes
    //if(pointer->type == NEA_BoundingBox && otherpointer->type == NEA_BoundingBox)
    //{
    if ((abs(posx - otherposx) < (pointer1->xsize + pointer2->xsize) >> 1) &&
        (abs(posy - otherposy) < (pointer1->ysize + pointer2->ysize) >> 1) &&
        (abs(posz - otherposz) < (pointer1->zsize + pointer2->zsize) >> 1))
    {
        return true;
    }
    // else if (...) TODO: Spheres and dots

    return false;
}
