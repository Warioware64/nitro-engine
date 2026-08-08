// SPDX-License-Identifier: MIT
//
// Copyright (c) 2008-2022 Antonio Niño Díaz
//
// This file is part of Nitro Engine Advanced

#include <nds/arm9/postest.h>

#include "NEAMain.h"

/// @file NEAAnimation.c

static NEA_Animation **NEA_AnimationPointers;
static int NEA_MAX_ANIMATIONS;
static bool ne_animation_system_inited = false;

NEA_Animation *NEA_AnimationCreate(void)
{
    if (!ne_animation_system_inited)
    {
        NEA_DebugPrint("System not initialized");
        return NULL;
    }

    NEA_Animation *animation = calloc(1, sizeof(NEA_Animation));
    if (animation == NULL)
    {
        NEA_DebugPrint("Not enough memory");
        return NULL;
    }

    int i = 0;
    while (1)
    {
        if (NEA_AnimationPointers[i] == NULL)
        {
            NEA_AnimationPointers[i] = animation;
            break;
        }
        i++;
        if (i == NEA_MAX_ANIMATIONS)
        {
            NEA_DebugPrint("No free slots");
            free(animation);
            return NULL;
        }
    }

    return animation;
}

void NEA_AnimationDelete(NEA_Animation *animation)
{
    if (!ne_animation_system_inited)
        return;

    NEA_AssertPointer(animation, "NULL pointer");

    // Abort any asynchronous load that would write into this animation, before
    // the memory it points to goes away.
    __NEA_AsyncCancelTarget(animation);

    int i = 0;
    while (1)
    {
        if (i == NEA_MAX_ANIMATIONS)
        {
            NEA_DebugPrint("Animation not found");
            return;
        }
        if (NEA_AnimationPointers[i] == animation)
        {
            NEA_AnimationPointers[i] = NULL;
            break;
        }
        i++;
    }

    if (animation->loadedfromfat)
        free((void *)animation->data);

    free(animation);
}

int NEA_AnimationLoadFAT(NEA_Animation *animation, const char *dsa_path)
{
    if (!ne_animation_system_inited)
        return 0;

    NEA_AssertPointer(animation, "NULL animation pointer");
    NEA_AssertPointer(dsa_path, "NULL path pointer");

    if (animation->loadedfromfat)
        free((void *)animation->data);

    animation->loadedfromfat = true;

    uint32_t *pointer = (uint32_t *)NEA_FATLoadData(dsa_path);
    if (pointer == NULL)
    {
        NEA_DebugPrint("Couldn't load file from FAT");
        return 0;
    }

    // Check version
    uint32_t version = pointer[0];
    if (version != 1 && version != 2)
    {
        NEA_DebugPrint("file version is %ld, it should be 1 or 2", version);
        free(pointer);
        return 0;
    }

    animation->data = (void *)pointer;
    return 1;
}

// Parameters of an asynchronous NEA_AnimationLoadFATAsync() job.
typedef struct {
    NEA_Animation *animation;
} ne_async_anim_param;

// Runs on the main thread during the vertical blank: validates the DSA data and
// assigns it to the animation object.
static void ne_async_anim_finalize(NEA_AsyncFile *job)
{
    ne_async_anim_param *p = __NEA_AsyncParam(job);

    // Take ownership of the buffer: the animation frees it when it is deleted.
    uint32_t *pointer = (uint32_t *)__NEA_AsyncTakeBuffer(job, NULL);
    if (pointer == NULL)
    {
        __NEA_AsyncSetResult(job, 0);
        return;
    }

    // Check version
    uint32_t version = pointer[0];
    if (version != 1 && version != 2)
    {
        NEA_DebugPrint("file version is %ld, it should be 1 or 2", version);
        free(pointer);
        __NEA_AsyncSetResult(job, 0);
        return;
    }

    // Only replace the old data now that the new data is known to be valid, so
    // that a failed load leaves the animation usable.
    if (p->animation->loadedfromfat)
        free((void *)p->animation->data);

    p->animation->loadedfromfat = true;
    p->animation->data = (void *)pointer;

    __NEA_AsyncSetResult(job, 1);
}

NEA_AsyncFile *NEA_AnimationLoadFATAsync(NEA_Animation *animation,
                                         const char *dsa_path)
{
    if (!ne_animation_system_inited)
        return NULL;

    NEA_AssertPointer(animation, "NULL animation pointer");
    NEA_AssertPointer(dsa_path, "NULL path pointer");

    ne_async_anim_param *p = malloc(sizeof(ne_async_anim_param));
    if (p == NULL)
    {
        NEA_DebugPrint("Not enough memory");
        return NULL;
    }

    p->animation = animation;

    NEA_AsyncFile *job = __NEA_AsyncQueue(dsa_path, NULL,
                                          ne_async_anim_finalize, NULL, p,
                                          animation);
    if (job == NULL)
        free(p);

    return job;
}

int NEA_AnimationLoad(NEA_Animation *animation, const void *dsa_pointer)
{
    if (!ne_animation_system_inited)
        return 0;

    NEA_AssertPointer(animation, "NULL animation pointer");
    NEA_AssertPointer(dsa_pointer, "NULL data pointer");

    if (animation->loadedfromfat)
        free((void *)animation->data);

    animation->loadedfromfat = false;

    const u32 *pointer = dsa_pointer;

    // Check version
    uint32_t version = pointer[0];
    if (version != 1 && version != 2)
    {
        NEA_DebugPrint("file version is %ld, it should be 1 or 2", version);
        return 0;
    }

    animation->data = (void *)pointer;

    return 1;
}

void NEA_AnimationDeleteAll(void)
{
    if (!ne_animation_system_inited)
        return;

    for (int i = 0; i < NEA_MAX_ANIMATIONS; i++)
    {
        if (NEA_AnimationPointers[i] != NULL)
            NEA_AnimationDelete(NEA_AnimationPointers[i]);
    }
}

int NEA_AnimationSystemReset(int max_animations)
{
    if (ne_animation_system_inited)
        NEA_AnimationSystemEnd();

    if (max_animations < 1)
        NEA_MAX_ANIMATIONS = NEA_DEFAULT_ANIMATIONS;
    else
        NEA_MAX_ANIMATIONS = max_animations;

    NEA_AnimationPointers = calloc(NEA_MAX_ANIMATIONS, sizeof(NEA_AnimationPointers));
    if (NEA_AnimationPointers == NULL)
    {
        NEA_DebugPrint("Not enough memory");
        return -1;
    }

    ne_animation_system_inited = true;
    return 0;
}

void NEA_AnimationSystemEnd(void)
{
    if (!ne_animation_system_inited)
        return;

    NEA_AnimationDeleteAll();

    free(NEA_AnimationPointers);

    ne_animation_system_inited = false;
}
