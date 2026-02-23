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
    if (version != 1)
    {
        NEA_DebugPrint("file version is %ld, it should be 1", version);
        free(pointer);
        return 0;
    }

    animation->data = (void *)pointer;
    return 1;
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
    if (version != 1)
    {
        NEA_DebugPrint("file version is %ld, it should be 1", version);
        free((void *)pointer);
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
