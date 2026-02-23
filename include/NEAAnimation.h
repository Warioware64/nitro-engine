// SPDX-License-Identifier: MIT
//
// Copyright (c) 2022, Antonio Niño Díaz
//
// This file is part of Nitro Engine Advanced

#ifndef NEA_ANIMATION_H__
#define NEA_ANIMATION_H__

/// @file   NEAAnimation.h
/// @brief  Functions to load animations.

/// @defgroup animation_system Animation system
///
/// System to create and manipulate animations.
///
/// @{

#define NEA_DEFAULT_ANIMATIONS 32 ///< Default max number of model animations.

/// Holds information of an animation.
typedef struct {
    bool loadedfromfat; ///< True if it was loaded from a filesystem.
    const void *data;   ///< Pointer to the animation data (DSA file).
} NEA_Animation;

/// Creates a new animation object.
///
/// @return Pointer to the newly created animation.
NEA_Animation *NEA_AnimationCreate(void);

/// Deletes an animation object.
///
/// @param animation Pointer to the animation.
void NEA_AnimationDelete(NEA_Animation *animation);

/// Loads a DSA file in RAM to an animation object.
///
/// @param animation Pointer to the animation.
/// @param pointer Pointer to the file.
/// @return It returns 1 on success.
int NEA_AnimationLoad(NEA_Animation *animation, const void *pointer);

/// Loads a DSA file in FAT to an animation object.
///
/// @param animation Pointer to the animation.
/// @param path Path to the file.
/// @return It returns 1 on success.
int NEA_AnimationLoadFAT(NEA_Animation *animation, const char *path);

/// Deletes all animations.
void NEA_AnimationDeleteAll(void);

/// Resets the animation system and sets the maximun number of animations.
///
/// @param max_animations Number of animations. If it is lower than 1, it
///                       will create space for NEA_DEFAULT_ANIMATIONS.
/// @return Returns 0 on success.
int NEA_AnimationSystemReset(int max_animations);

/// Ends animation system and all memory used by it.
void NEA_AnimationSystemEnd(void);

/// @}

#endif // NEA_ANIMATION_H__
