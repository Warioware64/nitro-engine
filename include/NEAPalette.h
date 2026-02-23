// SPDX-License-Identifier: MIT
//
// Copyright (c) 2008-2022 Antonio Niño Díaz
//
// This file is part of Nitro Engine Advanced

#ifndef NEA_PALETTE_H__
#define NEA_PALETTE_H__

#include <nds.h>

#include "NEAPolygon.h"

/// @file   NEAPalette.h
/// @brief  Functions for loading, using and deleting palettes.

/// @defgroup palette_system Palette system
///
/// Functions to load and manipulate texture palettes.
///
/// @{

#define NEA_DEFAULT_PALETTES 64 ///< Default max number of palettes

/// Holds information of a palette.
typedef struct {
    int index; ///< Index to internal palette object
} NEA_Palette;

/// Creates a new palette object.
///
/// @return Pointer to the newly created palette.
NEA_Palette *NEA_PaletteCreate(void);

/// Loads a palette from the filesystem into a palette object.
///
/// @param pal Pointer to the palette object.
/// @param path Path of the palette.
/// @param format Format of the palette.
/// @return It returns 1 on success, 0 on error.
int NEA_PaletteLoadFAT(NEA_Palette *pal, const char *path, NEA_TextureFormat format);

/// Assign a palette in RAM to a palette object, given its number of colors.
///
/// @param pal Pointer to the palette object.
/// @param pointer Pointer to the palette in RAM.
/// @param numcolor Number of colors of the palette.
/// @param format Format of the palette.
/// @return It returns 1 on success, 0 on error.
int NEA_PaletteLoad(NEA_Palette *pal, const void *pointer, u16 numcolor,
                   NEA_TextureFormat format);

/// Assign a palette in RAM to a palette object, given its size.
///
/// This function is like NEA_PaletteLoad(), but it takes the size of the texture
/// instead of the size.
///
/// @param pal Pointer to the palette object.
/// @param pointer Pointer to the palette in RAM.
/// @param size Size of the palette in bytes.
/// @param format Format of the palette.
/// @return It returns 1 on success, 0 on error.
int NEA_PaletteLoadSize(NEA_Palette *pal, const void *pointer, size_t size,
                       NEA_TextureFormat format);

/// Deletes a palette object.
///
/// @param pal Pointer to the palette object.
void NEA_PaletteDelete(NEA_Palette *pal);

/// Tells the GPU to use the palette in the specified object.
///
/// @param pal Pointer to the palette object.
void NEA_PaletteUse(const NEA_Palette *pal);

/// Resets the palette system and sets the new max number of palettes.
///
/// @param max_palettes Max number of palettes. If lower than 1, it will
///                     create space for NEA_DEFAULT_PALETTES.
/// @return Returns 0 on success.
int NEA_PaletteSystemReset(int max_palettes);

/// Returns the available free memory for palettes.
///
/// Note that, even if it is all available, it may not be contiguous, so you may
/// not be able to load a palette because there isn't enough space in any free
/// gap.
///
/// @return Returns the available memory in bytes.
int NEA_PaletteFreeMem(void);

/// Returns the percentage of available free memory for palettes.
///
/// @return Returns the percentage of available memory (0-100).
int NEA_PaletteFreeMemPercent(void);

/// Defragment memory used for palettes.
///
/// WARNING: This function is currently not working.
void NEA_PaletteDefragMem(void);

/// End palette system and free all memory used by it.
void NEA_PaletteSystemEnd(void);

/// Enables modification of the specified palette.
///
/// It unlocks VRAM so that you can modify the palette. It also returns a
/// pointer to the base address of the palette in VRAM.
///
/// Use this during VBL. Remember to use NEA_PaletteModificationEnd() when you
/// finish. If you don't, the GPU won't be able to render textures to the
/// screen.
///
/// @param pal Palette to modify.
/// @return Returns a pointer to the base address of the palette in VRAM.
void *NEA_PaletteModificationStart(const NEA_Palette *pal);

/// Set the desired entry of a palette to a new color.
///
/// Use this during VBL.
///
/// @param colorindex Color index to change.
/// @param color New color.
void NEA_PaletteRGB256SetColor(u8 colorindex, u16 color);

/// Disables modification of palettes.
///
/// Use this during VBL.
void NEA_PaletteModificationEnd(void);

/// @}

#endif // NEA_PALETTE_H__
