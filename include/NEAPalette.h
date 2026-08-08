// SPDX-License-Identifier: MIT
//
// Copyright (c) 2008-2022 Antonio Niño Díaz
//
// This file is part of Nitro Engine Advanced

#ifndef NEA_PALETTE_H__
#define NEA_PALETTE_H__

#include <nds.h>

#include "NEAFAT.h"
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
    int index;                    ///< Index to internal palette object
    bool ram_backed;              ///< Set by NEA_PaletteRamInit: palette is kept in main RAM
    void *ram_data;               ///< Owned RAM copy of the palette, or NULL
    u16 ram_numcolors;            ///< Number of colors in the RAM copy
    NEA_TextureFormat ram_format; ///< Format of the RAM copy
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

/// Asynchronously loads a palette from the filesystem into a palette object.
///
/// Unlike NEA_PaletteLoadFAT(), this returns right away and the file is read in
/// the background. The palette is uploaded to VRAM by NEA_AsyncProcess() once
/// the data is in RAM. See @ref async for details.
///
/// Deleting the palette cancels a load that is still in progress.
///
/// @param pal Pointer to the palette object.
/// @param path Path of the palette.
/// @param format Format of the palette.
/// @return Async handle to poll the operation, or NULL on error.
NEA_AsyncFile *NEA_PaletteLoadFATAsync(NEA_Palette *pal, const char *path,
                                       NEA_TextureFormat format);

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

/// Makes a palette keep its data in main RAM instead of palette VRAM.
///
/// Call this right after NEA_PaletteCreate() and before loading any palette.
/// Once a palette is RAM-backed, NEA_PaletteLoad() (and the FAT/size variants)
/// will store a private copy of the palette in main RAM instead of uploading it
/// to palette VRAM. Use NEA_PaletteVramLoad() to upload the palette to VRAM when
/// it is needed, and NEA_PaletteVramUnload() to free the VRAM while keeping the
/// RAM copy. This lets you stream palettes in and out of the small palette VRAM
/// on demand.
///
/// @param palette Palette (must have no palette loaded in VRAM yet).
void NEA_PaletteRamInit(NEA_Palette *palette);

/// Uploads the RAM copy of a RAM-backed palette into palette VRAM.
///
/// The palette must have been set up with NEA_PaletteRamInit() and have a palette
/// loaded (which was stashed in RAM). If the palette is already in VRAM this does
/// nothing and succeeds.
///
/// @param palette RAM-backed palette.
/// @return It returns 1 on success, 0 on error.
int NEA_PaletteVramLoad(NEA_Palette *palette);

/// Frees a RAM-backed palette from palette VRAM, keeping the RAM copy.
///
/// The palette can be uploaded again later with NEA_PaletteVramLoad(). If the
/// palette is not currently in VRAM this does nothing and succeeds. While the
/// palette is not in VRAM, any material using it must not be drawn as textured
/// (see NEA_PaletteUse()).
///
/// @param palette RAM-backed palette.
/// @return It returns 1 on success, 0 on error.
int NEA_PaletteVramUnload(NEA_Palette *palette);

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

/// Gets the number of colors in the palette under active modification.
///
/// @return Returns the number of colors of the palette being modified. 
u16 NEA_PaletteModificationGetNumColors();

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
