// SPDX-License-Identifier: MIT
//
// Copyright (c) 2023 Antonio Niño Díaz
//
// This file is part of Nitro Engine Advanced

#ifndef NEA_DISPLAYLIST_H__
#define NEA_DISPLAYLIST_H__

/// @file   NEADisplayList.h
/// @brief  Functions to send display lists to the GPU.

/// @defgroup display_list_system Display list handling system
///
/// Functions to send display lists to the GPU.
///
/// @{

/// Possible ways to send display lists to the GPU.
typedef enum {
    NEA_DL_CPU,          ///< Send all data to the GPU with CPU copy loop.
    NEA_DL_DMA_GFX_FIFO, ///< Default. DMA in GFX FIFO mode (incompatible with safe dual 3D)
    // TODO: Support DMA without GFX FIFO DMA mode, using GFX FIFO IRQ instead.
} NEA_DisplayListDrawFunction;

/// Sends a display list to the GPU by using the DMA in GFX FIFO mode.
///
/// Important note: Don't use this function when using safe dual 3D. Check the
/// documentation of NEA_DisplayListSetDefaultFunction() for more information.
///
/// @param list Pointer to the display list
void NEA_DisplayListDrawDMA_GFX_FIFO(const void *list);

/// Sends a display list to the GPU by using a CPU copy loop.
///
/// @param list Pointer to the display list
void NEA_DisplayListDrawCPU(const void *list);

/// Set the default way to send display lists to the GPU.
///
/// Important note: NEA_DL_DMA_GFX_FIFO isn't compatible with safe dual 3D mode
/// because it uses DMA in horizontal blanking start mode. There is a hardware
/// bug that makes it unreliable to have both DMA channels active at the same
/// time in HBL start and GFX FIFO mode.
///
/// @param type Copy type to use.
void NEA_DisplayListSetDefaultFunction(NEA_DisplayListDrawFunction type);

/// Draw a display list using the selected default function.
///
/// This will use the function selected by NEA_DisplayListSetDefaultFunction().
///
/// @param list Pointer to the display list
void NEA_DisplayListDrawDefault(const void *list);

/// @}

#endif // NEA_DISPLAYLIST_H__
