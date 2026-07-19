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

/// GFX FIFO commands
typedef enum {
    NEA_FIFO_NOP = 0x00,
    NEA_FIFO_MTX_MODE = 0x10,
    NEA_FIFO_MTX_PUSH = 0x11,
    NEA_FIFO_MTX_POP = 0x12,
    NEA_FIFO_MTX_STORE = 0x13,
    NEA_FIFO_MTX_RESTORE = 0x14,
    NEA_FIFO_MTX_IDENTITY = 0x15,
    NEA_FIFO_MTX_LOAD_4x4 = 0x16,
    NEA_FIFO_MTX_LOAD_4x3 = 0x17,
    NEA_FIFO_MTX_MULT_4x4 = 0x18,
    NEA_FIFO_MTX_MULT_4x3 = 0x19,
    NEA_FIFO_MTX_MULT_3x3 = 0x1A,
    NEA_FIFO_MTX_SCALE = 0x1B,
    NEA_FIFO_MTX_TRANS = 0x1C,
    NEA_FIFO_COLOR = 0x20,
    NEA_FIFO_NORMAL = 0x21,
    NEA_FIFO_TEXCOORD = 0x22,
    NEA_FIFO_VTX_16 = 0x23,
    NEA_FIFO_VTX_10 = 0x24,
    NEA_FIFO_VTX_XY = 0x25,
    NEA_FIFO_VTX_XZ = 0x26,
    NEA_FIFO_VTX_YZ = 0x27,
    NEA_FIFO_VTX_DIFF = 0x28,
    NEA_FIFO_POLYGON_ATTR = 0x29,
    NEA_FIFO_TEXIMAGE_PARAM = 0x2A,
    NEA_FIFO_PLTT_BASE = 0x2B,
    NEA_FIFO_DIF_AMB = 0x30,
    NEA_FIFO_SPE_EMI = 0x31,
    NEA_FIFO_LIGHT_VECTOR = 0x32,
    NEA_FIFO_LIGHT_COLOR = 0x33,
    NEA_FIFO_SHININESS = 0x34,
    NEA_FIFO_BEGIN_VTXS = 0x40,
    NEA_FIFO_END_VTXS = 0x41,
    NEA_FIFO_SWAP_BUFFERS = 0x50,
    NEA_FIFO_VIEWPORT = 0x60,
    NEA_FIFO_BOX_TEST = 0x70,
    NEA_FIFO_POS_TEST = 0x71,
    NEA_FIFO_VEC_TEST = 0x72
} NEA_GfxFifoCmd;


/// Possible ways to send display lists to the GPU.
typedef enum {
    NEA_DL_CPU,          ///< Send all data to the GPU with CPU copy loop.
    NEA_DL_DMA_GFX_FIFO, ///< Default. DMA in GFX FIFO mode (incompatible with safe dual 3D)
    NEA_DL_NDMA_GFX_FIFO,///< DSi-only NDMA in GFX FIFO mode (safe with dual 3D DMA and two-pass). Opt-in via NEA_DisplayListEnableNDMA(); never selected automatically.
    // TODO: Support DMA without GFX FIFO DMA mode, using GFX FIFO IRQ instead.
} NEA_DisplayListDrawFunction;

/// Sends a display list to the GPU by using the DMA in GFX FIFO mode.
///
/// Important note: Don't use this function when using safe dual 3D. Check the
/// documentation of NEA_DisplayListSetDefaultFunction() for more information.
///
/// @param list Pointer to the display list
void NEA_DisplayListDrawDMA_GFX_FIFO(const void *list);

/// Sends a display list to the GPU by using the DSi NDMA in GFX FIFO mode.
///
/// NDMA is a DSi-exclusive DMA engine that is independent from the legacy DS
/// DMA channels. Because of that, this function is safe to use in the modes
/// that keep a legacy DMA channel running continuously (safe dual 3D and the
/// two-pass modes), unlike NEA_DisplayListDrawDMA_GFX_FIFO().
///
/// On a DS (not DSi) this function automatically falls back to a CPU copy loop.
///
/// @param list Pointer to the display list
void NEA_DisplayListDrawNDMA_GFX_FIFO(const void *list);

/// Enables or disables NDMA display lists (DSi only, off by default).
///
/// When enabled on a DSi, the init functions automatically select the NDMA GFX
/// FIFO path (NEA_DL_NDMA_GFX_FIFO) for their display lists, which is faster and
/// works even in the modes that keep a legacy DMA channel running continuously
/// (safe dual 3D, two-pass). On a DS this has no effect (NDMA doesn't exist).
///
/// NDMA is not selected automatically, so you must call this to use it. The
/// change takes effect immediately if a mode is already initialized, and it
/// persists across re-initialization. A manual NEA_DisplayListSetDefaultFunction()
/// call afterwards overrides this choice.
///
/// @param enable true to use NDMA on DSi, false to use the legacy DMA / CPU path.
void NEA_DisplayListEnableNDMA(bool enable);

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


/// Modify a display list in place using a user-provided function
///
/// @param list Pointer to the display list
/// @param modification Function that can be used to modify the display list
void NEA_DisplayListModify(const void *list, void (*modification)(NEA_GfxFifoCmd cmd, void *params));



/// @}

#endif // NEA_DISPLAYLIST_H__
