// SPDX-License-Identifier: MIT
//
// Copyright (c) 2023 Antonio Niño Díaz
//
// This file is part of Nitro Engine Advanced

#include <nds.h>

#include "NEAMain.h"


static int num_params_for_fifo_cmd(NEA_GfxFifoCmd cmd)
{
    switch (cmd)
    {
    case NEA_FIFO_NOP:
        return 0;
    case NEA_FIFO_MTX_MODE:
        return 1;
    case NEA_FIFO_MTX_PUSH:
        return 0;
    case NEA_FIFO_MTX_POP:
        return 1;
    case NEA_FIFO_MTX_STORE:
        return 1;
    case NEA_FIFO_MTX_RESTORE:
        return 1;
    case NEA_FIFO_MTX_IDENTITY:
        return 0;
    case NEA_FIFO_MTX_LOAD_4x4:
        return 16;
    case NEA_FIFO_MTX_LOAD_4x3:
        return 12;
    case NEA_FIFO_MTX_MULT_4x4:
        return 16;
    case NEA_FIFO_MTX_MULT_4x3:
        return 12;
    case NEA_FIFO_MTX_MULT_3x3:
        return 9;
    case NEA_FIFO_MTX_SCALE:
        return 3;
    case NEA_FIFO_MTX_TRANS:
        return 3;
    case NEA_FIFO_COLOR:
        return 1;
    case NEA_FIFO_NORMAL:
        return 1;
    case NEA_FIFO_TEXCOORD:
        return 1;
    case NEA_FIFO_VTX_16:
        return 2;
    case NEA_FIFO_VTX_10:
        return 1;
    case NEA_FIFO_VTX_XY:
        return 1;
    case NEA_FIFO_VTX_XZ:
        return 1;
    case NEA_FIFO_VTX_YZ:
        return 1;
    case NEA_FIFO_VTX_DIFF:
        return 1;
    case NEA_FIFO_POLYGON_ATTR:
        return 1;
    case NEA_FIFO_TEXIMAGE_PARAM:
        return 1;
    case NEA_FIFO_PLTT_BASE:
        return 1;
    case NEA_FIFO_DIF_AMB:
        return 1;
    case NEA_FIFO_SPE_EMI:
        return 1;
    case NEA_FIFO_LIGHT_VECTOR:
        return 1;
    case NEA_FIFO_LIGHT_COLOR:
        return 1;
    case NEA_FIFO_SHININESS:
        return 32;
    case NEA_FIFO_BEGIN_VTXS:
        return 1;
    case NEA_FIFO_END_VTXS:
        return 0;
    case NEA_FIFO_SWAP_BUFFERS:
        return 1;
    case NEA_FIFO_VIEWPORT:
        return 1;
    case NEA_FIFO_BOX_TEST:
        return 3;
    case NEA_FIFO_POS_TEST:
        return 2;
    case NEA_FIFO_VEC_TEST:
        return 1;
    }

    return 0;
}


void NEA_DisplayListDrawDMA_GFX_FIFO(const void *list)
{
    const uint32_t *p = list;

    NEA_AssertPointer(p, "NULL display list pointer");

    uint32_t words = *p++;

    NEA_Assert(words > 0, "Empty display list");

    DC_FlushRange(p, words * 4);

    // There is a hardware bug that affects DMA when there are multiple channels
    // active, under certain conditions. Instead of checking for said
    // conditions, simply ensure that there are no DMA channels active.
    while (dmaBusy(0) || dmaBusy(1) || dmaBusy(2) || dmaBusy(3));

#ifdef NEA_BLOCKSDS
    dmaSetParams(0, p, (void *)&GFX_FIFO, DMA_FIFO | words);
#else
    DMA_SRC(0) = (uint32_t)p;
    DMA_DEST(0) = (uint32_t)&GFX_FIFO;
    DMA_CR(0) = DMA_FIFO | words;
#endif

    while (dmaBusy(0));
}

void NEA_DisplayListDrawCPU(const void *list)
{
    const uint32_t *p = list;

    NEA_AssertPointer(p, "NULL display list pointer");

    uint32_t words = *p++;

    NEA_Assert(words > 0, "Empty display list");

    while (words--)
        GFX_FIFO = *p++;
}

typedef void (*ne_display_list_draw_fn)(const void *);

static ne_display_list_draw_fn ne_display_list_draw = NEA_DisplayListDrawDMA_GFX_FIFO;

void NEA_DisplayListSetDefaultFunction(NEA_DisplayListDrawFunction type)
{
    if (type == NEA_DL_CPU)
    {
        ne_display_list_draw = NEA_DisplayListDrawCPU;
    }
    else if (type == NEA_DL_DMA_GFX_FIFO)
    {
        ne_display_list_draw = NEA_DisplayListDrawDMA_GFX_FIFO;
    }
    else
    {
        NEA_Assert(0, "Invalid display list function type");
        ne_display_list_draw = NEA_DisplayListDrawDMA_GFX_FIFO;
    }
}

void NEA_DisplayListDrawDefault(const void *list)
{
    ne_display_list_draw(list);
}


void NEA_DisplayListModify(const void *list, void (*modification)(NEA_GfxFifoCmd cmd, void *params))
{
    NEA_AssertPointer(list, "NULL display list pointer");
    NEA_AssertPointer(modification, "NULL modification function");

    uint32_t *dl = (uint32_t *)list;
    int size = *dl++;
    for (int i = 0; i < size;)
    {
        NEA_GfxFifoCmd dlCmd[4];
        char *dlCmdBuff = (char *)dl;
        for (int j = 0; j < 4; j++)
        {
            dlCmd[j] = (NEA_GfxFifoCmd)*dlCmdBuff++;
        }
        dl++;
        i++;
        for (int j = 0; j < 4; j++)
        {
            int dlParamSize = num_params_for_fifo_cmd(dlCmd[j]);
            modification(dlCmd[j], dl);
            dl += dlParamSize;
            i += dlParamSize;
        }
    }
}