// SPDX-License-Identifier: MIT
//
// Copyright (c) 2023 Antonio Niño Díaz
//
// This file is part of Nitro Engine Advanced

#include <nds.h>

#include "NEAMain.h"

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
