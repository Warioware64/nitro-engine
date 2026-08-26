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


// GXSTAT (0x04000600) fields used by the IRQ-driven display list feeder.
// Bit 25 = FIFO full, bit 26 = FIFO less than half full (>= 128 free entries),
// bits 30-31 = FIFO IRQ condition (0 = never, 1 = less than half full,
// 2 = empty).
#define NEA_GXSTAT_FIFO_HALF_EMPTY  BIT(26)

// The IRQ condition lives in the high halfword of GXSTAT. Write it through a
// 16-bit pointer so we never write bit 15 of the low halfword, which
// acknowledges (clears) the matrix stack error flag.
#define NEA_GXSTAT_HI               (*(vu16 *)0x04000602)
#define NEA_GXSTAT_HI_IRQ_MASK      (3 << 14)
#define NEA_GXSTAT_HI_IRQ_NEVER     (0 << 14)
#define NEA_GXSTAT_HI_IRQ_HALF      (1 << 14)

// Words pushed per burst by the IRQ-driven paths. Must stay <= 128: a burst is
// only started when GXSTAT says the FIFO is less than half full, which
// guarantees room for 128 entries. Lower this if the continuously-running
// DMA 2 of the dual 3D / two-pass modes shows display glitches on hardware.
#define NEA_GFX_IRQ_BLOCK 112

static void ne_gxfifo_irq_handler(void)
{
    // The condition is level-triggered: acknowledging IF isn't enough, the IRQ
    // would re-fire forever and livelock the console in the IRQ dispatcher.
    // Disarm it here; the waiter re-arms when it needs to sleep again.
    NEA_GXSTAT_HI = (NEA_GXSTAT_HI & ~NEA_GXSTAT_HI_IRQ_MASK)
                  | NEA_GXSTAT_HI_IRQ_NEVER;
}

static bool ne_gxfifo_irq_installed = false;

static void ne_gxfifo_irq_install(void)
{
    if (ne_gxfifo_irq_installed)
        return;

    irqSet(IRQ_GEOMETRY_FIFO, ne_gxfifo_irq_handler);
    irqEnable(IRQ_GEOMETRY_FIFO);
    ne_gxfifo_irq_installed = true;
}

void __NEA_GfxFifoIrqEnd(void)
{
    if (!ne_gxfifo_irq_installed)
        return;

    NEA_GXSTAT_HI = (NEA_GXSTAT_HI & ~NEA_GXSTAT_HI_IRQ_MASK)
                  | NEA_GXSTAT_HI_IRQ_NEVER;
    irqDisable(IRQ_GEOMETRY_FIFO);
    irqClear(IRQ_GEOMETRY_FIFO);
    ne_gxfifo_irq_installed = false;
}

// Block until the GFX FIFO has room for a full NEA_GFX_IRQ_BLOCK burst.
static void ne_gxfifo_wait_room(void)
{
    // Fast path: the GPU is keeping up, no interrupt round trip needed.
    if (GFX_STATUS & NEA_GXSTAT_FIFO_HALF_EMPTY)
        return;

    // Interrupts are off (called from inside an IRQ handler or a critical
    // section): waiting for an IRQ would hang, so spin on the condition.
    if (!REG_IME)
    {
        while (!(GFX_STATUS & NEA_GXSTAT_FIFO_HALF_EMPTY));
        return;
    }

    ne_gxfifo_irq_install();

    while (!(GFX_STATUS & NEA_GXSTAT_FIFO_HALF_EMPTY))
    {
        // Arm inside the loop: the handler disarms on every wake-up.
        NEA_GXSTAT_HI = (NEA_GXSTAT_HI & ~NEA_GXSTAT_HI_IRQ_MASK)
                      | NEA_GXSTAT_HI_IRQ_HALF;

        // waitForSet = 0 on purpose. If the IRQ fired between arming and this
        // call, the BIOS flag is already set and IntrWait returns immediately
        // instead of halting forever waiting for an IRQ that is now disarmed.
        // A stale flag from an earlier wait costs one extra loop iteration and
        // is consumed by that return, so this can neither spin nor hang.
        swiIntrWait(0, IRQ_GEOMETRY_FIFO);
    }

    NEA_GXSTAT_HI = (NEA_GXSTAT_HI & ~NEA_GXSTAT_HI_IRQ_MASK)
                  | NEA_GXSTAT_HI_IRQ_NEVER;
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

// Number of 32-bit words pushed to the GXFIFO on each NDMA "FIFO half-empty"
// trigger. The GXFIFO requests data when it is less than half full (room for
// ~128 entries), so this must stay <= 128. 112 is the well-known GXFIFO-DMA
// block size; tune here if polygons are lost on hardware.
#define NEA_NDMA_GFX_FIFO_BLOCK 112

void NEA_DisplayListDrawNDMA_GFX_FIFO(const void *list)
{
    const uint32_t *p = list;

    NEA_AssertPointer(p, "NULL display list pointer");

    uint32_t words = *p++;

    NEA_Assert(words > 0, "Empty display list");

    // NDMA is a DSi-only engine. On DS, fall back to the CPU copy loop: it is
    // the universally-safe path (legacy DMA GFX FIFO is unsafe in the
    // continuous-DMA modes that select this function).
    if (!isDSiMode())
    {
        while (words--)
            GFX_FIFO = *p++;
        return;
    }

    DC_FlushRange(p, words * 4);

    // Unlike the legacy DMA path, NDMA is a separate engine and is not affected
    // by the legacy multi-channel hardware bug, so there is no need to wait for
    // the legacy DMA channels to be idle. Only drain our own NDMA channel.
    while (ndmaBusy(0));

    REG_NDMA_SRC(0) = (uint32_t)p;
    REG_NDMA_DEST(0) = (uint32_t)&GFX_FIFO;
    REG_NDMA_LENGTH(0) = words;
    REG_NDMA_BLENGTH(0) = NEA_NDMA_GFX_FIFO_BLOCK;
    REG_NDMA_CR(0) = NDMA_ENABLE | NDMA_START_FIFO | NDMA_SRC_INC | NDMA_DST_FIX;

    while (ndmaBusy(0));
}

void NEA_DisplayListDrawDMA_GFX_IRQ(const void *list)
{
    const uint32_t *p = list;

    NEA_AssertPointer(p, "NULL display list pointer");

    uint32_t words = *p++;

    NEA_Assert(words > 0, "Empty display list");

    DC_FlushRange(p, words * 4);

    // Only our own channel needs to be idle. Unlike
    // NEA_DisplayListDrawDMA_GFX_FIFO(), this path uses start mode 0
    // (immediate), so the multi-channel GFX FIFO mode hardware bug doesn't
    // apply, and the channel is never held waiting for the GPU. That makes it
    // safe in the modes that keep a legacy DMA channel running continuously.
    while (dmaBusy(0));

    while (words > 0)
    {
        uint32_t n = words > NEA_GFX_IRQ_BLOCK ? NEA_GFX_IRQ_BLOCK : words;

        ne_gxfifo_wait_room();

#ifdef NEA_BLOCKSDS
        dmaSetParams(0, p, (void *)&GFX_FIFO,
                     DMA_COPY_WORDS | DMA_DST_FIX | n);
#else
        DMA_SRC(0) = (uint32_t)p;
        DMA_DEST(0) = (uint32_t)&GFX_FIFO;
        DMA_CR(0) = DMA_COPY_WORDS | DMA_DST_FIX | n;
#endif

        while (dmaBusy(0));

        p += n;
        words -= n;
    }
}

void NEA_DisplayListDrawNDMA_GFX_IRQ(const void *list)
{
    const uint32_t *p = list;

    NEA_AssertPointer(p, "NULL display list pointer");

    // NDMA is a DSi-only engine. On DS, use the legacy DMA version of this
    // path, which has the same properties (immediate mode bursts gated by the
    // GFX FIFO IRQ) and works everywhere.
    if (!isDSiMode())
    {
        NEA_DisplayListDrawDMA_GFX_IRQ(list);
        return;
    }

    uint32_t words = *p++;

    NEA_Assert(words > 0, "Empty display list");

    DC_FlushRange(p, words * 4);

    while (ndmaBusy(0));

    while (words > 0)
    {
        uint32_t n = words > NEA_GFX_IRQ_BLOCK ? NEA_GFX_IRQ_BLOCK : words;

        ne_gxfifo_wait_room();

        REG_NDMA_SRC(0) = (uint32_t)p;
        REG_NDMA_DEST(0) = (uint32_t)&GFX_FIFO;
        REG_NDMA_LENGTH(0) = n;
        // NDMA_START_NOW overrides the start mode, so the block length is
        // irrelevant here. Set it to the full length anyway.
        REG_NDMA_BLENGTH(0) = n;
        REG_NDMA_CR(0) = NDMA_ENABLE | NDMA_START_NOW | NDMA_SRC_INC
                       | NDMA_DST_FIX;

        while (ndmaBusy(0));

        p += n;
        words -= n;
    }
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
    else if (type == NEA_DL_NDMA_GFX_FIFO)
    {
        ne_display_list_draw = NEA_DisplayListDrawNDMA_GFX_FIFO;
    }
    else if (type == NEA_DL_DMA_GFX_IRQ)
    {
        ne_display_list_draw = NEA_DisplayListDrawDMA_GFX_IRQ;
    }
    else if (type == NEA_DL_NDMA_GFX_IRQ)
    {
        ne_display_list_draw = NEA_DisplayListDrawNDMA_GFX_IRQ;
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