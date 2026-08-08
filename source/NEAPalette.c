// SPDX-License-Identifier: MIT
//
// Copyright (c) 2008-2022 Antonio Niño Díaz
//
// This file is part of Nitro Engine Advanced

#include "NEAMain.h"
#include "NEAAlloc.h"

/// @file NEAPalette.c

typedef struct {
    u16 *pointer;
    int format;
    u16 numcolors;
} ne_palinfo_t;

static ne_palinfo_t *NEA_PalInfo = NULL;
static NEA_Palette **NEA_UserPalette = NULL;

static NEAChunk *NEA_PalAllocList; // See NEAAlloc.h

static bool ne_palette_system_inited = false;

static int NEA_MAX_PALETTES;

// Which VRAM bank(s) back the palette allocator (snapshot from NEA_GetTexPaletteBank)
static NEA_VRAMBankFlags ne_pal_banks;
// LCD-mode base address of the palette VRAM region (for GFX_PAL_FORMAT offset calc)
static uintptr_t ne_pal_lcd_base;

// Switch palette bank(s) to LCD mode (enables CPU writes)
static void ne_pal_to_lcd(void)
{
    if (ne_pal_banks & NEA_VRAM_E)
        vramSetBankE(VRAM_E_LCD);
    if (ne_pal_banks & NEA_VRAM_F)
        vramSetBankF(VRAM_F_LCD);
    if (ne_pal_banks & NEA_VRAM_G)
        vramSetBankG(VRAM_G_LCD);
}

// Switch palette bank(s) back to TEX_PALETTE mode
static void ne_pal_to_tex(void)
{
    if (ne_pal_banks & NEA_VRAM_E)
        vramSetBankE(VRAM_E_TEX_PALETTE);

    if (ne_pal_banks & NEA_VRAM_F)
        vramSetBankF(VRAM_F_TEX_PALETTE); // Slot 0

    if (ne_pal_banks & NEA_VRAM_G)
    {
        // If F is also used, G goes to slot 1 (after F's slot 0)
        if (ne_pal_banks & NEA_VRAM_F)
            vramSetBankG(VRAM_G_TEX_PALETTE_SLOT1);
        else
            vramSetBankG(VRAM_G_TEX_PALETTE); // Slot 0
    }
}

// Compute LCD address range for the configured palette banks.
// Returns 0 on success, -1 if no banks configured.
static int ne_pal_get_range(void **start, void **end)
{
    if (ne_pal_banks & NEA_VRAM_E)
    {
        // Bank E: 64KB at 0x06880000
        *start = (void *)VRAM_E;
        *end   = (void *)VRAM_F;
    }
    else if (ne_pal_banks & NEA_VRAM_F)
    {
        *start = (void *)VRAM_F;
        if (ne_pal_banks & NEA_VRAM_G)
            *end = (void *)((uintptr_t)VRAM_G + 0x4000); // F+G: 32KB
        else
            *end = (void *)VRAM_G; // F only: 16KB
    }
    else if (ne_pal_banks & NEA_VRAM_G)
    {
        *start = (void *)VRAM_G;
        *end   = (void *)((uintptr_t)VRAM_G + 0x4000); // G only: 16KB
    }
    else
    {
        *start = NULL;
        *end   = NULL;
        return -1;
    }
    return 0;
}

NEA_Palette *NEA_PaletteCreate(void)
{
    if (!ne_palette_system_inited)
    {
        NEA_DebugPrint("System not initialized");
        return NULL;
    }

    for (int i = 0; i < NEA_MAX_PALETTES; i++)
    {
        if (NEA_UserPalette[i] != NULL)
            continue;

        NEA_Palette *ptr = calloc(1, sizeof(NEA_Palette));
        if (ptr == NULL)
        {
            NEA_DebugPrint("Not enough memory");
            return NULL;
        }

        ptr->index = NEA_NO_PALETTE;
        NEA_UserPalette[i] = ptr;
        return ptr;
    }

    NEA_DebugPrint("No free slots");

    return NULL;
}

// Frees only the palette VRAM (and its internal slot), leaving the NEA_Palette
// object alive. This is the palette analog of ne_texture_delete().
static void ne_palette_vram_free(NEA_Palette *pal)
{
    if (pal->index != NEA_NO_PALETTE)
    {
        NEA_Free(NEA_PalAllocList, (void *)NEA_PalInfo[pal->index].pointer);
        NEA_PalInfo[pal->index].pointer = NULL;
        pal->index = NEA_NO_PALETTE;
    }
}

// Keeps a private RAM copy of a palette for a RAM-backed palette instead of
// uploading it to palette VRAM.
static int ne_palette_stash(NEA_Palette *pal, const void *pointer,
                            u16 numcolor, NEA_TextureFormat format)
{
    u32 size = (u32)numcolor << 1; // 2 bytes per color

    // Discard any previous RAM copy (this supports reloading with new data)
    free(pal->ram_data);
    pal->ram_data = malloc(size);
    if (pal->ram_data == NULL)
    {
        NEA_DebugPrint("Not enough memory");
        return 0;
    }

    memcpy(pal->ram_data, pointer, size);
    pal->ram_numcolors = numcolor;
    pal->ram_format = format;

    // If the palette was already resident in VRAM, drop it so the next
    // NEA_PaletteVramLoad() re-uploads this new data instead of keeping the
    // stale VRAM copy.
    ne_palette_vram_free(pal);

    return 1;
}

int NEA_PaletteLoadFAT(NEA_Palette *pal, const char *path, NEA_TextureFormat format)
{
    if (!ne_palette_system_inited)
        return 0;

    NEA_AssertPointer(pal, "NULL palette pointer");
    NEA_AssertPointer(path, "NULL path pointer");

    u32 size = NEA_FATFileSize(path);
    if (size < 1)
    {
        NEA_DebugPrint("Couldn't obtain file size");
        return 0;
    }

    void *ptr = NEA_FATLoadData(path);
    NEA_AssertPointer(ptr, "Couldn't load file from FAT");
    int ret = NEA_PaletteLoadSize(pal, ptr, size, format);
    free(ptr);

    return ret;
}

// Parameters of an asynchronous NEA_PaletteLoadFATAsync() job.
typedef struct {
    NEA_Palette *pal;
    NEA_TextureFormat format;
} ne_async_pal_param;

// Runs on the main thread during the vertical blank: uploads the loaded data to
// palette VRAM with the regular (synchronous) loader.
static void ne_async_pal_finalize(NEA_AsyncFile *job)
{
    ne_async_pal_param *p = __NEA_AsyncParam(job);

    size_t size = 0;
    char *buffer = __NEA_AsyncBuffer(job, &size);

    // Unlike NEA_PaletteLoadFAT(), the size comes from the read itself, so the
    // file doesn't need to be opened a second time to measure it.
    int ret = (buffer != NULL && size > 0)
            ? NEA_PaletteLoadSize(p->pal, buffer, size, p->format) : 0;

    __NEA_AsyncSetResult(job, ret);
}

NEA_AsyncFile *NEA_PaletteLoadFATAsync(NEA_Palette *pal, const char *path,
                                       NEA_TextureFormat format)
{
    if (!ne_palette_system_inited)
        return NULL;

    NEA_AssertPointer(pal, "NULL palette pointer");
    NEA_AssertPointer(path, "NULL path pointer");

    ne_async_pal_param *p = malloc(sizeof(ne_async_pal_param));
    if (p == NULL)
    {
        NEA_DebugPrint("Not enough memory");
        return NULL;
    }

    p->pal = pal;
    p->format = format;

    NEA_AsyncFile *job = __NEA_AsyncQueue(path, NULL, ne_async_pal_finalize,
                                          NULL, p, pal);
    if (job == NULL)
        free(p);

    return job;
}

int NEA_PaletteLoad(NEA_Palette *pal, const void *pointer, u16 numcolor,
                   NEA_TextureFormat format)
{
    if (!ne_palette_system_inited)
        return 0;

    NEA_AssertPointer(pal, "NULL pointer");

    // If this palette is RAM-backed, keep it in main RAM instead of uploading it
    // to palette VRAM. NEA_PaletteVramLoad() re-invokes this with
    // pointer == pal->ram_data, which falls through to the real upload.
    if (pal->ram_backed && pointer != pal->ram_data)
        return ne_palette_stash(pal, pointer, numcolor, format);

    if (pal->index != NEA_NO_PALETTE)
    {
        NEA_DebugPrint("Palette already loaded");
        // Free only the VRAM, not the NEA_Palette object (calling
        // NEA_PaletteDelete() here would free(pal) and leave a dangling pointer).
        ne_palette_vram_free(pal);
    }

    int slot = NEA_NO_PALETTE;

    for (int i = 0; i < NEA_MAX_PALETTES; i++)
    {
        if (NEA_PalInfo[i].pointer == NULL)
        {
            slot = i;
            break;
        }
    }

    if (slot == NEA_NO_PALETTE)
    {
        NEA_DebugPrint("No free lots");
        return 0;
    }

    NEA_PalInfo[slot].pointer = NEA_Alloc(NEA_PalAllocList, numcolor << 1);
    // Aligned to 16 bytes (except 8 bytes for NEA_PAL4).
    if (NEA_PalInfo[slot].pointer == NULL)
    {
        NEA_DebugPrint("Not enough memory");
        return 0;
    }

    NEA_PalInfo[slot].format = format;

    pal->index = slot;
    NEA_PalInfo[slot].numcolors = numcolor;
    // Allow CPU writes to palette VRAM
    ne_pal_to_lcd();
    swiCopy(pointer, NEA_PalInfo[slot].pointer, (numcolor / 2) | COPY_MODE_WORD);
    ne_pal_to_tex();

    return 1;
}

int NEA_PaletteLoadSize(NEA_Palette *pal, const void *pointer, size_t size,
                       NEA_TextureFormat format)
{
    return NEA_PaletteLoad(pal, pointer, size >> 1, format);
}

void NEA_PaletteDelete(NEA_Palette *pal)
{
    if (!ne_palette_system_inited)
        return;

    NEA_AssertPointer(pal, "NULL pointer");

    // Abort any asynchronous load that would write into this palette, before
    // the memory it points to goes away.
    __NEA_AsyncCancelTarget(pal);

    // Free the palette VRAM if there is one assigned
    ne_palette_vram_free(pal);

    // Free the RAM copy, if this is a RAM-backed palette
    free(pal->ram_data);

    for (int i = 0; i < NEA_MAX_PALETTES; i++)
    {
        if (NEA_UserPalette[i] == pal)
        {
            NEA_UserPalette[i] = NULL;
            free(pal);
            return;
        }
    }

    NEA_DebugPrint("Material not found");
}

void NEA_PaletteRamInit(NEA_Palette *palette)
{
    NEA_AssertPointer(palette, "NULL palette pointer");
    NEA_Assert(palette->index == NEA_NO_PALETTE,
              "RAM-backing must be enabled before loading a palette");

    palette->ram_backed = true;
}

int NEA_PaletteVramLoad(NEA_Palette *palette)
{
    NEA_AssertPointer(palette, "NULL palette pointer");

    // Nothing has been stashed in RAM, so there is nothing to upload
    if (!palette->ram_backed || palette->ram_data == NULL)
        return 0;

    // Already resident in VRAM
    if (palette->index != NEA_NO_PALETTE)
        return 1;

    // Upload the RAM copy. The ram_backed guard in NEA_PaletteLoad() lets this
    // call through because the buffer passed is the palette's own ram_data.
    return NEA_PaletteLoad(palette, palette->ram_data, palette->ram_numcolors,
                          palette->ram_format);
}

int NEA_PaletteVramUnload(NEA_Palette *palette)
{
    NEA_AssertPointer(palette, "NULL palette pointer");

    // Only RAM-backed palettes can be safely removed from VRAM: the RAM copy is
    // what allows uploading the palette again later.
    if (!palette->ram_backed || palette->ram_data == NULL)
        return 0;

    // Not currently in VRAM
    if (palette->index == NEA_NO_PALETTE)
        return 1;

    ne_palette_vram_free(palette);

    return 1;
}

void NEA_PaletteUse(const NEA_Palette *pal)
{
    NEA_AssertPointer(pal, "NULL pointer");

    // A palette with no data resident in VRAM (e.g. a RAM-backed palette that
    // has been removed from VRAM with NEA_PaletteVramUnload()) leaves the
    // palette register unchanged instead of reading out of bounds.
    if (pal->index == NEA_NO_PALETTE)
        return;

    unsigned int shift = 4 - (NEA_PalInfo[pal->index].format == NEA_PAL4);
    GFX_PAL_FORMAT = ((uintptr_t)NEA_PalInfo[pal->index].pointer
                      - ne_pal_lcd_base) >> shift;
}

int NEA_PaletteSystemReset(int max_palettes)
{
    if (ne_palette_system_inited)
        NEA_PaletteSystemEnd();

    if (max_palettes < 1)
        NEA_MAX_PALETTES = NEA_DEFAULT_PALETTES;
    else
        NEA_MAX_PALETTES = max_palettes;

    NEA_PalInfo = calloc(NEA_MAX_PALETTES, sizeof(ne_palinfo_t));
    NEA_UserPalette = calloc(NEA_MAX_PALETTES, sizeof(NEA_UserPalette));
    if ((NEA_PalInfo == NULL) || (NEA_UserPalette == NULL))
        goto cleanup;

    // Snapshot which VRAM banks are configured for texture palettes
    ne_pal_banks = NEA_GetTexPaletteBank();

    void *pal_start = NULL, *pal_end = NULL;
    if (ne_pal_get_range(&pal_start, &pal_end) == 0)
    {
        ne_pal_lcd_base = (uintptr_t)pal_start;

        if (NEA_AllocInit(&NEA_PalAllocList, pal_start, pal_end) != 0)
            goto cleanup;
    }
    else
    {
        // No palette VRAM configured — allocator stays NULL,
        // palette loads will fail gracefully.
        ne_pal_lcd_base = 0;
        NEA_PalAllocList = NULL;
    }

    GFX_PAL_FORMAT = 0;

    ne_palette_system_inited = true;
    return 0;

cleanup:
    NEA_DebugPrint("Not enough memory");
    free(NEA_PalInfo);
    free(NEA_UserPalette);
    return -1;
}

int NEA_PaletteFreeMem(void)
{
    if (!ne_palette_system_inited)
        return 0;

    NEAMemInfo info;
    NEA_MemGetInformation(NEA_PalAllocList, &info);

    return info.free;
}

int NEA_PaletteFreeMemPercent(void)
{
    if (!ne_palette_system_inited)
        return 0;

    NEAMemInfo info;
    NEA_MemGetInformation(NEA_PalAllocList, &info);

    return info.free_percent;
}

void NEA_PaletteDefragMem(void)
{
    NEA_Assert(0, "This function doesn't work");
    return;

    // TODO: Fix

    /*
    if (!ne_palette_system_inited)
        return;

    vramSetBankE(VRAM_E_LCD);
    bool ok = false;
    while (!ok)
    {
        ok = true;
        for (int i = 0; i < NEA_MAX_PALETTES; i++)
        {
            int size = NEA_GetSize(NEA_PalAllocList, (void*)NEA_PalInfo[i].pointer);

            NEA_Free(NEA_PalAllocList, (void*)NEA_PalInfo[i].pointer);
            void *pointer = NEA_Alloc(NEA_PalAllocList, size);
            // Aligned to 16 bytes (except 8 bytes for NEA_PAL4).

            NEA_AssertPointer(pointer, "Couldn't reallocate palette");

            if ((int)pointer != (int)NEA_PalInfo[i].pointer)
            {
                dmaCopy((void *)NEA_PalInfo[i].pointer, pointer, size);

                NEA_PalInfo[i].pointer = (void*)pointer;
                ok = false;
            }
        }
    }
    vramSetBankE(VRAM_E_TEX_PALETTE);
    */
}

void NEA_PaletteSystemEnd(void)
{
    if (!ne_palette_system_inited)
        return;

    NEA_AllocEnd(&NEA_PalAllocList);

    free(NEA_PalInfo);

    for (int i = 0; i < NEA_MAX_PALETTES; i++)
    {
        if (NEA_UserPalette[i])
        {
            __NEA_AsyncCancelTarget(NEA_UserPalette[i]);
            free(NEA_UserPalette[i]);
        }
    }

    free(NEA_UserPalette);

    ne_palette_system_inited = false;
}

static u16 *palette_adress = NULL;
static int palette_format;
static u16 palette_num_colors;

void *NEA_PaletteModificationStart(const NEA_Palette *pal)
{
    NEA_AssertPointer(pal, "NULL pointer");
    NEA_Assert(pal->index != NEA_NO_PALETTE, "No asigned palette");
    NEA_Assert(palette_adress == NULL, "Another palette already active");

    palette_adress = NEA_PalInfo[pal->index].pointer;
    palette_format = NEA_PalInfo[pal->index].format;
    palette_num_colors = NEA_PalInfo[pal->index].numcolors;
    // Enable CPU accesses to palette VRAM
    ne_pal_to_lcd();

    return palette_adress;
}

u16 NEA_PaletteModificationGetNumColors()
{
    return palette_num_colors;
}

void NEA_PaletteRGB256SetColor(u8 colorindex, u16 color)
{
    NEA_AssertPointer(palette_adress, "No active palette");
    NEA_Assert(palette_format == NEA_PAL256, "Active palette isn't NEA_PAL256");

    palette_adress[colorindex] = color;
}

void NEA_PaletteModificationEnd(void)
{
    NEA_Assert(palette_adress != NULL, "No active palette");

    // Disable CPU accesses to palette VRAM
    ne_pal_to_tex();

    palette_adress = NULL;
}
