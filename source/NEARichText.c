// SPDX-License-Identifier: MIT
//
// Copyright (c) 2024 Antonio Niño Díaz
//
// This file is part of Nitro Engine Advanced

#include "NEAMain.h"

#include "libdsf/dsf.h"

/// @file NEARichText.c

typedef struct {
    // Fields used when the font texture is stored in VRAM
    NEA_Material *material;
    NEA_Palette *palette;

    // Fields used when the font texture is stored in RAM
    bool has_to_free_buffers;
    NEA_TextureFormat fmt;
    void *texture_buffer;
    size_t texture_width;
    size_t texture_height;
    void *palette_buffer;
    size_t palette_size;

    // Other fields
    dsf_handle handle;
    bool active;
} ne_rich_textinfo_t;

static u32 NEA_NumRichTextSlots = 0;

static ne_rich_textinfo_t *NEA_RichTextInfo;

static int NEA_RICH_TEXT_PRIORITY = 0;

void NEA_RichTextPrioritySet(int priority)
{
    NEA_RICH_TEXT_PRIORITY = priority;
}

void NEA_RichTextPriorityReset(void)
{
    NEA_RICH_TEXT_PRIORITY = 0;
}

void NEA_RichTextInit(u32 slot)
{
    // Compatibility mode -- if someone tries to allocate a slot
    // without having called start system, we allocate the old maximum
    // number of slots for safety
    if (NEA_NumRichTextSlots == 0)
        NEA_RichTextStartSystem(NEA_DEFAULT_RICH_TEXT_FONTS);

    if (slot >= NEA_NumRichTextSlots)
    {
        NEA_DebugPrint("Attempted to initialize a slot greater than the number of slots allocated; skipping");
        return;
    }

    ne_rich_textinfo_t *info = &NEA_RichTextInfo[slot];
    if (info->active)
        NEA_RichTextEnd(slot);

    memset(info, 0, sizeof(ne_rich_textinfo_t));
    info->active = true;

    NEA_RichTextPriorityReset();
}

int NEA_RichTextEnd(u32 slot)
{
    if (slot >= NEA_NumRichTextSlots)
        return 0;

    ne_rich_textinfo_t *info = &NEA_RichTextInfo[slot];
    if (!info->active)
        return 0;

    if (info->material != NULL)
        NEA_MaterialDelete(info->material);
    if (info->palette != NULL)
        NEA_PaletteDelete(info->palette);

    if (info->has_to_free_buffers)
    {
        if (info->texture_buffer != NULL)
            free(info->texture_buffer);
        if (info->palette_buffer != NULL)
            free(info->palette_buffer);
    }

    int ret = 0;

    if (info->handle)
    {
        dsf_error err = DSF_FreeFont(&(info->handle));
        if (err != DSF_NO_ERROR)
            ret = err;
    }

    memset(info, 0, sizeof(ne_rich_textinfo_t));

    if (ret != 0)
        return 0;

    return 1;
}

int NEA_RichTextStartSystem(u32 numSlots)
{
    NEA_NumRichTextSlots = numSlots;
    NEA_RichTextInfo = calloc(sizeof(ne_rich_textinfo_t), NEA_NumRichTextSlots);
    if (NEA_RichTextInfo == NULL)
    {
        NEA_DebugPrint("Failed to allocate array for NEA_RichTextInfo");
        return 0;
    }
    return 1;
}

void NEA_RichTextResetSystem(void)
{
    for (int i = 0; i < NEA_NumRichTextSlots; i++)
        NEA_RichTextEnd(i);
    free(NEA_RichTextInfo);
    NEA_NumRichTextSlots = 0;
}

int NEA_RichTextMetadataLoadFAT(u32 slot, const char *path)
{
    NEA_AssertPointer(path, "NULL path pointer");

    if (slot >= NEA_NumRichTextSlots)
        return 0;

    ne_rich_textinfo_t *info = &NEA_RichTextInfo[slot];
    if (!info->active)
        return 0;

    dsf_handle handle;
    dsf_error ret = DSF_LoadFontFilesystem(&handle, path);
    if (ret != DSF_NO_ERROR)
    {
        NEA_DebugPrint("DSF_LoadFontFilesystem(): %d\n", ret);
        return 0;
    }

    info->handle = handle;

    return 1;
}

int NEA_RichTextMetadataLoadMemory(u32 slot, const void *data, size_t data_size)
{
    NEA_AssertPointer(data, "NULL data pointer");

    if (slot >= NEA_NumRichTextSlots)
        return 0;

    ne_rich_textinfo_t *info = &NEA_RichTextInfo[slot];
    if (!info->active)
        return 0;

    dsf_handle handle;
    dsf_error ret = DSF_LoadFontMemory(&handle, data, data_size);
    if (ret != DSF_NO_ERROR)
    {
        NEA_DebugPrint("DSF_LoadFontMemory(): %d\n", ret);
        return 0;
    }

    info->handle = handle;

    return 1;
}

int NEA_RichTextMaterialLoadGRF(u32 slot, const char *path)
{
    NEA_AssertPointer(path, "NULL path pointer");

    if (slot >= NEA_NumRichTextSlots)
        return 0;

    ne_rich_textinfo_t *info = &NEA_RichTextInfo[slot];
    if (!info->active)
        return 0;

    info->material = NEA_MaterialCreate();
    info->palette = NEA_PaletteCreate();

    int ret = NEA_MaterialTexLoadGRF(info->material, info->palette,
                    NEA_TEXGEN_TEXCOORD | NEA_TEXTURE_COLOR0_TRANSPARENT, path);
    if (ret == 0)
    {
        NEA_MaterialDelete(info->material);
        NEA_PaletteDelete(info->palette);
        return 0;
    }

    return 1;
}

int NEA_RichTextMaterialSet(u32 slot, NEA_Material *mat, NEA_Palette *pal)
{
    NEA_AssertPointer(mat, "NULL material pointer");

    if (slot >= NEA_NumRichTextSlots)
        return 0;

    ne_rich_textinfo_t *info = &NEA_RichTextInfo[slot];
    if (!info->active)
        return 0;

    info->material = mat;
    info->palette = pal;

    return 1;
}

int NEA_RichTextBitmapLoadGRF(u32 slot, const char *path)
{
#ifndef NEA_BLOCKSDS
    (void)slot;
    (void)path;
    NEA_DebugPrint("%s only supported in BlocksDS", __func__);
    return 0;
#else // NEA_BLOCKSDS
    NEA_AssertPointer(path, "NULL path pointer");

    if (slot >= NEA_NumRichTextSlots)
        return 0;

    ne_rich_textinfo_t *info = &NEA_RichTextInfo[slot];
    if (!info->active)
        return 0;

    if (info->has_to_free_buffers)
    {
        if (info->texture_buffer != NULL)
            free(info->texture_buffer);
        if (info->palette_buffer != NULL)
            free(info->palette_buffer);
    }

    void *gfxDst = NULL;
    void *palDst = NULL;
    size_t palSize;
    GRFHeader header = { 0 };
    GRFError err = grfLoadPath(path, &header, &gfxDst, NULL, NULL, NULL,
                               &palDst, &palSize);
    if (err != GRF_NO_ERROR)
    {
        NEA_DebugPrint("Couldn't load GRF file: %d", err);
        goto error;
    }

    if (gfxDst == NULL)
    {
        NEA_DebugPrint("No graphics found in GRF file");
        goto error;
    }

    bool palette_required = true;
    switch (header.gfxAttr)
    {
        case GRF_TEXFMT_A5I3:
            info->fmt = NEA_A5PAL8;
            break;
        case GRF_TEXFMT_A3I5:
            info->fmt = NEA_A3PAL32;
            break;
        case GRF_TEXFMT_4x4:
            info->fmt = NEA_TEX4X4;
            break;
        case 16:
            info->fmt = NEA_A1RGB5;
            palette_required = false;
            break;
        case 8:
            info->fmt = NEA_PAL256;
            break;
        case 4:
            info->fmt = NEA_PAL16;
            break;
        case 2:
            info->fmt = NEA_PAL4;
            break;
        default:
            NEA_DebugPrint("Invalid format in GRF file");
            goto error;
    }

    info->texture_buffer = gfxDst;
    info->texture_width = header.gfxWidth;
    info->texture_height = header.gfxHeight;

    if (palDst != NULL)
    {
        info->palette_buffer = palDst;
        info->palette_size = palSize;
    }
    else
    {
        if (palette_required)
        {
            NEA_DebugPrint("No palette found in GRF, but format requires it");
            goto error;
        }

        info->palette_buffer = NULL;
        info->palette_size = 0;
    }

    info->has_to_free_buffers = true;

    return 1; // Success

error:
    free(gfxDst);
    free(palDst);
    return 0;
#endif // NEA_BLOCKSDS
}

int NEA_RichTextBitmapSet(u32 slot, const void *texture_buffer,
                         size_t texture_width, size_t texture_height,
                         NEA_TextureFormat texture_fmt,
                         const void *palette_buffer, size_t palette_size)
{
    NEA_AssertPointer(texture_buffer, "NULL texture pointer");
    NEA_AssertPointer(palette_buffer, "NULL palette pointer");

    if (slot >= NEA_NumRichTextSlots)
        return 0;

    ne_rich_textinfo_t *info = &NEA_RichTextInfo[slot];
    if (!info->active)
        return 0;

    if (info->has_to_free_buffers)
    {
        if (info->texture_buffer != NULL)
            free(info->texture_buffer);
        if (info->palette_buffer != NULL)
            free(info->palette_buffer);
    }

    info->has_to_free_buffers = false;

    info->texture_buffer = (void *)texture_buffer;
    info->texture_width = texture_width;
    info->texture_height = texture_height;
    info->fmt = texture_fmt;
    info->palette_buffer = (void *)palette_buffer;
    info->palette_size = palette_size;

    return 1;
}

int NEA_RichTextRenderDryRunWithPos(u32 slot, const char *str,
                            size_t *size_x, size_t *size_y,
                            size_t *final_x, size_t *final_y)
{
    NEA_AssertPointer(str, "NULL str pointer");
    NEA_AssertPointer(size_x, "NULL size X pointer");
    NEA_AssertPointer(size_y, "NULL size Y pointer");
    NEA_AssertPointer(final_x, "NULL final X pointer");
    NEA_AssertPointer(final_y, "NULL final Y pointer");

    if (slot >= NEA_NumRichTextSlots)
        return 0;

    ne_rich_textinfo_t *info = &NEA_RichTextInfo[slot];
    if (!info->active)
        return 0;

    dsf_error err = DSF_StringRenderDryRunWithCursor(info->handle, str,
                                                     size_x, size_y,
                                                     final_x, final_y);
    if (err != DSF_NO_ERROR)
        return 0;

    return 1;
}

int NEA_RichTextRenderDryRun(u32 slot, const char *str,
                            size_t *size_x, size_t *size_y)
{
    size_t final_x, final_y;
    return NEA_RichTextRenderDryRunWithPos(slot, str, size_x, size_y, &final_x, &final_y);
}

int NEA_RichTextRender3DWithIndent(u32 slot, const char *str, s32 x, s32 y,
                                  s32 xIndent)
{
    NEA_AssertPointer(str, "NULL str pointer");

    if (slot >= NEA_NumRichTextSlots)
        return 0;

    ne_rich_textinfo_t *info = &NEA_RichTextInfo[slot];
    if (!info->active)
        return 0;

    NEA_MaterialUse(info->material);

    dsf_error err = DSF_StringRender3DWithIndent(info->handle, str, x, y,
                                               NEA_RICH_TEXT_PRIORITY, xIndent);
    if (err != DSF_NO_ERROR)
        return 0;

    return 1;
}

int NEA_RichTextRender3D(u32 slot, const char *str, s32 x, s32 y)
{
    return NEA_RichTextRender3DWithIndent(slot, str, x, y, 0);
}

int NEA_RichTextRender3DAlphaWithIndent(u32 slot, const char *str, s32 x, s32 y,
                                       uint32_t poly_fmt, int poly_id_base,
                                       s32 xIndent)
{
    NEA_AssertPointer(str, "NULL str pointer");

    if (slot >= NEA_NumRichTextSlots)
        return 0;

    ne_rich_textinfo_t *info = &NEA_RichTextInfo[slot];
    if (!info->active)
        return 0;

    NEA_MaterialUse(info->material);

    dsf_error err = DSF_StringRender3DAlphaWithIndent(info->handle, str, x, y,
                                                      NEA_RICH_TEXT_PRIORITY,
                                                      poly_fmt, poly_id_base, xIndent);
    if (err != DSF_NO_ERROR)
        return 0;

    return 1;
}

int NEA_RichTextRender3DAlpha(u32 slot, const char *str, s32 x, s32 y,
                             uint32_t poly_fmt, int poly_id_base)
{
    return NEA_RichTextRender3DAlphaWithIndent(slot, str, x, y, poly_fmt, poly_id_base, 0);
}

int NEA_RichTextRenderMaterial(u32 slot, const char *str, NEA_Material **mat,
                              NEA_Palette **pal)
{
    NEA_AssertPointer(str, "NULL str pointer");
    NEA_AssertPointer(mat, "NULL mat pointer");
    NEA_AssertPointer(pal, "NULL pal pointer");

    if (slot >= NEA_NumRichTextSlots)
        return 0;

    ne_rich_textinfo_t *info = &NEA_RichTextInfo[slot];
    if (!info->active)
        return 0;

    void *out_texture = NULL;
    size_t out_width, out_height;
    dsf_error err = DSF_StringRenderToTexture(info->handle,
                            str, info->fmt, info->texture_buffer,
                            info->texture_width, info->texture_height,
                            &out_texture, &out_width, &out_height);
    if (err != DSF_NO_ERROR)
    {
        free(out_texture);
        return 0;
    }

    *mat = NEA_MaterialCreate();
    if (NEA_MaterialTexLoad(*mat, info->fmt, out_width, out_height,
                           NEA_TEXGEN_TEXCOORD | NEA_TEXTURE_COLOR0_TRANSPARENT,
                           out_texture) == 0)
    {
        free(out_texture);
        return 0;
    }

    if (info->palette_buffer != NULL)
    {
        NEA_Palette *palette = NEA_PaletteCreate();
        if (NEA_PaletteLoad(palette, info->palette_buffer,
                           info->palette_size / 2, info->fmt) == 0)
        {
            NEA_MaterialDelete(*mat);
            free(out_texture);
            return 0;
        }

        NEA_MaterialSetPalette(*mat, palette);

        // If the caller has requested the pointer to the palette, return the
        // value to the user. If not, mark it to be autodeleted.
        if (pal)
            *pal = palette;
        else
            NEA_MaterialAutodeletePalette(*mat);
    }

    // This isn't needed after it has been loaded to VRAM
    free(out_texture);

    return 1;
}
