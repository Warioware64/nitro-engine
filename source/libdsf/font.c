// SPDX-License-Identifier: Zlib OR MIT
//
// Copyright (c) 2024-2026 Antonio Niño Díaz

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <dsf.h>

#include "bmf.h"
#include "font.h"

static int DSF_block_char_cmp(const void *a, const void *b)
{
    const block_char *a_ = a;
    const block_char *b_ = b;
    return a_->id - b_->id;
}

static int DSF_kerning_pair_cmp(const void *a, const void *b)
{
    const kerning_pair *a_ = a;
    const kerning_pair *b_ = b;

    // Compare the first codepoint. If it matches, compare the second codepoint.
    if (a_->first != b_->first)
        return a_->first - b_->first;

    return a_->second - b_->second;
}

const block_char *DSF_FontGetGlyph(dsf_handle handle, uint32_t codepoint)
{
    dsf_font_internal_state *font = (dsf_font_internal_state *)handle;

    // Codepoint to find
    block_char key = { 0 };
    key.id = codepoint;

    const block_char *ch = bsearch(&key, font->chars, font->num_chars,
                                   sizeof(block_char), DSF_block_char_cmp);
    if (ch == NULL)
        return font->replacement_character;

    return ch;
}

const kerning_pair *DSF_FontGetKerningPair(dsf_handle handle, kerning_pair *pair)
{
    dsf_font_internal_state *font = (dsf_font_internal_state *)handle;

    const kerning_pair *ker = bsearch(pair, font->kernings, font->num_kernings,
                                      sizeof(kerning_pair), DSF_kerning_pair_cmp);

    return ker;
}

static dsf_error DSF_LoadFile(const char *path, void **data, size_t *_size)
{
    FILE *f = fopen(path, "rb");
    if (f == NULL)
        return DSF_FILE_OPEN_ERROR;

    if (fseek(f, 0, SEEK_END) != 0)
    {
        fclose(f);
        return DSF_FILE_SEEK_ERROR;
    }

    size_t size = ftell(f);
    if (size == 0)
    {
        fclose(f);
        return DSF_FILE_EMPTY;
    }

    rewind(f);

    char *buffer = malloc(size);
    if (buffer == NULL)
    {
        fclose(f);
        return DSF_NO_MEMORY;
    }

    if (fread(buffer, 1, size, f) != size)
    {
        fclose(f);
        return DSF_FILE_READ_ERROR;
    }

    fclose(f);

    *_size = size;
    *data = buffer;
    return DSF_NO_ERROR;
}

dsf_error DSF_LoadFontMemory(dsf_handle *handle,
                             const void *data, int32_t data_size)
{
    dsf_error ret = DSF_NO_ERROR;
    const uint8_t *ptr = data;

    // Basic checks

    if ((handle == NULL) || (data == NULL) || (data_size <= 0))
        return DSF_INVALID_ARGUMENT;

    // Read header

    const bmf_header *header = (const bmf_header *)ptr;

    if (!((header->magic[0] == 'B') && (header->magic[1] == 'M') &&
          (header->magic[2] == 'F')))
        return DSF_BAD_MAGIC;

    if (header->version != 3)
        return DSF_BAD_VERSION;

    ptr += sizeof(bmf_header);
    data_size -= sizeof(bmf_header);

    // Allocate space for the handle

    dsf_font_internal_state *font = calloc(1, sizeof(dsf_font_internal_state));
    if (font == NULL)
        return DSF_NO_MEMORY;

    *handle = (dsf_handle)font; // Return handle to user

    // Read blocks

    while (1)
    {
        const bmf_block_header *block_header = (const bmf_block_header *)ptr;

        uint8_t type = block_header->type;
        uint32_t block_size = ((uint32_t)block_header->size[0]) |
                              ((uint32_t)block_header->size[1] << 8) |
                              ((uint32_t)block_header->size[2] << 16) |
                              ((uint32_t)block_header->size[3] << 24);

        const uint8_t *block_data = ptr + 1 + 4;

        if (type == 2)
        {
            // Ensure the size is correct
            if (block_size != sizeof(bmf_block_2_common))
            {
                ret = DSF_BAD_CHUNK_SIZE;
                goto error;
            }

            bmf_block_2_common block_common;
            memcpy(&block_common, block_data, block_size);

            font->line_height = block_common.line_height;
            font->base = block_common.base;
        }
        else if (type == 4)
        {
            // Ensure the total size is a multiple of the block size
            if (block_size % sizeof(bmf_block_4_char) != 0)
            {
                ret = DSF_BAD_CHUNK_SIZE;
                goto error;
            }

            font->num_chars = block_size / sizeof(bmf_block_4_char);
            font->chars = calloc(font->num_chars, sizeof(block_char));
            if (font->chars == NULL)
            {
                ret = DSF_NO_MEMORY;
                goto error;
            }

            memcpy(font->chars, block_data, block_size);

            qsort(font->chars, font->num_chars, sizeof(block_char),
                  DSF_block_char_cmp);
        }
        else if (type == 5)
        {
            // Ensure the total size is a multiple of the block size
            if (block_size % sizeof(bmf_block_5_kerning_pair) != 0)
            {
                ret = DSF_BAD_CHUNK_SIZE;
                goto error;
            }

            font->num_kernings = block_size / sizeof(bmf_block_5_kerning_pair);
            font->kernings = calloc(font->num_kernings, sizeof(kerning_pair));
            if (font->kernings == NULL)
            {
                ret = DSF_NO_MEMORY;
                goto error;
            }

            const uint8_t *src = block_data;
            uint8_t *dst = (uint8_t *)font->kernings;

            for (size_t i = 0; i < font->num_kernings; i++)
            {
                memcpy(dst, src, sizeof(bmf_block_5_kerning_pair));
                dst += sizeof(kerning_pair);
                src += sizeof(bmf_block_5_kerning_pair);
            }

            qsort(font->kernings, font->num_kernings, sizeof(kerning_pair),
                  DSF_kerning_pair_cmp);
        }

        ptr += block_size + 1 + 4;
        data_size -= block_size + 1 + 4;

        if (data_size == 0)
            break;

        if (data_size < 0)
        {
            ret = DSF_UNEXPECTED_END;
            goto error;
        }
    }

    if (font->num_chars == 0)
    {
        ret = DSF_NO_CHARACTERS;
        goto error;
    }

    // Look for a replacement character glyph in the font.

    // Initialize the value to NULL so that DSF_CodepointFindGlyph() returns
    // NULL if no replacement character glyph is found.
    font->replacement_character = NULL;
    font->replacement_character = DSF_FontGetGlyph(*handle, REPLACEMENT_CHARACTER);

    return DSF_NO_ERROR;

error:
    free(font->chars);
    free(font->kernings);
    free(font);
    return ret;
}

dsf_error DSF_FreeFont(dsf_handle *handle)
{
    if (handle == NULL)
        return DSF_INVALID_ARGUMENT;

    dsf_font_internal_state *font = (dsf_font_internal_state *)*handle;

    free(font->chars);
    free(font->kernings);
    free(font);

    *handle = 0;

    return DSF_NO_ERROR;
}

dsf_error DSF_LoadFontFilesystem(dsf_handle *handle, const char *path)
{
    if ((handle == NULL) || (path == NULL))
        return DSF_INVALID_ARGUMENT;

    size_t size;
    void *data;
    dsf_error error = DSF_LoadFile(path, &data, &size);
    if (error != DSF_NO_ERROR)
        return error;

    error = DSF_LoadFontMemory(handle, data, size);
    free(data);
    return error;
}
