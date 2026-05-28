// SPDX-License-Identifier: Zlib OR MIT
//
// Copyright (c) 2024-2026 Antonio Niño Díaz

#ifndef FONT_H__
#define FONT_H__

#include <stdint.h>

#include "bmf.h"

// Internal data structures
// ------------------------

typedef bmf_block_4_char block_char;

typedef struct {
    uint32_t first;
    uint32_t second;
    int16_t  amount;
} kerning_pair;

typedef struct {
    uint16_t      line_height;
    uint16_t      base;

    size_t        num_chars;
    block_char   *chars;
    size_t        num_kernings;
    kerning_pair *kernings;

    // If the font includes a replacement character glyph, its pointer will be
    // saved here for ease of access.
    const block_char *replacement_character;

    const void   *font_texture;
    size_t        font_width;
    size_t        font_height;
    dsf_format    font_format;

} dsf_font_internal_state;

const block_char *DSF_FontGetGlyph(dsf_handle handle, uint32_t codepoint);

const kerning_pair *DSF_FontGetKerningPair(dsf_handle handle, kerning_pair *pair);

#endif // FONT_H__
