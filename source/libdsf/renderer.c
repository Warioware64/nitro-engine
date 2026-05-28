// SPDX-License-Identifier: Zlib OR MIT
//
// Copyright (c) 2024-2026 Antonio Niño Díaz

#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef __NDS__
#include <nds.h>
#endif // __NDS__

#include <dsf.h>

#include "font.h"
#include "renderer.h"

dsf_error DSF_BufferAlloc(dsf_handle handle, void **buffer, size_t tex_width,
                    size_t tex_height)
{
    if ((handle == 0) || (buffer == NULL) || (tex_width == 0) || (tex_height == 0))
        return DSF_INVALID_ARGUMENT;

    dsf_font_internal_state *font = (dsf_font_internal_state *)handle;

    const uint8_t size_shift[] = {
        [DSF_NO_FORMAT]     = 0,
#ifdef __NDS__
        [DSF_BMP_RGB32_A3]  = 1,
#endif
        [DSF_BMP_RGB4]      = 3,
        [DSF_BMP_RGB16]     = 2,
        [DSF_BMP_RGB256]    = 1,
        [DSF_BMP_UNUSED]    = 0,
#ifdef __NDS__
        [DSF_BMP_RGB8_A5]   = 1,
#endif
        [DSF_BMP_RGBA]      = 0,
        [DSF_BMP_RGB]       = 0,
    };

    size_t tex_size = (2 * tex_width * tex_height) >> size_shift[font->font_format];

    *buffer = calloc(1, tex_size);
    if (*buffer == NULL)
        return DSF_NO_MEMORY;

    return DSF_NO_ERROR;
}

dsf_error DSF_CodepointRender(dsf_handle handle, dsf_renderer renderer,
                    uint32_t codepoint)
{
    if ((handle == 0) || (renderer == 0) || (codepoint == 0))
        return DSF_INVALID_ARGUMENT;

    dsf_font_internal_state *font = (dsf_font_internal_state *)handle;
    dsf_renderer_internal_state *state = (dsf_renderer_internal_state *)renderer;

    if (state->render_mode == DSF_RENDER_MODE_BUFFER)
    {
        if (font->font_format != state->out_format)
            return DSF_FORMAT_MISMATCH;
    }

    if (codepoint == '\n')
    {
        state->pointer_x = state->box_left;
        state->pointer_y += font->line_height;
        return DSF_NO_ERROR;
    }

    const block_char *ch = DSF_FontGetGlyph(handle, codepoint);
    if (ch == NULL)
        return DSF_CODEPOINT_NOT_FOUND;

    int tx1 = ch->x;
    int tx2 = tx1 + ch->width;
    int ty1 = ch->y;
    int ty2 = ty1 + ch->height;

    int x1 = state->pointer_x;
    int x2 = x1 + ch->width;
    int y1 = state->pointer_y;
    int y2 = y1 + ch->height;

    x1 += ch->xoffset;
    x2 += ch->xoffset;
    y1 += ch->yoffset;
    y2 += ch->yoffset;

    state->pointer_x += ch->xadvance;

    // Kerning pair to find
    kerning_pair key = {
        .first = state->last_codepoint,
        .second = codepoint
    };

    const kerning_pair *ker = DSF_FontGetKerningPair(handle, &key);
    if (ker != NULL)
    {
        x1 += ker->amount;
        x2 += ker->amount;
        state->pointer_x += ker->amount;
    }

    // Now that we have applied the kerning and offsets, check if the glyph fits
    // in the buffer. First, check the right edge of the canvas.

    if (state->render_right_mode == DSF_RENDER_RIGHT_WRAP)
    {
        // If we're going over the right limit, jump to the next line.

        if (x2 >= state->box_right)
        {
            state->pointer_x = state->box_left;
            state->pointer_y += font->line_height;

            x1 = state->pointer_x;
            x2 = x1 + ch->width;
            y1 = state->pointer_y;
            y2 = y1 + ch->height;

            x1 += ch->xoffset;
            x2 += ch->xoffset;
            y1 += ch->yoffset;
            y2 += ch->yoffset;

            state->pointer_x += ch->xadvance;
        }
    }
    else if (state->render_right_mode == DSF_RENDER_RIGHT_TRIM)
    {
        // If only part of the glyph fits, draw it. If no part of the glyph
        // fits, return error.

        if (x1 >= state->box_right)
            return DSF_CANVAS_FULL;

        if (x2 > state->box_right)
            x2 = state->box_right;
    }
    else if (state->render_right_mode == DSF_RENDER_RIGHT_ERROR)
    {
        // If the glyph doesn't completely fit, return an error message.
        if (x2 > state->box_right)
            return DSF_CANVAS_FULL;
    }

    // After checking the right end of the canvas, check the bottom end.

    if (state->render_bottom_mode == DSF_RENDER_BOTTOM_TRIM)
    {
        // If the current horizontal line is completely outside of the canvas, exit
        // right away.

        if (state->pointer_y >= state->box_bottom)
            return DSF_CANVAS_FULL;

        // If it partially fits verically, print the parts that fit.

        if (y2 > state->box_bottom)
            y2 = state->box_bottom;
    }
    else if (state->render_bottom_mode == DSF_RENDER_BOTTOM_ERROR)
    {
        // If the glyph doesn't completely fit, return an error message.
        if (y2 > state->box_bottom)
            return DSF_CANVAS_FULL;
    }

    // Update bounds of used space

    if (state->used_left > x1)
        state->used_left = x1;
    if (state->used_top > y1)
        state->used_top = y1;

    if (state->used_right < x2)
        state->used_right = x2;
    if (state->used_bottom < y2)
        state->used_bottom = y2;

    state->last_codepoint = codepoint;

    if (state->render_mode == DSF_RENDER_MODE_NONE)
    {
        // Do nothing
    }
    else if (state->render_mode == DSF_RENDER_MODE_BUFFER)
    {
        const void *font_texture = font->font_texture;
        size_t font_width = font->font_width;

        dsf_format texture_fmt = state->out_format;
        void *out_texture = state->out_texture;
        size_t out_width = state->out_width;

        if (texture_fmt == DSF_BMP_RGB256)
        {
            const uint16_t *src = font_texture;
            uint16_t *dst = out_texture;

            for (int y = 0; y < y2 - y1; y++)
            {
                const uint16_t *src_row = src + (((ty1 + y) * font_width) >> 1);
                uint16_t *dst_row = dst + (((y1 + y) * out_width) >> 1);

                for (int x = 0; x < x2 - x1; x++)
                {
                    const uint16_t *src_px = src_row + ((tx1 + x) >> 1);
                    uint16_t *dst_px = dst_row + ((x1 + x) >> 1);

                    int shift = ((tx1 + x) & 1) * 8;
                    uint16_t src_color = (*src_px >> shift) & 0xFF;

                    if (src_color == 0)
                        continue;

                    shift = ((x1 + x) & 1) * 8;
                    uint16_t mask = ~(0xFF << shift);
                    *dst_px = (*dst_px & mask) | (src_color << shift);
                }
            }
        }
        else if (texture_fmt == DSF_BMP_RGBA)
        {
            const uint16_t *src = font_texture;
            uint16_t *dst = out_texture;

            src += tx1 + ty1 * font_width;
            dst += x1 + y1 * out_width;

            for (int y = 0; y < y2 - y1; y++)
            {
                const uint16_t *src_row = src;
                uint16_t *dst_row = dst;

                for (int x = 0; x < x2 - x1; x++)
                {
                    uint16_t color = *src_row++;
                    if (color & (1 << 15))
                        *dst_row = color;
                    dst_row++;
                }

                src += font_width;
                dst += out_width;
            }
        }
        else if (texture_fmt == DSF_BMP_RGB)
        {
            const uint16_t *src = font_texture;
            uint16_t *dst = out_texture;

            src += tx1 + ty1 * font_width;
            dst += x1 + y1 * out_width;

            for (int y = 0; y < y2 - y1; y++)
            {
                const uint16_t *src_row = src;
                uint16_t *dst_row = dst;

                for (int x = 0; x < x2 - x1; x++)
                {
                    uint16_t color = *src_row++;
                    *dst_row++ = color | (1 << 15);
                }

                src += font_width;
                dst += out_width;
            }
        }
#ifdef __NDS__
        else if ((texture_fmt == DSF_BMP_RGB32_A3) || (texture_fmt == DSF_BMP_RGB8_A5))
        {
            const uint16_t *src = font_texture;
            uint16_t *dst = out_texture;

            for (int y = 0; y < y2 - y1; y++)
            {
                const uint16_t *src_row = src + (((ty1 + y) * font_width) >> 1);
                uint16_t *dst_row = dst + (((y1 + y) * out_width) >> 1);

                for (int x = 0; x < x2 - x1; x++)
                {
                    const uint16_t *src_px = src_row + ((tx1 + x) >> 1);
                    uint16_t *dst_px = dst_row + ((x1 + x) >> 1);

                    int shift = ((tx1 + x) & 1) * 8;
                    uint16_t src_color = (*src_px >> shift) & 0xFF;

                    // We can't really blend two different colors because we're
                    // limited by the palette. For that reason, we just directly
                    // copy to the new texture under the assumption that the user
                    // will be using a transparent texture as a base.
                    //if (src_color == 0)
                    //    continue;

                    shift = ((x1 + x) & 1) * 8;
                    uint16_t mask = ~(0xFF << shift);
                    *dst_px = (*dst_px & mask) | (src_color << shift);
                }
            }
        }
#endif // __NDS__
        else if (texture_fmt == DSF_BMP_RGB16)
        {
            const uint16_t *src = font_texture;
            uint16_t *dst = out_texture;

            for (int y = 0; y < y2 - y1; y++)
            {
                const uint16_t *src_row = src + (((ty1 + y) * font_width) >> 2);
                uint16_t *dst_row = dst + (((y1 + y) * out_width) >> 2);

                for (int x = 0; x < x2 - x1; x++)
                {
                    const uint16_t *src_px = src_row + ((tx1 + x) >> 2);
                    uint16_t *dst_px = dst_row + ((x1 + x) >> 2);

                    int shift = ((tx1 + x) & 3) * 4;
                    uint16_t src_color = (*src_px >> shift) & 0xF;

                    if (src_color == 0)
                        continue;

                    shift = ((x1 + x) & 3) * 4;
                    uint16_t mask = ~(0xF << shift);
                    *dst_px = (*dst_px & mask) | (src_color << shift);
                }
            }
        }
        else if (texture_fmt == DSF_BMP_RGB4)
        {
            const uint16_t *src = font_texture;
            uint16_t *dst = out_texture;

            for (int y = 0; y < y2 - y1; y++)
            {
                const uint16_t *src_row = src + (((ty1 + y) * font_width) >> 3);
                uint16_t *dst_row = dst + (((y1 + y) * out_width) >> 3);

                for (int x = 0; x < x2 - x1; x++)
                {
                    const uint16_t *src_px = src_row + ((tx1 + x) >> 3);
                    uint16_t *dst_px = dst_row + ((x1 + x) >> 3);

                    int shift = ((tx1 + x) & 7) * 2;
                    uint16_t src_color = (*src_px >> shift) & 0x3;

                    if (src_color == 0)
                        continue;

                    shift = ((x1 + x) & 7) * 2;
                    uint16_t mask = ~(0x3 << shift);
                    *dst_px = (*dst_px & mask) | (src_color << shift);
                }
            }
        }
    }
#ifdef __NDS__
    else if (state->render_mode == DSF_RENDER_MODE_NDS_3D)
    {
        GFX_TEX_COORD = TEXTURE_PACK(inttot16(tx1), inttot16(ty1));
        GFX_VERTEX16 = (y1 << 16) | (x1 & 0xFFFF); // Up-left
        GFX_VERTEX16 = state->z;

        GFX_TEX_COORD = TEXTURE_PACK(inttot16(tx1), inttot16(ty2));
        GFX_VERTEX_XY = (y2 << 16) | (x1 & 0xFFFF); // Down-left

        GFX_TEX_COORD = TEXTURE_PACK(inttot16(tx2), inttot16(ty2));
        GFX_VERTEX_XY = (y2 << 16) | (x2 & 0xFFFF); // Down-right

        GFX_TEX_COORD = TEXTURE_PACK(inttot16(tx2), inttot16(ty1));
        GFX_VERTEX_XY = (y1 << 16) | (x2 & 0xFFFF); // Up-right
    }
    else if (state->render_mode == DSF_RENDER_MODE_NDS_3D_BLEND)
    {
        glPolyFmt(state->poly_fmt | POLY_ID(state->poly_id_base + state->id_index));
        state->id_index ^= 1;

        glBegin(GL_QUADS);

        GFX_TEX_COORD = TEXTURE_PACK(inttot16(tx1), inttot16(ty1));
        GFX_VERTEX16 = (y1 << 16) | (x1 & 0xFFFF); // Up-left
        GFX_VERTEX16 = state->z;

        GFX_TEX_COORD = TEXTURE_PACK(inttot16(tx1), inttot16(ty2));
        GFX_VERTEX_XY = (y2 << 16) | (x1 & 0xFFFF); // Down-left

        GFX_TEX_COORD = TEXTURE_PACK(inttot16(tx2), inttot16(ty2));
        GFX_VERTEX_XY = (y2 << 16) | (x2 & 0xFFFF); // Down-right

        GFX_TEX_COORD = TEXTURE_PACK(inttot16(tx2), inttot16(ty1));
        GFX_VERTEX_XY = (y1 << 16) | (x2 & 0xFFFF); // Up-right

        glEnd();
    }
#else // __NDS__
    (void)tx2;
    (void)ty2;
#endif // __NDS__

    return DSF_NO_ERROR;
}

dsf_error DSF_FontTextureSet(dsf_handle handle, const void *font_texture,
                    size_t font_width, size_t font_height, dsf_format font_format)
{
    if ((handle == 0) || (font_texture == NULL) ||
        (font_width == 0) || (font_height == 0))
        return DSF_INVALID_ARGUMENT;

    if ((font_format >= DSF_FORMAT_END) || (font_format == DSF_NO_FORMAT) ||
        (font_format == DSF_BMP_UNUSED))
        return DSF_TEXTURE_BAD_FORMAT;

    dsf_font_internal_state *font = (dsf_font_internal_state *)handle;

    font->font_texture = font_texture;
    font->font_width = font_width;
    font->font_height = font_height;
    font->font_format = font_format;

    return DSF_NO_ERROR;
}

dsf_error DSF_RendererUsedBoxReset(dsf_renderer renderer)
{
    if (renderer == 0)
        return DSF_INVALID_ARGUMENT;

    dsf_renderer_internal_state *state = (dsf_renderer_internal_state *)renderer;

    // Set initial values that will always be changed as soon as a character is
    // printed, no matter the position.
    state->used_left = INT16_MAX;
    state->used_top = INT16_MAX;
    state->used_right = 0;
    state->used_bottom = 0;

    return DSF_NO_ERROR;
}

dsf_error DSF_RendererUsedBoxGet(dsf_renderer renderer, int16_t *left,
                    int16_t *top, int16_t *right, int16_t *bottom)
{
    if (renderer == 0)
        return DSF_INVALID_ARGUMENT;

    dsf_renderer_internal_state *state = (dsf_renderer_internal_state *)renderer;

    // Detect if the canvas hasn't been used yet
    if ((state->used_left > state->used_right) ||
        (state->used_top > state->used_bottom))
        return DSF_CANVAS_EMPTY;

    if (left)
        *left = state->used_left;
    if (top)
        *top = state->used_top;
    if (right)
        *right = state->used_right;
    if (bottom)
        *bottom = state->used_bottom;

    return DSF_NO_ERROR;
}

dsf_error DSF_RendererUsedBoxGetTextureSize(dsf_renderer renderer,
                    size_t *width, size_t *height)
{
    if ((renderer == 0) || (width == NULL) || (height == NULL))
        return DSF_INVALID_ARGUMENT;

    dsf_renderer_internal_state *state = (dsf_renderer_internal_state *)renderer;

    // Detect if the canvas hasn't been used yet
    if ((state->used_left > state->used_right) ||
        (state->used_top > state->used_bottom))
        return DSF_CANVAS_EMPTY;

    size_t tex_width = state->used_right;
    size_t tex_height = state->used_bottom;

    if ((tex_width > 1024) || (tex_height > 1024))
        return DSF_TEXTURE_TOO_BIG;

    // Expand to a valid texture size
    // We only expand the width as leaving the height clipped saves VRAM

    for (size_t i = 8; i <= 1024; i <<= 1)
    {
        if (tex_width <= i)
        {
            tex_width = i;
            break;
        }
    }

    *width = tex_width;
    *height = tex_height;

    return DSF_NO_ERROR;
}

dsf_error DSF_RendererCanvasSetup(dsf_renderer renderer, int16_t box_left,
                    int16_t box_top, int16_t box_right, int16_t box_bottom)
{
    if (renderer == 0)
        return DSF_INVALID_ARGUMENT;

    if ((box_right <= box_left) || (box_bottom <= box_top))
        return DSF_INVALID_ARGUMENT;

    dsf_renderer_internal_state *state = (dsf_renderer_internal_state *)renderer;

    state->box_left = box_left;
    state->box_top = box_top;
    state->box_right = box_right;
    state->box_bottom = box_bottom;

    state->pointer_x = box_left;
    state->pointer_y = box_top;
    state->last_codepoint = 0;

    return DSF_RendererUsedBoxReset(renderer);
}

dsf_error DSF_RendererCanvasClear(dsf_renderer renderer)
{
    if (renderer == 0)
        return DSF_INVALID_ARGUMENT;

    dsf_renderer_internal_state *state = (dsf_renderer_internal_state *)renderer;

    if (state->render_mode != DSF_RENDER_MODE_BUFFER)
        return DSF_INVALID_ARGUMENT;

    int x1 = state->box_left;
    int y1 = state->box_top;
    int x2 = state->box_right;
    int y2 = state->box_bottom;

    state->pointer_x = state->box_left;
    state->pointer_y = state->box_top;
    state->last_codepoint = 0;

    dsf_format texture_fmt = state->out_format;
    void *out_texture = state->out_texture;
    size_t out_width = state->out_width;

    if ((texture_fmt == DSF_BMP_RGB256)
#ifdef __NDS__
        || (texture_fmt == DSF_BMP_RGB32_A3) || (texture_fmt == DSF_BMP_RGB8_A5)
#endif // __NDS__
       )
    {
        uint16_t *dst = out_texture;

        for (int y = 0; y < y2 - y1; y++)
        {
            uint16_t *dst_row = dst + (((y1 + y) * out_width) >> 1);

            for (int x = 0; x < x2 - x1; x++)
            {
                uint16_t *dst_px = dst_row + ((x1 + x) >> 1);

                int shift = ((x1 + x) & 1) * 8;
                uint16_t mask = ~(0xFF << shift);
                *dst_px = *dst_px & mask;
            }
        }
    }
    else if ((texture_fmt == DSF_BMP_RGBA) || (texture_fmt == DSF_BMP_RGB))
    {
        uint16_t *dst = out_texture;

        dst += x1 + y1 * out_width;

        for (int y = 0; y < y2 - y1; y++)
        {
            uint16_t *dst_row = dst;

            for (int x = 0; x < x2 - x1; x++)
                *dst_row++ = 0;

            dst += out_width;
        }
    }
    else if (texture_fmt == DSF_BMP_RGB16)
    {
        uint16_t *dst = out_texture;

        for (int y = 0; y < y2 - y1; y++)
        {
            uint16_t *dst_row = dst + (((y1 + y) * out_width) >> 2);

            for (int x = 0; x < x2 - x1; x++)
            {
                uint16_t *dst_px = dst_row + ((x1 + x) >> 2);

                int shift = ((x1 + x) & 3) * 4;
                uint16_t mask = ~(0xF << shift);
                *dst_px = *dst_px & mask;
            }
        }
    }
    else if (texture_fmt == DSF_BMP_RGB4)
    {
        uint16_t *dst = out_texture;

        for (int y = 0; y < y2 - y1; y++)
        {
            uint16_t *dst_row = dst + (((y1 + y) * out_width) >> 3);

            for (int x = 0; x < x2 - x1; x++)
            {
                uint16_t *dst_px = dst_row + ((x1 + x) >> 3);

                int shift = ((x1 + x) & 7) * 2;
                uint16_t mask = ~(0x3 << shift);
                *dst_px = *dst_px & mask;
            }
        }
    }

    return DSF_RendererUsedBoxReset(renderer);
}

dsf_error DSF_RendererOverflowModeSet(dsf_renderer renderer,
                dsf_render_right_mode right_mode, dsf_render_bottom_mode bottom_mode)
{
    if (renderer == 0)
        return DSF_INVALID_ARGUMENT;

    if ((right_mode != DSF_RENDER_RIGHT_WRAP) &&
        (right_mode != DSF_RENDER_RIGHT_TRIM) &&
        (right_mode != DSF_RENDER_RIGHT_ERROR))
        return DSF_INVALID_ARGUMENT;

    if ((bottom_mode != DSF_RENDER_BOTTOM_TRIM) &&
        (bottom_mode != DSF_RENDER_BOTTOM_ERROR))
        return DSF_INVALID_ARGUMENT;

    dsf_renderer_internal_state *state = (dsf_renderer_internal_state *)renderer;

    state->render_right_mode = right_mode;
    state->render_bottom_mode = bottom_mode;

    return DSF_NO_ERROR;
}

dsf_error DSF_RendererWordWrapModeSet(dsf_renderer renderer, bool enabled,
                const uint32_t separators[])
{
    if (renderer == 0)
        return DSF_INVALID_ARGUMENT;

    dsf_renderer_internal_state *state = (dsf_renderer_internal_state *)renderer;

    if (enabled)
    {
        static const uint32_t default_separators[] = {
            ' ', '\n', '\t', 0
        };

        if (separators == NULL)
            separators = default_separators;

        for (int i = 0; i < DSF_RENDERER_MAX_SEPARATORS; i++)
        {
            uint32_t c = separators[i];
            state->separators[i] = c;

            if (c == 0)
                break;
        }

        state->word_wrap = true;
    }
    else
    {
        state->word_wrap = false;
    }

    return DSF_NO_ERROR;
}

dsf_error DSF_RendererReset(dsf_renderer renderer)
{
    dsf_error ret;

    ret = DSF_RendererModeSetNone(renderer);
    if (ret != DSF_NO_ERROR)
        return ret;

    ret = DSF_RendererCanvasSetup(renderer, 0, 0, INT16_MAX, INT16_MAX);
    if (ret != DSF_NO_ERROR)
        return ret;

    ret = DSF_RendererUsedBoxReset(renderer);
    if (ret != DSF_NO_ERROR)
        return ret;

    ret = DSF_RendererCursorSet(renderer, 0, 0);
    if (ret != DSF_NO_ERROR)
        return ret;

    ret = DSF_RendererOverflowModeSet(renderer, DSF_RENDER_RIGHT_WRAP,
                                     DSF_RENDER_BOTTOM_TRIM);
    if (ret != DSF_NO_ERROR)
        return ret;

    ret = DSF_RendererWordWrapModeSet(renderer, true, NULL);
    if (ret != DSF_NO_ERROR)
        return ret;

    return DSF_NO_ERROR;
}

dsf_error DSF_RendererNew(dsf_renderer *renderer)
{
    if (renderer == NULL)
        return DSF_INVALID_ARGUMENT;

    dsf_renderer_internal_state *state = calloc(1, sizeof(dsf_renderer_internal_state));
    if (state == NULL)
        return DSF_NO_MEMORY;

    *renderer = (dsf_renderer)state;

    return DSF_RendererReset(*renderer);
}

dsf_error DSF_RendererCopy(dsf_renderer destination, dsf_renderer source)
{
    if ((destination == 0) || (source == 0))
        return DSF_INVALID_ARGUMENT;

    dsf_renderer_internal_state *state_src = (dsf_renderer_internal_state *)source;
    dsf_renderer_internal_state *state_dst = (dsf_renderer_internal_state *)destination;

    *state_dst = *state_src;

    return DSF_NO_ERROR;
}

dsf_error DSF_RendererFree(dsf_renderer renderer)
{
    if (renderer == 0)
        return DSF_INVALID_ARGUMENT;

    dsf_renderer_internal_state *state = (dsf_renderer_internal_state *)renderer;
    free(state);

    return DSF_NO_ERROR;
}

dsf_error DSF_RendererCursorGet(dsf_renderer renderer, int *x, int *y)
{
    if ((renderer == 0) || (x == NULL) || (y == NULL))
        return DSF_INVALID_ARGUMENT;

    dsf_renderer_internal_state *state = (dsf_renderer_internal_state *)renderer;

    *x = state->pointer_x;
    *y = state->pointer_y;

    return DSF_NO_ERROR;
}

dsf_error DSF_RendererCursorSet(dsf_renderer renderer, int16_t x, int16_t y)
{
    if (renderer == 0)
        return DSF_INVALID_ARGUMENT;

    dsf_renderer_internal_state *state = (dsf_renderer_internal_state *)renderer;

    state->pointer_x = x;
    state->pointer_y = y;

    // Kerning with the last rendered codepoint doesn't matter if the cursor
    // has moved.
    state->last_codepoint = 0;

    return DSF_NO_ERROR;
}

dsf_error DSF_RendererModeSetNone(dsf_renderer renderer)
{
    if (renderer == 0)
        return DSF_INVALID_ARGUMENT;

    dsf_renderer_internal_state *state = (dsf_renderer_internal_state *)renderer;

    state->render_mode = DSF_RENDER_MODE_NONE;

    return DSF_NO_ERROR;
}

dsf_error DSF_RendererModeSetBuffer(dsf_renderer renderer, dsf_format out_format,
                    void *out_texture, size_t out_width, size_t out_height)
{
    if ((renderer == 0) || (out_texture == NULL) ||
        (out_width == 0) || (out_height == 0))
        return DSF_INVALID_ARGUMENT;

    dsf_renderer_internal_state *state = (dsf_renderer_internal_state *)renderer;

    state->render_mode = DSF_RENDER_MODE_BUFFER;

    state->out_format = out_format;
    state->out_texture = out_texture;
    state->out_width = out_width;
    state->out_height = out_height;

    // Make sure that the canvas has sensible limits or we may write out of
    // bounds and corrupt memory.

    if (state->box_left < 0)
        state->box_left  = 0;
    if (state->box_top < 0)
        state->box_top  = 0;

    if ((size_t)state->box_right > out_width)
        state->box_right = out_width;
    if ((size_t)state->box_bottom > out_height)
        state->box_bottom = out_height;

    return DSF_NO_ERROR;
}

#ifdef __NDS__

dsf_error DSF_RendererModeSet3D(dsf_renderer renderer, int32_t z)
{
    if (renderer == 0)
        return DSF_INVALID_ARGUMENT;

    dsf_renderer_internal_state *state = (dsf_renderer_internal_state *)renderer;

    state->render_mode = DSF_RENDER_MODE_NDS_3D;

    state->z = z;

    return DSF_NO_ERROR;
}

dsf_error DSF_RendererModeSet3DAlpha(dsf_renderer renderer, int32_t z,
                    uint32_t poly_fmt, int poly_id_base)
{
    if (renderer == 0)
        return DSF_INVALID_ARGUMENT;

    dsf_renderer_internal_state *state = (dsf_renderer_internal_state *)renderer;

    state->render_mode = DSF_RENDER_MODE_NDS_3D_BLEND;

    state->z = z;
    state->poly_fmt = poly_fmt;
    state->poly_id_base = poly_id_base;
    state->id_index = 0;

    return DSF_NO_ERROR;
}

#endif // __NDS__

dsf_error DSF_StringRender(dsf_handle handle, dsf_renderer renderer,
                    const char *str)
{
    if ((handle == 0) || (renderer == 0) || (str == NULL))
        return DSF_INVALID_ARGUMENT;

    if (strlen(str) == 0)
        return DSF_INVALID_ARGUMENT;

    dsf_error ret = DSF_NO_ERROR;

    dsf_renderer_internal_state *state = (dsf_renderer_internal_state *)renderer;

#ifdef __NDS__
    if (state->render_mode == DSF_RENDER_MODE_NDS_3D)
        glBegin(GL_QUADS);
#endif // __NDS__

    const char *readptr = str;

    if (state->word_wrap)
    {
        // Create a copy of the renderer and set it to dry-run mode. The copy will
        // have the same canvas size as the original one.
        dsf_renderer_internal_state dry_run_state;
        dsf_renderer dry_run_renderer = (dsf_renderer)&dry_run_state;

        ret = DSF_RendererCopy(dry_run_renderer, renderer);
        if (ret != DSF_NO_ERROR)
            goto end;

        ret = DSF_RendererModeSetNone(dry_run_renderer);
        if (ret != DSF_NO_ERROR)
            goto end;

        // Check if the first character is a word or separator
        bool inside_word = !DSF_UTF8_StringStartsWith(readptr, state->separators);

        while (*readptr != '\0')
        {
            if (inside_word)
            {
                if (!DSF_UTF8_StringStartsWith(readptr, state->separators))
                {
                    // Still inside the word
                }
                else
                {
                    // We've reached the end of the word
                    inside_word = false;
                }
            }
            else
            {
                if (DSF_UTF8_StringStartsWith(readptr, state->separators))
                {
                    // Still outside of the word
                }
                else
                {
                    // We've reached the start of a word
                    inside_word = true;

                    // Get number of codepoints of the word that has just
                    // started. If it doesn't fit in this line, jump to the next
                    // one. After that, don't try to adjust the word anymore,
                    // simply do glyph wrap.
                    size_t len = DSF_UTF8_WordLength(readptr, state->separators);
                    if (len > 0)
                    {
                        // We set the cursor of the dry-run renderer to the current
                        // posistion of the regular renderer, and see if the next
                        // word overflows to the next line.
                        dry_run_state.pointer_x = state->pointer_x;
                        dry_run_state.pointer_y = state->pointer_y;

                        // Print the next word with the dry-run renderer
                        const char *word = readptr;
                        ret = DSF_StringRenderLength(handle, dry_run_renderer, &len, &word);
                        if (ret != DSF_NO_ERROR)
                            goto end;

                        // If the word didn't fit in this line, advance a line.
                        if (dry_run_state.pointer_y != state->pointer_y)
                        {
                            ret = DSF_CodepointRender(handle, renderer, '\n');
                            if (ret != DSF_NO_ERROR)
                                goto end;
                        }
                    }
                }
            }

            uint32_t codepoint;
            size_t size = DSF_UTF8_CodepointRead(readptr, &codepoint);
            readptr += size;

            ret = DSF_CodepointRender(handle, renderer, codepoint);
            if (ret != DSF_NO_ERROR)
                goto end;
        }
    }
    else // No word wrap
    {
        while (*readptr != '\0')
        {
            uint32_t codepoint;
            size_t size = DSF_UTF8_CodepointRead(readptr, &codepoint);
            readptr += size;

            ret = DSF_CodepointRender(handle, renderer, codepoint);
            if (ret != DSF_NO_ERROR)
                goto end;
        }
    }

end:
#ifdef __NDS__
    if (state->render_mode == DSF_RENDER_MODE_NDS_3D)
        glEnd();
#endif // __NDS__

    return ret;
}

dsf_error DSF_StringRenderFormat(dsf_handle handle, dsf_renderer renderer,
                    const char *str, ...)
{
    if ((handle == 0) || (renderer == 0) || (str == NULL))
        return DSF_INVALID_ARGUMENT;

    if (strlen(str) == 0)
        return DSF_INVALID_ARGUMENT;

    char *buffer = NULL;

    va_list args;
    va_start(args, str);
    int rc = vasprintf(&buffer, str, args);
    va_end(args);

    // This can happen with more errors than running out of memory
    if (rc == -1)
        return DSF_NO_MEMORY;

    dsf_error ret = DSF_StringRender(handle, renderer, buffer);

    free(buffer);

    return ret;
}

dsf_error DSF_StringRenderLength(dsf_handle handle, dsf_renderer renderer,
                    size_t *characters, const char **str)
{
    if ((handle == 0) || (renderer == 0) || (characters == NULL) || (str == NULL))
        return DSF_INVALID_ARGUMENT;

    const char *readptr = *str;
    size_t left = *characters;

    if ((readptr == NULL) || (left == 0))
        return DSF_INVALID_ARGUMENT;

    if (strlen(readptr) == 0)
        return DSF_INVALID_ARGUMENT;

#ifdef __NDS__
    dsf_renderer_internal_state *state = (dsf_renderer_internal_state *)renderer;

    if (state->render_mode == DSF_RENDER_MODE_NDS_3D)
        glBegin(GL_QUADS);
#endif // __NDS__

    dsf_error ret = DSF_NO_ERROR;

    while (*readptr != '\0')
    {
        // Note: This function can't support word wrap because it's supposed to
        // receive incomplete strings by design. That makes it impossible to
        // know if a newline is required or not.

        uint32_t codepoint;
        size_t size = DSF_UTF8_CodepointRead(readptr, &codepoint);
        readptr += size;

        left--;

        ret = DSF_CodepointRender(handle, renderer, codepoint);
        if (ret != DSF_NO_ERROR)
            break;

        if (left == 0)
            break;
    }

#ifdef __NDS__
    if (state->render_mode == DSF_RENDER_MODE_NDS_3D)
        glEnd();
#endif // __NDS__

    *characters = left;
    *str = readptr;

    return ret;
}
