// SPDX-License-Identifier: Zlib OR MIT
//
// Copyright (c) 2024-2026 Antonio Niño Díaz

#include <stdint.h>
#include <stdlib.h>

#include <dsf.h>

#include "renderer.h"

dsf_error DSF_StringRenderDryRunWithCursor(dsf_handle handle, const char *str,
                                           size_t *size_x, size_t *size_y,
                                           size_t *final_x, size_t *final_y)
{
    // Create a temporary renderer
    dsf_renderer_internal_state state = { 0 };
    dsf_renderer renderer = (dsf_renderer)&state;

    dsf_error ret;

    ret = DSF_RendererReset(renderer);
    if (ret != DSF_NO_ERROR)
        return ret;

    ret = DSF_StringRender(handle, renderer, str);
    if (ret != DSF_NO_ERROR)
        return ret;

    *size_x = state.used_right;
    *size_y = state.used_bottom;
    *final_x = state.pointer_x;
    *final_y = state.pointer_y;

    return DSF_NO_ERROR;
}

dsf_error DSF_StringRenderDryRun(dsf_handle handle, const char *str,
                                 size_t *size_x, size_t *size_y)
{
    size_t final_x, final_y;
    return DSF_StringRenderDryRunWithCursor(handle, str, size_x, size_y,
                                            &final_x, &final_y);
}

#ifdef __NDS__

dsf_error DSF_StringRender3DWithIndent(dsf_handle handle, const char *str,
                             int32_t x, int32_t y, int32_t z,
                             int32_t xStart)
{
    // Create a temporary renderer
    dsf_renderer_internal_state state = { 0 };
    dsf_renderer renderer = (dsf_renderer)&state;

    dsf_error ret;

    ret = DSF_RendererReset(renderer);
    if (ret != DSF_NO_ERROR)
        return ret;

    ret = DSF_RendererModeSet3D(renderer, z);
    if (ret != DSF_NO_ERROR)
        return ret;

    ret = DSF_RendererCanvasSetup(renderer, x, y, INT16_MAX, INT16_MAX);
    if (ret != DSF_NO_ERROR)
        return ret;

    ret = DSF_RendererCursorSet(renderer, x + xStart, y);
    if (ret != DSF_NO_ERROR)
        return ret;

    ret = DSF_StringRender(handle, renderer, str);
    if (ret != DSF_NO_ERROR)
        return ret;

    return DSF_NO_ERROR;
}

dsf_error DSF_StringRender3D(dsf_handle handle, const char *str,
                             int32_t x, int32_t y, int32_t z)
{
    return DSF_StringRender3DWithIndent(handle, str, x, y, z, 0);
}

dsf_error DSF_StringRender3DAlphaWithIndent(dsf_handle handle, const char *str,
                                            int32_t x, int32_t y, int32_t z,
                                            uint32_t poly_fmt, int poly_id_base,
                                            int32_t xStart)
{
    // Create a temporary renderer
    dsf_renderer_internal_state state = { 0 };
    dsf_renderer renderer = (dsf_renderer)&state;

    dsf_error ret;

    ret = DSF_RendererReset(renderer);
    if (ret != DSF_NO_ERROR)
        return ret;

    ret = DSF_RendererModeSet3DAlpha(renderer, z, poly_fmt, poly_id_base);
    if (ret != DSF_NO_ERROR)
        return ret;

    ret = DSF_RendererCanvasSetup(renderer, x, y, INT16_MAX, INT16_MAX);
    if (ret != DSF_NO_ERROR)
        return ret;

    ret = DSF_RendererCursorSet(renderer, x + xStart, y);
    if (ret != DSF_NO_ERROR)
        return ret;

    ret = DSF_StringRender(handle, renderer, str);
    if (ret != DSF_NO_ERROR)
        return ret;

    return DSF_NO_ERROR;
}

dsf_error DSF_StringRender3DAlpha(dsf_handle handle, const char *str,
                                  int32_t x, int32_t y, int32_t z,
                                  uint32_t poly_fmt, int poly_id_base)
{
    return DSF_StringRender3DAlphaWithIndent(handle, str, x, y, z, poly_fmt,
                                             poly_id_base, 0);
}

#endif // __NDS__

dsf_error DSF_StringRenderToExistingBuffer(dsf_handle handle,
                    const char *str, dsf_format texture_fmt,
                    const void *font_texture, size_t font_width, size_t font_height,
                    void *out_texture, size_t out_width, size_t out_height,
                    uint32_t out_x, uint32_t out_y)
{
    // Create a temporary renderer
    dsf_renderer_internal_state state = { 0 };
    dsf_renderer renderer = (dsf_renderer)&state;

    dsf_error ret;

    ret = DSF_RendererReset(renderer);
    if (ret != DSF_NO_ERROR)
        return ret;

    ret = DSF_FontTextureSet(handle, font_texture, font_width, font_height, texture_fmt);
    if (ret != DSF_NO_ERROR)
        return ret;

    ret = DSF_RendererModeSetBuffer(renderer, texture_fmt, out_texture, out_width, out_height);
    if (ret != DSF_NO_ERROR)
        return ret;

    ret = DSF_RendererCursorSet(renderer, out_x, out_y);
    if (ret != DSF_NO_ERROR)
        return ret;

    ret = DSF_StringRender(handle, renderer, str);
    if (ret != DSF_NO_ERROR)
        return ret;

    return DSF_NO_ERROR;
}

dsf_error DSF_StringRenderToNewBuffer(dsf_handle handle,
                    const char *str, dsf_format texture_fmt,
                    const void *font_texture, size_t font_width, size_t font_height,
                    void **out_texture, size_t *out_width, size_t *out_height)
{
    // Create a temporary renderer
    dsf_renderer_internal_state state = { 0 };
    dsf_renderer renderer = (dsf_renderer)&state;

    // Start process

    dsf_error ret = DSF_NO_ERROR;

    ret = DSF_RendererReset(renderer);
    if (ret != DSF_NO_ERROR)
        return ret;

    ret = DSF_FontTextureSet(handle, font_texture, font_width, font_height, texture_fmt);
    if (ret != DSF_NO_ERROR)
        return ret;

    // Get size

    ret = DSF_RendererModeSetNone(renderer);
    if (ret != DSF_NO_ERROR)
        return ret;

    ret = DSF_RendererCanvasSetup(renderer, 0, 0, INT16_MAX, INT16_MAX);
    if (ret != DSF_NO_ERROR)
        return ret;

    size_t tex_width, tex_height;

    ret = DSF_StringRender(handle, renderer, str);
    if (ret != DSF_NO_ERROR)
        return ret;

    ret = DSF_RendererUsedBoxGetTextureSize(renderer, &tex_width, &tex_height);
    if (ret != DSF_NO_ERROR)
        return ret;

    ret = DSF_RendererCursorSet(renderer, 0, 0); // Return to the start
    if (ret != DSF_NO_ERROR)
        return ret;

    // Allocate buffer

    void *tex_buffer;
    ret = DSF_BufferAlloc(handle, &tex_buffer, tex_width, tex_height);
    if (ret != DSF_NO_ERROR)
        return ret;

    // Render to it

    ret = DSF_RendererModeSetBuffer(renderer, texture_fmt, tex_buffer, tex_width, tex_height);
    if (ret != DSF_NO_ERROR)
    {
        free(tex_buffer);
        return ret;
    }

    ret = DSF_StringRender(handle, renderer, str);
    if (ret != DSF_NO_ERROR)
    {
        free(tex_buffer);
        return ret;
    }

    // Return texture information

    *out_texture = tex_buffer;
    *out_width = tex_width;
    *out_height = tex_height;

    return ret;
}
