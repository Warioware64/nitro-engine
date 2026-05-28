// SPDX-License-Identifier: Zlib OR MIT
//
// Copyright (c) 2026 Antonio Niño Díaz

#ifndef RENDERER_H__
#define RENDERER_H__

#include <stdbool.h>
#include <stdint.h>

#include <dsf.h>

typedef enum {
    DSF_RENDER_MODE_NONE,
    DSF_RENDER_MODE_BUFFER,
#ifdef __NDS__
    DSF_RENDER_MODE_NDS_3D,
    DSF_RENDER_MODE_NDS_3D_BLEND,
#endif // __NDS__
} dsf_render_mode;

typedef struct {
    // Variables used for the current printing context
    int16_t pointer_x;
    int16_t pointer_y;

    // Canvas dimensions
    int16_t box_left;
    int16_t box_top;
    int16_t box_right;
    int16_t box_bottom;

    // Maximum coordinates used by the text that has been printed
    int16_t used_left;
    int16_t used_top;
    int16_t used_right;
    int16_t used_bottom;

    // Last codepoint that has been printed. Used for kerning.
    uint32_t last_codepoint;

    // Mode used to render text
    dsf_render_mode render_mode;

    // How to behave when the edges of the canvas are reached
    dsf_render_right_mode render_right_mode;
    dsf_render_bottom_mode render_bottom_mode;

    // Word wrap settings
    bool word_wrap;
    uint32_t separators[DSF_RENDERER_MAX_SEPARATORS];

    // Information about the output buffer in buffer rendering mode
    dsf_format out_format;
    void *out_texture;
    size_t out_width;
    size_t out_height;

#ifdef __NDS__

    // Parameters used in 3D rendering mode
    int32_t z;
    uint32_t poly_fmt;
    int poly_id_base;
    int id_index;

#endif // __NDS__

} dsf_renderer_internal_state;

#endif // RENDERER_H__
