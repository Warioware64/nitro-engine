// SPDX-License-Identifier: Zlib OR MIT
//
// Copyright (c) 2024-2026 Antonio Niño Díaz

#ifndef BMF_H__
#define BMF_H__

// BMFont format structures
// ------------------------

typedef struct __attribute__((packed)) {
    uint8_t magic[3]; // "BMF"
    uint8_t version;  // 3
} bmf_header;

typedef struct __attribute__((packed)) {
    uint8_t type;     // 1, 2, 3, 4 or 5
    uint8_t size[4];  // Size of the block
    uint8_t data[];
} bmf_block_header;

typedef struct __attribute__((packed)) {
    uint16_t line_height; // Distance in pixels between each line of text.
    uint16_t base; // Number of pixels from the top of the line to the base of the characters.
    uint16_t scale_w;
    uint16_t scale_h;
    uint16_t pages;
    uint8_t  bit_field;
    uint8_t  alpha_channel;
    uint8_t  red_channel;
    uint8_t  green_channel;
    uint8_t  blue_channel;
} bmf_block_2_common;

// The number of characters in one file can be calculated by calculating the
// result of "size / sizeof(bmf_block_4_char)"
typedef struct __attribute__((packed)) {
    uint32_t id;
    uint16_t x;
    uint16_t y;
    uint16_t width;
    uint16_t height;
    int16_t  xoffset;
    int16_t  yoffset;
    int16_t  xadvance;
    uint8_t  page;
    uint8_t  channel;
} bmf_block_4_char;

typedef struct __attribute__((packed)) {
    uint32_t first;
    uint32_t second;
    uint16_t amount;
} bmf_block_5_kerning_pair;

#endif // BMF_H__
