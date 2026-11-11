// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Warioware64, 2026
//
// This file is part of Nitro Engine Advanced

// The head of an NPAC archive of stored members, as tools/npac/npac_format.py
// writes it, so that the test can write a big archive a cluster at a time
// without holding it. check_head.py compiles this on the PC and holds it to
// npac_format.py byte for byte.
//
// The archive: `count` members (at most NPAC_FRAG_MOST) in the root, named
// d0.bin, d1.bin..., each `bytes` long (a multiple of 4), stored. The head is
// everything before the first member's bytes; the members follow it one
// after the other.

#ifndef NPAC_WRITE_H__
#define NPAC_WRITE_H__

#include <stdint.h>
#include <string.h>

#define NPAC_FRAG_MOST      10
#define NPAC_FRAG_HEAD_MAX  256

static inline void npac_put16(uint8_t *at, uint32_t v)
{
    at[0] = (uint8_t)v;
    at[1] = (uint8_t)(v >> 8);
}

static inline void npac_put32(uint8_t *at, uint32_t v)
{
    npac_put16(at, v);
    npac_put16(at + 2, v >> 16);
}

// Writes the head into `out` and returns its length; the whole file's
// length goes to `total`. 0 if the archive cannot be written this way.
static inline uint32_t npac_frag_head(uint8_t *out, uint32_t count,
                                      uint32_t bytes, uint32_t *total)
{
    if ((count == 0) || (count > NPAC_FRAG_MOST) || ((bytes & 3) != 0) ||
        (bytes > 0xFFFFFF))
        return 0;

    uint32_t at = 16;

    // BTAF: each member's start and end in the image.
    uint32_t btaf = 8 + 4 + 8 * count;
    memcpy(out + at, "BTAF", 4);
    npac_put32(out + at + 4, btaf);
    npac_put16(out + at + 8, count);
    npac_put16(out + at + 10, 0);
    for (uint32_t i = 0; i < count; i++)
    {
        npac_put32(out + at + 12 + 8 * i, i * bytes);
        npac_put32(out + at + 16 + 8 * i, (i + 1) * bytes);
    }
    at += btaf;

    // BTNF: the root's entry, then its subtable (the names), padded to a
    // word with FFh.
    uint32_t fnt = 8 + 7 * count + 1;
    uint32_t padded = (fnt + 3) & ~3u;
    memcpy(out + at, "BTNF", 4);
    npac_put32(out + at + 4, 8 + padded);
    uint8_t *t = out + at + 8;
    npac_put32(t, 8);
    npac_put16(t + 4, 0);
    npac_put16(t + 6, 1);
    for (uint32_t i = 0; i < count; i++)
    {
        uint8_t *e = t + 8 + 7 * i;
        e[0] = 6;
        memcpy(e + 1, "d0.bin", 6);
        e[2] = (uint8_t)('0' + i);
    }
    t[fnt - 1] = 0;
    for (uint32_t i = fnt; i < padded; i++)
        t[i] = 0xFF;
    at += 8 + padded;

    // GMIF: the members.
    memcpy(out + at, "GMIF", 4);
    npac_put32(out + at + 4, 8 + count * bytes);
    at += 8;

    *total = at + count * bytes;
    memcpy(out, "NPAC", 4);
    npac_put16(out + 4, 0xFFFE);
    npac_put16(out + 6, 0x0100);
    npac_put32(out + 8, *total);
    npac_put16(out + 12, 16);
    npac_put16(out + 14, 3);
    return at;
}

// The test's bytes: past the head, each word is its own offset in the file,
// mixed, so that a word from anywhere else in the file is never right.
static inline uint32_t npac_frag_word(uint32_t offset)
{
    return offset ^ 0x5A5A5A5Au;
}

#endif // NPAC_WRITE_H__
