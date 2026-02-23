// SPDX-License-Identifier: MIT
//
// Copyright (c) 2008-2022 Antonio Niño Díaz
//
// This file is part of Nitro Engine Advanced

#include "NEAMain.h"

/// @file NEAText.c

typedef struct {
    int sizex, sizey;
    const NEA_Material *material;
} ne_textinfo_t;

static ne_textinfo_t NEA_TextInfo[NEA_MAX_TEXT_FONTS];

static int NEA_TEXT_PRIORITY = 0;

void NEA_TextPrioritySet(int priority)
{
    NEA_TEXT_PRIORITY = priority;
}

void NEA_TextPriorityReset(void)
{
    NEA_TEXT_PRIORITY = 0;
}

void NEA_TextInit(int slot, const NEA_Material *mat, int sizex, int sizey)
{
    NEA_AssertMinMax(0, slot, NEA_MAX_TEXT_FONTS, "Invalid slot %d", slot);
    NEA_AssertPointer(mat, "NULL pointer");

    NEA_TextInfo[slot].sizex = sizex;
    NEA_TextInfo[slot].sizey = sizey;
    NEA_TextInfo[slot].material = mat;
}

void NEA_TextEnd(int slot)
{
    NEA_AssertMinMax(0, slot, NEA_MAX_TEXT_FONTS, "Invalid slot %d", slot);

    NEA_TextInfo[slot].sizex = 0;
    NEA_TextInfo[slot].sizey = 0;
    NEA_TextInfo[slot].material = NULL;
}

void NEA_TextResetSystem(void)
{
    for (int i = 0; i < NEA_MAX_TEXT_FONTS; i++)
    {
        NEA_TextInfo[i].sizex = 0;
        NEA_TextInfo[i].sizey = 0;
        NEA_TextInfo[i].material = NULL;
    }
}

static void _ne_texturecuadprint(int xcrd1, int ycrd1, int xcrd2, int ycrd2,
                                 int xtx1, int ytx1, int xtx2, int ytx2)
{
    GFX_TEX_COORD = TEXTURE_PACK(inttot16(xtx1), inttot16(ytx1));
    GFX_VERTEX16 = (ycrd1 << 16) | (xcrd1 & 0xFFFF);
    GFX_VERTEX16 = NEA_TEXT_PRIORITY;

    GFX_TEX_COORD = TEXTURE_PACK(inttot16(xtx1), inttot16(ytx2));
    GFX_VERTEX_XY = (ycrd2 << 16) | (xcrd1 & 0xFFFF);

    GFX_TEX_COORD = TEXTURE_PACK(inttot16(xtx2), inttot16(ytx2));
    GFX_VERTEX_XY = (ycrd2 << 16) | (xcrd2 & 0xFFFF);

    GFX_TEX_COORD = TEXTURE_PACK(inttot16(xtx2), inttot16(ytx1));
    GFX_VERTEX_XY = (ycrd1 << 16) | (xcrd2 & 0xFFFF);
}

static void _ne_charprint(const ne_textinfo_t * textinfo, int xcrd1, int ycrd1,
                          char character)
{
    // Texture coords
    int xcoord = ((character & 31) * textinfo->sizex);
    int xcoord2 = (xcoord + textinfo->sizex);

    int ycoord = ((character >> 5) * textinfo->sizey);
    int ycoord2 = ycoord + textinfo->sizey;

    _ne_texturecuadprint(xcrd1, ycrd1,
                         xcrd1 + textinfo->sizex, ycrd1 + textinfo->sizey,
                         xcoord, ycoord, xcoord2, ycoord2);
}

int NEA_TextPrint(int slot, int x, int y, u32 color, const char *text)
{
    NEA_AssertMinMax(0, slot, NEA_MAX_TEXT_FONTS, "Invalid slot %d", slot);

    const ne_textinfo_t *textinfo = &NEA_TextInfo[slot];

    if (textinfo->material == NULL)
        return -1;

    NEA_MaterialUse(textinfo->material);
    GFX_COLOR = color;

    int count = 0;
    int x_ = x * textinfo->sizex, y_ = y * textinfo->sizey;

    GFX_BEGIN = GL_QUADS;

    while (1)
    {
        if (text[count] == '\0')
        {
            break;
        }
        else if (text[count] == '\n')
        {
            y_ += textinfo->sizey;
            x_ = 0;
            count++;
        }
        else
        {
            if (x_ > 255)
            {
                y_ += textinfo->sizey;
                x_ = 0;
            }
            if (y_ > 191)
                break;

            _ne_charprint(textinfo, x_, y_, text[count]);

            count++;
            x_ += textinfo->sizex;
        }
    }

    return count;
}

int NEA_TextPrintBox(int slot, int x, int y, int endx, int endy, u32 color,
                    int charnum, const char *text)
{
    NEA_AssertMinMax(0, slot, NEA_MAX_TEXT_FONTS, "Invalid slot %d", slot);

    const ne_textinfo_t *textinfo = &NEA_TextInfo[slot];

    if (textinfo->material == NULL)
        return -1;

    NEA_MaterialUse(textinfo->material);
    GFX_COLOR = color;

    int count = 0;
    int x_ = x * textinfo->sizex, y_ = y * textinfo->sizey;
    int xlimit = endx * textinfo->sizex, ylimit = endy * textinfo->sizey;
    ylimit = (ylimit > 191) ? 191 : ylimit;

    if (charnum < 0)
        charnum = 0x0FFFFFFF;

    GFX_BEGIN = GL_QUADS;

    while (1) {
        if (charnum <= count)
        {
            break;
        }
        else if (text[count] == '\0')
        {
            break;
        }
        else if (text[count] == '\n')
        {
            y_ += textinfo->sizey;
            x_ = x * textinfo->sizex;
            count++;
        }
        else
        {
            if (x_ > xlimit)
            {
                y_ += textinfo->sizey;
                x_ = x * textinfo->sizex;
            }
            if (y_ > ylimit)
                break;

            _ne_charprint(textinfo, x_, y_, text[count]);

            count++;
            x_ += textinfo->sizex;
        }
    }

    return count;
}

int NEA_TextPrintFree(int slot, int x, int y, u32 color, const char *text)
{
    NEA_AssertMinMax(0, slot, NEA_MAX_TEXT_FONTS, "Invalid slot %d", slot);

    const ne_textinfo_t *textinfo = &NEA_TextInfo[slot];

    if (textinfo->material == NULL)
        return -1;

    NEA_MaterialUse(textinfo->material);
    GFX_COLOR = color;

    int count = 0;
    int x_ = x, y_ = y;

    GFX_BEGIN = GL_QUADS;

    while (1) {
        if (text[count] == '\0')
        {
            break;
        }
        else
        {
            if (x_ > 255)
                break;

            _ne_charprint(textinfo, x_, y_, text[count]);

            count++;
            x_ += textinfo->sizex;
        }
    }

    return count;
}

int NEA_TextPrintBoxFree(int slot, int x, int y, int endx, int endy, u32 color,
                        int charnum, const char *text)
{
    NEA_AssertMinMax(0, slot, NEA_MAX_TEXT_FONTS, "Invalid slot %d", slot);

    ne_textinfo_t *textinfo = &NEA_TextInfo[slot];

    if (textinfo->material == NULL)
        return -1;

    NEA_MaterialUse(textinfo->material);
    GFX_COLOR = color;

    int count = 0;
    int x_ = x, y_ = y;
    int xlimit = endx;
    int ylimit = (endy > 191) ? 191 : endy;

    if (charnum < 0)
        charnum = 0x0FFFFFFF;

    GFX_BEGIN = GL_QUADS;

    while (1) {
        if (charnum <= count)
        {
            break;
        }
        else if (text[count] == '\0')
        {
            break;
        }
        else if (text[count] == '\n')
        {
            y_ += textinfo->sizey;
            x_ = x;
            count++;
        }
        else
        {
            if (x_ > xlimit)
            {
                y_ += textinfo->sizey;
                x_ = x;
            }
            if (y_ > ylimit)
                break;

            _ne_charprint(textinfo, x_, y_, text[count]);

            count++;
            x_ += textinfo->sizex;
        }
    }

    return count;
}
