// SPDX-License-Identifier: MIT
//
// Copyright (c) 2008-2022 Antonio Niño Díaz
//
// This file is part of Nitro Engine Advanced

#include "NEAMain.h"

/// @file NEA2D.c

static NEA_Sprite **NEA_spritepointers = NULL;

static int NEA_MAX_SPRITES;

static bool ne_sprite_system_inited = false;

NEA_Sprite *NEA_SpriteCreate(void)
{
    if (!ne_sprite_system_inited)
    {
        NEA_DebugPrint("System not initialized");
        return NULL;
    }

    for (int i = 0; i < NEA_MAX_SPRITES; i++)
    {
        if (NEA_spritepointers[i] != NULL)
            continue;

        NEA_Sprite *sprite = calloc(1, sizeof(NEA_Sprite));
        if (sprite == NULL)
        {
            NEA_DebugPrint("Not enough memory");
            return NULL;
        }

        sprite->visible = true;
        sprite->xscale = inttof32(1);
        sprite->yscale = inttof32(1);
        sprite->color = NEA_White;
        sprite->mat = NULL;
        sprite->alpha = 31;

        NEA_spritepointers[i] = sprite;

        return sprite;
    }

    NEA_DebugPrint("No free slots");
    return NULL;
}

void NEA_SpriteSetPos(NEA_Sprite *sprite, int x, int y)
{
    NEA_AssertPointer(sprite, "NULL pointer");
    sprite->x = x;
    sprite->y = y;
}

void NEA_SpriteSetSize(NEA_Sprite *sprite, int w, int h)
{
    NEA_AssertPointer(sprite, "NULL pointer");
    sprite->w = w;
    sprite->h = h;
}

void NEA_SpriteSetRot(NEA_Sprite *sprite, int angle)
{
    NEA_AssertPointer(sprite, "NULL pointer");
    sprite->rot_angle = angle;
}

void NEA_SpriteSetScaleI(NEA_Sprite *sprite, int scale)
{
    NEA_AssertPointer(sprite, "NULL pointer");
    sprite->xscale = scale;
    sprite->yscale = scale;
}

void NEA_SpriteSetXScaleI(NEA_Sprite *sprite, int scale)
{
    NEA_AssertPointer(sprite, "NULL pointer");
    sprite->xscale = scale;
}

void NEA_SpriteSetYScaleI(NEA_Sprite *sprite, int scale)
{
    NEA_AssertPointer(sprite, "NULL pointer");
    sprite->yscale = scale;
}

void NEA_SpriteSetMaterial(NEA_Sprite *sprite, NEA_Material *mat)
{
    NEA_AssertPointer(sprite, "NULL sprite pointer");
    NEA_AssertPointer(mat, "NULL material pointer");
    sprite->mat = mat;

    int mat_w = NEA_TextureGetSizeX(mat);
    int mat_h = NEA_TextureGetSizeY(mat);

    sprite->w = mat_w;
    sprite->h = mat_h;

    sprite->tl = 0;
    sprite->tr = mat_w;
    sprite->tt = 0;
    sprite->tb = mat_h;
}

void NEA_SpriteSetMaterialCanvas(NEA_Sprite *sprite, int tl, int tt, int tr, int tb)
{
    NEA_AssertPointer(sprite, "NULL sprite pointer");
    NEA_AssertPointer(sprite->mat, "Sprite doesn't have a material");

    sprite->tl = tl;
    sprite->tr = tr;
    sprite->tt = tt;
    sprite->tb = tb;
}

void NEA_SpriteSetPriority(NEA_Sprite *sprite, int priority)
{
    NEA_AssertPointer(sprite, "NULL pointer");
    sprite->priority = priority;
}

void NEA_SpriteVisible(NEA_Sprite *sprite, bool visible)
{
    NEA_AssertPointer(sprite, "NULL pointer");
    sprite->visible = visible;
}

void NEA_SpriteSetParams(NEA_Sprite *sprite, u8 alpha, u8 id, u32 color)
{
    NEA_AssertPointer(sprite, "NULL pointer");
    NEA_AssertMinMax(0, alpha, 31, "Invalid alpha value %d", alpha);
    NEA_AssertMinMax(0, id, 63, "Invalid polygon ID %d", id);

    sprite->alpha = alpha;
    sprite->id = id;
    sprite->color = color;
}

void NEA_SpriteDelete(NEA_Sprite *sprite)
{
    if (!ne_sprite_system_inited)
        return;

    NEA_AssertPointer(sprite, "NULL pointer");

    for (int i = 0; i < NEA_MAX_SPRITES; i++)
    {
        if (NEA_spritepointers[i] != sprite)
            continue;

        NEA_spritepointers[i] = NULL;
        free((void *)sprite);

        return;
    }

    NEA_DebugPrint("Object not found");
    return;
}

void NEA_SpriteDeleteAll(void)
{
    if (!ne_sprite_system_inited)
        return;

    for (int i = 0; i < NEA_MAX_SPRITES; i++)
        NEA_SpriteDelete(NEA_spritepointers[i]);
}

int NEA_SpriteSystemReset(int max_sprites)
{
    if (ne_sprite_system_inited)
        NEA_SpriteSystemEnd();

    if (max_sprites < 1)
        NEA_MAX_SPRITES = NEA_DEFAULT_SPRITES;
    else
        NEA_MAX_SPRITES = max_sprites;

    NEA_spritepointers = calloc(NEA_MAX_SPRITES, sizeof(NEA_spritepointers));
    if (NEA_spritepointers == NULL)
    {
        NEA_DebugPrint("Not enough memory");
        return -1;
    }

    ne_sprite_system_inited = true;
    return 0;
}

void NEA_SpriteSystemEnd(void)
{
    if (!ne_sprite_system_inited)
        return;

    NEA_SpriteDeleteAll();

    free(NEA_spritepointers);

    ne_sprite_system_inited = false;
}

void NEA_SpriteDraw(const NEA_Sprite *sprite)
{
    if (!ne_sprite_system_inited)
        return;

    NEA_AssertPointer(sprite, "NULL pointer");

    if (!sprite->visible)
        return;

    if (sprite->rot_angle)
    {
        MATRIX_PUSH = 0;

        NEA_2DViewRotateScaleByPositionXYI(sprite->x + (sprite->w >> 1),
                                        sprite->y + (sprite->h >> 1),
                                        sprite->rot_angle,
                                        sprite->xscale, sprite->yscale);
    }
    else
    {
        NEA_2DViewScaleByPositionXYI(sprite->x + (sprite->w >> 1),
                                  sprite->y + (sprite->h >> 1),
                                  sprite->xscale, sprite->yscale);
    }

    GFX_POLY_FORMAT = POLY_ALPHA(sprite->alpha) | POLY_ID(sprite->id) |
                      NEA_CULL_NONE;

    NEA_2DDrawTexturedQuadColorCanvas(sprite->x, sprite->y,
                                     sprite->x + sprite->w,
                                     sprite->y + sprite->h,
                                     sprite->priority,
                                     sprite->tl, sprite->tt,
                                     sprite->tr, sprite->tb,
                                     sprite->mat, sprite->color);

    if (sprite->rot_angle)
        MATRIX_POP = 1;
}

void NEA_SpriteDrawAll(void)
{
    if (!ne_sprite_system_inited)
        return;

    for (int i = 0; i < NEA_MAX_SPRITES; i++)
    {
        if (NEA_spritepointers[i] == NULL)
            continue;

        NEA_Sprite *sprite = NEA_spritepointers[i];

        if (!sprite->visible)
            continue;

        if (sprite->rot_angle)
        {
            MATRIX_PUSH = 0;

            NEA_2DViewRotateScaleByPositionXYI(sprite->x + (sprite->w >> 1),
                                            sprite->y + (sprite->h >> 1),
                                            sprite->rot_angle,
                                            sprite->xscale, sprite->yscale);
        }
        else
        {
            NEA_2DViewScaleByPositionXYI(sprite->x + (sprite->w >> 1),
                                      sprite->y + (sprite->h >> 1),
                                      sprite->xscale, sprite->yscale);
        }

        GFX_POLY_FORMAT = POLY_ALPHA(sprite->alpha) |
                          POLY_ID(sprite->id) | NEA_CULL_NONE;

        NEA_2DDrawTexturedQuadColorCanvas(sprite->x, sprite->y,
                                         sprite->x + sprite->w,
                                         sprite->y + sprite->h,
                                         sprite->priority,
                                         sprite->tl, sprite->tt,
                                         sprite->tr, sprite->tb,
                                         sprite->mat, sprite->color);

        if (sprite->rot_angle)
            MATRIX_POP = 1;
    }
}

//----------------------------------------------------------
//
//              Functions to draw freely in 2D.
//
//----------------------------------------------------------

// Internal use. See NEATexture.c

int __NEA_TextureGetRawX(const NEA_Material *tex);
int __NEA_TextureGetRawY(const NEA_Material *tex);

//--------------------------------------------

void NEA_2DViewInit(void)
{
    GFX_VIEWPORT = 0 | (0 << 8) | (255 << 16) | (191 << 24);

    // The projection matrix actually thinks that the size of the DS is
    // (256 << factor) x (192 << factor). After this, we scale the MODELVIEW
    // matrix to match this scale factor.
    //
    // This way, it is possible to draw on the screen by using numbers up to 256
    // x 192, but internally the DS has more digits when it does transformations
    // like a rotation. Not having this factor results in noticeable flickering,
    // specially in some emulators.
    //
    // Unfortunately, applying this factor reduces the accuracy of the Y
    // coordinate a lot (nothing is noticeable in the X coordinate). Any factor
    // over 4 starts showing a noticeable accuracy loss: some sprites start
    // being slightly distorted, with missing some horizontal lines as the
    // height is reduced. When the number is higher, like 12, the Y coordinate
    // is significantly compressed. When the number is even higher, like 18, the
    // polygons disappear because too much accuracy has been lost.
    //
    // The current solution is to compromise, and use a factor of 2, which
    // doesn't cause any distortion, and solves most of the flickering. Ideally
    // we would use 0 to simplify the calculations, but we want to reduce the
    // flickering.
    //
    // On hardware, the difference in flickering between 0 and 2 isn't too
    // noticeable, but it is noticeable. In DeSmuMe it is very noticeable.
    // In my tests, Y axis distortion starts to happen with a factor of 4, so a
    // factor of 2 should be safe and reduce enough flickering.

    MATRIX_CONTROL = GL_PROJECTION;
    MATRIX_IDENTITY = 0;

    int factor = 2;

    glOrthof32(0, 256 << factor, 192 << factor, 0, inttof32(1), inttof32(-1));

    MATRIX_CONTROL = GL_MODELVIEW;
    MATRIX_IDENTITY = 0;

    MATRIX_SCALE = inttof32(1 << factor);
    MATRIX_SCALE = inttof32(1 << factor);
    MATRIX_SCALE = inttof32(1);

    NEA_PolyFormat(31, 0, 0, NEA_CULL_NONE, 0);
}

void NEA_2DViewRotateScaleByPositionXYI(int x, int y, int rotz, int xscale, int yscale)
{
    NEA_ViewMoveI(x, y, 0);

    MATRIX_SCALE = xscale;
    MATRIX_SCALE = yscale;
    MATRIX_SCALE = inttof32(1);

    glRotateZi(rotz << 6);

    NEA_ViewMoveI(-x, -y, 0);
}

void NEA_2DViewScaleByPositionI(int x, int y, int scale) {
    NEA_2DViewScaleByPositionXYI(x, y, scale, scale);
}

void NEA_2DViewRotateByPosition(int x, int y, int rotz)
{
    NEA_ViewMoveI(x, y, 0);

    glRotateZi(rotz << 6);

    NEA_ViewMoveI(-x, -y, 0);
}

void NEA_2DViewScaleByPositionXYI(int x, int y, int xscale, int yscale)
{
    NEA_ViewMoveI(x, y, 0);

    MATRIX_SCALE = xscale;
    MATRIX_SCALE = yscale;
    MATRIX_SCALE = inttof32(1);

    NEA_ViewMoveI(-x, -y, 0);
}

void NEA_2DViewRotateScaleByPositionI(int x, int y, int rotz, int scale) {
    NEA_2DViewRotateScaleByPositionXYI(x, y, rotz, scale, scale);
}

void NEA_2DDrawQuad(s16 x1, s16 y1, s16 x2, s16 y2, s16 z, u32 color)
{
    GFX_BEGIN = GL_QUADS;

    GFX_TEX_FORMAT = 0;

    GFX_COLOR = color;

    GFX_VERTEX16 = (y1 << 16) | (x1 & 0xFFFF); // Up-left
    GFX_VERTEX16 = z;

    GFX_VERTEX_XY = (y2 << 16) | (x1 & 0xFFFF); // Down-left

    GFX_VERTEX_XY = (y2 << 16) | (x2 & 0xFFFF); // Down-right

    GFX_VERTEX_XY = (y1 << 16) | (x2 & 0xFFFF); // Up-right
}

void NEA_2DDrawQuadGradient(s16 x1, s16 y1, s16 x2, s16 y2, s16 z, u32 color1,
                           u32 color2, u32 color3, u32 color4)
{
    GFX_BEGIN = GL_QUADS;

    GFX_TEX_FORMAT = 0;

    GFX_COLOR = color1;
    GFX_VERTEX16 = (y1 << 16) | (x1 & 0xFFFF); // Up-left
    GFX_VERTEX16 = z;

    GFX_COLOR = color4;
    GFX_VERTEX_XY = (y2 << 16) | (x1 & 0xFFFF); // Down-left

    GFX_COLOR = color3;
    GFX_VERTEX_XY = (y2 << 16) | (x2 & 0xFFFF); // Down-right

    GFX_COLOR = color2;
    GFX_VERTEX_XY = (y1 << 16) | (x2 & 0xFFFF); // Up-right
}

void NEA_2DDrawTexturedQuad(s16 x1, s16 y1, s16 x2, s16 y2, s16 z,
                           const NEA_Material *mat)
{
    NEA_AssertPointer(mat, "NULL pointer");
    NEA_Assert(mat->texindex != NEA_NO_TEXTURE, "No texture");

    int x = NEA_TextureGetSizeX(mat), y = NEA_TextureGetSizeY(mat);

    NEA_MaterialUse(mat);

    GFX_BEGIN = GL_QUADS;

    GFX_TEX_COORD = TEXTURE_PACK(0, 0);
    GFX_VERTEX16 = (y1 << 16) | (x1 & 0xFFFF); // Up-left
    GFX_VERTEX16 = z;

    GFX_TEX_COORD = TEXTURE_PACK(0, inttot16(y));
    GFX_VERTEX_XY = (y2 << 16) | (x1 & 0xFFFF); // Down-left

    GFX_TEX_COORD = TEXTURE_PACK(inttot16(x), inttot16(y));
    GFX_VERTEX_XY = (y2 << 16) | (x2 & 0xFFFF); // Down-right

    GFX_TEX_COORD = TEXTURE_PACK(inttot16(x), 0);
    GFX_VERTEX_XY = (y1 << 16) | (x2 & 0xFFFF); // Up-right
}

void NEA_2DDrawTexturedQuadColor(s16 x1, s16 y1, s16 x2, s16 y2, s16 z,
                                const NEA_Material *mat, u32 color)
{
    NEA_AssertPointer(mat, "NULL pointer");
    NEA_Assert(mat->texindex != NEA_NO_TEXTURE, "No texture");

    int x = NEA_TextureGetSizeX(mat), y = NEA_TextureGetSizeY(mat);

    NEA_MaterialUse(mat);

    GFX_COLOR = color;

    GFX_BEGIN = GL_QUADS;

    GFX_TEX_COORD = TEXTURE_PACK(0, 0);
    GFX_VERTEX16 = (y1 << 16) | (x1 & 0xFFFF); // Up-left
    GFX_VERTEX16 = z;

    GFX_TEX_COORD = TEXTURE_PACK(0, inttot16(y));
    GFX_VERTEX_XY = (y2 << 16) | (x1 & 0xFFFF); // Down-left

    GFX_TEX_COORD = TEXTURE_PACK(inttot16(x), inttot16(y));
    GFX_VERTEX_XY = (y2 << 16) | (x2 & 0xFFFF); // Down-right

    GFX_TEX_COORD = TEXTURE_PACK(inttot16(x), 0);
    GFX_VERTEX_XY = (y1 << 16) | (x2 & 0xFFFF); // Up-right
}

void NEA_2DDrawTexturedQuadGradient(s16 x1, s16 y1, s16 x2, s16 y2, s16 z,
                                   const NEA_Material *mat, u32 color1,
                                   u32 color2, u32 color3, u32 color4)
{
    NEA_AssertPointer(mat, "NULL pointer");
    NEA_Assert(mat->texindex != NEA_NO_TEXTURE, "No texture");

    int x = NEA_TextureGetSizeX(mat), y = NEA_TextureGetSizeY(mat);

    NEA_MaterialUse(mat);

    GFX_BEGIN = GL_QUADS;

    GFX_COLOR = color1;
    GFX_TEX_COORD = TEXTURE_PACK(0, 0);
    GFX_VERTEX16 = (y1 << 16) | (x1 & 0xFFFF); // Up-left
    GFX_VERTEX16 = z;

    GFX_COLOR = color4;
    GFX_TEX_COORD = TEXTURE_PACK(0, inttot16(y));
    GFX_VERTEX_XY = (y2 << 16) | (x1 & 0xFFFF); // Down-left

    GFX_COLOR = color3;
    GFX_TEX_COORD = TEXTURE_PACK(inttot16(x), inttot16(y));
    GFX_VERTEX_XY = (y2 << 16) | (x2 & 0xFFFF); // Down-right

    GFX_COLOR = color2;
    GFX_TEX_COORD = TEXTURE_PACK(inttot16(x), 0);
    GFX_VERTEX_XY = (y1 << 16) | (x2 & 0xFFFF); // Up-right
}

void NEA_2DDrawTexturedQuadColorCanvas(s16 x1, s16 y1, s16 x2, s16 y2, s16 z,
                                      int tl, int tt, int tr, int tb,
                                      const NEA_Material *mat, u32 color)
{
    NEA_AssertPointer(mat, "NULL pointer");
    NEA_Assert(mat->texindex != NEA_NO_TEXTURE, "No texture");

    NEA_MaterialUse(mat);

    GFX_COLOR = color;

    GFX_BEGIN = GL_QUADS;

    GFX_TEX_COORD = TEXTURE_PACK(inttot16(tl), inttot16(tt));
    GFX_VERTEX16 = (y1 << 16) | (x1 & 0xFFFF); // Up-left
    GFX_VERTEX16 = z;

    GFX_TEX_COORD = TEXTURE_PACK(inttot16(tl), inttot16(tb));
    GFX_VERTEX_XY = (y2 << 16) | (x1 & 0xFFFF); // Down-left

    GFX_TEX_COORD = TEXTURE_PACK(inttot16(tr), inttot16(tb));
    GFX_VERTEX_XY = (y2 << 16) | (x2 & 0xFFFF); // Down-right

    GFX_TEX_COORD = TEXTURE_PACK(inttot16(tr), inttot16(tt));
    GFX_VERTEX_XY = (y1 << 16) | (x2 & 0xFFFF); // Up-right
}
