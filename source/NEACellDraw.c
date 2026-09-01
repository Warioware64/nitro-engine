// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Warioware64
//
// This file is part of Nitro Engine Advanced
//
// The two 3D backends of the cell system: textured quads in screen space, and
// the same quads turned to face the camera in world space. Both read the pose
// NEACell.c resolved and nothing else, which is what keeps them agreeing with
// each other and with the hardware OBJ backend.

#include "NEAMain.h"

// The camera billboards face. Owned by NEACell.c, set with
// NEA_CellAnimSetCamera().
extern NEA_Camera *__ne_cell_camera;

//-----------------------------------------------------------------------------
// Shared: which material a part draws with, and its texture coordinates.
//-----------------------------------------------------------------------------

typedef struct {
    const NEA_CellPart *src;
    int32_t u0, v0, u1, v1; // t16 texture coordinates, flips already applied
} ne_cell_uv_t;

static void ne_cell_uv(const NEA_CellPartXform *x, ne_cell_uv_t *out)
{
    const NEA_CellPart *src = x->src;

    int32_t u0 = inttot16(src->src_x);
    int32_t v0 = inttot16(src->src_y);
    int32_t u1 = inttot16(src->src_x + src->src_w);
    int32_t v1 = inttot16(src->src_y + src->src_h);

    // Flipping is free in the texture coordinates, so it never costs a matrix.
    if (x->part->flags & NEA_CELL_PART_HFLIP)
    {
        int32_t t = u0;
        u0 = u1;
        u1 = t;
    }
    if (x->part->flags & NEA_CELL_PART_VFLIP)
    {
        int32_t t = v0;
        v0 = v1;
        v1 = t;
    }

    out->src = src;
    out->u0 = u0;
    out->v0 = v0;
    out->u1 = u1;
    out->v1 = v1;
}

// Polygon ID. Two translucent polygons with the *same* ID will not blend
// against each other -- the hardware refuses -- so a cell whose parts overlap
// would not layer at all if they all shared one. Cycling by part index is the
// same trick NEA_ParticleEmitterDraw uses.
static inline uint32_t ne_cell_poly_id(const NEA_CellAnim *anim,
                                       const NEA_CellPartXform *x, int index)
{
    return (anim->base_poly_id + x->part->poly_id_off + index) & 0x3F;
}

static inline uint32_t ne_cell_alpha(const NEA_CellAnim *anim,
                                     const NEA_CellPartXform *x)
{
    // The instance alpha scales the part's, so fading a whole character out
    // keeps whatever relative transparency it was authored with.
    uint32_t a = ((uint32_t)x->alpha * (anim->base_alpha + 1)) >> 5;
    return a > 31 ? 31 : a;
}

static inline uint32_t ne_cell_color(const NEA_CellAnim *anim,
                                     const NEA_CellPartXform *x)
{
    if (anim->base_color == 0x7FFF)
        return x->color;
    if (x->color == 0x7FFF)
        return anim->base_color;

    uint32_t r = ((x->color & 0x1F) * (anim->base_color & 0x1F)) >> 5;
    uint32_t g = (((x->color >> 5) & 0x1F) * ((anim->base_color >> 5) & 0x1F)) >> 5;
    uint32_t b = (((x->color >> 10) & 0x1F) * ((anim->base_color >> 10) & 0x1F)) >> 5;
    return r | (g << 5) | (b << 10);
}

//-----------------------------------------------------------------------------
// Backend: textured quads in screen space
//-----------------------------------------------------------------------------

// One part, as four transformed corners.
//
// The general path exists because a rotated or scaled part is not an
// axis-aligned rectangle any more, so NEA_2DDrawTexturedQuadColorCanvas()
// cannot draw it. When the transform *is* axis-aligned and unit-scaled -- by
// far the common case, and the only case an imported NANR without SRT
// produces -- the corners come out identical to what that function would have
// emitted, at the cost of three extra adds.
ARM_CODE static void ne_cell_draw_part_2d(const NEA_CellAnim *anim,
                                          const NEA_CellPartXform *x,
                                          int index, int ox, int oy)
{
    ne_cell_uv_t uv;
    ne_cell_uv(x, &uv);

    int32_t w = (int32_t)x->part->src_w << 12;
    int32_t h = (int32_t)x->part->src_h << 12;

    // corner_k = M * (u, v) + t, for the four corners of the part.
    int32_t bx = x->tx + (ox << 12);
    int32_t by = x->ty + (oy << 12);

    int32_t dxu = mulf32(x->m[0], w);
    int32_t dyu = mulf32(x->m[2], w);
    int32_t dxv = mulf32(x->m[1], h);
    int32_t dyv = mulf32(x->m[3], h);

    int32_t x0 = bx >> 12,               y0 = by >> 12;
    int32_t x1 = (bx + dxu) >> 12,       y1 = (by + dyu) >> 12;
    int32_t x2 = (bx + dxu + dxv) >> 12, y2 = (by + dyu + dyv) >> 12;
    int32_t x3 = (bx + dxv) >> 12,       y3 = (by + dyv) >> 12;

    // Priority is the Z. Lower is nearer, and NEA_2DViewInit's ortho matrix
    // runs 1 to -1, so a low priority number has to come out in front.
    int32_t z = x->priority;

    GFX_POLY_FORMAT = POLY_ALPHA(ne_cell_alpha(anim, x))
                    | POLY_ID(ne_cell_poly_id(anim, x, index))
                    | NEA_CULL_NONE;
    GFX_COLOR = ne_cell_color(anim, x);

    GFX_BEGIN = GL_QUADS;

    GFX_TEX_COORD = TEXTURE_PACK(uv.u0, uv.v0);
    GFX_VERTEX16 = ((y0 & 0xFFFF) << 16) | (x0 & 0xFFFF);
    GFX_VERTEX16 = z;

    GFX_TEX_COORD = TEXTURE_PACK(uv.u1, uv.v0);
    GFX_VERTEX_XY = ((y1 & 0xFFFF) << 16) | (x1 & 0xFFFF);

    GFX_TEX_COORD = TEXTURE_PACK(uv.u1, uv.v1);
    GFX_VERTEX_XY = ((y2 & 0xFFFF) << 16) | (x2 & 0xFFFF);

    GFX_TEX_COORD = TEXTURE_PACK(uv.u0, uv.v1);
    GFX_VERTEX_XY = ((y3 & 0xFFFF) << 16) | (x3 & 0xFFFF);
}

// Draw one instance's own pose. Children are handled by the caller, so this
// never recurses.
static void ne_cell_draw_pose_2d(NEA_CellAnim *anim, int x, int y)
{
    const NEA_CellData *data = anim->data;
    const NEA_Material *bound = NULL;

    for (int i = 0; i < anim->pose_count; i++)
    {
        const NEA_CellPartXform *pose = &anim->pose[i];
        if (!pose->visible || (pose->part->flags & NEA_CELL_PART_NO_3D))
            continue;

        NEA_Material *mat = data->atlas_mat[pose->src->atlas];
        // A RAM-backed atlas that has been evicted from VRAM with
        // NEA_MaterialTexVramUnload() has no image to sample. Drawing it
        // anyway would put untextured blocks of flat colour on screen, which
        // reads as a rendering bug rather than as an absent texture.
        if (mat == NULL || mat->texindex == NEA_NO_TEXTURE)
            continue;

        // Bind only on a change, and only the image and palette. A full
        // NEA_MaterialUse() would rewrite the lighting registers for every
        // part of every cell, which is what NEA_2DDrawTexturedQuad* does and
        // what a twenty-part character cannot afford sixty times a second.
        if (mat != bound)
        {
            NEA_MaterialTexUse(mat);
            if (mat->palette)
                NEA_PaletteUse(mat->palette);
            bound = mat;
        }

        ne_cell_draw_part_2d(anim, pose, i, x, y);
    }
}

void NEA_CellAnimDraw2D(NEA_CellAnim *anim, int x, int y)
{
    NEA_AssertPointer(anim, "NULL animation");
    if (anim->data == NULL)
        return;

    if (anim->num_children > 0)
    {
        // Multi-cell: a node's offset and the MULTI frame's own translation
        // are applied here rather than baked into the pose, so a node's pose
        // stays identical to the one its sequence produces on its own.
        for (int i = 0; i < anim->num_children; i++)
        {
            NEA_CellAnim *child = anim->children[i];
            if (child == NULL)
                continue;
            ne_cell_draw_pose_2d(child,
                                 x + child->node_x + anim->multi_px,
                                 y + child->node_y + anim->multi_py);
        }
        return;
    }

    ne_cell_draw_pose_2d(anim, x, y);
}

//-----------------------------------------------------------------------------
// Backend: camera-facing billboards in world space
//-----------------------------------------------------------------------------

// Where the cell's origin sits relative to the position it is drawn at.
//
// NEACell.c owns this, because the same point is also the pivot the instance
// transform turns about, and a billboard that stands on one point but spins
// about another is a bug nobody would think to look for. BOTTOM is the one
// that matters here: it puts the bottom centre of the cell's bounding box at
// the world position, so a character's feet meet the ground plane instead of
// its middle floating through it.
extern void __NEA_CellAnchorPoint(const NEA_CellAnim *anim, int cell_index,
                                  int32_t *ax, int32_t *ay);

// One part, as four world-space corners on the camera-facing plane.
//
// The part's own transform is applied in cell space first, so a rotated part
// rotates within the billboard plane -- which is what an animator drew -- and
// only then is the result laid onto the camera basis.
ARM_CODE static void ne_cell_draw_part_billboard(
    const NEA_CellAnim *anim, const NEA_CellPartXform *x, int index,
    const int32_t pos[3], const int32_t right[3], const int32_t up[3],
    int32_t ax, int32_t ay)
{
    ne_cell_uv_t uv;
    ne_cell_uv(x, &uv);

    int32_t w = (int32_t)x->part->src_w << 12;
    int32_t h = (int32_t)x->part->src_h << 12;

    int32_t bx = x->tx - ax;
    int32_t by = x->ty - ay;

    int32_t dxu = mulf32(x->m[0], w);
    int32_t dyu = mulf32(x->m[2], w);
    int32_t dxv = mulf32(x->m[1], h);
    int32_t dyv = mulf32(x->m[3], h);

    const int32_t cx[4] = { bx, bx + dxu, bx + dxu + dxv, bx + dxv };
    const int32_t cy[4] = { by, by + dyu, by + dyu + dyv, by + dyv };

    // Every part of a cell is exactly coplanar, so the depth buffer would
    // fight. Nudge each priority level a little toward the camera; four levels
    // is what the hardware OBJ backend gets from its priority field, so the two
    // agree on ordering.
    int32_t bias = mulf32(inttof32(3 - x->priority),
                          floattof32(1.0f / 512.0f));

    int32_t upp = anim->units_per_pixel;

    GFX_POLY_FORMAT = POLY_ALPHA(ne_cell_alpha(anim, x))
                    | POLY_ID(ne_cell_poly_id(anim, x, index))
                    | NEA_CULL_NONE;
    GFX_COLOR = ne_cell_color(anim, x);

    // The bias runs along the plane normal, right x up, computed once.
    int32_t nx = mulf32(mulf32(right[1], up[2]) - mulf32(right[2], up[1]), bias);
    int32_t ny = mulf32(mulf32(right[2], up[0]) - mulf32(right[0], up[2]), bias);
    int32_t nz = mulf32(mulf32(right[0], up[1]) - mulf32(right[1], up[0]), bias);

    static const uint8_t uv_order[4][2] = { { 0, 0 }, { 1, 0 }, { 1, 1 },
                                            { 0, 1 } };

    NEA_PolyBegin(GL_QUAD);

    for (int k = 0; k < 4; k++)
    {
        int32_t su = mulf32(cx[k], upp);
        int32_t sv = mulf32(cy[k], upp);

        // Cell space is Y down and world space is Y up, so the up vector is
        // subtracted, not added.
        int32_t px = pos[0] + mulf32(right[0], su) - mulf32(up[0], sv) + nx;
        int32_t py = pos[1] + mulf32(right[1], su) - mulf32(up[1], sv) + ny;
        int32_t pz = pos[2] + mulf32(right[2], su) - mulf32(up[2], sv) + nz;

        GFX_TEX_COORD = TEXTURE_PACK(uv_order[k][0] ? uv.u1 : uv.u0,
                                     uv_order[k][1] ? uv.v1 : uv.v0);
        NEA_PolyVertexI(px, py, pz);
    }
}

static void ne_cell_draw_pose_billboard(NEA_CellAnim *anim,
                                        const int32_t pos[3],
                                        const int32_t right[3],
                                        const int32_t up[3])
{
    const NEA_CellData *data = anim->data;
    const NEA_Material *bound = NULL;

    // Anchor against the cell the pose actually came from, not the bank's
    // first, so a sequence whose cells differ in height does not bob.
    int32_t ax = 0, ay = 0;
    __NEA_CellAnchorPoint(anim, anim->pose_cell, &ax, &ay);

    for (int i = 0; i < anim->pose_count; i++)
    {
        const NEA_CellPartXform *pose = &anim->pose[i];
        if (!pose->visible || (pose->part->flags & NEA_CELL_PART_NO_3D))
            continue;

        NEA_Material *mat = data->atlas_mat[pose->src->atlas];
        if (mat == NULL || mat->texindex == NEA_NO_TEXTURE)
            continue;

        if (mat != bound)
        {
            NEA_MaterialTexUse(mat);
            if (mat->palette)
                NEA_PaletteUse(mat->palette);
            bound = mat;
        }

        ne_cell_draw_part_billboard(anim, pose, i, pos, right, up, ax, ay);
    }
}

void NEA_CellAnimDrawBillboardI(NEA_CellAnim *anim, int32_t x, int32_t y,
                                int32_t z)
{
    NEA_AssertPointer(anim, "NULL animation");
    if (anim->data == NULL)
        return;

    int32_t right[3], up[3];
    if (!NEA_CameraBillboardBasis(__ne_cell_camera, right, up))
    {
        // No camera: fall back to the world XY plane, which is at least
        // deterministic, and say so once rather than drawing nothing.
        NEA_DebugPrint("NEACell: no camera set, billboard is axis-aligned");
        right[0] = floattof32(1.0f); right[1] = 0; right[2] = 0;
        up[0] = 0; up[1] = floattof32(1.0f); up[2] = 0;
    }

    if (anim->num_children > 0)
    {
        for (int i = 0; i < anim->num_children; i++)
        {
            NEA_CellAnim *child = anim->children[i];
            if (child == NULL)
                continue;

            // A node's offset is in cell pixels, so it moves along the same
            // billboard plane the parts do.
            int32_t nx = (int32_t)(child->node_x + anim->multi_px) << 12;
            int32_t ny = (int32_t)(child->node_y + anim->multi_py) << 12;
            int32_t su = mulf32(nx, anim->units_per_pixel);
            int32_t sv = mulf32(ny, anim->units_per_pixel);

            int32_t pos[3] = {
                x + mulf32(right[0], su) - mulf32(up[0], sv),
                y + mulf32(right[1], su) - mulf32(up[1], sv),
                z + mulf32(right[2], su) - mulf32(up[2], sv),
            };
            ne_cell_draw_pose_billboard(child, pos, right, up);
        }
        return;
    }

    int32_t pos[3] = { x, y, z };
    ne_cell_draw_pose_billboard(anim, pos, right, up);
}
