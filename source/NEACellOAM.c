// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Warioware64
//
// This file is part of Nitro Engine Advanced
//
// The hardware OBJ backend of the cell system.
//
// A separate translation unit on purpose: NEAHw2D is three thousand lines and
// claims VRAM banks, so a ROM that only draws cells as 3D quads should never
// link it. Nothing in NEACell.c calls into here except through a weak
// reference, so the linker only pulls this object in when the application
// itself calls NEA_CellAnimBindOAM().

#include <stdlib.h>
#include <string.h>

#include "NEAMain.h"

//-----------------------------------------------------------------------------
// Per-instance binding state
//-----------------------------------------------------------------------------

typedef struct {
    NEA_Hw2DOBJ *obj;
    uint32_t gfx_offset;  // what is currently resident, NEA_CELL_NO_GFX = none
    uint8_t size_class;
    uint8_t color_mode;   // a 4bpp part in a 256-colour OBJ renders garbage
    bool in_use;
} ne_cell_slot_t;

typedef struct {
    NEA_Hw2DEngine engine;
    NEA_CellOAMPolicy policy;

    const uint8_t *gfx;   // the .ncgfx blob, resident in main RAM
    size_t gfx_size;

    ne_cell_slot_t *slots;
    int num_slots;

    uint32_t affine_used;  // bitmask of the 32 hardware matrices we took
    int affine_taken;

    bool warned_slots;
    bool warned_affine;
} ne_cell_oam_t;

//-----------------------------------------------------------------------------
// Binding
//-----------------------------------------------------------------------------

static int ne_cell_obj_bytes(NEA_OBJSize size, NEA_OBJColorMode color)
{
    static const uint8_t dims[12][2] = {
        { 8, 8 }, { 16, 16 }, { 32, 32 }, { 64, 64 },
        { 16, 8 }, { 32, 8 }, { 32, 16 }, { 64, 32 },
        { 8, 16 }, { 8, 32 }, { 16, 32 }, { 32, 64 },
    };
    if ((int)size < 0 || (int)size >= 12)
        return 0;
    int bpp = (color == NEA_OBJ_COLOR_256) ? 8 : 4;
    return (dims[size][0] * dims[size][1] * bpp) / 8;
}

int NEA_CellAnimBindOAM(NEA_CellAnim *anim, NEA_Hw2DEngine engine,
                        const void *ncgfx, size_t size,
                        NEA_CellOAMPolicy policy)
{
    NEA_AssertPointer(anim, "NULL animation");
    NEA_AssertPointer(ncgfx, "NULL graphics blob");

    if (anim->data == NULL)
    {
        NEA_DebugPrint("NEACell: bind OAM before setting data");
        return -1;
    }

    const NEA_CellBudget *budget = anim->data->budget;
    if (budget == NULL)
    {
        NEA_DebugPrint("NEACell: the file carries no OAM budget");
        return -1;
    }

    if (anim->oam)
        NEA_CellAnimUnbindOAM(anim);

    // Every part the file says can be on screen at once, allocated once. The
    // pool is never grown or shrunk afterwards, because NEAHw2D hands out OAM
    // indices monotonically and never recycles them -- creating and deleting
    // sprites per frame runs the engine out of entries within seconds.
    int total = 0;
    for (int i = 0; i < NEA_CELL_OBJ_SIZES; i++)
        total += budget->max_objs[i];

    if (total == 0)
    {
        NEA_DebugPrint("NEACell: nothing in this bank can be drawn as an OBJ");
        return policy == NEA_CELL_OAM_FAIL ? -1 : 0;
    }
    if (total > 128)
    {
        NEA_DebugPrint("NEACell: bank needs %d OBJs, one engine has 128",
                       total);
        if (policy == NEA_CELL_OAM_FAIL)
            return -1;
    }

    ne_cell_oam_t *oam = calloc(1, sizeof(ne_cell_oam_t));
    if (oam == NULL)
        return -1;

    // Room for both colour modes of every class, since a bank that mixes them
    // needs a slot pool for each.
    oam->slots = calloc(total * 2, sizeof(ne_cell_slot_t));
    if (oam->slots == NULL)
    {
        free(oam);
        return -1;
    }

    oam->engine = engine;
    oam->policy = policy;
    oam->gfx = ncgfx;
    oam->gfx_size = size;

    // Which colour modes each size class is actually used with. A slot's
    // colour mode is baked into the OAM entry, so a 4bpp part cannot borrow a
    // 256-colour slot -- the hardware would read two pixels' worth of indices
    // as one. Nearly every bank uses a single mode, in which case this is
    // exactly the budget; one that mixes pays for both.
    uint8_t modes[NEA_CELL_OBJ_SIZES];
    memset(modes, 0, sizeof(modes));
    for (int p = 0; p < anim->data->num_parts; p++)
    {
        const NEA_CellPart *part = &anim->data->parts[p];
        if (part->obj_size >= NEA_CELL_OBJ_SIZES)
            continue;
        modes[part->obj_size] |= 1u << (part->obj_color ? 1 : 0);
    }

    // A parent seated on a multi-cell has no pose of its own -- the children
    // hold them -- so it needs no sprites. OAM entries are a resource there
    // are 128 of, and this is a quarter of them on a four-node character.
    int wanted = (anim->num_children > 0) ? 0 : total;

    int at = 0;
    for (int cls = 0; cls < NEA_CELL_OBJ_SIZES && at < 128 && wanted; cls++)
    {
        for (int mode = 0; mode < 2 && at < 128; mode++)
        {
            if (!(modes[cls] & (1u << mode)))
                continue;

            for (int n = 0; n < budget->max_objs[cls] && at < 128; n++)
            {
                NEA_Hw2DOBJ *obj = NEA_Hw2DOBJCreate(
                    engine, (NEA_OBJSize)cls,
                    mode ? NEA_OBJ_COLOR_256 : NEA_OBJ_COLOR_16);
                if (obj == NULL)
                {
                    NEA_DebugPrint("NEACell: out of OBJ slots after %d", at);
                    break;
                }
                NEA_Hw2DOBJSetVisible(obj, false);
                oam->slots[at].obj = obj;
                oam->slots[at].size_class = (uint8_t)cls;
                oam->slots[at].color_mode = (uint8_t)mode;
                oam->slots[at].gfx_offset = NEA_CELL_NO_GFX;
                at++;
            }
        }
    }

    oam->num_slots = at;
    anim->oam = oam;

    if (at == 0 && policy == NEA_CELL_OAM_FAIL)
    {
        NEA_CellAnimUnbindOAM(anim);
        return -1;
    }

    // Multi-cell nodes share the parent's blob but keep their own pools, so
    // that each node's parts get slots of their own.
    for (int i = 0; i < anim->num_children; i++)
    {
        if (anim->children[i] && anim->children[i]->oam == NULL)
            NEA_CellAnimBindOAM(anim->children[i], engine, ncgfx, size, policy);
    }

    return 0;
}

// Bind a child that was seated after its parent, using the parent's blob and
// engine. Called from ne_cell_seat_multicell() through a weak reference, so
// this file is still only linked when the application binds OAM itself.
void __NEA_CellBindChildOAM(NEA_CellAnim *parent, NEA_CellAnim *child)
{
    ne_cell_oam_t *oam = parent->oam;
    if (oam == NULL || child == NULL || child->oam != NULL)
        return;

    NEA_CellAnimBindOAM(child, oam->engine, oam->gfx, oam->gfx_size,
                        oam->policy);
}

int NEA_CellAnimLoadOAMPalette(NEA_CellAnim *anim, const void *ncpal,
                               int num_colors, int slot)
{
    NEA_AssertPointer(anim, "NULL animation");
    NEA_AssertPointer(ncpal, "NULL palette");

    ne_cell_oam_t *oam = anim->oam;
    if (oam == NULL)
    {
        NEA_DebugPrint("NEACell: load the palette after binding OAM");
        return -1;
    }

    return NEA_Hw2DOBJLoadPalette(oam->engine, ncpal, num_colors, slot);
}

void NEA_CellAnimUnbindOAM(NEA_CellAnim *anim)
{
    if (anim == NULL || anim->oam == NULL)
        return;

    ne_cell_oam_t *oam = anim->oam;
    for (int i = 0; i < oam->num_slots; i++)
    {
        if (oam->slots[i].obj)
            NEA_Hw2DOBJDelete(oam->slots[i].obj);
    }
    free(oam->slots);
    free(oam);
    anim->oam = NULL;

    for (int i = 0; i < anim->num_children; i++)
    {
        if (anim->children[i])
            NEA_CellAnimUnbindOAM(anim->children[i]);
    }
}

//-----------------------------------------------------------------------------
// Per-frame sync
//-----------------------------------------------------------------------------

// Invert a 2x2 f32 matrix into the 1.7.8 form the hardware stores.
//
// The affine entry maps screen pixels back to texture pixels, so it is the
// inverse of the transform the artist sees. Returns false when the matrix is
// singular -- a part scaled to nothing -- in which case the caller draws it
// unrotated rather than dividing by zero.
static bool ne_cell_affine_from(const int32_t m[4], int out[4])
{
    // det in f32
    int32_t det = mulf32(m[0], m[3]) - mulf32(m[1], m[2]);
    if (det == 0)
        return false;

    // inverse = adj / det, then f32 (4096 = 1.0) -> 1.7.8 (256 = 1.0)
    int32_t i0 = divf32(m[3], det);
    int32_t i1 = divf32(-m[1], det);
    int32_t i2 = divf32(-m[2], det);
    int32_t i3 = divf32(m[0], det);

    out[0] = i0 >> 4;
    out[1] = i2 >> 4;
    out[2] = i1 >> 4;
    out[3] = i3 >> 4;
    return true;
}

static int ne_cell_take_affine(ne_cell_oam_t *oam)
{
    for (int i = 0; i < 32; i++)
    {
        if (!(oam->affine_used & (1u << i)))
        {
            oam->affine_used |= 1u << i;
            oam->affine_taken++;
            return i;
        }
    }
    return -1;
}

static void ne_cell_apply_pose(NEA_CellAnim *anim, int ox, int oy)
{
    ne_cell_oam_t *oam = anim->oam;
    if (oam == NULL)
        return;

    const NEA_CellData *data = anim->data;

    // Affine matrices are handed out afresh every frame: which parts need one
    // changes as an animation plays, and 32 is few enough that hanging on to
    // an unused one is worse than re-taking it.
    oam->affine_used = 0;
    oam->affine_taken = 0;

    for (int i = 0; i < oam->num_slots; i++)
        oam->slots[i].in_use = false;

    for (int i = 0; i < anim->pose_count; i++)
    {
        const NEA_CellPartXform *pose = &anim->pose[i];
        const NEA_CellPart *part = pose->part;
        const NEA_CellPart *src = pose->src;

        if (!pose->visible)
            continue;
        if ((part->flags & NEA_CELL_PART_NO_OAM)
            || src->obj_size == NEA_CELL_OBJ_SIZE_NONE
            || src->gfx_offset == NEA_CELL_NO_GFX)
        {
            // Skipping is the default because a partly drawn cell is almost
            // always more useful than nothing at all; the FAIL policy already
            // refused to bind, so there is nothing to decide here.
            continue;
        }

        // Find a free slot of this size class.
        ne_cell_slot_t *slot = NULL;
        for (int s = 0; s < oam->num_slots; s++)
        {
            if (!oam->slots[s].in_use
                && oam->slots[s].size_class == src->obj_size
                && oam->slots[s].color_mode == (src->obj_color ? 1 : 0))
            {
                slot = &oam->slots[s];
                break;
            }
        }
        if (slot == NULL)
        {
            if (!oam->warned_slots)
            {
                NEA_DebugPrint("NEACell: out of OBJ slots for size class %d; "
                               "the file's budget is too small",
                               src->obj_size);
                oam->warned_slots = true;
            }
            continue;
        }
        slot->in_use = true;

        // The VRAM transfer. Only when the graphics actually changed: a part
        // that holds still across frames costs one comparison, which is what
        // makes streaming affordable at all.
        if (slot->gfx_offset != src->gfx_offset)
        {
            int bytes = ne_cell_obj_bytes((NEA_OBJSize)src->obj_size,
                                          (NEA_OBJColorMode)src->obj_color);
            if (bytes > 0
                && (size_t)src->gfx_offset + bytes <= oam->gfx_size)
            {
                NEA_Hw2DOBJLoadGfx(slot->obj, oam->gfx + src->gfx_offset,
                                   bytes);
                slot->gfx_offset = src->gfx_offset;
            }
            else
            {
                NEA_DebugPrint("NEACell: part graphics run past the blob");
                continue;
            }
        }

        // The pose gives a matrix and the position of the part's top-left
        // corner. A plain OBJ is placed by its corner, so only a transformed
        // part needs the pivot dance.
        int px = ox + (pose->tx >> 12);
        int py = oy + (pose->ty >> 12);

        bool axis_aligned = (pose->m[1] == 0 && pose->m[2] == 0
                             && pose->m[0] == (1 << 12)
                             && pose->m[3] == (1 << 12));

        if (axis_aligned)
        {
            NEA_Hw2DOBJSetAffine(slot->obj, -1, false);
            NEA_Hw2DOBJSetFlip(slot->obj,
                               (part->flags & NEA_CELL_PART_HFLIP) != 0,
                               (part->flags & NEA_CELL_PART_VFLIP) != 0);
        }
        else
        {
            int matrix[4];
            int index = ne_cell_affine_from(pose->m, matrix)
                      ? ne_cell_take_affine(oam) : -1;

            if (index < 0)
            {
                // Out of matrices, or a degenerate transform. Falling back to
                // flip-only keeps the part on screen in roughly the right
                // place, which beats dropping it.
                if (!oam->warned_affine)
                {
                    NEA_DebugPrint("NEACell: out of affine matrices; the rest "
                                   "of this cell is flip-only");
                    oam->warned_affine = true;
                }
                NEA_Hw2DOBJSetAffine(slot->obj, -1, false);
                NEA_Hw2DOBJSetFlip(slot->obj,
                                   (part->flags & NEA_CELL_PART_HFLIP) != 0,
                                   (part->flags & NEA_CELL_PART_VFLIP) != 0);
            }
            else
            {
                NEA_Hw2DOBJSetAffineMatrix(oam->engine, index, matrix[0],
                                           matrix[1], matrix[2], matrix[3]);
                bool dbl = (part->flags & NEA_CELL_PART_DOUBLE_SIZE) != 0;
                NEA_Hw2DOBJSetAffine(slot->obj, index, dbl);
                // The hardware rotates about the sprite's own centre, so the
                // corner the pose gives has to be pulled back by half the
                // sprite -- and by half again when double size doubles the
                // area the sprite is drawn into.
                px -= (dbl ? src->src_w : 0) / 2;
                py -= (dbl ? src->src_h : 0) / 2;
                NEA_Hw2DOBJSetFlip(slot->obj, false, false);
            }
        }

        NEA_Hw2DOBJSetPos(slot->obj, px, py);
        NEA_Hw2DOBJSetPriority(slot->obj, pose->priority);
        NEA_Hw2DOBJSetPaletteSlot(slot->obj, src->pal_slot);
        NEA_Hw2DOBJSetVisible(slot->obj, true);

        (void)data;
    }

    // Anything the pose did not claim this frame is hidden rather than
    // deleted: the pool is fixed for the life of the binding.
    for (int i = 0; i < oam->num_slots; i++)
    {
        if (!oam->slots[i].in_use && oam->slots[i].obj)
            NEA_Hw2DOBJSetVisible(oam->slots[i].obj, false);
    }
}

void NEA_CellAnimApplyOAM(NEA_CellAnim *anim, int x, int y)
{
    NEA_AssertPointer(anim, "NULL animation");
    if (anim->data == NULL)
        return;

    if (anim->num_children > 0)
    {
        for (int i = 0; i < anim->num_children; i++)
        {
            NEA_CellAnim *child = anim->children[i];
            if (child == NULL)
                continue;
            ne_cell_apply_pose(child,
                               x + child->node_x + anim->multi_px,
                               y + child->node_y + anim->multi_py);
        }
        return;
    }

    ne_cell_apply_pose(anim, x, y);
}
