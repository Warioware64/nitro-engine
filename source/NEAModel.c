// SPDX-License-Identifier: MIT
//
// Copyright (c) 2008-2022 Antonio Niño Díaz
//
// This file is part of Nitro Engine Advanced

#include <nds/arm9/postest.h>

#include "dsma/dsma.h"

#include "NEAMain.h"

/// @file NEAModel.c

typedef struct {
    void *address;
    int uses; // Number of models that use this mesh
    bool has_to_free;
} ne_mesh_info_t;

static ne_mesh_info_t *NEA_Mesh = NULL;
static NEA_Model **NEA_ModelPointers;
static int NEA_MAX_MODELS;
static bool ne_model_system_inited = false;

static void ne_mesh_delete(int mesh_index)
{
    int slot = mesh_index;

    // A mesh may be used by several models
    NEA_Mesh[slot].uses--;

    // If the number of users is zero, delete it.
    if (NEA_Mesh[slot].uses == 0)
    {
        if (NEA_Mesh[slot].has_to_free)
            free(NEA_Mesh[slot].address);

        NEA_Mesh[slot].address = NULL;
    }
}

static int ne_model_get_free_mesh_slot(void)
{
    // Get free slot
    for (int i = 0; i < NEA_MAX_MODELS; i++)
    {
        if (NEA_Mesh[i].address == NULL)
            return i;
    }

    NEA_DebugPrint("No free slots");
    return NEA_NO_MESH;
}

static int ne_model_load_ram_common(NEA_Model *model, const void *pointer)
{
    NEA_AssertPointer(model, "NULL model pointer");
    NEA_AssertPointer(pointer, "NULL data pointer");

    // Check if a mesh exists
    if (model->meshindex != NEA_NO_MESH)
        ne_mesh_delete(model->meshindex);

    int slot = ne_model_get_free_mesh_slot();
    if (slot == NEA_NO_MESH)
        return 0;

    model->meshindex = slot;

    ne_mesh_info_t *mesh = &NEA_Mesh[slot];

    mesh->address = (void *)pointer;
    mesh->has_to_free = false;
    mesh->uses = 1;

    return 1;
}

static int ne_model_load_filesystem_common(NEA_Model *model, const char *path)
{
    NEA_AssertPointer(model, "NULL model pointer");
    NEA_AssertPointer(path, "NULL path pointer");

    // Check if a mesh exists
    if (model->meshindex != NEA_NO_MESH)
        ne_mesh_delete(model->meshindex);

    int slot = ne_model_get_free_mesh_slot();
    if (slot == NEA_NO_MESH)
        return 0;

    void *pointer = NEA_FATLoadData(path);
    if (pointer == NULL)
        return 0;

    model->meshindex = slot;

    ne_mesh_info_t *mesh = &NEA_Mesh[slot];

    mesh->address = pointer;
    mesh->has_to_free = true;
    mesh->uses = 1;

    return 1;
}

//--------------------------------------------------------------------------

NEA_Model *NEA_ModelCreate(NEA_ModelType type)
{
    if (!ne_model_system_inited)
    {
        NEA_DebugPrint("System not initialized");
        return NULL;
    }

    NEA_Model *model = calloc(1, sizeof(NEA_Model));
    if (model == NULL)
    {
        NEA_DebugPrint("Not enough memory");
        return NULL;
    }

    int i = 0;
    while (1)
    {
        if (NEA_ModelPointers[i] == NULL)
        {
            NEA_ModelPointers[i] = model;
            break;
        }
        i++;
        if (i == NEA_MAX_MODELS)
        {
            NEA_DebugPrint("No free slots");
            free(model);
            return NULL;
        }
    }

    model->sx = model->sy = model->sz = inttof32(1);

    model->mat = NULL;

    model->modeltype = type;
    model->meshindex = NEA_NO_MESH;

    if (type == NEA_Animated)
    {
        for (int i = 0; i < 2; i++)
        {
            model->animinfo[i] = calloc(sizeof(NEA_AnimInfo), 1);
            NEA_AssertPointer(model->animinfo[i],
                             "Couldn't allocate animation info");
        }
    }

    return model;
}

void NEA_ModelDelete(NEA_Model *model)
{
    if (!ne_model_system_inited)
        return;

    NEA_AssertPointer(model, "NULL pointer");

    int i = 0;
    while (1)
    {
        if (i == NEA_MAX_MODELS)
        {
            NEA_DebugPrint("Model not found");
            return;
        }
        if (NEA_ModelPointers[i] == model)
        {
            NEA_ModelPointers[i] = NULL;
            break;
        }
        i++;
    }

    if (model->modeltype == NEA_Animated)
    {
        for (int i = 0; i < 2; i++)
            free(model->animinfo[i]);
    }

    if (model->mat != NULL)
        free(model->mat);

    // If there is an asigned mesh
    if (model->meshindex != NEA_NO_MESH)
        ne_mesh_delete(model->meshindex);

    free(model);
}

int NEA_ModelLoadStaticMeshFAT(NEA_Model *model, const char *path)
{
    if (!ne_model_system_inited)
        return 0;

    NEA_Assert(model->modeltype == NEA_Static, "Not a static model");

    return ne_model_load_filesystem_common(model, path);
}

int NEA_ModelLoadStaticMesh(NEA_Model *model, const void *pointer)
{
    if (!ne_model_system_inited)
        return 0;

    NEA_Assert(model->modeltype == NEA_Static, "Not a static model");

    return ne_model_load_ram_common(model, pointer);
}

void NEA_ModelFreeMeshWhenDeleted(NEA_Model *model)
{
    NEA_AssertPointer(model, "NULL model pointer");
    if (model->meshindex != NEA_NO_MESH)
    {
        ne_mesh_info_t *mesh = &NEA_Mesh[model->meshindex];
        mesh->has_to_free = true;
    }
}

void NEA_ModelSetMaterial(NEA_Model *model, NEA_Material *material)
{
    NEA_AssertPointer(model, "NULL model pointer");
    NEA_AssertPointer(material, "NULL material pointer");
    model->texture = material;
}

void NEA_ModelSetAnimation(NEA_Model *model, NEA_Animation *anim)
{
    NEA_AssertPointer(model, "NULL model pointer");
    NEA_AssertPointer(anim, "NULL animation pointer");
    NEA_Assert(model->modeltype == NEA_Animated, "Not an animated model");
    model->animinfo[0]->animation = anim;
    uint32_t frames = DSMA_GetNumFrames(anim->data);
    model->animinfo[0]->numframes = frames;
}

void NEA_ModelSetAnimationSecondary(NEA_Model *model, NEA_Animation *anim)
{
    NEA_AssertPointer(model, "NULL model pointer");
    NEA_AssertPointer(anim, "NULL animation pointer");
    NEA_Assert(model->modeltype == NEA_Animated, "Not an animated model");
    model->animinfo[1]->animation = anim;
    uint32_t frames = DSMA_GetNumFrames(anim->data);
    model->animinfo[1]->numframes = frames;
}

//---------------------------------------------------------

// Internal use... see below
extern bool NEA_TestTouch;

void NEA_ModelDraw(const NEA_Model *model)
{
    NEA_AssertPointer(model, "NULL pointer");
    if (model->meshindex == NEA_NO_MESH)
        return;

    if (model->modeltype == NEA_Animated)
    {
        // The base animation must always be present. The secondary animation
        // isn't required to draw the model.
        if (model->animinfo[0]->animation == NULL)
            return;
    }

    MATRIX_PUSH = 0;

    if (model->mat != NULL)
    {
        glMultMatrix4x3(model->mat);
    }
    else
    {
        MATRIX_TRANSLATE = model->x;
        MATRIX_TRANSLATE = model->y;
        MATRIX_TRANSLATE = model->z;

        if (model->rx != 0)
            glRotateXi(model->rx << 6);
        if (model->ry != 0)
            glRotateYi(model->ry << 6);
        if (model->rz != 0)
            glRotateZi(model->rz << 6);

        MATRIX_SCALE = model->sx;
        MATRIX_SCALE = model->sy;
        MATRIX_SCALE = model->sz;
    }

    if (NEA_TestTouch)
    {
        PosTest_Asynch(0, 0, 0);
    }
    else
    {
        // If the texture pointer is NULL, this will set GFX_TEX_FORMAT
        // to 0 and GFX_COLOR to white
        NEA_MaterialUse(model->texture);
    }

    ne_mesh_info_t *mesh = &NEA_Mesh[model->meshindex];
    const void *meshdata = mesh->address;

    if (model->modeltype == NEA_Static)
    {
        NEA_DisplayListDrawDefault(meshdata);
    }
    else // if(model->modeltype == NEA_Animated)
    {
        if (model->animinfo[0]->animation && model->animinfo[1]->animation)
        {
            int ret = DSMA_DrawModelBlendAnimation(meshdata,
                    model->animinfo[0]->animation->data,
                    model->animinfo[0]->currframe,
                    model->animinfo[1]->animation->data,
                    model->animinfo[1]->currframe,
                    model->anim_blend);
            NEA_Assert(ret == DSMA_SUCCESS, "Failed to draw animated model");
        }
        else // if (model->animinfo[0]->animation)
        {
            int ret = DSMA_DrawModel(meshdata,
                                     model->animinfo[0]->animation->data,
                                     model->animinfo[0]->currframe);
            NEA_Assert(ret == DSMA_SUCCESS, "Failed to draw animated model");
        }
    }

    MATRIX_POP = 1;
}

void NEA_ModelClone(NEA_Model *dest, NEA_Model *source)
{
    NEA_AssertPointer(dest, "NULL dest pointer");
    NEA_AssertPointer(source, "NULL source pointer");
    NEA_Assert(dest->modeltype == source->modeltype,
              "Different model types");

    if (dest->modeltype == NEA_Animated)
    {
        memcpy(dest->animinfo[0], source->animinfo[0], sizeof(NEA_AnimInfo));
        memcpy(dest->animinfo[1], source->animinfo[1], sizeof(NEA_AnimInfo));
        dest->anim_blend = source->anim_blend;
    }

    dest->x = source->x;
    dest->y = source->y;
    dest->z = source->z;
    dest->rx = source->rx;
    dest->ry = source->ry;
    dest->rz = source->rz;
    dest->sx = source->sx;
    dest->sy = source->sy;
    dest->sz = source->sz;

    dest->texture = source->texture;
    dest->meshindex = source->meshindex;

    // If the model has a mesh (which is the normal situation), increase the
    // count of users of that mesh.
    if (dest->meshindex != NEA_NO_MESH)
    {
        ne_mesh_info_t *mesh = &NEA_Mesh[dest->meshindex];
        mesh->uses++;
    }
}

void NEA_ModelScaleI(NEA_Model *model, int x, int y, int z)
{
    NEA_AssertPointer(model, "NULL pointer");
    model->sx = x;
    model->sy = y;
    model->sz = z;
}

void NEA_ModelTranslateI(NEA_Model *model, int x, int y, int z)
{
    NEA_AssertPointer(model, "NULL pointer");
    model->x += x;
    model->y += y;
    model->z += z;
}

void NEA_ModelSetCoordI(NEA_Model *model, int x, int y, int z)
{
    NEA_AssertPointer(model, "NULL pointer");
    model->x = x;
    model->y = y;
    model->z = z;
}

void NEA_ModelRotate(NEA_Model *model, int rx, int ry, int rz)
{
    NEA_AssertPointer(model, "NULL pointer");
    model->rx = (model->rx + rx + 512) & 0x1FF;
    model->ry = (model->ry + ry + 512) & 0x1FF;
    model->rz = (model->rz + rz + 512) & 0x1FF;
}

void NEA_ModelSetRot(NEA_Model *model, int rx, int ry, int rz)
{
    NEA_AssertPointer(model, "NULL pointer");
    model->rx = rx;
    model->ry = ry;
    model->rz = rz;
}

int NEA_ModelSetMatrix(NEA_Model *model, m4x3 *mat)
{
    NEA_AssertPointer(model, "NULL model pointer");
    NEA_AssertPointer(mat, "NULL matrix pointer");

    if (model->mat == NULL)
    {
        model->mat = malloc(sizeof(m4x3));
        if (model->mat == NULL)
            return 0;
    }

    memcpy(model->mat, mat, sizeof(m4x3));

    return 1;
}

void NEA_ModelClearMatrix(NEA_Model *model)
{
    NEA_AssertPointer(model, "NULL pointer");

    if (model->mat == NULL)
        return;

    free(model->mat);
}

void NEA_ModelAnimateAll(void)
{
    if (!ne_model_system_inited)
        return;

    for (int i = 0; i < NEA_MAX_MODELS; i++)
    {
        if (NEA_ModelPointers[i] == NULL)
            continue;

        if (NEA_ModelPointers[i]->modeltype != NEA_Animated)
            continue;

        for (int j = 0; j < 2; j++)
        {
            NEA_AnimInfo *animinfo = NEA_ModelPointers[i]->animinfo[j];

            animinfo->currframe += animinfo->speed;

            if (animinfo->type ==  NEA_ANIM_LOOP)
            {
                int32_t endval = inttof32(animinfo->numframes);
                if (animinfo->currframe >= endval)
                    animinfo->currframe -= endval;
                else if (animinfo->currframe < 0)
                    animinfo->currframe += endval;
            }
            else if (animinfo->type ==  NEA_ANIM_ONESHOT)
            {
                int32_t endval = inttof32(animinfo->numframes - 1);
                if (animinfo->currframe > endval)
                {
                    animinfo->currframe = endval;
                    animinfo->speed = 0;
                }
                else if (animinfo->currframe < 0)
                {
                    animinfo->currframe = 0;
                    animinfo->speed = 0;
                }
            }
        }
    }
}

void NEA_ModelAnimStart(NEA_Model *model, NEA_AnimationType type, int32_t speed)
{
    NEA_AssertPointer(model, "NULL pointer");
    NEA_Assert(model->modeltype == NEA_Animated, "Not an animated model");
    model->animinfo[0]->type = type;
    model->animinfo[0]->speed = speed;
    model->animinfo[0]->currframe = 0;
}

void NEA_ModelAnimSecondaryStart(NEA_Model *model, NEA_AnimationType type,
                                int32_t speed)
{
    NEA_AssertPointer(model, "NULL pointer");
    NEA_Assert(model->modeltype == NEA_Animated, "Not an animated model");
    model->animinfo[1]->type = type;
    model->animinfo[1]->speed = speed;
    model->animinfo[1]->currframe = 0;
    model->anim_blend = 0;
}

void NEA_ModelAnimSetSpeed(NEA_Model *model, int32_t speed)
{
    NEA_AssertPointer(model, "NULL pointer");
    NEA_Assert(model->modeltype == NEA_Animated, "Not an animated model");
    model->animinfo[0]->speed = speed;
}

void NEA_ModelAnimSecondarySetSpeed(NEA_Model *model, int32_t speed)
{
    NEA_AssertPointer(model, "NULL pointer");
    NEA_Assert(model->modeltype == NEA_Animated, "Not an animated model");
    model->animinfo[1]->speed = speed;
}

int32_t NEA_ModelAnimGetFrame(const NEA_Model *model)
{
    NEA_AssertPointer(model, "NULL pointer");
    NEA_Assert(model->modeltype == NEA_Animated, "Not an animated model");
    return model->animinfo[0]->currframe;
}

int32_t NEA_ModelAnimSecondaryGetFrame(const NEA_Model *model)
{
    NEA_AssertPointer(model, "NULL pointer");
    NEA_Assert(model->modeltype == NEA_Animated, "Not an animated model");
    return model->animinfo[1]->currframe;
}

void NEA_ModelAnimSetFrame(NEA_Model *model, int32_t frame)
{
    NEA_AssertPointer(model, "NULL pointer");
    NEA_Assert(model->modeltype == NEA_Animated, "Not an animated model");
    // TODO: Check if it is off bounds
    model->animinfo[0]->currframe = frame;
}

void NEA_ModelAnimSecondarySetFrame(NEA_Model *model, int32_t frame)
{
    NEA_AssertPointer(model, "NULL pointer");
    NEA_Assert(model->modeltype == NEA_Animated, "Not an animated model");
    // TODO: Check if it is off bounds
    model->animinfo[1]->currframe = frame;
}

void NEA_ModelAnimSecondarySetFactor(NEA_Model *model, int32_t factor)
{
    NEA_AssertPointer(model, "NULL pointer");
    NEA_Assert(model->modeltype == NEA_Animated, "Not an animated model");
    if (factor < 0)
        factor = 0;
    if (factor > inttof32(1))
        factor = inttof32(1);
    model->anim_blend = factor;
}

void NEA_ModelAnimSecondaryClear(NEA_Model *model, bool replace_base_anim)
{
    NEA_AssertPointer(model, "NULL pointer");
    NEA_Assert(model->modeltype == NEA_Animated, "Not an animated model");

    // Return if there is no animation to remove
    if (model->animinfo[1]->animation == NULL)
        return;

    if (replace_base_anim)
        memcpy(model->animinfo[0], model->animinfo[1], sizeof(NEA_AnimInfo));

    memset(model->animinfo[1], 0, sizeof(NEA_AnimInfo));
}

int NEA_ModelLoadDSMFAT(NEA_Model *model, const char *path)
{
    if (!ne_model_system_inited)
        return 0;

    NEA_Assert(model->modeltype == NEA_Animated, "Not an animated model");

    return ne_model_load_filesystem_common(model, path);
}

int NEA_ModelLoadDSM(NEA_Model *model, const void *pointer)
{
    if (!ne_model_system_inited)
        return 0;

    NEA_Assert(model->modeltype == NEA_Animated, "Not an animated model");

    return ne_model_load_ram_common(model, pointer);
}

void NEA_ModelDeleteAll(void)
{
    if (!ne_model_system_inited)
        return;

    for (int i = 0; i < NEA_MAX_MODELS; i++)
    {
        if (NEA_ModelPointers[i] != NULL)
            NEA_ModelDelete(NEA_ModelPointers[i]);
    }
}

int NEA_ModelSystemReset(int max_models)
{
    if (ne_model_system_inited)
        NEA_ModelSystemEnd();

    if (max_models < 1)
        NEA_MAX_MODELS = NEA_DEFAULT_MODELS;
    else
        NEA_MAX_MODELS = max_models;

    NEA_Mesh = calloc(NEA_MAX_MODELS, sizeof(ne_mesh_info_t));
    NEA_ModelPointers = calloc(NEA_MAX_MODELS, sizeof(NEA_ModelPointers));
    if ((NEA_Mesh == NULL) || (NEA_ModelPointers == NULL))
    {
        free(NEA_Mesh);
        free(NEA_ModelPointers);
        NEA_DebugPrint("Not enough memory");
        return -1;
    }

    ne_model_system_inited = true;
    return 0;
}

void NEA_ModelSystemEnd(void)
{
    if (!ne_model_system_inited)
        return;

    NEA_ModelDeleteAll();

    free(NEA_Mesh);
    free(NEA_ModelPointers);

    ne_model_system_inited = false;
}
