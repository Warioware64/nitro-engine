// SPDX-License-Identifier: MIT
//
// Copyright (c) 2024-2026 Warioware64
//
// This file is part of Nitro Engine Advanced

#include "NEAMain.h"

/// @file NEAScene.c

static int ne_scene_max_nodes = 0;
static bool ne_scene_system_inited = false;

// =========================================================================
// .neascene binary format structures
// =========================================================================

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint16_t num_nodes;
    uint16_t num_assets;
    uint16_t num_mat_refs;
    uint16_t active_camera_idx;
} neascene_header_t;

#define NEASCENE_ASSET_SIZE    64
#define NEASCENE_MATREF_SIZE   80
#define NEASCENE_NODE_SIZE     128

// =========================================================================
// System lifecycle
// =========================================================================

int NEA_SceneSystemReset(int max_nodes)
{
    if (ne_scene_system_inited)
        NEA_SceneSystemEnd();

    if (max_nodes < 1)
        ne_scene_max_nodes = NEA_DEFAULT_SCENE_NODES;
    else
        ne_scene_max_nodes = max_nodes;

    ne_scene_system_inited = true;
    return 0;
}

void NEA_SceneSystemEnd(void)
{
    ne_scene_system_inited = false;
    ne_scene_max_nodes = 0;
}

// =========================================================================
// Tag helper
// =========================================================================

static bool ne_node_has_tag(const NEA_SceneNode *node, const char *tag)
{
    for (int i = 0; i < node->num_tags; i++)
    {
        if (strcmp(node->tags[i], tag) == 0)
            return true;
    }
    return false;
}

// =========================================================================
// Scene loading (binary .neascene)
// =========================================================================

static NEA_Scene *ne_scene_parse(const void *data, size_t size)
{
    if (size < sizeof(neascene_header_t))
    {
        NEA_DebugPrint("Scene data too small");
        return NULL;
    }

    const neascene_header_t *hdr = (const neascene_header_t *)data;

    if (hdr->magic != NEA_SCENE_MAGIC)
    {
        NEA_DebugPrint("Invalid .neascene magic");
        return NULL;
    }
    if (hdr->version != NEA_SCENE_VERSION)
    {
        NEA_DebugPrint("Unsupported .neascene version");
        return NULL;
    }

    int num_nodes = hdr->num_nodes;
    if (num_nodes < 1 || num_nodes > ne_scene_max_nodes)
    {
        NEA_DebugPrint("Invalid node count");
        return NULL;
    }

    NEA_Scene *scene = calloc(1, sizeof(NEA_Scene));
    if (scene == NULL)
    {
        NEA_DebugPrint("Not enough memory for scene");
        return NULL;
    }

    scene->nodes = calloc(num_nodes, sizeof(NEA_SceneNode));
    if (scene->nodes == NULL)
    {
        NEA_DebugPrint("Not enough memory for nodes");
        free(scene);
        return NULL;
    }

    scene->num_nodes = num_nodes;
    scene->num_assets = hdr->num_assets;
    scene->num_mat_refs = hdr->num_mat_refs;

    const uint8_t *ptr = (const uint8_t *)data + sizeof(neascene_header_t);

    // --- Parse asset table ---
    for (int i = 0; i < scene->num_assets && i < NEA_SCENE_MAX_ASSETS; i++)
    {
        memcpy(scene->assets[i].path, ptr, 48);
        scene->assets[i].path[47] = '\0';
        scene->assets[i].type = ptr[48];
        scene->assets[i].loaded = false;
        scene->assets[i].data = NULL;
        ptr += NEASCENE_ASSET_SIZE;
    }

    // --- Parse material ref table ---
    for (int i = 0; i < scene->num_mat_refs && i < NEA_SCENE_MAX_MATERIALS; i++)
    {
        memcpy(scene->mat_refs[i].name, ptr, 32);
        scene->mat_refs[i].name[31] = '\0';
        memcpy(scene->mat_refs[i].tex_path, ptr + 32, 48);
        scene->mat_refs[i].tex_path[47] = '\0';
        ptr += NEASCENE_MATREF_SIZE;
    }

    // --- Parse node table ---
    for (int i = 0; i < num_nodes; i++)
    {
        NEA_SceneNode *node = &scene->nodes[i];
        const uint8_t *np = ptr;

        // Name (24 bytes)
        memcpy(node->name, np, NEA_NODE_NAME_LEN);
        node->name[NEA_NODE_NAME_LEN - 1] = '\0';

        // Type, parent, tags, flags
        node->type = (NEA_NodeType)np[24];
        uint8_t parent_idx = np[25];
        node->num_tags = np[26];
        if (node->num_tags > NEA_NODE_MAX_TAGS)
            node->num_tags = NEA_NODE_MAX_TAGS;
        node->visible = (np[27] & 1) != 0;

        // Position (f32 x3 at offset 28)
        const int32_t *pos = (const int32_t *)(np + 28);
        node->x = pos[0];
        node->y = pos[1];
        node->z = pos[2];

        // Rotation (int16 x3 at offset 40)
        const int16_t *rot = (const int16_t *)(np + 40);
        node->rx = rot[0];
        node->ry = rot[1];
        node->rz = rot[2];
        // 2 bytes padding at 46

        // Scale (f32 x3 at offset 48)
        const int32_t *scl = (const int32_t *)(np + 48);
        node->sx = scl[0];
        node->sy = scl[1];
        node->sz = scl[2];

        // Type-specific data at offset 60
        const uint8_t *td = np + 60;
        if (node->type == NEA_NODE_MESH)
        {
            const uint16_t *mesh_data = (const uint16_t *)td;
            node->ref.mesh.asset_index = mesh_data[0];
            node->ref.mesh.material_index = mesh_data[1];
            node->ref.mesh.is_animated = td[4];
        }
        else if (node->type == NEA_NODE_CAMERA)
        {
            const int32_t *cam_data = (const int32_t *)td;
            node->ref.cam.to[0] = cam_data[0];
            node->ref.cam.to[1] = cam_data[1];
            node->ref.cam.to[2] = cam_data[2];
            node->ref.cam.up[0] = cam_data[3];
            node->ref.cam.up[1] = cam_data[4];
            node->ref.cam.up[2] = cam_data[5];
        }
        else if (node->type == NEA_NODE_TRIGGER)
        {
            node->trigger = calloc(1, sizeof(NEA_TriggerData));
            if (node->trigger != NULL)
            {
                uint8_t shape_type = td[0];
                node->trigger->script_id = td[1];

                const int32_t *sp = (const int32_t *)(td + 4);
                if (shape_type == 1) // sphere
                    NEA_ColShapeInitSphereI(&node->trigger->shape, sp[0]);
                else if (shape_type == 2) // AABB
                    NEA_ColShapeInitAABBI(&node->trigger->shape,
                                          sp[0], sp[1], sp[2]);
            }
        }

        // Tags at offset 80 (3 * 16 = 48 bytes)
        const char *tag_ptr = (const char *)(np + 80);
        for (int t = 0; t < node->num_tags; t++)
        {
            memcpy(node->tags[t], tag_ptr + t * NEA_NODE_TAG_LEN,
                   NEA_NODE_TAG_LEN);
            node->tags[t][NEA_NODE_TAG_LEN - 1] = '\0';
        }

        // Build hierarchy links
        node->parent = NULL;
        node->first_child = NULL;
        node->next_sibling = NULL;

        if (parent_idx != 0xFF && parent_idx < num_nodes)
        {
            NEA_SceneNode *parent = &scene->nodes[parent_idx];
            node->parent = parent;

            // Append as last child
            if (parent->first_child == NULL)
            {
                parent->first_child = node;
            }
            else
            {
                NEA_SceneNode *sibling = parent->first_child;
                while (sibling->next_sibling != NULL)
                    sibling = sibling->next_sibling;
                sibling->next_sibling = node;
            }
        }

        // Initialize runtime pointers
        node->model = NULL;
        node->camera = NULL;
        node->animmat = NULL;
        node->user_data = NULL;

        ptr += NEASCENE_NODE_SIZE;
    }

    // --- Create engine objects for mesh and camera nodes ---
    for (int i = 0; i < num_nodes; i++)
    {
        NEA_SceneNode *node = &scene->nodes[i];

        if (node->type == NEA_NODE_MESH)
        {
            node->model = NEA_ModelCreate(
                node->ref.mesh.is_animated ? NEA_Animated : NEA_Static);

            if (node->model != NULL)
            {
                // Load mesh from asset table if available
                uint16_t ai = node->ref.mesh.asset_index;
                if (ai < scene->num_assets && scene->assets[ai].path[0] != '\0')
                {
                    if (!scene->assets[ai].loaded)
                    {
                        scene->assets[ai].data =
                            NEA_FATLoadData(scene->assets[ai].path);
                        if (scene->assets[ai].data != NULL)
                            scene->assets[ai].loaded = true;
                    }

                    if (scene->assets[ai].loaded)
                    {
                        NEA_ModelLoadStaticMesh(node->model,
                                                scene->assets[ai].data);
                        NEA_ModelFreeMeshWhenDeleted(node->model);
                    }
                }

                // Apply initial transform
                NEA_ModelSetCoordI(node->model, node->x, node->y, node->z);
                NEA_ModelSetRot(node->model, node->rx, node->ry, node->rz);
                NEA_ModelScaleI(node->model, node->sx, node->sy, node->sz);
            }
        }
        else if (node->type == NEA_NODE_CAMERA)
        {
            node->camera = NEA_CameraCreate();
            if (node->camera != NULL)
            {
                NEA_CameraSetI(node->camera,
                               node->x, node->y, node->z,
                               node->ref.cam.to[0], node->ref.cam.to[1],
                               node->ref.cam.to[2],
                               node->ref.cam.up[0], node->ref.cam.up[1],
                               node->ref.cam.up[2]);
            }
        }
    }

    // Set root
    scene->root = &scene->nodes[0];

    // Set active camera
    if (hdr->active_camera_idx != 0xFFFF &&
        hdr->active_camera_idx < num_nodes)
    {
        NEA_SceneNode *cam = &scene->nodes[hdr->active_camera_idx];
        if (cam->type == NEA_NODE_CAMERA)
            scene->active_camera = cam;
    }

    scene->loaded = true;
    return scene;
}

NEA_Scene *NEA_SceneLoad(const void *data, size_t size)
{
    NEA_AssertPointer(data, "NULL data pointer");

    if (!ne_scene_system_inited)
    {
        NEA_DebugPrint("Scene system not initialized");
        return NULL;
    }

    return ne_scene_parse(data, size);
}

NEA_Scene *NEA_SceneLoadFAT(const char *path)
{
    NEA_AssertPointer(path, "NULL path");

    if (!ne_scene_system_inited)
    {
        NEA_DebugPrint("Scene system not initialized");
        return NULL;
    }

    FILE *f = fopen(path, "rb");
    if (f == NULL)
    {
        NEA_DebugPrint("Can't open %s", path);
        return NULL;
    }

    fseek(f, 0, SEEK_END);
    long fsize = ftell(f);
    fseek(f, 0, SEEK_SET);

    void *data = malloc(fsize);
    if (data == NULL)
    {
        NEA_DebugPrint("Not enough memory");
        fclose(f);
        return NULL;
    }

    fread(data, 1, fsize, f);
    fclose(f);

    NEA_Scene *scene = ne_scene_parse(data, (size_t)fsize);
    free(data);

    if (scene == NULL)
        return NULL;

    // --- Auto-load GRF textures from material references ---
    for (int i = 0; i < scene->num_mat_refs; i++)
    {
        scene->materials[i] = NULL;

        if (scene->mat_refs[i].tex_path[0] == '\0')
            continue;

        NEA_Material *mat = NEA_MaterialCreate();
        if (mat == NULL)
        {
            NEA_DebugPrint("Can't create material %d", i);
            continue;
        }

        if (NEA_MaterialTexLoadGRF(mat, NULL,
                                    NEA_TEXGEN_TEXCOORD,
                                    scene->mat_refs[i].tex_path) == 0)
        {
            NEA_DebugPrint("Failed to load GRF: %s",
                           scene->mat_refs[i].tex_path);
            NEA_MaterialDelete(mat);
            continue;
        }

        scene->materials[i] = mat;
    }

    // --- Bind auto-loaded materials to mesh nodes ---
    for (int i = 0; i < scene->num_nodes; i++)
    {
        NEA_SceneNode *node = &scene->nodes[i];
        if (node->type != NEA_NODE_MESH || node->model == NULL)
            continue;

        uint16_t mi = node->ref.mesh.material_index;
        if (mi == 0xFFFF || mi >= scene->num_mat_refs)
            continue;

        if (scene->materials[mi] != NULL)
            NEA_ModelSetMaterial(node->model, scene->materials[mi]);
    }

    return scene;
}

void NEA_SceneFree(NEA_Scene *scene)
{
    if (scene == NULL)
        return;

    // Delete engine objects
    for (int i = 0; i < scene->num_nodes; i++)
    {
        NEA_SceneNode *node = &scene->nodes[i];

        if (node->model != NULL)
            NEA_ModelDelete(node->model);
        if (node->camera != NULL)
            NEA_CameraDelete(node->camera);
        if (node->trigger != NULL)
            free(node->trigger);
        if (node->animmat != NULL)
            NEA_AnimMatDelete(node->animmat);
    }

    // Free auto-loaded materials
    for (int i = 0; i < scene->num_mat_refs; i++)
    {
        if (scene->materials[i] != NULL)
            NEA_MaterialDelete(scene->materials[i]);
    }

    // Free loaded assets
    for (int i = 0; i < scene->num_assets; i++)
    {
        if (scene->assets[i].loaded && scene->assets[i].data != NULL)
            free(scene->assets[i].data);
    }

    free(scene->nodes);
    free(scene);
}

// =========================================================================
// Node access
// =========================================================================

NEA_SceneNode *NEA_SceneGetRoot(const NEA_Scene *scene)
{
    NEA_AssertPointer(scene, "NULL scene");
    return scene->root;
}

NEA_SceneNode *NEA_SceneFindNode(const NEA_Scene *scene, const char *name)
{
    NEA_AssertPointer(scene, "NULL scene");
    NEA_AssertPointer(name, "NULL name");

    for (int i = 0; i < scene->num_nodes; i++)
    {
        if (strcmp(scene->nodes[i].name, name) == 0)
            return &scene->nodes[i];
    }
    return NULL;
}

NEA_SceneNode *NEA_SceneGetActiveCamera(const NEA_Scene *scene)
{
    NEA_AssertPointer(scene, "NULL scene");
    return scene->active_camera;
}

void NEA_SceneSetActiveCamera(NEA_Scene *scene, NEA_SceneNode *cam_node)
{
    NEA_AssertPointer(scene, "NULL scene");
    if (cam_node != NULL)
        NEA_Assert(cam_node->type == NEA_NODE_CAMERA,
                   "Node is not a camera");
    scene->active_camera = cam_node;
}

// =========================================================================
// Tag queries
// =========================================================================

NEA_SceneNode *NEA_SceneFindByTag(const NEA_Scene *scene, const char *tag)
{
    NEA_AssertPointer(scene, "NULL scene");
    NEA_AssertPointer(tag, "NULL tag");

    for (int i = 0; i < scene->num_nodes; i++)
    {
        if (ne_node_has_tag(&scene->nodes[i], tag))
            return &scene->nodes[i];
    }
    return NULL;
}

int NEA_SceneForEachTag(const NEA_Scene *scene, const char *tag,
                        NEA_NodeVisitor visitor, void *arg)
{
    NEA_AssertPointer(scene, "NULL scene");
    NEA_AssertPointer(tag, "NULL tag");
    NEA_AssertPointer(visitor, "NULL visitor");

    int count = 0;
    for (int i = 0; i < scene->num_nodes; i++)
    {
        if (ne_node_has_tag(&scene->nodes[i], tag))
        {
            visitor(&scene->nodes[i], arg);
            count++;
        }
    }
    return count;
}

int NEA_SceneCountByTag(const NEA_Scene *scene, const char *tag)
{
    NEA_AssertPointer(scene, "NULL scene");
    NEA_AssertPointer(tag, "NULL tag");

    int count = 0;
    for (int i = 0; i < scene->num_nodes; i++)
    {
        if (ne_node_has_tag(&scene->nodes[i], tag))
            count++;
    }
    return count;
}

// =========================================================================
// Node manipulation
// =========================================================================

void NEA_SceneNodeSetCoordI(NEA_SceneNode *node,
                             int32_t x, int32_t y, int32_t z)
{
    NEA_AssertPointer(node, "NULL node");
    node->x = x;
    node->y = y;
    node->z = z;
}

void NEA_SceneNodeSetRot(NEA_SceneNode *node, int rx, int ry, int rz)
{
    NEA_AssertPointer(node, "NULL node");
    node->rx = rx;
    node->ry = ry;
    node->rz = rz;
}

void NEA_SceneNodeSetVisible(NEA_SceneNode *node, bool visible)
{
    NEA_AssertPointer(node, "NULL node");
    node->visible = visible;
}

void NEA_SceneNodeSetUserData(NEA_SceneNode *node, void *data)
{
    NEA_AssertPointer(node, "NULL node");
    node->user_data = data;
}

void *NEA_SceneNodeGetUserData(const NEA_SceneNode *node)
{
    NEA_AssertPointer(node, "NULL node");
    return node->user_data;
}

// =========================================================================
// Scene update
// =========================================================================

// Recursive transform propagation. Computes world position by adding
// parent's world position. Uses hardware mulf32 for any rotated offsets.
ARM_CODE static void ne_scene_update_recursive(NEA_SceneNode *node,
                                                int32_t parent_wx,
                                                int32_t parent_wy,
                                                int32_t parent_wz)
{
    if (node == NULL)
        return;

    // Compute world position (parent world + local offset)
    node->wx = parent_wx + node->x;
    node->wy = parent_wy + node->y;
    node->wz = parent_wz + node->z;

    // Sync to engine objects
    if (node->type == NEA_NODE_MESH && node->model != NULL)
    {
        NEA_ModelSetCoordI(node->model, node->wx, node->wy, node->wz);
        NEA_ModelSetRot(node->model, node->rx, node->ry, node->rz);
        NEA_ModelScaleI(node->model, node->sx, node->sy, node->sz);
    }
    else if (node->type == NEA_NODE_CAMERA && node->camera != NULL)
    {
        // Update camera from position. If the camera has a look-at target
        // stored in ref.cam.to, offset it by world position.
        node->camera->from[0] = node->wx;
        node->camera->from[1] = node->wy;
        node->camera->from[2] = node->wz;
        node->camera->to[0] = node->wx + node->ref.cam.to[0];
        node->camera->to[1] = node->wy + node->ref.cam.to[1];
        node->camera->to[2] = node->wz + node->ref.cam.to[2];
        node->camera->up[0] = node->ref.cam.up[0];
        node->camera->up[1] = node->ref.cam.up[1];
        node->camera->up[2] = node->ref.cam.up[2];
        node->camera->matrix_is_updated = false;
    }

    // Recurse children
    NEA_SceneNode *child = node->first_child;
    while (child != NULL)
    {
        ne_scene_update_recursive(child, node->wx, node->wy, node->wz);
        child = child->next_sibling;
    }
}

void NEA_SceneUpdate(NEA_Scene *scene)
{
    NEA_AssertPointer(scene, "NULL scene");

    if (!scene->loaded || scene->root == NULL)
        return;

    ne_scene_update_recursive(scene->root, 0, 0, 0);
}

// =========================================================================
// Trigger testing
// =========================================================================

void NEA_SceneTestTriggers(NEA_Scene *scene, const NEA_ColShape *shape,
                           NEA_Vec3 pos, void *user_data)
{
    NEA_AssertPointer(scene, "NULL scene");
    NEA_AssertPointer(shape, "NULL shape");

    for (int i = 0; i < scene->num_nodes; i++)
    {
        NEA_SceneNode *node = &scene->nodes[i];
        if (node->type != NEA_NODE_TRIGGER || node->trigger == NULL)
            continue;
        if (!node->visible)
            continue;

        NEA_Vec3 trigger_pos = NEA_Vec3Make(node->wx, node->wy, node->wz);

        NEA_ColResult result = NEA_ColTest(&node->trigger->shape, trigger_pos,
                                           shape, pos);

        bool was_active = node->trigger->is_active;
        bool is_inside = result.hit;

        if (is_inside && !was_active)
        {
            // Enter
            node->trigger->is_active = true;
            if (node->trigger->on_event)
                node->trigger->on_event(node, NEA_TRIGGER_ENTER, user_data);
        }
        else if (is_inside && was_active)
        {
            // Tick (still inside)
            if (node->trigger->on_event)
                node->trigger->on_event(node, NEA_TRIGGER_TICK, user_data);
        }
        else if (!is_inside && was_active)
        {
            // Exit
            node->trigger->is_active = false;
            if (node->trigger->on_event)
                node->trigger->on_event(node, NEA_TRIGGER_EXIT, user_data);
        }
    }
}

// =========================================================================
// Scene draw
// =========================================================================

ARM_CODE static void ne_scene_draw_recursive(NEA_SceneNode *node)
{
    if (node == NULL || !node->visible)
        return;

    if (node->type == NEA_NODE_MESH && node->model != NULL)
    {
        if (node->animmat != NULL)
            NEA_AnimMatApply(node->animmat);
        NEA_ModelDraw(node->model);
    }

    NEA_SceneNode *child = node->first_child;
    while (child != NULL)
    {
        ne_scene_draw_recursive(child);
        child = child->next_sibling;
    }
}

void NEA_SceneDraw(void *arg)
{
    NEA_Scene *scene = (NEA_Scene *)arg;
    if (scene == NULL || !scene->loaded)
        return;

    if (scene->active_camera != NULL && scene->active_camera->camera != NULL)
        NEA_CameraUse(scene->active_camera->camera);

    ne_scene_draw_recursive(scene->root);
}
