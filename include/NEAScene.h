// SPDX-License-Identifier: MIT
//
// Copyright (c) 2024-2026 Warioware64
//
// This file is part of Nitro Engine Advanced

#ifndef NEA_SCENE_H__
#define NEA_SCENE_H__

/// @file   NEAScene.h
/// @brief  Scene management system with node hierarchy, tags, and triggers.
///
/// Provides a parent-children node tree for organizing 3D scenes. Nodes can
/// be meshes, cameras, triggers (collision zones with callbacks), or empties
/// (transform groups). Scenes are loaded from binary .neascene files exported
/// from Blender.
///
/// Each node can carry up to 3 string tags for grouping/querying and a
/// user_data pointer for attaching game-specific state.

#include <nds.h>

#include "NEACamera.h"
#include "NEACollision.h"
#include "NEAModel.h"
#include "NEATexture.h"

/// @defgroup scene Scene management system
///
/// Scene node hierarchy with tagged grouping, trigger zones, and state
/// pointers. Scenes are loaded from binary .neascene files and automatically
/// create/manage NEA_Model and NEA_Camera objects.
///
/// @{

// =========================================================================
// Constants
// =========================================================================

#define NEA_SCENE_MAGIC          0x4E53434E  ///< "NSCN" little-endian.
#define NEA_SCENE_VERSION        1           ///< Current .neascene version.
#define NEA_DEFAULT_SCENE_NODES  64          ///< Default max nodes per scene.
#define NEA_NODE_TAG_LEN         16          ///< Max tag length (15 + null).
#define NEA_NODE_NAME_LEN        24          ///< Max node name length (23 + null).
#define NEA_NODE_MAX_TAGS        6           ///< Max tags per node.
#define NEA_SCENE_MAX_ASSETS     256         ///< Max asset references per scene.
#define NEA_SCENE_MAX_MATERIALS  128         ///< Max material references per scene.

// =========================================================================
// Enums
// =========================================================================

/// Node types in a scene hierarchy.
typedef enum {
    NEA_NODE_EMPTY   = 0, ///< Group/transform node (no visual).
    NEA_NODE_MESH    = 1, ///< Renderable mesh (static or animated).
    NEA_NODE_CAMERA  = 2, ///< Camera node.
    NEA_NODE_TRIGGER = 3, ///< Collision zone with enter/exit/tick callbacks.
} NEA_NodeType;

/// Trigger event types for callbacks.
typedef enum {
    NEA_TRIGGER_ENTER = 0, ///< Something entered the trigger zone.
    NEA_TRIGGER_EXIT  = 1, ///< Something left the trigger zone.
    NEA_TRIGGER_TICK  = 2, ///< Per-frame while something is inside.
} NEA_TriggerEvent;

// =========================================================================
// Forward declarations and callback types
// =========================================================================

/// Forward declaration.
typedef struct NEA_SceneNode_ NEA_SceneNode;

/// Trigger callback function signature.
///
/// @param trigger  The trigger node that fired.
/// @param event    The event type (enter, exit, or tick).
/// @param user_data  User data passed to NEA_SceneTestTriggers().
typedef void (*NEA_TriggerCallback)(NEA_SceneNode *trigger,
                                     NEA_TriggerEvent event,
                                     void *user_data);

/// Node visitor callback for tag iteration.
///
/// @param node  The matching node.
/// @param arg   User data passed to NEA_SceneForEachTag().
typedef void (*NEA_NodeVisitor)(NEA_SceneNode *node, void *arg);

// =========================================================================
// Trigger data
// =========================================================================

/// Per-node trigger data (only allocated for NEA_NODE_TRIGGER nodes).
typedef struct {
    NEA_ColShape       shape;    ///< Collision zone shape (sphere/AABB).
    NEA_TriggerCallback on_event; ///< Callback for enter/exit/tick events.
    bool               is_active; ///< True if something is currently inside.
    uint8_t            script_id; ///< Script identifier from .neascene file.
} NEA_TriggerData;

// =========================================================================
// Scene node
// =========================================================================

/// A single node in the scene hierarchy.
///
/// Nodes form a left-child right-sibling tree. Each node has a local
/// transform (position, rotation, scale) and optionally references an
/// NEA_Model or NEA_Camera created during scene loading.
struct NEA_SceneNode_ {
    // --- Hierarchy (left-child right-sibling tree) ---
    NEA_SceneNode *parent;       ///< Parent node (NULL for root).
    NEA_SceneNode *first_child;  ///< First child node.
    NEA_SceneNode *next_sibling; ///< Next sibling node.

    // --- Identity ---
    char          name[NEA_NODE_NAME_LEN]; ///< Unique node name.
    char          tags[NEA_NODE_MAX_TAGS][NEA_NODE_TAG_LEN]; ///< Tag strings.
    uint8_t       num_tags;      ///< Number of active tags (0-3).
    NEA_NodeType  type;          ///< Node type.

    // --- Local transform (same conventions as NEA_Model) ---
    int32_t  x, y, z;           ///< Local position (f32 fixed-point).
    int      rx, ry, rz;        ///< Local rotation (0-511).
    int32_t  sx, sy, sz;        ///< Local scale (f32, default = inttof32(1)).
    bool     visible;           ///< If false, skip draw for this subtree.

    // --- World position (computed by NEA_SceneUpdate) ---
    int32_t  wx, wy, wz;       ///< World position (f32, accumulated from parents).

    // --- Type-specific file references ---
    union {
        struct {
            uint16_t asset_index;    ///< Index into scene asset table.
            uint16_t material_index; ///< Index into scene material table (0xFFFF = none).
            uint8_t  is_animated;    ///< 1 = animated, 0 = static.
        } mesh;
        struct {
            int32_t  to[3];  ///< Look-at target (f32).
            int32_t  up[3];  ///< Up vector (f32).
        } cam;
    } ref;

    NEA_TriggerData *trigger;    ///< Non-NULL only for NEA_NODE_TRIGGER.

    // --- Runtime pointers (populated on scene load) ---
    NEA_Model  *model;           ///< Created model for mesh nodes.
    NEA_Camera *camera;          ///< Created camera for camera nodes.

    // --- Animated material (defined in NEAAnimMat.h) ---
    struct NEA_AnimMatInstance_ *animmat; ///< Animated material, or NULL.

    // --- User extension ---
    void *user_data;             ///< Game-specific state pointer.
};

// =========================================================================
// Scene asset/material tables
// =========================================================================

/// Asset entry loaded from the scene file.
typedef struct {
    char     path[48];  ///< Filesystem path (e.g. "nitro:/level/wall.bin").
    uint8_t  type;      ///< 0=static, 1=dsm, 2=dsa, 3=colmesh, 4=boncol.
    bool     loaded;    ///< True if data is loaded into RAM.
    void    *data;      ///< Runtime pointer to loaded data.
} NEA_SceneAsset;

/// Material reference in the scene file.
typedef struct {
    char name[32];      ///< Material name to bind.
    char tex_path[48];  ///< Texture file path (or empty for manual bind).
} NEA_SceneMaterialRef;

// =========================================================================
// Scene
// =========================================================================

/// A loaded scene containing a node hierarchy and asset references.
typedef struct {
    NEA_SceneNode *nodes;          ///< Flat array of all nodes.
    int            num_nodes;      ///< Number of nodes in the scene.
    NEA_SceneNode *root;           ///< Root node (nodes[0]).
    NEA_SceneNode *active_camera;  ///< Active camera node (or NULL).

    NEA_SceneAsset       assets[NEA_SCENE_MAX_ASSETS];
    int                  num_assets;
    NEA_SceneMaterialRef mat_refs[NEA_SCENE_MAX_MATERIALS];
    int                  num_mat_refs;

    NEA_Material *materials[NEA_SCENE_MAX_MATERIALS]; ///< Auto-loaded materials.

    bool loaded; ///< True if the scene is currently loaded.
} NEA_Scene;

// =========================================================================
// System lifecycle
// =========================================================================

/// Reset the scene system. Call during engine initialization.
///
/// @param max_nodes  Per-scene node limit. If < 1, uses
///                   NEA_DEFAULT_SCENE_NODES.
/// @return 0 on success, -1 on failure.
int NEA_SceneSystemReset(int max_nodes);

/// End the scene system and free all memory.
void NEA_SceneSystemEnd(void);

// =========================================================================
// Scene loading/freeing
// =========================================================================

/// Load a scene from a .neascene file on NitroFS/FAT.
///
/// Creates NEA_Model and NEA_Camera objects for mesh and camera nodes.
/// Assets referenced by the scene must be loadable from the filesystem.
///
/// @param path  Path to the .neascene file.
/// @return Pointer to the loaded scene, or NULL on error.
NEA_Scene *NEA_SceneLoadFAT(const char *path);

/// Load a scene from a .neascene binary already in RAM.
///
/// @param data  Pointer to the binary data.
/// @param size  Size of the data in bytes.
/// @return Pointer to the loaded scene, or NULL on error.
NEA_Scene *NEA_SceneLoad(const void *data, size_t size);

/// Free a scene and all its engine objects (models, cameras, triggers).
///
/// Does NOT free user_data on nodes -- the game must clean that up first.
///
/// @param scene  Pointer to the scene.
void NEA_SceneFree(NEA_Scene *scene);

// =========================================================================
// Node access
// =========================================================================

/// Get the root node of the scene.
///
/// @param scene  Pointer to the scene.
/// @return Root node, or NULL if scene is empty.
NEA_SceneNode *NEA_SceneGetRoot(const NEA_Scene *scene);

/// Find a node by name (linear search, O(n)).
///
/// @param scene  Pointer to the scene.
/// @param name   Node name to search for.
/// @return Pointer to the node, or NULL if not found.
NEA_SceneNode *NEA_SceneFindNode(const NEA_Scene *scene, const char *name);

/// Get the active camera node.
///
/// @param scene  Pointer to the scene.
/// @return Active camera node, or NULL if none is set.
NEA_SceneNode *NEA_SceneGetActiveCamera(const NEA_Scene *scene);

/// Set the active camera by node pointer.
///
/// @param scene     Pointer to the scene.
/// @param cam_node  Camera node to activate (must be NEA_NODE_CAMERA).
void NEA_SceneSetActiveCamera(NEA_Scene *scene, NEA_SceneNode *cam_node);

// =========================================================================
// Tag queries
// =========================================================================

/// Find the first node with a given tag.
///
/// @param scene  Pointer to the scene.
/// @param tag    Tag string to search for.
/// @return Pointer to the first matching node, or NULL.
NEA_SceneNode *NEA_SceneFindByTag(const NEA_Scene *scene, const char *tag);

/// Call a visitor callback for each node that has the given tag.
///
/// @param scene    Pointer to the scene.
/// @param tag      Tag string to match.
/// @param visitor  Callback to invoke for each match.
/// @param arg      User data passed to the callback.
/// @return Number of nodes matched.
int NEA_SceneForEachTag(const NEA_Scene *scene, const char *tag,
                        NEA_NodeVisitor visitor, void *arg);

/// Count nodes with a given tag.
///
/// @param scene  Pointer to the scene.
/// @param tag    Tag string to match.
/// @return Number of matching nodes.
int NEA_SceneCountByTag(const NEA_Scene *scene, const char *tag);

// =========================================================================
// Node manipulation
// =========================================================================

/// Set a node's local position (f32 fixed-point).
///
/// @param node  Pointer to the node.
/// @param x     X position (f32).
/// @param y     Y position (f32).
/// @param z     Z position (f32).
void NEA_SceneNodeSetCoordI(NEA_SceneNode *node,
                             int32_t x, int32_t y, int32_t z);

/// Set a node's local position (float).
///
/// @param n  Pointer to the node.
/// @param x  X position (float).
/// @param y  Y position (float).
/// @param z  Z position (float).
#define NEA_SceneNodeSetCoord(n, x, y, z) \
    NEA_SceneNodeSetCoordI(n, floattof32(x), floattof32(y), floattof32(z))

/// Set a node's local rotation.
///
/// @param node  Pointer to the node.
/// @param rx    X rotation (0-511).
/// @param ry    Y rotation (0-511).
/// @param rz    Z rotation (0-511).
void NEA_SceneNodeSetRot(NEA_SceneNode *node, int rx, int ry, int rz);

/// Set a node's visibility. Hidden nodes and their children are not drawn.
///
/// @param node     Pointer to the node.
/// @param visible  True to show, false to hide.
void NEA_SceneNodeSetVisible(NEA_SceneNode *node, bool visible);

/// Set user data on a node.
///
/// @param node  Pointer to the node.
/// @param data  User data pointer.
void NEA_SceneNodeSetUserData(NEA_SceneNode *node, void *data);

/// Get user data from a node.
///
/// @param node  Pointer to the node.
/// @return User data pointer, or NULL.
void *NEA_SceneNodeGetUserData(const NEA_SceneNode *node);

// =========================================================================
// Scene update and draw
// =========================================================================

/// Update all scene nodes.
///
/// Propagates transforms through the hierarchy and syncs NEA_Model and
/// NEA_Camera positions from their corresponding nodes. Call once per frame.
///
/// @param scene  Pointer to the scene.
void NEA_SceneUpdate(NEA_Scene *scene);

/// Test a shape against all trigger nodes in the scene.
///
/// Calls trigger callbacks (enter/exit/tick) as appropriate. Call after
/// NEA_SceneUpdate so that trigger world positions are up to date.
///
/// @param scene      Pointer to the scene.
/// @param shape      Collision shape to test against triggers.
/// @param pos        World position of the test shape (f32).
/// @param user_data  Passed to trigger callbacks.
void NEA_SceneTestTriggers(NEA_Scene *scene, const NEA_ColShape *shape,
                           NEA_Vec3 pos, void *user_data);

/// Draw the entire scene.
///
/// Activates the active camera, then traverses the node tree depth-first,
/// drawing all visible mesh nodes. Designed to be used with NEA_ProcessArg():
///
///     NEA_ProcessArg(NEA_SceneDraw, scene);
///
/// @param arg  Pointer to the NEA_Scene (cast to void* for ProcessArg).
void NEA_SceneDraw(void *arg);

/// @}

#endif // NEA_SCENE_H__
