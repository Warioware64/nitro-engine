// SPDX-License-Identifier: MIT
//
// Copyright (c) 2008-2022 Antonio Niño Díaz
// Copyright (c) 2024-2026 Warioware64
//
// This file is part of Nitro Engine Advanced

#ifndef NEA_COLLISION_H__
#define NEA_COLLISION_H__

/// @file   NEACollision.h
/// @brief  Collision shapes, detection algorithms, and ColMesh loading.

#include <nds.h>

/// @defgroup collision Collision system
///
/// Collision shapes and narrow-phase detection for AABB, Sphere, Capsule, and
/// triangle mesh (ColMesh) colliders. All values are in f32 fixed-point format.
///
/// @{

// =========================================================================
// Vector3 helper (f32 fixed-point)
// =========================================================================

/// 3D vector in f32 fixed-point format.
typedef struct {
    int32_t x, y, z;
} NEA_Vec3;

/// Create a NEA_Vec3 from three f32 components.
static inline NEA_Vec3 NEA_Vec3Make(int32_t x, int32_t y, int32_t z)
{
    NEA_Vec3 v = { x, y, z };
    return v;
}

/// Add two vectors.
static inline NEA_Vec3 NEA_Vec3Add(NEA_Vec3 a, NEA_Vec3 b)
{
    NEA_Vec3 v = { a.x + b.x, a.y + b.y, a.z + b.z };
    return v;
}

/// Subtract two vectors (a - b).
static inline NEA_Vec3 NEA_Vec3Sub(NEA_Vec3 a, NEA_Vec3 b)
{
    NEA_Vec3 v = { a.x - b.x, a.y - b.y, a.z - b.z };
    return v;
}

/// Negate a vector.
static inline NEA_Vec3 NEA_Vec3Neg(NEA_Vec3 a)
{
    NEA_Vec3 v = { -a.x, -a.y, -a.z };
    return v;
}

/// Dot product of two vectors (f32 result).
static inline int32_t NEA_Vec3Dot(NEA_Vec3 a, NEA_Vec3 b)
{
    int32_t va[3] = { a.x, a.y, a.z };
    int32_t vb[3] = { b.x, b.y, b.z };
    return dotf32(va, vb);
}

/// Squared length of a vector (f32 result, but may overflow for large vectors).
static inline int32_t NEA_Vec3LengthSq(NEA_Vec3 a)
{
    return NEA_Vec3Dot(a, a);
}

/// Scale a vector by an f32 scalar.
static inline NEA_Vec3 NEA_Vec3Scale(NEA_Vec3 a, int32_t s)
{
    NEA_Vec3 v = { mulf32(a.x, s), mulf32(a.y, s), mulf32(a.z, s) };
    return v;
}

/// Cross product of two vectors (f32).
static inline NEA_Vec3 NEA_Vec3Cross(NEA_Vec3 a, NEA_Vec3 b)
{
    int32_t va[3] = { a.x, a.y, a.z };
    int32_t vb[3] = { b.x, b.y, b.z };
    int32_t vr[3];
    crossf32(va, vb, vr);
    NEA_Vec3 v = { vr[0], vr[1], vr[2] };
    return v;
}

/// Normalize a vector in-place and return it.
static inline NEA_Vec3 NEA_Vec3Normalize(NEA_Vec3 a)
{
    int32_t v[3] = { a.x, a.y, a.z };
    normalizef32(v);
    NEA_Vec3 r = { v[0], v[1], v[2] };
    return r;
}

/// Clamp a scalar to [min, max].
static inline int32_t NEA_Clamp(int32_t val, int32_t min, int32_t max)
{
    if (val < min) return min;
    if (val > max) return max;
    return val;
}

// =========================================================================
// Collision shape types
// =========================================================================

/// Types of collision shapes.
typedef enum {
    NEA_COL_NONE    = 0, ///< No collision shape.
    NEA_COL_AABB    = 1, ///< Axis-aligned bounding box.
    NEA_COL_SPHERE  = 2, ///< Bounding sphere.
    NEA_COL_CAPSULE = 3, ///< Capsule (line segment + radius).
    NEA_COL_TRIMESH = 4, ///< Triangle mesh collider (ColMesh).
} NEA_ColShapeType;

// =========================================================================
// Shape definitions
// =========================================================================

/// AABB collision shape. Stored as half-extents from center.
typedef struct {
    NEA_Vec3 half; ///< Half-extents on each axis (f32).
} NEA_ColAABB;

/// Sphere collision shape.
typedef struct {
    int32_t radius; ///< Radius (f32).
} NEA_ColSphere;

/// Capsule collision shape.
///
/// A capsule is a line segment from (0, -half_height, 0) to
/// (0, +half_height, 0) in local space, swept by a sphere of given radius.
typedef struct {
    int32_t half_height; ///< Half the height of the center segment (f32).
    int32_t radius;      ///< Sweep radius (f32).
} NEA_ColCapsule;

/// A single triangle for ColMesh collision.
typedef struct {
    NEA_Vec3 v0, v1, v2; ///< Vertices in model-local space (f32).
    NEA_Vec3 normal;      ///< Pre-computed face normal (f32, unit length).
} NEA_ColTriangle;

/// ColMesh flags.
#define NEA_COLMESH_STATIC  0       ///< Position-only transform (default).
#define NEA_COLMESH_DYNAMIC (1 << 0) ///< Full transform (rotate/scale).

/// Triangle mesh collider (ColMesh).
///
/// Uses a flat triangle array with a whole-mesh AABB for early rejection.
/// Supports two modes:
/// - Static: position-only offset, triangles stay in local space.
/// - Dynamic: full transform via NEA_ColMeshUpdateTransform().
typedef struct {
    NEA_ColAABB bounds;          ///< Whole-mesh AABB half-extents for early rejection.
    NEA_Vec3    center;          ///< AABB center in local space (f32).
    uint16_t    num_triangles;   ///< Number of triangles (max 65535).
    uint16_t    flags;           ///< NEA_COLMESH_STATIC or NEA_COLMESH_DYNAMIC.
    NEA_ColTriangle *triangles;  ///< Local-space triangles (original data).
    NEA_ColTriangle *world_tris; ///< World-space triangles (NULL if static).
} NEA_ColMesh;

/// Unified collision shape.
typedef struct {
    NEA_ColShapeType type; ///< Shape type.
    union {
        NEA_ColAABB    aabb;    ///< AABB data.
        NEA_ColSphere  sphere;  ///< Sphere data.
        NEA_ColCapsule capsule; ///< Capsule data.
        NEA_ColMesh    *mesh;   ///< ColMesh pointer (loaded separately).
    } shape;
} NEA_ColShape;

// =========================================================================
// Collision result
// =========================================================================

/// Result of a narrow-phase collision test.
typedef struct {
    bool     hit;    ///< True if shapes overlap.
    NEA_Vec3 normal; ///< Collision normal from A toward B (f32, unit length).
    int32_t  depth;  ///< Penetration depth (f32, positive = overlapping).
    NEA_Vec3 point;  ///< Approximate contact point in world space (f32).
} NEA_ColResult;

// =========================================================================
// Shape initialization
// =========================================================================

/// Initialize an AABB collision shape (f32 half-extents).
///
/// @param shape Pointer to collision shape to initialize.
/// @param hx Half-extent on X axis (f32).
/// @param hy Half-extent on Y axis (f32).
/// @param hz Half-extent on Z axis (f32).
void NEA_ColShapeInitAABBI(NEA_ColShape *shape, int32_t hx, int32_t hy,
                           int32_t hz);

/// Initialize an AABB collision shape (float half-extents).
#define NEA_ColShapeInitAABB(s, hx, hy, hz) \
    NEA_ColShapeInitAABBI(s, floattof32(hx), floattof32(hy), floattof32(hz))

/// Initialize a sphere collision shape (f32 radius).
///
/// @param shape Pointer to collision shape to initialize.
/// @param radius Sphere radius (f32).
void NEA_ColShapeInitSphereI(NEA_ColShape *shape, int32_t radius);

/// Initialize a sphere collision shape (float radius).
#define NEA_ColShapeInitSphere(s, r) \
    NEA_ColShapeInitSphereI(s, floattof32(r))

/// Initialize a capsule collision shape (f32).
///
/// @param shape Pointer to collision shape to initialize.
/// @param half_height Half the height of the center line segment (f32).
/// @param radius Sweep radius (f32).
void NEA_ColShapeInitCapsuleI(NEA_ColShape *shape, int32_t half_height,
                              int32_t radius);

/// Initialize a capsule collision shape (float).
#define NEA_ColShapeInitCapsule(s, hh, r) \
    NEA_ColShapeInitCapsuleI(s, floattof32(hh), floattof32(r))

/// Initialize a collision shape from a loaded ColMesh.
///
/// @param shape Pointer to collision shape to initialize.
/// @param mesh Pointer to a loaded NEA_ColMesh.
void NEA_ColShapeInitMesh(NEA_ColShape *shape, NEA_ColMesh *mesh);

// =========================================================================
// ColMesh loading and management
// =========================================================================

/// Load a ColMesh from a .colmesh binary in RAM.
///
/// @param data Pointer to the .colmesh binary data.
/// @return Pointer to the loaded ColMesh, or NULL on error.
NEA_ColMesh *NEA_ColMeshLoad(const void *data);

/// Load a ColMesh from a .colmesh file on the filesystem (FAT).
///
/// @param path Path to the .colmesh file.
/// @return Pointer to the loaded ColMesh, or NULL on error.
NEA_ColMesh *NEA_ColMeshLoadFAT(const char *path);

/// Set a ColMesh to dynamic mode (full transform support).
///
/// In dynamic mode, NEA_ColMeshUpdateTransform() can be used to transform
/// all triangles by a matrix each frame. This allocates a second triangle
/// buffer for world-space data.
///
/// @param mesh Pointer to the ColMesh.
/// @param dynamic True to enable dynamic mode, false for static.
void NEA_ColMeshSetDynamic(NEA_ColMesh *mesh, bool dynamic);

/// Transform a dynamic ColMesh by a 4x3 matrix.
///
/// This transforms all triangles from local to world space and recomputes
/// the bounding AABB. Call once per frame for dynamic meshes before
/// collision tests. Only works if NEA_ColMeshSetDynamic() was called.
///
/// @param mesh Pointer to the ColMesh.
/// @param matrix 4x3 transformation matrix.
void NEA_ColMeshUpdateTransform(NEA_ColMesh *mesh, const m4x3 *matrix);

/// Free a ColMesh and all associated memory.
///
/// @param mesh Pointer to the ColMesh.
void NEA_ColMeshFree(NEA_ColMesh *mesh);

// =========================================================================
// Narrow-phase collision tests
// =========================================================================
// All functions take world-space positions for each shape's center.
// For ColMesh, pos is the mesh's position offset (identity for static level).

/// Test AABB vs AABB collision.
NEA_ColResult NEA_ColTestAABBvsAABB(const NEA_ColAABB *a, NEA_Vec3 pos_a,
                                    const NEA_ColAABB *b, NEA_Vec3 pos_b);

/// Test Sphere vs Sphere collision.
NEA_ColResult NEA_ColTestSphereVsSphere(const NEA_ColSphere *a, NEA_Vec3 pos_a,
                                        const NEA_ColSphere *b, NEA_Vec3 pos_b);

/// Test AABB vs Sphere collision.
NEA_ColResult NEA_ColTestAABBvsSphere(const NEA_ColAABB *a, NEA_Vec3 pos_a,
                                      const NEA_ColSphere *b, NEA_Vec3 pos_b);

/// Test Capsule vs Sphere collision.
NEA_ColResult NEA_ColTestCapsuleVsSphere(const NEA_ColCapsule *a,
                                         NEA_Vec3 pos_a,
                                         const NEA_ColSphere *b,
                                         NEA_Vec3 pos_b);

/// Test Capsule vs AABB collision.
NEA_ColResult NEA_ColTestCapsuleVsAABB(const NEA_ColCapsule *a,
                                       NEA_Vec3 pos_a,
                                       const NEA_ColAABB *b, NEA_Vec3 pos_b);

/// Test Capsule vs Capsule collision.
NEA_ColResult NEA_ColTestCapsuleVsCapsule(const NEA_ColCapsule *a,
                                          NEA_Vec3 pos_a,
                                          const NEA_ColCapsule *b,
                                          NEA_Vec3 pos_b);

/// Test Sphere vs ColMesh collision.
NEA_ColResult NEA_ColTestSphereVsMesh(const NEA_ColSphere *a, NEA_Vec3 pos_a,
                                      const NEA_ColMesh *mesh,
                                      NEA_Vec3 mesh_pos);

/// Test AABB vs ColMesh collision.
NEA_ColResult NEA_ColTestAABBvsMesh(const NEA_ColAABB *a, NEA_Vec3 pos_a,
                                    const NEA_ColMesh *mesh,
                                    NEA_Vec3 mesh_pos);

/// Test Capsule vs ColMesh collision.
NEA_ColResult NEA_ColTestCapsuleVsMesh(const NEA_ColCapsule *a,
                                       NEA_Vec3 pos_a,
                                       const NEA_ColMesh *mesh,
                                       NEA_Vec3 mesh_pos);

/// Generic collision test dispatcher.
///
/// Tests any pair of collision shapes. Automatically selects the correct
/// narrow-phase algorithm based on shape types.
///
/// @param a First collision shape.
/// @param pos_a World-space position of shape A (f32).
/// @param b Second collision shape.
/// @param pos_b World-space position of shape B (f32).
/// @return Collision result with hit flag, normal, depth, and contact point.
NEA_ColResult NEA_ColTest(const NEA_ColShape *a, NEA_Vec3 pos_a,
                          const NEA_ColShape *b, NEA_Vec3 pos_b);

/// @}

#endif // NEA_COLLISION_H__
