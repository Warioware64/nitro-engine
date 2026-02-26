// SPDX-License-Identifier: MIT
//
// Copyright (c) 2008-2022 Antonio Niño Díaz
// Copyright (c) 2024-2026 Warioware64
//
// This file is part of Nitro Engine Advanced

#include "NEAMain.h"

/// @file NEACollision.c

// =========================================================================
// .colmesh binary format constants
// =========================================================================

#define COLM_MAGIC   0x4D4C4F43  // "COLM" little-endian
#define COLM_VERSION 1

// .colmesh header layout
typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t num_triangles;
    uint32_t flags;
    int32_t  aabb_min[3]; // f32
    int32_t  aabb_max[3]; // f32
} colmesh_header_t;

// .colmesh per-triangle layout (12 x int32 = 48 bytes)
typedef struct {
    int32_t v0[3];
    int32_t v1[3];
    int32_t v2[3];
    int32_t normal[3];
} colmesh_triangle_t;

// =========================================================================
// Shape initialization
// =========================================================================

void NEA_ColShapeInitAABBI(NEA_ColShape *shape, int32_t hx, int32_t hy,
                           int32_t hz)
{
    NEA_AssertPointer(shape, "NULL shape pointer");
    shape->type = NEA_COL_AABB;
    shape->shape.aabb.half.x = hx;
    shape->shape.aabb.half.y = hy;
    shape->shape.aabb.half.z = hz;
}

void NEA_ColShapeInitSphereI(NEA_ColShape *shape, int32_t radius)
{
    NEA_AssertPointer(shape, "NULL shape pointer");
    NEA_Assert(radius >= 0, "Radius must be positive");
    shape->type = NEA_COL_SPHERE;
    shape->shape.sphere.radius = radius;
}

void NEA_ColShapeInitCapsuleI(NEA_ColShape *shape, int32_t half_height,
                              int32_t radius)
{
    NEA_AssertPointer(shape, "NULL shape pointer");
    NEA_Assert(half_height >= 0, "Half height must be positive");
    NEA_Assert(radius >= 0, "Radius must be positive");
    shape->type = NEA_COL_CAPSULE;
    shape->shape.capsule.half_height = half_height;
    shape->shape.capsule.radius = radius;
}

void NEA_ColShapeInitMesh(NEA_ColShape *shape, NEA_ColMesh *mesh)
{
    NEA_AssertPointer(shape, "NULL shape pointer");
    NEA_AssertPointer(mesh, "NULL mesh pointer");
    shape->type = NEA_COL_TRIMESH;
    shape->shape.mesh = mesh;
}

// =========================================================================
// ColMesh loading
// =========================================================================

static void ne_colmesh_compute_bounds(NEA_ColMesh *mesh)
{
    if (mesh->num_triangles == 0)
    {
        mesh->bounds.half = NEA_Vec3Make(0, 0, 0);
        mesh->center = NEA_Vec3Make(0, 0, 0);
        return;
    }

    int32_t min_x = INT32_MAX, min_y = INT32_MAX, min_z = INT32_MAX;
    int32_t max_x = INT32_MIN, max_y = INT32_MIN, max_z = INT32_MIN;

    NEA_ColTriangle *tris = mesh->triangles;
    for (int i = 0; i < mesh->num_triangles; i++)
    {
        NEA_Vec3 *verts[3] = { &tris[i].v0, &tris[i].v1, &tris[i].v2 };
        for (int j = 0; j < 3; j++)
        {
            if (verts[j]->x < min_x) min_x = verts[j]->x;
            if (verts[j]->y < min_y) min_y = verts[j]->y;
            if (verts[j]->z < min_z) min_z = verts[j]->z;
            if (verts[j]->x > max_x) max_x = verts[j]->x;
            if (verts[j]->y > max_y) max_y = verts[j]->y;
            if (verts[j]->z > max_z) max_z = verts[j]->z;
        }
    }

    // Store center and half-extents
    mesh->center.x = (min_x + max_x) >> 1;
    mesh->center.y = (min_y + max_y) >> 1;
    mesh->center.z = (min_z + max_z) >> 1;
    mesh->bounds.half.x = (max_x - min_x) >> 1;
    mesh->bounds.half.y = (max_y - min_y) >> 1;
    mesh->bounds.half.z = (max_z - min_z) >> 1;
}

NEA_ColMesh *NEA_ColMeshLoad(const void *data)
{
    NEA_AssertPointer(data, "NULL data pointer");

    const colmesh_header_t *hdr = (const colmesh_header_t *)data;

    if (hdr->magic != COLM_MAGIC)
    {
        NEA_DebugPrint("Invalid .colmesh magic");
        return NULL;
    }
    if (hdr->version != COLM_VERSION)
    {
        NEA_DebugPrint("Unsupported .colmesh version");
        return NULL;
    }

    uint32_t num_tris = hdr->num_triangles;

    NEA_ColMesh *mesh = calloc(1, sizeof(NEA_ColMesh));
    if (mesh == NULL)
    {
        NEA_DebugPrint("Not enough memory for ColMesh");
        return NULL;
    }

    mesh->num_triangles = (uint16_t)num_tris;
    mesh->flags = NEA_COLMESH_STATIC;
    mesh->world_tris = NULL;

    if (num_tris > 0)
    {
        mesh->triangles = malloc(num_tris * sizeof(NEA_ColTriangle));
        if (mesh->triangles == NULL)
        {
            NEA_DebugPrint("Not enough memory for triangles");
            free(mesh);
            return NULL;
        }

        const colmesh_triangle_t *src =
            (const colmesh_triangle_t *)((const uint8_t *)data
                                         + sizeof(colmesh_header_t));

        for (uint32_t i = 0; i < num_tris; i++)
        {
            mesh->triangles[i].v0 = NEA_Vec3Make(src[i].v0[0], src[i].v0[1],
                                                  src[i].v0[2]);
            mesh->triangles[i].v1 = NEA_Vec3Make(src[i].v1[0], src[i].v1[1],
                                                  src[i].v1[2]);
            mesh->triangles[i].v2 = NEA_Vec3Make(src[i].v2[0], src[i].v2[1],
                                                  src[i].v2[2]);
            mesh->triangles[i].normal = NEA_Vec3Make(src[i].normal[0],
                                                     src[i].normal[1],
                                                     src[i].normal[2]);
        }
    }

    // Compute bounding AABB center and half-extents from file min/max
    mesh->center.x = (hdr->aabb_min[0] + hdr->aabb_max[0]) >> 1;
    mesh->center.y = (hdr->aabb_min[1] + hdr->aabb_max[1]) >> 1;
    mesh->center.z = (hdr->aabb_min[2] + hdr->aabb_max[2]) >> 1;
    mesh->bounds.half.x = (hdr->aabb_max[0] - hdr->aabb_min[0]) >> 1;
    mesh->bounds.half.y = (hdr->aabb_max[1] - hdr->aabb_min[1]) >> 1;
    mesh->bounds.half.z = (hdr->aabb_max[2] - hdr->aabb_min[2]) >> 1;

    return mesh;
}

NEA_ColMesh *NEA_ColMeshLoadFAT(const char *path)
{
    NEA_AssertPointer(path, "NULL path");

    FILE *f = fopen(path, "rb");
    if (f == NULL)
    {
        NEA_DebugPrint("Can't open %s", path);
        return NULL;
    }

    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    fseek(f, 0, SEEK_SET);

    void *data = malloc(size);
    if (data == NULL)
    {
        NEA_DebugPrint("Not enough memory");
        fclose(f);
        return NULL;
    }

    fread(data, 1, size, f);
    fclose(f);

    NEA_ColMesh *mesh = NEA_ColMeshLoad(data);
    free(data);
    return mesh;
}

void NEA_ColMeshSetDynamic(NEA_ColMesh *mesh, bool dynamic)
{
    NEA_AssertPointer(mesh, "NULL mesh pointer");

    if (dynamic && !(mesh->flags & NEA_COLMESH_DYNAMIC))
    {
        // Allocate world-space triangle buffer
        mesh->world_tris = malloc(mesh->num_triangles
                                  * sizeof(NEA_ColTriangle));
        if (mesh->world_tris == NULL)
        {
            NEA_DebugPrint("Not enough memory for dynamic ColMesh");
            return;
        }
        // Initialize to local-space copy
        memcpy(mesh->world_tris, mesh->triangles,
               mesh->num_triangles * sizeof(NEA_ColTriangle));
        mesh->flags |= NEA_COLMESH_DYNAMIC;
    }
    else if (!dynamic && (mesh->flags & NEA_COLMESH_DYNAMIC))
    {
        free(mesh->world_tris);
        mesh->world_tris = NULL;
        mesh->flags &= ~NEA_COLMESH_DYNAMIC;
        // Recompute bounds from original triangles
        ne_colmesh_compute_bounds(mesh);
    }
}

// Transform a single Vec3 by a 4x3 matrix (rotation+scale+translation).
// m4x3.m layout: [r0c0 r1c0 r2c0  r0c1 r1c1 r2c1  r0c2 r1c2 r2c2
//                  tx   ty   tz]
// This matches the libnds m4x3 layout (column-major 4x3).
static inline NEA_Vec3 ne_vec3_transform(NEA_Vec3 v, const m4x3 *mat)
{
    NEA_Vec3 r;
    r.x = mulf32(v.x, mat->m[0]) + mulf32(v.y, mat->m[3])
        + mulf32(v.z, mat->m[6]) + mat->m[9];
    r.y = mulf32(v.x, mat->m[1]) + mulf32(v.y, mat->m[4])
        + mulf32(v.z, mat->m[7]) + mat->m[10];
    r.z = mulf32(v.x, mat->m[2]) + mulf32(v.y, mat->m[5])
        + mulf32(v.z, mat->m[8]) + mat->m[11];
    return r;
}

// Transform a direction vector (no translation).
static inline NEA_Vec3 ne_vec3_rotate(NEA_Vec3 v, const m4x3 *mat)
{
    NEA_Vec3 r;
    r.x = mulf32(v.x, mat->m[0]) + mulf32(v.y, mat->m[3])
        + mulf32(v.z, mat->m[6]);
    r.y = mulf32(v.x, mat->m[1]) + mulf32(v.y, mat->m[4])
        + mulf32(v.z, mat->m[7]);
    r.z = mulf32(v.x, mat->m[2]) + mulf32(v.y, mat->m[5])
        + mulf32(v.z, mat->m[8]);
    return r;
}

void NEA_ColMeshUpdateTransform(NEA_ColMesh *mesh, const m4x3 *matrix)
{
    NEA_AssertPointer(mesh, "NULL mesh pointer");
    NEA_AssertPointer(matrix, "NULL matrix pointer");

    if (!(mesh->flags & NEA_COLMESH_DYNAMIC) || mesh->world_tris == NULL)
    {
        NEA_DebugPrint("ColMesh is not set to dynamic mode");
        return;
    }

    for (int i = 0; i < mesh->num_triangles; i++)
    {
        mesh->world_tris[i].v0 = ne_vec3_transform(mesh->triangles[i].v0,
                                                    matrix);
        mesh->world_tris[i].v1 = ne_vec3_transform(mesh->triangles[i].v1,
                                                    matrix);
        mesh->world_tris[i].v2 = ne_vec3_transform(mesh->triangles[i].v2,
                                                    matrix);
        // Rotate normal (no translation), then re-normalize
        NEA_Vec3 n = ne_vec3_rotate(mesh->triangles[i].normal, matrix);
        mesh->world_tris[i].normal = NEA_Vec3Normalize(n);
    }

    // Recompute bounding AABB from transformed triangles
    NEA_ColTriangle *old_tris = mesh->triangles;
    mesh->triangles = mesh->world_tris;
    ne_colmesh_compute_bounds(mesh);
    mesh->triangles = old_tris;
}

void NEA_ColMeshFree(NEA_ColMesh *mesh)
{
    if (mesh == NULL)
        return;

    free(mesh->triangles);
    free(mesh->world_tris);
    free(mesh);
}

// =========================================================================
// Narrow-phase collision tests
// =========================================================================

// --- AABB vs AABB ---

NEA_ColResult NEA_ColTestAABBvsAABB(const NEA_ColAABB *a, NEA_Vec3 pos_a,
                                    const NEA_ColAABB *b, NEA_Vec3 pos_b)
{
    NEA_ColResult r = { .hit = false };
    NEA_Vec3 d = NEA_Vec3Sub(pos_b, pos_a);

    int32_t overlap_x = (a->half.x + b->half.x) - abs(d.x);
    if (overlap_x <= 0)
        return r;
    int32_t overlap_y = (a->half.y + b->half.y) - abs(d.y);
    if (overlap_y <= 0)
        return r;
    int32_t overlap_z = (a->half.z + b->half.z) - abs(d.z);
    if (overlap_z <= 0)
        return r;

    r.hit = true;

    // Minimum overlap axis becomes the collision normal
    if (overlap_x <= overlap_y && overlap_x <= overlap_z)
    {
        r.depth = overlap_x;
        r.normal = NEA_Vec3Make(d.x >= 0 ? inttof32(1) : inttof32(-1), 0, 0);
    }
    else if (overlap_y <= overlap_z)
    {
        r.depth = overlap_y;
        r.normal = NEA_Vec3Make(0, d.y >= 0 ? inttof32(1) : inttof32(-1), 0);
    }
    else
    {
        r.depth = overlap_z;
        r.normal = NEA_Vec3Make(0, 0, d.z >= 0 ? inttof32(1) : inttof32(-1));
    }

    // Contact point: midpoint of overlap along collision axis
    r.point = NEA_Vec3Make((pos_a.x + pos_b.x) >> 1,
                           (pos_a.y + pos_b.y) >> 1,
                           (pos_a.z + pos_b.z) >> 1);
    return r;
}

// --- Sphere vs Sphere ---

NEA_ColResult NEA_ColTestSphereVsSphere(const NEA_ColSphere *a,
                                        NEA_Vec3 pos_a,
                                        const NEA_ColSphere *b,
                                        NEA_Vec3 pos_b)
{
    NEA_ColResult r = { .hit = false };
    NEA_Vec3 d = NEA_Vec3Sub(pos_b, pos_a);

    // Compare squared distances to avoid sqrt in the common non-colliding case
    int64_t dist_sq_64 = (int64_t)d.x * d.x + (int64_t)d.y * d.y
                       + (int64_t)d.z * d.z;
    int32_t sum_r = a->radius + b->radius;
    int64_t sum_r_sq = (int64_t)sum_r * sum_r;

    if (dist_sq_64 >= sum_r_sq)
        return r;

    r.hit = true;

    // dist_sq_64 is f32*f32 = 24 fractional bits. sqrt64 yields 12 frac
    // bits, which is already f32 format — no additional shift needed.
    uint32_t dist_f32 = (uint32_t)sqrt64((uint64_t)dist_sq_64);

    if (dist_f32 > 0)
    {
        r.depth = sum_r - (int32_t)dist_f32;
        // Normalize: n = d / dist
        r.normal.x = divf32(d.x, (int32_t)dist_f32);
        r.normal.y = divf32(d.y, (int32_t)dist_f32);
        r.normal.z = divf32(d.z, (int32_t)dist_f32);
    }
    else
    {
        // Degenerate: centers overlap exactly
        r.depth = sum_r;
        r.normal = NEA_Vec3Make(0, inttof32(1), 0);
    }

    // Contact point: on the surface of sphere A in the direction of B
    r.point = NEA_Vec3Add(pos_a, NEA_Vec3Scale(r.normal, a->radius));
    return r;
}

// --- AABB vs Sphere ---

NEA_ColResult NEA_ColTestAABBvsSphere(const NEA_ColAABB *a, NEA_Vec3 pos_a,
                                      const NEA_ColSphere *b, NEA_Vec3 pos_b)
{
    NEA_ColResult r = { .hit = false };

    // Find the closest point on the AABB to the sphere center
    NEA_Vec3 closest;
    closest.x = NEA_Clamp(pos_b.x, pos_a.x - a->half.x,
                           pos_a.x + a->half.x);
    closest.y = NEA_Clamp(pos_b.y, pos_a.y - a->half.y,
                           pos_a.y + a->half.y);
    closest.z = NEA_Clamp(pos_b.z, pos_a.z - a->half.z,
                           pos_a.z + a->half.z);

    NEA_Vec3 d = NEA_Vec3Sub(pos_b, closest);
    int64_t dist_sq_64 = (int64_t)d.x * d.x + (int64_t)d.y * d.y
                       + (int64_t)d.z * d.z;
    int64_t r_sq = (int64_t)b->radius * b->radius;

    if (dist_sq_64 >= r_sq)
        return r;

    r.hit = true;
    r.point = closest;

    uint32_t dist_f32 = (uint32_t)sqrt64((uint64_t)dist_sq_64);

    if (dist_f32 > 0)
    {
        r.depth = b->radius - (int32_t)dist_f32;
        r.normal.x = divf32(d.x, (int32_t)dist_f32);
        r.normal.y = divf32(d.y, (int32_t)dist_f32);
        r.normal.z = divf32(d.z, (int32_t)dist_f32);
    }
    else
    {
        // Sphere center is inside the AABB — find minimum push axis
        int32_t dx_pos = (pos_a.x + a->half.x) - pos_b.x;
        int32_t dx_neg = pos_b.x - (pos_a.x - a->half.x);
        int32_t dy_pos = (pos_a.y + a->half.y) - pos_b.y;
        int32_t dy_neg = pos_b.y - (pos_a.y - a->half.y);
        int32_t dz_pos = (pos_a.z + a->half.z) - pos_b.z;
        int32_t dz_neg = pos_b.z - (pos_a.z - a->half.z);

        int32_t min_d = dx_pos;
        r.normal = NEA_Vec3Make(inttof32(1), 0, 0);
        r.depth = min_d + b->radius;

        if (dx_neg < min_d) {
            min_d = dx_neg;
            r.normal = NEA_Vec3Make(inttof32(-1), 0, 0);
            r.depth = min_d + b->radius;
        }
        if (dy_pos < min_d) {
            min_d = dy_pos;
            r.normal = NEA_Vec3Make(0, inttof32(1), 0);
            r.depth = min_d + b->radius;
        }
        if (dy_neg < min_d) {
            min_d = dy_neg;
            r.normal = NEA_Vec3Make(0, inttof32(-1), 0);
            r.depth = min_d + b->radius;
        }
        if (dz_pos < min_d) {
            min_d = dz_pos;
            r.normal = NEA_Vec3Make(0, 0, inttof32(1));
            r.depth = min_d + b->radius;
        }
        if (dz_neg < min_d) {
            min_d = dz_neg;
            r.normal = NEA_Vec3Make(0, 0, inttof32(-1));
            r.depth = min_d + b->radius;
        }
    }

    return r;
}

// =========================================================================
// Capsule helper: closest point on a Y-axis segment to a point
// =========================================================================

// Returns the closest point on the segment from
// (pos.x, pos.y - half_h, pos.z) to (pos.x, pos.y + half_h, pos.z)
// to the target point.
static inline NEA_Vec3 ne_closest_point_on_segment_y(NEA_Vec3 pos,
                                                     int32_t half_h,
                                                     NEA_Vec3 target)
{
    // Clamp the Y component to the segment range
    int32_t y = NEA_Clamp(target.y, pos.y - half_h, pos.y + half_h);
    return NEA_Vec3Make(pos.x, y, pos.z);
}

// --- Capsule vs Sphere ---

NEA_ColResult NEA_ColTestCapsuleVsSphere(const NEA_ColCapsule *a,
                                         NEA_Vec3 pos_a,
                                         const NEA_ColSphere *b,
                                         NEA_Vec3 pos_b)
{
    // Find closest point on capsule segment to sphere center
    NEA_Vec3 closest = ne_closest_point_on_segment_y(pos_a, a->half_height,
                                                     pos_b);

    // Now treat as sphere-vs-sphere from closest point
    NEA_ColSphere cap_sphere = { .radius = a->radius };
    return NEA_ColTestSphereVsSphere(&cap_sphere, closest, b, pos_b);
}

// --- Capsule vs AABB ---

NEA_ColResult NEA_ColTestCapsuleVsAABB(const NEA_ColCapsule *a,
                                       NEA_Vec3 pos_a,
                                       const NEA_ColAABB *b, NEA_Vec3 pos_b)
{
    // Find closest point on capsule's Y-axis segment to the AABB center
    NEA_Vec3 seg_point = ne_closest_point_on_segment_y(pos_a, a->half_height,
                                                       pos_b);

    // Treat as sphere-vs-AABB from the closest segment point
    NEA_ColSphere cap_sphere = { .radius = a->radius };
    NEA_ColResult r = NEA_ColTestAABBvsSphere(b, pos_b, &cap_sphere, seg_point);

    // Flip normal (AABBvsSphere returns normal from AABB to sphere)
    // We want normal from capsule (A) to AABB (B)
    r.normal = NEA_Vec3Neg(r.normal);
    return r;
}

// =========================================================================
// Capsule vs Capsule: closest points between two Y-axis segments
// =========================================================================

NEA_ColResult NEA_ColTestCapsuleVsCapsule(const NEA_ColCapsule *a,
                                          NEA_Vec3 pos_a,
                                          const NEA_ColCapsule *b,
                                          NEA_Vec3 pos_b)
{
    // Capsule A segment: (pos_a.x, pos_a.y ± a->half_height, pos_a.z)
    // Capsule B segment: (pos_b.x, pos_b.y ± b->half_height, pos_b.z)
    // Both segments are along Y axis, so closest points are found by
    // clamping Y coordinates.

    // Clamp A's Y to B's segment range and vice versa
    int32_t a_y = NEA_Clamp(pos_b.y, pos_a.y - a->half_height,
                             pos_a.y + a->half_height);
    int32_t b_y = NEA_Clamp(a_y, pos_b.y - b->half_height,
                             pos_b.y + b->half_height);
    // Re-clamp A to the clamped B
    a_y = NEA_Clamp(b_y, pos_a.y - a->half_height,
                     pos_a.y + a->half_height);

    NEA_Vec3 closest_a = NEA_Vec3Make(pos_a.x, a_y, pos_a.z);
    NEA_Vec3 closest_b = NEA_Vec3Make(pos_b.x, b_y, pos_b.z);

    NEA_ColSphere sa = { .radius = a->radius };
    NEA_ColSphere sb = { .radius = b->radius };
    return NEA_ColTestSphereVsSphere(&sa, closest_a, &sb, closest_b);
}

// =========================================================================
// Sphere vs Triangle helper (for ColMesh tests)
// =========================================================================

// Test a sphere against a single triangle. Returns collision result.
static NEA_ColResult ne_sphere_vs_triangle(NEA_Vec3 center, int32_t radius,
                                           const NEA_ColTriangle *tri)
{
    NEA_ColResult r = { .hit = false };

    // Distance from sphere center to triangle plane
    NEA_Vec3 v0_to_center = NEA_Vec3Sub(center, tri->v0);
    int32_t dist_to_plane = NEA_Vec3Dot(v0_to_center, tri->normal);

    // Quick reject: too far from plane
    if (abs(dist_to_plane) > radius)
        return r;

    // Project center onto triangle plane
    NEA_Vec3 projected = NEA_Vec3Sub(center,
                                     NEA_Vec3Scale(tri->normal,
                                                   dist_to_plane));

    // Check if projected point is inside triangle using barycentric coords
    NEA_Vec3 e0 = NEA_Vec3Sub(tri->v1, tri->v0);
    NEA_Vec3 e1 = NEA_Vec3Sub(tri->v2, tri->v0);
    NEA_Vec3 e2 = NEA_Vec3Sub(projected, tri->v0);

    int32_t d00 = NEA_Vec3Dot(e0, e0);
    int32_t d01 = NEA_Vec3Dot(e0, e1);
    int32_t d02 = NEA_Vec3Dot(e0, e2);
    int32_t d11 = NEA_Vec3Dot(e1, e1);
    int32_t d12 = NEA_Vec3Dot(e1, e2);

    // denom = d00*d11 - d01*d01
    int64_t denom_64 = (int64_t)d00 * d11 - (int64_t)d01 * d01;
    if (denom_64 == 0)
        return r; // Degenerate triangle

    // u = (d11*d02 - d01*d12) / denom
    // v = (d00*d12 - d01*d02) / denom
    int64_t u_num = (int64_t)d11 * d02 - (int64_t)d01 * d12;
    int64_t v_num = (int64_t)d00 * d12 - (int64_t)d01 * d02;

    bool inside = false;
    if (denom_64 > 0)
        inside = (u_num >= 0) && (v_num >= 0)
              && (u_num + v_num <= denom_64);
    else
        inside = (u_num <= 0) && (v_num <= 0)
              && (u_num + v_num >= denom_64);

    if (inside)
    {
        // Closest point is the projection on the plane
        r.hit = true;
        // Normal points from sphere toward triangle (first arg toward second)
        r.normal = NEA_Vec3Neg(tri->normal);
        if (dist_to_plane < 0)
            r.normal = tri->normal;
        r.depth = radius - abs(dist_to_plane);
        r.point = projected;
        return r;
    }

    // Not inside triangle — find closest point on triangle edges
    // Check all 3 edges and pick the closest

    NEA_Vec3 edges_start[3] = { tri->v0, tri->v1, tri->v2 };
    NEA_Vec3 edges_end[3]   = { tri->v1, tri->v2, tri->v0 };

    int64_t best_dist_sq_64 = INT64_MAX;
    NEA_Vec3 best_point = tri->v0;

    for (int i = 0; i < 3; i++)
    {
        NEA_Vec3 edge = NEA_Vec3Sub(edges_end[i], edges_start[i]);
        int32_t edge_len_sq = NEA_Vec3Dot(edge, edge);

        if (edge_len_sq == 0)
            continue;

        NEA_Vec3 to_center = NEA_Vec3Sub(center, edges_start[i]);
        int32_t t_num = NEA_Vec3Dot(to_center, edge);

        // t = t_num / edge_len_sq, clamped to [0, 1]
        NEA_Vec3 closest;
        if (t_num <= 0)
        {
            closest = edges_start[i];
        }
        else if (t_num >= edge_len_sq)
        {
            closest = edges_end[i];
        }
        else
        {
            // closest = start + edge * (t_num / edge_len_sq)
            int32_t t = divf32(t_num, edge_len_sq);
            closest = NEA_Vec3Add(edges_start[i], NEA_Vec3Scale(edge, t));
        }

        NEA_Vec3 diff = NEA_Vec3Sub(center, closest);
        // Use 64-bit squared distance (24 fractional bits) to avoid
        // precision loss from dotf32's >> 12 truncation.
        int64_t dist_sq_64 = (int64_t)diff.x * diff.x
                           + (int64_t)diff.y * diff.y
                           + (int64_t)diff.z * diff.z;

        if (dist_sq_64 < best_dist_sq_64)
        {
            best_dist_sq_64 = dist_sq_64;
            best_point = closest;
        }
    }

    // Compare in 64-bit (24 frac bits) to match precision
    int64_t r_sq_64 = (int64_t)radius * radius;
    if (best_dist_sq_64 >= r_sq_64)
        return r;

    // Normal points from sphere toward triangle (first arg toward second)
    NEA_Vec3 diff = NEA_Vec3Sub(best_point, center);
    // best_dist_sq_64 has 24 fractional bits. sqrt64 yields 12 frac
    // bits, which is already f32 format — no additional shift needed.
    uint32_t dist_f32 = (uint32_t)sqrt64((uint64_t)best_dist_sq_64);

    if (dist_f32 > 0)
    {
        r.hit = true;
        r.depth = radius - (int32_t)dist_f32;
        r.normal.x = divf32(diff.x, (int32_t)dist_f32);
        r.normal.y = divf32(diff.y, (int32_t)dist_f32);
        r.normal.z = divf32(diff.z, (int32_t)dist_f32);
        r.point = best_point;
    }

    return r;
}

// =========================================================================
// Shape vs ColMesh tests
// =========================================================================

// Get the effective triangle array (world_tris if dynamic, triangles otherwise)
static inline const NEA_ColTriangle *ne_colmesh_get_tris(
    const NEA_ColMesh *mesh)
{
    if ((mesh->flags & NEA_COLMESH_DYNAMIC) && mesh->world_tris != NULL)
        return mesh->world_tris;
    return mesh->triangles;
}

// AABB early rejection: test if a shape's AABB overlaps the mesh's AABB
static inline bool ne_aabb_overlap_check(NEA_Vec3 shape_pos,
                                         NEA_Vec3 shape_half,
                                         const NEA_ColMesh *mesh,
                                         NEA_Vec3 mesh_pos)
{
    // Account for mesh AABB center offset from mesh origin
    NEA_Vec3 mesh_center = NEA_Vec3Add(mesh_pos, mesh->center);
    NEA_Vec3 d = NEA_Vec3Sub(shape_pos, mesh_center);
    NEA_Vec3 sum_half = NEA_Vec3Add(shape_half, mesh->bounds.half);

    return (abs(d.x) < sum_half.x)
        && (abs(d.y) < sum_half.y)
        && (abs(d.z) < sum_half.z);
}

NEA_ColResult NEA_ColTestSphereVsMesh(const NEA_ColSphere *a, NEA_Vec3 pos_a,
                                      const NEA_ColMesh *mesh,
                                      NEA_Vec3 mesh_pos)
{
    NEA_ColResult r = { .hit = false };

    // AABB early rejection
    NEA_Vec3 sphere_half = NEA_Vec3Make(a->radius, a->radius, a->radius);
    if (!ne_aabb_overlap_check(pos_a, sphere_half, mesh, mesh_pos))
        return r;

    const NEA_ColTriangle *tris = ne_colmesh_get_tris(mesh);

    // For static meshes, offset sphere position by -mesh_pos to work in
    // local space. For dynamic meshes, triangles are already in world space.
    NEA_Vec3 local_pos = pos_a;
    if (!(mesh->flags & NEA_COLMESH_DYNAMIC))
        local_pos = NEA_Vec3Sub(pos_a, mesh_pos);

    NEA_ColResult best = { .hit = false, .depth = 0 };

    for (int i = 0; i < mesh->num_triangles; i++)
    {
        NEA_ColResult tri_r = ne_sphere_vs_triangle(local_pos, a->radius,
                                                    &tris[i]);
        if (tri_r.hit && tri_r.depth > best.depth)
        {
            best = tri_r;
        }
    }

    if (best.hit && !(mesh->flags & NEA_COLMESH_DYNAMIC))
    {
        // Convert contact point back to world space
        best.point = NEA_Vec3Add(best.point, mesh_pos);
    }

    return best;
}

NEA_ColResult NEA_ColTestAABBvsMesh(const NEA_ColAABB *a, NEA_Vec3 pos_a,
                                    const NEA_ColMesh *mesh,
                                    NEA_Vec3 mesh_pos)
{
    NEA_ColResult r = { .hit = false };

    // AABB early rejection
    if (!ne_aabb_overlap_check(pos_a, a->half, mesh, mesh_pos))
        return r;

    // Approximate AABB-vs-mesh by treating AABB as a sphere with radius
    // equal to the smallest half-extent. This is a simplification but
    // avoids the complexity of full SAT on arbitrary triangles.
    int32_t min_half = a->half.x;
    if (a->half.y < min_half) min_half = a->half.y;
    if (a->half.z < min_half) min_half = a->half.z;

    NEA_ColSphere approx = { .radius = min_half };
    return NEA_ColTestSphereVsMesh(&approx, pos_a, mesh, mesh_pos);
}

NEA_ColResult NEA_ColTestCapsuleVsMesh(const NEA_ColCapsule *a,
                                       NEA_Vec3 pos_a,
                                       const NEA_ColMesh *mesh,
                                       NEA_Vec3 mesh_pos)
{
    NEA_ColResult r = { .hit = false };

    // AABB early rejection with capsule's bounding AABB
    NEA_Vec3 cap_half = NEA_Vec3Make(a->radius,
                                     a->half_height + a->radius,
                                     a->radius);
    if (!ne_aabb_overlap_check(pos_a, cap_half, mesh, mesh_pos))
        return r;

    const NEA_ColTriangle *tris = ne_colmesh_get_tris(mesh);

    NEA_Vec3 local_pos = pos_a;
    if (!(mesh->flags & NEA_COLMESH_DYNAMIC))
        local_pos = NEA_Vec3Sub(pos_a, mesh_pos);

    NEA_ColResult best = { .hit = false, .depth = 0 };

    // For each triangle, find closest point on capsule segment to triangle,
    // then test sphere at that point against the triangle.
    for (int i = 0; i < mesh->num_triangles; i++)
    {
        // Find approximate closest Y on segment to triangle center
        NEA_Vec3 tri_center = NEA_Vec3Make(
            (tris[i].v0.x + tris[i].v1.x + tris[i].v2.x) / 3,
            (tris[i].v0.y + tris[i].v1.y + tris[i].v2.y) / 3,
            (tris[i].v0.z + tris[i].v1.z + tris[i].v2.z) / 3);

        NEA_Vec3 seg_point = ne_closest_point_on_segment_y(
            local_pos, a->half_height, tri_center);

        NEA_ColResult tri_r = ne_sphere_vs_triangle(seg_point, a->radius,
                                                    &tris[i]);
        if (tri_r.hit && tri_r.depth > best.depth)
        {
            best = tri_r;
        }
    }

    if (best.hit && !(mesh->flags & NEA_COLMESH_DYNAMIC))
    {
        best.point = NEA_Vec3Add(best.point, mesh_pos);
    }

    return best;
}

// =========================================================================
// Generic collision dispatcher
// =========================================================================

NEA_ColResult NEA_ColTest(const NEA_ColShape *a, NEA_Vec3 pos_a,
                          const NEA_ColShape *b, NEA_Vec3 pos_b)
{
    NEA_ColResult r = { .hit = false };

    if (a->type == NEA_COL_NONE || b->type == NEA_COL_NONE)
        return r;

    // Dispatch based on shape pair
    // For non-symmetric pairs, we call with a specific argument order
    // and negate the normal if needed.

    // AABB vs AABB
    if (a->type == NEA_COL_AABB && b->type == NEA_COL_AABB)
    {
        return NEA_ColTestAABBvsAABB(&a->shape.aabb, pos_a,
                                     &b->shape.aabb, pos_b);
    }

    // Sphere vs Sphere
    if (a->type == NEA_COL_SPHERE && b->type == NEA_COL_SPHERE)
    {
        return NEA_ColTestSphereVsSphere(&a->shape.sphere, pos_a,
                                         &b->shape.sphere, pos_b);
    }

    // AABB vs Sphere (and reverse)
    if (a->type == NEA_COL_AABB && b->type == NEA_COL_SPHERE)
    {
        return NEA_ColTestAABBvsSphere(&a->shape.aabb, pos_a,
                                       &b->shape.sphere, pos_b);
    }
    if (a->type == NEA_COL_SPHERE && b->type == NEA_COL_AABB)
    {
        r = NEA_ColTestAABBvsSphere(&b->shape.aabb, pos_b,
                                    &a->shape.sphere, pos_a);
        r.normal = NEA_Vec3Neg(r.normal);
        return r;
    }

    // Capsule vs Sphere (and reverse)
    if (a->type == NEA_COL_CAPSULE && b->type == NEA_COL_SPHERE)
    {
        return NEA_ColTestCapsuleVsSphere(&a->shape.capsule, pos_a,
                                          &b->shape.sphere, pos_b);
    }
    if (a->type == NEA_COL_SPHERE && b->type == NEA_COL_CAPSULE)
    {
        r = NEA_ColTestCapsuleVsSphere(&b->shape.capsule, pos_b,
                                       &a->shape.sphere, pos_a);
        r.normal = NEA_Vec3Neg(r.normal);
        return r;
    }

    // Capsule vs AABB (and reverse)
    if (a->type == NEA_COL_CAPSULE && b->type == NEA_COL_AABB)
    {
        return NEA_ColTestCapsuleVsAABB(&a->shape.capsule, pos_a,
                                        &b->shape.aabb, pos_b);
    }
    if (a->type == NEA_COL_AABB && b->type == NEA_COL_CAPSULE)
    {
        r = NEA_ColTestCapsuleVsAABB(&b->shape.capsule, pos_b,
                                     &a->shape.aabb, pos_a);
        r.normal = NEA_Vec3Neg(r.normal);
        return r;
    }

    // Capsule vs Capsule
    if (a->type == NEA_COL_CAPSULE && b->type == NEA_COL_CAPSULE)
    {
        return NEA_ColTestCapsuleVsCapsule(&a->shape.capsule, pos_a,
                                           &b->shape.capsule, pos_b);
    }

    // Sphere vs ColMesh (and reverse)
    if (a->type == NEA_COL_SPHERE && b->type == NEA_COL_TRIMESH)
    {
        return NEA_ColTestSphereVsMesh(&a->shape.sphere, pos_a,
                                       b->shape.mesh, pos_b);
    }
    if (a->type == NEA_COL_TRIMESH && b->type == NEA_COL_SPHERE)
    {
        r = NEA_ColTestSphereVsMesh(&b->shape.sphere, pos_b,
                                    a->shape.mesh, pos_a);
        r.normal = NEA_Vec3Neg(r.normal);
        return r;
    }

    // AABB vs ColMesh (and reverse)
    if (a->type == NEA_COL_AABB && b->type == NEA_COL_TRIMESH)
    {
        return NEA_ColTestAABBvsMesh(&a->shape.aabb, pos_a,
                                     b->shape.mesh, pos_b);
    }
    if (a->type == NEA_COL_TRIMESH && b->type == NEA_COL_AABB)
    {
        r = NEA_ColTestAABBvsMesh(&b->shape.aabb, pos_b,
                                  a->shape.mesh, pos_a);
        r.normal = NEA_Vec3Neg(r.normal);
        return r;
    }

    // Capsule vs ColMesh (and reverse)
    if (a->type == NEA_COL_CAPSULE && b->type == NEA_COL_TRIMESH)
    {
        return NEA_ColTestCapsuleVsMesh(&a->shape.capsule, pos_a,
                                        b->shape.mesh, pos_b);
    }
    if (a->type == NEA_COL_TRIMESH && b->type == NEA_COL_CAPSULE)
    {
        r = NEA_ColTestCapsuleVsMesh(&b->shape.capsule, pos_b,
                                     a->shape.mesh, pos_a);
        r.normal = NEA_Vec3Neg(r.normal);
        return r;
    }

    // ColMesh vs ColMesh: not supported
    return r;
}
