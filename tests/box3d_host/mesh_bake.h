// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Warioware64, 2026

// Host-side mesh baking: a neutral triangle soup in, a fixed-point b3MeshData
// blob out.
//
// This is the conversion tools/obj2dl/b3mesh.py performs in Python, and this
// is the reference it is checked against: bake_ref puts this baker behind a
// command line so test_bake_diff.py can bake a corpus both ways and compare the
// bytes. It lives here rather than in tools/ because run_pair needs it anyway --
// the port has no mesh builder, so the only way to give it a level is to bake
// one -- which is what makes it the trustworthy side of that comparison: every
// run_pair mesh case and every test_world mode exercises it.
//
// The signature mentions only pair_iface.h types, so callers do not need the
// port's headers to ask for a bake.

#pragma once

#include "pair_iface.h"

/// Bake a triangle soup into the port's blob format.
///
/// @param desc  the mesh, described in doubles
/// @param blob  destination buffer, which must be 8-byte aligned
/// @param size  size of the destination buffer in bytes
/// @return the number of bytes written, or 0 if the mesh does not fit, does not
/// survive quantization, or fails validation once baked.
int pdBakeMesh( const pdMesh* desc, void* blob, int size );

/// An upper bound on the byte size a bake would need, without performing it.
///
/// Only a bound, unlike pdBakeHullSize: the exact size depends on how many
/// triangles survive the degeneracy test and how the tree ends up splitting,
/// and neither is known until the bake runs.
int pdBakeMeshSize( const pdMesh* desc );
