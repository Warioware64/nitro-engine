// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Warioware64, 2026

// Host-side hull baking: a neutral hull description in, a fixed-point
// b3HullData blob out.
//
// This is the conversion the eventual tools/hull2b3 will perform. It lives
// here first because run_pair needs it anyway -- the port has no hull builder,
// so the only way to give it a cylinder is to bake one -- and putting it here
// means every run_pair case exercises the baker as well as the collider.
//
// The signature mentions only pair_iface.h types, so callers do not need the
// port's headers to ask for a bake.

#pragma once

#include "pair_iface.h"

/// Bake a hull description into the port's blob format.
///
/// @param desc  the hull, described in doubles
/// @param blob  destination buffer, which must be 8-byte aligned
/// @param size  size of the destination buffer in bytes
/// @return the number of bytes written, or 0 if the hull does not fit, does
/// not survive quantization, or fails validation once baked.
int pdBakeHull( const pdHull* desc, void* blob, int size );

/// Byte size a bake would need, without performing it.
int pdBakeHullSize( const pdHull* desc );
