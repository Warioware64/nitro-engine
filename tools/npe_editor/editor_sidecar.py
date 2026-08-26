#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
#
# Copyright (c) 2026 Warioware64
#
# Editor-only state that lives beside a binary asset.

"""editor_sidecar.py -- read and write a `<asset>.editor.json` sidecar.

The NPE and AnimMat binaries are read by the C runtime, which has no use for a
preview image and no field to put one in. Both formats reference their texture
indirectly -- NPE by a material name resolved at run time, an AnimMat target by
the name of a material the model supplies -- so the editor has a name and no
picture, and nowhere in the file to record one.

Rather than grow the formats for something only the editor cares about, that
state goes in a sidecar next to the asset:

    panels.bin  ->  panels.bin.editor.json

The sidecar is advisory in the strongest sense. Losing it, or the images it
points at, costs the preview and nothing else: every reader here returns a
default instead of raising, because a missing preview must never stand between
someone and their file.

Image paths are stored relative to the sidecar when that is possible, so moving
a project keeps them working. An absolute path is written when the image lives
somewhere a relative path cannot reach, and both forms are accepted on read.

This module only handles the file and the paths; each editor owns the shape of
what it stores inside.
"""

import json
import os

SIDECAR_SUFFIX = ".editor.json"
VERSION = 1


def sidecar_path(asset_path):
    """Where the sidecar for this asset lives."""
    return asset_path + SIDECAR_SUFFIX


def load(asset_path):
    """Read the sidecar for an asset.

    Returns an empty dict when there is nothing to read or the file is damaged.
    A corrupt sidecar is treated exactly like a missing one: it holds only
    preview settings, and refusing to open a file over them would be absurd.
    """
    if not asset_path:
        return {}

    path = sidecar_path(asset_path)
    try:
        with open(path, "r") as f:
            data = json.load(f)
    except (OSError, ValueError):
        return {}

    if not isinstance(data, dict):
        return {}

    return data


def save(asset_path, data):
    """Write the sidecar for an asset.

    Returns True on success. A failure is reported rather than raised, for the
    same reason: the asset itself has already been saved by the time this runs,
    and the sidecar failing is not a reason to tell someone their work is lost.
    """
    if not asset_path:
        return False

    out = dict(data)
    out["version"] = VERSION

    try:
        with open(sidecar_path(asset_path), "w") as f:
            json.dump(out, f, indent=2)
            f.write("\n")
    except OSError:
        return False

    return True


def delete(asset_path):
    """Remove an asset's sidecar, if it has one."""
    if not asset_path:
        return

    try:
        os.remove(sidecar_path(asset_path))
    except OSError:
        pass


# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------

def store_path(asset_path, image_path):
    """Turn an absolute image path into the form to write into the sidecar.

    Relative to the asset when that stays inside a sensible number of parent
    steps, absolute otherwise. The cutoff exists because a relative path that
    climbs out of the project and back down again is not more portable than an
    absolute one, just harder to read.
    """
    if not image_path:
        return None

    image_path = os.path.abspath(image_path)

    if not asset_path:
        return image_path

    base = os.path.dirname(os.path.abspath(asset_path))

    try:
        rel = os.path.relpath(image_path, base)
    except ValueError:
        # Different drives on Windows; there is no relative form.
        return image_path

    if rel.count("..") > 4:
        return image_path

    return rel.replace(os.sep, "/")


def resolve_path(asset_path, stored):
    """Turn a stored path back into an absolute one, or None if it is gone.

    Returning None for a path that no longer resolves is what lets the editor
    fall back to its untextured preview silently.
    """
    if not stored:
        return None

    candidate = stored.replace("/", os.sep)

    if os.path.isabs(candidate):
        return candidate if os.path.isfile(candidate) else None

    if asset_path:
        base = os.path.dirname(os.path.abspath(asset_path))
        full = os.path.normpath(os.path.join(base, candidate))
        if os.path.isfile(full):
            return full

    # Last resort: relative to the working directory.
    full = os.path.abspath(candidate)
    return full if os.path.isfile(full) else None
