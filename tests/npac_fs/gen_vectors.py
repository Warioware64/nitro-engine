#!/usr/bin/env python3
# SPDX-License-Identifier: CC0-1.0
#
# SPDX-FileContributor: Warioware64, 2026
#
# Generates the cross-check vectors for the NPAC filesystem.
#
# tools/npac/npac_format.py and source/NEANPAC.c are two implementations of the
# same container: one writes the FNT, the FAT and the compression table, the
# other walks them on a console with no debugger attached. Nothing enforces
# that they agree, and when they drift the symptom is a game that loads the
# wrong file -- or reads one file's bytes under another's name, which is worse
# because it looks like data corruption rather than a filesystem bug.
#
# So this writes two files from one source of truth: the archive itself, and a
# C table of what every member should contain. The test ROM mounts the archive
# with the real runtime and compares. If the two ever diverge, the ROM says
# which member and which property.
#
# The archive deliberately contains the awkward cases: one member per
# compression method, an empty file, a member whose size is not a multiple of
# four and spans more than one 512 byte block, a name at the 127 character
# limit the FNT allows, and a directory nested three deep.

import os
import random
import sys
import zlib

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "tools", "npac"))

import npac_format as N  # noqa: E402

# A name at the FNT's 127 character limit. One more byte and the type field
# would collide with the subdirectory flag.
LONG_NAME = "long_" + ("n" * (127 - len("long_") - len(".bin"))) + ".bin"
assert len(LONG_NAME) == 127


def repetitive(size):
    """Compresses well with LZ77: long-range matches, no long runs."""
    unit = b"The quick brown fox jumps over the lazy dog. "
    return (unit * (size // len(unit) + 1))[:size]


def skewed(size):
    """Compresses well with Huffman: low entropy, no useful matches."""
    rng = random.Random(0x5EED)
    alphabet = b"\x00\x01\x02\xFF"
    return bytes(rng.choice(alphabet) for _ in range(size))


def runs(size):
    """Compresses well with RLE: long runs of one byte."""
    out = bytearray()
    value = 0
    while len(out) < size:
        out += bytes([value & 0xFF]) * 40
        value += 7
    return bytes(out[:size])


def incompressible(size):
    return random.Random(0xC0FFEE).randbytes(size)


# (path, payload, compression to ask for). "auto" on incompressible data is
# there to check the fallback: nothing wins, so it must come out stored.
MEMBERS = [
    ("stored.bin",         incompressible(1024), "auto"),
    ("zero.bin",           b"",                  "auto"),
    ("odd.bin",            repetitive(1023),     "none"),
    (LONG_NAME,            b"named to the limit\n", "none"),
    ("lz/text.txt",        repetitive(4096),     "lzss"),
    ("huff/skew.bin",      skewed(4096),         "huffman"),
    ("rle/runs.bin",       runs(4096),           "rle"),
    ("a/b/c/deep.bin",     repetitive(300),      "none"),
]

MODE_BY_PATH = {path: mode for path, _payload, mode in MEMBERS}
PAYLOAD_BY_PATH = {path: payload for path, payload, _mode in MEMBERS}


def c_string(text):
    return '"' + text.replace("\\", "\\\\").replace('"', '\\"') + '"'


def main():
    entries = [(path, payload) for path, payload, _mode in MEMBERS]

    archive = N.build(entries, compress_mode=lambda p: MODE_BY_PATH[p])

    out_dir = os.path.join(HERE, "nitrofiles")
    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(out_dir, "test.npac"), "wb") as f:
        f.write(archive)

    # Read the archive back rather than trusting what we asked for: the table
    # has to describe the file that actually exists.
    members = N.read(archive)

    methods = {m.method for m in members}
    for required in (N.METHOD_STORED, N.METHOD_LZ77, N.METHOD_RLE):
        if required not in methods:
            raise SystemExit("no %s member was produced, the vectors would "
                             "not cover it" % N.METHOD_NAMES[required])
    if not (methods & {N.METHOD_HUFF4, N.METHOD_HUFF8}):
        raise SystemExit("no Huffman member was produced")

    for m in members:
        want = PAYLOAD_BY_PATH[m.path]
        got = m.data()
        if got != want:
            raise SystemExit("%s does not round trip through the packer"
                             % m.path)

    # Expected directory listings, in the order the format defines: this
    # directory's subdirectories, then its files, each group in name order.
    def listing(prefix):
        depth = prefix.count("/") if prefix else 0
        subdirs, files = set(), []
        for path in PAYLOAD_BY_PATH:
            if not path.startswith(prefix):
                continue
            rest = path[len(prefix):]
            if "/" in rest:
                subdirs.add(rest.split("/", 1)[0])
            else:
                files.append(rest)
        return ([(n, True) for n in sorted(subdirs)] +
                [(n, False) for n in sorted(files)])

    listings = [("/", listing("")), ("/a/b", listing("a/b/"))]

    lines = []
    lines.append("// SPDX-License-Identifier: CC0-1.0")
    lines.append("//")
    lines.append("// Generated by gen_vectors.py. Do not edit.")
    lines.append("")
    lines.append("#ifndef VECTORS_H__")
    lines.append("#define VECTORS_H__")
    lines.append("")
    lines.append("#include <stdint.h>")
    lines.append("")
    lines.append("typedef struct {")
    lines.append("    const char *path;")
    lines.append("    uint32_t size;")
    lines.append("    uint32_t crc;")
    lines.append("    uint8_t method;")
    lines.append("} TestVector;")
    lines.append("")
    lines.append("static const TestVector vectors[] = {")
    for m in members:
        payload = PAYLOAD_BY_PATH[m.path]
        lines.append("    { %s, %d, 0x%08XU, 0x%02X }," %
                     (c_string(m.path), len(payload),
                      zlib.crc32(payload) & 0xFFFFFFFF, m.method))
    lines.append("};")
    lines.append("")
    lines.append("#define NUM_VECTORS (sizeof(vectors) / sizeof(vectors[0]))")
    lines.append("")
    lines.append("typedef struct {")
    lines.append("    const char *name;")
    lines.append("    bool is_dir;")
    lines.append("} TestDirEntry;")
    lines.append("")
    lines.append("typedef struct {")
    lines.append("    const char *path;")
    lines.append("    const TestDirEntry *entries;")
    lines.append("    int count;")
    lines.append("} TestListing;")
    lines.append("")
    for index, (path, entries_) in enumerate(listings):
        lines.append("static const TestDirEntry listing_%d[] = {" % index)
        for name, is_dir in entries_:
            lines.append("    { %s, %s }," % (c_string(name),
                                              "true" if is_dir else "false"))
        lines.append("};")
        lines.append("")
    lines.append("static const TestListing listings[] = {")
    for index, (path, entries_) in enumerate(listings):
        lines.append("    { %s, listing_%d, %d }," %
                     (c_string(path), index, len(entries_)))
    lines.append("};")
    lines.append("")
    lines.append("#define NUM_LISTINGS (sizeof(listings) / sizeof(listings[0]))")
    lines.append("")
    lines.append("#endif // VECTORS_H__")

    with open(os.path.join(HERE, "source", "vectors.h"), "w") as f:
        f.write("\n".join(lines) + "\n")

    print("npac_fs: %d members, %d bytes" % (len(members), len(archive)))
    for m in members:
        print("  %-40s %-6s %6d -> %6d" %
              (m.path, m.method_name, m.raw_size, len(m.stored)))


if __name__ == "__main__":
    main()
