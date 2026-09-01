#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
#
# Copyright (c) 2026 Warioware64
#
# NPAC (Nitro Pak Archive Container) reader/writer. The C runtime in NEANPAC.c
# reads exactly the layout this module writes -- keep them in lockstep.

"""npac_format.py -- read/write NPAC archive containers.

NPAC is NARC with a name of its own and one extra chunk. The container layout
is the one GBATEK documents for NitroARC (see helpSrc/), so the directory tree
is a real NitroROM FNT and the reader can open a plain "NARC" too.

Binary layout (little-endian throughout):

    HEADER (16 bytes)
        magic       u32   'NPAC' (or 'NARC' when reading a foreign archive)
        bom         u16   FFFEh   -- as NARC: not FEFFh
        version     u16   0100h
        size        u32   from the magic to EOF
        header_size u16   0010h
        num_chunks  u16   4, or 3 when nothing in the archive is compressed

    "BTAF" -- file allocation table
        magic       u32   'BTAF'
        size        u32   including the magic: 12 + num_files * 8
        num_files   u16
        reserved    u16   0
        num_files * { u32 start; u32 end }
                          Offsets into the GMIF payload, end exclusive. These
                          are the *stored* extents: for a compressed member
                          they bound the compressed bytes, not the original.

    "BTNF" -- file name table
        magic       u32   'BTNF'
        size        u32   including the magic
        ..                FNT main table + subtables (see below)
        ..                pad to 4 bytes with FFh

    "CMPT" -- compression table (NPAC extension, absent in a NARC)
        magic       u32   'CMPT'
        size        u32   including the magic: 12 + num_files * 4
        num_files   u16   same as BTAF's
        reserved    u16   0
        num_files * u32   method | (raw_size << 8)

    "GMIF" -- file image
        magic       u32   'GMIF'
        size        u32   including the magic
        ..                member data, each padded to 4 bytes with FFh

A CMPT entry is shaped like the GBA/DS BIOS compression header word on purpose
-- byte 0 is the method, bytes 1..3 are the 24-bit decompressed size -- so the
method values are the BIOS ones (see METHOD_* below). It exists so that stat()
and fseek(SEEK_END) on the DS can report the *decompressed* size without the
runtime having to touch the GMIF payload at all.

FNT, unchanged from NitroROM. All offsets are relative to the FNT base, which
is the first byte after the BTNF magic and size.

    MAIN TABLE, 8 bytes per directory, directory IDs F000h and up, indexed by
    (id & 0FFFh):
        subtable_offset u32
        first_file      u16   ID of this directory's first file
        parent          u16   parent directory ID -- except in the root entry
                              (F000h), where it is the total directory count,
                              which is what gives the main table its size

    SUBTABLE, one per directory, ending at a 00h type byte:
        type            u8    00h      end of subtable
                              01h..7Fh file, length = type
                              81h..FFh directory, length = type & 7Fh
        name            char[length]   ASCII, *not* NUL-terminated
        dir_id          u16   directories only

    File IDs are not stored. They run sequentially through the file entries of
    a subtable, starting at that directory's first_file.

Members are laid out in GMIF in file-ID order, and file IDs are assigned by the
same directory walk that emits the subtables, so the two orders always agree.
"""

import os
import shutil
import struct
import subprocess
import tempfile

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

MAGIC_NPAC = b"NPAC"
MAGIC_NARC = b"NARC"

CHUNK_BTAF = b"BTAF"
CHUNK_BTNF = b"BTNF"
CHUNK_CMPT = b"CMPT"
CHUNK_GMIF = b"GMIF"

BOM = 0xFFFE
VERSION = 0x0100
HEADER_SIZE = 0x10

DIR_ID_ROOT = 0xF000
MAX_DIRS = 4096
MAX_FILES = 61440
MAX_NAME = 127
MAX_RAW_SIZE = 0xFFFFFF  # the CMPT size field is 24 bits

# Compression methods. These are the DS BIOS header bytes, so the runtime can
# hand the method straight to decompress() in <nds/decompress.h>.
METHOD_STORED = 0x00
METHOD_LZ77 = 0x10
METHOD_HUFF4 = 0x24
METHOD_HUFF8 = 0x28
METHOD_RLE = 0x30

METHOD_NAMES = {
    METHOD_STORED: "stored",
    METHOD_LZ77: "lz77",
    METHOD_HUFF4: "huff4",
    METHOD_HUFF8: "huff8",
    METHOD_RLE: "rle",
}

# Names the FNT cannot represent. GBATEK: "ASCII 20h..7Eh, except for the
# characters \/?"<>*:;|".
BAD_NAME_CHARS = set('\\/?"<>*:;|')

PAD_BYTE = 0xFF


class NpacError(Exception):
    pass


# ---------------------------------------------------------------------------
# Compression, via the Wonderful Toolchain encoders
# ---------------------------------------------------------------------------

DEFAULT_TOOL_DIR = os.environ.get(
    "WONDERFUL_TOOLCHAIN_BIN",
    os.path.join(os.environ.get("WONDERFUL_TOOLCHAIN", "/opt/wonderful"), "bin"),
)

# Each entry is (tool, encode args, decode args). The method is read back from
# byte 0 of the encoder's output rather than assumed here, because
# "wf-nnpack-huffman -e0" picks between 4- and 8-bit mode on its own.
_CODECS = {
    "lzss": ("wf-nnpack-lzss", ["-evo"], ["-d"]),
    "huffman": ("wf-nnpack-huffman", ["-e0"], ["-d"]),
    "rle": ("wf-nnpack-rle", ["-e"], ["-d"]),
}

_METHOD_CODEC = {
    METHOD_LZ77: "lzss",
    METHOD_HUFF4: "huffman",
    METHOD_HUFF8: "huffman",
    METHOD_RLE: "rle",
}

COMPRESS_MODES = ("auto", "lzss", "huffman", "rle", "none")


def tool_path(name, tool_dir=None):
    """Absolute path of a Wonderful Toolchain encoder, or None if absent."""
    if tool_dir:
        candidate = os.path.join(tool_dir, name)
        return candidate if os.path.isfile(candidate) else None

    candidate = os.path.join(DEFAULT_TOOL_DIR, name)
    if os.path.isfile(candidate):
        return candidate

    return shutil.which(name)


def _run_codec(codec, args, data, tool_dir):
    """Run one encoder/decoder over `data`, returning its output bytes.

    Returns None if the tool is missing or fails -- callers treat that as
    "this method is unavailable", never as a hard error, so that a machine
    without the toolchain can still pack an archive with everything stored.
    """
    exe = tool_path(_CODECS[codec][0], tool_dir)
    if exe is None:
        return None

    with tempfile.TemporaryDirectory(prefix="npac-") as tmp:
        src = os.path.join(tmp, "in.bin")
        dst = os.path.join(tmp, "out.bin")
        with open(src, "wb") as f:
            f.write(data)
        try:
            proc = subprocess.run([exe] + args + [src, dst],
                                  stdout=subprocess.PIPE,
                                  stderr=subprocess.PIPE)
        except OSError:
            return None
        if proc.returncode != 0 or not os.path.isfile(dst):
            return None
        with open(dst, "rb") as f:
            return f.read()


def compress(data, mode="auto", tool_dir=None, allow_huffman=False):
    """Compress `data`, returning (method, stored_bytes).

    Falls back to (METHOD_STORED, data) whenever compression does not actually
    save space, which for small files is most of the time. "auto" tries each
    candidate codec and keeps the smallest result.

    Huffman is left out of "auto" unless `allow_huffman` is set. It decodes
    correctly on hardware and under an emulator running a real BIOS dump, but
    melonDS's HLE BIOS -- what it uses by default, with no external BIOS
    configured -- decodes it to the wrong bytes, silently. Asking for it
    explicitly is fine; picking it automatically would hand that failure to
    someone who never chose it, and it rarely beats LZ77 on DS assets anyway.
    """
    if mode == "none" or len(data) == 0:
        return METHOD_STORED, data

    if len(data) > MAX_RAW_SIZE:
        raise NpacError("file is larger than the 16 MiB the format allows")

    if mode == "auto":
        codecs = ("lzss", "rle") + (("huffman",) if allow_huffman else ())
    else:
        codecs = (mode,)

    best_method = METHOD_STORED
    best = data

    for codec in codecs:
        out = _run_codec(codec, _CODECS[codec][1], data, tool_dir)
        if out is None or len(out) < 4 or len(out) >= len(best):
            continue
        # The encoders write the BIOS header themselves; trust byte 0 for the
        # method and check that they agree with us about the size.
        method = out[0]
        if method not in METHOD_NAMES or method == METHOD_STORED:
            continue
        raw_size = out[1] | (out[2] << 8) | (out[3] << 16)
        if raw_size != len(data):
            continue
        best_method, best = method, out

    return best_method, best


def decompress(method, data, raw_size, tool_dir=None):
    """Inverse of compress(). Raises NpacError if the codec is unavailable."""
    if method == METHOD_STORED:
        return data[:raw_size]

    codec = _METHOD_CODEC.get(method)
    if codec is None:
        raise NpacError("unknown compression method 0x%02X" % method)

    out = _run_codec(codec, _CODECS[codec][2], data, tool_dir)
    if out is None:
        raise NpacError("%s is needed to decode a %s member but is not "
                        "available (looked in %s)"
                        % (_CODECS[codec][0], METHOD_NAMES[method],
                           tool_dir or DEFAULT_TOOL_DIR))
    return out[:raw_size]


# ---------------------------------------------------------------------------
# Building the directory tree
# ---------------------------------------------------------------------------

def check_name(name):
    if not 1 <= len(name) <= MAX_NAME:
        raise NpacError("name %r must be 1..%d characters" % (name, MAX_NAME))
    for c in name:
        if not 0x20 <= ord(c) <= 0x7E or c in BAD_NAME_CHARS:
            raise NpacError("name %r contains %r, which a NitroROM FNT cannot "
                            "represent" % (name, c))


class _Dir:
    __slots__ = ("name", "parent", "subdirs", "files", "first_file",
                 "subtable_offset")

    def __init__(self, name, parent):
        self.name = name
        self.parent = parent
        self.subdirs = []   # indices into the directory list
        self.files = []     # (name, payload bytes)
        self.first_file = 0
        self.subtable_offset = 0


def _tree_from_entries(entries):
    """Turn [(posix path, bytes)] into the flat, ordered directory list.

    Directories come out in depth-first pre-order with siblings sorted, which
    is the order their subtables and their file IDs are assigned in.
    """
    # Group by directory first so the walk below is a plain sort.
    tree = {"dirs": {}, "files": {}}
    for path, payload in entries:
        parts = [p for p in path.replace("\\", "/").split("/") if p not in ("", ".")]
        if not parts:
            raise NpacError("empty path in the entry list")
        node = tree
        for part in parts[:-1]:
            check_name(part)
            node = node["dirs"].setdefault(part, {"dirs": {}, "files": {}})
        check_name(parts[-1])
        if parts[-1] in node["files"]:
            raise NpacError("duplicate entry %r" % path)
        node["files"][parts[-1]] = payload

    dirs = []

    def walk(node, name, parent, prefix):
        index = len(dirs)
        rec = _Dir(name, parent)
        dirs.append(rec)
        rec.files = [(n, prefix + n, node["files"][n])
                     for n in sorted(node["files"])]
        for sub in sorted(node["dirs"]):
            rec.subdirs.append(walk(node["dirs"][sub], sub, index,
                                    prefix + sub + "/"))
        return index

    walk(tree, "", -1, "")

    if len(dirs) > MAX_DIRS:
        raise NpacError("%d directories, the format allows %d"
                        % (len(dirs), MAX_DIRS))

    # File IDs: contiguous within a directory, running in directory order.
    next_id = 0
    for rec in dirs:
        rec.first_file = next_id
        next_id += len(rec.files)
    if next_id > MAX_FILES:
        raise NpacError("%d files, the format allows %d" % (next_id, MAX_FILES))

    return dirs


def _build_fnt(dirs):
    """Serialise the main table and the subtables into the FNT payload."""
    main_size = len(dirs) * 8

    subtables = bytearray()
    for rec in dirs:
        rec.subtable_offset = main_size + len(subtables)
        # Subdirectories first, then files, then the terminator. Only the file
        # entries advance the implicit file ID counter.
        for sub in rec.subdirs:
            name = dirs[sub].name.encode("ascii")
            subtables.append(0x80 | len(name))
            subtables += name
            subtables += struct.pack("<H", DIR_ID_ROOT + sub)
        for name, _path, _payload in rec.files:
            raw = name.encode("ascii")
            subtables.append(len(raw))
            subtables += raw
        subtables.append(0x00)

    main = bytearray()
    for index, rec in enumerate(dirs):
        # The root entry's third field is the directory count, not a parent.
        third = len(dirs) if index == 0 else DIR_ID_ROOT + rec.parent
        main += struct.pack("<IHH", rec.subtable_offset, rec.first_file, third)

    return bytes(main + subtables)


def _chunk(magic, payload):
    return magic + struct.pack("<I", len(payload) + 8) + payload


def _align4(data):
    pad = (-len(data)) & 3
    return data + bytes([PAD_BYTE]) * pad


# ---------------------------------------------------------------------------
# Writing
# ---------------------------------------------------------------------------

def build(entries, compress_mode="auto", tool_dir=None, progress=None,
          allow_huffman=False):
    """Build an archive from [(posix path, bytes)]. Returns the file bytes.

    `compress_mode` is one of COMPRESS_MODES, or a callable taking a member's
    path and returning one, when different files want different treatment.

    `progress`, if given, is called as progress(path, method, raw, stored)
    once per member.
    """
    dirs = _tree_from_entries(entries)

    # Members are laid out in file-ID order, which the walk above already fixed.
    ordered = []
    for rec in dirs:
        for _name, path, payload in rec.files:
            ordered.append((path, payload))

    image = bytearray()
    fat = bytearray()
    cmpt = bytearray()
    any_compressed = False

    for path, payload in ordered:
        if len(payload) > MAX_RAW_SIZE:
            raise NpacError("%r is larger than the 16 MiB the format allows"
                            % path)
        mode = compress_mode(path) if callable(compress_mode) else compress_mode
        method, stored = compress(payload, mode, tool_dir, allow_huffman)
        if method != METHOD_STORED:
            any_compressed = True

        start = len(image)
        image += stored
        fat += struct.pack("<II", start, start + len(stored))
        cmpt += struct.pack("<I", method | (len(payload) << 8))
        # Keep every member word-aligned; the DS reads them with word loads.
        image += bytes([PAD_BYTE]) * ((-len(image)) & 3)

        if progress is not None:
            progress(path, method, len(payload), len(stored))

    num_files = len(ordered)

    btaf = _chunk(CHUNK_BTAF, struct.pack("<HH", num_files, 0) + bytes(fat))
    btnf = _chunk(CHUNK_BTNF, _align4(_build_fnt(dirs)))
    gmif = _chunk(CHUNK_GMIF, bytes(image))

    chunks = [btaf, btnf]
    if any_compressed:
        chunks.append(_chunk(CHUNK_CMPT,
                             struct.pack("<HH", num_files, 0) + bytes(cmpt)))
    chunks.append(gmif)

    body = b"".join(chunks)
    header = struct.pack("<4sHHIHH", MAGIC_NPAC, BOM, VERSION,
                         HEADER_SIZE + len(body), HEADER_SIZE, len(chunks))
    return header + body


def scan_directory(root, follow_links=False):
    """Read a directory tree off disk into the [(posix path, bytes)] form."""
    entries = []
    root = os.path.abspath(root)
    for dirpath, dirnames, filenames in os.walk(root, followlinks=follow_links):
        dirnames.sort()
        for name in sorted(filenames):
            full = os.path.join(dirpath, name)
            if os.path.islink(full) and not follow_links:
                continue
            rel = os.path.relpath(full, root).replace(os.sep, "/")
            with open(full, "rb") as f:
                entries.append((rel, f.read()))
    if not entries:
        raise NpacError("%s contains no files" % root)
    return entries


def pack_directory(root, compress_mode="auto", tool_dir=None, progress=None,
                   allow_huffman=False):
    return build(scan_directory(root), compress_mode, tool_dir, progress,
                 allow_huffman)


# ---------------------------------------------------------------------------
# Reading
# ---------------------------------------------------------------------------

class Member:
    __slots__ = ("path", "file_id", "method", "raw_size", "stored")

    def __init__(self, path, file_id, method, raw_size, stored):
        self.path = path
        self.file_id = file_id
        self.method = method
        self.raw_size = raw_size
        self.stored = stored

    @property
    def method_name(self):
        return METHOD_NAMES.get(self.method, "0x%02X" % self.method)

    def data(self, tool_dir=None):
        return decompress(self.method, self.stored, self.raw_size, tool_dir)


def _split_chunks(data):
    """Walk the chunk list by magic rather than by position.

    Chunk order is fixed in practice, but reading by name is what lets the same
    parser accept a three-chunk NARC and a four-chunk NPAC without a special
    case, and it survives an archive that puts CMPT somewhere else.
    """
    magic = data[0:4]
    if magic not in (MAGIC_NPAC, MAGIC_NARC):
        raise NpacError("not an NPAC or NARC archive (magic %r)" % magic)

    bom, version, size, header_size, num_chunks = struct.unpack_from("<HHIHH", data, 4)
    if bom != BOM:
        raise NpacError("bad byte order mark %04Xh" % bom)
    if header_size < HEADER_SIZE:
        raise NpacError("bad header size %04Xh" % header_size)

    chunks = {}
    pos = header_size
    for index in range(num_chunks):
        if pos + 8 > len(data):
            break
        name = data[pos:pos + 4]
        (chunk_size,) = struct.unpack_from("<I", data, pos + 4)
        if chunk_size < 8:
            raise NpacError("chunk %r has a size of %d" % (name, chunk_size))
        # The BTNF size is written both with and without its FFh tail padding
        # depending on who produced the archive, so round it up either way.
        end = pos + ((chunk_size + 3) & ~3)

        # Plenty of NARCs record a GMIF size that disagrees with what the file
        # actually holds. When the image is the last chunk -- which it is in
        # every archive worth opening -- the file's own length is the honest
        # answer. NEANPAC.c reads it the same way.
        if name == CHUNK_GMIF and index == num_chunks - 1:
            end = len(data)

        chunks[name] = data[pos + 8:min(end, len(data))]
        pos = min(end, len(data))

    for required in (CHUNK_BTAF, CHUNK_BTNF, CHUNK_GMIF):
        if required not in chunks:
            raise NpacError("archive has no %r chunk" % required)

    return chunks


def _read_fnt(fnt, num_files):
    """Rebuild the path of every file ID from the FNT. Returns {id: path}."""
    if len(fnt) < 8:
        return {}
    num_dirs = struct.unpack_from("<H", fnt, 6)[0]
    if num_dirs == 0 or num_dirs * 8 > len(fnt):
        return {}

    paths = {}

    def walk(dir_index, prefix):
        offset, first_file, _third = struct.unpack_from("<IHH", fnt, dir_index * 8)
        file_id = first_file
        pos = offset
        while pos < len(fnt):
            kind = fnt[pos]
            pos += 1
            if kind == 0x00:
                return
            length = kind & 0x7F
            name = fnt[pos:pos + length].decode("ascii", "replace")
            pos += length
            if kind & 0x80:
                sub_id = struct.unpack_from("<H", fnt, pos)[0]
                pos += 2
                walk(sub_id & 0x0FFF, prefix + name + "/")
            else:
                paths[file_id] = prefix + name
                file_id += 1

    walk(0, "")
    return paths


def read(data):
    """Parse an archive. Returns a list of Member, in file-ID order."""
    chunks = _split_chunks(data)

    btaf = chunks[CHUNK_BTAF]
    num_files = struct.unpack_from("<H", btaf, 0)[0]
    image = chunks[CHUNK_GMIF]

    cmpt = chunks.get(CHUNK_CMPT)
    paths = _read_fnt(chunks[CHUNK_BTNF], num_files)

    members = []
    for i in range(num_files):
        start, end = struct.unpack_from("<II", btaf, 4 + i * 8)
        if start > end or end > len(image):
            raise NpacError("file %d has extents %d..%d, outside a %d byte "
                            "image" % (i, start, end, len(image)))
        stored = image[start:end]

        if cmpt is not None and 4 + i * 4 + 4 <= len(cmpt):
            word = struct.unpack_from("<I", cmpt, 4 + i * 4)[0]
            method = word & 0xFF
            raw_size = word >> 8
        else:
            # A NARC, or an NPAC with nothing compressed.
            method, raw_size = METHOD_STORED, len(stored)

        members.append(Member(paths.get(i, "file_%04d.bin" % i), i,
                              method, raw_size, stored))

    return members


def read_file(path):
    with open(path, "rb") as f:
        return read(f.read())
