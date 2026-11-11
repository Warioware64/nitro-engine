// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Warioware64
//
// This file is part of Nitro Engine Advanced

#ifndef NEA_NPAC_H__
#define NEA_NPAC_H__

#include <nds.h>

/// @file   NEANPAC.h
/// @brief  NPAC archive containers, mounted as ordinary drives.

/// @defgroup npac NPAC archive containers
///
/// NPAC (Nitro Pak Archive Container) packs a directory tree into a single
/// file, the way NARC does in retail DS games, and mounts it as a drive of its
/// own. It exists for two reasons: a few hundred loose assets waste a lot of
/// ROM on NitroFS padding and re-walk the cartridge FNT on every fopen(), and
/// loose files have nowhere to put compression.
///
/// An archive is built on the PC with tools/npac/npac.py and mounted at run
/// time under a name you choose:
///
/// ```
/// python3 tools/npac/npac.py create --input assets/ --output levels.npac
/// ```
/// ```c
/// nitroFSInit(NULL);
/// NEA_NpacMount("levels", "nitro:/levels.npac");
/// ```
///
/// After that "levels:/robot.bin" is a path like any other. It works with
/// fopen(), opendir() and stat(), and with every NEA_*LoadFAT() function,
/// because the archive is registered with libnds as a device I/O filesystem
/// rather than being a private thing this module reads:
///
/// ```c
/// NEA_ModelLoadStaticMeshFAT(Model, "levels:/robot.bin");
/// ```
///
/// Members compressed by the packer are decompressed transparently by their
/// first read, so nothing above this layer needs to know. stat() and
/// fseek(SEEK_END) report the decompressed size, which is what callers size
/// their buffers from; they come from the in-RAM index, so opening a file only
/// to ask its size costs no I/O.
///
/// Two things are worth knowing before leaning on compression:
///
/// - The whole member is inflated into main RAM by the first read(), and it
///   stays there until close(). A stored member costs nothing and is read
///   straight out of the archive. Running out of memory for that copy makes
///   the read fail with ENOMEM; open() itself no longer reads anything.
/// - That inflate is one uninterrupted block of work. NEA_FATLoadDataAsync()
///   yields between chunks while *reading*, so a large compressed asset loaded
///   asynchronously will still stall the frame of its first read. Store the
///   big ones.
///
/// One emulator caveat, because it costs a day to find: Huffman members decode
/// correctly on hardware and under an emulator using a real BIOS dump, but
/// melonDS's HLE BIOS -- its default, with no external BIOS configured --
/// decodes them to the wrong bytes and reports no error. LZ77 and RLE are fine
/// there. This is why the packer's "auto" mode does not choose Huffman unless
/// it is asked to, with --allow-huffman.
///
/// Opening a member does not touch the heap: up to 16 files open at once use
/// handles kept in a static array, and only those past that are allocated.
///
/// The container is NARC's, with a different magic and one extra chunk holding
/// the per-file compression table, so a plain NARC can be mounted too (all of
/// its members are then treated as stored).
///
/// @{

/// Maximum number of archives that can be mounted at the same time.
#define NEA_NPAC_MAX_MOUNTS         4

/// Maximum length of a drive name, not counting the terminator.
///
/// Shorter than libnds' DEVICE_IO_MAX_DRIVE_NAME_LENGTH, which is the real
/// limit, so that a name always fits in the buffers used to build paths.
#define NEA_NPAC_MAX_DRIVE_NAME     15

/// Compression applied to a member, as reported by NEA_NpacGetFileInfo().
///
/// The values are the DS BIOS compression header bytes, which is what lets the
/// method be handed straight to decompress() in <nds/decompress.h>.
typedef enum {
    NEA_NPAC_STORED = 0x00, ///< Not compressed
    NEA_NPAC_LZ77   = 0x10, ///< LZ77 / LZ10
    NEA_NPAC_HUFF4  = 0x24, ///< Huffman, 4 bit symbols
    NEA_NPAC_HUFF8  = 0x28, ///< Huffman, 8 bit symbols
    NEA_NPAC_RLE    = 0x30  ///< Run length encoding
} NEA_NpacMethod;

/// How one member is stored in its archive.
typedef struct {
    uint32_t size;          ///< Size once decompressed, what stat() reports
    uint32_t stored_size;   ///< Size of the bytes actually held in the archive
    uint16_t file_id;       ///< Index of the file inside the archive
    uint8_t method;         ///< One of NEA_NpacMethod
} NEA_NpacFileInfo;

/// Mounts an NPAC archive and makes it reachable as "drive:/".
///
/// The archive file stays open for as long as the mount does, and only its
/// index is held in RAM; member data is read on demand. The file may live on
/// any drive that is already usable, so nitroFSInit() or fatInitDefault() has
/// to have succeeded first.
///
/// @param drive
///     Drive name to mount under, without colon or slash (for example
///     "levels", giving "levels:/"). At most NEA_NPAC_MAX_DRIVE_NAME
///     characters. It may not be a name libnds already owns ("nitro", "fat",
///     "sd", "nand", "nand2"), which would be unreachable.
/// @param path
///     Path of the archive file, for example "nitro:/levels.npac".
/// @return
///     0 on success. On error, -1, with errno set.
int NEA_NpacMount(const char *drive, const char *path);

/// Mounts an NPAC archive with its index in memory the caller gives.
///
/// Like NEA_NpacMount(), but the index (the file and name tables, and the
/// compression table) is read into "index" instead of being allocated, and
/// unmounting does not free it. Mounting archives one after another into the
/// same memory touches the heap for none of them. The memory must stay
/// untouched for as long as the archive is mounted.
///
/// @param drive
///     Drive name to mount under, as for NEA_NpacMount().
/// @param path
///     Path of the archive file.
/// @param index
///     Memory for the index, word-aligned.
/// @param size
///     Its size in bytes. NEA_NpacIndexBytes() tells how much an archive
///     needs.
/// @return
///     0 on success. On error, -1, with errno set: ENOSPC when the index does
///     not fit, in which case nothing is mounted.
int NEA_NpacMountInto(const char *drive, const char *path, void *index,
                      size_t size);

/// Tells how much memory NEA_NpacMountInto() needs for an archive's index.
///
/// Only the archive's chunk headers are read.
///
/// @param path
///     Path of the archive file.
/// @return
///     The size in bytes. On error, -1, with errno set.
int NEA_NpacIndexBytes(const char *path);

/// Reads part of one member of an archive, without mounting the archive.
///
/// The archive is opened, the member's name is looked up by reading the name
/// table a little at a time, its bytes are read, and the archive is closed.
/// Nothing of the index is kept and nothing is allocated here (libnds' own
/// open() still allocates its file structure). For a look at one small file
/// in an archive that is not otherwise needed -- a level's header, say -- it
/// costs a few short reads instead of a mount.
///
/// @param archive
///     Path of the archive file, for example "fat:/levels/b.npac".
/// @param member
///     Path of the member inside it, for example "level.0".
/// @param from
///     First byte to read, as for NEA_NpacMemberRead().
/// @param dst
///     Where the bytes go.
/// @param len
///     How many bytes, as for NEA_NpacMemberRead(): any range of a stored
///     member, the whole of a packed (LZ77 or RLE) one.
/// @return
///     0 on success. On error, -1, with errno set: ENOENT when there is no
///     such member, EISDIR for a directory, and as NEA_NpacMemberRead().
int NEA_NpacPeek(const char *archive, const char *member, uint32_t from,
                 void *dst, uint32_t len);

/// Unmounts an archive mounted by NEA_NpacMount() and frees its index (but
/// not memory given to NEA_NpacMountInto(), which stays the caller's).
///
/// Files and directories still open on that archive are not closed by this.
/// Reading through one afterwards fails with EBADF rather than returning
/// anything, and it keeps failing even once another archive has taken the
/// freed mount slot. Close them first anyway: the memory a compressed member
/// was inflated into is only released by close().
///
/// @param drive
///     Drive name given to NEA_NpacMount().
/// @return
///     0 on success. On error, -1, with errno set.
int NEA_NpacUnmount(const char *drive);

/// Tells whether a drive name currently refers to a mounted archive.
///
/// @param drive
///     Drive name, without colon or slash.
/// @return
///     True if an archive is mounted under that name.
bool NEA_NpacIsMounted(const char *drive);

/// Returns how a member is stored, without opening or decompressing it.
///
/// This answers from the in-RAM index, so it costs no I/O. It is the way to
/// find out what compression a file ended up with, which the packer chooses
/// per file.
///
/// @param path
///     Path of a file inside a mounted archive, for example
///     "levels:/robot.bin".
/// @param info
///     Filled in on success.
/// @return
///     0 on success. On error, -1, with errno set.
int NEA_NpacGetFileInfo(const char *path, NEA_NpacFileInfo *info);

/// Where a member lies in its archive: found once, read as often as wanted.
///
/// Filled in by NEA_NpacMemberFind() and read by NEA_NpacMemberRead(), which
/// need no file handle and no heap. It stays good until its archive is
/// unmounted; a read after that fails with EBADF, even once another archive
/// has taken the mount slot.
typedef struct {
    uint32_t generation;    ///< Which mounting of the archive it belongs to
    uint32_t offset;        ///< Where its bytes start in the archive file
    uint32_t size;          ///< Size once decompressed
    uint32_t stored_size;   ///< Size of the bytes actually held in the archive
    uint16_t file_id;       ///< Index of the file inside the archive
    uint8_t mount;          ///< Mount slot of its archive
    uint8_t method;         ///< One of NEA_NpacMethod
} NEA_NpacMember;

/// Finds a member, for NEA_NpacMemberRead(), without opening it.
///
/// This answers from the in-RAM index, so it costs no I/O and no memory.
///
/// @param path
///     Path of a file inside a mounted archive, for example
///     "levels:/robot.bin".
/// @param member
///     Filled in on success.
/// @return
///     0 on success. On error, -1, with errno set (ENODEV when the path is
///     not on a mounted archive).
int NEA_NpacMemberFind(const char *path, NEA_NpacMember *member);

/// Reads a member found by NEA_NpacMemberFind() straight into a buffer.
///
/// Nothing is allocated and no file is opened. A stored member can be read in
/// any range, so a large one can be read in pieces. A packed one can only be
/// read whole (from 0, at least member->size bytes); it is decompressed
/// straight into the buffer while its bytes are read from the archive 512 at a
/// time. LZ77 and RLE members can be read this way, Huffman ones cannot.
///
/// @param member
///     The member.
/// @param from
///     First byte to read, in the member as it is once decompressed.
/// @param dst
///     Where the bytes go.
/// @param len
///     How many bytes to read. A stored member needs from + len to be within
///     its size; a packed one needs from == 0 and len >= its size, and
///     member->size bytes are written.
/// @return
///     0 on success. On error, -1, with errno set: EBADF once the archive is
///     gone, EINVAL for a range it cannot serve, ENOTSUP for a Huffman member,
///     EIO when the archive cannot be read or its packed bytes are broken.
int NEA_NpacMemberRead(const NEA_NpacMember *member, uint32_t from, void *dst,
                       uint32_t len);

/// Reads a whole member into a buffer, with no handle and no heap.
///
/// NEA_NpacMemberFind() then NEA_NpacMemberRead() of the whole member.
///
/// @param path
///     Path of a file inside a mounted archive.
/// @param dst
///     Where the bytes go.
/// @param capacity
///     Size of the buffer. A member bigger than this is not read (ENOSPC).
/// @return
///     The member's size on success. On error, -1, with errno set.
int NEA_NpacReadInto(const char *path, void *dst, uint32_t capacity);

/// Defined when NEA_NpacSeekInfo() and NEA_NpacFailReads() exist.
#define NEA_NPAC_HAS_SEEK_INFO 1

/// The largest fast-seek table a mount may take, in bytes, in DS mode and in
/// DSi mode (chosen by isDSiMode() as the archive is opened). A table takes
/// two words a fragment of the file and two more: 2048 bytes holds 255
/// fragments, 4096 holds 511. An archive in more pieces is read without fast
/// seek.
#ifndef NEA_NPAC_SEEK_TABLE_DS
#define NEA_NPAC_SEEK_TABLE_DS      2048
#endif
#ifndef NEA_NPAC_SEEK_TABLE_DSI
#define NEA_NPAC_SEEK_TABLE_DSI     4096
#endif

/// How a mounted archive's file seeks, as NEA_NpacSeekInfo() reports it.
typedef enum {
    NEA_NPAC_SEEK_FAST = 0,         ///< FatFs's fast seek, with a whole table
    NEA_NPAC_SEEK_NOT_FAT,          ///< Not on a FAT drive (NitroFS...)
    NEA_NPAC_SEEK_NO_MEMORY,        ///< The table could not be allocated
    NEA_NPAC_SEEK_TOO_FRAGMENTED,   ///< More pieces than the table may hold
    NEA_NPAC_SEEK_REOPENED,         ///< Opened again after a failed read, with no table
} NEA_NpacSeekState;

/// What NEA_NpacSeekInfo() fills in.
typedef struct {
    bool fast;                  ///< Seeks use the table (no walk of the FAT)
    NEA_NpacSeekState state;    ///< Why not, when not
    int32_t fragments;          ///< Pieces the file is in on the card; -1 if not known
    uint32_t table_bytes;       ///< The table's size on the heap, 0 with none
    uint32_t plain_reopens;     ///< Times opened again with no table since mounted
} NEA_NpacSeek;

/// Tells how a mounted archive's file seeks.
///
/// At mount, the archive is given FatFs's fast seek (a table of where each
/// piece of the file is) when the table fits NEA_NPAC_SEEK_TABLE_DS or _DSI.
/// Without one, a seek follows the file's chain through the FAT, which costs
/// more the further into the file it goes. After a failed read the archive is
/// opened again without a table (no table is allocated then) until it is
/// mounted again.
///
/// @param drive
///     Drive name given to NEA_NpacMount().
/// @param info
///     Filled in.
/// @return
///     0 on success. -1 if nothing is mounted under that name (ENOENT).
int NEA_NpacSeekInfo(const char *drive, NEA_NpacSeek *info);

/// Testing: the next `count` reads of any mounted archive's file fail as a
/// card's read would, so that the archive is opened again and the read tried
/// again. Not for use in a game.
void NEA_NpacFailReads(int count);

/// Unmounts every archive and unregisters the filesystem.
///
/// Called automatically by NEA_End(). There is no need to call it by hand
/// unless you want the memory back earlier.
void NEA_NpacSystemEnd(void);

/// @}

#endif // NEA_NPAC_H__
