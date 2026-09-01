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
/// Members compressed by the packer are decompressed transparently when they
/// are opened, so nothing above this layer needs to know. stat() and
/// fseek(SEEK_END) report the decompressed size, which is what callers size
/// their buffers from.
///
/// Two things are worth knowing before leaning on compression:
///
/// - The whole member is inflated into main RAM inside open(), and it stays
///   there until close(). A stored member costs nothing and is read straight
///   out of the archive.
/// - That inflate is one uninterrupted block of work. NEA_FATLoadDataAsync()
///   yields between chunks while *reading*, so a large compressed asset loaded
///   asynchronously will still stall the frame it opens on. Store the big ones.
///
/// One emulator caveat, because it costs a day to find: Huffman members decode
/// correctly on hardware and under an emulator using a real BIOS dump, but
/// melonDS's HLE BIOS -- its default, with no external BIOS configured --
/// decodes them to the wrong bytes and reports no error. LZ77 and RLE are fine
/// there. This is why the packer's "auto" mode does not choose Huffman unless
/// it is asked to, with --allow-huffman.
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

/// Unmounts an archive mounted by NEA_NpacMount() and frees its index.
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

/// Unmounts every archive and unregisters the filesystem.
///
/// Called automatically by NEA_End(). There is no need to call it by hand
/// unless you want the memory back earlier.
void NEA_NpacSystemEnd(void);

/// @}

#endif // NEA_NPAC_H__
