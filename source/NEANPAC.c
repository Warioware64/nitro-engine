// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Warioware64
//
// This file is part of Nitro Engine Advanced
//
// NPAC archive containers, exposed to the rest of the world as ordinary
// drives. Everything here hangs off one device_io_t handed to libnds, which
// is why no loader in the engine had to learn about archives: "levels:/a.bin"
// reaches fopen() and therefore reaches every NEA_*LoadFAT().
//
// One device serves every mount. That is forced, not tidiness: libnds allows
// DEVICE_IO_MAX_DEVICES (5) devices and reserves three of them, so a device
// per archive would cap us at two archives. isdrive() picks the mount out of
// the table by name instead.
//
// The whole index -- FAT, FNT and compression table -- is read into RAM at
// mount time and only the file image is left on disk. libnds' own NitroFS
// caches nothing and re-walks the FNT off the card through a 512 byte sliding
// window; holding a few KB here instead makes path lookup, stat() and
// readdir() pure RAM work, and it is what makes the directory iterator below
// a plain cursor rather than a refilling buffer.
//
// tools/npac/npac_format.py writes the layout this reads, and its module
// docstring is the format spec. If you change one, change the other.

#include <errno.h>
#include <fcntl.h>

#include <nds/cothread.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <fat.h>
#include <nds/arm9/device_io.h>
#include <nds/decompress.h>

#include "NEAMain.h"

/// @file NEANPAC.c

// Directory IDs start here; anything below is a file ID. Both live in the
// same 16 bit space, which is what lets a lookup return either one.
#define NPAC_DIR_ROOT       0xF000

// Depth limit when rebuilding a path for getcwd().
#define NPAC_MAX_DEPTH      32

//-----------------------------------------------------------------------------
// State
//-----------------------------------------------------------------------------

typedef struct {
    bool used;
    char name[NEA_NPAC_MAX_DRIVE_NAME + 1];
    int fd;                 // Archive file, held open for the mount's lifetime
    // Where 'fd' was opened from, to open it again after a failed read. NULL
    // when the path was too long to keep (the read is then not retried).
    const char *path;
    // Held across the seek+read pair on 'fd'. See ne_npac_io_lock().
    volatile bool io_busy;
    uint32_t img_offset;    // Offset of the file image within the archive file
    uint32_t img_size;
    uint8_t *fat;           // num_files * 8 bytes
    uint8_t *fnt;           // Name table, offsets inside it are relative to it
    uint8_t *cmp;           // num_files * 4 bytes, NULL for a NARC
    uint32_t fnt_size;
    uint32_t fnt_file;      // With no fnt (a peek): where it is in the file
    uint16_t num_files;
    uint16_t num_dirs;
    uint16_t current_dir;   // Working directory of this mount

    // When the caller gave the memory for the index (NEA_NpacMountInto()),
    // the tables are cut from it in turn and never freed here.
    uint8_t *index_buf;
    size_t index_size;
    size_t index_used;

    // How 'fd' seeks (NEA_NpacSeekInfo()).
    bool fast_seek;
    uint8_t seek_state;     // NEA_NpacSeekState
    int32_t fragments;      // -1: not known
    uint32_t seek_table;    // Bytes of the table on the heap
    uint32_t plain_reopens;

    // Bumped every time this slot is mounted or unmounted. Open files and
    // directories capture it so that a handle left over from a previous life
    // of the slot fails instead of reading the archive that replaced it.
    uint32_t generation;
} ne_npac_mount_t;

typedef struct {
    ne_npac_mount_t *mnt;
    uint32_t generation;
    uint32_t offset;        // Where the stored bytes start in the archive file
    uint32_t size;          // Decompressed size, which is the size callers see
    uint32_t position;
    uint32_t stored;        // Bytes held in the archive
    uint8_t *ram;           // Decompressed copy, made by the first read
    uint16_t file_index;
    uint8_t method;         // NEA_NPAC_STORED, or how to inflate it
} ne_npac_file_t;

// Cursor over one directory's subtable.
typedef struct {
    const ne_npac_mount_t *mnt;
    uint32_t pos;
    uint16_t file_index;    // File IDs are implicit, so they are counted here
} ne_npac_iter_t;

typedef struct {
    ne_npac_mount_t *mnt;
    uint32_t generation;
    ne_npac_iter_t it;
    uint16_t dir_id;
    uint16_t parent_id;
    int phase;              // 0 emits ".", 1 emits "..", 2 emits real entries
    long index;
} ne_npac_dir_t;

static ne_npac_mount_t ne_npac_mounts[NEA_NPAC_MAX_MOUNTS];

// Each mount's own copy of its archive's path (a peek's mount, on the stack
// for one call, points at the caller's string instead).
#define NPAC_PATH_MAX       128

static char ne_npac_paths[NEA_NPAC_MAX_MOUNTS][NPAC_PATH_MAX];

// Open files come from here, so that opening one does not touch the heap.
// Past this many open at once, they are calloc()ed as they always were. The
// descriptor is the handle's address, which libnds needs word-aligned and in
// main RAM (see ne_npac_open()); a static array is both.
#define NPAC_FILES_STATIC   16

static ne_npac_file_t ne_npac_files[NPAC_FILES_STATIC] __attribute__((aligned(4)));
static uint32_t ne_npac_files_used = 0;     // One bit per entry above

static ne_npac_file_t *ne_npac_file_new(void)
{
    for (int i = 0; i < NPAC_FILES_STATIC; i++)
    {
        if ((ne_npac_files_used & (1U << i)) == 0)
        {
            ne_npac_files_used |= 1U << i;
            memset(&ne_npac_files[i], 0, sizeof(ne_npac_files[i]));
            return &ne_npac_files[i];
        }
    }

    return calloc(1, sizeof(ne_npac_file_t));
}

static void ne_npac_file_delete(ne_npac_file_t *f)
{
    if ((f >= ne_npac_files) && (f < ne_npac_files + NPAC_FILES_STATIC))
        ne_npac_files_used &= ~(1U << (f - ne_npac_files));
    else
        free(f);
}
static ne_npac_mount_t *ne_npac_current = NULL;
static int ne_npac_device_index = -1;

//-----------------------------------------------------------------------------
// Reading the archive
//-----------------------------------------------------------------------------

// The index is read byte by byte. Subtable entries are not aligned to
// anything, so a directory ID can straddle a word boundary.
static inline uint16_t ne_rd16(const uint8_t *p)
{
    return (uint16_t)(p[0] | (p[1] << 8));
}

static inline uint32_t ne_rd32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

// Serialises the seek+read pair on a mount's archive descriptor.
//
// Every open member of an archive reads through the one descriptor held for the
// mount's lifetime, and the seek and the read are not atomic: with DLDI on the
// ARM7, read() yields to other cothreads while it waits for the card. So with
// two asynchronous loads running against the same archive, one worker's lseek()
// can land between the other's lseek() and read(), and that read then returns
// bytes from the wrong file -- silently, with no error anywhere.
//
// Cooperative scheduling makes a plain flag a sufficient lock: nothing can
// preempt the test-and-set below, only an explicit yield can switch away.
static void ne_npac_io_lock(ne_npac_mount_t *m)
{
    while (m->io_busy)
        cothread_yield();

    m->io_busy = true;
}

static void ne_npac_io_unlock(ne_npac_mount_t *m)
{
    m->io_busy = false;
}

// How many times a read of the archive is tried before it is given up.
#define NPAC_READ_TRIES     3

// NEA_NpacFailReads(): reads still to fail on purpose.
static volatile int ne_npac_fail_reads = 0;

void NEA_NpacFailReads(int count)
{
    ne_npac_fail_reads = count;
}

static int ne_npac_read_once(ne_npac_mount_t *m, uint32_t offset, void *dst,
                             size_t len)
{
    if (ne_npac_fail_reads > 0)
    {
        ne_npac_fail_reads--;
        return -1;
    }

    if ((m->fd < 0) || (lseek(m->fd, (off_t)offset, SEEK_SET) != (off_t)offset))
        return -1;

    uint8_t *out = dst;
    size_t done = 0;
    while (done < len)
    {
        ssize_t got = read(m->fd, out + done, len - done);
        if (got <= 0)
            return -1;
        done += (size_t)got;
    }

    return 0;
}

// A card can fail one read and give the same sectors on the next (a DSi's SD
// controller times out now and then). The file it failed on keeps failing
// all the same: FAT remembers the error on the open file. So the archive is
// opened again, and the read tried again from there.
static bool ne_npac_reopen(ne_npac_mount_t *m)
{
    if (m->path == NULL)
        return false;

    if (m->fd >= 0)
        close(m->fd);

    // With no fast-seek table: making one allocates, and this happens in
    // the middle of a game. (open() itself allocates its file, as it did
    // before.) The archive seeks the slow way until it is mounted again.
    m->fd = open(m->path, O_RDONLY, 0);
    if (m->fast_seek)
        m->seek_state = NEA_NPAC_SEEK_REOPENED;
    m->fast_seek = false;
    m->seek_table = 0;
    m->plain_reopens++;
    if (m->fd < 0)
    {
        m->fd = -1;
        return false;
    }

    return true;
}

static int ne_npac_read_at(ne_npac_mount_t *m, uint32_t offset, void *dst,
                           size_t len)
{
    if (len == 0)
        return 0;

    ne_npac_io_lock(m);

    int result = -1;
    for (int tries = 0; tries < NPAC_READ_TRIES; tries++)
    {
        if ((tries > 0) && !ne_npac_reopen(m))
            break;

        if (ne_npac_read_once(m, offset, dst, len) == 0)
        {
            result = 0;
            break;
        }

        NEA_DebugPrint("NPAC read of %u bytes at %u failed", (unsigned)len,
                       (unsigned)offset);
    }

    ne_npac_io_unlock(m);
    return result;
}

static uint8_t *ne_npac_read_chunk(ne_npac_mount_t *m, uint32_t offset,
                                   uint32_t len)
{
    uint8_t *buf;

    if (m->index_buf != NULL)
    {
        // Cut from the caller's memory, each table word-aligned.
        size_t room = (len + 3) & ~(size_t)3;
        if (room > m->index_size - m->index_used)
        {
            NEA_DebugPrint("NPAC index doesn't fit in the memory given");
            errno = ENOSPC;
            return NULL;
        }
        buf = m->index_buf + m->index_used;
        m->index_used += room;
    }
    else
    {
        buf = malloc(len ? len : 1);
        if (buf == NULL)
        {
            NEA_DebugPrint("Not enough memory");
            errno = ENOMEM;
            return NULL;
        }
    }

    if (ne_npac_read_at(m, offset, buf, len) != 0)
    {
        if (m->index_buf == NULL)
            free(buf);
        errno = EIO;
        return NULL;
    }

    return buf;
}

// Reads the header and the index. The chunks are walked by name rather than by
// position: the order is fixed in practice, but reading by name is what lets
// the same code accept a three chunk NARC and a four chunk NPAC.
static int ne_npac_load_index(ne_npac_mount_t *m)
{
    uint8_t header[16];

    if (ne_npac_read_at(m, 0, header, sizeof(header)) != 0)
    {
        NEA_DebugPrint("Couldn't read the NPAC header");
        errno = EIO;
        return -1;
    }

    if ((memcmp(header, "NPAC", 4) != 0) && (memcmp(header, "NARC", 4) != 0))
    {
        NEA_DebugPrint("Not an NPAC archive");
        errno = EINVAL;
        return -1;
    }

    // NARC's byte order mark is FFFEh, which is the other way round from the
    // usual convention. GBATEK calls this out; it is not a typo.
    if (ne_rd16(header + 4) != 0xFFFE)
    {
        NEA_DebugPrint("Bad NPAC byte order mark");
        errno = EINVAL;
        return -1;
    }

    uint32_t pos = ne_rd16(header + 12);
    uint16_t num_chunks = ne_rd16(header + 14);

    if (pos < sizeof(header))
    {
        NEA_DebugPrint("Bad NPAC header size");
        errno = EINVAL;
        return -1;
    }

    uint32_t cmp_offset = 0;
    uint32_t cmp_size = 0;
    bool img_is_last = false;

    for (uint16_t i = 0; i < num_chunks; i++)
    {
        uint8_t chunk[8];

        if (ne_npac_read_at(m, pos, chunk, sizeof(chunk)) != 0)
            break;

        uint32_t size = ne_rd32(chunk + 4);
        if (size < sizeof(chunk))
        {
            NEA_DebugPrint("Bad NPAC chunk size");
            errno = EINVAL;
            return -1;
        }

        uint32_t payload = pos + sizeof(chunk);
        uint32_t payload_size = size - sizeof(chunk);

        if (memcmp(chunk, "BTAF", 4) == 0)
        {
            if (payload_size < 4)
            {
                errno = EINVAL;
                return -1;
            }

            uint8_t count[4];
            if (ne_npac_read_at(m, payload, count, sizeof(count)) != 0)
            {
                errno = EIO;
                return -1;
            }

            m->num_files = ne_rd16(count);
            if ((uint32_t)m->num_files * 8 + 4 > payload_size)
            {
                NEA_DebugPrint("Truncated NPAC allocation table");
                errno = EINVAL;
                return -1;
            }

            m->fat = ne_npac_read_chunk(m, payload + 4,
                                        (uint32_t)m->num_files * 8);
            if (m->fat == NULL)
                return -1;
        }
        else if (memcmp(chunk, "BTNF", 4) == 0)
        {
            m->fnt_size = payload_size;
            m->fnt = ne_npac_read_chunk(m, payload, payload_size);
            if (m->fnt == NULL)
                return -1;
        }
        else if (memcmp(chunk, "CMPT", 4) == 0)
        {
            // Read after the loop: this needs the file count, and nothing
            // guarantees BTAF has been seen yet.
            cmp_offset = payload;
            cmp_size = payload_size;
        }
        else if (memcmp(chunk, "GMIF", 4) == 0)
        {
            m->img_offset = payload;
            m->img_size = payload_size;
            img_is_last = (i + 1) == num_chunks;
        }

        // The tail padding of the name table is counted by some writers and
        // not by others, so round up either way.
        pos += (size + 3) & ~3u;
    }

    if ((m->fat == NULL) || (m->fnt == NULL) || (m->img_offset == 0))
    {
        NEA_DebugPrint("NPAC archive is missing a chunk");
        errno = EINVAL;
        return -1;
    }

    // Plenty of NARCs in the wild record a GMIF size that disagrees with what
    // the file actually holds. When the image is the last chunk, which it is
    // in every archive worth opening, the file's own length is the honest
    // answer; npac_format.py reads it the same way.
    if (img_is_last)
    {
        ne_npac_io_lock(m);
        off_t end = lseek(m->fd, 0, SEEK_END);
        ne_npac_io_unlock(m);

        if ((end > 0) && ((uint32_t)end > m->img_offset))
            m->img_size = (uint32_t)end - m->img_offset;
    }

    if (cmp_offset != 0)
    {
        if ((uint32_t)m->num_files * 4 + 4 > cmp_size)
        {
            NEA_DebugPrint("Truncated NPAC compression table");
            errno = EINVAL;
            return -1;
        }

        m->cmp = ne_npac_read_chunk(m, cmp_offset + 4,
                                    (uint32_t)m->num_files * 4);
        if (m->cmp == NULL)
            return -1;
    }

    // The root entry's third field is the directory count, which is what gives
    // the main table its size.
    if (m->fnt_size >= 8)
        m->num_dirs = ne_rd16(m->fnt + 6);

    if ((m->num_dirs == 0) || ((uint32_t)m->num_dirs * 8 > m->fnt_size))
    {
        NEA_DebugPrint("Bad NPAC name table");
        errno = EINVAL;
        return -1;
    }

    return 0;
}

// Opens a mount's archive file with FatFs's fast seek, if it can have it.
//
// libnds's fatInitLookupCache() allocates a table and fills it with the
// file's pieces. When the file is in more pieces than the table holds, it
// says how many words it would need and leaves the half-made table on the
// file: a seek past the last piece in it then goes astray, and the archive
// reads the wrong bytes. So its answer is always looked at, and a file it
// failed on is never kept: closing it frees that table (libnds 1.25.0), and
// the file is opened again, clean.
//
// First a table for one piece (16 bytes): most archives on a card are in
// one. Then, for one in more, once more with a table of the size asked for,
// if that fits NEA_NPAC_SEEK_TABLE_DS or _DSI. Otherwise no table.
static ne_npac_mount_t *ne_npac_find(const char *name);

static int ne_npac_open_seekable(ne_npac_mount_t *m, const char *path)
{
    m->fast_seek = false;
    m->seek_state = NEA_NPAC_SEEK_NOT_FAT;
    m->fragments = -1;
    m->seek_table = 0;
    m->plain_reopens = 0;

    int fd = open(path, O_RDONLY, 0);
    if (fd < 0)
        return -1;

    int need = fatInitLookupCache(fd, 16);
    if (need == 0)
    {
        m->fast_seek = true;
        m->seek_state = NEA_NPAC_SEEK_FAST;
        m->fragments = 1;
        m->seek_table = 16;
        return fd;
    }
    if (need < 0)
    {
        // Not on FAT, or no memory for it (no table is left on the file).
        if (need == FAT_INIT_LOOKUP_CACHE_OUT_OF_MEMORY)
            m->seek_state = NEA_NPAC_SEEK_NO_MEMORY;
        return fd;
    }

    // In more pieces than one: 'need' words, two a piece and two more.
    m->fragments = (need - 2) / 2;
    close(fd);
    fd = open(path, O_RDONLY, 0);
    if (fd < 0)
        return -1;

    uint32_t most = isDSiMode() ? NEA_NPAC_SEEK_TABLE_DSI
                                : NEA_NPAC_SEEK_TABLE_DS;
    if ((uint32_t)need * 4 > most)
    {
        m->seek_state = NEA_NPAC_SEEK_TOO_FRAGMENTED;
        NEA_DebugPrint("%s is in %d pieces: no fast seek", path,
                       (int)m->fragments);
        return fd;
    }

    int again = fatInitLookupCache(fd, (uint32_t)need * 4);
    if (again == 0)
    {
        m->fast_seek = true;
        m->seek_state = NEA_NPAC_SEEK_FAST;
        m->seek_table = (uint32_t)need * 4;
        return fd;
    }

    // Still not (no memory, or the file changed between the two): never
    // kept with what was left on it.
    m->seek_state = (again == FAT_INIT_LOOKUP_CACHE_OUT_OF_MEMORY)
                  ? NEA_NPAC_SEEK_NO_MEMORY : NEA_NPAC_SEEK_TOO_FRAGMENTED;
    NEA_DebugPrint("%s: no fast seek (%d)", path, again);
    close(fd);
    return open(path, O_RDONLY, 0);
}

int NEA_NpacSeekInfo(const char *drive, NEA_NpacSeek *info)
{
    NEA_AssertPointer(info, "NULL info");

    ne_npac_mount_t *m = ne_npac_find(drive);
    if ((m == NULL) || (info == NULL))
    {
        errno = ENOENT;
        return -1;
    }

    info->fast = m->fast_seek;
    info->state = (NEA_NpacSeekState)m->seek_state;
    info->fragments = m->fragments;
    info->table_bytes = m->seek_table;
    info->plain_reopens = m->plain_reopens;
    return 0;
}

static void ne_npac_release(ne_npac_mount_t *m)
{
    if (m->fd >= 0)
        close(m->fd);

    if (m->index_buf == NULL)
    {
        free(m->fat);
        free(m->fnt);
        free(m->cmp);
    }

    uint32_t generation = m->generation + 1;

    memset(m, 0, sizeof(*m));
    m->fd = -1;
    m->generation = generation;
}

// True while a handle still refers to the archive it was opened on. An
// unmount frees the index the handle reads through, and the slot can then be
// taken by a different archive, so this is checked before every access rather
// than trusted.
static bool ne_npac_handle_valid(const ne_npac_mount_t *mnt, uint32_t generation)
{
    return mnt->used && (mnt->generation == generation);
}

//-----------------------------------------------------------------------------
// Walking the name table
//-----------------------------------------------------------------------------

// The name table's bytes from "pos" on, "len" of them (at most 130: an
// entry's type, a 127 byte name and a directory ID). A mounted archive has the
// table in RAM. NEA_NpacPeek() has not: its bytes come through this window,
// filled from the file, so a lookup costs no memory. The pointer is good until
// the next call.
static uint8_t ne_npac_fnt_window[256] __attribute__((aligned(4)));
static const ne_npac_mount_t *ne_npac_fnt_owner = NULL;
static uint32_t ne_npac_fnt_from;
static uint32_t ne_npac_fnt_fill;

static const uint8_t *ne_npac_fnt_at(const ne_npac_mount_t *m, uint32_t pos,
                                     uint32_t len)
{
    if (m->fnt != NULL)
        return m->fnt + pos;

    if ((ne_npac_fnt_owner == m) && (pos >= ne_npac_fnt_from) &&
        (pos + len <= ne_npac_fnt_from + ne_npac_fnt_fill))
        return ne_npac_fnt_window + (pos - ne_npac_fnt_from);

    if (pos > m->fnt_size)
        return NULL;
    uint32_t n = m->fnt_size - pos;
    if (n > sizeof(ne_npac_fnt_window))
        n = sizeof(ne_npac_fnt_window);
    if (n < len)
        return NULL;

    if (ne_npac_read_at((ne_npac_mount_t *)m, m->fnt_file + pos,
                        ne_npac_fnt_window, n) != 0)
    {
        ne_npac_fnt_owner = NULL;
        return NULL;
    }

    ne_npac_fnt_owner = m;
    ne_npac_fnt_from = pos;
    ne_npac_fnt_fill = n;
    return ne_npac_fnt_window;
}

static const uint8_t *ne_npac_dir_entry(const ne_npac_mount_t *m, uint16_t dir_id)
{
    uint32_t index = dir_id & 0x0FFF;

    if (index >= m->num_dirs)
        return NULL;

    return ne_npac_fnt_at(m, index * 8, 8);
}

static void ne_npac_iter_init(ne_npac_iter_t *it, const ne_npac_mount_t *m,
                              uint16_t dir_id)
{
    const uint8_t *entry = ne_npac_dir_entry(m, dir_id);

    it->mnt = m;

    if (entry == NULL)
    {
        // Past the end of the name table, so the iterator yields nothing.
        it->pos = m->fnt_size;
        it->file_index = 0;
        return;
    }

    it->pos = ne_rd32(entry);
    it->file_index = ne_rd16(entry + 4);
}

// Returns 0 at the end of the subtable, 1 for a file, 2 for a directory. The
// name points into the name table and is not terminated.
static int ne_npac_iter_next(ne_npac_iter_t *it, const char **name,
                             uint32_t *name_len, uint16_t *id)
{
    const ne_npac_mount_t *m = it->mnt;

    if (it->pos >= m->fnt_size)
        return 0;

    const uint8_t *at = ne_npac_fnt_at(m, it->pos, 1);
    if (at == NULL)
        return 0;

    uint8_t type = at[0];

    // 00h ends the subtable. 80h is reserved and has no length, so treat it as
    // the end too rather than walking off into the next directory.
    if ((type == 0x00) || (type == 0x80))
        return 0;

    uint32_t len = type & 0x7F;
    bool is_dir = (type & 0x80) != 0;
    uint32_t total = 1 + len + (is_dir ? 2 : 0);

    if (it->pos + total > m->fnt_size)
        return 0;

    at = ne_npac_fnt_at(m, it->pos, total);
    if (at == NULL)
        return 0;

    *name = (const char *)(at + 1);
    *name_len = len;

    if (is_dir)
    {
        *id = ne_rd16(at + 1 + len);
        it->pos += total;
        return 2;
    }

    // File IDs are not stored anywhere; they run in order through the file
    // entries of a subtable, starting at the directory's first file.
    *id = it->file_index++;
    it->pos += total;
    return 1;
}

// Resolves a path relative to the mount. Returns a file ID, a directory ID
// (NPAC_DIR_ROOT and up), or -1 with errno set.
//
// The path is never written to. libnds hands over the caller's own buffer, so
// the in-place tokenising NitroFS does would be writing into a string the
// caller may well have made const.
static int ne_npac_resolve(const ne_npac_mount_t *m, const char *path)
{
    uint16_t dir = m->current_dir;
    const char *p = path;

    if (*p == '/')
    {
        dir = NPAC_DIR_ROOT;
        while (*p == '/')
            p++;
    }

    while (*p != '\0')
    {
        const char *start = p;
        while ((*p != '\0') && (*p != '/'))
            p++;

        size_t len = (size_t)(p - start);

        while (*p == '/')
            p++;

        bool last = (*p == '\0');

        if ((len == 0) || ((len == 1) && (start[0] == '.')))
            continue;

        if ((len == 2) && (start[0] == '.') && (start[1] == '.'))
        {
            const uint8_t *entry = ne_npac_dir_entry(m, dir);
            if (entry == NULL)
            {
                errno = ENOENT;
                return -1;
            }
            if (dir != NPAC_DIR_ROOT)
                dir = ne_rd16(entry + 6);
            continue;
        }

        ne_npac_iter_t it;
        ne_npac_iter_init(&it, m, dir);

        const char *name;
        uint32_t name_len;
        uint16_t id;
        int kind;
        bool found = false;

        while ((kind = ne_npac_iter_next(&it, &name, &name_len, &id)) != 0)
        {
            if ((name_len == len) && (memcmp(name, start, len) == 0))
            {
                found = true;
                break;
            }
        }

        if (!found)
        {
            errno = ENOENT;
            return -1;
        }

        if (kind == 1)
        {
            if (!last)
            {
                errno = ENOTDIR;
                return -1;
            }
            return id;
        }

        dir = id;
    }

    return dir;
}

// Finds a directory's own name by looking for it in its parent's subtable.
// Only getcwd() needs this, so the linear scan is not worth avoiding.
static int ne_npac_dir_name(const ne_npac_mount_t *m, uint16_t dir_id,
                            const char **name, uint32_t *name_len)
{
    const uint8_t *entry = ne_npac_dir_entry(m, dir_id);

    if ((entry == NULL) || (dir_id == NPAC_DIR_ROOT))
        return -1;

    ne_npac_iter_t it;
    ne_npac_iter_init(&it, m, ne_rd16(entry + 6));

    const char *n;
    uint32_t nl;
    uint16_t id;
    int kind;

    while ((kind = ne_npac_iter_next(&it, &n, &nl, &id)) != 0)
    {
        if ((kind == 2) && (id == dir_id))
        {
            *name = n;
            *name_len = nl;
            return 0;
        }
    }

    return -1;
}

//-----------------------------------------------------------------------------
// Mount lookup and path splitting
//-----------------------------------------------------------------------------

static ne_npac_mount_t *ne_npac_find(const char *name)
{
    if (name == NULL)
        return NULL;

    for (int i = 0; i < NEA_NPAC_MAX_MOUNTS; i++)
    {
        ne_npac_mount_t *m = &ne_npac_mounts[i];
        if (m->used && (strcmp(m->name, name) == 0))
            return m;
    }

    return NULL;
}

// Splits "drive:/rest" into the mount and the part after the colon. Paths
// arrive here with the drive still attached: libnds strips it for chdir() but
// passes it through untouched to open(), stat() and opendir().
static ne_npac_mount_t *ne_npac_split(const char *path, const char **rest)
{
    const char *colon = strchr(path, ':');
    const char *slash = strchr(path, '/');

    if ((colon != NULL) && ((slash == NULL) || (colon < slash)))
    {
        char name[NEA_NPAC_MAX_DRIVE_NAME + 1];
        size_t len = (size_t)(colon - path);

        if (len > NEA_NPAC_MAX_DRIVE_NAME)
        {
            errno = ENODEV;
            return NULL;
        }

        memcpy(name, path, len);
        name[len] = '\0';

        ne_npac_mount_t *m = ne_npac_find(name);
        if (m == NULL)
        {
            errno = ENODEV;
            return NULL;
        }

        *rest = colon + 1;
        return m;
    }

    if (ne_npac_current == NULL)
    {
        errno = ENODEV;
        return NULL;
    }

    *rest = path;
    return ne_npac_current;
}

static void ne_npac_file_extent(const ne_npac_mount_t *m, uint16_t id,
                                uint32_t *start, uint32_t *end)
{
    const uint8_t *entry = m->fat + (uint32_t)id * 8;

    *start = ne_rd32(entry);
    *end = ne_rd32(entry + 4);
}

static void ne_npac_file_meta(const ne_npac_mount_t *m, uint16_t id,
                              uint8_t *method, uint32_t *size)
{
    uint32_t start, end;
    ne_npac_file_extent(m, id, &start, &end);

    if (m->cmp == NULL)
    {
        // A NARC, or an NPAC where nothing turned out worth compressing.
        *method = NEA_NPAC_STORED;
        *size = end - start;
        return;
    }

    uint32_t word = ne_rd32(m->cmp + (uint32_t)id * 4);

    *method = word & 0xFF;
    *size = (*method == NEA_NPAC_STORED) ? (end - start) : (word >> 8);
}

static void ne_npac_fill_stat(struct stat *st, uint16_t id, uint32_t size,
                              bool is_dir)
{
    memset(st, 0, sizeof(*st));

    st->st_ino = id;
    st->st_nlink = 1;
    st->st_size = is_dir ? 0 : (off_t)size;
    st->st_blksize = 512;
    st->st_blocks = (st->st_size + 511) / 512;
    st->st_mode = is_dir ? (S_IFDIR | 0555) : (S_IFREG | 0444);
}

//-----------------------------------------------------------------------------
// Device I/O callbacks: files
//-----------------------------------------------------------------------------

static bool ne_npac_isdrive(const char *name)
{
    return ne_npac_find(name) != NULL;
}

static int ne_npac_open(const char *path, int flags, mode_t mode)
{
    (void)mode;

    if (flags & (O_WRONLY | O_RDWR | O_CREAT | O_TRUNC | O_APPEND))
    {
        errno = EROFS;
        return -1;
    }

    const char *rest;
    ne_npac_mount_t *m = ne_npac_split(path, &rest);
    if (m == NULL)
        return -1;

    int id = ne_npac_resolve(m, rest);
    if (id < 0)
        return -1;

    if (id >= NPAC_DIR_ROOT)
    {
        errno = EISDIR;
        return -1;
    }

    uint32_t start, end;
    ne_npac_file_extent(m, (uint16_t)id, &start, &end);

    if ((start > end) || (end > m->img_size))
    {
        NEA_DebugPrint("NPAC file %d is outside the image", id);
        errno = EINVAL;
        return -1;
    }

    uint8_t method;
    uint32_t size;
    ne_npac_file_meta(m, (uint16_t)id, &method, &size);

    ne_npac_file_t *f = ne_npac_file_new();
    if (f == NULL)
    {
        NEA_DebugPrint("Not enough memory");
        errno = ENOMEM;
        return -1;
    }

    f->mnt = m;
    f->generation = m->generation;
    f->file_index = (uint16_t)id;
    f->offset = m->img_offset + start;
    f->size = size;

    switch (method)
    {
        case NEA_NPAC_STORED:
        case NEA_NPAC_LZ77:
        case NEA_NPAC_HUFF4:
        case NEA_NPAC_HUFF8:
        case NEA_NPAC_RLE:
            break;
        default:
            NEA_DebugPrint("Unknown NPAC compression 0x%02X", method);
            ne_npac_file_delete(f);
            errno = EINVAL;
            return -1;
    }

    // A packed member is inflated by its first read, not here: what opens a
    // file only to ask its size (fseek to the end, ftell, fclose) costs no
    // I/O at all.
    f->method = method;
    f->stored = end - start;

    // libnds tags the top four bits and the bottom two with the device index,
    // and crashes on a descriptor that has already used them.
    int fd = (int)(intptr_t)f;
    if ((fd & 0xF0000003) != 0)
    {
        NEA_DebugPrint("Misaligned NPAC file descriptor");
        ne_npac_file_delete(f);
        errno = ENOMEM;
        return -1;
    }

    return fd;
}

static int ne_npac_close(int fd)
{
    ne_npac_file_t *f = (ne_npac_file_t *)fd;

    free(f->ram);
    ne_npac_file_delete(f);

    return 0;
}

// Inflates a packed member into RAM, where its reads are served from until
// close(). The BIOS decompressors want the whole stream in one piece.
static int ne_npac_inflate(ne_npac_file_t *f)
{
    DecompressType type;

    switch (f->method)
    {
        case NEA_NPAC_LZ77:
            type = LZ77;
            break;
        case NEA_NPAC_HUFF4:
        case NEA_NPAC_HUFF8:
            type = HUFF;
            break;
        default:
            type = RLE;
            break;
    }

    uint8_t *src = malloc(f->stored ? f->stored : 1);
    if (src == NULL)
    {
        NEA_DebugPrint("Not enough memory");
        errno = ENOMEM;
        return -1;
    }

    if (ne_npac_read_at(f->mnt, f->offset, src, f->stored) != 0)
    {
        free(src);
        errno = EIO;
        return -1;
    }

    uint8_t *dst = malloc(f->size ? f->size : 1);
    if (dst == NULL)
    {
        NEA_DebugPrint("Not enough memory");
        free(src);
        errno = ENOMEM;
        return -1;
    }

    decompress(src, dst, type);
    free(src);

    f->ram = dst;
    return 0;
}

static ssize_t ne_npac_read_cb(int fd, void *ptr, size_t len)
{
    ne_npac_file_t *f = (ne_npac_file_t *)fd;

    if (!ne_npac_handle_valid(f->mnt, f->generation))
    {
        errno = EBADF;
        return -1;
    }

    if (f->position >= f->size)
        return 0;

    size_t available = f->size - f->position;
    if (len > available)
        len = available;

    if ((f->method != NEA_NPAC_STORED) && (f->ram == NULL) &&
        (ne_npac_inflate(f) != 0))
        return -1;

    if (f->ram != NULL)
    {
        memcpy(ptr, f->ram + f->position, len);
    }
    else if (ne_npac_read_at(f->mnt, f->offset + f->position, ptr, len) != 0)
    {
        errno = EIO;
        return -1;
    }

    f->position += len;

    return (ssize_t)len;
}

static off_t ne_npac_lseek(int fd, off_t pos, int whence)
{
    ne_npac_file_t *f = (ne_npac_file_t *)fd;
    off_t base;

    if (!ne_npac_handle_valid(f->mnt, f->generation))
    {
        errno = EBADF;
        return -1;
    }

    switch (whence)
    {
        case SEEK_SET:
            base = 0;
            break;
        case SEEK_CUR:
            base = (off_t)f->position;
            break;
        case SEEK_END:
            base = (off_t)f->size;
            break;
        default:
            errno = EINVAL;
            return -1;
    }

    off_t position = base + pos;

    if (position < 0)
    {
        errno = EINVAL;
        return -1;
    }

    if (position > (off_t)f->size)
        position = (off_t)f->size;

    f->position = (uint32_t)position;

    return position;
}

static int ne_npac_fstat(int fd, struct stat *st)
{
    ne_npac_file_t *f = (ne_npac_file_t *)fd;

    if (!ne_npac_handle_valid(f->mnt, f->generation))
    {
        errno = EBADF;
        return -1;
    }

    ne_npac_fill_stat(st, f->file_index, f->size, false);

    return 0;
}

static int ne_npac_stat(const char *path, struct stat *st)
{
    const char *rest;
    ne_npac_mount_t *m = ne_npac_split(path, &rest);
    if (m == NULL)
        return -1;

    int id = ne_npac_resolve(m, rest);
    if (id < 0)
        return -1;

    if (id >= NPAC_DIR_ROOT)
    {
        ne_npac_fill_stat(st, (uint16_t)id, 0, true);
        return 0;
    }

    uint8_t method;
    uint32_t size;
    ne_npac_file_meta(m, (uint16_t)id, &method, &size);

    ne_npac_fill_stat(st, (uint16_t)id, size, false);

    return 0;
}

static int ne_npac_access(const char *path, int amode)
{
    if (amode & W_OK)
    {
        errno = EROFS;
        return -1;
    }

    const char *rest;
    ne_npac_mount_t *m = ne_npac_split(path, &rest);
    if (m == NULL)
        return -1;

    return (ne_npac_resolve(m, rest) < 0) ? -1 : 0;
}

static int ne_npac_get_attr(const char *path)
{
    const char *rest;
    ne_npac_mount_t *m = ne_npac_split(path, &rest);
    if (m == NULL)
        return -1;

    int id = ne_npac_resolve(m, rest);
    if (id < 0)
        return -1;

    return ATTR_READONLY | ((id >= NPAC_DIR_ROOT) ? ATTR_DIRECTORY : 0);
}

static int ne_npac_set_attr(const char *path, uint8_t attr)
{
    (void)path;
    (void)attr;

    errno = EROFS;
    return -1;
}

//-----------------------------------------------------------------------------
// Device I/O callbacks: directories
//-----------------------------------------------------------------------------

static void *ne_npac_opendir(const char *path, DIR *dirp)
{
    (void)dirp;

    const char *rest;
    ne_npac_mount_t *m = ne_npac_split(path, &rest);
    if (m == NULL)
        return NULL;

    int id = ne_npac_resolve(m, rest);
    if (id < 0)
        return NULL;

    if (id < NPAC_DIR_ROOT)
    {
        errno = ENOTDIR;
        return NULL;
    }

    ne_npac_dir_t *d = calloc(1, sizeof(ne_npac_dir_t));
    if (d == NULL)
    {
        NEA_DebugPrint("Not enough memory");
        errno = ENOMEM;
        return NULL;
    }

    const uint8_t *entry = ne_npac_dir_entry(m, (uint16_t)id);

    d->mnt = m;
    d->generation = m->generation;
    d->dir_id = (uint16_t)id;
    d->parent_id = (id == NPAC_DIR_ROOT) ? NPAC_DIR_ROOT : ne_rd16(entry + 6);
    ne_npac_iter_init(&d->it, m, (uint16_t)id);

    return d;
}

static int ne_npac_closedir(DIR *dirp)
{
    // libnds frees the DIR itself, only the state allocated by opendir() is
    // ours to release.
    free(dirp->dp);

    return 0;
}

static struct dirent *ne_npac_readdir(DIR *dirp)
{
    ne_npac_dir_t *d = dirp->dp;
    struct dirent *ent = &dirp->dirent;

    // The iterator points into the name table, which an unmount has freed.
    if (!ne_npac_handle_valid(d->mnt, d->generation))
    {
        errno = EBADF;
        return NULL;
    }

    if (d->phase == 0)
    {
        strcpy(ent->d_name, ".");
        ent->d_type = DT_DIR;
        ent->d_ino = d->dir_id;
        d->phase = 1;
    }
    else if (d->phase == 1)
    {
        strcpy(ent->d_name, "..");
        ent->d_type = DT_DIR;
        ent->d_ino = d->parent_id;
        d->phase = 2;
    }
    else
    {
        const char *name;
        uint32_t name_len;
        uint16_t id;

        int kind = ne_npac_iter_next(&d->it, &name, &name_len, &id);
        if (kind == 0)
        {
            // End of the directory. POSIX wants errno left alone here.
            return NULL;
        }

        if (name_len >= sizeof(ent->d_name))
            name_len = sizeof(ent->d_name) - 1;

        memcpy(ent->d_name, name, name_len);
        ent->d_name[name_len] = '\0';
        ent->d_type = (kind == 2) ? DT_DIR : DT_REG;
        ent->d_ino = id;
    }

    ent->d_off = d->index;
    d->index++;
    dirp->index = d->index;

    return ent;
}

static void ne_npac_rewinddir(DIR *dirp)
{
    ne_npac_dir_t *d = dirp->dp;

    if (!ne_npac_handle_valid(d->mnt, d->generation))
    {
        errno = EBADF;
        return;
    }

    d->phase = 0;
    d->index = 0;
    dirp->index = 0;
    ne_npac_iter_init(&d->it, d->mnt, d->dir_id);
}

static void ne_npac_seekdir(DIR *dirp, long loc)
{
    ne_npac_dir_t *d = dirp->dp;

    ne_npac_rewinddir(dirp);

    while (d->index < loc)
    {
        if (ne_npac_readdir(dirp) == NULL)
            break;
    }
}

static long ne_npac_telldir(DIR *dirp)
{
    ne_npac_dir_t *d = dirp->dp;

    return d->index;
}

//-----------------------------------------------------------------------------
// Device I/O callbacks: working directory
//-----------------------------------------------------------------------------

static int ne_npac_chdrive(const char *drive)
{
    char name[NEA_NPAC_MAX_DRIVE_NAME + 1];
    size_t len = 0;

    // Accept the name with or without its colon; both spellings turn up.
    while ((drive[len] != '\0') && (drive[len] != ':') &&
           (len < NEA_NPAC_MAX_DRIVE_NAME))
    {
        name[len] = drive[len];
        len++;
    }
    name[len] = '\0';

    ne_npac_mount_t *m = ne_npac_find(name);
    if (m == NULL)
    {
        errno = ENODEV;
        return -1;
    }

    ne_npac_current = m;

    return 0;
}

// This one does get the path without the drive, unlike open() and friends.
static int ne_npac_chdir(const char *path)
{
    if (ne_npac_current == NULL)
    {
        errno = ENODEV;
        return -1;
    }

    int id = ne_npac_resolve(ne_npac_current, path);
    if (id < 0)
        return -1;

    if (id < NPAC_DIR_ROOT)
    {
        errno = ENOTDIR;
        return -1;
    }

    ne_npac_current->current_dir = (uint16_t)id;

    return 0;
}

static int ne_npac_getcwd(char *buf, size_t size)
{
    ne_npac_mount_t *m = ne_npac_current;

    if (m == NULL)
    {
        errno = ENODEV;
        return -1;
    }

    // Walk up to the root collecting IDs, because a directory only knows its
    // parent, then walk back down naming each one.
    uint16_t stack[NPAC_MAX_DEPTH];
    int depth = 0;
    uint16_t dir = m->current_dir;

    while ((dir != NPAC_DIR_ROOT) && (depth < NPAC_MAX_DEPTH))
    {
        const uint8_t *entry = ne_npac_dir_entry(m, dir);
        if (entry == NULL)
            break;

        stack[depth++] = dir;
        dir = ne_rd16(entry + 6);
    }

    size_t used = 0;
    size_t name_len = strlen(m->name);

    if (used + name_len + 2 >= size)
    {
        errno = ERANGE;
        return -1;
    }

    memcpy(buf + used, m->name, name_len);
    used += name_len;
    buf[used++] = ':';
    buf[used++] = '/';

    for (int i = depth - 1; i >= 0; i--)
    {
        const char *name;
        uint32_t len;

        if (ne_npac_dir_name(m, stack[i], &name, &len) != 0)
            break;

        if (used + len + 2 >= size)
        {
            errno = ERANGE;
            return -1;
        }

        memcpy(buf + used, name, len);
        used += len;

        if (i > 0)
            buf[used++] = '/';
    }

    buf[used] = '\0';

    return 0;
}

//-----------------------------------------------------------------------------
// The device
//-----------------------------------------------------------------------------

// Everything left out is a write operation, and libnds fails those with
// ENOSYS on its own. An archive is read-only by construction.
static const device_io_t ne_npac_device = {
    .isdrive = ne_npac_isdrive,

    .open = ne_npac_open,
    .close = ne_npac_close,
    .read = ne_npac_read_cb,
    .lseek = ne_npac_lseek,

    .fstat = ne_npac_fstat,
    .stat = ne_npac_stat,
    .lstat = ne_npac_stat,
    .access = ne_npac_access,

    .chdir = ne_npac_chdir,
    .getcwd = ne_npac_getcwd,
    .chdrive = ne_npac_chdrive,

    .opendir = ne_npac_opendir,
    .closedir = ne_npac_closedir,
    .readdir = ne_npac_readdir,
    .rewinddir = ne_npac_rewinddir,
    .seekdir = ne_npac_seekdir,
    .telldir = ne_npac_telldir,

    .get_attr = ne_npac_get_attr,
    .set_attr = ne_npac_set_attr,
};

//-----------------------------------------------------------------------------
// Public API
//-----------------------------------------------------------------------------

// Names libnds resolves itself, before any user device is consulted. Mounting
// under one of them would succeed and then be unreachable.
static const char *ne_npac_reserved[] = {
    "nitro", "fat", "sd", "nand", "nand2"
};

static int ne_npac_mount(const char *drive, const char *path, void *index,
                         size_t index_size)
{
    NEA_AssertPointer(drive, "NULL drive name");
    NEA_AssertPointer(path, "NULL path");

    if ((drive == NULL) || (path == NULL))
    {
        errno = EINVAL;
        return -1;
    }

    size_t len = strlen(drive);
    if ((len == 0) || (len > NEA_NPAC_MAX_DRIVE_NAME))
    {
        NEA_DebugPrint("Bad NPAC drive name");
        errno = EINVAL;
        return -1;
    }

    for (size_t i = 0; i < sizeof(ne_npac_reserved) / sizeof(char *); i++)
    {
        if (strcmp(drive, ne_npac_reserved[i]) == 0)
        {
            NEA_DebugPrint("%s is reserved by libnds", drive);
            errno = EINVAL;
            return -1;
        }
    }

    if (ne_npac_find(drive) != NULL)
    {
        NEA_DebugPrint("%s is already mounted", drive);
        errno = EBUSY;
        return -1;
    }

    ne_npac_mount_t *m = NULL;
    for (int i = 0; i < NEA_NPAC_MAX_MOUNTS; i++)
    {
        if (!ne_npac_mounts[i].used)
        {
            m = &ne_npac_mounts[i];
            break;
        }
    }

    if (m == NULL)
    {
        NEA_DebugPrint("No free NPAC mount slots");
        errno = EMFILE;
        return -1;
    }

    uint32_t generation = m->generation;
    memset(m, 0, sizeof(*m));
    m->generation = generation;
    m->index_buf = index;
    m->index_size = index_size;

    if (strlen(path) < NPAC_PATH_MAX)
    {
        char *copy = ne_npac_paths[m - ne_npac_mounts];
        strcpy(copy, path);
        m->path = copy;
    }

    m->fd = ne_npac_open_seekable(m, path);
    if (m->fd < 0)
    {
        NEA_DebugPrint("%s couldn't be opened", path);
        m->fd = -1;
        return -1;
    }

    if (ne_npac_load_index(m) != 0)
    {
        int saved = errno;
        ne_npac_release(m);
        errno = saved;
        return -1;
    }

    strcpy(m->name, drive);
    m->current_dir = NPAC_DIR_ROOT;
    m->used = true;

    // One device covers every mount, so it is registered once and stays until
    // NEA_NpacSystemEnd(). Removing it when the last archive is unmounted
    // would leave libnds' current drive pointing at a device that is gone.
    if (ne_npac_device_index < 0)
    {
        ne_npac_device_index = deviceIoAdd(&ne_npac_device);
        if (ne_npac_device_index < 0)
        {
            NEA_DebugPrint("Couldn't register the NPAC filesystem");
            ne_npac_release(m);
            errno = EMFILE;
            return -1;
        }
    }

    if (ne_npac_current == NULL)
        ne_npac_current = m;

    return 0;
}

int NEA_NpacMount(const char *drive, const char *path)
{
    return ne_npac_mount(drive, path, NULL, 0);
}

int NEA_NpacMountInto(const char *drive, const char *path, void *index,
                      size_t size)
{
    NEA_AssertPointer(index, "NULL index memory");

    if ((index == NULL) || (((uintptr_t)index & 3) != 0))
    {
        errno = EINVAL;
        return -1;
    }

    return ne_npac_mount(drive, path, index, size);
}

static int ne_npac_read_member(ne_npac_mount_t *m, uint32_t offset,
                               uint32_t size, uint32_t stored, uint8_t method,
                               uint32_t from, void *dst, uint32_t len);

// Where an archive's chunks are, from their headers alone.
typedef struct {
    uint32_t fat;           // File table entries (after the count)
    uint32_t files;
    uint32_t fnt, fnt_size;
    uint32_t cmp;           // Compression table entries, 0 if none
    uint32_t img, img_size;
} ne_npac_chunks_t;

static int ne_npac_scan(ne_npac_mount_t *m, ne_npac_chunks_t *c)
{
    uint8_t header[16];

    memset(c, 0, sizeof(*c));

    if (ne_npac_read_at(m, 0, header, sizeof(header)) != 0)
    {
        errno = EIO;
        return -1;
    }
    if (((memcmp(header, "NPAC", 4) != 0) && (memcmp(header, "NARC", 4) != 0)) ||
        (ne_rd16(header + 4) != 0xFFFE))
    {
        errno = EINVAL;
        return -1;
    }

    uint32_t pos = ne_rd16(header + 12);
    uint16_t num_chunks = ne_rd16(header + 14);
    bool img_is_last = false;

    for (uint16_t i = 0; i < num_chunks; i++)
    {
        uint8_t chunk[8];
        if (ne_npac_read_at(m, pos, chunk, sizeof(chunk)) != 0)
        {
            errno = EIO;
            return -1;
        }

        uint32_t size = ne_rd32(chunk + 4);
        if (size < sizeof(chunk))
        {
            errno = EINVAL;
            return -1;
        }

        uint32_t payload = pos + sizeof(chunk);

        if (memcmp(chunk, "BTAF", 4) == 0)
        {
            uint8_t count[4];
            if (ne_npac_read_at(m, payload, count, sizeof(count)) != 0)
            {
                errno = EIO;
                return -1;
            }
            c->files = ne_rd16(count);
            c->fat = payload + 4;
        }
        else if (memcmp(chunk, "BTNF", 4) == 0)
        {
            c->fnt = payload;
            c->fnt_size = size - sizeof(chunk);
        }
        else if (memcmp(chunk, "CMPT", 4) == 0)
        {
            c->cmp = payload + 4;
        }
        else if (memcmp(chunk, "GMIF", 4) == 0)
        {
            c->img = payload;
            c->img_size = size - sizeof(chunk);
            img_is_last = (i + 1) == num_chunks;
        }

        pos += (size + 3) & ~3u;
    }

    if ((c->fat == 0) || (c->fnt == 0) || (c->img == 0))
    {
        errno = EINVAL;
        return -1;
    }

    // As ne_npac_load_index(): the file's length is the image's honest end.
    if (img_is_last)
    {
        ne_npac_io_lock(m);
        off_t end = lseek(m->fd, 0, SEEK_END);
        ne_npac_io_unlock(m);

        if ((end > 0) && ((uint32_t)end > c->img))
            c->img_size = (uint32_t)end - c->img;
    }

    return 0;
}

int NEA_NpacIndexBytes(const char *path)
{
    NEA_AssertPointer(path, "NULL path");

    if (path == NULL)
    {
        errno = EINVAL;
        return -1;
    }

    ne_npac_mount_t m;
    memset(&m, 0, sizeof(m));
    m.path = path;
    m.fd = open(path, O_RDONLY, 0);
    if (m.fd < 0)
        return -1;

    ne_npac_chunks_t c;
    int result = -1;

    // As ne_npac_read_chunk() cuts them: each table rounded up to a word.
    if (ne_npac_scan(&m, &c) == 0)
        result = (int)(((c.files * 8 + 3) & ~3u) + ((c.fnt_size + 3) & ~3u) +
                       ((c.cmp != 0) ? ((c.files * 4 + 3) & ~3u) : 0));

    int saved = errno;
    if (m.fd >= 0)
        close(m.fd);
    errno = saved;
    return result;
}

int NEA_NpacPeek(const char *archive, const char *member, uint32_t from,
                 void *dst, uint32_t len)
{
    NEA_AssertPointer(archive, "NULL archive");
    NEA_AssertPointer(member, "NULL member");

    if ((archive == NULL) || (member == NULL) || ((dst == NULL) && (len != 0)))
    {
        errno = EINVAL;
        return -1;
    }

    // A mount of its own on the stack, never registered: its name table is
    // read through the window as the path is looked up, nothing else is kept.
    ne_npac_mount_t m;
    memset(&m, 0, sizeof(m));
    m.path = archive;
    m.fd = open(archive, O_RDONLY, 0);
    if (m.fd < 0)
        return -1;

    int result = -1;
    ne_npac_chunks_t c;
    uint8_t word[8];

    if (ne_npac_scan(&m, &c) != 0)
        goto out;

    m.fnt_file = c.fnt;
    m.fnt_size = c.fnt_size;
    m.num_files = (uint16_t)c.files;
    m.img_offset = c.img;
    m.img_size = c.img_size;
    m.current_dir = NPAC_DIR_ROOT;

    const uint8_t *root = ne_npac_fnt_at(&m, 0, 8);
    if (root == NULL)
    {
        errno = EIO;
        goto out;
    }
    m.num_dirs = ne_rd16(root + 6);
    if ((m.num_dirs == 0) || ((uint32_t)m.num_dirs * 8 > m.fnt_size))
    {
        errno = EINVAL;
        goto out;
    }

    int id = ne_npac_resolve(&m, member);
    if (id < 0)
        goto out;
    if (id >= NPAC_DIR_ROOT)
    {
        errno = EISDIR;
        goto out;
    }
    if ((uint32_t)id >= c.files)
    {
        errno = EINVAL;
        goto out;
    }

    if (ne_npac_read_at(&m, c.fat + (uint32_t)id * 8, word, 8) != 0)
    {
        errno = EIO;
        goto out;
    }
    uint32_t start = ne_rd32(word);
    uint32_t end = ne_rd32(word + 4);
    if ((start > end) || (end > c.img_size))
    {
        errno = EINVAL;
        goto out;
    }

    uint8_t method = NEA_NPAC_STORED;
    uint32_t size = end - start;
    if (c.cmp != 0)
    {
        if (ne_npac_read_at(&m, c.cmp + (uint32_t)id * 4, word, 4) != 0)
        {
            errno = EIO;
            goto out;
        }
        uint32_t cmp = ne_rd32(word);
        method = cmp & 0xFF;
        if (method != NEA_NPAC_STORED)
            size = cmp >> 8;
    }

    result = ne_npac_read_member(&m, c.img + start, size, end - start, method,
                                 from, dst, len);

out:
    {
        // The window may hold this stack mount's bytes: forget them.
        ne_npac_fnt_owner = NULL;
        int saved = errno;
        if (m.fd >= 0)
            close(m.fd);
        errno = saved;
    }
    return result;
}

int NEA_NpacUnmount(const char *drive)
{
    NEA_AssertPointer(drive, "NULL drive name");

    ne_npac_mount_t *m = ne_npac_find(drive);
    if (m == NULL)
    {
        errno = ENODEV;
        return -1;
    }

    ne_npac_release(m);

    if (ne_npac_current == m)
    {
        ne_npac_current = NULL;

        for (int i = 0; i < NEA_NPAC_MAX_MOUNTS; i++)
        {
            if (ne_npac_mounts[i].used)
            {
                ne_npac_current = &ne_npac_mounts[i];
                break;
            }
        }
    }

    return 0;
}

bool NEA_NpacIsMounted(const char *drive)
{
    return ne_npac_find(drive) != NULL;
}

int NEA_NpacGetFileInfo(const char *path, NEA_NpacFileInfo *info)
{
    NEA_AssertPointer(path, "NULL path");
    NEA_AssertPointer(info, "NULL info");

    if ((path == NULL) || (info == NULL))
    {
        errno = EINVAL;
        return -1;
    }

    const char *rest;
    ne_npac_mount_t *m = ne_npac_split(path, &rest);
    if (m == NULL)
        return -1;

    int id = ne_npac_resolve(m, rest);
    if (id < 0)
        return -1;

    if (id >= NPAC_DIR_ROOT)
    {
        errno = EISDIR;
        return -1;
    }

    uint32_t start, end;
    ne_npac_file_extent(m, (uint16_t)id, &start, &end);

    uint8_t method;
    uint32_t size;
    ne_npac_file_meta(m, (uint16_t)id, &method, &size);

    info->size = size;
    info->stored_size = end - start;
    info->file_id = (uint16_t)id;
    info->method = method;

    return 0;
}

//-----------------------------------------------------------------------------
// Reading a member straight into the caller's buffer
//-----------------------------------------------------------------------------

// The packed bytes of a member come in through this window, a piece at a
// time, so a decode needs no heap and no read is longer than the window.
#define NPAC_WINDOW_BYTES   512

typedef struct {
    ne_npac_mount_t *mnt;
    uint32_t next;          // Where the next piece starts in the archive file
    uint32_t left;          // Packed bytes not read into the window yet
    uint32_t at;
    uint32_t fill;
    bool failed;            // No more bytes: the stream ran out, or the card
    bool card_failed;       // ...the card would not give them
} ne_npac_source_t;

static uint8_t ne_npac_window[NPAC_WINDOW_BYTES] __attribute__((aligned(4)));
static volatile bool ne_npac_window_busy = false;

static uint8_t ne_npac_source_byte(ne_npac_source_t *src)
{
    if (src->at == src->fill)
    {
        uint32_t piece = (src->left < NPAC_WINDOW_BYTES) ? src->left
                                                         : NPAC_WINDOW_BYTES;
        if (piece == 0)
        {
            src->failed = true;
            return 0;
        }
        if (ne_npac_read_at(src->mnt, src->next, ne_npac_window, piece) != 0)
        {
            src->failed = true;
            src->card_failed = true;
            return 0;
        }
        src->next += piece;
        src->left -= piece;
        src->at = 0;
        src->fill = piece;
    }

    return ne_npac_window[src->at++];
}

// LZ77 (the BIOS's LZ10): a flag byte for every eight blocks, high bit first;
// a set bit is a copy of 3..18 bytes from 1..4096 bytes back, a clear one a
// byte as it is.
static bool ne_npac_unlz77(ne_npac_source_t *src, uint8_t *out, uint32_t size)
{
    uint32_t done = 0;

    while (done < size)
    {
        uint8_t flags = ne_npac_source_byte(src);

        for (int bit = 0; (bit < 8) && (done < size); bit++, flags <<= 1)
        {
            if ((flags & 0x80) == 0)
            {
                out[done++] = ne_npac_source_byte(src);
                continue;
            }

            uint8_t hi = ne_npac_source_byte(src);
            uint8_t lo = ne_npac_source_byte(src);
            uint32_t back = ((uint32_t)(hi & 0x0F) << 8 | lo) + 1;
            uint32_t len = (uint32_t)(hi >> 4) + 3;

            if ((back > done) || src->failed)
                return false;
            if (len > size - done)
                len = size - done;

            const uint8_t *from = out + done - back;
            for (uint32_t i = 0; i < len; i++)
                out[done + i] = from[i];
            done += len;
        }

        if (src->failed)
            return false;
    }

    return !src->failed;
}

// RLE: a flag byte, then either 1..128 bytes as they are (high bit clear) or
// one byte said 3..130 times (high bit set).
static bool ne_npac_unrle(ne_npac_source_t *src, uint8_t *out, uint32_t size)
{
    uint32_t done = 0;

    while (done < size)
    {
        uint8_t flag = ne_npac_source_byte(src);
        uint32_t len = (flag & 0x80) ? (uint32_t)(flag & 0x7F) + 3
                                     : (uint32_t)(flag & 0x7F) + 1;
        if (len > size - done)
            len = size - done;

        if (flag & 0x80)
        {
            uint8_t value = ne_npac_source_byte(src);
            memset(out + done, value, len);
        }
        else
        {
            for (uint32_t i = 0; i < len; i++)
                out[done + i] = ne_npac_source_byte(src);
        }
        done += len;

        if (src->failed)
            return false;
    }

    return true;
}

int NEA_NpacMemberFind(const char *path, NEA_NpacMember *member)
{
    NEA_AssertPointer(path, "NULL path");
    NEA_AssertPointer(member, "NULL member");

    if ((path == NULL) || (member == NULL))
    {
        errno = EINVAL;
        return -1;
    }

    const char *rest;
    ne_npac_mount_t *m = ne_npac_split(path, &rest);
    if (m == NULL)
        return -1;

    int id = ne_npac_resolve(m, rest);
    if (id < 0)
        return -1;

    if (id >= NPAC_DIR_ROOT)
    {
        errno = EISDIR;
        return -1;
    }

    uint32_t start, end;
    ne_npac_file_extent(m, (uint16_t)id, &start, &end);

    if ((start > end) || (end > m->img_size))
    {
        NEA_DebugPrint("NPAC file %d is outside the image", id);
        errno = EINVAL;
        return -1;
    }

    uint8_t method;
    uint32_t size;
    ne_npac_file_meta(m, (uint16_t)id, &method, &size);

    member->generation = m->generation;
    member->offset = m->img_offset + start;
    member->size = size;
    member->stored_size = end - start;
    member->file_id = (uint16_t)id;
    member->mount = (uint8_t)(m - ne_npac_mounts);
    member->method = method;

    return 0;
}

// Reads a member at "offset" in the archive file: a stored one in any range, a
// packed one whole, decompressed as it comes through the window.
static int ne_npac_read_member(ne_npac_mount_t *m, uint32_t offset,
                               uint32_t size, uint32_t stored, uint8_t method,
                               uint32_t from, void *dst, uint32_t len)
{
    if (method == NEA_NPAC_STORED)
    {
        if ((from > size) || (len > size - from))
        {
            errno = EINVAL;
            return -1;
        }
        if (ne_npac_read_at(m, offset + from, dst, len) != 0)
        {
            errno = EIO;
            return -1;
        }
        return 0;
    }

    // A packed member can only be had whole: each byte depends on the ones
    // before it.
    if ((from != 0) || (len < size))
    {
        errno = EINVAL;
        return -1;
    }

    if ((method != NEA_NPAC_LZ77) && (method != NEA_NPAC_RLE))
    {
        errno = ENOTSUP;
        return -1;
    }

    if (size == 0)
        return 0;

    while (ne_npac_window_busy)
        cothread_yield();
    ne_npac_window_busy = true;

    // A stream that does not decode was written whole and checked (the
    // tools read it back), so what came from the card was not what is on it:
    // it is read and decoded again from the start. A read the card refused
    // was tried again already, in ne_npac_read_at().
    bool ok = false;
    bool card_failed = false;
    for (int tries = 0; !ok && !card_failed && (tries < NPAC_READ_TRIES); tries++)
    {
        // The stream starts with the BIOS's four byte header: the method,
        // then the size, which the index already has.
        ne_npac_source_t src = {
            .mnt = m,
            .next = offset,
            .left = stored,
        };
        for (int i = 0; i < 4; i++)
            ne_npac_source_byte(&src);

        ok = !src.failed &&
             ((method == NEA_NPAC_LZ77) ? ne_npac_unlz77(&src, dst, size)
                                        : ne_npac_unrle(&src, dst, size));
        card_failed = src.card_failed;

        if (!ok && !card_failed)
            NEA_DebugPrint("NPAC member at %u did not decode", (unsigned)offset);
    }

    ne_npac_window_busy = false;

    if (!ok)
    {
        // EIO: the card refused a read. EILSEQ: it gave bytes, but not the
        // ones written.
        errno = card_failed ? EIO : EILSEQ;
        return -1;
    }

    return 0;
}

int NEA_NpacMemberRead(const NEA_NpacMember *member, uint32_t from, void *dst,
                       uint32_t len)
{
    NEA_AssertPointer(member, "NULL member");

    if ((member == NULL) || (member->mount >= NEA_NPAC_MAX_MOUNTS) ||
        ((dst == NULL) && (len != 0)))
    {
        errno = EINVAL;
        return -1;
    }

    ne_npac_mount_t *m = &ne_npac_mounts[member->mount];
    if (!ne_npac_handle_valid(m, member->generation))
    {
        errno = EBADF;
        return -1;
    }

    return ne_npac_read_member(m, member->offset, member->size,
                               member->stored_size, member->method, from, dst,
                               len);
}

int NEA_NpacReadInto(const char *path, void *dst, uint32_t capacity)
{
    NEA_NpacMember member;

    if (NEA_NpacMemberFind(path, &member) != 0)
        return -1;

    if (member.size > capacity)
    {
        errno = ENOSPC;
        return -1;
    }

    if (NEA_NpacMemberRead(&member, 0, dst, member.size) != 0)
        return -1;

    return (int)member.size;
}

void NEA_NpacSystemEnd(void)
{
    for (int i = 0; i < NEA_NPAC_MAX_MOUNTS; i++)
    {
        if (ne_npac_mounts[i].used)
            ne_npac_release(&ne_npac_mounts[i]);
    }

    ne_npac_current = NULL;

    if (ne_npac_device_index >= 0)
    {
        deviceIoRemove(ne_npac_device_index);
        ne_npac_device_index = -1;
    }
}
