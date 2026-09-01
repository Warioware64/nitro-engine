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
    uint32_t img_offset;    // Offset of the file image within the archive file
    uint32_t img_size;
    uint8_t *fat;           // num_files * 8 bytes
    uint8_t *fnt;           // Name table, offsets inside it are relative to it
    uint8_t *cmp;           // num_files * 4 bytes, NULL for a NARC
    uint32_t fnt_size;
    uint16_t num_files;
    uint16_t num_dirs;
    uint16_t current_dir;   // Working directory of this mount

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
    uint8_t *ram;           // Decompressed copy, or NULL when stored
    uint16_t file_index;
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

static int ne_npac_read_at(int fd, uint32_t offset, void *dst, size_t len)
{
    if (len == 0)
        return 0;

    if (lseek(fd, (off_t)offset, SEEK_SET) != (off_t)offset)
        return -1;

    uint8_t *out = dst;
    size_t done = 0;
    while (done < len)
    {
        ssize_t got = read(fd, out + done, len - done);
        if (got <= 0)
            return -1;
        done += (size_t)got;
    }

    return 0;
}

static uint8_t *ne_npac_read_chunk(int fd, uint32_t offset, uint32_t len)
{
    uint8_t *buf = malloc(len ? len : 1);
    if (buf == NULL)
    {
        NEA_DebugPrint("Not enough memory");
        return NULL;
    }

    if (ne_npac_read_at(fd, offset, buf, len) != 0)
    {
        free(buf);
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

    if (ne_npac_read_at(m->fd, 0, header, sizeof(header)) != 0)
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

        if (ne_npac_read_at(m->fd, pos, chunk, sizeof(chunk)) != 0)
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
            if (ne_npac_read_at(m->fd, payload, count, sizeof(count)) != 0)
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

            m->fat = ne_npac_read_chunk(m->fd, payload + 4,
                                        (uint32_t)m->num_files * 8);
            if (m->fat == NULL)
                return -1;
        }
        else if (memcmp(chunk, "BTNF", 4) == 0)
        {
            m->fnt_size = payload_size;
            m->fnt = ne_npac_read_chunk(m->fd, payload, payload_size);
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
        off_t end = lseek(m->fd, 0, SEEK_END);

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

        m->cmp = ne_npac_read_chunk(m->fd, cmp_offset + 4,
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

static void ne_npac_release(ne_npac_mount_t *m)
{
    if (m->fd >= 0)
        close(m->fd);

    free(m->fat);
    free(m->fnt);
    free(m->cmp);

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

static const uint8_t *ne_npac_dir_entry(const ne_npac_mount_t *m, uint16_t dir_id)
{
    uint32_t index = dir_id & 0x0FFF;

    if (index >= m->num_dirs)
        return NULL;

    return m->fnt + index * 8;
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

    uint8_t type = m->fnt[it->pos];

    // 00h ends the subtable. 80h is reserved and has no length, so treat it as
    // the end too rather than walking off into the next directory.
    if ((type == 0x00) || (type == 0x80))
        return 0;

    uint32_t len = type & 0x7F;
    bool is_dir = (type & 0x80) != 0;
    uint32_t total = 1 + len + (is_dir ? 2 : 0);

    if (it->pos + total > m->fnt_size)
        return 0;

    *name = (const char *)(m->fnt + it->pos + 1);
    *name_len = len;

    if (is_dir)
    {
        *id = ne_rd16(m->fnt + it->pos + 1 + len);
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

    ne_npac_file_t *f = calloc(1, sizeof(ne_npac_file_t));
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

    if (method != NEA_NPAC_STORED)
    {
        DecompressType type;

        switch (method)
        {
            case NEA_NPAC_LZ77:
                type = LZ77;
                break;
            case NEA_NPAC_HUFF4:
            case NEA_NPAC_HUFF8:
                type = HUFF;
                break;
            case NEA_NPAC_RLE:
                type = RLE;
                break;
            default:
                NEA_DebugPrint("Unknown NPAC compression 0x%02X", method);
                free(f);
                errno = EINVAL;
                return -1;
        }

        // The BIOS decompressors want the whole stream in one piece, so the
        // member is inflated here and served out of RAM from then on.
        uint32_t stored = end - start;

        uint8_t *src = malloc(stored ? stored : 1);
        if (src == NULL)
        {
            NEA_DebugPrint("Not enough memory");
            free(f);
            errno = ENOMEM;
            return -1;
        }

        if (ne_npac_read_at(m->fd, f->offset, src, stored) != 0)
        {
            free(src);
            free(f);
            errno = EIO;
            return -1;
        }

        uint8_t *dst = malloc(size ? size : 1);
        if (dst == NULL)
        {
            NEA_DebugPrint("Not enough memory");
            free(src);
            free(f);
            errno = ENOMEM;
            return -1;
        }

        decompress(src, dst, type);
        free(src);

        f->ram = dst;
    }

    // libnds tags the top four bits and the bottom two with the device index,
    // and crashes on a descriptor that has already used them.
    int fd = (int)(intptr_t)f;
    if ((fd & 0xF0000003) != 0)
    {
        NEA_DebugPrint("Misaligned NPAC file descriptor");
        free(f->ram);
        free(f);
        errno = ENOMEM;
        return -1;
    }

    return fd;
}

static int ne_npac_close(int fd)
{
    ne_npac_file_t *f = (ne_npac_file_t *)fd;

    free(f->ram);
    free(f);

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

    if (f->ram != NULL)
    {
        memcpy(ptr, f->ram + f->position, len);
    }
    else if (ne_npac_read_at(f->mnt->fd, f->offset + f->position, ptr, len) != 0)
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

int NEA_NpacMount(const char *drive, const char *path)
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

    m->fd = open(path, O_RDONLY, 0);
    if (m->fd < 0)
    {
        NEA_DebugPrint("%s couldn't be opened", path);
        m->fd = -1;
        return -1;
    }

    // Only helps when the archive is on a FAT drive; the call checks that
    // itself and reports that it isn't supported otherwise.
    fatInitLookupCache(m->fd, 2048);

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
