// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Warioware64, 2026
//
// This file is part of Nitro Engine Advanced

// NPAC archives in many pieces on a FAT card: are their bytes read right?
//
// libnds's fatInitLookupCache() gives a file FatFs's fast seek, a table of
// where each piece of it is. For a file in more pieces than the table holds,
// it reports the size it would need and leaves the half-made table on the
// file (libnds 1.25.0): seeks past the last piece it holds then go astray.
// NEA used to give each archive a 2 KB table (255 pieces) and not look at
// the answer. NEANPAC.c now looks, and keeps no file it failed on.
//
// This makes archives in a known number of pieces (1, 200, 400 -- past the
// old table, within DSi mode's 4 KB one -- and 600), by writing each a cluster
// at a time with a cluster of another file between them, and reads 4 KB at
// far places in them, each word of which says where it is:
//   raw     open() and fatInitLookupCache(fd, 2048), as NEA did: wrong past
//           255 pieces, which is the bug
//   nea     NEA_NpacMount() and NEA_NpacMemberRead(): right, always
//   reopen  a read made to fail, so that NEA opens the archive again: right
// The pieces a file ends up in are counted (on a card in use the clusters
// taken are wherever FatFs finds room), and what is expected is judged from
// that count, not from what was asked.
//
// It writes tens of megabytes. It does nothing unless the card has a file
// _nds/hl_nds/FRAGTEST_OK (on a card you can spare); with FRAGTEST_KEEP too,
// the archives stay in _nds/hl_nds/fragtest/ for the card-read probe to time.
// Results: _nds/hl_nds/npac_frag.txt, appended.

#include <errno.h>
#include <fcntl.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/statvfs.h>
#include <unistd.h>

#include <fat.h>
#include <nds.h>

#include <NEAMain.h>

#include "npac_write.h"

#define READS           48
#define PAGE            4096
#define CLUSTER_MOST    65536
#define MEMBER_MOST     (8u << 20)
#define OLD_TABLE       2048

// The table a mount may take: NEA's limits, or (the build before them) the
// 2 KB it always gave.
#ifdef NEA_NPAC_HAS_SEEK_INFO
#define TABLE_DS        NEA_NPAC_SEEK_TABLE_DS
#define TABLE_DSI       NEA_NPAC_SEEK_TABLE_DSI
#else
#define TABLE_DS        OLD_TABLE
#define TABLE_DSI       OLD_TABLE
#endif

static char root[64];
static FILE *out = NULL;
static int failures = 0;
static uint8_t chunk[CLUSTER_MOST] __attribute__((aligned(32)));
static uint8_t page[PAGE] __attribute__((aligned(32)));
static uint8_t zero[CLUSTER_MOST] __attribute__((aligned(32)));

static void say(bool screen, const char *format, ...)
{
    char line[160];
    va_list args;
    va_start(args, format);
    vsnprintf(line, sizeof(line), format, args);
    va_end(args);
    if (out != NULL)
        fprintf(out, "%s\n", line);
    if (screen)
        printf("%s\n", line);
}

static void expect(bool ok, const char *what)
{
    if (!ok)
        failures++;
    say(true, "  %s %s", ok ? "ok  " : "FAIL", what);
}

static uint64_t seed = 0x9E3779B97F4A7C15ull;
static uint32_t next(void)
{
    seed ^= seed << 13;
    seed ^= seed >> 7;
    seed ^= seed << 17;
    return (uint32_t)(seed >> 16);
}

static bool exists(const char *name)
{
    char path[128];
    struct stat st;
    snprintf(path, sizeof(path), "%s%s", root, name);
    return stat(path, &st) == 0;
}

// One archive: what was asked, what it came to.
typedef struct {
    const char *name;
    int asked;          // pieces
    char path[128];
    char filler[128];
    uint32_t count;     // members
    uint32_t bytes;     // each
    uint32_t head;
    uint32_t total;
    int pieces;         // counted on the card
} Archive;

// How many pieces the file is in: what a table of four words asks for, on a
// file closed at once (a table too small is left half made on it).
static int pieces_of(const char *path)
{
    int fd = open(path, O_RDONLY);
    if (fd < 0)
        return -1;
    int need = fatInitLookupCache(fd, 16);
    close(fd);
    if (need < 0)
        return need;
    return need == 0 ? 1 : (need - 2) / 2;
}

// The file's bytes from `from`, `len` of them (both multiples of 4).
static void bytes_at(const Archive *a, const uint8_t *head, uint32_t from, uint8_t *to, uint32_t len)
{
    for (uint32_t o = from; o < from + len; o += 4)
    {
        uint32_t w = o < a->head ? (uint32_t)head[o] | (uint32_t)head[o + 1] << 8 |
                                       (uint32_t)head[o + 2] << 16 | (uint32_t)head[o + 3] << 24
                                 : npac_frag_word(o);
        memcpy(to + (o - from), &w, 4);
    }
}

// Writes the archive a cluster at a time, a cluster of the filler after
// each of its first pieces, then the rest in one go.
static bool make(Archive *a, uint32_t cluster)
{
    uint8_t head[NPAC_FRAG_HEAD_MAX];
    uint32_t want = (uint32_t)(a->asked + 8) * cluster;
    a->count = (want + MEMBER_MOST - 1) / MEMBER_MOST;
    a->bytes = ((want / a->count) + 3) & ~3u;
    a->head = npac_frag_head(head, a->count, a->bytes, &a->total);
    if (a->head == 0)
        return false;

    int fd = open(a->path, O_WRONLY | O_CREAT | O_TRUNC, 0666);
    int fill = a->asked > 1 ? open(a->filler, O_WRONLY | O_CREAT | O_TRUNC, 0666) : -1;
    bool ok = fd >= 0 && (a->asked <= 1 || fill >= 0);
    uint32_t done = 0;
    for (int k = 0; ok && done < a->total; k++)
    {
        uint32_t len = a->total - done < cluster ? a->total - done : cluster;
        bytes_at(a, head, done, chunk, len);
        ok = write(fd, chunk, len) == (ssize_t)len;
        done += len;
        if (ok && k < a->asked - 1)
            ok = write(fill, zero, cluster) == (ssize_t)cluster;
    }
    if (fd >= 0)
        close(fd);
    if (fill >= 0)
        close(fill);
    unlink(a->filler);
    a->pieces = ok ? pieces_of(a->path) : -1;
    return ok;
}

// A far place to read: a member, and a word-aligned offset in its last half.
static void far_place(const Archive *a, uint32_t *member, uint32_t *from)
{
    *member = next() % a->count;
    uint32_t span = a->bytes / 2 - PAGE;
    *from = a->bytes / 2 + ((next() % (span / 4)) * 4);
}

static bool page_right(const uint8_t *data, uint32_t file_offset)
{
    for (uint32_t i = 0; i < PAGE; i += 4)
    {
        uint32_t w;
        memcpy(&w, data + i, 4);
        if (w != npac_frag_word(file_offset + i))
            return false;
    }
    return true;
}

// open() and a 2 KB table, as NEA did: how many reads are wrong.
static int raw_wrong(const Archive *a, int *answer)
{
    int fd = open(a->path, O_RDONLY);
    if (fd < 0)
        return READS;
    *answer = fatInitLookupCache(fd, OLD_TABLE);
    int wrong = 0;
    for (int i = 0; i < READS; i++)
    {
        uint32_t m, from;
        far_place(a, &m, &from);
        uint32_t at = a->head + m * a->bytes + from;
        bool ok = lseek(fd, (off_t)at, SEEK_SET) == (off_t)at && read(fd, page, PAGE) == PAGE;
        wrong += (ok && page_right(page, at)) ? 0 : 1;
    }
    close(fd);
    return wrong;
}

// Through NEA: how many reads are wrong, and what they took.
static int nea_wrong(const Archive *a, uint32_t *mean_us, uint32_t *most_us)
{
    int wrong = 0;
    uint64_t sum = 0;
    *most_us = 0;
    for (int i = 0; i < READS; i++)
    {
        uint32_t m, from;
        far_place(a, &m, &from);
        char path[24];
        snprintf(path, sizeof(path), "frag:/d%u.bin", (unsigned)m);
        NEA_NpacMember member;
        uint32_t before = cpuGetTiming();
        bool ok = NEA_NpacMemberFind(path, &member) == 0 && NEA_NpacMemberRead(&member, from, page, PAGE) == 0;
        uint32_t us = timerTicks2usec(cpuGetTiming() - before);
        sum += us;
        if (us > *most_us)
            *most_us = us;
        wrong += (ok && page_right(page, a->head + m * a->bytes + from)) ? 0 : 1;
    }
    *mean_us = (uint32_t)(sum / READS);
    return wrong;
}

static void check(Archive *a, uint32_t limit)
{
    say(true, "%s: asked %d pieces, got %d; %u x %u bytes", a->name, a->asked, a->pieces,
        (unsigned)a->count, (unsigned)a->bytes);
    // FatFs's table: two words a piece, and two more.
    bool old_fits = a->pieces >= 1 && (a->pieces * 2 + 2) * 4 <= OLD_TABLE;
    bool new_fits = a->pieces >= 1 && (a->pieces * 2 + 2) * 4 <= (int)limit;
    (void)new_fits; // (the build before the fix has no use for it)

    int answer = 0;
    int raw = raw_wrong(a, &answer);
    say(true, "  raw, 2 KB table (answer %d): %d of %d reads wrong", answer, raw, READS);
    if (old_fits)
        expect(raw == 0, "raw reads right (the table held the file)");
    else
        say(true, "  (raw: wrong reads expected past 255 pieces -- the bug)");

    if (NEA_NpacMount("frag", a->path) != 0)
    {
        expect(false, "NEA mounts the archive");
        return;
    }
    uint32_t mean = 0, most = 0;
    int nea = nea_wrong(a, &mean, &most);
    say(true, "  NEA: %d of %d reads wrong; 4 KB far: mean %u us, most %u us", nea, READS, (unsigned)mean,
        (unsigned)most);
    expect(nea == 0, "NEA reads every far page right");
#ifdef NEA_NPAC_HAS_SEEK_INFO
    NEA_NpacSeek seek;
    if (NEA_NpacSeekInfo("frag", &seek) == 0)
    {
        say(true, "  NEA: fast seek %s, state %d, %d pieces, table %u B", seek.fast ? "on" : "off", (int)seek.state,
            (int)seek.fragments, (unsigned)seek.table_bytes);
        expect(seek.fragments == a->pieces, "NEA counts the pieces the card has");
        expect(seek.fast == new_fits, new_fits ? "fast seek on (the table fits)" : "fast seek off (too many pieces)");
    }
    // One read made to fail: opened again, with no table, and still right.
    NEA_NpacFailReads(1);
    uint32_t m, from;
    far_place(a, &m, &from);
    char path[24];
    snprintf(path, sizeof(path), "frag:/d%u.bin", (unsigned)m);
    NEA_NpacMember member;
    bool ok = NEA_NpacMemberFind(path, &member) == 0 && NEA_NpacMemberRead(&member, from, page, PAGE) == 0 &&
              page_right(page, a->head + m * a->bytes + from);
    expect(ok, "a failed read is read again right");
    if (NEA_NpacSeekInfo("frag", &seek) == 0)
        expect(!seek.fast && seek.plain_reopens == 1, "opened again with no table");
    int after = nea_wrong(a, &mean, &most);
    say(true, "  NEA after: %d wrong; 4 KB far, no fast seek: mean %u us, most %u us", after, (unsigned)mean,
        (unsigned)most);
    expect(after == 0, "every far page right after");
#else
    say(true, "  (this NEA has no NEA_NpacSeekInfo: the build before the fix)");
#endif
    NEA_NpacUnmount("frag");
}

int main(int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    consoleDemoInit();
    if (isDSiMode())
        setCpuClock(true);
    cpuStartTiming(0);
    printf("NPAC fragmented archive test\n\n");

    Archive archives[] = {
        {"one piece", 1, "", "", 0, 0, 0, 0, 0},
        {"200 pieces", 200, "", "", 0, 0, 0, 0, 0},
        {"400 pieces", 400, "", "", 0, 0, 0, 0, 0},
        {"600 pieces", 600, "", "", 0, 0, 0, 0, 0},
    };
    const int count = sizeof(archives) / sizeof(archives[0]);
    bool made_dir = false;
    bool keep = false;
    char path[128];
    char dir[128] = "";

    if (!fatInitDefault())
    {
        printf("No card found.\n");
        goto halt;
    }
    snprintf(root, sizeof(root), "%s_nds/hl_nds/", fatGetDefaultDrive());
    if (!exists("FRAGTEST_OK"))
    {
        printf("Refused: this writes tens of\nmegabytes. Put an empty file\n_nds/hl_nds/FRAGTEST_OK on a\n"
               "card you can spare.\n");
        goto halt;
    }
    keep = exists("FRAGTEST_KEEP");
    snprintf(path, sizeof(path), "%snpac_frag.txt", root);
    out = fopen(path, "a");
    uint32_t limit = isDSiMode() ? TABLE_DSI : TABLE_DS;
    say(true, "== %s, %s, table limit %u B", isDSiMode() ? "DSi mode" : "DS mode", fatGetDefaultDrive(),
        (unsigned)limit);

    struct statvfs vfs;
    snprintf(dir, sizeof(dir), "%sfragtest", root);
    if (statvfs(root, &vfs) != 0 || vfs.f_bsize > CLUSTER_MOST || (vfs.f_bsize & 3) != 0)
    {
        say(true, "Cluster size not usable");
        goto done;
    }
    uint32_t cluster = (uint32_t)vfs.f_bsize;
    uint64_t need = 0;
    for (int i = 0; i < count; i++)
        need += (uint64_t)(archives[i].asked + 8) * cluster * 2;
    uint64_t room = (uint64_t)vfs.f_bavail * vfs.f_bsize;
    say(true, "cluster %u B; writes up to %u MB, %u MB free", (unsigned)cluster, (unsigned)(need >> 20),
        (unsigned)(room >> 20));
    if (room < need + need / 4)
    {
        say(true, "Not enough room on the card");
        goto done;
    }
    mkdir(dir, 0777);
    made_dir = true;
    for (int i = 0; i < count; i++)
    {
        snprintf(archives[i].path, sizeof(archives[i].path), "%sfragtest/fragtest_%d.npac", root, i);
        snprintf(archives[i].filler, sizeof(archives[i].filler), "%sfragtest/filler_%d.bin", root, i);
    }
    for (int i = 0; i < count; i++)
    {
        printf("writing %s...\n", archives[i].name);
        if (!make(&archives[i], cluster))
        {
            expect(false, "the archive is written");
            goto done;
        }
        check(&archives[i], limit);
        if (out != NULL)
            fflush(out);
    }

done:
    // Whatever happened: the fillers, and unless kept, the archives.
    if (made_dir)
    {
        for (int i = 0; i < count; i++)
        {
            if (archives[i].filler[0] != 0)
                unlink(archives[i].filler);
            if (!keep && archives[i].path[0] != 0)
                unlink(archives[i].path);
        }
        if (!keep)
            rmdir(dir);
    }
    say(true, "== %s: %d failed", failures == 0 ? "PASS" : "FAIL", failures);
    if (out != NULL)
        fclose(out);
halt:
    printf("\nSTART: power off\n");
    for (;;)
    {
        swiWaitForVBlank();
        scanKeys();
        if (keysDown() & KEY_START)
            break;
    }
    return 0;
}
