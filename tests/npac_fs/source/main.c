// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Warioware64, 2026
//
// This file is part of Nitro Engine Advanced

// Cross-check of the NPAC filesystem against its Python twin.
//
// tools/npac/npac_format.py writes the container that source/NEANPAC.c reads.
// Two implementations of the same FNT walk drift, and when they drift the
// symptom is a game that loads the wrong file -- or one member's bytes under
// another's name, which reads as data corruption rather than as a filesystem
// bug and gets debugged in the wrong place for a day.
//
// gen_vectors.py writes an archive and a table of what every member should
// contain. This mounts that archive with the real runtime and checks it: the
// compression method and size the index reports, the bytes that come back
// through fopen(), reads at an odd chunk size, seeks, stat(), the directory
// listings, the errors the missing cases should raise, chdir() with relative
// paths, and unmount followed by remount.
//
// One expected false alarm: the archive contains a Huffman member on purpose,
// and melonDS decodes Huffman wrongly under its default HLE BIOS. If the only
// failures name huff/skew.bin, point the emulator at a real BIOS dump
// (Config > Emu > external BIOS) before looking for a bug here.

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include <nds.h>
#include <filesystem.h>

#include <NEAMain.h>

#include "vectors.h"

static int checks = 0;
static int failures = 0;

static void check(bool ok, const char *what, const char *detail)
{
    checks++;

    if (ok)
        return;

    failures++;

    // The console is 32 columns and a failing name can be 127 of them, so
    // only the tail of a long detail is worth showing.
    if (failures <= 10)
    {
        size_t len = strlen(detail);
        const char *tail = (len > 20) ? detail + len - 20 : detail;
        printf("F %s: %s\n", what, tail);
    }
}

static uint32_t crc32_update(uint32_t crc, const uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (int bit = 0; bit < 8; bit++)
            crc = (crc >> 1) ^ (0xEDB88320U & (-(int32_t)(crc & 1)));
    }

    return crc;
}

static void full_path(char *out, size_t size, const char *path)
{
    snprintf(out, size, "test:/%s", path);
}

//-----------------------------------------------------------------------------

// Reads a member whole and checks its contents, its reported size, and that
// seeking to the end agrees with both.
static void CheckContents(const TestVector *v)
{
    char path[160];
    full_path(path, sizeof(path), v->path);

    FILE *f = fopen(path, "rb");
    if (f == NULL)
    {
        check(false, "fopen", v->path);
        return;
    }

    check(fseek(f, 0, SEEK_END) == 0, "seek end", v->path);
    check((uint32_t)ftell(f) == v->size, "size at end", v->path);
    rewind(f);

    uint8_t *buffer = malloc(v->size ? v->size : 1);
    if (buffer == NULL)
    {
        check(false, "malloc", v->path);
        fclose(f);
        return;
    }

    size_t got = fread(buffer, 1, v->size, f);
    check(got == v->size, "read length", v->path);

    uint32_t crc = crc32_update(0xFFFFFFFFU, buffer, got) ^ 0xFFFFFFFFU;
    check(crc == v->crc, "contents", v->path);

    // A read that runs past the end must come back empty rather than
    // wrapping or repeating the last block.
    uint8_t extra;
    check(fread(&extra, 1, 1, f) == 0, "read past end", v->path);

    free(buffer);
    fclose(f);
}

// Reads the same member again in small, deliberately unaligned pieces. The
// stored path serves these straight off the card and the compressed path out
// of RAM, and both have to end up in the same place.
static void CheckChunkedRead(const TestVector *v)
{
    char path[160];
    full_path(path, sizeof(path), v->path);

    FILE *f = fopen(path, "rb");
    if (f == NULL)
        return;

    uint32_t crc = 0xFFFFFFFFU;
    uint8_t chunk[7];
    size_t total = 0;
    size_t got;

    while ((got = fread(chunk, 1, sizeof(chunk), f)) > 0)
    {
        crc = crc32_update(crc, chunk, got);
        total += got;
    }

    check(total == v->size, "chunked length", v->path);
    check((crc ^ 0xFFFFFFFFU) == v->crc, "chunked crc", v->path);

    // Seek back into the middle and confirm the byte there is the one the
    // sequential read produced.
    if (v->size >= 2)
    {
        uint32_t middle = v->size / 2;

        check(fseek(f, middle, SEEK_SET) == 0, "seek set", v->path);
        check(ftell(f) == (long)middle, "tell", v->path);

        uint8_t byte;
        check(fread(&byte, 1, 1, f) == 1, "read middle", v->path);

        rewind(f);
        uint8_t *whole = malloc(v->size);
        if (whole != NULL)
        {
            fread(whole, 1, v->size, f);
            check(whole[middle] == byte, "seek agrees", v->path);
            free(whole);
        }
    }

    fclose(f);
}

static void CheckIndex(const TestVector *v)
{
    char path[160];
    full_path(path, sizeof(path), v->path);

    NEA_NpacFileInfo info;
    if (NEA_NpacGetFileInfo(path, &info) != 0)
    {
        check(false, "file info", v->path);
        return;
    }

    check(info.size == v->size, "info size", v->path);
    check(info.method == v->method, "info method", v->path);

    struct stat st;
    if (stat(path, &st) != 0)
    {
        check(false, "stat", v->path);
        return;
    }

    check((uint32_t)st.st_size == v->size, "stat size", v->path);
    check(S_ISREG(st.st_mode), "stat mode", v->path);
}

static void CheckListing(const TestListing *listing)
{
    char path[160];
    snprintf(path, sizeof(path), "test:%s", listing->path);

    DIR *dir = opendir(path);
    if (dir == NULL)
    {
        check(false, "opendir", listing->path);
        return;
    }

    struct dirent *entry;
    int index = 0;

    while ((entry = readdir(dir)) != NULL)
    {
        // "." and ".." come first and are not part of the table.
        if ((strcmp(entry->d_name, ".") == 0) ||
            (strcmp(entry->d_name, "..") == 0))
            continue;

        if (index >= listing->count)
        {
            check(false, "extra entry", entry->d_name);
            break;
        }

        const TestDirEntry *want = &listing->entries[index];

        check(strcmp(entry->d_name, want->name) == 0, "dir name", want->name);
        check((entry->d_type == DT_DIR) == want->is_dir, "dir type",
              want->name);
        index++;
    }

    check(index == listing->count, "entry count", listing->path);

    closedir(dir);
}

// The cases that have to fail, and fail for the stated reason. A filesystem
// that reports success on a missing path is worse than one that cannot open
// anything, because the caller then acts on whatever the buffer held.
static void CheckErrors(void)
{
    check(fopen("test:/does_not_exist.bin", "rb") == NULL,
          "missing file", "does_not_exist");
    check(fopen("test:/lz/nope/deeper.bin", "rb") == NULL,
          "missing dir", "nope");

    // A file used as a directory component is not a directory.
    check(fopen("test:/stored.bin/more", "rb") == NULL,
          "file as dir", "stored.bin/more");

    // Opening a directory as a file, and a file as a directory.
    check(fopen("test:/lz", "rb") == NULL, "open dir", "lz");
    check(opendir("test:/stored.bin") == NULL, "opendir file", "stored.bin");

    // The archive is read only.
    check(fopen("test:/new.bin", "wb") == NULL, "write", "new.bin");

    // A drive nobody mounted.
    check(fopen("nosuch:/a.bin", "rb") == NULL, "bad drive", "nosuch");
}

// A handle outliving its mount. The mount slot is deliberately reused by
// another archive in between, because that is the case that could quietly
// return the wrong file's bytes instead of an error, and the directory handle
// additionally reads through an index that unmounting has freed.
static void CheckStaleHandles(void)
{
    // POSIX calls rather than stdio: buffering would answer a small read from
    // the buffer and never reach the device.
    int fd = open("test:/lz/text.txt", O_RDONLY);
    DIR *dir = opendir("test:/lz");

    check(fd >= 0, "stale setup fd", "text.txt");
    check(dir != NULL, "stale setup dir", "lz");

    check(NEA_NpacUnmount("test") == 0, "unmount with handles", "test");
    check(NEA_NpacMount("test", "nitro:/test.npac") == 0, "slot reuse", "test");

    if (fd >= 0)
    {
        char byte;
        check(read(fd, &byte, 1) == -1, "stale read", "text.txt");
        check(errno == EBADF, "stale errno", "text.txt");
        check(lseek(fd, 0, SEEK_SET) == -1, "stale seek", "text.txt");
        close(fd);
    }

    if (dir != NULL)
    {
        check(readdir(dir) == NULL, "stale readdir", "lz");
        closedir(dir);
    }
}

static void CheckWorkingDirectory(void)
{
    if (chdir("test:/a/b/c") != 0)
    {
        check(false, "chdir", "a/b/c");
        return;
    }

    char cwd[160];
    if (getcwd(cwd, sizeof(cwd)) == NULL)
    {
        check(false, "getcwd", "a/b/c");
    }
    else
    {
        check(strcmp(cwd, "test:/a/b/c") == 0, "cwd value", cwd);
    }

    // Relative, and relative through a parent.
    FILE *f = fopen("deep.bin", "rb");
    check(f != NULL, "relative open", "deep.bin");
    if (f != NULL)
        fclose(f);

    f = fopen("../../../stored.bin", "rb");
    check(f != NULL, "dotdot open", "stored.bin");
    if (f != NULL)
        fclose(f);

    check(chdir("test:/") == 0, "chdir root", "/");
}

//-----------------------------------------------------------------------------

int main(int argc, char *argv[])
{
    consoleDemoInit();

    printf("NPAC filesystem test\n\n");

    if (!nitroFSInit(NULL))
    {
        printf("nitroFSInit failed\n");
        goto done;
    }

    if (NEA_NpacMount("test", "nitro:/test.npac") != 0)
    {
        printf("NEA_NpacMount failed\n");
        goto done;
    }

    check(NEA_NpacIsMounted("test"), "is mounted", "test");
    check(!NEA_NpacIsMounted("other"), "not mounted", "other");

    for (unsigned int i = 0; i < NUM_VECTORS; i++)
    {
        CheckIndex(&vectors[i]);
        CheckContents(&vectors[i]);
        CheckChunkedRead(&vectors[i]);
    }

    for (unsigned int i = 0; i < NUM_LISTINGS; i++)
        CheckListing(&listings[i]);

    CheckErrors();
    CheckWorkingDirectory();
    CheckStaleHandles();

    // Unmounting has to actually take the drive away...
    chdir("nitro:/");
    check(NEA_NpacUnmount("test") == 0, "unmount", "test");
    check(!NEA_NpacIsMounted("test"), "unmounted", "test");
    check(fopen("test:/stored.bin", "rb") == NULL, "gone", "stored.bin");
    check(NEA_NpacUnmount("test") != 0, "double unmount", "test");

    // ...and mounting again has to give it back, with the index rebuilt.
    check(NEA_NpacMount("test", "nitro:/test.npac") == 0, "remount", "test");
    CheckContents(&vectors[0]);

    // Reserved names belong to libnds and would be unreachable.
    check(NEA_NpacMount("nitro", "nitro:/test.npac") != 0, "reserved", "nitro");
    check(NEA_NpacMount("test", "nitro:/test.npac") != 0, "duplicate", "test");
    check(NEA_NpacMount("gone", "nitro:/missing.npac") != 0, "no file", "gone");

    NEA_NpacSystemEnd();
    check(!NEA_NpacIsMounted("test"), "system end", "test");

    printf("\n%d checks, %d failed\n", checks, failures);
    printf("%s\n", failures == 0 ? "PASS" : "FAIL");

done:
    printf("\nPress START to exit\n");

    while (1)
    {
        swiWaitForVBlank();
        scanKeys();
        if (keysDown() & KEY_START)
            break;
    }

    return 0;
}
