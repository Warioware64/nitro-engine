// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Nitro Engine Advanced contributors, 2026
//
// This file is part of Nitro Engine Advanced

// Runtime tests for the asynchronous file API: the write path added by
// NEA_FATWriteDataAsync() and the loaders that were converted to it.
//
// The test writes the assets it needs with the asynchronous writer, then reads
// them back with the asynchronous loaders, so both halves are covered. It needs
// a writable filesystem: point the emulator at a DLDI SD folder or an SD image.
//
// Results are printed on the console. The test stops on the first failure and
// prints the line number.

#include <stdio.h>

#include <fat.h>
#include <unistd.h>
#include <nds/arm9/dldi.h>

#include <NEAMain.h>

#define SAVE_PATH    "nea_test_save.bin"
#define TEMP_PATH    SAVE_PATH ".temp"
#define COLMESH_PATH "nea_test.colmesh"
#define BONCOL_PATH  "nea_test.boncol"
#define TILES_PATH   "nea_test_tiles.bin"

static int failures = 0;
static int checks = 0;

// Only failures are printed, so that the whole run fits on one console screen
// and every section's result stays readable instead of scrolling away.
#define CHECK(cond, name)                                       \
    do {                                                        \
        checks++;                                               \
        if (!(cond))                                            \
        {                                                       \
            printf("  FAIL %s (L%d)\n", name, __LINE__);        \
            failures++;                                         \
        }                                                       \
    } while (0)

// Prints one line per section: how many checks it ran and how many failed.
#define SECTION(fn)                                             \
    do {                                                        \
        int before_checks = checks;                             \
        int before_fails = failures;                            \
        fn();                                                   \
        printf("%-16s %2d/%2d ok\n", #fn + 5,                   \
               (checks - before_checks) - (failures - before_fails), \
               checks - before_checks);                         \
    } while (0)

// A job that never finishes would hang the whole test with nothing on screen,
// so give up after a few seconds and let the caller report the failure.
#define PUMP_TIMEOUT_FRAMES 600

// Runs the main loop until a job reaches a terminal state, the way a game would
// (rather than NEA_AsyncWait(), so that the frame path is exercised too).
static NEA_AsyncState PumpUntilDone(NEA_AsyncFile *job)
{
    NEA_AsyncState state = NEA_AsyncGetState(job);
    for (int i = 0; i < PUMP_TIMEOUT_FRAMES; i++)
    {
        if (state != NEA_ASYNC_PENDING && state != NEA_ASYNC_READY)
            return state;

        NEA_WaitForVBL(NEA_UPDATE_ASSETS);
        state = NEA_AsyncGetState(job);
    }

    printf("  TIMEOUT in state %d\n", (int)state);
    return NEA_ASYNC_ERROR;
}

// Runs a few frames so that queued work can make progress.
static void PumpFrames(int count)
{
    for (int i = 0; i < count; i++)
        NEA_WaitForVBL(NEA_UPDATE_ASSETS);
}

static bool FileExists(const char *path)
{
    FILE *f = fopen(path, "rb");
    if (f == NULL)
        return false;
    fclose(f);
    return true;
}

// Writes a buffer with the asynchronous writer and waits for it to finish.
static bool WriteSync(const char *path, const void *data, size_t size)
{
    NEA_AsyncFile *job = NEA_FATWriteDataAsync(path, data, size,
                                               NEA_ASYNC_WRITE_COPY);
    if (job == NULL)
        return false;

    NEA_AsyncState state = PumpUntilDone(job);
    NEA_AsyncRelease(job);
    return state == NEA_ASYNC_DONE;
}

// ---------------------------------------------------------------------------
// Asynchronous writing
// ---------------------------------------------------------------------------

#define PAYLOAD_SIZE (32 * 1024)

static void test_write_modes(void)
{
    char *buf = malloc(PAYLOAD_SIZE);
    memset(buf, 'A', PAYLOAD_SIZE);

    // COPY: the engine duplicates the buffer, so overwriting it right away must
    // not change what lands on disk.
    NEA_AsyncFile *job = NEA_FATWriteDataAsync(SAVE_PATH, buf, PAYLOAD_SIZE,
                                               NEA_ASYNC_WRITE_COPY);
    CHECK(job != NULL, "COPY queued");
    memset(buf, 'X', PAYLOAD_SIZE);
    CHECK(PumpUntilDone(job) == NEA_ASYNC_DONE, "COPY done");
    NEA_AsyncRelease(job);

    char *back = NEA_FATLoadData(SAVE_PATH);
    CHECK(back != NULL, "COPY readable");
    CHECK(back != NULL && back[0] == 'A' && back[PAYLOAD_SIZE - 1] == 'A',
          "COPY data intact");
    CHECK(NEA_FATFileSize(SAVE_PATH) == PAYLOAD_SIZE, "COPY size");
    free(back);
    CHECK(!FileExists(TEMP_PATH), "COPY no temp left");

    // BORROW: written straight out of the caller's buffer.
    memset(buf, 'B', PAYLOAD_SIZE);
    job = NEA_FATWriteDataAsync(SAVE_PATH, buf, PAYLOAD_SIZE,
                                NEA_ASYNC_WRITE_BORROW);
    CHECK(job != NULL, "BORROW queued");
    CHECK(PumpUntilDone(job) == NEA_ASYNC_DONE, "BORROW done");
    NEA_AsyncRelease(job);

    back = NEA_FATLoadData(SAVE_PATH);
    CHECK(back != NULL && back[0] == 'B', "BORROW data");
    free(back);
    free(buf);

    // TAKE: the engine owns and frees this buffer.
    char *owned = malloc(PAYLOAD_SIZE);
    memset(owned, 'C', PAYLOAD_SIZE);
    job = NEA_FATWriteDataAsync(SAVE_PATH, owned, PAYLOAD_SIZE,
                                NEA_ASYNC_WRITE_TAKE);
    CHECK(job != NULL, "TAKE queued");
    CHECK(PumpUntilDone(job) == NEA_ASYNC_DONE, "TAKE done");
    NEA_AsyncRelease(job);

    back = NEA_FATLoadData(SAVE_PATH);
    CHECK(back != NULL && back[0] == 'C', "TAKE data");
    free(back);

    // An empty write is a valid file, not an error.
    job = NEA_FATWriteDataAsync(SAVE_PATH, "", 0, NEA_ASYNC_WRITE_COPY);
    CHECK(job != NULL && PumpUntilDone(job) == NEA_ASYNC_DONE, "empty write");
    NEA_AsyncRelease(job);
    CHECK(NEA_FATFileSize(SAVE_PATH) == 0, "empty size");

    // A path that can't be created must fail rather than hang.
    job = NEA_FATWriteDataAsync("nea_no_such_dir/x.bin", "hi", 2,
                                NEA_ASYNC_WRITE_COPY);
    if (job != NULL)
    {
        CHECK(PumpUntilDone(job) == NEA_ASYNC_ERROR, "bad path errors");
        NEA_AsyncRelease(job);
    }
}

// Returns true if the whole file is filled with one of the two payload bytes,
// which is what "the write is atomic" means: the file is either entirely the
// old contents or entirely the new one, never a mixture and never truncated.
static bool FileIsWholly(const char *path, char a, char b, size_t expect_size)
{
    if (NEA_FATFileSize(path) != expect_size)
        return false;

    char *data = NEA_FATLoadData(path);
    if (data == NULL)
        return false;

    char first = data[0];
    bool ok = (first == a) || (first == b);
    for (size_t i = 0; ok && i < expect_size; i++)
    {
        if (data[i] != first)
            ok = false;
    }

    free(data);
    return ok;
}

// The point of the ".temp then rename" design: a cancelled write must leave the
// previous file exactly as it was.
static void test_write_atomicity(void)
{
    char *buf = malloc(PAYLOAD_SIZE);

    memset(buf, 'G', PAYLOAD_SIZE);
    CHECK(WriteSync(SAVE_PATH, buf, PAYLOAD_SIZE), "baseline written");

    // Queue a write of a different payload over the same path and drop the
    // handle without running a single frame. Cothreads only run while the main
    // thread yields, so the worker cannot have written anything yet: this
    // pins the cancellation to a point where the old file must survive whole.
    memset(buf, 'H', PAYLOAD_SIZE);
    NEA_AsyncFile *job = NEA_FATWriteDataAsync(SAVE_PATH, buf, PAYLOAD_SIZE,
                                               NEA_ASYNC_WRITE_COPY);
    CHECK(job != NULL, "overwrite queued");
    NEA_AsyncRelease(job);

    // Let the worker notice the cancellation and clean up after itself.
    PumpFrames(30);

    char *back = NEA_FATLoadData(SAVE_PATH);
    CHECK(back != NULL, "file survived cancel");
    CHECK(back != NULL && back[0] == 'G' && back[PAYLOAD_SIZE - 1] == 'G',
          "old payload intact");
    CHECK(NEA_FATFileSize(SAVE_PATH) == PAYLOAD_SIZE, "old size intact");
    CHECK(!FileExists(TEMP_PATH), "temp cleaned up");
    free(back);

    // Same again, but cancelled after a frame. Whether the write got as far as
    // the rename depends on how fast the filesystem is, so the invariant to
    // check is the one that holds either way: the file is never left torn.
    memset(buf, 'I', PAYLOAD_SIZE);
    job = NEA_FATWriteDataAsync(SAVE_PATH, buf, PAYLOAD_SIZE,
                                NEA_ASYNC_WRITE_COPY);
    CHECK(job != NULL, "late-cancel queued");
    PumpFrames(1);
    NEA_AsyncRelease(job);
    PumpFrames(30);

    CHECK(FileIsWholly(SAVE_PATH, 'G', 'I', PAYLOAD_SIZE), "no torn write");
    CHECK(!FileExists(TEMP_PATH), "no temp after late cancel");

    free(buf);
}

// ---------------------------------------------------------------------------
// Collision loaders
// ---------------------------------------------------------------------------

// Builds a minimal .colmesh file with one triangle.
static bool WriteColMesh(void)
{
    uint32_t header[10] = {
        0x4D4C4F43,             // "COLM"
        1,                      // version
        1,                      // num_triangles
        0,                      // flags
        0, 0, 0,                // aabb_min
        inttof32(1), inttof32(1), inttof32(1) // aabb_max
    };
    int32_t tri[12] = {
        0, 0, 0,
        inttof32(1), 0, 0,
        0, inttof32(1), 0,
        0, 0, inttof32(1)
    };

    uint8_t file[sizeof(header) + sizeof(tri)];
    memcpy(file, header, sizeof(header));
    memcpy(file + sizeof(header), tri, sizeof(tri));

    return WriteSync(COLMESH_PATH, file, sizeof(file));
}

// Builds a minimal .boncol file with two bones.
static bool WriteBoneCollision(void)
{
    uint32_t header[4] = { 0x4C434E42, 1, 2, 0 }; // "BNCL", v1, 2 bones

    // Two 32-byte entries: a sphere on joint 0 and an AABB on joint 1.
    int32_t bones[16] = { 0 };
    bones[0] = 1 | (0 << 8);        // type = sphere, joint_idx = 0
    bones[1] = inttof32(2);         // radius
    bones[8] = 3 | (1 << 8);        // type = aabb, joint_idx = 1
    bones[9] = inttof32(1);         // half_x
    bones[10] = inttof32(2);        // half_y
    bones[11] = inttof32(3);        // half_z

    uint8_t file[sizeof(header) + sizeof(bones)];
    memcpy(file, header, sizeof(header));
    memcpy(file + sizeof(header), bones, sizeof(bones));

    return WriteSync(BONCOL_PATH, file, sizeof(file));
}

static void test_collision_async(void)
{
    CHECK(WriteColMesh(), "colmesh written");
    CHECK(WriteBoneCollision(), "boncol written");

    // ColMesh
    NEA_ColMesh *mesh = NULL;
    NEA_AsyncFile *job = NEA_ColMeshLoadFATAsync(&mesh, COLMESH_PATH);
    CHECK(job != NULL, "colmesh queued");
    CHECK(PumpUntilDone(job) == NEA_ASYNC_DONE, "colmesh done");
    NEA_AsyncRelease(job);
    CHECK(mesh != NULL, "colmesh created");
    CHECK(mesh != NULL && mesh->num_triangles == 1, "colmesh triangle count");

    // The synchronous loader must agree with the asynchronous one.
    NEA_ColMesh *sync_mesh = NEA_ColMeshLoadFAT(COLMESH_PATH);
    CHECK(sync_mesh != NULL, "colmesh sync loads");
    CHECK(sync_mesh != NULL && mesh != NULL
          && sync_mesh->num_triangles == mesh->num_triangles,
          "colmesh sync matches async");
    NEA_ColMeshFree(sync_mesh);
    NEA_ColMeshFree(mesh);

    // BoneCollision
    NEA_BoneCollisionData *bcd = NULL;
    job = NEA_BoneCollisionLoadFATAsync(&bcd, BONCOL_PATH);
    CHECK(job != NULL, "boncol queued");
    CHECK(PumpUntilDone(job) == NEA_ASYNC_DONE, "boncol done");
    NEA_AsyncRelease(job);
    CHECK(bcd != NULL, "boncol created");
    CHECK(bcd != NULL && bcd->num_bones == 2, "boncol bone count");
    CHECK(bcd != NULL && bcd->bones[0].shape.type == NEA_COL_SPHERE,
          "boncol sphere bone");
    CHECK(bcd != NULL && bcd->bones[1].shape.type == NEA_COL_AABB,
          "boncol aabb bone");
    NEA_BoneCollisionFree(bcd);

    // A file that isn't a colmesh must fail cleanly, not corrupt anything.
    NEA_ColMesh *bad = NULL;
    job = NEA_ColMeshLoadFATAsync(&bad, BONCOL_PATH);
    CHECK(job != NULL, "bad colmesh queued");
    CHECK(PumpUntilDone(job) == NEA_ASYNC_ERROR, "bad colmesh errors");
    CHECK(bad == NULL, "bad colmesh left out NULL");
    NEA_AsyncRelease(job);

    // A file that doesn't exist at all.
    job = NEA_ColMeshLoadFATAsync(&bad, "nea_missing.colmesh");
    if (job != NULL)
    {
        CHECK(PumpUntilDone(job) == NEA_ASYNC_ERROR, "missing file errors");
        NEA_AsyncRelease(job);
    }
}

// ---------------------------------------------------------------------------
// Hardware 2D loaders
// ---------------------------------------------------------------------------

#define TILES_SIZE 1024

static void test_hw2d_async(void)
{
    uint8_t *tiles = malloc(TILES_SIZE);
    for (int i = 0; i < TILES_SIZE; i++)
        tiles[i] = (uint8_t)i;
    CHECK(WriteSync(TILES_PATH, tiles, TILES_SIZE), "tiles written");
    free(tiles);

    NEA_Hw2DBG *bg = NEA_Hw2DBGCreate(NEA_ENGINE_SUB, 0,
                                      NEA_HW2D_BG_TILED_8BPP, 256, 256);
    CHECK(bg != NULL, "BG created");
    if (bg == NULL)
        return;

    NEA_AsyncFile *job = NEA_Hw2DBGLoadTilesFATAsync(bg, TILES_PATH);
    CHECK(job != NULL, "BG tiles queued");
    CHECK(PumpUntilDone(job) == NEA_ASYNC_DONE, "BG tiles done");
    NEA_AsyncRelease(job);

    // The data really reached VRAM.
    const uint8_t *gfx = (const uint8_t *)NEA_Hw2DBGGetBitmapPtr(bg);
    CHECK(gfx[0] == 0 && gfx[1] == 1 && gfx[255] == 255, "BG tiles in VRAM");

    job = NEA_Hw2DBGLoadMapFATAsync(bg, TILES_PATH);
    CHECK(job != NULL, "BG map queued");
    CHECK(PumpUntilDone(job) == NEA_ASYNC_DONE, "BG map done");
    NEA_AsyncRelease(job);

    // Deleting the BG with a load in flight must abort it, not write into a
    // slot that has been handed back to the allocator.
    job = NEA_Hw2DBGLoadTilesFATAsync(bg, TILES_PATH);
    CHECK(job != NULL, "BG abort load queued");
    NEA_Hw2DBGDelete(bg);
    CHECK(PumpUntilDone(job) == NEA_ASYNC_ERROR, "BG delete aborts load");
    NEA_AsyncRelease(job);

    // OBJ sprite graphics.
    NEA_Hw2DOBJ *obj = NEA_Hw2DOBJCreate(NEA_ENGINE_SUB, NEA_OBJ_SIZE_32x32,
                                         NEA_OBJ_COLOR_256);
    CHECK(obj != NULL, "OBJ created");
    if (obj != NULL)
    {
        job = NEA_Hw2DOBJLoadGfxFATAsync(obj, TILES_PATH);
        CHECK(job != NULL, "OBJ gfx queued");
        CHECK(PumpUntilDone(job) == NEA_ASYNC_DONE, "OBJ gfx done");
        NEA_AsyncRelease(job);

        job = NEA_Hw2DOBJLoadGfxFATAsync(obj, TILES_PATH);
        CHECK(job != NULL, "OBJ abort load queued");
        NEA_Hw2DOBJDelete(obj);
        CHECK(PumpUntilDone(job) == NEA_ASYNC_ERROR, "OBJ delete aborts load");
        NEA_AsyncRelease(job);
    }

    // OBJ palette: not tied to an object, so it can only be aborted by
    // NEA_Hw2DSystemEnd().
    job = NEA_Hw2DOBJLoadPaletteFATAsync(NEA_ENGINE_SUB, TILES_PATH, 0);
    CHECK(job != NULL, "OBJ palette queued");
    CHECK(PumpUntilDone(job) == NEA_ASYNC_DONE, "OBJ palette done");
    NEA_AsyncRelease(job);

    // Shared OBJ asset.
    NEA_Hw2DOBJAsset *asset = NEA_Hw2DOBJAssetCreate(NEA_ENGINE_SUB,
                                                     NEA_OBJ_SIZE_32x32,
                                                     NEA_OBJ_COLOR_256);
    CHECK(asset != NULL, "asset created");
    if (asset != NULL)
    {
        job = NEA_Hw2DOBJAssetLoadGfxFATAsync(asset, TILES_PATH);
        CHECK(job != NULL, "asset gfx queued");
        CHECK(PumpUntilDone(job) == NEA_ASYNC_DONE, "asset gfx done");
        NEA_AsyncRelease(job);

        job = NEA_Hw2DOBJAssetLoadPaletteFATAsync(asset, TILES_PATH);
        CHECK(job != NULL, "asset palette queued");
        CHECK(PumpUntilDone(job) == NEA_ASYNC_DONE, "asset palette done");
        NEA_AsyncRelease(job);

        job = NEA_Hw2DOBJAssetLoadGfxFATAsync(asset, TILES_PATH);
        CHECK(job != NULL, "asset abort load queued");
        NEA_Hw2DOBJAssetDelete(asset);
        CHECK(PumpUntilDone(job) == NEA_ASYNC_ERROR,
              "asset delete aborts load");
        NEA_AsyncRelease(job);
    }
}

// An asset bigger than the VRAM a background owns must be truncated, not
// allowed to run on into the next background's blocks.
static void test_bg_overflow_clamp(void)
{
    // Two tiled BGs on the sub engine. The allocator gives out contiguous
    // blocks, so BG 1's tiles sit right after BG 0's: anything BG 0 writes past
    // its own 16 KB lands in BG 1.
    NEA_Hw2DBG *bg0 = NEA_Hw2DBGCreate(NEA_ENGINE_SUB, 0,
                                       NEA_HW2D_BG_TILED_8BPP, 256, 256);
    NEA_Hw2DBG *bg1 = NEA_Hw2DBGCreate(NEA_ENGINE_SUB, 1,
                                       NEA_HW2D_BG_TILED_8BPP, 256, 256);
    CHECK(bg0 != NULL && bg1 != NULL, "two BGs created");
    if (bg0 == NULL || bg1 == NULL)
        return;

    CHECK(bg0->gfx_size > 0 && bg0->map_size > 0, "BG reports its VRAM size");

    // Fill the victim with a known value so any spill is visible.
    volatile u16 *victim = bg1->gfx_ptr;
    for (size_t i = 0; i < bg0->gfx_size / 2; i++)
        victim[i] = 0xBEEF;

    // Hand BG 0 twice the tile data it can hold.
    size_t oversized = bg0->gfx_size * 2;
    u8 *big = malloc(oversized);
    CHECK(big != NULL, "oversized buffer allocated");
    if (big == NULL)
        return;
    memset(big, 0x5A, oversized);

    NEA_Hw2DBGLoadTiles(bg0, big, oversized);

    // The BG's own VRAM took the data it could hold...
    const u8 *gfx = (const u8 *)bg0->gfx_ptr;
    CHECK(gfx[0] == 0x5A && gfx[bg0->gfx_size - 1] == 0x5A, "tiles filled");

    // ...and the neighbour is untouched.
    bool spilled = false;
    for (size_t i = 0; i < bg0->gfx_size / 2; i++)
    {
        if (victim[i] != 0xBEEF)
            spilled = true;
    }
    CHECK(!spilled, "tiles did not overrun into next BG");

    // Same for the map, which has its own much smaller allocation.
    volatile u16 *map_victim = bg1->map_ptr;
    for (size_t i = 0; i < bg0->map_size / 2; i++)
        map_victim[i] = 0xCAFE;

    NEA_Hw2DBGLoadMap(bg0, big, oversized);

    spilled = false;
    for (size_t i = 0; i < bg0->map_size / 2; i++)
    {
        if (map_victim[i] != 0xCAFE)
            spilled = true;
    }
    CHECK(!spilled, "map did not overrun into next BG");

    // A palette padded out to 256 entries loaded into a high 4bpp slot must not
    // walk off the end of the palette region either.
    NEA_Hw2DBG *bg2 = NEA_Hw2DBGCreate(NEA_ENGINE_SUB, 2,
                                       NEA_HW2D_BG_TILED_4BPP, 256, 256);
    if (bg2 != NULL)
    {
        CHECK(NEA_Hw2DBGLoadPalette(bg2, big, 256, 15) == 0,
              "padded palette into last slot");
        NEA_Hw2DBGDelete(bg2);
    }

    free(big);
    NEA_Hw2DBGDelete(bg1);
    NEA_Hw2DBGDelete(bg0);
}

// Several jobs in flight at once, with a concurrency limit of 2 workers.
static void test_concurrency(void)
{
    NEA_ColMesh *m1 = NULL, *m2 = NULL, *m3 = NULL;
    NEA_AsyncFile *j1 = NEA_ColMeshLoadFATAsync(&m1, COLMESH_PATH);
    NEA_AsyncFile *j2 = NEA_ColMeshLoadFATAsync(&m2, COLMESH_PATH);
    NEA_AsyncFile *j3 = NEA_ColMeshLoadFATAsync(&m3, COLMESH_PATH);

    CHECK(j1 && j2 && j3, "three jobs queued");
    CHECK(NEA_AsyncPendingCount() == 3, "pending count is 3");

    CHECK(PumpUntilDone(j1) == NEA_ASYNC_DONE, "job 1 done");
    CHECK(PumpUntilDone(j2) == NEA_ASYNC_DONE, "job 2 done");
    CHECK(PumpUntilDone(j3) == NEA_ASYNC_DONE, "job 3 done");
    CHECK(m1 && m2 && m3, "all three loaded");

    NEA_AsyncRelease(j1);
    NEA_AsyncRelease(j2);
    NEA_AsyncRelease(j3);
    NEA_ColMeshFree(m1);
    NEA_ColMeshFree(m2);
    NEA_ColMeshFree(m3);

    // A read and a write running at the same time.
    NEA_ColMesh *m4 = NULL;
    char *buf = malloc(PAYLOAD_SIZE);
    memset(buf, 'M', PAYLOAD_SIZE);

    NEA_AsyncFile *r = NEA_ColMeshLoadFATAsync(&m4, COLMESH_PATH);
    NEA_AsyncFile *w = NEA_FATWriteDataAsync(SAVE_PATH, buf, PAYLOAD_SIZE,
                                             NEA_ASYNC_WRITE_COPY);
    CHECK(r != NULL && w != NULL, "read + write queued");
    CHECK(PumpUntilDone(r) == NEA_ASYNC_DONE, "concurrent read done");
    CHECK(PumpUntilDone(w) == NEA_ASYNC_DONE, "concurrent write done");
    NEA_AsyncRelease(r);
    NEA_AsyncRelease(w);
    NEA_ColMeshFree(m4);
    free(buf);

    char *back = NEA_FATLoadData(SAVE_PATH);
    CHECK(back != NULL && back[0] == 'M', "concurrent write landed");
    free(back);

    PumpFrames(2);
    CHECK(NEA_AsyncPendingCount() == 0, "queue drained");
}

int main(int argc, char *argv[])
{
    irqEnable(IRQ_HBLANK);
    irqSet(IRQ_VBLANK, NEA_VBLFunc);
    irqSet(IRQ_HBLANK, NEA_HBLFunc);

    NEA_Init3D();
    NEA_InitConsole();

    // Reserve VRAM banks A and B for 2D so the Hw2D tests have somewhere to
    // put their backgrounds and sprites.
    NEA_Hw2DVRAMConfig cfg = { 0 };
    cfg.sub_bg = NEA_VRAM_C;
    cfg.sub_obj = NEA_VRAM_D;
    NEA_Hw2DInit(&cfg);

    if (!isDSiMode())
        dldiSetMode(DLDI_MODE_ARM7);

    if (!fatInitDefault())
    {
        printf("fatInitDefault() failed.\n");
        printf("This test needs a\n");
        printf("writable filesystem.\n");
        while (1)
            NEA_WaitForVBL(0);
    }

    // Check the filesystem is actually writable before blaming the async code
    // for anything: a read-only mount would fail every test below.
    printf("Async API tests\n\n");
    {
        char cwd[64] = { 0 };
        getcwd(cwd, sizeof(cwd));
        printf("  cwd %s\n", cwd);

        FILE *probe = fopen(SAVE_PATH, "wb");
        CHECK(probe != NULL, "fopen wb");
        if (probe != NULL)
        {
            CHECK(fwrite("test", 1, 4, probe) == 4, "fwrite");
            CHECK(fclose(probe) == 0, "fclose");
            CHECK(NEA_FATFileSize(SAVE_PATH) == 4, "probe size");
        }
    }

    SECTION(test_write_modes);
    SECTION(test_write_atomicity);
    SECTION(test_collision_async);
    SECTION(test_hw2d_async);
    SECTION(test_bg_overflow_clamp);
    SECTION(test_concurrency);

    // Tearing the 2D system down with nothing in flight must be uneventful.
    NEA_Hw2DSystemEnd();
    PumpFrames(2);
    CHECK(NEA_AsyncPendingCount() == 0, "nothing left pending");

    printf("\n%d checks, %d failed\n", checks, failures);
    if (failures == 0)
        printf("ALL TESTS PASSED\n");
    else
        printf("TESTS FAILED\n");

    while (1)
        NEA_WaitForVBL(0);

    return 0;
}
