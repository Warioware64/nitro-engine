// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Warioware64
//
// This file is part of Nitro Engine Advanced

#include <stdarg.h>

#include "NEAMain.h"
#include "dsf.h"

/// @file NEAHw2D.c

// ---------------------------------------------------------------------------
// Internal state
// ---------------------------------------------------------------------------

#define NEA_HW2D_MAX_OAM      128
#define NEA_HW2D_MAX_ASSETS   32
#define NEA_HW2D_MAX_TEXT_CTX 8

struct NEA_Hw2DTextCtx {
    bool used;
    dsf_renderer renderer;
    dsf_handle font_handle;          // 0 if no metadata loaded yet
    bool owns_font_handle;           // true if loaded via MetadataLoad*
    bool bitmap_set;                 // true once a font texture is attached
    void *owned_bitmap;              // malloc'd by BitmapLoadGRF, owned by ctx
    void *owned_palette;             // malloc'd by BitmapLoadGRF, owned by ctx
    // Destination. Exactly one of these is set: `bg` for a ctx created with
    // NEA_Hw2DTextCtxCreate (libdsf writes into VRAM), `buffer` for one created
    // with NEA_Hw2DTextCtxCreateBuffer (libdsf writes into caller RAM, and
    // NEA_Hw2DTextCtxBlitToBG moves the result into a BG).
    NEA_Hw2DBG *bg;
    void *buffer;
    int buf_width, buf_height;       // dimensions of `buffer`, 0 when bg-backed
    dsf_format fmt;
    // Canvas in destination-pixel coords, cached so Clear can wipe the right rect.
    int canvas_left, canvas_top, canvas_right, canvas_bottom;
};

// BG VRAM block tracking — modeled on nflib (NF_TILEBLOCKS / NF_MAPBLOCKS).
//
// Each "tile block" is 16 KB (the BG_TILE_BASE / BG_BMP_BASE unit on NDS).
// Each "map block" is 2 KB (the BG_MAP_BASE unit). 8 map blocks = 1 tile block.
//
// At init, the first NEA_HW2D_RESERVED_TILE_BANKS tile blocks are reserved
// to host map data (marked with sentinel 128). Maps allocate from
// map_blocks_*; tile / bitmap data allocate from tile_blocks_*. This
// prevents the silent map↔tile overlap the previous "advance-by-one"
// scheme allowed (e.g. 9 small tiled BGs, or 2 BGs with 512x512 maps).
#define NEA_HW2D_TILE_BANKS          8 // up to 8 * 16 KB = 128 KB per engine
#define NEA_HW2D_MAP_BANKS           8 // 8 * 2 KB = 16 KB of map region
#define NEA_HW2D_RESERVED_TILE_BANKS 1 // first N tile blocks reserved for maps
#define NEA_HW2D_BLOCK_FREE          0
#define NEA_HW2D_BLOCK_MAP_RESERVED  128
#define NEA_HW2D_BLOCK_USED          255

typedef struct {
    u8 tile_base;   // First tile block used (16 KB units)
    u8 tile_blocks; // Number of tile blocks used
    u8 map_base;    // First map block used (2 KB units), 0xFF if not applicable
    u8 map_blocks;  // Number of map blocks used
} ne_hw2d_bg_alloc_t;

static struct {
    bool initialized;
    NEA_Hw2DVRAMConfig vram_config;
    NEA_VRAMBankFlags claimed_banks;

    NEA_Hw2DBG bgs_main[4];
    NEA_Hw2DBG bgs_sub[4];

    NEA_Hw2DOBJ objs_main[NEA_HW2D_MAX_OAM];
    NEA_Hw2DOBJ objs_sub[NEA_HW2D_MAX_OAM];

    NEA_Hw2DOBJAsset assets_main[NEA_HW2D_MAX_ASSETS];
    NEA_Hw2DOBJAsset assets_sub[NEA_HW2D_MAX_ASSETS];

    struct NEA_Hw2DTextCtx text_ctx[NEA_HW2D_MAX_TEXT_CTX];

    // 16-color OBJ palette slot bitmap (bit N = slot N in use). 256-color
    // assets always use slot 0 and are not tracked here — mixing 16- and
    // 256-color sprites on the same engine shares the OBJ palette region
    // and is the caller's responsibility, same as the manual API.
    u16 obj_pal_used_main;
    u16 obj_pal_used_sub;

    u8 tile_blocks_main[NEA_HW2D_TILE_BANKS];
    u8 tile_blocks_sub[NEA_HW2D_TILE_BANKS];
    u8 map_blocks_main[NEA_HW2D_MAP_BANKS];
    u8 map_blocks_sub[NEA_HW2D_MAP_BANKS];

    ne_hw2d_bg_alloc_t bg_alloc_main[4];
    ne_hw2d_bg_alloc_t bg_alloc_sub[4];

    int next_oam_main;
    int next_oam_sub;

    bool main_obj_inited;
    bool sub_obj_inited;
    bool main_has_bitmap;
} ne_hw2d_state;

// Find a contiguous run of `needed` blocks marked with `free_value`.
// Returns the starting index, or -1 if none found.
static int ne_find_contiguous(const u8 *blocks, int count, int needed,
                               u8 free_value)
{
    int run_start = -1;
    int run_len = 0;
    for (int i = 0; i < count; i++)
    {
        if (blocks[i] == free_value)
        {
            if (run_len == 0)
                run_start = i;
            run_len++;
            if (run_len >= needed)
                return run_start;
        }
        else
        {
            run_len = 0;
            run_start = -1;
        }
    }
    return -1;
}

// Safe copy into VRAM / palette / OAM memory. Those regions only accept
// 16-bit and 32-bit writes — an 8-bit store corrupts the other half of the
// 16-bit unit. A plain memcpy makes no width guarantee (it byte-copies an
// unaligned head/tail or odd length), so use swiCopy, which always transfers
// in 16- or 32-bit units and stays cache-coherent (unlike DMA, which would
// need a DC_FlushRange on the source first). Prefer 32-bit transfers when
// everything is word-aligned, else fall back to the 16-bit minimum.
static void ne_vram_copy(void *dst, const void *src, size_t bytes)
{
    if ((((uintptr_t)dst | (uintptr_t)src | bytes) & 3) == 0)
        swiCopy(src, dst, (bytes >> 2) | COPY_MODE_WORD);
    else
        swiCopy(src, dst, (bytes >> 1) | COPY_MODE_HWORD);
}

// ---------------------------------------------------------------------------
// Phase 1: System initialization
// ---------------------------------------------------------------------------

int NEA_Hw2DInit(const NEA_Hw2DVRAMConfig *config)
{
    if (ne_hw2d_state.initialized)
        NEA_Hw2DSystemEnd();

    NEA_AssertPointer(config, "NEA_Hw2DInit: NULL config");

    memset(&ne_hw2d_state, 0, sizeof(ne_hw2d_state));
    ne_hw2d_state.vram_config = *config;

    NEA_VRAMBankFlags mb = config->main_bg;
    NEA_VRAMBankFlags mo = config->main_obj;
    NEA_VRAMBankFlags sb = config->sub_bg;
    NEA_VRAMBankFlags so = config->sub_obj;

    // Check that no bank is assigned to multiple fields
    NEA_VRAMBankFlags all = mb | mo | sb | so;
    NEA_VRAMBankFlags overlap = 0;
    overlap |= (mb & mo) | (mb & sb) | (mb & so);
    overlap |= (mo & sb) | (mo & so);
    overlap |= (sb & so);
    if (overlap)
    {
        NEA_DebugPrint("NEA_Hw2DInit: VRAM bank assigned to multiple fields");
        return -1;
    }

    // Validate main BG banks (valid: A, B, C, E)
    if (mb & ~(NEA_VRAM_A | NEA_VRAM_B | NEA_VRAM_C | NEA_VRAM_E))
    {
        NEA_DebugPrint("NEA_Hw2DInit: Invalid main_bg banks");
        return -1;
    }

    // Validate main OBJ banks (valid: A, B, E)
    if (mo & ~(NEA_VRAM_A | NEA_VRAM_B | NEA_VRAM_E))
    {
        NEA_DebugPrint("NEA_Hw2DInit: Invalid main_obj banks");
        return -1;
    }

    // Validate sub BG banks (valid: C, H, I)
    if (sb & ~(NEA_VRAM_C | NEA_VRAM_H | NEA_VRAM_I))
    {
        NEA_DebugPrint("NEA_Hw2DInit: Invalid sub_bg banks");
        return -1;
    }

    // Validate sub OBJ banks (valid: D, I)
    if (so & ~(NEA_VRAM_D | NEA_VRAM_I))
    {
        NEA_DebugPrint("NEA_Hw2DInit: Invalid sub_obj banks");
        return -1;
    }

    // --- Configure VRAM banks ---

    // If bank E is claimed for 2D, remove it from texture palette duty. This
    // prevents ne_init_registers() from re-assigning it on future inits.
    //
    // Only bank E is dropped: banks F and G can still back texture palettes,
    // and clearing them here would leave a later NEA_TextureSystemReset() with
    // no palette VRAM at all, which makes every palette load fail.
    if ((mb | mo) & NEA_VRAM_E)
    {
        NEA_VRAMBankFlags pal_banks = NEA_GetTexPaletteBank();
        NEA_SetTexPaletteBank(pal_banks & ~NEA_VRAM_E);
    }

    // Main BG
    if (mb & NEA_VRAM_A)
        vramSetBankA(VRAM_A_MAIN_BG_0x06000000);
    if (mb & NEA_VRAM_B)
        vramSetBankB(VRAM_B_MAIN_BG_0x06020000);
    if (mb & NEA_VRAM_C)
        vramSetBankC(VRAM_C_MAIN_BG_0x06040000);
    if (mb & NEA_VRAM_E)
        vramSetBankE(VRAM_E_MAIN_BG);

    // Main OBJ
    if (mo & NEA_VRAM_A)
        vramSetBankA(VRAM_A_MAIN_SPRITE_0x06400000);
    if (mo & NEA_VRAM_B)
        vramSetBankB(VRAM_B_MAIN_SPRITE_0x06400000);
    if (mo & NEA_VRAM_E)
        vramSetBankE(VRAM_E_MAIN_SPRITE);

    // Sub BG
    if (sb & NEA_VRAM_C)
        vramSetBankC(VRAM_C_SUB_BG_0x06200000);
    if (sb & NEA_VRAM_H)
        vramSetBankH(VRAM_H_SUB_BG);
    if (sb & NEA_VRAM_I)
        vramSetBankI(VRAM_I_SUB_BG_0x06208000);

    // Sub OBJ
    if (so & NEA_VRAM_D)
        vramSetBankD(VRAM_D_SUB_SPRITE);
    if (so & NEA_VRAM_I)
        vramSetBankI(VRAM_I_SUB_SPRITE);

    ne_hw2d_state.claimed_banks = all;

    // --- Configure video modes ---

    // Main engine: BG0 is already 3D (MODE_0_3D).
    // Individual BG layers are enabled by bgInit()/bgShow() when
    // NEA_Hw2DBGCreate() is called — don't blanket-enable them here,
    // as unconfigured layers would render garbage at priority 0.

    // Main engine sprites
    if (mo)
    {
        oamInit(&oamMain, SpriteMapping_1D_128, false);
        REG_DISPCNT |= DISPLAY_SPR_ACTIVE | DISPLAY_SPR_1D;
        ne_hw2d_state.main_obj_inited = true;
    }

    // Sub engine: set up base video mode. Individual BG layers are enabled
    // by bgInitSub()/bgShow() in NEA_Hw2DBGCreate().
    if (sb || so)
    {
        // Set the base mode FIRST, then let oamInit() configure the OBJ tile
        // mapping/boundary bits (DISPLAY_SPR_1D_SIZE_128 = DISPCNT bits 20-21).
        // videoSetModeSub() assigns the whole register, so running it after
        // oamInit() wipes those bits back to SIZE_32 while libnds still computes
        // gfx offsets for a 128-byte boundary, making sprites fetch the wrong
        // gfx. Mirror the main-engine path, which uses |= to preserve the bits.
        videoSetModeSub(MODE_0_2D);
        if (so)
        {
            oamInit(&oamSub, SpriteMapping_1D_128, false);
            REG_DISPCNT_SUB |= DISPLAY_SPR_ACTIVE | DISPLAY_SPR_1D;
            ne_hw2d_state.sub_obj_inited = true;
        }
    }

    // BG VRAM block allocation: mark the first
    // NEA_HW2D_RESERVED_TILE_BANKS tile blocks as reserved for map data
    // (sentinel 128 = the same value nflib uses for this state). Map
    // blocks live inside that region; tile/bitmap data allocates from
    // tile_blocks[N..].
    for (int i = 0; i < NEA_HW2D_RESERVED_TILE_BANKS; i++)
    {
        ne_hw2d_state.tile_blocks_main[i] = NEA_HW2D_BLOCK_MAP_RESERVED;
        ne_hw2d_state.tile_blocks_sub[i]  = NEA_HW2D_BLOCK_MAP_RESERVED;
    }
    // bg_alloc_* slots default to 0xFF (no allocation) so Delete can
    // skip freeing for BG slots that were never created.
    memset(ne_hw2d_state.bg_alloc_main, 0xFF,
           sizeof(ne_hw2d_state.bg_alloc_main));
    memset(ne_hw2d_state.bg_alloc_sub, 0xFF,
           sizeof(ne_hw2d_state.bg_alloc_sub));

    ne_hw2d_state.initialized = true;
    return 0;
}

int NEA_Hw2DAutoInit(void)
{
    NEA_ExecutionModes mode = NEA_CurrentExecutionMode();
    if (mode == NEA_ModeUninitialized)
    {
        NEA_DebugPrint("NEA_Hw2DAutoInit: 3D not initialized");
        return -1;
    }

    NEA_Hw2DVRAMConfig cfg = { 0 };

    // Sub engine: banks H (32 KB) and I (16 KB) are never used by 3D and
    // are free unless libnds already owns the sub engine for dual-3D
    // display (Dual3D modes route the sub LCD via banks C/D and configure
    // sub video themselves).
    bool sub_free = (mode != NEA_ModeDual3D)
                 && (mode != NEA_ModeDual3D_FB)
                 && (mode != NEA_ModeDual3D_DMA);
    if (sub_free)
    {
        cfg.sub_bg = NEA_VRAM_H;
        cfg.sub_obj = NEA_VRAM_I;
    }

    // Main engine: bank E (64 KB) is the only main-engine 2D bank reachable
    // without displacing 3D textures (banks A-D). Only safe in single 3D
    // mode (two-pass modes drive the main display via FIFO or BG2 capture,
    // and dual 3D claims the main output for the 3D engine), and only if
    // the texture palette system isn't currently using bank E.
    if (mode == NEA_ModeSingle3D
        && !(NEA_GetTexPaletteBank() & NEA_VRAM_E))
    {
        cfg.main_bg = NEA_VRAM_E;
    }

    return NEA_Hw2DInit(&cfg);
}

void NEA_Hw2DSystemEnd(void)
{
    if (!ne_hw2d_state.initialized)
        return;

    // Abort every load still aimed at anything in the pools. This function
    // clears them directly instead of going through the Delete() calls, which
    // are what normally cancel the loads, so it has to do it itself before the
    // slots are zeroed and the VRAM banks are released.
    for (int i = 0; i < 4; i++)
    {
        __NEA_AsyncCancelTarget(&ne_hw2d_state.bgs_main[i]);
        __NEA_AsyncCancelTarget(&ne_hw2d_state.bgs_sub[i]);
    }
    for (int i = 0; i < NEA_HW2D_MAX_OAM; i++)
    {
        __NEA_AsyncCancelTarget(&ne_hw2d_state.objs_main[i]);
        __NEA_AsyncCancelTarget(&ne_hw2d_state.objs_sub[i]);
    }
    for (int i = 0; i < NEA_HW2D_MAX_ASSETS; i++)
    {
        __NEA_AsyncCancelTarget(&ne_hw2d_state.assets_main[i]);
        __NEA_AsyncCancelTarget(&ne_hw2d_state.assets_sub[i]);
    }
    for (int i = 0; i < NEA_HW2D_MAX_TEXT_CTX; i++)
        __NEA_AsyncCancelTarget(&ne_hw2d_state.text_ctx[i]);

    // OBJ palette loads write into the engine's palette region rather than into
    // an object, so they are registered against the 2D system itself.
    __NEA_AsyncCancelTarget(&ne_hw2d_state);

    // Delete all active BGs
    for (int i = 0; i < 4; i++)
    {
        if (ne_hw2d_state.bgs_main[i].used)
        {
            bgHide(ne_hw2d_state.bgs_main[i].bg_id);
            ne_hw2d_state.bgs_main[i].used = false;
        }
        if (ne_hw2d_state.bgs_sub[i].used)
        {
            bgHide(ne_hw2d_state.bgs_sub[i].bg_id);
            ne_hw2d_state.bgs_sub[i].used = false;
        }
    }

    // Free all OBJ sprites. Asset-bound OBJs don't own their gfx — the asset
    // cleanup pass below releases the shared allocation. Standalone OBJs
    // free their own gfx here.
    OamState *oam;
    for (int e = 0; e < 2; e++)
    {
        NEA_Hw2DOBJ *objs = (e == 0) ? ne_hw2d_state.objs_main
                                      : ne_hw2d_state.objs_sub;
        oam = (e == 0) ? &oamMain : &oamSub;
        int max = (e == 0) ? ne_hw2d_state.next_oam_main
                           : ne_hw2d_state.next_oam_sub;
        for (int i = 0; i < max; i++)
        {
            if (!objs[i].used)
                continue;
            if (objs[i].asset == NULL && objs[i].gfx)
                oamFreeGfx(oam, objs[i].gfx);
            objs[i].used = false;
        }
    }

    // Free all shared assets (gfx pointers and 16-color palette slots).
    for (int e = 0; e < 2; e++)
    {
        NEA_Hw2DOBJAsset *assets = (e == 0) ? ne_hw2d_state.assets_main
                                              : ne_hw2d_state.assets_sub;
        oam = (e == 0) ? &oamMain : &oamSub;
        for (int i = 0; i < NEA_HW2D_MAX_ASSETS; i++)
        {
            if (!assets[i].used)
                continue;
            if (assets[i].gfx)
                oamFreeGfx(oam, assets[i].gfx);
            assets[i].used = false;
        }
    }

    // Free all active text contexts. Renderers hold malloc'd state, and
    // contexts may own a libdsf font handle + GRF-loaded bitmap/palette.
    for (int i = 0; i < NEA_HW2D_MAX_TEXT_CTX; i++)
    {
        NEA_Hw2DTextCtx *ctx = &ne_hw2d_state.text_ctx[i];
        if (!ctx->used)
            continue;
        DSF_RendererFree(ctx->renderer);
        if (ctx->owns_font_handle && ctx->font_handle != 0)
            DSF_FreeFont(&ctx->font_handle);
        free(ctx->owned_bitmap);
        free(ctx->owned_palette);
        ctx->used = false;
    }

    // Reset VRAM banks to LCD mode
    NEA_VRAMBankFlags claimed = ne_hw2d_state.claimed_banks;
    if (claimed & NEA_VRAM_A)
        vramSetBankA(VRAM_A_LCD);
    if (claimed & NEA_VRAM_B)
        vramSetBankB(VRAM_B_LCD);
    if (claimed & NEA_VRAM_C)
        vramSetBankC(VRAM_C_LCD);
    if (claimed & NEA_VRAM_D)
        vramSetBankD(VRAM_D_LCD);
    if (claimed & NEA_VRAM_E)
        vramSetBankE(VRAM_E_LCD);
    if (claimed & NEA_VRAM_H)
        vramSetBankH(VRAM_H_LCD);
    if (claimed & NEA_VRAM_I)
        vramSetBankI(VRAM_I_LCD);

    memset(&ne_hw2d_state, 0, sizeof(ne_hw2d_state));
}

NEA_VRAMBankFlags NEA_Hw2DGetClaimedBanks(void)
{
    return ne_hw2d_state.claimed_banks;
}

// ---------------------------------------------------------------------------
// Phase 2: Tiled backgrounds
// ---------------------------------------------------------------------------

// Map NEA BG type + size to libnds BgType and BgSize
static int ne_hw2d_bg_params(NEA_Hw2DBGType type, int width, int height,
                              BgType *out_type, BgSize *out_size)
{
    switch (type)
    {
    case NEA_HW2D_BG_TILED_4BPP:
        *out_type = BgType_Text4bpp;
        break;
    case NEA_HW2D_BG_TILED_8BPP:
        *out_type = BgType_Text8bpp;
        break;
    case NEA_HW2D_BG_BITMAP_8:
        *out_type = BgType_Bmp8;
        break;
    case NEA_HW2D_BG_BITMAP_16:
        *out_type = BgType_Bmp16;
        break;
    default:
        return -1;
    }

    // Map dimensions to BgSize
    if (type <= NEA_HW2D_BG_TILED_8BPP)
    {
        // Tiled BG sizes
        if (width == 256 && height == 256)
            *out_size = BgSize_T_256x256;
        else if (width == 512 && height == 256)
            *out_size = BgSize_T_512x256;
        else if (width == 256 && height == 512)
            *out_size = BgSize_T_256x512;
        else if (width == 512 && height == 512)
            *out_size = BgSize_T_512x512;
        else
            return -1;
    }
    else
    {
        // Bitmap BG sizes
        if (width == 256 && height == 256)
        {
            *out_size = (type == NEA_HW2D_BG_BITMAP_8)
                      ? BgSize_B8_256x256 : BgSize_B16_256x256;
        }
        else
        {
            return -1;
        }
    }

    return 0;
}

NEA_Hw2DBG *NEA_Hw2DBGCreate(NEA_Hw2DEngine engine, int layer,
                              NEA_Hw2DBGType type, int width, int height)
{
    NEA_Assert(ne_hw2d_state.initialized, "NEA_Hw2DBGCreate: Hw2D not init");

    // Validate layer
    if (engine == NEA_ENGINE_MAIN && (layer < 1 || layer > 3))
    {
        NEA_DebugPrint("NEA_Hw2DBGCreate: main engine layer must be 1-3");
        return NULL;
    }
    if (engine == NEA_ENGINE_SUB && (layer < 0 || layer > 3))
    {
        NEA_DebugPrint("NEA_Hw2DBGCreate: sub engine layer must be 0-3");
        return NULL;
    }

    NEA_Hw2DBG *bg = (engine == NEA_ENGINE_MAIN)
                    ? &ne_hw2d_state.bgs_main[layer]
                    : &ne_hw2d_state.bgs_sub[layer];

    if (bg->used)
    {
        NEA_DebugPrint("NEA_Hw2DBGCreate: layer already in use");
        return NULL;
    }

    BgType bg_type;
    BgSize bg_size;
    if (ne_hw2d_bg_params(type, width, height, &bg_type, &bg_size) != 0)
    {
        NEA_DebugPrint("NEA_Hw2DBGCreate: invalid type/size combination");
        return NULL;
    }

    // For bitmap BGs on main engine, switch to MODE_5_3D
    bool is_bitmap = (type == NEA_HW2D_BG_BITMAP_8
                   || type == NEA_HW2D_BG_BITMAP_16);

    if (is_bitmap && engine == NEA_ENGINE_MAIN)
    {
        if (layer < 2)
        {
            NEA_DebugPrint("NEA_Hw2DBGCreate: bitmap BGs need layer 2 or 3");
            return NULL;
        }
        if (!ne_hw2d_state.main_has_bitmap)
        {
            // Switch to MODE_5_3D for bitmap BGs. Don't pre-enable BG2/3 —
            // bgInit()/bgShow() enables the specific layer when created.
            videoSetMode(MODE_5_3D
                       | (REG_DISPCNT & (DISPLAY_BG1_ACTIVE
                       | DISPLAY_SPR_ACTIVE | DISPLAY_SPR_1D)));
            ne_hw2d_state.main_has_bitmap = true;
        }
    }

    // Allocate VRAM blocks for this BG using the nflib-style block-bitmap
    // allocator. Each tile block is 16 KB (BG_TILE_BASE / BG_BMP_BASE unit);
    // each map block is 2 KB (BG_MAP_BASE unit). Map blocks live in the
    // tile blocks reserved at init time. find_contiguous() picks the
    // first free run, matching nflib's first-fit behaviour.
    u8 *tile_blocks = (engine == NEA_ENGINE_MAIN)
                      ? ne_hw2d_state.tile_blocks_main
                      : ne_hw2d_state.tile_blocks_sub;
    u8 *map_blocks = (engine == NEA_ENGINE_MAIN)
                     ? ne_hw2d_state.map_blocks_main
                     : ne_hw2d_state.map_blocks_sub;
    ne_hw2d_bg_alloc_t *alloc_slot = (engine == NEA_ENGINE_MAIN)
                                     ? &ne_hw2d_state.bg_alloc_main[layer]
                                     : &ne_hw2d_state.bg_alloc_sub[layer];

    int tile_base = 0;
    int map_base = 0;
    int tile_blocks_needed;
    int map_blocks_needed;

    if (is_bitmap)
    {
        // Bitmap BGs occupy contiguous 16 KB units. The libnds bgInit
        // mapBase parameter is the bitmap base for bitmap BG types
        // (BG_BMP_BASE = 16 KB units), so we pass our tile_base index as
        // map_base and leave tile_base = 0.
        int bmp_bytes = width * height
                        * ((type == NEA_HW2D_BG_BITMAP_16) ? 2 : 1);
        tile_blocks_needed = (bmp_bytes + 16383) / 16384;
        map_blocks_needed = 0;

        int start = ne_find_contiguous(tile_blocks, NEA_HW2D_TILE_BANKS,
                                        tile_blocks_needed,
                                        NEA_HW2D_BLOCK_FREE);
        if (start < 0)
        {
            NEA_DebugPrint("NEA_Hw2DBGCreate: out of BG VRAM (bitmap %d KB)",
                           bmp_bytes / 1024);
            return NULL;
        }
        for (int i = 0; i < tile_blocks_needed; i++)
            tile_blocks[start + i] = NEA_HW2D_BLOCK_USED;

        tile_base = 0;
        map_base = start;
        alloc_slot->tile_base = start;
        alloc_slot->tile_blocks = tile_blocks_needed;
        alloc_slot->map_base = 0xFF;
        alloc_slot->map_blocks = 0;
    }
    else
    {
        // Tiled BG: 1 tile block for the tileset, plus map slots
        // proportional to the screen size (each 32x32 quadrant = 2 KB).
        map_blocks_needed = 1;
        if (width == 512)
            map_blocks_needed *= 2;
        if (height == 512)
            map_blocks_needed *= 2;
        tile_blocks_needed = 1;

        int tile_start = ne_find_contiguous(tile_blocks,
                                             NEA_HW2D_TILE_BANKS,
                                             tile_blocks_needed,
                                             NEA_HW2D_BLOCK_FREE);
        if (tile_start < 0)
        {
            NEA_DebugPrint("NEA_Hw2DBGCreate: out of tile VRAM");
            return NULL;
        }

        int map_start = ne_find_contiguous(map_blocks, NEA_HW2D_MAP_BANKS,
                                            map_blocks_needed,
                                            NEA_HW2D_BLOCK_FREE);
        if (map_start < 0)
        {
            NEA_DebugPrint("NEA_Hw2DBGCreate: out of map VRAM");
            return NULL;
        }

        tile_blocks[tile_start] = NEA_HW2D_BLOCK_USED;
        for (int i = 0; i < map_blocks_needed; i++)
            map_blocks[map_start + i] = NEA_HW2D_BLOCK_USED;

        tile_base = tile_start;
        map_base = map_start;
        alloc_slot->tile_base = tile_start;
        alloc_slot->tile_blocks = 1;
        alloc_slot->map_base = map_start;
        alloc_slot->map_blocks = map_blocks_needed;
    }

    int bg_id;
    if (engine == NEA_ENGINE_MAIN)
        bg_id = bgInit(layer, bg_type, bg_size, map_base, tile_base);
    else
        bg_id = bgInitSub(layer, bg_type, bg_size, map_base, tile_base);

    if (bg_id < 0)
    {
        NEA_DebugPrint("NEA_Hw2DBGCreate: bgInit failed");
        return NULL;
    }

    bg->used = true;
    bg->engine = engine;
    bg->layer = layer;
    bg->type = type;
    bg->bg_id = bg_id;
    bg->width = width;
    bg->height = height;
    bg->gfx_ptr = bgGetGfxPtr(bg_id);
    bg->map_ptr = is_bitmap ? NULL : bgGetMapPtr(bg_id);

    // How much VRAM this BG actually owns, so that the loaders can refuse to
    // write past it into whatever the allocator handed to the next layer.
    //
    // A bitmap BG is bounded by the image itself rather than by the blocks it
    // was rounded up to: the tail of the last 16 KB block isn't displayed, and
    // treating it as usable would only hide an oversized file.
    if (is_bitmap)
    {
        bg->gfx_size = (size_t)width * height
                       * ((type == NEA_HW2D_BG_BITMAP_16) ? 2 : 1);
        bg->map_size = 0;
    }
    else
    {
        bg->gfx_size = (size_t)tile_blocks_needed * 16384;
        bg->map_size = (size_t)map_blocks_needed * 2048;
    }

    bg->scroll_x = 0;
    bg->scroll_y = 0;
    bg->visible = true;

    return bg;
}

void NEA_Hw2DBGDelete(NEA_Hw2DBG *bg)
{
    if (bg == NULL || !bg->used)
        return;

    // This slot is about to be cleared and its VRAM handed back to the
    // allocator, so any load still aimed at it must never reach its finalize
    // step.
    __NEA_AsyncCancelTarget(bg);

    // Return the BG's tile and map blocks to the free pool. Without this,
    // the allocator would treat them as used forever and repeated
    // create/delete cycles would exhaust VRAM after a few iterations.
    int layer = bg->layer;
    u8 *tile_blocks = (bg->engine == NEA_ENGINE_MAIN)
                      ? ne_hw2d_state.tile_blocks_main
                      : ne_hw2d_state.tile_blocks_sub;
    u8 *map_blocks = (bg->engine == NEA_ENGINE_MAIN)
                     ? ne_hw2d_state.map_blocks_main
                     : ne_hw2d_state.map_blocks_sub;
    ne_hw2d_bg_alloc_t *alloc_slot = (bg->engine == NEA_ENGINE_MAIN)
                                     ? &ne_hw2d_state.bg_alloc_main[layer]
                                     : &ne_hw2d_state.bg_alloc_sub[layer];

    if (alloc_slot->tile_base != 0xFF)
    {
        for (int i = 0; i < alloc_slot->tile_blocks; i++)
            tile_blocks[alloc_slot->tile_base + i] = NEA_HW2D_BLOCK_FREE;
    }
    if (alloc_slot->map_base != 0xFF)
    {
        for (int i = 0; i < alloc_slot->map_blocks; i++)
            map_blocks[alloc_slot->map_base + i] = NEA_HW2D_BLOCK_FREE;
    }
    memset(alloc_slot, 0xFF, sizeof(*alloc_slot));

    bgHide(bg->bg_id);
    memset(bg, 0, sizeof(*bg));
}

// Clamps a copy to the VRAM a BG actually owns.
//
// The BG allocator hands out contiguous blocks, so the bytes just past the end
// of one BG belong to another one. Copying a whole file in without checking
// silently corrupts a neighbouring layer, which shows up as garbage in an
// unrelated background rather than as an error at the point of the mistake.
static size_t ne_hw2d_bg_clip(size_t size, size_t capacity, const char *what)
{
    if (size <= capacity)
        return size;

    NEA_DebugPrint("Hw2D %s data is %d bytes, VRAM holds %d; truncating",
                   what, (int)size, (int)capacity);
    return capacity;
}

int NEA_Hw2DBGLoadTiles(NEA_Hw2DBG *bg, const void *data, size_t size)
{
    NEA_AssertPointer(bg, "NULL bg");
    NEA_AssertPointer(data, "NULL data");
    NEA_Assert(bg->used, "BG not active");
    NEA_Assert(bg->type <= NEA_HW2D_BG_TILED_8BPP, "Not a tiled BG");

    memcpy(bg->gfx_ptr, data, ne_hw2d_bg_clip(size, bg->gfx_size, "tile"));
    return 0;
}

int NEA_Hw2DBGLoadTilesFAT(NEA_Hw2DBG *bg, const char *path)
{
    NEA_AssertPointer(bg, "NULL bg");
    NEA_AssertPointer(path, "NULL path");
    NEA_Assert(bg->used, "BG not active");
    NEA_Assert(bg->type <= NEA_HW2D_BG_TILED_8BPP, "Not a tiled BG");

    size_t size = 0;
    void *buf = __NEA_FATLoadDataSize(path, &size);
    if (buf == NULL)
        return -1;

    int ret = NEA_Hw2DBGLoadTiles(bg, buf, size);
    free(buf);
    return ret;
}

int NEA_Hw2DBGLoadMap(NEA_Hw2DBG *bg, const void *data, size_t size)
{
    NEA_AssertPointer(bg, "NULL bg");
    NEA_AssertPointer(data, "NULL data");
    NEA_Assert(bg->used, "BG not active");
    NEA_Assert(bg->map_ptr != NULL, "Not a tiled BG");

    memcpy(bg->map_ptr, data, ne_hw2d_bg_clip(size, bg->map_size, "map"));
    return 0;
}

int NEA_Hw2DBGLoadMapFAT(NEA_Hw2DBG *bg, const char *path)
{
    NEA_AssertPointer(bg, "NULL bg");
    NEA_AssertPointer(path, "NULL path");
    NEA_Assert(bg->used, "BG not active");
    NEA_Assert(bg->map_ptr != NULL, "Not a tiled BG");

    size_t size = 0;
    void *buf = __NEA_FATLoadDataSize(path, &size);
    if (buf == NULL)
        return -1;

    int ret = NEA_Hw2DBGLoadMap(bg, buf, size);
    free(buf);
    return ret;
}

int NEA_Hw2DBGLoadPalette(NEA_Hw2DBG *bg, const void *data,
                           int num_colors, int slot)
{
    NEA_AssertPointer(bg, "NULL bg");
    NEA_AssertPointer(data, "NULL data");
    NEA_Assert(bg->used, "BG not active");

    u16 *pal_ptr;
    if (bg->engine == NEA_ENGINE_MAIN)
        pal_ptr = BG_PALETTE;
    else
        pal_ptr = BG_PALETTE_SUB;

    // For 4bpp, each slot is 16 colors (32 bytes)
    int first_color = 0;
    if (bg->type == NEA_HW2D_BG_TILED_4BPP)
        first_color = slot * 16;

    pal_ptr += first_color;

    // The BG palette region holds 256 colors. Clamp the copy the same way
    // NEA_Hw2DOBJLoadPalette() does, so a palette padded out to 256 entries
    // (which grit and ptexconv both do for 4bpp) can't run off the end of the
    // region when it is loaded into a non-zero slot.
    int max_colors = 256 - first_color;
    if (max_colors < 0)
        max_colors = 0;
    if (num_colors > max_colors)
        num_colors = max_colors;

    memcpy(pal_ptr, data, num_colors * 2);
    return 0;
}

void NEA_Hw2DBGSetScroll(NEA_Hw2DBG *bg, int x, int y)
{
    NEA_AssertPointer(bg, "NULL bg");
    NEA_Assert(bg->used, "BG not active");

    bg->scroll_x = x;
    bg->scroll_y = y;
    bgSetScroll(bg->bg_id, x, y);
    bgUpdate();
}

void NEA_Hw2DBGSetPriority(NEA_Hw2DBG *bg, int priority)
{
    NEA_AssertPointer(bg, "NULL bg");
    NEA_Assert(bg->used, "BG not active");

    bgSetPriority(bg->bg_id, priority);
}

void NEA_Hw2DBGSetVisible(NEA_Hw2DBG *bg, bool visible)
{
    NEA_AssertPointer(bg, "NULL bg");
    NEA_Assert(bg->used, "BG not active");

    bg->visible = visible;
    if (visible)
        bgShow(bg->bg_id);
    else
        bgHide(bg->bg_id);
}

void NEA_Hw2DBGSetTile(NEA_Hw2DBG *bg, int x, int y, u16 value)
{
    NEA_AssertPointer(bg, "NULL bg");
    NEA_Assert(bg->used && bg->map_ptr, "Invalid tiled BG");

    int tiles_per_row = bg->width / 8;
    bg->map_ptr[y * tiles_per_row + x] = value;
}

u16 NEA_Hw2DBGGetTile(const NEA_Hw2DBG *bg, int x, int y)
{
    NEA_AssertPointer(bg, "NULL bg");
    NEA_Assert(bg->used && bg->map_ptr, "Invalid tiled BG");

    int tiles_per_row = bg->width / 8;
    return bg->map_ptr[y * tiles_per_row + x];
}

// ---------------------------------------------------------------------------
// Phase 3: Bitmap backgrounds
// ---------------------------------------------------------------------------

int NEA_Hw2DBGLoadBitmap(NEA_Hw2DBG *bg, const void *data, size_t size)
{
    NEA_AssertPointer(bg, "NULL bg");
    NEA_AssertPointer(data, "NULL data");
    NEA_Assert(bg->used, "BG not active");
    NEA_Assert(bg->type >= NEA_HW2D_BG_BITMAP_8, "Not a bitmap BG");

    memcpy(bg->gfx_ptr, data, ne_hw2d_bg_clip(size, bg->gfx_size, "bitmap"));
    return 0;
}

int NEA_Hw2DBGLoadBitmapFAT(NEA_Hw2DBG *bg, const char *path)
{
    NEA_AssertPointer(bg, "NULL bg");
    NEA_AssertPointer(path, "NULL path");
    NEA_Assert(bg->used, "BG not active");
    NEA_Assert(bg->type >= NEA_HW2D_BG_BITMAP_8, "Not a bitmap BG");

    size_t size = 0;
    void *buf = __NEA_FATLoadDataSize(path, &size);
    if (buf == NULL)
        return -1;

    int ret = NEA_Hw2DBGLoadBitmap(bg, buf, size);
    free(buf);
    return ret;
}

void *NEA_Hw2DBGGetBitmapPtr(const NEA_Hw2DBG *bg)
{
    NEA_AssertPointer(bg, "NULL bg");
    return bg->gfx_ptr;
}

void NEA_Hw2DBGPutPixel16(NEA_Hw2DBG *bg, int x, int y, u16 color)
{
    NEA_AssertPointer(bg, "NULL bg");
    bg->gfx_ptr[y * bg->width + x] = color;
}

void NEA_Hw2DBGPutPixel8(NEA_Hw2DBG *bg, int x, int y, u8 index)
{
    NEA_AssertPointer(bg, "NULL bg");
    // VRAM ignores 8-bit stores, so touch the halfword that holds this pixel
    // and keep its neighbour intact.
    const int offset = y * bg->width + x;
    u16 *pair = &bg->gfx_ptr[offset >> 1];
    if (offset & 1)
        *pair = (u16)((*pair & 0x00FF) | (index << 8));
    else
        *pair = (u16)((*pair & 0xFF00) | index);
}

void NEA_Hw2DBGClearBitmap(NEA_Hw2DBG *bg, u32 value)
{
    NEA_AssertPointer(bg, "NULL bg");
    NEA_Assert(bg->used, "BG not active");

    size_t total;
    if (bg->type == NEA_HW2D_BG_BITMAP_16)
        total = bg->width * bg->height * 2;
    else
        total = bg->width * bg->height;

    // Use dmaFillWords for fast VRAM fill (value replicated to 32 bits)
    u32 fill;
    if (bg->type == NEA_HW2D_BG_BITMAP_16)
        fill = (value & 0xFFFF) | ((value & 0xFFFF) << 16);
    else
        fill = (value & 0xFF) | ((value & 0xFF) << 8)
             | ((value & 0xFF) << 16) | ((value & 0xFF) << 24);

    dmaFillWords(fill, bg->gfx_ptr, total);
}

#ifdef NEA_BLOCKSDS
// Applies the data decoded from a GRF file to a BG. The decoded buffers are not
// freed by this function.
//
// Shared by NEA_Hw2DBGLoadGRFFAT() and its asynchronous version, which only
// differ in where the decoding happens.
static int ne_hw2d_bg_grf_apply(NEA_Hw2DBG *bg, const GRFHeader *header,
                                void *gfxDst, size_t gfxSize,
                                void *mapDst, size_t mapSize,
                                void *palDst, size_t palSize,
                                int palette_slot)
{
    if (gfxDst == NULL)
    {
        NEA_DebugPrint("No graphics found in GRF file");
        return -1;
    }

    switch (bg->type)
    {
        case NEA_HW2D_BG_TILED_4BPP:
            if (header->gfxAttr != 4 ||
                (header->mapAttr != GRF_BGFMT_SBB_4BPP &&
                 header->mapAttr != GRF_BGFMT_FLAT_4BPP))
            {
                NEA_DebugPrint("GRF format mismatch for tiled 4bpp BG");
                return -1;
            }
            NEA_Hw2DBGLoadTiles(bg, gfxDst, gfxSize);
            if (mapDst != NULL)
                NEA_Hw2DBGLoadMap(bg, mapDst, mapSize);
            break;

        case NEA_HW2D_BG_TILED_8BPP:
            if (header->gfxAttr != 8 ||
                (header->mapAttr != GRF_BGFMT_SBB_8BPP &&
                 header->mapAttr != GRF_BGFMT_AFF_8BPP &&
                 header->mapAttr != GRF_BGFMT_FLAT_8BPP))
            {
                NEA_DebugPrint("GRF format mismatch for tiled 8bpp BG");
                return -1;
            }
            NEA_Hw2DBGLoadTiles(bg, gfxDst, gfxSize);
            if (mapDst != NULL)
                NEA_Hw2DBGLoadMap(bg, mapDst, mapSize);
            break;

        case NEA_HW2D_BG_BITMAP_8:
            if (header->gfxAttr != 8 || header->mapAttr != GRF_BGFMT_NO_DATA)
            {
                NEA_DebugPrint("GRF format mismatch for 8bpp bitmap BG");
                return -1;
            }
            NEA_Hw2DBGLoadBitmap(bg, gfxDst, gfxSize);
            break;

        case NEA_HW2D_BG_BITMAP_16:
            if (header->gfxAttr != 16 || header->mapAttr != GRF_BGFMT_NO_DATA)
            {
                NEA_DebugPrint("GRF format mismatch for 16bpp bitmap BG");
                return -1;
            }
            NEA_Hw2DBGLoadBitmap(bg, gfxDst, gfxSize);
            break;

        default:
            NEA_DebugPrint("Unknown BG type");
            return -1;
    }

    if (palDst != NULL)
        NEA_Hw2DBGLoadPalette(bg, palDst, palSize / 2, palette_slot);

    return 0;
}
#endif // NEA_BLOCKSDS

int NEA_Hw2DBGLoadGRFFAT(NEA_Hw2DBG *bg, const char *path, int palette_slot)
{
#ifndef NEA_BLOCKSDS
    (void)bg;
    (void)path;
    (void)palette_slot;
    NEA_DebugPrint("%s only supported in BlocksDS", __func__);
    return -1;
#else
    NEA_AssertPointer(bg, "NULL bg");
    NEA_AssertPointer(path, "NULL path");
    NEA_Assert(bg->used, "BG not active");

    void *gfxDst = NULL;
    void *mapDst = NULL;
    void *palDst = NULL;
    size_t gfxSize = 0;
    size_t mapSize = 0;
    size_t palSize = 0;

    GRFHeader header = { 0 };
    GRFError err = grfLoadPath(path, &header, &gfxDst, &gfxSize,
                               &mapDst, &mapSize, &palDst, &palSize);
    if (err != GRF_NO_ERROR)
    {
        NEA_DebugPrint("Couldn't load GRF file: %d", err);
        free(gfxDst);
        free(mapDst);
        free(palDst);
        return -1;
    }

    int ret = ne_hw2d_bg_grf_apply(bg, &header, gfxDst, gfxSize,
                                   mapDst, mapSize, palDst, palSize,
                                   palette_slot);

    free(gfxDst);
    free(mapDst);
    free(palDst);
    return ret;
#endif
}

// ---------------------------------------------------------------------------
// Asynchronous BG loading
// ---------------------------------------------------------------------------

// What a plain (non-GRF) asynchronous BG load has to do with the data once it
// is in RAM.
typedef enum {
    NE_ASYNC_BG_TILES,
    NE_ASYNC_BG_MAP,
    NE_ASYNC_BG_BITMAP
} ne_async_bg_kind;

// Parameters of an asynchronous BG load.
typedef struct {
    NEA_Hw2DBG *bg;
    ne_async_bg_kind kind;
} ne_async_bg_param;

// Runs on the main thread during the vertical blank: copies the loaded data to
// VRAM with the regular (synchronous) loader.
static void ne_async_bg_finalize(NEA_AsyncFile *job)
{
    ne_async_bg_param *p = __NEA_AsyncParam(job);

    size_t size = 0;
    char *buffer = __NEA_AsyncBuffer(job, &size);

    int ret;
    switch (p->kind)
    {
        case NE_ASYNC_BG_TILES:
            ret = NEA_Hw2DBGLoadTiles(p->bg, buffer, size);
            break;
        case NE_ASYNC_BG_MAP:
            ret = NEA_Hw2DBGLoadMap(p->bg, buffer, size);
            break;
        default:
            ret = NEA_Hw2DBGLoadBitmap(p->bg, buffer, size);
            break;
    }

    // The Hw2D loaders return 0 on success, the async system wants 1.
    __NEA_AsyncSetResult(job, ret == 0);
}

// Queues one of the three plain BG loads. They only differ in the loader the
// finalize step calls.
static NEA_AsyncFile *ne_async_bg_queue(NEA_Hw2DBG *bg, const char *path,
                                        ne_async_bg_kind kind)
{
    ne_async_bg_param *p = malloc(sizeof(ne_async_bg_param));
    if (p == NULL)
    {
        NEA_DebugPrint("Out of memory");
        return NULL;
    }

    p->bg = bg;
    p->kind = kind;

    NEA_AsyncFile *job = __NEA_AsyncQueue(path, NULL, ne_async_bg_finalize,
                                          NULL, p, bg);
    if (job == NULL)
        free(p);

    return job;
}

NEA_AsyncFile *NEA_Hw2DBGLoadTilesFATAsync(NEA_Hw2DBG *bg, const char *path)
{
    NEA_AssertPointer(bg, "NULL bg");
    NEA_AssertPointer(path, "NULL path");
    NEA_Assert(bg->used, "BG not active");
    NEA_Assert(bg->type <= NEA_HW2D_BG_TILED_8BPP, "Not a tiled BG");

    return ne_async_bg_queue(bg, path, NE_ASYNC_BG_TILES);
}

NEA_AsyncFile *NEA_Hw2DBGLoadMapFATAsync(NEA_Hw2DBG *bg, const char *path)
{
    NEA_AssertPointer(bg, "NULL bg");
    NEA_AssertPointer(path, "NULL path");
    NEA_Assert(bg->used, "BG not active");
    NEA_Assert(bg->map_ptr != NULL, "Not a tiled BG");

    return ne_async_bg_queue(bg, path, NE_ASYNC_BG_MAP);
}

NEA_AsyncFile *NEA_Hw2DBGLoadBitmapFATAsync(NEA_Hw2DBG *bg, const char *path)
{
    NEA_AssertPointer(bg, "NULL bg");
    NEA_AssertPointer(path, "NULL path");
    NEA_Assert(bg->used, "BG not active");
    NEA_Assert(bg->type >= NEA_HW2D_BG_BITMAP_8, "Not a bitmap BG");

    return ne_async_bg_queue(bg, path, NE_ASYNC_BG_BITMAP);
}

// ---------------------------------------------------------------------------
// Phase 4: OBJ sprites
// ---------------------------------------------------------------------------

// Map NEA_OBJSize to libnds SpriteSize
static SpriteSize ne_hw2d_obj_size(NEA_OBJSize s)
{
    static const SpriteSize map[] = {
        SpriteSize_8x8,   SpriteSize_16x16, SpriteSize_32x32, SpriteSize_64x64,
        SpriteSize_16x8,  SpriteSize_32x8,  SpriteSize_32x16, SpriteSize_64x32,
        SpriteSize_8x16,  SpriteSize_8x32,  SpriteSize_16x32, SpriteSize_32x64
    };
    return map[s];
}

// Map NEA_OBJColorMode to libnds SpriteColorFormat
static SpriteColorFormat ne_hw2d_obj_color(NEA_OBJColorMode c)
{
    return (c == NEA_OBJ_COLOR_256)
         ? SpriteColorFormat_256Color
         : SpriteColorFormat_16Color;
}

// Get sprite pixel width from NEA_OBJSize
static int ne_hw2d_obj_width(NEA_OBJSize s)
{
    static const int w[] = { 8, 16, 32, 64, 16, 32, 32, 64, 8, 8, 16, 32 };
    return w[s];
}

// Get sprite pixel height from NEA_OBJSize
static int ne_hw2d_obj_height(NEA_OBJSize s)
{
    static const int h[] = { 8, 16, 32, 64, 8, 8, 16, 32, 16, 32, 32, 64 };
    return h[s];
}

NEA_Hw2DOBJ *NEA_Hw2DOBJCreate(NEA_Hw2DEngine engine, NEA_OBJSize size,
                                NEA_OBJColorMode mode)
{
    NEA_Assert(ne_hw2d_state.initialized, "Hw2D not init");

    int *next;
    NEA_Hw2DOBJ *objs;
    OamState *oam;

    if (engine == NEA_ENGINE_MAIN)
    {
        NEA_Assert(ne_hw2d_state.main_obj_inited, "Main OBJ not configured");
        next = &ne_hw2d_state.next_oam_main;
        objs = ne_hw2d_state.objs_main;
        oam = &oamMain;
    }
    else
    {
        NEA_Assert(ne_hw2d_state.sub_obj_inited, "Sub OBJ not configured");
        next = &ne_hw2d_state.next_oam_sub;
        objs = ne_hw2d_state.objs_sub;
        oam = &oamSub;
    }

    if (*next >= NEA_HW2D_MAX_OAM)
    {
        NEA_DebugPrint("NEA_Hw2DOBJCreate: OAM full");
        return NULL;
    }

    int idx = *next;
    (*next)++;

    ObjSize libnds_size = ne_hw2d_obj_size(size);
    SpriteColorFormat libnds_color = ne_hw2d_obj_color(mode);

    u16 *gfx = oamAllocateGfx(oam, libnds_size, libnds_color);
    if (gfx == NULL)
    {
        NEA_DebugPrint("NEA_Hw2DOBJCreate: gfx alloc failed");
        (*next)--;
        return NULL;
    }

    // Compute bytes per frame
    int w = ne_hw2d_obj_width(size);
    int h = ne_hw2d_obj_height(size);
    int bpp = (mode == NEA_OBJ_COLOR_256) ? 8 : 4;
    int frame_size = (w * h * bpp) / 8;

    NEA_Hw2DOBJ *obj = &objs[idx];
    memset(obj, 0, sizeof(*obj));
    obj->used = true;
    obj->engine = engine;
    obj->oam_index = idx;
    obj->nea_size = size;
    obj->color = mode;
    obj->gfx = gfx;
    obj->gfx_size = frame_size;
    obj->visible = false;
    obj->priority = 0;
    obj->palette_slot = 0;
    obj->affine_index = -1;
    obj->double_size = false;

    return obj;
}

void NEA_Hw2DOBJDelete(NEA_Hw2DOBJ *obj)
{
    if (obj == NULL || !obj->used)
        return;

    __NEA_AsyncCancelTarget(obj);

    OamState *oam = (obj->engine == NEA_ENGINE_MAIN) ? &oamMain : &oamSub;

    // Shared-asset OBJs don't own their gfx — just release the ref count.
    // Standalone OBJs free the gfx they allocated in NEA_Hw2DOBJCreate().
    if (obj->asset)
        obj->asset->ref_count--;
    else if (obj->gfx)
        oamFreeGfx(oam, obj->gfx);

    // Hide in OAM
    oamSet(oam, obj->oam_index, 0, 0, 0, 0,
           ne_hw2d_obj_size(obj->nea_size),
           ne_hw2d_obj_color(obj->color),
           NULL, -1, false, true, false, false, false);

    memset(obj, 0, sizeof(*obj));
}

int NEA_Hw2DOBJLoadGfx(NEA_Hw2DOBJ *obj, const void *data, size_t size)
{
    NEA_AssertPointer(obj, "NULL obj");
    NEA_AssertPointer(data, "NULL data");
    NEA_Assert(obj->used, "OBJ not active");
    NEA_Assert(obj->gfx != NULL, "No gfx allocated");

    // Copy first frame. Never read more than the source provides, so a
    // too-small buffer can't over-read past the source allocation.
    size_t copy = (size < (size_t)obj->gfx_size) ? size : (size_t)obj->gfx_size;
    ne_vram_copy(obj->gfx, data, copy);

    obj->num_frames = size / obj->gfx_size;
    if (obj->num_frames < 1)
        obj->num_frames = 1;

    return 0;
}

int NEA_Hw2DOBJLoadGfxFAT(NEA_Hw2DOBJ *obj, const char *path)
{
    NEA_AssertPointer(obj, "NULL obj");
    NEA_AssertPointer(path, "NULL path");
    NEA_Assert(obj->used, "OBJ not active");

    size_t size = 0;
    void *buf = __NEA_FATLoadDataSize(path, &size);
    if (buf == NULL)
        return -1;

    int ret = NEA_Hw2DOBJLoadGfx(obj, buf, size);
    free(buf);
    return ret;
}

int NEA_Hw2DOBJLoadPalette(NEA_Hw2DEngine engine, const void *data,
                            int num_colors, int slot)
{
    NEA_AssertPointer(data, "NULL data");

    u16 *pal = (engine == NEA_ENGINE_MAIN)
             ? SPRITE_PALETTE : SPRITE_PALETTE_SUB;

    // The OBJ palette region holds 256 colors. Clamp the copy so a palette
    // loaded at a non-zero slot can't run past the end of the region into
    // adjacent palette RAM (GRF/ptexconv files often pad 4bpp palettes out
    // to 256 entries, which would otherwise overflow for any slot > 0).
    int max_colors = 256 - slot * 16;
    if (max_colors < 0)
        max_colors = 0;
    if (num_colors > max_colors)
        num_colors = max_colors;

    pal += slot * 16;
    ne_vram_copy(pal, data, num_colors * 2);
    return 0;
}

int NEA_Hw2DOBJLoadPaletteFAT(NEA_Hw2DEngine engine, const char *path, int slot)
{
    NEA_AssertPointer(path, "NULL path");

    size_t size = 0;
    void *buf = __NEA_FATLoadDataSize(path, &size);
    if (buf == NULL)
        return -1;

    // RGB15 palettes are 2 bytes per color.
    int ret = NEA_Hw2DOBJLoadPalette(engine, buf, size / 2, slot);

    free(buf);
    return ret;
}

#ifdef NEA_BLOCKSDS
// Applies the data decoded from a GRF file to an OBJ sprite. The decoded buffers
// are not freed by this function.
//
// Shared by NEA_Hw2DOBJLoadGRFFAT() and its asynchronous version.
static int ne_hw2d_obj_grf_apply(NEA_Hw2DOBJ *obj, const GRFHeader *header,
                                 void *gfxDst, size_t gfxSize,
                                 void *palDst, size_t palSize,
                                 int palette_slot)
{
    if (gfxDst == NULL)
    {
        NEA_DebugPrint("No graphics found in GRF file");
        return -1;
    }

    int expected_bpp = (obj->color == NEA_OBJ_COLOR_16) ? 4 : 8;
    if (header->gfxAttr != expected_bpp)
    {
        NEA_DebugPrint("GRF color depth mismatch for OBJ sprite");
        return -1;
    }

    // The OBJ's hardware size is fixed at creation; the GRF must match it.
    // Without this check a mismatch silently over-reads the GRF buffer (decl
    // larger) or loads a cropped image (decl smaller), which renders as a
    // wrong-size / wrong-ratio sprite.
    if (oamDimensionsToSize(header->gfxWidth, header->gfxHeight)
        != ne_hw2d_obj_size(obj->nea_size))
    {
        NEA_DebugPrint("OBJ GRF size mismatch: file is %dx%d",
                       (int)header->gfxWidth, (int)header->gfxHeight);
        return -1;
    }

    NEA_Hw2DOBJLoadGfx(obj, gfxDst, gfxSize);

    if (palDst != NULL)
    {
        int num_colors = palSize / 2;

        if (obj->color == NEA_OBJ_COLOR_16)
        {
            // A 16-color sprite occupies exactly one 16-entry palette bank.
            // GRF converters (grit, ptexconv) pad a 4bpp palette out to 256
            // entries; copying all of them would overwrite the banks belonging
            // to other 16-color sprites. Only the first 16 colors are ours.
            if (num_colors > 16)
                num_colors = 16;

            // Point the OAM palette-number field at the bank we just loaded.
            // Without this the sprite keeps its create-time bank (0) and
            // renders with the wrong colors.
            obj->palette_slot = palette_slot;
        }

        NEA_Hw2DOBJLoadPalette(obj->engine, palDst, num_colors, palette_slot);
    }

    return 0;
}
#endif // NEA_BLOCKSDS

int NEA_Hw2DOBJLoadGRFFAT(NEA_Hw2DOBJ *obj, const char *path, int palette_slot)
{
#ifndef NEA_BLOCKSDS
    (void)obj;
    (void)path;
    (void)palette_slot;
    NEA_DebugPrint("%s only supported in BlocksDS", __func__);
    return -1;
#else
    NEA_AssertPointer(obj, "NULL obj");
    NEA_AssertPointer(path, "NULL path");
    NEA_Assert(obj->used, "OBJ not active");

    void *gfxDst = NULL;
    void *palDst = NULL;
    size_t gfxSize = 0;
    size_t palSize = 0;

    GRFHeader header = { 0 };
    GRFError err = grfLoadPath(path, &header, &gfxDst, &gfxSize,
                               NULL, NULL, &palDst, &palSize);
    if (err != GRF_NO_ERROR)
    {
        NEA_DebugPrint("Couldn't load GRF file: %d", err);
        free(gfxDst);
        free(palDst);
        return -1;
    }

    int ret = ne_hw2d_obj_grf_apply(obj, &header, gfxDst, gfxSize,
                                    palDst, palSize, palette_slot);

    free(gfxDst);
    free(palDst);
    return ret;
#endif
}

void NEA_Hw2DOBJSetPos(NEA_Hw2DOBJ *obj, int x, int y)
{
    NEA_AssertPointer(obj, "NULL obj");
    obj->x = x;
    obj->y = y;
}

void NEA_Hw2DOBJSetVisible(NEA_Hw2DOBJ *obj, bool visible)
{
    NEA_AssertPointer(obj, "NULL obj");
    obj->visible = visible;
}

void NEA_Hw2DOBJSetFlip(NEA_Hw2DOBJ *obj, bool hflip, bool vflip)
{
    NEA_AssertPointer(obj, "NULL obj");
    obj->hflip = hflip;
    obj->vflip = vflip;
}

void NEA_Hw2DOBJSetPriority(NEA_Hw2DOBJ *obj, int priority)
{
    NEA_AssertPointer(obj, "NULL obj");
    obj->priority = priority;
}

void NEA_Hw2DOBJSetPaletteSlot(NEA_Hw2DOBJ *obj, int slot)
{
    NEA_AssertPointer(obj, "NULL obj");

    // Only 16-color sprites have a palette bank; 256-color sprites use the
    // whole OBJ palette region and the OAM palette-number field is unused.
    if (obj->color != NEA_OBJ_COLOR_16)
        return;

    NEA_Assert(slot >= 0 && slot < 16, "OBJ palette slot out of range");
    obj->palette_slot = slot;
}

void NEA_Hw2DOBJSetFrame(NEA_Hw2DOBJ *obj, int frame)
{
    NEA_AssertPointer(obj, "NULL obj");
    NEA_Assert(obj->used, "OBJ not active");

    if (frame >= obj->num_frames)
        frame = 0;

    obj->frame = frame;
}

void NEA_Hw2DOBJSetAffine(NEA_Hw2DOBJ *obj, int rot_index, bool double_size)
{
    NEA_AssertPointer(obj, "NULL obj");
    obj->affine_index = rot_index;
    obj->double_size = double_size;
}

void NEA_Hw2DOBJSetRotScaleI(NEA_Hw2DEngine engine, int rot_index,
                              int angle, int32_t sx, int32_t sy)
{
    OamState *oam = (engine == NEA_ENGINE_MAIN) ? &oamMain : &oamSub;

    // Translate NEA's units into libnds's. A full turn is 512 here and
    // 1 << 15 there, and oamRotateScale() wants the *inverse* scale in 1.7.8:
    // the affine entry maps screen pixels back to texture pixels, so making a
    // sprite twice as big means halving the step through its graphics.
    int lnds_angle = (angle << 6) & 0xFFFF;
    if (lnds_angle > 32767)
        lnds_angle -= 65536;

    int inv_sx = (sx != 0) ? (int)(((int64_t)256 * 4096) / sx) : (256 << 8);
    int inv_sy = (sy != 0) ? (int)(((int64_t)256 * 4096) / sy) : (256 << 8);

    oamRotateScale(oam, rot_index, lnds_angle, inv_sx, inv_sy);
}

void NEA_Hw2DOBJSetAffineMatrix(NEA_Hw2DEngine engine, int rot_index,
                                 int hdx, int hdy, int vdx, int vdy)
{
    NEA_Assert(rot_index >= 0 && rot_index < 32, "Affine index out of range");

    OamState *oam = (engine == NEA_ENGINE_MAIN) ? &oamMain : &oamSub;
    oamAffineTransformation(oam, rot_index, hdx, hdy, vdx, vdy);
}

void NEA_Hw2DOBJUpdate(NEA_Hw2DEngine engine)
{
    NEA_Hw2DOBJ *objs;
    OamState *oam;
    int count;

    if (engine == NEA_ENGINE_MAIN)
    {
        if (!ne_hw2d_state.main_obj_inited)
            return;
        objs = ne_hw2d_state.objs_main;
        oam = &oamMain;
        count = ne_hw2d_state.next_oam_main;
    }
    else
    {
        if (!ne_hw2d_state.sub_obj_inited)
            return;
        objs = ne_hw2d_state.objs_sub;
        oam = &oamSub;
        count = ne_hw2d_state.next_oam_sub;
    }

    for (int i = 0; i < count; i++)
    {
        NEA_Hw2DOBJ *o = &objs[i];
        if (!o->used)
            continue;

        // Step to the current frame. gfx_size is bytes and gfx is a u16 *,
        // hence the halving. Without this NEA_Hw2DOBJSetFrame() stored a
        // number that nothing ever read, and a flipbook never advanced.
        u16 *gfx = o->gfx;
        if (gfx != NULL && o->frame > 0 && o->gfx_size > 0)
            gfx += ((size_t)o->frame * (size_t)o->gfx_size) / 2;

        oamSet(oam, o->oam_index,
               o->x, o->y,
               o->priority,
               o->palette_slot,
               ne_hw2d_obj_size(o->nea_size),
               ne_hw2d_obj_color(o->color),
               gfx,
               o->affine_index,
               o->double_size,
               !o->visible,  // oamSet: hide = true to hide
               o->hflip, o->vflip,
               false);       // mosaic
    }

    oamUpdate(oam);
}

void NEA_Hw2DOBJUpdateAll(void)
{
    if (!ne_hw2d_state.initialized)
        return;

    NEA_Hw2DOBJUpdate(NEA_ENGINE_MAIN);
    NEA_Hw2DOBJUpdate(NEA_ENGINE_SUB);
}

// ---------------------------------------------------------------------------
// Phase 4b: Shared OBJ assets (de-duplicated gfx + palette slot)
// ---------------------------------------------------------------------------
//
// An asset is a single (gfx, palette_slot) pair that many OBJs can share.
// Compared to NEA_Hw2DOBJCreate(), which calls oamAllocateGfx() per sprite
// and would copy the same tile data N times for N identical enemies, the
// asset path allocates the gfx block once and hands out OBJs that point at
// it. This avoids RAM duplication and reduces churn in the OAM gfx
// allocator (which can fragment under repeated create/delete cycles).
//
// Palette slot allocation for 16-color assets uses the obj_pal_used_*
// bitmaps to find a free slot the first time a palette is loaded.

// First-fit allocation of a 16-color OBJ palette slot for the engine.
// Returns the slot index (0-15) or -1 if all are in use.
static int ne_alloc_obj_pal_slot(NEA_Hw2DEngine engine)
{
    u16 *used = (engine == NEA_ENGINE_MAIN)
              ? &ne_hw2d_state.obj_pal_used_main
              : &ne_hw2d_state.obj_pal_used_sub;
    for (int i = 0; i < 16; i++)
    {
        if (!(*used & (1u << i)))
        {
            *used |= (1u << i);
            return i;
        }
    }
    return -1;
}

static void ne_free_obj_pal_slot(NEA_Hw2DEngine engine, int slot)
{
    if (slot < 0 || slot >= 16)
        return;
    u16 *used = (engine == NEA_ENGINE_MAIN)
              ? &ne_hw2d_state.obj_pal_used_main
              : &ne_hw2d_state.obj_pal_used_sub;
    *used &= ~(1u << slot);
}

// Re-point every OBJ bound to this asset at the asset's current palette slot.
// Needed because OBJs can be bound before the asset's palette (and thus its
// auto-allocated 16-color slot) exists; without this their OAM palette-number
// field keeps the create-time default (0) and the sprite renders with the
// wrong palette bank.
static void ne_sync_asset_palette_slot(NEA_Hw2DOBJAsset *asset)
{
    if (asset->palette_slot < 0)
        return;

    NEA_Hw2DOBJ *objs = (asset->engine == NEA_ENGINE_MAIN)
                         ? ne_hw2d_state.objs_main
                         : ne_hw2d_state.objs_sub;
    int count = (asset->engine == NEA_ENGINE_MAIN)
                 ? ne_hw2d_state.next_oam_main
                 : ne_hw2d_state.next_oam_sub;

    for (int i = 0; i < count; i++)
    {
        if (objs[i].used && objs[i].asset == asset)
            objs[i].palette_slot = asset->palette_slot;
    }
}

// Find a free asset slot in the per-engine pool.
static NEA_Hw2DOBJAsset *ne_alloc_asset_slot(NEA_Hw2DEngine engine)
{
    NEA_Hw2DOBJAsset *pool = (engine == NEA_ENGINE_MAIN)
                              ? ne_hw2d_state.assets_main
                              : ne_hw2d_state.assets_sub;
    for (int i = 0; i < NEA_HW2D_MAX_ASSETS; i++)
    {
        if (!pool[i].used)
            return &pool[i];
    }
    return NULL;
}

NEA_Hw2DOBJAsset *NEA_Hw2DOBJAssetCreate(NEA_Hw2DEngine engine,
                                          NEA_OBJSize size,
                                          NEA_OBJColorMode mode)
{
    NEA_Assert(ne_hw2d_state.initialized, "Hw2D not init");

    bool obj_inited = (engine == NEA_ENGINE_MAIN)
                       ? ne_hw2d_state.main_obj_inited
                       : ne_hw2d_state.sub_obj_inited;
    if (!obj_inited)
    {
        NEA_DebugPrint("NEA_Hw2DOBJAssetCreate: OBJ engine not configured");
        return NULL;
    }

    NEA_Hw2DOBJAsset *asset = ne_alloc_asset_slot(engine);
    if (asset == NULL)
    {
        NEA_DebugPrint("NEA_Hw2DOBJAssetCreate: asset pool full");
        return NULL;
    }

    OamState *oam = (engine == NEA_ENGINE_MAIN) ? &oamMain : &oamSub;
    SpriteSize libnds_size = ne_hw2d_obj_size(size);
    SpriteColorFormat libnds_color = ne_hw2d_obj_color(mode);

    u16 *gfx = oamAllocateGfx(oam, libnds_size, libnds_color);
    if (gfx == NULL)
    {
        NEA_DebugPrint("NEA_Hw2DOBJAssetCreate: gfx alloc failed");
        return NULL;
    }

    int w = ne_hw2d_obj_width(size);
    int h = ne_hw2d_obj_height(size);
    int bpp = (mode == NEA_OBJ_COLOR_256) ? 8 : 4;

    memset(asset, 0, sizeof(*asset));
    asset->used = true;
    asset->engine = engine;
    asset->size = size;
    asset->color = mode;
    asset->gfx = gfx;
    asset->gfx_size = (w * h * bpp) / 8;
    asset->num_frames = 1;
    asset->palette_slot = -1;
    asset->ref_count = 0;
    return asset;
}

void NEA_Hw2DOBJAssetDelete(NEA_Hw2DOBJAsset *asset)
{
    if (asset == NULL || !asset->used)
        return;

    if (asset->ref_count > 0)
    {
        NEA_DebugPrint("NEA_Hw2DOBJAssetDelete: %d OBJ(s) still bound",
                       asset->ref_count);
        return;
    }

    __NEA_AsyncCancelTarget(asset);

    OamState *oam = (asset->engine == NEA_ENGINE_MAIN) ? &oamMain : &oamSub;
    if (asset->gfx)
        oamFreeGfx(oam, asset->gfx);

    if (asset->color == NEA_OBJ_COLOR_16 && asset->palette_slot >= 0)
        ne_free_obj_pal_slot(asset->engine, asset->palette_slot);

    memset(asset, 0, sizeof(*asset));
}

int NEA_Hw2DOBJAssetLoadGfx(NEA_Hw2DOBJAsset *asset,
                             const void *data, size_t size)
{
    NEA_AssertPointer(asset, "NULL asset");
    NEA_AssertPointer(data, "NULL data");
    NEA_Assert(asset->used, "Asset not active");

    // VRAM only holds one sprite's worth — copy that. Track total frames
    // present in the source so callers can know how much they had. Clamp the
    // copy to the source length so a too-small buffer can't over-read.
    size_t copy = (size < (size_t)asset->gfx_size) ? size
                                                   : (size_t)asset->gfx_size;
    ne_vram_copy(asset->gfx, data, copy);
    asset->num_frames = (int)(size / asset->gfx_size);
    if (asset->num_frames < 1)
        asset->num_frames = 1;
    return 0;
}

int NEA_Hw2DOBJAssetLoadGfxFAT(NEA_Hw2DOBJAsset *asset, const char *path)
{
    NEA_AssertPointer(asset, "NULL asset");
    NEA_AssertPointer(path, "NULL path");
    NEA_Assert(asset->used, "Asset not active");

    size_t size = 0;
    void *buf = __NEA_FATLoadDataSize(path, &size);
    if (buf == NULL)
        return -1;

    int ret = NEA_Hw2DOBJAssetLoadGfx(asset, buf, size);
    free(buf);
    return ret;
}

int NEA_Hw2DOBJAssetLoadPalette(NEA_Hw2DOBJAsset *asset,
                                 const void *data, int num_colors)
{
    NEA_AssertPointer(asset, "NULL asset");
    NEA_AssertPointer(data, "NULL data");
    NEA_Assert(asset->used, "Asset not active");

    int slot;
    if (asset->color == NEA_OBJ_COLOR_16)
    {
        if (asset->palette_slot < 0)
        {
            slot = ne_alloc_obj_pal_slot(asset->engine);
            if (slot < 0)
            {
                NEA_DebugPrint("Asset palette: no free 16-color slot");
                return -1;
            }
            asset->palette_slot = slot;
        }
        else
        {
            slot = asset->palette_slot;
        }

        // A 16-color sprite occupies exactly one 16-entry palette slot. GRF
        // converters (grit, ptexconv) typically pad a 4bpp palette out to 256
        // entries; copying all of them would overwrite the slots belonging to
        // other 16-color assets. Only the first 16 colors are this sprite's.
        if (num_colors > 16)
            num_colors = 16;
    }
    else
    {
        // 256-color sprites use the full OBJ palette region; slot is unused.
        slot = 0;
        asset->palette_slot = 0;
    }

    // Keep any already-bound OBJs pointing at the slot the palette landed in.
    ne_sync_asset_palette_slot(asset);

    return NEA_Hw2DOBJLoadPalette(asset->engine, data, num_colors, slot);
}

int NEA_Hw2DOBJAssetLoadPaletteFAT(NEA_Hw2DOBJAsset *asset, const char *path)
{
    NEA_AssertPointer(asset, "NULL asset");
    NEA_AssertPointer(path, "NULL path");
    NEA_Assert(asset->used, "Asset not active");

    size_t size = 0;
    void *buf = __NEA_FATLoadDataSize(path, &size);
    if (buf == NULL)
        return -1;

    // RGB15 palettes are 2 bytes per color.
    int ret = NEA_Hw2DOBJAssetLoadPalette(asset, buf, size / 2);

    free(buf);
    return ret;
}

#ifdef NEA_BLOCKSDS
// Applies the data decoded from a GRF file to a shared OBJ asset. The decoded
// buffers are not freed by this function.
//
// Shared by NEA_Hw2DOBJAssetLoadGRFFAT() and its asynchronous version.
static int ne_hw2d_asset_grf_apply(NEA_Hw2DOBJAsset *asset,
                                   const GRFHeader *header,
                                   void *gfxDst, size_t gfxSize,
                                   void *palDst, size_t palSize)
{
    if (gfxDst == NULL)
    {
        NEA_DebugPrint("No graphics found in GRF file");
        return -1;
    }

    int expected_bpp = (asset->color == NEA_OBJ_COLOR_16) ? 4 : 8;
    if (header->gfxAttr != expected_bpp)
    {
        NEA_DebugPrint("GRF color depth mismatch for OBJ asset");
        return -1;
    }

    // The asset's hardware size is fixed at creation; the GRF must match it.
    // Without this check a mismatch silently over-reads the GRF buffer (decl
    // larger) or loads a cropped image (decl smaller), which renders as a
    // wrong-size / wrong-ratio sprite.
    if (oamDimensionsToSize(header->gfxWidth, header->gfxHeight)
        != ne_hw2d_obj_size(asset->size))
    {
        NEA_DebugPrint("OBJ asset GRF size mismatch: file is %dx%d",
                       (int)header->gfxWidth, (int)header->gfxHeight);
        return -1;
    }

    NEA_Hw2DOBJAssetLoadGfx(asset, gfxDst, gfxSize);

    if (palDst != NULL)
    {
        if (NEA_Hw2DOBJAssetLoadPalette(asset, palDst, palSize / 2) != 0)
            return -1;
    }

    return 0;
}
#endif // NEA_BLOCKSDS

int NEA_Hw2DOBJAssetLoadGRFFAT(NEA_Hw2DOBJAsset *asset, const char *path)
{
#ifndef NEA_BLOCKSDS
    (void)asset;
    (void)path;
    NEA_DebugPrint("%s only supported in BlocksDS", __func__);
    return -1;
#else
    NEA_AssertPointer(asset, "NULL asset");
    NEA_AssertPointer(path, "NULL path");
    NEA_Assert(asset->used, "Asset not active");

    void *gfxDst = NULL;
    void *palDst = NULL;
    size_t gfxSize = 0;
    size_t palSize = 0;

    GRFHeader header = { 0 };
    GRFError err = grfLoadPath(path, &header, &gfxDst, &gfxSize,
                               NULL, NULL, &palDst, &palSize);
    if (err != GRF_NO_ERROR)
    {
        NEA_DebugPrint("Couldn't load GRF file: %d", err);
        free(gfxDst);
        free(palDst);
        return -1;
    }

    int ret = ne_hw2d_asset_grf_apply(asset, &header, gfxDst, gfxSize,
                                      palDst, palSize);

    free(gfxDst);
    free(palDst);
    return ret;
#endif
}

NEA_Hw2DOBJ *NEA_Hw2DOBJCreateFromAsset(NEA_Hw2DOBJAsset *asset)
{
    NEA_AssertPointer(asset, "NULL asset");
    NEA_Assert(asset->used, "Asset not active");

    int *next;
    NEA_Hw2DOBJ *objs;
    if (asset->engine == NEA_ENGINE_MAIN)
    {
        next = &ne_hw2d_state.next_oam_main;
        objs = ne_hw2d_state.objs_main;
    }
    else
    {
        next = &ne_hw2d_state.next_oam_sub;
        objs = ne_hw2d_state.objs_sub;
    }

    if (*next >= NEA_HW2D_MAX_OAM)
    {
        NEA_DebugPrint("NEA_Hw2DOBJCreateFromAsset: OAM full");
        return NULL;
    }

    int idx = (*next)++;
    NEA_Hw2DOBJ *obj = &objs[idx];
    memset(obj, 0, sizeof(*obj));
    obj->used = true;
    obj->engine = asset->engine;
    obj->oam_index = idx;
    obj->nea_size = asset->size;
    obj->color = asset->color;
    obj->gfx = asset->gfx;
    obj->gfx_size = asset->gfx_size;
    obj->num_frames = asset->num_frames;
    obj->palette_slot = (asset->palette_slot >= 0) ? asset->palette_slot : 0;
    obj->affine_index = -1;
    obj->visible = false;
    obj->asset = asset;
    asset->ref_count++;
    return obj;
}

int NEA_Hw2DOBJBindAsset(NEA_Hw2DOBJ *obj, NEA_Hw2DOBJAsset *asset)
{
    NEA_AssertPointer(obj, "NULL obj");
    NEA_AssertPointer(asset, "NULL asset");
    NEA_Assert(obj->used, "OBJ not active");
    NEA_Assert(asset->used, "Asset not active");

    if (obj->engine != asset->engine)
    {
        NEA_DebugPrint("BindAsset: engine mismatch");
        return -1;
    }
    if (obj->nea_size != asset->size || obj->color != asset->color)
    {
        NEA_DebugPrint("BindAsset: size/color mismatch");
        return -1;
    }

    // Release the OBJ's prior gfx allocation, whatever its source.
    if (obj->asset)
    {
        obj->asset->ref_count--;
    }
    else if (obj->gfx)
    {
        OamState *oam = (obj->engine == NEA_ENGINE_MAIN) ? &oamMain : &oamSub;
        oamFreeGfx(oam, obj->gfx);
    }

    obj->asset = asset;
    obj->gfx = asset->gfx;
    obj->gfx_size = asset->gfx_size;
    obj->num_frames = asset->num_frames;
    if (asset->palette_slot >= 0)
        obj->palette_slot = asset->palette_slot;
    asset->ref_count++;
    return 0;
}

void NEA_Hw2DOBJAssetSetPaletteSlot(NEA_Hw2DOBJAsset *asset, int slot)
{
    NEA_AssertPointer(asset, "NULL asset");
    NEA_Assert(asset->used, "Asset not active");

    // Only 16-color assets have a palette bank; 256-color assets use the whole
    // OBJ palette region and the OAM palette-number field is unused.
    if (asset->color != NEA_OBJ_COLOR_16)
        return;

    NEA_Assert(slot >= 0 && slot < 16, "OBJ palette slot out of range");

    if (asset->palette_slot == slot)
        return;

    // Hand bitmap ownership from the old bank to the new one so the
    // auto-allocator won't later reuse the requested bank for another asset.
    if (asset->palette_slot >= 0)
        ne_free_obj_pal_slot(asset->engine, asset->palette_slot);

    u16 *used = (asset->engine == NEA_ENGINE_MAIN)
              ? &ne_hw2d_state.obj_pal_used_main
              : &ne_hw2d_state.obj_pal_used_sub;
    *used |= (1u << slot);

    asset->palette_slot = slot;

    // Re-point every bound OBJ at the new bank.
    ne_sync_asset_palette_slot(asset);
}

// ---------------------------------------------------------------------------
// Phase 5: Text rendering on bitmap backgrounds
// ---------------------------------------------------------------------------

int NEA_Hw2DTextRender(NEA_Hw2DBG *bg, u32 slot, const char *str,
                        int x, int y)
{
    NEA_AssertPointer(bg, "NULL bg");
    NEA_AssertPointer(str, "NULL str");
    NEA_Assert(bg->used, "BG not active");

    // Pick the libdsf format from the BG type. Tiled BGs aren't supported —
    // libdsf writes linear bitmap data, not tile-rearranged glyphs.
    dsf_format bg_fmt;
    if (bg->type == NEA_HW2D_BG_BITMAP_16)
        bg_fmt = DSF_BMP_RGBA;
    else if (bg->type == NEA_HW2D_BG_BITMAP_8)
        bg_fmt = DSF_BMP_RGB256;
    else
    {
        NEA_DebugPrint("NEA_Hw2DTextRender: needs a bitmap BG");
        return -1;
    }

    uintptr_t handle;
    const void *font_texture;
    size_t font_w, font_h;
    unsigned int font_fmt;

    if (NEA_RichTextGetBitmapState(slot, &handle, &font_texture,
                                    &font_w, &font_h, &font_fmt) != 0)
    {
        NEA_DebugPrint("NEA_Hw2DTextRender: invalid rich text slot");
        return -1;
    }

    // libdsf processes the font texture and the destination buffer with the
    // same `texture_fmt`, so the font format must match the BG format.
    // NEA_TextureFormat values are identical to dsf_format values by design.
    if ((dsf_format)font_fmt != bg_fmt)
    {
        NEA_DebugPrint("NEA_Hw2DTextRender: font/BG format mismatch "
                       "(font=%u, bg_fmt=%d)", font_fmt, bg_fmt);
        return -1;
    }

    // Render straight into VRAM. libdsf 0.2 skips transparent source pixels
    // internally (alpha-0 for RGBA, palette-index-0 for RGB256), so this
    // composites over existing BG content with no engine-side loop.
    dsf_error err = DSF_StringRenderToExistingBuffer((dsf_handle)handle,
                            str, bg_fmt, font_texture, font_w, font_h,
                            bg->gfx_ptr, bg->width, bg->height,
                            (uint32_t)x, (uint32_t)y);
    if (err != DSF_NO_ERROR)
    {
        NEA_DebugPrint("NEA_Hw2DTextRender: render failed (%d)", err);
        return -1;
    }
    return 0;
}

// ---------------------------------------------------------------------------
// Phase 5b: Persistent text context (canvas / cursor / typewriter)
// ---------------------------------------------------------------------------

// Map a dsf_error from the renderer-driving calls onto the NEA convention:
// 0 = success, -1 = real error, +1 = canvas filled up (normal "done" for
// typewriter loops, not a failure).
static int ne_hw2d_text_map_err(dsf_error err, const char *what)
{
    if (err == DSF_NO_ERROR)
        return 0;
    if (err == DSF_CANVAS_FULL)
        return 1;
    NEA_DebugPrint("NEA_Hw2DTextCtx %s: %d", what, err);
    return -1;
}

// Map a bitmap BG type onto the libdsf format that writes it. Returns -1 for
// tiled types, which libdsf can't target (it lays down linear pixels).
static int ne_hw2d_text_fmt(NEA_Hw2DBGType type, dsf_format *out)
{
    if (type == NEA_HW2D_BG_BITMAP_16)
        *out = DSF_BMP_RGBA;
    else if (type == NEA_HW2D_BG_BITMAP_8)
        *out = DSF_BMP_RGB256;
    else
        return -1;
    return 0;
}

// Claim a free ctx slot and point a fresh libdsf renderer at `dst`. Shared by
// the BG-backed and buffer-backed constructors; the caller fills in whichever
// destination field applies.
static NEA_Hw2DTextCtx *ne_hw2d_text_ctx_new(dsf_format fmt, void *dst,
                                              int width, int height,
                                              const char *what)
{
    NEA_Hw2DTextCtx *ctx = NULL;
    for (int i = 0; i < NEA_HW2D_MAX_TEXT_CTX; i++)
    {
        if (!ne_hw2d_state.text_ctx[i].used)
        {
            ctx = &ne_hw2d_state.text_ctx[i];
            break;
        }
    }
    if (ctx == NULL)
    {
        NEA_DebugPrint("%s: text ctx pool full", what);
        return NULL;
    }

    dsf_renderer renderer;
    dsf_error err = DSF_RendererNew(&renderer);
    if (err != DSF_NO_ERROR)
    {
        NEA_DebugPrint("%s: RendererNew %d", what, err);
        return NULL;
    }

    err = DSF_RendererModeSetBuffer(renderer, fmt, dst, width, height);
    if (err != DSF_NO_ERROR)
    {
        NEA_DebugPrint("%s: ModeSetBuffer %d", what, err);
        DSF_RendererFree(renderer);
        return NULL;
    }

    memset(ctx, 0, sizeof(*ctx));
    ctx->used = true;
    ctx->renderer = renderer;
    ctx->fmt = fmt;
    // Default canvas = the whole destination. SetCanvas overrides.
    ctx->canvas_right = width;
    ctx->canvas_bottom = height;
    // font_handle / bitmap_set stay zero — caller loads them next.
    return ctx;
}

NEA_Hw2DTextCtx *NEA_Hw2DTextCtxCreate(NEA_Hw2DBG *bg)
{
    NEA_AssertPointer(bg, "NULL bg");
    NEA_Assert(bg->used, "BG not active");

    dsf_format fmt;
    if (ne_hw2d_text_fmt(bg->type, &fmt) != 0)
    {
        NEA_DebugPrint("NEA_Hw2DTextCtxCreate: needs a bitmap BG");
        return NULL;
    }

    NEA_Hw2DTextCtx *ctx = ne_hw2d_text_ctx_new(fmt, bg->gfx_ptr, bg->width,
                                                 bg->height,
                                                 "NEA_Hw2DTextCtxCreate");
    if (ctx == NULL)
        return NULL;

    ctx->bg = bg;
    return ctx;
}

NEA_Hw2DTextCtx *NEA_Hw2DTextCtxCreateBuffer(NEA_Hw2DBGType type, void *buffer,
                                              int width, int height)
{
    NEA_AssertPointer(buffer, "NULL buffer");

    dsf_format fmt;
    if (ne_hw2d_text_fmt(type, &fmt) != 0)
    {
        NEA_DebugPrint("NEA_Hw2DTextCtxCreateBuffer: needs a bitmap type");
        return NULL;
    }
    if (width <= 0 || height <= 0)
    {
        NEA_DebugPrint("NEA_Hw2DTextCtxCreateBuffer: bad size %dx%d",
                       width, height);
        return NULL;
    }

    NEA_Hw2DTextCtx *ctx = ne_hw2d_text_ctx_new(fmt, buffer, width, height,
                                                 "NEA_Hw2DTextCtxCreateBuffer");
    if (ctx == NULL)
        return NULL;

    ctx->buffer = buffer;
    ctx->buf_width = width;
    ctx->buf_height = height;
    return ctx;
}

void NEA_Hw2DTextCtxDelete(NEA_Hw2DTextCtx *ctx)
{
    if (ctx == NULL || !ctx->used)
        return;

    __NEA_AsyncCancelTarget(ctx);

    DSF_RendererFree(ctx->renderer);

    if (ctx->owns_font_handle && ctx->font_handle != 0)
        DSF_FreeFont(&ctx->font_handle);

    free(ctx->owned_bitmap);
    free(ctx->owned_palette);

    memset(ctx, 0, sizeof(*ctx));
}

// ---- Font metadata loading ----

int NEA_Hw2DTextCtxMetadataLoadMemory(NEA_Hw2DTextCtx *ctx,
                                       const void *data, size_t size)
{
    NEA_AssertPointer(ctx, "NULL ctx");
    NEA_AssertPointer(data, "NULL data");
    NEA_Assert(ctx->used, "ctx not active");

    // Replace any existing font owned by this ctx.
    if (ctx->owns_font_handle && ctx->font_handle != 0)
        DSF_FreeFont(&ctx->font_handle);

    dsf_handle handle;
    dsf_error err = DSF_LoadFontMemory(&handle, data, (int32_t)size);
    if (err != DSF_NO_ERROR)
    {
        NEA_DebugPrint("NEA_Hw2DTextCtxMetadataLoadMemory: %d", err);
        ctx->font_handle = 0;
        ctx->owns_font_handle = false;
        return -1;
    }
    ctx->font_handle = handle;
    ctx->owns_font_handle = true;
    return 0;
}

int NEA_Hw2DTextCtxMetadataLoadFAT(NEA_Hw2DTextCtx *ctx, const char *path)
{
    NEA_AssertPointer(ctx, "NULL ctx");
    NEA_AssertPointer(path, "NULL path");
    NEA_Assert(ctx->used, "ctx not active");

    if (ctx->owns_font_handle && ctx->font_handle != 0)
        DSF_FreeFont(&ctx->font_handle);

    dsf_handle handle;
    dsf_error err = DSF_LoadFontFilesystem(&handle, path);
    if (err != DSF_NO_ERROR)
    {
        NEA_DebugPrint("NEA_Hw2DTextCtxMetadataLoadFAT: %d", err);
        ctx->font_handle = 0;
        ctx->owns_font_handle = false;
        return -1;
    }
    ctx->font_handle = handle;
    ctx->owns_font_handle = true;
    return 0;
}

// ---- Font bitmap loading ----

// Copy a font palette into the BG palette region appropriate for the engine.
// For RGB256 (8bpp) BGs only — 16bpp BGs ignore the palette entirely.
static void ne_hw2d_text_copy_pal(NEA_Hw2DTextCtx *ctx,
                                   const void *pal, size_t bytes)
{
    if (pal == NULL || bytes == 0 || ctx->fmt != DSF_BMP_RGB256)
        return;
    u16 *dst = (ctx->bg->engine == NEA_ENGINE_MAIN)
             ? BG_PALETTE : BG_PALETTE_SUB;
    memcpy(dst, pal, bytes);
}

int NEA_Hw2DTextCtxBitmapSet(NEA_Hw2DTextCtx *ctx,
                              const void *texture, size_t width, size_t height,
                              const void *palette, size_t palette_bytes)
{
    NEA_AssertPointer(ctx, "NULL ctx");
    NEA_AssertPointer(texture, "NULL texture");
    NEA_Assert(ctx->used, "ctx not active");

    dsf_error err = DSF_FontTextureSet(ctx->font_handle, texture,
                                        width, height, ctx->fmt);
    if (err != DSF_NO_ERROR)
    {
        NEA_DebugPrint("NEA_Hw2DTextCtxBitmapSet: FontTextureSet %d", err);
        return -1;
    }

    ne_hw2d_text_copy_pal(ctx, palette, palette_bytes);
    ctx->bitmap_set = true;
    return 0;
}

#ifdef NEA_BLOCKSDS
// Applies the data decoded from a GRF file to a text context as its font bitmap.
//
// On success the context takes ownership of both decoded buffers, and they are
// cleared in the caller's variables so that its cleanup leaves them alone. On
// failure they are left untouched and the caller still owns them.
//
// Shared by NEA_Hw2DTextCtxBitmapLoadGRF() and its asynchronous version.
static int ne_hw2d_textctx_grf_apply(NEA_Hw2DTextCtx *ctx,
                                     const GRFHeader *header,
                                     void **gfxDst,
                                     void **palDst, size_t palSize)
{
    if (*gfxDst == NULL)
    {
        NEA_DebugPrint("NEA_Hw2DTextCtxBitmapLoadGRF: no gfx in GRF");
        return -1;
    }

    // Validate the GRF's color depth against the ctx's BG format.
    bool ok = (ctx->fmt == DSF_BMP_RGBA   && header->gfxAttr == 16)
           || (ctx->fmt == DSF_BMP_RGB256 && header->gfxAttr == 8);
    if (!ok)
    {
        NEA_DebugPrint("NEA_Hw2DTextCtxBitmapLoadGRF: GRF/BG fmt mismatch "
                       "(gfxAttr=%u)", header->gfxAttr);
        return -1;
    }

    // 8bpp requires a palette; 16bpp ignores it.
    if (ctx->fmt == DSF_BMP_RGB256 && *palDst == NULL)
    {
        NEA_DebugPrint("NEA_Hw2DTextCtxBitmapLoadGRF: 8bpp font needs palette");
        return -1;
    }

    dsf_error derr = DSF_FontTextureSet(ctx->font_handle, *gfxDst,
                                         header->gfxWidth, header->gfxHeight,
                                         ctx->fmt);
    if (derr != DSF_NO_ERROR)
    {
        NEA_DebugPrint("NEA_Hw2DTextCtxBitmapLoadGRF: FontTextureSet %d", derr);
        return -1;
    }

    ne_hw2d_text_copy_pal(ctx, *palDst, palSize);

    // Release any previously-owned buffers, take ownership of the new ones.
    free(ctx->owned_bitmap);
    free(ctx->owned_palette);
    ctx->owned_bitmap = *gfxDst;
    ctx->owned_palette = *palDst;
    ctx->bitmap_set = true;

    *gfxDst = NULL;
    *palDst = NULL;
    return 0;
}
#endif // NEA_BLOCKSDS

int NEA_Hw2DTextCtxBitmapLoadGRF(NEA_Hw2DTextCtx *ctx, const char *path)
{
#ifndef NEA_BLOCKSDS
    (void)ctx;
    (void)path;
    NEA_DebugPrint("%s only supported in BlocksDS", __func__);
    return -1;
#else
    NEA_AssertPointer(ctx, "NULL ctx");
    NEA_AssertPointer(path, "NULL path");
    NEA_Assert(ctx->used, "ctx not active");

    void *gfxDst = NULL;
    void *palDst = NULL;
    size_t palSize = 0;
    GRFHeader header = { 0 };

    GRFError err = grfLoadPath(path, &header, &gfxDst, NULL, NULL, NULL,
                               &palDst, &palSize);
    if (err != GRF_NO_ERROR)
    {
        NEA_DebugPrint("NEA_Hw2DTextCtxBitmapLoadGRF: grfLoadPath %d", err);
        free(gfxDst);
        free(palDst);
        return -1;
    }

    int ret = ne_hw2d_textctx_grf_apply(ctx, &header, &gfxDst,
                                        &palDst, palSize);

    // Both are NULL if the context took them over.
    free(gfxDst);
    free(palDst);
    return ret;
#endif
}

int NEA_Hw2DTextCtxSetCanvas(NEA_Hw2DTextCtx *ctx, int left, int top,
                              int right, int bottom)
{
    NEA_AssertPointer(ctx, "NULL ctx");
    NEA_Assert(ctx->used, "ctx not active");

    dsf_error err = DSF_RendererCanvasSetup(ctx->renderer,
                                             (int16_t)left, (int16_t)top,
                                             (int16_t)right, (int16_t)bottom);
    if (err != DSF_NO_ERROR)
    {
        NEA_DebugPrint("NEA_Hw2DTextCtxSetCanvas: %d", err);
        return -1;
    }
    ctx->canvas_left = left;
    ctx->canvas_top = top;
    ctx->canvas_right = right;
    ctx->canvas_bottom = bottom;
    return 0;
}

// The pixel buffer a ctx renders into, plus its dimensions: either the BG's
// VRAM or the caller's RAM buffer, depending on which constructor was used.
static u16 *ne_hw2d_text_dst(const NEA_Hw2DTextCtx *ctx, int *width,
                              int *height)
{
    if (ctx->bg != NULL)
    {
        *width = ctx->bg->width;
        *height = ctx->bg->height;
        return ctx->bg->gfx_ptr;
    }
    *width = ctx->buf_width;
    *height = ctx->buf_height;
    return (u16 *)ctx->buffer;
}

// memset for a byte-indexed run that may live in VRAM. VRAM drops 8-bit
// stores, so the unaligned head/tail bytes are written as a read-modify-write
// of the halfword that contains them, and the body goes out 16 bits at a time.
static void ne_vram_set8(void *dst, u8 value, size_t bytes)
{
    u8 *p = (u8 *)dst;
    const u16 pair = (u16)(value | (value << 8));

    if (bytes > 0 && (((uintptr_t)p) & 1))
    {
        u16 *h = (u16 *)(p - 1);
        *h = (u16)((*h & 0x00FF) | (value << 8));
        p++;
        bytes--;
    }
    while (bytes >= 2)
    {
        *(u16 *)p = pair;
        p += 2;
        bytes -= 2;
    }
    if (bytes > 0)
    {
        u16 *h = (u16 *)p;
        *h = (u16)((*h & 0xFF00) | value);
    }
}

int NEA_Hw2DTextCtxClear(NEA_Hw2DTextCtx *ctx)
{
    NEA_AssertPointer(ctx, "NULL ctx");
    NEA_Assert(ctx->used, "ctx not active");

    int dst_w, dst_h;
    u16 *dst = ne_hw2d_text_dst(ctx, &dst_w, &dst_h);

    // Wipe the canvas region. Both supported formats use 0 as "no glyph":
    // index 0 is transparent for RGB256, alpha-0 (== full zero) is
    // transparent for RGBA. So byte-zero is correct for both.
    int l = ctx->canvas_left;
    int t = ctx->canvas_top;
    int r = ctx->canvas_right;
    int b = ctx->canvas_bottom;
    if (l < 0) l = 0;
    if (t < 0) t = 0;
    if (r > dst_w) r = dst_w;
    if (b > dst_h) b = dst_h;

    const int width = r - l;
    if (width > 0)
    {
        if (ctx->fmt == DSF_BMP_RGB256)
        {
            // 8bpp: one byte per pixel, so a row can start or end mid-halfword.
            // ne_vram_set8 keeps that legal when the destination is VRAM.
            u8 *base = (u8 *)dst;
            for (int row = t; row < b; row++)
                ne_vram_set8(base + row * dst_w + l, 0, (size_t)width);
        }
        else
        {
            // 16bpp: every pixel is its own halfword, always VRAM-legal.
            for (int row = t; row < b; row++)
            {
                u16 *p = dst + row * dst_w + l;
                for (int i = 0; i < width; i++)
                    p[i] = 0;
            }
        }
    }

    dsf_error err = DSF_RendererCanvasClear(ctx->renderer);
    if (err != DSF_NO_ERROR)
    {
        NEA_DebugPrint("NEA_Hw2DTextCtxClear: %d", err);
        return -1;
    }
    return 0;
}

int NEA_Hw2DTextCtxBlitToBG(NEA_Hw2DTextCtx *ctx, NEA_Hw2DBG *bg, int x, int y)
{
    NEA_AssertPointer(ctx, "NULL ctx");
    NEA_AssertPointer(bg, "NULL bg");
    NEA_Assert(ctx->used, "ctx not active");
    NEA_Assert(bg->used, "BG not active");

    if (ctx->buffer == NULL)
    {
        NEA_DebugPrint("NEA_Hw2DTextCtxBlitToBG: ctx is not buffer-backed");
        return -1;
    }

    dsf_format bg_fmt;
    if (ne_hw2d_text_fmt(bg->type, &bg_fmt) != 0 || bg_fmt != ctx->fmt)
    {
        NEA_DebugPrint("NEA_Hw2DTextCtxBlitToBG: ctx/BG format mismatch");
        return -1;
    }

    // Clip the source rectangle against the destination BG.
    int src_x = 0, src_y = 0;
    int w = ctx->buf_width;
    int h = ctx->buf_height;
    if (x < 0) { src_x = -x; w += x; x = 0; }
    if (y < 0) { src_y = -y; h += y; y = 0; }
    if (x + w > bg->width)  w = bg->width - x;
    if (y + h > bg->height) h = bg->height - y;
    if (w <= 0 || h <= 0)
        return 0;

    const int pixel_bytes = (ctx->fmt == DSF_BMP_RGB256) ? 1 : 2;
    const size_t row_bytes = (size_t)w * pixel_bytes;

    u8 *src = (u8 *)ctx->buffer
            + ((size_t)src_y * ctx->buf_width + src_x) * pixel_bytes;
    u8 *dst = (u8 *)bg->gfx_ptr
            + ((size_t)y * bg->width + x) * pixel_bytes;

    for (int row = 0; row < h; row++)
    {
        ne_vram_copy(dst, src, row_bytes);
        src += (size_t)ctx->buf_width * pixel_bytes;
        dst += (size_t)bg->width * pixel_bytes;
    }
    return 0;
}

int NEA_Hw2DTextCtxSetOverflow(NEA_Hw2DTextCtx *ctx,
                                NEA_Hw2DTextRightMode right_mode,
                                NEA_Hw2DTextBottomMode bottom_mode)
{
    NEA_AssertPointer(ctx, "NULL ctx");
    NEA_Assert(ctx->used, "ctx not active");

    dsf_error err = DSF_RendererOverflowModeSet(ctx->renderer,
                            (dsf_render_right_mode)right_mode,
                            (dsf_render_bottom_mode)bottom_mode);
    return ne_hw2d_text_map_err(err, "SetOverflow");
}

int NEA_Hw2DTextCtxSetWordWrap(NEA_Hw2DTextCtx *ctx, bool enabled,
                                const uint32_t *separators)
{
    NEA_AssertPointer(ctx, "NULL ctx");
    NEA_Assert(ctx->used, "ctx not active");

    dsf_error err = DSF_RendererWordWrapModeSet(ctx->renderer, enabled,
                                                 separators);
    return ne_hw2d_text_map_err(err, "SetWordWrap");
}

int NEA_Hw2DTextCtxCursorGet(NEA_Hw2DTextCtx *ctx, int *x, int *y)
{
    NEA_AssertPointer(ctx, "NULL ctx");
    NEA_Assert(ctx->used, "ctx not active");
    dsf_error err = DSF_RendererCursorGet(ctx->renderer, x, y);
    return ne_hw2d_text_map_err(err, "CursorGet");
}

int NEA_Hw2DTextCtxCursorSet(NEA_Hw2DTextCtx *ctx, int x, int y)
{
    NEA_AssertPointer(ctx, "NULL ctx");
    NEA_Assert(ctx->used, "ctx not active");
    dsf_error err = DSF_RendererCursorSet(ctx->renderer,
                                           (int16_t)x, (int16_t)y);
    return ne_hw2d_text_map_err(err, "CursorSet");
}

int NEA_Hw2DTextCtxUsedBoxGet(NEA_Hw2DTextCtx *ctx,
                               int16_t *left, int16_t *top,
                               int16_t *right, int16_t *bottom)
{
    NEA_AssertPointer(ctx, "NULL ctx");
    NEA_Assert(ctx->used, "ctx not active");
    dsf_error err = DSF_RendererUsedBoxGet(ctx->renderer,
                                            left, top, right, bottom);
    return ne_hw2d_text_map_err(err, "UsedBoxGet");
}

int NEA_Hw2DTextCtxPrint(NEA_Hw2DTextCtx *ctx, const char *str)
{
    NEA_AssertPointer(ctx, "NULL ctx");
    NEA_AssertPointer(str, "NULL str");
    NEA_Assert(ctx->used, "ctx not active");
    dsf_error err = DSF_StringRender(ctx->font_handle, ctx->renderer, str);
    return ne_hw2d_text_map_err(err, "Print");
}

int NEA_Hw2DTextCtxPrintLength(NEA_Hw2DTextCtx *ctx, size_t *characters,
                                const char **str)
{
    NEA_AssertPointer(ctx, "NULL ctx");
    NEA_AssertPointer(characters, "NULL characters");
    NEA_AssertPointer(str, "NULL str");
    NEA_Assert(ctx->used, "ctx not active");
    dsf_error err = DSF_StringRenderLength(ctx->font_handle, ctx->renderer,
                                            characters, str);
    return ne_hw2d_text_map_err(err, "PrintLength");
}

int NEA_Hw2DTextCtxPrintCodepoint(NEA_Hw2DTextCtx *ctx, uint32_t codepoint)
{
    NEA_AssertPointer(ctx, "NULL ctx");
    NEA_Assert(ctx->used, "ctx not active");
    dsf_error err = DSF_CodepointRender(ctx->font_handle, ctx->renderer,
                                         codepoint);
    return ne_hw2d_text_map_err(err, "PrintCodepoint");
}

int NEA_Hw2DTextCtxPrintf(NEA_Hw2DTextCtx *ctx, const char *fmt, ...)
{
    NEA_AssertPointer(ctx, "NULL ctx");
    NEA_AssertPointer(fmt, "NULL fmt");
    NEA_Assert(ctx->used, "ctx not active");

    // Mirror DSF_StringRenderFormat's vasprintf-based approach so the format
    // semantics match what callers already get from libdsf directly.
    char *buf = NULL;
    va_list args;
    va_start(args, fmt);
    int rc = vasprintf(&buf, fmt, args);
    va_end(args);

    if (rc == -1)
    {
        NEA_DebugPrint("NEA_Hw2DTextCtxPrintf: vasprintf failed");
        return -1;
    }

    dsf_error err = DSF_StringRender(ctx->font_handle, ctx->renderer, buf);
    free(buf);
    return ne_hw2d_text_map_err(err, "Printf");
}

// ---------------------------------------------------------------------------
// Asynchronous OBJ, asset and text context loading
// ---------------------------------------------------------------------------

// What kind of plain (non-GRF) asynchronous load this is.
typedef enum {
    NE_ASYNC_HW2D_OBJ_GFX,
    NE_ASYNC_HW2D_OBJ_PALETTE,
    NE_ASYNC_HW2D_ASSET_GFX,
    NE_ASYNC_HW2D_ASSET_PALETTE,
    NE_ASYNC_HW2D_TEXTCTX_METADATA
} ne_async_hw2d_kind;

// Parameters of a plain asynchronous OBJ / asset / text context load.
typedef struct {
    ne_async_hw2d_kind kind;
    union {
        NEA_Hw2DOBJ *obj;
        NEA_Hw2DOBJAsset *asset;
        NEA_Hw2DTextCtx *ctx;
    } target;
    NEA_Hw2DEngine engine;  // NE_ASYNC_HW2D_OBJ_PALETTE only
    int palette_slot;       // NE_ASYNC_HW2D_OBJ_PALETTE only
} ne_async_hw2d_param;

// Runs on the main thread during the vertical blank: hands the loaded data to
// the regular (synchronous) loader.
static void ne_async_hw2d_finalize(NEA_AsyncFile *job)
{
    ne_async_hw2d_param *p = __NEA_AsyncParam(job);

    size_t size = 0;
    char *buffer = __NEA_AsyncBuffer(job, &size);

    int ret;
    switch (p->kind)
    {
        case NE_ASYNC_HW2D_OBJ_GFX:
            ret = NEA_Hw2DOBJLoadGfx(p->target.obj, buffer, size);
            break;

        case NE_ASYNC_HW2D_OBJ_PALETTE:
            // RGB15 palettes are 2 bytes per color.
            ret = NEA_Hw2DOBJLoadPalette(p->engine, buffer, size / 2,
                                         p->palette_slot);
            break;

        case NE_ASYNC_HW2D_ASSET_GFX:
            ret = NEA_Hw2DOBJAssetLoadGfx(p->target.asset, buffer, size);
            break;

        case NE_ASYNC_HW2D_ASSET_PALETTE:
            ret = NEA_Hw2DOBJAssetLoadPalette(p->target.asset, buffer,
                                              size / 2);
            break;

        default:
            ret = NEA_Hw2DTextCtxMetadataLoadMemory(p->target.ctx, buffer,
                                                    size);
            break;
    }

    // The Hw2D loaders return 0 on success, the async system wants 1.
    __NEA_AsyncSetResult(job, ret == 0);
}

// Queues a plain asynchronous load. 'target' is the object the finalize step
// writes into, so that deleting it aborts the load; it is NULL for the OBJ
// palette loader, which writes into the global OBJ palette region instead.
static NEA_AsyncFile *ne_async_hw2d_queue(const char *path,
                                          const ne_async_hw2d_param *params,
                                          void *target)
{
    ne_async_hw2d_param *p = malloc(sizeof(ne_async_hw2d_param));
    if (p == NULL)
    {
        NEA_DebugPrint("Out of memory");
        return NULL;
    }

    *p = *params;

    NEA_AsyncFile *job = __NEA_AsyncQueue(path, NULL, ne_async_hw2d_finalize,
                                          NULL, p, target);
    if (job == NULL)
        free(p);

    return job;
}

NEA_AsyncFile *NEA_Hw2DOBJLoadGfxFATAsync(NEA_Hw2DOBJ *obj, const char *path)
{
    NEA_AssertPointer(obj, "NULL obj");
    NEA_AssertPointer(path, "NULL path");
    NEA_Assert(obj->used, "OBJ not active");

    ne_async_hw2d_param params = { 0 };
    params.kind = NE_ASYNC_HW2D_OBJ_GFX;
    params.target.obj = obj;

    return ne_async_hw2d_queue(path, &params, obj);
}

NEA_AsyncFile *NEA_Hw2DOBJLoadPaletteFATAsync(NEA_Hw2DEngine engine,
                                              const char *path, int slot)
{
    NEA_AssertPointer(path, "NULL path");

    ne_async_hw2d_param params = { 0 };
    params.kind = NE_ASYNC_HW2D_OBJ_PALETTE;
    params.engine = engine;
    params.palette_slot = slot;

    // There is no object to hang this load off: it writes into the engine's OBJ
    // palette region. Target the 2D system itself, so that NEA_Hw2DSystemEnd()
    // aborts it before the VRAM banks are released.
    return ne_async_hw2d_queue(path, &params, &ne_hw2d_state);
}

NEA_AsyncFile *NEA_Hw2DOBJAssetLoadGfxFATAsync(NEA_Hw2DOBJAsset *asset,
                                               const char *path)
{
    NEA_AssertPointer(asset, "NULL asset");
    NEA_AssertPointer(path, "NULL path");
    NEA_Assert(asset->used, "Asset not active");

    ne_async_hw2d_param params = { 0 };
    params.kind = NE_ASYNC_HW2D_ASSET_GFX;
    params.target.asset = asset;

    return ne_async_hw2d_queue(path, &params, asset);
}

NEA_AsyncFile *NEA_Hw2DOBJAssetLoadPaletteFATAsync(NEA_Hw2DOBJAsset *asset,
                                                   const char *path)
{
    NEA_AssertPointer(asset, "NULL asset");
    NEA_AssertPointer(path, "NULL path");
    NEA_Assert(asset->used, "Asset not active");

    ne_async_hw2d_param params = { 0 };
    params.kind = NE_ASYNC_HW2D_ASSET_PALETTE;
    params.target.asset = asset;

    return ne_async_hw2d_queue(path, &params, asset);
}

NEA_AsyncFile *NEA_Hw2DTextCtxMetadataLoadFATAsync(NEA_Hw2DTextCtx *ctx,
                                                   const char *path)
{
    NEA_AssertPointer(ctx, "NULL ctx");
    NEA_AssertPointer(path, "NULL path");
    NEA_Assert(ctx->used, "ctx not active");

    ne_async_hw2d_param params = { 0 };
    params.kind = NE_ASYNC_HW2D_TEXTCTX_METADATA;
    params.target.ctx = ctx;

    return ne_async_hw2d_queue(path, &params, ctx);
}

// ---------------------------------------------------------------------------
// Asynchronous GRF loading
// ---------------------------------------------------------------------------

#ifdef NEA_BLOCKSDS

// Which kind of object an asynchronous GRF load targets.
typedef enum {
    NE_ASYNC_GRF_BG,
    NE_ASYNC_GRF_OBJ,
    NE_ASYNC_GRF_ASSET,
    NE_ASYNC_GRF_TEXTCTX
} ne_async_grf_kind;

// Parameters of an asynchronous GRF load. The decoded buffers are produced by
// the worker thread and consumed (or freed) on the main thread.
typedef struct {
    ne_async_grf_kind kind;
    union {
        NEA_Hw2DBG *bg;
        NEA_Hw2DOBJ *obj;
        NEA_Hw2DOBJAsset *asset;
        NEA_Hw2DTextCtx *ctx;
    } target;
    int palette_slot;

    GRFHeader header;
    void *gfxDst;       // Decoded buffers (owned)
    void *mapDst;
    void *palDst;
    size_t gfxSize;
    size_t mapSize;
    size_t palSize;
} ne_async_hw2d_grf_param;

// Runs in the worker thread: decodes the GRF file from the RAM buffer. This is
// the expensive part (decompression), which is why it doesn't run on the main
// thread like the finalize step does.
static bool ne_async_hw2d_grf_stage2(NEA_AsyncFile *job)
{
    ne_async_hw2d_grf_param *p = __NEA_AsyncParam(job);

    size_t raw_size = 0;
    char *raw = __NEA_AsyncBuffer(job, &raw_size);

    // grfLoadMemEx() trusts the lengths inside the file, so check the buffer
    // really holds a complete GRF before letting it walk off the end.
    if (!__NEA_GRFBufferIsSane(raw, raw_size))
    {
        NEA_DebugPrint("Not a complete GRF file: %s", "async buffer");
        __NEA_AsyncFreeBuffer(job);
        return false;
    }

    // Only BGs can use a map; asking for one everywhere else would decode a
    // chunk that is then thrown away.
    bool wants_map = (p->kind == NE_ASYNC_GRF_BG);

    GRFError err = grfLoadMemEx(raw, &p->header,
                                &p->gfxDst, &p->gfxSize,
                                wants_map ? &p->mapDst : NULL,
                                wants_map ? &p->mapSize : NULL,
                                &p->palDst, &p->palSize,
                                NULL, NULL, NULL, NULL);

    // The raw GRF file isn't needed once it has been decoded.
    __NEA_AsyncFreeBuffer(job);

    if (err != GRF_NO_ERROR)
    {
        NEA_DebugPrint("Couldn't decode GRF file: %d", err);
        return false;
    }

    return true;
}

static void ne_async_hw2d_grf_finalize(NEA_AsyncFile *job)
{
    ne_async_hw2d_grf_param *p = __NEA_AsyncParam(job);

    int ret;
    switch (p->kind)
    {
        case NE_ASYNC_GRF_BG:
            ret = ne_hw2d_bg_grf_apply(p->target.bg, &p->header,
                                       p->gfxDst, p->gfxSize,
                                       p->mapDst, p->mapSize,
                                       p->palDst, p->palSize,
                                       p->palette_slot);
            break;

        case NE_ASYNC_GRF_OBJ:
            ret = ne_hw2d_obj_grf_apply(p->target.obj, &p->header,
                                        p->gfxDst, p->gfxSize,
                                        p->palDst, p->palSize,
                                        p->palette_slot);
            break;

        case NE_ASYNC_GRF_ASSET:
            ret = ne_hw2d_asset_grf_apply(p->target.asset, &p->header,
                                          p->gfxDst, p->gfxSize,
                                          p->palDst, p->palSize);
            break;

        default:
            // On success the context takes over gfxDst and palDst and clears
            // them here, so the frees below leave them alone.
            ret = ne_hw2d_textctx_grf_apply(p->target.ctx, &p->header,
                                            &p->gfxDst, &p->palDst,
                                            p->palSize);
            break;
    }

    // The Hw2D loaders return 0 on success, the async system wants 1.
    __NEA_AsyncSetResult(job, ret == 0);

    free(p->gfxDst);
    free(p->mapDst);
    free(p->palDst);
    p->gfxDst = NULL;
    p->mapDst = NULL;
    p->palDst = NULL;
}

// Runs when a job is dropped before its finalize step, so that the buffers the
// worker decoded aren't leaked.
static void ne_async_hw2d_grf_discard(NEA_AsyncFile *job)
{
    ne_async_hw2d_grf_param *p = __NEA_AsyncParam(job);
    free(p->gfxDst);
    free(p->mapDst);
    free(p->palDst);
}

static NEA_AsyncFile *ne_async_hw2d_grf_queue(
        const char *path, const ne_async_hw2d_grf_param *params, void *target)
{
    ne_async_hw2d_grf_param *p = calloc(1, sizeof(ne_async_hw2d_grf_param));
    if (p == NULL)
    {
        NEA_DebugPrint("Out of memory");
        return NULL;
    }

    *p = *params;

    NEA_AsyncFile *job = __NEA_AsyncQueue(path, ne_async_hw2d_grf_stage2,
                                          ne_async_hw2d_grf_finalize,
                                          ne_async_hw2d_grf_discard, p, target);
    if (job == NULL)
        free(p);

    return job;
}

#endif // NEA_BLOCKSDS

NEA_AsyncFile *NEA_Hw2DBGLoadGRFFATAsync(NEA_Hw2DBG *bg, const char *path,
                                         int palette_slot)
{
#ifndef NEA_BLOCKSDS
    (void)bg;
    (void)path;
    (void)palette_slot;
    NEA_DebugPrint("%s only supported in BlocksDS", __func__);
    return NULL;
#else
    NEA_AssertPointer(bg, "NULL bg");
    NEA_AssertPointer(path, "NULL path");
    NEA_Assert(bg->used, "BG not active");

    ne_async_hw2d_grf_param params = { 0 };
    params.kind = NE_ASYNC_GRF_BG;
    params.target.bg = bg;
    params.palette_slot = palette_slot;

    return ne_async_hw2d_grf_queue(path, &params, bg);
#endif
}

NEA_AsyncFile *NEA_Hw2DOBJLoadGRFFATAsync(NEA_Hw2DOBJ *obj, const char *path,
                                          int palette_slot)
{
#ifndef NEA_BLOCKSDS
    (void)obj;
    (void)path;
    (void)palette_slot;
    NEA_DebugPrint("%s only supported in BlocksDS", __func__);
    return NULL;
#else
    NEA_AssertPointer(obj, "NULL obj");
    NEA_AssertPointer(path, "NULL path");
    NEA_Assert(obj->used, "OBJ not active");

    ne_async_hw2d_grf_param params = { 0 };
    params.kind = NE_ASYNC_GRF_OBJ;
    params.target.obj = obj;
    params.palette_slot = palette_slot;

    return ne_async_hw2d_grf_queue(path, &params, obj);
#endif
}

NEA_AsyncFile *NEA_Hw2DOBJAssetLoadGRFFATAsync(NEA_Hw2DOBJAsset *asset,
                                               const char *path)
{
#ifndef NEA_BLOCKSDS
    (void)asset;
    (void)path;
    NEA_DebugPrint("%s only supported in BlocksDS", __func__);
    return NULL;
#else
    NEA_AssertPointer(asset, "NULL asset");
    NEA_AssertPointer(path, "NULL path");
    NEA_Assert(asset->used, "Asset not active");

    ne_async_hw2d_grf_param params = { 0 };
    params.kind = NE_ASYNC_GRF_ASSET;
    params.target.asset = asset;

    return ne_async_hw2d_grf_queue(path, &params, asset);
#endif
}

NEA_AsyncFile *NEA_Hw2DTextCtxBitmapLoadGRFAsync(NEA_Hw2DTextCtx *ctx,
                                                 const char *path)
{
#ifndef NEA_BLOCKSDS
    (void)ctx;
    (void)path;
    NEA_DebugPrint("%s only supported in BlocksDS", __func__);
    return NULL;
#else
    NEA_AssertPointer(ctx, "NULL ctx");
    NEA_AssertPointer(path, "NULL path");
    NEA_Assert(ctx->used, "ctx not active");

    ne_async_hw2d_grf_param params = { 0 };
    params.kind = NE_ASYNC_GRF_TEXTCTX;
    params.target.ctx = ctx;

    return ne_async_hw2d_grf_queue(path, &params, ctx);
#endif
}
