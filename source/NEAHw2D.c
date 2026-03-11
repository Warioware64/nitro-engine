// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Warioware64
//
// This file is part of Nitro Engine Advanced

#include "NEAMain.h"
#include "dsf.h"

/// @file NEAHw2D.c

// ---------------------------------------------------------------------------
// Internal state
// ---------------------------------------------------------------------------

#define NEA_HW2D_MAX_OAM 128

static struct {
    bool initialized;
    NEA_Hw2DVRAMConfig vram_config;
    NEA_VRAMBankFlags claimed_banks;

    NEA_Hw2DBG bgs_main[4];
    NEA_Hw2DBG bgs_sub[4];

    NEA_Hw2DOBJ objs_main[NEA_HW2D_MAX_OAM];
    NEA_Hw2DOBJ objs_sub[NEA_HW2D_MAX_OAM];

    int tile_base_next_main;
    int map_base_next_main;
    int tile_base_next_sub;
    int map_base_next_sub;

    int next_oam_main;
    int next_oam_sub;

    bool main_obj_inited;
    bool sub_obj_inited;
    bool main_has_bitmap;
} ne_hw2d_state;

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

    // If bank E is claimed for 2D, remove it from texture palette duty.
    // This prevents ne_init_registers() from re-assigning it on future inits.
    if ((mb | mo) & NEA_VRAM_E)
        NEA_SetTexPaletteBank(0);

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
        u32 sub_mode = MODE_0_2D;
        if (so)
        {
            sub_mode |= DISPLAY_SPR_ACTIVE | DISPLAY_SPR_1D;
            oamInit(&oamSub, SpriteMapping_1D_128, false);
            ne_hw2d_state.sub_obj_inited = true;
        }
        videoSetModeSub(sub_mode);
    }

    // Allocation layout (matching standard NDS/BlocksDS convention):
    // Maps at low offsets (base 0+), tiles at higher offsets (base 1+ = 16KB+).
    // This keeps map and tile data separate with no overlap.
    ne_hw2d_state.map_base_next_main = 0;
    ne_hw2d_state.map_base_next_sub = 0;
    ne_hw2d_state.tile_base_next_main = 1;
    ne_hw2d_state.tile_base_next_sub = 1;

    ne_hw2d_state.initialized = true;
    return 0;
}

void NEA_Hw2DSystemEnd(void)
{
    if (!ne_hw2d_state.initialized)
        return;

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

    // Free all OBJ sprites
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
            if (objs[i].used && objs[i].gfx)
            {
                oamFreeGfx(oam, objs[i].gfx);
                objs[i].used = false;
            }
        }
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

    // Allocate tile and map bases.
    // Layout: maps at low offsets (growing up), tiles at higher offsets
    // (growing up from base 1 = 16KB). This matches the standard libnds
    // convention (e.g. mapBase=0, tileBase=1 in BlocksDS examples).
    int tile_base, map_base;
    if (is_bitmap)
    {
        tile_base = 0;
        map_base = 0;
    }
    else
    {
        // Map slot count: each SBB is 2KB, 32x32 tiles
        int map_slots = 1;
        if (width == 512)
            map_slots *= 2;
        if (height == 512)
            map_slots *= 2;

        if (engine == NEA_ENGINE_MAIN)
        {
            map_base = ne_hw2d_state.map_base_next_main;
            ne_hw2d_state.map_base_next_main += map_slots;

            tile_base = ne_hw2d_state.tile_base_next_main;
            ne_hw2d_state.tile_base_next_main += 1;
        }
        else
        {
            map_base = ne_hw2d_state.map_base_next_sub;
            ne_hw2d_state.map_base_next_sub += map_slots;

            tile_base = ne_hw2d_state.tile_base_next_sub;
            ne_hw2d_state.tile_base_next_sub += 1;
        }
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
    bg->scroll_x = 0;
    bg->scroll_y = 0;
    bg->visible = true;

    return bg;
}

void NEA_Hw2DBGDelete(NEA_Hw2DBG *bg)
{
    if (bg == NULL || !bg->used)
        return;

    bgHide(bg->bg_id);
    memset(bg, 0, sizeof(*bg));
}

int NEA_Hw2DBGLoadTiles(NEA_Hw2DBG *bg, const void *data, size_t size)
{
    NEA_AssertPointer(bg, "NULL bg");
    NEA_AssertPointer(data, "NULL data");
    NEA_Assert(bg->used, "BG not active");
    NEA_Assert(bg->type <= NEA_HW2D_BG_TILED_8BPP, "Not a tiled BG");

    memcpy(bg->gfx_ptr, data, size);
    return 0;
}

int NEA_Hw2DBGLoadTilesFAT(NEA_Hw2DBG *bg, const char *path)
{
    NEA_AssertPointer(bg, "NULL bg");
    NEA_AssertPointer(path, "NULL path");
    NEA_Assert(bg->used, "BG not active");
    NEA_Assert(bg->type <= NEA_HW2D_BG_TILED_8BPP, "Not a tiled BG");

    FILE *f = fopen(path, "rb");
    if (f == NULL)
    {
        NEA_DebugPrint("Failed to open: %s", path);
        return -1;
    }

    fseek(f, 0, SEEK_END);
    size_t size = ftell(f);
    fseek(f, 0, SEEK_SET);

    void *buf = malloc(size);
    if (buf == NULL)
    {
        fclose(f);
        NEA_DebugPrint("Out of memory");
        return -1;
    }

    fread(buf, 1, size, f);
    fclose(f);

    memcpy(bg->gfx_ptr, buf, size);
    free(buf);
    return 0;
}

int NEA_Hw2DBGLoadMap(NEA_Hw2DBG *bg, const void *data, size_t size)
{
    NEA_AssertPointer(bg, "NULL bg");
    NEA_AssertPointer(data, "NULL data");
    NEA_Assert(bg->used, "BG not active");
    NEA_Assert(bg->map_ptr != NULL, "Not a tiled BG");

    memcpy(bg->map_ptr, data, size);
    return 0;
}

int NEA_Hw2DBGLoadMapFAT(NEA_Hw2DBG *bg, const char *path)
{
    NEA_AssertPointer(bg, "NULL bg");
    NEA_AssertPointer(path, "NULL path");
    NEA_Assert(bg->used, "BG not active");
    NEA_Assert(bg->map_ptr != NULL, "Not a tiled BG");

    FILE *f = fopen(path, "rb");
    if (f == NULL)
    {
        NEA_DebugPrint("Failed to open: %s", path);
        return -1;
    }

    fseek(f, 0, SEEK_END);
    size_t size = ftell(f);
    fseek(f, 0, SEEK_SET);

    void *buf = malloc(size);
    if (buf == NULL)
    {
        fclose(f);
        NEA_DebugPrint("Out of memory");
        return -1;
    }

    fread(buf, 1, size, f);
    fclose(f);

    memcpy(bg->map_ptr, buf, size);
    free(buf);
    return 0;
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
    if (bg->type == NEA_HW2D_BG_TILED_4BPP)
        pal_ptr += slot * 16;

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

    memcpy(bg->gfx_ptr, data, size);
    return 0;
}

int NEA_Hw2DBGLoadBitmapFAT(NEA_Hw2DBG *bg, const char *path)
{
    NEA_AssertPointer(bg, "NULL bg");
    NEA_AssertPointer(path, "NULL path");
    NEA_Assert(bg->used, "BG not active");
    NEA_Assert(bg->type >= NEA_HW2D_BG_BITMAP_8, "Not a bitmap BG");

    FILE *f = fopen(path, "rb");
    if (f == NULL)
    {
        NEA_DebugPrint("Failed to open: %s", path);
        return -1;
    }

    fseek(f, 0, SEEK_END);
    size_t size = ftell(f);
    fseek(f, 0, SEEK_SET);

    void *buf = malloc(size);
    if (buf == NULL)
    {
        fclose(f);
        NEA_DebugPrint("Out of memory");
        return -1;
    }

    fread(buf, 1, size, f);
    fclose(f);

    memcpy(bg->gfx_ptr, buf, size);
    free(buf);
    return 0;
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
    u8 *ptr = (u8 *)bg->gfx_ptr;
    ptr[y * bg->width + x] = index;
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

    OamState *oam = (obj->engine == NEA_ENGINE_MAIN) ? &oamMain : &oamSub;

    if (obj->gfx)
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

    // Copy first frame
    memcpy(obj->gfx, data, obj->gfx_size);

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

    FILE *f = fopen(path, "rb");
    if (f == NULL)
    {
        NEA_DebugPrint("Failed to open: %s", path);
        return -1;
    }

    fseek(f, 0, SEEK_END);
    size_t size = ftell(f);
    fseek(f, 0, SEEK_SET);

    void *buf = malloc(size);
    if (buf == NULL)
    {
        fclose(f);
        NEA_DebugPrint("Out of memory");
        return -1;
    }

    fread(buf, 1, size, f);
    fclose(f);

    memcpy(obj->gfx, buf, obj->gfx_size);
    obj->num_frames = size / obj->gfx_size;
    if (obj->num_frames < 1)
        obj->num_frames = 1;

    free(buf);
    return 0;
}

int NEA_Hw2DOBJLoadPalette(NEA_Hw2DEngine engine, const void *data,
                            int num_colors, int slot)
{
    NEA_AssertPointer(data, "NULL data");

    u16 *pal = (engine == NEA_ENGINE_MAIN)
             ? SPRITE_PALETTE : SPRITE_PALETTE_SUB;

    pal += slot * 16;
    memcpy(pal, data, num_colors * 2);
    return 0;
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
    oamRotateScale(oam, rot_index, angle, sx, sy);
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

        oamSet(oam, o->oam_index,
               o->x, o->y,
               o->priority,
               o->palette_slot,
               ne_hw2d_obj_size(o->nea_size),
               ne_hw2d_obj_color(o->color),
               o->gfx,
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
// Phase 5: Text rendering on bitmap backgrounds
// ---------------------------------------------------------------------------

int NEA_Hw2DTextRender(NEA_Hw2DBG *bg, u32 slot, const char *str,
                        int x, int y)
{
    NEA_AssertPointer(bg, "NULL bg");
    NEA_AssertPointer(str, "NULL str");
    NEA_Assert(bg->used, "BG not active");
    NEA_Assert(bg->type == NEA_HW2D_BG_BITMAP_16, "Must be 16bpp bitmap BG");

    // Get font bitmap state from rich text slot
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

    // Render text to RAM buffer using libdsf (A1RGB5 format = GL_RGBA)
    void *out_texture = NULL;
    size_t out_w, out_h;
    dsf_error err = DSF_StringRenderToTexture((dsf_handle)handle,
                            str, GL_RGBA, font_texture, font_w, font_h,
                            &out_texture, &out_w, &out_h);
    if (err != DSF_NO_ERROR || out_texture == NULL)
    {
        NEA_DebugPrint("NEA_Hw2DTextRender: render failed (%d)", err);
        free(out_texture);
        return -1;
    }

    // Copy rendered pixels to bitmap BG with clipping and transparency
    u16 *src = (u16 *)out_texture;
    u16 *dst = bg->gfx_ptr;
    int bg_w = bg->width;
    int bg_h = bg->height;

    for (int row = 0; row < (int)out_h; row++)
    {
        int dst_y = y + row;
        if (dst_y < 0)
            continue;
        if (dst_y >= bg_h)
            break;

        for (int col = 0; col < (int)out_w; col++)
        {
            int dst_x = x + col;
            if (dst_x < 0)
                continue;
            if (dst_x >= bg_w)
                break;

            u16 pixel = src[row * out_w + col];
            // Only copy pixels with alpha bit set (bit 15)
            if (pixel & BIT(15))
                dst[dst_y * bg_w + dst_x] = pixel;
        }
    }

    free(out_texture);
    return 0;
}
