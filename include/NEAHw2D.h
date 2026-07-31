// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Warioware64
//
// This file is part of Nitro Engine Advanced

#ifndef NEA_HW2D_H__
#define NEA_HW2D_H__

/// @file   NEAHw2D.h
/// @brief  NDS hardware 2D pipeline (backgrounds, OBJ sprites).

/// @defgroup hw2d Hardware 2D
///
/// Functions to use the NDS 2D hardware alongside the 3D engine.
/// Supports tiled backgrounds, bitmap backgrounds, and hardware OBJ sprites.
///
/// Call NEA_Hw2DInit() after NEA_Init3D() to configure VRAM banks for 2D.
/// Banks claimed for 2D are automatically excluded from 3D texture allocation.
///
/// @{

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------------------------
// Enumerations
// ---------------------------------------------------------------------------

/// 2D engine selection.
typedef enum {
    NEA_ENGINE_MAIN = 0, ///< Main engine (top screen by default)
    NEA_ENGINE_SUB  = 1  ///< Sub engine (bottom screen by default)
} NEA_Hw2DEngine;

/// Background type.
typedef enum {
    NEA_HW2D_BG_TILED_4BPP = 0, ///< 16-color tiled background
    NEA_HW2D_BG_TILED_8BPP = 1, ///< 256-color tiled background
    NEA_HW2D_BG_BITMAP_8   = 2, ///< 8bpp indexed bitmap background
    NEA_HW2D_BG_BITMAP_16  = 3  ///< 16bpp direct color bitmap (RGB15 + alpha)
} NEA_Hw2DBGType;

/// OBJ sprite size. Matches NDS hardware sprite sizes.
typedef enum {
    NEA_OBJ_SIZE_8x8   = 0,
    NEA_OBJ_SIZE_16x16 = 1,
    NEA_OBJ_SIZE_32x32 = 2,
    NEA_OBJ_SIZE_64x64 = 3,
    NEA_OBJ_SIZE_16x8  = 4,
    NEA_OBJ_SIZE_32x8  = 5,
    NEA_OBJ_SIZE_32x16 = 6,
    NEA_OBJ_SIZE_64x32 = 7,
    NEA_OBJ_SIZE_8x16  = 8,
    NEA_OBJ_SIZE_8x32  = 9,
    NEA_OBJ_SIZE_16x32 = 10,
    NEA_OBJ_SIZE_32x64 = 11
} NEA_OBJSize;

/// OBJ sprite color mode.
typedef enum {
    NEA_OBJ_COLOR_16  = 0, ///< 4bpp, 16 colors per palette slot
    NEA_OBJ_COLOR_256 = 1  ///< 8bpp, 256 colors
} NEA_OBJColorMode;

// ---------------------------------------------------------------------------
// Structures
// ---------------------------------------------------------------------------

/// VRAM bank configuration for 2D hardware.
///
/// Each field specifies which VRAM bank(s) to assign for 2D use.
/// Banks assigned here are excluded from 3D texture allocation.
///
/// Hardware constraints:
/// - main_bg:  A, B, C, E (128/128/128/64 KB)
/// - main_obj: A, B, E (128/128/64 KB)
/// - sub_bg:   C, H, I (128/32/16 KB)
/// - sub_obj:  D, I (128/16 KB)
///
/// A bank cannot be assigned to multiple fields.
/// Set a field to 0 to skip that 2D capability.
typedef struct {
    NEA_VRAMBankFlags main_bg;   ///< VRAM for main engine backgrounds
    NEA_VRAMBankFlags main_obj;  ///< VRAM for main engine OBJ sprites
    NEA_VRAMBankFlags sub_bg;    ///< VRAM for sub engine backgrounds
    NEA_VRAMBankFlags sub_obj;   ///< VRAM for sub engine OBJ sprites
} NEA_Hw2DVRAMConfig;

/// Hardware 2D background state.
typedef struct {
    bool used;               ///< Whether this BG slot is active
    NEA_Hw2DEngine engine;   ///< Engine (main or sub)
    int layer;               ///< BG layer (0-3, main engine: 1-3 only)
    NEA_Hw2DBGType type;     ///< Background type
    int bg_id;               ///< libnds BG ID from bgInit/bgInitSub
    int width;               ///< Width in pixels
    int height;              ///< Height in pixels
    u16 *gfx_ptr;            ///< Tile graphics (tiled) or bitmap data (bitmap)
    u16 *map_ptr;            ///< Tile map (tiled only, NULL for bitmap)
    int scroll_x;            ///< Horizontal scroll
    int scroll_y;            ///< Vertical scroll
    bool visible;            ///< Whether the BG is visible
} NEA_Hw2DBG;

/// Forward declaration of shared OBJ asset (see NEA_Hw2DOBJAssetCreate).
typedef struct NEA_Hw2DOBJAsset NEA_Hw2DOBJAsset;

/// Hardware OBJ sprite state.
typedef struct {
    bool used;               ///< Whether this OBJ slot is active
    NEA_Hw2DEngine engine;   ///< Engine (main or sub)
    int oam_index;           ///< OAM entry index (0-127)
    NEA_OBJSize nea_size;    ///< NEA sprite size enum
    NEA_OBJColorMode color;  ///< Color mode
    u16 *gfx;               ///< VRAM graphics data
    int gfx_size;            ///< Bytes per animation frame
    int x, y;                ///< Screen position
    bool visible;            ///< Whether the sprite is visible
    bool hflip, vflip;       ///< Flip state
    int priority;            ///< Draw priority (0 = highest)
    int palette_slot;        ///< Palette slot (16-color mode)
    int frame;               ///< Current animation frame
    int num_frames;          ///< Total animation frames
    int affine_index;        ///< Affine matrix (-1 = none)
    bool double_size;        ///< Double area for affine sprites
    NEA_Hw2DOBJAsset *asset; ///< Shared asset (NULL if OBJ owns its gfx)
} NEA_Hw2DOBJ;

/// Shared OBJ sprite asset.
///
/// Holds a single VRAM gfx allocation and an optional palette slot that can
/// be bound to many NEA_Hw2DOBJ entries. Use this when the same sprite
/// graphic is rendered multiple times on screen (enemies, bullets, tiles):
/// load the gfx and palette once into an asset, then point each OBJ at it
/// via NEA_Hw2DOBJCreateFromAsset() or NEA_Hw2DOBJBindAsset(). This avoids
/// the per-sprite duplicate gfx allocation that fragments OAM gfx memory
/// over create/delete cycles.
struct NEA_Hw2DOBJAsset {
    bool used;               ///< Whether this asset slot is active
    NEA_Hw2DEngine engine;   ///< Engine the asset lives in
    NEA_OBJSize size;        ///< Sprite size class
    NEA_OBJColorMode color;  ///< Color mode (16 or 256)
    u16 *gfx;                ///< VRAM gfx pointer (owned by asset)
    int gfx_size;            ///< Bytes per frame
    int num_frames;          ///< Frames present in gfx (set by Load*)
    int palette_slot;        ///< Palette slot in OBJ palette, -1 if none
    int ref_count;           ///< Number of OBJs currently bound
};

// ---------------------------------------------------------------------------
// System initialization
// ---------------------------------------------------------------------------

/// Initialize the 2D hardware pipeline.
///
/// Call after NEA_Init3D(). Configures VRAM banks for 2D use and sets up
/// video modes and OAM. Banks assigned to 2D are automatically excluded
/// from 3D texture allocation.
///
/// @param config VRAM bank configuration.
/// @return 0 on success, -1 on error (invalid config or overlap).
int NEA_Hw2DInit(const NEA_Hw2DVRAMConfig *config);

/// Auto-configure the 2D hardware pipeline based on the current 3D state.
///
/// Inspects the current NEA execution mode and texture-palette bank, and
/// assigns VRAM banks to 2D that do not displace 3D textures or
/// framebuffers. The chosen layout is:
///
/// - **Sub BG  -> bank H (32 KB)**: when the sub engine is free (all modes
///   except Dual3D / Dual3D_FB / Dual3D_DMA, where libnds owns C/D for
///   sub display).
/// - **Sub OBJ -> bank I (16 KB)**: same condition as sub BG.
/// - **Main BG -> bank E (64 KB)**: only in NEA_ModeSingle3D, and only if
///   no texture-palette banks (NEA_GetTexPaletteBank()) include E.
/// - **Main OBJ -> none**: no main-engine OBJ bank is free without
///   displacing 3D textures.
///
/// Call NEA_Hw2DInit() directly with a custom NEA_Hw2DVRAMConfig if you
/// need a richer layout (e.g. claim bank A or C for main BG after shrinking
/// the 3D texture footprint via NEA_TextureSystemReset()).
///
/// Must be called after one of NEA_Init3D*().
///
/// @return 0 on success, -1 if 3D is not initialized.
int NEA_Hw2DAutoInit(void);

/// Shut down the 2D hardware pipeline and release all resources.
void NEA_Hw2DSystemEnd(void);

/// Returns a bitmask of VRAM banks claimed by the 2D pipeline.
///
/// Used by the texture system to avoid allocating 3D textures in 2D banks.
NEA_VRAMBankFlags NEA_Hw2DGetClaimedBanks(void);

// ---------------------------------------------------------------------------
// Tiled backgrounds
// ---------------------------------------------------------------------------

/// Create a tiled or bitmap background on the specified engine and layer.
///
/// On the main engine, layer 0 is reserved for 3D (use layers 1-3).
/// On the sub engine, layers 0-3 are all available.
///
/// @param engine NEA_ENGINE_MAIN or NEA_ENGINE_SUB.
/// @param layer  BG layer (1-3 for main, 0-3 for sub).
/// @param type   Background type.
/// @param width  Width in pixels (256 or 512 for tiled, 256 for bitmap).
/// @param height Height in pixels (256 or 512 for tiled, 256 for bitmap).
/// @return Pointer to the BG handle, or NULL on error.
NEA_Hw2DBG *NEA_Hw2DBGCreate(NEA_Hw2DEngine engine, int layer,
                              NEA_Hw2DBGType type, int width, int height);

/// Delete a background and free its resources.
///
/// @param bg Background to delete.
void NEA_Hw2DBGDelete(NEA_Hw2DBG *bg);

/// Load tile graphics into a tiled background from RAM.
///
/// @param bg   Tiled background.
/// @param data Pointer to tile graphics data.
/// @param size Size in bytes.
/// @return 0 on success, -1 on error.
int NEA_Hw2DBGLoadTiles(NEA_Hw2DBG *bg, const void *data, size_t size);

/// Load tile graphics from a NitroFS file.
int NEA_Hw2DBGLoadTilesFAT(NEA_Hw2DBG *bg, const char *path);

/// Load a tile map into a tiled background from RAM.
///
/// @param bg   Tiled background.
/// @param data Pointer to map data.
/// @param size Size in bytes.
/// @return 0 on success, -1 on error.
int NEA_Hw2DBGLoadMap(NEA_Hw2DBG *bg, const void *data, size_t size);

/// Load a tile map from a NitroFS file.
int NEA_Hw2DBGLoadMapFAT(NEA_Hw2DBG *bg, const char *path);

/// Load palette data for a background.
///
/// @param bg         Background.
/// @param data       Pointer to palette data (RGB15 colors).
/// @param num_colors Number of colors to load.
/// @param slot       Palette slot (0-15 for 4bpp, 0 for 8bpp).
/// @return 0 on success, -1 on error.
int NEA_Hw2DBGLoadPalette(NEA_Hw2DBG *bg, const void *data,
                           int num_colors, int slot);

/// Set background scroll offset.
void NEA_Hw2DBGSetScroll(NEA_Hw2DBG *bg, int x, int y);

/// Set background draw priority (0 = highest, 3 = lowest).
void NEA_Hw2DBGSetPriority(NEA_Hw2DBG *bg, int priority);

/// Show or hide a background.
void NEA_Hw2DBGSetVisible(NEA_Hw2DBG *bg, bool visible);

/// Set a tile entry in a tiled background's map.
///
/// @param bg    Tiled background.
/// @param x     Tile X coordinate (in tiles, not pixels).
/// @param y     Tile Y coordinate.
/// @param value Tile entry value (tile index + palette + flip bits).
void NEA_Hw2DBGSetTile(NEA_Hw2DBG *bg, int x, int y, u16 value);

/// Get a tile entry from a tiled background's map.
u16 NEA_Hw2DBGGetTile(const NEA_Hw2DBG *bg, int x, int y);

// ---------------------------------------------------------------------------
// Bitmap backgrounds
// ---------------------------------------------------------------------------

/// Load bitmap data into a bitmap background from RAM.
///
/// @param bg   Bitmap background (type must be BITMAP_8 or BITMAP_16).
/// @param data Pointer to bitmap pixel data.
/// @param size Size in bytes.
/// @return 0 on success, -1 on error.
int NEA_Hw2DBGLoadBitmap(NEA_Hw2DBG *bg, const void *data, size_t size);

/// Load bitmap data from a NitroFS file.
int NEA_Hw2DBGLoadBitmapFAT(NEA_Hw2DBG *bg, const char *path);

/// Load a background from a NitroFS GRF file (BlocksDS only).
///
/// Loads tile graphics + map + palette (for tiled backgrounds) or pixel data +
/// palette (for bitmap backgrounds) in a single call. Format is auto-detected
/// from the GRF header and must match the BG's type:
///   NEA_HW2D_BG_TILED_4BPP : gfxAttr=4, mapAttr in {SBB_4BPP, FLAT_4BPP}
///   NEA_HW2D_BG_TILED_8BPP : gfxAttr=8, mapAttr in {SBB_8BPP, AFF_8BPP, FLAT_8BPP}
///   NEA_HW2D_BG_BITMAP_8   : gfxAttr=8, mapAttr = NO_DATA
///   NEA_HW2D_BG_BITMAP_16  : gfxAttr=16, mapAttr = NO_DATA
///
/// @param bg           Background previously created with NEA_Hw2DBGCreate().
/// @param path         NitroFS path to the .grf file.
/// @param palette_slot Palette slot for 4bpp backgrounds (0-15). Ignored for
///                     8bpp/16bpp.
/// @return 0 on success, -1 on error.
int NEA_Hw2DBGLoadGRFFAT(NEA_Hw2DBG *bg, const char *path, int palette_slot);

/// Get the raw VRAM pointer for a bitmap background.
void *NEA_Hw2DBGGetBitmapPtr(const NEA_Hw2DBG *bg);

/// Write a 16bpp pixel to a bitmap background.
///
/// @param bg    16bpp bitmap background.
/// @param x     Pixel X coordinate.
/// @param y     Pixel Y coordinate.
/// @param color RGB15 color with alpha bit (BIT(15) = opaque).
void NEA_Hw2DBGPutPixel16(NEA_Hw2DBG *bg, int x, int y, u16 color);

/// Write an 8bpp palette index to a bitmap background.
void NEA_Hw2DBGPutPixel8(NEA_Hw2DBG *bg, int x, int y, u8 index);

/// Clear a bitmap background to a uniform value.
///
/// For 16bpp: value is an RGB15 color. For 8bpp: replicated palette index.
void NEA_Hw2DBGClearBitmap(NEA_Hw2DBG *bg, u32 value);

// ---------------------------------------------------------------------------
// OBJ sprites
// ---------------------------------------------------------------------------

/// Create a hardware OBJ sprite.
///
/// @param engine NEA_ENGINE_MAIN or NEA_ENGINE_SUB.
/// @param size   Sprite dimensions.
/// @param mode   Color mode (16 or 256 colors).
/// @return Pointer to the OBJ handle, or NULL on error.
NEA_Hw2DOBJ *NEA_Hw2DOBJCreate(NEA_Hw2DEngine engine, NEA_OBJSize size,
                                NEA_OBJColorMode mode);

/// Delete an OBJ sprite and free its graphics memory.
void NEA_Hw2DOBJDelete(NEA_Hw2DOBJ *obj);

/// Load sprite graphics from RAM.
///
/// @param obj  OBJ sprite.
/// @param data Pointer to graphics data.
/// @param size Size in bytes (must be a multiple of the sprite frame size).
/// @return 0 on success, -1 on error.
int NEA_Hw2DOBJLoadGfx(NEA_Hw2DOBJ *obj, const void *data, size_t size);

/// Load sprite graphics from a NitroFS file.
int NEA_Hw2DOBJLoadGfxFAT(NEA_Hw2DOBJ *obj, const char *path);

/// Load OBJ palette data for an engine.
///
/// @param engine    NEA_ENGINE_MAIN or NEA_ENGINE_SUB.
/// @param data      Pointer to palette data (RGB15 colors).
/// @param num_colors Number of colors.
/// @param slot      Palette slot (0-15 for 16-color sprites).
/// @return 0 on success, -1 on error.
int NEA_Hw2DOBJLoadPalette(NEA_Hw2DEngine engine, const void *data,
                            int num_colors, int slot);

/// Load OBJ palette data for an engine from a NitroFS file.
///
/// The color count is derived from the file size (RGB15, 2 bytes per color).
///
/// @param engine NEA_ENGINE_MAIN or NEA_ENGINE_SUB.
/// @param path   NitroFS path to the palette file.
/// @param slot   Palette slot (0-15 for 16-color sprites).
/// @return 0 on success, -1 on error.
int NEA_Hw2DOBJLoadPaletteFAT(NEA_Hw2DEngine engine, const char *path,
                               int slot);

/// Load a sprite from a NitroFS GRF file (BlocksDS only).
///
/// Loads sprite graphics + palette in a single call. The GRF's color depth
/// must match the OBJ's color mode:
///   NEA_OBJ_COLOR_16  : gfxAttr = 4
///   NEA_OBJ_COLOR_256 : gfxAttr = 8
/// Multi-frame sprite sheets are supported: the gfx size may be a multiple of
/// the sprite's frame size.
///
/// @param obj          OBJ sprite previously created with NEA_Hw2DOBJCreate().
/// @param path         NitroFS path to the .grf file.
/// @param palette_slot Palette slot for 16-color sprites (0-15). Ignored for
///                     256-color sprites.
/// @return 0 on success, -1 on error.
int NEA_Hw2DOBJLoadGRFFAT(NEA_Hw2DOBJ *obj, const char *path,
                           int palette_slot);

// ---------------------------------------------------------------------------
// Shared OBJ assets (de-duplicated gfx + palette slot)
// ---------------------------------------------------------------------------

/// Create a shared sprite asset.
///
/// Allocates one sprite's worth of OAM gfx VRAM and reserves an asset slot.
/// Palette slot allocation is deferred until a palette is loaded. The asset
/// can then be bound to one or more NEA_Hw2DOBJ entries to share its gfx
/// and palette slot.
///
/// @param engine NEA_ENGINE_MAIN or NEA_ENGINE_SUB.
/// @param size   Sprite dimensions.
/// @param mode   Color mode.
/// @return Pointer to the asset, or NULL on error (pool full, gfx alloc fail).
NEA_Hw2DOBJAsset *NEA_Hw2DOBJAssetCreate(NEA_Hw2DEngine engine,
                                          NEA_OBJSize size,
                                          NEA_OBJColorMode mode);

/// Delete a shared asset. Fails (prints a warning, no-op) if any OBJ is still
/// bound to it — call NEA_Hw2DOBJDelete() on the dependents first, or rebind
/// them to another asset.
void NEA_Hw2DOBJAssetDelete(NEA_Hw2DOBJAsset *asset);

/// Load sprite graphics into an asset from RAM. Updates the asset's
/// frame count (size / per-frame size).
int NEA_Hw2DOBJAssetLoadGfx(NEA_Hw2DOBJAsset *asset,
                             const void *data, size_t size);

/// Load sprite graphics into an asset from a NitroFS file.
int NEA_Hw2DOBJAssetLoadGfxFAT(NEA_Hw2DOBJAsset *asset, const char *path);

/// Load palette data for an asset.
///
/// For 16-color assets, auto-allocates a free 16-color palette slot on the
/// asset's engine the first time it is called, and reuses that slot on
/// subsequent calls. For 256-color assets, always uses slot 0.
///
/// @param asset      Asset previously created by NEA_Hw2DOBJAssetCreate().
/// @param data       Palette data (RGB15 colors).
/// @param num_colors Number of colors.
/// @return 0 on success, -1 on error (no free 16-color slot).
int NEA_Hw2DOBJAssetLoadPalette(NEA_Hw2DOBJAsset *asset,
                                 const void *data, int num_colors);

/// Load palette data for an asset from a NitroFS file.
///
/// Same slot-allocation behavior as NEA_Hw2DOBJAssetLoadPalette. The color
/// count is derived from the file size (RGB15, 2 bytes per color).
///
/// @param asset Asset previously created by NEA_Hw2DOBJAssetCreate().
/// @param path  NitroFS path to the palette file.
/// @return 0 on success, -1 on error.
int NEA_Hw2DOBJAssetLoadPaletteFAT(NEA_Hw2DOBJAsset *asset, const char *path);

/// Pin a 16-color asset to a specific palette bank (0-15).
///
/// Overrides the bank that would otherwise be auto-allocated when a palette is
/// loaded, transfers the allocator's bank ownership accordingly, and re-points
/// every OBJ already bound to the asset. Ignored for 256-color assets (no-op).
void NEA_Hw2DOBJAssetSetPaletteSlot(NEA_Hw2DOBJAsset *asset, int slot);

/// Load gfx + palette into an asset from a NitroFS GRF file (BlocksDS only).
///
/// Color depth must match the asset's color mode. Palette slot is allocated
/// automatically (see NEA_Hw2DOBJAssetLoadPalette).
int NEA_Hw2DOBJAssetLoadGRFFAT(NEA_Hw2DOBJAsset *asset, const char *path);

/// Create an OBJ that shares the asset's gfx and palette slot.
///
/// Unlike NEA_Hw2DOBJCreate(), this does NOT allocate a new gfx block — the
/// returned OBJ points at the asset's gfx, and the asset's ref count is
/// incremented. The OBJ inherits the asset's size, color mode, and palette
/// slot. Call NEA_Hw2DOBJDelete() as usual; it will release the OBJ entry
/// without freeing the shared gfx.
NEA_Hw2DOBJ *NEA_Hw2DOBJCreateFromAsset(NEA_Hw2DOBJAsset *asset);

/// Bind an existing OBJ to a shared asset.
///
/// If the OBJ previously owned its own gfx (created with NEA_Hw2DOBJCreate()),
/// that gfx is freed back to the OAM allocator. If it was bound to a
/// different asset, that asset's ref count is decremented. The OBJ's
/// size and color mode must match the asset's.
///
/// @return 0 on success, -1 on engine / size / color mismatch.
int NEA_Hw2DOBJBindAsset(NEA_Hw2DOBJ *obj, NEA_Hw2DOBJAsset *asset);

/// Set sprite screen position.
void NEA_Hw2DOBJSetPos(NEA_Hw2DOBJ *obj, int x, int y);

/// Show or hide a sprite.
void NEA_Hw2DOBJSetVisible(NEA_Hw2DOBJ *obj, bool visible);

/// Set sprite horizontal and vertical flip.
void NEA_Hw2DOBJSetFlip(NEA_Hw2DOBJ *obj, bool hflip, bool vflip);

/// Set sprite draw priority (0 = highest, 3 = lowest).
void NEA_Hw2DOBJSetPriority(NEA_Hw2DOBJ *obj, int priority);

/// Select which 16-color palette bank a sprite uses (0-15).
///
/// Use this for 16-color sprites whose palette was loaded into a non-zero slot
/// via NEA_Hw2DOBJLoadPalette(). Ignored for 256-color sprites (no-op).
void NEA_Hw2DOBJSetPaletteSlot(NEA_Hw2DOBJ *obj, int slot);

/// Set the current animation frame (for multi-frame sprite sheets).
void NEA_Hw2DOBJSetFrame(NEA_Hw2DOBJ *obj, int frame);

/// Assign an affine transformation matrix to a sprite.
///
/// @param obj         OBJ sprite.
/// @param rot_index   Affine matrix index (0-31), or -1 to disable affine.
/// @param double_size If true, double the rendering area to avoid clipping.
void NEA_Hw2DOBJSetAffine(NEA_Hw2DOBJ *obj, int rot_index, bool double_size);

/// Set rotation and scale for an affine matrix.
///
/// @param engine    NEA_ENGINE_MAIN or NEA_ENGINE_SUB.
/// @param rot_index Affine matrix index (0-31).
/// @param angle     Rotation angle (0-511, 512 = full rotation).
/// @param sx        Horizontal scale (f32 fixed-point, 4096 = 1.0).
/// @param sy        Vertical scale (f32 fixed-point, 4096 = 1.0).
void NEA_Hw2DOBJSetRotScaleI(NEA_Hw2DEngine engine, int rot_index,
                              int angle, int32_t sx, int32_t sy);

/// Flush OAM data for a specific engine. Called internally during VBL.
void NEA_Hw2DOBJUpdate(NEA_Hw2DEngine engine);

/// Flush OAM data for all engines. Called via weak reference from
/// NEA_WaitForVBL() when NEA_UPDATE_HW2D is set.
void NEA_Hw2DOBJUpdateAll(void);

// ---------------------------------------------------------------------------
// Text rendering on bitmap backgrounds
// ---------------------------------------------------------------------------

/// Render text onto a bitmap background using a rich text font slot.
///
/// The rich text slot must have been initialized with font metadata and a
/// bitmap loaded to RAM (via NEA_RichTextBitmapLoadGRF or
/// NEA_RichTextBitmapSet). The font's texture format must match the BG type:
///   NEA_HW2D_BG_BITMAP_16 needs an A1RGB5 (NEA_A1RGB5) font;
///   NEA_HW2D_BG_BITMAP_8  needs a PAL256 (NEA_PAL256) font, and the font
///   palette must already be loaded into BG_PALETTE / BG_PALETTE_SUB.
///
/// One-shot rendering: each call lays the whole string down at (x, y) with
/// libdsf's default canvas (the full BG) and default settings (word wrap on).
/// For typewriter / stateful canvas / cursor control, use NEA_Hw2DTextCtx*.
///
/// @param bg   8bpp or 16bpp bitmap background.
/// @param slot Rich text font slot (previously set up via NEA_RichText*).
/// @param str  Null-terminated UTF-8 string to render.
/// @param x    X position in pixels on the background.
/// @param y    Y position in pixels on the background.
/// @return 0 on success, -1 on error.
int NEA_Hw2DTextRender(NEA_Hw2DBG *bg, u32 slot, const char *str,
                        int x, int y);

// ---------------------------------------------------------------------------
// Persistent text context (stateful canvas / cursor / typewriter)
// ---------------------------------------------------------------------------

/// Opaque handle for a persistent text rendering context.
typedef struct NEA_Hw2DTextCtx NEA_Hw2DTextCtx;

/// Behaviour when text reaches the right edge of the canvas.
typedef enum {
    NEA_HW2D_TEXT_RIGHT_WRAP  = 0, ///< Glyph wraps to the next line (default).
    NEA_HW2D_TEXT_RIGHT_TRIM  = 1, ///< Glyph is clipped; full overflow = full.
    NEA_HW2D_TEXT_RIGHT_ERROR = 2  ///< Any partial fit = full.
} NEA_Hw2DTextRightMode;

/// Behaviour when text reaches the bottom edge of the canvas.
typedef enum {
    NEA_HW2D_TEXT_BOTTOM_TRIM  = 0, ///< Glyph is clipped (default).
    NEA_HW2D_TEXT_BOTTOM_ERROR = 1  ///< Any partial fit = full.
} NEA_Hw2DTextBottomMode;

/// Create a persistent text context attached to a bitmap background.
///
/// The context wraps a libdsf renderer aimed at `bg->gfx_ptr` with a
/// clipping canvas. The newly-created ctx has no font yet — call
/// NEA_Hw2DTextCtxMetadataLoad* and NEA_Hw2DTextCtxBitmap* before printing.
/// Multiple contexts can target the same BG.
///
/// @note For BITMAP_8 backgrounds libdsf lays glyphs down one *byte* at a
///       time, and VRAM ignores 8-bit stores. Rendering an 8bpp ctx straight
///       into VRAM therefore drops pixels on hardware — use
///       NEA_Hw2DTextCtxCreateBuffer() + NEA_Hw2DTextCtxBlitToBG() instead.
///       BITMAP_16 backgrounds are unaffected (one halfword per pixel).
///
/// @param bg 8bpp or 16bpp bitmap BG. The font loaded afterward must match:
///           BITMAP_16 → A1RGB5 (16bpp), BITMAP_8 → PAL256 (8bpp).
/// @return Pointer to the context, or NULL on error.
NEA_Hw2DTextCtx *NEA_Hw2DTextCtxCreate(NEA_Hw2DBG *bg);

/// Create a persistent text context that renders into a caller-owned buffer.
///
/// Same as NEA_Hw2DTextCtxCreate(), except the destination is plain memory
/// rather than a background, so the byte-sized stores libdsf performs for
/// 8bpp text are legal. Compose the text, then call NEA_Hw2DTextCtxBlitToBG()
/// to move the result onto a background with VRAM-safe transfers.
///
/// The buffer must outlive the context; NEA never frees it.
///
/// @param type   NEA_HW2D_BG_BITMAP_8 or NEA_HW2D_BG_BITMAP_16. Fixes the
///               pixel size and the font format the ctx expects, exactly like
///               the BG type does for NEA_Hw2DTextCtxCreate().
/// @param buffer Destination, at least width * height * (1 or 2) bytes.
/// @param width,height Buffer dimensions in pixels.
/// @return Pointer to the context, or NULL on error.
NEA_Hw2DTextCtx *NEA_Hw2DTextCtxCreateBuffer(NEA_Hw2DBGType type, void *buffer,
                                              int width, int height);

/// Copy a buffer-backed context's pixels onto a bitmap background.
///
/// The blit is clipped against the background and uses 16/32-bit transfers,
/// so it is safe against VRAM. The whole buffer is copied, transparent pixels
/// included: it replaces the destination rectangle instead of compositing
/// over it.
///
/// @param ctx  Context created with NEA_Hw2DTextCtxCreateBuffer().
/// @param bg   Destination background. Its type must match the ctx's.
/// @param x,y  Top-left destination position, in BG pixels.
/// @return 0 on success, -1 on error (BG-backed ctx, or format mismatch).
int NEA_Hw2DTextCtxBlitToBG(NEA_Hw2DTextCtx *ctx, NEA_Hw2DBG *bg, int x, int y);

/// Release a text context.
///
/// Frees the libdsf renderer, any font handle the ctx owns (loaded via
/// NEA_Hw2DTextCtxMetadataLoad*), and any bitmap/palette buffers the ctx
/// owns (loaded via NEA_Hw2DTextCtxBitmapLoadGRF). Does not touch buffers
/// passed in via NEA_Hw2DTextCtxBitmapSet — the caller still owns those.
void NEA_Hw2DTextCtxDelete(NEA_Hw2DTextCtx *ctx);

/// Load BMFont metadata (.fnt binary) from RAM into the ctx.
///
/// The ctx takes ownership of the resulting dsf font handle and will free
/// it on Delete. Calling this again on the same ctx replaces (and frees)
/// the previous font.
int NEA_Hw2DTextCtxMetadataLoadMemory(NEA_Hw2DTextCtx *ctx,
                                       const void *data, size_t size);

/// Load BMFont metadata (.fnt binary) from the filesystem (NitroFS / FAT).
int NEA_Hw2DTextCtxMetadataLoadFAT(NEA_Hw2DTextCtx *ctx, const char *path);

/// Attach a font texture (and optional palette) from RAM to the ctx.
///
/// The format is fixed by the BG: BITMAP_16 expects an A1RGB5 texture and
/// ignores the palette; BITMAP_8 expects a PAL256 texture and copies the
/// palette into the engine's BG palette (BG_PALETTE / BG_PALETTE_SUB)
/// starting at slot 0. The caller retains ownership of these buffers.
///
/// @param texture        Font bitmap (must outlive the ctx).
/// @param width,height   Font texture dimensions in pixels.
/// @param palette        Palette data, or NULL for 16bpp BGs.
/// @param palette_bytes  Palette byte count (ignored if palette is NULL).
/// @return 0 on success, -1 on error.
int NEA_Hw2DTextCtxBitmapSet(NEA_Hw2DTextCtx *ctx,
                              const void *texture,
                              size_t width, size_t height,
                              const void *palette, size_t palette_bytes);

/// Load a font bitmap (and palette) from a GRF file into the ctx.
///
/// The GRF's gfxAttr must match the ctx's BG: 16 for BITMAP_16, 8 for
/// BITMAP_8 (which also requires the GRF to carry a palette). The ctx takes
/// ownership of the malloc'd texture/palette buffers and frees them on
/// Delete. The palette is copied into the engine's BG palette automatically
/// for 8bpp BGs. BlocksDS only.
int NEA_Hw2DTextCtxBitmapLoadGRF(NEA_Hw2DTextCtx *ctx, const char *path);

/// Set the clipping rectangle in BG-pixel coordinates. Text only writes
/// inside this box; the cursor wraps and clears within it.
int NEA_Hw2DTextCtxSetCanvas(NEA_Hw2DTextCtx *ctx, int left, int top,
                              int right, int bottom);

/// Wipe the canvas region in VRAM (writes 0 = transparent for both
/// supported formats) and reset the renderer cursor to the canvas top-left.
int NEA_Hw2DTextCtxClear(NEA_Hw2DTextCtx *ctx);

/// Configure what happens when text would overflow the canvas edges.
int NEA_Hw2DTextCtxSetOverflow(NEA_Hw2DTextCtx *ctx,
                                NEA_Hw2DTextRightMode right_mode,
                                NEA_Hw2DTextBottomMode bottom_mode);

/// Enable or disable word wrapping. Separators is a 0-terminated codepoint
/// array, or NULL for the libdsf default (space, newline, tab).
int NEA_Hw2DTextCtxSetWordWrap(NEA_Hw2DTextCtx *ctx, bool enabled,
                                const uint32_t *separators);

/// Read or write the cursor position (BG-pixel coordinates).
int NEA_Hw2DTextCtxCursorGet(NEA_Hw2DTextCtx *ctx, int *x, int *y);
int NEA_Hw2DTextCtxCursorSet(NEA_Hw2DTextCtx *ctx, int x, int y);

/// Read the bounding box that has been actually written to since the last
/// canvas clear (useful for laying out balloons / underlines / cursors).
int NEA_Hw2DTextCtxUsedBoxGet(NEA_Hw2DTextCtx *ctx,
                               int16_t *left, int16_t *top,
                               int16_t *right, int16_t *bottom);

/// Print a UTF-8 string at the current cursor.
///
/// @return 0 on success, -1 on libdsf error, +1 if the canvas filled up
///         mid-string (further glyphs were dropped). Treat +1 as a normal
///         "done" condition for typewriter loops.
int NEA_Hw2DTextCtxPrint(NEA_Hw2DTextCtx *ctx, const char *str);

/// Print up to *characters codepoints. *characters and *str are advanced
/// past the printed portion — pass the same variables back on the next call
/// to continue. Suitable for character-per-frame typewriter effects.
///
/// @return 0 on success, -1 on libdsf error, +1 if the canvas filled up.
int NEA_Hw2DTextCtxPrintLength(NEA_Hw2DTextCtx *ctx, size_t *characters,
                                const char **str);

/// Print a single codepoint at the current cursor.
int NEA_Hw2DTextCtxPrintCodepoint(NEA_Hw2DTextCtx *ctx, uint32_t codepoint);

/// Print a printf-style formatted string at the current cursor.
///
/// The formatted output is built with vasprintf and then rendered with the
/// same path as NEA_Hw2DTextCtxPrint, so word wrap / canvas / cursor /
/// overflow settings all apply.
///
/// @return 0 on success, -1 on error, +1 if the canvas filled up mid-string.
int NEA_Hw2DTextCtxPrintf(NEA_Hw2DTextCtx *ctx, const char *fmt, ...)
    __attribute__((format(printf, 2, 3)));

/// @}

#ifdef __cplusplus
}
#endif

#endif // NEA_HW2D_H__
