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
} NEA_Hw2DOBJ;

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

/// Set sprite screen position.
void NEA_Hw2DOBJSetPos(NEA_Hw2DOBJ *obj, int x, int y);

/// Show or hide a sprite.
void NEA_Hw2DOBJSetVisible(NEA_Hw2DOBJ *obj, bool visible);

/// Set sprite horizontal and vertical flip.
void NEA_Hw2DOBJSetFlip(NEA_Hw2DOBJ *obj, bool hflip, bool vflip);

/// Set sprite draw priority (0 = highest, 3 = lowest).
void NEA_Hw2DOBJSetPriority(NEA_Hw2DOBJ *obj, int priority);

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

/// Render text onto a 16bpp bitmap background using a rich text font slot.
///
/// The rich text slot must have been initialized with font metadata and a
/// bitmap loaded to RAM (via NEA_RichTextBitmapLoadGRF or
/// NEA_RichTextBitmapSet). The font must use GL_RGBA (A1RGB5) format for
/// correct rendering on 16bpp bitmap backgrounds.
///
/// @param bg   16bpp bitmap background (NEA_HW2D_BG_BITMAP_16).
/// @param slot Rich text font slot (previously set up via NEA_RichText*).
/// @param str  Null-terminated UTF-8 string to render.
/// @param x    X position in pixels on the background.
/// @param y    Y position in pixels on the background.
/// @return 0 on success, -1 on error.
int NEA_Hw2DTextRender(NEA_Hw2DBG *bg, u32 slot, const char *str,
                        int x, int y);

/// @}

#ifdef __cplusplus
}
#endif

#endif // NEA_HW2D_H__
