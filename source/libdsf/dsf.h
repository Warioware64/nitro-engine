// SPDX-License-Identifier: Zlib OR MIT
//
// Copyright (c) 2024-2026 Antonio Niño Díaz

#ifndef DSF_H__
#define DSF_H__

#ifdef __cplusplus
extern "C" {
#endif

/// @file dsf.h
///
/// @brief Global include of LibDSF.

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/// @defgroup libdsf_types LibDSF definitions and types.
/// @{

/// Major version of LibDSF (semver)
#define LIBDSF_VERSION_MAJOR 0
/// Minor version of LibDSF (semver)
#define LIBDSF_VERSION_MINOR 2
/// Patch version of LibDSF (semver)
#define LIBDSF_VERSION_PATCH 0

/// Value that combines all version numbers, useful for version checks.
#define LIBDSF_VERSION \
                ((LIBDSF_VERSION_MAJOR << 16) | \
                 (LIBDSF_VERSION_MINOR << 8) | \
                 (LIBDSF_VERSION_PATCH << 1))

/// Version string
#define LIBDSF_VERSION_STRING "0.2.0"

/// Character that is used when there are UTF-8 decoding errors.
///
/// This is returned when a character with an invalid encoding is found.
#define REPLACEMENT_CHARACTER 0xFFFD

/// List of possible errors of the library.
typedef enum {
    DSF_NO_ERROR              =  0,  ///< No error
    DSF_INVALID_ARGUMENT      = -1,  ///< Invalid argument (e.g. NULL pointer)
    DSF_BAD_MAGIC             = -2,  ///< Bad BMFont magic number
    DSF_BAD_VERSION           = -3,  ///< Bad BMFont version
    DSF_BAD_CHUNK_SIZE        = -4,  ///< The size of a BMFont chunk is wrong
    DSF_NO_MEMORY             = -5,  ///< Not enough RAM to allocate font data
    DSF_UNEXPECTED_END        = -6,  ///< File ends unexpectedly
    DSF_NO_CHARACTERS         = -7,  ///< No characters present in font
    DSF_FILE_OPEN_ERROR       = -8,  ///< Can't open the provided file
    DSF_FILE_EMPTY            = -9,  ///< The provided file is empty
    DSF_FILE_SEEK_ERROR       = -10, ///< Can't fseek() file
    DSF_FILE_READ_ERROR       = -11, ///< Can't read file
    DSF_CODEPOINT_NOT_FOUND   = -12, ///< Codepoint not present in font
    DSF_TEXTURE_TOO_BIG       = -12, ///< The texture size required is too big for the NDS
    DSF_TEXTURE_BAD_FORMAT    = -13, ///< Unsupported NDS texture format
    DSF_CANVAS_EMPTY          = -14, ///< No characters have been printed yet
    DSF_CANVAS_FULL           = -15, ///< No more characters can be printed
    DSF_FORMAT_MISMATCH       = -16, ///< Font format and renderer buffer format are different
} dsf_error;

/// List of graphics formats supported by the library.
typedef enum {
    // NDS texture formats. Values picked for maximum compatibility
    DSF_NO_FORMAT       = 0, ///< No format (GL_NOTEXTURE).
#ifdef __NDS__
    DSF_BMP_RGB32_A3    = 1, ///< Paletted format with 32 colors and 3 bits of alpha. (GL_RGB32_A3)
#endif
    DSF_BMP_RGB4        = 2, ///< Paletted format with 4 colors. (GL_RGB4)
    DSF_BMP_RGB16       = 3, ///< Paletted format with 16 colors. (GL_RGB16)
    DSF_BMP_RGB256      = 4, ///< Paletted format with 256 colors. (GL_RGB256)
    DSF_BMP_UNUSED      = 5, // Unused value. (GL_COMPRESSED)
#ifdef __NDS__
    DSF_BMP_RGB8_A5     = 6, ///< Paletted format with 8 colors and 5 bits of alpha. (GL_RGB8_A5)
#endif
    DSF_BMP_RGBA        = 7, ///< Format with 5 bits for R, G, B and 1 bit of alpha. (GL_RGBA)
    DSF_BMP_RGB         = 8, ///< Format with 5 bits for R, G, B. (GL_RGB)

    DSF_FORMAT_END,
} dsf_format;

/// Type that represents a DSF font internal state.
typedef uintptr_t dsf_handle;

/// Type that represents a DSF renderer internal state.
typedef uintptr_t dsf_renderer;

/// @}
/// @defgroup libdsf_utf8 UTF-8 utilities.
/// @{

/// This function returns the first codepoint in the provided UTF-8 string.
///
/// On error this function returns codepoint REPLACEMENT_CHARACTER and a number
/// of bytes that should be enough to skip the codepoint encoded incorrectly. It
/// isn't guaranteed that the next codepoint will be valid, though.
///
/// @param str
///     String in UTF-8 format.
/// @param codepoint
///     Location to store the resulting codepoint.
///
/// @return
///     Number of bytes to advance for the next codepoint.
size_t DSF_UTF8_CodepointRead(const char *str, uint32_t *codepoint);

/// This function returns the length of a UTF-8 string in codepoints.
///
/// @param str
///     String in UTF-8 format.
///
/// @return
///     Number of codepoints in the string.
size_t DSF_UTF8_StringLength(const char *str);

/// This returns true if the string begins with one of the given codepoints.
///
/// @param str
///     String in UTF-8 format.
/// @param codepoints
///     Array of codepoints that ends with 0.
///
/// @return
///     It returns true if the first codepoint in the string is one of the
///     codepoints in the array.
bool DSF_UTF8_StringStartsWith(const char *str, const uint32_t codepoints[]);

/// Calculates the number of characters in a UTF-8 string until a separator.
///
/// You can assign your own list of separators, or pass NULL to use the default
/// separators (' ', '\n', '\t').
///
/// ```c
/// const char *text = "Hello world!";
/// const uint32_t separators[] = { ' ', '\n', '\t', 0 };
/// size_t len = DSF_UTF8_WordLength(text, separators);
/// ```
///
/// @param str
///     String in UTF-8 format.
/// @param separators
///     Array of separators that define the end of a word, or NULL to use the
///     default separators.
///
/// @return
///     Number of codepoints in the word.
size_t DSF_UTF8_WordLength(const char *str, const uint32_t separators[]);

/// @}
/// @defgroup libdsf_load_unload Font loading and unloading functions.
/// @{

/// This function loads a BMFont in binary format from RAM.
///
/// @param handle
///     Pointer to a handle that will be configured for this font.
/// @param data
///     Pointer to the font data (".fnt" file).
/// @param data_size
///     Size of the font data.
///
/// @return
///     An error code or DSF_NO_ERROR on success.
dsf_error DSF_LoadFontMemory(dsf_handle *handle,
                             const void *data, int32_t data_size);

/// This function loads a BMFont in binary format from the filesystem.
///
/// @param handle
///     Pointer to a handle that will be configured for this font.
/// @param path
///     Path to the ".fnt" file in the filesystem.
///
/// @return
///     An error code or DSF_NO_ERROR on success.
dsf_error DSF_LoadFontFilesystem(dsf_handle *handle, const char *path);

/// Free all memory used by this font.
///
/// It also invalidates the provided handler.
///
/// @param handle
///     Pointer to the font handler.
///
/// @return
///     An error code or DSF_NO_ERROR on success.
dsf_error DSF_FreeFont(dsf_handle *handle);

/// @}
/// @defgroup libdsf_simplified_api Simplified functions to draw text.
/// @{

/// Pretends to print text and determines its size and where the cursor ends.
///
/// @param handle
///     Handler of the font to use.
/// @param str
///     String to print.
/// @param size_x
///     Pointer to a variable to store the size.
/// @param size_y
///     Pointer to a variable to store the size.
/// @param final_x
///     Pointer to a variable to store the final X cursor position.
/// @param final_y
///     Pointer to a variable to store the final Y cursor position.
///
/// @return
///     An error code or DSF_NO_ERROR on success.
dsf_error DSF_StringRenderDryRunWithCursor(dsf_handle handle, const char *str,
                                 size_t *size_x, size_t *size_y,
                                 size_t *final_x, size_t *final_y);

/// Pretend to render a string to calculate its final size once rendered.
///
/// @param handle
///     Handler of the font to use.
/// @param str
///     String to print.
/// @param size_x
///     Pointer to a variable to store the size.
/// @param size_y
///     Pointer to a variable to store the size.
///
/// @return
///     An error code or DSF_NO_ERROR on success.
dsf_error DSF_StringRenderDryRun(dsf_handle handle, const char *str,
                                 size_t *size_x, size_t *size_y);

#ifdef __NDS__

/// Render a string by rendering one 3D quad per codepoint.
///
/// @param handle
///     Handler of the font to use.
/// @param str
///     String to print.
/// @param x
///     Top x coordinate (0 to 255, but you can go outside of that).
/// @param y
///     Left y coordinate (0 to 191, but you can go outside of that).
/// @param z
///     Z coordinate (depth).
/// @param xStart
///     The horizontal component of the cursor's starting offset (reset to x on
///     line break)
///
/// @return
///     An error code or DSF_NO_ERROR on success.
dsf_error DSF_StringRender3DWithIndent(dsf_handle handle, const char *str,
                                       int32_t x, int32_t y, int32_t z,
                                       int32_t xStart);

/// Render a string by rendering one 3D quad per codepoint.
///
/// @param handle
///     Handler of the font to use.
/// @param str
///     String to print.
/// @param x
///     Top x coordinate (0 to 255, but you can go outside of that).
/// @param y
///     Left y coordinate (0 to 191, but you can go outside of that).
/// @param z
///     Z coordinate (depth).
///
/// @return
///     An error code or DSF_NO_ERROR on success.
dsf_error DSF_StringRender3D(dsf_handle handle, const char *str,
                             int32_t x, int32_t y, int32_t z);

/// Render a string by rendering one 3D quad per codepoint with alternating
/// polygon IDs.
///
/// This function will alternate between polygon IDs so that alpha blending
/// works between multiple polygons when they overlap. This is a requirement of
/// the NDS 3D hardware.
///
/// It is required to pass the base polygon format as a parameter because the
/// polygon format data is write-only. Whenever the polygon ID needs to be
/// changed, the rest of the polygon format flags need to be set as well.
///
/// @param handle
///     Handler of the font to use.
/// @param str
///     String to print.
/// @param x
///     Top x coordinate (0 to 255, but you can go outside of that).
/// @param y
///     Left y coordinate (0 to 191, but you can go outside of that).
/// @param z
///     Z coordinate (depth).
/// @param poly_fmt
///     Polygon formats to apply to the characters.
/// @param poly_id_base
///     `poly_id_base` and `poly_id_base ^ 1` will be used.
/// @param xStart
///     The horizontal component of the cursor's starting offset (reset to x on
///     line break)
///
/// @return
///     An error code or DSF_NO_ERROR on success.
dsf_error DSF_StringRender3DAlphaWithIndent(dsf_handle handle, const char *str,
                                            int32_t x, int32_t y, int32_t z,
                                            uint32_t poly_fmt, int poly_id_base,
                                            int32_t xStart);

/// Render a string by rendering one 3D quad per codepoint with alternating
/// polygon IDs.
///
/// This function will alternate between polygon IDs so that alpha blending
/// works between multiple polygons when they overlap. This is a requirement of
/// the NDS 3D hardware.
///
/// It is required to pass the base polygon format as a parameter because the
/// polygon format data is write-only. Whenever the polygon ID needs to be
/// changed, the rest of the polygon format flags need to be set as well.
///
/// @param handle
///     Handler of the font to use.
/// @param str
///     String to print.
/// @param x
///     Top x coordinate (0 to 255, but you can go outside of that).
/// @param y
///     Left y coordinate (0 to 191, but you can go outside of that).
/// @param z
///     Z coordinate (depth).
/// @param poly_fmt
///     Polygon formats to apply to the characters.
/// @param poly_id_base
///     `poly_id_base` and `poly_id_base ^ 1` will be used.
///
/// @return
///     An error code or DSF_NO_ERROR on success.
dsf_error DSF_StringRender3DAlpha(dsf_handle handle, const char *str,
                                  int32_t x, int32_t y, int32_t z,
                                  uint32_t poly_fmt, int poly_id_base);

#endif // __NDS__

/// Renders the provided string to a pre-existing buffer.
///
/// This function is VRAM-safe. You can use it to write directly to VRAM.
///
/// @param handle
///     Handler of the font to use.
/// @param str
///     String to print.
/// @param texture_fmt
///     Texture format (normally a dsf_format value, but it can be a
///     GL_TEXTURE_TYPE_ENUM or NE_TextureFormat value).
/// @param font_texture
///     Pointer to the font texture.
/// @param font_width
///     Width of the font texture.
/// @param font_height
///     Height of the font texture.
/// @param out_texture
///     The pointer to the new buffer is returned here.
/// @param out_width
///     The width to the new buffer is returned here.
/// @param out_height
///     The height to the new buffer is returned here.
/// @param out_x
///     X offset to draw the text inside the buffer.
/// @param out_y
///     Y offset to draw the text inside the buffer.
///
/// @return
///     An error code or DSF_NO_ERROR on success.
dsf_error DSF_StringRenderToExistingBuffer(dsf_handle handle,
                    const char *str, dsf_format texture_fmt,
                    const void *font_texture, size_t font_width, size_t font_height,
                    void *out_texture, size_t out_width, size_t out_height,
                    uint32_t out_x, uint32_t out_y);

/// Allocates a buffer and renders the provided string to that buffer.
///
/// This function takes a bitmap stored in RAM with the desired font. It will
/// calculate the final size of the text it has to print, and it will allocate
/// enough space for it. Note that NDS texture sizes must be powers of two, so
/// the end result may be bigger than the actual text. After allocating the
/// buffer, it will render the string to that buffer.
///
/// The buffer must be freed with free() when it is no longer needed.
///
/// This function won't load the texture to VRAM. The returned buffer needs to
/// be loaded to texture VRAM to be used (with functions like glTexImage2D() or
/// NE_MaterialTexLoad()).
///
/// Also, note that this texture doesn't need to have a size that is a power of
/// two. However, consider that the size of a row should at least be a multiple
/// of a full pixel (for example, for a 16 color texture, don't use a texture
/// with width of 143 bytes because the last pixel won't be full). To be sure
/// that you never find any issue, ensure that your textures have a width
/// multiple of 4 pixels, that will work with all texture formats.
///
/// @param handle
///     Handler of the font to use.
/// @param str
///     String to print.
/// @param texture_fmt
///     Texture format (normally a dsf_format value, but it can be a
///     GL_TEXTURE_TYPE_ENUM or NE_TextureFormat value).
/// @param font_texture
///     Pointer to the font texture.
/// @param font_width
///     Width of the font texture.
/// @param font_height
///     Height of the font texture.
/// @param out_texture
///     The pointer to the new buffer is returned here.
/// @param out_width
///     The width to the new buffer is returned here.
/// @param out_height
///     The height to the new buffer is returned here.
///
/// @return
///     An error code or DSF_NO_ERROR on success.
dsf_error DSF_StringRenderToNewBuffer(dsf_handle handle,
                    const char *str, dsf_format texture_fmt,
                    const void *font_texture, size_t font_width, size_t font_height,
                    void **out_texture, size_t *out_width, size_t *out_height);

// This name is preserved for backwards compatibility.
static inline
dsf_error DSF_StringRenderToTexture(dsf_handle handle,
                    const char *str, dsf_format texture_fmt,
                    const void *font_texture, size_t font_width, size_t font_height,
                    void **out_texture, size_t *out_width, size_t *out_height)
{
    return DSF_StringRenderToNewBuffer(handle, str, texture_fmt,
                    font_texture, font_width, font_height,
                    out_texture, out_width, out_height);
}

/// @}
/// @defgroup libdsf_render_base Low level rendering functions.
/// @{

/// Assigns a font texture to a dsf_handle to be used when rendering to buffers.
///
/// @param handle
///     Handler of the font to modify.
/// @param font_texture
///     Pointer to the font texture.
/// @param font_width
///     Width of the font texture.
/// @param font_height
///     Height of the font texture.
/// @param font_format
///     Texture format (normally a dsf_format value, but it can be a
///     GL_TEXTURE_TYPE_ENUM or NE_TextureFormat value).
///
/// @return
///     An error code or DSF_NO_ERROR on success.
dsf_error DSF_FontTextureSet(dsf_handle handle, const void *font_texture,
                    size_t font_width, size_t font_height, dsf_format font_format);

/// Allocates a new renderer.
///
/// The renderer starts in dry run mode.
///
/// @param renderer
///     Pointer to a dsf_renderer to contain the new renderer.
///
/// @return
///     An error code or DSF_NO_ERROR on success.
dsf_error DSF_RendererNew(dsf_renderer *renderer);

/// Sets a renderer to its initial settings.
///
/// This function is called internally by DSF_RendererNew().
///
/// @param renderer
///     Pointer to a dsf_renderer to reset.
///
/// @return
///     An error code or DSF_NO_ERROR on success.
dsf_error DSF_RendererReset(dsf_renderer renderer);

/// Copy the settings of a renderer into another renderer.
///
/// @param destination
///     Renderer that receives the settings.
/// @param source
///     Renderer that acts as origin of the settings.
///
/// @return
///     An error code or DSF_NO_ERROR on success.
dsf_error DSF_RendererCopy(dsf_renderer destination, dsf_renderer source);

/// Frees a renderer.
///
/// @param renderer
///     Renderer to be freed.
///
/// @return
///     An error code or DSF_NO_ERROR on success.
dsf_error DSF_RendererFree(dsf_renderer renderer);

/// Setup a renderer to dry run mode.
///
/// @param renderer
///     Renderer to be setup.
///
/// @return
///     An error code or DSF_NO_ERROR on success.
dsf_error DSF_RendererModeSetNone(dsf_renderer renderer);

/// Setup a renderer to render to a buffer in memory.
///
/// This mode is VRAM-safe.
///
/// It also adjusts the size of the canvas to prevent writing out of bounds.
///
/// @param renderer
///     Renderer to be setup.
/// @param out_format
///     Format of the buffer.
/// @param out_texture
///     Pointer to the buffer.
/// @param out_width
///     Width of the buffer.
/// @param out_height
///     Height of the buffer.
///
/// @return
///     An error code or DSF_NO_ERROR on success.
dsf_error DSF_RendererModeSetBuffer(dsf_renderer renderer, dsf_format out_format,
                    void *out_texture, size_t out_width, size_t out_height);

#ifdef __NDS__

/// Setup a renderer to render using 3D quads.
///
/// @param renderer
///     Renderer to be setup.
/// @param z
///     Z coordinate (depth).
///
/// @return
///     An error code or DSF_NO_ERROR on success.
dsf_error DSF_RendererModeSet3D(dsf_renderer renderer, int32_t z);

/// Setup a renderer to render using 3D quads.
///
/// @param renderer
///     Renderer to be setup.
/// @param z
///     Z coordinate (depth).
/// @param poly_fmt
///     Polygon formats to apply to the characters.
/// @param poly_id_base
///     `poly_id_base` and `poly_id_base ^ 1` will be used.
///
/// @return
///     An error code or DSF_NO_ERROR on success.
dsf_error DSF_RendererModeSet3DAlpha(dsf_renderer renderer, int32_t z,
                    uint32_t poly_fmt, int poly_id_base);

#endif // __NDS__

/// Setup a renderer to render only inside a specific box.
///
/// @param renderer
///     Renderer to be setup.
/// @param box_left
///     Left coordinate.
/// @param box_top
///     Top coordinate.
/// @param box_right
///     Right coordinate.
/// @param box_bottom
///     Bottom coordinate.
///
/// @return
///     An error code or DSF_NO_ERROR on success.
dsf_error DSF_RendererCanvasSetup(dsf_renderer renderer, int16_t box_left,
                    int16_t box_top, int16_t box_right, int16_t box_bottom);

/// Clears canvas of a renderer in buffer mode.
///
/// It also sets the cursor to the top-left corner of the canvas, and resets the
/// used box dimensions.
///
/// @param renderer
///     Renderer to be cleared.
///
/// @return
///     An error code or DSF_NO_ERROR on success.
dsf_error DSF_RendererCanvasClear(dsf_renderer renderer);

/// Options for the renderer when it reaches the right edge of the canvas.
typedef enum {
    /// Don't print partial glyphs, jump to the next line. Default behaviour.
    DSF_RENDER_RIGHT_WRAP = 0,
    /// If a glyph doesn't fully fit, it is trimmed. If a glyph is completely
    /// outside, it causes a DSF_CANVAS_FULL error.
    DSF_RENDER_RIGHT_TRIM = 1,
    /// If a glyph doesn't fully fit it causes a DSF_CANVAS_FULL error.
    DSF_RENDER_RIGHT_ERROR = 2,
} dsf_render_right_mode;

/// Options for the renderer when it reaches the bottom edge of the canvas.
typedef enum {
    /// If a glyph doesn't fully fit, it is trimmed. If a glyph is completely
    /// outside, it causes a DSF_CANVAS_FULL error. Default behaviour.
    DSF_RENDER_BOTTOM_TRIM = 0,
    /// If a glyph doesn't fully fit it causes a DSF_CANVAS_FULL error.
    DSF_RENDER_BOTTOM_ERROR = 1,
} dsf_render_bottom_mode;

/// Configures the behaviour of the renderer when the text overflows the canvas.
///
/// @param renderer
///     Renderer.
/// @param right_mode
///     Action when text reaches the right edge of the canvas.
/// @param bottom_mode
///     Action when text reaches the bottom edge of the canvas.
///
/// @return
///     An error code or DSF_NO_ERROR on success.
dsf_error DSF_RendererOverflowModeSet(dsf_renderer renderer,
                dsf_render_right_mode right_mode, dsf_render_bottom_mode bottom_mode);

/// Maximum number of separators that can be remembered by a renderer.
#define DSF_RENDERER_MAX_SEPARATORS 10

/// This enables or disables word wrapping.
///
/// When word wrapping is enabled, the renderer will try to avoid splitting
/// words into multiple lines. If a word doesn't fit in the current line, it
/// will jump to the next line before printing it.
///
/// When it's disabled, it will print characters until it reaches the end of the
/// line, and then it will use the settings of DSF_RendererOverflowModeSet().
///
/// @note
///     It only affects DSF_StringRender() and DSF_StringRenderFormat(). It
///     doesn't affect DSF_StringRenderLength() because it only has incomplete
///     information about the string, so it can't predict if a word will fit or
///     not.
///
/// @param renderer
///     Renderer.
/// @param enabled
///     Enables or disables word wrapping.
/// @param separators
///     Array of codepoints to use as separator.
///
/// @return
///     An error code or DSF_NO_ERROR on success.
dsf_error DSF_RendererWordWrapModeSet(dsf_renderer renderer, bool enabled,
                const uint32_t separators[]);

/// Gets the current coordinates of the cursor.
///
/// @param renderer
///     Renderer.
/// @param x
///     Pointer to save the current X coordinate.
/// @param y
///     Pointer to save the current Y coordinate.
///
/// @return
///     An error code or DSF_NO_ERROR on success.
dsf_error DSF_RendererCursorGet(dsf_renderer renderer, int *x, int *y);

/// Sets the current coordinates of the cursor.
///
/// @param renderer
///     Renderer.
/// @param x
///     New X coordinate.
/// @param y
///     New Y coordinate.
///
/// @return
///     An error code or DSF_NO_ERROR on success.
dsf_error DSF_RendererCursorSet(dsf_renderer renderer, int16_t x, int16_t y);

/// Clears the current recorded bounds of the printed text.
///
/// @param renderer
///     Renderer.
///
/// @return
///     An error code or DSF_NO_ERROR on success.
dsf_error DSF_RendererUsedBoxReset(dsf_renderer renderer);

/// Gets the current recorded bounds of the printed text.
///
/// @param renderer
///     Renderer.
/// @param left
///     Pointer to store the left coordinate.
/// @param top
///     Pointer to store the top coordinate.
/// @param right
///     Pointer to store the right coordinate.
/// @param bottom
///     Pointer to store the bottom coordinate.
///
/// @return
///     An error code or DSF_NO_ERROR on success.
dsf_error DSF_RendererUsedBoxGet(dsf_renderer renderer, int16_t *left,
                    int16_t *top, int16_t *right, int16_t *bottom);

/// Gets the current recorded bounds of the printed text as a safe size for DS
/// textures.
///
/// @param renderer
///     Renderer.
/// @param width
///     Pointer to store the width (forced to a power of two).
/// @param height
///     Pointer to store the height (may not be a power of two).
///
/// @return
///     An error code or DSF_NO_ERROR on success.
dsf_error DSF_RendererUsedBoxGetTextureSize(dsf_renderer renderer,
                    size_t *width, size_t *height);

/// Allocates a buffer for the specified size and the format in a dsf_handle.
///
/// The buffer must be freed with free() when it is no longer needed.
///
/// @param handle
///     Handler of the font to use.
/// @param buffer
///     Pointer to store the address of the new buffer.
/// @param tex_width
///     Requested buffer width.
/// @param tex_height
///     Requested buffer height.
///
/// @return
///     An error code or DSF_NO_ERROR on success.
dsf_error DSF_BufferAlloc(dsf_handle handle, void **buffer, size_t tex_width,
                    size_t tex_height);

/// Renders the requested codepoint.
///
/// @warning
///     If you're using mode DSF_RendererModeSet3D() you need to do
///     glBegin(GL_QUADS) before calling this function, and glEnd() when it
///     returns. You will also need to call glPolyFmt() before glBegin() to set
///     the right attributes for your polygons. This is required to avoid
///     sending glBegin() and glEnd() for every polygon. However,
///     DSF_RendererModeSet3DAlpha() needs to send the begin/end commands
///     internally, so it isn't required in that case.
///
/// @param handle
///     Handler of the font to use.
/// @param renderer
///     Renderer.
/// @param codepoint
///     Codepoint to render.
///
/// @return
///     An error code or DSF_NO_ERROR on success.
dsf_error DSF_CodepointRender(dsf_handle handle, dsf_renderer renderer,
                    uint32_t codepoint);

/// Renders the requested string.
///
/// @param handle
///     Handler of the font to use.
/// @param renderer
///     Renderer.
/// @param str
///     String to print.
///
/// @return
///     An error code or DSF_NO_ERROR on success.
dsf_error DSF_StringRender(dsf_handle handle, dsf_renderer renderer,
                    const char *str);

/// Renders the requested string with printf-like formatting.
///
/// @param handle
///     Handler of the font to use.
/// @param renderer
///     Renderer.
/// @param format
///     String with formatting to print.
/// @param ...
///     Additional arguments.
///
/// @return
///     An error code or DSF_NO_ERROR on success.
__attribute__((format(printf, 3, 4)))
dsf_error DSF_StringRenderFormat(dsf_handle handle, dsf_renderer renderer,
                    const char *format, ...);

/// Renders the requested string up to the requested number of codepoints.
///
/// This function expects the string and the character number as pointers
/// because it updates the values before it returns. For example:
///
/// ```c
/// const char *string = "My string";
///
/// size_t characters = 4;
/// const char *print_pointer = string;
///
/// DSF_StringRenderLength(handle, renderer, &characters, &print_pointer);
///
/// // Now:
/// //
/// // - characters = 0
/// // - print_pointer = "tring"
/// ```
///
/// This behaviour is useful if you want to print text to a buffer
/// incrementally over several frames. You can specify the number of codepoints
/// to write per frame and simply keep calling this function with the char
/// pointer updated during the last call to this function.
///
/// @warning This function ignores the word wrapping mode set with
/// DSF_RendererWordWrapModeSet().
///
/// @param handle
///     Handler of the font to use.
/// @param renderer
///     Renderer.
/// @param characters
///     Number of codepoints to print.
/// @param str
///     String to print.
///
/// @return
///     An error code or DSF_NO_ERROR on success.
dsf_error DSF_StringRenderLength(dsf_handle handle, dsf_renderer renderer,
                    size_t *characters, const char **str);

/// @}

#ifdef __cplusplus
}
#endif

#endif // DSF_H__
