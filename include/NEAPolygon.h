// SPDX-License-Identifier: MIT
//
// Copyright (c) 2008-2022 Antonio Niño Díaz
//
// This file is part of Nitro Engine Advanced

#ifndef NEA_POLYGON_H__
#define NEA_POLYGON_H__

#include "NEAMain.h"

/// @file   NEAPolygon.h
/// @brief  Functions to draw polygons and more...

/// @defgroup other_functions Other functions
///
/// Some functions to set lights and its properties, to draw polygons, configure
/// the rear plane, etc...
///
/// @{

/// Predefined colors.
typedef enum {
    NEA_Brown     = RGB15(10, 6, 1),     ///<  Brown
    NEA_Red       = RGB15(31, 0, 0),     ///<  Red
    NEA_Orange    = RGB15(31, 20, 0),    ///<  Orange
    NEA_Yellow    = RGB15(31, 31, 0),    ///<  Yellow
    NEA_LimeGreen = RGB15(15, 31, 0),    ///<  Lime green
    NEA_Green     = RGB15(0, 31, 0),     ///<  Green
    NEA_DarkGreen = RGB15(0, 15, 0),     ///<  Dark green
    NEA_LightBlue = RGB15(7, 15, 31),    ///<  Light blue
    NEA_Blue      = RGB15(0, 0, 31),     ///<  Blue
    NEA_DarkBlue  = RGB15(0, 6, 15),     ///<  Dark blue
    NEA_Violet    = RGB15(28, 8, 28),    ///<  Violet
    NEA_Pink      = RGB15(31, 15, 22),   ///<  Pink
    NEA_Purple    = RGB15(20, 4, 14),    ///<  Purple
    NEA_Indigo    = RGB15(15, 15, 30),   ///<  Purple
    NEA_Magenta   = RGB15(31, 0, 31),    ///<  Magenta
    NEA_White     = RGB15(31, 31, 31),   ///<  White
    NEA_Gray      = RGB15(20, 20, 20),   ///<  Gray
    NEA_DarkGray  = RGB15(10, 10, 10),   ///<  Dark gray
    NEA_Black     = RGB15(0, 0, 0)       ///<  Black
} NEA_ColorEnum;

/// Supported texture formats
typedef enum {
    NEA_A3PAL32    = 1, ///< 32 color palette, 3 bits of alpha
    NEA_PAL4       = 2, ///< 4 color palette
    NEA_PAL16      = 3, ///< 16 color palette
    NEA_PAL256     = 4, ///< 256 color palette
    NEA_COMPRESSED = 5, ///< @deprecated 4x4 compressed format (compatibilty name)
    NEA_TEX4X4     = 5, ///< 4x4 compressed format
    NEA_A5PAL8     = 6, ///< 8 color palette, 5 bits of alpha
    NEA_A1RGB5     = 7, ///< Direct color (5 bits per channel), 1 bit of alpha
    NEA_RGB5       = 8  ///< @deprecated Like NEA_A1RGB5, but sets alpha to 1 when loading
} NEA_TextureFormat;

/// Switch off a light.
///
/// @param index Index of the light to switch off (0 - 3).
void NEA_LightOff(int index);

/// Switch on a light and define its color.
///
/// @param index Index of the light to switch on (0 - 3).
/// @param color Color of the light.
/// @param x (x, y, z) Vector of the light (v10).
/// @param y (x, y, z) Vector of the light (v10).
/// @param z (x, y, z) Vector of the light (v10).
void NEA_LightSetI(int index, u32 color, int x, int y, int z);

/// Switch on a light and define its color.
///
/// @param i Index of the light to switch on (0 - 3).
/// @param c Color of the light.
/// @param x (x, y, z) Vector of the light (float).
/// @param y (x, y, z) Vector of the light (float).
/// @param z (x, y, z) Vector of the light (float).
#define NEA_LightSet(i, c, x, y, z) \
    NEA_LightSetI(i ,c, floattov10(x), floattov10(y), floattov10(z))

/// Sets the color of a light.
///
/// @param index Index of the light (0 - 3).
/// @param color Color.
void NEA_LightSetColor(int index, u32 color);

/// Types of functions used to generate a shininess table.
typedef enum {
    NEA_SHININESS_NONE,      ///< Fill table with zeroes
    NEA_SHININESS_LINEAR,    ///< Increase values linearly
    NEA_SHININESS_QUADRATIC, ///< Increase values proportionaly to x^2
    NEA_SHININESS_CUBIC,     ///< Increase values proportionaly to x^3
    NEA_SHININESS_QUARTIC,   ///< Increase values proportionaly to x^4

    /// Quantize the linear ramp into a few flat levels. The specular highlight
    /// becomes a set of hard-edged bands instead of a smooth blob, which is the
    /// specular counterpart of the cel ramps built by NEA_ToonTableBands().
    NEA_SHININESS_STEPPED,

    /// Zero until near the end of the table, then a hard jump to full. Produces
    /// the sharp, clipped highlight used by anime-styled shading, and reads as a
    /// rim light on curved surfaces.
    NEA_SHININESS_THRESHOLD
} NEA_ShininessFunction;

/// Number of entries in the hardware shininess table.
#define NEA_SHININESS_TABLE_SIZE 128

/// Generate and load a shininess table used for specular lighting.
///
/// @param function The name of the function used to generate the table.
void NEA_ShininessTableGenerate(NEA_ShininessFunction function);

/// Upload a custom shininess table used for specular lighting.
///
/// The table is indexed by the cosine of the angle between the half vector and
/// the normal, so entry 0 is the grazing angle and entry 127 is a highlight
/// pointing straight at the viewer. Any shape may be uploaded; the generators
/// in NEA_ShininessFunction are only the common ones.
///
/// Costs nothing per frame: the 128 bytes are pushed to the GPU once, as 32
/// words. Uses no VRAM.
///
/// @param table Table of NEA_SHININESS_TABLE_SIZE (128) entries.
void NEA_ShininessTableSet(const u8 *table);

/// Begins a polygon.
///
/// @param mode Type of polygon to draw (GL_TRIANGLE, GL_QUAD...).
static inline void NEA_PolyBegin(int mode)
{
    GFX_BEGIN = mode;
}

/// Stops drawing polygons.
static inline void NEA_PolyEnd(void)
{
    GFX_END = 0;
}

/// Sets the color for the following vertices.
///
/// @param color Color.
static inline void NEA_PolyColor(u32 color)
{
    GFX_COLOR = color;
}

/// Set the normal vector for next group of vertices.
///
/// @param x (x, y, z) Unit vector (v10).
/// @param y (x, y, z) Unit vector (v10).
/// @param z (x, y, z) Unit vector (v10).
static inline void NEA_PolyNormalI(int x, int y, int z)
{
    GFX_NORMAL = NORMAL_PACK(x, y, z);
}

/// Set the normal vector for next group of vertices.
///
/// @param x (x, y, z) Unit vector (float).
/// @param y (x, y, z) Unit vector (float).
/// @param z (x, y, z) Unit vector (float).
#define NEA_PolyNormal(x, y, z) \
    NEA_PolyNormalI(floattov10(x), floattov10(y), floattov10(z))

/// Send vertex to the GPU.
///
/// @param x (x, y, z) Vertex coordinates (v16).
/// @param y (x, y, z) Vertex coordinates (v16).
/// @param z (x, y, z) Vertex coordinates (v16).
static inline void NEA_PolyVertexI(int x, int y, int z)
{
    GFX_VERTEX16 = (y << 16) | (x & 0xFFFF);
    GFX_VERTEX16 = (uint32_t)(uint16_t)z;
}

/// Send vertex to the GPU.
///
/// @param x (x, y, z) Vertex coordinates (float).
/// @param y (x, y, z) Vertex coordinates (float).
/// @param z (x, y, z) Vertex coordinates (float).
#define NEA_PolyVertex(x, y, z) \
    NEA_PolyVertexI(floattov16(x), floattov16(y), floattov16(z))

/// Set texture coordinates.
///
/// "When texture mapping, the Geometry Engine works faster if you issue commands
/// in the order TexCoord -> Normal -> Vertex."
///
/// https://problemkaputt.de/gbatek.htm#ds3dtextureattributes
///
/// @param u (u, v) Texture coordinates (0 - texturesize).
/// @param v (u, v) Texture coordinates (0 - texturesize).
static inline void NEA_PolyTexCoord(int u, int v)
{
    GFX_TEX_COORD = TEXTURE_PACK(inttot16(u), inttot16(v));
}

/// Flags for NEA_PolyFormat() to enable lights.
typedef enum {
    NEA_LIGHT_0 = (1 << 0), ///< Light 0
    NEA_LIGHT_1 = (1 << 1), ///< Light 1
    NEA_LIGHT_2 = (1 << 2), ///< Light 2
    NEA_LIGHT_3 = (1 << 3), ///< Light 3

    NEA_LIGHT_01 = NEA_LIGHT_0 | NEA_LIGHT_1, ///< Lights 0 and 1
    NEA_LIGHT_02 = NEA_LIGHT_0 | NEA_LIGHT_2, ///< Lights 0 and 2
    NEA_LIGHT_03 = NEA_LIGHT_0 | NEA_LIGHT_3, ///< Lights 0 and 3
    NEA_LIGHT_12 = NEA_LIGHT_1 | NEA_LIGHT_2, ///< Lights 1 and 2
    NEA_LIGHT_13 = NEA_LIGHT_1 | NEA_LIGHT_3, ///< Lights 1 and 3
    NEA_LIGHT_23 = NEA_LIGHT_2 | NEA_LIGHT_3, ///< Lights 2 and 3

    NEA_LIGHT_012 = NEA_LIGHT_0 | NEA_LIGHT_1 | NEA_LIGHT_2, ///< Lights 0, 1 and 2
    NEA_LIGHT_013 = NEA_LIGHT_0 | NEA_LIGHT_1 | NEA_LIGHT_3, ///< Lights 0, 1 and 3
    NEA_LIGHT_023 = NEA_LIGHT_0 | NEA_LIGHT_2 | NEA_LIGHT_3, ///< Lights 0, 2 and 3
    NEA_LIGHT_123 = NEA_LIGHT_1 | NEA_LIGHT_2 | NEA_LIGHT_3, ///< Lights 1, 2 and 3

    NEA_LIGHT_0123 = NEA_LIGHT_0 | NEA_LIGHT_1 | NEA_LIGHT_2 | NEA_LIGHT_3, ///< All lights

    NEA_LIGHT_ALL = NEA_LIGHT_0123 ///< All lights
} NEA_LightEnum;

/// Flags for NEA_PolyFormat() to specify the type of culling.
typedef enum {
    NEA_CULL_FRONT = (1 << 6), ///< Don't draw polygons looking at the camera
    NEA_CULL_BACK  = (2 << 6), ///< Don't draw polygons not looking at the camera
    NEA_CULL_NONE  = (3 << 6)  ///< Draw all polygons
} NEA_CullingEnum;

/// Miscellaneous flags used in NEA_PolyFormat().
typedef enum {
    NEA_MODULATION             = (0 << 4), ///< Modulation (normal) shading
    NEA_DECAL                  = (1 << 4), ///< Decal
    NEA_TOON_HIGHLIGHT_SHADING = (2 << 4), ///< Toon or highlight shading
    NEA_SHADOW_POLYGONS        = (3 << 4), ///< Shadow polygons

    NEA_TRANS_DEPTH_KEEP   = (0 << 11), ///< Keep old depth for translucent pixels
    NEA_TRANS_DEPTH_UPDATE = (1 << 11), ///< Set new depth for translucent pixels

    NEA_HIDE_FAR_CLIPPED   = (0 << 12), ///< Hide far-plane intersecting polys
    NEA_RENDER_FAR_CLIPPED = (1 << 12), ///< Draw far-plane intersecting polys

    NEA_HIDE_ONEA_DOT_POLYS   = (0 << 13), ///< Hide 1-dot polygons behind DISP_1DOT_DEPTH
    NEA_RENDER_ONEA_DOT_POLYS = (1 << 13), ///< Draw 1-dot polygons behind DISP_1DOT_DEPTH

    NEA_DEPTH_TEST_LESS  = (0 << 14), ///< Depth Test: draw pixels with less depth
    NEA_DEPTH_TEST_EQUAL = (1 << 14), ///< Depth Test: draw pixels with equal depth

    NEA_FOG_DISABLE = (0 << 15), ///< Enable fog
    NEA_FOG_ENABLE  = (1 << 15), ///< Enable fog
} NEA_OtherFormatEnum;

/// Enable or disable multiple polygon-related options.
///
/// Remember that translucent polygons can only be blended on top of other
/// translucent polygons if they have different polygon IDs.
///
/// @param alpha Alpha value (0 = wireframe, 31 = opaque, 1-30 translucent).
/// @param id Polygon ID used for antialias, blending and outlining (0 - 63).
/// @param lights Lights enabled. Use the enum NEA_LightEnum for this.
/// @param culling Which polygons must be drawn. Use the enum  NEA_CullingEnum.
/// @param other Other parameters. All possible flags are in NEA_OtherFormatEnum.
void NEA_PolyFormat(u32 alpha, u32 id, NEA_LightEnum lights,
                   NEA_CullingEnum culling, NEA_OtherFormatEnum other);

/// Returns the last value written to the polygon format register.
///
/// GFX_POLY_FORMAT is write-only, so the engine keeps a copy of whatever
/// NEA_PolyFormat() last sent. Use this to change one field without having to
/// know (or re-specify) the rest, which is how NEA_ModelSetPolyID() overrides
/// only the polygon ID of a model.
///
/// @return The last polygon format value written by NEA_PolyFormat().
u32 NEA_PolyFormatGet(void);

/// Enables the alpha test and sets the value pixels are compared against.
///
/// With the alpha test enabled a pixel is drawn only if its alpha is strictly
/// **greater** than the threshold; with it disabled the comparison is against
/// zero. The test happens on the final pixel, after texture blending, so it
/// sees the combined texture and vertex alpha.
///
/// This is what makes cutout textures cheap. A polygon with alpha 31 and an
/// A3I5 or A5I3 texture stays an opaque polygon as far as the renderer is
/// concerned: it writes depth, it needs no manual sorting, and it doesn't have
/// to be given a translucency polygon ID. Foliage, fences, ladders and grates
/// are the usual cases. Doing the same with translucent polygons costs a sort
/// and a set of IDs, and still breaks when two of them overlap.
///
/// It applies to the whole frame, not to individual polygons.
///
/// @param threshold Alpha to compare against (0 - 30). A threshold of 0 behaves
///                  exactly like having the test disabled, and 31 would hide
///                  every polygon, so it is rejected.
void NEA_AlphaTestEnable(u32 threshold);

/// Disables the alpha test.
///
/// Pixels are then drawn if their alpha is greater than zero.
void NEA_AlphaTestDisable(void);

/// Enable or disable polygon outline (edge marking).
///
/// For outlining to work, set up the colors with NEA_OutliningSetColor().
///
/// Color 0 works with polygon IDs 0 to 7, color 1 works with IDs 8 to 15, up to
/// color 7.
///
/// It only works with opaque or wireframe polygons.
///
/// **The hardware outlines a pixel only where the neighbouring pixel has a
/// different polygon ID**, so an object whose polygons all share one ID is
/// outlined against its surroundings but not across its own surface. That makes
/// the polygon ID the thing to plan, not the color: give each object that
/// should have its own outline a distinct ID with NEA_ModelSetPolyID(), or set
/// one by hand with NEA_PolyFormat().
///
/// Two more rules follow from how the hardware works:
///
/// - The outline is drawn only where the edge is *nearer* than the neighbour it
///   differs from. Edges hidden behind other geometry stay hidden.
/// - At the screen borders the comparison is against the rear plane's polygon
///   ID, which NEA sets to 63. Any object with an ID other than 63 is therefore
///   outlined against the background for free.
///
/// Polygon IDs do double duty as the translucency blending key, and NEA already
/// reserves a few: 63 for the rear plane, and 62 and 61 for the GUI
/// (NEA_GUI_POLY_ID). Avoid those when assigning IDs to outlined objects.
///
/// Edge marking and antialiasing interfere with each other. Antialiasing is on
/// by default; turn it off with NEA_AntialiasEnable(false) for clean outlines.
///
/// @param value True enables it, false disables it.
static inline void NEA_OutliningEnable(bool value)
{
    if (value)
        GFX_CONTROL |= GL_OUTLINE;
    else
        GFX_CONTROL &= ~GL_OUTLINE;
}

/// Set outlining color for the specified index.
///
/// Index 0 is used by polygon IDs 0 to 7, index 1 by IDs 8 to 15, and so on up
/// to index 7 for IDs 56 to 63.
///
/// @param index Color index (0 - 7).
/// @param color Color.
void NEA_OutliningSetColor(u32 index, u32 color);

/// Set the same outlining color for every index.
///
/// Convenience for the common case of one outline color for the whole scene,
/// where the polygon IDs exist to separate objects rather than to pick colors.
///
/// @param color Color.
void NEA_OutliningSetColorAll(u32 color);

/// Number of entries in the hardware toon table.
#define NEA_TOON_TABLE_SIZE 32

/// Setup shading tables for toon shading.
///
/// This is the original two-band ramp: the darker half of the shading range
/// becomes RGB15(8, 8, 8), the brighter half RGB15(24, 24, 24). It is the same
/// as calling:
///
/// ```
/// NEA_ToonTableBands(2, RGB15(8, 8, 8), RGB15(24, 24, 24));
/// ```
///
/// For the shading to look nice, change the properties of materials affecte
/// by this to, for example:
///
/// - AMBIENT = RGB15(8, 8, 8)
/// - DIFFUSE = RGB15(24, 24, 24)
/// - SPECULAR = RGB15(0, 0, 0)
/// - EMISSION = RGB15(0, 0, 0)
///
/// @param value True sets up tables for toon shading, false clears them.
void NEA_SetupToonShadingTables(bool value);

#define NEA_ShadingEnable NEA_SetupToonShadingTables

/// @defgroup toon_table Toon table
///
/// The toon table is a 32-entry color lookup used by polygons drawn with the
/// NEA_TOON_HIGHLIGHT_SHADING mode. The hardware takes the **red** channel of
/// the lit vertex color as the index (so 0 is unlit and 31 is fully lit) and
/// uses the color it finds there in place of the vertex color. Green and blue
/// of the vertex color are ignored, which is why toon-shaded materials are
/// usually set up with grey diffuse and ambient: the material decides *how lit*
/// a surface is, and the table decides what that lighting looks like.
///
/// That makes the table a great deal more than a cel-shading switch. Because it
/// is an arbitrary color ramp indexed by light intensity, it is a gradient map:
/// the shading can shift hue as it darkens (cool shadows, warm highlights)
/// rather than only losing brightness.
///
/// All of these cost nothing per frame and use no VRAM. They write 32 halfwords
/// to the GPU once; call them again whenever the look should change, including
/// every frame if something is meant to pulse or shift.
///
/// Toon and highlight are a per-frame choice made by NEA_ToonHighlightEnable(),
/// not a per-polygon one.
///
/// @{

/// Upload a raw toon table.
///
/// @param table Table of NEA_TOON_TABLE_SIZE (32) RGB15 colors.
void NEA_ToonTableSet(const u16 *table);

/// Fill the whole toon table with a single color.
///
/// Shading then has no effect on the color at all, which flattens every lit
/// surface to one tone. Useful for silhouettes and for objects that should read
/// as pure shape.
///
/// @param color Color.
void NEA_ToonTableFill(u32 color);

/// Build a cel ramp of evenly sized bands between two colors.
///
/// With 2 bands this is classic two-tone cel shading; 3 or 4 bands give the
/// shadow/mid/light look most hand-drawn styles use. Above roughly 8 the bands
/// stop being visible and NEA_ToonTableGradient() is the better tool.
///
/// The bands are interpolated per channel, so the ramp may change hue as well
/// as brightness.
///
/// @param bands Number of bands (1 - 32).
/// @param dark Color of the least lit band.
/// @param bright Color of the most lit band.
void NEA_ToonTableBands(int bands, u32 dark, u32 bright);

/// Build a smooth two-color gradient across the whole toon table.
///
/// This is a gradient map rather than a cel ramp: shading stays smooth, but it
/// travels between two colors instead of just fading to black. A cool blue at
/// the dark end and a warm cream at the light end is the usual painterly
/// choice.
///
/// @param lo Color at the unlit end.
/// @param hi Color at the fully lit end.
void NEA_ToonTableGradient(u32 lo, u32 hi);

/// Build a multi-stop gradient across the toon table.
///
/// The general form of the two above. Each stop pins a color to a table index,
/// and the entries between two stops are interpolated per channel. Entries
/// before the first stop and after the last one are held flat at that stop's
/// color.
///
/// This is what makes non-obvious ramps available: a cool shadow rising through
/// a neutral midtone into a warm rim, a band of subsurface red where light just
/// grazes a surface, or a hard step with soft shoulders on either side.
///
/// The indices must be strictly increasing.
///
/// @param colors Array of RGB15 colors, one per stop.
/// @param indices Array of table indices (0 - 31), strictly increasing.
/// @param count Number of stops (1 - NEA_TOON_TABLE_SIZE).
void NEA_ToonTableGradientStops(const u32 *colors, const u8 *indices, int count);

/// @}

/// Set highlight shading or toon shading modes.
///
/// Both modes read the toon table with the red channel of the vertex color. In
/// toon shading the table color replaces the vertex color; in highlight shading
/// it is also *added* on top afterwards, which brightens the result and can
/// shift its hue. Highlight shading is the one to use for a metallic or glossy
/// sheen, toon shading for flat painted surfaces.
///
/// This is a per-frame selection: every polygon drawn with
/// NEA_TOON_HIGHLIGHT_SHADING in a given frame uses whichever mode is set here.
///
/// By default, toon shading is selected.
///
/// @param value True enables highlight shading, false enables toon shading.
static inline void NEA_ToonHighlightEnable(bool value)
{
    if (value)
        GFX_CONTROL |= GL_TOON_HIGHLIGHT;
    else
        GFX_CONTROL &= ~GL_TOON_HIGHLIGHT;
}

/// Set color and related values of the rear plane.
///
/// @param color Color.
/// @param alpha Alpha value.
/// @param id Rear plane polygon ID.
void NEA_ClearColorSet(u32 color, u32 alpha, u32 id);

/// Returns the current clear color register value (internal use).
///
/// Used by two-pass FB mode to read and override the clear color alpha for
/// compositing.
///
/// @return The GFX_CLEAR_COLOR value stored internally.
u32 NEA_ClearColorGet(void);

/// Set the depth of the rear plane.
///
/// The rear plane is what the depth buffer is cleared to, so this decides how
/// far away "nothing" is. It defaults to the maximum, which is what you want
/// almost always: every polygon is then in front of the background.
///
/// Moving it nearer makes the rear plane occlude distant geometry, which is one
/// way to hide a far clipping plane. It also matters for edge marking, which
/// only outlines an edge that is nearer than what it is being compared against:
/// with a rear plane that is too near, objects stop being outlined against the
/// background.
///
/// When the clear bitmap is enabled with NEA_ClearBMPEnable() this register is
/// ignored and the per-pixel depths in VRAM_D are used instead.
///
/// @param depth Depth value (0 = near, 0x7FFF = far).
void NEA_ClearDepthSet(u32 depth);

/// Returns the current rear plane depth.
///
/// GFX_CLEAR_DEPTH is write-only, so this returns the engine's copy of it.
///
/// @return The depth value last set by NEA_ClearDepthSet().
u32 NEA_ClearDepthGet(void);

/// 1-dot polygon display boundary depth register.
#ifndef REG_DISP_1DOT_DEPTH
#define REG_DISP_1DOT_DEPTH (*(vu16*)0x4000610)
#endif

/// Set the depth past which 1-dot polygons are discarded.
///
/// A "1-dot" polygon is one so small or so distant that it covers a single
/// pixel. Beyond this depth the hardware can throw them away instead of
/// rendering them, which both saves polygon RAM and stops distant geometry from
/// speckling the screen with stray lit pixels.
///
/// The check is per polygon and can be turned off for the ones that matter, by
/// passing NEA_RENDER_ONEA_DOT_POLYS to NEA_PolyFormat(). It always uses the W
/// coordinate, whichever depth buffer mode is selected, and the polygon
/// survives if any one of its vertices is nearer than this value.
///
/// This register is not routed through the geometry FIFO, so a change takes
/// effect immediately and would otherwise apply to polygons that are already
/// queued. This function drains the FIFO first, which stalls until the geometry
/// engine is idle. Call it during setup, or at most once per frame before
/// drawing anything, not between models.
///
/// @param depth Depth value in 12.3 fixed point (0 = closest, 0x7FFF = most
///              distant). Use intto12d3() or floatto12d3().
void NEA_OneDotDepthSet(u32 depth);

/// Clear BMP scroll register.
#ifndef REG_CLRIMAGE_OFFSET
#define REG_CLRIMAGE_OFFSET (*(vu16*)0x4000356)
#endif

/// Enable or disable the clear bitmap.
///
/// The clear bitmap uses VRAM_C as color bitmap and VRAM_D as depth bitmap. You
/// have to copy data there and then use this function to enable it. Those 2
/// VRAM banks can't be used as texture banks with clear bitmap enabled, so you
/// have to call NEA_TextureSystemReset(0, 0, USE_VRAM_AB) before enabling it.
///
/// The dual 3D mode needs those two banks for the display capture, so you can't
/// use a clear BMP (even if you could, you would have no space for textures).
///
/// VRAM_C: ABBBBBGGGGGRRRRR (Alpha, Blue, Green, Red)
///
/// VRAM_D: FDDDDDDDDDDDDDDD (Fog enable, Depth) [0 = near, 0x7FFF = far]
///
/// The per-pixel depth channel in VRAM_D is what makes this worth the two VRAM
/// banks. Because the background carries its own depth, geometry can be
/// *occluded* by it rather than merely drawn behind it: write a skyline or a
/// ridge at a middling depth and distant objects disappear behind it while near
/// ones still pass in front, with no polygons spent on the background at all.
/// Scrolling the bitmap with NEA_ClearBMPScroll() as the camera turns then gives
/// a parallax backdrop that keeps that occlusion.
///
/// If you just want to display a flat background image with no depth, it's
/// better to use a textured polygon (or the 2D hardware).
///
/// @param value True to enable it, false to disable it.
void NEA_ClearBMPEnable(bool value);

/// Sets scroll of the clear BMP.
///
/// @param x Scroll on the X axis (0 - 255).
/// @param y Scroll on the Y axis (0 - 255).
static inline void NEA_ClearBMPScroll(u32 x, u32 y)
{
    REG_CLRIMAGE_OFFSET = (x & 0xFF) | ((y & 0xFF) << 8);
}

/// Enables fog and sets its parameters.
///
/// The values must be determined by trial and error.
///
/// The depth is the distance to the start of the fog from the camera. Use the
/// helpers floatto12d3() or intto12d3().
///
/// @param shift Distance between fog bands (1 - 15).
/// @param color Fog color.
/// @param alpha Alpha value.
/// @param mass Mass of fog.
/// @param depth Start point of fog (0 - 7FFFh)
void NEA_FogEnable(u32 shift, u32 color, u32 alpha, int mass, int depth);

/// Shape of the fog density ramp used by NEA_FogEnableCurve().
///
/// All four ramps start at zero density at the near distance and rise to full
/// density at the far distance. They differ in how the density is distributed
/// in between, which is what gives a scene its atmosphere.
typedef enum {
    /// Density rises evenly with distance. The original NEA_FogEnable()
    /// behaviour, and the right default for outdoor or neutral scenes.
    NEA_FOG_LINEAR = 0,

    /// Density rises slowly at first, then quickly. Keeps near geometry crisp
    /// while still burying the far wall. Good for large open interiors.
    NEA_FOG_SQUARED,

    /// Density rises quickly and then flattens out. Everything past the middle
    /// of the range is nearly fully fogged, which reads as thick smoke or dust.
    /// The heaviest of the four.
    NEA_FOG_EXP,

    /// Eased in and out (3t^2 - 2t^3). The band boundaries are least visible
    /// with this ramp, so use it when linear fog looks stepped.
    NEA_FOG_SMOOTHSTEP
} NEA_FogCurve;

/// Enables fog with a density curve and an explicit near/far depth.
///
/// This is a friendlier alternative to NEA_FogEnable(): instead of the
/// trial-and-error `shift`/`mass` pair, you say where the fog starts, where it
/// becomes opaque, and what shape the ramp has in between. The hardware fog
/// shift is derived from the range for you (the tightest bands that still reach
/// `far_depth`).
///
/// **Depths are raw 15 bit depth-buffer values (0 - 0x7FFF), not world units.**
/// What they mean depends on the depth buffer mode:
///
/// - Z-buffering (the NEA default): depth is the usual non-linear perspective
///   depth. With the default 0.1/40 clipping planes almost the whole visible
///   scene lands in 0x7000 - 0x7FFF, which is why hand-tuned fog offsets always
///   look like 0x7C00.
/// - W-buffering (NEA_SetDepthBufferMode(NEA_WBUFFER)): depth is the view
///   distance in 12.3 fixed point, so intto12d3() works.
///
/// Rather than working this out by hand, use NEA_FogDepthFromDistance() to turn
/// a world-space distance into the right value for the current mode.
///
/// The range the hardware can express is quantised to powers of two, so the
/// effective far depth is `near_depth + (0x8000 >> shift)`, which is at least
/// the `far_depth` you asked for and less than twice the requested range.
///
/// Costs nothing per frame: this builds the 32-entry density table once and
/// uploads it. Call it again to change the look. Uses no VRAM.
///
/// @param curve Shape of the density ramp.
/// @param color Fog color.
/// @param alpha Alpha value (0 - 31).
/// @param near_depth Depth at which fog starts (0 - 0x7FFF).
/// @param far_depth Depth at which fog is fully opaque (0 - 0x7FFF).
void NEA_FogEnableCurve(NEA_FogCurve curve, u32 color, u32 alpha,
                       u32 near_depth, u32 far_depth);

/// Converts a view-space distance into a depth value usable as a fog boundary.
///
/// Honours the current depth buffer mode (NEA_SetDepthBufferMode()) and the
/// current clipping planes (NEA_ClippingPlanesSetI()), so the result stays
/// correct if either changes. Feed the results straight into
/// NEA_FogEnableCurve().
///
/// This is a setup-time helper: it uses 64 bit maths and a division. Call it
/// when fog settings change, not every frame.
///
/// @param distance View-space distance from the camera, in f32 (1.19.12).
///                 Use floattof32() or inttof32().
/// @return The matching 15 bit depth value (0 - 0x7FFF).
u32 NEA_FogDepthFromDistance(int32_t distance);

/// Enable or disable the background fog.
///
/// This only affects the clear plane, not polygons.
///
/// @param value True enables it, false disables it.
void NEA_FogEnableBackground(bool value);

/// Disable fog.
static inline void NEA_FogDisable(void)
{
    GFX_CONTROL &= ~(GL_FOG | (15 << 8));
}

/// @}

#endif // NEA_POLYGON_H__
