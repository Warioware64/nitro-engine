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
    NEA_SHININESS_QUARTIC    ///< Increase values proportionaly to x^4
} NEA_ShininessFunction;

/// Generate and load a shininess table used for specular lighting.
///
/// @param function The name of the function used to generate the table.
void NEA_ShininessTableGenerate(NEA_ShininessFunction function);

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
    NEA_LIGHT_013 = NEA_LIGHT_0 | NEA_LIGHT_1 | NEA_LIGHT_2, ///< Lights 0, 1 and 3
    NEA_LIGHT_023 = NEA_LIGHT_0 | NEA_LIGHT_1 | NEA_LIGHT_2, ///< Lights 0, 2 and 3
    NEA_LIGHT_123 = NEA_LIGHT_0 | NEA_LIGHT_1 | NEA_LIGHT_2, ///< Lights 1, 2 and 3

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
    NEA_RENDER_ONEA_DOT_POLYS = (0 << 13), ///< Draw 1-dot polygons behind DISP_1DOT_DEPTH

    NEA_DEPTH_TEST_LESS  = (0 << 14), ///< Depth Test: draw pixels with less depth
    NEA_DEPTH_TEST_EQUAL = (0 << 14), ///< Depth Test: draw pixels with equal depth

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

/// Enable or disable polygon outline.
///
/// For outlining to work, set up the colors with NEA_OutliningSetColor().
///
/// Color 0 works with polygon IDs 0 to 7, color 1 works with IDs 8 to 15, up to
/// color 7.
///
/// It only works with opaque or wireframe polygons.
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
/// @param index Color index.
/// @param color Color.
void NEA_OutliningSetColor(u32 index, u32 color);

/// Setup shading tables for toon shading.
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

/// Set highlight shading or toon shading modes.
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
/// The only real use for this seems to be having a background image with
/// different depths per pixel. If you just want to display a background image
/// it's better to use a textured polygon (or the 2D hardware).
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
/// helpers float_to_12d3() or int_to_12d3().
///
/// @param shift Distance between fog bands (1 - 15).
/// @param color Fog color.
/// @param alpha Alpha value.
/// @param mass Mass of fog.
/// @param depth Start point of fog (0 - 7FFFh)
void NEA_FogEnable(u32 shift, u32 color, u32 alpha, int mass, int depth);

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

/// TODO: Function to set value of DISP_1DOT_DEPTH

/// @}

#endif // NEA_POLYGON_H__
