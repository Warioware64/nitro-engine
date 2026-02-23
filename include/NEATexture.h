// SPDX-License-Identifier: MIT
//
// Copyright (c) 2008-2022 Antonio Niño Díaz
//
// This file is part of Nitro Engine Advanced

#ifndef NEA_TEXTURE_H__
#define NEA_TEXTURE_H__

#include <nds.h>

#include "NEAPalette.h"
#include "NEAPolygon.h"

/// @file   NEATexture.h
/// @brief  Texture and material functions.

/// @defgroup material_system Material system
///
/// Material manipulation system. A material is composed of a texture and,
/// optionally, a palette. It also has diffuse, ambient, specular and emission
/// properties.
///
/// @{

#define NEA_DEFAULT_TEXTURES 128 ///< Default max number of materials

#define NEA_NO_PALETTE       -1 ///< Value that represents not having a palette

/// Holds information of one material.
typedef struct {
    int texindex;            ///< Index to internal texture object
    NEA_Palette *palette;     ///< Palette used by this material
    u32 color;               ///< Color of this material when lights aren't used
    u32 diffuse_ambient;     ///< Diffuse and ambient lighting material color
    u32 specular_emission;   ///< Specular and emission lighting material color
    bool palette_autodelete; ///< Set to true for the palette to be deleted with the material.
} NEA_Material;

/// Supported texture options
typedef enum {
    NEA_TEXTURE_WRAP_S = (1U << 16), ///< Wrap/repeat texture on S axis
    NEA_TEXTURE_WRAP_T = (1U << 17), ///< Wrap/repeat texture on T axis
    NEA_TEXTURE_FLIP_S = (1U << 18), ///< Flip texture on S axis when wrapping
    NEA_TEXTURE_FLIP_T = (1U << 19), ///< Flip texture on T axis when wrapping
    NEA_TEXTURE_COLOR0_TRANSPARENT = (1U << 29), ///< Make palette index 0 transparent
    NEA_TEXGEN_OFF      = (0U << 30), ///< Don't modify texture coordinates
    NEA_TEXGEN_TEXCOORD = (1U << 30), ///< Multiply coordinates by texture matrix
    NEA_TEXGEN_NORMAL   = (2U << 30), ///< Texcoords = Normal * texture matrix (spherical reflection)
    NEA_TEXGEN_POSITION = (3U << 30)  ///< Texcoords = Vertex * texture matrix
} NEA_TextureFlags;

/// Creates a new material object.
///
/// @return Pointer to the newly created material.
NEA_Material *NEA_MaterialCreate(void);

/// Applies a color to a material.
///
/// Note that the color will only be displayed if no normal commands are used.
/// Any model with normals will ignore this color.
///
/// @param tex Material.
/// @param color Color.
void NEA_MaterialColorSet(NEA_Material *tex, u32 color);

/// Removes the color of a material (sets it back to white).
///
/// @param tex Material.
void NEA_MaterialColorDelete(NEA_Material *tex);

/// Loads a texture from the filesystem and assigns it to a material object.
///
/// The height doesn't need to be a power of two, but he width must be a power
/// of two.
///
/// Textures with width that isn't a power of two need to be resized manually,
/// which is very slow, and they don't save any VRAM when loaded compared to a
/// texture with the full width. The only advantage is that they need less
/// storage space, but you can achieve the same effect by compressing them.
///
/// Textures with a height that isn't a power of two don't need to be resized,
/// and they actually save VRAM space (you tell the GPU that the texture is
/// bigger, but then you ignore the additional space, as it will be used by
/// other textures).
///
/// Textures with the 4x4 Texel format (NEA_TEX4X4) are normally split into two
/// parts: one that goes into texture slots 0 or 2 and another one that goes
/// into texture slot 1. This function expects the two parts to be concatenated
/// (with the slot 1 part after the other part).
///
/// @param tex Material.
/// @param fmt Texture format.
/// @param sizeX (sizeX, sizeY) Texture size.
/// @param sizeY (sizeX, sizeY) Texture size.
/// @param flags Parameters of the texture.
/// @param path Path of the texture file.
/// @return It returns 1 on success, 0 on error.
int NEA_MaterialTexLoadFAT(NEA_Material *tex, NEA_TextureFormat fmt,
                          int sizeX, int sizeY, NEA_TextureFlags flags,
                          const char *path);

/// Loads a texture in Texel 4x4 format from the filesystem and assigns it to a
/// material object.
///
/// Width and height need to be powers of two.
///
/// @param tex Material.
/// @param sizeX (sizeX, sizeY) Texture size.
/// @param sizeY (sizeX, sizeY) Texture size.
/// @param flags Parameters of the texture.
/// @param path02 Path of the texture file (part that goes in slot 0/2).
/// @param path1 Path of the texture file (part that goes in slot 1).
/// @return It returns 1 on success, 0 on error.
int NEA_MaterialTex4x4LoadFAT(NEA_Material *tex, int sizeX, int sizeY,
                             NEA_TextureFlags flags, const char *path02,
                             const char *path1);

/// Loads a texture in any format from a GRF file to a material and palette.
///
/// The size and format are obtained from the GRF header.
///
/// @param tex Material.
/// @param pal Palette. If the format is 16 bit, nothing will be loaded here.
/// @param flags Parameters of the texture.
/// @param path Path of the GRF file.
/// @return It returns 1 on success, 0 on error.
int NEA_MaterialTexLoadGRF(NEA_Material *tex, NEA_Palette *pal,
                          NEA_TextureFlags flags, const char *path);

/// Loads a texture from RAM and assigns it to a material object.
///
/// Textures with width that isn't a power of two need to be resized manually,
/// which is very slow, and they don't save any VRAM when loaded compared to a
/// texture with the full width. The only advantage is that they need less
/// storage space, but you can achieve the same effect by compressing them.
///
/// Textures with a height that isn't a power of two don't need to be resized,
/// and they actually save VRAM space (you tell the GPU that the texture is
/// bigger, but then you ignore the additional space, as it will be used by
/// other textures).
///
/// Textures with the 4x4 Texel format (NEA_TEX4X4) are normally split into two
/// parts: one that goes into texture slots 0 or 2 and another one that goes
/// into texture slot 1. This function expects the two parts to be concatenated
/// (with the slot 1 part after the other part).
///
/// @param tex Material.
/// @param fmt Texture format.
/// @param sizeX (sizeX, sizeY) Texture size.
/// @param sizeY (sizeX, sizeY) Texture size.
/// @param flags Parameters of the texture.
/// @param texture Pointer to the texture data.
/// @return It returns 1 on success, 0 on error.
int NEA_MaterialTexLoad(NEA_Material *tex, NEA_TextureFormat fmt,
                       int sizeX, int sizeY, NEA_TextureFlags flags,
                       const void *texture);

/// Loads a texture from RAM and assigns it to a material object.
///
/// Width and height need to be powers of two.
///
/// @param tex Material.
/// @param sizeX (sizeX, sizeY) Texture size.
/// @param sizeY (sizeX, sizeY) Texture size.
/// @param flags Parameters of the texture.
/// @param texture02 Pointer to the texture data (part that goes in slot 0/2).
/// @param texture1 Pointer to the texture data (part that goes in slot 1).
/// @return It returns 1 on success, 0 on error.
int NEA_MaterialTex4x4Load(NEA_Material *tex, int sizeX, int sizeY,
                          NEA_TextureFlags flags, const void *texture02,
                          const void *texture1);

/// Tell a material that it has to delete its palette on deletion.
///
/// Normally, when a material is deleted, the palette isn't deleted with it.
/// This function will tell the material to delete its palette when
/// NEA_MaterialDelete() is called. This is helpful because it lets the developer
/// stop caring about the palette.
///
/// @param mat Material.
void NEA_MaterialAutodeletePalette(NEA_Material *mat);

/// Copies the texture of a material into another material.
///
/// Unlike with models, you can delete the source and destination materials as
/// desired. Nitro Engine Advanced will keep track of how many materials use any specific
/// texture and palette and it will remove them when no more materials are using
/// them.
///
/// @param source Source.
/// @param dest Destination.
void NEA_MaterialClone(NEA_Material *source, NEA_Material *dest);

/// Alias of NEA_MaterialClone
///
/// @deprecated This definition is only present for backwards compatibility and
/// it will be removed.
#define NEA_MaterialTexClone NEA_MaterialClone

/// Assigns a palette to a material.
///
/// @param tex Material.
/// @param pal Palette.
void NEA_MaterialSetPalette(NEA_Material *tex, NEA_Palette *pal);

/// Alias of NEA_MaterialSetPalette().
///
/// @deprecated This definition is only present for backwards compatibility and
/// it will be removed.
#define NEA_MaterialTexSetPal NEA_MaterialSetPalette

/// Set active material to use when drawing polygons.
///
/// If the pointer passed is NULL the function will disable textures and new
/// polygons won't be affected by them until this function is called again with
/// a valid material.
///
/// @param tex Material to be used.
void NEA_MaterialUse(const NEA_Material *tex);

/// Flags to choose which VRAM banks Nitro Engine Advanced can use to allocate textures.
typedef enum {
    NEA_VRAM_A = (1 << 0), ///< Bank A
    NEA_VRAM_B = (1 << 1), ///< Bank B
    NEA_VRAM_C = (1 << 2), ///< Bank C
    NEA_VRAM_D = (1 << 3), ///< Bank D

    NEA_VRAM_AB = NEA_VRAM_A | NEA_VRAM_B, ///< Banks A and B
    NEA_VRAM_AC = NEA_VRAM_A | NEA_VRAM_C, ///< Banks A and C
    NEA_VRAM_AD = NEA_VRAM_A | NEA_VRAM_D, ///< Banks A and D
    NEA_VRAM_BC = NEA_VRAM_B | NEA_VRAM_C, ///< Banks B and C
    NEA_VRAM_BD = NEA_VRAM_B | NEA_VRAM_D, ///< Banks B and D
    NEA_VRAM_CD = NEA_VRAM_C | NEA_VRAM_D, ///< Banks C and D

    NEA_VRAM_ABC = NEA_VRAM_A | NEA_VRAM_B | NEA_VRAM_C, ///< Banks A, B and C
    NEA_VRAM_ABD = NEA_VRAM_A | NEA_VRAM_B | NEA_VRAM_D, ///< Banks A, B and D
    NEA_VRAM_ACD = NEA_VRAM_A | NEA_VRAM_C | NEA_VRAM_D, ///< Banks A, C and D
    NEA_VRAM_BCD = NEA_VRAM_B | NEA_VRAM_C | NEA_VRAM_D, ///< Banks B, C and D

    NEA_VRAM_ABCD = NEA_VRAM_A | NEA_VRAM_B | NEA_VRAM_C | NEA_VRAM_D, ///< All main banks
} NEA_VRAMBankFlags;

/// Resets the material system and sets the new max number of objects.
///
/// In Dual 3D mode, only VRAM A and B are available for textures.
///
/// If no VRAM banks are specified in this function, all VRAM banks A to D will
/// be used for textures (or just A and B in dual 3D mode).
///
/// @param max_textures Max number of textures. If lower than 1, it will
///                     create space for NEA_DEFAULT_TEXTURES.
/// @param max_palettes Max number of palettes. If lower than 1, it will
///                     create space for NEA_DEFAULT_PALETTES.
/// @param bank_flags VRAM banks where Nitro Engine Advanced can allocate textures.
/// @return Returns 0 on success.
int NEA_TextureSystemReset(int max_textures, int max_palettes,
                          NEA_VRAMBankFlags bank_flags);

/// Deletes a material object.
///
/// @param tex Pointer to the material object.
void NEA_MaterialDelete(NEA_Material *tex);

/// Returns the available free memory for textures.
///
/// Note that, even if it is all available, it may not be contiguous, so you may
/// not be able to load a texture because there isn't enough space in any free
/// gap.
///
/// @return Returns the available memory in bytes.
int NEA_TextureFreeMem(void);

/// Returns the percentage of available free memory for textures.
///
/// @return Returns the percentage of available memory (0-100).
int NEA_TextureFreeMemPercent(void);

/// Defragment memory used for textures.
///
/// WARNING: This function is currently not working.
void NEA_TextureDefragMem(void);

/// End texture system and free all memory used by it.
void NEA_TextureSystemEnd(void);

/// Returns the width of a texture.
///
/// This is the size given when the texture was loaded.
///
/// @param tex Material.
/// @return Returns the size in pixels.
int NEA_TextureGetSizeX(const NEA_Material *tex);

/// Returns the height of a texture.
///
/// This is the size given when the texture was loaded.
///
/// @param tex Material.
/// @return Returns the size in pixels.
int NEA_TextureGetSizeY(const NEA_Material *tex);

/// Returns the real width of a texture.
///
/// This is the internal size given to the GPU when the texture is used, not the
/// size used to load the texture, which may have been smaller.
///
/// @param tex Material.
/// @return Returns the size in pixels.
int NEA_TextureGetRealSizeX(const NEA_Material *tex);

/// Returns the real height size of a texture.
///
/// This is the internal size given to the GPU when the texture is used, not the
/// size used to load the texture, which may have been smaller.
///
/// @param tex Material.
/// @return Returns the size in pixels.
int NEA_TextureGetRealSizeY(const NEA_Material *tex);

/// Sets lighting properties of this material.
///
/// @param tex Material to modify.
/// @param diffuse Set diffuse color: lights that directly hits the polygon.
/// @param ambient Set ambient color: lights that indirectly hit the polygon
///                (reflections from the walls, etc).
/// @param specular Set specular color: lights reflected towards the camera,
///                 like a mirror.
/// @param emission Set emission color: light emitted by the polygon.
/// @param vtxcolor If true, diffuse reflection will work as a color command.
/// @param useshininess If true, specular reflection will use the shininess
///                     table.
void NEA_MaterialSetProperties(NEA_Material *tex, u32 diffuse, u32 ambient,
                              u32 specular, u32 emission, bool vtxcolor,
                              bool useshininess);

/// Alias of NEA_MaterialSetProperties
///
/// @deprecated This definition is only present for backwards compatibility and
/// it will be removed.
#define NEA_MaterialSetPropierties NEA_MaterialSetProperties

/// Sets default lighting properties of materials when they are created.
///
/// @param diffuse Set diffuse color: lights that directly hits the polygon.
/// @param ambient Set ambient color: lights that indirectly hit the polygon
///                (reflections from the walls, etc).
/// @param specular Set specular color: lights reflected towards the camera,
///                 like a mirror.
/// @param emission Set emission color: light emitted by the polygon.
/// @param vtxcolor If true, diffuse reflection will work as a color command.
/// @param useshininess If true, specular reflection will use the shininess
///                     table.
void NEA_MaterialSetDefaultProperties(u32 diffuse, u32 ambient, u32 specular,
                                     u32 emission, bool vtxcolor,
                                     bool useshininess);

/// Alias of NEA_MaterialSetDefaultProperties
///
/// @deprecated This definition is only present for backwards compatibility and
/// it will be removed.
#define NEA_MaterialSetDefaultPropierties NEA_MaterialSetDefaultProperties

/// Enables modification of the specified texture.
///
/// Use this during VBL. Remember to use NEA_TextureDrawingEnd() when you finish.
/// If you don't, the GPU won't be able to render textures to the screen.
///
/// @param tex Texture to modify.
/// @return Returns a pointer to the base address of the texture in VRAM.
void *NEA_TextureDrawingStart(const NEA_Material *tex);

/// Sets the specified pixel to the specified color.
///
/// This only works for textures in RGBA/RGB format.
///
/// Use this during VBL.
///
/// @param x (x, y) Pixel coordinates.
/// @param y (x, y) Pixel coordinates.
/// @param color Color in RGB15. Bit 15 must be set to make the pixel visible.
void NEA_TexturePutPixelRGBA(u32 x, u32 y, u16 color);

/// Sets the specified pixel to the specified palette color index.
///
/// This only works for textures in RGB256 format.
///
/// Use this during VBL.
///
/// @param x (x,y) Pixel coordinates.
/// @param y (x,y) Pixel coordinates.
/// @param palettecolor New palette color index.
void NEA_TexturePutPixelRGB256(u32 x, u32 y, u8 palettecolor);

/// Disables modification of textures.
///
/// Use this during VBL.
void NEA_TextureDrawingEnd(void);

/// @}

#endif // NEA_TEXTURE_H__
