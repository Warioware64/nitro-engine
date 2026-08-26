// SPDX-License-Identifier: MIT
//
// Copyright (c) 2008-2022 Antonio Niño Díaz
//
// This file is part of Nitro Engine Advanced

#ifndef NEA_TEXTURE_H__
#define NEA_TEXTURE_H__

#include <nds.h>

#include "NEAFAT.h"
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

/// Maximum length of a material name (including null terminator).
#define NEA_MATERIAL_NAME_LEN 32

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

/// Holds information of one material.
typedef struct {
    int texindex;            ///< Index to internal texture object
    NEA_Palette *palette;     ///< Palette used by this material
    u32 color;               ///< Color of this material when lights aren't used
    u32 diffuse_ambient;     ///< Diffuse and ambient lighting material color
    u32 specular_emission;   ///< Specular and emission lighting material color
    bool palette_autodelete; ///< Set to true for the palette to be deleted with the material.
    bool ram_backed;         ///< Set by NEA_MaterialRamInit: texture is kept in main RAM
    char name[NEA_MATERIAL_NAME_LEN]; ///< Name/alias for material lookup
    void *ram_data;          ///< Owned RAM copy of the texture image, or NULL
    u32 ram_size;            ///< Byte size of ram_data
    int ram_sizex;           ///< Width of the stashed texture
    int ram_sizey;           ///< Height of the stashed texture
    NEA_TextureFormat ram_format; ///< Format of the stashed texture
    NEA_TextureFlags ram_flags;   ///< Flags of the stashed texture
    bool dirty;              ///< RAM buffer changed; next VramUpdate re-uploads it
} NEA_Material;

/// Creates a new material object.
///
/// @return Pointer to the newly created material.
NEA_Material *NEA_MaterialCreate(void);

/// Set the name/alias of a material for lookup purposes.
///
/// @param mat Material to name.
/// @param name Name string (max 31 characters, will be truncated).
void NEA_MaterialSetName(NEA_Material *mat, const char *name);

/// Get the name of a material.
///
/// @param mat Material.
/// @return Pointer to the name string (empty string if unset).
const char *NEA_MaterialGetName(const NEA_Material *mat);

/// Find a material by name.
///
/// Searches all created materials for one matching the given name.
///
/// @param name Name to search for.
/// @return Pointer to the matching material, or NULL if not found.
NEA_Material *NEA_MaterialFindByName(const char *name);

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

/// Binds only a material's texture image, leaving everything else alone.
///
/// NEA_MaterialUse() binds the image together with the material's colour, its
/// lighting properties and its palette. This binds the image and nothing else,
/// which is what a texture animation wants: the material stays as it was and
/// only the picture on it changes.
///
/// @param tex Material to take the texture from. NULL unbinds the texture.
void NEA_MaterialTexUse(const NEA_Material *tex);

/// Changes how texture coordinates are generated for a material.
///
/// The texgen mode is normally chosen once, in the flags passed to the loader.
/// This switches it afterwards, which is what makes it possible to turn a
/// material's reflection on and off at runtime.
///
/// Passing NEA_TEXGEN_NORMAL turns the material into an environment map: see
/// NEA_TextureMatrixEnvMap() for what else that needs.
///
/// @param tex Material (it must have a texture in VRAM).
/// @param texgen One of NEA_TEXGEN_OFF, NEA_TEXGEN_TEXCOORD,
///               NEA_TEXGEN_NORMAL or NEA_TEXGEN_POSITION.
void NEA_MaterialSetTexGen(NEA_Material *tex, NEA_TextureFlags texgen);

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

/// Asynchronously loads a texture from the filesystem into a material object.
///
/// This works like NEA_MaterialTexLoadFAT(), but the file is read in the
/// background. The texture is uploaded to VRAM by NEA_AsyncProcess() once the
/// data is in RAM. See @ref async for details.
///
/// The material must not be deleted until the load reaches NEA_ASYNC_DONE.
///
/// @param tex Material.
/// @param fmt Texture format.
/// @param sizeX (sizeX, sizeY) Texture size.
/// @param sizeY (sizeX, sizeY) Texture size.
/// @param flags Parameters of the texture.
/// @param path Path of the texture file.
/// @return Async handle to poll the operation, or NULL on error.
NEA_AsyncFile *NEA_MaterialTexLoadFATAsync(NEA_Material *tex,
                NEA_TextureFormat fmt, int sizeX, int sizeY,
                NEA_TextureFlags flags, const char *path);

/// Asynchronously loads a Texel 4x4 texture from the filesystem.
///
/// This works like NEA_MaterialTex4x4LoadFAT(), but the files are read in the
/// background. See @ref async for details.
///
/// The material must not be deleted until the load reaches NEA_ASYNC_DONE.
///
/// @param tex Material.
/// @param sizeX (sizeX, sizeY) Texture size.
/// @param sizeY (sizeX, sizeY) Texture size.
/// @param flags Parameters of the texture.
/// @param path02 Path of the texture file (part that goes in slot 0/2).
/// @param path1 Path of the texture file (part that goes in slot 1).
/// @return Async handle to poll the operation, or NULL on error.
NEA_AsyncFile *NEA_MaterialTex4x4LoadFATAsync(NEA_Material *tex,
                int sizeX, int sizeY, NEA_TextureFlags flags,
                const char *path02, const char *path1);

/// Asynchronously loads a texture from a GRF file to a material and palette.
///
/// This works like NEA_MaterialTexLoadGRF(), but the file is read in the
/// background. The GRF data is decoded in the worker thread and the texture is
/// uploaded to VRAM by NEA_AsyncProcess(). See @ref async for details.
///
/// Note: decoding a large compressed GRF runs as a single step in the worker
/// thread and may cause a single dropped frame, unlike the chunked file read.
///
/// The material and palette must not be deleted until the load reaches
/// NEA_ASYNC_DONE.
///
/// @param tex Material.
/// @param pal Palette. If the format is 16 bit, nothing will be loaded here.
/// @param flags Parameters of the texture.
/// @param path Path of the GRF file.
/// @return Async handle to poll the operation, or NULL on error.
NEA_AsyncFile *NEA_MaterialTexLoadGRFAsync(NEA_Material *tex, NEA_Palette *pal,
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

/// Makes a material keep its texture in main RAM instead of VRAM.
///
/// Call this right after NEA_MaterialCreate() and before loading any texture.
/// Once a material is RAM-backed, NEA_MaterialTexLoad() (and the FAT/GRF/4x4
/// variants) will store a private copy of the texture image in main RAM instead
/// of uploading it to VRAM. Use NEA_MaterialTexVramLoad() to upload the texture
/// to VRAM when it is needed, and NEA_MaterialTexVramUnload() to free the VRAM
/// while keeping the RAM copy. This lets you stream textures in and out of the
/// scarce texture VRAM on demand.
///
/// The palette (if any) is not affected; it stays resident in palette VRAM.
///
/// @param material Material (must have no texture assigned yet).
void NEA_MaterialRamInit(NEA_Material *material);

/// Uploads the RAM copy of a RAM-backed material's texture into VRAM.
///
/// The material must have been set up with NEA_MaterialRamInit() and have a
/// texture loaded (which was stashed in RAM). If the texture is already in VRAM
/// this does nothing and succeeds.
///
/// @param material RAM-backed material.
/// @return It returns 1 on success, 0 on error.
int NEA_MaterialTexVramLoad(NEA_Material *material);

/// Frees a RAM-backed material's texture from VRAM, keeping the RAM copy.
///
/// The texture can be uploaded again later with NEA_MaterialTexVramLoad(). If
/// the texture is not currently in VRAM this does nothing and succeeds. The
/// palette (if any) is left untouched. While the texture is not in VRAM, drawing
/// the material renders it untextured.
///
/// @param material RAM-backed material.
/// @return It returns 1 on success, 0 on error.
int NEA_MaterialTexVramUnload(NEA_Material *material);

/// Returns a writable pointer to a material's RAM-side texture buffer.
///
/// Only valid for RAM-backed materials (see NEA_MaterialRamInit() and
/// NEA_MaterialTexBlank()). Modify the pixels in this buffer in place, mark the
/// material dirty with NEA_MaterialTexSetDirty(), then push the changes to VRAM
/// with NEA_MaterialTexVramUpdate(). This is the basis of the "dirty texture"
/// workflow: manipulate texels freely on the CPU and upload on demand.
///
/// @param material RAM-backed material.
/// @return Pointer to the RAM buffer, or NULL if the material has none.
void *NEA_MaterialTexGetData(NEA_Material *material);

/// Allocates a blank (zero-filled) RAM-backed texture and uploads it to VRAM.
///
/// Use this to build a texture in code from scratch, with no source image. The
/// material becomes RAM-backed (as if NEA_MaterialRamInit() had been called) and
/// gains a VRAM copy so it is immediately renderable. Get the buffer with
/// NEA_MaterialTexGetData(), draw into it, and push changes with
/// NEA_MaterialTexVramUpdate().
///
/// @param material Material (must have no texture assigned yet).
/// @param fmt Texture format.
/// @param sizeX (sizeX, sizeY) Texture size.
/// @param sizeY (sizeX, sizeY) Texture size.
/// @param flags Parameters of the texture.
/// @return It returns 1 on success, 0 on error.
int NEA_MaterialTexBlank(NEA_Material *material, NEA_TextureFormat fmt,
                        int sizeX, int sizeY, NEA_TextureFlags flags);

/// Marks a RAM-backed material's texture as modified.
///
/// The next NEA_MaterialTexVramUpdate() will re-upload the RAM buffer to VRAM.
///
/// @param material RAM-backed material.
void NEA_MaterialTexSetDirty(NEA_Material *material);

/// Pushes a RAM-backed material's texture buffer to VRAM if it is dirty.
///
/// For a texture that is currently resident in VRAM the pixels are copied in
/// place into the existing VRAM slot, with no reallocation. If the texture is
/// not currently resident it is uploaded like NEA_MaterialTexVramLoad(). If the
/// material has not been marked dirty this does nothing and succeeds.
///
/// Like the other VRAM upload functions, this remaps texture VRAM to LCD mode
/// while copying, so it must be called when that is safe: immediately after
/// NEA_WaitForVBL(), during the vertical blank. Do any heavy edits to the RAM
/// buffer (from NEA_MaterialTexGetData()) *before* NEA_WaitForVBL() so this
/// upload stays inside the short vblank window; otherwise the bank remap races
/// the GPU while it samples textures and the texture renders black.
///
/// @param material RAM-backed material.
/// @return It returns 1 on success, 0 on error.
int NEA_MaterialTexVramUpdate(NEA_Material *material);

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

    NEA_VRAM_E = (1 << 4), ///< Bank E (64KB)
    NEA_VRAM_F = (1 << 5), ///< Bank F (16KB)
    NEA_VRAM_G = (1 << 6), ///< Bank G (16KB)
    NEA_VRAM_H = (1 << 7), ///< Bank H (32KB)
    NEA_VRAM_I = (1 << 8), ///< Bank I (16KB)
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

/// @defgroup texture_matrix Texture matrix
///
/// Functions to manipulate the GPU texture matrix. Materials loaded with
/// NEA_TEXGEN_OFF (the default) are not affected, so there is no conflict; all
/// three other texgen modes go through this matrix.
///
/// The scroll, rotate and scale helpers are meant for NEA_TEXGEN_TEXCOORD, and
/// transform the UVs baked into the mesh. NEA_TextureMatrixEnvMap() is the one
/// for NEA_TEXGEN_NORMAL, and does something rather different: it makes the
/// matrix generate coordinates from the surface normal.
///
/// @{

/// Reset the texture matrix to identity.
///
/// Call this before applying new texture transforms.
void NEA_TextureMatrixIdentity(void);

/// Translate the texture matrix (fixed-point).
///
/// **The unit is a sixteenth of a texel, not a texel.** The hardware multiplies
/// the translation row of the texture matrix by a constant 1/16 (see GBATEK's
/// texture coordinate transformation, mode 1), so a translation of `x` shifts
/// the texture by `x / 16` texels. Scrolling a 64 texel texture through one
/// full wrap therefore needs `inttof32(64 * 16)`, not `inttof32(64)`.
///
/// Use NEA_TextureMatrixTranslateTexels() if you would rather say it in texels.
///
/// @param x Translation on the S (horizontal) axis (f32, in 1/16 texels).
/// @param y Translation on the T (vertical) axis (f32, in 1/16 texels).
void NEA_TextureMatrixTranslateI(int x, int y);

/// Translate the texture matrix by a number of texels.
///
/// The same as NEA_TextureMatrixTranslateI() with the sixteenths worked out for
/// you, which is what almost every caller actually wants: scrolling a texture by
/// its own width is `NEA_TextureMatrixTranslateTexels(inttof32(width), 0)`.
///
/// @param u Translation on the S (horizontal) axis, in texels (f32).
/// @param v Translation on the T (vertical) axis, in texels (f32).
static inline void NEA_TextureMatrixTranslateTexels(int u, int v)
{
    NEA_TextureMatrixTranslateI(u * 16, v * 16);
}

/// Translate the texture matrix (float).
///
/// @param x Translation on the S (horizontal) axis.
/// @param y Translation on the T (vertical) axis.
#define NEA_TextureMatrixTranslate(x, y) \
    NEA_TextureMatrixTranslateI(floattof32(x), floattof32(y))

/// Rotate the texture matrix.
///
/// Rotates texture coordinates in the S-T plane. Uses the same angle units
/// as NEA_ModelSetRot() (0-511 = full rotation).
///
/// @param angle Rotation angle.
void NEA_TextureMatrixRotate(int angle);

/// Scale the texture matrix (fixed-point).
///
/// @param sx Scale on the S (horizontal) axis (f32).
/// @param sy Scale on the T (vertical) axis (f32).
void NEA_TextureMatrixScaleI(int sx, int sy);

/// Scale the texture matrix (float).
///
/// @param sx Scale on the S (horizontal) axis.
/// @param sy Scale on the T (vertical) axis.
#define NEA_TextureMatrixScale(sx, sy) \
    NEA_TextureMatrixScaleI(floattof32(sx), floattof32(sy))

/// Loads a texture matrix that turns surface normals into sphere-map
/// coordinates.
///
/// This is environment mapping: a "matcap" or chrome look, where a single
/// circular image is reflected off the surface and slides across it as the
/// object or the camera turns. It costs no extra polygons and no per-vertex
/// work, because the coordinate generation happens in the same hardware unit
/// that already transforms the normal for lighting.
///
/// Three things have to line up for it to work:
///
/// 1. **The material** must use NEA_TEXGEN_NORMAL, either from its load flags
///    or via NEA_MaterialSetTexGen(). The texture itself should be a sphere
///    map: a circular image where the centre is the part of the surface facing
///    the camera and the rim is the part facing away.
/// 2. **The matrix**, which is this function. Call it after the camera and the
///    model's own transform are in place, because it reads the transform that
///    is current at that moment. That is what makes the reflection follow the
///    object rather than only the camera.
/// 3. **The mesh** must have every texture coordinate at the centre of the
///    texture. In this mode the hardware *adds* the mesh's coordinate to the
///    generated one, so it acts as the origin of the sphere map; anything else
///    smears the reflection across the surface. Export models for this with
///    `obj2dl.py --envmap-uv`, or emit NEA_PolyTexCoord(w / 2, h / 2) before
///    each normal when building geometry by hand.
///
/// So the per-model draw order is: camera, model transform, this, draw.
///
/// This reads back a GPU result register, which means waiting for the geometry
/// engine to go idle. That is a real stall, so it is a per-object cost, not a
/// per-vertex one: call it once per environment-mapped model, not more.
///
/// @param tex Material whose texture is the sphere map. Its size determines the
///            scale of the mapping.
void NEA_TextureMatrixEnvMap(const NEA_Material *tex);

/// Loads a sphere-map texture matrix with explicit scale factors.
///
/// The general form of NEA_TextureMatrixEnvMap(), for when the sphere map does
/// not fill its texture, or to deliberately over- or under-scale the
/// reflection. The scales are the half-size of the area the reflection should
/// cover, in texels: a unit normal maps to +/- that many texels around the
/// mesh's texture coordinate.
///
/// Everything said about NEA_TextureMatrixEnvMap() applies here too.
///
/// Note the sign of `scale_t`. Texture T grows downwards while a normal's +Y
/// points up, so a sphere map image wants a **negative** T scale; passing a
/// positive one mirrors the reflection vertically, which reads as lighting the
/// model from below. NEA_TextureMatrixEnvMap() handles this for you.
///
/// @param scale_s Half-width of the sphere map in texels (f32). Use inttof32().
/// @param scale_t Half-height of the sphere map in texels (f32), normally
///                negative.
void NEA_TextureMatrixEnvMapI(int scale_s, int scale_t);

/// @}

#endif // NEA_TEXTURE_H__
