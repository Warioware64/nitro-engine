// SPDX-License-Identifier: MIT
//
// Copyright (c) 2008-2022 Antonio Niño Díaz
//
// This file is part of Nitro Engine Advanced

#include "NEAMain.h"
#include "NEAAlloc.h"

/// @file NEATexture.c

typedef struct {
    u32 param;
    // For regular textures, this is the base address in VRAM of the texture.
    // For compressed textures, this is the address in slot 0 or 2. The address
    // of the data in slot 1 can be calculated from it.
    char *address;
    int uses; // Number of materials that use this texture
    int sizex, sizey;
    // The memory is the caller's (NEA_MaterialTexBindFixed()): it is not
    // freed with the texture, and the texture is never moved.
    bool fixed;
} ne_textureinfo_t;

static ne_textureinfo_t *NEA_Texture = NULL;
static NEA_Material **NEA_UserMaterials = NULL;

static NEAChunk *NEA_TexAllocList; // See NEAAlloc.h

static bool ne_texture_system_inited = false;

static int NEA_MAX_TEXTURES;

// Default material properties
static u32 ne_default_diffuse_ambient;
static u32 ne_default_specular_emission;

// The primary banks an upload writes to, switched to LCD mode for the length
// of the copy -- and only those. A bank the copy does not touch keeps doing
// whatever it was doing: showing a bitmap background or a captured frame, or
// holding the textures of what is being drawn. (Switching all four, as this
// used to, takes such a bank off the screen for as many scanlines as the copy
// lasts: a band across the picture for every texture loaded.)
//
// In LCD mode the banks are 128 KB each from 0x06800000, which is where the
// allocator's addresses already point.
typedef struct {
    u8 mask;     // Bit n: bank n (A to D) was switched
    u8 saved[4]; // What its control register held
} ne_tex_unlock_t;

static inline u8 ne_tex_bank_mask(const void *addr, uint32_t size)
{
    uintptr_t first = (uintptr_t)addr;

    if ((size == 0) || (first < 0x06800000) || (first >= 0x06880000))
        return 0;

    uintptr_t last = first + size - 1;
    if (last >= 0x06880000)
        last = 0x06880000 - 1;

    u8 mask = 0;
    for (uintptr_t bank = (first - 0x06800000) >> 17;
         bank <= ((last - 0x06800000) >> 17); bank++)
        mask |= 1 << bank;

    return mask;
}

static void ne_tex_unlock(ne_tex_unlock_t *state, u8 mask)
{
    vu8 *const control[4] = { &VRAM_A_CR, &VRAM_B_CR, &VRAM_C_CR, &VRAM_D_CR };

    state->mask = mask;
    for (int i = 0; i < 4; i++)
    {
        if ((mask & (1 << i)) == 0)
            continue;
        state->saved[i] = *control[i];
        *control[i] = VRAM_ENABLE; // (LCD mode is 0 in every primary bank)
    }
}

static void ne_tex_lock(const ne_tex_unlock_t *state)
{
    vu8 *const control[4] = { &VRAM_A_CR, &VRAM_B_CR, &VRAM_C_CR, &VRAM_D_CR };

    for (int i = 0; i < 4; i++)
    {
        if (state->mask & (1 << i))
            *control[i] = state->saved[i];
    }
}

// Defined below, before NEA_MaterialTexLoad()
static void ne_copy_tex_to_vram(void *addr, NEA_TextureFormat fmt,
                                uint32_t size, const void *texture);

// Computes the number of bytes a texture of this format and size occupies in
// RAM/VRAM, matching the layout used by ne_material_tex_stash() and
// NEA_MaterialTexLoad().
static uint32_t ne_tex_data_size(NEA_TextureFormat fmt, int sizeX, int sizeY)
{
    if (fmt == NEA_TEX4X4)
    {
        // Compressed: main data (slot 0/2) plus index data (slot 1)
        uint32_t size02 = (sizeX * sizeY) >> 2;
        uint32_t size1 = size02 >> 1;
        return size02 + size1;
    }

    static const int size_shift[] = {
        0, // Nothing
        1, // NEA_A3PAL32
        3, // NEA_PAL4
        2, // NEA_PAL16
        1, // NEA_PAL256
        0, // NEA_TEX4X4 (unused here)
        1, // NEA_A5PAL8
        0, // NEA_A1RGB5
        0, // NEA_RGB5
    };
    return (sizeX * sizeY << 1) >> size_shift[fmt];
}

static int ne_is_valid_tex_size(int size)
{
    for (int i = 0; i < 8; i++)
    {
        if (size <= (8 << i))
            return (8 << i);
    }
    return 0;
}

static int ne_tex_raw_size(int size)
{
    for (int i = 0; i < 8; i++)
    {
        if (size == 8)
            return i;
        size >>= 1;
    }
    return 0;
}

#ifdef NEA_DEBUG
// A texture with a height that isn't a power of two is stored trimmed: only the
// real rows are in VRAM, but the GPU is told the padded height. It therefore
// wraps at the padded height, not at the real one, so repeating on the T axis
// samples the rows past the end of the data, which belong to whatever texture
// was allocated after this one.
static void ne_warn_trimmed_wrap_t(int sizeY, NEA_TextureFlags flags)
{
    if ((ne_is_valid_tex_size(sizeY) != sizeY)
        && (flags & NEA_TEXTURE_WRAP_T))
    {
        NEA_DebugPrint("NEA_TEXTURE_WRAP_T with a height that isn't a power of 2 "
                       "(%d): the GPU wraps at %d and samples unallocated VRAM",
                       sizeY, ne_is_valid_tex_size(sizeY));
    }
}
#else
#define ne_warn_trimmed_wrap_t(sizeY, flags) ((void)0)
#endif

// The provided address must be in VRAM_A
static inline void *slot0_to_slot1(void *ptr)
{
    uintptr_t offset0 = (uintptr_t)ptr - (uintptr_t)VRAM_A;
    return (void *)((uintptr_t)VRAM_B + (offset0 / 2));
}

// The provided address must be in VRAM_B
static inline void *slot1_to_slot0(void *ptr)
{
    uintptr_t offset1 = (uintptr_t)ptr - (uintptr_t)VRAM_B;
    return (void *)((uintptr_t)VRAM_A + (offset1 * 2));
}

// The provided address must be in VRAM_C
static inline void *slot2_to_slot1(void *ptr)
{
    uintptr_t offset2 = (uintptr_t)ptr - (uintptr_t)VRAM_C;
    return (void *)((uintptr_t)VRAM_B + (64 * 1024) + (offset2 / 2));
}

// The provided address must be in VRAM_B
static inline void *slot1_to_slot2(void *ptr)
{
    uintptr_t offset1 = (uintptr_t)ptr - (uintptr_t)VRAM_B - (64 * 1024);
    return (void *)((uintptr_t)VRAM_C + (offset1 * 2));
}

static inline void ne_set_material_tex_param(NEA_Material *tex,
                            int sizeX, int sizeY, uint32_t *addr,
                            GL_TEXTURE_TYPE_ENUM mode, u32 param)
{
    NEA_AssertPointer(tex, "NULL pointer");
    NEA_Assert(tex->texindex != NEA_NO_TEXTURE, "No assigned texture");
    NEA_Texture[tex->texindex].param =
            (ne_tex_raw_size(sizeX) << 20) |
            (ne_tex_raw_size(sizeY) << 23) |
            (((uint32_t)addr >> 3) & 0xFFFF) |
            (mode << 26) | param;
}

// The defragmenter (below): a texture deleted while it is being moved stops
// the move; and where a texture that is not 4x4 is put.
static void ne_defrag_forget_slot(int slot);
static void *ne_alloc_plain(size_t size);

static void ne_texture_delete(int texture_index)
{
    int slot = texture_index;

    // A texture may be used by several materials
    NEA_Texture[slot].uses--;

    // If the number of users is zero, delete it.
    if (NEA_Texture[slot].uses == 0)
    {
        ne_defrag_forget_slot(slot);

        uint32_t fmt = (NEA_Texture[slot].param >> 26) & 7;

        if (NEA_Texture[slot].fixed)
        {
            // (The caller's memory: it stays the caller's.)
        }
        else if (fmt == NEA_TEX4X4)
        {
            // Check if the texture is allocated in VRAM_A or VRAM_C, and
            // calculate the corresponding address in VRAM_B.
            void *slot02 = NEA_Texture[slot].address;
            void *slot1 = (slot02 < (void *)VRAM_B) ?
                          slot0_to_slot1(slot02) : slot2_to_slot1(slot02);
            NEA_Free(NEA_TexAllocList, slot02);
            NEA_Free(NEA_TexAllocList, slot1);
        }
        else
        {
            NEA_Free(NEA_TexAllocList, NEA_Texture[slot].address);
        }

        NEA_Texture[slot].address = NULL;
        NEA_Texture[slot].param = 0;
        NEA_Texture[slot].fixed = false;
    }
}

//--------------------------------------------------------------------------

NEA_Material *NEA_MaterialCreate(void)
{
    if (!ne_texture_system_inited)
    {
        NEA_DebugPrint("System not initialized");
        return NULL;
    }

    for (int i = 0; i < NEA_MAX_TEXTURES; i++)
    {
        if (NEA_UserMaterials[i] != NULL)
            continue;

        NEA_Material *mat = calloc(1, sizeof(NEA_Material));
        if (mat == NULL)
        {
            NEA_DebugPrint("Not enough memory");
            return NULL;
        }

        NEA_UserMaterials[i] = mat;
        mat->texindex = NEA_NO_TEXTURE;
        mat->palette = NULL;
        mat->palette_autodelete = false;
        mat->color = NEA_White;
        mat->diffuse_ambient = ne_default_diffuse_ambient;
        mat->specular_emission = ne_default_specular_emission;

        return mat;
    }

    NEA_DebugPrint("No free slots");

    return NULL;
}

void NEA_MaterialRamInit(NEA_Material *material)
{
    NEA_AssertPointer(material, "NULL material pointer");
    NEA_Assert(material->texindex == NEA_NO_TEXTURE,
              "RAM-backing must be enabled before loading a texture");

    material->ram_backed = true;
}

int NEA_MaterialTexVramLoad(NEA_Material *material)
{
    NEA_AssertPointer(material, "NULL material pointer");

    // Nothing has been stashed in RAM, so there is nothing to upload
    if (!material->ram_backed || material->ram_data == NULL)
        return 0;

    // Already resident in VRAM
    if (material->texindex != NEA_NO_TEXTURE)
        return 1;

    // Upload the RAM copy. The ram_backed guard in NEA_MaterialTexLoad() lets
    // this call through because the buffer passed is the material's own
    // ram_data.
    return NEA_MaterialTexLoad(material, material->ram_format,
                              material->ram_sizex, material->ram_sizey,
                              material->ram_flags, material->ram_data);
}

int NEA_MaterialTexVramUnload(NEA_Material *material)
{
    NEA_AssertPointer(material, "NULL material pointer");

    // Only RAM-backed materials can be safely removed from VRAM: the RAM copy
    // is what allows uploading the texture again later.
    if (!material->ram_backed || material->ram_data == NULL)
        return 0;

    // Not currently in VRAM
    if (material->texindex == NEA_NO_TEXTURE)
        return 1;

    ne_texture_delete(material->texindex);
    material->texindex = NEA_NO_TEXTURE;

    return 1;
}

void *NEA_MaterialTexGetData(NEA_Material *material)
{
    NEA_AssertPointer(material, "NULL material pointer");

    if (!material->ram_backed)
        return NULL;

    return material->ram_data;
}

int NEA_MaterialTexBlank(NEA_Material *material, NEA_TextureFormat fmt,
                        int sizeX, int sizeY, NEA_TextureFlags flags)
{
    NEA_AssertPointer(material, "NULL material pointer");
    NEA_Assert(fmt != 0, "No texture format provided");
    NEA_Assert(material->texindex == NEA_NO_TEXTURE,
              "Blank texture must be created before loading a texture");

    material->ram_backed = true;

    uint32_t size = ne_tex_data_size(fmt, sizeX, sizeY);

    // Discard any previous RAM copy and allocate a fresh, zeroed buffer
    free(material->ram_data);
    material->ram_data = malloc(size);
    if (material->ram_data == NULL)
    {
        NEA_DebugPrint("Not enough memory");
        material->ram_size = 0;
        return 0;
    }

    memset(material->ram_data, 0, size);

    material->ram_size = size;
    material->ram_sizex = sizeX;
    material->ram_sizey = sizeY;
    material->ram_format = fmt;
    material->ram_flags = flags;
    material->dirty = false;

    // Create the (blank) VRAM copy so the material is immediately renderable
    return NEA_MaterialTexVramLoad(material);
}

void NEA_MaterialTexSetDirty(NEA_Material *material)
{
    NEA_AssertPointer(material, "NULL material pointer");
    material->dirty = true;
}

int NEA_MaterialTexVramUpdate(NEA_Material *material)
{
    NEA_AssertPointer(material, "NULL material pointer");

    if (!material->ram_backed || material->ram_data == NULL)
        return 0;

    // Nothing has changed since the last upload
    if (!material->dirty)
        return 1;

    // Not currently in VRAM: do a full upload, which allocates a slot
    if (material->texindex == NEA_NO_TEXTURE)
    {
        int ret = NEA_MaterialTexVramLoad(material);
        if (ret)
            material->dirty = false;
        return ret;
    }

    // Compressed textures live in two coupled VRAM slots, which makes an
    // in-place copy awkward. Re-upload them the simple way instead.
    if (material->ram_format == NEA_TEX4X4)
    {
        if (!NEA_MaterialTexVramUnload(material))
            return 0;
        int ret = NEA_MaterialTexVramLoad(material);
        if (ret)
            material->dirty = false;
        return ret;
    }

    // Resident and uncompressed: copy the RAM buffer straight into the existing
    // VRAM slot. The size, format and GPU parameters are unchanged, so there is
    // no need to reallocate or touch the material parameters.
    ne_copy_tex_to_vram(NEA_Texture[material->texindex].address,
                        material->ram_format, material->ram_size,
                        material->ram_data);

    material->dirty = false;

    return 1;
}

void NEA_MaterialSetName(NEA_Material *mat, const char *name)
{
    NEA_AssertPointer(mat, "NULL material pointer");
    NEA_AssertPointer(name, "NULL name pointer");
    strncpy(mat->name, name, NEA_MATERIAL_NAME_LEN - 1);
    mat->name[NEA_MATERIAL_NAME_LEN - 1] = '\0';
}

const char *NEA_MaterialGetName(const NEA_Material *mat)
{
    NEA_AssertPointer(mat, "NULL material pointer");
    return mat->name;
}

NEA_Material *NEA_MaterialFindByName(const char *name)
{
    NEA_AssertPointer(name, "NULL name pointer");

    if (!ne_texture_system_inited)
        return NULL;

    for (int i = 0; i < NEA_MAX_TEXTURES; i++)
    {
        if (NEA_UserMaterials[i] == NULL)
            continue;
        if (strcmp(NEA_UserMaterials[i]->name, name) == 0)
            return NEA_UserMaterials[i];
    }

    return NULL;
}

void NEA_MaterialColorSet(NEA_Material *tex, u32 color)
{
    NEA_AssertPointer(tex, "NULL pointer");
    tex->color = color;
}

void NEA_MaterialColorDelete(NEA_Material *tex)
{
    NEA_AssertPointer(tex, "NULL pointer");
    tex->color = NEA_White;
}

void NEA_MaterialTexUse(const NEA_Material *tex)
{
    // Deliberately binds nothing but the image: no colours, no properties, and
    // no palette. A texture/palette animation swaps those independently, and a
    // flipbook whose frames share one palette should not have it rebound on
    // every frame.
    if (tex == NULL || tex->texindex == NEA_NO_TEXTURE)
    {
        GFX_TEX_FORMAT = 0;
        return;
    }

    GFX_TEX_FORMAT = NEA_Texture[tex->texindex].param;
}

void NEA_MaterialSetTexGen(NEA_Material *tex, NEA_TextureFlags texgen)
{
    NEA_AssertPointer(tex, "NULL pointer");
    NEA_Assert(tex->texindex != NEA_NO_TEXTURE, "No texture asigned to material");

    // Texgen lives in bits 30-31 of the texture parameter word.
    u32 param = NEA_Texture[tex->texindex].param & ~(3U << 30);
    NEA_Texture[tex->texindex].param = param | (texgen & (3U << 30));
}

#ifdef NEA_BLOCKSDS

// Chunk IDs of the GRF container. libnds builds these the same way but doesn't
// export them, so they are repeated here for the sanity check below.
#define NEA_GRF_CHUNK_ID(a, b, c, d) \
    ((uint32_t)((a) | ((b) << 8) | ((c) << 16) | ((d) << 24)))

#define NEA_GRF_ID_RIFF NEA_GRF_CHUNK_ID('R', 'I', 'F', 'F')
#define NEA_GRF_ID_GRF  NEA_GRF_CHUNK_ID('G', 'R', 'F', ' ')

// Checks that a buffer really holds a whole GRF file before it is decoded.
//
// grfLoadMemEx() walks the chunk list using the lengths stored inside the file
// and never compares them against the size of the buffer it was given, so a
// truncated or corrupt file makes it read (and decompress) past the end of the
// allocation. Only the loaders that decode from RAM can check this, because
// they are the only ones that know how many bytes were actually read.
//
// This bounds the outer walk only. The length of each individual chunk is still
// up to the library to trust.
//
// Shared with the NEAHw2D asynchronous GRF loaders, hence the internal name.
bool __NEA_GRFBufferIsSane(const void *buffer, size_t size)
{
    // "RIFF" + size + "GRF ", which is the smallest possible valid header.
    if (buffer == NULL || size < 12)
        return false;

    const uint32_t *words = buffer;

    if (words[0] != NEA_GRF_ID_RIFF)
        return false;

    if (words[2] != NEA_GRF_ID_GRF)
        return false;

    // grfLoadMemEx() stops at 'buffer + riff_size + 8', so everything up to
    // there has to be inside the buffer.
    uint32_t riff_size = words[1];
    if (riff_size > size - 8)
        return false;

    return true;
}

// Applies the data decoded from a GRF file to a material (and palette). The
// decoded buffers (gfxDst, pidxDst, palDst) are not freed by this function.
//
// This is shared by NEA_MaterialTexLoadGRF() (synchronous, decodes with
// grfLoadPath()) and the asynchronous GRF loader (decodes with grfLoadMemEx()
// in the worker thread).
static int ne_grf_apply(NEA_Material *tex, NEA_Palette *pal,
                        NEA_TextureFlags flags, const GRFHeader *header,
                        void *gfxDst, void *pidxDst,
                        void *palDst, size_t palSize)
{
    if (header->flags & GRF_FLAG_COLOR0_TRANSPARENT)
        flags |= GL_TEXTURE_COLOR0_TRANSPARENT;

    if (gfxDst == NULL)
    {
        NEA_DebugPrint("No graphics found in GRF file");
        return 0;
    }

    NEA_TextureFormat fmt;
    switch (header->gfxAttr)
    {
        case GRF_TEXFMT_A5I3:
            fmt = NEA_A5PAL8;
            break;
        case GRF_TEXFMT_A3I5:
            fmt = NEA_A3PAL32;
            break;
        case GRF_TEXFMT_4x4:
            fmt = NEA_TEX4X4;
            break;
        case 16:
            fmt = NEA_A1RGB5;
            break;
        case 8:
            fmt = NEA_PAL256;
            break;
        case 4:
            fmt = NEA_PAL16;
            break;
        case 2:
            fmt = NEA_PAL4;
            break;
        default:
            NEA_DebugPrint("Invalid format in GRF file");
            return 0;
    }

    if (header->gfxAttr == GRF_TEXFMT_4x4)
    {
        // A tex4x4 texture needs the palette index data from the PIDX chunk.
        // A file that declares the format but doesn't carry one would be
        // passed to the loader as a NULL second half.
        if (pidxDst == NULL)
        {
            NEA_DebugPrint("Tex4x4 GRF file without PIDX data");
            return 0;
        }

        if (NEA_MaterialTex4x4Load(tex, header->gfxWidth, header->gfxHeight,
                           flags, gfxDst, pidxDst) == 0)
        {
            NEA_DebugPrint("Failed to load GRF texture");
            return 0;
        }
    }
    else
    {
        if (NEA_MaterialTexLoad(tex, fmt, header->gfxWidth, header->gfxHeight,
                            flags, gfxDst) == 0)
        {
            NEA_DebugPrint("Failed to load GRF texture");
            return 0;
        }
    }

    // If there is no palette to be loaded there is nothing else to do
    if (palDst == NULL)
        return 1; // Success

    // There is a palette to load.

    // If the user has provided a palette object, use that one to store the
    // palette. If not, create a palette object and mark it to be autodeleted
    // when the material is deleted.
    bool create_palette = false;

    if (pal == NULL)
    {
        NEA_DebugPrint("GRF with a palette, but no palette object provided");
        create_palette = true;
    }

    if (create_palette)
    {
        pal = NEA_PaletteCreate();
        if (pal == NULL)
        {
            NEA_DebugPrint("Not enough memory for palette object");
            return 0;
        }
    }

    if (NEA_PaletteLoadSize(pal, palDst, palSize, fmt) == 0)
    {
        NEA_DebugPrint("Failed to load GRF palette");
        if (create_palette)
            NEA_PaletteDelete(pal);
        return 0;
    }

    NEA_MaterialSetPalette(tex, pal);

    if (create_palette)
        NEA_MaterialAutodeletePalette(tex);

    return 1; // Success
}
#endif // NEA_BLOCKSDS

int NEA_MaterialTexLoadGRF(NEA_Material *tex, NEA_Palette *pal,
                          NEA_TextureFlags flags, const char *path)
{
#ifndef NEA_BLOCKSDS
    (void)tex;
    (void)pal;
    (void)flags;
    (void)path;
    NEA_DebugPrint("%s only supported in BlocksDS", __func__);
    return 0;
#else // NEA_BLOCKSDS
    NEA_AssertPointer(tex, "NULL material pointer");
    NEA_AssertPointer(path, "NULL path pointer");

    void *gfxDst = NULL;
    void *pidxDst = NULL;
    void *palDst = NULL;
    size_t palSize = 0;

    GRFHeader header = { 0 };
    GRFError err = grfLoadPath(path, &header, &gfxDst, NULL, &pidxDst, NULL,
                               &palDst, &palSize);
    if (err != GRF_NO_ERROR)
    {
        NEA_DebugPrint("Couldn't load GRF file: %d", err);
        free(gfxDst);
        free(pidxDst);
        free(palDst);
        return 0;
    }

    int ret = ne_grf_apply(tex, pal, flags, &header, gfxDst, pidxDst,
                           palDst, palSize);

    free(gfxDst);
    free(pidxDst);
    free(palDst);
    return ret;
#endif // NEA_BLOCKSDS
}

int NEA_MaterialTexLoadFAT(NEA_Material *tex, NEA_TextureFormat fmt,
                          int sizeX, int sizeY, NEA_TextureFlags flags,
                          const char *path)
{
    NEA_AssertPointer(tex, "NULL material pointer");
    NEA_AssertPointer(path, "NULL path pointer");
    NEA_Assert(sizeX > 0 && sizeY > 0, "Size must be positive");

    void *ptr = NEA_FATLoadData(path);
    if (ptr == NULL)
    {
        NEA_DebugPrint("Couldn't load file from FAT");
        return 0;
    }

    int ret = NEA_MaterialTexLoad(tex, fmt, sizeX, sizeY, flags, ptr);
    free(ptr);

    return ret;
}

int NEA_MaterialTex4x4LoadFAT(NEA_Material *tex, int sizeX, int sizeY,
                             NEA_TextureFlags flags, const char *path02,
                             const char *path1)
{
    NEA_AssertPointer(tex, "NULL material pointer");
    NEA_AssertPointer(path02, "NULL path02 pointer");
    NEA_AssertPointer(path1, "NULL path1 pointer");
    NEA_Assert(sizeX > 0 && sizeY > 0, "Size must be positive");

    void *texture02 = NEA_FATLoadData(path02);
    if (texture02 == NULL)
    {
        NEA_DebugPrint("Couldn't load file from FAT");
        return 0;
    }

    void *texture1 = NEA_FATLoadData(path1);
    if (texture1 == NULL)
    {
        NEA_DebugPrint("Couldn't load file from FAT");
        free(texture02);
        return 0;
    }

    int ret = NEA_MaterialTex4x4Load(tex, sizeX, sizeY, flags, texture02,
                                    texture1);

    free(texture02);
    free(texture1);

    return ret;
}

//--------------------------------------------------------------------------
// Asynchronous texture loading
//--------------------------------------------------------------------------

// Parameters of an asynchronous NEA_MaterialTexLoadFATAsync() job.
typedef struct {
    NEA_Material *tex;
    NEA_TextureFormat fmt;
    int sizeX, sizeY;
    NEA_TextureFlags flags;
} ne_async_tex_param;

// Runs on the main thread during the vertical blank: uploads the loaded data
// to VRAM with the regular (synchronous) texture loader.
static void ne_async_tex_finalize(NEA_AsyncFile *job)
{
    ne_async_tex_param *p = __NEA_AsyncParam(job);
    char *buffer = __NEA_AsyncBuffer(job, NULL);

    int ret = NEA_MaterialTexLoad(p->tex, p->fmt, p->sizeX, p->sizeY,
                                  p->flags, buffer);
    __NEA_AsyncSetResult(job, ret);
}

NEA_AsyncFile *NEA_MaterialTexLoadFATAsync(NEA_Material *tex,
                NEA_TextureFormat fmt, int sizeX, int sizeY,
                NEA_TextureFlags flags, const char *path)
{
    NEA_AssertPointer(tex, "NULL material pointer");
    NEA_AssertPointer(path, "NULL path pointer");
    NEA_Assert(sizeX > 0 && sizeY > 0, "Size must be positive");

    ne_async_tex_param *p = malloc(sizeof(ne_async_tex_param));
    if (p == NULL)
    {
        NEA_DebugPrint("Not enough memory");
        return NULL;
    }

    p->tex = tex;
    p->fmt = fmt;
    p->sizeX = sizeX;
    p->sizeY = sizeY;
    p->flags = flags;

    NEA_AsyncFile *job = __NEA_AsyncQueue(path, NULL, ne_async_tex_finalize,
                                          NULL, p, tex);
    if (job == NULL)
        free(p);

    return job;
}

// Parameters of an asynchronous NEA_MaterialTex4x4LoadFATAsync() job.
typedef struct {
    NEA_Material *tex;
    int sizeX, sizeY;
    NEA_TextureFlags flags;
    char *path1;        // Path of the second file (owned, freed after read)
    void *texture1;     // Contents of the second file (owned)
} ne_async_tex4x4_param;

// Runs in the worker thread: loads the second file of a tex4x4 texture.
static bool ne_async_tex4x4_stage2(NEA_AsyncFile *job)
{
    ne_async_tex4x4_param *p = __NEA_AsyncParam(job);

    // Chunked and cancellable, unlike NEA_FATLoadData(): this is the second
    // half of a texture that may be several hundred KB, and reading it in one
    // blocking call would stall every other cothread for the whole transfer and
    // ignore a delete of the material it is being loaded into.
    p->texture1 = __NEA_AsyncReadFile(job, p->path1, NULL);
    free(p->path1);
    p->path1 = NULL;

    if (p->texture1 == NULL)
    {
        NEA_DebugPrint("Couldn't load second tex4x4 file");
        return false;
    }

    return true;
}

static void ne_async_tex4x4_finalize(NEA_AsyncFile *job)
{
    ne_async_tex4x4_param *p = __NEA_AsyncParam(job);
    char *buffer = __NEA_AsyncBuffer(job, NULL);

    int ret = NEA_MaterialTex4x4Load(p->tex, p->sizeX, p->sizeY, p->flags,
                                     buffer, p->texture1);
    __NEA_AsyncSetResult(job, ret);

    free(p->texture1);
    p->texture1 = NULL;
}

static void ne_async_tex4x4_discard(NEA_AsyncFile *job)
{
    ne_async_tex4x4_param *p = __NEA_AsyncParam(job);
    free(p->path1);
    free(p->texture1);
}

NEA_AsyncFile *NEA_MaterialTex4x4LoadFATAsync(NEA_Material *tex,
                int sizeX, int sizeY, NEA_TextureFlags flags,
                const char *path02, const char *path1)
{
    NEA_AssertPointer(tex, "NULL material pointer");
    NEA_AssertPointer(path02, "NULL path02 pointer");
    NEA_AssertPointer(path1, "NULL path1 pointer");
    NEA_Assert(sizeX > 0 && sizeY > 0, "Size must be positive");

    ne_async_tex4x4_param *p = calloc(1, sizeof(ne_async_tex4x4_param));
    if (p == NULL)
    {
        NEA_DebugPrint("Not enough memory");
        return NULL;
    }

    size_t len = strlen(path1) + 1;
    p->path1 = malloc(len);
    if (p->path1 == NULL)
    {
        NEA_DebugPrint("Not enough memory");
        free(p);
        return NULL;
    }
    memcpy(p->path1, path1, len);

    p->tex = tex;
    p->sizeX = sizeX;
    p->sizeY = sizeY;
    p->flags = flags;

    NEA_AsyncFile *job = __NEA_AsyncQueue(path02, ne_async_tex4x4_stage2,
                                          ne_async_tex4x4_finalize,
                                          ne_async_tex4x4_discard, p, tex);
    if (job == NULL)
    {
        free(p->path1);
        free(p);
    }

    return job;
}

#ifdef NEA_BLOCKSDS
// Parameters of an asynchronous NEA_MaterialTexLoadGRFAsync() job.
typedef struct {
    NEA_Material *tex;
    NEA_Palette *pal;
    NEA_TextureFlags flags;
    GRFHeader header;
    void *gfxDst;       // Decoded buffers (owned)
    void *pidxDst;
    void *palDst;
    size_t palSize;
} ne_async_grf_param;

// Runs in the worker thread: decodes the GRF file from the RAM buffer.
static bool ne_async_grf_stage2(NEA_AsyncFile *job)
{
    ne_async_grf_param *p = __NEA_AsyncParam(job);

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

    GRFError err = grfLoadMemEx(raw, &p->header,
                                &p->gfxDst, NULL, &p->pidxDst, NULL,
                                &p->palDst, &p->palSize, NULL, NULL,
                                NULL, NULL);

    // The raw GRF file isn't needed once it has been decoded.
    __NEA_AsyncFreeBuffer(job);

    if (err != GRF_NO_ERROR)
    {
        NEA_DebugPrint("Couldn't decode GRF file: %d", err);
        return false;
    }

    return true;
}

static void ne_async_grf_finalize(NEA_AsyncFile *job)
{
    ne_async_grf_param *p = __NEA_AsyncParam(job);

    int ret = ne_grf_apply(p->tex, p->pal, p->flags, &p->header,
                           p->gfxDst, p->pidxDst, p->palDst, p->palSize);
    __NEA_AsyncSetResult(job, ret);

    free(p->gfxDst);
    free(p->pidxDst);
    free(p->palDst);
    p->gfxDst = NULL;
    p->pidxDst = NULL;
    p->palDst = NULL;
}

static void ne_async_grf_discard(NEA_AsyncFile *job)
{
    ne_async_grf_param *p = __NEA_AsyncParam(job);
    free(p->gfxDst);
    free(p->pidxDst);
    free(p->palDst);
}
#endif // NEA_BLOCKSDS

NEA_AsyncFile *NEA_MaterialTexLoadGRFAsync(NEA_Material *tex, NEA_Palette *pal,
                NEA_TextureFlags flags, const char *path)
{
#ifndef NEA_BLOCKSDS
    (void)tex;
    (void)pal;
    (void)flags;
    (void)path;
    NEA_DebugPrint("%s only supported in BlocksDS", __func__);
    return NULL;
#else // NEA_BLOCKSDS
    NEA_AssertPointer(tex, "NULL material pointer");
    NEA_AssertPointer(path, "NULL path pointer");

    ne_async_grf_param *p = calloc(1, sizeof(ne_async_grf_param));
    if (p == NULL)
    {
        NEA_DebugPrint("Not enough memory");
        return NULL;
    }

    p->tex = tex;
    p->pal = pal;
    p->flags = flags;

    NEA_AsyncFile *job = __NEA_AsyncQueue(path, ne_async_grf_stage2,
                                          ne_async_grf_finalize,
                                          ne_async_grf_discard, p, tex);
    if (job == NULL)
    {
        free(p);
        return NULL;
    }

    // The finalize step writes into the palette too, so deleting it must abort
    // this load as well.
    __NEA_AsyncAddTarget(job, pal);

    return job;
#endif // NEA_BLOCKSDS
}

// This function takes as argument the size of the chunk of the compressed
// texture chunk that goes into slots 0 or 2. The size that goes into slot 1 is
// always half of this size, so it isn't needed to provide it.
//
// It returns 0 on success, as well as pointers to the address where both chunks
// need to be copied.
static int ne_alloc_compressed_tex(size_t size, void **slot02, void **slot1)
{
    size_t size02 = size;
    size_t size1 = size / 2;

    // First, try with slot 0 + slot 1
    // -------------------------------

    // Get the first valid range in slot 0
    void *addr0 = NEA_AllocFindInRange(NEA_TexAllocList, VRAM_A, VRAM_B, size02);
    if (addr0 != NULL)
    {
        // Only use the first half of slot 1 for slot 0
        void *addr1;
        void *addr1_end = (void *)((uintptr_t)VRAM_B + (64 * 1024));

        while (1)
        {
            // Get the address in bank 1 assigned to the current bank 0 address
            addr1 = slot0_to_slot1(addr0);

            // Check if this address is free and has enough space
            addr1 = NEA_AllocFindInRange(NEA_TexAllocList, addr1, addr1_end, size1);
            if (addr1 == NULL)
                break;

            // If both addresses match, both of them are free
            if (addr1 == slot0_to_slot1(addr0))
            {
                *slot02 = addr0;
                *slot1 = addr1;
                return 0;
            }

            // Get the address in bank 0 assigned to the current bank 1 address
            addr0 = slot1_to_slot0(addr1);

            // Check if this address is free and has enough space
            addr0 = NEA_AllocFindInRange(NEA_TexAllocList, addr0, VRAM_B, size02);
            if (addr0 == NULL)
                break;

            // If both addresses match, both of them are free
            if (addr1 == slot0_to_slot1(addr0))
            {
                *slot02 = addr0;
                *slot1 = addr1;
                return 0;
            }
        }
    }

    // Then, try with slot 2 + slot 1
    // ------------------------------

    // Get the first valid range in slot 2
    void *addr2 = NEA_AllocFindInRange(NEA_TexAllocList, VRAM_C, VRAM_D, size02);
    if (addr2 == NULL)
        return -1;

    // Only use the second half of slot 1 for slot 2
    void *addr1;
    void *addr1_end = VRAM_C;

    while (1)
    {
        // Get the address in bank 1 assigned to the current bank 2 address
        addr1 = slot2_to_slot1(addr2);

        // Check if this address is free and has enough space
        addr1 = NEA_AllocFindInRange(NEA_TexAllocList, addr1, addr1_end, size1);
        if (addr1 == NULL)
            break;

        // If both addresses match, both of them are free
        if (addr1 == slot2_to_slot1(addr2))
        {
            *slot02 = addr2;
            *slot1 = addr1;
            return 0;
        }

        // Get the address in bank 2 assigned to the current bank 1 address
        addr2 = slot1_to_slot2(addr1);

        // Check if this address is free and has enough space (slot 2 is
        // bank C: the range ends where bank C does)
        addr2 = NEA_AllocFindInRange(NEA_TexAllocList, addr2, VRAM_D, size02);
        if (addr2 == NULL)
            break;

        // If both addresses match, both of them are free
        if (addr1 == slot2_to_slot1(addr2))
        {
            *slot02 = addr2;
            *slot1 = addr1;
            return 0;
        }
    }

    return -1;
}

// Keeps a private RAM copy of a texture for a RAM-backed material instead of
// uploading it to VRAM. For non-compressed textures pass the data in texture02
// and NULL in texture1. For tex4x4 textures, either pass the two parts
// separately (texture02 = slot 0/2 data, texture1 = slot 1 data) or pass the
// concatenated buffer in texture02 with texture1 as NULL. The stored copy is
// always laid out as [slot 0/2 data][slot 1 data], which is exactly what the
// upload path (NEA_MaterialTexVramLoad -> NEA_MaterialTexLoad) expects.
static int ne_material_tex_stash(NEA_Material *tex, NEA_TextureFormat fmt,
                                 int sizeX, int sizeY, NEA_TextureFlags flags,
                                 const void *texture02, const void *texture1)
{
    // Compute the number of bytes to keep in RAM
    u32 size02, size1;
    if (fmt == NEA_TEX4X4)
    {
        // Compressed: main data (slot 0/2) plus index data (slot 1)
        size02 = (sizeX * sizeY) >> 2;
        size1 = size02 >> 1;
    }
    else
    {
        static const int size_shift[] = {
            0, // Nothing
            1, // NEA_A3PAL32
            3, // NEA_PAL4
            2, // NEA_PAL16
            1, // NEA_PAL256
            0, // NEA_TEX4X4 (unused here)
            1, // NEA_A5PAL8
            0, // NEA_A1RGB5
            0, // NEA_RGB5
        };
        size02 = (sizeX * sizeY << 1) >> size_shift[fmt];
        size1 = 0;
    }

    u32 size = size02 + size1;

    // Discard any previous RAM copy (this supports reloading with new data)
    free(tex->ram_data);
    tex->ram_data = malloc(size);
    if (tex->ram_data == NULL)
    {
        NEA_DebugPrint("Not enough memory");
        tex->ram_size = 0;
        return 0;
    }

    if (texture1 != NULL)
    {
        // Two separate buffers (direct NEA_MaterialTex4x4Load() call)
        memcpy(tex->ram_data, texture02, size02);
        memcpy((char *)tex->ram_data + size02, texture1, size1);
    }
    else
    {
        // Single contiguous buffer (also covers a concatenated tex4x4 buffer)
        memcpy(tex->ram_data, texture02, size);
    }

    tex->ram_size = size;
    tex->ram_sizex = sizeX;
    tex->ram_sizey = sizeY;
    tex->ram_format = fmt;
    tex->ram_flags = flags;

    // If the texture was already resident in VRAM, drop it so the next
    // NEA_MaterialTexVramLoad() re-uploads this new data instead of keeping the
    // stale VRAM copy.
    if (tex->texindex != NEA_NO_TEXTURE)
    {
        ne_texture_delete(tex->texindex);
        tex->texindex = NEA_NO_TEXTURE;
    }

    return 1;
}

int NEA_MaterialTex4x4Load(NEA_Material *tex, int sizeX, int sizeY,
                          NEA_TextureFlags flags, const void *texture02,
                          const void *texture1)
{
    NEA_AssertPointer(tex, "NULL material pointer");

    // If this material is RAM-backed, keep the texture in main RAM instead of
    // uploading it to VRAM. The upload path re-invokes this with
    // texture02 == tex->ram_data, which falls through to the real upload.
    if (tex->ram_backed && texture02 != tex->ram_data)
        return ne_material_tex_stash(tex, NEA_TEX4X4, sizeX, sizeY, flags,
                                     texture02, texture1);

    // The width of a tex4x4 texture must be a power of 2. The height doesn't
    // need to be one: the texels are stored as rows of 4x4 blocks, so a height
    // that is a multiple of 4 can be stored trimmed (ptexconv -tt) and the GPU
    // told the next power of 2 instead. See NEA_MaterialTexLoad().
    if (ne_is_valid_tex_size(sizeX) != sizeX)
    {
        NEA_DebugPrint("Width of tex4x4 textures must be a power of 2");
        return 0;
    }

    if ((sizeY < 4) || (sizeY & 3) || (ne_is_valid_tex_size(sizeY) == 0))
    {
        NEA_DebugPrint("Height of tex4x4 textures must be a multiple of 4, up to 1024");
        return 0;
    }

    ne_warn_trimmed_wrap_t(sizeY, flags);

    // Check if a texture exists
    if (tex->texindex != NEA_NO_TEXTURE)
        ne_texture_delete(tex->texindex);

    // Get free slot
    tex->texindex = NEA_NO_TEXTURE;
    for (int i = 0; i < NEA_MAX_TEXTURES; i++)
    {
        if (NEA_Texture[i].address == NULL)
        {
            tex->texindex = i;
            break;
        }
    }

    if (tex->texindex == NEA_NO_TEXTURE)
    {
        NEA_DebugPrint("No free slots");
        return 0;
    }

    size_t size02 = (sizeX * sizeY) >> 2;
    size_t size1 = size02 >> 1;

    void *slot02, *slot1;
    int ret = ne_alloc_compressed_tex(size02, &slot02, &slot1);
    if (ret != 0)
    {
        tex->texindex = NEA_NO_TEXTURE; // (the slot was never taken)
        NEA_DebugPrint("Can't find space for compressed texture");
        return 0;
    }

    ret = NEA_AllocAddress(NEA_TexAllocList, slot02, size02);
    if (ret != 0)
    {
        tex->texindex = NEA_NO_TEXTURE; // (the slot was never taken)
        NEA_DebugPrint("Can't allocate slot 0/2");
        return 0;
    }

    ret = NEA_AllocAddress(NEA_TexAllocList, slot1, size1);
    if (ret != 0)
    {
        NEA_Free(NEA_TexAllocList, slot02);
        tex->texindex = NEA_NO_TEXTURE; // (the slot was never taken)
        NEA_DebugPrint("Can't allocate slot 1");
        return 0;
    }

    // Save information
    int slot = tex->texindex;
    NEA_Texture[slot].sizex = sizeX;
    NEA_Texture[slot].sizey = sizeY;
    NEA_Texture[slot].address = slot02;
    NEA_Texture[slot].uses = 1; // Initially only this material uses the texture

    // Unlock the banks the two parts go to for writing, and no others
    ne_tex_unlock_t unlock;
    ne_tex_unlock(&unlock, ne_tex_bank_mask(slot02, size02) |
                           ne_tex_bank_mask(slot1, size1));

    swiCopy(texture02, slot02, (size02 >> 2) | COPY_MODE_WORD);
    swiCopy(texture1, slot1, (size1 >> 2) | COPY_MODE_WORD);

    ne_tex_lock(&unlock);

    int hardware_size_y = ne_is_valid_tex_size(sizeY);
    ne_set_material_tex_param(tex, sizeX, hardware_size_y, slot02,
                              NEA_TEX4X4, flags);

    return 1;
}

// Copies raw texture data from RAM into an already-allocated VRAM address. It
// remaps the primary VRAM banks the data goes to (and only those) to LCD mode
// for the duration of the copy: what is drawn with textures in those banks is
// drawn without them for as long, so a copy into a bank that is in use on the
// screen is best done during the vertical blank. Handles the
// NEA_RGB5 special case, where each alpha bit is forced to 1 so the data becomes
// NEA_A1RGB5 in VRAM.
static void ne_copy_tex_to_vram(void *addr, NEA_TextureFormat fmt,
                                uint32_t size, const void *texture)
{
    // Unlock the banks the texture goes to for writing, and no others
    ne_tex_unlock_t unlock;
    ne_tex_unlock(&unlock, ne_tex_bank_mask(addr, size));

    if (fmt == NEA_RGB5)
    {
        uint32_t *src = (uint32_t *)texture;
        uint32_t *dest = addr;
        uint32_t words = size >> 2; // Four bytes each iteration
        while (words--)
            *dest++ = *src++ | ((1 << 15) | (1 << 31));
    }
    else
    {
        swiCopy((u32 *)texture, addr, (size >> 2) | COPY_MODE_WORD);
    }

    ne_tex_lock(&unlock);
}

int NEA_MaterialTexLoad(NEA_Material *tex, NEA_TextureFormat fmt,
                       int sizeX, int sizeY, NEA_TextureFlags flags,
                       const void *texture)
{
    NEA_AssertPointer(tex, "NULL material pointer");
    NEA_Assert(fmt != 0, "No texture format provided");

    // If this material is RAM-backed, keep the texture in main RAM instead of
    // uploading it to VRAM. NEA_MaterialTexVramLoad() re-invokes this with
    // texture == tex->ram_data, which falls through to the real upload.
    if (tex->ram_backed && texture != tex->ram_data)
        return ne_material_tex_stash(tex, fmt, sizeX, sizeY, flags,
                                     texture, NULL);

    if (fmt == NEA_TEX4X4)
    {
        // Split tex4x4 texture into its two parts, that have been concatenated

        size_t size02 = (sizeX * sizeY) >> 2;

        const void *texture02 = texture;
        const void *texture1 = (const void *)((uintptr_t)texture + size02);

        return NEA_MaterialTex4x4Load(tex, sizeX, sizeY, flags,
                                     texture02, texture1);
    }

    // The width of a texture must be a power of 2. The height doesn't need to
    // be a power of 2, but we will have to cheat later and make the DS believe
    // it is a power of 2.
    if (ne_is_valid_tex_size(sizeX) != sizeX)
    {
        NEA_DebugPrint("Width of textures must be a power of 2");
        return 0;
    }

    // ne_is_valid_tex_size() returns 0 for anything over 1024. Without this
    // check ne_tex_raw_size(0) would encode an 8 pixel tall texture instead of
    // failing, and the load would silently produce garbage.
    if ((sizeY < 1) || (ne_is_valid_tex_size(sizeY) == 0))
    {
        NEA_DebugPrint("Height of textures must be between 1 and 1024");
        return 0;
    }

    ne_warn_trimmed_wrap_t(sizeY, flags);

    // Check if a texture exists
    if (tex->texindex != NEA_NO_TEXTURE)
        ne_texture_delete(tex->texindex);

    // Get free slot
    tex->texindex = NEA_NO_TEXTURE;
    for (int i = 0; i < NEA_MAX_TEXTURES; i++)
    {
        if (NEA_Texture[i].address == NULL)
        {
            tex->texindex = i;
            break;
        }
    }

    if (tex->texindex == NEA_NO_TEXTURE)
    {
        NEA_DebugPrint("No free slots");
        return 0;
    }

    // All non-compressed texture types are handled here

    const int size_shift[] = {
        0, // Nothing
        1, // NEA_A3PAL32
        3, // NEA_PAL4
        2, // NEA_PAL16
        1, // NEA_PAL256
        0, // NEA_TEX4X4 (This value isn't used)
        1, // NEA_A5PAL8
        0, // NEA_A1RGB5
        0, // NEA_RGB5
    };

    uint32_t size = (sizeX * sizeY << 1) >> size_shift[fmt];

    // This pointer must be aligned to 8 bytes at least
    void *addr = ne_alloc_plain(size);
    if (!addr)
    {
        tex->texindex = NEA_NO_TEXTURE;
        NEA_DebugPrint("Not enough memory");
        return 0;
    }

    // Save information
    int slot = tex->texindex;
    NEA_Texture[slot].sizex = sizeX;
    NEA_Texture[slot].sizey = sizeY;
    NEA_Texture[slot].address = addr;
    NEA_Texture[slot].uses = 1; // Initially only this material uses the texture

    ne_copy_tex_to_vram(addr, fmt, size, texture);

    // NEA_RGB5 is stored in VRAM as NEA_A1RGB5 (see ne_copy_tex_to_vram())
    NEA_TextureFormat hw_fmt = (fmt == NEA_RGB5) ? NEA_A1RGB5 : fmt;
    int hardware_size_y = ne_is_valid_tex_size(sizeY);
    ne_set_material_tex_param(tex, sizeX, hardware_size_y, addr, hw_fmt, flags);

    return 1;
}

void NEA_MaterialAutodeletePalette(NEA_Material *mat)
{
    NEA_AssertPointer(mat, "NULL material pointer");

    mat->palette_autodelete = true;
}

void NEA_MaterialClone(NEA_Material *source, NEA_Material *dest)
{
    NEA_AssertPointer(source, "NULL source pointer");
    NEA_AssertPointer(dest, "NULL dest pointer");

    // Increase count of materials using this texture (if there is one resident
    // in VRAM). A RAM-backed material may have no texture in VRAM at this point.
    if (source->texindex != NEA_NO_TEXTURE)
        NEA_Texture[source->texindex].uses++;

    memcpy(dest, source, sizeof(NEA_Material));

    // The clone must not inherit ownership of the source's private RAM copy (it
    // would be freed twice). The clone shares the VRAM texture by reference
    // count only and is not RAM-backed.
    dest->ram_backed = false;
    dest->ram_data = NULL;
    dest->ram_size = 0;
    dest->dirty = false;
}

void NEA_MaterialSetPalette(NEA_Material *tex, NEA_Palette *pal)
{
    NEA_AssertPointer(tex, "NULL material pointer");
    NEA_AssertPointer(pal, "NULL palette pointer");
    // A RAM-backed material may not have a texture in VRAM yet (texindex ==
    // NEA_NO_TEXTURE) while its texture is stashed in RAM. That is valid here.
    NEA_Assert(tex->texindex != NEA_NO_TEXTURE || tex->ram_backed,
              "No texture asigned to material");
    tex->palette = pal;
}

void NEA_MaterialUse(const NEA_Material *tex)
{
    if (tex == NULL)
    {
        GFX_TEX_FORMAT = 0;
        GFX_COLOR = NEA_White;
        GFX_DIFFUSE_AMBIENT = ne_default_diffuse_ambient;
        GFX_SPECULAR_EMISSION = ne_default_specular_emission;
        return;
    }

    GFX_DIFFUSE_AMBIENT = tex->diffuse_ambient;
    GFX_SPECULAR_EMISSION = tex->specular_emission;

    // A material with no texture resident in VRAM (e.g. a RAM-backed material
    // that has been removed from VRAM with NEA_MaterialTexVramUnload()) is
    // rendered untextured using its color.
    if (tex->texindex == NEA_NO_TEXTURE)
    {
        GFX_COLOR = tex->color;
        GFX_TEX_FORMAT = 0;
        return;
    }

    if (tex->palette)
        NEA_PaletteUse(tex->palette);

    GFX_COLOR = tex->color;
    GFX_TEX_FORMAT = NEA_Texture[tex->texindex].param;
}

uint32_t NEA_MaterialGetTexParam(const NEA_Material *tex)
{
    NEA_AssertPointer(tex, "NULL pointer");

    if (tex->texindex == NEA_NO_TEXTURE)
        return 0;

    return NEA_Texture[tex->texindex].param;
}

static void ne_defrag_reset(void);

int NEA_TextureSystemReset(int max_textures, int max_palettes,
                          NEA_VRAMBankFlags bank_flags)
{
    if (ne_texture_system_inited)
        NEA_TextureSystemEnd();

    NEA_Assert((bank_flags & 0xF) != 0, "No VRAM banks selected");

    if (max_textures < 1)
        NEA_MAX_TEXTURES = NEA_DEFAULT_TEXTURES;
    else
        NEA_MAX_TEXTURES = max_textures;

    if (NEA_PaletteSystemReset(max_palettes) != 0)
        return -1;

    NEA_Texture = calloc(NEA_MAX_TEXTURES, sizeof(ne_textureinfo_t));
    NEA_UserMaterials = calloc(NEA_MAX_TEXTURES, sizeof(NEA_UserMaterials));
    if ((NEA_Texture == NULL) || (NEA_UserMaterials == NULL))
        goto cleanup;

    if (NEA_AllocInit(&NEA_TexAllocList, VRAM_A, VRAM_E) != 0)
        goto cleanup;

    // Prevent user from not selecting any bank
    if ((bank_flags & 0xF) == 0)
        bank_flags = NEA_VRAM_ABCD;

    // Exclude banks claimed by the Hw2D system (weak reference)
    extern NEA_VRAMBankFlags NEA_Hw2DGetClaimedBanks(void) __attribute__((weak));
    if (NEA_Hw2DGetClaimedBanks)
        bank_flags &= ~NEA_Hw2DGetClaimedBanks();

    // Same for the PostFX frame capture, which owns two of A-D while running.
    extern NEA_VRAMBankFlags NEA_PostFXGetCaptureBanks(void) __attribute__((weak));
    if (NEA_PostFXGetCaptureBanks)
        bank_flags &= ~NEA_PostFXGetCaptureBanks();

    // And for the DSP render target / full-screen effects (NEADspFX.c)
    extern NEA_VRAMBankFlags NEA_DspFXGetReservedBanks(void) __attribute__((weak));
    if (NEA_DspFXGetReservedBanks)
        bank_flags &= ~NEA_DspFXGetReservedBanks();

    // VRAM_C and VRAM_D can't be used in dual 3D mode (they are used for
    // framebuffers). In two-pass FIFO/DMA modes, only VRAM_D is reserved (for
    // capture), so VRAM_C is available for textures. In two-pass FB mode,
    // both VRAM_C and VRAM_D alternate as framebuffers/capture destinations.
    NEA_ExecutionModes mode = NEA_CurrentExecutionMode();
    if (mode == NEA_ModeSingle3D_TwoPass
        || mode == NEA_ModeSingle3D_TwoPass_DMA)
        bank_flags &= ~NEA_VRAM_D;
    else if (mode != NEA_ModeSingle3D)
        bank_flags &= ~NEA_VRAM_CD;

    // Now, configure allocation system. The buffer size always sees the
    // four banks of VRAM. It is needed to allocate and lock one chunk per bank
    // that isn't allocated to Nitro Engine Advanced.

    if (bank_flags & NEA_VRAM_A)
    {
        vramSetBankA(VRAM_A_TEXTURE_SLOT0);
    }
    else
    {
        NEA_AllocAddress(NEA_TexAllocList, VRAM_A, 128 * 1024);
        NEA_Lock(NEA_TexAllocList, VRAM_A);
    }

    if (bank_flags & NEA_VRAM_B)
    {
        vramSetBankB(VRAM_B_TEXTURE_SLOT1);
    }
    else
    {
        NEA_AllocAddress(NEA_TexAllocList, VRAM_B, 128 * 1024);
        NEA_Lock(NEA_TexAllocList, VRAM_B);
    }

    if (bank_flags & NEA_VRAM_C)
    {
        vramSetBankC(VRAM_C_TEXTURE_SLOT2);
    }
    else
    {
        NEA_AllocAddress(NEA_TexAllocList, VRAM_C, 128 * 1024);
        NEA_Lock(NEA_TexAllocList, VRAM_C);
    }

    if (bank_flags & NEA_VRAM_D)
    {
        vramSetBankD(VRAM_D_TEXTURE_SLOT3);
    }
    else
    {
        NEA_AllocAddress(NEA_TexAllocList, VRAM_D, 128 * 1024);
        NEA_Lock(NEA_TexAllocList, VRAM_D);
    }

    GFX_TEX_FORMAT = 0;

    ne_defrag_reset();
    ne_texture_system_inited = true;
    return 0;

cleanup:
    NEA_DebugPrint("Not enough memory");
    NEA_PaletteSystemEnd();
    free(NEA_Texture);
    free(NEA_UserMaterials);
    return -1;
}

void NEA_MaterialDelete(NEA_Material *tex)
{
    NEA_AssertPointer(tex, "NULL pointer");

    // Abort any asynchronous load that would write into this material, before
    // the memory it points to goes away.
    __NEA_AsyncCancelTarget(tex);

    // Delete the palette if it has been flagged to be autodeleted
    if (tex->palette_autodelete)
        NEA_PaletteDelete(tex->palette);

    // If there is an asigned texture
    if (tex->texindex != NEA_NO_TEXTURE)
        ne_texture_delete(tex->texindex);

    // Free the RAM copy of the texture, if this is a RAM-backed material
    free(tex->ram_data);

    for (int i = 0; i < NEA_MAX_TEXTURES; i++)
    {
        if (NEA_UserMaterials[i] == tex)
        {
            NEA_UserMaterials[i] = NULL;
            free(tex);
            return;
        }
    }

    NEA_DebugPrint("Object not found");
}

int NEA_TextureFreeMem(void)
{
    if (!ne_texture_system_inited)
        return 0;

    NEAMemInfo info;
    NEA_MemGetInformation(NEA_TexAllocList, &info);

    return info.free;
}

void NEA_TextureGetStats(NEA_TextureStats *out)
{
    memset(out, 0, sizeof(*out));
    if (!ne_texture_system_inited)
        return;

    const uintptr_t ranges[4][2] = {
        { (uintptr_t)VRAM_A, (uintptr_t)VRAM_B },
        { (uintptr_t)VRAM_B, (uintptr_t)VRAM_B + 64 * 1024 },
        { (uintptr_t)VRAM_B + 64 * 1024, (uintptr_t)VRAM_C },
        { (uintptr_t)VRAM_C, (uintptr_t)VRAM_D },
    };
    for (NEAChunk *c = NEA_TexAllocList; c != NULL; c = c->next)
    {
        if (c->state != NEA_STATE_FREE)
            continue;
        const uintptr_t start = (uintptr_t)c->start, end = (uintptr_t)c->end;
        const int size = (int)(end - start);
        out->free += size;
        out->freeChunks++;
        if (size > out->largest)
            out->largest = size;
        for (int r = 0; r < 4; r++)
        {
            const uintptr_t a = start > ranges[r][0] ? start : ranges[r][0];
            const uintptr_t b = end < ranges[r][1] ? end : ranges[r][1];
            if (b > a && (int)(b - a) > out->largestIn[r])
                out->largestIn[r] = (int)(b - a);
        }
    }

    out->slotsMax = NEA_MAX_TEXTURES;
    for (int i = 0; i < NEA_MAX_TEXTURES; i++)
    {
        if (NEA_Texture[i].address == NULL)
            continue;
        out->slotsUsed++;
        bool named = false;
        for (int m = 0; m < NEA_MAX_TEXTURES && !named; m++)
        {
            if (NEA_UserMaterials[m] != NULL && NEA_UserMaterials[m]->texindex == i)
                named = true;
        }
        if (!named)
            out->leaked++;
    }
    for (int m = 0; m < NEA_MAX_TEXTURES; m++)
    {
        const NEA_Material *mat = NEA_UserMaterials[m];
        if (mat != NULL)
            out->materials++;
        if (mat != NULL && mat->texindex != NEA_NO_TEXTURE &&
            (mat->texindex < 0 || mat->texindex >= NEA_MAX_TEXTURES ||
             NEA_Texture[mat->texindex].address == NULL))
            out->dangling++;
    }
}

int NEA_TextureFree4x4(void)
{
    if (!ne_texture_system_inited)
        return 0;

    // For each free piece of A or C, the parts of it whose index memory (in
    // the matching half of B, at half the address offset) is free as well.
    int total = 0;
    for (NEAChunk *t = NEA_TexAllocList; t != NULL; t = t->next)
    {
        if (t->state != NEA_STATE_FREE)
            continue;
        const uintptr_t start = (uintptr_t)t->start, end = (uintptr_t)t->end;
        for (int bank = 0; bank < 2; bank++)
        {
            const uintptr_t lo = bank == 0 ? (uintptr_t)VRAM_A : (uintptr_t)VRAM_C;
            const uintptr_t hi = bank == 0 ? (uintptr_t)VRAM_B : (uintptr_t)VRAM_D;
            const uintptr_t index = (uintptr_t)VRAM_B + (bank == 0 ? 0 : 64 * 1024);
            const uintptr_t a = start > lo ? start : lo;
            const uintptr_t b = end < hi ? end : hi;
            if (b <= a)
                continue;
            for (NEAChunk *i = NEA_TexAllocList; i != NULL; i = i->next)
            {
                if (i->state != NEA_STATE_FREE)
                    continue;
                // That index piece, as the texel range it serves.
                const uintptr_t is = (uintptr_t)i->start, ie = (uintptr_t)i->end;
                if (ie <= index || is >= index + 64 * 1024)
                    continue;
                const uintptr_t s0 = (is > index ? is : index) - index;
                const uintptr_t s1 = (ie < index + 64 * 1024 ? ie : index + 64 * 1024) - index;
                const uintptr_t c = lo + s0 * 2 > a ? lo + s0 * 2 : a;
                const uintptr_t d = lo + s1 * 2 < b ? lo + s1 * 2 : b;
                if (d > c)
                    total += (int)(d - c);
            }
        }
    }
    return total;
}

int NEA_TextureFreeMemPercent(void)
{
    if (!ne_texture_system_inited)
        return 0;

    NEAMemInfo info;
    NEA_MemGetInformation(NEA_TexAllocList, &info);

    return info.free_percent;
}

//--------------------------------------------------------------------------
// Defragmentation
//
// A 4x4 texture is in two pieces: its texels in bank A (or C), and its index
// data in the first (or second) half of bank B, at half the texels' offset.
// What a 4x4 texture can use is therefore memory free in BOTH places, and the
// pieces that matter are the "pair holes": ranges of A or C that are free and
// whose index range in B is free too.
//
// Two things eat into those. A texture that is not 4x4, put in A or C, takes
// texels whose index memory then serves nothing ("dead" index memory); and
// textures coming and going leave holes too small to use. So:
//
// - A texture that is not 4x4 goes, if it can, into dead index memory, where
//   it costs no 4x4 room at all (ne_alloc_plain).
// - NEA_TextureDefragStep() moves one texture at a time, a slice of it in each
//   vertical blank, to where it makes the largest pair hole larger: a texture
//   that is not 4x4 out of A or C into dead index memory; else a small 4x4
//   texture between two holes to a hole elsewhere, so that the two join.
//
// A texture moves by being copied to a range that does not overlap it (the
// picture on the screen is drawn from the old one until it is replaced: it
// can't be written over) and then, in one go, given its new address. Its old
// range stays allocated for two more steps -- two more frames -- and is then
// freed.

typedef struct {
    uintptr_t start, end;
} ne_range_t;

#define NE_DEFRAG_RANGES 320
#define NE_DEFRAG_GHOSTS 16

static struct {
    bool moving;
    int slot;
    bool is4x4;
    char *src02, *src1;     // where it is (src1: 4x4 only)
    char *dst02, *dst1;     // where it is going
    uint32_t size02, size1; // bytes of each piece
    uint32_t done;          // bytes copied so far (of size02 + size1)
    struct {
        void *addr;
        int age;
    } ghost[NE_DEFRAG_GHOSTS];
    int ghosts;
    unsigned moves;         // textures moved (NEA_TextureMoves())
    unsigned bytes;         // bytes moved
    unsigned steps;         // steps that did some work
} ne_defrag;

static void ne_defrag_reset(void)
{
    memset(&ne_defrag, 0, sizeof(ne_defrag));
}

// The bytes of the allocation that starts at `addr` (0 if none does).
static uint32_t ne_chunk_size(const void *addr)
{
    for (NEAChunk *c = NEA_TexAllocList; c != NULL; c = c->next)
    {
        if (c->start == addr && c->state == NEA_STATE_USED)
            return (uint32_t)((uintptr_t)c->end - (uintptr_t)c->start);
    }
    return 0;
}

// The free ranges inside [lo, hi), in order, with `freed` taken as free and
// `taken` as used (what the memory WOULD be like after a move; either may be
// NULL). Returns how many.
static int ne_free_ranges(uintptr_t lo, uintptr_t hi, const ne_range_t *freed, int nfreed,
                          const ne_range_t *taken, int ntaken, ne_range_t *out, int max)
{
    int n = 0;
    for (NEAChunk *c = NEA_TexAllocList; c != NULL; c = c->next)
    {
        uintptr_t a = (uintptr_t)c->start, b = (uintptr_t)c->end;
        bool free = c->state == NEA_STATE_FREE;
        for (int i = 0; !free && i < nfreed; i++)
            free = freed[i].start <= a && b <= freed[i].end;
        if (!free)
            continue;
        if (a < lo)
            a = lo;
        if (b > hi)
            b = hi;
        if (b <= a)
            continue;
        // (Joined to the one before, if they touch.)
        if (n > 0 && out[n - 1].end == a)
            out[n - 1].end = b;
        else if (n < max)
            out[n++] = (ne_range_t){ a, b };
    }
    // What a move would take, cut out.
    for (int t = 0; t < ntaken; t++)
    {
        for (int i = 0; i < n; i++)
        {
            const uintptr_t a = out[i].start, b = out[i].end;
            const uintptr_t ta = taken[t].start, tb = taken[t].end;
            if (tb <= a || ta >= b)
                continue;
            if (ta > a && tb < b && n < max)
            {
                // Split in two.
                memmove(&out[i + 2], &out[i + 1], (size_t)(n - i - 1) * sizeof(ne_range_t));
                out[i + 1] = (ne_range_t){ tb, b };
                out[i].end = ta;
                n++;
                i++;
            }
            else if (ta <= a && tb >= b)
            {
                out[i].end = out[i].start; // (all of it: left empty)
            }
            else if (ta <= a)
            {
                out[i].start = tb;
            }
            else
            {
                out[i].end = ta;
            }
        }
    }
    return n;
}

// The largest pair hole (and, in `total`, all of them), with a move's
// `freed` and `taken` ranges applied. In texel bytes.
static int ne_pair_holes(const ne_range_t *freed, int nfreed, const ne_range_t *taken, int ntaken, int *total,
                         uintptr_t *largestAt)
{
    static ne_range_t texels[NE_DEFRAG_RANGES], index[NE_DEFRAG_RANGES];
    int largest = 0, sum = 0;
    for (int bank = 0; bank < 2; bank++)
    {
        const uintptr_t lo = bank == 0 ? (uintptr_t)VRAM_A : (uintptr_t)VRAM_C;
        const uintptr_t hi = lo + 128 * 1024;
        const uintptr_t ilo = (uintptr_t)VRAM_B + (bank == 0 ? 0 : 64 * 1024);
        const int nt = ne_free_ranges(lo, hi, freed, nfreed, taken, ntaken, texels, NE_DEFRAG_RANGES);
        const int ni = ne_free_ranges(ilo, ilo + 64 * 1024, freed, nfreed, taken, ntaken, index, NE_DEFRAG_RANGES);
        // Both lists are in order: walk them together, the index ranges
        // seen as the texels they serve.
        int i = 0, j = 0;
        while (i < nt && j < ni)
        {
            const uintptr_t ia = lo + (index[j].start - ilo) * 2;
            const uintptr_t ib = lo + (index[j].end - ilo) * 2;
            const uintptr_t a = texels[i].start > ia ? texels[i].start : ia;
            const uintptr_t b = texels[i].end < ib ? texels[i].end : ib;
            if (b > a)
            {
                const int len = (int)(b - a);
                sum += len;
                if (len > largest)
                {
                    largest = len;
                    if (largestAt != NULL)
                        *largestAt = a;
                }
            }
            if (texels[i].end < ib)
                i++;
            else
                j++;
        }
    }
    if (total != NULL)
        *total = sum;
    return largest;
}

int NEA_TextureLargest4x4(void)
{
    if (!ne_texture_system_inited)
        return 0;
    return ne_pair_holes(NULL, 0, NULL, 0, NULL, NULL);
}

// Dead index memory: a free range of bank B, `size` bytes long, all of whose
// texels in A or C are taken. NULL if there is none. (The free ranges of B,
// cut to where the texels they serve are not free.)
static void *ne_find_dead_index(uint32_t size)
{
    static ne_range_t index[NE_DEFRAG_RANGES], texels[NE_DEFRAG_RANGES];
    size = (size + NEA_ALLOC_MIN_SIZE - 1) & ~(NEA_ALLOC_MIN_SIZE - 1);
    for (int half = 0; half < 2; half++)
    {
        const uintptr_t ilo = (uintptr_t)VRAM_B + (uintptr_t)half * 64 * 1024;
        const uintptr_t lo = half == 0 ? (uintptr_t)VRAM_A : (uintptr_t)VRAM_C;
        const int ni = ne_free_ranges(ilo, ilo + 64 * 1024, NULL, 0, NULL, 0, index, NE_DEFRAG_RANGES);
        const int nt = ne_free_ranges(lo, lo + 128 * 1024, NULL, 0, NULL, 0, texels, NE_DEFRAG_RANGES);
        for (int i = 0; i < ni; i++)
        {
            // This free index range, a piece at a time between the free texel
            // ranges (seen as index addresses): those pieces are dead.
            uintptr_t at = (index[i].start + 7) & ~(uintptr_t)7;
            for (int t = 0; t <= nt && at < index[i].end; t++)
            {
                // The free texels from here on, as index addresses.
                const uintptr_t fa = t < nt ? ilo + (texels[t].start - lo) / 2 : index[i].end;
                const uintptr_t fb = t < nt ? ilo + (texels[t].end - lo + 1) / 2 : index[i].end;
                if (fb <= at)
                    continue;
                const uintptr_t deadEnd = fa < index[i].end ? fa : index[i].end;
                if (deadEnd > at && deadEnd - at >= size)
                    return (void *)at;
                at = (fb + 7) & ~(uintptr_t)7;
            }
        }
    }
    return NULL;
}

static bool ne_place_in_dead_index = true;

void NEA_TextureSetDeadIndexPlacement(bool on)
{
    ne_place_in_dead_index = on;
}

static void *ne_alloc_plain(size_t size)
{
    if (!ne_place_in_dead_index)
        return NEA_AllocFromEnd(NEA_TexAllocList, size);
    // Dead index memory first: there it costs 4x4 textures nothing.
    void *dead = ne_find_dead_index((uint32_t)size);
    if (dead != NULL && NEA_AllocAddress(NEA_TexAllocList, dead, size) == 0)
        return dead;
    return NEA_AllocFromEnd(NEA_TexAllocList, size);
}

static void ne_defrag_forget_slot(int slot)
{
    if (!ne_defrag.moving || ne_defrag.slot != slot)
        return;
    // The texture goes: so does where it was going.
    NEA_Free(NEA_TexAllocList, ne_defrag.dst02);
    if (ne_defrag.is4x4)
        NEA_Free(NEA_TexAllocList, ne_defrag.dst1);
    ne_defrag.moving = false;
}

static bool ne_in_safe_time(int lastLine)
{
    const int line = REG_VCOUNT;
    return line >= 192 && line < lastLine;
}

// Choose a move, and take its destination. False if no move helps.
static bool ne_defrag_choose(void)
{
    int totalNow = 0;
    const int largestNow = ne_pair_holes(NULL, 0, NULL, 0, &totalNow, NULL);
    int bestSlot = -1, bestLargest = largestNow, bestTotal = totalNow;
    void *bestDst02 = NULL, *bestDst1 = NULL;

    for (int i = 0; i < NEA_MAX_TEXTURES; i++)
    {
        char *addr = NEA_Texture[i].address;
        if (addr == NULL || NEA_Texture[i].uses <= 0 || NEA_Texture[i].fixed)
            continue;
        const bool is4x4 = ((NEA_Texture[i].param >> 26) & 7) == NEA_TEX4X4;
        const uintptr_t at = (uintptr_t)addr;
        const bool inTexelBank = (at >= (uintptr_t)VRAM_A && at < (uintptr_t)VRAM_B) ||
                                 (at >= (uintptr_t)VRAM_C && at < (uintptr_t)VRAM_D);
        const uint32_t size02 = ne_chunk_size(addr);
        if (size02 == 0)
            continue;
        ne_range_t freed[2], taken[2];
        int nfreed = 0, ntaken = 0;
        void *dst02 = NULL, *dst1 = NULL;
        if (!is4x4)
        {
            // (a) Out of A or C, into dead index memory.
            if (!inTexelBank)
                continue;
            dst02 = ne_find_dead_index(size02);
            if (dst02 == NULL)
                continue;
            freed[nfreed++] = (ne_range_t){ at, at + size02 };
            taken[ntaken++] = (ne_range_t){ (uintptr_t)dst02, (uintptr_t)dst02 + size02 };
        }
        else
        {
            // (b) A 4x4 texture to another pair hole. (Only one that is
            // small, and with a hole beside it: else it gains nothing.)
            if (size02 > 8 * 1024)
                continue;
            void *slot1 = at < (uintptr_t)VRAM_B ? slot0_to_slot1(addr) : slot2_to_slot1(addr);
            const uint32_t size1 = ne_chunk_size(slot1);
            if (size1 == 0)
                continue;
            if (ne_alloc_compressed_tex(size02, &dst02, &dst1) != 0)
                continue;
            freed[nfreed++] = (ne_range_t){ at, at + size02 };
            freed[nfreed++] = (ne_range_t){ (uintptr_t)slot1, (uintptr_t)slot1 + size1 };
            taken[ntaken++] = (ne_range_t){ (uintptr_t)dst02, (uintptr_t)dst02 + size02 };
            taken[ntaken++] = (ne_range_t){ (uintptr_t)dst1, (uintptr_t)dst1 + size1 };
        }
        int total = 0;
        const int largest = ne_pair_holes(freed, nfreed, taken, ntaken, &total, NULL);
        // Better: a larger largest hole; or (moving a texture that is not
        // 4x4 into dead memory) as large a one, and more room in all.
        const bool better = largest > bestLargest ||
                            (!is4x4 && largest == bestLargest && total > bestTotal + 512);
        if (better)
        {
            bestSlot = i;
            bestLargest = largest;
            bestTotal = total;
            bestDst02 = dst02;
            bestDst1 = dst1;
        }
    }
    if (bestSlot < 0)
        return false;

    // Take the destination.
    char *src02 = NEA_Texture[bestSlot].address;
    const bool is4x4 = ((NEA_Texture[bestSlot].param >> 26) & 7) == NEA_TEX4X4;
    const uint32_t size02 = ne_chunk_size(src02);
    if (NEA_AllocAddress(NEA_TexAllocList, bestDst02, size02) != 0)
        return false;
    char *src1 = NULL;
    uint32_t size1 = 0;
    if (is4x4)
    {
        src1 = (uintptr_t)src02 < (uintptr_t)VRAM_B ? slot0_to_slot1(src02) : slot2_to_slot1(src02);
        size1 = ne_chunk_size(src1);
        if (NEA_AllocAddress(NEA_TexAllocList, bestDst1, size1) != 0)
        {
            NEA_Free(NEA_TexAllocList, bestDst02);
            return false;
        }
    }
    ne_defrag.moving = true;
    ne_defrag.slot = bestSlot;
    ne_defrag.is4x4 = is4x4;
    ne_defrag.src02 = src02;
    ne_defrag.src1 = src1;
    ne_defrag.dst02 = bestDst02;
    ne_defrag.dst1 = bestDst1;
    ne_defrag.size02 = size02;
    ne_defrag.size1 = size1;
    ne_defrag.done = 0;
    return true;
}

// Copy as much of the move as there is time for. True when it is all there.
static bool ne_defrag_copy(int lastLine, bool timed)
{
    const uint32_t total = ne_defrag.size02 + ne_defrag.size1;
    while (ne_defrag.done < total)
    {
        if (timed && !ne_in_safe_time(lastLine))
            return false;
        // The next slice: of the texels, then of the index.
        const bool texels = ne_defrag.done < ne_defrag.size02;
        const uint32_t at = texels ? ne_defrag.done : ne_defrag.done - ne_defrag.size02;
        const uint32_t size = texels ? ne_defrag.size02 : ne_defrag.size1;
        uint32_t slice = size - at;
        if (slice > 512)
            slice = 512;
        const char *from = (texels ? ne_defrag.src02 : ne_defrag.src1) + at;
        char *to = (texels ? ne_defrag.dst02 : ne_defrag.dst1) + at;
        ne_tex_unlock_t state;
        ne_tex_unlock(&state, ne_tex_bank_mask(from, slice) | ne_tex_bank_mask(to, slice));
        // (VRAM takes 16 or 32-bit writes; sizes and addresses here are
        // multiples of 8.)
        const vu32 *s = (const vu32 *)from;
        vu32 *d = (vu32 *)to;
        for (uint32_t n = 0; n < slice / 4; n++)
            d[n] = s[n];
        ne_tex_lock(&state);
        ne_defrag.done += slice;
    }
    return true;
}

bool NEA_TextureDefragChoose(void)
{
    if (!ne_texture_system_inited || ne_defrag.moving || __NEA_VramSessionOpen() ||
        ne_defrag.ghosts + 2 > NE_DEFRAG_GHOSTS)
        return ne_defrag.moving;
    return ne_defrag_choose();
}

int NEA_TextureDefragStep(int lastLine)
{
    if (!ne_texture_system_inited || __NEA_VramSessionOpen())
        return 0;
    const bool timed = lastLine > 0;

    // What was moved two steps ago: nothing on the screen draws it now.
    int kept = 0;
    for (int g = 0; g < ne_defrag.ghosts; g++)
    {
        if (++ne_defrag.ghost[g].age > 2)
            NEA_Free(NEA_TexAllocList, ne_defrag.ghost[g].addr);
        else
            ne_defrag.ghost[kept++] = ne_defrag.ghost[g];
    }
    ne_defrag.ghosts = kept;

    if (!ne_defrag.moving)
        return 0;
    ne_defrag.steps++;
    if (!ne_defrag_copy(lastLine, timed))
        return 1;

    // All of it is there: the texture is where it was copied to, from the
    // next thing drawn on.
    const int slot = ne_defrag.slot;
    NEA_Texture[slot].address = ne_defrag.dst02;
    NEA_Texture[slot].param = (NEA_Texture[slot].param & 0xFFFF0000) |
                              (((uint32_t)(uintptr_t)ne_defrag.dst02 >> 3) & 0xFFFF);
    ne_defrag.ghost[ne_defrag.ghosts].addr = ne_defrag.src02;
    ne_defrag.ghost[ne_defrag.ghosts++].age = 0;
    if (ne_defrag.is4x4)
    {
        ne_defrag.ghost[ne_defrag.ghosts].addr = ne_defrag.src1;
        ne_defrag.ghost[ne_defrag.ghosts++].age = 0;
    }
    ne_defrag.moves++;
    ne_defrag.bytes += ne_defrag.size02 + ne_defrag.size1;
    ne_defrag.moving = false;
    return 1;
}

unsigned NEA_TextureMoves(void)
{
    return ne_defrag.moves;
}

void NEA_TextureDefragStats(int *moves, int *bytes, int *steps)
{
    if (moves != NULL)
        *moves = (int)ne_defrag.moves;
    if (bytes != NULL)
        *bytes = (int)ne_defrag.bytes;
    if (steps != NULL)
        *steps = (int)ne_defrag.steps;
}

void NEA_TextureDefragMem(void)
{
    // As many moves as help, with no regard for the time: only where nothing
    // drawn with textures is on the screen. (Each step ages the ghosts, so a
    // few more at the end free them all.)
    for (int guard = 0; guard < 4096; guard++)
    {
        NEA_TextureDefragChoose();
        if (!NEA_TextureDefragStep(0) && ne_defrag.ghosts == 0)
            break;
    }
}

void NEA_TextureSystemEnd(void)
{
    if (!ne_texture_system_inited)
        return;

    NEA_AllocEnd(&NEA_TexAllocList);

    free(NEA_Texture);

    for (int i = 0; i < NEA_MAX_TEXTURES; i++)
    {
        if (NEA_UserMaterials[i])
        {
            __NEA_AsyncCancelTarget(NEA_UserMaterials[i]);
            free(NEA_UserMaterials[i]);
        }
    }

    free(NEA_UserMaterials);

    NEA_Texture = NULL;

    NEA_PaletteSystemEnd();

    ne_defrag_reset();
    ne_texture_system_inited = false;
}

// Internal use
int __NEA_TextureGetRawX(const NEA_Material *tex)
{
    NEA_AssertPointer(tex, "NULL pointer");
    NEA_Assert(tex->texindex != NEA_NO_TEXTURE,
          "No texture asigned to material");
    return (NEA_Texture[tex->texindex].param & (0x7 << 20)) >> 20;
}

// Internal use
int __NEA_TextureGetRawY(const NEA_Material *tex)
{
    NEA_AssertPointer(tex, "NULL pointer");
    NEA_Assert(tex->texindex != NEA_NO_TEXTURE, "No texture asigned to material");
    return (NEA_Texture[tex->texindex].param & (0x7 << 23)) >> 23;
}

int NEA_TextureGetRealSizeX(const NEA_Material *tex)
{
    NEA_AssertPointer(tex, "NULL pointer");
    NEA_Assert(tex->texindex != NEA_NO_TEXTURE, "No texture asigned to material");
    return 8 << __NEA_TextureGetRawX(tex);
}

int NEA_TextureGetRealSizeY(const NEA_Material *tex)
{
    NEA_AssertPointer(tex, "NULL pointer");
    NEA_Assert(tex->texindex != NEA_NO_TEXTURE, "No texture asigned to material");
    return 8 << __NEA_TextureGetRawY(tex);
}

int NEA_TextureGetSizeX(const NEA_Material *tex)
{
    NEA_AssertPointer(tex, "NULL pointer");
    NEA_Assert(tex->texindex != NEA_NO_TEXTURE, "No texture asigned to material");
    return NEA_Texture[tex->texindex].sizex;
}

int NEA_TextureGetSizeY(const NEA_Material *tex)
{
    NEA_AssertPointer(tex, "NULL pointer");
    NEA_Assert(tex->texindex != NEA_NO_TEXTURE, "No texture asigned to material");
    return NEA_Texture[tex->texindex].sizey;
}

void NEA_MaterialSetProperties(NEA_Material *tex, u32 diffuse,
                              u32 ambient, u32 specular, u32 emission,
                              bool vtxcolor, bool useshininess)
{
    NEA_AssertPointer(tex, "NULL pointer");
    tex->diffuse_ambient = diffuse | (ambient << 16) | (vtxcolor << 15);
    tex->specular_emission = specular | (emission << 16) | (useshininess << 15);
}

void NEA_MaterialSetDefaultProperties(u32 diffuse, u32 ambient,
                                     u32 specular, u32 emission,
                                     bool vtxcolor, bool useshininess)
{
    ne_default_diffuse_ambient = diffuse | (ambient << 16) | (vtxcolor << 15);
    ne_default_specular_emission = specular | (emission << 16)
                                 | (useshininess << 15);

    GFX_DIFFUSE_AMBIENT = ne_default_diffuse_ambient;
    GFX_SPECULAR_EMISSION = ne_default_specular_emission;
}

static u16 *drawingtexture_address = NULL;
static int drawingtexture_x, drawingtexture_y;
static int drawingtexture_type;
static int drawingtexture_realx;
static u32 ne_vram_saved;

// Depth of open VRAM editing sessions. See __NEA_VramSessionOpen().
static int ne_vram_session_depth = 0;

bool __NEA_VramSessionOpen(void)
{
    return ne_vram_session_depth > 0;
}

void __NEA_VramSessionEnter(void)
{
    ne_vram_session_depth++;
}

void __NEA_VramSessionExit(void)
{
    NEA_Assert(ne_vram_session_depth > 0, "Unbalanced VRAM session");

    if (ne_vram_session_depth > 0)
        ne_vram_session_depth--;
}

void *NEA_TextureDrawingStart(const NEA_Material *tex)
{
    NEA_AssertPointer(tex, "NULL pointer");
    NEA_Assert(tex->texindex != NEA_NO_TEXTURE, "No texture asigned to material");

    NEA_Assert(drawingtexture_address == NULL,
              "Another texture is already active");

    drawingtexture_x = NEA_TextureGetSizeX(tex);
    drawingtexture_realx = NEA_TextureGetRealSizeX(tex);
    drawingtexture_y = NEA_TextureGetSizeY(tex);
    drawingtexture_address = (u16 *) ((uintptr_t)VRAM_A
                          + ((NEA_Texture[tex->texindex].param & 0xFFFF) << 3));
    drawingtexture_type = ((NEA_Texture[tex->texindex].param >> 26) & 0x7);

    ne_vram_saved = vramSetPrimaryBanks(VRAM_A_LCD, VRAM_B_LCD, VRAM_C_LCD,
                                        VRAM_D_LCD);

    // drawingtexture_address is a raw pointer into texture VRAM. An async
    // finalize running now would call NEA_MaterialTexLoad(), which can free and
    // reissue that very range, so hold off the finalize steps until the session
    // closes.
    __NEA_VramSessionEnter();

    return drawingtexture_address;
}

void NEA_TexturePutPixelRGBA(u32 x, u32 y, u16 color)
{
    NEA_AssertPointer(drawingtexture_address,
                     "No texture active for drawing");
    NEA_Assert(drawingtexture_type == NEA_A1RGB5,
              "Active texture isn't NEA_A1RGB5");

    if (x >= drawingtexture_x || y >= drawingtexture_y)
        return;

    drawingtexture_address[x + (y * drawingtexture_realx)] = color;
}

void NEA_TexturePutPixelRGB256(u32 x, u32 y, u8 palettecolor)
{
    NEA_AssertPointer(drawingtexture_address,
                     "No texture active for drawing.");
    NEA_Assert(drawingtexture_type == NEA_PAL256,
              "Active texture isn't NEA_PAL256");

    if (x >= drawingtexture_x || y >= drawingtexture_y)
        return;

    int position = (x + (y * drawingtexture_realx)) >> 1;
    int desp = (x & 1) << 3;

    drawingtexture_address[position] &= 0xFF00 >> desp;
    drawingtexture_address[position] |= ((u16) palettecolor) << desp;
}

void NEA_TextureDrawingEnd(void)
{
    NEA_Assert(drawingtexture_address != NULL, "No active texture");

    vramRestorePrimaryBanks(ne_vram_saved);

    __NEA_VramSessionExit();

    drawingtexture_address = NULL;
}

void NEA_TextureMatrixIdentity(void)
{
    MATRIX_CONTROL = GL_TEXTURE;
    MATRIX_IDENTITY = 0;
    MATRIX_CONTROL = GL_MODELVIEW;
}

void NEA_TextureMatrixTranslateI(int x, int y)
{
    MATRIX_CONTROL = GL_TEXTURE;
    MATRIX_TRANSLATE = x;
    MATRIX_TRANSLATE = y;
    MATRIX_TRANSLATE = 0;
    MATRIX_CONTROL = GL_MODELVIEW;
}

void NEA_TextureMatrixRotate(int angle)
{
    MATRIX_CONTROL = GL_TEXTURE;
    glRotateZi(angle << 6);
    MATRIX_CONTROL = GL_MODELVIEW;
}

void NEA_TextureMatrixScaleI(int sx, int sy)
{
    MATRIX_CONTROL = GL_TEXTURE;
    MATRIX_SCALE = sx;
    MATRIX_SCALE = sy;
    MATRIX_SCALE = inttof32(1);
    MATRIX_CONTROL = GL_MODELVIEW;
}

// Current directional vector matrix, 9 words in 3x3 order. This is the rotation
// part of the modelview transform, the same one the GPU uses to bring normals
// into view space for lighting. libnds doesn't export it.
#ifndef VECMTX_RESULT
#define VECMTX_RESULT ((vs32 *)0x04000680)
#endif

void NEA_TextureMatrixEnvMapI(int scale_s, int scale_t)
{
    // Normals arrive as 1.0.9 and the matrix as 1.19.12, so their product has 21
    // fractional bits, and the hardware takes it as the raw 1.11.4 texture
    // coordinate. A matrix entry of N therefore maps a unit normal to N *raw*
    // units, and a texel is 16 of those. The callers of this function think in
    // texels, so convert here.
    scale_s *= 16;
    scale_t *= 16;

    // Result registers may only be read with the geometry engine stopped.
    while (GFX_STATUS & BIT(27));

    int32_t vec[9];

    for (int i = 0; i < 9; i++)
        vec[i] = VECMTX_RESULT[i];

    MATRIX_CONTROL = GL_TEXTURE;
    MATRIX_IDENTITY = 0;

    // The hardware computes texcoords as the row vector (Nx, Ny, Nz, 1) times
    // this matrix, and every matrix command sets C = M * C, so the factors go in
    // right to left: scale first, then the rotation, giving C = Vec * Scale. The
    // normal is rotated into view space and only then scaled into texels.
    MATRIX_SCALE = scale_s;
    MATRIX_SCALE = scale_t;
    MATRIX_SCALE = inttof32(1);

    for (int i = 0; i < 9; i++)
        MATRIX_MULT3x3 = vec[i];

    MATRIX_CONTROL = GL_MODELVIEW;
}

void NEA_TextureMatrixEnvMap(const NEA_Material *tex)
{
    NEA_AssertPointer(tex, "NULL pointer");

    // Half the texture size each way, so a unit normal reaches the edge of the
    // sphere map from the centre the mesh's texture coordinate puts it at.
    //
    // T is negated because the two axes disagree about which way is up: texture
    // T grows downwards, while a normal's +Y points up. Without this a sphere
    // map comes out vertically mirrored, and anything lit from above in the
    // image ends up lighting the model from below.
    int half_s = inttof32(NEA_TextureGetSizeX(tex) / 2);
    int half_t = inttof32(NEA_TextureGetSizeY(tex) / 2);

    NEA_TextureMatrixEnvMapI(half_s, -half_t);
}

//--------------------------------------------------------------------------
// Fixed regions: memory the caller places textures in itself.

int NEA_TextureRegionReserve4x4(int maxTexelBytes, NEA_TextureRegion *out)
{
    NEA_AssertPointer(out, "NULL pointer");
    memset(out, 0, sizeof(*out));
    if (!ne_texture_system_inited || maxTexelBytes <= 0)
        return 0;

    uintptr_t at = 0;
    const int largest = ne_pair_holes(NULL, 0, NULL, 0, NULL, &at);
    // (Texels on a 32-byte boundary, so that the index is on a 16-byte one:
    // the allocator's.)
    const uintptr_t start = (at + 31) & ~(uintptr_t)31;
    int bytes = largest - (int)(start - at);
    if (bytes > maxTexelBytes)
        bytes = maxTexelBytes;
    bytes &= ~31;
    if (bytes <= 0)
        return 0;

    void *texels = (void *)start;
    void *index = NEA_Texture4x4IndexAddress(texels);
    if (NEA_AllocAddress(NEA_TexAllocList, texels, bytes) != 0)
        return 0;
    if (NEA_AllocAddress(NEA_TexAllocList, index, bytes / 2) != 0)
    {
        NEA_Free(NEA_TexAllocList, texels);
        return 0;
    }
    out->texels = texels;
    out->index = index;
    out->texelBytes = bytes;
    return bytes;
}

void NEA_TextureRegionRelease(NEA_TextureRegion *region)
{
    NEA_AssertPointer(region, "NULL pointer");
    if (ne_texture_system_inited && region->texelBytes > 0)
    {
        NEA_Free(NEA_TexAllocList, region->texels);
        NEA_Free(NEA_TexAllocList, region->index);
    }
    memset(region, 0, sizeof(*region));
}

void *NEA_Texture4x4IndexAddress(const void *texels)
{
    void *t = (void *)texels;
    return (t < (void *)VRAM_B) ? slot0_to_slot1(t) : slot2_to_slot1(t);
}

int NEA_MaterialTexBindFixed(NEA_Material *tex, NEA_TextureFormat fmt,
                             int sizeX, int sizeY, NEA_TextureFlags flags,
                             void *texels)
{
    NEA_AssertPointer(tex, "NULL material pointer");
    if (!ne_texture_system_inited || texels == NULL)
        return 0;
    if (ne_is_valid_tex_size(sizeX) != sizeX || ne_is_valid_tex_size(sizeY) == 0)
        return 0;

    if (tex->texindex != NEA_NO_TEXTURE)
        ne_texture_delete(tex->texindex);
    tex->texindex = NEA_NO_TEXTURE;

    for (int i = 0; i < NEA_MAX_TEXTURES; i++)
    {
        if (NEA_Texture[i].address == NULL)
        {
            tex->texindex = i;
            break;
        }
    }
    if (tex->texindex == NEA_NO_TEXTURE)
    {
        NEA_DebugPrint("No free slots");
        return 0;
    }

    int slot = tex->texindex;
    NEA_Texture[slot].sizex = sizeX;
    NEA_Texture[slot].sizey = sizeY;
    NEA_Texture[slot].address = texels;
    NEA_Texture[slot].uses = 1;
    NEA_Texture[slot].fixed = true;
    ne_set_material_tex_param(tex, sizeX, ne_is_valid_tex_size(sizeY), texels,
                              fmt, flags);
    return 1;
}

// The banks a write may go to, in LCD mode: A to D, then E, F and G.
static const struct {
    uintptr_t start, end;
    vu8 *control;
} ne_vram_banks[7] = {
    { 0x06800000, 0x06820000, &VRAM_A_CR },
    { 0x06820000, 0x06840000, &VRAM_B_CR },
    { 0x06840000, 0x06860000, &VRAM_C_CR },
    { 0x06860000, 0x06880000, &VRAM_D_CR },
    { 0x06880000, 0x06890000, &VRAM_E_CR },
    { 0x06890000, 0x06894000, &VRAM_F_CR },
    { 0x06894000, 0x06898000, &VRAM_G_CR },
};

// Words, eight at a time where it can (ldmia/stmia).
ARM_CODE static void ne_copy_words(u32 *dst, const u32 *src, u32 words)
{
    while (words >= 8)
    {
        __asm__ volatile("ldmia %0!, {r3-r10}\n\t"
                         "stmia %1!, {r3-r10}"
                         : "+r"(src), "+r"(dst)
                         :
                         : "r3", "r4", "r5", "r6", "r7", "r8", "r9", "r10", "memory");
        words -= 8;
    }
    while (words--)
        *dst++ = *src++;
}

ARM_CODE void NEA_VramWrite(void *dst, const void *src, size_t bytes)
{
    const uintptr_t first = (uintptr_t)dst, last = first + bytes;
    u8 saved[7];
    u8 mask = 0;

    if (bytes == 0)
        return;
    for (int i = 0; i < 7; i++)
    {
        if (last <= ne_vram_banks[i].start || first >= ne_vram_banks[i].end)
            continue;
        mask |= 1 << i;
        saved[i] = *ne_vram_banks[i].control;
        *ne_vram_banks[i].control = VRAM_ENABLE; // (LCD mode is 0 in every bank)
    }

    ne_copy_words((u32 *)dst, (const u32 *)src, (u32)(bytes >> 2));

    for (int i = 0; i < 7; i++)
    {
        if (mask & (1 << i))
            *ne_vram_banks[i].control = saved[i];
    }
}
