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
} ne_textureinfo_t;

static ne_textureinfo_t *NEA_Texture = NULL;
static NEA_Material **NEA_UserMaterials = NULL;

static NEAChunk *NEA_TexAllocList; // See NEAAlloc.h

static bool ne_texture_system_inited = false;

static int NEA_MAX_TEXTURES;

// Default material properties
static u32 ne_default_diffuse_ambient;
static u32 ne_default_specular_emission;

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

static void ne_texture_delete(int texture_index)
{
    int slot = texture_index;

    // A texture may be used by several materials
    NEA_Texture[slot].uses--;

    // If the number of users is zero, delete it.
    if (NEA_Texture[slot].uses == 0)
    {
        uint32_t fmt = (NEA_Texture[slot].param >> 26) & 7;

        if (fmt == NEA_TEX4X4)
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

    int ret = 0;

    void *gfxDst = NULL;
    void *palDst = NULL;
    GRFHeader header = { 0 };
    GRFError err = grfLoadPath(path, &header, &gfxDst, NULL, NULL, NULL,
                               &palDst, NULL);
    if (err != GRF_NO_ERROR)
    {
        NEA_DebugPrint("Couldn't load GRF file: %d", err);
        goto cleanup;
    }

    if (header.flags & GRF_FLAG_COLOR0_TRANSPARENT)
        flags |= GL_TEXTURE_COLOR0_TRANSPARENT;

    if (gfxDst == NULL)
    {
        NEA_DebugPrint("No graphics found in GRF file");
        goto cleanup;
    }

    NEA_TextureFormat fmt;
    switch (header.gfxAttr)
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
            goto cleanup;
    }

    if (NEA_MaterialTexLoad(tex, fmt, header.gfxWidth, header.gfxHeight,
                           flags, gfxDst) == 0)
    {
        NEA_DebugPrint("Failed to load GRF texture");
        goto cleanup;
    }

    // If there is no palette to be loaded there is nothing else to do
    if (palDst == NULL)
    {
        ret = 1; // Success
        goto cleanup;
    }

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
            goto cleanup;
        }
    }

    if (NEA_PaletteLoadSize(pal, palDst, header.palAttr * 2, fmt) == 0)
    {
        NEA_DebugPrint("Failed to load GRF palette");
        if (create_palette)
            NEA_PaletteDelete(pal);
        goto cleanup;
    }

    NEA_MaterialSetPalette(tex, pal);

    if (create_palette)
        NEA_MaterialAutodeletePalette(tex);

    ret = 1; // Success

cleanup:
    free(gfxDst);
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

        // Check if this address is free and has enough space
        addr2 = NEA_AllocFindInRange(NEA_TexAllocList, addr2, VRAM_B, size02);
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

int NEA_MaterialTex4x4Load(NEA_Material *tex, int sizeX, int sizeY,
                          NEA_TextureFlags flags, const void *texture02,
                          const void *texture1)
{
    NEA_AssertPointer(tex, "NULL material pointer");

    // For tex4x4 textures, both width and height must be valid
    if ((ne_is_valid_tex_size(sizeX) != sizeX)
        || (ne_is_valid_tex_size(sizeY) != sizeY))
    {
        NEA_DebugPrint("Width and height of tex4x4 textures must be a power of 2");
        return 0;
    }

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
        NEA_DebugPrint("Can't find space for compressed texture");
        return 0;
    }

    ret = NEA_AllocAddress(NEA_TexAllocList, slot02, size02);
    if (ret != 0)
    {
        NEA_DebugPrint("Can't allocate slot 0/2");
        return 0;
    }

    ret = NEA_AllocAddress(NEA_TexAllocList, slot1, size1);
    if (ret != 0)
    {
        NEA_Free(NEA_TexAllocList, slot02);
        NEA_DebugPrint("Can't allocate slot 1");
        return 0;
    }

    // Save information
    int slot = tex->texindex;
    NEA_Texture[slot].sizex = sizeX;
    NEA_Texture[slot].sizey = sizeY;
    NEA_Texture[slot].address = slot02;
    NEA_Texture[slot].uses = 1; // Initially only this material uses the texture

    // Unlock texture memory for writing
    // TODO: Only unlock the banks that Nitro Engine Advanced uses.
    u32 vramTemp = vramSetPrimaryBanks(VRAM_A_LCD, VRAM_B_LCD, VRAM_C_LCD,
                                        VRAM_D_LCD);

    swiCopy(texture02, slot02, (size02 >> 2) | COPY_MODE_WORD);
    swiCopy(texture1, slot1, (size1 >> 2) | COPY_MODE_WORD);

    int hardware_size_y = ne_is_valid_tex_size(sizeY);
    ne_set_material_tex_param(tex, sizeX, hardware_size_y, slot02,
                              NEA_TEX4X4, flags);

    vramRestorePrimaryBanks(vramTemp);

    return 1;
}

int NEA_MaterialTexLoad(NEA_Material *tex, NEA_TextureFormat fmt,
                       int sizeX, int sizeY, NEA_TextureFlags flags,
                       const void *texture)
{
    NEA_AssertPointer(tex, "NULL material pointer");
    NEA_Assert(fmt != 0, "No texture format provided");

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
    void *addr = NEA_AllocFromEnd(NEA_TexAllocList, size);
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

    // Unlock texture memory for writing
    // TODO: Only unlock the banks that Nitro Engine Advanced uses.
    u32 vramTemp = vramSetPrimaryBanks(VRAM_A_LCD, VRAM_B_LCD, VRAM_C_LCD,
                                       VRAM_D_LCD);

    if (fmt == NEA_RGB5)
    {
        // NEA_RGB5 is NEA_A1RGB5 with each alpha bit manually set to 1 during the
        // copy to VRAM.
        uint32_t *src = (uint32_t *)texture;
        uint32_t *dest = addr;
        size >>= 2; // We are going to process four bytes each iteration
        while (size--)
            *dest++ = *src++ | ((1 << 15) | (1 << 31));

        fmt = NEA_A1RGB5;
    }
    else
    {
        swiCopy((u32 *)texture, addr, (size >> 2) | COPY_MODE_WORD);
    }

    int hardware_size_y = ne_is_valid_tex_size(sizeY);
    ne_set_material_tex_param(tex, sizeX, hardware_size_y, addr, fmt, flags);

    vramRestorePrimaryBanks(vramTemp);

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
    NEA_Assert(source->texindex != NEA_NO_TEXTURE,
              "No texture asigned to source material");
    // Increase count of materials using this texture
    NEA_Texture[source->texindex].uses++;
    memcpy(dest, source, sizeof(NEA_Material));
}

void NEA_MaterialSetPalette(NEA_Material *tex, NEA_Palette *pal)
{
    NEA_AssertPointer(tex, "NULL material pointer");
    NEA_AssertPointer(pal, "NULL palette pointer");
    NEA_Assert(tex->texindex != NEA_NO_TEXTURE, "No texture asigned to material");
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

    NEA_Assert(tex->texindex != NEA_NO_TEXTURE, "No texture asigned to material");

    if (tex->palette)
        NEA_PaletteUse(tex->palette);

    GFX_COLOR = tex->color;
    GFX_TEX_FORMAT = NEA_Texture[tex->texindex].param;
}

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

    // Delete the palette if it has been flagged to be autodeleted
    if (tex->palette_autodelete)
        NEA_PaletteDelete(tex->palette);

    // If there is an asigned texture
    if (tex->texindex != NEA_NO_TEXTURE)
        ne_texture_delete(tex->texindex);

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

int NEA_TextureFreeMemPercent(void)
{
    if (!ne_texture_system_inited)
        return 0;

    NEAMemInfo info;
    NEA_MemGetInformation(NEA_TexAllocList, &info);

    return info.free_percent;
}

void NEA_TextureDefragMem(void)
{
    NEA_Assert(0, "This function doesn't work");
    return;
    /*
    // REALLY OLD CODE -- DOESN'T WORK

    if (!ne_texture_system_inited)
        return;

    uint32_t vramTemp = vramSetMainBanks(VRAM_A_LCD, VRAM_B_LCD, VRAM_C_LCD,
                                       VRAM_D_LCD);

    bool ok = false;
    while (!ok)
    {
        ok = true;
        int i;
        for (i = 0; i < NEA_MAX_TEXTURES; i++)
        {
            int size = NEA_GetSize(NEA_TexAllocList, (void*)NEA_Texture[i].address);
            NEA_Free(NEA_TexAllocList,(void*)NEA_Texture[i].address);
            void *pointer = NEA_Alloc(NEA_TexAllocList, size);
            // Aligned to 8 bytes

            NEA_AssertPointer(pointer, "Couldn't reallocate texture");

            if (pointer != NEA_Texture[i].address)
            {
                dmaCopy((void *)NEA_Texture[i].address, pointer, size);
                NEA_Texture[i].address = pointer;
                NEA_Texture[i].param &= 0xFFFF0000;
                NEA_Texture[i].param |= ((uint32_t)pointer >> 3) & 0xFFFF;
                ok = false;
            }
        }
    }
    vramRestoreMainBanks(vramTemp);
    */
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
            free(NEA_UserMaterials[i]);
    }

    free(NEA_UserMaterials);

    NEA_Texture = NULL;

    NEA_PaletteSystemEnd();

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

    return drawingtexture_address;
}

void NEA_TexturePutPixelRGBA(u32 x, u32 y, u16 color)
{
    NEA_AssertPointer(drawingtexture_address,
                     "No texture active for drawing");
    NEA_Assert(drawingtexture_type == NEA_A1RGB5,
              "Ative texture isn't NEA_A1RGB5");

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

    drawingtexture_address = NULL;
}
