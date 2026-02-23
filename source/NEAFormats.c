// SPDX-License-Identifier: MIT
//
// Copyright (c) 2008-2022 Antonio Niño Díaz
//
// This file is part of Nitro Engine Advanced

#include "NEAMain.h"

/// @file NEAFormats.c

static int lastx = 0, lasty = 0;
static u32 numcolors = 0;

static void *NEA_ConvertBMPtoRGBA(void *pointer, bool transpcolor)
{
    NEA_BMPHeader *header = pointer;
    NEA_BMPInfoHeader *infoheader = (void *)(u8 *)header + sizeof(NEA_BMPHeader);
    u8 *IMAGEDATA = (void *)((u8 *)infoheader + sizeof(NEA_BMPInfoHeader));
    if (header->type != 0x4D42)
    {
        NEA_DebugPrint("Not a BMP file");
        return NULL;
    }

    int sizex = infoheader->width;
    int sizey = infoheader->height;
    if (sizex > 1024 || sizey > 1024)
    {
        NEA_DebugPrint("BMP file too big (%d, %d)", sizex, sizey);
        return NULL;
    }

    if (infoheader->compression != 0)
    {
        NEA_DebugPrint("Compressed BMP not supported");
        return NULL;
    }

    if (infoheader->bits != 16 && infoheader->bits != 24)
    {
        NEA_DebugPrint("Unsuported depth for NEA_A1RGB5 conversion (%d)",
                      infoheader->bits);
        return NULL;
    }

    // Decode
    u16 *buffer = malloc(2 * sizex * sizey);
    NEA_AssertPointer(buffer, "Couldn't allocate temporary buffer");

    u8 transr = 0, transb = 0, transg = 0;
    u16 transcolor16bit = 0;

    if (transpcolor)
    {
        if (infoheader->bits == 16) // X1RGB5
        {
            u16 red, green, blue;
            transcolor16bit = (u16)IMAGEDATA[2 * sizey * (sizex - 1)]
                            | (((u16)IMAGEDATA[2 * sizey * (sizex - 1) + 1]) << 8);

            // Swap R and B channels
            red = (transcolor16bit & 0x7C00) >> 10;
            green = (transcolor16bit & 0x3E0);
            blue = (transcolor16bit & 0x1F);
            transcolor16bit = red | green | (blue << 10);
        }
        else // 24 bits
        {
            transr = IMAGEDATA[3 * sizey * (sizex - 1) + 2];
            transg = IMAGEDATA[3 * sizey * (sizex - 1) + 1];
            transb = IMAGEDATA[3 * sizey * (sizex - 1) + 0];
        }
    }
    // IMAGEDATA -> buffer

    int disalign = ((sizex * infoheader->bits) >> 3) & 3;
    if (disalign)
        disalign = 4 - disalign;

    if (infoheader->bits == 16) // X1RGB5
    {
        for (int y = 0; y < sizey; y++)
        {
            for (int x = 0; x < sizex; x++)
            {
                u16 red, green, blue;

                int base_pos = (sizex * (sizey - y - 1) + x) << 1;

                if (disalign)
                    base_pos += disalign * (sizey - y - 1);

                u16 color = (u16) IMAGEDATA[base_pos]
                          | ((u16)IMAGEDATA[base_pos + 1] << 8);

                // Swap R and B channels
                red = (color & 0x7C00) >> 10;
                green = (color & 0x3E0);
                blue = (color & 0x1F);
                color = red | green | (blue << 10);

                if (!(transpcolor && color == transcolor16bit))
                    buffer[y * sizex + x] = color | BIT(15);
                else
                    buffer[y * sizex + x] = 0;
            }
        }
    }
    else // 24 bits
    {
        for (int y = 0; y < sizey; y++)
        {
            for (int x = 0; x < sizex; x++)
            {
                u8 r, g, b;

                int base_pos = 3 * (sizex * (sizey - y - 1) + x);

                if (disalign)
                    base_pos += disalign * (sizey - y - 1);

                r = IMAGEDATA[base_pos + 2];
                g = IMAGEDATA[base_pos + 1];
                b = IMAGEDATA[base_pos + 0];

                if (!(transpcolor && r == transr && g == transg && b == transb))
                {
                    buffer[y * sizex + x] = RGB15((r >> 3) & 31, (g >> 3) & 31,
                                                  (b >> 3) & 31) | BIT(15);
                }
                else
                {
                    buffer[y * sizex + x] = 0;
                }
            }
        }
    }

    lasty = sizey;
    lastx = sizex;

    return (void *)buffer;
}

static void *NEA_ConvertBMPtoRGB256(void *pointer, u16 *palettebuffer)
{
    NEA_BMPHeader *header = pointer;
    NEA_BMPInfoHeader *infoheader = (void *)((u8 *)header + sizeof(NEA_BMPHeader));
    if (header->type != 0x4D42)
    {
        NEA_DebugPrint("Not a BMP file");
        return NULL;
    }

    int sizex = infoheader->width;
    int sizey = infoheader->height;
    if (sizex > 1024 || sizey > 1024)
    {
        NEA_DebugPrint("BMP file too big (%d, %d)", sizex, sizey);
        return NULL;
    }

    if (infoheader->compression != 0)
    {
        NEA_DebugPrint("Compressed BMP not supported");
        return NULL;
    }

    if (infoheader->bits != 8 && infoheader->bits != 4)
    {
        NEA_DebugPrint("Unsupported depth for NEA_PAL256 conversion (%d)",
                      infoheader->bits);
        return NULL;
    }

    // Decode
    int colornumber = (infoheader->bits == 8) ? 256 : 16;
    u8 *PALETTEDATA = (u8 *)infoheader + sizeof(NEA_BMPInfoHeader);
    u8 *IMAGEDATA = (u8 *)header + header->offset;

    // numcolors is used by the other functions (look at the start of this
    // file)
    numcolors = colornumber;

    // First, we read the palette
    int i = 0;
    while (i < numcolors)
    {
        u8 r, g, b;
        g = PALETTEDATA[(i << 2) + 1] & 0xFF;
        r = PALETTEDATA[(i << 2) + 2] & 0xFF;
        b = PALETTEDATA[(i << 2) + 0] & 0xFF;
        palettebuffer[i] = RGB15(r >> 3, g >> 3, b >> 3);
        i++;
    }

    u8 *buffer = malloc(sizex * sizey);
    NEA_AssertPointer(buffer, "Couldn't allocate temporary buffer");

    // Then, the image
    if (colornumber == 256)
    {
        // For BMPs with width not multiple of 4
        int disalign = sizex & 3;

        if (disalign)
        {
            disalign = 4 - disalign;

            for (int y = 0; y < sizey; y++)
            {
                for (int x = 0; x < sizex; x++)
                {
                    buffer[y * sizex + x] = IMAGEDATA[(sizex * (sizey - y - 1)) + x
                                          + (((disalign) * (sizey - y - 1)) * 1)];
                }
            }
        }
        else
        {
            for (int y = 0; y < sizey; y++)
            {
                for (int x = 0; x < sizex; x++)
                {
                    buffer[y * sizex + x] =
                        IMAGEDATA[(sizex * (sizey - y - 1)) + x];
                }
            }
        }
    }
    else // colornumber == 16
    {
        // For BMPs with width not multiple of 8
        int disalign = sizex & 7;

        if (disalign)
        {
            disalign = 8 - disalign;

            for (int y = 0; y < sizey; y++)
            {
                for (int x = 0; x < sizex; x++)
                {
                    u32 value;
                    u32 srcidx = ((sizex * (sizey - y - 1) + x)
                               + (disalign * (sizey - y - 1))) >> 1;

                    if (x & 1)
                        value = IMAGEDATA[srcidx] & 0x0F;
                    else
                        value = (IMAGEDATA[srcidx] >> 4) & 0x0F;

                    buffer[y * sizex + x] = value;
                }
            }
        }
        else
        {
            for (int y = 0; y < sizey; y++)
            {
                for (int x = 0; x < sizex; x++)
                {
                    u32 value;
                    u32 srcidx = (sizex * (sizey - y - 1) + x) >> 1;

                    if (x & 1)
                        value = IMAGEDATA[srcidx] & 0x0F;
                    else
                        value = (IMAGEDATA[srcidx] >> 4) & 0x0F;

                    buffer[y * sizex + x] = value;
                }
            }
        }
    }

    lastx = sizex;
    lasty = sizey;

    return (void *)buffer;
}

int NEA_FATMaterialTexLoadBMPtoRGBA(NEA_Material *tex, char *filename,
                                   bool transpcolor)
{
    NEA_AssertPointer(tex, "NULL material pointer");
    NEA_AssertPointer(filename, "NULL filename pointer");

    void *pointer = NEA_FATLoadData(filename);
    int ret = NEA_MaterialTexLoadBMPtoRGBA(tex, pointer, transpcolor);
    free(pointer);

    return ret;
}

int NEA_FATMaterialTexLoadBMPtoRGB256(NEA_Material *tex, NEA_Palette *pal,
                                     char *filename, bool transpcolor)
{
    NEA_AssertPointer(tex, "NULL material pointer");
    NEA_AssertPointer(pal, "NULL palette pointer");
    NEA_AssertPointer(filename, "NULL filename pointer");

    char *pointer = NEA_FATLoadData(filename);
    int ret = NEA_MaterialTexLoadBMPtoRGB256(tex, pal, pointer, transpcolor);
    free(pointer);

    return ret;
}

int NEA_MaterialTexLoadBMPtoRGBA(NEA_Material *tex, void *pointer,
                                bool transpcolor)
{
    NEA_AssertPointer(tex, "NULL material pointer");
    NEA_AssertPointer(pointer, "NULL data pointer");

    void *temp = NEA_ConvertBMPtoRGBA(pointer, transpcolor);
    if (temp == NULL)
        return 0;

    int ret = NEA_MaterialTexLoad(tex, NEA_A1RGB5, lastx, lasty,
                                 NEA_TEXGEN_TEXCOORD, (u8 *)temp);
    free(temp);

    if (ret == 0)
        return 0;

    return 1;
}

int NEA_MaterialTexLoadBMPtoRGB256(NEA_Material *tex, NEA_Palette *pal,
                                  void *pointer, bool transpcolor)
{
    NEA_AssertPointer(tex, "NULL material pointer");
    NEA_AssertPointer(pal, "NULL palette pointer");
    NEA_AssertPointer(pointer, "NULL data pointer");

    u16 *palettebuffer = malloc(256 * sizeof(u16));
    NEA_AssertPointer(palettebuffer, "Couldn't allocate temp palette buffer");
    if (palettebuffer == NULL)
        return 0;

    void *texturepointer = NEA_ConvertBMPtoRGB256(pointer, palettebuffer);
    NEA_AssertPointer(texturepointer, "Couldn't convert BMP file to NEA_PAL256");
    if (texturepointer == NULL)
    {
        free(palettebuffer);
        return 0;
    }

    u32 transp = transpcolor ? NEA_TEXTURE_COLOR0_TRANSPARENT : 0;

    int ret = NEA_MaterialTexLoad(tex, NEA_PAL256, lastx, lasty,
                                 NEA_TEXGEN_TEXCOORD | transp,
                                 (u8 *)texturepointer);
    free(texturepointer);

    if (ret == 0)
    {
        NEA_DebugPrint("Error while loading texture");
        free(palettebuffer);
        return 0;
    }
    ret = NEA_PaletteLoad(pal, palettebuffer, numcolors, NEA_PAL256);
    free(palettebuffer);

    if (ret == 0)
    {
        NEA_DebugPrint("Error while loading palette");
        NEA_MaterialDelete(tex);
        return 0;
    }

    NEA_MaterialSetPalette(tex, pal);
    return 1;
}
