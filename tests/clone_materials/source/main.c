// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Antonio Niño Díaz, 2008-2022
//
// This file is part of Nitro Engine Advanced

#include <NEAMain.h>

#include "a3pal32.h"

#define NUM_CLONES 5

int main(int argc, char *argv[])
{
    // This is needed for special screen effects
    irqEnable(IRQ_HBLANK);
    irqSet(IRQ_VBLANK, NEA_VBLFunc);
    irqSet(IRQ_VBLANK, NEA_HBLFunc);

    // Init console and Nitro Engine Advanced
    NEA_Init3D();
    // Use banks A and B for teapots. libnds uses bank C for the demo text
    // console.
    NEA_TextureSystemReset(0, 0, NEA_VRAM_AB);
    // This is needed to print text
    consoleDemoInit();

    NEA_Material *Material[NUM_CLONES];
    for (int i = 0; i < NUM_CLONES; i++)
        Material[i] = NEA_MaterialCreate();

    NEA_Palette *Palette = NEA_PaletteCreate();

    int total_tex_mem = NEA_TextureFreeMem();
    int total_pal_mem = NEA_PaletteFreeMem();

    NEA_MaterialTexLoad(Material[0],
                       NEA_A3PAL32, // Texture type
                       64, 200,    // Width, height (in pixels)
                       NEA_TEXGEN_TEXCOORD, a3pal32Bitmap);
    NEA_PaletteLoad(Palette, a3pal32Pal, 32, NEA_A3PAL32);
    NEA_MaterialSetPalette(Material[0], Palette);

    for (int i = 1; i < NUM_CLONES; i++)
        NEA_MaterialClone(Material[0], Material[i]);

    int remaining_tex_mem = NEA_TextureFreeMem();
    int remaining_pal_mem = NEA_PaletteFreeMem();

    printf("Total:     %6d | %6d\n"
           "Remaining: %6d | %6d\n",
           total_tex_mem, total_pal_mem,
           remaining_tex_mem, remaining_pal_mem);

    // Delete all materials but one, so that the texture isn't freed yet
    for (int i = 0; i < NUM_CLONES - 1; i++)
        NEA_MaterialDelete(Material[i]);

    if (remaining_tex_mem != NEA_TextureFreeMem())
        printf("Texture memory wrongly freed\n");

    if (remaining_pal_mem != NEA_PaletteFreeMem())
        printf("Palette memory wrongly freed\n");

    // Delete the last material so that the texture is freed
    NEA_MaterialDelete(Material[NUM_CLONES - 1]);
    NEA_PaletteDelete(Palette);

    if (total_tex_mem != NEA_TextureFreeMem())
        printf("Texture memory not freed\n");

    if (total_pal_mem != NEA_PaletteFreeMem())
        printf("Palette memory not freed\n");

    printf("Tests finished\n");

    while (1)
        NEA_WaitForVBL(0);

    return 0;
}
