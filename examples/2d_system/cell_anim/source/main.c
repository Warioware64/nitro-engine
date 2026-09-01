// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Warioware64, 2026
//
// This file is part of Nitro Engine Advanced

// Cell animation drawn as textured quads through the 3D engine.
//
// The .neacell holds cells and timing only; the artwork is an ordinary NDS
// texture the application loads and registers under the name the file expects.
// Three instances share one bank, at different speeds and with different
// per-instance transforms, to show that playback state is per-instance and the
// data is not.

#include <NEAMain.h>

#include "hopper_atlas_bin.h"
#include "hopper_pal_bin.h"
#include "hopper_cells_bin.h"

#define NUM_HOPPERS 3

static NEA_CellData *bank;
static NEA_CellAnim *hoppers[NUM_HOPPERS];
static int spin;

static void draw_3d(void)
{
    NEA_2DViewInit();

    NEA_CellAnimDraw2D(hoppers[0], 64, 110);
    NEA_CellAnimDraw2D(hoppers[1], 128, 130);
    NEA_CellAnimDraw2D(hoppers[2], 196, 150);
}

int main(int argc, char *argv[])
{
    irqEnable(IRQ_HBLANK);
    irqSet(IRQ_VBLANK, NEA_VBLFunc);
    irqSet(IRQ_HBLANK, NEA_HBLFunc);

    NEA_Init3D();
    consoleDemoInit();

    // The atlas is a plain material. The cell bank finds it by name, so
    // nothing about the animation has to know where it came from.
    NEA_CellSystemReset(8);

    // Load the bank first: it records how big its atlas is, and hardcoding
    // that here would break silently the next time the art is repacked into a
    // different shape.
    bank = NEA_CellDataLoad(hopper_cells_bin);
    if (bank == NULL)
    {
        while (1)
            swiWaitForVBlank();
    }

    NEA_Material *atlas = NEA_MaterialCreate();
    NEA_Palette *pal = NEA_PaletteCreate();

    NEA_MaterialTexLoad(atlas, NEA_PAL16,
                        bank->atlases[0].width, bank->atlases[0].height,
                        NEA_TEXGEN_TEXCOORD | NEA_TEXTURE_COLOR0_TRANSPARENT,
                        (void *)hopper_atlas_bin);
    NEA_PaletteLoad(pal, (void *)hopper_pal_bin, 16, NEA_PAL16);
    NEA_MaterialSetPalette(atlas, pal);
    NEA_MaterialSetName(atlas, "hopper");
    if (NEA_CellDataBindMaterials(bank) != 0)
    {
        printf("failed to load the cell bank\n");
        while (1)
            swiWaitForVBlank();
    }

    for (int i = 0; i < NUM_HOPPERS; i++)
    {
        hoppers[i] = NEA_CellAnimCreate();
        NEA_CellAnimSetData(hoppers[i], bank);
        // Different speeds, so the three fall out of phase immediately.
        NEA_CellAnimPlay(hoppers[i], 0, floattof32(0.6f + 0.4f * i));
    }

    // The rightmost one is tinted and half transparent, which only the 3D
    // backends can do -- the hardware OBJ path has no per-part colour.
    NEA_CellAnimSetParams(hoppers[2], 18, 4, RGB15(31, 20, 12));

    printf("Cell animation: 3D quads\n\n");
    printf("UP/DOWN   scale\n");
    printf("LEFT/RIGHT spin\n");
    printf("A         pause\n");

    int scale = floattof32(1.0f);

    while (1)
    {
        NEA_WaitForVBL(NEA_UPDATE_CELL);

        scanKeys();
        uint32_t keys = keysHeld();
        uint32_t down = keysDown();

        if (keys & KEY_UP)
            scale += floattof32(0.02f);
        if (keys & KEY_DOWN)
            scale -= floattof32(0.02f);
        if (scale < floattof32(0.25f))
            scale = floattof32(0.25f);
        if (scale > floattof32(3.0f))
            scale = floattof32(3.0f);

        if (keys & KEY_LEFT)
            spin -= 4;
        if (keys & KEY_RIGHT)
            spin += 4;

        if (down & KEY_A)
        {
            for (int i = 0; i < NUM_HOPPERS; i++)
                NEA_CellAnimPause(hoppers[i], hoppers[i]->playing);
        }

        // The instance transform sits on top of whatever the file animates.
        NEA_CellAnimSetTransformI(hoppers[1], spin, scale, scale);

        NEA_Process(draw_3d);
    }

    return 0;
}
