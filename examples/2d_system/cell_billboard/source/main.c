// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Warioware64, 2026
//
// This file is part of Nitro Engine Advanced

// The same cell bank again, this time as camera-facing billboards standing in
// a 3D scene.
//
// Nothing about the .neacell changed: the evaluator resolves the same pose,
// and this backend lays it onto the plane that faces the camera instead of
// onto the screen. NEA_CELL_ANCHOR_BOTTOM puts the bottom of the cell's
// bounding box at the world position, so the characters' feet stay on the
// ground as the camera orbits.

#include <NEAMain.h>

#include "hopper_atlas_bin.h"
#include "hopper_pal_bin.h"
#include "hopper_cells_bin.h"

#define NUM_HOPPERS 5

static NEA_Camera *camera;
static NEA_CellData *bank;
static NEA_CellAnim *hoppers[NUM_HOPPERS];

static const struct { float x, z; } spots[NUM_HOPPERS] = {
    { 0.0f, 0.0f }, { 2.5f, 1.0f }, { -2.0f, 1.8f },
    { 1.2f, -2.2f }, { -1.6f, -1.4f },
};

static void ground(void)
{
    // A grid, so the billboards visibly stand on something and the orbit reads
    // as a camera move rather than the sprites sliding around.
    //
    // Lighting off: with a light enabled the hardware ignores the vertex
    // colour and shades from the normals, which these quads do not have, so
    // the whole grid would come out black.
    NEA_PolyFormat(31, 0, 0, NEA_CULL_NONE, 0);
    GFX_TEX_FORMAT = 0;
    NEA_PolyBegin(GL_QUADS);
    for (int z = -4; z < 4; z++)
    {
        for (int x = -4; x < 4; x++)
        {
            u32 c = ((x + z) & 1) ? RGB15(6, 9, 7) : RGB15(4, 6, 5);
            NEA_PolyColor(c);
            NEA_PolyVertex(x, 0, z);
            NEA_PolyVertex(x + 1, 0, z);
            NEA_PolyVertex(x + 1, 0, z + 1);
            NEA_PolyVertex(x, 0, z + 1);
        }
    }
}

static void draw_3d(void)
{
    NEA_CameraUse(camera);
    ground();

    for (int i = 0; i < NUM_HOPPERS; i++)
    {
        NEA_CellAnimDrawBillboard(hoppers[i], spots[i].x, 0.0f, spots[i].z);
    }
}

int main(int argc, char *argv[])
{
    irqEnable(IRQ_HBLANK);
    irqSet(IRQ_VBLANK, NEA_VBLFunc);
    irqSet(IRQ_HBLANK, NEA_HBLFunc);

    NEA_Init3D();
    consoleDemoInit();

    camera = NEA_CameraCreate();
    NEA_CameraSet(camera,
                  0, 2, 5,
                  0, 0.5, 0,
                  0, 1, 0);

    NEA_CellSystemReset(8);
    // Billboards need to know which way to face, and the camera is a system
    // setting rather than a per-instance one, exactly as it is for particles.
    NEA_CellAnimSetCamera(camera);

    // Load the bank first: it records how big its atlas is, and hardcoding
    // that here would break silently the next time the art is repacked
    // into a different shape.
    bank = NEA_CellDataLoad(hopper_cells_bin);
    if (bank == NULL)
    {
        while (1)
            swiWaitForVBlank();
    }

    NEA_Material *atlas = NEA_MaterialCreate();
    NEA_Palette *pal = NEA_PaletteCreate();
    NEA_MaterialTexLoad(atlas, NEA_PAL16,
                        bank->atlases[0].width,
                        bank->atlases[0].height,
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
        NEA_CellAnimSetAnchor(hoppers[i], NEA_CELL_ANCHOR_BOTTOM);
        NEA_CellAnimPlay(hoppers[i], 0, floattof32(0.5f + 0.25f * i));
    }

    printf("Cell animation: billboards\n\n");
    printf("The camera orbits; the\n");
    printf("sprites keep facing it.\n\n");
    printf("A/B  bigger / smaller\n");

    int angle = 0;
    int upp_shift = 32;

    while (1)
    {
        NEA_WaitForVBL(NEA_UPDATE_CELL);

        scanKeys();
        uint32_t keys = keysHeld();
        if ((keys & KEY_A) && upp_shift > 8)
            upp_shift--;
        if ((keys & KEY_B) && upp_shift < 96)
            upp_shift++;

        for (int i = 0; i < NUM_HOPPERS; i++)
            NEA_CellAnimSetUnitsPerPixelI(hoppers[i],
                                          divf32(inttof32(1),
                                                 inttof32(upp_shift)));

        angle = (angle + 1) & 511;
        NEA_CameraSetI(camera,
                       mulf32(inttof32(6), cosLerp(angle << 6)),
                       inttof32(2),
                       mulf32(inttof32(6), sinLerp(angle << 6)),
                       0, floattof32(0.5f), 0,
                       0, inttof32(1), 0);

        NEA_Process(draw_3d);
    }

    return 0;
}
