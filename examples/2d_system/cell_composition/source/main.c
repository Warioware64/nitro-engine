// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Warioware64, 2026
//
// This file is part of Nitro Engine Advanced

// Multi-cell composition: one entity, several sequences, one clock each.
//
// A cell animation usually plays one sequence. A multi-cell plays several at
// once -- here a body, a head, a swinging arm and a waving flag -- each with
// its own frame list, its own length and its own play head. That is what
// retail NMCR banks were for, and it is why the four pieces below are 24, 20,
// 15 and 28 ticks long: coprime, so they drift against each other and only
// realign every fourteen seconds. Anything that quietly resampled the nodes
// onto one shared frame counter would be obvious within a second.
//
// The nodes are reachable individually with NEA_CellAnimGetNode(), so a game
// can pause an arm, speed up a cape, or leave the rest alone.
//
// The bottom screen draws the same composed entity as hardware OBJ sprites,
// which is the awkward case: binding OAM and seating a multi-cell can happen
// in either order, and a node seated after the bind still has to end up with
// sprites of its own.

#include <NEAMain.h>

#include "hero_atlas_bin.h"
#include "hero_pal_bin.h"
#include "hero_cells_bin.h"
#include "hero_gfx_bin.h"

#define NODE_FLAG 0
#define NODE_BODY 1
#define NODE_ARM  2
#define NODE_HEAD 3

static NEA_CellData *bank;
static NEA_CellAnim *hero;
static NEA_CellAnim *reference;

static int composition;   // which multi-cell is seated
static bool arm_frozen;

static void draw_3d(void)
{
    NEA_2DViewInit();

    // Left: the composed entity. Right: the same bank playing only the body
    // sequence, so the difference composition makes is on screen at once.
    NEA_CellAnimDraw2D(hero, 80, 150);
    NEA_CellAnimDraw2D(reference, 190, 150);
}

int main(int argc, char *argv[])
{
    irqEnable(IRQ_HBLANK);
    irqSet(IRQ_VBLANK, NEA_VBLFunc);
    irqSet(IRQ_HBLANK, NEA_HBLFunc);

    NEA_Init3D();

    // Leave bank C to the console; A and B are far more texture VRAM than a
    // 2 KB atlas needs.
    NEA_TextureSystemReset(0, 0, NEA_VRAM_AB);

    // Order matters here. consoleDemoInit() assigns the whole sub display
    // register, and that does not just clear the sprite-enable bit -- it also
    // resets OBJ mapping from 1D back to 2D, after which a sprite whose tiles
    // were laid out for 1D draws only its first row of tiles and looks like a
    // sliver of itself. So let the console go first and let NEA_Hw2DInit()'s
    // oamInit() have the last word on the OBJ bits; the only thing left to put
    // back afterwards is the console's own background layer.
    consoleDemoInit();

    // The sub engine's OBJ bank carries the sprites. The main engine's OBJ
    // banks all overlap 3D texture memory, which is why they are on the bottom
    // screen.
    NEA_Hw2DVRAMConfig vram = { 0 };
    vram.sub_obj = NEA_VRAM_I;
    if (NEA_Hw2DInit(&vram) != 0)
    {
        while (1)
            swiWaitForVBlank();
    }

    REG_DISPCNT_SUB |= DISPLAY_BG0_ACTIVE;

    NEA_CellSystemReset(16);

    // Load the bank first: it records how big its atlas is, and hardcoding
    // that here would break silently the next time the art is repacked
    // into a different shape.
    bank = NEA_CellDataLoad(hero_cells_bin);
    if (bank == NULL)
    {
        while (1)
            swiWaitForVBlank();
    }

    int vram_free = NEA_TextureFreeMem();

    NEA_Material *atlas = NEA_MaterialCreate();
    NEA_Palette *pal = NEA_PaletteCreate();
    NEA_MaterialTexLoad(atlas, NEA_PAL16,
                        bank->atlases[0].width,
                        bank->atlases[0].height,
                        NEA_TEXGEN_TEXCOORD | NEA_TEXTURE_COLOR0_TRANSPARENT,
                        (void *)hero_atlas_bin);
    NEA_PaletteLoad(pal, (void *)hero_pal_bin, 16, NEA_PAL16);
    NEA_MaterialSetPalette(atlas, pal);
    NEA_MaterialSetName(atlas, "hero");

    // The atlas is 128x32, not a square 128x128: the pieces all sit in the top
    // 32 rows, and the DS sizes the two axes of a texture independently, so
    // rounding both up to the larger one would cost four times this.
    int atlas_bytes = vram_free - NEA_TextureFreeMem();

    if (NEA_CellDataBindMaterials(bank) != 0)
    {
        while (1)
            swiWaitForVBlank();
    }

    hero = NEA_CellAnimCreate();
    NEA_CellAnimSetData(hero, bank);
    // Seats one child instance per node and starts them all. Each keeps its
    // own play head from here on.
    NEA_CellAnimPlayMulti(hero, 0);

    reference = NEA_CellAnimCreate();
    NEA_CellAnimSetData(reference, bank);
    NEA_CellAnimPlayNamed(reference, "body", floattof32(1.0f));


    // Bound after the nodes were seated, so the existing children are bound
    // along with the parent. Pressing B re-seats them, which exercises the
    // other order: a node created while the parent is already bound.
    NEA_CellAnimBindOAM(hero, NEA_ENGINE_SUB, hero_gfx_bin,
                        hero_gfx_bin_size, NEA_CELL_OAM_SKIP);
    NEA_CellAnimLoadOAMPalette(hero, hero_pal_bin, 16, 0);

    printf("Multi-cell composition\n");
    printf("======================\n\n");
    printf("top:    4 nodes, 4 clocks,\n");
    printf("        and the body alone\n");
    printf("bottom: the same, as sprites\n\n");
    printf("atlas in texture VRAM: %d B\n", atlas_bytes);
    printf("OBJ tiles in main RAM: %d B\n\n", (int)hero_gfx_bin_size);
    printf("A    freeze/thaw the arm\n");
    printf("B    swap composition\n");
    printf("L/R  slow/speed the flag\n");

    int32_t flag_speed = floattof32(1.0f);

    while (1)
    {
        NEA_WaitForVBL(NEA_UPDATE_CELL | NEA_UPDATE_HW2D);

        scanKeys();
        uint32_t down = keysDown();
        uint32_t held = keysHeld();

        if (down & KEY_A)
        {
            // One node, on its own. The others do not notice.
            NEA_CellAnim *arm = NEA_CellAnimGetNode(hero, NODE_ARM);
            if (arm)
            {
                arm_frozen = !arm_frozen;
                NEA_CellAnimPause(arm, arm_frozen);
            }
        }

        if (down & KEY_B)
        {
            // "plain" drops the flag node and keeps the other three. Re-seating
            // restarts their clocks, which is the honest behaviour: the nodes
            // of one composition are not the nodes of another.
            composition ^= 1;
            NEA_CellAnimPlayMulti(hero, composition);
            arm_frozen = false;
        }

        if ((held & KEY_L) && flag_speed > floattof32(0.1f))
            flag_speed -= floattof32(0.02f);
        if ((held & KEY_R) && flag_speed < floattof32(4.0f))
            flag_speed += floattof32(0.02f);

        NEA_CellAnim *flag = (composition == 0)
            ? NEA_CellAnimGetNode(hero, NODE_FLAG) : NULL;
        if (flag)
            NEA_CellAnimSetSpeed(flag, flag_speed);

        // The same composed pose, pushed into the OBJ pool on the sub screen.
        NEA_CellAnimApplyOAM(hero, 128, 180);

        NEA_Process(draw_3d);
    }

    return 0;
}
