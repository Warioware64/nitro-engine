// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Warioware64, 2026
//
// This file is part of Nitro Engine Advanced

// The same cell bank on two backends, and what each one costs in VRAM.
//
// The top screen draws it as textured quads through the 3D engine; the bottom
// screen draws it as hardware OBJ sprites, out of the same .neacell, with no
// per-backend authoring.
//
// They do not cost the same. The 3D path has to have the atlas resident in
// texture VRAM, because that is the only memory the geometry engine can sample
// from -- there is no way to draw a textured polygon out of main RAM. The
// hardware OBJ path needs none of it: its graphics are streamed a frame at a
// time out of the .ncgfx blob in main RAM into the OBJ bank.
//
// Press A to prove it. The atlas is RAM-backed, so it can be evicted from
// texture VRAM and brought back; while it is gone the quads disappear, the
// sprites keep animating, and the free-VRAM figure goes up by the size of the
// atlas. A project that only ever draws cells as sprites should not create the
// material at all.

#include <NEAMain.h>

#include "hopper_atlas_bin.h"
#include "hopper_pal_bin.h"
#include "hopper_cells_bin.h"
#include "hopper_gfx_bin.h"

static NEA_CellData *bank;
static NEA_CellAnim *quads;
static NEA_CellAnim *sprites;

static void draw_3d(void)
{
    NEA_2DViewInit();
    NEA_CellAnimDraw2D(quads, 128, 140);
}

int main(int argc, char *argv[])
{
    irqEnable(IRQ_HBLANK);
    irqSet(IRQ_VBLANK, NEA_VBLFunc);
    irqSet(IRQ_HBLANK, NEA_HBLFunc);

    NEA_Init3D();

    // Leave bank C to the console. A and B are 256 KB between them, far more
    // than one small atlas needs, and this is what makes the free-VRAM figure
    // below mean something.
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

    NEA_CellSystemReset(8);

    // Load the bank first: it records how big its atlas is, and hardcoding
    // that here would break silently the next time the art is repacked into a
    // different shape.
    bank = NEA_CellDataLoad(hopper_cells_bin);
    if (bank == NULL)
    {
        printf("failed to load the cell bank\n");
        while (1)
            swiWaitForVBlank();
    }

    NEA_Material *atlas = NEA_MaterialCreate();
    NEA_Palette *pal = NEA_PaletteCreate();

    // RAM-backed. On a material marked this way, NEA_MaterialTexLoad() only
    // stashes a copy in main RAM -- it does not touch VRAM at all, and
    // NEA_MaterialTexVramLoad() below is what actually uploads it. That split
    // is the whole point: the VRAM copy can be dropped and put back, and the
    // RAM copy is what it comes back from.
    NEA_MaterialRamInit(atlas);
    NEA_PaletteRamInit(pal);

    NEA_MaterialTexLoad(atlas, NEA_PAL16,
                        bank->atlases[0].width, bank->atlases[0].height,
                        NEA_TEXGEN_TEXCOORD | NEA_TEXTURE_COLOR0_TRANSPARENT,
                        (void *)hopper_atlas_bin);
    NEA_PaletteLoad(pal, (void *)hopper_pal_bin, 16, NEA_PAL16);
    NEA_MaterialSetPalette(atlas, pal);
    NEA_MaterialSetName(atlas, "hopper");

    // Nothing is in VRAM yet, so this baseline is the empty figure.
    int vram_free = NEA_TextureFreeMem();

    NEA_MaterialTexVramLoad(atlas);
    NEA_PaletteVramLoad(pal);

    int atlas_bytes = vram_free - NEA_TextureFreeMem();

    if (NEA_CellDataBindMaterials(bank) != 0)
    {
        printf("no material for the atlas\n");
        while (1)
            swiWaitForVBlank();
    }

    quads = NEA_CellAnimCreate();
    NEA_CellAnimSetData(quads, bank);
    NEA_CellAnimPlay(quads, 0, floattof32(1.0f));

    sprites = NEA_CellAnimCreate();
    NEA_CellAnimSetData(sprites, bank);

    // The blob has to outlive the binding: it is what each frame's tiles are
    // copied out of, not a one-off upload.
    NEA_CellAnimBindOAM(sprites, NEA_ENGINE_SUB, hopper_gfx_bin,
                        hopper_gfx_bin_size, NEA_CELL_OAM_SKIP);
    NEA_CellAnimLoadOAMPalette(sprites, hopper_pal_bin, 16, 0);

    NEA_CellAnimPlay(sprites, 0, floattof32(1.0f));

    bool resident = true;
    int x = 128;

    while (1)
    {
        NEA_WaitForVBL(NEA_UPDATE_CELL | NEA_UPDATE_HW2D);

        scanKeys();
        uint32_t keys = keysHeld();
        uint32_t down = keysDown();

        if (keys & KEY_LEFT)
            x -= 2;
        if (keys & KEY_RIGHT)
            x += 2;

        if (down & KEY_A)
        {
            resident = !resident;
            if (resident)
            {
                NEA_MaterialTexVramLoad(atlas);
                NEA_PaletteVramLoad(pal);
            }
            else
            {
                NEA_MaterialTexVramUnload(atlas);
                NEA_PaletteVramUnload(pal);
            }
        }

        consoleClear();
        printf("Cell animation: two backends\n");
        printf("============================\n\n");
        printf("top:    3D textured quads\n");
        printf("bottom: hardware OBJ sprites\n\n");
        printf("atlas in texture VRAM: %s\n", resident ? "yes" : "NO ");
        printf("  it costs        %5d B\n", atlas_bytes);
        printf("  free tex VRAM   %5d B\n", NEA_TextureFreeMem());
        printf("  OBJ tiles in RAM %4d B\n", (int)hopper_gfx_bin_size);
        printf("\nA      evict / restore it\n");
        printf("LEFT/RIGHT  move the sprite\n");

        // Push the resolved pose into the OBJ pool. The OAM flush itself is
        // NEA_UPDATE_HW2D's job, above.
        NEA_CellAnimApplyOAM(sprites, x, 150);

        NEA_Process(draw_3d);
    }

    return 0;
}
