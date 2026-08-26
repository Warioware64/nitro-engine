// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Nitro Engine Advanced contributors, 2026
//
// This file is part of Nitro Engine Advanced

// Material animation, version 2: one file driving several materials by name.
//
// A version 1 animation instance is one draw call's worth of GPU state. You
// call NEA_AnimMatApply() and then NEA_ModelDraw(), and since NEA_ModelDraw()
// walks every submesh internally there is no way to give two submeshes of one
// model two different animations. Retail DS material animations solved this by
// addressing a material by name, and so does version 2.
//
// The model here has four named materials. The animation has three targets, so
// NEA_ModelSetAnimMat() matches three of them and leaves the fourth alone:
//
//   scroll   UV scroll, stored baked -- one value per frame, one indexed load
//   pulse    alpha and vertex colour, keyframed and interpolated
//   flip     palette cycling through a texture/palette swap track, plus a
//            constant alpha track that costs no per-frame work at all
//   still    no target, so it just draws
//
// The name matching happens once, inside NEA_ModelSetAnimMat(). Drawing only
// looks up an index, so nothing compares strings per frame.

#include <NEAMain.h>

#include "panels_mesh_bin.h"
#include "panels_bin.h"

#include "scroll.h"
#include "pulse.h"
#include "flip.h"
#include "still.h"

// Three palettes for the one 16-colour texture the "flip" panel uses. The swap
// track moves between them with the texture index left at 0xFF, which means
// "leave the texture alone" -- so the image never changes, only its colours.
#define PAL_COLORS 16

static const u16 PaletteEmber[PAL_COLORS] = {
    RGB15(2, 0, 0),   RGB15(6, 1, 0),   RGB15(10, 2, 0),  RGB15(14, 3, 0),
    RGB15(18, 5, 0),  RGB15(22, 7, 0),  RGB15(26, 9, 0),  RGB15(30, 12, 0),
    RGB15(31, 15, 2), RGB15(31, 18, 5), RGB15(31, 21, 8), RGB15(31, 24, 12),
    RGB15(31, 27, 17), RGB15(31, 29, 22), RGB15(31, 31, 27), RGB15(31, 31, 31),
};

static const u16 PaletteIce[PAL_COLORS] = {
    RGB15(0, 0, 4),   RGB15(0, 2, 8),   RGB15(0, 4, 12),  RGB15(0, 6, 16),
    RGB15(1, 9, 20),  RGB15(2, 12, 24), RGB15(3, 15, 27), RGB15(5, 18, 30),
    RGB15(8, 21, 31), RGB15(11, 24, 31), RGB15(15, 26, 31), RGB15(19, 28, 31),
    RGB15(23, 29, 31), RGB15(26, 30, 31), RGB15(29, 31, 31), RGB15(31, 31, 31),
};

static const u16 PaletteMoss[PAL_COLORS] = {
    RGB15(1, 3, 1),   RGB15(2, 6, 2),   RGB15(3, 9, 3),   RGB15(4, 12, 4),
    RGB15(5, 15, 5),  RGB15(7, 18, 6),  RGB15(9, 21, 7),  RGB15(12, 24, 9),
    RGB15(15, 26, 11), RGB15(18, 28, 13), RGB15(21, 29, 16), RGB15(24, 30, 19),
    RGB15(26, 31, 22), RGB15(28, 31, 25), RGB15(30, 31, 28), RGB15(31, 31, 31),
};

typedef struct {
    NEA_Camera *Camera;
    NEA_Model *Model;
    bool animate;
} SceneData;

void Draw3DScene(void *arg)
{
    SceneData *Scene = arg;

    NEA_CameraUse(Scene->Camera);

    // The base format every target starts from. A track only overrides the
    // fields it actually animates, so this still decides the rest.
    NEA_PolyFormat(31, 0, 0, NEA_CULL_NONE, 0);

    // No per-submesh work here: the model knows which target drives which
    // submesh, because NEA_ModelSetAnimMat() worked it out at bind time.
    NEA_ModelDraw(Scene->Model);
}

int main(int argc, char *argv[])
{
    SceneData Scene = { 0 };

    irqEnable(IRQ_HBLANK);
    irqSet(IRQ_VBLANK, NEA_VBLFunc);
    irqSet(IRQ_HBLANK, NEA_HBLFunc);

    NEA_Init3D();
    // libnds uses VRAM_C for the text console, reserve A and B only
    NEA_TextureSystemReset(0, 0, NEA_VRAM_AB);
    consoleDemoInit();

    NEA_AnimMatSystemReset(4);

    Scene.Camera = NEA_CameraCreate();
    NEA_CameraSet(Scene.Camera,
                 0, 0, -7,
                 0, 0, 0,
                 0, 1, 0);

    // ---- Materials, named to match the model's submeshes ----------------

    NEA_Material *MatScroll = NEA_MaterialCreate();
    NEA_Material *MatPulse = NEA_MaterialCreate();
    NEA_Material *MatFlip = NEA_MaterialCreate();
    NEA_Material *MatStill = NEA_MaterialCreate();

    // Wrapping matters for the scrolling panel: without it the texture clamps
    // at its edge instead of repeating.
    NEA_MaterialTexLoad(MatScroll, NEA_A1RGB5, 64, 64,
                       NEA_TEXGEN_TEXCOORD | NEA_TEXTURE_WRAP_S
                       | NEA_TEXTURE_WRAP_T, scrollBitmap);
    NEA_MaterialTexLoad(MatPulse, NEA_A1RGB5, 64, 64,
                       NEA_TEXGEN_TEXCOORD, pulseBitmap);
    NEA_MaterialTexLoad(MatFlip, NEA_PAL16, 64, 64,
                       NEA_TEXGEN_TEXCOORD, flipBitmap);
    NEA_MaterialTexLoad(MatStill, NEA_A1RGB5, 64, 64,
                       NEA_TEXGEN_TEXCOORD, stillBitmap);

    NEA_MaterialSetName(MatScroll, "scroll");
    NEA_MaterialSetName(MatPulse, "pulse");
    NEA_MaterialSetName(MatFlip, "flip");
    NEA_MaterialSetName(MatStill, "still");

    // The three palettes the swap track moves between.
    NEA_Palette *PalEmber = NEA_PaletteCreate();
    NEA_Palette *PalIce = NEA_PaletteCreate();
    NEA_Palette *PalMoss = NEA_PaletteCreate();

    NEA_PaletteLoad(PalEmber, PaletteEmber, PAL_COLORS, NEA_PAL16);
    NEA_PaletteLoad(PalIce, PaletteIce, PAL_COLORS, NEA_PAL16);
    NEA_PaletteLoad(PalMoss, PaletteMoss, PAL_COLORS, NEA_PAL16);

    NEA_MaterialSetPalette(MatFlip, PalEmber);

    // Unlit, so the animation's own colours are what you see.
    NEA_Material *mats[4] = { MatScroll, MatPulse, MatFlip, MatStill };
    for (int i = 0; i < 4; i++)
    {
        NEA_MaterialSetProperties(mats[i],
                                 RGB15(0, 0, 0),    // diffuse
                                 RGB15(0, 0, 0),    // ambient
                                 RGB15(0, 0, 0),    // specular
                                 RGB15(31, 31, 31), // emission
                                 false, false);
    }

    // ---- Model ----------------------------------------------------------

    Scene.Model = NEA_ModelCreate(NEA_Static);
    NEA_ModelLoadMultiMesh(Scene.Model, panels_mesh_bin);

    // Binds each submesh to the material whose name matches the one the DLMM
    // file recorded.
    NEA_ModelAutoBindMaterials(Scene.Model);

    // ---- Animation ------------------------------------------------------

    NEA_AnimMatData *Data = NEA_AnimMatDataLoad(panels_bin);
    NEA_AnimMatInstance *Anim = NEA_AnimMatCreate();

    NEA_AnimMatSetData(Anim, Data);
    NEA_AnimMatSetBasePolyFormat(Anim, 31, 0, 0, NEA_CULL_NONE, 0);

    // The tables the texture/palette swap track indexes into. No textures are
    // given, because every key leaves the texture alone and moves the palette.
    NEA_Palette *pal_table[3] = { PalEmber, PalIce, PalMoss };
    NEA_AnimMatSetTexPalTables(Anim, NULL, 0, pal_table, 3);

    NEA_AnimMatStart(Anim, NEA_ANIM_LOOP, floattof32(1.0));

    // This is the whole of the version 2 binding: match the animation's target
    // names against the model's submesh material names, once.
    NEA_ModelSetAnimMat(Scene.Model, Anim);

    Scene.animate = true;

    while (1)
    {
        NEA_WaitForVBL(NEA_UPDATE_ANIM_MAT);

        scanKeys();
        uint32_t keys = keysDown();

        if (keys & KEY_A)
        {
            Scene.animate = !Scene.animate;
            // Unbinding leaves every submesh drawing with its own material,
            // which is what the model looks like with no animation at all.
            NEA_ModelSetAnimMat(Scene.Model, Scene.animate ? Anim : NULL);
        }
        if (keys & KEY_B)
            NEA_AnimMatPause(Anim, false);
        if (keys & KEY_START)
            NEA_AnimMatSetFrame(Anim, 0);

        int frame = NEA_AnimMatGetFrame(Anim) >> 12;

        printf("\x1b[0;0H"
               "AnimMat v2: many materials\n"
               "==========================\n\n"
               "One file, three targets,\n"
               "matched by material name.\n\n"
               "A  Animation: %s\n"
               "Start  Rewind\n\n"
               "frame %2d / %d\n\n"
               "scroll  baked UV scroll\n"
               "pulse   keyed alpha+color\n"
               "flip    palette swap\n"
               "still   no target, untouched\n",
               Scene.animate ? "on " : "off",
               frame, Data->num_frames);

        NEA_ProcessArg(Draw3DScene, &Scene);
    }

    return 0;
}
