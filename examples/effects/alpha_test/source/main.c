// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Nitro Engine Advanced contributors, 2026
//
// This file is part of Nitro Engine Advanced

// Alpha test: cutout textures without paying for translucency.
//
// A leaf texture has soft edges, and there are two ways to draw it. The obvious
// one is to make the polygons translucent, which costs a manual depth sort, a
// set of polygon IDs so overlapping quads blend instead of cancelling, and still
// falls apart when two of them cross.
//
// The other is to leave the polygons fully opaque and let the alpha test throw
// away the pixels below a threshold. They then write depth like any other opaque
// polygon: no sorting, no IDs, and crossing quads simply work.
//
// Press B to switch between the two on the same geometry. The quads are
// deliberately arranged to intersect, which is where the translucent version
// shows its seams.
//
// Cost: nothing. It is one comparison in the pixel pipeline that is already
// happening (against zero) with a different value on one side.

#include <NEAMain.h>

#include "leaves.h"

#define NUM_QUADS 9

typedef struct {
    NEA_Camera *Camera;
    NEA_Material *Leaves;
    NEA_Palette *Palette;
    bool use_alpha_test;
    u32 threshold;
    int angle;
} SceneData;

// A cluster of quads at assorted angles and depths, several of them crossing.
static const struct { int x, z, rot; } QUADS[NUM_QUADS] = {
    {  0,  0,   0 }, {  0,  0, 128 },
    { -1,  1,  40 }, {  1,  1, 300 },
    { -1, -1, 200 }, {  1, -1,  90 },
    {  2,  0, 160 }, { -2,  0,  20 },
    {  0,  2, 250 }
};

static void DrawLeafQuad(int x, int z, int rot)
{
    MATRIX_PUSH = 0;

    MATRIX_TRANSLATE = inttof32(x);
    MATRIX_TRANSLATE = 0;
    MATRIX_TRANSLATE = inttof32(z);

    glRotateYi(rot << 6);

    NEA_PolyBegin(GL_QUAD);

        NEA_PolyTexCoord(0, 64);
        NEA_PolyVertex(-1, -1, 0);

        NEA_PolyTexCoord(64, 64);
        NEA_PolyVertex(1, -1, 0);

        NEA_PolyTexCoord(64, 0);
        NEA_PolyVertex(1, 1, 0);

        NEA_PolyTexCoord(0, 0);
        NEA_PolyVertex(-1, 1, 0);

    NEA_PolyEnd();

    MATRIX_POP = 1;
}

void Draw3DScene(void *arg)
{
    SceneData *Scene = arg;

    NEA_CameraUse(Scene->Camera);

    NEA_MaterialUse(Scene->Leaves);

    if (Scene->use_alpha_test)
    {
        // Fully opaque polygons. The alpha test does the cutting, and the depth
        // buffer sorts everything for free.
        NEA_PolyFormat(31, 0, 0, NEA_CULL_NONE, 0);

        for (int i = 0; i < NUM_QUADS; i++)
            DrawLeafQuad(QUADS[i].x, QUADS[i].z, QUADS[i].rot);
    }
    else
    {
        // The translucent version. Alternating polygon IDs are needed or the
        // quads refuse to blend with each other, and even then the result
        // depends on the order they happen to be drawn in.
        for (int i = 0; i < NUM_QUADS; i++)
        {
            NEA_PolyFormat(30, i & 7, 0, NEA_CULL_NONE, 0);
            DrawLeafQuad(QUADS[i].x, QUADS[i].z, QUADS[i].rot);
        }
    }
}

int main(int argc, char *argv[])
{
    SceneData Scene = { 0 };

    irqEnable(IRQ_HBLANK);
    irqSet(IRQ_VBLANK, NEA_VBLFunc);
    irqSet(IRQ_HBLANK, NEA_HBLFunc);

    NEA_Init3D();
    // Use banks A and B for textures. libnds uses bank C for the text console.
    NEA_TextureSystemReset(0, 0, NEA_VRAM_AB);
    consoleDemoInit();

    Scene.Camera = NEA_CameraCreate();
    Scene.Leaves = NEA_MaterialCreate();
    Scene.Palette = NEA_PaletteCreate();
    Scene.use_alpha_test = true;
    Scene.threshold = 8;

    NEA_CameraSet(Scene.Camera,
                 0, 1.5, -5,
                 0, 0, 0,
                 0, 1, 0);

    // A5PAL8 keeps 5 bits of alpha, which is what makes a threshold slider mean
    // anything. A 1 bit format would have nothing to slide.
    NEA_MaterialTexLoad(Scene.Leaves, NEA_A5PAL8, 64, 64, NEA_TEXGEN_TEXCOORD,
                       leavesBitmap);
    NEA_PaletteLoad(Scene.Palette, leavesPal, 8, NEA_A5PAL8);
    NEA_MaterialSetPalette(Scene.Leaves, Scene.Palette);

    NEA_ClearColorSet(RGB15(20, 24, 31), 31, 63);

    NEA_AlphaTestEnable(Scene.threshold);

    bool spin = true;

    while (1)
    {
        NEA_WaitForVBL(0);

        scanKeys();
        uint32_t keys = keysDown();
        uint32_t held = keysHeld();

        if (keys & KEY_B)
        {
            Scene.use_alpha_test = !Scene.use_alpha_test;

            if (Scene.use_alpha_test)
                NEA_AlphaTestEnable(Scene.threshold);
            else
                NEA_AlphaTestDisable();
        }

        if (Scene.use_alpha_test && (held & (KEY_UP | KEY_DOWN)))
        {
            if ((held & KEY_UP) && Scene.threshold < 30)
                Scene.threshold++;
            if ((held & KEY_DOWN) && Scene.threshold > 0)
                Scene.threshold--;

            NEA_AlphaTestEnable(Scene.threshold);
        }

        if (keys & KEY_START)
            spin = !spin;

        if (spin)
            Scene.angle = (Scene.angle + 1) & 511;
        if (held & KEY_LEFT)
            Scene.angle = (Scene.angle - 3) & 511;
        if (held & KEY_RIGHT)
            Scene.angle = (Scene.angle + 3) & 511;

        NEA_CameraSetI(Scene.Camera,
                      5 * sinLerp(Scene.angle << 6), floattof32(1.5),
                      -5 * cosLerp(Scene.angle << 6),
                      0, 0, 0,
                      0, inttof32(1), 0);

        printf("\x1b[0;0H"
               "Alpha test\n"
               "==========\n\n"
               "B  Mode:      %s\n"
               "Up/Down  Threshold: %2lu\n"
               "Start  Spin: %s\n"
               "Left/Right  Orbit\n\n"
               "Opaque quads plus the alpha\n"
               "test need no sorting and no\n"
               "polygon IDs, and crossing   \n"
               "quads just work. The        \n"
               "translucent version needs   \n"
               "both and still seams.       \n",
               Scene.use_alpha_test ? "alpha test " : "translucent",
               (unsigned long)Scene.threshold,
               spin ? "on " : "off");

        NEA_ProcessArg(Draw3DScene, &Scene);
    }

    return 0;
}
