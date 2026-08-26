// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Nitro Engine Advanced contributors, 2026
//
// This file is part of Nitro Engine Advanced

// Toon table ramps.
//
// The GPU has a 32-entry colour table that polygons drawn in
// NEA_TOON_HIGHLIGHT_SHADING mode use instead of their vertex colour. The index
// is the red channel of the lit vertex colour, so entry 0 is unlit and entry 31
// is fully lit.
//
// NEA_SetupToonShadingTables() only ever wrote a two-band step into it. That is
// one thing a 32-entry ramp can do. This example walks through the others: more
// bands, a smooth two-colour gradient, and a multi-stop ramp that shifts hue as
// it goes -- a gradient map, effectively, where shading changes colour and not
// only brightness.
//
// Cost: nothing per frame. Each ramp is 32 halfwords pushed to the GPU once, and
// no VRAM is used. The animated ramp rebuilds itself every frame just to show
// that even that is affordable.

#include <NEAMain.h>

#include "surface.h"
#include "sphere_bin.h"
#include "teapot_bin.h"

typedef struct {
    NEA_Camera *Camera;
    NEA_Model *Sphere, *Teapot;
} SceneData;

enum {
    RAMP_TWO_BAND,
    RAMP_FOUR_BAND,
    RAMP_GRADIENT,
    RAMP_SUNSET,
    RAMP_SUBSURFACE,
    RAMP_FLAT,
    RAMP_ANIMATED,
    RAMP_COUNT
};

static const char *RAMP_NAMES[RAMP_COUNT] = {
    "2-band cel      ",
    "4-band cel      ",
    "cool-warm ramp  ",
    "sunset (3 stops)",
    "subsurface rim  ",
    "flat silhouette ",
    "animated ramp   "
};

static const char *RAMP_NOTES[RAMP_COUNT] = {
    "The classic. Same as    \nNEA_SetupToonShadingTables.",
    "Shadow, two midtones and\na highlight.              ",
    "Blue shadow to warm cream.\nSmooth, but not grey.     ",
    "Multi-stop: purple, red,  \ngold. Hue moves with light.",
    "Red band where light just \ngrazes: fake subsurface.  ",
    "One colour everywhere.    \nShading stops mattering.  ",
    "Rebuilt every frame.      \n32 halfwords is nothing.  "
};

// Applies one of the ramps. `phase` only matters for the animated one.
static void ApplyRamp(int ramp, int phase)
{
    switch (ramp)
    {
        case RAMP_TWO_BAND:
            NEA_ToonTableBands(2, RGB15(6, 7, 12), RGB15(28, 28, 26));
            break;

        case RAMP_FOUR_BAND:
            NEA_ToonTableBands(4, RGB15(5, 6, 12), RGB15(31, 30, 26));
            break;

        case RAMP_GRADIENT:
            NEA_ToonTableGradient(RGB15(4, 6, 14), RGB15(31, 29, 22));
            break;

        case RAMP_SUNSET:
        {
            // Three stops, so the shading travels through a hue rather than
            // just getting brighter.
            static const u32 colors[3] = {
                RGB15(6, 2, 12),  // deep purple in shadow
                RGB15(26, 8, 6),  // red at the midtone
                RGB15(31, 28, 14) // gold at the highlight
            };
            static const u8 indices[3] = { 0, 16, 31 };

            NEA_ToonTableGradientStops(colors, indices, 3);
            break;
        }

        case RAMP_SUBSURFACE:
        {
            // A narrow warm band low in the ramp, where light only grazes the
            // surface. Reads as light bleeding through something thin.
            static const u32 colors[4] = {
                RGB15(3, 3, 6),
                RGB15(26, 6, 8),
                RGB15(22, 20, 20),
                RGB15(31, 31, 31)
            };
            static const u8 indices[4] = { 0, 8, 14, 31 };

            NEA_ToonTableGradientStops(colors, indices, 4);
            break;
        }

        case RAMP_FLAT:
            NEA_ToonTableFill(RGB15(20, 22, 28));
            break;

        case RAMP_ANIMATED:
        {
            // Nothing here is precomputed: the whole table is rebuilt from the
            // current phase every single frame.
            int t = (phase >> 1) & 31;
            u32 lo = RGB15(t, 4, 31 - t);
            u32 hi = RGB15(31, 31 - (t >> 1), t);

            NEA_ToonTableGradient(lo, hi);
            break;
        }
    }
}

void Draw3DScene(void *arg)
{
    SceneData *Scene = arg;

    NEA_CameraUse(Scene->Camera);

    // Mode 2 is what sends a polygon through the toon table.
    NEA_PolyFormat(31, 0, NEA_LIGHT_0, NEA_CULL_BACK,
                  NEA_TOON_HIGHLIGHT_SHADING);

    NEA_ModelDraw(Scene->Sphere);
    NEA_ModelDraw(Scene->Teapot);
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

    Scene.Sphere = NEA_ModelCreate(NEA_Static);
    Scene.Teapot = NEA_ModelCreate(NEA_Static);
    Scene.Camera = NEA_CameraCreate();

    NEA_Material *Material = NEA_MaterialCreate();

    NEA_CameraSet(Scene.Camera,
                 0, 1, -4,
                 0, 0, 0,
                 0, 1, 0);

    NEA_ModelLoadStaticMesh(Scene.Sphere, sphere_bin);
    NEA_ModelLoadStaticMesh(Scene.Teapot, teapot_bin);

    NEA_MaterialTexLoad(Material, NEA_A1RGB5, 64, 64, NEA_TEXGEN_TEXCOORD,
                       surfaceBitmap);

    NEA_ModelSetMaterial(Scene.Sphere, Material);
    NEA_ModelSetMaterial(Scene.Teapot, Material);

    // Toon shading reads only the red channel of the lit vertex colour, so the
    // material wants to be grey: it decides how lit a surface is, and the table
    // decides what that looks like. Specular is off for the same reason.
    NEA_MaterialSetProperties(Material,
                             RGB15(24, 24, 24), // diffuse
                             RGB15(8, 8, 8),    // ambient
                             RGB15(0, 0, 0),    // specular
                             RGB15(0, 0, 0),    // emission
                             false, false);

    NEA_LightSet(0, NEA_White, -0.5, -0.5, -0.6);

    NEA_ModelSetCoord(Scene.Sphere, -1.3, 0, 0);
    NEA_ModelSetCoord(Scene.Teapot, 1.1, -0.4, 0);

    int ramp = RAMP_TWO_BAND;
    bool highlight = false;
    bool spin = true;
    int angle = 0;
    int phase = 0;

    ApplyRamp(ramp, phase);

    while (1)
    {
        NEA_WaitForVBL(0);

        scanKeys();
        uint32_t keys = keysDown();
        uint32_t held = keysHeld();

        if (keys & KEY_A)
        {
            ramp = (ramp + 1) % RAMP_COUNT;
            ApplyRamp(ramp, phase);
        }
        if (keys & KEY_B)
        {
            // Toon and highlight are a per-frame choice, not a per-polygon one.
            // Highlight adds the table colour on top instead of just replacing
            // the vertex colour, which brightens and can shift the hue.
            highlight = !highlight;
            NEA_ToonHighlightEnable(highlight);
        }
        if (keys & KEY_START)
            spin = !spin;

        if (spin)
            angle = (angle + 1) & 511;
        if (held & KEY_LEFT)
            angle = (angle - 3) & 511;
        if (held & KEY_RIGHT)
            angle = (angle + 3) & 511;

        NEA_ModelSetRot(Scene.Sphere, 0, angle, 0);
        NEA_ModelSetRot(Scene.Teapot, 0, angle, 0);

        phase++;
        if (ramp == RAMP_ANIMATED)
            ApplyRamp(ramp, phase);

        printf("\x1b[0;0H"
               "Toon table ramps\n"
               "================\n\n"
               "A      Ramp: %s\n"
               "B      Mode: %s\n"
               "Start  Spin: %s\n"
               "Left/Right  Rotate\n\n"
               "%s\n",
               RAMP_NAMES[ramp],
               highlight ? "highlight" : "toon     ",
               spin ? "on " : "off",
               RAMP_NOTES[ramp]);

        NEA_ProcessArg(Draw3DScene, &Scene);
    }

    return 0;
}
