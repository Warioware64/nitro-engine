// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Nitro Engine Advanced contributors, 2026
//
// This file is part of Nitro Engine Advanced

// A background that occludes geometry, for zero polygons.
//
// The clear bitmap is usually described as "a background image", which
// undersells it. VRAM_C holds the colour, but VRAM_D holds a *depth per pixel*,
// and the depth buffer is cleared to those values. So the background is not
// merely drawn behind the scene: geometry that is further away than a background
// pixel disappears behind it.
//
// That turns a flat image into a piece of scenery. The ridge drawn here has its
// own depth, so the orbiting sphere passes behind it on the far half of its
// circle and in front of it on the near half, and the sky above it never
// occludes anything because it is at maximum depth.
//
// Scrolling the bitmap (NEA_ClearBMPScroll) as the camera turns keeps all of
// that intact, which is the cheapest parallax backdrop the hardware can do.
//
// NEA_ClearDepthSet() is the flat version of the same idea, for when no bitmap
// is involved: it decides how far away "nothing" is. Press Y to pull it in and
// watch it swallow the far half of the orbit.
//
// Cost: two VRAM banks and no polygons. Filling the banks happens once at
// startup.

#include <NEAMain.h>

#include "sphere_bin.h"

#define BMP_W 256
#define BMP_H 256

// The ridge sits this many world units from the camera.
#define RIDGE_UNITS 9

typedef struct {
    NEA_Camera *Camera;
    NEA_Model *Sphere;
} SceneData;

// Height of the ridge silhouette at a given column, in pixels from the top.
static int RidgeHeight(int x)
{
    // A couple of sines at different rates, which is enough to read as hills.
    int a = sinLerp((x * 191) & 0xFFFF) >> 8;
    int b = sinLerp((x * 517 + 12000) & 0xFFFF) >> 9;
    int c = sinLerp((x * 1303 + 40000) & 0xFFFF) >> 10;

    return 118 + a + b + c;
}

// Fills VRAM_C with the colours and VRAM_D with the matching per-pixel depths.
// Both banks have to be in LCD mode for the CPU to reach them.
static void BuildBackground(u32 ridge_depth)
{
    vramSetBankC(VRAM_C_LCD);
    vramSetBankD(VRAM_D_LCD);

    u16 *color = (u16 *)VRAM_C;
    u16 *depth = (u16 *)VRAM_D;

    for (int x = 0; x < BMP_W; x++)
    {
        int horizon = RidgeHeight(x);

        for (int y = 0; y < BMP_H; y++)
        {
            u16 c;
            u16 d;

            if (y < horizon)
            {
                // Sky: a vertical gradient that pales towards the horizon, and
                // as far away as the hardware can express so that nothing is
                // ever hidden behind it.
                int t = (y * 31) / horizon;
                c = RGB15(6 + (t >> 1), 14 + (t >> 2), 31 - (t >> 4)) | BIT(15);
                d = 0x7FFF;
            }
            else
            {
                // The ridge. Dark, because the whole point is the silhouette,
                // with a slight lift down the slope so it isn't a flat cutout.
                int t = ((y - horizon) * 31) / (BMP_H - horizon);
                c = RGB15(2 + (t >> 3), 6 + (t >> 3), 4 + (t >> 4)) | BIT(15);
                d = ridge_depth;
            }

            // VRAM_C is ABBBBBGGGGGRRRRR, VRAM_D is FDDDDDDDDDDDDDDD where F
            // enables fog for that pixel. Fog is off here, so bit 15 stays 0.
            color[y * BMP_W + x] = c;
            depth[y * BMP_W + x] = d & 0x7FFF;
        }
    }

    // Hands the two banks back to the GPU as the clear bitmap.
    NEA_ClearBMPEnable(true);
}

void Draw3DScene(void *arg)
{
    SceneData *Scene = arg;

    NEA_CameraUse(Scene->Camera);

    NEA_PolyFormat(31, 0, NEA_LIGHT_0, NEA_CULL_BACK, 0);

    NEA_ModelDraw(Scene->Sphere);
}

int main(int argc, char *argv[])
{
    SceneData Scene = { 0 };

    irqEnable(IRQ_HBLANK);
    irqSet(IRQ_VBLANK, NEA_VBLFunc);
    irqSet(IRQ_HBLANK, NEA_HBLFunc);

    // Single 3D mode only: the clear bitmap needs VRAM_C and VRAM_D, which dual
    // 3D spends on the display capture.
    NEA_Init3D();
    NEA_TextureSystemReset(0, 0, NEA_VRAM_AB);

    // The usual consoles are no good here. consoleDemoInit() wants VRAM_C and
    // NEA_InitConsole() would draw over the 3D on the main screen, and the whole
    // example is about what the main screen looks like. VRAM_H is free, so put
    // the text on the sub screen.
    vramSetBankH(VRAM_H_SUB_BG);
    videoSetModeSub(MODE_0_2D);
    consoleInit(NULL, 0, BgType_Text4bpp, BgSize_T_256x256, 0, 1, false, true);

    Scene.Camera = NEA_CameraCreate();
    Scene.Sphere = NEA_ModelCreate(NEA_Static);

    NEA_ModelLoadStaticMesh(Scene.Sphere, sphere_bin);

    NEA_LightSet(0, NEA_White, -0.4, -0.5, -0.7);

    // Depth values are raw depth-buffer numbers, not world units, and what they
    // mean depends on the depth buffer mode. This does that conversion for the
    // mode that is actually active.
    u32 ridge_depth = NEA_FogDepthFromDistance(inttof32(RIDGE_UNITS));

    BuildBackground(ridge_depth);

    // Colour is unused while the clear bitmap is on, but the alpha and the
    // polygon ID still come from here.
    NEA_ClearColorSet(0, 31, 63);

    int orbit = 0;
    int scroll = 0;
    bool use_bitmap = true;
    bool spin = true;
    int flat_units = 40;

    while (1)
    {
        NEA_WaitForVBL(0);

        scanKeys();
        uint32_t keys = keysDown();
        uint32_t held = keysHeld();

        if (keys & KEY_A)
        {
            // Turning the bitmap off falls back to the flat rear plane: one
            // colour, one depth, no occlusion by scenery.
            use_bitmap = !use_bitmap;
            NEA_ClearBMPEnable(use_bitmap);
            if (!use_bitmap)
                NEA_ClearColorSet(RGB15(8, 14, 28), 31, 63);
        }
        if (keys & KEY_START)
            spin = !spin;

        if (!use_bitmap && (held & (KEY_UP | KEY_DOWN)))
        {
            // The flat rear plane pulled in until it starts eating the orbit.
            flat_units += (held & KEY_UP) ? 1 : -1;
            if (flat_units < 1)
                flat_units = 1;
            if (flat_units > 40)
                flat_units = 40;

            NEA_ClearDepthSet(NEA_FogDepthFromDistance(inttof32(flat_units)));
        }

        if (spin)
            orbit = (orbit + 2) & 511;
        if (held & KEY_LEFT)
            orbit = (orbit - 4) & 511;
        if (held & KEY_RIGHT)
            orbit = (orbit + 4) & 511;

        // The sphere circles the origin, so it crosses the ridge depth twice per
        // lap: behind it on the far side, in front of it on the near side.
        NEA_ModelSetCoordI(Scene.Sphere,
                          5 * sinLerp(orbit << 6),
                          0,
                          5 * cosLerp(orbit << 6));

        NEA_CameraSet(Scene.Camera,
                     0, 1.5, -9,
                     0, 0, 0,
                     0, 1, 0);

        // A backdrop that doesn't move at all reads as a painted wall, so give
        // it a slow drift. The depth scrolls with the colour, so the occlusion
        // follows the ridge.
        if (held & KEY_L)
            scroll = (scroll - 1) & 255;
        if (held & KEY_R)
            scroll = (scroll + 1) & 255;

        NEA_ClearBMPScroll(scroll, 0);

        printf("\x1b[0;0H"
               "Rear plane depth\n"
               "================\n\n"
               "A  Background: %s\n"
               "Start  Orbit: %s\n"
               "Left/Right  Move sphere\n"
               "L/R  Scroll backdrop\n"
               "%s\n\n"
               "The ridge sits %d units out\n"
               "(depth 0x%04X). The sphere \n"
               "orbits through that depth, \n"
               "so it goes behind the hills\n"
               "and comes back in front.   \n",
               use_bitmap ? "bitmap + depth" : "flat rear plane",
               spin ? "on " : "off",
               use_bitmap ? "                       "
                          : "Up/Down  Rear plane depth",
               RIDGE_UNITS, (unsigned)ridge_depth);

        NEA_ProcessArg(Draw3DScene, &Scene);
    }

    return 0;
}
