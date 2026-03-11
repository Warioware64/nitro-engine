// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Warioware64, 2026
//
// This file is part of Nitro Engine Advanced
//
// Hardware 2D bitmap background example
// --------------------------------------
// Demonstrates a 16bpp direct-color bitmap background behind a 3D scene.
// Uses NEA_Hw2DBGPutPixel16() for drawing and NEA_Hw2DBGClearBitmap() for
// clearing. Bitmap BGs require layer 2 or 3 (MODE_5_3D).

#include <NEAMain.h>

typedef struct {
    NEA_Camera *camera;
} SceneData;

void Draw3DScene(void *arg)
{
    SceneData *scene = arg;
    NEA_CameraUse(scene->camera);

    NEA_PolyFormat(31, 0, NEA_LIGHT_0, NEA_CULL_NONE, 0);

    GFX_BEGIN = GL_TRIANGLES;

    GFX_COLOR = RGB15(31, 0, 0);
    glVertex3v16(floattov16(-1.0), floattov16(-1.0), 0);

    GFX_COLOR = RGB15(0, 31, 0);
    glVertex3v16(floattov16(1.0), floattov16(-1.0), 0);

    GFX_COLOR = RGB15(0, 0, 31);
    glVertex3v16(floattov16(0.0), floattov16(1.0), 0);

    GFX_END = 0;
}

// Draw a horizontal gradient bar
static void draw_gradient(NEA_Hw2DBG *bg, int y_start, int height,
                          int r0, int g0, int b0,
                          int r1, int g1, int b1)
{
    for (int y = y_start; y < y_start + height && y < 192; y++)
    {
        for (int x = 0; x < 256; x++)
        {
            int r = r0 + (r1 - r0) * x / 255;
            int g = g0 + (g1 - g0) * x / 255;
            int b = b0 + (b1 - b0) * x / 255;
            NEA_Hw2DBGPutPixel16(bg, x, y, RGB15(r, g, b) | BIT(15));
        }
    }
}

// Draw a filled rectangle
static void draw_rect(NEA_Hw2DBG *bg, int x0, int y0, int w, int h, u16 color)
{
    for (int y = y0; y < y0 + h && y < 192; y++)
        for (int x = x0; x < x0 + w && x < 256; x++)
            NEA_Hw2DBGPutPixel16(bg, x, y, color | BIT(15));
}

int main(int argc, char *argv[])
{
    (void)argc;
    (void)argv;

    SceneData scene = { 0 };

    irqEnable(IRQ_HBLANK);
    irqSet(IRQ_VBLANK, NEA_VBLFunc);
    irqSet(IRQ_HBLANK, NEA_HBLFunc);

    NEA_SetTexPaletteBank(NEA_VRAM_F);
    NEA_Init3D();

    NEA_Hw2DVRAMConfig cfg = {
        .main_bg  = NEA_VRAM_E,
        .main_obj = 0,
        .sub_bg   = 0,
        .sub_obj  = 0,
    };
    NEA_Hw2DInit(&cfg);

    scene.camera = NEA_CameraCreate();
    NEA_CameraSet(scene.camera,
                  0, 0, 3,
                  0, 0, 0,
                  0, 1, 0);

    NEA_LightSet(0, NEA_White, 0, -1, -1);

    // 3D clear color with alpha=0: transparent, shows bitmap BG behind
    NEA_ClearColorSet(NEA_Black, 0, 63);

    // 16bpp bitmap BG on layer 2 (required for bitmap mode)
    NEA_Hw2DBG *bg = NEA_Hw2DBGCreate(NEA_ENGINE_MAIN, 2,
                                       NEA_HW2D_BG_BITMAP_16, 256, 256);

    // BG behind 3D (priority 3 = lowest)
    NEA_Hw2DBGSetPriority(bg, 3);

    // Draw initial pattern: gradient bars
    draw_gradient(bg,   0, 48,  0,  0, 10,   0,  0, 31);  // dark to blue
    draw_gradient(bg,  48, 48,  0, 10,  0,   0, 31,  0);  // dark to green
    draw_gradient(bg,  96, 48, 10,  0,  0,  31,  0,  0);  // dark to red
    draw_gradient(bg, 144, 48, 10, 10,  0,  31, 31,  0);  // dark to yellow

    // Colored rectangles
    draw_rect(bg, 200, 10, 40, 40, RGB15(31, 16, 0));
    draw_rect(bg, 210, 20, 40, 40, RGB15(0, 31, 16));

    // Console on sub screen
    consoleDemoInit();
    printf("NEA Hw2D Bitmap BG Demo\n");
    printf("-----------------------\n");
    printf("A: redraw gradient\n");
    printf("B: clear to black\n");
    printf("D-pad: draw colored dot\n");

    int cx = 128, cy = 96;

    while (1)
    {
        NEA_WaitForVBL(NEA_UPDATE_HW2D);

        scanKeys();
        uint32_t keys = keysHeld();
        uint32_t down = keysDown();

        if (keys & KEY_RIGHT) cx += 2;
        if (keys & KEY_LEFT)  cx -= 2;
        if (keys & KEY_DOWN)  cy += 2;
        if (keys & KEY_UP)    cy -= 2;

        // Clamp
        if (cx < 0) cx = 0;
        if (cx > 255) cx = 255;
        if (cy < 0) cy = 0;
        if (cy > 191) cy = 191;

        // Draw cursor dot
        if (keys & (KEY_RIGHT | KEY_LEFT | KEY_DOWN | KEY_UP))
            draw_rect(bg, cx - 2, cy - 2, 5, 5, RGB15(31, 31, 31));

        if (down & KEY_A)
        {
            draw_gradient(bg,   0, 48,  0,  0, 10,   0,  0, 31);
            draw_gradient(bg,  48, 48,  0, 10,  0,   0, 31,  0);
            draw_gradient(bg,  96, 48, 10,  0,  0,  31,  0,  0);
            draw_gradient(bg, 144, 48, 10, 10,  0,  31, 31,  0);
        }

        if (down & KEY_B)
            NEA_Hw2DBGClearBitmap(bg, 0);

        printf("\x1b[6;0HCursor: %d, %d   \n", cx, cy);

        NEA_ProcessArg(Draw3DScene, &scene);
    }

    return 0;
}
