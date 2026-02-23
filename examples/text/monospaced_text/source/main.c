// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Antonio Niño Díaz, 2008-2024
//
// This file is part of Nitro Engine Advanced

#include <NEAMain.h>

#include "text.h"
#include "text2.h"

typedef struct {
    int px, py;
} SceneData;

void Draw3DScene(void *arg)
{
    SceneData *Scene = arg;

    NEA_2DViewInit(); // Init 2D view

    // Print text on columns/rows
    NEA_TextPrint(0, // Font slot
                 0, 0, // Coordinates
                 NEA_White, // Color
                 "abcdefghijklmnopqrstuvwxyz012345\n6789\n"
                 "\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n"
                 "12345678901234567890123456789012");
    NEA_TextPrint(1, 1, 4, NEA_Red, "AntonioND\n            rules!!!");
    NEA_TextPrint(1, 8, 8, NEA_Yellow, "Yeah!!!");

    // Print text on any coordinates
    NEA_TextPrintFree(1, Scene->px, Scene->py, NEA_Blue, "Test");
    NEA_TextPrintBoxFree(1,
                        Scene->px + 32, Scene->py + 32,
                        Scene->px + 96, Scene->py + 64,
                        NEA_Green, -1, "This is a\ntest!!");

    // Print formated text
    char text[64];
    sprintf(text, "Polys: %d\nVertices: %d\nCPU: %d%%",
            NEA_GetPolygonCount(), NEA_GetVertexCount(), NEA_GetCPUPercent());
    NEA_TextPrint(0, 0, 13, NEA_White, text);
}

int main(int argc, char *argv[])
{
    SceneData Scene = { 0 };

    irqEnable(IRQ_HBLANK);
    irqSet(IRQ_VBLANK, NEA_VBLFunc);
    irqSet(IRQ_HBLANK, NEA_HBLFunc);

    NEA_Init3D();
    // Move 3D screen to lower screen
    NEA_SwapScreens();

    NEA_Material *Text = NEA_MaterialCreate();
    NEA_Material *Text2 = NEA_MaterialCreate();
    NEA_MaterialTexLoad(Text, NEA_A1RGB5, 256, 64, NEA_TEXGEN_TEXCOORD,
                       textBitmap);
    NEA_MaterialTexLoad(Text2, NEA_A1RGB5, 512, 128, NEA_TEXGEN_TEXCOORD,
                       text2Bitmap);

    NEA_TextInit(0,     // Font slot
                Text,  // Image
                8, 8); // Size of one character (x, y)
    NEA_TextInit(1, Text2, 12, 16);

    while (1)
    {
        NEA_WaitForVBL(0);

        NEA_ProcessArg(Draw3DScene, &Scene);

        // Update stylus coordinates when screen is pressed
        scanKeys();

        if (keysHeld() & KEY_TOUCH)
        {
            touchPosition touch;
            touchRead(&touch);
            Scene.px = touch.px;
            Scene.py = touch.py;
        }

    }

    return 0;
}
