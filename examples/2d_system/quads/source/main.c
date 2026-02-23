// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Antonio Niño Díaz, 2008-2024
//
// This file is part of Nitro Engine Advanced

#include <NEAMain.h>

#define NUM_QUADS 40

typedef struct {
   bool enabled;
   int x1, y1,  x2, y2;
   int alpha, id;
   int color;
} quad_t;

quad_t Quad[NUM_QUADS];

bool enable_alpha = false;

void UpdateQuads(void)
{
    for (int i = 0; i < NUM_QUADS; i++)
    {
        if (!Quad[i].enabled)
        {
            // Always recreate quads if not enabled
            Quad[i].enabled = true;
            Quad[i].x1 = rand() & 255;
            Quad[i].x2 = rand() & 255;
            Quad[i].y1 = rand() % 192;
            Quad[i].y2 = rand() % 192;
            Quad[i].alpha = (rand() % 30) + 1;
            Quad[i].id = rand() & 63;
            Quad[i].color = rand() & 0xFFFF;
        }
        else
        {
            // Disable quads randomly
            if ((rand() & 31) == 31)
                Quad[i].enabled = false;
        }
    }
}

void Draw3DScene(void)
{
    NEA_2DViewInit();

    NEA_PolyFormat(31, 0,0,NEA_CULL_BACK,0);

    for (int i = 0; i < NUM_QUADS; i++)
    {
        if (!Quad[i].enabled)
            continue;

        if (enable_alpha)
            NEA_PolyFormat(Quad[i].alpha, Quad[i].id, 0, NEA_CULL_NONE, 0);

        NEA_2DDrawQuad(Quad[i].x1, Quad[i].y1, Quad[i].x2, Quad[i].y2, i,
                      Quad[i].color);
    }
}

int main(int argc, char *argv[])
{
    irqEnable(IRQ_HBLANK);
    irqSet(IRQ_VBLANK, NEA_VBLFunc);
    irqSet(IRQ_HBLANK, NEA_HBLFunc);

    NEA_Init3D();

    // libnds uses VRAM_C for the text console, reserve A and B only
    NEA_TextureSystemReset(0, 0, NEA_VRAM_AB);
    // Init console in non-3D screen
    consoleDemoInit();

    printf("A: Alpha");

    while (1)
    {
        NEA_WaitForVBL(0);

        scanKeys();
        uint16_t kheld = keysHeld();
        if (kheld & KEY_A)
            enable_alpha = true;
        else
            enable_alpha = false;

        UpdateQuads();

        NEA_Process(Draw3DScene);
    }

    return 0;
}
