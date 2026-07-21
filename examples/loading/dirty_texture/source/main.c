// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Nitro Engine Advanced, 2024
//
// This file is part of Nitro Engine Advanced

// Dirty texture example.
//
// Keep a texture in a CPU-side buffer, modify its texels freely, and push the
// changes to VRAM on demand ("dirty texture"). This is handy for procedural or
// hand-drawn textures that change at runtime.
//
//   NEA_MaterialTexBlank()      : allocate a blank RAM-backed texture (+ VRAM).
//   NEA_MaterialTexGetData()    : get a writable pointer to the CPU buffer.
//   NEA_MaterialTexSetDirty()   : mark the buffer as changed.
//   NEA_MaterialTexVramUpdate() : push the buffer into VRAM (in place) if dirty.
//
// On boot it runs automated self-checks (results on the bottom screen) and then
// animates a plasma-like pattern generated entirely on the CPU each frame.

#include <NEAMain.h>

// 64x64 direct-color texture (NEA_A1RGB5, 16 bpp).
#define TEX_W       64
#define TEX_H       64
#define TEX_BYTES   (TEX_W * TEX_H * 2)

typedef struct {
    NEA_Material *Mat;
    int frame;
} SceneData;

void Draw3DScene(void *arg)
{
    SceneData *Scene = arg;

    NEA_2DViewInit();

    NEA_2DDrawTexturedQuad(96, 32,
                          96 + TEX_W, 32 + TEX_H,
                          0, Scene->Mat);
}

// Fill the CPU buffer with a moving pattern. Each texel is A1RGB5: alpha bit +
// 5 bits per channel.
static void FillPattern(u16 *buf, int frame)
{
    for (int y = 0; y < TEX_H; y++)
    {
        for (int x = 0; x < TEX_W; x++)
        {
            int r = (x + frame) & 31;
            int g = (y + frame) & 31;
            int b = (x + y - frame) & 31;
            buf[y * TEX_W + x] = (1 << 15) | (b << 10) | (g << 5) | r;
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
    // libnds uses VRAM_C for the text console, so reserve only banks A and B for
    // textures.
    NEA_TextureSystemReset(0, 0, NEA_VRAM_AB);
    consoleDemoInit();

    // ---- Create a blank RAM-backed texture ---------------------------------

    Scene.Mat = NEA_MaterialCreate();

    int baseline = NEA_TextureFreeMem();
    NEA_MaterialTexBlank(Scene.Mat, NEA_A1RGB5, TEX_W, TEX_H,
                        NEA_TEXGEN_TEXCOORD);
    int afterBlank = NEA_TextureFreeMem();

    // ---- Automated self-check ----------------------------------------------

    printf("Dirty texture\n");
    printf("=============\n\n");

    // Blank allocates the CPU buffer and one VRAM slot of exactly TEX_BYTES.
    bool t1 = (baseline - afterBlank == TEX_BYTES)
              && (Scene.Mat->texindex != NEA_NO_TEXTURE)
              && (NEA_MaterialTexGetData(Scene.Mat) != NULL)
              && (Scene.Mat->dirty == false);
    printf("[%s] blank alloc uses %d B\n", t1 ? "PASS" : "FAIL",
           baseline - afterBlank);

    // Editing + update pushes in place: VRAM usage must not change, dirty clears.
    u16 *buf = NEA_MaterialTexGetData(Scene.Mat);
    FillPattern(buf, 0);
    NEA_MaterialTexSetDirty(Scene.Mat);
    bool t2dirty = (Scene.Mat->dirty == true);
    NEA_MaterialTexVramUpdate(Scene.Mat);
    bool t2 = t2dirty
              && (Scene.Mat->dirty == false)
              && (NEA_TextureFreeMem() == afterBlank)
              && (Scene.Mat->texindex != NEA_NO_TEXTURE);
    printf("[%s] update pushes in place\n", t2 ? "PASS" : "FAIL");

    // A non-dirty update is a no-op success.
    bool t3 = (NEA_MaterialTexVramUpdate(Scene.Mat) == 1)
              && (NEA_TextureFreeMem() == afterBlank);
    printf("[%s] update skips when clean\n\n", t3 ? "PASS" : "FAIL");

    bool all = t1 && t2 && t3;
    printf("%s\n\n", all ? "ALL TESTS PASSED" : "SOME TESTS FAILED");

    printf("Animating CPU-drawn texture...\n");

    while (1)
    {
        // Redraw the CPU buffer. This only touches main RAM, so it is
        // timing-independent and must run BEFORE the vertical blank: the heavy
        // work would otherwise overrun the short vblank window and push the VRAM
        // upload into active display, where remapping texture VRAM to LCD races
        // the GPU and makes the texture render black.
        u16 *data = NEA_MaterialTexGetData(Scene.Mat);
        FillPattern(data, Scene.frame);
        NEA_MaterialTexSetDirty(Scene.Mat);
        Scene.frame++;

        NEA_WaitForVBL(0);

        // Push the RAM buffer into VRAM now, during the vertical blank, where it
        // is safe to remap texture VRAM. Keep this the only VRAM work here.
        NEA_MaterialTexVramUpdate(Scene.Mat);

        NEA_ProcessArg(Draw3DScene, &Scene);
    }

    return 0;
}
