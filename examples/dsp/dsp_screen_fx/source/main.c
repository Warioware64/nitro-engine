// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Nitro Engine Advanced contributors, 2026
//
// This file is part of Nitro Engine Advanced
//
// Full-screen effects on the DSi's DSP.
//
// The 3D image is captured, blurred, bloomed and colour graded by the DSP, and
// shown as a bitmap background. An image takes two frames when the effects fit
// in one (30 fps); heavier combinations drop to 20 fps.
//
// L/R: grade. X: blur passes (0-2). Y: bloom on/off. START: exit.
//
// Without a DSP (or in melonDS) the ARM9 does the same work, much slower.

#include <stdio.h>

#include <NEAMain.h>

#include "teapot_bin.h"
#include "teapot.h"

#define NUM_MODELS 3

typedef struct {
    NEA_Camera *Camera;
    NEA_Model *Model[NUM_MODELS];
    NEA_Material *Material;
} SceneData;

static PrintConsole con;

static void Draw3DScene(void *arg)
{
    SceneData *Scene = arg;

    NEA_CameraUse(Scene->Camera);
    NEA_PolyFormat(31, 0, NEA_LIGHT_0, NEA_CULL_BACK, 0);

    for (int i = 0; i < NUM_MODELS; i++)
        NEA_ModelDraw(Scene->Model[i]);
}

static void init_scene(SceneData *Scene)
{
    for (int i = 0; i < NUM_MODELS; i++)
        Scene->Model[i] = NEA_ModelCreate(NEA_Static);

    Scene->Camera = NEA_CameraCreate();
    Scene->Material = NEA_MaterialCreate();

    NEA_CameraSet(Scene->Camera, 0, 0, -6, 0, 0, 0, 0, 1, 0);

    NEA_ModelLoadStaticMesh(Scene->Model[0], teapot_bin);
    NEA_MaterialTexLoad(Scene->Material, NEA_A1RGB5, 256, 256,
                        NEA_TEXGEN_TEXCOORD | NEA_TEXTURE_WRAP_S
                        | NEA_TEXTURE_WRAP_T,
                        teapotBitmap);
    NEA_ModelSetMaterial(Scene->Model[0], Scene->Material);

    for (int i = 1; i < NUM_MODELS; i++)
        NEA_ModelClone(Scene->Model[i], Scene->Model[0]);

    for (int i = 0; i < NUM_MODELS; i++)
        NEA_ModelSetCoord(Scene->Model[i], (i - 1) * 2.5, 0, 0);

    NEA_LightSet(0, NEA_White, -0.5, -0.5, -0.5);

    // Opaque, so that every captured pixel is shown
    NEA_ClearColorSet(RGB15(4, 5, 10), 31, 63);
}

typedef enum {
    GRADE_NONE,
    GRADE_SEPIA,
    GRADE_GREY,
    GRADE_NIGHT,
    GRADE_VIVID,
    GRADE_COUNT
} grade_id;

static const char *grade_names[GRADE_COUNT] = {
    "none   ", "sepia  ", "grey   ", "night  ", "vivid  ",
};

static void set_grade(NEA_DspGradeTables *t, grade_id id)
{
    NEA_DspGrade g;
    NEA_DspGradeInit(&g);

    switch (id)
    {
        case GRADE_SEPIA:
            NEA_DspGradeSepia(&g, 256);
            break;
        case GRADE_GREY:
            NEA_DspGradeSaturate(&g, 0);
            break;
        case GRADE_NIGHT:
            NEA_DspGradeSaturate(&g, 128);
            NEA_DspGradeTint(&g, 150, 180, 256, 0);
            NEA_DspGradeContrast(&g, 300, -3);
            break;
        case GRADE_VIVID:
            NEA_DspGradeSaturate(&g, 400);
            break;
        default:
            break;
    }

    NEA_DspGradeBuild(t, &g);
    NEA_DspScreenFXSetGrade(id == GRADE_NONE ? NULL : t);
}

int main(int argc, char *argv[])
{
    SceneData Scene = { 0 };

    irqEnable(IRQ_HBLANK);
    irqSet(IRQ_VBLANK, NEA_VBLFunc);
    irqSet(IRQ_HBLANK, NEA_HBLFunc);

    NEA_Init3D();

    videoSetModeSub(MODE_0_2D);
    vramSetBankH(VRAM_H_SUB_BG);
    consoleInit(&con, 0, BgType_Text4bpp, BgSize_T_256x256, 15, 0, false,
                true);

    bool dsp = NEA_DspInit()
               && NEA_DspGetTransport() == NEA_DSP_TRANSPORT_DMA;

    // Banks C and D hold the captured and the shown frame: set up before the
    // texture system, which then keeps A and B
    if (!NEA_DspScreenFXInit(NEA_VRAM_CD))
    {
        printf("NEA_DspScreenFXInit() failed\n");
        while (1)
            swiWaitForVBlank();
    }
    NEA_TextureSystemReset(0, 0, NEA_VRAM_AB);

    init_scene(&Scene);

    NEA_DspGradeTables *tables = NEA_DspGradeTablesCreate();
    grade_id grade = GRADE_SEPIA;
    set_grade(tables, grade);

    int blur = 0;
    bool bloom = false;

    while (1)
    {
        NEA_WaitForVBL(NEA_UPDATE_DSPFX);

        scanKeys();
        u16 down = keysDown();
        if (down & KEY_START)
            break;
        if (down & (KEY_L | KEY_R))
        {
            grade = (grade + ((down & KEY_R) ? 1 : GRADE_COUNT - 1))
                    % GRADE_COUNT;
            set_grade(tables, grade);
        }
        if (down & KEY_X)
        {
            blur = (blur + 1) % 3;
            NEA_DspScreenFXSetBlur(blur);
        }
        if (down & KEY_Y)
        {
            bloom = !bloom;
            NEA_DspScreenFXSetBloom(bloom ? 16 : -1);
        }

        for (int i = 0; i < NUM_MODELS; i++)
            NEA_ModelRotate(Scene.Model[i], 0, 2 + i, 1);

        NEA_ProcessArg(Draw3DScene, &Scene);

        // Move the effects on as soon as the scene is out
        NEA_DspScreenFXPoll();

        printf("\x1b[0;0H"
               "DSP full-screen effects\n"
               "\n"
               "Runs on: %s\n"
               "Images: %2d per second  \n"
               "Failures: %lu  restarts: %lu\n"
               "\n"
               "L/R: grade %s\n"
               "X: blur passes %d\n"
               "Y: bloom %s\n"
               "START: exit\n",
               dsp ? "DSP " : "ARM9", NEA_DspScreenFXGetRate(),
               NEA_DspGradeGetFailureCount() + NEA_DspBlurGetFailureCount()
               + NEA_DspBloomGetFailureCount(),
               NEA_DspGetResetCount(), grade_names[grade], blur,
               bloom ? "on " : "off");
    }

    NEA_DspScreenFXEnd();
    NEA_DspEnd();
    NEA_End();

    return 0;
}
