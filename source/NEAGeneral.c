// SPDX-License-Identifier: MIT
//
// Copyright (c) 2008-2023 Antonio Niño Díaz
//
// This file is part of Nitro Engine Advanced

#include <nds/arm9/background.h>
#include <nds/arm9/postest.h>
#include <nds/dma.h>

#include "NEAMain.h"
#include "NEAMath.h"

/// @file NEAGeneral.c

const char NEA_VersionString[] =
    "Nitro Engine Advanced - Version " NITRO_ENGINE_ADVANCED_VERSION_STRING " - "
    "(C) 2008-2011, 2019, 2022-2023 Antonio Nino Diaz";

static bool NEA_UsingConsole;
bool NEA_TestTouch;
static int NEA_screenratio;
static uint32_t NEA_viewport;
static u8 NEA_Screen; // 1 = main screen, 0 = sub screen

static NEA_ExecutionModes ne_execution_mode = NEA_ModeUninitialized;

NEA_Input ne_input;

static int ne_znear, ne_zfar;
static int fov;

static int ne_main_screen = 1; // 1 = top, 0 = bottom

static int ne_depth_buffer_mode = 0; // GL_ZBUFFERING by default

static uint32_t ne_dma_enabled = 0;
static uint32_t ne_dma_src = 0;
static uint32_t ne_dma_dst = 0;
static uint32_t ne_dma_cr = 0;

// Two-pass rendering state
static u16 *ne_two_pass_fb[2];              // Two framebuffers in main RAM
static volatile u8 ne_two_pass_displayed;   // Index of framebuffer being displayed
static volatile bool ne_two_pass_next_ready;// Next framebuffer ready to swap
static u8 ne_two_pass_frame;                // Current pass: 0 = left, 1 = right
static bool ne_two_pass_enabled;            // True when in two-pass mode

#define NEA_TWO_PASS_SPLIT 128 // Horizontal split at screen center

// Scanline row offset in the VRAM_F bitmap used for HBL DMA display.
// Same technique as NEA_DUAL_DMA_3D_LINES_OFFSET: the bitmap BG has PD=0 so it
// repeats one row, and DMA writes new scanline data to that row at each HBL.
#define NEA_TWO_PASS_LINE_OFFSET 20

NEA_ExecutionModes NEA_CurrentExecutionMode(void)
{
    return ne_execution_mode;
}

void NEA_End(void)
{
    if (ne_execution_mode == NEA_ModeUninitialized)
        return;

    vramSetBankA(VRAM_A_LCD);
    vramSetBankB(VRAM_B_LCD);

    switch (ne_execution_mode)
    {
        case NEA_ModeSingle3D:
        {
            videoSetMode(0);

            if (GFX_CONTROL & GL_CLEAR_BMP)
                NEA_ClearBMPEnable(false);

            if (NEA_UsingConsole)
                vramSetBankF(VRAM_F_LCD);

            break;
        }

        case NEA_ModeDual3D:
        {
            videoSetMode(0);
            videoSetModeSub(0);

            vramSetBankC(VRAM_C_LCD);
            vramSetBankD(VRAM_D_LCD);

            if (NEA_UsingConsole)
                vramSetBankF(VRAM_F_LCD);

            break;
        }

        case NEA_ModeDual3D_FB:
        {
            videoSetMode(0);
            videoSetModeSub(0);

            vramSetBankC(VRAM_C_LCD);
            vramSetBankD(VRAM_D_LCD);
            break;
        }

        case NEA_ModeDual3D_DMA:
        {
            ne_dma_enabled = 0;

#ifdef NEA_BLOCKSDS
            dmaStopSafe(2);
#else
            DMA_CR(2) = 0;
#endif

            videoSetMode(0);
            videoSetModeSub(0);

            vramSetBankC(VRAM_C_LCD);
            vramSetBankD(VRAM_D_LCD);

            // A pseudo framebuffer and the debug console go here
            vramSetBankI(VRAM_I_LCD);
            break;
        }

        case NEA_ModeSingle3D_TwoPass:
        case NEA_ModeSingle3D_TwoPass_DMA:
        {
            ne_two_pass_enabled = false;

#ifdef NEA_BLOCKSDS
            dmaStopSafe(2);
#else
            DMA_CR(2) = 0;
#endif

            videoSetMode(0);
            videoSetModeSub(0);

            vramSetBankC(VRAM_C_LCD);
            vramSetBankD(VRAM_D_LCD);

            if (ne_execution_mode == NEA_ModeSingle3D_TwoPass_DMA)
                vramSetBankF(VRAM_F_LCD);

            if (NEA_UsingConsole)
                vramSetBankH(VRAM_H_LCD);

            for (int i = 0; i < 2; i++)
            {
                if (ne_two_pass_fb[i] != NULL)
                {
                    free(ne_two_pass_fb[i]);
                    ne_two_pass_fb[i] = NULL;
                }
            }
            break;
        }

        case NEA_ModeSingle3D_TwoPass_FB:
        {
            ne_two_pass_enabled = false;

            videoSetMode(0);
            videoSetModeSub(0);

            vramSetBankC(VRAM_C_LCD);
            vramSetBankD(VRAM_D_LCD);

            if (NEA_UsingConsole)
                vramSetBankH(VRAM_H_LCD);

            // No main RAM framebuffers to free in FB mode
            break;
        }

        default:
            break;
    }

    NEA_UsingConsole = false;

    vramSetBankE(VRAM_E_LCD); // Palettes

    NEA_GUISystemEnd();
    NEA_SpriteSystemEnd();
    NEA_PhysicsSystemEnd();
    NEA_ModelSystemEnd();
    NEA_AnimationSystemEnd();
    NEA_TextResetSystem();
    NEA_TextureSystemEnd();
    NEA_CameraSystemEnd();
    NEA_SpecialEffectSet(0);

    //Power off 3D hardware
    powerOff(POWER_3D_CORE | POWER_MATRIX);

    NEA_DebugPrint("Nitro Engine Advanced disabled");

    ne_execution_mode = NEA_ModeUninitialized;
}

void NEA_Viewport(int x1, int y1, int x2, int y2)
{
    // Start calculating screen ratio in f32 format
    ne_div_start((x2 - x1 + 1) << 12, (y2 - y1 + 1));

    // Save viewport
    NEA_viewport = x1 | (y1 << 8) | (x2 << 16) | (y2 << 24);
    GFX_VIEWPORT = NEA_viewport;

    MATRIX_CONTROL = GL_PROJECTION; // New projection matix for this viewport
    MATRIX_IDENTITY = 0;

    int fovy = fov * DEGREES_IN_CIRCLE / 360;
    NEA_screenratio = ne_div_result();
    gluPerspectivef32(fovy, NEA_screenratio, ne_znear, ne_zfar);

    MATRIX_CONTROL = GL_MODELVIEW;
}

void NEA_MainScreenSetOnTop(void)
{
    ne_main_screen = 1;
}

void NEA_MainScreenSetOnBottom(void)
{
    ne_main_screen = 0;
}

int NEA_MainScreenIsOnTop(void)
{
    return ne_main_screen;
}

void NEA_SwapScreens(void)
{
    ne_main_screen ^= 1;
}

void NEA_SetFov(int fovValue)
{
    fov = fovValue;
}

static void ne_systems_end_all(void)
{
    NEA_GUISystemEnd();
    NEA_SpriteSystemEnd();
    NEA_PhysicsSystemEnd();
    NEA_ModelSystemEnd();
    NEA_AnimationSystemEnd();
    NEA_TextResetSystem();
    NEA_TextureSystemEnd();
    NEA_CameraSystemEnd();
    NEA_SpecialEffectSet(0);
}

static int ne_systems_reset_all(NEA_VRAMBankFlags vram_banks)
{
    // Default number of objects for everyting.
    if (NEA_CameraSystemReset(0) != 0)
        goto cleanup;
    if (NEA_PhysicsSystemReset(0) != 0)
        goto cleanup;
    if (NEA_SpriteSystemReset(0) != 0)
        goto cleanup;
    if (NEA_GUISystemReset(0) != 0)
        goto cleanup;
    if (NEA_ModelSystemReset(0) != 0)
        goto cleanup;
    if (NEA_AnimationSystemReset(0) != 0)
        goto cleanup;
    if (NEA_TextureSystemReset(0, 0, vram_banks) != 0)
        goto cleanup;

    NEA_TextPriorityReset();

    return 0;

cleanup:
    ne_systems_end_all();
    return -1;
}

static void ne_init_registers(void)
{
    // This function is usually called when the program boots. We don't know
    // which time in the frame exactly. In order to make the behaviour
    // consistent across emulators and hardware, it is required to synchronize
    // to the LCD refresh here.

    swiWaitForVBlank();

    // Power all 3D and 2D. Hide 3D screen during init
    powerOn(POWER_ALL);

    videoSetMode(0);

    vramSetBankE(VRAM_E_TEX_PALETTE);

    // Wait for geometry engine operations to end
    while (GFX_STATUS & BIT(27));

    // Clear the FIFO
    GFX_STATUS |= (1 << 29);

    GFX_FLUSH = 0;
    GFX_FLUSH = 0;

    ne_depth_buffer_mode = 0; // Reset to Z-buffering

    NEA_MainScreenSetOnTop();
    lcdMainOnTop();

    glResetMatrixStack();

    GFX_CONTROL = GL_TEXTURE_2D | GL_ANTIALIAS | GL_BLEND;

    GFX_ALPHA_TEST = 0;

    NEA_ClearColorSet(NEA_Black, 31, 63);
    NEA_FogEnableBackground(false);

    GFX_CLEAR_DEPTH = GL_MAX_DEPTH;

    MATRIX_CONTROL = GL_TEXTURE;
    MATRIX_IDENTITY = 0;

    MATRIX_CONTROL = GL_PROJECTION;
    MATRIX_IDENTITY = 0;

    // Shininess table used for specular lighting
    NEA_ShininessTableGenerate(NEA_SHININESS_CUBIC);

    // setup default material properties
    NEA_MaterialSetDefaultProperties(RGB15(20, 20, 20), RGB15(16, 16, 16),
                                    RGB15(8, 8, 8), RGB15(5, 5, 5),
                                    false, true);

    // Turn off some things...
    for (int i = 0; i < 4; i++)
        NEA_LightOff(i);

    GFX_COLOR = 0;
    GFX_POLY_FORMAT = 0;

    for (int i = 0; i < 8; i++)
        NEA_OutliningSetColor(i, 0);

    ne_znear = floattof32(0.1);
    ne_zfar = floattof32(40.0);
    fov = 70;
    NEA_Viewport(0, 0, 255, 191);

    MATRIX_CONTROL = GL_MODELVIEW;
    MATRIX_IDENTITY = 0;

    // Make sure that this function is left always at the same time regardless
    // of whether it runs on hardware or emulators (which can be more or less
    // accurate). If not, the output of the screens in dual 3D mode may be
    // switched initially (and change once there is a framerate drop when
    // loading assets, for example).

    swiWaitForVBlank();

    // Ready!!

    videoSetMode(MODE_0_3D);
}

void NEA_UpdateInput(void)
{
    ne_input.kdown = keysDown();
    ne_input.kheld = keysHeld();
    ne_input.kup = keysUp();

    if (ne_input.kheld & KEY_TOUCH)
        touchRead(&ne_input.touch);
}

void NEA_SetDepthBufferMode(NEA_DepthBufferMode mode)
{
    ne_depth_buffer_mode = mode;
}

int NEA_Init3D(void)
{
    NEA_End();

    if (ne_systems_reset_all(NEA_VRAM_ABCD) != 0)
        return -1;

    NEA_DisplayListSetDefaultFunction(NEA_DL_DMA_GFX_FIFO);

    NEA_UpdateInput();

    ne_init_registers();

    ne_execution_mode = NEA_ModeSingle3D;

    NEA_DebugPrint("Nitro Engine Advanced initialized in normal 3D mode");

    return 0;
}

// Setup 2D sprites used for Dual 3D mode
static int ne_setup_sprites(void)
{
    SpriteEntry *Sprites = calloc(128, sizeof(SpriteEntry));
    if (Sprites == NULL)
    {
        NEA_DebugPrint("Not enough memory");
        return -1;
    }

    // Reset sprites
    for (int i = 0; i < 128; i++)
        Sprites[i].attribute[0] = ATTR0_DISABLED;

    int i = 0;
    for (int y = 0; y < 3; y++)
    {
        for (int x = 0; x < 4; x++)
        {
            Sprites[i].attribute[0] = ATTR0_BMP | ATTR0_SQUARE | (64 * y);
            Sprites[i].attribute[1] = ATTR1_SIZE_64 | (64 * x);
            Sprites[i].attribute[2] = ATTR2_ALPHA(1) | (8 * 32 * y) | (8 * x);
            i++;
        }
    }

    DC_FlushRange(Sprites, sizeof(SpriteEntry) * 128);

    dmaCopy(Sprites, OAM_SUB, 128 * sizeof(SpriteEntry));

    free(Sprites);

    return 0;
}

int NEA_InitDual3D(void)
{
    NEA_End();

    if (ne_systems_reset_all(NEA_VRAM_AB) != 0)
        return -1;

    if (ne_setup_sprites() != 0)
        return -2;

    NEA_DisplayListSetDefaultFunction(NEA_DL_DMA_GFX_FIFO);

    NEA_UpdateInput();

    ne_init_registers();

    videoSetModeSub(0);

    REG_BG2CNT_SUB = BG_BMP16_256x256;
    REG_BG2PA_SUB = 1 << 8;
    REG_BG2PB_SUB = 0;
    REG_BG2PC_SUB = 0;
    REG_BG2PD_SUB = 1 << 8;
    REG_BG2X_SUB = 0;
    REG_BG2Y_SUB = 0;

    vramSetBankC(VRAM_C_SUB_BG);
    vramSetBankD(VRAM_D_SUB_SPRITE);

    videoSetModeSub(MODE_5_2D | DISPLAY_BG2_ACTIVE | DISPLAY_SPR_ACTIVE |
                    DISPLAY_SPR_2D_BMP_256);

    ne_execution_mode = NEA_ModeDual3D;

    NEA_Screen = 0;

    NEA_DebugPrint("Nitro Engine Advanced initialized in dual 3D mode");

    return 0;
}

int NEA_InitDual3D_FB(void)
{
    NEA_End();

    if (ne_systems_reset_all(NEA_VRAM_AB) != 0)
        return -1;

    if (ne_setup_sprites() != 0)
        return -2;

    NEA_DisplayListSetDefaultFunction(NEA_DL_DMA_GFX_FIFO);

    NEA_UpdateInput();

    ne_init_registers();

    videoSetModeSub(0);

    REG_BG2CNT = BG_BMP16_256x256;
    REG_BG2PA = 1 << 8;
    REG_BG2PB = 0;
    REG_BG2PC = 0;
    REG_BG2PD = 1 << 8;
    REG_BG2X = 0;
    REG_BG2Y = 0;

    REG_BG2CNT_SUB = BG_BMP16_256x256;
    REG_BG2PA_SUB = 1 << 8;
    REG_BG2PB_SUB = 0;
    REG_BG2PC_SUB = 0;
    REG_BG2PD_SUB = 1 << 8;
    REG_BG2X_SUB = 0;
    REG_BG2Y_SUB = 0;

    vramSetBankC(VRAM_C_LCD);
    vramSetBankD(VRAM_D_LCD);

    videoSetMode(0);
    videoSetModeSub(0);

    ne_execution_mode = NEA_ModeDual3D_FB;

    NEA_Screen = 0;

    NEA_DebugPrint("Nitro Engine Advanced initialized in dual 3D FB mode");

    return 0;
}

int NEA_InitDual3D_DMA(void)
{
    NEA_End();

    if (ne_systems_reset_all(NEA_VRAM_AB) != 0)
        return -1;

    NEA_DisplayListSetDefaultFunction(NEA_DL_CPU);

    NEA_UpdateInput();

    ne_init_registers();

    videoSetMode(0);
    videoSetModeSub(0);

    ne_execution_mode = NEA_ModeDual3D_DMA;

    NEA_Screen = 0;

    NEA_DebugPrint("Nitro Engine Advanced initialized in dual 3D DMA mode");

    return 0;
}

// Helper: allocate two main RAM framebuffers for two-pass FIFO and DMA modes.
// Returns 0 on success, -2 on allocation failure.
static int ne_two_pass_alloc_framebuffers(void)
{
    for (int i = 0; i < 2; i++)
    {
        ne_two_pass_fb[i] = calloc(256 * 192, sizeof(u16));
        if (ne_two_pass_fb[i] == NULL)
        {
            NEA_DebugPrint("Not enough memory for two-pass framebuffers");
            for (int j = 0; j < i; j++)
            {
                free(ne_two_pass_fb[j]);
                ne_two_pass_fb[j] = NULL;
            }
            return -2;
        }
        // Flush dirty cache lines to physical RAM. DMA 2 reads directly from
        // physical RAM, bypassing the CPU cache. Without flushing, dirty cache
        // lines from calloc could be evicted later, overwriting DMA-written
        // pixel data with zeros and causing a black screen.
        DC_FlushRange(ne_two_pass_fb[i], 256 * 192 * sizeof(u16));
    }
    return 0;
}

int NEA_Init3D_TwoPass(void)
{
    NEA_End();

    // VRAM D is reserved for display capture, use A, B, C for textures.
    // Set execution mode BEFORE ne_systems_reset_all() so that
    // NEA_TextureSystemReset only masks off VRAM_D (not VRAM_C).
    ne_execution_mode = NEA_ModeSingle3D_TwoPass;

    if (ne_systems_reset_all(NEA_VRAM_ABC) != 0)
    {
        ne_execution_mode = NEA_ModeUninitialized;
        return -1;
    }

    // Use CPU-based display list drawing. DMA channel 0 display lists
    // (NEA_DL_DMA_GFX_FIFO) wait for ALL DMA channels to be idle before
    // starting. In FIFO mode, DMA 2 runs continuously (DMA_DISP_FIFO |
    // DMA_REPEAT), so dmaBusy(2) is always true and the wait deadlocks.
    NEA_DisplayListSetDefaultFunction(NEA_DL_CPU);

    NEA_UpdateInput();

    ne_init_registers();

    int ret = ne_two_pass_alloc_framebuffers();
    if (ret != 0)
        return ret;

    ne_two_pass_displayed = 0;
    ne_two_pass_next_ready = false;
    ne_two_pass_frame = 0;
    ne_two_pass_enabled = true;

    // Use VRAM D as destination for video capture
    vramSetBankD(VRAM_D_LCD);

    // Display from main RAM via display FIFO. DMA 2 is set up in
    // NEA_VBLFunc() with DMA_DISP_FIFO to continuously feed pixel data.
    //
    // Warning: this mode causes visible horizontal line artifacts on real
    // hardware because DMA 2 (display FIFO) and DMA 3 (dmaCopy for capture
    // data) contend on the main RAM bus simultaneously.
    videoSetMode(MODE_FIFO | ENABLE_3D);

    // Setup sub engine for 2D (available for console)
    videoSetModeSub(MODE_0_2D);

    NEA_DebugPrint("Nitro Engine Advanced initialized in two-pass FIFO mode");

    return 0;
}

int NEA_Init3D_TwoPass_FB(void)
{
    NEA_End();

    // VRAM C and D alternate between LCD (capture dest) and main BG (display).
    // Only VRAM A and B are available for textures (50% of max VRAM).
    ne_execution_mode = NEA_ModeSingle3D_TwoPass_FB;

    if (ne_systems_reset_all(NEA_VRAM_AB) != 0)
    {
        ne_execution_mode = NEA_ModeUninitialized;
        return -1;
    }

    // DMA 2 is not used in FB mode, so DMA-based display list drawing is safe
    // and provides better performance than CPU-based drawing.
    NEA_DisplayListSetDefaultFunction(NEA_DL_DMA_GFX_FIFO);

    NEA_UpdateInput();

    ne_init_registers();

    ne_two_pass_displayed = 0;
    ne_two_pass_next_ready = false;
    ne_two_pass_frame = 0;
    ne_two_pass_enabled = true;

    // No main RAM framebuffers needed - VRAM C/D serve as framebuffers.
    ne_two_pass_fb[0] = NULL;
    ne_two_pass_fb[1] = NULL;

    // Initial VRAM setup. The processing function reconfigures C/D each frame.
    vramSetBankC(VRAM_C_LCD);
    vramSetBankD(VRAM_D_LCD);

    // BG2 is used to display the captured half via a 16-bit bitmap BG.
    REG_BG2CNT = BG_BMP16_256x256;
    REG_BG2PA = 1 << 8;
    REG_BG2PB = 0;
    REG_BG2PC = 0;
    REG_BG2PD = 1 << 8;
    REG_BG2X = 0;
    REG_BG2Y = 0;

    videoSetMode(MODE_5_3D | DISPLAY_BG2_ACTIVE);

    // Setup sub engine for 2D (available for console)
    videoSetModeSub(MODE_0_2D);

    NEA_DebugPrint("Nitro Engine Advanced initialized in two-pass FB mode");

    return 0;
}

int NEA_Init3D_TwoPass_DMA(void)
{
    NEA_End();

    // VRAM D is reserved for display capture, use A, B, C for textures.
    // Set execution mode BEFORE ne_systems_reset_all() so that
    // NEA_TextureSystemReset only masks off VRAM_D (not VRAM_C).
    ne_execution_mode = NEA_ModeSingle3D_TwoPass_DMA;

    if (ne_systems_reset_all(NEA_VRAM_ABC) != 0)
    {
        ne_execution_mode = NEA_ModeUninitialized;
        return -1;
    }

    // Use CPU-based display list drawing. DMA channel 0 display lists
    // (NEA_DL_DMA_GFX_FIFO) wait for ALL DMA channels to be idle. DMA 2 is
    // active during HBL periods, which could cause occasional stalls.
    NEA_DisplayListSetDefaultFunction(NEA_DL_CPU);

    NEA_UpdateInput();

    ne_init_registers();

    int ret = ne_two_pass_alloc_framebuffers();
    if (ret != 0)
        return ret;

    ne_two_pass_displayed = 0;
    ne_two_pass_next_ready = false;
    ne_two_pass_frame = 0;
    ne_two_pass_enabled = true;

    // Use VRAM D as destination for video capture
    vramSetBankD(VRAM_D_LCD);

    // Use VRAM F as a one-scanline display buffer for HBL DMA.
    // The bitmap BG has vertical scale PD=0, so it repeats one row for all
    // 192 lines. DMA 2 is triggered at each HBL to copy a new scanline from
    // main RAM to this row.
    //
    // This avoids MODE_FIFO + DMA_DISP_FIFO which would keep DMA 2
    // continuously active and conflict with dmaCopy (DMA 3) on the main RAM
    // bus, causing visible line artifacts on real hardware.
    vramSetBankF(VRAM_F_MAIN_BG_0x06000000);

    videoSetMode(MODE_5_2D | DISPLAY_BG2_ACTIVE);

    REG_BG2CNT = BG_BMP16_256x256 | BG_BMP_BASE(0) | BG_PRIORITY(0);
    REG_BG2PA = 1 << 8;
    REG_BG2PB = 0;
    REG_BG2PC = 0;
    REG_BG2PD = 0; // Scale Y to 0: repeat the same row for all scanlines
    REG_BG2X = 0;
    REG_BG2Y = NEA_TWO_PASS_LINE_OFFSET << 8;

    // Setup sub engine for 2D (available for console)
    videoSetModeSub(MODE_0_2D);

    NEA_DebugPrint("Nitro Engine Advanced initialized in two-pass DMA mode");

    return 0;
}

void NEA_InitConsole(void)
{
    if (ne_execution_mode == NEA_ModeUninitialized)
        return;

    switch (ne_execution_mode)
    {
        case NEA_ModeSingle3D:
        case NEA_ModeDual3D:
        {
            videoBgEnable(1);

            vramSetBankF(VRAM_F_MAIN_BG);

            BG_PALETTE[255] = 0xFFFF;

            // Use BG 1 for text, set to highest priority
            REG_BG1CNT = BG_MAP_BASE(4) | BG_PRIORITY(0);

            // Set BG 0 (3D background) to be a lower priority than BG 1
            REG_BG0CNT = BG_PRIORITY(1);

            consoleInit(NULL, 1, BgType_Text4bpp, BgSize_T_256x256, 4, 0, true, true);

            break;
        }

        case NEA_ModeDual3D_FB:
        {
            NEA_Assert(false, "Debug console not supported in ModeDual3D_FB");
            break;
        }

        case NEA_ModeDual3D_DMA:
        {
            BG_PALETTE[255] = 0xFFFF;

            vramSetBankF(VRAM_F_LCD);
            vramSetBankG(VRAM_G_LCD);
            vramSetBankH(VRAM_H_LCD);

            // Main engine - VRAM_C:
            //     BMP base 0 (256x192): 0x06008000 - 0x06020000 (96 KB)
            //     Tile base 0: 0x06000000 - 0x06001000 (4 KB)
            //     Map base 8: 0x06004000 - 0x06004800 (2 KB)
            vramSetBankC(VRAM_C_MAIN_BG_0x06000000);
            consoleInit(NULL, 1, BgType_Text4bpp, BgSize_T_256x256, 8, 0, true, true);

            // We have just called consoleInit() to initialize the background
            // and to load the graphics. The next call to consoleInit() will
            // deinitialize the console from VRAM C, but it will leave the
            // graphics that have been loaded, which is what we wanted.
            //
            // VRAM I is always setup as sub background RAM, so we can rely on
            // the console always being mapped at the same address, while VRAM C
            // will alternate between LCD and main BG. We use libnds to write to
            // VRAM I, and copy the map to VRAM C whenever VRAM C is set as main
            // BG RAM.

            // Sub engine - VRAM_I:
            //     Available memory: 0x06208000 - 0x0620C000 (16 KB)
            //     Tile base 2: 0x06208000 - 0x06209000 (4 KB)
            //     Framebuffer (one line): 0x0620B000 - 0x0620B200 (512 B)
            //     Map base 23: 0x0620B800 - 0x0620C000 (2 KB)
            //
            // There is an overlap between the framebuffer and the tileset.
            // Tileset slots are 16 KB in size, so there is only one tileset
            // available in VRAM I. However, bitmap slots are also 16 KB in
            // size, so both the bitmap and tileset need to share the same
            // space.
            //
            // Luckily, the tileset of the console doesn't use the whole 16 KB.
            // The current console of libnds uses up to 128 characters. The
            // absolute maximum would be 256 characters, which requires (for a 4
            // bpp tileset) 8 KB of VRAM. This means that anything in the second
            // half of the bank (VRAM I is 16 KB in size) can be used for maps
            // or to store the pseudo framebuffer line.
            //
            // 8 KB is the same size as 16 lines in a 16-bit background. We can
            // setup our 16-bit bitmap as if it started at the same base as the
            // tileset, and we can store our line bitmap at some point between
            // 16 and 32 lines.
            //
            // Now, remember that the map uses the last 2 KB of the VRAM bank,
            // and that uses as much memory as 8 lines of a 16-bit bitmap. The
            // free lines of the bitmap are actually 16 to 24.
            //
            // Nitro Engine Advanced uses line 20, located at offset 0x3000 from the
            // start of the bank.
            vramSetBankI(VRAM_I_SUB_BG_0x06208000);
            consoleInit(NULL, 1, BgType_Text4bpp, BgSize_T_256x256, 23, 2, false, true);

            break;
        }

        case NEA_ModeSingle3D_TwoPass:
        case NEA_ModeSingle3D_TwoPass_FB:
        case NEA_ModeSingle3D_TwoPass_DMA:
        {
            // In all two-pass modes, the main screen is used for 3D display
            // and cannot host a console. Place the console on the sub screen.
            vramSetBankH(VRAM_H_SUB_BG);

            BG_PALETTE_SUB[255] = 0xFFFF;

            consoleInit(NULL, 3, BgType_Text4bpp, BgSize_T_256x256, 4, 0,
                        false, true);

            break;
        }

        default:
        {
            break;
        }
    }

    NEA_UsingConsole = true;
}

void NEA_SetConsoleColor(u32 color)
{
    BG_PALETTE[255] = color;
}

static void ne_process_common(void)
{
    NEA_UpdateInput();

    if (ne_main_screen == 1)
        lcdMainOnTop();
    else
        lcdMainOnBottom();

    NEA_PolyFormat(31, 0, NEA_LIGHT_ALL, NEA_CULL_BACK, 0);

    GFX_VIEWPORT = NEA_viewport;

    MATRIX_CONTROL = GL_PROJECTION;
    MATRIX_IDENTITY = 0;
    gluPerspectivef32(fov * DEGREES_IN_CIRCLE / 360, NEA_screenratio,
                      ne_znear, ne_zfar);

    MATRIX_CONTROL = GL_MODELVIEW;
    MATRIX_IDENTITY = 0;
}

void NEA_Process(NEA_Voidfunc drawscene)
{
    ne_process_common();

    NEA_AssertPointer(drawscene, "NULL function pointer");
    drawscene();

    GFX_FLUSH = GL_TRANS_MANUALSORT | ne_depth_buffer_mode;
}

void NEA_ProcessArg(NEA_VoidArgfunc drawscene, void *arg)
{
    ne_process_common();

    NEA_AssertPointer(drawscene, "NULL function pointer");
    drawscene(arg);

    GFX_FLUSH = GL_TRANS_MANUALSORT | ne_depth_buffer_mode;
}

int NEA_TwoPassGetPass(void)
{
    return ne_two_pass_frame;
}

// Shared helper: compute asymmetric frustum and set viewport for the current
// two-pass half. Used by all three two-pass modes.
static void ne_two_pass_setup_frustum(void)
{
    const int split = NEA_TWO_PASS_SPLIT;

    if (ne_two_pass_frame == 0)
        GFX_VIEWPORT = 0 | (0 << 8) | ((split - 1) << 16) | (191 << 24);
    else
        GFX_VIEWPORT = (split) | (0 << 8) | (255 << 16) | (191 << 24);

    // Compute asymmetric frustum from the user's FOV/znear/zfar settings.
    // This gives the same perspective as a full-screen render, but only
    // draws the left or right half.
    int32_t fovy = fov * DEGREES_IN_CIRCLE / 360;
    int32_t top = mulf32(ne_znear, tanLerp(fovy >> 1));
    int32_t full_aspect = divf32(256 << 12, 192 << 12);
    int32_t right_full = mulf32(top, full_aspect);

    MATRIX_CONTROL = GL_PROJECTION;
    MATRIX_IDENTITY = 0;

    if (ne_two_pass_frame == 0)
        glFrustumf32(-right_full, 0, -top, top, ne_znear, ne_zfar);
    else
        glFrustumf32(0, right_full, -top, top, ne_znear, ne_zfar);
}

// Processing for FIFO and DMA modes: copy capture from VRAM_D to main RAM
// framebuffer, set viewport, frustum, and enable display capture.
static void ne_process_two_pass_fifo_dma(void)
{
    NEA_UpdateInput();

    if (ne_main_screen == 1)
        lcdMainOnTop();
    else
        lcdMainOnBottom();

    // Enable display capture: capture 3D output into VRAM_D
    REG_DISPCAPCNT = DCAP_BANK(DCAP_BANK_VRAM_D)
                   | DCAP_SIZE(DCAP_SIZE_256x192)
                   | DCAP_MODE(DCAP_MODE_A)
                   | DCAP_SRC_A(DCAP_SRC_A_3DONLY)
                   | DCAP_ENABLE;

    const int split = NEA_TWO_PASS_SPLIT;

    if (ne_two_pass_frame == 0)
    {
        // Copy the previous left-half capture from VRAM_D to the next
        // framebuffer. VRAM_D contains the capture from the previous frame's
        // GPU output (one-frame delay due to double-buffered 3D rendering).
        const u16 *fb_src = VRAM_D;
        u16 *fb_dst = ne_two_pass_fb[ne_two_pass_displayed ^ 1];

        for (int j = 0; j < 192; j++)
        {
            dmaCopy(fb_src, fb_dst, split * sizeof(u16));
            fb_src += 256;
            fb_dst += 256;
        }
    }
    else
    {
        // Copy the previous right-half capture from VRAM_D
        const u16 *fb_src = VRAM_D + split;
        u16 *fb_dst = ne_two_pass_fb[ne_two_pass_displayed ^ 1] + split;

        for (int j = 0; j < 192; j++)
        {
            dmaCopy(fb_src, fb_dst, (256 - split) * sizeof(u16));
            fb_src += 256;
            fb_dst += 256;
        }

        // Both halves are now filled in the next framebuffer
        ne_two_pass_next_ready = true;
    }

    ne_two_pass_setup_frustum();

    NEA_PolyFormat(31, 0, NEA_LIGHT_ALL, NEA_CULL_BACK, 0);

    MATRIX_CONTROL = GL_MODELVIEW;
    MATRIX_IDENTITY = 0;
}

// Processing for FB mode: alternate VRAM C/D between LCD (capture) and main BG
// (display), toggle BG priorities and clear color alpha to composite both halves.
//
// How it works:
// The DS GPU has a one-frame rendering delay: the displayed 3D output is from
// the PREVIOUS glFlush, not the current one. We exploit this:
//
//   Frame 0 (left pass): submit left-half geometry. The displayed 3D shows the
//   previous right-half render. VRAM_D (BG2) shows the previous left-half
//   capture. BG2 is in front; its transparent pixels (from the transparent
//   clear of the right pass) let BG0's right-half 3D show through.
//
//   Frame 1 (right pass): submit right-half geometry. The displayed 3D shows
//   the previous left-half render. VRAM_C (BG2) shows the previous right-half
//   capture. BG0 is in front; its transparent clear pixels let BG2's left-half
//   capture show through.
//
// Each captured frame contains 3D objects with bit15=1 (opaque) in the rendered
// half, and clear-color pixels in the other half. The clear color alpha controls
// bit15 in the capture: opaque (alpha>=1) sets bit15=1, transparent (alpha=0)
// leaves bit15=0. BG2 treats bit15=0 pixels as transparent.
static void ne_process_two_pass_fb(void)
{
    NEA_UpdateInput();

    if (ne_main_screen == 1)
        lcdMainOnTop();
    else
        lcdMainOnBottom();

    u32 clearcolor = NEA_ClearColorGet();

    if (ne_two_pass_frame == 0)
    {
        // Left pass: submit left-half geometry.
        // Display VRAM_D as BG2 (shows previous right-half capture).
        // Capture 3D output to VRAM_C.
        vramSetBankC(VRAM_C_LCD);
        vramSetBankD(VRAM_D_MAIN_BG_0x06000000);

        // BG0 in front, BG2 behind. Due to the one-frame 3D delay, BG0
        // shows the RIGHT-half 3D from the previous frame (rendered with
        // transparent clear). Transparent pixels (alpha=0) in the left
        // columns let BG2 (the captured left-half) show through.
        REG_BG0CNT = (REG_BG0CNT & ~BG_PRIORITY(3)) | BG_PRIORITY(0);
        REG_BG2CNT = (REG_BG2CNT & ~BG_PRIORITY(3)) | BG_PRIORITY(1);

        REG_DISPCAPCNT = DCAP_BANK(DCAP_BANK_VRAM_C)
                       | DCAP_SIZE(DCAP_SIZE_256x192)
                       | DCAP_MODE(DCAP_MODE_A)
                       | DCAP_SRC_A(DCAP_SRC_A_3DONLY)
                       | DCAP_ENABLE;

        // Opaque clear color: the user's background color with alpha forced
        // to 31. Pixels in the left half will be either 3D objects or this
        // opaque clear, both with bit15=1 in the capture.
        GFX_CLEAR_COLOR = (clearcolor & ~(0x1F << 16)) | (31 << 16);
    }
    else
    {
        // Right pass: submit right-half geometry.
        // Display VRAM_C as BG2 (shows previous left-half capture).
        // Capture 3D output to VRAM_D.
        vramSetBankC(VRAM_C_MAIN_BG_0x06000000);
        vramSetBankD(VRAM_D_LCD);

        // BG2 in front, BG0 behind. Due to the one-frame 3D delay, BG0
        // shows the LEFT-half 3D from the previous frame (rendered with
        // opaque clear, all pixels bit15=1). BG2 shows the captured
        // RIGHT-half (transparent clear): right objects bit15=1, left
        // columns bit15=0, letting BG0's left-half show through.
        REG_BG0CNT = (REG_BG0CNT & ~BG_PRIORITY(3)) | BG_PRIORITY(1);
        REG_BG2CNT = (REG_BG2CNT & ~BG_PRIORITY(3)) | BG_PRIORITY(0);

        REG_DISPCAPCNT = DCAP_BANK(DCAP_BANK_VRAM_D)
                       | DCAP_SIZE(DCAP_SIZE_256x192)
                       | DCAP_MODE(DCAP_MODE_A)
                       | DCAP_SRC_A(DCAP_SRC_A_3DONLY)
                       | DCAP_ENABLE;

        // Transparent clear color: alpha=0 so that pixels outside the
        // right-half viewport have bit15=0 in the 3D output, letting BG2
        // (the captured left half) show through.
        GFX_CLEAR_COLOR = clearcolor & ~(0x1F << 16);

        // Both halves are now being composited on screen
        ne_two_pass_next_ready = true;
    }

    ne_two_pass_setup_frustum();

    NEA_PolyFormat(31, 0, NEA_LIGHT_ALL, NEA_CULL_BACK, 0);

    MATRIX_CONTROL = GL_MODELVIEW;
    MATRIX_IDENTITY = 0;
}

static void ne_process_two_pass_end(void)
{
    GFX_FLUSH = GL_TRANS_MANUALSORT | ne_depth_buffer_mode;

    ne_two_pass_frame ^= 1;
}

void NEA_ProcessTwoPass(NEA_Voidfunc drawscene)
{
    NEA_AssertPointer(drawscene, "NULL function pointer");

    if (ne_execution_mode == NEA_ModeSingle3D_TwoPass_FB)
        ne_process_two_pass_fb();
    else
        ne_process_two_pass_fifo_dma();

    drawscene();

    ne_process_two_pass_end();
}

void NEA_ProcessTwoPassArg(NEA_VoidArgfunc drawscene, void *arg)
{
    NEA_AssertPointer(drawscene, "NULL function pointer");

    if (ne_execution_mode == NEA_ModeSingle3D_TwoPass_FB)
        ne_process_two_pass_fb();
    else
        ne_process_two_pass_fifo_dma();

    drawscene(arg);

    ne_process_two_pass_end();
}

static void ne_process_dual_3d_common_start(void)
{
    NEA_UpdateInput();

    if (NEA_Screen == ne_main_screen)
        lcdMainOnTop();
    else
        lcdMainOnBottom();

    if (NEA_Screen == 1)
    {
        if (NEA_UsingConsole)
        {
            REG_BG1CNT = BG_MAP_BASE(4) | BG_PRIORITY(0);
            REG_BG0CNT = BG_PRIORITY(1);
        }

        vramSetBankC(VRAM_C_SUB_BG);
        vramSetBankD(VRAM_D_LCD);

        REG_DISPCAPCNT = DCAP_SIZE(DCAP_SIZE_256x192)
                       | DCAP_BANK(DCAP_BANK_VRAM_D)
                       | DCAP_MODE(DCAP_MODE_A)
                       | DCAP_SRC_A(DCAP_SRC_A_COMPOSITED)
                       | DCAP_ENABLE;
    }
    else
    {
        if (NEA_UsingConsole)
        {
            REG_BG1CNT = BG_PRIORITY(1);
            REG_BG0CNT = BG_PRIORITY(0);
        }

        vramSetBankC(VRAM_C_LCD);
        vramSetBankD(VRAM_D_SUB_SPRITE);

        REG_DISPCAPCNT = DCAP_SIZE(DCAP_SIZE_256x192)
                       | DCAP_BANK(DCAP_BANK_VRAM_C)
                       | DCAP_MODE(DCAP_MODE_A)
                       | DCAP_SRC_A(DCAP_SRC_A_COMPOSITED)
                       | DCAP_ENABLE;
    }

    NEA_PolyFormat(31, 0, NEA_LIGHT_ALL, NEA_CULL_BACK, 0);

    NEA_Viewport(0, 0, 255, 191);

    MATRIX_IDENTITY = 0;
}

static void ne_process_dual_3d_common_end(void)
{
    GFX_FLUSH = GL_TRANS_MANUALSORT | ne_depth_buffer_mode;

    NEA_Screen ^= 1;
}

static void ne_process_dual_3d(NEA_Voidfunc mainscreen, NEA_Voidfunc subscreen)
{
    ne_process_dual_3d_common_start();

    if (NEA_Screen == 1)
        mainscreen();
    else
        subscreen();

    ne_process_dual_3d_common_end();
}

static void ne_process_dual_3d_arg(NEA_VoidArgfunc mainscreen,
                                   NEA_VoidArgfunc subscreen,
                                   void *argmain, void *argsub)
{
    ne_process_dual_3d_common_start();

    if (NEA_Screen == 1)
        mainscreen(argmain);
    else
        subscreen(argsub);

    ne_process_dual_3d_common_end();
}

#define NEA_DUAL_DMA_3D_LINES_OFFSET 20

static void ne_process_dual_3d_fb_common_start(void)
{
    NEA_UpdateInput();

    if (NEA_Screen == ne_main_screen)
        lcdMainOnTop();
    else
        lcdMainOnBottom();

    if (NEA_Screen == 1)
    {
        videoSetMode(MODE_FB3);
        videoSetModeSub(MODE_5_2D | DISPLAY_BG2_ACTIVE);

        vramSetBankC(VRAM_C_SUB_BG);
        vramSetBankD(VRAM_D_LCD);

        REG_DISPCAPCNT = DCAP_SIZE(DCAP_SIZE_256x192)
                       | DCAP_BANK(DCAP_BANK_VRAM_D)
                       | DCAP_MODE(DCAP_MODE_A)
                       | DCAP_SRC_A(DCAP_SRC_A_3DONLY)
                       | DCAP_ENABLE;
    }
    else
    {
        videoSetMode(MODE_FB2);
        videoSetModeSub(MODE_5_2D | DISPLAY_SPR_ACTIVE | DISPLAY_SPR_2D_BMP_256);

        vramSetBankC(VRAM_C_LCD);
        vramSetBankD(VRAM_D_SUB_SPRITE);

        REG_DISPCAPCNT = DCAP_SIZE(DCAP_SIZE_256x192)
                       | DCAP_BANK(DCAP_BANK_VRAM_C)
                       | DCAP_MODE(DCAP_MODE_A)
                       | DCAP_SRC_A(DCAP_SRC_A_3DONLY)
                       | DCAP_ENABLE;
    }

    NEA_PolyFormat(31, 0, NEA_LIGHT_ALL, NEA_CULL_BACK, 0);

    NEA_Viewport(0, 0, 255, 191);

    MATRIX_IDENTITY = 0;
}

static void ne_process_dual_3d_fb_common_end(void)
{
    GFX_FLUSH = GL_TRANS_MANUALSORT | ne_depth_buffer_mode;

    NEA_Screen ^= 1;
}

static void ne_process_dual_3d_fb(NEA_Voidfunc mainscreen, NEA_Voidfunc subscreen)
{
    ne_process_dual_3d_fb_common_start();

    if (NEA_Screen == 1)
        mainscreen();
    else
        subscreen();

    ne_process_dual_3d_fb_common_end();
}

static void ne_process_dual_3d_fb_arg(NEA_VoidArgfunc mainscreen,
                                      NEA_VoidArgfunc subscreen,
                                      void *argmain, void *argsub)
{
    ne_process_dual_3d_fb_common_start();

    if (NEA_Screen == 1)
        mainscreen(argmain);
    else
        subscreen(argsub);

    ne_process_dual_3d_fb_common_end();
}

static void ne_do_dma(void)
{
    // BlocksDS has a safe way to start DMA copies that doesn't involve writing
    // to registers directly. It's safer to call the functions directly. The
    // libnds of devkitPro doesn't have this functionality.
#ifdef NEA_BLOCKSDS
    dmaStopSafe(2);

    dmaSetParams(2, (const void *)ne_dma_src, (void *)ne_dma_dst, ne_dma_cr);
#else
    DMA_CR(2) = 0;

    DMA_SRC(2) = ne_dma_src;
    DMA_DEST(2) = ne_dma_dst;
    DMA_CR(2) = ne_dma_cr;
#endif
}

static void ne_process_dual_3d_dma_common_start(void)
{
    if (NEA_Screen == ne_main_screen)
        lcdMainOnBottom();
    else
        lcdMainOnTop();

    NEA_PolyFormat(31, 0, NEA_LIGHT_ALL, NEA_CULL_BACK, 0);

    NEA_Viewport(0, 0, 255, 191);

    MATRIX_IDENTITY = 0;

    if (NEA_Screen == 1)
    {
        // DMA copies from VRAM C to VRAM I

        // Main engine: displays VRAM D as 16-bit BG
        // Sub engine: displays VRAM I as 16-bit BG

        videoSetMode(MODE_5_2D | DISPLAY_BG2_ACTIVE);
        if (NEA_UsingConsole)
            videoSetModeSub(MODE_5_2D | DISPLAY_BG1_ACTIVE | DISPLAY_BG2_ACTIVE);
        else
            videoSetModeSub(MODE_5_2D | DISPLAY_BG2_ACTIVE);

        vramSetBankC(VRAM_C_LCD);
        vramSetBankD(VRAM_D_MAIN_BG_0x06000000);
        vramSetBankI(VRAM_I_SUB_BG_0x06208000);

        REG_DISPCAPCNT = DCAP_SIZE(DCAP_SIZE_256x192)
                       | DCAP_BANK(DCAP_BANK_VRAM_C)
                       | DCAP_OFFSET(1) // Write with an offset of 0x8000
                       | DCAP_MODE(DCAP_MODE_A)
                       | DCAP_SRC_A(DCAP_SRC_A_3DONLY)
                       | DCAP_ENABLE;

        REG_BG2CNT = BG_BMP16_256x256 | BG_BMP_BASE(2) | BG_PRIORITY(2);
        REG_BG2PA = 1 << 8;
        REG_BG2PB = 0;
        REG_BG2PC = 0;
        REG_BG2PD = 1 << 8;
        REG_BG2X = 0;
        REG_BG2Y = -1 << 8;

        REG_BG2CNT_SUB = BG_BMP16_256x256 | BG_BMP_BASE(2) | BG_PRIORITY(2);
        REG_BG2PA_SUB = 1 << 8;
        REG_BG2PB_SUB = 0;
        REG_BG2PC_SUB = 0;
        REG_BG2PD_SUB = 0; // Scale first row to expand to the full screen
        REG_BG2X_SUB = 0;
        REG_BG2Y_SUB = NEA_DUAL_DMA_3D_LINES_OFFSET << 8;

        ne_dma_enabled = 1;
        ne_dma_src = (uint32_t)VRAM_C + 0x8000;
        ne_dma_dst = ((uint32_t)BG_BMP_RAM_SUB(2))
                   + 256 * NEA_DUAL_DMA_3D_LINES_OFFSET * 2;
        ne_dma_cr = DMA_COPY_WORDS | (256 * 2 / 4) |
                    DMA_START_HBL | DMA_REPEAT | DMA_SRC_INC | DMA_DST_RESET;

        ne_do_dma();
    }
    else
    {
        // DMA copies from VRAM D to VRAM I

        // Main engine: displays VRAM C as 16-bit sprites
        // Sub engine: displays VRAM I as 16-bit BG

        if (NEA_UsingConsole)
            videoSetMode(MODE_5_2D | DISPLAY_BG1_ACTIVE | DISPLAY_BG2_ACTIVE);
        else
            videoSetMode(MODE_5_2D | DISPLAY_BG2_ACTIVE);
        videoSetModeSub(MODE_5_2D | DISPLAY_BG2_ACTIVE);

        vramSetBankC(VRAM_C_MAIN_BG_0x06000000);
        vramSetBankD(VRAM_D_LCD);
        vramSetBankI(VRAM_I_SUB_BG_0x06208000);

        REG_DISPCAPCNT = DCAP_SIZE(DCAP_SIZE_256x192)
                       | DCAP_BANK(DCAP_BANK_VRAM_D)
                       | DCAP_OFFSET(1) // Write with an offset of 0x8000
                       | DCAP_MODE(DCAP_MODE_A)
                       | DCAP_SRC_A(DCAP_SRC_A_3DONLY)
                       | DCAP_ENABLE;

        REG_BG2CNT = BG_BMP16_256x256 | BG_BMP_BASE(2) | BG_PRIORITY(2);
        REG_BG2PA = 1 << 8;
        REG_BG2PB = 0;
        REG_BG2PC = 0;
        REG_BG2PD = 1 << 8;
        REG_BG2X = 0;
        REG_BG2Y = -1 << 8;

        REG_BG2CNT_SUB = BG_BMP16_256x256 | BG_BMP_BASE(2) | BG_PRIORITY(2);
        REG_BG2PA_SUB = 1 << 8;
        REG_BG2PB_SUB = 0;
        REG_BG2PC_SUB = 0;
        REG_BG2PD_SUB = 0; // Scale first row to expand to the full screen
        REG_BG2X_SUB = 0;
        REG_BG2Y_SUB = NEA_DUAL_DMA_3D_LINES_OFFSET << 8;

        ne_dma_enabled = 1;
        ne_dma_src = (uint32_t)VRAM_D + 0x8000;
        ne_dma_dst = ((uint32_t)BG_BMP_RAM_SUB(2))
                   + 256 * NEA_DUAL_DMA_3D_LINES_OFFSET * 2;
        ne_dma_cr = DMA_COPY_WORDS | (256 * 2 / 4) |
                    DMA_START_HBL | DMA_REPEAT | DMA_SRC_INC | DMA_DST_RESET;

        // Synchronize console of the main engine from the sub engine. Use a
        // channel other than 2, that one is used for HBL copies.
        if (NEA_UsingConsole)
            dmaCopyWords(3, BG_MAP_RAM_SUB(23), BG_MAP_RAM(8), 32 * 32 * 2);

        ne_do_dma();
    }
}

static void ne_process_dual_3d_dma_common_end(void)
{
    GFX_FLUSH = GL_TRANS_MANUALSORT | ne_depth_buffer_mode;

    NEA_Screen ^= 1;

    NEA_UpdateInput();
}

static void ne_process_dual_3d_dma(NEA_Voidfunc mainscreen, NEA_Voidfunc subscreen)
{
    ne_process_dual_3d_dma_common_start();

    if (NEA_Screen == 1)
        mainscreen();
    else
        subscreen();

    ne_process_dual_3d_dma_common_end();
}

static void ne_process_dual_3d_dma_arg(NEA_VoidArgfunc mainscreen,
                                       NEA_VoidArgfunc subscreen,
                                       void *argmain, void *argsub)
{
    ne_process_dual_3d_dma_common_start();

    if (NEA_Screen == 1)
        mainscreen(argmain);
    else
        subscreen(argsub);

    ne_process_dual_3d_dma_common_end();
}

void NEA_ProcessDual(NEA_Voidfunc mainscreen, NEA_Voidfunc subscreen)
{
    NEA_AssertPointer(mainscreen, "NULL function pointer (main screen)");
    NEA_AssertPointer(subscreen, "NULL function pointer (sub screen)");

    switch (ne_execution_mode)
    {
        case NEA_ModeDual3D:
        {
            ne_process_dual_3d(mainscreen, subscreen);
            return;
        }

        case NEA_ModeDual3D_FB:
        {
            ne_process_dual_3d_fb(mainscreen, subscreen);
            return;
        }

        case NEA_ModeDual3D_DMA:
        {
            ne_process_dual_3d_dma(mainscreen, subscreen);
            return;
        }
        default:
        {
            return;
        }
    }
}

void NEA_ProcessDualArg(NEA_VoidArgfunc mainscreen, NEA_VoidArgfunc subscreen,
                       void *argmain, void *argsub)
{
    NEA_AssertPointer(mainscreen, "NULL function pointer (main screen)");
    NEA_AssertPointer(subscreen, "NULL function pointer (sub screen)");

    switch (ne_execution_mode)
    {
        case NEA_ModeDual3D:
        {
            ne_process_dual_3d_arg(mainscreen, subscreen, argmain, argsub);
            return;
        }

        case NEA_ModeDual3D_FB:
        {
            ne_process_dual_3d_fb_arg(mainscreen, subscreen, argmain, argsub);
            return;
        }

        case NEA_ModeDual3D_DMA:
        {
            ne_process_dual_3d_dma_arg(mainscreen, subscreen, argmain, argsub);
            return;
        }
        default:
        {
            return;
        }
    }
}

void NEA_ClippingPlanesSetI(int znear, int zfar)
{
    NEA_Assert(znear < zfar, "znear must be smaller than zfar");
    ne_znear = znear;
    ne_zfar = zfar;
}

void NEA_AntialiasEnable(bool value)
{
    if (value)
        GFX_CONTROL |= GL_ANTIALIAS;
    else
        GFX_CONTROL &= ~GL_ANTIALIAS;
}

int NEA_GetPolygonCount(void)
{
    // Wait for geometry engine operations to end
    while (GFX_STATUS & BIT(27));

    return GFX_POLYGON_RAM_USAGE;
}

int NEA_GetVertexCount(void)
{
    // Wait for geometry engine operations to end
    while (GFX_STATUS & BIT(27));

    return GFX_VERTEX_RAM_USAGE;
}

static int NEA_Effect = NEA_NONE;
static int NEA_lastvbladd = 0;
static bool NEA_effectpause;
#define NEA_NOISEPAUSE_SIZE 512
static int *ne_noisepause;
static int ne_cpucount;
static int ne_noise_value = 0xF;
static int ne_sine_mult = 10, ne_sine_shift = 9;

void NEA_VBLFunc(void)
{
    if (ne_execution_mode == NEA_ModeUninitialized)
        return;

    if (ne_dma_enabled)
    {
        // The first line of the sub screen must be set to black during VBL
        // because the DMA transfer won't start until the first HBL, which
        // happens after the first line has been drawn.
        //
        // For consistency, in the main screen this is achieved by simply
        // scrolling the bitmap background by one pixel.
        dmaFillWords(0, BG_BMP_RAM_SUB(2) + 256 * NEA_DUAL_DMA_3D_LINES_OFFSET,
                     256 * 2);

        ne_do_dma();
    }

    if (ne_two_pass_enabled)
    {
        // Swap to the next framebuffer when it is complete (all modes)
        if (ne_two_pass_next_ready)
        {
            ne_two_pass_next_ready = false;
            ne_two_pass_displayed ^= 1;
        }

        if (ne_execution_mode == NEA_ModeSingle3D_TwoPass)
        {
            // FIFO mode: stop DMA 2, then restart with DMA_DISP_FIFO.
            // DMA 2 continuously feeds pixel data from main RAM to the
            // display hardware via REG_DISP_MMEM_FIFO.
#ifdef NEA_BLOCKSDS
            dmaStopSafe(2);
#else
            DMA_CR(2) = 0;
#endif

            void *fb = ne_two_pass_fb[ne_two_pass_displayed];

#ifdef NEA_BLOCKSDS
            dmaSetParams(2, fb, (void *)&REG_DISP_MMEM_FIFO,
                         DMA_DISP_FIFO | DMA_SRC_INC | DMA_DST_FIX
                         | DMA_REPEAT | DMA_COPY_WORDS | 4);
#else
            DMA_SRC(2) = (uint32_t)fb;
            DMA_DEST(2) = (uint32_t)&REG_DISP_MMEM_FIFO;
            DMA_CR(2) = DMA_DISP_FIFO | DMA_SRC_INC | DMA_DST_FIX
                       | DMA_REPEAT | DMA_COPY_WORDS | 4;
#endif
        }
        else if (ne_execution_mode == NEA_ModeSingle3D_TwoPass_DMA)
        {
            // HBL DMA mode: stop DMA 2, black out the scanline row in
            // VRAM_F, then restart with HBL-triggered transfer.
#ifdef NEA_BLOCKSDS
            dmaStopSafe(2);
#else
            DMA_CR(2) = 0;
#endif

            // The first display line is drawn before the first HBL fires,
            // so it would show stale data. Fill the row with black.
            dmaFillWords(0,
                         BG_BMP_RAM(0) + 256 * NEA_TWO_PASS_LINE_OFFSET,
                         256 * 2);

            // Copy one scanline at a time from main RAM to VRAM_F at each
            // HBL. DMA 2 is only briefly active during each HBL, avoiding
            // continuous bus contention with dmaCopy (DMA 3).
            uint32_t two_pass_cr =
                DMA_COPY_WORDS  |
                (256 * 2 / 4)   | // 128 words = one scanline
                DMA_START_HBL   | // Trigger at each HBL
                DMA_REPEAT      | // Repeat for all 192 lines
                DMA_SRC_INC     | // Increment source (next line each HBL)
                DMA_DST_RESET;    // Reset dest to same VRAM row each HBL

            void *dst = (void *)((uint32_t)BG_BMP_RAM(0)
                       + 256 * NEA_TWO_PASS_LINE_OFFSET * sizeof(u16));

#ifdef NEA_BLOCKSDS
            dmaSetParams(2, ne_two_pass_fb[ne_two_pass_displayed],
                         dst, two_pass_cr);
#else
            DMA_SRC(2) = (uint32_t)ne_two_pass_fb[ne_two_pass_displayed];
            DMA_DEST(2) = (uint32_t)dst;
            DMA_CR(2) = two_pass_cr;
#endif
        }
        // FB mode: no DMA 2 handling needed. Compositing is done via BG
        // priority alternation in the processing function.
    }

    if (NEA_Effect == NEA_NOISE || NEA_Effect == NEA_SINE)
    {
        if (!NEA_effectpause)
            NEA_lastvbladd = (NEA_lastvbladd + 1) & (NEA_NOISEPAUSE_SIZE - 1);
    }
}

void NEA_SpecialEffectPause(bool pause)
{
    if (NEA_Effect == 0)
        return;

    if (pause)
    {
        ne_noisepause = malloc(sizeof(int) * NEA_NOISEPAUSE_SIZE);
        if (ne_noisepause == NULL)
        {
            NEA_DebugPrint("Not enough memory");
            return;
        }

        for (int i = 0; i < NEA_NOISEPAUSE_SIZE; i++)
        {
            ne_noisepause[i] = (rand() & ne_noise_value)
                             - (ne_noise_value >> 1);
        }
    }
    else
    {
        if (ne_noisepause != NULL)
        {
            free(ne_noisepause);
            ne_noisepause = NULL;
        }
    }

    NEA_effectpause = pause;
}

void NEA_HBLFunc(void)
{
    if (ne_execution_mode == NEA_ModeUninitialized)
        return;

    s16 angle;
    int val;

    // This counter is used to estimate CPU usage
    ne_cpucount++;

    // Fix a problem with the first line when using effects
    int vcount = REG_VCOUNT;
    if (vcount == 262)
        vcount = 0;

    switch (NEA_Effect)
    {
        case NEA_NOISE:
            if (NEA_effectpause && ne_noisepause)
                val = ne_noisepause[vcount & (NEA_NOISEPAUSE_SIZE - 1)];
            else
                val = (rand() & ne_noise_value) - (ne_noise_value >> 1);
            REG_BG0HOFS = val;
            break;

        case NEA_SINE:
            angle = (vcount + NEA_lastvbladd) * ne_sine_mult;
            REG_BG0HOFS = sinLerp(angle << 6) >> ne_sine_shift;
            break;

        default:
            break;
    }
}

void NEA_SpecialEffectNoiseConfig(int value)
{
    ne_noise_value = value;
}

void NEA_SpecialEffectSineConfig(int mult, int shift)
{
    ne_sine_mult = mult;
    ne_sine_shift = shift;
}

void NEA_SpecialEffectSet(NEA_SpecialEffects effect)
{
    NEA_Effect = effect;

    if (effect == NEA_NONE)
        REG_BG0HOFS = 0;
}

static int NEA_CPUPercent;

void NEA_WaitForVBL(NEA_UpdateFlags flags)
{
    if (flags & NEA_UPDATE_GUI)
        NEA_GUIUpdate();
    if (flags & NEA_UPDATE_ANIMATIONS)
        NEA_ModelAnimateAll();
    if (flags & NEA_UPDATE_PHYSICS)
        NEA_PhysicsUpdateAll();
    // Weak reference: if user code links NEASound (by calling any NEA_Sound*
    // function), the strong definition is pulled from the archive and used.
    // Otherwise this resolves to NULL and the call is safely skipped.
    // This avoids forcing -lmm9 on examples that don't use sound.
    extern void NEA_SoundUpdateAll(void) __attribute__((weak));
    if ((flags & NEA_UPDATE_SOUND) && NEA_SoundUpdateAll)
        NEA_SoundUpdateAll();

    NEA_CPUPercent = div32(ne_cpucount * 100, 263);
    if (flags & NEA_CAN_SKIP_VBL)
    {
        if (NEA_CPUPercent > 100)
        {
            ne_cpucount = 0;
            return;

            // REG_DISPSTAT & DISP_IN_VBLANK
        }
    }

    swiWaitForVBlank();
    ne_cpucount = 0;
}

int NEA_GetCPUPercent(void)
{
    return NEA_CPUPercent;
}

bool NEA_GPUIsRendering(void)
{
    if (REG_VCOUNT > 190 && REG_VCOUNT < 214)
        return false;

    return true;
}

#ifdef NEA_DEBUG
static void (*ne_userdebugfn)(const char *) = NULL;

void __ne_debugoutputtoconsole(const char *text)
{
    printf(text);
}

void __NEA_debugprint(const char *text)
{
    if (ne_execution_mode == NEA_ModeUninitialized)
        return;

    if (ne_userdebugfn)
        ne_userdebugfn(text);
}

void NEA_DebugSetHandler(void (*fn)(const char *))
{
    ne_userdebugfn = fn;
}

void NEA_DebugSetHandlerConsole(void)
{
    NEA_InitConsole();
    ne_userdebugfn = __ne_debugoutputtoconsole;
}
#endif

static int ne_vertexcount;

void NEA_TouchTestStart(void)
{
    // Hide what we are going to draw
    GFX_VIEWPORT = 255 | (255 << 8) | (255 << 16) | (255 << 24);

    // Save current state
    MATRIX_CONTROL = GL_MODELVIEW;
    MATRIX_PUSH = 0;
    MATRIX_CONTROL = GL_PROJECTION;
    MATRIX_PUSH = 0;

    // Setup temporary render environment
    MATRIX_IDENTITY = 0;

    int temp[4] = {
        NEA_viewport & 0xFF,
        (NEA_viewport >> 8) & 0xFF,
        (NEA_viewport >> 16) & 0xFF,
        (NEA_viewport >> 24) & 0xFF
    };

    // Render only what is below the cursor
    gluPickMatrix(ne_input.touch.px, 191 - ne_input.touch.py, 3, 3, temp);
    gluPerspectivef32(fov * DEGREES_IN_CIRCLE / 360, NEA_screenratio,
                      ne_znear, ne_zfar);

    MATRIX_CONTROL = GL_MODELVIEW;

    NEA_Assert(!NEA_TestTouch, "Test already active");

    NEA_TestTouch = true;
}

void NEA_TouchTestObject(void)
{
    NEA_Assert(NEA_TestTouch, "No active test");

    // Wait for the position test to finish
    while (PosTestBusy());

    // Wait for geometry engine operations to end
    while (GFX_STATUS & BIT(27));

    // Save the vertex ram count
    ne_vertexcount = NEA_GetVertexCount();
}

int NEA_TouchTestResult(void)
{
    NEA_Assert(NEA_TestTouch, "No active test");

    // Wait for geometry engine operations to end
    while (GFX_STATUS & BIT(27));

    // Wait for the position test to finish
    while (PosTestBusy());

    // If a polygon was drawn
    if (NEA_GetVertexCount() > ne_vertexcount)
        return PosTestWresult();

    return -1;
}

void NEA_TouchTestEnd(void)
{
    NEA_Assert(NEA_TestTouch, "No active test");

    NEA_TestTouch = false;

    // Reset the viewport
    GFX_VIEWPORT = NEA_viewport;

    // Restore previous state
    MATRIX_CONTROL = GL_PROJECTION;
    MATRIX_POP = 1;
    MATRIX_CONTROL = GL_MODELVIEW;
    MATRIX_POP = 1;
}
