// SPDX-License-Identifier: MIT
//
// Copyright (c) 2008-2023 Antonio Niño Díaz
//
// This file is part of Nitro Engine Advanced

#ifndef NEA_GENERAL_H__
#define NEA_GENERAL_H__

#include <nds.h>

#include "NEAMain.h"

/// @file   NEAGeneral.h
/// @brief  Init 3D mode and process functions.

/// @defgroup general_utils General utilities
///
/// General functions to control Nitro Engine Advanced, setup screens, print debug
/// messages, etc.
///
/// If Nitro Engine Advanced is compiled with debug features, it will check a lot of
/// things and it will print error messages to an user-defined function. Check
/// the error handling example for more details. If you have finished testing
/// your code, just comment the define out and recompile Nitro Engine Advanced to save
/// RAM and CPU usage.
///
/// @{

/// Void function pointer used in NEA_Process and NEA_ProcessDual.
typedef void (*NEA_Voidfunc)(void);

/// Void function pointer that takes a void pointer as argument.
///
/// Used in NEA_ProcessArg and NEA_ProcessDualArg.
typedef void (*NEA_VoidArgfunc)(void *);

/// Holds information of keys and stylus input for internal use.
typedef struct {
    uint32_t kdown;         ///< Keys that have just been pressed this frame.
    uint32_t kheld;         ///< Keys that are pressed.
    uint32_t kup;           ///< Keys that have just been released this frame.
    touchPosition touch;    ///< Touch screen state
} NEA_Input;

/// Updates input data for internal use.
///
/// Use this if you are using NEA_GUI or window system. You have to call
/// scanKeys() each frame for this to work.
void NEA_UpdateInput(void);

/// List of all the possible initialization states of Nitro Engine Advanced.
typedef enum {
    NEA_ModeUninitialized        = 0, ///< Nitro Engine Advanced hasn't been initialized.
    NEA_ModeSingle3D             = 1, ///< Initialized in single 3D mode.
    NEA_ModeDual3D               = 2, ///< Initialized in regular dual 3D mode.
    NEA_ModeDual3D_FB            = 3, ///< Initialized in dual 3D FB mode (no debug console).
    NEA_ModeDual3D_DMA           = 4, ///< Initialized in safe dual 3D mode.
    NEA_ModeSingle3D_TwoPass     = 5, ///< Initialized in two-pass FIFO mode.
    NEA_ModeSingle3D_TwoPass_FB  = 6, ///< Initialized in two-pass framebuffer mode.
    NEA_ModeSingle3D_TwoPass_DMA = 7  ///< Initialized in two-pass HBL DMA mode.
} NEA_ExecutionModes;

/// Returns the current execution mode.
///
/// @return Returns the execution mode.
NEA_ExecutionModes NEA_CurrentExecutionMode(void);

/// Ends Nitro Engine Advanced and frees all memory used by it.
void NEA_End(void);

/// Inits Nitro Engine Advanced and 3D mode in one screen.
///
/// @return Returns 0 on success.
int NEA_Init3D(void);

/// Draws a 3D scene.
///
/// @param drawscene Function that draws the screen.
void NEA_Process(NEA_Voidfunc drawscene);

/// Draws a 3D scene passing an argument to the draw function.
///
/// @param drawscene Function that draws the screen.
/// @param arg Argument to pass to the drawscene function.
void NEA_ProcessArg(NEA_VoidArgfunc drawscene, void *arg);

/// Inits Nitro Engine Advanced to draw 3D to both screens.
///
/// VRAM banks C and D are used as framebuffers, which means there is only 50%
/// of the normally available VRAM for textures.
///
/// This way to display 3D is unsafe. If the framerate of the game drops below
/// 60 FPS the result will be having the same 3D output displayed on both
/// screens during the frames when it doesn't have time to draw anything new.
///
/// However, this mode allows you to use DMA in GFX FIFO mode to draw models,
/// which is more efficient.
///
/// In general, prefer NEA_InitDual3D_DMA() over NEA_InitDual3D().
///
/// @return Returns 0 on success.
int NEA_InitDual3D(void);

/// Inits Nitro Engine Advanced to draw 3D to both screens.
///
/// VRAM banks C and D are used as framebuffers, which means there is only 50%
/// of the normally available VRAM for textures.
///
/// Direct VRAM display mode is used for the main engine, which means there is
/// no way to display the debug console on either screen in a stable way, so the
/// debug console is not supported in this mode.
///
/// This mode is stable. If the framerate drops below 60 FPS the screens will
/// remain stable.
///
/// @return Returns 0 on success.
int NEA_InitDual3D_FB(void);

/// Inits Nitro Engine Advanced to draw 3D to both screens.
///
/// VRAM banks C and D are used as framebuffers, which means there is only 50%
/// of the normally available VRAM for textures.
///
/// VRAM bank I is used as a pseudo-framebuffer, so that there are three
/// framebuffers in total, which is required to make the output of both screens
/// stable if framerate drops below 60 FPS.
///
/// DMA 2 is used in HBL trigger mode to transfer data from VRAM banks C and D
/// to bank I. Because of this, it is unsafe to use the DMA In GFX FIFO mode to
/// draw models, which has a small performance hit.
///
/// This mode is stable. If the framerate drops below 60 FPS the screens will
/// remain stable.
///
/// In general, prefer NEA_InitDual3D_DMA() over NEA_InitDual3D().
///
/// @return Returns 0 on success.
int NEA_InitDual3D_DMA(void);

/// Inits Nitro Engine Advanced in two-pass FIFO mode.
///
/// This mode doubles the polygon budget by splitting the screen into left and
/// right halves and rendering each half in a separate hardware frame. The
/// effective framerate is 30 FPS.
///
/// VRAM bank D is used for display capture. VRAM banks A, B, and C are
/// available for textures (75% of the normally available VRAM).
///
/// Two framebuffers are allocated in main RAM (~192 KB total). The main screen
/// uses display FIFO mode (MODE_FIFO) with DMA 2 continuously feeding pixel
/// data from main RAM to the display hardware.
///
/// Warning: This mode may show horizontal line artifacts on real hardware due
/// to bus contention between the continuous DMA 2 display transfer and dmaCopy
/// (DMA 3) used to copy captured data. Prefer NEA_Init3D_TwoPass_DMA() or
/// NEA_Init3D_TwoPass_FB() for artifact-free output.
///
/// The sub screen is available for a 2D console via NEA_InitConsole().
///
/// Note: NEA_ClearBMPEnable(), NEA_2DViewInit(), and touch test are not supported
/// in any two-pass mode.
///
/// @return Returns 0 on success.
int NEA_Init3D_TwoPass(void);

/// Inits Nitro Engine Advanced in two-pass framebuffer mode.
///
/// This mode doubles the polygon budget by splitting the screen into left and
/// right halves and rendering each half in a separate hardware frame. The
/// effective framerate is 30 FPS.
///
/// VRAM banks C and D alternate between display capture and main BG display
/// each frame. VRAM banks A and B are available for textures (50% of the
/// normally available VRAM, same as dual 3D modes).
///
/// No main RAM framebuffers are allocated and DMA 2 is not used. This means
/// DMA-based display list drawing (NEA_DL_DMA_GFX_FIFO) is safe and will be
/// used by default, providing better performance than the other two-pass modes.
///
/// The compositing works by alternating the clear color alpha (opaque vs
/// transparent) and BG layer priorities each frame, using the DS 3D hardware's
/// one-frame rendering delay to combine both halves on screen.
///
/// No line artifacts on real hardware.
///
/// The sub screen is available for a 2D console via NEA_InitConsole().
///
/// @return Returns 0 on success.
int NEA_Init3D_TwoPass_FB(void);

/// Inits Nitro Engine Advanced in two-pass HBL DMA mode.
///
/// This mode doubles the polygon budget by splitting the screen into left and
/// right halves and rendering each half in a separate hardware frame. The
/// effective framerate is 30 FPS.
///
/// VRAM bank D is used for display capture. VRAM banks A, B, and C are
/// available for textures (75% of the normally available VRAM). VRAM bank F is
/// used as a one-scanline display buffer.
///
/// Two framebuffers are allocated in main RAM (~192 KB total). DMA 2 is
/// triggered at each HBL to copy one scanline from the main RAM framebuffer to
/// VRAM F, which is displayed as a bitmap BG. Because DMA 2 is only briefly
/// active during each HBL (not continuously), there is no bus contention with
/// dmaCopy (DMA 3).
///
/// No line artifacts on real hardware.
///
/// The sub screen is available for a 2D console via NEA_InitConsole().
///
/// @return Returns 0 on success.
int NEA_Init3D_TwoPass_DMA(void);

/// Draws a 3D scene using two-pass rendering.
///
/// Works with all three two-pass modes (FIFO, FB, DMA). This must be called
/// once per hardware frame (60 FPS loop). Internally, it alternates between
/// rendering the left and right halves of the screen. The user's draw function
/// is called every frame and should draw the full 3D scene.
///
/// @param drawscene Function that draws the screen.
void NEA_ProcessTwoPass(NEA_Voidfunc drawscene);

/// Draws a 3D scene using two-pass rendering, passing an argument.
///
/// Works with all three two-pass modes (FIFO, FB, DMA).
///
/// @param drawscene Function that draws the screen.
/// @param arg Argument to pass to the drawscene function.
void NEA_ProcessTwoPassArg(NEA_VoidArgfunc drawscene, void *arg);

/// Returns the current two-pass frame index.
///
/// Works with all three two-pass modes (FIFO, FB, DMA).
///
/// Returns 0 when the next pass will render the left half (meaning a complete
/// frame has just finished). Returns 1 when the next pass will render the right
/// half. Use this to only update scene state every other frame:
///
/// @code
/// NEA_WaitForVBL(NEA_TwoPassGetPass() == 0 ? NEA_UPDATE_ANIMATIONS : 0);
/// @endcode
///
/// @return Current pass index (0 or 1).
int NEA_TwoPassGetPass(void);

/// Draws 3D scenes in both screens.
///
/// By default, the main screen is the top screen and the sub screen is the
/// bottom screen. This can be changed with NEA_MainScreenSetOnTop(),
/// NEA_MainScreenSetOnBottom() and NEA_SwapScreens(). To check the current
/// position of the main screen, use NEA_MainScreenIsOnTop().
///
/// @param mainscreen Function that draws the main screen.
/// @param subscreen Function that draws the sub screen.
void NEA_ProcessDual(NEA_Voidfunc mainscreen, NEA_Voidfunc subscreen);

/// Draws 3D scenes in both screens and passes arguments to the draw functions.
///
/// By default, the main screen is the top screen and the sub screen is the
/// bottom screen. This can be changed with NEA_MainScreenSetOnTop(),
/// NEA_MainScreenSetOnBottom() and NEA_SwapScreens(). To check the current
/// position of the main screen, use NEA_MainScreenIsOnTop().
///
/// @param mainscreen Function that draws the main screen.
/// @param subscreen Function that draws the sub screen.
/// @param argmain Argument to pass to the mainscreen function.
/// @param argsub Argument to pass to the subscreen function.
void NEA_ProcessDualArg(NEA_VoidArgfunc mainscreen, NEA_VoidArgfunc subscreen,
                       void *argmain, void *argsub);

/// Inits the console of libnds in the main screen.
///
/// It works in dual 3D mode as well, and it uses VRAM_F for the background
/// layer that contains the text.
///
/// Important note: When using safe dual 3D mode, use NEA_InitConsoleSafeDual3D()
/// instead.
void NEA_InitConsole(void);

/// Changes the color of the text of the console.
///
/// @param color New color.
void NEA_SetConsoleColor(u32 color);

/// Set the top screen as main screen.
void NEA_MainScreenSetOnTop(void);

/// Set the bottom screen as main screen.
void NEA_MainScreenSetOnBottom(void);

/// Returns the current main screen.
///
/// @return Returns 1 if the top screen is the main screen, 0 otherwise.
int NEA_MainScreenIsOnTop(void);

/// Swap top and bottom screen.
void NEA_SwapScreens(void);

/// Setup the viewport.
///
/// The viewport is the part of the screen where the 3D scene is drawn.
///
/// NEA_ProcessDual() sets the viewport is set to the full screen.
///
/// After NEA_2DViewInit() the viewport is set to (0, 0, 255, 191).
///
/// NEA_Process() sets the viewport back to the viewport set by NEA_Viewport().
///
/// @param x1 (x1, y1) Bottom left pixel.
/// @param y1 (x1, y1) Bottom left pixel.
/// @param x2 (x2, y2) Top right pixel.
/// @param y2 (x2, y2) Top right pixel.
void NEA_Viewport(int x1, int y1, int x2, int y2);

///  Set cameras field of view.
///
///  @param fovValue FOV [0, ...] in degrees.
void NEA_SetFov(int fovValue);

/// Set near and far clipping planes.
///
/// @param znear Near plane (f32).
/// @param zfar Far plane (f32).
void NEA_ClippingPlanesSetI(int znear, int zfar);

/// Set near and far clipping planes.
///
/// @param n Near plane (float).
/// @param f Far plane (float).
#define NEA_ClippingPlanesSet(n, f) \
    NEA_ClippingPlanesSetI(floattof32(n), floattof32(f))

/// Enable or disable antialiasing.
///
/// @param value true enables antialiasing, false disables it.
void NEA_AntialiasEnable(bool value);

/// Depth buffering modes for the 3D engine.
typedef enum {
    NEA_ZBUFFER = 0,       ///< Z-buffering (linear depth, default)
    NEA_WBUFFER = (1 << 1) ///< W-buffering (reciprocal depth, better for perspective)
} NEA_DepthBufferMode;

/// Set the depth buffering mode.
///
/// Z-buffering is the default. W-buffering provides better depth precision
/// for perspective projection but should not be used with orthographic views.
///
/// @param mode NEA_ZBUFFER or NEA_WBUFFER.
void NEA_SetDepthBufferMode(NEA_DepthBufferMode mode);

/// Returns the number of polygons drawn since the last glFlush().
///
/// @return Returns the number of polygons (0 - 2048).
int NEA_GetPolygonCount(void);

/// Returns the number of vertices drawn since the last glFlush().
///
/// @return Returns the number of vertices (0 - 6144).
int NEA_GetVertexCount(void);

/// Effects supported by NEA_SpecialEffectSet().
typedef enum {
    NEA_NONE,  ///< Disable effects
    NEA_NOISE, ///< Horizontal noise
    NEA_SINE   ///< Horizontal sine waves
} NEA_SpecialEffects;

/// Vertical blank interrupt handler.
///
/// Internal use, must be called every vertical blank.
void NEA_VBLFunc(void);

/// Pause or unpause special effects.
///
/// @param pause true pauses the effect, false unpauses it.
void NEA_SpecialEffectPause(bool pause);

/// Horizontal blank interrupt handler.
///
/// Internal use, must be called every horizontal blank. It's only needed to
/// enable CPU usage measuring and things like special effects enabled by
/// NEA_SpecialEffectSet().
void NEA_HBLFunc(void);

/// Specify which special effect to display in the 3D screens.
///
/// Note: In safe dual 3D mode, DesMuME doesn't emulate the effects correctly.
/// It doesn't seem to emulate capturing the 3D output scrolled by a horizontal
/// offset.
///
/// @param effect One effect out of NEA_NOISE, NEA_SINE or NEA_NONE.
void NEA_SpecialEffectSet(NEA_SpecialEffects effect);

/// Configures the special effect NEA_NOISE.
///
/// If the effect is paused, the values won't be refreshed until it is unpaused.
///
/// @param value The value must be a power of two minus one. The default is 15.
void NEA_SpecialEffectNoiseConfig(int value);

/// Configures the special effect NEA_SINE.
///
/// @param mult Frecuency of the wave. The default value is 10.
/// @param shift Amplitude of the wave. The default value is 9. Bigger values
///              result in smaller waves.
void NEA_SpecialEffectSineConfig(int mult, int shift);

/// Arguments for NEA_WaitForVBL().
typedef enum {
    /// Update the GUI implemented by Nitro Engine Advanced.
    NEA_UPDATE_GUI = BIT(0),
    /// Update all animated models.
    NEA_UPDATE_ANIMATIONS = BIT(1),
    /// Updates the physics engine.
    NEA_UPDATE_PHYSICS = BIT(2),
    /// Allows Nitro Engine Advanced to skip the wait to the vertical blank if CPU load
    /// is greater than 100%. You can use this if you don't need to load
    /// textures or do anything else during the VBL. It is needed to set
    /// NEA_HBLFunc() as a HBL interrupt handler for this flag to work.
    NEA_CAN_SKIP_VBL = BIT(3)
} NEA_UpdateFlags;

/// Waits for the vertical blank and updates the selected systems.
///
/// You should OR all the flags that you need. For example, you can call this
/// function like NEA_WaitForVBL(NEA_UPDATE_GUI | NEA_UPDATE_ANIMATIONS);
///
/// @param flags Look at NEA_UpdateFlags.
void NEA_WaitForVBL(NEA_UpdateFlags flags);

/// Returns the approximate CPU usage in the previous frame.
///
/// You need to set NEA_WaitForVBL() as a VBL interrupt handler and NEA_HBLFunc()
/// as a HBL interrupt handler for the CPU meter to work.
///
/// @return CPU usage (0 - 100).
int NEA_GetCPUPercent(void);

/// Returns true if the GPU is in a rendering period.
///
/// The period when the GPU isn't drawing is when VCOUNT is between 192 and 213.
///
/// During the drawing period you should't load textures. If you try to load
/// textures, there is a moment when the GPU can't access that data, so there
/// will be glitches in the 3D output.
///
/// @return Returns true if the GPU is in the rendering period.
bool NEA_GPUIsRendering(void);

#ifdef NEA_DEBUG

// TODO: Replace sprintf by snprintf

/// TODO: Document
#define NEA_AssertMinMax(min, value, max, format...)         \
    do                                                      \
    {                                                       \
        if (((min) > (value)) || ((max) < (value)))         \
        {                                                   \
            char string[256];                               \
            sprintf(string, "%s:%d:", __func__, __LINE__);  \
            __NEA_debugprint(string);                        \
            sprintf(string, ##format);                      \
            __NEA_debugprint(string);                        \
            __NEA_debugprint("\n");                          \
        }                                                   \
    } while (0)

/// TODO: Document
#define NEA_AssertPointer(ptr, format...)                    \
    do                                                      \
    {                                                       \
        if (!(ptr))                                         \
        {                                                   \
            char string[256];                               \
            sprintf(string, "%s:%d:",  __func__, __LINE__); \
            __NEA_debugprint(string);                        \
            sprintf(string, ##format);                      \
            __NEA_debugprint(string);                        \
            __NEA_debugprint("\n");                          \
        }                                                   \
    } while (0)

/// TODO: Document
#define NEA_Assert(cond, format...)                          \
    do                                                      \
    {                                                       \
        if (!(cond))                                        \
        {                                                   \
            char string[256];                               \
            sprintf(string, "%s:%d:", __func__, __LINE__);  \
            __NEA_debugprint(string);                        \
            sprintf(string, ##format);                      \
            __NEA_debugprint(string);                        \
            __NEA_debugprint("\n");                          \
        }                                                   \
    } while (0)

/// TODO: Document
#define NEA_DebugPrint(format...)                            \
    do                                                      \
    {                                                       \
        char string[256];                                   \
        sprintf(string, "%s:%d:", __func__, __LINE__);      \
        __NEA_debugprint(string);                            \
        sprintf(string, ##format);                          \
        __NEA_debugprint(string);                            \
        __NEA_debugprint("\n");                              \
    } while (0)

/// Function used internally by Nitro Engine Advanced to report error messages.
///
/// It is only used when NEA_DEBUG is defined.
///
/// @param text Text to print.
void __NEA_debugprint(const char *text);

/// Sets a debug handler where Nitro Engine Advanced will send debug information.
///
/// @param fn Handler where Nitro Engine Advanced will send debug information.
void NEA_DebugSetHandler(void (*fn)(const char *));

/// Sets the console of libnds as destination of the debug information.
void NEA_DebugSetHandlerConsole(void);

#else // #ifndef NEA_DEBUG

#define NEA_AssertMinMax(min, value, max, format...) \
    do {                                            \
        (void)(min);                                \
        (void)(value);                              \
        (void)(max);                                \
    } while (0);

#define NEA_AssertPointer(ptr, format...)            \
    do {                                            \
        (void)(ptr);                                \
    } while (0);

#define NEA_Assert(cond, format...)                  \
    do {                                            \
        (void)(cond);                               \
    } while (0);

#define NEA_DebugPrint(format...)

#define NEA_DebugSetHandler(fn)                      \
    do {                                            \
        (void)(fn);                                 \
    } while (0);

#define NEA_DebugSetHandlerConsole()

#endif

/// @}

/// @defgroup touch_test Touch test
///
/// Functions to detect if an object is being pressed by the stylus.
///
/// If you want to know if you are touching something in the touch screen you
/// can use the functions of this group. They perform tests to know if an object
/// is under the coordinates of the stylus, and they return its distance from
/// the camera.
///
/// TODO: There is a bug in gluPickMatrix() when first two coordinates of
/// viewport are different from (0,0).
///
/// Note: This uses last stylus coordinates even if it has been released. You
/// will have to check by yourself if the stylus is really in contact with the
/// screen.
///
/// Note: It two objects overlap, the test may fail to diferenciate which of
/// them is closer to the camera.
///
/// Note: If you want to draw something without using NEA_ModelDraw(), you will
/// have to move the view to the center of your model and use
/// PosTest_Asynch(0, 0, 0).
///
/// This doesn't work in most emulators, but it works in melonDS.
///
/// How to use this test:
///
/// 1. Init the "touch test mode" with NEA_TouchTestStart() to prepare the
///    hardware. During this mode, polygons are not drawn on the screen. For
///    this reason, it is possible to use simplified models (without texture,
///    normals, with fewer polygons, etc) to speed up the process.
///
/// 2. Call NEA_TouchTestObject().
///
/// 3. Draw the model with NEA_ModelDraw(), for example.
///
/// 4. Call NEA_TouchTestResult() to know if it is being touched. If it is being
///    touched, it returns the distance from the camera.
///
/// 5. Repeat 2-4 for each model you want to test.
///
/// 6. Call NEA_TouchTestEnd() to exit "touch test mode".
///
/// Adapted from the Picking example of libnds by Gabe Ghearing.
///
/// @{

/// Starts "touch test mode" and saves all current matrices.
///
/// Polygons drawn after this won't be displayed on screen. It is a good idea to
/// use models with less details than the original to perform this test.
void NEA_TouchTestStart(void);

/// Starts a test for a model.
void NEA_TouchTestObject(void);

/// Gets the result of the touch test.
///
/// @return It returns -1 if the model is NOT being touched. If it is touched,
///         it returns the distance to the camera.
int NEA_TouchTestResult(void);

/// Ends "touch test mode" and restores the previous matrices.
///
/// Polygons drawn after this will be displayed on screen.
void NEA_TouchTestEnd(void);

/// @}

#endif // NEA_GENERAL_H__
