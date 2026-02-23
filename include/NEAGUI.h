// SPDX-License-Identifier: MIT
//
// Copyright (c) 2008-2022 Antonio Niño Díaz
//
// This file is part of Nitro Engine Advanced

#ifndef NEA_GUI_H__
#define NEA_GUI_H__

#include "NEAMain.h"

/// @file   NEAGUI.h
/// @brief  GUI functions.

/// @defgroup gui_system GUI system
///
/// Functions to create basic buttons, check boxes, radio buttons and slidebars.
///
/// @{

#define NEA_GUI_DEFAULT_OBJECTS  64 ///< Default max number of GUI elements

#define NEA_GUI_POLY_ID          62 ///< Polygon ID to use for most GUI elements.
#define NEA_GUI_POLY_ID_ALT      61 ///< Alternative poly ID to use for the GUI.

#define NEA_GUI_MIN_PRIORITY     100 ///< Minimum 2D priority of GUI elements.

/// GUI element types.
typedef enum {
    NEA_Button = 1,  ///< Button
    NEA_CheckBox,    ///< Check box
    NEA_RadioButton, ///< Radio button
    NEA_SlideBar     ///< Slidebar (horizontal or vertical)
} NEA_GUITypes;

/// Possible states of a GUI object.
typedef enum {
    /// The GUI hasn't been updated or the input hasn't been updated.
    NEA_Error   = -1,
    /// The object isn't pressed.
    NEA_None    = 0,
    /// The object has just been pressed.
    NEA_Pressed = 1,
    /// The object is being held pressed.
    NEA_Held    = 1 << 1,
    /// The object has just been released.
    NEA_Clicked = 1 << 2
} NEA_GUIState;

/// Holds information of an GUI object.
typedef struct {
    NEA_GUITypes type; ///< Type of GUI object
    void *pointer;    ///< Pointer to object-specific information.
} NEA_GUIObj;

/// Updates all GUI objects.
///
/// NEA_UpdateInput() must be called every frame for this function to work.
void NEA_GUIUpdate(void);

/// Draw all GUI objects.
///
/// You must call NEA_2DViewInit() before this.
void NEA_GUIDraw(void);

/// Creates a new button object.
///
/// @param x1 (x1, y1) Top left corner.
/// @param y1 (x1, y1) Top left corner.
/// @param x2 (x2, y2) Bottom right corner.
/// @param y2 (x2, y2) Bottom right corner.
/// @return Returns a pointer to the new object.
NEA_GUIObj *NEA_GUIButtonCreate(s16 x1, s16 y1, s16 x2, s16 y2);

/// Creates a new check button.
///
/// @param x1 (x1, y1) Top left corner.
/// @param y1 (x1, y1) Top left corner.
/// @param x2 (x2, y2) Bottom right corner.
/// @param y2 (x2, y2) Bottom right corner.
/// @param initialvalue If true, the checkbox is created in "checked" state.
/// @return Returns a pointer to the new object.
NEA_GUIObj *NEA_GUICheckBoxCreate(s16 x1, s16 y1, s16 x2, s16 y2,
                                bool initialvalue);

/// Creates a new radio button.
///
/// @param x1 (x1, y1) Top left corner.
/// @param y1 (x1, y1) Top left corner.
/// @param x2 (x2, y2) Bottom right corner.
/// @param y2 (x2, y2) Bottom right corner.
/// @param group Radio button group.
/// @param initialvalue If true, the button is created in "checked" state and
///                     resets all other buttons in same group.
/// @return Returns a pointer to the new object.
NEA_GUIObj *NEA_GUIRadioButtonCreate(s16 x1, s16 y1, s16 x2, s16 y2, int group,
                                   bool initialvalue);

/// Creates a new slidebar.
///
/// It determines wether it is a vertical bar or horizontal bar depending on the
/// size of the bar.
///
/// The size of the sliding button depends on the range of values. It changes
/// sizes between the ranges of 1 to 100. Any range bigger than 100 has the same
/// button size as a range of 100.
///
/// @param x1 (x1, y1) Top left corner.
/// @param y1 (x1, y1) Top left corner.
/// @param x2 (x2, y2) Bottom right corner.
/// @param y2 (x2, y2) Bottom right corner.
/// @param min Minimum value (it can be negative).
/// @param max Maximum value (it can be negative).
/// @param initialvalue Initial value.
/// @return Returns a pointer to the new object.
NEA_GUIObj *NEA_GUISlideBarCreate(s16 x1, s16 y1, s16 x2, s16 y2, int min,
                                int max, int initialvalue);

/// Configures display properties of a button.
///
/// @param btn Button to configure.
/// @param material Material used when not pressed.
/// @param color Color used when not pressed.
/// @param alpha Alpha value used when not pressed.
/// @param pressedmaterial Material used when pressed.
/// @param pressedcolor Color used when pressed.
/// @param pressedalpha Alpha value used when pressed.
void NEA_GUIButtonConfig(NEA_GUIObj *btn, NEA_Material *material, u32 color,
                        u32 alpha, NEA_Material *pressedmaterial,
                        u32 pressedcolor, u32 pressedalpha);

/// Configures display properties of a check box.
///
/// @param chbx Check box to configure.
/// @param materialtrue Material used when the check box is checked.
/// @param materialfalse Material used when the check box not is checked.
/// @param color Color used when not pressed.
/// @param alpha Alpha value used when not pressed.
/// @param pressedcolor Color used when pressed.
/// @param pressedalpha Alpha value used when pressed.
void NEA_GUICheckBoxConfig(NEA_GUIObj *chbx, NEA_Material *materialtrue,
                          NEA_Material *materialfalse, u32 color, u32 alpha,
                          u32 pressedcolor, u32 pressedalpha);

/// Configures display properties of a radio button.
///
/// @param rdbtn Radio button to configure.
/// @param materialtrue Material used when the radio button is checked.
/// @param materialfalse Material used when the radio button is not checked.
/// @param color Color used when not pressed.
/// @param alpha Alpha value used when not pressed.
/// @param pressedcolor Color used when pressed.
/// @param pressedalpha Alpha value used when pressed.
void NEA_GUIRadioButtonConfig(NEA_GUIObj *rdbtn, NEA_Material *materialtrue,
                             NEA_Material *materialfalse, u32 color, u32 alpha,
                             u32 pressedcolor, u32 pressedalpha);

/// Configures display properties of a slide bar.
///
/// @param sldbar Slide bar to configure.
/// @param matbtn Material of the buttons on the sides.
/// @param matbarbtn Material of the sliding button.
/// @param matbar Material of the bar.
/// @param normalcolor Color of buttons when not pressed.
/// @param pressedcolor Color of buttons when pressed.
/// @param barcolor Color of the bar.
/// @param alpha Alpha value of buttons when not pressed.
/// @param pressedalpha Alpha value of buttons when pressed.
/// @param baralpha Alpha value of the bar.
void NEA_GUISlideBarConfig(NEA_GUIObj *sldbar, NEA_Material *matbtn,
                          NEA_Material *matbarbtn, NEA_Material *matbar,
                          u32 normalcolor, u32 pressedcolor, u32 barcolor,
                          u32 alpha, u32 pressedalpha, u32 baralpha);

/// Sets the minimum and maximum values of this slide bar.
///
/// @param sldbr Slide bar to configure.
/// @param min Minimum value (it can be negative).
/// @param max Maximum value (it can be negative).
void NEA_GUISlideBarSetMinMax(NEA_GUIObj *sldbr, int min, int max);

/// Returns the current event of a GUI object.
///
/// It returns a NEA_GUIState value. If the object is a slide is a slide bar it
/// returns (event button A | event button B | event button bar) instead.
///
/// @param obj Pointer to the object.
/// @return Event.
NEA_GUIState NEA_GUIObjectGetEvent(const NEA_GUIObj *obj);

/// Gets the value of a check box.
///
/// @param chbx Check box.
/// @return It returns true if the check box is checked.
bool NEA_GUICheckBoxGetValue(const NEA_GUIObj *chbx);

/// Gets the value of a radio button.
///
/// @param rdbtn Radio button.
/// @return It returns true if the radio button is checked.
bool NEA_GUIRadioButtonGetValue(const NEA_GUIObj *rdbtn);

/// Gets the value of a slide bar.
///
/// @param sldbr Slide bar.
/// @return It returns the value of the slide bar.
int NEA_GUISlideBarGetValue(const NEA_GUIObj *sldbr);

/// Deletes the given object.
///
/// @param obj Object.
void NEA_GUIDeleteObject(NEA_GUIObj *obj);

/// Deletes all GUI objects and all memory used by them.
void NEA_GUIDeleteAll(void);

/// Resets the GUI system and sets the maximun number of objects.
///
/// @param max_objects Max number of objects. If it is lower than 1, the
///                    function will create space for NEA_GUI_DEFAULT_OBJECTS.
/// @return Returns 0 on success.
int NEA_GUISystemReset(int max_objects);

/// Ends GUI system and all memory used by it.
void NEA_GUISystemEnd(void);

/// @}

#endif // NEA_GUI_H__
