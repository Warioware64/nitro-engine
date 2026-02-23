// SPDX-License-Identifier: MIT
//
// Copyright (c) 2008-2022 Antonio Niño Díaz
//
// This file is part of Nitro Engine Advanced

#include "NEAMain.h"

/// @file NEAGUI.c

extern NEA_Input ne_input;

static NEA_GUIObj **NEA_guipointers;
static int NEA_GUI_OBJECTS;
static bool ne_gui_system_inited = false;

typedef struct {
    int x1, y1, x2, y2;
    int event; // 0 = nothing, 1 = just pressed, 2 = held, 3 = just released
    NEA_Material *tex_1, *tex_2;
    // Colors when button isn't pressed / is pressed. The default is white
    u32 color1, color2;
    u32 alpha1, alpha2;
} ne_button_t;

typedef struct {
    int x1, y1, x2, y2;
    int event;
    bool checked;
    NEA_Material *tex_1, *tex_2;
    u32 color1, color2;
    u32 alpha1, alpha2;
} ne_checkbox_t;

typedef struct {
    int x1, y1, x2, y2;
    int event;
    bool checked;
    int group;
    NEA_Material *tex_1, *tex_2;
    u32 color1, color2;
    u32 alpha1, alpha2;
} ne_radiobutton_t;

typedef struct {
    int x1, y1, x2, y2;
    int event_minus, event_plus, event_bar;
    int value;
    int range, desp; //range = max - min; value + desp = real value
    bool isvertical;
    NEA_Material *texbtn, *texbar, *texlong;
    int totalsize, barsize;
    int coord; // helper to avoid some frequent operations
    // 1,2 used for buttons, barcolor used for background of slidebar
    u32 color1, color2, barcolor;
    u32 alpha1, alpha2, baralpha;
} ne_slidebar_t;

// Internal use
static void NEA_ResetRadioButtonGroup(int group)
{
    for (int i = 0; i < NEA_GUI_OBJECTS; i++)
    {
        if (NEA_guipointers[i] == NULL)
            continue;

        if (NEA_guipointers[i]->type != NEA_RadioButton)
            continue;

        ne_radiobutton_t *rabtn = (void *)NEA_guipointers[i]->pointer;

        if (rabtn->group == group)
            rabtn->checked = false;
    }
}

static void NEA_GUIUpdateButton(NEA_GUIObj *obj)
{
    ne_button_t *button = (void *)obj;

    if (button->x1 < ne_input.touch.px && button->x2 > ne_input.touch.px
     && button->y1 < ne_input.touch.py && button->y2 > ne_input.touch.py)
    {
        if (ne_input.kdown & KEY_TOUCH)
        {
            button->event = 1;
        }
        else if ((ne_input.kheld & KEY_TOUCH)
              && (button->event == 1 || button->event == 2))
        {
            button->event = 2;
        }
        else if (ne_input.kup & KEY_TOUCH && button->event == 2)
        {
            button->event = 4;
        }
        else
        {
            button->event = 0;
        }
    }
    else
    {
        button->event = 0;
    }
}

static void NEA_GUIUpdateCheckBox(NEA_GUIObj *obj)
{
    ne_checkbox_t *chbox = (void *)obj;

    if (chbox->x1 < ne_input.touch.px && chbox->x2 > ne_input.touch.px
     && chbox->y1 < ne_input.touch.py && chbox->y2 > ne_input.touch.py)
    {
        if (ne_input.kdown & KEY_TOUCH)
        {
            chbox->event = 1;
        }
        else if ((ne_input.kheld & KEY_TOUCH)
              && (chbox->event == 1 || chbox->event == 2))
        {
            chbox->event = 2;
        }
        else if (ne_input.kup & KEY_TOUCH && chbox->event == 2)
        {
            chbox->event = 4;
            chbox->checked = !chbox->checked;
        }
        else
        {
            chbox->event = 0;
        }
    }
    else
    {
        chbox->event = 0;
    }
}

static void NEA_GUIUpdateRadioButton(NEA_GUIObj *obj)
{
    ne_radiobutton_t *rabtn = (void *)obj;

    if (rabtn->x1 < ne_input.touch.px && rabtn->x2 > ne_input.touch.px
     && rabtn->y1 < ne_input.touch.py && rabtn->y2 > ne_input.touch.py)
    {
        if (ne_input.kdown & KEY_TOUCH)
        {
            rabtn->event = 1;
        }
        else if ((ne_input.kheld & KEY_TOUCH)
              && (rabtn->event == 1 || rabtn->event == 2))
        {
            rabtn->event = 2;
        }
        else if (ne_input.kup & KEY_TOUCH && rabtn->event == 2)
        {
            rabtn->event = 4;
            NEA_ResetRadioButtonGroup(rabtn->group);
            rabtn->checked = true;
        }
        else
        {
            rabtn->event = 0;
        }
    }
    else
    {
        rabtn->event = 0;
    }
}

static void NEA_GUIUpdateSlideBar(NEA_GUIObj *obj)
{
    ne_slidebar_t *sldbar = (void *)obj;

    // Simplify code...
    int x1 = sldbar->x1, x2 = sldbar->x2;
    int y1 = sldbar->y1, y2 = sldbar->y2;
    int px = ne_input.touch.px, py = ne_input.touch.py;
    bool vertical = sldbar->isvertical;
    int coord = sldbar->coord, barsize = sldbar->barsize;
    int tmp1, tmp2; // auxiliary coordinates
    if (sldbar->isvertical)
    {
        tmp1 = y1 + (x2 - x1);
        tmp2 = y2 - (x2 - x1);
    }
    else
    {
        tmp1 = x1 + (y2 - y1);
        tmp2 = x2 - (y2 - y1);
    }

    // Plus button
    // -----------

    bool pluspressed;

    if (vertical)
        pluspressed = x1 < px && x2 > px && tmp2 < py && y2 > py;
    else
        pluspressed = tmp2 < px && x2 > px && y1 < py && y2 > py;

    if (pluspressed)
    {
        if (ne_input.kdown & KEY_TOUCH)
        {
            sldbar->event_plus = 1;
        }
        else if ((ne_input.kheld & KEY_TOUCH)
              && (sldbar->event_plus == 1 || sldbar->event_plus == 2))
        {
            sldbar->event_plus = 2;
            sldbar->value++;
        }
        else if (ne_input.kup & KEY_TOUCH && sldbar->event_plus == 2)
        {
            sldbar->event_plus = 4;
        }
        else
        {
            sldbar->event_plus = 0;
        }
    }
    else
    {
        sldbar->event_plus = 0;
    }

    // Minus button
    // ------------

    bool minuspressed;

    if (vertical)
        minuspressed = x1 < px && x2 > px && y1 < py && tmp1 > py;
    else
        minuspressed = x1 < px && tmp1 > px && y1 < py && y2 > py;

    if (minuspressed)
    {
        if (ne_input.kdown & KEY_TOUCH)
        {
            sldbar->event_minus = 1;
        }
        else if ((ne_input.kheld & KEY_TOUCH)
              && (sldbar->event_minus == 1 || sldbar->event_minus == 2))
        {
            sldbar->event_minus = 2;
            sldbar->value--;
        }
        else if ((ne_input.kup & KEY_TOUCH) && (sldbar->event_minus == 2))
        {
            sldbar->event_minus = 4;
        }
        else
        {
            sldbar->event_minus = 0;
        }
    }
    else
    {
        sldbar->event_minus = 0;
    }

    // Bar button
    // ----------

    if (sldbar->event_bar == 2)
    {
        int tmp = ((vertical) ? py : px) - tmp1 - (barsize >> 1);
        tmp *= sldbar->range;
        tmp = divf32(tmp << 12, (tmp2 - tmp1 - barsize) << 12) >> 12;
        sldbar->value = tmp;
    }

    sldbar->value = (sldbar->value > sldbar->range) ? sldbar->range : sldbar->value;
    sldbar->value = (sldbar->value < 0) ? 0 : sldbar->value;

    sldbar->coord = (sldbar->totalsize - barsize) * sldbar->value;
    sldbar->coord = divf32(sldbar->coord << 12, sldbar->range << 12) >> 12;
    sldbar->coord += (vertical) ? y1 + (x2 - x1) : x1 + (y2 - y1);
    coord = sldbar->coord;

    bool barpressed;

    if (vertical)
        barpressed = x1 < px && x2 > px && coord < py && (coord + barsize) > py;
    else
        barpressed = y1 < py && y2 > py && coord < px && (coord + barsize) > px;

    if (barpressed)
    {
        if (ne_input.kdown & KEY_TOUCH)
        {
            sldbar->event_bar = 1;
        }
        else if ((ne_input.kheld & KEY_TOUCH)
              && (sldbar->event_bar == 1 || sldbar->event_bar == 2))
        {
            sldbar->event_bar = 2;
        }
        else if (ne_input.kup & KEY_TOUCH && sldbar->event_bar == 2)
        {
            sldbar->event_bar = 4;
        }
        else
        {
            sldbar->event_bar = 0;
        }
    }
    else
    {
        sldbar->event_bar = 0;
    }
}

void NEA_GUIUpdate(void)
{
    if (!ne_gui_system_inited)
        return;

    for (int i = 0; i < NEA_GUI_OBJECTS; i++)
    {
        if (NEA_guipointers[i] == NULL)
            continue;

        NEA_GUITypes type = NEA_guipointers[i]->type;

        if (type == NEA_Button)
            NEA_GUIUpdateButton(NEA_guipointers[i]->pointer);
        else if (type == NEA_CheckBox)
            NEA_GUIUpdateCheckBox(NEA_guipointers[i]->pointer);
        else if (type == NEA_RadioButton)
            NEA_GUIUpdateRadioButton(NEA_guipointers[i]->pointer);
        else if (type == NEA_SlideBar)
            NEA_GUIUpdateSlideBar(NEA_guipointers[i]->pointer);
        else
            NEA_DebugPrint("Unknown GUI object type: %d", type);
    }
}

static void NEA_GUIDrawButton(NEA_GUIObj *obj, int priority)
{
    ne_button_t *button = (void *)obj;
    NEA_Material *tex;
    u32 color;

    if (button->event > 0)
    {
        // Pressed
        GFX_POLY_FORMAT = POLY_ALPHA(button->alpha2)
                        | POLY_ID(NEA_GUI_POLY_ID) | NEA_CULL_NONE;
        tex = button->tex_2;
        color = button->color2;
    }
    else
    {
        // Not-pressed
        GFX_POLY_FORMAT = POLY_ALPHA(button->alpha1)
                        | POLY_ID(NEA_GUI_POLY_ID) | NEA_CULL_NONE;
        tex = button->tex_1;
        color = button->color1;
    }

    if (tex == NULL)
    {
        NEA_2DDrawQuad(button->x1, button->y1, button->x2, button->y2,
                      priority, color);
    }
    else
    {
        NEA_2DDrawTexturedQuadColor(button->x1, button->y1, button->x2,
                                   button->y2, priority, tex, color);
    }
}

static void NEA_GUIDrawCheckBox(NEA_GUIObj *obj, int priority)
{
    ne_checkbox_t *chbox = (void *)obj;
    u32 color;

    if (chbox->event > 0)
    {
        GFX_POLY_FORMAT = POLY_ALPHA(chbox->alpha2)
                        | POLY_ID(NEA_GUI_POLY_ID) | NEA_CULL_NONE;
        color = chbox->color2;
    }
    else
    {
        GFX_POLY_FORMAT = POLY_ALPHA(chbox->alpha1)
                        | POLY_ID(NEA_GUI_POLY_ID) | NEA_CULL_NONE;
        color = chbox->color1;
    }

    NEA_Material *tex = chbox->checked ? chbox->tex_2 : chbox->tex_1;

    if (tex == NULL)
    {
        NEA_2DDrawQuad(chbox->x1, chbox->y1, chbox->x2, chbox->y2,
                      priority, color);
    }
    else
    {
        NEA_2DDrawTexturedQuadColor(chbox->x1, chbox->y1, chbox->x2,
                                   chbox->y2, priority, tex, color);
    }
}

static void NEA_GUIDrawRadioButton(NEA_GUIObj *obj, int priority)
{
    ne_radiobutton_t *rabtn = (void *)obj;
    u32 color;

    if (rabtn->event > 0)
    {
        GFX_POLY_FORMAT = POLY_ALPHA(rabtn->alpha2)
                        | POLY_ID(NEA_GUI_POLY_ID) | NEA_CULL_NONE;
        color = rabtn->color2;
    }
    else
    {
        GFX_POLY_FORMAT = POLY_ALPHA(rabtn->alpha1)
                        | POLY_ID(NEA_GUI_POLY_ID) | NEA_CULL_NONE;
        color = rabtn->color1;
    }

    NEA_Material *tex = rabtn->checked ?  rabtn->tex_2 : rabtn->tex_1;

    if (tex == NULL)
    {
        NEA_2DDrawQuad(rabtn->x1, rabtn->y1, rabtn->x2, rabtn->y2,
                  priority, color);
    }
    else
    {
        NEA_2DDrawTexturedQuadColor(rabtn->x1, rabtn->y1, rabtn->x2,
                       rabtn->y2, priority, tex, color);
    }
}

static void NEA_GUIDrawSlideBar(NEA_GUIObj *obj, int priority)
{
    ne_slidebar_t *sldbar = (void *)obj;
    u32 color;

    // Helper variables

    int x1 = sldbar->x1, x2 = sldbar->x2;
    int y1 = sldbar->y1, y2 = sldbar->y2;
    int tmp1, tmp2; // Auxiliary coordinates
    if (sldbar->isvertical)
    {
        tmp1 = y1 + (x2 - x1);
        tmp2 = y2 - (x2 - x1);
    }
    else
    {
        tmp1 = x1 + (y2 - y1);
        tmp2 = x2 - (y2 - y1);
    }

    // Load texture of the two buttons
    NEA_Material *tex = sldbar->texbtn;

    // Plus button
    // -----------

    if (sldbar->event_plus > 0)
    {
        GFX_POLY_FORMAT = POLY_ALPHA(sldbar->alpha2)
                        | POLY_ID(NEA_GUI_POLY_ID) | NEA_CULL_NONE;
        color = sldbar->color2;
    }
    else
    {
        GFX_POLY_FORMAT = POLY_ALPHA(sldbar->alpha1)
                        | POLY_ID(NEA_GUI_POLY_ID) | NEA_CULL_NONE;
        color = sldbar->color1;
    }

    if (sldbar->isvertical)
    {
        if (tex == NULL)
            NEA_2DDrawQuad(x1, tmp2, x2, y2, priority, color);
        else
            NEA_2DDrawTexturedQuadColor(x1, tmp2, x2, y2, priority, tex, color);
    }
    else
    {
        if (tex == NULL)
            NEA_2DDrawQuad(tmp2, y1, x2, y2, priority, color);
        else
            NEA_2DDrawTexturedQuadColor(tmp2, y1, x2, y2, priority, tex, color);
    }

    // Minus button
    // ------------

    if (sldbar->event_minus > 0)
    {
        GFX_POLY_FORMAT = POLY_ALPHA(sldbar->alpha2)
                        | POLY_ID(NEA_GUI_POLY_ID) | NEA_CULL_NONE;
        color = sldbar->color2;
    }
    else
    {
        GFX_POLY_FORMAT = POLY_ALPHA(sldbar->alpha1)
                        | POLY_ID(NEA_GUI_POLY_ID) | NEA_CULL_NONE;
        color = sldbar->color1;
    }

    if (sldbar->isvertical)
    {
        if (tex == NULL)
            NEA_2DDrawQuad(x1, y1, x2, tmp1, priority, color);
        else
            NEA_2DDrawTexturedQuadColor(x1, y1, x2, tmp1, priority, tex, color);
    }
    else
    {
        if (tex == NULL)
            NEA_2DDrawQuad(x1, y1, tmp1, y2, priority, color);
        else
            NEA_2DDrawTexturedQuadColor(x1, y1, tmp1, y2, priority, tex, color);
    }

    // Bar button
    // ----------

    // Load texture of the bar button
    tex = sldbar->texbar;

    if (sldbar->event_bar > 0)
    {
        GFX_POLY_FORMAT = POLY_ALPHA(sldbar->alpha2)
                        | POLY_ID(NEA_GUI_POLY_ID) | NEA_CULL_NONE;
        color = sldbar->color2;
    }
    else
    {
        GFX_POLY_FORMAT = POLY_ALPHA(sldbar->alpha1)
                        | POLY_ID(NEA_GUI_POLY_ID) | NEA_CULL_NONE;
        color = sldbar->color1;
    }

    if (sldbar->isvertical)
    {
        if (tex == NULL)
        {
            NEA_2DDrawQuad(x1, sldbar->coord,
                          x2, sldbar->coord + sldbar->barsize,
                          priority, color);
        }
        else
        {
            NEA_2DDrawTexturedQuadColor(x1, sldbar->coord,
                                       x2, sldbar->coord + sldbar->barsize,
                                       priority, tex, color);
        }
    }
    else
    {
        if (tex == NULL)
        {
            NEA_2DDrawQuad(sldbar->coord, y1,
                          sldbar->coord + sldbar->barsize, y2,
                          priority, color);
        }
        else
        {
            NEA_2DDrawTexturedQuadColor(sldbar->coord, y1,
                                       sldbar->coord + sldbar->barsize, y2,
                                       priority, tex, color);
        }
    }

    // Slide bar
    // ---------

    // Load texture and color of the slide bar background
    tex = sldbar->texlong;
    color = sldbar->barcolor;
    GFX_POLY_FORMAT = POLY_ALPHA(sldbar->baralpha)
                    | POLY_ID(NEA_GUI_POLY_ID_ALT) | NEA_CULL_NONE;

    // Now we need to use `priority + 1` as priority. The bar button must
    // be in front of bar. `priority + 1` is less priority than `priority`.

    if (sldbar->isvertical)
    {
        if (tex == NULL)
        {
            NEA_2DDrawQuad(x1, tmp1, x2, tmp2, priority + 1, color);
        }
        else
        {
            NEA_2DDrawTexturedQuadColor(x1, tmp1, x2, tmp2,
                                       priority + 1, tex, color);
        }
    }
    else
    {
        if (tex == NULL)
        {
            NEA_2DDrawQuad(tmp1, y1, tmp2, y2, priority + 1, color);
        }
        else
        {
            NEA_2DDrawTexturedQuadColor(tmp1, y1, tmp2, y2,
                                       priority + 1, tex, color);
        }
    }
}

void NEA_GUIDraw(void)
{
    if (!ne_gui_system_inited)
        return;

    for (int i = 0; i < NEA_GUI_OBJECTS; i++)
    {
        if (NEA_guipointers[i] == NULL)
            continue;

        NEA_GUIObj *obj = NEA_guipointers[i]->pointer;
        NEA_GUITypes type = NEA_guipointers[i]->type;
        int priority = i + NEA_GUI_MIN_PRIORITY;

        if (type == NEA_Button)
            NEA_GUIDrawButton(obj, priority);
        else if (type == NEA_CheckBox)
            NEA_GUIDrawCheckBox(obj, priority);
        else if (type == NEA_RadioButton)
            NEA_GUIDrawRadioButton(obj, priority);
        else if (type == NEA_SlideBar)
            NEA_GUIDrawSlideBar(obj, priority);
        else
            NEA_DebugPrint("Unknown GUI object type: %d", type);
    }
}

NEA_GUIObj *NEA_GUIButtonCreate(s16 x1, s16 y1, s16 x2, s16 y2)
{
    if (!ne_gui_system_inited)
    {
        NEA_DebugPrint("System not initialized");
        return NULL;
    }

    for (int i = 0; i < NEA_GUI_OBJECTS; i++)
    {
        if (NEA_guipointers[i] != NULL)
            continue;

        ne_button_t *ptr = malloc(sizeof(ne_button_t));
        if (ptr == NULL)
        {
            NEA_DebugPrint("Not enough memory");
            return NULL;
        }

        NEA_guipointers[i] = malloc(sizeof(NEA_GUIObj));
        if (NEA_guipointers[i] == NULL)
        {
            free(ptr);
            NEA_DebugPrint("Not enough memory");
            return NULL;
        }

        NEA_guipointers[i]->pointer = (void *)ptr;
        NEA_guipointers[i]->type = NEA_Button;

        ptr->x1 = x1;
        ptr->y1 = y1;
        ptr->x2 = x2;
        ptr->y2 = y2;
        ptr->event = -1;
        ptr->tex_1 = ptr->tex_2 = NULL;
        ptr->color1 = ptr->color2 = NEA_White;
        ptr->alpha1 = ptr->alpha2 = 31;

        return NEA_guipointers[i];
    }

    NEA_DebugPrint("No free slots");

    return NULL;
}

NEA_GUIObj *NEA_GUICheckBoxCreate(s16 x1, s16 y1, s16 x2, s16 y2, bool initialvalue)
{
    if (!ne_gui_system_inited)
    {
        NEA_DebugPrint("System not initialized");
        return NULL;
    }

    for (int i = 0; i < NEA_GUI_OBJECTS; i++)
    {
        if (NEA_guipointers[i] != NULL)
            continue;

        ne_checkbox_t *ptr = malloc(sizeof(ne_checkbox_t));
        if (ptr == NULL)
        {
            NEA_DebugPrint("Not enough memory");
            return NULL;
        }

        NEA_guipointers[i] = malloc(sizeof(NEA_GUIObj));
        if (NEA_guipointers[i] == NULL)
        {
            free(ptr);
            NEA_DebugPrint("Not enough memory");
            return NULL;
        }

        NEA_guipointers[i]->pointer = (void *)ptr;
        NEA_guipointers[i]->type = NEA_CheckBox;

        ptr->x1 = x1;
        ptr->y1 = y1;
        ptr->x2 = x2;
        ptr->y2 = y2;
        ptr->event = -1;
        ptr->tex_1 = ptr->tex_2 = NULL;
        ptr->color1 = ptr->color2 = NEA_White;
        ptr->alpha1 = ptr->alpha2 = 31;
        ptr->checked = initialvalue;

        return NEA_guipointers[i];
    }

    NEA_DebugPrint("No free slots");

    return NULL;
}

NEA_GUIObj *NEA_GUIRadioButtonCreate(s16 x1, s16 y1, s16 x2, s16 y2, int group,
                                   bool initialvalue)
{
    if (!ne_gui_system_inited)
    {
        NEA_DebugPrint("System not initialized");
        return NULL;
    }

    for (int i = 0; i < NEA_GUI_OBJECTS; i++)
    {
        if (NEA_guipointers[i] != NULL)
            continue;

        ne_radiobutton_t *ptr = malloc(sizeof(ne_radiobutton_t));
        if (ptr == NULL)
        {
            NEA_DebugPrint("Not enough memory");
            return NULL;
        }

        NEA_guipointers[i] = malloc(sizeof(NEA_GUIObj));
        if (NEA_guipointers[i] == NULL)
        {
            free(ptr);
            NEA_DebugPrint("Not enough memory");
            return NULL;
        }

        NEA_guipointers[i]->pointer = (void *)ptr;
        NEA_guipointers[i]->type = NEA_RadioButton;

        ptr->x1 = x1;
        ptr->y1 = y1;
        ptr->x2 = x2;
        ptr->y2 = y2;
        ptr->event = -1;
        ptr->tex_1 = ptr->tex_2 = NULL;
        ptr->color1 = ptr->color2 = NEA_White;
        ptr->alpha1 = ptr->alpha2 = 31;
        ptr->group = group;

        if (initialvalue)
            NEA_ResetRadioButtonGroup(group);

        ptr->checked = initialvalue;
        return NEA_guipointers[i];
    }

    NEA_DebugPrint("No free slots");

    return NULL;
}

NEA_GUIObj *NEA_GUISlideBarCreate(s16 x1, s16 y1, s16 x2, s16 y2, int min,
                                int max, int initialvalue)
{
    if (!ne_gui_system_inited)
    {
        NEA_DebugPrint("System not initialized");
        return NULL;
    }

    for (int i = 0; i < NEA_GUI_OBJECTS; i++)
    {
        if (NEA_guipointers[i] != NULL)
            continue;

        ne_slidebar_t *ptr = malloc(sizeof(ne_slidebar_t));
        if (ptr == NULL)
        {
            NEA_DebugPrint("Not enough memory");
            return NULL;
        }

        NEA_guipointers[i] = malloc(sizeof(NEA_GUIObj));
        if (NEA_guipointers[i] == NULL)
        {
            free(ptr);
            NEA_DebugPrint("Not enough memory");
            return NULL;
        }

        NEA_guipointers[i]->pointer = (void *)ptr;
        NEA_guipointers[i]->type = NEA_SlideBar;

        ptr->x1 = x1;
        ptr->y1 = y1;
        ptr->x2 = x2;
        ptr->y2 = y2;
        ptr->event_minus = ptr->event_plus = ptr->event_bar = -1;
        ptr->texbtn = ptr->texbar = ptr->texlong = NULL;
        ptr->color1 = ptr->color2 = ptr->barcolor = NEA_White;
        ptr->alpha1 = ptr->alpha2 = ptr->baralpha = 31;
        ptr->value = initialvalue - min;
        ptr->desp = min;
        ptr->range = max - min;

        ptr->isvertical = (x2 - x1 > y2 - y1) ? false : true;

        if (ptr->isvertical)
            ptr->totalsize = y2 - y1 - ((x2 - x1) << 1);
        else
            ptr->totalsize = x2 - x1 - ((y2 - y1) << 1);

        ptr->barsize = 100 - ptr->range;
        ptr->barsize = (ptr->barsize < 20) ?  (20 << 12) : (ptr->barsize << 12);
        ptr->barsize = (divf32(ptr->barsize, 100 << 12) * ptr->totalsize) >> 12;

        ptr->coord = (ptr->totalsize - ptr->barsize) * ptr->value;
        ptr->coord = divf32(ptr->coord << 12, ptr->range << 12) >> 12;
        ptr->coord += (ptr->isvertical) ?
                            ptr->y1 + (ptr->x2 - ptr->x1) :
                            ptr->x1 + (ptr->y2 - ptr->y1);

        return NEA_guipointers[i];
    }

    NEA_DebugPrint("No free slots");

    return NULL;
}

void NEA_GUIButtonConfig(NEA_GUIObj *btn, NEA_Material *material, u32 color,
                        u32 alpha, NEA_Material *pressedmaterial,
                        u32 pressedcolor, u32 pressedalpha)
{
    NEA_AssertPointer(btn, "NULL pointer");
    NEA_Assert(btn->type == NEA_Button, "Not a button");

    ne_button_t *button = btn->pointer;

    button->tex_1 = material;
    button->tex_2 = pressedmaterial;
    button->color1 = color;
    button->color2 = pressedcolor;
    button->alpha1 = alpha;
    button->alpha2 = pressedalpha;
}

void NEA_GUICheckBoxConfig(NEA_GUIObj *chbx, NEA_Material *materialtrue,
                          NEA_Material *materialfalse, u32 color, u32 alpha,
                          u32 pressedcolor, u32 pressedalpha)
{
    NEA_AssertPointer(chbx, "NULL pointer");
    NEA_Assert(chbx->type == NEA_CheckBox, "Not a check box");

    ne_checkbox_t *checkbox = chbx->pointer;

    checkbox->tex_1 = materialfalse;
    checkbox->tex_2 = materialtrue;
    checkbox->color1 = color;
    checkbox->color2 = pressedcolor;
    checkbox->alpha1 = alpha;
    checkbox->alpha2 = pressedalpha;
}

void NEA_GUIRadioButtonConfig(NEA_GUIObj *rdbtn, NEA_Material *materialtrue,
                             NEA_Material *materialfalse, u32 color, u32 alpha,
                             u32 pressedcolor, u32 pressedalpha)
{
    NEA_AssertPointer(rdbtn, "NULL pointer");
    NEA_Assert(rdbtn->type == NEA_RadioButton, "Not a radio button");

    ne_radiobutton_t *radiobutton = rdbtn->pointer;

    radiobutton->tex_1 = materialfalse;
    radiobutton->tex_2 = materialtrue;
    radiobutton->color1 = color;
    radiobutton->color2 = pressedcolor;
    radiobutton->alpha1 = alpha;
    radiobutton->alpha2 = pressedalpha;
}

void NEA_GUISlideBarConfig(NEA_GUIObj *sldbar, NEA_Material *matbtn,
                          NEA_Material *matbarbtn, NEA_Material *matbar,
                          u32 normalcolor, u32 pressedcolor, u32 barcolor,
                          u32 alpha, u32 pressedalpha, u32 baralpha)
{
    NEA_AssertPointer(sldbar, "NULL pointer");
    NEA_Assert(sldbar->type == NEA_SlideBar, "Not a slide bar");

    ne_slidebar_t *slidebar = sldbar->pointer;

    slidebar->texbtn = matbtn;
    slidebar->texbar = matbarbtn;
    slidebar->texlong = matbar;
    slidebar->color1 = normalcolor;
    slidebar->color2 = pressedcolor;
    slidebar->barcolor = barcolor;
    slidebar->alpha1 = alpha;
    slidebar->alpha2 = pressedalpha;
    slidebar->baralpha = baralpha;
}

void NEA_GUISlideBarSetMinMax(NEA_GUIObj *sldbr, int min, int max)
{
    NEA_AssertPointer(sldbr, "NULL pointer");
    NEA_Assert(sldbr->type == NEA_SlideBar, "Not a slide bar");

    ne_slidebar_t *slidebar = sldbr->pointer;

    slidebar->desp = min;
    slidebar->range = max - min;

    // Bar size
    slidebar->barsize = 100 - slidebar->range;
    slidebar->barsize = (slidebar->barsize < 20) ?
                        (20 << 12) : (slidebar->barsize << 12);
    slidebar->barsize =
        (divf32(slidebar->barsize, 100 << 12) * slidebar->totalsize) >> 12;

    // Current coordinate
    slidebar->coord =
        (slidebar->totalsize - slidebar->barsize) * slidebar->value;
    slidebar->coord =
         divf32(slidebar->coord << 12, slidebar->range << 12) >> 12;
    slidebar->coord += (slidebar->isvertical) ?
        slidebar->y1 + (slidebar->x2 - slidebar->x1) :
        slidebar->x1 + (slidebar->y2 - slidebar->y1);
}

NEA_GUIState NEA_GUIObjectGetEvent(const NEA_GUIObj *obj)
{
    ne_slidebar_t *ptr;

    NEA_AssertPointer(obj, "NULL pointer");

    switch (obj->type)
    {
        case NEA_Button:
            return ((ne_button_t *) (obj->pointer))->event;
        case NEA_CheckBox:
            return ((ne_checkbox_t *) (obj->pointer))->event;
        case NEA_RadioButton:
            return ((ne_radiobutton_t *) (obj->pointer))->event;
        case NEA_SlideBar:
            ptr = obj->pointer;
            return ptr->event_plus | ptr->event_minus | ptr->event_bar;
        default:
            NEA_DebugPrint("Unknown GUI object type: %d", obj->type);
            return -1;
    }
}

bool NEA_GUICheckBoxGetValue(const NEA_GUIObj *chbx)
{
    NEA_AssertPointer(chbx, "NULL pointer");
    NEA_Assert(chbx->type == NEA_CheckBox, "Not a check box");
    return ((ne_checkbox_t *)(chbx->pointer))->checked;
}

bool NEA_GUIRadioButtonGetValue(const NEA_GUIObj *rdbtn)
{
    NEA_AssertPointer(rdbtn, "NULL pointer");
    NEA_Assert(rdbtn->type == NEA_RadioButton, "Not a radio button");
    return ((ne_radiobutton_t *)(rdbtn->pointer))->checked;
}

int NEA_GUISlideBarGetValue(const NEA_GUIObj *sldbr)
{
    NEA_AssertPointer(sldbr, "NULL pointer");
    NEA_Assert(sldbr->type == NEA_SlideBar, "Not a slide bar");
    ne_slidebar_t *slidebar = sldbr->pointer;
    return slidebar->value + slidebar->desp;
}

void NEA_GUIDeleteObject(NEA_GUIObj *obj)
{
    NEA_AssertPointer(obj, "NULL pointer");

    for (int i = 0; i < NEA_GUI_OBJECTS; i++)
    {
        if (NEA_guipointers[i] == obj)
        {
            free(obj->pointer);
            free(obj);
            NEA_guipointers[i] = NULL;
            return;
        }
    }

    NEA_DebugPrint("Object not found");
}

void NEA_GUIDeleteAll(void)
{
    if (!ne_gui_system_inited)
        return;

    for (int i = 0; i < NEA_GUI_OBJECTS; i++)
    {
        if (NEA_guipointers[i])
        {
            free(NEA_guipointers[i]->pointer);
            free(NEA_guipointers[i]);
            NEA_guipointers[i] = NULL;
        }
    }
}

int NEA_GUISystemReset(int max_objects)
{
    if (ne_gui_system_inited)
        NEA_GUISystemEnd();

    if (max_objects < 1)
        NEA_GUI_OBJECTS = NEA_GUI_DEFAULT_OBJECTS;
    else
        NEA_GUI_OBJECTS = max_objects;

    NEA_guipointers = calloc(NEA_GUI_OBJECTS, sizeof(NEA_guipointers));
    if (NEA_guipointers == NULL)
    {
        NEA_DebugPrint("Not enough memory");
        return -1;
    }

    ne_gui_system_inited = true;
    return 0;
}

void NEA_GUISystemEnd(void)
{
    if (!ne_gui_system_inited)
        return;

    NEA_GUIDeleteAll();

    free(NEA_guipointers);

    ne_gui_system_inited = false;
}
