// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Antonio Niño Díaz, 2008-2022
//
// This file is part of Nitro Engine Advanced

#include <NEAMain.h>

#include "button.h"
#include "empty.h"
#include "true.h"

void Draw3DScene(void)
{
    NEA_2DViewInit();
    NEA_GUIDraw();
}

int main(int argc, char *argv[])
{
    irqEnable(IRQ_HBLANK);
    irqSet(IRQ_VBLANK, NEA_VBLFunc);
    irqSet(IRQ_HBLANK, NEA_HBLFunc);

    NEA_Init3D();
    // Move 3D screen to lower screen
    NEA_SwapScreens();
    // libnds uses VRAM_C for the text console, reserve A and B only
    NEA_TextureSystemReset(0, 0, NEA_VRAM_AB);
    // Init console in non-3D screen
    consoleDemoInit();

    // Set bg color for 3D screen
    NEA_ClearColorSet(RGB15(15, 15, 15), 31, 63);

    // Load textures
    NEA_Material *ButtonImg, *TrueImg, *EmptyImg;
    NEA_Palette *ButtonPal, *TruePal, *EmptyPal;

    ButtonImg = NEA_MaterialCreate();
    TrueImg = NEA_MaterialCreate();
    EmptyImg = NEA_MaterialCreate();

    ButtonPal = NEA_PaletteCreate();
    TruePal = NEA_PaletteCreate();
    EmptyPal = NEA_PaletteCreate();

    NEA_MaterialTexLoad(ButtonImg, NEA_PAL256, 64, 64,
                       NEA_TEXGEN_TEXCOORD | NEA_TEXTURE_COLOR0_TRANSPARENT,
                       buttonBitmap);
    NEA_PaletteLoad(ButtonPal, buttonPal, 256, NEA_PAL256);
    NEA_MaterialSetPalette(ButtonImg, ButtonPal);

    NEA_MaterialTexLoad(EmptyImg, NEA_PAL256, 64, 64,
                       NEA_TEXGEN_TEXCOORD | NEA_TEXTURE_COLOR0_TRANSPARENT,
                       emptyBitmap);
    NEA_PaletteLoad(EmptyPal, emptyPal, 256, NEA_PAL256);
    NEA_MaterialSetPalette(EmptyImg, EmptyPal);

    NEA_MaterialTexLoad(TrueImg, NEA_PAL256, 64, 64,
                       NEA_TEXGEN_TEXCOORD | NEA_TEXTURE_COLOR0_TRANSPARENT,
                       trueBitmap);
    NEA_PaletteLoad(TruePal, truePal, 256, NEA_PAL256);
    NEA_MaterialSetPalette(TrueImg, TruePal);


    // Create one button
    // -----------------

    NEA_GUIObj *Button = NEA_GUIButtonCreate(116, 16,  // Upper-left pixel
                                           180, 80); // Down-right pixel
    NEA_GUIButtonConfig(Button,
                       // Appearance when pressed (texture, color, alpha)
                       ButtonImg, NEA_White, 31,
                       //Appearance when not pressed
                       ButtonImg, NEA_Blue, 25);

    // Create one check box
    // --------------------

    NEA_GUIObj *ChBx = NEA_GUICheckBoxCreate(16, 16, 80, 80, // Coordinates
                                           true); // Initial value
    NEA_GUICheckBoxConfig(ChBx,
                         TrueImg,  // Texture when value is true
                         EmptyImg, // Texture when value is false
                         // Appearance when pressed (color, alpha)
                         NEA_White, 31,
                         // Appearance when not pressed
                         NEA_Yellow, 15);

    // Create three radio buttons
    // --------------------------

    NEA_GUIObj *RaBtn1 = NEA_GUIRadioButtonCreate(16, 116, 56, 156, // Coordinates
                                                0, // Button group
                                                false); // Initial value

    // Same arguments as check boxes.
    NEA_GUIRadioButtonConfig(RaBtn1,
                            TrueImg, EmptyImg,
                            NEA_White, 31,
                            NEA_Gray, 31);

    NEA_GUIObj *RaBtn2 = NEA_GUIRadioButtonCreate(72, 116, 112, 156,
                                                0,
                                                true);

    // When creating a radio button, if the initial value is true, all
    // other buttons of the same group will be set to false.

    NEA_GUIRadioButtonConfig(RaBtn2, TrueImg, EmptyImg,
                            NEA_White, 31, NEA_Gray, 31);

    NEA_GUIObj *RaBtn3 = NEA_GUIRadioButtonCreate(128, 116, 168, 156,
                                                0, false);
    NEA_GUIRadioButtonConfig(RaBtn3, TrueImg, EmptyImg,
                            NEA_White, 31, NEA_Gray, 31);

    // Create two slide bars
    // ---------------------

    // This function decides if the slide bar is vertical or horizontal based on
    // the size.
    NEA_GUIObj *SldBar1 = NEA_GUISlideBarCreate(255 - 10 - 20, 10, // Coordinates
                                              255 - 10, 192 - 10,
                                              0, 100, // Min. and max. values
                                              50); // Initial value

    NEA_GUISlideBarConfig(SldBar1,
                         EmptyImg, // Buttons' texture
                         EmptyImg, // Sliding button's texture
                         NULL, // Bar texture (NULL = No image...)
                         // Buttons' pressed/not pressed colors.
                         NEA_White, NEA_Yellow,
                         NEA_Black, // Bar color
                         31, 29, // Buttons' pressed/not pressed alpha.
                         15); // Bar alpha

    NEA_GUIObj *SldBar2 = NEA_GUISlideBarCreate(10, 192 - 10 - 20,
                                              255 - 50, 192 - 10,
                                              -30, 20,
                                              0);
    NEA_GUISlideBarConfig(SldBar2,
                         EmptyImg, NULL, NULL,
                         NEA_Green, RGB15(25, 31, 0), NEA_Yellow,
                         31, 20, 31);

    while (1)
    {
        // Update GUI, input and wait for vertical blank. You have to
        // call scanKeys() each frame for this to work.
        NEA_WaitForVBL(NEA_UPDATE_GUI);

        scanKeys(); // This function is needed for the GUI

        printf("\x1b[0;0H");
        printf("Slide bar 1: %d  \n", NEA_GUISlideBarGetValue(SldBar1));
        printf("Slide bar 2: %d  \n", NEA_GUISlideBarGetValue(SldBar2));
        printf("\n");
        printf("Radio button 1: %d \n", NEA_GUIRadioButtonGetValue(RaBtn1));
        printf("Radio button 2: %d \n", NEA_GUIRadioButtonGetValue(RaBtn2));
        printf("Radio button 3: %d \n", NEA_GUIRadioButtonGetValue(RaBtn3));
        printf("\n");
        printf("Check box: %d \n", NEA_GUICheckBoxGetValue(ChBx));
        printf("\n");
        printf("Button event: %d ", NEA_GUIObjectGetEvent(Button));

        // Draw things...
        NEA_Process(Draw3DScene);
    }

    return 0;
}
