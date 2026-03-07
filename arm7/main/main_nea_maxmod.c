// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Warioware64
//
// This file is part of Nitro Engine Advanced
//
// ARM7 main: libnds services + Maxmod audio + NEA rigid body physics.

#include <nds.h>
#include <maxmod7.h>
#include "nea_rb7.h"

volatile bool exit_loop = false;

void power_button_callback(void)
{
    exit_loop = true;
}

void vblank_handler(void)
{
    inputGetAndSend();
}

int main(int argc, char *argv[])
{
    (void)argc;
    (void)argv;

    enableSound();
    readUserSettings();
    ledBlink(LED_ALWAYS_ON);
    touchInit();

    irqInit();
    fifoInit();

    installSoundFIFO();
    installSystemFIFO();

    // Initialize Maxmod (uses timer 0 internally)
    mmInstall(FIFO_MAXMOD);

    setPowerButtonCB(power_button_callback);
    initClockIRQTimer(LIBNDS_DEFAULT_TIMER_RTC);

    irqSet(IRQ_VBLANK, vblank_handler);
    irqEnable(IRQ_VBLANK);

    // Initialize rigid body physics
    nea_rb7_init();

    while (!exit_loop)
    {
        // Process ARM9 commands
        nea_rb7_listen();

        // Run physics sub-steps
        if (nea_rb7_running)
        {
            for (int i = 0; i < NEA_RB7_SUBSTEPS; i++)
                nea_rb7_update();

            // Sleep check once per frame (not per sub-step)
            nea_rb7_sleep_check_all();

            // Send updated state to ARM9
            nea_rb7_send_state();
        }

        const uint16_t key_mask = KEY_SELECT | KEY_START | KEY_L | KEY_R;
        uint16_t keys_pressed = ~REG_KEYINPUT;
        if ((keys_pressed & key_mask) == key_mask)
            exit_loop = true;

        swiWaitForVBlank();
    }

    return 0;
}
