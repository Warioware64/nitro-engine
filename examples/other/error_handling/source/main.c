// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Antonio Niño Díaz, 2008-2024
//
// This file is part of Nitro Engine Advanced

#include <NEAMain.h>

// Don't forget to compile Nitro Engine Advanced with NEA_DEBUG defined or this won't work

void Draw3DScene(void *arg)
{
    (void)arg;

    // Let's generate some error messages...
    NEA_LightOff(100);
    NEA_CameraSetI(NULL,
                  1, 1, 1,
                  1, 1, 1,
                  1, 1, 1);
    NEA_PolyFormat(100, 120, 0, 0, 0);
}

void error_handler(const char *text)
{
    // Simple handler. You could write this to a file instead, for example.
    printf(text);
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

    // Set a custom error handler
    NEA_DebugSetHandler(error_handler);

    // In order to use the default handler again it is needed to call
    // NEA_DebugSetHandlerConsole(). After that, all messages will be printed
    // to the default console

    while (1)
    {
        NEA_WaitForVBL(0);

        NEA_ProcessArg(Draw3DScene, NULL);
    }

    return 0;
}
