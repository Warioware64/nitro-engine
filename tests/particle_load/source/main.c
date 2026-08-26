// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Warioware64, 2026
//
// This file is part of Nitro Engine Advanced

// The NPE loader must refuse a truncated file rather than read past its end.
//
// NEA_ParticleEmitterLoad() walks the file using counts stored inside it -- the
// number of colour keys, then the number of size keys -- and originally took a
// bare pointer with no length, so there was nothing to compare those counts
// against. A short or corrupt .npe read off the end of the allocation.
//
// This feeds the parser one good file and a set of prefixes cut at each stage
// of the parse, and checks that every short one is rejected. It cannot prove
// the absence of an overrun, but it does prove the guard is reached: before the
// fix, every one of these returned success.

#include <stdio.h>

#include <nds.h>

#include <NEAMain.h>

#include "good_bin.h"
#include "cut004_bin.h"
#include "cut015_bin.h"
#include "cut060_bin.h"
#include "cut149_bin.h"
#include "cut151_bin.h"
#include "cut160_bin.h"
#include "cut200_bin.h"

typedef struct {
    const char *name;
    const void *data;
    size_t      size;
    int         expect;   // 1 = must load, 0 = must be refused
} case_t;

int main(int argc, char *argv[])
{
    irqEnable(IRQ_HBLANK);
    irqSet(IRQ_VBLANK, NEA_VBLFunc);
    irqSet(IRQ_HBLANK, NEA_HBLFunc);

    NEA_Init3D();
    NEA_TextureSystemReset(0, 0, NEA_VRAM_AB);
    consoleDemoInit();

    NEA_ParticleSystemReset(4);

    // cut200 is longer than the file, so it *is* the file: a control that the
    // guard rejects short input without rejecting valid input.
    const case_t cases[] = {
        { "good",   good_bin,   good_bin_size,   1 },
        { "cut200", cut200_bin, cut200_bin_size, 1 },
        // A zero length buffer: the guard has to reject before any read.
        { "zero",   cut004_bin, 0,                   0 },
        { "cut004", cut004_bin, cut004_bin_size, 0 },
        { "cut015", cut015_bin, cut015_bin_size, 0 },
        { "cut060", cut060_bin, cut060_bin_size, 0 },
        { "cut149", cut149_bin, cut149_bin_size, 0 },
        { "cut151", cut151_bin, cut151_bin_size, 0 },
        { "cut160", cut160_bin, cut160_bin_size, 0 },
    };

    const int n = (int)(sizeof(cases) / sizeof(cases[0]));
    int failures = 0;

    printf("NPE loader bounds check\n");
    printf("=======================\n\n");

    for (int i = 0; i < n; i++)
    {
        NEA_ParticleEmitter *e = NEA_ParticleEmitterCreate();
        if (e == NULL)
        {
            printf("no emitter slot\n");
            failures++;
            break;
        }

        int got = NEA_ParticleEmitterLoadSize(e, cases[i].data, cases[i].size);

        if (got != cases[i].expect)
        {
            printf("%s: got %d want %d\n", cases[i].name, got,
                   cases[i].expect);
            failures++;
        }

        NEA_ParticleEmitterDelete(e);
    }

    printf("%d cases, %d failures\n\n", n, failures);
    printf("%s\n", failures == 0 ? "PASS" : "FAIL");

    while (1)
        swiWaitForVBlank();

    return 0;
}
