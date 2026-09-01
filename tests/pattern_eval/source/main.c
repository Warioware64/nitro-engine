// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Warioware64, 2026
//
// This file is part of Nitro Engine Advanced
//
// Checks NEAPattern.c against tools/pattern_editor/pattern_format.py.
//
// The editor previews a bank by running the Python evaluator. That preview is
// only worth anything if it produces the integers the ARM9 produces, so this
// runs every gesture in vectors.h through the real runtime, at every
// algorithm, every resampling method and every threshold the generator swept,
// and compares the reduced point set and the whole ranked result list against
// what Python said. A mismatch names the case.

#include <NEAMain.h>

#include "bank_bin.h"
#include "vectors.h"

static NEA_PatternBank *bank;
static NEA_PatternStrokes *ink;
static NEA_PatternRecognizer *rec;

static int checks;
static int failures;
static int reported;

static void fail(const ptn_case *c, const char *what, int got, int expected)
{
    failures++;
    if (reported >= 6)
        return;
    reported++;

    printf("\x1b[31m%s\n", ptn_inputs[c->input].name);
    printf(" a%d m%d t%d k%lx\n", c->algo, c->method, c->threshold,
           (unsigned long)c->mask);
    printf(" %s: %d != %d\x1b[39m\n", what, got, expected);
}

static void load_ink(const ptn_case *c)
{
    const ptn_input *in = &ptn_inputs[c->input];

    NEA_PatternStrokesClear(ink);
    for (int i = 0; i < in->count; i++)
    {
        int x = in->points[i * 2];
        int y = in->points[i * 2 + 1];
        if (x == NEA_PATTERN_PEN_UP_X)
            NEA_PatternStrokesAppendPenUp(ink);
        else
            NEA_PatternStrokesAppendPoint(ink, x, y);
    }
}

static bool check_reduced(const ptn_case *c)
{
    const ptn_reduced *want = &ptn_reduceds[c->reduced];
    const NEA_PatternPoint *got = NULL;
    int n = NEA_PatternRecognizerGetInputPoints(rec, &got);

    checks++;
    if (n != want->count)
    {
        fail(c, "reduced n", n, want->count);
        return false;
    }

    for (int i = 0; i < n; i++)
    {
        if (got[i].x != want->points[i * 2])
        {
            fail(c, "reduced x", got[i].x, want->points[i * 2]);
            return false;
        }
        if (got[i].y != want->points[i * 2 + 1])
        {
            fail(c, "reduced y", got[i].y, want->points[i * 2 + 1]);
            return false;
        }
    }
    return true;
}

static void run_case(const ptn_case *c)
{
    NEA_PatternRecognizerSetAlgorithm(rec, (NEA_PatternAlgorithm)c->algo);
    NEA_PatternRecognizerSetResample(rec, (NEA_PatternResampleMethod)c->method,
                                     c->threshold);

    load_ink(c);

    NEA_PatternResult results[PTN_MAX_RESULTS];
    int n = NEA_PatternRecognize(rec, bank, ink, c->mask, results,
                                 PTN_MAX_RESULTS);

    check_reduced(c);

    checks++;
    if (n != c->nresults)
    {
        fail(c, "nresults", n, c->nresults);
        return;
    }

    for (int i = 0; i < n; i++)
    {
        checks += 3;
        if (results[i].entry != c->results[i * 3])
        {
            fail(c, "entry", results[i].entry, c->results[i * 3]);
            return;
        }
        if (results[i].code != c->results[i * 3 + 1])
        {
            fail(c, "code", results[i].code, c->results[i * 3 + 1]);
            return;
        }
        if (results[i].score != c->results[i * 3 + 2])
        {
            fail(c, "score", results[i].score, c->results[i * 3 + 2]);
            return;
        }
    }
}

int main(int argc, char *argv[])
{
    irqEnable(IRQ_HBLANK);
    irqSet(IRQ_VBLANK, NEA_VBLFunc);
    irqSet(IRQ_HBLANK, NEA_HBLFunc);

    NEA_Init3D();
    NEA_TextureSystemReset(0, 0, NEA_VRAM_AB);
    consoleDemoInit();

    printf("NEAPattern cross-check\n\n");

    if (NEA_PatternSystemReset(0) != 0)
    {
        printf("\x1b[31msystem reset failed\x1b[39m\n");
        goto done;
    }

    bank = NEA_PatternBankLoad(bank_bin);
    if (bank == NULL)
    {
        printf("\x1b[31mbank failed to load\x1b[39m\n");
        goto done;
    }

    if (NEA_PatternBankGetNormalizeSize(bank) != PTN_NORMALIZE_SIZE)
    {
        printf("\x1b[31mwrong normalize size\x1b[39m\n");
        goto done;
    }

    ink = NEA_PatternStrokesCreate(256);
    rec = NEA_PatternRecognizerCreate(PTN_MAX_POINTS, PTN_MAX_STROKES);
    if (ink == NULL || rec == NULL)
    {
        printf("\x1b[31mout of memory\x1b[39m\n");
        goto done;
    }

    // Fine's tables are allocated on demand, so ask for them up front rather
    // than discovering half way through the sweep that they would not fit.
    if (NEA_PatternRecognizerSetAlgorithm(rec, NEA_PATTERN_FINE) != 0)
    {
        printf("\x1b[31mno memory for Fine\x1b[39m\n");
        goto done;
    }

    printf("%d cases\n\n", PTN_NUM_CASES);

    for (int i = 0; i < PTN_NUM_CASES; i++)
    {
        run_case(&ptn_cases[i]);
        if ((i & 63) == 0)
            swiWaitForVBlank();
    }

    printf("\n%d checks, ", checks);
    if (failures == 0)
        printf("\x1b[32mall passed\x1b[39m\n");
    else
        printf("\x1b[31m%d FAILED\x1b[39m\n", failures);

done:
    while (1)
    {
        swiWaitForVBlank();
        scanKeys();
        if (keysDown() & KEY_START)
            break;
    }

    return 0;
}
