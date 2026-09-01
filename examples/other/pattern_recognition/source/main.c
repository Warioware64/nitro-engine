// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Warioware64, 2026
//
// This file is part of Nitro Engine Advanced
//
// Recognises what the stylus draws.
//
// Draw a digit or a gesture on the touch screen. The ink appears as you draw
// it, and when you lift the stylus the shape is matched against a dictionary
// and the five best answers are listed with their scores. The prototype that
// won is drawn in the corner, so a wrong answer shows you what it thought you
// meant.
//
// Two things are worth playing with. Switching to the Light algorithm and
// drawing a T and then a plus gets the same answer for both: Light compares
// stroke directions and nothing else, and those two shapes have the same
// directions in the same order. Standard scores where the strokes sit as
// well, and tells them apart. And the small squares drawn over the ink are
// the points the matching actually saw, so winding the resample threshold up
// until they stop describing your shape shows exactly what that setting costs.

#include <NEAMain.h>

#include "patterns_bin.h"

#define INK_POINTS   384
#define MAX_POINTS   40
#define MAX_STROKES  8
#define MAX_RESULTS  5

// Long enough that a pause between two strokes of a T is not mistaken for the
// end of the gesture, short enough not to feel like a wait.
#define IDLE_FRAMES  20

#define KIND_DIGIT   1
#define KIND_GESTURE 2

#define TRAIN_ENTRIES 16
#define TRAIN_POINTS  512

typedef struct {
    NEA_PatternStrokes *ink;
    NEA_PatternRecognizer *rec;

    NEA_PatternBank *dict;
    NEA_PatternBank *trained;
    NEA_PatternBank *active;

    NEA_PatternResult results[MAX_RESULTS];
    int nresults;

    int best_entry;
    bool have_best;
} SceneData;

static const char *algo_names[] = { "light", "standard", "fine" };

static const struct {
    const char *name;
    uint32_t mask;
} kind_filters[] = {
    { "all",      NEA_PATTERN_KIND_ALL },
    { "digits",   KIND_DIGIT },
    { "gestures", KIND_GESTURE },
};

static int algo = NEA_PATTERN_STANDARD;
static int threshold = 2;
static int kind_filter;
static bool use_trained;
static int trained_count;

static void draw_dot(int x, int y, int half, int z, uint32_t color)
{
    NEA_2DDrawQuad(x - half, y - half, x + half + 1, y + half + 1, z, color);
}

// The prototype is stored in the bank's own space, which is much smaller than
// the screen, so it is scaled up into a box in the corner.
static void draw_prototype(const NEA_PatternPoint *points, int count,
                           int size, int ox, int oy, int box)
{
    for (int i = 0; i < count; i++)
    {
        if (points[i].x == NEA_PATTERN_PEN_UP_X)
            continue;
        int x = ox + points[i].x * box / size;
        int y = oy + points[i].y * box / size;
        draw_dot(x, y, 0, 2, NEA_Yellow);
    }
}

void Draw3DScene(void *arg)
{
    SceneData *Scene = arg;

    NEA_2DViewInit();

    // Lower z is nearer, so the two backdrops go first and furthest back.
    NEA_2DDrawQuad(6, 126, 68, 188, 8, RGB15(4, 4, 8));
    if (Scene->have_best)
        NEA_2DDrawQuad(188, 126, 250, 188, 8, RGB15(8, 8, 4));

    // The ink, as it was sampled.
    const NEA_PatternPoint *raw = NULL;
    int nraw = NEA_PatternStrokesGetPoints(Scene->ink, &raw);
    for (int i = 0; i < nraw; i++)
    {
        if (raw[i].x == NEA_PATTERN_PEN_UP_X)
            continue;
        draw_dot(raw[i].x, raw[i].y, 1, 4, NEA_White);
    }

    // The points the matching actually compared, over the top of it. When
    // these stop tracing the shape, the resample threshold is too high.
    const NEA_PatternPoint *reduced = NULL;
    int nred = NEA_PatternRecognizerGetInputPoints(Scene->rec, &reduced);
    int nsize = NEA_PatternBankGetNormalizeSize(Scene->active);
    if (nsize > 0)
    {
        for (int i = 0; i < nred; i++)
        {
            if (reduced[i].x == NEA_PATTERN_PEN_UP_X)
                continue;
            // Drawn back in the bank's space, in the same corner box as the
            // prototype, so the two can be compared directly.
            int x = 8 + reduced[i].x * 56 / nsize;
            int y = 128 + reduced[i].y * 56 / nsize;
            draw_dot(x, y, 1, 2, NEA_Green);
        }
        if (Scene->have_best)
        {
            const NEA_PatternPoint *proto = NULL;
            int nproto = NEA_PatternBankGetEntryPoints(Scene->active,
                                                       Scene->best_entry,
                                                       &proto);
            if (nproto > 0)
                draw_prototype(proto, nproto, nsize, 190, 128, 56);
        }
    }
}

static void print_status(SceneData *Scene, bool drawing)
{
    printf("\x1b[0;0HNEAPattern           \n\n");

    printf("algorithm  %-9s\n", algo_names[algo]);
    printf("resample   %-4d\n", threshold);
    printf("kinds      %-9s\n", kind_filters[kind_filter].name);
    printf("bank       %-10s %3d\n",
           use_trained ? "trained" : "dictionary",
           NEA_PatternBankGetEntryCount(Scene->active));
    printf("\n");

    if (drawing)
    {
        printf("drawing... %d strokes      \n\n",
               NEA_PatternStrokesGetCount(Scene->ink));
    }
    else
    {
        printf("%d stroke(s)               \n\n",
               NEA_PatternStrokesGetCount(Scene->ink));
    }

    for (int i = 0; i < MAX_RESULTS; i++)
    {
        if (i >= Scene->nresults)
        {
            printf("                        \n");
            continue;
        }

        const NEA_PatternResult *r = &Scene->results[i];
        const char *name = NEA_PatternBankGetCodeName(Scene->active, r->code);

        // Scores are f32, so 4096 is a perfect match.
        int whole = r->score >> 12;
        int frac = ((r->score & 4095) * 1000) >> 12;

        if (name != NULL)
            printf(" %d. %-8s  %d.%03d   \n", i + 1, name, whole, frac);
        else
            printf(" %d. user %-3d  %d.%03d   \n", i + 1, r->code, whole, frac);
    }

    printf("\n\x1b[19;0H"
           "L/R algo   <> threshold\n"
           "SELECT kinds  A recognise\n"
           "B clear    START train\n"
           "X switch bank");
}

static void recognize(SceneData *Scene)
{
    Scene->nresults = NEA_PatternRecognize(Scene->rec, Scene->active,
                                           Scene->ink,
                                           kind_filters[kind_filter].mask,
                                           Scene->results, MAX_RESULTS);
    Scene->have_best = Scene->nresults > 0;
    if (Scene->have_best)
        Scene->best_entry = Scene->results[0].entry;
}

int main(int argc, char *argv[])
{
    SceneData Scene = { 0 };

    irqEnable(IRQ_HBLANK);
    irqSet(IRQ_VBLANK, NEA_VBLFunc);
    irqSet(IRQ_HBLANK, NEA_HBLFunc);

    NEA_Init3D();
    // Move the 3D screen to the lower one, which is the one with the stylus.
    NEA_SwapScreens();
    // libnds uses VRAM_C for the text console, reserve A and B only
    NEA_TextureSystemReset(0, 0, NEA_VRAM_AB);
    // Init console in non-3D screen
    consoleDemoInit();

    if (NEA_PatternSystemReset(0) != 0)
    {
        printf("Could not start the pattern system");
        while (1)
            swiWaitForVBlank();
    }

    Scene.dict = NEA_PatternBankLoad(patterns_bin);
    Scene.ink = NEA_PatternStrokesCreate(INK_POINTS);
    Scene.rec = NEA_PatternRecognizerCreate(MAX_POINTS, MAX_STROKES);

    // A second bank the player fills in themselves. It has no name table, so
    // its entries are reported by code, which is what "user 3" below is.
    Scene.trained = NEA_PatternBankCreate(TRAIN_ENTRIES, TRAIN_POINTS,
                                          NEA_PatternBankGetNormalizeSize(
                                              Scene.dict));

    if (Scene.dict == NULL || Scene.ink == NULL || Scene.rec == NULL ||
        Scene.trained == NULL)
    {
        printf("Could not set up the example");
        while (1)
            swiWaitForVBlank();
    }

    Scene.active = Scene.dict;

    int idle = 0;
    bool drawing = false;
    bool dirty = true;

    while (1)
    {
        NEA_WaitForVBL(0);

        scanKeys();
        uint32_t keys = keysDown();

        // Collects points while the stylus is down and closes the stroke when
        // it lifts. Returns 1 on the frame it lifted.
        if (NEA_PatternStrokesFeedTouch(Scene.ink) == 1)
        {
            idle = 0;
            dirty = true;
        }

        bool touching = (keysHeld() & KEY_TOUCH) != 0;
        if (touching)
        {
            if (!drawing)
            {
                drawing = true;
                dirty = true;
            }
            idle = 0;
        }
        else if (drawing)
        {
            idle++;
            // Recognise once the hand has stopped, rather than the moment the
            // stylus lifts, or a two stroke shape is judged on its first
            // stroke alone.
            if (idle > IDLE_FRAMES)
            {
                recognize(&Scene);
                drawing = false;
                dirty = true;
            }
        }

        if (keys & KEY_A)
        {
            recognize(&Scene);
            drawing = false;
            dirty = true;
        }

        if (keys & KEY_B)
        {
            NEA_PatternStrokesClear(Scene.ink);
            Scene.nresults = 0;
            Scene.have_best = false;
            drawing = false;
            dirty = true;
        }

        if (keys & (KEY_L | KEY_R))
        {
            algo += (keys & KEY_R) ? 1 : 2;
            algo %= 3;
            // Fine's tables are allocated here rather than at startup, so if
            // there is no room for them the setting simply does not take.
            if (NEA_PatternRecognizerSetAlgorithm(Scene.rec, algo) != 0)
                algo = NEA_PATTERN_STANDARD;
            recognize(&Scene);
            dirty = true;
        }

        if (keys & (KEY_LEFT | KEY_RIGHT))
        {
            threshold += (keys & KEY_RIGHT) ? 1 : -1;
            if (threshold < 0)
                threshold = 0;
            if (threshold > 16)
                threshold = 16;
            NEA_PatternRecognizerSetResample(Scene.rec,
                                             NEA_PATTERN_RESAMPLE_RECURSIVE,
                                             threshold);
            recognize(&Scene);
            dirty = true;
        }

        if (keys & KEY_SELECT)
        {
            kind_filter = (kind_filter + 1) % 3;
            recognize(&Scene);
            dirty = true;
        }

        if (keys & KEY_X)
        {
            use_trained = !use_trained;
            Scene.active = use_trained ? Scene.trained : Scene.dict;
            Scene.nresults = 0;
            Scene.have_best = false;
            dirty = true;
        }

        if (keys & KEY_START)
        {
            // Training: the gesture on screen becomes a prototype of its own,
            // under the next free code. This is what lets a game offer the
            // player gestures the author never drew.
            int code = trained_count;
            if (NEA_PatternBankAdd(Scene.trained, Scene.ink, code,
                                   NEA_PATTERN_KIND_ALL) >= 0)
            {
                trained_count++;
                use_trained = true;
                Scene.active = Scene.trained;
                NEA_PatternStrokesClear(Scene.ink);
                Scene.nresults = 0;
                Scene.have_best = false;
                drawing = false;
            }
            dirty = true;
        }

        if (dirty)
        {
            print_status(&Scene, drawing);
            dirty = false;
        }

        NEA_ProcessArg(Draw3DScene, &Scene);
    }

    return 0;
}
