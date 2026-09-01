// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Warioware64, 2026
//
// This file is part of Nitro Engine Advanced

// Cross-check of the NEACell evaluator against its Python twin.
//
// tools/cell_editor/cell_format.py contains a second implementation of the
// evaluator, because the editor's live preview has to show what the DS will
// actually do. Two implementations of the same fixed-point arithmetic drift,
// and when they drift the symptom is an authoring tool that quietly lies.
//
// gen_vectors.py writes a bank and, from the Python evaluator, a table of what
// every tick of it should resolve to. This loads that bank, runs it through the
// real runtime, and compares. It covers all three sequence kinds, all four
// playback modes, all three storage modes, both interpolation modes, a
// three-deep parent chain, a negative scale composed with a rotation, and a
// multi-cell whose nodes have coprime lengths.

#include <stdio.h>
#include <string.h>

#include <nds.h>

#include <NEAMain.h>

#include "vectors.h"
#include "vectors_bin.h"

static int failures = 0;
static int checks = 0;

static int cur_instance;

static void report(int seq, int tick, int slot, const char *field,
                   long got, long want)
{
    checks++;
    if (got == want)
        return;

    failures++;
    if (failures <= 8)
    {
        printf("i%d s%d t%d p%d %s\n got %ld want %ld\n",
               cur_instance, seq, tick, slot, field, got, want);
    }
}

// The whole pose of one instance, in the order the Python side emits it: a
// multi-cell contributes its nodes' poses back to front, which is the order
// the backends draw them in.
static int collect(NEA_CellAnim *anim, const NEA_CellPartXform **out, int max)
{
    int n = 0;

    if (anim->num_children > 0)
    {
        for (int i = 0; i < anim->num_children; i++)
        {
            int count = 0;
            const NEA_CellPartXform *pose =
                NEA_CellAnimGetPose(anim->children[i], &count);
            for (int p = 0; p < count && n < max; p++)
                out[n++] = &pose[p];
        }
        return n;
    }

    int count = 0;
    const NEA_CellPartXform *pose = NEA_CellAnimGetPose(anim, &count);
    for (int p = 0; p < count && n < max; p++)
        out[n++] = &pose[p];
    return n;
}

int main(int argc, char *argv[])
{
    irqEnable(IRQ_HBLANK);
    irqSet(IRQ_VBLANK, NEA_VBLFunc);
    irqSet(IRQ_HBLANK, NEA_HBLFunc);

    NEA_Init3D();
    // libnds puts the console in VRAM_C, so keep the texture system off it.
    NEA_TextureSystemReset(0, 0, NEA_VRAM_AB);
    consoleDemoInit();

    printf("NEACell evaluator check\n");
    printf("=======================\n\n");

    // The sine table is the one place the two sides could drift without any
    // frame or track looking obviously wrong, so check it before anything else.
    extern const int16_t *__NEA_CellSinTable(void);
    const int16_t *sin_table = __NEA_CellSinTable();
    int sin_bad = 0;
    for (int i = 0; i < VEC_SIN_SIZE; i++)
    {
        if (sin_table[i] != vec_sin[i])
            sin_bad++;
    }
    if (sin_bad)
    {
        printf("sine table: %d of %d entries differ\n", sin_bad, VEC_SIN_SIZE);
        failures += sin_bad;
    }

    NEA_CellSystemReset(8);

    NEA_CellData *data = NEA_CellDataLoad(vectors_bin);
    if (data == NULL)
    {
        printf("FAILED to load vectors.bin\n");
        while (1)
            swiWaitForVBlank();
    }

    printf("%d cells  %d parts\n%d seqs  %d frames  %d tracks\n\n",
           data->num_cells, data->num_parts, data->num_sequences,
           data->num_frames, data->num_tracks);

    if (data->num_sequences != VEC_NUM_SEQS)
    {
        printf("sequence count %d, expected %d\n",
               data->num_sequences, VEC_NUM_SEQS);
        failures++;
    }

    NEA_CellAnim *anim = NEA_CellAnimCreate();
    NEA_CellAnimSetData(anim, data);

    int pose_at = 0;

    for (int inst = 0; inst < VEC_NUM_INSTANCES; inst++)
    {
    const vec_instance_t *vi = &vec_instances[inst];
    cur_instance = inst;
    NEA_CellAnimSetAnchor(anim, (NEA_CellAnchor)vi->anchor);
    NEA_CellAnimSetTransformI(anim, vi->rot, vi->sx, vi->sy);

    for (int s = 0; s < VEC_NUM_SEQS && s < data->num_sequences; s++)
    {
        if (inst == 0 && strncmp(data->sequences[s].name,
                                 vec_seq_names[s], 16) != 0)
        {
            printf("seq %d name '%s', expected '%s'\n",
                   s, data->sequences[s].name, vec_seq_names[s]);
            failures++;
        }

        // Stepped, not seeked. A multi-cell seats its nodes on the first
        // evaluate and they then run off the parent's clock, so seeking the
        // parent would leave them behind -- and stepping is what a game does
        // anyway.
        NEA_CellAnimPlay(anim, s, inttof32(1));

        for (int tick = 0; tick < VEC_NUM_TICKS; tick++)
        {
            if (tick > 0)
                NEA_CellAnimUpdate(anim);

            const NEA_CellPartXform *got[32];
            int n = collect(anim, got, 32);
            int want_n = vec_counts[inst * VEC_NUM_SEQS + s][tick];

            if (n != want_n)
            {
                checks++;
                failures++;
                if (failures <= 8)
                    printf("i%d s%d t%d: %d parts, expected %d\n",
                           inst, s, tick, n, want_n);
                pose_at += want_n;
                continue;
            }

            for (int p = 0; p < n; p++)
            {
                const vec_pose_t *want = &vec_poses[pose_at + p];
                const NEA_CellPartXform *x = got[p];

                report(s, tick, p, "m0", x->m[0], want->m0);
                report(s, tick, p, "m1", x->m[1], want->m1);
                report(s, tick, p, "m2", x->m[2], want->m2);
                report(s, tick, p, "m3", x->m[3], want->m3);
                report(s, tick, p, "tx", x->tx, want->tx);
                report(s, tick, p, "ty", x->ty, want->ty);
                report(s, tick, p, "color", x->color, want->color);
                report(s, tick, p, "alpha", x->alpha, want->alpha);
                report(s, tick, p, "prio", x->priority, want->priority);
                report(s, tick, p, "vis", x->visible, want->visible);
                report(s, tick, p, "part",
                       (long)(x->part - data->parts), want->part);
                report(s, tick, p, "src",
                       (long)(x->src - data->parts), want->src);
            }

            pose_at += want_n;
        }
    }
    }

    printf("\n%d checks, %d failures\n", checks, failures);
    printf("\n%s\n", failures == 0 ? "PASS" : "FAIL");

    // Nothing is drawn: this test only reads back evaluator output.
    while (1)
        swiWaitForVBlank();

    return 0;
}
