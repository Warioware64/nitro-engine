// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Warioware64, 2026
//
// This file is part of Nitro Engine Advanced

// Cross-check of the AnimMat evaluator against its Python twin.
//
// tools/animmat_editor/animmat_format.py contains a second implementation of
// the evaluator, because the editor's live preview has to show what the DS will
// actually do. Two implementations of the same fixed-point arithmetic drift,
// and when they drift the symptom is an authoring tool that quietly lies.
//
// gen_vectors.py writes an animation and, from the Python evaluator, a table of
// what every frame of it should produce. This loads that animation, runs it
// through the real runtime, and compares. It exercises all three storage modes,
// both interpolation modes, and the track types whose interpolation is not
// plain linear: clamped alpha, per-channel colour, packed dual colour, the
// wrapping angle and the stepped swaps.

#include <stdio.h>

#include <nds.h>

#include <NEAMain.h>

#include "vectors.h"
#include "vectors_bin.h"

// Fake table entries. The evaluator only ever stores these pointers, so they
// are never dereferenced; their only job is to be distinguishable so the test
// can work out which index a swap track selected.
#define FAKE_TABLE_SIZE 8
static NEA_Material *fake_tex[FAKE_TABLE_SIZE];
static NEA_Palette *fake_pal[FAKE_TABLE_SIZE];

static int failures = 0;
static int checks = 0;

static int index_of_tex(const NEA_Material *m)
{
    for (int i = 0; i < FAKE_TABLE_SIZE; i++)
        if (fake_tex[i] == m)
            return i;
    return 0xFF;
}

static int index_of_pal(const NEA_Palette *p)
{
    for (int i = 0; i < FAKE_TABLE_SIZE; i++)
        if (fake_pal[i] == p)
            return i;
    return 0xFF;
}

// Reads back what the runtime resolved for one track type, in the same encoding
// the Python evaluator returns.
static uint32_t read_back(const NEA_AnimMatInstance *inst, uint8_t type)
{
    switch (type)
    {
        case NEA_AMTRACK_ALPHA:
            return (inst->out_poly_format >> 16) & 0x1F;
        case NEA_AMTRACK_POLYID:
            return (inst->out_poly_format >> 24) & 0x3F;
        case NEA_AMTRACK_LIGHTS:
            return inst->out_poly_format & 0x0F;
        case NEA_AMTRACK_CULLING:
            return inst->out_poly_format & 0xC0;
        case NEA_AMTRACK_COLOR:
            return inst->out_color;
        case NEA_AMTRACK_DIFFUSE_AMBIENT:
            return inst->out_diff_amb;
        case NEA_AMTRACK_SPECULAR_EMISSION:
            return inst->out_spec_emi;
        case NEA_AMTRACK_TEX_SCROLL_X:
            return (uint32_t)inst->out_tex_scroll_x;
        case NEA_AMTRACK_TEX_SCROLL_Y:
            return (uint32_t)inst->out_tex_scroll_y;
        case NEA_AMTRACK_TEX_ROTATE:
            return (uint32_t)inst->out_tex_rotate;
        case NEA_AMTRACK_TEX_SCALE_X:
            return (uint32_t)inst->out_tex_scale_x;
        case NEA_AMTRACK_TEX_SCALE_Y:
            return (uint32_t)inst->out_tex_scale_y;
        case NEA_AMTRACK_TEXPAL_SWAP:
            return ((uint32_t)index_of_tex(inst->out_texture) << 8)
                 | (uint32_t)index_of_pal(inst->out_palette);
        default:
            return 0;
    }
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

    NEA_AnimMatSystemReset(4);

    for (int i = 0; i < FAKE_TABLE_SIZE; i++)
    {
        fake_tex[i] = (NEA_Material *)(uintptr_t)(0x1000 + i * 4);
        fake_pal[i] = (NEA_Palette *)(uintptr_t)(0x2000 + i * 4);
    }

    NEA_AnimMatData *data = NEA_AnimMatDataLoad(vectors_bin);
    if (data == NULL)
    {
        printf("FAILED to load vectors.bin\n");
        while (1)
            swiWaitForVBlank();
    }

    printf("AnimMat evaluator check\n");
    printf("=======================\n\n");
    printf("targets %d  frames %d\n\n", data->num_targets, data->num_frames);

    if (data->num_targets != VEC_NUM_TARGETS)
    {
        printf("target count %d, expected %d\n",
               data->num_targets, VEC_NUM_TARGETS);
        failures++;
    }

    NEA_AnimMatInstance *inst = NEA_AnimMatCreate();
    NEA_AnimMatSetData(inst, data);
    NEA_AnimMatSetTexPalTables(inst, fake_tex, FAKE_TABLE_SIZE,
                               fake_pal, FAKE_TABLE_SIZE);

    // The base values have to match what the Python side assumes, or every
    // frame of a track that is not present in a target would differ.
    NEA_AnimMatSetBasePolyFormat(inst, 31, 0, NEA_LIGHT_0, NEA_CULL_BACK, 0);

    for (int t = 0; t < VEC_NUM_TARGETS && t < data->num_targets; t++)
    {
        const vec_target_t *expect = &vec_targets[t];

        if (strcmp(data->targets[t].name, expect->name) != 0)
        {
            printf("target %d name '%s', expected '%s'\n",
                   t, data->targets[t].name, expect->name);
            failures++;
        }

        for (int f = 0; f < VEC_NUM_FRAMES; f++)
        {
            NEA_AnimMatSetFrame(inst, inttof32(f));
            NEA_AnimMatEvaluateTarget(inst, t);

            for (int k = 0; k < expect->num_tracks; k++)
            {
                uint32_t got = read_back(inst, expect->tracks[k].track_type);
                uint32_t want = expect->tracks[k].values[f];

                checks++;

                if (got != want && failures < 8)
                {
                    printf("t%d f%d type%d\n"
                           " got  %08lX\n want %08lX\n",
                           t, f, expect->tracks[k].track_type,
                           (unsigned long)got, (unsigned long)want);
                    failures++;
                }
                else if (got != want)
                {
                    failures++;
                }
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
