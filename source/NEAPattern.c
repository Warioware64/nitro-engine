// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Warioware64
//
// This file is part of Nitro Engine Advanced
//
// Stylus pattern recognition. A gesture is fitted to the bank's coordinate
// space, reduced to the points that carry its shape, described by the angle
// and arc length share of each segment, and then compared against every
// prototype with the same number of strokes.
//
// Everything here is integer, and every operation is one the host can perform
// identically -- mulf32 is an arithmetic shift, the DS divider truncates
// toward zero the way C does, and the DS square root floors. That is not an
// accident: tools/pattern_editor/pattern_format.py implements the same
// evaluator so the editor's live scores are the scores the DS will produce,
// and tests/pattern_eval compares the two score for score. If you change an
// expression here, change it there.
//
// atan2 is the one thing libnds does not provide, and sinLerp()'s table is no
// help because it lives inside libnds.a where nothing on the host can mirror
// it. So the table is generated into NEAPatternAtan.h from the Python side.

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "NEAMain.h"
#include "NEAPatternAtan.h"

/// @file NEAPattern.c

//-----------------------------------------------------------------------------
// File format
//-----------------------------------------------------------------------------

#define NEA_PATTERN_MAGIC   0x4E54504E // 'NPTN', little endian
#define NEA_PATTERN_VERSION 1

#define NEA_PATTERN_HEADER_SIZE 32
#define NEA_PATTERN_ENTRY_SIZE  20

#define NEA_PATTERN_MAX_NORMALIZE_SIZE 512

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t flags;
    uint16_t normalize_size;
    uint16_t entry_count;
    uint32_t point_count;
    uint32_t point_offset;
    uint32_t entry_offset;
    uint32_t name_offset;
    uint32_t name_size;
} ne_pattern_header;

//-----------------------------------------------------------------------------
// Internal types
//-----------------------------------------------------------------------------

// One stroke, with the per point features the matchers read. pts, ang and
// ratio are parallel and all point into the pattern's pools.
typedef struct {
    const NEA_PatternPoint *pts;
    const uint16_t *ang;   // direction of the segment ending at this point
    const int32_t *ratio;  // f32 share of the stroke that segment covers
    int npoints;
    int32_t len;           // f32 arc length
    int32_t sratio;        // f32 share of the whole gesture
} ne_pattern_stroke;

typedef struct {
    uint32_t kind;
    uint16_t code;
    int16_t correction;
    uint32_t point_index;  // into bank->points, markers included
    uint16_t point_count;
    uint16_t stroke_count; // as stored
    bool enabled;
    int stroke_first;      // into bank->strokes
    int stroke_valid;      // strokes that survived feature extraction
} ne_pattern_entry;

struct NEA_PatternStrokes {
    NEA_PatternPoint *points;
    int count;
    int capacity;
    bool pen_down;
};

struct NEA_PatternBank {
    // The source of truth: raw marker terminated points, exactly as the file
    // holds them. Everything else is derived and rebuilt from here.
    NEA_PatternPoint *points;
    int point_count;
    int point_capacity;

    ne_pattern_entry *entries;
    int entry_count;
    int entry_capacity;

    // Derived feature pools, marker free.
    NEA_PatternPoint *fpts;
    uint16_t *fang;
    int32_t *fratio;
    ne_pattern_stroke *strokes;
    int stroke_capacity;

    // The name table, kept verbatim so a load followed by a save produces
    // the same bytes back.
    uint8_t *names;
    uint32_t name_size;
    int name_count;

    int normalize_size;
    bool growable;
};

struct NEA_PatternRecognizer {
    int max_points;
    int max_strokes;

    NEA_PatternAlgorithm algo;
    NEA_PatternResampleMethod method;
    int threshold;
    int32_t lf_threshold;
    int lf_ratio;

    // The gesture being matched, after normalising and resampling.
    NEA_PatternPoint *fpts;
    uint16_t *fang;
    int32_t *fratio;
    ne_pattern_stroke *strokes;
    int nstrokes;

    // The same points in the marker terminated form the caller can draw.
    NEA_PatternPoint *out_points;
    int out_count;

    // The gesture after normalising, before reducing. Grown to fit whatever
    // ink it is handed, because the caps above describe the reduced gesture.
    NEA_PatternPoint *norm;
    int norm_capacity;

    // Scratch for the resamplers: a keep flag per point and a stack of
    // ranges, so the recursive one does not recurse.
    uint8_t *keep;
    int32_t *stack;

    // Per stroke scores and weights, one slot per stroke.
    int32_t *raws;
    int32_t *weights;

    // Fine's dynamic programming tables, allocated only when it is selected.
    int32_t *dp_sum;
    int32_t *dp_cnt;
};

//-----------------------------------------------------------------------------
// System state
//-----------------------------------------------------------------------------

static NEA_PatternBank **ne_pattern_banks;
static NEA_PatternRecognizer **ne_pattern_recognizers;
static int ne_pattern_max_banks;
static int ne_pattern_max_recognizers;
static bool ne_pattern_inited;

//-----------------------------------------------------------------------------
// Fixed point, mirroring pattern_format.py
//-----------------------------------------------------------------------------

// mulf32() is (int64)a * b >> 12. The shift is arithmetic, so it floors for
// negative values, which is what Python's >> does too.

/// Integer division truncating toward zero, which is what C does natively and
/// what the DS divider does. Spelled out so the host mirror is unambiguous.
static inline int32_t ne_pattern_div(int32_t a, int32_t b)
{
    if (b == 0)
        return 0;
    return a / b;
}

static inline int32_t ne_pattern_div64(int64_t a, int32_t b)
{
    if (b == 0)
        return 0;
    return (int32_t)(a / b);
}

/// Angle of (x, y) in 16 bit units, 0 to 65535, clockwise from +x because y
/// runs down the screen.
ARM_CODE
static uint16_t ne_pattern_atan2(int y, int x)
{
    if (x == 0 && y == 0)
        return 0;

    int ax = x < 0 ? -x : x;
    int ay = y < 0 ? -y : y;

    int a;
    if (ax >= ay)
    {
        // The tangent is at most 1, so it indexes the table directly.
        a = nea_pattern_atan_table[(ay << NEA_PATTERN_ATAN_SHIFT) / ax];
    }
    else
    {
        // Past 45 degrees: look up the reciprocal slope and reflect.
        a = 16384 - nea_pattern_atan_table[(ax << NEA_PATTERN_ATAN_SHIFT) / ay];
    }

    if (x >= 0)
        return (uint16_t)(y >= 0 ? a : (65536 - a));

    return (uint16_t)(y >= 0 ? (32768 - a) : (32768 + a));
}

/// Shortest distance between two angles, 0 to 32768.
static inline int ne_pattern_angle_diff(uint16_t a, uint16_t b)
{
    int d = (int)(int16_t)(uint16_t)(a - b);
    return d < 0 ? -d : d;
}

static inline int ne_pattern_city_block(const NEA_PatternPoint *p,
                                        const NEA_PatternPoint *q)
{
    int dx = p->x - q->x;
    int dy = p->y - q->y;
    if (dx < 0)
        dx = -dx;
    if (dy < 0)
        dy = -dy;
    return dx + dy;
}

/// Euclidean distance as f32. The squared distance is a plain integer, so it
/// is shifted into f32 before the root; coordinates are normalised into
/// [0, 511] by the time this runs, so the shifted value stays in 32 bits.
static inline int32_t ne_pattern_distance(const NEA_PatternPoint *p,
                                          const NEA_PatternPoint *q)
{
    int dx = p->x - q->x;
    int dy = p->y - q->y;
    uint32_t d2 = (uint32_t)(dx * dx + dy * dy);
    return (int32_t)sqrtf32(d2 << 12);
}

static inline bool ne_pattern_is_pen_up(const NEA_PatternPoint *p)
{
    return p->x == NEA_PATTERN_PEN_UP_X;
}

//-----------------------------------------------------------------------------
// Normalisation
//
// Fit the whole gesture's bounding box into the bank's space, uniformly, and
// centre the short axis. That is what lets a pattern match at any size and
// anywhere on the screen. Points landing on their predecessor are dropped,
// and a stroke left with fewer than two points goes with them: it has no
// direction, so there is nothing to score.
//-----------------------------------------------------------------------------

/// One fit. Marker terminated in, marker terminated out.
///
/// Safe to call with out == in: nothing is ever written ahead of where the
/// read is, because a pass only ever removes points.
///
/// @return Number of points written, 0 if nothing survived. *dropped is set
///         to the number of strokes that did not survive.
static int ne_pattern_normalize_pass(const NEA_PatternPoint *in, int in_count,
                                     NEA_PatternPoint *out, int out_capacity,
                                     int normalize_size, int *dropped)
{
    int strokes_in = 0;
    int strokes_out = 0;

    for (int i = 0; i < in_count; i++)
    {
        if (ne_pattern_is_pen_up(&in[i]))
            strokes_in++;
    }

    int x1 = 0, x2 = 0, y1 = 0, y2 = 0;
    bool any = false;

    for (int i = 0; i < in_count; i++)
    {
        if (ne_pattern_is_pen_up(&in[i]))
            continue;
        if (!any)
        {
            x1 = x2 = in[i].x;
            y1 = y2 = in[i].y;
            any = true;
            continue;
        }
        if (in[i].x < x1) x1 = in[i].x;
        if (in[i].x > x2) x2 = in[i].x;
        if (in[i].y < y1) y1 = in[i].y;
        if (in[i].y > y2) y2 = in[i].y;
    }

    if (!any)
    {
        *dropped = strokes_in;
        return 0;
    }

    int wx = x2 - x1;
    int wy = y2 - y1;
    int w = wx >= wy ? wx : wy;
    if (w <= 0)
    {
        // Every sample landed on the same point. There is no shape here.
        *dropped = strokes_in;
        return 0;
    }

    int limit = normalize_size - 1;

    // 16.16, mapping the long axis onto [0, limit]. The rounding term is what
    // makes that exact rather than a pixel short. The product cannot
    // overflow: (p - min) never exceeds w, so it is at most limit << 16.
    int scale = (limit << 16) / w;

    // Scale first, then centre the short axis, in output units. Centring
    // before the scale -- which is the obvious way round -- lets the centring
    // error be scaled along with everything else, and the fit stops being
    // idempotent.
    int sx = (wx * scale + 32768) >> 16;
    int sy = (wy * scale + 32768) >> 16;
    int cx = (limit - sx) / 2;
    int cy = (limit - sy) / 2;

    int written = 0;
    int stroke_start = 0;

    for (int i = 0; i < in_count; i++)
    {
        if (!ne_pattern_is_pen_up(&in[i]))
        {
            // Leave room for the marker this stroke will need.
            if (written >= out_capacity - 1)
                break;

            int x = (((in[i].x - x1) * scale + 32768) >> 16) + cx;
            int y = (((in[i].y - y1) * scale + 32768) >> 16) + cy;
            if (x < 0) x = 0; else if (x > limit) x = limit;
            if (y < 0) y = 0; else if (y > limit) y = limit;

            // Two samples that scale onto the same pixel say nothing new.
            if (written > stroke_start &&
                out[written - 1].x == x && out[written - 1].y == y)
                continue;

            out[written].x = (int16_t)x;
            out[written].y = (int16_t)y;
            written++;
            continue;
        }

        if (written - stroke_start >= 2)
        {
            out[written].x = NEA_PATTERN_PEN_UP_X;
            out[written].y = NEA_PATTERN_PEN_UP_Y;
            written++;
            stroke_start = written;
            strokes_out++;
        }
        else
        {
            // A stroke of fewer than two points has no direction, so there is
            // nothing to score. Roll it back.
            written = stroke_start;
        }
    }

    // The last stroke may not have been closed, either because the caller
    // never lifted the pen or because the buffer ran out above.
    if (written - stroke_start >= 2 && written < out_capacity)
    {
        out[written].x = NEA_PATTERN_PEN_UP_X;
        out[written].y = NEA_PATTERN_PEN_UP_Y;
        written++;
        strokes_out++;
    }
    else if (written - stroke_start < 2)
    {
        written = stroke_start;
    }

    *dropped = strokes_in > strokes_out ? strokes_in - strokes_out : 0;
    return written;
}

/// Fits a gesture into the bank's space, keeping its shape.
///
/// The fit is iterated because dropping a stroke shrinks what is left: a
/// gesture whose stray dot collapses away has a smaller bounding box than the
/// gesture did, and fitting to the box that dot was part of would leave the
/// survivors short of the edge. Iterating settles on a fixed point -- the
/// stroke count only ever falls, so it terminates quickly -- and that fixed
/// point is what makes re-fitting an already fitted prototype a no-op, which
/// is what a bank exported and re-imported goes through.
///
/// @return Number of points written, 0 if nothing survived.
static int ne_pattern_normalize(const NEA_PatternPoint *in, int in_count,
                                NEA_PatternPoint *out, int out_capacity,
                                int normalize_size)
{
    int dropped = 0;
    int written = ne_pattern_normalize_pass(in, in_count, out, out_capacity,
                                            normalize_size, &dropped);

    for (int guard = in_count; written > 0 && dropped > 0 && guard > 0; guard--)
    {
        written = ne_pattern_normalize_pass(out, written, out, out_capacity,
                                            normalize_size, &dropped);
    }

    return written;
}

//-----------------------------------------------------------------------------
// Resampling
//
// Each of these takes one stroke and marks the points it keeps. The first and
// last are always kept.
//-----------------------------------------------------------------------------

static void ne_pattern_resample_none(const NEA_PatternPoint *p, int n,
                                     uint8_t *keep, int threshold)
{
    (void)threshold;
    for (int i = 0; i < n; i++)
        keep[i] = 1;
}

static void ne_pattern_resample_distance(const NEA_PatternPoint *p, int n,
                                         uint8_t *keep, int threshold)
{
    if (threshold < 1)
        threshold = 1;

    memset(keep, 0, n);
    keep[0] = 1;
    keep[n - 1] = 1;

    int acc = 0;
    for (int i = 1; i < n - 1; i++)
    {
        acc += ne_pattern_city_block(&p[i - 1], &p[i]);
        if (acc >= threshold)
        {
            keep[i] = 1;
            acc = 0;
        }
    }
}

static void ne_pattern_resample_angle(const NEA_PatternPoint *p, int n,
                                      uint8_t *keep, int threshold)
{
    if (threshold < 1)
        threshold = 1;

    memset(keep, 0, n);
    keep[0] = 1;
    keep[n - 1] = 1;

    int last = 0;
    uint16_t ref = ne_pattern_atan2(p[1].y - p[0].y, p[1].x - p[0].x);

    for (int i = 1; i < n - 1; i++)
    {
        uint16_t a = ne_pattern_atan2(p[i + 1].y - p[i].y,
                                      p[i + 1].x - p[i].x);
        if (ne_pattern_angle_diff(a, ref) < threshold)
            continue;
        // Without a separation floor, a slow hand rounding a corner emits a
        // run of points that each look like a big turn from the last.
        if (ne_pattern_city_block(&p[last], &p[i]) < 2)
            continue;
        keep[i] = 1;
        last = i;
        ref = a;
    }
}

/// Ramer-Douglas-Peucker, with no divide and no square root.
///
/// The distance of C from the line AB is |cross(B - A, C - A)| / |B - A|, so
/// testing it against t is testing cross^2 against t^2 * |B - A|^2.
static void ne_pattern_resample_recursive(const NEA_PatternPoint *p, int n,
                                          uint8_t *keep, int threshold,
                                          int32_t *stack)
{
    if (threshold < 1)
        threshold = 1;

    memset(keep, 0, n);
    keep[0] = 1;
    keep[n - 1] = 1;

    int64_t tsq = (int64_t)threshold * threshold;
    int top = 0;
    stack[top++] = 0;
    stack[top++] = n - 1;

    while (top > 0)
    {
        int hi = stack[--top];
        int lo = stack[--top];
        if (hi - lo < 2)
            continue;

        int ax = p[lo].x, ay = p[lo].y;
        int ux = p[hi].x - ax;
        int uy = p[hi].y - ay;
        int64_t span = (int64_t)ux * ux + (int64_t)uy * uy;

        int best = -1;
        int64_t best_dist = 0;

        for (int i = lo + 1; i < hi; i++)
        {
            int64_t d;
            if (span == 0)
            {
                // The chord is a point: fall back to distance from it.
                int cb = ne_pattern_city_block(&p[i], &p[lo]);
                d = (int64_t)cb * cb;
            }
            else
            {
                int64_t cross = (int64_t)ux * (p[i].y - ay)
                              - (int64_t)uy * (p[i].x - ax);
                d = cross < 0 ? -cross : cross;
            }
            if (d > best_dist)
            {
                best_dist = d;
                best = i;
            }
        }

        if (best < 0)
            continue;

        bool over;
        if (span == 0)
            over = best_dist >= tsq;
        else
            over = best_dist * best_dist >= tsq * span;

        if (over)
        {
            keep[best] = 1;
            // The stack cannot overflow: every range pushed is strictly
            // smaller than its parent and they partition the stroke, so at
            // most n ranges are ever live.
            stack[top++] = lo;
            stack[top++] = best;
            stack[top++] = best;
            stack[top++] = hi;
        }
    }
}

//-----------------------------------------------------------------------------
// Feature extraction
//-----------------------------------------------------------------------------

/// Fills strokes[] and the feature pools from marker terminated points.
///
/// @return Number of strokes that survived, 0 if none did.
static int ne_pattern_extract(const NEA_PatternPoint *in, int in_count,
                              NEA_PatternPoint *fpts, uint16_t *fang,
                              int32_t *fratio, int fcapacity,
                              ne_pattern_stroke *strokes, int max_strokes,
                              int *points_written)
{
    int nstrokes = 0;
    int written = 0;
    int32_t total = 0;

    int start = 0;
    for (int i = 0; i <= in_count; i++)
    {
        bool end = (i == in_count) || ne_pattern_is_pen_up(&in[i]);
        if (!end)
            continue;

        int n = i - start;
        if (n >= 2 && nstrokes < max_strokes && written + n <= fcapacity)
        {
            NEA_PatternPoint *pts = &fpts[written];
            uint16_t *ang = &fang[written];
            int32_t *ratio = &fratio[written];

            int32_t length = 0;
            pts[0] = in[start];
            for (int j = 1; j < n; j++)
            {
                pts[j] = in[start + j];
                // Parked in ratio[] and turned into a share below, so the
                // square root is paid for once per segment rather than twice.
                ratio[j] = ne_pattern_distance(&pts[j - 1], &pts[j]);
                length += ratio[j];
                ang[j] = ne_pattern_atan2(pts[j].y - pts[j - 1].y,
                                          pts[j].x - pts[j - 1].x);
            }

            if (length > 0)
            {
                // There is no segment 0. Giving it segment 1's direction is
                // the same as reflecting the first point through the second,
                // and it keeps every array the same length as the points.
                ang[0] = ang[1];
                ratio[0] = 0;

                // Share of the stroke each segment covers. The last absorbs
                // the truncation so the shares sum to exactly 1.0, which is
                // what lets two strokes be walked in lockstep without either
                // running out early.
                int32_t acc = 0;
                for (int j = 1; j < n - 1; j++)
                {
                    ratio[j] = (int32_t)(((int64_t)ratio[j] << 12) / length);
                    acc += ratio[j];
                }
                ratio[n - 1] = 4096 - acc;

                strokes[nstrokes].pts = pts;
                strokes[nstrokes].ang = ang;
                strokes[nstrokes].ratio = ratio;
                strokes[nstrokes].npoints = n;
                strokes[nstrokes].len = length;
                strokes[nstrokes].sratio = 0;
                nstrokes++;
                written += n;
                total += length;
            }
        }

        start = i + 1;
        if (i >= in_count)
            break;
    }

    if (nstrokes == 0 || total <= 0)
    {
        if (points_written)
            *points_written = 0;
        return 0;
    }

    // Share of the whole gesture each stroke covers, same trick.
    int32_t acc = 0;
    for (int i = 0; i < nstrokes - 1; i++)
    {
        strokes[i].sratio = (int32_t)(((int64_t)strokes[i].len << 12) / total);
        acc += strokes[i].sratio;
    }
    strokes[nstrokes - 1].sratio = 4096 - acc;

    if (points_written)
        *points_written = written;
    return nstrokes;
}

//-----------------------------------------------------------------------------
// The arc length walk
//
// Both strokes are parameterised over [0, 1] by arc length. Merging their
// breakpoints gives intervals that each sit inside one segment of each
// stroke, and the pair score of that interval is weighted by its length. It
// is what removes timing from the comparison and leaves only shape.
//
// The cell score is inlined by hand into each caller rather than passed as a
// function pointer, because an indirect call per interval is real money on a
// 67 MHz ARM9 with no branch predictor.
//-----------------------------------------------------------------------------

/// Angle agreement scaled to 0..256, times closeness scaled to 0..dbl_w.
static inline int ne_pattern_cell(const ne_pattern_stroke *a, int ia,
                                  const ne_pattern_stroke *b, int ib,
                                  int dbl_w)
{
    int ang = (32768 - ne_pattern_angle_diff(a->ang[ia], b->ang[ib])) >> 7;
    int dist = dbl_w - ne_pattern_city_block(&a->pts[ia], &b->pts[ib]);
    if (dist < 0)
        dist = 0;
    return ang * dist;
}

/// Light's walk: angle difference only, integrated over the arc length.
ARM_CODE
static int32_t ne_pattern_walk_angle(const ne_pattern_stroke *a,
                                     const ne_pattern_stroke *b)
{
    int ia = 1, ib = 1;
    int32_t rema = a->ratio[1];
    int32_t remb = b->ratio[1];
    int32_t total = 0;

    while (ia < a->npoints && ib < b->npoints)
    {
        int32_t step = rema <= remb ? rema : remb;
        if (step)
            total += step * ne_pattern_angle_diff(a->ang[ia], b->ang[ib]);

        rema -= step;
        remb -= step;
        if (rema == 0)
        {
            ia++;
            rema = ia < a->npoints ? a->ratio[ia] : 0;
        }
        if (remb == 0)
        {
            ib++;
            remb = ib < b->npoints ? b->ratio[ib] : 0;
        }
    }

    return total;
}

/// Standard's walk: angle agreement weighted by how close the points are.
ARM_CODE
static int32_t ne_pattern_walk_cell(const ne_pattern_stroke *a,
                                    const ne_pattern_stroke *b, int dbl_w)
{
    int ia = 1, ib = 1;
    int32_t rema = a->ratio[1];
    int32_t remb = b->ratio[1];
    int32_t total = 0;

    while (ia < a->npoints && ib < b->npoints)
    {
        int32_t step = rema <= remb ? rema : remb;
        if (step)
            total += step * ne_pattern_cell(a, ia, b, ib, dbl_w);

        rema -= step;
        remb -= step;
        if (rema == 0)
        {
            ia++;
            rema = ia < a->npoints ? a->ratio[ia] : 0;
        }
        if (remb == 0)
        {
            ib++;
            remb = ib < b->npoints ? b->ratio[ib] : 0;
        }
    }

    return total;
}

//-----------------------------------------------------------------------------
// The matchers
//-----------------------------------------------------------------------------

/// Weighted mean of per stroke scores, mapped onto 0..4096.
static int32_t ne_pattern_combine(const int32_t *raws, const int32_t *weights,
                                  int n, int dbl_w)
{
    int32_t wsum = 0;
    int32_t acc = 0;

    for (int i = 0; i < n; i++)
    {
        acc += (int32_t)(((int64_t)raws[i] * weights[i]) >> 12);
        wsum += weights[i];
    }

    if (wsum <= 0)
        return 0;

    int32_t denom = (int32_t)(((int64_t)wsum * dbl_w) >> 4);
    if (denom <= 0)
        return 0;

    int32_t score = ne_pattern_div64((int64_t)acc << 12, denom);
    if (score < 0)
        return 0;
    return score > 4096 ? 4096 : score;
}

/// Light. Direction only, so it is blind to where the strokes sit relative to
/// one another and cannot tell a T from a plus. It needs no scratch at all.
///
/// worst is the score of the poorest result currently held, or -1 when the
/// ranking is not yet full. Once the accumulated error passes what that score
/// allows, the entry cannot enter the ranking, so abandoning it changes
/// nothing but the time spent.
static int32_t ne_pattern_match_light(const ne_pattern_stroke *pa, int na,
                                      const ne_pattern_stroke *pb, int nb,
                                      int32_t worst)
{
    if (na != nb)
        return 0;

    int32_t border = worst >= 0 ? ((4096 - worst) << 3) : 0;
    int32_t err = 0;

    for (int i = 0; i < na; i++)
    {
        int32_t total = ne_pattern_walk_angle(&pa[i], &pb[i]);
        err += (int32_t)(((int64_t)(total >> 12) * pa[i].sratio) >> 12);
        if (worst >= 0 && err > border)
            return 0;
    }

    int32_t score = 4096 - (err >> 3);
    if (score < 0)
        return 0;
    return score > 4096 ? 4096 : score;
}

/// Standard. Angle agreement weighted by position, so stroke placement counts
/// and T and plus come apart. Still no scratch memory.
static int32_t ne_pattern_match_standard(const ne_pattern_stroke *pa, int na,
                                         const ne_pattern_stroke *pb, int nb,
                                         int dbl_w, int32_t *raws,
                                         int32_t *weights)
{
    if (na != nb)
        return 0;

    for (int i = 0; i < na; i++)
    {
        raws[i] = ne_pattern_walk_cell(&pa[i], &pb[i], dbl_w) >> 12;
        // A stroke that is a large part of *either* pattern matters. Taking
        // the greater share stops a long input stroke matched against a short
        // prototype one from being scored as if it were a detail.
        int32_t w = pa[i].sratio >= pb[i].sratio ? pa[i].sratio : pb[i].sratio;
        weights[i] = w;
    }

    return ne_pattern_combine(raws, weights, na, dbl_w);
}

/// One stroke of Fine: pair the points with dynamic programming instead of
/// assuming both strokes were drawn at proportional speeds, maximising the
/// *mean* pair score so a long path is not rewarded for being long.
ARM_CODE
static int32_t ne_pattern_fine_stroke(const ne_pattern_stroke *a,
                                      const ne_pattern_stroke *b, int dbl_w,
                                      int32_t lf_threshold, int lf_ratio,
                                      int32_t *sums, int32_t *cnts)
{
    if (a->len > lf_threshold || b->len > lf_threshold)
    {
        if ((int64_t)a->len * lf_ratio < b->len ||
            (int64_t)b->len * lf_ratio < a->len)
            return 0;
    }

    int n = a->npoints;
    int m = b->npoints;

    // Row 0 and column 0 are the real first points, not padding: the two
    // strokes start together and end together by construction.
    sums[0] = ne_pattern_cell(a, 0, b, 0, dbl_w);
    cnts[0] = 1;

    for (int i = 0; i < n; i++)
    {
        int32_t *srow = &sums[i * m];
        int32_t *crow = &cnts[i * m];
        const int32_t *sprev = i > 0 ? &sums[(i - 1) * m] : NULL;
        const int32_t *cprev = i > 0 ? &cnts[(i - 1) * m] : NULL;

        for (int j = 0; j < m; j++)
        {
            if (i == 0 && j == 0)
                continue;

            int32_t bs = -1;
            int32_t bc = 1;

            // Three predecessors: consume a prototype point, an input point,
            // or both. Candidates compare on the mean, cross multiplied so
            // there is no divide in the inner loop.
            if (sprev)
            {
                if (cprev[j] > 0)
                {
                    bs = sprev[j];
                    bc = cprev[j];
                }
                if (j > 0 && cprev[j - 1] > 0)
                {
                    if (bs < 0 ||
                        (int64_t)sprev[j - 1] * bc > (int64_t)bs * cprev[j - 1])
                    {
                        bs = sprev[j - 1];
                        bc = cprev[j - 1];
                    }
                }
            }
            if (j > 0 && crow[j - 1] > 0)
            {
                if (bs < 0 ||
                    (int64_t)srow[j - 1] * bc > (int64_t)bs * crow[j - 1])
                {
                    bs = srow[j - 1];
                    bc = crow[j - 1];
                }
            }

            if (bs < 0)
            {
                srow[j] = 0;
                crow[j] = 0;
                continue;
            }

            srow[j] = bs + ne_pattern_cell(a, i, b, j, dbl_w);
            crow[j] = bc + 1;
        }
    }

    int32_t cnt = cnts[(n - 1) * m + (m - 1)];
    if (cnt <= 0)
        return 0;
    return ne_pattern_div(sums[(n - 1) * m + (m - 1)], cnt);
}

static int32_t ne_pattern_match_fine(const ne_pattern_stroke *pa, int na,
                                     const ne_pattern_stroke *pb, int nb,
                                     int dbl_w, int32_t lf_threshold,
                                     int lf_ratio, int32_t *sums,
                                     int32_t *cnts, int32_t *raws,
                                     int32_t *weights)
{
    if (na != nb)
        return 0;

    for (int i = 0; i < na; i++)
    {
        int32_t w = pa[i].sratio >= pb[i].sratio ? pa[i].sratio : pb[i].sratio;
        weights[i] = w;
        raws[i] = ne_pattern_fine_stroke(&pa[i], &pb[i], dbl_w, lf_threshold,
                                         lf_ratio, sums, cnts);
    }

    return ne_pattern_combine(raws, weights, na, dbl_w);
}

//-----------------------------------------------------------------------------
// Strokes
//-----------------------------------------------------------------------------

NEA_PatternStrokes *NEA_PatternStrokesCreate(int max_points)
{
    if (max_points < 2)
    {
        NEA_DebugPrint("NEAPattern: a strokes buffer needs at least 2 points");
        return NULL;
    }

    NEA_PatternStrokes *strokes = calloc(1, sizeof(NEA_PatternStrokes));
    if (strokes == NULL)
    {
        NEA_DebugPrint("NEAPattern: not enough memory");
        return NULL;
    }

    strokes->points = malloc(sizeof(NEA_PatternPoint) * max_points);
    if (strokes->points == NULL)
    {
        NEA_DebugPrint("NEAPattern: not enough memory");
        free(strokes);
        return NULL;
    }

    strokes->capacity = max_points;
    return strokes;
}

void NEA_PatternStrokesDelete(NEA_PatternStrokes *strokes)
{
    if (strokes == NULL)
        return;
    free(strokes->points);
    free(strokes);
}

void NEA_PatternStrokesClear(NEA_PatternStrokes *strokes)
{
    NEA_AssertPointer(strokes, "NULL pointer");
    if (strokes == NULL)
        return;
    strokes->count = 0;
    strokes->pen_down = false;
}

int NEA_PatternStrokesAppendPoint(NEA_PatternStrokes *strokes, int x, int y)
{
    NEA_AssertPointer(strokes, "NULL pointer");
    if (strokes == NULL)
        return -1;

    if (x == NEA_PATTERN_PEN_UP_X)
    {
        // The marker's x is what makes it a marker, so a real sample must
        // never use it. Nudging is kinder than dropping the point.
        x = NEA_PATTERN_PEN_UP_X + 1;
    }

    if (strokes->count >= strokes->capacity)
        return -1;

    strokes->points[strokes->count].x = (int16_t)x;
    strokes->points[strokes->count].y = (int16_t)y;
    strokes->count++;
    strokes->pen_down = true;
    return 0;
}

int NEA_PatternStrokesAppendPenUp(NEA_PatternStrokes *strokes)
{
    NEA_AssertPointer(strokes, "NULL pointer");
    if (strokes == NULL)
        return -1;

    if (!strokes->pen_down)
        return 0;

    if (strokes->count >= strokes->capacity)
        return -1;

    strokes->points[strokes->count].x = NEA_PATTERN_PEN_UP_X;
    strokes->points[strokes->count].y = NEA_PATTERN_PEN_UP_Y;
    strokes->count++;
    strokes->pen_down = false;
    return 0;
}

int NEA_PatternStrokesFeedTouch(NEA_PatternStrokes *strokes)
{
    NEA_AssertPointer(strokes, "NULL pointer");
    if (strokes == NULL)
        return -1;

    if (keysHeld() & KEY_TOUCH)
    {
        touchPosition touch;
        touchRead(&touch);
        NEA_PatternStrokesAppendPoint(strokes, touch.px, touch.py);
        return 0;
    }

    if (strokes->pen_down)
    {
        NEA_PatternStrokesAppendPenUp(strokes);
        return 1;
    }

    return 0;
}

bool NEA_PatternStrokesIsEmpty(const NEA_PatternStrokes *strokes)
{
    NEA_AssertPointer(strokes, "NULL pointer");
    if (strokes == NULL)
        return true;
    return strokes->count == 0;
}

bool NEA_PatternStrokesIsFull(const NEA_PatternStrokes *strokes)
{
    NEA_AssertPointer(strokes, "NULL pointer");
    if (strokes == NULL)
        return true;
    return strokes->count >= strokes->capacity;
}

int NEA_PatternStrokesGetCount(const NEA_PatternStrokes *strokes)
{
    NEA_AssertPointer(strokes, "NULL pointer");
    if (strokes == NULL)
        return -1;

    int count = 0;
    for (int i = 0; i < strokes->count; i++)
    {
        if (ne_pattern_is_pen_up(&strokes->points[i]))
            count++;
    }
    // The stroke in progress has no marker yet, but it is a stroke.
    if (strokes->pen_down)
        count++;
    return count;
}

int NEA_PatternStrokesGetPoints(const NEA_PatternStrokes *strokes,
                                const NEA_PatternPoint **points)
{
    NEA_AssertPointer(strokes, "NULL pointer");
    if (strokes == NULL)
        return -1;
    if (points != NULL)
        *points = strokes->points;
    return strokes->count;
}

//-----------------------------------------------------------------------------
// Bank internals
//-----------------------------------------------------------------------------

static int ne_pattern_stroke_budget(int point_capacity)
{
    // The shortest possible stroke is two points and a marker.
    return point_capacity / 3 + 1;
}

static void ne_pattern_bank_free_derived(NEA_PatternBank *bank)
{
    free(bank->fpts);
    free(bank->fang);
    free(bank->fratio);
    free(bank->strokes);
    bank->fpts = NULL;
    bank->fang = NULL;
    bank->fratio = NULL;
    bank->strokes = NULL;
    bank->stroke_capacity = 0;
}

/// Recomputes every entry's features from the raw point array.
///
/// One code path for loading, for training and for removal, so the three
/// cannot disagree about what an entry means.
static int ne_pattern_bank_rebuild(NEA_PatternBank *bank)
{
    int stroke_capacity = ne_pattern_stroke_budget(bank->point_capacity);

    if (bank->fpts == NULL || bank->stroke_capacity < stroke_capacity)
    {
        ne_pattern_bank_free_derived(bank);

        bank->fpts = malloc(sizeof(NEA_PatternPoint) * bank->point_capacity);
        bank->fang = malloc(sizeof(uint16_t) * bank->point_capacity);
        bank->fratio = malloc(sizeof(int32_t) * bank->point_capacity);
        bank->strokes = malloc(sizeof(ne_pattern_stroke) * stroke_capacity);

        if (bank->fpts == NULL || bank->fang == NULL ||
            bank->fratio == NULL || bank->strokes == NULL)
        {
            NEA_DebugPrint("NEAPattern: not enough memory");
            ne_pattern_bank_free_derived(bank);
            return -1;
        }
        bank->stroke_capacity = stroke_capacity;
    }

    int used_points = 0;
    int used_strokes = 0;

    for (int i = 0; i < bank->entry_count; i++)
    {
        ne_pattern_entry *entry = &bank->entries[i];
        int written = 0;

        entry->stroke_first = used_strokes;
        entry->stroke_valid = ne_pattern_extract(
            &bank->points[entry->point_index], entry->point_count,
            &bank->fpts[used_points], &bank->fang[used_points],
            &bank->fratio[used_points], bank->point_capacity - used_points,
            &bank->strokes[used_strokes], bank->stroke_capacity - used_strokes,
            &written);

        used_points += written;
        used_strokes += entry->stroke_valid;

        if (entry->stroke_valid == 0)
            NEA_DebugPrint("NEAPattern: entry %d has no measurable length", i);
    }

    return 0;
}

static NEA_PatternBank *ne_pattern_bank_alloc(int entry_capacity,
                                              int point_capacity,
                                              int normalize_size)
{
    if (!ne_pattern_inited)
    {
        NEA_DebugPrint("NEAPattern: system not initialized");
        return NULL;
    }

    int slot = -1;
    for (int i = 0; i < ne_pattern_max_banks; i++)
    {
        if (ne_pattern_banks[i] == NULL)
        {
            slot = i;
            break;
        }
    }
    if (slot < 0)
    {
        NEA_DebugPrint("NEAPattern: no free bank slots");
        return NULL;
    }

    if (normalize_size < 1 || normalize_size > NEA_PATTERN_MAX_NORMALIZE_SIZE)
    {
        NEA_DebugPrint("NEAPattern: bad normalize size %d", normalize_size);
        return NULL;
    }
    if (entry_capacity < 1 || point_capacity < 3)
    {
        NEA_DebugPrint("NEAPattern: bank capacities are too small");
        return NULL;
    }

    NEA_PatternBank *bank = calloc(1, sizeof(NEA_PatternBank));
    if (bank == NULL)
    {
        NEA_DebugPrint("NEAPattern: not enough memory");
        return NULL;
    }

    bank->points = malloc(sizeof(NEA_PatternPoint) * point_capacity);
    bank->entries = calloc(entry_capacity, sizeof(ne_pattern_entry));
    if (bank->points == NULL || bank->entries == NULL)
    {
        NEA_DebugPrint("NEAPattern: not enough memory");
        free(bank->points);
        free(bank->entries);
        free(bank);
        return NULL;
    }

    bank->point_capacity = point_capacity;
    bank->entry_capacity = entry_capacity;
    bank->normalize_size = normalize_size;

    ne_pattern_banks[slot] = bank;
    return bank;
}

//-----------------------------------------------------------------------------
// Bank loading and saving
//-----------------------------------------------------------------------------

static uint16_t ne_pattern_read16(const uint8_t *p)
{
    return (uint16_t)(p[0] | (p[1] << 8));
}

static uint32_t ne_pattern_read32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void ne_pattern_write16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
}

static void ne_pattern_write32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

static NEA_PatternBank *ne_pattern_bank_parse(const uint8_t *data, size_t size)
{
    if (size < NEA_PATTERN_HEADER_SIZE)
    {
        NEA_DebugPrint("NEAPattern: file is too short to be a bank");
        return NULL;
    }

    ne_pattern_header h;
    h.magic = ne_pattern_read32(data + 0);
    h.version = ne_pattern_read16(data + 4);
    h.flags = ne_pattern_read16(data + 6);
    h.normalize_size = ne_pattern_read16(data + 8);
    h.entry_count = ne_pattern_read16(data + 10);
    h.point_count = ne_pattern_read32(data + 12);
    h.point_offset = ne_pattern_read32(data + 16);
    h.entry_offset = ne_pattern_read32(data + 20);
    h.name_offset = ne_pattern_read32(data + 24);
    h.name_size = ne_pattern_read32(data + 28);

    if (h.magic != NEA_PATTERN_MAGIC)
    {
        NEA_DebugPrint("NEAPattern: bad magic 0x%08lX", (unsigned long)h.magic);
        return NULL;
    }
    if (h.version != NEA_PATTERN_VERSION)
    {
        NEA_DebugPrint("NEAPattern: unsupported version %d", h.version);
        return NULL;
    }
    if (h.entry_count == 0 || h.point_count == 0)
    {
        NEA_DebugPrint("NEAPattern: bank is empty");
        return NULL;
    }
    if ((size_t)h.point_offset + (size_t)h.point_count * 4 > size ||
        (size_t)h.entry_offset +
            (size_t)h.entry_count * NEA_PATTERN_ENTRY_SIZE > size ||
        (h.name_offset != 0 &&
         (size_t)h.name_offset + (size_t)h.name_size > size))
    {
        NEA_DebugPrint("NEAPattern: bank is truncated");
        return NULL;
    }

    NEA_PatternBank *bank = ne_pattern_bank_alloc(h.entry_count, h.point_count,
                                                  h.normalize_size);
    if (bank == NULL)
        return NULL;

    int limit = h.normalize_size - 1;
    const uint8_t *pp = data + h.point_offset;
    for (uint32_t i = 0; i < h.point_count; i++)
    {
        int16_t x = (int16_t)ne_pattern_read16(pp + i * 4);
        int16_t y = (int16_t)ne_pattern_read16(pp + i * 4 + 2);
        if (x != NEA_PATTERN_PEN_UP_X &&
            (x < 0 || x > limit || y < 0 || y > limit))
        {
            NEA_DebugPrint("NEAPattern: point %ld is outside the bank space",
                           (long)i);
            NEA_PatternBankDelete(bank);
            return NULL;
        }
        bank->points[i].x = x;
        bank->points[i].y = y;
    }
    bank->point_count = (int)h.point_count;

    const uint8_t *ep = data + h.entry_offset;
    for (uint32_t i = 0; i < h.entry_count; i++)
    {
        const uint8_t *e = ep + i * NEA_PATTERN_ENTRY_SIZE;
        ne_pattern_entry *dst = &bank->entries[i];
        dst->kind = ne_pattern_read32(e + 0);
        dst->code = ne_pattern_read16(e + 4);
        dst->correction = (int16_t)ne_pattern_read16(e + 6);
        dst->point_index = ne_pattern_read32(e + 8);
        dst->point_count = ne_pattern_read16(e + 12);
        dst->stroke_count = ne_pattern_read16(e + 14);
        dst->enabled = e[16] != 0;

        if ((size_t)dst->point_index + dst->point_count > h.point_count)
        {
            NEA_DebugPrint("NEAPattern: entry %ld points past the array",
                           (long)i);
            NEA_PatternBankDelete(bank);
            return NULL;
        }
    }
    bank->entry_count = (int)h.entry_count;

    if (h.name_offset != 0 && h.name_size >= 4)
    {
        bank->names = malloc(h.name_size);
        if (bank->names == NULL)
        {
            NEA_DebugPrint("NEAPattern: not enough memory");
            NEA_PatternBankDelete(bank);
            return NULL;
        }
        memcpy(bank->names, data + h.name_offset, h.name_size);
        bank->name_size = h.name_size;
        bank->name_count = ne_pattern_read16(bank->names);

        // Every offset must land inside the table and the blob must end in a
        // terminator, so the lookup can hand out a plain const char *.
        bool bad = bank->names[h.name_size - 1] != 0;
        for (int i = 0; i < bank->name_count && !bad; i++)
        {
            uint32_t off = ne_pattern_read32(bank->names + 4 + i * 4);
            if (off >= h.name_size)
                bad = true;
        }
        if (bad)
        {
            NEA_DebugPrint("NEAPattern: name table is malformed");
            free(bank->names);
            bank->names = NULL;
            bank->name_size = 0;
            bank->name_count = 0;
        }
    }

    if (ne_pattern_bank_rebuild(bank) != 0)
    {
        NEA_PatternBankDelete(bank);
        return NULL;
    }

    return bank;
}

NEA_PatternBank *NEA_PatternBankLoad(const void *pointer)
{
    NEA_AssertPointer(pointer, "NULL pointer");
    if (pointer == NULL)
        return NULL;

    const uint8_t *data = pointer;
    if (ne_pattern_read32(data) != NEA_PATTERN_MAGIC)
    {
        NEA_DebugPrint("NEAPattern: bad magic");
        return NULL;
    }

    // The header carries every offset, so the end of the file is wherever the
    // last section ends. Nothing before it can be validated without it.
    uint32_t point_end = ne_pattern_read32(data + 16) +
                         ne_pattern_read32(data + 12) * 4;
    uint32_t entry_end = ne_pattern_read32(data + 20) +
                         (uint32_t)ne_pattern_read16(data + 10) *
                             NEA_PATTERN_ENTRY_SIZE;
    uint32_t name_end = ne_pattern_read32(data + 24);
    if (name_end)
        name_end += ne_pattern_read32(data + 28);

    uint32_t size = point_end > entry_end ? point_end : entry_end;
    if (name_end > size)
        size = name_end;

    return ne_pattern_bank_parse(data, size);
}

NEA_PatternBank *NEA_PatternBankLoadFAT(const char *path)
{
    NEA_AssertPointer(path, "NULL pointer");
    if (path == NULL)
        return NULL;

    size_t size = 0;
    char *data = __NEA_FATLoadDataSize(path, &size);
    if (data == NULL)
    {
        NEA_DebugPrint("NEAPattern: could not read %s", path);
        return NULL;
    }

    NEA_PatternBank *bank = ne_pattern_bank_parse((const uint8_t *)data, size);
    free(data);
    return bank;
}

NEA_PatternBank *NEA_PatternBankCreate(int max_entries, int max_points,
                                       int normalize_size)
{
    NEA_PatternBank *bank = ne_pattern_bank_alloc(max_entries, max_points,
                                                  normalize_size);
    if (bank == NULL)
        return NULL;

    bank->growable = true;
    if (ne_pattern_bank_rebuild(bank) != 0)
    {
        NEA_PatternBankDelete(bank);
        return NULL;
    }
    return bank;
}

void NEA_PatternBankDelete(NEA_PatternBank *bank)
{
    if (bank == NULL)
        return;

    for (int i = 0; i < ne_pattern_max_banks; i++)
    {
        if (ne_pattern_banks[i] == bank)
        {
            ne_pattern_banks[i] = NULL;
            break;
        }
    }

    ne_pattern_bank_free_derived(bank);
    free(bank->points);
    free(bank->entries);
    free(bank->names);
    free(bank);
}

int NEA_PatternBankAdd(NEA_PatternBank *bank, const NEA_PatternStrokes *strokes,
                       int code, uint32_t kind)
{
    NEA_AssertPointer(bank, "NULL pointer");
    NEA_AssertPointer(strokes, "NULL pointer");
    if (bank == NULL || strokes == NULL)
        return -1;

    if (!bank->growable)
    {
        NEA_DebugPrint("NEAPattern: bank is read-only");
        return -1;
    }
    if (bank->entry_count >= bank->entry_capacity)
    {
        NEA_DebugPrint("NEAPattern: bank is full");
        return -1;
    }
    if (code < 0 || code > 0xFFFF)
    {
        NEA_DebugPrint("NEAPattern: code %d is out of range", code);
        return -1;
    }

    int room = bank->point_capacity - bank->point_count;
    int written = ne_pattern_normalize(strokes->points, strokes->count,
                                       &bank->points[bank->point_count], room,
                                       bank->normalize_size);
    if (written == 0)
    {
        NEA_DebugPrint("NEAPattern: gesture has nothing to store");
        return -1;
    }

    int nstrokes = 0;
    for (int i = 0; i < written; i++)
    {
        if (ne_pattern_is_pen_up(&bank->points[bank->point_count + i]))
            nstrokes++;
    }

    ne_pattern_entry *entry = &bank->entries[bank->entry_count];
    entry->kind = kind;
    entry->code = (uint16_t)code;
    entry->correction = 0;
    entry->point_index = (uint32_t)bank->point_count;
    entry->point_count = (uint16_t)written;
    entry->stroke_count = (uint16_t)nstrokes;
    entry->enabled = true;

    bank->point_count += written;
    bank->entry_count++;

    if (ne_pattern_bank_rebuild(bank) != 0)
        return -1;

    return bank->entry_count - 1;
}

int NEA_PatternBankRemove(NEA_PatternBank *bank, int entry)
{
    NEA_AssertPointer(bank, "NULL pointer");
    if (bank == NULL)
        return -1;
    if (entry < 0 || entry >= bank->entry_count)
    {
        NEA_DebugPrint("NEAPattern: entry %d is out of range", entry);
        return -1;
    }
    if (!bank->growable)
    {
        NEA_DebugPrint("NEAPattern: bank is read-only");
        return -1;
    }

    ne_pattern_entry *e = &bank->entries[entry];
    uint32_t index = e->point_index;
    uint16_t count = e->point_count;

    memmove(&bank->points[index], &bank->points[index + count],
            sizeof(NEA_PatternPoint) * (bank->point_count - index - count));
    bank->point_count -= count;

    memmove(&bank->entries[entry], &bank->entries[entry + 1],
            sizeof(ne_pattern_entry) * (bank->entry_count - entry - 1));
    bank->entry_count--;

    for (int i = entry; i < bank->entry_count; i++)
        bank->entries[i].point_index -= count;

    return ne_pattern_bank_rebuild(bank);
}

int NEA_PatternBankSetEnabled(NEA_PatternBank *bank, int entry, bool enabled)
{
    NEA_AssertPointer(bank, "NULL pointer");
    if (bank == NULL)
        return -1;
    if (entry < 0 || entry >= bank->entry_count)
    {
        NEA_DebugPrint("NEAPattern: entry %d is out of range", entry);
        return -1;
    }
    bank->entries[entry].enabled = enabled;
    return 0;
}

int NEA_PatternBankSaveFAT(const NEA_PatternBank *bank, const char *path)
{
    NEA_AssertPointer(bank, "NULL pointer");
    NEA_AssertPointer(path, "NULL pointer");
    if (bank == NULL || path == NULL)
        return -1;
    if (bank->entry_count == 0)
    {
        NEA_DebugPrint("NEAPattern: refusing to save an empty bank");
        return -1;
    }

    uint32_t point_offset = NEA_PATTERN_HEADER_SIZE;
    uint32_t entry_offset = point_offset + (uint32_t)bank->point_count * 4;
    uint32_t name_offset = bank->name_size
        ? entry_offset + (uint32_t)bank->entry_count * NEA_PATTERN_ENTRY_SIZE
        : 0;

    size_t size = (name_offset ? name_offset + bank->name_size
                               : entry_offset +
                                 (uint32_t)bank->entry_count *
                                     NEA_PATTERN_ENTRY_SIZE);

    uint8_t *out = calloc(1, size);
    if (out == NULL)
    {
        NEA_DebugPrint("NEAPattern: not enough memory");
        return -1;
    }

    ne_pattern_write32(out + 0, NEA_PATTERN_MAGIC);
    ne_pattern_write16(out + 4, NEA_PATTERN_VERSION);
    ne_pattern_write16(out + 6, 0);
    ne_pattern_write16(out + 8, (uint16_t)bank->normalize_size);
    ne_pattern_write16(out + 10, (uint16_t)bank->entry_count);
    ne_pattern_write32(out + 12, (uint32_t)bank->point_count);
    ne_pattern_write32(out + 16, point_offset);
    ne_pattern_write32(out + 20, entry_offset);
    ne_pattern_write32(out + 24, name_offset);
    ne_pattern_write32(out + 28, bank->name_size);

    for (int i = 0; i < bank->point_count; i++)
    {
        ne_pattern_write16(out + point_offset + i * 4,
                           (uint16_t)bank->points[i].x);
        ne_pattern_write16(out + point_offset + i * 4 + 2,
                           (uint16_t)bank->points[i].y);
    }

    for (int i = 0; i < bank->entry_count; i++)
    {
        const ne_pattern_entry *e = &bank->entries[i];
        uint8_t *d = out + entry_offset + i * NEA_PATTERN_ENTRY_SIZE;
        ne_pattern_write32(d + 0, e->kind);
        ne_pattern_write16(d + 4, e->code);
        ne_pattern_write16(d + 6, (uint16_t)e->correction);
        ne_pattern_write32(d + 8, e->point_index);
        ne_pattern_write16(d + 12, e->point_count);
        ne_pattern_write16(d + 14, e->stroke_count);
        d[16] = e->enabled ? 1 : 0;
    }

    if (name_offset)
        memcpy(out + name_offset, bank->names, bank->name_size);

    FILE *f = fopen(path, "wb");
    if (f == NULL)
    {
        NEA_DebugPrint("NEAPattern: could not open %s", path);
        free(out);
        return -1;
    }

    size_t written = fwrite(out, 1, size, f);
    int ret = (fclose(f) == 0 && written == size) ? 0 : -1;
    if (ret != 0)
        NEA_DebugPrint("NEAPattern: could not write %s", path);

    free(out);
    return ret;
}

int NEA_PatternBankGetEntryCount(const NEA_PatternBank *bank)
{
    NEA_AssertPointer(bank, "NULL pointer");
    if (bank == NULL)
        return -1;
    return bank->entry_count;
}

int NEA_PatternBankGetEntryCode(const NEA_PatternBank *bank, int entry)
{
    NEA_AssertPointer(bank, "NULL pointer");
    if (bank == NULL)
        return -1;
    if (entry < 0 || entry >= bank->entry_count)
        return -1;
    return bank->entries[entry].code;
}

const char *NEA_PatternBankGetCodeName(const NEA_PatternBank *bank, int code)
{
    NEA_AssertPointer(bank, "NULL pointer");
    if (bank == NULL || bank->names == NULL)
        return NULL;
    if (code < 0 || code >= bank->name_count)
        return NULL;

    uint32_t off = ne_pattern_read32(bank->names + 4 + code * 4);
    return (const char *)(bank->names + off);
}

int NEA_PatternBankGetEntryPoints(const NEA_PatternBank *bank, int entry,
                                  const NEA_PatternPoint **points)
{
    NEA_AssertPointer(bank, "NULL pointer");
    if (bank == NULL)
        return -1;
    if (entry < 0 || entry >= bank->entry_count)
        return -1;

    if (points != NULL)
        *points = &bank->points[bank->entries[entry].point_index];
    return bank->entries[entry].point_count;
}

int NEA_PatternBankGetNormalizeSize(const NEA_PatternBank *bank)
{
    NEA_AssertPointer(bank, "NULL pointer");
    if (bank == NULL)
        return -1;
    return bank->normalize_size;
}

//-----------------------------------------------------------------------------
// Recognizer
//-----------------------------------------------------------------------------

/// Grows the normalisation scratch to fit the ink it has been handed.
///
/// The caps a recognizer is created with describe the *reduced* gesture, not
/// how much the stylus may draw, so this cannot be sized up front.
static bool ne_pattern_reserve_norm(NEA_PatternRecognizer *rec, int needed)
{
    if (rec->norm_capacity >= needed)
        return true;

    NEA_PatternPoint *norm = realloc(rec->norm,
                                     sizeof(NEA_PatternPoint) * needed);
    if (norm == NULL)
    {
        NEA_DebugPrint("NEAPattern: not enough memory");
        return false;
    }
    rec->norm = norm;

    uint8_t *keep = realloc(rec->keep, needed);
    if (keep == NULL)
    {
        NEA_DebugPrint("NEAPattern: not enough memory");
        return false;
    }
    rec->keep = keep;

    // The recursive resampler's ranges partition disjoint sub-intervals of a
    // stroke, so at most one per point can be live, two ints each.
    int32_t *stack = realloc(rec->stack, sizeof(int32_t) * needed * 2);
    if (stack == NULL)
    {
        NEA_DebugPrint("NEAPattern: not enough memory");
        return false;
    }
    rec->stack = stack;

    rec->norm_capacity = needed;
    return true;
}

/// Reduces marker terminated normalised points into marker terminated output.
static int ne_pattern_resample_into(NEA_PatternRecognizer *rec,
                                    const NEA_PatternPoint *in, int in_count,
                                    NEA_PatternPoint *out, int out_capacity)
{
    int used = 0;
    int start = 0;

    for (int i = 0; i <= in_count; i++)
    {
        bool end = (i == in_count) || ne_pattern_is_pen_up(&in[i]);
        if (!end)
            continue;

        int n = i - start;
        if (n >= 2)
        {
            const NEA_PatternPoint *p = &in[start];

            switch (rec->method)
            {
                case NEA_PATTERN_RESAMPLE_NONE:
                    ne_pattern_resample_none(p, n, rec->keep, rec->threshold);
                    break;
                case NEA_PATTERN_RESAMPLE_DISTANCE:
                    ne_pattern_resample_distance(p, n, rec->keep,
                                                 rec->threshold);
                    break;
                case NEA_PATTERN_RESAMPLE_ANGLE:
                    ne_pattern_resample_angle(p, n, rec->keep, rec->threshold);
                    break;
                default:
                    ne_pattern_resample_recursive(p, n, rec->keep,
                                                  rec->threshold, rec->stack);
                    break;
            }

            // Count first, because a stroke is taken whole or not at all.
            int kept = 0;
            int16_t lx = 0, ly = 0;
            for (int j = 0; j < n; j++)
            {
                if (!rec->keep[j])
                    continue;
                // Two kept points can coincide once the shape has been
                // fitted into a small space.
                if (kept > 0 && p[j].x == lx && p[j].y == ly)
                    continue;
                lx = p[j].x;
                ly = p[j].y;
                kept++;
            }

            if (kept >= 2 && used + kept + 1 <= out_capacity)
            {
                int written = 0;
                for (int j = 0; j < n; j++)
                {
                    if (!rec->keep[j])
                        continue;
                    if (written > 0 && p[j].x == out[used + written - 1].x &&
                        p[j].y == out[used + written - 1].y)
                        continue;
                    out[used + written] = p[j];
                    written++;
                }
                out[used + written].x = NEA_PATTERN_PEN_UP_X;
                out[used + written].y = NEA_PATTERN_PEN_UP_Y;
                used += written + 1;
            }
            else if (kept >= 2)
            {
                // Out of room. Stop rather than skip, so what is kept is the
                // start of the gesture rather than an arbitrary subset of it.
                break;
            }
        }

        start = i + 1;
        if (i >= in_count)
            break;
    }

    return used;
}

NEA_PatternRecognizer *NEA_PatternRecognizerCreate(int max_points,
                                                   int max_strokes)
{
    if (!ne_pattern_inited)
    {
        NEA_DebugPrint("NEAPattern: system not initialized");
        return NULL;
    }
    if (max_points < 4 || max_strokes < 1)
    {
        NEA_DebugPrint("NEAPattern: recognizer capacities are too small");
        return NULL;
    }

    int slot = -1;
    for (int i = 0; i < ne_pattern_max_recognizers; i++)
    {
        if (ne_pattern_recognizers[i] == NULL)
        {
            slot = i;
            break;
        }
    }
    if (slot < 0)
    {
        NEA_DebugPrint("NEAPattern: no free recognizer slots");
        return NULL;
    }

    NEA_PatternRecognizer *rec = calloc(1, sizeof(NEA_PatternRecognizer));
    if (rec == NULL)
    {
        NEA_DebugPrint("NEAPattern: not enough memory");
        return NULL;
    }

    rec->max_points = max_points;
    rec->max_strokes = max_strokes;
    rec->algo = NEA_PATTERN_STANDARD;
    rec->method = NEA_PATTERN_RESAMPLE_RECURSIVE;
    rec->threshold = 2;
    rec->lf_threshold = 0;
    rec->lf_ratio = 3;

    rec->fpts = malloc(sizeof(NEA_PatternPoint) * max_points);
    rec->fang = malloc(sizeof(uint16_t) * max_points);
    rec->fratio = malloc(sizeof(int32_t) * max_points);
    rec->out_points = malloc(sizeof(NEA_PatternPoint) * max_points);
    rec->strokes = malloc(sizeof(ne_pattern_stroke) * max_strokes);
    rec->raws = malloc(sizeof(int32_t) * max_strokes);
    rec->weights = malloc(sizeof(int32_t) * max_strokes);

    if (rec->fpts == NULL || rec->fang == NULL || rec->fratio == NULL ||
        rec->out_points == NULL || rec->strokes == NULL ||
        rec->raws == NULL || rec->weights == NULL)
    {
        NEA_DebugPrint("NEAPattern: not enough memory");
        NEA_PatternRecognizerDelete(rec);
        return NULL;
    }

    ne_pattern_recognizers[slot] = rec;
    return rec;
}

void NEA_PatternRecognizerDelete(NEA_PatternRecognizer *rec)
{
    if (rec == NULL)
        return;

    for (int i = 0; i < ne_pattern_max_recognizers; i++)
    {
        if (ne_pattern_recognizers[i] == rec)
        {
            ne_pattern_recognizers[i] = NULL;
            break;
        }
    }

    free(rec->fpts);
    free(rec->fang);
    free(rec->fratio);
    free(rec->out_points);
    free(rec->strokes);
    free(rec->raws);
    free(rec->weights);
    free(rec->norm);
    free(rec->keep);
    free(rec->stack);
    free(rec->dp_sum);
    free(rec->dp_cnt);
    free(rec);
}

int NEA_PatternRecognizerSetAlgorithm(NEA_PatternRecognizer *rec,
                                      NEA_PatternAlgorithm algo)
{
    NEA_AssertPointer(rec, "NULL pointer");
    if (rec == NULL)
        return -1;

    if (algo != NEA_PATTERN_LIGHT && algo != NEA_PATTERN_STANDARD &&
        algo != NEA_PATTERN_FINE)
    {
        NEA_DebugPrint("NEAPattern: unknown algorithm %d", (int)algo);
        return -1;
    }

    // Fine's tables are what makes it expensive in RAM, so they are allocated
    // the first time it is asked for and not before.
    if (algo == NEA_PATTERN_FINE && rec->dp_sum == NULL)
    {
        size_t cells = (size_t)rec->max_points * rec->max_points;
        rec->dp_sum = malloc(sizeof(int32_t) * cells);
        rec->dp_cnt = malloc(sizeof(int32_t) * cells);
        if (rec->dp_sum == NULL || rec->dp_cnt == NULL)
        {
            NEA_DebugPrint("NEAPattern: not enough memory for the Fine tables");
            free(rec->dp_sum);
            free(rec->dp_cnt);
            rec->dp_sum = NULL;
            rec->dp_cnt = NULL;
            return -1;
        }
    }

    rec->algo = algo;
    return 0;
}

int NEA_PatternRecognizerSetResample(NEA_PatternRecognizer *rec,
                                     NEA_PatternResampleMethod method,
                                     int threshold)
{
    NEA_AssertPointer(rec, "NULL pointer");
    if (rec == NULL)
        return -1;

    if (method != NEA_PATTERN_RESAMPLE_NONE &&
        method != NEA_PATTERN_RESAMPLE_DISTANCE &&
        method != NEA_PATTERN_RESAMPLE_ANGLE &&
        method != NEA_PATTERN_RESAMPLE_RECURSIVE)
    {
        NEA_DebugPrint("NEAPattern: unknown resample method %d", (int)method);
        return -1;
    }

    rec->method = method;
    rec->threshold = threshold;
    return 0;
}

int NEA_PatternRecognizerSetLengthFilter(NEA_PatternRecognizer *rec,
                                         int32_t threshold, int ratio)
{
    NEA_AssertPointer(rec, "NULL pointer");
    if (rec == NULL)
        return -1;

    rec->lf_threshold = threshold;
    rec->lf_ratio = ratio > 0 ? ratio : 3;
    return 0;
}

int NEA_PatternRecognize(NEA_PatternRecognizer *rec, NEA_PatternBank *bank,
                         const NEA_PatternStrokes *strokes, uint32_t kind_mask,
                         NEA_PatternResult *results, int max_results)
{
    NEA_AssertPointer(rec, "NULL pointer");
    NEA_AssertPointer(bank, "NULL pointer");
    NEA_AssertPointer(strokes, "NULL pointer");
    NEA_AssertPointer(results, "NULL pointer");
    if (rec == NULL || bank == NULL || strokes == NULL || results == NULL)
        return -1;
    if (max_results < 1)
        return 0;

    rec->out_count = 0;
    rec->nstrokes = 0;

    if (strokes->count < 2)
        return 0;

    // Normalising can add one marker to close a stroke the caller left open.
    if (!ne_pattern_reserve_norm(rec, strokes->count + 2))
        return -1;

    int n = ne_pattern_normalize(strokes->points, strokes->count, rec->norm,
                                 rec->norm_capacity, bank->normalize_size);
    if (n == 0)
        return 0;

    rec->out_count = ne_pattern_resample_into(rec, rec->norm, n,
                                              rec->out_points, rec->max_points);
    if (rec->out_count == 0)
        return 0;

    rec->nstrokes = ne_pattern_extract(rec->out_points, rec->out_count,
                                       rec->fpts, rec->fang, rec->fratio,
                                       rec->max_points, rec->strokes,
                                       rec->max_strokes, NULL);
    if (rec->nstrokes == 0)
        return 0;

    int dbl_w = bank->normalize_size * 2;
    int32_t lf_threshold = rec->lf_threshold > 0
        ? rec->lf_threshold : (int32_t)(bank->normalize_size << 12);

    int count = 0;

    for (int i = 0; i < bank->entry_count; i++)
    {
        const ne_pattern_entry *entry = &bank->entries[i];

        if (!entry->enabled)
            continue;
        if (!(entry->kind & kind_mask))
            continue;
        if (entry->stroke_valid == 0)
            continue;

        const ne_pattern_stroke *proto = &bank->strokes[entry->stroke_first];
        int32_t score;

        switch (rec->algo)
        {
            case NEA_PATTERN_LIGHT:
            {
                int32_t worst = count >= max_results
                    ? results[count - 1].score : -1;
                score = ne_pattern_match_light(proto, entry->stroke_valid,
                                               rec->strokes, rec->nstrokes,
                                               worst);
                break;
            }

            case NEA_PATTERN_FINE:
                score = ne_pattern_match_fine(proto, entry->stroke_valid,
                                              rec->strokes, rec->nstrokes,
                                              dbl_w, lf_threshold,
                                              rec->lf_ratio, rec->dp_sum,
                                              rec->dp_cnt, rec->raws,
                                              rec->weights);
                break;

            case NEA_PATTERN_STANDARD:
            default:
                score = ne_pattern_match_standard(proto, entry->stroke_valid,
                                                  rec->strokes, rec->nstrokes,
                                                  dbl_w, rec->raws,
                                                  rec->weights);
                break;
        }

        if (score <= 0)
            continue;

        // The correction biases an entry toward 1, which is how a prototype
        // that is known to be hard to draw well is given a hand.
        int32_t c = entry->correction;
        if (c)
        {
            score = (int32_t)(((int64_t)score * (4096 - c)) >> 12) + c;
            if (score > 4096)
                score = 4096;
        }

        // Strictly greater, so an earlier entry keeps its place on a tie and
        // the ranking does not depend on an accident of iteration order.
        int pos = count;
        while (pos > 0 && score > results[pos - 1].score)
            pos--;
        if (pos >= max_results)
            continue;

        int last = count < max_results ? count : max_results - 1;
        for (int j = last; j > pos; j--)
            results[j] = results[j - 1];

        results[pos].entry = i;
        results[pos].code = entry->code;
        results[pos].score = score;

        if (count < max_results)
            count++;
    }

    return count;
}

int NEA_PatternRecognizerGetInputPoints(const NEA_PatternRecognizer *rec,
                                        const NEA_PatternPoint **points)
{
    NEA_AssertPointer(rec, "NULL pointer");
    if (rec == NULL)
        return -1;
    if (points != NULL)
        *points = rec->out_points;
    return rec->out_count;
}

//-----------------------------------------------------------------------------
// System
//-----------------------------------------------------------------------------

int NEA_PatternSystemReset(int max_banks)
{
    if (ne_pattern_inited)
        NEA_PatternSystemEnd();

    if (max_banks < 1)
        max_banks = NEA_DEFAULT_PATTERN_BANKS;

    int max_recognizers = NEA_DEFAULT_PATTERN_RECOGNIZERS;
    if (max_recognizers < max_banks)
        max_recognizers = max_banks;

    ne_pattern_banks = calloc(max_banks, sizeof(NEA_PatternBank *));
    ne_pattern_recognizers = calloc(max_recognizers,
                                    sizeof(NEA_PatternRecognizer *));

    if (ne_pattern_banks == NULL || ne_pattern_recognizers == NULL)
    {
        NEA_DebugPrint("NEAPattern: not enough memory");
        free(ne_pattern_banks);
        free(ne_pattern_recognizers);
        ne_pattern_banks = NULL;
        ne_pattern_recognizers = NULL;
        return -1;
    }

    ne_pattern_max_banks = max_banks;
    ne_pattern_max_recognizers = max_recognizers;
    ne_pattern_inited = true;
    return 0;
}

void NEA_PatternSystemEnd(void)
{
    if (!ne_pattern_inited)
        return;

    for (int i = 0; i < ne_pattern_max_banks; i++)
    {
        if (ne_pattern_banks[i] != NULL)
            NEA_PatternBankDelete(ne_pattern_banks[i]);
    }
    for (int i = 0; i < ne_pattern_max_recognizers; i++)
    {
        if (ne_pattern_recognizers[i] != NULL)
            NEA_PatternRecognizerDelete(ne_pattern_recognizers[i]);
    }

    free(ne_pattern_banks);
    free(ne_pattern_recognizers);
    ne_pattern_banks = NULL;
    ne_pattern_recognizers = NULL;
    ne_pattern_max_banks = 0;
    ne_pattern_max_recognizers = 0;
    ne_pattern_inited = false;
}
