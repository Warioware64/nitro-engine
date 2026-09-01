#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
#
# Copyright (c) 2026 Warioware64
#
# Reader, writer and evaluator for .neaptn stylus pattern banks.
#
# source/NEAPattern.c reads exactly what this writes, and computes exactly the
# integers the evaluator below computes -- tests/pattern_eval is what keeps the
# two honest. Every operation here is integer, and every one of them is chosen
# so the ARM9 can perform it identically:
#
#   mulf32(a, b)  ==  (a * b) >> 12          both arithmetic-shift, both floor
#   divf32(a, b)  ==  trunc((a << 12) / b)   the DS divider truncates to zero
#   sqrtf32(a)    ==  isqrt(a << 12)         the DS sqrt unit floors
#
# atan2 is the one primitive libnds does not have, and sinLerp()/cosLerp() are
# no help because their table lives inside libnds.a where nothing on the host
# can mirror it. So the table is generated from here into
# source/NEAPatternAtan.h and both sides run the same octant fold over it.
#
#
# THE .neaptn FORMAT
# ==================
#
# Little endian, 4 byte aligned throughout.
#
#   Header, 32 bytes
#      0  char[4]  'N','P','T','N'
#      4  u16      version, currently 1
#      6  u16      flags, currently 0
#      8  u16      normalize_size    the bank's coordinate space, e.g. 64
#     10  u16      entry_count
#     12  u32      point_count
#     16  u32      point_array_offset
#     20  u32      entry_array_offset
#     24  u32      name_table_offset  0 when the bank carries no names
#     28  u32      name_table_size
#
#   Point array, point_count * 4 bytes
#     s16 x, s16 y. (-1, -1) is a pen up marker and terminates every stroke,
#     including the last stroke of an entry. No real point may have x == -1,
#     which is what makes the marker unambiguous.
#
#   Entry array, entry_count * 20 bytes
#     u32 kind         bitmask, filtered against kind_mask when recognising
#     u16 code         index into the name table; entries may share a code, and
#                      normally do -- that is how one character gets several
#                      prototypes for its stroke order variants
#     s16 correction   4096 = 1.0, biases the score toward 1
#     u32 point_index  first point of this entry in the point array
#     u16 point_count  markers included
#     u16 stroke_count
#     u8  enabled
#     u8  reserved[3]
#
#   Name table, optional
#     u16 count, u16 pad, u32 offset[count], then a UTF-8 blob. Each offset is
#     relative to the start of the name table and points at a NUL terminated
#     string. Retail kept these host side in a generated table; keeping them in
#     the bank means an application can print what it recognised without a
#     parallel array that can fall out of sync.
#
# Every coordinate in the point array is in [0, normalize_size - 1] on both
# axes. write_bank() refuses to write anything else and read_bank() refuses to
# load it.

import math
import struct

MAGIC = b"NPTN"
VERSION = 1

HEADER_SIZE = 32
ENTRY_SIZE = 20

FX32_SHIFT = 12
FX32_ONE = 1 << FX32_SHIFT

PEN_UP_X = -1
PEN_UP_Y = -1

DEFAULT_NORMALIZE_SIZE = 64

# Algorithms. The numbering is the NEA_PatternAlgorithm enum, and 3 is left
# free for the Superfine equivalent should anyone want it.
ALGO_LIGHT = 0
ALGO_STANDARD = 1
ALGO_FINE = 2

ALGO_NAMES = {ALGO_LIGHT: "light", ALGO_STANDARD: "standard", ALGO_FINE: "fine"}

# Resampling methods, the NEA_PatternResampleMethod enum.
RESAMPLE_NONE = 0
RESAMPLE_DISTANCE = 1
RESAMPLE_ANGLE = 2
RESAMPLE_RECURSIVE = 3

RESAMPLE_NAMES = {
    RESAMPLE_NONE: "none",
    RESAMPLE_DISTANCE: "distance",
    RESAMPLE_ANGLE: "angle",
    RESAMPLE_RECURSIVE: "recursive",
}

DEFAULT_RESAMPLE_METHOD = RESAMPLE_RECURSIVE
DEFAULT_RESAMPLE_THRESHOLD = 2

# A candidate point must be at least this far, city block, from the last point
# the angle resampler kept. Without it a slow hand at a corner emits a run of
# points that are all "a big direction change" from each other.
ANGLE_MIN_SEPARATION = 2

# Fine's length filter. A stroke pair is rejected before the DP runs when both
# are longer than the threshold and one is more than RATIO times the other.
DEFAULT_LENGTH_FILTER_RATIO = 3

MAX_POINTS_HARD_LIMIT = 4096


class PatternError(Exception):
    pass


# ---------------------------------------------------------------------------
# Fixed point, mirroring the ARM9 exactly
# ---------------------------------------------------------------------------

def mulf32(a, b):
    """(int64)a * b >> 12, the libnds mulf32()."""
    return (a * b) >> FX32_SHIFT


def trunc_div(a, b):
    """Integer division truncating toward zero, as C and the DS divider do."""
    if b == 0:
        raise ZeroDivisionError("pattern_format: divide by zero")
    q = abs(a) // abs(b)
    return -q if (a < 0) != (b < 0) else q


def divf32(a, b):
    """(a << 12) / b on the DS divider, the libnds divf32()."""
    return trunc_div(a << FX32_SHIFT, b)


def sqrtf32(a):
    """floor(sqrt(a << 12)), the libnds sqrtf32(). a must not be negative."""
    if a < 0:
        raise ValueError("pattern_format: sqrtf32 of a negative value")
    return math.isqrt(a << FX32_SHIFT)


# ---------------------------------------------------------------------------
# Angles. 65536 is a full turn, measured clockwise from +x because y runs down.
# ---------------------------------------------------------------------------

ATAN_TAB_SHIFT = 8
ATAN_TAB_SIZE = (1 << ATAN_TAB_SHIFT) + 1


def _build_atan_table():
    # atan(i / 256) in 16 bit angle units. The last entry is atan(1) = 45
    # degrees = 65536 / 8 = 8192.
    tab = []
    for i in range(ATAN_TAB_SIZE):
        a = math.atan2(i, 1 << ATAN_TAB_SHIFT)
        tab.append(int(round(a * 65536.0 / (2.0 * math.pi))))
    return tab


ATAN_TAB = _build_atan_table()


def atan2_16(y, x):
    """Angle of the vector (x, y) in 16 bit units, 0 <= result < 65536.

    Screen space: +x is right, +y is down, and the angle grows clockwise.
    """
    if x == 0 and y == 0:
        return 0

    ax = -x if x < 0 else x
    ay = -y if y < 0 else y

    if ax >= ay:
        # First octant of the quadrant: tan = ay / ax <= 1.
        a = ATAN_TAB[(ay << ATAN_TAB_SHIFT) // ax]
    else:
        # Second octant: reflect about the 45 degree line.
        a = 16384 - ATAN_TAB[(ax << ATAN_TAB_SHIFT) // ay]

    if x >= 0:
        return a if y >= 0 else (65536 - a) & 0xFFFF
    return (32768 - a) if y >= 0 else (32768 + a) & 0xFFFF


def angle_diff(a, b):
    """Shortest distance between two 16 bit angles, 0 .. 32768."""
    d = ((a - b + 32768) & 0xFFFF) - 32768
    return -d if d < 0 else d


def city_block(p, q):
    dx = p[0] - q[0]
    dy = p[1] - q[1]
    if dx < 0:
        dx = -dx
    if dy < 0:
        dy = -dy
    return dx + dy


def point_distance(p, q):
    """Euclidean distance as f32, computed the way the ARM9 computes it.

    The squared distance is a plain integer, so it has to be shifted into f32
    before the root is taken. Coordinates are normalised into [0, 511] by the
    time this runs, so the shifted value stays inside 32 bits.
    """
    dx = p[0] - q[0]
    dy = p[1] - q[1]
    return sqrtf32((dx * dx + dy * dy) << FX32_SHIFT)


# ---------------------------------------------------------------------------
# Strokes
#
# Two representations. Inside the evaluator a gesture is a list of strokes and
# a stroke is a list of (x, y) tuples, which is the shape everything here wants
# to work with. On the wire, and across the C API, it is one flat point array
# with (-1, -1) terminating each stroke, which is the shape the DS wants so it
# can hold a whole dictionary in one allocation.
# ---------------------------------------------------------------------------

def points_to_strokes(points):
    """Flat marker terminated point list -> list of strokes."""
    strokes = []
    current = []
    for x, y in points:
        if x == PEN_UP_X:
            if current:
                strokes.append(current)
            current = []
        else:
            current.append((x, y))
    if current:
        strokes.append(current)
    return strokes


def strokes_to_points(strokes):
    """List of strokes -> flat point list, every stroke marker terminated."""
    points = []
    for stroke in strokes:
        for p in stroke:
            points.append((int(p[0]), int(p[1])))
        points.append((PEN_UP_X, PEN_UP_Y))
    return points


def count_points(strokes):
    return sum(len(s) + 1 for s in strokes)


# ---------------------------------------------------------------------------
# Normalisation
#
# Fit the whole gesture's bounding box into normalize_size, uniformly, keeping
# the aspect ratio and centring on the short axis. That is what makes a pattern
# match at any size and anywhere on the screen. Points that land on top of
# their predecessor are dropped, and a stroke left with fewer than two points
# is dropped whole -- it carries no direction, so it cannot be scored.
# ---------------------------------------------------------------------------

def _normalize_pass(strokes, normalize_size):
    """One fit. Returns (strokes, dropped)."""
    if normalize_size <= 0:
        raise ValueError("pattern_format: normalize_size must be positive")

    pts = [p for s in strokes for p in s]
    if not pts:
        return [], 0

    x1 = min(p[0] for p in pts)
    x2 = max(p[0] for p in pts)
    y1 = min(p[1] for p in pts)
    y2 = max(p[1] for p in pts)

    wx = x2 - x1
    wy = y2 - y1
    w = wx if wx >= wy else wy
    if w <= 0:
        # Every sample landed on the same point. There is no shape here.
        return [], len(strokes)

    limit = normalize_size - 1

    # 16.16, mapping the long axis onto [0, limit]. The rounding term is what
    # makes that exact rather than a pixel short.
    scale = (limit << 16) // w

    # Scale first, then centre the short axis, in output units. Centring
    # before the scale -- which is the obvious way round -- lets the centring
    # error be scaled along with everything else, and the fit stops being
    # idempotent: re-fitting an already fitted prototype walks it sideways.
    # That matters because re-importing an exported bank does exactly that.
    sx = (wx * scale + 32768) >> 16
    sy = (wy * scale + 32768) >> 16
    cx = (limit - sx) // 2
    cy = (limit - sy) // 2

    out = []
    for stroke in strokes:
        acc = []
        for px, py in stroke:
            x = (((px - x1) * scale + 32768) >> 16) + cx
            y = (((py - y1) * scale + 32768) >> 16) + cy
            if x < 0:
                x = 0
            elif x > limit:
                x = limit
            if y < 0:
                y = 0
            elif y > limit:
                y = limit
            if acc and acc[-1] == (x, y):
                continue
            acc.append((x, y))
        if len(acc) >= 2:
            out.append(acc)
    return out, len(strokes) - len(out)


def normalize(strokes, normalize_size):
    """Fits a gesture into [0, normalize_size - 1], keeping its shape.

    Returns a new list of strokes, or [] when there is nothing to fit.

    The fit is iterated because dropping a stroke shrinks what is left: a
    gesture whose stray dot collapses away has a smaller bounding box than the
    gesture did, and fitting to the box the dot was part of would leave the
    survivors short of the edge. Iterating settles on a fixed point -- the
    stroke count only ever falls, so it terminates quickly -- and that fixed
    point is what makes re-fitting an already fitted prototype a no-op.
    """
    out, dropped = _normalize_pass(strokes, normalize_size)
    guard = len(strokes) + 1
    while out and dropped and guard > 0:
        out, dropped = _normalize_pass(out, normalize_size)
        guard -= 1
    return out


# ---------------------------------------------------------------------------
# Resampling
#
# Each of these takes one stroke and returns the indices it keeps, ascending,
# always including the first and last point.
# ---------------------------------------------------------------------------

def resample_none(stroke, threshold):
    return list(range(len(stroke)))


def resample_distance(stroke, threshold):
    if threshold < 1:
        threshold = 1
    kept = [0]
    acc = 0
    for i in range(1, len(stroke) - 1):
        acc += city_block(stroke[i - 1], stroke[i])
        if acc >= threshold:
            kept.append(i)
            acc = 0
    kept.append(len(stroke) - 1)
    return kept


def resample_angle(stroke, threshold):
    if threshold < 1:
        threshold = 1
    kept = [0]
    last = 0
    ref = atan2_16(stroke[1][1] - stroke[0][1], stroke[1][0] - stroke[0][0])
    for i in range(1, len(stroke) - 1):
        a = atan2_16(stroke[i + 1][1] - stroke[i][1],
                     stroke[i + 1][0] - stroke[i][0])
        if angle_diff(a, ref) < threshold:
            continue
        if city_block(stroke[last], stroke[i]) < ANGLE_MIN_SEPARATION:
            continue
        kept.append(i)
        last = i
        ref = a
    kept.append(len(stroke) - 1)
    return kept


def resample_recursive(stroke, threshold):
    """Ramer-Douglas-Peucker, integer, with no divide anywhere.

    The perpendicular distance of C from the line AB is
    |cross(B - A, C - A)| / |B - A|, so comparing it against a threshold t is
    comparing cross^2 against t^2 * |B - A|^2, which needs no division and no
    square root.
    """
    if threshold < 1:
        threshold = 1
    n = len(stroke)
    keep = [False] * n
    keep[0] = True
    keep[n - 1] = True

    tsq = threshold * threshold
    # An explicit stack, because the ARM9 side must not recurse.
    stack = [(0, n - 1)]
    while stack:
        lo, hi = stack.pop()
        if hi - lo < 2:
            continue
        ax, ay = stroke[lo]
        bx, by = stroke[hi]
        ux = bx - ax
        uy = by - ay
        span = ux * ux + uy * uy

        best = -1
        best_dist = 0
        for i in range(lo + 1, hi):
            cx, cy = stroke[i]
            if span == 0:
                # A degenerate span: fall back to the distance from A.
                d = city_block(stroke[i], stroke[lo])
                d = d * d
            else:
                cross = ux * (cy - ay) - uy * (cx - ax)
                if cross < 0:
                    cross = -cross
                d = cross
            if d > best_dist:
                best_dist = d
                best = i

        if best < 0:
            continue
        if span == 0:
            over = best_dist >= tsq
        else:
            over = best_dist * best_dist >= tsq * span
        if over:
            keep[best] = True
            stack.append((lo, best))
            stack.append((best, hi))

    return [i for i in range(n) if keep[i]]


RESAMPLERS = {
    RESAMPLE_NONE: resample_none,
    RESAMPLE_DISTANCE: resample_distance,
    RESAMPLE_ANGLE: resample_angle,
    RESAMPLE_RECURSIVE: resample_recursive,
}


def resample(strokes, method, threshold, max_points=0):
    """Resample every stroke, dropping any left with fewer than two points.

    max_points, when non-zero, is the budget for the whole gesture including
    the pen up markers the flat form would need. Strokes are taken in order
    until it runs out, which is what the runtime does when a gesture overruns
    the recognizer it was given.
    """
    fn = RESAMPLERS.get(method)
    if fn is None:
        raise ValueError("pattern_format: unknown resample method %r" % method)

    out = []
    used = 0
    for stroke in strokes:
        if len(stroke) < 2:
            continue
        kept = [stroke[i] for i in fn(stroke, threshold)]
        # Resampling can leave duplicates when two kept points coincide.
        dedup = [kept[0]]
        for p in kept[1:]:
            if p != dedup[-1]:
                dedup.append(p)
        if len(dedup) < 2:
            continue
        if max_points:
            need = len(dedup) + 1
            if used + need > max_points:
                break
            used += need
        out.append(dedup)
    return out


# ---------------------------------------------------------------------------
# Features
#
# Everything the matchers read, computed once per pattern. A pattern is a dict
# so the editor can hold it next to the artwork-free bank entry it came from.
# ---------------------------------------------------------------------------

def extract(strokes):
    """Feature extraction. Returns None when the gesture cannot be scored."""
    out = []
    total = 0

    for stroke in strokes:
        n = len(stroke)
        if n < 2:
            continue

        seg = [0] * n
        ang = [0] * n
        length = 0
        for i in range(1, n):
            seg[i] = point_distance(stroke[i - 1], stroke[i])
            length += seg[i]
            ang[i] = atan2_16(stroke[i][1] - stroke[i - 1][1],
                              stroke[i][0] - stroke[i - 1][0])
        if length <= 0:
            continue

        # Segment 0 does not exist. Giving it segment 1's direction is the same
        # as reflecting p0 through p1, and it keeps the two arrays the same
        # length as the point array, which the walk below relies on.
        ang[0] = ang[1]

        # Share of the stroke each segment covers. The last one absorbs the
        # truncation so the shares sum to exactly 1.0, which is what lets two
        # strokes be walked in lockstep without either running out early.
        ratio = [0] * n
        acc = 0
        for i in range(1, n - 1):
            ratio[i] = (seg[i] << FX32_SHIFT) // length
            acc += ratio[i]
        ratio[n - 1] = FX32_ONE - acc

        out.append({
            "pts": stroke,
            "seg": seg,
            "ang": ang,
            "ratio": ratio,
            "len": length,
        })
        total += length

    if not out or total <= 0:
        return None

    # Share of the whole gesture each stroke covers, same trick.
    acc = 0
    for s in out[:-1]:
        s["sratio"] = (s["len"] << FX32_SHIFT) // total
        acc += s["sratio"]
    out[-1]["sratio"] = FX32_ONE - acc

    return {"strokes": out, "len": total, "nstrokes": len(out)}


def make_input(raw_strokes, normalize_size, method=DEFAULT_RESAMPLE_METHOD,
               threshold=DEFAULT_RESAMPLE_THRESHOLD, max_points=0):
    """Raw screen space strokes -> a scoreable pattern, or None."""
    norm = normalize(raw_strokes, normalize_size)
    if not norm:
        return None
    red = resample(norm, method, threshold, max_points)
    if not red:
        return None
    return extract(red)


# ---------------------------------------------------------------------------
# The arc length walk
#
# Both strokes are parameterised over [0, 1] by arc length. Merging the two
# breakpoint sets gives a sequence of intervals, each of which sits inside one
# segment of each stroke; the score of that segment pair is weighted by the
# interval's length. It is what makes a fast scribble and a slow one score the
# same: only shape is left, timing is gone.
# ---------------------------------------------------------------------------

def _walk(pa, pb, cell):
    """Sum of interval_share * cell(ia, ib) over the merged parameterisation."""
    ra = pa["ratio"]
    rb = pb["ratio"]
    na = len(ra)
    nb = len(rb)

    ia = 1
    ib = 1
    rema = ra[1]
    remb = rb[1]
    total = 0

    while ia < na and ib < nb:
        step = rema if rema <= remb else remb
        if step:
            total += step * cell(ia, ib)
        rema -= step
        remb -= step
        if rema == 0:
            ia += 1
            rema = ra[ia] if ia < na else 0
        if remb == 0:
            ib += 1
            remb = rb[ib] if ib < nb else 0

    return total


# ---------------------------------------------------------------------------
# Light: angle only
#
# Integrates the absolute angle difference along the arc length. It knows
# nothing about where the strokes sit relative to each other, so it cannot
# tell a T from a plus -- and it needs no scratch memory at all, which is why
# it is worth keeping for single stroke gestures.
# ---------------------------------------------------------------------------

def match_light(proto, inp, worst=None):
    if proto["nstrokes"] != inp["nstrokes"]:
        return 0

    # The running total that would already be worse than the worst result we
    # are keeping. Passing it means this entry cannot enter the ranking, so
    # abandoning it changes nothing but the time spent.
    border = None
    if worst is not None:
        border = (FX32_ONE - worst) << 3

    err = 0
    for ps, qs in zip(proto["strokes"], inp["strokes"]):
        pa = ps["ang"]
        qa = qs["ang"]
        total = _walk(ps, qs, lambda i, j: angle_diff(pa[i], qa[j]))
        err += mulf32(total >> FX32_SHIFT, ps["sratio"])
        if border is not None and err > border:
            return 0

    score = FX32_ONE - (err >> 3)
    if score < 0:
        return 0
    return score if score <= FX32_ONE else FX32_ONE


# ---------------------------------------------------------------------------
# Standard: angle weighted by position
#
# Same walk, but each segment pair's angle agreement is multiplied by how close
# the two points are. Relative stroke position now matters, so T and plus come
# apart, and it still needs no scratch memory.
# ---------------------------------------------------------------------------

def _cell_score(pp, qp, pa, qa, dbl_w):
    """0 .. 256 * dbl_w. Perfect agreement, coincident points, is the maximum."""
    def cell(i, j):
        ang = (32768 - angle_diff(pa[i], qa[j])) >> 7
        dist = dbl_w - city_block(pp[i], qp[j])
        if dist < 0:
            dist = 0
        return ang * dist
    return cell


def _combine(raw_strokes, weights, dbl_w):
    """Weighted mean of per stroke scores, mapped onto 0 .. 4096."""
    wsum = 0
    acc = 0
    for raw, w in zip(raw_strokes, weights):
        acc += mulf32(raw, w)
        wsum += w
    if wsum <= 0:
        return 0
    denom = (wsum * dbl_w) >> 4
    if denom <= 0:
        return 0
    score = trunc_div(acc << FX32_SHIFT, denom)
    if score < 0:
        return 0
    return score if score <= FX32_ONE else FX32_ONE


def match_standard(proto, inp, dbl_w):
    if proto["nstrokes"] != inp["nstrokes"]:
        return 0

    raws = []
    weights = []
    for ps, qs in zip(proto["strokes"], inp["strokes"]):
        cell = _cell_score(ps["pts"], qs["pts"], ps["ang"], qs["ang"], dbl_w)
        total = _walk(ps, qs, cell)
        raws.append(total >> FX32_SHIFT)
        # A stroke that is a big part of either pattern matters. Taking the max
        # rather than the prototype's share alone stops a long input stroke
        # matched against a short prototype one from being scored as a detail.
        pw = ps["sratio"]
        qw = qs["sratio"]
        weights.append(pw if pw >= qw else qw)

    return _combine(raws, weights, dbl_w)


# ---------------------------------------------------------------------------
# Fine: elastic matching
#
# The fixed arc length walk assumes the two strokes are traced at proportional
# speeds. A hand that lingers on one part of a character breaks that. So pair
# the points with dynamic programming instead, maximising the *mean* pair
# score, which stops the path from being rewarded for simply being long.
#
# Scratch is two int arrays of M * M, M being the larger point count, so it is
# the one algorithm with a memory cost worth thinking about: 12.5 KB at
# M = 40.
# ---------------------------------------------------------------------------

def match_fine(proto, inp, dbl_w, lf_threshold, lf_ratio):
    if proto["nstrokes"] != inp["nstrokes"]:
        return 0

    raws = []
    weights = []
    for ps, qs in zip(proto["strokes"], inp["strokes"]):
        pw = ps["sratio"]
        qw = qs["sratio"]
        weights.append(pw if pw >= qw else qw)
        raws.append(_fine_stroke(ps, qs, dbl_w, lf_threshold, lf_ratio))

    return _combine(raws, weights, dbl_w)


def _fine_stroke(ps, qs, dbl_w, lf_threshold, lf_ratio):
    pl = ps["len"]
    ql = qs["len"]
    if pl > lf_threshold or ql > lf_threshold:
        if pl * lf_ratio < ql or ql * lf_ratio < pl:
            return 0

    pp = ps["pts"]
    qp = qs["pts"]
    pa = ps["ang"]
    qa = qs["ang"]
    cell = _cell_score(pp, qp, pa, qa, dbl_w)

    n = len(pp)
    m = len(qp)

    # sums[i][j] and cnts[i][j]: the best path to the pair (i, j), stored as a
    # sum and a count so the comparison can be on the mean without dividing.
    # Row 0 and column 0 are the real first points, not a padding row: the two
    # strokes start together and end together by construction.
    sums = [[0] * m for _ in range(n)]
    cnts = [[0] * m for _ in range(n)]

    sums[0][0] = cell(0, 0)
    cnts[0][0] = 1

    for i in range(n):
        for j in range(m):
            if i == 0 and j == 0:
                continue
            bs = -1
            bc = 1
            # Three predecessors: consume a prototype point, an input point, or
            # both. Compare candidates on sum_a / cnt_a > sum_b / cnt_b, cross
            # multiplied so it stays integer.
            for pi, pj in ((i - 1, j), (i, j - 1), (i - 1, j - 1)):
                if pi < 0 or pj < 0:
                    continue
                cs = sums[pi][pj]
                cc = cnts[pi][pj]
                if cc == 0:
                    continue
                if bs < 0 or cs * bc > bs * cc:
                    bs = cs
                    bc = cc
            if bs < 0:
                continue
            sums[i][j] = bs + cell(i, j)
            cnts[i][j] = bc + 1

    cnt = cnts[n - 1][m - 1]
    if cnt <= 0:
        return 0
    return trunc_div(sums[n - 1][m - 1], cnt)


# ---------------------------------------------------------------------------
# Recognition
# ---------------------------------------------------------------------------

def recognize(bank, raw_strokes, algo=ALGO_STANDARD,
              method=DEFAULT_RESAMPLE_METHOD,
              threshold=DEFAULT_RESAMPLE_THRESHOLD,
              kind_mask=0xFFFFFFFF, max_results=5, max_points=0,
              length_filter_threshold=None,
              length_filter_ratio=DEFAULT_LENGTH_FILTER_RATIO):
    """Rank a gesture against a bank.

    Returns a list of {'entry', 'code', 'name', 'score'}, best first, at most
    max_results long. An empty list means nothing scored above zero, or the
    gesture could not be turned into a pattern at all.
    """
    nsize = bank["normalize_size"]
    inp = make_input(raw_strokes, nsize, method, threshold, max_points)
    if inp is None:
        return []
    return recognize_pattern(bank, inp, algo, kind_mask, max_results,
                             length_filter_threshold, length_filter_ratio)


def recognize_pattern(bank, inp, algo=ALGO_STANDARD, kind_mask=0xFFFFFFFF,
                      max_results=5, length_filter_threshold=None,
                      length_filter_ratio=DEFAULT_LENGTH_FILTER_RATIO):
    """recognize() for a gesture that is already a pattern."""
    nsize = bank["normalize_size"]
    dbl_w = nsize * 2
    if length_filter_threshold is None:
        length_filter_threshold = nsize << FX32_SHIFT

    ensure_features(bank)

    results = []
    for index, entry in enumerate(bank["entries"]):
        if not entry["enabled"]:
            continue
        if not (entry["kind"] & kind_mask):
            continue
        proto = entry["_pattern"]
        if proto is None:
            continue

        if algo == ALGO_LIGHT:
            worst = results[-1]["score"] if len(results) >= max_results else None
            score = match_light(proto, inp, worst)
        elif algo == ALGO_STANDARD:
            score = match_standard(proto, inp, dbl_w)
        elif algo == ALGO_FINE:
            score = match_fine(proto, inp, dbl_w, length_filter_threshold,
                               length_filter_ratio)
        else:
            raise ValueError("pattern_format: unknown algorithm %r" % algo)

        if score <= 0:
            continue

        c = entry["correction"]
        if c:
            score = mulf32(score, FX32_ONE - c) + c
            if score > FX32_ONE:
                score = FX32_ONE

        # Strictly greater, so an earlier entry keeps its place on a tie and
        # the ranking does not depend on iteration order accidents.
        pos = len(results)
        while pos > 0 and score > results[pos - 1]["score"]:
            pos -= 1
        if pos >= max_results:
            continue
        results.insert(pos, {
            "entry": index,
            "code": entry["code"],
            "name": code_name(bank, entry["code"]),
            "score": score,
        })
        del results[max_results:]

    return results


# ---------------------------------------------------------------------------
# The bank
# ---------------------------------------------------------------------------

def new_bank(normalize_size=DEFAULT_NORMALIZE_SIZE):
    return {
        "normalize_size": normalize_size,
        "entries": [],
        "names": [],
    }


def code_for_name(bank, name):
    """Index of name in the bank's name table, appending it if it is new."""
    names = bank["names"]
    try:
        return names.index(name)
    except ValueError:
        names.append(name)
        return len(names) - 1


def code_name(bank, code):
    names = bank["names"]
    if 0 <= code < len(names):
        return names[code]
    return None


def add_entry(bank, strokes, code, kind=1, correction=0, enabled=True,
              normalized=False):
    """Add one prototype. strokes is a list of strokes of (x, y).

    Unless normalized is set, the strokes are taken to be in whatever space
    they were drawn in and are fitted to the bank's normalize_size first.
    """
    nsize = bank["normalize_size"]
    pts = strokes if normalized else normalize(strokes, nsize)
    pts = [s for s in pts if len(s) >= 2]
    if not pts:
        raise PatternError("entry has no stroke with two or more points")

    limit = nsize - 1
    for stroke in pts:
        for x, y in stroke:
            if not (0 <= x <= limit and 0 <= y <= limit):
                raise PatternError(
                    "point (%d, %d) is outside [0, %d]" % (x, y, limit))

    entry = {
        "kind": kind,
        "code": code,
        "correction": correction,
        "enabled": bool(enabled),
        "strokes": pts,
        "_pattern": None,
    }
    bank["entries"].append(entry)
    return len(bank["entries"]) - 1


def remove_entry(bank, index):
    del bank["entries"][index]


def ensure_features(bank):
    """Compute the features of every entry that does not have them yet."""
    for entry in bank["entries"]:
        if entry.get("_pattern") is None:
            entry["_pattern"] = extract(entry["strokes"])


def invalidate_features(bank):
    for entry in bank["entries"]:
        entry["_pattern"] = None


def validate(bank):
    """Returns a list of human readable problems, empty when the bank is good."""
    problems = []
    nsize = bank["normalize_size"]
    if not (1 <= nsize <= 512):
        problems.append("normalize_size %d is outside 1..512" % nsize)
    limit = nsize - 1

    for i, entry in enumerate(bank["entries"]):
        if not entry["strokes"]:
            problems.append("entry %d has no strokes" % i)
            continue
        for j, stroke in enumerate(entry["strokes"]):
            if len(stroke) < 2:
                problems.append(
                    "entry %d stroke %d has fewer than two points" % (i, j))
            for x, y in stroke:
                if not (0 <= x <= limit and 0 <= y <= limit):
                    problems.append(
                        "entry %d stroke %d has a point outside [0, %d]"
                        % (i, j, limit))
                    break
        if extract(entry["strokes"]) is None:
            problems.append("entry %d has no measurable length" % i)
        if entry["code"] < 0 or entry["code"] > 0xFFFF:
            problems.append("entry %d has an out of range code" % i)
        if not (-FX32_ONE < entry["correction"] < FX32_ONE):
            problems.append("entry %d has an out of range correction" % i)

    return problems


# ---------------------------------------------------------------------------
# Reading and writing
# ---------------------------------------------------------------------------

def pack_bank(bank):
    problems = validate(bank)
    if problems:
        raise PatternError("; ".join(problems))

    points = []
    entries = []
    for entry in bank["entries"]:
        index = len(points)
        flat = strokes_to_points(entry["strokes"])
        points.extend(flat)
        entries.append((entry["kind"], entry["code"], entry["correction"],
                        index, len(flat), len(entry["strokes"]),
                        1 if entry["enabled"] else 0))

    point_blob = b"".join(struct.pack("<hh", x, y) for x, y in points)

    entry_blob = b"".join(
        struct.pack("<IHhIHHBxxx", kind, code, corr, idx, pc, sc, en)
        for kind, code, corr, idx, pc, sc, en in entries)

    names = bank["names"]
    if names:
        encoded = [n.encode("utf-8") + b"\0" for n in names]
        table_header = 4 + 4 * len(encoded)
        offsets = []
        pos = table_header
        for blob in encoded:
            offsets.append(pos)
            pos += len(blob)
        name_blob = struct.pack("<HH", len(encoded), 0)
        name_blob += b"".join(struct.pack("<I", o) for o in offsets)
        name_blob += b"".join(encoded)
        # Keep whatever follows word aligned, even though nothing does today.
        name_blob += b"\0" * ((4 - len(name_blob) % 4) % 4)
    else:
        name_blob = b""

    point_offset = HEADER_SIZE
    entry_offset = point_offset + len(point_blob)
    assert entry_offset % 4 == 0
    name_offset = entry_offset + len(entry_blob) if name_blob else 0

    header = struct.pack(
        "<4sHHHHIIIII",
        MAGIC, VERSION, 0, bank["normalize_size"], len(entries),
        len(points), point_offset, entry_offset, name_offset, len(name_blob))
    assert len(header) == HEADER_SIZE

    return header + point_blob + entry_blob + name_blob


def unpack_bank(data):
    if len(data) < HEADER_SIZE:
        raise PatternError("not a .neaptn: too short")
    (magic, version, _flags, nsize, entry_count, point_count,
     point_offset, entry_offset, name_offset) = struct.unpack_from(
        "<4sHHHHIIII", data, 0)
    (name_size,) = struct.unpack_from("<I", data, 28)

    if magic != MAGIC:
        raise PatternError("not a .neaptn: bad magic %r" % magic)
    if version != VERSION:
        raise PatternError("unsupported .neaptn version %d" % version)
    if nsize < 1 or nsize > 512:
        raise PatternError("bad normalize_size %d" % nsize)
    if point_offset + point_count * 4 > len(data):
        raise PatternError("point array runs past the end of the file")
    if entry_offset + entry_count * ENTRY_SIZE > len(data):
        raise PatternError("entry array runs past the end of the file")

    points = [struct.unpack_from("<hh", data, point_offset + 4 * i)
              for i in range(point_count)]

    names = []
    if name_offset:
        if name_offset + name_size > len(data):
            raise PatternError("name table runs past the end of the file")
        count, _pad = struct.unpack_from("<HH", data, name_offset)
        for i in range(count):
            (off,) = struct.unpack_from("<I", data, name_offset + 4 + 4 * i)
            start = name_offset + off
            end = data.index(b"\0", start)
            names.append(data[start:end].decode("utf-8"))

    bank = new_bank(nsize)
    bank["names"] = names
    limit = nsize - 1

    for i in range(entry_count):
        (kind, code, corr, index, pc, sc, enabled) = struct.unpack_from(
            "<IHhIHHBxxx", data, entry_offset + i * ENTRY_SIZE)
        if index + pc > point_count:
            raise PatternError("entry %d points past the end of the array" % i)
        strokes = points_to_strokes(points[index:index + pc])
        if len(strokes) != sc:
            raise PatternError(
                "entry %d says %d strokes but holds %d" % (i, sc, len(strokes)))
        for stroke in strokes:
            for x, y in stroke:
                if not (0 <= x <= limit and 0 <= y <= limit):
                    raise PatternError(
                        "entry %d has a point outside [0, %d]" % (i, limit))
        bank["entries"].append({
            "kind": kind,
            "code": code,
            "correction": corr,
            "enabled": bool(enabled),
            "strokes": strokes,
            "_pattern": None,
        })

    return bank


def write_bank(bank, path):
    with open(path, "wb") as f:
        f.write(pack_bank(bank))


def read_bank(path):
    with open(path, "rb") as f:
        return unpack_bank(f.read())


# ---------------------------------------------------------------------------
# The generated header
# ---------------------------------------------------------------------------

ATAN_HEADER_TEMPLATE = """\
// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Warioware64
//
// This file is part of Nitro Engine Advanced
//
// Generated by tools/pattern_editor/pattern_format.py -- do not edit by hand.
//
// atan(i / %(scale)d) in 16 bit angle units, where 65536 is a full turn. The
// table is generated rather than taken from libnds because the editor has to
// produce the same angles the ARM9 does, and sinLerp()'s table lives inside
// libnds.a where nothing on the host can mirror it without guessing.

#ifndef NEA_PATTERN_ATAN_H__
#define NEA_PATTERN_ATAN_H__

#include <nds.h>

#define NEA_PATTERN_ATAN_SHIFT %(shift)d
#define NEA_PATTERN_ATAN_SIZE  %(size)d

static const uint16_t nea_pattern_atan_table[NEA_PATTERN_ATAN_SIZE] = {
%(rows)s};

#endif // NEA_PATTERN_ATAN_H__
"""


def write_atan_header(path):
    rows = []
    for i in range(0, ATAN_TAB_SIZE, 8):
        chunk = ATAN_TAB[i:i + 8]
        rows.append("    " + " ".join("%5d," % v for v in chunk))
    text = ATAN_HEADER_TEMPLATE % {
        "scale": 1 << ATAN_TAB_SHIFT,
        "shift": ATAN_TAB_SHIFT,
        "size": ATAN_TAB_SIZE,
        "rows": "\n".join(rows) + "\n",
    }
    with open(path, "w") as f:
        f.write(text)


def main():
    import argparse

    ap = argparse.ArgumentParser(
        description="Inspect a .neaptn bank, or regenerate the ARM9 tables")
    ap.add_argument("file", nargs="?", help="a .neaptn to describe")
    ap.add_argument("--write-atan-header", metavar="PATH",
                    help="regenerate source/NEAPatternAtan.h")
    args = ap.parse_args()

    if args.write_atan_header:
        write_atan_header(args.write_atan_header)
        print("wrote %s" % args.write_atan_header)

    if args.file:
        bank = read_bank(args.file)
        print("normalize_size %d, %d entries, %d names"
              % (bank["normalize_size"], len(bank["entries"]),
                 len(bank["names"])))
        for i, e in enumerate(bank["entries"]):
            print("  %3d  code %3d %-8s kind 0x%x  %d strokes, %d points%s"
                  % (i, e["code"], code_name(bank, e["code"]) or "", e["kind"],
                     len(e["strokes"]), sum(len(s) for s in e["strokes"]),
                     "" if e["enabled"] else "  (disabled)"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
