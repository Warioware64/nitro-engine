#!/usr/bin/env python3
# SPDX-License-Identifier: CC0-1.0
#
# SPDX-FileContributor: Warioware64, 2026
#
# Generates the cross-check vectors for the NEAPattern recognizer.
#
# The editor's live scores are only honest if pattern_format.py evaluates a
# gesture exactly the way NEAPattern.c does. Nothing enforces that on its own,
# and a drift between the two would show up as an editor that quietly
# disagrees with the hardware about which shape wins -- the worst kind of bug
# for an authoring tool, because the author trusts the preview.
#
# So this writes two files from one source of truth: the .neaptn binary, and a
# C table of what the Python evaluator says every algorithm, every resampling
# method and every threshold resolve every input gesture to. The test ROM
# loads the binary, recognises the same gestures with the real runtime, and
# compares. If the two implementations ever diverge, the ROM says which
# gesture, which algorithm and which result slot.

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "tools", "pattern_editor"))

import pattern_format as P  # noqa: E402

NORMALIZE_SIZE = 64

KIND_LETTER = 1
KIND_SYMBOL = 2


def build_bank():
    """A bank chosen so that nothing can drift quietly.

    Single and multi stroke entries; a T and a plus, which Light must fail to
    separate and Standard must separate; the same T drawn in the other stroke
    order, sharing its code, which is how retail handled stroke order variants;
    an entry with a non-zero correction, so the bias is exercised; a disabled
    entry, which must never appear in a result; and two kind groups, so the
    mask is exercised.
    """
    bank = P.new_bank(NORMALIZE_SIZE)

    def add(name, strokes, kind=KIND_LETTER, correction=0, enabled=True):
        code = P.code_for_name(bank, name)
        return P.add_entry(bank, strokes, code, kind, correction, enabled,
                           normalized=True)

    # Single stroke, one segment: the shortest thing that can be scored.
    add("I", [[(32, 2), (32, 61)]])
    # Single stroke, two segments with a right angle.
    add("L", [[(14, 2), (14, 61), (52, 61)]])
    # Single stroke, curved, so the resamplers have something to disagree over.
    add("C", [[(52, 6), (30, 2), (12, 20), (10, 38), (26, 60), (54, 58)]])
    # Two strokes, drawn in two orders under one code.
    add("T", [[(2, 6), (61, 6)], [(32, 6), (32, 61)]], KIND_SYMBOL)
    add("T", [[(32, 6), (32, 61)], [(2, 6), (61, 6)]], KIND_SYMBOL)
    add("+", [[(2, 32), (61, 32)], [(32, 2), (32, 61)]], KIND_SYMBOL)
    # A correction of 0.25 pulls this one's score a quarter of the way to 1.
    add("S", [[(52, 8), (20, 4), (12, 22), (44, 38), (50, 52), (16, 58)]],
        KIND_LETTER, correction=1024)
    # Never allowed to win anything.
    add("X", [[(4, 4), (60, 60)], [(60, 4), (4, 60)]], KIND_SYMBOL,
        enabled=False)
    # Three strokes, so a three stroke input has something to reach.
    add("H", [[(10, 4), (10, 60)], [(52, 4), (52, 60)], [(10, 32), (52, 32)]])

    return bank


# Gestures in screen space, the way the stylus would deliver them. They are
# deliberately not multiples of the bank's space, so the normalisation is under
# test rather than being handed round numbers.
INPUTS = [
    ("clean T", [[(60, 40), (200, 40)], [(130, 40), (130, 180)]]),
    ("T reversed order", [[(130, 40), (130, 180)], [(60, 40), (200, 40)]]),
    ("clean plus", [[(40, 110), (210, 110)], [(125, 25), (125, 195)]]),
    ("tiny plus", [[(96, 104), (118, 104)], [(107, 93), (107, 115)]]),
    ("sloppy C", [[(180, 30), (120, 28), (70, 45), (45, 90), (60, 140),
                   (120, 168), (185, 170)]]),
    ("hesitant C", [[(180, 30), (170, 29), (160, 29), (150, 30), (120, 28),
                     (70, 45), (45, 90), (60, 140), (120, 168), (185, 170)]]),
    ("I upward", [[(128, 180), (128, 30)]]),
    ("L", [[(70, 20), (70, 170), (190, 170)]]),
    ("S", [[(190, 34), (90, 20), (58, 92), (170, 150), (188, 200), (70, 220)]]),
    ("H", [[(50, 20), (50, 170)], [(180, 20), (180, 170)],
           [(50, 95), (180, 95)]]),
    ("zigzag noise", [[(40, 100), (60, 102), (80, 99), (100, 101), (120, 100),
                       (140, 101), (160, 99), (180, 100)]]),
    ("single point", [[(100, 100)]]),
    ("two coincident points", [[(100, 100), (100, 100)]]),
    ("four strokes", [[(10, 10), (60, 10)], [(10, 30), (60, 30)],
                      [(10, 50), (60, 50)], [(10, 70), (60, 70)]]),
]

ALGOS = [P.ALGO_LIGHT, P.ALGO_STANDARD, P.ALGO_FINE]

# One threshold per method that is meaningful for it. The angle method counts
# in 16 bit angle units, the other two in coordinate units.
METHODS = [
    (P.RESAMPLE_NONE, 0),
    (P.RESAMPLE_DISTANCE, 4),
    (P.RESAMPLE_DISTANCE, 12),
    (P.RESAMPLE_ANGLE, 4096),
    (P.RESAMPLE_ANGLE, 12288),
    (P.RESAMPLE_RECURSIVE, 1),
    (P.RESAMPLE_RECURSIVE, 2),
    (P.RESAMPLE_RECURSIVE, 6),
]

# Both the whole bank and one kind on its own, so the mask is under test.
KIND_MASKS = [0xFFFFFFFF, KIND_SYMBOL]

MAX_RESULTS = 5
MAX_POINTS = 40
MAX_STROKES = 8


def c_points(strokes):
    """Marker terminated flat point list, as the C side stores a gesture."""
    out = []
    for stroke in strokes:
        out.extend(stroke)
        out.append((P.PEN_UP_X, P.PEN_UP_Y))
    return out


def fmt_points(points):
    parts = []
    for i in range(0, len(points), 6):
        chunk = points[i:i + 6]
        parts.append("    " + " ".join("%d, %d," % (x, y) for x, y in chunk))
    return "\n".join(parts)


def main():
    bank = build_bank()

    problems = P.validate(bank)
    if problems:
        for p in problems:
            print("bank: %s" % p, file=sys.stderr)
        return 1

    blob = P.pack_bank(bank)
    bank_path = os.path.join(HERE, "data", "bank.bin")
    with open(bank_path, "wb") as f:
        f.write(blob)

    # A round trip through the reader has to be byte identical, or the C side
    # is being handed something the Python side would not accept back.
    if P.pack_bank(P.unpack_bank(blob)) != blob:
        print("bank does not survive a round trip", file=sys.stderr)
        return 1

    lines = []
    w = lines.append

    w("// SPDX-License-Identifier: CC0-1.0")
    w("//")
    w("// SPDX-FileContributor: Warioware64, 2026")
    w("//")
    w("// Generated by tests/pattern_eval/gen_vectors.py -- do not edit.")
    w("//")
    w("// What tools/pattern_editor/pattern_format.py says every case below")
    w("// resolves to. NEAPattern.c has to agree exactly.")
    w("")
    w("#ifndef PATTERN_VECTORS_H__")
    w("#define PATTERN_VECTORS_H__")
    w("")
    w("#include <stdint.h>")
    w("")
    w("#define PTN_NORMALIZE_SIZE %d" % NORMALIZE_SIZE)
    w("#define PTN_MAX_RESULTS    %d" % MAX_RESULTS)
    w("#define PTN_MAX_POINTS     %d" % MAX_POINTS)
    w("#define PTN_MAX_STROKES    %d" % MAX_STROKES)
    w("")

    # The inputs.
    for i, (name, strokes) in enumerate(INPUTS):
        pts = c_points(strokes)
        w("static const int16_t ptn_input_%d[] = {" % i)
        w(fmt_points(pts))
        w("};")
    w("")
    w("typedef struct {")
    w("    const char *name;")
    w("    const int16_t *points;")
    w("    int count;")
    w("} ptn_input;")
    w("")
    w("static const ptn_input ptn_inputs[] = {")
    for i, (name, strokes) in enumerate(INPUTS):
        pts = c_points(strokes)
        w('    { "%s", ptn_input_%d, %d },' % (name, i, len(pts)))
    w("};")
    w("")
    w("#define PTN_NUM_INPUTS %d" % len(INPUTS))
    w("")

    # The cases.
    cases = []
    reduced_blobs = {}

    for ii, (name, strokes) in enumerate(INPUTS):
        for method, threshold in METHODS:
            inp = P.make_input(strokes, NORMALIZE_SIZE, method, threshold,
                               MAX_POINTS)
            # The reduced gesture depends on the input and the resampling
            # only, so it is shared across the algorithms and masks.
            key = (ii, method, threshold)
            if inp is None:
                reduced_blobs[key] = []
            else:
                reduced = [s["pts"] for s in inp["strokes"]]
                reduced_blobs[key] = c_points(reduced)

            for algo in ALGOS:
                for mask in KIND_MASKS:
                    if inp is None:
                        results = []
                    else:
                        results = P.recognize_pattern(
                            bank, inp, algo, mask, MAX_RESULTS)
                    cases.append({
                        "input": ii,
                        "algo": algo,
                        "method": method,
                        "threshold": threshold,
                        "mask": mask,
                        "results": results,
                        "key": key,
                    })

    keys = sorted(reduced_blobs)
    key_index = {k: i for i, k in enumerate(keys)}
    for i, k in enumerate(keys):
        blob_pts = reduced_blobs[k]
        if blob_pts:
            w("static const int16_t ptn_reduced_%d[] = {" % i)
            w(fmt_points(blob_pts))
            w("};")
    w("")
    w("typedef struct {")
    w("    const int16_t *points;")
    w("    int count;")
    w("} ptn_reduced;")
    w("")
    w("static const ptn_reduced ptn_reduceds[] = {")
    for i, k in enumerate(keys):
        blob_pts = reduced_blobs[k]
        if blob_pts:
            w("    { ptn_reduced_%d, %d }," % (i, len(blob_pts)))
        else:
            w("    { NULL, 0 },")
    w("};")
    w("")

    for i, case in enumerate(cases):
        if not case["results"]:
            continue
        w("static const int32_t ptn_case_%d[] = {" % i)
        w("    " + " ".join("%d, %d, %d," % (r["entry"], r["code"], r["score"])
                            for r in case["results"]))
        w("};")
    w("")
    w("typedef struct {")
    w("    int input;")
    w("    int algo;")
    w("    int method;")
    w("    int threshold;")
    w("    uint32_t mask;")
    w("    int reduced;")
    w("    int nresults;")
    w("    const int32_t *results; // entry, code, score per result")
    w("} ptn_case;")
    w("")
    w("static const ptn_case ptn_cases[] = {")
    for i, case in enumerate(cases):
        ref = ("ptn_case_%d" % i) if case["results"] else "NULL"
        w("    { %d, %d, %d, %d, 0x%08XU, %d, %d, %s },"
          % (case["input"], case["algo"], case["method"], case["threshold"],
             case["mask"], key_index[case["key"]], len(case["results"]), ref))
    w("};")
    w("")
    w("#define PTN_NUM_CASES %d" % len(cases))
    w("")
    w("#endif // PATTERN_VECTORS_H__")

    out_path = os.path.join(HERE, "source", "vectors.h")
    with open(out_path, "w") as f:
        f.write("\n".join(lines) + "\n")

    nonempty = sum(1 for c in cases if c["results"])
    print("bank:    %s (%d bytes, %d entries)"
          % (bank_path, len(blob), len(bank["entries"])))
    print("vectors: %s (%d cases, %d with results)"
          % (out_path, len(cases), nonempty))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
