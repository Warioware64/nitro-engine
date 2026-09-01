#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
#
# Copyright (c) 2026 Warioware64
#
# Builds a .neaptn pattern bank from a dictionary written as text.
#
# The syntax is the one retail DS dictionaries were authored in, so an
# existing dictionary converts without being redrawn. It is also what
# NEA_PatternBankSaveFAT()'s companion dump prints and what the editor exports
# through File -> Export text, which makes it the format a gesture trained on
# hardware travels back through.
#
# Usage:
#     python3 pattern_import.py --input patterns.txt --output patterns.neaptn
#     python3 pattern_import.py --input patterns.txt --output out.bin \
#         --normalize-size 64
#     python3 pattern_import.py --export bank.neaptn --output patterns.txt

"""pattern_import.py -- text dictionary <-> .neaptn.

THE TEXT SYNTAX

One prototype per line, blank lines and `#` comments ignored:

    "NAME" KIND CORRECTION SIZE | x,y x,y ... | x,y x,y ... |

  NAME        the meaning, in quotes. Several lines may share a name, and
              normally do: that is how one character gets a prototype per
              stroke order, or per the way different hands draw it. Lines
              sharing a name share a code.
  KIND        a bitmask, matched against the mask passed to recognition.
  CORRECTION  4096 = 1.0, biasing this prototype's score toward a perfect
              match. 0 for no bias, which is almost always what you want.
  SIZE        the coordinate space the points on *this line* are written in.
              Points are rescaled to the output bank's space, so a dictionary
              may mix them.
  strokes     separated by `|`, each a run of `x,y` points. Parentheses around
              a point are accepted and ignored, because retail dictionaries
              wrote them that way.

A line with no stroke of two or more points is an error, not a silent skip:
the usual cause is a stray `|`, and dropping it quietly would leave a
dictionary one prototype short of what its author thought they had.
"""

import argparse
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import pattern_format as P  # noqa: E402

LINE_RE = re.compile(
    r'^"(?P<name>(?:[^"\\]|\\.)*)"\s+'
    r"(?P<kind>-?\d+)\s+"
    r"(?P<correction>-?\d+)\s+"
    r"(?P<size>\d+)\s*"
    r"(?P<points>\|.*)$")

POINT_RE = re.compile(r"\(?\s*(-?\d+)\s*,\s*(-?\d+)\s*\)?")


class ImportError_(P.PatternError):
    pass


def parse_line(line, lineno):
    """Returns (name, kind, correction, size, strokes) or None for a no-op."""
    # A `#` inside the quoted name is not a comment, so cut at the first one
    # that is outside quotes.
    out = []
    in_quotes = False
    for ch in line:
        if ch == '"':
            in_quotes = not in_quotes
        if ch == "#" and not in_quotes:
            break
        out.append(ch)
    text = "".join(out).strip()

    if not text:
        return None

    m = LINE_RE.match(text)
    if m is None:
        raise ImportError_("line %d: cannot parse %r" % (lineno, text))

    size = int(m.group("size"))
    if size < 1:
        raise ImportError_("line %d: size must be positive" % lineno)

    strokes = []
    for part in m.group("points").split("|"):
        pts = [(int(a), int(b)) for a, b in POINT_RE.findall(part)]
        if pts:
            strokes.append(pts)

    if not any(len(s) >= 2 for s in strokes):
        raise ImportError_(
            "line %d: no stroke has two or more points" % lineno)

    return (m.group("name").replace('\\"', '"'), int(m.group("kind")),
            int(m.group("correction")), size, strokes)


def rescale(strokes, src_size, dst_size):
    """Fits points authored in one space into another, keeping the shape."""
    if src_size == dst_size:
        return [[(x, y) for x, y in s] for s in strokes]

    limit = dst_size - 1
    out = []
    for stroke in strokes:
        acc = []
        for x, y in stroke:
            nx = x * dst_size // src_size
            ny = y * dst_size // src_size
            if nx < 0: nx = 0
            elif nx > limit: nx = limit
            if ny < 0: ny = 0
            elif ny > limit: ny = limit
            if acc and acc[-1] == (nx, ny):
                continue
            acc.append((nx, ny))
        out.append(acc)
    return out


def load_text(path, normalize_size):
    bank = P.new_bank(normalize_size)

    with open(path, "r", encoding="utf-8") as f:
        for lineno, line in enumerate(f, 1):
            parsed = parse_line(line, lineno)
            if parsed is None:
                continue
            name, kind, correction, size, strokes = parsed

            scaled = rescale(strokes, size, normalize_size)
            scaled = [s for s in scaled if len(s) >= 2]
            if not scaled:
                raise ImportError_(
                    "line %d: every stroke collapsed when rescaled from %d to "
                    "%d -- the prototype is too small for this bank"
                    % (lineno, size, normalize_size))

            code = P.code_for_name(bank, name)
            # normalized=False, so the prototype is fitted to the bank's box
            # the same way a recognised gesture will be. A dictionary authored
            # with a margin and one authored edge to edge then agree.
            P.add_entry(bank, scaled, code, kind, correction)

    if not bank["entries"]:
        raise ImportError_("%s holds no prototypes" % path)

    return bank


def dump_text(bank, out):
    size = bank["normalize_size"]
    for entry in bank["entries"]:
        name = (P.code_name(bank, entry["code"]) or
                "code%d" % entry["code"]).replace('"', '\\"')
        parts = []
        for stroke in entry["strokes"]:
            parts.append(" ".join("%d,%d" % (x, y) for x, y in stroke))
        line = '"%s" %d %d %d | %s |' % (
            name, entry["kind"], entry["correction"], size, " | ".join(parts))
        if not entry["enabled"]:
            line += "  # disabled"
        out.write(line + "\n")


def main():
    ap = argparse.ArgumentParser(
        description=__doc__.splitlines()[0],
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    ap.add_argument("--input", help="dictionary text to read")
    ap.add_argument("--export", metavar="BANK",
                    help="a .neaptn to write back out as text instead")
    ap.add_argument("--output", required=True, help="file to write")
    ap.add_argument("--normalize-size", type=int,
                    default=P.DEFAULT_NORMALIZE_SIZE,
                    help="coordinate space of the bank (default %d)"
                         % P.DEFAULT_NORMALIZE_SIZE)
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()

    if bool(args.input) == bool(args.export):
        ap.error("give exactly one of --input and --export")

    try:
        if args.export:
            bank = P.read_bank(args.export)
            with open(args.output, "w", encoding="utf-8") as f:
                dump_text(bank, f)
            if args.verbose:
                print("wrote %d prototypes to %s"
                      % (len(bank["entries"]), args.output))
            return 0

        bank = load_text(args.input, args.normalize_size)
        P.write_bank(bank, args.output)

        if args.verbose:
            counts = {}
            for e in bank["entries"]:
                name = P.code_name(bank, e["code"])
                counts[name] = counts.get(name, 0) + 1
            print("%s: %d prototypes over %d names, space %d"
                  % (args.output, len(bank["entries"]), len(bank["names"]),
                     bank["normalize_size"]))
            for name in bank["names"]:
                n = counts.get(name, 0)
                print("  %-8s %d prototype%s" % (name, n, "" if n == 1 else "s"))
        return 0

    except (P.PatternError, OSError) as e:
        print("pattern_import: %s" % e, file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
