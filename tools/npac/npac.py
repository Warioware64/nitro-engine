#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
#
# Copyright (c) 2026 Warioware64
#
# Command line front end for the NPAC archive container. The format itself,
# and the reason each field is there, lives in npac_format.py.

"""npac.py -- create, extract and inspect NPAC archives.

    python3 npac.py create  --input assets/ --output levels.npac
    python3 npac.py extract --input levels.npac --output out/
    python3 npac.py list    --input levels.npac

Compression is per file and is done by the Wonderful Toolchain encoders
(wf-nnpack-lzss, wf-nnpack-huffman, wf-nnpack-rle), which write the same BIOS
headers the DS decompresses. The default, --compress auto, tries all three per
file and keeps whichever is smallest, storing the file uncompressed when none
of them wins -- which for small files is the usual outcome.

Huffman is available but excluded from "auto" unless --allow-huffman is given:
melonDS decodes it wrongly under its default HLE BIOS, and it seldom beats LZ77
on DS assets, so picking it automatically would be trading a rare few percent
for a silent corruption nobody asked for.

On the DS the archive is mounted with NEA_NpacMount("levels",
"nitro:/levels.npac") and read through the ordinary file functions;
"levels:/robot.bin" then works anywhere a path does, including in every
NEA_*LoadFAT() call. Compressed members are inflated transparently on open().
"""

import argparse
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import npac_format as N  # noqa: E402


def _human(n):
    return "%d" % n


def cmd_create(args):
    if not os.path.isdir(args.input):
        print("npac: %s is not a directory" % args.input, file=sys.stderr)
        return 1

    if args.compress != "none" and N.tool_path("wf-nnpack-lzss", args.tool_dir) is None:
        print("npac: warning: no Wonderful Toolchain encoders found in %s, "
              "everything will be stored uncompressed"
              % (args.tool_dir or N.DEFAULT_TOOL_DIR), file=sys.stderr)

    total_raw = [0]
    total_stored = [0]

    def progress(name, method, raw, stored):
        total_raw[0] += raw
        total_stored[0] += stored
        if args.verbose:
            saved = "" if method == N.METHOD_STORED else \
                " (-%d%%)" % (100 - (stored * 100 // raw) if raw else 0)
            print("  %-40s %-6s %7d -> %7d%s"
                  % (name, N.METHOD_NAMES[method], raw, stored, saved))

    try:
        entries = N.scan_directory(args.input)
        data = N.build(entries, args.compress, args.tool_dir, progress,
                       args.allow_huffman)
    except N.NpacError as e:
        print("npac: %s" % e, file=sys.stderr)
        return 1

    with open(args.output, "wb") as f:
        f.write(data)

    print("npac: %s -- %d files, %d bytes of data in a %d byte archive"
          % (args.output, len(entries), total_raw[0], len(data)))
    return 0


def cmd_extract(args):
    try:
        members = N.read_file(args.input)
    except N.NpacError as e:
        print("npac: %s" % e, file=sys.stderr)
        return 1

    for member in members:
        out = os.path.join(args.output, *member.path.split("/"))
        os.makedirs(os.path.dirname(out) or ".", exist_ok=True)
        try:
            payload = member.data(args.tool_dir)
        except N.NpacError as e:
            print("npac: %s: %s" % (member.path, e), file=sys.stderr)
            return 1
        with open(out, "wb") as f:
            f.write(payload)
        if args.verbose:
            print("  %s" % member.path)

    print("npac: extracted %d files to %s" % (len(members), args.output))
    return 0


def cmd_list(args):
    try:
        members = N.read_file(args.input)
    except N.NpacError as e:
        print("npac: %s" % e, file=sys.stderr)
        return 1

    print("%-5s %-6s %9s %9s  %s" % ("id", "method", "size", "stored", "path"))
    raw = stored = 0
    for member in members:
        raw += member.raw_size
        stored += len(member.stored)
        print("%-5d %-6s %9d %9d  %s"
              % (member.file_id, member.method_name, member.raw_size,
                 len(member.stored), member.path))
    print("%-5s %-6s %9d %9d  %d files"
          % ("", "", raw, stored, len(members)))
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(
        prog="npac.py",
        description="Create, extract and inspect NPAC archive containers.")
    parser.add_argument("--tool-dir", metavar="DIR",
                        help="where the wf-nnpack-* encoders live "
                             "(default: %s)" % N.DEFAULT_TOOL_DIR)
    parser.add_argument("-v", "--verbose", action="store_true",
                        help="list every file as it is processed")
    sub = parser.add_subparsers(dest="command")

    p = sub.add_parser("create", help="pack a directory tree into an archive")
    p.add_argument("--input", required=True, metavar="DIR")
    p.add_argument("--output", required=True, metavar="FILE")
    p.add_argument("--compress", default="auto", choices=N.COMPRESS_MODES,
                   help="compression method (default: auto)")
    p.add_argument("--allow-huffman", action="store_true",
                   help="let --compress auto pick Huffman as well. It is left "
                        "out by default because melonDS's HLE BIOS decodes it "
                        "to the wrong bytes without saying so; hardware and a "
                        "real BIOS dump are both fine")
    p.set_defaults(func=cmd_create)

    p = sub.add_parser("extract", help="unpack an archive into a directory")
    p.add_argument("--input", required=True, metavar="FILE")
    p.add_argument("--output", required=True, metavar="DIR")
    p.set_defaults(func=cmd_extract)

    p = sub.add_parser("list", help="print the contents of an archive")
    p.add_argument("--input", required=True, metavar="FILE")
    p.set_defaults(func=cmd_list)

    args = parser.parse_args(argv)
    if args.command is None:
        parser.print_help()
        return 1
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
