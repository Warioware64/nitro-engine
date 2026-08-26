#!/usr/bin/env python3
# SPDX-License-Identifier: CC0-1.0
#
# SPDX-FileContributor: Warioware64, 2026
#
# Builds the inputs for the NPE loader test: one good file and a set of
# truncated ones.
#
# The NPE parser walks the file using counts stored inside it, so a truncated
# image used to read past the end of the buffer. The cut points below are chosen
# to land in each stage of the parse -- inside the header, inside the emitter
# block, inside the material reference, and part-way through each key array.

import os
import shutil

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, "..", "..", "examples", "effects", "particles",
                   "nitrofiles", "fire.npe")

# 16 header + 80 emitter + 52 material + 2 = 150 is the smallest valid prefix
# before the key arrays.
# bin2c refuses an empty file, so the zero-length case is covered in the test
# by passing a size of 0 against the shortest real prefix instead.
CUTS = [4, 15, 60, 149, 151, 160, 200]


def main():
    data = open(SRC, "rb").read()
    os.makedirs(os.path.join(HERE, "data"), exist_ok=True)

    # The build converts *.bin, so that is what these are called.
    shutil.copyfile(SRC, os.path.join(HERE, "data", "good.bin"))

    for n in CUTS:
        with open(os.path.join(HERE, "data", f"cut{n:03d}.bin"), "wb") as f:
            f.write(data[:n])

    print(f"good.bin: {len(data)} bytes")
    print("truncated:", ", ".join(f"cut{n:03d}" for n in CUTS))


if __name__ == "__main__":
    main()
