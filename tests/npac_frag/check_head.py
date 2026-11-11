#!/usr/bin/env python3
# SPDX-License-Identifier: CC0-1.0
#
# SPDX-FileContributor: Warioware64, 2026
#
# This file is part of Nitro Engine Advanced

"""Holds source/npac_write.h to tools/npac/npac_format.py, byte for byte.

usage: python3 check_head.py     (needs a C compiler as `cc`)

The test ROM writes its archives' heads itself, a cluster at a time; this
compiles that code on the PC and compares the head and the file length it
gives with what npac_format.build() makes of the same members.
"""
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "tools", "npac"))
import npac_format  # noqa: E402

PROGRAM = r"""
#include <stdio.h>
#include <stdlib.h>
#include "npac_write.h"
int main(int argc, char **argv)
{
    uint8_t head[NPAC_FRAG_HEAD_MAX];
    uint32_t total = 0;
    uint32_t n = npac_frag_head(head, (uint32_t)atoi(argv[1]), (uint32_t)atoi(argv[2]), &total);
    printf("%u\n", total);
    for (uint32_t i = 0; i < n; i++)
        printf("%02x", head[i]);
    printf("\n");
    return n == 0;
}
"""


def main():
    with tempfile.TemporaryDirectory() as tmp:
        src = os.path.join(tmp, "head.c")
        exe = os.path.join(tmp, "head")
        open(src, "w").write(PROGRAM)
        subprocess.run(["cc", "-std=c99", "-I", os.path.join(HERE, "source"), "-o", exe, src], check=True)
        failed = 0
        for count, size in [(1, 4), (1, 4096), (3, 65536), (7, 8 << 20), (10, 1 << 20)]:
            out = subprocess.run([exe, str(count), str(size)], capture_output=True, text=True, check=True)
            total, head = out.stdout.split()
            members = [("d%d.bin" % i, bytes(size)) for i in range(count)]
            want = npac_format.build(members, compress_mode="none")
            ok = int(total) == len(want) and bytes.fromhex(head) == want[:len(head) // 2] and \
                len(want) - len(head) // 2 == count * size
            print("%s: %d members of %d bytes" % ("ok" if ok else "DIFFERENT", count, size))
            failed += 0 if ok else 1
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
