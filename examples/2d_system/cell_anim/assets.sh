#!/bin/sh

# The spritesheet is generated rather than shipped, and cell_pack turns it into
# the .neacell plus the artwork both backends need.
python3 graphics/make_sheet.py || exit 1
mv hopper.png graphics/hopper.png 2>/dev/null

python3 ../../../tools/cell_editor/cell_pack.py graphics/hopper.png \
    --grid 4x2 --out data/hopper --name hopper --fps 12 \
    --anchor-bottom --bank-ext _cells.bin \
    || exit 1
