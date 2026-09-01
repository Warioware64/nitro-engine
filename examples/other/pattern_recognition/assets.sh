#!/bin/sh

# The dictionary is generated as text and then packed, rather than shipped as
# a binary, so what the example recognises can be read and edited.

NITRO_ENGINE=../../..
TOOLS=$NITRO_ENGINE/tools

mkdir -p data

python3 graphics/make_patterns.py || exit 1

python3 $TOOLS/pattern_editor/pattern_import.py -v \
    --input graphics/patterns.txt \
    --output data/patterns.bin \
    --normalize-size 64 \
    || exit 1
