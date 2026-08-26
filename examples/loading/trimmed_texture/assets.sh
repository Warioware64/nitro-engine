#!/bin/sh

NITRO_ENGINE=../../..
TOOLS=$NITRO_ENGINE/tools
OBJ2DL=$TOOLS/obj2dl/obj2dl.py

BLOCKSDSEXT="${BLOCKSDSEXT:-/opt/wonderful/thirdparty/blocksds/external}"
PTEXCONV=$BLOCKSDSEXT/ptexconv/ptexconv

rm -rf data
mkdir -p data

# banner.png is 256x160. The height isn't a power of two, so it has to be
# either trimmed or padded. Convert it both ways from the same source image to
# compare what each costs in VRAM.

# "-tt" trims the T axis: only the 160 real rows are written out (40960 bytes).
$PTEXCONV \
    -gt -ob -f palette256 -v \
    -o data/banner_trim \
    assets/banner.png \
    -tt

# Without "-tt" the image is padded up to 256x256 (65536 bytes). The extra rows
# are dead weight in VRAM: the model never samples them.
$PTEXCONV \
    -gt -ob -f palette256 -v \
    -o data/banner_pad \
    assets/banner.png

# One display list serves both textures. ptexconv stores the image at rows
# 0..159 whether it trims or pads, so the texture coordinates are scaled by the
# real height in both cases.
python3 $OBJ2DL \
    --input assets/banner.obj \
    --output data/banner.bin \
    --texture 256 160
