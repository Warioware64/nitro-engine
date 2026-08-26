#!/bin/sh

BLOCKSDSEXT="${BLOCKSDSEXT:-/opt/wonderful/thirdparty/blocksds/external}"
PTEXCONV=$BLOCKSDSEXT/ptexconv/ptexconv

rm -rf data
mkdir -p data

# landscape_256x160.png is the landscape from examples/loading/compressed_texture
# resized to 160 rows. Convert it both ways to compare what each costs in VRAM.

# "-tt" trims the T axis. For tex4x4 the texels are stored as rows of 4x4
# blocks, so trimming happens at block granularity: 160 rows is 40 block rows.
#
#   texel data (slot 0/2)  256 * 160 / 4 = 10240 bytes
#   palette indices (slot 1)        / 2 =  5120 bytes
$PTEXCONV \
    -gt -ob -k FF00FF -f tex4x4 -v \
    -o data/landscape_trim \
    assets/landscape_256x160.png \
    -tt

# Without "-tt" the image is padded up to 256x256: 16384 + 8192 bytes, of which
# the last 96 rows are never sampled.
$PTEXCONV \
    -gt -ob -k FF00FF -f tex4x4 -v \
    -o data/landscape_pad \
    assets/landscape_256x160.png
