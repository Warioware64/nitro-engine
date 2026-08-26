#!/bin/sh

BLOCKSDSEXT="${BLOCKSDSEXT:-/opt/wonderful/thirdparty/blocksds/external}"
PTEXCONV=$BLOCKSDSEXT/ptexconv/ptexconv

rm -rf nitrofiles
mkdir -p nitrofiles

# "-og" writes a GRF instead of the loose _tex/_idx/_pal binaries, and it
# composes with "-tt": the trimmed rows go into the GFX chunk and the *real*
# height goes into the GRF header. NEA_MaterialTexLoadGRF() takes the size from
# that header, so a trimmed GRF needs nothing special on the loading side.

# Paletted, trimmed: 256x160 in the header, 40960 bytes of GFX.
$PTEXCONV \
    -gt -og -f palette256 -v \
    -o nitrofiles/brick_trim \
    assets/brick_256x160.png \
    -tt

# The same image padded up to 256x256: 65536 bytes of GFX, for comparison.
$PTEXCONV \
    -gt -og -f palette256 -v \
    -o nitrofiles/brick_pad \
    assets/brick_256x160.png

# Trimmed *and* compressed. GRF stores every chunk behind a BIOS compression
# header, so "-clz" shrinks what sits in the ROM while the VRAM footprint stays
# the trimmed 40960 bytes. The two savings are independent and stack.
$PTEXCONV \
    -gt -og -f palette256 -clz -v \
    -o nitrofiles/brick_trim_lz \
    assets/brick_256x160.png \
    -tt

# tex4x4, trimmed and compressed. A single GRF carries the texels (GFX), the
# palette indices (PIDX) and the palette (PAL).
$PTEXCONV \
    -gt -og -f tex4x4 -clz -v \
    -o nitrofiles/brick_t4_trim_lz \
    assets/brick_256x160.png \
    -tt
