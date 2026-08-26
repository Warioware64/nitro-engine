#!/bin/sh

NITRO_ENGINE=../../..
ASSETS=$NITRO_ENGINE/examples/assets
TOOLS=$NITRO_ENGINE/tools
OBJ2DL=$TOOLS/obj2dl/obj2dl.py

BLOCKSDSEXT="${BLOCKSDSEXT:-/opt/wonderful/thirdparty/blocksds/external}"
PTEXCONV=$BLOCKSDSEXT/ptexconv/ptexconv

rm -rf data
mkdir -p data

# brick_256x160.png is examples/assets/brick_wall_small/brick_wall.jpg resized
# to 160 rows, which is all this model needs. Convert it both ways from that one
# source image to compare what each costs in VRAM. The brick courses run
# horizontally, so a mistake on the T axis would be impossible to miss.

# "-tt" trims the T axis: only the 160 real rows are written out (40960 bytes).
$PTEXCONV \
    -gt -ob -f palette256 -v \
    -o data/brick_trim \
    assets/brick_256x160.png \
    -tt

# Without "-tt" the image is padded up to 256x256 (65536 bytes). The extra rows
# are dead weight in VRAM: the model never samples them.
$PTEXCONV \
    -gt -ob -f palette256 -v \
    -o data/brick_pad \
    assets/brick_256x160.png

# One display list serves both textures. obj2dl scales the texture coordinates
# by the height given here, which is the real height, and ptexconv stores the
# image at rows 0..159 whether it trims or pads.
#
# robot.obj is used rather than teapot.obj because its texture coordinates stay
# inside 0..1 on both axes. The teapot's run up to 2, so it needs the texture to
# repeat on the T axis, and that is the one thing a trimmed texture cannot do:
# the GPU repeats at the height it was told (256), not at the real one (160), so
# it would sample rows that were never stored.
python3 $OBJ2DL \
    --input $ASSETS/robot/robot.obj \
    --output data/robot.bin \
    --texture 256 160
