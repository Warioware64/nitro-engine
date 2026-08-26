#!/bin/sh

NITRO_ENGINE=../../..
ASSETS=$NITRO_ENGINE/examples/assets
TOOLS=$NITRO_ENGINE/tools
MD5_TO_DSMA=$TOOLS/md5_to_dsma/md5_to_dsma.py

BLOCKSDSEXT="${BLOCKSDSEXT:-/opt/wonderful/thirdparty/blocksds/external}"
PTEXCONV=$BLOCKSDSEXT/ptexconv/ptexconv

rm -rf data
mkdir -p data

# brick_256x160.png is examples/assets/brick_wall_small/brick_wall.jpg resized to
# 160 rows. "-tt" trims the T axis, so only those 160 rows are written out
# (40960 bytes) instead of being padded up to 256 (65536 bytes). The brick
# courses run horizontally, so a texture that crept along the T axis during the
# animation would be obvious.
$PTEXCONV \
    -gt -ob -f palette256 -v \
    -o data/brick_trim \
    assets/brick_256x160.png \
    -tt

# md5_to_dsma scales the V coordinates by the height given here. Passing the
# real height, not the padded one, is what makes the animated model sample rows
# 0..159 and nothing else, at every frame of the animation.
python3 $MD5_TO_DSMA \
    --model $ASSETS/robot/Robot.md5mesh \
    --name robot \
    --output data \
    --texture 256 160 \
    --anim $ASSETS/robot/Walk.md5anim $ASSETS/robot/Wave.md5anim \
    --skip-frames 1 \
    --bin \
    --blender-fix
