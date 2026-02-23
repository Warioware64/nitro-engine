#!/bin/sh

NITRO_ENGINE=../../..
ASSETS=$NITRO_ENGINE/examples/assets
TOOLS=$NITRO_ENGINE/tools
OBJ2DL=$TOOLS/obj2dl/obj2dl.py

BLOCKSDSEXT="${BLOCKSDSEXT:-/opt/wonderful/thirdparty/blocksds/external}"

rm -rf data
mkdir -p data

# Convert textures to NDS palette256 format using ptexconv
$BLOCKSDSEXT/ptexconv/ptexconv \
    -gt -ob -f palette256 -v \
    -o data/brick_wall \
    $ASSETS/brick_wall_small/brick_wall.jpg

$BLOCKSDSEXT/ptexconv/ptexconv \
    -gt -ob -f palette256 -v \
    -o data/concrete_bare \
    $ASSETS/brick_wall_small/concrete_bare.jpg

# Convert multi-material model to DLMM format
# Texture sizes are auto-detected from the images referenced in the .mtl file
python3 $OBJ2DL \
    --input $ASSETS/brick_wall_small/brick_wall_small.obj \
    --output data/brick_wall_small.bin \
    --multi-material
