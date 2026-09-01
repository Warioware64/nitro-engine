#!/bin/sh
#
# Builds the assets, then packs them into a single NPAC archive. Only the
# archive is put in nitrofiles/, so the ROM's own filesystem holds one file
# instead of three.

NITRO_ENGINE=../../..
ASSETS=$NITRO_ENGINE/examples/assets
TOOLS=$NITRO_ENGINE/tools
OBJ2DL=$TOOLS/obj2dl/obj2dl.py
NPAC=$TOOLS/npac/npac.py
BLOCKSDS="${BLOCKSDS:-/opt/blocksds/core/}"
GRIT=$BLOCKSDS/tools/grit/grit

rm -rf nitrofiles pack
mkdir -p nitrofiles pack/models pack/textures

python3 $OBJ2DL \
    --input $ASSETS/robot/robot.obj \
    --output pack/models/robot.bin \
    --texture 256 256

$GRIT \
    graphics/texture.png \
    -ftb -fh! -W1 \
    -opack/textures/texture

printf 'Everything this ROM loads lives in one NPAC archive.\n' > pack/readme.txt

# --compress auto tries every codec on every file and keeps whichever wins,
# storing the file when none of them does.
python3 $NPAC -v create \
    --input pack \
    --output nitrofiles/assets.npac \
    --compress auto

rm -rf pack
