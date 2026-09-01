#!/bin/sh
#
# Builds two archives, so that the example has something to mount twice.

NITRO_ENGINE=../../..
ASSETS=$NITRO_ENGINE/examples/assets
TOOLS=$NITRO_ENGINE/tools
OBJ2DL=$TOOLS/obj2dl/obj2dl.py
NPAC=$TOOLS/npac/npac.py
BLOCKSDS="${BLOCKSDS:-/opt/blocksds/core/}"
GRIT=$BLOCKSDS/tools/grit/grit

rm -rf nitrofiles pack_level pack_docs
mkdir -p nitrofiles pack_level/models pack_level/textures
mkdir -p pack_docs/notes/deeper

python3 $OBJ2DL \
    --input $ASSETS/robot/robot.obj \
    --output pack_level/models/robot.bin \
    --texture 256 256

$GRIT \
    graphics/texture.png \
    -ftb -fh! -W1 \
    -opack_level/textures/texture

printf 'The level archive holds the model and its texture.\n' > pack_docs/readme.txt
printf 'A directory tree survives packing intact.\n' > pack_docs/notes/hello.txt
printf 'Opened through a relative path.\n' > pack_docs/notes/deeper/deep.txt

python3 $NPAC create --input pack_level --output nitrofiles/level.npac --compress auto
python3 $NPAC create --input pack_docs  --output nitrofiles/docs.npac  --compress auto

rm -rf pack_level pack_docs
