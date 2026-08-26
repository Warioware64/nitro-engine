#!/bin/sh

NITRO_ENGINE=../../..
ASSETS=$NITRO_ENGINE/examples/assets
TOOLS=$NITRO_ENGINE/tools
OBJ2DL=$TOOLS/obj2dl/obj2dl.py

rm -rf data
mkdir -p data

# --envmap-uv pins every texture coordinate to the centre of the texture, which
# is what sphere mapping needs. The same mesh is also exported normally so the
# example can show what happens without it.
python3 $OBJ2DL \
    --input $ASSETS/teapot.obj \
    --output data/teapot_env.bin \
    --texture 64 64 \
    --scale 0.1 \
    --envmap-uv

python3 $OBJ2DL \
    --input $ASSETS/teapot.obj \
    --output data/teapot_uv.bin \
    --texture 64 64 \
    --scale 0.1
