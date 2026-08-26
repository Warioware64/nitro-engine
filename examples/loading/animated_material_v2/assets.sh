#!/bin/sh

NITRO_ENGINE=../../..
ASSETS=$NITRO_ENGINE/examples/assets
TOOLS=$NITRO_ENGINE/tools
OBJ2DL=$TOOLS/obj2dl/obj2dl.py

rm -rf data
mkdir -p data

# Four quads, four named materials. obj2dl reads the names straight out of the
# .mtl, and the DLMM file carries them through to the runtime, which is what
# lets a version 2 animation address a material by name.
python3 $OBJ2DL \
    --input $ASSETS/panels/panels.obj \
    --output data/panels_mesh.bin \
    --multi-material

# The textures live with the model so obj2dl can read their sizes from the .mtl;
# grit needs them here.
cp $ASSETS/panels/scroll.png graphics/
cp $ASSETS/panels/pulse.png graphics/
cp $ASSETS/panels/flip.png graphics/
cp $ASSETS/panels/still.png graphics/

python3 gen_animmat.py
