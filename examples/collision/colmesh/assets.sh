#!/bin/sh

NITRO_ENGINE=../../..
ASSETS=$NITRO_ENGINE/examples/assets
TOOLS=$NITRO_ENGINE/tools
OBJ2DL=$TOOLS/obj2dl/obj2dl.py
MD5_TO_DSMA=$TOOLS/md5_to_dsma/md5_to_dsma.py

rm -rf data
mkdir -p data

# Teapot: display list + collision mesh (--collision generates .colmesh)
python3 $OBJ2DL \
    --input $ASSETS/teapot.obj \
    --output data/teapot.bin \
    --texture 32 32 \
    --scale 0.3 \
    --collision

# Rename .colmesh to .bin so BlocksDS BIN2C rule picks it up
mv data/teapot.colmesh data/teapot_col.bin

# Sphere for bouncing object
python3 $OBJ2DL \
    --input $ASSETS/sphere.obj \
    --output data/sphere.bin \
    --texture 32 32

# Robot animated model
python3 $MD5_TO_DSMA \
    --model $ASSETS/robot_multi_material/Robot.md5mesh \
    --name robot \
    --output data \
    --texture 256 256 \
    --anim $ASSETS/robot_multi_material/Walk.md5anim \
    --skip-frames 1 \
    --bin \
    --multi-material \
    --collision $ASSETS/robot_multi_material/Robot.md5collimesh \
    --blender-fix
