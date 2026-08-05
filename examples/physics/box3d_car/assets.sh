#!/bin/sh

NITRO_ENGINE=../../..
ASSETS=$NITRO_ENGINE/examples/assets
TOOLS=$NITRO_ENGINE/tools
OBJ2DL=$TOOLS/obj2dl/obj2dl.py

mkdir -p data source

# The level: a display list to draw, and a .b3mesh to collide against.
#
# The two scales are different, and that is the point of this example's asset
# step rather than an accident:
#
#   --scale 0.5              sizes the DISPLAY LIST. A display list stores
#                            positions as v16, which tops out at 8 units, and
#                            the level is 12 x 17. main.c scales the model back
#                            up by 2 when it draws.
#
#   --collision-b3-scale 0.5 divides on the way into the BAKE, undoing that, so
#                            the collision mesh is at true scale. The solver's
#                            tolerances are absolute and want 1 unit to be about
#                            1 metre; a level baked at render scale is half the
#                            size the solver was tuned for.
#
# --collision-b3-c writes source/level_b3mesh.c, an aligned(8) C array. Use it
# rather than dropping the .b3mesh in data/: bin2c emits aligned(4), b3MeshData
# opens with a uint64_t, and the ARM9 reads one with LDRD.
python3 $OBJ2DL \
    --input $ASSETS/level.obj \
    --output data/level.bin \
    --texture 32 32 \
    --scale 0.5 \
    --collision-b3 \
    --collision-b3-scale 0.5 \
    --collision-b3-c source/level

# The raw .b3mesh is not what this example ships -- source/level_b3mesh.c is.
# Removed so that data/ carries only files that belong there; keep it instead if
# you are loading the level from NitroFS with NEA_FATLoadData(), where malloc
# gives you the 8-byte alignment the array attribute gives you here.
rm -f data/level.b3mesh

# The falling boxes.
python3 $OBJ2DL \
    --input $ASSETS/cube.obj \
    --output data/cube.bin \
    --texture 32 32

# The wheels. Spheres rather than boxes: the wheel joint constrains the axle,
# so the shape only has to roll -- and a box wheel judders between its corners
# and its faces, which is a property of the mesh rather than of the joint.
python3 $OBJ2DL \
    --input $ASSETS/sphere.obj \
    --output data/sphere.bin \
    --texture 32 32
