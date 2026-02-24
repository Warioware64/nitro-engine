#!/bin/sh
#!/bin/sh

NITRO_ENGINE=../../..
ASSETS=$NITRO_ENGINE/examples/assets
TOOLS=$NITRO_ENGINE/tools
MD5_TO_DSMA=$TOOLS/md5_to_dsma/md5_to_dsma.py

rm -rf data
mkdir -p data

python3 $MD5_TO_DSMA \
    --model $ASSETS/robot_multi_material/Robot.md5mesh \
    --name robot \
    --output data \
    --anim $ASSETS/robot_multi_material/Walk.md5anim $ASSETS/robot_multi_material/Wave.md5anim \
    --skip-frames 1 \
    --bin \
    --blender-fix \
    --multi-material
