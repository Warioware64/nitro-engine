#!/bin/sh

NITRO_ENGINE=../../..
ASSETS=$NITRO_ENGINE/examples/assets
TOOLS=$NITRO_ENGINE/tools
MD5_TO_DSMA=$TOOLS/md5_to_dsma/md5_to_dsma.py

rm -rf data
mkdir -p data

# The tentacle is smoothly weighted: every vertex blends between two bones, and
# the blend varies per vertex, so it wants 194 matrix-palette slots for its 11
# bone pairs. The stack has 30. The exporter's clustering pass is what makes it
# fit, and it prints the blend error it cost to get there.
#
# Three budgets are exported so the example can show the same mesh deformed with
# progressively fewer slots.
for budget in 30 16 11; do
    python3 $MD5_TO_DSMA \
        --model $ASSETS/tentacle/Tentacle.md5mesh \
        --name tentacle$budget \
        --output data \
        --anims $ASSETS/tentacle/Wave.md5anim \
        --texture 64 64 \
        --bin \
        --max-nodes $budget \
        --format nsmw
done
