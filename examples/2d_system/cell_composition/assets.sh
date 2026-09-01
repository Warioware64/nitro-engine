#!/bin/sh

# The art and the multi-cell bank are generated together: the bank references
# the pieces by their rectangles in the sheet, so drawing and packing have to
# agree and there is no point shipping one without the other.
python3 graphics/make_assets.py || exit 1
