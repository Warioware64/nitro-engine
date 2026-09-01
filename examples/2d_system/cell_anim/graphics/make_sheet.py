#!/usr/bin/env python3
# SPDX-License-Identifier: CC0-1.0
#
# SPDX-FileContributor: Warioware64, 2026
#
# Draws the example spritesheet. The art is generated rather than shipped as a
# PNG so that the example carries no third-party pixels and so the sheet can be
# regenerated at a different size without an editor.

import math

from PIL import Image, ImageDraw

COLS, ROWS = 4, 2
TILE = 32
FRAMES = COLS * ROWS

BODY = (232, 96, 72, 255)
DARK = (152, 40, 40, 255)
EYE = (255, 255, 255, 255)
FOOT = (64, 48, 96, 255)


def frame(draw, ox, oy, i):
    t = i / FRAMES
    # A squash-and-stretch hop, which reads clearly at 32 pixels and makes a
    # dropped or mistimed frame obvious at a glance.
    bounce = math.sin(t * 2 * math.pi)
    squash = 1.0 - 0.18 * bounce
    lift = -5.0 * max(0.0, bounce)

    w = 20 / squash
    h = 20 * squash
    cx = ox + TILE / 2
    cy = oy + TILE - 6 + lift - h / 2

    draw.ellipse([cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2], fill=BODY)
    draw.ellipse([cx - w / 2, cy + h / 2 - 5, cx + w / 2, cy + h / 2],
                 fill=DARK)

    look = 3 * math.cos(t * 2 * math.pi)
    for dx in (-4, 4):
        draw.ellipse([cx + dx - 2 + look, cy - 4, cx + dx + 2 + look, cy],
                     fill=EYE)

    # Feet swing out of phase, so the walk cycle is not mirror-symmetric.
    for k, dx in enumerate((-6, 6)):
        swing = 3 * math.sin(t * 2 * math.pi + k * math.pi)
        fy = oy + TILE - 4 - min(0.0, lift) * 0.4
        draw.ellipse([cx + dx + swing - 4, fy - 3,
                      cx + dx + swing + 4, fy + 1], fill=FOOT)


def main():
    img = Image.new("RGBA", (COLS * TILE, ROWS * TILE), (0, 0, 0, 0))
    draw = ImageDraw.Draw(img)
    for i in range(FRAMES):
        frame(draw, (i % COLS) * TILE, (i // COLS) * TILE, i)
    img.save("hopper.png")
    print("wrote hopper.png (%dx%d, %d frames)"
          % (img.size[0], img.size[1], FRAMES))


if __name__ == "__main__":
    main()
