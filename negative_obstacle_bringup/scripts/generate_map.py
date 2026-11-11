#!/usr/bin/env python3
# Copyright 2026 negative_obstacle_bringup contributors.
#
# Generates a static occupancy map of the factory (walls, loading dock, racks).
# The negative obstacles (pits) are intentionally left FREE: they are detected
# at runtime by the negative-obstacle layer, not baked into the map.

import os

import numpy as np

RESOLUTION = 0.05  # m/pixel
WIDTH_M = 24.0
HEIGHT_M = 18.0
ORIGIN_X = -12.0
ORIGIN_Y = -9.0

W = int(WIDTH_M / RESOLUTION)   # 480
H = int(HEIGHT_M / RESOLUTION)  # 360

# 255 = free (white), 0 = occupied (black).
img = np.full((H, W), 255, dtype=np.uint8)


def fill_rect(xmin, xmax, ymin, ymax):
    """Fill a world-space rectangle as occupied."""
    x0 = int((xmin - ORIGIN_X) / RESOLUTION)
    x1 = int((xmax - ORIGIN_X) / RESOLUTION)
    y0 = int((ORIGIN_Y + HEIGHT_M - ymax) / RESOLUTION)
    y1 = int((ORIGIN_Y + HEIGHT_M - ymin) / RESOLUTION)
    img[max(0, y0):min(H, y1), max(0, x0):min(W, x1)] = 0


# Perimeter walls (0.2 m thick). The east wall has a 4 m door opening
# (y in [-2, 2]) at ground level.
fill_rect(-12.1, -11.9, -9.1, 9.1)      # west
fill_rect(11.9, 12.1, -9.1, -2.0)       # east, south of door
fill_rect(11.9, 12.1, 2.0, 9.1)         # east, north of door
fill_rect(-12.1, 12.1, 8.9, 9.1)        # north
fill_rect(-12.1, 12.1, -9.1, -8.9)      # south

# Loading dock platform (0.30 m raised).
fill_rect(7.5, 11.5, -7.5, -4.5)

# Storage racks (2.4 x 1.0 m) at y = +/-6.
for xc in (-6.0, 0.0):
    for yc in (6.0, -6.0):
        fill_rect(xc - 1.2, xc + 1.2, yc - 0.5, yc + 0.5)


def main():
    out_dir = os.path.join(os.path.dirname(__file__), '..', 'maps')
    os.makedirs(out_dir, exist_ok=True)

    pgm_path = os.path.join(out_dir, 'map.pgm')
    with open(pgm_path, 'wb') as f:
        f.write(f'P5\n{W} {H}\n255\n'.encode())
        f.write(img.tobytes())

    yaml_path = os.path.join(out_dir, 'map.yaml')
    with open(yaml_path, 'w') as f:
        f.write('image: map.pgm\n')
        f.write(f'resolution: {RESOLUTION}\n')
        f.write(f'origin: [{ORIGIN_X}, {ORIGIN_Y}, 0.0]\n')
        f.write('negate: 0\n')
        f.write('occupied_thresh: 0.65\n')
        f.write('free_thresh: 0.196\n')

    print(f'Wrote {pgm_path} and {yaml_path} ({W}x{H} px)')


if __name__ == '__main__':
    main()
