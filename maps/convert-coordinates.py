#!/usr/bin/env python3

import sys

assert len(sys.argv) == 7

width_pixels = int(sys.argv[1])
height_pixels = int(sys.argv[2])
resolution = float(sys.argv[3])
if sys.argv[4] == 'px2m':
    x_pixels = int(sys.argv[5])
    y_pixels = int(sys.argv[6])

    x_meters = x_pixels * resolution
    y_meters = (height_pixels - y_pixels) * resolution

    print(x_meters, y_meters)

elif sys.argv[4] == 'm2px':
    x_meters = float(sys.argv[5])
    y_meters = float(sys.argv[6])

    x_pixels = int(x_meters / resolution)
    y_pixels = height_pixels - int(y_meters / resolution)

    print(x_pixels, y_pixels)

else:
    raise ValueError('unknown mode')
