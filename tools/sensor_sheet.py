#!/usr/bin/env python3
"""Contact sheet of the Phase 2 sensor test: wall camera | front camera | lidar scan (robot frame,
x up the face), one row per world. Reads results/sensors_<world>_{wall,front}.png and _lidar.csv
(written by tools/win_webots_test.ps1 -Mode sensors) and writes results/webots_sensors_sheet.png.

usage: python tools/sensor_sheet.py winter_dam gravity_dam goldeneye_gorge
needs: pip install pillow
"""
import csv
import math
import sys
from pathlib import Path

from PIL import Image, ImageDraw

RES = Path(__file__).resolve().parents[1] / 'results'
W, H = 400, 300                      # tile size
RANGE = 12.0                         # lidar plot radius [m]


def lidar_tile(path):
    im = Image.new('RGB', (W, H), (24, 26, 30))
    d = ImageDraw.Draw(im)
    cx, cy, s = W / 2, H / 2, (H / 2 - 10) / RANGE
    for r in (2, 4, 6, 8, 10, 12):
        d.ellipse([cx - r * s, cy - r * s, cx + r * s, cy + r * s], outline=(55, 60, 68))
    d.line([cx, cy, cx, cy - 20], fill=(90, 200, 90), width=2)        # x (up the face) = image up
    for row in csv.DictReader(open(path)):
        if row['range_m'] == 'inf':
            continue
        a, r = float(row['angle_rad']), float(row['range_m'])
        x, y = r * math.cos(a), r * math.sin(a)                       # robot frame
        px, py = cx - y * s, cy - x * s
        d.ellipse([px - 1.5, py - 1.5, px + 1.5, py + 1.5], fill=(255, 170, 40))
    d.ellipse([cx - 3, cy - 3, cx + 3, cy + 3], fill=(90, 200, 90))
    d.text((6, 4), 'lidar (12 m rings 2 m), up = up the face', fill=(200, 200, 200))
    return im


def main():
    worlds = sys.argv[1:]
    sheet = Image.new('RGB', (3 * W, len(worlds) * (H + 18)), (12, 12, 14))
    d = ImageDraw.Draw(sheet)
    for i, w in enumerate(worlds):
        y = i * (H + 18)
        d.text((6, y + 3), w, fill=(230, 230, 230))
        tiles = [Image.open(RES / f'sensors_{w}_wall.png').convert('RGB').resize((W, H)),
                 Image.open(RES / f'sensors_{w}_front.png').convert('RGB').resize((W, H)),
                 lidar_tile(RES / f'sensors_{w}_lidar.csv')]
        for j, t in enumerate(tiles):
            sheet.paste(t, (j * W, y + 18))
        d.text((6, y + 22), 'wall camera', fill=(255, 255, 0))
        d.text((W + 6, y + 22), 'front camera', fill=(255, 255, 0))
    sheet.save(RES / 'webots_sensors_sheet.png')
    print('wrote', RES / 'webots_sensors_sheet.png')


if __name__ == '__main__':
    main()
