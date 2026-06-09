#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Generate the default map_server occupancy map from worlds/rm_map.world.

The script reads the existing Gazebo world geometry and writes maps/raicom_known_map.pgm/.yaml.
It does not modify worlds/rm_map.world.
"""

from __future__ import print_function

import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


def generate(pkg_root):
    pkg_root = Path(pkg_root).resolve()
    world = pkg_root / 'worlds' / 'rm_map.world'
    map_dir = pkg_root / 'maps'
    map_dir.mkdir(exist_ok=True)

    resolution = 0.02
    x_min, x_max = -0.50, 5.00
    y_min, y_max = -4.00, 0.50
    floor_x_min, floor_x_max = -0.25, 4.75
    floor_y_min, floor_y_max = -3.75, 0.25

    width = int(math.ceil((x_max - x_min) / resolution))
    height = int(math.ceil((y_max - y_min) / resolution))
    img = [[254 for _ in range(width)] for __ in range(height)]

    # Occupy outside the legal floor area so global planning stays inside the arena.
    for v in range(height):
        wy = y_max - (v + 0.5) * resolution
        for u in range(width):
            wx = x_min + (u + 0.5) * resolution
            if wx < floor_x_min or wx > floor_x_max or wy < floor_y_min or wy > floor_y_max:
                img[v][u] = 0

    tree = ET.parse(str(world))
    boxes = []
    for collision in tree.getroot().findall('.//collision'):
        name = collision.attrib.get('name', '')
        if 'floor' in name:
            continue
        pose_text = collision.findtext('pose')
        size_text = collision.findtext('geometry/box/size')
        if not pose_text or not size_text:
            continue
        pose = [float(x) for x in pose_text.split()]
        size = [float(x) for x in size_text.split()]
        x, y, z = pose[0], pose[1], pose[2]
        sx, sy, sz = size[0], size[1], size[2]
        if z + sz / 2.0 < 0.05:
            continue
        inflate = 0.015
        boxes.append((name, x - sx / 2 - inflate, x + sx / 2 + inflate, y - sy / 2 - inflate, y + sy / 2 + inflate))

    for _name, bx0, bx1, by0, by1 in boxes:
        u0 = max(0, int(math.floor((bx0 - x_min) / resolution)))
        u1 = min(width - 1, int(math.ceil((bx1 - x_min) / resolution)))
        v0 = max(0, int(math.floor((y_max - by1) / resolution)))
        v1 = min(height - 1, int(math.ceil((y_max - by0) / resolution)))
        for v in range(v0, v1 + 1):
            for u in range(u0, u1 + 1):
                img[v][u] = 0

    pgm_path = map_dir / 'raicom_known_map.pgm'
    with open(str(pgm_path), 'w') as f:
        f.write('P2\n')
        f.write('# Generated from worlds/rm_map.world for ROS map_server. The world file itself is not modified.\n')
        f.write('%d %d\n255\n' % (width, height))
        for row in img:
            f.write(' '.join(str(v) for v in row) + '\n')

    yaml_path = map_dir / 'raicom_known_map.yaml'
    yaml_path.write_text('''image: raicom_known_map.pgm
resolution: %.6f
origin: [%.6f, %.6f, 0.000000]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
''' % (resolution, x_min, y_min))
    return pgm_path, yaml_path, len(boxes), width, height


def main():
    pkg_root = Path(__file__).resolve().parents[1]
    pgm_path, yaml_path, box_count, width, height = generate(pkg_root)
    print('[OK] generated %s' % pgm_path)
    print('[OK] generated %s' % yaml_path)
    print('[OK] occupied boxes: %d, size: %dx%d' % (box_count, width, height))
    return 0


if __name__ == '__main__':
    sys.exit(main())
