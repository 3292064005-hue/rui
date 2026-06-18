#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Save nav_msgs/OccupancyGrid from /map as PGM + PNG + YAML.

This is a small replacement for calling map_server/map_saver manually. It keeps
mapping handoff simple for the project: run slam_mapping.launch, wait until the
map looks complete in RViz, then run save_slam_map.launch.
"""

from __future__ import print_function

import os
import sys

import cv2
import numpy as np
import rospy
from nav_msgs.msg import OccupancyGrid


def map_cell_to_pgm(value):
    # Match the usual ROS map_server convention: occupied black, free white, unknown gray.
    if value < 0:
        return 205
    if value >= 65:
        return 0
    if value <= 25:
        return 254
    return 205


def grid_to_image(grid):
    width = int(grid.info.width)
    height = int(grid.info.height)
    data = list(grid.data)
    image = np.zeros((height, width), dtype=np.uint8)
    # OccupancyGrid origin is bottom-left in map coordinates; image files are top row first.
    for y in range(height):
        row_start = y * width
        image[height - 1 - y, :] = [
            map_cell_to_pgm(data[row_start + x]) for x in range(width)
        ]
    return image


def write_pgm(path, image):
    height, width = image.shape
    with open(path, 'wb') as f:
        header = 'P5\n# CREATOR: myrobot_navigation save_slam_map.py\n%d %d\n255\n' % (width, height)
        f.write(header.encode('ascii'))
        f.write(image.tobytes())


def write_png(path, image):
    if not cv2.imwrite(path, image):
        raise IOError('failed to write PNG: %s' % path)


def write_yaml(path, image_name, grid):
    origin = grid.info.origin
    q = origin.orientation
    # For this 2D map the origin yaw is normally zero. Preserve full quaternion in a comment for traceability.
    with open(path, 'w') as f:
        f.write('image: %s\n' % image_name)
        f.write('resolution: %.10f\n' % grid.info.resolution)
        f.write('origin: [%.10f, %.10f, 0.0]\n' % (origin.position.x, origin.position.y))
        f.write('negate: 0\n')
        f.write('occupied_thresh: 0.65\n')
        f.write('free_thresh: 0.196\n')
        f.write('# origin_quaternion: [%.10f, %.10f, %.10f, %.10f]\n' % (q.x, q.y, q.z, q.w))


def main():
    rospy.init_node('save_slam_map')
    output_dir = os.path.expandvars(os.path.expanduser(str(rospy.get_param('~output_dir', '~/myrobot_navigation_maps'))))
    map_name = str(rospy.get_param('~map_name', 'raicom_slam_map'))
    map_topic = str(rospy.get_param('~map_topic', '/map'))
    timeout = float(rospy.get_param('~timeout', 15.0))

    os.makedirs(output_dir, exist_ok=True)
    pgm_path = os.path.join(output_dir, map_name + '.pgm')
    png_path = os.path.join(output_dir, map_name + '.png')
    yaml_path = os.path.join(output_dir, map_name + '.yaml')

    rospy.loginfo('save_slam_map: waiting for OccupancyGrid on %s ...', map_topic)
    try:
        grid = rospy.wait_for_message(map_topic, OccupancyGrid, timeout=timeout)
    except Exception as exc:
        rospy.logerr('save_slam_map: failed to receive map: %s', exc)
        return 1

    if grid.info.width <= 0 or grid.info.height <= 0 or not grid.data:
        rospy.logerr('save_slam_map: received empty map, not saving')
        return 2

    image = grid_to_image(grid)
    write_pgm(pgm_path, image)
    write_png(png_path, image)
    write_yaml(yaml_path, os.path.basename(pgm_path), grid)
    rospy.loginfo('save_slam_map: saved %s', pgm_path)
    rospy.loginfo('save_slam_map: saved %s', png_path)
    rospy.loginfo('save_slam_map: saved %s', yaml_path)
    print('Saved SLAM map:')
    print('  %s' % pgm_path)
    print('  %s' % png_path)
    print('  %s' % yaml_path)
    return 0


if __name__ == '__main__':
    sys.exit(main())
