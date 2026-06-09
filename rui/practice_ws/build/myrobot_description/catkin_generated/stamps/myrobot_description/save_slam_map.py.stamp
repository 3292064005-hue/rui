#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Save nav_msgs/OccupancyGrid from /map as PGM + YAML.

This is a small replacement for calling map_server/map_saver manually. It keeps
mapping handoff simple for the project: run slam_mapping.launch, wait until the
map looks complete in RViz, then run save_slam_map.launch.
"""

from __future__ import print_function

import os
import sys

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


def write_pgm(path, grid):
    width = int(grid.info.width)
    height = int(grid.info.height)
    data = list(grid.data)
    with open(path, 'wb') as f:
        header = 'P5\n# CREATOR: myrobot_description save_slam_map.py\n%d %d\n255\n' % (width, height)
        f.write(header.encode('ascii'))
        # OccupancyGrid origin is bottom-left in map coordinates; PGM is top row first.
        for y in range(height - 1, -1, -1):
            row_start = y * width
            row = bytearray(map_cell_to_pgm(data[row_start + x]) for x in range(width))
            f.write(row)


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
    output_dir = os.path.expandvars(os.path.expanduser(str(rospy.get_param('~output_dir', '~/myrobot_description_maps'))))
    map_name = str(rospy.get_param('~map_name', 'raicom_slam_map'))
    map_topic = str(rospy.get_param('~map_topic', '/map'))
    timeout = float(rospy.get_param('~timeout', 15.0))

    os.makedirs(output_dir, exist_ok=True)
    pgm_path = os.path.join(output_dir, map_name + '.pgm')
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

    write_pgm(pgm_path, grid)
    write_yaml(yaml_path, os.path.basename(pgm_path), grid)
    rospy.loginfo('save_slam_map: saved %s', pgm_path)
    rospy.loginfo('save_slam_map: saved %s', yaml_path)
    print('Saved SLAM map:')
    print('  %s' % pgm_path)
    print('  %s' % yaml_path)
    return 0


if __name__ == '__main__':
    sys.exit(main())
