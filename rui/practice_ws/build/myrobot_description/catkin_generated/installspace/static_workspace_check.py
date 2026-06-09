#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Offline static checks for this ROS package.

Run after unzipping, before catkin_make if you want a quick sanity check:

  python3 src/myrobot_description/scripts/static_workspace_check.py

The script intentionally does not import rospy, xacro, or Gazebo. It verifies
basic file presence, XML parseability, mesh references, executable bits, and that
worlds/rm_map.world matches the stored SHA-256 baseline for this package.
"""

from __future__ import print_function

import hashlib
import os
import re
import stat
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


def sha256(path):
    h = hashlib.sha256()
    with open(str(path), 'rb') as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


def check_xml(path, errors):
    try:
        ET.parse(str(path))
        print('[OK] XML parse:', path.name)
    except Exception as exc:
        errors.append('%s XML parse failed: %s' % (path, exc))


def main():
    root = Path(__file__).resolve().parents[1]
    errors = []

    required = [
        root / 'package.xml',
        root / 'CMakeLists.txt',
        root / 'urdf' / 'turtlebot3_mecanum.urdf.xacro',
        root / 'urdf' / 'camera.xacro',
        root / 'launch' / 'task_patrol.launch',
        root / 'launch' / 'task_patrol_simple.launch',
        root / 'launch' / 'slam_mapping.launch',
        root / 'launch' / 'save_slam_map.launch',
        root / 'launch' / 'navigation.launch',
        root / 'launch' / 'autonomous_navigation.launch',
        root / 'launch' / 'send_navigation_goals.launch',
        root / 'launch' / 'full_simulation.launch',
        root / 'launch' / 'test_sim_drivers.launch',
        root / 'config' / 'simulation_params.yaml',
        root / 'config' / 'rm_map.sha256',
        root / 'config' / 'simulation.rviz',
        root / 'scripts' / 'mecanum_sim_driver.py',
        root / 'scripts' / 'simulation_status_monitor.py',
        root / 'scripts' / 'cmd_vel_test_motion.py',
        root / 'config' / 'task_params.yaml',
        root / 'config' / 'navigation_params.yaml',
        root / 'config' / 'slam_gmapping_params.yaml',
        root / 'config' / 'amcl_params.yaml',
        root / 'config' / 'move_base_params.yaml',
        root / 'config' / 'costmap_common_params.yaml',
        root / 'config' / 'global_costmap_params.yaml',
        root / 'config' / 'local_costmap_params.yaml',
        root / 'config' / 'base_local_planner_params.yaml',
        root / 'config' / 'slam.rviz',
        root / 'config' / 'navigation.rviz',
        root / 'maps' / 'raicom_known_map.yaml',
        root / 'maps' / 'raicom_known_map.pgm',
        root / 'scripts' / 'slam_status_monitor.py',
        root / 'scripts' / 'scan_self_filter.py',
        root / 'scripts' / 'save_slam_map.py',
        root / 'scripts' / 'move_base_waypoint_navigator.py',
        root / 'scripts' / 'navigation_initializer.py',
        root / 'scripts' / 'navigation_status_monitor.py',
        root / 'scripts' / 'generate_known_map.py',
        root / 'worlds' / 'rm_map.world',
        root / 'materials' / 'scripts' / 'raicom_targets.material',
        root / 'materials' / 'textures' / 'zone1_panel.jpg',
        root / 'materials' / 'textures' / 'zone2_panel.jpg',
    ]
    for path in required:
        if path.exists():
            print('[OK] exists:', path.relative_to(root))
        else:
            errors.append('missing file: %s' % path)

    for path in [root / 'package.xml'] + sorted((root / 'launch').glob('*.launch')) + sorted((root / 'urdf').glob('*.xacro')):
        if path.exists():
            check_xml(path, errors)

    rospack = None
    for candidate in ('rospack', '/opt/ros/noetic/bin/rospack'):
        if os.path.exists(candidate) or os.system('command -v %s >/dev/null 2>&1' % candidate) == 0:
            rospack = candidate
            break
    if rospack:
        required_ros_packages = [
            'gazebo_ros',
            'gazebo_plugins',
            'joint_state_publisher',
            'joint_state_publisher_gui',
            'robot_state_publisher',
            'rviz',
            'xacro',
            'cv_bridge',
            'map_server',
            'amcl',
            'move_base',
            'navfn',
            'base_local_planner',
            'costmap_2d',
            'clear_costmap_recovery',
            'rotate_recovery',
            'gmapping',
        ]
        for package_name in required_ros_packages:
            result = subprocess.run([rospack, 'find', package_name], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            if result.returncode == 0:
                print('[OK] ros package:', package_name)
            else:
                errors.append('missing ROS package: %s' % package_name)
    else:
        print('[WARN] rospack not found; skipping runtime ROS package checks')

    # Verify package:// mesh references used in xacro files.
    mesh_refs = set()
    for path in sorted((root / 'urdf').glob('*.xacro')):
        text = path.read_text(errors='ignore')
        for rel in re.findall(r'package://myrobot_description/([^"\']+)', text):
            mesh_refs.add(rel)
    for rel in sorted(mesh_refs):
        target = root / rel
        if target.exists():
            print('[OK] mesh reference:', rel)
        else:
            errors.append('missing package resource: %s' % rel)

    # Verify script executable bits.
    for path in sorted((root / 'scripts').glob('*.py')):
        mode = path.stat().st_mode
        if mode & stat.S_IXUSR:
            print('[OK] executable:', path.name)
        else:
            errors.append('script is not executable: %s' % path)

    # Verify the checked-in world against the checked-in baseline hash.
    expected_path = root / 'config' / 'rm_map.sha256'
    world_path = root / 'worlds' / 'rm_map.world'
    if expected_path.exists() and world_path.exists():
        expected = expected_path.read_text().strip().split()[0]
        actual = sha256(world_path)
        if actual == expected:
            print('[OK] rm_map.world SHA-256 unchanged:', actual)
        else:
            errors.append('rm_map.world hash changed: expected %s actual %s' % (expected, actual))

    template_files = sorted((root / 'recognition_templates').glob('*.jpg'))
    if len(template_files) >= 5:
        print('[OK] recognition templates:', len(template_files))
    else:
        errors.append('recognition_templates has too few images: found %d' % len(template_files))


    # Verify default navigation map files are internally consistent.
    map_yaml = root / 'maps' / 'raicom_known_map.yaml'
    map_pgm = root / 'maps' / 'raicom_known_map.pgm'
    if map_yaml.exists() and map_pgm.exists():
        yaml_text = map_yaml.read_text(errors='ignore')
        if 'image: raicom_known_map.pgm' in yaml_text and 'resolution:' in yaml_text and 'origin:' in yaml_text:
            print('[OK] navigation map yaml:', map_yaml.relative_to(root))
        else:
            errors.append('navigation map yaml is missing image/resolution/origin fields')
        try:
            with open(str(map_pgm), 'r') as f:
                magic = f.readline().strip()
                comment_or_size = f.readline().strip()
                size_line = f.readline().strip() if comment_or_size.startswith('#') else comment_or_size
                width, height = [int(x) for x in size_line.split()[:2]]
            if magic == 'P2' and width > 0 and height > 0:
                print('[OK] navigation map pgm: %dx%d' % (width, height))
            else:
                errors.append('navigation map pgm header is invalid')
        except Exception as exc:
            errors.append('navigation map pgm cannot be read: %s' % exc)

    if errors:
        print('\nStatic check failed:')
        for item in errors:
            print('  -', item)
        return 1

    print('\nStatic check passed.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
