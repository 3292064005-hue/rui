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


def read_pgm(path):
    with open(str(path), 'rb') as handle:
        tokens = []
        while len(tokens) < 4:
            line = handle.readline()
            if not line:
                raise ValueError('incomplete PGM header')
            line = line.split(b'#', 1)[0]
            tokens.extend(line.split())
        magic = tokens[0].decode('ascii')
        width = int(tokens[1])
        height = int(tokens[2])
        max_value = int(tokens[3])
        if magic == 'P5':
            pixels = list(handle.read(width * height))
        elif magic == 'P2':
            pixels = [int(value) for value in handle.read().split()]
        else:
            raise ValueError('unsupported PGM magic %s' % magic)
    if max_value != 255 or len(pixels) != width * height:
        raise ValueError('invalid PGM pixels or max value')
    return magic, width, height, pixels


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
        root / 'maps' / 'raicom_slam_map_final.yaml',
        root / 'maps' / 'raicom_slam_map_final.pgm',
        root / 'recognition_weights' / 'README.md',
        root / 'scripts' / 'slam_status_monitor.py',
        root / 'scripts' / 'scan_self_filter.py',
        root / 'scripts' / 'odom_laser_mapper.py',
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

    task_params = root / 'config' / 'task_params.yaml'
    task_text = task_params.read_text(encoding='utf-8')
    for key in [
            'recognition_backend:',
            'recognition_weights:',
            'recognition_model_labels:',
            'recognition_model_input_size:',
            'recognition_model_confidence:',
            'recognition_yolo_iou_threshold:',
            'recognition_yolo_class_names:',
            'recognition_yolo_label_map:']:
        if key in task_text:
            print('[OK] recognition model parameter:', key[:-1])
        else:
            errors.append('missing recognition model parameter: %s' % key[:-1])

    yolo_weights = root / 'recognition_weights' / 'best.onnx'
    if yolo_weights.exists() and yolo_weights.stat().st_size > 1024 * 1024:
        print('[OK] YOLO recognition weights:', yolo_weights.relative_to(root))
    else:
        errors.append('missing or invalid YOLO recognition weights: %s' %
                      yolo_weights.relative_to(root))
    ort_binding = (
        root / 'python_vendor' / 'onnxruntime' / 'capi' /
        'onnxruntime_pybind11_state.cpython-38-x86_64-linux-gnu.so')
    if ort_binding.exists() and ort_binding.stat().st_size > 1024 * 1024:
        print('[OK] bundled ONNX Runtime:', ort_binding.relative_to(root))
    else:
        errors.append('missing bundled ONNX Runtime Python 3.8 binding')

    # Verify the final SLAM map and every main navigation entrypoint.
    map_yaml = root / 'maps' / 'raicom_slam_map_final.yaml'
    map_pgm = root / 'maps' / 'raicom_slam_map_final.pgm'
    if map_yaml.exists() and map_pgm.exists():
        yaml_text = map_yaml.read_text(errors='ignore')
        if 'image: raicom_slam_map_final.pgm' in yaml_text and 'resolution:' in yaml_text and 'origin:' in yaml_text:
            print('[OK] navigation map yaml:', map_yaml.relative_to(root))
        else:
            errors.append('navigation map yaml is missing image/resolution/origin fields')
        try:
            magic, width, height, pixels = read_pgm(map_pgm)
            if magic in ('P2', 'P5') and width > 0 and height > 0:
                print('[OK] navigation map pgm: %dx%d' % (width, height))
            else:
                errors.append('navigation map pgm header is invalid')
            known_cells = sum(
                1 for value in pixels if value < 65 or value > 250)
            known_ratio = float(known_cells) / float(width * height)
            if known_ratio >= 0.95:
                print('[OK] final map known coverage: %.1f%%' %
                      (known_ratio * 100.0))
            else:
                errors.append(
                    'final map known coverage is too low: %.1f%% < 95.0%%' %
                    (known_ratio * 100.0))

            occupied = [
                index for index, value in enumerate(pixels) if value < 65]
            if len(occupied) >= 3000:
                print('[OK] final map occupied cells:', len(occupied))
            else:
                errors.append(
                    'final map has too few occupied cells: %d < 3000' %
                    len(occupied))
            if occupied:
                occupied_rows = [index // width for index in occupied]
                occupied_cols = [index % width for index in occupied]
                bbox_width = max(occupied_cols) - min(occupied_cols) + 1
                bbox_height = max(occupied_rows) - min(occupied_rows) + 1
                if bbox_width >= 240 and bbox_height >= 190:
                    print('[OK] final map occupied bounds: %dx%d' %
                          (bbox_width, bbox_height))
                else:
                    errors.append(
                        'final map occupied bounds are incomplete: %dx%d' %
                        (bbox_width, bbox_height))

            resolution_match = re.search(
                r'^resolution:\s*([-+0-9.eE]+)', yaml_text, re.MULTILINE)
            origin_match = re.search(
                r'^origin:\s*\[\s*([-+0-9.eE]+)\s*,\s*'
                r'([-+0-9.eE]+)', yaml_text, re.MULTILINE)
            if resolution_match and origin_match:
                resolution = float(resolution_match.group(1))
                origin_x = float(origin_match.group(1))
                origin_y = float(origin_match.group(2))
                false_wall_cells = 0
                for index, value in enumerate(pixels):
                    if value >= 65:
                        continue
                    row, col = divmod(index, width)
                    world_x = origin_x + (col + 0.5) * resolution
                    world_y = origin_y + (height - row - 0.5) * resolution
                    if (2.50 <= world_x <= 4.25 and
                            -3.60 <= world_y <= -3.40):
                        false_wall_cells += 1
                if false_wall_cells == 0:
                    print('[OK] final map lower-right corridor is clear')
                else:
                    errors.append(
                        'final map lower-right corridor contains %d occupied cells' %
                        false_wall_cells)
        except Exception as exc:
            errors.append('navigation map pgm cannot be read: %s' % exc)

    expected_map_default = 'maps/raicom_slam_map_final.yaml'
    for launch_name in (
            'navigation.launch',
            'task_patrol.launch',
            'autonomous_navigation.launch'):
        launch_path = root / 'launch' / launch_name
        if expected_map_default in launch_path.read_text(errors='ignore'):
            print('[OK] default navigation map:', launch_name)
        else:
            errors.append(
                '%s does not default to %s' %
                (launch_name, expected_map_default))

    slam_launch_text = (
        root / 'launch' / 'slam_mapping.launch').read_text(errors='ignore')
    if '<arg name="patrol_repeats" default="3" />' in slam_launch_text:
        print('[OK] SLAM patrol repeats: 3')
    else:
        errors.append('slam_mapping.launch does not default to 3 patrol repeats')

    if errors:
        print('\nStatic check failed:')
        for item in errors:
            print('  -', item)
        return 1

    print('\nStatic check passed.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
