#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Offline static checks for the split myrobot workspace.

Run after unzipping, before catkin_make if you want a quick sanity check:

  python3 src/myrobot_task/scripts/static_workspace_check.py
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

import yaml


PACKAGE_NAMES = [
    'myrobot_description',
    'myrobot_simulation',
    'myrobot_navigation',
    'myrobot_recognition',
    'myrobot_task',
]


def sha256(path):
    h = hashlib.sha256()
    with open(str(path), 'rb') as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


def check_xml(path, errors):
    try:
        ET.parse(str(path))
        print('[OK] XML parse:', path.relative_to(workspace_src()))
    except Exception as exc:
        errors.append('%s XML parse failed: %s' % (path, exc))


def workspace_src():
    return Path(__file__).resolve().parents[2]


def package_root(name):
    return workspace_src() / name


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


def navigation_goal_names(path):
    text = path.read_text(errors='ignore')
    names = []
    in_goals = False
    for line in text.splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith('#'):
            continue
        if stripped == 'navigation_goals:':
            in_goals = True
            continue
        if not in_goals:
            continue
        if stripped.startswith('- '):
            match = re.search(r'name\s*:\s*([^,\}\s]+)', stripped)
            names.append(match.group(1) if match else 'goal_%02d' % len(names))
        elif not line.startswith((' ', '\t')):
            break
    return names


def require_files(paths, errors):
    root = workspace_src()
    for path in paths:
        if path.exists():
            print('[OK] exists:', path.relative_to(root))
        else:
            errors.append('missing file: %s' % path)


def check_executable_scripts(errors):
    root = workspace_src()
    for package in PACKAGE_NAMES:
        script_dir = package_root(package) / 'scripts'
        if not script_dir.exists():
            continue
        for path in sorted(script_dir.glob('*.py')):
            if path.stat().st_mode & stat.S_IXUSR:
                print('[OK] executable:', path.relative_to(root))
            else:
                errors.append('script is not executable: %s' % path)


def check_ros_packages(errors):
    rospack = None
    for candidate in ('rospack', '/opt/ros/noetic/bin/rospack'):
        if os.path.exists(candidate) or os.system('command -v %s >/dev/null 2>&1' % candidate) == 0:
            rospack = candidate
            break
    if not rospack:
        print('[WARN] rospack not found; skipping runtime ROS package checks')
        return

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
        'slam_toolbox',
        'navfn',
        'global_planner',
        'base_local_planner',
        'teb_local_planner',
        'costmap_2d',
        'clear_costmap_recovery',
        'rotate_recovery',
        'gmapping',
    ]
    for package_name in required_ros_packages:
        result = subprocess.run(
            [rospack, 'find', package_name],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE)
        if result.returncode == 0:
            print('[OK] ros package:', package_name)
        else:
            errors.append('missing ROS package: %s' % package_name)


def check_description(errors):
    desc = package_root('myrobot_description')
    for path in [desc / 'package.xml'] + sorted((desc / 'urdf').glob('*.xacro')):
        check_xml(path, errors)

    mesh_refs = set()
    for path in sorted((desc / 'urdf').glob('*.xacro')):
        text = path.read_text(errors='ignore')
        for rel in re.findall(r'package://myrobot_description/([^"\']+)', text):
            mesh_refs.add(rel)
    for rel in sorted(mesh_refs):
        target = desc / rel
        if target.exists():
            print('[OK] mesh reference:', rel)
        else:
            errors.append('missing package resource: %s' % rel)

    expected_path = desc / 'config' / 'rm_map.sha256'
    world_path = desc / 'worlds' / 'rm_map.world'
    if expected_path.exists() and world_path.exists():
        expected = expected_path.read_text().strip().split()[0]
        actual = sha256(world_path)
        if actual == expected:
            print('[OK] rm_map.world SHA-256 unchanged:', actual)
        else:
            errors.append('rm_map.world hash changed: expected %s actual %s' % (expected, actual))


def check_recognition(errors):
    recognition = package_root('myrobot_recognition')
    templates = sorted((recognition / 'recognition_templates').glob('*.jpg'))
    if len(templates) >= 5:
        print('[OK] recognition templates:', len(templates))
    else:
        errors.append('recognition_templates has too few images: found %d' % len(templates))

    yolo_weights = recognition / 'recognition_weights' / 'best.onnx'
    if yolo_weights.exists() and yolo_weights.stat().st_size > 1024 * 1024:
        print('[OK] YOLO recognition weights:', yolo_weights.relative_to(workspace_src()))
    else:
        errors.append('missing or invalid YOLO recognition weights: %s' % yolo_weights)

    ort_binding = (
        recognition / 'python_vendor' / 'onnxruntime' / 'capi' /
        'onnxruntime_pybind11_state.cpython-38-x86_64-linux-gnu.so')
    if ort_binding.exists() and ort_binding.stat().st_size > 1024 * 1024:
        print('[OK] bundled ONNX Runtime:', ort_binding.relative_to(workspace_src()))
    else:
        errors.append('missing bundled ONNX Runtime Python 3.8 binding')

    task_params = package_root('myrobot_task') / 'config' / 'task_params.yaml'
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


def check_navigation(errors):
    nav = package_root('myrobot_navigation')
    map_yaml = nav / 'maps' / 'raicom_slam_map_01.yaml'
    map_pgm = nav / 'maps' / 'raicom_slam_map_01.pgm'
    if map_yaml.exists() and map_pgm.exists():
        yaml_text = map_yaml.read_text(errors='ignore')
        if 'image: raicom_slam_map_01.pgm' in yaml_text and 'resolution:' in yaml_text and 'origin:' in yaml_text:
            print('[OK] navigation map yaml:', map_yaml.relative_to(workspace_src()))
        else:
            errors.append('navigation map yaml is missing image/resolution/origin fields')
        try:
            magic, width, height, pixels = read_pgm(map_pgm)
            print('[OK] navigation map pgm: %dx%d %s' % (width, height, magic))
            occupied = [index for index, value in enumerate(pixels) if value < 65]
            if len(occupied) >= 3000:
                print('[OK] final map occupied cells:', len(occupied))
            else:
                errors.append('final map has too few occupied cells: %d < 3000' % len(occupied))
        except Exception as exc:
            errors.append('navigation map pgm cannot be read: %s' % exc)

    expected_map_default = 'maps/raicom_slam_map_01.yaml'
    launch_checks = [
        nav / 'launch' / 'navigation.launch',
        nav / 'launch' / 'autonomous_navigation.launch',
        package_root('myrobot_task') / 'launch' / 'task_patrol.launch',
    ]
    for launch_path in launch_checks:
        text = launch_path.read_text(errors='ignore')
        if expected_map_default in text:
            print('[OK] default navigation map:', launch_path.relative_to(workspace_src()))
        else:
            errors.append('%s does not default to %s' % (launch_path, expected_map_default))

    slam_launch_text = (nav / 'launch' / 'slam_mapping.launch').read_text(errors='ignore')
    if '<arg name="patrol_repeats" default="1" />' in slam_launch_text:
        print('[OK] SLAM patrol repeats: 1')
    else:
        errors.append('slam_mapping.launch does not default to 1 patrol repeats')
    if 'slam_navigation_params.yaml' in slam_launch_text:
        print('[OK] SLAM uses dedicated mapping route')
    else:
        errors.append('slam_mapping.launch does not default to slam_navigation_params.yaml')
    if '<arg name="mapping_backend" default="odom_laser" />' in slam_launch_text:
        print('[OK] default SLAM backend: odom_laser')
    else:
        errors.append('slam_mapping.launch does not default to odom_laser')

    formal_goals = navigation_goal_names(nav / 'config' / 'navigation_params.yaml')
    expected_formal_goals = ['zone_1', 'right_wall_exit', 'zone_2', 'finish']
    if formal_goals == expected_formal_goals:
        print('[OK] formal navigation route:', ', '.join(formal_goals))
    else:
        errors.append('navigation_params.yaml route must visit %s; got %s' %
                      (', '.join(expected_formal_goals), ', '.join(formal_goals)))

    navigation_text = (nav / 'config' / 'navigation_params.yaml').read_text(errors='ignore')
    for phrase, label in [
            ('navigation_use_holonomic_path_follower: false', 'formal navigation uses TEB'),
            ('navigation_use_internal_route: true', 'formal navigation uses internal route'),
            ('navigation_separate_rotation: false', 'formal navigation avoids separate yaw goals'),
            ('navigation_cmd_vel_target_yaw_filter: false', 'formal navigation leaves angular velocity to TEB'),
            ('navigation_path_velocity_filter: false', 'formal navigation leaves translational velocity to TEB')]:
        if phrase in navigation_text:
            print('[OK]', label)
        else:
            errors.append('missing navigation setting: %s' % phrase)

    move_base_text = (nav / 'config' / 'move_base_params.yaml').read_text(errors='ignore')
    local_planner_text = (nav / 'config' / 'base_local_planner_params.yaml').read_text(errors='ignore')
    if 'base_local_planner: teb_local_planner/TebLocalPlannerROS' in move_base_text:
        print('[OK] formal local planner: TebLocalPlannerROS')
    else:
        errors.append('move_base_params.yaml should use TebLocalPlannerROS')
    if 'base_global_planner: global_planner/GlobalPlanner' in move_base_text:
        print('[OK] formal global planner: GlobalPlanner')
    else:
        errors.append('move_base_params.yaml should use global_planner/GlobalPlanner')

    try:
        teb_params = yaml.safe_load(local_planner_text).get('TebLocalPlannerROS', {})
    except Exception:
        teb_params = {}
    if float(teb_params.get('max_vel_theta', 0.0)) > 0.0:
        print('[OK] TEB angular velocity enabled')
    else:
        errors.append('TebLocalPlannerROS leaves max_vel_theta disabled')
    if float(teb_params.get('max_vel_y', 0.0)) > 0.0:
        print('[OK] TEB holonomic strafing enabled')
    else:
        errors.append('TebLocalPlannerROS should keep max_vel_y > 0')

    slam_goals = navigation_goal_names(nav / 'config' / 'slam_navigation_params.yaml')
    if len(slam_goals) == 12 and 'zone_1' in slam_goals and 'zone_2' in slam_goals:
        print('[OK] SLAM mapping route: 12 goals')
    else:
        errors.append('slam_navigation_params.yaml route must keep 12 mapping goals; got %d' % len(slam_goals))


def main():
    errors = []
    root = workspace_src()

    required = []
    for package in PACKAGE_NAMES:
        pkg = package_root(package)
        required.extend([pkg / 'package.xml', pkg / 'CMakeLists.txt'])
    required.extend([
        package_root('myrobot_description') / 'urdf' / 'turtlebot3_mecanum.urdf.xacro',
        package_root('myrobot_description') / 'urdf' / 'camera.xacro',
        package_root('myrobot_description') / 'worlds' / 'rm_map.world',
        package_root('myrobot_description') / 'materials' / 'scripts' / 'raicom_targets.material',
        package_root('myrobot_simulation') / 'launch' / 'mycar_gazebo.launch',
        package_root('myrobot_simulation') / 'config' / 'simulation_params.yaml',
        package_root('myrobot_navigation') / 'launch' / 'navigation.launch',
        package_root('myrobot_navigation') / 'launch' / 'autonomous_navigation.launch',
        package_root('myrobot_navigation') / 'launch' / 'slam_mapping.launch',
        package_root('myrobot_navigation') / 'launch' / 'save_slam_map.launch',
        package_root('myrobot_navigation') / 'config' / 'navigation_params.yaml',
        package_root('myrobot_navigation') / 'config' / 'base_local_planner_params.yaml',
        package_root('myrobot_navigation') / 'maps' / 'raicom_slam_map_01.yaml',
        package_root('myrobot_recognition') / 'scripts' / 'battlefield_recognition.py',
        package_root('myrobot_task') / 'launch' / 'task_patrol.launch',
        package_root('myrobot_task') / 'config' / 'task_params.yaml',
    ])
    require_files(required, errors)

    for package in PACKAGE_NAMES:
        pkg = package_root(package)
        for path in [pkg / 'package.xml'] + sorted((pkg / 'launch').glob('*.launch')):
            if path.exists():
                check_xml(path, errors)

    check_ros_packages(errors)
    check_executable_scripts(errors)
    check_description(errors)
    check_recognition(errors)
    check_navigation(errors)

    if errors:
        print('\nStatic check failed:')
        for item in errors:
            print('  -', item)
        return 1

    print('\nStatic check passed for split workspace:', root)
    return 0


if __name__ == '__main__':
    sys.exit(main())
