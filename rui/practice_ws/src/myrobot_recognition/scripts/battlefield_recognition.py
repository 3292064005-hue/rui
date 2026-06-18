#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Image-based recognition node for the RAICOM scouting task."""

from __future__ import print_function

import json
import math
import os
import subprocess
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import String


CARD_SIZE = (256, 340)  # width, height


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def order_points(points):
    pts = np.array(points, dtype=np.float32)
    s = pts.sum(axis=1)
    diff = np.diff(pts, axis=1)
    return np.array([
        pts[np.argmin(s)],
        pts[np.argmin(diff)],
        pts[np.argmax(s)],
        pts[np.argmax(diff)],
    ], dtype=np.float32)


def warp_quad(image, quad, size):
    width, height = size
    src = order_points(quad)
    dst = np.array([
        [0.0, 0.0],
        [width - 1.0, 0.0],
        [width - 1.0, height - 1.0],
        [0.0, height - 1.0],
    ], dtype=np.float32)
    matrix = cv2.getPerspectiveTransform(src, dst)
    return cv2.warpPerspective(image, matrix, (width, height))


class BattlefieldRecognition(object):
    def __init__(self):
        self.odom_topic = rospy.get_param('~odom_topic', '/odom')
        self.result_topic = rospy.get_param('~recognition_topic', '/recognition_result')
        self.summary_topic = rospy.get_param('~recognition_summary_topic', '/recognition_summary')
        self.status_topic = rospy.get_param('~patrol_status_topic', '/patrol_status')
        self.report_once = bool(rospy.get_param('~report_once', True))
        self.use_camera_image_detection = bool(rospy.get_param('~use_camera_image_detection', True))
        self.allow_expected_count_fallback = bool(rospy.get_param('~allow_expected_count_fallback', False))
        self.image_topic = rospy.get_param('~image_topic', '/camera/image_raw')
        self.image_timeout = float(rospy.get_param('~image_timeout', 1.5))
        self.capture_delay = float(rospy.get_param('~recognition_capture_delay', 0.45))
        self.min_card_area_px = float(rospy.get_param('~min_card_area_px', 3500.0))
        self.orb_match_threshold = int(rospy.get_param('~orb_match_threshold', 12))
        self.template_match_threshold = float(rospy.get_param('~template_match_threshold', 0.68))
        self.camera_crop = self._parse_crop(rospy.get_param('~camera_crop', [0.05, 0.95, 0.05, 0.95]))
        self.recognition_backend = str(
            rospy.get_param('~recognition_backend', 'auto')).strip().lower()
        self.weights_path = self._resolve_optional_path(
            rospy.get_param('~recognition_weights', ''))
        self.model_labels = self._parse_model_labels(
            rospy.get_param('~recognition_model_labels',
                            ['enemy', 'friendly', 'hostage']))
        model_size = rospy.get_param('~recognition_model_input_size', [224, 224])
        self.model_input_size = self._parse_model_size(model_size)
        self.model_scale = float(
            rospy.get_param('~recognition_model_scale', 1.0 / 255.0))
        self.model_mean = self._parse_model_mean(rospy.get_param(
            '~recognition_model_mean', [0.0, 0.0, 0.0]))
        self.model_swap_rb = bool(
            rospy.get_param('~recognition_model_swap_rb', True))
        self.model_confidence = float(
            rospy.get_param('~recognition_model_confidence', 0.60))
        self.yolo_iou_threshold = float(
            rospy.get_param('~recognition_yolo_iou_threshold', 0.45))
        self.yolo_class_names = self._parse_yolo_class_names(
            rospy.get_param(
                '~recognition_yolo_class_names',
                ['renzhi', 'youjun', 'dijun']))
        self.yolo_label_map = self._parse_yolo_label_map(
            rospy.get_param(
                '~recognition_yolo_label_map',
                {'renzhi': 'hostage', 'youjun': 'friendly', 'dijun': 'enemy'}))
        self.model_net = None
        self.model_session = None
        self.model_input_name = ''
        self.active_backend = 'template'

        self.zones = self._parse_zones(rospy.get_param('~zones', []))
        if not self.zones:
            raise rospy.ROSException('No valid ~zones were loaded. Check config/task_params.yaml.')

        self.pose_lock = threading.Lock()
        self.image_lock = threading.Lock()
        self.state_lock = threading.Lock()
        self.pose = None
        self.latest_frame = None
        self.latest_frame_stamp = None
        self.received_status = False
        self.reported = set()
        self.pending = set()
        self.results = []
        self.zone_by_name = {zone['name']: zone for zone in self.zones}

        log_root = rospy.get_param('~log_dir', '~/.ros/myrobot_description_logs')
        log_root = os.path.expandvars(os.path.expanduser(str(log_root)))
        self.frame_dir = os.path.join(log_root, 'recognition_frames', time.strftime('session_%Y%m%d_%H%M%S'))
        os.makedirs(self.frame_dir, exist_ok=True)

        self.bridge = CvBridge()
        self.orb = cv2.ORB_create(nfeatures=800)
        self.template_dir = self._resolve_template_dir(rospy.get_param('~template_dir', ''))
        self.templates = self._load_templates(self.template_dir)
        self._load_optional_model()

        self.result_pub = rospy.Publisher(self.result_topic, String, queue_size=10, latch=True)
        self.summary_pub = rospy.Publisher(self.summary_topic, String, queue_size=10, latch=True)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self._odom_cb, queue_size=1)
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self._image_cb, queue_size=1)
        self.status_sub = rospy.Subscriber(self.status_topic, String, self._status_cb, queue_size=10)

        rospy.loginfo(
            'battlefield_recognition: backend=%s loaded %d zones, %d templates, '
            'weights=%s image_topic=%s status_topic=%s frame_dir=%s',
            self.active_backend, len(self.zones), len(self.templates),
            self.weights_path or '<not configured>', self.image_topic,
            self.status_topic, self.frame_dir
        )

    def _resolve_optional_path(self, raw):
        if not raw:
            return ''
        value = os.path.expandvars(os.path.expanduser(str(raw)))
        path = Path(value)
        if path.is_absolute():
            return str(path)
        try:
            package_root = subprocess.check_output(
                ['rospack', 'find', 'myrobot_recognition']
            ).decode('utf-8').strip()
            return str(Path(package_root) / path)
        except Exception:
            return str(Path(__file__).resolve().parents[1] / path)

    def _parse_model_size(self, raw):
        if not isinstance(raw, (list, tuple)) or len(raw) != 2:
            return (224, 224)
        return (max(16, int(raw[0])), max(16, int(raw[1])))

    def _parse_model_labels(self, raw):
        if not isinstance(raw, (list, tuple)) or not raw:
            raise rospy.ROSException(
                'recognition_model_labels must be a non-empty list')
        labels = [str(label).strip().lower() for label in raw]
        supported = {'enemy', 'friendly', 'hostage'}
        unknown = sorted(set(labels) - supported)
        if unknown:
            raise rospy.ROSException(
                'Unsupported recognition_model_labels: %s' %
                ', '.join(unknown))
        return labels

    def _parse_yolo_class_names(self, raw):
        if not isinstance(raw, (list, tuple)) or not raw:
            raise rospy.ROSException(
                'recognition_yolo_class_names must be a non-empty list')
        names = [str(name).strip().lower() for name in raw]
        if any(not name for name in names):
            raise rospy.ROSException(
                'recognition_yolo_class_names cannot contain empty names')
        return names

    def _parse_yolo_label_map(self, raw):
        if not isinstance(raw, dict):
            raise rospy.ROSException(
                'recognition_yolo_label_map must be a dictionary')
        supported = {'enemy', 'friendly', 'hostage'}
        label_map = {
            str(name).strip().lower(): str(label).strip().lower()
            for name, label in raw.items()
        }
        unknown = sorted(set(label_map.values()) - supported)
        if unknown:
            raise rospy.ROSException(
                'Unsupported recognition_yolo_label_map values: %s' %
                ', '.join(unknown))
        missing = [
            name for name in self.yolo_class_names if name not in label_map
        ]
        if missing:
            raise rospy.ROSException(
                'recognition_yolo_label_map is missing classes: %s' %
                ', '.join(missing))
        return label_map

    def _parse_model_mean(self, raw):
        if not isinstance(raw, (list, tuple)) or len(raw) != 3:
            raise rospy.ROSException(
                'recognition_model_mean must contain three values')
        return tuple(float(value) for value in raw)

    def _load_optional_model(self):
        supported_backends = ('auto', 'template', 'onnx', 'yolo_onnx')
        if self.recognition_backend not in supported_backends:
            raise rospy.ROSException(
                'recognition_backend must be auto, template, onnx, or '
                'yolo_onnx')
        if self.recognition_backend == 'template':
            return
        if not self.weights_path or not os.path.isfile(self.weights_path):
            if self.recognition_backend in ('onnx', 'yolo_onnx'):
                raise rospy.ROSException(
                    'ONNX weights not found: %s' %
                    (self.weights_path or '<empty recognition_weights>'))
            rospy.logwarn(
                'battlefield_recognition: no ONNX weights configured; '
                'using template backend')
            return
        if Path(self.weights_path).suffix.lower() != '.onnx':
            raise rospy.ROSException(
                'Only ONNX recognition weights are supported: %s' %
                self.weights_path)
        try:
            if self.recognition_backend == 'yolo_onnx':
                ort = self._import_onnxruntime()
                options = ort.SessionOptions()
                options.intra_op_num_threads = 2
                options.graph_optimization_level = (
                    ort.GraphOptimizationLevel.ORT_ENABLE_ALL)
                self.model_session = ort.InferenceSession(
                    self.weights_path,
                    sess_options=options,
                    providers=['CPUExecutionProvider'])
                self.model_input_name = self.model_session.get_inputs()[0].name
                self.active_backend = 'yolo_onnx'
            else:
                self.model_net = cv2.dnn.readNetFromONNX(self.weights_path)
                self.active_backend = 'onnx'
        except Exception as exc:
            if self.recognition_backend in ('onnx', 'yolo_onnx'):
                raise rospy.ROSException(
                    'Failed to load ONNX weights %s: %s' %
                    (self.weights_path, exc))
            rospy.logwarn(
                'battlefield_recognition: failed to load ONNX weights %s; '
                'using template backend: %s', self.weights_path, exc)
            self.model_net = None

    def _import_onnxruntime(self):
        package_root = Path(__file__).resolve().parents[1]
        candidates = [
            package_root / 'python_vendor',
            package_root.parent.parent / 'share' / 'myrobot_recognition' /
            'python_vendor',
        ]
        try:
            rospack_root = subprocess.check_output(
                ['rospack', 'find', 'myrobot_recognition']
            ).decode('utf-8').strip()
            candidates.insert(0, Path(rospack_root) / 'python_vendor')
        except Exception:
            pass
        for candidate in candidates:
            if (candidate / 'onnxruntime' / '__init__.py').is_file():
                path = str(candidate)
                if path not in sys.path:
                    sys.path.insert(0, path)
                break
        try:
            import onnxruntime
            return onnxruntime
        except Exception as exc:
            raise rospy.ROSException(
                'Bundled ONNX Runtime could not be loaded: %s' % exc)

    def _resolve_template_dir(self, raw):
        if raw:
            candidate = Path(os.path.expandvars(os.path.expanduser(str(raw))))
            if candidate.exists():
                return str(candidate)

        here = Path(__file__).resolve()
        candidates = [
            here.parents[1] / 'recognition_templates',
            here.parents[2] / 'share' / 'myrobot_recognition' / 'recognition_templates',
            here.parents[3] / 'share' / 'myrobot_recognition' / 'recognition_templates',
        ]
        for candidate in candidates:
            if candidate.exists():
                return str(candidate)

        try:
            output = subprocess.check_output(['rospack', 'find', 'myrobot_recognition']).decode('utf-8').strip()
            candidate = Path(output) / 'recognition_templates'
            if candidate.exists():
                return str(candidate)
        except Exception:
            pass
        raise rospy.ROSException('Could not locate recognition_templates directory.')

    def _parse_crop(self, raw):
        if not isinstance(raw, (list, tuple)) or len(raw) != 4:
            return (0.05, 0.95, 0.05, 0.95)
        left, right, top, bottom = [float(v) for v in raw]
        left = min(max(left, 0.0), 0.95)
        right = min(max(right, left + 0.01), 1.0)
        top = min(max(top, 0.0), 0.95)
        bottom = min(max(bottom, top + 0.01), 1.0)
        return (left, right, top, bottom)

    def _parse_zones(self, raw):
        zones = []
        for index, item in enumerate(raw):
            if not isinstance(item, dict):
                rospy.logwarn('battlefield_recognition: skip invalid zone %r', item)
                continue
            try:
                name = str(item.get('name', 'zone_%d' % (index + 1)))
                zones.append({
                    'name': name,
                    'center_x': float(item['center_x']),
                    'center_y': float(item['center_y']),
                    'radius': float(item.get('radius', 0.35)),
                    'enemy': int(item.get('enemy', 0)),
                    'friendly': int(item.get('friendly', 0)),
                    'hostage': int(item.get('hostage', 0)),
                })
            except Exception as exc:
                rospy.logwarn('battlefield_recognition: skip zone %r: %s', item, exc)
        return zones

    def _image_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            rospy.logwarn_throttle(5.0, 'battlefield_recognition: cv_bridge failed: %s', exc)
            return
        stamp = msg.header.stamp.to_sec() if msg.header.stamp else rospy.Time.now().to_sec()
        with self.image_lock:
            self.latest_frame = frame
            self.latest_frame_stamp = stamp

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        pose = (float(p.x), float(p.y), yaw_from_quaternion(q))
        with self.pose_lock:
            self.pose = pose
        if not self.received_status:
            self._check_zones(pose, trigger='odom_zone_entry')

    def _status_cb(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        self.received_status = True
        if data.get('event') != 'waypoint_reached':
            return
        waypoint = data.get('waypoint') or data.get('goal') or {}
        zone = self.zone_by_name.get(str(waypoint.get('name', '')))
        if zone is None:
            return
        pose = self._get_pose()
        if pose is None:
            pose = (zone['center_x'], zone['center_y'], 0.0)
        distance = math.hypot(pose[0] - zone['center_x'], pose[1] - zone['center_y'])
        self._schedule_zone(zone, pose, distance, trigger='status_waypoint_reached')

    def _get_pose(self):
        with self.pose_lock:
            return self.pose

    def _check_zones(self, pose, trigger):
        x, y, _yaw = pose
        for zone in self.zones:
            dx = x - zone['center_x']
            dy = y - zone['center_y']
            distance = math.hypot(dx, dy)
            if distance > zone['radius']:
                continue
            self._schedule_zone(zone, pose, distance, trigger=trigger)

    def _schedule_zone(self, zone, pose, distance, trigger):
        with self.state_lock:
            if self.report_once and zone['name'] in self.reported:
                return
            if zone['name'] in self.pending:
                return
            self.pending.add(zone['name'])
        thread = threading.Thread(target=self._process_zone, args=(zone, pose, distance, trigger))
        thread.daemon = True
        thread.start()

    def _process_zone(self, zone, pose, distance, trigger):
        try:
            if self.capture_delay > 0.0:
                rospy.sleep(self.capture_delay)
            if self.use_camera_image_detection:
                result = self._detect_from_camera(zone, pose, distance)
            else:
                result = self._configured_result(zone, pose, distance, mode='configured_zone_trigger_legacy')
            result['trigger'] = trigger

            with self.state_lock:
                self.results.append(result)
                self.reported.add(zone['name'])
            self._publish_result(result)
            self._publish_summary(final=(len(self.reported) >= len(self.zones)))
        finally:
            with self.state_lock:
                self.pending.discard(zone['name'])

    def _get_latest_image(self):
        with self.image_lock:
            if self.latest_frame is None:
                return None, None
            return self.latest_frame.copy(), self.latest_frame_stamp

    def _wait_for_recent_image(self):
        deadline = rospy.Time.now().to_sec() + max(self.image_timeout, 0.1)
        while not rospy.is_shutdown():
            frame, stamp = self._get_latest_image()
            now = rospy.Time.now().to_sec()
            if frame is not None and stamp is not None and (now - stamp) <= self.image_timeout:
                return frame, max(0.0, now - stamp)
            if now >= deadline:
                return None, None
            rospy.sleep(0.05)
        return None, None

    def _crop_frame(self, frame):
        left, right, top, bottom = self.camera_crop
        h, w = frame.shape[:2]
        x0 = int(round(w * left))
        x1 = int(round(w * right))
        y0 = int(round(h * top))
        y1 = int(round(h * bottom))
        x0 = max(0, min(x0, w - 1))
        x1 = max(x0 + 1, min(x1, w))
        y0 = max(0, min(y0, h - 1))
        y1 = max(y0 + 1, min(y1, h))
        return frame[y0:y1, x0:x1].copy()

    def _configured_result(self, zone, pose, distance, mode):
        x, y, yaw = pose
        return {
            'zone': zone['name'],
            'enemy': zone['enemy'],
            'friendly': zone['friendly'],
            'hostage': zone['hostage'],
            'robot_pose': {'x': round(x, 3), 'y': round(y, 3), 'yaw': round(yaw, 3)},
            'distance_to_zone_center': round(distance, 3),
            'mode': mode,
            'recognition_backend': self.active_backend,
            'recognition_weights': self._active_weights_path(),
            'stamp': rospy.Time.now().to_sec(),
            'detections': [],
            'expected_counts': {
                'enemy': zone['enemy'],
                'friendly': zone['friendly'],
                'hostage': zone['hostage'],
            },
        }

    def _detect_from_camera(self, zone, pose, distance):
        frame, image_age = self._wait_for_recent_image()
        if frame is None:
            reason = 'camera image timeout on %s' % self.image_topic
            if self.allow_expected_count_fallback:
                result = self._configured_result(
                    zone, pose, distance,
                    mode='%s_image_detection_fallback' % self.active_backend)
                result['fallback_reason'] = reason
                return result
            return self._failed_result(zone, pose, distance, reason)

        cropped = self._crop_frame(frame)
        detections, annotated = self._detect_cards(cropped)
        if not detections and self.allow_expected_count_fallback:
            result = self._configured_result(
                zone, pose, distance,
                mode='%s_image_detection_fallback' % self.active_backend)
            result['fallback_reason'] = 'no cards detected'
            raw_path, ann_path = self._save_evidence(zone['name'], cropped, annotated)
            result['evidence_image'] = ann_path
            result['evidence_raw_image'] = raw_path
            result['image_age_sec'] = round(image_age, 3)
            return result

        counts = {'enemy': 0, 'friendly': 0, 'hostage': 0}
        for item in detections:
            label = item.get('label')
            if label in counts:
                counts[label] += 1

        raw_path, ann_path = self._save_evidence(zone['name'], cropped, annotated)
        x, y, yaw = pose
        result = {
            'zone': zone['name'],
            'enemy': counts['enemy'],
            'friendly': counts['friendly'],
            'hostage': counts['hostage'],
            'robot_pose': {'x': round(x, 3), 'y': round(y, 3), 'yaw': round(yaw, 3)},
            'distance_to_zone_center': round(distance, 3),
            'mode': '%s_image_detection' % self.active_backend,
            'recognition_backend': self.active_backend,
            'recognition_weights': self._active_weights_path(),
            'stamp': rospy.Time.now().to_sec(),
            'detections': detections,
            'evidence_image': ann_path,
            'evidence_raw_image': raw_path,
            'expected_counts': {
                'enemy': zone['enemy'],
                'friendly': zone['friendly'],
                'hostage': zone['hostage'],
            },
            'image_age_sec': round(image_age, 3),
        }
        return result

    def _failed_result(self, zone, pose, distance, reason):
        x, y, yaw = pose
        return {
            'zone': zone['name'],
            'enemy': 0,
            'friendly': 0,
            'hostage': 0,
            'robot_pose': {'x': round(x, 3), 'y': round(y, 3), 'yaw': round(yaw, 3)},
            'distance_to_zone_center': round(distance, 3),
            'mode': '%s_image_detection_failed' % self.active_backend,
            'recognition_backend': self.active_backend,
            'recognition_weights': self._active_weights_path(),
            'stamp': rospy.Time.now().to_sec(),
            'detections': [],
            'evidence_image': '',
            'expected_counts': {
                'enemy': zone['enemy'],
                'friendly': zone['friendly'],
                'hostage': zone['hostage'],
            },
            'error': reason,
        }

    def _save_evidence(self, zone_name, raw_image, annotated_image):
        prefix = '%s_%s' % (zone_name, time.strftime('%Y%m%d_%H%M%S'))
        raw_path = os.path.join(self.frame_dir, prefix + '_raw.jpg')
        ann_path = os.path.join(self.frame_dir, prefix + '_annotated.jpg')
        cv2.imwrite(raw_path, raw_image)
        cv2.imwrite(ann_path, annotated_image)
        return raw_path, ann_path

    def _active_weights_path(self):
        if self.active_backend in ('onnx', 'yolo_onnx'):
            return self.weights_path
        return ''

    def _detect_cards(self, frame):
        if self.active_backend == 'yolo_onnx':
            return self._detect_yolo_onnx(frame)

        quads = self._find_card_quads(frame, self.min_card_area_px)
        annotated = frame.copy()
        detections = []
        for quad in quads:
            card = warp_quad(frame, quad, CARD_SIZE)
            detection = self._classify_card(card)
            x0 = int(np.min(quad[:, 0]))
            y0 = int(np.min(quad[:, 1]))
            x1 = int(np.max(quad[:, 0]))
            y1 = int(np.max(quad[:, 1]))
            detection['bbox'] = [x0, y0, x1, y1]
            detections.append(detection)

            color = {
                'enemy': (0, 0, 255),
                'friendly': (255, 0, 0),
                'hostage': (0, 255, 0),
            }.get(detection['label'], (0, 255, 255))
            cv2.polylines(annotated, [quad.astype(np.int32)], True, color, 2)
            label_text = '%s %.2f' % (detection['label'], detection['score'])
            cv2.putText(annotated, label_text, (x0, max(24, y0 - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2, cv2.LINE_AA)
        return detections, annotated

    def _detect_yolo_onnx(self, frame):
        frame_height, frame_width = frame.shape[:2]
        regions = [(0, 0, frame_width, frame_height)]
        if frame_width > frame_height * 1.45:
            tile_width = min(
                frame_width, max(frame_height, int(round(frame_height * 1.35))))
            tile_count = max(
                2, int(math.ceil(
                    float(frame_width - tile_width) /
                    max(tile_width * 0.65, 1.0))) + 1)
            for index in range(tile_count):
                x0 = int(round(
                    index * float(frame_width - tile_width) /
                    max(tile_count - 1, 1)))
                regions.append((x0, 0, x0 + tile_width, frame_height))
        elif frame_height > frame_width * 1.45:
            tile_height = min(
                frame_height, max(frame_width, int(round(frame_width * 1.35))))
            tile_count = max(
                2, int(math.ceil(
                    float(frame_height - tile_height) /
                    max(tile_height * 0.65, 1.0))) + 1)
            for index in range(tile_count):
                y0 = int(round(
                    index * float(frame_height - tile_height) /
                    max(tile_count - 1, 1)))
                regions.append((0, y0, frame_width, y0 + tile_height))

        candidates = []
        for x0, y0, x1, y1 in regions:
            candidates.extend(self._infer_yolo_region(
                frame[y0:y1, x0:x1], x0, y0))

        # Re-run the same model on enlarged card crops. This preserves small
        # distant targets that lose detail when the whole frame is letterboxed.
        card_candidates = self._infer_yolo_cards(frame)
        for card in card_candidates:
            x, y, width, height = card['box']
            candidates = [
                item for item in candidates
                if not (
                    x <= item['box'][0] + item['box'][2] * 0.5 <= x + width and
                    y <= item['box'][1] + item['box'][3] * 0.5 <= y + height)
            ]
            candidates.append(card)

        kept = []
        for class_index in range(len(self.yolo_class_names)):
            class_candidates = [
                item for item in candidates
                if item['class_index'] == class_index
            ]
            if not class_candidates:
                continue
            indices = cv2.dnn.NMSBoxes(
                [item['box'] for item in class_candidates],
                [item['confidence'] for item in class_candidates],
                self.model_confidence,
                self.yolo_iou_threshold)
            if indices is None or len(indices) == 0:
                continue
            for index in np.asarray(indices).reshape(-1):
                kept.append(class_candidates[int(index)])

        kept.sort(key=lambda item: item['box'][0])
        annotated = frame.copy()
        detections = []
        for item in kept:
            class_index = item['class_index']
            class_name = self.yolo_class_names[class_index]
            label = self.yolo_label_map[class_name]
            x, y, width, height = item['box']
            x1 = x + width
            y1 = y + height
            confidence = item['confidence']
            detections.append({
                'label': label,
                'score': round(confidence, 3),
                'method': item.get('method', 'yolo_onnx_detector'),
                'bbox': [x, y, x1, y1],
                'model_class_index': class_index,
                'model_class_name': class_name,
            })
            color = {
                'enemy': (0, 0, 255),
                'friendly': (255, 0, 0),
                'hostage': (0, 255, 0),
            }[label]
            cv2.rectangle(annotated, (x, y), (x1, y1), color, 2)
            label_text = '%s %.2f' % (label, confidence)
            cv2.putText(
                annotated, label_text, (x, max(24, y - 8)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2, cv2.LINE_AA)
        return detections, annotated

    def _infer_yolo_cards(self, frame):
        frame_height, frame_width = frame.shape[:2]
        card_candidates = []
        for quad in self._find_card_quads(frame, self.min_card_area_px):
            card_x0 = max(0, int(math.floor(np.min(quad[:, 0]))))
            card_y0 = max(0, int(math.floor(np.min(quad[:, 1]))))
            card_x1 = min(frame_width, int(math.ceil(np.max(quad[:, 0]))))
            card_y1 = min(frame_height, int(math.ceil(np.max(quad[:, 1]))))
            card_width = card_x1 - card_x0
            card_height = card_y1 - card_y0
            if card_width <= 0 or card_height <= 0:
                continue

            margin_x = max(12, int(round(card_width * 0.45)))
            margin_y = max(16, int(round(card_height * 0.30)))
            crop_x0 = max(0, card_x0 - margin_x)
            crop_y0 = max(0, card_y0 - margin_y)
            crop_x1 = min(frame_width, card_x1 + margin_x)
            crop_y1 = min(frame_height, card_y1 + margin_y)
            local = self._infer_yolo_region(
                frame[crop_y0:crop_y1, crop_x0:crop_x1],
                crop_x0, crop_y0)
            if not local:
                continue

            best = max(local, key=lambda item: item['confidence'])
            best = dict(best)
            best['box'] = [card_x0, card_y0, card_width, card_height]
            best['method'] = 'yolo_onnx_card_crop'
            card_candidates.append(best)
        return card_candidates

    def _infer_yolo_region(self, frame, offset_x, offset_y):
        input_width, input_height = self.model_input_size
        frame_height, frame_width = frame.shape[:2]
        scale = min(
            float(input_width) / max(frame_width, 1),
            float(input_height) / max(frame_height, 1))
        resized_width = max(1, int(round(frame_width * scale)))
        resized_height = max(1, int(round(frame_height * scale)))
        resized = cv2.resize(
            frame, (resized_width, resized_height),
            interpolation=cv2.INTER_LINEAR)
        pad_left = (input_width - resized_width) // 2
        pad_right = input_width - resized_width - pad_left
        pad_top = (input_height - resized_height) // 2
        pad_bottom = input_height - resized_height - pad_top
        letterboxed = cv2.copyMakeBorder(
            resized, pad_top, pad_bottom, pad_left, pad_right,
            cv2.BORDER_CONSTANT, value=(114, 114, 114))

        try:
            blob = cv2.dnn.blobFromImage(
                letterboxed,
                scalefactor=self.model_scale,
                size=(input_width, input_height),
                mean=self.model_mean,
                swapRB=self.model_swap_rb,
                crop=False)
            if self.model_session is not None:
                output = np.asarray(self.model_session.run(
                    None, {self.model_input_name: blob})[0])
            else:
                self.model_net.setInput(blob)
                output = np.asarray(self.model_net.forward())
        except Exception as exc:
            rospy.logerr_throttle(
                5.0, 'battlefield_recognition: YOLO ONNX inference failed: %s',
                exc)
            return []

        predictions = np.squeeze(output)
        expected_columns = 4 + len(self.yolo_class_names)
        if predictions.ndim == 1:
            predictions = predictions.reshape(1, -1)
        if predictions.ndim != 2:
            rospy.logerr_throttle(
                5.0, 'battlefield_recognition: unexpected YOLO output shape %s',
                str(output.shape))
            return []
        if predictions.shape[0] == expected_columns:
            predictions = predictions.T
        if predictions.shape[1] != expected_columns:
            rospy.logerr_throttle(
                5.0, 'battlefield_recognition: YOLO output shape %s does not '
                'match %d configured classes',
                str(output.shape), len(self.yolo_class_names))
            return []

        candidates = []
        for row in predictions:
            scores = row[4:]
            class_index = int(np.argmax(scores))
            confidence = float(scores[class_index])
            if confidence < self.model_confidence:
                continue

            center_x, center_y, box_width, box_height = [
                float(value) for value in row[:4]
            ]
            x0 = int(round((center_x - box_width * 0.5 - pad_left) / scale))
            y0 = int(round((center_y - box_height * 0.5 - pad_top) / scale))
            x1 = int(round((center_x + box_width * 0.5 - pad_left) / scale))
            y1 = int(round((center_y + box_height * 0.5 - pad_top) / scale))
            x0 = max(0, min(x0, frame_width - 1))
            y0 = max(0, min(y0, frame_height - 1))
            x1 = max(0, min(x1, frame_width))
            y1 = max(0, min(y1, frame_height))
            if x1 <= x0 or y1 <= y0:
                continue
            candidates.append({
                'class_index': class_index,
                'confidence': confidence,
                'box': [
                    x0 + offset_x, y0 + offset_y,
                    x1 - x0, y1 - y0,
                ],
            })
        return candidates

    def _find_card_quads(self, frame, min_area):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (0, 0, 145), (180, 80, 255))
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        contours, _hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        candidates = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < float(min_area):
                continue
            rect = cv2.minAreaRect(cnt)
            w, h = rect[1]
            if w <= 1.0 or h <= 1.0:
                continue
            short_side = min(w, h)
            long_side = max(w, h)
            aspect = short_side / long_side
            fill_ratio = area / max(w * h, 1.0)
            if short_side < 40.0 or long_side < 70.0:
                continue
            # The zone-2 board is viewed obliquely from the narrow corridor,
            # so valid cards can project to roughly one-third width/height.
            if not (0.30 <= aspect <= 0.90):
                continue
            if fill_ratio < 0.48:
                continue
            quad = cv2.boxPoints(rect)
            candidates.append((rect[0][0], quad))

        candidates.sort(key=lambda item: item[0])
        return [quad for _center_x, quad in candidates]

    def _classify_card(self, card_image):
        if self.model_net is not None:
            model_result = self._classify_card_onnx(card_image)
            if model_result is not None:
                return model_result
        gray = self._preprocess_gray(card_image)
        query_keypoints, query_desc = self.orb.detectAndCompute(gray, None)
        orb_result = self._best_orb_match(query_desc)
        if orb_result and orb_result['score'] >= self.orb_match_threshold:
            return {
                'label': orb_result['label'],
                'score': round(float(orb_result['score']), 3),
                'method': 'orb',
                'template_name': orb_result['template_name'],
                'keypoints': 0 if query_keypoints is None else len(query_keypoints),
            }

        template_result = self._best_template_match(gray)
        if template_result and template_result['score'] >= self.template_match_threshold:
            return {
                'label': template_result['label'],
                'score': round(float(template_result['score']), 3),
                'method': 'template',
                'template_name': template_result['template_name'],
                'keypoints': 0 if query_keypoints is None else len(query_keypoints),
            }

        best_label = 'unknown'
        best_score = 0.0
        best_method = 'unknown'
        best_template = ''
        if orb_result and orb_result['score'] > best_score:
            best_label = orb_result['label']
            best_score = float(orb_result['score'])
            best_method = 'orb_low_confidence'
            best_template = orb_result['template_name']
        if template_result and template_result['score'] > best_score:
            best_label = template_result['label']
            best_score = float(template_result['score'])
            best_method = 'template_low_confidence'
            best_template = template_result['template_name']
        return {
            'label': best_label if best_method != 'unknown' else 'unknown',
            'score': round(best_score, 3),
            'method': best_method,
            'template_name': best_template,
            'keypoints': 0 if query_keypoints is None else len(query_keypoints),
        }

    def _classify_card_onnx(self, card_image):
        try:
            blob = cv2.dnn.blobFromImage(
                card_image,
                scalefactor=self.model_scale,
                size=self.model_input_size,
                mean=self.model_mean,
                swapRB=self.model_swap_rb,
                crop=False)
            self.model_net.setInput(blob)
            output = np.asarray(self.model_net.forward()).reshape(-1)
        except Exception as exc:
            rospy.logwarn_throttle(
                5.0, 'battlefield_recognition: ONNX inference failed; '
                'falling back to templates: %s', exc)
            return None
        if output.size != len(self.model_labels):
            rospy.logwarn_throttle(
                5.0, 'battlefield_recognition: ONNX output has %d values, '
                'but recognition_model_labels has %d entries',
                output.size, len(self.model_labels))
            return None
        shifted = output - np.max(output)
        probabilities = np.exp(shifted)
        probabilities /= max(float(np.sum(probabilities)), 1e-12)
        index = int(np.argmax(probabilities))
        confidence = float(probabilities[index])
        if confidence < self.model_confidence:
            return None
        return {
            'label': self.model_labels[index],
            'score': round(confidence, 3),
            'method': 'onnx_classifier',
            'template_name': '',
            'model_class_index': index,
        }

    def _preprocess_gray(self, image):
        if image.ndim == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image.copy()
        resized = cv2.resize(gray, CARD_SIZE)
        return cv2.equalizeHist(resized)

    def _best_orb_match(self, query_desc):
        if query_desc is None:
            return None
        matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        best = None
        for template in self.templates:
            desc = template.get('desc')
            if desc is None or len(desc) == 0:
                continue
            matches = matcher.match(query_desc, desc)
            if not matches:
                continue
            good_matches = [m for m in matches if m.distance < 64]
            score = float(len(good_matches))
            if best is None or score > best['score']:
                best = {
                    'label': template['label'],
                    'score': score,
                    'template_name': template['name'],
                }
        return best

    def _best_template_match(self, query_gray):
        best = None
        for template in self.templates:
            ref = template['gray']
            score = cv2.matchTemplate(query_gray, ref, cv2.TM_CCOEFF_NORMED)[0][0]
            if best is None or score > best['score']:
                best = {
                    'label': template['label'],
                    'score': float(score),
                    'template_name': template['name'],
                }
        return best

    def _extract_template_card(self, image):
        image_area = float(image.shape[0] * image.shape[1])
        quads = self._find_card_quads(image, max(2000.0, image_area * 0.02))
        if not quads:
            return cv2.resize(image, CARD_SIZE)
        areas = []
        for quad in quads:
            x0 = np.min(quad[:, 0])
            y0 = np.min(quad[:, 1])
            x1 = np.max(quad[:, 0])
            y1 = np.max(quad[:, 1])
            areas.append(((x1 - x0) * (y1 - y0), quad))
        _area, best_quad = max(areas, key=lambda item: item[0])
        return warp_quad(image, best_quad, CARD_SIZE)

    def _load_templates(self, template_dir):
        template_path = Path(template_dir)
        templates = []
        for path in sorted(template_path.glob('*.jpg')):
            image = cv2.imread(str(path))
            if image is None:
                rospy.logwarn('battlefield_recognition: could not read template %s', path)
                continue
            card = self._extract_template_card(image)
            gray = self._preprocess_gray(card)
            _kp, desc = self.orb.detectAndCompute(gray, None)
            name = path.stem
            label = name.split('_', 1)[0]
            templates.append({
                'name': name,
                'label': label,
                'gray': gray,
                'desc': desc,
            })
        if not templates:
            raise rospy.ROSException('No readable templates found in %s' % template_dir)
        return templates

    def _publish_result(self, result):
        text = json.dumps(result, ensure_ascii=False, sort_keys=True)
        self.result_pub.publish(String(data=text))
        rospy.loginfo(
            'RECOGNITION_RESULT %s: enemy=%d friendly=%d hostage=%d mode=%s',
            result['zone'], result['enemy'], result['friendly'], result['hostage'], result['mode']
        )
        rospy.loginfo(
            '识别结果：在%s区发现%d个敌军，%d个友军，%d个人质',
            self._format_zone_name(result.get('zone', 'unknown')),
            int(result.get('enemy', 0)),
            int(result.get('friendly', 0)),
            int(result.get('hostage', 0))
        )

    def _format_zone_name(self, zone_name):
        name = str(zone_name)
        if name.startswith('zone_'):
            return name.replace('zone_', 'zone', 1)
        return name

    def _select_optimal_solution(self, results):
        if not results:
            return None
        ordered = sorted(
            results,
            key=lambda item: (
                int(item.get('hostage', 0)),
                -int(item.get('enemy', 0)),
                int(item.get('friendly', 0)),
                str(item.get('zone', '')),
            ),
            reverse=True)
        best = ordered[0]
        return {
            'zone': str(best.get('zone', 'unknown')),
            'display_zone': self._format_zone_name(best.get('zone', 'unknown')),
            'enemy': int(best.get('enemy', 0)),
            'friendly': int(best.get('friendly', 0)),
            'hostage': int(best.get('hostage', 0)),
            'rule': 'hostage_desc_enemy_asc_friendly_desc',
        }

    def _publish_summary(self, final=False):
        with self.state_lock:
            snapshot = list(self.results)
            reported = sorted(list(self.reported))
        total_enemy = sum(int(item.get('enemy', 0)) for item in snapshot)
        total_friendly = sum(int(item.get('friendly', 0)) for item in snapshot)
        total_hostage = sum(int(item.get('hostage', 0)) for item in snapshot)
        optimal_solution = self._select_optimal_solution(snapshot)
        summary = {
            'recognition_backend': self.active_backend,
            'reported_zones': len(reported),
            'total_zones': len(self.zones),
            'all_zones_reported': len(reported) >= len(self.zones),
            'final': bool(final),
            'enemy_total': total_enemy,
            'friendly_total': total_friendly,
            'hostage_total': total_hostage,
            'reported_zone_names': reported,
            'optimal_solution': optimal_solution,
            'stamp': rospy.Time.now().to_sec(),
        }
        self.summary_pub.publish(String(data=json.dumps(summary, ensure_ascii=False, sort_keys=True)))
        if final:
            rospy.loginfo(
                'RECOGNITION_SUMMARY enemy_total=%d friendly_total=%d hostage_total=%d zones=%d/%d',
                total_enemy, total_friendly, total_hostage, len(reported), len(self.zones)
            )
            if optimal_solution:
                rospy.loginfo(
                    '识别汇总：共发现%d个敌军，%d个友军，%d个人质；最优解：优先处置%s区'
                    '（人质优先，其次敌军更少，其次友军更多；该区敌军%d、友军%d、人质%d）',
                    total_enemy, total_friendly, total_hostage,
                    optimal_solution['display_zone'],
                    optimal_solution['enemy'],
                    optimal_solution['friendly'],
                    optimal_solution['hostage'])

    def run(self):
        rospy.loginfo('battlefield_recognition: waiting for odometry on %s and images on %s ...',
                      self.odom_topic, self.image_topic)
        self._publish_summary(final=False)
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('battlefield_recognition')
    try:
        BattlefieldRecognition().run()
    except rospy.ROSInterruptException:
        pass
