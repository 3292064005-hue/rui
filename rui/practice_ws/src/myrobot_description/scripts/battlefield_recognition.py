#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Image-based recognition node for the RAICOM scouting task."""

from __future__ import print_function

import json
import math
import os
import subprocess
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

        self.result_pub = rospy.Publisher(self.result_topic, String, queue_size=10, latch=True)
        self.summary_pub = rospy.Publisher(self.summary_topic, String, queue_size=10, latch=True)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self._odom_cb, queue_size=1)
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self._image_cb, queue_size=1)
        self.status_sub = rospy.Subscriber(self.status_topic, String, self._status_cb, queue_size=10)

        rospy.loginfo(
            'battlefield_recognition: loaded %d zones, %d templates, image_topic=%s status_topic=%s frame_dir=%s',
            len(self.zones), len(self.templates), self.image_topic, self.status_topic, self.frame_dir
        )

    def _resolve_template_dir(self, raw):
        if raw:
            candidate = Path(os.path.expandvars(os.path.expanduser(str(raw))))
            if candidate.exists():
                return str(candidate)

        here = Path(__file__).resolve()
        candidates = [
            here.parents[1] / 'recognition_templates',
            here.parents[2] / 'share' / 'myrobot_description' / 'recognition_templates',
            here.parents[3] / 'share' / 'myrobot_description' / 'recognition_templates',
        ]
        for candidate in candidates:
            if candidate.exists():
                return str(candidate)

        try:
            output = subprocess.check_output(['rospack', 'find', 'myrobot_description']).decode('utf-8').strip()
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
        waypoint = data.get('waypoint') or {}
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
                result = self._configured_result(zone, pose, distance, mode='template_image_detection_fallback')
                result['fallback_reason'] = reason
                return result
            return self._failed_result(zone, pose, distance, reason)

        cropped = self._crop_frame(frame)
        detections, annotated = self._detect_cards(cropped)
        if not detections and self.allow_expected_count_fallback:
            result = self._configured_result(zone, pose, distance, mode='template_image_detection_fallback')
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
            'mode': 'template_image_detection',
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
            'mode': 'template_image_detection_failed',
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

    def _detect_cards(self, frame):
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
            if not (0.45 <= aspect <= 0.90):
                continue
            if fill_ratio < 0.55:
                continue
            quad = cv2.boxPoints(rect)
            candidates.append((rect[0][0], quad))

        candidates.sort(key=lambda item: item[0])
        return [quad for _center_x, quad in candidates]

    def _classify_card(self, card_image):
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

    def _publish_summary(self, final=False):
        with self.state_lock:
            snapshot = list(self.results)
            reported = sorted(list(self.reported))
        total_enemy = sum(int(item.get('enemy', 0)) for item in snapshot)
        total_friendly = sum(int(item.get('friendly', 0)) for item in snapshot)
        total_hostage = sum(int(item.get('hostage', 0)) for item in snapshot)
        summary = {
            'reported_zones': len(reported),
            'total_zones': len(self.zones),
            'all_zones_reported': len(reported) >= len(self.zones),
            'final': bool(final),
            'enemy_total': total_enemy,
            'friendly_total': total_friendly,
            'hostage_total': total_hostage,
            'reported_zone_names': reported,
            'stamp': rospy.Time.now().to_sec(),
        }
        self.summary_pub.publish(String(data=json.dumps(summary, ensure_ascii=False, sort_keys=True)))
        if final:
            rospy.loginfo(
                'RECOGNITION_SUMMARY enemy_total=%d friendly_total=%d hostage_total=%d zones=%d/%d',
                total_enemy, total_friendly, total_hostage, len(reported), len(self.zones)
            )

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
