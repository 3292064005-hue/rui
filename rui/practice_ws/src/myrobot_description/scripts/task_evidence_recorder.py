#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Mission evidence recorder and RViz task visualizer.

This node does not modify the map. It records what the robot did during the
simulation and publishes RViz overlays for waypoints and recognition zones.
Generated files are intended for report/PPT/video proof:

  ~/.ros/myrobot_description_logs/<mission_id>/trajectory.csv
  ~/.ros/myrobot_description_logs/<mission_id>/recognition_results.jsonl
  ~/.ros/myrobot_description_logs/<mission_id>/patrol_status.jsonl
  ~/.ros/myrobot_description_logs/<mission_id>/mission_summary.md
"""

from __future__ import print_function

import csv
import json
import math
import os
import threading
import time

import rospy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def parse_json_message(msg):
    try:
        return json.loads(msg.data)
    except Exception:
        return {'raw': msg.data, 'parse_error': True}


class TaskEvidenceRecorder(object):
    def __init__(self):
        self.odom_topic = rospy.get_param('~odom_topic', '/odom')
        self.status_topic = rospy.get_param('~patrol_status_topic', '/patrol_status')
        self.recognition_topic = rospy.get_param('~recognition_topic', '/recognition_result')
        self.recognition_summary_topic = rospy.get_param('~recognition_summary_topic', '/recognition_summary')
        self.marker_topic = rospy.get_param('~task_marker_topic', '/task_markers')
        self.path_topic = rospy.get_param('~trajectory_topic', '/task_trajectory')
        self.frame_id = rospy.get_param('~visualization_frame', 'odom')
        self.sample_period = float(rospy.get_param('~trajectory_sample_period', 0.20))
        self.publish_markers = bool(rospy.get_param('~publish_task_markers', True))

        log_root = rospy.get_param('~log_dir', '~/.ros/myrobot_description_logs')
        log_root = os.path.expandvars(os.path.expanduser(str(log_root)))
        self.mission_id = time.strftime('mission_%Y%m%d_%H%M%S')
        self.log_dir = os.path.join(log_root, self.mission_id)
        os.makedirs(self.log_dir, exist_ok=True)

        self.waypoints = self._parse_waypoints(rospy.get_param('~navigation_goals', rospy.get_param('~waypoints', [])))
        self.zones = self._parse_zones(rospy.get_param('~zones', []))

        self.pose_lock = threading.Lock()
        self.latest_pose = None
        self.last_sampled_pose = None
        self.distance_travelled = 0.0
        self.first_stamp = None
        self.last_stamp = None
        self.recognition_results = []
        self.patrol_events = []
        self.recognition_summary = None
        self.completed = False
        self.summary_written = False

        self.trajectory_csv = open(os.path.join(self.log_dir, 'trajectory.csv'), 'w', newline='')
        self.trajectory_writer = csv.writer(self.trajectory_csv)
        self.trajectory_writer.writerow(['stamp', 'x', 'y', 'yaw', 'distance_travelled'])
        self.recognition_file = open(os.path.join(self.log_dir, 'recognition_results.jsonl'), 'w')
        self.status_file = open(os.path.join(self.log_dir, 'patrol_status.jsonl'), 'w')

        self.path_msg = Path()
        self.path_msg.header.frame_id = self.frame_id
        self.path_pub = rospy.Publisher(self.path_topic, Path, queue_size=1, latch=True)
        self.marker_pub = rospy.Publisher(self.marker_topic, MarkerArray, queue_size=1, latch=True)

        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self._odom_cb, queue_size=10)
        self.status_sub = rospy.Subscriber(self.status_topic, String, self._status_cb, queue_size=20)
        self.recognition_sub = rospy.Subscriber(self.recognition_topic, String, self._recognition_cb, queue_size=20)
        self.summary_sub = rospy.Subscriber(self.recognition_summary_topic, String, self._summary_cb, queue_size=20)

        self.sample_timer = rospy.Timer(rospy.Duration(self.sample_period), self._sample_timer_cb)
        self.marker_timer = rospy.Timer(rospy.Duration(1.0), self._marker_timer_cb)
        rospy.on_shutdown(self._shutdown)

        rospy.loginfo('task_evidence_recorder: log directory: %s', self.log_dir)
        rospy.loginfo('task_evidence_recorder: topics odom=%s status=%s recognition=%s markers=%s path=%s',
                      self.odom_topic, self.status_topic, self.recognition_topic, self.marker_topic, self.path_topic)

    def _parse_waypoints(self, raw):
        waypoints = []
        for index, item in enumerate(raw):
            if not isinstance(item, dict):
                continue
            try:
                waypoints.append({
                    'name': str(item.get('name', 'wp_%02d' % index)),
                    'x': float(item['x']),
                    'y': float(item['y']),
                    'yaw': None if item.get('yaw', None) is None else float(item.get('yaw')),
                    'hold': float(item.get('hold', 0.0)),
                })
            except Exception:
                pass
        return waypoints

    def _parse_zones(self, raw):
        zones = []
        for index, item in enumerate(raw):
            if not isinstance(item, dict):
                continue
            try:
                zones.append({
                    'name': str(item.get('name', 'zone_%d' % (index + 1))),
                    'center_x': float(item['center_x']),
                    'center_y': float(item['center_y']),
                    'radius': float(item.get('radius', 0.35)),
                    'enemy': int(item.get('enemy', 0)),
                    'friendly': int(item.get('friendly', 0)),
                    'hostage': int(item.get('hostage', 0)),
                })
            except Exception:
                pass
        return zones

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        stamp = msg.header.stamp.to_sec() if msg.header.stamp else rospy.Time.now().to_sec()
        pose = (float(p.x), float(p.y), yaw_from_quaternion(q), stamp)
        with self.pose_lock:
            self.latest_pose = pose

    def _get_pose(self):
        with self.pose_lock:
            return self.latest_pose

    def _sample_timer_cb(self, _event):
        pose = self._get_pose()
        if pose is None:
            return
        x, y, yaw, stamp = pose
        if self.first_stamp is None:
            self.first_stamp = stamp
        self.last_stamp = stamp
        if self.last_sampled_pose is not None:
            lx, ly, _lyaw, _lstamp = self.last_sampled_pose
            self.distance_travelled += math.hypot(x - lx, y - ly)
        self.last_sampled_pose = pose

        self.trajectory_writer.writerow(['%.3f' % stamp, '%.4f' % x, '%.4f' % y, '%.4f' % yaw, '%.4f' % self.distance_travelled])
        self.trajectory_csv.flush()

        ps = PoseStamped()
        ps.header.frame_id = self.frame_id
        ps.header.stamp = rospy.Time.now()
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.position.z = 0.05
        ps.pose.orientation.w = 1.0
        self.path_msg.header.stamp = ps.header.stamp
        self.path_msg.poses.append(ps)
        self.path_pub.publish(self.path_msg)

    def _status_cb(self, msg):
        data = parse_json_message(msg)
        self.patrol_events.append(data)
        self.status_file.write(json.dumps(data, ensure_ascii=False, sort_keys=True) + '\n')
        self.status_file.flush()
        if data.get('event') == 'patrol_completed':
            self.completed = True
            self._write_summary()

    def _recognition_cb(self, msg):
        data = parse_json_message(msg)
        self.recognition_results.append(data)
        self.recognition_file.write(json.dumps(data, ensure_ascii=False, sort_keys=True) + '\n')
        self.recognition_file.flush()

    def _summary_cb(self, msg):
        self.recognition_summary = parse_json_message(msg)

    def _marker_timer_cb(self, _event):
        if self.publish_markers:
            self._publish_markers()

    def _publish_markers(self):
        arr = MarkerArray()
        now = rospy.Time.now()
        marker_id = 0

        # Waypoint line strip.
        if self.waypoints:
            line = Marker()
            line.header.frame_id = self.frame_id
            line.header.stamp = now
            line.ns = 'task_waypoint_path'
            line.id = marker_id
            marker_id += 1
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.pose.orientation.w = 1.0
            line.scale.x = 0.035
            line.color.r = 0.1
            line.color.g = 0.7
            line.color.b = 1.0
            line.color.a = 1.0
            for wp in self.waypoints:
                pt = Point()
                pt.x = wp['x']
                pt.y = wp['y']
                pt.z = 0.10
                line.points.append(pt)
            arr.markers.append(line)

        for index, wp in enumerate(self.waypoints):
            sphere = Marker()
            sphere.header.frame_id = self.frame_id
            sphere.header.stamp = now
            sphere.ns = 'task_waypoints'
            sphere.id = marker_id
            marker_id += 1
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = wp['x']
            sphere.pose.position.y = wp['y']
            sphere.pose.position.z = 0.12
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = 0.12
            sphere.scale.y = 0.12
            sphere.scale.z = 0.12
            sphere.color.r = 0.0
            sphere.color.g = 0.85
            sphere.color.b = 0.25
            sphere.color.a = 0.9
            arr.markers.append(sphere)

            text = Marker()
            text.header.frame_id = self.frame_id
            text.header.stamp = now
            text.ns = 'task_waypoint_labels'
            text.id = marker_id
            marker_id += 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = wp['x']
            text.pose.position.y = wp['y']
            text.pose.position.z = 0.35
            text.pose.orientation.w = 1.0
            text.scale.z = 0.16
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.color.a = 1.0
            text.text = '%02d %s' % (index + 1, wp['name'])
            arr.markers.append(text)

        for zone in self.zones:
            cyl = Marker()
            cyl.header.frame_id = self.frame_id
            cyl.header.stamp = now
            cyl.ns = 'task_recognition_zones'
            cyl.id = marker_id
            marker_id += 1
            cyl.type = Marker.CYLINDER
            cyl.action = Marker.ADD
            cyl.pose.position.x = zone['center_x']
            cyl.pose.position.y = zone['center_y']
            cyl.pose.position.z = 0.03
            cyl.pose.orientation.w = 1.0
            cyl.scale.x = 2.0 * zone['radius']
            cyl.scale.y = 2.0 * zone['radius']
            cyl.scale.z = 0.04
            cyl.color.r = 1.0
            cyl.color.g = 0.3
            cyl.color.b = 0.05
            cyl.color.a = 0.35
            arr.markers.append(cyl)

            label = Marker()
            label.header.frame_id = self.frame_id
            label.header.stamp = now
            label.ns = 'task_recognition_labels'
            label.id = marker_id
            marker_id += 1
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = zone['center_x']
            label.pose.position.y = zone['center_y']
            label.pose.position.z = 0.45
            label.pose.orientation.w = 1.0
            label.scale.z = 0.18
            label.color.r = 1.0
            label.color.g = 0.85
            label.color.b = 0.1
            label.color.a = 1.0
            label.text = '%s E:%d F:%d H:%d' % (zone['name'], zone['enemy'], zone['friendly'], zone['hostage'])
            arr.markers.append(label)

        self.marker_pub.publish(arr)

    def _write_summary(self):
        if self.summary_written:
            return
        self.summary_written = True
        duration = None
        if self.first_stamp is not None and self.last_stamp is not None:
            duration = max(0.0, self.last_stamp - self.first_stamp)

        total_enemy = 0
        total_friendly = 0
        total_hostage = 0
        zone_names = []
        for item in self.recognition_results:
            total_enemy += int(item.get('enemy', 0))
            total_friendly += int(item.get('friendly', 0))
            total_hostage += int(item.get('hostage', 0))
            if 'zone' in item:
                zone_names.append(str(item['zone']))

        summary_path = os.path.join(self.log_dir, 'mission_summary.md')
        with open(summary_path, 'w') as f:
            f.write('# Mission Summary\n\n')
            f.write('- Completed: `%s`\n' % str(self.completed))
            f.write('- Log directory: `%s`\n' % self.log_dir)
            if duration is not None:
                f.write('- Duration: `%.2f s`\n' % duration)
            f.write('- Distance travelled: `%.3f m`\n' % self.distance_travelled)
            f.write('- Waypoints configured: `%d`\n' % len(self.waypoints))
            f.write('- Recognition zones configured: `%d`\n' % len(self.zones))
            f.write('- Recognition zones reported: `%d`\n' % len(self.recognition_results))
            f.write('- Reported zone names: `%s`\n' % ', '.join(zone_names))
            f.write('- Enemy total: `%d`\n' % total_enemy)
            f.write('- Friendly total: `%d`\n' % total_friendly)
            f.write('- Hostage total: `%d`\n\n' % total_hostage)
            f.write('## Generated Evidence Files\n\n')
            f.write('- `trajectory.csv`: robot path sampled from odometry.\n')
            f.write('- `recognition_results.jsonl`: per-zone recognition output.\n')
            f.write('- `patrol_status.jsonl`: waypoint start/reached/completion events.\n')
        rospy.loginfo('task_evidence_recorder: wrote summary: %s', summary_path)

    def _shutdown(self):
        self._write_summary()
        for handle in [self.trajectory_csv, self.recognition_file, self.status_file]:
            try:
                handle.close()
            except Exception:
                pass

    def run(self):
        self._publish_markers()
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('task_evidence_recorder')
    try:
        TaskEvidenceRecorder().run()
    except rospy.ROSInterruptException:
        pass
