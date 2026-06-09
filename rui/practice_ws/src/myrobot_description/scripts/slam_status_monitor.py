#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Publish simple SLAM health/status JSON for demonstration and debugging.

Subscribes to /map and /scan, then publishes /slam_status. This gives visible
proof that gmapping is receiving laser data and producing an occupancy grid.
"""

from __future__ import print_function

import json
import math
import threading

import rospy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class SlamStatusMonitor(object):
    def __init__(self):
        self.map_topic = rospy.get_param('~map_topic', '/map')
        self.scan_topic = rospy.get_param('~scan_topic', '/scan')
        self.status_topic = rospy.get_param('~status_topic', '/slam_status')
        self.publish_rate = float(rospy.get_param('~publish_rate', 1.0))

        self.lock = threading.Lock()
        self.latest_map = None
        self.latest_scan = None

        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=10, latch=True)
        self.map_sub = rospy.Subscriber(self.map_topic, OccupancyGrid, self._map_cb, queue_size=1)
        self.scan_sub = rospy.Subscriber(self.scan_topic, LaserScan, self._scan_cb, queue_size=1)

        rospy.loginfo('slam_status_monitor: map=%s scan=%s status=%s', self.map_topic, self.scan_topic, self.status_topic)

    def _map_cb(self, msg):
        with self.lock:
            self.latest_map = msg

    def _scan_cb(self, msg):
        with self.lock:
            self.latest_scan = msg

    def _build_status(self):
        with self.lock:
            map_msg = self.latest_map
            scan_msg = self.latest_scan

        payload = {
            'stamp': rospy.Time.now().to_sec(),
            'map_received': map_msg is not None,
            'scan_received': scan_msg is not None,
        }

        if map_msg is not None:
            data = list(map_msg.data)
            total = len(data)
            unknown = sum(1 for v in data if v < 0)
            occupied = sum(1 for v in data if v >= 65)
            free = sum(1 for v in data if 0 <= v <= 25)
            known = total - unknown
            payload['map'] = {
                'frame_id': map_msg.header.frame_id,
                'width': int(map_msg.info.width),
                'height': int(map_msg.info.height),
                'resolution': round(float(map_msg.info.resolution), 4),
                'total_cells': int(total),
                'known_cells': int(known),
                'unknown_cells': int(unknown),
                'free_cells': int(free),
                'occupied_cells': int(occupied),
                'known_ratio': round(float(known) / float(total), 4) if total else 0.0,
            }

        if scan_msg is not None:
            finite = [r for r in scan_msg.ranges if math.isfinite(r)]
            payload['scan'] = {
                'frame_id': scan_msg.header.frame_id,
                'range_count': len(scan_msg.ranges),
                'finite_count': len(finite),
                'range_min': round(min(finite), 3) if finite else None,
                'range_max': round(max(finite), 3) if finite else None,
            }

        return payload

    def run(self):
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            payload = self._build_status()
            self.status_pub.publish(String(data=json.dumps(payload, ensure_ascii=False, sort_keys=True)))
            if payload.get('map_received') and payload.get('scan_received'):
                m = payload.get('map', {})
                rospy.loginfo_throttle(5.0, 'slam_status_monitor: map %sx%s res=%.3f known=%.1f%%',
                                       m.get('width', 0), m.get('height', 0), m.get('resolution', 0.0),
                                       100.0 * float(m.get('known_ratio', 0.0)))
            else:
                rospy.logwarn_throttle(5.0, 'slam_status_monitor: waiting for map/scan: map=%s scan=%s',
                                       payload.get('map_received'), payload.get('scan_received'))
            rate.sleep()


def main():
    rospy.init_node('slam_status_monitor')
    SlamStatusMonitor().run()


if __name__ == '__main__':
    main()
