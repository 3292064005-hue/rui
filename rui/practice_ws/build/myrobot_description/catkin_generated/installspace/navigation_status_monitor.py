#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Monitor map-based navigation topics and publish a compact JSON health status."""

from __future__ import print_function

import json
import threading

import rospy
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class NavigationStatusMonitor(object):
    def __init__(self):
        self.status_topic = rospy.get_param('~status_topic', '/navigation_health')
        self.publish_rate = float(rospy.get_param('~publish_rate', 1.0))
        self.lock = threading.Lock()
        self.map_info = None
        self.scan_stamp = None
        self.odom_stamp = None
        self.amcl_stamp = None
        self.move_base_state = None

        self.pub = rospy.Publisher(self.status_topic, String, queue_size=5, latch=True)
        rospy.Subscriber(rospy.get_param('~map_topic', '/map'), OccupancyGrid, self._map_cb, queue_size=1)
        rospy.Subscriber(rospy.get_param('~scan_topic', '/scan'), LaserScan, self._scan_cb, queue_size=1)
        rospy.Subscriber(rospy.get_param('~odom_topic', '/odom'), Odometry, self._odom_cb, queue_size=1)
        rospy.Subscriber(rospy.get_param('~amcl_pose_topic', '/amcl_pose'), PoseWithCovarianceStamped, self._amcl_cb, queue_size=1)
        rospy.Subscriber(rospy.get_param('~move_base_status_topic', '/move_base/status'), GoalStatusArray, self._move_base_cb, queue_size=1)

    def _map_cb(self, msg):
        with self.lock:
            self.map_info = {
                'width': msg.info.width,
                'height': msg.info.height,
                'resolution': msg.info.resolution,
                'stamp': rospy.Time.now().to_sec(),
            }

    def _scan_cb(self, _msg):
        with self.lock:
            self.scan_stamp = rospy.Time.now().to_sec()

    def _odom_cb(self, _msg):
        with self.lock:
            self.odom_stamp = rospy.Time.now().to_sec()

    def _amcl_cb(self, _msg):
        with self.lock:
            self.amcl_stamp = rospy.Time.now().to_sec()

    def _move_base_cb(self, msg):
        with self.lock:
            self.move_base_state = msg.status_list[-1].status if msg.status_list else None

    def run(self):
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            now = rospy.Time.now().to_sec()
            with self.lock:
                status = {
                    'stamp': now,
                    'map_received': self.map_info is not None,
                    'map_info': self.map_info,
                    'scan_age': None if self.scan_stamp is None else round(now - self.scan_stamp, 3),
                    'odom_age': None if self.odom_stamp is None else round(now - self.odom_stamp, 3),
                    'amcl_pose_age': None if self.amcl_stamp is None else round(now - self.amcl_stamp, 3),
                    'move_base_state': self.move_base_state,
                    'healthy': bool(self.map_info is not None and self.scan_stamp is not None and self.odom_stamp is not None and self.amcl_stamp is not None),
                }
            self.pub.publish(String(data=json.dumps(status, sort_keys=True)))
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('navigation_status_monitor')
    try:
        NavigationStatusMonitor().run()
    except rospy.ROSInterruptException:
        pass
