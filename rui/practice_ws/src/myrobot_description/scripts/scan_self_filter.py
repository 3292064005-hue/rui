#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Filter near-field laser returns that come from the robot itself.

The simulated lidar is mounted close to the chassis, so raw /scan frequently
contains minimum-range hits from the robot body or wheels. Those self-returns
confuse gmapping and move_base costmaps. This node republishes a filtered scan
where ranges below a configurable threshold are replaced with +inf.
"""

from __future__ import print_function

import math

import rospy
from sensor_msgs.msg import LaserScan


class ScanSelfFilter(object):
    def __init__(self):
        self.input_topic = rospy.get_param('~input_scan_topic', '/scan')
        self.output_topic = rospy.get_param('~output_scan_topic', '/scan_filtered')
        self.min_valid_range = float(rospy.get_param('~min_valid_range', 0.18))
        self.keep_original_min = bool(rospy.get_param('~keep_original_min', True))

        self.pub = rospy.Publisher(self.output_topic, LaserScan, queue_size=10)
        self.sub = rospy.Subscriber(self.input_topic, LaserScan, self._scan_cb, queue_size=10)
        rospy.loginfo('scan_self_filter: %s -> %s, min_valid_range=%.3f',
                      self.input_topic, self.output_topic, self.min_valid_range)

    def _scan_cb(self, msg):
        filtered = LaserScan()
        filtered.header = msg.header
        filtered.angle_min = msg.angle_min
        filtered.angle_max = msg.angle_max
        filtered.angle_increment = msg.angle_increment
        filtered.time_increment = msg.time_increment
        filtered.scan_time = msg.scan_time
        filtered.range_min = msg.range_min if self.keep_original_min else max(msg.range_min, self.min_valid_range)
        filtered.range_max = msg.range_max
        filtered.intensities = list(msg.intensities)

        ranges = []
        threshold = max(msg.range_min, self.min_valid_range)
        for value in msg.ranges:
            if math.isfinite(value) and value < threshold:
                ranges.append(float('inf'))
            else:
                ranges.append(value)
        filtered.ranges = ranges
        self.pub.publish(filtered)


def main():
    rospy.init_node('scan_self_filter')
    ScanSelfFilter()
    rospy.spin()


if __name__ == '__main__':
    main()
