#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Build a fixed-size occupancy grid from simulated odometry and laser scans."""

from __future__ import print_function

import math

import cv2
import numpy as np
import rospy
import tf
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan


def bresenham(x0, y0, x1, y1):
    points = []
    dx = abs(x1 - x0)
    dy = -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    error = dx + dy
    while True:
        points.append((x0, y0))
        if x0 == x1 and y0 == y1:
            return points
        twice = 2 * error
        if twice >= dy:
            error += dy
            x0 += sx
        if twice <= dx:
            error += dx
            y0 += sy


class OdomLaserMapper(object):
    def __init__(self):
        self.scan_topic = rospy.get_param('~scan_topic', '/scan_filtered')
        self.map_topic = rospy.get_param('~map_topic', '/map')
        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.odom_frame = rospy.get_param('~odom_frame', 'odom')
        self.resolution = float(rospy.get_param('~resolution', 0.02))
        self.xmin = float(rospy.get_param('~xmin', -0.50))
        self.ymin = float(rospy.get_param('~ymin', -4.00))
        self.xmax = float(rospy.get_param('~xmax', 5.00))
        self.ymax = float(rospy.get_param('~ymax', 0.50))
        self.max_usable_range = float(rospy.get_param('~max_usable_range', 6.0))
        self.hit_increment = float(rospy.get_param('~hit_increment', 2.0))
        self.free_increment = float(rospy.get_param('~free_increment', 0.45))
        self.occupied_threshold = float(rospy.get_param('~occupied_threshold', 1.0))
        self.min_component_cells = int(rospy.get_param('~min_component_cells', 8))
        self.free_threshold = float(rospy.get_param('~free_threshold', -0.4))
        self.publish_rate = float(rospy.get_param('~publish_rate', 2.0))

        self.width = int(math.ceil((self.xmax - self.xmin) / self.resolution))
        self.height = int(math.ceil((self.ymax - self.ymin) / self.resolution))
        self.log_odds = np.zeros((self.height, self.width), dtype=np.float32)
        self.observed = np.zeros((self.height, self.width), dtype=np.bool_)
        self.listener = tf.TransformListener()
        self.broadcaster = tf.TransformBroadcaster()
        self.map_pub = rospy.Publisher(self.map_topic, OccupancyGrid, queue_size=1, latch=True)
        self.scan_sub = rospy.Subscriber(self.scan_topic, LaserScan, self._scan_cb, queue_size=1)
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / max(0.2, self.publish_rate)), self._publish_map)
        rospy.loginfo(
            'odom_laser_mapper: %s -> %s, bounds=[%.2f, %.2f] x [%.2f, %.2f], resolution=%.3f',
            self.scan_topic, self.map_topic, self.xmin, self.xmax,
            self.ymin, self.ymax, self.resolution)

    def _cell(self, x, y):
        col = int(math.floor((x - self.xmin) / self.resolution))
        row = int(math.floor((y - self.ymin) / self.resolution))
        if 0 <= col < self.width and 0 <= row < self.height:
            return col, row
        return None

    def _scan_cb(self, msg):
        try:
            translation, rotation = self.listener.lookupTransform(
                self.odom_frame, msg.header.frame_id, msg.header.stamp)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return

        yaw = tf.transformations.euler_from_quaternion(rotation)[2]
        start = self._cell(translation[0], translation[1])
        if start is None:
            return

        angle = msg.angle_min
        max_range = min(msg.range_max, self.max_usable_range)
        for distance in msg.ranges:
            if not math.isfinite(distance) or distance < msg.range_min or distance > max_range:
                angle += msg.angle_increment
                continue
            world_angle = yaw + angle
            end = self._cell(
                translation[0] + distance * math.cos(world_angle),
                translation[1] + distance * math.sin(world_angle))
            angle += msg.angle_increment
            if end is None:
                continue
            ray = bresenham(start[0], start[1], end[0], end[1])
            for col, row in ray[:-1]:
                self.observed[row, col] = True
                self.log_odds[row, col] = max(
                    -5.0, self.log_odds[row, col] - self.free_increment)
            col, row = ray[-1]
            self.observed[row, col] = True
            self.log_odds[row, col] = min(
                5.0, self.log_odds[row, col] + self.hit_increment)

    def _publish_map(self, _event):
        now = rospy.Time.now()
        self.broadcaster.sendTransform(
            (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0),
            now, self.odom_frame, self.map_frame)
        grid = OccupancyGrid()
        grid.header.stamp = now
        grid.header.frame_id = self.map_frame
        grid.info.map_load_time = now
        grid.info.resolution = self.resolution
        grid.info.width = self.width
        grid.info.height = self.height
        grid.info.origin.position.x = self.xmin
        grid.info.origin.position.y = self.ymin
        grid.info.origin.orientation.w = 1.0
        values = np.full((self.height, self.width), -1, dtype=np.int8)
        occupied = (self.log_odds >= self.occupied_threshold).astype(np.uint8)
        occupied = cv2.morphologyEx(
            occupied, cv2.MORPH_CLOSE, np.ones((3, 3), dtype=np.uint8))
        count, labels, stats, _centroids = cv2.connectedComponentsWithStats(
            occupied, connectivity=8)
        for label in range(1, count):
            if stats[label, cv2.CC_STAT_AREA] < self.min_component_cells:
                occupied[labels == label] = 0
        occupied = occupied.astype(bool)
        free = np.logical_and(
            self.observed, np.logical_and(~occupied, self.log_odds <= self.free_threshold))
        values[free] = 0
        values[occupied] = 100
        uncertain = np.logical_and(self.observed, np.logical_and(~occupied, ~free))
        values[uncertain] = 50
        grid.data = values.reshape(-1).tolist()
        self.map_pub.publish(grid)


if __name__ == '__main__':
    rospy.init_node('odom_laser_mapper')
    OdomLaserMapper()
    rospy.spin()
