#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Build a fixed-size occupancy grid from simulated odometry and laser scans."""

from __future__ import print_function

import math
import threading

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
        self.hit_range_margin = float(rospy.get_param('~hit_range_margin', 0.03))
        self.min_hit_count = int(rospy.get_param('~min_hit_count', 1))
        self.beam_step = max(1, int(rospy.get_param('~beam_step', 1)))
        self.min_insert_translation = float(rospy.get_param(
            '~min_insert_translation', 0.03))
        self.min_insert_rotation = float(rospy.get_param(
            '~min_insert_rotation', 0.06))
        self.traversed_clearance_radius = max(
            0.0, float(rospy.get_param('~traversed_clearance_radius', 0.08)))
        self.min_component_cells = int(rospy.get_param('~min_component_cells', 8))
        self.wall_thickness_cells = int(
            rospy.get_param('~wall_thickness_cells', 3))
        self.max_gap_cells = int(rospy.get_param('~max_gap_cells', 11))
        self.use_axis_line_filter = bool(rospy.get_param(
            '~use_axis_line_filter', True))
        self.hough_threshold = int(rospy.get_param('~hough_threshold', 22))
        self.hough_min_line_length = int(rospy.get_param(
            '~hough_min_line_length', 18))
        self.hough_max_line_gap = int(rospy.get_param(
            '~hough_max_line_gap', 9))
        self.axis_angle_tolerance = float(rospy.get_param(
            '~axis_angle_tolerance', 0.10))
        self.axis_merge_tolerance_cells = int(rospy.get_param(
            '~axis_merge_tolerance_cells', 5))
        self.axis_seed_gap_cells = int(rospy.get_param(
            '~axis_seed_gap_cells', 5))
        self.axis_short_min_line_length = int(rospy.get_param(
            '~axis_short_min_line_length', 10))
        self.axis_short_min_pixels = int(rospy.get_param(
            '~axis_short_min_pixels', 8))
        self.axis_short_max_thickness_cells = int(rospy.get_param(
            '~axis_short_max_thickness_cells', 6))
        self.axis_short_min_aspect = float(rospy.get_param(
            '~axis_short_min_aspect', 4.0))
        self.axis_short_gap_cells = int(rospy.get_param(
            '~axis_short_gap_cells', 4))
        self.axis_keep_short_segments = bool(rospy.get_param(
            '~axis_keep_short_segments', False))
        self.publish_rate = float(rospy.get_param('~publish_rate', 2.0))

        self.width = int(math.ceil((self.xmax - self.xmin) / self.resolution))
        self.height = int(math.ceil((self.ymax - self.ymin) / self.resolution))
        self.hit_count = np.zeros((self.height, self.width), dtype=np.uint16)
        self.observed = np.zeros((self.height, self.width), dtype=np.bool_)
        self.traversed = np.zeros((self.height, self.width), dtype=np.uint8)
        self._last_insert_pose = None
        self.map_lock = threading.Lock()
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

    def _ray_distance_to_map_edge(self, x, y, angle):
        """Return distance from an in-map point to the first map boundary."""
        dx = math.cos(angle)
        dy = math.sin(angle)
        distances = []
        epsilon = self.resolution * 0.25
        if dx > 1e-9:
            distances.append((self.xmax - epsilon - x) / dx)
        elif dx < -1e-9:
            distances.append((self.xmin + epsilon - x) / dx)
        if dy > 1e-9:
            distances.append((self.ymax - epsilon - y) / dy)
        elif dy < -1e-9:
            distances.append((self.ymin + epsilon - y) / dy)
        positive = [distance for distance in distances if distance >= 0.0]
        return min(positive) if positive else 0.0

    def _should_insert_scan(self, x, y, yaw):
        if self._last_insert_pose is None:
            self._last_insert_pose = (x, y, yaw)
            return True
        last_x, last_y, last_yaw = self._last_insert_pose
        moved = math.hypot(x - last_x, y - last_y)
        turned = abs(math.atan2(
            math.sin(yaw - last_yaw), math.cos(yaw - last_yaw)))
        if moved < self.min_insert_translation and turned < self.min_insert_rotation:
            return False
        self._last_insert_pose = (x, y, yaw)
        return True

    def _merge_axis_segments(self, segments):
        groups = []
        tolerance = max(1, self.axis_merge_tolerance_cells)
        gap = max(1, self.hough_max_line_gap + self.max_gap_cells)
        min_length = max(1, self.hough_min_line_length)
        for coord, start, end in sorted(segments, key=lambda item: (item[0], item[1])):
            for group in groups:
                if abs(int(round(np.median(group['coords']))) - coord) <= tolerance:
                    group['coords'].append(coord)
                    group['ranges'].append((start, end))
                    break
            else:
                groups.append({'coords': [coord], 'ranges': [(start, end)]})

        merged = []
        for group in groups:
            coord = int(round(np.median(group['coords'])))
            ranges = sorted(group['ranges'])
            current_start, current_end = ranges[0]
            for start, end in ranges[1:]:
                if start <= current_end + gap:
                    current_end = max(current_end, end)
                else:
                    if current_end - current_start >= min_length:
                        merged.append((coord, current_start, current_end))
                    current_start, current_end = start, end
            if current_end - current_start >= min_length:
                merged.append((coord, current_start, current_end))
        return merged

    def _axis_component_segments(self, occupied, horizontal):
        source = (occupied.astype(np.uint8) * 255)
        gap = max(1, self.axis_short_gap_cells)
        if horizontal:
            source = cv2.morphologyEx(
                source, cv2.MORPH_CLOSE, np.ones((1, gap), dtype=np.uint8))
        else:
            source = cv2.morphologyEx(
                source, cv2.MORPH_CLOSE, np.ones((gap, 1), dtype=np.uint8))

        count, labels, stats, centroids = cv2.connectedComponentsWithStats(
            (source > 0).astype(np.uint8), connectivity=8)
        segments = []
        min_length = max(1, self.axis_short_min_line_length)
        min_pixels = max(1, self.axis_short_min_pixels)
        max_thickness = max(1, self.axis_short_max_thickness_cells)
        min_aspect = max(1.0, self.axis_short_min_aspect)
        for label in range(1, count):
            x, y, width, height, area = [
                int(value) for value in stats[label]]
            if area < min_pixels:
                continue
            if horizontal:
                if width < min_length or height > max_thickness:
                    continue
                if (float(width) / max(1.0, float(height))) < min_aspect:
                    continue
                row = int(round(centroids[label][1]))
                segments.append((row, x, x + width - 1))
            else:
                if height < min_length or width > max_thickness:
                    continue
                if (float(height) / max(1.0, float(width))) < min_aspect:
                    continue
                col = int(round(centroids[label][0]))
                segments.append((col, y, y + height - 1))
        return segments

    def _extract_axis_lines(self, occupied):
        source = (occupied.astype(np.uint8) * 255)
        seed_gap = max(1, self.axis_seed_gap_cells)
        horizontal_seed = cv2.morphologyEx(
            source, cv2.MORPH_CLOSE, np.ones((1, seed_gap), dtype=np.uint8))
        vertical_seed = cv2.morphologyEx(
            source, cv2.MORPH_CLOSE, np.ones((seed_gap, 1), dtype=np.uint8))
        source = np.maximum(horizontal_seed, vertical_seed)
        edges = cv2.Canny(source, 50, 150, apertureSize=3)
        lines = cv2.HoughLinesP(
            edges, 1, np.pi / 180.0,
            threshold=max(1, self.hough_threshold),
            minLineLength=max(1, self.hough_min_line_length),
            maxLineGap=max(1, self.hough_max_line_gap))

        horizontal_segments = []
        vertical_segments = []
        if lines is not None:
            for line in lines[:, 0, :]:
                x1, y1, x2, y2 = [int(value) for value in line]
                dx = x2 - x1
                dy = y2 - y1
                length = math.hypot(dx, dy)
                if length < self.hough_min_line_length:
                    continue
                if abs(dy) <= max(2, abs(dx) * self.axis_angle_tolerance):
                    horizontal_segments.append(
                        (int(round((y1 + y2) / 2.0)), min(x1, x2), max(x1, x2)))
                elif abs(dx) <= max(2, abs(dy) * self.axis_angle_tolerance):
                    vertical_segments.append(
                        (int(round((x1 + x2) / 2.0)), min(y1, y2), max(y1, y2)))

        if self.axis_keep_short_segments:
            horizontal_segments.extend(
                self._axis_component_segments(occupied, True))
            vertical_segments.extend(
                self._axis_component_segments(occupied, False))

        filtered = np.zeros_like(occupied, dtype=np.uint8)
        thickness = max(1, self.wall_thickness_cells)
        half = thickness // 2
        for row, col_start, col_end in self._merge_axis_segments(horizontal_segments):
            row_start = max(0, row - half)
            row_end = min(self.height, row_start + thickness)
            cv2.rectangle(
                filtered, (col_start, row_start), (col_end, row_end - 1), 1, -1)
        for col, row_start, row_end in self._merge_axis_segments(vertical_segments):
            col_start = max(0, col - half)
            col_end = min(self.width, col_start + thickness)
            cv2.rectangle(
                filtered, (col_start, row_start), (col_end - 1, row_end), 1, -1)
        return filtered

    def _scan_cb(self, msg):
        try:
            translation, rotation = self.listener.lookupTransform(
                self.odom_frame, msg.header.frame_id, msg.header.stamp)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            try:
                translation, rotation = self.listener.lookupTransform(
                    self.odom_frame, msg.header.frame_id, rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as exc:
                rospy.logwarn_throttle(
                    5.0,
                    'odom_laser_mapper: waiting for TF %s -> %s: %s',
                    self.odom_frame, msg.header.frame_id, exc)
                return

        yaw = tf.transformations.euler_from_quaternion(rotation)[2]
        start = self._cell(translation[0], translation[1])
        if start is None:
            return
        if not self._should_insert_scan(translation[0], translation[1], yaw):
            return

        angle = msg.angle_min
        usable_range = min(msg.range_max, self.max_usable_range)
        with self.map_lock:
            clearance_cells = int(math.ceil(
                self.traversed_clearance_radius / self.resolution))
            if clearance_cells > 0:
                cv2.circle(
                    self.traversed, start, clearance_cells, 1, thickness=-1)
                self.observed[self.traversed.astype(bool)] = True
            for index, measured_range in enumerate(msg.ranges):
                world_angle = yaw + angle
                angle += msg.angle_increment
                if index % self.beam_step != 0:
                    continue
                if math.isnan(measured_range) or measured_range < msg.range_min:
                    continue

                has_hit = (
                    math.isfinite(measured_range)
                    and measured_range < usable_range - self.hit_range_margin)
                requested_distance = (
                    measured_range if has_hit else usable_range)
                edge_distance = self._ray_distance_to_map_edge(
                    translation[0], translation[1], world_angle)
                distance = min(requested_distance, edge_distance)
                if distance <= self.resolution:
                    continue

                end = self._cell(
                    translation[0] + distance * math.cos(world_angle),
                    translation[1] + distance * math.sin(world_angle))
                if end is None:
                    continue
                ray = bresenham(start[0], start[1], end[0], end[1])
                endpoint_is_hit = (
                    has_hit
                    and measured_range <= edge_distance + self.resolution)
                free_ray = ray[:-1] if endpoint_is_hit else ray
                for col, row in free_ray:
                    self.observed[row, col] = True
                if endpoint_is_hit:
                    col, row = ray[-1]
                    self.observed[row, col] = True
                    if self.hit_count[row, col] < np.iinfo(np.uint16).max:
                        self.hit_count[row, col] += 1

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
        with self.map_lock:
            hit_count = self.hit_count.copy()
            observed = self.observed.copy()
            traversed = self.traversed.copy()
        values = np.full((self.height, self.width), -1, dtype=np.int8)
        # Endpoint hits are more stable than per-cell free-space log odds at
        # 2 cm resolution: adjacent beams can traverse a neighboring wall cell.
        occupied = (hit_count >= self.min_hit_count).astype(np.uint8)
        # A robot pose is direct evidence of free space. Clear accumulated
        # self-reflections or floor returns along the driven centerline.
        occupied[traversed.astype(bool)] = 0
        count, labels, stats, _centroids = cv2.connectedComponentsWithStats(
            occupied, connectivity=8)
        for label in range(1, count):
            if stats[label, cv2.CC_STAT_AREA] < self.min_component_cells:
                occupied[labels == label] = 0
        if self.use_axis_line_filter:
            occupied = self._extract_axis_lines(occupied)
        else:
            thickness = max(1, self.wall_thickness_cells)
            if thickness > 1:
                occupied = cv2.dilate(
                    occupied, np.ones((thickness, thickness), dtype=np.uint8))
        gap = max(1, self.max_gap_cells)
        if gap > 1:
            horizontal = cv2.morphologyEx(
                occupied, cv2.MORPH_CLOSE,
                np.ones((1, gap), dtype=np.uint8))
            vertical = cv2.morphologyEx(
                occupied, cv2.MORPH_CLOSE,
                np.ones((gap, 1), dtype=np.uint8))
            occupied = np.maximum(horizontal, vertical)
        # The axis-line post filter can bridge a real opening if both sides of
        # a doorway look like one wall segment. Robot-traversed cells are direct
        # free-space evidence, so clear them again after all wall completion.
        occupied[traversed.astype(bool)] = 0
        occupied = occupied.astype(bool)
        free = np.logical_and(observed, ~occupied)
        values[free] = 0
        values[occupied] = 100
        grid.data = values.reshape(-1).tolist()
        self.map_pub.publish(grid)


if __name__ == '__main__':
    rospy.init_node('odom_laser_mapper')
    OdomLaserMapper()
    rospy.spin()
