#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Keep holonomic translation from move_base while steering yaw to waypoint yaw."""

from __future__ import print_function

import json
import math
import threading

import rospy
import tf
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, String


def clamp(value, low, high):
    return max(low, min(high, value))


def wrap_to_pi(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class CmdVelTargetYawFilter(object):
    def __init__(self):
        self.input_topic = rospy.get_param('~input_cmd_vel_topic', '/cmd_vel_raw')
        self.output_topic = rospy.get_param('~output_cmd_vel_topic', '/cmd_vel')
        self.target_yaw_topic = rospy.get_param(
            '~navigation_target_yaw_topic', '/navigation_target_yaw')
        self.status_topic = rospy.get_param(
            '~navigation_status_topic', '/navigation_status')
        self.base_frame = rospy.get_param(
            '~navigation_base_frame', rospy.get_param('~base_frame', 'base_footprint'))
        self.control_frame = rospy.get_param(
            '~navigation_control_frame', rospy.get_param('~control_frame', 'map'))

        self.enabled = bool(rospy.get_param(
            '~enabled', rospy.get_param('~navigation_cmd_vel_target_yaw_filter', True)))
        self.require_active_goal = bool(rospy.get_param('~require_active_goal', True))
        default_status_timeout = float(rospy.get_param('~navigation_goal_timeout', 90.0)) + 10.0
        self.status_timeout = float(rospy.get_param(
            '~status_timeout',
            rospy.get_param(
                '~navigation_cmd_vel_target_yaw_status_timeout',
                default_status_timeout)))
        self.yaw_gain = float(rospy.get_param(
            '~yaw_gain', rospy.get_param('~navigation_rotation_gain', 1.8)))
        self.max_angular_speed = float(rospy.get_param(
            '~max_angular_speed', rospy.get_param('~navigation_rotation_max_speed', 0.65)))
        self.min_angular_speed = float(rospy.get_param(
            '~min_angular_speed', rospy.get_param('~navigation_rotation_min_speed', 0.12)))
        self.yaw_tolerance = float(rospy.get_param(
            '~yaw_tolerance', rospy.get_param('~navigation_rotation_tolerance', 0.08)))

        self._lock = threading.Lock()
        self._active_goal = False
        self._target_yaw = None
        self._last_status_time = None

        self._tf_listener = tf.TransformListener()
        self._cmd_pub = rospy.Publisher(self.output_topic, Twist, queue_size=10)
        rospy.Subscriber(self.input_topic, Twist, self._cmd_cb, queue_size=10)
        rospy.Subscriber(self.target_yaw_topic, Float32, self._target_yaw_cb, queue_size=5)
        rospy.Subscriber(self.status_topic, String, self._status_cb, queue_size=20)

        rospy.loginfo(
            'cmd_vel_target_yaw_filter: %s -> %s, yaw topic=%s, status=%s, frame=%s->%s',
            self.input_topic, self.output_topic, self.target_yaw_topic,
            self.status_topic, self.control_frame, self.base_frame)

    def _target_yaw_cb(self, msg):
        with self._lock:
            self._target_yaw = float(msg.data)

    def _status_cb(self, msg):
        try:
            payload = json.loads(msg.data)
        except Exception:
            return

        event = str(payload.get('event', ''))
        waypoint = payload.get('waypoint') or {}
        with self._lock:
            if 'yaw' in waypoint and waypoint.get('yaw') is not None:
                self._target_yaw = float(waypoint['yaw'])

            if event in (
                    'target_started',
                    'rotation_started',
                    'rotation_completed',
                    'rotation_timeout',
            ):
                self._active_goal = self._target_yaw is not None
                self._last_status_time = rospy.Time.now()
            elif event in (
                    'waypoint_reached',
                    'goal_failed',
                    'goal_timeout',
                    'patrol_completed',
                    'patrol_completed_with_failures',
                    'patrol_aborted',
                    'move_base_server_timeout',
            ):
                self._active_goal = False
                self._last_status_time = rospy.Time.now()

    def _current_yaw(self):
        try:
            _translation, rotation = self._tf_listener.lookupTransform(
                self.control_frame, self.base_frame, rospy.Time(0))
            return tf.transformations.euler_from_quaternion(rotation)[2]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return None

    def _should_override(self):
        if not self.enabled:
            return False, None
        with self._lock:
            target_yaw = self._target_yaw
            active_goal = self._active_goal
            last_status_time = self._last_status_time

        if target_yaw is None:
            return False, None
        if self.require_active_goal and not active_goal:
            return False, None
        if active_goal and self.status_timeout > 0.0 and last_status_time is not None:
            age = (rospy.Time.now() - last_status_time).to_sec()
            if age > self.status_timeout:
                return False, None
        return True, target_yaw

    def _copy_twist(self, msg):
        out = Twist()
        out.linear.x = msg.linear.x
        out.linear.y = msg.linear.y
        out.linear.z = msg.linear.z
        out.angular.x = msg.angular.x
        out.angular.y = msg.angular.y
        out.angular.z = msg.angular.z
        return out

    def _cmd_cb(self, msg):
        out = self._copy_twist(msg)
        should_override, target_yaw = self._should_override()
        if not should_override:
            self._cmd_pub.publish(out)
            return

        current_yaw = self._current_yaw()
        if current_yaw is None:
            self._cmd_pub.publish(out)
            return

        error = wrap_to_pi(target_yaw - current_yaw)
        if abs(error) <= self.yaw_tolerance:
            out.angular.z = 0.0
        else:
            speed = clamp(
                self.yaw_gain * error,
                -self.max_angular_speed,
                self.max_angular_speed)
            if self.min_angular_speed > 0.0 and abs(speed) < self.min_angular_speed:
                speed = math.copysign(self.min_angular_speed, error)
            out.angular.z = speed
        self._cmd_pub.publish(out)


if __name__ == '__main__':
    rospy.init_node('cmd_vel_target_yaw_filter')
    try:
        CmdVelTargetYawFilter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
