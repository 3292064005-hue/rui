#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Keep move_base translation aligned with the global plan and steer final yaw."""

from __future__ import print_function

import json
import math
import threading

import rospy
import tf
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from nav_msgs.srv import GetPlan
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
        self.min_yaw_control_scale = float(rospy.get_param(
            '~min_yaw_control_scale',
            rospy.get_param('~navigation_min_yaw_control_scale', 0.20)))
        self.full_yaw_control_distance = float(rospy.get_param(
            '~full_yaw_control_distance',
            rospy.get_param('~navigation_full_yaw_control_distance', 0.80)))
        self.min_teb_angular_keep_ratio = float(rospy.get_param(
            '~teb_angular_keep_ratio',
            rospy.get_param('~navigation_teb_angular_keep_ratio', 0.15)))
        self.teb_angular_blend_error = float(rospy.get_param(
            '~teb_angular_blend_error',
            rospy.get_param('~navigation_teb_angular_blend_error', 0.45)))
        self.path_velocity_filter_enabled = bool(rospy.get_param(
            '~path_velocity_filter_enabled',
            rospy.get_param('~navigation_path_velocity_filter', True)))
        self.global_plan_topic = rospy.get_param(
            '~global_plan_topic',
            rospy.get_param('~navigation_global_plan_topic',
                            '/move_base/GlobalPlanner/plan'))
        self.make_plan_service_name = rospy.get_param(
            '~make_plan_service',
            rospy.get_param('~navigation_make_plan_service', '/move_base/make_plan'))
        self.refresh_plan_from_service = bool(rospy.get_param(
            '~refresh_plan_from_service',
            rospy.get_param('~navigation_path_refresh_from_make_plan', True)))
        self.plan_refresh_interval = float(rospy.get_param(
            '~plan_refresh_interval',
            rospy.get_param('~navigation_path_refresh_interval', 0.5)))
        self.path_alignment_gain = clamp(float(rospy.get_param(
            '~path_alignment_gain',
            rospy.get_param('~navigation_path_alignment_gain', 0.92))),
            0.0, 1.0)
        self.path_lateral_gain = float(rospy.get_param(
            '~path_lateral_gain',
            rospy.get_param('~navigation_path_lateral_gain', 0.45)))
        self.max_lateral_speed_ratio = clamp(float(rospy.get_param(
            '~max_lateral_speed_ratio',
            rospy.get_param('~navigation_path_max_lateral_speed_ratio', 0.18))),
            0.0, 1.0)
        self.min_projected_speed = float(rospy.get_param(
            '~min_projected_speed',
            rospy.get_param('~navigation_path_min_projected_speed', 0.05)))
        self.max_path_age = float(rospy.get_param(
            '~max_path_age',
            rospy.get_param('~navigation_path_max_age', 2.0)))

        self._lock = threading.Lock()
        self._active_goal = False
        self._target_yaw = None
        self._target_xy = None
        self._last_status_time = None
        self._global_plan = []
        self._global_plan_stamp = None
        self._global_plan_frame = None
        self._last_plan_request_time = rospy.Time(0)
        self._last_make_plan_connect_time = rospy.Time(0)
        self._last_make_plan_failure_time = rospy.Time(0)

        self._tf_listener = tf.TransformListener()
        self._make_plan = None
        self._cmd_pub = rospy.Publisher(self.output_topic, Twist, queue_size=10)
        rospy.Subscriber(self.input_topic, Twist, self._cmd_cb, queue_size=10)
        rospy.Subscriber(self.target_yaw_topic, Float32, self._target_yaw_cb, queue_size=5)
        rospy.Subscriber(self.status_topic, String, self._status_cb, queue_size=20)
        rospy.Subscriber(self.global_plan_topic, Path, self._global_plan_cb, queue_size=1)

        rospy.loginfo(
            'cmd_vel_target_yaw_filter: %s -> %s, yaw topic=%s, status=%s, plan=%s, frame=%s->%s',
            self.input_topic, self.output_topic, self.target_yaw_topic,
            self.status_topic, self.global_plan_topic, self.control_frame, self.base_frame)

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
            if 'x' in waypoint and 'y' in waypoint:
                try:
                    self._target_xy = (float(waypoint['x']), float(waypoint['y']))
                except Exception:
                    self._target_xy = None

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

    def _global_plan_cb(self, msg):
        points = []
        for pose in msg.poses:
            points.append((float(pose.pose.position.x), float(pose.pose.position.y)))
        with self._lock:
            self._global_plan = points
            self._global_plan_frame = msg.header.frame_id
            self._global_plan_stamp = msg.header.stamp

    def _current_pose(self):
        try:
            translation, rotation = self._tf_listener.lookupTransform(
                self.control_frame, self.base_frame, rospy.Time(0))
            yaw = tf.transformations.euler_from_quaternion(rotation)[2]
            return float(translation[0]), float(translation[1]), float(yaw)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return None

    def _make_pose_stamped(self, x, y, yaw):
        msg = PoseStamped()
        msg.header.frame_id = self.control_frame
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        qx, qy, qz, qw = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        return msg

    def _connect_make_plan(self):
        if not self.refresh_plan_from_service or self._make_plan is not None:
            return
        now = rospy.Time.now()
        if (now - self._last_make_plan_connect_time).to_sec() < 1.0:
            return
        self._last_make_plan_connect_time = now
        try:
            rospy.wait_for_service(self.make_plan_service_name, timeout=0.05)
            self._make_plan = rospy.ServiceProxy(self.make_plan_service_name, GetPlan)
            rospy.loginfo(
                'cmd_vel_target_yaw_filter: connected to %s for global path alignment',
                self.make_plan_service_name)
        except Exception:
            pass

    def _refresh_plan_from_make_plan(self, pose, target_xy):
        if self._has_fresh_global_plan():
            return
        self._connect_make_plan()
        if self._make_plan is None or target_xy is None:
            return
        now = rospy.Time.now()
        if (now - self._last_make_plan_failure_time).to_sec() < 5.0:
            return
        if self.plan_refresh_interval > 0.0:
            elapsed = (now - self._last_plan_request_time).to_sec()
            if elapsed < self.plan_refresh_interval:
                return
        self._last_plan_request_time = now
        try:
            start = self._make_pose_stamped(pose[0], pose[1], pose[2])
            goal = self._make_pose_stamped(target_xy[0], target_xy[1], pose[2])
            response = self._make_plan(start=start, goal=goal, tolerance=0.02)
            self._global_plan_cb(response.plan)
        except Exception as exc:
            self._last_make_plan_failure_time = rospy.Time.now()
            rospy.logwarn_throttle(
                2.0, 'cmd_vel_target_yaw_filter: make_plan refresh failed: %s', exc)

    def _has_fresh_global_plan(self):
        with self._lock:
            plan_len = len(self._global_plan)
            frame_id = self._global_plan_frame
            stamp = self._global_plan_stamp
        if plan_len < 2:
            return False
        if frame_id and frame_id != self.control_frame:
            return False
        if stamp is None or self.max_path_age <= 0.0:
            return True
        try:
            return (rospy.Time.now() - stamp).to_sec() <= self.max_path_age
        except Exception:
            return False

    def _should_override(self):
        if not self.enabled:
            return False, None
        with self._lock:
            target_yaw = self._target_yaw
            target_xy = self._target_xy
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
        return True, (target_yaw, target_xy)

    def _copy_twist(self, msg):
        out = Twist()
        out.linear.x = msg.linear.x
        out.linear.y = msg.linear.y
        out.linear.z = msg.linear.z
        out.angular.x = msg.angular.x
        out.angular.y = msg.angular.y
        out.angular.z = msg.angular.z
        return out

    def _plan_tangent(self, pose):
        if not self.path_velocity_filter_enabled:
            return None
        with self._lock:
            plan = list(self._global_plan)
            frame_id = self._global_plan_frame
            stamp = self._global_plan_stamp

        if len(plan) < 2:
            return None
        if frame_id and frame_id != self.control_frame:
            return None
        if stamp is not None and self.max_path_age > 0.0:
            try:
                if (rospy.Time.now() - stamp).to_sec() > self.max_path_age:
                    return None
            except Exception:
                pass

        px, py = pose[0], pose[1]
        best = None
        for i in range(len(plan) - 1):
            ax, ay = plan[i]
            bx, by = plan[i + 1]
            sx = bx - ax
            sy = by - ay
            seg_len_sq = sx * sx + sy * sy
            if seg_len_sq <= 1e-9:
                continue
            t = clamp(((px - ax) * sx + (py - ay) * sy) / seg_len_sq, 0.0, 1.0)
            proj_x = ax + t * sx
            proj_y = ay + t * sy
            dist_sq = (px - proj_x) * (px - proj_x) + (py - proj_y) * (py - proj_y)
            if best is None or dist_sq < best[0]:
                seg_len = math.sqrt(seg_len_sq)
                best = (dist_sq, sx / seg_len, sy / seg_len, proj_x, proj_y)
        if best is None:
            return None
        return best[1], best[2], best[3], best[4]

    def _align_translation_to_path(self, out, pose):
        tangent = self._plan_tangent(pose)
        if tangent is None:
            return out

        tx, ty, proj_x, proj_y = tangent
        cos_yaw = math.cos(pose[2])
        sin_yaw = math.sin(pose[2])
        vx_world = cos_yaw * out.linear.x - sin_yaw * out.linear.y
        vy_world = sin_yaw * out.linear.x + cos_yaw * out.linear.y
        speed = math.hypot(vx_world, vy_world)
        if speed <= 1e-6:
            return out

        raw_along_speed = vx_world * tx + vy_world * ty
        along_speed = clamp(raw_along_speed, 0.0, speed)
        if along_speed < self.min_projected_speed and speed >= self.min_projected_speed:
            along_speed = self.min_projected_speed

        error_x = proj_x - pose[0]
        error_y = proj_y - pose[1]
        lateral_x = error_x - (error_x * tx + error_y * ty) * tx
        lateral_y = error_y - (error_x * tx + error_y * ty) * ty
        lateral_norm = math.hypot(lateral_x, lateral_y)
        lateral_speed = 0.0
        if lateral_norm > 1e-6:
            lateral_speed = clamp(
                self.path_lateral_gain * lateral_norm,
                0.0,
                self.max_lateral_speed_ratio * speed)

        aligned_vx = along_speed * tx
        aligned_vy = along_speed * ty
        if lateral_speed > 0.0:
            aligned_vx += lateral_speed * lateral_x / lateral_norm
            aligned_vy += lateral_speed * lateral_y / lateral_norm

        aligned_speed = math.hypot(aligned_vx, aligned_vy)
        if aligned_speed > speed:
            scale = speed / aligned_speed
            aligned_vx *= scale
            aligned_vy *= scale

        blend = self.path_alignment_gain
        vx_world = (1.0 - blend) * vx_world + blend * aligned_vx
        vy_world = (1.0 - blend) * vy_world + blend * aligned_vy
        out.linear.x = cos_yaw * vx_world + sin_yaw * vy_world
        out.linear.y = -sin_yaw * vx_world + cos_yaw * vy_world
        return out

    def _cmd_cb(self, msg):
        out = self._copy_twist(msg)
        should_override, target = self._should_override()
        pose = self._current_pose()
        target_xy = target[1] if should_override else None
        if pose is not None:
            self._refresh_plan_from_make_plan(pose, target_xy)
            out = self._align_translation_to_path(out, pose)
        if not should_override:
            self._cmd_pub.publish(out)
            return
        target_yaw, target_xy = target

        if pose is None:
            self._cmd_pub.publish(out)
            return

        current_yaw = pose[2]
        error = wrap_to_pi(target_yaw - current_yaw)
        abs_error = abs(error)
        if abs_error <= self.yaw_tolerance:
            out.angular.z = 0.0
        else:
            control_scale = 1.0
            if target_xy is not None and self.full_yaw_control_distance > 0.0:
                distance = math.hypot(target_xy[0] - pose[0], target_xy[1] - pose[1])
                if distance > self.full_yaw_control_distance:
                    self._cmd_pub.publish(out)
                    return
                control_scale = 1.0 - clamp(
                    distance / self.full_yaw_control_distance, 0.0, 1.0)
                control_scale = max(self.min_yaw_control_scale, control_scale)

            speed = clamp(
                self.yaw_gain * control_scale * error,
                -self.max_angular_speed,
                self.max_angular_speed)
            if self.min_angular_speed > 0.0 and abs(speed) < self.min_angular_speed:
                speed = math.copysign(self.min_angular_speed, error)
            blend = clamp(
                abs_error / max(self.yaw_tolerance, self.teb_angular_blend_error),
                0.0,
                1.0)
            teb_keep_ratio = self.min_teb_angular_keep_ratio + (
                1.0 - control_scale) * (1.0 - self.min_teb_angular_keep_ratio)
            teb_component = teb_keep_ratio * blend * msg.angular.z
            out.angular.z = clamp(
                speed + teb_component,
                -self.max_angular_speed,
                self.max_angular_speed)
        self._cmd_pub.publish(out)


if __name__ == '__main__':
    rospy.init_node('cmd_vel_target_yaw_filter')
    try:
        CmdVelTargetYawFilter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
