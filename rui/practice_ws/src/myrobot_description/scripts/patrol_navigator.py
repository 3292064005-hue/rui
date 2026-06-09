#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Holonomic waypoint patrol node for the existing Gazebo map.

The node publishes geometry_msgs/Twist to /cmd_vel. The URDF in this package
contains a gazebo_ros_planar_move plugin, so the mecanum base can move in x/y/yaw
without changing the map file.

It also publishes machine-readable patrol status JSON on /patrol_status. The
status stream is used by the evidence recorder to generate terminal logs,
trajectory files, and a mission summary for reports/PPT/video proof.
"""

from __future__ import print_function

import json
import math
import threading

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String


def clamp(value, low, high):
    return max(low, min(high, value))


def wrap_to_pi(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quaternion(q):
    # yaw from geometry_msgs/Quaternion
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class PatrolNavigator(object):
    def __init__(self):
        self.cmd_vel_topic = rospy.get_param('~cmd_vel_topic', '/cmd_vel')
        self.odom_topic = rospy.get_param('~odom_topic', '/odom')
        self.status_topic = rospy.get_param('~patrol_status_topic', '/patrol_status')
        self.control_rate = float(rospy.get_param('~control_rate', 20.0))

        self.goal_tolerance = float(rospy.get_param('~goal_tolerance', 0.08))
        self.yaw_tolerance = float(rospy.get_param('~yaw_tolerance', 0.20))
        self.position_gain = float(rospy.get_param('~position_gain', 0.90))
        self.yaw_gain = float(rospy.get_param('~yaw_gain', 1.60))
        self.max_linear_speed = float(rospy.get_param('~max_linear_speed', 0.20))
        self.max_lateral_speed = float(rospy.get_param('~max_lateral_speed', 0.16))
        self.max_angular_speed = float(rospy.get_param('~max_angular_speed', 0.90))
        self.face_path = bool(rospy.get_param('~face_path', False))
        self.loop = bool(rospy.get_param('~loop', False))
        self.stop_at_end = bool(rospy.get_param('~stop_at_end', True))
        self.odom_timeout = float(rospy.get_param('~odom_timeout', 8.0))

        self.pose_lock = threading.Lock()
        self.pose = None  # (x, y, yaw)
        self.last_odom_time = None

        raw_waypoints = rospy.get_param('~waypoints', [])
        self.waypoints = self._parse_waypoints(raw_waypoints)
        if not self.waypoints:
            raise rospy.ROSException('No valid ~waypoints were loaded. Check config/task_params.yaml.')

        self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=1)
        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=20, latch=True)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self._odom_cb, queue_size=1)

        rospy.on_shutdown(self._publish_stop)
        rospy.loginfo('patrol_navigator: loaded %d waypoints, cmd_vel=%s, odom=%s, status=%s',
                      len(self.waypoints), self.cmd_vel_topic, self.odom_topic, self.status_topic)

    def _parse_waypoints(self, raw):
        waypoints = []
        for index, item in enumerate(raw):
            try:
                if isinstance(item, dict):
                    name = str(item.get('name', 'wp_%02d' % index))
                    x = float(item['x'])
                    y = float(item['y'])
                    yaw = item.get('yaw', None)
                    yaw = None if yaw is None else float(yaw)
                    hold = float(item.get('hold', 0.0))
                elif isinstance(item, (list, tuple)) and len(item) >= 2:
                    name = 'wp_%02d' % index
                    x = float(item[0])
                    y = float(item[1])
                    yaw = float(item[2]) if len(item) >= 3 else None
                    hold = float(item[3]) if len(item) >= 4 else 0.0
                else:
                    rospy.logwarn('patrol_navigator: skip invalid waypoint %r', item)
                    continue
                waypoints.append({'name': name, 'x': x, 'y': y, 'yaw': yaw, 'hold': hold})
            except Exception as exc:
                rospy.logwarn('patrol_navigator: skip waypoint %r: %s', item, exc)
        return waypoints

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        with self.pose_lock:
            self.pose = (float(p.x), float(p.y), yaw_from_quaternion(q))
            self.last_odom_time = rospy.Time.now()

    def _get_pose(self):
        with self.pose_lock:
            return self.pose

    def _publish_stop(self):
        try:
            self.cmd_pub.publish(Twist())
        except Exception:
            pass

    def _publish_status(self, event, waypoint=None, index=None, pose=None, distance=None):
        payload = {
            'event': event,
            'stamp': rospy.Time.now().to_sec(),
            'total_waypoints': len(self.waypoints),
        }
        if waypoint is not None:
            payload['waypoint'] = {
                'name': waypoint.get('name'),
                'x': waypoint.get('x'),
                'y': waypoint.get('y'),
                'yaw': waypoint.get('yaw'),
                'hold': waypoint.get('hold', 0.0),
            }
        if index is not None:
            payload['waypoint_index'] = int(index)
        if pose is not None:
            payload['robot_pose'] = {
                'x': round(float(pose[0]), 3),
                'y': round(float(pose[1]), 3),
                'yaw': round(float(pose[2]), 3),
            }
        if distance is not None:
            payload['distance_to_goal'] = round(float(distance), 3)
        self.status_pub.publish(String(data=json.dumps(payload, ensure_ascii=False, sort_keys=True)))

    def _make_cmd(self, pose, waypoint):
        x, y, yaw = pose
        dx = waypoint['x'] - x
        dy = waypoint['y'] - y
        distance = math.hypot(dx, dy)

        # World-frame proportional command.
        if distance > 1e-6:
            speed = min(self.max_linear_speed, self.position_gain * distance)
            vx_world = speed * dx / distance
            vy_world = speed * dy / distance
        else:
            vx_world = 0.0
            vy_world = 0.0

        # Convert world-frame x/y command into robot body frame.
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        vx_body = cos_yaw * vx_world + sin_yaw * vy_world
        vy_body = -sin_yaw * vx_world + cos_yaw * vy_world

        vx_body = clamp(vx_body, -self.max_linear_speed, self.max_linear_speed)
        vy_body = clamp(vy_body, -self.max_lateral_speed, self.max_lateral_speed)

        if self.face_path and distance > self.goal_tolerance:
            yaw_target = math.atan2(dy, dx)
        else:
            yaw_target = waypoint.get('yaw')

        wz = 0.0
        yaw_error = 0.0
        if yaw_target is not None:
            yaw_error = wrap_to_pi(float(yaw_target) - yaw)
            wz = clamp(self.yaw_gain * yaw_error, -self.max_angular_speed, self.max_angular_speed)

        cmd = Twist()
        cmd.linear.x = vx_body
        cmd.linear.y = vy_body
        cmd.angular.z = wz
        return cmd, distance, abs(yaw_error)

    def run(self):
        rospy.loginfo('patrol_navigator: waiting for odometry on %s ...', self.odom_topic)
        start_wait = rospy.Time.now()
        rate = rospy.Rate(self.control_rate)
        while not rospy.is_shutdown() and self._get_pose() is None:
            if (rospy.Time.now() - start_wait).to_sec() > self.odom_timeout:
                self._publish_status('odom_timeout')
                rospy.logwarn_throttle(3.0, 'patrol_navigator: still waiting for odom. Check gazebo_ros_planar_move plugin and topic names.')
            rate.sleep()

        self._publish_status('patrol_started', pose=self._get_pose())
        waypoint_index = 0
        while not rospy.is_shutdown():
            waypoint = self.waypoints[waypoint_index]
            rospy.loginfo('patrol_navigator: target %d/%d %s -> (%.2f, %.2f)',
                          waypoint_index + 1, len(self.waypoints), waypoint['name'], waypoint['x'], waypoint['y'])
            self._publish_status('target_started', waypoint=waypoint, index=waypoint_index, pose=self._get_pose())

            reached = False
            while not rospy.is_shutdown() and not reached:
                pose = self._get_pose()
                if pose is None:
                    self._publish_stop()
                    rate.sleep()
                    continue

                cmd, distance, yaw_error = self._make_cmd(pose, waypoint)
                yaw_required = waypoint.get('yaw') is not None
                if distance <= self.goal_tolerance and (not yaw_required or yaw_error <= self.yaw_tolerance):
                    reached = True
                    self._publish_stop()
                    rospy.loginfo('patrol_navigator: reached %s at pose x=%.2f y=%.2f yaw=%.2f',
                                  waypoint['name'], pose[0], pose[1], pose[2])
                    self._publish_status('waypoint_reached', waypoint=waypoint, index=waypoint_index, pose=pose, distance=distance)
                    hold = max(0.0, float(waypoint.get('hold', 0.0)))
                    if hold > 0.0:
                        rospy.sleep(hold)
                    break

                self.cmd_pub.publish(cmd)
                rate.sleep()

            waypoint_index += 1
            if waypoint_index >= len(self.waypoints):
                if self.loop:
                    waypoint_index = 0
                    rospy.loginfo('patrol_navigator: loop enabled, restarting waypoint list')
                    self._publish_status('patrol_loop_restart', pose=self._get_pose())
                else:
                    if self.stop_at_end:
                        self._publish_stop()
                    rospy.loginfo('patrol_navigator: patrol completed')
                    self._publish_status('patrol_completed', pose=self._get_pose())
                    return


if __name__ == '__main__':
    rospy.init_node('patrol_navigator')
    try:
        PatrolNavigator().run()
    except rospy.ROSInterruptException:
        pass
