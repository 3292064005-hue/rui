#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Gazebo helper driver for a four-wheel mecanum base.

The physical base motion in this package is handled by Gazebo's planar move
plugin. This node completes the simulation interface by converting /cmd_vel into
four mecanum wheel angular velocities and publishing wheel joint states so RViz
and robot_state_publisher show the wheels rotating.

Optional modes can also publish odometry/TF or directly set the Gazebo model
state, but those modes are disabled by default to avoid conflicting with the
planar move plugin.
"""
from __future__ import print_function

import math
import threading

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, String
from tf.broadcaster import TransformBroadcaster
from tf.transformations import quaternion_from_euler

try:
    from gazebo_msgs.msg import ModelState
    from gazebo_msgs.srv import SetModelState
except Exception:  # pragma: no cover - ROS import guard for static checks
    ModelState = None
    SetModelState = None


class MecanumSimDriver(object):
    def __init__(self):
        self.wheel_radius = float(rospy.get_param('~wheel_radius', 0.033))
        self.half_wheel_base = float(rospy.get_param('~half_wheel_base', 0.0638))
        self.half_track_width = float(rospy.get_param('~half_track_width', 0.0850))
        self.cmd_vel_timeout = float(rospy.get_param('~cmd_vel_timeout', 0.5))
        self.rate_hz = float(rospy.get_param('~joint_state_rate', 50.0))
        self.max_wheel_angular_speed = float(rospy.get_param('~max_wheel_angular_speed', 45.0))
        self.publish_joint_states = bool(rospy.get_param('~publish_joint_states', True))
        self.publish_wheel_speeds = bool(rospy.get_param('~publish_wheel_speeds', True))
        self.publish_odometry = bool(rospy.get_param('~publish_odometry', False))
        self.publish_tf = bool(rospy.get_param('~publish_tf', False))
        self.control_gazebo_model = bool(rospy.get_param('~control_gazebo_model', False))
        self.gazebo_model_name = rospy.get_param('~gazebo_model_name', 'mycar')
        self.base_frame = rospy.get_param('~base_frame', 'base_footprint')
        self.odom_frame = rospy.get_param('~odom_frame', 'odom')

        self.cmd_vel_topic = rospy.get_param('~cmd_vel_topic', '/cmd_vel')
        self.joint_states_topic = rospy.get_param('~joint_states_topic', '/joint_states')
        self.wheel_speed_topic = rospy.get_param('~wheel_speed_topic', '/mecanum_wheel_speeds')
        self.odom_topic = rospy.get_param('~odom_topic', '/sim_driver/odom')

        self.joint_names = [
            'wheel_left_front_joint',
            'wheel_right_front_joint',
            'wheel_left_rear_joint',
            'wheel_right_rear_joint',
        ]

        self._lock = threading.Lock()
        self._cmd = Twist()
        self._last_cmd_time = rospy.Time.now()
        self._wheel_pos = [0.0, 0.0, 0.0, 0.0]
        self._pose_x = 0.0
        self._pose_y = 0.0
        self._pose_yaw = 0.0
        self._last_update = rospy.Time.now()

        self._joint_pub = rospy.Publisher(self.joint_states_topic, JointState, queue_size=10) if self.publish_joint_states else None
        self._wheel_pub = rospy.Publisher(self.wheel_speed_topic, Float32MultiArray, queue_size=10) if self.publish_wheel_speeds else None
        self._odom_pub = rospy.Publisher(self.odom_topic, Odometry, queue_size=10) if self.publish_odometry else None
        self._status_pub = rospy.Publisher('/mecanum_sim_driver/status', String, queue_size=5, latch=True)
        self._tf_broadcaster = TransformBroadcaster() if self.publish_tf else None

        self._set_model_state = None
        if self.control_gazebo_model:
            if SetModelState is None:
                rospy.logwarn('gazebo_msgs is not available; control_gazebo_model disabled')
                self.control_gazebo_model = False
            else:
                rospy.loginfo('Waiting for /gazebo/set_model_state service...')
                try:
                    rospy.wait_for_service('/gazebo/set_model_state', timeout=10.0)
                    self._set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
                except Exception as exc:
                    rospy.logwarn('Could not connect to /gazebo/set_model_state: %s', exc)
                    self.control_gazebo_model = False

        rospy.Subscriber(self.cmd_vel_topic, Twist, self._cmd_cb, queue_size=10)
        self._status_pub.publish(String(data='mecanum_sim_driver_ready'))

    def _cmd_cb(self, msg):
        with self._lock:
            self._cmd = msg
            self._last_cmd_time = rospy.Time.now()

    def _get_cmd(self, now):
        with self._lock:
            age = (now - self._last_cmd_time).to_sec()
            if age > self.cmd_vel_timeout:
                return 0.0, 0.0, 0.0
            return self._cmd.linear.x, self._cmd.linear.y, self._cmd.angular.z

    def _mecanum_inverse(self, vx, vy, wz):
        # Robot frame: +x forward, +y left, +z yaw left.
        k = self.half_wheel_base + self.half_track_width
        r = self.wheel_radius
        fl = (vx - vy - k * wz) / r
        fr = (vx + vy + k * wz) / r
        rl = (vx + vy - k * wz) / r
        rr = (vx - vy + k * wz) / r
        wheels = [fl, fr, rl, rr]
        if self.max_wheel_angular_speed > 0:
            wheels = [max(-self.max_wheel_angular_speed, min(self.max_wheel_angular_speed, w)) for w in wheels]
        return wheels

    def _integrate_pose(self, vx, vy, wz, dt):
        c = math.cos(self._pose_yaw)
        s = math.sin(self._pose_yaw)
        self._pose_x += (vx * c - vy * s) * dt
        self._pose_y += (vx * s + vy * c) * dt
        self._pose_yaw = math.atan2(math.sin(self._pose_yaw + wz * dt), math.cos(self._pose_yaw + wz * dt))

    def _publish_joint_states(self, now, wheel_vel, dt):
        for i, w in enumerate(wheel_vel):
            self._wheel_pos[i] += w * dt
        msg = JointState()
        msg.header.stamp = now
        msg.name = list(self.joint_names)
        msg.position = list(self._wheel_pos)
        msg.velocity = list(wheel_vel)
        msg.effort = [0.0, 0.0, 0.0, 0.0]
        self._joint_pub.publish(msg)

    def _publish_wheel_speeds(self, wheel_vel):
        msg = Float32MultiArray()
        # order: left_front, right_front, left_rear, right_rear
        msg.data = list(wheel_vel)
        self._wheel_pub.publish(msg)

    def _publish_odom(self, now, vx, vy, wz):
        quat = quaternion_from_euler(0.0, 0.0, self._pose_yaw)
        msg = Odometry()
        msg.header.stamp = now
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = self._pose_x
        msg.pose.pose.position.y = self._pose_y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = quat[0]
        msg.pose.pose.orientation.y = quat[1]
        msg.pose.pose.orientation.z = quat[2]
        msg.pose.pose.orientation.w = quat[3]
        msg.twist.twist.linear.x = vx
        msg.twist.twist.linear.y = vy
        msg.twist.twist.angular.z = wz
        self._odom_pub.publish(msg)
        if self._tf_broadcaster:
            self._tf_broadcaster.sendTransform(
                (self._pose_x, self._pose_y, 0.0), quat, now, self.base_frame, self.odom_frame
            )

    def _set_gazebo_model_state(self, vx, vy, wz):
        if not self._set_model_state or ModelState is None:
            return
        quat = quaternion_from_euler(0.0, 0.0, self._pose_yaw)
        state = ModelState()
        state.model_name = self.gazebo_model_name
        state.reference_frame = 'world'
        state.pose.position.x = self._pose_x
        state.pose.position.y = self._pose_y
        state.pose.position.z = 0.1
        state.pose.orientation.x = quat[0]
        state.pose.orientation.y = quat[1]
        state.pose.orientation.z = quat[2]
        state.pose.orientation.w = quat[3]
        state.twist.linear.x = vx
        state.twist.linear.y = vy
        state.twist.angular.z = wz
        try:
            self._set_model_state(state)
        except Exception as exc:
            rospy.logwarn_throttle(5.0, 'set_model_state failed: %s', exc)

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            dt = max(0.0, (now - self._last_update).to_sec())
            if dt <= 0.0:
                rate.sleep()
                continue
            if dt > 1.0:
                dt = 1.0 / max(self.rate_hz, 1.0)
            self._last_update = now

            vx, vy, wz = self._get_cmd(now)
            wheel_vel = self._mecanum_inverse(vx, vy, wz)

            if self.publish_joint_states:
                self._publish_joint_states(now, wheel_vel, dt)
            if self.publish_wheel_speeds:
                self._publish_wheel_speeds(wheel_vel)
            if self.publish_odometry or self.control_gazebo_model:
                self._integrate_pose(vx, vy, wz, dt)
            if self.publish_odometry:
                self._publish_odom(now, vx, vy, wz)
            if self.control_gazebo_model:
                self._set_gazebo_model_state(vx, vy, wz)

            rate.sleep()


def main():
    rospy.init_node('mecanum_sim_driver')
    driver = MecanumSimDriver()
    try:
        driver.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
