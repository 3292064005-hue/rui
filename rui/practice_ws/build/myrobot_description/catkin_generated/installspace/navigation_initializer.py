#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Publish an initial AMCL pose repeatedly for deterministic Gazebo navigation."""

from __future__ import print_function

import math

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped


def quaternion_from_yaw(yaw):
    qz = math.sin(yaw * 0.5)
    qw = math.cos(yaw * 0.5)
    return (0.0, 0.0, qz, qw)


def main():
    rospy.init_node('navigation_initializer')
    frame_id = rospy.get_param('~frame_id', 'map')
    topic = rospy.get_param('~initialpose_topic', '/initialpose')
    x = float(rospy.get_param('~x', 0.0))
    y = float(rospy.get_param('~y', 0.0))
    yaw = float(rospy.get_param('~yaw', -1.5708))
    repeat = int(rospy.get_param('~repeat', 12))
    rate_hz = float(rospy.get_param('~rate', 2.0))
    cov_xx = float(rospy.get_param('~cov_xx', 0.02))
    cov_yy = float(rospy.get_param('~cov_yy', 0.02))
    cov_aa = float(rospy.get_param('~cov_aa', 0.04))

    pub = rospy.Publisher(topic, PoseWithCovarianceStamped, queue_size=1)
    rate = rospy.Rate(rate_hz)
    rospy.loginfo('navigation_initializer: publishing initial pose x=%.3f y=%.3f yaw=%.3f on %s', x, y, yaw, topic)

    qx, qy, qz, qw = quaternion_from_yaw(yaw)
    for _ in range(max(1, repeat)):
        if rospy.is_shutdown():
            break
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = frame_id
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.pose.covariance[0] = cov_xx
        msg.pose.covariance[7] = cov_yy
        msg.pose.covariance[35] = cov_aa
        pub.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
