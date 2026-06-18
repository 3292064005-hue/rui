#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Publish a short mecanum motion pattern to verify simulated drive interfaces."""
from __future__ import print_function

import rospy
from geometry_msgs.msg import Twist


def publish_for(pub, rate, duration, vx=0.0, vy=0.0, wz=0.0):
    msg = Twist()
    msg.linear.x = vx
    msg.linear.y = vy
    msg.angular.z = wz
    end = rospy.Time.now() + rospy.Duration(duration)
    while not rospy.is_shutdown() and rospy.Time.now() < end:
        pub.publish(msg)
        rate.sleep()


def main():
    rospy.init_node('cmd_vel_test_motion')
    topic = rospy.get_param('~cmd_vel_topic', '/cmd_vel')
    speed = float(rospy.get_param('~linear_speed', 0.18))
    lateral = float(rospy.get_param('~lateral_speed', 0.12))
    yaw = float(rospy.get_param('~angular_speed', 0.45))
    segment_time = float(rospy.get_param('~segment_time', 3.0))
    repeat = int(rospy.get_param('~repeat', 1))
    pub = rospy.Publisher(topic, Twist, queue_size=5)
    rate = rospy.Rate(20)
    rospy.sleep(1.0)
    for _ in range(repeat):
        publish_for(pub, rate, segment_time, vx=speed)
        publish_for(pub, rate, segment_time, vy=lateral)
        publish_for(pub, rate, segment_time, vx=-speed)
        publish_for(pub, rate, segment_time, vy=-lateral)
        publish_for(pub, rate, segment_time, wz=yaw)
    publish_for(pub, rate, 0.5)


if __name__ == '__main__':
    main()
