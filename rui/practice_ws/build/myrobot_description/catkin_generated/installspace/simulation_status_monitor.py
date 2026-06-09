#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Monitor core Gazebo simulation topics and publish a compact health report."""
from __future__ import print_function

import rospy
from std_msgs.msg import String


class TopicProbe(object):
    def __init__(self, topic, msg_type, min_rate_hz=0.0):
        self.topic = topic
        self.min_rate_hz = float(min_rate_hz)
        self.count = 0
        self.last_time = None
        self.first_time = None
        rospy.Subscriber(topic, msg_type, self._cb, queue_size=5)

    def _cb(self, _msg):
        now = rospy.Time.now()
        if self.first_time is None:
            self.first_time = now
        self.last_time = now
        self.count += 1

    def status(self, now, stale_after):
        if self.last_time is None:
            return 'MISSING %s' % self.topic
        age = (now - self.last_time).to_sec()
        span = max((now - self.first_time).to_sec(), 1e-6) if self.first_time else 1e-6
        rate = float(self.count) / span
        if age > stale_after:
            return 'STALE %s age=%.2fs rate=%.1fHz' % (self.topic, age, rate)
        if self.min_rate_hz > 0 and rate < self.min_rate_hz and span > 3.0:
            return 'SLOW %s rate=%.1fHz expected>=%.1fHz' % (self.topic, rate, self.min_rate_hz)
        return 'OK %s rate=%.1fHz age=%.2fs' % (self.topic, rate, age)


class SimulationStatusMonitor(object):
    def __init__(self):
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import LaserScan, Image, CameraInfo, Imu, JointState
        from nav_msgs.msg import Odometry
        from std_msgs.msg import Float32MultiArray

        self.stale_after = float(rospy.get_param('~stale_after', 2.0))
        self.publish_rate = float(rospy.get_param('~publish_rate', 1.0))
        self.required_topics = rospy.get_param('~required_topics', [
            '/clock', '/odom', '/scan', '/imu/data', '/camera/image_raw',
            '/camera/camera_info', '/joint_states', '/mecanum_wheel_speeds'
        ])
        msg_types = {
            '/clock': Clock,
            '/odom': Odometry,
            '/scan': LaserScan,
            '/imu/data': Imu,
            '/camera/image_raw': Image,
            '/camera/camera_info': CameraInfo,
            '/joint_states': JointState,
            '/mecanum_wheel_speeds': Float32MultiArray,
        }
        min_rates = {
            '/clock': 1.0,
            '/odom': 5.0,
            '/scan': 3.0,
            '/imu/data': 20.0,
            '/camera/image_raw': 5.0,
            '/camera/camera_info': 5.0,
            '/joint_states': 10.0,
            '/mecanum_wheel_speeds': 10.0,
        }
        self.probes = []
        for topic in self.required_topics:
            if topic in msg_types:
                self.probes.append(TopicProbe(topic, msg_types[topic], min_rates.get(topic, 0.0)))
            else:
                rospy.logwarn('No built-in message type for required topic %s; skipping', topic)
        self.pub = rospy.Publisher('/simulation_health', String, queue_size=5, latch=True)

    def spin(self):
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            items = [probe.status(now, self.stale_after) for probe in self.probes]
            ok = all(item.startswith('OK') for item in items)
            prefix = 'SIMULATION_OK' if ok else 'SIMULATION_NOT_READY'
            self.pub.publish(String(data=prefix + ' | ' + ' ; '.join(items)))
            rate.sleep()


def main():
    rospy.init_node('simulation_status_monitor')
    SimulationStatusMonitor().spin()


if __name__ == '__main__':
    main()
