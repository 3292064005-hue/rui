#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Drive sequential waypoints with move_base or direct odometry control.

The normal navigation mode follows Navfn plans. During SLAM, direct-path mode
uses the same waypoint list without requiring a map or move_base.
"""

from __future__ import print_function

import json
import math

import actionlib
import rospy
import tf
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.srv import GetPlan
from std_msgs.msg import String
from std_srvs.srv import Empty


def quaternion_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def clamp(value, low, high):
    return max(low, min(high, value))


def wrap_to_pi(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def parse_goals(raw):
    goals = []
    for index, item in enumerate(raw):
        try:
            if isinstance(item, dict):
                name = str(item.get('name', 'goal_%02d' % index))
                x = float(item['x'])
                y = float(item['y'])
                yaw = float(item.get('yaw', 0.0))
                hold = float(item.get('hold', 0.0))
                direct_path = bool(item.get('direct_path', False))
            elif isinstance(item, (list, tuple)) and len(item) >= 2:
                name = 'goal_%02d' % index
                x = float(item[0])
                y = float(item[1])
                yaw = float(item[2]) if len(item) >= 3 else 0.0
                hold = float(item[3]) if len(item) >= 4 else 0.0
                direct_path = False
            else:
                rospy.logwarn('move_base_waypoint_navigator: skip invalid goal %r', item)
                continue
            goals.append({
                'name': name,
                'x': x,
                'y': y,
                'yaw': yaw,
                'hold': hold,
                'direct_path': direct_path,
            })
        except Exception as exc:
            rospy.logwarn('move_base_waypoint_navigator: skip goal %r: %s', item, exc)
    return goals


class MoveBaseWaypointNavigator(object):
    def __init__(self):
        self.action_name = rospy.get_param('~move_base_action', '/move_base')
        self.status_topic = rospy.get_param('~navigation_status_topic', '/navigation_status')
        self.patrol_status_topic = rospy.get_param('~patrol_status_topic', self.status_topic)
        self.frame_id = rospy.get_param('~navigation_goal_frame', 'map')
        self.goal_timeout = float(rospy.get_param('~navigation_goal_timeout', 90.0))
        self.server_timeout = float(rospy.get_param('~navigation_server_timeout', 30.0))
        self.retry_count = int(rospy.get_param('~navigation_retry_count', 1))
        self.clear_on_retry = bool(rospy.get_param('~navigation_clear_costmaps_on_retry', True))
        self.stop_on_failure = bool(rospy.get_param('~navigation_stop_on_failure', False))
        self.default_hold = float(rospy.get_param('~navigation_hold_at_goal', 0.8))
        self.ignore_goal_hold = bool(rospy.get_param('~navigation_ignore_goal_hold', False))
        self.startup_stabilization_delay = float(rospy.get_param('~navigation_startup_stabilization_delay', 2.0))
        self.cmd_vel_topic = rospy.get_param('~cmd_vel_topic', '/cmd_vel')
        self.base_frame = rospy.get_param('~navigation_base_frame', 'base_footprint')
        self.control_frame = rospy.get_param('~navigation_control_frame', 'odom')
        self.separate_rotation = bool(rospy.get_param('~navigation_separate_rotation', True))
        self.rotation_rate = float(rospy.get_param('~navigation_rotation_rate', 20.0))
        self.rotation_gain = float(rospy.get_param('~navigation_rotation_gain', 1.8))
        self.rotation_max_speed = float(rospy.get_param('~navigation_rotation_max_speed', 0.65))
        self.rotation_min_speed = float(rospy.get_param('~navigation_rotation_min_speed', 0.12))
        self.rotation_tolerance = float(rospy.get_param('~navigation_rotation_tolerance', 0.08))
        self.rotation_timeout = float(rospy.get_param('~navigation_rotation_timeout', 20.0))
        self.stop_duration = float(rospy.get_param('~navigation_stop_duration', 0.35))
        self.use_holonomic_follower = bool(
            rospy.get_param('~navigation_use_holonomic_path_follower', True))
        self.use_direct_path = bool(
            rospy.get_param('~navigation_use_direct_path', False))
        self.make_plan_service_name = rospy.get_param(
            '~navigation_make_plan_service', self.action_name.rstrip('/') + '/make_plan')
        self.translation_rate = float(rospy.get_param('~navigation_translation_rate', 20.0))
        self.translation_tolerance = float(
            rospy.get_param('~navigation_translation_tolerance', 0.04))
        self.arrival_stable_cycles = int(
            rospy.get_param('~navigation_arrival_stable_cycles', 6))
        self.translation_gain = float(rospy.get_param('~navigation_translation_gain', 1.0))
        self.translation_max_forward_speed = float(
            rospy.get_param('~navigation_translation_max_forward_speed', 0.20))
        self.translation_max_lateral_speed = float(
            rospy.get_param('~navigation_translation_max_lateral_speed', 0.16))
        self.path_lookahead = float(rospy.get_param('~navigation_path_lookahead', 0.22))
        self.replan_interval = float(rospy.get_param('~navigation_replan_interval', 1.0))
        self.stuck_timeout = float(rospy.get_param('~navigation_stuck_timeout', 8.0))
        self.progress_distance = float(rospy.get_param('~navigation_progress_distance', 0.03))
        self.patrol_repeats = max(1, int(rospy.get_param('~navigation_patrol_repeats', 1)))

        raw_goals = rospy.get_param('~navigation_goals', None)
        if raw_goals is None:
            raw_goals = rospy.get_param('~waypoints', [])
        self.goals = parse_goals(raw_goals)
        if not self.goals:
            raise rospy.ROSException('No valid ~navigation_goals or ~waypoints loaded.')

        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=20, latch=True)
        if self.patrol_status_topic != self.status_topic:
            self.patrol_status_pub = rospy.Publisher(self.patrol_status_topic, String, queue_size=20, latch=True)
        else:
            self.patrol_status_pub = self.status_pub
        self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=1)
        self.client = actionlib.SimpleActionClient(self.action_name, MoveBaseAction)
        self.tf_listener = tf.TransformListener()
        self.clear_costmaps = None
        self.make_plan = None
        if not self.use_direct_path:
            self._setup_clear_costmaps()
            self._setup_make_plan()
        rospy.on_shutdown(self._shutdown)

    def _setup_clear_costmaps(self):
        if not self.clear_on_retry:
            return
        service_name = self.action_name.rstrip('/') + '/clear_costmaps'
        try:
            rospy.wait_for_service(service_name, timeout=5.0)
            self.clear_costmaps = rospy.ServiceProxy(service_name, Empty)
            rospy.loginfo('move_base_waypoint_navigator: connected to %s', service_name)
        except Exception as exc:
            rospy.logwarn('move_base_waypoint_navigator: clear_costmaps unavailable: %s', exc)
            self.clear_costmaps = None

    def _setup_make_plan(self):
        if not self.use_holonomic_follower:
            return
        try:
            rospy.wait_for_service(self.make_plan_service_name, timeout=8.0)
            self.make_plan = rospy.ServiceProxy(self.make_plan_service_name, GetPlan)
            rospy.loginfo('move_base_waypoint_navigator: connected to %s',
                          self.make_plan_service_name)
        except Exception as exc:
            raise rospy.ROSException('make_plan service unavailable: %s' % exc)

    def _publish_status(self, event, goal=None, index=None, attempt=None, state=None, text=None):
        payload = {
            'event': event,
            'stamp': rospy.Time.now().to_sec(),
            'total_waypoints': len(self.goals) * self.patrol_repeats,
            'route_waypoints': len(self.goals),
            'patrol_repeats': self.patrol_repeats,
            'ignore_goal_hold': self.ignore_goal_hold,
            'navigation_stack': (
                'direct_odom_holonomic' if self.use_direct_path else 'move_base'),
        }
        if goal is not None:
            payload['waypoint'] = {
                'name': goal['name'],
                'x': goal['x'],
                'y': goal['y'],
                'yaw': goal['yaw'],
                'hold': self._goal_hold(goal),
                'direct_path': bool(goal.get('direct_path', False)),
            }
        if index is not None:
            payload['waypoint_index'] = int(index)
        if attempt is not None:
            payload['attempt'] = int(attempt)
        if state is not None:
            payload['move_base_state'] = int(state)
            payload['move_base_state_text'] = GoalStatus.to_string(int(state))
        if text:
            payload['message'] = str(text)
        msg = String(data=json.dumps(payload, ensure_ascii=False, sort_keys=True))
        self.status_pub.publish(msg)
        if self.patrol_status_pub is not self.status_pub:
            self.patrol_status_pub.publish(msg)

    def _goal_hold(self, item):
        if self.ignore_goal_hold:
            return 0.0
        return max(0.0, float(item.get('hold', self.default_hold)))

    def _make_goal(self, item, yaw=None):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.frame_id
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = item['x']
        goal.target_pose.pose.position.y = item['y']
        goal.target_pose.pose.position.z = 0.0
        qx, qy, qz, qw = quaternion_from_yaw(item['yaw'] if yaw is None else yaw)
        goal.target_pose.pose.orientation.x = qx
        goal.target_pose.pose.orientation.y = qy
        goal.target_pose.pose.orientation.z = qz
        goal.target_pose.pose.orientation.w = qw
        return goal

    def _stop_robot(self):
        try:
            self.cmd_pub.publish(Twist())
        except Exception:
            pass

    def _stop_and_settle(self):
        deadline = rospy.Time.now() + rospy.Duration(max(0.0, self.stop_duration))
        rate = rospy.Rate(max(5.0, self.rotation_rate))
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            self._stop_robot()
            rate.sleep()

    def _current_yaw(self):
        pose = self._current_pose()
        return None if pose is None else pose[2]

    def _current_pose(self):
        try:
            translation, rotation = self.tf_listener.lookupTransform(
                self.control_frame, self.base_frame, rospy.Time(0))
            yaw = tf.transformations.euler_from_quaternion(rotation)[2]
            return float(translation[0]), float(translation[1]), float(yaw)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return None

    def _wait_for_current_yaw(self, timeout=5.0):
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        rate = rospy.Rate(20.0)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            yaw = self._current_yaw()
            if yaw is not None:
                return yaw
            rate.sleep()
        return None

    def _rotate_to_yaw(self, item, index, attempt):
        target_yaw = float(item['yaw'])
        self._stop_and_settle()
        self._publish_status('rotation_started', goal=item, index=index, attempt=attempt)
        rospy.loginfo('move_base_waypoint_navigator: stopped; rotating %s in place to yaw %.2f',
                      item['name'], target_yaw)

        deadline = rospy.Time.now() + rospy.Duration(self.rotation_timeout)
        rate = rospy.Rate(max(5.0, self.rotation_rate))
        stable_cycles = 0
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            current_yaw = self._current_yaw()
            if current_yaw is None:
                self._stop_robot()
                rate.sleep()
                continue

            error = wrap_to_pi(target_yaw - current_yaw)
            if abs(error) <= self.rotation_tolerance:
                stable_cycles += 1
                self._stop_robot()
                if stable_cycles >= 3:
                    self._stop_and_settle()
                    self._publish_status('rotation_completed', goal=item, index=index,
                                         attempt=attempt, text='pure in-place rotation completed')
                    return True
            else:
                stable_cycles = 0
                speed = clamp(self.rotation_gain * error,
                              -self.rotation_max_speed, self.rotation_max_speed)
                if abs(speed) < self.rotation_min_speed:
                    speed = math.copysign(self.rotation_min_speed, error)
                cmd = Twist()
                cmd.angular.z = speed
                self.cmd_pub.publish(cmd)
            rate.sleep()

        self._stop_and_settle()
        if not rospy.is_shutdown():
            self._publish_status('rotation_timeout', goal=item, index=index, attempt=attempt,
                                 text='in-place rotation did not reach yaw tolerance')
            rospy.logwarn('move_base_waypoint_navigator: rotation timeout at %s', item['name'])
        return False

    def _make_pose_stamped(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        qx, qy, qz, qw = quaternion_from_yaw(yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def _request_plan(self, pose, item):
        if self.use_direct_path or item.get('direct_path', False):
            return [
                self._make_pose_stamped(pose[0], pose[1], pose[2]),
                self._make_pose_stamped(item['x'], item['y'], pose[2]),
            ]
        if self.make_plan is None:
            return []
        start = self._make_pose_stamped(pose[0], pose[1], pose[2])
        goal = self._make_pose_stamped(item['x'], item['y'], pose[2])
        try:
            response = self.make_plan(start=start, goal=goal,
                                      tolerance=self.translation_tolerance)
            return list(response.plan.poses)
        except Exception as exc:
            rospy.logwarn_throttle(
                2.0, 'move_base_waypoint_navigator: make_plan failed: %s', exc)
            return []

    def _path_target(self, plan, pose):
        if not plan:
            return None
        px, py = pose[0], pose[1]
        closest = min(
            range(len(plan)),
            key=lambda i: math.hypot(
                plan[i].pose.position.x - px, plan[i].pose.position.y - py))
        for candidate in plan[closest + 1:]:
            if math.hypot(candidate.pose.position.x - px,
                          candidate.pose.position.y - py) >= self.path_lookahead:
                return candidate.pose.position.x, candidate.pose.position.y
        end = plan[-1].pose.position
        return end.x, end.y

    def _translation_cmd(self, pose, target, goal_distance):
        dx = target[0] - pose[0]
        dy = target[1] - pose[1]
        target_distance = math.hypot(dx, dy)
        if target_distance <= 1e-6:
            return Twist()

        speed = min(
            self.translation_max_forward_speed,
            self.translation_gain * goal_distance)
        vx_world = speed * dx / target_distance
        vy_world = speed * dy / target_distance

        cos_yaw = math.cos(pose[2])
        sin_yaw = math.sin(pose[2])
        vx_body = cos_yaw * vx_world + sin_yaw * vy_world
        vy_body = -sin_yaw * vx_world + cos_yaw * vy_world

        scale = 1.0
        if abs(vx_body) > self.translation_max_forward_speed:
            scale = min(scale, self.translation_max_forward_speed / abs(vx_body))
        if abs(vy_body) > self.translation_max_lateral_speed:
            scale = min(scale, self.translation_max_lateral_speed / abs(vy_body))

        cmd = Twist()
        cmd.linear.x = vx_body * scale
        cmd.linear.y = vy_body * scale
        cmd.angular.z = 0.0
        return cmd

    def _follow_holonomic_path(self, item):
        deadline = rospy.Time.now() + rospy.Duration(self.goal_timeout)
        rate = rospy.Rate(max(5.0, self.translation_rate))
        plan = []
        last_plan_time = rospy.Time(0)
        best_distance = float('inf')
        last_progress_time = rospy.Time.now()
        arrival_cycles = 0

        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            pose = self._current_pose()
            if pose is None:
                self._stop_robot()
                rate.sleep()
                continue

            goal_distance = math.hypot(item['x'] - pose[0], item['y'] - pose[1])
            if goal_distance <= self.translation_tolerance:
                self._stop_robot()
                arrival_cycles += 1
                if arrival_cycles >= max(1, self.arrival_stable_cycles):
                    self._stop_and_settle()
                    final_pose = self._current_pose()
                    if final_pose is not None:
                        final_error = math.hypot(
                            item['x'] - final_pose[0], item['y'] - final_pose[1])
                        rospy.loginfo(
                            'move_base_waypoint_navigator: translation reached %s, '
                            'final error %.3fm', item['name'], final_error)
                    return True
                rate.sleep()
                continue
            arrival_cycles = 0

            if best_distance - goal_distance >= self.progress_distance:
                best_distance = goal_distance
                last_progress_time = rospy.Time.now()

            now = rospy.Time.now()
            if not plan or (now - last_plan_time).to_sec() >= self.replan_interval:
                new_plan = self._request_plan(pose, item)
                last_plan_time = now
                if new_plan:
                    plan = new_plan

            target = self._path_target(plan, pose)
            if target is None:
                self._stop_robot()
            else:
                self.cmd_pub.publish(
                    self._translation_cmd(pose, target, goal_distance))

            if (now - last_progress_time).to_sec() >= self.stuck_timeout:
                self._stop_and_settle()
                rospy.logwarn(
                    'move_base_waypoint_navigator: no translation progress at %s',
                    item['name'])
                return False
            rate.sleep()

        self._stop_and_settle()
        return False

    def _run_holonomic_goal(self, item, index):
        for attempt in range(1, max(1, self.retry_count + 1) + 1):
            if rospy.is_shutdown():
                return False
            if attempt > 1:
                self._clear_costmaps()
                rospy.sleep(0.5)
            rospy.loginfo(
                'move_base_waypoint_navigator: holonomic goal %d/%d attempt %d: '
                '%s -> %.2f %.2f, then rotate to %.2f',
                index + 1, len(self.goals) * self.patrol_repeats, attempt, item['name'],
                item['x'], item['y'], item['yaw'])
            self._publish_status('target_started', goal=item, index=index, attempt=attempt)
            if not self._follow_holonomic_path(item):
                if rospy.is_shutdown():
                    return False
                self._publish_status(
                    'goal_failed', goal=item, index=index, attempt=attempt,
                    state=GoalStatus.ABORTED,
                    text='holonomic translation failed or made no progress')
                continue
            if self.separate_rotation and not self._rotate_to_yaw(item, index, attempt):
                if rospy.is_shutdown():
                    return False
                continue

            self._publish_status(
                'waypoint_reached', goal=item, index=index, attempt=attempt,
                state=GoalStatus.SUCCEEDED,
                text=(
                    'direct odometry path followed with separate in-place rotation'
                    if self.use_direct_path or item.get('direct_path', False)
                    else 'Navfn path followed with separate in-place rotation'))
            hold = self._goal_hold(item)
            if hold > 0.0:
                rospy.sleep(hold)
            return True
        return False

    def _clear_costmaps(self):
        if self.clear_costmaps is None:
            return
        try:
            self.clear_costmaps()
            rospy.loginfo('move_base_waypoint_navigator: requested costmap clear')
        except Exception as exc:
            rospy.logwarn('move_base_waypoint_navigator: failed to clear costmaps: %s', exc)

    def _run_one_goal(self, item, index):
        if self.use_holonomic_follower:
            return self._run_holonomic_goal(item, index)
        for attempt in range(1, max(1, self.retry_count + 1) + 1):
            if rospy.is_shutdown():
                return False
            if attempt > 1:
                self._clear_costmaps()
                rospy.sleep(0.5)
            rospy.loginfo('move_base_waypoint_navigator: goal %d/%d attempt %d: %s -> %.2f %.2f %.2f',
                          index + 1, len(self.goals) * self.patrol_repeats,
                          attempt, item['name'], item['x'], item['y'], item['yaw'])
            self._publish_status('target_started', goal=item, index=index, attempt=attempt)
            translation_yaw = self._wait_for_current_yaw()
            if translation_yaw is None:
                translation_yaw = item['yaw']
                rospy.logwarn('move_base_waypoint_navigator: current map yaw unavailable; using goal yaw')
            self.client.send_goal(self._make_goal(item, yaw=translation_yaw))
            finished = self.client.wait_for_result(rospy.Duration(self.goal_timeout))
            if not finished:
                self.client.cancel_goal()
                self._stop_robot()
                if rospy.is_shutdown():
                    return False
                self._publish_status('goal_timeout', goal=item, index=index, attempt=attempt,
                                     text='move_base did not finish before timeout')
                rospy.logwarn('move_base_waypoint_navigator: timeout at %s', item['name'])
                continue
            state = self.client.get_state()
            result_text = self.client.get_goal_status_text()
            if state == GoalStatus.SUCCEEDED:
                self._stop_and_settle()
                if self.separate_rotation and not self._rotate_to_yaw(item, index, attempt):
                    if rospy.is_shutdown():
                        return False
                    continue
                self._publish_status('waypoint_reached', goal=item, index=index, attempt=attempt, state=state, text=result_text)
                hold = self._goal_hold(item)
                if hold > 0.0:
                    rospy.sleep(hold)
                return True
            self._stop_robot()
            self._publish_status('goal_failed', goal=item, index=index, attempt=attempt, state=state, text=result_text)
            rospy.logwarn('move_base_waypoint_navigator: goal %s failed with state=%s text=%s',
                          item['name'], GoalStatus.to_string(state), result_text)
        return False

    def run(self):
        if self.use_direct_path:
            rospy.loginfo(
                'move_base_waypoint_navigator: direct odometry mode; '
                'move_base and a prebuilt map are not required')
        else:
            rospy.loginfo('move_base_waypoint_navigator: waiting for action server %s ...',
                          self.action_name)
            if not self.client.wait_for_server(rospy.Duration(self.server_timeout)):
                self._publish_status(
                    'move_base_server_timeout',
                    text='move_base action server unavailable')
                raise rospy.ROSException(
                    'move_base action server unavailable: %s' % self.action_name)

        if self.startup_stabilization_delay > 0.0:
            rospy.loginfo(
                'move_base_waypoint_navigator: waiting %.1fs for localization stabilization ...',
                self.startup_stabilization_delay)
            rospy.sleep(self.startup_stabilization_delay)

        self._publish_status('patrol_started')
        completed = 0
        failures = 0
        for lap in range(self.patrol_repeats):
            if self.patrol_repeats > 1:
                rospy.loginfo(
                    'move_base_waypoint_navigator: patrol lap %d/%d',
                    lap + 1, self.patrol_repeats)
            for index, item in enumerate(self.goals):
                absolute_index = lap * len(self.goals) + index
                ok = self._run_one_goal(item, absolute_index)
                if ok:
                    completed += 1
                else:
                    failures += 1
                    if self.stop_on_failure:
                        self._publish_status('patrol_aborted', goal=item,
                                             index=absolute_index,
                                             text='stop_on_failure enabled')
                        return False
        self._stop_robot()
        event = 'patrol_completed' if failures == 0 else 'patrol_completed_with_failures'
        self._publish_status(event, text='completed_goals=%d failed_goals=%d' % (completed, failures))
        return failures == 0

    def _shutdown(self):
        try:
            self.client.cancel_all_goals()
        except Exception:
            pass
        self._stop_robot()


if __name__ == '__main__':
    rospy.init_node('move_base_waypoint_navigator')
    try:
        ok = MoveBaseWaypointNavigator().run()
        if not ok and not rospy.is_shutdown():
            rospy.logwarn('move_base_waypoint_navigator: finished with navigation failures')
    except rospy.ROSInterruptException:
        pass
