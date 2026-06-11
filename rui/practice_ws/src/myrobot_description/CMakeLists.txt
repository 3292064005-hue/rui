cmake_minimum_required(VERSION 3.0.2)
project(myrobot_description)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  rosgraph_msgs
  tf
  gazebo_msgs
  cv_bridge
  visualization_msgs
  std_srvs
  actionlib_msgs
  actionlib
  move_base_msgs
  joint_state_publisher
  robot_state_publisher
  urdf
  xacro
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/patrol_navigator.py
  scripts/battlefield_recognition.py
  scripts/task_evidence_recorder.py
  scripts/latest_mission_summary.py
  scripts/static_workspace_check.py
  scripts/save_slam_map.py
  scripts/slam_status_monitor.py
  scripts/scan_self_filter.py
  scripts/odom_laser_mapper.py
  scripts/navigation_status_monitor.py
  scripts/navigation_initializer.py
  scripts/move_base_waypoint_navigator.py
  scripts/cmd_vel_target_yaw_filter.py
  scripts/generate_known_map.py
  scripts/cmd_vel_test_motion.py
  scripts/simulation_status_monitor.py
  scripts/mecanum_sim_driver.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY urdf meshes launch config worlds maps materials recognition_templates recognition_weights python_vendor
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)
