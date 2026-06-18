cmake_minimum_required(VERSION 3.0.2)
project(myrobot_navigation)

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
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

catkin_install_python(PROGRAMS
  scripts/cmd_vel_target_yaw_filter.py
  scripts/generate_known_map.py
  scripts/move_base_waypoint_navigator.py
  scripts/navigation_initializer.py
  scripts/navigation_status_monitor.py
  scripts/odom_laser_mapper.py
  scripts/save_slam_map.py
  scripts/scan_self_filter.py
  scripts/slam_status_monitor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config maps
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)
