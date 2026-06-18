cmake_minimum_required(VERSION 3.0.2)
project(myrobot_simulation)

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
  joint_state_publisher
  robot_state_publisher
  urdf
  xacro
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

catkin_install_python(PROGRAMS
  scripts/cmd_vel_test_motion.py
  scripts/mecanum_sim_driver.py
  scripts/simulation_status_monitor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)
