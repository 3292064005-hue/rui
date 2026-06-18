cmake_minimum_required(VERSION 3.0.2)
project(myrobot_description)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY urdf meshes worlds materials config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)
