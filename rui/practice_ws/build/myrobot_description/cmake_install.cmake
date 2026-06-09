# Install script for directory: /root/workspace/rui/rui/practice_ws/src/myrobot_description

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/root/workspace/rui/rui/practice_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/root/workspace/rui/rui/practice_ws/build/myrobot_description/catkin_generated/installspace/myrobot_description.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/myrobot_description/cmake" TYPE FILE FILES
    "/root/workspace/rui/rui/practice_ws/build/myrobot_description/catkin_generated/installspace/myrobot_descriptionConfig.cmake"
    "/root/workspace/rui/rui/practice_ws/build/myrobot_description/catkin_generated/installspace/myrobot_descriptionConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/myrobot_description" TYPE FILE FILES "/root/workspace/rui/rui/practice_ws/src/myrobot_description/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/myrobot_description" TYPE PROGRAM FILES "/root/workspace/rui/rui/practice_ws/build/myrobot_description/catkin_generated/installspace/patrol_navigator.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/myrobot_description" TYPE PROGRAM FILES "/root/workspace/rui/rui/practice_ws/build/myrobot_description/catkin_generated/installspace/battlefield_recognition.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/myrobot_description" TYPE PROGRAM FILES "/root/workspace/rui/rui/practice_ws/build/myrobot_description/catkin_generated/installspace/task_evidence_recorder.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/myrobot_description" TYPE PROGRAM FILES "/root/workspace/rui/rui/practice_ws/build/myrobot_description/catkin_generated/installspace/latest_mission_summary.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/myrobot_description" TYPE PROGRAM FILES "/root/workspace/rui/rui/practice_ws/build/myrobot_description/catkin_generated/installspace/static_workspace_check.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/myrobot_description" TYPE PROGRAM FILES "/root/workspace/rui/rui/practice_ws/build/myrobot_description/catkin_generated/installspace/save_slam_map.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/myrobot_description" TYPE PROGRAM FILES "/root/workspace/rui/rui/practice_ws/build/myrobot_description/catkin_generated/installspace/slam_status_monitor.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/myrobot_description" TYPE PROGRAM FILES "/root/workspace/rui/rui/practice_ws/build/myrobot_description/catkin_generated/installspace/scan_self_filter.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/myrobot_description" TYPE PROGRAM FILES "/root/workspace/rui/rui/practice_ws/build/myrobot_description/catkin_generated/installspace/navigation_status_monitor.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/myrobot_description" TYPE PROGRAM FILES "/root/workspace/rui/rui/practice_ws/build/myrobot_description/catkin_generated/installspace/navigation_initializer.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/myrobot_description" TYPE PROGRAM FILES "/root/workspace/rui/rui/practice_ws/build/myrobot_description/catkin_generated/installspace/move_base_waypoint_navigator.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/myrobot_description" TYPE PROGRAM FILES "/root/workspace/rui/rui/practice_ws/build/myrobot_description/catkin_generated/installspace/generate_known_map.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/myrobot_description" TYPE PROGRAM FILES "/root/workspace/rui/rui/practice_ws/build/myrobot_description/catkin_generated/installspace/cmd_vel_test_motion.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/myrobot_description" TYPE PROGRAM FILES "/root/workspace/rui/rui/practice_ws/build/myrobot_description/catkin_generated/installspace/simulation_status_monitor.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/myrobot_description" TYPE PROGRAM FILES "/root/workspace/rui/rui/practice_ws/build/myrobot_description/catkin_generated/installspace/mecanum_sim_driver.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/myrobot_description" TYPE DIRECTORY FILES
    "/root/workspace/rui/rui/practice_ws/src/myrobot_description/urdf"
    "/root/workspace/rui/rui/practice_ws/src/myrobot_description/meshes"
    "/root/workspace/rui/rui/practice_ws/src/myrobot_description/launch"
    "/root/workspace/rui/rui/practice_ws/src/myrobot_description/config"
    "/root/workspace/rui/rui/practice_ws/src/myrobot_description/worlds"
    "/root/workspace/rui/rui/practice_ws/src/myrobot_description/maps"
    "/root/workspace/rui/rui/practice_ws/src/myrobot_description/materials"
    "/root/workspace/rui/rui/practice_ws/src/myrobot_description/recognition_templates"
    )
endif()

