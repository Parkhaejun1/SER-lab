# Install script for directory: /home/robot/PycharmProjects/realsense/catkin_ws/src/moveit_calibration/moveit_calibration_plugins

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/robot/PycharmProjects/realsense/catkin_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/robot/PycharmProjects/realsense/catkin_ws/build/moveit_calibration/moveit_calibration_plugins/catkin_generated/installspace/moveit_calibration_plugins.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/moveit_calibration_plugins/cmake" TYPE FILE FILES
    "/home/robot/PycharmProjects/realsense/catkin_ws/build/moveit_calibration/moveit_calibration_plugins/catkin_generated/installspace/moveit_calibration_pluginsConfig.cmake"
    "/home/robot/PycharmProjects/realsense/catkin_ws/build/moveit_calibration/moveit_calibration_plugins/catkin_generated/installspace/moveit_calibration_pluginsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/moveit_calibration_plugins" TYPE FILE FILES "/home/robot/PycharmProjects/realsense/catkin_ws/src/moveit_calibration/moveit_calibration_plugins/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/moveit_calibration_plugins" TYPE FILE FILES
    "/home/robot/PycharmProjects/realsense/catkin_ws/src/moveit_calibration/moveit_calibration_plugins/handeye_calibration_target_plugin_description.xml"
    "/home/robot/PycharmProjects/realsense/catkin_ws/src/moveit_calibration/moveit_calibration_plugins/handeye_calibration_solver_plugin_description.xml"
    )
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/robot/PycharmProjects/realsense/catkin_ws/build/moveit_calibration/moveit_calibration_plugins/handeye_calibration_target/cmake_install.cmake")
  include("/home/robot/PycharmProjects/realsense/catkin_ws/build/moveit_calibration/moveit_calibration_plugins/handeye_calibration_solver/cmake_install.cmake")

endif()

