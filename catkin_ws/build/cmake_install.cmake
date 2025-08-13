# Install script for directory: /home/agilex/AgileXDemo/catkin_ws/src

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/agilex/AgileXDemo/catkin_ws/install")
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
  
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
        file(MAKE_DIRECTORY "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
      endif()
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin")
        file(WRITE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin" "")
      endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/agilex/AgileXDemo/catkin_ws/install/_setup_util.py")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/agilex/AgileXDemo/catkin_ws/install" TYPE PROGRAM FILES "/home/agilex/AgileXDemo/catkin_ws/build/catkin_generated/installspace/_setup_util.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/agilex/AgileXDemo/catkin_ws/install/env.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/agilex/AgileXDemo/catkin_ws/install" TYPE PROGRAM FILES "/home/agilex/AgileXDemo/catkin_ws/build/catkin_generated/installspace/env.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/agilex/AgileXDemo/catkin_ws/install/setup.bash;/home/agilex/AgileXDemo/catkin_ws/install/local_setup.bash")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/agilex/AgileXDemo/catkin_ws/install" TYPE FILE FILES
    "/home/agilex/AgileXDemo/catkin_ws/build/catkin_generated/installspace/setup.bash"
    "/home/agilex/AgileXDemo/catkin_ws/build/catkin_generated/installspace/local_setup.bash"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/agilex/AgileXDemo/catkin_ws/install/setup.sh;/home/agilex/AgileXDemo/catkin_ws/install/local_setup.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/agilex/AgileXDemo/catkin_ws/install" TYPE FILE FILES
    "/home/agilex/AgileXDemo/catkin_ws/build/catkin_generated/installspace/setup.sh"
    "/home/agilex/AgileXDemo/catkin_ws/build/catkin_generated/installspace/local_setup.sh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/agilex/AgileXDemo/catkin_ws/install/setup.zsh;/home/agilex/AgileXDemo/catkin_ws/install/local_setup.zsh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/agilex/AgileXDemo/catkin_ws/install" TYPE FILE FILES
    "/home/agilex/AgileXDemo/catkin_ws/build/catkin_generated/installspace/setup.zsh"
    "/home/agilex/AgileXDemo/catkin_ws/build/catkin_generated/installspace/local_setup.zsh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/agilex/AgileXDemo/catkin_ws/install/setup.fish;/home/agilex/AgileXDemo/catkin_ws/install/local_setup.fish")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/agilex/AgileXDemo/catkin_ws/install" TYPE FILE FILES
    "/home/agilex/AgileXDemo/catkin_ws/build/catkin_generated/installspace/setup.fish"
    "/home/agilex/AgileXDemo/catkin_ws/build/catkin_generated/installspace/local_setup.fish"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/agilex/AgileXDemo/catkin_ws/install/.rosinstall")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/agilex/AgileXDemo/catkin_ws/install" TYPE FILE FILES "/home/agilex/AgileXDemo/catkin_ws/build/catkin_generated/installspace/.rosinstall")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/agilex/AgileXDemo/catkin_ws/build/gtest/cmake_install.cmake")
  include("/home/agilex/AgileXDemo/catkin_ws/build/piper_ros/src/piper_moveit/piper_no_gripper_moveit/cmake_install.cmake")
  include("/home/agilex/AgileXDemo/catkin_ws/build/piper_ros/src/piper_moveit/piper_with_gripper_moveit/cmake_install.cmake")
  include("/home/agilex/AgileXDemo/catkin_ws/build/realsense2_camera/realsense2_description/cmake_install.cmake")
  include("/home/agilex/AgileXDemo/catkin_ws/build/chassis_planner/cmake_install.cmake")
  include("/home/agilex/AgileXDemo/catkin_ws/build/object_tracker/cmake_install.cmake")
  include("/home/agilex/AgileXDemo/catkin_ws/build/piper_ros/src/piper_sim/piper_mujoco/cmake_install.cmake")
  include("/home/agilex/AgileXDemo/catkin_ws/build/piper_ros/src/piper_noetic/cmake_install.cmake")
  include("/home/agilex/AgileXDemo/catkin_ws/build/target_filter/cmake_install.cmake")
  include("/home/agilex/AgileXDemo/catkin_ws/build/task_router/cmake_install.cmake")
  include("/home/agilex/AgileXDemo/catkin_ws/build/arm_planner/cmake_install.cmake")
  include("/home/agilex/AgileXDemo/catkin_ws/build/arm_controller/cmake_install.cmake")
  include("/home/agilex/AgileXDemo/catkin_ws/build/camera_driver/cmake_install.cmake")
  include("/home/agilex/AgileXDemo/catkin_ws/build/imu_driver/cmake_install.cmake")
  include("/home/agilex/AgileXDemo/catkin_ws/build/inference_abstraction/cmake_install.cmake")
  include("/home/agilex/AgileXDemo/catkin_ws/build/lidar_driver/cmake_install.cmake")
  include("/home/agilex/AgileXDemo/catkin_ws/build/perception/cmake_install.cmake")
  include("/home/agilex/AgileXDemo/catkin_ws/build/piper_ros/src/piper/cmake_install.cmake")
  include("/home/agilex/AgileXDemo/catkin_ws/build/piper_ros/src/piper_msgs/cmake_install.cmake")
  include("/home/agilex/AgileXDemo/catkin_ws/build/slam/cmake_install.cmake")
  include("/home/agilex/AgileXDemo/catkin_ws/build/task_mgr/cmake_install.cmake")
  include("/home/agilex/AgileXDemo/catkin_ws/build/sensor_calibration/cmake_install.cmake")
  include("/home/agilex/AgileXDemo/catkin_ws/build/chassis_controller/cmake_install.cmake")
  include("/home/agilex/AgileXDemo/catkin_ws/build/piper_ros/src/piper_sim/piper_gazebo/cmake_install.cmake")
  include("/home/agilex/AgileXDemo/catkin_ws/build/realsense2_camera/realsense2_camera/cmake_install.cmake")
  include("/home/agilex/AgileXDemo/catkin_ws/build/odom/cmake_install.cmake")
  include("/home/agilex/AgileXDemo/catkin_ws/build/piper_ros/src/mobile_manipulator2_description/mobile_manipulator2_description/cmake_install.cmake")
  include("/home/agilex/AgileXDemo/catkin_ws/build/piper_ros/src/mobile_manipulator2_description/piper_description/cmake_install.cmake")
  include("/home/agilex/AgileXDemo/catkin_ws/build/piper_ros/src/mobile_manipulator2_description/tracer2_description/cmake_install.cmake")
  include("/home/agilex/AgileXDemo/catkin_ws/build/piper_ros/src/piper_moveit/moveit_ctrl/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/agilex/AgileXDemo/catkin_ws/build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
