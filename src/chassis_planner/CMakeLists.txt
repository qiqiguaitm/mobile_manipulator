cmake_minimum_required(VERSION 3.0.2)
project(chassis_planner)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  rviz
  actionlib_msgs
)

## This setting causes Qt's "MOC" generation to happen automatically.
set(CMAKE_AUTOMOC ON)

## This plugin includes Qt widgets, so we must include Qt.
## We'll use the version that rviz used so they are compatible.
if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  ## pull in all required include dirs, define QT_LIBRARIES, etc.
  include(${QT_USE_FILE})
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
  set(QT_LIBRARIES Qt5::Widgets)
endif()
## I prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)

catkin_package(
  CATKIN_DEPENDS rviz actionlib_msgs geometry_msgs std_msgs
)

catkin_python_setup()

## RViz Plugin
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

## Here we specify the list of source files for the plugin
set(SOURCE_FILES
  rviz_navi_multi_goals_pub_plugin/navi_multi_goals_pub_rviz_plugin/src/multi_navi_goal_panel.cpp
)

## Create the plugin library
add_library(chassis_planner_rviz_plugin ${SOURCE_FILES})

## Link the library
target_link_libraries(chassis_planner_rviz_plugin ${QT_LIBRARIES} ${catkin_LIBRARIES})

## Install rules
install(TARGETS chassis_planner_rviz_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  rviz_navi_multi_goals_pub_plugin/navi_multi_goals_pub_rviz_plugin/plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS
  scripts/chassis_planner_node.py
  scripts/goal_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)