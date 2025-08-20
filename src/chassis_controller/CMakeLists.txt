cmake_minimum_required(VERSION 3.0.2)
project(chassis_controller)

# This is a metapackage CMakeLists.txt that builds all chassis controller components
# Following catkin workspace conventions for package organization

# Find catkin build system
find_package(catkin REQUIRED)

# Declare this as a metapackage
catkin_metapackage()

# Build order based on dependencies:
# 1. tracer_msgs (messages first)
# 2. ugv_sdk (core library)  
# 3. tracer_base (depends on ugv_sdk and tracer_msgs)
# 4. tracer_description (URDF/visualization)
# 5. tracer_gazebo_sim (simulation, depends on description)

# The actual building is handled by catkin through package.xml dependencies
# This CMakeLists.txt serves as the entry point for the metapackage