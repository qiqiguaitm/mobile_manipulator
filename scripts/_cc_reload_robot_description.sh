#!/bin/bash
# 重新加载robot_description
source ~/MobileManipulator/devel/setup.bash
rosparam load ~/MobileManipulator/src/robot_description/mobile_manipulator2_description/urdf/mobile_manipulator2_description.urdf robot_description
echo "已重新加载robot_description"
