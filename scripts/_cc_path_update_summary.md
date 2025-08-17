# Files Requiring Path Updates for mobile_manipulator2_description

Based on the search results, the following files contain references to "mobile_manipulator2_description" that may need updating to include the new "robot_desc" parent directory:

## Launch Files
1. `/home/agilex/MobileManipulator/src/arm_controller/launch/start_single_piper_rviz.launch`
   - Line 4: References `$(find mobile_manipulator2_description)/piper_description/launch/...`

2. `/home/agilex/MobileManipulator/src/arm_controller/launch/start_single_piper.launch`
   - Line 5: Commented reference to `$(find mobile_manipulator2_description)/piper_description/launch/...`

3. `/home/agilex/MobileManipulator/src/arm_planner/moveit_config/piper_no_gripper/launch/gazebo.launch`
   - Line 16: References `$(find mobile_manipulator2_description)/piper_description/urdf/...`

4. `/home/agilex/MobileManipulator/src/arm_planner/moveit_config/piper_no_gripper/launch/planning_context.launch`
   - Line 9: References `$(find mobile_manipulator2_description)/piper_description/urdf/...`
   - Line 12: References `$(find mobile_manipulator2_description)/config/...`

5. `/home/agilex/MobileManipulator/src/arm_planner/moveit_config/piper_with_gripper/launch/gazebo.launch`
   - Line 16: References `$(find mobile_manipulator2_description)/piper_description/urdf/...`

6. `/home/agilex/MobileManipulator/src/arm_planner/moveit_config/piper_with_gripper/launch/planning_context.launch`
   - Line 9: References `$(find mobile_manipulator2_description)/urdf/...`
   - Line 12: References `$(find mobile_manipulator2_description)/config/...`

## URDF/Xacro Files in robot_desc
These files are already in the correct location but contain self-references that are likely correct:
- `/home/agilex/MobileManipulator/src/robot_desc/mobile_manipulator2_description/urdf/mobile_manipulator2_description.urdf`
  - Multiple mesh references using `package://mobile_manipulator2_description/meshes/...`
- `/home/agilex/MobileManipulator/src/robot_desc/mobile_manipulator2_description/urdf/mobile_manipulator2_with_v100_camera.xacro`
- `/home/agilex/MobileManipulator/src/robot_desc/mobile_manipulator2_description/urdf/mobile_manipulator2_with_hand_cam.xacro`

## Launch Files in robot_desc
These are also in the correct location and their self-references are likely correct:
- `/home/agilex/MobileManipulator/src/robot_desc/mobile_manipulator2_description/launch/display_with_hand_cam.launch`
- `/home/agilex/MobileManipulator/src/robot_desc/mobile_manipulator2_description/launch/display.launch`
- `/home/agilex/MobileManipulator/src/robot_desc/mobile_manipulator2_description/launch/display_xacro.launch`
- `/home/agilex/MobileManipulator/src/robot_desc/mobile_manipulator2_description/launch/display_no_gui.launch`
- `/home/agilex/MobileManipulator/src/robot_desc/mobile_manipulator2_description/launch/gazebo.launch`

## Package Dependencies
- `/home/agilex/MobileManipulator/src/arm_planner/package.xml`
  - Line 37: Contains dependency on `mobile_manipulator2_description`

## Shell Scripts
- `/home/agilex/MobileManipulator/scripts/visualize_collision_models.sh`
  - Line 48: References path `/home/agilex/MobileManipulator/src/mobile_manipulator2_description/config/...`

## SRDF Files
- `/home/agilex/MobileManipulator/src/arm_planner/moveit_config/piper_with_gripper/config/_piper_description.srdf`
  - Line 6: Contains robot name `mobile_manipulator2_description`

## Notes:
1. The files in the `robot_desc/mobile_manipulator2_description` directory appear to be correctly placed and their internal references are likely correct as ROS package references.
2. The main concern is with files outside of `robot_desc` that reference the package, as they may need updating if the package has been moved.
3. The build and devel directories contain generated files that will be updated automatically when rebuilding.