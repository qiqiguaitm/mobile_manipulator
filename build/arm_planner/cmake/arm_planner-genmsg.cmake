# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "arm_planner: 3 messages, 4 services")

set(MSG_I_FLAGS "-Iarm_planner:/home/agilex/MobileManipulator/src/arm_planner/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Itrajectory_msgs:/opt/ros/noetic/share/trajectory_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(arm_planner_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/agilex/MobileManipulator/src/arm_planner/msg/PiperEulerPose.msg" NAME_WE)
add_custom_target(_arm_planner_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "arm_planner" "/home/agilex/MobileManipulator/src/arm_planner/msg/PiperEulerPose.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/agilex/MobileManipulator/src/arm_planner/msg/PiperStatusMsg.msg" NAME_WE)
add_custom_target(_arm_planner_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "arm_planner" "/home/agilex/MobileManipulator/src/arm_planner/msg/PiperStatusMsg.msg" ""
)

get_filename_component(_filename "/home/agilex/MobileManipulator/src/arm_planner/msg/PosCmd.msg" NAME_WE)
add_custom_target(_arm_planner_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "arm_planner" "/home/agilex/MobileManipulator/src/arm_planner/msg/PosCmd.msg" ""
)

get_filename_component(_filename "/home/agilex/MobileManipulator/src/arm_planner/srv/Enable.srv" NAME_WE)
add_custom_target(_arm_planner_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "arm_planner" "/home/agilex/MobileManipulator/src/arm_planner/srv/Enable.srv" ""
)

get_filename_component(_filename "/home/agilex/MobileManipulator/src/arm_planner/srv/GoZero.srv" NAME_WE)
add_custom_target(_arm_planner_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "arm_planner" "/home/agilex/MobileManipulator/src/arm_planner/srv/GoZero.srv" ""
)

get_filename_component(_filename "/home/agilex/MobileManipulator/src/arm_planner/srv/Gripper.srv" NAME_WE)
add_custom_target(_arm_planner_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "arm_planner" "/home/agilex/MobileManipulator/src/arm_planner/srv/Gripper.srv" ""
)

get_filename_component(_filename "/home/agilex/MobileManipulator/src/arm_planner/srv/JointMoveitCtrl.srv" NAME_WE)
add_custom_target(_arm_planner_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "arm_planner" "/home/agilex/MobileManipulator/src/arm_planner/srv/JointMoveitCtrl.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(arm_planner
  "/home/agilex/MobileManipulator/src/arm_planner/msg/PiperEulerPose.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/arm_planner
)
_generate_msg_cpp(arm_planner
  "/home/agilex/MobileManipulator/src/arm_planner/msg/PiperStatusMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/arm_planner
)
_generate_msg_cpp(arm_planner
  "/home/agilex/MobileManipulator/src/arm_planner/msg/PosCmd.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/arm_planner
)

### Generating Services
_generate_srv_cpp(arm_planner
  "/home/agilex/MobileManipulator/src/arm_planner/srv/Enable.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/arm_planner
)
_generate_srv_cpp(arm_planner
  "/home/agilex/MobileManipulator/src/arm_planner/srv/GoZero.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/arm_planner
)
_generate_srv_cpp(arm_planner
  "/home/agilex/MobileManipulator/src/arm_planner/srv/Gripper.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/arm_planner
)
_generate_srv_cpp(arm_planner
  "/home/agilex/MobileManipulator/src/arm_planner/srv/JointMoveitCtrl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/arm_planner
)

### Generating Module File
_generate_module_cpp(arm_planner
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/arm_planner
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(arm_planner_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(arm_planner_generate_messages arm_planner_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/agilex/MobileManipulator/src/arm_planner/msg/PiperEulerPose.msg" NAME_WE)
add_dependencies(arm_planner_generate_messages_cpp _arm_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agilex/MobileManipulator/src/arm_planner/msg/PiperStatusMsg.msg" NAME_WE)
add_dependencies(arm_planner_generate_messages_cpp _arm_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agilex/MobileManipulator/src/arm_planner/msg/PosCmd.msg" NAME_WE)
add_dependencies(arm_planner_generate_messages_cpp _arm_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agilex/MobileManipulator/src/arm_planner/srv/Enable.srv" NAME_WE)
add_dependencies(arm_planner_generate_messages_cpp _arm_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agilex/MobileManipulator/src/arm_planner/srv/GoZero.srv" NAME_WE)
add_dependencies(arm_planner_generate_messages_cpp _arm_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agilex/MobileManipulator/src/arm_planner/srv/Gripper.srv" NAME_WE)
add_dependencies(arm_planner_generate_messages_cpp _arm_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agilex/MobileManipulator/src/arm_planner/srv/JointMoveitCtrl.srv" NAME_WE)
add_dependencies(arm_planner_generate_messages_cpp _arm_planner_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(arm_planner_gencpp)
add_dependencies(arm_planner_gencpp arm_planner_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS arm_planner_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(arm_planner
  "/home/agilex/MobileManipulator/src/arm_planner/msg/PiperEulerPose.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/arm_planner
)
_generate_msg_eus(arm_planner
  "/home/agilex/MobileManipulator/src/arm_planner/msg/PiperStatusMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/arm_planner
)
_generate_msg_eus(arm_planner
  "/home/agilex/MobileManipulator/src/arm_planner/msg/PosCmd.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/arm_planner
)

### Generating Services
_generate_srv_eus(arm_planner
  "/home/agilex/MobileManipulator/src/arm_planner/srv/Enable.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/arm_planner
)
_generate_srv_eus(arm_planner
  "/home/agilex/MobileManipulator/src/arm_planner/srv/GoZero.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/arm_planner
)
_generate_srv_eus(arm_planner
  "/home/agilex/MobileManipulator/src/arm_planner/srv/Gripper.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/arm_planner
)
_generate_srv_eus(arm_planner
  "/home/agilex/MobileManipulator/src/arm_planner/srv/JointMoveitCtrl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/arm_planner
)

### Generating Module File
_generate_module_eus(arm_planner
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/arm_planner
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(arm_planner_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(arm_planner_generate_messages arm_planner_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/agilex/MobileManipulator/src/arm_planner/msg/PiperEulerPose.msg" NAME_WE)
add_dependencies(arm_planner_generate_messages_eus _arm_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agilex/MobileManipulator/src/arm_planner/msg/PiperStatusMsg.msg" NAME_WE)
add_dependencies(arm_planner_generate_messages_eus _arm_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agilex/MobileManipulator/src/arm_planner/msg/PosCmd.msg" NAME_WE)
add_dependencies(arm_planner_generate_messages_eus _arm_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agilex/MobileManipulator/src/arm_planner/srv/Enable.srv" NAME_WE)
add_dependencies(arm_planner_generate_messages_eus _arm_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agilex/MobileManipulator/src/arm_planner/srv/GoZero.srv" NAME_WE)
add_dependencies(arm_planner_generate_messages_eus _arm_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agilex/MobileManipulator/src/arm_planner/srv/Gripper.srv" NAME_WE)
add_dependencies(arm_planner_generate_messages_eus _arm_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agilex/MobileManipulator/src/arm_planner/srv/JointMoveitCtrl.srv" NAME_WE)
add_dependencies(arm_planner_generate_messages_eus _arm_planner_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(arm_planner_geneus)
add_dependencies(arm_planner_geneus arm_planner_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS arm_planner_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(arm_planner
  "/home/agilex/MobileManipulator/src/arm_planner/msg/PiperEulerPose.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/arm_planner
)
_generate_msg_lisp(arm_planner
  "/home/agilex/MobileManipulator/src/arm_planner/msg/PiperStatusMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/arm_planner
)
_generate_msg_lisp(arm_planner
  "/home/agilex/MobileManipulator/src/arm_planner/msg/PosCmd.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/arm_planner
)

### Generating Services
_generate_srv_lisp(arm_planner
  "/home/agilex/MobileManipulator/src/arm_planner/srv/Enable.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/arm_planner
)
_generate_srv_lisp(arm_planner
  "/home/agilex/MobileManipulator/src/arm_planner/srv/GoZero.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/arm_planner
)
_generate_srv_lisp(arm_planner
  "/home/agilex/MobileManipulator/src/arm_planner/srv/Gripper.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/arm_planner
)
_generate_srv_lisp(arm_planner
  "/home/agilex/MobileManipulator/src/arm_planner/srv/JointMoveitCtrl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/arm_planner
)

### Generating Module File
_generate_module_lisp(arm_planner
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/arm_planner
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(arm_planner_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(arm_planner_generate_messages arm_planner_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/agilex/MobileManipulator/src/arm_planner/msg/PiperEulerPose.msg" NAME_WE)
add_dependencies(arm_planner_generate_messages_lisp _arm_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agilex/MobileManipulator/src/arm_planner/msg/PiperStatusMsg.msg" NAME_WE)
add_dependencies(arm_planner_generate_messages_lisp _arm_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agilex/MobileManipulator/src/arm_planner/msg/PosCmd.msg" NAME_WE)
add_dependencies(arm_planner_generate_messages_lisp _arm_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agilex/MobileManipulator/src/arm_planner/srv/Enable.srv" NAME_WE)
add_dependencies(arm_planner_generate_messages_lisp _arm_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agilex/MobileManipulator/src/arm_planner/srv/GoZero.srv" NAME_WE)
add_dependencies(arm_planner_generate_messages_lisp _arm_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agilex/MobileManipulator/src/arm_planner/srv/Gripper.srv" NAME_WE)
add_dependencies(arm_planner_generate_messages_lisp _arm_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agilex/MobileManipulator/src/arm_planner/srv/JointMoveitCtrl.srv" NAME_WE)
add_dependencies(arm_planner_generate_messages_lisp _arm_planner_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(arm_planner_genlisp)
add_dependencies(arm_planner_genlisp arm_planner_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS arm_planner_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(arm_planner
  "/home/agilex/MobileManipulator/src/arm_planner/msg/PiperEulerPose.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/arm_planner
)
_generate_msg_nodejs(arm_planner
  "/home/agilex/MobileManipulator/src/arm_planner/msg/PiperStatusMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/arm_planner
)
_generate_msg_nodejs(arm_planner
  "/home/agilex/MobileManipulator/src/arm_planner/msg/PosCmd.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/arm_planner
)

### Generating Services
_generate_srv_nodejs(arm_planner
  "/home/agilex/MobileManipulator/src/arm_planner/srv/Enable.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/arm_planner
)
_generate_srv_nodejs(arm_planner
  "/home/agilex/MobileManipulator/src/arm_planner/srv/GoZero.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/arm_planner
)
_generate_srv_nodejs(arm_planner
  "/home/agilex/MobileManipulator/src/arm_planner/srv/Gripper.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/arm_planner
)
_generate_srv_nodejs(arm_planner
  "/home/agilex/MobileManipulator/src/arm_planner/srv/JointMoveitCtrl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/arm_planner
)

### Generating Module File
_generate_module_nodejs(arm_planner
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/arm_planner
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(arm_planner_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(arm_planner_generate_messages arm_planner_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/agilex/MobileManipulator/src/arm_planner/msg/PiperEulerPose.msg" NAME_WE)
add_dependencies(arm_planner_generate_messages_nodejs _arm_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agilex/MobileManipulator/src/arm_planner/msg/PiperStatusMsg.msg" NAME_WE)
add_dependencies(arm_planner_generate_messages_nodejs _arm_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agilex/MobileManipulator/src/arm_planner/msg/PosCmd.msg" NAME_WE)
add_dependencies(arm_planner_generate_messages_nodejs _arm_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agilex/MobileManipulator/src/arm_planner/srv/Enable.srv" NAME_WE)
add_dependencies(arm_planner_generate_messages_nodejs _arm_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agilex/MobileManipulator/src/arm_planner/srv/GoZero.srv" NAME_WE)
add_dependencies(arm_planner_generate_messages_nodejs _arm_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agilex/MobileManipulator/src/arm_planner/srv/Gripper.srv" NAME_WE)
add_dependencies(arm_planner_generate_messages_nodejs _arm_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agilex/MobileManipulator/src/arm_planner/srv/JointMoveitCtrl.srv" NAME_WE)
add_dependencies(arm_planner_generate_messages_nodejs _arm_planner_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(arm_planner_gennodejs)
add_dependencies(arm_planner_gennodejs arm_planner_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS arm_planner_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(arm_planner
  "/home/agilex/MobileManipulator/src/arm_planner/msg/PiperEulerPose.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/arm_planner
)
_generate_msg_py(arm_planner
  "/home/agilex/MobileManipulator/src/arm_planner/msg/PiperStatusMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/arm_planner
)
_generate_msg_py(arm_planner
  "/home/agilex/MobileManipulator/src/arm_planner/msg/PosCmd.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/arm_planner
)

### Generating Services
_generate_srv_py(arm_planner
  "/home/agilex/MobileManipulator/src/arm_planner/srv/Enable.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/arm_planner
)
_generate_srv_py(arm_planner
  "/home/agilex/MobileManipulator/src/arm_planner/srv/GoZero.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/arm_planner
)
_generate_srv_py(arm_planner
  "/home/agilex/MobileManipulator/src/arm_planner/srv/Gripper.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/arm_planner
)
_generate_srv_py(arm_planner
  "/home/agilex/MobileManipulator/src/arm_planner/srv/JointMoveitCtrl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/arm_planner
)

### Generating Module File
_generate_module_py(arm_planner
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/arm_planner
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(arm_planner_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(arm_planner_generate_messages arm_planner_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/agilex/MobileManipulator/src/arm_planner/msg/PiperEulerPose.msg" NAME_WE)
add_dependencies(arm_planner_generate_messages_py _arm_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agilex/MobileManipulator/src/arm_planner/msg/PiperStatusMsg.msg" NAME_WE)
add_dependencies(arm_planner_generate_messages_py _arm_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agilex/MobileManipulator/src/arm_planner/msg/PosCmd.msg" NAME_WE)
add_dependencies(arm_planner_generate_messages_py _arm_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agilex/MobileManipulator/src/arm_planner/srv/Enable.srv" NAME_WE)
add_dependencies(arm_planner_generate_messages_py _arm_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agilex/MobileManipulator/src/arm_planner/srv/GoZero.srv" NAME_WE)
add_dependencies(arm_planner_generate_messages_py _arm_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agilex/MobileManipulator/src/arm_planner/srv/Gripper.srv" NAME_WE)
add_dependencies(arm_planner_generate_messages_py _arm_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agilex/MobileManipulator/src/arm_planner/srv/JointMoveitCtrl.srv" NAME_WE)
add_dependencies(arm_planner_generate_messages_py _arm_planner_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(arm_planner_genpy)
add_dependencies(arm_planner_genpy arm_planner_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS arm_planner_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/arm_planner)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/arm_planner
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(arm_planner_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(arm_planner_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET trajectory_msgs_generate_messages_cpp)
  add_dependencies(arm_planner_generate_messages_cpp trajectory_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/arm_planner)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/arm_planner
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(arm_planner_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(arm_planner_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET trajectory_msgs_generate_messages_eus)
  add_dependencies(arm_planner_generate_messages_eus trajectory_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/arm_planner)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/arm_planner
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(arm_planner_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(arm_planner_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET trajectory_msgs_generate_messages_lisp)
  add_dependencies(arm_planner_generate_messages_lisp trajectory_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/arm_planner)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/arm_planner
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(arm_planner_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(arm_planner_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET trajectory_msgs_generate_messages_nodejs)
  add_dependencies(arm_planner_generate_messages_nodejs trajectory_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/arm_planner)
  install(CODE "execute_process(COMMAND \"/home/agilex/miniconda3/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/arm_planner\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/arm_planner
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(arm_planner_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(arm_planner_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET trajectory_msgs_generate_messages_py)
  add_dependencies(arm_planner_generate_messages_py trajectory_msgs_generate_messages_py)
endif()
