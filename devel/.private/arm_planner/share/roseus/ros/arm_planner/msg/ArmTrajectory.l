;; Auto-generated. Do not edit!


(when (boundp 'arm_planner::ArmTrajectory)
  (if (not (find-package "ARM_PLANNER"))
    (make-package "ARM_PLANNER"))
  (shadow 'ArmTrajectory (find-package "ARM_PLANNER")))
(unless (find-package "ARM_PLANNER::ARMTRAJECTORY")
  (make-package "ARM_PLANNER::ARMTRAJECTORY"))

(in-package "ROS")
;;//! \htmlinclude ArmTrajectory.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))
(if (not (find-package "TRAJECTORY_MSGS"))
  (ros::roseus-add-msgs "trajectory_msgs"))


(defclass arm_planner::ArmTrajectory
  :super ros::object
  :slots (_header _joint_names _points _end_pose ))

(defmethod arm_planner::ArmTrajectory
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:joint_names __joint_names) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:points __points) ())
    ((:end_pose __end_pose) (instance geometry_msgs::Pose :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _joint_names __joint_names)
   (setq _points __points)
   (setq _end_pose __end_pose)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:joint_names
   (&optional __joint_names)
   (if __joint_names (setq _joint_names __joint_names)) _joint_names)
  (:points
   (&rest __points)
   (if (keywordp (car __points))
       (send* _points __points)
     (progn
       (if __points (setq _points (car __points)))
       _points)))
  (:end_pose
   (&rest __end_pose)
   (if (keywordp (car __end_pose))
       (send* _end_pose __end_pose)
     (progn
       (if __end_pose (setq _end_pose (car __end_pose)))
       _end_pose)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string[] _joint_names
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _joint_names)) 4
    ;; trajectory_msgs/JointTrajectoryPoint[] _points
    (apply #'+ (send-all _points :serialization-length)) 4
    ;; geometry_msgs/Pose _end_pose
    (send _end_pose :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string[] _joint_names
     (write-long (length _joint_names) s)
     (dolist (elem _joint_names)
       (write-long (length elem) s) (princ elem s)
       )
     ;; trajectory_msgs/JointTrajectoryPoint[] _points
     (write-long (length _points) s)
     (dolist (elem _points)
       (send elem :serialize s)
       )
     ;; geometry_msgs/Pose _end_pose
       (send _end_pose :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string[] _joint_names
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _joint_names (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _joint_names i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; trajectory_msgs/JointTrajectoryPoint[] _points
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _points (let (r) (dotimes (i n) (push (instance trajectory_msgs::JointTrajectoryPoint :init) r)) r))
     (dolist (elem- _points)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; geometry_msgs/Pose _end_pose
     (send _end_pose :deserialize buf ptr-) (incf ptr- (send _end_pose :serialization-length))
   ;;
   self)
  )

(setf (get arm_planner::ArmTrajectory :md5sum-) "68689ced2efc48fd92ee34a688a837e9")
(setf (get arm_planner::ArmTrajectory :datatype-) "arm_planner/ArmTrajectory")
(setf (get arm_planner::ArmTrajectory :definition-)
      "Header header
string[] joint_names
trajectory_msgs/JointTrajectoryPoint[] points
geometry_msgs/Pose end_pose
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: trajectory_msgs/JointTrajectoryPoint
# Each trajectory point specifies either positions[, velocities[, accelerations]]
# or positions[, effort] for the trajectory to be executed.
# All specified values are in the same order as the joint names in JointTrajectory.msg

float64[] positions
float64[] velocities
float64[] accelerations
float64[] effort
duration time_from_start

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

")



(provide :arm_planner/ArmTrajectory "68689ced2efc48fd92ee34a688a837e9")


