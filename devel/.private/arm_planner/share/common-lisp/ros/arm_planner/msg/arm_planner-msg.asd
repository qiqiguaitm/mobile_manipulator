
(cl:in-package :asdf)

(defsystem "arm_planner-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
               :trajectory_msgs-msg
)
  :components ((:file "_package")
    (:file "ArmTrajectory" :depends-on ("_package_ArmTrajectory"))
    (:file "_package_ArmTrajectory" :depends-on ("_package"))
  ))