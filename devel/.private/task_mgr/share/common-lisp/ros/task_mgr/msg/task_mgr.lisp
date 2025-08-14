; Auto-generated. Do not edit!


(cl:in-package task_mgr-msg)


;//! \htmlinclude task_mgr.msg.html

(cl:defclass <task_mgr> (roslisp-msg-protocol:ros-message)
  ((fsm_state
    :reader fsm_state
    :initarg :fsm_state
    :type cl:string
    :initform "")
   (task_list
    :reader task_list
    :initarg :task_list
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element ""))
   (cur_task_id
    :reader cur_task_id
    :initarg :cur_task_id
    :type cl:string
    :initform "")
   (cur_task_name
    :reader cur_task_name
    :initarg :cur_task_name
    :type cl:string
    :initform "")
   (cur_stage
    :reader cur_stage
    :initarg :cur_stage
    :type cl:string
    :initform "")
   (cur_goal
    :reader cur_goal
    :initarg :cur_goal
    :type cl:string
    :initform "")
   (target_object
    :reader target_object
    :initarg :target_object
    :type cl:string
    :initform ""))
)

(cl:defclass task_mgr (<task_mgr>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <task_mgr>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'task_mgr)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name task_mgr-msg:<task_mgr> is deprecated: use task_mgr-msg:task_mgr instead.")))

(cl:ensure-generic-function 'fsm_state-val :lambda-list '(m))
(cl:defmethod fsm_state-val ((m <task_mgr>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader task_mgr-msg:fsm_state-val is deprecated.  Use task_mgr-msg:fsm_state instead.")
  (fsm_state m))

(cl:ensure-generic-function 'task_list-val :lambda-list '(m))
(cl:defmethod task_list-val ((m <task_mgr>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader task_mgr-msg:task_list-val is deprecated.  Use task_mgr-msg:task_list instead.")
  (task_list m))

(cl:ensure-generic-function 'cur_task_id-val :lambda-list '(m))
(cl:defmethod cur_task_id-val ((m <task_mgr>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader task_mgr-msg:cur_task_id-val is deprecated.  Use task_mgr-msg:cur_task_id instead.")
  (cur_task_id m))

(cl:ensure-generic-function 'cur_task_name-val :lambda-list '(m))
(cl:defmethod cur_task_name-val ((m <task_mgr>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader task_mgr-msg:cur_task_name-val is deprecated.  Use task_mgr-msg:cur_task_name instead.")
  (cur_task_name m))

(cl:ensure-generic-function 'cur_stage-val :lambda-list '(m))
(cl:defmethod cur_stage-val ((m <task_mgr>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader task_mgr-msg:cur_stage-val is deprecated.  Use task_mgr-msg:cur_stage instead.")
  (cur_stage m))

(cl:ensure-generic-function 'cur_goal-val :lambda-list '(m))
(cl:defmethod cur_goal-val ((m <task_mgr>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader task_mgr-msg:cur_goal-val is deprecated.  Use task_mgr-msg:cur_goal instead.")
  (cur_goal m))

(cl:ensure-generic-function 'target_object-val :lambda-list '(m))
(cl:defmethod target_object-val ((m <task_mgr>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader task_mgr-msg:target_object-val is deprecated.  Use task_mgr-msg:target_object instead.")
  (target_object m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <task_mgr>) ostream)
  "Serializes a message object of type '<task_mgr>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'fsm_state))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'fsm_state))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'task_list))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'task_list))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'cur_task_id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'cur_task_id))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'cur_task_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'cur_task_name))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'cur_stage))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'cur_stage))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'cur_goal))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'cur_goal))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'target_object))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'target_object))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <task_mgr>) istream)
  "Deserializes a message object of type '<task_mgr>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'fsm_state) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'fsm_state) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'task_list) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'task_list)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'cur_task_id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'cur_task_id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'cur_task_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'cur_task_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'cur_stage) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'cur_stage) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'cur_goal) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'cur_goal) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'target_object) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'target_object) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<task_mgr>)))
  "Returns string type for a message object of type '<task_mgr>"
  "task_mgr/task_mgr")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'task_mgr)))
  "Returns string type for a message object of type 'task_mgr"
  "task_mgr/task_mgr")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<task_mgr>)))
  "Returns md5sum for a message object of type '<task_mgr>"
  "85e6602b4a8e4a88f3d67a09ef6bcc82")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'task_mgr)))
  "Returns md5sum for a message object of type 'task_mgr"
  "85e6602b4a8e4a88f3d67a09ef6bcc82")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<task_mgr>)))
  "Returns full string definition for message of type '<task_mgr>"
  (cl:format cl:nil "string fsm_state~%string[] task_list~%string cur_task_id~%string cur_task_name~%string cur_stage~%string cur_goal~%string target_object~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'task_mgr)))
  "Returns full string definition for message of type 'task_mgr"
  (cl:format cl:nil "string fsm_state~%string[] task_list~%string cur_task_id~%string cur_task_name~%string cur_stage~%string cur_goal~%string target_object~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <task_mgr>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'fsm_state))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'task_list) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
     4 (cl:length (cl:slot-value msg 'cur_task_id))
     4 (cl:length (cl:slot-value msg 'cur_task_name))
     4 (cl:length (cl:slot-value msg 'cur_stage))
     4 (cl:length (cl:slot-value msg 'cur_goal))
     4 (cl:length (cl:slot-value msg 'target_object))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <task_mgr>))
  "Converts a ROS message object to a list"
  (cl:list 'task_mgr
    (cl:cons ':fsm_state (fsm_state msg))
    (cl:cons ':task_list (task_list msg))
    (cl:cons ':cur_task_id (cur_task_id msg))
    (cl:cons ':cur_task_name (cur_task_name msg))
    (cl:cons ':cur_stage (cur_stage msg))
    (cl:cons ':cur_goal (cur_goal msg))
    (cl:cons ':target_object (target_object msg))
))
