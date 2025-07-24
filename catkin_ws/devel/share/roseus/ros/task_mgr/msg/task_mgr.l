;; Auto-generated. Do not edit!


(when (boundp 'task_mgr::task_mgr)
  (if (not (find-package "TASK_MGR"))
    (make-package "TASK_MGR"))
  (shadow 'task_mgr (find-package "TASK_MGR")))
(unless (find-package "TASK_MGR::TASK_MGR")
  (make-package "TASK_MGR::TASK_MGR"))

(in-package "ROS")
;;//! \htmlinclude task_mgr.msg.html


(defclass task_mgr::task_mgr
  :super ros::object
  :slots (_fsm_state _task_list _cur_task_id _cur_task_name _cur_stage _cur_goal _target_object ))

(defmethod task_mgr::task_mgr
  (:init
   (&key
    ((:fsm_state __fsm_state) "")
    ((:task_list __task_list) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:cur_task_id __cur_task_id) "")
    ((:cur_task_name __cur_task_name) "")
    ((:cur_stage __cur_stage) "")
    ((:cur_goal __cur_goal) "")
    ((:target_object __target_object) "")
    )
   (send-super :init)
   (setq _fsm_state (string __fsm_state))
   (setq _task_list __task_list)
   (setq _cur_task_id (string __cur_task_id))
   (setq _cur_task_name (string __cur_task_name))
   (setq _cur_stage (string __cur_stage))
   (setq _cur_goal (string __cur_goal))
   (setq _target_object (string __target_object))
   self)
  (:fsm_state
   (&optional __fsm_state)
   (if __fsm_state (setq _fsm_state __fsm_state)) _fsm_state)
  (:task_list
   (&optional __task_list)
   (if __task_list (setq _task_list __task_list)) _task_list)
  (:cur_task_id
   (&optional __cur_task_id)
   (if __cur_task_id (setq _cur_task_id __cur_task_id)) _cur_task_id)
  (:cur_task_name
   (&optional __cur_task_name)
   (if __cur_task_name (setq _cur_task_name __cur_task_name)) _cur_task_name)
  (:cur_stage
   (&optional __cur_stage)
   (if __cur_stage (setq _cur_stage __cur_stage)) _cur_stage)
  (:cur_goal
   (&optional __cur_goal)
   (if __cur_goal (setq _cur_goal __cur_goal)) _cur_goal)
  (:target_object
   (&optional __target_object)
   (if __target_object (setq _target_object __target_object)) _target_object)
  (:serialization-length
   ()
   (+
    ;; string _fsm_state
    4 (length _fsm_state)
    ;; string[] _task_list
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _task_list)) 4
    ;; string _cur_task_id
    4 (length _cur_task_id)
    ;; string _cur_task_name
    4 (length _cur_task_name)
    ;; string _cur_stage
    4 (length _cur_stage)
    ;; string _cur_goal
    4 (length _cur_goal)
    ;; string _target_object
    4 (length _target_object)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _fsm_state
       (write-long (length _fsm_state) s) (princ _fsm_state s)
     ;; string[] _task_list
     (write-long (length _task_list) s)
     (dolist (elem _task_list)
       (write-long (length elem) s) (princ elem s)
       )
     ;; string _cur_task_id
       (write-long (length _cur_task_id) s) (princ _cur_task_id s)
     ;; string _cur_task_name
       (write-long (length _cur_task_name) s) (princ _cur_task_name s)
     ;; string _cur_stage
       (write-long (length _cur_stage) s) (princ _cur_stage s)
     ;; string _cur_goal
       (write-long (length _cur_goal) s) (princ _cur_goal s)
     ;; string _target_object
       (write-long (length _target_object) s) (princ _target_object s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _fsm_state
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _fsm_state (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string[] _task_list
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _task_list (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _task_list i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; string _cur_task_id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _cur_task_id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _cur_task_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _cur_task_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _cur_stage
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _cur_stage (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _cur_goal
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _cur_goal (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _target_object
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _target_object (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get task_mgr::task_mgr :md5sum-) "85e6602b4a8e4a88f3d67a09ef6bcc82")
(setf (get task_mgr::task_mgr :datatype-) "task_mgr/task_mgr")
(setf (get task_mgr::task_mgr :definition-)
      "string fsm_state
string[] task_list
string cur_task_id
string cur_task_name
string cur_stage
string cur_goal
string target_object
")



(provide :task_mgr/task_mgr "85e6602b4a8e4a88f3d67a09ef6bcc82")


