
(cl:in-package :asdf)

(defsystem "task_mgr-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "task_mgr" :depends-on ("_package_task_mgr"))
    (:file "_package_task_mgr" :depends-on ("_package"))
  ))