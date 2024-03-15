
(cl:in-package :asdf)

(defsystem "virtual_master_device-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "master_dev_state" :depends-on ("_package_master_dev_state"))
    (:file "_package_master_dev_state" :depends-on ("_package"))
  ))