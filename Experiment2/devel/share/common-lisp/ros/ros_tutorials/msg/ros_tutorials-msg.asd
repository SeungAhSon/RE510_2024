
(cl:in-package :asdf)

(defsystem "ros_tutorials-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Counts" :depends-on ("_package_Counts"))
    (:file "_package_Counts" :depends-on ("_package"))
    (:file "IsClap" :depends-on ("_package_IsClap"))
    (:file "_package_IsClap" :depends-on ("_package"))
  ))