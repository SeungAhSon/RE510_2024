;; Auto-generated. Do not edit!


(when (boundp 'ros_tutorials::IsClap)
  (if (not (find-package "ROS_TUTORIALS"))
    (make-package "ROS_TUTORIALS"))
  (shadow 'IsClap (find-package "ROS_TUTORIALS")))
(unless (find-package "ROS_TUTORIALS::ISCLAP")
  (make-package "ROS_TUTORIALS::ISCLAP"))

(in-package "ROS")
;;//! \htmlinclude IsClap.msg.html


(defclass ros_tutorials::IsClap
  :super ros::object
  :slots (_data _isclap ))

(defmethod ros_tutorials::IsClap
  (:init
   (&key
    ((:data __data) 0)
    ((:isclap __isclap) "")
    )
   (send-super :init)
   (setq _data (round __data))
   (setq _isclap (string __isclap))
   self)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:isclap
   (&optional __isclap)
   (if __isclap (setq _isclap __isclap)) _isclap)
  (:serialization-length
   ()
   (+
    ;; int32 _data
    4
    ;; string _isclap
    4 (length _isclap)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _data
       (write-long _data s)
     ;; string _isclap
       (write-long (length _isclap) s) (princ _isclap s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _data
     (setq _data (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _isclap
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _isclap (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get ros_tutorials::IsClap :md5sum-) "02936c85ca2c23baf7e87b1b66fbc9e8")
(setf (get ros_tutorials::IsClap :datatype-) "ros_tutorials/IsClap")
(setf (get ros_tutorials::IsClap :definition-)
      "int32 data
string isclap

")



(provide :ros_tutorials/IsClap "02936c85ca2c23baf7e87b1b66fbc9e8")


