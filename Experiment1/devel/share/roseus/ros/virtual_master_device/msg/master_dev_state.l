;; Auto-generated. Do not edit!


(when (boundp 'virtual_master_device::master_dev_state)
  (if (not (find-package "VIRTUAL_MASTER_DEVICE"))
    (make-package "VIRTUAL_MASTER_DEVICE"))
  (shadow 'master_dev_state (find-package "VIRTUAL_MASTER_DEVICE")))
(unless (find-package "VIRTUAL_MASTER_DEVICE::MASTER_DEV_STATE")
  (make-package "VIRTUAL_MASTER_DEVICE::MASTER_DEV_STATE"))

(in-package "ROS")
;;//! \htmlinclude master_dev_state.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass virtual_master_device::master_dev_state
  :super ros::object
  :slots (_pos _button ))

(defmethod virtual_master_device::master_dev_state
  (:init
   (&key
    ((:pos __pos) (instance geometry_msgs::PoseStamped :init))
    ((:button __button) nil)
    )
   (send-super :init)
   (setq _pos __pos)
   (setq _button __button)
   self)
  (:pos
   (&rest __pos)
   (if (keywordp (car __pos))
       (send* _pos __pos)
     (progn
       (if __pos (setq _pos (car __pos)))
       _pos)))
  (:button
   (&optional (__button :null))
   (if (not (eq __button :null)) (setq _button __button)) _button)
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/PoseStamped _pos
    (send _pos :serialization-length)
    ;; bool _button
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/PoseStamped _pos
       (send _pos :serialize s)
     ;; bool _button
       (if _button (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/PoseStamped _pos
     (send _pos :deserialize buf ptr-) (incf ptr- (send _pos :serialization-length))
   ;; bool _button
     (setq _button (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get virtual_master_device::master_dev_state :md5sum-) "fd5bdc6b38592bad02082844596f8c6d")
(setf (get virtual_master_device::master_dev_state :datatype-) "virtual_master_device/master_dev_state")
(setf (get virtual_master_device::master_dev_state :definition-)
      "geometry_msgs/PoseStamped pos
bool button


================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

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



(provide :virtual_master_device/master_dev_state "fd5bdc6b38592bad02082844596f8c6d")


