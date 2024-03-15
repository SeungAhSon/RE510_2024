; Auto-generated. Do not edit!


(cl:in-package virtual_master_device-msg)


;//! \htmlinclude master_dev_state.msg.html

(cl:defclass <master_dev_state> (roslisp-msg-protocol:ros-message)
  ((pos
    :reader pos
    :initarg :pos
    :type geometry_msgs-msg:PoseStamped
    :initform (cl:make-instance 'geometry_msgs-msg:PoseStamped))
   (button
    :reader button
    :initarg :button
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass master_dev_state (<master_dev_state>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <master_dev_state>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'master_dev_state)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name virtual_master_device-msg:<master_dev_state> is deprecated: use virtual_master_device-msg:master_dev_state instead.")))

(cl:ensure-generic-function 'pos-val :lambda-list '(m))
(cl:defmethod pos-val ((m <master_dev_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader virtual_master_device-msg:pos-val is deprecated.  Use virtual_master_device-msg:pos instead.")
  (pos m))

(cl:ensure-generic-function 'button-val :lambda-list '(m))
(cl:defmethod button-val ((m <master_dev_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader virtual_master_device-msg:button-val is deprecated.  Use virtual_master_device-msg:button instead.")
  (button m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <master_dev_state>) ostream)
  "Serializes a message object of type '<master_dev_state>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pos) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'button) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <master_dev_state>) istream)
  "Deserializes a message object of type '<master_dev_state>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pos) istream)
    (cl:setf (cl:slot-value msg 'button) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<master_dev_state>)))
  "Returns string type for a message object of type '<master_dev_state>"
  "virtual_master_device/master_dev_state")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'master_dev_state)))
  "Returns string type for a message object of type 'master_dev_state"
  "virtual_master_device/master_dev_state")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<master_dev_state>)))
  "Returns md5sum for a message object of type '<master_dev_state>"
  "fd5bdc6b38592bad02082844596f8c6d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'master_dev_state)))
  "Returns md5sum for a message object of type 'master_dev_state"
  "fd5bdc6b38592bad02082844596f8c6d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<master_dev_state>)))
  "Returns full string definition for message of type '<master_dev_state>"
  (cl:format cl:nil "geometry_msgs/PoseStamped pos~%bool button~%~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'master_dev_state)))
  "Returns full string definition for message of type 'master_dev_state"
  (cl:format cl:nil "geometry_msgs/PoseStamped pos~%bool button~%~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <master_dev_state>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pos))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <master_dev_state>))
  "Converts a ROS message object to a list"
  (cl:list 'master_dev_state
    (cl:cons ':pos (pos msg))
    (cl:cons ':button (button msg))
))
