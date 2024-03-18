; Auto-generated. Do not edit!


(cl:in-package ros_tutorials-msg)


;//! \htmlinclude IsClap.msg.html

(cl:defclass <IsClap> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type cl:integer
    :initform 0)
   (isclap
    :reader isclap
    :initarg :isclap
    :type cl:string
    :initform ""))
)

(cl:defclass IsClap (<IsClap>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <IsClap>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'IsClap)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ros_tutorials-msg:<IsClap> is deprecated: use ros_tutorials-msg:IsClap instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <IsClap>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_tutorials-msg:data-val is deprecated.  Use ros_tutorials-msg:data instead.")
  (data m))

(cl:ensure-generic-function 'isclap-val :lambda-list '(m))
(cl:defmethod isclap-val ((m <IsClap>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_tutorials-msg:isclap-val is deprecated.  Use ros_tutorials-msg:isclap instead.")
  (isclap m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <IsClap>) ostream)
  "Serializes a message object of type '<IsClap>"
  (cl:let* ((signed (cl:slot-value msg 'data)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'isclap))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'isclap))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <IsClap>) istream)
  "Deserializes a message object of type '<IsClap>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'data) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'isclap) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'isclap) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<IsClap>)))
  "Returns string type for a message object of type '<IsClap>"
  "ros_tutorials/IsClap")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'IsClap)))
  "Returns string type for a message object of type 'IsClap"
  "ros_tutorials/IsClap")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<IsClap>)))
  "Returns md5sum for a message object of type '<IsClap>"
  "02936c85ca2c23baf7e87b1b66fbc9e8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'IsClap)))
  "Returns md5sum for a message object of type 'IsClap"
  "02936c85ca2c23baf7e87b1b66fbc9e8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<IsClap>)))
  "Returns full string definition for message of type '<IsClap>"
  (cl:format cl:nil "int32 data~%string isclap~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'IsClap)))
  "Returns full string definition for message of type 'IsClap"
  (cl:format cl:nil "int32 data~%string isclap~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <IsClap>))
  (cl:+ 0
     4
     4 (cl:length (cl:slot-value msg 'isclap))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <IsClap>))
  "Converts a ROS message object to a list"
  (cl:list 'IsClap
    (cl:cons ':data (data msg))
    (cl:cons ':isclap (isclap msg))
))
