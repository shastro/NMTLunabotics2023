; Auto-generated. Do not edit!


(cl:in-package motor_bridge-msg)


;//! \htmlinclude Digger.msg.html

(cl:defclass <Digger> (roslisp-msg-protocol:ros-message)
  ((speed
    :reader speed
    :initarg :speed
    :type cl:integer
    :initform 0))
)

(cl:defclass Digger (<Digger>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Digger>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Digger)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name motor_bridge-msg:<Digger> is deprecated: use motor_bridge-msg:Digger instead.")))

(cl:ensure-generic-function 'speed-val :lambda-list '(m))
(cl:defmethod speed-val ((m <Digger>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motor_bridge-msg:speed-val is deprecated.  Use motor_bridge-msg:speed instead.")
  (speed m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Digger>) ostream)
  "Serializes a message object of type '<Digger>"
  (cl:let* ((signed (cl:slot-value msg 'speed)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Digger>) istream)
  "Deserializes a message object of type '<Digger>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'speed) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Digger>)))
  "Returns string type for a message object of type '<Digger>"
  "motor_bridge/Digger")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Digger)))
  "Returns string type for a message object of type 'Digger"
  "motor_bridge/Digger")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Digger>)))
  "Returns md5sum for a message object of type '<Digger>"
  "9f7a812be2def1e9db804a7fbea8c3a5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Digger)))
  "Returns md5sum for a message object of type 'Digger"
  "9f7a812be2def1e9db804a7fbea8c3a5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Digger>)))
  "Returns full string definition for message of type '<Digger>"
  (cl:format cl:nil "# Digger Control Message~%~%# Speed Control (Full Backward = -1024, Stop = 0, Full Forward = 1024)~%int32 speed~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Digger)))
  "Returns full string definition for message of type 'Digger"
  (cl:format cl:nil "# Digger Control Message~%~%# Speed Control (Full Backward = -1024, Stop = 0, Full Forward = 1024)~%int32 speed~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Digger>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Digger>))
  "Converts a ROS message object to a list"
  (cl:list 'Digger
    (cl:cons ':speed (speed msg))
))
