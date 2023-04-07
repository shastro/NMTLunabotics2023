; Auto-generated. Do not edit!


(cl:in-package motor_bridge-msg)


;//! \htmlinclude Drive.msg.html

(cl:defclass <Drive> (roslisp-msg-protocol:ros-message)
  ((speed
    :reader speed
    :initarg :speed
    :type cl:integer
    :initform 0)
   (motor
    :reader motor
    :initarg :motor
    :type cl:integer
    :initform 0))
)

(cl:defclass Drive (<Drive>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Drive>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Drive)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name motor_bridge-msg:<Drive> is deprecated: use motor_bridge-msg:Drive instead.")))

(cl:ensure-generic-function 'speed-val :lambda-list '(m))
(cl:defmethod speed-val ((m <Drive>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motor_bridge-msg:speed-val is deprecated.  Use motor_bridge-msg:speed instead.")
  (speed m))

(cl:ensure-generic-function 'motor-val :lambda-list '(m))
(cl:defmethod motor-val ((m <Drive>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motor_bridge-msg:motor-val is deprecated.  Use motor_bridge-msg:motor instead.")
  (motor m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Drive>) ostream)
  "Serializes a message object of type '<Drive>"
  (cl:let* ((signed (cl:slot-value msg 'speed)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'motor)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Drive>) istream)
  "Deserializes a message object of type '<Drive>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'speed) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'motor) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Drive>)))
  "Returns string type for a message object of type '<Drive>"
  "motor_bridge/Drive")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Drive)))
  "Returns string type for a message object of type 'Drive"
  "motor_bridge/Drive")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Drive>)))
  "Returns md5sum for a message object of type '<Drive>"
  "14b3be7c56f8564b45133d0af1560d8f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Drive)))
  "Returns md5sum for a message object of type 'Drive"
  "14b3be7c56f8564b45133d0af1560d8f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Drive>)))
  "Returns full string definition for message of type '<Drive>"
  (cl:format cl:nil "# Drive Control Message~%~%# Speed Control (Full Backward = -1024, Stop = 0, Full Forward = 1024)~%int32 speed~%~%# Which motor (0 = both, 1 = left, 2 = right)~%int32 motor~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Drive)))
  "Returns full string definition for message of type 'Drive"
  (cl:format cl:nil "# Drive Control Message~%~%# Speed Control (Full Backward = -1024, Stop = 0, Full Forward = 1024)~%int32 speed~%~%# Which motor (0 = both, 1 = left, 2 = right)~%int32 motor~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Drive>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Drive>))
  "Converts a ROS message object to a list"
  (cl:list 'Drive
    (cl:cons ':speed (speed msg))
    (cl:cons ':motor (motor msg))
))
