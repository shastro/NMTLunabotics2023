; Auto-generated. Do not edit!


(cl:in-package motor_bridge-msg)


;//! \htmlinclude Pitch.msg.html

(cl:defclass <Pitch> (roslisp-msg-protocol:ros-message)
  ((direction
    :reader direction
    :initarg :direction
    :type cl:integer
    :initform 0)
   (motor
    :reader motor
    :initarg :motor
    :type cl:integer
    :initform 0))
)

(cl:defclass Pitch (<Pitch>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Pitch>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Pitch)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name motor_bridge-msg:<Pitch> is deprecated: use motor_bridge-msg:Pitch instead.")))

(cl:ensure-generic-function 'direction-val :lambda-list '(m))
(cl:defmethod direction-val ((m <Pitch>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motor_bridge-msg:direction-val is deprecated.  Use motor_bridge-msg:direction instead.")
  (direction m))

(cl:ensure-generic-function 'motor-val :lambda-list '(m))
(cl:defmethod motor-val ((m <Pitch>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motor_bridge-msg:motor-val is deprecated.  Use motor_bridge-msg:motor instead.")
  (motor m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Pitch>) ostream)
  "Serializes a message object of type '<Pitch>"
  (cl:let* ((signed (cl:slot-value msg 'direction)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Pitch>) istream)
  "Deserializes a message object of type '<Pitch>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'direction) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'motor) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Pitch>)))
  "Returns string type for a message object of type '<Pitch>"
  "motor_bridge/Pitch")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Pitch)))
  "Returns string type for a message object of type 'Pitch"
  "motor_bridge/Pitch")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Pitch>)))
  "Returns md5sum for a message object of type '<Pitch>"
  "8e837e7a28d61ce2adf5af5ddb3d0d20")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Pitch)))
  "Returns md5sum for a message object of type 'Pitch"
  "8e837e7a28d61ce2adf5af5ddb3d0d20")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Pitch>)))
  "Returns full string definition for message of type '<Pitch>"
  (cl:format cl:nil "# Pitch Control Message~%~%# Pitch motion direction: (0 = stop, 1 = extend, 2 = retract)~%int32 direction~%~%# Which motor (0 = both, 1 = left, 2 = right)~%int32 motor~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Pitch)))
  "Returns full string definition for message of type 'Pitch"
  (cl:format cl:nil "# Pitch Control Message~%~%# Pitch motion direction: (0 = stop, 1 = extend, 2 = retract)~%int32 direction~%~%# Which motor (0 = both, 1 = left, 2 = right)~%int32 motor~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Pitch>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Pitch>))
  "Converts a ROS message object to a list"
  (cl:list 'Pitch
    (cl:cons ':direction (direction msg))
    (cl:cons ':motor (motor msg))
))
