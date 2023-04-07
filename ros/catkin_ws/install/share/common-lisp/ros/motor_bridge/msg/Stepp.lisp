; Auto-generated. Do not edit!


(cl:in-package motor_bridge-msg)


;//! \htmlinclude Stepp.msg.html

(cl:defclass <Stepp> (roslisp-msg-protocol:ros-message)
  ((rpm
    :reader rpm
    :initarg :rpm
    :type cl:integer
    :initform 0)
   (direction
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

(cl:defclass Stepp (<Stepp>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Stepp>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Stepp)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name motor_bridge-msg:<Stepp> is deprecated: use motor_bridge-msg:Stepp instead.")))

(cl:ensure-generic-function 'rpm-val :lambda-list '(m))
(cl:defmethod rpm-val ((m <Stepp>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motor_bridge-msg:rpm-val is deprecated.  Use motor_bridge-msg:rpm instead.")
  (rpm m))

(cl:ensure-generic-function 'direction-val :lambda-list '(m))
(cl:defmethod direction-val ((m <Stepp>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motor_bridge-msg:direction-val is deprecated.  Use motor_bridge-msg:direction instead.")
  (direction m))

(cl:ensure-generic-function 'motor-val :lambda-list '(m))
(cl:defmethod motor-val ((m <Stepp>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motor_bridge-msg:motor-val is deprecated.  Use motor_bridge-msg:motor instead.")
  (motor m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Stepp>) ostream)
  "Serializes a message object of type '<Stepp>"
  (cl:let* ((signed (cl:slot-value msg 'rpm)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Stepp>) istream)
  "Deserializes a message object of type '<Stepp>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'rpm) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Stepp>)))
  "Returns string type for a message object of type '<Stepp>"
  "motor_bridge/Stepp")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Stepp)))
  "Returns string type for a message object of type 'Stepp"
  "motor_bridge/Stepp")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Stepp>)))
  "Returns md5sum for a message object of type '<Stepp>"
  "43447b26b2e072455822d7bf86b01b32")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Stepp)))
  "Returns md5sum for a message object of type 'Stepp"
  "43447b26b2e072455822d7bf86b01b32")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Stepp>)))
  "Returns full string definition for message of type '<Stepp>"
  (cl:format cl:nil "# Stepper Control Message~%~%# RPM (0 = stop, 1024 = full speed)~%int32 rpm~%~%# Direction (0 = stop, 1 = forward, 2 = backward)~%int32 direction~%~%# Which motor (0 = both, 1 = left, 2 = right)~%int32 motor~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Stepp)))
  "Returns full string definition for message of type 'Stepp"
  (cl:format cl:nil "# Stepper Control Message~%~%# RPM (0 = stop, 1024 = full speed)~%int32 rpm~%~%# Direction (0 = stop, 1 = forward, 2 = backward)~%int32 direction~%~%# Which motor (0 = both, 1 = left, 2 = right)~%int32 motor~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Stepp>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Stepp>))
  "Converts a ROS message object to a list"
  (cl:list 'Stepp
    (cl:cons ':rpm (rpm msg))
    (cl:cons ':direction (direction msg))
    (cl:cons ':motor (motor msg))
))
