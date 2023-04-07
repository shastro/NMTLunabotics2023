; Auto-generated. Do not edit!


(cl:in-package motor_bridge-msg)


;//! \htmlinclude Estop.msg.html

(cl:defclass <Estop> (roslisp-msg-protocol:ros-message)
  ((stop
    :reader stop
    :initarg :stop
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Estop (<Estop>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Estop>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Estop)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name motor_bridge-msg:<Estop> is deprecated: use motor_bridge-msg:Estop instead.")))

(cl:ensure-generic-function 'stop-val :lambda-list '(m))
(cl:defmethod stop-val ((m <Estop>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motor_bridge-msg:stop-val is deprecated.  Use motor_bridge-msg:stop instead.")
  (stop m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Estop>) ostream)
  "Serializes a message object of type '<Estop>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'stop) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Estop>) istream)
  "Deserializes a message object of type '<Estop>"
    (cl:setf (cl:slot-value msg 'stop) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Estop>)))
  "Returns string type for a message object of type '<Estop>"
  "motor_bridge/Estop")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Estop)))
  "Returns string type for a message object of type 'Estop"
  "motor_bridge/Estop")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Estop>)))
  "Returns md5sum for a message object of type '<Estop>"
  "71f1172402e56b82716ca71681cded6b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Estop)))
  "Returns md5sum for a message object of type 'Estop"
  "71f1172402e56b82716ca71681cded6b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Estop>)))
  "Returns full string definition for message of type '<Estop>"
  (cl:format cl:nil "# Emergency stop message~%~%# Emergency stop (0 = false, 1 = true)~%bool stop~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Estop)))
  "Returns full string definition for message of type 'Estop"
  (cl:format cl:nil "# Emergency stop message~%~%# Emergency stop (0 = false, 1 = true)~%bool stop~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Estop>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Estop>))
  "Converts a ROS message object to a list"
  (cl:list 'Estop
    (cl:cons ':stop (stop msg))
))
