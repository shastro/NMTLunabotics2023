; Auto-generated. Do not edit!


(cl:in-package se2_navigation_msgs-msg)


;//! \htmlinclude ControllerCommandMsg.msg.html

(cl:defclass <ControllerCommandMsg> (roslisp-msg-protocol:ros-message)
  ((command
    :reader command
    :initarg :command
    :type cl:fixnum
    :initform 0))
)

(cl:defclass ControllerCommandMsg (<ControllerCommandMsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ControllerCommandMsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ControllerCommandMsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name se2_navigation_msgs-msg:<ControllerCommandMsg> is deprecated: use se2_navigation_msgs-msg:ControllerCommandMsg instead.")))

(cl:ensure-generic-function 'command-val :lambda-list '(m))
(cl:defmethod command-val ((m <ControllerCommandMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader se2_navigation_msgs-msg:command-val is deprecated.  Use se2_navigation_msgs-msg:command instead.")
  (command m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<ControllerCommandMsg>)))
    "Constants for message type '<ControllerCommandMsg>"
  '((:START_TRACKING . 0)
    (:STOP_TRACKING . 1))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'ControllerCommandMsg)))
    "Constants for message type 'ControllerCommandMsg"
  '((:START_TRACKING . 0)
    (:STOP_TRACKING . 1))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ControllerCommandMsg>) ostream)
  "Serializes a message object of type '<ControllerCommandMsg>"
  (cl:let* ((signed (cl:slot-value msg 'command)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ControllerCommandMsg>) istream)
  "Deserializes a message object of type '<ControllerCommandMsg>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'command) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ControllerCommandMsg>)))
  "Returns string type for a message object of type '<ControllerCommandMsg>"
  "se2_navigation_msgs/ControllerCommandMsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControllerCommandMsg)))
  "Returns string type for a message object of type 'ControllerCommandMsg"
  "se2_navigation_msgs/ControllerCommandMsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ControllerCommandMsg>)))
  "Returns md5sum for a message object of type '<ControllerCommandMsg>"
  "f666cfd8f3589400e119791c2d52c7ee")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ControllerCommandMsg)))
  "Returns md5sum for a message object of type 'ControllerCommandMsg"
  "f666cfd8f3589400e119791c2d52c7ee")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ControllerCommandMsg>)))
  "Returns full string definition for message of type '<ControllerCommandMsg>"
  (cl:format cl:nil "~%int8 START_TRACKING=0~%int8 STOP_TRACKING=1~%~%int8 command~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ControllerCommandMsg)))
  "Returns full string definition for message of type 'ControllerCommandMsg"
  (cl:format cl:nil "~%int8 START_TRACKING=0~%int8 STOP_TRACKING=1~%~%int8 command~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ControllerCommandMsg>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ControllerCommandMsg>))
  "Converts a ROS message object to a list"
  (cl:list 'ControllerCommandMsg
    (cl:cons ':command (command msg))
))
