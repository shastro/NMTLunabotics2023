; Auto-generated. Do not edit!


(cl:in-package se2_navigation_msgs-srv)


;//! \htmlinclude SendControllerCommandSrv-request.msg.html

(cl:defclass <SendControllerCommandSrv-request> (roslisp-msg-protocol:ros-message)
  ((command
    :reader command
    :initarg :command
    :type se2_navigation_msgs-msg:ControllerCommandMsg
    :initform (cl:make-instance 'se2_navigation_msgs-msg:ControllerCommandMsg)))
)

(cl:defclass SendControllerCommandSrv-request (<SendControllerCommandSrv-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SendControllerCommandSrv-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SendControllerCommandSrv-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name se2_navigation_msgs-srv:<SendControllerCommandSrv-request> is deprecated: use se2_navigation_msgs-srv:SendControllerCommandSrv-request instead.")))

(cl:ensure-generic-function 'command-val :lambda-list '(m))
(cl:defmethod command-val ((m <SendControllerCommandSrv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader se2_navigation_msgs-srv:command-val is deprecated.  Use se2_navigation_msgs-srv:command instead.")
  (command m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SendControllerCommandSrv-request>) ostream)
  "Serializes a message object of type '<SendControllerCommandSrv-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'command) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SendControllerCommandSrv-request>) istream)
  "Deserializes a message object of type '<SendControllerCommandSrv-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'command) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SendControllerCommandSrv-request>)))
  "Returns string type for a service object of type '<SendControllerCommandSrv-request>"
  "se2_navigation_msgs/SendControllerCommandSrvRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SendControllerCommandSrv-request)))
  "Returns string type for a service object of type 'SendControllerCommandSrv-request"
  "se2_navigation_msgs/SendControllerCommandSrvRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SendControllerCommandSrv-request>)))
  "Returns md5sum for a message object of type '<SendControllerCommandSrv-request>"
  "8f41fa61e7f9b864f313c51bd6bcb1ba")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SendControllerCommandSrv-request)))
  "Returns md5sum for a message object of type 'SendControllerCommandSrv-request"
  "8f41fa61e7f9b864f313c51bd6bcb1ba")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SendControllerCommandSrv-request>)))
  "Returns full string definition for message of type '<SendControllerCommandSrv-request>"
  (cl:format cl:nil "# Request~%ControllerCommandMsg command                          # Query, maybe useful for the future~%~%================================================================================~%MSG: se2_navigation_msgs/ControllerCommandMsg~%~%int8 START_TRACKING=0~%int8 STOP_TRACKING=1~%~%int8 command~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SendControllerCommandSrv-request)))
  "Returns full string definition for message of type 'SendControllerCommandSrv-request"
  (cl:format cl:nil "# Request~%ControllerCommandMsg command                          # Query, maybe useful for the future~%~%================================================================================~%MSG: se2_navigation_msgs/ControllerCommandMsg~%~%int8 START_TRACKING=0~%int8 STOP_TRACKING=1~%~%int8 command~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SendControllerCommandSrv-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'command))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SendControllerCommandSrv-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SendControllerCommandSrv-request
    (cl:cons ':command (command msg))
))
;//! \htmlinclude SendControllerCommandSrv-response.msg.html

(cl:defclass <SendControllerCommandSrv-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SendControllerCommandSrv-response (<SendControllerCommandSrv-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SendControllerCommandSrv-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SendControllerCommandSrv-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name se2_navigation_msgs-srv:<SendControllerCommandSrv-response> is deprecated: use se2_navigation_msgs-srv:SendControllerCommandSrv-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SendControllerCommandSrv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader se2_navigation_msgs-srv:success-val is deprecated.  Use se2_navigation_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SendControllerCommandSrv-response>) ostream)
  "Serializes a message object of type '<SendControllerCommandSrv-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SendControllerCommandSrv-response>) istream)
  "Deserializes a message object of type '<SendControllerCommandSrv-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SendControllerCommandSrv-response>)))
  "Returns string type for a service object of type '<SendControllerCommandSrv-response>"
  "se2_navigation_msgs/SendControllerCommandSrvResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SendControllerCommandSrv-response)))
  "Returns string type for a service object of type 'SendControllerCommandSrv-response"
  "se2_navigation_msgs/SendControllerCommandSrvResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SendControllerCommandSrv-response>)))
  "Returns md5sum for a message object of type '<SendControllerCommandSrv-response>"
  "8f41fa61e7f9b864f313c51bd6bcb1ba")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SendControllerCommandSrv-response)))
  "Returns md5sum for a message object of type 'SendControllerCommandSrv-response"
  "8f41fa61e7f9b864f313c51bd6bcb1ba")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SendControllerCommandSrv-response>)))
  "Returns full string definition for message of type '<SendControllerCommandSrv-response>"
  (cl:format cl:nil "# Response~%bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SendControllerCommandSrv-response)))
  "Returns full string definition for message of type 'SendControllerCommandSrv-response"
  (cl:format cl:nil "# Response~%bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SendControllerCommandSrv-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SendControllerCommandSrv-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SendControllerCommandSrv-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SendControllerCommandSrv)))
  'SendControllerCommandSrv-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SendControllerCommandSrv)))
  'SendControllerCommandSrv-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SendControllerCommandSrv)))
  "Returns string type for a service object of type '<SendControllerCommandSrv>"
  "se2_navigation_msgs/SendControllerCommandSrv")