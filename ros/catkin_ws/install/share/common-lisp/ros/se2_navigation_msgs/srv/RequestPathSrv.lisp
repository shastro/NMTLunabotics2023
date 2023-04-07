; Auto-generated. Do not edit!


(cl:in-package se2_navigation_msgs-srv)


;//! \htmlinclude RequestPathSrv-request.msg.html

(cl:defclass <RequestPathSrv-request> (roslisp-msg-protocol:ros-message)
  ((pathRequest
    :reader pathRequest
    :initarg :pathRequest
    :type se2_navigation_msgs-msg:PathRequestMsg
    :initform (cl:make-instance 'se2_navigation_msgs-msg:PathRequestMsg)))
)

(cl:defclass RequestPathSrv-request (<RequestPathSrv-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RequestPathSrv-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RequestPathSrv-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name se2_navigation_msgs-srv:<RequestPathSrv-request> is deprecated: use se2_navigation_msgs-srv:RequestPathSrv-request instead.")))

(cl:ensure-generic-function 'pathRequest-val :lambda-list '(m))
(cl:defmethod pathRequest-val ((m <RequestPathSrv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader se2_navigation_msgs-srv:pathRequest-val is deprecated.  Use se2_navigation_msgs-srv:pathRequest instead.")
  (pathRequest m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RequestPathSrv-request>) ostream)
  "Serializes a message object of type '<RequestPathSrv-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pathRequest) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RequestPathSrv-request>) istream)
  "Deserializes a message object of type '<RequestPathSrv-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pathRequest) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RequestPathSrv-request>)))
  "Returns string type for a service object of type '<RequestPathSrv-request>"
  "se2_navigation_msgs/RequestPathSrvRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RequestPathSrv-request)))
  "Returns string type for a service object of type 'RequestPathSrv-request"
  "se2_navigation_msgs/RequestPathSrvRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RequestPathSrv-request>)))
  "Returns md5sum for a message object of type '<RequestPathSrv-request>"
  "5539d8cfd1e0b02f407ce0465be5beeb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RequestPathSrv-request)))
  "Returns md5sum for a message object of type 'RequestPathSrv-request"
  "5539d8cfd1e0b02f407ce0465be5beeb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RequestPathSrv-request>)))
  "Returns full string definition for message of type '<RequestPathSrv-request>"
  (cl:format cl:nil "# Request~%PathRequestMsg 		pathRequest                          # Query, maybe useful for the future~%~%================================================================================~%MSG: se2_navigation_msgs/PathRequestMsg~%geometry_msgs/Pose startingPose~%geometry_msgs/Pose goalPose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RequestPathSrv-request)))
  "Returns full string definition for message of type 'RequestPathSrv-request"
  (cl:format cl:nil "# Request~%PathRequestMsg 		pathRequest                          # Query, maybe useful for the future~%~%================================================================================~%MSG: se2_navigation_msgs/PathRequestMsg~%geometry_msgs/Pose startingPose~%geometry_msgs/Pose goalPose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RequestPathSrv-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pathRequest))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RequestPathSrv-request>))
  "Converts a ROS message object to a list"
  (cl:list 'RequestPathSrv-request
    (cl:cons ':pathRequest (pathRequest msg))
))
;//! \htmlinclude RequestPathSrv-response.msg.html

(cl:defclass <RequestPathSrv-response> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass RequestPathSrv-response (<RequestPathSrv-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RequestPathSrv-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RequestPathSrv-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name se2_navigation_msgs-srv:<RequestPathSrv-response> is deprecated: use se2_navigation_msgs-srv:RequestPathSrv-response instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <RequestPathSrv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader se2_navigation_msgs-srv:status-val is deprecated.  Use se2_navigation_msgs-srv:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RequestPathSrv-response>) ostream)
  "Serializes a message object of type '<RequestPathSrv-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'status) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RequestPathSrv-response>) istream)
  "Deserializes a message object of type '<RequestPathSrv-response>"
    (cl:setf (cl:slot-value msg 'status) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RequestPathSrv-response>)))
  "Returns string type for a service object of type '<RequestPathSrv-response>"
  "se2_navigation_msgs/RequestPathSrvResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RequestPathSrv-response)))
  "Returns string type for a service object of type 'RequestPathSrv-response"
  "se2_navigation_msgs/RequestPathSrvResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RequestPathSrv-response>)))
  "Returns md5sum for a message object of type '<RequestPathSrv-response>"
  "5539d8cfd1e0b02f407ce0465be5beeb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RequestPathSrv-response)))
  "Returns md5sum for a message object of type 'RequestPathSrv-response"
  "5539d8cfd1e0b02f407ce0465be5beeb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RequestPathSrv-response>)))
  "Returns full string definition for message of type '<RequestPathSrv-response>"
  (cl:format cl:nil "# Response~%bool status~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RequestPathSrv-response)))
  "Returns full string definition for message of type 'RequestPathSrv-response"
  (cl:format cl:nil "# Response~%bool status~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RequestPathSrv-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RequestPathSrv-response>))
  "Converts a ROS message object to a list"
  (cl:list 'RequestPathSrv-response
    (cl:cons ':status (status msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'RequestPathSrv)))
  'RequestPathSrv-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'RequestPathSrv)))
  'RequestPathSrv-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RequestPathSrv)))
  "Returns string type for a service object of type '<RequestPathSrv>"
  "se2_navigation_msgs/RequestPathSrv")