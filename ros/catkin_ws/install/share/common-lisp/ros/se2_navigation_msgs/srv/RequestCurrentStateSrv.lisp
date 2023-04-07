; Auto-generated. Do not edit!


(cl:in-package se2_navigation_msgs-srv)


;//! \htmlinclude RequestCurrentStateSrv-request.msg.html

(cl:defclass <RequestCurrentStateSrv-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass RequestCurrentStateSrv-request (<RequestCurrentStateSrv-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RequestCurrentStateSrv-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RequestCurrentStateSrv-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name se2_navigation_msgs-srv:<RequestCurrentStateSrv-request> is deprecated: use se2_navigation_msgs-srv:RequestCurrentStateSrv-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RequestCurrentStateSrv-request>) ostream)
  "Serializes a message object of type '<RequestCurrentStateSrv-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RequestCurrentStateSrv-request>) istream)
  "Deserializes a message object of type '<RequestCurrentStateSrv-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RequestCurrentStateSrv-request>)))
  "Returns string type for a service object of type '<RequestCurrentStateSrv-request>"
  "se2_navigation_msgs/RequestCurrentStateSrvRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RequestCurrentStateSrv-request)))
  "Returns string type for a service object of type 'RequestCurrentStateSrv-request"
  "se2_navigation_msgs/RequestCurrentStateSrvRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RequestCurrentStateSrv-request>)))
  "Returns md5sum for a message object of type '<RequestCurrentStateSrv-request>"
  "c79f0d88a7597db980a56d7ac144c654")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RequestCurrentStateSrv-request)))
  "Returns md5sum for a message object of type 'RequestCurrentStateSrv-request"
  "c79f0d88a7597db980a56d7ac144c654")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RequestCurrentStateSrv-request>)))
  "Returns full string definition for message of type '<RequestCurrentStateSrv-request>"
  (cl:format cl:nil "# Request~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RequestCurrentStateSrv-request)))
  "Returns full string definition for message of type 'RequestCurrentStateSrv-request"
  (cl:format cl:nil "# Request~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RequestCurrentStateSrv-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RequestCurrentStateSrv-request>))
  "Converts a ROS message object to a list"
  (cl:list 'RequestCurrentStateSrv-request
))
;//! \htmlinclude RequestCurrentStateSrv-response.msg.html

(cl:defclass <RequestCurrentStateSrv-response> (roslisp-msg-protocol:ros-message)
  ((pose
    :reader pose
    :initarg :pose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose))
   (twist
    :reader twist
    :initarg :twist
    :type geometry_msgs-msg:Twist
    :initform (cl:make-instance 'geometry_msgs-msg:Twist)))
)

(cl:defclass RequestCurrentStateSrv-response (<RequestCurrentStateSrv-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RequestCurrentStateSrv-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RequestCurrentStateSrv-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name se2_navigation_msgs-srv:<RequestCurrentStateSrv-response> is deprecated: use se2_navigation_msgs-srv:RequestCurrentStateSrv-response instead.")))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <RequestCurrentStateSrv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader se2_navigation_msgs-srv:pose-val is deprecated.  Use se2_navigation_msgs-srv:pose instead.")
  (pose m))

(cl:ensure-generic-function 'twist-val :lambda-list '(m))
(cl:defmethod twist-val ((m <RequestCurrentStateSrv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader se2_navigation_msgs-srv:twist-val is deprecated.  Use se2_navigation_msgs-srv:twist instead.")
  (twist m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RequestCurrentStateSrv-response>) ostream)
  "Serializes a message object of type '<RequestCurrentStateSrv-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'twist) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RequestCurrentStateSrv-response>) istream)
  "Deserializes a message object of type '<RequestCurrentStateSrv-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'twist) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RequestCurrentStateSrv-response>)))
  "Returns string type for a service object of type '<RequestCurrentStateSrv-response>"
  "se2_navigation_msgs/RequestCurrentStateSrvResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RequestCurrentStateSrv-response)))
  "Returns string type for a service object of type 'RequestCurrentStateSrv-response"
  "se2_navigation_msgs/RequestCurrentStateSrvResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RequestCurrentStateSrv-response>)))
  "Returns md5sum for a message object of type '<RequestCurrentStateSrv-response>"
  "c79f0d88a7597db980a56d7ac144c654")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RequestCurrentStateSrv-response)))
  "Returns md5sum for a message object of type 'RequestCurrentStateSrv-response"
  "c79f0d88a7597db980a56d7ac144c654")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RequestCurrentStateSrv-response>)))
  "Returns full string definition for message of type '<RequestCurrentStateSrv-response>"
  (cl:format cl:nil "# Response~%geometry_msgs/Pose pose~%geometry_msgs/Twist twist~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RequestCurrentStateSrv-response)))
  "Returns full string definition for message of type 'RequestCurrentStateSrv-response"
  (cl:format cl:nil "# Response~%geometry_msgs/Pose pose~%geometry_msgs/Twist twist~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RequestCurrentStateSrv-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'twist))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RequestCurrentStateSrv-response>))
  "Converts a ROS message object to a list"
  (cl:list 'RequestCurrentStateSrv-response
    (cl:cons ':pose (pose msg))
    (cl:cons ':twist (twist msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'RequestCurrentStateSrv)))
  'RequestCurrentStateSrv-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'RequestCurrentStateSrv)))
  'RequestCurrentStateSrv-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RequestCurrentStateSrv)))
  "Returns string type for a service object of type '<RequestCurrentStateSrv>"
  "se2_navigation_msgs/RequestCurrentStateSrv")