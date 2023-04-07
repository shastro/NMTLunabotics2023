; Auto-generated. Do not edit!


(cl:in-package se2_grid_map_generator_msgs-srv)


;//! \htmlinclude AddCircularObstacle-request.msg.html

(cl:defclass <AddCircularObstacle-request> (roslisp-msg-protocol:ros-message)
  ((obstacle
    :reader obstacle
    :initarg :obstacle
    :type se2_grid_map_generator_msgs-msg:CircularObstacle
    :initform (cl:make-instance 'se2_grid_map_generator_msgs-msg:CircularObstacle)))
)

(cl:defclass AddCircularObstacle-request (<AddCircularObstacle-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AddCircularObstacle-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AddCircularObstacle-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name se2_grid_map_generator_msgs-srv:<AddCircularObstacle-request> is deprecated: use se2_grid_map_generator_msgs-srv:AddCircularObstacle-request instead.")))

(cl:ensure-generic-function 'obstacle-val :lambda-list '(m))
(cl:defmethod obstacle-val ((m <AddCircularObstacle-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader se2_grid_map_generator_msgs-srv:obstacle-val is deprecated.  Use se2_grid_map_generator_msgs-srv:obstacle instead.")
  (obstacle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AddCircularObstacle-request>) ostream)
  "Serializes a message object of type '<AddCircularObstacle-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'obstacle) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AddCircularObstacle-request>) istream)
  "Deserializes a message object of type '<AddCircularObstacle-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'obstacle) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AddCircularObstacle-request>)))
  "Returns string type for a service object of type '<AddCircularObstacle-request>"
  "se2_grid_map_generator_msgs/AddCircularObstacleRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AddCircularObstacle-request)))
  "Returns string type for a service object of type 'AddCircularObstacle-request"
  "se2_grid_map_generator_msgs/AddCircularObstacleRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AddCircularObstacle-request>)))
  "Returns md5sum for a message object of type '<AddCircularObstacle-request>"
  "5c6462e7f4d4eb409c7358cc94ad2577")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AddCircularObstacle-request)))
  "Returns md5sum for a message object of type 'AddCircularObstacle-request"
  "5c6462e7f4d4eb409c7358cc94ad2577")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AddCircularObstacle-request>)))
  "Returns full string definition for message of type '<AddCircularObstacle-request>"
  (cl:format cl:nil "# Request~%se2_grid_map_generator_msgs/CircularObstacle obstacle~%~%================================================================================~%MSG: se2_grid_map_generator_msgs/CircularObstacle~%se2_grid_map_generator_msgs/Circle2D circle~%std_msgs/String[] layers~%std_msgs/Float64[] values~%================================================================================~%MSG: se2_grid_map_generator_msgs/Circle2D~%se2_grid_map_generator_msgs/Position2D center~%std_msgs/Float64 radius~%================================================================================~%MSG: se2_grid_map_generator_msgs/Position2D~%std_msgs/Float64 x~%std_msgs/Float64 y~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%================================================================================~%MSG: std_msgs/String~%string data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AddCircularObstacle-request)))
  "Returns full string definition for message of type 'AddCircularObstacle-request"
  (cl:format cl:nil "# Request~%se2_grid_map_generator_msgs/CircularObstacle obstacle~%~%================================================================================~%MSG: se2_grid_map_generator_msgs/CircularObstacle~%se2_grid_map_generator_msgs/Circle2D circle~%std_msgs/String[] layers~%std_msgs/Float64[] values~%================================================================================~%MSG: se2_grid_map_generator_msgs/Circle2D~%se2_grid_map_generator_msgs/Position2D center~%std_msgs/Float64 radius~%================================================================================~%MSG: se2_grid_map_generator_msgs/Position2D~%std_msgs/Float64 x~%std_msgs/Float64 y~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%================================================================================~%MSG: std_msgs/String~%string data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AddCircularObstacle-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'obstacle))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AddCircularObstacle-request>))
  "Converts a ROS message object to a list"
  (cl:list 'AddCircularObstacle-request
    (cl:cons ':obstacle (obstacle msg))
))
;//! \htmlinclude AddCircularObstacle-response.msg.html

(cl:defclass <AddCircularObstacle-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass AddCircularObstacle-response (<AddCircularObstacle-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AddCircularObstacle-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AddCircularObstacle-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name se2_grid_map_generator_msgs-srv:<AddCircularObstacle-response> is deprecated: use se2_grid_map_generator_msgs-srv:AddCircularObstacle-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <AddCircularObstacle-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader se2_grid_map_generator_msgs-srv:success-val is deprecated.  Use se2_grid_map_generator_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AddCircularObstacle-response>) ostream)
  "Serializes a message object of type '<AddCircularObstacle-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AddCircularObstacle-response>) istream)
  "Deserializes a message object of type '<AddCircularObstacle-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AddCircularObstacle-response>)))
  "Returns string type for a service object of type '<AddCircularObstacle-response>"
  "se2_grid_map_generator_msgs/AddCircularObstacleResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AddCircularObstacle-response)))
  "Returns string type for a service object of type 'AddCircularObstacle-response"
  "se2_grid_map_generator_msgs/AddCircularObstacleResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AddCircularObstacle-response>)))
  "Returns md5sum for a message object of type '<AddCircularObstacle-response>"
  "5c6462e7f4d4eb409c7358cc94ad2577")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AddCircularObstacle-response)))
  "Returns md5sum for a message object of type 'AddCircularObstacle-response"
  "5c6462e7f4d4eb409c7358cc94ad2577")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AddCircularObstacle-response>)))
  "Returns full string definition for message of type '<AddCircularObstacle-response>"
  (cl:format cl:nil "# Response~%bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AddCircularObstacle-response)))
  "Returns full string definition for message of type 'AddCircularObstacle-response"
  (cl:format cl:nil "# Response~%bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AddCircularObstacle-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AddCircularObstacle-response>))
  "Converts a ROS message object to a list"
  (cl:list 'AddCircularObstacle-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'AddCircularObstacle)))
  'AddCircularObstacle-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'AddCircularObstacle)))
  'AddCircularObstacle-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AddCircularObstacle)))
  "Returns string type for a service object of type '<AddCircularObstacle>"
  "se2_grid_map_generator_msgs/AddCircularObstacle")