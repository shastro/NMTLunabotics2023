; Auto-generated. Do not edit!


(cl:in-package se2_grid_map_generator_msgs-srv)


;//! \htmlinclude AddPolygonObstacle-request.msg.html

(cl:defclass <AddPolygonObstacle-request> (roslisp-msg-protocol:ros-message)
  ((obstacle
    :reader obstacle
    :initarg :obstacle
    :type se2_grid_map_generator_msgs-msg:PolygonObstacle
    :initform (cl:make-instance 'se2_grid_map_generator_msgs-msg:PolygonObstacle)))
)

(cl:defclass AddPolygonObstacle-request (<AddPolygonObstacle-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AddPolygonObstacle-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AddPolygonObstacle-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name se2_grid_map_generator_msgs-srv:<AddPolygonObstacle-request> is deprecated: use se2_grid_map_generator_msgs-srv:AddPolygonObstacle-request instead.")))

(cl:ensure-generic-function 'obstacle-val :lambda-list '(m))
(cl:defmethod obstacle-val ((m <AddPolygonObstacle-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader se2_grid_map_generator_msgs-srv:obstacle-val is deprecated.  Use se2_grid_map_generator_msgs-srv:obstacle instead.")
  (obstacle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AddPolygonObstacle-request>) ostream)
  "Serializes a message object of type '<AddPolygonObstacle-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'obstacle) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AddPolygonObstacle-request>) istream)
  "Deserializes a message object of type '<AddPolygonObstacle-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'obstacle) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AddPolygonObstacle-request>)))
  "Returns string type for a service object of type '<AddPolygonObstacle-request>"
  "se2_grid_map_generator_msgs/AddPolygonObstacleRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AddPolygonObstacle-request)))
  "Returns string type for a service object of type 'AddPolygonObstacle-request"
  "se2_grid_map_generator_msgs/AddPolygonObstacleRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AddPolygonObstacle-request>)))
  "Returns md5sum for a message object of type '<AddPolygonObstacle-request>"
  "8cdfa34c81ded8fbfe0b3c21eb0b43e9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AddPolygonObstacle-request)))
  "Returns md5sum for a message object of type 'AddPolygonObstacle-request"
  "8cdfa34c81ded8fbfe0b3c21eb0b43e9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AddPolygonObstacle-request>)))
  "Returns full string definition for message of type '<AddPolygonObstacle-request>"
  (cl:format cl:nil "# Request~%se2_grid_map_generator_msgs/PolygonObstacle obstacle~%~%================================================================================~%MSG: se2_grid_map_generator_msgs/PolygonObstacle~%se2_grid_map_generator_msgs/Polygon2D polygon~%std_msgs/String[] layers~%std_msgs/Float64[] values~%================================================================================~%MSG: se2_grid_map_generator_msgs/Polygon2D~%se2_grid_map_generator_msgs/Position2D[] vertices~%================================================================================~%MSG: se2_grid_map_generator_msgs/Position2D~%std_msgs/Float64 x~%std_msgs/Float64 y~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%================================================================================~%MSG: std_msgs/String~%string data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AddPolygonObstacle-request)))
  "Returns full string definition for message of type 'AddPolygonObstacle-request"
  (cl:format cl:nil "# Request~%se2_grid_map_generator_msgs/PolygonObstacle obstacle~%~%================================================================================~%MSG: se2_grid_map_generator_msgs/PolygonObstacle~%se2_grid_map_generator_msgs/Polygon2D polygon~%std_msgs/String[] layers~%std_msgs/Float64[] values~%================================================================================~%MSG: se2_grid_map_generator_msgs/Polygon2D~%se2_grid_map_generator_msgs/Position2D[] vertices~%================================================================================~%MSG: se2_grid_map_generator_msgs/Position2D~%std_msgs/Float64 x~%std_msgs/Float64 y~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%================================================================================~%MSG: std_msgs/String~%string data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AddPolygonObstacle-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'obstacle))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AddPolygonObstacle-request>))
  "Converts a ROS message object to a list"
  (cl:list 'AddPolygonObstacle-request
    (cl:cons ':obstacle (obstacle msg))
))
;//! \htmlinclude AddPolygonObstacle-response.msg.html

(cl:defclass <AddPolygonObstacle-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass AddPolygonObstacle-response (<AddPolygonObstacle-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AddPolygonObstacle-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AddPolygonObstacle-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name se2_grid_map_generator_msgs-srv:<AddPolygonObstacle-response> is deprecated: use se2_grid_map_generator_msgs-srv:AddPolygonObstacle-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <AddPolygonObstacle-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader se2_grid_map_generator_msgs-srv:success-val is deprecated.  Use se2_grid_map_generator_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AddPolygonObstacle-response>) ostream)
  "Serializes a message object of type '<AddPolygonObstacle-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AddPolygonObstacle-response>) istream)
  "Deserializes a message object of type '<AddPolygonObstacle-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AddPolygonObstacle-response>)))
  "Returns string type for a service object of type '<AddPolygonObstacle-response>"
  "se2_grid_map_generator_msgs/AddPolygonObstacleResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AddPolygonObstacle-response)))
  "Returns string type for a service object of type 'AddPolygonObstacle-response"
  "se2_grid_map_generator_msgs/AddPolygonObstacleResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AddPolygonObstacle-response>)))
  "Returns md5sum for a message object of type '<AddPolygonObstacle-response>"
  "8cdfa34c81ded8fbfe0b3c21eb0b43e9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AddPolygonObstacle-response)))
  "Returns md5sum for a message object of type 'AddPolygonObstacle-response"
  "8cdfa34c81ded8fbfe0b3c21eb0b43e9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AddPolygonObstacle-response>)))
  "Returns full string definition for message of type '<AddPolygonObstacle-response>"
  (cl:format cl:nil "# Response~%bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AddPolygonObstacle-response)))
  "Returns full string definition for message of type 'AddPolygonObstacle-response"
  (cl:format cl:nil "# Response~%bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AddPolygonObstacle-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AddPolygonObstacle-response>))
  "Converts a ROS message object to a list"
  (cl:list 'AddPolygonObstacle-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'AddPolygonObstacle)))
  'AddPolygonObstacle-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'AddPolygonObstacle)))
  'AddPolygonObstacle-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AddPolygonObstacle)))
  "Returns string type for a service object of type '<AddPolygonObstacle>"
  "se2_grid_map_generator_msgs/AddPolygonObstacle")