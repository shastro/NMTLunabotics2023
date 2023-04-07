; Auto-generated. Do not edit!


(cl:in-package se2_grid_map_generator_msgs-srv)


;//! \htmlinclude UpdateMapPosition-request.msg.html

(cl:defclass <UpdateMapPosition-request> (roslisp-msg-protocol:ros-message)
  ((position
    :reader position
    :initarg :position
    :type se2_grid_map_generator_msgs-msg:Position2D
    :initform (cl:make-instance 'se2_grid_map_generator_msgs-msg:Position2D)))
)

(cl:defclass UpdateMapPosition-request (<UpdateMapPosition-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <UpdateMapPosition-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'UpdateMapPosition-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name se2_grid_map_generator_msgs-srv:<UpdateMapPosition-request> is deprecated: use se2_grid_map_generator_msgs-srv:UpdateMapPosition-request instead.")))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <UpdateMapPosition-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader se2_grid_map_generator_msgs-srv:position-val is deprecated.  Use se2_grid_map_generator_msgs-srv:position instead.")
  (position m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <UpdateMapPosition-request>) ostream)
  "Serializes a message object of type '<UpdateMapPosition-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'position) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <UpdateMapPosition-request>) istream)
  "Deserializes a message object of type '<UpdateMapPosition-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'position) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<UpdateMapPosition-request>)))
  "Returns string type for a service object of type '<UpdateMapPosition-request>"
  "se2_grid_map_generator_msgs/UpdateMapPositionRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'UpdateMapPosition-request)))
  "Returns string type for a service object of type 'UpdateMapPosition-request"
  "se2_grid_map_generator_msgs/UpdateMapPositionRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<UpdateMapPosition-request>)))
  "Returns md5sum for a message object of type '<UpdateMapPosition-request>"
  "ce04429a523a22c93100dc2950508e3d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'UpdateMapPosition-request)))
  "Returns md5sum for a message object of type 'UpdateMapPosition-request"
  "ce04429a523a22c93100dc2950508e3d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<UpdateMapPosition-request>)))
  "Returns full string definition for message of type '<UpdateMapPosition-request>"
  (cl:format cl:nil "# Request~%se2_grid_map_generator_msgs/Position2D position~%~%================================================================================~%MSG: se2_grid_map_generator_msgs/Position2D~%std_msgs/Float64 x~%std_msgs/Float64 y~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'UpdateMapPosition-request)))
  "Returns full string definition for message of type 'UpdateMapPosition-request"
  (cl:format cl:nil "# Request~%se2_grid_map_generator_msgs/Position2D position~%~%================================================================================~%MSG: se2_grid_map_generator_msgs/Position2D~%std_msgs/Float64 x~%std_msgs/Float64 y~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <UpdateMapPosition-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'position))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <UpdateMapPosition-request>))
  "Converts a ROS message object to a list"
  (cl:list 'UpdateMapPosition-request
    (cl:cons ':position (position msg))
))
;//! \htmlinclude UpdateMapPosition-response.msg.html

(cl:defclass <UpdateMapPosition-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass UpdateMapPosition-response (<UpdateMapPosition-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <UpdateMapPosition-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'UpdateMapPosition-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name se2_grid_map_generator_msgs-srv:<UpdateMapPosition-response> is deprecated: use se2_grid_map_generator_msgs-srv:UpdateMapPosition-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <UpdateMapPosition-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader se2_grid_map_generator_msgs-srv:success-val is deprecated.  Use se2_grid_map_generator_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <UpdateMapPosition-response>) ostream)
  "Serializes a message object of type '<UpdateMapPosition-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <UpdateMapPosition-response>) istream)
  "Deserializes a message object of type '<UpdateMapPosition-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<UpdateMapPosition-response>)))
  "Returns string type for a service object of type '<UpdateMapPosition-response>"
  "se2_grid_map_generator_msgs/UpdateMapPositionResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'UpdateMapPosition-response)))
  "Returns string type for a service object of type 'UpdateMapPosition-response"
  "se2_grid_map_generator_msgs/UpdateMapPositionResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<UpdateMapPosition-response>)))
  "Returns md5sum for a message object of type '<UpdateMapPosition-response>"
  "ce04429a523a22c93100dc2950508e3d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'UpdateMapPosition-response)))
  "Returns md5sum for a message object of type 'UpdateMapPosition-response"
  "ce04429a523a22c93100dc2950508e3d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<UpdateMapPosition-response>)))
  "Returns full string definition for message of type '<UpdateMapPosition-response>"
  (cl:format cl:nil "# Response~%bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'UpdateMapPosition-response)))
  "Returns full string definition for message of type 'UpdateMapPosition-response"
  (cl:format cl:nil "# Response~%bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <UpdateMapPosition-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <UpdateMapPosition-response>))
  "Converts a ROS message object to a list"
  (cl:list 'UpdateMapPosition-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'UpdateMapPosition)))
  'UpdateMapPosition-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'UpdateMapPosition)))
  'UpdateMapPosition-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'UpdateMapPosition)))
  "Returns string type for a service object of type '<UpdateMapPosition>"
  "se2_grid_map_generator_msgs/UpdateMapPosition")