; Auto-generated. Do not edit!


(cl:in-package se2_grid_map_generator_msgs-srv)


;//! \htmlinclude AddNan-request.msg.html

(cl:defclass <AddNan-request> (roslisp-msg-protocol:ros-message)
  ((polygon
    :reader polygon
    :initarg :polygon
    :type se2_grid_map_generator_msgs-msg:Polygon2D
    :initform (cl:make-instance 'se2_grid_map_generator_msgs-msg:Polygon2D)))
)

(cl:defclass AddNan-request (<AddNan-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AddNan-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AddNan-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name se2_grid_map_generator_msgs-srv:<AddNan-request> is deprecated: use se2_grid_map_generator_msgs-srv:AddNan-request instead.")))

(cl:ensure-generic-function 'polygon-val :lambda-list '(m))
(cl:defmethod polygon-val ((m <AddNan-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader se2_grid_map_generator_msgs-srv:polygon-val is deprecated.  Use se2_grid_map_generator_msgs-srv:polygon instead.")
  (polygon m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AddNan-request>) ostream)
  "Serializes a message object of type '<AddNan-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'polygon) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AddNan-request>) istream)
  "Deserializes a message object of type '<AddNan-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'polygon) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AddNan-request>)))
  "Returns string type for a service object of type '<AddNan-request>"
  "se2_grid_map_generator_msgs/AddNanRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AddNan-request)))
  "Returns string type for a service object of type 'AddNan-request"
  "se2_grid_map_generator_msgs/AddNanRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AddNan-request>)))
  "Returns md5sum for a message object of type '<AddNan-request>"
  "ed846c130da1159d54d0e3b43ecc2916")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AddNan-request)))
  "Returns md5sum for a message object of type 'AddNan-request"
  "ed846c130da1159d54d0e3b43ecc2916")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AddNan-request>)))
  "Returns full string definition for message of type '<AddNan-request>"
  (cl:format cl:nil "# Request~%se2_grid_map_generator_msgs/Polygon2D polygon~%~%================================================================================~%MSG: se2_grid_map_generator_msgs/Polygon2D~%se2_grid_map_generator_msgs/Position2D[] vertices~%================================================================================~%MSG: se2_grid_map_generator_msgs/Position2D~%std_msgs/Float64 x~%std_msgs/Float64 y~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AddNan-request)))
  "Returns full string definition for message of type 'AddNan-request"
  (cl:format cl:nil "# Request~%se2_grid_map_generator_msgs/Polygon2D polygon~%~%================================================================================~%MSG: se2_grid_map_generator_msgs/Polygon2D~%se2_grid_map_generator_msgs/Position2D[] vertices~%================================================================================~%MSG: se2_grid_map_generator_msgs/Position2D~%std_msgs/Float64 x~%std_msgs/Float64 y~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AddNan-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'polygon))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AddNan-request>))
  "Converts a ROS message object to a list"
  (cl:list 'AddNan-request
    (cl:cons ':polygon (polygon msg))
))
;//! \htmlinclude AddNan-response.msg.html

(cl:defclass <AddNan-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass AddNan-response (<AddNan-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AddNan-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AddNan-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name se2_grid_map_generator_msgs-srv:<AddNan-response> is deprecated: use se2_grid_map_generator_msgs-srv:AddNan-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <AddNan-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader se2_grid_map_generator_msgs-srv:success-val is deprecated.  Use se2_grid_map_generator_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AddNan-response>) ostream)
  "Serializes a message object of type '<AddNan-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AddNan-response>) istream)
  "Deserializes a message object of type '<AddNan-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AddNan-response>)))
  "Returns string type for a service object of type '<AddNan-response>"
  "se2_grid_map_generator_msgs/AddNanResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AddNan-response)))
  "Returns string type for a service object of type 'AddNan-response"
  "se2_grid_map_generator_msgs/AddNanResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AddNan-response>)))
  "Returns md5sum for a message object of type '<AddNan-response>"
  "ed846c130da1159d54d0e3b43ecc2916")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AddNan-response)))
  "Returns md5sum for a message object of type 'AddNan-response"
  "ed846c130da1159d54d0e3b43ecc2916")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AddNan-response>)))
  "Returns full string definition for message of type '<AddNan-response>"
  (cl:format cl:nil "# Response~%bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AddNan-response)))
  "Returns full string definition for message of type 'AddNan-response"
  (cl:format cl:nil "# Response~%bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AddNan-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AddNan-response>))
  "Converts a ROS message object to a list"
  (cl:list 'AddNan-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'AddNan)))
  'AddNan-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'AddNan)))
  'AddNan-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AddNan)))
  "Returns string type for a service object of type '<AddNan>"
  "se2_grid_map_generator_msgs/AddNan")