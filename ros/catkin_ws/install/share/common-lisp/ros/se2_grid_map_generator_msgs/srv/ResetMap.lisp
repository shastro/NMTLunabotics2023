; Auto-generated. Do not edit!


(cl:in-package se2_grid_map_generator_msgs-srv)


;//! \htmlinclude ResetMap-request.msg.html

(cl:defclass <ResetMap-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass ResetMap-request (<ResetMap-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ResetMap-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ResetMap-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name se2_grid_map_generator_msgs-srv:<ResetMap-request> is deprecated: use se2_grid_map_generator_msgs-srv:ResetMap-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ResetMap-request>) ostream)
  "Serializes a message object of type '<ResetMap-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ResetMap-request>) istream)
  "Deserializes a message object of type '<ResetMap-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ResetMap-request>)))
  "Returns string type for a service object of type '<ResetMap-request>"
  "se2_grid_map_generator_msgs/ResetMapRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ResetMap-request)))
  "Returns string type for a service object of type 'ResetMap-request"
  "se2_grid_map_generator_msgs/ResetMapRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ResetMap-request>)))
  "Returns md5sum for a message object of type '<ResetMap-request>"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ResetMap-request)))
  "Returns md5sum for a message object of type 'ResetMap-request"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ResetMap-request>)))
  "Returns full string definition for message of type '<ResetMap-request>"
  (cl:format cl:nil "# Request~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ResetMap-request)))
  "Returns full string definition for message of type 'ResetMap-request"
  (cl:format cl:nil "# Request~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ResetMap-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ResetMap-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ResetMap-request
))
;//! \htmlinclude ResetMap-response.msg.html

(cl:defclass <ResetMap-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass ResetMap-response (<ResetMap-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ResetMap-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ResetMap-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name se2_grid_map_generator_msgs-srv:<ResetMap-response> is deprecated: use se2_grid_map_generator_msgs-srv:ResetMap-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <ResetMap-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader se2_grid_map_generator_msgs-srv:success-val is deprecated.  Use se2_grid_map_generator_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ResetMap-response>) ostream)
  "Serializes a message object of type '<ResetMap-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ResetMap-response>) istream)
  "Deserializes a message object of type '<ResetMap-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ResetMap-response>)))
  "Returns string type for a service object of type '<ResetMap-response>"
  "se2_grid_map_generator_msgs/ResetMapResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ResetMap-response)))
  "Returns string type for a service object of type 'ResetMap-response"
  "se2_grid_map_generator_msgs/ResetMapResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ResetMap-response>)))
  "Returns md5sum for a message object of type '<ResetMap-response>"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ResetMap-response)))
  "Returns md5sum for a message object of type 'ResetMap-response"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ResetMap-response>)))
  "Returns full string definition for message of type '<ResetMap-response>"
  (cl:format cl:nil "# Response~%bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ResetMap-response)))
  "Returns full string definition for message of type 'ResetMap-response"
  (cl:format cl:nil "# Response~%bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ResetMap-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ResetMap-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ResetMap-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ResetMap)))
  'ResetMap-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ResetMap)))
  'ResetMap-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ResetMap)))
  "Returns string type for a service object of type '<ResetMap>"
  "se2_grid_map_generator_msgs/ResetMap")