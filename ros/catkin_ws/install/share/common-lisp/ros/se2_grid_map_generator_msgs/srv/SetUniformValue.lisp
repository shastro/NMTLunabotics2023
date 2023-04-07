; Auto-generated. Do not edit!


(cl:in-package se2_grid_map_generator_msgs-srv)


;//! \htmlinclude SetUniformValue-request.msg.html

(cl:defclass <SetUniformValue-request> (roslisp-msg-protocol:ros-message)
  ((layers
    :reader layers
    :initarg :layers
    :type (cl:vector std_msgs-msg:String)
   :initform (cl:make-array 0 :element-type 'std_msgs-msg:String :initial-element (cl:make-instance 'std_msgs-msg:String)))
   (values
    :reader values
    :initarg :values
    :type (cl:vector std_msgs-msg:Float64)
   :initform (cl:make-array 0 :element-type 'std_msgs-msg:Float64 :initial-element (cl:make-instance 'std_msgs-msg:Float64))))
)

(cl:defclass SetUniformValue-request (<SetUniformValue-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetUniformValue-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetUniformValue-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name se2_grid_map_generator_msgs-srv:<SetUniformValue-request> is deprecated: use se2_grid_map_generator_msgs-srv:SetUniformValue-request instead.")))

(cl:ensure-generic-function 'layers-val :lambda-list '(m))
(cl:defmethod layers-val ((m <SetUniformValue-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader se2_grid_map_generator_msgs-srv:layers-val is deprecated.  Use se2_grid_map_generator_msgs-srv:layers instead.")
  (layers m))

(cl:ensure-generic-function 'values-val :lambda-list '(m))
(cl:defmethod values-val ((m <SetUniformValue-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader se2_grid_map_generator_msgs-srv:values-val is deprecated.  Use se2_grid_map_generator_msgs-srv:values instead.")
  (values m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetUniformValue-request>) ostream)
  "Serializes a message object of type '<SetUniformValue-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'layers))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'layers))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'values))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'values))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetUniformValue-request>) istream)
  "Deserializes a message object of type '<SetUniformValue-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'layers) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'layers)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'std_msgs-msg:String))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'values) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'values)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'std_msgs-msg:Float64))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetUniformValue-request>)))
  "Returns string type for a service object of type '<SetUniformValue-request>"
  "se2_grid_map_generator_msgs/SetUniformValueRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetUniformValue-request)))
  "Returns string type for a service object of type 'SetUniformValue-request"
  "se2_grid_map_generator_msgs/SetUniformValueRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetUniformValue-request>)))
  "Returns md5sum for a message object of type '<SetUniformValue-request>"
  "e0f18e88964ccbda0238df7be68a4fd8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetUniformValue-request)))
  "Returns md5sum for a message object of type 'SetUniformValue-request"
  "e0f18e88964ccbda0238df7be68a4fd8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetUniformValue-request>)))
  "Returns full string definition for message of type '<SetUniformValue-request>"
  (cl:format cl:nil "# Request~%std_msgs/String[] layers~%std_msgs/Float64[] values~%~%================================================================================~%MSG: std_msgs/String~%string data~%~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetUniformValue-request)))
  "Returns full string definition for message of type 'SetUniformValue-request"
  (cl:format cl:nil "# Request~%std_msgs/String[] layers~%std_msgs/Float64[] values~%~%================================================================================~%MSG: std_msgs/String~%string data~%~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetUniformValue-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'layers) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'values) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetUniformValue-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetUniformValue-request
    (cl:cons ':layers (layers msg))
    (cl:cons ':values (values msg))
))
;//! \htmlinclude SetUniformValue-response.msg.html

(cl:defclass <SetUniformValue-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetUniformValue-response (<SetUniformValue-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetUniformValue-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetUniformValue-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name se2_grid_map_generator_msgs-srv:<SetUniformValue-response> is deprecated: use se2_grid_map_generator_msgs-srv:SetUniformValue-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetUniformValue-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader se2_grid_map_generator_msgs-srv:success-val is deprecated.  Use se2_grid_map_generator_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetUniformValue-response>) ostream)
  "Serializes a message object of type '<SetUniformValue-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetUniformValue-response>) istream)
  "Deserializes a message object of type '<SetUniformValue-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetUniformValue-response>)))
  "Returns string type for a service object of type '<SetUniformValue-response>"
  "se2_grid_map_generator_msgs/SetUniformValueResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetUniformValue-response)))
  "Returns string type for a service object of type 'SetUniformValue-response"
  "se2_grid_map_generator_msgs/SetUniformValueResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetUniformValue-response>)))
  "Returns md5sum for a message object of type '<SetUniformValue-response>"
  "e0f18e88964ccbda0238df7be68a4fd8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetUniformValue-response)))
  "Returns md5sum for a message object of type 'SetUniformValue-response"
  "e0f18e88964ccbda0238df7be68a4fd8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetUniformValue-response>)))
  "Returns full string definition for message of type '<SetUniformValue-response>"
  (cl:format cl:nil "# Response~%bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetUniformValue-response)))
  "Returns full string definition for message of type 'SetUniformValue-response"
  (cl:format cl:nil "# Response~%bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetUniformValue-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetUniformValue-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetUniformValue-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetUniformValue)))
  'SetUniformValue-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetUniformValue)))
  'SetUniformValue-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetUniformValue)))
  "Returns string type for a service object of type '<SetUniformValue>"
  "se2_grid_map_generator_msgs/SetUniformValue")