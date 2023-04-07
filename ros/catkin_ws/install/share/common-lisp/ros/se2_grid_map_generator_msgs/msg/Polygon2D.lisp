; Auto-generated. Do not edit!


(cl:in-package se2_grid_map_generator_msgs-msg)


;//! \htmlinclude Polygon2D.msg.html

(cl:defclass <Polygon2D> (roslisp-msg-protocol:ros-message)
  ((vertices
    :reader vertices
    :initarg :vertices
    :type (cl:vector se2_grid_map_generator_msgs-msg:Position2D)
   :initform (cl:make-array 0 :element-type 'se2_grid_map_generator_msgs-msg:Position2D :initial-element (cl:make-instance 'se2_grid_map_generator_msgs-msg:Position2D))))
)

(cl:defclass Polygon2D (<Polygon2D>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Polygon2D>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Polygon2D)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name se2_grid_map_generator_msgs-msg:<Polygon2D> is deprecated: use se2_grid_map_generator_msgs-msg:Polygon2D instead.")))

(cl:ensure-generic-function 'vertices-val :lambda-list '(m))
(cl:defmethod vertices-val ((m <Polygon2D>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader se2_grid_map_generator_msgs-msg:vertices-val is deprecated.  Use se2_grid_map_generator_msgs-msg:vertices instead.")
  (vertices m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Polygon2D>) ostream)
  "Serializes a message object of type '<Polygon2D>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'vertices))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'vertices))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Polygon2D>) istream)
  "Deserializes a message object of type '<Polygon2D>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'vertices) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'vertices)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'se2_grid_map_generator_msgs-msg:Position2D))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Polygon2D>)))
  "Returns string type for a message object of type '<Polygon2D>"
  "se2_grid_map_generator_msgs/Polygon2D")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Polygon2D)))
  "Returns string type for a message object of type 'Polygon2D"
  "se2_grid_map_generator_msgs/Polygon2D")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Polygon2D>)))
  "Returns md5sum for a message object of type '<Polygon2D>"
  "bccfee9dbb3170a57e32edc6d3cfcef5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Polygon2D)))
  "Returns md5sum for a message object of type 'Polygon2D"
  "bccfee9dbb3170a57e32edc6d3cfcef5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Polygon2D>)))
  "Returns full string definition for message of type '<Polygon2D>"
  (cl:format cl:nil "se2_grid_map_generator_msgs/Position2D[] vertices~%================================================================================~%MSG: se2_grid_map_generator_msgs/Position2D~%std_msgs/Float64 x~%std_msgs/Float64 y~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Polygon2D)))
  "Returns full string definition for message of type 'Polygon2D"
  (cl:format cl:nil "se2_grid_map_generator_msgs/Position2D[] vertices~%================================================================================~%MSG: se2_grid_map_generator_msgs/Position2D~%std_msgs/Float64 x~%std_msgs/Float64 y~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Polygon2D>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'vertices) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Polygon2D>))
  "Converts a ROS message object to a list"
  (cl:list 'Polygon2D
    (cl:cons ':vertices (vertices msg))
))
