; Auto-generated. Do not edit!


(cl:in-package se2_grid_map_generator_msgs-msg)


;//! \htmlinclude Position2D.msg.html

(cl:defclass <Position2D> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type std_msgs-msg:Float64
    :initform (cl:make-instance 'std_msgs-msg:Float64))
   (y
    :reader y
    :initarg :y
    :type std_msgs-msg:Float64
    :initform (cl:make-instance 'std_msgs-msg:Float64)))
)

(cl:defclass Position2D (<Position2D>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Position2D>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Position2D)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name se2_grid_map_generator_msgs-msg:<Position2D> is deprecated: use se2_grid_map_generator_msgs-msg:Position2D instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <Position2D>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader se2_grid_map_generator_msgs-msg:x-val is deprecated.  Use se2_grid_map_generator_msgs-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <Position2D>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader se2_grid_map_generator_msgs-msg:y-val is deprecated.  Use se2_grid_map_generator_msgs-msg:y instead.")
  (y m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Position2D>) ostream)
  "Serializes a message object of type '<Position2D>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'x) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'y) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Position2D>) istream)
  "Deserializes a message object of type '<Position2D>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'x) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'y) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Position2D>)))
  "Returns string type for a message object of type '<Position2D>"
  "se2_grid_map_generator_msgs/Position2D")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Position2D)))
  "Returns string type for a message object of type 'Position2D"
  "se2_grid_map_generator_msgs/Position2D")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Position2D>)))
  "Returns md5sum for a message object of type '<Position2D>"
  "5507085553fb157248c9d5a62e2bae14")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Position2D)))
  "Returns md5sum for a message object of type 'Position2D"
  "5507085553fb157248c9d5a62e2bae14")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Position2D>)))
  "Returns full string definition for message of type '<Position2D>"
  (cl:format cl:nil "std_msgs/Float64 x~%std_msgs/Float64 y~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Position2D)))
  "Returns full string definition for message of type 'Position2D"
  (cl:format cl:nil "std_msgs/Float64 x~%std_msgs/Float64 y~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Position2D>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'x))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'y))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Position2D>))
  "Converts a ROS message object to a list"
  (cl:list 'Position2D
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
))
