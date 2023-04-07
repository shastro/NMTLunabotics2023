; Auto-generated. Do not edit!


(cl:in-package se2_grid_map_generator_msgs-msg)


;//! \htmlinclude Circle2D.msg.html

(cl:defclass <Circle2D> (roslisp-msg-protocol:ros-message)
  ((center
    :reader center
    :initarg :center
    :type se2_grid_map_generator_msgs-msg:Position2D
    :initform (cl:make-instance 'se2_grid_map_generator_msgs-msg:Position2D))
   (radius
    :reader radius
    :initarg :radius
    :type std_msgs-msg:Float64
    :initform (cl:make-instance 'std_msgs-msg:Float64)))
)

(cl:defclass Circle2D (<Circle2D>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Circle2D>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Circle2D)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name se2_grid_map_generator_msgs-msg:<Circle2D> is deprecated: use se2_grid_map_generator_msgs-msg:Circle2D instead.")))

(cl:ensure-generic-function 'center-val :lambda-list '(m))
(cl:defmethod center-val ((m <Circle2D>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader se2_grid_map_generator_msgs-msg:center-val is deprecated.  Use se2_grid_map_generator_msgs-msg:center instead.")
  (center m))

(cl:ensure-generic-function 'radius-val :lambda-list '(m))
(cl:defmethod radius-val ((m <Circle2D>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader se2_grid_map_generator_msgs-msg:radius-val is deprecated.  Use se2_grid_map_generator_msgs-msg:radius instead.")
  (radius m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Circle2D>) ostream)
  "Serializes a message object of type '<Circle2D>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'center) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'radius) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Circle2D>) istream)
  "Deserializes a message object of type '<Circle2D>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'center) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'radius) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Circle2D>)))
  "Returns string type for a message object of type '<Circle2D>"
  "se2_grid_map_generator_msgs/Circle2D")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Circle2D)))
  "Returns string type for a message object of type 'Circle2D"
  "se2_grid_map_generator_msgs/Circle2D")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Circle2D>)))
  "Returns md5sum for a message object of type '<Circle2D>"
  "0dfece0138cac79dd8ef38f9d1fa9947")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Circle2D)))
  "Returns md5sum for a message object of type 'Circle2D"
  "0dfece0138cac79dd8ef38f9d1fa9947")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Circle2D>)))
  "Returns full string definition for message of type '<Circle2D>"
  (cl:format cl:nil "se2_grid_map_generator_msgs/Position2D center~%std_msgs/Float64 radius~%================================================================================~%MSG: se2_grid_map_generator_msgs/Position2D~%std_msgs/Float64 x~%std_msgs/Float64 y~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Circle2D)))
  "Returns full string definition for message of type 'Circle2D"
  (cl:format cl:nil "se2_grid_map_generator_msgs/Position2D center~%std_msgs/Float64 radius~%================================================================================~%MSG: se2_grid_map_generator_msgs/Position2D~%std_msgs/Float64 x~%std_msgs/Float64 y~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Circle2D>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'center))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'radius))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Circle2D>))
  "Converts a ROS message object to a list"
  (cl:list 'Circle2D
    (cl:cons ':center (center msg))
    (cl:cons ':radius (radius msg))
))
