; Auto-generated. Do not edit!


(cl:in-package se2_grid_map_generator_msgs-msg)


;//! \htmlinclude CircularObstacle.msg.html

(cl:defclass <CircularObstacle> (roslisp-msg-protocol:ros-message)
  ((circle
    :reader circle
    :initarg :circle
    :type se2_grid_map_generator_msgs-msg:Circle2D
    :initform (cl:make-instance 'se2_grid_map_generator_msgs-msg:Circle2D))
   (layers
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

(cl:defclass CircularObstacle (<CircularObstacle>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CircularObstacle>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CircularObstacle)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name se2_grid_map_generator_msgs-msg:<CircularObstacle> is deprecated: use se2_grid_map_generator_msgs-msg:CircularObstacle instead.")))

(cl:ensure-generic-function 'circle-val :lambda-list '(m))
(cl:defmethod circle-val ((m <CircularObstacle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader se2_grid_map_generator_msgs-msg:circle-val is deprecated.  Use se2_grid_map_generator_msgs-msg:circle instead.")
  (circle m))

(cl:ensure-generic-function 'layers-val :lambda-list '(m))
(cl:defmethod layers-val ((m <CircularObstacle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader se2_grid_map_generator_msgs-msg:layers-val is deprecated.  Use se2_grid_map_generator_msgs-msg:layers instead.")
  (layers m))

(cl:ensure-generic-function 'values-val :lambda-list '(m))
(cl:defmethod values-val ((m <CircularObstacle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader se2_grid_map_generator_msgs-msg:values-val is deprecated.  Use se2_grid_map_generator_msgs-msg:values instead.")
  (values m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CircularObstacle>) ostream)
  "Serializes a message object of type '<CircularObstacle>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'circle) ostream)
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CircularObstacle>) istream)
  "Deserializes a message object of type '<CircularObstacle>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'circle) istream)
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CircularObstacle>)))
  "Returns string type for a message object of type '<CircularObstacle>"
  "se2_grid_map_generator_msgs/CircularObstacle")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CircularObstacle)))
  "Returns string type for a message object of type 'CircularObstacle"
  "se2_grid_map_generator_msgs/CircularObstacle")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CircularObstacle>)))
  "Returns md5sum for a message object of type '<CircularObstacle>"
  "d3e815e4c24596dee9474400db6e2dc7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CircularObstacle)))
  "Returns md5sum for a message object of type 'CircularObstacle"
  "d3e815e4c24596dee9474400db6e2dc7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CircularObstacle>)))
  "Returns full string definition for message of type '<CircularObstacle>"
  (cl:format cl:nil "se2_grid_map_generator_msgs/Circle2D circle~%std_msgs/String[] layers~%std_msgs/Float64[] values~%================================================================================~%MSG: se2_grid_map_generator_msgs/Circle2D~%se2_grid_map_generator_msgs/Position2D center~%std_msgs/Float64 radius~%================================================================================~%MSG: se2_grid_map_generator_msgs/Position2D~%std_msgs/Float64 x~%std_msgs/Float64 y~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%================================================================================~%MSG: std_msgs/String~%string data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CircularObstacle)))
  "Returns full string definition for message of type 'CircularObstacle"
  (cl:format cl:nil "se2_grid_map_generator_msgs/Circle2D circle~%std_msgs/String[] layers~%std_msgs/Float64[] values~%================================================================================~%MSG: se2_grid_map_generator_msgs/Circle2D~%se2_grid_map_generator_msgs/Position2D center~%std_msgs/Float64 radius~%================================================================================~%MSG: se2_grid_map_generator_msgs/Position2D~%std_msgs/Float64 x~%std_msgs/Float64 y~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%================================================================================~%MSG: std_msgs/String~%string data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CircularObstacle>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'circle))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'layers) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'values) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CircularObstacle>))
  "Converts a ROS message object to a list"
  (cl:list 'CircularObstacle
    (cl:cons ':circle (circle msg))
    (cl:cons ':layers (layers msg))
    (cl:cons ':values (values msg))
))
