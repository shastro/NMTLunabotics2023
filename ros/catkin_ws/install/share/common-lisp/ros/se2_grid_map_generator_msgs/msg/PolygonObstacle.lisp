; Auto-generated. Do not edit!


(cl:in-package se2_grid_map_generator_msgs-msg)


;//! \htmlinclude PolygonObstacle.msg.html

(cl:defclass <PolygonObstacle> (roslisp-msg-protocol:ros-message)
  ((polygon
    :reader polygon
    :initarg :polygon
    :type se2_grid_map_generator_msgs-msg:Polygon2D
    :initform (cl:make-instance 'se2_grid_map_generator_msgs-msg:Polygon2D))
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

(cl:defclass PolygonObstacle (<PolygonObstacle>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PolygonObstacle>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PolygonObstacle)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name se2_grid_map_generator_msgs-msg:<PolygonObstacle> is deprecated: use se2_grid_map_generator_msgs-msg:PolygonObstacle instead.")))

(cl:ensure-generic-function 'polygon-val :lambda-list '(m))
(cl:defmethod polygon-val ((m <PolygonObstacle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader se2_grid_map_generator_msgs-msg:polygon-val is deprecated.  Use se2_grid_map_generator_msgs-msg:polygon instead.")
  (polygon m))

(cl:ensure-generic-function 'layers-val :lambda-list '(m))
(cl:defmethod layers-val ((m <PolygonObstacle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader se2_grid_map_generator_msgs-msg:layers-val is deprecated.  Use se2_grid_map_generator_msgs-msg:layers instead.")
  (layers m))

(cl:ensure-generic-function 'values-val :lambda-list '(m))
(cl:defmethod values-val ((m <PolygonObstacle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader se2_grid_map_generator_msgs-msg:values-val is deprecated.  Use se2_grid_map_generator_msgs-msg:values instead.")
  (values m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PolygonObstacle>) ostream)
  "Serializes a message object of type '<PolygonObstacle>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'polygon) ostream)
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PolygonObstacle>) istream)
  "Deserializes a message object of type '<PolygonObstacle>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'polygon) istream)
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PolygonObstacle>)))
  "Returns string type for a message object of type '<PolygonObstacle>"
  "se2_grid_map_generator_msgs/PolygonObstacle")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PolygonObstacle)))
  "Returns string type for a message object of type 'PolygonObstacle"
  "se2_grid_map_generator_msgs/PolygonObstacle")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PolygonObstacle>)))
  "Returns md5sum for a message object of type '<PolygonObstacle>"
  "c82e92cc64d3dead3fb26d6715443026")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PolygonObstacle)))
  "Returns md5sum for a message object of type 'PolygonObstacle"
  "c82e92cc64d3dead3fb26d6715443026")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PolygonObstacle>)))
  "Returns full string definition for message of type '<PolygonObstacle>"
  (cl:format cl:nil "se2_grid_map_generator_msgs/Polygon2D polygon~%std_msgs/String[] layers~%std_msgs/Float64[] values~%================================================================================~%MSG: se2_grid_map_generator_msgs/Polygon2D~%se2_grid_map_generator_msgs/Position2D[] vertices~%================================================================================~%MSG: se2_grid_map_generator_msgs/Position2D~%std_msgs/Float64 x~%std_msgs/Float64 y~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%================================================================================~%MSG: std_msgs/String~%string data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PolygonObstacle)))
  "Returns full string definition for message of type 'PolygonObstacle"
  (cl:format cl:nil "se2_grid_map_generator_msgs/Polygon2D polygon~%std_msgs/String[] layers~%std_msgs/Float64[] values~%================================================================================~%MSG: se2_grid_map_generator_msgs/Polygon2D~%se2_grid_map_generator_msgs/Position2D[] vertices~%================================================================================~%MSG: se2_grid_map_generator_msgs/Position2D~%std_msgs/Float64 x~%std_msgs/Float64 y~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%================================================================================~%MSG: std_msgs/String~%string data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PolygonObstacle>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'polygon))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'layers) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'values) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PolygonObstacle>))
  "Converts a ROS message object to a list"
  (cl:list 'PolygonObstacle
    (cl:cons ':polygon (polygon msg))
    (cl:cons ':layers (layers msg))
    (cl:cons ':values (values msg))
))
