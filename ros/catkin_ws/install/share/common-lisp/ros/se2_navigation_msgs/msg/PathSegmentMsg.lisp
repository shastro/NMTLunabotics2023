; Auto-generated. Do not edit!


(cl:in-package se2_navigation_msgs-msg)


;//! \htmlinclude PathSegmentMsg.msg.html

(cl:defclass <PathSegmentMsg> (roslisp-msg-protocol:ros-message)
  ((drivingDirection
    :reader drivingDirection
    :initarg :drivingDirection
    :type cl:fixnum
    :initform 0)
   (points
    :reader points
    :initarg :points
    :type (cl:vector geometry_msgs-msg:Pose)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Pose :initial-element (cl:make-instance 'geometry_msgs-msg:Pose))))
)

(cl:defclass PathSegmentMsg (<PathSegmentMsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PathSegmentMsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PathSegmentMsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name se2_navigation_msgs-msg:<PathSegmentMsg> is deprecated: use se2_navigation_msgs-msg:PathSegmentMsg instead.")))

(cl:ensure-generic-function 'drivingDirection-val :lambda-list '(m))
(cl:defmethod drivingDirection-val ((m <PathSegmentMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader se2_navigation_msgs-msg:drivingDirection-val is deprecated.  Use se2_navigation_msgs-msg:drivingDirection instead.")
  (drivingDirection m))

(cl:ensure-generic-function 'points-val :lambda-list '(m))
(cl:defmethod points-val ((m <PathSegmentMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader se2_navigation_msgs-msg:points-val is deprecated.  Use se2_navigation_msgs-msg:points instead.")
  (points m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<PathSegmentMsg>)))
    "Constants for message type '<PathSegmentMsg>"
  '((:FORWARD . 0)
    (:BACKWARDS . 1))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'PathSegmentMsg)))
    "Constants for message type 'PathSegmentMsg"
  '((:FORWARD . 0)
    (:BACKWARDS . 1))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PathSegmentMsg>) ostream)
  "Serializes a message object of type '<PathSegmentMsg>"
  (cl:let* ((signed (cl:slot-value msg 'drivingDirection)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'points))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'points))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PathSegmentMsg>) istream)
  "Deserializes a message object of type '<PathSegmentMsg>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'drivingDirection) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'points) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'points)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Pose))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PathSegmentMsg>)))
  "Returns string type for a message object of type '<PathSegmentMsg>"
  "se2_navigation_msgs/PathSegmentMsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PathSegmentMsg)))
  "Returns string type for a message object of type 'PathSegmentMsg"
  "se2_navigation_msgs/PathSegmentMsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PathSegmentMsg>)))
  "Returns md5sum for a message object of type '<PathSegmentMsg>"
  "8da26ed105802ea1199162a6750daa5e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PathSegmentMsg)))
  "Returns md5sum for a message object of type 'PathSegmentMsg"
  "8da26ed105802ea1199162a6750daa5e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PathSegmentMsg>)))
  "Returns full string definition for message of type '<PathSegmentMsg>"
  (cl:format cl:nil "~%~%int8 FORWARD = 0~%int8 BACKWARDS = 1~%~%int8 drivingDirection~%~%~%geometry_msgs/Pose[] points~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PathSegmentMsg)))
  "Returns full string definition for message of type 'PathSegmentMsg"
  (cl:format cl:nil "~%~%int8 FORWARD = 0~%int8 BACKWARDS = 1~%~%int8 drivingDirection~%~%~%geometry_msgs/Pose[] points~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PathSegmentMsg>))
  (cl:+ 0
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'points) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PathSegmentMsg>))
  "Converts a ROS message object to a list"
  (cl:list 'PathSegmentMsg
    (cl:cons ':drivingDirection (drivingDirection msg))
    (cl:cons ':points (points msg))
))
