; Auto-generated. Do not edit!


(cl:in-package se2_navigation_msgs-msg)


;//! \htmlinclude PathRequestMsg.msg.html

(cl:defclass <PathRequestMsg> (roslisp-msg-protocol:ros-message)
  ((startingPose
    :reader startingPose
    :initarg :startingPose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose))
   (goalPose
    :reader goalPose
    :initarg :goalPose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose)))
)

(cl:defclass PathRequestMsg (<PathRequestMsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PathRequestMsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PathRequestMsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name se2_navigation_msgs-msg:<PathRequestMsg> is deprecated: use se2_navigation_msgs-msg:PathRequestMsg instead.")))

(cl:ensure-generic-function 'startingPose-val :lambda-list '(m))
(cl:defmethod startingPose-val ((m <PathRequestMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader se2_navigation_msgs-msg:startingPose-val is deprecated.  Use se2_navigation_msgs-msg:startingPose instead.")
  (startingPose m))

(cl:ensure-generic-function 'goalPose-val :lambda-list '(m))
(cl:defmethod goalPose-val ((m <PathRequestMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader se2_navigation_msgs-msg:goalPose-val is deprecated.  Use se2_navigation_msgs-msg:goalPose instead.")
  (goalPose m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PathRequestMsg>) ostream)
  "Serializes a message object of type '<PathRequestMsg>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'startingPose) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'goalPose) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PathRequestMsg>) istream)
  "Deserializes a message object of type '<PathRequestMsg>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'startingPose) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'goalPose) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PathRequestMsg>)))
  "Returns string type for a message object of type '<PathRequestMsg>"
  "se2_navigation_msgs/PathRequestMsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PathRequestMsg)))
  "Returns string type for a message object of type 'PathRequestMsg"
  "se2_navigation_msgs/PathRequestMsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PathRequestMsg>)))
  "Returns md5sum for a message object of type '<PathRequestMsg>"
  "e0af408eeb123474bf9e677b140defe0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PathRequestMsg)))
  "Returns md5sum for a message object of type 'PathRequestMsg"
  "e0af408eeb123474bf9e677b140defe0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PathRequestMsg>)))
  "Returns full string definition for message of type '<PathRequestMsg>"
  (cl:format cl:nil "geometry_msgs/Pose startingPose~%geometry_msgs/Pose goalPose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PathRequestMsg)))
  "Returns full string definition for message of type 'PathRequestMsg"
  (cl:format cl:nil "geometry_msgs/Pose startingPose~%geometry_msgs/Pose goalPose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PathRequestMsg>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'startingPose))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'goalPose))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PathRequestMsg>))
  "Converts a ROS message object to a list"
  (cl:list 'PathRequestMsg
    (cl:cons ':startingPose (startingPose msg))
    (cl:cons ':goalPose (goalPose msg))
))
