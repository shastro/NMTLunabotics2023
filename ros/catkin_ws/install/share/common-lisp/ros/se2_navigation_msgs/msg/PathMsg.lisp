; Auto-generated. Do not edit!


(cl:in-package se2_navigation_msgs-msg)


;//! \htmlinclude PathMsg.msg.html

(cl:defclass <PathMsg> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (segment
    :reader segment
    :initarg :segment
    :type (cl:vector se2_navigation_msgs-msg:PathSegmentMsg)
   :initform (cl:make-array 0 :element-type 'se2_navigation_msgs-msg:PathSegmentMsg :initial-element (cl:make-instance 'se2_navigation_msgs-msg:PathSegmentMsg))))
)

(cl:defclass PathMsg (<PathMsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PathMsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PathMsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name se2_navigation_msgs-msg:<PathMsg> is deprecated: use se2_navigation_msgs-msg:PathMsg instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <PathMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader se2_navigation_msgs-msg:header-val is deprecated.  Use se2_navigation_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'segment-val :lambda-list '(m))
(cl:defmethod segment-val ((m <PathMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader se2_navigation_msgs-msg:segment-val is deprecated.  Use se2_navigation_msgs-msg:segment instead.")
  (segment m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PathMsg>) ostream)
  "Serializes a message object of type '<PathMsg>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'segment))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'segment))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PathMsg>) istream)
  "Deserializes a message object of type '<PathMsg>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'segment) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'segment)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'se2_navigation_msgs-msg:PathSegmentMsg))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PathMsg>)))
  "Returns string type for a message object of type '<PathMsg>"
  "se2_navigation_msgs/PathMsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PathMsg)))
  "Returns string type for a message object of type 'PathMsg"
  "se2_navigation_msgs/PathMsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PathMsg>)))
  "Returns md5sum for a message object of type '<PathMsg>"
  "903b0d0b2bfe4cbcdeb7a06291ea8df4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PathMsg)))
  "Returns md5sum for a message object of type 'PathMsg"
  "903b0d0b2bfe4cbcdeb7a06291ea8df4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PathMsg>)))
  "Returns full string definition for message of type '<PathMsg>"
  (cl:format cl:nil "~%std_msgs/Header header~%se2_navigation_msgs/PathSegmentMsg[] segment~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: se2_navigation_msgs/PathSegmentMsg~%~%~%int8 FORWARD = 0~%int8 BACKWARDS = 1~%~%int8 drivingDirection~%~%~%geometry_msgs/Pose[] points~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PathMsg)))
  "Returns full string definition for message of type 'PathMsg"
  (cl:format cl:nil "~%std_msgs/Header header~%se2_navigation_msgs/PathSegmentMsg[] segment~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: se2_navigation_msgs/PathSegmentMsg~%~%~%int8 FORWARD = 0~%int8 BACKWARDS = 1~%~%int8 drivingDirection~%~%~%geometry_msgs/Pose[] points~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PathMsg>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'segment) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PathMsg>))
  "Converts a ROS message object to a list"
  (cl:list 'PathMsg
    (cl:cons ':header (header msg))
    (cl:cons ':segment (segment msg))
))
