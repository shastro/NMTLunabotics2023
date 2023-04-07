; Auto-generated. Do not edit!


(cl:in-package kindr_msgs-msg)


;//! \htmlinclude VectorAtPosition.msg.html

(cl:defclass <VectorAtPosition> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (type
    :reader type
    :initarg :type
    :type cl:fixnum
    :initform 0)
   (name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
   (vector
    :reader vector
    :initarg :vector
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (position
    :reader position
    :initarg :position
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (position_frame_id
    :reader position_frame_id
    :initarg :position_frame_id
    :type cl:string
    :initform ""))
)

(cl:defclass VectorAtPosition (<VectorAtPosition>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <VectorAtPosition>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'VectorAtPosition)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kindr_msgs-msg:<VectorAtPosition> is deprecated: use kindr_msgs-msg:VectorAtPosition instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <VectorAtPosition>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kindr_msgs-msg:header-val is deprecated.  Use kindr_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <VectorAtPosition>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kindr_msgs-msg:type-val is deprecated.  Use kindr_msgs-msg:type instead.")
  (type m))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <VectorAtPosition>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kindr_msgs-msg:name-val is deprecated.  Use kindr_msgs-msg:name instead.")
  (name m))

(cl:ensure-generic-function 'vector-val :lambda-list '(m))
(cl:defmethod vector-val ((m <VectorAtPosition>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kindr_msgs-msg:vector-val is deprecated.  Use kindr_msgs-msg:vector instead.")
  (vector m))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <VectorAtPosition>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kindr_msgs-msg:position-val is deprecated.  Use kindr_msgs-msg:position instead.")
  (position m))

(cl:ensure-generic-function 'position_frame_id-val :lambda-list '(m))
(cl:defmethod position_frame_id-val ((m <VectorAtPosition>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kindr_msgs-msg:position_frame_id-val is deprecated.  Use kindr_msgs-msg:position_frame_id instead.")
  (position_frame_id m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<VectorAtPosition>)))
    "Constants for message type '<VectorAtPosition>"
  '((:TYPE_TYPELESS . 0)
    (:TYPE_JERK . 6)
    (:TYPE_ACCELERATION . 7)
    (:TYPE_VELOCITY . 8)
    (:TYPE_POSITION . 9)
    (:TYPE_FORCE . 10)
    (:TYPE_MOMEMTUM . 11)
    (:TYPE_ANGULAR_JERK . 12)
    (:TYPE_ANGULAR_ACCELERATION . 13)
    (:TYPE_ANGULAR_VELOCITY . 14)
    (:TYPE_TORQUE . 16)
    (:TYPE_ANGULAR_MOMEMTUM . 17))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'VectorAtPosition)))
    "Constants for message type 'VectorAtPosition"
  '((:TYPE_TYPELESS . 0)
    (:TYPE_JERK . 6)
    (:TYPE_ACCELERATION . 7)
    (:TYPE_VELOCITY . 8)
    (:TYPE_POSITION . 9)
    (:TYPE_FORCE . 10)
    (:TYPE_MOMEMTUM . 11)
    (:TYPE_ANGULAR_JERK . 12)
    (:TYPE_ANGULAR_ACCELERATION . 13)
    (:TYPE_ANGULAR_VELOCITY . 14)
    (:TYPE_TORQUE . 16)
    (:TYPE_ANGULAR_MOMEMTUM . 17))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <VectorAtPosition>) ostream)
  "Serializes a message object of type '<VectorAtPosition>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'type)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'vector) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'position) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'position_frame_id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'position_frame_id))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <VectorAtPosition>) istream)
  "Deserializes a message object of type '<VectorAtPosition>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'type)) (cl:read-byte istream))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'vector) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'position) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'position_frame_id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'position_frame_id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<VectorAtPosition>)))
  "Returns string type for a message object of type '<VectorAtPosition>"
  "kindr_msgs/VectorAtPosition")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'VectorAtPosition)))
  "Returns string type for a message object of type 'VectorAtPosition"
  "kindr_msgs/VectorAtPosition")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<VectorAtPosition>)))
  "Returns md5sum for a message object of type '<VectorAtPosition>"
  "fcf32a1df9f6d53ef1926f20ce6b66e0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'VectorAtPosition)))
  "Returns md5sum for a message object of type 'VectorAtPosition"
  "fcf32a1df9f6d53ef1926f20ce6b66e0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<VectorAtPosition>)))
  "Returns full string definition for message of type '<VectorAtPosition>"
  (cl:format cl:nil "# Vector type definition (based on Kindr definitions)~%uint8 TYPE_TYPELESS=0~%uint8 TYPE_JERK=6~%uint8 TYPE_ACCELERATION=7~%uint8 TYPE_VELOCITY=8~%uint8 TYPE_POSITION=9~%uint8 TYPE_FORCE=10~%uint8 TYPE_MOMEMTUM=11~%uint8 TYPE_ANGULAR_JERK=12~%uint8 TYPE_ANGULAR_ACCELERATION=13~%uint8 TYPE_ANGULAR_VELOCITY=14~%uint8 TYPE_TORQUE=16~%uint8 TYPE_ANGULAR_MOMEMTUM=17~%~%Header header~%uint8 type~%string name~%geometry_msgs/Vector3 vector # Frame defined in header~%geometry_msgs/Point position # Point of origin of the vector~%string position_frame_id # If empty same as vector frame~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'VectorAtPosition)))
  "Returns full string definition for message of type 'VectorAtPosition"
  (cl:format cl:nil "# Vector type definition (based on Kindr definitions)~%uint8 TYPE_TYPELESS=0~%uint8 TYPE_JERK=6~%uint8 TYPE_ACCELERATION=7~%uint8 TYPE_VELOCITY=8~%uint8 TYPE_POSITION=9~%uint8 TYPE_FORCE=10~%uint8 TYPE_MOMEMTUM=11~%uint8 TYPE_ANGULAR_JERK=12~%uint8 TYPE_ANGULAR_ACCELERATION=13~%uint8 TYPE_ANGULAR_VELOCITY=14~%uint8 TYPE_TORQUE=16~%uint8 TYPE_ANGULAR_MOMEMTUM=17~%~%Header header~%uint8 type~%string name~%geometry_msgs/Vector3 vector # Frame defined in header~%geometry_msgs/Point position # Point of origin of the vector~%string position_frame_id # If empty same as vector frame~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <VectorAtPosition>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     4 (cl:length (cl:slot-value msg 'name))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'vector))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'position))
     4 (cl:length (cl:slot-value msg 'position_frame_id))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <VectorAtPosition>))
  "Converts a ROS message object to a list"
  (cl:list 'VectorAtPosition
    (cl:cons ':header (header msg))
    (cl:cons ':type (type msg))
    (cl:cons ':name (name msg))
    (cl:cons ':vector (vector msg))
    (cl:cons ':position (position msg))
    (cl:cons ':position_frame_id (position_frame_id msg))
))
