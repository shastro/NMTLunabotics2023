;; Auto-generated. Do not edit!


(when (boundp 'kindr_msgs::VectorAtPosition)
  (if (not (find-package "KINDR_MSGS"))
    (make-package "KINDR_MSGS"))
  (shadow 'VectorAtPosition (find-package "KINDR_MSGS")))
(unless (find-package "KINDR_MSGS::VECTORATPOSITION")
  (make-package "KINDR_MSGS::VECTORATPOSITION"))

(in-package "ROS")
;;//! \htmlinclude VectorAtPosition.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*TYPE_TYPELESS*" (find-package "KINDR_MSGS::VECTORATPOSITION"))
(shadow '*TYPE_TYPELESS* (find-package "KINDR_MSGS::VECTORATPOSITION"))
(defconstant kindr_msgs::VectorAtPosition::*TYPE_TYPELESS* 0)
(intern "*TYPE_JERK*" (find-package "KINDR_MSGS::VECTORATPOSITION"))
(shadow '*TYPE_JERK* (find-package "KINDR_MSGS::VECTORATPOSITION"))
(defconstant kindr_msgs::VectorAtPosition::*TYPE_JERK* 6)
(intern "*TYPE_ACCELERATION*" (find-package "KINDR_MSGS::VECTORATPOSITION"))
(shadow '*TYPE_ACCELERATION* (find-package "KINDR_MSGS::VECTORATPOSITION"))
(defconstant kindr_msgs::VectorAtPosition::*TYPE_ACCELERATION* 7)
(intern "*TYPE_VELOCITY*" (find-package "KINDR_MSGS::VECTORATPOSITION"))
(shadow '*TYPE_VELOCITY* (find-package "KINDR_MSGS::VECTORATPOSITION"))
(defconstant kindr_msgs::VectorAtPosition::*TYPE_VELOCITY* 8)
(intern "*TYPE_POSITION*" (find-package "KINDR_MSGS::VECTORATPOSITION"))
(shadow '*TYPE_POSITION* (find-package "KINDR_MSGS::VECTORATPOSITION"))
(defconstant kindr_msgs::VectorAtPosition::*TYPE_POSITION* 9)
(intern "*TYPE_FORCE*" (find-package "KINDR_MSGS::VECTORATPOSITION"))
(shadow '*TYPE_FORCE* (find-package "KINDR_MSGS::VECTORATPOSITION"))
(defconstant kindr_msgs::VectorAtPosition::*TYPE_FORCE* 10)
(intern "*TYPE_MOMEMTUM*" (find-package "KINDR_MSGS::VECTORATPOSITION"))
(shadow '*TYPE_MOMEMTUM* (find-package "KINDR_MSGS::VECTORATPOSITION"))
(defconstant kindr_msgs::VectorAtPosition::*TYPE_MOMEMTUM* 11)
(intern "*TYPE_ANGULAR_JERK*" (find-package "KINDR_MSGS::VECTORATPOSITION"))
(shadow '*TYPE_ANGULAR_JERK* (find-package "KINDR_MSGS::VECTORATPOSITION"))
(defconstant kindr_msgs::VectorAtPosition::*TYPE_ANGULAR_JERK* 12)
(intern "*TYPE_ANGULAR_ACCELERATION*" (find-package "KINDR_MSGS::VECTORATPOSITION"))
(shadow '*TYPE_ANGULAR_ACCELERATION* (find-package "KINDR_MSGS::VECTORATPOSITION"))
(defconstant kindr_msgs::VectorAtPosition::*TYPE_ANGULAR_ACCELERATION* 13)
(intern "*TYPE_ANGULAR_VELOCITY*" (find-package "KINDR_MSGS::VECTORATPOSITION"))
(shadow '*TYPE_ANGULAR_VELOCITY* (find-package "KINDR_MSGS::VECTORATPOSITION"))
(defconstant kindr_msgs::VectorAtPosition::*TYPE_ANGULAR_VELOCITY* 14)
(intern "*TYPE_TORQUE*" (find-package "KINDR_MSGS::VECTORATPOSITION"))
(shadow '*TYPE_TORQUE* (find-package "KINDR_MSGS::VECTORATPOSITION"))
(defconstant kindr_msgs::VectorAtPosition::*TYPE_TORQUE* 16)
(intern "*TYPE_ANGULAR_MOMEMTUM*" (find-package "KINDR_MSGS::VECTORATPOSITION"))
(shadow '*TYPE_ANGULAR_MOMEMTUM* (find-package "KINDR_MSGS::VECTORATPOSITION"))
(defconstant kindr_msgs::VectorAtPosition::*TYPE_ANGULAR_MOMEMTUM* 17)

(defun kindr_msgs::VectorAtPosition-to-symbol (const)
  (cond
        ((= const 0) 'kindr_msgs::VectorAtPosition::*TYPE_TYPELESS*)
        ((= const 6) 'kindr_msgs::VectorAtPosition::*TYPE_JERK*)
        ((= const 7) 'kindr_msgs::VectorAtPosition::*TYPE_ACCELERATION*)
        ((= const 8) 'kindr_msgs::VectorAtPosition::*TYPE_VELOCITY*)
        ((= const 9) 'kindr_msgs::VectorAtPosition::*TYPE_POSITION*)
        ((= const 10) 'kindr_msgs::VectorAtPosition::*TYPE_FORCE*)
        ((= const 11) 'kindr_msgs::VectorAtPosition::*TYPE_MOMEMTUM*)
        ((= const 12) 'kindr_msgs::VectorAtPosition::*TYPE_ANGULAR_JERK*)
        ((= const 13) 'kindr_msgs::VectorAtPosition::*TYPE_ANGULAR_ACCELERATION*)
        ((= const 14) 'kindr_msgs::VectorAtPosition::*TYPE_ANGULAR_VELOCITY*)
        ((= const 16) 'kindr_msgs::VectorAtPosition::*TYPE_TORQUE*)
        ((= const 17) 'kindr_msgs::VectorAtPosition::*TYPE_ANGULAR_MOMEMTUM*)
        (t nil)))

(defclass kindr_msgs::VectorAtPosition
  :super ros::object
  :slots (_header _type _name _vector _position _position_frame_id ))

(defmethod kindr_msgs::VectorAtPosition
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:type __type) 0)
    ((:name __name) "")
    ((:vector __vector) (instance geometry_msgs::Vector3 :init))
    ((:position __position) (instance geometry_msgs::Point :init))
    ((:position_frame_id __position_frame_id) "")
    )
   (send-super :init)
   (setq _header __header)
   (setq _type (round __type))
   (setq _name (string __name))
   (setq _vector __vector)
   (setq _position __position)
   (setq _position_frame_id (string __position_frame_id))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:type
   (&optional __type)
   (if __type (setq _type __type)) _type)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:vector
   (&rest __vector)
   (if (keywordp (car __vector))
       (send* _vector __vector)
     (progn
       (if __vector (setq _vector (car __vector)))
       _vector)))
  (:position
   (&rest __position)
   (if (keywordp (car __position))
       (send* _position __position)
     (progn
       (if __position (setq _position (car __position)))
       _position)))
  (:position_frame_id
   (&optional __position_frame_id)
   (if __position_frame_id (setq _position_frame_id __position_frame_id)) _position_frame_id)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint8 _type
    1
    ;; string _name
    4 (length _name)
    ;; geometry_msgs/Vector3 _vector
    (send _vector :serialization-length)
    ;; geometry_msgs/Point _position
    (send _position :serialization-length)
    ;; string _position_frame_id
    4 (length _position_frame_id)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint8 _type
       (write-byte _type s)
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; geometry_msgs/Vector3 _vector
       (send _vector :serialize s)
     ;; geometry_msgs/Point _position
       (send _position :serialize s)
     ;; string _position_frame_id
       (write-long (length _position_frame_id) s) (princ _position_frame_id s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint8 _type
     (setq _type (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; geometry_msgs/Vector3 _vector
     (send _vector :deserialize buf ptr-) (incf ptr- (send _vector :serialization-length))
   ;; geometry_msgs/Point _position
     (send _position :deserialize buf ptr-) (incf ptr- (send _position :serialization-length))
   ;; string _position_frame_id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _position_frame_id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get kindr_msgs::VectorAtPosition :md5sum-) "fcf32a1df9f6d53ef1926f20ce6b66e0")
(setf (get kindr_msgs::VectorAtPosition :datatype-) "kindr_msgs/VectorAtPosition")
(setf (get kindr_msgs::VectorAtPosition :definition-)
      "# Vector type definition (based on Kindr definitions)
uint8 TYPE_TYPELESS=0
uint8 TYPE_JERK=6
uint8 TYPE_ACCELERATION=7
uint8 TYPE_VELOCITY=8
uint8 TYPE_POSITION=9
uint8 TYPE_FORCE=10
uint8 TYPE_MOMEMTUM=11
uint8 TYPE_ANGULAR_JERK=12
uint8 TYPE_ANGULAR_ACCELERATION=13
uint8 TYPE_ANGULAR_VELOCITY=14
uint8 TYPE_TORQUE=16
uint8 TYPE_ANGULAR_MOMEMTUM=17

Header header
uint8 type
string name
geometry_msgs/Vector3 vector # Frame defined in header
geometry_msgs/Point position # Point of origin of the vector
string position_frame_id # If empty same as vector frame

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :kindr_msgs/VectorAtPosition "fcf32a1df9f6d53ef1926f20ce6b66e0")


