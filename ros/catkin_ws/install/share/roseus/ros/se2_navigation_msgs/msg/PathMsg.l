;; Auto-generated. Do not edit!


(when (boundp 'se2_navigation_msgs::PathMsg)
  (if (not (find-package "SE2_NAVIGATION_MSGS"))
    (make-package "SE2_NAVIGATION_MSGS"))
  (shadow 'PathMsg (find-package "SE2_NAVIGATION_MSGS")))
(unless (find-package "SE2_NAVIGATION_MSGS::PATHMSG")
  (make-package "SE2_NAVIGATION_MSGS::PATHMSG"))

(in-package "ROS")
;;//! \htmlinclude PathMsg.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass se2_navigation_msgs::PathMsg
  :super ros::object
  :slots (_header _segment ))

(defmethod se2_navigation_msgs::PathMsg
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:segment __segment) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _segment __segment)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:segment
   (&rest __segment)
   (if (keywordp (car __segment))
       (send* _segment __segment)
     (progn
       (if __segment (setq _segment (car __segment)))
       _segment)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; se2_navigation_msgs/PathSegmentMsg[] _segment
    (apply #'+ (send-all _segment :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; se2_navigation_msgs/PathSegmentMsg[] _segment
     (write-long (length _segment) s)
     (dolist (elem _segment)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; se2_navigation_msgs/PathSegmentMsg[] _segment
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _segment (let (r) (dotimes (i n) (push (instance se2_navigation_msgs::PathSegmentMsg :init) r)) r))
     (dolist (elem- _segment)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get se2_navigation_msgs::PathMsg :md5sum-) "903b0d0b2bfe4cbcdeb7a06291ea8df4")
(setf (get se2_navigation_msgs::PathMsg :datatype-) "se2_navigation_msgs/PathMsg")
(setf (get se2_navigation_msgs::PathMsg :definition-)
      "
std_msgs/Header header
se2_navigation_msgs/PathSegmentMsg[] segment
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
MSG: se2_navigation_msgs/PathSegmentMsg


int8 FORWARD = 0
int8 BACKWARDS = 1

int8 drivingDirection


geometry_msgs/Pose[] points
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

")



(provide :se2_navigation_msgs/PathMsg "903b0d0b2bfe4cbcdeb7a06291ea8df4")


