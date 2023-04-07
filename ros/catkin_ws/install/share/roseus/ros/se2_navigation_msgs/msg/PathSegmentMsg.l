;; Auto-generated. Do not edit!


(when (boundp 'se2_navigation_msgs::PathSegmentMsg)
  (if (not (find-package "SE2_NAVIGATION_MSGS"))
    (make-package "SE2_NAVIGATION_MSGS"))
  (shadow 'PathSegmentMsg (find-package "SE2_NAVIGATION_MSGS")))
(unless (find-package "SE2_NAVIGATION_MSGS::PATHSEGMENTMSG")
  (make-package "SE2_NAVIGATION_MSGS::PATHSEGMENTMSG"))

(in-package "ROS")
;;//! \htmlinclude PathSegmentMsg.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(intern "*FORWARD*" (find-package "SE2_NAVIGATION_MSGS::PATHSEGMENTMSG"))
(shadow '*FORWARD* (find-package "SE2_NAVIGATION_MSGS::PATHSEGMENTMSG"))
(defconstant se2_navigation_msgs::PathSegmentMsg::*FORWARD* 0)
(intern "*BACKWARDS*" (find-package "SE2_NAVIGATION_MSGS::PATHSEGMENTMSG"))
(shadow '*BACKWARDS* (find-package "SE2_NAVIGATION_MSGS::PATHSEGMENTMSG"))
(defconstant se2_navigation_msgs::PathSegmentMsg::*BACKWARDS* 1)

(defun se2_navigation_msgs::PathSegmentMsg-to-symbol (const)
  (cond
        ((= const 0) 'se2_navigation_msgs::PathSegmentMsg::*FORWARD*)
        ((= const 1) 'se2_navigation_msgs::PathSegmentMsg::*BACKWARDS*)
        (t nil)))

(defclass se2_navigation_msgs::PathSegmentMsg
  :super ros::object
  :slots (_drivingDirection _points ))

(defmethod se2_navigation_msgs::PathSegmentMsg
  (:init
   (&key
    ((:drivingDirection __drivingDirection) 0)
    ((:points __points) ())
    )
   (send-super :init)
   (setq _drivingDirection (round __drivingDirection))
   (setq _points __points)
   self)
  (:drivingDirection
   (&optional __drivingDirection)
   (if __drivingDirection (setq _drivingDirection __drivingDirection)) _drivingDirection)
  (:points
   (&rest __points)
   (if (keywordp (car __points))
       (send* _points __points)
     (progn
       (if __points (setq _points (car __points)))
       _points)))
  (:serialization-length
   ()
   (+
    ;; int8 _drivingDirection
    1
    ;; geometry_msgs/Pose[] _points
    (apply #'+ (send-all _points :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _drivingDirection
       (write-byte _drivingDirection s)
     ;; geometry_msgs/Pose[] _points
     (write-long (length _points) s)
     (dolist (elem _points)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _drivingDirection
     (setq _drivingDirection (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _drivingDirection 127) (setq _drivingDirection (- _drivingDirection 256)))
   ;; geometry_msgs/Pose[] _points
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _points (let (r) (dotimes (i n) (push (instance geometry_msgs::Pose :init) r)) r))
     (dolist (elem- _points)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get se2_navigation_msgs::PathSegmentMsg :md5sum-) "8da26ed105802ea1199162a6750daa5e")
(setf (get se2_navigation_msgs::PathSegmentMsg :datatype-) "se2_navigation_msgs/PathSegmentMsg")
(setf (get se2_navigation_msgs::PathSegmentMsg :definition-)
      "

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



(provide :se2_navigation_msgs/PathSegmentMsg "8da26ed105802ea1199162a6750daa5e")


