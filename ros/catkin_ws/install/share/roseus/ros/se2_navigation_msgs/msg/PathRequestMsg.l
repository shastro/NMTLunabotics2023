;; Auto-generated. Do not edit!


(when (boundp 'se2_navigation_msgs::PathRequestMsg)
  (if (not (find-package "SE2_NAVIGATION_MSGS"))
    (make-package "SE2_NAVIGATION_MSGS"))
  (shadow 'PathRequestMsg (find-package "SE2_NAVIGATION_MSGS")))
(unless (find-package "SE2_NAVIGATION_MSGS::PATHREQUESTMSG")
  (make-package "SE2_NAVIGATION_MSGS::PATHREQUESTMSG"))

(in-package "ROS")
;;//! \htmlinclude PathRequestMsg.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass se2_navigation_msgs::PathRequestMsg
  :super ros::object
  :slots (_startingPose _goalPose ))

(defmethod se2_navigation_msgs::PathRequestMsg
  (:init
   (&key
    ((:startingPose __startingPose) (instance geometry_msgs::Pose :init))
    ((:goalPose __goalPose) (instance geometry_msgs::Pose :init))
    )
   (send-super :init)
   (setq _startingPose __startingPose)
   (setq _goalPose __goalPose)
   self)
  (:startingPose
   (&rest __startingPose)
   (if (keywordp (car __startingPose))
       (send* _startingPose __startingPose)
     (progn
       (if __startingPose (setq _startingPose (car __startingPose)))
       _startingPose)))
  (:goalPose
   (&rest __goalPose)
   (if (keywordp (car __goalPose))
       (send* _goalPose __goalPose)
     (progn
       (if __goalPose (setq _goalPose (car __goalPose)))
       _goalPose)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Pose _startingPose
    (send _startingPose :serialization-length)
    ;; geometry_msgs/Pose _goalPose
    (send _goalPose :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Pose _startingPose
       (send _startingPose :serialize s)
     ;; geometry_msgs/Pose _goalPose
       (send _goalPose :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Pose _startingPose
     (send _startingPose :deserialize buf ptr-) (incf ptr- (send _startingPose :serialization-length))
   ;; geometry_msgs/Pose _goalPose
     (send _goalPose :deserialize buf ptr-) (incf ptr- (send _goalPose :serialization-length))
   ;;
   self)
  )

(setf (get se2_navigation_msgs::PathRequestMsg :md5sum-) "e0af408eeb123474bf9e677b140defe0")
(setf (get se2_navigation_msgs::PathRequestMsg :datatype-) "se2_navigation_msgs/PathRequestMsg")
(setf (get se2_navigation_msgs::PathRequestMsg :definition-)
      "geometry_msgs/Pose startingPose
geometry_msgs/Pose goalPose

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



(provide :se2_navigation_msgs/PathRequestMsg "e0af408eeb123474bf9e677b140defe0")


