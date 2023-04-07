;; Auto-generated. Do not edit!


(when (boundp 'se2_navigation_msgs::RequestCurrentStateSrv)
  (if (not (find-package "SE2_NAVIGATION_MSGS"))
    (make-package "SE2_NAVIGATION_MSGS"))
  (shadow 'RequestCurrentStateSrv (find-package "SE2_NAVIGATION_MSGS")))
(unless (find-package "SE2_NAVIGATION_MSGS::REQUESTCURRENTSTATESRV")
  (make-package "SE2_NAVIGATION_MSGS::REQUESTCURRENTSTATESRV"))
(unless (find-package "SE2_NAVIGATION_MSGS::REQUESTCURRENTSTATESRVREQUEST")
  (make-package "SE2_NAVIGATION_MSGS::REQUESTCURRENTSTATESRVREQUEST"))
(unless (find-package "SE2_NAVIGATION_MSGS::REQUESTCURRENTSTATESRVRESPONSE")
  (make-package "SE2_NAVIGATION_MSGS::REQUESTCURRENTSTATESRVRESPONSE"))

(in-package "ROS")



(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass se2_navigation_msgs::RequestCurrentStateSrvRequest
  :super ros::object
  :slots ())

(defmethod se2_navigation_msgs::RequestCurrentStateSrvRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass se2_navigation_msgs::RequestCurrentStateSrvResponse
  :super ros::object
  :slots (_pose _twist ))

(defmethod se2_navigation_msgs::RequestCurrentStateSrvResponse
  (:init
   (&key
    ((:pose __pose) (instance geometry_msgs::Pose :init))
    ((:twist __twist) (instance geometry_msgs::Twist :init))
    )
   (send-super :init)
   (setq _pose __pose)
   (setq _twist __twist)
   self)
  (:pose
   (&rest __pose)
   (if (keywordp (car __pose))
       (send* _pose __pose)
     (progn
       (if __pose (setq _pose (car __pose)))
       _pose)))
  (:twist
   (&rest __twist)
   (if (keywordp (car __twist))
       (send* _twist __twist)
     (progn
       (if __twist (setq _twist (car __twist)))
       _twist)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Pose _pose
    (send _pose :serialization-length)
    ;; geometry_msgs/Twist _twist
    (send _twist :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Pose _pose
       (send _pose :serialize s)
     ;; geometry_msgs/Twist _twist
       (send _twist :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Pose _pose
     (send _pose :deserialize buf ptr-) (incf ptr- (send _pose :serialization-length))
   ;; geometry_msgs/Twist _twist
     (send _twist :deserialize buf ptr-) (incf ptr- (send _twist :serialization-length))
   ;;
   self)
  )

(defclass se2_navigation_msgs::RequestCurrentStateSrv
  :super ros::object
  :slots ())

(setf (get se2_navigation_msgs::RequestCurrentStateSrv :md5sum-) "c79f0d88a7597db980a56d7ac144c654")
(setf (get se2_navigation_msgs::RequestCurrentStateSrv :datatype-) "se2_navigation_msgs/RequestCurrentStateSrv")
(setf (get se2_navigation_msgs::RequestCurrentStateSrv :request) se2_navigation_msgs::RequestCurrentStateSrvRequest)
(setf (get se2_navigation_msgs::RequestCurrentStateSrv :response) se2_navigation_msgs::RequestCurrentStateSrvResponse)

(defmethod se2_navigation_msgs::RequestCurrentStateSrvRequest
  (:response () (instance se2_navigation_msgs::RequestCurrentStateSrvResponse :init)))

(setf (get se2_navigation_msgs::RequestCurrentStateSrvRequest :md5sum-) "c79f0d88a7597db980a56d7ac144c654")
(setf (get se2_navigation_msgs::RequestCurrentStateSrvRequest :datatype-) "se2_navigation_msgs/RequestCurrentStateSrvRequest")
(setf (get se2_navigation_msgs::RequestCurrentStateSrvRequest :definition-)
      "# Request
---
# Response
geometry_msgs/Pose pose
geometry_msgs/Twist twist

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

================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular

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
")

(setf (get se2_navigation_msgs::RequestCurrentStateSrvResponse :md5sum-) "c79f0d88a7597db980a56d7ac144c654")
(setf (get se2_navigation_msgs::RequestCurrentStateSrvResponse :datatype-) "se2_navigation_msgs/RequestCurrentStateSrvResponse")
(setf (get se2_navigation_msgs::RequestCurrentStateSrvResponse :definition-)
      "# Request
---
# Response
geometry_msgs/Pose pose
geometry_msgs/Twist twist

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

================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular

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
")



(provide :se2_navigation_msgs/RequestCurrentStateSrv "c79f0d88a7597db980a56d7ac144c654")


