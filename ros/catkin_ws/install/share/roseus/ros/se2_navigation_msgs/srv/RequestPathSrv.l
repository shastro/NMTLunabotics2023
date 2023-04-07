;; Auto-generated. Do not edit!


(when (boundp 'se2_navigation_msgs::RequestPathSrv)
  (if (not (find-package "SE2_NAVIGATION_MSGS"))
    (make-package "SE2_NAVIGATION_MSGS"))
  (shadow 'RequestPathSrv (find-package "SE2_NAVIGATION_MSGS")))
(unless (find-package "SE2_NAVIGATION_MSGS::REQUESTPATHSRV")
  (make-package "SE2_NAVIGATION_MSGS::REQUESTPATHSRV"))
(unless (find-package "SE2_NAVIGATION_MSGS::REQUESTPATHSRVREQUEST")
  (make-package "SE2_NAVIGATION_MSGS::REQUESTPATHSRVREQUEST"))
(unless (find-package "SE2_NAVIGATION_MSGS::REQUESTPATHSRVRESPONSE")
  (make-package "SE2_NAVIGATION_MSGS::REQUESTPATHSRVRESPONSE"))

(in-package "ROS")





(defclass se2_navigation_msgs::RequestPathSrvRequest
  :super ros::object
  :slots (_pathRequest ))

(defmethod se2_navigation_msgs::RequestPathSrvRequest
  (:init
   (&key
    ((:pathRequest __pathRequest) (instance se2_navigation_msgs::PathRequestMsg :init))
    )
   (send-super :init)
   (setq _pathRequest __pathRequest)
   self)
  (:pathRequest
   (&rest __pathRequest)
   (if (keywordp (car __pathRequest))
       (send* _pathRequest __pathRequest)
     (progn
       (if __pathRequest (setq _pathRequest (car __pathRequest)))
       _pathRequest)))
  (:serialization-length
   ()
   (+
    ;; se2_navigation_msgs/PathRequestMsg _pathRequest
    (send _pathRequest :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; se2_navigation_msgs/PathRequestMsg _pathRequest
       (send _pathRequest :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; se2_navigation_msgs/PathRequestMsg _pathRequest
     (send _pathRequest :deserialize buf ptr-) (incf ptr- (send _pathRequest :serialization-length))
   ;;
   self)
  )

(defclass se2_navigation_msgs::RequestPathSrvResponse
  :super ros::object
  :slots (_status ))

(defmethod se2_navigation_msgs::RequestPathSrvResponse
  (:init
   (&key
    ((:status __status) nil)
    )
   (send-super :init)
   (setq _status __status)
   self)
  (:status
   (&optional (__status :null))
   (if (not (eq __status :null)) (setq _status __status)) _status)
  (:serialization-length
   ()
   (+
    ;; bool _status
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _status
       (if _status (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _status
     (setq _status (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass se2_navigation_msgs::RequestPathSrv
  :super ros::object
  :slots ())

(setf (get se2_navigation_msgs::RequestPathSrv :md5sum-) "5539d8cfd1e0b02f407ce0465be5beeb")
(setf (get se2_navigation_msgs::RequestPathSrv :datatype-) "se2_navigation_msgs/RequestPathSrv")
(setf (get se2_navigation_msgs::RequestPathSrv :request) se2_navigation_msgs::RequestPathSrvRequest)
(setf (get se2_navigation_msgs::RequestPathSrv :response) se2_navigation_msgs::RequestPathSrvResponse)

(defmethod se2_navigation_msgs::RequestPathSrvRequest
  (:response () (instance se2_navigation_msgs::RequestPathSrvResponse :init)))

(setf (get se2_navigation_msgs::RequestPathSrvRequest :md5sum-) "5539d8cfd1e0b02f407ce0465be5beeb")
(setf (get se2_navigation_msgs::RequestPathSrvRequest :datatype-) "se2_navigation_msgs/RequestPathSrvRequest")
(setf (get se2_navigation_msgs::RequestPathSrvRequest :definition-)
      "# Request
PathRequestMsg 		pathRequest                          # Query, maybe useful for the future

================================================================================
MSG: se2_navigation_msgs/PathRequestMsg
geometry_msgs/Pose startingPose
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
---
# Response
bool status
")

(setf (get se2_navigation_msgs::RequestPathSrvResponse :md5sum-) "5539d8cfd1e0b02f407ce0465be5beeb")
(setf (get se2_navigation_msgs::RequestPathSrvResponse :datatype-) "se2_navigation_msgs/RequestPathSrvResponse")
(setf (get se2_navigation_msgs::RequestPathSrvResponse :definition-)
      "# Request
PathRequestMsg 		pathRequest                          # Query, maybe useful for the future

================================================================================
MSG: se2_navigation_msgs/PathRequestMsg
geometry_msgs/Pose startingPose
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
---
# Response
bool status
")



(provide :se2_navigation_msgs/RequestPathSrv "5539d8cfd1e0b02f407ce0465be5beeb")


