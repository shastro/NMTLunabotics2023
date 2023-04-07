;; Auto-generated. Do not edit!


(when (boundp 'se2_grid_map_generator_msgs::AddCircularObstacle)
  (if (not (find-package "SE2_GRID_MAP_GENERATOR_MSGS"))
    (make-package "SE2_GRID_MAP_GENERATOR_MSGS"))
  (shadow 'AddCircularObstacle (find-package "SE2_GRID_MAP_GENERATOR_MSGS")))
(unless (find-package "SE2_GRID_MAP_GENERATOR_MSGS::ADDCIRCULAROBSTACLE")
  (make-package "SE2_GRID_MAP_GENERATOR_MSGS::ADDCIRCULAROBSTACLE"))
(unless (find-package "SE2_GRID_MAP_GENERATOR_MSGS::ADDCIRCULAROBSTACLEREQUEST")
  (make-package "SE2_GRID_MAP_GENERATOR_MSGS::ADDCIRCULAROBSTACLEREQUEST"))
(unless (find-package "SE2_GRID_MAP_GENERATOR_MSGS::ADDCIRCULAROBSTACLERESPONSE")
  (make-package "SE2_GRID_MAP_GENERATOR_MSGS::ADDCIRCULAROBSTACLERESPONSE"))

(in-package "ROS")





(defclass se2_grid_map_generator_msgs::AddCircularObstacleRequest
  :super ros::object
  :slots (_obstacle ))

(defmethod se2_grid_map_generator_msgs::AddCircularObstacleRequest
  (:init
   (&key
    ((:obstacle __obstacle) (instance se2_grid_map_generator_msgs::CircularObstacle :init))
    )
   (send-super :init)
   (setq _obstacle __obstacle)
   self)
  (:obstacle
   (&rest __obstacle)
   (if (keywordp (car __obstacle))
       (send* _obstacle __obstacle)
     (progn
       (if __obstacle (setq _obstacle (car __obstacle)))
       _obstacle)))
  (:serialization-length
   ()
   (+
    ;; se2_grid_map_generator_msgs/CircularObstacle _obstacle
    (send _obstacle :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; se2_grid_map_generator_msgs/CircularObstacle _obstacle
       (send _obstacle :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; se2_grid_map_generator_msgs/CircularObstacle _obstacle
     (send _obstacle :deserialize buf ptr-) (incf ptr- (send _obstacle :serialization-length))
   ;;
   self)
  )

(defclass se2_grid_map_generator_msgs::AddCircularObstacleResponse
  :super ros::object
  :slots (_success ))

(defmethod se2_grid_map_generator_msgs::AddCircularObstacleResponse
  (:init
   (&key
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _success __success)
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass se2_grid_map_generator_msgs::AddCircularObstacle
  :super ros::object
  :slots ())

(setf (get se2_grid_map_generator_msgs::AddCircularObstacle :md5sum-) "5c6462e7f4d4eb409c7358cc94ad2577")
(setf (get se2_grid_map_generator_msgs::AddCircularObstacle :datatype-) "se2_grid_map_generator_msgs/AddCircularObstacle")
(setf (get se2_grid_map_generator_msgs::AddCircularObstacle :request) se2_grid_map_generator_msgs::AddCircularObstacleRequest)
(setf (get se2_grid_map_generator_msgs::AddCircularObstacle :response) se2_grid_map_generator_msgs::AddCircularObstacleResponse)

(defmethod se2_grid_map_generator_msgs::AddCircularObstacleRequest
  (:response () (instance se2_grid_map_generator_msgs::AddCircularObstacleResponse :init)))

(setf (get se2_grid_map_generator_msgs::AddCircularObstacleRequest :md5sum-) "5c6462e7f4d4eb409c7358cc94ad2577")
(setf (get se2_grid_map_generator_msgs::AddCircularObstacleRequest :datatype-) "se2_grid_map_generator_msgs/AddCircularObstacleRequest")
(setf (get se2_grid_map_generator_msgs::AddCircularObstacleRequest :definition-)
      "# Request
se2_grid_map_generator_msgs/CircularObstacle obstacle

================================================================================
MSG: se2_grid_map_generator_msgs/CircularObstacle
se2_grid_map_generator_msgs/Circle2D circle
std_msgs/String[] layers
std_msgs/Float64[] values
================================================================================
MSG: se2_grid_map_generator_msgs/Circle2D
se2_grid_map_generator_msgs/Position2D center
std_msgs/Float64 radius
================================================================================
MSG: se2_grid_map_generator_msgs/Position2D
std_msgs/Float64 x
std_msgs/Float64 y
================================================================================
MSG: std_msgs/Float64
float64 data
================================================================================
MSG: std_msgs/String
string data
---
# Response
bool success

")

(setf (get se2_grid_map_generator_msgs::AddCircularObstacleResponse :md5sum-) "5c6462e7f4d4eb409c7358cc94ad2577")
(setf (get se2_grid_map_generator_msgs::AddCircularObstacleResponse :datatype-) "se2_grid_map_generator_msgs/AddCircularObstacleResponse")
(setf (get se2_grid_map_generator_msgs::AddCircularObstacleResponse :definition-)
      "# Request
se2_grid_map_generator_msgs/CircularObstacle obstacle

================================================================================
MSG: se2_grid_map_generator_msgs/CircularObstacle
se2_grid_map_generator_msgs/Circle2D circle
std_msgs/String[] layers
std_msgs/Float64[] values
================================================================================
MSG: se2_grid_map_generator_msgs/Circle2D
se2_grid_map_generator_msgs/Position2D center
std_msgs/Float64 radius
================================================================================
MSG: se2_grid_map_generator_msgs/Position2D
std_msgs/Float64 x
std_msgs/Float64 y
================================================================================
MSG: std_msgs/Float64
float64 data
================================================================================
MSG: std_msgs/String
string data
---
# Response
bool success

")



(provide :se2_grid_map_generator_msgs/AddCircularObstacle "5c6462e7f4d4eb409c7358cc94ad2577")


