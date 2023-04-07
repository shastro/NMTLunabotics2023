;; Auto-generated. Do not edit!


(when (boundp 'se2_grid_map_generator_msgs::AddPolygonObstacle)
  (if (not (find-package "SE2_GRID_MAP_GENERATOR_MSGS"))
    (make-package "SE2_GRID_MAP_GENERATOR_MSGS"))
  (shadow 'AddPolygonObstacle (find-package "SE2_GRID_MAP_GENERATOR_MSGS")))
(unless (find-package "SE2_GRID_MAP_GENERATOR_MSGS::ADDPOLYGONOBSTACLE")
  (make-package "SE2_GRID_MAP_GENERATOR_MSGS::ADDPOLYGONOBSTACLE"))
(unless (find-package "SE2_GRID_MAP_GENERATOR_MSGS::ADDPOLYGONOBSTACLEREQUEST")
  (make-package "SE2_GRID_MAP_GENERATOR_MSGS::ADDPOLYGONOBSTACLEREQUEST"))
(unless (find-package "SE2_GRID_MAP_GENERATOR_MSGS::ADDPOLYGONOBSTACLERESPONSE")
  (make-package "SE2_GRID_MAP_GENERATOR_MSGS::ADDPOLYGONOBSTACLERESPONSE"))

(in-package "ROS")





(defclass se2_grid_map_generator_msgs::AddPolygonObstacleRequest
  :super ros::object
  :slots (_obstacle ))

(defmethod se2_grid_map_generator_msgs::AddPolygonObstacleRequest
  (:init
   (&key
    ((:obstacle __obstacle) (instance se2_grid_map_generator_msgs::PolygonObstacle :init))
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
    ;; se2_grid_map_generator_msgs/PolygonObstacle _obstacle
    (send _obstacle :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; se2_grid_map_generator_msgs/PolygonObstacle _obstacle
       (send _obstacle :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; se2_grid_map_generator_msgs/PolygonObstacle _obstacle
     (send _obstacle :deserialize buf ptr-) (incf ptr- (send _obstacle :serialization-length))
   ;;
   self)
  )

(defclass se2_grid_map_generator_msgs::AddPolygonObstacleResponse
  :super ros::object
  :slots (_success ))

(defmethod se2_grid_map_generator_msgs::AddPolygonObstacleResponse
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

(defclass se2_grid_map_generator_msgs::AddPolygonObstacle
  :super ros::object
  :slots ())

(setf (get se2_grid_map_generator_msgs::AddPolygonObstacle :md5sum-) "8cdfa34c81ded8fbfe0b3c21eb0b43e9")
(setf (get se2_grid_map_generator_msgs::AddPolygonObstacle :datatype-) "se2_grid_map_generator_msgs/AddPolygonObstacle")
(setf (get se2_grid_map_generator_msgs::AddPolygonObstacle :request) se2_grid_map_generator_msgs::AddPolygonObstacleRequest)
(setf (get se2_grid_map_generator_msgs::AddPolygonObstacle :response) se2_grid_map_generator_msgs::AddPolygonObstacleResponse)

(defmethod se2_grid_map_generator_msgs::AddPolygonObstacleRequest
  (:response () (instance se2_grid_map_generator_msgs::AddPolygonObstacleResponse :init)))

(setf (get se2_grid_map_generator_msgs::AddPolygonObstacleRequest :md5sum-) "8cdfa34c81ded8fbfe0b3c21eb0b43e9")
(setf (get se2_grid_map_generator_msgs::AddPolygonObstacleRequest :datatype-) "se2_grid_map_generator_msgs/AddPolygonObstacleRequest")
(setf (get se2_grid_map_generator_msgs::AddPolygonObstacleRequest :definition-)
      "# Request
se2_grid_map_generator_msgs/PolygonObstacle obstacle

================================================================================
MSG: se2_grid_map_generator_msgs/PolygonObstacle
se2_grid_map_generator_msgs/Polygon2D polygon
std_msgs/String[] layers
std_msgs/Float64[] values
================================================================================
MSG: se2_grid_map_generator_msgs/Polygon2D
se2_grid_map_generator_msgs/Position2D[] vertices
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

(setf (get se2_grid_map_generator_msgs::AddPolygonObstacleResponse :md5sum-) "8cdfa34c81ded8fbfe0b3c21eb0b43e9")
(setf (get se2_grid_map_generator_msgs::AddPolygonObstacleResponse :datatype-) "se2_grid_map_generator_msgs/AddPolygonObstacleResponse")
(setf (get se2_grid_map_generator_msgs::AddPolygonObstacleResponse :definition-)
      "# Request
se2_grid_map_generator_msgs/PolygonObstacle obstacle

================================================================================
MSG: se2_grid_map_generator_msgs/PolygonObstacle
se2_grid_map_generator_msgs/Polygon2D polygon
std_msgs/String[] layers
std_msgs/Float64[] values
================================================================================
MSG: se2_grid_map_generator_msgs/Polygon2D
se2_grid_map_generator_msgs/Position2D[] vertices
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



(provide :se2_grid_map_generator_msgs/AddPolygonObstacle "8cdfa34c81ded8fbfe0b3c21eb0b43e9")


