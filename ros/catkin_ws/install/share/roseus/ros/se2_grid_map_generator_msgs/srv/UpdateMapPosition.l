;; Auto-generated. Do not edit!


(when (boundp 'se2_grid_map_generator_msgs::UpdateMapPosition)
  (if (not (find-package "SE2_GRID_MAP_GENERATOR_MSGS"))
    (make-package "SE2_GRID_MAP_GENERATOR_MSGS"))
  (shadow 'UpdateMapPosition (find-package "SE2_GRID_MAP_GENERATOR_MSGS")))
(unless (find-package "SE2_GRID_MAP_GENERATOR_MSGS::UPDATEMAPPOSITION")
  (make-package "SE2_GRID_MAP_GENERATOR_MSGS::UPDATEMAPPOSITION"))
(unless (find-package "SE2_GRID_MAP_GENERATOR_MSGS::UPDATEMAPPOSITIONREQUEST")
  (make-package "SE2_GRID_MAP_GENERATOR_MSGS::UPDATEMAPPOSITIONREQUEST"))
(unless (find-package "SE2_GRID_MAP_GENERATOR_MSGS::UPDATEMAPPOSITIONRESPONSE")
  (make-package "SE2_GRID_MAP_GENERATOR_MSGS::UPDATEMAPPOSITIONRESPONSE"))

(in-package "ROS")





(defclass se2_grid_map_generator_msgs::UpdateMapPositionRequest
  :super ros::object
  :slots (_position ))

(defmethod se2_grid_map_generator_msgs::UpdateMapPositionRequest
  (:init
   (&key
    ((:position __position) (instance se2_grid_map_generator_msgs::Position2D :init))
    )
   (send-super :init)
   (setq _position __position)
   self)
  (:position
   (&rest __position)
   (if (keywordp (car __position))
       (send* _position __position)
     (progn
       (if __position (setq _position (car __position)))
       _position)))
  (:serialization-length
   ()
   (+
    ;; se2_grid_map_generator_msgs/Position2D _position
    (send _position :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; se2_grid_map_generator_msgs/Position2D _position
       (send _position :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; se2_grid_map_generator_msgs/Position2D _position
     (send _position :deserialize buf ptr-) (incf ptr- (send _position :serialization-length))
   ;;
   self)
  )

(defclass se2_grid_map_generator_msgs::UpdateMapPositionResponse
  :super ros::object
  :slots (_success ))

(defmethod se2_grid_map_generator_msgs::UpdateMapPositionResponse
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

(defclass se2_grid_map_generator_msgs::UpdateMapPosition
  :super ros::object
  :slots ())

(setf (get se2_grid_map_generator_msgs::UpdateMapPosition :md5sum-) "ce04429a523a22c93100dc2950508e3d")
(setf (get se2_grid_map_generator_msgs::UpdateMapPosition :datatype-) "se2_grid_map_generator_msgs/UpdateMapPosition")
(setf (get se2_grid_map_generator_msgs::UpdateMapPosition :request) se2_grid_map_generator_msgs::UpdateMapPositionRequest)
(setf (get se2_grid_map_generator_msgs::UpdateMapPosition :response) se2_grid_map_generator_msgs::UpdateMapPositionResponse)

(defmethod se2_grid_map_generator_msgs::UpdateMapPositionRequest
  (:response () (instance se2_grid_map_generator_msgs::UpdateMapPositionResponse :init)))

(setf (get se2_grid_map_generator_msgs::UpdateMapPositionRequest :md5sum-) "ce04429a523a22c93100dc2950508e3d")
(setf (get se2_grid_map_generator_msgs::UpdateMapPositionRequest :datatype-) "se2_grid_map_generator_msgs/UpdateMapPositionRequest")
(setf (get se2_grid_map_generator_msgs::UpdateMapPositionRequest :definition-)
      "# Request
se2_grid_map_generator_msgs/Position2D position

================================================================================
MSG: se2_grid_map_generator_msgs/Position2D
std_msgs/Float64 x
std_msgs/Float64 y
================================================================================
MSG: std_msgs/Float64
---
# Response
bool success

")

(setf (get se2_grid_map_generator_msgs::UpdateMapPositionResponse :md5sum-) "ce04429a523a22c93100dc2950508e3d")
(setf (get se2_grid_map_generator_msgs::UpdateMapPositionResponse :datatype-) "se2_grid_map_generator_msgs/UpdateMapPositionResponse")
(setf (get se2_grid_map_generator_msgs::UpdateMapPositionResponse :definition-)
      "# Request
se2_grid_map_generator_msgs/Position2D position

================================================================================
MSG: se2_grid_map_generator_msgs/Position2D
std_msgs/Float64 x
std_msgs/Float64 y
================================================================================
MSG: std_msgs/Float64
---
# Response
bool success

")



(provide :se2_grid_map_generator_msgs/UpdateMapPosition "ce04429a523a22c93100dc2950508e3d")


