;; Auto-generated. Do not edit!


(when (boundp 'se2_grid_map_generator_msgs::Position2D)
  (if (not (find-package "SE2_GRID_MAP_GENERATOR_MSGS"))
    (make-package "SE2_GRID_MAP_GENERATOR_MSGS"))
  (shadow 'Position2D (find-package "SE2_GRID_MAP_GENERATOR_MSGS")))
(unless (find-package "SE2_GRID_MAP_GENERATOR_MSGS::POSITION2D")
  (make-package "SE2_GRID_MAP_GENERATOR_MSGS::POSITION2D"))

(in-package "ROS")
;;//! \htmlinclude Position2D.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass se2_grid_map_generator_msgs::Position2D
  :super ros::object
  :slots (_x _y ))

(defmethod se2_grid_map_generator_msgs::Position2D
  (:init
   (&key
    ((:x __x) (instance std_msgs::Float64 :init))
    ((:y __y) (instance std_msgs::Float64 :init))
    )
   (send-super :init)
   (setq _x __x)
   (setq _y __y)
   self)
  (:x
   (&rest __x)
   (if (keywordp (car __x))
       (send* _x __x)
     (progn
       (if __x (setq _x (car __x)))
       _x)))
  (:y
   (&rest __y)
   (if (keywordp (car __y))
       (send* _y __y)
     (progn
       (if __y (setq _y (car __y)))
       _y)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Float64 _x
    (send _x :serialization-length)
    ;; std_msgs/Float64 _y
    (send _y :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Float64 _x
       (send _x :serialize s)
     ;; std_msgs/Float64 _y
       (send _y :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Float64 _x
     (send _x :deserialize buf ptr-) (incf ptr- (send _x :serialization-length))
   ;; std_msgs/Float64 _y
     (send _y :deserialize buf ptr-) (incf ptr- (send _y :serialization-length))
   ;;
   self)
  )

(setf (get se2_grid_map_generator_msgs::Position2D :md5sum-) "5507085553fb157248c9d5a62e2bae14")
(setf (get se2_grid_map_generator_msgs::Position2D :datatype-) "se2_grid_map_generator_msgs/Position2D")
(setf (get se2_grid_map_generator_msgs::Position2D :definition-)
      "std_msgs/Float64 x
std_msgs/Float64 y
================================================================================
MSG: std_msgs/Float64
float64 data
")



(provide :se2_grid_map_generator_msgs/Position2D "5507085553fb157248c9d5a62e2bae14")


