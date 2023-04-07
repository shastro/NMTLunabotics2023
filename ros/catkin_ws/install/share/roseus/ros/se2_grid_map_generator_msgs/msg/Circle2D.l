;; Auto-generated. Do not edit!


(when (boundp 'se2_grid_map_generator_msgs::Circle2D)
  (if (not (find-package "SE2_GRID_MAP_GENERATOR_MSGS"))
    (make-package "SE2_GRID_MAP_GENERATOR_MSGS"))
  (shadow 'Circle2D (find-package "SE2_GRID_MAP_GENERATOR_MSGS")))
(unless (find-package "SE2_GRID_MAP_GENERATOR_MSGS::CIRCLE2D")
  (make-package "SE2_GRID_MAP_GENERATOR_MSGS::CIRCLE2D"))

(in-package "ROS")
;;//! \htmlinclude Circle2D.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass se2_grid_map_generator_msgs::Circle2D
  :super ros::object
  :slots (_center _radius ))

(defmethod se2_grid_map_generator_msgs::Circle2D
  (:init
   (&key
    ((:center __center) (instance se2_grid_map_generator_msgs::Position2D :init))
    ((:radius __radius) (instance std_msgs::Float64 :init))
    )
   (send-super :init)
   (setq _center __center)
   (setq _radius __radius)
   self)
  (:center
   (&rest __center)
   (if (keywordp (car __center))
       (send* _center __center)
     (progn
       (if __center (setq _center (car __center)))
       _center)))
  (:radius
   (&rest __radius)
   (if (keywordp (car __radius))
       (send* _radius __radius)
     (progn
       (if __radius (setq _radius (car __radius)))
       _radius)))
  (:serialization-length
   ()
   (+
    ;; se2_grid_map_generator_msgs/Position2D _center
    (send _center :serialization-length)
    ;; std_msgs/Float64 _radius
    (send _radius :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; se2_grid_map_generator_msgs/Position2D _center
       (send _center :serialize s)
     ;; std_msgs/Float64 _radius
       (send _radius :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; se2_grid_map_generator_msgs/Position2D _center
     (send _center :deserialize buf ptr-) (incf ptr- (send _center :serialization-length))
   ;; std_msgs/Float64 _radius
     (send _radius :deserialize buf ptr-) (incf ptr- (send _radius :serialization-length))
   ;;
   self)
  )

(setf (get se2_grid_map_generator_msgs::Circle2D :md5sum-) "0dfece0138cac79dd8ef38f9d1fa9947")
(setf (get se2_grid_map_generator_msgs::Circle2D :datatype-) "se2_grid_map_generator_msgs/Circle2D")
(setf (get se2_grid_map_generator_msgs::Circle2D :definition-)
      "se2_grid_map_generator_msgs/Position2D center
std_msgs/Float64 radius
================================================================================
MSG: se2_grid_map_generator_msgs/Position2D
std_msgs/Float64 x
std_msgs/Float64 y
================================================================================
MSG: std_msgs/Float64
float64 data
")



(provide :se2_grid_map_generator_msgs/Circle2D "0dfece0138cac79dd8ef38f9d1fa9947")


