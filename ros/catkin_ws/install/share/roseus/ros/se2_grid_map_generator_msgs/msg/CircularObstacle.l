;; Auto-generated. Do not edit!


(when (boundp 'se2_grid_map_generator_msgs::CircularObstacle)
  (if (not (find-package "SE2_GRID_MAP_GENERATOR_MSGS"))
    (make-package "SE2_GRID_MAP_GENERATOR_MSGS"))
  (shadow 'CircularObstacle (find-package "SE2_GRID_MAP_GENERATOR_MSGS")))
(unless (find-package "SE2_GRID_MAP_GENERATOR_MSGS::CIRCULAROBSTACLE")
  (make-package "SE2_GRID_MAP_GENERATOR_MSGS::CIRCULAROBSTACLE"))

(in-package "ROS")
;;//! \htmlinclude CircularObstacle.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass se2_grid_map_generator_msgs::CircularObstacle
  :super ros::object
  :slots (_circle _layers _values ))

(defmethod se2_grid_map_generator_msgs::CircularObstacle
  (:init
   (&key
    ((:circle __circle) (instance se2_grid_map_generator_msgs::Circle2D :init))
    ((:layers __layers) ())
    ((:values __values) ())
    )
   (send-super :init)
   (setq _circle __circle)
   (setq _layers __layers)
   (setq _values __values)
   self)
  (:circle
   (&rest __circle)
   (if (keywordp (car __circle))
       (send* _circle __circle)
     (progn
       (if __circle (setq _circle (car __circle)))
       _circle)))
  (:layers
   (&rest __layers)
   (if (keywordp (car __layers))
       (send* _layers __layers)
     (progn
       (if __layers (setq _layers (car __layers)))
       _layers)))
  (:values
   (&rest __values)
   (if (keywordp (car __values))
       (send* _values __values)
     (progn
       (if __values (setq _values (car __values)))
       _values)))
  (:serialization-length
   ()
   (+
    ;; se2_grid_map_generator_msgs/Circle2D _circle
    (send _circle :serialization-length)
    ;; std_msgs/String[] _layers
    (apply #'+ (send-all _layers :serialization-length)) 4
    ;; std_msgs/Float64[] _values
    (apply #'+ (send-all _values :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; se2_grid_map_generator_msgs/Circle2D _circle
       (send _circle :serialize s)
     ;; std_msgs/String[] _layers
     (write-long (length _layers) s)
     (dolist (elem _layers)
       (send elem :serialize s)
       )
     ;; std_msgs/Float64[] _values
     (write-long (length _values) s)
     (dolist (elem _values)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; se2_grid_map_generator_msgs/Circle2D _circle
     (send _circle :deserialize buf ptr-) (incf ptr- (send _circle :serialization-length))
   ;; std_msgs/String[] _layers
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _layers (let (r) (dotimes (i n) (push (instance std_msgs::String :init) r)) r))
     (dolist (elem- _layers)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; std_msgs/Float64[] _values
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _values (let (r) (dotimes (i n) (push (instance std_msgs::Float64 :init) r)) r))
     (dolist (elem- _values)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get se2_grid_map_generator_msgs::CircularObstacle :md5sum-) "d3e815e4c24596dee9474400db6e2dc7")
(setf (get se2_grid_map_generator_msgs::CircularObstacle :datatype-) "se2_grid_map_generator_msgs/CircularObstacle")
(setf (get se2_grid_map_generator_msgs::CircularObstacle :definition-)
      "se2_grid_map_generator_msgs/Circle2D circle
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

")



(provide :se2_grid_map_generator_msgs/CircularObstacle "d3e815e4c24596dee9474400db6e2dc7")


