;; Auto-generated. Do not edit!


(when (boundp 'se2_grid_map_generator_msgs::Polygon2D)
  (if (not (find-package "SE2_GRID_MAP_GENERATOR_MSGS"))
    (make-package "SE2_GRID_MAP_GENERATOR_MSGS"))
  (shadow 'Polygon2D (find-package "SE2_GRID_MAP_GENERATOR_MSGS")))
(unless (find-package "SE2_GRID_MAP_GENERATOR_MSGS::POLYGON2D")
  (make-package "SE2_GRID_MAP_GENERATOR_MSGS::POLYGON2D"))

(in-package "ROS")
;;//! \htmlinclude Polygon2D.msg.html


(defclass se2_grid_map_generator_msgs::Polygon2D
  :super ros::object
  :slots (_vertices ))

(defmethod se2_grid_map_generator_msgs::Polygon2D
  (:init
   (&key
    ((:vertices __vertices) ())
    )
   (send-super :init)
   (setq _vertices __vertices)
   self)
  (:vertices
   (&rest __vertices)
   (if (keywordp (car __vertices))
       (send* _vertices __vertices)
     (progn
       (if __vertices (setq _vertices (car __vertices)))
       _vertices)))
  (:serialization-length
   ()
   (+
    ;; se2_grid_map_generator_msgs/Position2D[] _vertices
    (apply #'+ (send-all _vertices :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; se2_grid_map_generator_msgs/Position2D[] _vertices
     (write-long (length _vertices) s)
     (dolist (elem _vertices)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; se2_grid_map_generator_msgs/Position2D[] _vertices
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _vertices (let (r) (dotimes (i n) (push (instance se2_grid_map_generator_msgs::Position2D :init) r)) r))
     (dolist (elem- _vertices)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get se2_grid_map_generator_msgs::Polygon2D :md5sum-) "bccfee9dbb3170a57e32edc6d3cfcef5")
(setf (get se2_grid_map_generator_msgs::Polygon2D :datatype-) "se2_grid_map_generator_msgs/Polygon2D")
(setf (get se2_grid_map_generator_msgs::Polygon2D :definition-)
      "se2_grid_map_generator_msgs/Position2D[] vertices
================================================================================
MSG: se2_grid_map_generator_msgs/Position2D
std_msgs/Float64 x
std_msgs/Float64 y
================================================================================
MSG: std_msgs/Float64
float64 data
")



(provide :se2_grid_map_generator_msgs/Polygon2D "bccfee9dbb3170a57e32edc6d3cfcef5")


