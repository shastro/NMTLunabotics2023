;; Auto-generated. Do not edit!


(when (boundp 'se2_grid_map_generator_msgs::AddNan)
  (if (not (find-package "SE2_GRID_MAP_GENERATOR_MSGS"))
    (make-package "SE2_GRID_MAP_GENERATOR_MSGS"))
  (shadow 'AddNan (find-package "SE2_GRID_MAP_GENERATOR_MSGS")))
(unless (find-package "SE2_GRID_MAP_GENERATOR_MSGS::ADDNAN")
  (make-package "SE2_GRID_MAP_GENERATOR_MSGS::ADDNAN"))
(unless (find-package "SE2_GRID_MAP_GENERATOR_MSGS::ADDNANREQUEST")
  (make-package "SE2_GRID_MAP_GENERATOR_MSGS::ADDNANREQUEST"))
(unless (find-package "SE2_GRID_MAP_GENERATOR_MSGS::ADDNANRESPONSE")
  (make-package "SE2_GRID_MAP_GENERATOR_MSGS::ADDNANRESPONSE"))

(in-package "ROS")





(defclass se2_grid_map_generator_msgs::AddNanRequest
  :super ros::object
  :slots (_polygon ))

(defmethod se2_grid_map_generator_msgs::AddNanRequest
  (:init
   (&key
    ((:polygon __polygon) (instance se2_grid_map_generator_msgs::Polygon2D :init))
    )
   (send-super :init)
   (setq _polygon __polygon)
   self)
  (:polygon
   (&rest __polygon)
   (if (keywordp (car __polygon))
       (send* _polygon __polygon)
     (progn
       (if __polygon (setq _polygon (car __polygon)))
       _polygon)))
  (:serialization-length
   ()
   (+
    ;; se2_grid_map_generator_msgs/Polygon2D _polygon
    (send _polygon :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; se2_grid_map_generator_msgs/Polygon2D _polygon
       (send _polygon :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; se2_grid_map_generator_msgs/Polygon2D _polygon
     (send _polygon :deserialize buf ptr-) (incf ptr- (send _polygon :serialization-length))
   ;;
   self)
  )

(defclass se2_grid_map_generator_msgs::AddNanResponse
  :super ros::object
  :slots (_success ))

(defmethod se2_grid_map_generator_msgs::AddNanResponse
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

(defclass se2_grid_map_generator_msgs::AddNan
  :super ros::object
  :slots ())

(setf (get se2_grid_map_generator_msgs::AddNan :md5sum-) "ed846c130da1159d54d0e3b43ecc2916")
(setf (get se2_grid_map_generator_msgs::AddNan :datatype-) "se2_grid_map_generator_msgs/AddNan")
(setf (get se2_grid_map_generator_msgs::AddNan :request) se2_grid_map_generator_msgs::AddNanRequest)
(setf (get se2_grid_map_generator_msgs::AddNan :response) se2_grid_map_generator_msgs::AddNanResponse)

(defmethod se2_grid_map_generator_msgs::AddNanRequest
  (:response () (instance se2_grid_map_generator_msgs::AddNanResponse :init)))

(setf (get se2_grid_map_generator_msgs::AddNanRequest :md5sum-) "ed846c130da1159d54d0e3b43ecc2916")
(setf (get se2_grid_map_generator_msgs::AddNanRequest :datatype-) "se2_grid_map_generator_msgs/AddNanRequest")
(setf (get se2_grid_map_generator_msgs::AddNanRequest :definition-)
      "# Request
se2_grid_map_generator_msgs/Polygon2D polygon

================================================================================
MSG: se2_grid_map_generator_msgs/Polygon2D
se2_grid_map_generator_msgs/Position2D[] vertices
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

(setf (get se2_grid_map_generator_msgs::AddNanResponse :md5sum-) "ed846c130da1159d54d0e3b43ecc2916")
(setf (get se2_grid_map_generator_msgs::AddNanResponse :datatype-) "se2_grid_map_generator_msgs/AddNanResponse")
(setf (get se2_grid_map_generator_msgs::AddNanResponse :definition-)
      "# Request
se2_grid_map_generator_msgs/Polygon2D polygon

================================================================================
MSG: se2_grid_map_generator_msgs/Polygon2D
se2_grid_map_generator_msgs/Position2D[] vertices
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



(provide :se2_grid_map_generator_msgs/AddNan "ed846c130da1159d54d0e3b43ecc2916")


