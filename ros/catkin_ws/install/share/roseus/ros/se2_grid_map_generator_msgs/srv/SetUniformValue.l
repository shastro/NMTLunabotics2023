;; Auto-generated. Do not edit!


(when (boundp 'se2_grid_map_generator_msgs::SetUniformValue)
  (if (not (find-package "SE2_GRID_MAP_GENERATOR_MSGS"))
    (make-package "SE2_GRID_MAP_GENERATOR_MSGS"))
  (shadow 'SetUniformValue (find-package "SE2_GRID_MAP_GENERATOR_MSGS")))
(unless (find-package "SE2_GRID_MAP_GENERATOR_MSGS::SETUNIFORMVALUE")
  (make-package "SE2_GRID_MAP_GENERATOR_MSGS::SETUNIFORMVALUE"))
(unless (find-package "SE2_GRID_MAP_GENERATOR_MSGS::SETUNIFORMVALUEREQUEST")
  (make-package "SE2_GRID_MAP_GENERATOR_MSGS::SETUNIFORMVALUEREQUEST"))
(unless (find-package "SE2_GRID_MAP_GENERATOR_MSGS::SETUNIFORMVALUERESPONSE")
  (make-package "SE2_GRID_MAP_GENERATOR_MSGS::SETUNIFORMVALUERESPONSE"))

(in-package "ROS")

(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))




(defclass se2_grid_map_generator_msgs::SetUniformValueRequest
  :super ros::object
  :slots (_layers _values ))

(defmethod se2_grid_map_generator_msgs::SetUniformValueRequest
  (:init
   (&key
    ((:layers __layers) ())
    ((:values __values) ())
    )
   (send-super :init)
   (setq _layers __layers)
   (setq _values __values)
   self)
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
    ;; std_msgs/String[] _layers
    (apply #'+ (send-all _layers :serialization-length)) 4
    ;; std_msgs/Float64[] _values
    (apply #'+ (send-all _values :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
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

(defclass se2_grid_map_generator_msgs::SetUniformValueResponse
  :super ros::object
  :slots (_success ))

(defmethod se2_grid_map_generator_msgs::SetUniformValueResponse
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

(defclass se2_grid_map_generator_msgs::SetUniformValue
  :super ros::object
  :slots ())

(setf (get se2_grid_map_generator_msgs::SetUniformValue :md5sum-) "e0f18e88964ccbda0238df7be68a4fd8")
(setf (get se2_grid_map_generator_msgs::SetUniformValue :datatype-) "se2_grid_map_generator_msgs/SetUniformValue")
(setf (get se2_grid_map_generator_msgs::SetUniformValue :request) se2_grid_map_generator_msgs::SetUniformValueRequest)
(setf (get se2_grid_map_generator_msgs::SetUniformValue :response) se2_grid_map_generator_msgs::SetUniformValueResponse)

(defmethod se2_grid_map_generator_msgs::SetUniformValueRequest
  (:response () (instance se2_grid_map_generator_msgs::SetUniformValueResponse :init)))

(setf (get se2_grid_map_generator_msgs::SetUniformValueRequest :md5sum-) "e0f18e88964ccbda0238df7be68a4fd8")
(setf (get se2_grid_map_generator_msgs::SetUniformValueRequest :datatype-) "se2_grid_map_generator_msgs/SetUniformValueRequest")
(setf (get se2_grid_map_generator_msgs::SetUniformValueRequest :definition-)
      "# Request
std_msgs/String[] layers
std_msgs/Float64[] values

================================================================================
MSG: std_msgs/String
string data

================================================================================
MSG: std_msgs/Float64
---
# Response
bool success

")

(setf (get se2_grid_map_generator_msgs::SetUniformValueResponse :md5sum-) "e0f18e88964ccbda0238df7be68a4fd8")
(setf (get se2_grid_map_generator_msgs::SetUniformValueResponse :datatype-) "se2_grid_map_generator_msgs/SetUniformValueResponse")
(setf (get se2_grid_map_generator_msgs::SetUniformValueResponse :definition-)
      "# Request
std_msgs/String[] layers
std_msgs/Float64[] values

================================================================================
MSG: std_msgs/String
string data

================================================================================
MSG: std_msgs/Float64
---
# Response
bool success

")



(provide :se2_grid_map_generator_msgs/SetUniformValue "e0f18e88964ccbda0238df7be68a4fd8")


