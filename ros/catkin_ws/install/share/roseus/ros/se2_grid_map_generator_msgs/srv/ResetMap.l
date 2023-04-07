;; Auto-generated. Do not edit!


(when (boundp 'se2_grid_map_generator_msgs::ResetMap)
  (if (not (find-package "SE2_GRID_MAP_GENERATOR_MSGS"))
    (make-package "SE2_GRID_MAP_GENERATOR_MSGS"))
  (shadow 'ResetMap (find-package "SE2_GRID_MAP_GENERATOR_MSGS")))
(unless (find-package "SE2_GRID_MAP_GENERATOR_MSGS::RESETMAP")
  (make-package "SE2_GRID_MAP_GENERATOR_MSGS::RESETMAP"))
(unless (find-package "SE2_GRID_MAP_GENERATOR_MSGS::RESETMAPREQUEST")
  (make-package "SE2_GRID_MAP_GENERATOR_MSGS::RESETMAPREQUEST"))
(unless (find-package "SE2_GRID_MAP_GENERATOR_MSGS::RESETMAPRESPONSE")
  (make-package "SE2_GRID_MAP_GENERATOR_MSGS::RESETMAPRESPONSE"))

(in-package "ROS")





(defclass se2_grid_map_generator_msgs::ResetMapRequest
  :super ros::object
  :slots ())

(defmethod se2_grid_map_generator_msgs::ResetMapRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass se2_grid_map_generator_msgs::ResetMapResponse
  :super ros::object
  :slots (_success ))

(defmethod se2_grid_map_generator_msgs::ResetMapResponse
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

(defclass se2_grid_map_generator_msgs::ResetMap
  :super ros::object
  :slots ())

(setf (get se2_grid_map_generator_msgs::ResetMap :md5sum-) "358e233cde0c8a8bcfea4ce193f8fc15")
(setf (get se2_grid_map_generator_msgs::ResetMap :datatype-) "se2_grid_map_generator_msgs/ResetMap")
(setf (get se2_grid_map_generator_msgs::ResetMap :request) se2_grid_map_generator_msgs::ResetMapRequest)
(setf (get se2_grid_map_generator_msgs::ResetMap :response) se2_grid_map_generator_msgs::ResetMapResponse)

(defmethod se2_grid_map_generator_msgs::ResetMapRequest
  (:response () (instance se2_grid_map_generator_msgs::ResetMapResponse :init)))

(setf (get se2_grid_map_generator_msgs::ResetMapRequest :md5sum-) "358e233cde0c8a8bcfea4ce193f8fc15")
(setf (get se2_grid_map_generator_msgs::ResetMapRequest :datatype-) "se2_grid_map_generator_msgs/ResetMapRequest")
(setf (get se2_grid_map_generator_msgs::ResetMapRequest :definition-)
      "# Request
---
# Response
bool success

")

(setf (get se2_grid_map_generator_msgs::ResetMapResponse :md5sum-) "358e233cde0c8a8bcfea4ce193f8fc15")
(setf (get se2_grid_map_generator_msgs::ResetMapResponse :datatype-) "se2_grid_map_generator_msgs/ResetMapResponse")
(setf (get se2_grid_map_generator_msgs::ResetMapResponse :definition-)
      "# Request
---
# Response
bool success

")



(provide :se2_grid_map_generator_msgs/ResetMap "358e233cde0c8a8bcfea4ce193f8fc15")


