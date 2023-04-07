;; Auto-generated. Do not edit!


(when (boundp 'se2_grid_map_generator_msgs::SaveMap)
  (if (not (find-package "SE2_GRID_MAP_GENERATOR_MSGS"))
    (make-package "SE2_GRID_MAP_GENERATOR_MSGS"))
  (shadow 'SaveMap (find-package "SE2_GRID_MAP_GENERATOR_MSGS")))
(unless (find-package "SE2_GRID_MAP_GENERATOR_MSGS::SAVEMAP")
  (make-package "SE2_GRID_MAP_GENERATOR_MSGS::SAVEMAP"))
(unless (find-package "SE2_GRID_MAP_GENERATOR_MSGS::SAVEMAPREQUEST")
  (make-package "SE2_GRID_MAP_GENERATOR_MSGS::SAVEMAPREQUEST"))
(unless (find-package "SE2_GRID_MAP_GENERATOR_MSGS::SAVEMAPRESPONSE")
  (make-package "SE2_GRID_MAP_GENERATOR_MSGS::SAVEMAPRESPONSE"))

(in-package "ROS")





(defclass se2_grid_map_generator_msgs::SaveMapRequest
  :super ros::object
  :slots (_filepath ))

(defmethod se2_grid_map_generator_msgs::SaveMapRequest
  (:init
   (&key
    ((:filepath __filepath) "")
    )
   (send-super :init)
   (setq _filepath (string __filepath))
   self)
  (:filepath
   (&optional __filepath)
   (if __filepath (setq _filepath __filepath)) _filepath)
  (:serialization-length
   ()
   (+
    ;; string _filepath
    4 (length _filepath)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _filepath
       (write-long (length _filepath) s) (princ _filepath s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _filepath
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _filepath (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass se2_grid_map_generator_msgs::SaveMapResponse
  :super ros::object
  :slots (_success _filepath ))

(defmethod se2_grid_map_generator_msgs::SaveMapResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:filepath __filepath) "")
    )
   (send-super :init)
   (setq _success __success)
   (setq _filepath (string __filepath))
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:filepath
   (&optional __filepath)
   (if __filepath (setq _filepath __filepath)) _filepath)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; string _filepath
    4 (length _filepath)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _filepath
       (write-long (length _filepath) s) (princ _filepath s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _filepath
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _filepath (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass se2_grid_map_generator_msgs::SaveMap
  :super ros::object
  :slots ())

(setf (get se2_grid_map_generator_msgs::SaveMap :md5sum-) "5922feee4022105a8dced2452aed4b3f")
(setf (get se2_grid_map_generator_msgs::SaveMap :datatype-) "se2_grid_map_generator_msgs/SaveMap")
(setf (get se2_grid_map_generator_msgs::SaveMap :request) se2_grid_map_generator_msgs::SaveMapRequest)
(setf (get se2_grid_map_generator_msgs::SaveMap :response) se2_grid_map_generator_msgs::SaveMapResponse)

(defmethod se2_grid_map_generator_msgs::SaveMapRequest
  (:response () (instance se2_grid_map_generator_msgs::SaveMapResponse :init)))

(setf (get se2_grid_map_generator_msgs::SaveMapRequest :md5sum-) "5922feee4022105a8dced2452aed4b3f")
(setf (get se2_grid_map_generator_msgs::SaveMapRequest :datatype-) "se2_grid_map_generator_msgs/SaveMapRequest")
(setf (get se2_grid_map_generator_msgs::SaveMapRequest :definition-)
      "# Request
string filepath
---
# Response
bool success
string filepath
")

(setf (get se2_grid_map_generator_msgs::SaveMapResponse :md5sum-) "5922feee4022105a8dced2452aed4b3f")
(setf (get se2_grid_map_generator_msgs::SaveMapResponse :datatype-) "se2_grid_map_generator_msgs/SaveMapResponse")
(setf (get se2_grid_map_generator_msgs::SaveMapResponse :definition-)
      "# Request
string filepath
---
# Response
bool success
string filepath
")



(provide :se2_grid_map_generator_msgs/SaveMap "5922feee4022105a8dced2452aed4b3f")


