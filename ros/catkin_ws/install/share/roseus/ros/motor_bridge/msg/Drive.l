;; Auto-generated. Do not edit!


(when (boundp 'motor_bridge::Drive)
  (if (not (find-package "MOTOR_BRIDGE"))
    (make-package "MOTOR_BRIDGE"))
  (shadow 'Drive (find-package "MOTOR_BRIDGE")))
(unless (find-package "MOTOR_BRIDGE::DRIVE")
  (make-package "MOTOR_BRIDGE::DRIVE"))

(in-package "ROS")
;;//! \htmlinclude Drive.msg.html


(defclass motor_bridge::Drive
  :super ros::object
  :slots (_speed _motor ))

(defmethod motor_bridge::Drive
  (:init
   (&key
    ((:speed __speed) 0)
    ((:motor __motor) 0)
    )
   (send-super :init)
   (setq _speed (round __speed))
   (setq _motor (round __motor))
   self)
  (:speed
   (&optional __speed)
   (if __speed (setq _speed __speed)) _speed)
  (:motor
   (&optional __motor)
   (if __motor (setq _motor __motor)) _motor)
  (:serialization-length
   ()
   (+
    ;; int32 _speed
    4
    ;; int32 _motor
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _speed
       (write-long _speed s)
     ;; int32 _motor
       (write-long _motor s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _speed
     (setq _speed (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _motor
     (setq _motor (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get motor_bridge::Drive :md5sum-) "14b3be7c56f8564b45133d0af1560d8f")
(setf (get motor_bridge::Drive :datatype-) "motor_bridge/Drive")
(setf (get motor_bridge::Drive :definition-)
      "# Drive Control Message

# Speed Control (Full Backward = -1024, Stop = 0, Full Forward = 1024)
int32 speed

# Which motor (0 = both, 1 = left, 2 = right)
int32 motor


")



(provide :motor_bridge/Drive "14b3be7c56f8564b45133d0af1560d8f")


