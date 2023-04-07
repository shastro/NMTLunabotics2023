;; Auto-generated. Do not edit!


(when (boundp 'motor_bridge::Digger)
  (if (not (find-package "MOTOR_BRIDGE"))
    (make-package "MOTOR_BRIDGE"))
  (shadow 'Digger (find-package "MOTOR_BRIDGE")))
(unless (find-package "MOTOR_BRIDGE::DIGGER")
  (make-package "MOTOR_BRIDGE::DIGGER"))

(in-package "ROS")
;;//! \htmlinclude Digger.msg.html


(defclass motor_bridge::Digger
  :super ros::object
  :slots (_speed ))

(defmethod motor_bridge::Digger
  (:init
   (&key
    ((:speed __speed) 0)
    )
   (send-super :init)
   (setq _speed (round __speed))
   self)
  (:speed
   (&optional __speed)
   (if __speed (setq _speed __speed)) _speed)
  (:serialization-length
   ()
   (+
    ;; int32 _speed
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _speed
       (write-long _speed s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _speed
     (setq _speed (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get motor_bridge::Digger :md5sum-) "9f7a812be2def1e9db804a7fbea8c3a5")
(setf (get motor_bridge::Digger :datatype-) "motor_bridge/Digger")
(setf (get motor_bridge::Digger :definition-)
      "# Digger Control Message

# Speed Control (Full Backward = -1024, Stop = 0, Full Forward = 1024)
int32 speed


")



(provide :motor_bridge/Digger "9f7a812be2def1e9db804a7fbea8c3a5")


