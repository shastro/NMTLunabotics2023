;; Auto-generated. Do not edit!


(when (boundp 'motor_bridge::Pitch)
  (if (not (find-package "MOTOR_BRIDGE"))
    (make-package "MOTOR_BRIDGE"))
  (shadow 'Pitch (find-package "MOTOR_BRIDGE")))
(unless (find-package "MOTOR_BRIDGE::PITCH")
  (make-package "MOTOR_BRIDGE::PITCH"))

(in-package "ROS")
;;//! \htmlinclude Pitch.msg.html


(defclass motor_bridge::Pitch
  :super ros::object
  :slots (_direction _motor ))

(defmethod motor_bridge::Pitch
  (:init
   (&key
    ((:direction __direction) 0)
    ((:motor __motor) 0)
    )
   (send-super :init)
   (setq _direction (round __direction))
   (setq _motor (round __motor))
   self)
  (:direction
   (&optional __direction)
   (if __direction (setq _direction __direction)) _direction)
  (:motor
   (&optional __motor)
   (if __motor (setq _motor __motor)) _motor)
  (:serialization-length
   ()
   (+
    ;; int32 _direction
    4
    ;; int32 _motor
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _direction
       (write-long _direction s)
     ;; int32 _motor
       (write-long _motor s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _direction
     (setq _direction (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _motor
     (setq _motor (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get motor_bridge::Pitch :md5sum-) "8e837e7a28d61ce2adf5af5ddb3d0d20")
(setf (get motor_bridge::Pitch :datatype-) "motor_bridge/Pitch")
(setf (get motor_bridge::Pitch :definition-)
      "# Pitch Control Message

# Pitch motion direction: (0 = stop, 1 = extend, 2 = retract)
int32 direction

# Which motor (0 = both, 1 = left, 2 = right)
int32 motor

")



(provide :motor_bridge/Pitch "8e837e7a28d61ce2adf5af5ddb3d0d20")


