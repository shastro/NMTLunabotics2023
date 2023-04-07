;; Auto-generated. Do not edit!


(when (boundp 'motor_bridge::Stepp)
  (if (not (find-package "MOTOR_BRIDGE"))
    (make-package "MOTOR_BRIDGE"))
  (shadow 'Stepp (find-package "MOTOR_BRIDGE")))
(unless (find-package "MOTOR_BRIDGE::STEPP")
  (make-package "MOTOR_BRIDGE::STEPP"))

(in-package "ROS")
;;//! \htmlinclude Stepp.msg.html


(defclass motor_bridge::Stepp
  :super ros::object
  :slots (_rpm _direction _motor ))

(defmethod motor_bridge::Stepp
  (:init
   (&key
    ((:rpm __rpm) 0)
    ((:direction __direction) 0)
    ((:motor __motor) 0)
    )
   (send-super :init)
   (setq _rpm (round __rpm))
   (setq _direction (round __direction))
   (setq _motor (round __motor))
   self)
  (:rpm
   (&optional __rpm)
   (if __rpm (setq _rpm __rpm)) _rpm)
  (:direction
   (&optional __direction)
   (if __direction (setq _direction __direction)) _direction)
  (:motor
   (&optional __motor)
   (if __motor (setq _motor __motor)) _motor)
  (:serialization-length
   ()
   (+
    ;; int32 _rpm
    4
    ;; int32 _direction
    4
    ;; int32 _motor
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _rpm
       (write-long _rpm s)
     ;; int32 _direction
       (write-long _direction s)
     ;; int32 _motor
       (write-long _motor s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _rpm
     (setq _rpm (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _direction
     (setq _direction (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _motor
     (setq _motor (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get motor_bridge::Stepp :md5sum-) "43447b26b2e072455822d7bf86b01b32")
(setf (get motor_bridge::Stepp :datatype-) "motor_bridge/Stepp")
(setf (get motor_bridge::Stepp :definition-)
      "# Stepper Control Message

# RPM (0 = stop, 1024 = full speed)
int32 rpm

# Direction (0 = stop, 1 = forward, 2 = backward)
int32 direction

# Which motor (0 = both, 1 = left, 2 = right)
int32 motor


")



(provide :motor_bridge/Stepp "43447b26b2e072455822d7bf86b01b32")


