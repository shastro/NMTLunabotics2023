;; Auto-generated. Do not edit!


(when (boundp 'motor_bridge::Estop)
  (if (not (find-package "MOTOR_BRIDGE"))
    (make-package "MOTOR_BRIDGE"))
  (shadow 'Estop (find-package "MOTOR_BRIDGE")))
(unless (find-package "MOTOR_BRIDGE::ESTOP")
  (make-package "MOTOR_BRIDGE::ESTOP"))

(in-package "ROS")
;;//! \htmlinclude Estop.msg.html


(defclass motor_bridge::Estop
  :super ros::object
  :slots (_stop ))

(defmethod motor_bridge::Estop
  (:init
   (&key
    ((:stop __stop) nil)
    )
   (send-super :init)
   (setq _stop __stop)
   self)
  (:stop
   (&optional (__stop :null))
   (if (not (eq __stop :null)) (setq _stop __stop)) _stop)
  (:serialization-length
   ()
   (+
    ;; bool _stop
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _stop
       (if _stop (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _stop
     (setq _stop (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get motor_bridge::Estop :md5sum-) "71f1172402e56b82716ca71681cded6b")
(setf (get motor_bridge::Estop :datatype-) "motor_bridge/Estop")
(setf (get motor_bridge::Estop :definition-)
      "# Emergency stop message

# Emergency stop (0 = false, 1 = true)
bool stop

")



(provide :motor_bridge/Estop "71f1172402e56b82716ca71681cded6b")


