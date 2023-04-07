;; Auto-generated. Do not edit!


(when (boundp 'se2_navigation_msgs::ControllerCommandMsg)
  (if (not (find-package "SE2_NAVIGATION_MSGS"))
    (make-package "SE2_NAVIGATION_MSGS"))
  (shadow 'ControllerCommandMsg (find-package "SE2_NAVIGATION_MSGS")))
(unless (find-package "SE2_NAVIGATION_MSGS::CONTROLLERCOMMANDMSG")
  (make-package "SE2_NAVIGATION_MSGS::CONTROLLERCOMMANDMSG"))

(in-package "ROS")
;;//! \htmlinclude ControllerCommandMsg.msg.html


(intern "*START_TRACKING*" (find-package "SE2_NAVIGATION_MSGS::CONTROLLERCOMMANDMSG"))
(shadow '*START_TRACKING* (find-package "SE2_NAVIGATION_MSGS::CONTROLLERCOMMANDMSG"))
(defconstant se2_navigation_msgs::ControllerCommandMsg::*START_TRACKING* 0)
(intern "*STOP_TRACKING*" (find-package "SE2_NAVIGATION_MSGS::CONTROLLERCOMMANDMSG"))
(shadow '*STOP_TRACKING* (find-package "SE2_NAVIGATION_MSGS::CONTROLLERCOMMANDMSG"))
(defconstant se2_navigation_msgs::ControllerCommandMsg::*STOP_TRACKING* 1)

(defun se2_navigation_msgs::ControllerCommandMsg-to-symbol (const)
  (cond
        ((= const 0) 'se2_navigation_msgs::ControllerCommandMsg::*START_TRACKING*)
        ((= const 1) 'se2_navigation_msgs::ControllerCommandMsg::*STOP_TRACKING*)
        (t nil)))

(defclass se2_navigation_msgs::ControllerCommandMsg
  :super ros::object
  :slots (_command ))

(defmethod se2_navigation_msgs::ControllerCommandMsg
  (:init
   (&key
    ((:command __command) 0)
    )
   (send-super :init)
   (setq _command (round __command))
   self)
  (:command
   (&optional __command)
   (if __command (setq _command __command)) _command)
  (:serialization-length
   ()
   (+
    ;; int8 _command
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _command
       (write-byte _command s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _command
     (setq _command (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _command 127) (setq _command (- _command 256)))
   ;;
   self)
  )

(setf (get se2_navigation_msgs::ControllerCommandMsg :md5sum-) "f666cfd8f3589400e119791c2d52c7ee")
(setf (get se2_navigation_msgs::ControllerCommandMsg :datatype-) "se2_navigation_msgs/ControllerCommandMsg")
(setf (get se2_navigation_msgs::ControllerCommandMsg :definition-)
      "
int8 START_TRACKING=0
int8 STOP_TRACKING=1

int8 command
")



(provide :se2_navigation_msgs/ControllerCommandMsg "f666cfd8f3589400e119791c2d52c7ee")


