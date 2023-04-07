;; Auto-generated. Do not edit!


(when (boundp 'se2_navigation_msgs::SendControllerCommandSrv)
  (if (not (find-package "SE2_NAVIGATION_MSGS"))
    (make-package "SE2_NAVIGATION_MSGS"))
  (shadow 'SendControllerCommandSrv (find-package "SE2_NAVIGATION_MSGS")))
(unless (find-package "SE2_NAVIGATION_MSGS::SENDCONTROLLERCOMMANDSRV")
  (make-package "SE2_NAVIGATION_MSGS::SENDCONTROLLERCOMMANDSRV"))
(unless (find-package "SE2_NAVIGATION_MSGS::SENDCONTROLLERCOMMANDSRVREQUEST")
  (make-package "SE2_NAVIGATION_MSGS::SENDCONTROLLERCOMMANDSRVREQUEST"))
(unless (find-package "SE2_NAVIGATION_MSGS::SENDCONTROLLERCOMMANDSRVRESPONSE")
  (make-package "SE2_NAVIGATION_MSGS::SENDCONTROLLERCOMMANDSRVRESPONSE"))

(in-package "ROS")





(defclass se2_navigation_msgs::SendControllerCommandSrvRequest
  :super ros::object
  :slots (_command ))

(defmethod se2_navigation_msgs::SendControllerCommandSrvRequest
  (:init
   (&key
    ((:command __command) (instance se2_navigation_msgs::ControllerCommandMsg :init))
    )
   (send-super :init)
   (setq _command __command)
   self)
  (:command
   (&rest __command)
   (if (keywordp (car __command))
       (send* _command __command)
     (progn
       (if __command (setq _command (car __command)))
       _command)))
  (:serialization-length
   ()
   (+
    ;; se2_navigation_msgs/ControllerCommandMsg _command
    (send _command :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; se2_navigation_msgs/ControllerCommandMsg _command
       (send _command :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; se2_navigation_msgs/ControllerCommandMsg _command
     (send _command :deserialize buf ptr-) (incf ptr- (send _command :serialization-length))
   ;;
   self)
  )

(defclass se2_navigation_msgs::SendControllerCommandSrvResponse
  :super ros::object
  :slots (_success ))

(defmethod se2_navigation_msgs::SendControllerCommandSrvResponse
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

(defclass se2_navigation_msgs::SendControllerCommandSrv
  :super ros::object
  :slots ())

(setf (get se2_navigation_msgs::SendControllerCommandSrv :md5sum-) "8f41fa61e7f9b864f313c51bd6bcb1ba")
(setf (get se2_navigation_msgs::SendControllerCommandSrv :datatype-) "se2_navigation_msgs/SendControllerCommandSrv")
(setf (get se2_navigation_msgs::SendControllerCommandSrv :request) se2_navigation_msgs::SendControllerCommandSrvRequest)
(setf (get se2_navigation_msgs::SendControllerCommandSrv :response) se2_navigation_msgs::SendControllerCommandSrvResponse)

(defmethod se2_navigation_msgs::SendControllerCommandSrvRequest
  (:response () (instance se2_navigation_msgs::SendControllerCommandSrvResponse :init)))

(setf (get se2_navigation_msgs::SendControllerCommandSrvRequest :md5sum-) "8f41fa61e7f9b864f313c51bd6bcb1ba")
(setf (get se2_navigation_msgs::SendControllerCommandSrvRequest :datatype-) "se2_navigation_msgs/SendControllerCommandSrvRequest")
(setf (get se2_navigation_msgs::SendControllerCommandSrvRequest :definition-)
      "# Request
ControllerCommandMsg command                          # Query, maybe useful for the future

================================================================================
MSG: se2_navigation_msgs/ControllerCommandMsg

int8 START_TRACKING=0
int8 STOP_TRACKING=1

---
# Response
bool success
")

(setf (get se2_navigation_msgs::SendControllerCommandSrvResponse :md5sum-) "8f41fa61e7f9b864f313c51bd6bcb1ba")
(setf (get se2_navigation_msgs::SendControllerCommandSrvResponse :datatype-) "se2_navigation_msgs/SendControllerCommandSrvResponse")
(setf (get se2_navigation_msgs::SendControllerCommandSrvResponse :definition-)
      "# Request
ControllerCommandMsg command                          # Query, maybe useful for the future

================================================================================
MSG: se2_navigation_msgs/ControllerCommandMsg

int8 START_TRACKING=0
int8 STOP_TRACKING=1

---
# Response
bool success
")



(provide :se2_navigation_msgs/SendControllerCommandSrv "8f41fa61e7f9b864f313c51bd6bcb1ba")


