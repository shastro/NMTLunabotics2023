
(cl:in-package :asdf)

(defsystem "se2_navigation_msgs-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :se2_navigation_msgs-msg
)
  :components ((:file "_package")
    (:file "RequestCurrentStateSrv" :depends-on ("_package_RequestCurrentStateSrv"))
    (:file "_package_RequestCurrentStateSrv" :depends-on ("_package"))
    (:file "RequestPathSrv" :depends-on ("_package_RequestPathSrv"))
    (:file "_package_RequestPathSrv" :depends-on ("_package"))
    (:file "SendControllerCommandSrv" :depends-on ("_package_SendControllerCommandSrv"))
    (:file "_package_SendControllerCommandSrv" :depends-on ("_package"))
  ))