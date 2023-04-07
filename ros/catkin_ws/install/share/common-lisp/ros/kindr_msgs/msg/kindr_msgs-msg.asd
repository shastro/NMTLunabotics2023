
(cl:in-package :asdf)

(defsystem "kindr_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "VectorAtPosition" :depends-on ("_package_VectorAtPosition"))
    (:file "_package_VectorAtPosition" :depends-on ("_package"))
  ))