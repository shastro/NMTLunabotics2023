
(cl:in-package :asdf)

(defsystem "motor_bridge-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Digger" :depends-on ("_package_Digger"))
    (:file "_package_Digger" :depends-on ("_package"))
    (:file "Drive" :depends-on ("_package_Drive"))
    (:file "_package_Drive" :depends-on ("_package"))
    (:file "Estop" :depends-on ("_package_Estop"))
    (:file "_package_Estop" :depends-on ("_package"))
    (:file "Pitch" :depends-on ("_package_Pitch"))
    (:file "_package_Pitch" :depends-on ("_package"))
    (:file "Stepp" :depends-on ("_package_Stepp"))
    (:file "_package_Stepp" :depends-on ("_package"))
  ))