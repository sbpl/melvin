
(in-package :asdf)

(defsystem "segbot_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :roslib-msg
)
  :components ((:file "_package")
    (:file "ServoAngle" :depends-on ("_package"))
    (:file "_package_ServoAngle" :depends-on ("_package"))
    (:file "ServoCmd" :depends-on ("_package"))
    (:file "_package_ServoCmd" :depends-on ("_package"))
    ))
