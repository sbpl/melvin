
(in-package :asdf)

(defsystem "segwayrmp-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils)
  :components ((:file "_package")
    (:file "RawData" :depends-on ("_package"))
    (:file "_package_RawData" :depends-on ("_package"))
    ))
