
(in-package :asdf)

(defsystem "laptop_battery-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :roslib-msg
)
  :components ((:file "_package")
    (:file "Battery" :depends-on ("_package"))
    (:file "_package_Battery" :depends-on ("_package"))
    ))
