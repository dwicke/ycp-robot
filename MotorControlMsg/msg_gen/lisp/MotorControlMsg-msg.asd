
(cl:in-package :asdf)

(defsystem "MotorControlMsg-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "MotorCommand" :depends-on ("_package_MotorCommand"))
    (:file "_package_MotorCommand" :depends-on ("_package"))
    (:file "WheelVelocities" :depends-on ("_package_WheelVelocities"))
    (:file "_package_WheelVelocities" :depends-on ("_package"))
  ))