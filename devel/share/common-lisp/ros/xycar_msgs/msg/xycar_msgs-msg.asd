
(cl:in-package :asdf)

(defsystem "xycar_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :sensor_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "XycarMotor" :depends-on ("_package_XycarMotor"))
    (:file "_package_XycarMotor" :depends-on ("_package"))
    (:file "XycarUltrasonic" :depends-on ("_package_XycarUltrasonic"))
    (:file "_package_XycarUltrasonic" :depends-on ("_package"))
  ))