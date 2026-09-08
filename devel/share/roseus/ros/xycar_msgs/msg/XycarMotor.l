;; Auto-generated. Do not edit!


(when (boundp 'xycar_msgs::XycarMotor)
  (if (not (find-package "XYCAR_MSGS"))
    (make-package "XYCAR_MSGS"))
  (shadow 'XycarMotor (find-package "XYCAR_MSGS")))
(unless (find-package "XYCAR_MSGS::XYCARMOTOR")
  (make-package "XYCAR_MSGS::XYCARMOTOR"))

(in-package "ROS")
;;//! \htmlinclude XycarMotor.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass xycar_msgs::XycarMotor
  :super ros::object
  :slots (_header _angle _speed ))

(defmethod xycar_msgs::XycarMotor
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:angle __angle) 0.0)
    ((:speed __speed) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _angle (float __angle))
   (setq _speed (float __speed))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:angle
   (&optional __angle)
   (if __angle (setq _angle __angle)) _angle)
  (:speed
   (&optional __speed)
   (if __speed (setq _speed __speed)) _speed)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float32 _angle
    4
    ;; float32 _speed
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float32 _angle
       (sys::poke _angle (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _speed
       (sys::poke _speed (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float32 _angle
     (setq _angle (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _speed
     (setq _speed (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get xycar_msgs::XycarMotor :md5sum-) "86f102c3027ac5a649446e1f9f364baf")
(setf (get xycar_msgs::XycarMotor :datatype-) "xycar_msgs/XycarMotor")
(setf (get xycar_msgs::XycarMotor :definition-)
      "std_msgs/Header header
float32 angle
float32 speed

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

")



(provide :xycar_msgs/XycarMotor "86f102c3027ac5a649446e1f9f364baf")


