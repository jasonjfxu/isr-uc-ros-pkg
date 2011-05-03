; Auto-generated. Do not edit!


(in-package laptop_battery-msg)


;//! \htmlinclude Battery.msg.html

(defclass <Battery> (ros-message)
  ((header
    :reader header-val
    :initarg :header
    :type roslib-msg:<Header>
    :initform (make-instance 'roslib-msg:<Header>))
   (charging
    :reader charging-val
    :initarg :charging
    :type boolean
    :initform nil)
   (level
    :reader level-val
    :initarg :level
    :type float
    :initform 0.0)
   (time_remaining
    :reader time_remaining-val
    :initarg :time_remaining
    :type integer
    :initform 0))
)
(defmethod serialize ((msg <Battery>) ostream)
  "Serializes a message object of type '<Battery>"
  (serialize (slot-value msg 'header) ostream)
    (write-byte (ldb (byte 8 0) (if (slot-value msg 'charging) 1 0)) ostream)
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'level))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
    (write-byte (ldb (byte 8 0) (slot-value msg 'time_remaining)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'time_remaining)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'time_remaining)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'time_remaining)) ostream)
)
(defmethod deserialize ((msg <Battery>) istream)
  "Deserializes a message object of type '<Battery>"
  (deserialize (slot-value msg 'header) istream)
  (setf (slot-value msg 'charging) (not (zerop (read-byte istream))))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'level) (roslisp-utils:decode-single-float-bits bits)))
  (setf (ldb (byte 8 0) (slot-value msg 'time_remaining)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'time_remaining)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'time_remaining)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'time_remaining)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<Battery>)))
  "Returns string type for a message object of type '<Battery>"
  "laptop_battery/Battery")
(defmethod md5sum ((type (eql '<Battery>)))
  "Returns md5sum for a message object of type '<Battery>"
  "21f54717ee47fec662ccdb9a6ca81a7d")
(defmethod message-definition ((type (eql '<Battery>)))
  "Returns full string definition for message of type '<Battery>"
  (format nil "Header header~%bool charging~%float32 level				# in %~%uint32 time_remaining		# in minutes~%~%================================================================================~%MSG: roslib/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(defmethod serialization-length ((msg <Battery>))
  (+ 0
     (serialization-length (slot-value msg 'header))
     1
     4
     4
))
(defmethod ros-message-to-list ((msg <Battery>))
  "Converts a ROS message object to a list"
  (list '<Battery>
    (cons ':header (header-val msg))
    (cons ':charging (charging-val msg))
    (cons ':level (level-val msg))
    (cons ':time_remaining (time_remaining-val msg))
))
