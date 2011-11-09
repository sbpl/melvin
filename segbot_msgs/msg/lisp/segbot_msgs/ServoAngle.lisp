; Auto-generated. Do not edit!


(in-package segbot_msgs-msg)


;//! \htmlinclude ServoAngle.msg.html

(defclass <ServoAngle> (ros-message)
  ((header
    :reader header-val
    :initarg :header
    :type roslib-msg:<Header>
    :initform (make-instance 'roslib-msg:<Header>))
   (angle
    :reader angle-val
    :initarg :angle
    :type float
    :initform 0.0)
   (id
    :reader id-val
    :initarg :id
    :type integer
    :initform 0))
)
(defmethod serialize ((msg <ServoAngle>) ostream)
  "Serializes a message object of type '<ServoAngle>"
  (serialize (slot-value msg 'header) ostream)
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'angle))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
    (write-byte (ldb (byte 8 0) (slot-value msg 'id)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'id)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'id)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'id)) ostream)
)
(defmethod deserialize ((msg <ServoAngle>) istream)
  "Deserializes a message object of type '<ServoAngle>"
  (deserialize (slot-value msg 'header) istream)
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'angle) (roslisp-utils:decode-single-float-bits bits)))
  (setf (ldb (byte 8 0) (slot-value msg 'id)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'id)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'id)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'id)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<ServoAngle>)))
  "Returns string type for a message object of type '<ServoAngle>"
  "segbot_msgs/ServoAngle")
(defmethod md5sum ((type (eql '<ServoAngle>)))
  "Returns md5sum for a message object of type '<ServoAngle>"
  "4b91cb9ea6444a870c92e12218f20e17")
(defmethod message-definition ((type (eql '<ServoAngle>)))
  "Returns full string definition for message of type '<ServoAngle>"
  (format nil "#the joint angle of a servo~%Header header~%float32 angle~%int32   id~%~%================================================================================~%MSG: roslib/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(defmethod serialization-length ((msg <ServoAngle>))
  (+ 0
     (serialization-length (slot-value msg 'header))
     4
     4
))
(defmethod ros-message-to-list ((msg <ServoAngle>))
  "Converts a ROS message object to a list"
  (list '<ServoAngle>
    (cons ':header (header-val msg))
    (cons ':angle (angle-val msg))
    (cons ':id (id-val msg))
))
