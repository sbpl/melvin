; Auto-generated. Do not edit!


(in-package segbot_msgs-msg)


;//! \htmlinclude ServoCmd.msg.html

(defclass <ServoCmd> (ros-message)
  ((minAngle
    :reader minAngle-val
    :initarg :minAngle
    :type float
    :initform 0.0)
   (maxAngle
    :reader maxAngle-val
    :initarg :maxAngle
    :type float
    :initform 0.0)
   (velocity
    :reader velocity-val
    :initarg :velocity
    :type float
    :initform 0.0)
   (id
    :reader id-val
    :initarg :id
    :type integer
    :initform 0))
)
(defmethod serialize ((msg <ServoCmd>) ostream)
  "Serializes a message object of type '<ServoCmd>"
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'minAngle))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'maxAngle))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'velocity))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
    (write-byte (ldb (byte 8 0) (slot-value msg 'id)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'id)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'id)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'id)) ostream)
)
(defmethod deserialize ((msg <ServoCmd>) istream)
  "Deserializes a message object of type '<ServoCmd>"
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'minAngle) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'maxAngle) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'velocity) (roslisp-utils:decode-single-float-bits bits)))
  (setf (ldb (byte 8 0) (slot-value msg 'id)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'id)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'id)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'id)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<ServoCmd>)))
  "Returns string type for a message object of type '<ServoCmd>"
  "segbot_msgs/ServoCmd")
(defmethod md5sum ((type (eql '<ServoCmd>)))
  "Returns md5sum for a message object of type '<ServoCmd>"
  "7fda552e8aa60e0937cbdc527614be1f")
(defmethod message-definition ((type (eql '<ServoCmd>)))
  "Returns full string definition for message of type '<ServoCmd>"
  (format nil "#a servo command message~%float32 minAngle~%float32 maxAngle~%float32 velocity~%int32   id~%~%~%"))
(defmethod serialization-length ((msg <ServoCmd>))
  (+ 0
     4
     4
     4
     4
))
(defmethod ros-message-to-list ((msg <ServoCmd>))
  "Converts a ROS message object to a list"
  (list '<ServoCmd>
    (cons ':minAngle (minAngle-val msg))
    (cons ':maxAngle (maxAngle-val msg))
    (cons ':velocity (velocity-val msg))
    (cons ':id (id-val msg))
))
