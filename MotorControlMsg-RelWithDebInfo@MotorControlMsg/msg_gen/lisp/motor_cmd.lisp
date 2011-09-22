; Auto-generated. Do not edit!


(cl:in-package MotorControlMsg-msg)


;//! \htmlinclude motor_cmd.msg.html

(cl:defclass <motor_cmd> (roslisp-msg-protocol:ros-message)
  ((seq
    :reader seq
    :initarg :seq
    :type cl:integer
    :initform 0)
   (stamp
    :reader stamp
    :initarg :stamp
    :type cl:real
    :initform 0)
   (frame_id
    :reader frame_id
    :initarg :frame_id
    :type cl:string
    :initform "")
   (precedence
    :reader precedence
    :initarg :precedence
    :type cl:integer
    :initform 0)
   (x_velocity
    :reader x_velocity
    :initarg :x_velocity
    :type cl:float
    :initform 0.0)
   (y_velocity
    :reader y_velocity
    :initarg :y_velocity
    :type cl:float
    :initform 0.0))
)

(cl:defclass motor_cmd (<motor_cmd>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <motor_cmd>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'motor_cmd)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name MotorControlMsg-msg:<motor_cmd> is deprecated: use MotorControlMsg-msg:motor_cmd instead.")))

(cl:ensure-generic-function 'seq-val :lambda-list '(m))
(cl:defmethod seq-val ((m <motor_cmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader MotorControlMsg-msg:seq-val is deprecated.  Use MotorControlMsg-msg:seq instead.")
  (seq m))

(cl:ensure-generic-function 'stamp-val :lambda-list '(m))
(cl:defmethod stamp-val ((m <motor_cmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader MotorControlMsg-msg:stamp-val is deprecated.  Use MotorControlMsg-msg:stamp instead.")
  (stamp m))

(cl:ensure-generic-function 'frame_id-val :lambda-list '(m))
(cl:defmethod frame_id-val ((m <motor_cmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader MotorControlMsg-msg:frame_id-val is deprecated.  Use MotorControlMsg-msg:frame_id instead.")
  (frame_id m))

(cl:ensure-generic-function 'precedence-val :lambda-list '(m))
(cl:defmethod precedence-val ((m <motor_cmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader MotorControlMsg-msg:precedence-val is deprecated.  Use MotorControlMsg-msg:precedence instead.")
  (precedence m))

(cl:ensure-generic-function 'x_velocity-val :lambda-list '(m))
(cl:defmethod x_velocity-val ((m <motor_cmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader MotorControlMsg-msg:x_velocity-val is deprecated.  Use MotorControlMsg-msg:x_velocity instead.")
  (x_velocity m))

(cl:ensure-generic-function 'y_velocity-val :lambda-list '(m))
(cl:defmethod y_velocity-val ((m <motor_cmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader MotorControlMsg-msg:y_velocity-val is deprecated.  Use MotorControlMsg-msg:y_velocity instead.")
  (y_velocity m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <motor_cmd>) ostream)
  "Serializes a message object of type '<motor_cmd>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'seq)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'seq)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'seq)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'seq)) ostream)
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'stamp)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'stamp) (cl:floor (cl:slot-value msg 'stamp)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'frame_id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'frame_id))
  (cl:let* ((signed (cl:slot-value msg 'precedence)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x_velocity))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y_velocity))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <motor_cmd>) istream)
  "Deserializes a message object of type '<motor_cmd>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'seq)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'seq)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'seq)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'seq)) (cl:read-byte istream))
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'stamp) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'frame_id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'frame_id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'precedence) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x_velocity) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y_velocity) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<motor_cmd>)))
  "Returns string type for a message object of type '<motor_cmd>"
  "MotorControlMsg/motor_cmd")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'motor_cmd)))
  "Returns string type for a message object of type 'motor_cmd"
  "MotorControlMsg/motor_cmd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<motor_cmd>)))
  "Returns md5sum for a message object of type '<motor_cmd>"
  "9dc8c565ea1d6aa726eb1232cda46018")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'motor_cmd)))
  "Returns md5sum for a message object of type 'motor_cmd"
  "9dc8c565ea1d6aa726eb1232cda46018")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<motor_cmd>)))
  "Returns full string definition for message of type '<motor_cmd>"
  (cl:format cl:nil "#Standard metadata for higher-level flow data types~%#sequence ID: consecutively increasing ID~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%int32 precedence~%~%float32 x_velocity~%float32 y_velocity~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'motor_cmd)))
  "Returns full string definition for message of type 'motor_cmd"
  (cl:format cl:nil "#Standard metadata for higher-level flow data types~%#sequence ID: consecutively increasing ID~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%int32 precedence~%~%float32 x_velocity~%float32 y_velocity~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <motor_cmd>))
  (cl:+ 0
     4
     8
     4 (cl:length (cl:slot-value msg 'frame_id))
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <motor_cmd>))
  "Converts a ROS message object to a list"
  (cl:list 'motor_cmd
    (cl:cons ':seq (seq msg))
    (cl:cons ':stamp (stamp msg))
    (cl:cons ':frame_id (frame_id msg))
    (cl:cons ':precedence (precedence msg))
    (cl:cons ':x_velocity (x_velocity msg))
    (cl:cons ':y_velocity (y_velocity msg))
))
