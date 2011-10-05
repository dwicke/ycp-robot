; Auto-generated. Do not edit!


(cl:in-package sim-msg)


;//! \htmlinclude MotorData.msg.html

(cl:defclass <MotorData> (roslisp-msg-protocol:ros-message)
  ((motor_left_velocity
    :reader motor_left_velocity
    :initarg :motor_left_velocity
    :type cl:float
    :initform 0.0)
   (motor_left_time
    :reader motor_left_time
    :initarg :motor_left_time
    :type cl:fixnum
    :initform 0)
   (motor_right_velocity
    :reader motor_right_velocity
    :initarg :motor_right_velocity
    :type cl:float
    :initform 0.0)
   (motor_right_time
    :reader motor_right_time
    :initarg :motor_right_time
    :type cl:fixnum
    :initform 0))
)

(cl:defclass MotorData (<MotorData>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MotorData>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MotorData)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sim-msg:<MotorData> is deprecated: use sim-msg:MotorData instead.")))

(cl:ensure-generic-function 'motor_left_velocity-val :lambda-list '(m))
(cl:defmethod motor_left_velocity-val ((m <MotorData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sim-msg:motor_left_velocity-val is deprecated.  Use sim-msg:motor_left_velocity instead.")
  (motor_left_velocity m))

(cl:ensure-generic-function 'motor_left_time-val :lambda-list '(m))
(cl:defmethod motor_left_time-val ((m <MotorData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sim-msg:motor_left_time-val is deprecated.  Use sim-msg:motor_left_time instead.")
  (motor_left_time m))

(cl:ensure-generic-function 'motor_right_velocity-val :lambda-list '(m))
(cl:defmethod motor_right_velocity-val ((m <MotorData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sim-msg:motor_right_velocity-val is deprecated.  Use sim-msg:motor_right_velocity instead.")
  (motor_right_velocity m))

(cl:ensure-generic-function 'motor_right_time-val :lambda-list '(m))
(cl:defmethod motor_right_time-val ((m <MotorData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sim-msg:motor_right_time-val is deprecated.  Use sim-msg:motor_right_time instead.")
  (motor_right_time m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MotorData>) ostream)
  "Serializes a message object of type '<MotorData>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'motor_left_velocity))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'motor_left_time)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'motor_left_time)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'motor_right_velocity))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'motor_right_time)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'motor_right_time)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MotorData>) istream)
  "Deserializes a message object of type '<MotorData>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'motor_left_velocity) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'motor_left_time)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'motor_left_time)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'motor_right_velocity) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'motor_right_time)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'motor_right_time)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MotorData>)))
  "Returns string type for a message object of type '<MotorData>"
  "sim/MotorData")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MotorData)))
  "Returns string type for a message object of type 'MotorData"
  "sim/MotorData")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MotorData>)))
  "Returns md5sum for a message object of type '<MotorData>"
  "2c057021fd288de5b88a00bb6f218a61")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MotorData)))
  "Returns md5sum for a message object of type 'MotorData"
  "2c057021fd288de5b88a00bb6f218a61")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MotorData>)))
  "Returns full string definition for message of type '<MotorData>"
  (cl:format cl:nil "float32 motor_left_velocity~%uint16 motor_left_time~%float32 motor_right_velocity~%uint16 motor_right_time~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MotorData)))
  "Returns full string definition for message of type 'MotorData"
  (cl:format cl:nil "float32 motor_left_velocity~%uint16 motor_left_time~%float32 motor_right_velocity~%uint16 motor_right_time~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MotorData>))
  (cl:+ 0
     4
     2
     4
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MotorData>))
  "Converts a ROS message object to a list"
  (cl:list 'MotorData
    (cl:cons ':motor_left_velocity (motor_left_velocity msg))
    (cl:cons ':motor_left_time (motor_left_time msg))
    (cl:cons ':motor_right_velocity (motor_right_velocity msg))
    (cl:cons ':motor_right_time (motor_right_time msg))
))
