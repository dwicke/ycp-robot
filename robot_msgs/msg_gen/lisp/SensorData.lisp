; Auto-generated. Do not edit!


(cl:in-package robot_msgs-msg)


;//! \htmlinclude SensorData.msg.html

(cl:defclass <SensorData> (roslisp-msg-protocol:ros-message)
  ((ultrasonic_frontLeft_distance
    :reader ultrasonic_frontLeft_distance
    :initarg :ultrasonic_frontLeft_distance
    :type cl:fixnum
    :initform 0)
   (ultrasonic_frontCenter_distance
    :reader ultrasonic_frontCenter_distance
    :initarg :ultrasonic_frontCenter_distance
    :type cl:fixnum
    :initform 0)
   (ultrasonic_frontRight_distance
    :reader ultrasonic_frontRight_distance
    :initarg :ultrasonic_frontRight_distance
    :type cl:fixnum
    :initform 0)
   (ultrasonic_rearRight_distance
    :reader ultrasonic_rearRight_distance
    :initarg :ultrasonic_rearRight_distance
    :type cl:fixnum
    :initform 0)
   (ultrasonic_rearCenter_distance
    :reader ultrasonic_rearCenter_distance
    :initarg :ultrasonic_rearCenter_distance
    :type cl:fixnum
    :initform 0)
   (ultrasonic_rearLeft_distance
    :reader ultrasonic_rearLeft_distance
    :initarg :ultrasonic_rearLeft_distance
    :type cl:fixnum
    :initform 0)
   (infrared_frontLeftLeft_distance
    :reader infrared_frontLeftLeft_distance
    :initarg :infrared_frontLeftLeft_distance
    :type cl:float
    :initform 0.0)
   (infrared_frontLeftCenter_distance
    :reader infrared_frontLeftCenter_distance
    :initarg :infrared_frontLeftCenter_distance
    :type cl:float
    :initform 0.0)
   (infrared_frontRightCenter_distance
    :reader infrared_frontRightCenter_distance
    :initarg :infrared_frontRightCenter_distance
    :type cl:float
    :initform 0.0)
   (infrared_frontRightRight_distance
    :reader infrared_frontRightRight_distance
    :initarg :infrared_frontRightRight_distance
    :type cl:float
    :initform 0.0)
   (infrared_right_distance
    :reader infrared_right_distance
    :initarg :infrared_right_distance
    :type cl:float
    :initform 0.0)
   (infrared_rear_distance
    :reader infrared_rear_distance
    :initarg :infrared_rear_distance
    :type cl:float
    :initform 0.0)
   (infrared_left_distance
    :reader infrared_left_distance
    :initarg :infrared_left_distance
    :type cl:float
    :initform 0.0)
   (human_left_motion
    :reader human_left_motion
    :initarg :human_left_motion
    :type cl:fixnum
    :initform 0)
   (human_left_presence
    :reader human_left_presence
    :initarg :human_left_presence
    :type cl:fixnum
    :initform 0)
   (human_right_motion
    :reader human_right_motion
    :initarg :human_right_motion
    :type cl:fixnum
    :initform 0)
   (human_right_presence
    :reader human_right_presence
    :initarg :human_right_presence
    :type cl:fixnum
    :initform 0))
)

(cl:defclass SensorData (<SensorData>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SensorData>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SensorData)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_msgs-msg:<SensorData> is deprecated: use robot_msgs-msg:SensorData instead.")))

(cl:ensure-generic-function 'ultrasonic_frontLeft_distance-val :lambda-list '(m))
(cl:defmethod ultrasonic_frontLeft_distance-val ((m <SensorData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_msgs-msg:ultrasonic_frontLeft_distance-val is deprecated.  Use robot_msgs-msg:ultrasonic_frontLeft_distance instead.")
  (ultrasonic_frontLeft_distance m))

(cl:ensure-generic-function 'ultrasonic_frontCenter_distance-val :lambda-list '(m))
(cl:defmethod ultrasonic_frontCenter_distance-val ((m <SensorData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_msgs-msg:ultrasonic_frontCenter_distance-val is deprecated.  Use robot_msgs-msg:ultrasonic_frontCenter_distance instead.")
  (ultrasonic_frontCenter_distance m))

(cl:ensure-generic-function 'ultrasonic_frontRight_distance-val :lambda-list '(m))
(cl:defmethod ultrasonic_frontRight_distance-val ((m <SensorData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_msgs-msg:ultrasonic_frontRight_distance-val is deprecated.  Use robot_msgs-msg:ultrasonic_frontRight_distance instead.")
  (ultrasonic_frontRight_distance m))

(cl:ensure-generic-function 'ultrasonic_rearRight_distance-val :lambda-list '(m))
(cl:defmethod ultrasonic_rearRight_distance-val ((m <SensorData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_msgs-msg:ultrasonic_rearRight_distance-val is deprecated.  Use robot_msgs-msg:ultrasonic_rearRight_distance instead.")
  (ultrasonic_rearRight_distance m))

(cl:ensure-generic-function 'ultrasonic_rearCenter_distance-val :lambda-list '(m))
(cl:defmethod ultrasonic_rearCenter_distance-val ((m <SensorData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_msgs-msg:ultrasonic_rearCenter_distance-val is deprecated.  Use robot_msgs-msg:ultrasonic_rearCenter_distance instead.")
  (ultrasonic_rearCenter_distance m))

(cl:ensure-generic-function 'ultrasonic_rearLeft_distance-val :lambda-list '(m))
(cl:defmethod ultrasonic_rearLeft_distance-val ((m <SensorData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_msgs-msg:ultrasonic_rearLeft_distance-val is deprecated.  Use robot_msgs-msg:ultrasonic_rearLeft_distance instead.")
  (ultrasonic_rearLeft_distance m))

(cl:ensure-generic-function 'infrared_frontLeftLeft_distance-val :lambda-list '(m))
(cl:defmethod infrared_frontLeftLeft_distance-val ((m <SensorData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_msgs-msg:infrared_frontLeftLeft_distance-val is deprecated.  Use robot_msgs-msg:infrared_frontLeftLeft_distance instead.")
  (infrared_frontLeftLeft_distance m))

(cl:ensure-generic-function 'infrared_frontLeftCenter_distance-val :lambda-list '(m))
(cl:defmethod infrared_frontLeftCenter_distance-val ((m <SensorData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_msgs-msg:infrared_frontLeftCenter_distance-val is deprecated.  Use robot_msgs-msg:infrared_frontLeftCenter_distance instead.")
  (infrared_frontLeftCenter_distance m))

(cl:ensure-generic-function 'infrared_frontRightCenter_distance-val :lambda-list '(m))
(cl:defmethod infrared_frontRightCenter_distance-val ((m <SensorData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_msgs-msg:infrared_frontRightCenter_distance-val is deprecated.  Use robot_msgs-msg:infrared_frontRightCenter_distance instead.")
  (infrared_frontRightCenter_distance m))

(cl:ensure-generic-function 'infrared_frontRightRight_distance-val :lambda-list '(m))
(cl:defmethod infrared_frontRightRight_distance-val ((m <SensorData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_msgs-msg:infrared_frontRightRight_distance-val is deprecated.  Use robot_msgs-msg:infrared_frontRightRight_distance instead.")
  (infrared_frontRightRight_distance m))

(cl:ensure-generic-function 'infrared_right_distance-val :lambda-list '(m))
(cl:defmethod infrared_right_distance-val ((m <SensorData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_msgs-msg:infrared_right_distance-val is deprecated.  Use robot_msgs-msg:infrared_right_distance instead.")
  (infrared_right_distance m))

(cl:ensure-generic-function 'infrared_rear_distance-val :lambda-list '(m))
(cl:defmethod infrared_rear_distance-val ((m <SensorData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_msgs-msg:infrared_rear_distance-val is deprecated.  Use robot_msgs-msg:infrared_rear_distance instead.")
  (infrared_rear_distance m))

(cl:ensure-generic-function 'infrared_left_distance-val :lambda-list '(m))
(cl:defmethod infrared_left_distance-val ((m <SensorData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_msgs-msg:infrared_left_distance-val is deprecated.  Use robot_msgs-msg:infrared_left_distance instead.")
  (infrared_left_distance m))

(cl:ensure-generic-function 'human_left_motion-val :lambda-list '(m))
(cl:defmethod human_left_motion-val ((m <SensorData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_msgs-msg:human_left_motion-val is deprecated.  Use robot_msgs-msg:human_left_motion instead.")
  (human_left_motion m))

(cl:ensure-generic-function 'human_left_presence-val :lambda-list '(m))
(cl:defmethod human_left_presence-val ((m <SensorData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_msgs-msg:human_left_presence-val is deprecated.  Use robot_msgs-msg:human_left_presence instead.")
  (human_left_presence m))

(cl:ensure-generic-function 'human_right_motion-val :lambda-list '(m))
(cl:defmethod human_right_motion-val ((m <SensorData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_msgs-msg:human_right_motion-val is deprecated.  Use robot_msgs-msg:human_right_motion instead.")
  (human_right_motion m))

(cl:ensure-generic-function 'human_right_presence-val :lambda-list '(m))
(cl:defmethod human_right_presence-val ((m <SensorData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_msgs-msg:human_right_presence-val is deprecated.  Use robot_msgs-msg:human_right_presence instead.")
  (human_right_presence m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SensorData>) ostream)
  "Serializes a message object of type '<SensorData>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ultrasonic_frontLeft_distance)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ultrasonic_frontCenter_distance)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ultrasonic_frontRight_distance)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ultrasonic_rearRight_distance)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ultrasonic_rearCenter_distance)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ultrasonic_rearLeft_distance)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'infrared_frontLeftLeft_distance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'infrared_frontLeftCenter_distance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'infrared_frontRightCenter_distance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'infrared_frontRightRight_distance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'infrared_right_distance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'infrared_rear_distance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'infrared_left_distance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'human_left_motion)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'human_left_motion)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'human_left_presence)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'human_left_presence)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'human_right_motion)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'human_right_motion)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'human_right_presence)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'human_right_presence)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SensorData>) istream)
  "Deserializes a message object of type '<SensorData>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ultrasonic_frontLeft_distance)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ultrasonic_frontCenter_distance)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ultrasonic_frontRight_distance)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ultrasonic_rearRight_distance)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ultrasonic_rearCenter_distance)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ultrasonic_rearLeft_distance)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'infrared_frontLeftLeft_distance) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'infrared_frontLeftCenter_distance) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'infrared_frontRightCenter_distance) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'infrared_frontRightRight_distance) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'infrared_right_distance) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'infrared_rear_distance) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'infrared_left_distance) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'human_left_motion)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'human_left_motion)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'human_left_presence)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'human_left_presence)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'human_right_motion)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'human_right_motion)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'human_right_presence)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'human_right_presence)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SensorData>)))
  "Returns string type for a message object of type '<SensorData>"
  "robot_msgs/SensorData")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SensorData)))
  "Returns string type for a message object of type 'SensorData"
  "robot_msgs/SensorData")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SensorData>)))
  "Returns md5sum for a message object of type '<SensorData>"
  "0720f31a3880984759c88032ef6503d8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SensorData)))
  "Returns md5sum for a message object of type 'SensorData"
  "0720f31a3880984759c88032ef6503d8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SensorData>)))
  "Returns full string definition for message of type '<SensorData>"
  (cl:format cl:nil "uint8 ultrasonic_frontLeft_distance~%uint8 ultrasonic_frontCenter_distance~%uint8 ultrasonic_frontRight_distance~%uint8 ultrasonic_rearRight_distance~%uint8 ultrasonic_rearCenter_distance~%uint8 ultrasonic_rearLeft_distance~%~%float32 infrared_frontLeftLeft_distance~%float32 infrared_frontLeftCenter_distance~%float32 infrared_frontRightCenter_distance~%float32 infrared_frontRightRight_distance~%float32 infrared_right_distance~%float32 infrared_rear_distance~%float32 infrared_left_distance~%~%uint16 human_left_motion~%uint16 human_left_presence~%uint16 human_right_motion~%uint16 human_right_presence~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SensorData)))
  "Returns full string definition for message of type 'SensorData"
  (cl:format cl:nil "uint8 ultrasonic_frontLeft_distance~%uint8 ultrasonic_frontCenter_distance~%uint8 ultrasonic_frontRight_distance~%uint8 ultrasonic_rearRight_distance~%uint8 ultrasonic_rearCenter_distance~%uint8 ultrasonic_rearLeft_distance~%~%float32 infrared_frontLeftLeft_distance~%float32 infrared_frontLeftCenter_distance~%float32 infrared_frontRightCenter_distance~%float32 infrared_frontRightRight_distance~%float32 infrared_right_distance~%float32 infrared_rear_distance~%float32 infrared_left_distance~%~%uint16 human_left_motion~%uint16 human_left_presence~%uint16 human_right_motion~%uint16 human_right_presence~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SensorData>))
  (cl:+ 0
     1
     1
     1
     1
     1
     1
     4
     4
     4
     4
     4
     4
     4
     2
     2
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SensorData>))
  "Converts a ROS message object to a list"
  (cl:list 'SensorData
    (cl:cons ':ultrasonic_frontLeft_distance (ultrasonic_frontLeft_distance msg))
    (cl:cons ':ultrasonic_frontCenter_distance (ultrasonic_frontCenter_distance msg))
    (cl:cons ':ultrasonic_frontRight_distance (ultrasonic_frontRight_distance msg))
    (cl:cons ':ultrasonic_rearRight_distance (ultrasonic_rearRight_distance msg))
    (cl:cons ':ultrasonic_rearCenter_distance (ultrasonic_rearCenter_distance msg))
    (cl:cons ':ultrasonic_rearLeft_distance (ultrasonic_rearLeft_distance msg))
    (cl:cons ':infrared_frontLeftLeft_distance (infrared_frontLeftLeft_distance msg))
    (cl:cons ':infrared_frontLeftCenter_distance (infrared_frontLeftCenter_distance msg))
    (cl:cons ':infrared_frontRightCenter_distance (infrared_frontRightCenter_distance msg))
    (cl:cons ':infrared_frontRightRight_distance (infrared_frontRightRight_distance msg))
    (cl:cons ':infrared_right_distance (infrared_right_distance msg))
    (cl:cons ':infrared_rear_distance (infrared_rear_distance msg))
    (cl:cons ':infrared_left_distance (infrared_left_distance msg))
    (cl:cons ':human_left_motion (human_left_motion msg))
    (cl:cons ':human_left_presence (human_left_presence msg))
    (cl:cons ':human_right_motion (human_right_motion msg))
    (cl:cons ':human_right_presence (human_right_presence msg))
))
