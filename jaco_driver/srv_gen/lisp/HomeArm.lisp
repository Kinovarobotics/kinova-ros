; Auto-generated. Do not edit!


(cl:in-package jaco_driver-srv)


;//! \htmlinclude HomeArm-request.msg.html

(cl:defclass <HomeArm-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass HomeArm-request (<HomeArm-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <HomeArm-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'HomeArm-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jaco_driver-srv:<HomeArm-request> is deprecated: use jaco_driver-srv:HomeArm-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <HomeArm-request>) ostream)
  "Serializes a message object of type '<HomeArm-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <HomeArm-request>) istream)
  "Deserializes a message object of type '<HomeArm-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<HomeArm-request>)))
  "Returns string type for a service object of type '<HomeArm-request>"
  "jaco_driver/HomeArmRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HomeArm-request)))
  "Returns string type for a service object of type 'HomeArm-request"
  "jaco_driver/HomeArmRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<HomeArm-request>)))
  "Returns md5sum for a message object of type '<HomeArm-request>"
  "46e470f2c1a7177398c57a43eafe8d67")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'HomeArm-request)))
  "Returns md5sum for a message object of type 'HomeArm-request"
  "46e470f2c1a7177398c57a43eafe8d67")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<HomeArm-request>)))
  "Returns full string definition for message of type '<HomeArm-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'HomeArm-request)))
  "Returns full string definition for message of type 'HomeArm-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <HomeArm-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <HomeArm-request>))
  "Converts a ROS message object to a list"
  (cl:list 'HomeArm-request
))
;//! \htmlinclude HomeArm-response.msg.html

(cl:defclass <HomeArm-response> (roslisp-msg-protocol:ros-message)
  ((homearm_result
    :reader homearm_result
    :initarg :homearm_result
    :type cl:string
    :initform ""))
)

(cl:defclass HomeArm-response (<HomeArm-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <HomeArm-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'HomeArm-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jaco_driver-srv:<HomeArm-response> is deprecated: use jaco_driver-srv:HomeArm-response instead.")))

(cl:ensure-generic-function 'homearm_result-val :lambda-list '(m))
(cl:defmethod homearm_result-val ((m <HomeArm-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jaco_driver-srv:homearm_result-val is deprecated.  Use jaco_driver-srv:homearm_result instead.")
  (homearm_result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <HomeArm-response>) ostream)
  "Serializes a message object of type '<HomeArm-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'homearm_result))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'homearm_result))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <HomeArm-response>) istream)
  "Deserializes a message object of type '<HomeArm-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'homearm_result) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'homearm_result) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<HomeArm-response>)))
  "Returns string type for a service object of type '<HomeArm-response>"
  "jaco_driver/HomeArmResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HomeArm-response)))
  "Returns string type for a service object of type 'HomeArm-response"
  "jaco_driver/HomeArmResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<HomeArm-response>)))
  "Returns md5sum for a message object of type '<HomeArm-response>"
  "46e470f2c1a7177398c57a43eafe8d67")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'HomeArm-response)))
  "Returns md5sum for a message object of type 'HomeArm-response"
  "46e470f2c1a7177398c57a43eafe8d67")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<HomeArm-response>)))
  "Returns full string definition for message of type '<HomeArm-response>"
  (cl:format cl:nil "string homearm_result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'HomeArm-response)))
  "Returns full string definition for message of type 'HomeArm-response"
  (cl:format cl:nil "string homearm_result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <HomeArm-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'homearm_result))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <HomeArm-response>))
  "Converts a ROS message object to a list"
  (cl:list 'HomeArm-response
    (cl:cons ':homearm_result (homearm_result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'HomeArm)))
  'HomeArm-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'HomeArm)))
  'HomeArm-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HomeArm)))
  "Returns string type for a service object of type '<HomeArm>"
  "jaco_driver/HomeArm")