; Auto-generated. Do not edit!


(cl:in-package jaco_driver-srv)


;//! \htmlinclude Estop-request.msg.html

(cl:defclass <Estop-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Estop-request (<Estop-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Estop-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Estop-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jaco_driver-srv:<Estop-request> is deprecated: use jaco_driver-srv:Estop-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Estop-request>) ostream)
  "Serializes a message object of type '<Estop-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Estop-request>) istream)
  "Deserializes a message object of type '<Estop-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Estop-request>)))
  "Returns string type for a service object of type '<Estop-request>"
  "jaco_driver/EstopRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Estop-request)))
  "Returns string type for a service object of type 'Estop-request"
  "jaco_driver/EstopRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Estop-request>)))
  "Returns md5sum for a message object of type '<Estop-request>"
  "f64ab946586799a83eedd5cd4b0fa002")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Estop-request)))
  "Returns md5sum for a message object of type 'Estop-request"
  "f64ab946586799a83eedd5cd4b0fa002")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Estop-request>)))
  "Returns full string definition for message of type '<Estop-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Estop-request)))
  "Returns full string definition for message of type 'Estop-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Estop-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Estop-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Estop-request
))
;//! \htmlinclude Estop-response.msg.html

(cl:defclass <Estop-response> (roslisp-msg-protocol:ros-message)
  ((estop_result
    :reader estop_result
    :initarg :estop_result
    :type cl:string
    :initform ""))
)

(cl:defclass Estop-response (<Estop-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Estop-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Estop-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jaco_driver-srv:<Estop-response> is deprecated: use jaco_driver-srv:Estop-response instead.")))

(cl:ensure-generic-function 'estop_result-val :lambda-list '(m))
(cl:defmethod estop_result-val ((m <Estop-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jaco_driver-srv:estop_result-val is deprecated.  Use jaco_driver-srv:estop_result instead.")
  (estop_result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Estop-response>) ostream)
  "Serializes a message object of type '<Estop-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'estop_result))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'estop_result))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Estop-response>) istream)
  "Deserializes a message object of type '<Estop-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'estop_result) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'estop_result) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Estop-response>)))
  "Returns string type for a service object of type '<Estop-response>"
  "jaco_driver/EstopResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Estop-response)))
  "Returns string type for a service object of type 'Estop-response"
  "jaco_driver/EstopResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Estop-response>)))
  "Returns md5sum for a message object of type '<Estop-response>"
  "f64ab946586799a83eedd5cd4b0fa002")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Estop-response)))
  "Returns md5sum for a message object of type 'Estop-response"
  "f64ab946586799a83eedd5cd4b0fa002")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Estop-response>)))
  "Returns full string definition for message of type '<Estop-response>"
  (cl:format cl:nil "string estop_result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Estop-response)))
  "Returns full string definition for message of type 'Estop-response"
  (cl:format cl:nil "string estop_result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Estop-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'estop_result))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Estop-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Estop-response
    (cl:cons ':estop_result (estop_result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Estop)))
  'Estop-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Estop)))
  'Estop-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Estop)))
  "Returns string type for a service object of type '<Estop>"
  "jaco_driver/Estop")