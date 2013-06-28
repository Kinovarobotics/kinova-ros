; Auto-generated. Do not edit!


(cl:in-package jaco_driver-srv)


;//! \htmlinclude Estart-request.msg.html

(cl:defclass <Estart-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Estart-request (<Estart-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Estart-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Estart-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jaco_driver-srv:<Estart-request> is deprecated: use jaco_driver-srv:Estart-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Estart-request>) ostream)
  "Serializes a message object of type '<Estart-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Estart-request>) istream)
  "Deserializes a message object of type '<Estart-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Estart-request>)))
  "Returns string type for a service object of type '<Estart-request>"
  "jaco_driver/EstartRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Estart-request)))
  "Returns string type for a service object of type 'Estart-request"
  "jaco_driver/EstartRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Estart-request>)))
  "Returns md5sum for a message object of type '<Estart-request>"
  "869d428a7dc3ab7bd4a9c986929fc210")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Estart-request)))
  "Returns md5sum for a message object of type 'Estart-request"
  "869d428a7dc3ab7bd4a9c986929fc210")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Estart-request>)))
  "Returns full string definition for message of type '<Estart-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Estart-request)))
  "Returns full string definition for message of type 'Estart-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Estart-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Estart-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Estart-request
))
;//! \htmlinclude Estart-response.msg.html

(cl:defclass <Estart-response> (roslisp-msg-protocol:ros-message)
  ((estart_result
    :reader estart_result
    :initarg :estart_result
    :type cl:string
    :initform ""))
)

(cl:defclass Estart-response (<Estart-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Estart-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Estart-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jaco_driver-srv:<Estart-response> is deprecated: use jaco_driver-srv:Estart-response instead.")))

(cl:ensure-generic-function 'estart_result-val :lambda-list '(m))
(cl:defmethod estart_result-val ((m <Estart-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jaco_driver-srv:estart_result-val is deprecated.  Use jaco_driver-srv:estart_result instead.")
  (estart_result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Estart-response>) ostream)
  "Serializes a message object of type '<Estart-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'estart_result))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'estart_result))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Estart-response>) istream)
  "Deserializes a message object of type '<Estart-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'estart_result) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'estart_result) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Estart-response>)))
  "Returns string type for a service object of type '<Estart-response>"
  "jaco_driver/EstartResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Estart-response)))
  "Returns string type for a service object of type 'Estart-response"
  "jaco_driver/EstartResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Estart-response>)))
  "Returns md5sum for a message object of type '<Estart-response>"
  "869d428a7dc3ab7bd4a9c986929fc210")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Estart-response)))
  "Returns md5sum for a message object of type 'Estart-response"
  "869d428a7dc3ab7bd4a9c986929fc210")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Estart-response>)))
  "Returns full string definition for message of type '<Estart-response>"
  (cl:format cl:nil "string estart_result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Estart-response)))
  "Returns full string definition for message of type 'Estart-response"
  (cl:format cl:nil "string estart_result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Estart-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'estart_result))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Estart-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Estart-response
    (cl:cons ':estart_result (estart_result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Estart)))
  'Estart-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Estart)))
  'Estart-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Estart)))
  "Returns string type for a service object of type '<Estart>"
  "jaco_driver/Estart")