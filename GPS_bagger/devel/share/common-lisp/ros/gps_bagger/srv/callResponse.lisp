; Auto-generated. Do not edit!


(cl:in-package gps_bagger-srv)


;//! \htmlinclude callResponse-request.msg.html

(cl:defclass <callResponse-request> (roslisp-msg-protocol:ros-message)
  ((command
    :reader command
    :initarg :command
    :type cl:string
    :initform ""))
)

(cl:defclass callResponse-request (<callResponse-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <callResponse-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'callResponse-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name gps_bagger-srv:<callResponse-request> is deprecated: use gps_bagger-srv:callResponse-request instead.")))

(cl:ensure-generic-function 'command-val :lambda-list '(m))
(cl:defmethod command-val ((m <callResponse-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gps_bagger-srv:command-val is deprecated.  Use gps_bagger-srv:command instead.")
  (command m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <callResponse-request>) ostream)
  "Serializes a message object of type '<callResponse-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'command))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'command))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <callResponse-request>) istream)
  "Deserializes a message object of type '<callResponse-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'command) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'command) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<callResponse-request>)))
  "Returns string type for a service object of type '<callResponse-request>"
  "gps_bagger/callResponseRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'callResponse-request)))
  "Returns string type for a service object of type 'callResponse-request"
  "gps_bagger/callResponseRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<callResponse-request>)))
  "Returns md5sum for a message object of type '<callResponse-request>"
  "22c7c465d64c7e74c6ae22029c7ca150")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'callResponse-request)))
  "Returns md5sum for a message object of type 'callResponse-request"
  "22c7c465d64c7e74c6ae22029c7ca150")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<callResponse-request>)))
  "Returns full string definition for message of type '<callResponse-request>"
  (cl:format cl:nil "# Request~%string command~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'callResponse-request)))
  "Returns full string definition for message of type 'callResponse-request"
  (cl:format cl:nil "# Request~%string command~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <callResponse-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'command))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <callResponse-request>))
  "Converts a ROS message object to a list"
  (cl:list 'callResponse-request
    (cl:cons ':command (command msg))
))
;//! \htmlinclude callResponse-response.msg.html

(cl:defclass <callResponse-response> (roslisp-msg-protocol:ros-message)
  ((response
    :reader response
    :initarg :response
    :type cl:string
    :initform ""))
)

(cl:defclass callResponse-response (<callResponse-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <callResponse-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'callResponse-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name gps_bagger-srv:<callResponse-response> is deprecated: use gps_bagger-srv:callResponse-response instead.")))

(cl:ensure-generic-function 'response-val :lambda-list '(m))
(cl:defmethod response-val ((m <callResponse-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gps_bagger-srv:response-val is deprecated.  Use gps_bagger-srv:response instead.")
  (response m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <callResponse-response>) ostream)
  "Serializes a message object of type '<callResponse-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'response))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'response))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <callResponse-response>) istream)
  "Deserializes a message object of type '<callResponse-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'response) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'response) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<callResponse-response>)))
  "Returns string type for a service object of type '<callResponse-response>"
  "gps_bagger/callResponseResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'callResponse-response)))
  "Returns string type for a service object of type 'callResponse-response"
  "gps_bagger/callResponseResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<callResponse-response>)))
  "Returns md5sum for a message object of type '<callResponse-response>"
  "22c7c465d64c7e74c6ae22029c7ca150")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'callResponse-response)))
  "Returns md5sum for a message object of type 'callResponse-response"
  "22c7c465d64c7e74c6ae22029c7ca150")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<callResponse-response>)))
  "Returns full string definition for message of type '<callResponse-response>"
  (cl:format cl:nil "string response~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'callResponse-response)))
  "Returns full string definition for message of type 'callResponse-response"
  (cl:format cl:nil "string response~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <callResponse-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'response))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <callResponse-response>))
  "Converts a ROS message object to a list"
  (cl:list 'callResponse-response
    (cl:cons ':response (response msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'callResponse)))
  'callResponse-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'callResponse)))
  'callResponse-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'callResponse)))
  "Returns string type for a service object of type '<callResponse>"
  "gps_bagger/callResponse")