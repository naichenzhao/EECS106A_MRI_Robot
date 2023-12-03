; Auto-generated. Do not edit!


(cl:in-package planner-msg)


;//! \htmlinclude GantryPath.msg.html

(cl:defclass <GantryPath> (roslisp-msg-protocol:ros-message)
  ((path
    :reader path
    :initarg :path
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass GantryPath (<GantryPath>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GantryPath>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GantryPath)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name planner-msg:<GantryPath> is deprecated: use planner-msg:GantryPath instead.")))

(cl:ensure-generic-function 'path-val :lambda-list '(m))
(cl:defmethod path-val ((m <GantryPath>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader planner-msg:path-val is deprecated.  Use planner-msg:path instead.")
  (path m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GantryPath>) ostream)
  "Serializes a message object of type '<GantryPath>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'path))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'path))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GantryPath>) istream)
  "Deserializes a message object of type '<GantryPath>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'path) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'path)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GantryPath>)))
  "Returns string type for a message object of type '<GantryPath>"
  "planner/GantryPath")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GantryPath)))
  "Returns string type for a message object of type 'GantryPath"
  "planner/GantryPath")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GantryPath>)))
  "Returns md5sum for a message object of type '<GantryPath>"
  "48ba10a81da6976bce0a9617ff2a025d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GantryPath)))
  "Returns md5sum for a message object of type 'GantryPath"
  "48ba10a81da6976bce0a9617ff2a025d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GantryPath>)))
  "Returns full string definition for message of type '<GantryPath>"
  (cl:format cl:nil "float64[] path~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GantryPath)))
  "Returns full string definition for message of type 'GantryPath"
  (cl:format cl:nil "float64[] path~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GantryPath>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'path) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GantryPath>))
  "Converts a ROS message object to a list"
  (cl:list 'GantryPath
    (cl:cons ':path (path msg))
))
