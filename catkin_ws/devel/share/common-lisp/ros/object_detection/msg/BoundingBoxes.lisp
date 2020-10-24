; Auto-generated. Do not edit!


(cl:in-package object_detection-msg)


;//! \htmlinclude BoundingBoxes.msg.html

(cl:defclass <BoundingBoxes> (roslisp-msg-protocol:ros-message)
  ((BoundingBoxes
    :reader BoundingBoxes
    :initarg :BoundingBoxes
    :type (cl:vector object_detection-msg:BoundingBox)
   :initform (cl:make-array 0 :element-type 'object_detection-msg:BoundingBox :initial-element (cl:make-instance 'object_detection-msg:BoundingBox))))
)

(cl:defclass BoundingBoxes (<BoundingBoxes>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BoundingBoxes>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BoundingBoxes)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name object_detection-msg:<BoundingBoxes> is deprecated: use object_detection-msg:BoundingBoxes instead.")))

(cl:ensure-generic-function 'BoundingBoxes-val :lambda-list '(m))
(cl:defmethod BoundingBoxes-val ((m <BoundingBoxes>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader object_detection-msg:BoundingBoxes-val is deprecated.  Use object_detection-msg:BoundingBoxes instead.")
  (BoundingBoxes m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BoundingBoxes>) ostream)
  "Serializes a message object of type '<BoundingBoxes>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'BoundingBoxes))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'BoundingBoxes))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BoundingBoxes>) istream)
  "Deserializes a message object of type '<BoundingBoxes>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'BoundingBoxes) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'BoundingBoxes)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'object_detection-msg:BoundingBox))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BoundingBoxes>)))
  "Returns string type for a message object of type '<BoundingBoxes>"
  "object_detection/BoundingBoxes")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BoundingBoxes)))
  "Returns string type for a message object of type 'BoundingBoxes"
  "object_detection/BoundingBoxes")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BoundingBoxes>)))
  "Returns md5sum for a message object of type '<BoundingBoxes>"
  "a68dab7e3456d8b1363c13112d36861c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BoundingBoxes)))
  "Returns md5sum for a message object of type 'BoundingBoxes"
  "a68dab7e3456d8b1363c13112d36861c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BoundingBoxes>)))
  "Returns full string definition for message of type '<BoundingBoxes>"
  (cl:format cl:nil "BoundingBox[] BoundingBoxes~%~%================================================================================~%MSG: object_detection/BoundingBox~%float32 center_x~%float32 center_y~%float32 width~%float32 height~%float32 confidence~%float32 class_id~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BoundingBoxes)))
  "Returns full string definition for message of type 'BoundingBoxes"
  (cl:format cl:nil "BoundingBox[] BoundingBoxes~%~%================================================================================~%MSG: object_detection/BoundingBox~%float32 center_x~%float32 center_y~%float32 width~%float32 height~%float32 confidence~%float32 class_id~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BoundingBoxes>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'BoundingBoxes) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BoundingBoxes>))
  "Converts a ROS message object to a list"
  (cl:list 'BoundingBoxes
    (cl:cons ':BoundingBoxes (BoundingBoxes msg))
))
