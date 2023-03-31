; Auto-generated. Do not edit!


(cl:in-package ni_slam-msg)


;//! \htmlinclude MapInfo.msg.html

(cl:defclass <MapInfo> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (resolution
    :reader resolution
    :initarg :resolution
    :type cl:float
    :initform 0.0)
   (width
    :reader width
    :initarg :width
    :type cl:integer
    :initform 0)
   (height
    :reader height
    :initarg :height
    :type cl:integer
    :initform 0)
   (origin
    :reader origin
    :initarg :origin
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose)))
)

(cl:defclass MapInfo (<MapInfo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MapInfo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MapInfo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ni_slam-msg:<MapInfo> is deprecated: use ni_slam-msg:MapInfo instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <MapInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ni_slam-msg:header-val is deprecated.  Use ni_slam-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'resolution-val :lambda-list '(m))
(cl:defmethod resolution-val ((m <MapInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ni_slam-msg:resolution-val is deprecated.  Use ni_slam-msg:resolution instead.")
  (resolution m))

(cl:ensure-generic-function 'width-val :lambda-list '(m))
(cl:defmethod width-val ((m <MapInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ni_slam-msg:width-val is deprecated.  Use ni_slam-msg:width instead.")
  (width m))

(cl:ensure-generic-function 'height-val :lambda-list '(m))
(cl:defmethod height-val ((m <MapInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ni_slam-msg:height-val is deprecated.  Use ni_slam-msg:height instead.")
  (height m))

(cl:ensure-generic-function 'origin-val :lambda-list '(m))
(cl:defmethod origin-val ((m <MapInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ni_slam-msg:origin-val is deprecated.  Use ni_slam-msg:origin instead.")
  (origin m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MapInfo>) ostream)
  "Serializes a message object of type '<MapInfo>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'resolution))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'height)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'height)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'height)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'height)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'origin) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MapInfo>) istream)
  "Deserializes a message object of type '<MapInfo>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'resolution) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'height)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'height)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'height)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'height)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'origin) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MapInfo>)))
  "Returns string type for a message object of type '<MapInfo>"
  "ni_slam/MapInfo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MapInfo)))
  "Returns string type for a message object of type 'MapInfo"
  "ni_slam/MapInfo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MapInfo>)))
  "Returns md5sum for a message object of type '<MapInfo>"
  "21930ed739995b1ba31aa9c9a8e1cc31")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MapInfo)))
  "Returns md5sum for a message object of type 'MapInfo"
  "21930ed739995b1ba31aa9c9a8e1cc31")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MapInfo>)))
  "Returns full string definition for message of type '<MapInfo>"
  (cl:format cl:nil "# This hold basic information about the characterists of each sub-map the ni-slam~%Header header ~%float32 resolution             # The map resolution [m/cell]~%uint32 width                   # Map width [cells]~%uint32 height                  # Map height [cells]~%geometry_msgs/Pose origin      # The origin of the map.~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MapInfo)))
  "Returns full string definition for message of type 'MapInfo"
  (cl:format cl:nil "# This hold basic information about the characterists of each sub-map the ni-slam~%Header header ~%float32 resolution             # The map resolution [m/cell]~%uint32 width                   # Map width [cells]~%uint32 height                  # Map height [cells]~%geometry_msgs/Pose origin      # The origin of the map.~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MapInfo>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'origin))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MapInfo>))
  "Converts a ROS message object to a list"
  (cl:list 'MapInfo
    (cl:cons ':header (header msg))
    (cl:cons ':resolution (resolution msg))
    (cl:cons ':width (width msg))
    (cl:cons ':height (height msg))
    (cl:cons ':origin (origin msg))
))
