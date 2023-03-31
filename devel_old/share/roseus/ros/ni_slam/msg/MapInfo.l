;; Auto-generated. Do not edit!


(when (boundp 'ni_slam::MapInfo)
  (if (not (find-package "NI_SLAM"))
    (make-package "NI_SLAM"))
  (shadow 'MapInfo (find-package "NI_SLAM")))
(unless (find-package "NI_SLAM::MAPINFO")
  (make-package "NI_SLAM::MAPINFO"))

(in-package "ROS")
;;//! \htmlinclude MapInfo.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass ni_slam::MapInfo
  :super ros::object
  :slots (_header _resolution _width _height _origin ))

(defmethod ni_slam::MapInfo
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:resolution __resolution) 0.0)
    ((:width __width) 0)
    ((:height __height) 0)
    ((:origin __origin) (instance geometry_msgs::Pose :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _resolution (float __resolution))
   (setq _width (round __width))
   (setq _height (round __height))
   (setq _origin __origin)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:resolution
   (&optional __resolution)
   (if __resolution (setq _resolution __resolution)) _resolution)
  (:width
   (&optional __width)
   (if __width (setq _width __width)) _width)
  (:height
   (&optional __height)
   (if __height (setq _height __height)) _height)
  (:origin
   (&rest __origin)
   (if (keywordp (car __origin))
       (send* _origin __origin)
     (progn
       (if __origin (setq _origin (car __origin)))
       _origin)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float32 _resolution
    4
    ;; uint32 _width
    4
    ;; uint32 _height
    4
    ;; geometry_msgs/Pose _origin
    (send _origin :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float32 _resolution
       (sys::poke _resolution (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint32 _width
       (write-long _width s)
     ;; uint32 _height
       (write-long _height s)
     ;; geometry_msgs/Pose _origin
       (send _origin :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float32 _resolution
     (setq _resolution (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint32 _width
     (setq _width (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _height
     (setq _height (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; geometry_msgs/Pose _origin
     (send _origin :deserialize buf ptr-) (incf ptr- (send _origin :serialization-length))
   ;;
   self)
  )

(setf (get ni_slam::MapInfo :md5sum-) "21930ed739995b1ba31aa9c9a8e1cc31")
(setf (get ni_slam::MapInfo :datatype-) "ni_slam/MapInfo")
(setf (get ni_slam::MapInfo :definition-)
      "# This hold basic information about the characterists of each sub-map the ni-slam
Header header 
float32 resolution             # The map resolution [m/cell]
uint32 width                   # Map width [cells]
uint32 height                  # Map height [cells]
geometry_msgs/Pose origin      # The origin of the map.
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

")



(provide :ni_slam/MapInfo "21930ed739995b1ba31aa9c9a8e1cc31")


