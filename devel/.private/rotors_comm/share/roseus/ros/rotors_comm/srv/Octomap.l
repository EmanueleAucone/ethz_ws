;; Auto-generated. Do not edit!


(when (boundp 'rotors_comm::Octomap)
  (if (not (find-package "ROTORS_COMM"))
    (make-package "ROTORS_COMM"))
  (shadow 'Octomap (find-package "ROTORS_COMM")))
(unless (find-package "ROTORS_COMM::OCTOMAP")
  (make-package "ROTORS_COMM::OCTOMAP"))
(unless (find-package "ROTORS_COMM::OCTOMAPREQUEST")
  (make-package "ROTORS_COMM::OCTOMAPREQUEST"))
(unless (find-package "ROTORS_COMM::OCTOMAPRESPONSE")
  (make-package "ROTORS_COMM::OCTOMAPRESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(if (not (find-package "OCTOMAP_MSGS"))
  (ros::roseus-add-msgs "octomap_msgs"))


(defclass rotors_comm::OctomapRequest
  :super ros::object
  :slots (_bounding_box_origin _bounding_box_lengths _leaf_size _publish_octomap _filename ))

(defmethod rotors_comm::OctomapRequest
  (:init
   (&key
    ((:bounding_box_origin __bounding_box_origin) (instance geometry_msgs::Point :init))
    ((:bounding_box_lengths __bounding_box_lengths) (instance geometry_msgs::Point :init))
    ((:leaf_size __leaf_size) 0.0)
    ((:publish_octomap __publish_octomap) nil)
    ((:filename __filename) "")
    )
   (send-super :init)
   (setq _bounding_box_origin __bounding_box_origin)
   (setq _bounding_box_lengths __bounding_box_lengths)
   (setq _leaf_size (float __leaf_size))
   (setq _publish_octomap __publish_octomap)
   (setq _filename (string __filename))
   self)
  (:bounding_box_origin
   (&rest __bounding_box_origin)
   (if (keywordp (car __bounding_box_origin))
       (send* _bounding_box_origin __bounding_box_origin)
     (progn
       (if __bounding_box_origin (setq _bounding_box_origin (car __bounding_box_origin)))
       _bounding_box_origin)))
  (:bounding_box_lengths
   (&rest __bounding_box_lengths)
   (if (keywordp (car __bounding_box_lengths))
       (send* _bounding_box_lengths __bounding_box_lengths)
     (progn
       (if __bounding_box_lengths (setq _bounding_box_lengths (car __bounding_box_lengths)))
       _bounding_box_lengths)))
  (:leaf_size
   (&optional __leaf_size)
   (if __leaf_size (setq _leaf_size __leaf_size)) _leaf_size)
  (:publish_octomap
   (&optional __publish_octomap)
   (if __publish_octomap (setq _publish_octomap __publish_octomap)) _publish_octomap)
  (:filename
   (&optional __filename)
   (if __filename (setq _filename __filename)) _filename)
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Point _bounding_box_origin
    (send _bounding_box_origin :serialization-length)
    ;; geometry_msgs/Point _bounding_box_lengths
    (send _bounding_box_lengths :serialization-length)
    ;; float64 _leaf_size
    8
    ;; bool _publish_octomap
    1
    ;; string _filename
    4 (length _filename)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Point _bounding_box_origin
       (send _bounding_box_origin :serialize s)
     ;; geometry_msgs/Point _bounding_box_lengths
       (send _bounding_box_lengths :serialize s)
     ;; float64 _leaf_size
       (sys::poke _leaf_size (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; bool _publish_octomap
       (if _publish_octomap (write-byte -1 s) (write-byte 0 s))
     ;; string _filename
       (write-long (length _filename) s) (princ _filename s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Point _bounding_box_origin
     (send _bounding_box_origin :deserialize buf ptr-) (incf ptr- (send _bounding_box_origin :serialization-length))
   ;; geometry_msgs/Point _bounding_box_lengths
     (send _bounding_box_lengths :deserialize buf ptr-) (incf ptr- (send _bounding_box_lengths :serialization-length))
   ;; float64 _leaf_size
     (setq _leaf_size (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; bool _publish_octomap
     (setq _publish_octomap (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _filename
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _filename (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass rotors_comm::OctomapResponse
  :super ros::object
  :slots (_map ))

(defmethod rotors_comm::OctomapResponse
  (:init
   (&key
    ((:map __map) (instance octomap_msgs::Octomap :init))
    )
   (send-super :init)
   (setq _map __map)
   self)
  (:map
   (&rest __map)
   (if (keywordp (car __map))
       (send* _map __map)
     (progn
       (if __map (setq _map (car __map)))
       _map)))
  (:serialization-length
   ()
   (+
    ;; octomap_msgs/Octomap _map
    (send _map :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; octomap_msgs/Octomap _map
       (send _map :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; octomap_msgs/Octomap _map
     (send _map :deserialize buf ptr-) (incf ptr- (send _map :serialization-length))
   ;;
   self)
  )

(defclass rotors_comm::Octomap
  :super ros::object
  :slots ())

(setf (get rotors_comm::Octomap :md5sum-) "7b66ab5bfca356ccfebdd11157126a9f")
(setf (get rotors_comm::Octomap :datatype-) "rotors_comm/Octomap")
(setf (get rotors_comm::Octomap :request) rotors_comm::OctomapRequest)
(setf (get rotors_comm::Octomap :response) rotors_comm::OctomapResponse)

(defmethod rotors_comm::OctomapRequest
  (:response () (instance rotors_comm::OctomapResponse :init)))

(setf (get rotors_comm::OctomapRequest :md5sum-) "7b66ab5bfca356ccfebdd11157126a9f")
(setf (get rotors_comm::OctomapRequest :datatype-) "rotors_comm/OctomapRequest")
(setf (get rotors_comm::OctomapRequest :definition-)
      "
geometry_msgs/Point bounding_box_origin

geometry_msgs/Point bounding_box_lengths

float64 leaf_size

bool publish_octomap

string filename

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
---
octomap_msgs/Octomap map


================================================================================
MSG: octomap_msgs/Octomap
# A 3D map in binary format, as Octree
Header header

# Flag to denote a binary (only free/occupied) or full occupancy octree (.bt/.ot file)
bool binary

# Class id of the contained octree 
string id

# Resolution (in m) of the smallest octree nodes
float64 resolution

# binary serialization of octree, use conversions.h to read and write octrees
int8[] data

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
# 0: no frame
# 1: global frame
string frame_id
")

(setf (get rotors_comm::OctomapResponse :md5sum-) "7b66ab5bfca356ccfebdd11157126a9f")
(setf (get rotors_comm::OctomapResponse :datatype-) "rotors_comm/OctomapResponse")
(setf (get rotors_comm::OctomapResponse :definition-)
      "
geometry_msgs/Point bounding_box_origin

geometry_msgs/Point bounding_box_lengths

float64 leaf_size

bool publish_octomap

string filename

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
---
octomap_msgs/Octomap map


================================================================================
MSG: octomap_msgs/Octomap
# A 3D map in binary format, as Octree
Header header

# Flag to denote a binary (only free/occupied) or full occupancy octree (.bt/.ot file)
bool binary

# Class id of the contained octree 
string id

# Resolution (in m) of the smallest octree nodes
float64 resolution

# binary serialization of octree, use conversions.h to read and write octrees
int8[] data

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
# 0: no frame
# 1: global frame
string frame_id
")



(provide :rotors_comm/Octomap "7b66ab5bfca356ccfebdd11157126a9f")


