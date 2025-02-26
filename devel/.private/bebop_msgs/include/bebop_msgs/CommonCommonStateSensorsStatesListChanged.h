// Generated by gencpp from file bebop_msgs/CommonCommonStateSensorsStatesListChanged.msg
// DO NOT EDIT!


#ifndef BEBOP_MSGS_MESSAGE_COMMONCOMMONSTATESENSORSSTATESLISTCHANGED_H
#define BEBOP_MSGS_MESSAGE_COMMONCOMMONSTATESENSORSSTATESLISTCHANGED_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace bebop_msgs
{
template <class ContainerAllocator>
struct CommonCommonStateSensorsStatesListChanged_
{
  typedef CommonCommonStateSensorsStatesListChanged_<ContainerAllocator> Type;

  CommonCommonStateSensorsStatesListChanged_()
    : header()
    , sensorName(0)
    , sensorState(0)  {
    }
  CommonCommonStateSensorsStatesListChanged_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , sensorName(0)
    , sensorState(0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef uint8_t _sensorName_type;
  _sensorName_type sensorName;

   typedef uint8_t _sensorState_type;
  _sensorState_type sensorState;



  enum {
    sensorName_IMU = 0u,
    sensorName_barometer = 1u,
    sensorName_ultrasound = 2u,
    sensorName_GPS = 3u,
    sensorName_magnetometer = 4u,
    sensorName_vertical_camera = 5u,
  };


  typedef boost::shared_ptr< ::bebop_msgs::CommonCommonStateSensorsStatesListChanged_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::bebop_msgs::CommonCommonStateSensorsStatesListChanged_<ContainerAllocator> const> ConstPtr;

}; // struct CommonCommonStateSensorsStatesListChanged_

typedef ::bebop_msgs::CommonCommonStateSensorsStatesListChanged_<std::allocator<void> > CommonCommonStateSensorsStatesListChanged;

typedef boost::shared_ptr< ::bebop_msgs::CommonCommonStateSensorsStatesListChanged > CommonCommonStateSensorsStatesListChangedPtr;
typedef boost::shared_ptr< ::bebop_msgs::CommonCommonStateSensorsStatesListChanged const> CommonCommonStateSensorsStatesListChangedConstPtr;

// constants requiring out of line definition

   

   

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::bebop_msgs::CommonCommonStateSensorsStatesListChanged_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::bebop_msgs::CommonCommonStateSensorsStatesListChanged_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace bebop_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'bebop_msgs': ['/home/emanuele/ethz_ws/src/bebop_autonomy/bebop_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::bebop_msgs::CommonCommonStateSensorsStatesListChanged_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::bebop_msgs::CommonCommonStateSensorsStatesListChanged_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::bebop_msgs::CommonCommonStateSensorsStatesListChanged_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::bebop_msgs::CommonCommonStateSensorsStatesListChanged_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::bebop_msgs::CommonCommonStateSensorsStatesListChanged_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::bebop_msgs::CommonCommonStateSensorsStatesListChanged_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::bebop_msgs::CommonCommonStateSensorsStatesListChanged_<ContainerAllocator> >
{
  static const char* value()
  {
    return "21324261f7a8c1805999cb90c3c5949b";
  }

  static const char* value(const ::bebop_msgs::CommonCommonStateSensorsStatesListChanged_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x21324261f7a8c180ULL;
  static const uint64_t static_value2 = 0x5999cb90c3c5949bULL;
};

template<class ContainerAllocator>
struct DataType< ::bebop_msgs::CommonCommonStateSensorsStatesListChanged_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bebop_msgs/CommonCommonStateSensorsStatesListChanged";
  }

  static const char* value(const ::bebop_msgs::CommonCommonStateSensorsStatesListChanged_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::bebop_msgs::CommonCommonStateSensorsStatesListChanged_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# CommonCommonStateSensorsStatesListChanged\n\
# auto-generated from up stream XML files at\n\
#   github.com/Parrot-Developers/libARCommands/tree/master/Xml\n\
# To check upstream commit hash, refer to last_build_info file\n\
# Do not modify this file by hand. Check scripts/meta folder for generator files.\n\
#\n\
# SDK Comment: Sensors state list.\n\
\n\
Header header\n\
\n\
# Sensor name\n\
uint8 sensorName_IMU=0  # Inertial Measurement Unit sensor\n\
uint8 sensorName_barometer=1  # Barometer sensor\n\
uint8 sensorName_ultrasound=2  # Ultrasonic sensor\n\
uint8 sensorName_GPS=3  # GPS sensor\n\
uint8 sensorName_magnetometer=4  # Magnetometer sensor\n\
uint8 sensorName_vertical_camera=5  # Vertical Camera sensor\n\
uint8 sensorName\n\
# Sensor state (1 if the sensor is OK, 0 if the sensor is NOT OK)\n\
uint8 sensorState\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
";
  }

  static const char* value(const ::bebop_msgs::CommonCommonStateSensorsStatesListChanged_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::bebop_msgs::CommonCommonStateSensorsStatesListChanged_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.sensorName);
      stream.next(m.sensorState);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct CommonCommonStateSensorsStatesListChanged_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::bebop_msgs::CommonCommonStateSensorsStatesListChanged_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::bebop_msgs::CommonCommonStateSensorsStatesListChanged_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "sensorName: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.sensorName);
    s << indent << "sensorState: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.sensorState);
  }
};

} // namespace message_operations
} // namespace ros

#endif // BEBOP_MSGS_MESSAGE_COMMONCOMMONSTATESENSORSSTATESLISTCHANGED_H
