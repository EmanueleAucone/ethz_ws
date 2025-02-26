// Generated by gencpp from file bebop_msgs/Ardrone3PilotingStatemoveToChanged.msg
// DO NOT EDIT!


#ifndef BEBOP_MSGS_MESSAGE_ARDRONE3PILOTINGSTATEMOVETOCHANGED_H
#define BEBOP_MSGS_MESSAGE_ARDRONE3PILOTINGSTATEMOVETOCHANGED_H


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
struct Ardrone3PilotingStatemoveToChanged_
{
  typedef Ardrone3PilotingStatemoveToChanged_<ContainerAllocator> Type;

  Ardrone3PilotingStatemoveToChanged_()
    : header()
    , latitude(0.0)
    , longitude(0.0)
    , altitude(0.0)
    , orientation_mode(0)
    , heading(0.0)
    , status(0)  {
    }
  Ardrone3PilotingStatemoveToChanged_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , latitude(0.0)
    , longitude(0.0)
    , altitude(0.0)
    , orientation_mode(0)
    , heading(0.0)
    , status(0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef double _latitude_type;
  _latitude_type latitude;

   typedef double _longitude_type;
  _longitude_type longitude;

   typedef double _altitude_type;
  _altitude_type altitude;

   typedef uint8_t _orientation_mode_type;
  _orientation_mode_type orientation_mode;

   typedef float _heading_type;
  _heading_type heading;

   typedef uint8_t _status_type;
  _status_type status;



  enum {
    orientation_mode_NONE = 0u,
    orientation_mode_TO_TARGET = 1u,
    orientation_mode_HEADING_START = 2u,
    orientation_mode_HEADING_DURING = 3u,
    status_RUNNING = 0u,
    status_DONE = 1u,
    status_CANCELED = 2u,
    status_ERROR = 3u,
  };


  typedef boost::shared_ptr< ::bebop_msgs::Ardrone3PilotingStatemoveToChanged_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::bebop_msgs::Ardrone3PilotingStatemoveToChanged_<ContainerAllocator> const> ConstPtr;

}; // struct Ardrone3PilotingStatemoveToChanged_

typedef ::bebop_msgs::Ardrone3PilotingStatemoveToChanged_<std::allocator<void> > Ardrone3PilotingStatemoveToChanged;

typedef boost::shared_ptr< ::bebop_msgs::Ardrone3PilotingStatemoveToChanged > Ardrone3PilotingStatemoveToChangedPtr;
typedef boost::shared_ptr< ::bebop_msgs::Ardrone3PilotingStatemoveToChanged const> Ardrone3PilotingStatemoveToChangedConstPtr;

// constants requiring out of line definition

   

   

   

   

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::bebop_msgs::Ardrone3PilotingStatemoveToChanged_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::bebop_msgs::Ardrone3PilotingStatemoveToChanged_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::bebop_msgs::Ardrone3PilotingStatemoveToChanged_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::bebop_msgs::Ardrone3PilotingStatemoveToChanged_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::bebop_msgs::Ardrone3PilotingStatemoveToChanged_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::bebop_msgs::Ardrone3PilotingStatemoveToChanged_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::bebop_msgs::Ardrone3PilotingStatemoveToChanged_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::bebop_msgs::Ardrone3PilotingStatemoveToChanged_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::bebop_msgs::Ardrone3PilotingStatemoveToChanged_<ContainerAllocator> >
{
  static const char* value()
  {
    return "f356580a128658358cde541e12e3218b";
  }

  static const char* value(const ::bebop_msgs::Ardrone3PilotingStatemoveToChanged_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xf356580a12865835ULL;
  static const uint64_t static_value2 = 0x8cde541e12e3218bULL;
};

template<class ContainerAllocator>
struct DataType< ::bebop_msgs::Ardrone3PilotingStatemoveToChanged_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bebop_msgs/Ardrone3PilotingStatemoveToChanged";
  }

  static const char* value(const ::bebop_msgs::Ardrone3PilotingStatemoveToChanged_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::bebop_msgs::Ardrone3PilotingStatemoveToChanged_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Ardrone3PilotingStatemoveToChanged\n\
# auto-generated from up stream XML files at\n\
#   github.com/Parrot-Developers/libARCommands/tree/master/Xml\n\
# To check upstream commit hash, refer to last_build_info file\n\
# Do not modify this file by hand. Check scripts/meta folder for generator files.\n\
#\n\
# SDK Comment: The drone moves or moved to a given location.\n\
\n\
Header header\n\
\n\
# Latitude of the location (in degrees) to reach\n\
float64 latitude\n\
# Longitude of the location (in degrees) to reach\n\
float64 longitude\n\
# Altitude above sea level (in m) to reach\n\
float64 altitude\n\
# Orientation mode of the move to\n\
uint8 orientation_mode_NONE=0  # The drone wont change its orientation\n\
uint8 orientation_mode_TO_TARGET=1  # The drone will make a rotation to look in direction of the given location\n\
uint8 orientation_mode_HEADING_START=2  # The drone will orientate itself to the given heading before moving to the location\n\
uint8 orientation_mode_HEADING_DURING=3  # The drone will orientate itself to the given heading while moving to the location\n\
uint8 orientation_mode\n\
# Heading (relative to the North in degrees). This value is only used if the orientation mode is HEADING_START or HEADING_DURING\n\
float32 heading\n\
# Status of the move to\n\
uint8 status_RUNNING=0  # The drone is actually flying to the given position\n\
uint8 status_DONE=1  # The drone has reached the target\n\
uint8 status_CANCELED=2  # The move to has been canceled, either by a new moveTo command or by a CancelMoveTo command.\n\
uint8 status_ERROR=3  # The move to has not been finished or started because of an error.\n\
uint8 status\n\
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

  static const char* value(const ::bebop_msgs::Ardrone3PilotingStatemoveToChanged_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::bebop_msgs::Ardrone3PilotingStatemoveToChanged_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.latitude);
      stream.next(m.longitude);
      stream.next(m.altitude);
      stream.next(m.orientation_mode);
      stream.next(m.heading);
      stream.next(m.status);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Ardrone3PilotingStatemoveToChanged_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::bebop_msgs::Ardrone3PilotingStatemoveToChanged_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::bebop_msgs::Ardrone3PilotingStatemoveToChanged_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "latitude: ";
    Printer<double>::stream(s, indent + "  ", v.latitude);
    s << indent << "longitude: ";
    Printer<double>::stream(s, indent + "  ", v.longitude);
    s << indent << "altitude: ";
    Printer<double>::stream(s, indent + "  ", v.altitude);
    s << indent << "orientation_mode: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.orientation_mode);
    s << indent << "heading: ";
    Printer<float>::stream(s, indent + "  ", v.heading);
    s << indent << "status: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.status);
  }
};

} // namespace message_operations
} // namespace ros

#endif // BEBOP_MSGS_MESSAGE_ARDRONE3PILOTINGSTATEMOVETOCHANGED_H
