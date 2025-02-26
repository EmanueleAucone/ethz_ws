// Generated by gencpp from file sensor_fusion_comm/PointWithCovarianceStamped.msg
// DO NOT EDIT!


#ifndef SENSOR_FUSION_COMM_MESSAGE_POINTWITHCOVARIANCESTAMPED_H
#define SENSOR_FUSION_COMM_MESSAGE_POINTWITHCOVARIANCESTAMPED_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <geometry_msgs/Point.h>

namespace sensor_fusion_comm
{
template <class ContainerAllocator>
struct PointWithCovarianceStamped_
{
  typedef PointWithCovarianceStamped_<ContainerAllocator> Type;

  PointWithCovarianceStamped_()
    : header()
    , point()
    , covariance()  {
      covariance.assign(0.0);
  }
  PointWithCovarianceStamped_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , point(_alloc)
    , covariance()  {
  (void)_alloc;
      covariance.assign(0.0);
  }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _point_type;
  _point_type point;

   typedef boost::array<double, 9>  _covariance_type;
  _covariance_type covariance;





  typedef boost::shared_ptr< ::sensor_fusion_comm::PointWithCovarianceStamped_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::sensor_fusion_comm::PointWithCovarianceStamped_<ContainerAllocator> const> ConstPtr;

}; // struct PointWithCovarianceStamped_

typedef ::sensor_fusion_comm::PointWithCovarianceStamped_<std::allocator<void> > PointWithCovarianceStamped;

typedef boost::shared_ptr< ::sensor_fusion_comm::PointWithCovarianceStamped > PointWithCovarianceStampedPtr;
typedef boost::shared_ptr< ::sensor_fusion_comm::PointWithCovarianceStamped const> PointWithCovarianceStampedConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::sensor_fusion_comm::PointWithCovarianceStamped_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::sensor_fusion_comm::PointWithCovarianceStamped_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace sensor_fusion_comm

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'sensor_fusion_comm': ['/home/emanuele/ethz_ws/src/ethzasl_msf/sensor_fusion_comm/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::sensor_fusion_comm::PointWithCovarianceStamped_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::sensor_fusion_comm::PointWithCovarianceStamped_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::sensor_fusion_comm::PointWithCovarianceStamped_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::sensor_fusion_comm::PointWithCovarianceStamped_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::sensor_fusion_comm::PointWithCovarianceStamped_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::sensor_fusion_comm::PointWithCovarianceStamped_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::sensor_fusion_comm::PointWithCovarianceStamped_<ContainerAllocator> >
{
  static const char* value()
  {
    return "0702610ecb647cea3bb121e1743a30e7";
  }

  static const char* value(const ::sensor_fusion_comm::PointWithCovarianceStamped_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x0702610ecb647ceaULL;
  static const uint64_t static_value2 = 0x3bb121e1743a30e7ULL;
};

template<class ContainerAllocator>
struct DataType< ::sensor_fusion_comm::PointWithCovarianceStamped_<ContainerAllocator> >
{
  static const char* value()
  {
    return "sensor_fusion_comm/PointWithCovarianceStamped";
  }

  static const char* value(const ::sensor_fusion_comm::PointWithCovarianceStamped_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::sensor_fusion_comm::PointWithCovarianceStamped_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n\
geometry_msgs/Point point\n\
float64[9] covariance\n\
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
\n\
================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
";
  }

  static const char* value(const ::sensor_fusion_comm::PointWithCovarianceStamped_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::sensor_fusion_comm::PointWithCovarianceStamped_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.point);
      stream.next(m.covariance);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct PointWithCovarianceStamped_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::sensor_fusion_comm::PointWithCovarianceStamped_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::sensor_fusion_comm::PointWithCovarianceStamped_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "point: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.point);
    s << indent << "covariance[]" << std::endl;
    for (size_t i = 0; i < v.covariance.size(); ++i)
    {
      s << indent << "  covariance[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.covariance[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // SENSOR_FUSION_COMM_MESSAGE_POINTWITHCOVARIANCESTAMPED_H
