// Generated by gencpp from file sensor_fusion_comm/InitScaleRequest.msg
// DO NOT EDIT!


#ifndef SENSOR_FUSION_COMM_MESSAGE_INITSCALEREQUEST_H
#define SENSOR_FUSION_COMM_MESSAGE_INITSCALEREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace sensor_fusion_comm
{
template <class ContainerAllocator>
struct InitScaleRequest_
{
  typedef InitScaleRequest_<ContainerAllocator> Type;

  InitScaleRequest_()
    : scale(0.0)  {
    }
  InitScaleRequest_(const ContainerAllocator& _alloc)
    : scale(0.0)  {
  (void)_alloc;
    }



   typedef float _scale_type;
  _scale_type scale;





  typedef boost::shared_ptr< ::sensor_fusion_comm::InitScaleRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::sensor_fusion_comm::InitScaleRequest_<ContainerAllocator> const> ConstPtr;

}; // struct InitScaleRequest_

typedef ::sensor_fusion_comm::InitScaleRequest_<std::allocator<void> > InitScaleRequest;

typedef boost::shared_ptr< ::sensor_fusion_comm::InitScaleRequest > InitScaleRequestPtr;
typedef boost::shared_ptr< ::sensor_fusion_comm::InitScaleRequest const> InitScaleRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::sensor_fusion_comm::InitScaleRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::sensor_fusion_comm::InitScaleRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace sensor_fusion_comm

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'sensor_fusion_comm': ['/home/emanuele/ethz_ws/src/ethzasl_msf/sensor_fusion_comm/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::sensor_fusion_comm::InitScaleRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::sensor_fusion_comm::InitScaleRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::sensor_fusion_comm::InitScaleRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::sensor_fusion_comm::InitScaleRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::sensor_fusion_comm::InitScaleRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::sensor_fusion_comm::InitScaleRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::sensor_fusion_comm::InitScaleRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "6de42640623c526fd99227d7dc6323fb";
  }

  static const char* value(const ::sensor_fusion_comm::InitScaleRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x6de42640623c526fULL;
  static const uint64_t static_value2 = 0xd99227d7dc6323fbULL;
};

template<class ContainerAllocator>
struct DataType< ::sensor_fusion_comm::InitScaleRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "sensor_fusion_comm/InitScaleRequest";
  }

  static const char* value(const ::sensor_fusion_comm::InitScaleRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::sensor_fusion_comm::InitScaleRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32 scale\n\
";
  }

  static const char* value(const ::sensor_fusion_comm::InitScaleRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::sensor_fusion_comm::InitScaleRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.scale);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct InitScaleRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::sensor_fusion_comm::InitScaleRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::sensor_fusion_comm::InitScaleRequest_<ContainerAllocator>& v)
  {
    s << indent << "scale: ";
    Printer<float>::stream(s, indent + "  ", v.scale);
  }
};

} // namespace message_operations
} // namespace ros

#endif // SENSOR_FUSION_COMM_MESSAGE_INITSCALEREQUEST_H
