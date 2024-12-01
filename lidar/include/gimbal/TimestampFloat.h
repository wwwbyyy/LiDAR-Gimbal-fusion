// Generated by gencpp from file gimbal/TimestampFloat.msg
// DO NOT EDIT!


#ifndef GIMBAL_MESSAGE_TIMESTAMPFLOAT_H
#define GIMBAL_MESSAGE_TIMESTAMPFLOAT_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace gimbal
{
template <class ContainerAllocator>
struct TimestampFloat_
{
  typedef TimestampFloat_<ContainerAllocator> Type;

  TimestampFloat_()
    : header()
    , value(0.0)  {
    }
  TimestampFloat_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , value(0.0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef float _value_type;
  _value_type value;





  typedef boost::shared_ptr< ::gimbal::TimestampFloat_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::gimbal::TimestampFloat_<ContainerAllocator> const> ConstPtr;

}; // struct TimestampFloat_

typedef ::gimbal::TimestampFloat_<std::allocator<void> > TimestampFloat;

typedef boost::shared_ptr< ::gimbal::TimestampFloat > TimestampFloatPtr;
typedef boost::shared_ptr< ::gimbal::TimestampFloat const> TimestampFloatConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::gimbal::TimestampFloat_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::gimbal::TimestampFloat_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::gimbal::TimestampFloat_<ContainerAllocator1> & lhs, const ::gimbal::TimestampFloat_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.value == rhs.value;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::gimbal::TimestampFloat_<ContainerAllocator1> & lhs, const ::gimbal::TimestampFloat_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace gimbal

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::gimbal::TimestampFloat_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::gimbal::TimestampFloat_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::gimbal::TimestampFloat_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::gimbal::TimestampFloat_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::gimbal::TimestampFloat_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::gimbal::TimestampFloat_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::gimbal::TimestampFloat_<ContainerAllocator> >
{
  static const char* value()
  {
    return "4bea522f9243fd34ea7bc74ce85697a8";
  }

  static const char* value(const ::gimbal::TimestampFloat_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x4bea522f9243fd34ULL;
  static const uint64_t static_value2 = 0xea7bc74ce85697a8ULL;
};

template<class ContainerAllocator>
struct DataType< ::gimbal::TimestampFloat_<ContainerAllocator> >
{
  static const char* value()
  {
    return "gimbal/TimestampFloat";
  }

  static const char* value(const ::gimbal::TimestampFloat_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::gimbal::TimestampFloat_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/Header header\n"
"float32 value\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
;
  }

  static const char* value(const ::gimbal::TimestampFloat_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::gimbal::TimestampFloat_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.value);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct TimestampFloat_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::gimbal::TimestampFloat_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::gimbal::TimestampFloat_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "value: ";
    Printer<float>::stream(s, indent + "  ", v.value);
  }
};

} // namespace message_operations
} // namespace ros

#endif // GIMBAL_MESSAGE_TIMESTAMPFLOAT_H
