// Generated by gencpp from file gps_bagger/callResponseRequest.msg
// DO NOT EDIT!


#ifndef GPS_BAGGER_MESSAGE_CALLRESPONSEREQUEST_H
#define GPS_BAGGER_MESSAGE_CALLRESPONSEREQUEST_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace gps_bagger
{
template <class ContainerAllocator>
struct callResponseRequest_
{
  typedef callResponseRequest_<ContainerAllocator> Type;

  callResponseRequest_()
    : command()  {
    }
  callResponseRequest_(const ContainerAllocator& _alloc)
    : command(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _command_type;
  _command_type command;





  typedef boost::shared_ptr< ::gps_bagger::callResponseRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::gps_bagger::callResponseRequest_<ContainerAllocator> const> ConstPtr;

}; // struct callResponseRequest_

typedef ::gps_bagger::callResponseRequest_<std::allocator<void> > callResponseRequest;

typedef boost::shared_ptr< ::gps_bagger::callResponseRequest > callResponseRequestPtr;
typedef boost::shared_ptr< ::gps_bagger::callResponseRequest const> callResponseRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::gps_bagger::callResponseRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::gps_bagger::callResponseRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::gps_bagger::callResponseRequest_<ContainerAllocator1> & lhs, const ::gps_bagger::callResponseRequest_<ContainerAllocator2> & rhs)
{
  return lhs.command == rhs.command;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::gps_bagger::callResponseRequest_<ContainerAllocator1> & lhs, const ::gps_bagger::callResponseRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace gps_bagger

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::gps_bagger::callResponseRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::gps_bagger::callResponseRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::gps_bagger::callResponseRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::gps_bagger::callResponseRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::gps_bagger::callResponseRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::gps_bagger::callResponseRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::gps_bagger::callResponseRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cba5e21e920a3a2b7b375cb65b64cdea";
  }

  static const char* value(const ::gps_bagger::callResponseRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xcba5e21e920a3a2bULL;
  static const uint64_t static_value2 = 0x7b375cb65b64cdeaULL;
};

template<class ContainerAllocator>
struct DataType< ::gps_bagger::callResponseRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "gps_bagger/callResponseRequest";
  }

  static const char* value(const ::gps_bagger::callResponseRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::gps_bagger::callResponseRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Request\n"
"string command\n"
;
  }

  static const char* value(const ::gps_bagger::callResponseRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::gps_bagger::callResponseRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.command);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct callResponseRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::gps_bagger::callResponseRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::gps_bagger::callResponseRequest_<ContainerAllocator>& v)
  {
    s << indent << "command: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.command);
  }
};

} // namespace message_operations
} // namespace ros

#endif // GPS_BAGGER_MESSAGE_CALLRESPONSEREQUEST_H