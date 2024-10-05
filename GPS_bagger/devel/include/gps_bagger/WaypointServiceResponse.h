// Generated by gencpp from file gps_bagger/WaypointServiceResponse.msg
// DO NOT EDIT!


#ifndef GPS_BAGGER_MESSAGE_WAYPOINTSERVICERESPONSE_H
#define GPS_BAGGER_MESSAGE_WAYPOINTSERVICERESPONSE_H


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
struct WaypointServiceResponse_
{
  typedef WaypointServiceResponse_<ContainerAllocator> Type;

  WaypointServiceResponse_()
    : success(false)  {
    }
  WaypointServiceResponse_(const ContainerAllocator& _alloc)
    : success(false)  {
  (void)_alloc;
    }



   typedef uint8_t _success_type;
  _success_type success;





  typedef boost::shared_ptr< ::gps_bagger::WaypointServiceResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::gps_bagger::WaypointServiceResponse_<ContainerAllocator> const> ConstPtr;

}; // struct WaypointServiceResponse_

typedef ::gps_bagger::WaypointServiceResponse_<std::allocator<void> > WaypointServiceResponse;

typedef boost::shared_ptr< ::gps_bagger::WaypointServiceResponse > WaypointServiceResponsePtr;
typedef boost::shared_ptr< ::gps_bagger::WaypointServiceResponse const> WaypointServiceResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::gps_bagger::WaypointServiceResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::gps_bagger::WaypointServiceResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::gps_bagger::WaypointServiceResponse_<ContainerAllocator1> & lhs, const ::gps_bagger::WaypointServiceResponse_<ContainerAllocator2> & rhs)
{
  return lhs.success == rhs.success;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::gps_bagger::WaypointServiceResponse_<ContainerAllocator1> & lhs, const ::gps_bagger::WaypointServiceResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace gps_bagger

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::gps_bagger::WaypointServiceResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::gps_bagger::WaypointServiceResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::gps_bagger::WaypointServiceResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::gps_bagger::WaypointServiceResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::gps_bagger::WaypointServiceResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::gps_bagger::WaypointServiceResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::gps_bagger::WaypointServiceResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "358e233cde0c8a8bcfea4ce193f8fc15";
  }

  static const char* value(const ::gps_bagger::WaypointServiceResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x358e233cde0c8a8bULL;
  static const uint64_t static_value2 = 0xcfea4ce193f8fc15ULL;
};

template<class ContainerAllocator>
struct DataType< ::gps_bagger::WaypointServiceResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "gps_bagger/WaypointServiceResponse";
  }

  static const char* value(const ::gps_bagger::WaypointServiceResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::gps_bagger::WaypointServiceResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool success\n"
"\n"
;
  }

  static const char* value(const ::gps_bagger::WaypointServiceResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::gps_bagger::WaypointServiceResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.success);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct WaypointServiceResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::gps_bagger::WaypointServiceResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::gps_bagger::WaypointServiceResponse_<ContainerAllocator>& v)
  {
    s << indent << "success: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.success);
  }
};

} // namespace message_operations
} // namespace ros

#endif // GPS_BAGGER_MESSAGE_WAYPOINTSERVICERESPONSE_H
