// Generated by gencpp from file gps_bagger/WaypointService.msg
// DO NOT EDIT!


#ifndef GPS_BAGGER_MESSAGE_WAYPOINTSERVICE_H
#define GPS_BAGGER_MESSAGE_WAYPOINTSERVICE_H

#include <ros/service_traits.h>


#include <gps_bagger/WaypointServiceRequest.h>
#include <gps_bagger/WaypointServiceResponse.h>


namespace gps_bagger
{

struct WaypointService
{

typedef WaypointServiceRequest Request;
typedef WaypointServiceResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct WaypointService
} // namespace gps_bagger


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::gps_bagger::WaypointService > {
  static const char* value()
  {
    return "019088792a6bd1eace996d8d0485951f";
  }

  static const char* value(const ::gps_bagger::WaypointService&) { return value(); }
};

template<>
struct DataType< ::gps_bagger::WaypointService > {
  static const char* value()
  {
    return "gps_bagger/WaypointService";
  }

  static const char* value(const ::gps_bagger::WaypointService&) { return value(); }
};


// service_traits::MD5Sum< ::gps_bagger::WaypointServiceRequest> should match
// service_traits::MD5Sum< ::gps_bagger::WaypointService >
template<>
struct MD5Sum< ::gps_bagger::WaypointServiceRequest>
{
  static const char* value()
  {
    return MD5Sum< ::gps_bagger::WaypointService >::value();
  }
  static const char* value(const ::gps_bagger::WaypointServiceRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::gps_bagger::WaypointServiceRequest> should match
// service_traits::DataType< ::gps_bagger::WaypointService >
template<>
struct DataType< ::gps_bagger::WaypointServiceRequest>
{
  static const char* value()
  {
    return DataType< ::gps_bagger::WaypointService >::value();
  }
  static const char* value(const ::gps_bagger::WaypointServiceRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::gps_bagger::WaypointServiceResponse> should match
// service_traits::MD5Sum< ::gps_bagger::WaypointService >
template<>
struct MD5Sum< ::gps_bagger::WaypointServiceResponse>
{
  static const char* value()
  {
    return MD5Sum< ::gps_bagger::WaypointService >::value();
  }
  static const char* value(const ::gps_bagger::WaypointServiceResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::gps_bagger::WaypointServiceResponse> should match
// service_traits::DataType< ::gps_bagger::WaypointService >
template<>
struct DataType< ::gps_bagger::WaypointServiceResponse>
{
  static const char* value()
  {
    return DataType< ::gps_bagger::WaypointService >::value();
  }
  static const char* value(const ::gps_bagger::WaypointServiceResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // GPS_BAGGER_MESSAGE_WAYPOINTSERVICE_H