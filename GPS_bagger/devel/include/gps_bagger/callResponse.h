// Generated by gencpp from file gps_bagger/callResponse.msg
// DO NOT EDIT!


#ifndef GPS_BAGGER_MESSAGE_CALLRESPONSE_H
#define GPS_BAGGER_MESSAGE_CALLRESPONSE_H

#include <ros/service_traits.h>


#include <gps_bagger/callResponseRequest.h>
#include <gps_bagger/callResponseResponse.h>


namespace gps_bagger
{

struct callResponse
{

typedef callResponseRequest Request;
typedef callResponseResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct callResponse
} // namespace gps_bagger


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::gps_bagger::callResponse > {
  static const char* value()
  {
    return "22c7c465d64c7e74c6ae22029c7ca150";
  }

  static const char* value(const ::gps_bagger::callResponse&) { return value(); }
};

template<>
struct DataType< ::gps_bagger::callResponse > {
  static const char* value()
  {
    return "gps_bagger/callResponse";
  }

  static const char* value(const ::gps_bagger::callResponse&) { return value(); }
};


// service_traits::MD5Sum< ::gps_bagger::callResponseRequest> should match
// service_traits::MD5Sum< ::gps_bagger::callResponse >
template<>
struct MD5Sum< ::gps_bagger::callResponseRequest>
{
  static const char* value()
  {
    return MD5Sum< ::gps_bagger::callResponse >::value();
  }
  static const char* value(const ::gps_bagger::callResponseRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::gps_bagger::callResponseRequest> should match
// service_traits::DataType< ::gps_bagger::callResponse >
template<>
struct DataType< ::gps_bagger::callResponseRequest>
{
  static const char* value()
  {
    return DataType< ::gps_bagger::callResponse >::value();
  }
  static const char* value(const ::gps_bagger::callResponseRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::gps_bagger::callResponseResponse> should match
// service_traits::MD5Sum< ::gps_bagger::callResponse >
template<>
struct MD5Sum< ::gps_bagger::callResponseResponse>
{
  static const char* value()
  {
    return MD5Sum< ::gps_bagger::callResponse >::value();
  }
  static const char* value(const ::gps_bagger::callResponseResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::gps_bagger::callResponseResponse> should match
// service_traits::DataType< ::gps_bagger::callResponse >
template<>
struct DataType< ::gps_bagger::callResponseResponse>
{
  static const char* value()
  {
    return DataType< ::gps_bagger::callResponse >::value();
  }
  static const char* value(const ::gps_bagger::callResponseResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // GPS_BAGGER_MESSAGE_CALLRESPONSE_H