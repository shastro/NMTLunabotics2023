// Generated by gencpp from file se2_grid_map_generator_msgs/ResetMap.msg
// DO NOT EDIT!


#ifndef SE2_GRID_MAP_GENERATOR_MSGS_MESSAGE_RESETMAP_H
#define SE2_GRID_MAP_GENERATOR_MSGS_MESSAGE_RESETMAP_H

#include <ros/service_traits.h>


#include <se2_grid_map_generator_msgs/ResetMapRequest.h>
#include <se2_grid_map_generator_msgs/ResetMapResponse.h>


namespace se2_grid_map_generator_msgs
{

struct ResetMap
{

typedef ResetMapRequest Request;
typedef ResetMapResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct ResetMap
} // namespace se2_grid_map_generator_msgs


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::se2_grid_map_generator_msgs::ResetMap > {
  static const char* value()
  {
    return "358e233cde0c8a8bcfea4ce193f8fc15";
  }

  static const char* value(const ::se2_grid_map_generator_msgs::ResetMap&) { return value(); }
};

template<>
struct DataType< ::se2_grid_map_generator_msgs::ResetMap > {
  static const char* value()
  {
    return "se2_grid_map_generator_msgs/ResetMap";
  }

  static const char* value(const ::se2_grid_map_generator_msgs::ResetMap&) { return value(); }
};


// service_traits::MD5Sum< ::se2_grid_map_generator_msgs::ResetMapRequest> should match
// service_traits::MD5Sum< ::se2_grid_map_generator_msgs::ResetMap >
template<>
struct MD5Sum< ::se2_grid_map_generator_msgs::ResetMapRequest>
{
  static const char* value()
  {
    return MD5Sum< ::se2_grid_map_generator_msgs::ResetMap >::value();
  }
  static const char* value(const ::se2_grid_map_generator_msgs::ResetMapRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::se2_grid_map_generator_msgs::ResetMapRequest> should match
// service_traits::DataType< ::se2_grid_map_generator_msgs::ResetMap >
template<>
struct DataType< ::se2_grid_map_generator_msgs::ResetMapRequest>
{
  static const char* value()
  {
    return DataType< ::se2_grid_map_generator_msgs::ResetMap >::value();
  }
  static const char* value(const ::se2_grid_map_generator_msgs::ResetMapRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::se2_grid_map_generator_msgs::ResetMapResponse> should match
// service_traits::MD5Sum< ::se2_grid_map_generator_msgs::ResetMap >
template<>
struct MD5Sum< ::se2_grid_map_generator_msgs::ResetMapResponse>
{
  static const char* value()
  {
    return MD5Sum< ::se2_grid_map_generator_msgs::ResetMap >::value();
  }
  static const char* value(const ::se2_grid_map_generator_msgs::ResetMapResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::se2_grid_map_generator_msgs::ResetMapResponse> should match
// service_traits::DataType< ::se2_grid_map_generator_msgs::ResetMap >
template<>
struct DataType< ::se2_grid_map_generator_msgs::ResetMapResponse>
{
  static const char* value()
  {
    return DataType< ::se2_grid_map_generator_msgs::ResetMap >::value();
  }
  static const char* value(const ::se2_grid_map_generator_msgs::ResetMapResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // SE2_GRID_MAP_GENERATOR_MSGS_MESSAGE_RESETMAP_H