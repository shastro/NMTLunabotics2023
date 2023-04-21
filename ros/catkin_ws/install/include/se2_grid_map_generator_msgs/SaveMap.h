// Generated by gencpp from file se2_grid_map_generator_msgs/SaveMap.msg
// DO NOT EDIT!


#ifndef SE2_GRID_MAP_GENERATOR_MSGS_MESSAGE_SAVEMAP_H
#define SE2_GRID_MAP_GENERATOR_MSGS_MESSAGE_SAVEMAP_H

#include <ros/service_traits.h>


#include <se2_grid_map_generator_msgs/SaveMapRequest.h>
#include <se2_grid_map_generator_msgs/SaveMapResponse.h>


namespace se2_grid_map_generator_msgs
{

struct SaveMap
{

typedef SaveMapRequest Request;
typedef SaveMapResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct SaveMap
} // namespace se2_grid_map_generator_msgs


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::se2_grid_map_generator_msgs::SaveMap > {
  static const char* value()
  {
    return "5922feee4022105a8dced2452aed4b3f";
  }

  static const char* value(const ::se2_grid_map_generator_msgs::SaveMap&) { return value(); }
};

template<>
struct DataType< ::se2_grid_map_generator_msgs::SaveMap > {
  static const char* value()
  {
    return "se2_grid_map_generator_msgs/SaveMap";
  }

  static const char* value(const ::se2_grid_map_generator_msgs::SaveMap&) { return value(); }
};


// service_traits::MD5Sum< ::se2_grid_map_generator_msgs::SaveMapRequest> should match
// service_traits::MD5Sum< ::se2_grid_map_generator_msgs::SaveMap >
template<>
struct MD5Sum< ::se2_grid_map_generator_msgs::SaveMapRequest>
{
  static const char* value()
  {
    return MD5Sum< ::se2_grid_map_generator_msgs::SaveMap >::value();
  }
  static const char* value(const ::se2_grid_map_generator_msgs::SaveMapRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::se2_grid_map_generator_msgs::SaveMapRequest> should match
// service_traits::DataType< ::se2_grid_map_generator_msgs::SaveMap >
template<>
struct DataType< ::se2_grid_map_generator_msgs::SaveMapRequest>
{
  static const char* value()
  {
    return DataType< ::se2_grid_map_generator_msgs::SaveMap >::value();
  }
  static const char* value(const ::se2_grid_map_generator_msgs::SaveMapRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::se2_grid_map_generator_msgs::SaveMapResponse> should match
// service_traits::MD5Sum< ::se2_grid_map_generator_msgs::SaveMap >
template<>
struct MD5Sum< ::se2_grid_map_generator_msgs::SaveMapResponse>
{
  static const char* value()
  {
    return MD5Sum< ::se2_grid_map_generator_msgs::SaveMap >::value();
  }
  static const char* value(const ::se2_grid_map_generator_msgs::SaveMapResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::se2_grid_map_generator_msgs::SaveMapResponse> should match
// service_traits::DataType< ::se2_grid_map_generator_msgs::SaveMap >
template<>
struct DataType< ::se2_grid_map_generator_msgs::SaveMapResponse>
{
  static const char* value()
  {
    return DataType< ::se2_grid_map_generator_msgs::SaveMap >::value();
  }
  static const char* value(const ::se2_grid_map_generator_msgs::SaveMapResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // SE2_GRID_MAP_GENERATOR_MSGS_MESSAGE_SAVEMAP_H