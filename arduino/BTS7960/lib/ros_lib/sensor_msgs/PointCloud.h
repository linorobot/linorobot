#ifndef _ROS_sensor_msgs_PointCloud_h
#define _ROS_sensor_msgs_PointCloud_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Point32.h"
#include "sensor_msgs/ChannelFloat32.h"

namespace sensor_msgs
{

  class PointCloud : public ros::Msg
  {
    public:
      std_msgs::Header header;
      uint8_t points_length;
      geometry_msgs::Point32 st_points;
      geometry_msgs::Point32 * points;
      uint8_t channels_length;
      sensor_msgs::ChannelFloat32 st_channels;
      sensor_msgs::ChannelFloat32 * channels;

    PointCloud():
      header(),
      points_length(0), points(NULL),
      channels_length(0), channels(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset++) = points_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < points_length; i++){
      offset += this->points[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset++) = channels_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < channels_length; i++){
      offset += this->channels[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint8_t points_lengthT = *(inbuffer + offset++);
      if(points_lengthT > points_length)
        this->points = (geometry_msgs::Point32*)realloc(this->points, points_lengthT * sizeof(geometry_msgs::Point32));
      offset += 3;
      points_length = points_lengthT;
      for( uint8_t i = 0; i < points_length; i++){
      offset += this->st_points.deserialize(inbuffer + offset);
        memcpy( &(this->points[i]), &(this->st_points), sizeof(geometry_msgs::Point32));
      }
      uint8_t channels_lengthT = *(inbuffer + offset++);
      if(channels_lengthT > channels_length)
        this->channels = (sensor_msgs::ChannelFloat32*)realloc(this->channels, channels_lengthT * sizeof(sensor_msgs::ChannelFloat32));
      offset += 3;
      channels_length = channels_lengthT;
      for( uint8_t i = 0; i < channels_length; i++){
      offset += this->st_channels.deserialize(inbuffer + offset);
        memcpy( &(this->channels[i]), &(this->st_channels), sizeof(sensor_msgs::ChannelFloat32));
      }
     return offset;
    }

    const char * getType(){ return "sensor_msgs/PointCloud"; };
    const char * getMD5(){ return "d8e9c3f5afbdd8a130fd1d2763945fca"; };

  };

}
#endif