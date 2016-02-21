#ifndef _ROS_sensor_msgs_RelativeHumidity_h
#define _ROS_sensor_msgs_RelativeHumidity_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace sensor_msgs
{

  class RelativeHumidity : public ros::Msg
  {
    public:
      std_msgs::Header header;
      double relative_humidity;
      double variance;

    RelativeHumidity():
      header(),
      relative_humidity(0),
      variance(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        double real;
        uint64_t base;
      } u_relative_humidity;
      u_relative_humidity.real = this->relative_humidity;
      *(outbuffer + offset + 0) = (u_relative_humidity.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_relative_humidity.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_relative_humidity.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_relative_humidity.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_relative_humidity.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_relative_humidity.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_relative_humidity.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_relative_humidity.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->relative_humidity);
      union {
        double real;
        uint64_t base;
      } u_variance;
      u_variance.real = this->variance;
      *(outbuffer + offset + 0) = (u_variance.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_variance.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_variance.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_variance.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_variance.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_variance.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_variance.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_variance.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->variance);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        double real;
        uint64_t base;
      } u_relative_humidity;
      u_relative_humidity.base = 0;
      u_relative_humidity.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_relative_humidity.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_relative_humidity.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_relative_humidity.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_relative_humidity.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_relative_humidity.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_relative_humidity.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_relative_humidity.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->relative_humidity = u_relative_humidity.real;
      offset += sizeof(this->relative_humidity);
      union {
        double real;
        uint64_t base;
      } u_variance;
      u_variance.base = 0;
      u_variance.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_variance.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_variance.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_variance.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_variance.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_variance.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_variance.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_variance.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->variance = u_variance.real;
      offset += sizeof(this->variance);
     return offset;
    }

    const char * getType(){ return "sensor_msgs/RelativeHumidity"; };
    const char * getMD5(){ return "8730015b05955b7e992ce29a2678d90f"; };

  };

}
#endif