#ifndef _ROS_ros_arduino_msgs_RawImu_h
#define _ROS_ros_arduino_msgs_RawImu_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Vector3.h"

namespace ros_arduino_msgs
{

  class RawImu : public ros::Msg
  {
    public:
      std_msgs::Header header;
      bool accelerometer;
      bool gyroscope;
      bool magnetometer;
      geometry_msgs::Vector3 raw_linear_acceleration;
      geometry_msgs::Vector3 raw_angular_velocity;
      geometry_msgs::Vector3 raw_magnetic_field;

    RawImu():
      header(),
      accelerometer(0),
      gyroscope(0),
      magnetometer(0),
      raw_linear_acceleration(),
      raw_angular_velocity(),
      raw_magnetic_field()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_accelerometer;
      u_accelerometer.real = this->accelerometer;
      *(outbuffer + offset + 0) = (u_accelerometer.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->accelerometer);
      union {
        bool real;
        uint8_t base;
      } u_gyroscope;
      u_gyroscope.real = this->gyroscope;
      *(outbuffer + offset + 0) = (u_gyroscope.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->gyroscope);
      union {
        bool real;
        uint8_t base;
      } u_magnetometer;
      u_magnetometer.real = this->magnetometer;
      *(outbuffer + offset + 0) = (u_magnetometer.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->magnetometer);
      offset += this->raw_linear_acceleration.serialize(outbuffer + offset);
      offset += this->raw_angular_velocity.serialize(outbuffer + offset);
      offset += this->raw_magnetic_field.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_accelerometer;
      u_accelerometer.base = 0;
      u_accelerometer.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->accelerometer = u_accelerometer.real;
      offset += sizeof(this->accelerometer);
      union {
        bool real;
        uint8_t base;
      } u_gyroscope;
      u_gyroscope.base = 0;
      u_gyroscope.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->gyroscope = u_gyroscope.real;
      offset += sizeof(this->gyroscope);
      union {
        bool real;
        uint8_t base;
      } u_magnetometer;
      u_magnetometer.base = 0;
      u_magnetometer.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->magnetometer = u_magnetometer.real;
      offset += sizeof(this->magnetometer);
      offset += this->raw_linear_acceleration.deserialize(inbuffer + offset);
      offset += this->raw_angular_velocity.deserialize(inbuffer + offset);
      offset += this->raw_magnetic_field.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "ros_arduino_msgs/RawImu"; };
    const char * getMD5(){ return "3bc0ea37781da51ad41a6868ff62faa9"; };

  };

}
#endif