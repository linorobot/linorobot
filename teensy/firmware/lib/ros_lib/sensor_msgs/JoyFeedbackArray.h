#ifndef _ROS_sensor_msgs_JoyFeedbackArray_h
#define _ROS_sensor_msgs_JoyFeedbackArray_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "sensor_msgs/JoyFeedback.h"

namespace sensor_msgs
{

  class JoyFeedbackArray : public ros::Msg
  {
    public:
      uint8_t array_length;
      sensor_msgs::JoyFeedback st_array;
      sensor_msgs::JoyFeedback * array;

    JoyFeedbackArray():
      array_length(0), array(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset++) = array_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < array_length; i++){
      offset += this->array[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint8_t array_lengthT = *(inbuffer + offset++);
      if(array_lengthT > array_length)
        this->array = (sensor_msgs::JoyFeedback*)realloc(this->array, array_lengthT * sizeof(sensor_msgs::JoyFeedback));
      offset += 3;
      array_length = array_lengthT;
      for( uint8_t i = 0; i < array_length; i++){
      offset += this->st_array.deserialize(inbuffer + offset);
        memcpy( &(this->array[i]), &(this->st_array), sizeof(sensor_msgs::JoyFeedback));
      }
     return offset;
    }

    const char * getType(){ return "sensor_msgs/JoyFeedbackArray"; };
    const char * getMD5(){ return "cde5730a895b1fc4dee6f91b754b213d"; };

  };

}
#endif