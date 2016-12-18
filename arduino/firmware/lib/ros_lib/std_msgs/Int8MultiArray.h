#ifndef _ROS_std_msgs_Int8MultiArray_h
#define _ROS_std_msgs_Int8MultiArray_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/MultiArrayLayout.h"

namespace std_msgs
{

  class Int8MultiArray : public ros::Msg
  {
    public:
      std_msgs::MultiArrayLayout layout;
      uint8_t data_length;
      int8_t st_data;
      int8_t * data;

    Int8MultiArray():
      layout(),
      data_length(0), data(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->layout.serialize(outbuffer + offset);
      *(outbuffer + offset++) = data_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < data_length; i++){
      union {
        int8_t real;
        uint8_t base;
      } u_datai;
      u_datai.real = this->data[i];
      *(outbuffer + offset + 0) = (u_datai.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->data[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->layout.deserialize(inbuffer + offset);
      uint8_t data_lengthT = *(inbuffer + offset++);
      if(data_lengthT > data_length)
        this->data = (int8_t*)realloc(this->data, data_lengthT * sizeof(int8_t));
      offset += 3;
      data_length = data_lengthT;
      for( uint8_t i = 0; i < data_length; i++){
      union {
        int8_t real;
        uint8_t base;
      } u_st_data;
      u_st_data.base = 0;
      u_st_data.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->st_data = u_st_data.real;
      offset += sizeof(this->st_data);
        memcpy( &(this->data[i]), &(this->st_data), sizeof(int8_t));
      }
     return offset;
    }

    const char * getType(){ return "std_msgs/Int8MultiArray"; };
    const char * getMD5(){ return "d7c1af35a1b4781bbe79e03dd94b7c13"; };

  };

}
#endif