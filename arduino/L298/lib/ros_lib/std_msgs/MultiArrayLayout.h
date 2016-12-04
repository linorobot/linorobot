#ifndef _ROS_std_msgs_MultiArrayLayout_h
#define _ROS_std_msgs_MultiArrayLayout_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/MultiArrayDimension.h"

namespace std_msgs
{

  class MultiArrayLayout : public ros::Msg
  {
    public:
      uint8_t dim_length;
      std_msgs::MultiArrayDimension st_dim;
      std_msgs::MultiArrayDimension * dim;
      uint32_t data_offset;

    MultiArrayLayout():
      dim_length(0), dim(NULL),
      data_offset(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset++) = dim_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < dim_length; i++){
      offset += this->dim[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->data_offset >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->data_offset >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->data_offset >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->data_offset >> (8 * 3)) & 0xFF;
      offset += sizeof(this->data_offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint8_t dim_lengthT = *(inbuffer + offset++);
      if(dim_lengthT > dim_length)
        this->dim = (std_msgs::MultiArrayDimension*)realloc(this->dim, dim_lengthT * sizeof(std_msgs::MultiArrayDimension));
      offset += 3;
      dim_length = dim_lengthT;
      for( uint8_t i = 0; i < dim_length; i++){
      offset += this->st_dim.deserialize(inbuffer + offset);
        memcpy( &(this->dim[i]), &(this->st_dim), sizeof(std_msgs::MultiArrayDimension));
      }
      this->data_offset =  ((uint32_t) (*(inbuffer + offset)));
      this->data_offset |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->data_offset |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->data_offset |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->data_offset);
     return offset;
    }

    const char * getType(){ return "std_msgs/MultiArrayLayout"; };
    const char * getMD5(){ return "0fed2a11c13e11c5571b4e2a995a91a3"; };

  };

}
#endif