#ifndef _ROS_tf_tfMessage_h
#define _ROS_tf_tfMessage_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/TransformStamped.h"

namespace tf
{

  class tfMessage : public ros::Msg
  {
    public:
      uint8_t transforms_length;
      geometry_msgs::TransformStamped st_transforms;
      geometry_msgs::TransformStamped * transforms;

    tfMessage():
      transforms_length(0), transforms(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset++) = transforms_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < transforms_length; i++){
      offset += this->transforms[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint8_t transforms_lengthT = *(inbuffer + offset++);
      if(transforms_lengthT > transforms_length)
        this->transforms = (geometry_msgs::TransformStamped*)realloc(this->transforms, transforms_lengthT * sizeof(geometry_msgs::TransformStamped));
      offset += 3;
      transforms_length = transforms_lengthT;
      for( uint8_t i = 0; i < transforms_length; i++){
      offset += this->st_transforms.deserialize(inbuffer + offset);
        memcpy( &(this->transforms[i]), &(this->st_transforms), sizeof(geometry_msgs::TransformStamped));
      }
     return offset;
    }

    const char * getType(){ return "tf/tfMessage"; };
    const char * getMD5(){ return "94810edda583a504dfda3829e70d7eec"; };

  };

}
#endif