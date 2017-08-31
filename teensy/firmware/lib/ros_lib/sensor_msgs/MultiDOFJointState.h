#ifndef _ROS_sensor_msgs_MultiDOFJointState_h
#define _ROS_sensor_msgs_MultiDOFJointState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Wrench.h"

namespace sensor_msgs
{

  class MultiDOFJointState : public ros::Msg
  {
    public:
      std_msgs::Header header;
      uint8_t joint_names_length;
      char* st_joint_names;
      char* * joint_names;
      uint8_t transforms_length;
      geometry_msgs::Transform st_transforms;
      geometry_msgs::Transform * transforms;
      uint8_t twist_length;
      geometry_msgs::Twist st_twist;
      geometry_msgs::Twist * twist;
      uint8_t wrench_length;
      geometry_msgs::Wrench st_wrench;
      geometry_msgs::Wrench * wrench;

    MultiDOFJointState():
      header(),
      joint_names_length(0), joint_names(NULL),
      transforms_length(0), transforms(NULL),
      twist_length(0), twist(NULL),
      wrench_length(0), wrench(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset++) = joint_names_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < joint_names_length; i++){
      uint32_t length_joint_namesi = strlen(this->joint_names[i]);
      memcpy(outbuffer + offset, &length_joint_namesi, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->joint_names[i], length_joint_namesi);
      offset += length_joint_namesi;
      }
      *(outbuffer + offset++) = transforms_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < transforms_length; i++){
      offset += this->transforms[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset++) = twist_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < twist_length; i++){
      offset += this->twist[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset++) = wrench_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < wrench_length; i++){
      offset += this->wrench[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint8_t joint_names_lengthT = *(inbuffer + offset++);
      if(joint_names_lengthT > joint_names_length)
        this->joint_names = (char**)realloc(this->joint_names, joint_names_lengthT * sizeof(char*));
      offset += 3;
      joint_names_length = joint_names_lengthT;
      for( uint8_t i = 0; i < joint_names_length; i++){
      uint32_t length_st_joint_names;
      memcpy(&length_st_joint_names, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_joint_names; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_joint_names-1]=0;
      this->st_joint_names = (char *)(inbuffer + offset-1);
      offset += length_st_joint_names;
        memcpy( &(this->joint_names[i]), &(this->st_joint_names), sizeof(char*));
      }
      uint8_t transforms_lengthT = *(inbuffer + offset++);
      if(transforms_lengthT > transforms_length)
        this->transforms = (geometry_msgs::Transform*)realloc(this->transforms, transforms_lengthT * sizeof(geometry_msgs::Transform));
      offset += 3;
      transforms_length = transforms_lengthT;
      for( uint8_t i = 0; i < transforms_length; i++){
      offset += this->st_transforms.deserialize(inbuffer + offset);
        memcpy( &(this->transforms[i]), &(this->st_transforms), sizeof(geometry_msgs::Transform));
      }
      uint8_t twist_lengthT = *(inbuffer + offset++);
      if(twist_lengthT > twist_length)
        this->twist = (geometry_msgs::Twist*)realloc(this->twist, twist_lengthT * sizeof(geometry_msgs::Twist));
      offset += 3;
      twist_length = twist_lengthT;
      for( uint8_t i = 0; i < twist_length; i++){
      offset += this->st_twist.deserialize(inbuffer + offset);
        memcpy( &(this->twist[i]), &(this->st_twist), sizeof(geometry_msgs::Twist));
      }
      uint8_t wrench_lengthT = *(inbuffer + offset++);
      if(wrench_lengthT > wrench_length)
        this->wrench = (geometry_msgs::Wrench*)realloc(this->wrench, wrench_lengthT * sizeof(geometry_msgs::Wrench));
      offset += 3;
      wrench_length = wrench_lengthT;
      for( uint8_t i = 0; i < wrench_length; i++){
      offset += this->st_wrench.deserialize(inbuffer + offset);
        memcpy( &(this->wrench[i]), &(this->st_wrench), sizeof(geometry_msgs::Wrench));
      }
     return offset;
    }

    const char * getType(){ return "sensor_msgs/MultiDOFJointState"; };
    const char * getMD5(){ return "690f272f0640d2631c305eeb8301e59d"; };

  };

}
#endif