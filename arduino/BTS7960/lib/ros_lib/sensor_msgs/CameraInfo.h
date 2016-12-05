#ifndef _ROS_sensor_msgs_CameraInfo_h
#define _ROS_sensor_msgs_CameraInfo_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/RegionOfInterest.h"

namespace sensor_msgs
{

  class CameraInfo : public ros::Msg
  {
    public:
      std_msgs::Header header;
      uint32_t height;
      uint32_t width;
      const char* distortion_model;
      uint8_t D_length;
      double st_D;
      double * D;
      double K[9];
      double R[9];
      double P[12];
      uint32_t binning_x;
      uint32_t binning_y;
      sensor_msgs::RegionOfInterest roi;

    CameraInfo():
      header(),
      height(0),
      width(0),
      distortion_model(""),
      D_length(0), D(NULL),
      K(),
      R(),
      P(),
      binning_x(0),
      binning_y(0),
      roi()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->height >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->height >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->height >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->height >> (8 * 3)) & 0xFF;
      offset += sizeof(this->height);
      *(outbuffer + offset + 0) = (this->width >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->width >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->width >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->width >> (8 * 3)) & 0xFF;
      offset += sizeof(this->width);
      uint32_t length_distortion_model = strlen(this->distortion_model);
      memcpy(outbuffer + offset, &length_distortion_model, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->distortion_model, length_distortion_model);
      offset += length_distortion_model;
      *(outbuffer + offset++) = D_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < D_length; i++){
      union {
        double real;
        uint64_t base;
      } u_Di;
      u_Di.real = this->D[i];
      *(outbuffer + offset + 0) = (u_Di.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Di.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Di.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Di.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_Di.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_Di.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_Di.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_Di.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->D[i]);
      }
      for( uint8_t i = 0; i < 9; i++){
      union {
        double real;
        uint64_t base;
      } u_Ki;
      u_Ki.real = this->K[i];
      *(outbuffer + offset + 0) = (u_Ki.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Ki.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Ki.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Ki.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_Ki.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_Ki.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_Ki.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_Ki.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->K[i]);
      }
      for( uint8_t i = 0; i < 9; i++){
      union {
        double real;
        uint64_t base;
      } u_Ri;
      u_Ri.real = this->R[i];
      *(outbuffer + offset + 0) = (u_Ri.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Ri.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Ri.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Ri.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_Ri.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_Ri.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_Ri.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_Ri.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->R[i]);
      }
      for( uint8_t i = 0; i < 12; i++){
      union {
        double real;
        uint64_t base;
      } u_Pi;
      u_Pi.real = this->P[i];
      *(outbuffer + offset + 0) = (u_Pi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Pi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Pi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Pi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_Pi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_Pi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_Pi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_Pi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->P[i]);
      }
      *(outbuffer + offset + 0) = (this->binning_x >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->binning_x >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->binning_x >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->binning_x >> (8 * 3)) & 0xFF;
      offset += sizeof(this->binning_x);
      *(outbuffer + offset + 0) = (this->binning_y >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->binning_y >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->binning_y >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->binning_y >> (8 * 3)) & 0xFF;
      offset += sizeof(this->binning_y);
      offset += this->roi.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->height =  ((uint32_t) (*(inbuffer + offset)));
      this->height |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->height |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->height |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->height);
      this->width =  ((uint32_t) (*(inbuffer + offset)));
      this->width |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->width |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->width |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->width);
      uint32_t length_distortion_model;
      memcpy(&length_distortion_model, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_distortion_model; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_distortion_model-1]=0;
      this->distortion_model = (char *)(inbuffer + offset-1);
      offset += length_distortion_model;
      uint8_t D_lengthT = *(inbuffer + offset++);
      if(D_lengthT > D_length)
        this->D = (double*)realloc(this->D, D_lengthT * sizeof(double));
      offset += 3;
      D_length = D_lengthT;
      for( uint8_t i = 0; i < D_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_D;
      u_st_D.base = 0;
      u_st_D.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_D.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_D.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_D.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_D.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_D.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_D.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_D.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_D = u_st_D.real;
      offset += sizeof(this->st_D);
        memcpy( &(this->D[i]), &(this->st_D), sizeof(double));
      }
      for( uint8_t i = 0; i < 9; i++){
      union {
        double real;
        uint64_t base;
      } u_Ki;
      u_Ki.base = 0;
      u_Ki.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Ki.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Ki.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Ki.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_Ki.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_Ki.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_Ki.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_Ki.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->K[i] = u_Ki.real;
      offset += sizeof(this->K[i]);
      }
      for( uint8_t i = 0; i < 9; i++){
      union {
        double real;
        uint64_t base;
      } u_Ri;
      u_Ri.base = 0;
      u_Ri.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Ri.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Ri.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Ri.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_Ri.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_Ri.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_Ri.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_Ri.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->R[i] = u_Ri.real;
      offset += sizeof(this->R[i]);
      }
      for( uint8_t i = 0; i < 12; i++){
      union {
        double real;
        uint64_t base;
      } u_Pi;
      u_Pi.base = 0;
      u_Pi.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Pi.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Pi.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Pi.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_Pi.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_Pi.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_Pi.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_Pi.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->P[i] = u_Pi.real;
      offset += sizeof(this->P[i]);
      }
      this->binning_x =  ((uint32_t) (*(inbuffer + offset)));
      this->binning_x |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->binning_x |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->binning_x |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->binning_x);
      this->binning_y =  ((uint32_t) (*(inbuffer + offset)));
      this->binning_y |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->binning_y |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->binning_y |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->binning_y);
      offset += this->roi.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "sensor_msgs/CameraInfo"; };
    const char * getMD5(){ return "c9a58c1b0b154e0e6da7578cb991d214"; };

  };

}
#endif