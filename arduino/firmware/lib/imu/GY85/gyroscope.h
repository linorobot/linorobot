#ifndef _GYROSCOPE_H_
#define _GYROSCOPE_H_

uint8_t gyro_reads = 0;
byte gyro_buffer[6];

bool initGyroscope();

geometry_msgs::Vector3 angular_velocity;
geometry_msgs::Vector3 readIMUgyroscope();

#if defined(ITG3205)
  #include "gyroscope_ITG3205.h"
#elif defined(L3G4200D)
  #include "gyroscope_L3G4200D.h"
#endif

#endif  // _GYROSCOPE_H_

