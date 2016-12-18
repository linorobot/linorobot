#ifndef _MAGNETOMETER_H_
#define _MAGNETOMETER_H_

uint8_t mag_reads = 0;
byte mag_buffer[6];

bool checkMagnetometer();
void measureMagnetometer();
geometry_msgs::Vector3 raw_magnetic_field;

#if defined(HMC5883L)
  #include "magnetometer_HMC5883L.h"
#endif

#endif  // _MAGNETOMETER_H_
