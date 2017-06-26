#ifndef _ACCELEROMETER_H_
#define _ACCELEROMETER_H_

uint8_t acc_reads = 0;
byte acc_buffer[6];

bool initAccelerometer();

geometry_msgs::Vector3 readIMUaccelerometer();
geometry_msgs::Vector3 acceleration;

#if defined(ADXL345)
    #include "accelerometer_ADXL345.h"
#endif

#endif  // _ACCELEROMETER_H_
