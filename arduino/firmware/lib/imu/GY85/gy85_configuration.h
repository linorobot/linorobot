#ifndef _IMU_CONFIGURATION_H_
#define _IMU_CONFIGURATION_H_

#include "imu_wire.h"

// Set the baud rate
#define BAUD 115200

// Use advanced teensy i2c library from https://github.com/nox771/i2c_t3
// You can't have both
// #define WIRE_T3

// IMU Configuration
// Select your IMU board from the list below.
// If not avaliable, define a custom one but feel free to added it.
//#define SEN10724
#define GY85
//#define GY80
//#define OTHER

#if defined(SEN10724)
  // Accelerometer
  #define ADXL345
  #define ACC_X_AXIS 1
  #define ACC_Y_AXIS 0
  #define ACC_Z_AXIS 2
  #define ACC_X_INVERT 1
  #define ACC_Y_INVERT -1
  #define ACC_Z_INVERT 1
  // Gyroscope
  #define ITG3205
  #define GYRO_X_AXIS 1
  #define GYRO_Y_AXIS 0
  #define GYRO_Z_AXIS 2
  #define GYRO_X_INVERT 1
  #define GYRO_Y_INVERT 1
  #define GYRO_Z_INVERT 1
  // Magnetometer
  #define HMC5883L
  #define MAG_X_AXIS 0
  #define MAG_Y_AXIS 2
  #define MAG_Z_AXIS 1
  #define MAG_X_INVERT -1
  #define MAG_Y_INVERT -1
  #define MAG_Z_INVERT -1
#elif defined(GY85)
  // Accelerometer
  #define ADXL345
  #define ACC_X_AXIS 0
  #define ACC_Y_AXIS 1
  #define ACC_Z_AXIS 2
  #define ACC_X_INVERT 1
  #define ACC_Y_INVERT 1
  #define ACC_Z_INVERT 1
  // Gyroscope
  #define ITG3205
  #define GYRO_X_AXIS 0
  #define GYRO_Y_AXIS 1
  #define GYRO_Z_AXIS 2
  #define GYRO_X_INVERT 1
  #define GYRO_Y_INVERT 1
  #define GYRO_Z_INVERT 1
  // Magnetometer
  #define HMC5883L
  #define MAG_X_AXIS 0
  #define MAG_Y_AXIS 2
  #define MAG_Z_AXIS 1
  #define MAG_X_INVERT 1
  #define MAG_Y_INVERT 1
  #define MAG_Z_INVERT 1
#elif defined(GY80)
  // Accelerometer
  #define ADXL345
  #define ACC_X_AXIS 0
  #define ACC_Y_AXIS 1
  #define ACC_Z_AXIS 2
  #define ACC_X_INVERT 1
  #define ACC_Y_INVERT 1
  #define ACC_Z_INVERT 1
  // Gyroscope
  #define L3G4200D
  #define GYRO_X_AXIS 0
  #define GYRO_Y_AXIS 1
  #define GYRO_Z_AXIS 2
  #define GYRO_X_INVERT 1
  #define GYRO_Y_INVERT 1
  #define GYRO_Z_INVERT 1
  // Magnetometer
  #define HMC5883L
  #define MAG_X_AXIS 0
  #define MAG_Y_AXIS 2
  #define MAG_Z_AXIS 1
  #define MAG_X_INVERT 1
  #define MAG_Y_INVERT 1
  #define MAG_Z_INVERT 1
#elif defined(OTHER)
  // Accelerometer
  #define ADXL345
  //Change the axis the read order
  #define ACC_X_AXIS 1
  #define ACC_Y_AXIS 0
  #define ACC_Z_AXIS 2
  //Invervt the axis
  #define ACC_X_INVERT  1
  #define ACC_Y_INVERT -1
  #define ACC_Z_INVERT  1
  //Gyroscope
  #define ITG3205
  //#define L3G4200D
  #define GYRO_X_AXIS 1
  #define GYRO_Y_AXIS 0
  #define GYRO_Z_AXIS 2
  #define GYRO_X_INVERT 1
  #define GYRO_Y_INVERT -1
  #define GYRO_Z_INVERT 1
  // Magnetometer
  #define HMC5883L
  #define MAG_X_AXIS 0
  #define MAG_Y_AXIS 2
  #define MAG_Z_AXIS 1
  #define MAG_X_INVERT -1
  #define MAG_Y_INVERT -1
  #define MAG_Z_INVERT -1
#endif

#include "accelerometer.h"
#include "gyroscope.h"
#include "magnetometer.h"

#endif  // _IMU_CONFIGURATION_H_

