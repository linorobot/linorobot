#ifndef _IMU_CONFIG_H_
#define _IMU_CONFIG_H_

#include "I2Cdev.h"

#define G_TO_ACCEL 9.81

#ifdef USE_GY85_IMU
    #include "ADXL345.h"
    #include "ITG3200.h"
    #include "HMC5883L.h"

    #define ACCEL_SCALE 1 / 232 // 1/232 LSB/g
    #define GYRO_SCALE 1 / 14.375 // 1/14.375 LSB(%s)
    #define MAG_SCALE 0.92 

    ADXL345 accelerometer;
    ITG3200 gyroscope;
    HMC5883L magnetometer;
#endif

#ifdef USE_MPU6050_IMU
    #include "MPU6050.h"
    #include "fake_mag.h"

    #define ACCEL_SCALE 1 / 16384 // 1/16,384 LSB/g
    #define GYRO_SCALE 1 / 131 // 1/131 LSB(%s)
    #define MAG_SCALE 0 // Not used. This is just a placeholder
    
    MPU6050 accelerometer;
    MPU6050 gyroscope;    
    FakeMag magnetometer;
#endif

#endif

//ADXL345 https://www.sparkfun.com/datasheets/Sensors/Accelerometer/ADXL345.pdf
//MPU6050 https://store.invensense.com/datasheets/invensense/MPU-6050_DataSheet_V3%204.pdf
//ITG320 https://www.sparkfun.com/datasheets/Sensors/Gyro/PS-ITG-3200-00-01.4.pdf