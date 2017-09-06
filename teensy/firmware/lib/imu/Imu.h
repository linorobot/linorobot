#ifndef _IMU_H_
#define _IMU_H_

#ifdef USE_GY85_IMU
    #include "GY85/gy85_configuration.h"
#endif

#ifdef USE_MP6050_IMU
    #include "MP6050/test.h"
#endif

bool initIMU()
{
    Wire.begin();
    delay(5);
    bool accel, gyro, mag;
    accel = initAccelerometer();
    gyro = initGyroscope();
    mag = initMagnetometer();

    if(accel && gyro && mag)
        return true;
    
    else
        return false;
}

geometry_msgs::Vector3 readAccelerometer()
{
    return readIMUaccelerometer();
}

geometry_msgs::Vector3 readGyroscope()
{
    return readIMUgyroscope();
}

geometry_msgs::Vector3 readMagnetometer()
{
    return readIMUmagnetometer();
}


#endif