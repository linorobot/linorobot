#ifndef _ACCELEROMETER_ADXL345_H_
#define _ACCELEROMETER_ADXL345_H_

#include "accelerometer.h"

//ADXL345 REGISTERS
#define ADXL345_DEVID 0x00
#define ADXL345_BW_RATE 0x2C
#define ADXL345_POWER_CTL 0x2D
#define ADXL345_DATA_FORMAT 0x31
#define ADXL345_DATAX0 0x32
#define ADXL345_ACCELEROMETER_ADDRESS 0x53
#define ADXL345_DEVICE_ID 0xE5
#define ADXL345_SCALE 25.60000

bool checkAccelerometer()
{
  if (check_ID(ADXL345_ACCELEROMETER_ADDRESS,ADXL345_DEVID) == ADXL345_DEVICE_ID)
  {
    write_to_register(ADXL345_ACCELEROMETER_ADDRESS,ADXL345_POWER_CTL,0x08);  //D3, enables measuring
    delay(5);
    write_to_register(ADXL345_ACCELEROMETER_ADDRESS,ADXL345_DATA_FORMAT,0x09); //D3 and D0, enables FULL_RES and +/-4g
    delay(5);
    write_to_register(ADXL345_ACCELEROMETER_ADDRESS,ADXL345_BW_RATE,0x09); //Set the bw to 0 Hz
    delay(5);
    return true;
  }
  else
    return false;
}

void measureAcceleration()
{
  acc_reads = 0;
  send_value(ADXL345_ACCELEROMETER_ADDRESS, ADXL345_DATAX0);
  Wire.requestFrom(ADXL345_ACCELEROMETER_ADDRESS, 6);
  while(Wire.available())
  {
    acc_buffer[acc_reads] = Wire.read();
    acc_reads++;
  }

  raw_acceleration.x =  ((float)ACC_X_INVERT*(int16_t)((int)acc_buffer[2*ACC_X_AXIS+1]<<8 | acc_buffer[2*ACC_X_AXIS]) / ADXL345_SCALE);
  raw_acceleration.y =  ((float)ACC_Y_INVERT*(int16_t)((int)acc_buffer[2*ACC_Y_AXIS+1]<<8 | acc_buffer[2*ACC_Y_AXIS]) / ADXL345_SCALE);
  raw_acceleration.z =  ((float)ACC_Z_INVERT*(int16_t)((int)acc_buffer[2*ACC_Z_AXIS+1]<<8 | acc_buffer[2*ACC_Z_AXIS]) / ADXL345_SCALE);

}

#endif  // _ACCELEROMETER_ADXL345_H_
