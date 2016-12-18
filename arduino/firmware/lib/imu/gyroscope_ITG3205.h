#ifndef _GYROSCOPE_ITFG3205_H_
#define _GYROSCOPE_ITFG3205_H_

//ITG3205 REGISTERS
#define ITG3205_GYRO_ADDRESS 0x68
#define ITG3205_WHO_AM_I 0x00
#define ITG3205_PWR_MGM 0x3E
#define ITG3205_RESET 0x80
#define ITG3205_DLPF_FS 0x16
#define ITG3205_SCALE 0.00121414209  //rad/s

bool checkGyroscope()
{
  if ((check_ID(ITG3205_GYRO_ADDRESS,ITG3205_WHO_AM_I) & 0x7E) == ITG3205_GYRO_ADDRESS)
  {
    write_to_register(ITG3205_GYRO_ADDRESS,ITG3205_PWR_MGM,ITG3205_RESET); //reset the gyro
    delay(5);
    write_to_register(ITG3205_GYRO_ADDRESS,ITG3205_DLPF_FS,0x1B); // Set LP filter bandwidth to 42Hz
    delay(5);
    write_to_register(ITG3205_GYRO_ADDRESS,0x15,0x13); // sample rate to to 50Hz
    delay(5);
    write_to_register(ITG3205_GYRO_ADDRESS,ITG3205_PWR_MGM,0x03); // PLL with Z gyro ref
    delay(10);
    return true;
  }
  else
    return false;
}

void measureGyroscope()
{
  gyro_reads = 0;;
  send_value(ITG3205_GYRO_ADDRESS,0x1D);
  Wire.requestFrom(ITG3205_GYRO_ADDRESS,6);
  while(Wire.available())
  {
    gyro_buffer[gyro_reads] = Wire.read();
    gyro_reads++;
  }
  raw_rotation.x = (float)(GYRO_X_INVERT*(int16_t)(((int)gyro_buffer[2*GYRO_X_AXIS] <<8) | gyro_buffer[2*GYRO_X_AXIS+1])) * ITG3205_SCALE;  //rad/s
  raw_rotation.y = (float)(GYRO_Y_INVERT*(int16_t)(((int)gyro_buffer[2*GYRO_Y_AXIS] <<8) | gyro_buffer[2*GYRO_Y_AXIS+1])) * ITG3205_SCALE;
  raw_rotation.z = (float)(GYRO_Z_INVERT*(int16_t)(((int)gyro_buffer[2*GYRO_Z_AXIS] <<8) | gyro_buffer[2*GYRO_Z_AXIS+1])) * ITG3205_SCALE;


}
#endif  // _GYROSCOPE_ITFG3205_H_
