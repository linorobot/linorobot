#ifndef _MAGNETOMETER_HMC5883L_H_
#define _MAGNETOMETER_HMC5883L_H_

//HMC5883L REGISTERS
#define HMC5883L_MAG_ADDRESS 0x1E
#define HMC5883L_MAG_ID 0x10
#define HMC5883L_MAG_REG_A 0x00
#define HMC5883L_MAG_REG_B 0x01
#define HMC5883L_MAG_MODE 0x02
#define HMC5883L_MAG_DATAX0 0x03
#define HMC5883L_MAG_GAIN 0x20  //Default gain
#define HMC5883L_MAG_SCALE 0.92  // mG/LSb

bool checkMagnetometer()
{
  write_to_register(HMC5883L_MAG_ADDRESS,HMC5883L_MAG_REG_B,HMC5883L_MAG_GAIN);  //Sets the gain
  delay(5);
  write_to_register(HMC5883L_MAG_ADDRESS,HMC5883L_MAG_REG_A,0x18); //75Hz output
  delay(5);
  write_to_register(HMC5883L_MAG_ADDRESS,HMC5883L_MAG_MODE,0x01); //Single-Measurement Mode
  delay(5);
  return true;;
}

void measureMagnetometer()
{
  mag_reads = 0;
  send_value(HMC5883L_MAG_ADDRESS,HMC5883L_MAG_DATAX0);
  Wire.requestFrom(HMC5883L_MAG_ADDRESS,6);
  while(Wire.available())
  {
    mag_buffer[mag_reads] = Wire.read();
    mag_reads++;
  }
  raw_magnetic_field.x =  (float)(MAG_X_INVERT * ((int16_t)((int)mag_buffer[2*MAG_X_AXIS] << 8) | (mag_buffer[2*MAG_X_AXIS+1]))) * HMC5883L_MAG_SCALE;
  raw_magnetic_field.y =  (float)(MAG_Y_INVERT * ((int16_t)((int)mag_buffer[2*MAG_Y_AXIS] << 8) | (mag_buffer[2*MAG_Y_AXIS+1]))) * HMC5883L_MAG_SCALE;
  raw_magnetic_field.z =  (float)(MAG_Z_INVERT * ((int16_t)((int)mag_buffer[2*MAG_Z_AXIS] << 8) | (mag_buffer[2*MAG_Z_AXIS+1]))) * HMC5883L_MAG_SCALE;
  write_to_register(HMC5883L_MAG_ADDRESS,HMC5883L_MAG_MODE,0x01);

}

#endif  // _MAGNETOMETER_HMC5883L_H_
