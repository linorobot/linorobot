#ifndef _IMU_WIRE_H_
#define _IMU_WIRE_H_


void send_value(int device_address, byte value) {

  Wire.beginTransmission(device_address);
  Wire.write(value);
  Wire.endTransmission();
}

void write_to_register(int device_address, byte register_address, byte register_value) {

  Wire.beginTransmission(device_address);
  Wire.write(register_address);
  Wire.write(register_value);
  Wire.endTransmission();

}

byte check_ID(int device_address, byte register_address) {

  Wire.beginTransmission(device_address);
  Wire.write(register_address);
  Wire.endTransmission();
  delay(20);
  Wire.requestFrom(device_address, 1);
  return Wire.read();
}

#endif  // _IMU_WIRE_H_
