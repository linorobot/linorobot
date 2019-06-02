#ifndef LINO_BASE_CONFIG_H
#define LINO_BASE_CONFIG_H

//uncomment the base you're building
#define LINO_BASE DIFFERENTIAL_DRIVE // 2WD and Tracked robot w/ 2 motors
// #define LINO_BASE SKID_STEER      // 4WD robot
// #define LINO_BASE ACKERMANN       // Car-like steering robot w/ 2 motors
// #define LINO_BASE ACKERMANN1      // Car-like steering robot w/ 1 motor
// #define LINO_BASE MECANUM         // Mecanum drive robot

//uncomment the motor driver you're using
#define USE_L298_DRIVER
// #define USE_BTS7960_DRIVER
// #define USE_ESC

//uncomment the IMU you're using
#define USE_GY85_IMU
// #define USE_MPU6050_IMU
// #define USE_MPU9150_IMU
// #define USE_MPU9250_IMU

#define DEBUG 1

#define K_P 0.6 // P constant
#define K_I 0.3 // I constant
#define K_D 0.5 // D constant

//define your robot' specs here
#define MAX_RPM 330               // motor's maximum RPM
#define COUNTS_PER_REV 1550       // wheel encoder's no of ticks per rev
#define WHEEL_DIAMETER 0.10       // wheel's diameter in meters
#define PWM_BITS 8                // PWM Resolution of the microcontroller
#define LR_WHEELS_DISTANCE 0.235  // distance between left and right wheels
#define FR_WHEELS_DISTANCE 0.30   // distance between front and rear wheels. Ignore this if you're on 2WD/ACKERMANN
#define MAX_STEERING_ANGLE 0.415  // max steering angle. This only applies to Ackermann steering

//=================BIGGER ROBOT SPEC (BTS7960)=============================
// #define K_P 0.05  // P constant
// #define K_I 0.9   // I constant
// #define K_D 0.1   // D constant

// define your robot' specs here
// #define MAX_RPM 45               // motor's maximum RPM
// #define COUNTS_PER_REV 4000      // wheel encoder's no of ticks per rev
// #define WHEEL_DIAMETER 0.15      // wheel's diameter in meters
// #define PWM_BITS 8               // PWM Resolution of the microcontroller
// #define LR_WHEELS_DISTANCE 0.32  // distance between left and right wheels
// #define FR_WHEELS_DISTANCE 0.38  // distance between front and back wheels. Ignore this if you're on 2WD/ACKERMANN
//================= END OF BIGGER ROBOT SPEC =============================

/*
ROBOT ORIENTATION
         FRONT
    MOTOR1  MOTOR2  (2WD/ACKERMANN)
    MOTOR3  MOTOR4  (4WD/MECANUM)
         BACK
*/

// Arduino Mega pins used in the setup below:
// External Interrupts: 2 (interrupt 0), 3 (interrupt 1), 18 (interrupt 5),
// 19 (interrupt 4), 20 (interrupt 3), and 21 (interrupt 2)
// I2C: 20 (SDA) and 21 (SCL)
// Interrupt pins used:
// Int0 Pin 2    --> MOTOR1_ENCODER_A
// Int1 Pin 3    --> MOTOR1_ENCODER_B
// Int5 Pin 18   --> MOTOR2_ENCODER_A
// Int4 Pin 19   --> MOTOR2_ENCODER_B
// Int3 Pin 20 SDA --> for IMU I2C
// Int2 Pin 21 SCL --> for IMU I2C
// SPI: 50 (MISO), 51 (MOSI), 52 (SCK), 53 (SS)

// Connection to the IBT-2 board:
// LOOK AT: linorobot/teensy/firmware/lib/config/IBT-2-Input-Ports.jpg
// IBT-2 (BTS7960) pin 1 (RPWM) to Arduino (PWM) pins - for the left motor 4 / for the right motor 6
// IBT-2 (BTS7960) pin 2 (LPWM) to Arduino (PWM) pins - for the left motor 5 / for the right motor 7
// IBT-2 (BTS7960) pins 3 (R_EN), 4 (L_EN), 7 (VCC) to Arduino 5V pin
// IBT-2 (BTS7960) pin 8 (GND) to Arduino GND
// IBT-2 (BTS7960) pins 5 (R_IS) and 6 (L_IS) not connected

// Arduino Mega PWM pins: 2 - 13 and 44 - 46

// PWM pins used:
// Pin  4 MOTOR1_IN_A
// Pin  5 MOTOR1_IN_B
// Pin  6 MOTOR2_IN_A
// Pin  7 MOTOR2_IN_B
// Pins 30 to 42 are used as placeholders!
// */


// ENCODER PINS
#define MOTOR1_ENCODER_A 2  //Int0 Pin 2
#define MOTOR1_ENCODER_B 3  //Int1 Pin 3

#define MOTOR2_ENCODER_A 18 //Int2 Pin 18
#define MOTOR2_ENCODER_B 19 //Int3 Pin 19

#define MOTOR3_ENCODER_A 30  //Placeholder (set to unused mega pin)
#define MOTOR3_ENCODER_B 31  //Placeholder (set to unused mega pin)

#define MOTOR4_ENCODER_A 32  //Placeholder (set to unused mega pin)
#define MOTOR4_ENCODER_B 33  //Placeholder (set to unused mega pin)


//MOTOR PINS

#define MOTOR_DRIVER L298

#define MOTOR1_PWM 4
#define MOTOR1_IN_A 44
#define MOTOR1_IN_B 45

#define MOTOR2_PWM 13
#define MOTOR2_IN_A 46
#define MOTOR2_IN_B 47

#define MOTOR3_PWM 36   //Placeholder (set to unused mega pin)
#define MOTOR3_IN_A 37  //Placeholder (set to unused mega pin)
#define MOTOR3_IN_B 38  //Placeholder (set to unused mega pin)

#define MOTOR4_PWM 39   //Placeholder (set to unused mega pin)
#define MOTOR4_IN_A 40  //Placeholder (set to unused mega pin)
#define MOTOR4_IN_B 41  //Placeholder (set to unused mega pin)

#define STEERING_PIN 7

#define PWM_MAX 255
#define PWM_MIN -PWM_MAX

#endif
