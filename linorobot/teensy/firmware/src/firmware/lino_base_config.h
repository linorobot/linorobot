#ifndef LINO_BASE_CONFIG_H
#define LINO_BASE_CONFIG_H

//uncomment the base you're building
//#define LINO_BASE DIFFERENTIAL_DRIVE // 2WD and Tracked robot w/ 2 motors
#define LINO_BASE SKID_STEER      // 4WD robot
// #define LINO_BASE ACKERMANN       // Car-like steering robot w/ 2 motors
// #define LINO_BASE ACKERMANN1      // Car-like steering robot w/ 1 motor
// #define LINO_BASE MECANUM         // Mecanum drive robot

//uncomment the motor driver you're using
#define USE_L298_DRIVER
// #define USE_BTS7960_DRIVER
// #define USE_ESC

//uncomment the IMU you're using
//#define USE_GY85_IMU
 #define USE_MPU6050_IMU
// #define USE_MPU9150_IMU
// #define USE_MPU9250_IMU

#define DEBUG 1

#define K_P 1.2 // P constant
#define K_I 0.7 // I constant
#define K_D 0.5 // D constant

//define your robot' specs here
#define MAX_RPM 170               // motor's maximum RPM
#define COUNTS_PER_REV 1380       // wheel encoder's no of ticks per rev
#define WHEEL_DIAMETER 0.065      // wheel's diameter in meters
#define PWM_BITS 8                // PWM Resolution of the microcontroller
#define LR_WHEELS_DISTANCE 0.262  // distance between left and right wheels
#define FR_WHEELS_DISTANCE 0.222   // distance between front and rear wheels. Ignore this if you're on 2WD/ACKERMANN
//#define MAX_STEERING_ANGLE 0.415  // max steering angle. This only applies to Ackermann steering

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

/// ENCODER PINS
#define MOTOR1_ENCODER_A 3
#define MOTOR1_ENCODER_B 17 

#define MOTOR2_ENCODER_A 18
#define MOTOR2_ENCODER_B 16 

#define MOTOR3_ENCODER_A 19
#define MOTOR3_ENCODER_B 15 

#define MOTOR4_ENCODER_A 2
#define MOTOR4_ENCODER_B 14

//MOTOR PINS
#ifdef USE_L298_DRIVER
  #define MOTOR_DRIVER L298

  #define MOTOR1_PWM 11
  #define MOTOR1_IN_A 40
  #define MOTOR1_IN_B 42

  #define MOTOR2_PWM 12
  #define MOTOR2_IN_A 30
  #define MOTOR2_IN_B 32

  #define MOTOR3_PWM 10
  #define MOTOR3_IN_A 4
  #define MOTOR3_IN_B 5

  #define MOTOR4_PWM 9
  #define MOTOR4_IN_A 7
  #define MOTOR4_IN_B 8

  #define PWM_MAX 255
  #define PWM_MIN -PWM_MAX
#endif 


#endif
