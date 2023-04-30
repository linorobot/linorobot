
#ifndef LINO_BASE_CONFIG_H
#define LINO_BASE_CONFIG_H


#define DEBUG 1

#define K_P 1.2 // P constant
#define K_I 0.7 // I constant
#define K_D 0.5 // D constant

//uncomment the base you're building
#define LINO_BASE DIFFERENTIAL_DRIVE // 2WD and Tracked robot w/ 2 motors   
// #define LINO_BASE SKID_STEER      // 4WD robot
// #define LINO_BASE ACKERMANN       // Car-like steering robot w/ 2 motors
// #define LINO_BASE ACKERMANN1      // Car-like steering robot w/ 1 motor
// #define LINO_BASE MECANUM         // Mecanum drive robot

//uncomment the motor driver you're using
#define USE_L298_DRIVER
// #define USE_BTS7960_DRIVER
// #define USE_ESCS

//uncomment the IMU you're using
// #define USE_GY85_IMU
#define USE_MPU6050_IMU
// #define USE_MPU9150_IMU
// #define USE_MPU9250_IMU



//define your robot' specs here
#define MAX_RPM 170               // motor's maximum RPM
#define COUNTS_PER_REV 1520       // wheel encoder's no of ticks per rev
#define WHEEL_DIAMETER 0.065       // wheel's diameter in meters
#define PWM_BITS 8                // PWM Resolution of the microcontroller
#define LR_WHEELS_DISTANCE 0.262  // distance between left and right wheels
#define FR_WHEELS_DISTANCE 0.00   // distance between front and rear wheels. Ignore this if you're on 2WD/ACKERMANN//k
//#define MAX_STEERING_ANGLE 0.415  // max steering angle. This only applies to Ackermann steering


/// ENCODER PINS
// motor 1
#define MOTOR1_ENCODER_A 18
#define MOTOR1_ENCODER_B 19

// motor 2
#define MOTOR2_ENCODER_A 2
#define MOTOR2_ENCODER_B 3 

 // motor 3
//#define MOTOR3_ENCODER_A 19
//#define MOTOR3_ENCODER_B 18
//
//// motor 4
//#define MOTOR4_ENCODER_A 19
//#define MOTOR4_ENCODER_B 18

//MOTOR PINS
#ifdef USE_L298_DRIVER
  #define MOTOR_DRIVER L298

  #define MOTOR1_PWM 10
  #define MOTOR1_IN_A 4
  #define MOTOR1_IN_B 5

  #define MOTOR2_PWM 9
  #define MOTOR2_IN_A 7
  #define MOTOR2_IN_B 8

//// motor 3
//  #define MOTOR3_PWM 8
//  #define MOTOR3_IN_A 24
//  #define MOTOR3_IN_B 25

// motor 4
//  #define MOTOR4_PWM 8
//  #define MOTOR4_IN_A 24
//  #define MOTOR4_IN_B 25

  #define PWM_MAX 255
  #define PWM_MIN -PWM_MAX
#endif 


#endif
