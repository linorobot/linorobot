#ifndef LINO_BASE_CONFIG_H
#define LINO_BASE_CONFIG_H

//uncomment the base you're building
#define BASE_2WD
// #define BASE_4WD
// #define BASE_ACKERMANN
// #define BASE_MECANUM

//uncomment the IMU you're using
#define GY85_IMU
// #define MP6050_IMU

//uncomment the motor driver you're using
#define L298_DRIVER
// #define BTS7960_DRIVER

#define DEBUG 0

#define K_P 0.6 // P constant
#define K_I 0.3 // I constant
#define K_D 0.5 // D constant

/*BTS7960 PID
#define K_P 0.05 // P constant
#define K_I 0.9 // I constant
#define K_D 0.1 // D constant
*/

// define your robot' specs here
#define MAX_RPM 45 // motor's maximum RPM
#define COUNTS_PER_REV 4000 // wheel encoder's no of ticks per rev
#define WHEEL_DIAMETER 0.15 // wheel's diameter in meters
#define PWM_BITS 8 // PWM Resolution of the microcontroller
#define BASE_WIDTH 0.32 // width of the plate you are using

/*
ROBOT ORIENTATION
         FRONT
    MOTOR1  MOTOR2  (2WD/ACKERMANN)
    MOTOR3  MOTOR4  (4WD/MECANUM)  
         BACK
*/

/// ENCODER PINS
#define MOTOR1_ENCODER_A 15 
#define MOTOR1_ENCODER_B 14 

#define MOTOR2_ENCODER_A 12 
#define MOTOR2_ENCODER_B 11 

#define MOTOR3_ENCODER_A 17 
#define MOTOR3_ENCODER_B 16 

#define MOTOR4_ENCODER_A 10 
#define MOTOR4_ENCODER_B 9 

//MOTOR PINS
#ifdef L298_DRIVER
  #define MOTOR1_PWM 21
  #define MOTOR1_IN_A 20
  #define MOTOR1_IN_B 1

  #define MOTOR2_PWM 5
  #define MOTOR2_IN_A 8
  #define MOTOR2_IN_B 6

  #define MOTOR3_PWM 22
  #define MOTOR3_IN_A 23
  #define MOTOR3_IN_B 0

  #define MOTOR4_PWM 4
  #define MOTOR4_IN_A 2
  #define MOTOR4_IN_B 3
#endif

#ifdef BTS7960_DRIVER
  #define MOTOR1_IN_A 21
  #define MOTOR1_IN_B 20

  #define MOTOR2_IN_A 5
  #define MOTOR2_IN_B 6

  #define MOTOR3_IN_A 22
  #define MOTOR3_IN_B 23

  #define MOTOR4_IN_A 4
  #define MOTOR4_IN_B 3
#endif

#ifdef BASE_ACKERMANN
    #define STEERING_PIN 7
#endif

#endif