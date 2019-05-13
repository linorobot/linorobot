#ifndef LINO_BASE_CONFIG_H
#define LINO_BASE_CONFIG_H

//uncomment the base you're building
#define LINO_BASE DIFFERENTIAL_DRIVE // 2WD and Tracked robot w/ 2 motors
// #define LINO_BASE SKID_STEER      // 4WD robot
// #define LINO_BASE ACKERMANN       // Car-like steering robot w/ 2 motors
// #define LINO_BASE ACKERMANN1      // Car-like steering robot w/ 1 motor
// #define LINO_BASE MECANUM         // Mecanum drive robot

//uncomment the motor driver you're using
//#define USE_L298_DRIVER
// #define USE_BTS7960_DRIVER
#define USE_ESC

//uncomment the IMU you're using
// #define USE_GY85_IMU
#define USE_MPU6050_IMU
// #define USE_MPU9150_IMU
// #define USE_MPU9250_IMU

#define DEBUG 1

// #define K_P 0.6 // P constant
// #define K_I 0.0 // I constant
// #define K_D 0.0 // D constant
#define K_P 0.6 // P constant
#define K_I 0.0001 // I constant
#define K_D 0.9 // D constant

//define your robot' specs here
#define MAX_RPM 2000               // motor's maximum RPM
#define COUNTS_PER_REV 900 		  // wheel encoder's no of ticks per rev (e.g motors for one rotation should be 6, then multiplz for the number of the ration e.g 1:6 is 36 counts per rev???)
#define WHEEL_DIAMETER 0.20       // wheel's diameter in meters
#define PWM_BITS 8                // PWM Resolution of the microcontroller
#define LR_WHEELS_DISTANCE 0.235  // distance between left and right wheels
#define FR_WHEELS_DISTANCE 0.30   // distance between front and rear wheels. Ignore this if you're on 2WD/ACKERMANN
#define MAX_STEERING_ANGLE 0.415  // max steering angle. This only applies to Ackermann steering

//Added for ESC only, see in motor.h
//#define NEUTRAL_BAND_WAIT_MS_PER_PWM 3 //Find by approximation, the time must spend the ESC in neutral to invert the direction, change in motor.h 

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
#ifdef USE_ESC // Using an ESC requires normally 3 Pins
#define MOTOR1_ENCODER_A 15
#define MOTOR1_ENCODER_B 14 
#define MOTOR1_ENCODER_C 20 

#define MOTOR2_ENCODER_A 10//11
#define MOTOR2_ENCODER_B 11//10 
#define MOTOR2_ENCODER_C 6 

#define MOTOR3_ENCODER_A 17
#define MOTOR3_ENCODER_B 16 
#define MOTOR3_ENCODER_C 23 

#define MOTOR4_ENCODER_A 9
#define MOTOR4_ENCODER_B 12
#define MOTOR4_ENCODER_C 3

#else
#define MOTOR1_ENCODER_A 15
#define MOTOR1_ENCODER_B 14 

#define MOTOR2_ENCODER_A 11
#define MOTOR2_ENCODER_B 12 

#define MOTOR3_ENCODER_A 17
#define MOTOR3_ENCODER_B 16 

#define MOTOR4_ENCODER_A 9
#define MOTOR4_ENCODER_B 10
#endif


//MOTOR PINS
#ifdef USE_L298_DRIVER
  #define MOTOR_DRIVER L298

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

  #define PWM_MAX pow(2, PWM_BITS) - 1
  #define PWM_MIN -PWM_MAX
#endif 

#ifdef USE_BTS7960_DRIVER
  #define MOTOR_DRIVER BTS7960  

  #define MOTOR1_PWM 1 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR1_IN_A 21
  #define MOTOR1_IN_B 20

  #define MOTOR2_PWM 8 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR2_IN_A 5
  #define MOTOR2_IN_B 6

  #define MOTOR3_PWM 0 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR3_IN_A 22
  #define MOTOR3_IN_B 23

  #define MOTOR4_PWM 2 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR4_IN_A 4
  #define MOTOR4_IN_B 3

  #define PWM_MAX pow(2, PWM_BITS) - 1
  #define PWM_MIN -PWM_MAX
#endif

#ifdef USE_ESC
  #define MOTOR_DRIVER ESC  

  #define MOTOR1_PWM 1 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR1_IN_A 21
  #define MOTOR1_IN_B 20 // Not use in case of ESC. Only for compatibility, PIN is reused for ENCODER!!

  #define MOTOR2_PWM 8 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR2_IN_A 5
  #define MOTOR2_IN_B 6 // Not use in case of ESC. Only for compatibility, PIN is reused for ENCODER!!

  #define MOTOR3_PWM 0 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR3_IN_A 22
  #define MOTOR3_IN_B 23 // Not use in case of ESC. Only for compatibility, PIN is reused for ENCODER!!

  #define MOTOR4_PWM 2 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR4_IN_A 4
  #define MOTOR4_IN_B 3 // Not use in case of ESC. Only for compatibility, PIN is reused for ENCODER!!

  #define PWM_MAX 400
  #define PWM_MIN -PWM_MAX
  #define PWM_MIN_THRESHOLD 20 	  // The minimum threshold for pwm in brushless control. An offset for the 0 to PWM MAX

#endif

#define STEERING_PIN 7

#endif
