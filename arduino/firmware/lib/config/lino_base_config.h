#ifndef LINO_BASE_CONFIG_H
#define LINO_BASE_CONFIG_H

#define L298_DRIVER
// #define BTS7960_DRIVER

#define DEBUG 0

/*BTS7960 PID
#define K_P 0.05 // P constant
#define K_I 0.5 // I constant
#define K_D 0.1 // D constant
*/

#define K_P 0.6 // P constant
#define K_I 0.3 // I constant
#define K_D 0.5 // D constant

// define your robot' specs here
#define MAX_RPM 45 // motor's maximum RPM
#define COUNTS_PER_REV 4000 // wheel encoder's no of ticks per rev
#define WHEEL_DIAMETER 0.15 // wheel's diameter in meters
#define PWM_BITS 8 // PWM Resolution of the microcontroller
#define BASE_WIDTH 0.43 // width of the plate you are using

// ENCODER PINS
// left side encoders pins
#define MOTOR1_ENCODER_A 15 // front_A
#define MOTOR1_ENCODER_B 14 // front_B

// right side encoders pins
#define MOTOR2_ENCODER_A 12 // front_A
#define MOTOR2_ENCODER_B 11 // front_B

#ifdef L298_DRIVER
  //left side motor pins
  #define MOTOR1_PWM 21
  #define MOTOR1_IN_A 20
  #define MOTOR1_IN_B 1

  //right side motor pins
  #define MOTOR2_PWM 5
  #define MOTOR2_IN_A 8
  #define MOTOR2_IN_B 6
#endif

#ifdef BTS7960_DRIVER
  // left side motor pins
  #define MOTOR1_IN_A 22
  #define MOTOR1_IN_B 23

  // right side motor pins
  #define MOTOR2_IN_A 6
  #define MOTOR2_IN_B 5
#endif

#endif
