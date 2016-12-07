#ifndef LINO_BASE_CONFIG_H
#define LINO_BASE_CONFIG_H

#define DEBUG 0

const float K_P = 0.05; // P constant
const float K_I = 0.9;  // I constant
const float K_D = 0.1; // D constant

// define your motors' specs here

const int MAX_RPM = 45;            // motor's maximum RPM
const int COUNTS_PER_REV = 4000;   // wheel encoder's no of ticks per rev
const float WHEEL_DIAMETER = 0.15; // wheel's diameter in meters

#define BASE_WIDTH 0.43 // width of the plate you are using

// ENCODER PINS
// left side encoders pins
#define MOTOR1_ENCODER_A 17 // front_A
#define MOTOR1_ENCODER_B 16 // front_B

// right side encoders pins
#define MOTOR2_ENCODER_A 12 // front_A
#define MOTOR2_ENCODER_B 11 // front_B

// MOTOR PINS
// left side motor pins
#define MOTOR1_IN_A 22
#define MOTOR1_IN_B 23

// right side motor pins
#define MOTOR2_IN_A 6
#define MOTOR2_IN_B 5

#endif
