#ifndef LINO_BASE_CONFIG_H
#define LINO_BASE_CONFIG_H

#define DEBUG 0

float K_P = 0.4; // P constant
float K_I = 0.0; // I constant
float K_D = 1.0; // D constant

//define your motors' specs here

const int MAX_RPM = 330; //motor's maximum RPM
const int COUNTS_PER_REV = 1650; //wheel encoder's no of ticks per rev(gear_ratio * pulse_per_rev)
const float WHEEL_DIAMETER = 0.069; //wheel's diameter in meters

#define TRACK_WIDTH 0.22 // width of the plate you are using

//ENCODER PINS
// left side encoders pins
#define MOTOR1_ENCODER_A 3 //front_A
#define MOTOR1_ENCODER_B 2 //front_B

// right side encoders pins
#define MOTOR2_ENCODER_A 6 //front_A
#define MOTOR2_ENCODER_B 7 //front_B

//don't change this if you followed the schematic diagram
//MOTOR PINS
//left side motor pins
#define MOTOR1_PWM 23
#define MOTOR1_IN_A 17
#define MOTOR1_IN_B 16

//right side motor pins
#define MOTOR2_PWM 21
#define MOTOR2_IN_A 13
#define MOTOR2_IN_B 12

#endif
