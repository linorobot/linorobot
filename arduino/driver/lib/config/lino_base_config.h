#ifndef LINO_BASE_CONFIG_H
#define LINO_BASE_CONFIG_H

#define DEBUG 0

#define k_p 0.4 // P constant
#define k_i 0.0 // I constant
#define k_d 1.0 // D constant

#define pi 3.1415926 
#define two_pi 6.2831853

//define your motors' specs here
#define encoder_pulse 55 //encoder's number of ticks per revolution 
#define gear_ratio 30 //motor's gear ratio
#define max_rpm 330 //motor's maximum RPM

//define your robot base's specs here
#define wheel_diameter 0.069 //wheel's diameter in meters
#define wheel_width 0.027 //wheel's width in meters
#define track_width 0.22 // width of the plate you are using

//don't change this if you followed the schematic diagram
//ENCODER PINS
#define left_encoder_a 7 
#define left_encoder_b 6  
#define right_encoder_a 8  
#define right_encoder_b 9 

//don't change this if you followed the schematic diagram
//MOTOR PINS
//Left Motor
#define left_motor_direction 20
#define left_motor_pwm 21

//Right Motor
#define right_motor_direction 22
#define right_motor_pwm 23

#endif