/*
  Copyright (c) 2016, Juan Jimeno
  Source: http://research.ijcaonline.org/volume113/number3/pxc3901586.pdf
  All rights reserved.
  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
   Redistributions of source code must retain the above copyright notice,
  this list of conditions and the following disclaimer.
   Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in the
  documentation and/or other materials provided with the distribution.
   Neither the name of  nor the names of its contributors may be used to
  endorse or promote products derived from this software without specific
  prior written permission.
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORTPPIPI (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
*/

#include "Kinematics.h"

/*Kinematics(int motor_max_rpm, float wheel_diameter, float base_width, int pwm_bits)
   motor_max_rpm = motor's maximum rpm
   wheel_diameter = robot's wheel diameter expressed in meters
   base_width = distance between two wheels expressed in meters
   pwm_bits = microcontroller's PWM pin resolution. Arduino Uno/Mega Teensy is using 8 bits(0-255)
*/
Kinematics kinematics(90, 0.2, 0.5, 8);

void setup()
{
  Serial.begin(9600);
}

void loop()
{
  Kinematics::output rpm;

  /*kinematics.getRPM(linear_x, linear_y, angular_z);
    linear_x = target linear velocity in x axis (right hand rule)
    linear_y = target linear velocity in y axis (right hand rule)
    angular_z = target angular velocity in z axis (right hand rule)
  */
  //target velocities
  float linear_vel_x = 1;
  float linear_vel_y = 0;
  float angular_vel_z = 1;
  rpm = kinematics.getRPM(linear_vel_x, linear_vel_y, angular_vel_z);

  Serial.print(" FRONT LEFT MOTOR: ");
  Serial.print(rpm.motor1);

  Serial.print(" FRONT RIGHT MOTOR: ");
  Serial.print(rpm.motor2);

  Serial.print(" REAR LEFT MOTOR: ");
  Serial.print(rpm.motor3);

  Serial.print(" REAR RIGHT MOTOR: ");
  Serial.println(rpm.motor4);

  delay(5000);


  int motor1_feedback = rpm.motor1;//in rpm
  int motor2_feedback = rpm.motor2; //in rpm
  int motor3_feedback = rpm.motor3; //in rpm
  int motor4_feedback = rpm.motor4; //in rpm

  Kinematics::velocities vel;
  vel = kinematics.getVelocities(motor1_feedback, motor2_feedback, motor3_feedback, motor4_feedback);
  Serial.print(" VEL X: ");
  Serial.print(vel.linear_x, 4);

  Serial.print(" VEL_Y: ");
  Serial.print(vel.linear_y, 4);

  Serial.print(" ANGULAR_Z: ");
  Serial.println(vel.angular_z, 4);
  Serial.println("");
}
