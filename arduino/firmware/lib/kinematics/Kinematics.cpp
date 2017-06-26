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

#include "Arduino.h"
#include "Kinematics.h"

Kinematics::Kinematics(int motor_max_rpm, float wheel_diameter, float base_width, int pwm_bits)
{
  wheel_diameter_ = wheel_diameter;
  circumference_ = PI * wheel_diameter_;
  max_rpm_ = motor_max_rpm;
  base_width_ = base_width;
  pwm_res_ = pow(2, pwm_bits) - 1;
}

Kinematics::output Kinematics::getRPM(float linear_x, float linear_y, float angular_z)
{
  //convert m/s to m/min
  linear_vel_x_mins_ = linear_x * 60;
  linear_vel_y_mins_ = linear_y * 60;

  //convert rad/s to rad/min
  angular_vel_z_mins_ = angular_z * 60;

  //Vt = ω * radius
  tangential_vel_ = angular_vel_z_mins_ * base_width_;

  x_rpm_ = linear_vel_x_mins_ / circumference_;
  y_rpm_ = linear_vel_y_mins_ / circumference_;
  tan_rpm_ = tangential_vel_ / circumference_;

  Kinematics::output rpm;

  //calculate for the target motor RPM and direction
  //front-left motor
  rpm.motor1 = x_rpm_ - y_rpm_ - tan_rpm_;
  //rear-left motor
  rpm.motor3 = x_rpm_ + y_rpm_ - tan_rpm_;

  //front-right motor
  rpm.motor2 = x_rpm_ + y_rpm_ + tan_rpm_;
  //rear-right motor
  rpm.motor4 = x_rpm_ - y_rpm_ + tan_rpm_;

  return rpm;
}

Kinematics::output Kinematics::getPWM(float linear_x, float linear_y, float angular_z)
{
  Kinematics::output rpm;
  Kinematics::output pwm;

  rpm = getRPM(linear_x, linear_y, angular_z);

  //convert from RPM to PWM
  //front-left motor
  pwm.motor1 = rpmToPWM(rpm.motor1);
  //rear-left motor
  pwm.motor2 = rpmToPWM(rpm.motor2);

  //front-right motor
  pwm.motor3 = rpmToPWM(rpm.motor3);
  //rear-right motor
  pwm.motor4 = rpmToPWM(rpm.motor4);

  return pwm;
}

Kinematics::velocities Kinematics::getVelocities(int motor1, int motor2)
{
  Kinematics::velocities vel;

  double average_rpm_x = (motor1 + motor2) / 2; // RPM
  //convert revolutions per minute to revolutions per second
  double average_rps_x = average_rpm_x / 60; // RPS
  vel.linear_x = (average_rps_x * (wheel_diameter_ * PI)); // m/s

  double average_rpm_a = (motor2 - motor1) / 2;
  //convert revolutions per minute to revolutions per second
  double average_rps_a = average_rpm_a / 60;
  vel.angular_z =  (average_rps_a * (wheel_diameter_ * PI)) / base_width_;
  vel.linear_y = 0.0;
  return vel;
}

Kinematics::velocities Kinematics::getVelocities(int motor1, int motor2, int motor3, int motor4)
{
  Kinematics::velocities vel;

  double average_rpm_x = (motor1 + motor2 + motor3 + motor4) / 4; // RPM
  //convert revolutions per minute to revolutions per second
  double average_rps_x = average_rpm_x / 60; // RPS
  vel.linear_x = (average_rps_x * (wheel_diameter_ * PI)); // m/s

  double average_rpm_y = (-motor1 + motor2 + motor3 - motor4) / 4; // RPM
  //convert revolutions per minute in y axis to revolutions per second
  double average_rps_y = average_rpm_y / 60; // RPS
  vel.linear_y = (average_rps_y * (wheel_diameter_ * PI)); // m/s

  double average_rpm_a = (-motor1 + motor2 - motor3 + motor4) / 4;
  //convert revolutions per minute to revolutions per second
  double average_rps_a = average_rpm_a / 60;
  vel.angular_z =  (average_rps_a * (wheel_diameter_ * PI)) / base_width_;

  return vel;
}

int Kinematics::rpmToPWM(int rpm)
{
  //remap scale of target RPM vs MAX_RPM to PWM
 return (((double) rpm / (double) max_rpm_) * 255);
}
