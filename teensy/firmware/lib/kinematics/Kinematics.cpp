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

Kinematics::Kinematics(base base_platform, int motor_max_rpm, float wheel_diameter, 
float wheels_x_distance, float wheels_y_distance, int pwm_bits):
    base_platform_(base_platform),
    max_rpm_(motor_max_rpm),
    wheel_diameter_(wheel_diameter),
    wheels_x_distance_(base_platform_ == DIFFERENTIAL_DRIVE ? 0 : wheels_x_distance),
    wheels_y_distance_(wheels_y_distance),
    pwm_res_(pow(2, pwm_bits) - 1),
    circumference_(PI * wheel_diameter_)
{    
}

Kinematics::rpm Kinematics::calculateRPM(float linear_x, float linear_y, float angular_z)
{
    float linear_vel_x_mins;
    float linear_vel_y_mins;
    float angular_vel_z_mins;
    float tangential_vel;
    float x_rpm;
    float y_rpm;
    float tan_rpm;

    //convert m/s to m/min
    linear_vel_x_mins = linear_x * 60;
    linear_vel_y_mins = linear_y * 60;

    //convert rad/s to rad/min
    angular_vel_z_mins = angular_z * 60;

    tangential_vel = angular_vel_z_mins * ((wheels_x_distance_ / 2) + (wheels_y_distance_ / 2));

    x_rpm = linear_vel_x_mins / circumference_;
    y_rpm = linear_vel_y_mins / circumference_;
    tan_rpm = tangential_vel / circumference_;

    Kinematics::rpm rpm;

    //calculate for the target motor RPM and direction
    //front-left motor
    rpm.motor1 = x_rpm - y_rpm - tan_rpm;
    rpm.motor1 = constrain(rpm.motor1, -max_rpm_, max_rpm_);

    //front-right motor
    rpm.motor2 = x_rpm + y_rpm + tan_rpm;
    rpm.motor2 = constrain(rpm.motor2, -max_rpm_, max_rpm_);

    //rear-left motor
    rpm.motor3 = x_rpm + y_rpm - tan_rpm;
    rpm.motor3 = constrain(rpm.motor3, -max_rpm_, max_rpm_);

    //rear-right motor
    rpm.motor4 = x_rpm - y_rpm + tan_rpm;
    rpm.motor4 = constrain(rpm.motor4, -max_rpm_, max_rpm_);

    return rpm;
}

Kinematics::rpm Kinematics::getRPM(float linear_x, float linear_y, float angular_z)
{
    Kinematics::rpm rpm;

    if(base_platform_ == DIFFERENTIAL_DRIVE || base_platform_ == SKID_STEER)
    {
        rpm = calculateRPM(linear_x, 0.0 , angular_z);
    }
    else if(base_platform_ == ACKERMANN)
    {
        rpm = calculateRPM(linear_x, 0.0, 0.0);
    }
    else if(base_platform_ == MECANUM)
    {
        rpm = calculateRPM(linear_x, linear_y, angular_z);
    }

    return rpm;
}

Kinematics::pwm Kinematics::calculatePWM(float linear_x, float linear_y, float angular_z)
{
    Kinematics::rpm rpm;
    Kinematics::pwm pwm;

    rpm = calculateRPM(linear_x, linear_y, angular_z);

    //convert from RPM to PWM
    //front-left motor
    pwm.motor1 = rpmToPWM(rpm.motor1);
    pwm.motor1 = constrain(pwm.motor1, -pwm_res_, pwm_res_);

    //rear-left motor
    pwm.motor2 = rpmToPWM(rpm.motor2);
    pwm.motor2 = constrain(pwm.motor2, -pwm_res_, pwm_res_);

    //front-right motor
    pwm.motor3 = rpmToPWM(rpm.motor3);
    pwm.motor3 = constrain(pwm.motor3, -pwm_res_, pwm_res_);

    //rear-right motor
    pwm.motor4 = rpmToPWM(rpm.motor4);
    pwm.motor4 = constrain(pwm.motor4, -pwm_res_, pwm_res_);

    return pwm;
}

Kinematics::pwm Kinematics::getPWM(float linear_x, float linear_y, float angular_z)
{
    Kinematics::pwm pwm;

    if(base_platform_ == DIFFERENTIAL_DRIVE || base_platform_ == SKID_STEER)
    {
        pwm = calculatePWM(linear_x, 0.0 , angular_z);
    }
    else if(base_platform_ == ACKERMANN)
    {
        pwm = calculatePWM(linear_x, 0.0, 0.0);
    }
    else if(base_platform_ == MECANUM)
    {
        pwm = calculatePWM(linear_x, linear_y, angular_z);
    }

    return pwm;
}

Kinematics::velocities Kinematics::calculateVelocities(int motor1, int motor2)
{
    Kinematics::velocities vel;

    float average_rpm_x = (motor1 + motor2) / 2; // RPM
    //convert revolutions per minute to revolutions per second
    float average_rps_x = average_rpm_x / 60; // RPS
    vel.linear_x = average_rps_x * circumference_; // m/s

    float average_rpm_a = (motor2 - motor1) / 2;
    //convert revolutions per minute to revolutions per second
    float average_rps_a = average_rpm_a / 60;
    vel.angular_z =  (average_rps_a * circumference_) / (wheels_y_distance_ / 2);

    vel.linear_y = 0.0;

    return vel;
}

Kinematics::velocities Kinematics::calculateVelocities(int motor1, int motor2, int motor3, int motor4)
{
    Kinematics::velocities vel;

    float average_rpm_x = (motor1 + motor2 + motor3 + motor4) / 4; // RPM
    //convert revolutions per minute to revolutions per second
    float average_rps_x = average_rpm_x / 60; // RPS

    vel.linear_x = average_rps_x * circumference_; // m/s

    float average_rpm_y = (-motor1 + motor2 + motor3 - motor4) / 4; // RPM
    //convert revolutions per minute in y axis to revolutions per second
    float average_rps_y = average_rpm_y / 60; // RPS

    vel.linear_y = average_rps_y * circumference_; // m/s

    float average_rpm_a = (-motor1 + motor2 - motor3 + motor4) / 4;
    //convert revolutions per minute to revolutions per second
    float average_rps_a = average_rpm_a / 60;

    vel.angular_z =  (average_rps_a * circumference_) / ((wheels_x_distance_ / 2) + (wheels_y_distance_ / 2)); //  rad/s

    return vel;
}

Kinematics::velocities Kinematics::getVelocities(int motor1, int motor2, int motor3, int motor4)
{
    Kinematics::velocities vel;

    if(base_platform_ == DIFFERENTIAL_DRIVE || base_platform_ == ACKERMANN)
    {   
        vel = calculateVelocities(motor1, motor2);
    }
    else if(base_platform_ == SKID_STEER || base_platform_ == MECANUM)
    {
        vel = calculateVelocities(motor1, motor2, motor3, motor4);
    }

    return vel;
}

int Kinematics::rpmToPWM(int rpm)
{
    //remap scale of target RPM vs MAX_RPM to PWM
    return (((double) rpm / (double) max_rpm_) * pwm_res_);
}