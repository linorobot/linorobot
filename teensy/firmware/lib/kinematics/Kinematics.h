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

#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "Arduino.h"

class Kinematics
{
    public:
        enum base {DIFFERENTIAL_DRIVE, SKID_STEER, ACKERMANN, MECANUM};

        struct output
        {
            int motor1;
            int motor2;
            int motor3;
            int motor4;
        };
        struct velocities
        {
            float linear_x;
            float linear_y;
            float angular_z;
        };

        Kinematics(base base_platform, int motor_max_rpm, float wheel_diameter, float base_width, int pwm_bits);
        velocities getVelocities(int motor1, int motor2, int motor3, int motor4);
        output getRPM(float linear_x, float linear_y, float angular_z);
        output getPWM(float linear_x, float linear_y, float angular_z);


    private:
        base base_platform_;
        velocities calculateVelocities(int motor1, int motor2);
        velocities calculateVelocities(int motor1, int motor2, int motor3, int motor4);
        output calculateRPM(float linear_x, float linear_y, float angular_z);
        output calculatePWM(float linear_x, float linear_y, float angular_z);
        int rpmToPWM(int rpm);
        
        float linear_vel_x_mins_;
        float linear_vel_y_mins_;
        float angular_vel_z_mins_;
        float circumference_;
        float tangential_vel_;
        float x_rpm_;
        float y_rpm_;
        float tan_rpm_;
        int max_rpm_;
        double wheel_diameter_;
        float base_width_;
        double pwm_res_;
};

#endif