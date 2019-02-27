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

#define RPM_TO_RPS 1/60

class Kinematics
{
    public:
        enum base {DIFFERENTIAL_DRIVE, SKID_STEER, ACKERMANN, ACKERMANN1, MECANUM};

        base base_platform;

        struct rpm
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

        struct pwm
        {
            int motor1;
            int motor2;
            int motor3;
            int motor4;
        };

        Kinematics(base robot_base, int motor_max_rpm, float wheel_diameter, float wheels_x_distance, float wheels_y_distance);
        velocities getVelocities(float steering_angle, int rpm1, int rpm2);
        velocities getVelocities(int rpm1, int rpm2, int rpm3, int rpm4);
        rpm getRPM(float linear_x, float linear_y, float angular_z);

    private:
        rpm calculateRPM(float linear_x, float linear_y, float angular_z);
        int getTotalWheels(base robot_base);

        int max_rpm_;
        float wheels_x_distance_;
        float wheels_y_distance_;
        float pwm_res_;
        float wheel_circumference_;
        int total_wheels_;
};

#endif